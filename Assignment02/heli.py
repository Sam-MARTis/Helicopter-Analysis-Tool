from typing import Optional, Tuple, Dict, List, Union
import numpy as np
from time import sleep
from numba import njit
import matplotlib.pyplot as plt

MAX_ITERATIONS = 100
ROTOR_DIVISIONS_01 = 100
ITERATION_OMEGA_CONVERGENCE = 100
ITERATION_POWER_CONVERGENCE = 100
TOLERANCE = 1e-6 #Convergence tolerance
TOLERANCE2 = 1e-8 # Divide by zero tolerance
# f_MAX_CAP = 50

MACH_CAP = 0.8

ITERATION_RELAXATION = 0.9

a_DEFAULT = 5.75  
F_DEFAULT = 1.1
k_DEFAULT = 0.037807
CD0 = 0.0113


@njit(cache=True)
def solve_iteratively_lambda_inflow_optimized(r: float, rmax: float, rmin:float, b:int, rc:float, tc:float, θ: float, Ω: float, Vy: float, itermax: int = MAX_ITERATIONS, tol: float = TOLERANCE):
    λᵪ = Vy / (Ω * rmax)
    c = rc - (rc - tc) * (r - rmax) / (rmax - rmin)
    σ = (b * c) / (np.pi * rmax)
    a = a_DEFAULT
    F = F_DEFAULT
    λ = λᵪ
    
    for _ in range(itermax):
        if F <= TOLERANCE2:
            F = TOLERANCE2

        λₙ = np.sqrt(np.square((σ * a) / (16 * F) - (λᵪ * 0.5)) + ((σ * a) / (8 * F)) * θ * (r / rmax)) - (σ*a/(16*F) - λᵪ/2)
        
        if λₙ < TOLERANCE2:
            λₙ = TOLERANCE2

        f = (b*0.5)  * (1-r/rmax)/λₙ

        # assert F_MAX_CAP>f>TOLERANCE2
        Fₙ = (2/np.pi) * np.arccos(np.exp(-f))
        dλ = abs(λₙ - λ)
        dF = abs(Fₙ - F)
        if dλ < tol and dF < tol:
            return λₙ, Fₙ


        λ = (1-ITERATION_RELAXATION) * λ + ITERATION_RELAXATION * λₙ
        F = (1-ITERATION_RELAXATION) * F + ITERATION_RELAXATION * Fₙ


    return λ, F

@njit(cache=True)
def get_Cl_Cd_from_λ(r: float, rmax:float,rmin:float, rp:float, sp:float, λ: float, Ω: float, sound_speed:float, a: float = a_DEFAULT, k: float = k_DEFAULT) -> Tuple[float, float]:
    pitch = rp + sp * (r - rmin)
    φ: float = np.arctan(λ * rmax / r)
    
    effective_α = pitch - φ
    Mach_number = (Ω * r) / sound_speed
    if Mach_number > MACH_CAP:
        Mach_number = MACH_CAP
    compressibility_factor = 1 / np.sqrt(1 - Mach_number**2)
    Cl = a * (effective_α) * compressibility_factor

    Cd = CD0 + k * Cl**2
    return Cl, Cd

@njit(cache=True, parallel=True)
def get_rotor_outputs(rmin: float, rmax: float, b: int, rp:float, sp: float, rc:float, tc:float,  Vy: float, Ω: float, sound_speed: float, density:float, divisions: int = ROTOR_DIVISIONS_01) -> np.ndarray:

    dT_arr = np.zeros(divisions)
    dQ_arr = np.zeros(divisions)
    dP_arr = np.zeros(divisions)
    values = np.zeros(3)
    # dFx_arr = np.zeros(divisions)
    r = np.linspace(rmin, rmax, divisions)
    dr = (rmax - rmin) / (divisions)
    
    for i in range(divisions):
        chord = rc - (rc - tc) * (r[i] - rmin) / (rmax - rmin)
        plain_pitch = float(rp + sp * (1.0 * r[i] - rmin))
        [λ, F] = solve_iteratively_lambda_inflow_optimized(r=r[i], rmax=rmax, rmin=rmin, b=b, rc=rc, tc=tc, θ=plain_pitch, Ω=Ω, Vy=Vy)
        Cl, Cd = get_Cl_Cd_from_λ(r=r[i], rmax=rmax, rmin=rmin, rp=rp, sp=sp, λ=λ, Ω=Ω, sound_speed=sound_speed)
        effective_α = plain_pitch- np.arctan(λ * rmax / r[i])

        Vsq = (Ω * r[i])**2 + np.square(λ * Ω * rmax)
        dD = 0.5 * density * (b * chord) * Cd * Vsq * F
        dL = 0.5 * density * (b * chord) * Cl * Vsq * F
        dT = (dL *np.cos(effective_α) - dD * np.sin(effective_α)) * dr
        dFx = (dL *np.sin(effective_α) + dD * np.cos(effective_α)) * dr
        dQ = (dFx * r[i]) * dr
        dP = Ω * dQ
        
        dT_arr[i] = dT
        dQ_arr[i] = dQ
        dP_arr[i] = dP
    
    values[0] = np.sum(dT_arr)
    values[1] = np.sum(dQ_arr)
    values[2] = np.sum(dP_arr)
    
    return values

@njit(parallel=True)
def calculate_effective_aoa_vs_r(rmin: float, rmax: float, b: int, rc: float, tc: float, rp: float, sp: float, Vy: float, Ω: float, sound_speed: float, divisions: int = ROTOR_DIVISIONS_01):
    r = np.linspace(rmin, rmax, divisions)
    effective_α_arr = np.zeros(divisions)
    λ_arr = np.zeros(divisions)
    for i in range(divisions):
        θ = rp + sp * (r[i] - rmin)
        λ, F = solve_iteratively_lambda_inflow_optimized(r=r[i], rmax=rmax, rmin=rmin, b=b, rc=rc, tc=tc, θ=θ, Vy=Vy, Ω=Ω)
        λ_arr[i] = λ
        effective_α_arr[i] = θ - np.arctan(λ * rmax / r[i])
    return r, effective_α_arr

class Environment:
    def __init__(self) -> None:
        self.environment_set: bool = False
        self.temperature_sea_level: Optional[float] = None
        self.Reynolds_number: Optional[float] = None
        self.pressure_sea_level: Optional[float] = None
        self.Lapse_rate_troposphere: Optional[float] = None
        self.Specific_gas_constant: float = 287.053  
        self.gravitational_acceleration: float = 9.80665 
        self.specific_heat_constant: float = 1.4     
        self.wind_velocity: Optional[float] = None
        self.ISA_OFFSET: Optional[float] = None

    
    def set_atmosphere_parameters(self, temperature_sea_level: float, Reynolds_number: float,
                                pressure_sea_level: float,
                                Lapse_rate_troposphere: float, wind_velocity: float,
                                ISA_OFFSET: float) -> None:
        self.environment_set = True
        self.temperature_sea_level = temperature_sea_level
        self.Reynolds_number = Reynolds_number
        self.pressure_sea_level = pressure_sea_level
        self.Lapse_rate_troposphere = Lapse_rate_troposphere
        self.wind_velocity = wind_velocity
        self.ISA_OFFSET = ISA_OFFSET
        
 
    
    def get_density(self, altitude:float) -> float:
        assert self.environment_set, "Environment parameters not set. Please set them using 'set_atmosphere_parameters' method."

        temperature = self.temperature_sea_level + self.ISA_OFFSET - self.Lapse_rate_troposphere * altitude
        pressure = self.pressure_sea_level *  np.pow((temperature / (self.temperature_sea_level + self.ISA_OFFSET)), (-self.gravitational_acceleration / (self.Lapse_rate_troposphere * self.Specific_gas_constant)))
        density = pressure / (self.Specific_gas_constant * temperature)
        return density
    
    def get_speed_of_sound(self, altitude:float) -> float:
        assert self.environment_set, "Environment parameters not set. Please set them using 'set_atmosphere_parameters' method."
        temperature = self.temperature_sea_level + self.ISA_OFFSET - self.Lapse_rate_troposphere * altitude
        speed_of_sound = np.sqrt(self.specific_heat_constant * self.Specific_gas_constant * temperature)
        return speed_of_sound
    
    
class Rotor:
    def __init__(self, environment: Environment) -> None:
        self.environment = environment
        self.parameters_set: bool = False
        self.number_of_blades: Optional[int] = None
        self.radius_of_rotors: Optional[float] = None
        self.root_cutout: Optional[float] = None
        self.root_chord: Optional[float] = None
        self.tip_chord: Optional[float] = None
        self.root_pitch: Optional[float] = None
        self.slope_pitch: Optional[float] = None
        self.default_airfoil:bool = True
        
    def set_rotor_parameters(self, number_of_blades: int, radius_of_rotors: float,
                            root_cutout: float, root_chord: float,
                            tip_chord: float, root_pitch: float,
                            slope_pitch: float, default_airfoil: bool = True) -> None:
        self.parameters_set = True
        self.number_of_blades = number_of_blades
        self.radius_of_rotors = radius_of_rotors
        self.root_cutout = root_cutout
        self.root_chord = root_chord
        self.tip_chord = tip_chord
        self.root_pitch = root_pitch
        self.slope_pitch = slope_pitch
        self.default_airfoil = default_airfoil
        
    def get_chord_length(self, r: float) -> float:
        assert self.parameters_set, "Rotor parameters not set. Please set them using 'set_rotor_parameters' method."
        return self.root_chord - (self.root_chord - self.tip_chord) * r
    
    def get_pitch_angle(self, r: float) -> float:
        """
        Enter the radius from center in meters
        """
        assert self.parameters_set, "Rotor parameters not set. Please set them using 'set_rotor_parameters' method."
        return self.root_pitch + self.slope_pitch * (r - self.root_cutout)
    
    def solve_λ_inflow(self, r: float, Vy: float, Ω: float, itermax: int = MAX_ITERATIONS, tol: float = TOLERANCE):
        return solve_iteratively_lambda_inflow_optimized(r=r, rmax=self.radius_of_rotors, rmin=self.root_cutout, rc=self.root_chord, tc=self.tip_chord, b=self.number_of_blades, θ=self.get_pitch_angle(r), Ω=Ω, Vy=Vy, itermax=itermax, tol=tol)


    def get_effective_α(self, r: float, λ: float) -> float:
        if not self.parameters_set:
            raise ValueError("Rotor parameters have not been set.")
        φ: float = np.arctan(λ * self.radius_of_rotors / r)
        θ: float = self.get_pitch_angle(r)
        return θ - φ

    def compressibility_correction(self, Ω: float, altitude: float) -> float:
        assert self.environment.environment_set, "Environment parameters not set. Please set them using 'set_atmosphere_parameters' method."
        speed_of_sound = self.environment.get_speed_of_sound(altitude)
        tip_speed = Ω * self.radius_of_rotors
        Mach_number = tip_speed / speed_of_sound
        if Mach_number > MACH_CAP:
            Mach_number = MACH_CAP
            print(f"Warning: Mach number capped at {MACH_CAP}. Actual Mach number would be {Mach_number:.3f}")
        return 1 / np.sqrt(1 - Mach_number**2)
        
    def get_Cl(self, r:float, Vy:float, λ:float, Ω: float, altitude:float, a: float = a_DEFAULT) -> float:
        assert self.parameters_set, "Rotor parameters not set. Please set them using 'set_rotor_parameters' method."
        α = self.get_pitch_angle(r)
        compressibility_factor = self.compressibility_correction(Ω, altitude)
        if self.default_airfoil:
            α_L0 = 0.0
            Cl = a * (α) * compressibility_factor
        else:
            raise NotImplementedError("Custom airfoil data handling not implemented yet.")
        
    def get_Cd(self, r:float, Vy:float, λ:float, Ω: float, altitude:float, a: float = a_DEFAULT, k: float = k_DEFAULT) -> float:
        assert self.parameters_set, "Rotor parameters not set. Please set them using 'set_rotor_parameters' method."
        α = self.get_pitch_angle(r)
        compressibility_factor = self.compressibility_correction(Ω, altitude)
        if self.default_airfoil:
            α_L0 = 0.0
            Cl = a * (α) * compressibility_factor
            Cd0 = 0.01  
            k = 0.02    
            Cd = Cd0 + k * Cl**2
        else:
            raise NotImplementedError("Custom airfoil data handling not implemented yet.")
        return Cd
    def get_rotor_performance(self, Vy, Ω: float, altitude: float, divisions: int = ROTOR_DIVISIONS_01) -> Dict[str, float]:
        assert self.parameters_set, "Rotor parameters not set. Please set them using 'set_rotor_parameters' method."
        assert self.environment.environment_set, "Environment parameters not set. Please set them using 'set_atmosphere_parameters' method."
        density = self.environment.get_density(altitude)
        sound_speed = self.environment.get_speed_of_sound(altitude)
        print("Rmin:", self.root_cutout, "Rmax:", self.radius_of_rotors, "Blades:", self.number_of_blades)
        print("rp:", np.degrees(self.root_pitch), "sp:", np.degrees(self.slope_pitch), "rc:", self.root_chord, "tc:", self.tip_chord)
        print("Vy:", Vy + self.environment.wind_velocity, "Ω:", Ω, "Altitude:", altitude, "Density:", density, "Sound Speed:", sound_speed)
        values = get_rotor_outputs(rmin=self.root_cutout, rmax=self.radius_of_rotors, b=self.number_of_blades,
                                  rp=self.root_pitch, sp=self.slope_pitch, rc=self.root_chord, tc=self.tip_chord,
                                  Vy=Vy + self.environment.wind_velocity, Ω=Ω, sound_speed=sound_speed, density=density,
                                  divisions=divisions)
        T = values[0]
        Q = values[1]
        P = values[2] 
        return {"thrust": T, "torque": Q, "power": P}
    
    def find_omega_needed_for_thrust_uncoupled(self, target_thrust:float, Vy:float, altitude: float, Ω_initial:float, Ω_step: float, Ω_max:float, tol:float= TOLERANCE, itermax: int = ITERATION_OMEGA_CONVERGENCE) -> float:
        assert self.parameters_set, "Rotor parameters not set. Please set them using 'set_rotor_parameters' method."
        assert self.environment.environment_set, "Environment parameters not set. Please set them using 'set_atmosphere_parameters' method."
        density = self.environment.get_density(altitude)
        sound_speed = self.environment.get_speed_of_sound(altitude)
        Ω = Ω_initial
        for _ in range(itermax):
            if Ω > Ω_max:
                return [False, float('inf')]
            values = get_rotor_outputs(rmin=self.root_cutout, rmax=self.radius_of_rotors, b=self.number_of_blades,
                                  rp=self.root_pitch, sp=self.slope_pitch, rc=self.root_chord, tc=self.tip_chord,
                                  Vy=Vy, Ω=Ω, sound_speed=sound_speed, density=density,
                                  divisions=ROTOR_DIVISIONS_01)
            T = values[0]
            if abs(T - target_thrust) < tol:
                return [True, Ω]
            if T < target_thrust:
                Ω += Ω_step
            else:
                Ω_step *= 0.5
                Ω -= Ω_step
        return [False, float('inf')]
    
    def max_thrust_omega_by_power(self, available_power: float, Vy: float, altitude: float, Ω_initial: float, Ω_step: float, Ω_max: float, tol: float = TOLERANCE, itermax: int = ITERATION_POWER_CONVERGENCE) :
        assert self.parameters_set, "Rotor parameters not set. Please set them using 'set_rotor_parameters' method."
        assert self.environment.environment_set, "Environment parameters not set. Please set them using 'set_atmosphere_parameters' method."
        density = self.environment.get_density(altitude)
        sound_speed = self.environment.get_speed_of_sound(altitude)
        Ω = Ω_initial
        max_thrust = 0.0
        best_Ω = 0.0
        for _ in range(itermax):
            if Ω > Ω_max:
                break
            values = get_rotor_outputs(rmin=self.root_cutout, rmax=self.radius_of_rotors, b=self.number_of_blades,
                                  rp=self.root_pitch, sp=self.slope_pitch, rc=self.root_chord, tc=self.tip_chord,
                                  Vy=Vy + self.environment.wind_velocity, Ω=Ω, sound_speed=sound_speed, density=density,
                                  divisions=ROTOR_DIVISIONS_01)
            T = values[0]
            P = values[2]
            if P > available_power:
                Ω_step *= 0.5
                Ω -= Ω_step
            else:
                if T > max_thrust:
                    max_thrust = T
                    best_Ω = Ω
                Ω += Ω_step
            if Ω_step < tol:
                break
        else:
            print("Warning: Maximum iterations reached in max_thrust_by_power without convergence.")
            return [False, 0.0, 0.0]

        return [True, max_thrust, best_Ω]
    
    

    def plot_effective_aoa_vs_r(self, Vy: float, Ω: float, altitude: float, divisions: int = ROTOR_DIVISIONS_01) -> Tuple[np.ndarray, np.ndarray]:

        assert self.parameters_set, "Rotor parameters not set. Please set them using 'set_rotor_parameters' method."
        assert self.environment.environment_set, "Environment parameters not set. Please set them using 'set_atmosphere_parameters' method."
        r, effective_α_arr = calculate_effective_aoa_vs_r(rmin=self.root_cutout, rmax=self.radius_of_rotors, b=self.number_of_blades,
                                  rc=self.root_chord, tc=self.tip_chord, rp=self.root_pitch, sp=self.slope_pitch,
                                  Vy=Vy + self.environment.wind_velocity, Ω=Ω,
                                  sound_speed=self.environment.get_speed_of_sound(altitude),
                                  divisions=divisions)
        plt.plot(r, np.degrees(effective_α_arr), label='Ω')
        r, effective_α_arr = calculate_effective_aoa_vs_r(rmin=self.root_cutout, rmax=self.radius_of_rotors, b=self.number_of_blades,
                                  rc=self.root_chord, tc=self.tip_chord, rp=self.root_pitch, sp=self.slope_pitch,
                                  Vy=Vy + self.environment.wind_velocity, Ω=Ω/3,
                                  sound_speed=self.environment.get_speed_of_sound(altitude),
                                  divisions=divisions)
        plt.plot(r, np.degrees(effective_α_arr), linestyle='--', label='Ω/3')   
        plt.xlabel("Radius (m)")
        plt.ylabel("Effective Angle of Attack (degrees)")
        plt.title("Effective Angle of Attack vs Radius")
        plt.grid()
        plt.legend()
        plt.show()
    
    def copy(self) -> 'Rotor':
        new_rotor = Rotor(self.environment)
        if self.parameters_set:
            new_rotor.set_rotor_parameters(
                number_of_blades=self.number_of_blades,
                radius_of_rotors=self.radius_of_rotors,
                root_cutout=self.root_cutout,
                root_chord=self.root_chord,
                tip_chord=self.tip_chord,
                root_pitch=self.root_pitch,
                slope_pitch=self.slope_pitch,
                default_airfoil=self.default_airfoil
            )
        return new_rotor
class Engine:
    def __init__(self, max_power: float, mass: float) -> None:
        self.max_power = max_power
        self.mass = mass
class Fuselage:
    def __init__(self) -> None:
        self.parameters_set: bool = False
        self.Cd0: Optional[float] = None
        self.f: Optional[float] = None
        self.rotor1_location: Optional[float] = None
        self.rotor2_location: Optional[float] = None
        
    def get_drag_for_speed():
        

class Helicopter:
    def __init__(self, environment: Environment) -> None:
        self.environment = environment
        self.main_rotor: Optional[Rotor] = None
        self.tail_rotor: Optional[Rotor] = None
        self.parameters_set: bool = False
        self.fuselage: Optional[float] = None
        self.cg_location: Optional[Tuple[float, float, float]] = None
        self.main_rotor_location: Optional[Tuple[float, float, float]] = None
        self.tail_rotor_location: Optional[Tuple[float, float, float]] = None
        self.inertia_tensor: Optional[np.ndarray] = None
        
    def set_helicopter_parameters(self, mass: float, cg_location: Tuple[float, float, float],
                                 main_rotor_location: Tuple[float, float, float],
                                 tail_rotor_location: Tuple[float, float, float],
                                 inertia_tensor: np.ndarray) -> None:
        self.parameters_set = True
        self.mass = mass
        self.cg_location = cg_location
        self.main_rotor_location = main_rotor_location
        self.tail_rotor_location = tail_rotor_location
        self.inertia_tensor = inertia_tensor

    
