from typing import Optional, Tuple, Dict, List, Union
import numpy as np
from time import sleep
from numba import njit

MAX_ITERATIONS = 100
TOLERANCE = 1e-6 #Convergence tolerance
TOLERANCE2 = 1e-8 # Divide by zero tolerance
F_MAX_CAP = 50

ITERATION_RELAXATION = 0.9



@njit 
def solve_iteratively_lambda_inflow_optimized(r: float, rmax: float, b:float,  θ: float, Ω: float, Vy: float, itermax: int = MAX_ITERATIONS, tol: float = TOLERANCE) -> float:
    λᵪ = Vy / (Ω * rmax)
    σ = (b * r) / (np.pi * rmax)
    a = 5.75
    F = 1.1
    λ = λᵪ
    
    for _ in range(itermax):
        if F <= TOLERANCE2:
            F = TOLERANCE2

        λₙ = np.sqrt(np.square((σ * a) / (16 * F) - (λᵪ * 0.5)) + ((σ * a) / (8 * F)) * θ * (r / rmax))
        
        if λₙ < TOLERANCE2:
            λₙ = TOLERANCE2

        f = (b*0.5)  * (1-r/rmax)/λₙ

        assert F_MAX_CAP>f>TOLERANCE2
        Fₙ = (2/np.pi) * np.arccos(np.exp(-f))
        dλ = abs(λₙ - λ)
        dF = abs(Fₙ - F)
        if dλ < tol and dF < tol:
            return λₙ, Fₙ


        λ += ITERATION_RELAXATION * dλ
        F += ITERATION_RELAXATION * dF

    return λ, F

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
        self.NACA_for_airfoil: Optional[str] = None
        
    def set_rotor_parameters(self, number_of_blades: int, radius_of_rotors: float,
                            root_cutout: float, root_chord: float,
                            tip_chord: float, root_pitch: float,
                            slope_pitch: float, default_airfoil: bool = True,
                            NACA_for_airfoil: Optional[str] = None) -> None:
        self.parameters_set = True
        self.number_of_blades = number_of_blades
        self.radius_of_rotors = radius_of_rotors
        self.root_cutout = root_cutout
        self.root_chord = root_chord
        self.tip_chord = tip_chord
        self.root_pitch = root_pitch
        self.slope_pitch = slope_pitch
        self.default_airfoil = default_airfoil
        self.NACA_for_airfoil = NACA_for_airfoil
        
    def get_chord_length(self, r: float) -> float:
        assert self.parameters_set, "Rotor parameters not set. Please set them using 'set_rotor_parameters' method."
        return self.root_chord - (self.root_chord - self.tip_chord) * r
    
    def get_pitch_angle(self, r: float) -> float:
        """
        Enter the radius from center in meters
        """
        assert self.parameters_set, "Rotor parameters not set. Please set them using 'set_rotor_parameters' method."
        return self.root_pitch + self.slope_pitch * (r - self.root_cutout)
    
    def solve_λ_inflow(self, r: float, Vy: float, Ω: float, itermax: int = MAX_ITERATIONS, tol: float = TOLERANCE) -> float:
        return solve_iteratively_lambda_inflow_optimized(r, self.radius_of_rotors, self.number_of_blades, self.get_pitch_angle(r), Ω, Vy, itermax, tol)
