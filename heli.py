
from typing import Optional, Tuple, Dict, List, Union
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd


class Environment:
    def __init__(self) -> None:
        self.environment_set: bool = False
        self.temperature_sea_level: Optional[float] = None
        self.Reynolds_number: Optional[float] = None
        self.altitude: Optional[float] = None
        self.pressure_sea_level: Optional[float] = None
        self.Lapse_rate_troposphere: Optional[float] = None
        self.Specific_gas_constant: float = 287.053   # constant
        self.gravitational_acceleration: float = 9.80665  # constant
        self.specific_heat_constant: float = 1.4      # constant
        self.wind_velocity: Optional[float] = None
        self.ISA_OFFSET: Optional[float] = None

    def get_atmosphere_parameters(self) -> None:
        self.environment_set = True
        self.temperature_sea_level = float(input("Please enter the sea level temperature (K): "))
        self.Reynolds_number = float(input("Please enter the Reynolds number: "))
        self.altitude = float(input("Please enter the altitude (m): "))
        self.pressure_sea_level = float(input("Please enter the pressure at sea level (Pa): "))
        self.Lapse_rate_troposphere = float(input("Please enter the lapse rate of the troposphere (K/m): "))
        self.wind_velocity = float(input("Please enter the wind velocity (m/s): "))
        self.ISA_OFFSET = float(input("Please enter the ISA offset (K): "))

    def set_atmosphere_parameters(self, temperature_sea_level: float, Reynolds_number: float, 
                                altitude: float, pressure_sea_level: float, 
                                Lapse_rate_troposphere: float, wind_velocity: float, 
                                ISA_OFFSET: float) -> None:
        self.environment_set = True
        self.temperature_sea_level = temperature_sea_level
        self.Reynolds_number = Reynolds_number
        self.altitude = altitude
        self.pressure_sea_level = pressure_sea_level
        self.Lapse_rate_troposphere = Lapse_rate_troposphere
        self.wind_velocity = wind_velocity
        self.ISA_OFFSET = ISA_OFFSET

    def get_temperature(self, altitude: Optional[float] = None) -> float:
        if not self.environment_set:
            raise ValueError("Environment parameters have not been set.")
        alt: float = altitude if altitude is not None else self.altitude
        T_new: float = self.temperature_sea_level - self.Lapse_rate_troposphere * alt + self.ISA_OFFSET
        return T_new

    def get_pressure(self, temperature: float, altitude: Optional[float] = None) -> float:
        if not self.environment_set:
            raise ValueError("Environment parameters have not been set.")
        altitude: float = altitude if altitude is not None else self.altitude
        return self.pressure_sea_level * ((temperature / self.temperature_sea_level) ** 
                                        (self.gravitational_acceleration / (self.Lapse_rate_troposphere * self.Specific_gas_constant)))

    def get_speed_of_sound(self, temperature: float) -> float:
        # function to calculate the speed of sound
        if not self.environment_set:
            raise ValueError("Environment parameters have not been set.")
        return np.sqrt(1.4 * self.Specific_gas_constant * temperature)

    def get_mach_number(self, velocity: float, temperature: float) -> float:
        if not self.environment_set:
            raise ValueError("Environment parameters have not been set.")
        sound_speed: float = self.get_speed_of_sound(temperature)
        return velocity / sound_speed

    def get_density(self, temperature: float, pressure: Optional[float] = None) -> float:
        if not self.environment_set:
            raise ValueError("Environment parameters have not been set.")
        p: float = pressure if pressure is not None else self.get_pressure(temperature)
        return p / (self.Specific_gas_constant * temperature)

class Rotor:
    """Rotor Assembly characteristics"""
    def __init__(self, environment: Optional['Environment'] = None) -> None:
        self.environment: Optional['Environment'] = environment
        self.parameters_set: bool = False
        self.number_of_blades: Optional[int] = None
        self.omega: Optional[float] = None
        self.blade_mass: Optional[float] = None
        self.NACA_for_airfoil: Optional[str] = None
        self.radius_of_rotors: Optional[float] = None
        self.root_cutout: Optional[float] = None
        self.angle_of_attack: Optional[float] = None
        self.root_chord: Optional[float] = None
        self.tip_chord: Optional[float] = None
        self.root_pitch: Optional[float] = None
        self.slope_pitch: Optional[float] = None

    def get_rotor_parameters(self) -> None:
        self.parameters_set = True
        self.number_of_blades = int(input("Please enter the number of rotors: "))
        self.omega = float(input("Please enter the angular velocity of the rotor (rad/s): "))  # rotor_omega=2*np.pi*rotor_rpm/60
        self.blade_mass = float(input("Please enter the mass of the rotors (kg): "))
        self.NACA_for_airfoil = input("Please enter the NACA number of the airfoil: ")
        self.radius_of_rotors = float(input("Please enter the radius of the rotors (m): "))
        self.root_cutout = float(input("Please enter the root cutout of the rotors (m): "))
        self.angle_of_attack = np.deg2rad(float(input("Please enter the angle of attack (degrees): ")))
        self.root_chord = float(input("Please enter the root chord (m): "))
        self.tip_chord = float(input("Please enter the tip chord (m): "))
        self.root_pitch = np.deg2rad(float(input("Please enter the root pitch (degrees): ")))
        self.slope_pitch = float(input("Please enter the slope of the pitch: "))
        

    def set_rotor_parameters(self, number_of_blades: int, omega: float, blade_mass: float, 
                           NACA_for_airfoil: str, radius_of_rotors: float, root_cutout: float, 
                           angle_of_attack: float, root_chord: float, tip_chord: float, 
                           root_pitch: float, slope_pitch: float) -> None:
        self.parameters_set = True
        self.number_of_blades = number_of_blades
        self.omega = omega
        self.blade_mass = blade_mass
        self.NACA_for_airfoil = NACA_for_airfoil
        self.radius_of_rotors = radius_of_rotors
        self.root_cutout = root_cutout
        self.angle_of_attack = np.deg2rad(angle_of_attack) if angle_of_attack > 1 else angle_of_attack
        self.root_chord = root_chord
        self.tip_chord = tip_chord
        self.root_pitch = np.deg2rad(root_pitch) if root_pitch > 1 else root_pitch
        self.slope_pitch = slope_pitch

    def get_chord_length(self, x: float) -> float:
        # Function to get chord_length at a given location.
        if not self.parameters_set:
            raise ValueError("Rotor parameters have not been set.")
        return self.root_chord - ((self.root_chord - self.tip_chord) / (self.radius_of_rotors - self.root_cutout)) * x

    def get_pitch(self, x: float) -> float:
        # Basically involving twists
        if not self.parameters_set:
            raise ValueError("Rotor parameters have not been set.")
        return self.root_pitch + self.slope_pitch * x

    def get_Cl_Cd_Cm(self, naca_number: str, alpha: float) -> Tuple[float, float, float]:
        # Extract coefficients
        cl: float = 5.75
        cd: float = 0.013
        cm: float = 0.03
        return cl, cd, cm

    def elemental_inflow_ratio(self, r_distance: float, climb_velocity: float, 
                             max_iter: int = 100, tol: float = 1e-6) -> Tuple[float, float]:
        """Inflow for each blade section elementally
        
        λ = sqrt((σa/16F - λc/2)² + σa/8F * θ * r/R) - (σa/16F - λc/2)
        
        where:
        λc = V/ΩR,  λ = (V + v)/ΩR
        F = (2/π)cos⁻¹(e^(-f))  where f = b(1-r/R)/λ
        dT = 4πρrF(V+v)vdr = 4πρrF(λωr)(λωr - V)dr
        T = b∫[Rc to R]dT
        
        * we also include prandtl's tip loss model in the inflow ratio formula
        * Since Lambda is dependent on F and f further is dependent on lambda we have to 
          iteratively find out the value of Lambda until solution stabilizes.
        """
        if not self.parameters_set:
            raise ValueError("Rotor parameters have not been set.")
            
        lambda_c: float = climb_velocity / (self.omega * self.radius_of_rotors)
        a: float = 5.75  # Define some useful parameters
        F: float = 1.1  # Initial guess values to start with
        lambda_inflow: float = 0.2

        for i in range(max_iter):
            theta: float = self.get_pitch(r_distance)
            
            # getting the Cl (lift coefficient)
            sigma: float = self.number_of_blades * self.get_chord_length(r_distance) / (np.pi * self.radius_of_rotors)  # since sigma is not constant
            
            # Update lambda_inflow using current F
            new_lambda_inflow: float = (
                np.sqrt(((sigma * a) / (16 * F) - (lambda_c / 2))**2
                        + (sigma * a) / (8 * F) * theta * (r_distance / self.radius_of_rotors))
                - ((sigma * a) / (16 * F) - (lambda_c / 2))
            )

            # Update F using the new lambda_inflow (Gauss–Seidel step)
            f: float = (self.number_of_blades / 2) * ((1 - r_distance / self.radius_of_rotors) / lambda_inflow)
            new_F: float = (2 / np.pi) * np.arccos(np.exp(-f))

            # Check convergence
            if abs(new_lambda_inflow - lambda_inflow) < tol and abs(new_F - F) < tol:
                return new_lambda_inflow, new_F

            # Update values for next iteration
            lambda_inflow, F = new_lambda_inflow, new_F

        return lambda_inflow, F

    def get_phi_r(self, r_distance: float, climb_velocity: float) -> float:
        if not self.parameters_set:
            raise ValueError("Rotor parameters have not been set.")
        lambda_inflow: float
        lambda_inflow, _ = self.elemental_inflow_ratio(r_distance, climb_velocity)
        return np.arctan(lambda_inflow * self.radius_of_rotors / r_distance)

    def get_alpha_effective_r(self, r_distance: float, climb_velocity: float) -> float:
        if not self.parameters_set:
            raise ValueError("Rotor parameters have not been set.")
        phi: float = self.get_phi_r(r_distance, climb_velocity)
        theta: float = self.get_pitch(r_distance)
        return theta - phi

    def get_cL(self, r_distance: float, climb_velocity: float, a: float = 5.75) -> float:
        if not self.parameters_set:
            raise ValueError("Rotor parameters have not been set.")
        alpha_eff: float = self.get_alpha_effective_r(r_distance, climb_velocity)
        return a * alpha_eff

    def get_cD(self, r_distance: float, climb_velocity: float, CD0: float = 0.005, 
             k: float = 0.1, a: float = 5.75) -> float:
        if not self.parameters_set:
            raise ValueError("Rotor parameters have not been set.")
        alpha_eff: float = self.get_alpha_effective_r(r_distance, climb_velocity)
        return CD0 + k * ((a * alpha_eff) ** 2)

    def elemental_thrust(self, r_distance: float, dr: float, climb_velocity: float, 
                        density: float = 1.38) -> float:
        # we now compute the elemental thrust
        if not self.parameters_set:
            raise ValueError("Rotor parameters have not been set.")
        lambda_inflow: float
        lambda_inflow, _ = self.elemental_inflow_ratio(r_distance, climb_velocity)
        dT: float = 4 * np.pi * density * r_distance * (lambda_inflow * self.omega * self.radius_of_rotors) * ((lambda_inflow * self.omega * self.radius_of_rotors) - climb_velocity) * dr
        return dT

    def elemental_torque(self, r_distance: float, dr: float, climb_velocity: float) -> float:
        if not self.parameters_set:
            raise ValueError("Rotor parameters have not been set.")
        dT: float = self.elemental_thrust(r_distance, dr, climb_velocity)
        return dT * r_distance

    def total_thrust(self, climb_velocity: float, n_divisions: int = 50, density: float = 1.38) -> float:
        """Calculating the thrust from all rotors"""
        if not self.parameters_set:
            raise ValueError("Rotor parameters have not been set.")
        r: np.ndarray = np.linspace(self.root_cutout, self.radius_of_rotors, n_divisions)
        delta_r: float = r[1] - r[0]
        store_dT: List[float] = []
        for i in range(1, n_divisions - 1):
            store_dT.append(self.elemental_thrust(r[i], delta_r, climb_velocity, density))
        net_thrust: float = self.number_of_blades * np.sum(store_dT)
        return net_thrust

    def total_torque(self, climb_velocity: float, n_divisions: int = 50, 
                    density: float = 1.2, cm: float = 0.1) -> float:  # per blade
        if not self.parameters_set:
            raise ValueError("Rotor parameters have not been set.")
        r: np.ndarray = np.linspace(self.root_cutout, self.radius_of_rotors, n_divisions)
        delta_r: float = r[1] - r[0]
        store_dT: List[float] = []
        for i in range(1, n_divisions - 1):
            torque_element: float = self.elemental_torque(r[i], delta_r, climb_velocity)
            additional_torque: float = 0.5 * density * cm * (self.omega * r[i]) ** 2
            store_dT.append(torque_element * r[i] + additional_torque)
        net_moment: float = self.number_of_blades * np.sum(store_dT)
        return net_moment

    def total_power(self, climb_velocity: float, n_divisions: int = 50) -> float:
        if not self.parameters_set:
            raise ValueError("Rotor parameters have not been set.")
        r: np.ndarray = np.linspace(self.root_cutout, self.radius_of_rotors, n_divisions)
        delta_r: float = r[1] - r[0]
        store_dT: List[float] = []
        for i in range(1, n_divisions - 1):
            dT: float = self.elemental_thrust(r[i], delta_r, climb_velocity)
            lambda_inflow: float
            lambda_inflow, _ = self.elemental_inflow_ratio(r[i], climb_velocity)
            store_dT.append(dT * lambda_inflow * self.omega * r[i])
        net_power: float = self.number_of_blades * np.sum(store_dT)
        return net_power

    def calculate_performance(self, climb_velocity: float, n_divisions: int = 50, 
                            density: float = 1.2) -> Dict[str, float]:
        if not self.parameters_set:
            raise ValueError("Rotor parameters have not been set.")
        
        r: np.ndarray = np.linspace(self.root_cutout, self.radius_of_rotors, n_divisions)
        delta_r: float = r[1] - r[0]
        
        dL_list: List[float] = []
        dD_list: List[float] = []
        dT_list: List[float] = []
        dFx_list: List[float] = []
        dP_list: List[float] = []
        dtorque_list: List[float] = []
        
        for i in range(len(r)):
            chord: float = self.get_chord_length(r[i])
            cl: float = self.get_cL(r[i], climb_velocity)
            cd: float = self.get_cD(r[i], climb_velocity)
            lambda_inflow: float
            lambda_inflow, _ = self.elemental_inflow_ratio(r[i], climb_velocity)
            phi: float = self.get_phi_r(r[i], climb_velocity)
            
            velocity_squared: float = (self.omega * r[i]) ** 2 + (lambda_inflow * self.omega * self.radius_of_rotors) ** 2
            
            dL: float = 0.5 * density * chord * cl * velocity_squared
            dD: float = 0.5 * density * chord * cd * velocity_squared
            dT: float = (dL * np.cos(phi) - dD * np.sin(phi)) * delta_r
            dFx: float = (dD * np.cos(phi) + dL * np.sin(phi)) * delta_r
            dP: float = self.omega * r[i] * dFx
            dtorque: float = r[i] * dFx
            
            dL_list.append(dL)
            dD_list.append(dD)
            dT_list.append(dT)
            dFx_list.append(dFx)
            dP_list.append(dP)
            dtorque_list.append(dtorque)
        
        performance: Dict[str, float] = {
            'thrust': np.sum(dT_list) * self.number_of_blades,
            'torque': np.sum(dtorque_list) * self.number_of_blades,
            'power': np.sum(dP_list) * self.number_of_blades,
            'thrust_x': np.sum(dFx_list) * self.number_of_blades,
            'lift': np.sum(dL_list),
            'drag': np.sum(dD_list)
        }
        
        return performance

class Helicopter:
    """Aircraft Helicopter Parameters from the user"""
    def __init__(self, environment: Optional['Environment'] = None) -> None:
        self.environment: Optional['Environment'] = environment if environment else Environment()
        self.rotor: Optional['Rotor'] = None
        self.fuselage_set: bool = False
        self.engine_set: bool = False
        self.rotor_set: bool = False
        
        self.fuselage_mass: Optional[float] = None
        self.fuselage_length: Optional[float] = None
        self.fuselage_width: Optional[float] = None
        self.fuselage_height: Optional[float] = None
        self.fuselage_cl: Optional[float] = None
        self.fuselage_cd: Optional[float] = None
        
        self.engine_mass: Optional[float] = None
        self.engine_BSFC: Optional[float] = None
        self.engine_shaft_power_conversion_efficiency: Optional[float] = None
        self.engine_fractional_power_tail_rotor: Optional[float] = None

    def get_fuselage_parameters(self) -> None:
        # fuselage parameters
        self.fuselage_set = True
        self.fuselage_mass = float(input("Please enter the fuselage mass (kg): "))
        self.fuselage_length = float(input("Please enter the fuselage length (m): "))
        self.fuselage_width = float(input("Please enter the fuselage width (m): "))
        self.fuselage_height = float(input("Please enter the fuselage height (m): "))
        self.fuselage_cl = float(input("Please enter the fuselage lift coefficient (Cl): "))
        self.fuselage_cd = float(input("Please enter the fuselage drag coefficient (Cd): "))

    def set_fuselage_parameters(self, mass: float, length: float, width: float, 
                              height: float, cl: float, cd: float) -> None:
        self.fuselage_set = True
        self.fuselage_mass = mass
        self.fuselage_length = length
        self.fuselage_width = width
        self.fuselage_height = height
        self.fuselage_cl = cl
        self.fuselage_cd = cd

    def get_engine_parameters(self) -> None:
        # Aircraft engine parameters
        self.engine_set = True
        self.engine_mass = float(input("Please enter the mass of the engine (kg): "))
        self.engine_BSFC = float(input("Please enter the BSFC of the engine (kg/kWh): "))
        self.engine_shaft_power_conversion_efficiency = float(input("Please enter the shaft power conversion efficiency: "))
        self.engine_fractional_power_tail_rotor = float(input("Please enter the fractional power of the tail rotor: "))

    def set_engine_parameters(self, mass: float, BSFC: float, 
                            shaft_power_conversion_efficiency: float, 
                            fractional_power_tail_rotor: float) -> None:
        self.engine_set = True
        self.engine_mass = mass
        self.engine_BSFC = BSFC
        self.engine_shaft_power_conversion_efficiency = shaft_power_conversion_efficiency
        self.engine_fractional_power_tail_rotor = fractional_power_tail_rotor

    def setup_rotor(self) -> 'Rotor':
        if not self.rotor_set:
            self.rotor = Rotor(self.environment)
            self.rotor.get_rotor_parameters()
            self.rotor_set = True
        return self.rotor

    def set_rotor(self, rotor: 'Rotor') -> None:
        self.rotor = rotor
        self.rotor_set = True

    def get_total_mass(self) -> float:
        total_mass: float = 0
        if self.fuselage_set:
            total_mass += self.fuselage_mass
        if self.engine_set:
            total_mass += self.engine_mass
        if self.rotor_set and self.rotor:
            total_mass += self.rotor.blade_mass * self.rotor.number_of_blades
        return total_mass

    def calculate_hover_performance(self,n_divisions: int = 50) -> Dict[str, float]:
        if not self.rotor_set:
            raise ValueError("Rotor has not been set up.")
        
        density: float = 1.2
        if self.environment and self.environment.environment_set:
            temperature: float = self.environment.get_temperature()
            pressure: float = self.environment.get_pressure(temperature)
            density = self.environment.get_density(temperature, pressure)
        
        climb_velocity = 0
        return self.rotor.calculate_performance(climb_velocity, n_divisions, density)

    def calculate_thrust_required_for_hover(self) -> float:
        total_mass: float = self.get_total_mass()
        g: float
        if self.environment and self.environment.environment_set:
            g = self.environment.gravitational_acceleration
        else:
            g = 9.80665
        return total_mass * g

    def can_hover(self, climb_velocity: float = 0, n_divisions: int = 50) -> Tuple[bool, str]:
        if not self.rotor_set:
            return False, "Rotor parameters not set"
        
        performance: Dict[str, float] = self.calculate_hover_performance(climb_velocity, n_divisions)
        thrust_required: float = self.calculate_thrust_required_for_hover()
        
        if performance['thrust'] >= thrust_required:
            return True, f"Can hover. Available thrust: {performance['thrust']:.1f} N, Required: {thrust_required:.1f} N"
        else:
            return False, f"Cannot hover. Available thrust: {performance['thrust']:.1f} N, Required: {thrust_required:.1f} N"
