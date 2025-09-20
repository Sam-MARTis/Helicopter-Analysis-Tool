
from typing import Optional, Tuple, Dict, List, Union
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
from scipy.optimize import minimize_scalar
from time import sleep

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
        return min(velocity / sound_speed, 0.9)

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
        self.blade_mass: Optional[float] = None
        self.NACA_for_airfoil: Optional[str] = None
        self.radius_of_rotors: Optional[float] = None
        self.root_cutout: Optional[float] = None
        self.root_chord: Optional[float] = None
        self.tip_chord: Optional[float] = None
        self.root_pitch: Optional[float] = None
        self.slope_pitch: Optional[float] = None
        self.default_airfoil:bool = True

    def get_rotor_parameters(self) -> None:
        self.parameters_set = True
        self.number_of_blades = int(input("Please enter the number of rotors: "))
        # self.omega = float(input("Please enter the angular velocity of the rotor (rad/s): "))  # rotor_omega=2*np.pi*rotor_rpm/60
        self.blade_mass = float(input("Please enter the mass of the rotors (kg): "))
        self.NACA_for_airfoil = input("Please enter the NACA number of the airfoil: ")
        self.radius_of_rotors = float(input("Please enter the radius of the rotors (m): "))
        self.root_cutout = float(input("Please enter the root cutout of the rotors (m): "))
        self.root_chord = float(input("Please enter the root chord (m): "))
        self.tip_chord = float(input("Please enter the tip chord (m): "))
        self.root_pitch = np.deg2rad(float(input("Please enter the root pitch (degrees): ")))
        self.slope_pitch = float(input("Please enter the slope of the pitch: "))
        custom_file:bool = False
        if(input("Do you want to provide a custom aero csv file with cl, cd and cm? (y/n): ").lower() == 'y'):
            custom_file = True
            self.aero_filepath = input("Please enter the path to the custom aero csv file: ")
            
            try:
                self._build_aero_polynomials(self.aero_filepath)
                self.default_airfoil = False
            except Exception as e:
                print(f"Error building aero polynomials: {e}")
                print("Switching to default airfoil")
                self.default_airfoil = True

    def set_rotor_parameters(self, number_of_blades: int, blade_mass: float,
                           NACA_for_airfoil: str, radius_of_rotors: float, root_cutout: float, root_chord: float, tip_chord: float,
                           root_pitch: float, slope_pitch: float, filepath:str = None) -> None:
        self.parameters_set = True
        self.number_of_blades = number_of_blades
        self.blade_mass = blade_mass
        self.NACA_for_airfoil = NACA_for_airfoil
        self.radius_of_rotors = radius_of_rotors
        self.root_cutout = root_cutout
        self.root_chord = root_chord
        self.tip_chord = tip_chord
        self.root_pitch = np.deg2rad(root_pitch)
        self.slope_pitch = slope_pitch
        if filepath is not None:
            self.aero_filepath = filepath
            try:
                self._build_aero_polynomials(self.aero_filepath)
                self.default_airfoil = False
            except Exception as e:
                print(f"Error building aero polynomials: {e}")
                print("Switching to default airfoil")
                self.default_airfoil = True

    # def set_aero_data_file(self, filename: str) -> None:
    #     """Set the aero data file and build Cl, Cd functions"""
    #     self.aero_filepath = filename
    #     self._build_aero_polynomials()

    def _build_aero_polynomials(self, filepath, degree: int = 4):
        """Read aero table and fit polynomials for Cl(α) and Cd(α)."""
        if self.aero_filepath is None:
            raise ValueError("Aero data file not set. Call set_aero_data_file(filename).")

        try:
            # Read CSV with comma delimiter and use the existing header
            df = pd.read_csv(filepath, comment='#')
            print(f"Successfully read airfoil data from {filepath}")
        except Exception as e:
            raise ValueError(f"Could not read aero file '{filepath}': {e}")

        if df.empty:
            raise ValueError(f"Aero file '{filepath}' is empty or not in expected format.")

        # Note: alpha is already in degrees in the CSV file
        alpha = df["alpha"].to_numpy()
        cl = df["cl"].to_numpy()
        cd = df["cd"].to_numpy()
        cm = df["cm"].to_numpy()
        

        # Fit polynomials
        # Fit polynomials for Cl data
        temp_poly = np.poly1d(np.polyfit(alpha, cl, degree))
        # Find alpha at maximum Cl (stall angle)
        res = minimize_scalar(lambda x: -temp_poly(x),
                              bounds=(alpha.min(), alpha.max()), method='bounded')
        self.alpha_stall = res.x  # This is in degrees since alpha is in degrees
        self.cl_stall = temp_poly(self.alpha_stall)
        
        print(f"Calculated stall angle: {self.alpha_stall:.2f} degrees, Cl_max: {self.cl_stall:.4f}")

        # Use 93% of stall angle for the pre-stall region
        self.alpha_cutoff = 0.93 * self.alpha_stall
        mask = alpha <= self.alpha_cutoff

        # Create polynomials from pre-stall data
        alpha_pre_stall = alpha[mask]
        cl_pre_stall = cl[mask]
        cd_pre_stall = cd[mask]

        # Make sure we have enough points for fitting
        if len(alpha_pre_stall) < degree + 1:
            print(f"Warning: Not enough pre-stall points ({len(alpha_pre_stall)}) for degree {degree}. Reducing degree.")
            degree = max(1, len(alpha_pre_stall) - 1)

        self.cl_poly = np.poly1d(np.polyfit(alpha_pre_stall, cl_pre_stall, degree))
        # Use the same length array for cd_poly
        self.cd_poly = np.poly1d(np.polyfit(alpha, cd, degree))
        print(f"Airfoil polynomials created successfully with {len(alpha_pre_stall)} pre-stall points")




    def get_chord_length(self, x: float) -> float:
        # Function to get chord_length at a given location.
        if not self.parameters_set:
            raise ValueError("Rotor parameters have not been set.")
        return self.root_chord - ((self.root_chord - self.tip_chord) / (self.radius_of_rotors - self.root_cutout)) * (x - self.root_cutout)

    def get_pitch(self, x: float) -> float:
        # Basically involving twists
        if not self.parameters_set:
            raise ValueError("Rotor parameters have not been set.")
        return self.root_pitch + self.slope_pitch * x

    def get_Cl_Cd_Cm(self, naca_number: str, alpha: float) -> Tuple[float, float, float]:
        # Extract coefficients
        cl: float = 5.75
        cd: float = 0.0113
        cm: float = 0.03
        return cl, cd, cm

    def elemental_inflow_ratio(self, r_distance: float, climb_velocity: float, omega: float,
                             max_iter: int = 20, tol: float = 1e-6) -> Tuple[float, float]:
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

        lambda_c: float = climb_velocity / (omega * self.radius_of_rotors)
        a: float = 5.75  # Define some useful parameters
        F: float = 1.1  # Initial guess values to start with
        lambda_inflow: float = 0.2

        
        theta: float = self.get_pitch(r_distance)
        sigma: float = self.number_of_blades * self.get_chord_length(r_distance) / (np.pi * self.radius_of_rotors)  # since sigma is not constant
        
        for iteration in range(max_iter):
            # Ensure F doesn't become zero to prevent divide by zero
            if F <= 1e-8:
                F = 1e-8
            
            new_lambda_inflow: float = (
                np.sqrt(((sigma * a) / (16 * F) - (lambda_c / 2))**2
                        + (sigma * a) / (8 * F) * theta * (r_distance / self.radius_of_rotors))
                - ((sigma * a) / (16 * F) - (lambda_c / 2))
            )

            # Prevent negative or zero lambda_inflow
            if new_lambda_inflow <= 1e-8:
                new_lambda_inflow = 1e-8 
            
            f: float = (self.number_of_blades / 2) * ((1 - r_distance / self.radius_of_rotors) / new_lambda_inflow)

            # Limit f to prevent numerical issues
            f = min(max(f, 1e-8), 50)  # Cap f at reasonable value
            new_F: float = (2 / np.pi) * np.arccos(np.exp(-f))

            # Check convergence
            lambda_diff = abs(new_lambda_inflow - lambda_inflow)
            F_diff = abs(new_F - F)
            
            if lambda_diff < tol and F_diff < tol:
                return new_lambda_inflow, new_F

            # Use relaxation to improve convergence
            relaxation = 0.7  # Under-relaxation factor
            lambda_inflow = relaxation * new_lambda_inflow + (1 - relaxation) * lambda_inflow
            F = relaxation * new_F + (1 - relaxation) * F

        return lambda_inflow, F

    def get_phi_r(self, r_distance: float, climb_velocity: float, omega: float, lambda_inflow: float = None) -> float:
        if not self.parameters_set:
            raise ValueError("Rotor parameters have not been set.")
        if lambda_inflow is None:
            lambda_inflow, _ = self.elemental_inflow_ratio(r_distance, climb_velocity, omega)
        return np.arctan(lambda_inflow * self.radius_of_rotors / r_distance)

    def get_alpha_effective_r(self, r_distance: float, climb_velocity: float, omega: float, lambda_inflow: float = None) -> float:
        if not self.parameters_set:
            raise ValueError("Rotor parameters have not been set.")
        phi: float = self.get_phi_r(r_distance, climb_velocity, omega, lambda_inflow)
        theta: float = self.get_pitch(r_distance)
        return theta - phi

    def safe_compressibility_factor(self, omega: float, r_distance: float, max_mach: float = 0.9) -> float:
        """Calculate compressibility factor with bounds to prevent supersonic issues"""
        temperature = self.environment.get_temperature()
        mach = self.environment.get_mach_number(omega * r_distance, temperature)
        mach = min(abs(mach), max_mach)

        return 1.0 / np.sqrt(1 - mach**2)

    def get_cL(self, r_distance: float, climb_velocity: float, omega: float, lambda_inflow: float = None, a: float = 5.75) -> Tuple[float, bool]:
        if not self.parameters_set:
            raise ValueError("Rotor parameters have not been set.")
        alpha_eff: float = self.get_alpha_effective_r(r_distance, climb_velocity, omega, lambda_inflow)
        compressibility_factor = self.safe_compressibility_factor(omega, r_distance)
        if(self.default_airfoil):
            return a * alpha_eff * compressibility_factor, False
        else:
            # Convert alpha_eff from radians to degrees for the polynomial that was fit to degree data
            cl_value = self.cl_poly(np.degrees(alpha_eff)) * compressibility_factor
            return cl_value, alpha_eff > self.alpha_stall

    def get_cD(self, r_distance: float, climb_velocity: float, omega: float, lambda_inflow: float = None, CD0: float = 0.0113,
             k: float = 0.037807, a: float = 5.75) -> Tuple[float, bool]:

        if not self.parameters_set:
            raise ValueError("Rotor parameters have not been set.")
        alpha_eff: float = self.get_alpha_effective_r(r_distance, climb_velocity, omega, lambda_inflow)
        compressibility_factor = self.safe_compressibility_factor(omega, r_distance)
        if(self.default_airfoil):
            return CD0 + k * ((a * alpha_eff) ** 2) * compressibility_factor, False
        else:
            return self.cd_poly(np.degrees(alpha_eff)) * compressibility_factor, alpha_eff > self.alpha_stall 



    def elemental_thrust(self, r_distance: float, dr: float, climb_velocity: float, omega:float,
                        density: float = None) -> float:
        assert density is not None, "Density must be provided in elemental_thrust function"
        """
        Returns the thrust at r distance from the root distance from chord

        Calls the elemental_inflow_ratio method to get the inflow ratio.

        Parameters
        ----------
        r_distance : float
            Distance from the rotor hub.
        dr : float
            Element size.
        climb_velocity : float
            Climb velocity.
        omega : float
            Rotor angular velocity.
        density : float
            Air density.

        Returns
        -------
        float
            Elemental thrust at that distance. .
        """
        # we now compute the elemental thrust using blade element theory
        if not self.parameters_set:
            raise ValueError("Rotor parameters have not been set.")
        lambda_inflow: float
        lambda_inflow, F = self.elemental_inflow_ratio(r_distance, climb_velocity, omega)
        
        # Get aerodynamic coefficients
        cl, _ = self.get_cL(r_distance, climb_velocity, omega, lambda_inflow)
        cd, _ = self.get_cD(r_distance, climb_velocity, omega, lambda_inflow)
        phi: float = self.get_phi_r(r_distance, climb_velocity, omega, lambda_inflow)
        chord: float = self.get_chord_length(r_distance)
        
        
        velocity_squared: float = (omega * r_distance) ** 2 + (lambda_inflow * omega * self.radius_of_rotors) ** 2

        # Apply tip loss using F
        dL: float = 0.5 * density * chord * cl * velocity_squared * F 
        dD: float = 0.5 * density * chord * cd * velocity_squared * F 
        dT: float = (dL * np.cos(phi) - dD * np.sin(phi)) * dr
        return dT

    def elemental_torque(self, r_distance: float, dr: float, climb_velocity: float, omega: float, density: float) -> float:
        """
        returns the torque it requires to move that element due to drag on it.

        inputs:
        1. r_distance
        2. dr
        3. climb_velocity

        returns:
        elemental torque

        """

        if not self.parameters_set:
            raise ValueError("Rotor parameters have not been set.")
        
        # Calculate torque using blade element theory (not thrust * radius)
        lambda_inflow, F = self.elemental_inflow_ratio(r_distance, climb_velocity, omega)
        
        # Get aerodynamic coefficients
        cl, _ = self.get_cL(r_distance, climb_velocity, omega, lambda_inflow)
        cd, _ = self.get_cD(r_distance, climb_velocity, omega, lambda_inflow)
        phi: float = self.get_phi_r(r_distance, climb_velocity, omega, lambda_inflow)
        chord: float = self.get_chord_length(r_distance)
        
        # Calculate velocity magnitude squared
        velocity_squared: float = (omega * r_distance) ** 2 + (lambda_inflow * omega * self.radius_of_rotors) ** 2
        
        # Blade element torque calculation
        dL: float = 0.5 * density * chord * cl * velocity_squared * F
        dD: float = 0.5 * density * chord * cd * velocity_squared * F
        dQ: float = (dD * np.cos(phi) + dL * np.sin(phi)) * r_distance * dr
        return dQ

    def total_thrust(self, climb_velocity: float, omega: float, density: float, n_divisions: int = 50) -> float:
        """Calculating the thrust from all rotors"""

        """Calculating the thrust from all rotors

          inputs:
          1. climb_velocity
          2. n_divisions : the no.of elements we are taking
          3. density

          returns:
          total thrust

        """

        if not self.parameters_set:
            raise ValueError("Rotor parameters have not been set.")
        r: np.ndarray = np.linspace(self.root_cutout, self.radius_of_rotors, n_divisions)
        delta_r: float = r[1] - r[0]
        store_dT: List[float] = []
        for i in range(1, n_divisions - 1):
            store_dT.append(self.elemental_thrust(r[i], delta_r, climb_velocity, omega=omega, density=density))
        net_thrust: float = self.number_of_blades * np.sum(store_dT)
        return net_thrust

    def total_torque(self, climb_velocity: float, omega: float,
                    density: float, n_divisions: int = 50,  cm: float = 0.1) -> float:  # per blade
        """
          calculates the net torque required to generate the thrust

          inputs:
          1. climb_velocity
          2. n_divisions
          3. density
          4. cm

          returns:
          total torque

        """

        if not self.parameters_set:
            raise ValueError("Rotor parameters have not been set.")
        r: np.ndarray = np.linspace(self.root_cutout, self.radius_of_rotors, n_divisions)
        delta_r: float = r[1] - r[0]
        store_dT: List[float] = []
        for i in range(1, n_divisions - 1):
            torque_element: float = self.elemental_torque(r[i], delta_r, climb_velocity, omega=omega, density=density)
            additional_torque: float = 0.5 * density * cm * self.get_chord_length(r[i]) * (omega * r[i]) ** 2 * delta_r
            store_dT.append(torque_element + additional_torque)
        net_moment: float = self.number_of_blades * np.sum(store_dT)
        return net_moment

    def total_power(self, climb_velocity: float, omega: float, density: float, n_divisions: int = 50) -> float:
        """
          calculates the net power required to generate the thrust

          inputs:
          1. climb_velocity
          2. n_divisions

          returns:
          total power

        """

        if not self.parameters_set:
            raise ValueError("Rotor parameters have not been set.")
        
        # Power = Omega * Torque (P = Ω × Q)
        total_torque_value: float = self.total_torque(climb_velocity, omega, density, n_divisions)
        net_power: float = omega * total_torque_value
        return net_power

    def calculate_performance(self, climb_velocity: float, omega: float, density: float, n_divisions: int = 50, cm: float = 0.1) -> Dict[str, float]:
        """
          calculates the performance of the rotor

          inputs:
          1. climb_velocity
          2. n_divisions
          3. density

          returns:
            performance
              1. thrust
              2. torque
              3. power
              4. lift
              5. drag

        """

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
            lambda_inflow, F = self.elemental_inflow_ratio(r[i], climb_velocity, omega)
            cl, _ = self.get_cL(r[i], climb_velocity, omega, lambda_inflow)
            cd, _ = self.get_cD(r[i], climb_velocity, omega, lambda_inflow)
            phi: float = self.get_phi_r(r[i], climb_velocity, omega, lambda_inflow=lambda_inflow)

            velocity_squared: float = (omega * r[i]) ** 2 + (lambda_inflow * omega * self.radius_of_rotors) ** 2

            dL: float = 0.5 * density * chord * cl * velocity_squared * F  # Apply tip loss factor
            dD: float = 0.5 * density * chord * cd * velocity_squared * F  # Apply tip loss factor
            dT: float = (dL * np.cos(phi) - dD * np.sin(phi)) * delta_r
            dFx: float = (dD * np.cos(phi) + dL * np.sin(phi)) * delta_r
            dP: float = omega * r[i] * dFx
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
    
    def find_omega_needed_uncoupled(self, thrust_needed, vertical_velocity: float, altitude: float, initial_guess: float, iterations: int= 300, tol: float = 0.05) -> float:
        temperature = self.environment.get_temperature(altitude)
        pressure = self.environment.get_pressure(temperature, altitude)
        density = self.environment.get_density(temperature=temperature, pressure=pressure)
        omega = initial_guess*0.9
        domega = initial_guess*0.1
        prev_exceeded = False
        for _ in range(iterations):
            omega += domega
            thrust = self.total_thrust(vertical_velocity, omega, density)
            error = abs(thrust - thrust_needed)
            
            if error < tol:
                return omega
            if(thrust > thrust_needed):
                if(not prev_exceeded):
                    domega *= -0.5
                prev_exceeded = True
            else:
                if(prev_exceeded):
                    domega *= -0.5
                prev_exceeded = False
                
        if(abs(thrust - thrust_needed) > 10*tol):
            print("Warning: Omega calculation did not converge")
        return omega

    def is_rotor_stalling(self, vertical_velocity: float, omega: float, divisions: int = 10) -> bool:
        if(self.default_airfoil):
            # For default airfoil, use 20 degrees as target (in radians for comparison)
            target_rad = np.deg2rad(20)
        else:
            # For custom airfoil, convert stall angle from degrees to radians for comparison
            target_rad = np.deg2rad(self.alpha_stall)
        
        dr = (self.radius_of_rotors - self.root_cutout) / divisions
        stalling = False
        for i in range(divisions):
            r = self.root_cutout + (i + 0.5) * dr
            angle_rad = self.get_alpha_effective_r(r, vertical_velocity, omega=omega)
            
            if angle_rad > target_rad:
                print(f"Stall detected at r={r:.2f}m: angle={np.degrees(angle_rad):.2f}°, limit={np.degrees(target_rad):.2f}°")
                stalling = True
                break
            
        return stalling
    def get_cL_custom(self, r_distance: float, climb_velocity: float, omega: float, a: float = 5.75, pitch_factor = 1) -> Tuple[float, bool, float]:

        if not self.parameters_set:
            raise ValueError("Rotor parameters have not been set.")
        if self.cl_poly is None:
            raise ValueError("Cl function not built. Call set_aero_data_file().")
        compressibility_factor = self.safe_compressibility_factor(omega, r_distance)
        alpha_eff: float = self.get_alpha_effective_r(r_distance, climb_velocity=climb_velocity, omega=omega)*pitch_factor
        return self.cl_poly(alpha_eff)*compressibility_factor, alpha_eff > self.alpha_stall, self.cl_poly(self.alpha_stall)

    # def get_cD(self, r_distance: float, climb_velocity: float,
    #            CD0: float = 0.005, k: float = 0.1, a: float = 5.75, pitch_factor = 1):

    #     if not self.parameters_set:
    #         raise ValueError("Rotor parameters have not been set.")
    #     if self.cd_poly is None:
    #         raise ValueError("Cd function not built. Call set_aero_data_file().")

    #     alpha_eff: float = self.get_alpha_effective_r(r_distance, climb_velocity, pitch_factor)
    #     return self.cd_poly(alpha_eff)
    
    def get_cD_custom(self, r_distance: float, climb_velocity: float, omega: float, pitch_factor = 1) -> Tuple[float, bool]:
        if not self.parameters_set:
            raise ValueError("Rotor parameters have not been set.")
        if self.cd_poly is None:
            raise ValueError("Cd function not built. Call set_aero_data_file().")
        compressibility_factor = self.safe_compressibility_factor(omega, r_distance)

        alpha_eff: float = self.get_alpha_effective_r(r_distance, climb_velocity=climb_velocity, omega=omega)*pitch_factor
        return self.cd_poly(alpha_eff)*compressibility_factor

        # if not self.parameters_set:
        #     raise ValueError("Rotor parameters have not been set.")
        # alpha_eff: float = self.get_alpha_effective_r(r_distance, climb_velocity, omega, lambda_inflow)
        # compressibility_factor = self.safe_compressibility_factor(omega, r_distance)
        # if(self.default_airfoil):
        #     return a * alpha_eff * compressibility_factor, False
        # else:
        #     return self.cl_poly(np.degrees(alpha_eff)) * compressibility_factor, alpha_eff > self.alpha_stall

    def stall_maxWeight(self, climb_velocity, altitude):
        stall = 0
        temperature = self.environment.get_temperature(altitude)
        pressure = self.environment.get_density(temperature, altitude)
        density = self.environment.get_density(temperature, pressure)

        omega = 24 # To prevent shocks
        gross_weight = 0
        pitch_factor = 1
        d_pf = 0.1
        for _ in range(100):
          r: np.ndarray = np.linspace(self.root_cutout, self.radius_of_rotors, 10)
          delta_r: float = r[1] - r[0]

          dT_list: List[float] = []

          for i in range(len(r)):
              chord: float = self.get_chord_length(r[i])
              cl, stall, __ = self.get_cL_custom(r[i], climb_velocity=climb_velocity, omega=omega, pitch_factor=pitch_factor)
              cd: float = self.get_cD_custom(r[i], climb_velocity=climb_velocity, omega=omega, pitch_factor=pitch_factor)

              lambda_inflow: float
              lambda_inflow, _ = self.elemental_inflow_ratio(r[i], climb_velocity, omega)
              phi: float = self.get_phi_r(r[i], climb_velocity)
              
              if stall:
                  break
              
              else:
                velocity_squared: float = (omega * r[i]) * 2 + (lambda_inflow * omega * self.radius_of_rotors) * 2

                dL: float = 0.5 * density * chord * cl * velocity_squared
                dD: float = 0.5 * density * chord * cd * velocity_squared
                dT: float = (dL * np.cos(phi) - dD * np.sin(phi)) * delta_r

                dT_list.append(dT)


          if not stall:

            pitch_factor += d_pf

          if stall:
            gross_weight = np.sum(dT_list) * self.number_of_blades/self.environment.gravitational_acceleration
            break
        

        return gross_weight

        
        # dr = (self.radius_of_rotors - self.root_cutout) / divisions
        # for i in range(divisions):
        #     _, stalling = self.get_cL(r_distance= self.root_cutout + (i+0.5) * dr, climb_velocity=vertical_velocity, omega=omega)
        #     if stalling:
        #         return True
        # return False

    def find_max_thrust_before_stall(self, vertical_velocity: float, omega: float, altitude: float, initial_omega_guess: float = 40, iterations: int = 100) -> float:
        temperature = self.environment.get_temperature(altitude)
        pressure = self.environment.get_pressure(temperature, altitude)
        density = self.environment.get_density(temperature=temperature, pressure=pressure)
        omega = initial_omega_guess*0.9
        domega = initial_omega_guess*0.1
        prev_stalled = False
        for _ in range(iterations):
            omega += domega
            if(self.is_rotor_stalling(vertical_velocity, omega, divisions=30)):
                if(not prev_stalled):
                    domega *= -0.5
                prev_stalled = True
            else:
                if(prev_stalled):
                    domega *= -0.5
                prev_stalled = False
        thrust = self.total_thrust(vertical_velocity, omega-(1*abs(domega)), density)
        return thrust, omega-(1*abs(domega))

class Helicopter:
    """Aircraft Helicopter Parameters from the user"""
    def __init__(self, environment: Optional['Environment'] = None) -> None:
        self.environment: Optional['Environment'] = environment if environment else Environment()
        self.main_rotor: Optional['Rotor'] = None
        self.tail_rotor: Optional['Rotor'] = None
        self.fuselage_set: bool = False
        self.tail_set: bool = False
        self.engine_set: bool = False
        self.main_rotor_set: bool = False
        self.tail_rotor_set: bool = False
        #For now max omega is infinity. Will make it better later
        self.max_omega: float = float('inf')

        self.fuselage_mass: Optional[float] = None
        self.fuselage_length: Optional[float] = None
        self.fuselage_width: Optional[float] = None
        self.fuselage_height: Optional[float] = None
        self.tail_mass: Optional[float] = None
        self.tail_length: Optional[float] = None
        self.tail_width: Optional[float] = None
        self.tail_height: Optional[float] = None
        self.main_rotor_position: Optional[float] = None
        self.tail_rotor_position: Optional[float] = None


        self.engine_mass: Optional[float] = None
        self.engine_max_power: Optional[float] = None
        self.power_loss = 0.1
        self.engine_max_available_power = None

    def get_fuselage_parameters(self) -> None:
        # fuselage parameters
        self.fuselage_mass = float(input("Please enter the fuselage mass (kg): "))
        self.fuselage_length = float(input("Please enter the fuselage length (m): "))
        self.fuselage_width = float(input("Please enter the fuselage width (m): "))
        self.fuselage_height = float(input("Please enter the fuselage height (m): "))
        self.tail_mass = float(input("Please enter the tail mass (kg): "))
        self.tail_length = float(input("Please enter the tail length (m): "))
        self.tail_width = float(input("Please enter the tail width (m): "))
        self.tail_height = float(input("Please enter the tail height (m): "))
        self.fuselage_set = True
        self.tail_set = True


    def set_fuselage_parameters(self, mass: float, length: float, width: float,
                              height: float, tail_mass: float, tail_length: float,
                              tail_width: float, tail_height: float) -> None:
        self.fuselage_set = True
        self.tail_set = True
        self.fuselage_mass = mass
        self.fuselage_length = length
        self.fuselage_width = width
        self.fuselage_height = height
        self.tail_mass = tail_mass
        self.tail_length = tail_length
        self.tail_width = tail_width
        self.tail_height = tail_height
        

    def get_engine_parameters(self) -> None:
        # Aircraft engine parameters
        self.engine_set = True
        self.engine_mass = float(input("Please enter the mass of the engine (kg): "))
        self.engine_max_power = float(input("Please enter the maximum power of the engine (kW): "))
        self.engine_max_available_power = self.engine_max_power * (1 - self.power_loss)

    def set_engine_parameters(self, mass: float, max_power: float) -> None:
        self.engine_set = True
        self.engine_mass = mass
        self.engine_max_power = max_power
        self.engine_max_available_power = self.engine_max_power * (1 - self.power_loss)

    def setup_rotor(self) -> 'Rotor':
        if not self.main_rotor_set:
            self.main_rotor = Rotor(self.environment)
            print("Setting up main rotor...\n")
            sleep(1)
            self.main_rotor.get_rotor_parameters()
            self.main_rotor_position = float(input("Please enter the position of the main rotor from the nose (m): "))
            sleep(0.2)
            print("Setting up tail rotor...\n")
            sleep(1)
            self.tail_rotor = Rotor(self.environment)
            self.tail_rotor.get_rotor_parameters()
            self.tail_rotor_position = float(input("Please enter the position of the tail rotor from the nose (m): "))
            sleep(0.2)
            self.main_rotor_set = True
            self.tail_rotor_set = True
        return self.main_rotor, self.tail_rotor

    def set_rotor(self, rotor1: 'Rotor', rotor2: 'Rotor') -> None:
        self.main_rotor = rotor1
        self.tail_rotor = rotor2
        self.main_rotor_set = True
        self.tail_rotor_set = True

    def set_rotor_positions(self, main_rotor_position: float, tail_rotor_position: float) -> None:
        """Set rotor positions programmatically for testing"""
        self.main_rotor_position = main_rotor_position
        self.tail_rotor_position = tail_rotor_position

    def get_total_mass(self) -> float:
        total_mass: float = 0
        if self.fuselage_set:
            total_mass += self.fuselage_mass
        if self.engine_set:
            total_mass += self.engine_mass
        if self.main_rotor_set and self.main_rotor:
            total_mass += self.main_rotor.blade_mass * self.main_rotor.number_of_blades
        if self.tail_rotor_set and self.tail_rotor:
            total_mass += self.tail_rotor.blade_mass * self.tail_rotor.number_of_blades
        return total_mass

    def find_power_needed(self, weight: float, vertical_velocity: float, altitude: float, omega: float = None, initial_guess: float = 10.0) -> float:
        if omega is None:
            omega = self.main_rotor.find_omega_needed_uncoupled(thrust_needed=weight*self.environment.gravitational_acceleration, vertical_velocity=vertical_velocity, altitude=altitude, initial_guess=initial_guess)
        temperature = self.environment.get_temperature(altitude=altitude)
        density = self.environment.get_density(temperature=temperature)
        performance = self.main_rotor.calculate_performance(vertical_velocity, omega, density)

        tail_omega = self.tail_rotor.find_omega_needed_uncoupled(thrust_needed=performance['torque']/(self.tail_rotor_position - self.main_rotor_position), vertical_velocity=0, altitude=altitude, initial_guess=omega*np.sqrt(self.main_rotor.radius_of_rotors/self.tail_rotor.radius_of_rotors))
        tail_power = self.tail_rotor.total_power(climb_velocity=0, omega=tail_omega, density=density)
        return performance['power'] + tail_power

    def is_helicopter_stalling(self, vertical_velocity: float, altitude: float, omega: float = None) -> bool:
        temperature = self.environment.get_temperature(altitude)
        density = self.environment.get_density(temperature)
        performance = self.main_rotor.calculate_performance(vertical_velocity, omega, density)
        tail_omega = self.tail_rotor.find_omega_needed_uncoupled(thrust_needed=performance['torque']/(self.tail_rotor_position - self.main_rotor_position), vertical_velocity=0, altitude=altitude, initial_guess=omega*np.sqrt(self.main_rotor.radius_of_rotors/self.tail_rotor.radius_of_rotors))
        is_heli_stalling = self.main_rotor.is_rotor_stalling(vertical_velocity, omega) or self.tail_rotor.is_rotor_stalling(0, tail_omega)
        
        return is_heli_stalling

    

    def find_thrust_provided(self, vertical_velocity: float, altitude: float, omega: float = None) -> float:
        temperature: float = self.environment.get_temperature(altitude=altitude)
        pressure: float = self.environment.get_pressure(temperature)
        density: float = self.environment.get_density(temperature, pressure)

        # performance = self.main_rotor.calculate_performance(vertical_velocity, omega, density)
        thrust = self.main_rotor.total_thrust(vertical_velocity, omega, density)
        return thrust

    def find_fuselage_vertical_drag(self, vertical_velocity: float, altitude: float, omega: float = None) -> float:
        pass
        return 0.0
    

    def find_omega_needed(self, weight: float, vertical_velocity: float, altitude: float, initial_guess: float, iterations: int= 10, tol: float = 0.05) -> float:
        gravity = self.environment.gravitational_acceleration
        weight_force = weight * gravity

        omega = initial_guess
        for _ in range(iterations):
            thrust = self.find_thrust_provided(vertical_velocity, altitude, omega)
            fuselage_drag = self.find_fuselage_vertical_drag(vertical_velocity, altitude, omega)
            downward_force = weight_force + fuselage_drag
            if abs(thrust - downward_force) < tol:
                return omega
            d_fraction = (downward_force - thrust) / thrust
            omega *= (1 + d_fraction*0.5)
        if(abs(thrust - downward_force) > 10*tol):
            print("Warning: Omega calculation did not converge")
        return omega




class MissionPlanner:
    def __init__(self, heli: Helicopter):
        self.flight_parameters_set: bool = False
        self.helicopter: Helicopter = heli
        self.flight_path: list[int] = []
        self.max_omega = self.helicopter.max_omega
        
    def set_flight_parameters(self):
        self.flight_parameters_set = True
        self.dry_weight = float(input("Enter the dry weight of the helicopter in kg: "))
        self.fuel_weight = float(input("Enter the fuel weight of the helicopter in kg: "))
        self.fuel_specific_energy = 1000*float(input("Enter the fuel specific energy in kJ/kg: "))
        reserve_fuel_fraction  = float(input("Enter the reserve fuel fraction (e.g., 0.2 for 20%): "))
        self.reserved_fuel = self.fuel_weight * reserve_fuel_fraction
        self.fuel_weight -= self.reserved_fuel
        self.dry_weight += self.reserved_fuel
        # self.min_fuel = self.fuel_weight * reserve_fuel_fraction
        self.max_fuel = self.fuel_weight + self.reserved_fuel

    def set_flight_parameters_programmatic(self, dry_weight: float, fuel_weight: float, 
                                          fuel_specific_energy_kj_kg: float, reserve_fuel_fraction: float):
        """Set flight parameters programmatically for testing"""
        self.flight_parameters_set = True
        self.dry_weight = dry_weight
        self.fuel_weight = fuel_weight
        self.fuel_specific_energy = 1000 * fuel_specific_energy_kj_kg  # Convert kJ/kg to J/kg
        self.reserved_fuel = self.fuel_weight * reserve_fuel_fraction
        self.fuel_weight -= self.reserved_fuel
        self.dry_weight += self.reserved_fuel
        # self.min_fuel = self.fuel_weight * reserve_fuel_fraction
        self.max_fuel = self.fuel_weight + self.reserved_fuel
        
        
    # def set_flight_path(self):
    #     if not self.flight_parameters_set:
    #         print("Flight parameters not set. Please set them first.")
    #         return
    
    """
    
    If True, operation unsuccessfull.
    In that case first arguement will be the error code
    -2 -> Flight parameters not set
    -1 -> Flight path parameters not set
    0 -> Stall
    1 -> Max omega exceeded
    2 -> Max power exceeded
    
    """
    
    def climb_cost(self, starting_altitude, final_altitude, climb_rate, divisions: int = 500) -> Tuple[List[float], bool]:
        if not self.flight_parameters_set:
            print("Flight parameters not set. Please set them first.")
            return
        altitude = starting_altitude
        delta_t = (final_altitude - starting_altitude)/(climb_rate)
        stall_check_interval = divisions // 10  
        dt = delta_t / divisions 
        fuel_consumed_weight = 0.0
        stalling = False
        g = self.helicopter.environment.gravitational_acceleration
        fuel_specific_energy_inv = 1.0 / self.fuel_specific_energy
        for i in range(divisions):
            omega_needed = self.helicopter.main_rotor.find_omega_needed_uncoupled((self.dry_weight + self.fuel_weight-fuel_consumed_weight)*g, climb_rate, altitude=altitude, initial_guess=30.0)
            if(omega_needed>self.max_omega):
                return [1, True]
            if i % stall_check_interval == 0:
                stalling = self.helicopter.is_helicopter_stalling(climb_rate, altitude=altitude, omega=omega_needed)
                if stalling:
                    print(f"Helicopter is stalling at altitude {altitude:.2f} m. Climb aborted.")
                    return [0, True]
        
            
            
            power_needed = self.helicopter.find_power_needed(self.dry_weight + self.fuel_weight-fuel_consumed_weight, climb_rate, altitude=altitude)
            d_fuel_consumed = power_needed*dt * fuel_specific_energy_inv
            fuel_consumed_weight += d_fuel_consumed
            
            self.flight_path.append(altitude)
            altitude += climb_rate * dt
        return [fuel_consumed_weight, False]

    def hover_cost(self, duration: float, altitude: float, divisions: int = 500) -> Tuple[List[float], bool]:
        if not self.flight_parameters_set:
            print("Flight parameters not set. Please set them first.")
            return [0, True]

        dt = duration / divisions
        fuel_consumed = 0.0
        fuel_specific_energy_inv = 1.0 / self.fuel_specific_energy
        omega_needed = self.helicopter.main_rotor.find_omega_needed_uncoupled((self.dry_weight + self.fuel_weight)*self.helicopter.environment.gravitational_acceleration, 0, altitude=altitude, initial_guess=30.0)

        
        if(omega_needed>self.helicopter.max_omega):
            return [1, True]
        
        
        
        if(self.helicopter.is_helicopter_stalling(0, altitude=altitude, omega=omega_needed)):
            return [0, True]
        for _ in range(divisions):
            omega_needed = self.helicopter.main_rotor.find_omega_needed_uncoupled((self.dry_weight + self.fuel_weight)*self.helicopter.environment.gravitational_acceleration, 0, altitude=altitude, initial_guess=30.0)
            power_needed = self.helicopter.find_power_needed(self.dry_weight + self.fuel_weight - fuel_consumed, 0, altitude=altitude)
            fuel_consumed += power_needed * dt * fuel_specific_energy_inv
        return [fuel_consumed, False]
    
    def find_available_hover_time(self, altitude, dt = 0.1) -> Tuple[float, bool]:
        if not self.flight_parameters_set:
            print("Flight parameters not set. Please set them first.")
            return -2, True

        omega_needed = self.helicopter.main_rotor.find_omega_needed_uncoupled((self.dry_weight + self.fuel_weight)*self.helicopter.environment.gravitational_acceleration, 0, altitude=altitude, initial_guess=10.0)
        if(self.helicopter.is_helicopter_stalling(0, altitude=altitude, omega=omega_needed)):
            return 0, True
        
        time = 0.0
        fuel_remaining = self.fuel_weight
        fuel_specific_energy_inv = 1.0 / self.fuel_specific_energy
            
    
        
        while fuel_remaining > 0:
            omega_needed = self.helicopter.main_rotor.find_omega_needed_uncoupled((self.dry_weight + fuel_remaining)*self.helicopter.environment.gravitational_acceleration, 0, altitude=altitude, initial_guess=30.0)
            power_needed = self.helicopter.find_power_needed(self.dry_weight + fuel_remaining, 0, altitude=altitude, omega=omega_needed)
            fuel_remaining -= power_needed * dt * fuel_specific_energy_inv
            time += dt
        return time, False

    def find_max_hover_weight(self, altitude, stall_constraint:bool = False, power_constraint:bool = True, iterations:int = 200, initial_omega_guess:float = 30, initial_weight_guess: float = None, tol: float = 1e-1) -> float:
        if not self.flight_parameters_set:
            print("Flight parameters not set. Please set them first.")
            return [-2, True]

       # "Cannot have both stall and power constraints false simultaneously."
        assert (stall_constraint or power_constraint) is True, "Cannot have both stall and power constraints false simultaneously."
        power_max_weight = float('inf')
        stall_max_weight = float('inf')
        
    
        temperature = self.helicopter.environment.get_temperature(altitude)
        density = self.helicopter.environment.get_density(temperature)
        if(stall_constraint):
            _, max_main_rotor_omega = self.helicopter.main_rotor.find_max_thrust_before_stall(0, omega=initial_omega_guess, altitude=altitude, initial_omega_guess=initial_omega_guess, iterations=iterations)
            _ , max_tail_rotor_omega = self.helicopter.tail_rotor.find_max_thrust_before_stall(0, omega=initial_omega_guess*np.sqrt(self.helicopter.main_rotor.radius_of_rotors/self.helicopter.tail_rotor.radius_of_rotors), altitude=altitude, initial_omega_guess=initial_omega_guess*np.sqrt(self.helicopter.main_rotor.radius_of_rotors/self.helicopter.tail_rotor.radius_of_rotors), iterations=iterations)
            max_omega = min(max_main_rotor_omega, max_tail_rotor_omega)
            
            max_thrust = self.helicopter.main_rotor.total_thrust(climb_velocity=0, omega=max_omega, density=density)
            stall_max_weight = (max_thrust - self.helicopter.find_fuselage_vertical_drag(vertical_velocity=0, altitude=altitude, omega=max_omega))/ self.helicopter.environment.gravitational_acceleration
        weight = self.dry_weight*0.1
        d_weight = weight*0.5
        power_needed = self.helicopter.find_power_needed(weight=weight, vertical_velocity=0, altitude=altitude)
        prev_exceeded_power = False
        

        if(power_constraint):
            
            for _ in range(iterations):
                power_needed = self.helicopter.find_power_needed(weight=weight, vertical_velocity=0, altitude=altitude)
                if power_needed > self.helicopter.engine_max_available_power:
                    if(prev_exceeded_power is False):
                        d_weight *= 0.5
                    weight -= d_weight
                    prev_exceeded_power = True
                else:
                    if(prev_exceeded_power is True):
                        d_weight *= 0.5
                    weight += d_weight
                if(abs(power_needed - self.helicopter.engine_max_available_power) < tol):
                    break
            power_max_weight = weight  # This is the result of binary search
        return float(min(stall_max_weight, power_max_weight)), float(stall_max_weight), float(power_max_weight)
