from typing import List


class Helicopter:
    def __init__(self):
        self.altitude = 0.0
        self.omega = 0.0
        self.max_omega = 100.0
        pass
        

    def find_power_needed(self, weight: float, vertical_velocity: float, altitude: float, omega: float = None) -> float:
        omega = self.omega if omega is None else omega
        pass
        return 0.0

    def is_helicopter_stalling(self, vertical_velocity: float, altitude: float, omega: float = None) -> bool:
        omega = self.omega if omega is None else omega
        pass
        return False
    
    def find_thrust_provided(self, vertical_velocity: float, altitude: float, omega: float = None) -> float:
        omega = self.omega if omega is None else omega
        pass
        return 0.0

    def find_fuselage_vertical_drag(self, vertical_velocity: float, altitude: float, omega: float = None) -> float:
        omega = self.omega if omega is None else omega
        pass
        return 0.0
    
    def find_omega_needed(self, weight: float, vertical_velocity: float, altitude: float) -> float:
        pass
        return 0.0

class MissionPlanner:
    def __init__(self, heli: Helicopter):
        self.flight_parameters_set: bool = False
        self.helicopter = heli
        self.flight_path: list[int] = []
        self.max_omega = self.helicopter.max_omega
        
    def set_flight_parameters(self):
        self.flight_parameters_set = True
        self.dry_weight = float(input("Enter the dry weight of the helicopter in kg: "))
        self.fuel_weight = float(input("Enter the fuel weight of the helicopter in kg: "))
        self.fuel_specific_energy = 1000*float(input("Enter the fuel specific energy in kJ/kg: "))
        reserve_fuel_fraction  = float(input("Enter the reserve fuel fraction (e.g., 0.2 for 20%): "))
        self.min_fuel = self.fuel_weight * reserve_fuel_fraction
        self.max_fuel = self.fuel_weight
        
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
    
    def climb_cost(self, starting_altitude, final_altitude, climb_rate, divisions: int = 500) -> List[float, bool]:
        if not self.flight_parameters_set:
            print("Flight parameters not set. Please set them first.")
            return
        altitude = starting_altitude
        delta_t = (final_altitude - starting_altitude)/(climb_rate)
        stall_check_interval = divisions // 10  
        dt = delta_t / divisions 
        fuel_consumed_weight = 0.0
        stalling = False
        fuel_specific_energy_inv = 1.0 / self.fuel_specific_energy
        for i in range(divisions):
            omega_needed = self.helicopter.find_omega_needed(self.dry_weight + self.fuel_weight-fuel_consumed_weight, climb_rate, altitude=altitude)
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

    def hover_cost(self, duration: float, altitude: float, divisions: int = 500) -> List[float, bool]:
        if not self.flight_parameters_set:
            print("Flight parameters not set. Please set them first.")
            return [0, True]

        dt = duration / divisions
        fuel_consumed = 0.0
        fuel_specific_energy_inv = 1.0 / self.fuel_specific_energy
        omega_needed = self.helicopter.find_omega_needed(self.dry_weight + self.fuel_weight, 0, altitude=altitude)
        
        if(omega_needed>self.helicopter.max_omega):
            return [1, True]
        
        
        
        if(self.helicopter.is_helicopter_stalling(0, altitude=altitude, omega=omega_needed)):
            return [0, True]
        for i in range(divisions):
            omega_needed = self.helicopter.find_omega_needed(self.dry_weight + self.fuel_weight, 0, altitude=altitude)
            power_needed = self.helicopter.find_power_needed(self.dry_weight + self.fuel_weight - fuel_consumed, 0, altitude=altitude)
            fuel_consumed += power_needed * dt * fuel_specific_energy_inv
        return [fuel_consumed, False]
    
    def find_available_hover_time(self, altitude, dt = 0.1) -> List[float, bool]:
        if not self.flight_parameters_set:
            print("Flight parameters not set. Please set them first.")
            return [-2, True]

        omega_needed = self.helicopter.find_omega_needed(self.dry_weight + self.fuel_weight, 0, altitude=altitude)
        if(self.helicopter.is_helicopter_stalling(0, altitude=altitude, omega=omega_needed)):
            return [0, True]
        
        time = 0.0
        fuel_remaining = self.fuel_weight - self.min_fuel
        fuel_specific_energy_inv = 1.0 / self.fuel_specific_energy
            
    
        
        while fuel_remaining > 0:
            omega_needed = self.helicopter.find_omega_needed(self.dry_weight + fuel_remaining, 0, altitude=altitude)
            power_needed = self.helicopter.find_power_needed(self.dry_weight + fuel_remaining, 0, altitude=altitude, omega=omega_needed)
            fuel_remaining -= power_needed * dt * fuel_specific_energy_inv
            time += dt
        return [time, False]
    
    
    
    

