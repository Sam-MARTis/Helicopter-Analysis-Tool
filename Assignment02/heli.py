from typing import Optional, Tuple, Dict, List, Union
import numpy as np
from time import sleep
from numba import njit



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
        if not self.environment_set:
            raise ValueError("Environment parameters not set. Please set them using 'set_atmosphere_parameters' method.")
        
        temperature = self.temperature_sea_level + self.ISA_OFFSET - self.Lapse_rate_troposphere * altitude
        pressure = self.pressure_sea_level *  np.pow((temperature / (self.temperature_sea_level + self.ISA_OFFSET)), (-self.gravitational_acceleration / (self.Lapse_rate_troposphere * self.Specific_gas_constant)))
        density = pressure / (self.Specific_gas_constant * temperature)
        return density
    
    def get_speed_of_sound(self, altitude:float) -> float:
        if not self.environment_set:
            raise ValueError("Environment parameters not set. Please set them using 'set_atmosphere_parameters' method.")

        temperature = self.temperature_sea_level + self.ISA_OFFSET - self.Lapse_rate_troposphere * altitude
        speed_of_sound = np.sqrt(self.specific_heat_constant * self.Specific_gas_constant * temperature)
        return speed_of_sound