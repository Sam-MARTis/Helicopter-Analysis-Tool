from heli_improved import Rotor as HoverRotor
from heli_improved import Environment
from Rotor import Rotor as ForwardFlightRotor
import numpy as np
import matplotlib.pyplot as plt
from typing import Optional, Tuple, Dict, List, Union
import numpy as np
from time import sleep
from numba import njit, prange
import matplotlib.pyplot as plt
from numpy import sin, cos, tan, sqrt, atan2, atan
from scipy.optimize import minimize
import multiprocessing as mp
from concurrent.futures import ThreadPoolExecutor, ProcessPoolExecutor
import copy
import pickle




class Heli:
    def __init__(self, environment: Environment, altitude_initial:float, Ω_forward: float, rotor_mass, blade_count: int, R:float, rc:0.1, chord_function, θtw:float, V_forward:float, r_divisions:int, ψ_divisions:int, Cd0:float, Cd_Clsq_slope:float, Hover_Pitch:float):
        self.environment = environment
        self.altitude_initial = altitude_initial
        self.Ω_forward = Ω_forward
        self.rotor_mass = rotor_mass
        self.blade_count = blade_count
        self.R = R
        self.rc = rc
        self.chord_function = chord_function
        self.θtw = θtw
        self.ρ = environment.get_density(altitude_initial)
        self.V_forward = V_forward
        self.r_divisions = r_divisions
        self.ψ_divisions = ψ_divisions
        self.Cd0 = Cd0
        self.Cd_Clsq_slope = Cd_Clsq_slope
        self.Hover_Pitch = Hover_Pitch

        self.rotorHover = HoverRotor(environment=environment)
        self.rotorHover.set_rotor_parameters(number_of_blades=blade_count, radius_of_rotors=R, root_cutout=rc, root_chord=chord_function(rc),
                              tip_chord=chord_function(R), root_pitch=Hover_Pitch, slope_pitch=Hover_Pitch + θtw*(R-rc), default_airfoil=True)
        self.rotorForward = ForwardFlightRotor(Ω=Ω_forward, rotor_mass=rotor_mass, blade_count=blade_count, R=R, rc=rc, chord_function= chord_function, θtw = θtw, ρ=self.ρ, V_forward=V_forward, r_divisions=r_divisions, ψ_divisions=ψ_divisions, Cd0=Cd0, Cd_Clsq_slope=Cd_Clsq_slope )
            


    # def add_rotor(self, rotor: Union[HoverRotor, ForwardFlightRotor]):
    #     self.rotors.append(rotor)

    # def set_environment(self, environment: Environment):
    #     self.environment = environment
    
if __name__ == "__main__":
    Ω_FORWARD = 30
    ROTOR_MASS = 150
    BLADE_COUNT = 4
    R = 5
    RC = 0.1
    chord_function = lambda r: 0.5
    θtw = np.radians(2)
    ρ_sea = 1.225
    V_FORWARD = 30
    r_divisions = 50
    ψ_divisions = 50
    Cd0 = 0.0133
    Cd_Clsq_slope = 0.037
    T_sea_level = 288.15
    Reynolds_number = 1e6
    P_sea_level = 101325
    Lapse_rate_troposphere = 0.0065
    wind_velocity = 0
    ISA_OFFSET = 0
    initial_altitude = 0
    
    HOVER_θ = np.radians(20)

    
    env = Environment()
    env.set_atmosphere_parameters(temperature_sea_level=T_sea_level, Reynolds_number=Reynolds_number, pressure_sea_level=P_sea_level,
                                  Lapse_rate_troposphere=Lapse_rate_troposphere, wind_velocity=wind_velocity, ISA_OFFSET=ISA_OFFSET)
    rotor_hover = HoverRotor(environment=env)
    
    rotor_hover.set_rotor_parameters(number_of_blades=BLADE_COUNT, radius_of_rotors=R, root_cutout=RC, root_chord=chord_function(RC),
                              tip_chord=chord_function(R), root_pitch=HOVER_θ, slope_pitch=HOVER_θ + θtw*(R-RC), default_airfoil=True)

    rotor_forward = ForwardFlightRotor(Ω=Ω_FORWARD, rotor_mass=ROTOR_MASS, blade_count=BLADE_COUNT, R=R, rc=RC, chord_function= chord_function, θtw = θtw, ρ=ρ_sea, V_forward=V_FORWARD, r_divisions=r_divisions, ψ_divisions=ψ_divisions, Cd0=Cd0, Cd_Clsq_slope=Cd_Clsq_slope )
 


    # rotor.plot_effective_aoa_vs_r(Vy=30, Ω=100, altitude=1000, divisions=1000)
    # rotor.plot_effective_aoa_vs_r(Vy=30, Ω=100, altitude=1000, divisions=1000)
