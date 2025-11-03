from heli_improved import Rotor as HoverRotor
from heli_improved import Environment
from Rotor_vert_included import Rotor as ForwardFlightRotor
from Rotor import get_Parasitic_Drag, fuselage, trimSolve, deg_to_rad
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

drag = fuselage()
inv_fuel_specific_energy = 1/(43e6)
inv_engine_efficiency = 1.1 + 0/0.3
max_actual_power_available = 1600e3 * inv_engine_efficiency
SFC = 0.4
class Heli:
    def __init__(
        self,
        environment: Environment,
        altitude_initial: float,
        heli_dry_mass: float,
        payload_mass: float,
        fuel_mass: float,
        Ω_forward: float,
        rotor_mass,
        blade_count: int,
        R: float,
        rc: 0.1,
        chord_function,
        θtw: float,
        V_forward: float,
        r_divisions: int,
        ψ_divisions: int,
        Cd0: float,
        Cd_Clsq_slope: float,
        Hover_Pitch: float,
    ):
        self.environment = environment
        self.heli_dry_mass = heli_dry_mass
        self.fuel_mass = fuel_mass
        self.altitude = altitude_initial
        self.Ω_forward = Ω_forward
        self.hover_Ω = self.Ω_forward
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
        self.x0 = 0
        self.is_payload_attached = (payload_mass>0)
        self.rotorHover = HoverRotor(environment=environment)
        self.rotorHover.set_rotor_parameters(
            number_of_blades=blade_count,
            radius_of_rotors=R,
            root_cutout=rc,
            root_chord=chord_function(rc),
            tip_chord=chord_function(R),
            root_pitch=Hover_Pitch,
            slope_pitch=Hover_Pitch + θtw * (R - rc),
            default_airfoil=True,
        )
        self.rotorForward = ForwardFlightRotor(
            Ω=Ω_forward,
            rotor_mass=rotor_mass,
            blade_count=blade_count,
            R=R,
            rc=rc,
            chord_function=chord_function,
            θtw=θtw,
            ρ=self.ρ,
            V_infty=V_forward,
            r_divisions=r_divisions,
            ψ_divisions=ψ_divisions,
            Cd0=Cd0,
            Cd_Clsq_slope=Cd_Clsq_slope,
        )

    def forward(self, Velocity: float, Climb:float, dt: float):
        self.V_forward = Velocity
        self.ρ = self.environment.get_density(self.altitude)
        self.rotorForward.Vinfty = Velocity
        drag_value = drag.get_drag(self.ρ, self.V_forward)
        weight = (self.heli_dry_mass +  + self.fuel_mass) * 9.81
        thrust_needed = np.sqrt(weight**2 + drag_value**2)
        trims, trim_state_vals = trimSolve(rotor=self.rotorForward,  Ω=self.Ω_forward, θ0_initial=self.rotorForward.θ0, θ1s_initial=self.rotorForward.θ1s, θ1c_initial=self.rotorForward.θ1c, W=weight, D=drag_value, coning_angle_iterations=2, β0_step_fraction=1.00, iterations=10, relaxation=0.3, verbose=False)
        self.rotorForward.set_all_calculation_batch_properties(Thrust_Needed=thrust_needed, Vy=Climb, Ω=self.Ω_forward, θ0=trims[0], θ1s=trims[1], θ1c=trims[2])
        T, M, P = self.rotorForward.perform_all_calculations(W=weight, D=drag_value, coning_angle_iterations=2, β0_step_fraction=1.00)
        fuel_burn_rate = P * inv_engine_efficiency * inv_fuel_specific_energy
        fuel_burned = fuel_burn_rate * dt
        self.fuel_mass -= fuel_burned
        self.x0 += Velocity * dt
        self.altitude += Climb * dt
        return True
    
    def upward(self, Climb_Rate: float, dt: float):
        self.ρ = self.environment.get_density(self.altitude)
        weight = (self.heli_dry_mass + self.fuel_mass)

        [is_bound, omega] = self.rotorHover.find_omega_needed_for_thrust_uncoupled(target_thrust=weight, Vy=Climb_Rate, altitude=self.altitude, Ω_initial=5, Ω_step=1, Ω_max = 100, itermax=400)
        self.hover_Ω = omega
        vals = self.rotorHover.get_rotor_performance(Vy=Climb_Rate, Ω=self.hover_Ω, altitude=self.altitude, divisions=r_divisions)
        P = vals["power"]
        P = P * inv_engine_efficiency
        if(not is_bound):
            P = max_actual_power_available
            # return False
        self.altitude += Climb_Rate * dt
        return P
    
    


        
        

class MissionSegment:
    def __init__(self, heli:Heli, altitude: float, fuel_mass: float, payload_mass: float, person_mass: float, person_count: int):
        self.heli = heli
        self.heli.altitude = altitude
        
        self.heli_dry_mass = heli.heli_dry_mass
        self.fuel_mass = fuel_mass
        self.payload_mass = payload_mass
        self.person_mass = person_mass
        self.person_count = person_count
        self.heli.heli_dry_mass = self.heli_dry_mass + self.payload_mass + self.person_mass * self.person_count
    # def execute_forward_segment(self, Velocity: float, duration: float, dt: float):
    def executeAb(self):
        # self.fuel_mass -= 1
        dy = 2040 - 2000
        Vy = 10
        dt = dy / Vy
        P = self.heli.upward(Climb_Rate=Vy, dt=dt)
        P = P * inv_engine_efficiency
        fuel_burn_rate = (P / 1000) * SFC / 3600
        self.heli.fuel_mass -= fuel_burn_rate  * dt
        print("Initial Climb Segment: Power={:.0f} kW, Burnrate={:.25} kg/s".format(P / 1000, fuel_burn_rate))
    def executeAc(self):
        rho = self.heli.environment.get_density(self.heli.altitude)
        dy = 2500 - 2040
        # dt = 5*60
        Vy = 10
        dt = dy / Vy
        Vx = V_FORWARD
        weight = (self.heli.heli_dry_mass  + self.heli.fuel_mass) * 9.81
        drag_value = drag.get_drag(rho, Vx)
        thrust_needed = np.sqrt(weight**2 + drag_value**2)
        self.heli.rotorForward.Vy = Vy
        trims, trim_state_vals = trimSolve(rotor=self.heli.rotorForward,  Ω=self.heli.Ω_forward, θ0_initial=self.heli.rotorForward.θ0, θ1s_initial=self.heli.rotorForward.θ1s, θ1c_initial=self.heli.rotorForward.θ1c, W=weight, D=drag_value, coning_angle_iterations=2, β0_step_fraction=1.00, iterations=10, relaxation=0.3, verbose=False)
        self.heli.rotorForward.set_all_calculation_batch_properties(Thrust_Needed=thrust_needed, Vy=Vy, Ω=self.heli.Ω_forward, θ0=trims[0], θ1s=trims[1], θ1c=trims[2])
        T, M, P = self.heli.rotorForward.perform_all_calculations(W=weight, D=drag_value, coning_angle_iterations=2, β0_step_fraction=1.00)
        P = P* inv_engine_efficiency
        fuel_burn_rate =( P/1000) * SFC/3600
        print("Upward forward flight segment:")
        print("Burnrate: {:.5f} kg/s".format(fuel_burn_rate))
        fuel_burned = fuel_burn_rate * dt
        self.heli.fuel_mass -= fuel_burned
        print(f"Forward Flight Segment: Power={P/1000:.0f} kW, Fuel Burned={fuel_burned:.2f} kg")
        # self.heli.fuel_mass -= fuel_burned
        # self.heli.x0 += Vx * dt
    
    def executeAd(self):
        # self.heli.Ω = 
        self.heli.altitude = 2500
        Vx = 250*5/18
        rho = self.heli.environment.get_density(self.heli.altitude)
        Vy = 0
        dt = 5*60
        weight = (self.heli.heli_dry_mass  + self.heli.fuel_mass) * 9.81
        drag_value = drag.get_drag(rho, Vx)
        thrust_needed = np.sqrt(weight**2 + drag_value**2)
        self.heli.rotorForward.Vy = Vy
        self.heli.rotorForward.Vinfty = Vx
        trims, trim_state_vals = trimSolve(rotor=self.heli.rotorForward,  Ω=self.heli.Ω_forward, θ0_initial=self.heli.rotorForward.θ0, θ1s_initial=self.heli.rotorForward.θ1s, θ1c_initial=self.heli.rotorForward.θ1c, W=weight, D=drag_value, coning_angle_iterations=2, β0_step_fraction=1.00, iterations=10, relaxation=0.3, verbose=False)
        self.heli.rotorForward.set_all_calculation_batch_properties(Thrust_Needed=thrust_needed, Vy=Vy, Ω=self.heli.Ω_forward, θ0=trims[0], θ1s=trims[1], θ1c=trims[2])
        T, M, P = self.heli.rotorForward.perform_all_calculations(W=weight, D=drag_value, coning_angle_iterations=2, β0_step_fraction=1.00)
        P = P* inv_engine_efficiency
        fuel_burn_rate =( P/1000) * SFC/3600
        print("Segment 2: Steady forward flight at 2500m altitude")
        print("Burnrate: {:.5f} kg/s".format(fuel_burn_rate))
        fuel_burned = fuel_burn_rate * dt
        self.heli.fuel_mass -= fuel_burned
        print(f"Forward Flight Segment: Power={P/1000:.0f} kW, Fuel Burned={fuel_burned:.2f} kg")
    
    
    def executeAe(self):
        self.heli.fuel_mass -= 70
        print("AE heli fuel mass: ", self.heli.fuel_mass)
        
        rho = self.heli.environment.get_density(self.heli.altitude)
        dy = 2500 - 2100
        # dt = 5*60
        Vy = -10
        dt = abs(dy / Vy)
        Vx = V_FORWARD
        weight = (self.heli.heli_dry_mass  + self.heli.fuel_mass) * 9.81
        drag_value = drag.get_drag(rho, Vx)
        thrust_needed = np.sqrt(weight**2 + drag_value**2)
        self.heli.rotorForward.Vy = Vy
        trims, trim_state_vals = trimSolve(rotor=self.heli.rotorForward,  Ω=self.heli.Ω_forward, θ0_initial=self.heli.rotorForward.θ0, θ1s_initial=self.heli.rotorForward.θ1s, θ1c_initial=self.heli.rotorForward.θ1c, W=weight, D=drag_value, coning_angle_iterations=2, β0_step_fraction=1.00, iterations=10, relaxation=0.3, verbose=False)
        self.heli.rotorForward.set_all_calculation_batch_properties(Thrust_Needed=thrust_needed, Vy=Vy, Ω=self.heli.Ω_forward, θ0=trims[0], θ1s=trims[1], θ1c=trims[2])
        T, M, P = self.heli.rotorForward.perform_all_calculations(W=weight, D=drag_value, coning_angle_iterations=2, β0_step_fraction=1.00)
        P = abs(P* inv_engine_efficiency)
        fuel_burn_rate =abs(( P/1000) * SFC/3600)
        print("Upward forward flight segment:")
        print("Burnrate: {:.5f} kg/s".format(fuel_burn_rate))
        fuel_burned = fuel_burn_rate * dt
        self.heli.fuel_mass -= fuel_burned
        print(f"Forward Flight Segment: Power={P/1000:.0f} kW, Fuel Burned={fuel_burned:.2f} kg")
    def executeAf(self):
        self.heli.altitude = 2400
        dt = 10*60
        P = self.heli.upward(Climb_Rate=0, dt=dt)
        P = P * inv_engine_efficiency
        fuel_burn_rate = (P / 1000) * SFC / 3600
        self.heli.fuel_mass -= fuel_burn_rate  * dt
        print("Initial Climb Segment: Power={:.0f} kW, Burnrate={:.25} kg/s".format(P / 1000, fuel_burn_rate))
        
    def executeAg(self): 
        self.heli.altitude = 2400
        dt = 2
        Vy = -10
        P = self.heli.upward(Climb_Rate=Vy, dt=dt)
        P = P * inv_engine_efficiency
        fuel_burn_rate = (P / 1000) * SFC / 3600
        self.heli.fuel_mass -= fuel_burn_rate  * dt
        print("Initial Climb Segment: Power={:.0f} kW, Burnrate={:.25} kg/s".format(P / 1000, fuel_burn_rate))
        
        
    def executeAh(self):
        self.heli.heli_dry_mass -= self.person_mass * (self.person_count -2)
        
    def executeAi(self):
        self.heli.altitude = 2380
        dt = 2
        Vy = 10
        P = self.heli.upward(Climb_Rate=Vy, dt=dt)
        P = P * inv_engine_efficiency
        fuel_burn_rate = (P / 1000) * SFC / 3600
        self.heli.fuel_mass -= fuel_burn_rate  * dt
        print("Initial Climb Segment: Power={:.0f} kW, Burnrate={:.25} kg/s".format(P / 1000, fuel_burn_rate))
        
        
    def executeAj(self):
        self.fuel_mass = 122
        rho = self.heli.environment.get_density(self.heli.altitude)
        dy = 2600 - 2400
        # dt = 5*60
        Vy = 10
        dt = dy / Vy
        Vx = V_FORWARD
        weight = (self.heli.heli_dry_mass  + self.heli.fuel_mass) * 9.81
        drag_value = drag.get_drag(rho, Vx)
        thrust_needed = np.sqrt(weight**2 + drag_value**2)
        self.heli.rotorForward.Vy = Vy
        trims, trim_state_vals = trimSolve(rotor=self.heli.rotorForward,  Ω=self.heli.Ω_forward, θ0_initial=self.heli.rotorForward.θ0, θ1s_initial=self.heli.rotorForward.θ1s, θ1c_initial=self.heli.rotorForward.θ1c, W=weight, D=drag_value, coning_angle_iterations=2, β0_step_fraction=1.00, iterations=10, relaxation=0.3, verbose=False)
        self.heli.rotorForward.set_all_calculation_batch_properties(Thrust_Needed=thrust_needed, Vy=Vy, Ω=self.heli.Ω_forward, θ0=trims[0], θ1s=trims[1], θ1c=trims[2])
        T, M, P = self.heli.rotorForward.perform_all_calculations(W=weight, D=drag_value, coning_angle_iterations=2, β0_step_fraction=1.00)
        P = P* inv_engine_efficiency
        fuel_burn_rate =( P/1000) * SFC/3600
        print("Upward forward flight segment:")
        print("Burnrate: {:.5f} kg/s".format(fuel_burn_rate))
        fuel_burned = fuel_burn_rate * dt
        self.heli.fuel_mass -= fuel_burned
        print(f"Forward Flight Segment: Power={P/1000:.0f} kW, Fuel Burned={fuel_burned:.2f} kg")
    
    def executeAk(self):
        # self.heli.Ω = 
        self.heli.altitude = 2600
        Vx = 350*5/18
        rho = self.heli.environment.get_density(self.heli.altitude)
        Vy = 0
        dt = 5*60
        weight = (self.heli.heli_dry_mass  + self.heli.fuel_mass) * 9.81
        f = 0.1 # High velocity compensator
        drag_value = drag.get_drag(rho, Vx) *f
        thrust_needed = np.sqrt(weight**2 + drag_value**2)
        self.heli.rotorForward.Vy = Vy
        self.heli.rotorForward.Vinfty = Vx
        trims, trim_state_vals = trimSolve(rotor=self.heli.rotorForward,  Ω=self.heli.Ω_forward, θ0_initial=self.heli.rotorForward.θ0, θ1s_initial=self.heli.rotorForward.θ1s, θ1c_initial=self.heli.rotorForward.θ1c, W=weight, D=drag_value, coning_angle_iterations=2, β0_step_fraction=1.00, iterations=10, relaxation=0.3, verbose=False)
        self.heli.rotorForward.set_all_calculation_batch_properties(Thrust_Needed=thrust_needed, Vy=Vy, Ω=self.heli.Ω_forward, θ0=trims[0], θ1s=trims[1], θ1c=trims[2])
        T, M, P = self.heli.rotorForward.perform_all_calculations(W=weight, D=drag_value, coning_angle_iterations=2, β0_step_fraction=1.00)
        P = P* inv_engine_efficiency
        fuel_burn_rate =( P/1000) * SFC/9600
        print("Segment 2: Steady forward flight at 2500m altitude")
        print("Burnrate: {:.5f} kg/s".format(fuel_burn_rate))
        fuel_burned = fuel_burn_rate * dt
        self.heli.fuel_mass -= fuel_burned
        print(f"Forward Flight Segment: Power={P/1000:.0f} kW, Fuel Burned={fuel_burned:.2f} kg")
    
    def executeAl(self):
        print("AE heli fuel mass: ", self.heli.fuel_mass)
        self.heli.altitude = 2600
        rho = self.heli.environment.get_density(self.heli.altitude)
        dy = 2600 - 2020
        # dt = 5*60
        Vy = -10
        dt = abs(dy / Vy)
        Vx = V_FORWARD
        weight = (self.heli.heli_dry_mass  + self.heli.fuel_mass) * 9.81
        drag_value = drag.get_drag(rho, Vx)
        thrust_needed = np.sqrt(weight**2 + drag_value**2)
        self.heli.rotorForward.Vy = Vy
        trims, trim_state_vals = trimSolve(rotor=self.heli.rotorForward,  Ω=self.heli.Ω_forward, θ0_initial=self.heli.rotorForward.θ0, θ1s_initial=self.heli.rotorForward.θ1s, θ1c_initial=self.heli.rotorForward.θ1c, W=weight, D=drag_value, coning_angle_iterations=2, β0_step_fraction=1.00, iterations=10, relaxation=0.3, verbose=False)
        self.heli.rotorForward.set_all_calculation_batch_properties(Thrust_Needed=thrust_needed, Vy=Vy, Ω=self.heli.Ω_forward, θ0=trims[0], θ1s=trims[1], θ1c=trims[2])
        T, M, P = self.heli.rotorForward.perform_all_calculations(W=weight, D=drag_value, coning_angle_iterations=2, β0_step_fraction=1.00)
        P = abs(P* inv_engine_efficiency)
        fuel_burn_rate =abs(( P/1000) * SFC/3600)
        print("Upward forward flight segment:")
        print("Burnrate: {:.5f} kg/s".format(fuel_burn_rate))
        fuel_burned = fuel_burn_rate * dt
        self.heli.fuel_mass -= fuel_burned
        print(f"Forward Flight Segment: Power={P/1000:.0f} kW, Fuel Burned={fuel_burned:.2f} kg")

if __name__ == "__main__":
    # Ω_FORWARD = 30
    # ROTOR_MASS = 130
    # BLADE_COUNT = 4
    # R = 5
    # RC = 0.1
    # chord_function = lambda r: 0.5
    # θtw = np.radians(1)
    # ρ_sea = 1.225
    # V_FORWARD = 150 * 5/18
    # r_divisions = 50
    # ψ_divisions = 50
    # Cd0 = 0.01
    # Cd_Clsq_slope = 0.02
    # T_sea_level = 288.15
    # Reynolds_number = 1e6
    # P_sea_level = 101325
    # Lapse_rate_troposphere = 0.0065
    # wind_velocity = 0
    # ISA_OFFSET = 0
    # initial_altitude = 0
    # HOVER_θ = np.radians(20)

    # HELI_DRY_MASS = 5000
    # PAYLOAD_MASS = 500*0
    # FUEL_MASS = 1000
    # g = 9.81

    """
    rc = 0.3
    R = 6.3
    chord_function = lambda r: 0.5
    theta_tw = 3 -> 0 degrees
    rotor_mass = 130kg
    mass_heli = 5000kg
    fuel = 1000kg
    person_mass = 1000kg
    blade_count = 4
    V_forward = 50 kmph
    Cd0 = 0.01
    Cd_Clsq_slope = 0.02
    payload = 0
    omega = 50
    rho = 0.96
    
    """
    Ω_FORWARD = 20
    ROTOR_MASS = 130
    BLADE_COUNT = 4
    R = 6.3
    RC = 0.3
    chord_function = lambda r: 0.5
    θtw = np.radians(-3/(R - RC))
    ρ_sea = 1.225
    V_FORWARD = 50 * 5/18
    r_divisions = 50
    ψ_divisions = 50
    Cd0 = 0.01
    Cd_Clsq_slope = 0.02
    T_sea_level = 288.15
    Reynolds_number = 1e6
    P_sea_level = 101325
    Lapse_rate_troposphere = 0.0065
    wind_velocity = 0
    ISA_OFFSET = 0
    initial_altitude = 0
    HOVER_θ = np.radians(20)

    HELI_DRY_MASS = 5000
    PAYLOAD_MASS = 500*0
    FUEL_MASS = 1000
    g = 9.81


    env = Environment()
    env.set_atmosphere_parameters(
        temperature_sea_level=T_sea_level,
        Reynolds_number=Reynolds_number,
        pressure_sea_level=P_sea_level,
        Lapse_rate_troposphere=Lapse_rate_troposphere,
        wind_velocity=wind_velocity,
        ISA_OFFSET=ISA_OFFSET,
    )
    rotor_hover = HoverRotor(environment=env)

    rotor_hover.set_rotor_parameters(
        number_of_blades=BLADE_COUNT,
        radius_of_rotors=R,
        root_cutout=RC,
        root_chord=chord_function(RC),
        tip_chord=chord_function(R),
        root_pitch=HOVER_θ,
        slope_pitch=HOVER_θ + θtw * (R - RC),
        default_airfoil=True,
    )

    rotor_forward = ForwardFlightRotor(
        Ω=Ω_FORWARD,
        rotor_mass=ROTOR_MASS,
        blade_count=BLADE_COUNT,
        R=R,
        rc=RC,
        chord_function=chord_function,
        θtw=θtw,
        ρ=ρ_sea,
        V_infty=V_FORWARD,
        r_divisions=r_divisions,
        ψ_divisions=ψ_divisions,
        Cd0=Cd0,
        Cd_Clsq_slope=Cd_Clsq_slope,
    )
    heli = Heli(
        environment=env,
        altitude_initial=initial_altitude,
        heli_dry_mass=HELI_DRY_MASS,
        payload_mass=PAYLOAD_MASS,
        fuel_mass=FUEL_MASS,
        Ω_forward=Ω_FORWARD,
        rotor_mass=ROTOR_MASS,
        blade_count=BLADE_COUNT,
        R=R,
        rc=RC,
        chord_function=chord_function,
        θtw=θtw,
        V_forward=V_FORWARD,
        r_divisions=r_divisions,
        ψ_divisions=ψ_divisions,
        Cd0=Cd0,
        Cd_Clsq_slope=Cd_Clsq_slope,
        Hover_Pitch=HOVER_θ,
    )
    segment = MissionSegment(
        heli=heli,
        altitude=initial_altitude,
        fuel_mass=FUEL_MASS,
        payload_mass=PAYLOAD_MASS,
        person_mass=100,
        person_count=10,
    )
    segment.executeAb()
    segment.executeAc()
    segment.executeAd()
    segment.executeAe()
    segment.executeAf()
    segment.executeAg()
    segment.executeAh()
    segment.executeAi()
    segment.executeAj()
    segment.executeAk()
    segment.executeAl()
    
    
    

    # rotor.plot_effective_aoa_vs_r(Vy=30, Ω=100, altitude=1000, divisions=1000)
    # rotor.plot_effective_aoa_vs_r(Vy=30, Ω=100, altitude=1000, divisions=1000)
