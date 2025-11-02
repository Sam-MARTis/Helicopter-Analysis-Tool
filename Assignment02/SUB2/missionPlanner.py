from typing import Optional, Tuple, Dict, List, Union
import numpy as np
from time import sleep
from numba import njit
import matplotlib.pyplot as plt
from numpy import sin, cos, tan, sqrt, atan2, atan
from scipy.optimize import minimize
import multiprocessing as mp
from concurrent.futures import ThreadPoolExecutor, ProcessPoolExecutor
import copy
import pickle
from Rotor import Rotor, trimSolve, get_Thrust_Total, get_Parasitic_Drag


MAX_CPU_WORKERS = 18
deg_to_rad = np.pi / 180
θ_epsilon = 0.001 * deg_to_rad
thrust_tolerance = 100.0 
moment_tolerance = 200.0 
θtw = -0 * deg_to_rad
g = 9.81

NUMBER_OF_MAIN_ROTORS = 2
NUMBER_OF_ENGINES = 2
MASS_OF_CHINOOK = 5000 
FUEL_WEIGHT = 500
ENGINE_EFFICIENCY = 0.25
RHO = 1.0  
Ω = 20
R = 8

MAX_POWER = 7 * 1e6  
prev_theta0 = None
prev_theta1s = None
prev_theta1c = None

class MissionPlanner:
    def __init__(self, rotor, Ω, f, fuelweight, dryweight, fuel_specific_energy, fuel_reserve_fraction = 0.2):
        self.ρ = 1.225 
        self.f = f
        self.fuelweight = fuelweight
        self.dryweight = dryweight
        self.fuel_specific_energy = fuel_specific_energy  
        self.fuel_reserve_fraction = fuel_reserve_fraction
        self.rotor = rotor
        self.Ω = Ω
        
    def copy(self):
        """Create a deep copy of the MissionPlanner instance for multiprocessing"""
        new_mp = MissionPlanner(
            f=self.f,
            fuelweight=self.fuelweight,
            dryweight=self.dryweight,
            fuel_specific_energy=self.fuel_specific_energy,
            fuel_reserve_fraction=self.fuel_reserve_fraction
        )
        
        new_mp.ρ = self.ρ
        return new_mp
        

    def get_max_range_speed(self, weight, dV = 10, V_start = 10, V_end = 150) -> Tuple[float, float]:
        best_v_by_power = 0
        best_speed = V_start
        V_by_power = {}
        for V in np.arange(V_start, V_end, dV):
            D = get_Parasitic_Drag(ρ=self.ρ, f=self.f, Vinfty=V)
            T_Needed = get_Thrust_Total(W=weight*g, D=D)
            
            try:
                _, trim_state_vals = trimSolve(rotor=self.rotor, Ω=self.Ω, θ0_initial=5.015*deg_to_rad, θ1s_initial=-4.02*deg_to_rad, θ1c_initial=3*deg_to_rad, W=weight*g, D=D, coning_angle_iterations=2, β0_step_fraction=1.00, iterations=10, relaxation=0.3, verbose=False)
                trim_power = trim_state_vals[2]
                if trim_power > 0:  
                    V_by_power[V] = V/trim_power
            except Exception as e:
                print(f"Warning: Trim solve failed at V={V} m/s: {e}")
                continue
        
        
        for V, val in V_by_power.items():
            if val > best_v_by_power:
                best_v_by_power = val
                best_speed = V
        return best_speed, best_v_by_power
    
    def get_max_range(self, dV, V_start, V_end, dt=600):
        fuel_weight_available = self.fuelweight*(1 - self.fuel_reserve_fraction)
        weight = self.dryweight + self.fuelweight
        total_distance = 0
        time_elapsed = 0
        iteration = 0
        max_iterations = int(fuel_weight_available * self.fuel_specific_energy / (50000 * dt)) + 10 
        
        while fuel_weight_available > 0 and iteration < max_iterations:
            print(f"Iteration {iteration}: Weight={weight:.1f} kg, Fuel left={fuel_weight_available:.1f} kg")
            try:
                best_speed, best_v_by_power = self.get_max_range_speed(weight=weight, dV=dV, V_start=V_start, V_end=V_end)
                if best_speed <= 0:
                    print("Warning: Invalid speed in range calculation")
                    break
                if best_v_by_power <= 0:
                    print("Warning: Invalid efficiency in range calculation")
                    break
                    
                power_consumed = best_speed / best_v_by_power
                fuel_consumed = power_consumed * dt / (self.fuel_specific_energy * ENGINE_EFFICIENCY)
                
                
                if fuel_consumed > fuel_weight_available:
                    
                    partial_dt = fuel_weight_available * self.fuel_specific_energy / power_consumed
                    total_distance += best_speed * partial_dt
                    time_elapsed += partial_dt
                    break
                
                total_distance += best_speed * dt
                fuel_weight_available -= fuel_consumed
                weight -= fuel_consumed
                time_elapsed += dt
                iteration += 1
                
            except Exception as e:
                print(f"Error in range calculation at iteration {iteration}: {e}")
                break
        
        return total_distance, time_elapsed
    
    def get_max_endurance_speed(self, weight, dV = 5, V_start = 10, V_end = 100) -> Tuple[float, float]:
        least_power = float('inf')
        best_speed = V_start
        V_power = {}
        for V in np.arange(V_start, V_end, dV):
            D = get_Parasitic_Drag(ρ=self.ρ, f=self.f, Vinfty=V)
            T_Needed = get_Thrust_Total(W=weight*g, D=D)
            
            try:
                _, trim_state_vals = trimSolve(rotor=self.rotor,  Ω=self.Ω, θ0_initial=5.015*deg_to_rad, θ1s_initial=-4.02*deg_to_rad, θ1c_initial=3*deg_to_rad, W=weight*g, D=D, coning_angle_iterations=2, β0_step_fraction=1.00, iterations=10, relaxation=0.3, verbose=False)
                trim_power = trim_state_vals[2]
                if trim_power > 0:  
                    V_power[V] = trim_power
                    if trim_power < least_power:
                        least_power = trim_power
                        best_speed = V
            except Exception as e:
                print(f"Warning: Trim solve failed at V={V} m/s: {e}")
                continue
        return best_speed, least_power
    
    def get_endurance(self, dV, V_start, V_end, dt=600):
        fuel_weight_available = self.fuelweight*(1 - self.fuel_reserve_fraction)
        weight = self.dryweight + self.fuelweight
        total_time = 0
        iteration = 0
        max_iterations = int(fuel_weight_available * self.fuel_specific_energy / (50000 * dt)) + 100  
        
        while fuel_weight_available > 0 and iteration < max_iterations:
            print(f"Iteration {iteration}: Weight={weight:.1f} kg, Fuel left={fuel_weight_available:.1f} kg")
            try:
                best_speed, least_power = self.get_max_endurance_speed(weight=weight, dV=dV, V_start=V_start, V_end=V_end)
                
                if least_power <= 0:
                    print("Warning: Invalid power in endurance calculation")
                    break

                fuel_consumed = (least_power * dt / self.fuel_specific_energy) / ENGINE_EFFICIENCY

                if fuel_consumed > fuel_weight_available:
                    
                    partial_dt = fuel_weight_available * self.fuel_specific_energy / least_power
                    total_time += partial_dt
                    break
                
                total_time += dt
                fuel_weight_available -= fuel_consumed
                weight -= fuel_consumed
                iteration += 1
                
            except Exception as e:
                print(f"Error in endurance calculation at iteration {iteration}: {e}")
                break
        
        return total_time
   
    def get_trim_conditions(self, V, iterations = 20, relaxation = 0.6):
        D = get_Parasitic_Drag(ρ=self.ρ, f=self.f, Vinfty=V)
        T_Needed = get_Thrust_Total(W=(self.dryweight + self.fuelweight)*g, D=D)
        trim_vals, trim_state_vals = trimSolve(rotor=self.rotor,  Ω=self.Ω, θ0_initial=5.015*deg_to_rad, θ1s_initial=-4.02*deg_to_rad, θ1c_initial=3*deg_to_rad, W=(self.dryweight + self.fuelweight)*g, D=D, coning_angle_iterations=2, β0_step_fraction=1.00, iterations=iterations, relaxation=relaxation, verbose=False)
        return trim_vals, trim_state_vals
    




def test_endurance_and_range(mp1:MissionPlanner):
    
    import time
    start_time = time.time()
    range_m, time_s = mp1.get_max_range(dV=80, V_start=10, V_end=50, dt=3600)
    end_time = time.time()
    print(f"Max Range: {range_m/1000:.1f} km")
    print(f"Range calculation took {end_time - start_time:.1f} seconds")
    
    start_time = time.time()
    endurance_s = mp1.get_endurance(dV=10, V_start=10, V_end=30, dt=3600)
    end_time = time.time()
    print(f"Max Endurance: {endurance_s/3600:.1f} hours")
    print(f"Endurance calculation took {end_time - start_time:.1f} seconds")


def test_max_speed_power_constraint(rotor_old:Rotor, vmin=50, vmax=150, dv=20, Ω=15, f=0.05):
    rotor = Rotor(rotor_mass=rotor_old.mass, Ω=Ω, blade_count=rotor_old.blade_count, R=rotor_old.R, rc=rotor_old.rc, chord_function=rotor_old.chord_function, θtw=rotor_old.θtw, ρ=rotor_old.ρ)  
    highest_power = 0
    for v in range(vmin, vmax+1, dv):
        D = get_Parasitic_Drag(ρ=1.0, f=f, Vinfty=v)
        T_Needed = get_Thrust_Total(W=(MASS_OF_CHINOOK/NUMBER_OF_MAIN_ROTORS)*g, D=D)
        rotor.set_calculation_batch_properties(Thrust_Needed=T_Needed, Ω=Ω, θ0=5.015*deg_to_rad, θ1s=-4.02*deg_to_rad, θ1c=3*deg_to_rad)
        try:
            _, trim_state_vals = trimSolve(rotor=rotor,  Ω=Ω, θ0_initial=5.015*deg_to_rad, θ1s_initial=-4.02*deg_to_rad, θ1c_initial=3*deg_to_rad, W=(MASS_OF_CHINOOK/NUMBER_OF_MAIN_ROTORS)*g, D=D, coning_angle_iterations=2, β0_step_fraction=1.00, iterations=10, relaxation=0.3, verbose=False)
            trim_power = trim_state_vals[2]
            if trim_power > highest_power:
                highest_power = trim_power
            print(f"V={v} m/s: Power={trim_power/10000:.1f} kW, Max Power={MAX_POWER*ENGINE_EFFICIENCY/10000:.1f} kW")
            if trim_power > MAX_POWER*ENGINE_EFFICIENCY:
                print("Power exceeds limit!")
                print("Velocity is: ", v)
                break
        except Exception as e:
            # print(f"Warning: Trim solve failed at V={v} m/s: {e}")
            print("Highest power reached was:", highest_power/1000, "kW at V =", v)
            return
            continue


def test_max_speed_stall_constraint(rotor_old, vmin=120, vmax=250, dv=10, Ω=15):
    stall_angle = 60*deg_to_rad
    rotor = Rotor(rotor_mass=rotor_old.mass, Ω=Ω,blade_count=rotor_old.blade_count, R=rotor_old.R, rc=rotor_old.rc, chord_function=rotor_old.chord_function, θtw=rotor_old.θtw, ρ=rotor_old.ρ)
    for v in range(vmin, vmax+1, dv):
        D = get_Parasitic_Drag(ρ=1.0, f=0.4, Vinfty=v)
        T_Needed = get_Thrust_Total(W=(MASS_OF_CHINOOK/NUMBER_OF_MAIN_ROTORS)*g, D=D)
        rotor.set_calculation_batch_properties(Thrust_Needed=T_Needed, Ω=Ω, θ0=5.015*deg_to_rad, θ1s=-4.02*deg_to_rad, θ1c=3*deg_to_rad)
        try:
            trims, trim_state_vals = trimSolve(rotor=rotor,  Ω=Ω, θ0_initial=5.015*deg_to_rad, θ1s_initial=-4.02*deg_to_rad, θ1c_initial=3*deg_to_rad, W=(MASS_OF_CHINOOK/NUMBER_OF_MAIN_ROTORS)*g, D=D, coning_angle_iterations=2, β0_step_fraction=1.00, iterations=10, relaxation=0.3, verbose=False)
            rotor.set_calculation_batch_properties(Thrust_Needed=T_Needed, Ω=Ω, θ0=trims[0], θ1s=trims[1], θ1c=trims[2])
            rotor.perform_all_calculations(W=(MASS_OF_CHINOOK/NUMBER_OF_MAIN_ROTORS)*g, D=D, coning_angle_iterations=2, β0_step_fraction=1.00)
            mesh_values= rotor.mesh
            
            for i in range(len(mesh_values)):
                for j in range(len(mesh_values[0])):
                    if mesh_values[i,j][1] > stall_angle:
                        print("Max speed limited by stall at V =", v)
                        return
        except Exception as e:
            print(f"Warning: Trim solve failed at V={v} m/s: {e}")
            continue
    return vmax


if __name__ == "__main__":
    f = 0.5
    V = 55
    W = 6000 * 9.81
    D = get_Parasitic_Drag(ρ=1.0, f=f, Vinfty=V)
    Ω = 50
    R = 5
    Thrust_Needed = get_Thrust_Total(W, D)
    θtw = 6 * deg_to_rad/R
    rotor = Rotor(rotor_mass=150, Ω=Ω, blade_count=5, R=R, rc=0.4, V_infty=90, chord_function=lambda r: 0.3, θtw=θtw, ρ=1.0)
    mp1 = MissionPlanner(rotor=rotor, Ω=Ω, f=f, fuelweight=FUEL_WEIGHT, dryweight=MASS_OF_CHINOOK - FUEL_WEIGHT, fuel_specific_energy=43e6)
    
    print("Testing endurance and range calculations...")
    test_endurance_and_range(mp1)
    
    # print("\nTesting max speed with power constraint...")
    test_max_speed_power_constraint(rotor, Ω=Ω, f=f)

    # print("\nTesting max speed with stall constraint...")
    test_max_speed_stall_constraint(rotor, Ω=Ω)