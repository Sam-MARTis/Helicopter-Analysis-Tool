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

MAX_CPU_WORKERS = 18
deg_to_rad = np.pi / 180
θ_epsilon = 0.001 * deg_to_rad
thrust_tolerance = 100.0 
moment_tolerance = 200.0 
θtw = 1 * deg_to_rad

def get_Parasitic_Drag(ρ, f, Vinfty):
    return ρ * f * Vinfty * Vinfty *0.5

def get_Thrust_Total(W, D):
    return sqrt(W*W + D*D)

def get_α_tpp(W, D):
    return atan2(D, W)



class Rotor:
    def __init__(self, rotor_mass, blade_count, R, rc, chord_function, θtw, ρ, V_infty = 110, r_divisions = 20, ψ_divisions = 40, Cd0 = 0.0113, Cd_Clsq_slope = 0.037):
        
        # Supplied at instantiation
        self.mass = rotor_mass
        self.blade_count = blade_count
        self.R = R
        self.rc = rc
        self.chord_function = chord_function
        self.θtw = θtw
        self.a = 5.75
        self.ρ = ρ
        self.Vinfty  = V_infty
        self.r_divisions = r_divisions
        self.ψ_divisions = ψ_divisions
        self.Cd0 = Cd0
        self.Cd_Clsq_slope = Cd_Clsq_slope


        # Supplied before calculations. Can be varied between different calculation batches
        self.Thrust_Needed = 0
        self.Ω = 40
        self.θ0 = 10 * deg_to_rad
        self.θ1s = -5 * deg_to_rad
        self.θ1c = -0.01 * deg_to_rad
        
        
        # Calculated quantities
        self.α_tpp = 0
        self.mu = 0
        self.A = 0
        self.Ct = 0
        self.I = 0.6
        self.dr = (self.R - self.rc)/ self.r_divisions
        self.dψ = 2 * np.pi / self.ψ_divisions
        

        # Quantities to store: v, α_effective, Up, Ut, dT, d_drag
        self.mesh = np.zeros((self.r_divisions, self.ψ_divisions, 6))
    def set_mesh(self, r_divisions, ψ_divisions):
        self.r_divisions = r_divisions
        self.ψ_divisions = ψ_divisions
        self.dr = (self.R - self.rc)/ self.r_divisions
        self.dψ = 2 * np.pi / self.ψ_divisions
        self.mesh = np.zeros((self.r_divisions, self.ψ_divisions, 6))
    
    def set_calculation_batch_properties(self, Thrust_Needed, Ω, θ0, θ1s, θ1c):
        self.Thrust_Needed = Thrust_Needed
        self.Ω = Ω
        self.θ0 = θ0
        self.θ1s = θ1s
        self.θ1c = θ1c
        
        

    def map_r_ψ_to_mesh_indices(self, r, ψ) -> Tuple[int, int]:
        if r < self.rc or r > self.R:
            raise ValueError("r is out of bounds")
        if ψ < 0 or ψ >= 2 * np.pi:
            raise ValueError("ψ is out of bounds")
        
        dr = (self.R - self.rc) / self.r_divisions
        dψ = 2 * np.pi / self.ψ_divisions
        
        i = int((r - self.rc) / dr)
        j = int(ψ / dψ)
        
        return i, j
    def map_mesh_indices_to_r_ψ(self, i, j) -> Tuple[float, float]:

        dr = (self.R - self.rc) / self.r_divisions
        dψ = 2 * np.pi / self.ψ_divisions
        
        r = self.rc + (i + 0.5) * dr
        ψ = (j + 0.5) * dψ
        
        return r, ψ
    def get_Area_Disc(self, set = True):
        A = np.pi * self.R * self.R
        if set:
            self.A = A
        return A

    def get_I(self, set = True):
        I = (1/3) * ((self.mass/self.blade_count)/(self.R - self.rc)) * (np.power(self.R, 3) - np.power(self.rc, 3))
        if set:
            self.I = I
        return I

    def get_Ct(self, set = True):
        Ct = self.Thrust_Needed / (self.ρ * self.A * (self.Ω * self.R) ** 2)
        if set:
            self.Ct = Ct
        return Ct

    def get_α_tpp(self, W, D, set = True):
        α_tpp = atan2(D, W)
        if set:
            self.α_tpp = α_tpp
        return α_tpp

    def get_mu(self, set = True):
        mu = self.Vinfty * cos(self.α_tpp) / (self.Ω * self.R)
        if set:
            self.mu = mu
        return mu

    def get_λi_Glaubert(self, set=True):
        assert self.mu >= 0.2
        λi_Glaubert = self.Ct/(2*self.mu)
        if set:
            self.λi_Glaubert = λi_Glaubert
        return λi_Glaubert

    def get_derived_properties(self):
        self.A = self.get_Area_Disc()
        self.Ct = self.get_Ct()
        
    def get_λG(self, set=True):
        λG = self.λi_Glaubert + (self.Vinfty * sin(self.α_tpp)) / (self.Ω * self.R)
        if set:
            self.λG = λG
        return λG
    def get_v(self, r, ψ):
        num = (4/3) * (self.mu/self.λG) * r * cos(ψ)
        den = (1.2 + (self.mu/self.λG))*self.R 
        λi = self.λi_Glaubert*(1 + (num/den))
        return λi * self.Ω * self.R
    
    def get_effective_aoa(self, r, ψ, set_u_vels = True):
        i, j = self.map_r_ψ_to_mesh_indices(r, ψ)
        v = self.mesh[i, j, 0]
        dβ_by_dt = -self.α_tpp *(sin(ψ)*self.Ω)
        Up = v + r*dβ_by_dt + self.Vinfty * sin(self.β0) * cos(ψ) + self.Vinfty * sin(self.α_tpp)
        Ut = self.Ω * r + self.Vinfty * cos(self.α_tpp) * sin(ψ)
        if(set_u_vels):
            self.mesh[i, j, 2] = Up
            self.mesh[i, j, 3] = Ut
        θ = self.θ1s * sin(ψ) + self.θ1c * cos(ψ) + self.θ0 + self.θtw * (r) - atan((Up) / (Ut))
        return θ
    
    def get_β0(self, β0_previous = None, set=False):
        β0_previous = self.β0 if β0_previous is None else β0_previous

        internal_sum = 0
        for i in range(self.r_divisions):
            for j in range(self.ψ_divisions):
                r, ψ = self.map_mesh_indices_to_r_ψ(i, j)
                θ = self.get_effective_aoa(r, ψ)
                Cl = self.a * θ
                c = self.chord_function(r)
                dS = 0.5 * self.ρ * r*r * c * Cl * self.dr * self.dψ
                internal_sum += dS
        β0 = internal_sum/(self.I * 2*np.pi)
        
        if set:
            self.β0 = β0
        return β0
    
    
    def update_grid_induced_velocity(self):
        for i in range(self.r_divisions):
            for j in range(self.ψ_divisions):
                r, ψ = self.map_mesh_indices_to_r_ψ(i, j)
                v = self.get_v(r, ψ)
                self.mesh[i, j, 0] = v
    def update_grid_effective_aoa_and_U_vels(self):
        for i in range(self.r_divisions):
            for j in range(self.ψ_divisions):
                r, ψ = self.map_mesh_indices_to_r_ψ(i, j)
                θ = self.get_effective_aoa(r, ψ)
                self.mesh[i, j, 1] = θ

    def calculate_primary_derived_quantities(self, W, D, coning_angle_iterations = 5, β0_step_fraction = 0.6):
        self.get_Area_Disc()
        self.get_I()
        self.get_Ct()
        self.get_α_tpp(W, D)
        self.get_mu()
        self.get_λi_Glaubert()
        self.get_λG()
        self.β0 = 0
        self.update_grid_induced_velocity()
        self.update_grid_effective_aoa_and_U_vels()
        
        for _ in range(coning_angle_iterations):
            
            β0 = self.get_β0()
            dβ0 = β0 - self.β0
            self.β0 = self.β0 + β0_step_fraction * dβ0
            self.update_grid_effective_aoa_and_U_vels()
            
    def update_grid_secondary_derived_quantities(self):
        self.update_grid_induced_velocity()
        self.update_grid_effective_aoa_and_U_vels()
        cosβ0 = cos(self.β0)
        # Quantities to store: v, α_effective, Up, Ut, dT, d_drag
        for i in range(self.r_divisions):
            for j in range(self.ψ_divisions):
                v = self.mesh[i, j, 0]
                θ = self.mesh[i, j, 1]
                r, ψ = self.map_mesh_indices_to_r_ψ(i, j)
                c = self.chord_function(r)
                Up = self.mesh[i, j, 2]
                Ut = self.mesh[i, j, 3]
                U_sq = Up*Up + Ut*Ut
                Cl = self.a * θ
                Cd = self.Cd0 + self.Cd_Clsq_slope * Cl * Cl
                dL = 0.5 * self.ρ * U_sq * c * Cl
                dD = 0.5 * self.ρ * U_sq * c * Cd
                dT = (dL * cos(θ) - dD * sin(θ)) * cosβ0
                d_drag = dL * sin(θ) + dD * cos(θ)
                self.mesh[i, j, 4] = dT * self.blade_count
                self.mesh[i, j, 5] = d_drag * self.blade_count

    def get_post_processed_results_average(self):
        thrust = 0
        moment = np.zeros(3) # x y z
        # x is right, y is up, z is back
        #FUTURE ME: READ THIS:
        # The tip path plane is used as referecne here, y direction is normal to tip path plane.
        
        for i in range(self.r_divisions):
            for j in range(self.ψ_divisions):
                r, ψ = self.map_mesh_indices_to_r_ψ(i, j)
                dT = self.mesh[i, j, 4] * self.dr * self.dψ
                d_drag = self.mesh[i, j, 5] * self.dr * self.dψ
                thrust += dT
                moment[0] += dT * r * sin(ψ - np.pi/2) 
                moment[1] += d_drag * r
                moment[2] += dT * r * sin(ψ)
        moment = moment/(2 * np.pi)
        power = moment[1] * self.Ω
        return thrust, moment, power
    
    def perform_all_calculations(self, W, D, coning_angle_iterations = 5, β0_step_fraction = 0.6):
        #Rememebrr to initialize rotor properties before calling this
        self.calculate_primary_derived_quantities(W, D, coning_angle_iterations, β0_step_fraction)
        self.update_grid_secondary_derived_quantities()
        return self.get_post_processed_results_average()


rotor1 = Rotor(rotor_mass=150, blade_count=4, R=5, rc=0.2, chord_function=lambda r: 0.3, θtw=θtw, ρ=1.225)

# rotor1.set_calculation_batch_properties(Thrust_Needed=30000, Ω=20, θ0=3.515*deg_to_rad, θ1s=-4.42*deg_to_rad, θ1c=1.8*deg_to_rad)
# vals = rotor1.perform_all_calculations(W=2000, D=200, coning_angle_iterations=2, β0_step_fraction=1.00)
# print(vals)


def trimSolve(rotor:Rotor, Thrust_Needed, Ω, θ0_initial, θ1s_initial, θ1c_initial, W, D, coning_angle_iterations=2, β0_step_fraction=1.00, iterations=20, relaxation=0.3, θ0_ϵ = θ_epsilon, θ1s_ϵ = θ_epsilon, θ1c_ϵ = θ_epsilon, verbose=True):
    """
    Newton-Raphson trim solver using full 3x3 Jacobian matrix
    """
    θ0 = θ0_initial
    θ1s = θ1s_initial
    θ1c = θ1c_initial
    

    
    if verbose:
        print(f"Starting trim solve: Target Thrust = {Thrust_Needed} N")
        print(f"Initial guess: θ0={θ0/deg_to_rad:.2f}°, θ1s={θ1s/deg_to_rad:.2f}°, θ1c={θ1c/deg_to_rad:.2f}°")
    
    for iteration in range(iterations):
        rotor.set_calculation_batch_properties(Thrust_Needed=Thrust_Needed, Ω=Ω, θ0=θ0, θ1s=θ1s, θ1c=θ1c)
        vals_current = rotor.perform_all_calculations(W=W, D=D, coning_angle_iterations=coning_angle_iterations, β0_step_fraction=β0_step_fraction)
        
        T_current = vals_current[0]
        Mx_current = vals_current[1][0]  
        Mz_current = vals_current[1][2]  

        residual_T = Thrust_Needed - T_current
        residual_Mx = 0 - Mx_current  
        residual_Mz = 0 - Mz_current  
        
        if (abs(residual_T) < thrust_tolerance and 
            abs(residual_Mx) < moment_tolerance and 
            abs(residual_Mz) < moment_tolerance):
            if verbose:
                print(f"Converged in {iteration+1} iterations!")
            break
        
        if verbose and (iteration % 5 == 0 or iteration < 3):
            print(f"Iter {iteration}: T={T_current:.0f}N (err:{residual_T:.0f}), Mx={Mx_current:.0f}N⋅m, Mz={Mz_current:.0f}N⋅m")
        J = np.zeros((3, 3))

        rotor.set_calculation_batch_properties(Thrust_Needed=Thrust_Needed, Ω=Ω, θ0=θ0 + θ0_ϵ, θ1s=θ1s, θ1c=θ1c)
        vals_θ0 = rotor.perform_all_calculations(W=W, D=D, coning_angle_iterations=coning_angle_iterations, β0_step_fraction=β0_step_fraction)
        J[0, 0] = (vals_θ0[0] - T_current) / θ0_ϵ      
        J[1, 0] = (vals_θ0[1][0] - Mx_current) / θ0_ϵ   
        J[2, 0] = (vals_θ0[1][2] - Mz_current) / θ0_ϵ   
        

        rotor.set_calculation_batch_properties(Thrust_Needed=Thrust_Needed, Ω=Ω, θ0=θ0, θ1s=θ1s + θ1s_ϵ, θ1c=θ1c)
        vals_θ1s = rotor.perform_all_calculations(W=W, D=D, coning_angle_iterations=coning_angle_iterations, β0_step_fraction=β0_step_fraction)
        J[0, 1] = (vals_θ1s[0] - T_current) / θ1s_ϵ     
        J[1, 1] = (vals_θ1s[1][0] - Mx_current) / θ1s_ϵ  
        J[2, 1] = (vals_θ1s[1][2] - Mz_current) / θ1s_ϵ  
        
        
        rotor.set_calculation_batch_properties(Thrust_Needed=Thrust_Needed, Ω=Ω, θ0=θ0, θ1s=θ1s, θ1c=θ1c + θ1c_ϵ)
        vals_θ1c = rotor.perform_all_calculations(W=W, D=D, coning_angle_iterations=coning_angle_iterations, β0_step_fraction=β0_step_fraction)
        J[0, 2] = (vals_θ1c[0] - T_current) / θ1c_ϵ     
        J[1, 2] = (vals_θ1c[1][0] - Mx_current) / θ1c_ϵ  
        J[2, 2] = (vals_θ1c[1][2] - Mz_current) / θ1c_ϵ  
        
     
        det_J = np.linalg.det(J)
        if abs(det_J) < 1e-10:
            if verbose:
                print(f"Jacobian is singular (det={det_J}).")
            break
        
        
        residuals = np.array([residual_T, residual_Mx, residual_Mz])
        try:
            Δθ = np.linalg.solve(J, residuals)
        except np.linalg.LinAlgError:
            if verbose:
                print("Failed to solve Jacobian system, stopping iteration.")
            break

        θ0 += relaxation * Δθ[0]
        θ1s += relaxation * Δθ[1]
        θ1c += relaxation * Δθ[2]
        
        θ0 = np.clip(θ0, 0*deg_to_rad, 25*deg_to_rad)      
        θ1s = np.clip(θ1s, -20*deg_to_rad, 20*deg_to_rad)   
        θ1c = np.clip(θ1c, -20*deg_to_rad, 20*deg_to_rad)  
        
        if verbose and (iteration % 5 == 0 or iteration < 3):
            print(f"  Δθ: [{Δθ[0]/deg_to_rad:.3f}°, {Δθ[1]/deg_to_rad:.3f}°, {Δθ[2]/deg_to_rad:.3f}°]")
            print(f"  New θ: [{θ0/deg_to_rad:.3f}°, {θ1s/deg_to_rad:.3f}°, {θ1c/deg_to_rad:.3f}°]")
    

    rotor.set_calculation_batch_properties(Thrust_Needed=Thrust_Needed, Ω=Ω, θ0=θ0, θ1s=θ1s, θ1c=θ1c)
    vals_final = rotor.perform_all_calculations(W=W, D=D, coning_angle_iterations=coning_angle_iterations, β0_step_fraction=β0_step_fraction)
    
    if verbose:
        print(f"\nFinal Results:")
        print(f"Controls: θ0={θ0/deg_to_rad:.3f}°, θ1s={θ1s/deg_to_rad:.3f}°, θ1c={θ1c/deg_to_rad:.3f}°")
        print(f"Thrust: {vals_final[0]:.0f} N (target: {Thrust_Needed} N, error: {vals_final[0]-Thrust_Needed:.0f} N)")
        print(f"Moments: Mx={vals_final[1][0]:.0f} N⋅m, My={vals_final[1][1]:.0f} N⋅m, Mz={vals_final[1][2]:.0f} N⋅m")
        print(f"Power: {vals_final[2]:.0f} W ({vals_final[2]/1000:.0f} kW)")
    
    return np.array([θ0, θ1s, θ1c]), vals_final




        

# trim_vals, state_vals = trimSolve(rotor=rotor1, Thrust_Needed=30000, Ω=20, θ0_initial=5.015*deg_to_rad, θ1s_initial=-4.02*deg_to_rad, θ1c_initial=3*deg_to_rad, W=2000, D=200, coning_angle_iterations=2, β0_step_fraction=1.00, iterations=10, relaxation=0.8, verbose=False)
# print(trim_vals * (180/np.pi))
# print(state_vals)


"""
θ0 -> Thrust + Mz + Mx + My
Θ1s -> Thrust + alpha.tpp + Mz
Θ1c -> Mx + beta1s
T_tail -> My

"""

g = 9.81

def _calculate_speed_performance(args):
    """Helper function for multiprocessing speed calculations"""
    V, weight, mp_params = args
    
    # Recreate rotor instance
    rotor = Rotor(
        rotor_mass=mp_params['rotor_mass'], 
        blade_count=mp_params['blade_count'], 
        R=mp_params['R'], 
        rc=mp_params['rc'], 
        chord_function=lambda r: 0.5, 
        θtw=mp_params['θtw'], 
        ρ=mp_params['ρ']
    )
    
    # Recreate mission planner instance
    mp_instance = MissionPlanner(
        rotor=rotor,
        Ω=mp_params['Ω'],
        f=mp_params['f'],
        fuelweight=mp_params['fuelweight'], 
        dryweight=mp_params['dryweight'],
        fuel_specific_energy=mp_params['fuel_specific_energy'],
        fuel_reserve_fraction=mp_params['fuel_reserve_fraction']
    )
    
    try:
        D = get_Parasitic_Drag(ρ=mp_instance.ρ, f=mp_instance.f, Vinfty=V)
        T_Needed = get_Thrust_Total(W=weight*g, D=D)
        
        _, trim_state_vals = trimSolve(
            rotor=mp_instance.rotor, 
            Thrust_Needed=T_Needed, 
            Ω=20, 
            θ0_initial=5.015*deg_to_rad, 
            θ1s_initial=-4.02*deg_to_rad, 
            θ1c_initial=3*deg_to_rad, 
            W=weight*g, 
            D=D, 
            coning_angle_iterations=2, 
            β0_step_fraction=1.00, 
            iterations=10, 
            relaxation=0.3, 
            verbose=False
        )
        
        trim_power = trim_state_vals[2]
        if trim_power > 0:
            return V, trim_power, V/trim_power  # speed, power, efficiency
        else:
            return V, None, None
            
    except Exception as e:
        return V, None, None

def _calculate_endurance_performance(args):
    """Helper function for multiprocessing endurance calculations"""
    V, weight, mp_params = args
    
    # Recreate rotor instance
    rotor = Rotor(
        rotor_mass=mp_params['rotor_mass'], 
        blade_count=mp_params['blade_count'], 
        R=mp_params['R'], 
        rc=mp_params['rc'], 
        chord_function=lambda r: 0.5, 
        θtw=mp_params['θtw'], 
        ρ=mp_params['ρ']
    )
    
    # Recreate mission planner instance
    mp_instance = MissionPlanner(
        rotor=rotor,
        Ω=mp_params['Ω'],
        f=mp_params['f'],
        fuelweight=mp_params['fuelweight'], 
        dryweight=mp_params['dryweight'],
        fuel_specific_energy=mp_params['fuel_specific_energy'],
        fuel_reserve_fraction=mp_params['fuel_reserve_fraction']
    )
    
    try:
        D = get_Parasitic_Drag(ρ=mp_instance.ρ, f=mp_instance.f, Vinfty=V)
        T_Needed = get_Thrust_Total(W=weight*g, D=D)
        
        _, trim_state_vals = trimSolve(
            rotor=mp_instance.rotor, 
            Thrust_Needed=T_Needed, 
            Ω=20, 
            θ0_initial=5.015*deg_to_rad, 
            θ1s_initial=-4.02*deg_to_rad, 
            θ1c_initial=3*deg_to_rad, 
            W=weight*g, 
            D=D, 
            coning_angle_iterations=2, 
            β0_step_fraction=1.00, 
            iterations=10, 
            relaxation=0.3, 
            verbose=False
        )
        
        trim_power = trim_state_vals[2]
        if trim_power > 0:
            return V, trim_power
        else:
            return V, None
            
    except Exception as e:
        return V, None


class MissionPlanner:
    def __init__(self, rotor, Ω, f, fuelweight, dryweight, fuel_specific_energy, fuel_reserve_fraction = 0.2):
        self.ρ = 1.225 
        self.f = f
        self.fuelweight = fuelweight
        self.dryweight = dryweight
        self.fuel_specific_energy = fuel_specific_energy  # in J/kg
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
        # Copy rotor parameters
        new_mp.ρ = self.ρ
        return new_mp
        

    def get_max_range_speed(self, weight, dV = 10, V_start = 10, V_end = 150) -> Tuple[float, float]:
        best_v_by_power = 0
        best_speed = V_start
        V_by_power = {}
        for V in np.arange(V_start, V_end, dV):
            D = get_Parasitic_Drag(ρ=self.ρ, f=self.f, Vinfty=V)
            T_Needed = get_Thrust_Total(W=weight*g, D=D)
            # self.rotor.set_calculation_batch_properties(Thrust_Needed=T_Needed, Ω=20, θ0=5.015*deg_to_rad, θ1s=-4.02*deg_to_rad, θ1c=3*deg_to_rad)
            try:
                _, trim_state_vals = trimSolve(rotor=self.rotor, Thrust_Needed=T_Needed, Ω=self.Ω, θ0_initial=5.015*deg_to_rad, θ1s_initial=-4.02*deg_to_rad, θ1c_initial=3*deg_to_rad, W=weight*g, D=D, coning_angle_iterations=2, β0_step_fraction=1.00, iterations=10, relaxation=0.3, verbose=False)
                trim_power = trim_state_vals[2]
                if trim_power > 0:  # Only consider positive power
                    V_by_power[V] = V/trim_power
            except Exception as e:
                print(f"Warning: Trim solve failed at V={V} m/s: {e}")
                continue
        
        # Return the best V
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
        max_iterations = int(fuel_weight_available * self.fuel_specific_energy / (50000 * dt)) + 100  # Safety limit
        
        while fuel_weight_available > 0 and iteration < max_iterations:
            print(f"Iteration {iteration}: Weight={weight:.1f} kg, Fuel left={fuel_weight_available:.1f} kg, Distance={total_distance/1000:.2f} km, Time={time_elapsed/3600:.2f} hr")
            try:
                best_speed, best_v_by_power = self.get_max_range_speed(weight=weight, dV=dV, V_start=V_start, V_end=V_end)
                
                if best_v_by_power <= 0:
                    print("Warning: Invalid efficiency in range calculation")
                    break
                    
                power_consumed = best_speed / best_v_by_power
                fuel_consumed = power_consumed * dt / self.fuel_specific_energy
                
                # Don't consume more fuel than available
                if fuel_consumed > fuel_weight_available:
                    # Calculate partial time step
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
            # self.rotor.set_calculation_batch_properties(Thrust_Needed=T_Needed, Ω=20, θ0=5.015*deg_to_rad, θ1s=-4.02*deg_to_rad, θ1c=3*deg_to_rad)
            try:
                _, trim_state_vals = trimSolve(rotor=self.rotor, Thrust_Needed=T_Needed, Ω=self.Ω, θ0_initial=5.015*deg_to_rad, θ1s_initial=-4.02*deg_to_rad, θ1c_initial=3*deg_to_rad, W=weight*g, D=D, coning_angle_iterations=2, β0_step_fraction=1.00, iterations=10, relaxation=0.3, verbose=False)
                trim_power = trim_state_vals[2]
                if trim_power > 0:  # Only consider positive power
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
        max_iterations = int(fuel_weight_available * self.fuel_specific_energy / (50000 * dt)) + 100  # Safety limit
        
        while fuel_weight_available > 0 and iteration < max_iterations:
            print(f"Iteration {iteration}: Weight={weight:.1f} kg, Fuel left={fuel_weight_available:.1f} kg, Time={total_time/3600:.2f} hr")
            try:
                best_speed, least_power = self.get_max_endurance_speed(weight=weight, dV=dV, V_start=V_start, V_end=V_end)
                
                if least_power <= 0:
                    print("Warning: Invalid power in endurance calculation")
                    break
                    
                fuel_consumed = least_power * dt / self.fuel_specific_energy
                
                # Don't consume more fuel than available
                if fuel_consumed > fuel_weight_available:
                    # Calculate partial time step
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
    
    def get_max_range_speed_mt(self, weight, dV=30, V_start=10, V_end=150, num_workers=None) -> Tuple[float, float]:
        """Multithreaded version of get_max_range_speed for better performance"""
        if num_workers is None:
            num_workers = min(mp.cpu_count(), MAX_CPU_WORKERS)  # Limit to 18 workers max

        # Prepare parameters for multiprocessing
        mp_params = {
            'rotor_mass': self.rotor.mass,
            'blade_count': self.rotor.blade_count,
            'R': self.rotor.R,
            'rc': self.rotor.rc,
            'θtw': self.rotor.θtw,
            'ρ': self.rotor.ρ,
            'Ω': self.Ω,
            'f': self.f,
            'fuelweight': self.fuelweight,
            'dryweight': self.dryweight,
            'fuel_specific_energy': self.fuel_specific_energy,
            'fuel_reserve_fraction': self.fuel_reserve_fraction
        }
        
        # Create speed points to test
        speeds = list(np.arange(V_start, V_end, dV))
        args_list = [(V, weight, mp_params) for V in speeds]
        
        best_v_by_power = 0
        best_speed = V_start
        
        # Use ProcessPoolExecutor for CPU-bound tasks
        with ProcessPoolExecutor(max_workers=num_workers) as executor:
            results = list(executor.map(_calculate_speed_performance, args_list))
        
        # Process results
        for V, power, efficiency in results:
            if efficiency is not None and efficiency > best_v_by_power:
                best_v_by_power = efficiency
                best_speed = V
        
        return best_speed, best_v_by_power
    
    def get_max_endurance_speed_mt(self, weight, dV=5, V_start=10, V_end=50, num_workers=None) -> Tuple[float, float]:
        """Multithreaded version of get_max_endurance_speed for better performance"""
        if num_workers is None:
            num_workers = min(mp.cpu_count(), MAX_CPU_WORKERS)  # Limit to 18 workers max

        # Prepare parameters for multiprocessing
        mp_params = {
            'rotor_mass': self.rotor.mass,
            'blade_count': self.rotor.blade_count,
            'R': self.rotor.R,
            'rc': self.rotor.rc,
            'θtw': self.rotor.θtw,
            'ρ': self.rotor.ρ,
            'Ω': self.Ω,
            'f': self.f,
            'fuelweight': self.fuelweight,
            'dryweight': self.dryweight,
            'fuel_specific_energy': self.fuel_specific_energy,
            'fuel_reserve_fraction': self.fuel_reserve_fraction
        }
        
        # Create speed points to test
        speeds = list(np.arange(V_start, V_end, dV))
        args_list = [(V, weight, mp_params) for V in speeds]
        
        least_power = float('inf')
        best_speed = V_start
        
        # Use ProcessPoolExecutor for CPU-bound tasks
        with ProcessPoolExecutor(max_workers=num_workers) as executor:
            results = list(executor.map(_calculate_endurance_performance, args_list))
        
        # Process results
        for V, power in results:
            if power is not None and power < least_power:
                least_power = power
                best_speed = V
        
        return best_speed, least_power
    
    def get_max_range_mt(self, dV, V_start, V_end, dt=600, num_workers=None):
        """Multithreaded version of get_max_range"""
        fuel_weight_available = self.fuelweight*(1 - self.fuel_reserve_fraction)
        weight = self.dryweight + self.fuelweight
        total_distance = 0
        time_elapsed = 0
        iteration = 0
        max_iterations = int(fuel_weight_available * self.fuel_specific_energy / (50000 * dt)) + 100
        
        while fuel_weight_available > 0 and iteration < max_iterations:
            print("Fuel left: {:.1f} kg, Weight: {:.1f} kg".format(fuel_weight_available, weight))
            print("Distance so far: {:.1f} km, Time elapsed: {:.1f} hours".format(total_distance/1000, time_elapsed/3600))
            try:
                best_speed, best_v_by_power = self.get_max_range_speed_mt(
                    weight=weight, dV=dV, V_start=V_start, V_end=V_end, num_workers=num_workers
                )
                
                if best_v_by_power <= 0:
                    print("Warning: Invalid efficiency in range calculation")
                    break
                    
                power_consumed = best_speed / best_v_by_power
                fuel_consumed = power_consumed * dt / self.fuel_specific_energy
                
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

    def get_endurance_mt(self, dV, V_start, V_end, dt=1000, num_workers=None, max_iterations=100):
        """Multithreaded version of get_endurance"""
        fuel_weight_available = self.fuelweight*(1 - self.fuel_reserve_fraction)
        weight = self.dryweight + self.fuelweight
        total_time = 0
        iteration = 0
        # max_iterations = int(fuel_weight_available * self.fuel_specific_energy / (50000 * dt)) + 100
        
        while fuel_weight_available > 0 and iteration < max_iterations:
            try:
                best_speed, least_power = self.get_max_endurance_speed_mt(
                    weight=weight, dV=dV, V_start=V_start, V_end=V_end, num_workers=num_workers
                )
                
                if least_power <= 0:
                    print("Warning: Invalid power in endurance calculation")
                    break
                    
                fuel_consumed = least_power * dt / self.fuel_specific_energy
                
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
    def get_trim_conditions(self, V):
        D = get_Parasitic_Drag(ρ=self.ρ, f=self.f, Vinfty=V)
        T_Needed = get_Thrust_Total(W=(self.dryweight + self.fuelweight)*g, D=D)
        trim_vals, trim_state_vals = trimSolve(rotor=self.rotor, Thrust_Needed=T_Needed, Ω=self.Ω, θ0_initial=5.015*deg_to_rad, θ1s_initial=-4.02*deg_to_rad, θ1c_initial=3*deg_to_rad, W=(self.dryweight + self.fuelweight)*g, D=D, coning_angle_iterations=2, β0_step_fraction=1.00, iterations=100, relaxation=0.3, verbose=False)
        return trim_vals, trim_state_vals
    

NUMBER_OF_MAIN_ROTORS = 2
NUMBER_OF_ENGINES = 2
MASS_OF_CHINOOK = 3000 
FUEL_WEIGHT = 800
ENGINE_EFFICIENCY = 1
RHO = 1.0  
Ω = 15  # rad/s
# if __name__ == "__main__":
MAX_POWER = 200000  
def test_endurance_and_range(mp1:MissionPlanner):
    
    import time
    start_time = time.time()
    range_m, time_s = mp1.get_max_range(dV=40, V_start=10, V_end=150, dt=3000)
    end_time = time.time()
    print(f"Max Range: {range_m/1000:.1f} km in {time_s/3600:.1f} hours")
    print(f"Range calculation took {end_time - start_time:.1f} seconds")
    
    start_time = time.time()
    endurance_s = mp1.get_endurance(dV=10, V_start=10, V_end=50, dt=3600)
    end_time = time.time()
    print(f"Max Endurance: {endurance_s/3600:.1f} hours")
    print(f"Endurance calculation took {end_time - start_time:.1f} seconds")
    
    
def test_max_speed_power_constraint(vmin:130, vmax=250, dv=10):
    rotor = Rotor(rotor_mass=150, blade_count=3, R=12, rc=0.4, chord_function=lambda r: 0.5, θtw=θtw, ρ=RHO)  # Smaller rotor radius
    for v in range(vmin, vmax+1, dv):
        D = get_Parasitic_Drag(ρ=1.225, f=0.4, Vinfty=v)
        T_Needed = get_Thrust_Total(W=(MASS_OF_CHINOOK/NUMBER_OF_MAIN_ROTORS)*g, D=D)
        rotor.set_calculation_batch_properties(Thrust_Needed=T_Needed, Ω=Ω, θ0=5.015*deg_to_rad, θ1s=-4.02*deg_to_rad, θ1c=3*deg_to_rad)
        try:
            _, trim_state_vals = trimSolve(rotor=rotor, Thrust_Needed=T_Needed, Ω=Ω, θ0_initial=5.015*deg_to_rad, θ1s_initial=-4.02*deg_to_rad, θ1c_initial=3*deg_to_rad, W=(MASS_OF_CHINOOK/NUMBER_OF_MAIN_ROTORS)*g, D=D, coning_angle_iterations=2, β0_step_fraction=1.00, iterations=10, relaxation=0.3, verbose=False)
            trim_power = trim_state_vals[2]
            print(f"V={v} m/s: Power={trim_power/1000:.1f} kW")
            if trim_power > MAX_POWER:
                print("Power exceeds 200 kW limit!")
                print("Velocity is: ", v)
                break
        except Exception as e:
            print(f"Warning: Trim solve failed at V={v} m/s: {e}")
            continue



if __name__ == "__main__":
    # test_max_speed_power_constraint(vmin=130, vmax=250, dv=10)
    
    rotor = Rotor(rotor_mass=150, blade_count=3, R=12, rc=0.4, chord_function=lambda r: 0.5, θtw=θtw, ρ=RHO)  # Smaller rotor radius
    mp1 = MissionPlanner(rotor=rotor, Ω=Ω, f=0.4, fuelweight=FUEL_WEIGHT/NUMBER_OF_MAIN_ROTORS, dryweight=(MASS_OF_CHINOOK-FUEL_WEIGHT)/NUMBER_OF_MAIN_ROTORS, fuel_specific_energy=43e6*ENGINE_EFFICIENCY, fuel_reserve_fraction=0.15)
    trims, trim_out = mp1.get_trim_conditions(V=111)
    print("Θ0 (deg):", trims[0]/deg_to_rad)
    print("Θ1s (deg):", trims[1]/deg_to_rad)
    print("Θ1c (deg):", trims[2]/deg_to_rad)
    print("Thrust (N):", trim_out[0])
    print("Moments (N·m):")
    print(" Mx:", trim_out[1][0])
    print(" My:", trim_out[1][1])
    print(" Mz:", trim_out[1][2])
    print("Power (kW):", trim_out[2]/1000)
        
    
            
            
        
    