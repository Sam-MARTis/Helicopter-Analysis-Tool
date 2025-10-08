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

g = 9.81


NUMBER_OF_MAIN_ROTORS = 2
NUMBER_OF_ENGINES = 2
MASS_OF_CHINOOK = 3000 
FUEL_WEIGHT = 800
ENGINE_EFFICIENCY = 1
RHO = 1.0  
Ω = 15  
import numpy as np
from typing import Optional

class fuselage:
    def __init__(self) -> None:
        self.length: Optional[float] = 18
        self.diameter: Optional[float] = 2

        self.tailboom_length: Optional[float] = 8
        self.tailboom_diameter: Optional[float] = 0.7

        self.hrztail_h: Optional[float] = 1.5
        self.hrztail_b: Optional[float] = 0.5
        self.hrztail_t: Optional[float] = 0.3

        self.velocity: Optional[float] = 100
        self.density: Optional[float] = 1.0

    # backwards drag
    def Re_back_body(self):
        rho = self.density
        velocity = self.velocity
        d = self.length
        nu = 0.00002

        return (rho*velocity*d)/nu
    
    def D1(self) -> float:
        Re = self.Re_back_body()
        cd = (0.455/(np.log10(Re))) - (1700/Re)
        SA = (np.pi)*self.diameter*self.length

        return SA*cd
    
    def Re_back_tail(self):
        rho = self.density
        velocity = self.velocity
        d = self.tailboom_length
        nu = 0.00002

        return (rho*velocity*d)/nu
    
    def D2(self) -> float:
        Re = self.Re_back_tail()
        cd = (0.455/(np.log10(Re))) - (1700/Re)
        SA = (np.pi)*self.tailboom_diameter*self.length

        return SA*cd
    
    def Re_back_hrztail(self):
        rho = self.density
        velocity = self.velocity
        d = self.hrztail_b
        nu = 0.00002200

        return (rho*velocity*d)/nu
    
    def D3(self) -> float:
        Re = self.Re_back_hrztail()
        cd = (0.455/(np.log10(Re))) - (1700/Re)
        SA = (np.pi)*self.hrztail_b*self.length

        return SA*cd
    
    def get_drag(self, ρ, v):
        self.velocity = v
        return (self.D1() + self.D2() + self.D3()) * 0.5 * ρ * v**2

drag = fuselage()

def get_Parasitic_Drag(ρ,f, Vinfty):
    # return ρ * f * Vinfty * Vinfty *0.5
    return drag.get_drag(ρ, Vinfty)*f

def get_Thrust_Total(W, D):
    return sqrt(W*W + D*D)

def get_α_tpp(W, D):
    return atan2(D, W)


# V_INDEX = 0
# THETA_GEOMETRY_INDEX = 1
# THETA_DOWNWASH_INDEX = 2
class Rotor:
    def __init__(self, Ω, rotor_mass, blade_count, R, rc, chord_function, θtw, ρ, V_infty = 110, r_divisions = 30, ψ_divisions = 40, Cd0 = 0.0113, Cd_Clsq_slope = 0.037):
        
        
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
        self.Ω = Ω


        
        self.Thrust_Needed = 0
        self.θ0 = 10 * deg_to_rad
        self.θ1s = -5 * deg_to_rad
        self.θ1c = -0.01 * deg_to_rad
        
        
        
        self.α_tpp = 0
        self.mu = 0
        self.A = 0
        self.Ct = 0
        self.I = 0.06
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
        dβ_by_dt = (-self.α_tpp *(sin(ψ)*self.Ω)) *0 # Since in TPP
        Up = v + r*dβ_by_dt + self.Vinfty * sin(self.β0) * cos(ψ) + self.Vinfty * sin(self.α_tpp)
        Ut = self.Ω * r + self.Vinfty * cos(self.α_tpp) * sin(ψ)
        θ = self.θ1s * sin(ψ) + self.θ1c * cos(ψ) + self.θ0 + self.θtw * (r) - atan((Up) / (Ut))
        if(set_u_vels):
            self.mesh[i, j, 1] = θ
            self.mesh[i, j, 2] = Up
            self.mesh[i, j, 3] = Ut
        return θ
    
    def get_β0(self, β0_previous = None, set=False):
        β0_previous = self.β0 if β0_previous is None else β0_previous
        if set:
            self.β0 = β0_previous
        βs = []
        Up_moments = []
        sum = 0
        for j in range(self.ψ_divisions):
            for i in range(self.r_divisions):
                r, ψ = self.map_mesh_indices_to_r_ψ(i, j)
                α = self.get_effective_aoa(r, ψ)
                Up = self.mesh[i, j, 2]
                Ut = self.mesh[i, j, 3]
                if(α < 0):
                    α = 0
                Cl = self.a * (3*α/(3 + α*α)) #tanh approximation

                Cd = self.Cd0 + self.Cd_Clsq_slope * Cl * Cl
                c = self.chord_function(r)
                q = 0.5 * self.ρ * (Up * Up + Ut * Ut) * c
                θ = atan2(Up, Ut)
                dS = (Cl * cos(θ) - Cd * sin(θ)) * q

                sum += (Up * r)
            # β0 = internal_sum 
            # βs.append(β0)
        sum *= self.dr * self.dψ
        β0 = sum/(self.Ω * self.Ω * self.I)
        # print("β0:", β0)
        
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
        
        for i in range(self.r_divisions):
            for j in range(self.ψ_divisions):
                v = self.mesh[i, j, 0]
                # θ = np.arctan(self.mesh[i, j, 1]
                α = self.mesh[i, j, 1]
                r, ψ = self.map_mesh_indices_to_r_ψ(i, j)
                c = self.chord_function(r)
                Up = self.mesh[i, j, 2]
                Ut = self.mesh[i, j, 3]
                θ = np.atan2(Up, Ut) 
                U_sq = Up*Up + Ut*Ut
                Cl = self.a * (3*α/(3 + α*α)) #tanh approximation
                Cd = self.Cd0 + self.Cd_Clsq_slope * Cl * Cl
                dL = 0.5 * self.ρ * U_sq * c * Cl
                dD = 0.5 * self.ρ * U_sq * c * Cd
                dT = (dL * cos(θ) - dD * sin(θ)) * cosβ0
                d_drag = dL * sin(θ) + dD * cos(θ)
                self.mesh[i, j, 4] = dT * self.blade_count
                self.mesh[i, j, 5] = d_drag * self.blade_count

    def get_post_processed_results_average(self):
        thrust = 0
        moment = np.zeros(3) 
        
        
        
        
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
        
        self.calculate_primary_derived_quantities(W, D, coning_angle_iterations, β0_step_fraction)
        self.update_grid_secondary_derived_quantities()
        return self.get_post_processed_results_average()







def trimSolve(rotor:Rotor, Ω, θ0_initial, θ1s_initial, θ1c_initial, W, D, coning_angle_iterations=2, β0_step_fraction=1.00, iterations=20, relaxation=0.3, θ0_ϵ = θ_epsilon, θ1s_ϵ = θ_epsilon, θ1c_ϵ = θ_epsilon, verbose=True):
    """
    Newton-Raphson trim solver using full 3x3 Jacobian matrix
    """
    θ0 = θ0_initial
    θ1s = θ1s_initial
    θ1c = θ1c_initial

    Thrust_Needed = np.sqrt(W*W + D*D)

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
        # step_range = 1
        # Δθ = [np.clip(Δθ[i], -step_range*deg_to_rad, step_range*deg_to_rad) for i in range(3)]
        θ0 += relaxation * Δθ[0]
        θ1s += relaxation * Δθ[1]
        θ1c += relaxation * Δθ[2]
        
        # θ0 = np.clip(θ0, -20*deg_to_rad, 25*deg_to_rad)      
        # θ1s = np.clip(θ1s, -20*deg_to_rad, 20*deg_to_rad)   
        # θ1c = np.clip(θ1c, -20*deg_to_rad, 20*deg_to_rad)  
        
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



    



"""
θ0 -> Thrust + Mz + Mx + My
Θ1s -> Thrust + alpha.tpp + Mz
Θ1c -> Mx + beta1s
T_tail -> My

"""





def print_trim_values_and_state(rotor:Rotor, trim_values, state_values):
    θ0, θ1s, θ1c = trim_values
    print(f"  θ0: {θ0/deg_to_rad:.3f}°")
    print(f"  θ1s: {θ1s/deg_to_rad:.3f}°")
    print(f"  θ1c: {θ1c/deg_to_rad:.3f}°")
    print(f"  β0: {rotor.β0/deg_to_rad:.3f}°")
    print(f"  Thrust: {state_values[0]:.0f} N")
    print(f"  Moments: Mx={state_values[1][0]:.0f} N⋅m, My={state_values[1][1]:.0f} N⋅m, Mz={state_values[1][2]:.0f} N⋅m")
    print(f"  Power: {state_values[2]:.0f} W ({state_values[2]/1000:.0f} kW)")

if __name__ == "__main__":

    V = 55
    W = 5000 * 9.81
    D = get_Parasitic_Drag(ρ=1.0, f=1, Vinfty=V)
    Ω = 50
    R = 6.1
    Thrust_Needed = get_Thrust_Total(W, D)
    θtw = 6 * deg_to_rad/R
    rotor = Rotor(rotor_mass=150, Ω=Ω, blade_count=5, R=R, rc=0.4, V_infty=90, chord_function=lambda r: 0.3, θtw=θtw, ρ=1.0)
    # trimSolve(rotor=rotor1, Ω=Ω, θ0_initial=10.015*deg_to_rad, θ1s_initial=-14.02*deg_to_rad, θ1c_initial=3*deg_to_rad, W=MASS_OF_CHINOOK*g, D=drag, coning_angle_iterations=2, β0_step_fraction=1, iterations=5, relaxation=1, verbose=False)
    trim_vals, trim_outs = trimSolve(rotor=rotor, Ω=Ω, θ0_initial=10.015*deg_to_rad, θ1s_initial=-14.02*deg_to_rad, θ1c_initial=3*deg_to_rad, W=W, D=D, coning_angle_iterations=5, β0_step_fraction=1, iterations=10, relaxation=1, verbose=False)
    print_trim_values_and_state(rotor, trim_vals, trim_outs)
    print(f"Thrust needed: {Thrust_Needed:.1f} N")

    

    