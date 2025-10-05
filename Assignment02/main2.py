from typing import Optional, Tuple, Dict, List, Union
import numpy as np
from time import sleep
from numba import njit
import matplotlib.pyplot as plt
from numpy import sin, cos, tan, sqrt, atan2, atan
from scipy.optimize import minimize

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




        

trim_vals, state_vals = trimSolve(rotor=rotor1, Thrust_Needed=30000, Ω=20, θ0_initial=5.015*deg_to_rad, θ1s_initial=-4.02*deg_to_rad, θ1c_initial=3*deg_to_rad, W=2000, D=200, coning_angle_iterations=2, β0_step_fraction=1.00, iterations=10, relaxation=0.8, verbose=False)
print(trim_vals * (180/np.pi))
print(state_vals)


"""
θ0 -> Thrust + Mz + Mx + My
Θ1s -> Thrust + alpha.tpp + Mz
Θ1c -> Mx + beta1s
T_tail -> My

"""

g = 9.81


class MissionPlanner:
    def __init__(self, f, fuelweight, dryweight, fuel_specific_energy, fuel_reserve_fraction = 0.2):
        self.ρ = 1.225 
        self.f = f
        self.fuelweight = fuelweight
        self.dryweight = dryweight
        self.fuel_specific_energy = fuel_specific_energy  # in J/kg
        self.fuel_reserve_fraction = fuel_reserve_fraction
        self.rotor = Rotor(rotor_mass=150, blade_count=4, R=5, rc=0.2, chord_function=lambda r: 0.3, θtw=θtw, ρ=self.ρ) 
        
        

    def get_max_range_speed(self, weight, dV = 30, V_start = 10, V_end = 200) -> Tuple[float, float]:
        best_v_by_power = 0
        best_speed = V_start
        V_by_power = {}
        for V in np.arange(V_start, V_end, dV):
            D = get_Parasitic_Drag(ρ=self.ρ, f=self.f, Vinfty=V)
            T_Needed = get_Thrust_Total(W=weight*g, D=D)
            # self.rotor.set_calculation_batch_properties(Thrust_Needed=T_Needed, Ω=20, θ0=5.015*deg_to_rad, θ1s=-4.02*deg_to_rad, θ1c=3*deg_to_rad)
            _, trim_state_vals = trimSolve(rotor=self.rotor, Thrust_Needed=T_Needed, Ω=20, θ0_initial=5.015*deg_to_rad, θ1s_initial=-4.02*deg_to_rad, θ1c_initial=3*deg_to_rad, W=self.dryweight, D=D, coning_angle_iterations=2, β0_step_fraction=1.00, iterations=10, relaxation=0.8, verbose=False)
            trim_power = trim_state_vals[2]
            V_by_power[V] = V/trim_power
        
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
        while fuel_weight_available > 0:
            best_speed, best_v_by_power = self.get_max_range_speed(weight=weight, dV=dV, V_start=V_start, V_end=V_end)
            power_consumed = best_speed / best_v_by_power
            fuel_consumed = power_consumed * dt / self.fuel_specific_energy
            total_distance += best_speed * dt
            fuel_weight_available -= fuel_consumed
            weight -= fuel_consumed
            time_elapsed += dt
        
        return total_distance, time_elapsed
    
    def get_max_endurance_speed(self, weight, dV = 5, V_start = 10, V_end = 100) -> Tuple[float, float]:
        least_power = float('inf')
        best_speed = V_start
        V_power = {}
        for V in np.arange(V_start, V_end, dV):
            D = get_Parasitic_Drag(ρ=self.ρ, f=self.f, Vinfty=V)
            T_Needed = get_Thrust_Total(W=weight*g, D=D)
            # self.rotor.set_calculation_batch_properties(Thrust_Needed=T_Needed, Ω=20, θ0=5.015*deg_to_rad, θ1s=-4.02*deg_to_rad, θ1c=3*deg_to_rad)
            _, trim_state_vals = trimSolve(rotor=self.rotor, Thrust_Needed=T_Needed, Ω=20, θ0_initial=5.015*deg_to_rad, θ1s_initial=-4.02*deg_to_rad, θ1c_initial=3*deg_to_rad, W=self.dryweight, D=D, coning_angle_iterations=2, β0_step_fraction=1.00, iterations=10, relaxation=0.8, verbose=False)
            trim_power = trim_state_vals[2]
            V_power[V] = trim_power
            if trim_power < least_power:
                least_power = trim_power
                best_speed = V
        return best_speed, least_power
    
    def get_endurance(self, dV, V_start, V_end, dt=600):
        fuel_weight_available = self.fuelweight*(1 - self.fuel_reserve_fraction)
        weight = self.dryweight + self.fuelweight
        total_time = 0
        while fuel_weight_available > 0:
            best_speed, least_power = self.get_max_endurance_speed(weight=weight, dV=dV, V_start=V_start, V_end=V_end)
            fuel_consumed = least_power * dt / self.fuel_specific_energy
            total_time += dt
            fuel_weight_available -= fuel_consumed
            weight -= fuel_consumed
        
        return total_time
    
    
    
        
            
            
        
        
        
        
            
        
    # def set_flight_parameters(self):
    #     self.flight_parameters_set = True
    #     self.dry_weight = float(input("Enter the dry weight of the helicopter in kg: "))
    #     self.fuel_weight = float(input("Enter the fuel weight of the helicopter in kg: "))
    #     self.fuel_specific_energy = 1000*float(input("Enter the fuel specific energy in kJ/kg: "))
    #     reserve_fuel_fraction  = float(input("Enter the reserve fuel fraction (e.g., 0.2 for 20%): "))
    #     self.reserved_fuel = self.fuel_weight * reserve_fuel_fraction
    #     self.fuel_weight -= self.reserved_fuel
    #     self.dry_weight += self.reserved_fuel
    #     # self.min_fuel = self.fuel_weight * reserve_fuel_fraction
    #     self.max_fuel = self.fuel_weight + self.reserved_fuel

    # def set_flight_parameters_programmatic(self, dry_weight: float, fuel_weight: float, 
    #                                       fuel_specific_energy_kj_kg: float, reserve_fuel_fraction: float):
    #     """Set flight parameters programmatically for testing"""
    #     self.flight_parameters_set = True
    #     self.dry_weight = dry_weight
    #     self.fuel_weight = fuel_weight
    #     self.fuel_specific_energy = 1000 * fuel_specific_energy_kj_kg  # Convert kJ/kg to J/kg
    #     self.reserved_fuel = self.fuel_weight * reserve_fuel_fraction
    #     self.fuel_weight -= self.reserved_fuel
    #     self.dry_weight += self.reserved_fuel
    #     # self.min_fuel = self.fuel_weight * reserve_fuel_fraction
    #     self.max_fuel = self.fuel_weight + self.reserved_fuel


    
    
