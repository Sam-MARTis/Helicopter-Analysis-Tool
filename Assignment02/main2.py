from typing import Optional, Tuple, Dict, List, Union
import numpy as np
from time import sleep
from numba import njit
import matplotlib.pyplot as plt
from numpy import sin, cos, tan, sqrt, atan2, atan

deg_to_rad = np.pi / 180

θtw = 1 * deg_to_rad

def get_Parasitic_Drag(ρ, f, Vinfty):
    return ρ * f * Vinfty * Vinfty *0.5

def get_Thrust_Total(W, D):
    return sqrt(W*W + D*D)

def get_α_tpp(W, D):
    return atan2(D, W)



class Blade:
    def __init__(self, R, rc, θtw, chord_function):
        self.R = R
        self.rc = rc
        self.θtw = θtw
        self.a = 5.75
        self.chord_function = chord_function
        self.rho = 1.225
        self.θ0 = 10 * deg_to_rad
        self.θ1s = -5 * deg_to_rad
        self.θ1c = -0.01 * deg_to_rad
        self.Thrust_Needed = 0
        self.Ω = 40
        self.Vinfty  = 110
        self.α_tpp = 0
        self.mu = 0
        self.A = 0
        self.Ct = 0
        self.I = 0.6
        self.r_divisions = 20
        self.ψ_divisions = 40
        self.dr = (self.R - self.rc)/ self.r_divisions
        self.dψ = 2 * np.pi / self.ψ_divisions
        

        # Quantities to store: v, α_effective, dL, dD, v, dTorque, 
        self.mesh = np.zeros((self.r_divisions, self.ψ_divisions, 6))
    def set_mesh(self, r_divisions, ψ_divisions):
        self.r_divisions = r_divisions
        self.ψ_divisions = ψ_divisions
        self.dr = (self.R - self.rc)/ self.r_divisions
        self.dψ = 2 * np.pi / self.ψ_divisions
        self.mesh = np.zeros((self.r_divisions, self.ψ_divisions, 6))

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
    def get_Area_Disc(self):
        self.A = np.pi * self.R * self.R
        return self.A

    def get_Ct(self):
        self.Ct = self.Thrust_Needed / (self.rho * self.A * (self.Ω * self.R) ** 2)
        return self.Ct
    
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
    
    def get_effective_aoa(self, r, ψ):
        v = self.mesh[i, j, 0]
        dβ_by_dt = -self.α_tpp *(sin(ψ)*self.Ω)
        Up = v + r*dβ_by_dt + self.Vinfty * sin(self.β0) * cos(ψ) + self.Vinfty * sin(self.α_tpp)
        Ut = self.Ω * r + self.Vinfty * cos(self.α_tpp) * sin(ψ)
        θ = self.θ1s * sin(ψ) + self.θ1c * cos(ψ) + self.θ0 + self.θtw * (r) - atan((Up) / (Ut))
        return θ
    def get_β0(self, β0_previous = None, set=False):
        β0_previous = self.β0 if β0_previous is None else β0_previous
        dr = (self.R - self.rc)/ self.r_divisions
        dψ = 
        internal_sum = 0
        for i in range(self.r_divisions):
            r = self.rc + (i + 0.5) * dr
            c = self.chord_function(r)
            θ = self.get_effective_aoa(r, 0)
            Cl = self.a * θ
            dS = 0.5 * self.rho * r*r*r * Cl * dr
            internal_sum += dS
        # return internal_sum
        β0 = internal_sum/self.I
        if set:
            self.β0 = β0
        return β0
    def update_grid_induced_velocity(self):
        for i in range(self.r_divisions):
            r = self.rc + (i + 0.5) * (self.R - self.rc)/ self.r_divisions
            for j in range(self.ψ_divisions):
                ψ = (j + 0.5) * 2 * np.pi / self.ψ_divisions
                v = self.get_v(r, ψ)
                θ = self.get_effective_aoa(r, ψ)
                self.discretitization[i, j, 0] = v
                self.discretitization[i, j, 1] = θ
        
        
    def caclulate_primary_derived_quantities(self, W, D, coning_angle_iterations = 5, step_fraction = 0.6):
        self.get_Area_Disc()
        self.get_Ct()
        self.get_α_tpp(W, D)
        self.get_mu()
        self.get_λi_Glaubert()
        self.get_λG()
        
        for _ in range(coning_angle_iterations):
            
            β0 = self.get_β0()
            dβ0 = β0 - self.β0
            self.β0 = self.β0 + step_fraction * dβ0
        return self.β0
    
    
    

            


def get_Area_Disc(R):
    return np.pi * R * R

def get_Ct(T, ρ, A, Ω, R):
    return T / (ρ * A * (Ω * R) ** 2)

def get_mu(Vinfty, α_tpp, Ω, R):
    return Vinfty * cos(α_tpp) / (Ω * R)

# Now with Glaubert formulae
def get_λi_Glaubert(Ct, mu):
    assert mu >= 0.2
    return Ct/(2*mu)

def get_λG(λi, Vinfty, α_tpp, Ω, R):
    return λi + (Vinfty * sin(α_tpp)) / (Ω * R)

def get_λi(r, ψ, λG, λi_Glaubert, mu, R):
    num = (4/3) * (mu/λG) * r * cos(ψ)
    den = (1.2 + (mu/λG))*R 
    return λi_Glaubert*(1 + (num/den))

def get_v_induced(λi, Ω, R):   
    return λi * Ω * R


def get_effective_aoa(θ1s, θ1c, θ0, θtw, r, ψ, v, Ω, Vinfty, α_tpp, β0):
    dβ_by_dt = -α_tpp *(sin(ψ)*Ω)
    Up = v + r*dβ_by_dt + Vinfty * sin(β0) * cos(ψ) + Vinfty * sin(α_tpp)
    Ut = Ω * r + Vinfty * cos(α_tpp) * sin(ψ)
    θ = θ1s * sin(ψ) + θ1c * cos(ψ) + θ0 + θtw * (r) - atan((Up) / (Ut))
    return θ

def get_β0(Vinfty, chord_function, Ω, a, I, rc, R, ρ, divisions, β0_previous):
    dx = (R - rc)/ divisions
    for i in range(divisions):
        r = rc + (i + 0.5) * dx
        c = chord_function(r)

    
    

## ITERATION LOOP
θ1s = -5 * deg_to_rad
θ1c = -0.01 * deg_to_rad
θ0 = 10 * deg_to_rad








