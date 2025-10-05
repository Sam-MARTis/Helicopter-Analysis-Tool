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








