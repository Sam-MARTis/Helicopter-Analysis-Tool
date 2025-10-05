from typing import Optional, Tuple, Dict, List, Union
import numpy as np
from time import sleep
from numba import njit
import matplotlib.pyplot as plt



def get_Parasitic_Drag(ρ, f, Vinfty):
    return ρ * f * Vinfty * Vinfty *0.5

def get_Thrust_Total(W, D):
    return np.sqrt(W*W + D*D)

def get_α_tpp(W, D):
    return np.arctan2(D, W)

def get_Area_Disc(R):
    return np.pi * R * R

def get_Ct(T, ρ, A, Ω, R):
    return T / (ρ * A * (Ω * R) ** 2)

def get_mu(Vinfty, α_tpp, Ω, R):
    return Vinfty * np.cos(α_tpp) / (Ω * R)

# Now with Glaubert formulae
def get_λi_Glaubert(Ct, mu):
    assert mu >= 0.2
    return Ct/(2*mu)

def get_λG(λi, Vinfty, α_tpp, Ω, R):
    return λi + (Vinfty * np.sin(α_tpp)) / (Ω * R)
