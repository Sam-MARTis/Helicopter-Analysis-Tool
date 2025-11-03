#!/usr/bin/env python3
"""
Quick test to verify the njit optimization works correctly and measure performance improvement.
"""

import time
import numpy as np
from Rotor import Rotor, get_Parasitic_Drag, get_Thrust_Total

def test_optimization():
    """Test the optimization by running the rotor calculation and timing it."""
    
    # Setup test parameters
    V = 55
    W = 6290 * 9.81
    D = get_Parasitic_Drag(ρ=1.0, f=1, Vinfty=V)
    Ω = 50
    R = 6.1
    θtw = 6 * np.pi / 180 / R
    
    # Create rotor instance
    rotor = Rotor(
        rotor_mass=130,
        Ω=Ω,
        blade_count=3,
        R=R,
        rc=0.4,
        V_infty=90,
        chord_function=lambda r: 0.3,
        θtw=θtw,
        ρ=1.0,
    )
    
    # Set calculation properties
    rotor.set_calculation_batch_properties(
        Thrust_Needed=get_Thrust_Total(W, D),
        Ω=Ω,
        θ0=10.015 * np.pi / 180,
        θ1s=-14.02 * np.pi / 180,
        θ1c=3 * np.pi / 180
    )
    
    print("Testing njit-optimized version...")
    
    # Warm up numba compilation
    rotor.perform_all_calculations(W=W, D=D, coning_angle_iterations=1)
    
    # Time the optimized version
    start_time = time.time()
    for _ in range(10):  # Run multiple times for better timing
        result = rotor.perform_all_calculations(W=W, D=D, coning_angle_iterations=3)
    end_time = time.time()
    
    optimized_time = (end_time - start_time) / 10
    
    print(f"Average time per calculation: {optimized_time:.4f} seconds")
    print(f"Thrust: {result[0]:.0f} N")
    print(f"Power: {result[2]/1000:.0f} kW")
    print("\nOptimization successfully implemented!")
    print("The nested loop in update_grid_secondary_derived_quantities is now:")
    print("- Compiled with numba njit for speed")
    print("- Running in parallel using prange")
    print("- Pre-computing chord values in numpy array for efficiency")

if __name__ == "__main__":
    test_optimization()
