#!/usr/bin/env python3
"""
Test script for checking rotor stalling with custom airfoil data
"""

import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
from heli import Helicopter, Rotor, Environment
import sys
import os

def setup_test_rotor(env, csv_filepath=None):
    """Setup a test rotor with specified dimensions from self_max_takeoff_weight.py"""
    rotor = Rotor(environment=env)
    
    # Using same dimensions as in self_max_takeoff_weight.py
    rotor.set_rotor_parameters(
        number_of_blades=3,
        blade_mass=0,
        NACA_for_airfoil="0012",
        radius_of_rotors=6.6,
        root_cutout=0.3,
        root_chord=0.9,
        tip_chord=0.5,
        root_pitch=10.0,  # degrees
        slope_pitch=3.0,  # degrees/m
        filepath=csv_filepath  # This will use the custom CSV file
    )
    
    return rotor

def find_stall_angle(rotor):
    """Find the stall angle from the rotor object"""
    if rotor.default_airfoil:
        print("WARNING: Using default airfoil which doesn't model stall")
        return None
    
    # Access the stall angle from the rotor object
    # This was calculated during _build_aero_polynomials
    if hasattr(rotor, 'alpha_stall'):
        return rotor.alpha_stall
    else:
        print("ERROR: No stall angle found in rotor object")
        return None

def analyze_airfoil_data(csv_filepath):
    """Analyze and plot the airfoil data from the CSV file"""
    try:
        # Read the CSV file
        df = pd.read_csv(csv_filepath, delim_whitespace=True, 
                         names=["alpha", "cl", "cd", "cm"], comment='#')
        
        # Plot Cl vs alpha
        plt.figure(figsize=(12, 8))
        
        plt.subplot(2, 2, 1)
        plt.plot(df["alpha"], df["cl"], 'b-', marker='o')
        plt.grid(True)
        plt.xlabel('Angle of Attack (degrees)')
        plt.ylabel('Lift Coefficient (Cl)')
        plt.title('Cl vs Alpha')
        
        # Plot Cd vs alpha
        plt.subplot(2, 2, 2)
        plt.plot(df["alpha"], df["cd"], 'r-', marker='o')
        plt.grid(True)
        plt.xlabel('Angle of Attack (degrees)')
        plt.ylabel('Drag Coefficient (Cd)')
        plt.title('Cd vs Alpha')
        
        # Plot Cl/Cd ratio vs alpha
        plt.subplot(2, 2, 3)
        plt.plot(df["alpha"], df["cl"]/df["cd"], 'g-', marker='o')
        plt.grid(True)
        plt.xlabel('Angle of Attack (degrees)')
        plt.ylabel('Lift/Drag Ratio')
        plt.title('L/D Ratio vs Alpha')
        
        # Plot polar (Cl vs Cd)
        plt.subplot(2, 2, 4)
        plt.plot(df["cd"], df["cl"], 'm-', marker='o')
        plt.grid(True)
        plt.xlabel('Drag Coefficient (Cd)')
        plt.ylabel('Lift Coefficient (Cl)')
        plt.title('Drag Polar')
        
        plt.tight_layout()
        plt.savefig('airfoil_analysis.png')
        plt.close()
        
        print(f"Airfoil data plots saved to 'airfoil_analysis.png'")
        return True
    
    except Exception as e:
        print(f"Error analyzing airfoil data: {e}")
        return False

def test_is_rotor_stalling(rotor, csv_filepath):
    """Test the is_rotor_stalling function with various omegas"""
    if rotor.default_airfoil:
        print("Cannot test stalling with default airfoil - it never stalls")
        return
    
    print("\n=== Testing is_rotor_stalling function ===")
    
    # Find stall angle
    stall_angle = find_stall_angle(rotor)
    if stall_angle is not None:
        print(f"Airfoil stall angle: {np.degrees(stall_angle):.2f} degrees")
        print(f"Stall cutoff angle (93% of stall): {np.degrees(rotor.alpha_cutoff):.2f} degrees")
    
    # Calculate polynomial fit details 
    if hasattr(rotor, 'cl_poly'):
        alpha_range = np.linspace(-5, 25, 100)
        cl_values = [rotor.cl_poly(a) for a in alpha_range]
        max_cl_idx = np.argmax(cl_values)
        max_cl_alpha = alpha_range[max_cl_idx]
        max_cl = cl_values[max_cl_idx]
        print(f"Peak Cl from polynomial: {max_cl:.3f} at {max_cl_alpha:.2f} degrees")
    
    # Test stalling at different omega values
    print("\nTesting stalling at different omegas:")
    print(f"{'Omega (rad/s)':<15} {'Stalling?':<10} {'Details':<40}")
    print("-" * 65)
    
    # Test a range of omega values
    for omega in [10, 20, 30, 40, 50, 60]:
        try:
            # Check if rotor is stalling
            is_stalling = rotor.is_rotor_stalling(vertical_velocity=0, omega=omega)
            
            # Get details for each blade section
            details = []
            divisions = 10
            dr = (rotor.radius_of_rotors - rotor.root_cutout) / divisions
            
            for i in range(divisions):
                r_distance = rotor.root_cutout + (i+0.5) * dr
                # Calculate effective angle of attack
                lambda_inflow, _ = rotor.elemental_inflow_ratio(r_distance, 0, omega)
                alpha_eff = rotor.get_alpha_effective_r(r_distance, 0, omega, lambda_inflow)
                alpha_eff_deg = np.degrees(alpha_eff)
                
                # Check if this section is stalling
                _, section_stalling = rotor.get_cL(r_distance, 0, omega, lambda_inflow)
                
                if section_stalling:
                    details.append(f"r={r_distance:.2f}m, α={alpha_eff_deg:.1f}°")
            
            status = "STALLING" if is_stalling else "OK"
            details_str = ", ".join(details) if details else "No stalling sections"
            
            print(f"{omega:<15.1f} {status:<10} {details_str}")
            
        except Exception as e:
            print(f"{omega:<15.1f} ERROR: {str(e)}")
    
def main():
    """Main function for testing rotor stalling"""
    if len(sys.argv) < 2:
        print("Usage: python test_rotor_stalling.py <airfoil_csv_file>")
        return
    
    csv_filepath = sys.argv[1]
    
    # Check if file exists
    if not os.path.exists(csv_filepath):
        print(f"Error: File '{csv_filepath}' does not exist!")
        return
    
    print(f"Testing rotor stalling with airfoil data from: {csv_filepath}")
    
    # Setup environment
    env = Environment()
    env.set_atmosphere_parameters(
        temperature_sea_level=288.15,
        Reynolds_number=1e6,
        altitude=0.0,
        pressure_sea_level=101325,
        Lapse_rate_troposphere=0.0065,
        wind_velocity=0,
        ISA_OFFSET=0
    )
    
    # Create rotor with custom airfoil data
    rotor = setup_test_rotor(env, csv_filepath)
    
    # Check if using custom airfoil
    if rotor.default_airfoil:
        print("WARNING: Failed to load custom airfoil. Using default airfoil which doesn't model stall.")
    else:
        print("Successfully loaded custom airfoil data.")
        
        # Analyze and plot airfoil data
        analyze_airfoil_data(csv_filepath)
        
        # Test stalling
        test_is_rotor_stalling(rotor, csv_filepath)
        
if __name__ == "__main__":
    main()
