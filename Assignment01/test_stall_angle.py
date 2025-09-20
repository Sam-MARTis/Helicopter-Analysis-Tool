#!/usr/bin/env python3
"""
Test script to find the stall angle of a rotor with NACA2412 airfoil data
"""

import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
import os

class Rotor:
    def __init__(self, num_blades=3, radius=6.6, root_cutout=0.3, 
                 root_chord=0.9, tip_chord=0.5, root_pitch=10.0, pitch_slope=3.0):
        """
        Initialize a rotor object with the given parameters
        
        Parameters:
        -----------
        num_blades : int
            Number of blades in the rotor
        radius : float
            Rotor radius in meters
        root_cutout : float
            Root cutout distance in meters
        root_chord : float
            Chord length at the root in meters
        tip_chord : float
            Chord length at the tip in meters
        root_pitch : float
            Pitch angle at the root in degrees
        pitch_slope : float
            Rate of change of pitch angle with radius in degrees/meter
        """
        self.num_blades = num_blades
        self.radius = radius
        self.root_cutout = root_cutout
        self.root_chord = root_chord
        self.tip_chord = tip_chord
        self.root_pitch = root_pitch
        self.pitch_slope = pitch_slope
        self.stall_angle = None
        
    def get_chord_length(self, r):
        """Calculate chord length at position r along the blade"""
        return self.root_chord - ((self.root_chord - self.tip_chord) / 
                                 (self.radius - self.root_cutout)) * (r - self.root_cutout)
    
    def get_pitch_angle(self, r):
        """Calculate pitch angle in degrees at position r along the blade"""
        return self.root_pitch + self.pitch_slope * (r - self.root_cutout)
    
    def get_phi(self, r, omega, climb_velocity):
        """Calculate inflow angle phi at position r along the blade"""
        # Simplified inflow model for this example
        # In a real implementation, this would use the inflow model from the assignment
        inflow_ratio = climb_velocity / (omega * self.radius)
        return np.arctan2(inflow_ratio * self.radius, r)
    
    def get_alpha_effective(self, r, omega, climb_velocity):
        """Calculate effective angle of attack at position r along the blade"""
        pitch_rad = np.deg2rad(self.get_pitch_angle(r))
        phi = self.get_phi(r, omega, climb_velocity)
        return np.rad2deg(pitch_rad - phi)
    
    def is_stalling(self, r, omega, climb_velocity):
        """Check if the blade section at radius r is stalling"""
        if self.stall_angle is None:
            raise ValueError("Stall angle has not been set. Call set_stall_angle first.")
        
        alpha_eff = self.get_alpha_effective(r, omega, climb_velocity)
        return alpha_eff > self.stall_angle
    
    def set_stall_angle(self, angle):
        """Set the stall angle for the rotor in degrees"""
        self.stall_angle = angle
        
    def is_rotor_stalling(self, omega, climb_velocity, num_sections=20):
        """Check if any part of the rotor is stalling"""
        if self.stall_angle is None:
            raise ValueError("Stall angle has not been set. Call set_stall_angle first.")
            
        radial_positions = np.linspace(self.root_cutout, self.radius, num_sections)
        
        for r in radial_positions:
            if self.is_stalling(r, omega, climb_velocity):
                return True, r
        
        return False, None


def get_stall_angle_from_airfoil_data(csv_filepath):
    """
    Analyze airfoil data to find the stall angle
    
    Parameters:
    -----------
    csv_filepath : str
        Path to the CSV file containing airfoil data
        
    Returns:
    --------
    stall_angle : float
        The stall angle in degrees
    """
    try:
        print(f"Importing airfoil data from: {os.path.basename(csv_filepath)}")
        # Read the CSV file - the NACA2412.csv file has headers already
        df = pd.read_csv(csv_filepath)
        
        # Find the angle of attack with maximum lift coefficient
        max_cl_idx = df["cl"].idxmax()
        stall_angle = df.iloc[max_cl_idx]["alpha"]
        max_cl = df.iloc[max_cl_idx]["cl"]
        
        # Print summary of the airfoil data
        print("\nAIRFOIL DATA ANALYSIS:")
        print("---------------------")
        print(f"Alpha range: {df['alpha'].min():.2f}° to {df['alpha'].max():.2f}°")
        print(f"Maximum CL: {max_cl:.3f} at alpha = {stall_angle:.2f}°")
        print(f"Stall angle detected at: {stall_angle:.2f}°")
        
        return stall_angle
        
    except Exception as e:
        print(f"Error analyzing airfoil data: {e}")
        return 15.0  # Default stall angle if data analysis fails


def test_rotor_stalling(rotor, stall_angle, omegas, climb_velocity=0.0):
    """
    Test if the rotor stalls at different angular velocities
    
    Parameters:
    -----------
    rotor : Rotor
        The rotor object to test
    stall_angle : float
        The stall angle in degrees
    omegas : list
        List of angular velocities to test in rad/s
    climb_velocity : float
        Climb velocity in m/s
    """
    print("\nTESTING STALL AT DIFFERENT OMEGAS:")
    print("--------------------------------")
    print(f"Climb velocity: {climb_velocity:.1f} m/s\n")
    
    first_stall_omega = None
    first_stall_radius = None
    first_stall_alpha = None
    
    for omega in omegas:
        stalling, stall_radius = rotor.is_rotor_stalling(omega, climb_velocity)
        
        if stalling:
            alpha_effective = rotor.get_alpha_effective(stall_radius, omega, climb_velocity)
            print(f"Omega: {omega:.1f} rad/s - Stalling: {stalling}")
            print(f"  - First stall at r = {stall_radius:.2f} m")
            print(f"  - Alpha effective: {alpha_effective:.2f}°")
            print(f"  - Alpha stall: {stall_angle:.2f}°")
            
            if first_stall_omega is None:
                first_stall_omega = omega
                first_stall_radius = stall_radius
                first_stall_alpha = alpha_effective
                
                # Analyze stall in more detail for the first stalling omega
                print(f"\nSTALL ANALYSIS AT OMEGA = {omega:.1f} rad/s:")
                print("----------------------------------")
                print("Radial    Alpha     Stalling")
                print("Position  Effective")
                print("-------------------------------")
                
                radial_positions = np.linspace(rotor.root_cutout, rotor.radius, 9)
                for r in radial_positions:
                    alpha_eff = rotor.get_alpha_effective(r, omega, climb_velocity)
                    is_stalling = alpha_eff > stall_angle
                    print(f"{r:.2f} m    {alpha_eff:.2f}°    {is_stalling}")
                
        else:
            print(f"Omega: {omega:.1f} rad/s - Stalling: {stalling}")
    
    if first_stall_omega is not None:
        print(f"\nThe rotor begins stalling at omega = {first_stall_omega:.1f} rad/s")
        print(f"Stalling occurs first at the {get_position_name(first_stall_radius, rotor)} (r = {first_stall_radius:.2f} m)")
        print(f"with alpha_effective = {first_stall_alpha:.2f}°")


def get_position_name(radius, rotor):
    """Get a descriptive name for a position on the blade"""
    relative_pos = (radius - rotor.root_cutout) / (rotor.radius - rotor.root_cutout)
    
    if relative_pos < 0.2:
        return "root"
    elif relative_pos > 0.8:
        return "tip"
    else:
        return "middle section"


def main():
    """Main function to test the stall angle"""
    print("Testing Stall Angle with NACA2412 Airfoil")
    print("=======================================")
    
    # Define the rotor parameters
    rotor = Rotor(
        num_blades=3,         # Number of blades
        radius=6.6,           # Radius in meters
        root_cutout=0.3,      # Root cutout in meters
        root_chord=0.5,       # Root chord in meters
        tip_chord=0.4,        # Tip chord in meters
        root_pitch=10.0,      # Root pitch in degrees
        pitch_slope= -2       # Pitch slope in degrees/meter
    )
    
    # Find the stall angle from airfoil data
    csv_filepath = "naca2412.csv"
    stall_angle = get_stall_angle_from_airfoil_data(csv_filepath)
    
    # Set the stall angle for the rotor
    rotor.set_stall_angle(stall_angle)
    
    # Print rotor configuration
    print("\nROTOR CONFIGURATION:")
    print("------------------")
    print(f"Number of blades: {rotor.num_blades}")
    print(f"Radius: {rotor.radius:.2f} m")
    print(f"Root cutout: {rotor.root_cutout:.2f} m")
    print(f"Chord: {rotor.root_chord:.2f} m (root) to {rotor.tip_chord:.2f} m (tip)")
    print(f"Pitch: {rotor.root_pitch:.2f}° (root) + {rotor.pitch_slope:.2f}°/m (slope)")
    
    print("\nSTALL DETECTION RESULTS:")
    print("----------------------")
    print(f"Stall angle from rotor object: {rotor.stall_angle:.2f}°")
    
    # Test stall at different angular velocities
    omegas = [5*i for i in range(1, 30)]
    test_rotor_stalling(rotor, stall_angle, omegas)


if __name__ == "__main__":
    main()
