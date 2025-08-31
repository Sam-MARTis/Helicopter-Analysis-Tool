#!/usr/bin/env python3
"""
Test script to verify the corrected BEMT implementation in heli.py
"""

import sys
import os
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from heli import Environment, Rotor, Helicopter
import numpy as np

def test_bemt_implementation():
    """Test the corrected BEMT implementation"""
    print("Testing corrected BEMT implementation...")
    
    # Set up environment
    env = Environment()
    env.set_atmosphere_parameters(
        temperature_sea_level=288.15,  # 15°C
        Reynolds_number=1e6,
        altitude=0,
        pressure_sea_level=101325,
        Lapse_rate_troposphere=0.0065,
        wind_velocity=0,
        ISA_OFFSET=0
    )
    
    # Set up rotor
    rotor = Rotor(environment=env)
    rotor.set_rotor_parameters(
        number_of_blades=2,
        blade_mass=50,
        NACA_for_airfoil="0012",
        radius_of_rotors=5.0,
        root_cutout=0.5,
        angle_of_attack=8.0,  # degrees
        root_chord=0.3,
        tip_chord=0.15,
        root_pitch=12.0,  # degrees
        slope_pitch=0.1
    )
    
    # Test parameters
    climb_velocity = 0.0  # hover
    omega = 2 * np.pi * 400 / 60  # 400 RPM
    temperature = env.get_temperature()
    density = env.get_density(temperature)
    
    print(f"Test parameters:")
    print(f"  Climb velocity: {climb_velocity} m/s")
    print(f"  Omega: {omega:.2f} rad/s ({omega * 60 / (2 * np.pi):.1f} RPM)")
    print(f"  Density: {density:.3f} kg/m³")
    print()
    
    try:
        # Test elemental calculations
        r_test = 3.0  # 3m from hub
        print("Testing elemental calculations at r = 3.0m:")
        
        # Test inflow ratio calculation
        lambda_inflow, F = rotor.elemental_inflow_ratio(r_test, climb_velocity, omega)
        print(f"  Lambda inflow: {lambda_inflow:.4f}")
        print(f"  Tip loss factor F: {F:.4f}")
        
        # Test phi calculation
        phi = rotor.get_phi_r(r_test, climb_velocity, omega, lambda_inflow)
        print(f"  Flow angle phi: {np.degrees(phi):.2f}°")
        
        # Test effective angle of attack
        alpha_eff = rotor.get_alpha_effective_r(r_test, climb_velocity, omega, lambda_inflow)
        print(f"  Effective AoA: {np.degrees(alpha_eff):.2f}°")
        
        # Test coefficients
        cl = rotor.get_cL(r_test, climb_velocity, omega, lambda_inflow)
        cd = rotor.get_cD(r_test, climb_velocity, omega, lambda_inflow)
        print(f"  CL: {cl:.4f}")
        print(f"  CD: {cd:.4f}")
        
        # Test elemental thrust and torque
        dr = 0.1
        dT = rotor.elemental_thrust(r_test, dr, climb_velocity, omega, density)
        dQ = rotor.elemental_torque(r_test, dr, climb_velocity, omega, density)
        print(f"  Elemental thrust: {dT:.2f} N")
        print(f"  Elemental torque: {dQ:.4f} N⋅m")
        print()
        
        # Test total calculations
        print("Testing total rotor calculations:")
        total_T = rotor.total_thrust(climb_velocity, omega, density)
        total_Q = rotor.total_torque(climb_velocity, omega, density)
        total_P = rotor.total_power(climb_velocity, omega, density)
        
        print(f"  Total thrust: {total_T:.1f} N")
        print(f"  Total torque: {total_Q:.1f} N⋅m")
        print(f"  Total power: {total_P:.1f} W ({total_P/1000:.1f} kW)")
        print()
        
        # Test performance calculation
        print("Testing performance calculation:")
        performance = rotor.calculate_performance(climb_velocity, omega, density)
        print(f"  Performance dict keys: {list(performance.keys())}")
        print(f"  Thrust from performance: {performance.get('thrust', 'N/A')}")
        print(f"  Power from performance: {performance.get('power', 'N/A')}")
        print()
        
        print("✅ All BEMT calculations completed successfully!")
        print("✅ No runtime errors encountered!")
        
    except Exception as e:
        print(f"❌ Error encountered: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    test_bemt_implementation()
