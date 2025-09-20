#!/usr/bin/env python3
"""
Debug the omega vs thrust relationship to understand the fundamental issue
"""

import numpy as np
from heli import Environment, Rotor, Helicopter, MissionPlanner

def create_test_helicopter():
    """Create helicopter with minimal parameters"""
    env = Environment()
    env.set_atmosphere_parameters(
        temperature_sea_level=288.15,
        Reynolds_number=1e6,
        altitude=0,
        pressure_sea_level=101325,
        Lapse_rate_troposphere=0.0065,
        wind_velocity=0.0,
        ISA_OFFSET=0.0
    )
    
    heli = Helicopter(env)
    
    # Very small main rotor
    main_rotor = Rotor(env)
    main_rotor.set_rotor_parameters(
        number_of_blades=2,
        blade_mass=10,
        NACA_for_airfoil="0012",
        radius_of_rotors=2.0,  # Small
        root_cutout=0.1,
        root_chord=0.3,
        tip_chord=0.15,
        root_pitch=4.0,
        slope_pitch=1.0
    )
    
    # Tiny tail rotor
    tail_rotor = Rotor(env)
    tail_rotor.set_rotor_parameters(
        number_of_blades=2,
        blade_mass=2,
        NACA_for_airfoil="0012",
        radius_of_rotors=0.3,
        root_cutout=0.02,
        root_chord=0.08,
        tip_chord=0.04,
        root_pitch=6.0,
        slope_pitch=0.0
    )
    
    heli.set_rotor(main_rotor, tail_rotor)
    heli.set_rotor_positions(2.0, 5.0)
    
    # Light components
    heli.set_fuselage_parameters(
        mass=100, length=5.0, width=1.0, height=1.5,
        tail_mass=20, tail_length=1.5, tail_width=0.5, tail_height=0.8
    )
    
    # Small engine - but let's see what power we actually need
    heli.set_engine_parameters(mass=30, max_power=150)  # Increase to 150 kW
    
    return heli

def test_omega_thrust_relationship():
    """Test how omega relates to thrust for understanding convergence"""
    print("=== Omega vs Thrust Relationship ===")
    
    heli = create_test_helicopter()
    
    test_omegas = np.linspace(10, 100, 15)  # Wide range
    test_weight = 300  # kg
    
    print(f"Target weight: {test_weight} kg")
    print(f"Target thrust: {test_weight * 9.81:.1f} N")
    print(f"\n{'Omega':<8} {'Thrust (N)':<12} {'Power (kW)':<12} {'Tip Speed':<12}")
    print("-" * 50)
    
    feasible_points = []
    
    for omega in test_omegas:
        try:
            thrust = heli.find_thrust_provided(
                vertical_velocity=0,
                altitude=0,
                omega=omega
            )
            
            # Calculate power only if we're not hitting limits
            if omega <= 60:  # Below supersonic limit
                power = heli.find_power_needed(
                    weight=test_weight,
                    vertical_velocity=0,
                    altitude=0,
                    omega=omega
                )
                power_kw = power / 1000
            else:
                power_kw = float('inf')
            
            tip_speed = omega * heli.main_rotor.radius_of_rotors
            
            print(f"{omega:<8.1f} {thrust:<12.1f} {power_kw:<12.1f} {tip_speed:<12.1f}")
            
            # Check if thrust can support weight
            if thrust >= test_weight * 9.81:
                feasible_points.append((omega, thrust, power_kw))
                
        except Exception as e:
            print(f"{omega:<8.1f} ERROR: {str(e)[:20]}")
    
    print(f"\nFeasible points (can support {test_weight} kg):")
    if feasible_points:
        for omega, thrust, power in feasible_points:
            print(f"  Omega: {omega:.1f}, Thrust: {thrust:.1f} N, Power: {power:.1f} kW")
        
        min_power_point = min(feasible_points, key=lambda x: x[2])
        print(f"\nMinimum power point:")
        print(f"  Omega: {min_power_point[0]:.1f} rad/s")
        print(f"  Power: {min_power_point[2]:.1f} kW")
        
    else:
        print("  None - rotor cannot generate enough thrust")

def test_omega_convergence():
    """Test what omega is actually needed for different weights"""
    print("\n=== Omega Convergence Test ===")
    
    heli = create_test_helicopter()
    
    test_weights = [100, 200, 300, 400, 500]
    
    print(f"{'Weight (kg)':<12} {'Omega Needed':<15} {'Status':<20}")
    print("-" * 50)
    
    for weight in test_weights:
        try:
            omega_needed = heli.main_rotor.find_omega_needed_uncoupled(
                thrust_needed=weight * 9.81,  # Convert to Newtons
                vertical_velocity=0,
                altitude=0,
                initial_guess=30.0
            )
            
            if omega_needed > 0:
                status = f"Success: {omega_needed:.1f}"
            else:
                status = "Failed: Negative omega"
                
        except Exception as e:
            status = f"Error: {str(e)[:15]}"
            
        print(f"{weight:<12} {status}")

if __name__ == "__main__":
    print("Omega and Thrust Debug Analysis")
    print("=" * 35)
    
    test_omega_thrust_relationship()
    test_omega_convergence()
