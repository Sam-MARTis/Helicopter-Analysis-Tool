#!/usr/bin/env python3
"""
Simple test with realistic parameters to verify max weight logic
"""

import numpy as np
from heli import Environment, Rotor, Helicopter, MissionPlanner

def test_simple_case():
    """Test with very simple, realistic parameters"""
    print("=== Simple Max Weight Test ===")
    
    # Setup environment
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
    
    # Create helicopter
    heli = Helicopter(env)
    
    # Small main rotor to avoid supersonic issues
    main_rotor = Rotor(env)
    main_rotor.set_rotor_parameters(
        number_of_blades=3,
        blade_mass=30,
        NACA_for_airfoil="0012",
        radius_of_rotors=3.0,  # Small radius
        root_cutout=0.15,
        root_chord=0.4,
        tip_chord=0.2,
        root_pitch=6.0,
        slope_pitch=2.0
    )
    
    # Small tail rotor
    tail_rotor = Rotor(env)
    tail_rotor.set_rotor_parameters(
        number_of_blades=2,
        blade_mass=5,
        NACA_for_airfoil="0012",
        radius_of_rotors=0.5,
        root_cutout=0.05,
        root_chord=0.1,
        tip_chord=0.05,
        root_pitch=8.0,
        slope_pitch=0.0
    )
    
    heli.set_rotor(main_rotor, tail_rotor)
    heli.set_rotor_positions(3.0, 8.0)
    
    # Light fuselage
    heli.set_fuselage_parameters(
        mass=300, length=8.0, width=1.5, height=2.0,
        tail_mass=50, tail_length=2.0, tail_width=0.8, tail_height=1.0
    )
    
    # Moderate engine
    heli.set_engine_parameters(mass=80, max_power=300)  # 300 kW
    
    # Mission planner
    planner = MissionPlanner(heli)
    planner.set_flight_parameters_programmatic(
        dry_weight=800,
        fuel_weight=200,
        fuel_specific_energy_kj_kg=43000,
        reserve_fuel_fraction=0.1
    )
    
    print(f"Configuration:")
    print(f"  Main rotor radius: {main_rotor.radius_of_rotors} m")
    print(f"  Max tip speed at 40 rad/s: {40 * main_rotor.radius_of_rotors} m/s")
    print(f"  Speed of sound: ~343 m/s")
    print(f"  Engine power: {heli.engine_max_available_power} kW")
    
    # Test at different starting weights
    test_weights = [1000, 1200, 1500, 1800, 2000]
    
    print(f"\n{'Start Weight (kg)':<16} {'Power Needed (kW)':<18} {'Within Limit?':<12}")
    print("-" * 50)
    
    for weight in test_weights:
        try:
            power_needed = heli.find_power_needed(
                weight=weight,
                vertical_velocity=0,
                altitude=0
            )
            
            within_limit = power_needed <= heli.engine_max_available_power
            
            print(f"{weight:<16.0f} {power_needed/1000:<18.1f} {within_limit!s:<12}")
            
        except Exception as e:
            print(f"{weight:<16.0f} ERROR: {str(e)[:20]}")
    
    # Now test the max weight function
    print(f"\n=== Testing find_max_hover_weight ===")
    try:
        result = planner.find_max_hover_weight(
            altitude=0,
            stall_constraint=False,  # Disable stall (default airfoil never stalls)
            power_constraint=True,   # Only power constraint
            iterations=20,           # Fewer iterations
            initial_weight_guess=1000,
            tol=100.0  # Relaxed tolerance (100W)
        )
        
        min_weight, stall_weight, power_weight = result
        
        print(f"Results:")
        print(f"  Power-limited max weight: {power_weight:.1f} kg")
        print(f"  Overall max weight: {min_weight:.1f} kg")
        
        # Verify this makes sense
        if power_weight > 0:
            print(f"\n✓ Found reasonable max weight!")
            
            # Quick verification
            test_power = heli.find_power_needed(power_weight, 0, 0)
            print(f"  Verification - Power at max weight: {test_power/1000:.1f} kW")
            print(f"  Power limit: {heli.engine_max_available_power/1000:.1f} kW")
        else:
            print(f"\n✗ Negative weight indicates numerical issues")
            
    except Exception as e:
        print(f"Max weight test failed: {e}")

if __name__ == "__main__":
    test_simple_case()
