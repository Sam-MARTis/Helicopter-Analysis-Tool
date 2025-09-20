#!/usr/bin/env python3
"""
Simple test to debug max weight calculation step by step
"""

import numpy as np
from heli import Environment, Rotor, Helicopter, MissionPlanner

def create_ultralight_helicopter():
    """Create an ultralight helicopter for testing"""
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
        number_of_blades=2,  # Minimal
        blade_mass=8,  # kg - very light
        NACA_for_airfoil="0012",
        radius_of_rotors=1.8,  # m - small rotor
        root_cutout=0.1,  # m
        root_chord=0.25,  # m
        tip_chord=0.15,  # m
        root_pitch=5.0,  # degrees
        slope_pitch=1.0  # degrees/m
    )
    
    # Tiny tail rotor
    tail_rotor = Rotor(env)
    tail_rotor.set_rotor_parameters(
        number_of_blades=2,
        blade_mass=2,  # kg
        NACA_for_airfoil="0012",
        radius_of_rotors=0.3,  # m - very small
        root_cutout=0.02,  # m
        root_chord=0.05,  # m
        tip_chord=0.03,  # m
        root_pitch=10.0,  # degrees
        slope_pitch=0.0  # degrees/m
    )
    
    heli.set_rotor(main_rotor, tail_rotor)
    heli.set_rotor_positions(main_rotor_position=1.5, tail_rotor_position=3.5)
    
    # Ultralight fuselage
    heli.set_fuselage_parameters(
        mass=80,   # kg - ultralight
        length=4.0,  # m
        width=1.0,   # m
        height=1.2,  # m
        tail_mass=10,  # kg
        tail_length=1.0,  # m
        tail_width=0.5,   # m
        tail_height=0.6   # m
    )
    
    # Small engine - use Watts for consistency
    heli.set_engine_parameters(
        mass=25,  # kg
        max_power=50000  # W (50 kW) - use Watts instead of kW
    )
    
    return heli

def debug_power_calculation():
    """Debug step by step what's happening in power calculation"""
    print("=== Debugging Power Calculation ===")
    
    heli = create_ultralight_helicopter()
    planner = MissionPlanner(heli)
    
    # Very light setup
    planner.set_flight_parameters_programmatic(
        dry_weight=150,  # kg - ultralight
        fuel_weight=30,   # kg - minimal
        fuel_specific_energy_kj_kg=43000,
        reserve_fuel_fraction=0.1
    )
    
    print(f"Helicopter specs:")
    print(f"  Main rotor radius: {heli.main_rotor.radius_of_rotors} m")
    print(f"  Dry weight: {planner.dry_weight} kg")
    print(f"  Total mass: {heli.get_total_mass()} kg")
    print(f"  Engine max power: {heli.engine_max_power} kW")
    print(f"  Engine available power: {heli.engine_max_available_power} kW")
    
    # Test a specific weight
    test_weight = 200  # kg
    altitude = 0
    
    print(f"\nTesting weight: {test_weight} kg")
    
    try:
        # Step 1: Find omega needed
        thrust_needed = test_weight * heli.environment.gravitational_acceleration
        print(f"  Thrust needed: {thrust_needed:.0f} N")
        
        omega_needed = heli.main_rotor.find_omega_needed_uncoupled(
            thrust_needed=thrust_needed,
            vertical_velocity=0,
            altitude=altitude,
            initial_guess=20.0
        )
        print(f"  Omega needed: {omega_needed:.2f} rad/s")
        
        # Step 2: Calculate power needed
        power_needed = heli.find_power_needed(
            weight=test_weight,
            vertical_velocity=0,
            altitude=altitude,
            omega=omega_needed
        )
        print(f"  Power needed: {power_needed/1000:.2f} kW")
        print(f"  Power available: {heli.engine_max_available_power/1000:.2f} kW")
        
        # Check if within power limit
        if power_needed <= heli.engine_max_available_power:
            print(f"  ✓ Within power limit")
        else:
            print(f"  ✗ Exceeds power limit by {(power_needed - heli.engine_max_available_power)/1000:.2f} kW")
        
        return True
        
    except Exception as e:
        print(f"  ERROR: {e}")
        return False

def test_ultralight_max_weight():
    """Test max weight with ultralight helicopter"""
    print("\n=== Testing Ultralight Max Weight ===")
    
    heli = create_ultralight_helicopter()
    planner = MissionPlanner(heli)
    
    planner.set_flight_parameters_programmatic(
        dry_weight=150,
        fuel_weight=30,
        fuel_specific_energy_kj_kg=43000,
        reserve_fuel_fraction=0.1
    )
    
    try:
        result = planner.find_max_hover_weight(
            altitude=0,
            stall_constraint=False,  # Only test power
            power_constraint=True,
            iterations=15,
            initial_weight_guess=200,
            tol=500  # Very relaxed tolerance
        )
        
        min_weight, stall_weight, power_weight = result
        
        print(f"Max weight (power limited): {power_weight:.1f} kg")
        
        if power_weight > 0:
            print("✓ Algorithm found a valid max weight!")
            return power_weight
        else:
            print("✗ Algorithm returned 0 - helicopter too weak")
            return None
            
    except Exception as e:
        print(f"Max weight test failed: {e}")
        return None

if __name__ == "__main__":
    print("Debugging Max Weight Power Constraint")
    print("=" * 40)
    
    # Step-by-step debugging
    debug_success = debug_power_calculation()
    
    if debug_success:
        max_weight = test_ultralight_max_weight()
        
        if max_weight:
            print(f"\n✓ SUCCESS: Max weight = {max_weight:.1f} kg")
        else:
            print(f"\n✗ Algorithm needs further debugging")
    else:
        print(f"\n✗ Basic power calculation failed")
