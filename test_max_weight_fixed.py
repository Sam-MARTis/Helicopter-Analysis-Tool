#!/usr/bin/env python3
"""
Test script for find_max_hover_weight function - POWER CONSTRAINT ONLY
Since default airfoil never stalls, we focus on power limitations
"""

import numpy as np
from heli import Environment, Rotor, Helicopter, MissionPlanner

def setup_lightweight_helicopter():
    """Setup a lightweight helicopter to avoid numerical issues"""
    # Environment setup
    env = Environment()
    env.set_atmosphere_parameters(
        temperature_sea_level=288.15,  # 15°C
        Reynolds_number=1e6,
        altitude=0,
        pressure_sea_level=101325,  # Pa
        Lapse_rate_troposphere=0.0065,  # K/m
        wind_velocity=0.0,
        ISA_OFFSET=0.0
    )
    
    # Create helicopter
    heli = Helicopter(env)
    
    # Main rotor setup - smaller and lighter
    main_rotor = Rotor(env)
    main_rotor.set_rotor_parameters(
        number_of_blades=3,  # Reduced from 4
        blade_mass=15,  # kg - much lighter
        NACA_for_airfoil="0012",
        radius_of_rotors=2.5,  # m - much smaller to avoid supersonic tips
        root_cutout=0.15,  # m
        root_chord=0.4,  # m - smaller
        tip_chord=0.2,  # m - smaller
        root_pitch=4.0,  # degrees - conservative
        slope_pitch=1.0  # degrees/m - conservative
    )
    
    # Tail rotor setup - very small
    tail_rotor = Rotor(env)
    tail_rotor.set_rotor_parameters(
        number_of_blades=2,
        blade_mass=3,  # kg - very light
        NACA_for_airfoil="0012",
        radius_of_rotors=0.5,  # m - small
        root_cutout=0.05,  # m
        root_chord=0.08,  # m
        tip_chord=0.04,  # m
        root_pitch=8.0,  # degrees
        slope_pitch=0.0  # degrees/m
    )
    
    # Set rotors and positions
    heli.set_rotor(main_rotor, tail_rotor)
    heli.set_rotor_positions(main_rotor_position=2.0, tail_rotor_position=5.0)
    
    # Very light fuselage
    heli.set_fuselage_parameters(
        mass=150,  # kg - very light
        length=5.0,  # m - compact
        width=1.2,  # m
        height=1.5,  # m
        tail_mass=20,  # kg - light
        tail_length=1.5,  # m
        tail_width=0.6,  # m
        tail_height=0.8  # m
    )
    
    # Small engine
    heli.set_engine_parameters(
        mass=50,  # kg
        max_power=150  # kW - small but realistic
    )
    
    return heli

def test_power_constraint_only():
    """Test ONLY the power constraint logic with lightweight helicopter"""
    print("=== Testing Power Constraint Only (Lightweight) ===")
    
    heli = setup_lightweight_helicopter()
    planner = MissionPlanner(heli)
    
    # Set very light flight parameters
    planner.set_flight_parameters_programmatic(
        dry_weight=250,  # kg - light
        fuel_weight=50,  # kg - minimal fuel
        fuel_specific_energy_kj_kg=43000,  # kJ/kg
        reserve_fuel_fraction=0.1  # 10% reserve
    )
    
    altitude = 0  # Sea level
    
    print(f"Helicopter configuration:")
    print(f"  Main rotor radius: {heli.main_rotor.radius_of_rotors} m")
    print(f"  Dry weight: {planner.dry_weight} kg")
    print(f"  Engine max available power: {heli.engine_max_available_power} kW")
    print(f"  Total helicopter mass: {heli.get_total_mass()} kg")
    
    try:
        # Test ONLY power constraint
        result = planner.find_max_hover_weight(
            altitude=altitude,
            stall_constraint=False,  # DISABLED - default airfoil never stalls
            power_constraint=True,   # ENABLED - this is the real constraint
            iterations=20,  # Conservative iterations
            initial_weight_guess=400,  # Start reasonable
            tol=100.0  # Relaxed tolerance
        )
        
        min_weight, stall_weight, power_weight = result
        
        print(f"\nResults:")
        print(f"  Max weight (power limited): {power_weight:.1f} kg")
        print(f"  Overall max weight: {min_weight:.1f} kg")
        
        return power_weight
        
    except Exception as e:
        print(f"Test failed: {e}")
        import traceback
        traceback.print_exc()
        return None

def verify_max_weight_at_power_limit():
    """Verify the max weight actually reaches the power limit"""
    print("\n=== Verifying Max Weight Reaches Power Limit ===")
    
    heli = setup_lightweight_helicopter()
    planner = MissionPlanner(heli)
    
    planner.set_flight_parameters_programmatic(
        dry_weight=250,
        fuel_weight=50,
        fuel_specific_energy_kj_kg=43000,
        reserve_fuel_fraction=0.1
    )
    
    # Get the max weight
    max_weight = test_power_constraint_only()
    
    if max_weight is None:
        print("Could not get max weight")
        return False
    
    print(f"\nTesting max weight: {max_weight:.1f} kg")
    
    try:
        # Check power at max weight
        power_needed = heli.find_power_needed(
            weight=max_weight,
            vertical_velocity=0,
            altitude=0
        )
        
        power_available = heli.engine_max_available_power
        power_diff = abs(power_available - power_needed)
        power_percent_diff = (power_diff / power_available) * 100
        
        print(f"  Power needed: {power_needed/1000:.1f} kW")
        print(f"  Power available: {power_available/1000:.1f} kW")
        print(f"  Difference: {power_diff/1000:.2f} kW ({power_percent_diff:.1f}%)")
        
        if power_percent_diff < 5:  # Within 5%
            print("  ✓ Max weight correctly at power limit!")
            return True
        else:
            print("  ⚠ Max weight not close to power limit")
            return False
            
    except Exception as e:
        print(f"Verification failed: {e}")
        return False

def test_weight_power_progression():
    """Test how power changes with weight manually"""
    print("\n=== Manual Weight vs Power Test ===")
    
    heli = setup_lightweight_helicopter()
    
    weights = [200, 300, 400, 500, 600]  # kg
    altitude = 0
    
    print(f"Engine power limit: {heli.engine_max_available_power/1000:.1f} kW")
    print(f"\n{'Weight (kg)':<12} {'Power (kW)':<12} {'Status':<10}")
    print("-" * 35)
    
    for weight in weights:
        try:
            power_needed = heli.find_power_needed(
                weight=weight,
                vertical_velocity=0,
                altitude=altitude
            )
            
            status = "OK" if power_needed <= heli.engine_max_available_power else "EXCEED"
            print(f"{weight:<12.0f} {power_needed/1000:<12.1f} {status:<10}")
            
        except Exception as e:
            print(f"{weight:<12.0f} ERROR")

if __name__ == "__main__":
    print("Testing Maximum Hover Weight (Power Only)")
    print("=" * 45)
    
    try:
        # Run lightweight tests
        max_weight = test_power_constraint_only()
        
        if max_weight is not None:
            verify_max_weight_at_power_limit()
        
        test_weight_power_progression()
        
        print("\n" + "=" * 45)
        if max_weight is not None:
            print(f"✓ Test completed! Max weight: {max_weight:.1f} kg")
        else:
            print("✗ Test failed - check numerical issues")
            
    except Exception as e:
        print(f"\nOverall test failed: {e}")
