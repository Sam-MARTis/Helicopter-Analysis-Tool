#!/usr/bin/env python3
"""
Test script for find_max_hover_weight function - POWER CONSTRAINT ONLY
Since default airfoil never stalls, we focus on power limitations
"""

import numpy as np
from heli import Environment, Rotor, Helicopter, MissionPlanner

def setup_test_helicopter():
    """Setup a test helicopter with realistic parameters"""
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
    
    # Main rotor setup - smaller, more realistic for testing
    main_rotor = Rotor(env)
    main_rotor.set_rotor_parameters(
        number_of_blades=4,
        blade_mass=25,  # kg - reduced from 75
        NACA_for_airfoil="0012",
        radius_of_rotors=3.5,  # m - reduced from 6.6 to avoid supersonic tips
        root_cutout=0.2,  # m - reduced proportionally
        root_chord=0.5,  # m - reduced from 0.9
        tip_chord=0.25,  # m - reduced from 0.5
        root_pitch=6.0,  # degrees - reduced from 8
        slope_pitch=2.0  # degrees/m - reduced from 3
    )
    
    # Tail rotor setup - proportionally smaller
    tail_rotor = Rotor(env)
    tail_rotor.set_rotor_parameters(
        number_of_blades=2,
        blade_mass=5,  # kg - reduced from 10
        NACA_for_airfoil="0012",
        radius_of_rotors=0.7,  # m - reduced from 1.0
        root_cutout=0.05,  # m - reduced
        root_chord=0.12,  # m - reduced
        tip_chord=0.06,  # m - reduced
        root_pitch=10.0,  # degrees
        slope_pitch=0.0  # degrees/m
    )
    
    # Set rotors and positions
    heli.set_rotor(main_rotor, tail_rotor)
    heli.set_rotor_positions(main_rotor_position=3.0, tail_rotor_position=8.0)  # Closer positions
    
    # Fuselage parameters - much lighter
    heli.set_fuselage_parameters(
        mass=300,  # kg - reduced from 800
        length=8.0,  # m - reduced
        width=1.5,  # m
        height=2.0,  # m
        tail_mass=40,  # kg - reduced from 100
        tail_length=2.0,  # m
        tail_width=0.8,  # m
        tail_height=1.0  # m
    )
    
    # Engine parameters - more conservative
    heli.set_engine_parameters(
        mass=80,  # kg - reduced from 200
        max_power=400  # kW - reduced from 1500 for testing
    )
    
    return heli

def test_power_constraint_only():
    """Test ONLY the power constraint logic"""
    print("=== Testing Power Constraint Only ===")
    
    heli = setup_test_helicopter()
    planner = MissionPlanner(heli)
    
    # Set lighter flight parameters
    planner.set_flight_parameters_programmatic(
        dry_weight=500,  # kg - much lighter, reduced from 2000
        fuel_weight=100,  # kg - reduced from 500
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
        # Test ONLY power constraint (disable stall constraint)
        result = planner.find_max_hover_weight(
            altitude=altitude,
            stall_constraint=False,  # DISABLED - default airfoil never stalls
            power_constraint=True,   # ENABLED - this is the real constraint
            iterations=30,  # Reduced iterations
            initial_weight_guess=600,  # Start closer to expected result
            tol=50.0  # Relaxed tolerance to 50W
        )
        
        min_weight, stall_weight, power_weight = result
        
        print(f"\nResults:")
        print(f"  Max weight (power limited): {power_weight:.1f} kg")
        print(f"  Overall max weight: {min_weight:.1f} kg")
        print(f"  Stall weight (disabled): {stall_weight}")
        
        return power_weight
        
    except Exception as e:
        print(f"Test failed: {e}")
        import traceback
        traceback.print_exc()
        return None

def verify_power_limit():
    """Verify that the calculated max weight actually hits the power limit"""
    print("\n=== Verifying Power Limit ===")
    
    heli = setup_test_helicopter()
    planner = MissionPlanner(heli)
    
    planner.set_flight_parameters_programmatic(
        dry_weight=500,  # kg - lighter
        fuel_weight=100,  # kg - lighter
        fuel_specific_energy_kj_kg=43000,
        reserve_fuel_fraction=0.1
    )
    
    altitude = 0
    
    # Get max weight from power constraint
    max_weight = test_power_constraint_only()
    
    if max_weight is None:
        print("Could not determine max weight")
        return False
    
    print(f"\nVerification for max weight: {max_weight:.1f} kg")
    
    try:
        # Test weights around the maximum to see power progression
        test_weights = [
            max_weight * 0.8,  # 80% of max
            max_weight * 0.9,  # 90% of max
            max_weight * 1.0,  # Exactly max
            max_weight * 1.1   # 110% of max (should exceed power)
        ]
        
        print(f"\n{'Weight (kg)':<12} {'Power Needed (kW)':<18} {'Available (kW)':<15} {'Status':<10}")
        print("-" * 60)
        
        for weight in test_weights:
            power_needed = heli.find_power_needed(
                weight=weight,
                vertical_velocity=0,
                altitude=altitude
            )
            
            power_available = heli.engine_max_available_power
            status = "OK" if power_needed <= power_available else "EXCEED"
            
            print(f"{weight:<12.1f} {power_needed/1000:<18.1f} {power_available/1000:<15.1f} {status:<10}")
        
        return True
        
    except Exception as e:
        print(f"Verification failed: {e}")
        return False

def test_simple_power_search():
    """Test a manual power search to understand the relationship"""
    print("\n=== Manual Power Search Test ===")
    
    heli = setup_test_helicopter()
    
    print(f"Testing power vs weight relationship:")
    print(f"Engine available power: {heli.engine_max_available_power} kW")
    
    weights = [400, 600, 800, 1000, 1200]  # Lighter weights
        
        power_available = heli.engine_max_available_power
        power_diff = power_available - power_needed
        power_percent = (power_diff / power_available) * 100
        
        print(f"  Power needed: {power_needed/1000:.1f} kW")
        print(f"  Power available: {power_available/1000:.1f} kW")
        print(f"  Power difference: {power_diff/1000:.1f} kW ({power_percent:.2f}%)")
        
        # Should be very close to power limit
        if abs(power_diff) < 1000:  # Within 1 kW
            print("  ✓ Max weight correctly at power limit!")
            return True
        else:
            print("  ✗ Max weight not at power limit")
            return False
            
    except Exception as e:
        print(f"Verification failed: {e}")
        return False

def test_weight_vs_power_relationship():
    """Test how power requirement changes with weight"""
    print("\n=== Testing Weight vs Power Relationship ===")
    
    heli = setup_test_helicopter()
    planner = MissionPlanner(heli)
    
    planner.set_flight_parameters_programmatic(
        dry_weight=2000,
        fuel_weight=500,
        fuel_specific_energy_kj_kg=43000,
        reserve_fuel_fraction=0.1
    )
    
    altitude = 0
    power_limit = heli.engine_max_available_power
    
    # Test different weights
    test_weights = [3000, 4000, 5000, 6000, 7000, 8000]  # kg
    
    print(f"Power limit: {power_limit/1000:.1f} kW")
    print(f"{'Weight (kg)':<12} {'Power Needed (kW)':<18} {'Within Limit?':<12}")
    print("-" * 45)
    
    max_feasible_weight = 0
    
    for weight in test_weights:
        try:
            power_needed = heli.find_power_needed(
                weight=weight,
                vertical_velocity=0,
                altitude=altitude
            )
            
            within_limit = power_needed <= power_limit
            if within_limit:
                max_feasible_weight = weight
            
            print(f"{weight:<12.0f} {power_needed/1000:<18.1f} {within_limit!s:<12}")
            
        except Exception as e:
            print(f"{weight:<12.0f} ERROR: {str(e)[:15]}...")
    
    print(f"\nLast feasible weight: {max_feasible_weight} kg")
    return max_feasible_weight

def test_different_altitudes():
    """Test power-limited max weight at different altitudes"""
    print("\n=== Testing Power Limit at Different Altitudes ===")
    
    heli = setup_test_helicopter()
    planner = MissionPlanner(heli)
    
    planner.set_flight_parameters_programmatic(
        dry_weight=2000,
        fuel_weight=500,
        fuel_specific_energy_kj_kg=43000,
        reserve_fuel_fraction=0.1
    )
    
    altitudes = [0, 1000, 2000, 3000]  # meters
    
    print(f"{'Altitude (m)':<12} {'Max Weight (kg)':<15} {'Power Available (kW)':<20}")
    print("-" * 50)
    
    for alt in altitudes:
        try:
            result = planner.find_max_hover_weight(
                altitude=alt,
                stall_constraint=False,  # Disable stall constraint
                power_constraint=True,   # Only test power constraint
                iterations=30,
                initial_weight_guess=4000,
                tol=50.0  # Relaxed tolerance
            )
            
            max_weight = result[2]  # Power-limited weight
            
            print(f"{alt:<12.0f} {max_weight:<15.1f} {heli.engine_max_available_power/1000:<20.1f}")
            
        except Exception as e:
            print(f"{alt:<12.0f} ERROR: {str(e)[:20]}...")

if __name__ == "__main__":
    print("Testing Maximum Hover Weight - POWER CONSTRAINT ONLY")
    print("=" * 55)
    print("Note: Stall constraint disabled since default airfoil never stalls")
    
    try:
        # Run power-focused tests
        max_weight = test_power_constraint_only()
        
        if max_weight is not None:
            verify_power_limit()
            test_weight_vs_power_relationship()
            test_different_altitudes()
        
        print("\n" + "=" * 55)
        print("Power constraint testing completed!")
        
    except Exception as e:
        print(f"\nTest failed with error: {e}")
        import traceback
        traceback.print_exc()
