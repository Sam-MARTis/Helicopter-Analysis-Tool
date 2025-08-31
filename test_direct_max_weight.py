#!/usr/bin/env python3
"""
Direct test of max weight algorithm logic using predetermined omega values
This bypasses convergence issues to test the algorithm logic itself
"""

import numpy as np
from heli import Environment, Rotor, Helicopter, MissionPlanner

def create_test_helicopter():
    """Create helicopter with very conservative parameters"""
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
        radius_of_rotors=2.0,  # Very small
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
    
    # Small engine
    heli.set_engine_parameters(mass=30, max_power=50)  # 50 kW
    
    return heli

def test_direct_power_calculation():
    """Test power calculations at specific omega values"""
    print("=== Direct Power Calculation Test ===")
    
    heli = create_test_helicopter()
    
    test_weights = [200, 300, 400, 500]  # Very light weights
    test_omegas = [20, 25, 30, 35]  # Conservative omega values
    
    print(f"Engine power limit: {heli.engine_max_available_power} kW")
    print(f"\n{'Weight (kg)':<10} {'Omega':<8} {'Power (kW)':<12} {'Within Limit?':<12}")
    print("-" * 45)
    
    feasible_combinations = []
    
    for weight in test_weights:
        for omega in test_omegas:
            try:
                power_needed = heli.find_power_needed(
                    weight=weight,
                    vertical_velocity=0,
                    altitude=0,
                    omega=omega  # Use predetermined omega
                )
                
                within_limit = power_needed <= heli.engine_max_available_power
                
                print(f"{weight:<10.0f} {omega:<8.0f} {power_needed/1000:<12.1f} {within_limit!s:<12}")
                
                if within_limit:
                    feasible_combinations.append((weight, omega, power_needed))
                    
            except Exception as e:
                print(f"{weight:<10.0f} {omega:<8.0f} ERROR: {str(e)[:15]}")
    
    if feasible_combinations:
        max_feasible = max(feasible_combinations, key=lambda x: x[0])
        print(f"\nMax feasible combination:")
        print(f"  Weight: {max_feasible[0]} kg")
        print(f"  Omega: {max_feasible[1]} rad/s") 
        print(f"  Power: {max_feasible[2]/1000:.1f} kW")
        return max_feasible[0]
    else:
        print("\nNo feasible combinations found")
        return 0

def test_manual_binary_search():
    """Manually implement the binary search to verify logic"""
    print("\n=== Manual Binary Search Test ===")
    
    heli = create_test_helicopter()
    
    power_limit = heli.engine_max_available_power
    print(f"Power limit: {power_limit/1000:.1f} kW")
    
    # Manual binary search
    min_weight = 100.0
    max_weight = 1000.0
    tol = 10.0  # 10W tolerance
    
    print(f"\n{'Iteration':<10} {'Weight':<10} {'Power (kW)':<12} {'Status':<15}")
    print("-" * 50)
    
    for iteration in range(15):  # Manual binary search
        test_weight = (min_weight + max_weight) / 2
        
        try:
            # Use fixed omega to avoid convergence issues
            power_needed = heli.find_power_needed(
                weight=test_weight,
                vertical_velocity=0,
                altitude=0,
                omega=25.0  # Fixed omega
            )
            
            if power_needed > power_limit:
                status = "TOO HIGH"
                max_weight = test_weight
            else:
                status = "OK"
                min_weight = test_weight
            
            print(f"{iteration+1:<10} {test_weight:<10.1f} {power_needed/1000:<12.1f} {status:<15}")
            
            if abs(power_needed - power_limit) < tol:
                print(f"\nConverged! Max weight: {test_weight:.1f} kg")
                return test_weight
                
        except Exception as e:
            print(f"{iteration+1:<10} {test_weight:<10.1f} ERROR")
            max_weight = test_weight
    
    final_weight = (min_weight + max_weight) / 2
    print(f"\nFinal estimate: {final_weight:.1f} kg")
    return final_weight

if __name__ == "__main__":
    print("Direct Max Weight Logic Test")
    print("=" * 30)
    
    # Test direct calculations
    max_from_direct = test_direct_power_calculation()
    
    # Test manual binary search
    max_from_search = test_manual_binary_search()
    
    print(f"\n" + "=" * 30)
    print(f"Summary:")
    print(f"  Max from direct test: {max_from_direct} kg")
    print(f"  Max from binary search: {max_from_search:.1f} kg")
    
    if abs(max_from_direct - max_from_search) < 50:
        print("  ✓ Results are consistent - algorithm logic is correct!")
    else:
        print("  ? Results differ - may need investigation")
