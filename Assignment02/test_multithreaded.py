#!/usr/bin/env python3
"""
Test script to compare single-threaded vs multithreaded mission planner performance
"""

import sys
import time
sys.path.append('.')

# Import the mission planner
exec(open('main2.py').read())

def test_multithreaded_performance():
    print("="*80)
    print("MULTITHREADED MISSION PLANNER PERFORMANCE TEST")
    print("="*80)
    
    # Create mission planner instance
    mp = MissionPlanner(
        f=0.05,
        fuelweight=200,
        dryweight=800,
        fuel_specific_energy=43e6,
        fuel_reserve_fraction=0.2
    )
    
    test_weight = 1000  # kg
    
    print(f"\nTesting with weight: {test_weight} kg")
    print(f"Available CPU cores: {mp.cpu_count()}")
    
    # Test 1: Max Range Speed Comparison
    print(f"\n{'-'*60}")
    print("TEST 1: MAX RANGE SPEED CALCULATION")
    print(f"{'-'*60}")
    
    # Single-threaded
    print("Single-threaded version:")
    start_time = time.time()
    speed_st, eff_st = mp.get_max_range_speed(weight=test_weight, dV=15, V_start=30, V_end=120)
    time_st = time.time() - start_time
    print(f"  Time: {time_st:.2f} seconds")
    print(f"  Best speed: {speed_st} m/s ({speed_st*3.6:.0f} km/h)")
    print(f"  Efficiency: {eff_st:.6f} m/W")
    
    # Multithreaded
    print("Multithreaded version:")
    start_time = time.time()
    speed_mt, eff_mt = mp.get_max_range_speed_mt(weight=test_weight, dV=15, V_start=30, V_end=120, num_workers=4)
    time_mt = time.time() - start_time
    print(f"  Time: {time_mt:.2f} seconds")
    print(f"  Best speed: {speed_mt} m/s ({speed_mt*3.6:.0f} km/h)")
    print(f"  Efficiency: {eff_mt:.6f} m/W")
    print(f"  Speedup: {time_st/time_mt:.2f}x")
    
    # Test 2: Max Endurance Speed Comparison
    print(f"\n{'-'*60}")
    print("TEST 2: MAX ENDURANCE SPEED CALCULATION")
    print(f"{'-'*60}")
    
    # Single-threaded
    print("Single-threaded version:")
    start_time = time.time()
    speed_st, power_st = mp.get_max_endurance_speed(weight=test_weight, dV=10, V_start=20, V_end=80)
    time_st = time.time() - start_time
    print(f"  Time: {time_st:.2f} seconds")
    print(f"  Best speed: {speed_st} m/s ({speed_st*3.6:.0f} km/h)")
    print(f"  Power: {power_st/1000:.0f} kW")
    
    # Multithreaded
    print("Multithreaded version:")
    start_time = time.time()
    speed_mt, power_mt = mp.get_max_endurance_speed_mt(weight=test_weight, dV=10, V_start=20, V_end=80, num_workers=4)
    time_mt = time.time() - start_time
    print(f"  Time: {time_mt:.2f} seconds")
    print(f"  Best speed: {speed_mt} m/s ({speed_mt*3.6:.0f} km/h)")
    print(f"  Power: {power_mt/1000:.0f} kW")
    print(f"  Speedup: {time_st/time_mt:.2f}x")
    
    # Test 3: Quick Range Calculation
    print(f"\n{'-'*60}")
    print("TEST 3: QUICK RANGE CALCULATION")
    print(f"{'-'*60}")
    
    print("Multithreaded range calculation:")
    start_time = time.time()
    range_mt, time_range_mt = mp.get_max_range_mt(dV=20, V_start=40, V_end=100, dt=1800, num_workers=4)  # 30-min intervals
    calc_time = time.time() - start_time
    print(f"  Calculation time: {calc_time:.2f} seconds")
    print(f"  Max range: {range_mt/1000:.1f} km")
    print(f"  Flight time: {time_range_mt/3600:.1f} hours")
    print(f"  Average speed: {(range_mt/time_range_mt)*3.6:.1f} km/h")
    
    print(f"\n{'-'*60}")
    print("TEST 4: QUICK ENDURANCE CALCULATION")
    print(f"{'-'*60}")
    
    print("Multithreaded endurance calculation:")
    start_time = time.time()
    endurance_mt = mp.get_endurance_mt(dV=15, V_start=25, V_end=70, dt=1800, num_workers=4)  # 30-min intervals
    calc_time = time.time() - start_time
    print(f"  Calculation time: {calc_time:.2f} seconds")
    print(f"  Max endurance: {endurance_mt/3600:.1f} hours")
    
    print(f"\n{'='*80}")
    print("PERFORMANCE TEST COMPLETED!")
    print("="*80)

if __name__ == "__main__":
    test_multithreaded_performance()
