#!/usr/bin/env python3
"""
Simple test for multithreaded mission planner
"""

if __name__ == "__main__":
    import sys
    import time
    sys.path.append('.')
    exec(open('main2.py').read())
    
    print("="*70)
    print("MULTITHREADED MISSION PLANNER TEST")
    print("="*70)
    
    # Create mission planner
    mp = MissionPlanner(
        f=0.05,
        fuelweight=200,
        dryweight=800,
        fuel_specific_energy=43e6,
        fuel_reserve_fraction=0.2
    )
    
    test_weight = 1000
    print(f"Test weight: {test_weight} kg")
    print(f"Available CPUs: {mp.cpu_count()}")
    
    # Test range speed calculation
    print(f"\n{'-'*50}")
    print("RANGE SPEED OPTIMIZATION")
    print(f"{'-'*50}")
    
    print("Single-threaded:")
    start = time.time()
    speed_st, eff_st = mp.get_max_range_speed(weight=test_weight, dV=15, V_start=30, V_end=100)
    time_st = time.time() - start
    print(f"  Time: {time_st:.2f}s")
    print(f"  Best speed: {speed_st} m/s ({speed_st*3.6:.0f} km/h)")
    print(f"  Efficiency: {eff_st:.6f} m/W")
    
    print("Multithreaded:")
    start = time.time()
    speed_mt, eff_mt = mp.get_max_range_speed_mt(weight=test_weight, dV=15, V_start=30, V_end=100, num_workers=4)
    time_mt = time.time() - start
    print(f"  Time: {time_mt:.2f}s")
    print(f"  Best speed: {speed_mt} m/s ({speed_mt*3.6:.0f} km/h)")
    print(f"  Efficiency: {eff_mt:.6f} m/W")
    print(f"  Speedup: {time_st/time_mt:.2f}x")
    
    # Test endurance speed calculation
    print(f"\n{'-'*50}")
    print("ENDURANCE SPEED OPTIMIZATION")
    print(f"{'-'*50}")
    
    print("Single-threaded:")
    start = time.time()
    speed_st, power_st = mp.get_max_endurance_speed(weight=test_weight, dV=8, V_start=25, V_end=70)
    time_st = time.time() - start
    print(f"  Time: {time_st:.2f}s")
    print(f"  Best speed: {speed_st} m/s ({speed_st*3.6:.0f} km/h)")
    print(f"  Power: {power_st/1000:.0f} kW")
    
    print("Multithreaded:")
    start = time.time()
    speed_mt, power_mt = mp.get_max_endurance_speed_mt(weight=test_weight, dV=8, V_start=25, V_end=70, num_workers=4)
    time_mt = time.time() - start
    print(f"  Time: {time_mt:.2f}s")
    print(f"  Best speed: {speed_mt} m/s ({speed_mt*3.6:.0f} km/h)")
    print(f"  Power: {power_mt/1000:.0f} kW")
    print(f"  Speedup: {time_st/time_mt:.2f}x")
    
    # Test full mission calculations
    print(f"\n{'-'*50}")
    print("MISSION CALCULATIONS")
    print(f"{'-'*50}")
    
    print("Range calculation (multithreaded):")
    start = time.time()
    range_km, flight_time_h = mp.get_max_range_mt(dV=20, V_start=40, V_end=100, dt=3600, num_workers=3)
    calc_time = time.time() - start
    print(f"  Calculation time: {calc_time:.2f}s")
    print(f"  Maximum range: {range_km/1000:.1f} km")
    print(f"  Flight time: {flight_time_h/3600:.1f} hours")
    print(f"  Average speed: {(range_km/flight_time_h)*3.6:.1f} km/h")
    
    print("Endurance calculation (multithreaded):")
    start = time.time()
    endurance_h = mp.get_endurance_mt(dV=15, V_start=30, V_end=70, dt=3600, num_workers=3)
    calc_time = time.time() - start
    print(f"  Calculation time: {calc_time:.2f}s")
    print(f"  Maximum endurance: {endurance_h/3600:.1f} hours")
    
    print(f"\n{'='*70}")
    print("TEST COMPLETED SUCCESSFULLY!")
    print("="*70)
