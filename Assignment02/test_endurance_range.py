#!/usr/bin/env p    def test_comprehensive_mission_planning():
        print("="*80)
        print("COMPREHENSIVE HELICOPTER MISSION PLANNING TEST")
        print("="*80)
        print("Using realistic engine efficiency (30%) to correct unrealistic endurance")
        print()
        
        # Realistic fuel energy accounting for turbine engine efficiency
        fuel_energy_raw = 43e6  # J/kg - raw fuel energy  
        engine_efficiency = 0.30  # 30% typical turbine efficiency
        effective_fuel_energy = fuel_energy_raw * engine_efficiency  # 12.9 MJ/kg effective
        
        # Test configurations based on Chinook reference values
        test_configs = [
            {
                "name": "Lightweight Config",
                "description": "Light helicopter configuration",
                "params": {
                    "f": 0.035,
                    "fuelweight": 600, 
                    "dryweight": 2800,
                    "fuel_specific_energy": effective_fuel_energy,
                    "fuel_reserve_fraction": 0.15
                }
            },
            {
                "name": "Chinook-based Config", 
                "description": "Based on CH-47 Chinook specifications",
                "params": {
                    "f": 0.055,
                    "fuelweight": 750,
                    "dryweight": 3500, 
                    "fuel_specific_energy": effective_fuel_energy,
                    "fuel_reserve_fraction": 0.10
                }
            },
            {
                "name": "Heavy Config",
                "description": "Heavy helicopter configuration", 
                "params": {
                    "f": 0.070,
                    "fuelweight": 900,
                    "dryweight": 4200,
                    "fuel_specific_energy": effective_fuel_energy,
                    "fuel_reserve_fraction": 0.12
                }
            }
        ]sive test suite for helicopter endurance and range calculations
"""

if __name__ == "__main__":
    import sys
    import time
    import numpy as np
    import matplotlib.pyplot as plt
    sys.path.append('.')
    exec(open('main2.py').read())
    
    def test_endurance_and_range():
        """
        Comprehensive test of endurance and range calculations
        """
        print("="*80)
        print("HELICOPTER ENDURANCE AND RANGE TEST SUITE")
        print("="*80)
        
        # Chinook-based helicopter configurations (per rotor equivalent)
    configurations = {
        "light_single_rotor": {
            "name": "Light Single Rotor",
            "params": {"f": 0.035, "fuelweight": 500, "dryweight": 2000, "fuel_specific_energy": 43e6, "fuel_reserve_fraction": 0.08}
        },
        "chinook_equivalent": {
            "name": "Chinook-like Per Rotor",
            "params": {"f": 0.055, "fuelweight": 750, "dryweight": 3500, "fuel_specific_energy": 43e6, "fuel_reserve_fraction": 0.10}
        },
        "heavy_transport": {
            "name": "Heavy Transport",
            "params": {"f": 0.075, "fuelweight": 1200, "dryweight": 5500, "fuel_specific_energy": 43e6, "fuel_reserve_fraction": 0.12}
        }
    }
        
        results = []
        
        for i, config in enumerate(test_configs):
            print(f"\n{'='*60}")
            print(f"TEST {i+1}: {config['name']}")
            print(f"{'='*60}")
            
            # Create mission planner
            mp = MissionPlanner(
                f=config['f'],
                fuelweight=config['fuelweight'],
                dryweight=config['dryweight'],
                fuel_specific_energy=config['fuel_specific_energy'],
                fuel_reserve_fraction=config['fuel_reserve_fraction']
            )
            
            print(f"Configuration:")
            print(f"  Dry weight: {config['dryweight']} kg")
            print(f"  Fuel weight: {config['fuelweight']} kg")
            print(f"  Total weight: {config['dryweight'] + config['fuelweight']} kg")
            print(f"  Available fuel: {config['fuelweight']*(1-config['fuel_reserve_fraction']):.0f} kg")
            print(f"  Drag factor: {config['f']}")
            
            # Test different weight conditions
            weight_conditions = [
                ("Full Fuel", config['dryweight'] + config['fuelweight']),
                ("Half Fuel", config['dryweight'] + config['fuelweight']*0.5),
                ("Quarter Fuel", config['dryweight'] + config['fuelweight']*0.25)
            ]
            
            config_results = {
                "name": config['name'],
                "endurance_speeds": [],
                "range_speeds": [],
                "max_endurance": 0,
                "max_range": 0,
                "endurance_calc_time": 0,
                "range_calc_time": 0
            }
            
            # Test 1: Endurance Speed Analysis
            print(f"\n{'-'*40}")
            print("ENDURANCE SPEED ANALYSIS")
            print(f"{'-'*40}")
            
            for condition_name, weight in weight_conditions:
                print(f"\n{condition_name} ({weight:.0f} kg):")
                
                # Single-threaded endurance speed
                start_time = time.time()
                best_speed_st, min_power_st = mp.get_max_endurance_speed(
                    weight=weight, dV=8, V_start=20, V_end=70
                )
                time_st = time.time() - start_time
                
                # Multithreaded endurance speed
                start_time = time.time()
                best_speed_mt, min_power_mt = mp.get_max_endurance_speed_mt(
                    weight=weight, dV=8, V_start=20, V_end=70, num_workers=4
                )
                time_mt = time.time() - start_time
                
                print(f"  Single-threaded: {best_speed_st} m/s ({best_speed_st*3.6:.0f} km/h), {min_power_st/1000:.0f} kW, Time: {time_st:.2f}s")
                print(f"  Multithreaded:   {best_speed_mt} m/s ({best_speed_mt*3.6:.0f} km/h), {min_power_mt/1000:.0f} kW, Time: {time_mt:.2f}s")
                print(f"  Speedup: {time_st/time_mt:.2f}x")
                
                config_results["endurance_speeds"].append({
                    "condition": condition_name,
                    "weight": weight,
                    "speed": best_speed_mt,
                    "power": min_power_mt
                })
            
            # Test 2: Range Speed Analysis
            print(f"\n{'-'*40}")
            print("RANGE SPEED ANALYSIS")
            print(f"{'-'*40}")
            
            for condition_name, weight in weight_conditions:
                print(f"\n{condition_name} ({weight:.0f} kg):")
                
                # Single-threaded range speed
                start_time = time.time()
                best_speed_st, max_eff_st = mp.get_max_range_speed(
                    weight=weight, dV=10, V_start=30, V_end=120
                )
                time_st = time.time() - start_time
                
                # Multithreaded range speed
                start_time = time.time()
                best_speed_mt, max_eff_mt = mp.get_max_range_speed_mt(
                    weight=weight, dV=10, V_start=30, V_end=120, num_workers=4
                )
                time_mt = time.time() - start_time
                
                print(f"  Single-threaded: {best_speed_st} m/s ({best_speed_st*3.6:.0f} km/h), Eff: {max_eff_st:.6f} m/W, Time: {time_st:.2f}s")
                print(f"  Multithreaded:   {best_speed_mt} m/s ({best_speed_mt*3.6:.0f} km/h), Eff: {max_eff_mt:.6f} m/W, Time: {time_mt:.2f}s")
                print(f"  Speedup: {time_st/time_mt:.2f}x")
                
                config_results["range_speeds"].append({
                    "condition": condition_name,
                    "weight": weight,
                    "speed": best_speed_mt,
                    "efficiency": max_eff_mt
                })
            
            # Test 3: Maximum Endurance Calculation
            print(f"\n{'-'*40}")
            print("MAXIMUM ENDURANCE CALCULATION")
            print(f"{'-'*40}")
            
            print("Calculating maximum endurance...")
            start_time = time.time()
            max_endurance = mp.get_endurance_mt(
                dV=12, V_start=25, V_end=70, dt=1800, num_workers=4  # 30-minute intervals
            )
            endurance_calc_time = time.time() - start_time
            
            config_results["max_endurance"] = max_endurance
            config_results["endurance_calc_time"] = endurance_calc_time
            
            print(f"  Maximum endurance: {max_endurance/3600:.2f} hours")
            print(f"  Calculation time: {endurance_calc_time:.2f} seconds")
            
            # Test 4: Maximum Range Calculation
            print(f"\n{'-'*40}")
            print("MAXIMUM RANGE CALCULATION")
            print(f"{'-'*40}")
            
            print("Calculating maximum range...")
            start_time = time.time()
            max_range, flight_time = mp.get_max_range_mt(
                dV=15, V_start=40, V_end=120, dt=1800, num_workers=4  # 30-minute intervals
            )
            range_calc_time = time.time() - start_time
            
            config_results["max_range"] = max_range
            config_results["range_calc_time"] = range_calc_time
            
            print(f"  Maximum range: {max_range/1000:.1f} km")
            print(f"  Flight time: {flight_time/3600:.2f} hours")
            print(f"  Average speed: {(max_range/flight_time)*3.6:.1f} km/h")
            print(f"  Calculation time: {range_calc_time:.2f} seconds")
            
            # Test 5: Performance Envelope Analysis
            print(f"\n{'-'*40}")
            print("PERFORMANCE ENVELOPE ANALYSIS")
            print(f"{'-'*40}")
            
            weight = config['dryweight'] + config['fuelweight']  # Full weight
            speeds = [30, 50, 70, 90, 110, 130]
            
            print(f"Speed vs Power Analysis (Weight: {weight} kg):")
            print(f"{'Speed (m/s)':>12} {'Speed (km/h)':>12} {'Power (kW)':>12} {'Efficiency':>12}")
            print("-" * 48)
            
            for speed in speeds:
                try:
                    D = get_Parasitic_Drag(ρ=mp.ρ, f=mp.f, Vinfty=speed)
                    T_Needed = get_Thrust_Total(W=weight*g, D=D)
                    
                    trim_result = trimSolve(
                        rotor=mp.rotor, 
                        Thrust_Needed=T_Needed, 
                        Ω=20, 
                        θ0_initial=5*deg_to_rad, 
                        θ1s_initial=-4*deg_to_rad, 
                        θ1c_initial=2*deg_to_rad, 
                        W=weight*g, 
                        D=D, 
                        verbose=False
                    )
                    
                    power = trim_result[3][2]
                    efficiency = speed / power if power > 0 else 0
                    
                    print(f"{speed:>12} {speed*3.6:>12.0f} {power/1000:>12.0f} {efficiency*1e6:>12.3f}")
                    
                except Exception as e:
                    print(f"{speed:>12} {speed*3.6:>12.0f} {'Error':>12} {'---':>12}")
            
            results.append(config_results)
        
        # Summary comparison
        print(f"\n{'='*80}")
        print("SUMMARY COMPARISON")
        print(f"{'='*80}")
        
        print(f"{'Configuration':<25} {'Max Endurance (h)':<18} {'Max Range (km)':<15} {'Calc Time (s)'}")
        print("-" * 75)
        
        for result in results:
            print(f"{result['name']:<25} {result['max_endurance']/3600:<18.2f} {result['max_range']/1000:<15.1f} {result['endurance_calc_time']+result['range_calc_time']:<12.1f}")
        
        # Performance insights
        print(f"\n{'-'*60}")
        print("PERFORMANCE INSIGHTS")
        print(f"{'-'*60}")
        
        for result in results:
            print(f"\n{result['name']}:")
            
            # Best endurance speed
            if result['endurance_speeds']:
                full_fuel_endurance = result['endurance_speeds'][0]  # Full fuel condition
                print(f"  Best endurance speed: {full_fuel_endurance['speed']} m/s ({full_fuel_endurance['speed']*3.6:.0f} km/h)")
                print(f"  Power at best endurance: {full_fuel_endurance['power']/1000:.0f} kW")
            
            # Best range speed
            if result['range_speeds']:
                full_fuel_range = result['range_speeds'][0]  # Full fuel condition
                print(f"  Best range speed: {full_fuel_range['speed']} m/s ({full_fuel_range['speed']*3.6:.0f} km/h)")
                print(f"  Efficiency at best range: {full_fuel_range['efficiency']:.6f} m/W")
            
            # Mission capabilities
            print(f"  Maximum endurance: {result['max_endurance']/3600:.2f} hours")
            print(f"  Maximum range: {result['max_range']/1000:.1f} km")
            print(f"  Range/Endurance ratio: {(result['max_range']/1000)/(result['max_endurance']/3600):.1f} km/h")
        
        print(f"\n{'='*80}")
        print("ENDURANCE AND RANGE TESTING COMPLETED!")
        print("="*80)
        
        return results
    
    # Run the comprehensive test
    test_results = test_endurance_and_range()
