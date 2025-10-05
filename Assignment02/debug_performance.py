#!/usr/bin/env python3
"""
Debug helicopter performance calculations
"""

if __name__ == "__main__":
    import sys
    sys.path.append('.')
    exec(open('main2.py').read())
    
    print("="*60)
    print("DEBUGGING HELICOPTER PERFORMANCE")
    print("="*60)
    
    # Create the same helicopter as in main2.py with adjusted parameters
    rotor = Rotor(rotor_mass=150, blade_count=3, R=16, rc=0.4, chord_function=lambda r: 0.5, θtw=θtw, ρ=1.225) 
    mp1 = MissionPlanner(rotor=rotor, Ω=25, f=0.05, fuelweight=500, dryweight=1500, fuel_specific_energy=43e6*0.35, fuel_reserve_fraction=0.3)  # Higher RPM, lower weight
    
    # Test different speeds manually
    test_speeds = [10, 20, 30, 40, 50]  # m/s
    total_weight = 2000  # kg - Much lighter for testing
    
    print(f"Configuration:")
    print(f"  Total weight: {total_weight} kg")
    print(f"  Rotor radius: {rotor.R} m") 
    print(f"  Drag factor f: {mp1.f}")
    print(f"  Fuel energy: {mp1.fuel_specific_energy/1e6:.1f} MJ/kg")
    print()
    
    print(f"{'Speed':>8} {'Drag':>8} {'Thrust':>8} {'Power':>8} {'Status':>12}")
    print(f"{'(m/s)':>8} {'(N)':>8} {'(N)':>8} {'(kW)':>8} {''}")
    print("-" * 50)
    
    for V in test_speeds:
        try:
            # Calculate drag and thrust required
            D = get_Parasitic_Drag(ρ=mp1.ρ, f=mp1.f, Vinfty=V)
            T_Needed = get_Thrust_Total(W=total_weight*9.81, D=D)
            
            print(f"{V:>8.0f} {D:>8.0f} {T_Needed:>8.0f}", end="")
            
            # Try trim solution
            result = trimSolve(
                rotor=rotor, 
                Thrust_Needed=T_Needed, 
                Ω=10, 
                θ0_initial=5*deg_to_rad, 
                θ1s_initial=-4*deg_to_rad, 
                θ1c_initial=2*deg_to_rad, 
                W=total_weight*9.81, 
                D=D, 
                verbose=False
            )
            
            if result is not None and len(result) >= 4:
                power = result[3][2]  # Power from trim solution
                print(f" {power/1000:>8.0f} {'Success':>12}")
            else:
                print(f" {'---':>8} {'Failed':>12}")
                
        except Exception as e:
            print(f" {'---':>8} {'Error':>12}")
            print(f"  Error: {str(e)[:50]}")
    
    print()
    print("If power values are extremely high (>1000 kW), there may be an issue with:")
    print("- Rotor parameters (size, RPM)")
    print("- Trim solver convergence") 
    print("- Unit conversions")
