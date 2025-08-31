#!/usr/bin/env python3
"""
Debug power calculation for hover at 2000m
"""

from heli import Environment, Rotor, Helicopter
import numpy as np

def debug_power_calculation():
    """Debug power calculation for hover at 2000m"""
    print("=== DEBUGGING POWER CALCULATION ===")
    
    # Set up environment
    env = Environment()
    env.set_atmosphere_parameters(
        temperature_sea_level=288.15,
        Reynolds_number=1e6,
        altitude=2000.0,
        pressure_sea_level=101325,
        Lapse_rate_troposphere=0.0065,
        wind_velocity=0,
        ISA_OFFSET=0
    )
    
    density_2000m = env.get_density(env.get_temperature(2000), env.get_pressure(env.get_temperature(2000), 2000))
    print(f"Air density at 2000m: {density_2000m:.3f} kg/m³")
    
    # Set up main rotor
    main_rotor = Rotor(environment=env)
    main_rotor.set_rotor_parameters(
        number_of_blades=2,
        blade_mass=75,
        NACA_for_airfoil="0012",
        radius_of_rotors=6.0,
        root_cutout=0.6,
        angle_of_attack=8.0,
        root_chord=0.4,
        tip_chord=0.2,
        root_pitch=14.0,
        slope_pitch=0.08
    )
    
    # Test rotor performance at different RPMs
    print("\n=== ROTOR PERFORMANCE TEST ===")
    rpms = [300, 350, 400, 450, 500]
    
    for rpm in rpms:
        omega = 2 * np.pi * rpm / 60
        try:
            performance = main_rotor.calculate_performance(0, omega, density_2000m)
            thrust = performance.get('thrust', 0)
            power = performance.get('power', 0)
            torque = performance.get('torque', 0)
            
            print(f"RPM {rpm}: Thrust={thrust:.0f}N, Power={power/1000:.1f}kW, Torque={torque:.0f}Nm")
            
            # Check if this provides enough lift for our helicopter (1862 kg = ~18,267 N)
            required_thrust = 1862 * 9.81
            if thrust > required_thrust:
                print(f"  ✅ Sufficient thrust for hover (required: {required_thrust:.0f}N)")
            else:
                print(f"  ❌ Insufficient thrust (required: {required_thrust:.0f}N)")
                
        except Exception as e:
            print(f"RPM {rpm}: Error - {e}")
    
    print("\n=== HELICOPTER INTEGRATION TEST ===")
    
    # Set up helicopter
    tail_rotor = Rotor(environment=env)
    tail_rotor.set_rotor_parameters(
        number_of_blades=4,
        blade_mass=8,
        NACA_for_airfoil="0012",
        radius_of_rotors=1.2,
        root_cutout=0.15,
        angle_of_attack=8.0,
        root_chord=0.15,
        tip_chord=0.08,
        root_pitch=12.0,
        slope_pitch=0.05
    )
    
    heli = Helicopter(environment=env)
    heli.set_rotor(main_rotor, tail_rotor)
    heli.set_rotor_positions(5.0, 11.0)
    heli.set_fuselage_parameters(1200, 12.0, 2.5, 3.0, 0.1, 0.8)
    heli.set_engine_parameters(180, 0.35, 0.93, 1500)
    
    weight = heli.get_total_mass() + 300  # Add fuel weight
    print(f"Testing helicopter with weight: {weight:.0f} kg")
    
    # Test find_omega_needed
    try:
        omega_needed = heli.find_omega_needed(weight, 0, altitude=2000, initial_guess=30.0)
        print(f"Omega needed for hover: {omega_needed:.2f} rad/s ({omega_needed*60/(2*np.pi):.0f} RPM)")
        
        # Test power needed
        power_needed = heli.find_power_needed(weight, 0, altitude=2000, omega=omega_needed)
        print(f"Power needed for hover: {power_needed/1000:.1f} kW")
        
        # Check if helicopter is stalling
        stalling = heli.is_helicopter_stalling(0, altitude=2000, omega=omega_needed)
        print(f"Is stalling: {stalling}")
        
    except Exception as e:
        print(f"Error in helicopter calculations: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    debug_power_calculation()
