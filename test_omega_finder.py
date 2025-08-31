#!/usr/bin/env python3

import numpy as np
from heli import Environment, Rotor

def test_find_omega_needed_uncoupled():
    print("=== TESTING find_omega_needed_uncoupled METHOD ===")
    
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
    
    # Create rotor with your parameters
    rotor = Rotor(environment=env)
    rotor.set_rotor_parameters(
        number_of_blades=3,
        blade_mass=75,
        NACA_for_airfoil="0012",
        radius_of_rotors=6.6,
        root_cutout=0.3,
        root_chord=0.9,
        tip_chord=0.5,
        root_pitch=np.deg2rad(10.0),
        slope_pitch=np.deg2rad(3),
    )
    
    print(f"Rotor configuration:")
    print(f"  Radius: {rotor.radius_of_rotors} m")
    print(f"  Root chord: {rotor.root_chord} m")
    print(f"  Tip chord: {rotor.tip_chord} m")
    print(f"  Taper ratio: {rotor.tip_chord/rotor.root_chord:.2f}")
    print(f"  Number of blades: {rotor.number_of_blades}")
    print()
    
    # Test different thrust targets
    thrust_targets = [10000, 20000, 30000, 40000, 50000]  # N
    vertical_velocity = 0  # Hover condition
    altitude = 2000  # m
    
    print("Test Results:")
    print(f"{'Target Thrust (N)':<18} {'Found Omega (rad/s)':<20} {'Actual Thrust (N)':<18} {'Error (%)':<12}")
    print("-" * 70)
    
    for thrust_target in thrust_targets:
        try:
            # Find omega needed for this thrust
            omega_found = rotor.find_omega_needed_uncoupled(
                thrust_needed=thrust_target,
                vertical_velocity=vertical_velocity,
                altitude=altitude,
                initial_guess=30.0
            )
            
            # Verify by calculating performance at found omega
            temperature = env.get_temperature(altitude)
            pressure = env.get_pressure(temperature, altitude)
            density = env.get_density(temperature, pressure)
            
            performance = rotor.calculate_performance(vertical_velocity, omega_found, density)
            actual_thrust = performance['thrust']
            
            error_percent = abs(actual_thrust - thrust_target) / thrust_target * 100
            
            print(f"{thrust_target:<18.0f} {omega_found:<20.2f} {actual_thrust:<18.0f} {error_percent:<12.2f}")
            
        except Exception as e:
            print(f"{thrust_target:<18.0f} {'ERROR':<20} {'ERROR':<18} {'ERROR':<12}")
            print(f"  Error: {e}")
    
    print()
    print("=== CONVERGENCE TEST ===")
    
    # Test convergence with different initial guesses
    thrust_test = 25000  # N
    initial_guesses = [10, 20, 30, 40, 50, 100]
    
    print(f"{'Initial Guess':<15} {'Found Omega':<15} {'Actual Thrust':<15} {'Iterations':<12}")
    print("-" * 60)
    
    for guess in initial_guesses:
        try:
            omega_found = rotor.find_omega_needed_uncoupled(
                thrust_needed=thrust_test,
                vertical_velocity=0,
                altitude=2000,
                initial_guess=guess,
                iterations=20
            )
            
            temperature = env.get_temperature(2000)
            pressure = env.get_pressure(temperature, 2000)
            density = env.get_density(temperature, pressure)
            performance = rotor.calculate_performance(0, omega_found, density)
            actual_thrust = performance['thrust']
            
            print(f"{guess:<15.1f} {omega_found:<15.2f} {actual_thrust:<15.0f} {'OK':<12}")
            
        except Exception as e:
            print(f"{guess:<15.1f} {'ERROR':<15} {'ERROR':<15} {'FAILED':<12}")

if __name__ == "__main__":
    test_find_omega_needed_uncoupled()
