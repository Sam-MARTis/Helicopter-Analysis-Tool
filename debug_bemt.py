#!/usr/bin/env python3
"""
Deep debug of BEMT calculation
"""

from heli import Environment, Rotor
import numpy as np

def debug_bemt():
    """Debug BEMT calculation step by step"""
    print("=== DEBUGGING BEMT CALCULATION ===")
    
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
    
    # Set up main rotor with different parameters
    rotor = Rotor(environment=env)
    rotor.set_rotor_parameters(
        number_of_blades=2,
        blade_mass=75,
        NACA_for_airfoil="0012",
        radius_of_rotors=6.0,
        root_cutout=0.6,
        angle_of_attack=8.0,
        root_chord=0.4,
        tip_chord=0.2,
        root_pitch=14.0,  # Try higher collective
        slope_pitch=0.08
    )
    
    print(f"\nRotor parameters:")
    print(f"  Radius: {rotor.radius_of_rotors} m")
    print(f"  Root cutout: {rotor.root_cutout} m")
    print(f"  Root chord: {rotor.root_chord} m")
    print(f"  Tip chord: {rotor.tip_chord} m")
    print(f"  Root pitch: {np.rad2deg(rotor.root_pitch):.1f}° ({rotor.root_pitch:.3f} rad)")
    print(f"  Angle of attack: {np.rad2deg(rotor.angle_of_attack):.1f}° ({rotor.angle_of_attack:.3f} rad)")
    print(f"  Number of blades: {rotor.number_of_blades}")
    
    # Test BEMT with different collective pitch settings
    print(f"\n=== TESTING DIFFERENT COLLECTIVE PITCH ===")
    
    omega = 2 * np.pi * 400 / 60  # 400 RPM
    vertical_velocity = 0  # Hover
    
    for collective in [8, 12, 16, 20, 24]:
        rotor.root_pitch = np.deg2rad(collective)  # Convert to radians
        print(f"\nCollective pitch: {collective}° ({rotor.root_pitch:.3f} rad)")
        
        try:
            performance = rotor.calculate_performance(vertical_velocity, omega, density_2000m)
            thrust = performance.get('thrust', 0)
            power = performance.get('power', 0)
            torque = performance.get('torque', 0)
            
            print(f"  Thrust: {thrust:.0f} N")
            print(f"  Power: {power/1000:.1f} kW") 
            print(f"  Torque: {torque:.0f} Nm")
            
            # Check power loading
            disc_area = np.pi * rotor.radius_of_rotors**2
            disc_loading = thrust / disc_area if disc_area > 0 else 0
            print(f"  Disc loading: {disc_loading:.1f} N/m²")
            
        except Exception as e:
            print(f"  Error: {e}")
    
    # Test with more reasonable helicopter parameters
    print(f"\n=== TESTING SMALLER HELICOPTER ===")
    
    rotor2 = Rotor(environment=env)
    rotor2.set_rotor_parameters(
        number_of_blades=2,
        blade_mass=30,
        NACA_for_airfoil="0012",
        radius_of_rotors=4.0,  # Smaller rotor
        root_cutout=0.3,
        angle_of_attack=6.0,
        root_chord=0.25,
        tip_chord=0.15,
        root_pitch=18.0,  # Higher collective
        slope_pitch=0.08
    )
    
    print(f"Smaller rotor - Radius: {rotor2.radius_of_rotors}m, Collective: {rotor2.root_pitch}°")
    
    try:
        performance = rotor2.calculate_performance(0, omega, density_2000m)
        thrust = performance.get('thrust', 0)
        power = performance.get('power', 0)
        
        print(f"  Thrust: {thrust:.0f} N")
        print(f"  Power: {power/1000:.1f} kW")
        
        # Calculate theoretical hover power
        required_thrust = 1000 * 9.81  # 1000 kg helicopter
        disc_area = np.pi * rotor2.radius_of_rotors**2
        ideal_power = required_thrust * np.sqrt(required_thrust / (2 * density_2000m * disc_area))
        
        print(f"  Required thrust for 1000kg: {required_thrust:.0f} N")
        print(f"  Theoretical ideal hover power: {ideal_power/1000:.1f} kW")
        
    except Exception as e:
        print(f"  Error: {e}")

if __name__ == "__main__":
    debug_bemt()
