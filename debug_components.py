#!/usr/bin/env python3
"""
Deep debug of BEMT individual components
"""

from heli import Environment, Rotor
import numpy as np

def debug_bemt_components():
    """Debug individual BEMT components"""
    print("=== DEBUGGING BEMT COMPONENTS ===")
    
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
    
    # Set up rotor
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
        root_pitch=14.0,
        slope_pitch=0.08
    )
    
    print(f"Rotor setup complete")
    print(f"  Root pitch: {np.rad2deg(rotor.root_pitch):.1f}°")
    print(f"  Angle of attack: {np.rad2deg(rotor.angle_of_attack):.1f}°")
    
    # Test parameters
    omega = 2 * np.pi * 400 / 60  # 400 RPM
    vertical_velocity = 0  # Hover
    
    # Test at mid-span
    r_test = (rotor.root_cutout + rotor.radius_of_rotors) / 2  # Mid-span
    print(f"\nTesting at r = {r_test:.2f}m (mid-span)")
    
    # Test chord length
    chord = rotor.get_chord_length(r_test)
    print(f"Chord length: {chord:.3f}m")
    
    # Test elemental inflow ratio
    try:
        lambda_inflow, F = rotor.elemental_inflow_ratio(r_test, vertical_velocity, omega)
        print(f"Inflow ratio λ: {lambda_inflow:.6f}")
        print(f"Tip loss factor F: {F:.6f}")
    except Exception as e:
        print(f"Error in elemental_inflow_ratio: {e}")
        return
    
    # Test phi angle
    try:
        phi = rotor.get_phi_r(r_test, vertical_velocity, omega, lambda_inflow=lambda_inflow)
        print(f"Inflow angle φ: {np.rad2deg(phi):.2f}°")
    except Exception as e:
        print(f"Error in get_phi_r: {e}")
        return
    
    # Test lift and drag coefficients
    try:
        cl = rotor.get_cL(r_test, vertical_velocity, omega, lambda_inflow)
        cd = rotor.get_cD(r_test, vertical_velocity, omega, lambda_inflow)
        print(f"Lift coefficient cL: {cl:.6f}")
        print(f"Drag coefficient cD: {cd:.6f}")
    except Exception as e:
        print(f"Error in get_cL/get_cD: {e}")
        return
    
    # Test velocity calculation
    velocity_squared = (omega * r_test) ** 2 + (lambda_inflow * omega * rotor.radius_of_rotors) ** 2
    velocity = np.sqrt(velocity_squared)
    print(f"Relative velocity: {velocity:.2f} m/s")
    
    # Test elemental forces
    dL = 0.5 * density_2000m * chord * cl * velocity_squared * F
    dD = 0.5 * density_2000m * chord * cd * velocity_squared * F
    print(f"Elemental lift dL: {dL:.2f} N/m")
    print(f"Elemental drag dD: {dD:.2f} N/m")
    
    # Test thrust component
    dT = (dL * np.cos(phi) - dD * np.sin(phi))
    print(f"Elemental thrust dT: {dT:.2f} N/m")
    
    if abs(dT) < 1e-6:
        print("❌ Zero thrust - checking components:")
        print(f"  dL * cos(φ) = {dL:.6f} * {np.cos(phi):.6f} = {dL * np.cos(phi):.6f}")
        print(f"  dD * sin(φ) = {dD:.6f} * {np.sin(phi):.6f} = {dD * np.sin(phi):.6f}")
        
        if abs(cl) < 1e-6:
            print("❌ Zero lift coefficient - possible issues:")
            print("    1. Angle of attack too low")
            print("    2. Stall condition")
            print("    3. Airfoil data problem")
    else:
        print("✅ Non-zero thrust element found")

if __name__ == "__main__":
    debug_bemt_components()
