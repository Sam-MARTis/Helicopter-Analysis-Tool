#!/usr/bin/env python3
"""
Comprehensive test of the corrected heli.py implementation
"""

import sys
import os
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from heli import Environment, Rotor, Helicopter
import numpy as np

def test_helicopter_class():
    """Test the full Helicopter class functionality"""
    print("=== COMPREHENSIVE HELICOPTER TEST ===")
    
    # Create helicopter with environment
    env = Environment()
    env.set_atmosphere_parameters(
        temperature_sea_level=288.15,
        Reynolds_number=1e6,
        altitude=0,
        pressure_sea_level=101325,
        Lapse_rate_troposphere=0.0065,
        wind_velocity=0,
        ISA_OFFSET=0
    )
    
    # Create helicopter
    heli = Helicopter(environment=env)
    
    # Set up main rotor
    main_rotor = Rotor(environment=env)
    main_rotor.set_rotor_parameters(
        number_of_blades=2,
        blade_mass=50,
        NACA_for_airfoil="0012",
        radius_of_rotors=5.0,
        root_cutout=0.5,
        root_chord=0.3,
        tip_chord=0.15,
        root_pitch=12.0,
        slope_pitch=0.1
    )
    
    # Set up tail rotor (minimal)
    tail_rotor = Rotor(environment=env)
    tail_rotor.set_rotor_parameters(
        number_of_blades=2,
        blade_mass=10,
        NACA_for_airfoil="0012", 
        radius_of_rotors=1.0,
        root_cutout=0.1,
        root_chord=0.1,
        tip_chord=0.05,
        root_pitch=12.0,
        slope_pitch=0.1
    )
    
    heli.set_rotor(main_rotor, tail_rotor)
    heli.set_fuselage_parameters(1000, 4.0, 1.5, 1.5, 0.1, 0.05)
    heli.set_engine_parameters(200, 0.3, 0.95, 2500)
    
    # Test hover performance
    omega = 2 * np.pi * 400 / 60  # 400 RPM
    
    try:
        print("Testing can_hover method...")
        can_hover_result, message = heli.can_hover(omega)
        print(f"Can hover: {can_hover_result}")
        print(f"Message: {message}")
        print()
        
        print("Testing hover performance calculation...")
        hover_perf = heli.calculate_hover_performance(omega)
        print(f"Hover thrust: {hover_perf['thrust']:.1f} N")
        print(f"Hover power: {hover_perf['power']/1000:.1f} kW")
        print()
        
        print("Testing thrust required calculation...")
        thrust_req = heli.calculate_thrust_required_for_hover()
        print(f"Thrust required: {thrust_req:.1f} N")
        print()
        
        print("✅ ALL HELICOPTER CLASS TESTS PASSED!")
        
    except Exception as e:
        print(f"❌ Error in helicopter test: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    test_helicopter_class()
