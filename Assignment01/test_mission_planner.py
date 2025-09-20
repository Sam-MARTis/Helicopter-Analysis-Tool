#!/usr/bin/env python3
"""
Mission Planner Test - Hover Endurance at 2000m AMSL
"""

import sys
import os

from heli import Enviro    # Step 6: Calculate hover endurance at 2000m
    print("Step 6: Calculating hover endurance at 2000m AMSL...")
    
    try:
        hover_time, error = mission_planner.find_available_hover_time(altitude=2000.0, dt=60.0)  # 60 second (1 minute) time steps, Rotor, Helicopter, MissionPlanner
import numpy as np

def test_mission_planner_hover_endurance():
    """Test hover endurance at 2000m AMSL using Mission Planner"""
    print("=== MISSION PLANNER HOVER ENDURANCE TEST ===")
    print("Testing hover endurance at 2000m AMSL")
    print()
    
    # Step 1: Create environment with proper atmospheric conditions
    print("Step 1: Setting up atmospheric environment...")
    env = Environment()
    env.set_atmosphere_parameters(
        temperature_sea_level=288.15,  # 15°C at sea level
        Reynolds_number=1e6,
        altitude=2000.0,  # 2000m AMSL
        pressure_sea_level=101325,
        Lapse_rate_troposphere=0.0065,  # Standard lapse rate
        wind_velocity=0,
        ISA_OFFSET=0
    )
    
    # Calculate atmospheric conditions at 2000m
    temp_2000m = env.get_temperature(altitude=2000)
    pressure_2000m = env.get_pressure(temp_2000m, altitude=2000)
    density_2000m = env.get_density(temp_2000m, pressure_2000m)
    
    print(f"  Atmospheric conditions at 2000m:")
    print(f"    Temperature: {temp_2000m:.1f} K ({temp_2000m-273.15:.1f}°C)")
    print(f"    Pressure: {pressure_2000m:.0f} Pa")
    print(f"    Density: {density_2000m:.3f} kg/m³")
    print()
    
    # Step 2: Create and configure main rotor (realistic helicopter parameters)
    print("Step 2: Setting up main rotor...")
    main_rotor = Rotor(environment=env)
    main_rotor.set_rotor_parameters(
        number_of_blades=2,           # 2-blade main rotor
        blade_mass=75,                # 75 kg per blade
        NACA_for_airfoil="0012",      # NACA 0012 airfoil
        radius_of_rotors=6.0,         # 6m radius (realistic)
        root_cutout=0.6,              # 0.6m root cutout
        angle_of_attack=8.0,          # 8 degrees
        root_chord=0.4,               # 0.4m root chord
        tip_chord=0.2,                # 0.2m tip chord
        root_pitch=14.0,              # 14 degrees collective
        slope_pitch=0.08              # Blade twist
    )
    print(f"  Main rotor: {main_rotor.number_of_blades} blades, {main_rotor.radius_of_rotors}m radius")
    print()
    
    # Step 3: Create and configure tail rotor
    print("Step 3: Setting up tail rotor...")
    tail_rotor = Rotor(environment=env)
    tail_rotor.set_rotor_parameters(
        number_of_blades=4,           # 4-blade tail rotor
        blade_mass=8,                 # 8 kg per blade
        NACA_for_airfoil="0012",      
        radius_of_rotors=1.2,         # 1.2m radius
        root_cutout=0.15,             # 0.15m root cutout
        angle_of_attack=8.0,          
        root_chord=0.15,              # 0.15m root chord
        tip_chord=0.08,               # 0.08m tip chord
        root_pitch=12.0,              # 12 degrees
        slope_pitch=0.05              
    )
    print(f"  Tail rotor: {tail_rotor.number_of_blades} blades, {tail_rotor.radius_of_rotors}m radius")
    print()
    
    # Step 4: Create helicopter and set parameters
    print("Step 4: Setting up helicopter...")
    heli = Helicopter(environment=env)
    heli.set_rotor(main_rotor, tail_rotor)
    
    # Set rotor positions (realistic for medium helicopter)
    heli.set_rotor_positions(
        main_rotor_position=5.0,    # Main rotor at 5m from nose
        tail_rotor_position=11.0    # Tail rotor at 11m from nose (6m separation)
    )
    
    # Set fuselage parameters (realistic medium helicopter)
    heli.set_fuselage_parameters(
        mass=1200,      # 1200 kg fuselage
        length=12.0,    # 12m length
        width=2.5,      # 2.5m width  
        height=3.0,     # 3m height
        cl=0.1,         # Fuselage lift coefficient
        cd=0.8          # Fuselage drag coefficient
    )
    
    # Set engine parameters (realistic turbine engine)
    heli.set_engine_parameters(
        mass=180,       # 180 kg engine
        BSFC=0.35,      # 0.35 kg/kWh brake specific fuel consumption
        shaft_power_conversion_efficiency=0.93,  # 93% efficiency
        max_power=1500  # 1500 kW maximum power
    )
    
    total_mass = heli.get_total_mass()
    print(f"  Total helicopter mass: {total_mass:.0f} kg")
    print(f"  Engine max power: {heli.engine_max_power} kW")
    print()
    
    # Step 5: Create mission planner and set flight parameters
    print("Step 5: Setting up mission planner...")
    mission_planner = MissionPlanner(heli)
    
    # Set realistic flight parameters
    mission_planner.set_flight_parameters_programmatic(
        dry_weight=total_mass,        # Use calculated helicopter mass
        fuel_weight=300,              # 300 kg fuel capacity
        fuel_specific_energy_kj_kg=43000,  # Jet fuel ~43 MJ/kg
        reserve_fuel_fraction=0.20    # 20% fuel reserve
    )
    
    print(f"  Mission parameters:")
    print(f"    Dry weight: {mission_planner.dry_weight} kg")
    print(f"    Fuel weight: {mission_planner.fuel_weight} kg")
    print(f"    Usable fuel: {mission_planner.fuel_weight - mission_planner.min_fuel} kg")
    print(f"    Fuel energy: {mission_planner.fuel_specific_energy/1e6:.1f} MJ/kg")
    print()
    
    # Step 6: Test hover endurance at 2000m
    print("Step 6: Calculating hover endurance at 2000m AMSL...")
    
    try:
        hover_time, error = mission_planner.find_available_hover_time(altitude=2000.0, dt=60.0)  # 60 second (1 minute) time steps
        
        if error:
            if hover_time == -2:
                print("❌ Flight parameters not set")
            elif hover_time == 0:
                print("❌ Helicopter cannot hover - stalling condition")
            else:
                print(f"❌ Error code: {hover_time}")
        else:
            hours = hover_time / 3600
            minutes = (hover_time % 3600) / 60
            print(f"✅ Hover endurance at 2000m AMSL: {hover_time:.0f} seconds")
            print(f"✅ Hover endurance: {hours:.1f} hours ({hours*60:.0f} minutes)")
            
            # Additional analysis
            total_weight = mission_planner.dry_weight + mission_planner.fuel_weight
            usable_fuel = mission_planner.fuel_weight - mission_planner.min_fuel
            
            print(f"\n📊 Mission Analysis:")
            print(f"    Total weight at start: {total_weight:.0f} kg")
            print(f"    Usable fuel for hover: {usable_fuel:.0f} kg")
            print(f"    Fuel consumption rate: {usable_fuel/hover_time*3600:.1f} kg/hour")
            
            # Calculate power requirements
            omega_test = 2 * np.pi * 400 / 60  # 400 RPM test
            performance = heli.main_rotor.calculate_performance(0, omega_test, density_2000m)
            print(f"    Estimated power required: {performance['power']/1000:.0f} kW")
        
    except Exception as e:
        print(f"❌ Error during hover endurance calculation: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    test_mission_planner_hover_endurance()
