#!/usr/bin/env python3
"""
Mission Planner Test - Hover Endurance at 2000m AMSL
"""

from heli import Environment, Rotor, Helicopter, MissionPlanner
import numpy as np
from time import sleep

def test_mission_planner_hover_endurance():
    """Test hover endurance at 2000m AMSL using Mission Planner"""
    print("!!!Preparing to test mission planner!!!")
    print("Gods above and below, have mercy on us\n")
    
    
    print("Step 1: Setting up atmospheric environment...")
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
    
    
    temp_2000m = env.get_temperature(altitude=2000)
    pressure_2000m = env.get_pressure(temp_2000m, altitude=2000)
    density_2000m = env.get_density(temp_2000m, pressure_2000m)
    
    print(f"  Atmospheric conditions at 2000m:")
    print(f"    Temperature: {temp_2000m:.1f} K ({temp_2000m-273.15:.1f}°C)")
    print(f"    Pressure: {pressure_2000m:.0f} Pa")
    print(f"    Density: {density_2000m:.3f} kg/m³")
    print()
    
    sleep(0.5)
    print("\n\nSetting up main rotor...")
    main_rotor = Rotor(environment=env)
    main_rotor.set_rotor_parameters(
        number_of_blades=4,           
        blade_mass=20,                
        NACA_for_airfoil="0012",      
        radius_of_rotors=0.762,         
        root_cutout=0.125,              
        root_chord=0.0508,               
        tip_chord=0.05,                
        root_pitch=5.0,              
        slope_pitch=0.08              # Blade twist
    )
    print(f"  Main rotor: {main_rotor.number_of_blades} blades, {main_rotor.radius_of_rotors}m radius")
    print("\n")
    
    
    print("Setting up tail rotor...")
    tail_rotor = Rotor(environment=env)
    tail_rotor.set_rotor_parameters(
        number_of_blades=4,           
        blade_mass=3,                 
        NACA_for_airfoil="0012",      
        radius_of_rotors=0.2,         
        root_cutout=0.03,             
        root_chord=0.15,              
        tip_chord=0.08,               
        root_pitch=4.0,             
        slope_pitch=0.05              
    )
    print(f"  Tail rotor: {tail_rotor.number_of_blades} blades, {tail_rotor.radius_of_rotors}m radius")
    print()
    
    
    print("Step 4: Setting up helicopter...")
    heli = Helicopter(environment=env)
    heli.set_rotor(main_rotor, tail_rotor)
    
    # Set rotor positions 
    heli.set_rotor_positions(
        main_rotor_position=5.0,    # Main rotor at 5m from nose
        tail_rotor_position=11.0    # Tail rotor at 11m from nose (6m separation)
    )
    
    # Set fuselage parameters 
    heli.set_fuselage_parameters(
        mass=300,      
        length=12.0,    
        width=2.5,      
        height=3.0,     
        cl=0.1,         
        cd=0.8          
    )
    
    # Set engine parameters (realistic turbine engine)
    heli.set_engine_parameters(
        mass=180,      
        BSFC=0.35,     
        shaft_power_conversion_efficiency=0.90, 
        max_power=1500 
    )
    
    total_mass = heli.get_total_mass()
    print(f"  Total helicopter mass: {total_mass:.0f} kg")
    print(f"  Engine max power: {heli.engine_max_power} kW")
    print()
    
    
    print("Setting up mission planner...")
    mission_planner = MissionPlanner(heli)
    
    
    mission_planner.set_flight_parameters_programmatic(
        dry_weight=total_mass,        
        fuel_weight=300,              
        fuel_specific_energy_kj_kg=43000,  
        reserve_fuel_fraction=0.20    
    )
    
    print(f"  Mission parameters:")
    print(f"    Dry weight: {mission_planner.dry_weight} kg")
    print(f"    Fuel weight: {mission_planner.fuel_weight} kg")
    print(f"    Usable fuel: {mission_planner.fuel_weight - mission_planner.min_fuel} kg")
    print(f"    Fuel energy: {mission_planner.fuel_specific_energy/1e6:.1f} MJ/kg")
    print()
    
    
    print("Step 6: Calculating hover endurance at 2000m AMSL...")
    
    try:
        
        hover_time, error = mission_planner.find_available_hover_time(altitude=2000.0, dt=60.0)
        
        if error:
            if hover_time == -2:
                print("!!! Flight parameters not set")
            elif hover_time == 0:
                print("!!! Helicopter cannot hover - stalling condition")
            else:
                print(f"!!! Error code: {hover_time}")
        else:
            hours = hover_time / 3600
            minutes = (hover_time % 3600) / 60
            print(f"Hover endurance at 2000m AMSL: {hover_time:.0f} seconds")
            print(f"Hover endurance: {hours:.1f} hours ({hours*60:.0f} minutes)")
            
            
            total_weight = mission_planner.dry_weight + mission_planner.fuel_weight
            usable_fuel = mission_planner.fuel_weight - mission_planner.min_fuel

            print(f"\n--- Mission Analysis ---")
            print(f"    Total weight at start: {total_weight:.0f} kg")
            print(f"    Usable fuel for hover: {usable_fuel:.0f} kg")
            print(f"    Average fuel consumption rate: {usable_fuel/hover_time*3600:.1f} kg/hour")
            
            
            omega_test = 2 * np.pi * 400 / 60  # 400 RPM test
            performance = heli.main_rotor.calculate_performance(0, omega_test, density_2000m)
            print(f"    Estimated power required: {performance['power']/1000:.0f} kW")
        
    except Exception as e:
        print(f"❌ Error during hover endurance calculation: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    test_mission_planner_hover_endurance()
