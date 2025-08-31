#!/usr/bin/env python3

import numpy as np
import matplotlib.pyplot as plt
from heli import Environment, Rotor, Helicopter, MissionPlanner
import pandas as pd

def create_base_rotor(env):
    rotor = Rotor(environment=env)
    rotor.set_rotor_parameters(
        number_of_blades=3,
        blade_mass=15,
        NACA_for_airfoil="0012",
        radius_of_rotors=0.762,
        root_cutout=0.125,
        root_chord=0.0508,
        tip_chord=0.0508,
        root_pitch=12.0,
        slope_pitch=0.0
    )
    return rotor

def create_rotor_performance_plots():
    env = Environment()
    env.set_atmosphere_parameters(
        temperature_sea_level=288.15,
        Reynolds_number=1e6,
        altitude=0.0,
        pressure_sea_level=101325,
        Lapse_rate_troposphere=0.0065,
        wind_velocity=0,
        ISA_OFFSET=0
    )
    
    density_sl = env.get_density(env.get_temperature(0), env.get_pressure(env.get_temperature(0), 0))
    
    plt.style.use('default')
    fig = plt.figure(figsize=(20, 15))
    
    ax1 = plt.subplot(3, 3, 1)
    rotor = create_base_rotor(env)
    rpms = np.linspace(800, 2000, 20)
    thrusts = []
    powers = []
    
    for rpm in rpms:
        omega = 2 * np.pi * rpm / 60
        try:
            performance = rotor.calculate_performance(0, omega, density_sl)
            thrusts.append(performance['thrust'])
            powers.append(performance['power'] / 1000)
        except:
            thrusts.append(0)
            powers.append(0)
    
    ax1.plot(thrusts, powers, 'b-o', linewidth=2, markersize=4)
    ax1.set_xlabel('Thrust (N)')
    ax1.set_ylabel('Power (kW)')
    ax1.set_title('Power vs Thrust')
    ax1.grid(True, alpha=0.3)
    
    ax2 = plt.subplot(3, 3, 2)
    taper_ratios = np.linspace(0.3, 1.0, 15)
    target_thrust = 1000
    thrusts_taper = []
    
    for taper in taper_ratios:
        rotor = create_base_rotor(env)
        rotor.tip_chord = rotor.root_chord * taper
        
        best_thrust = 0
        for rpm in np.linspace(800, 2000, 50):
            omega = 2 * np.pi * rpm / 60
            try:
                performance = rotor.calculate_performance(0, omega, density_sl)
                if abs(performance['thrust'] - target_thrust) < abs(best_thrust - target_thrust):
                    best_thrust = performance['thrust']
            except:
                continue
        thrusts_taper.append(best_thrust)
    
    ax2.plot(taper_ratios, thrusts_taper, 'r-s', linewidth=2, markersize=4)
    ax2.set_xlabel('Taper Ratio')
    ax2.set_ylabel('Thrust (N)')
    ax2.set_title('Thrust vs Taper Ratio (Fixed Thrust)')
    ax2.grid(True, alpha=0.3)
    
    ax3 = plt.subplot(3, 3, 3)
    target_power = 15000
    thrusts_fixed_power = []
    
    for taper in taper_ratios:
        rotor = create_base_rotor(env)
        rotor.tip_chord = rotor.root_chord * taper
        
        best_thrust = 0
        best_diff = float('inf')
        for rpm in np.linspace(800, 2000, 50):
            omega = 2 * np.pi * rpm / 60
            try:
                performance = rotor.calculate_performance(0, omega, density_sl)
                power_diff = abs(performance['power'] - target_power)
                if power_diff < best_diff:
                    best_diff = power_diff
                    best_thrust = performance['thrust']
            except:
                continue
        thrusts_fixed_power.append(best_thrust)
    
    ax3.plot(taper_ratios, thrusts_fixed_power, 'g-^', linewidth=2, markersize=4)
    ax3.set_xlabel('Taper Ratio')
    ax3.set_ylabel('Thrust (N)')
    ax3.set_title('Thrust vs Taper Ratio (Fixed Power)')
    ax3.grid(True, alpha=0.3)
    
    ax4 = plt.subplot(3, 3, 4)
    rotor = create_base_rotor(env)
    r_positions = np.linspace(rotor.root_cutout, rotor.radius_of_rotors, 50)
    
    for taper, color in [(0.5, 'blue'), (0.8, 'green'), (1.0, 'red')]:
        rotor.tip_chord = rotor.root_chord * taper
        chord_dist = [rotor.get_chord_length(r) for r in r_positions]
        ax4.plot(r_positions, chord_dist, color=color, linewidth=2, label=f'Taper {taper}')
    
    ax4.set_xlabel('Radial Position (m)')
    ax4.set_ylabel('Chord Length (m)')
    ax4.set_title('Chord Length Distribution')
    ax4.legend()
    ax4.grid(True, alpha=0.3)
    
    ax5 = plt.subplot(3, 3, 5)
    rotor = create_base_rotor(env)
    
    for twist, color in [(0.0, 'blue'), (0.05, 'green'), (0.1, 'red')]:
        rotor.slope_pitch = twist
        pitch_dist = []
        for r in r_positions:
            pitch = rotor.root_pitch + rotor.slope_pitch * (r - rotor.root_cutout)
            pitch_dist.append(np.rad2deg(pitch))
        ax5.plot(r_positions, pitch_dist, color=color, linewidth=2, label=f'Twist {twist}')
    
    ax5.set_xlabel('Radial Position (m)')
    ax5.set_ylabel('Blade Pitch (degrees)')
    ax5.set_title('Blade Twist Distribution')
    ax5.legend()
    ax5.grid(True, alpha=0.3)
    
    ax6 = plt.subplot(3, 3, 6)
    rpms_torque = np.linspace(800, 1800, 15)
    
    for n_blades, color in [(2, 'blue'), (3, 'green'), (4, 'red'), (5, 'purple')]:
        torques = []
        for rpm in rpms_torque:
            rotor = create_base_rotor(env)
            rotor.number_of_blades = n_blades
            omega = 2 * np.pi * rpm / 60
            try:
                performance = rotor.calculate_performance(0, omega, density_sl)
                torques.append(performance['torque'])
            except:
                torques.append(0)
        ax6.plot(rpms_torque, torques, color=color, linewidth=2, marker='o', markersize=3, label=f'{n_blades} Blades')
    
    ax6.set_xlabel('RPM')
    ax6.set_ylabel('Torque (Nm)')
    ax6.set_title('Torque vs RPM')
    ax6.legend()
    ax6.grid(True, alpha=0.3)
    
    env_2000 = Environment()
    env_2000.set_atmosphere_parameters(
        temperature_sea_level=288.15,
        Reynolds_number=1e6,
        altitude=2000.0,
        pressure_sea_level=101325,
        Lapse_rate_troposphere=0.0065,
        wind_velocity=0,
        ISA_OFFSET=0
    )
    
    density_2000 = env_2000.get_density(env_2000.get_temperature(2000), env_2000.get_pressure(env_2000.get_temperature(2000), 2000))
    
    ax7 = plt.subplot(3, 3, 7)
    gross_weights = np.linspace(100, 500, 20)
    fuel_burn_rates = []
    
    for weight in gross_weights:
        thrust_required = weight * 9.81
        
        best_fuel_flow = 0
        for rpm in np.linspace(1000, 2000, 30):
            omega = 2 * np.pi * rpm / 60
            try:
                rotor = create_base_rotor(env_2000)
                performance = rotor.calculate_performance(0, omega, density_2000)
                if abs(performance['thrust'] - thrust_required) < 100:
                    power_kw = performance['power'] / 1000
                    fuel_flow = power_kw * 0.3
                    best_fuel_flow = fuel_flow
                    break
            except:
                continue
        fuel_burn_rates.append(best_fuel_flow)
    
    ax7.plot(gross_weights, fuel_burn_rates, 'b-o', linewidth=2, markersize=4)
    ax7.set_xlabel('Gross Weight (kg)')
    ax7.set_ylabel('Fuel Burn Rate (kg/hr)')
    ax7.set_title('Fuel Burn Rate vs Gross Weight (2000m)')
    ax7.grid(True, alpha=0.3)
    
    ax8 = plt.subplot(3, 3, 8)
    takeoff_weights = np.linspace(150, 400, 10)
    endurances = []
    
    for weight in takeoff_weights:
        heli = Helicopter(env_2000)
        rotor = create_base_rotor(env_2000)
        heli.main_rotor = rotor
        heli.main_rotor_set = True
        
        mission = MissionPlanner(heli)
        mission.set_flight_parameters_programmatic(
            dry_weight=weight*0.7,
            fuel_weight=weight*0.3,
            fuel_specific_energy_kj_kg=43.0,
            reserve_fuel_fraction=0.2
        )
        
        try:
            endurance_seconds, error = mission.find_available_hover_time(altitude=2000.0, dt=300)
            if not error:
                endurances.append(endurance_seconds / 60)
            else:
                endurances.append(0)
        except:
            endurances.append(0)
    
    ax8.plot(takeoff_weights, endurances, 'r-s', linewidth=2, markersize=4)
    ax8.set_xlabel('Takeoff Weight (kg)')
    ax8.set_ylabel('Hover Endurance (minutes)')
    ax8.set_title('Hover Endurance vs Takeoff Weight (2000m)')
    ax8.grid(True, alpha=0.3)
    
    ax9 = plt.subplot(3, 3, 9)
    collectives = np.linspace(6, 18, 15)
    omega_fixed = 2 * np.pi * 1200 / 60
    efficiencies = []
    
    for collective in collectives:
        rotor = create_base_rotor(env_2000)
        rotor.root_pitch = np.deg2rad(collective)
        try:
            performance = rotor.calculate_performance(0, omega_fixed, density_2000)
            thrust = performance['thrust']
            power = performance['power']
            
            disc_area = np.pi * rotor.radius_of_rotors**2
            ideal_power = thrust * np.sqrt(thrust / (2 * density_2000 * disc_area))
            efficiency = ideal_power / power if power > 0 else 0
            efficiencies.append(efficiency)
        except:
            efficiencies.append(0)
    
    ax9.plot(collectives, efficiencies, 'g-^', linewidth=2, markersize=4)
    ax9.set_xlabel('Collective Pitch (degrees)')
    ax9.set_ylabel('Efficiency')
    ax9.set_title('Rotor Efficiency vs Collective Pitch')
    ax9.grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.savefig('rotor_analysis_complete.png', dpi=300, bbox_inches='tight')
    
    # Generate data CSV
    generate_performance_data(env)
    
    plt.show()

def generate_performance_data(env):
    density_sl = env.get_density(env.get_temperature(0), env.get_pressure(env.get_temperature(0), 0))
    
    data = []
    rpms = [1000, 1200, 1400, 1600, 1800]
    collectives = [8, 10, 12, 14, 16]
    taper_ratios = [0.5, 0.7, 0.9, 1.0]
    
    for rpm in rpms:
        for collective in collectives:
            for taper in taper_ratios:
                rotor = create_base_rotor(env)
                rotor.root_pitch = np.deg2rad(collective)
                rotor.tip_chord = rotor.root_chord * taper
                
                omega = 2 * np.pi * rpm / 60
                
                try:
                    performance = rotor.calculate_performance(0, omega, density_sl)
                    thrust = performance['thrust']
                    power = performance['power']
                    torque = performance['torque']
                    
                    disc_area = np.pi * rotor.radius_of_rotors**2
                    ideal_power = thrust * np.sqrt(thrust / (2 * density_sl * disc_area))
                    efficiency = ideal_power / power if power > 0 else 0
                    
                    data.append({
                        'RPM': rpm,
                        'Collective': collective,
                        'Taper_Ratio': taper,
                        'Thrust_N': thrust,
                        'Power_kW': power/1000,
                        'Torque_Nm': torque,
                        'Efficiency': efficiency
                    })
                    
                except Exception:
                    continue
    
    df = pd.DataFrame(data)
    df.to_csv('rotor_complete_data.csv', index=False)
    print(f"Performance data saved to 'rotor_complete_data.csv' ({len(data)} data points)")

if __name__ == "__main__":
    print("Creating comprehensive rotor analysis plots...")
    create_rotor_performance_plots()
    print("All plots completed successfully!")
