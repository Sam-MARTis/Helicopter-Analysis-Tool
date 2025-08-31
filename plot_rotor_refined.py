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
    fig = plt.figure(figsize=(16, 12))
    
    # Plot 1: Power vs Thrust
    ax1 = plt.subplot(2, 3, 1)
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
    
    # Plot 2: Thrust vs Taper Ratio (Fixed Thrust)
    ax2 = plt.subplot(2, 3, 2)
    taper_ratios = np.linspace(0.3, 1.0, 15)
    target_thrust = 1000  # N
    thrusts_taper = []
    
    for taper in taper_ratios:
        rotor = create_base_rotor(env)
        rotor.tip_chord = rotor.root_chord * taper
        
        for rpm in np.linspace(800, 2000, 50):
            omega = 2 * np.pi * rpm / 60
            try:
                performance = rotor.calculate_performance(0, omega, density_sl)
                if abs(performance['thrust'] - target_thrust) < 50:
                    thrusts_taper.append(performance['thrust'])
                    break
            except:
                continue
        else:
            thrusts_taper.append(0)
    
    ax2.plot(taper_ratios, thrusts_taper, 'r-s', linewidth=2, markersize=4)
    ax2.set_xlabel('Taper Ratio')
    ax2.set_ylabel('Thrust (N)')
    ax2.set_title('Thrust vs Taper Ratio (Fixed Thrust)')
    ax2.grid(True, alpha=0.3)
    
    # Plot 3: Thrust vs Taper Ratio (Fixed Power)
    ax3 = plt.subplot(2, 3, 3)
    target_power = 15000  # W
    thrusts_fixed_power = []
    
    for taper in taper_ratios:
        rotor = create_base_rotor(env)
        rotor.tip_chord = rotor.root_chord * taper
        
        for rpm in np.linspace(800, 2000, 50):
            omega = 2 * np.pi * rpm / 60
            try:
                performance = rotor.calculate_performance(0, omega, density_sl)
                if abs(performance['power'] - target_power) < 1000:
                    thrusts_fixed_power.append(performance['thrust'])
                    break
            except:
                continue
        else:
            thrusts_fixed_power.append(0)
    
    ax3.plot(taper_ratios, thrusts_fixed_power, 'g-^', linewidth=2, markersize=4)
    ax3.set_xlabel('Taper Ratio')
    ax3.set_ylabel('Thrust (N)')
    ax3.set_title('Thrust vs Taper Ratio (Fixed Power)')
    ax3.grid(True, alpha=0.3)
    
    # Plot 4: Chord Length Distribution
    ax4 = plt.subplot(2, 3, 4)
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
    
    # Plot 5: Twist Distribution
    ax5 = plt.subplot(2, 3, 5)
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
    
    # Plot 6: Torque vs RPM
    ax6 = plt.subplot(2, 3, 6)
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
    
    plt.tight_layout()
    plt.savefig('rotor_performance_analysis.png', dpi=300, bbox_inches='tight')
    plt.show()

def plot_mission_analysis():
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
    
    fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2, figsize=(14, 10))
    
    # Plot 1: Fuel burn rate vs gross weight at 2000m
    gross_weights = np.linspace(100, 500, 20)  # kg
    fuel_burn_rates = []
    
    for weight in gross_weights:
        rotor = create_base_rotor(env)
        mission = MissionPlanner(rotor, env)
        mission.helicopter_weight = weight
        
        thrust_required = weight * 9.81
        
        for rpm in np.linspace(1000, 2000, 30):
            omega = 2 * np.pi * rpm / 60
            try:
                performance = rotor.calculate_performance(0, omega, env.get_density(env.get_temperature(2000), env.get_pressure(env.get_temperature(2000), 2000)))
                if abs(performance['thrust'] - thrust_required) < 50:
                    power_kw = performance['power'] / 1000
                    fuel_flow = power_kw * 0.3  # kg/hour (approximate fuel consumption)
                    fuel_burn_rates.append(fuel_flow)
                    break
            except:
                continue
        else:
            fuel_burn_rates.append(0)
    
    ax1.plot(gross_weights, fuel_burn_rates, 'b-o', linewidth=2, markersize=4)
    ax1.set_xlabel('Gross Weight (kg)')
    ax1.set_ylabel('Fuel Burn Rate (kg/hr)')
    ax1.set_title('Fuel Burn Rate vs Gross Weight (2000m)')
    ax1.grid(True, alpha=0.3)
    
    # Plot 2: Hover endurance vs takeoff weight
    takeoff_weights = np.linspace(150, 400, 15)  # kg
    endurances = []
    
    for weight in takeoff_weights:
        rotor = create_base_rotor(env)
        mission = MissionPlanner(rotor, env)
        mission.helicopter_weight = weight
        mission.fuel_weight = weight * 0.3  # 30% fuel fraction
        
        try:
            endurance_hours = mission.calculate_hover_endurance(2000.0, dt=300)
            endurances.append(endurance_hours * 60)  # Convert to minutes
        except:
            endurances.append(0)
    
    ax2.plot(takeoff_weights, endurances, 'r-s', linewidth=2, markersize=4)
    ax2.set_xlabel('Takeoff Weight (kg)')
    ax2.set_ylabel('Hover Endurance (minutes)')
    ax2.set_title('Hover Endurance vs Takeoff Weight (2000m)')
    ax2.grid(True, alpha=0.3)
    
    # Plot 3: Power required vs altitude for different weights
    altitudes = np.linspace(0, 4000, 20)
    
    for weight, color in [(200, 'blue'), (300, 'green'), (400, 'red')]:
        powers_alt = []
        for alt in altitudes:
            env_temp = Environment()
            env_temp.set_atmosphere_parameters(
                temperature_sea_level=288.15, Reynolds_number=1e6, altitude=alt,
                pressure_sea_level=101325, Lapse_rate_troposphere=0.0065,
                wind_velocity=0, ISA_OFFSET=0
            )
            
            rotor = create_base_rotor(env_temp)
            thrust_required = weight * 9.81
            density_alt = env_temp.get_density(env_temp.get_temperature(alt), env_temp.get_pressure(env_temp.get_temperature(alt), alt))
            
            for rpm in np.linspace(1000, 2500, 40):
                omega = 2 * np.pi * rpm / 60
                try:
                    performance = rotor.calculate_performance(0, omega, density_alt)
                    if abs(performance['thrust'] - thrust_required) < 100:
                        powers_alt.append(performance['power'] / 1000)
                        break
                except:
                    continue
            else:
                powers_alt.append(0)
        
        ax3.plot(altitudes, powers_alt, color=color, linewidth=2, label=f'{weight} kg')
    
    ax3.set_xlabel('Altitude (m)')
    ax3.set_ylabel('Power Required (kW)')
    ax3.set_title('Power Required vs Altitude')
    ax3.legend()
    ax3.grid(True, alpha=0.3)
    
    # Plot 4: Efficiency vs collective pitch
    collectives = np.linspace(6, 18, 15)
    omega_fixed = 2 * np.pi * 1200 / 60
    
    rotor = create_base_rotor(env)
    efficiencies = []
    
    for collective in collectives:
        rotor.root_pitch = np.deg2rad(collective)
        try:
            performance = rotor.calculate_performance(0, omega_fixed, env.get_density(env.get_temperature(2000), env.get_pressure(env.get_temperature(2000), 2000)))
            thrust = performance['thrust']
            power = performance['power']
            
            disc_area = np.pi * rotor.radius_of_rotors**2
            ideal_power = thrust * np.sqrt(thrust / (2 * env.get_density(env.get_temperature(2000), env.get_pressure(env.get_temperature(2000), 2000)) * disc_area))
            efficiency = ideal_power / power if power > 0 else 0
            efficiencies.append(efficiency)
        except:
            efficiencies.append(0)
    
    ax4.plot(collectives, efficiencies, 'g-^', linewidth=2, markersize=4)
    ax4.set_xlabel('Collective Pitch (degrees)')
    ax4.set_ylabel('Efficiency')
    ax4.set_title('Rotor Efficiency vs Collective Pitch')
    ax4.grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.savefig('rotor_mission_analysis.png', dpi=300, bbox_inches='tight')
    plt.show()

if __name__ == "__main__":
    print("Creating rotor performance plots...")
    create_rotor_performance_plots()
    print("Creating mission analysis plots...")
    plot_mission_analysis()
    print("All plots completed successfully!")
