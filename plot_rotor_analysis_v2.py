#!/usr/bin/env python3
"""
Rotor Performance Plotting Analysis
Based on README.md specifications and parameter studies
"""

import numpy as np
import matplotlib.pyplot as plt
from heli import Environment, Rotor
import pandas as pd

def create_base_rotor(env):
    """Create base rotor using README.md specifications"""
    rotor = Rotor(environment=env)
    rotor.set_rotor_parameters(
        number_of_blades=3,           # Choose 3 blades from 2/3/4/5 options
        blade_mass=15,                # Estimated for small rotor
        NACA_for_airfoil="0012",      # Standard symmetric airfoil
        radius_of_rotors=0.762,       # From README: 0.762m
        root_cutout=0.125,            # From README: 0.125m
        root_chord=0.0508,            # From README: 0.0508m
        tip_chord=0.0508,             # Constant chord (no taper initially)
        root_pitch=12.0,              # 12 degrees collective
        slope_pitch=0.0               # No twist initially
    )
    return rotor

def create_rotor_performance_plots():
    """Create comprehensive rotor performance plots"""
    print("=== ROTOR PERFORMANCE PLOTTING ANALYSIS ===")
    print("Using parameters from README.md specifications")
    print()
    
    # Set up environment (sea level conditions)
    env = Environment()
    env.set_atmosphere_parameters(
        temperature_sea_level=288.15,
        Reynolds_number=1e6,
        altitude=0.0,  # Sea level
        pressure_sea_level=101325,
        Lapse_rate_troposphere=0.0065,
        wind_velocity=0,
        ISA_OFFSET=0
    )
    
    density_sl = env.get_density(env.get_temperature(0), env.get_pressure(env.get_temperature(0), 0))
    print(f"Sea level air density: {density_sl:.3f} kg/m³")
    print("README Specifications: R=0.762m, Root cutout=0.125m, Chord=0.0508m")
    print()
    
    # Set up plotting parameters
    plt.style.use('default')
    fig = plt.figure(figsize=(16, 12))
    
    # Plot 1: Thrust vs Power for different RPMs
    print("Generating Plot 1: Thrust vs Power...")
    ax1 = plt.subplot(2, 3, 1)
    
    rotor = create_base_rotor(env)
    rpms = np.linspace(1000, 3000, 20)  # RPM range for small rotor
    thrusts = []
    powers = []
    
    for rpm in rpms:
        omega = 2 * np.pi * rpm / 60
        try:
            performance = rotor.calculate_performance(0, omega, density_sl)
            thrusts.append(performance['thrust'])
            powers.append(performance['power'] / 1000)  # Convert to kW
        except:
            thrusts.append(0)
            powers.append(0)
    
    ax1.plot(powers, thrusts, 'b-o', linewidth=2, markersize=4)
    ax1.set_xlabel('Power (kW)')
    ax1.set_ylabel('Thrust (N)')
    ax1.set_title('Thrust vs Power\n(README Rotor: R=0.762m)')
    ax1.grid(True, alpha=0.3)
    
    # Plot 2: Thrust vs Taper Ratio
    print("Generating Plot 2: Thrust vs Taper Ratio...")
    ax2 = plt.subplot(2, 3, 2)
    
    taper_ratios = np.linspace(0.3, 1.0, 15)  # Tip/root chord ratio
    thrusts_taper = []
    powers_taper = []
    
    omega_fixed = 2 * np.pi * 1800 / 60  # Fixed RPM
    
    for taper in taper_ratios:
        rotor = create_base_rotor(env)
        rotor.tip_chord = rotor.root_chord * taper  # Apply taper ratio
        try:
            performance = rotor.calculate_performance(0, omega_fixed, density_sl)
            thrusts_taper.append(performance['thrust'])
            powers_taper.append(performance['power'] / 1000)
        except:
            thrusts_taper.append(0)
            powers_taper.append(0)
    
    ax2.plot(taper_ratios, thrusts_taper, 'r-s', linewidth=2, markersize=4)
    ax2.set_xlabel('Taper Ratio (Tip/Root Chord)')
    ax2.set_ylabel('Thrust (N)')
    ax2.set_title('Thrust vs Taper Ratio\n(1800 RPM)')
    ax2.grid(True, alpha=0.3)
    
    # Plot 3: Power vs Taper Ratio
    print("Generating Plot 3: Power vs Taper Ratio...")
    ax3 = plt.subplot(2, 3, 3)
    
    ax3.plot(taper_ratios, powers_taper, 'g-^', linewidth=2, markersize=4)
    ax3.set_xlabel('Taper Ratio (Tip/Root Chord)')
    ax3.set_ylabel('Power (kW)')
    ax3.set_title('Power vs Taper Ratio\n(1800 RPM)')
    ax3.grid(True, alpha=0.3)
    
    # Plot 4: Chord Length Distribution
    print("Generating Plot 4: Chord Length Distribution...")
    ax4 = plt.subplot(2, 3, 4)
    
    rotor = create_base_rotor(env)
    r_positions = np.linspace(rotor.root_cutout, rotor.radius_of_rotors, 50)
    
    # Test different taper ratios
    for taper, color, label in [(0.5, 'blue', 'Taper 0.5'), (0.7, 'green', 'Taper 0.7'), 
                               (0.9, 'orange', 'Taper 0.9'), (1.0, 'red', 'No Taper')]:
        rotor.tip_chord = rotor.root_chord * taper
        chord_dist = [rotor.get_chord_length(r) for r in r_positions]
        ax4.plot(r_positions, chord_dist, color=color, linewidth=2, label=label)
    
    ax4.set_xlabel('Radial Position (m)')
    ax4.set_ylabel('Chord Length (m)')
    ax4.set_title('Chord Length Distribution\n(README Specs)')
    ax4.legend()
    ax4.grid(True, alpha=0.3)
    
    # Plot 5: Twist Distribution and Angle of Attack
    print("Generating Plot 5: Twist and AOA Distribution...")
    ax5 = plt.subplot(2, 3, 5)
    
    rotor = create_base_rotor(env)
    
    # Test different twist rates
    for twist, color, label in [(0.0, 'black', 'No Twist'), (0.05, 'blue', 'Linear Twist 0.05'), 
                               (0.10, 'red', 'Linear Twist 0.10'), (0.15, 'green', 'Linear Twist 0.15')]:
        rotor.slope_pitch = twist
        
        # Calculate pitch distribution
        pitch_dist = []
        for r in r_positions:
            pitch = rotor.root_pitch + rotor.slope_pitch * (r - rotor.root_cutout)
            pitch_dist.append(np.rad2deg(pitch))
        
        ax5.plot(r_positions, pitch_dist, color=color, linewidth=2, label=label)
    
    # Add reference line for constant pitch
    constant_pitch_line = [12.0] * len(r_positions)  # 12 degree reference
    ax5.plot(r_positions, constant_pitch_line, 'k--', linewidth=1, alpha=0.7, label='Base Pitch (12°)')
    
    ax5.set_xlabel('Radial Position (m)')
    ax5.set_ylabel('Angle (degrees)')
    ax5.set_title('Blade Twist Distribution\n(Different Twist Rates)')
    ax5.legend()
    ax5.grid(True, alpha=0.3)
    
    # Plot 6: Torque vs RPM for different number of blades
    print("Generating Plot 6: Torque vs RPM (Blade Number Study)...")
    ax6 = plt.subplot(2, 3, 6)
    
    rpms_torque = np.linspace(1000, 2500, 15)
    
    for n_blades, color, label in [(2, 'blue', '2 Blades'), (3, 'green', '3 Blades'), 
                                  (4, 'red', '4 Blades'), (5, 'purple', '5 Blades')]:
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
        
        ax6.plot(rpms_torque, torques, color=color, linewidth=2, marker='o', markersize=3, label=label)
    
    ax6.set_xlabel('RPM')
    ax6.set_ylabel('Torque (Nm)')
    ax6.set_title('Torque vs RPM\n(README Specs, Blade Study)')
    ax6.legend()
    ax6.grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.savefig('rotor_performance_analysis.png', dpi=300, bbox_inches='tight')
    print("\n✅ Main plots saved as 'rotor_performance_analysis.png'")
    
    # Generate detailed performance data table
    print("\nGenerating detailed performance data...")
    create_performance_data_table(env)
    
    plt.show()

def create_performance_data_table(env):
    """Create detailed performance data table"""
    
    density_sl = env.get_density(env.get_temperature(0), env.get_pressure(env.get_temperature(0), 0))
    
    # Create base rotor from README specs
    rotor = create_base_rotor(env)
    
    # Performance data collection
    data = []
    
    # Test matrix: RPM vs Collective Pitch vs Taper Ratio
    rpms = [1200, 1500, 1800, 2100, 2400]
    collectives = [8, 10, 12, 14, 16]
    taper_ratios = [0.5, 0.7, 0.9, 1.0]
    
    print("\nPerformance Data Summary (README Specifications):")
    print("=" * 85)
    print(f"{'RPM':<6} {'Collective':<11} {'Taper':<7} {'Thrust(N)':<10} {'Power(kW)':<10} {'Torque(Nm)':<12} {'Efficiency':<10}")
    print("=" * 85)
    
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
                    
                    # Calculate figure of merit (efficiency)
                    disc_area = np.pi * rotor.radius_of_rotors**2
                    if thrust > 0 and power > 0:
                        ideal_power = thrust * np.sqrt(thrust / (2 * density_sl * disc_area))
                        efficiency = ideal_power / power
                    else:
                        efficiency = 0
                    
                    print(f"{rpm:<6} {collective:<11} {taper:<7.1f} {thrust:<10.0f} {power/1000:<10.1f} {torque:<12.0f} {efficiency:<10.3f}")
                    
                    data.append({
                        'RPM': rpm,
                        'Collective': collective,
                        'Taper_Ratio': taper,
                        'Thrust_N': thrust,
                        'Power_kW': power/1000,
                        'Torque_Nm': torque,
                        'Efficiency': efficiency
                    })
                    
                except Exception as e:
                    print(f"{rpm:<6} {collective:<11} {taper:<7.1f} {'ERROR':<10} {'ERROR':<10} {'ERROR':<12} {'ERROR':<10}")
    
    # Save data to CSV
    df = pd.DataFrame(data)
    df.to_csv('rotor_performance_data.csv', index=False)
    print(f"\n✅ Performance data saved to 'rotor_performance_data.csv'")
    print(f"   Data points: {len(data)} configurations tested")

def create_advanced_analysis_plots():
    """Create additional advanced analysis plots"""
    print("\n=== ADVANCED ROTOR ANALYSIS PLOTS ===")
    
    # Set up environment
    env = Environment()
    env.set_atmosphere_parameters(
        temperature_sea_level=288.15, Reynolds_number=1e6, altitude=0.0,
        pressure_sea_level=101325, Lapse_rate_troposphere=0.0065,
        wind_velocity=0, ISA_OFFSET=0
    )
    
    density_sl = env.get_density(env.get_temperature(0), env.get_pressure(env.get_temperature(0), 0))
    
    fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2, figsize=(14, 10))
    
    # Plot 1: Power Loading vs Disc Loading
    print("Creating power loading analysis...")
    rotor = create_base_rotor(env)
    rpms = np.linspace(1200, 2400, 15)
    collectives = [10, 12, 14, 16]
    
    for collective, color, marker in [(10, 'blue', 'o'), (12, 'green', 's'), (14, 'red', '^'), (16, 'purple', 'd')]:
        disc_loadings = []
        power_loadings = []
        
        for rpm in rpms:
            rotor = create_base_rotor(env)
            rotor.root_pitch = np.deg2rad(collective)
            omega = 2 * np.pi * rpm / 60
            
            try:
                performance = rotor.calculate_performance(0, omega, density_sl)
                thrust = performance['thrust']
                power = performance['power']
                
                disc_area = np.pi * rotor.radius_of_rotors**2
                disc_loading = thrust / disc_area if disc_area > 0 else 0
                power_loading = thrust / power if power > 0 else 0
                
                disc_loadings.append(disc_loading)
                power_loadings.append(power_loading)
            except:
                disc_loadings.append(0)
                power_loadings.append(0)
        
        ax1.plot(disc_loadings, power_loadings, color=color, marker=marker, linewidth=2,
                markersize=4, label=f'Collective: {collective}°')
    
    ax1.set_xlabel('Disc Loading (N/m²)')
    ax1.set_ylabel('Power Loading (N/W)')
    ax1.set_title('Power Loading vs Disc Loading\n(README Rotor)')
    ax1.legend()
    ax1.grid(True, alpha=0.3)
    
    # Plot 2: Efficiency vs RPM (Different Blade Numbers)
    print("Creating efficiency analysis...")
    rpms_eff = np.linspace(1200, 2400, 12)
    
    for n_blades, color, marker in [(2, 'blue', 'o'), (3, 'green', 's'), (4, 'red', '^'), (5, 'purple', 'd')]:
        efficiencies = []
        for rpm in rpms_eff:
            rotor = create_base_rotor(env)
            rotor.number_of_blades = n_blades
            omega = 2 * np.pi * rpm / 60
            
            try:
                performance = rotor.calculate_performance(0, omega, density_sl)
                thrust = performance['thrust']
                power = performance['power']
                
                if thrust > 0 and power > 0:
                    disc_area = np.pi * rotor.radius_of_rotors**2
                    ideal_power = thrust * np.sqrt(thrust / (2 * density_sl * disc_area))
                    efficiency = ideal_power / power
                else:
                    efficiency = 0
                
                efficiencies.append(efficiency)
            except:
                efficiencies.append(0)
        
        ax2.plot(rpms_eff, efficiencies, color=color, marker=marker, linewidth=2,
                markersize=4, label=f'{n_blades} Blades')
    
    ax2.set_xlabel('RPM')
    ax2.set_ylabel('Figure of Merit (Efficiency)')
    ax2.set_title('Efficiency vs RPM\n(Blade Number Study)')
    ax2.legend()
    ax2.grid(True, alpha=0.3)
    
    # Plot 3: Thrust vs Collective Pitch (Different Blade Numbers)
    print("Creating thrust vs collective pitch analysis...")
    collectives = np.linspace(6, 18, 15)
    omega_fixed = 2 * np.pi * 1800 / 60
    
    for n_blades, color, marker in [(2, 'blue', 'o'), (3, 'green', 's'), (4, 'red', '^'), (5, 'purple', 'd')]:
        thrusts_collective = []
        for collective in collectives:
            rotor = create_base_rotor(env)
            rotor.number_of_blades = n_blades
            rotor.root_pitch = np.deg2rad(collective)
            try:
                performance = rotor.calculate_performance(0, omega_fixed, density_sl)
                thrusts_collective.append(performance['thrust'])
            except:
                thrusts_collective.append(0)
        
        ax3.plot(collectives, thrusts_collective, color=color, marker=marker, linewidth=2,
                markersize=4, label=f'{n_blades} Blades')
    
    ax3.set_xlabel('Collective Pitch (degrees)')
    ax3.set_ylabel('Thrust (N)')
    ax3.set_title('Thrust vs Collective Pitch\n(Different Blade Numbers)')
    ax3.legend()
    ax3.grid(True, alpha=0.3)
    
    # Plot 4: Power vs Number of Blades (Different RPMs)
    print("Creating power vs blade number analysis...")
    blade_numbers = [2, 3, 4, 5]
    
    for rpm, color, marker in [(1200, 'blue', 'o'), (1500, 'green', 's'), (1800, 'red', '^'), (2100, 'purple', 'd')]:
        powers_blades = []
        for n_blades in blade_numbers:
            rotor = create_base_rotor(env)
            rotor.number_of_blades = n_blades
            omega = 2 * np.pi * rpm / 60
            
            try:
                performance = rotor.calculate_performance(0, omega, density_sl)
                powers_blades.append(performance['power'] / 1000)
            except:
                powers_blades.append(0)
        
        ax4.plot(blade_numbers, powers_blades, color=color, marker=marker, linewidth=2,
                markersize=6, label=f'{rpm} RPM')
    
    ax4.set_xlabel('Number of Blades')
    ax4.set_ylabel('Power (kW)')
    ax4.set_title('Power vs Number of Blades\n(Different RPMs)')
    ax4.legend()
    ax4.grid(True, alpha=0.3)
    ax4.set_xticks(blade_numbers)
    
    plt.tight_layout()
    plt.savefig('rotor_advanced_analysis.png', dpi=300, bbox_inches='tight')
    print("✅ Advanced plots saved as 'rotor_advanced_analysis.png'")
    
    plt.show()

if __name__ == "__main__":
    print("Starting rotor performance analysis...")
    print("Using README.md specifications:")
    print("  - Radius: 0.762m")
    print("  - Root cutout: 0.125m") 
    print("  - Chord length: 0.0508m")
    print("  - Number of blades: 2/3/4/5 (all tested)")
    print()
    
    create_rotor_performance_plots()
    create_advanced_analysis_plots()
    
    print("\n🎯 Analysis Complete!")
    print("Generated files:")
    print("  - rotor_performance_analysis.png (6 main plots)")
    print("  - rotor_advanced_analysis.png (4 advanced plots)")
    print("  - rotor_performance_data.csv (detailed data table)")
    print("\nPlots include:")
    print("  ✓ Thrust vs Power")
    print("  ✓ Thrust vs Taper Ratio") 
    print("  ✓ Power vs Taper Ratio")
    print("  ✓ Chord Length Distribution")
    print("  ✓ Blade Twist Distribution")
    print("  ✓ Torque vs RPM (Blade Number Study)")
    print("  ✓ Power Loading vs Disc Loading")
    print("  ✓ Efficiency vs RPM")
    print("  ✓ Thrust vs Collective Pitch")
    print("  ✓ Power vs Number of Blades")
