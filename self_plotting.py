

import numpy as np
import matplotlib.pyplot as plt
from heli import Environment, Rotor, Helicopter, MissionPlanner
import pandas as pd














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


temperature_2000m = env.get_temperature(2000)
density_2000m = env.get_density(temperature_2000m)

rotor = Rotor(environment=env)
def reset_rotor():
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
    
reset_rotor()

def plot_quantities_vs_omega():
    altitude = 2000
    temperature = env.get_temperature(altitude)
    pressure = env.get_pressure(temperature, altitude)
    density = env.get_density(temperature, pressure)
    
    vertical_velocities = [0, 5, 10, 15, 20, 25, 30, 35, 40]  
    omegas = [4*i for i in range(1, 21)]  

    colors = plt.cm.viridis(np.linspace(0, 1, len(vertical_velocities)))
    
    # Calculate data once for all plots
    data = {}
    for idx, v in enumerate(vertical_velocities):
        thrusts = []
        powers = []
        torques = []
        
        for omega in omegas:
            try:
                values = rotor.calculate_performance(v, omega=omega, density=density)
                thrusts.append(values['thrust'])
                powers.append(values['power'] / 1000) 
                torques.append(values['torque'])
            except:
                thrusts.append(0)
                powers.append(0)
                torques.append(0)
        
        data[v] = {'thrust': thrusts, 'power': powers, 'torque': torques}
    
    
    plt.figure(figsize=(10, 6))
    for idx, v in enumerate(vertical_velocities):
        plt.plot(omegas, data[v]['thrust'], color=colors[idx], linewidth=2, label=f'v = {v} m/s')
    
    plt.title('Thrust vs Omega')
    plt.xlabel('Omega (rad/s)')
    plt.ylabel('Thrust (N)')
    plt.legend()
    plt.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.savefig('thrust_vs_omega.png', dpi=300, bbox_inches='tight')
    plt.show()
    
    
    plt.figure(figsize=(10, 6))
    for idx, v in enumerate(vertical_velocities):
        plt.plot(omegas, data[v]['power'], color=colors[idx], linewidth=2, label=f'v = {v} m/s')
    
    plt.title('Power vs Omega')
    plt.xlabel('Omega (rad/s)')
    plt.ylabel('Power (kW)')
    plt.legend()
    plt.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.savefig('power_vs_omega.png', dpi=300, bbox_inches='tight')
    plt.show()
    
    
    plt.figure(figsize=(10, 6))
    for idx, v in enumerate(vertical_velocities):
        plt.plot(omegas, data[v]['torque'], color=colors[idx], linewidth=2, label=f'v = {v} m/s')
    
    plt.title('Torque vs Omega')
    plt.xlabel('Omega (rad/s)')
    plt.ylabel('Torque (Nm)')
    plt.legend()
    plt.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.savefig('torque_vs_omega.png', dpi=300, bbox_inches='tight')
    plt.show()

plot_quantities_vs_omega()


def plot_quantities_vs_taper_ratio():
    altitude = 2000
    temperature = env.get_temperature(altitude)
    pressure = env.get_pressure(temperature, altitude)
    density = env.get_density(temperature, pressure)
    
    taper_ratios = [0.1*i for i in range(1, 11)]
    thrust_levels = [5000, 10000, 15000, 20000, 25000, 30000, 35000, 40000, 45000]
    
    colors = plt.cm.viridis(np.linspace(0, 1, len(thrust_levels)))
    
    data = {}
    for idx, thrust_target in enumerate(thrust_levels):
        powers = []
        torques = []
        omegas_found = []
        
        for taper_ratio in taper_ratios:
            rotor.tip_chord = rotor.root_chord * taper_ratio
            try:
                omega = rotor.find_omega_needed_uncoupled(vertical_velocity=0, altitude=altitude, thrust_needed=thrust_target, initial_guess=30.0)
                values = rotor.calculate_performance(vertical_velocity=0, omega=omega, density=density)
                powers.append(values['power'] / 1000)
                torques.append(values['torque'])
                omegas_found.append(omega)
            except:
                powers.append(0)
                torques.append(0)
                omegas_found.append(0)
        
        data[thrust_target] = {'power': powers, 'torque': torques, 'omega': omegas_found}
    
    plt.figure(figsize=(10, 6))
    for idx, thrust_target in enumerate(thrust_levels):
        plt.plot(taper_ratios, data[thrust_target]['omega'], color=colors[idx], linewidth=2, label=f'T = {thrust_target/1000:.0f} kN')
    
    plt.title('Omega vs Taper Ratio')
    plt.xlabel('Taper Ratio')
    plt.ylabel('Omega (rad/s)')
    plt.legend()
    plt.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.savefig('omega_vs_taper_ratio.png', dpi=300, bbox_inches='tight')
    plt.show()
    
    plt.figure(figsize=(10, 6))
    for idx, thrust_target in enumerate(thrust_levels):
        plt.plot(taper_ratios, data[thrust_target]['power'], color=colors[idx], linewidth=2, label=f'T = {thrust_target/1000:.0f} kN')
    
    plt.title('Power vs Taper Ratio')
    plt.xlabel('Taper Ratio')
    plt.ylabel('Power (kW)')
    plt.legend()
    plt.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.savefig('power_vs_taper_ratio.png', dpi=300, bbox_inches='tight')
    plt.show()
    
    plt.figure(figsize=(10, 6))
    for idx, thrust_target in enumerate(thrust_levels):
        plt.plot(taper_ratios, data[thrust_target]['torque'], color=colors[idx], linewidth=2, label=f'T = {thrust_target/1000:.0f} kN')
    
    plt.title('Torque vs Taper Ratio')
    plt.xlabel('Taper Ratio')
    plt.ylabel('Torque (Nm)')
    plt.legend()
    plt.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.savefig('torque_vs_taper_ratio.png', dpi=300, bbox_inches='tight')
    plt.show()

plot_quantities_vs_taper_ratio()
