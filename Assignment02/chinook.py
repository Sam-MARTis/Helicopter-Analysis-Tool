from heli import Helicopter, Rotor, MissionPlanner, Environment
import numpy as np

import matplotlib.pyplot as plt
import multiprocessing
import time
from tqdm import tqdm

NUMBER_OF_MAIN_ROTORS = 2
NUMBER_OF_ENGINES = 2
MASS_OF_CHINOOK = 7000
FUEL_WEIGHT = 1500


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

rotor1 = Rotor(environment=env)
rotor2 = Rotor(environment=env)
def reset_rotor():
    rotor1.set_rotor_parameters(
        number_of_blades=3,
        blade_mass=0,
        NACA_for_airfoil="2412",
        radius_of_rotors=16,
        root_cutout=0.4,
        root_chord=0.9,
        tip_chord=0.8,
        root_pitch=np.deg2rad(0.0),
        slope_pitch=np.deg2rad(1),
        filepath="naca2412.csv"
    )
    rotor2.set_rotor_parameters( 
        number_of_blades=4,
        blade_mass=0,
        NACA_for_airfoil="0015",
        radius_of_rotors=1.0,
        root_cutout=0.4,
        root_chord=1.0,
        tip_chord=0.6,
        root_pitch=np.deg2rad(10.0),
        slope_pitch=np.deg2rad(2),
        # filepath="naca2412.csv"
    )

reset_rotor()


heli = Helicopter(environment=env)
heli.set_rotor(rotor1=rotor1, rotor2=rotor2)
def reset_heli():
    heli.set_fuselage_parameters(
        mass=MASS_OF_CHINOOK / NUMBER_OF_MAIN_ROTORS,
        length=18,
        width=2.3,
        height=1.5,
        tail_mass=0,
        tail_length=1,
        tail_width=1.5,
        tail_height=0.5,
    )
    heli.set_engine_parameters(
        mass=0,
        max_power=1500*1000*NUMBER_OF_ENGINES / NUMBER_OF_MAIN_ROTORS

    )
    heli.set_rotor_positions(
        main_rotor_position= 1,
        tail_rotor_position= 999999 # So as to not contribute in power calculations
    )
    
reset_heli()

mp = MissionPlanner(heli)

def reset_mission_planner():
    mp.set_flight_parameters_programmatic(
        dry_weight=heli.get_total_mass(),
        fuel_weight=FUEL_WEIGHT / NUMBER_OF_MAIN_ROTORS,
        fuel_specific_energy_kj_kg=43000,
        reserve_fuel_fraction=0.1
    )

reset_mission_planner()


#Endurance plots
def copy_env(env:Environment):
    new_env = Environment()
    new_env.set_atmosphere_parameters(
        temperature_sea_level=env.get_temperature(0),
        Reynolds_number=env.Reynolds_number,
        altitude=env.altitude,
        pressure_sea_level=env.pressure_sea_level,
        Lapse_rate_troposphere=env.Lapse_rate_troposphere,
        wind_velocity=env.wind_velocity,
        ISA_OFFSET=env.ISA_OFFSET
    )
    return new_env

def copy_rotor(rotor:Rotor):
    new_rotor = Rotor(environment=copy_env(rotor.environment))
    new_rotor.set_rotor_parameters(
        number_of_blades=rotor.number_of_blades,
        blade_mass=rotor.blade_mass,
        NACA_for_airfoil=rotor.NACA_for_airfoil,
        radius_of_rotors=rotor.radius_of_rotors,
        root_cutout=rotor.root_cutout,
        root_chord=rotor.root_chord,
        tip_chord=rotor.tip_chord,
        root_pitch=rotor.root_pitch,
        slope_pitch=rotor.slope_pitch
    )
    return new_rotor

def copy_heli(heli:Helicopter):
    new_heli = Helicopter(environment=copy_env(heli.environment))
    new_heli.set_rotor(rotor1=copy_rotor(heli.main_rotor), rotor2=copy_rotor(heli.tail_rotor))
    new_heli.set_fuselage_parameters(
        mass=heli.fuselage_mass,
        length=heli.fuselage_length,
        width=heli.fuselage_width,
        height=heli.fuselage_height,
        tail_mass=heli.tail_mass,
        tail_length=heli.tail_length,
        tail_width=heli.tail_width,
        tail_height=heli.tail_height
    )
    new_heli.set_engine_parameters(
        mass=heli.engine_mass,
        max_power=heli.engine_max_power
    )
    new_heli.set_rotor_positions(
        main_rotor_position=heli.main_rotor_position,
        tail_rotor_position=heli.tail_rotor_position
    )
    return new_heli


def copy_mp(mp:MissionPlanner):
    new_mp = MissionPlanner(copy_heli(mp.helicopter))
    new_mp.set_flight_parameters_programmatic(
        dry_weight=mp.dry_weight,
        fuel_weight=mp.fuel_weight,
        fuel_specific_energy_kj_kg=mp.fuel_specific_energy*0.001,
        reserve_fuel_fraction=0.1
    )
    return new_mp

def calculate_endurance(params):
    """Helper function for multiprocessing - calculates endurance for a single weight"""
    weight, altitude, dt = params
    
      
    local_env = copy_env(env)
    
    
    local_rotor1 = copy_rotor(rotor1)
    local_rotor2 = copy_rotor(rotor2)
    
    
    local_rotor1.environment = local_env
    local_rotor2.environment = local_env

    local_heli = copy_heli(heli)
    local_heli.set_rotor(rotor1=local_rotor1, rotor2=local_rotor2)
    local_heli.environment = local_env
    
    
    # local_mp = MissionPlanner(local_heli)
    local_mp = copy_mp(mp)
    local_mp.helicopter = local_heli

    local_mp.set_flight_parameters_programmatic(
        dry_weight=weight,
        fuel_weight=local_mp.fuel_weight,
        fuel_specific_energy_kj_kg=local_mp.fuel_specific_energy*0.001,
        reserve_fuel_fraction=0.1
    )
    
    
    endurance, success = local_mp.find_available_hover_time(altitude=altitude, dt=dt)
    return weight, endurance/60, success  

def plot_endurance_vs_weight():
    print("Starting endurance calculation with multiprocessing...")
    weights = [500 * i for i in range(3, 10)]
    altitude = 2000
    dt = 400  
    
    
    params = [(weight, altitude, dt) for weight in weights]
    
    
    num_processes = min(multiprocessing.cpu_count(), len(params))
    print(f"Using {num_processes} CPU cores for parallel processing")
    
    start_time = time.time()
    with multiprocessing.Pool(processes=num_processes) as pool:
        results = list(tqdm(pool.imap(calculate_endurance, params), total=len(params)))
    
    end_time = time.time()
    print(f"Calculations completed in {end_time - start_time:.2f} seconds")
    
    
    results.sort()  
    
    # Extract results for plotting - including all calculations
    weights_result = [w for w, e, s in results]  
    endurances = [round(e, 1) for w, e, s in results]
    
    plt.figure(figsize=(10, 6))
    plt.plot(weights_result, endurances, marker='o')
    plt.title('Endurance vs Weight (Multiprocessing)')
    plt.xlabel('Weight (N)')
    plt.ylabel('Endurance (min)')
    plt.grid(True)
    plt.tight_layout()
    plt.savefig('endurance_vs_weight.png', dpi=300, bbox_inches='tight')
    plt.show()
    reset_rotor()
    reset_mission_planner()
    reset_heli()


def calculate_burnrate(params):
    #Used multiprocessing to speed this up. Takes too long otherwise
    weight, vertical_velocity, altitude = params
    
    local_env = copy_env(env)
    
    
    local_rotor1 = copy_rotor(rotor1)
    local_rotor2 = copy_rotor(rotor2)
    
    
    local_rotor1.environment = local_env
    local_rotor2.environment = local_env

    local_heli = copy_heli(heli)
    local_heli.set_rotor(rotor1=local_rotor1, rotor2=local_rotor2)
    local_heli.environment = local_env

    power = local_heli.find_power_needed(weight=weight, vertical_velocity=vertical_velocity, altitude=altitude)
    return (weight, vertical_velocity, power)

def plot_burnrate_vs_weight():
    import multiprocessing as mpr
    from tqdm import tqdm  
    
    print("Starting burnrate calculation with multiprocessing...")
    altitude = 2000
    temperature = env.get_temperature(altitude)
    pressure = env.get_pressure(temperature, altitude)
    density = env.get_density(temperature, pressure)
    
    weights = [500*i for i in range(3, 10)]
    vertical_velocities = [10*i for i in range(3)]
    
    
    params = []
    for weight in weights:
        for v in vertical_velocities:
            params.append((weight, v, altitude))


    num_processes = min(mpr.cpu_count(), len(params))
    print(f"Using {num_processes} CPU cores for parallel processing")

    with mpr.Pool(processes=num_processes) as pool:
        results = list(tqdm(pool.imap(calculate_burnrate, params), total=len(params)))
    
    
    data = {v: {'burnrate': [], 'weights': []} for v in vertical_velocities}
    for weight, v, power in results:
        data[v]['burnrate'].append(60*power/(mp.fuel_specific_energy))  
        data[v]['weights'].append(weight)
    
    
    for v in vertical_velocities:
        weights_sorted = sorted(zip(data[v]['weights'], data[v]['burnrate']))
        data[v]['weights'] = [w for w, _ in weights_sorted]
        data[v]['burnrate'] = [br for _, br in weights_sorted]
    
    
    colors = plt.cm.viridis(np.linspace(0, 1, len(vertical_velocities)))
    plt.figure(figsize=(10, 6))
    
    for idx, v in enumerate(vertical_velocities):
        plt.plot(data[v]['weights'], data[v]['burnrate'], 
                 color=colors[idx], marker='o', label=f'v = {v} m/s')

    plt.title('Burnrate vs Weight (Multiprocessing)')
    plt.xlabel('Weight (N)')
    plt.ylabel('Burnrate (kg/min)')
    plt.legend()
    plt.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.savefig('burnrate_vs_weight.png', dpi=300, bbox_inches='tight')
    plt.show()
    reset_rotor()




def plot_quantities_vs_omega():
    altitude = 2000
    temperature = env.get_temperature(altitude)
    pressure = env.get_pressure(temperature, altitude)
    density = env.get_density(temperature, pressure)
    
    vertical_velocities = [0, 5, 10, 15, 20, 25, 30, 35, 40]  
    omegas = [4*i for i in range(5, 21)]  

    colors = plt.cm.viridis(np.linspace(0, 1, len(vertical_velocities)))
    
    
    data = {}
    for idx, v in enumerate(vertical_velocities):
        thrusts = []
        powers = []
        torques = []
        
        for omega in omegas:
            try:
                values = rotor1.calculate_performance(v, omega=omega, density=density)
                thrusts.append(values['thrust'])
                powers.append( values['power'] / 1000) 
                torques.append(values['torque'])
            except:
                thrusts.append(0)
                powers.append(0)
                torques.append(0)
        
        data[v] = {'thrust': thrusts, 'power': powers, 'torque': torques}
    
    
    plt.figure(figsize=(10, 6))
    for idx, v in enumerate(vertical_velocities):
        plt.plot(omegas, data[v]['thrust'], color=colors[idx], linewidth=2, label=f'v = {v} m/s')

    plt.title('Thrust vs Omega for single rotor')
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

    plt.title('Power vs Omega for single rotor')
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

    plt.title('Torque vs Omega for single rotor')
    plt.xlabel('Omega (rad/s)')
    plt.ylabel('Torque (Nm)')
    plt.legend()
    plt.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.savefig('torque_vs_omega.png', dpi=300, bbox_inches='tight')
    reset_rotor()
    plt.show()
    
    
    plt.figure(figsize=(10, 6))
    for idx, v in enumerate(vertical_velocities):
        plt.plot(data[v]['thrust'], data[v]['power'], color=colors[idx], linewidth=2, label=f'v = {v} m/s')

    plt.title('Power vs Thrust for single rotor')
    plt.xlabel('Thrust (N)')
    plt.ylabel('Power (kW)')
    plt.legend()
    plt.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.savefig('power_vs_thrust.png', dpi=300, bbox_inches='tight')
    reset_rotor()
    plt.show()
    
    



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
            rotor1.tip_chord = rotor1.root_chord * taper_ratio
            try:
                omega = rotor1.find_omega_needed_uncoupled(vertical_velocity=0, altitude=altitude, thrust_needed=thrust_target, initial_guess=30.0)
                values = rotor1.calculate_performance(climb_velocity=0, omega=omega, density=density)
                powers.append(values['power'] / 1000)
                torques.append(values['torque'])
                omegas_found.append(omega)
            except Exception as e:
                print("There was an error:", e)
                powers.append(0)
                torques.append(0)
                omegas_found.append(0)
        
        data[thrust_target] = {'power': powers, 'torque': torques, 'omega': omegas_found}
    
    plt.figure(figsize=(10, 6))
    for idx, thrust_target in enumerate(thrust_levels):
        plt.plot(taper_ratios, data[thrust_target]['omega'], color=colors[idx], linewidth=2, label=f'T = {thrust_target/1000:.0f} kN')

    plt.title('Omega vs Taper Ratio (single rotor)')
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

    plt.title('Power vs Taper Ratio (single rotor)')
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

    plt.title('Torque vs Taper Ratio (single rotor)')
    plt.xlabel('Taper Ratio')
    plt.ylabel('Torque (Nm)')
    plt.legend()
    plt.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.savefig('torque_vs_taper_ratio.png', dpi=300, bbox_inches='tight')
    reset_rotor()
    plt.show()






def plot_quantities_vs_twist():
    altitude = 2000
    temperature = env.get_temperature(altitude)
    pressure = env.get_pressure(temperature, altitude)
    density = env.get_density(temperature, pressure)
    
    twist_values = [ np.deg2rad(0.1*i) for i in range(0, 21)]
    vertical_velocities = [5*i for i in range(5)]
    omega_fixed = 30.0
    
    colors = plt.cm.viridis(np.linspace(0, 1, len(vertical_velocities)))
    
    data = {}
    for idx, v in enumerate(vertical_velocities):
        thrusts = []
        powers = []
        torques = []
        
        for twist in twist_values:
            rotor1.root_pitch = np.deg2rad(5.0)
            rotor1.slope_pitch = twist
            try:
                values = rotor1.calculate_performance(v, omega=omega_fixed, density=density)
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
        plt.plot(twist_values, data[v]['thrust'], color=colors[idx], linewidth=2, label=f'v = {v} m/s')

    plt.title('Thrust vs Twist, ' + str(omega_fixed) + ' rad/s')
    plt.xlabel('Twist (slope_pitch) (rad/m)')
    plt.ylabel('Thrust (N)')
    plt.legend()
    plt.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.savefig('thrust_vs_twist.png', dpi=300, bbox_inches='tight')
    plt.show()
    
    plt.figure(figsize=(10, 6))
    for idx, v in enumerate(vertical_velocities):
        plt.plot(twist_values, data[v]['power'], color=colors[idx], linewidth=2, label=f'v = {v} m/s')

    plt.title('Power vs Twist, ' + str(omega_fixed) + ' rad/s')
    plt.xlabel('Twist (slope_pitch) (rad/m)')
    plt.ylabel('Power (kW)')
    plt.legend()
    plt.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.savefig('power_vs_twist.png', dpi=300, bbox_inches='tight')
    plt.show()
    
    plt.figure(figsize=(10, 6))
    for idx, v in enumerate(vertical_velocities):
        plt.plot(twist_values, data[v]['torque'], color=colors[idx], linewidth=2, label=f'v = {v} m/s')

    plt.title('Torque vs Twist, ' + str(omega_fixed) + ' rad/s')
    plt.xlabel('Twist (slope_pitch) (rad/m)')
    plt.ylabel('Torque (Nm)')
    plt.legend()
    plt.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.savefig('torque_vs_twist.png', dpi=300, bbox_inches='tight')
    plt.show()



def plot_quantities_vs_pitch():
    altitude = 2000
    temperature = env.get_temperature(altitude)
    pressure = env.get_pressure(temperature, altitude)
    density = env.get_density(temperature, pressure)
    
    pitch_values = [np.deg2rad(i) for i in range(0, 31)]
    vertical_velocities = [5*i for i in range(5)]
    omega_fixed = 30.0
    
    colors = plt.cm.viridis(np.linspace(0, 1, len(vertical_velocities)))
    
    data = {}
    for idx, v in enumerate(vertical_velocities):
        thrusts = []
        powers = []
        torques = []
        
        for pitch in pitch_values:
            rotor2.root_pitch = pitch
            rotor2.slope_pitch = np.deg2rad(1.0)
            try:
                values = rotor2.calculate_performance(v, omega=omega_fixed, density=density)
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
        plt.plot(np.rad2deg(pitch_values), data[v]['thrust'], color=colors[idx], linewidth=2, label=f'v = {v} m/s')

    plt.title('Thrust vs Pitch, ' + str(omega_fixed) + ' rad/s')
    plt.xlabel('Root Pitch (degrees)')
    plt.ylabel('Thrust (N)')
    plt.legend()
    plt.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.savefig('thrust_vs_pitch.png', dpi=300, bbox_inches='tight')
    plt.show()
    
    plt.figure(figsize=(10, 6))
    for idx, v in enumerate(vertical_velocities):
        plt.plot(np.rad2deg(pitch_values), data[v]['power'], color=colors[idx], linewidth=2, label=f'v = {v} m/s')

    plt.title('Power vs Pitch, ' + str(omega_fixed) + ' rad/s')
    plt.xlabel('Root Pitch (degrees)')
    plt.ylabel('Power (kW)')
    plt.legend()
    plt.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.savefig('power_vs_pitch.png', dpi=300, bbox_inches='tight')
    plt.show()
    
    plt.figure(figsize=(10, 6))
    for idx, v in enumerate(vertical_velocities):
        plt.plot(np.rad2deg(pitch_values), data[v]['torque'], color=colors[idx], linewidth=2, label=f'v = {v} m/s')

    plt.title('Torque vs Pitch, ' + str(omega_fixed) + ' rad/s')
    plt.xlabel('Root Pitch (degrees)')
    plt.ylabel('Torque (Nm)')
    plt.legend()
    plt.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.savefig('torque_vs_pitch.png', dpi=300, bbox_inches='tight')
    reset_rotor()
    plt.show()

if __name__ == "__main__":
    
    # Printing graphs
    
    
    # plot_quantities_vs_omega()
    # plot_quantities_vs_taper_ratio()
    # plot_quantities_vs_twist()
    # plot_burnrate_vs_weight()
    # plot_endurance_vs_weight()
    # print("\n\n\n\n")
    # print(f"Maximumum take off weight: {(mp.find_max_hover_weight(altitude=2000, stall_constraint=False, power_constraint=True, iterations=100)[0]) * NUMBER_OF_MAIN_ROTORS}\n")
    plot_quantities_vs_pitch()
    
    
    # plot_endurance_vs_weight()
    
    
    
    # print(mp.find_max_hover_weight(altitude=2000, stall_constraint=False, power_constraint=True, iterations=100) * 2)
    # print(mp.helicopter.main_rotor.stall_maxWeight(0, 2000))
    # plot_endurance_vs_weight()
    # plot_burnrate_vs_weight()

    # print(mp.find_max_takeoff_weight(altitude=2000, stall_constraint=True, power_constraint=True))