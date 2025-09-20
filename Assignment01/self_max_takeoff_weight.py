from heli import Helicopter, Rotor, MissionPlanner, Environment
import numpy as np

import matplotlib.pyplot as plt



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
        NACA_for_airfoil="0012",
        radius_of_rotors=6.6,
        root_cutout=0.3,
        root_chord=1.0,
        tip_chord=0.9,
        root_pitch=np.deg2rad(15.0),
        slope_pitch=np.deg2rad(2),
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
        filepath="naca2412.csv"
    )

reset_rotor()


heli = Helicopter(environment=env)
heli.set_rotor(rotor1=rotor1, rotor2=rotor2)
def reset_heli():
    heli.set_fuselage_parameters(
        mass=5090,
        length=11.5,
        width=2.3,
        height=1.5,
        tail_mass=0,
        tail_length=6.5,
        tail_width=1.5,
        tail_height=0.5,
    )
    heli.set_engine_parameters(
        mass=0,
        max_power=1142*1000*2
        
    )
    heli.set_rotor_positions(
        main_rotor_position= 4,
        tail_rotor_position= 17
    )
    
reset_heli()

mp = MissionPlanner(heli)

def reset_mission_planner():
    mp.set_flight_parameters_programmatic(
        dry_weight=heli.get_total_mass(),
        fuel_weight=900,
        fuel_specific_energy_kj_kg=43000,
        reserve_fuel_fraction=0.1
    )

reset_mission_planner()


#Endurance plots

def plot_endurance_vs_weight():
    weights = [500 * i for i in range(3, 10)]
    enduraces = []
    for weight in weights:
        mp.dry_weight = weight
        endurance, _ = mp.find_available_hover_time(altitude=2000, dt=400)
        enduraces.append(round(endurance/60 , ndigits=1))

    plt.plot(weights, enduraces)
    plt.title('Endurance vs Weight')
    plt.xlabel('Weight (N)')
    plt.ylabel('Endurance (min)')
    plt.grid(True)
    plt.tight_layout()
    plt.savefig('endurance_vs_weight.png', dpi=300, bbox_inches='tight')
    plt.show()
    reset_rotor()
    reset_mission_planner()
    reset_heli()




def plot_burnrate_vs_weight():
    altitude = 2000
    temperature = env.get_temperature(altitude)
    pressure = env.get_pressure(temperature, altitude)
    density = env.get_density(temperature, pressure)
    
    weights = [500*i for i in range(3, 10)]
    vertical_velocities = [10*i for i in range(10)]
    omega_fixed = 30.0
    
    colors = plt.cm.viridis(np.linspace(0, 1, len(vertical_velocities)))
    
    data = {}
    for idx, v in enumerate(vertical_velocities):
        burnrate = []
        
        for weight in weights:
            burnrate.append(heli.find_power_needed(weight=weight, vertical_velocity=v, omega=omega_fixed, altitude=altitude))

        data[v] = {'burnrate': burnrate}

    plt.figure(figsize=(10, 6))
    for idx, v in enumerate(vertical_velocities):
        plt.plot(weights, data[v]['burnrate'], color=colors[idx], linewidth=2, label=f'v = {v} m/s')

    plt.title('Burnrate vs Weight')
    plt.xlabel('Weight (N)')
    plt.ylabel('Burnrate (kW)')
    plt.legend()
    plt.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.savefig('burnrate_vs_weight.png', dpi=300, bbox_inches='tight')
    plt.show()
    


if __name__ == "__main__":
    print(mp.find_max_hover_weight(altitude=2000, stall_constraint=False, power_constraint=True, iterations=100))
    # print(mp.helicopter.main_rotor.stall_maxWeight(0, 2000))
    plot_endurance_vs_weight()
    plot_burnrate_vs_weight()

    # print(mp.find_max_takeoff_weight(altitude=2000, stall_constraint=True, power_constraint=True))