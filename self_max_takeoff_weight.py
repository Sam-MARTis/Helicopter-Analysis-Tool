from heli import Helicopter, Rotor, MissionPlanner, Environment
import numpy as np





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
        root_chord=0.9,
        tip_chord=0.5,
        root_pitch=np.deg2rad(10.0),
        slope_pitch=np.deg2rad(3),
    )
    rotor2.set_rotor_parameters(
        number_of_blades=4,
        blade_mass=0,
        NACA_for_airfoil="0015",
        radius_of_rotors=7.0,
        root_cutout=0.4,
        root_chord=1.0,
        tip_chord=0.6,
        root_pitch=np.deg2rad(12.0),
        slope_pitch=np.deg2rad(4),
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
        dry_weight=5090,
        fuel_weight=1200,
        fuel_specific_energy_kj_kg=43000,
        reserve_fuel_fraction=0.1
    )

reset_mission_planner()
if __name__ == "__main__":
    print(mp.find_max_hover_weight(altitude=2000, stall_constraint=False, power_constraint=True))
    
    