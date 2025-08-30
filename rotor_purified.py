# -*- coding: utf-8 -*-
"""rotor_purified.py

Pure function implementation of rotor aerodynamics calculations.
All functions are rewritten to avoid global variable dependencies.
"""

import numpy as np
import matplotlib.pyplot as plt
import pandas as pd

def get_fuselage_parameters():
    fuselage = {}
    fuselage["mass"] = float(input("Please enter the mass of the fuselage:"))
    fuselage["length"] = float(input("Please enter the length of the fuselage:"))
    fuselage["width"] = float(input("Please enter the width of the fuselage:"))
    fuselage["height"] = float(input("Please enter the height of the fuselage:"))
    fuselage["CL"] = float(input("Please enter the CL of the fuselage:"))
    fuselage["CD"] = float(input("Please enter the CD of the fuselage:"))
    return fuselage

def get_engine_parameters():
    engine = {}
    engine["mass"] = float(input("Please enter the mass of the engine:"))
    engine["BSFC"] = float(input("Please enter the BSFC of the engine:"))
    engine["shaft power conversion efficiency"] = float(input("Please enter the shaft power conversion efficiency:"))
    engine["fractional power tail rotor"] = float(input("Please enter the fractional power of the tail rotor:"))
    return engine

def get_rotor_parameters():
    rotor = {}
    rotor["number_of_blades"] = int(input("Please enter the number of rotors: "))
    rotor["rotor_omega"] = float(input("Please enter the angular velocity of the rotor (rad/s): "))
    rotor["blade_mass"] = float(input("Please enter the mass of the rotors (kg): "))
    rotor["NACA_for_airfoil"] = input("Please enter the NACA number of the airfoil: ")
    rotor["radius_of_rotors"] = float(input("Please enter the radius of the rotors (m): "))
    rotor["root_cutout"] = float(input("Please enter the root cutout of the rotors (m): "))
    rotor["angle_of_attack"] = float(input("Please enter the angle of attack (degrees): "))
    rotor["root_chord"] = float(input("Please enter the root chord (m): "))
    rotor["tip_chord"] = float(input("Please enter the tip chord (m): "))
    rotor["root_pitch"] = float(input("Please enter the root pitch (degrees): "))
    rotor["slope_pitch"] = float(input("Please enter the slope of the pitch: "))
    rotor["climb_velocity"] = float(input("Please enter the desired climb velocity (m/s): "))
    return rotor

def get_atmosphere_parameters():
    atmosphere = {}
    atmosphere["temperature_sea_level"] = float(input("Please enter the sea level temperature (K): "))
    atmosphere["Reynolds_number"] = float(input("Please enter the Reynolds number: "))
    atmosphere["altitude"] = float(input("Please enter the altitude (m): "))
    atmosphere["pressure_sea_level"] = float(input("Please enter the pressure at sea level (Pa): "))
    atmosphere["Lapse_rate_troposphere"] = float(input("Please enter the lapse rate of the troposphere (K/m): "))
    atmosphere["Specific_gas_constant"] = 287.053
    atmosphere["gravitational_acceleration"] = 9.80665
    atmosphere["specific_heat_constant"] = 1.4
    atmosphere["wind_velocity"] = float(input("Please enter the wind velocity (m/s): "))
    atmosphere["ISA_OFFSET"] = float(input("Please enter the ISA offset (K): "))
    return atmosphere

def Temp(temperature_sea_level, lapse_rate_troposphere, altitude, ISA_OFFSET):
    T_new = temperature_sea_level - lapse_rate_troposphere * altitude + ISA_OFFSET
    return T_new

def Pressure(pressure_sea_level, T_new, temperature_sea_level, lapse_rate_troposphere, gravitational_acceleration, specific_gas_constant):
    return pressure_sea_level * ((T_new / temperature_sea_level) ** (gravitational_acceleration / (lapse_rate_troposphere * specific_gas_constant)))

def speed_of_sound(temperature):
    return np.sqrt(1.4 * 8.314 * temperature)

def mach_number(velocity, sound_speed):
    return velocity / sound_speed

def density(temperature, pressure):
    return pressure / (287 * temperature)

def Cl_Cd_Cm(naca_number, alpha):
    cl = 5.75
    cd = 0.013
    cm = 0.03
    return cl, cd, cm

def get_chord_length(x, radius_of_rotors, root_cutout, root_chord, tip_chord):
    chord_length_x = (root_chord - ((root_chord - tip_chord) / (radius_of_rotors - root_cutout)) * x)
    return chord_length_x

def pitch(root_pitch, slope_pitch, x):
    local_pitch = root_pitch + slope_pitch * x
    return np.deg2rad(local_pitch)

def elemental_inflow_ratio(r_distance, number_of_blades, radius_of_rotors, root_cutout, root_chord, tip_chord, 
                          root_pitch, slope_pitch, rotor_omega, climb_velocity, a=5.75, max_iter=100, tol=1e-6):
    lambda_c = climb_velocity / (rotor_omega * radius_of_rotors)
    F = 1.1
    lambda_inflow = 0.2

    for i in range(max_iter):
        theta = pitch(root_pitch, slope_pitch, r_distance)
        sigma = number_of_blades * get_chord_length(r_distance, radius_of_rotors, root_cutout, root_chord, tip_chord) / (np.pi * radius_of_rotors)
        
        new_lambda_inflow = (
            np.sqrt(((sigma * a) / (16 * F) - (lambda_c / 2))**2
                    + (sigma * a) / (8 * F) * theta * (r_distance / radius_of_rotors))
            - ((sigma * a) / (16 * F) - (lambda_c / 2))
        )

        f = (number_of_blades / 2) * ((1 - r_distance / radius_of_rotors) / lambda_inflow)
        new_F = (2 / np.pi) * np.arccos(np.exp(-f))

        if abs(new_lambda_inflow - lambda_inflow) < tol and abs(new_F - F) < tol:
            return new_lambda_inflow, new_F

        lambda_inflow, F = new_lambda_inflow, new_F

    return lambda_inflow, F

def phi_r(r_distance, number_of_blades, radius_of_rotors, root_cutout, root_chord, tip_chord, 
          root_pitch, slope_pitch, rotor_omega, climb_velocity):
    lambda_inflow, _ = elemental_inflow_ratio(r_distance, number_of_blades, radius_of_rotors, root_cutout, 
                                            root_chord, tip_chord, root_pitch, slope_pitch, rotor_omega, climb_velocity)
    c = np.arctan((lambda_inflow) * radius_of_rotors / r_distance)
    return c

def alpha_effective_r(r_distance, number_of_blades, radius_of_rotors, root_cutout, root_chord, tip_chord, 
                     root_pitch, slope_pitch, rotor_omega, climb_velocity):
    phi = phi_r(r_distance, number_of_blades, radius_of_rotors, root_cutout, root_chord, tip_chord, 
                root_pitch, slope_pitch, rotor_omega, climb_velocity)
    theta = pitch(root_pitch, slope_pitch, r_distance)
    d = theta - phi
    return d

def cL(r_distance, number_of_blades, radius_of_rotors, root_cutout, root_chord, tip_chord, 
       root_pitch, slope_pitch, rotor_omega, climb_velocity, a=5.75):
    alpha_eff = alpha_effective_r(r_distance, number_of_blades, radius_of_rotors, root_cutout, 
                                 root_chord, tip_chord, root_pitch, slope_pitch, rotor_omega, climb_velocity)
    return a * alpha_eff

def cD(r_distance, number_of_blades, radius_of_rotors, root_cutout, root_chord, tip_chord, 
       root_pitch, slope_pitch, rotor_omega, climb_velocity, CD0=0.005, k=0.1, a=5.75):
    alpha_eff = alpha_effective_r(r_distance, number_of_blades, radius_of_rotors, root_cutout, 
                                 root_chord, tip_chord, root_pitch, slope_pitch, rotor_omega, climb_velocity)
    return CD0 + k * ((a * alpha_eff) ** 2)

def elemental_thrust(r_distance, dr, number_of_blades, radius_of_rotors, root_cutout, root_chord, tip_chord, 
                    root_pitch, slope_pitch, rotor_omega, climb_velocity, density=1.38):
    lambda_inflow, _ = elemental_inflow_ratio(r_distance, number_of_blades, radius_of_rotors, root_cutout, 
                                            root_chord, tip_chord, root_pitch, slope_pitch, rotor_omega, climb_velocity)
    dT = 4 * np.pi * density * r_distance * (lambda_inflow * rotor_omega * radius_of_rotors) * ((lambda_inflow * rotor_omega * radius_of_rotors) - climb_velocity) * dr
    return dT

def elemental_torque(r_distance, dr, number_of_blades, radius_of_rotors, root_cutout, root_chord, tip_chord, 
                    root_pitch, slope_pitch, rotor_omega, climb_velocity):
    dT = elemental_thrust(r_distance, dr, number_of_blades, radius_of_rotors, root_cutout, root_chord, tip_chord, 
                         root_pitch, slope_pitch, rotor_omega, climb_velocity)
    return dT * r_distance

def total_thrust(n_divisions, number_of_blades, radius_of_rotors, root_cutout, root_chord, tip_chord, 
                root_pitch, slope_pitch, rotor_omega, climb_velocity):
    r = np.linspace(root_cutout, radius_of_rotors, n_divisions)
    delta_r = r[1] - r[0]
    store_dT = []
    for i in range(1, n_divisions - 1):
        store_dT.append(elemental_thrust(r[i], delta_r, number_of_blades, radius_of_rotors, root_cutout, 
                                       root_chord, tip_chord, root_pitch, slope_pitch, rotor_omega, climb_velocity))
    net_thrust = number_of_blades * np.sum(store_dT)
    return net_thrust

def total_torque(n_divisions, number_of_blades, radius_of_rotors, root_cutout, root_chord, tip_chord, 
                root_pitch, slope_pitch, rotor_omega, climb_velocity, rho=1.2, cm=0.1):
    r = np.linspace(root_cutout, radius_of_rotors, n_divisions)
    delta_r = r[1] - r[0]
    store_dT = []
    for i in range(1, n_divisions - 1):
        torque_element = elemental_torque(r[i], delta_r, number_of_blades, radius_of_rotors, root_cutout, 
                                        root_chord, tip_chord, root_pitch, slope_pitch, rotor_omega, climb_velocity)
        additional_torque = 0.5 * rho * cm * (rotor_omega * r[i]) ** 2
        store_dT.append(torque_element * r[i] + additional_torque)
    net_moment = number_of_blades * np.sum(store_dT)
    return net_moment

def total_power(n_divisions, number_of_blades, radius_of_rotors, root_cutout, root_chord, tip_chord, 
               root_pitch, slope_pitch, rotor_omega, climb_velocity):
    r = np.linspace(root_cutout, radius_of_rotors, n_divisions)
    delta_r = r[1] - r[0]
    store_dT = []
    for i in range(1, n_divisions - 1):
        dT = elemental_thrust(r[i], delta_r, number_of_blades, radius_of_rotors, root_cutout, 
                            root_chord, tip_chord, root_pitch, slope_pitch, rotor_omega, climb_velocity)
        lambda_inflow, _ = elemental_inflow_ratio(r[i], number_of_blades, radius_of_rotors, root_cutout, 
                                                root_chord, tip_chord, root_pitch, slope_pitch, rotor_omega, climb_velocity)
        store_dT.append(dT * lambda_inflow * rotor_omega * r[i])
    net_power = number_of_blades * np.sum(store_dT)
    return net_power

def values(n_divisions, number_of_blades, radius_of_rotors, root_cutout, root_chord, tip_chord, 
          root_pitch, slope_pitch, rotor_omega, climb_velocity, rho=1.2):
    r = np.linspace(root_cutout, radius_of_rotors, n_divisions)
    delta_r = r[1] - r[0]
    
    dL_list = []
    dD_list = []
    dT_list = []
    dFx_list = []
    dP_list = []
    dtorque_list = []
    
    for i in range(len(r)):
        chord = get_chord_length(r[i], radius_of_rotors, root_cutout, root_chord, tip_chord)
        cl = cL(r[i], number_of_blades, radius_of_rotors, root_cutout, root_chord, tip_chord, 
                root_pitch, slope_pitch, rotor_omega, climb_velocity)
        cd = cD(r[i], number_of_blades, radius_of_rotors, root_cutout, root_chord, tip_chord, 
                root_pitch, slope_pitch, rotor_omega, climb_velocity)
        lambda_inflow, _ = elemental_inflow_ratio(r[i], number_of_blades, radius_of_rotors, root_cutout, 
                                                root_chord, tip_chord, root_pitch, slope_pitch, rotor_omega, climb_velocity)
        phi = phi_r(r[i], number_of_blades, radius_of_rotors, root_cutout, root_chord, tip_chord, 
                   root_pitch, slope_pitch, rotor_omega, climb_velocity)
        
        velocity_squared = (rotor_omega * r[i]) ** 2 + (lambda_inflow * rotor_omega * radius_of_rotors) ** 2
        
        dL = 0.5 * rho * chord * cl * velocity_squared
        dD = 0.5 * rho * chord * cd * velocity_squared
        dT = (dL * np.cos(phi) - dD * np.sin(phi)) * delta_r
        dFx = (dD * np.cos(phi) + dL * np.sin(phi)) * delta_r
        dP = rotor_omega * r[i] * dFx
        dtorque = r[i] * dFx
        
        dL_list.append(dL)
        dD_list.append(dD)
        dT_list.append(dT)
        dFx_list.append(dFx)
        dP_list.append(dP)
        dtorque_list.append(dtorque)
    
    Thrust = np.sum(dT_list) * number_of_blades
    Torque = np.sum(dtorque_list) * number_of_blades
    Power = np.sum(dP_list) * number_of_blades
    Thrust_x = np.sum(dFx_list) * number_of_blades
    lift = np.sum(dL_list)
    drag = np.sum(dD_list)
    
    return Thrust, Torque, Power, Thrust_x, lift, drag
