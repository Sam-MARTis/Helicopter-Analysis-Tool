#!/usr/bin/env python3

import numpy as np
import matplotlib.pyplot as plt
from heli import Environment, Rotor

# Create environment
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

# Create rotor with NACA2412 airfoil data
rotor = Rotor(environment=env)
rotor.set_rotor_parameters(
    number_of_blades=3,
    blade_mass=75,
    NACA_for_airfoil="2412",  # Using NACA2412
    radius_of_rotors=6.6,
    root_cutout=0.3,
    root_chord=0.4,
    tip_chord=0.3,
    root_pitch=np.deg2rad(10.0),
    slope_pitch=np.deg2rad(3),
    filepath="naca2412.csv"  # Using the custom airfoil data
)

# Get density
altitude = 0.0
temperature = env.get_temperature(altitude)
density = env.get_density(temperature)

# Check if custom airfoil was loaded successfully
print(f"\nUsing custom airfoil: {not rotor.default_airfoil}")

# Test calculations with different angles of attack
print("\nTesting with different angles of attack:")
test_angles_deg = [-10, 0, 5, 10, 15, 20]
for angle_deg in test_angles_deg:
    angle_rad = np.deg2rad(angle_deg)
    if hasattr(rotor, 'cl_poly') and not rotor.default_airfoil:
        # Since get_cL expects internal parameters, we'll check the polynomial directly
        cl_val = rotor.cl_poly(angle_deg)  # Use degrees since that's what the CSV has
        cd_val = rotor.cd_poly(angle_deg)
        print(f"At {angle_deg:+3d}°: Cl = {cl_val:.4f}, Cd = {cd_val:.4f}")
    else:
        print(f"At {angle_deg:+3d}°: Using default airfoil model (no polynomial)")

# Test rotor performance
print("\nCalculating rotor performance:")
omega = 30  # rad/s
climb_velocity = 0  # m/s
perf = rotor.calculate_performance(climb_velocity, omega, density)
print(f"Thrust: {perf['thrust']:.2f} N")
print(f"Power: {perf['power']/1000:.2f} kW")
print(f"Torque: {perf['torque']:.2f} N-m")

# Test stall detection
print("\nTesting stall detection:")
stall_test_omegas = [20, 25, 30, 35]
for test_omega in stall_test_omegas:
    is_stalling = rotor.is_rotor_stalling(vertical_velocity=0, omega=test_omega)
    print(f"Omega = {test_omega} rad/s: {'STALLING' if is_stalling else 'OK'}")

# Plot Cl and Cd curves from the polynomial fits if using custom airfoil
if hasattr(rotor, 'cl_poly') and hasattr(rotor, 'cd_poly') and not rotor.default_airfoil:
    alpha_range = np.linspace(-20, 25, 100)
    cl_values = [rotor.cl_poly(alpha) for alpha in alpha_range]
    cd_values = [rotor.cd_poly(alpha) for alpha in alpha_range]

    plt.figure(figsize=(12, 6))

    plt.subplot(1, 2, 1)
    plt.plot(alpha_range, cl_values, 'b-', linewidth=2)
    if hasattr(rotor, 'alpha_stall'):
        plt.axvline(x=rotor.alpha_stall, color='r', linestyle='--', label=f'Stall angle: {rotor.alpha_stall:.2f}°')
    plt.grid(True, alpha=0.3)
    plt.xlabel('Angle of Attack (degrees)')
    plt.ylabel('Lift Coefficient (Cl)')
    plt.title('NACA 2412 - Lift Coefficient')
    plt.legend()

    plt.subplot(1, 2, 2)
    plt.plot(alpha_range, cd_values, 'g-', linewidth=2)
    plt.grid(True, alpha=0.3)
    plt.xlabel('Angle of Attack (degrees)')
    plt.ylabel('Drag Coefficient (Cd)')
    plt.title('NACA 2412 - Drag Coefficient')
    
    plt.tight_layout()
    plt.savefig('naca2412_curves.png', dpi=300)
    plt.show()
else:
    print("\nSkipping plot generation - custom airfoil not loaded successfully")

plt.tight_layout()
plt.savefig('naca2412_curves.png', dpi=300)
plt.show()
