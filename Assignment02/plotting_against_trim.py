from Rotor import Rotor, trimSolve, get_Thrust_Total, get_Parasitic_Drag, deg_to_rad, print_trim_values_and_state
import numpy as np
import matplotlib.pyplot as plt




theta0_min = -10
theta0_max = 10
theta1s_min = -10
theta1s_max = 10
theta1c_min = -10
theta1c_max = 10
V = 100
W = 3000 * 9.81
D = get_Parasitic_Drag(ρ=1.0, f=0.05, Vinfty=90)
Thrust_Needed = get_Thrust_Total(W, D)
N = 40
Ω = 15
R = 10
# θ0_def = 0.4
rotor = Rotor(rotor_mass=150, Ω=Ω, blade_count=3, R=R, rc=0.4, V_infty=90, chord_function=lambda r: 1, θtw=0, ρ=1.0)
# trimSolve(rotor=rotor1, Ω=Ω, θ0_initial=10.015*deg_to_rad, θ1s_initial=-14.02*deg_to_rad, θ1c_initial=3*deg_to_rad, W=MASS_OF_CHINOOK*g, D=drag, coning_angle_iterations=2, β0_step_fraction=1, iterations=5, relaxation=1, verbose=False)
trim_vals, trim_outs = trimSolve(rotor=rotor, Ω=Ω, θ0_initial=10.015*deg_to_rad, θ1s_initial=-14.02*deg_to_rad, θ1c_initial=3*deg_to_rad, W=W, D=D, coning_angle_iterations=5, β0_step_fraction=1, iterations=10, relaxation=1, verbose=False)
print_trim_values_and_state(rotor, trim_vals, trim_outs)
print(f"Thrust needed: {Thrust_Needed:.1f} N")
trim_my = trim_outs[1][1]
θ_def = trim_vals[0]
θ1c_def = trim_vals[1]
θ1s_def = trim_vals[2]

# Convert trim values to degrees for plotting
θ_def_deg = θ_def / deg_to_rad
θ1s_def_deg = θ1s_def / deg_to_rad  
θ1c_def_deg = θ1c_def / deg_to_rad


def plot_variation_against_theta0(fig_name='variation_against_theta0.png'):
    thrusts = []
    powers = []
    momentxs = []
    momentys = []
    momentzs = []
    theta0_deg_values = np.linspace(θ_def_deg + theta0_min, θ_def_deg + theta0_max, N)
    
    for θ0_deg in theta0_deg_values:
        θ0_rad = θ0_deg * deg_to_rad  # Convert back to radians for calculations
        rotor.set_calculation_batch_properties(Thrust_Needed=Thrust_Needed, Ω=Ω, θ0=θ0_rad, θ1s=θ1s_def, θ1c=θ1c_def)
        thrust, moments, power = rotor.perform_all_calculations(W=W, D=D, coning_angle_iterations=5, β0_step_fraction=1)
        thrusts.append(thrust)
        powers.append(power)
        momentxs.append(moments[0])
        momentys.append(moments[1])
        momentzs.append(moments[2])
    # Also add a main title and omega value
    # plt.suptitle(f"Variation of Performance Metrics with θ0 (Ω = {rotor.Ω} rad/s, Thrust = {Thrust_Needed:.0f} N, Rotor Radius = {rotor.R} m)")
    plt.figure(figsize=(12, 8))
    plt.subplot(2, 2, 1)
    plt.plot(theta0_deg_values, thrusts, marker='o')
    plt.title('Thrust vs θ0')
    plt.xlabel('θ0 (degrees)')
    plt.ylabel('Thrust (N)')
    plt.grid()
    plt.subplot(2, 2, 2)
    plt.plot(theta0_deg_values, powers, marker='o', color='orange')
    plt.title('Power vs θ0')   
    plt.xlabel('θ0 (degrees)')
    plt.ylabel('Power (W)')
    plt.grid()
    plt.subplot(2, 2, 3)
    plt.plot(theta0_deg_values, momentxs, marker='o', color='green')
    plt.title('Moment X vs θ0')
    plt.xlabel('θ0 (degrees)')
    plt.ylabel('Moment X (N·m)')
    plt.grid()
    plt.subplot(2, 2, 4)
    plt.plot(theta0_deg_values, momentys, marker='o', color='red')
    plt.title('Moment Y vs θ0')
    plt.xlabel('θ0 (degrees)')
    plt.ylabel('Moment Y (N·m)')
    plt.grid()
    plt.tight_layout()
    plt.savefig(f"./figures/{fig_name}")
    plt.show()
    
    # theta0_values = np.linspace(theta0_min, theta0_max, 50)

def plot_variation_against_theta1s(fig_name='variation_against_theta1s.png'):
    thrusts = []
    powers = []
    momentxs = []
    momentys = []
    momentzs = []
    theta1s_deg_values = np.linspace(θ1s_def_deg + theta1s_min, θ1s_def_deg + theta1s_max, N)
    
    for θ1s_deg in theta1s_deg_values:
        θ1s_rad = θ1s_deg * deg_to_rad  # Convert back to radians for calculations
        rotor.set_calculation_batch_properties(Thrust_Needed=Thrust_Needed, Ω=Ω, θ0=θ_def, θ1s=θ1s_rad, θ1c=θ1c_def)
        thrust, moments, power = rotor.perform_all_calculations(W=W, D=D, coning_angle_iterations=5, β0_step_fraction=1)
        thrusts.append(thrust)
        powers.append(power)
        momentxs.append(moments[0])
        momentys.append(moments[1])
        momentzs.append(moments[2])
    # Also add a main title and omega value
    # plt.suptitle(f"Variation of Performance Metrics with θ1s (Ω = {rotor.Ω} rad/s, Thrust = {Thrust_Needed:.0f} N, Rotor Radius = {rotor.R} m)")
    plt.figure(figsize=(12, 8))
    plt.subplot(2, 2, 1)
    plt.plot(theta1s_deg_values, thrusts, marker='o')
    plt.title('Thrust vs θ1s')
    plt.xlabel('θ1s (degrees)')
    plt.ylabel('Thrust (N)')
    plt.grid()
    plt.subplot(2, 2, 2)
    plt.plot(theta1s_deg_values, powers, marker='o', color='orange')
    plt.title('Power vs θ1s')   
    plt.xlabel('θ1s (degrees)')
    plt.ylabel('Power (W)')
    plt.grid()
    plt.subplot(2, 2, 3)
    plt.plot(theta1s_deg_values, momentxs, marker='o', color='green')
    plt.title('Moment X vs θ1s')
    plt.xlabel('θ1s (degrees)')
    plt.ylabel('Moment X (N·m)')
    plt.grid()
    plt.subplot(2, 2, 4)
    plt.plot(theta1s_deg_values, momentys, marker='o', color='red')
    plt.title('Moment Y vs θ1s')
    plt.xlabel('θ1s (degrees)')
    plt.ylabel('Moment Y (N·m)')
    plt.grid()
    plt.tight_layout()
    plt.savefig(f"./figures/{fig_name}")
    plt.show()

def plot_variation_against_theta1c(fig_name='variation_against_theta1c.png'):
    thrusts = []
    powers = []
    momentxs = []
    momentys = []
    momentzs = []
    theta1c_deg_values = np.linspace(θ1c_def_deg + theta1c_min, θ1c_def_deg + theta1c_max, N)
    
    for θ1c_deg in theta1c_deg_values:
        θ1c_rad = θ1c_deg * deg_to_rad  # Convert back to radians for calculations
        rotor.set_calculation_batch_properties(Thrust_Needed=Thrust_Needed, Ω=Ω, θ0=θ_def, θ1s=θ1s_def, θ1c=θ1c_rad)
        thrust, moments, power = rotor.perform_all_calculations(W=W, D=D, coning_angle_iterations=5, β0_step_fraction=1)
        thrusts.append(thrust)
        powers.append(power)
        momentxs.append(moments[0])
        momentys.append(moments[1])
        momentzs.append(moments[2])
    # Also add a main title and omega value
    # plt.suptitle(f"Variation of Performance Metrics with θ1c (Ω = {rotor.Ω} rad/s, Thrust = {Thrust_Needed:.0f} N, Rotor Radius = {rotor.R} m)")
    plt.figure(figsize=(12, 8))
    plt.subplot(2, 2, 1)
    plt.plot(theta1c_deg_values, thrusts, marker='o')
    plt.title('Thrust vs θ1c')
    plt.xlabel('θ1c (degrees)')
    plt.ylabel('Thrust (N)')
    plt.grid()
    plt.subplot(2, 2, 2)
    plt.plot(theta1c_deg_values, powers, marker='o', color='orange')
    plt.title('Power vs θ1c')   
    plt.xlabel('θ1c (degrees)')
    plt.ylabel('Power (W)')
    plt.grid()
    plt.subplot(2, 2, 3)
    plt.plot(theta1c_deg_values, momentxs, marker='o', color='green')
    plt.title('Moment X vs θ1c')
    plt.xlabel('θ1c (degrees)')
    plt.ylabel('Moment X (N·m)')
    plt.grid()
    plt.subplot(2, 2, 4)
    plt.plot(theta1c_deg_values, momentys, marker='o', color='red')
    plt.title('Moment Y vs θ1c')
    plt.xlabel('θ1c (degrees)')
    plt.ylabel('Moment Y (N·m)')
    plt.grid()
    plt.tight_layout()
    plt.savefig(f"./figures/{fig_name}")
    plt.show()
    
    

if __name__ == "__main__":
    plot_variation_against_theta0(f"N_{N}_theta0_min_{theta0_min}_max_{theta0_max}.png")
    plot_variation_against_theta1s(f"N_{N}_theta1s_min_{theta1s_min}_max_{theta1s_max}.png")
    plot_variation_against_theta1c(f"N_{N}_theta1c_min_{theta1c_min}_max_{theta1c_max}.png")