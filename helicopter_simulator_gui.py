#!/usr/bin/env python3

import tkinter as tk
from tkinter import ttk
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
import sys
import os
import time

from heli import Rotor, Helicopter, Environment

class Interface:
    def __init__(se            self.update_visualization()
            self.update_plot()
            
        except Exception as e:
            print(f"Error: {e}")
    
    def update_visualization(self):
        self.vis_ax.clear()
        
        fuselage_length = 11.5
        fuselage_width = 2.3
        main_radius = self.rotor_radius_var.get()
        tail_radius = 1.5      self.root = root
        self.root.title("Simple Helicopter Simulator")
        self.root.geometry("1000x600")
        
        self.env = Environment()
        self.env = Environment()
        self.env.set_atmosphere_parameters(
            temperature_sea_level=288.15,
            Reynolds_number=1e6,
            altitude=0.0,
            pressure_sea_level=101325,
            Lapse_rate_troposphere=0.0065,
            wind_velocity=0,
            ISA_OFFSET=0
        )
        
        self.main_rotor = Rotor(environment=self.env)
        self.main_rotor.set_rotor_parameters(
            number_of_blades=3,
            blade_mass=50,
            NACA_for_airfoil="0012",
            radius_of_rotors=3.6,
            root_cutout=0.3,
            root_chord=0.09,
            tip_chord=0.05,
            root_pitch=5.0,
            slope_pitch=0.5
        )
        self.tail_rotor = Rotor(environment=self.env)
        self.tail_rotor.set_rotor_parameters(
            number_of_blades=4,
            blade_mass=10,
            NACA_for_airfoil="0012",
            radius_of_rotors=1.2,
            root_cutout=0.1,
            root_chord=0.04,
            tip_chord=0.03,
            root_pitch=2.0,
            slope_pitch=0.1
        )
        self.heli = Helicopter(environment=self.env)
        self.heli.set_rotor(rotor1=self.main_rotor, rotor2=self.tail_rotor)
        self.heli.set_fuselage_parameters(
            mass=5090,
            length=11.5,
            width=2.3,
            height=1.5,
            tail_mass=100,
            tail_length=6.5,
            tail_width=1.5,
            tail_height=0.5,
        )
        self.heli.set_engine_parameters(
            mass=500,
            max_power=1142*1000
        )
        self.heli.set_rotor_positions(
            main_rotor_position=4,
            tail_rotor_position=17
        )
        self.create_gui()
        self.update_simulation()
    
    def create_gui(self):
        main_frame = ttk.Frame(self.root)
        main_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        left_frame = ttk.Frame(main_frame)
        left_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=5, pady=5)
        
        right_frame = ttk.Frame(main_frame)
        right_frame.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True, padx=5, pady=5)
        
        vis_frame = ttk.LabelFrame(left_frame, text="Helicopter")
        vis_frame.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        
        self.vis_figure = Figure(figsize=(5, 5), dpi=100)
        self.vis_canvas = FigureCanvasTkAgg(self.vis_figure, vis_frame)
        self.vis_canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        
        self.vis_ax = self.vis_figure.add_subplot(111)
        control_frame = ttk.LabelFrame(right_frame, text="Controls")
        control_frame.pack(fill=tk.X, padx=5, pady=5)
        
        self.omega_var = tk.DoubleVar(value=30.0)
        
        omega_label_frame = ttk.Frame(control_frame)
        omega_label_frame.pack(fill=tk.X, padx=5, pady=5)
        
        ttk.Label(omega_label_frame, text="Omega (rad/s):").pack(side=tk.LEFT)
        ttk.Label(omega_label_frame, textvariable=self.omega_var, width=6).pack(side=tk.RIGHT)
        
        omega_slider = ttk.Scale(
            control_frame, 
            from_=0.0, 
            to=120.0,
            variable=self.omega_var, 
            orient=tk.HORIZONTAL,
            length=300,
            command=self.on_slider_change
        )
        omega_slider.pack(fill=tk.X, padx=5, pady=5)
        
        # Add helicopter mass slider
        self.mass_var = tk.DoubleVar(value=5090.0)  # Default mass from helicopter
        
        mass_label_frame = ttk.Frame(control_frame)
        mass_label_frame.pack(fill=tk.X, padx=5, pady=5)
        
        ttk.Label(mass_label_frame, text="Helicopter Mass (kg):").pack(side=tk.LEFT)
        ttk.Label(mass_label_frame, textvariable=self.mass_var, width=6).pack(side=tk.RIGHT)
        
        mass_slider = ttk.Scale(
            control_frame, 
            from_=2000.0,
            to=10000.0,
            variable=self.mass_var, 
            orient=tk.HORIZONTAL,
            length=300,
            command=self.on_slider_change
        )
        mass_slider.pack(fill=tk.X, padx=5, pady=5)
        
        # Add main rotor radius slider
        self.rotor_radius_var = tk.DoubleVar(value=3.6)  # Default from initialization
        
        radius_label_frame = ttk.Frame(control_frame)
        radius_label_frame.pack(fill=tk.X, padx=5, pady=5)
        
        ttk.Label(radius_label_frame, text="Main Rotor Radius (m):").pack(side=tk.LEFT)
        ttk.Label(radius_label_frame, textvariable=self.rotor_radius_var, width=6).pack(side=tk.RIGHT)
        
        radius_slider = ttk.Scale(
            control_frame, 
            from_=2.0,
            to=8.0,
            variable=self.rotor_radius_var, 
            orient=tk.HORIZONTAL,
            length=300,
            command=self.on_slider_change
        )
        radius_slider.pack(fill=tk.X, padx=5, pady=5)
        
        # Add root chord slider
        self.root_chord_var = tk.DoubleVar(value=0.09)  # Default from initialization
        
        root_chord_label_frame = ttk.Frame(control_frame)
        root_chord_label_frame.pack(fill=tk.X, padx=5, pady=5)
        
        ttk.Label(root_chord_label_frame, text="Root Chord (m):").pack(side=tk.LEFT)
        ttk.Label(root_chord_label_frame, textvariable=self.root_chord_var, width=6).pack(side=tk.RIGHT)
        
        root_chord_slider = ttk.Scale(
            control_frame, 
            from_=0.05,
            to=0.20,
            variable=self.root_chord_var, 
            orient=tk.HORIZONTAL,
            length=300,
            command=self.on_slider_change
        )
        root_chord_slider.pack(fill=tk.X, padx=5, pady=5)
        
        # Add tip chord slider
        self.tip_chord_var = tk.DoubleVar(value=0.05)  # Default from initialization
        
        tip_chord_label_frame = ttk.Frame(control_frame)
        tip_chord_label_frame.pack(fill=tk.X, padx=5, pady=5)
        
        ttk.Label(tip_chord_label_frame, text="Tip Chord (m):").pack(side=tk.LEFT)
        ttk.Label(tip_chord_label_frame, textvariable=self.tip_chord_var, width=6).pack(side=tk.RIGHT)
        
        tip_chord_slider = ttk.Scale(
            control_frame, 
            from_=0.03,
            to=0.15,
            variable=self.tip_chord_var, 
            orient=tk.HORIZONTAL,
            length=300,
            command=self.on_slider_change
        )
        tip_chord_slider.pack(fill=tk.X, padx=5, pady=5)
        
        # Add climb velocity slider (only positive values 0-60)
        self.climb_velocity_var = tk.DoubleVar(value=0.0)
        
        climb_label_frame = ttk.Frame(control_frame)
        climb_label_frame.pack(fill=tk.X, padx=5, pady=5)
        
        ttk.Label(climb_label_frame, text="Climb Velocity (m/s):").pack(side=tk.LEFT)
        ttk.Label(climb_label_frame, textvariable=self.climb_velocity_var, width=6).pack(side=tk.RIGHT)
        
        climb_slider = ttk.Scale(
            control_frame, 
            from_=0.0,  # Only positive values
            to=60.0,  # Increased range to 60
            variable=self.climb_velocity_var, 
            orient=tk.HORIZONTAL,
            length=300,
            command=self.on_slider_change
        )
        climb_slider.pack(fill=tk.X, padx=5, pady=5)
        
        # Add altitude slider
        self.altitude_var = tk.DoubleVar(value=0.0)
        
        altitude_label_frame = ttk.Frame(control_frame)
        altitude_label_frame.pack(fill=tk.X, padx=5, pady=5)
        
        ttk.Label(altitude_label_frame, text="Altitude (m):").pack(side=tk.LEFT)
        ttk.Label(altitude_label_frame, textvariable=self.altitude_var, width=6).pack(side=tk.RIGHT)
        
        altitude_slider = ttk.Scale(
            control_frame, 
            from_=0.0,
            to=5000.0,  # 5000 meters max altitude
            variable=self.altitude_var, 
            orient=tk.HORIZONTAL,
            length=300,
            command=self.on_slider_change
        )
        altitude_slider.pack(fill=tk.X, padx=5, pady=5)
        
        # Create performance plot
        plot_frame = ttk.LabelFrame(right_frame, text="Performance")
        plot_frame.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        
        self.plot_figure = Figure(figsize=(5, 4), dpi=100)
        self.plot_canvas = FigureCanvasTkAgg(self.plot_figure, plot_frame)
        self.plot_canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        
        self.plot_ax = self.plot_figure.add_subplot(111)
        
        # Add update button
        update_button = ttk.Button(right_frame, text="Update Simulation", command=self.update_simulation)
        update_button.pack(pady=10)
        
    def on_slider_change(self, event=None):
        if not hasattr(self, 'last_update_time'):
            self.last_update_time = time.time()
            self.last_omega = self.omega_var.get()
            self.last_climb_velocity = self.climb_velocity_var.get()
            self.last_altitude = self.altitude_var.get()
            self.last_mass = self.mass_var.get()
            self.last_rotor_radius = self.rotor_radius_var.get()
            self.last_root_chord = self.root_chord_var.get()
            self.last_tip_chord = self.tip_chord_var.get()
        
        current_time = time.time()
        
        if current_time - self.last_update_time > 0.2:
            self.last_omega = self.omega_var.get()
            self.last_climb_velocity = self.climb_velocity_var.get()
            self.last_altitude = self.altitude_var.get()
            self.last_mass = self.mass_var.get()
            self.last_rotor_radius = self.rotor_radius_var.get()
            self.last_root_chord = self.root_chord_var.get()
            self.last_tip_chord = self.tip_chord_var.get()
            self.last_update_time = current_time
            
            # Schedule update to give time for GUI to respond
            self.root.after(10, self.update_simulation)
        
    def update_simulation(self):
        try:
            omega = self.omega_var.get()
            vertical_velocity = self.climb_velocity_var.get()
            altitude = self.altitude_var.get()
            mass = self.mass_var.get()
            rotor_radius = self.rotor_radius_var.get()
            root_chord = self.root_chord_var.get()
            tip_chord = self.tip_chord_var.get()
            self.heli.set_fuselage_parameters(
                mass=mass,
                length=11.5,
                width=2.3,
                height=1.5,
                tail_mass=100,
                tail_length=6.5,
                tail_width=1.5,
                tail_height=0.5,
            )
            
            # Update main rotor parameters
            self.main_rotor.set_rotor_parameters(
                number_of_blades=3,
                blade_mass=50,
                NACA_for_airfoil="0012",
                radius_of_rotors=rotor_radius,
                root_cutout=0.3,
                root_chord=root_chord,
                tip_chord=tip_chord,
                root_pitch=5.0,
                slope_pitch=0.5,
            )
            
            # Calculate performance metrics
            temperature = self.env.get_temperature(altitude=altitude)
            density = self.env.get_density(temperature=temperature)
            
            try:
                main_perf = self.main_rotor.calculate_performance(vertical_velocity, omega, density)
                
                if main_perf['thrust'] < 0:
                    main_perf['thrust'] = abs(main_perf['thrust'])
            except Exception as e:
                print(f"Error: {e}")
                main_perf = {
                    'thrust': 50000,
                    'power': 500000,
                    'torque': 20000
                }            
            tail_moment_arm = self.heli.tail_rotor_position - self.heli.main_rotor_position
            tail_thrust_needed = main_perf['torque'] / tail_moment_arm if tail_moment_arm > 0 else 0
            
            tail_omega = omega * 2
            weight = self.heli.get_total_mass() * self.env.gravitational_acceleration
            
            self.performance = {
                'omega': omega,
                'thrust': main_perf['thrust'],
                'power': main_perf['power'],
                'torque': main_perf['torque'],
                'weight': weight
            }
            self.update_visualization()
            self.update_plot()
            
        except Exception as e:
            print(f"Error in simulation update: {e}")
    
    def update_visualization(self):
        self.vis_ax.clear()
        
        fuselage_length = 11.5
        fuselage_width = 2.3
        main_radius = self.rotor_radius_var.get()
        tail_radius = 1.5
        
        # Draw fuselage (simple rectangle)
        fuselage = plt.Rectangle((-fuselage_length/4, -fuselage_width/2), 
                                 fuselage_length/2, fuselage_width, 
                                 fill=True, color='gray', alpha=0.7)
        self.vis_ax.add_patch(fuselage)
        
        # Draw tail boom
        tail_boom = plt.Rectangle((-fuselage_length/4, -0.3), 
                                  fuselage_length*0.75, 0.6, 
                                  fill=True, color='gray', alpha=0.5)
        self.vis_ax.add_patch(tail_boom)
        
        # Draw main rotor disk
        main_rotor_circle = plt.Circle((0, 0), main_radius, 
                                      fill=False, color='blue', linestyle='--')
        self.vis_ax.add_patch(main_rotor_circle)
        
        # Draw tail rotor disk
        tail_rotor_circle = plt.Circle((fuselage_length*0.5, 0), 
                                      tail_radius, 
                                      fill=False, color='green', linestyle='--')
        self.vis_ax.add_patch(tail_rotor_circle)
        
            # Draw thrust arrow if performance results exist
        if hasattr(self, 'performance'):
            thrust = self.performance['thrust']
            weight = self.performance['weight']
            net_force = thrust - weight
            
            # Scale arrow length based on net force
            arrow_scale = 10000  # Adjust this value to get a good arrow size
            max_radius = main_radius * 1.2
            arrow_length = min(max_radius, abs(net_force) / arrow_scale)            # Arrow direction: up if positive net force, down if negative
            arrow_direction = 1 if net_force > 0 else -1
            dx, dy = 0, arrow_length * arrow_direction
            
            if arrow_length > 0:
                arrow_color = 'green' if net_force > 0 else 'red'
                self.vis_ax.arrow(0, 0, dx, dy, 
                                 head_width=max_radius*0.15, 
                                 head_length=max_radius*0.2,
                                 fc=arrow_color, ec=arrow_color, 
                                 linewidth=2)
                
                # Add text labels for thrust and weight
                self.vis_ax.text(main_radius*0.2, main_radius*0.8, 
                                f"Thrust: {thrust/1000:.1f} kN", 
                                fontsize=10, color='blue')
                self.vis_ax.text(main_radius*0.2, main_radius*0.7, 
                                f"Weight: {weight/1000:.1f} kN", 
                                fontsize=10, color='purple')
                self.vis_ax.text(main_radius*0.2, main_radius*0.6, 
                                f"Net Force: {net_force/1000:.1f} kN", 
                                fontsize=10, color=arrow_color)
        
        # Set axis properties
        self.vis_ax.set_xlim(-fuselage_length, fuselage_length)
        self.vis_ax.set_ylim(-main_radius*1.5, main_radius*1.5)
        self.vis_ax.set_aspect('equal')
        self.vis_ax.set_xlabel('Length (m)')
        self.vis_ax.set_ylabel('Width (m)')
        self.vis_ax.set_title('Helicopter Schematic')
        self.vis_ax.grid(True)
        
        # Update the canvas
        self.vis_figure.tight_layout()
        self.vis_canvas.draw()
    
    def update_plot(self):
        self.plot_ax.clear()
        
        if hasattr(self, 'performance'):
            power_kw = self.performance['power'] / 1000
            thrust_kn = self.performance['thrust'] / 1000
            weight_kn = self.performance['weight'] / 1000
            net_force_kn = (self.performance['thrust'] - self.performance['weight']) / 1000
            torque_nm = self.performance['torque']
            
            omega = self.performance['omega']
            climb_velocity = self.climb_velocity_var.get()
            altitude = self.altitude_var.get()
            
            text_props = dict(boxstyle='round', facecolor='wheat', alpha=0.5)
            info_text = (
                f"FLIGHT PARAMETERS:\n"
                f"Omega: {omega:.1f} rad/s\n"
                f"Climb Velocity: {climb_velocity:.1f} m/s\n"
                f"Altitude: {altitude:.1f} m\n\n"
                f"PERFORMANCE METRICS:\n"
                f"Thrust: {thrust_kn:.1f} kN\n"
                f"Weight: {weight_kn:.1f} kN\n"
                f"Net Force: {net_force_kn:.1f} kN\n"
                f"Power: {power_kw:.1f} kW\n"
                f"Torque: {torque_nm:.1f} N-m (cancelled by tail rotor)\n\n"
                f"STATUS: {'ASCENDING' if net_force_kn > 0 else 'DESCENDING' if net_force_kn < 0 else 'HOVERING'}"
            )
            
            self.plot_ax.axis('off')
            self.plot_ax.text(0.5, 0.5, info_text, 
                transform=self.plot_ax.transAxes,
                fontsize=11, verticalalignment='center', horizontalalignment='center',
                bbox=text_props)
        
        # Update the canvas
        self.plot_figure.tight_layout()
        self.plot_canvas.draw()



if __name__ == "__main__":
    root = tk.Tk()
    app = Interface(root)
    root.mainloop()
