# Rotor Purified Classes Documentation

## Overview

The `rotor_purified_classes.py` module provides a comprehensive, object-oriented framework for helicopter rotor aerodynamics analysis. The module is designed with pure functions and clean separation of concerns, making it suitable for mission planning and performance analysis.

## Architecture

The module consists of three main classes that work together to model helicopter performance:

### 1. Environment Class
**Purpose**: Manages atmospheric conditions and environmental calculations.

**Key Attributes**:
- `temperature_sea_level`: Sea level temperature (K)
- `pressure_sea_level`: Sea level pressure (Pa)
- `altitude`: Operating altitude (m)
- `Lapse_rate_troposphere`: Temperature lapse rate (K/m)
- `gravitational_acceleration`: Earth's gravity constant (m/s²)
- `Specific_gas_constant`: Air gas constant (J/kg·K)

**Key Methods**:
- `set_atmosphere_parameters()`: Programmatically set all atmospheric parameters
- `get_atmosphere_parameters()`: Interactive input method
- `get_temperature(altitude)`: Calculate temperature at given altitude
- `get_pressure(temperature)`: Calculate pressure using temperature
- `get_density(temperature, pressure)`: Calculate air density
- `get_speed_of_sound(temperature)`: Calculate speed of sound
- `get_mach_number(velocity, temperature)`: Calculate Mach number

### 2. Rotor Class
**Purpose**: Handles all rotor-specific parameters and aerodynamic calculations.

**Key Attributes**:
- `number_of_blades`: Number of rotor blades
- `omega`: Rotor angular velocity (rad/s)
- `radius_of_rotors`: Rotor disk radius (m)
- `root_cutout`: Hub radius (m)
- `root_chord`, `tip_chord`: Chord lengths at root and tip (m)
- `root_pitch`: Blade pitch at root (rad)
- `slope_pitch`: Blade twist distribution

**Key Methods**:
- `set_rotor_parameters()`: Set all rotor parameters programmatically
- `elemental_inflow_ratio()`: Calculate local inflow ratio using momentum theory
- `total_thrust()`: Calculate total rotor thrust
- `total_power()`: Calculate total power required
- `total_torque()`: Calculate total torque
- `calculate_performance()`: Comprehensive performance analysis

### 3. Helicopter Class
**Purpose**: Main coordinator class that manages all helicopter components and provides mission analysis capabilities.

**Key Attributes**:
- `environment`: Environment instance
- `rotor`: Rotor instance
- `fuselage_mass`, `engine_mass`: Component masses (kg)
- Component flags: `fuselage_set`, `engine_set`, `rotor_set`

**Key Methods**:
- `set_fuselage_parameters()`: Define fuselage characteristics
- `set_engine_parameters()`: Define engine specifications
- `get_total_mass()`: Calculate total helicopter mass
- `can_hover()`: Check if helicopter can hover at given climb rate
- `mission_analysis()`: Analyze performance across multiple flight conditions
- `calculate_hover_performance()`: Detailed hover performance metrics

## Workflow

### Basic Usage Workflow

1. **Environment Setup**
   ```python
   env = Environment()
   env.set_atmosphere_parameters(
       temperature_sea_level=288.15,
       Reynolds_number=1e6,
       altitude=1000.0,
       pressure_sea_level=101325.0,
       Lapse_rate_troposphere=0.0065,
       wind_velocity=5.0,
       ISA_OFFSET=0.0
   )
   ```

2. **Rotor Configuration**
   ```python
   rotor = Rotor(env)
   rotor.set_rotor_parameters(
       number_of_blades=4,
       omega=27.0,
       blade_mass=50.0,
       NACA_for_airfoil="0012",
       radius_of_rotors=5.5,
       root_cutout=0.5,
       angle_of_attack=8.0,
       root_chord=0.4,
       tip_chord=0.2,
       root_pitch=12.0,
       slope_pitch=-0.08
   )
   ```

3. **Helicopter Assembly**
   ```python
   helicopter = Helicopter(env)
   helicopter.set_fuselage_parameters(mass=1200.0, length=12.0, width=2.5, height=3.0, cl=0.3, cd=0.05)
   helicopter.set_engine_parameters(mass=300.0, BSFC=0.3, shaft_power_conversion_efficiency=0.92, fractional_power_tail_rotor=0.15)
   helicopter.set_rotor(rotor)
   ```

4. **Performance Analysis**
   ```python
   # Check hover capability
   can_hover, message = helicopter.can_hover(climb_velocity=0)
   
   # Mission analysis
   results = helicopter.mission_analysis([0, 2, 4, 6, 8, 10])
   ```

### Advanced Workflow

For detailed aerodynamic analysis:

1. **Elemental Analysis**
   ```python
   r = rotor.radius_of_rotors * 0.7  # 70% radius
   lambda_inflow, F = rotor.elemental_inflow_ratio(r, climb_velocity=2.0)
   phi = rotor.get_phi_r(r, climb_velocity=2.0)
   alpha_eff = rotor.get_alpha_effective_r(r, climb_velocity=2.0)
   ```

2. **Performance Mapping**
   ```python
   performance = rotor.calculate_performance(climb_velocity=3.0, n_divisions=100, density=1.2)
   # Returns: thrust, torque, power, thrust_x, lift, drag
   ```

## Mathematical Foundation

### Momentum Theory Implementation

The core aerodynamic calculations are based on blade element momentum theory (BEMT):

1. **Inflow Ratio Calculation**
   - Uses iterative solution for inflow ratio (λ)
   - Includes Prandtl tip loss factor (F)
   - Accounts for climb velocity effects

2. **Local Aerodynamics**
   - Effective angle of attack: `α_eff = θ - φ`
   - Lift coefficient: `C_L = a × α_eff`
   - Drag coefficient: `C_D = C_D0 + k × C_L²`

3. **Forces and Moments**
   - Elemental thrust: `dT = ½ρc(C_L cos φ - C_D sin φ)V²dr`
   - Elemental torque: `dQ = r × dFx`
   - Power: `P = Ω × Q`

### Convergence and Accuracy

- Inflow ratio calculation uses iterative convergence (tolerance: 1e-6)
- Default discretization: 50 blade elements
- Tip loss factor provides realistic blade tip effects
- Variable chord and pitch distributions supported

## Key Features

### 1. Pure Function Design
- No global variables
- All dependencies passed as parameters
- Consistent outputs for identical inputs
- Easy testing and validation

### 2. Modular Architecture
- Independent components can work standalone
- Clear separation of atmospheric, rotor, and helicopter concerns
- Easy to extend or modify individual components

### 3. Comprehensive Error Handling
- Parameter validation before calculations
- Clear error messages for missing configurations
- Graceful handling of edge cases

### 4. Mission Planning Integration
- `can_hover()` for quick feasibility checks
- `mission_analysis()` for performance envelopes
- Parametric studies across flight conditions

### 5. Type Safety
- Complete type hints for all methods and attributes
- Support for static type checking
- Clear interface definitions

## Performance Considerations

### Computational Efficiency
- Default discretization (50 elements) provides good accuracy/speed balance
- Increase `n_divisions` for higher precision analysis
- Iterative convergence typically reaches solution in <20 iterations

### Memory Usage
- Lightweight object design
- Arrays created only during calculations
- No persistent large data structures

## Testing and Validation

The module includes comprehensive tests:

1. **Integration Tests**: Full helicopter analysis workflow
2. **Component Tests**: Individual class functionality
3. **Error Handling Tests**: Validation of error conditions
4. **Performance Tests**: Verification of aerodynamic calculations

Run tests with:
```bash
python rotor_purified_classes.py
```

## Limitations and Assumptions

1. **Rotor Model Limitations**:
   - Uniform inflow assumption
   - No dynamic effects (steady-state only)
   - Linear aerodynamics model
   - No compressibility effects

2. **Environmental Assumptions**:
   - Standard atmosphere model
   - No wind shear effects
   - Uniform density across rotor disk

3. **Helicopter Model**:
   - Point mass assumption
   - No fuselage interference on rotor
   - Simplified engine model

## Extension Possibilities

The modular design allows for easy extensions:

1. **Advanced Rotor Models**: Non-uniform inflow, dynamic wake
2. **Flight Dynamics**: Forward flight, maneuvering loads
3. **Optimization**: Parameter optimization for mission requirements
4. **Visualization**: Performance plots and contour maps
5. **Database Integration**: Aircraft parameter databases

## Usage Examples

See the test section (`if __name__ == '__main__':`) in the main file for comprehensive usage examples including:

- Basic helicopter setup and analysis
- Component isolation testing
- Mission analysis workflows
- Error handling demonstrations
- Performance comparison studies

This framework provides a solid foundation for helicopter performance analysis while maintaining code clarity, testability, and extensibility.
