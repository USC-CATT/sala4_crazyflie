# Crazyflie PID Controller - Python Implementation

A complete Python implementation of the Crazyflie firmware PID controller, converted from the original C code.

## Overview

This package provides a Python implementation of the cascaded PID controller used in the Crazyflie drone firmware. It includes:

- **Base PID Controller** with feedforward and low-pass filtering
- **Attitude Controller** (inner and outer loops for roll, pitch, yaw)
- **Position Controller** (velocity and position control)
- **Main Controller** integrating all components
- **Data Types** matching the Crazyflie firmware structures
- **Default Parameters** from the Crazyflie firmware

## File Structure

```
controller/
├── __init__.py                  # Package initialization and exports
├── pid.py                       # Base PID controller implementation
├── types.py                     # Data types and structures
├── constants.py                 # Default PID parameters
├── attitude_controller.py       # Attitude control (inner/outer loops)
├── position_controller.py       # Position and velocity control
├── controller_pid.py            # Main controller integration
├── example_usage.py             # Usage examples
└── README.md                    # This file
```

## Architecture

The controller follows a cascaded architecture:

```
Position Setpoint → Position Controller → Velocity Setpoint
                                              ↓
                         Velocity Controller → Attitude Setpoint (Roll/Pitch)
                                              ↓
                         Attitude Controller → Rate Setpoint
                                              ↓
                         Rate Controller → Motor Commands
```

### Control Loops

1. **Position Loop** (100 Hz)
   - Converts position error to velocity commands
   - PIDs: X, Y, Z position

2. **Velocity Loop** (100 Hz)
   - Converts velocity error to attitude commands (roll/pitch) and thrust
   - PIDs: VX, VY, VZ velocity

3. **Attitude Loop** (500 Hz)
   - Converts attitude error to rate commands
   - PIDs: Roll, Pitch, Yaw attitude

4. **Rate Loop** (500 Hz)
   - Converts rate error to motor commands
   - PIDs: Roll rate, Pitch rate, Yaw rate

## Installation

Simply ensure the `controller` package is in your Python path:

```python
from controller import ControllerPID, Setpoint, State, Control, SensorData
```

## Quick Start

### Basic Hover Example

```python
from controller import (
    ControllerPID, 
    Setpoint, 
    State, 
    Control, 
    SensorData,
    Position,
    Velocity,
    Attitude,
    SetpointMode,
    StabMode
)

# Initialize controller
controller = ControllerPID()
controller.init()

# Create state (current drone state)
state = State()
state.position = Position(x=0.0, y=0.0, z=0.5)
state.velocity = Velocity(x=0.0, y=0.0, z=0.0)
state.attitude = Attitude(roll=0.0, pitch=0.0, yaw=0.0)

# Create sensor data
sensors = SensorData()
sensors.gyro.x = 0.0  # deg/s
sensors.gyro.y = 0.0
sensors.gyro.z = 0.0
sensors.acc.z = 1.0   # g

# Create setpoint (desired state)
setpoint = Setpoint()
setpoint.position = Position(x=0.0, y=0.0, z=1.0)  # Hover at 1m
setpoint.mode.x = StabMode.MODE_ABS  # Position control
setpoint.mode.y = StabMode.MODE_ABS
setpoint.mode.z = StabMode.MODE_ABS

# Create control output
control = Control()

# Run controller
stabilizer_step = 0
controller.controller_pid(control, setpoint, sensors, state, stabilizer_step)

# Use control outputs
print(f"Thrust: {control.thrust}")
print(f"Roll: {control.roll}")
print(f"Pitch: {control.pitch}")
print(f"Yaw: {control.yaw}")
```

## Control Modes

The controller supports different modes for each axis:

### StabMode

- **MODE_DISABLE**: No control (manual or passthrough)
- **MODE_ABS**: Absolute position/attitude control
- **MODE_VELOCITY**: Velocity/rate control

### Example Mode Configurations

#### Position Control (GPS-like)
```python
setpoint.mode.x = StabMode.MODE_ABS
setpoint.mode.y = StabMode.MODE_ABS
setpoint.mode.z = StabMode.MODE_ABS
setpoint.position = Position(x=1.0, y=2.0, z=1.5)
```

#### Velocity Control
```python
setpoint.mode.x = StabMode.MODE_VELOCITY
setpoint.mode.y = StabMode.MODE_VELOCITY
setpoint.mode.z = StabMode.MODE_ABS
setpoint.velocity = Velocity(x=0.5, y=0.0, z=0.0)  # Move forward at 0.5 m/s
```

#### Rate Control (Acro mode)
```python
setpoint.mode.roll = StabMode.MODE_VELOCITY
setpoint.mode.pitch = StabMode.MODE_VELOCITY
setpoint.mode.yaw = StabMode.MODE_VELOCITY
setpoint.attitude_rate.roll = 30.0  # deg/s
setpoint.thrust = 40000  # Manual thrust
```

## Data Types

### State
Current state of the drone:
- `position`: Position(x, y, z) in meters
- `velocity`: Velocity(x, y, z) in m/s
- `attitude`: Attitude(roll, pitch, yaw) in degrees

### Setpoint
Desired state:
- `position`: Position(x, y, z) in meters
- `velocity`: Velocity(x, y, z) in m/s
- `attitude`: Attitude(roll, pitch, yaw) in degrees
- `attitude_rate`: AttitudeRate(roll, pitch, yaw) in deg/s
- `thrust`: Direct thrust value (0-65535)
- `mode`: SetpointMode for each axis

### SensorData
Sensor readings:
- `gyro`: GyroData(x, y, z) in deg/s
- `acc`: AccData(x, y, z) in g

### Control
Output commands:
- `roll`: Roll command (-32767 to 32767)
- `pitch`: Pitch command (-32767 to 32767)
- `yaw`: Yaw command (-32767 to 32767)
- `thrust`: Thrust command (0-65535)

## PID Parameters

Default parameters are defined in `constants.py`. You can modify them:

### Attitude PIDs
```python
from controller import constants

# Modify attitude gains
constants.PID_ROLL_KP = 6.0
constants.PID_ROLL_KI = 3.0
constants.PID_ROLL_KD = 0.0
```

### Position PIDs
```python
# Modify position gains
constants.PID_POS_X_KP = 2.0
constants.PID_POS_Y_KP = 2.0
constants.PID_POS_Z_KP = 2.0
```

### Velocity PIDs
```python
# Modify velocity gains
constants.PID_VEL_X_KP = 25.0
constants.PID_VEL_Y_KP = 25.0
constants.PID_VEL_Z_KP = 25.0
```

## Coordinate Frames

### World Frame (Global)
- X: Forward
- Y: Left
- Z: Up

### Body Frame
- X: Forward relative to drone
- Y: Left relative to drone
- Z: Up relative to drone

The controller handles coordinate transformations automatically based on the yaw angle.

## Advanced Usage

### Accessing Individual Controllers

```python
# Access attitude controller directly
attitude_controller = controller.attitude_controller

# Get current PID outputs
roll_out, pitch_out, yaw_out = attitude_controller.get_actuator_output()

# Reset PIDs
attitude_controller.reset_all_pid(0.0, 0.0, 0.0)
```

### Custom PID Object

```python
from controller import PIDObject

# Create custom PID
pid = PIDObject(
    kp=5.0,
    ki=2.0,
    kd=0.5,
    dt=0.01,
    cutoff_freq=20.0
)

# Use it
output = pid.update(measured_value)
```

### Logging Data

```python
# Get all logging variables
log_data = controller.get_logging_data()

print(f"Commanded thrust: {log_data['cmd_thrust']}")
print(f"Desired attitude: Roll={log_data['attitude_desired_roll']}")
print(f"Rate setpoint: {log_data['rate_desired_pitch']}")
```

## Examples

Run the example file to see various usage patterns:

```bash
python example_usage.py
```

This demonstrates:
1. Hovering at a fixed height
2. Tracking a circular trajectory
3. Velocity control mode
4. Rate control (manual flying)
5. Accessing logging data

## Comparison with C Implementation

This Python implementation closely follows the C firmware:

| Feature | C Firmware | Python Implementation |
|---------|------------|----------------------|
| Cascaded PIDs | ✓ | ✓ |
| Low-pass filtering | ✓ | ✓ |
| Feedforward | ✓ | ✓ |
| Integral anti-windup | ✓ | ✓ |
| Multiple control modes | ✓ | ✓ |
| Rate limiting | ✓ | ✓ |
| Coordinate transforms | ✓ | ✓ |

### Key Differences

1. **Object-Oriented**: Python version uses classes while C uses structs
2. **Type Safety**: Python uses dataclasses for better type hints
3. **Compatibility**: Both C-style functions and object methods provided
4. **Performance**: C is faster, but Python is more readable and easier to modify

## Tuning Guide

### Step 1: Rate Loop (Inner)
Start with the rate loop (gyro-based):
1. Set `PID_ROLL_RATE_KP`, `PID_PITCH_RATE_KP` around 70
2. Add integral gain if steady-state error exists
3. Add derivative for damping if needed

### Step 2: Attitude Loop
Tune the attitude loop:
1. Set `PID_ROLL_KP`, `PID_PITCH_KP` around 6
2. Add integral to eliminate steady-state error
3. Adjust derivative for stability

### Step 3: Velocity Loop
Tune velocity control:
1. Set `PID_VEL_X_KP`, `PID_VEL_Y_KP` around 25
2. Adjust Z axis separately (usually needs higher gains)

### Step 4: Position Loop
Finally tune position:
1. Start with `PID_POS_X_KP`, `PID_POS_Y_KP` around 2
2. Increase until oscillations appear, then reduce

## Integration with Simulation

This controller can be integrated with physics simulators:

```python
# Simulation loop
while simulating:
    # Get state from simulator
    state = simulator.get_state()
    sensors = simulator.get_sensors()
    
    # Run controller
    controller.controller_pid(control, setpoint, sensors, state, step)
    
    # Apply commands to simulator
    simulator.set_motor_commands(
        thrust=control.thrust,
        roll=control.roll,
        pitch=control.pitch,
        yaw=control.yaw
    )
    
    step += 1
```

## Troubleshooting

### Controller oscillates
- Reduce proportional gains (KP)
- Increase derivative gains (KD)
- Check filter cutoff frequencies

### Slow response
- Increase proportional gains (KP)
- Verify output limits aren't too restrictive

### Steady-state error
- Increase integral gains (KI)
- Check integral limits
- Verify setpoint is achievable

### Unstable
- Reduce all gains
- Check sensor data is valid
- Verify coordinate frame transformations

## References

- [Crazyflie Firmware](https://github.com/bitcraze/crazyflie-firmware)
- [Bitcraze Documentation](https://www.bitcraze.io/documentation/)
- Original C files:
  - `controller_pid.c`
  - `attitude_pid_controller.c`
  - `position_controller_pid.c`

## License

Based on the Crazyflie firmware by Bitcraze AB, licensed under GPL v3.

## Contributing

This is a conversion of the C firmware to Python. When updating, ensure compatibility with the latest firmware version.