# Crazyflie PID Controller - C to Python Conversion Summary

## Overview

This document summarizes the conversion of the Crazyflie firmware PID controller from C to Python.

## Conversion Date
2024

## Source Files (C)

The following C source files were converted to Python:

1. **controller_pid.c** - Main controller integration
2. **attitude_pid_controller.c** - Attitude control (inner/outer loops)
3. **position_controller_pid.c** - Position and velocity control

## Generated Files (Python)

### Core Implementation

1. **pid.py** (270 lines)
   - Base PID controller class with feedforward and filtering
   - Implements: `PIDObject` class
   - Features: Low-pass filtering, integral anti-windup, output limiting
   - C-style compatibility functions provided

2. **types.py** (252 lines)
   - Data structures matching C `stabilizer_types.h`
   - Dataclasses: `State`, `Setpoint`, `Control`, `SensorData`, etc.
   - Utility functions: angle wrapping, coordinate conversions, quaternion to RPY
   - Enumerations: `StabMode`, `ControlMode`

3. **constants.py** (122 lines)
   - Default PID parameters from `platform_defaults.h`
   - All gains, limits, and filter parameters
   - Configuration flags

4. **attitude_controller.py** (393 lines)
   - Converted from `attitude_pid_controller.c`
   - Class: `AttitudeController`
   - Implements cascaded attitude and rate PIDs (6 PIDs total)
   - Handles roll, pitch, yaw control at 500 Hz
   - C-style function wrappers for compatibility

5. **position_controller.py** (413 lines)
   - Converted from `position_controller_pid.c`
   - Class: `PositionController`
   - Implements position and velocity PIDs (6 PIDs total)
   - Handles X, Y, Z position/velocity control at 100 Hz
   - Body frame coordinate transformations

6. **controller_pid.py** (291 lines)
   - Converted from `controller_pid.c`
   - Class: `ControllerPID`
   - Main controller integrating attitude and position controllers
   - Rate limiting logic (ATTITUDE_RATE, POSITION_RATE)
   - Multiple control mode support
   - Logging variables

### Supporting Files

7. **__init__.py** (153 lines)
   - Package initialization
   - Exports all public classes and functions
   - Clean import interface

8. **example_usage.py** (376 lines)
   - 5 complete examples demonstrating usage
   - Hover control, trajectory tracking, velocity control, rate control, logging

9. **test_controller.py** (298 lines)
   - Automated test suite
   - 7 test cases covering all control modes
   - Validates initialization, control outputs, logging

10. **README.md** (402 lines)
    - Comprehensive documentation
    - Architecture diagrams
    - Usage examples
    - Tuning guide
    - Troubleshooting

11. **CONVERSION_SUMMARY.md** (This file)
    - Conversion documentation
    - Design decisions
    - Differences from C implementation

## Total Lines of Code

- **C Source**: ~900 lines (3 files)
- **Python Implementation**: ~2,970 lines (11 files including docs/tests)
- **Core Python Code**: ~1,741 lines (files 1-6)
- **Documentation/Examples**: ~1,229 lines (files 7-11)

## Conversion Approach

### Object-Oriented Design

The C structs and functions were converted to Python classes:

```c
// C: Struct with functions
typedef struct {
    float kp, ki, kd;
    float error, integ, deriv;
} PidObject;

void pidInit(PidObject* pid, ...);
float pidUpdate(PidObject* pid, float measured, bool updateError);
```

```python
# Python: Class with methods
class PIDObject:
    def __init__(self, kp=0.0, ki=0.0, kd=0.0, ...):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.error = 0.0
        self.integ = 0.0
        self.deriv = 0.0
    
    def update(self, measured: float, update_error: bool = True) -> float:
        # Implementation
```

### Compatibility Layer

C-style function interfaces were provided for backward compatibility:

```python
# C-style function wrapper
def pid_update(pid: PIDObject, measured: float, update_error: bool) -> float:
    return pid.update(measured, update_error)
```

### Type Safety

Python dataclasses with type hints replace C structs:

```c
// C
typedef struct {
    float x, y, z;
} position_t;
```

```python
# Python
@dataclass
class Position:
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
```

## Key Differences from C Implementation

### 1. Memory Management
- **C**: Manual memory management, stack allocation
- **Python**: Automatic garbage collection, dynamic allocation

### 2. Numeric Types
- **C**: Fixed-size types (int16_t, uint16_t, float)
- **Python**: Dynamic typing with type hints for clarity

### 3. Control Flow
- **C**: Preprocessor macros (RATE_DO_EXECUTE)
- **Python**: Functions with same logic

```c
// C Macro
#define RATE_DO_EXECUTE(RATE_HZ, TICK) ((TICK % (ATTITUDE_RATE / RATE_HZ)) == 0)
```

```python
# Python Function
def rate_do_execute(rate: int, step: int) -> bool:
    if rate == ATTITUDE_RATE:
        return True
    elif rate == POSITION_RATE:
        return (step % 5) == 0
    return False
```

### 4. Filter Implementation
- **C**: First-order IIR filter with careful initialization
- **Python**: Same algorithm, more readable code

### 5. Logging
- **C**: LOG_ADD macros for firmware logging system
- **Python**: `get_logging_data()` method returning dictionary

## Functional Equivalence

All core functionality is preserved:

| Feature | C Implementation | Python Implementation |
|---------|------------------|----------------------|
| Cascaded PID loops | ✓ | ✓ |
| 12 total PIDs | ✓ | ✓ |
| Low-pass filtering | ✓ | ✓ |
| Feedforward control | ✓ | ✓ |
| Integral anti-windup | ✓ | ✓ |
| Output saturation | ✓ | ✓ |
| Multiple control modes | ✓ | ✓ |
| Rate limiting | ✓ | ✓ |
| Body/world frame transforms | ✓ | ✓ |
| Quaternion support | ✓ | ✓ |
| Yaw angle wrapping | ✓ | ✓ |

## Control Architecture

### Update Rates
- **Attitude/Rate Loop**: 500 Hz
- **Position/Velocity Loop**: 100 Hz

### PID Cascade

```
Position (100 Hz)
    ├── X Position PID → X Velocity Setpoint
    ├── Y Position PID → Y Velocity Setpoint
    └── Z Position PID → Z Velocity Setpoint

Velocity (100 Hz)
    ├── X Velocity PID → Pitch Command
    ├── Y Velocity PID → Roll Command
    └── Z Velocity PID → Thrust Command

Attitude (500 Hz)
    ├── Roll Attitude PID → Roll Rate Setpoint
    ├── Pitch Attitude PID → Pitch Rate Setpoint
    └── Yaw Attitude PID → Yaw Rate Setpoint

Rate (500 Hz)
    ├── Roll Rate PID → Roll Motor Command
    ├── Pitch Rate PID → Pitch Motor Command
    └── Yaw Rate PID → Yaw Motor Command
```

## Control Modes

### Position Control (MODE_ABS)
- Setpoint: Position (x, y, z) in meters
- Output: Velocity commands
- Use case: GPS-like waypoint navigation

### Velocity Control (MODE_VELOCITY)
- Setpoint: Velocity (vx, vy, vz) in m/s
- Output: Attitude commands (roll, pitch) and thrust
- Use case: Smooth trajectory following

### Attitude Control (MODE_ABS)
- Setpoint: Attitude (roll, pitch, yaw) in degrees
- Output: Rate commands
- Use case: Stabilized flight

### Rate Control (MODE_VELOCITY)
- Setpoint: Angular rates (deg/s)
- Output: Motor commands
- Use case: Acrobatic/manual flight

### Manual Control (MODE_DISABLE)
- Direct thrust and rate commands
- No PID control
- Use case: Low-level testing

## Testing

### Automated Tests (test_controller.py)
1. **Initialization Test**: Verifies controller setup
2. **Hover Control Test**: Basic altitude hold
3. **Position Control Test**: XYZ position tracking
4. **Velocity Control Test**: Velocity following
5. **Rate Control Test**: Angular rate control
6. **Logging Test**: Data access
7. **Zero Thrust Reset Test**: Safety shutdown

All tests pass successfully.

### Example Scenarios (example_usage.py)
1. Hovering at fixed height
2. Circular trajectory tracking
3. Forward velocity control
4. Manual rate control
5. Logging data access

## Usage Example

```python
from controller import (
    ControllerPID,
    Setpoint, State, Control, SensorData,
    Position, Velocity, Attitude,
    SetpointMode, StabMode
)

# Initialize
controller = ControllerPID()
controller.init()

# Create inputs
state = State()
state.position = Position(x=0.0, y=0.0, z=0.5)
state.velocity = Velocity(x=0.0, y=0.0, z=0.0)
state.attitude = Attitude(roll=0.0, pitch=0.0, yaw=0.0)

sensors = SensorData()
sensors.gyro.x = 0.0  # deg/s
sensors.acc.z = 1.0   # g

setpoint = Setpoint()
setpoint.position = Position(x=0.0, y=0.0, z=1.0)
setpoint.mode.x = StabMode.MODE_ABS
setpoint.mode.y = StabMode.MODE_ABS
setpoint.mode.z = StabMode.MODE_ABS

control = Control()

# Run controller
controller.controller_pid(control, setpoint, sensors, state, step=0)

# Use outputs
print(f"Thrust: {control.thrust}")
print(f"Roll: {control.roll}")
print(f"Pitch: {control.pitch}")
print(f"Yaw: {control.yaw}")
```

## Performance Considerations

### C Firmware
- Runs on embedded STM32 microcontroller
- Real-time constraints (500 Hz)
- Memory: ~100 KB RAM
- Optimized for speed

### Python Implementation
- Runs on desktop/single-board computers
- Less strict timing constraints
- Memory: Depends on Python interpreter
- Optimized for readability

**Note**: For real-time drone control, the C firmware should be used on the Crazyflie hardware. This Python implementation is ideal for:
- Simulation
- Algorithm development
- Testing
- Education
- Integration with ROS/Python tools

## Validation

The Python implementation was validated against the C firmware by:
1. Matching parameter values exactly
2. Replicating all control algorithms
3. Testing with identical inputs
4. Verifying output ranges
5. Comparing control behavior

## Default Parameters

All default parameters match the Crazyflie firmware:

### Attitude Rate PIDs
- Roll rate: KP=70, KI=0, KD=0
- Pitch rate: KP=70, KI=0, KD=0
- Yaw rate: KP=70, KI=16.7, KD=0

### Attitude PIDs
- Roll: KP=6, KI=3, KD=0
- Pitch: KP=6, KI=3, KD=0
- Yaw: KP=6, KI=1, KD=0.35

### Velocity PIDs
- X velocity: KP=25, KI=1, KD=0
- Y velocity: KP=25, KI=1, KD=0
- Z velocity: KP=25, KI=15, KD=0

### Position PIDs
- X position: KP=2, KI=0, KD=0
- Y position: KP=2, KI=0, KD=0
- Z position: KP=2, KI=0.5, KD=0

## Future Enhancements

Potential improvements:
1. Add nonlinear control modes (mellinger controller)
2. Implement parameter adaptation
3. Add disturbance observer
4. Support for custom PID tuning files
5. Real-time plotting of control signals
6. Integration with Gazebo/other simulators
7. SITL (Software In The Loop) support

## References

- [Crazyflie Firmware Repository](https://github.com/bitcraze/crazyflie-firmware)
- [Bitcraze Documentation](https://www.bitcraze.io/documentation/)
- Original files:
  - `src/modules/src/controller_pid.c`
  - `src/modules/src/attitude_pid_controller.c`
  - `src/modules/src/position_controller_pid.c`
  - `src/modules/interface/stabilizer_types.h`
  - `src/modules/interface/platform_defaults.h`

## License

This Python implementation maintains the same GPL v3 license as the original Crazyflie firmware.

Copyright (C) 2011-2024 Bitcraze AB

## Conclusion

This conversion successfully translates the Crazyflie PID controller from C to Python while maintaining:
- ✓ Functional equivalence
- ✓ Parameter compatibility
- ✓ Algorithm accuracy
- ✓ Control structure
- ✓ Documentation completeness

The Python implementation provides a readable, maintainable, and testable version of the controller suitable for simulation, development, and educational purposes.