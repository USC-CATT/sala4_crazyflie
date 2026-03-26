"""
Example usage of the Crazyflie PID controller in Python

This demonstrates how to use the PID controller classes and functions.
"""

import math
import time

from .controller_pid import ControllerPID
from .controller_types import (
    AccData,
    Attitude,
    AttitudeRate,
    Control,
    GyroData,
    Position,
    SensorData,
    Setpoint,
    SetpointMode,
    StabMode,
    State,
    Velocity,
)


def example_hover():
    """Example: Hover at 1 meter height"""
    print("=" * 60)
    print("Example 1: Hovering at 1 meter")
    print("=" * 60)

    # Initialize controller
    controller = ControllerPID()
    controller.init()

    # Verify initialization
    if not controller.test():
        print("ERROR: Controller failed to initialize!")
        return

    print("Controller initialized successfully")

    # Create state (current drone state)
    state = State()
    state.position = Position(x=0.0, y=0.0, z=0.5)  # Start at 0.5m height
    state.velocity = Velocity(x=0.0, y=0.0, z=0.0)
    state.attitude = Attitude(roll=0.0, pitch=0.0, yaw=0.0)

    # Create sensor data
    sensors = SensorData()
    sensors.gyro = GyroData(x=0.0, y=0.0, z=0.0)  # deg/s
    sensors.acc = AccData(x=0.0, y=0.0, z=1.0)  # 1g in z-axis

    # Create setpoint (desired state)
    setpoint = Setpoint()
    setpoint.position = Position(x=0.0, y=0.0, z=1.0)  # Hover at 1m
    setpoint.velocity = Velocity(x=0.0, y=0.0, z=0.0)
    setpoint.attitude = Attitude(roll=0.0, pitch=0.0, yaw=0.0)
    setpoint.attitude_rate = AttitudeRate(roll=0.0, pitch=0.0, yaw=0.0)
    setpoint.thrust = 0.0
    setpoint.velocity_body = False

    # Set control modes
    setpoint.mode = SetpointMode()
    setpoint.mode.x = StabMode.MODE_ABS  # Position control
    setpoint.mode.y = StabMode.MODE_ABS  # Position control
    setpoint.mode.z = StabMode.MODE_ABS  # Position control
    setpoint.mode.roll = StabMode.MODE_DISABLE
    setpoint.mode.pitch = StabMode.MODE_DISABLE
    setpoint.mode.yaw = StabMode.MODE_ABS

    # Create control output
    control = Control()

    # Simulate controller updates
    print("\nRunning controller for 10 steps...")
    print(f"{'Step':<6} {'Thrust':<10} {'Roll':<10} {'Pitch':<10} {'Yaw':<10}")
    print("-" * 50)

    for step in range(10):
        # Run controller
        controller.controller_pid(control, setpoint, sensors, state, step)

        # Print results
        print(
            f"{step:<6} {control.thrust:<10.1f} {control.roll:<10.1f} "
            f"{control.pitch:<10.1f} {control.yaw:<10.1f}"
        )

        # Simulate state update (simplified physics)
        # In real scenario, this would come from state estimator
        if step < 5:
            state.position.z += 0.1  # Rising

    print("\n✓ Hover example completed\n")


def example_position_tracking():
    """Example: Track a circular trajectory"""
    print("=" * 60)
    print("Example 2: Circular trajectory tracking")
    print("=" * 60)

    # Initialize controller
    controller = ControllerPID()
    controller.init()

    print("Controller initialized successfully")

    # Initial state
    state = State()
    state.position = Position(x=0.0, y=0.0, z=1.0)
    state.velocity = Velocity(x=0.0, y=0.0, z=0.0)
    state.attitude = Attitude(roll=0.0, pitch=0.0, yaw=0.0)

    sensors = SensorData()
    sensors.gyro = GyroData(x=0.0, y=0.0, z=0.0)
    sensors.acc = AccData(x=0.0, y=0.0, z=1.0)

    setpoint = Setpoint()
    setpoint.velocity_body = False
    setpoint.mode = SetpointMode()
    setpoint.mode.x = StabMode.MODE_ABS
    setpoint.mode.y = StabMode.MODE_ABS
    setpoint.mode.z = StabMode.MODE_ABS
    setpoint.mode.yaw = StabMode.MODE_ABS

    control = Control()

    # Circular trajectory parameters
    radius = 0.5  # meters
    angular_velocity = 0.1  # rad/s
    center_height = 1.0  # meters

    print("\nTracking circular path...")
    print(f"{'Step':<6} {'X_des':<10} {'Y_des':<10} {'Z_des':<10} {'Thrust':<10}")
    print("-" * 56)

    for step in range(20):
        # Calculate circular trajectory setpoint
        angle = angular_velocity * step * 0.1  # time = step * dt
        setpoint.position.x = radius * math.cos(angle)
        setpoint.position.y = radius * math.sin(angle)
        setpoint.position.z = center_height

        # Run controller
        controller.controller_pid(control, setpoint, sensors, state, step)

        # Print results
        print(
            f"{step:<6} {setpoint.position.x:<10.3f} {setpoint.position.y:<10.3f} "
            f"{setpoint.position.z:<10.3f} {control.thrust:<10.1f}"
        )

        # Simulate gradual convergence to setpoint (simplified)
        state.position.x += (setpoint.position.x - state.position.x) * 0.1
        state.position.y += (setpoint.position.y - state.position.y) * 0.1
        state.position.z += (setpoint.position.z - state.position.z) * 0.1

    print("\n✓ Trajectory tracking example completed\n")


def example_velocity_control():
    """Example: Velocity control mode"""
    print("=" * 60)
    print("Example 3: Velocity control mode")
    print("=" * 60)

    # Initialize controller
    controller = ControllerPID()
    controller.init()

    print("Controller initialized successfully")

    # Initial state
    state = State()
    state.position = Position(x=0.0, y=0.0, z=1.0)
    state.velocity = Velocity(x=0.0, y=0.0, z=0.0)
    state.attitude = Attitude(roll=0.0, pitch=0.0, yaw=0.0)

    sensors = SensorData()
    sensors.gyro = GyroData(x=0.0, y=0.0, z=0.0)
    sensors.acc = AccData(x=0.0, y=0.0, z=1.0)

    setpoint = Setpoint()
    setpoint.velocity = Velocity(x=0.5, y=0.0, z=0.0)  # Move forward at 0.5 m/s
    setpoint.position = Position(x=0.0, y=0.0, z=1.0)
    setpoint.velocity_body = False
    setpoint.mode = SetpointMode()
    setpoint.mode.x = StabMode.MODE_VELOCITY  # Velocity control
    setpoint.mode.y = StabMode.MODE_VELOCITY  # Velocity control
    setpoint.mode.z = StabMode.MODE_ABS  # Position control for height
    setpoint.mode.yaw = StabMode.MODE_ABS

    control = Control()

    print("\nVelocity control: Moving forward at 0.5 m/s")
    print(f"{'Step':<6} {'Vx_des':<10} {'Roll':<12} {'Pitch':<12} {'Thrust':<10}")
    print("-" * 56)

    for step in range(15):
        # Run controller
        controller.controller_pid(control, setpoint, sensors, state, step)

        # Print results
        print(
            f"{step:<6} {setpoint.velocity.x:<10.2f} {control.roll:<12.2f} "
            f"{control.pitch:<12.2f} {control.thrust:<10.1f}"
        )

        # Simulate velocity following setpoint
        state.velocity.x += (setpoint.velocity.x - state.velocity.x) * 0.15
        state.position.x += state.velocity.x * 0.1  # dt = 0.1s

    print("\n✓ Velocity control example completed\n")


def example_rate_control():
    """Example: Rate control (angular velocity)"""
    print("=" * 60)
    print("Example 4: Rate control mode (manual flying)")
    print("=" * 60)

    # Initialize controller
    controller = ControllerPID()
    controller.init()

    print("Controller initialized successfully")

    # Initial state
    state = State()
    state.position = Position(x=0.0, y=0.0, z=1.0)
    state.velocity = Velocity(x=0.0, y=0.0, z=0.0)
    state.attitude = Attitude(roll=0.0, pitch=0.0, yaw=0.0)

    sensors = SensorData()
    sensors.gyro = GyroData(x=0.0, y=0.0, z=0.0)
    sensors.acc = AccData(x=0.0, y=0.0, z=1.0)

    setpoint = Setpoint()
    setpoint.thrust = 40000  # Manual thrust
    setpoint.attitude_rate = AttitudeRate(roll=30.0, pitch=0.0, yaw=10.0)  # deg/s
    setpoint.velocity_body = True
    setpoint.mode = SetpointMode()
    setpoint.mode.x = StabMode.MODE_DISABLE
    setpoint.mode.y = StabMode.MODE_DISABLE
    setpoint.mode.z = StabMode.MODE_DISABLE  # Manual thrust
    setpoint.mode.roll = StabMode.MODE_VELOCITY  # Rate control
    setpoint.mode.pitch = StabMode.MODE_VELOCITY  # Rate control
    setpoint.mode.yaw = StabMode.MODE_VELOCITY  # Rate control

    control = Control()

    print("\nRate control: Roll 30°/s, Yaw 10°/s")
    print(f"{'Step':<6} {'Thrust':<12} {'Roll_out':<12} {'Yaw_out':<12}")
    print("-" * 48)

    for step in range(10):
        # Run controller
        controller.controller_pid(control, setpoint, sensors, state, step)

        # Print results
        print(
            f"{step:<6} {control.thrust:<12.1f} {control.roll:<12.1f} "
            f"{control.yaw:<12.1f}"
        )

        # Simulate gyro readings following commanded rates
        sensors.gyro.x += (setpoint.attitude_rate.roll - sensors.gyro.x) * 0.2
        sensors.gyro.z += (setpoint.attitude_rate.yaw - sensors.gyro.z) * 0.2

        # Update attitude
        state.attitude.roll += sensors.gyro.x * 0.002  # dt = 0.002s
        state.attitude.yaw += sensors.gyro.z * 0.002

    print("\n✓ Rate control example completed\n")


def example_logging():
    """Example: Accessing logging data"""
    print("=" * 60)
    print("Example 5: Logging and telemetry data")
    print("=" * 60)

    # Initialize controller
    controller = ControllerPID()
    controller.init()

    print("Controller initialized successfully")

    # Setup simple hover scenario
    state = State()
    state.position = Position(x=0.0, y=0.0, z=0.8)
    state.velocity = Velocity(x=0.0, y=0.0, z=0.0)
    state.attitude = Attitude(roll=2.0, pitch=-1.5, yaw=45.0)

    sensors = SensorData()
    sensors.gyro = GyroData(x=5.0, y=-3.0, z=2.0)
    sensors.acc = AccData(x=0.1, y=-0.05, z=1.0)

    setpoint = Setpoint()
    setpoint.position = Position(x=0.0, y=0.0, z=1.0)
    setpoint.mode = SetpointMode()
    setpoint.mode.x = StabMode.MODE_ABS
    setpoint.mode.y = StabMode.MODE_ABS
    setpoint.mode.z = StabMode.MODE_ABS
    setpoint.mode.yaw = StabMode.MODE_ABS

    control = Control()

    # Run controller
    controller.controller_pid(control, setpoint, sensors, state, 0)

    # Get logging data
    log_data = controller.get_logging_data()

    print("\nController outputs:")
    print(f"  Thrust command:  {log_data['cmd_thrust']:.2f}")
    print(f"  Roll command:    {log_data['cmd_roll']:.2f}")
    print(f"  Pitch command:   {log_data['cmd_pitch']:.2f}")
    print(f"  Yaw command:     {log_data['cmd_yaw']:.2f}")

    print("\nDesired attitude:")
    print(f"  Roll:   {log_data['attitude_desired_roll']:.2f}°")
    print(f"  Pitch:  {log_data['attitude_desired_pitch']:.2f}°")
    print(f"  Yaw:    {log_data['attitude_desired_yaw']:.2f}°")

    print("\nDesired rates:")
    print(f"  Roll rate:   {log_data['rate_desired_roll']:.2f}°/s")
    print(f"  Pitch rate:  {log_data['rate_desired_pitch']:.2f}°/s")
    print(f"  Yaw rate:    {log_data['rate_desired_yaw']:.2f}°/s")

    print("\nSensor readings (rad/s):")
    print(f"  Roll rate:   {log_data['r_roll']:.4f}")
    print(f"  Pitch rate:  {log_data['r_pitch']:.4f}")
    print(f"  Yaw rate:    {log_data['r_yaw']:.4f}")
    print(f"  Accel Z:     {log_data['accelz']:.2f}g")

    print("\n✓ Logging example completed\n")


def main():
    """Run all examples"""
    print("\n" + "=" * 60)
    print("Crazyflie PID Controller - Python Implementation Examples")
    print("=" * 60 + "\n")

    try:
        example_hover()
        time.sleep(0.5)

        example_position_tracking()
        time.sleep(0.5)

        example_velocity_control()
        time.sleep(0.5)

        example_rate_control()
        time.sleep(0.5)

        example_logging()

        print("=" * 60)
        print("All examples completed successfully!")
        print("=" * 60 + "\n")

    except Exception as e:
        print(f"\n❌ Error running examples: {e}")
        import traceback

        traceback.print_exc()


if __name__ == "__main__":
    main()
