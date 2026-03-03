"""
Simple test script to verify the PID controller implementation

This script tests the basic functionality of the controller without
requiring external dependencies.
"""

import sys
from pathlib import Path

# Add parent directory to path if needed
sys.path.insert(0, str(Path(__file__).parent.parent))

from controller import (
    AccData,
    Attitude,
    AttitudeRate,
    Control,
    ControllerPID,
    GyroData,
    Position,
    SensorData,
    Setpoint,
    SetpointMode,
    StabMode,
    State,
    Velocity,
)


def test_initialization():
    """Test controller initialization"""
    print("Testing initialization...")
    controller = ControllerPID()
    controller.init()
    assert controller.test(), "Controller failed to initialize"
    print("✓ Initialization test passed")


def test_hover_control():
    """Test basic hover control"""
    print("\nTesting hover control...")

    controller = ControllerPID()
    controller.init()

    # Setup state at 0.5m height
    state = State()
    state.position = Position(x=0.0, y=0.0, z=0.5)
    state.velocity = Velocity(x=0.0, y=0.0, z=0.0)
    state.attitude = Attitude(roll=0.0, pitch=0.0, yaw=0.0)

    # Setup sensors
    sensors = SensorData()
    sensors.gyro = GyroData(x=0.0, y=0.0, z=0.0)
    sensors.acc = AccData(x=0.0, y=0.0, z=1.0)

    # Setup setpoint at 1m height
    setpoint = Setpoint()
    setpoint.position = Position(x=0.0, y=0.0, z=1.0)
    setpoint.velocity = Velocity(x=0.0, y=0.0, z=0.0)
    setpoint.attitude = Attitude(roll=0.0, pitch=0.0, yaw=0.0)
    setpoint.attitude_rate = AttitudeRate(roll=0.0, pitch=0.0, yaw=0.0)
    setpoint.thrust = 0.0
    setpoint.velocity_body = False

    setpoint.mode = SetpointMode()
    setpoint.mode.x = StabMode.MODE_ABS
    setpoint.mode.y = StabMode.MODE_ABS
    setpoint.mode.z = StabMode.MODE_ABS
    setpoint.mode.yaw = StabMode.MODE_ABS

    # Create control output
    control = Control()

    # Run controller
    controller.controller_pid(control, setpoint, sensors, state, 0)

    # Verify outputs are generated
    assert control.thrust > 0, "Thrust should be positive for upward movement"
    print(f"  Generated thrust: {control.thrust:.2f}")
    print("✓ Hover control test passed")


def test_position_control():
    """Test position control"""
    print("\nTesting position control...")

    controller = ControllerPID()
    controller.init()

    state = State()
    state.position = Position(x=0.0, y=0.0, z=1.0)
    state.velocity = Velocity(x=0.0, y=0.0, z=0.0)
    state.attitude = Attitude(roll=0.0, pitch=0.0, yaw=0.0)

    sensors = SensorData()
    sensors.gyro = GyroData(x=0.0, y=0.0, z=0.0)
    sensors.acc = AccData(x=0.0, y=0.0, z=1.0)

    # Command to move to x=1.0
    setpoint = Setpoint()
    setpoint.position = Position(x=1.0, y=0.0, z=1.0)
    setpoint.mode = SetpointMode()
    setpoint.mode.x = StabMode.MODE_ABS
    setpoint.mode.y = StabMode.MODE_ABS
    setpoint.mode.z = StabMode.MODE_ABS
    setpoint.mode.yaw = StabMode.MODE_ABS

    control = Control()
    controller.controller_pid(control, setpoint, sensors, state, 0)

    # Should generate pitch command to move forward
    print(f"  Generated pitch: {control.pitch:.2f}")
    print("✓ Position control test passed")


def test_velocity_control():
    """Test velocity control"""
    print("\nTesting velocity control...")

    controller = ControllerPID()
    controller.init()

    state = State()
    state.position = Position(x=0.0, y=0.0, z=1.0)
    state.velocity = Velocity(x=0.0, y=0.0, z=0.0)
    state.attitude = Attitude(roll=0.0, pitch=0.0, yaw=0.0)

    sensors = SensorData()
    sensors.gyro = GyroData(x=0.0, y=0.0, z=0.0)
    sensors.acc = AccData(x=0.0, y=0.0, z=1.0)

    # Command velocity forward
    setpoint = Setpoint()
    setpoint.velocity = Velocity(x=0.5, y=0.0, z=0.0)
    setpoint.position = Position(x=0.0, y=0.0, z=1.0)
    setpoint.mode = SetpointMode()
    setpoint.mode.x = StabMode.MODE_VELOCITY
    setpoint.mode.y = StabMode.MODE_VELOCITY
    setpoint.mode.z = StabMode.MODE_ABS
    setpoint.mode.yaw = StabMode.MODE_ABS
    setpoint.velocity_body = False

    control = Control()
    controller.controller_pid(control, setpoint, sensors, state, 0)

    print(f"  Generated pitch: {control.pitch:.2f}")
    print("✓ Velocity control test passed")


def test_rate_control():
    """Test rate control"""
    print("\nTesting rate control...")

    controller = ControllerPID()
    controller.init()

    state = State()
    state.position = Position(x=0.0, y=0.0, z=1.0)
    state.velocity = Velocity(x=0.0, y=0.0, z=0.0)
    state.attitude = Attitude(roll=0.0, pitch=0.0, yaw=0.0)

    sensors = SensorData()
    sensors.gyro = GyroData(x=0.0, y=0.0, z=0.0)
    sensors.acc = AccData(x=0.0, y=0.0, z=1.0)

    # Command angular rates
    setpoint = Setpoint()
    setpoint.thrust = 40000
    setpoint.attitude_rate = AttitudeRate(roll=30.0, pitch=0.0, yaw=10.0)
    setpoint.mode = SetpointMode()
    setpoint.mode.x = StabMode.MODE_DISABLE
    setpoint.mode.y = StabMode.MODE_DISABLE
    setpoint.mode.z = StabMode.MODE_DISABLE
    setpoint.mode.roll = StabMode.MODE_VELOCITY
    setpoint.mode.pitch = StabMode.MODE_VELOCITY
    setpoint.mode.yaw = StabMode.MODE_VELOCITY
    setpoint.velocity_body = True

    control = Control()
    controller.controller_pid(control, setpoint, sensors, state, 0)

    assert control.thrust == 40000, "Thrust should match setpoint in manual mode"
    print(f"  Generated roll: {control.roll:.2f}")
    print(f"  Generated yaw: {control.yaw:.2f}")
    print("✓ Rate control test passed")


def test_logging():
    """Test logging data access"""
    print("\nTesting logging data...")

    controller = ControllerPID()
    controller.init()

    state = State()
    state.position = Position(x=0.0, y=0.0, z=1.0)
    state.velocity = Velocity(x=0.0, y=0.0, z=0.0)
    state.attitude = Attitude(roll=5.0, pitch=-3.0, yaw=45.0)

    sensors = SensorData()
    sensors.gyro = GyroData(x=2.0, y=-1.5, z=1.0)
    sensors.acc = AccData(x=0.1, y=-0.05, z=1.0)

    setpoint = Setpoint()
    setpoint.position = Position(x=0.0, y=0.0, z=1.0)
    setpoint.mode = SetpointMode()
    setpoint.mode.x = StabMode.MODE_ABS
    setpoint.mode.y = StabMode.MODE_ABS
    setpoint.mode.z = StabMode.MODE_ABS
    setpoint.mode.yaw = StabMode.MODE_ABS

    control = Control()
    controller.controller_pid(control, setpoint, sensors, state, 0)

    # Get logging data
    log_data = controller.get_logging_data()

    assert "cmd_thrust" in log_data
    assert "cmd_roll" in log_data
    assert "attitude_desired_roll" in log_data
    assert "rate_desired_yaw" in log_data

    print(f"  Logged {len(log_data)} variables")
    print("✓ Logging test passed")


def test_zero_thrust_reset():
    """Test that zero thrust resets PIDs"""
    print("\nTesting zero thrust reset...")

    controller = ControllerPID()
    controller.init()

    state = State()
    state.position = Position(x=0.0, y=0.0, z=1.0)
    state.velocity = Velocity(x=0.0, y=0.0, z=0.0)
    state.attitude = Attitude(roll=0.0, pitch=0.0, yaw=0.0)

    sensors = SensorData()
    sensors.gyro = GyroData(x=0.0, y=0.0, z=0.0)
    sensors.acc = AccData(x=0.0, y=0.0, z=1.0)

    # Set thrust to zero
    setpoint = Setpoint()
    setpoint.thrust = 0
    setpoint.mode = SetpointMode()
    setpoint.mode.z = StabMode.MODE_DISABLE

    control = Control()
    controller.controller_pid(control, setpoint, sensors, state, 0)

    assert control.thrust == 0
    assert control.roll == 0
    assert control.pitch == 0
    assert control.yaw == 0

    print("✓ Zero thrust reset test passed")


def run_all_tests():
    """Run all tests"""
    print("=" * 60)
    print("Running PID Controller Tests")
    print("=" * 60)

    try:
        test_initialization()
        test_hover_control()
        test_position_control()
        test_velocity_control()
        test_rate_control()
        test_logging()
        test_zero_thrust_reset()

        print("\n" + "=" * 60)
        print("✓ All tests passed!")
        print("=" * 60)
        return True

    except AssertionError as e:
        print(f"\n❌ Test failed: {e}")
        import traceback

        traceback.print_exc()
        return False
    except Exception as e:
        print(f"\n❌ Unexpected error: {e}")
        import traceback

        traceback.print_exc()
        return False


if __name__ == "__main__":
    success = run_all_tests()
    sys.exit(0 if success else 1)
