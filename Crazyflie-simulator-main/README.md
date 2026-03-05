# Crazyflie Simulator

This repository contains a Python-based quadrotor simulator configured around a Crazyflie 2.1-style platform with:

- 6-DoF rigid-body dynamics
- cascaded PID flight control
- augmented disturbance-cancellation control (can be turned on or off)
- waypoint/trajectory tracking
- hover-under-wind disturbance testing
- live 3D visualization

## What Is Implemented

- Dynamics (`model/quadcopter.py`)
  - Quaternion-based rigid-body state propagation
  - Body wrench to motor thrust mixing and inverse mapping
  - First-order motor dynamics (`motor_tau_up`, `motor_tau_down`)
  - Translational drag + angular damping
  - Ground plane constraint (`z >= 0`)

- Vehicle parameters (`model/params.py`)
  - Mass, inertia, geometry, thrust limits
  - Mixer geometry (`mixer_arm`, `thrust_to_torque`)
  - Drag/damping constants

- Baseline controller (`controller.py`)
  - Firmware-style cascaded PID structure:
    - Position -> Velocity -> Acceleration -> Attitude -> Rate
  - Inner loop at 500 Hz, outer loop at 100 Hz
  - Integrator limits and practical saturation limits

- Augmented controller (`augmented_controller.py`)
  - Optional least-squares virtual input on XY channels
  - Uses digital-twin error filtering and adds correction to commanded acceleration
  - Tunable parameters at top of file:
    - `MU`
    - `V_LIMIT_XY`
    - `START_TIME`
    - `ENABLED`

- Trajectory generation (`trajGen3D.py`)
  - Minimum-snap polynomial trajectory through waypoints
  - Waypoints can be `Nx3` (`x,y,z`) or `Nx4` (`x,y,z,yaw`)
  - Yaw is explicitly interpolated from waypoint yaw values

- Disturbance model (`disturbance.py`)
  - Lateral sinusoidal wind force in world frame
  - User controls start time, amplitude, frequency, phase, direction

- Visualization (`quadPlot.py`)
  - 3D vehicle + path + waypoints
  - Wind force time plot
  - Status text: position, roll/pitch/yaw, controller mode, position-error norm
  - Save animation as MP4/GIF via CLI argument

## How to run

- `runsim.py` is the primary script.
- Configure simulation/controller modes and key parameters at the top of `runsim.py`.

Important switches:

- `controller_mode = "baseline" | "augmented"`
- `sim_mode = "trajectory" | "hover_wind"`
- `waypoint_source = "helix" | "list" | "csv"`

## Installation

```bash
python -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

Or use your existing conda env and run:

```bash
pip install -r requirements.txt
```

## Run

Interactive simulation:

```bash
python runsim.py
```

Save animation (no GUI):

```bash
python runsim.py save mp4
python runsim.py save gif
```

Notes:

- MP4 requires `ffmpeg` available to Matplotlib.
- If `ffmpeg` is unavailable, GIF fallback is used.
- Install ffmpeg (conda):

```bash
conda install -n base -c conda-forge ffmpeg
```

## Typical Workflow

1. Set `sim_mode` and `controller_mode` in `runsim.py`.
2. For trajectory tracking, define waypoints (`USER_WAYPOINTS`) or CSV in `runsim.py`.
3. For disturbance tests, tune wind in `disturbance.py`.
4. If using augmentation, tune `MU`/`V_LIMIT_XY` in `augmented_controller.py`.
5. Run and compare `|position error|` behavior between modes.

## TODO: Realism Upgrades

- [ ] Add timing imperfections: sensor delay, control delay, jitter, packet dropouts.
- [ ] Improve motor/propulsion model using RPM/voltage relationships and battery sag.
- [ ] Add near-ground and aerodynamic effects (ground effect, stronger coupling under aggressive motion).
- [ ] Upgrade disturbance model from pure sinusoid to gust/turbulence profiles.
- [ ] Add parameter uncertainty + Monte Carlo runs (mass/inertia/drag/CG variations).
- [ ] Replicate firmware filtering/scheduling more exactly.
- [ ] Add richer actuator nonlinearities and anti-windup behavior.
- [ ] Add contact/collision model for landing/touch events.
- [ ] Build validation harness against real logs (rise time, overshoot, RMS tracking, disturbance rejection).
