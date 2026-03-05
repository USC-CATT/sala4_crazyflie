import numpy as np

# Crazyflie 2.1 Brushless (44 g all-up)

mass = 0.044 # kg
g = 9.81 # m/s/s
# Inertia is an approximation (Source: Brunke et al. (2025), "Nonlinear System Identification Nano-drone Benchmark" (University of Toronto / UTIAS))

I = np.array([(2.39e-5, 0, 0),
              (0, 2.39e-5, 0),
              (0, 0, 3.23e-5)])

invI = np.linalg.inv(I)
arm_length = 0.05 # meter center-to-motor (100 mm motor-to-motor diagonal)
height = 0.03
minF = 0.0 # minimum total thrust
# Max thrust from 4 brushless motors (30 g thrust each at full throttle).
max_motor_thrust_g = 30.0
maxF = 4.0 * (max_motor_thrust_g / 1000.0) * g
minF_per_motor = minF / 4.0
maxF_per_motor = maxF / 4.0

# Crazyflie firmware-style force/torque mixer parameters.
# Effective arm along roll/pitch axes for X quad layout.
mixer_arm = np.sqrt(0.5) * arm_length
# tau_z = thrust_to_torque * F for each rotor; this starts from the legacy model ratio.
thrust_to_torque = 1.5e-9 / 6.11e-8

# First-order motor/ESC dynamics (seconds).
# Separate rise/fall time constants capture spool asymmetry.
motor_tau_up = 0.025
motor_tau_down = 0.04

# Simple aerodynamic damping terms.
# Translational drag is in world frame: F_drag = -(c1*v + c2*|v|*v)
linear_drag = np.array([0.09, 0.09, 0.14])   # N / (m/s)
quadratic_drag = np.array([0.025, 0.025, 0.035])  # N / (m/s)^2

# Rotational damping torque: tau_damp = -c_omega * omega
angular_damping = np.array([2.0e-6, 2.0e-6, 3.0e-6])  # N*m / (rad/s)

L = arm_length
H = height

# X-layout motor positions (m1..m4) used by plotting.
xy = arm_length / np.sqrt(2.0)
body_frame = np.array([(-xy, -xy, 0, 1),
                       ( xy, -xy, 0, 1),
                       ( xy,  xy, 0, 1),
                       (-xy,  xy, 0, 1),
                       (0, 0, 0, 1),
                       (0, 0, H, 1)])
