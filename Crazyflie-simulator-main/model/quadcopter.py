import numpy as np
import scipy.integrate as integrate
from utils.quaternion import Quaternion
from utils.utils import RPYToRot, RotToQuat, RotToRPY
import model.params as params

class Quadcopter:
    """ Quadcopter class

    state  - 1 dimensional vector but used as 13 x 1. [x, y, z, xd, yd, zd, qw, qx, qy, qz, p, q, r]
             where [qw, qx, qy, qz] is quternion and [p, q, r] are angular velocity [roll_dot, pitch_dot, yaw_dot]
    F      - 1 x 1, thrust output from controller
    M      - 3 x 1, moments output from controller
    params - system parameters struct, arm_length, g, mass, etc.
    """

    def __init__(self, pos, attitude):
        """ pos = [x,y,z] attitude = [rool,pitch,yaw]
            """
        self.state = np.zeros(13)
        roll, pitch, yaw = attitude
        rot    = RPYToRot(roll, pitch, yaw)
        quat   = RotToQuat(rot)
        self.state[0] = pos[0]
        self.state[1] = pos[1]
        self.state[2] = max(0.0, pos[2])
        self.state[6] = quat[0]
        self.state[7] = quat[1]
        self.state[8] = quat[2]
        self.state[9] = quat[3]
        # Start motors close to hover thrust to avoid an unrealistic takeoff transient.
        hover_per_motor = params.mass * params.g / 4.0
        self.motor_thrusts = np.full(4, hover_per_motor)

    def world_frame(self):
        """ position returns a 3x6 matrix
            where row is [x, y, z] column is m1 m2 m3 m4 origin h
            """
        origin = self.state[0:3]
        quat = Quaternion(self.state[6:10])
        rot = quat.as_rotation_matrix()
        wHb = np.r_[np.c_[rot,origin], np.array([[0, 0, 0, 1]])]
        quadBodyFrame = params.body_frame.T
        quadWorldFrame = wHb.dot(quadBodyFrame)
        world_frame = quadWorldFrame[0:3]
        return world_frame

    def position(self):
        return self.state[0:3]

    def velocity(self):
        return self.state[3:6]

    def attitude(self):
        rot = Quaternion(self.state[6:10]).as_rotation_matrix()
        return RotToRPY(rot)

    def omega(self):
        return self.state[10:13]

    def _mix_force_moment_to_motors(self, F, M):
        """Map desired total force/moment to per-motor thrust commands."""
        Mx, My, Mz = M.flatten()
        thrust_part = 0.25 * F
        roll_part = 0.25 * Mx / params.mixer_arm
        pitch_part = 0.25 * My / params.mixer_arm
        yaw_part = 0.25 * Mz / params.thrust_to_torque

        prop_thrusts = np.array([
            thrust_part - roll_part - pitch_part - yaw_part,
            thrust_part - roll_part + pitch_part + yaw_part,
            thrust_part + roll_part + pitch_part - yaw_part,
            thrust_part + roll_part - pitch_part + yaw_part
        ])

        # Crazyflie-style cap: if one motor is too high, shift all down equally.
        max_thrust = np.max(prop_thrusts)
        reduction = max(0.0, max_thrust - params.maxF_per_motor)
        prop_thrusts = prop_thrusts - reduction
        prop_thrusts = np.clip(prop_thrusts, params.minF_per_motor, params.maxF_per_motor)
        return prop_thrusts

    def _motors_to_wrench(self, prop_thrusts):
        f1, f2, f3, f4 = prop_thrusts
        F = np.sum(prop_thrusts)
        M = np.array([
            params.mixer_arm * (f3 + f4 - f1 - f2),
            params.mixer_arm * (f2 + f3 - f1 - f4),
            params.thrust_to_torque * (-f1 + f2 - f3 + f4)
        ]).reshape(3, 1)
        return F, M

    def state_dot(self, state, t, F, M, external_force_world):
        x, y, z, xdot, ydot, zdot, qw, qx, qy, qz, p, q, r = state
        quat = np.array([qw,qx,qy,qz])

        bRw = Quaternion(quat).as_rotation_matrix() # world to body rotation matrix
        wRb = bRw.T # orthogonal matrix inverse = transpose
        # acceleration - Newton's second law of motion
        thrust_world = wRb.dot(np.array([0.0, 0.0, F]))
        vel_world = np.array([xdot, ydot, zdot])
        drag_world = params.linear_drag * vel_world + params.quadratic_drag * np.abs(vel_world) * vel_world
        accel = (thrust_world + external_force_world - np.array([0.0, 0.0, params.mass * params.g]) - drag_world) / params.mass
        # angular velocity - using quternion
        # http://www.euclideanspace.com/physics/kinematics/angularvelocity/
        K_quat = 2.0; # this enforces the magnitude 1 constraint for the quaternion
        quaterror = 1.0 - (qw**2 + qx**2 + qy**2 + qz**2)
        qdot = (-1.0/2) * np.array([[0, -p, -q, -r],
                                    [p,  0, -r,  q],
                                    [q,  r,  0, -p],
                                    [r, -q,  p,  0]]).dot(quat) + K_quat * quaterror * quat;

        # angular acceleration - Euler's equation of motion
        # https://en.wikipedia.org/wiki/Euler%27s_equations_(rigid_body_dynamics)
        omega = np.array([p,q,r])
        damping_torque = params.angular_damping * omega
        pqrdot = params.invI.dot(M.flatten() - np.cross(omega, params.I.dot(omega)) - damping_torque)
        state_dot = np.zeros(13)
        state_dot[0]  = xdot
        state_dot[1]  = ydot
        state_dot[2]  = zdot
        state_dot[3]  = accel[0]
        state_dot[4]  = accel[1]
        state_dot[5]  = accel[2]
        state_dot[6]  = qdot[0]
        state_dot[7]  = qdot[1]
        state_dot[8]  = qdot[2]
        state_dot[9]  = qdot[3]
        state_dot[10] = pqrdot[0]
        state_dot[11] = pqrdot[1]
        state_dot[12] = pqrdot[2]

        return state_dot

    def update(self, dt, F, M, external_force_world=None):
        # Desired per-motor thrusts from mixer.
        desired_motor_thrusts = self._mix_force_moment_to_motors(F, M)
        if external_force_world is None:
            external_force_world = np.zeros(3)
        external_force_world = np.asarray(external_force_world, dtype=float).reshape(3,)

        # First-order motor dynamics (rise/fall asymmetric).
        taus = np.where(
            desired_motor_thrusts >= self.motor_thrusts,
            params.motor_tau_up,
            params.motor_tau_down
        )
        alpha = np.clip(dt / np.maximum(taus, 1e-6), 0.0, 1.0)
        self.motor_thrusts = self.motor_thrusts + alpha * (desired_motor_thrusts - self.motor_thrusts)
        self.motor_thrusts = np.clip(self.motor_thrusts, params.minF_per_motor, params.maxF_per_motor)

        # Realized force/moment after actuator dynamics.
        F_realized, M_realized = self._motors_to_wrench(self.motor_thrusts)
        self.state = integrate.odeint(
            self.state_dot,
            self.state,
            [0, dt],
            args=(F_realized, M_realized, external_force_world)
        )[1]

        # Ground contact model: z=0 is an impenetrable floor.
        if self.state[2] < 0.0:
            self.state[2] = 0.0
        if self.state[2] <= 0.0 and self.state[5] < 0.0:
            self.state[5] = 0.0
