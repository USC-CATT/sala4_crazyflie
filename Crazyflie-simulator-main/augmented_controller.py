import numpy as np

import controller
import disturbance
import trajGen3D

# Centralized augmented-controller tuning parameters.
MU = 0.01
V_LIMIT_XY = 10.0
START_TIME = disturbance.START_TIME
ENABLED = True


class AugmentedController:
    """Least-squares augmentation on top of baseline cascaded PID.

    The baseline PID output is used as-is, but we modify desired translational
    acceleration by adding a virtual input v on x/y axes:
        a_cmd_aug = a_cmd_nom + v

    Each axis uses a 2-state digital twin:
        x_i = [p_i, v_i]^T
        xhat_dot = A x + B u_nom,   A=[[0,1],[0,0]], B=[[0],[1]]
    and filtered model error:
        e = x - xhat
        e_f_dot = (e_dot_est - e_f)/mu
        v = - (B^T B)^(-1) B^T e_f = -e_f[1]
        v is added to the desired acceleration from the baseline controller, which is then converted to attitude/thrust commands as usual.
    """

    def __init__(
        self,
        dt=1.0 / 500.0,
        mu=None, # filter constant for e_f_dot, smaller -> more aggressive v but noisier
        v_limit_xy=None, # limit for virtual input v on x/y axes (m/s^2 equivalent correction)
        start_time=None,
        enabled=None,
    ):
        self.dt = float(dt)
        self.mu = float(MU if mu is None else mu)
        self.v_limit_xy = float(V_LIMIT_XY if v_limit_xy is None else v_limit_xy)
        self.start_time = float(START_TIME if start_time is None else start_time)
        self.enabled = bool(ENABLED if enabled is None else enabled)

        self._xhat = np.zeros((2, 2), dtype=float)    # rows: axis x/y, cols: [pos, vel]
        self._e_prev = np.zeros((2, 2), dtype=float)
        self._ef = np.zeros((2, 2), dtype=float)
        self._initialized = False

        self._last_v_aug = np.zeros(2, dtype=float)
        self._last_acc_aug = np.zeros(3, dtype=float)

    def set_dt(self, dt):
        self.dt = float(dt)
        controller.set_dt(dt)

    def reset(self):
        self._xhat.fill(0.0)
        self._e_prev.fill(0.0)
        self._ef.fill(0.0)
        self._initialized = False
        self._last_v_aug.fill(0.0)
        self._last_acc_aug.fill(0.0)
        controller.reset()

    def _axis_virtual_input(self, axis, pos, vel, u_nom):
        dt = max(self.dt, 1e-6)
        mu = max(self.mu, 1e-6)

        x = np.array([pos, vel], dtype=float)
        # Digital twin propagation using measured state:
        # xhat_dot = A*x + B*u_nom = [vel, u_nom]^T
        self._xhat[axis, 0] += dt * x[1]
        self._xhat[axis, 1] += dt * u_nom
        e = x - self._xhat[axis]

        if self._initialized:
            e_dot_est = (e - self._e_prev[axis]) / dt
        else:
            e_dot_est = np.zeros(2, dtype=float)

        # Stable discrete update for: e_f_dot = (e_dot_est - e_f) / mu
        # Using exact first-order hold gain avoids Euler blow-up when mu << dt.
        alpha = 1.0 - np.exp(-dt / mu)
        self._ef[axis] += alpha * (e_dot_est - self._ef[axis])
        self._e_prev[axis] = e

        # LS solution with B=[0,1]^T -> v = -e_f[1].
        v = -self._ef[axis, 1]
        #print(f"virtual input (axis {axis}): {v:.3f}, e_f: {self._ef[axis]}, e_dot_est: {e_dot_est}, e: {e}")
        v = float(np.clip(v, -self.v_limit_xy, self.v_limit_xy))
        return v

    def run(self, quad, des_state, sim_time=0.0):
        # Keep baseline behavior when disabled.
        if not self.enabled:
            self._initialized = True
            self._last_v_aug[:] = 0.0
            self._last_acc_aug = np.array(des_state.acc, dtype=float)
            return controller.run(quad, des_state)

        pos = quad.position()
        vel = quad.velocity()

        acc_nom = np.array(des_state.acc, dtype=float).copy()
        vx_est = self._axis_virtual_input(axis=0, pos=pos[0], vel=vel[0], u_nom=acc_nom[0])
        vy_est = self._axis_virtual_input(axis=1, pos=pos[1], vel=vel[1], u_nom=acc_nom[1])
        self._initialized = True

        if sim_time < self.start_time:
            vx, vy = 0.0, 0.0
        else:
            vx, vy = vx_est, vy_est

        acc_aug = acc_nom.copy()
        acc_aug[0] += vx
        acc_aug[1] += vy

        self._last_v_aug[:] = np.array([vx, vy], dtype=float)
        self._last_acc_aug = acc_aug

        des_state_aug = trajGen3D.DesiredState(
            pos=des_state.pos,
            vel=des_state.vel,
            acc=acc_aug,
            yaw=des_state.yaw,
            yawdot=des_state.yawdot,
        )
        return controller.run(quad, des_state_aug)

    def diagnostics(self):
        return {
            "v_aug_xy": self._last_v_aug.copy(),
            "acc_aug": self._last_acc_aug.copy(),
            "ef_x": self._ef[0].copy(),
            "ef_y": self._ef[1].copy(),
        }
