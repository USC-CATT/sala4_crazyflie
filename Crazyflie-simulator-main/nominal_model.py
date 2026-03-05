import numpy as np

import model.params as params
from model.quadcopter import Quadcopter
from utils.utils import RPYToRot, RotToQuat

# Full nonlinear simulator state order.
STATE_ORDER = (
    "x", "y", "z",
    "xdot", "ydot", "zdot",
    "qw", "qx", "qy", "qz",
    "p", "q", "r",
)

# Input order for linearization.
INPUT_ORDER = ("F", "Mx", "My", "Mz")


def hover_equilibrium(pos=(0.0, 0.0, 0.0), attitude=(0.0, 0.0, 0.0)):
    """Return equilibrium state/input around hover.

    The nominal input is total thrust equal to weight and zero body moments.
    """
    x_eq = np.zeros(13, dtype=float)
    rot = RPYToRot(*attitude)
    quat = RotToQuat(rot)

    x_eq[0:3] = np.asarray(pos, dtype=float)
    x_eq[6:10] = quat

    u_eq = np.array([params.mass * params.g, 0.0, 0.0, 0.0], dtype=float)
    return x_eq, u_eq


def _dynamics(state, control, external_force_world):
    """Continuous-time nonlinear dynamics x_dot = f(x,u)."""
    # state_dot() is stateless; this helper only gives us access to model equations.
    quad = Quadcopter((0.0, 0.0, 0.0), (0.0, 0.0, 0.0))
    F = float(control[0])
    M = np.asarray(control[1:4], dtype=float).reshape(3, 1)
    return quad.state_dot(
        np.asarray(state, dtype=float),
        0.0,
        F,
        M,
        np.asarray(external_force_world, dtype=float).reshape(3,),
    )


def linearize(state_eq, input_eq, state_eps=1e-6, input_eps=1e-6, external_force_world=None):
    """Numerically linearize x_dot = f(x,u) around (state_eq, input_eq).

    Returns matrices (A, B) of:
        d/dt(delta_x) = A delta_x + B delta_u
    where delta_x = x - state_eq and delta_u = u - input_eq.
    """
    x0 = np.asarray(state_eq, dtype=float).copy()
    u0 = np.asarray(input_eq, dtype=float).copy()
    if x0.shape != (13,):
        raise ValueError("state_eq must have shape (13,).")
    if u0.shape != (4,):
        raise ValueError("input_eq must have shape (4,).")

    if external_force_world is None:
        external_force_world = np.zeros(3, dtype=float)
    else:
        external_force_world = np.asarray(external_force_world, dtype=float).reshape(3,)

    n = x0.size
    m = u0.size
    A = np.zeros((n, n), dtype=float)
    B = np.zeros((n, m), dtype=float)

    state_step = np.full(n, float(state_eps), dtype=float) if np.isscalar(state_eps) else np.asarray(state_eps, dtype=float)
    input_step = np.full(m, float(input_eps), dtype=float) if np.isscalar(input_eps) else np.asarray(input_eps, dtype=float)

    if state_step.shape != (n,):
        raise ValueError("state_eps must be scalar or shape (13,).")
    if input_step.shape != (m,):
        raise ValueError("input_eps must be scalar or shape (4,).")

    # A = df/dx via central difference
    for i in range(n):
        dx = np.zeros(n, dtype=float)
        h = max(abs(state_step[i]), 1e-9)
        dx[i] = h
        f_plus = _dynamics(x0 + dx, u0, external_force_world)
        f_minus = _dynamics(x0 - dx, u0, external_force_world)
        A[:, i] = (f_plus - f_minus) / (2.0 * h)

    # B = df/du via central difference
    for j in range(m):
        du = np.zeros(m, dtype=float)
        h = max(abs(input_step[j]), 1e-9)
        du[j] = h
        f_plus = _dynamics(x0, u0 + du, external_force_world)
        f_minus = _dynamics(x0, u0 - du, external_force_world)
        B[:, j] = (f_plus - f_minus) / (2.0 * h)

    return A, B


def linearize_hover(
    pos=(0.0, 0.0, 0.0),
    attitude=(0.0, 0.0, 0.0),
    state_eps=1e-6,
    input_eps=1e-6,
):
    """Convenience wrapper for hover linearization."""
    x_eq, u_eq = hover_equilibrium(pos=pos, attitude=attitude)
    A, B = linearize(
        state_eq=x_eq,
        input_eq=u_eq,
        state_eps=state_eps,
        input_eps=input_eps,
        external_force_world=np.zeros(3),
    )
    return A, B, x_eq, u_eq
