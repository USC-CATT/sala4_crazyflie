
import numpy as np
from numpy.linalg import inv
from collections import namedtuple
from numpy import linalg as LA

DesiredState = namedtuple('DesiredState', 'pos vel acc yaw yawdot')


def _wrap_pi(angle):
    """Wrap angle to [-pi, pi].
    avoid discontinuity when interpolating yaw angles across the -pi/pi boundary.
    """
    return (angle + np.pi) % (2.0 * np.pi) - np.pi

def format_waypoints(waypoints):
    """Return waypoints as Nx4 [x, y, z, yaw(rad)].

    If Nx3 is provided, yaw defaults to 0.0 for every waypoint.
    """
    waypoints = np.asarray(waypoints, dtype=float)
    if waypoints.ndim != 2 or waypoints.shape[1] not in (3, 4):
        raise ValueError("Waypoints must be an Nx3 or Nx4 array/list.")
    if waypoints.shape[0] < 2:
        raise ValueError("Need at least 2 waypoints.")

    if waypoints.shape[1] == 3:
        zeros = np.zeros((waypoints.shape[0], 1), dtype=float)
        waypoints = np.hstack((waypoints, zeros))
    return waypoints

def get_helix_waypoints(t, n, yaw=0.0):
    """Generate n helix waypoints with explicit yaw.

    Output shape is [n, 4] with columns [x, y, z, yaw(rad)].
    """
    waypoints_t = np.linspace(t, t + 2*np.pi, n)
    x = 0.5*np.cos(waypoints_t)
    y = 0.5*np.sin(waypoints_t)
    z = waypoints_t
    yaw_col = np.full((n,), float(yaw), dtype=float)

    return np.stack((x, y, z, yaw_col), axis=-1)

def get_MST_coefficients(waypoints):
    # generate MST coefficients for each segment from xyz columns only
    waypoints = format_waypoints(waypoints)
    coeff_x = MST(waypoints[:,0]).transpose()[0]
    coeff_y = MST(waypoints[:,1]).transpose()[0]
    coeff_z = MST(waypoints[:,2]).transpose()[0]
    return (coeff_x, coeff_y, coeff_z)

def generate_trajectory(t, v, waypoints, coeff_x=None, coeff_y=None, coeff_z=None):
    """ The function takes known number of waypoints and time, then generates a
    minimum snap trajectory which goes through each waypoint. The output is
    the desired state associated with the next waypont for the time t.
    waypoints is [N,3] or [N,4] matrix, waypoints = [[x,y,z] or [x,y,z,yaw]].
    v is velocity in m/s
    """
    waypoints = format_waypoints(waypoints)
    if v <= 0.0:
        raise ValueError("Trajectory speed v must be > 0.")

    if coeff_x is None and coeff_y is None and coeff_z is None:
        coeff_x, coeff_y, coeff_z = get_MST_coefficients(waypoints)
    elif coeff_x is None or coeff_y is None or coeff_z is None:
        raise ValueError("Provide all of coeff_x/coeff_y/coeff_z or none of them.")

    yawdot = 0.0
    pos = np.zeros(3)
    acc = np.zeros(3)
    vel = np.zeros(3)

    # distance vector array, represents each segment's distance
    distance = waypoints[0:-1, :3] - waypoints[1:, :3]
    # T is now each segment's travel time
    T = (1.0 / v) * np.sqrt(distance[:,0]**2 + distance[:,1]**2 + distance[:,2]**2)
    T = np.maximum(T, 1e-6)
    # accumulated time
    S = np.zeros(len(T) + 1)
    S[1:] = np.cumsum(T)
    yaw_points = np.unwrap(waypoints[:, 3])
    t = float(t)

    # prepare the next desired state
    if t <= 0.0:
        pos = waypoints[0, :3]
        yaw = _wrap_pi(yaw_points[0])
        yawdot = 0.0
    # stay hover at the last waypoint position
    elif t >= S[-1]:
        pos = waypoints[-1, :3]
        yaw = _wrap_pi(yaw_points[-1])
        yawdot = 0.0
    else:
        # find which segment current t belongs to
        t_index = int(np.searchsorted(S, t, side="right") - 1)
        t_index = int(np.clip(t_index, 0, len(T) - 1))

        # scaled time
        scale = (t - S[t_index]) / T[t_index]
        start = 8 * t_index
        end = 8 * (t_index + 1)

        t0 = get_poly_cc(8, 0, scale)
        pos = np.array([coeff_x[start:end].dot(t0), coeff_y[start:end].dot(t0), coeff_z[start:end].dot(t0)])

        t1 = get_poly_cc(8, 1, scale)
        # chain rule applied
        vel = np.array([coeff_x[start:end].dot(t1), coeff_y[start:end].dot(t1), coeff_z[start:end].dot(t1)]) * (1.0 / T[t_index])

        t2 = get_poly_cc(8, 2, scale)
        # chain rule applied
        acc = np.array([coeff_x[start:end].dot(t2), coeff_y[start:end].dot(t2), coeff_z[start:end].dot(t2)]) * (1.0 / T[t_index]**2)

        # User-defined yaw interpolation along the same segment schedule.
        yaw_start = yaw_points[t_index]
        yaw_end = yaw_points[t_index + 1]
        yaw = _wrap_pi(yaw_start + scale * (yaw_end - yaw_start))
        yawdot = (yaw_end - yaw_start) / T[t_index]
    return DesiredState(pos, vel, acc, yaw, yawdot)

def get_poly_cc(n, k, t):
    """ This is a helper function to get the coeffitient of coefficient for n-th
        order polynomial with k-th derivative at time t.
    """
    assert (n > 0 and k >= 0), "order and derivative must be positive."

    cc = np.ones(n)
    D  = np.linspace(0, n-1, n)

    for i in range(n):
        for j in range(k):
            cc[i] = cc[i] * D[i]
            D[i] = D[i] - 1
            if D[i] == -1:
                D[i] = 0

    for i, c in enumerate(cc):
        cc[i] = c * np.power(t, D[i])

    return cc

# Minimum Snap Trajectory
def MST(waypoints):
    """ This function takes a list of desired waypoint i.e. [x0, x1, x2...xN] and
    time, returns a [8N,1] coeffitients matrix for the N+1 waypoints.

    1.The Problem
    Generate a full trajectory across N+1 waypoint made of N polynomial line segment.
    Each segment is defined as 7 order polynomial defined as follow:
    Pi = ai_0 + ai1*t + ai2*t^2 + ai3*t^3 + ai4*t^4 + ai5*t^5 + ai6*t^6 + ai7*t^7

    Each polynomial has 8 unknown coefficients, thus we will have 8*N unknown to
    solve in total, so we need to come up with 8*N constraints.

    2.The constraints
    In general, the constraints is a set of condition which define the initial
    and final state, continuity between each piecewise function. This includes
    specifying continuity in higher derivatives of the trajectory at the
    intermediate waypoints.

    3.Matrix Design
    Since we have 8*N unknown coefficients to solve, and if we are given 8*N
    equations(constraints), then the problem becomes solving a linear equation.

    A * Coeff = B

    Let's look at B matrix first, B matrix is simple because it is just some constants
    on the right hand side of the equation. There are 8xN constraints,
    so B matrix will be [8N, 1].

    Now, how do we determine the dimension of Coeff matrix? Coeff is the final
    output matrix consists of 8*N elements. Since B matrix is only one column,
    thus Coeff matrix must be [8N, 1].

    Coeff.transpose = [a10 a11..a17...aN0 aN1..aN7]

    A matrix is tricky, we then can think of A matrix as a coeffient-coeffient matrix.
    We are no longer looking at a particular polynomial Pi, but rather P1, P2...PN
    as a whole. Since now our Coeff matrix is [8N, 1], and B is [8N, 8N], thus
    A matrix must have the form [8N, 8N].

    A = [A10 A12 ... A17 ... AN0 AN1 ...AN7
         ...
        ]

    Each element in a row represents the coefficient of coeffient aij under
    a certain constraint, where aij is the jth coeffient of Pi with i = 1...N, j = 0...7.
    """

    n = len(waypoints) - 1

    # initialize A, and B matrix
    A = np.zeros((8*n, 8*n))
    B = np.zeros((8*n, 1))

    # populate B matrix.
    for i in range(n):
        B[i] = waypoints[i]
        B[i + n] = waypoints[i+1]

    # Constraint 1
    for i in range(n):
        A[i][8*i:8*(i+1)] = get_poly_cc(8, 0, 0)

    # Constraint 2
    for i in range(n):
        A[i+n][8*i:8*(i+1)] = get_poly_cc(8, 0, 1)

    # Constraint 3
    for k in range(1, 4):
        A[2*n+k-1][:8] = get_poly_cc(8, k, 0)

    # Constraint 4
    for k in range(1, 4):
        A[2*n+3+k-1][-8:] = get_poly_cc(8, k, 1)

    # Constraint 5
    for i in range(n-1):
        for k in range(1, 7):
            A[2*n+6 + i*6+k-1][i*8 : (i*8+16)] = np.concatenate((get_poly_cc(8, k, 1), -get_poly_cc(8, k, 0)))

    # solve for the coefficients
    Coeff = np.linalg.solve(A, B)
    return Coeff
