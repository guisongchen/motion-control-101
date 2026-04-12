"""
LIPM / Cart-Table Pattern Generator for Biped Walking.
"""
import numpy as np
from scipy.linalg import solve_discrete_are


def build_cart_table_model(dt: float, zc: float, g: float = 9.81):
    """
    Build discrete-time Cart-Table model.
    State: x = [p; v; a], input: u = jerk.
    Returns A (3x3), b (3x1), c (3x1) for ZMP output z = c^T x.
    """
    A = np.array([
        [1.0, dt, 0.5 * dt**2],
        [0.0, 1.0, dt],
        [0.0, 0.0, 1.0],
    ])
    b = np.array([[dt**3 / 6.0],
                  [dt**2 / 2.0],
                  [dt]])
    c = np.array([[1.0],
                  [0.0],
                  [-zc / g]])
    return A, b, c


def compute_preview_gains(A, b, c, Q: float, R: float, N: int):
    """
    Compute Riccati gain K and preview gains f[0..N-1].
    Returns K (1x3), f (N,) as 1D arrays.
    Control law: u_k = -K x_k + sum_j f_j * z_ref_{k+j}
    """
    # DARE for LQR tracking
    P = solve_discrete_are(A, b, Q * (c @ c.T), R)
    K = (R + b.T @ P @ b).item() ** -1 * (b.T @ P @ A)
    K = K.reshape(1, 3)
    f = np.zeros(N)
    G_inv = (R + b.T @ P @ b).item()
    Ac = A - b @ K
    for j in range(1, N + 1):
        f[j - 1] = G_inv * (Q * c.T @ np.linalg.matrix_power(Ac, j - 1) @ b).item()
    return K, f


def generate_footsteps_zmp_ref(
    num_steps: int,
    step_length: float,
    step_width: float,
    T_ss: float,
    T_ds: float,
    dt: float,
):
    """
    Generate ZMP reference for straight-line walking.

    ZMP stays at the center of support foot during single support,
    and linearly transitions between feet during double support.
    Returns t_axis, zmp_x, zmp_y, foot_refs (list of dicts with timing and positions).
    """
    # Build footstep schedule
    # Start with both feet on ground (initial DS)
    # Then SS_L, DS, SS_R, DS, ...
    foot_refs = []
    # Assume starting feet at x=0, y=+/-step_width/2
    left_pos = np.array([0.0, step_width / 2.0])
    right_pos = np.array([0.0, -step_width / 2.0])

    t = 0.0
    # Initial double support (half? use full T_ds)
    foot_refs.append({"t_start": t, "t_end": t + T_ds, "support": "both",
                      "left": left_pos.copy(), "right": right_pos.copy()})
    t += T_ds

    for i in range(num_steps):
        if i % 2 == 0:
            # Left single support
            foot_refs.append({"t_start": t, "t_end": t + T_ss, "support": "left",
                              "left": left_pos.copy(), "right": right_pos.copy()})
            t += T_ss
            # Double support, move right foot forward
            right_pos[0] += step_length
            foot_refs.append({"t_start": t, "t_end": t + T_ds, "support": "both",
                              "left": left_pos.copy(), "right": right_pos.copy()})
            t += T_ds
        else:
            # Right single support
            foot_refs.append({"t_start": t, "t_end": t + T_ss, "support": "right",
                              "left": left_pos.copy(), "right": right_pos.copy()})
            t += T_ss
            # Double support, move left foot forward
            left_pos[0] += step_length
            foot_refs.append({"t_start": t, "t_end": t + T_ds, "support": "both",
                              "left": left_pos.copy(), "right": right_pos.copy()})
            t += T_ds

    total_time = t
    n_samples = int(np.ceil(total_time / dt)) + 1
    t_axis = np.arange(n_samples) * dt
    zmp_x = np.zeros(n_samples)
    zmp_y = np.zeros(n_samples)

    for seg in foot_refs:
        i0 = int(np.round(seg["t_start"] / dt))
        i1 = int(np.round(seg["t_end"] / dt))
        i0 = max(0, min(i0, n_samples - 1))
        i1 = max(0, min(i1, n_samples - 1))
        if seg["support"] == "left":
            zmp_x[i0:i1] = seg["left"][0]
            zmp_y[i0:i1] = seg["left"][1]
        elif seg["support"] == "right":
            zmp_x[i0:i1] = seg["right"][0]
            zmp_y[i0:i1] = seg["right"][1]
        else:
            # linear interpolation between left and right
            alpha = np.linspace(0, 1, max(1, i1 - i0))
            zmp_x[i0:i1] = seg["left"][0] + alpha * (seg["right"][0] - seg["left"][0])
            zmp_y[i0:i1] = seg["left"][1] + alpha * (seg["right"][1] - seg["left"][1])

    return t_axis, zmp_x, zmp_y, foot_refs


def simulate_preview_control(x0, A, b, c, K, f, z_ref):
    """
    Simulate closed-loop preview control.
    x0: initial state (3,)
    z_ref: ZMP reference array (K,)
    Returns com_traj (K,), com_vel (K,), com_acc (K,), zmp_actual (K,), u (K,)
    """
    N = len(f)
    K_steps = len(z_ref)
    x = np.zeros((K_steps, 3))
    u = np.zeros(K_steps)
    x[0] = x0
    zmp_actual = np.zeros(K_steps)

    for k in range(K_steps):
        zmp_actual[k] = (c.T @ x[k]).item()
        preview_sum = 0.0
        for j in range(1, N + 1):
            if k + j < K_steps:
                preview_sum += f[j - 1] * z_ref[k + j]
        u[k] = (-K @ x[k]).item() + preview_sum
        if k + 1 < K_steps:
            x[k + 1] = (A @ x[k] + b.squeeze() * u[k]).squeeze()

    return x[:, 0], x[:, 1], x[:, 2], zmp_actual, u


def generate_swing_foot_trajectory(p_start, p_end, H: float, T_swing: float, dt: float):
    """
    Generate cycloid swing foot trajectory.
    p_start, p_end: arrays of shape (2,) or (3,) for [x, y] or [x, y, z].
    H: max foot clearance (m).
    Returns t_axis, traj array.
    """
    p_start = np.asarray(p_start)
    p_end = np.asarray(p_end)
    n_samples = int(np.round(T_swing / dt)) + 1
    t_axis = np.arange(n_samples) * dt
    tau = t_axis / T_swing

    # horizontal interpolation with cycloid smoothing
    horiz = p_start + (p_end - p_start) * (tau[:, None] - np.sin(2 * np.pi * tau[:, None]) / (2 * np.pi))

    # vertical: base ground level linearly interpolated, plus cycloid arch
    # assume input z is ground contact level
    z_base = p_start[-1] + (p_end[-1] - p_start[-1]) * tau[:, None]
    z_arch = (H / 2.0) * (1.0 - np.cos(2 * np.pi * tau[:, None]))
    vert = z_base + z_arch

    if p_start.size == 2:
        traj = np.hstack([horiz[:, 0:1], vert])
    else:
        # keep y coordinate linearly interpolated if 3D
        if p_start.size == 3:
            y = p_start[1] + (p_end[1] - p_start[1]) * tau[:, None]
            traj = np.hstack([horiz[:, 0:1], y, vert])
        else:
            raise ValueError("p_start must be 2D or 3D")
    return t_axis, traj


def build_full_gait(
    num_steps: int = 5,
    step_length: float = 0.3,
    step_width: float = 0.2,
    zc: float = 0.8,
    T_ss: float = 0.6,
    T_ds: float = 0.2,
    H: float = 0.08,
    dt: float = 0.01,
    Q: float = 1.0,
    R: float = 1e-4,
    N_preview: int = 160,
):
    """
    Convenience function: build full 3D gait trajectory.
    Returns dict with t_axis, com_x, com_y, zmp_x, zmp_y, left_foot, right_foot, foot_refs.
    """
    A, b, c = build_cart_table_model(dt, zc)
    K, f = compute_preview_gains(A, b, c, Q, R, N_preview)
    t_axis, zmp_x, zmp_y, foot_refs = generate_footsteps_zmp_ref(
        num_steps, step_length, step_width, T_ss, T_ds, dt
    )

    x0 = np.array([zmp_x[0], 0.0, 0.0])
    com_x, _, _, _, _ = simulate_preview_control(x0, A, b, c, K, f, zmp_x)

    x0 = np.array([zmp_y[0], 0.0, 0.0])
    com_y, _, _, _, _ = simulate_preview_control(x0, A, b, c, K, f, zmp_y)

    # Build foot trajectories
    n_samples = len(t_axis)
    left_foot = np.zeros((n_samples, 3))
    right_foot = np.zeros((n_samples, 3))

    for seg in foot_refs:
        i0 = int(np.round(seg["t_start"] / dt))
        i1 = int(np.round(seg["t_end"] / dt))
        i0 = max(0, min(i0, n_samples - 1))
        i1 = max(0, min(i1, n_samples - 1))

        if seg["support"] == "left":
            # Left foot is stance
            left_foot[i0:i1, 0] = seg["left"][0]
            left_foot[i0:i1, 1] = seg["left"][1]
            left_foot[i0:i1, 2] = 0.0
            # Right foot swings from current right_pos to the next right_pos
            # The landing position is the right_pos in the next DS/SS segment
            landing_pos = np.array([seg["right"][0] + step_length, seg["right"][1], 0.0])
            start_pos = np.array([seg["right"][0], seg["right"][1], 0.0])
            _, traj = generate_swing_foot_trajectory(start_pos, landing_pos, H, T_ss, dt)
            n_write = min(i1 - i0, len(traj))
            right_foot[i0:i0 + n_write, :] = traj[:n_write, :]
        elif seg["support"] == "right":
            right_foot[i0:i1, 0] = seg["right"][0]
            right_foot[i0:i1, 1] = seg["right"][1]
            right_foot[i0:i1, 2] = 0.0
            landing_pos = np.array([seg["left"][0] + step_length, seg["left"][1], 0.0])
            start_pos = np.array([seg["left"][0], seg["left"][1], 0.0])
            _, traj = generate_swing_foot_trajectory(start_pos, landing_pos, H, T_ss, dt)
            n_write = min(i1 - i0, len(traj))
            left_foot[i0:i0 + n_write, :] = traj[:n_write, :]
        else:
            left_foot[i0:i1, 0] = seg["left"][0]
            left_foot[i0:i1, 1] = seg["left"][1]
            left_foot[i0:i1, 2] = 0.0
            right_foot[i0:i1, 0] = seg["right"][0]
            right_foot[i0:i1, 1] = seg["right"][1]
            right_foot[i0:i1, 2] = 0.0

    return {
        "t": t_axis,
        "com": np.column_stack([com_x, com_y, np.full_like(com_x, zc)]),
        "zmp": np.column_stack([zmp_x, zmp_y, np.zeros_like(zmp_x)]),
        "left_foot": left_foot,
        "right_foot": right_foot,
        "foot_refs": foot_refs,
    }


if __name__ == "__main__":
    traj = build_full_gait(num_steps=5, step_length=0.3, T_ss=0.6, T_ds=0.2, dt=0.01)
    print(f"Generated {len(traj['t'])} samples, duration {traj['t'][-1]:.2f}s")
    print(f"CoM x range: [{traj['com'][:, 0].min():.3f}, {traj['com'][:, 0].max():.3f}]")
