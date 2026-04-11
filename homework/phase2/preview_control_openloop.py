import numpy as np
import matplotlib.pyplot as plt
from scipy import linalg


def build_cart_table_model(dt, z_c, g):
    """Build Cart-Table model matrices."""
    T = dt
    A = np.array([
        [1, T, T**2 / 2],
        [0, 1, T],
        [0, 0, 1]
    ], dtype=float)
    b = np.array([T**3 / 6, T**2 / 2, T], dtype=float).reshape(-1, 1)
    c = np.array([1, 0, -z_c / g], dtype=float).reshape(-1, 1)
    return A, b, c


def generate_footsteps_zmp_ref(dt, T_total, T_single, T_double, S, init_left=0.0, init_right=0.2):
    """
    Generate ZMP reference trajectory for straight walking.

    Pattern:
      - left single support:  z = left_pos
      - double support:       z linearly transitions left_pos -> right_pos
      - right single support: z = right_pos
      - double support:       z linearly transitions right_pos -> next_left_pos
    """
    t = np.arange(0, T_total, dt)
    z_ref = np.zeros_like(t)

    full_cycle = 2 * (T_single + T_double)  # 1.6 s for this test case

    for i, ti in enumerate(t):
        n_cycle = int(ti // full_cycle)
        ct = ti - n_cycle * full_cycle

        left_pos = init_left + 2 * S * n_cycle
        right_pos = init_right + 2 * S * n_cycle
        next_left = left_pos + 2 * S

        if ct < T_single:
            # left single support
            z_ref[i] = left_pos
        elif ct < T_single + T_double:
            # double: left -> right
            alpha = (ct - T_single) / T_double
            z_ref[i] = left_pos + alpha * (right_pos - left_pos)
        elif ct < T_single + T_double + T_single:
            # right single support
            z_ref[i] = right_pos
        else:
            # double: right -> next left
            alpha = (ct - T_single - T_double - T_single) / T_double
            z_ref[i] = right_pos + alpha * (next_left - right_pos)

    return t, z_ref


def build_batch_matrices(A, b, c, K):
    """
    Build batch matrices for open-loop preview control.

    Given x_0, the ZMP sequence Z = [z_1, ..., z_K]^T can be written as:
        Z = M_x0 @ x_0 + M_u @ U
    where U = [u_0, ..., u_{K-1}]^T.
    """
    M_x0 = np.zeros((K, 3))
    M_u = np.zeros((K, K))

    for k in range(1, K + 1):
        # c^T @ A^k
        M_x0[k - 1, :] = (c.T @ np.linalg.matrix_power(A, k)).flatten()

        for i in range(k):
            # c^T @ A^{k-1-i} @ b
            M_u[k - 1, i] = (c.T @ np.linalg.matrix_power(A, k - 1 - i) @ b).item()

    return M_x0, M_u


def solve_open_loop_preview(A, b, c, x0, z_ref, Q, R):
    """
    Solve open-loop preview control as a batch QP.

    Minimizes:
        J = sum_{k=1}^{K} Q*(z_ref_k - z_k)^2 + R*u_{k-1}^2
    """
    K = len(z_ref)
    M_x0, M_u = build_batch_matrices(A, b, c, K)

    # Stacked cost
    # J = (z_ref - M_x0*x0 - M_u*u)^T * Q*I * (z_ref - M_x0*x0 - M_u*u) + u^T * R*I * u
    # Optimal u: (M_u^T * Q * M_u + R*I) * u = M_u^T * Q * (z_ref - M_x0*x0)

    Q_eye = Q * np.eye(K)
    R_eye = R * np.eye(K)

    H = M_u.T @ Q_eye @ M_u + R_eye
    h = M_u.T @ Q_eye @ (z_ref - M_x0 @ x0)

    U = linalg.solve(H, h, assume_a='pos')
    return U

def batch_optimization_control(A, b, c, x0, z_ref, Q, R):
    # --- Open-loop batch optimization over the entire horizon ---
    U = solve_open_loop_preview(A, b, c, x0, z_ref, Q, R)
    x, z = simulate_trajectory(A, b, c, x0, U)
    return x, z, U

def receding_horizon_control(A, b, c, x0, z_ref, Q, R, N):
    # receding horizon control: solve a smaller batch problem at each time step, then apply the first control input
    K = len(z_ref)  # horizon length = total steps
    x = np.zeros((3, K + 1))
    z = np.zeros(K)
    u_applied = np.zeros(K)
    x[:, 0] = x0

    for k in range(K):
        # Solve preview control for the remaining horizon
        end = min(k + N, K)  # ensure we don't go beyond the reference length
        z_preview = z_ref[end - min(N, K - k) : end]  # preview reference (up to N steps ahead)
        
        if len(z_preview) < N:
            # Pad with the last reference if we have less than N steps left
            z_preview = np.pad(z_preview, (0, N - len(z_preview)), 'edge')
        
        U = solve_open_loop_preview(A, b, c, x[:, k], z_preview, Q, R)

        # Apply the first control input
        u_k = U[0]
        u_applied[k] = u_k

        # Simulate one step
        x[:, k + 1] = A @ x[:, k] + b.flatten() * u_k
        z[k] = (c.T @ x[:, k + 1]).item()
    return x, z, u_applied

# def compute_preview_gains(A, b, c, Q, R, N):
#     """
#     Compute steady-state LQR gain K and preview gains f_j.

#     Returns:
#         K: steady-state feedback gain (1 x n)
#         f: preview gains (N,)
#     """
#     n = A.shape[0]
#     P = Q * (c @ c.T)

#     # --- Backward Riccati iteration until convergence ---
#     max_iter = 10000
#     tol = 1e-12
#     K = None
#     for _ in range(max_iter):
#         D = (R + b.T @ P @ b).item()
#         K_new = (1.0 / D) * (b.T @ P @ A)
#         P_new = Q * (c @ c.T) + A.T @ P @ A - D * (K_new.T @ K_new)

#         diff = np.max(np.abs(P_new - P))
#         K = K_new.flatten()
#         P = P_new

#         if diff < tol:
#             break
#     else:
#         print(f"Warning: Riccati did not converge in {max_iter} iters， final max diff: {diff:.8f}")

#     # --- Preview gains: f_j = Q * c.T @ (A - bK)^{j-1} @ b / D ---
#     D = (R + b.T @ P @ b).item()
#     A_cl = A - b @ K.reshape(1, -1)
#     f = np.zeros(N)

#     power = np.eye(n)
#     for j in range(N):
#         f[j] = (Q / D) * (c.T @ power @ b).item()
#         power = A_cl @ power

#     return K, f

def compute_preview_gains(A, b, c, Q, R, N):
    """
    Compute steady-state LQR gain K and preview gains f_j using DARE.
    """
    n = A.shape[0]
    Qeff = Q * (c @ c.T)

    # --- Solve Discrete Algebraic Riccati Equation directly ---
    P = linalg.solve_discrete_are(A, b, Qeff, R)

    # --- Steady-state feedback gain ---
    D = (R + b.T @ P @ b).item()
    K = ((1.0 / D) * (b.T @ P @ A)).flatten()

    # --- Preview gains ---
    A_cl = A - b @ K.reshape(1, -1)
    f = np.zeros(N)

    power = np.eye(n)
    for j in range(N):
        f[j] = (Q / D) * (c.T @ power @ b).item()
        power = A_cl @ power

    return K, f


def riccati_preview_control(A, b, c, x0, z_ref, Q, R, N):
    """
    Simulate using the classic preview control law:
        uk = -K @ xk + sum{j=1}^{N} fj * zref{k+j}
    """

    K, f = compute_preview_gains(A, b, c, Q, R, N)

    K = np.asarray(K).flatten()
    N = len(f)
    K_total = len(z_ref)

    x = np.zeros((3, K_total + 1))
    z = np.zeros(K_total)
    u = np.zeros(K_total)
    x[:, 0] = x0

    for k in range(K_total):
        end = min(k + N, K_total)
        preview = z_ref[end - min(N, K_total - k): end]
        if len(preview) < N:
            preview = np.pad(preview, (0, N - len(preview)), mode='edge')

        u_k = -K @ x[:, k] + f @ preview
        u[k] = u_k
        x[:, k + 1] = A @ x[:, k] + b.flatten() * u_k
        z[k] = (c.T @ x[:, k + 1]).item()

    return x, z, u



def simulate_trajectory(A, b, c, x0, U):
    """Roll out state and ZMP trajectory given control sequence U."""
    K = len(U)
    x = np.zeros((3, K + 1))
    z = np.zeros(K)
    x[:, 0] = x0

    for k in range(K):
        u_k = U[k]
        x[:, k + 1] = A @ x[:, k] + b.flatten() * u_k
        z[k] = (c.T @ x[:, k + 1]).item()

    return x, z

def generate_swing_foot_trajectory(x_start, x_end, H, T_swing, dt):
    """
    Generate swing foot trajectory from x_start to x_end with height H over duration T_swing.

    Uses a cycloidal trajectory in the horizontal plane and a cosine-based vertical trajectory.
    """
    t = np.arange(0, T_swing, dt)
    N = len(t)
    pos = np.zeros((3, N))
    vel = np.zeros((3, N))

    S = x_end - x_start

    for i, ti in enumerate(t):
        tau = ti / T_swing

        # Horizontal cycloid trajectory
        pos[0, i] = x_start[0] + S[0] * (tau - np.sin(2 * np.pi * tau) / (2 * np.pi))
        pos[1, i] = x_start[1] + S[1] * (tau - np.sin(2 * np.pi * tau) / (2 * np.pi))

        # Vertical trajectory: base linear interpolation + cycloidal lift
        z_base = x_start[2] + (x_end[2] - x_start[2]) * tau
        pos[2, i] = z_base + (H / 2) * (1 - np.cos(2 * np.pi * tau))

        # Velocities
        vel[0, i] = (S[0] / T_swing) * (1 - np.cos(2 * np.pi * tau))
        vel[1, i] = (S[1] / T_swing) * (1 - np.cos(2 * np.pi * tau))
        vel[2, i] = ((x_end[2] - x_start[2]) / T_swing) + (H * np.pi / T_swing) * np.sin(2 * np.pi * tau)

    return t, pos, vel

def generate_full_gait_trajectory(x, z, u, t, T_single, T_double, S, H, z_ground):
    """
    Generate full gait trajectory (CoM, left foot, right foot) from CoM and ZMP trajectories.
    
    """

    left_pose = np.zeros((3, len(t)))
    right_pose = np.zeros((3, len(t)))

    full_cycle = 2 * (T_single + T_double)

    for i, ti in enumerate(t):
        n_cycle = int(ti // full_cycle)
        ct = ti - n_cycle * full_cycle

        lt = 2 * S * n_cycle
        rt = S + 2 * S * n_cycle

        left_pose[0, i] = lt
        left_pose[1, i] = 0.0
        left_pose[2, i] = z_ground

        right_pose[0, i] = rt
        right_pose[1, i] = 0.0
        right_pose[2, i] = z_ground

    T_total = t[-1]
    dt = t[1] - t[0]
    n_cycles = int(T_total // full_cycle) + 2
    for n in range(n_cycles):
        # Right foot swing (left single support)
        t_start_right = n * full_cycle
        if t_start_right >= T_total:
            break
        
        idx_start = int(t_start_right / dt)
        count = min(int(T_single / dt), len(t) - idx_start)
        if count > 0:
            rt_start = S + 2 * S * n
            rt_start_3d = np.array([rt_start, 0.0, z_ground])
            rt_end_3d = np.array([rt_start + 2 * S, 0.0, z_ground])
            _, swing_pos, _ = generate_swing_foot_trajectory(rt_start_3d, rt_end_3d, H, T_single, dt)
            right_pose[:, idx_start:idx_start + count] = swing_pos[:, :count]
        # Left foot swing (right single support)
        t_start_left = n * full_cycle + T_single + T_double
        
        if t_start_left >= T_total:            
            break
        
        idx_start = int(t_start_left / dt)
        count = min(int(T_single / dt), len(t) - idx_start)
        if count > 0:
            lt_start = 2 * S * n
            lt_start_3d = np.array([lt_start, 0.0, z_ground])
            lt_end_3d = np.array([lt_start + 2 * S, 0.0, z_ground])
            _, swing_pos, _ = generate_swing_foot_trajectory(lt_start_3d, lt_end_3d, H, T_single, dt)
            left_pose[:, idx_start:idx_start + count] = swing_pos[:, :count]
    
    return left_pose, right_pose

def plot_results(t, z_ref, z, x, u):
    """Plot ZMP, CoM, velocity, acceleration, and jerk."""
    fig, axes = plt.subplots(4, 1, figsize=(12, 10), sharex=True)

    # 1. ZMP reference vs actual ZMP vs CoM position
    axes[0].plot(t, z_ref, 'k--', label='ZMP ref')
    axes[0].plot(t, z, 'r-', label='ZMP actual')
    axes[0].plot(t, x[0, 1:], 'b-', label='CoM position')
    axes[0].set_ylabel('Position (m)')
    axes[0].set_title('ZMP Reference vs Actual vs CoM')
    axes[0].legend()
    axes[0].grid(True)

    # 2. CoM velocity
    axes[1].plot(t, x[1, 1:], 'g-', label='CoM velocity')
    axes[1].set_ylabel('Velocity (m/s)')
    axes[1].set_title('CoM Velocity')
    axes[1].legend()
    axes[1].grid(True)

    # 3. CoM acceleration
    axes[2].plot(t, x[2, 1:], 'm-', label='CoM acceleration')
    axes[2].set_ylabel('Acceleration (m/s²)')
    axes[2].set_title('CoM Acceleration')
    axes[2].legend()
    axes[2].grid(True)

    # 4. Control input (jerk)
    axes[3].plot(t, u, 'c-', label='Jerk')
    axes[3].set_ylabel('Jerk (m/s³)')
    axes[3].set_title('Control Input (Jerk)')
    axes[3].set_xlabel('Time (s)')
    axes[3].legend()
    axes[3].grid(True)

    plt.tight_layout()
    plt.savefig('/home/ccc/projects/motion_control_101/homework/phase2/preview_control_openloop.png', dpi=150)
    plt.close()


def plot_full_gait(t, com_pos, z_ref, z_actual, left_pos, right_pos):
    fig, axes = plt.subplots(2, 1, figsize=(12, 8))

    axes[0].plot(t, z_ref, 'k--', label='ZMP ref', linewidth=1.5)
    axes[0].plot(t, z_actual, 'r-', label='ZMP actual', linewidth=1.5)
    axes[0].plot(t, com_pos, 'b-', label='CoM', linewidth=2.0)
    axes[0].plot(t, left_pos[0, :], 'g-', label='Left foot x', linewidth=1.5)
    axes[0].plot(t, right_pos[0, :], 'm-', label='Right foot x', linewidth=1.5)
    axes[0].set_ylabel('X position (m)')
    axes[0].set_title('Horizontal Gait Trajectories')
    axes[0].legend()
    axes[0].grid(True)

    axes[1].plot(t, com_pos * 0 + 0.8, 'b--', label='CoM height (zc=0.8)', linewidth=1.0)
    axes[1].plot(t, left_pos[2, :], 'g-', label='Left foot z', linewidth=1.5)
    axes[1].plot(t, right_pos[2, :], 'm-', label='Right foot z', linewidth=1.5)
    axes[1].set_ylabel('Z position (m)')
    axes[1].set_xlabel('Time (s)')
    axes[1].set_title('Vertical Foot Trajectories')
    axes[1].legend()
    axes[1].grid(True)

    plt.tight_layout()
    plt.savefig('/home/ccc/projects/motion_control_101/homework/phase2/full_gait_trajectory.png', dpi=150)
    plt.close()


def main():
    # --- Robot parameters ---
    g = 9.81
    z_c = 0.8
    dt = 0.01
    Tc = np.sqrt(z_c / g)

    # --- Footstep parameters ---
    S = 0.2
    T_single = 0.6
    T_double = 0.2
    T_step = T_single + T_double

    # --- Preview control parameters ---
    N = 160
    Q = 1.0
    R = 1e-6

    method = "riccati"  # "batch", "receding_horizon", or "riccati"

    # --- Initial state ---
    x0 = np.array([0.0, 0.0, 0.0])

    # --- Generate ZMP reference ---
    # Simulate enough time to see periodic gait (e.g., 6 seconds)
    T_total = 6.0
    t, z_ref = generate_footsteps_zmp_ref(dt, T_total, T_single, T_double, S)

    # --- Build model ---
    A, b, c = build_cart_table_model(dt, z_c, g)

    # --- Run preview control ---
    if method == "batch":
        x, z, U = batch_optimization_control(A, b, c, x0, z_ref, Q, R)
    elif method == "receding_horizon":
        x, z, U = receding_horizon_control(A, b, c, x0, z_ref, Q, R, N)
    elif method == "riccati":
        x, z, U = riccati_preview_control(A, b, c, x0, z_ref, Q, R, N)
    else:
        raise ValueError("Invalid method. Choose 'batch', 'receding_horizon', or 'riccati'.")

    # --- Numerical checks ---
    rms_error = np.sqrt(np.mean((z - z_ref)**2))
    max_jerk = np.max(np.abs(U))

    print(f"ZMP tracking RMS error: {rms_error*1000:.3f} mm")
    print(f"Max jerk: {max_jerk:.4f} m/s³")

    # --- Plot results ---
    # plot_results(t, z_ref, z, x, U)

    # --- Plot full gait ---
    left_pos, right_pos = generate_full_gait_trajectory(x, z, U, t, T_single, T_double, S, H=0.08, z_ground=0.0)
    plot_full_gait(t, x[0, 1:], z_ref, z, left_pos, right_pos)
    print("Full gait trajectory saved.")

if __name__ == "__main__":
    main()
