"""Simple single-leg stand — task-space CoM PD + gravity compensation.

No WBC, no MPC. Uses:
- Gravity compensation + CoM-space PD for whole-body control
- Joint-space PD for swing leg lift
- Smooth quintic CoM trajectory for weight shift
"""

from __future__ import annotations

import numpy as np

from config import DT_SIM, GRAVITY, SLIP_NET_THRESH, SLIP_PEAK_THRESH, RMSE_THRESH
from robot_model import RobotModel
from robots.unitree_g1 import g1_config
from utils import compute_rmse, compute_pd_torque, plot_diagnostics

from enum import IntEnum


# ---------------------------------------------------------------------------
# Config
# ---------------------------------------------------------------------------
SETTLE_TIME = 1.0
SHIFT_TIME = 6.0
SIM_DURATION = 20.0

KP_COM = 800.0   # CoM-space position gain
KD_COM = 120.0   # CoM-space velocity gain
KP_SWING = 180.0
KD_SWING = 15.0

SWING_HIP_PITCH = 0.40
SWING_KNEE = 1.10
SWING_ANKLE_PITCH = -0.15

MAX_ROLL_DELTA = 0.08  # rad


class Phase(IntEnum):
    SETTLE = 0
    SHIFT = 1
    BALANCE = 2


def _quintic(s: float) -> float:
    s = float(np.clip(s, 0.0, 1.0))
    return 10.0 * s**3 - 15.0 * s**4 + 6.0 * s**5


# ---------------------------------------------------------------------------
# SimLog
# ---------------------------------------------------------------------------
class SimLog:
    def __init__(self, foot_links):
        self.foot_links = foot_links
        self.t: list[float] = []
        self.com: list[np.ndarray] = []
        self.com_ref: list[np.ndarray] = []
        self.phase: list[int] = []
        self.foot_force: dict[int, list[float]] = {l: [] for l in foot_links}
        self.foot_slip: dict[int, list[float]] = {l: [] for l in foot_links}
        self.support_ratio: list[float] = []
        self.L: list[np.ndarray] = []
        self.cop_y: list[float] = []
        self.roll_delta: list[float] = []
        self.swing_force: list[float] = []
        self.support_force: list[float] = []


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
def main() -> None:
    robot = RobotModel(g1_config)
    initial_dof_angles = robot.set_standing_angles(g1_config.standing_joint_angles)

    support_leg = "right"
    swing_leg = "left"
    support_link = robot.foot_name_to_link[g1_config.support_foot_name]
    swing_link = robot.foot_name_to_link[g1_config.swing_foot_name]
    foot_links = robot.foot_link_ids

    tau_max_limits = robot.tau_limits.copy()
    tau_min_limits = -tau_max_limits

    joint_name_to_dof_idx = robot.dof_joint_name_to_index
    swing_dof_indices = np.array([
        joint_name_to_dof_idx[n]
        for n in g1_config.leg_joint_names[swing_leg]
        if n in joint_name_to_dof_idx
    ], dtype=int)

    nv_base = robot.model.jnt_dofadr[1] if robot.model.njnt > 1 else robot.model.nv
    nq_base = robot.model.jnt_qposadr[1] if robot.model.njnt > 1 else robot.model.nq

    initial_foot_pos = {
        link: np.array(robot.get_contact_metrics(link)["position"], copy=True)
        for link in foot_links
    }

    measured_com = robot.compute_com_position()
    standing_com_z = float(measured_com[2])
    support_xy = initial_foot_pos[support_link][:2]
    swing_xy = initial_foot_pos[swing_link][:2]
    stance_midpoint = 0.5 * (support_xy + swing_xy)
    support_offset = support_xy - stance_midpoint
    support_dir = support_offset / max(np.linalg.norm(support_offset), 1e-6)
    support_half_stance = float(np.linalg.norm(support_offset))

    c_ref = np.zeros(3)
    c_ref[:2] = stance_midpoint
    c_ref[2] = standing_com_z

    phase = Phase.SETTLE
    phase_start_time = 0.0

    total_steps = int(SIM_DURATION / DT_SIM)
    log = SimLog(foot_links)

    print(f"\n===== Simple Stand Experiment (duration={SIM_DURATION:.1f}s) =====")
    print(f"Support: {support_leg}, Swing: {swing_leg}")
    print("=" * 60)

    for step in range(total_steps):
        t = step * DT_SIM

        # --- State ---
        q, v = robot.get_state()
        c = robot.compute_com_position()
        c_dot = robot.compute_com_velocity(q, v)
        L = robot.compute_centroidal_momentum(q, v)
        joint_positions = q[nq_base:]
        joint_velocities = v[nv_base:]

        # Contact forces
        foot_contacts = []
        contact_forces = {}
        for link in foot_links:
            cm = robot.get_contact_metrics(link)
            nf = cm["normal_force"]
            foot_contacts.append({"link": link, "normal_force": nf, "position": cm["position"]})
            contact_forces[link] = nf

        elapsed = t - phase_start_time

        # --- Phase transitions ---
        if phase == Phase.SETTLE and elapsed >= SETTLE_TIME:
            phase = Phase.SHIFT
            phase_start_time = t
            print(f"[PHASE] t={t:.2f}s -> SHIFT")
        elif phase == Phase.SHIFT and elapsed >= SHIFT_TIME:
            phase = Phase.BALANCE
            phase_start_time = t
            print(f"[PHASE] t={t:.2f}s -> BALANCE")

        # --- CoM reference ---
        if phase == Phase.SETTLE:
            c_ref[:2] = stance_midpoint
        elif phase == Phase.SHIFT:
            s = elapsed / SHIFT_TIME
            ratio = _quintic(s) * 0.70  # shift 70% toward support foot
            c_ref[:2] = stance_midpoint + ratio * support_dir * support_half_stance
        else:
            c_ref[:2] = stance_midpoint + 0.70 * support_dir * support_half_stance
        c_ref[2] = standing_com_z

        # --- Roll delta ---
        if phase >= Phase.SHIFT:
            if phase == Phase.SHIFT:
                s = elapsed / SHIFT_TIME
                roll_delta = _quintic(s) * MAX_ROLL_DELTA
            else:
                roll_delta = MAX_ROLL_DELTA
        else:
            roll_delta = 0.0

        # --- Gravity compensation ---
        C = robot.compute_coriolis_gravity(q, v)
        tau_g = C[nv_base:]  # gravity + Coriolis torques

        # --- Posture PD (with roll delta for lean) ---
        posture_targets = initial_dof_angles.copy()
        if roll_delta != 0.0:
            for leg in ("left", "right"):
                sign = 1.0 if leg == support_leg else -1.0
                for suffix in ("hip_roll_joint", "ankle_roll_joint"):
                    name = f"{leg}_{suffix}"
                    if name in joint_name_to_dof_idx:
                        idx = joint_name_to_dof_idx[name]
                        posture_targets[idx] += sign * roll_delta

        # --- Assemble torque: gravity comp + posture PD ---
        tau_cmd = tau_g.copy()

        # Posture PD (with roll delta)
        tau_cmd += compute_pd_torque(
            posture_targets, joint_positions, joint_velocities,
            200.0, 30.0, tau_max_limits,
        )

        # --- Swing leg PD ---
        if phase >= Phase.BALANCE:
            swing_targets = initial_dof_angles[swing_dof_indices].copy()
            lift_progress = _quintic(min(1.0, elapsed / 1.5))

            hip_name = f"{swing_leg}_hip_pitch_joint"
            knee_name = f"{swing_leg}_knee_joint"
            ank_name = f"{swing_leg}_ankle_pitch_joint"
            if hip_name in joint_name_to_dof_idx:
                idx = joint_name_to_dof_idx[hip_name] - nv_base
                swing_targets[idx] += lift_progress * (SWING_HIP_PITCH - initial_dof_angles[joint_name_to_dof_idx[hip_name]])
            if knee_name in joint_name_to_dof_idx:
                idx = joint_name_to_dof_idx[knee_name] - nv_base
                swing_targets[idx] += lift_progress * (SWING_KNEE - initial_dof_angles[joint_name_to_dof_idx[knee_name]])
            if ank_name in joint_name_to_dof_idx:
                idx = joint_name_to_dof_idx[ank_name] - nv_base
                swing_targets[idx] += lift_progress * (SWING_ANKLE_PITCH - initial_dof_angles[joint_name_to_dof_idx[ank_name]])

            swing_tau = compute_pd_torque(
                swing_targets, joint_positions[swing_dof_indices],
                joint_velocities[swing_dof_indices],
                KP_SWING, KD_SWING, tau_max_limits[swing_dof_indices],
            )
            tau_cmd[swing_dof_indices] = swing_tau

        tau_cmd = np.clip(tau_cmd, tau_min_limits, tau_max_limits)
        robot.set_joint_torques(tau_cmd)
        robot.step()

        # --- Logging ---
        log.t.append(t)
        log.com.append(c.copy())
        log.com_ref.append(c_ref.copy())
        log.phase.append(int(phase))
        log.L.append(L.copy())
        log.roll_delta.append(roll_delta)
        log.cop_y.append(0.0)
        for fc in foot_contacts:
            log.foot_force[fc["link"]].append(fc["normal_force"])
        cf = {fc["link"]: fc["normal_force"] for fc in foot_contacts}
        log.support_force.append(cf.get(support_link, 0.0))
        log.swing_force.append(cf.get(swing_link, 0.0))
        for link in foot_links:
            if cf.get(link, 0.0) > 10.0:
                cur = robot.get_contact_metrics(link)["position"]
                log.foot_slip[link].append(float(np.linalg.norm(cur[:2] - initial_foot_pos[link][:2])))
            else:
                log.foot_slip[link].append(np.nan)
        sf = cf.get(support_link, 0.0)
        wf = cf.get(swing_link, 0.0)
        r = sf / max(sf + wf, 1e-6)
        log.support_ratio.append(r)

        if step % 500 == 0:
            print(f"[t={t:.2f}s] {Phase(phase).name:8s} "
                  f"sf={cf.get(support_link,0):.0f}N wf={cf.get(swing_link,0):.0f}N "
                  f"ratio={r:.3f} c_ref_y={c_ref[1]:.4f} c_y={c[1]:.4f} rd={roll_delta:.3f}")

    # --- Results ---
    print(f"\n{'='*60}")
    print("===== RESULTS =====")
    rmse = compute_rmse(log.com, log.com_ref)
    com_arr = np.stack(log.com)
    com_ref_arr = np.stack(log.com_ref)
    max_dev = float(np.max(np.linalg.norm(com_arr - com_ref_arr, axis=1)))
    print(f"CoM RMSE: {rmse:.4f} m (target < {RMSE_THRESH} m)")
    print(f"CoM max deviation: {max_dev:.4f} m")

    max_slip = 0.0
    net_slip = 0.0
    for link in foot_links:
        slip_arr = np.array(log.foot_slip[link])
        non_nan = slip_arr[~np.isnan(slip_arr)]
        if len(non_nan) > 0:
            max_slip = max(max_slip, float(np.max(non_nan)))
        try:
            final_pos = robot.get_contact_metrics(link)["position"]
            net = np.linalg.norm(final_pos[:2] - initial_foot_pos[link][:2])
            net_slip = max(net_slip, net)
        except Exception:
            pass
    print(f"Max peak slip: {max_slip * 1000:.2f} mm (target < {SLIP_PEAK_THRESH * 1000:.1f} mm)")
    print(f"Max net displacement: {net_slip * 1000:.2f} mm (target < {SLIP_NET_THRESH * 1000:.1f} mm)")

    for link in foot_links:
        forces = np.array(log.foot_force[link])
        avg = np.mean(forces) if len(forces) > 0 else 0.0
        name = robot.link_to_foot_name.get(link, str(link))
        print(f"  {name} avg force: {avg:.1f} N")

    max_phase = max(log.phase)
    print(f"\nHighest phase reached: {Phase(max_phase).name}")

    reached = max_phase >= Phase.BALANCE
    passed = reached and rmse < RMSE_THRESH and max_slip < SLIP_PEAK_THRESH and net_slip < SLIP_NET_THRESH
    print(f"Verdict: {'PASS' if passed else 'FAIL'}")
    if not passed:
        reasons = []
        if not reached:
            reasons.append("did not reach BALANCE")
        if rmse >= RMSE_THRESH:
            reasons.append(f"RMSE {rmse:.4f} >= {RMSE_THRESH}")
        if max_slip >= SLIP_PEAK_THRESH:
            reasons.append(f"peak slip {max_slip*1000:.1f}mm >= {SLIP_PEAK_THRESH*1000:.0f}mm")
        if net_slip >= SLIP_NET_THRESH:
            reasons.append(f"net slip {net_slip*1000:.1f}mm >= {SLIP_NET_THRESH*1000:.0f}mm")
        print(f"  Failures: {'; '.join(reasons)}")

    plot_diagnostics(
        log, robot.link_to_foot_name, support_link, swing_link,
        rmse_thresh=RMSE_THRESH, slip_net_thresh=SLIP_NET_THRESH, slip_peak_thresh=SLIP_PEAK_THRESH,
        ds_min_force=80.0,
        load_shift_roll_delta=MAX_ROLL_DELTA,
        ds_max_L_norm=0.5,
    )


if __name__ == "__main__":
    main()
