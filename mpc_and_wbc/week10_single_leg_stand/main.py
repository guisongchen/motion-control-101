"""主循环：MPC + WBC + PyBullet"""

import time
import numpy as np
import pybullet as p
import pybullet_data

from config import (
    DT_SIM, WBC_FREQ, MPC_FREQ, SIM_DURATION,
    URDF_PATH, INITIAL_POSE, LIFT_LEG, LIFT_TIME, H_COM,
    FOOT_LINK_NAMES, STANDING_JOINT_ANGLES,
    BASE_INITIAL_POS, BASE_INITIAL_ORN,
    NX, NU, N_HORIZON, T_S, GRAVITY, MU,
    RMSE_THRESH, SLIP_THRESH, MPC_TIME_THRESH, WBC_TIME_THRESH,
)
from robot_model import RobotModel
from state_estimator import StateEstimator
from mpc import CentroidalMPC
from wbc import WholeBodyController
from utils import compute_rmse, plot_com_tracking, plot_contact_force, plot_torques


def skew(v: np.ndarray) -> np.ndarray:
    """向量叉乘矩阵。"""
    return np.array([
        [0.0, -v[2], v[1]],
        [v[2], 0.0, -v[0]],
        [-v[1], v[0], 0.0],
    ])


def compute_centroidal_dynamics(m: float, c: np.ndarray, p_foot: np.ndarray,
                                dt: float) -> tuple:
    """
    在当前工作点处离散化 centroidal dynamics。

    状态 x = [c; c_dot; L]，控制 u = f (3D 线力)
    离散形式: x_{k+1} = A_d x_k + B_d u_k + d_d

    在当前步假设支撑足位置 p_foot 不变，进行欧拉离散化。
    """
    A_d = np.eye(9)
    A_d[0:3, 3:6] = dt * np.eye(3)

    B_d = np.zeros((9, 3))
    B_d[3:6, :] = dt * np.eye(3) / m
    r = p_foot - c
    B_d[6:9, :] = dt * skew(r)

    d_d = np.zeros(9)
    d_d[3:6] = dt * (-GRAVITY)

    return A_d, B_d, d_d


def main():
    # =====================================================================
    # 1. 初始化 PyBullet
    # =====================================================================
    physics_client = p.connect(p.DIRECT)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(*GRAVITY)
    p.setTimeStep(DT_SIM)
    plane_id = p.loadURDF("plane.urdf")

    # =====================================================================
    # 2. 加载机器人
    # =====================================================================
    robot = RobotModel(URDF_PATH)
    candidate_foot_links = [
        robot.link_name_to_index[name] for name in FOOT_LINK_NAMES
    ]
    estimator = StateEstimator(robot, candidate_foot_links)

    # -----------------------------------------------------------------
    # 设置初始姿势（双脚站立）
    # -----------------------------------------------------------------
    robot.reset_base_pose(BASE_INITIAL_POS, BASE_INITIAL_ORN)

    # 按关节名构造初始角度数组，同时建立名称映射
    initial_dof_angles = np.zeros(len(robot.dof_joints))
    joint_name_to_dof_idx = {}
    for idx, joint_idx in enumerate(robot.dof_joints):
        joint_name = robot.joint_info[joint_idx][1].decode('utf-8')
        joint_name_to_dof_idx[joint_name] = idx
        if joint_name in STANDING_JOINT_ANGLES:
            initial_dof_angles[idx] = STANDING_JOINT_ANGLES[joint_name]

    robot.reset_joint_positions(initial_dof_angles)

    # 读取各关节最大力矩
    tau_max_limits = np.zeros(robot.nv - 6)
    for idx, joint_idx in enumerate(robot.dof_joints):
        tau_max_limits[idx] = robot.joint_info[joint_idx][10]
    tau_min_limits = -tau_max_limits

    # 左腿关节索引（用于抬腿动作）
    left_leg_names = [
        "left_hip_pitch_joint",
        "left_hip_roll_joint",
        "left_hip_yaw_joint",
        "left_knee_joint",
        "left_ankle_pitch_joint",
        "left_ankle_roll_joint",
    ]
    left_leg_dof_indices = [
        joint_name_to_dof_idx[name] for name in left_leg_names
        if name in joint_name_to_dof_idx
    ]

    # =====================================================================
    # 3. 初始化控制器
    # =====================================================================
    mpc = CentroidalMPC()
    wbc = WholeBodyController(robot.nv)

    # MPC 参考轨迹（固定点）
    x_ref = np.zeros(NX)
    x_ref[2] = H_COM          # z 方向高度
    u_ref = np.zeros(NU)
    u_ref[2] = -GRAVITY[2] * robot.total_mass

    mpc.set_reference(x_ref, u_ref)

    c_ref = x_ref[:3]
    c_dot_ref = x_ref[3:6]
    L_ref = x_ref[6:9]
    c_ddot_ref = np.zeros(3)
    L_dot_ref = np.zeros(3)

    # =====================================================================
    # 4. 仿真主循环
    # =====================================================================
    wbc_period = max(1, int(1.0 / (WBC_FREQ * DT_SIM)))
    mpc_period = max(1, int(1.0 / (MPC_FREQ * DT_SIM)))

    step = 0
    total_steps = int(SIM_DURATION / DT_SIM)

    # 日志
    time_log = []
    com_log = []
    com_ref_log = []
    foot_pos_log = {link: [] for link in candidate_foot_links}
    foot_force_log = {link: [] for link in candidate_foot_links}
    support_foot_log = []
    mpc_time_log = []
    wbc_time_log = []
    tau_log = []
    mpc_f_ref_log = []
    wbc_f_log = []

    # 记录初始足端位置（用于滑移检测）
    initial_foot_pos = {
        link: robot.get_link_com_position(link)
        for link in candidate_foot_links
    }

    # 控制状态
    leg_lifted = False
    use_mpc_wbc = False
    mpc_result = None
    wbc_result = None

    # 用于 WBC 的 f_ref（MPC 输出，在两次 MPC 求解之间保持）
    f_ref = u_ref.copy()

    print("\n===== 开始仿真（单足站立测试模式）=====")
    print(f"总质量: {robot.total_mass:.2f} kg")
    print(f"仿真时长: {SIM_DURATION:.1f} s")
    print(f"MPC 周期: {mpc_period} 步 ({mpc_period * DT_SIM * 1000:.1f} ms)")
    print(f"WBC 周期: {wbc_period} 步 ({wbc_period * DT_SIM * 1000:.3f} ms)")
    print("=" * 50)

    while step < total_steps:
        t = step * DT_SIM

        # -----------------------------------------------------------------
        # 4.1 状态估计
        # -----------------------------------------------------------------
        state = estimator.update()
        c = state["c"]
        c_dot = state["c_dot"]
        L = state["L"]
        q = state["q"]
        v = state["v"]
        support_foot_link = state["support_foot_link"]
        p_foot = state["p_foot"]
        foot_contacts = state["foot_contacts"]

        x0 = np.concatenate([c, c_dot, L])

        # -----------------------------------------------------------------
        # 4.2 抬腿动作与模式切换
        # -----------------------------------------------------------------
        if not leg_lifted and t >= LIFT_TIME:
            leg_lifted = True
            use_mpc_wbc = True
            print(f"[INFO] t={t:.3f}s 切换至 MPC+WBC 单足站立控制")

        # -----------------------------------------------------------------
        # 4.3 MPC 求解（低频）
        # -----------------------------------------------------------------
        if use_mpc_wbc and step % mpc_period == 0:
            # 更新动力学矩阵
            A_d, B_d, d_d = compute_centroidal_dynamics(
                robot.total_mass, c, p_foot, T_S
            )
            mpc.set_dynamics(A_d, B_d, d_d)

            mpc_result = mpc.solve(x0)
            if mpc_result is not None:
                f_ref = mpc_result["u0"]
                mpc_time_log.append(mpc_result["solve_time"])
            else:
                print(f"[WARN] t={t:.3f}s MPC 求解失败")

        # -----------------------------------------------------------------
        # 4.4 WBC 求解（高频）
        # -----------------------------------------------------------------
        if use_mpc_wbc and step % wbc_period == 0:
            # 计算动力学量
            M = robot.compute_mass_matrix(q)
            C = robot.compute_coriolis_gravity(q, v)
            J_c = robot.get_foot_jacobian(support_foot_link, q)
            J_com = robot.get_com_jacobian(q)
            J_L = robot.get_angular_momentum_jacobian(q)

            # 期望加速度与角动量变化率
            c_ddot_des = wbc.compute_desired_acceleration(
                c_ref, c, c_dot_ref, c_dot, c_ddot_ref
            )
            L_dot_des = wbc.compute_desired_momentum_rate(
                L_ref, L, L_dot_ref, np.zeros(3)
            )

            # Jc_dot 忽略（低速近似）
            Jc_dot = np.zeros((6, robot.nv))

            wbc_result = wbc.solve(
                M, C, J_c, Jc_dot,
                J_com, J_L,
                c_ddot_des, L_dot_des,
                f_ref, v,
                tau_min_limits, tau_max_limits,
            )
            if wbc_result is None:
                print(f"[WARN] t={t:.3f}s WBC 求解失败")

        # -----------------------------------------------------------------
        # 4.5 施加控制指令
        # -----------------------------------------------------------------
        if not use_mpc_wbc:
            # 阶段 1：双腿站立，位置控制
            for idx, joint_idx in enumerate(robot.dof_joints):
                p.setJointMotorControl2(
                    robot.robot_id, joint_idx,
                    p.POSITION_CONTROL,
                    targetPosition=initial_dof_angles[idx],
                    force=500.0,
                )
        else:
            # 阶段 2：单足站立
            # 左腿（抬起的腿）：位置控制跟踪抬腿轨迹
            lift_progress = min(1.0, (t - LIFT_TIME) / 0.5)
            left_targets = initial_dof_angles.copy()
            if "left_hip_pitch_joint" in joint_name_to_dof_idx:
                left_targets[joint_name_to_dof_idx["left_hip_pitch_joint"]] = 0.3 * lift_progress
            if "left_knee_joint" in joint_name_to_dof_idx:
                left_targets[joint_name_to_dof_idx["left_knee_joint"]] = -0.6 * lift_progress

            for idx, joint_idx in enumerate(robot.dof_joints):
                if idx in left_leg_dof_indices:
                    # 抬起的腿：位置控制
                    p.setJointMotorControl2(
                        robot.robot_id, joint_idx,
                        p.POSITION_CONTROL,
                        targetPosition=left_targets[idx],
                        force=100.0,
                    )
                else:
                    # 支撑腿及其他关节：力矩控制
                    if wbc_result is not None:
                        tau_cmd = float(wbc_result["tau"][idx])
                    else:
                        tau_cmd = 0.0
                    p.setJointMotorControl2(
                        robot.robot_id, joint_idx,
                        p.TORQUE_CONTROL,
                        force=tau_cmd,
                    )

        # -----------------------------------------------------------------
        # 4.6 单步仿真推进
        # -----------------------------------------------------------------
        p.stepSimulation()
        step += 1

        # -----------------------------------------------------------------
        # 4.7 记录数据
        # -----------------------------------------------------------------
        time_log.append(t)
        com_log.append(c.copy())
        com_ref_log.append(x_ref[:3].copy())
        support_foot_log.append(support_foot_link)

        for fc in foot_contacts:
            link = fc["link"]
            foot_pos_log[link].append(fc["position"].copy())
            foot_force_log[link].append(fc["normal_force"])

        if wbc_result is not None:
            tau_log.append(wbc_result["tau"].copy())
            wbc_f_log.append(wbc_result["f"].copy())
            wbc_time_log.append(wbc_result["solve_time"])
        else:
            # 填充零以保持长度一致
            tau_log.append(np.zeros(robot.nv - 6))
            wbc_f_log.append(np.zeros(3))

        if mpc_result is not None:
            mpc_f_ref_log.append(f_ref.copy())
        else:
            mpc_f_ref_log.append(np.zeros(3))

        # -----------------------------------------------------------------
        # 4.8 周期性打印状态
        # -----------------------------------------------------------------
        if step % 240 == 0:  # 每秒打印一次
            print(f"\n--- t={t:.3f}s ---")
            print(f"  CoM: [{c[0]:.3f}, {c[1]:.3f}, {c[2]:.3f}]  (ref z={H_COM:.2f})")
            mpc_t = mpc_time_log[-1] * 1000 if mpc_time_log else 0.0
            wbc_t = wbc_time_log[-1] * 1000 if wbc_time_log else 0.0
            print(f"  MPC solve: {mpc_t:.2f} ms | WBC solve: {wbc_t:.2f} ms")
            for fc in foot_contacts:
                link_name = FOOT_LINK_NAMES[candidate_foot_links.index(fc["link"])]
                slip = np.linalg.norm(fc["position"][:2] - initial_foot_pos[fc["link"]][:2])
                print(f"  {link_name}: force={fc['normal_force']:.1f}N  slip={slip*1000:.2f}mm")

    # =====================================================================
    # 5. 评估指标
    # =====================================================================
    rmse = compute_rmse(com_log, com_ref_log)
    print(f"\n{'='*50}")
    print("===== 实验结果 =====")
    print(f"CoM 位置 RMSE: {rmse:.4f} m (目标 < {RMSE_THRESH} m)")

    # 足端滑移
    max_slip = 0.0
    for link in candidate_foot_links:
        positions = np.array(foot_pos_log[link])
        if len(positions) > 0:
            slips = np.linalg.norm(positions[:, :2] - initial_foot_pos[link][:2], axis=1)
            max_slip = max(max_slip, np.max(slips))
    print(f"最大足端滑移: {max_slip*1000:.2f} mm (目标 < {SLIP_THRESH*1000:.1f} mm)")

    # 接触力统计
    for link in candidate_foot_links:
        forces = np.array(foot_force_log[link])
        link_name = FOOT_LINK_NAMES[candidate_foot_links.index(link)]
        avg_force = np.mean(forces) if len(forces) > 0 else 0.0
        print(f"{link_name} 平均接触力: {avg_force:.1f} N")

    # MPC / WBC 求解时间
    if mpc_time_log:
        avg_mpc_time = np.mean(mpc_time_log) * 1000
        max_mpc_time = np.max(mpc_time_log) * 1000
        print(f"MPC 平均求解时间: {avg_mpc_time:.2f} ms (目标 < {MPC_TIME_THRESH*1000:.1f} ms)")
        print(f"MPC 最大求解时间: {max_mpc_time:.2f} ms")
    if wbc_time_log:
        avg_wbc_time = np.mean(wbc_time_log) * 1000
        max_wbc_time = np.max(wbc_time_log) * 1000
        print(f"WBC 平均求解时间: {avg_wbc_time:.3f} ms (目标 < {WBC_TIME_THRESH*1000:.1f} ms)")
        print(f"WBC 最大求解时间: {max_wbc_time:.3f} ms")

    # =====================================================================
    # 6. 可视化
    # =====================================================================
    plot_com_tracking(time_log, com_log, com_ref_log)

    if len(wbc_f_log) > 0:
        plot_contact_force(time_log, wbc_f_log)

    if len(tau_log) > 0:
        plot_torques(time_log, tau_log)

    p.disconnect()


if __name__ == "__main__":
    main()
