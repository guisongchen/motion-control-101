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


def main():
    # =====================================================================
    # 1. 初始化 PyBullet
    # =====================================================================
    physics_client = p.connect(p.GUI)  # 或 p.DIRECT
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

    # 按关节名构造初始角度数组
    initial_dof_angles = np.zeros(len(robot.dof_joints))
    for idx, joint_idx in enumerate(robot.dof_joints):
        joint_name = robot.joint_info[joint_idx][1].decode('utf-8')
        if joint_name in STANDING_JOINT_ANGLES:
            initial_dof_angles[idx] = STANDING_JOINT_ANGLES[joint_name]

    robot.reset_joint_positions(initial_dof_angles)

    # 设置所有自由度关节为位置控制模式（测试双足站立姿态）
    for idx, joint_idx in enumerate(robot.dof_joints):
        p.setJointMotorControl2(
            robot.robot_id, joint_idx,
            p.POSITION_CONTROL,
            targetPosition=initial_dof_angles[idx],
            force=500.0,
        )

    # =====================================================================
    # 3. 初始化控制器（暂不启用）
    # =====================================================================
    mpc = CentroidalMPC()
    nv = robot.nv
    wbc = WholeBodyController(nv)

    # MPC 参考轨迹（固定点）
    x_ref = np.zeros(NX)
    x_ref[2] = H_COM          # z 方向高度
    u_ref = np.zeros(NU)
    u_ref[2] = -GRAVITY[2] * robot.total_mass

    mpc.set_reference(x_ref, u_ref)

    # =====================================================================
    # 4. 仿真主循环
    # =====================================================================
    wbc_period = max(1, int(1.0 / (WBC_FREQ * DT_SIM)))   # 不低于物理步长
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

    # 记录初始足端位置（用于滑移检测）
    initial_foot_pos = {
        link: robot.get_link_com_position(link)
        for link in candidate_foot_links
    }

    # t=0.5s 抬腿动作（预设轨迹，非 MPC/WBC 控制）
    leg_lifted = False

    print("\n===== 开始仿真（双足站立测试模式）=====")
    print(f"总质量: {robot.total_mass:.2f} kg")
    print(f"仿真时长: {SIM_DURATION:.1f} s")
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
        foot_contacts = state["foot_contacts"]

        # -----------------------------------------------------------------
        # 4.2 MPC / WBC 求解（当前禁用，仅保留占位）
        # -----------------------------------------------------------------
        # TODO: 实现 MPC + WBC 闭环控制
        # x0 = np.concatenate([c, c_dot, L])
        # if step % mpc_period == 0:
        #     mpc_result = mpc.solve(x0)
        # if step % wbc_period == 0:
        #     wbc_result = wbc.solve(...)

        # -----------------------------------------------------------------
        # 4.3 维持初始姿态（POSITION_CONTROL 测试模式）
        # -----------------------------------------------------------------
        if step % wbc_period == 0:
            for idx, joint_idx in enumerate(robot.dof_joints):
                p.setJointMotorControl2(
                    robot.robot_id, joint_idx,
                    p.POSITION_CONTROL,
                    targetPosition=initial_dof_angles[idx],
                    force=500.0,
                )

        # -----------------------------------------------------------------
        # 4.4 抬腿动作（t=0.5s）
        # -----------------------------------------------------------------
        if not leg_lifted and t >= LIFT_TIME:
            # TODO: 执行抬腿动作（通过关节位置指令或预设轨迹）
            leg_lifted = True
            print(f"[INFO] t={t:.3f}s 抬起 {LIFT_LEG} 腿")

        # -----------------------------------------------------------------
        # 4.5 单步仿真推进
        # -----------------------------------------------------------------
        p.stepSimulation()
        step += 1

        # -----------------------------------------------------------------
        # 4.6 记录数据
        # -----------------------------------------------------------------
        time_log.append(t)
        com_log.append(c.copy())
        com_ref_log.append(x_ref[:3].copy())
        support_foot_log.append(support_foot_link)

        for fc in foot_contacts:
            link = fc["link"]
            foot_pos_log[link].append(fc["position"].copy())
            foot_force_log[link].append(fc["normal_force"])

        # -----------------------------------------------------------------
        # 4.7 周期性打印状态
        # -----------------------------------------------------------------
        if step % 240 == 0:  # 每秒打印一次
            print(f"\n--- t={t:.3f}s ---")
            print(f"  CoM: [{c[0]:.3f}, {c[1]:.3f}, {c[2]:.3f}]  (ref z={H_COM:.2f})")
            for fc in foot_contacts:
                link_name = FOOT_LINK_NAMES[candidate_foot_links.index(fc["link"])]
                slip = np.linalg.norm(fc["position"][:2] - initial_foot_pos[fc["link"]][:2])
                print(f"  {link_name}: force={fc['normal_force']:.1f}N  slip={slip*1000:.2f}mm")

    # =====================================================================
    # 5. 评估指标
    # =====================================================================
    rmse = compute_rmse(com_log, com_ref_log)
    print(f"\n{'='*50}")
    print("===== 实验结果（双足站立测试模式）=====")
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

    # =====================================================================
    # 6. 可视化
    # =====================================================================
    plot_com_tracking(time_log, com_log, com_ref_log)

    p.disconnect()


if __name__ == "__main__":
    main()
