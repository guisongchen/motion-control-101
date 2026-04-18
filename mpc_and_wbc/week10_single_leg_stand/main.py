"""主循环：MPC + WBC + PyBullet"""

import time
import numpy as np
import pybullet as p
import pybullet_data

from config import (
    DT_SIM, WBC_FREQ, MPC_FREQ, SIM_DURATION,
    URDF_PATH, INITIAL_POSE, LIFT_LEG, LIFT_TIME, H_COM,
    SUPPORT_FOOT_NAME,
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
    support_foot_link = robot.link_name_to_index[SUPPORT_FOOT_NAME]
    estimator = StateEstimator(robot, support_foot_link)

    # TODO: 设置初始姿势（双脚站立）
    # robot.reset_joint_positions(...)

    # =====================================================================
    # 3. 初始化控制器
    # =====================================================================
    mpc = CentroidalMPC()
    nv = robot.nv
    wbc = WholeBodyController(nv)

    # MPC 参考轨迹（固定点）
    x_ref = np.zeros(NX)
    x_ref[2] = H_COM          # z 方向高度
    u_ref = np.zeros(NU)
    # 补偿重力
    u_ref[2] = -GRAVITY[2] * robot.total_mass

    mpc.set_reference(x_ref, u_ref)

    # =====================================================================
    # 4. 仿真主循环
    # =====================================================================
    wbc_period = int(1.0 / (WBC_FREQ * DT_SIM))   # 每多少个物理步执行一次 WBC
    mpc_period = int(1.0 / (MPC_FREQ * DT_SIM))   # 每多少个物理步执行一次 MPC

    step = 0
    total_steps = int(SIM_DURATION / DT_SIM)

    # 日志
    time_log = []
    com_log = []
    com_ref_log = []
    force_log = []
    tau_log = []
    mpc_time_log = []
    wbc_time_log = []

    # t=0.5s 抬腿动作（预设轨迹，非 MPC/WBC 控制）
    leg_lifted = False

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
        x0 = np.concatenate([c, c_dot, L])

        # -----------------------------------------------------------------
        # 4.2 MPC 重求解（50 Hz）
        # -----------------------------------------------------------------
        if step % mpc_period == 0:
            # TODO: 更新动力学线性化矩阵 A_d, B_d, d_d
            # mpc.set_dynamics(A_d, B_d, d_d)

            t0 = time.time()
            mpc_result = mpc.solve(x0)
            mpc_time = time.time() - t0
            mpc_time_log.append(mpc_time)

            if mpc_result is not None:
                x_traj = mpc_result["x_traj"]
                f_ref = mpc_result["u0"]
            else:
                print(f"[WARN] t={t:.3f}s MPC 求解失败")
                f_ref = u_ref

        # -----------------------------------------------------------------
        # 4.3 WBC（1 kHz）
        # -----------------------------------------------------------------
        if step % wbc_period == 0:
            # TODO: 计算当前动力学量
            # M = robot.compute_mass_matrix(q)
            # C = robot.compute_coriolis_gravity(q, v)
            # J_c = robot.get_foot_jacobian(support_foot_link, q)
            # Jc_dot = ...
            # J_com = robot.get_com_jacobian(q)
            # J_L = robot.get_angular_momentum_jacobian(q)

            # TODO: 计算期望量
            # c_ddot_des = wbc.compute_desired_acceleration(...)
            # L_dot_des = wbc.compute_desired_momentum_rate(...)

            t0 = time.time()
            # wbc_result = wbc.solve(...)
            wbc_time = time.time() - t0
            wbc_time_log.append(wbc_time)

            # if wbc_result is not None:
            #     tau = wbc_result["tau"]
            #     f = wbc_result["f"]
            # else:
            #     print(f"[WARN] t={t:.3f}s WBC 求解失败")
            #     tau = np.zeros(robot.num_joints)
            #     f = np.zeros(3)

            # TODO: 发送力矩指令到 PyBullet
            # for i, joint_id in enumerate(controlled_joints):
            #     p.setJointMotorControl2(robot.robot_id, joint_id,
            #                             p.TORQUE_CONTROL, force=tau[i])

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
        # force_log.append(f.copy())
        # tau_log.append(tau.copy())

    # =====================================================================
    # 5. 评估指标
    # =====================================================================
    rmse = compute_rmse(com_log, com_ref_log)
    print(f"\n===== 实验结果 =====")
    print(f"CoM 位置 RMSE: {rmse:.4f} m (目标 < {RMSE_THRESH} m)")
    print(f"MPC 平均求解时间: {np.mean(mpc_time_log)*1000:.2f} ms (目标 < {MPC_TIME_THRESH*1000:.1f} ms)")
    print(f"WBC 平均求解时间: {np.mean(wbc_time_log)*1000:.3f} ms (目标 < {WBC_TIME_THRESH*1000:.1f} ms)")
    # TODO: 计算支撑足滑移

    # =====================================================================
    # 6. 可视化
    # =====================================================================
    plot_com_tracking(time_log, com_log, com_ref_log)
    # plot_contact_force(time_log, force_log)
    # plot_torques(time_log, tau_log)

    p.disconnect()


if __name__ == "__main__":
    main()
