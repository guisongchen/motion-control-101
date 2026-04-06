"""
项目1.1: PyBullet 2D双足站稳 - 改进版
目标: 让双足机器人在仿真中稳定站立

改进的控制策略:
1. 分层控制架构: 质心控制 -> 关节分配
2. 高度控制: 通过膝盖屈伸保持质心高度
3. 姿态控制: 通过踝关节调整保持直立 (类似倒立摆)
4. 前向速度控制: 通过髋关节调整防止漂移

验收标准:
- [x] 机器人能在原地站立5秒不倒
- [ ] 能抵抗小扰动（用手推不倒）
- [x] 理解PD参数对稳定性的影响
"""

import pybullet as p
import pybullet_data
import numpy as np
import time


class BalanceController:
    """
    改进的双足平衡控制器

    控制策略:
    1. 高度控制: 根据torso高度误差调整膝盖角度
    2. 姿态控制: 根据pitch误差调整踝关节角度 (主要稳定手段)
    3. 位置控制: 保持髋关节在站立位置
    """

    def __init__(self):
        # 高度控制参数 (通过膝盖) - 保守值避免数值不稳定
        self.height_kp = 100.0
        self.height_kd = 20.0
        self.target_height = 0.82  # 目标站立高度

        # 姿态控制参数 (通过踝关节, 最关键!)
        self.pitch_kp = 80.0  # 适度增益
        self.pitch_kd = 10.0

        # 位置控制参数 (通过髋关节, 防止漂移)
        self.hip_kp = 30.0
        self.hip_kd = 3.0

        # 关节角度限制
        self.max_knee_angle = np.radians(45)  # 膝盖最大弯曲
        self.max_ankle_angle = np.radians(20)  # 脚踝最大角度

    def compute_joint_targets(self, height, height_vel, pitch, pitch_vel,
                               hip_pos, hip_vel, foot_contact):
        """
        计算关节目标角度

        Args:
            height: 当前 torso 高度
            height_vel: 高度变化率
            pitch: 当前倾斜角 (rad)
            pitch_vel: 倾斜角速度
            hip_pos: 当前髋关节角度
            hip_vel: 髋关节角速度
            foot_contact: 脚是否接触地面

        Returns:
            dict: 各关节目标角度
        """
        targets = {}

        # === 1. 姿态控制 (最关键!) ===
        # 如果身体前倾(pitch>0), 踝关节要向后转(负角度)来推身体回去
        # 类似倒立摆: 底部施加力矩来保持直立
        ankle_target = -self.pitch_kp * pitch - self.pitch_kd * pitch_vel
        ankle_target = np.clip(ankle_target, -self.max_ankle_angle, self.max_ankle_angle)

        targets['right_ankle'] = ankle_target
        targets['left_ankle'] = ankle_target

        # === 2. 高度控制 ===
        # 如果太低, 弯曲膝盖来抬高身体
        height_error = self.target_height - height
        knee_bend = self.height_kp * height_error - self.height_kd * height_vel
        knee_bend = np.clip(knee_bend, 0, self.max_knee_angle)

        # 膝盖弯曲时, 髋关节也要相应调整保持身体垂直
        targets['right_knee'] = knee_bend
        targets['left_knee'] = knee_bend

        # === 3. 髋关节位置控制 ===
        # 保持髋关节在站立角度, 防止身体漂移
        hip_target = -knee_bend * 0.5  # 补偿膝盖弯曲带来的姿态变化
        hip_target += self.hip_kp * (-hip_pos) - self.hip_kd * hip_vel  # 保持直立

        targets['right_hip'] = hip_target
        targets['left_hip'] = hip_target

        return targets


class BipedSimulation:
    """双足机器人仿真环境"""

    # 关节名称到索引的映射 (根据humanoid.urdf)
    JOINT_MAP = {
        'chest': 1,
        'neck': 2,
        'right_shoulder': 3,
        'right_elbow': 4,
        'left_shoulder': 6,
        'left_elbow': 7,
        'right_hip': 9,
        'right_knee': 10,
        'right_ankle': 11,
        'left_hip': 12,
        'left_knee': 13,
        'left_ankle': 14,
    }

    def __init__(self, render=True):
        """初始化仿真环境"""
        self.render = render

        # 连接PyBullet
        if render:
            self.physics_client = p.connect(p.GUI)
        else:
            self.physics_client = p.connect(p.DIRECT)

        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)

        # 加载地面
        self.plane_id = p.loadURDF("plane.urdf")

        # 加载双足机器人
        start_pos = [0, 0, 0.88]  # 合适的初始高度
        start_orientation = p.getQuaternionFromEuler([0, 0, 0])

        self.robot_id = p.loadURDF(
            "humanoid/humanoid.urdf",
            start_pos,
            start_orientation,
            useFixedBase=False
        )

        self.num_joints = p.getNumJoints(self.robot_id)

        # 创建控制器
        self.controller = BalanceController()

        # 记录数据
        self.time_history = []
        self.height_history = []
        self.pitch_history = []
        self.ankle_torque_history = []

        # 先稳定站立姿态 (关键!)
        self._init_standing_pose()

    def _init_standing_pose(self):
        """
        初始化站立姿态
        在启动控制器前，让机器人先处于合理的站立姿势
        """
        # 重置关节到站立位置
        standing_angles = {
            'right_hip': np.radians(-5),
            'right_knee': np.radians(10),
            'right_ankle': np.radians(-5),
            'left_hip': np.radians(-5),
            'left_knee': np.radians(10),
            'left_ankle': np.radians(-5),
        }

        for name, angle in standing_angles.items():
            idx = self.JOINT_MAP[name]
            p.resetJointState(self.robot_id, idx, angle)

        # 初始几步让机器人稳定
        for _ in range(100):
            p.stepSimulation()

        print("站立姿态初始化完成")

    def _reset_joint_control(self):
        """禁用默认电机控制，准备用力矩控制"""
        for i in range(self.num_joints):
            p.setJointMotorControl2(
                bodyUniqueId=self.robot_id,
                jointIndex=i,
                controlMode=p.VELOCITY_CONTROL,
                force=0
            )

    def _get_robot_state(self):
        """获取机器人完整状态"""
        # 基座状态
        pos, orn = p.getBasePositionAndOrientation(self.robot_id)
        vel, ang_vel = p.getBaseVelocity(self.robot_id)

        # 转换为欧拉角
        euler = p.getEulerFromQuaternion(orn)

        # 关节状态
        joint_states = p.getJointStates(self.robot_id, range(self.num_joints))

        state = {
            'height': pos[2],
            'height_vel': vel[2],
            'pitch': euler[1],  # 前后倾斜
            'pitch_vel': ang_vel[1],
            'roll': euler[0],
            'yaw': euler[2],
            'joint_positions': [s[0] for s in joint_states],
            'joint_velocities': [s[1] for s in joint_states],
        }

        return state

    def _check_foot_contact(self):
        """检查脚是否接触地面"""
        # 获取脚链接的索引 (humanoid模型中)
        contact_points = p.getContactPoints(self.robot_id, self.plane_id)

        right_contact = False
        left_contact = False

        for cp in contact_points:
            link_index = cp[3]  # 机器人上的链接索引
            # 根据URDF结构判断是左脚还是右脚
            if link_index in [11, 10]:  # 右腿相关
                right_contact = True
            if link_index in [14, 13]:  # 左腿相关
                left_contact = True

        return right_contact or left_contact

    def _apply_control(self, targets):
        """
        应用关节目标角度控制

        使用PD位置控制 (比纯力矩控制更稳定)
        """
        joint_map = self.JOINT_MAP

        # 腿部控制 (关键关节)
        leg_joints = ['right_hip', 'right_knee', 'right_ankle',
                      'left_hip', 'left_knee', 'left_ankle']

        for joint_name in leg_joints:
            if joint_name in targets:
                idx = joint_map[joint_name]
                target = targets[joint_name]

                # 使用位置控制 + 力矩限制
                p.setJointMotorControl2(
                    bodyUniqueId=self.robot_id,
                    jointIndex=idx,
                    controlMode=p.POSITION_CONTROL,
                    targetPosition=target,
                    positionGain=100.0,
                    velocityGain=10.0,
                    force=500.0  # 最大力矩限制
                )

        # 手臂保持下垂 (减少晃动)
        arm_joints = ['right_shoulder', 'left_shoulder']
        for joint_name in arm_joints:
            idx = joint_map[joint_name]
            p.setJointMotorControl2(
                bodyUniqueId=self.robot_id,
                jointIndex=idx,
                controlMode=p.POSITION_CONTROL,
                targetPosition=0.0,
                positionGain=50.0,
                force=100.0
            )

    def step(self):
        """仿真一步"""
        # 获取状态
        state = self._get_robot_state()

        # 获取髋关节角度 (用于位置控制)
        hip_pos = state['joint_positions'][self.JOINT_MAP['right_hip']]
        hip_vel = state['joint_velocities'][self.JOINT_MAP['right_hip']]

        # 计算控制目标
        targets = self.controller.compute_joint_targets(
            height=state['height'],
            height_vel=state['height_vel'],
            pitch=state['pitch'],
            pitch_vel=state['pitch_vel'],
            hip_pos=hip_pos,
            hip_vel=hip_vel,
            foot_contact=self._check_foot_contact()
        )

        # 应用控制
        self._apply_control(targets)

        # 步进仿真
        p.stepSimulation()

        return state['height'], state['pitch']

    def apply_disturbance(self, force, duration=0.1):
        """
        施加外部扰动力 (用于测试稳定性)

        Args:
            force: [fx, fy, fz] 力向量 (N)
            duration: 持续时间 (秒)
        """
        p.applyExternalForce(
            objectUniqueId=self.robot_id,
            linkIndex=-1,  # 基座
            forceObj=force,
            posObj=[0, 0, 0.5],  # 施加在腰部位置
            flags=p.LINK_FRAME
        )

    def run(self, duration=5.0, dt=0.01, test_disturbance=False):
        """
        运行仿真

        Args:
            duration: 仿真时长
            dt: 时间步长
            test_disturbance: 是否在中间施加扰动测试
        """
        print(f"\n{'='*50}")
        print("双足平衡控制仿真 (改进版)")
        print(f"{'='*50}")
        print(f"目标高度: {self.controller.target_height}m")
        print(f"高度控制: Kp={self.controller.height_kp}, Kd={self.controller.height_kd}")
        print(f"姿态控制: Kp={self.controller.pitch_kp}, Kd={self.controller.pitch_kd}")
        print(f"按 Ctrl+C 停止仿真\n")

        # 重置控制模式
        self._reset_joint_control()

        steps = int(duration / dt)
        disturbance_applied = False

        try:
            for step in range(steps):
                t = step * dt

                # 执行一步
                height, pitch = self.step()

                # 记录数据
                self.time_history.append(t)
                self.height_history.append(height)
                self.pitch_history.append(pitch)

                # 在2.5秒时施加扰动 (如果开启测试)
                if test_disturbance and not disturbance_applied and t > 2.5:
                    print(f"\n[扰动测试] 在 {t:.2f}s 施加向前的推力!")
                    self.apply_disturbance([50, 0, 0], 0.2)
                    disturbance_applied = True

                # 实时打印
                if step % 100 == 0:
                    status = "✓" if height > 0.6 else "✗"
                    print(f"[{status}] t={t:.2f}s | 高度: {height:.3f}m | 倾斜: {np.degrees(pitch):5.2f}°")

                # 检查是否摔倒
                if height < 0.5:
                    print(f"\n机器人在 {t:.2f}秒 摔倒!")
                    return False

                # 渲染延时
                if self.render:
                    time.sleep(dt)

        except KeyboardInterrupt:
            print("\n用户中断仿真")
            return None

        print(f"\n✓ 成功站立 {duration} 秒!")
        self._print_stats()
        return True

    def _print_stats(self):
        """打印统计信息"""
        if len(self.height_history) == 0:
            return

        print(f"\n{'='*50}")
        print("仿真统计")
        print(f"{'='*50}")
        print(f"站立时长: {self.time_history[-1]:.2f}秒")
        print(f"平均高度: {np.mean(self.height_history):.3f}m")
        print(f"高度范围: [{np.min(self.height_history):.3f}, {np.max(self.height_history):.3f}]m")
        print(f"平均倾斜: {np.degrees(np.mean(self.pitch_history)):2.2f}°")
        print(f"最大倾斜: {np.degrees(np.max(np.abs(self.pitch_history))):2.2f}°")
        print(f"{'='*50}\n")

    def disconnect(self):
        p.disconnect()


def main():
    """主函数"""
    # 参数
    RENDER = True
    DURATION = 5.0
    TEST_DISTURBANCE = False  # 设为True测试抗扰动能力

    sim = BipedSimulation(render=RENDER)

    try:
        success = sim.run(duration=DURATION, test_disturbance=TEST_DISTURBANCE)

        if success:
            print("✓ 验收标准通过: 站立5秒不倒")
        elif success is False:
            print("✗ 验收标准未通过: 提前摔倒")

    finally:
        sim.disconnect()


if __name__ == "__main__":
    main()
