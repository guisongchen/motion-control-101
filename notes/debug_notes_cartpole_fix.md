# Cart-Pole 小车不动问题分析与解决方案

## 问题现象

运行 `cartpole_balance.py` 时，小车完全没有移动：
- 小车位置始终是 `Pos: +0.0000m`
- 小车速度始终是 `0.0000m/s`
- 摆杆角度没有变化
- 施加的控制力没有产生任何效果

## 根本原因

PyBullet 的 **滑动关节（Prismatic Joint）默认启用了位置保持电机**。这意味着：

1. 关节被内部电机锁定在初始位置
2. 即使施加外力或使用 `TORQUE_CONTROL`，关节也不会移动
3. 必须先"释放"关节控制，才能让关节自由运动

### URDF 结构分析

PyBullet 内置 `cartpole.urdf` 的结构：

```
base (固定在世界坐标系)
  └── 关节 0: slider_to_cart (滑动关节, Type=1) → Link: cart
        └── 关节 1: cart_to_pole (旋转关节, Type=0) → Link: pole
```

**关键发现**：
- **关节 0** 是滑动关节，其 `joint_position` 就是小车的水平位置
- **关节 1** 是旋转关节，其 `joint_position` 就是摆杆角度
- 小车不是 base，不能通过 `getBasePositionAndOrientation` 获取位置

## 解决方案

### 关键代码：释放关节控制

在初始化时，必须将关节设为自由模式：

```python
# 释放滑动关节（小车）
p.setJointMotorControl2(cartpole, 0, p.VELOCITY_CONTROL,
                        targetVelocity=0, force=0)

# 释放旋转关节（摆杆）
p.setJointMotorControl2(cartpole, 1, p.VELOCITY_CONTROL,
                        targetVelocity=0, force=0)
```

**原理**：
- `VELOCITY_CONTROL` 模式：使用速度控制
- `targetVelocity=0`：目标速度设为 0
- `force=0`：电机输出力设为 0，相当于禁用电机

### 正确的状态获取

```python
def get_cartpole_state(cartpole):
    # 小车位置 = 滑动关节(0)的位置
    cart_joint_state = p.getJointState(cartpole, 0)
    cart_position = cart_joint_state[0]
    cart_velocity = cart_joint_state[1]

    # 摆杆角度 = 旋转关节(1)的位置
    pole_joint_state = p.getJointState(cartpole, 1)
    angle = pole_joint_state[0]
    angular_velocity = pole_joint_state[1]

    return cart_position, angle, angular_velocity, cart_velocity
```

### 正确的控制方式

使用 `TORQUE_CONTROL` 模式施加力：

```python
force = pd_controller(angle, ang_vel, pos, cart_vel)
p.setJointMotorControl2(cartpole, 0, p.TORQUE_CONTROL, force=force)
```

## 调试技巧

### 1. 打印 URDF 结构

```python
def debug_urdf_structure(cartpole):
    num_joints = p.getNumJoints(cartpole)
    print(f"关节数量: {num_joints}")

    for i in range(num_joints):
        joint_info = p.getJointInfo(cartpole, i)
        link_name = joint_info[12].decode('utf-8')
        joint_name = joint_info[1].decode('utf-8')
        joint_type = joint_info[2]
        print(f"  关节 {i}: Link='{link_name}', Joint='{joint_name}', Type={joint_type}")
```

关节类型代码：
- `0` = JOINT_REVOLUTE (旋转关节)
- `1` = JOINT_PRISMATIC (滑动关节)
- `4` = JOINT_FIXED (固定关节)

### 2. 监控关节状态

```python
# 每帧打印关节位置和速度
for i in range(num_joints):
    joint_state = p.getJointState(cartpole, i)
    print(f"关节 {i}: Pos={joint_state[0]:.4f}, Vel={joint_state[1]:.4f}")
```

## 完整修复后的代码结构

```python
# 1. 加载 URDF
cartpole = p.loadURDF("cartpole.urdf", [0, 0, 0.1])

# 2. 【关键】释放关节控制
p.setJointMotorControl2(cartpole, 0, p.VELOCITY_CONTROL, targetVelocity=0, force=0)
p.setJointMotorControl2(cartpole, 1, p.VELOCITY_CONTROL, targetVelocity=0, force=0)

# 3. 仿真循环
for step in range(SIMULATION_STEPS):
    # 获取状态
    pos, angle, ang_vel, cart_vel = get_cartpole_state(cartpole)

    # 计算控制力
    force = pd_controller(angle, ang_vel, pos, cart_vel)

    # 施加控制
    p.setJointMotorControl2(cartpole, 0, p.TORQUE_CONTROL, force=force)

    p.stepSimulation()
```

## 经验教训

1. **PyBullet 关节默认不是自由的**：许多 URDF 模型的关节默认有位置保持，需要显式释放
2. **理解 URDF 结构很重要**：要先弄清楚哪个关节控制什么运动
3. **使用正确的 API**：
   - 关节信息用 `getJointInfo`
   - 关节状态用 `getJointState`
   - 关节控制用 `setJointMotorControl2`
4. **调试时打印关节状态**：位置、速度是否为 0 能快速定位问题
