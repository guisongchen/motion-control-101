# 双足平衡控制 - 循序渐进方案

**核心思路**: 从最简单的可控系统开始，逐步增加复杂度，每一步都确保可控。

---

## 阶段 0: 理解模型 (当前 - 必须完成)

**目标**: 搞懂 PyBullet humanoid.urdf 的关节方向和动力学特性

### 0.1 单关节测试

```python
# 测试每个关节的正方向
test_joints = {
    'r_knee': +1.0,    # 观察弯曲方向
    'r_ankle': +0.5,   # 观察脚尖移动方向
    'r_hip': -0.3,     # 观察大腿摆动方向
}
```

**关键问题要回答**:
- 膝盖正角度 → 向前弯还是向后弯？
- 脚踝正角度 → 脚尖上翘还是下压？
- 髋关节正角度 → 大腿前摆还是后摆？

### 0.2 静态平衡分析

```python
# 在无控制情况下，观察质心位置
# 打印: base position, base orientation, 各关节角度
# 分析: 质心相对于脚的位置
```

**输出**: 一张关节方向对照表

---

## 阶段 1: 固定脚踝单腿 (简化到核心)

**目标**: 消除浮动基座问题，先验证关节控制逻辑

### 1.1 创建简化 URDF

基于 humanoid.urdf，修改:
```xml
<!-- 将双脚固定在地面 -->
<!-- 或使用 createConstraint 将 base 限制在 x-z 平面 -->
```

或者用代码约束:
```python
# 创建 prismatic + revolute 约束，限制只在 sagittal 平面运动
# 固定 y 位置和绕 x/z 轴的旋转
```

### 1.2 验证控制逻辑

```python
# 控制目标: 保持 torso 直立 (pitch = 0)
# 控制量: hip 和 knee 角度
# 脚踝: 固定或被动
```

**成功标准**: 能在固定脚踝条件下保持直立 5 秒

---

## 阶段 2: 虚拟 2D 双足 ( sagittal 平面 )

**目标**: 完整双足但在 2D 平面运动，降低复杂度

### 2.1 约束设置

```python
# 每帧重置 base 的 y 位置和 roll/yaw 旋转
def constrain_to_2d():
    pos, orn = p.getBasePositionAndOrientation(robot)
    # 保持 y = 0, roll = 0, yaw = 0
    # 只保留 x, z, pitch (绕 y 轴)
```

### 2.2 分层控制

```
Layer 1: 质心高度控制
  └── 通过双膝同步弯曲/伸展

Layer 2: 质心水平位置控制 (相对脚)
  └── 通过髋-膝协调 (hip strategy)

Layer 3: 躯干姿态控制
  └── 通过脚踝角度调整
```

**成功标准**: 能在 2D 约束下稳定站立 10 秒

---

## 阶段 3: 引入 ZMP 概念

**目标**: 显式控制压力中心 (ZMP) 在支撑多边形内

### 3.1 ZMP 计算

```python
def calculate_zmp():
    # 基于脚底接触力计算 ZMP
    contact_points = p.getContactPoints(robot, plane, foot_link)
    # ZMP = (sum(force_i * pos_i)) / sum(force_i)
    return zmp_x, zmp_y

def get_support_polygon():
    # 根据双脚接触点计算支撑区域
    return polygon_bounds
```

### 3.2 ZMP 跟踪控制

```python
# 目标: 保持 ZMP 在支撑多边形中心附近
# 方法: 调整 torso 倾斜角度来移动 ZMP
zmp_error = zmp_target - zmp_current
ankle_compensation = kp_zmp * zmp_error
```

**成功标准**: ZMP 保持在支撑多边形内，机器人不倒

---

## 阶段 4: 完整 3D 双足

**目标**: 移除 2D 约束，实现完整 3D 平衡

### 4.1 额外自由度控制

```
新增控制:
- 侧向平衡 (y 方向): 通过髋关节外展/内收
- 偏航控制: 通过双脚不对称调整
- 横滚控制: 通过脚踝外翻/内翻
```

### 4.2 双脚协调

```python
def distribute_control():
    # 根据负重分配左右脚控制量
    left_weight = left_contact_force / total_weight
    right_weight = right_contact_force / total_weight
    # 控制量加权分配
```

---

## 快速验证路径 (推荐)

如果希望快速看到效果，建议按此顺序:

```
Day 1: 阶段 0 → 搞懂关节方向
Day 2: 阶段 1 → 固定脚踝，单腿稳定
Day 3: 阶段 2 → 2D 约束双足稳定
Day 4+: 阶段 3-4 → 逐步增加复杂度
```

---

## 备选方案: 先用 Cart-Pole

如果双足仍然太复杂，可以先验证控制逻辑在 Cart-Pole:

```python
# Cart-Pole 环境
import gym
cartpole = gym.make('CartPole-v1')

# 相同的 PD 控制逻辑
# 成功后再迁移到双足
```

**优势**:
- 状态空间小 (4D vs 36D+)
- 控制理论成熟
- 调试容易

---

## 需要立即执行的下一步

1. **创建单关节测试脚本** `test_joint_directions.py`
2. **记录关节方向表** 明确正负方向对应动作
3. **选择路径**: 固定脚踝 vs 2D约束 vs Cart-Pole

---

*创建日期: 2026-04-07*
