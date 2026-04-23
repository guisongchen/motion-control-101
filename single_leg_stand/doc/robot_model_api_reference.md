# RobotModel API 参考与设计说明

本文档汇总 `robot_model.py` 中所有函数、属性及关键设计决策的物理意义与实现细节。

---

## 1. 概述

`RobotModel` 封装 PyBullet 机器人模型的运动学与动力学接口，为 MPC-WBC 闭环控制提供统一的状态读取、Jacobian 计算和动力学量查询。

**核心设计原则：**
- `q`（广义位置）包含所有关节（含固定关节），保持与 PyBullet 索引一一对应
- `v`（广义速度）仅包含自由度关节，与动力学矩阵维度一致
- 总质量在初始化时计算并缓存，供 CoM 相关函数复用

---

## 2. 初始化与属性

### `__init__(urdf_path, base_position, base_orientation, use_fixed_base)`

加载 URDF，解析关节信息，建立固定关节/自由度关节映射。

**关键属性：**

| 属性 | 类型 | 说明 |
|------|------|------|
| `robot_id` | `int` | PyBullet 机器人实例 ID |
| `num_joints` | `int` | URDF 中声明的关节总数 |
| `dof_joints` | `List[int]` | 自由度关节索引列表（排除 `JOINT_FIXED`） |
| `nv` | `int` | 广义速度维度 = `6 + len(dof_joints)` |
| `link_name_to_index` | `Dict[str, int]` | link 名到 PyBullet link 索引的映射（基座为 -1） |
| `nq` | `int` | 广义位置维度 = `7 + num_joints`（含固定关节占位） |
| `total_mass` | `float` | 机器人总质量 [kg]，初始化时计算并断言为正 |

**固定关节 vs 自由度关节：**

| | 固定关节 (`JOINT_FIXED`) | 自由度关节（转动/移动） |
|---|---|---|
| 运动 | 零自由度，刚性固连 | 1 个或多个自由度 |
| 广义坐标 | 无关节角变量 | 有独立的 `q` / `v` |
| 用途 | 传感器挂载、外壳、子模块拼接 | 实现运动功能 |

Unitree G1 中存在固定关节的原因：头部相机/IMU 模块、手部简化表示、装饰性外壳等作为独立 link 通过固定关节连接。

**为什么不从 `v` 中排除固定关节：**
- `q` 保持完整长度（含固定关节占位），方便从 `getJointStates` 直接填充且索引不错位
- `v` 只保留真实自由度，与动力学矩阵 $M$、Jacobians 的列数一致
- PyBullet API（2025 年 1 月版本起）要求 `calculateMassMatrix`、`calculateJacobian`、`calculateInverseDynamics` 的 `jointPositions` / `jointVelocities` 长度为**自由度数量**（不含固定关节），代码通过 `_get_dof_positions` / `_get_dof_velocities` 自动提取

---

## 3. 状态读取

### `get_state() -> Tuple[np.ndarray, np.ndarray]`

返回当前状态 `(q, v)`。

- `q`：广义位置 `[base_pos(3), base_quat(4), joint_pos(num_joints)]`
- `v`：广义速度 `[base_vel(3), base_omega(3), joint_vel(num_dof_joints)]`

**为什么基座和关节要分开读取：**

PyBullet 没有统一的"读取全部状态"API：
- 基座位姿：`getBasePositionAndOrientation` + `getBaseVelocity`
- 关节状态：`getJointStates`

因此 `get_state()` 内部是三次独立调用，最后拼接成统一的 `q` 和 `v` 向量。

---

## 4. 动力学计算

### `_get_dof_positions(q) -> list`

从完整 `q` 中提取自由度关节位置列表（长度 = `len(dof_joints)`），供 PyBullet 动力学 API 使用。

### `_get_dof_velocities(v) -> list`

从完整 `v` 中提取自由度关节速度列表，供 `calculateInverseDynamics` 使用。

> **PyBullet API 兼容性（2025 年 1 月版本起）：** `calculateMassMatrix`、`calculateJacobian`、`calculateInverseDynamics` 的 `jointPositions` / `jointVelocities` 参数长度必须等于**自由度数量**（不含固定关节）。此前版本允许传入 `num_joints` 长度（固定关节填 0），新版会抛出 `numDof needs to be positive` 错误。

---

### `compute_mass_matrix(q) -> np.ndarray`

计算质量矩阵 $M(q)$，维度 `(nv, nv)`。

**实现：**
```python
joint_positions = self._get_dof_positions(q)
M, _ = p.calculateMassMatrix(self.robot_id, joint_positions)
```

**为什么输入只包含关节位置，不含基座？**

质量矩阵只取决于机器人的"构型"（各连杆相对姿态），不取决于在世界坐标系中的绝对位置。PyBullet 内部已保存基座状态，且质量矩阵是"构型相关、位姿无关"的——机器人平移/旋转到任何地方，只要关节角不变，$M$ 就不变。

**返回矩阵已包含基座：** 虽然只传了自由度关节位置，返回的 $M$ 维度仍是 `(nv, nv)`，前 6×6 块就是浮动基的惯性。

---

### `compute_coriolis_gravity(q, v) -> np.ndarray`

计算非线性偏置力 $h(q, v) = C(q, \dot{q})\dot{q} + G(q)$，维度 `(nv,)`。

**实现：**
```python
positions = self._get_dof_positions(q)
velocities = self._get_dof_velocities(v)
C = p.calculateInverseDynamics(robot_id, positions, velocities, [0.0]*n)
```

**物理意义：**
- `calculateInverseDynamics` 计算 $\tau = M\ddot{q} + C\dot{q} + G$
- 传入零加速度后，$M\ddot{q}$ 项消失，返回值为 $C\dot{q} + G$

**为什么把科氏力和重力放在一起？**

在 WBC 动力学约束 $M\dot{v} + h = S^\top \tau + J_c^\top f$ 中，$h$ 就是全部非线性项。对控制器而言：
1. 不需要拆开——逆动力学一次调用给出总和
2. 代入约束时直接用 $h$ 整体替换即可

---

### `compute_com_position(q) -> np.ndarray`

计算质心 CoM 位置 `(3,)`。

**方法：** 加权平均各 link 质心位置

$$c = \frac{1}{M} \sum_i m_i \cdot p_i$$

**实现细节：**
- 遍历范围：`range(-1, num_joints)`（`-1` 为基座）
- 质量来源：`p.getDynamicsInfo(i)[0]`，与 URDF 中 `<inertial><mass>` 一一对应
- 使用缓存的 `self.total_mass` 做除法，避免重复累加

---

### `compute_com_velocity(q, v) -> np.ndarray`

计算质心 CoM 速度 `(3,)`。

$$\dot{c} = J_{\text{com}} \cdot v$$

---

### `compute_centroidal_momentum(q, v) -> np.ndarray`

计算质心角动量 $L$ `(3,)`。

$$L = J_L \cdot v$$

---

## 5. Jacobian 计算

### `get_foot_jacobian(foot_link, q, local_position) -> np.ndarray`

计算支撑足接触 Jacobian $J_c$，维度 `(6, nv)`。

**输出结构：**
- 前 3 行：线速度映射 $\dot{p}_{\text{foot}} = J_{c,\text{lin}} \cdot v$
- 后 3 行：角速度映射 $\omega_{\text{foot}} = J_{c,\text{ang}} \cdot v$

**在 WBC 中的双重作用：**
1. **无滑动约束：** $J_c \dot{v} + \dot{J}_c v = 0$（支撑足固定）
2. **接触力映射：** 广义力通过 $J_c^\top f$ 进入动力学方程

---

### `get_com_jacobian(q) -> np.ndarray`

计算 CoM Jacobian $J_{\text{com}}$，维度 `(3, nv)`。

**物理定义：**

$$J_{\text{com}} = \frac{1}{M} \sum_i m_i \cdot J_{v_i}$$

- $J_{v_i}$：连杆 $i$ 质心线速度的 Jacobian
- $M = \sum_i m_i$：总质量

**物理意义：** 质心是各连杆质心的质量加权平均，因此 CoM Jacobian 也是各连杆线速度 Jacobian 的质量加权平均。

**在 WBC 中的使用：**

WBC 目标函数中的 CoM 加速度项：

$$\|J_{\text{com}} \dot{v} + \dot{J}_{\text{com}} v - \ddot{c}_{\text{des}}\|_{W_1}^2$$

通过优化 $\dot{v}$ 使质心加速度逼近期望值 $\ddot{c}_{\text{des}}$。

---

### `get_angular_momentum_jacobian(q) -> np.ndarray`

计算角动量 Jacobian $J_L$，维度 `(3, nv)`。

**物理定义：**

$$J_L = \sum_i \Big( I_i^{\text{world}} \cdot J_{\omega_i} + m_i \cdot [p_i - c]_\times \cdot J_{v_i} \Big)$$

**两项含义：**

| 项 | 物理意义 | 说明 |
|---|---|---|
| $I_i^{\text{world}} J_{\omega_i}$ | 自旋角动量 | 连杆绕自身质心旋转 |
| $m_i [r_i]_\times J_{v_i}$ | 轨道角动量 | 连杆质心绕整体质心平动 |

**惯性张量转换：**
```python
I_world = R @ diag(local_inertia_diag) @ R.T
```

**在 WBC 中的使用：**

$$\|J_L \dot{v} - \dot{L}_{\text{des}}\|_{W_2}^2$$

角动量控制是姿态稳定的关键——即使 CoM 位置正确，角动量失控也会导致旋转倾倒。

---

## 6. 接触检测

### `check_contact(link_index, other_body_id=-1) -> Tuple[bool, float]`

检测指定 link 是否与目标 body 接触。

- `link_index`: 待检测的 link 索引（`-1` 为基座）
- `other_body_id`: 目标 body ID，默认 `-1` 表示与**所有** body 检测
- 返回：`(is_contact, normal_force)` —— 是否接触、法向力之和 [N]

**实现细节：** 调用 `p.getContactPoints(bodyA=robot_id, linkIndexA=link_index)`。注意 `bodyB` 参数不可传 `None`，因此当 `other_body_id < 0` 时直接省略 `bodyB` 关键字。

---

## 7. 连杆位姿查询

### `get_link_com_position(link_index) -> np.ndarray`

获取指定连杆质心的世界坐标位置 `(3,)`。

### `get_link_velocity(link_index) -> Tuple[np.ndarray, np.ndarray]`

获取指定连杆质心的线速度和角速度（世界坐标），各 `(3,)`。

**与 Jacobian 方法的对比：**

| | 直接查询 (`get_link_*`) | Jacobian 推算 (`J @ v`) |
|---|---|---|
| 信息类型 | 绝对位置/速度（世界坐标） | 相对速度映射 |
| 用途 | 状态估计、滑移检测、轨迹规划 | WBC 优化系数 |
| 依赖 | PyBullet 内部前向运动学 | 需要正确的 `q` 和 `J` 实现 |

**典型使用场景：**
- `StateEstimator` 读取支撑足位置（替代硬编码）
- 滑移检测：比较前后帧足端水平位移
- $\dot{J}_c$ 的数值近似：`(v_foot_curr - v_foot_prev) / dt`
- 调试验证：验证 `J_c @ v` 计算是否正确

---

## 7. 辅助函数

### `_skew(v) -> np.ndarray`

向量叉乘矩阵（反对称矩阵）：

$$[v]_\times = \begin{bmatrix} 0 & -v_z & v_y \\ v_z & 0 & -v_x \\ -v_y & v_x & 0 \end{bmatrix}$$

用于角动量 Jacobian 中 $r \times \dot{p}$ 的矩阵表示。

---

### `reset_joint_positions(joint_positions)`

重置**自由度关节**位置到指定值（跳过固定关节）。`joint_positions` 长度应与 `len(dof_joints)` 对应，按 `dof_joints` 的顺序填充。

### `reset_base_pose(position, orientation)`

重置基座位姿（位置 + 四元数）。

---

## 8. 关键设计决策汇总

| 决策 | 说明 |
|---|---|
| `q` 含固定关节，`v` 不含 | `q` 保持与 PyBullet API 索引对齐；`v` 与动力学矩阵维度一致 |
| 总质量初始化时计算 | 确保 `compute_com_position` 和 `get_com_jacobian` 作为除数时不为 `None` 或零 |
| `compute_coriolis_gravity` 返回 $C\dot{q}+G$ | WBC 动力学约束需要非线性偏置力的总和，无需拆分 |
| `get_link_*` 直接查询 + Jacobian 计算并存 | 直接查询用于观测/验证，Jacobians 用于控制/优化 |
