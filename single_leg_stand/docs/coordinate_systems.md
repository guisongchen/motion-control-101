# 坐标系与 Centroidal Quantities 定义

本文档明确 `robot_model.py` 和 MPC/WBC 框架中所有物理量的坐标系约定，避免世界坐标、局部坐标、相对坐标之间的混淆。

---

## 1. 世界坐标系 (World Frame)

PyBullet 仿真器使用右手坐标系：

| 轴 | 方向 | 说明 |
|---|---|---|
| **X** | 水平向前 | 机器人面朝 +X |
| **Y** | 水平向左 | 从机器人背后看 |
| **Z** | 竖直向上 | 地面在 `z = 0` |

**原点：** 仿真启动时的 `(0, 0, 0)`，通常位于地面中心。

**重力：** `GRAVITY = [0, 0, -9.81]`，沿 -Z 方向。

> 所有通过 `getBasePositionAndOrientation`、`getLinkState`、`calculateJacobian` 获取的位置、速度、Jacobian，默认都在**世界坐标系**下。

---

## 2. 质心位置 CoM Position

### `compute_com_position() -> np.ndarray (3,)`

$$c_{\text{world}} = \frac{1}{M} \sum_i m_i \cdot p_i^{\text{world}}$$

- $p_i^{\text{world}}$：各 link 质心在世界坐标系中的绝对位置
- $M = \sum_i m_i$：机器人总质量
- 结果 $c$ 是**世界坐标系中的绝对位置**，单位米
- **不是相对于基座的局部坐标**

**数据来源：**
- 基座：`getBasePositionAndOrientation` 返回的世界位置
- 连杆：`getLinkState(..., computeForwardKinematics=1)` 的 `linkWorldPosition`

---

## 3. 质心速度 CoM Velocity

### `compute_com_velocity(q, v) -> np.ndarray (3,)`

$$\dot{c}_{\text{world}} = J_{\text{com}} \cdot v$$

- $v$：广义速度 `[base_lin_vel(3), base_ang_vel(3), joint_vel(n_dof)]`
- 输出 $\dot{c}$ 是 CoM 在**世界坐标系中的线速度**
- $J_{\text{com}}$ 的每一列表示对应自由度对世界 CoM 速度的贡献权重

---

## 4. 角动量 Centroidal Momentum

### `compute_centroidal_momentum(q, v) -> np.ndarray (3,)`

$$L = J_L \cdot v$$

这里的 $L$ 是**相对于整体 CoM 的 centroidal angular momentum**（质心角动量），不是对世界原点的角动量。

**物理定义：**

$$L = \sum_i \Big( I_i^{\text{world}} \omega_i + m_i \cdot (p_i - c) \times (\dot{p}_i - \dot{c}) \Big)$$

- $I_i^{\text{world}}$：连杆 $i$ 在世界坐标系中的惯性张量
- $(p_i - c)$：连杆质心相对整体 CoM 的位置
- 第一项：连杆绕自身质心的自旋角动量
- 第二项：连杆质心绕整体 CoM 的轨道角动量

**关键特性：**
- 机器人整体平移（$\dot{c} \neq 0$）但不旋转时，$L$ 不一定为零
- 机器人绕自身 CoM 旋转但 CoM 静止时，$L$ 反映纯自旋
- MPC 状态中的 $L$ 直接就是这个量

---

## 5. 足端位置与接触力

### `get_link_com_position(link_index)`

返回指定 link 质心的**世界坐标绝对位置** `(3,)`。

### `check_contact(link_index)`

返回接触法向力的大小（标量），方向由 PyBullet 内部计算，始终沿接触法线。

**滑移检测：** 比较足端水平位置与初始位置的偏差：

$$\text{slip} = \sqrt{(x - x_0)^2 + (y - y_0)^2}$$

---

## 6. 广义坐标 q 与广义速度 v

### `q` — 广义位置 `(nq,)`

$$q = [\underbrace{p_x, p_y, p_z}_{\text{base pos (world)}}, \underbrace{q_x, q_y, q_z, q_w}_{\text{base orn (world quat)}}, \underbrace{\theta_0, \theta_1, \dots}_{\text{all joint pos (incl. fixed)}}]$$

- 基座位置：世界坐标绝对值
- 基座姿态：世界坐标四元数
- 关节位置：**所有关节**（含固定关节）按 URDF 顺序排列，固定关节值为 0

### `v` — 广义速度 `(nv,)`

$$v = [\underbrace{v_x, v_y, v_z}_{\text{base lin vel (world)}}, \underbrace{\omega_x, \omega_y, \omega_z}_{\text{base ang vel (world)}}, \underbrace{\dot{\theta}_0, \dot{\theta}_1, \dots}_{\text{dof joint vel only}}]$$

- 基座线速度：世界坐标
- 基座角速度：世界坐标
- 关节速度：**仅自由度关节**，不含固定关节

---

## 7. MPC 参考轨迹的坐标约定

MPC 状态 $x = [c, \dot{c}, L]^\top$ 中的 $c$ 和 $\dot{c}$ 都是**世界坐标系下的绝对值**。

参考轨迹 $c_{\text{ref}} = [0, 0, h_{\text{com}}]$ 也是世界坐标——隐含假设支撑足在世界原点附近。

**重要：** 如果支撑足不在世界原点（例如双足站立时左右偏移），需要根据实际足端位置 $p_{\text{foot}}$ 实时修正 $c_{\text{ref}}$：

```python
c_ref = np.array([p_foot[0], p_foot[1], p_foot[2] + H_COM])
```

否则 MPC 会让 CoM 追踪错误的全局位置。

---

## 8. 汇总表

| 量 | 坐标系 | 参考点 | 维度 |
|---|---|---|---|
| CoM 位置 $c$ | 世界坐标 | 世界原点 | `(3,)` |
| CoM 速度 $\dot{c}$ | 世界坐标 | — | `(3,)` |
| 角动量 $L$ | 世界坐标 | **整体 CoM** | `(3,)` |
| 基座位置 | 世界坐标 | 世界原点 | `(3,)` |
| 基座速度 | 世界坐标 | — | `(3,)` |
| 足端位置 | 世界坐标 | 世界原点 | `(3,)` |
| 接触力 $f$ | 世界坐标 | 接触点 | `(3,)` |
| $J_{\text{com}}$ | 世界坐标 | — | `(3, nv)` |
| $J_L$ | 世界坐标 | 整体 CoM | `(3, nv)` |
| $J_c$ | 世界坐标 | 接触点 | `(6, nv)` |
