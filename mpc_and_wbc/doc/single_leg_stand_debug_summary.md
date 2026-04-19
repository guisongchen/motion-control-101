# 单足站立仿真调试记录

## 1. 当前状态

- **仿真可跑通**：`main.py` 能在 `DIRECT` 模式下完整跑完 3s，不崩溃。
- **MPC / WBC 求解器正常**：
  - MPC 平均求解时间 ~1.3 ms（目标 < 20 ms）
  - WBC 平均求解时间 ~0.16 ms（目标 < 0.5 ms）
- **控制效果不达标**：
  - CoM 位置 RMSE: **0.187 m**（目标 < 0.02 m）
  - 最大足端滑移: **45.4 mm**（目标 < 5 mm）
  - t=3s 时 CoM z 方向掉了约 20 cm，机器人在摔倒。

## 2. 已实现的代码改动

### 2.1 utils.py
- 实现了 `add_friction_cone_to_qp`：将摩擦锥约束叠加到现有 QP 的通用接口。
- 实现了 `setup_logger`：带时间戳的 `StreamHandler`。
- 实现了 `plot_com_tracking`、`plot_contact_force`、`plot_torques`：matplotlib 可视化。
- 实现了 `compute_rmse`：跟踪误差评估。

### 2.2 wbc.py
- 完成了 WBC QP 构造：
  - 目标函数：CoM 跟踪 + 角动量跟踪 + 接触力参考 + 加速度最小化。
  - 约束：无滑动等式 + 摩擦锥不等式 + 力矩限幅双边约束。
- `solve()` 每次重新调用 `solver.setup()`（而非 `update(Ax=...)`），避免 OSQP 稀疏模式变化导致的错误。

### 2.3 main.py
- 实现了完整的 MPC-WBC 闭环：
  - t < 0.5s：双腿站立，所有关节 `POSITION_CONTROL`。
  - t >= 0.5s：切换 MPC+WBC，左腿 `POSITION_CONTROL` 跟踪抬腿轨迹，其余关节 `TORQUE_CONTROL`。
- 增加了 MPC/WBC 求解时间日志和足端滑移统计。
- 将 `p.connect(p.GUI)` 临时改为 `p.connect(p.DIRECT)`，并移除了 `input()` 阻塞，以便后台自动化运行。

## 3. 已修复的 Bug

| Bug | 原因 | 修复方法 |
|-----|------|----------|
| MPC OSQP `update(Ax=...)` 报错 "new number of elements out of bounds" | `setup` 时 A 矩阵占位符填 `0.0`，dok_matrix 不存储，导致 nnz 不足 | 占位符改为 `1.0`，并在 `solve()` 中改用 `solver.setup()` |
| `calculateMassMatrix` 返回格式错误 | PyBullet 返回的是嵌套元组 `(29, 29)`，而非 `(M, _)` | 直接 `np.array(result)` |
| `calculateInverseDynamics` 参数长度不对 | 浮动基机器人需要传入 **所有关节**（含固定关节，共 32 个）的位置/速度 | 新增 `_get_all_joint_positions` / `_get_all_joint_velocities` |
| GUI 模式阻塞 stdin | `input("Press Enter...")` 在后台无 stdin 时抛 `EOFError` | 临时改为 `DIRECT` 模式并移除 `input()` |

## 4. 未解决的核心问题

### 4.1 PyBullet `calculateInverseDynamics` 对浮动基 G1 始终失败

- **现象**：在 DIRECT 和 GUI 模式下，只要调用 `p.calculateInverseDynamics(robot_id, all_positions, all_velocities, zero_acc)`，PyBullet 内部就报错 "Inverse Dynamics computations failed"，抛出 `SystemError`。
- **影响**：WBC 每一步都回退到重力近似 `C[2] = -mg`，缺少关节部分的重力补偿和科氏力项。
- **已验证**：
  - 与 `TORQUE_CONTROL` / `POSITION_CONTROL` 模式无关。
  - 与 `reset_base_pose` / `reset_joint_positions` 无关。
  - 与 stepSimulation 次数无关（立即调用也失败）。
  - 传入所有 32 个关节的位置/速度后仍然失败。
  - 使用 `p.CALCULATE_INVERSE_DYNAMICS_ARGUMENT_TORQUE_ONLY` 标志不可用。
- **根因判断**：PyBullet 对浮动基机器人的逆动力学支持不完善，G1 URDF 中存在部分无惯性数据的 link（如 `imu_in_torso`、`d435_link` 等），可能导致内部计算不稳定。

### 4.2 控制性能不达标

- **C 的重力近似不完整**：只有基座的 `-mg`，没有 23 个关节的重力补偿矩。这导致 WBC 计算出的 `tau` 严重偏离真实需求。
- **单腿切换瞬间失去支撑**：在 t=0.5s 切换时，若 `wbc_result` 为 None（第一次调用），支撑腿力矩被设为 0，机器人瞬间失稳。
- **PD 增益和 MPC 参考可能需调参**：当前 `Kp_c=100, Kd_c=20` 以及 `Q=diag([100,100,100, 1,1,1, 1,1,1])` 可能不足以单足站立。

## 5. 下一步计划

### 5.1 短期：绕过 PyBullet ID 限制

**方案 A：手动关节重力补偿**
- 遍历运动学树，为每个关节计算连杆重力产生的力矩近似。
- 不需要外部库，但代码量较大，精度有限。

**方案 B：临时加载固定基副本计算 ID**
- 用 `useFixedBase=True` 重新加载一份 G1 URDF。
- 固定基下 `calculateInverseDynamics` 通常稳定工作。
- 每步将浮动基状态复制到固定基副本，计算 C，再用于主机器人的 WBC。
- 性能开销约等于一次 URDF 加载 + step，需要评估是否可接受。

**方案 C：用 PyBullet `getJointStates` 的 `jointReactionForces` 反推**
- 利用仿真器已经计算好的关节反力来估计 C。
- 精度受仿真步长和约束求解器影响。

### 5.2 中期：改进控制策略

- **平滑过渡**：在 0.5s 切换时，不要瞬间将支撑腿力矩设为 0，而是先用 POSITION_CONTROL 维持姿态 50~100ms，待 WBC 稳定后再切换 TORQUE_CONTROL。
- **调参**：增大 `Kp_c`、`Q` 对角线权重，测试不同参数组合。
- **支撑足检测**：当前使用法向力最大的足端作为支撑足，但单足时应锁定为右腿，避免动态切换造成 MPC 参考突变。

### 5.3 长期：引入 Pinocchio

- 安装 `pinocchio` 库，用其解析 G1 URDF 并计算高质量的动力学量（M, C, J）。
- 这是人形机器人控制中的标准做法，可以彻底解决 PyBullet ID 不可靠的问题。

## 6. 关键结论

- **QP 框架本身已经跑通**：MPC 和 WBC 的 OSQP 求解稳定、快速，约束构造正确。
- **瓶颈在动力学计算**：PyBullet 不是为浮动基机器人的精确逆动力学设计的。要实现稳定的单足站立，必须获得准确的 `C(q, v)`。
- **建议优先级**：先尝试方案 B（固定基副本计算 ID），因为它改动最小且能复用现有 PyBullet API；如果性能或精度不足，再转向 Pinocchio。
