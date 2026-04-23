Week 10 实验配置：MuJoCo 单腿站立（MPC-WBC 闭环验证）

---
## 1. 实验目标

验证 `mpc_and_wbc/week10_single_leg_stand/` 中的 MuJoCo 版本单腿站立实验，重点检查以下接口与阶段衔接是否成立：

- **双足站立保持**：MuJoCo 下初始双足站立是否稳定
- **阶段机切换**：是否严格按阶段从双足过渡到单足
- **MPC（20 Hz）**：基于 centroidal dynamics 生成单支撑参考接触力
- **WBC（240 Hz / 每仿真步）**：将 MPC 参考转化为关节力矩，并在不可用时回退到安全姿态力矩

当前实验的目标已经从“直接验证单腿站立成功”调整为：
1. 先保证 **MuJoCo 双足站立稳定**
2. 再验证 **双足 -> 负载转移 -> 单足** 的分阶段控制流程
3. 最后继续解决 **单足支撑仍失稳** 的问题

---
## 2. 仿真环境

| 参数 | 设置 |
|------|------|
| 仿真器 | MuJoCo |
| 机器人模型 | `g1_23dof.xml` |
| 时间步长 | `dt_sim = 1/240 s` |
| WBC 周期 | 每个仿真步执行一次（240 Hz） |
| MPC 重求解周期 | `T_s = 0.05 s`（20 Hz） |
| 重力 | `[0, 0, -9.81]` |
| 地面 | MuJoCo 模型自带平面 |
| 摩擦系数 | `mu = 0.8` |

---
## 3. 机器人模型与初始条件

### 3.1 模型

- 模型路径：`/home/ccc/projects/unitree_ros/robots/g1_description/g1_23dof.xml`
- 模型类型：浮基人形（MuJoCo `free joint` + 23 个驱动关节）
- 当前实验使用 MuJoCo 内置动力学接口：
  - 质量矩阵：`mj_fullM`
  - 偏置项：`qfrc_bias`
  - Jacobian：`mj_jac` / `mj_jacBodyCom`
  - 接触力：`mj_contactForce`

### 3.2 初始站姿

MuJoCo 下零位姿态会导致双足站立非常脆弱，因此当前默认站姿已经调整为轻微屈膝：

- 左右髋 pitch：`-0.2`
- 左右膝：`0.4`
- 左右踝 pitch：`-0.2`
- 基座初始高度：`z = 0.76`
- 额外加入 MuJoCo DOF damping：
  - base damping = `5.0`
  - joint damping = `10.0`

### 3.3 双足站立控制策略

双足站立阶段不使用 MPC/WBC，而是使用：

- **姿态 PD**
- **MuJoCo 偏置项补偿**（`C(q, v)`）

当前已经验证：
- 双足站立 3 秒内不发散
- 最大足端滑移约 `1.06 mm`

---
## 4. 控制阶段设计

当前 `main.py` 已重构为显式阶段机：

1. `INIT_SETTLE`
2. `DOUBLE_SUPPORT_HOLD`
3. `LOAD_SHIFT`
4. `PRE_LIFTOFF`
5. `SINGLE_SUPPORT`

### 4.1 INIT_SETTLE

- 目的：让 MuJoCo 接触与初始姿态先稳定下来
- 时长：`INIT_SETTLE_TIME = 0.20 s`

### 4.2 DOUBLE_SUPPORT_HOLD

- 目的：强制先验证双足站立稳定性
- 判据：
  - 双脚法向力都超过 `DOUBLE_SUPPORT_MIN_FORCE`
  - 双脚滑移低于 `SLIP_THRESH`
- 只有满足条件持续一段时间后，才能进入下一阶段

### 4.3 LOAD_SHIFT

- 目的：在仍保持双足接触时，将负载转移到目标支撑脚
- 当前支撑脚默认由 `LIFT_LEG` 推导：
  - 若抬左腿，则支撑脚为右脚

### 4.4 PRE_LIFTOFF

- 目的：在完全进入单足前，先做小幅 swing target 预热
- 当前 swing target 仅对摆动腿 hip/knee 生效

### 4.5 SINGLE_SUPPORT

- 目的：启用单足支撑的 MPC + WBC
- 特点：
  - MPC 使用单接触点 centroidal dynamics
  - WBC 失败或支撑脚接触不足时，会退回安全姿态力矩

---
## 5. MPC 配置

### 5.1 状态与控制

| 量 | 定义 | 维度 |
|---|---|---|
| 状态 `x` | `[c, c_dot, L]^T` | 9 |
| 控制 `u` | 单支撑接触力 `f` | 3 |

### 5.2 参考轨迹

- `c_ref = [0, 0, h_com]^T`
- `c_dot_ref = 0`
- `L_ref = 0`

### 5.3 动力学

- 离散时间：`T_s = 0.05 s`
- 预测时域：`N = 10`
- 线性化模型：
  - `x_(k+1) = A_d x_k + B_d u_k + d_d`
  - `B_d` 中使用当前支撑点 `p_foot`

### 5.4 QP 约束

- 初始状态约束
- 离散动力学约束
- 摩擦锥线性近似：
  - `|f_x| <= mu f_z`
  - `|f_y| <= mu f_z`
  - `f_z >= 0`

### 5.5 权重

- `Q = diag(100,100,100,1,1,1,1,1,1)`
- `R = diag(0.001,0.001,0.001)`
- `Q_N = Q`

---
## 6. WBC 配置

### 6.1 决策变量

`z = [v_dot; f]`

其中：
- `v_dot`：广义加速度
- `f`：支撑脚线接触力

### 6.2 目标

- CoM 跟踪
- 角动量跟踪
- 接触力软参考
- 最小化加速度

### 6.3 约束

- 无滑动约束：`J_c v_dot = 0`（当前实现忽略 `Jc_dot`）
- 摩擦锥约束
- 力矩限幅约束

### 6.4 当前实现特点

- 关节力矩不直接作为优化变量
- 通过
  `tau = S(M v_dot + C - J_c^T f)`
  后验求得
- OSQP 每步重新 `setup()`，避免稀疏模式更新问题

---
## 7. 当前代码结构

```text
week10_single_leg_stand/
├── main.py              # MuJoCo 主循环 + 阶段机
├── mpc.py               # Centroidal MPC
├── wbc.py               # Whole-body controller QP
├── robot_model.py       # MuJoCo 模型封装
├── state_estimator.py   # 状态估计与接触检测
├── config.py            # 参数与阶段阈值
├── utils.py             # 绘图、日志、辅助函数
└── output/              # 保存绘图结果
```

---
## 8. 评估指标

| 指标 | 目标值 | 当前状态 |
|------|--------|----------|
| 双足站立稳定性 | 不发散 | 已满足 |
| 双足足端滑移 | `< 0.005 m` | 已满足（约 `0.00106 m`） |
| CoM 位置 RMSE | `< 0.02 m` | 双足阶段未满足 |
| MPC 求解时间 | `< 20 ms` | 满足 |
| WBC 求解时间 | `< 0.5 ms` | 满足 |
| 单足支撑稳定性 | 3 s | 尚未满足 |

---
## 9. 文档评审结论

本次评审后，原始任务描述已做如下修正：

### 9.1 已修正的问题

- 将 **PyBullet** 改为 **MuJoCo**
- 将“固定时刻直接切单足”改为“**显式阶段机**”
- 将“WBC 1 kHz”改为与当前实现一致的“**每个仿真步执行一次**”
- 将机器人模型从“任意 URDF”改为“**当前 G1 MuJoCo XML**”
- 补充了当前已实现的：
  - MuJoCo 动力学接口
  - 稳定双足站立配置
  - output 目录绘图输出

### 9.2 当前仍存在的核心问题

- 单足阶段中，目标支撑脚在进入 `SINGLE_SUPPORT` 后仍可能失去支撑
- WBC 在接触不足时会退回安全力矩，但仍未形成稳定单足闭环
- 当前 CoM 高度参考 `H_COM = 0.8` 与实际稳定双足姿态下 CoM 高度仍有偏差

### 9.3 下一步建议

1. 先把 `LOAD_SHIFT` / `PRE_LIFTOFF` 做成真正的**负载转移控制**
2. 让进入 `SINGLE_SUPPORT` 的判据包含：
   - 支撑脚法向力充足
   - 摆动脚法向力接近 0
   - CoM 已移动到支撑脚附近
3. 再继续调 `SINGLE_SUPPORT` 阶段的 MPC/WBC 参数

---
## 10. 总结

当前任务定义已经从“直接做单足站立”收敛为一个更合理的工程路径：

- **先稳定双足**
- **再做阶段切换**
- **最后解决单足控制**

这与当前 MuJoCo 代码实现是一致的，也更适合作为后续迭代的真实任务基线。
