# 修复计划：单腿站立阶段切换失败

## 根因分析

仿真卡在 `LOAD_SHIFT` 阶段，`support_ratio` 始终低于阈值（~0.40，
需要 ≥0.45），CoM 几乎不向支撑脚偏移（`com_shift_ratio` ≈ 0.02 vs 目标 0.16）。
根本原因有 **三个层次**：

### 1. 双支撑 WBC 接触模型缺陷（核心）

当前 DS WBC 对每只脚用 **1个质心接触点 + 4D力（fx,fy,fz,mz）**，
但接触 Jacobian 是 **6行（3线速度+3角速度）**。

- 4D 力只有偏航力矩（mz），没有 roll/pitch 力矩（mx,my）
- 真实箱形足底有 4 个角点接触，角点间的差分法向力产生 roll/pitch 力矩
- **没有 roll/pitch 力矩通道 → WBC 无法产生横滚力矩来转移重心**
- 对比：单支撑阶段使用 4角点接触（每角3D力=12DOF），可以自然产生 CoP 偏移

### 2. 负载转移参数过于保守

| 参数 | 当前值 | 问题 |
|------|--------|------|
| `LOAD_SHIFT_SUPPORT_RATIO` | 0.48 | 几乎 50/50，最小偏移 |
| `LOAD_SHIFT_COM_RATIO` | 0.16 | CoM 参考几乎不移动 |
| `LOAD_SHIFT_TIME` | 0.20 | 过渡仅 0.2 秒 |
| `LOAD_SHIFT_ROLL_DELTA` | 0.025 | 姿态横滚量极小 |
| `LOAD_SHIFT_SUPPORT_RATIO_MIN` | 0.45 | 阈值太接近 50% |

即使 WBC 工作正常，这些参数只要求 WBC 把体重从 50% 移到 48%，
物理上几乎不产生重心偏移。

### 3. WBC 和 safe_tau 冲突

在 DS 阶段，WBC 求出的力矩直接覆盖了 `safe_tau`（含 roll 偏移的姿态力矩）。
WBC 没有 roll 力矩通道，所以它会 **主动抵消** safe_tau 产生的横滚倾斜。
结果：
- safe_tau 推机器人向支撑脚偏 → WBC 推回来保持"对称站立"
- 实际效果：机器人几乎不动

---

## 修复计划

### 步骤 1：修复 DS WBC 接触模型

**目标**：让 DS WBC 和 SS WBC 使用相同的角点接触模型（4角点/足），
使 WBC 能通过差分法向力产生 roll/pitch 力矩。

**具体改动**：
- `main.py`：DS WBC 使用 4角点/足（8 contacts × 3D = 24D力空间）替代
  2质心/足（2 contacts × 4D = 8D力空间）
- `wbc.py`：`wbc_ds` 从 `WholeBodyController(nv, num_contacts=2, contact_dim=4)`
  改为 `WholeBodyController(nv, num_contacts=8, contact_dim=3)`
- `main.py`：构建 DS WBC 的 force reference 时，将期望的支撑/摆动比值分配
  到各角点法向力
- `main.py`：构建 DS WBC 的 Jacobian 时，使用 8个角点 Jacobian（4/足）

**验证**：DS WBC 是否能将 `support_ratio` 推到 60%+。

### 步骤 2：修复负载转移参数

**目标**：让阶段切换条件在物理上可达。

**具体改动** (`config.py`)：

| 参数 | 旧值 | 新值 | 理由 |
|------|------|------|------|
| `LOAD_SHIFT_SUPPORT_RATIO` | 0.48 | 0.70 | WBC 力分配目标 |
| `LOAD_SHIFT_SUPPORT_RATIO_MIN` | 0.45 | 0.55 | 阶段切换阈值 |
| `LOAD_SHIFT_COM_RATIO` | 0.16 | 0.35 | CoM 参考偏移量 |
| `LOAD_SHIFT_TIME` | 0.20 | 1.00 | 给足够的过渡时间 |
| `LOAD_SHIFT_ROLL_DELTA` | 0.025 | 0.06 | 姿态横滚更大 |
| `PRE_LIFTOFF_SUPPORT_RATIO` | 0.56 | 0.80 | 继续转移到更不对称 |
| `PRE_LIFTOFF_COM_RATIO` | 0.20 | 0.50 | 预抬起阶段的 CoM 偏移 |
| `DOUBLE_SUPPORT_MIN_FORCE` | 120.0 | 80.0 | DS 阈值不太严 |
| `LOAD_SHIFT_SWING_FORCE_MIN` | 50.0 | 20.0 | 允许摆动脚负载下降 |
| `SIM_DURATION` | 6.0 | 15.0 | 给足时间完成全流程 |

### 步骤 3：添加 DS WBC / safe_tau 混合力矩

**目标**：在 DS 阶段，WBC 力矩不是直接写入，而是与 safe_tau 按比例混合，
保证 WBC 不会抵消姿态 PD 产生的横滚。

**具体改动**：
- `config.py`：添加 `DS_WBC_POSTURE_BLEND = 0.35`
- `main.py`：DS 阶段应用力矩时：
  `applied_tau = (1-blend) * wbc_tau + blend * safe_tau`
- 混合比随阶段进度变化：LOAD_SHIFT 开头 blend 较高，末尾降低

### 步骤 4：修复 PRE_LIFTOFF 和 SINGLE_SUPPORT 入口条件

**目标**：确保一旦负载转移成功，后续阶段能顺利切换。

**具体改动**：
- `phases/load_shift.py`：`support_ratio >= 0.55` 即可（从 0.45 降低到 0.55
  实际上是提高，因为原来 0.45 是不够的）
- `phases/pre_liftoff.py`：将 `support_ratio >= 0.50` 提高到 `>= 0.65`，
  确保 PRE_LIFTOFF 时已经充分转移
- 添加 `swing_force < PRE_LIFTOFF_SWING_FORCE_MAX` 作为额外条件

### 步骤 5：清理诊断日志与最终验证

**目标**：清理临时代码，跑完整的 15 秒仿真验证全流程。

**验证指标**：
- `LOAD_SHIFT` → `PRE_LIFTOFF` → `SINGLE_SUPPORT` 阶段完整切换
- CoM RMSE < 0.02m (SS 阶段)
- 足端滑移 < 5mm
- 仿真稳定运行超过 10 秒的单支撑

---

## 执行顺序

1. **步骤 1**（接触模型修复）→ 测试 support_ratio 是否改善
2. **步骤 2**（参数调整）→ 测试阶段是否能切换
3. **步骤 3**（力矩混合）→ 测试稳定性
4. **步骤 4**（入口条件）→ 测试完整流程
5. **步骤 5**（清理和最终调参）→ 最终验证

每步完成后如果测试失败，先修当前步骤的问题再继续。