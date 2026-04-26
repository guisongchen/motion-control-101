# Issue Report: WBC CoM Trajectory Tracking in Double Support

## Date
2026-04-26

## Objective
Use Whole-Body Control (WBC) to track a smoothstep CoM trajectory during the `LOAD_SHIFT` and `PRE_LIFTOFF` phases, replacing the existing posture-control-based weight transfer.

## What Was Implemented
1. **Smoothstep CoM trajectory** from stance midpoint (double support) to support-foot centroid (single support), with analytical velocity and acceleration references.
2. **Double-support WBC** (`wbc_ds`) using `WholeBodyController` to track the trajectory.

## Debug Findings

### Test 1: Single centroid contact per foot (num_contacts=2)
- WBC solve succeeds (avg ~0.15 ms, status optimal).
- **CoM does not move at all.** Actual `c` stays frozen at `[0.03, -0.01]` for 5+ seconds while `c_ref` moves to `[0.04, -0.12]`.
- Debug output reveals:
  - `vdot_norm` ~54 (huge joint accelerations commanded)
  - `c_ddot_actual` matches `c_ddot_des` almost perfectly in the WBC solution
  - `tau_norm` ~13 Nm (tiny actual torques)
- **Root cause:** WBC computes `tau = (M @ v_dot + C - J_c^T @ f)[6:]`. The large `v_dot` is canceled out by large contact forces `f` in the optimization, yielding tiny torques. These contact forces are *internal WBC variables* — MuJoCo's actual 4-sphere contacts per foot produce completely different force distributions. The WBC command is dynamically inconsistent with the physics simulator.

### Test 2: Four corner contacts per foot (num_contacts=8)
- No-slip constraints on all 8 corners lock foot orientation completely.
- Ankle roll is impossible, so lateral CoM shift is kinematically blocked.
- Feet don't slip (2.28 mm), but CoM still doesn't move.

### Test 3: Revert to posture control for LOAD_SHIFT/PRE_LIFTOFF
- Phases progress successfully: `DOUBLE_SUPPORT_HOLD` -> `LOAD_SHIFT` -> `PRE_LIFTOFF` -> `SINGLE_SUPPORT`.
- But single support fails immediately because the robot enters with:
  - Support foot rolled (from posture-control ankle roll offsets)
  - Knees too straight (0.4 rad vs direct-benchmark 0.62 rad)
- Max slip 347 mm, RMSE 0.55 m. Robot falls over.

### Test 4: Blend toward direct single-support pose during PRE_LIFTOFF
- Passing `build_direct_pose(...)` as `optimized_joint_angles` to `build_safe_targets`.
- **Result:** Worse. Simulation never reaches `SINGLE_SUPPORT`. RMSE 0.78 m, slip 650 mm.
- **Cause:** Direct pose lifts swing knee to 1.05 rad while swing foot is still on the ground in `PRE_LIFTOFF`. Forcing this blend while planted creates severe contortion.

### Test 5: 4D box contact model + centroidal force task
- **Implemented full 4D contact infrastructure:**
  - Replaced 4 sphere geoms per foot with 1 box geom per foot in `g1_23dof_4d_contact.xml`
  - Added torsional friction (`gamma=0.05`) to friction cone: `|mz| <= gamma * fz`
  - Updated WBC core to support `contact_dim=4` (3D force + yaw moment)
  - Updated all geometry-querying code (trajectory, double_support, single_support, primitives) to handle box geoms
- **Added force task to double-support WBC** to prevent v_dot/f cancellation:
  ```python
  force_task_matrix = np.zeros((3, 8))
  force_task_matrix[0, 0] = 1.0; force_task_matrix[0, 4] = 1.0
  force_task_matrix[1, 1] = 1.0; force_task_matrix[1, 5] = 1.0
  force_task_matrix[2, 2] = 1.0; force_task_matrix[2, 6] = 1.0
  force_task_ref = robot.total_mass * (c_ddot_des - GRAVITY)
  force_task_weight = np.array([100.0, 100.0, 10.0])
  ```
- **Result:** `tau_norm` improved from ~13 to ~27-35, but CoM still does not move laterally.

### Test 6: Fix H_COM mismatch
- **Discovered origin of H_COM=0.8:** hardcoded guess from commit `6ce51f9` on 2026-04-23 with comment "期望 CoM 高度 [m]（相对于支撑足）".
- **Actual standing CoM height:** z≈0.70m (verified from optimized single-support pose: `[0.009, -0.008, 0.703]`).
- **Fix applied:** Removed hardcoded `H_COM` usage from `trajectory.py`, `phases/single_support.py`, and `main.py`. Now captures actual `c[2]` at `DOUBLE_SUPPORT_HOLD` entry and uses it as the z-reference for all subsequent phases.
- **Result:** H_COM fix removed the vertical acceleration drain (was commanding ~10.5 m/s² upward), but **lateral CoM transfer still fails**.
  - Robot gets stuck in `LOAD_SHIFT` for ~4.7 seconds (should be 0.2s).
  - `support_ratio` plateaus at ~0.37 (target 0.52).
  - `com_shift` plateaus at ~0.11 (target 0.16).
  - Foot slip ~10mm (exceeds 5mm threshold).
  - CoM RMSE: 0.093m, max slip: 10.04mm.

## Root Cause Summary
The fundamental problem is that the double-support WBC cannot command a lateral CoM shift that is actually realized by MuJoCo's contact physics, even with:
1. A 4D box-contact model that matches the XML geometry
2. A centroidal force task that prevents v_dot/f cancellation
3. The correct CoM height reference

The WBC computes optimal `v_dot` and `f` that satisfy the centroidal dynamics *algebraically*, but the mapping from these variables to joint torques (`tau = M@v_dot + C - J_c^T @ f`) is under-determined. The optimizer finds solutions where `v_dot` and `f` cancel each other in the torque equation, leaving tiny torques that don't actually move the robot.

The force task mitigates this but doesn't eliminate it — the weight ratio between the force task (W=100) and the acceleration minimization (W=0.01) still allows the optimizer to "cheat" by finding dynamically inconsistent solutions.

## Open Questions / Next Steps
1. **Implement full QP-ID WBC formulation** (Task #7): Use a proper task hierarchy with strict dynamics constraints, or use a null-space projection approach where the contact forces are solved first from centroidal dynamics, then joint accelerations are solved from the equations of motion.
2. **Debug the WBC debug output** during LOAD_SHIFT: inspect `c_ddot_des`, actual `f` distribution between feet, and why lateral forces don't materialize.
3. **Re-evaluate the single centroid contact per foot**: the 4D contact at the box centroid should allow ankle roll, but maybe the no-slip constraint on translation is still too restrictive. Consider relaxing to inequality constraints or using a soft contact model.
4. **Consider abandoning WBC for double support**: the posture-control `LOAD_SHIFT` works (phases progress), but leaves the robot in a bad pose for single support. Maybe fix the handoff pose instead of fixing the WBC.
