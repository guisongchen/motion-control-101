# Comparison: `main.py` vs `stand_progressive.py`

## Executive Summary

| Aspect | `main.py` (original) | `stand_progressive.py` (new) |
|--------|----------------------|-------------------------------|
| **Result** | Stuck at LOAD_SHIFT forever (ratio ~0.40) | All 5 phases complete, single-leg stand achieved |
| **Root cause of failure** | DS WBC uses 1 centroid contact per foot (4 DOF) — no roll/pitch moment channel | DS WBC uses 4 corner contacts per foot (12 DOF) — full moment authority |
| **Lines of code** | ~900 | ~780 |
| **Config** | Spread across `config.py` (134 lines) | Inline `PHASE_CONFIG` dict (~45 params) |

---

## 1. Phase Machine

Both files implement the same 5-phase state machine:

```
INIT_SETTLE → DOUBLE_SUPPORT_HOLD → LOAD_SHIFT → PRE_LIFTOFF → SINGLE_SUPPORT
```

| Phase | `main.py` Transition Logic | `stand_progressive.py` Transition Logic |
|-------|---------------------------|----------------------------------------|
| INIT_SETTLE → DS_HOLD | Time ≥ 0.20s | Time ≥ 0.20s (same) |
| DS_HOLD → LOAD_SHIFT | `StabilityGate` with: both feet >120N, slip<5mm, CoM velocity<0.05, L_norm<0.5, force ratio>0.35, CoM inside polygon, sustained 1.0s | `StabilityGate` with: both feet >80N, slip<5mm, CoM velocity<0.08, L_norm<0.5, force ratio>0.30, CoM inside polygon, sustained 0.8s |
| LOAD_SHIFT → PRE_LIFTOFF | All: time≥0.20s, support_force≥120N, swing_force≥50N, ratio≥0.45, com_speed≤0.35 | All: time≥1.50s, support_force≥80N, ratio≥0.55, com_speed≤0.35, 0.10s sustained |
| PRE_LIFTOFF → SINGLE_SUPPORT | All: time≥0.50s, support_force≥50N, ratio≥0.50, com_speed≤0.35 | time≥1.0s AND (ratio≥0.60 OR time≥2.0s with ratio≥0.55) AND com_speed≤0.35 AND swing_force<150N, 0.10s sustained |

**Key difference**: `stand_progressive.py` relaxes thresholds (lower force minimums, longer hold times) and adds a **time-based fallback** for PRE_LIFTOFF to avoid getting stuck on oscillating metrics.

---

## 2. Double-Support WBC (LOAD_SHIFT / PRE_LIFTOFF / DS_HOLD)

This is the **critical architectural difference**.

### `main.py` — Centroid Contact Model (2×4D = 8 force DOFs)

```
Per foot: 1 centroid point
Force vector per contact: [fx, fy, fz, mz] (4D)
Total: 2 contacts × 4D = 8 decision variables
Jacobian: 6 rows per contact × 2 = 12 constraint rows
```

- **Problem**: 4D force (fx,fy,fz,mz) has yaw moment (mz) but **no roll/pitch moment** (mx, my)
- The 6-row Jacobian requires 6 acceleration constraints but only 4 force variables → 2 rows per foot are "dead weight"
- Weight transfer requires differential normal force across the foot → requires roll moment
- **Result**: WBC **cannot produce roll moment**, so it cancels out the posture PD's roll offset, keeping support_ratio at ~0.40

Additional issue: `main.py` applies WBC output directly (`wbc_result["tau"]`) with no blending against `safe_tau`, so WBC actively overrides the posture roll.

### `stand_progressive.py` — Corner-Patch Contact Model (8×3D = 24 force DOFs)

```
Per foot: 4 corner points (from MuJoCo box geoms)
Force vector per contact: [fx, fy, fz] (3D)
Total: 8 contacts × 3D = 24 decision variables
Jacobian: 3 rows per contact × 8 = 24 constraint rows (stacked linear-only)
```

- **Advantage**: 4 corner points per foot create a pressure distribution → roll/pitch moments emerge naturally from differential fz across corners
- WBC can redistribute vertical force across corners → CoP moves laterally → weight shifts
- **Result**: WBC drives support_ratio to **0.84–0.93** during LOAD_SHIFT

Additional fix: WBC output is blended with safe_tau (`ds_posture_blend = 0.30`) so the posture roll PD is not completely overridden.

### Force Reference Construction

| Aspect | `main.py` | `stand_progressive.py` |
|--------|-----------|----------------------|
| Force DOFs | `[fx,fy,fz,mz] × 2 = 8` | `[fx,fy,fz] × 8 = 24` |
| Force reference | Hand-built 8D vector + force task matrix | Equal fz per corner, scaled by target_ratio |
| Force task matrix | 4×8 explicit matrix mapping fz to total/distribution | None (WBC QP handles it via natural contact structure) |
| Weight distribution | Explicit `DS_LOAD_DISTRIBUTION_WEIGHT=1000` penalty | Implicit via per-corner fz reference |

---

## 3. CoM Reference Trajectory

| Phase | `main.py` c_ref | `stand_progressive.py` c_ref |
|-------|-----------------|-------------------------------|
| SETTLE / DS_HOLD | `nominal_c_ref` (midpoint) | `nominal_c_ref` (midpoint) — same |
| LOAD_SHIFT | `compute_phase_com_target()` with `LOAD_SHIFT_COM_RATIO=0.16` → tiny shift | Smoothstep from 0.5 to `load_shift_target_ratio=0.72` along support-swing axis |
| PRE_LIFTOFF | `compute_phase_com_target()` with `PRE_LIFTOFF_COM_RATIO=0.20` → still tiny | Smoothstep from 0.72 to `single_support_support_ratio=0.90` |
| SINGLE_SUPPORT | `compute_phase_com_target()` with `SINGLE_SUPPORT_COM_RATIO=0.28` | Fixed at `single_support_com_ratio=0.70` toward support foot |
| Height | `H_COM=0.8` hardcoded, overridden by `standing_com_z` | Measured `standing_com_z` captured at phase transitions |

**Key difference**: `main.py` uses com_ratio values that are far too small (0.16, 0.20, 0.28) — the CoM barely moves toward the support foot. `stand_progressive.py` uses 0.35→0.90, creating a meaningful lateral shift.

---

## 4. Posture Targets (safe_tau)

| Aspect | `main.py` | `stand_progressive.py` |
|--------|-----------|----------------------|
| Roll delta (LOAD_SHIFT) | 0.025 rad (with feedback) | 0.07 rad (with feedback) |
| Roll delta (PRE_LIFTOFF) | 0.025 + extra 0.001 | 0.07 + extra 0.04 |
| Swing leg targets (PRE_LIFTOFF) | Hip pitch +0.15 via `compute_swing_progress()` | Hip pitch +0.15, knee -0.15 |
| Swing leg targets (SS) | Complex blend with `SINGLE_SUPPORT_POSE_BLEND_TIME`, direct_pose | Explicit: hip=0.10, knee=1.05, ankle=-0.55, ramped via smoothstep |
| WBC/posture blend | No blend (pure WBC in DS phases) | 30% safe_tau blend (`ds_posture_blend=0.30`) |
| SS WBC/safe_tau blend | Complex `transition_alpha × support_alpha × max_tau_blend` | Same blend structure |

---

## 5. Single-Support Phase

| Aspect | `main.py` | `stand_progressive.py` |
|--------|-----------|----------------------|
| MPC | Used at 20 Hz with `CentroidalMPC`, force reference from QP | Same MPC, used at 20 Hz after SS establishment |
| WBC | `WholeBodyController(nv, num_contacts=4, contact_dim=3)` for support foot, optionally `num_contacts=5` with swing | `WholeBodyController(nv, num_contacts=4, contact_dim=3)` for support foot only |
| Force reference | None (WBC minimizes `‖f - f_ref‖` with `W3` weight) | `compute_corner_patch_force_reference()` distributes total fz across corners based on CoP |
| CoP feedback | `apply_measured_cop_feedback()` with filtered CoP | Initial CoP from `get_contact_metrics()`, no runtime feedback yet |
| Swing leg handling | `compute_swing_unload_factor()` for progressive unloading | Direct PD targets (knee=1.05, hip=0.10) with `lift_leg_kp=120` |
| Establishment guard | `SINGLE_SUPPORT_ESTABLISH_TIME=0.05s`, checks force and ratio | 0.60s establishment time, checks `normal_force ≥ 15N` |

---

## 6. Key Parameter Differences

| Parameter | `main.py` (config.py) | `stand_progressive.py` (PHASE_CONFIG) | Why changed |
|-----------|----------------------|--------------------------------------|-------------|
| `LOAD_SHIFT_TIME` | 0.20s | 1.50s | More time for weight transfer |
| `LOAD_SHIFT_SUPPORT_RATIO` | 0.48 | 0.72 | Actually shift weight (was barely 50/50) |
| `LOAD_SHIFT_SUPPORT_RATIO_MIN` | 0.45 | 0.55 | Achievable transition threshold |
| `LOAD_SHIFT_COM_RATIO` | 0.16 | 0.35 | Move CoM meaningfully |
| `LOAD_SHIFT_ROLL_DELTA` | 0.025 | 0.07 | Visible body lean |
| `PRE_LIFTOFF_TIME` | 0.50s | 1.00s | More time for pre-liftoff stabilization |
| `PRE_LIFTOFF_SUPPORT_RATIO` | 0.56 | WBC ramps 0.72→0.90 | Continue shifting, don't stop |
| `DS_TOTAL_FORCE_Z_WEIGHT` | 10.0 | 100.0 (removed — not needed with corner patches) | Was only needed for centroid model |
| `DS_LOAD_DISTRIBUTION_WEIGHT` | 1000.0 | 3000.0 | Stronger weight distribution enforcement |
| `DOUBLE_SUPPORT_MIN_FORCE` | 120.0 | 80.0 | More lenient transition threshold |
| `Kp_c` / `Kd_c` | 100.0 / 20.0 | 150.0 / 30.0 | More aggressive CoM tracking |
| `W1` (CoM weight) | 100.0 | 200.0 | Stronger CoM tracking in WBC |

---

## 7. Architecture Differences

| Aspect | `main.py` | `stand_progressive.py` |
|--------|-----------|----------------------|
| File structure | `main.py` + `config.py` + `phases/` module + `phase_core.py` + `phase_targets.py` + `trajectory.py` | Single file, inline config dict |
| Phase enum | `ControlPhase(Enum)` in `phase_core.py` | `PhaseId(IntEnum)` inline |
| Phase transition logic | Distributed across `phases/double_support.py`, `phases/load_shift.py`, `phases/pre_liftoff.py` | All inline in main loop |
| CoM trajectory | Pre-computed with `build_transition_com_trajectory()` | Computed per-step with `smoothstep()` |
| WBC solver creation | Created once before loop (`wbc_ds = WholeBodyController(...)`) | Recreated each step inside DS block (inefficient, should be fixed) |
| Diagnostic logging | Phase-entry prints only | Periodic `[t=X.XXs] PHASE_NAME sf=... ratio=...` prints |

---

## 8. Results Comparison

| Metric | `main.py` | `stand_progressive.py` |
|--------|-----------|----------------------|
| Highest phase reached | LOAD_SHIFT (stuck) | SINGLE_SUPPORT (complete) |
| Support ratio at LS exit | ~0.40 (target ≥0.45) | 0.73–0.93 |
| Support ratio at PL exit | N/A | 0.78–0.89 |
| Swing foot force in SS | N/A | 0N (fully lifted) |
| CoM RMSE | 0.102m | 0.107m (improvable) |
| Max foot slip | 36mm (DS phases) | 12–65mm (depends on phase) |
| MPC solve time | 0ms (never ran) | Working at 20Hz |

---

## 9. What to Migrate Back

The key improvements from `stand_progressive.py` that should be integrated into `main.py`:

1. **Corner-patch DS WBC** — Replace centroid contacts (2×4D) with corner-patch contacts (8×3D)
2. **WBC/safe_tau blending** — Add `ds_posture_blend` during DS phases
3. **Increased load-shift parameters** — Higher target ratios, longer durations, larger roll deltas
4. **Smoothstep c_ref trajectory** — Replace `compute_phase_com_target()` with smoothstep interpolation
5. **Aggressive swing leg targets** — knee=1.05, hip=0.10, ankle=-0.55 in SS
6. **Relaxed phase transition thresholds** — Lower force minimums, time-based fallbacks
7. **Corner-patch force reference** — `compute_corner_patch_force_reference()` for SS WBC