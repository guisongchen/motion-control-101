# Week 10 Single-Leg Stand Tuning Report

**Date:** 2026-04-22  
**Branch:** `direct-single-support-contact-point`  
**Scope:** Refactor `main.py` to reuse the proven direct single-support WBC strategy inside the double-to-single-support phase machine.

---

## 1. Progress

### 1.1 Architecture Complete
- `main.py` now imports and uses the proven corner-patch WBC primitives from `direct_single_support.py`:
  - CoP feedback (`apply_measured_cop_feedback`)
  - Slip force feedback (XY restoring force on support foot position error)
  - Yaw moment feedback (PD on support foot yaw error)
  - Wrench task objective (`build_corner_patch_wrench_task`)
- Infrastructure fixes applied:
  - Resolved circular import by moving shared functions to `utils.py`
  - Fixed matplotlib headless crash (`matplotlib.use("Agg")`)
  - Added forward-drift damping in the CoM reference builder

### 1.2 Phase Machine Works
The robot successfully transitions through all phases:

```
INIT_SETTLE (0.20 s)
  -> DOUBLE_SUPPORT_HOLD (~0.60 s)
    -> LOAD_SHIFT (~1.15 s)
      -> PRE_LIFTOFF (~0.70 s)
        -> SINGLE_SUPPORT (t ~ 2.64 s)
```

### 1.3 WBC Runs in Closed Loop
- Corner-patch QP solves successfully (~0.2 ms, well under 0.5 ms threshold)
- MPC force blending activates correctly after `SINGLE_SUPPORT_MPC_DELAY`
- Fallback logging and torque blending logic are functional

---

## 2. Remaining Problems

The robot **falls during SINGLE_SUPPORT** every time. Observed sequence:

| Time | Phase | CoM (m) | Support slip | Swing force | Notes |
|------|-------|---------|--------------|-------------|-------|
| 2.64 s | `SINGLE_SUPPORT` entry | `[0.10, -0.06, 0.66]` | ~0 mm | ~130 N | Entry triggered with forward drift already present |
| 2.99 s | `SINGLE_SUPPORT` early | `[0.14, -0.06, 0.66]` | 17.7 mm | 0 N | Support foot slips; swing foot lifts off |
| 3.60 s | `SINGLE_SUPPORT` mid | `[0.40, 0.15, 0.30]` | 154 mm | 0 N | Support force drops to 0 N; WBC falls back to safe torque |
| 4.00 s | `SINGLE_SUPPORT` end | `[0.44, 0.30, 0.18]` | 433 mm | 0 N | Both feet slipped; CoM collapses |

### 2.1 Root Cause 1: Unmodeled Swing Contact
- The WBC is initialized with `num_contacts=4` (corner patch on the support foot **only**).
- Upon entering `SINGLE_SUPPORT`, the swing foot still carries `~130 N` of contact force.
- The WBC dynamics model (`M vdot + C = tau + J_c^T f`) does not include this 5th contact, creating a model mismatch.
- The mismatch drives the support foot to slip because the commanded torques expect a different net external wrench than reality provides.

### 2.2 Root Cause 2: Low WBC Authority During Early SINGLE_SUPPORT
- `single_support_established` requires `support_slip <= SLIP_THRESH` (5 mm) and other conditions.
- Slip exceeds 5 mm almost immediately, so `single_support_established` never becomes `True`.
- Until established, the WBC torque blend is capped at `SINGLE_SUPPORT_PRE_ESTABLISH_TAU_BLEND = 0.20`.
- This means **80% of the torque is passive PD posture control**, which cannot actively balance the CoM.
- By the time the blend could increase, the foot has already slipped beyond recovery.

### 2.3 Root Cause 3: Perturbed Initial State
- `direct_single_support.py` works because it spawns the robot **at rest** in a tuned single-support pose (base shifted laterally, support leg pre-bent, arms counterbalancing).
- Our transition enters `SINGLE_SUPPORT` with:
  - Non-zero CoM velocity (~0.05-0.10 m/s)
  - Forward CoM offset (~0.10 m ahead of support foot)
  - Arms hanging down (no counterbalance)
  - The WBC (with `Jc_dot = 0`) struggles to recover from this perturbation before the foot slips.

---

## 3. What Has Been Ruled Out

| Hypothesis | Evidence | Verdict |
|------------|----------|---------|
| Corner patch positions are wrong | Measured span: `0.17 m x 0.06 m`, consistent with foot geometry | **Ruled out** |
| WBC solver is failing | Status val = 1, solve time ~0.2 ms, no OSQP errors | **Ruled out** |
| Transition thresholds are too strict | Relaxed to 150 N swing force, 0.35 m/s CoM speed; entry succeeds | **Ruled out** |
| Corner patch Jacobian shape mismatch | `J_c.shape = (24, 29)`, `_linear_contact_jacobian` correctly extracts `(12, 29)` | **Ruled out** |

---

## 4. Configuration Changes Applied During This Session

| Parameter | Old Value | New Value | Rationale |
|-----------|-----------|-----------|-----------|
| `SIM_DURATION` | 3.0 s | 4.0 s | Give more time for stabilization |
| `PRE_LIFTOFF_SWING_FORCE_MAX` | 110 N | 150 N | Allow entry with remaining swing contact |
| `COM_VEL_READY_THRESH` | 0.20 m/s | 0.35 m/s | Relax CoM speed requirement |
| `SINGLE_SUPPORT_PRE_ESTABLISH_TAU_BLEND` | 0.20 | 0.70 | Increase WBC authority early (reverted from 0.20->0.70) |
| `SINGLE_SUPPORT_MAX_TAU_BLEND` | 0.45 | 0.85 | Increase max WBC authority |
| `BASE_DOF_DAMPING` | 5.0 | 10.0 | Increase base damping (reverted) |
| `JOINT_DOF_DAMPING` | 10.0 | 20.0 | Increase joint damping (reverted) |
| `POSTURE_KP` | 80.0 | 120.0 | Stiffer posture (reverted) |
| `POSTURE_KD` | 12.0 | 18.0 | More posture damping (reverted) |

*Note: Damping and posture gains were reverted because they made the `LOAD_SHIFT` phase too sluggish to meet its own readiness thresholds.*

---

## 5. Likely Next Steps (Not Yet Attempted)

### 5.1 Model the Swing Foot as a 5th Contact Point
- During early `SINGLE_SUPPORT`, when swing force is still significant, include the swing foot as an additional contact point in the WBC.
- Gradually reduce its weight or remove it once swing force drops below a threshold (e.g., 20 N).
- This eliminates the dynamics model mismatch at the critical entry moment.

### 5.2 Start WBC at High Blend Immediately
- Bypass the `single_support_established` guard and start with `SINGLE_SUPPORT_MAX_TAU_BLEND` immediately upon entering `SINGLE_SUPPORT`.
- The establishment guard was designed for safety, but in this case the low blend is the primary cause of instability.
- Alternatively, base the blend on elapsed time in `SINGLE_SUPPORT` rather than on establishment conditions.

### 5.3 Reduce the Initial Perturbation
- Improve `PRE_LIFTOFF` balance control:
  - Use a simplified balance controller (e.g., ankle strategy or LIPM-based CoM feedback) to actively drive CoM velocity toward zero before liftoff.
  - Apply arm counterbalance targets during `PRE_LIFTOFF` to match the optimized single-support pose.
  - Consider pre-positioning the base closer to the optimized pose (lateral shift, roll) before entering `SINGLE_SUPPORT`.

### 5.4 Compute Non-Zero `Jc_dot`
- Currently `Jc_dot = np.zeros_like(J_c)` is passed to `wbc.solve`.
- Computing the actual contact Jacobian time derivative would make the no-slip constraint account for current foot velocity, potentially reducing initial slip.
- However, this is a secondary issue compared to the unmodeled contact and low blend problems.

---

## 6. Test Commands

```bash
# Run the full transition simulation
uv run python mpc_and_wbc/week10_single_leg_stand/main.py

# Run the direct single-support benchmark for comparison
uv run python mpc_and_wbc/week10_single_leg_stand/direct_single_support.py
```
