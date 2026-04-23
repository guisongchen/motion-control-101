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

## 4. Configuration / Logic Changes Applied During This Session

| Parameter | Old Value | New Value | Rationale |
|-----------|-----------|-----------|-----------|
| `SIM_DURATION` | 3.0 s | 4.0 s | Give more time for stabilization |
| `PRE_LIFTOFF_SWING_FORCE_MAX` | 110 N | 150 N | Allow entry with remaining swing contact |
| `COM_VEL_READY_THRESH` | 0.20 m/s | 0.35 m/s | Relax CoM speed requirement |
| `SINGLE_SUPPORT_MAX_TAU_BLEND` | 0.45 | 0.85 | Increase max WBC authority |
| `PRE_LIFTOFF_FORWARD_ERROR_THRESH` | N/A | 0.05 m | Block handoff when forward CoM position error is still large |
| `PRE_LIFTOFF_FORWARD_VEL_THRESH` | N/A | 0.12 m/s | Block handoff when forward CoM velocity is still large |
| `BASE_DOF_DAMPING` | 5.0 | 10.0 | Increase base damping (reverted) |
| `JOINT_DOF_DAMPING` | 10.0 | 20.0 | Increase joint damping (reverted) |
| `POSTURE_KP` | 80.0 | 120.0 | Stiffer posture (reverted) |
| `POSTURE_KD` | 12.0 | 18.0 | More posture damping (reverted) |

*Note: Damping and posture gains were reverted because they made the `LOAD_SHIFT` phase too sluggish to meet its own readiness thresholds.*

---

## 5. Follow-Up Experiments Attempted After the Initial Report

### 5.1 Experiment Log

| ID | Change | Key Observations | Outcome |
|----|--------|------------------|---------|
| A | **Model residual swing contact as a 5th WBC contact** during early `SINGLE_SUPPORT` | WBC now solves with the correct contact topology when the swing foot still carries load. At `t=2.996s`: CoM `[0.145, -0.064, 0.662]`, swing force `38.6 N`, support slip `17.4 mm`. Max foot slip `~305 mm`. | **Helpful but insufficient.** Removes the worst model mismatch, but the robot still falls. |
| B | **Uncap WBC authority immediately** in `SINGLE_SUPPORT` (`SINGLE_SUPPORT_MAX_TAU_BLEND` available without waiting for `single_support_established`) | Max foot slip increased from `~305 mm` to `~455 mm`. CoM RMSE `0.2421 m`. | **Negative result.** More support torque authority alone does not fix the problem; it can amplify a bad handoff state. |
| C | **Blend toward the optimized single-support pose earlier** in `PRE_LIFTOFF` and add **forward CoM PD** on support-leg pitch | Entered `SINGLE_SUPPORT` slightly earlier (`t=2.617s`). At `t=2.996s`: CoM `[0.194, -0.099, 0.646]`, swing force `0 N`, CoM speed `0.298 m/s`. Max foot slip `~375.8 mm`. | **Mixed result.** Swing-foot unloading improved, but the forward state at handoff was still too dynamic. |
| D | **Start MPC by elapsed time** after `SINGLE_SUPPORT_MPC_DELAY` instead of waiting for `single_support_established` | MPC solved in `SINGLE_SUPPORT` (`~1.12 ms` at `t=2.996s`), proving the establishment gate was no longer blocking the controller path. Max foot slip `~399.4 mm`. | **Helpful for diagnosis, not for stabilization.** MPC was active, but it still could not recover the bad entry state. |
| E | **Tighten the `PRE_LIFTOFF -> SINGLE_SUPPORT` gate** using explicit forward CoM limits (`0.05 m`, `0.12 m/s`) | The robot **never entered `SINGLE_SUPPORT`**. At `t=2.996s` while still in `PRE_LIFTOFF`: forward error `0.134 m`, forward velocity `0.243 m/s`. Max foot slip `~340.6 mm`. | **Useful diagnostic result.** The handoff gate now correctly blocks clearly bad forward states. |
| F | **Activate support-foot WBC already in `PRE_LIFTOFF`** (keep MPC off there) | WBC solved in `PRE_LIFTOFF` (`~0.21 ms`), but the state got worse. At `t=2.996s`: forward error `0.213 m`, forward velocity `0.548 m/s`. Max foot slip `~651.4 mm`. | **Rejected.** Full support-foot WBC during the double-contact unloading phase is too aggressive / mismatched. |

### 5.2 Stable Checkpoint vs Withdrawn Experiment

- **Committed checkpoint:** `14dbeb3` (`Improve single-support handoff control`)
  - Includes experiments **A-E**:
    - 5th swing-contact modeling in early `SINGLE_SUPPORT`
    - Full single-support WBC authority
    - Earlier optimized-pose blending and forward CoM PD in `PRE_LIFTOFF`
    - Time-based MPC activation in `SINGLE_SUPPORT`
    - Forward-error / forward-velocity handoff gate
- **Withdrawn experiment:** **F** (active WBC already in `PRE_LIFTOFF`)
  - This experiment was run after `14dbeb3`, produced significantly worse results, and has been removed from `main.py`.
  - It remains documented here because it materially changed the diagnosis: a full direct single-support WBC strategy should not be moved earlier into the double-contact unloading phase.

---

## 6. Current Interpretation

The follow-up experiments substantially narrowed the failure source:

1. **The original model-mismatch problem is real and is now fixed.**
   - Adding the swing foot as a temporary 5th contact removed the most obvious WBC dynamics inconsistency at single-support entry.
2. **Low WBC authority was not the dominant root cause.**
   - Removing the pre-establish blend cap made performance worse, not better.
3. **The MPC establishment gate was not the dominant blocker either.**
   - Once MPC was allowed to run on time, it solved normally but still could not recover the entry disturbance.
4. **The remaining bottleneck is upstream of `SINGLE_SUPPORT`.**
   - The real problem is that `PRE_LIFTOFF` still cannot reduce forward CoM error / velocity enough before handoff.
5. **A full direct single-support WBC strategy should not simply be moved into `PRE_LIFTOFF`.**
   - In the double-contact unloading phase, that control law is too aggressive and destabilizing.

---

## 7. Recommended Next Move

The next experiment should **not** be more single-support force authority.  
It should be a **bounded, simpler forward-balance controller in `PRE_LIFTOFF`**, for example:

- keep the stricter forward handoff gate,
- keep the earlier optimized-pose blending,
- **remove the `PRE_LIFTOFF` full WBC experiment**, and
- add a limited support-leg **ankle/hip pitch strategy** driven by forward CoM error and forward CoM velocity.

This is the most plausible next step because the current evidence says the controller must arrive at handoff with a calmer forward state, rather than trying to recover after the disturbance is already too large.

---

## 8. Test Commands

```bash
# Run the full transition simulation
uv run python mpc_and_wbc/week10_single_leg_stand/main.py

# Run the direct single-support benchmark for comparison
uv run python mpc_and_wbc/week10_single_leg_stand/direct_single_support.py
```
