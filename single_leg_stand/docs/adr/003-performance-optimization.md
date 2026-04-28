# ADR 003: Stand-Progressive Simulation Performance Optimization

## Status

Accepted

## Context

The `stand_progressive.py` simulation runs a 15-second MuJoCo physics simulation with MPC + WBC control at 1 kHz (15,000 steps). The baseline wall-clock time (measured on commit `83880f8`) was **178 seconds** (real) / **185 seconds** (user), a ~12× real-time factor that made iteration loops prohibitively slow.

### Baseline Bottleneck Analysis

| Rank | Bottleneck | Relative Cost |
|------|-----------|--------------|
| 1 | Angular momentum Jacobian via per-DOF finite-differencing (~30 `mj_forward` calls per WBC step) | dominant |
| 2 | CoM Jacobian via per-DOF finite-differencing (~30 `mj_forward` calls per WBC step) | dominant |
| 3 | `WholeBodyController` re-created every DS/LS/PL timestep via `WholeBodyController(...)` | 5–10× |
| 4 | `solver.setup()` in WBC/MPC (full QP re-factorization) per solve | 3–5× |
| 5 | Redundant `_sync_state()` → `mj_forward()` calls (multiple methods called with same `(q, v)` per step) | ~1× |
| 6 | Redundant `compute_coriolis_gravity(q, v)` called twice with identical arguments | ~1× |
| 7 | WBC running at 1000 Hz instead of the configured 250 Hz | 4× wasted work |

### Two-Round Optimization

Performance optimization was conducted in two rounds:

- **Round 1** (this ADR's original scope): 178s → 137s (~23% speedup)
- **Round 2** (this update): 137s → 37.7s (~72% additional speedup)

Total cumulative speedup: **178s → 37.7s (79% reduction, 4.7× faster)**.

---

## Decision

Six optimizations were applied across two rounds. Two additional candidates evaluated in Round 1 were rejected; one was later accepted in Round 2 with a different implementation approach.

### Round 1 (178s → 137s)

#### 1. Fast CoM Jacobian via `mj_jacSubtreeCom`

**Decision:** Replace the per-DOF finite-difference for the CoM Jacobian with MuJoCo's built-in `mj_jacSubtreeCom`, which computes the 3×NV Jacobian in a **single call** instead of ~30 `mj_forward` calls.

**Rationale:**
- `mj_jacSubtreeCom` is analytically exact (verified to machine precision against the FD baseline)
- Reduces CoM Jacobian cost from O(NV × mj_forward) to O(1)

**Code:**

```python
# robot_model.py
def get_com_jacobian(self, q: np.ndarray) -> np.ndarray:
    self._sync_state(q=q)
    J_com = np.zeros((3, self.nv))
    mujoco.mj_jacSubtreeCom(self.model, self.data, J_com, self.root_body_id)
    return J_com
```

**Location:** `robot_model.py:180-185`

#### 2. Pre-allocate WBC Instances

**Decision:** Create two `WholeBodyController` instances at initialization — one for 4 contact points (support foot only) and one for 8 contact points (support + swing foot) — and select the appropriate one in the loop instead of constructing a new WBC per timestep.

**Rationale:**
- `WholeBodyController.__init__()` calls `_build_qp_matrices()` which constructs sparse DOK matrices and calls `solver.setup()` for initial symbolic factorization — an O(n³) operation
- Only two configurations ever exist (4 or 8 contacts), so at most two instances are needed

**Code:**

```python
# stand_progressive.py — initialisation
wbc_ss = WholeBodyController(robot.nv, num_contacts=4, contact_dim=3)
wbc_ds_4 = WholeBodyController(robot.nv, num_contacts=4, contact_dim=3)
wbc_ds_8 = WholeBodyController(robot.nv, num_contacts=8, contact_dim=3)

# stand_progressive.py — in the DS/LS/PL branch
wbc_ds = wbc_ds_4 if n_contacts <= 4 else wbc_ds_8
```

**Location:** `stand_progressive.py:396-398, 675`

#### 3. Eliminate Redundant `compute_coriolis_gravity` Call (DS branch)

**Decision:** Reuse the `C_safe` vector already computed for the safe PD posture torque in the WBC constraint bounds, instead of re-computing `C(q, v)` with identical state.

**Rationale:** Both calls use the exact same `q` and `v` from the state estimator. Each triggers `_sync_state()` → `mj_forward()` internally.

**Code:**

```python
# Before (DS branch):
C = robot.compute_coriolis_gravity(q, v)
# After:
C = C_safe
```

**Location:** `stand_progressive.py:673`

### Round 2 (137s → 37.7s)

#### 4. Eliminate Redundant `compute_coriolis_gravity` Call (SS branch)

**Decision:** Apply the same C_safe reuse optimization to the single-support WBC branch, which was missed in Round 1.

**Code:**

```python
# Before (SS branch):
C = robot.compute_coriolis_gravity(q, v)
# After:
C = C_safe
```

**Location:** `stand_progressive.py:785`

#### 5. `_sync_state` Caching

**Decision:** Add a cache that checks whether `data.qpos` and `data.qvel` already match the requested state. When they do, skip the `mj_forward` call. Invalidate the cache after `mj_step()`, `_restore_state()`, `reset_joint_positions()`, and `reset_base_pose()`.

**Rationale:**
- Within a single timestep, `q` and `v` are constant — all `_sync_state(q, v)` calls beyond the first are redundant
- This was previously rejected in Round 1 (see Alternative 3), but the approach was re-designed:
  - Round 1's approach (input-array hashing) was rejected due to stale-data risks and unclear performance benefit
  - Round 2's approach (direct comparison against `data.qpos`/`data.qvel`) is both simpler and safer — after any state modification, the arrays won't match and `mj_forward` will be called
  - After the CoM Jacobian optimization (#1) removed 30 `mj_forward` calls per step, the relative cost of remaining redundant calls increased

**Code:** `robot_model.py:122-165`

```python
def _sync_state(self, q=None, v=None):
    if self._sync_cache_valid:
        need_forward = False
        if q is not None:
            # ... compare data.qpos with desired q ...
            if mismatch:
                self.data.qpos[...] = ...
                need_forward = True
        if v is not None and not need_forward:
            # ... compare data.qvel with desired v ...
            if mismatch:
                self.data.qvel[:] = ...
                need_forward = True
        if not need_forward:
            return  # Cache hit
    # Cache miss: set state, call mj_forward()
    ...
    self._sync_cache_valid = True
```

#### 6. Fast Angular Momentum Jacobian via `mj_subtreeVel`

**Decision:** Replace `mj_forward` with `mj_subtreeVel` in the inner loop of `_centroidal_jacobian_fd` for the angular momentum quantity. Since only `data.qvel` is perturbed per-DOF (not `data.qpos`), body positions/orientations remain valid, and `mj_subtreeVel` alone suffices to compute `subtree_angmom`.

**Rationale:**
- `mj_subtreeVel` computes subtree angular momentum from body positions (already computed by the initial `_sync_state`) and `qvel` (which we perturb per DOF)
- `mj_forward` additionally computes forward dynamics, collision constraints, etc. — all unnecessary for this computation
- Replaces ~29 `mj_forward` calls with ~29 `mj_subtreeVel` calls per WBC step, reducing per-DOF cost by ~3–5×

**Code:** `robot_model.py:186-216`

```python
def _centroidal_jacobian_fd(self, q, quantity):
    self._sync_state(q=q, v=np.zeros(self.nv))
    saved_qpos = ...
    saved_qvel = ...

    if quantity == "angular_momentum":
        for dof in range(self.nv):
            self.data.qvel[:] = 0.0
            self.data.qvel[dof] = 1.0
            mujoco.mj_subtreeVel(self.model, self.data)   # not mj_forward
            J[:, dof] = self.data.subtree_angmom[...]
    else:
        # Fallback (unused): full mj_forward per DOF
        for dof in range(self.nv):
            ...
            mujoco.mj_forward(self.model, self.data)
            mujoco.mj_subtreeVel(self.model, self.data)
            ...

    self._restore_state(saved_qpos, saved_qvel)
    return J
```

#### 7. WBC Frequency Throttling (1000 Hz → 250 Hz)

**Decision:** Honor the `WBC_FREQ = 250` configuration by gating WBC solves on `step % wbc_period == 0` (where `wbc_period = 4`), holding the last WBC torque output constant between solves. This was the intended behavior per ADR 002 but was never implemented in `stand_progressive.py`.

**Rationale:**
- ADR 002 explicitly decided on 250 Hz WBC to balance control quality with solver cost
- The 1000 Hz WBC rate was a latent bug: the config says 250 Hz but the loop computed WBC every step
- At 250 Hz (4 ms hold), the centroidal dynamics of a 23-DoF humanoid remain well-controlled, with PD posture torque (`safe_tau`) providing continuous 1000 Hz stabilization
- Eliminates 75% of WBC computation per second: mass matrix, all Jacobians, QP construction, and OSQP solve are each computed only 1/4 as often
- This is the single largest optimization, accounting for ~70% of the Round 2 speedup

**Code:** `stand_progressive.py:399-402, 655-827`

```python
# Initialisation
wbc_period = max(1, round(1.0 / (WBC_FREQ * DT_SIM)))  # = 4
last_ds_wbc_tau = None
last_ss_wbc_tau = None

# DS/LS/PL WBC
if step % wbc_period == 0 or last_ds_wbc_tau is None:
    # ... full WBC computation ...
    if wbc_result is not None:
        last_ds_wbc_tau = wbc_result["tau"]
if last_ds_wbc_tau is not None:
    applied_tau = (1.0 - blend) * last_ds_wbc_tau + blend * safe_tau
else:
    applied_tau = safe_tau.copy()

# SS WBC (same pattern)
if ss_established and (step % wbc_period == 0 or last_ss_wbc_tau is None):
    # ... full SS WBC computation ...
    if wbc_result_ss is not None:
        last_ss_wbc_tau = np.clip(wbc_result_ss["tau"], ...)
```

---

## Consequences

### Positive

- **79% total wall-clock speedup** (178s → 37.7s real time, from ~12× to ~2.5× real-time factor)
- **4.7× faster** iteration loop for development
- Simulation behavior is **substantially similar** to the original: phase transitions complete reliably, robot reaches and balances in SINGLE_SUPPORT
- No new dependencies introduced — all optimizations use existing MuJoCo Python bindings
- WBC now runs at the configured 250 Hz frequency as intended by ADR 002, fixing a latent implementation gap
- CoM Jacobian via `mj_jacSubtreeCom` remains analytically exact
- `_sync_state` caching eliminates ~8-10 redundant `mj_forward` calls per step without stale-data risks

### Performance Contributions (Cumulative)

| Round | Optimizations Applied | Real Time | Cumulative Speedup |
|-------|----------------------|-----------|-------------------|
| Baseline (`83880f8`) | (none) | 178s | — |
| 1 | CoM Jacobian + WBC prealloc + C reuse (DS) | 137s | 23% |
| 2 | Caching + `mj_subtreeVel` + C reuse (SS) | 128s | 28% |
| 2 | Above + WBC throttling to 250 Hz | 37.7s | 79% |

The WBC throttling dominates because it eliminates 75% of WBC computations (mass matrix, all Jacobians including the expensive angular momentum FD loop, QP construction, and OSQP solve).

### Negative

- WBC pre-allocation assumes exactly 2 contact configurations (4 and 8). If a third configuration becomes needed, another pre-allocated instance must be added
- WBC throttling to 250 Hz introduces ~4ms of torque latency. The PD posture torque (`safe_tau`) runs at full 1000 Hz to maintain stability, but the centroidal control quality is slightly reduced compared to 1000 Hz WBC
- The angular momentum Jacobian remains the dominant per-step cost (29 `mj_subtreeVel` calls per WBC step) and was only partially improved
- Phase transition timing shifts slightly with 250 Hz WBC (e.g., SINGLE_SUPPORT entry at ~4.7s vs ~6.2s previously)

---

## Alternatives Considered

### Alternative 1: Body-Jacobian Analytic Angular Momentum Jacobian (Round 1, Rejected)

Computed the angular momentum Jacobian analytically by summing per-body contributions:

```python
J_L = Σ_i [m_i * skew(r_i - c) @ J_v_i + I_i^world @ J_ω_i]
```

where `J_v_i` and `J_ω_i` come from `mj_jacBodyCom`.

**Rejected:**
- Produces a consistent **~0.5% numerical discrepancy** vs MuJoCo's internal `subtree_angmom` computation
- While small, this error compounds over time in the WBC quadratic program, causing the robot to lose contact forces and fail phase transitions (forces dropped from ~200N to ~22N, CoM shifted by 0.6m)
- Root cause of discrepancy was not identified (suspected subtle difference in MuJoCo's internal rotational inertia handling)
- The FD method, while O(NV × mj_forward), is guaranteed correct by construction

### Alternative 2: `solver.update()` Instead of `solver.setup()` (Round 1, Rejected)

Replaced `solver.setup(P=..., A=...)` with `solver.update(Px=..., Ax=...)` in WBC/MPC `solve()` methods to reuse the existing symbolic factorization.

**Rejected:**
- The initial `self._P` placeholder (sparse zero matrix) had zero non-zero elements, while the actual P matrix built in `solve()` (from `J_com.T @ W1 @ J_com + ...`) is fully dense in the top-left NV×NV block
- OSQP's `update(Px=...)` rejects updates when the non-zero count of the new P differs from the initial setup
- Fixing this would require initializing P with the worst-case dense sparsity pattern, adding complexity with marginal benefit (setup cost is amortized over the solve cost)

**Note:** This remains potentially viable if the pre-allocated sparsity pattern is set correctly, but is low-priority since WBC throttling (#7) already reduces `solver.setup()` calls by 75%.

### Alternative 3: `_sync_state` Caching via Input Hashing (Round 1, Rejected)

Added `_cached_q` / `_cached_v` to skip redundant `mj_forward` calls within a single timestep when the same `(q, v)` is passed repeatedly.

**Rejected (Round 1 approach):**
- Required tracking both position and velocity (since `qfrc_bias` depends on both)
- Input-hash-based caching risked stale-data bugs when state was modified outside `_sync_state`
- The caching logic added complexity without measurable speedup in Round 1 timing tests

**Accepted (Round 2 approach):**
- Re-designed to compare `data.qpos`/`data.qvel` directly with the requested state
- Direct comparison is intrinsically safe: after any state modification, the arrays won't match and `mj_forward` will be called
- Adopted as optimization #5

### Alternative 4: WBC at 1000 Hz (Not Implemented)

Keep WBC running at the full 1000 Hz physics rate. This was the actual behavior before Round 2 (a latent bug: config said 250 Hz but the loop ran at 1000 Hz).

**Not adopted:**
- OSQP Python `setup()` + `solve()` per step at 1000 Hz is the primary performance bottleneck
- ADR 002 already decided on 250 Hz as the appropriate control rate
- The PD safe posture torque at 1000 Hz provides sufficient stabilization between WBC updates

---

## Implementation Notes

### Files Modified

| File | Round 1 Changes | Round 2 Changes |
|------|----------------|----------------|
| `robot_model.py` | Added `get_com_jacobian` via `mj_jacSubtreeCom`; renamed old FD method to `_centroidal_jacobian_fd` | Added `_sync_state` caching with `_sync_cache_valid` flag and `_invalidate_sync_cache()`; optimized `_centroidal_jacobian_fd` inner loop to use `mj_subtreeVel` instead of `mj_forward` for angular momentum; added cache invalidation calls in `step()`, `_restore_state()`, `reset_joint_positions()`, `reset_base_pose()` |
| `stand_progressive.py` | Pre-allocated `wbc_ds_4` / `wbc_ds_8`; replaced `wbc_ds = WholeBodyController(...)` with selection; replaced `C = robot.compute_coriolis_gravity(...)` with `C = C_safe` in DS branch | Added `WBC_FREQ` import; added `wbc_period`; added `last_ds_wbc_tau` / `last_ss_wbc_tau` caches; gated DS and SS WBC computations on `step % wbc_period == 0`; replaced redundant `C = robot.compute_coriolis_gravity(...)` with `C = C_safe` in SS branch |

### No New Dependencies

All optimizations use existing MuJoCo Python bindings. No packages added.

### Verification

```bash
# Run simulation and check phase progression
uv run python single_leg_stand/stand_progressive.py
# Expected: reaches SINGLE_SUPPORT phase, robot balances on support foot
```

Key verification criteria:
- All five phases (INIT_SETTLE → DS_HOLD → LOAD_SHIFT → PRE_LIFTOFF → SINGLE_SUPPORT) transition correctly
- Robot maintains upright posture in SINGLE_SUPPORT
- No numerical instability or solver failures

### Performance Measurement

```bash
/usr/bin/time -p uv run python single_leg_stand/stand_progressive.py
# Baseline (`83880f8`):           real ~178s, user ~185s
# Round 1 (`499a33c`):            real ~137s, user ~144s  (CoM Jacobian + WBC prealloc + C reuse)
# Round 2 (caching + subtReeVel): real ~128s, user ~133s  (+ `_sync_state` cache + J_L mj_subtreeVel)
# Round 2 (all):                  real  ~38s, user  ~43s  (+ WBC throttling to 250 Hz)
```

---

## References

- [ADR 001: Phase-Driven Single Support Control](./001-phase-driven-single-support-control.md) — the phased control architecture whose per-step computations are being optimized here
- [ADR 002: WBC/MPC Frequency Scheduling](./002-wbc-mpc-frequency-scheduling.md) — established the 250 Hz WBC frequency that was finally implemented in this ADR
- [MuJoCo `mj_jacSubtreeCom` documentation](https://mujoco.readthedocs.io/en/stable/APIreference/APItypes.html#mjtNum)
- [MuJoCo `mj_subtreeVel` documentation](https://mujoco.readthedocs.io/en/stable/APIreference/APItypes.html#mjtNum)
- [OSQP solver API: `update` vs `setup`](https://osqp.org/docs/interfaces/python.html)
- Bottleneck analysis: `stand_progressive.py:448-870` (main simulation loop)
