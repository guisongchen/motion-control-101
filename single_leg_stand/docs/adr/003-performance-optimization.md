# ADR 003: Stand-Progressive Simulation Performance Optimization

## Status

Accepted

## Context

The `stand_progressive.py` simulation runs a 15-second MuJoCo physics simulation with MPC + WBC control at 1 kHz (15,000 steps). The baseline wall-clock time (measured on commit `83880f8`) was **178 seconds** (real) / **185 seconds** (user), a ~12× real-time factor that made iteration loops prohibitively slow.

A bottleneck analysis identified the most expensive per-step operations:

| Rank | Bottleneck | Relative Cost |
|------|-----------|--------------|
| 1 | Centroidal Jacobian via per-DOF finite-differencing (~60 `mj_forward` calls per WBC step) | 50–100× |
| 2 | `WholeBodyController` re-created every DS/LS/PL timestep via `WholeBodyController(...)` | 5–10× |
| 3 | `solver.setup()` in WBC/MPC (full QP re-factorization) per solve | 3–5× |
| 4 | Redundant `compute_coriolis_gravity(q, v)` called twice with identical arguments | ~1× |
| 5 | `get_foot_jacobian()` triggers redundant `mj_forward` per contact point (4–8 calls) | ~1× |

## Decision

We applied three optimizations totaling **~23% speedup** (178s → 137s real time). Two additional candidates were evaluated and rejected.

### 1. Fast CoM Jacobian via `mj_jacSubtreeCom`

**Decision:** Replace the per-DOF finite-difference `_centroidal_jacobian_from_unit_velocities` for the CoM Jacobian with MuJoCo's built-in `mj_jacSubtreeCom`, which computes the 3×NV Jacobian in a **single call** instead of ~30 `mj_forward` calls.

**Rationale:**
- `mj_jacSubtreeCom` is analytically exact (verified to machine precision against the FD baseline)
- Reduces CoM Jacobian cost from O(NV × mj_forward) to O(1)
- The original method perturbed each of ~29 generalized velocities individually, calling `mj_forward` + `mj_subtreeVel` for each

**Code:**

```python
# robot_model.py
def get_com_jacobian(self, q: np.ndarray) -> np.ndarray:
    """Compute center-of-mass Jacobian via mj_jacSubtreeCom, shape (3, nv)."""
    self._sync_state(q=q)
    J_com = np.zeros((3, self.nv))
    mujoco.mj_jacSubtreeCom(self.model, self.data, J_com, self.root_body_id)
    return J_com
```

**Location:** `single_leg_stand/robot_model.py:180-185`

### 2. Pre-allocate WBC Instances

**Decision:** Create two `WholeBodyController` instances at initialization — one for 4 contact points (support foot only) and one for 8 contact points (support + swing foot) — and select the appropriate one in the loop instead of constructing a new WBC per timestep.

**Rationale:**
- `WholeBodyController.__init__()` calls `_build_qp_matrices()` which constructs sparse DOK matrices and calls `solver.setup()` for initial symbolic factorization — an O(n³) operation
- Only two configurations ever exist (4 or 8 contacts), so at most two instances are needed
- Eliminates ~5,000 object constructions per 15s simulation

**Code:**

```python
# stand_progressive.py — initialisation
wbc_ss = WholeBodyController(robot.nv, num_contacts=4, contact_dim=3)
wbc_ds_4 = WholeBodyController(robot.nv, num_contacts=4, contact_dim=3)
wbc_ds_8 = WholeBodyController(robot.nv, num_contacts=8, contact_dim=3)

# stand_progressive.py — in the DS/LS/PL branch
wbc_ds = wbc_ds_4 if n_contacts <= 4 else wbc_ds_8
```

**Location:** `single_leg_stand/stand_progressive.py:396-398, 671`

### 3. Eliminate Redundant `compute_coriolis_gravity` Call

**Decision:** Reuse the `C_safe` vector already computed for the safe PD posture torque in the WBC constraint bounds, instead of re-computing `C(q, v)` with identical state.

**Rationale:**
- Both calls use the exact same `q` and `v` from the state estimator (neither is modified between)
- Each call triggers `_sync_state()` → `mj_forward()` internally
- Eliminates 1 redundant `mj_forward` call per WBC timestep

**Code:**

```python
# Before (DS branch):
C = robot.compute_coriolis_gravity(q, v)  # redundant

# After:
C = C_safe  # reuse already-computed value
```

**Location:** `single_leg_stand/stand_progressive.py:673`

## Consequences

### Positive

- **~23% wall-clock speedup** (178s → 137s real time for 15s simulation, from ~12× to ~9× real-time factor)
- Simulation behavior is **identical** to the original (same CoM tracking, same phase transitions, same force profiles)
- No new dependencies introduced
- Code is cleaner: pre-allocated WBC instances eliminate a long-lived code smell

### Negative

- WBC pre-allocation assumes exactly 2 contact configurations (4 and 8). If a third configuration becomes needed, another pre-allocated instance must be added
- The CoM Jacobian and angular momentum Jacobian are now computed via separate paths (CoM = `mj_jacSubtreeCom`, angular momentum = finite-difference). This means two `_sync_state` calls instead of one combined call, adding 1 extra `mj_forward` per WBC step
- The angular momentum Jacobian remains the dominant per-step cost and was not improved

## Alternatives Considered

### Alternative 1: Body-Jacobian Angular Momentum Jacobian

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

### Alternative 2: `solver.update()` Instead of `solver.setup()`

Replaced `solver.setup(P=..., A=...)` with `solver.update(Px=..., Ax=...)` in WBC/MPC `solve()` methods to reuse the existing symbolic factorization.

**Rejected:**
- The initial `self._P` placeholder (sparse zero matrix) had zero non-zero elements, while the actual P matrix built in `solve()` (from `J_com.T @ W1 @ J_com + ...`) is fully dense in the top-left NV×NV block
- OSQP's `update(Px=...)` rejects updates when the non-zero count of the new P differs from the initial setup
- Fixing this would require initializing P with the worst-case dense sparsity pattern, adding complexity with marginal benefit (setup cost is amortized over the solve cost)

### Alternative 3: `_sync_state` Caching

Added `_cached_q` / `_cached_v` to skip redundant `mj_forward` calls within a single timestep when the same `(q, v)` is passed repeatedly.

**Rejected:**
- Requires tracking both position and velocity (since `qfrc_bias` depends on both)
- When v changes but q stays the same (common between estimator steps), `mj_forward` is still needed to recompute `qfrc_bias`
- The caching logic added complexity without measurable speedup in timing tests
- Risk of stale-data bugs outweighed the marginal benefit

## Implementation Notes

### Files Modified

| File | Changes |
|------|---------|
| `single_leg_stand/robot_model.py` | Added `get_com_jacobian` via `mj_jacSubtreeCom`; renamed old FD method to `_centroidal_jacobian_fd` |
| `single_leg_stand/stand_progressive.py` | Pre-allocated `wbc_ds_4` / `wbc_ds_8`; replaced `wbc_ds = WholeBodyController(...)` with selection; replaced `C = robot.compute_coriolis_gravity(...)` with `C = C_safe` |

### No New Dependencies

All optimizations use existing MuJoCo Python bindings. No packages added.

### Verification

```bash
# Run simulation and check phase progression
uv run python single_leg_stand/stand_progressive.py
# Expected: reaches SINGLE_SUPPORT phase, CoM RMSE < 0.05 m
```

### Performance Measurement

```bash
/usr/bin/time -p uv run python single_leg_stand/stand_progressive.py
# Before (commit 83880f8): real ~178s, user ~185s
# After  (commit 499a33c):  real ~137s, user ~144s
```

## References

- [ADR 001: Phase-Driven Single Support Control](./001-phase-driven-single-support-control.md) — the phased control architecture whose per-step computations are being optimized here
- [ADR 002: WBC/MPC Frequency Scheduling](./002-wbc-mpc-frequency-scheduling.md) — related decision on WBC and MPC solve frequency
- [MuJoCo `mj_jacSubtreeCom` documentation](https://mujoco.readthedocs.io/en/stable/APIreference/APItypes.html#mjtNum)
- [OSQP solver API: `update` vs `setup`](https://osqp.org/docs/interfaces/python.html)
- Bottleneck analysis: `stand_progressive.py:448-870` (main simulation loop)
