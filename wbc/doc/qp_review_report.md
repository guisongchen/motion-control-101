# QP-based IK Review Report

## Issue
The `QP(1, 1)` curve in `output/elbow_angle.png` showed a significant elbow-angle deviation compared to the other solvers, suggesting a possible bug in `solve_ik_qp`.

## Investigation

### 1. Mathematical correctness
The QP formulation in `wbc/arm_model.py` is correct:
- Objective: `min 0.5 * qdot^T * H * qdot + f^T * qdot`
- `H = J_combined^T * W * J_combined + lambda^2 * I`
- `f = -J_combined^T * W * xdot_combined`

This is mathematically equivalent to the weighted damped least-squares (DLS) solver.

### 2. Solver tolerance bug
OSQP was called with default tolerances (`eps_abs=1e-3`, `eps_rel=1e-3`). This produced per-step errors of ~0.03 rad/s. Over 400 integration steps the numerical drift compounded and the trajectory diverged from the DLS solution.

**Fix applied:** tightened OSQP tolerances to `1e-6` in `wbc/arm_model.py`.

```python
prob.setup(
    ...
    eps_abs=1e-6,
    eps_rel=1e-6,
)
```

### 3. Root cause of the visual deviation
Even with the tolerance fix, the elbow plot still deviates because `QP(1, 1)` is **actively hitting the velocity limits** defined in `SimConfig` (`qdot_min = -5.0`, `qdot_max = 5.0`).


| Solver | mean tracking error | mean elbow error | peak |qdot| |
|--------|---------------------|------------------|-------------|
| Nullspace IK | 0.0827 | 0.0000 | 29.97 |
| DLS(100, 1) | 0.0848 | 0.0051 | 14.01 |
| DLS(10, 10) | 0.0828 | 0.0003 | 14.16 |
| **QP(1, 1)** | **0.0868** | **0.0126** | **5.00** |
| QP(10, 10) | 0.0877 | 0.0164 | 5.00 |

`QP(1, 1)` caps peak velocity at exactly `5.00` rad/s. The unconstrained solvers (nullspace, DLS) ignore those bounds and reach `14-30` rad/s, allowing better elbow tracking. Because the QP clips velocities, the arm lags behind the reference and the elbow angle drifts up to ~0.54 rad before slowly recovering.

This was verified by running the QP with relaxed limits (`±20`) and tight tolerances — it matched the DLS solution to within `1e-9`.

## Conclusion
- `solve_ik_qp` was mathematically correct but suffered from loose OSQP tolerances. **Fixed.**
- The remaining elbow-angle deviation in the plot is an **expected consequence of the velocity constraints**, not a bug. If tighter elbow tracking is desired, raise `qdot_min`/`qdot_max` in `SimConfig` or accept the trade-off that comes with constrained IK.
