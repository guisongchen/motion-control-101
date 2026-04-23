# ADR 002: WBC / MPC Frequency Scheduling

## Status

Accepted

## Context

The MuJoCo simulation loop couples three independent rates:

- Physics integration step (`DT_SIM`): how often MuJoCo advances the state.
- WBC target frequency (`WBC_FREQ`): desired rate of the Whole-Body QP solver.
- MPC target frequency (`MPC_FREQ`): desired rate of the Centroidal MPC replanning.

The original configuration used:

```python
DT_SIM   = 1.0 / 240.0   # ~4.17 ms, 240 Hz physics
WBC_FREQ = 1000          # 1000 Hz target
MPC_FREQ = 20            # 20 Hz target
```

The scheduler computed integer skip periods with:

```python
wbc_period = max(1, int(1.0 / (WBC_FREQ * DT_SIM)))
```

Because `int(1.0 / (1000 * 1/240)) = int(0.24) = 0`, `max(1, 0)` clamped it to `1`. The WBC therefore ran **once per physics step**, i.e. at 240 Hz, while the config falsely claimed 1000 Hz. All PD gains (`Kp_c`, `Kd_c`, `POSTURE_KP`, etc.) were implicitly tuned for this hidden 240 Hz effective rate.

When `DT_SIM` was later raised to `0.001` (1000 Hz) to improve contact stability, the same formula produced `int(1.0 / (1000 * 0.001)) = 1`, and the WBC suddenly ran at a true 1000 Hz. The unchanged gains destabilized the robot.

Additionally, a velocity differentiation bug in the single-support WBC module hardcoded `DT_SIM` as the time delta:

```python
measured_cop_velocity = (cop - prev_cop) / DT_SIM
```

With `wbc_period = 4`, the actual interval between updates is `4 * DT_SIM`, so the computed velocity was 4× too large.

## Decision

1. **Decouple physics fidelity from control rate.**
   - Physics runs at 1000 Hz (`DT_SIM = 0.001`) for stable contact integration.
   - WBC runs at 250 Hz (`wbc_period = 4`).
   - MPC runs at 20 Hz (`mpc_period = 50`).

2. **Add explicit guardrails in the scheduler.**
   - Raise `ValueError` if `WBC_FREQ` or `MPC_FREQ` exceeds `1/DT_SIM`.
   - Use `round()` instead of `int()` to reduce discretization bias when converting a target frequency to an integer step period.

3. **Use the true control interval for numerical differentiation.**
   - Replace `DT_SIM` with `dt_wbc = wbc_period * DT_SIM` wherever the WBC computes finite-difference velocities (CoP velocity, support slip velocity).

4. **Express time-based throttles in seconds, not steps.**
   - Convert `last_wbc_warn_step` (hardcoded 60 steps) to `last_wbc_warn_time` (0.25 s) so warning frequency does not change with physics rate.

## Consequences

### Positive

- Contact dynamics are smoother because MuJoCo integrates at 1 kHz.
- WBC and MPC run at frequencies that match their original tuning.
- The scheduler is now explicit: it fails fast if a developer sets an impossible target frequency.
- `round()` yields fairer integer periods (e.g. `round(6.67) = 7` vs `int(6.67) = 6`).

### Negative / Trade-offs

- WBC does not run at the physics rate, so torques are held constant for 4 ms between QP solves. For this robot's centroidal dynamics, 250 Hz is sufficient; higher rates would require re-tuning gains and a faster QP solver.
- `MPC_FREQ` and `WBC_FREQ` must be chosen as clean divisors of `1/DT_SIM` to avoid jitter from `round()`.

## Alternatives Considered

### Alternative A: WBC at 1000 Hz

Run WBC every physics step (`wbc_period = 1`). Rejected because:
- OSQP Python `setup()` + `solve()` takes ~0.3–0.5 ms; at 1000 Hz the solver budget is tight.
- All PD gains and filter alphas would need re-tuning for the 4× faster discrete loop.
- The zeroed `Jc_dot` approximation in the slip constraint accumulates more error at higher control rates.

### Alternative B: Keep physics at 240 Hz

Keep `DT_SIM = 1/240` and accept the hidden 240 Hz WBC cap. Rejected because:
- 240 Hz is too coarse for reliable contact dynamics with a 23-DoF humanoid; foot slip and ground penetration increase.
- The hidden frequency cap is a latent bug that will surprise anyone who changes `DT_SIM` later.

### Alternative C: Custom substepping inside `mj_step()`

Implement a custom loop that calls `mj_step1()`, injects a new control, then `mj_step2()` at 1 kHz while running WBC at 250 Hz. Rejected because:
- It adds significant complexity to `RobotModel.step()`.
- The current codebase uses a simple `mj_step()` wrapper; substepping is overkill for this research prototype.

## References

- `config.py`: `DT_SIM`, `WBC_FREQ`, `MPC_FREQ`
- `main.py`: scheduler construction, `ValueError` guards
- `phases/single_support.py`: `dt_wbc = solvers.wbc_period * DT_SIM`
