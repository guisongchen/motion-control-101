# ADR 001: Phase-driven control flow for MuJoCo single-support standing

## Status

Accepted

## Context

The `mpc_and_wbc/week10_single_leg_stand` experiment started as a direct translation of a PyBullet control loop into MuJoCo. The original loop used a small number of booleans (`use_mpc_wbc`, lift timing checks, and fallback branches) to coordinate all stages of the task inside one large `while` loop.

That structure made the experiment hard to reason about for three reasons:
- the initial double-support standing phase was not enforced as a prerequisite for later phases
- transition logic, fallback logic, swing-leg targeting, and single-support control were interleaved in one block
- failures in single-support control were hard to distinguish from failures in initialization or double-support standing

After stabilizing the MuJoCo double-support stage, the next requirement was to refactor the runtime so:
- double-support validation always happens first
- later phases only run after earlier readiness checks pass
- single-support failures are isolated to a specific phase instead of being hidden inside mixed control flow

## Decision

We refactor the experiment loop into an explicit phase/state machine with helper functions for phase transitions, posture targets, support selection, and safe torque generation.

### 1. Introduce an explicit control phase enum

**Decision:** Replace boolean-driven flow with a `ControlPhase` enum and a `PhaseState` data class.

**Rationale:**
- makes task progression explicit instead of implicit
- gives each phase a clear entry point, duration, and readiness logic
- keeps persistent phase-specific data together, such as locked support foot, filtered support point, and fallback torque memory

**Design:**
```python
class ControlPhase(Enum):
    INIT_SETTLE = auto()
    DOUBLE_SUPPORT_HOLD = auto()
    LOAD_SHIFT = auto()
    PRE_LIFTOFF = auto()
    SINGLE_SUPPORT = auto()


@dataclass
class PhaseState:
    phase: ControlPhase
    phase_start_time: float
    ready_since: Optional[float]
    locked_support_foot_link: int
    filtered_support_point: np.ndarray
    last_valid_support_tau: Optional[np.ndarray]
    last_wbc_warn_step: int
```

**Location:** `mpc_and_wbc/week10_single_leg_stand/main.py`

### 2. Make double-support validation mandatory

**Decision:** Require the controller to pass through `INIT_SETTLE` and `DOUBLE_SUPPORT_HOLD` before any load shift or lift-off logic can run.

**Rationale:**
- prevents single-support debugging from masking basic standing failures
- enforces a stable two-foot stance as a precondition for the task
- makes double-support-only testing a first-class path instead of a manual hack

**Design:**
```python
if phase_state.phase == ControlPhase.INIT_SETTLE:
    if elapsed >= INIT_SETTLE_TIME:
        transition_phase(phase_state, ControlPhase.DOUBLE_SUPPORT_HOLD, t)

if phase_state.phase == ControlPhase.DOUBLE_SUPPORT_HOLD:
    both_feet_ready = all(force >= DOUBLE_SUPPORT_MIN_FORCE for force in double_support_forces)
    stable_slip = max_double_support_slip <= SLIP_THRESH
```

**Location:** `mpc_and_wbc/week10_single_leg_stand/main.py`, `mpc_and_wbc/week10_single_leg_stand/config.py`

### 3. Separate transition phases from single-support control

**Decision:** Insert `LOAD_SHIFT` and `PRE_LIFTOFF` phases between double-support hold and full single-support WBC.

**Rationale:**
- avoids a single timed jump from two-foot posture holding to one-foot MPC/WBC
- creates a natural place to shift load to the intended support foot
- allows future work to add stricter readiness gates without restructuring the whole loop again

**Design:**
```python
INIT_SETTLE -> DOUBLE_SUPPORT_HOLD -> LOAD_SHIFT -> PRE_LIFTOFF -> SINGLE_SUPPORT
```

**Location:** `mpc_and_wbc/week10_single_leg_stand/main.py`

### 4. Factor common control logic into helpers

**Decision:** Extract helper functions for support-foot lookup, swing target generation, safe torque generation, and phase transitions.

**Rationale:**
- reduces the size and nesting of `main()`
- keeps the control loop readable as `estimate -> update phase -> solve/apply -> step`
- makes future tuning easier because phase-independent logic lives in one place

**Design:**
```python
def update_phase_machine(...): ...
def build_safe_targets(...): ...
def compute_safe_tau(...): ...
def choose_support_foot(...): ...
```

**Location:** `mpc_and_wbc/week10_single_leg_stand/main.py`

### 5. Move phase thresholds into configuration

**Decision:** Store phase durations and readiness thresholds in `config.py` instead of hardcoding them inside the loop.

**Rationale:**
- phase timing is part of experiment configuration, not loop structure
- allows tuning without editing state-machine logic
- keeps the phase model inspectable from one file

**Configuration:**
```python
INIT_SETTLE_TIME = 0.20
DOUBLE_SUPPORT_READY_TIME = 0.60
LOAD_SHIFT_TIME = 0.20
PRE_LIFTOFF_TIME = 0.20
DOUBLE_SUPPORT_MIN_FORCE = 120.0
PRE_LIFTOFF_SWING_PROGRESS = 0.25
```

**Location:** `mpc_and_wbc/week10_single_leg_stand/config.py`

## Consequences

### Positive

- **Clear sequencing**: double-support standing is always tested before single-support control.
- **Better debuggability**: logs now identify which phase failed, instead of mixing all stages together.
- **Safer extension path**: future load-transfer and readiness logic can be added to dedicated phases.
- **Cleaner `main()`**: common logic is extracted into helpers instead of repeated in-line branches.

### Negative

- **More control state**: the phase machine introduces more bookkeeping than a simple timed switch.
- **More configuration values**: phase timing and readiness thresholds add another set of parameters to tune.
- **Still not sufficient for success**: the refactor improves structure and observability, but does not by itself solve the remaining single-support stability problem.

## Alternatives Considered

### Alternative 1: Keep the boolean-driven loop and add more conditionals

Continue extending the old `use_mpc_wbc` / timed-switch logic with additional branches for validation and fallback.

**Rejected:**
- would make `main.py` even harder to read and maintain
- would keep phase prerequisites implicit instead of enforced
- would make debugging phase-specific failures unnecessarily difficult

### Alternative 2: Split each stage into separate scripts

Create one script for double-support standing, another for load shift, and another for single-support MPC/WBC.

**Rejected:**
- duplicates setup, logging, and estimator/controller wiring
- makes it harder to validate end-to-end progression in one runtime
- fragments the experiment instead of improving the structure of the existing task

### Alternative 3: Build a generic workflow engine first

Introduce a reusable experiment framework with pluggable stage objects and callbacks.

**Rejected:**
- too heavy for the current scope
- adds abstraction before the stage boundaries and runtime needs are fully stable
- risks solving a generic problem before the task-specific phase model is proven

## Implementation Notes

### Dependencies

```toml
# No new dependencies
```

### Testing Strategy

```bash
uv run python -m py_compile mpc_and_wbc/week10_single_leg_stand/*.py

# Double-support-only path
MPLBACKEND=Agg uv run python - <<'PY'
import main as sim
sim.PRE_LIFTOFF_TIME = 999.0
sim.LOAD_SHIFT_TIME = 999.0
sim.plot_com_tracking = lambda *args, **kwargs: None
sim.plot_contact_force = lambda *args, **kwargs: None
sim.plot_torques = lambda *args, **kwargs: None
sim.main()
PY

# Normal phased run
MPLBACKEND=Agg uv run python - <<'PY'
import main as sim
sim.plot_com_tracking = lambda *args, **kwargs: None
sim.plot_contact_force = lambda *args, **kwargs: None
sim.plot_torques = lambda *args, **kwargs: None
sim.main()
PY
```

### Migration notes

- This ADR documents a structural refactor of `main.py`, not a change to the MPC or WBC formulations.
- The stable MuJoCo double-support standing setup remains the baseline prerequisite for all later phases.

## References

- `mpc_and_wbc/week10_single_leg_stand/main.py`
- `mpc_and_wbc/week10_single_leg_stand/config.py`
- `mpc_and_wbc/doc/single_leg_stand_debug_summary.md`
