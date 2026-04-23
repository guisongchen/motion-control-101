# Single-support assumption verification

## Purpose

This note records the assumption checks we ran after sustained `SINGLE_SUPPORT` tuning stalled. The goal was to separate three different questions that had been mixed together:

1. Can the MuJoCo model hold a one-leg pose at all?
2. Is the current contact / physics setup compatible with one-leg support?
3. Is the current `MPC + WBC + phase machine` the right controller to achieve stable, realistic single support?

## Summary

The task is **not ill-formed in principle**. We now have evidence that:

- a clean one-leg support geometry exists in MuJoCo,
- pure posture-hold control is not enough to maintain it,
- WBC can keep the robot upright in direct one-leg support,
- but the main remaining failure is **large support-foot slip**, not immediate falling.

That means the real problem is now better described as a **no-slip single-support control** problem, not a generic “don’t fall over” problem.

## Verification sequence

### 1. Direct-spawn one-leg posture-only test

We removed the transition logic entirely and spawned the robot directly into hand-designed one-leg poses:

- right foot as support foot,
- left foot tucked and clear of the ground,
- posture-hold torque only,
- no phase machine,
- no MPC,
- no WBC.

We swept:

- one-leg pose geometry,
- base height and lateral shift,
- roll lean,
- posture PD gains,
- damping,
- friction.

### Result

**Failed.**

- Best posture-only direct-spawn runs lasted only about **0.59–0.97 s**.
- Failure mode was large support-foot slip followed by collapse.

Conclusion:

- **Pure pose-hold control cannot solve single support** in this setup.

## 2. Spawn-geometry / contact-feasibility check

We then checked whether a direct one-leg spawn with clean contact geometry even exists.

### Result

**Passed.**

We found direct-spawn poses with:

- **support-foot contact only**,
- **zero swing-foot contact**,
- and large vertical clearance on the swing foot.

Example of a favorable geometry:

- base height around `0.72`,
- small base roll toward the support foot,
- modest support-leg roll loading,
- tucked swing leg.

Conclusion:

- The task is **not geometrically impossible** in MuJoCo.
- The model can represent a one-leg support state cleanly.

## 3. WBC feasibility on the same direct one-leg pose

We then tested the same direct one-leg spawn with WBC enabled.

### Result

**Passed in a weak sense.**

Representative outcome:

- posture-only: survives **< 1 s**
- WBC blend: survives **5.0 s**
- WBC full: survives **5.0 s**

WBC also strongly reduced or eliminated swing-foot contact in the direct-spawn benchmark.

Conclusion:

- **One-leg support is feasible with the current model when using WBC.**
- The earlier failures were not proof that single support itself is impossible.

## 4. MPC usefulness on the direct one-leg pose

We compared:

- WBC with fixed support-force reference,
- WBC + MPC force updates.

### Result

**No meaningful improvement from MPC.**

Representative comparison:

- `wbc-fixed`: survived `5.0 s`, max slip about `196 mm`, max swing-foot force about `47.7 N`
- `wbc-mpc`: survived `5.0 s`, max slip about `202 mm`, max swing-foot force about `49.0 N`

Conclusion:

- MPC is **not currently the lever** that unlocks the task.
- The bottleneck is elsewhere.

## 5. Default-physics direct one-leg check

We also re-ran the direct one-leg WBC hold under the repo’s default damping / friction settings rather than the favorable test settings.

### Result

**Still survives, but with very large slip.**

Representative outcome:

- survived `5.0 s`
- max support-foot slip about **467 mm**
- swing-foot contact stayed suppressed

Conclusion:

- The robot can be kept upright on one leg,
- but the support foot is skating heavily,
- so “survival” is not a sufficient success metric.

## What these checks prove

### Assumption 1: “The model can statically hold a one-leg pose.”

**Partially true.**

- Not with posture-only control.
- Yes with WBC-assisted control.

### Assumption 2: “The MuJoCo contact/model setup supports one-leg stance.”

**Yes, but weakly.**

- Clean single-support geometry exists.
- The bigger issue is not contact existence, but contact quality under load: the support foot slips too much.

### Assumption 3: “The current integrated controller is the right path.”

**Only partly.**

- WBC is clearly useful.
- MPC does not currently add value.
- The transition controller is no longer the main unknown.
- The real missing piece is explicit **no-slip support control**.

## “Don’t fall over” vs “no-slip single support”

These are not the same task.

### Don’t fall over

This only requires:

- the robot stays upright,
- the base does not collapse,
- some support remains under it for the test duration.

This can still happen while the support foot slides a long distance.

### No-slip single support

This requires:

- upright survival,
- swing foot unloaded / clear,
- support foot approximately fixed,
- support wrench staying inside a realistic friction-limited region.

This is the stricter and more physically meaningful target.

## Recommended next step

The next step should **not** be more transition tuning.

Instead, build a standalone direct-spawn benchmark focused on the real remaining problem:

## Next-step plan: direct no-slip single-support benchmark

### 1. Create a direct single-support test mode

Add a dedicated benchmark that:

- spawns directly into the validated one-leg pose,
- bypasses `LOAD_SHIFT`, `PRE_LIFTOFF`, and transition logic,
- runs a fixed-duration single-support hold.

### 2. Make slip the primary metric

Track at least:

- support-foot slip,
- swing-foot normal force,
- support-foot tangential force,
- friction ratio `sqrt(fx^2 + fy^2) / fz`,
- support wrench / CoP drift if available.

Success should be defined in terms of **both** survival and low slip.

### 3. Retune WBC around contact realism

Keep WBC, because it clearly helps, but retune it for no-slip behavior:

- reduce aggressive horizontal wrench demand,
- increase penalties against tangential force,
- favor support-leg stabilization over motion generation,
- clamp or regularize support-torque blending more tightly.

### 4. Re-evaluate MPC only after no-slip WBC works

Once direct one-leg WBC can hold with acceptably low slip:

- compare WBC-only vs WBC+MPC again,
- only keep MPC if it improves the no-slip benchmark.

### 5. Reconnect the transition later

Only after direct no-slip single support works should the phase machine be reconnected.

At that point the transition problem becomes:

- enter a known-good single-support mode,
- rather than trying to invent that mode during the transition itself.

## Practical conclusion

The project is no longer blocked on “is one-leg support possible?”

We now know:

- **yes, one-leg support is possible,**
- **yes, WBC helps materially,**
- **no, MPC is not currently the key unlock,**
- and the actual engineering target is:

> **stable single support with low support-foot slip**

That should be the definition of the next iteration.
