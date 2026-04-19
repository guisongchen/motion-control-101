# MuJoCo load-shift tuning notes

## Context

During the MuJoCo single-leg-stand migration, `LOAD_SHIFT` was changed from a timer-only phase into an active control phase.

The old behavior was:

- enter `LOAD_SHIFT`
- wait for `LOAD_SHIFT_TIME`
- enter `PRE_LIFTOFF`

That was structurally clean, but physically weak: the controller did not actually move load onto the intended support foot before attempting liftoff.

The new behavior makes `LOAD_SHIFT` state-driven:

- actively lean toward the support foot
- measure support-foot loading and CoM migration
- block phase promotion unless readiness conditions are satisfied

## Why new constants were added

Several new constants were added to `week10_single_leg_stand/config.py`. They may look like magic numbers, but each one makes an explicit control assumption tunable instead of burying it inside `main.py`.

### 1. Load-shift actuation limits

- `LOAD_SHIFT_ROLL_DELTA`
- `PRE_LIFTOFF_EXTRA_ROLL_DELTA`
- `PRE_LIFTOFF_EXTRA_SWING_PROGRESS`
- `PRE_LIFTOFF_SWING_KP_SCALE`
- `PRE_LIFTOFF_SWING_KD_SCALE`

These tune how strongly the controller biases the body toward the support foot and how aggressively `PRE_LIFTOFF` relaxes the swing leg.

Reason:

- the first active-load-shift attempt used sagittal leg-length changes and did not transfer load effectively
- quick probing in MuJoCo showed that coordinated hip-roll / ankle-roll leaning loads the support foot much more directly
- too much lean can unload the swing foot, but also create slip, so the commanded lean needs an explicit cap

### 2. Support-loading thresholds

- `LOAD_SHIFT_SUPPORT_RATIO`
- `PRE_LIFTOFF_SUPPORT_RATIO`

These define how much of the measured total vertical foot load must be carried by the intended support foot before advancing phases.

Reason:

- a timed transition can enter `PRE_LIFTOFF` or `SINGLE_SUPPORT` while the robot is still close to 50/50 loading
- single-support is not credible unless the support foot already carries a clear majority of the load

### 3. CoM-shift thresholds

- `LOAD_SHIFT_COM_RATIO`
- `PRE_LIFTOFF_COM_RATIO`

These define how far the horizontal CoM must move toward the support side, normalized by the stance width.

Reason:

- contact force alone can be noisy
- requiring CoM migration as well makes the transition less dependent on a transient contact-force fluctuation

### 4. Swing unloading threshold

- `PRE_LIFTOFF_SWING_FORCE_MAX`

This limits how much normal force can remain on the swing foot before entering single-support.

Reason:

- liftoff should only be attempted when the swing foot is already mostly unloaded
- demanding perfect zero is unrealistic in MuJoCo contact, so this threshold is intentionally nonzero

### 5. Motion bound

- `COM_VEL_READY_THRESH`

This prevents phase promotion while the body is still moving too quickly.

Reason:

- a phase transition during a large transient often promotes the controller into a worse condition, even if force thresholds are briefly satisfied

### 6. Dwell / hysteresis

- `LOAD_SHIFT_READY_TIME`
- `PRE_LIFTOFF_READY_TIME`

These require readiness conditions to hold for a short time before phase promotion.

Reason:

- avoids switching phases on a single noisy frame
- adds hysteresis without hard-coding longer phase durations

## Why the current values were chosen

The values were chosen from observed MuJoCo behavior, not from theory alone.

### Load-shift and pre-liftoff targets

- `LOAD_SHIFT_ROLL_DELTA = 0.05`
- `PRE_LIFTOFF_EXTRA_ROLL_DELTA = 0.002`
- `PRE_LIFTOFF_EXTRA_SWING_PROGRESS = 0.05`
- `PRE_LIFTOFF_SWING_KP_SCALE = 0.75`
- `PRE_LIFTOFF_SWING_KD_SCALE = 0.85`

These are intentionally gentle values:

- larger roll or swing-leg relaxation commands transferred more load, but also caused major swing-foot slip and runaway lean
- smaller values preserved stability but failed to unload the swing foot at all

### Force ratio targets

- `LOAD_SHIFT_SUPPORT_RATIO = 0.52`
- `PRE_LIFTOFF_SUPPORT_RATIO = 0.56`

These are above neutral loading, but below the clearly unstable over-lean regime seen during aggressive tests.

### CoM shift targets

- `LOAD_SHIFT_COM_RATIO = 0.16`
- `PRE_LIFTOFF_COM_RATIO = 0.20`

These ask for visible support-side migration without demanding an extreme lateral shift.

### Swing-foot force target

- `PRE_LIFTOFF_SWING_FORCE_MAX = 110.0`

This means "mostly unloaded" rather than "perfectly detached".

### Velocity and dwell

- `COM_VEL_READY_THRESH = 0.20`
- `LOAD_SHIFT_READY_TIME = 0.08`
- `PRE_LIFTOFF_READY_TIME = 0.10`
- `PRE_LIFTOFF_TIME = 0.50`

These keep promotion conservative without making the controller wait on fixed long timers.

## What changed in behavior

With the new controller structure:

- `LOAD_SHIFT` now actively transfers weight toward the support foot
- readiness is measured with support ratio, CoM shift, velocity, and slip
- the controller backs off load-shift lean when swing-foot slip grows
- phase progression is now blocked when the robot is not physically ready

This is an important change in meaning:

- old `LOAD_SHIFT`: a scheduled pause
- new `LOAD_SHIFT`: a controlled preparation phase

## Current status

The current tuning achieves the following on the default 3-second run:

- stable double-support standing remains intact
- the default 3-second run now reaches `PRE_LIFTOFF` at about `t = 1.93 s`
- by the end of the default run, swing-foot force drops from about `173 N` to about `125 N`
- max foot slip stays at about `3.59 mm`, below the `5 mm` threshold

That is preferable to the previous behavior, which either stayed stuck in `LOAD_SHIFT` forever or advanced into unstable liftoff based only on elapsed time.

## Important caveat

These constants should be treated as **explicit policy parameters**, not final tuned truth.

They were introduced to make the phase logic legible and tunable while stabilizing the MuJoCo migration. A later cleanup should likely group them into structured phase-specific configs such as:

- `load_shift_controller`
- `load_shift_readiness`
- `pre_liftoff_readiness`

That would express intent more clearly than a flat list of scalars in `config.py`.
