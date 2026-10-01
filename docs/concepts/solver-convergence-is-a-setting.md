---
type: Measurement
title: 'Solver convergence is a setting, and more of it is worse'
description: 'The velocity iteration count was exposed and measured on 2026-10-01: raising it made a 240:1 load less stable, not more, so both counts stay at 1 and F7 is resolved against its own premise.'
tags: [physics, solver, performance, dwyer, negative-result]
generated: { by: claude-opus-5/claude-code, at: 2026-10-01T00:00:00Z }
sources:
  - id: drift
    resource: /reference/dwyer-drift.md
    title: How far BEVOX has drifted from Dwyer
  - id: plan
    resource: /superpowers/plans/2026-10-01-bevox-solver-iteration-counts.md
    title: "The solver's iteration counts"
  - id: inference
    resource: /concepts/an-inference-is-not-an-observation.md
    title: An inference is not an observation
---

# The headline

**The lever the spec reached for is not the lever.** F7 said BEVOX ran one
velocity pass where Dwyer's solver exposes a count, that raising the count is
*"the standard lever for stacking stability"*, and that BEVOX had been running
*"at its least stable setting without knowing there was a setting."*

The setting is now real, and it was measured. **Raising the velocity count made
a heavily loaded stack less stable, not more.** Worst upward velocity on a 240:1
mass-ratio load went **12.83 v/s at one iteration to 18.02 v/s at eight**. Both
counts stay at **1**.

# The number

`solver::tests::what_the_iteration_counts_cost` in
`crates/bevox_physics/src/solver.rs`, `#[ignore]`d. Median of seven, both
fixtures timed every round with the six variants interleaved round-robin in one
invocation, so a system hiccup lands on every variant equally. Drift check —
the first `(1,1)` against the repeated one — came out at **0.3%**, against the
test's own 15% bound.

| tuning | load ms/tick | resting ms/tick | worst upward v/s | resting penetration |
|---|---|---|---|---|
| **(1,1)** | **2.0834** | **2.7393** | **12.8272** | **0.0017** |
| (2,1) | 3.2296 | 3.3751 | 15.7513 | 0.0012 |
| (4,1) | 4.2214 | 4.4555 | 14.8053 | 0.0009 |
| (8,1) | 5.7309 | 7.0288 | **18.0217** | 0.0003 |
| (4,4) | 6.7395 | 6.4823 | 11.9548 | 0.0005 |
| (1,1) repeat | 2.0905 | 2.7496 | 12.8272 | 0.0017 |

The two velocity columns are deterministic given the tuning, which is why the
first and last rows repeat them exactly. Only the two ms/tick columns are
wall-clock.

# Three readings

**1. Stability goes the wrong way.** 12.83 → 15.75 → 14.81 → 18.02 as the
velocity count doubles. The only variant that beats `(1,1)` at all is `(4,4)`,
by **0.9 v/s for 3.2x the tick cost** — which is not a result, it is inside the
noise of a different fixture and bought at triple price. There is no setting on
this axis that makes this engine's loaded stack meaningfully better behaved.

**2. Resting penetration is the one real gain, and it is monotonic.**
0.0017 → 0.0012 → 0.0009 → 0.0003 as the velocity count rises. That is the
property worth holding, and it now has a gate:
`solver::tests::more_iterations_do_not_deepen_a_resting_contact` settles the
three-cube stack of `a_stack_of_three_stands_still` at eight velocity
iterations and asserts it sits no deeper than at `Tuning::default()`
(0.00035 against 0.00166). Deeper penetration under more work would be the
signature of a mis-scaled push-out bias, and nothing else in the suite looks at
penetration as a function of the count. **Three ten-thousandths of a voxel is
not worth 2.8x a tick on its own.**

**3. The recorded 240:1 divergence did not reproduce.** See the section below;
it is its own problem.

## What this does *not* say

The measurement shows **that** raising the velocity count destabilised this
fixture. It does not show **why**. No impulse magnitudes were instrumented, no
per-iteration trace was taken, and nothing here identifies which term goes
wrong. Any sentence of the form "because the bias accumulates…" would be an
inference dressed as an observation —
[see the rule](an-inference-is-not-an-observation.md). **What actually
destabilises a 240:1 load is unmeasured and open.**

# What the loops cost when they are off

Nothing, exactly. `bevox_physics::Tuning` defaults to
`(velocity_iterations: 1, relaxation_iterations: 1)` from the two `pub const`s,
and the loops added to `solver::step_with` — the `for _ in
0..tuning.velocity_iterations` around the biased pass and the `for _ in
0..tuning.relaxation_iterations` around the relax pass — run exactly once at
that default. `solver::tests::a_tick_is_unchanged_by_the_default_tuning` runs
the three-body stack 2000 ticks through `step` and through
`step_with(.., Tuning::default())` and asserts position, orientation, velocity
and angular momentum **equal**, not close. The exposure was free.

# The biased pass is not what holds a stack up

A fact about this solver nobody had written down, found by the Task 1
deliberate break (`VELOCITY_ITERATIONS` edited to `0`):

**Zeroing the velocity count does not drop a body through the floor.**
`a_stack_of_three_stands_still` stayed green. What failed was **eleven tests,
mostly joints** — all six `joint::tests`, both `sleep::tests` that involve
waking, and `a_pendulum_keeps_its_pivot_and_gains_no_energy`,
`a_lone_voxel_stops_rolling_and_sleeps`, `a_knocked_chain_stays_together`.

The code that makes non-penetration survive: the relax loop in `step_with`
calls the same `solve`, `solve_friction` and `solve_rolling` as the biased loop,
with `bias = false`. Only the push-out correction is missing, not the contact
resolution. The joints are the asymmetric half — the biased pass runs
`solve_joints` over the whole joint slice, while the relax pass runs it over
`joints[..scene_joints]` only, excluding the grab.

*(Which tests fail is observed. That the relax pass's unbiased contact solving
is what kept the stack standing is read off the call sites above, not measured —
no impulses were instrumented.)*

And one consequence for anything downstream: `peak[at]`, the accumulated normal
impulse the fracture threshold reads, is taken **inside both loops** (`*peak =
peak.max(impulse.normal)` in each). So the counts are an input to every
`strength` in the material table. Their staying at 1 is what keeps that
calibration valid.

# Open defect: the 240:1 load collapses through the floor by tick 55

**This is a live bug, it is independent of the iteration counts, and it is not
fixed by anything above.** It is recorded here because it was found here.

The fixture is the load fixture inside
`solver::tests::what_the_iteration_counts_cost`: on `slab(64, 0..8)`, a
`block(4,4,4)` cube at `y = 10.2`, a 4x1x4 plate at `y = 12.2`, and a 4x1x4
heavy body at `y = 13.2` whose material density of 48,000 against the others'
200 gives it **exactly 240x the plate's mass at equal volume** (checked against
the engine's own `recompute`, asserted to 1e-3, not hand arithmetic).

At `Tuning::default()`, over the first 60 ticks:

- worst **upward** velocity **12.83 v/s**, on the heavy body, around **tick
  9-10**;
- by **tick ~55** the heavy body is moving **downward at about −23.6 v/s**, and
  the stack goes on through the floor.

**This does not match the failure recorded on 2026-09-28** ("23 v/s upward by
tick 33") in magnitude, direction or timing. At tick 33 the rebuilt fixture is
settling, at roughly `(−0.52, −1.75, −1.60)` v/s. Nothing was adjusted to chase
the recorded number; a fixture tuned until it reproduces a remembered symptom
proves nothing.

So one of three things is true and **which one is unknown**: the recorded bug
reproduces only under a fixture parameter the brief left unspecified (the heavy
body's shape is the obvious candidate — the plate's own footprint was used), or
it no longer reproduces because something else changed it, or this collapse is a
different bug. Either way, **the collapse itself is real, deterministic, and
runnable today**:

```
cargo test -p bevox_physics --lib what_the_iteration_counts_cost -- --ignored --nocapture
```

No cause is offered. Nothing was instrumented past body velocities.

# For the next person reaching for this lever

Both counts exist, are plumbed through `step_with`'s `Tuning`, and are
measured. The question "would more velocity iterations fix our stacking?" is
**closed for this engine** — the answer is no, and the number is in the table
above. The questions that are still open are what makes an extreme mass ratio
collapse, and why more convergence here buys penetration depth and not
stability.
