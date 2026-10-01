---
type: Measurement
title: 'Solver convergence is a setting, and the spec reached for the wrong one'
description: 'Both iteration counts were exposed and measured: raising the biased count alone bought no stability, and the measurement cannot separate convergence from the bias:relax ratio, because four of its six variants varied both. Both counts stay at 1, and the balanced axis is open.'
tags: [physics, solver, performance, dwyer, negative-result]
generated: { by: claude-opus-5/claude-code, at: 2026-10-02T00:00:00Z }
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

The setting is now real, and it was measured. **Raising the velocity count
while holding the relax count at 1 bought no stability on a 240:1 load.** Worst
upward velocity went **12.83 v/s at `(1,1)` to 18.02 v/s at `(8,1)`**, and the
fixture breached the floor no later: **tick 55 at `(1,1)`, tick 55 at `(8,1)`**.
Both counts stay at **1**.

## The claim this page used to make, and why it was narrowed

It said "raising this count made a 240:1 load *less* stable", that "there is no
setting on this axis that makes this engine's loaded stack meaningfully better
behaved", and that "the question is closed for this engine". **All three were
wider than the experiment could carry, and are withdrawn in that form.**

Four of the six original variants — `(2,1)`, `(4,1)`, `(8,1)` against `(1,1)` —
**raise `velocity_iterations` while holding `relaxation_iterations` at 1**. The
relax pass's own job is removing the velocity the bias added. So those rows vary
the **bias:relax ratio**, not the velocity count alone, and **the bias:relax
ratio is an unexcluded explanation of every number in them.** The design did not
separate the two variables.

The mechanism, checkable in `solver::solve` and `solver::step_with`: `bias` is a
**velocity** target —

```rust
(BIAS * (separation + SLOP).min(0.0) * inv_h).max(-MAX_PUSH)
```

— and positions are **not integrated inside the velocity loop** (`integrate`
runs after the `for _ in 0..tuning.velocity_iterations` block, before the relax
block). So more biased sweeps converge toward *delivering more push-out
velocity* against the same separation. That predicts both measured columns at
once: penetration falling **and** upward velocity rising. One unbalanced lever
moving two columns the way a single mechanism predicts is exactly the case a
confounded design cannot rule on.

**What survives.** The headline stands: *the lever F7 reached for — the biased
count on its own — did not deliver stacking stability.* What changes is which
lever the data can speak about.

# The number

`solver::tests::what_the_iteration_counts_cost` in
`crates/bevox_physics/src/solver.rs`, `#[ignore]`d. Median of seven, both
fixtures timed every round with the variants interleaved round-robin in one
invocation, so a system hiccup lands on every variant equally. Re-run
**2026-10-02** with `(2,2)` added, so the balanced axis has two samples, and
with the stability scan corrected (below). Drift check — the first `(1,1)`
against the repeated one — came out at **5.0%**, against the test's own 15%
bound.

| tuning | load ms/tick | resting ms/tick | worst upward v/s | at tick | breach tick | resting penetration |
|---|---|---|---|---|---|---|
| **(1,1)** | **2.2348** | **2.8080** | **12.8272** | 11 | **55** | **0.0017** |
| (2,1) | 3.4292 | 3.4850 | 15.7513 | 36 | 88 | 0.0012 |
| (4,1) | 4.4079 | 4.4870 | 14.8053 | 32 | 67 | 0.0009 |
| (8,1) | 5.7311 | 6.8879 | **18.0217** | 31 | 55 | 0.0003 |
| (2,2) | 4.1202 | 4.0094 | 13.3643 | 6 | **105** | 0.0011 |
| (4,4) | 6.6001 | 6.3653 | 11.9548 | 5 | **123** | 0.0005 |
| (1,1) repeat | 2.1220 | 2.7429 | 12.8272 | 11 | 55 | 0.0017 |

`worst upward v/s`, `at tick`, `breach tick` and `resting penetration` are
deterministic given the tuning, which is why the first and last rows repeat them
exactly; they were identical across four consecutive runs. Only the two ms/tick
columns are wall-clock.

**The ms/tick columns have no warmup.** They are ticks 1-7 of fresh fixtures, so
the "resting ms/tick" column is **a stack still settling**, not a settled one —
`the_cost_of_sixteen_bodies`, in the same file, warms 300 ticks before timing
anything and this does not. The columns are good for comparing variants against
each other on identical state; neither is a resting-cost figure for the engine.

**And the published run is the quietest of four attempts.** The drift gate's
15% bound discarded nothing on these four, but they came out at 9.3%, 22.5%,
5.0% and 12.6% — one over the bound — and 5.0% is the one printed above. The
deterministic columns were bit-identical in all four, so only the timings are
cherry-picked, and they are cherry-picked for quiet rather than for a result.

## The stability metric, and what was wrong with it

`worst upward v/s` was originally the maximum of `velocity.y` over **200
unconditional ticks**. The fixture breaches the floor at tick 55, and
`step_with`'s first statement deletes any body whose world box falls below
`y = 0` —

```rust
bodies.retain(|b| world_box(b, 0.0).is_none_or(|(_, max)| max.y >= 0.0));
```

— so roughly three quarters of that window measured **a collapsing,
body-deleting scene** rather than a stack. The deterministic
non-monotonicity in the old numbers (12.83 → 15.75 → 14.81 → 18.02) is the
symptom: a convergence parameter should not make a monotone sequence wander.

The scan now **stops at the first tick any body is a voxel into the slab or is
removed** (`breached_the_slab`), takes its maximum only over that valid window,
and prints the peak's tick and the breach tick per variant. Every peak above
turned out to fall inside its valid window, so the five velocity figures are
unchanged — what is new is that the window they came from is now stated, and
that the breach tick is a stability number in its own right.

# Four readings

**1. On the unbalanced axis, nothing improves.** 12.83 → 15.75 → 14.81 → 18.02
on worst upward velocity, and 55 → 88 → 67 → 55 on how long the fixture stands,
as the biased count doubles with the relax count pinned at 1. `(8,1)` is
`(1,1)`'s breach tick exactly, for 2.6x the cost. **Neither column is monotone**,
which is the signature of a lever that is not the one in the mechanism. And
because these four rows vary the bias:relax ratio too, they cannot distinguish
"more convergence does not help" from "a bias:relax ratio above 1 hurts".

**1b. On the balanced axis, the fixture stands measurably longer.** `(2,2)` to
tick **105** and `(4,4)` to tick **123**, against `(1,1)`'s **55** — roughly
double and triple, monotone in the count, for 1.9x and 3.0x the tick. On worst
upward velocity the same two rows are 13.36 and 11.95 against 12.83, which is
flat. **Two samples is two samples**, and the fixture collapses at every setting
measured, so this is not a fix and the counts are not raised on it. But it is the
one place a convergence reading is not confounded, and it points the opposite way
from the headline this page originally carried.

**2. Resting penetration is the one real gain, and it is monotonic.**
0.0017 → 0.0012 → 0.0009 → 0.0003 as the velocity count rises. That is the
property worth holding, and it now has a gate:
`solver::tests::more_iterations_do_not_deepen_a_resting_contact` settles the
three-cube stack of `a_stack_of_three_stands_still` at eight velocity
iterations and asserts it sits no deeper than at `Tuning::default()`
(0.00035 against 0.00166). Deeper penetration under more work would be the
signature of a mis-scaled push-out bias, and nothing else in the suite looks at
penetration as a function of the count. **Three ten-thousandths of a voxel is
not worth 2.6x a tick on its own.**

(The gate and the measurement both read this as `(10.0 - y).abs()`. It was
`.max(0.0)`, which scores a body floating *above* its reference as a perfect
contact — the mis-scaled bias this is meant to watch for moves a body in either
direction.)

**3. The recorded 240:1 divergence did not reproduce.** See the section below;
it is its own problem.

## What this does *not* say

The measurement shows **that** the unbalanced variants did not help this
fixture. It does not show **why**, and it does not show that convergence is the
variable responsible — the design did not separate convergence from the
bias:relax ratio, so **the ratio is an unexcluded explanation of all four
unbalanced rows**. No impulse magnitudes were instrumented, no per-iteration
trace was taken, and nothing here identifies which term goes wrong.

The paragraph about `bias` being a velocity target, under the headline, is
**read off `solve` and `step_with` and marked as the prediction it is** — it
says what the design cannot exclude, not what the numbers proved. Any sentence
of the form "because the bias accumulates, therefore…" stated as a finding would
be an inference dressed as an observation —
[see the rule](an-inference-is-not-an-observation.md). **What actually
destabilises a 240:1 load is unmeasured and open.**

**To separate the two variables**, the experiment that is missing is a
bias:relax sweep at fixed total work — `(2,2)` against `(4,1)` and `(1,4)` at
comparable cost — plus a per-iteration trace of the push-out impulse. Neither
was run.

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

# Open defect: the 240:1 load collapses through the floor at tick 55

**This is a live bug, it is independent of the iteration counts, and it is not
fixed by anything above.** It is recorded here because it was found here.

**It has a test.** `solver::tests::the_240_to_1_load_collapses_through_the_floor`,
`#[ignore]`d, in `crates/bevox_physics/src/solver.rs`:

```
cargo test -p bevox_physics --lib the_240_to_1_load_collapses_through_the_floor -- --ignored --nocapture
```

It is a **characterisation of a known defect**, so it **passes today** and
asserts today's wrong behaviour: that the breach happens, inside tick 50-60,
with a downward velocity of at least 20 v/s. It is **expected to fail the day
the collapse is fixed**, and the fix is to invert it — assert the fixture is
still standing after 200 ticks and delete the breach numbers. Its doc comment
says so.

The previous version of this section named
`what_the_iteration_counts_cost` and a command that prints a timing table,
which says nothing about the collapse, and the trace that produced the numbers
was deleted before committing with its only record in `.superpowers/` — which is
**git-ignored**, so it does not survive a clone. That is fixed: the defect is now
reproducible from the repository.

The fixture is the shared `load_fixture` in `solver.rs`'s test module: on
`slab(64, 0..8)`, a `block(4,4,4)` cube at `y = 10.2`, a 4x1x4 plate at
`y = 12.2`, and a 4x1x4 heavy body at `y = 13.2` whose material density of
48,000 against the others' 200 gives it **exactly 240x the plate's mass at equal
volume** (checked against the engine's own `recompute`, asserted to 1e-3, not
hand arithmetic).

At `Tuning::default()`, measured 2026-10-02:

- worst **upward** velocity **12.83 v/s**, on the heavy body, at **tick 11**;
- at **tick 55** a body is a voxel and a half into the slab (lowest world box
  bottom **6.59**, against **7.5** at rest) and falling at **−22.08 v/s**, and
  the stack goes on through the floor.

(The 2026-10-01 write-up gave "−23.6 v/s" and "tick 9-10" from a trace sampled
differently. −22.08 at the breach tick and tick 11 for the peak are the numbers
the committed test reproduces.)

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
reproducible from a clone** — by
`the_240_to_1_load_collapses_through_the_floor`, the command at the top of this
section, and not by the measurement, which prints a timing table and says
nothing about the collapse.

No cause is offered — nothing was instrumented past body velocities and world
boxes. **Two hypotheses, marked as hypotheses**, both read off the code and
neither tested:

- **`MAX_PUSH` is a velocity, and it is mass-independent.** `solve` clamps the
  push-out bias at `MAX_PUSH = 20.0` voxels per second, the same number
  whichever body it is pushing. Across a 240:1 pair, the impulse needed to give
  the heavy body that velocity is enormous, and the equal and opposite reaction
  lands on the light plate. `[Likely]` — the clamp and its mass-independence are
  in `solve`, the consequence is not instrumented.
- **Contacts are reused across substeps, and the middle body is one voxel
  thick.** Contacts are detected once per tick and reused by all four
  `SUBSTEPS`, and the plate is `block(4, 1, 4, ..)` — the thinnest geometry that
  scheme can be asked to hold. `[Guessing]` — plausible from the structure, with
  no measurement separating it from the clamp above.

Both predict things that can be checked cheaply: the first by logging the
reaction impulse on the plate, the second by thickening the plate to two voxels
and re-running. Neither was run here.

# For the next person reaching for this lever

Both counts exist, are plumbed through `step_with`'s `Tuning`, are measured,
and are pinned at 1 by `solver::tests::the_solver_iteration_counts_are_one` —
whose failure message points back at this page, because the counts are an input
to `peak` and therefore to every `strength` in the material table.

**Closed:** "would raising the biased count alone fix our stacking?" The answer
is no, the numbers are in the table, and `(8,1)` is `(1,1)`'s breach tick for
2.6x the cost.

**Not closed, and this page previously said it was:**

- Whether *balanced* convergence helps. `(2,2)` and `(4,4)` hold the fixture up
  roughly twice and three times as long. Two samples, both expensive, both still
  collapsing — but the sign is the opposite of the headline.
- Whether the four unbalanced rows measured convergence at all, or the
  bias:relax ratio. **The design cannot tell them apart.**
- What makes an extreme mass ratio collapse. Two hypotheses above, neither
  tested.
- Why more convergence buys penetration depth and not stability.
