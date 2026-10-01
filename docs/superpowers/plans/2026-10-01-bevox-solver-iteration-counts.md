---
type: Implementation Plan
title: "The solver's iteration counts"
description: "Turn the solver's one biased pass and one relax pass per substep into counted loops, bit-identical at one, then measure what raising them costs a tick and what they do to the 240:1 divergence."
tags: [physics, solver, convergence, dwyer, measurement]
generated: { by: claude-opus-5/claude-code, at: 2026-10-01T00:00:00Z }
sources:
  - id: spec
    resource: /superpowers/specs/2026-09-30-bevox-fracture-and-detection-design.md
    title: 'Fracture and detection, as he built them'
  - id: breaks
    resource: /concepts/deliberate-breaks.md
    title: Every gate is proven by a deliberate break
---

# The solver's iteration counts — implementation plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Give the solver the two iteration counts Dwyer's exposes, measure them,
and set them on the measurement — so the accumulated normal impulse that Part 2
thresholds fracture on is produced by a solver whose convergence is a decision.

**Architecture:** `solver::step` runs, per substep, exactly one biased pass and
one unbiased relax pass, neither a loop. Both become loops over a new `Tuning`
value carried as a parameter — the idiom `Air` already set in this crate, where
what a test must vary is a parameter rather than a constant. `step` keeps its
present signature and delegates to a new `step_with`, so none of its twelve call
sites change. Defaults of 1 make the first commit bit-identical.

**Tech Stack:** Rust 2024, no new dependencies. `cargo test -p bevox_physics`.

**Spec:** [Fracture and detection, as he built them](../specs/2026-09-30-bevox-fracture-and-detection-design.md) — **Part 1 only.** Parts 2 and 3 are separate plans.

## Global Constraints

- `step`'s existing signature does not change. Twelve call sites, nine of them
  tests, and churning them would bury the one change that matters.
- Defaults are `VELOCITY_ITERATIONS = 1` and `RELAXATION_ITERATIONS = 1`, so
  Task 1 is bit-identical to `dd9bf8d`. Task 3 is the only task allowed to change
  them, and only on a measurement.
- **A new `pub const` at column zero in `crates/bevox_physics/src/lib.rs` fails
  `the_maps_name_every_constant`** until `docs/map/physics-constants.md` lists it.
  That gate is in `crates/bevox/tests/map.rs`; run it in the same task.
- Every perf number is median-of-N, interleaved **A/B/A in one `cargo test`
  invocation**, never against a number from an earlier run.
- Every gate is proven by a deliberate break, recorded in the task's Breaks line.

## Review Focus

Five things Part 1 can disturb that no task above tests by name. Each is pinned
to the task that owns the code.

1. **Warm starting carries the converged impulse into the next tick.** `b.warm`
   stores the final accumulated impulse; more iterations make it larger, so the
   next tick's warm start kicks harder. Pinned in Task 1.
2. **Bodies sleep sooner.** `SLEEP_SPEED` is 0.05 voxels a second measured on
   residual velocity, which more iterations reduce. `a_stack_sleeps_without_creeping`
   is the gate nearest the edge. Pinned in Task 1.
3. **The bounce pass reads impulses that converged differently.**
   `apply_restitution` runs `RESTITUTION_SWEEPS` after the substeps, on
   `approach` and the solved state. Pinned in Task 1.
4. **The grab approaches its cap more completely per substep.** `clamp_add`
   limits the *accumulated* impulse to `max_force / inv_h`, so looping cannot
   exceed the cap — but a grab that previously under-delivered now reaches it.
   `a_grab_lifts_a_light_body_but_not_a_heavy_one` is calibrated on
   `GRAB_MAX_FORCE * DT / mass`. Pinned in Task 1.
5. **A large count must not make a tick unbounded.** The counts multiply the
   hottest loop in the physics; `BUDGET` is 20,000 contacts. Pinned in Task 2.

---

### Task 1: `Tuning`, the two loops, and bit-identity at one

**Files:**
- Modify: `crates/bevox_physics/src/lib.rs` — add `VELOCITY_ITERATIONS`, `RELAXATION_ITERATIONS`, `Tuning`
- Modify: `crates/bevox_physics/src/solver.rs:55-180` — `step_with`, `step` delegates, the two blocks become loops
- Modify: `docs/map/physics-constants.md` — the two constants, in the "How a contact behaves" table beside `SUBSTEPS`
- Test: `crates/bevox_physics/src/solver.rs` (the existing `#[cfg(test)] mod` at the foot)

**Interfaces:**
- Consumes: nothing.
- Produces:
  - `bevox_physics::VELOCITY_ITERATIONS: u32` = 1, `bevox_physics::RELAXATION_ITERATIONS: u32` = 1
  - `bevox_physics::Tuning { pub velocity_iterations: u32, pub relaxation_iterations: u32 }`, `#[derive(Clone, Copy, Debug, PartialEq)]`, with `impl Default` reading the two constants
  - `solver::step_with(bodies: &mut Vec<Body>, tree: &Contree, field: &DistanceField, materials: &MaterialTable, air: Air, dt: f32, grab: Option<&mut Joint>, joints: &mut [Joint], tuning: Tuning) -> StepOutcome`
  - `solver::step(..same eight..) -> StepOutcome`, unchanged, now `step_with(.., Tuning::default())`

- [ ] **Step 1: Write the failing bit-identity test**

In `solver.rs`'s test module. `a_tick_is_unchanged_by_the_default_tuning` runs the
three-body stack of `a_stack_of_three_stands_still` for 2000 ticks twice — once
through `step`, once through `step_with(.., Tuning::default())` — and asserts
every body's `position`, `orientation`, `velocity` and `angular_momentum` are
equal **bit for bit** (`==`, not a tolerance; this is the one gate in the file
entitled to exact equality besides momentum).

- [ ] **Step 2: Run it to verify it fails**

Run: `cargo test -p bevox_physics a_tick_is_unchanged_by_the_default_tuning`
Expected: FAIL to compile — `step_with` and `Tuning` do not exist.

- [ ] **Step 3: Add the constants and `Tuning` to `lib.rs`**

Beside `SUBSTEPS`, with doc comments saying what each loop is for and that both
are 1 until Task 3's measurement says otherwise.

- [ ] **Step 4: Add `step_with` in `solver.rs` and make `step` delegate**

Move the present body of `step` into `step_with`, add the `tuning` parameter, and
leave `step` as a one-line call with `Tuning::default()`. No behaviour change yet.

- [ ] **Step 5: Wrap the two passes in their loops**

Inside `for _ in 0..SUBSTEPS`, at `solver.rs:155-163` wrap the biased pass —
`solve_joints(.., true)` and the contact/friction/rolling loop that updates
`peak` — in `for _ in 0..tuning.velocity_iterations`. At `solver.rs:172-181` wrap
the unbiased pass the same way in `tuning.relaxation_iterations`. Warm starting,
gravity and `integrate` stay outside both loops: they are once-per-substep, and
putting gravity inside would apply it N times.

- [ ] **Step 6: Run the bit-identity test and the whole crate**

Run: `cargo test -p bevox_physics`
Expected: PASS, every gate, unchanged. In particular
`a_stack_sleeps_without_creeping`, `a_bouncy_body_returns_to_a_quarter_of_its_height`,
`a_grab_lifts_a_light_body_but_not_a_heavy_one` and
`resting_weight_breaks_nothing` — Review Focus 1-4 — must pass untouched, because
the loops run once.

- [ ] **Step 7: Prove the break**

Set `VELOCITY_ITERATIONS` to 0, run `cargo test -p bevox_physics`, and record
which gates fail. Expected: the contact gates fail — a body falls through the
floor, because nothing solves the normal constraint. Restore to 1. Record the
failing gate names in the commit message; a loop that is never entered is not
known to be entered.

- [ ] **Step 8: List the constants on the map page**

Add both to the "How a contact behaves" table in `docs/map/physics-constants.md`,
beside `SUBSTEPS`.

Run: `cargo test -p bevox --test map`
Expected: PASS 4/4. Without this step `the_maps_name_every_constant` fails.

- [ ] **Step 9: Commit**

```bash
git add crates/bevox_physics/src/lib.rs crates/bevox_physics/src/solver.rs docs/map/physics-constants.md
git commit -m "refactor(physics): give the solver its iteration counts, at one"
```

---

### Task 2: measure the counts, interleaved

**Files:**
- Test: `crates/bevox_physics/src/solver.rs` (test module) — `what_the_iteration_counts_cost`

**Interfaces:**
- Consumes: `Tuning`, `step_with` from Task 1.
- Produces: the numbers Task 3 chooses on. No production code.

This is a measurement in the shape of `sleep::sleeping_is_timed` and
`detach::a_detach_is_timed` — a `#[test]` that prints and asserts only what must
not regress, not a gate on a number.

- [ ] **Step 1: Write the measurement**

`what_the_iteration_counts_cost`, marked `#[ignore]` because it times things and
is not a gate, builds **two** fixtures and times every variant against both, in
one invocation, medians of seven, variants interleaved so cross-run drift cannot
be read as an effect:

- *the load fixture*, which is **not in the suite** — the original was deleted on
  2026-09-28 when it was found to diverge, so build it from this description
  rather than looking for it. On the `slab(64, 0..8)` floor every other solver
  test uses: a `cube(4, 4)` resting at y 10.2, a **one-voxel-thick plate**
  (`cube(4, 1)`) on top of it at y 12.2, and a heavy body on the plate at y
  13.2 whose material density gives it **240 times** the plate's mass. The
  recorded failure is the stack **accelerating upward, at 23 voxels a second by
  tick 33**; if the rebuilt fixture does not reproduce that at `(1,1)`, say so
  and stop rather than tuning it until it does.
- *the resting fixture*: the three-cube stack of `a_stack_of_three_stands_still`,
  which is the common case and must not get slower for nothing.

Tunings: `(1,1)`, `(2,1)`, `(4,1)`, `(8,1)`, `(4,4)`, and `(1,1)` again last.

For each it prints median ms a tick, the stack's worst upward velocity over 200
ticks, and the resting penetration after 2000.

- [ ] **Step 2: Assert only the two things that are not opinions**

- The repeated `(1,1)` is within 15% of the first `(1,1)`, or the run is drift
  and its numbers may not be used.
- Every variant completes 200 ticks of the load fixture in under 2 seconds —
  **Review Focus 5**, that a count cannot make a tick unbounded.

- [ ] **Step 3: Run it and read all of the output**

Run: `cargo test -p bevox_physics what_the_iteration_counts_cost -- --nocapture --ignored`
Expected: PASS, with a printed table.

- [ ] **Step 4: Commit the measurement with its numbers in the message**

```bash
git add crates/bevox_physics/src/solver.rs
git commit -m "test(physics): measure what the solver's iteration counts cost"
```

---

### Task 3: choose the counts, and gate what the choice bought

**Files:**
- Modify: `crates/bevox_physics/src/lib.rs` — the two constants take their measured values
- Modify: `docs/map/physics-constants.md` — the values and their one-line reasons
- Create: `docs/concepts/solver-convergence-is-a-setting.md` — the measurement, and what it did or did not fix
- Modify: `docs/reference/dwyer-drift.md` — F7 resolved
- Modify: `docs/log.md`
- Test: `crates/bevox_physics/src/solver.rs`

**Interfaces:**
- Consumes: Task 2's numbers.
- Produces: the converged impulse Part 2 of the spec thresholds fracture on.

- [ ] **Step 1: Write the convergence gate**

`a_heavy_body_on_a_thin_plate_does_not_climb`: the Task 2 load fixture, run 200
ticks, asserting no body's `velocity.y` exceeds a small positive bound at any
tick. Write it **before** changing the constants.

- [ ] **Step 2: Run it at (1,1) to verify it fails**

Run: `cargo test -p bevox_physics a_heavy_body_on_a_thin_plate_does_not_climb`
Expected: FAIL, the stack climbing, which is the 2026-09-28 finding reproduced as
a gate.

**If it passes at (1,1), stop and report.** The divergence is then not a
convergence failure, Task 3's premise is wrong, and the rest of this task is
unjustified — the counts stay at 1 and the spec's Part 1 claim needs rewriting
rather than implementing.

- [ ] **Step 3: Set the constants to the lowest measured values that pass it**

From Task 2's table. Lowest, not best: each iteration is paid every substep of
every tick forever.

- [ ] **Step 4: Write the monotonicity gate**

`more_iterations_do_not_deepen_a_resting_contact`: the three-cube stack settled
2000 ticks at `Tuning::default()` penetrates no more than at `(1,1)`. More work
making resting worse is the signature of a mis-scaled bias, and it is the thing
most likely to be wrong about raising a count.

- [ ] **Step 5: Run the whole crate at the new defaults**

Run: `cargo test -p bevox_physics`
Expected: PASS. **Review Focus 1-4 are now live**: `a_stack_sleeps_without_creeping`,
`a_bouncy_body_returns_to_a_quarter_of_its_height`,
`a_grab_lifts_a_light_body_but_not_a_heavy_one` and the restitution gates are
reading a differently converged solver for the first time. A failure here is a
real finding about the gate's calibration, not a reason to lower the count —
report it rather than tuning until green.

- [ ] **Step 6: Prove the break**

Return both constants to 1; `a_heavy_body_on_a_thin_plate_does_not_climb` must
fail. Restore.

- [ ] **Step 7: Write the bundle**

`docs/concepts/solver-convergence-is-a-setting.md`: the table from Task 2, the
chosen counts and why, and whether the 240:1 divergence was convergence — stated
as measured, not as expected. Mark F7 resolved in the ledger with the count and
the tick cost. Add the log entry. Link the concept from
`docs/concepts/index.md` and `docs/index.md`, and validate.

Run `cargo test -p bevox --test map`, and validate the bundle with the
`okf:validate` skill.
Expected: conformant, 4/4.

- [ ] **Step 8: Commit**

```bash
git add crates/bevox_physics/src/lib.rs docs/
git commit -m "perf(physics): converge the solver, and say what it cost"
```

---

# What this plan does not do

- **It does not touch fracture.** `peak` is read, never changed. Part 2 of the
  spec is the next plan, and it depends on this one's constants being settled.
- **It does not touch detection.** Part 3 is after that.
- **It does not multithread anything.** His devlog 27 took 12 ms to 8 ms with a
  thread pool; that is a different drift item and a different plan.
