---
type: Implementation Plan
title: 'Fracture on a size-scaled impulse'
description: "Widen Material::strength to f32, threshold the accumulated normal impulse instead of the closing speed, scale each side's threshold by its own size, and hand back exactly the excess."
tags: [physics, fracture, dwyer, materials]
generated: { by: claude-opus-5/claude-code, at: 2026-10-02T00:00:00Z }
sources:
  - id: spec
    resource: /superpowers/specs/2026-09-30-bevox-fracture-and-detection-design.md
    title: 'Fracture and detection, as he built them'
  - id: rigid_pixels
    resource: /reference/rigid-pixels.md
    title: "Dwyer's rigid_pixels, read from source"
  - id: counts
    resource: /concepts/solver-convergence-is-a-setting.md
    title: "The solver's iteration counts"
---

# Fracture on a size-scaled impulse — implementation plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make a slowly-crushed body break. Today `break_what_gave_way` thresholds
the closing speed, so the mouse grab — enormous force, near-zero speed — cannot
break anything however hard it presses. Threshold the accumulated normal impulse
instead, as Dwyer does.

**Architecture:** Four changes, each landing on its own. The units widen first
(`u16` → `f32`, behaviour unchanged), then the quantity changes (closing speed →
accumulated impulse, with every strength recalibrated), then each side's
threshold is scaled by its own size, then `REBOUND`'s fixed fraction is replaced
by handing back exactly the excess. Splitting the units from the quantity is the
point: **the units were the problem the first time, and changing both at once is
why the physics got changed to fix an integer.**

**Tech Stack:** Rust 2024, no new dependencies. `cargo test -p bevox_physics --lib`.

**Spec:** [Fracture and detection, as he built them](../specs/2026-09-30-bevox-fracture-and-detection-design.md) — **Part 2 only.** Part 1 is done (`7f031e1..271c660`); Part 3 is a later plan.

## Global Constraints

- `VELOCITY_ITERATIONS` and `RELAXATION_ITERATIONS` stay at **1**. Every strength
  calibrated here is calibrated against that, and `solver::tests::the_solver_iteration_counts_are_one`
  exists to stop them drifting. If a task needs them changed, stop and report.
- Run the suite as `cargo test -p bevox_physics --lib`. **Without `--lib` cargo
  runs an empty integration target and prints "0 passed"**, which looks like
  success.
- A new column-zero `pub const` in `crates/bevox_physics/src/lib.rs` or
  `crates/bevox_core/src/material.rs` fails `the_maps_name_every_constant` until
  `docs/map/physics-constants.md` lists it. Run `cargo test -p bevox --test map`
  in the task that adds one.
- Every gate is proven by a deliberate break, named in the task's commit message.
- Do not run `cargo fmt` across the repo; `classify.rs` is pre-existingly dirty
  and is not ours.
- Commit on `feat/interleaved-stepping`. Do not push, merge, or touch `master`.

## Review Focus

Five things this change can break that no task's own tests name. Each pinned to
the task that owns the code.

1. **A resting contact's `peak` is not zero — it is the load.** Warm starting
   seeds each tick's impulse from the last, so a settled 20,000-voxel piece
   carries ~1e7 of accumulated normal impulse while doing nothing. This is
   exactly what failed by 61,134 fractures on 2026-09-28 and drove the retreat to
   a closing speed. `resting_weight_breaks_nothing` is the gate; it is the hardest
   constraint in this plan, not a formality. Pinned in Task 2.
2. **A multi-material body gets one size and many strengths.** The threshold comes
   from the struck *voxel's* material, the size term from the *body's* voxel
   count. A hard voxel in a small body is the case to check. Pinned in Task 3.
3. **The static world has no voxel count.** It is unbounded, so its size term
   saturates at `SIZE_CAP` rather than reading 1 and making terrain infinitely
   fragile. Pinned in Task 3.
4. **`UNBREAKABLE` becomes `f32::INFINITY`, and comparisons with NaN are false.**
   A NaN blow would silently never break anything. Pinned in Task 1.
5. **Handing back the excess can exceed what the contact carried.** The excess is
   over the *weaker* threshold while the impulse is the pair's; a hand-back larger
   than `peak` would add energy. Pinned in Task 4.

---

### Task 1: widen `strength` to `f32`, changing nothing else

**Files:**
- Modify: `crates/bevox_core/src/material.rs` — `strength` type, `UNBREAKABLE`, the `Eq` derive and the comment that justifies it
- Modify: `crates/bevox_physics/src/fracture.rs` — `over_strength` signature
- Modify: `crates/bevox_physics/src/solver.rs` — `side`'s `strength` parameter
- Modify: `crates/bevox_physics/src/lib.rs` — `GLASS_STRENGTH` and any other `u16` strength
- Test: `crates/bevox_core/src/material.rs`, `crates/bevox_physics/src/solver.rs`

**Interfaces:**
- Consumes: nothing.
- Produces: `Material::strength: f32`; `material::UNBREAKABLE: f32 = f32::INFINITY`;
  `fracture::over_strength(blow: f32, strength: f32) -> Option<f32>`.

- [ ] **Step 1: Write the failing test**

`a_material_keeps_its_strength_as_a_float` in `material.rs`'s test module: a
material pushed with `strength: 150.5` reads back `150.5`. It must not compile
today.

- [ ] **Step 2: Run it to verify it fails**

Run: `cargo test -p bevox_core --lib a_material_keeps_its_strength_as_a_float`
Expected: FAIL to compile — `expected u16, found floating-point number`.

- [ ] **Step 3: Change the type**

`strength: u16` → `f32`. `UNBREAKABLE: u16 = u16::MAX` → `f32 = f32::INFINITY`.
`over_strength` takes `f32`. **Keep every existing value numerically identical**
(`DEFAULT_STRENGTH` 150 → 150.0, `GLASS_STRENGTH` 20 → 20.0): this task changes
units, not behaviour.

`Material` loses `Eq` because `f32` is not `Eq`. Keep `PartialEq`. The comment at
`material.rs` saying friction is an integer "so `Material` stays `Eq`" is now
false — rewrite it to say what is actually true of those fields, and do **not**
change the fields themselves.

Guard **Review Focus 4**: `over_strength` returns `None` for a non-finite blow as
well as for `UNBREAKABLE`. A NaN comparison is false, so without this a NaN blow
silently breaks nothing.

- [ ] **Step 4: Run the whole suite**

Run: `cargo test -p bevox_physics --lib` and `cargo test -p bevox_core --lib`
Expected: PASS, every gate, unchanged. `strength` values are small integers that
`f32` represents exactly, so no fracture gate may move. **If one moves, stop and
report** — it means something depended on integer truncation.

- [ ] **Step 5: Prove the break**

Set `UNBREAKABLE` to `0.0` and run: the fracture gates must fail, because
everything becomes infinitely fragile. Restore. Record the failing gate names.

- [ ] **Step 6: Commit**

```bash
git add crates/bevox_core/src/material.rs crates/bevox_physics/
git commit -m "refactor(physics): widen Material::strength to f32"
```

---

### Task 2: threshold the accumulated impulse, per side

**Files:**
- Modify: `crates/bevox_physics/src/solver.rs` — `break_what_gave_way`
- Modify: `crates/bevox_core/src/material.rs` — every strength, recalibrated
- Modify: `docs/map/physics-constants.md` if any constant's value or meaning changes
- Test: `crates/bevox_physics/src/solver.rs`

**Interfaces:**
- Consumes: `over_strength(f32, f32)` from Task 1; `peak: &[f32]` already computed in `step_with`.
- Produces: `Fracture::blow` is an impulse; `Fracture::over` is per side.

- [ ] **Step 1: Write the crush gate — the symptom this whole plan exists for**

`a_slow_crush_breaks_what_it_presses` in `solver.rs`'s test module. A body held
by a grab joint (as `joint.rs`'s test scene does) pressed into a brittle body at
near-zero closing speed over many ticks: the brittle body breaks. Assert
`StepOutcome::fractures` is non-empty.

- [ ] **Step 2: Run it to verify it fails**

Run: `cargo test -p bevox_physics --lib a_slow_crush_breaks_what_it_presses`
Expected: FAIL — nothing breaks. Closing speed is ~0 however hard the grab presses.
**This failure is the bug Flori reported; confirm you have reproduced it before
changing anything.**

- [ ] **Step 3: Change the blow and make the threshold per side**

In `break_what_gave_way`: `blow` becomes `peak[at]`, not `-approach[at]`. The
`side` closure tests that blow against **that side's own** strength and records
**that side's own** `over`. The existing `if peak[at] <= 0.0 { continue; }` guard
stays — it is what distinguishes a speculative contact that never touched.

Replace the long comment arguing for closing speed with one saying what is true
now: the impulse is the quantity, it is rate-independent, and the units are why
the first attempt failed.

- [ ] **Step 4: Recalibrate every strength against resting load**

**Review Focus 1 is this step.** Run `resting_weight_breaks_nothing` and read the
accumulated impulse a settled body actually carries — print it before choosing
numbers. Set `DEFAULT_STRENGTH` and the named strengths above that load and below
a real collision's impulse. State in the commit message what resting load you
measured and what margin you left.

- [ ] **Step 5: Run the fracture gates**

Run: `cargo test -p bevox_physics --lib`
Expected: PASS, including `resting_weight_breaks_nothing`,
`a_hard_landing_breaks_and_a_soft_one_does_not`, and
`the_same_collision_breaks_at_any_tick_rate`. The last is the claim that an
impulse is rate-independent; if it fails, the quantity is wrong, not the
calibration — stop and report rather than tuning strengths until it passes.

- [ ] **Step 6: Prove the break**

Revert `blow` to `-approach[at]`: the crush gate must fail. Restore.

- [ ] **Step 7: Commit**

```bash
git add crates/bevox_physics/src/solver.rs crates/bevox_core/src/material.rs docs/
git commit -m "fix(physics): break on the impulse a contact carried, not how fast it met"
```

---

### Task 3: scale each side's threshold by its own size

**Files:**
- Modify: `crates/bevox_core/src/body.rs` — cache a voxel count on `Body`
- Modify: `crates/bevox_physics/src/mass.rs` — set it in `recompute`
- Modify: `crates/bevox_physics/src/fracture.rs` — the size term, `SIZE_CAP`
- Modify: `crates/bevox_physics/src/solver.rs` — pass each side's size
- Modify: `docs/map/physics-constants.md` — `SIZE_CAP`
- Test: `crates/bevox_physics/src/fracture.rs`, `crates/bevox_physics/src/solver.rs`

**Interfaces:**
- Consumes: Task 2's per-side threshold.
- Produces: `Body::voxel_count: u32`; `fracture::SIZE_CAP`; `over_strength` takes the side's voxel count.

His 2D rule is `sqrt(min(area, 7))`, commented as making small objects break
sooner. The 3D analogue of a length from an area is a length from a volume:
**`cbrt(min(voxels, SIZE_CAP))`**, with `SIZE_CAP = 27` so the factor runs 1 → 3,
the same span his 1 → 2.65 covers. Change `SIZE_CAP` only if a gate demands it,
and say so if you do.

- [ ] **Step 1: Write the size gate**

`a_small_body_breaks_before_a_large_one` in `solver.rs`: two bodies of the same
material, one small and one large, struck identically — the small one breaks and
the large one does not.

- [ ] **Step 2: Run it to verify it fails**

Expected: FAIL — both break, or neither, because size does not enter the threshold.

- [ ] **Step 3: Cache the voxel count**

`mass::recompute` already holds the `&[(UVec3, MaterialId)]` slice that
`mass_properties` reads. Store its length on `Body` there. Do **not** call
`Contree::voxels()` inside `break_what_gave_way` — it allocates a `Vec`, and this
runs per contact per tick.

- [ ] **Step 4: Apply the size term, and handle the world**

**Review Focus 3:** the static world has no body and no voxel count. It takes
`SIZE_CAP` — saturated, the largest thing there is — not 1. Say so in a comment,
because reading 1 would make terrain the most fragile thing in the scene and
nothing else would catch it.

**Review Focus 2:** the threshold is the struck voxel's material scaled by the
owning body's size. Add a gate or an assertion covering a hard voxel in a small
body if the behaviour surprises you.

- [ ] **Step 5: Run everything, including the map gate**

Run: `cargo test -p bevox_physics --lib` and `cargo test -p bevox --test map`
Expected: PASS 4/4 on the map gate — `SIZE_CAP` must be listed.

- [ ] **Step 6: Prove the break**

Drop the size term (factor 1 everywhere): the size gate must fail. Restore.

- [ ] **Step 7: Commit**

```bash
git add crates/ docs/
git commit -m "feat(physics): small things break before large ones"
```

---

### Task 4: hand back the excess, and delete `REBOUND`

**Files:**
- Modify: `crates/bevox_physics/src/fracture.rs` — `REBOUND` removed
- Modify: `crates/bevox_physics/src/solver.rs` — the give-back loop
- Modify: `docs/map/physics-constants.md` — `REBOUND` removed
- Test: `crates/bevox_physics/src/solver.rs`

**Interfaces:**
- Consumes: Tasks 2 and 3's per-side thresholds.
- Produces: no `REBOUND`.

- [ ] **Step 1: Write the asymmetry gate**

`the_weaker_side_breaks_and_the_stronger_does_not`: two bodies of different
strength in one collision — the weaker breaks, the stronger does not, and each
`Fracture` reports its own `over`. This is the behaviour Flori read as a bug, and
it is correct; the gate is what makes it explicable.

- [ ] **Step 2: Run it to verify it fails or passes, and say which**

If Tasks 2 and 3 already made it pass, say so and keep it — a gate that passes on
arrival is still the record of the claim. Do not manufacture a failure.

- [ ] **Step 3: Replace the fixed fraction with the excess**

A contact carries what it takes to break the weaker side and returns the
remainder, so `REBOUND = 0.6` goes. **Review Focus 5:** clamp the hand-back to
`peak[at]` — the excess is measured over the weaker threshold while the impulse
belongs to the pair, and returning more than was carried would add energy. Gate
that clamp.

- [ ] **Step 4: Run everything**

Run: `cargo test -p bevox_physics --lib`
Expected: PASS, including `breaking_something_does_not_stop_you` — a rock that
breaks a window carries on through it.

- [ ] **Step 5: Prove the break**

Hand back nothing: `breaking_something_does_not_stop_you` must fail. Restore.

- [ ] **Step 6: Write the bundle**

Update [the fracture plan](2026-09-24-bevox-fracture.md) with a "Names as
shipped" note if any gate was renamed. Correct
[the drift ledger](../../reference/dwyer-drift.md): the C-cause row saying the
threshold is a closing speed, and the F-count if it changes. Add the log entry.
Validate with the `okf:validate` skill, and run `cargo test -p bevox --test map`.

- [ ] **Step 7: Commit**

```bash
git add crates/ docs/
git commit -m "feat(physics): hand back exactly the excess, and retire REBOUND"
```

---

# What this plan does not do

- **It does not touch detection.** Part 3 replaces `MAX_SPEED` with substepped
  detection and is a separate plan.
- **It does not change the iteration counts.** They are 1, and every strength
  here is calibrated against that.
- **It does not fix the 240:1 collapse.** That defect is characterised by
  `the_240_to_1_load_collapses_through_the_floor` and is independent of fracture.
