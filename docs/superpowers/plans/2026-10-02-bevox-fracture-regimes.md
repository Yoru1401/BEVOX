---
type: Implementation Plan
title: 'Fracture in two regimes'
description: "Give Material a crush force, branch fracture on the closing speed, then raise the impact strengths that were only ever held down by the crush ceiling."
tags: [physics, fracture, materials, dwyer]
generated: { by: claude-opus-5/claude-code, at: 2026-10-02T00:00:00Z }
sources:
  - id: spec
    resource: /superpowers/specs/2026-10-02-bevox-fracture-regimes-design.md
    title: Fracture in two regimes
  - id: window
    resource: /concepts/fracture-load-window.md
    title: The fracture load window
---

# Fracture in two regimes — implementation plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** A four-high glass stack lands and stands, and a slow crush breaks stone.
Today neither is true, because one number has to be above a 356,751 landing and
below a 365,906 crush ceiling at the same time.

**Architecture:** `Material` gains a `crush` force beside its impulse `strength`.
`break_what_gave_way` branches on `approach[at]`, the closing speed at detection:
an impact is judged on the impulse exactly as today, a held contact on
`peak * SUBSTEPS / dt`, a force. The regimes stop competing for one constant.

**Tech Stack:** Rust 2024, no new dependencies. `cargo test -p bevox_physics --lib`.

**Spec:** [Fracture in two regimes](../specs/2026-10-02-bevox-fracture-regimes-design.md)

## Global Constraints

- `VELOCITY_ITERATIONS` and `RELAXATION_ITERATIONS` stay **1**; `SUBSTEPS` stays **4**.
- Run the suite as `cargo test -p bevox_physics --lib`. **Without `--lib`** cargo
  runs an empty integration target and prints "0 passed", which looks like success.
- A new column-zero `pub const` fails `the_maps_name_every_constant` until
  `docs/map/physics-constants.md` lists it. Run `cargo test -p bevox --test map`.
- Every gate is proven by a deliberate break that **could** fail. A change moving
  both sides of a comparison together proves nothing — that has already happened
  twice on this branch.
- `cargo fmt` is pre-existingly dirty on `classify.rs`; do not run it repo-wide.
- Commit on `feat/interleaved-stepping`. Do not push, merge, or touch `master`.

## The thing the spec leaves implicit, and it sets the order

**Splitting the regimes does not by itself stop the four-cube landing breaking.**
A landing closes at **−5.36 v/s**, so it is an *impact*, and the impact branch is
unchanged: 356,751 against a strength of 350,000 still breaks.

What the split buys is that **`strength` is no longer held down by the crush
ceiling**. It only sat at 350,000 to stay under 365,906. Once the crush has its
own threshold, the impact strengths are free to rise above every landing
transient — 414,166 at eight cubes. **So the landing is fixed by Task 3's
recalibration, which Task 2 makes possible.** Any task order that raises
`strength` before the branch exists silently disables the crush.

## Review Focus

1. **A resting stack is a held contact with a real force.** Its weight does not
   vanish into the new branch — `resting_weight_breaks_nothing` must pass against
   `crush`, not only against `strength`. Pinned in Task 2.
2. **Sleeping bodies stop generating contacts.** A slept stack produces no `peak`,
   so the held branch cannot fire. Confirm a crush keeps its target awake, or the
   crush gate passes for the wrong reason. Pinned in Task 2.
3. **`dt` is a parameter of `step`, and the held branch divides by it.** A caller
   passing 0 or a denormal makes the force infinite. Pinned in Task 2.
4. **`UNBREAKABLE` must work for both columns**, and a material breakable by
   impact but not by crush (or the reverse) must be expressible. Pinned in Task 1.
5. **Raising `strength` widens cracks.** `fracture::over` sizes the crack pattern,
   and `over` is blow over threshold — raising thresholds shrinks `over` and makes
   smaller cracks for the same hit. Pinned in Task 3.

---

### Task 1: `Material::crush`, read by nothing

**Files:** `crates/bevox_core/src/material.rs`; `crates/bevox_physics/src/lib.rs` (fixtures); `crates/bevox/src/scenes.rs` (palette); `crates/bevox_core/src/vox.rs`, `crates/bevox_core/examples/render_reference.rs`, `crates/bevox_render/tests/common/mod.rs` if they construct `Material` literally.

**Interfaces produced:** `Material::crush: f32`; `material::DEFAULT_CRUSH: f32`.

- [ ] **Step 1: Write the failing test** — `a_material_keeps_its_crush_force` in `material.rs`: a material pushed with `crush: 1.5e6` reads it back.
- [ ] **Step 2: Run it; expect a compile failure** (`no field crush`).
- [ ] **Step 3: Add the field**, `f32`, documented as a **force** in the solver's own units, explicitly contrasted with `strength`'s impulse and naming `IMPACT_SPEED` as what chooses between them. Set it to `UNBREAKABLE` in **every** table for now — **Review Focus 4**: nothing reads it yet, so this task cannot change behaviour, and a material unbreakable by crush is the conservative default.
- [ ] **Step 4: Run `cargo test --workspace`.** Expected: every gate unchanged. **If one moves, stop and report** — a field nothing reads cannot change behaviour, so a moved gate means something else did.
- [ ] **Step 5: Prove the break** — set one material's `crush` to 0.0 and confirm **nothing** fails, which is the point: the field is inert. Record that as the task's evidence, and say plainly it is a null result rather than dressing it as a gate.
- [ ] **Step 6:** `docs/map/physics-constants.md` if `DEFAULT_CRUSH` is column-zero; `cargo test -p bevox --test map`.
- [ ] **Step 7: Commit.**

---

### Task 2: branch on the closing speed

**Files:** `crates/bevox_physics/src/solver.rs` (`break_what_gave_way`, and its call site — `approach` is computed at `solver.rs:151` and must be passed in); `crates/bevox_physics/src/fracture.rs` (`over_crush`, `IMPACT_SPEED`); `docs/map/physics-constants.md`.

**Interfaces produced:** `fracture::IMPACT_SPEED: f32`; `fracture::over_crush(force: f32, crush: f32) -> Option<f32>`.

- [ ] **Step 1: Write the crush gate at three rates** — `a_slow_crush_breaks_what_it_presses_at_any_rate`, running the existing crush fixture at `DT`, `DT/2` and `DT/4` and asserting it breaks at all three. Today it breaks only at `DT`.
- [ ] **Step 2: Run it; expect failure at `DT/2` and `DT/4`.** Quote the failure. This is the rate-dependence measured on 2026-10-02 (365,906 at 64 Hz, 93,512 at 512) showing up as a test.
- [ ] **Step 3: Add `IMPACT_SPEED`** — `0.1`, with a doc comment saying it is **not tuned**: the two events sit at 5.36 and 0.0003 v/s, so any value across four orders of magnitude separates them.
- [ ] **Step 4: Branch.** Pass `approach` into `break_what_gave_way`. When `-approach[at] >= IMPACT_SPEED`, the blow is `peak[at]` against `strength`, exactly as now. Otherwise the blow is `peak[at] * SUBSTEPS as f32 / dt` against `crush`. **Review Focus 3:** guard a non-positive or non-finite `dt` — treat it as an impact rather than dividing.
- [ ] **Step 5: Calibrate the crush column only.** Measure what a press delivers *as a force* and set `crush` per material so the crush gate passes at all three rates. **Do not touch `strength`.** Print the measured force and say what margin you left.
- [ ] **Step 6: Run everything.** **Review Focus 1 and 2 are live here**: `resting_weight_breaks_nothing` now meets the held branch and must pass against `crush`; and confirm the crush target is awake when it breaks, so the gate is not passing because a slept body generates no contacts.
- [ ] **Step 7: Prove the breaks** — force the impact branch always (crush gate must fail); force the held branch always (`the_same_collision_breaks_at_any_tick_rate` must fail); `IMPACT_SPEED` to 100 and to 0, each failing a different gate.
- [ ] **Step 8:** map page; `cargo test -p bevox --test map`. **Commit.**

---

### Task 3: raise the impact strengths the ceiling was holding down

**Files:** `crates/bevox_core/src/material.rs`; `crates/bevox_physics/src/lib.rs`; `crates/bevox/src/scenes.rs`; `crates/bevox_physics/src/solver.rs` (un-ignore a gate).

- [ ] **Step 1: Un-ignore `a_four_high_glass_stack_stands`** and add `an_eight_high_stack_lands_without_breaking`. Both must fail now — the landing transients are 356,751 and 414,166 against a strength of 350,000.
- [ ] **Step 2: Run them; quote both failures.**
- [ ] **Step 3: Raise `GLASS_STRENGTH` above every landing transient** — the measured series is 126,129 / 246,227 / 331,306 / 356,751 / 391,195 / 414,166 for n = 1, 2, 3, 4, 6, 8, and it is **sub-linear** (`~n^0.55`), so a value clear of n=8 is clear of much taller stacks too. State the margin. Raise `DEFAULT_STRENGTH` and the demo palette in the same proportion so the ordering ice < brick < stone survives.
- [ ] **Step 4: Run everything.** `a_hard_landing_breaks_and_a_soft_one_does_not` is the gate most at risk — raising strengths can make a hard landing stop breaking. **If it fails, that is a finding about how much headroom a real impact has, not something to tune away: report the numbers.**
- [ ] **Step 5: Check the cracks.** **Review Focus 5:** `over` is blow over threshold, so raising thresholds shrinks it and shrinks the crack pattern. Run the fracture visual gates and say whether cracks changed size; if they did and it reads worse, say so rather than silently accepting it.
- [ ] **Step 6: Prove the break** — return `GLASS_STRENGTH` to 350,000; the four-high gate must fail again.
- [ ] **Step 7: Commit.**

---

### Task 4: the crush reaches stone, and the bundle says what is true

**Files:** `crates/bevox/src/scenes.rs`; `crates/bevox_physics/src/solver.rs`; `docs/concepts/fracture-load-window.md`; `docs/reference/dwyer-drift.md`; `docs/log.md`; the indexes.

- [ ] **Step 1: Write `a_slow_crush_breaks_stone`** — the user-visible half. Today a crush reaches ice and nothing else, because stone's 1,312,500 sat above a 365,906 ceiling.
- [ ] **Step 2: Run it; expect failure if the demo palette's crush column is still `UNBREAKABLE`.**
- [ ] **Step 3: Give the demo palette its crush column**, keeping stone harder to crush than brick, and brick than ice. Extend `the_demo_palette_outlasts_a_resting_stack` with a crush sibling, or say why one column cannot be checked the way the other is.
- [ ] **Step 4: Run everything**, plus `cargo test -p bevox` and `cargo test -p bevox --test map`.
- [ ] **Step 5: Prove the break** — set stone's crush to `UNBREAKABLE`; the new gate must fail.
- [ ] **Step 6: Rewrite `docs/concepts/fracture-load-window.md`.** The window is gone and the page is now a record of **why one number could not work**: the 1.06x-versus-18,000x measurement, the three ruled-out causes, and what replaced it. Keep the two things this did **not** fix — the n=6 and n=8 stacks that never settle, and the 1.7x indeterminacy in one contact's share. Correct the C-cause row in the drift ledger. Log, link, validate with `okf:validate`.
- [ ] **Step 7: Commit.**

---

# What this plan does not do

- **It does not fix stacking.** n=6 and n=8 never settle — late speeds 2.03 and
  4.12 v/s, held load swinging 59k–133k and 87k–217k. Independent of fracture,
  and the next physics item after this.
- **It does not improve the indeterminate split.** One contact's share moves 1.7x
  with body size; that bounds any threshold and this change does not touch it.
- **It does not do Part 3**, substepped detection, which is its own plan.
