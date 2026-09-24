---
type: Implementation Plan
title: 'A Physics Crate, Forces, and a Tick-Free Ceiling'
description: 'Physics moves to its own crate; bodies take forces and impulses from outside; and the speed ceiling stops depending on the tick rate.'
tags: [physics, architecture, forces, determinism]
generated: { by: claude-opus-5/claude-code, at: 2026-09-24T00:00:00Z }
---

# A Physics Crate, Forces, and a Tick-Free Ceiling

**Goal:** Physics in its own crate; a way for gameplay code to push a body with a force or an impulse; and a speed ceiling that does not move when the tick rate does.

**Architecture:** `bevox_physics` holds the simulation and depends on `bevox_core`, which keeps the data. `bevox_render` keeps importing neither. `Body` stays in core, because the renderer marches it, and the per-body state it carries — features, warm-start impulses, mass properties — moves to core with it; the code that *computes* them moves to the physics crate.

**Spec:** `docs/superpowers/specs/2026-09-15-bevox-rigid-bodies-design.md`

## What the audit found, which is why this plan is small

Measured before planning, and it decided the scope:

- **The render frame rate cannot affect physics already.** The tick runs in `FixedUpdate`, where Bevy's `Res<Time>` is `Time<Fixed>`.
- **Across physics tick rates the solver is close to invariant**: the same scene at 32, 64, 128 and 240 Hz stops a slide within **0.38%** and lands a fall within **0.02%**.
- **Motion is already impulse-driven.** Outside tests there are three direct velocity writes in the whole module: a split piece inheriting rigid motion, a body being put to sleep, and `cap_speed`.

So nothing here rebuilds the solver. Three real gaps get closed.

## Global Constraints

- No new dependencies. `bevox_physics` depends on `bevox_core` and `glam`, nothing else. No Bevy, no GPU.
- `bevox_render` must still not depend on physics when this is done.
- Every gate is proven by a deliberate break.
- Physics is never merged to master.
- Behaviour at 64 Hz must not change: the crate move and the ceiling rewrite are refactors, and the existing gates are the proof.

## Tasks

### Task 1: The crate

**Files:**
- Create: `crates/bevox_physics/{Cargo.toml,src/lib.rs}`
- Move: every `crates/bevox_core/src/physics/*.rs` except the data types named below
- Modify: `Cargo.toml` (workspace members), `crates/bevox_core/src/{lib.rs,body.rs}`, `crates/bevox/{Cargo.toml,src/*.rs}`

What stays in `bevox_core`, because `Body` holds it and the renderer holds `Body`: `Features`, `ContactKey`, `ContactImpulse`, `MassProperties`. Data, not algorithms.

What moves: `classify`, `contact`, `detach`, `fracture`, `joint`, `mass`, `merge`, `sculpt`, `sleep`, `solver`, the constants, and the test fixtures. `Body::recompute` goes with them, as `physics::mass::recompute(&mut Body, &MaterialTable) -> bool`, because it computes mass properties and features.

Gate: the existing suite passes unchanged — this is a move, and the gates that already exist are what prove it. Plus one new check: `bevox_render`'s manifest does not name `bevox_physics`, so the layering cannot rot quietly.

### Task 2: Forces and impulses from outside

**Files:**
- Modify: `crates/bevox_core/src/body.rs`, `crates/bevox_physics/src/solver.rs`
- Test: in `body.rs` and `solver.rs`

`Body` gains an accumulator and four ways to push it:

- `add_impulse(j)` and `add_impulse_at(j, world_point)` — applied at once, because an impulse *is* an instantaneous change of momentum.
- `add_force(f)` and `add_force_at(f, world_point)` — accumulated, applied by the integrator over the tick, cleared at the end of it.

Gravity and drag keep their own path: they are accelerations, not forces, and dividing one by the mass to multiply it back is noise.

Gates: an impulse changes velocity by `j / m` at once and nothing else; a force applied every tick for a second changes it by `f / m` within a percent; a force applied once is gone the next tick; a force off the centre of mass produces the right spin and the right linear change; a sleeping body ignores neither — anything pushed wakes up.

Breaks: apply forces without clearing them — the once-only gate must fail; apply an impulse divided by the tick — the instantaneous gate must fail; ignore the lever arm in `add_force_at` — the spin gate must fail.

### Task 3: A ceiling in voxels per second

**Files:**
- Modify: `crates/bevox_physics/src/{mod.rs,solver.rs}`
- Test: in `solver.rs`

`MAX_TRAVEL` (4 voxels a tick) becomes `MAX_SPEED` (256 voxels a second). At 64 Hz that is the same number and nothing changes; at any other rate the ceiling stops moving. The detection margin is still the body's own travel, so nothing else shifts.

Gates: the existing tunnelling gate, now run at 32, 64 and 128 Hz rather than one; and the ceiling itself is the same speed at every rate.

Break: put the cap back in voxels per tick — the ceiling gate must fail at every rate but one.

### Task 4: The cross-rate gate

**Files:**
- Test: `crates/bevox_physics/src/solver.rs`

The measurement that shaped this plan becomes a gate: one scene at 32, 64, 128 and 240 Hz, with the spread of a slide's stopping point and a fall's position pinned to what was measured, so a later change cannot widen it without saying so.

It asserts a **bound, not equality**: a discrete solver is not tick-invariant and this plan does not pretend otherwise. What it must not do is drift.

Break: make `SUBSTEPS` scale with the tick — the spread must narrow, which fails the bound from the other side and proves the gate reads what it claims.

### Task 5: Record

The spec's structure section, this plan's Measurements, the bundle, and a concept if the audit's numbers turn out to be worth carrying — the tick-rate spread is the candidate.

## Measurements

(Filled in during execution.)

## What changed during execution

(Filled in during execution.)
