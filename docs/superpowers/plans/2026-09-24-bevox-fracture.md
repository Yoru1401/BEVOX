---
type: Implementation Plan
title: 'Fracture'
description: 'Bodies and terrain crack where a collision is too hard for the material, after Dwyer devlog 28: cracks are drawn as empty voxels and detachment produces the pieces.'
tags: [physics, fracture, materials, dwyer]
generated: { by: claude-opus-5/claude-code, at: 2026-09-24T00:00:00Z }
---

# Fracture Implementation Plan

**Goal:** Throw a box hard enough and it breaks. Hit a wall hard enough and the wall breaks. What breaks, and into how many pieces, comes from the material and the strength of the collision.

**Architecture:** After Dwyer's devlog 28, whose insight is that fracture needs almost no new machinery. A contact whose accumulated impulse passes the material's strength raises a fracture event; the event draws cracks by **setting voxels empty**; and the code that already turns disconnected voxels into rigid bodies -- `detach` for the world, `sculpt`'s split for a body -- produces the pieces. Nothing plans a pattern, labels a piece or copies a volume.

**Spec:** `docs/superpowers/specs/2026-09-15-bevox-rigid-bodies-design.md`

**Reference:** `docs/reference/dwyer-devlogs.md`, devlog 28.

## Global Constraints

- No new dependencies. `bevox_core` keeps no Bevy and no GPU dependency.
- Every gate is proven by a deliberate break.
- **Impulse, never force.** Force depends on the tick rate, so a threshold on force breaks differently at 30 and 60 fps. This gets its own gate.
- Physics is never merged to master.
- `MAX_BODIES` is 16. A fracture that would free more pieces than there is room for leaves the rest in place rather than deleting them.
- Flori runs the app; I do not.

## What Dwyer does, and what this takes from it

| His | Here |
|---|---|
| Impulse over a per-material threshold raises a fracture event | the same, read off the solver's accumulated normal impulse |
| The impulse at those contacts is reduced, so a rock carries on through the window | the same, as a fraction of the normal impulse returned to the bodies |
| Cracks are drawn by setting voxels empty; the "neighbourhood disconnector" makes the pieces | the same, with `detach` for the world and `sculpt`'s split for a body |
| Patterns are authored boolean voxel volumes in a per-material table, cached in six orientations | **not taken.** Cracks are generated analytically from planes through the impact. There is no modding API to author patterns for, and an authored-pattern table is a data format, a cache and an editor for a look we can get from three random planes |

That last row is the one deliberate departure, and it is reversible: the crack generator is one function behind one call, so a pattern table can replace it without touching the solver or the application path.

## Tasks

### Task 1: `Material::strength`

**Files:**
- Modify: `crates/bevox_core/src/material.rs`, `crates/bevox/src/scenes.rs`
- Test: in `material.rs`

A `u16` column, so `Material` still derives `Eq`: the impulse a contact on this material can take before it cracks, in the solver's own units. `UNBREAKABLE` is `u16::MAX`, and `DEFAULT_STRENGTH` sits where ordinary stone sits. The demo scene's materials get values that differ in the order they should: ice brittle, stone ordinary, rubber unbreakable.

Gates: a table round-trips strength; the demo scene's ice is more brittle than its stone, and its rubber never breaks. Break: give every material the same strength -- the ordering gate must fail.

### Task 2: Fracture events in the solver

**Files:**
- Create: `crates/bevox_core/src/physics/fracture.rs`
- Modify: `crates/bevox_core/src/physics/solver.rs`, `crates/bevox_core/src/physics/mod.rs`
- Test: in `fracture.rs` and `solver.rs`

`Fracture { at: Vec3, voxel: UVec3, body: Option<BodyId>, impulse: f32 }`. After the substeps, each contact's accumulated normal impulse is compared against the strength of the material at **each side** of it -- the body's voxel and the world's or other body's voxel -- so a hard landing can break the thing that lands, the thing it lands on, or both.

`step` returns `StepOutcome { rebuild: bool, fractures: Vec<Fracture> }` rather than a bare `bool`. A ninth parameter would be the alternative, and this function already carries eight.

Part of the normal impulse is returned to the bodies along the contact normal, so a body that breaks something keeps moving through it. The fraction is one named constant.

Gates:
- a fast body into the floor raises events, a slow one raises none;
- the threshold tracks the material: the same collision breaks ice and not stone;
- **the same collision raises the same events at `dt` and at `dt/2`** -- the frame-rate gate;
- a body that breaks something is still moving afterwards.

Breaks: threshold on force (`impulse / dt`) rather than impulse -- the frame-rate gate must fail; compare against a fixed constant rather than the material -- the material gate must fail; return none of the impulse -- the momentum gate must fail.

### Task 3: Crack generation

**Files:**
- Modify: `crates/bevox_core/src/physics/fracture.rs`
- Test: in `fracture.rs`

`cracks(at: IVec3, reach: i32, planes: u32, seed: u64) -> Vec<IVec3>`: voxels within `reach` of the impact that lie within half a voxel of one of `planes` planes through it, the plane normals drawn from a seeded generator so a tick is reproducible. `reach` and `planes` are derived from how far past the threshold the impulse went, so a harder hit breaks a bigger region into more pieces.

Gates: every crack voxel is inside the reach; a hard hit cuts a solid cube into two or more connected components, checked with the detachment code that already answers exactly that question; a hit barely over the threshold cuts fewer pieces than one well over it; the same seed gives the same cracks twice.

Breaks: emit no planes -- the component gate must fail; ignore the impulse when sizing the reach -- the harder-hit gate must fail.

### Task 4: Breaking a body

**Files:**
- Modify: `crates/bevox/src/main.rs`
- Test: in `main.rs`

For each event naming a body: clear the crack voxels in that body's volume, then run the split that `sculpt` already performs, up to the room left under `MAX_BODIES`. The pieces inherit `from_terrain` from the parent, as a sculpted piece does, so debris that came from the terrain can still merge back.

Gates: a body hit hard enough becomes two or more bodies; their total mass is the parent's less what the cracks removed; a body of an unbreakable material never splits; with no room under the cap, the body is left whole rather than half-erased.

Breaks: apply cracks without splitting -- the count gate must fail; ignore the cap -- the room gate must fail.

### Task 5: Breaking terrain

**Files:**
- Modify: `crates/bevox/src/main.rs`
- Test: in `main.rs`

For each event naming the world: clear the crack voxels in the tree, tell the scene what changed through `world_changed`, then run `detach` over the impact's box with whatever room is left.

The distance field needs nothing: clearing only raises true distances, which is the safe direction. Fullness is not safe in either direction, which is exactly what `world_changed` is for.

Gates: a hard hit on the floor frees at least one body; fullness still describes the world afterwards, reusing the check the detachment and merge gates use; a hit on unbreakable terrain changes nothing at all.

Break: skip `world_changed` -- the fullness gate must fail.

### Task 6: Measure and record

**Files:**
- Modify: `crates/bevox_core/tests` or `crates/bevox_render/tests/gpu_bench.rs`, the spec, this plan, the bundle

A tick that fractures against a tick that does not, on the same scene, medians over many ticks. No shader changes, so no codegen re-bench is owed.

Then: the spec's physics section, this plan's Measurements, the bundle's index and log, and a concept if something here turns out to be a rule worth carrying -- the impulse-not-force reason is the candidate.

## Measurements

Release, the GTX 1650 machine, 2026-09-24. Sixteen bodies resting on the floor
and **kept awake on both sides**, medians of 200 ticks, A/B/A in one run:

| Materials | Median tick | Fractures |
|---|---|---|
| hold (strength 40) | 0.9257 / 0.9261 ms | 0 |
| give way (strength 0) | 0.9294 ms | 25,600 over 200 ticks |

So looking for fractures costs **+0.0035 ms against 0.0004 ms of drift, at 128
events a tick** -- a rate far past anything a game would produce. A tick that
finds nothing pays a comparison per contact and allocates nothing.

What this does **not** measure is applying them: `cracks` plus `split` or
`detach`, which happens in the app, once per event rather than once per tick.
Its cost is bounded by the reach cap -- a hit a hundred times over strength
tests the same 25³ box as one nine times over.

The first version of this measurement was wrong and said so: it timed materials
that hold against materials that give way without noticing that **the first set
fell asleep and the second never could**, so it read 0.0042 ms against 1.1024 ms
and would have made fracture look 260 times more expensive than it is.

## What changed during execution

- **Strength is a speed, not an impulse.** The column began as "the impulse a
  contact takes before it cracks" and every physics test in the workspace failed
  at once: a resting 4-voxel cube already carries about 2,400 impulse per
  contact, so everything shattered where it stood. A contact's impulse grows
  with the mass resting on it, which makes a big body break under its own
  weight; the speed that impulse takes away does not. `strength` is now voxels
  per second, and the blow is `impulse / mass`. Dwyer's threshold is still an
  impulse threshold -- this only divides both sides by the same mass.
- **The tick-rate gate had to be rewritten around outcomes.** The peak per-tick
  blow is *not* rate-independent: 16.4 at 64 Hz against 10.4 at 128 on the test
  scene, because a collision the detector sees coming is spread over more ticks
  at a finer rate. What is rate-independent is **what breaks**, so the gate
  asserts that a blow over strength breaks at both rates and a blow under it
  breaks at neither. The second half is the discriminating one: a threshold read
  as a force shatters things at every rate and fails it.
- **The body has to start inside the contact margin** for a collision to land in
  one tick. Further out, the speculative contact appears a tick early and bleeds
  the approach speed away over two, which halves the blow. That is a property of
  the detector, not of fracture, and the fixture says so.
- **`sculpt`'s split came out into `physics::sculpt::split`.** The brush and
  fracture differ only in which voxels they clear -- a sphere against cracks --
  and everything after that is one question: who keeps the identity, what the
  pieces inherit, what happens at the cap. It now has one answer.
- **The crack patterns are analytic, not authored.** Dwyer stores a boolean
  voxel volume per material and caches six orientations of it. There is no
  modding API here to author patterns for, so cracks are a few random planes
  through the impact, seeded by the voxel and the impulse. It is one function
  behind one call, and a pattern table can replace it without the solver or the
  application path noticing.
- **The speed cap sets the ceiling on any blow.** `MAX_TRAVEL` is 1.25 voxels a
  tick, so at 64 Hz nothing can arrive faster than 80 voxels a second. Every
  strength must sit well under that or the material is unbreakable in practice,
  which is why `DEFAULT_STRENGTH` is 40 and not 400.

## Breaks that failed as required

| Break | Gate that caught it |
|---|---|
| Every material given the same strength | `the_demo_scene_breaks_in_the_right_order` |
| Empty space made breakable | `a_material_keeps_its_friction_and_bounce` |
| The threshold read as a force (`blow / dt`) | `the_same_collision_breaks_at_any_tick_rate` |
| One fixed threshold instead of the material's | `what_breaks_depends_on_the_material` |
| No impulse handed back | `breaking_something_does_not_stop_you` |
| Nothing ever past strength | `a_hard_landing_breaks_and_a_soft_one_does_not` |
| No crack planes emitted | `cracks_cut_a_solid_block_into_pieces` |
| Cracks cut but nothing split | `a_body_hit_hard_enough_comes_apart` |
| The world cut without telling the grids | `breaking_the_floor_keeps_the_grids_honest` |
| The body cap ignored | `a_full_scene_leaves_a_broken_body_whole` |
