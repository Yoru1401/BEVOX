---
type: Design Spec
title: 'Fracture and detection, as he built them'
description: "Expose the solver's iteration counts, replace the closing-speed fracture threshold with his size-scaled impulse, and replace the velocity cap with his substepped detection, now that rigid_pixels can be read."
tags: [physics, fracture, detection, solver, dwyer, respec]
generated: { by: claude-opus-5/claude-code, at: 2026-09-30T00:00:00Z }
sources:
  - id: rigid_pixels
    resource: /reference/rigid-pixels.md
    title: "Dwyer's rigid_pixels, read from source"
  - id: devlogs
    resource: /reference/dwyer-devlogs.md
    title: "Douglas Dwyer's voxel engine, devlog by devlog"
  - id: drift
    resource: /reference/dwyer-drift.md
    title: How far BEVOX has drifted from Dwyer
---

# Fracture and detection, as he built them — design

**Date:** 2026-09-30, **revised 2026-10-01**
**Status:** awaiting review

# Why this exists

Two of BEVOX's inventions were built because his design was unknown. It is known
now: his 2D prototype `rigid_pixels` is public, and both inventions turn out to
be worse than what they replaced.

A third difference was found on the way: his solver exposes two iteration counts
where BEVOX hardcodes one of each (drift **F7**).

**All three are replacements, not additions.** None adds a feature.

**What the 2026-10-01 revision changed.** F7 was listed as out of scope and is
now Part 1, because the quantity Part 2 thresholds on is produced by the loops
F7 changes — see below. Part 2's provenance also firmed up: his own name for
substepped detection is *continuous collision detection*.

# Part 1 — The solver's iteration counts

## What is here now, and why it is wrong

`solver.rs` runs, per substep, **exactly one** biased velocity pass and **exactly
one** unbiased relax pass. Neither is a loop; neither count has a name. His
solver exposes `velocity_iterations` and `relaxation_iterations` as settings.

More velocity iterations is the standard lever for stacking stability, and
stacking stability is what failed on 2026-09-28, when a 240:1 mass ratio
*diverged* — a stack accelerating upward at 23 voxels a second by tick 33.
**BEVOX has been running its solver at the least converged setting there is,
without knowing there was a setting.**

> **Measured, and this premise did not hold.** The counts were exposed and
> measured on 2026-10-01 and re-run on 2026-10-02. Raising the biased count
> alone bought no stability, and the 2026-09-28 divergence above **did not
> reproduce** — the rebuilt fixture collapses *downward* through the floor at
> tick 55 instead. Both counts stay at 1. The paragraph above is left as the
> premise this part was written on; what it actually found is in
> [solver convergence is a setting, and the spec reached for the wrong one](../../concepts/solver-convergence-is-a-setting.md).

## Why this must come first

`peak[at]`, the accumulated normal impulse that Part 2 thresholds fracture on, is
`peak.max(impulse.normal)` taken **inside** both solve loops. A sequential-impulse
solver converges its accumulated impulse upward, so **fewer iterations means a
smaller accumulated impulse**, which means fracture fires later than it should.

So the iteration count is an input to every `strength` value in the material
table. Calibrating strengths first and changing the count afterwards would
invalidate the whole table. Doing it the other way round costs nothing.

## The design

Two constants beside `SUBSTEPS`, and two loops:

```text
VELOCITY_ITERATIONS    wraps the biased pass   (joints, contacts, friction, rolling)
RELAXATION_ITERATIONS  wraps the unbiased pass (the same, after integration)
```

Both default to **1**, so the first commit is bit-identical to today and the gate
below proves it. Then they are *measured*: raise each, and report what it costs a
tick and what it does to the 240:1 case.

**The count is chosen on the measurement, not on the argument.** This project has
twice lost time to a change that ought to have been faster.

## Gates

| Gate | What it proves |
|---|---|
| Every existing solver gate passes unchanged at 1 and 1 | The loops are a refactor before they are a change |
| A new convergence gate: the 240:1 stack **does not accelerate upward** | The divergence is a convergence failure, or it is not — either way it is now known rather than assumed |
| A new monotonicity gate: raising `VELOCITY_ITERATIONS` does not raise the resting penetration | More work does not make resting worse, which is the failure mode of a mis-scaled bias |

Break required: set `VELOCITY_ITERATIONS` to 0 — every contact gate must fail.

# Part 2 — Fracture: a size-scaled impulse, not a closing speed

## What is here now, and why it is wrong

`break_what_gave_way` thresholds **the closing speed at detection**. That was
chosen on 2026-09-28 because `Material::strength` is a `u16` and his impulse rule
failed `resting_weight_breaks_nothing` by 61,134 fractures.

**The units were the problem and the physics was changed to fix them.** His
`breaking_impulse` is an `f32`. Widening the type was available and was not
considered.

What the speed threshold cost, which Flori found by playing:

- **A slow crush breaks nothing.** The mouse grab is a joint with
  `GRAB_MAX_FORCE` near 2e8 that drives toward a velocity goal, so a grabbed body
  pressed into another moves slowly with enormous force. Closing speed is
  approximately zero, so nothing breaks however hard you press. An impulse
  threshold fires there.
- **Asymmetry reads as a bug.** Both sides are given the same blow, so when only
  one breaks it looks arbitrary.

## The design

Per contact, after the substeps, as now:

1. `blow` becomes the contact's **accumulated normal impulse** — the peak already
   tracked as `peak[at]`, which is the right quantity and already exists.
2. Each side's threshold is its own material's strength **scaled by a size term**.
   His is `sqrt(min(area, 7))` in 2D, named for the area force spreads over and
   commented as making small objects break sooner. The 3D analogue of a length
   from an area is a length from a volume: `cbrt(min(voxels, N))`, with `N` chosen
   so the correction applies to small bodies and flattens above them.
3. **Test each side against its own threshold.** One side breaking and the other
   not is then correct and explicable, rather than arbitrary.
4. `strength_ratio` becomes **per side** — that side's impulse over that side's
   threshold — and sizes that side's cracks. `Fracture::over` already carries this
   shape; it stops being shared.
5. **Hand back exactly the excess** over the smaller threshold, replacing
   `REBOUND = 0.6`. A contact carries what it takes to break the weaker side and
   returns the remainder. `REBOUND` disappears.

`Material::strength` changes from `u16` to `f32`, which is the change that should
have been made instead of moving to a speed. `Material` stops deriving `Eq`; the
material table's round-trip gate is the only thing that wanted it, and
`PartialEq` serves that.

**Every strength in the table is recalibrated against the iteration counts fixed
in Part 1**, and the table records which counts it was calibrated at.

## Gates

| Gate | What it proves |
|---|---|
| `resting_weight_breaks_nothing` | **Must still pass**, now on an impulse. It is why the speed was chosen, and an `f32` strength in the impulse's own units is what makes it pass |
| A new crush gate | A grabbed body pressed slowly into a brittle one **breaks it**. This is what the speed threshold cannot do, and it is the symptom Flori reported |
| `the_same_collision_breaks_at_any_tick_rate` | Must still pass. An impulse is rate-independent, which is his whole argument |
| A new asymmetry gate | Two bodies of different strength in one collision: the weaker breaks, the stronger does not, and each reports its own ratio |
| `breaking_something_does_not_stop_you` | Must still pass with the excess handed back instead of a fixed fraction |
| A new size gate | Two bodies of the same material, one small and one large, struck identically: the small one breaks first |

Breaks required: revert to one shared blow (the asymmetry gate must fail); drop
the size term (the size gate must fail); hand back a fixed fraction (the
momentum gate must fail on the fraction that is wrong for the collision).

# Part 3 — Detection: substepped in time, not widened in space

## What is here now, and why it is wrong

`MAX_SPEED` caps a body at 256 voxels a second so that speculative contacts can
stand in for continuous detection. Flori wants it gone: physics should be limited
by drag, not by a clamp.

It cannot simply be deleted. The detection margin is the body's travel, and the
lookup is `(2 * ceil(margin + 0.5) + 1)` cubed per corner voxel — so cost grows
as the **cube** of speed. One impulse at 10,000 voxels a second asks for
31 million lookups per corner and the tick never returns.

**He has no cap, and does not need one**, because he does not widen the search.

**And this is not a departure from him.** His 2026-02-20 sneak peek says the TGS
engine *"features continuous collision detection"*, which is the same phrase
devlog 11 earns from separating-axis projection gaps. Substepped speculative
detection is what he calls CCD, so Part 3 implements his design rather than
diverging from it.

## The design

Keep the margin fixed near `BASE_MARGIN`. For each pair, and for each body
against the world, compute a speed bound — relative linear speed plus each
body's furthest-voxel swing, plus whatever force and torque will add over the
tick — and **detect at several points along the trajectory** instead of once:

```text
steps = ceil(dt * speed_bound / TUNNEL_MARGIN)      at least 1
```

`TUNNEL_MARGIN` is a distance, the analogue of his 0.4. Detection cost is then
**linear** in speed. Contacts found at each point are merged by their existing
voxel-pair key, so warm starting is untouched.

**Detection only.** The solver keeps `SUBSTEPS`, its passes and its structure;
this changes how many times contacts are *looked for*, not how they are solved.
That is what makes it affordable, and it is what his doc comment is emphatic
about.

`MAX_SPEED` is then deleted, and drag is the only thing limiting a fall.

## Gates

| Gate | What it proves |
|---|---|
| `a_body_at_the_speed_cap_does_not_tunnel_through_a_thin_floor` | Rewritten with no cap: a body at any speed up to a large bound does not pass through a thin floor |
| A new cost gate | A body given a very large impulse completes its tick in bounded time. This is the failure that made the cap necessary, and it must be gated rather than trusted |
| `the_same_scene_lands_alike_at_every_tick_rate` | Must still pass; substep counts change with `dt` and the outcome must not |
| `a_resting_body_keeps_its_contact_keys` | Warm starting survives merging contacts found at several trajectory points |

Break required: fix the substep count at 1, and the tunnelling gate must fail.

# Order, and what is not in scope

**1, then 2, then 3.**

- **Part 1 first** because it is a refactor that starts bit-identical, and because
  it fixes the units of the number Part 2 thresholds on. Doing it after Part 2
  would invalidate every strength in the material table.
- **Part 2 second.** It is small, it fixes a symptom Flori can see, and it does
  not touch detection.
- **Part 3 last.** It changes the cost model of the hottest CPU path in the
  physics and wants its own measurement.

Not in scope: the stress scene (drift F4) and anything in the renderer. Both are
recorded in [the ledger](../../reference/dwyer-drift.md) and wait their turn.

# Provenance

Parts 2 and 3 are read from `rigid_pixels`, which carries **no licence** — no
`LICENSE` file and no `license` field — so it is all rights reserved however
public it is. This spec describes mechanisms and constants so that both parts can
be implemented without reference to his source. Nothing is to be copied. Part 1
is read from the same repository's public API shape and from devlog 26, and is
in any case the standard structure of a sequential-impulse solver.
