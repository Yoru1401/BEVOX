---
type: Design Spec
title: 'Fracture and detection, as he built them'
description: "Replace the closing-speed fracture threshold with his size-scaled impulse, and the velocity cap with his substepped detection, now that rigid_pixels can be read."
tags: [physics, fracture, detection, dwyer, respec]
generated: { by: claude-opus-5/claude-code, at: 2026-09-30T00:00:00Z }
sources:
  - id: rigid_pixels
    resource: /reference/rigid-pixels.md
    title: "Dwyer's rigid_pixels, read from source"
  - id: drift
    resource: /reference/dwyer-drift.md
    title: How far BEVOX has drifted from Dwyer
---

# Fracture and detection, as he built them — design

**Date:** 2026-09-30
**Status:** awaiting review

# Why this exists

Two of BEVOX's inventions were built because his design was unknown. It is known
now: his 2D prototype `rigid_pixels` is public, and both inventions turn out to
be worse than what they replaced.

Both changes are **replacements, not additions.** Neither adds a feature.

# Part 1 — Fracture: a size-scaled impulse, not a closing speed

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

# Part 2 — Detection: substepped in time, not widened in space

## What is here now, and why it is wrong

`MAX_SPEED` caps a body at 256 voxels a second so that speculative contacts can
stand in for continuous detection. Flori wants it gone: physics should be limited
by drag, not by a clamp.

It cannot simply be deleted. The detection margin is the body's travel, and the
lookup is `(2 * ceil(margin + 0.5) + 1)` cubed per corner voxel — so cost grows
as the **cube** of speed. One impulse at 10,000 voxels a second asks for
31 million lookups per corner and the tick never returns.

**He has no cap, and does not need one**, because he does not widen the search.

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

**Part 1 first.** It is smaller, it fixes a symptom Flori can see, and it does
not touch detection. Part 2 changes the cost model of the hottest CPU path in
the physics and wants its own measurement.

Not in scope: the stress scene (drift F4), the `velocity_iterations` knob his
solver has and BEVOX lacks, and anything in the renderer. All three are recorded
in [the ledger](../../reference/dwyer-drift.md) and wait their turn.

# Provenance

Both designs are read from `rigid_pixels`, which carries **no licence** — no
`LICENSE` file and no `license` field — so it is all rights reserved however
public it is. This spec describes mechanisms and constants so that both parts can
be implemented without reference to his source. Nothing is to be copied.
