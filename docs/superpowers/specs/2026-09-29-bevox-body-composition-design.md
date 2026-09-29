---
type: Design Spec
title: 'Body composition in ray order'
description: 'Cut the primary march half of a body cost by giving the marched table the bounding-sphere reject the shadow path already has, then visiting bodies in ray order against the running nearest hit.'
tags: [render, performance, bodies, dwyer, gpu]
generated: { by: claude-opus-5/claude-code, at: 2026-09-29T00:00:00Z }
sources:
  - id: drift
    resource: /reference/dwyer-drift.md
    title: How far BEVOX has drifted from Dwyer
  - id: cap
    resource: /concepts/body-cap.md
    title: 'The body cap, and what a body costs'
---

# Body composition in ray order — design

**Date:** 2026-09-29
**Status:** awaiting review

# Goal

Cut the **primary march** half of a visible body's cost. Measured 2026-09-29: a
body costs **0.629 ms**, split 0.316 ms primary against 0.314 ms shadow. This
addresses the primary half only; the shadow half is a separate piece of work
(divergences 2 and 3 in the drift ledger).

Success is a measured reduction in the primary half with **no pixel changed**.

# What is already here

This matters more than the goal, because the gap to Dwyer's devlog 2 is much
narrower than the drift ledger first claimed, and three of the four things one
would reach for are already built.

| Already built | Where |
|---|---|
| A body march **starts at its bounding box**, not at the camera | `traverse_at` seeds its first frame at `max(root_slab.t_enter, t_start)` |
| The search **tightens as hits are found**, so a body behind the nearest hit exits on its first step | `compose_bodies`'s `limit`, and `t_cur > max_dist` in the loop |
| A body is **skipped for pixels outside its screen rectangle**, before its entry is read or the ray transformed | `FLAG_BODY_RECT` |
| A shadow ray **rejects a body by its world bounding sphere** before reading or transforming it | `shadowed`, measured at 20 ms a frame for sixteen bodies behind the camera |

So the claim "`1 + N` unaccelerated marches from the ray origin" in the drift
ledger overstates it. The ledger is corrected alongside this spec.

# The two things that are missing

## 1. The primary path has no bounding-sphere reject

`GpuBody::bound` carries a body's world bounding sphere, and its own doc
comment says:

> Filled in only for the shadow casters (`pipeline::shadow_casters`); the
> marched table leaves it zero.

So the optimisation this project built, measured at 20 ms, and documented is
applied to **one of its two ray types**. A primary ray inside a body's screen
rectangle but nowhere near the body itself still pays a 160-byte `GpuBody` read,
two `mat4` transforms, a `ray_box`, a stack frame and a loop entry before
`traverse_at` rejects it.

A screen rectangle is a 2D footprint: it says the body is somewhere along this
pixel's ray, not that the ray passes through it. For a rotated body in a
64-voxel volume the rectangle is a loose bound on a tight object.

This is not a divergence from Dwyer. It is an asymmetry inside BEVOX, and it is
the cheaper half of this work.

## 2. Bodies are visited in table order, not ray order

`compose_bodies` iterates the table as the cull built it. `limit` therefore
tightens late: with bodies in arbitrary order, the number of traversals that
actually narrow the answer is the number of running minima — about `ln(N)`, so
roughly 3 of 16 — and the rest are traversed only to be discarded.

Dwyer's devlog 2 is exactly this problem. His first renderer sorted the objects
a ray might hit, which "capped a ray at four or eight objects because GPU code
cannot allocate", and he replaced it with stepping: take the shortest step, and
if another object offers a shorter one, continue in that object instead.

# Design

Two steps, measured separately, with the second conditional on the first.

## Step 1: give the marched table its sphere, and reject with it

- `pipeline::march_table` (or wherever the marched table is built) fills `bound`
  from `cull::world_bound`, exactly as `shadow_casters` already does. The
  function exists; only the caller differs.
- `compose_bodies` gains the same sphere reject `shadowed` has, in the same
  place: after the screen rectangle, before the `GpuBody` read.
- **No new flag.** This is a rejection that cannot change a pixel, and the
  project already carries `CULL_BODIES` and `BODY_RECT` as flags because each
  had to be measured against its absence. This one is measured by reverting the
  two lines, not by a runtime branch, because a branch in the per-pixel path is
  itself a codegen risk.

Expected effect: removes the per-body floor for rays that pass near nothing. On
the bench scene, where sixteen bodies are spread across the view and rectangles
overlap little, this is most of what a covered pixel wastes.

## Step 2: visit in ray order, and stop when the rest cannot win

Only if step 1 leaves the primary half worth attacking.

- Per pixel, compute each surviving body's sphere entry distance `t_i`.
- Visit bodies by increasing `t_i`, selected with a min-scan against a 16-bit
  visited mask — no sort, no allocation, `MAX_BODIES` is 16.
- Skip any body whose `t_i >= limit`, and **break the loop** when the smallest
  remaining `t_i >= limit`, which is the part that makes cost sublinear in the
  body count.

# The risk, named up front

**The codegen cliff.** This project has lost 48% of a frame to a loop that never
ran, and 0.58 ms to *removing* code from a branch a body-free scene never enters.
Step 2 adds a selection loop to the innermost per-pixel path, which is precisely
the shape that has bitten twice.

So: step 2 is measured A/B/A against step 1 in one session, on a body-free scene
as well as a sixteen-body one, and is **abandoned if it does not pay**. Step 1 is
measured the same way. Neither is kept on the argument that it ought to be
faster.

# Gates

| Gate | What it proves |
|---|---|
| The parity suite passes unchanged | No pixel moved. A rejection that changes a pixel is a defect, not a trade-off |
| The zero-body scene is bit-identical to `d893ce8` | The composition path costs a body-free scene nothing |
| A body whose sphere the ray misses but whose rectangle contains the pixel is still **not** drawn when it should be | The sphere is not rejecting geometry it should keep — the break this needs is a radius shrunk by a voxel, which must make a body disappear |
| `where_a_body_s_cost_goes` re-run | The primary half moved, and by how much |

The third gate is the one that matters. The shadow path widens its radius —
`bound.w * 1.001 + 1e-3` — precisely because a ray grazing the box's corner lies
on the sphere and must not be rounded out. The primary path must do the same, and
the gate has to prove the widening is load-bearing.

# Measurements

Against `d893ce8`, the same bench in the same form:

| | GPU ms |
|---|---|
| static world, `DEFAULT` | 13.23 |
| sixteen bodies | 23.30 |
| sixteen bodies, no body shadows | 17.83 |
| primary per body | 0.316 |
| shadow per body | 0.314 |

# What this does not do

- **It does not touch the static march.** That is 13.23 ms of a 16.7 ms frame,
  79% of the budget before a body exists, and it is the real ceiling on the body
  count at this camera. Nothing here moves it.
- **It does not raise `MAX_BODIES`.** The cap is changed when a measurement says
  what it should be, not before.
- **It does not address the shadow half.** Sun visibility is a property of a
  voxel rather than a pixel, and collecting that needs a per-voxel store BEVOX
  does not have. Divergences 2 and 3, next.
- **It is not literal interleaved stepping.** Dwyer steps whichever volume offers
  the nearest next *voxel*; this orders whole volumes by entry and terminates
  early. True interleaving needs a live traversal state per volume — seventeen
  descent stacks in registers — which is a register-pressure bet this project's
  own history says to make last, not first. If step 2 pays and is still not
  enough, that is the next thing to consider, with these numbers in hand.

# Why the order is this way round

Step 1 mirrors code that already exists, already passes its own gates, and was
already measured at 20 ms on the other ray type. Step 2 is new machinery in the
hottest loop in the engine. Doing the second before the first would risk the
cliff to win something the first may already have taken.
