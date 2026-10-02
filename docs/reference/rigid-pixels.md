---
type: Reference
title: "Dwyer's rigid_pixels, read from source"
description: "His open-source 2D prototype: how detection avoids tunnelling with no speed cap, how fracture thresholds an impulse scaled by object size, and the two solver knobs BEVOX collapsed to one."
tags: [dwyer, physics, reference, fracture, detection]
generated: { by: claude-opus-5/claude-code, at: 2026-09-30T00:00:00Z }
sources:
  - id: repo
    resource: https://github.com/DouglasDwyer/rigid_pixels
    title: DouglasDwyer/rigid_pixels
    author: human:douglas-dwyer
  - id: devlogs
    resource: /reference/dwyer-devlogs.md
    title: "Douglas Dwyer's voxel engine, devlog by devlog"
---

# What this is

The 2D engine from his devlog 25 — the prototype he built **before** the 3D
physics of devlog 26, so that detectors and solvers could be swapped and
compared for stability. Read at `master`, 3,116 lines of Rust across eleven
files.

It answers questions the devlogs leave open, because it is the same design
before it went into a closed-source 3D engine.

> **Licence: there is none.** No `LICENSE` file and no `license` field in
> `Cargo.toml`, so the default is all rights reserved however public the
> repository is. **Read it for design, never copy its source.** Everything below
> is a description of what it does — facts and mechanisms — written to be
> implementable without reference to his expression of them.

# Collision detection: no speed cap, substepped detection

`DetectorKind` is a runtime choice between `Naive` — detect once at the start of
the tick — and `Speculative`, whose doc comment is exact about the scope:

> Split *detection only* into multiple substeps based upon the unaffected motion
> of the objects.

**There is no velocity clamp anywhere in the engine.** Tunnelling is bounded by a
*distance* instead:

| | |
|---|---|
| `TUNNEL_THRESHOLD_DISTANCE` | `0.4` — "the theoretical distance an object may move before tunneling occurs" |
| substeps for a pair | `ceil(dt / (0.4 / max_speed))`, at least 1 |
| `max_speed` | relative linear speed + each body's `radius x angular`, each plus its force or torque integrated over the tick |

So the number of detection passes for a pair rises **linearly** with how fast
the pair is closing, and the margin stays fixed. Substep placement within the
tick is itself a choice — `Equidistant`, `Floor`, `Midpoint` or `Ceil`.

`include_external_forces` decides whether the predicted trajectory accounts for
gravity and applied force, or only current velocity.

**Why this matters against BEVOX.** BEVOX bounds tunnelling by widening the
*lookup radius* with speed: `reach = ceil(margin + 0.5)` and the search is
`(2 * reach + 1)` cubed, so cost grows as the **cube** of speed and a velocity
cap becomes necessary to keep the tick finite. His grows **linearly** and needs
no cap. Same goal, and the cost curve is what differs.

# Fracture: an impulse, scaled by how big the object is

Inside the solver, once per constraint after the substeps:

- The tested quantity is the contact's **accumulated normal impulse**, which
  confirms devlog 28. His `breaking_impulse` is an **`f32`**.
- Each side's threshold is its own material's breaking impulse **scaled by
  `sqrt(min(area, 7))`**, where the constant is named for "the surface area over
  which force spreads out" and commented as making *small* objects fracture
  quicker. Capped at 7, so it is a small-object correction rather than general
  stress scaling — above that size the threshold is flat.
- **Both sides are tested separately**, each against its own scaled threshold, so
  one body breaking while the other does not is the designed behaviour.
- Each side gets its own `strength_ratio` = its impulse over its own threshold,
  discretised to tiers 1-3, which selects the crack pattern. The two sides of one
  collision therefore legitimately differ.
- The impulse handed back is **exactly the excess** over the smaller of the two
  thresholds — not a fixed fraction. A contact carries what it takes to break the
  weaker side and returns the rest.

# The solver: BEVOX's shape, with two more knobs

`Solver::SequentialImpulse`, described in its own comment as "Catto-style" — the
same lineage BEVOX credits for its relax pass.

```text
substep_time = dt / substeps
constraints built once, from joints then contacts
repeat substeps times:
    integrate external forces
    repeat velocity_iterations times:  solve velocity WITH stabilisation
    integrate velocities
    repeat relaxation_iterations times: solve velocity WITHOUT it
cache constraint forces          -- warm starting, a config flag
solve fracture, per constraint
```

Four tunables: `baumgarte` (the bias BEVOX calls `BIAS`),
`velocity_iterations`, `relaxation_iterations`, `substeps`, plus
`warm_starting` as a flag.

**BEVOX runs exactly one velocity pass and one relax pass per substep.** His are
counts. That is an undocumented difference and a cheap tuning axis: more velocity
iterations per substep is the standard lever for stacking stability, which is
precisely the failure BEVOX hit on 2026-09-28 when a 240:1 mass ratio diverged.

# What else the source settles

- `GeometryKind` is a runtime choice of `Sphere` — every voxel a sphere — or
  `Surface`, where "when two voxels touch, a smooth surface will extend between
  them". That is devlog 26's rounded-corners-with-full-faces, in 2D.
- A retired `pgs.old.rs` sits beside `si.rs`, which is the modular
  swap-and-compare devlog 25 describes, kept in the tree.
- Normals are clamped at corner-corner contacts so that a normal pointing into a
  voxel's neighbour is made perpendicular to the edge instead — the
  rotation-invariance problem of devlog 26, handled explicitly.

# Where this lands in the drift ledger

- **C1**, the size term on the fracture threshold: it is calibrated at
  cap-sized bodies rather than at one voxel, which his rule is not. The
  threshold itself *is* his — an accumulated contact impulse. BEVOX read a
  closing speed until 2026-10-02, and that was the deviation this row used to
  record; it is closed. See
  [the respec](../superpowers/specs/2026-09-30-bevox-fracture-and-detection-design.md).
- **E2**, the speed ceiling: he has none, and his substepped detection is what
  replaces it.
- **F4**, the stress scene: his velocity-iteration count is a lever BEVOX does
  not have.
