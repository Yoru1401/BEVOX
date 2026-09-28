---
type: Map
title: 'bevox_physics on one page'
description: 'Where every part of the simulation lives, what one tick does in order, and what a voxel is when something touches it.'
tags: [map, physics, rigid-bodies, reference]
generated: { by: claude-opus-5/claude-code, at: 2026-09-28T00:00:00Z }
sources:
  - id: spec
    resource: /superpowers/specs/2026-09-15-bevox-rigid-bodies-design.md
    title: BEVOX Rigid Bodies — Design
---

# bevox_physics

Voxel rigid bodies, after Douglas Dwyer's engine. Units are **voxels and
seconds** throughout. Depends on `bevox_core`; nothing depends on it, and a
manifest gate keeps it that way — `bevox_render` marches bodies without ever
asking the simulation anything.

A body's state lives on `Body` in `bevox_core`, because the renderer needs it.
This crate is what computes and advances it.

# Where it lives

**Five modules run on every tick.** They are the simulation proper.

|            |                                                                                                                                 |
| ---------- | ------------------------------------------------------------------------------------------------------------------------------- |
| `solver`   | Owns the tick. Solves contacts, applies the bounce, raises fracture events, and hands them back to the caller. Start at `step`. |
| `contact`  | Finds contacts: a body against the world, and bodies against each other. Holds the sphere, plane and cylinder tests.            |
| `classify` | Decides which voxels can be touched at all, and in what shape. Runs on a body's own voxels, and on the world's on demand.       |
| `joint`    | Sixteen joint types, their friction and motors, and the mouse grab. Solved before contacts, so contacts have the last word.     |
| `sleep`    | Wakes anything that something awake could move, then puts still groups to sleep. A sleeper costs the tick nothing.              |

**Five run when something changes the voxels**, not on a clock.

| | |
|---|---|
| `mass` | Mass, centre of mass and inertia, summed per voxel from material density. Re-run on every edit, never only at creation. |
| `detach` | Walks the tree for terrain an edit cut loose from the ground, and turns each piece into a body. |
| `sculpt` | The brush on a body. Painting grows it; an erase that cuts it in two leaves two bodies. |
| `fracture` | Decides whether a blow was too hard for the material, and where the cracks fall. Cuts nothing itself. |
| `merge` | Writes a settled body back into the world and frees its slot. |

# One tick

The shape that matters is the nesting: contacts are found **once** and reused by
every substep, which is what makes the solver temporal Gauss-Seidel rather than
a plain iterative one.

```text
step(dt):

  ONCE, before anything is solved
    1  drop any body wholly below the world
    2  cap every speed, and record how far each body can travel
    3  wake whatever an awake body could reach          -- to a fixed point, so
                                                           nothing is ever
                                                           solved against a sleeper
    4  DETECT every contact                             -- once; substeps reuse these
    5  read each contact's approach speed                  the bounce and the blow
                                                           are both written against it

  FOUR TIMES, with h = dt/4
    6  gravity, drag, and whatever gameplay pushed with
    7  re-apply what each contact and joint carried     -- warm starting
    8  solve joints, with the drift correction
    9  solve contacts: normal, then friction, then rolling
   10  INTEGRATE position and orientation
   11  solve again with no drift correction             -- the relax pass; it takes
                                                           back the velocity the
                                                           correction added

  ONCE, after the substeps
   12  store what each contact carried, for next tick
   13  apply restitution, swept RESTITUTION_SWEEPS times
   14  raise a fracture event for every contact that was too hard
   15  count how long each body has been still, and sleep the still groups
```

Three choices here are load-bearing and easy to undo by accident:

- **Position is integrated only after velocities are solved** (step 10 after 9).
  A resting body therefore has no velocity left when it moves, so it does not
  move at all. Correcting position in a pass of its own jitters at float
  precision, which is the bug Dwyer's devlog 26 is about.
- **The relax pass carries no bias** (step 11), after Box2D v3. Pushing a body
  out of the floor would otherwise leave it with the push as real velocity, and
  it would bounce.
- **The mouse grab is left out of the relax pass.** Its target moves, and the
  bias is how that motion reaches the body; relaxing it would stop the body dead
  every substep and letting go would throw nothing.

# What a voxel is, on contact

A voxel is a cube **rounded at its corners and edges and flat across its
faces** — Dwyer's devlog 26, after pure spheres let bodies sink into each
other's gaps and axis-aligned boxes gave wrong normals under rotation.

Which it is depends on how many axes have a solid neighbour on **both** sides:

| Axes enclosed | Shape | Geometry | Tested against |
|---|---|---|---|
| 0 | `Corner` | sphere, radius `RADIUS` | any voxel of the other side |
| 1 | `Edge(axis)` | cylinder along that axis | other edges only |
| 2 | `Face{axis}` | flat slab on the exposed axis | nothing; corners cover it |
| 3 | `Interior` | none | never touched first |

> A voxel alone in space is enclosed on no axis, so it is a **sphere**, and a
> one-voxel-wide tower is cylinders standing on one. That is not an
> approximation to tighten — it is what Dwyer's scheme asks for, and it is why
> contacts need [a force that opposes rolling](../concepts/rolling-needs-its-own-resistance.md).

# Tuning

Every constant, grouped by what it governs, is on its own page:
[the physics constants](physics-constants.md). It is a page you consult; this
one you read.

# Read next

- [The spec](../superpowers/specs/2026-09-15-bevox-rigid-bodies-design.md) — why
  any of this is the way it is. Its Provenance section separates what Dwyer
  showed from what this design filled in.
- [Dwyer devlog by devlog](../reference/dwyer-devlogs.md) — 20, 26, 28 and 30
  are the physics ones.
- Learned the hard way:
  [every gate is proven by a deliberate break](../concepts/deliberate-breaks.md),
  [padding a joint block at its own scale](../concepts/joint-block-conditioning.md),
  [only terrain debris merges back](../concepts/terrain-only-merging.md).
