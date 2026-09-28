---
type: Map
title: 'The physics constants'
description: 'Every tunable in bevox_physics, grouped by what it governs, with what each one is for.'
tags: [map, physics, tuning, reference]
generated: { by: claude-opus-5/claude-code, at: 2026-09-28T00:00:00Z }
sources:
  - id: map
    resource: /map/physics.md
    title: bevox_physics on one page
---

# The physics constants

Units are voxels and seconds. Grouped by what each one governs, because that is
how you arrive here: knowing the behaviour you want to change, not the name.

**How a body falls.** Gravity is an acceleration, so it skips the mass; a force
from gameplay does not.

| | | |
|---|---|---|
| `VOXEL_METRES` | 0.1 | How long a voxel is in metres. The dial for how heavy falling feels. |
| `GRAVITY` | −98.1 y | Voxels per second squared, from `9.81 / VOXEL_METRES`. |
| `TERMINAL_SPEED` | 200 | What a long fall settles at: 20 metres a second. |
| `DRAG` | ≈0.00245 | Quadratic drag, set so it balances gravity exactly at terminal speed. A fall eases into that speed instead of being clamped to it. |
| `MAX_SPEED` | 256 | The hard ceiling on linear speed plus the furthest voxel's swing. A safety net above terminal, not a speed limit: what reaches it was thrown or blasted, and being clamped beats tunnelling. |

**How a contact behaves.** `SUBSTEPS` sets `h = dt / SUBSTEPS` everywhere else.

| | | |
|---|---|---|
| `SUBSTEPS` | 4 | Velocity solves per tick. Contacts are found once and reused by all of them. |
| `contact::RADIUS` | 0.5 | The radius a voxel's corners and edges are rounded to. |
| `BASE_MARGIN` | 0.1 | How far ahead of itself a body looks for contacts even at rest, so one arrives before it is needed rather than after. |
| `SLOP` | 0.02 | Penetration left alone, in voxels, so a resting contact is not pushed out and dropped back every substep. |
| `BIAS` | 0.2 | The share of any penetration deeper than `SLOP` removed per substep, as velocity. |
| `MAX_PUSH` | 20 | A ceiling, in voxels a second, on what `BIAS` may push with. A body an edit buried floats out instead of being fired out. |
| `ROLLING` | 0.08 | Rolling resistance, as a share of the contact's friction. Small on purpose: it must stop a lone voxel rolling without stopping a cube tipping onto its face. |
| `RESTITUTION_SWEEPS` | 4 | Sweeps of the bounce pass. One is not enough — every contact of a flat landing would push the whole body to the bounce target by itself. |

**When a body stops costing anything.**

| | | |
|---|---|---|
| `SLEEP_SPEED` | 0.05 | Below this, in voxels a second, a body counts as still — measured at its centre of mass *and* at its furthest voxel, so spinning in place is not rest. |
| `SLEEP_AFTER` | 0.5 | Seconds of stillness before a group falls asleep. Groups sleep together or not at all. |
| `MERGE_AFTER` | 3.0 | Further seconds asleep before a body that came out of the terrain may go back into it. Long enough for a pile to settle. |

**What the mouse can do.** Capped absolutely, never scaled by mass, so a heavy
body sags and drags and the grab cannot force anything through terrain.

| | | |
|---|---|---|
| `GRAB_MAX_FORCE` | 1.962e8 | The weight of 2 000 voxels at density 1000. |
| `GRAB_MAX_TORQUE` | 3.924e8 | That force at two voxels. Enough to hold a cube level by its corner, weak enough that a grab far from a hinge swings a door instead of locking it. |

**What comes apart.**

| | | |
|---|---|---|
| `BUDGET` | 20 000 | Voxels a detachment walk visits before giving up and calling the piece grounded. A cut into a mountainside must not walk the mountain. |
| `fracture::REBOUND` | 0.6 | The share of a breaking contact's peak impulse handed back, so a body carries on through what it broke instead of stopping at a hole nothing went through. |

A material's own `density`, `friction`, `restitution` and `strength` are not
here: they are in `bevox_core::material`, per material, and are read per voxel
— so a body half on ice drags on one side only.

See [bevox_physics on one page](physics.md) for where each of these is used.
