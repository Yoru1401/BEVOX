---
type: Measurement
title: 'The impulse threshold separates a crush from a lean by ten per cent'
description: 'Fracture now thresholds the accumulated contact impulse, which is what sees a slow crush. Measured: a three-cube glass stack carries 331,306 at rest, a grab presses with 365,906, and the crush load saturates in mass while the resting load grows with the stack -- so past four cubes the ordering inverts and no single strength can serve both.'
tags: [physics, fracture, dwyer, measurement, open-defect]
generated: { by: claude-opus-5/claude-code, at: 2026-10-02T00:00:00Z }
sources:
  - id: plan
    resource: /superpowers/plans/2026-10-02-bevox-fracture-on-impulse.md
    title: Fracture on a size-scaled impulse
  - id: fracture
    resource: /superpowers/plans/2026-09-24-bevox-fracture.md
    title: Fracture Implementation Plan
---

# The headline

**The blow is the accumulated contact impulse, not the closing speed.** A
closing speed cannot see a crush: the mouse grab is a joint with a
1.962e8 force limit driving toward a *velocity* goal, so a grabbed body leaning
on something presses with up to that force while the contact holds both
surfaces still. The approach speed is then **under 0.011 voxels a second**
however hard the press, and a player could lean a rock through a window without
marking it. `a_slow_crush_breaks_what_it_presses` is that gate.

Dwyer's argument against a *force* is kept intact — a collision resolves inside
one tick, so the force it reports scales with the tick rate and the impulse it
exchanges does not. The blow is never divided by a mass either: a contact
belongs to whichever body the scene lists first, so a quotient would read one
collision two ways.

# What the units cost, measured 2026-10-02

An impulse grows with the mass a contact holds up, and warm starting seeds each
tick's impulse from the last, so **a settled stack carries a large impulse for
standing still**. Measured on the fixture glass (density 1000) in
`resting_weight_breaks_nothing`'s scene, as the peak normal impulse any contact
carried over 1200 ticks:

| Stack of 4-voxel cubes | Resting load |
| --- | --- |
| 1 | 126,129 |
| 2 | 246,227 |
| 3 | **331,306** |
| 4 | 356,751 |
| 6 | 391,195 |

Against that, a grab pressing a cube straight down into the floor, target
creeping 0.002 voxels a tick:

| Pressed body | Mass | Crush load |
| --- | --- | --- |
| 2-voxel cube | 8,000 | 588,636 |
| 4-voxel cube | 64,000 | **365,906** |
| 8-voxel cube | 512,000 | 343,399 |
| 10-voxel cube | 1,000,000 | 395,677 |

**The crush load saturates — it is near-flat in the pressed body's mass, and
flat in how long and how far the target is driven** (0.002 a tick for 400 ticks
and for 1500 give the identical 365,906). The press settles into a penetration
equilibrium instead of running up to its force limit, so the contact never
carries more than about 4e5 however much force is behind it.

# The open defect

**The two bound each other at about 1.1x, and past four cubes the ordering
inverts.** 391,195 for a six-high stack is already above the 365,906 a grab can
press with, so **no single strength both survives a six-high glass stack and
gives way under a slow crush.** `GLASS_STRENGTH` is 350,000 — the geometric
middle of the binding pair 331,306 and 365,906, five per cent of headroom
either way — and that fits only the fixtures the suite uses.

Two things would widen it, and neither is done: raise the impulse a contact can
carry before the penetration equilibrium holds it, or read the load a contact
has carried *steadily* apart from the load it just took. The second is the real
answer, because the steady load is exactly what resting weight is and exactly
what a crush is not.

`DEFAULT_STRENGTH` has no such problem: 2.8e6 is where a 4-voxel stone cube
(mass 64,000) arrives at about 150 voxels a second, measured from peaks of
2.48e6 at 120 and 3.43e6 at 200, which keeps the meaning the closing-speed
threshold had and leaves a factor of eight over the resting load.

# What an impulse means for the player

`strength` is no longer a speed a material survives. **A bigger body breaks the
same material at a lower speed**, because the impulse it carries scales with its
mass. That is what an impulse threshold is, not a flaw in it, but every number
on a palette now has to be read against a body.
