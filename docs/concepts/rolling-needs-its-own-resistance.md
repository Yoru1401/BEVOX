---
type: Post-mortem
title: 'Sliding friction cannot stop a roll'
description: 'A lone voxel is a sphere, and a rolling sphere barely slips, so friction never slows it; it rolled for ever and never slept until contacts got rolling resistance.'
tags: [physics, solver, sleeping, materials]
generated: { by: claude-opus-5/claude-code, at: 2026-09-24T00:00:00Z }
sources:
  - id: dwyer
    resource: /reference/dwyer-devlogs.md
    title: "Douglas Dwyer's voxel engine, devlog by devlog"
---

# The symptom

Lone voxels and one-voxel-wide towers never fell asleep, and so never merged
back into the terrain and never stopped costing physics time. Measured: a lone
voxel nudged along a flat floor at 4 voxels a second was **still moving at 3.3
after eight seconds**, spinning at 6.6 rad/s, with `still_for` never once
leaving zero.

# The cause

A voxel's collision shape comes from how many axes it has solid neighbours on
(`physics::classify`, after Dwyer's devlogs 20 and 26):

| Neighbours on both sides along | Shape |
|---|---|
| no axis | **`Corner` — a sphere of radius 0.5** |
| one axis | `Edge` — a cylinder along it |
| two axes | `Face` — flat |
| all three | `Interior` — never touched |

A lone voxel has no neighbours at all, so **it is a sphere**. A one-wide tower
is cylinders standing on a sphere.

And a rolling sphere has almost no slip where it touches the ground. Sliding
friction opposes relative surface velocity, and in a true roll that velocity is
near zero — so friction finds nothing to act against and takes nothing away.
The body rolls until something else stops it, and nothing else does.

# The fix

`ROLLING`, a contact force of its own: an angular impulse opposing the spin,
limited by what the contact is actually pressing with —
`ROLLING * friction * normal_impulse * RADIUS` — and never more than the spin
that is there, so it can bring a roll to a stop but never reverse one or add
energy. A lone voxel now sleeps after **1.0 s**, a one-wide tower after 3.1 s.

It is small (0.08) on purpose: it must not stop a cube tipping onto its face,
which is the same angular motion at the same kind of contact.

# What the gate has to prove, and what nearly fooled it

That the resistance comes from **the contact**, not from a global angular
damper — a damper would pass every "does it stop" assertion and quietly drain
every thrown body's spin in mid-air.

The first attempt tested a voxel *rolling* on frictionless ice, expecting it to
keep rolling. It passed against a deliberately broken damper, because **with no
friction the voxel slides instead of turning**: there was no spin for either
version to damp. The gate now sets a voxel **spinning in place** on ice and
requires it to keep spinning. See [deliberate breaks](deliberate-breaks.md) —
this is the sixth blind gate this project has found, and the fix was a stronger
fixture, not a stronger threshold.

# The rule

**A shape that can roll needs a force that opposes rolling.** Friction is not
that force. And when a body will not settle, look at what shape the contact
thinks it is before looking at the sleep thresholds — see
[terrain-only merging](terrain-only-merging.md) for what depends on bodies
actually falling asleep.
