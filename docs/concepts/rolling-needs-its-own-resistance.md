---
type: Post-mortem
title: 'Sliding friction cannot stop a roll'
description: "Dwyer's rounded corners and edges make a lone voxel a sphere, and a rolling sphere barely slips, so friction never slows it; it rolled for ever and never slept until contacts got rolling resistance."
tags: [physics, solver, sleeping, materials, dwyer]
generated: { by: claude-opus-5/claude-code, at: 2026-09-24T00:00:00Z }
sources:
  - id: shapes
    resource: /reference/dwyer-devlogs.md
    title: "Douglas Dwyer's voxel engine, devlog by devlog — #20 and #26, for the collision shapes this follows from"
  - id: deferred
    resource: /superpowers/plans/2026-09-18-bevox-physics-friction-restitution.md
    title: 'Friction and Restitution Implementation Plan — where "a ball would roll forever; nothing in the scene is a ball yet" was written down'
---

> **Where this sits against Dwyer.** Rolling resistance appears in none of the
> thirty devlogs, and it is still not a departure from him: it is what his own
> collision shapes force. Taking his rounded corners and edges means a voxel
> with no neighbours is a sphere and a one-wide column is cylinders on a
> sphere, so his geometry produces bodies that roll. He never had to name the
> force because his debris is trees and rubble, many voxels wide; a voxel
> engine that detaches single voxels does.

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
is cylinders standing on a sphere. Nothing about that is an approximation to be
tightened: it is the exact shape Dwyer's scheme asks for, and rounding is what
makes the contact rotation-invariant in the first place (his devlog 26, where
axis-aligned boxes gave wrong normals and jitter). The roll comes with the fix
for the jitter.

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
that force. And **the shapes decide what can roll, so read them before deciding
nothing can**: two plans deferred this with "a ball would roll forever; nothing
in the scene is a ball yet", written while every lone voxel in the engine was
already a ball. And when a body will not settle, look at what shape the contact
thinks it is before looking at the sleep thresholds — see
[terrain-only merging](terrain-only-merging.md) for what depends on bodies
actually falling asleep.
