---
type: Decision
title: 'Only terrain debris merges back'
description: 'A body merges into the world only if detachment cut it out of the terrain, it has slept three seconds, and the camera cannot see it.'
tags: [physics, merging, sleeping, dwyer]
generated: { by: claude-opus-5/claude-code, at: 2026-09-24T00:00:00Z }
sources:
  - id: sleep
    resource: /superpowers/plans/2026-09-19-bevox-sleep-and-merge.md
    title: Sleeping and Merging Debris Implementation Plan
---

# The rule

A body goes back into the world only when all of these hold:

- **`from_terrain`** — detachment cut it loose, or it split off something that
  did. A body the app spawned outright never merges, however long it sleeps.
  This was Flori's call (2026-09-24), against Dwyer's devlog #13, which merges
  any settled body.
- **Asleep for `SLEEP_AFTER + MERGE_AFTER`** — about three seconds of stillness
  past the sleep threshold.
- **Out of view** — its world bounding sphere outside the frustum.
- **Not named by a joint, and not held by the mouse.**

# Why out of view is not a nicety

A body at an angle is not on the world's grid. Merging snaps each voxel to the
world cell under its centre, so **the shape can change by up to a voxel**. Doing
that on screen is a visible pop; doing it out of sight is free. Two voxels that
snap to one cell keep the first, and a voxel past the world's edge is dropped.

# What sleeping is worth

Sixteen bodies resting on the floor, medians of 1000 ticks over three runs:
**0.0039 ms asleep against 0.93 ms awake**, about 240 times less. Merging then
gives the slot back entirely, which matters because of
[the body cap](body-cap.md).

# The part that bites

Merging writes solid voxels into the world wherever the body came to rest —
never where the brush last was. That is one of the two paths that must tell the
coarse grids what changed: see [no safe stale direction](stale-direction.md).
