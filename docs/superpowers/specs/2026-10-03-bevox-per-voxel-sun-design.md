---
type: Design Spec
title: 'Sun visibility per voxel, not per pixel'
description: "The shadow ray is a pure function of (voxel, face) and costs 8 ms of a 26 ms frame at 1080p. Compute it once per voxel face and read it back per pixel, bit-identically."
tags: [render, performance, lighting, gpu]
generated: { by: claude-opus-5/claude-code, at: 2026-10-03T00:00:00Z }
sources:
  - id: drift
    resource: /reference/dwyer-drift.md
    title: How far BEVOX has drifted from Dwyer
  - id: steps
    resource: /concepts/nine-steps-a-ray.md
    title: 'Nine steps a ray, so the walk is not the cost'
  - id: cliff
    resource: /concepts/gpu-codegen-cliff.md
    title: The GPU codegen cliff
---

# Sun visibility per voxel, not per pixel — design

**Date:** 2026-10-03
**Status:** awaiting review

# The measurement

Measured 2026-10-03 by `what_the_sun_shadow_ray_costs`, a shader A/B interleaved
in one invocation, two runs at each resolution:

| | with the shadow march | without | cost | share |
|---|---|---|---|---|
| 1280x720 | 13.5–14.2 | — | **4.67 / 4.82 ms** | 33.3% / 33.9% |
| **1920x1080** | 25.9–26.3 | — | **7.98 / 8.36 ms** | 30.9% / 31.9% |

**A third of the frame.** The target is 60 fps at 1920x1080, which is 16.7 ms
against 26.28 today — a **1.57x** speedup wanted. Removing this cost outright
leaves ~18.3 ms, so F3 alone does not reach the target but takes what remains
from 1.57x to about **1.10x**, which is the difference between a subsystem and
some tuning.

# Why it is redundant, exactly

`shadow_origin` in `march.wgsl` returns

```text
voxel_centre(hit) + face_normal * 0.75
```

Both terms are per-voxel-per-face: `voxel_centre` is the voxel that was hit, and
`face_normal` is axis-aligned, one of six. The direction is `view.sun_direction`,
constant across the frame.

**So the shadow ray is a pure function of `(voxel, face)`**, and every pixel
covering the same voxel face marches an identical ray to an identical answer. At
1080p a voxel covers many pixels. The engine already *decided* to make sun
visibility a per-voxel property — the comment on `shadow_origin` says so: "a whole
voxel face is lit or shadowed together" — and then collects it per pixel.

**This makes the change bit-identical by construction**, which is the strongest
property available to an optimisation here and the reason it is worth doing before
anything speculative.

# Design

Four dispatches where there are two. The beam prepass is untouched.

```text
  1  BEAM          unchanged
  2  PRIMARY       march to the hit; insert (voxel, face) into the store;
                   write the slot index to a per-pixel buffer. No shading.
  3  SUN           one invocation per occupied slot: march one shadow ray,
                   write one bit.
  4  COMPOSITE     per pixel: read the slot, read its bit, shade as today.
```

## The key

`(voxel position, face)`. At `MAX_EXTENT` 4096 a position is 12 bits an axis and a
face is 3 bits, so **39 bits — a `u64` key with room spare.** No hashing of
coordinates into a smaller space, so no false sharing between distinct voxels.

## The store

An **open-addressed table in a storage buffer**, linear probing, inserted with
atomic compare-exchange. Capacity is fixed and sized from the pixel count, since
the number of distinct visible voxel faces cannot exceed the number of pixels and
in practice is far below it.

**Stamped, not cleared.** Each entry carries a frame id; an entry whose id is not
this frame's is free. That avoids a clear pass over the whole table every frame,
which would give back part of what this buys.

## Overflow

A full table is **a correctness-preserving fallback, not a failure**: a pixel
whose insert fails marches its own shadow ray exactly as today. The image is
unchanged and only the saving degrades. This is gated, because a fallback that is
never exercised is not known to work.

## Bodies are out of scope

A body hit takes the existing per-pixel path. Keying a body voxel needs the body
id as well, bodies are parked behind the physics work, and including them would
double the design for a case that is currently switched off. The spec says so
rather than leaving it implied.

# The risk, named first

**The codegen cliff.** This project has lost **48% of a frame** to a loop that
never ran, and **0.58 ms** to *removing* code from a branch a body-free scene never
enters. This change splits the main pass in two and adds two dispatches — the
largest shader restructure the engine has had.

So: the spec is **abandoned if the measurement does not hold**, and the
measurement is interleaved A/B/A in one invocation at both resolutions. The
8 ms above is a ceiling on the win, not a prediction of it: the store's own cost —
the atomic inserts, the extra dispatches, the per-pixel slot read — comes out of
it, and what remains is unknown until it is built.

**The second risk is the scattered sun pass.** Dispatching over the whole table
would waste most invocations on free slots. The occupied slots want compacting
into a work list, which is itself a pass, and whether that is cheaper than a
sparse dispatch is a measurement rather than a choice.

# Gates

| Gate | What it proves |
|---|---|
| The image is **bit-identical** to the current one, every pixel, both resolutions | The whole premise. Sun visibility is already per-voxel, so sharing it may not move a pixel. A changed pixel is a defect, not a trade-off |
| A full table still renders correctly | The overflow fallback works. Proven by shrinking the capacity until it overflows, not by argument |
| A scene where voxels cover ~1 pixel each is **not slower** | The worst case. Distant geometry has no redundancy to collect, so this measures the store's own overhead against nothing |
| `what_the_sun_shadow_ray_costs` re-run | What was actually collected of the 8 ms |
| The body path is unchanged | Bodies take the old route; a body-free scene and a body scene both stay bit-identical |

Breaks required: force every insert to fail (the image must stay correct and the
frame must return to today's cost); seed the store with a stale frame id (every
lookup misses, and the image must still be correct).

# What this does not do

- **It does not reach 60 fps at 1080p on its own.** It takes the needed speedup
  from 1.57x to about 1.10x.
- **It does not touch the traversal.** A primary ray takes nine steps; there is
  nothing there.
- **It does not build a path tracer, a denoiser, or temporal accumulation.** The
  store exists for this optimisation alone, which is why it is simpler than the
  structure that usually carries it: no history, no reprojection, no accumulation.
- **It does not help a scene with no sun.** The cost it removes is the shadow
  march, and a scene already in shadow pays little of it.
