---
type: Measurement
title: 'The frame budget, and where it goes'
description: 'Sixteen bodies in view with shadows take 22.6 ms against a 16.7 ms frame; Flori chose to keep both rather than lower the cap.'
tags: [performance, rigid-bodies, shadows, gpu]
generated: { by: claude-opus-5/claude-code, at: 2026-09-24T00:00:00Z }
sources:
  - id: shadows
    resource: /superpowers/plans/2026-09-18-bevox-body-shadows.md
    title: Body Shadows Implementation Plan
  - id: ao
    resource: /superpowers/plans/2026-09-24-bevox-ambient-occlusion.md
    title: Ambient Occlusion Implementation Plan
---

# Where the frame stands

GTX 1650, 1280x720, bench camera. The target is 16.7 ms.

| Scene | Wall | Note |
|---|---|---|
| static world, no bodies | ~11.8-12.4 ms | the march itself |
| 16 bodies in view, no shadows | 17.81 ms | already over |
| 16 bodies in view, with shadows | **22.64 ms** | 22.88 on the GPU |

Body shadows cost **+4.85 ms** for sixteen bodies in view (88,171 pixels
shadowed), and nothing at all for bodies the shadow rays miss — bodies behind
the camera came out at -0.23 ms, thanks to the per-caster world bounding sphere.

Ambient occlusion, added later, measured **inside its own drift** on both a
body-free scene and sixteen in view: -0.15/+0.08 and +0.28/+0.34 ms against
drift of 0.12/0.37 and 0.16/0.32. The honest claim is under half a millisecond
and not separable from noise — not "free".

# The decision

Sixteen bodies in view go over the frame and Flori chose that knowingly
(2026-09-18), over lowering the cap to about six, putting shadows behind a
toggle, or optimising before shipping them. Both `BODY_SHADOWS` and `AO` are in
`march_flags::DEFAULT`.

So the standing position is: **the worst case is over budget by design.** The
bench's worst case is a wall of cubes filling the view; a scene with sixteen
bodies scattered is cheaper. The next optimisation, if one is wanted, is aimed
at the per-pixel body cost — see [the body cap](body-cap.md).
