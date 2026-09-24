---
type: Measurement
title: 'The body cap, and what a body costs'
description: 'MAX_BODIES is 16 because a visible body costs 0.265 ms at 1280x720 on a GTX 1650, down from 3.38 ms before the two rejections.'
tags: [rigid-bodies, performance, culling, gpu]
generated: { by: claude-opus-5/claude-code, at: 2026-09-24T00:00:00Z }
sources:
  - id: culling
    resource: /superpowers/plans/2026-09-17-bevox-body-culling.md
    title: Body Culling Implementation Plan
---

# The number

At 1280x720 on a GTX 1650, at the bench camera, cost per *visible* body as the
slope over 0, 1, 4 and 16 bodies, wall / GPU:

| Rejections | ms per body | % of the static march | Bodies that fit 16.7 ms |
|---|---|---|---|
| neither | 3.378 / 3.287 | 28.1 / 27.8 | 1.4 / 1.5 |
| cull only | 3.367 / 3.556 | 28.0 / 30.1 | 1.4 / 1.4 |
| rectangle only | 0.282 / 0.207 | 2.3 / 1.7 | 16.6 / 23.6 |
| **both** | **0.265 / 0.273** | 2.2 / 2.3 | 17.7 / 17.9 |

`MAX_BODIES` is **16**: sixteen visible bodies under both rejections marched in
16.33 ms wall and 16.12 ms GPU. The slope says 17.7 would fit, but sixteen is
the largest count actually run and the difference is under a tenth of a
millisecond of headroom.

# What the number does not say

These are dispatch timings at one camera and one resolution, not frame rates.
**The cap is resolution-dependent**, because most of a body's cost is paid per
pixel — which is also why the screen rectangle, not the frustum cull, is what
made bodies cheap. The cull earns its place only on bodies the camera cannot
see, where it removes them from the table entirely.

Since this was measured, body shadows were added on top: see
[the frame budget](frame-budget.md) for what sixteen bodies actually cost now.
