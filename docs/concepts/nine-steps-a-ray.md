---
type: Measurement
title: 'Nine steps a ray, so the walk is not the cost'
description: 'A primary ray takes a mean of 9 traversal steps on the bench scene, so the static march is bound by what a step costs and not by how many there are; workgroup size is within drift at 16 and worse at 32.'
tags: [performance, gpu, traversal, dwyer]
generated: { by: claude-opus-5/claude-code, at: 2026-09-30T00:00:00Z }
sources:
  - id: budget
    resource: /concepts/frame-budget.md
    title: 'The frame budget, and where it goes'
  - id: drift
    resource: /reference/dwyer-drift.md
    title: How far BEVOX has drifted from Dwyer
---

# The number

`the_static_march_is_measured`, GTX 1650, 1280x720, bench camera, `DEFAULT`.

**A primary ray takes a mean of 9.0 traversal steps**, read from the
`march_steps` view's own output. 1.40% of rays saturate its 64-step scale, so
the true mean is a little above 9 — not far above it.

Nine. Against a `MAX_STEPS` of 4096 in the shader.

# What that rules out

The static march is **13-15 ms of a 16.7 ms frame**, and 921,600 rays at 9 steps
is 8.3 million steps. Spread over a 1650's 896 cores at 1.665 GHz, that is
**thousands of core-cycles per step** — orders of magnitude more than a step's
arithmetic costs.

So the marcher is **not step-bound.** Dwyer's four accelerations — the 64-child
mask, DDA inside a brick, the compile-time direction masks and the beam prepass —
have already reduced the walk to almost nothing. There is no traversal cleverness
left to buy, and **any further work aimed at reducing steps is aimed at 9 of
them.**

What the remaining time is instead: memory latency on dependent reads, the
per-pixel setup every ray pays before it walks (the 3x3 beam seed is nine reads,
then the distance-field skip), the sun shadow ray, and ambient occlusion's eight
fullness reads at the hit. **None of those are counted in the nine**, because
`hit.steps` is the primary walk only.

# Workgroup size: measured, and not the lever

`WORKGROUP` had been 8 since the shader was written, never measured. The shader
is read from disk, so all three variants were compiled and timed in one
invocation, medians of three rounds of seven, with every variant asserted
pixel-identical:

| | GPU ms | spread | against 8x8 |
|---|---|---|---|
| 8x8, 64 invocations | 15.40 | 1.43 | — |
| 16x16, 256 invocations | 14.97 | 0.63 | −0.43 |
| 32x32, 1024 invocations | 17.84 | 0.37 | **+2.45** |

- **16x16 is inside the drift.** −0.43 ms against spreads of 0.63 and 1.43 is not
  a result, and must not be reported as one.
- **32x32 is a real regression**, +2.45 ms against a spread of 0.37.
- So **8 stays**, and the question is closed rather than open. 256 invocations is
  exactly the WebGPU baseline's ceiling, which is part of why this was never
  tried; `device_limits` asks for 1024 now, so 32x32 was reachable and turned out
  to be worse.

# What this says about the gap to Dwyer

His reference point is **7 ms for a Teardown castle on a 1660 Ti**, primary and
shadow ray (devlog 17). The bench scene here is 13-15 ms on a 1650.

**And he states a resolution, once, in a video description**: devlog 22 was
captured *"in real-time 1080p on my GTX 1660 TI"*, eleven months later, with
textures, foliage, transparency and volumetrics added since. So the comparison
is **1920x1080 against this bench's 1280x720** — 2.25x the rays. That widens the
gap rather than explaining it, and it is the figure to beat, not the 7 ms.

That gap is **not the traversal**, because the traversal takes nine steps. The
candidates left are the hardware (a 1650 is roughly a third down on a 1660 Ti),
the scene — the bench camera is deliberately the worst case, close to geometry
looking along the floor between columns, where his castle is not — and the work
BEVOX does per pixel that he did not: ambient occlusion, and a body-composition
path that runs even with no bodies.

**So "be more like Dwyer's traversal" cannot close it.** Anyone reaching for that
conclusion should read this number first.
