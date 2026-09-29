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

# Re-measured 2026-09-29, and the number above is stale

`0.265 ms` describes a shader that no longer exists. `BODY_SHADOWS` joined
`DEFAULT` the day after this was measured and `AO` six days later, and nothing
re-measured the cap until now.

`where_a_body_s_cost_goes` in `tests/gpu_bench.rs`, same GPU, same camera, same
resolution, every configuration interleaved in one invocation, GPU medians of
three rounds of seven:

| | GPU ms |
|---|---|
| static world, `DEFAULT` | 13.23 |
| sixteen bodies, `DEFAULT` | **23.30** |
| sixteen bodies, `DEFAULT` without `BODY_SHADOWS` | 17.83 |

**A visible body costs 0.629 ms, not 0.265 — 2.4 times the figure the cap rests
on.** And it splits almost exactly in half:

- **primary march: 0.316 ms a body** (+5.05 ms for sixteen)
- **shadow-ray caster tests: 0.314 ms a body** (+5.02 ms for sixteen)

Two things follow that the old number hid.

**Neither half alone buys the frame back.** Removing all of the primary cost
leaves 18.25 ms; removing all of the shadow cost leaves 18.28. Both are over
16.7. Sixteen bodies in view needs both halves attacked, or a different target.

**The static march is 13.23 ms of a 16.7 ms frame on its own** — 79% of the
budget before a single body exists. That is the real ceiling on the body count at
this camera, and no amount of work on body composition moves it.

Read with care: the per-body figures at **one** and **four** bodies are inside
the noise (spreads of 1.0-3.0 ms against effects of 0.3), and the one-body shadow
delta came out negative, which is impossible and is drift. Only the sixteen-body
decomposition is above the noise floor. `MAX_BODIES` has not been changed on the
strength of this; what to do about it is
[the drift ledger's](../reference/dwyer-drift.md) divergences 1 to 3.
