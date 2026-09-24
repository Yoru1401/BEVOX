---
type: Post-mortem
title: 'Padding a joint block at its own scale'
description: 'A joint block padded with the identity destroys the matrix inverse in f32, because a body k is about 1e-5; pad with trace(k)/3 instead.'
tags: [physics, joints, solver, numerics]
generated: { by: claude-opus-5/claude-code, at: 2026-09-24T00:00:00Z }
sources:
  - id: joints
    resource: /superpowers/plans/2026-09-18-bevox-physics-dwyer-joints.md
    title: "Dwyer's Joints Implementation Plan"
---

# The rule

`joint::block` solves one part's 1-3 active rows at once. The inactive rows must
be padded so the matrix is invertible, and **the padding is scaled to the
block's own magnitude**, never to 1:

```rust
let scale = (k.x_axis.x + k.y_axis.y + k.z_axis.z) / 3.0;
let padded = rows * k * rows + (Mat3::IDENTITY - rows) * scale;
```

# Why

A body's `k` is about `1e-5`. A matrix mixing `1e-5` with `1` loses its small
eigenvalues in f32: `padded · padded⁻¹` came out nowhere near the identity, with
entries as large as 14.

# What it looked like

A Free + Axis joint between two bodies grew its carried impulse about fivefold
per tick, until the speed cap stopped the bodies. Nothing about the symptom
pointed at conditioning.

Padding at `trace(k)/3` leaves the answer unchanged and brought the residual to
7e-8. The momentum gate caught it — see [deliberate breaks](deliberate-breaks.md)
for why that gate was strong enough to.
