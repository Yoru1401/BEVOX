---
type: Rule
title: 'The GPU codegen cliff'
description: 'Code the shader almost never runs can still cost tens of percent, so every march.wgsl edit is re-benched A/B/A in one session.'
tags: [gpu, wgsl, performance, benchmarking]
generated: { by: claude-opus-5/claude-code, at: 2026-09-24T00:00:00Z }
sources:
  - id: culling
    resource: /superpowers/plans/2026-09-17-bevox-body-culling.md
    title: Body Culling Implementation Plan
  - id: shadows
    resource: /superpowers/plans/2026-09-18-bevox-body-shadows.md
    title: Body Shadows Implementation Plan
---

# The rule

Any edit to `march.wgsl` is measured A/B/A against the shader before it, in one
session, even when the edit cannot change a pixel and even when it sits in a
branch the benchmark never takes. The driver compiles the whole module; what it
does with register pressure and loop structure is not visible from the source.

# What it cost twice

**48%, from a loop that never ran** (at `6b47b61`, fixed at `8b59104`). An
exact-tie fix added a small loop to `touched_child`. The scan's pixel output was
identical across the commit, yet the scan went from about 39 to 58 ms wall and
`DEFAULT` from 12.7 to 18.8. Deleting only that loop took the scan's GPU time
from 57.1 back to 38.1 ms, against 37.7 before the change. Unrolling it was
worse still (65.4), and so was splitting `traverse_at` by flag (62.2). The fix
was to drop the loop entirely: the grazed cells form a box, so `touched_child`
builds a 64-bit child mask and takes its lowest set bit, integer-only.[^culling]

**+0.58 ms of 11.8, from *removing* code** (body shadows). Three different edits
to a body branch that a body-free scene never enters each cost the same +0.58 ms
— including replacing the branch with a one-line stub. The trigger was dropping
the `primary_ray` call from it. The shipped form reaches the voxel centre
through the world hit point, which keeps that call: +0.04 ms with no bodies,
-0.04 with sixteen.[^shadows]

# How to measure it

The GPU bench reads `march.wgsl` at run time, so one temporary test can A/B/A
two shader *source files* in the same session — the only way to compare before
and after without cross-run drift, which on this machine is routinely larger
than the effect under test. Ambient occlusion was checked this way and showed
no cliff.

Never compare against a number from an earlier invocation. Interleave A, B, A in
one command and report the drift beside the difference; a difference smaller
than its drift is not a result. See [the frame budget](frame-budget.md) for what
the numbers are spent on.
