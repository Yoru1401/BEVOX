---
type: Decision
title: 'Both coarse grids ride in one storage buffer'
description: 'The shader was already at wgpu default limit of 8 storage buffers per compute stage, so fullness is packed behind the distance field with a word offset.'
tags: [gpu, wgpu, ambient-occlusion, distance-field]
generated: { by: claude-opus-5/claude-code, at: 2026-09-24T00:00:00Z }
sources:
  - id: ao
    resource: /superpowers/plans/2026-09-24-bevox-ambient-occlusion.md
    title: Ambient Occlusion Implementation Plan
---

# The constraint

wgpu's default limit is **8 storage buffers per compute stage**, and `march.wgsl`
was already at 8 when ambient occlusion needed a fullness grid.

# What was done

The fullness bytes are packed *behind* the distance field's words in the same
buffer. `ao_params.x` holds the word the grid starts at; nothing about the
field's own indexing changed. `pack_grids(field, fullness)` is the single
spelling of that layout, and `field_words(field)` of the offset.

# The trap that came with it

The shader indexes fullness with `field_params.x` — the *distance field's* edge.
The two agree only because both are `(extent / CELL_VOXELS).max(1)`. Nothing
enforced that, and no picture would look wrong enough to notice if it parted, so
`pack_grids` now asserts the two edges are equal.

# What to do next time

Another grid goes in the same buffer behind another offset, not in a ninth
binding. Raising the limit is a device-capability request that would narrow the
hardware this runs on.
