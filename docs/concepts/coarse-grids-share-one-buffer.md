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

**Superseded 2026-09-29. This advice was wrong, and it was wrong for an
instructive reason.**

It said: another grid goes in the same buffer behind another offset, not in a
ninth binding, because raising the limit is a device-capability request that
would narrow the hardware this runs on.

Measured instead of assumed (`what_the_adapter_allows` in
`tests/gpu_bench.rs`), a GTX 1650 over Vulkan:

| limit | the engine was asking for | the adapter allows |
|---|---|---|
| `max_storage_buffers_per_shader_stage` | 8 | **524,288** |
| `max_sampled_textures_per_shader_stage` | 16 | 524,288 |
| `max_storage_buffer_binding_size` | 128 MB | **2,047 MB** |

**Eight was never the hardware's limit. It is wgpu's default, which is the
WebGPU spec baseline** — the set chosen so a shader runs everywhere, browsers
included. The core spec excludes WebAssembly *permanently* and says native
desktop may be assumed everywhere, so this project had been paying a browser's
ceiling for nothing, and then written a rule telling all future work to keep
paying it.

`pipeline::device_limits` now asks for what the engine needs plus a margin — not
the adapter's maximum, which would encode one GTX 1650's capabilities as the
engine's requirement.

**Two things follow.**

- **A grid that wants filtering does not want a storage buffer at all.** It wants
  a sampled 3D texture and a linear sampler, which is a *different budget* and
  one this engine uses none of. That is exactly Dwyer's "one hardware-filtered
  texture read" (his devlog 15), which is the reason he calls his ambient
  occlusion cheap, and it is what packing behind an offset gave up.
- **A latent bug came with it.** `VOXEL_BUDGET_BYTES` is 512 MB and
  `within_budget` checks only the *sum*, while a single storage binding capped at
  128 MB. A large enough scene passed the budget check and would have failed at
  buffer creation. `the_voxel_budget_fits_in_one_storage_binding` gates it now.

**The rule this leaves, which is the general one:** a default is not a
constraint. Before a limit shapes a design, ask the adapter what it actually
allows — see [an inference is not an observation](an-inference-is-not-an-observation.md)
for the same failure one level down.

# What was still right

The packing itself works and is measured, and the assertion that `pack_grids`
makes — that the two grids agree on their edge count — earns its place either
way. What was wrong was the reason, and the advice built on it.
