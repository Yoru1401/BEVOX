---
type: Map
title: 'bevox_render on one page'
description: 'The Bevy plugin: what each module owns, what one frame does from upload to sprite, the ten bindings the shader reads, and the flags that switch each optimisation.'
tags: [map, render, gpu, wgsl, reference]
generated: { by: claude-opus-5/claude-code, at: 2026-09-28T00:00:00Z }
sources:
  - id: spec
    resource: /superpowers/specs/2026-09-13-bevox-raymarcher-core-design.md
    title: BEVOX ray-marcher core — design
---

# bevox_render

A Bevy plugin that uploads the voxel scene and ray marches it in a compute
shader. Depends on `bevox_core`; **never on `bevox_physics`**, and a test on
this crate's own manifest keeps it honest — the renderer marches bodies and must
not need the simulation to do it.

Intended to be replaceable: Bevy's rendering internals move between releases, so
everything that would break lives here rather than in the core.

# Where it lives

| | |
|---|---|
| `upload` | Owns `VoxelScene` — the tree, its materials, the two coarse grids and the bodies — and stages what changed into the GPU buffers. Also where the brush enters. |
| `pipeline` | The bind group layout, one compute pipeline per debug view, and the dispatch that runs them. Holds the body cap and the memory budget. |
| `cull` | Drops bodies the camera cannot see, and gives the survivors a screen rectangle so a pixel outside it pays nothing. |
| `camera` | The fly camera: keys and mouse to a `Transform`. The clip-to-world matrix the shader takes rays from is **not** here — it is `upload::offset_from_clip`, beside the uniform that carries it. |
| `pick` | What is under the cursor, marched against the same tree the shader draws, so a click lands on what was seen. |
| `debug` | The ten views and the whole table behind them — entry point, name and key — so a view cannot be added in one place and forgotten in another. |

The shader itself is `assets/shaders/march.wgsl`, loaded at run time. The GPU
benchmark reads it from disk too, which is what lets two shader *source files*
be measured against each other in one session.

# One frame

```text
  MAIN WORLD, in `Update`
    1  stage whatever changed: dirty arena ranges, the lowered field cells,
       the recounted fullness cells                     -- a body that only
                                                           MOVED needs none of
                                                           this; its table entry
                                                           is rebuilt anyway

  RENDER WORLD, in `Prepare` -- past the extract boundary
    2  cull bodies the camera cannot see, and give the rest a screen rectangle
    3  build the uniform: camera, sun, extents, flags, body count

  BEAM PREPASS, one dispatch at 1/8 resolution per axis
    4  march one ray per coarse pixel and record how far it got, capped at the
       distance where a voxel could slip between two beams

  MAIN PASS, one dispatch at full resolution
    5  seed the ray at the MINIMUM of the 3x3 beam neighbourhood around it
       -- a pixel sits anywhere inside its beam's cell, so the geometry it is
          about to meet may have been seen by the beam on either side
    6  skip empty space the distance field proves clear
    7  march the static world; then each body in turn, keeping the nearest hit
    8  on a hit: the implicit normal, one shadow ray to the sun, and the
       ambient term darkened by the fullness around the voxel
    9  write the pixel

  PRESENT
   10  a window-sized Sprite under a Camera2d draws the storage texture
```

- **Per ray this is `1 + N` marches**, one per body, which is why the body count
  is capped and why most of a body's cost is paid per pixel.
- **Shadow casters are not the culled table.** A body just off screen can
  shadow what is on it, so casters are taken separately, each carrying a world
  bounding sphere a ray can reject before any transform.
- **Bit-identity is the rule for optimisations.** Every flag below must produce
  the same pixels off as on; one that changes a pixel is a defect, not a
  trade-off.

# What the shader reads

Ten bindings, of which **eight are storage buffers.** That used to be the
ceiling, because the engine asked for `WgpuLimits::default()` — the WebGPU spec
baseline, which allows eight. It is why the fullness grid rides packed in the
distance field's buffer, with `ao_params.x` as the word it starts at.

**Since 2026-09-29 the engine asks for sixteen** (`pipeline::device_limits`),
because eight was a browser's baseline and this project excludes WebAssembly
permanently — the adapter offers 524,288. A grid that wants *filtering* should
become a sampled 3D texture rather than a ninth storage buffer, which is a
different budget again and one nothing here uses yet. See
[both coarse grids ride in one storage buffer](../concepts/coarse-grids-share-one-buffer.md).

| # | Binding | Kind |
|---|---|---|
| 0 | camera, sun, extents, flags, body count | uniform |
| 1–2 | the node arena, and the leaf voxel bytes | storage |
| 3 | the palette | storage |
| 4 | the output image | storage texture |
| 5 | the direction mask table | storage |
| 6 | beam prepass distances | storage, read-write |
| 7 | the distance field, with the fullness grid behind it | storage |
| 8 | the body table | storage |
| 9 | body screen rectangles | storage |

# Flags

Each joins `DEFAULT` only once it has measured faster while staying
bit-identical — or, for a feature, once its cost has been accepted knowingly.

| Flag | What it switches |
|---|---|
| `DDA` | Stepping inside a brick by addition instead of ray-box tests. |
| `MASK_FILTER` | Rejecting a whole brick by ANDing its mask with the direction table. |
| `BEAM` | Seeding each ray from the coarse prepass. |
| `DISTANCE_FIELD` | Advancing a ray through space the field proves empty. |
| `BODIES` | Composing rigid bodies into the march at all. |
| `CULL_BODIES` | Leaving invisible bodies out of the table. **CPU only** — the shader never reads this bit. |
| `BODY_RECT` | Skipping a body for pixels outside its screen rectangle. |
| `BODY_SHADOWS` | Shadow rays test bodies as well as the world. |
| `AO` | Darkening the ambient term where the world is full around a voxel. |
| `NONE`, `DEFAULT` | Nothing, and everything above. |

# Constants

| | | |
|---|---|---|
| `MAX_BODIES` | 16 | Bodies the shader will march. Measured, not chosen — but the figure it was set from is stale: 0.265 ms a body in 2026-09-17, **0.629 ms re-measured on 2026-09-29** once shadows and AO had joined `DEFAULT`. **Resolution-dependent**, because most of a body's cost is per pixel. |
| `VOXEL_BUDGET_BYTES` | 512 MB | Voxel data on the GPU. Checked at upload; exceeding it is an error, never an allocation attempt. |
| `WORKGROUP` | 8 | Compute threads per axis. |
| `BEAM_SCALE` | 8 | Full-resolution pixels per beam sample, per axis. Must match the shader's own copy. |
| `BEAM_CAPACITY` | 518 400 | Coarse pixels the beam buffer holds — enough for 7680x4320, so it is sized once and never resized. |
| `MARCH_BINDING_COUNT` | 12 | Bindings the shader declares, and so the number the layout must contain. A gate, because a mismatch is not a compile error. |
| `STORAGE_BUFFERS_DECLARED` | 10 | How many of those are storage buffers — the one budget that has ever been binding. |
| `SHADER_PATH` | `shaders/march.wgsl` | Loaded at run time, which is what makes A/B/A of two shader sources possible. |
| `SUN_DIRECTION` | (0.4, 1, 0.25) | Where the sun is, shared by the renderer and the parity tests. |
| `SUN_SLOTS` | 2 097 152 | Slots in the per-voxel sun store, a power of two so a probe masks. More than 1080p has pixels, and a measured 118 423 distinct voxel faces — one slot each — fill 5.7% of it. |
| `SUN_STAMP_BITS` | 8 | Bits of a claim word the frame stamp takes; the other 24 are a tag cut from the key, so one compare-exchange settles both "free?" and "mine?" and an insert has no race to lose. `next_sun_stamp` cycles 1..255, never 0. |
| `SUN_PIXEL_CAPACITY` | 8 294 400 | Pixels the store's per-pixel slot region holds, 3840x2160. Sized for a window rather than resized with one, as the beam buffer is; a pixel past it writes nothing. |

# Read next

- [The frame budget](../concepts/frame-budget.md) — where the 16.7 ms goes, and
  why the worst case is over it by design.
- [The body cap](../concepts/body-cap.md) — what a body costs and how 16 was
  arrived at.
- [The GPU codegen cliff](../concepts/gpu-codegen-cliff.md) — **read this before
  editing `march.wgsl`.** Code the shader never runs has cost 48%.
