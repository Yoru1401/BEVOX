# Ambient Occlusion Implementation Plan

**Goal:** Creases and corners darken, so a body sitting on the floor reads as sitting on it, following Dwyer's devlog #15.

**Decision (Flori, 2026-09-24):** his fullness grid, not per-voxel corner AO and not SSAO.

**His method, from the devlog transcript:**
- The world is divided into 16³ cubes; each stores how full it is.
- The fullness at a point is the trilinear blend between neighbouring cube centres.
- A flat surface sits at 50% full, an inner corner at about 75%. Above half, the excess is the darkening.
- One sample per voxel, so the shading stays per voxel, the way the rest of this engine shades.

## Decisions I am taking, with reasons

| Choice | Taken | Why |
|---|---|---|
| Cube size | 16 voxels | His. Gives shading that reaches about 8 voxels. Our distance field's 8-voxel cells are a separate grid with a different job. |
| Where it is sampled | The hit voxel's centre, in world space | One value per voxel, as he does, which keeps the voxelized look our shadows already have. |
| What it darkens | The ambient term only, not sunlight | That is what ambient occlusion is. Sun shadows are already handled. |
| Bodies | Sample the world's grid at the body voxel's world centre | A body in a crevice darkens. A body does not occlude itself; that would need a grid per body, and can come later if it shows. |
| Edits | Recount every cube the edit touched, from the tree | Simple and always right. An edit touches a handful of cubes. |
| Flag | `AO`, joining `DEFAULT` only if the bench allows | Same rule every optimisation and feature here has followed. |

## Tasks

1. **The grid, in `bevox_core`.** `Fullness::build(tree)`, one byte per 16³ cube, a byte being solid voxels scaled to 0..255; `recount(tree, lo, hi)` for an edit, returning the dirty range. Gates: a solid cube reads full and an empty one reads empty; a build matches a brute-force count on random trees; a recount after an edit matches a fresh build. Breaks: counting a uniform node as one voxel; a recount that misses the cubes on the edit's far edge.
2. **Upload.** A storage buffer beside the distance field, whole on a rebuild and by range on an edit, as the field does.
3. **Shader.** `fullness_at(point)`: trilinear blend of the eight surrounding cube values. `ao = clamp((fullness - 0.5) * 2, 0, 1)`, applied to the ambient term at the hit voxel's centre, behind `FLAG_AO`. Gates: the GPU's AO matches a CPU reference per pixel; with the flag off every entry point is bit-identical; a scene with a flat floor shows no AO on the open floor and does show it in a corner. Breaks: sampling at the hit point rather than the voxel centre; nearest instead of blended.
4. **Measure and record.** A/B/A of the flag on the bench scene, and with sixteen bodies. Then spec, memory, and this plan's Measurements.

## Measurements

GTX 1650, 1280x720, bench camera, extent 1024, 2026-09-24. A/B/A interleaved in
one session, `AO` against the same flags without it, wall / GPU:

| Scene | A/B/A | Drift | Pixels darkened |
|---|---|---|---|
| 0 bodies | -0.15 / +0.08 ms | 0.12 / 0.37 | 13,871 |
| 16 bodies in view | +0.28 / +0.34 ms | 0.16 / 0.32 | 8,495 |

Both readings sit inside their own drift, so the honest claim is *under half a
millisecond, not separable from noise* -- not "free". The frame with AO on is
13.87 ms with no bodies and 24.57 ms with sixteen, the same as without.

The codegen cliff was checked the way `bevox-gpu-driver-cliff` says to: the
shader before this change against the shader after, both with `AO` off, in the
same session. 0 bodies +0.08 / +0.10 ms, 16 bodies -0.69 / -0.04 ms, drift
0.17 / 0.22 and 0.44 / 0.07. No cliff.

`AO` joins `DEFAULT` on that.

## What changed during execution

- **The fullness grid shares the distance field's buffer.** wgpu's default limit
  is 8 storage buffers per compute stage and the shader was already at 8. The
  fullness bytes are packed behind the field's words in one buffer and
  `ao_params.x` says at which word, so nothing about the field's own indexing
  changed.
- **`voxel_centre` came out of `shadow_origin`.** Both need the hit voxel's
  centre in the world, and for a body that means going through the world hit
  point rather than the voxel index -- `shadow_origin`'s comment says why, and
  that reason is a measured codegen cliff, so the route is not optional.
- **Erasing now stages fullness cells.** An erase needs no distance-field update
  (a stale field only under-estimates), but it does empty cubes, and AO left
  stale would darken what is now open. `erasing_stages_fullness_but_no_field_cells`
  holds both halves of that.
- **The distance-field parity gate carries `AO` on both sides.** It compared
  `DEFAULT` against `NONE`; with AO in `DEFAULT` that would have been an AO
  comparison, not a skip comparison. The reference runs with `AO` now, which
  also proves the skip reaches the same voxel centres the unskipped scan does.

## Breaks that failed as required

| Break | Gate that caught it |
|---|---|
| A uniform node counts as one voxel | `a_build_matches_a_brute_force_count` |
| A recount misses the edit's far edge | `a_recount_matches_a_fresh_build` |
| Sampled at the hit point, not the voxel centre | `ambient_occlusion_matches_the_cpu`, 4155 pixels |
| Nearest cell, not blended | `ambient_occlusion_matches_the_cpu`, 3894 pixels |
