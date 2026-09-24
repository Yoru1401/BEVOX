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

(Filled in during execution.)

## What changed during execution

(Filled in during execution.)
