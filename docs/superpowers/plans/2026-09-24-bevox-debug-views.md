---
type: Implementation Plan
title: 'Debug Views'
description: 'Ten views on the function keys -- normals, depth, UV, ids, unlit, shadows, occlusion and a ray-step heatmap -- each its own entry point, measured against the lit frame.'
tags: [rendering, debugging, gpu, tooling]
generated: { by: claude-opus-5/claude-code, at: 2026-09-24T00:00:00Z }
---

# Debug Views Implementation Plan

**Goal:** See what the marcher sees. Ten views, chosen from the function keys while the app runs: the lit scene and nine ways of looking behind it -- normal, depth, UV, material id, voxel id, unlit, shadows only, AO only, and a ray-step heatmap.

**Architecture:** Each view is its own compute entry point in `march.wgsl`, and the render world queues one pipeline per view and dispatches whichever the app selected. Five entry points already exist, written for the parity tests. Nothing branches per pixel in the shipped path: the lit view runs exactly the shader it runs today.

**Spec:** `docs/superpowers/specs/2026-09-15-bevox-rigid-bodies-design.md` (Rendering)

## Global Constraints

- No new dependencies. `bevox_core` gains nothing; this is all render-side and app-side.
- Every gate is proven by a deliberate break.
- Any change to `march.wgsl` is re-benched A/B/A in one session against the shader before it. See `docs/concepts/gpu-codegen-cliff.md`.
- The shipped frame is not allowed to get slower for a debug feature. If the heatmap costs more than drift, it is reverted, and that is reported rather than absorbed.
- Flori runs the app; I do not.

## The views

| Key | View | Entry point | Status |
|---|---|---|---|
| F1 | the lit scene | `march` | exists |
| F2 | normal | `march_normal` | exists, doc comment to update |
| F3 | depth | `march_depth` | new |
| F4 | UV within the hit face | `march_uv` | new |
| F5 | material id | `march_identity` | exists |
| F6 | voxel id | `march_voxel_id` | exists |
| F7 | unlit | `march_unlit` | new |
| F8 | shadows only | `march_shadow` | exists |
| F9 | ambient occlusion | `march_ao` | exists |
| F10 | ray steps | `march_steps` | new, and the risky one |

**Depth** is linear in the world's own units: `t / extent`, clamped, written to all three channels. Not a reversed-Z or a projection depth — the marcher has a distance along the ray and nothing else, and a scene-relative ramp is the one that stays readable when the volume changes size.

**UV** is where on the hit voxel's face the ray landed, in `[0, 1)`: take the world hit point, drop the component along `hit.face_normal`, and write the other two to red and green. It is the view that shows a face-mapping or a coordinate-flip bug at a glance.

**Ray steps** needs a counter incremented inside the traversal, which is where this engine's two codegen cliffs both landed. It is built the simple way — one counter in the shared traversal — and measured. If the lit frame moves by more than the drift, it is reverted and Flori is told the heatmap costs the frame, with the alternative (a duplicated traversal loop used only by the heatmap) named but not built.

## Tasks

### Task 1: The three safe new entry points

**Files:**
- Modify: `crates/bevox_render/assets/shaders/march.wgsl`
- Test: `crates/bevox_render/tests/gpu_parity.rs`

Add `march_depth`, `march_uv`, `march_unlit` beside the existing debug entry points, each following the shape those already use: `primary_hit`, then a `textureStore` with the hit flag in alpha.

Gates, one test covering all three against a CPU reference on the existing corner scene:
- depth at a pixel equals `t / extent` to within a byte, and a nearer surface reads darker than a farther one;
- UV lies in `[0, 1)` and varies across a single face, rather than being constant per voxel;
- unlit equals the palette colour exactly where the CPU says there is a hit, and the sky elsewhere.

Breaks: write `t` without dividing by the extent (depth gate must fail); use the voxel's corner rather than the hit point for UV (UV gate must fail, since it goes constant per voxel); multiply unlit by the diffuse term (unlit gate must fail).

### Task 2: Selecting a view in the render world

**Files:**
- Create: `crates/bevox_render/src/debug.rs`
- Modify: `crates/bevox_render/src/pipeline.rs`, `crates/bevox_render/src/lib.rs`
- Test: in `debug.rs` and `pipeline.rs`

`DebugView` is an enum with one variant per view and `Lit` as its default, a `Resource` in the app world extracted to the render world, carrying the entry-point name and a display name. `MarchPipeline` holds one queued pipeline per view; the dispatch reads the extracted resource and picks.

Gates: every `DebugView` variant names an entry point that exists in the shader source, read from the asset file at test time — the check that catches a typo or a renamed entry point; and the pipeline list has exactly one entry per variant. Break: rename one entry point in the table.

### Task 3: The keys, in the app

**Files:**
- Modify: `crates/bevox/src/main.rs`
- Test: in `main.rs`

A system maps F1-F10 to the views and logs the name on a change. Gate: pressing each key leaves the resource on the matching view, and a frame with no key pressed leaves it alone. Break: map two keys to the same view.

### Task 4: The heatmap, and the measurement that decides it

**Files:**
- Modify: `crates/bevox_render/assets/shaders/march.wgsl`
- Modify: `crates/bevox_render/tests/gpu_bench.rs`

Add the step counter and `march_steps`, then A/B/A the **lit** view against the shader before this task, with no bodies and with sixteen, in one session. Two outcomes, both acceptable:
- inside drift: keep it, record the numbers;
- outside drift: revert the counter and the entry point, keep the other nine views, and record what it would have cost.

Gate: the heatmap is non-zero wherever there is a hit, and a grazing ray through a sparse region counts more steps than one that hits the first voxel it meets.

### Task 5: Record

Spec's Rendering section, this plan's Measurements, the bundle index and log, and a note in `docs/concepts/gpu-codegen-cliff.md` if task 4 produces a third cliff.

## Measurements

GTX 1650, 1280x720, bench camera, extent 1024, 2026-09-24. Three shader sources
A/B/A in one session, every row rendering the **lit** view: the shader before
the debug views, the shader with all ten, and the shader with every view except
the heatmap -- so a cost could be attributed rather than guessed at.

| Scene | before vs all views | before vs no heatmap | lit pixels changed |
|---|---|---|---|
| 0 bodies | -0.23 / +0.04 ms (drift 0.42 / 0.01) | +0.00 / +0.08 ms (drift 0.00 / 0.01) | 0 |
| 16 in view | +0.07 / +0.04 ms (drift 0.09 / 0.05) | +0.06 / +0.01 ms (drift 0.00 / 0.03) | 0 |

Read honestly: everything is **under 0.1 ms**, and the variant with *less* code
measured slower than the variant with more on two rows, which cannot be a real
cost. So the debug views are not separable from noise at this resolution, and
**no pixel of the lit view moved** -- which is the stronger half of the result,
since the `Hit` struct grew a field the lit path carries.

**The heatmap is kept.** The fear behind the question was that a counter inside
the traversal would trip a third codegen cliff; it did not, and it turned out
the loop was already counting its steps for `MAX_STEPS` -- the change only
carries that number out.

## What changed during execution

- **The step counter already existed.** `traverse_at` counts steps to enforce
  `MAX_STEPS`. Nothing new runs in the loop; `Hit` gained a field and
  `compose_bodies` sums the count across the static world and every body the ray
  was marched against, so a pixel showing terrain still reports what the bodies
  in front of it cost.
- **The heatmap is grey, not a colour ramp.** A ramp is easier on the eye and is
  not monotone per channel, so no test could hold it to counting what it claims
  to count. Bright is expensive, and that is enough to find the camera angle
  that hurts.
- **Its scale is 64 steps, against a cap of 4096.** At the cap the view is black
  everywhere, because the distance field and the beam prepass seed a primary ray
  to just short of what it hits.
- **Three gates were rejected before one held**, and the rejects are worth
  recording because each was a plausible-sounding thing that is not true here:
  - *steps should rise with distance* -- they do not. Traversal cost follows the
    geometry along the ray, not its length. 19.9 against 20.8 between the
    nearest and farthest quarters of the image.
  - *steps should fall when the accelerators are on* -- barely. They change
    where a ray starts, not the descent that follows: 17.2 against 17.2 on the
    corner scene.
  - *steps should match `MarchStats`* -- they cannot. The reference marcher
    counts a step per child considered, the shader one per stack frame: 56
    against 3 on the same ray. Both are honest about different things.

  What the gate asserts instead is what a cost view must do to be worth opening:
  never zero where a ray hit, a real spread across the image, and **not a
  function of depth** -- rays stopping at the same distance must still be seen to
  cost different amounts, which is the whole reason to look at it.
- **`MarchPipeline` holds a pipeline per view** rather than one, and
  `DebugView::index` is the single place that orders them. A selection that
  collapsed to one index would show every view as the lit scene, which reads as
  the keys not working, so it has its own gate.
- **A view still compiling falls back to the lit pipeline**, so pressing a key
  never blanks the window for a frame.

## Breaks that failed as required

| Break | Gate that caught it |
|---|---|
| Depth in raw world units, not against the extent | `the_data_debug_views_match_the_cpu` |
| UV taken at the voxel's corner rather than the hit point | same gate, on the spread check |
| UV built from the wrong two axes of the face | same gate |
| Unlit multiplied by the diffuse term | same gate |
| A view naming an entry point the shader does not declare | `the_views_name_entry_points_that_exist` |
| Two views sharing a key | `every_view_is_distinct` |
| Every view indexing to the same pipeline | `a_view_indexes_to_itself` |
| The key system reselecting every frame | `the_function_keys_select_every_view` |
| The heatmap writing a constant | `the_ray_step_view_counts_steps` |
| The heatmap writing depth instead | same gate, on the same-depth check |
| A uniform node reporting no steps | same gate, on the non-zero check |
