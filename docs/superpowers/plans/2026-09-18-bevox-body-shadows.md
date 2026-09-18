# Body Shadows Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. **Flori prefers inline execution for this project.**

**Goal:** Rigid bodies cast shadows: on the static world, on each other, and on themselves.

**Architecture:**
- A shadow ray tests the static world as now. If nothing there blocks it and `BODY_SHADOWS` is on, it then tests every body, stopping at the first hit.
- "Every body" means the shadow casters: the first `MAX_BODIES` placed bodies, uncompacted. They are never the culled table, because a body off screen can shadow what is on it.
- The casters sit in the second half of the existing body buffer, so no new binding is needed. The uniform's free `field_params.zw` carry their count and where they start.
- The primary path is untouched.

**Tech Stack:** Rust stable 1.96, Bevy 0.19.1, wgpu, WGSL. No new dependencies.

**Spec:** `docs/superpowers/specs/2026-09-15-bevox-rigid-bodies-design.md`, Rendering.

## Global Constraints

- Native desktop only. No new dependencies. `bevox_core` is untouched.
- **Physics stays off master.** Commit on `feat/rigid-body-physics`; push only when Flori asks.
- Verify with `cargo test --workspace`, output to a file, checking `$?`. The GPU tests run headless and skip without an adapter.
- Do not run the app; Flori tests it.
- Every correctness gate is proven by a deliberate break.
- **Any change to `march.wgsl`'s traversal is re-benched, A/B/A in one session**, because a driver codegen cliff once cost 48% with identical pixels.

## Decisions (Flori, 2026-09-18)

- **Approach A:** shadow rays test every body up to the cap, whether or not it is on screen.
- A `BODY_SHADOWS` flag, for A/B benching and bit-identity. It joins `DEFAULT` if the measurement allows. If 16 bodies go over the 16.7 ms frame, the fix is Flori's decision, made on the numbers.
- The bounding-sphere pre-test from the design is **not built unless the bench asks for it**: `traverse_at`'s slab test already rejects a ray that misses a body's box, and whether the transform is worth skipping is a measurement, not a guess.

## Facts that are not obvious from the code

- **The culled table is compacted.** `bodies_to_march` drops bodies the camera cannot see, so `bodies[i]` for `i < count` lists only visible bodies. A shadow ray reading it would lose every off-screen caster, which is exactly the case the cull comment in `cull.rs` warns about.
- **The body buffer is sized from the placed count.** With room for `capacity = placed.len().max(1)`, the table fills the front and the casters start at `capacity`, so the buffer is `2 × capacity`. A change in the number of bodies bumps the generation and rebuilds, so the per-frame rewrite always sees the capacity the buffer was built with.
- **Bodies already receive shadows.** `shadow_origin` handles a body hit. Only casting is new.

## Tasks

### Task 1: The caster list, on the CPU (the shader unchanged)
- `march_flags::BODY_SHADOWS = 128`, not yet in `DEFAULT`.
- `pipeline::shadow_casters(placed) -> &[GpuBody]` returns the first `MAX_BODIES`.
- `pipeline::body_buffer_contents(table, casters, capacity) -> Vec<GpuBody>` lays out the table, pads it to `capacity`, then appends the casters, padded to `2 × capacity`.
- `frame_uniform` also returns the casters, and sets `field_params = [edge, cell, casters.len(), capacity]`.
- Both paths of `prepare_march_buffers` write the casters. `BODY_BYTES` counts two `GpuBody`s per body.
- The test harness in `tests/common` lays its buffer out through the same two functions.
- **Gate:** `frame_uniform`'s casters include bodies the cull dropped; its count is capped at `MAX_BODIES`; its base is the capacity.
- **Break:** casters taken from the culled table must fail it.

### Task 2: The shader
- Add `shadowed(origin, dir, max_dist)`: `traverse_any` first, then under `FLAG_BODY_SHADOWS` loop over `bodies[base + i]` for `i < count`, transforming the ray as `compose_bodies` does, from `t = 0`, stopping at the first hit.
- `march` and `march_shadow` call it.
- **Gates**, in `gpu_parity.rs`:
  1. `bodies_cast_shadows_that_match_the_cpu`: an L-shaped, turned body floats over the floor and over a cube. Every pixel's shadow flag must match a CPU reference: the static world marched any-hit, or any body's `march_world` from the same origin toward the sun. Non-vacuous counts are required: floor pixels shadowed only by a body, pixels of one body shadowed by another, and pixels of the L shadowed by itself.
  2. `an_off_screen_body_shadows_what_is_on_screen`: the cull must drop the body, yet more pixels are shadowed than with no body at all.
  3. `body_shadows_off_leave_every_entry_bit_identical_to_no_bodies_casting`: with the flag off, the off-screen body's scene renders exactly as with no body.
  4. The existing body tests' CPU references now include body shadows, and the zero-body bit-identity test runs with the flag on.
- **Breaks:**
  - the body loop removed;
  - the casters read from the culled table (base 0);
  - a body's shadow origin pushed inside its surface (`−0.25`).

### Task 3: Measure, and decide `DEFAULT`
- In `gpu_bench.rs`, run A/B/A in one session, flag on against off: static world only (the codegen cliff), and 16 bodies at the bench camera.
- If the cost fits the 16.7 ms frame at 16 bodies, `BODY_SHADOWS` joins `DEFAULT`. If not, stop and report the numbers to Flori.
- Update the spec's Rendering section, memory, and this plan's Measurements.

## Measurements

`body_shadows_are_measured`: GTX 1650, 1280x720, bench camera, extent 1024. Each row is `BODY_SHADOWS` A/B/A against the same flags without it, in one session; the figures are what B adds over its bracketing A readings, wall / GPU, with drift in brackets.

| Scene | Before the sphere test | With it |
|---|---|---|
| no bodies | +0.00 (0.13) / −0.50 (0.03) | +0.04 (0.11) / −0.00 (0.03) |
| 16 bodies in view, 88,171 px shadowed | +27.29 (0.10) / +26.56 (0.96) | **+4.85 (0.03) / +5.31 (0.04)** |
| 16 bodies behind the camera, none shadowing | +20.22 (0.01) / +20.34 (0.19) | −0.23 (0.02) / −0.34 (0.51) |

- **The frame, with the sphere test:** sixteen bodies in view take 22.64 ms on the wall clock (22.88 on the GPU) against a 16.7 ms frame. Without shadows the same view took 17.81 ms in this session, already over the frame.
- **No codegen cliff:** the body-free row changes nothing.
- **The entry's growth from 144 to 160 bytes** did not move the primary march: 16 bodies in view without shadows read 17.81/17.91 ms before the change and 17.81/17.78 after, in the same session. This is not an A/B/A, since both builds cannot run in one binary.
- **The workspace:** 331 pass, 10 ignored, no warnings.

## What changed during execution

1. **The sphere test was built after all.** The plan held it back until the bench asked, and the bench did: sixteen bodies behind the camera, shadowing nothing, cost +20 ms a frame in rejection alone.
   - Each caster now carries its world bounding sphere, and the shader reads that one field before loading the entry.
   - A body with no voxels is left out of the casters.
   - The break (radius halved) fails the parity gate: 376 pixels disagree.
2. **`bodies_cast_shadows_that_match_the_cpu` looks from the shadow side (−X −Z).** From the planned camera the floor shadows lay behind the bodies casting them, and only 98 pixels of floor were shadowed by a body, under the 100 required. From the other side: 381 floor pixels, 259 pixels of one body shadowed by the other, and 73 self-shadowed pixels facing the sun; 0 mismatches.
3. **Sixteen bodies in view go over the frame: 22.6 ms against 16.7.** Flori chose to keep shadows on by default and the cap at sixteen (option 1 of 4), over lowering the cap to about six, a toggle, or further optimisation first. `BODY_SHADOWS` joined `DEFAULT`, and the numbers are recorded at `MAX_BODIES` and `DEFAULT`.
4. **Breaks, all reverted, each failing its gate:**
   - casters taken from the culled table (CPU test, and the off-screen test through the shader);
   - the body loop removed;
   - the shadow origin pushed inside the body's surface;
   - the sphere too small.
