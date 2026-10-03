---
type: Implementation Plan
title: 'Sun visibility per voxel'
description: "Build the per-voxel store and measure its own cost before splitting the march: insert first and read nothing, then add the sun pass and collect the win."
tags: [render, performance, lighting, gpu]
generated: { by: claude-opus-5/claude-code, at: 2026-10-03T00:00:00Z }
sources:
  - id: spec
    resource: /superpowers/specs/2026-10-03-bevox-per-voxel-sun-design.md
    title: 'Sun visibility per voxel, not per pixel'
  - id: cliff
    resource: /concepts/gpu-codegen-cliff.md
    title: The GPU codegen cliff
---

# Sun visibility per voxel — implementation plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Stop recomputing the same sun answer once per pixel. `shadow_origin` is
`voxel_centre(hit) + face_normal * 0.75` and the sun direction is constant, so the
shadow ray is a pure function of `(voxel, face)` — measured at **8 ms of a 26 ms
frame at 1080p**.

**Architecture:** A per-voxel-face store in two storage buffers. The primary pass
inserts and records a slot per pixel; a sun pass marches one ray per occupied
slot; the composite reads the bit back. Built in the order that measures the
store's own cost **before** the march is restructured around it.

**Tech Stack:** Rust 2024, WGSL, wgpu through Bevy 0.19.1. `cargo test -p bevox_render`.

**Spec:** [Sun visibility per voxel, not per pixel](../specs/2026-10-03-bevox-per-voxel-sun-design.md)

## Global Constraints

- **The image is bit-identical at every step.** Sun visibility is already a
  per-voxel property, so sharing it may not move a pixel. A changed pixel is a
  defect, not a trade-off, and the parity suite is the gate.
- Every perf number **interleaved A/B/A in one invocation**, at **both** 1280x720
  and 1920x1080, with the drift reported beside the difference.
- **Read [the codegen cliff](../../concepts/gpu-codegen-cliff.md) before editing
  `march.wgsl`.** This project has lost 48% of a frame to a loop that never ran and
  0.58 ms to *removing* code from a branch a body-free scene never enters. Every
  task here re-measures rather than assuming its change was free.
- Bodies keep the existing per-pixel path. Not in scope, and the body gates must
  stay bit-identical.
- Commit on `feat/interleaved-stepping`. Do not push, merge, or touch `master`.
- No repo-wide `cargo fmt`; `classify.rs` is pre-existingly dirty.

## Two constraints the spec left open, decided here

**WGSL has no 64-bit atomics.** Core WGSL offers `atomic<u32>` and `atomic<i32>`
only, and the key is 39 bits — 12 per axis at `MAX_EXTENT` 4096, plus 3 for the
face. So the key cannot be compare-exchanged directly.

**The resolution: claim a slot with a 32-bit atomic, verify with the full key.**

```text
  slots[i] : atomic<u32>   the frame stamp that owns this slot
  keys[i]  : vec2<u32>     the full (position, face) key, written after the claim
```

Insert probes from `hash(key)`: compare-exchange `slots[i]` from *stale or zero*
to this frame's stamp; on success write `keys[i]` and take the slot. If the slot
is already claimed **this** frame, read `keys[i]` — equal means another pixel on
the same face got there first and the slot is shared, unequal means a collision
and the probe moves on.

**Stamped, never cleared.** An entry whose stamp is not this frame's is free, so
no clear pass runs over the table.

**Two new bindings, 10 and 11.** 10 is `array<atomic<u32>>` for the claim words;
11 packs keys, the visibility bits and the per-pixel slot index at offsets, as the
fullness grid already rides behind the distance field. Mixing atomic and
non-atomic in one WGSL binding is legal but clumsy, and the budget allows this:
`device_limits` asks for **16** storage buffers and 8 are used.

`MARCH_BINDING_COUNT` 10 → 12 and `STORAGE_BUFFERS_DECLARED` 8 → 10, both of which
have gates.

## Review Focus

1. **A slot shared between two different voxels is a wrong pixel.** The key
   verification is the only thing preventing it, and a hash collision is the
   normal case rather than the rare one. Pinned in Task 1.
2. **The stamp wraps.** A `u32` frame counter wraps after ~2.3 years at 60 fps, but
   a stamp of 0 must mean "never used" or frame 0 collides with free. Pinned in
   Task 1.
3. **Two faces of the same voxel are different keys.** `shadow_origin` offsets
   along the face normal, so a voxel's six faces have six origins and six answers.
   Keying on position alone would light a whole voxel from one face's answer.
   Pinned in Task 1.
4. **A body hit must not insert.** Body voxels need the body id to be distinct, and
   bodies are out of scope; inserting them would alias a body voxel onto a world
   voxel at the same coordinates. Pinned in Task 2.
5. **The debug views share the shader.** Each is its own entry point over the same
   bindings; a new binding they do not use must still be declared for them, or the
   layout gate fails for views nobody changed. Pinned in Task 1.

---

### Task 1: the store, written and never read

**Files:** `crates/bevox_render/assets/shaders/march.wgsl`; `crates/bevox_render/src/pipeline.rs`; `crates/bevox_render/tests/common/mod.rs`; `crates/bevox_render/tests/gpu_bench.rs`

**Interfaces produced:** bindings 10 and 11; `pipeline::SUN_SLOTS`; the insert in the primary path; a per-pixel slot index that nothing consumes yet.

**This task is the go/no-go.** It pays the whole cost of the store and collects
none of the benefit, so what it measures is the store's own overhead. If that
overhead is a large share of the 8 ms ceiling, the rest of the plan is not worth
building and this task is the cheapest possible way to learn it.

- [ ] **Step 1: Write the bit-identity gate first.** `the_store_changes_no_pixel`:
      the bench scene at both resolutions, with and without the insert, every pixel
      equal. It must pass trivially today and keep passing after Step 3.
- [ ] **Step 2: Add the bindings.** Shader declarations for 10 and 11, layout
      entries, buffer creation sized from the pixel count, `MARCH_BINDING_COUNT`
      10 → 12, `STORAGE_BUFFERS_DECLARED` 8 → 10. **Review Focus 5**: every debug
      entry point shares these bindings and must still compile.
      Run `cargo test -p bevox_render` — the layout and binding-count gates are
      the ones that fire.
- [ ] **Step 3: Insert at the hit.** Key is `(voxel position, face)` — **Review
      Focus 3**, six faces, six keys. Claim with compare-exchange on the stamp,
      verify with the full key, probe linearly on collision — **Review Focus 1**.
      Stamp 0 means never used — **Review Focus 2**. Write the slot index to the
      per-pixel array. **Shading is unchanged and still marches its own ray.**
- [ ] **Step 4: Report the occupancy, which is the number that predicts the win.**
      Read back the table and count distinct occupied slots at both resolutions.
      **Pixels divided by distinct faces is the redundancy factor** — how many
      times the engine is computing each answer today. Print it. If it is near 1,
      there is nothing to collect and the plan should stop here.
- [ ] **Step 5: Measure the store's own cost**, interleaved A/B/A at both
      resolutions, insert on against off. Expect a regression; report its size
      against the 8 ms ceiling.
- [ ] **Step 6: Run the parity suite and the body gates.** Bit-identical, or stop.
- [ ] **Step 7: Prove the break.** Force every key comparison to report equal; the
      image must break, because distinct voxels then share an answer. That is the
      failure Review Focus 1 guards and it must be shown, not argued.
- [ ] **Step 8: Commit**, with the occupancy and the overhead in the message.

---

### Task 2: the sun pass, and the win

**Files:** `march.wgsl`; `pipeline.rs`; `tests/common/mod.rs`

**Interfaces consumed:** everything from Task 1.

- [ ] **Step 1: Split the main pass.** The primary entry point marches to the hit,
      inserts, and writes the slot — no shading. A new composite entry point reads
      the slot and shades. Between them, a sun entry point: one invocation per
      occupied slot, one shadow march, one bit written.
- [ ] **Step 2: Dispatch the three in order**, with the sun pass sized from the
      table rather than the screen. **If dispatching over the whole table wastes
      most invocations, say so with the number** — compacting the occupied slots
      into a work list is a further pass and whether it pays is a measurement, not
      a choice. Do not build it speculatively.
- [ ] **Step 3: Body hits take the old path** — **Review Focus 4**. A body hit
      marches its own shadow ray and inserts nothing.
- [ ] **Step 4: Run the parity suite.** Bit-identical at both resolutions, bodies
      on and off. **This is the gate the whole plan rests on**; a differing pixel
      means the key does not identify what the ray depends on.
- [ ] **Step 5: Measure**, interleaved A/B/A at both resolutions, against
      `34f2d45`. Report what was collected of the 8 ms ceiling and what the three
      dispatches cost against the two they replaced.
- [ ] **Step 6: Prove the break.** Make the sun pass write a constant; the image
      must change. Then skip the sun pass entirely; the image must change
      differently. A cached value nothing computes is not a cache.
- [ ] **Step 7: Commit** with both numbers.

---

### Task 3: overflow, the worst case, and the bundle

**Files:** `march.wgsl`; `pipeline.rs`; `tests/gpu_bench.rs`; `docs/map/render.md`; a new concept; `docs/reference/dwyer-drift.md`; `docs/log.md`; the indexes

- [ ] **Step 1: The overflow fallback.** A pixel whose insert fails marches its own
      ray, exactly as today. Gate it by **shrinking the capacity until it
      overflows**, not by argument: the image must stay bit-identical and only the
      frame time may move.
- [ ] **Step 2: The worst-case gate.** A camera far enough that voxels cover about
      one pixel each has no redundancy to collect, so this measures the store's
      overhead against nothing. **It must not be slower than `34f2d45`.** If it is,
      report by how much and let the number decide whether the store needs a
      distance cutoff.
- [ ] **Step 3: Update the render map page** — the two bindings, the new constants,
      the dispatch list that is now four. It is gated and capped at a page.
- [ ] **Step 4: Write the concept.** The measured ceiling, the redundancy factor,
      what was collected, what the store cost, and the worst case. Record the
      64-bit-atomic constraint and the claim-then-verify resolution, because it is
      the non-obvious part and the next person meets it immediately.
- [ ] **Step 5: Resolve F3 in the ledger**, log, link, validate with `okf:validate`,
      and run `cargo test -p bevox --test map`.
- [ ] **Step 6: Commit.**

---

# What this plan does not do

- **It does not reach 60 fps at 1080p.** It takes the needed speedup from 1.57x to
  about 1.10x if the full ceiling is collected, and less if it is not.
- **It does not touch bodies** (F1 and F2), the traversal, or anything in physics.
- **It does not build temporal accumulation.** The store holds one frame. Seeding
  it from the last is a separate question and a separate measurement.
