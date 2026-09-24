---
type: Implementation Plan
title: 'BEVOX traversal optimisation'
description: 'Make the ray marcher fast enough to fly through a composed scene, without changing a single pixel it produces.'
tags: [gpu, performance, raymarching]
generated: { by: claude-opus-5/claude-code, at: 2026-09-14T00:00:00Z }
---

# BEVOX traversal optimisation — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make the ray marcher fast enough to fly through a composed scene, without changing a single pixel it produces.

**Architecture:** Three optimisations land one at a time behind runtime flags: DDA stepping within a brick, a bitmask filter that skips bricks a ray cannot hit, and a low-resolution beam prepass that seeds full-resolution rays. Each is measured in a headless harness with interleaved A/B/A runs, and each must leave output bit-identical to the same frame rendered with it disabled.

**Tech Stack:** Rust, wgpu 29.0.4, WGSL. Measurement is headless; no window, no vsync.

**Spec:** `docs/superpowers/specs/2026-09-13-bevox-raymarcher-core-design.md`
**Predecessor:** `2026-09-13-bevox-scene-composition.md`

## Why this exists

Measured on a GTX 1650 at 1280x720, Church_Of_St_Sophia composed at extent 4096:

| camera | frame time | fps |
|---|---|---|
| framed back at 1.1 extents | 16.6 ms | 60, vsync-locked |
| close to geometry | 37–101 ms | 10–27 |

The cost appears where the shader is weakest. Every node visit scans all 64
children linearly to find the nearest unvisited one, whether the node holds one
child or sixty-four. A node crossed by a ray that touches four of its children is
scanned four times, 256 comparisons, to answer a question DDA answers in four
steps.

## Global Constraints

- **Bit-identical or it does not land.** Each optimisation is compared against
  the same scene rendered with it disabled, pixel for pixel. An optimisation that
  changes output is a defect, not a trade-off, however plausible the new image.
- The existing parity tests against the CPU reference must stay green throughout.
  They are what makes "bit-identical to ourselves" mean "still correct".
- **Frame rate is not a measurement here.** It is vsync-locked, and a blank
  screen reports 60 fps — that is how the ray-budget bug hid. Timings come from
  the headless harness, which runs without a surface.
- Every measurement is interleaved A/B/A within one process, never a number
  compared against one recorded earlier. Cross-run drift on this hardware is
  larger than some of the effects being measured.
- Flags live in `volume_params.z`, which is currently unused padding, so nothing
  about the bind group layout changes for Tasks 1–3.
- Every WGSL loop keeps a hard iteration bound.

## Feature flags

`volume_params.z` carries a bitmask, so one shader binary renders both sides of
every comparison and the test cannot accidentally compare two different builds:

| bit | meaning |
|---|---|
| 0 | DDA stepping within a brick |
| 1 | Bitmask filter |
| 2 | Beam prepass |

Zero is the current shader, which is the reference every comparison uses.

## File Structure

| File | Responsibility |
|---|---|
| `crates/bevox_render/tests/gpu_bench.rs` | Headless timing harness and A/B/A comparisons |
| `crates/bevox_render/tests/gpu_parity.rs` | Bit-identity tests per optimisation |
| `crates/bevox_render/assets/shaders/march.wgsl` | The three optimisations |
| `crates/bevox_render/src/upload.rs` | Flags into the uniform |
| `crates/bevox/src/main.rs` | Flag defaults for the app |

---

### Task 1: Headless timing harness and the baseline

No optimisation lands before there is something to measure it with.

**Files:**
- Create: `crates/bevox_render/tests/gpu_bench.rs`

**Interfaces:**
- Produces: `bench_scene() -> (Contree, u32)`; `Prepared` holding one ready-to-dispatch configuration, with `Prepared::new(...)` and `dispatch(&self, device, queue)`; `time_dispatches(device, queue, &Prepared, iterations) -> f32` in milliseconds per frame; `compare_aba(device, queue, &Prepared, &Prepared) -> (f32, f32, f32)` returning baseline, variant and the second baseline reading.

The scene is generated rather than loaded: `assets/` is gitignored, and a
benchmark that needs a file nobody has is a benchmark nobody runs.

- [ ] **Step 1: Write the harness**

`crates/bevox_render/tests/gpu_bench.rs`:

```rust
//! Headless timing. No window and no surface, so nothing here is vsync-locked.
//!
//! Frame rate in the app cannot measure traversal cost: it is capped at 60, and
//! a scene that renders nothing at all reports exactly the same number as one
//! that renders correctly.

use bevox_core::contree::Contree;
use bevox_core::material::MaterialId;
use glam::{Mat4, UVec3, Vec3};

/// Warm-up dispatches discarded before timing, so shader compilation and first
/// touch of the buffers do not land in the measurement.
const WARMUP: u32 = 10;
/// Dispatches per timed batch. Batching amortises submit overhead.
const BATCH: u32 = 30;
/// A/B/A rounds. The two A readings bracket B, so drift is visible rather than
/// silently attributed to the change under test.
const ROUNDS: u32 = 3;

/// A scene with the shape that hurts: a large floor, columns rising from it, and
/// open space between them, at an extent big enough to need real traversal.
pub fn bench_scene() -> (Contree, u32) {
    let extent = 1024u32;
    let mut voxels = Vec::new();

    // Floor slab, four voxels thick.
    for z in 0..extent {
        for x in 0..extent {
            for y in 0..4 {
                voxels.push((UVec3::new(x, y, z), MaterialId(1)));
            }
        }
    }
    // A grid of columns.
    for cz in 0..8u32 {
        for cx in 0..8u32 {
            let ox = 64 + cx * 128;
            let oz = 64 + cz * 128;
            for y in 4..120 {
                for dz in 0..24 {
                    for dx in 0..24 {
                        voxels.push((UVec3::new(ox + dx, y, oz + dz), MaterialId(2)));
                    }
                }
            }
        }
    }

    (Contree::from_voxels(extent, &voxels), extent)
}
```

- [ ] **Step 2: Write the timing function**

Timing is wall clock around a submitted batch, with the device polled to
completion. That is coarser than GPU timestamp queries but needs no optional
device feature, and it is the same coarseness on both sides of a comparison,
which is all an A/B needs.

```rust
/// Milliseconds per dispatch, averaged over a batch.
#[allow(clippy::too_many_arguments)]
pub fn time_dispatches(
    device: &wgpu::Device,
    queue: &wgpu::Queue,
    prepared: &Prepared,
    iterations: u32,
) -> f32 {
    // Warm up: first dispatch pays for pipeline creation and buffer residency.
    for _ in 0..WARMUP {
        prepared.dispatch(device, queue);
    }
    device.poll(wgpu::PollType::wait_indefinitely()).expect("poll");

    let started = std::time::Instant::now();
    for _ in 0..iterations {
        prepared.dispatch(device, queue);
    }
    device.poll(wgpu::PollType::wait_indefinitely()).expect("poll");
    started.elapsed().as_secs_f32() * 1000.0 / iterations as f32
}

/// Runs baseline, variant, baseline, interleaved, and returns the medians.
///
/// The two baseline readings are reported so drift is visible. A variant that
/// "wins" by less than the spread between them has not been shown to win.
pub fn compare_aba(
    device: &wgpu::Device,
    queue: &wgpu::Queue,
    baseline: &Prepared,
    variant: &Prepared,
) -> (f32, f32, f32) {
    let mut a1 = Vec::new();
    let mut b = Vec::new();
    let mut a2 = Vec::new();
    for _ in 0..ROUNDS {
        a1.push(time_dispatches(device, queue, baseline, BATCH));
        b.push(time_dispatches(device, queue, variant, BATCH));
        a2.push(time_dispatches(device, queue, baseline, BATCH));
    }
    (median(&mut a1), median(&mut b), median(&mut a2))
}

fn median(v: &mut [f32]) -> f32 {
    v.sort_by(f32::total_cmp);
    v[v.len() / 2]
}
```

`Prepared` holds everything one configuration needs, so a comparison never
rebuilds resources between readings — resource creation inside the timed loop
would measure the driver, not the shader.

Put it in `crates/bevox_render/tests/common/mod.rs` and have both test files
declare `mod common;`, rather than copying it into each:

```rust
/// One ready-to-dispatch configuration: pipeline, bindings and dimensions.
pub struct Prepared {
    pipeline: wgpu::ComputePipeline,
    bind_group: wgpu::BindGroup,
    width: u32,
    height: u32,
}

impl Prepared {
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        device: &wgpu::Device,
        shader_source: &str,
        entry_point: &str,
        tree: &Contree,
        world_from_clip: Mat4,
        eye: Vec3,
        width: u32,
        height: u32,
        flags: u32,
    ) -> Self {
        // Same buffers, layout and pipeline construction as run_march_flagged in
        // gpu_parity.rs, which moves here so both callers share one definition.
        // The only difference is that the texture and bind group are retained
        // rather than read back and dropped.
        todo_shared_setup()
    }

    /// Submits one dispatch. Does not wait: the caller polls once per batch.
    pub fn dispatch(&self, device: &wgpu::Device, queue: &wgpu::Queue) {
        let mut encoder = device.create_command_encoder(&Default::default());
        {
            let mut pass = encoder.begin_compute_pass(&Default::default());
            pass.set_pipeline(&self.pipeline);
            pass.set_bind_group(0, &self.bind_group, &[]);
            pass.dispatch_workgroups(self.width.div_ceil(8), self.height.div_ceil(8), 1);
        }
        queue.submit([encoder.finish()]);
    }
}
```

> **`todo_shared_setup()` is not a placeholder to leave in.** Task 1 Step 2 is
> complete only when the buffer, layout and pipeline construction currently
> inside `run_march_flagged` has physically moved into `Prepared::new`, and
> `run_march_flagged` calls it and then reads the texture back. Two copies of
> that setup drifting apart is precisely how the harness stopped testing the
> layout the app ships, in the previous plan.

- [ ] **Step 3: Record the baseline**

```rust
#[test]
fn record_the_baseline() {
    let Some((device, queue)) = gpu_device() else {
        eprintln!("no GPU adapter available, skipping");
        return;
    };
    let (tree, extent) = bench_scene();
    // Close to the geometry, which is where the app slowed down.
    let eye = Vec3::new(extent as f32 * 0.5, 40.0, extent as f32 * 0.5 - 120.0);
    let target = Vec3::new(extent as f32 * 0.5, 40.0, extent as f32 * 0.5);

    let prepared = Prepared::new(&device, &tree, eye, target, 1280, 720, 0, "march");
    let ms = time_dispatches(&device, &queue, &prepared, BATCH);
    println!("baseline: {ms:.2} ms per frame at 1280x720");

    assert!(ms > 0.0, "the timer returned nothing");
}
```

Run: `cargo test -p bevox_render --test gpu_bench -- --nocapture`
Expected: a printed baseline. Record the number in this plan before continuing —
every later task compares against a baseline measured in the same run, but the
recorded figure is what says whether the work was worth doing at all.

**Recorded baseline**, GTX 1650, release, 1280x720, camera close to geometry:

| | |
|---|---|
| scene | extent 1024, 72,424 arena nodes, 2.1 MB voxel bytes, built in 0.16s |
| baseline | **36.70 ms per frame**, about 27 fps |

That matches the app's measured 37-101 ms close to geometry, so the synthetic
scene reproduces the symptom rather than an artificial one. Every later task
compares against a baseline measured in the same run; this figure is what says
whether the work was worth doing at all.

- [ ] **Step 4: Commit**

```bash
git add crates/bevox_render
git commit -m "test(render): add a headless timing harness" -m "Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>"
```

---

### Task 2: Feature flags through the uniform

**Files:**
- Modify: `crates/bevox_render/src/upload.rs`, `crates/bevox_render/assets/shaders/march.wgsl`, both test files

**Interfaces:**
- Produces: `MarchFlags` constants (`DDA = 1`, `MASK_FILTER = 2`, `BEAM = 4`); `volume_params.z` carrying them; WGSL `fn flag_enabled(bit: u32) -> bool`.

- [ ] **Step 1: Write the failing test**

```rust
/// Flags must reach the shader. With no optimisation implemented yet, setting
/// them must change nothing — which is also the shape every later test takes.
#[test]
fn setting_flags_does_not_change_output_yet() {
    let Some((device, queue)) = gpu_device() else {
        eprintln!("no GPU adapter available, skipping");
        return;
    };
    let tree = parity_scene();
    let gpu_volume = GpuVolume::from_contree(&tree);
    let (width, height) = (64u32, 64u32);
    let eye = Vec3::new(-30.0, 40.0, -30.0);
    let view = Mat4::look_at_rh(eye, Vec3::new(32.0, 12.0, 32.0), Vec3::Y);
    let projection = Mat4::perspective_rh(0.9, 1.0, 0.1, 500.0);
    let world_from_clip = (projection * view).inverse();
    let shader = std::fs::read_to_string("assets/shaders/march.wgsl").expect("shader missing");

    let off = run_march_flagged(
        &device, &queue, &shader, "march_identity", world_from_clip, eye,
        &tree, &gpu_volume, width, height, 0,
    );
    let on = run_march_flagged(
        &device, &queue, &shader, "march_identity", world_from_clip, eye,
        &tree, &gpu_volume, width, height, 0b111,
    );

    assert_eq!(off, on, "flags changed output before any optimisation exists");
}
```

`run_march_flagged` is `run_march` with a flags argument; the existing
`run_march` becomes a wrapper passing zero, so no earlier test changes.

- [ ] **Step 2: Run to verify it fails**

Run: `cargo test -p bevox_render --test gpu_parity setting_flags`
Expected: FAIL — `run_march_flagged` not found.

- [ ] **Step 3: Implement**

In `upload.rs`:

```rust
/// Traversal optimisations, carried in `volume_params.z`.
///
/// One shader renders both sides of every comparison, so a bit-identity test
/// cannot accidentally compare two different builds.
pub mod march_flags {
    pub const NONE: u32 = 0;
    pub const DDA: u32 = 1;
    pub const MASK_FILTER: u32 = 2;
    pub const BEAM: u32 = 4;
    /// What the app runs once each optimisation has proven itself.
    pub const DEFAULT: u32 = NONE;
}
```

`march_uniform` takes a `flags: u32` and writes
`volume_params: [depth, extent, flags, 0]`.

In the shader:

```wgsl
fn flag_enabled(bit: u32) -> bool {
    return (view.volume_params.z & bit) != 0u;
}
```

- [ ] **Step 4: Run to verify it passes**

Run: `cargo test -p bevox_render`
Expected: PASS. Every parity test still green, since flags do nothing yet.

- [ ] **Step 5: Commit**

```bash
git add crates
git commit -m "feat(render): carry traversal flags in the uniform" -m "Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>"
```

---

### Task 3: DDA stepping within a brick

The current inner loop answers "which child does this ray reach next" by scanning
all 64 and taking the nearest unvisited. DDA answers it by stepping.

**Files:**
- Modify: `crates/bevox_render/assets/shaders/march.wgsl`
- Modify: `crates/bevox_render/tests/gpu_parity.rs`, `crates/bevox_render/tests/gpu_bench.rs`

- [ ] **Step 1: Write the bit-identity test**

```rust
/// DDA changes how children are visited, not which voxel is hit. Any pixel that
/// differs is a defect.
#[test]
fn dda_leaves_output_bit_identical() {
    let Some((device, queue)) = gpu_device() else {
        eprintln!("no GPU adapter available, skipping");
        return;
    };
    let tree = parity_scene();
    let gpu_volume = GpuVolume::from_contree(&tree);
    let (width, height) = (128u32, 128u32);
    let eye = Vec3::new(-30.0, 40.0, -30.0);
    let view = Mat4::look_at_rh(eye, Vec3::new(32.0, 12.0, 32.0), Vec3::Y);
    let projection = Mat4::perspective_rh(0.9, 1.0, 0.1, 500.0);
    let world_from_clip = (projection * view).inverse();
    let shader = std::fs::read_to_string("assets/shaders/march.wgsl").expect("shader missing");

    for entry in ["march_identity", "march_normal", "march_shadow", "march"] {
        let off = run_march_flagged(
            &device, &queue, &shader, entry, world_from_clip, eye,
            &tree, &gpu_volume, width, height, march_flags::NONE,
        );
        let on = run_march_flagged(
            &device, &queue, &shader, entry, world_from_clip, eye,
            &tree, &gpu_volume, width, height, march_flags::DDA,
        );
        let differing = off.iter().zip(&on).filter(|(a, b)| a != b).count();
        assert_eq!(differing, 0, "{entry}: {differing} bytes differ with DDA on");
    }
}
```

Testing all four entry points matters: normals and shadows call `traverse`
through different paths, and an ordering change that only shows up on a shadow
ray would otherwise pass.

- [ ] **Step 2: Run to verify it fails**

Run: `cargo test -p bevox_render --test gpu_parity dda_leaves`
Expected: PASS trivially, because the flag does nothing yet. This is the one
test in the plan whose first run is green; it turns red only if Step 3 gets the
traversal wrong, which is precisely its job.

- [ ] **Step 3: Implement DDA stepping**

Replace the nearest-unvisited scan with a stepped walk when the flag is set. The
node's children form a uniform 4x4x4 grid, which is the case DDA is for: the
distance to the next plane on each axis is maintained incrementally, and the
smallest one names the axis to step.

```wgsl
/// State for stepping a ray through one node's 4x4x4 children.
struct Dda {
    cell: vec3<i32>,
    step: vec3<i32>,
    t_max: vec3<f32>,
    t_delta: vec3<f32>,
};

fn dda_begin(origin: vec3<f32>, dir: vec3<f32>, inv_dir: vec3<f32>,
             node_origin: vec3<f32>, cell_size: f32, t_enter: f32) -> Dda {
    let point = origin + dir * t_enter;
    let local = (point - node_origin) / cell_size;
    var cell = clamp(vec3<i32>(floor(local)), vec3<i32>(0), vec3<i32>(3));

    var step = vec3<i32>(1);
    var t_max = vec3<f32>(1e30);
    var t_delta = vec3<f32>(1e30);
    for (var axis = 0u; axis < 3u; axis = axis + 1u) {
        let d = dir[axis];
        if d != 0.0 {
            let s = select(-1, 1, d > 0.0);
            step[axis] = s;
            let boundary = node_origin[axis]
                + (f32(cell[axis]) + select(0.0, 1.0, s > 0)) * cell_size;
            t_max[axis] = (boundary - origin[axis]) * inv_dir[axis];
            t_delta[axis] = abs(cell_size * inv_dir[axis]);
        }
    }
    return Dda(cell, step, t_max, t_delta);
}

/// Advances to the next cell. Returns false when the walk leaves the node.
fn dda_advance(d: ptr<function, Dda>) -> bool {
    let t = (*d).t_max;
    var axis = 0u;
    if t.y < t.x && t.y <= t.z { axis = 1u; }
    else if t.z < t.x && t.z < t.y { axis = 2u; }

    (*d).cell[axis] = (*d).cell[axis] + (*d).step[axis];
    (*d).t_max[axis] = (*d).t_max[axis] + (*d).t_delta[axis];
    return (*d).cell[axis] >= 0 && (*d).cell[axis] <= 3;
}
```

Inside `traverse`, when `flag_enabled(1u)`, a frame walks its children with this
rather than rescanning. The visited-mask bookkeeping disappears on that path:
DDA visits each crossed cell exactly once, in order, so there is nothing to
remember.

> **Where this will be subtly wrong, if it is.** DDA emits cells in ray order;
> the scan emitted them in nearest-entry order. Those agree only while cells are
> the same size and disjoint, which holds within one node. If the bit-identity
> test fails on a handful of pixels rather than most of them, suspect the
> boundary case where a ray enters exactly on a cell edge and `floor` picks the
> neighbour the scan would not have.

- [ ] **Step 4: Run the bit-identity test and the parity suite**

Run: `cargo test -p bevox_render`
Expected: PASS, all entry points identical, all CPU parity intact.

- [ ] **Step 5: Measure**

```rust
#[test]
fn dda_is_measured_against_the_baseline() {
    let Some((device, queue)) = gpu_device() else {
        eprintln!("no GPU adapter available, skipping");
        return;
    };
    let (tree, extent) = bench_scene();
    let half = extent as f32 * 0.5;
    // Close to the geometry: the camera position the app slowed down at.
    let eye = Vec3::new(half, 40.0, half - 120.0);
    let view = Mat4::look_at_rh(eye, Vec3::new(half, 40.0, half), Vec3::Y);
    let projection = Mat4::perspective_rh(0.9, 1280.0 / 720.0, 0.1, 20_000.0);
    let world_from_clip = (projection * view).inverse();
    let shader = std::fs::read_to_string("assets/shaders/march.wgsl").expect("shader missing");

    let baseline = Prepared::new(
        &device, &shader, "march", &tree, world_from_clip, eye, 1280, 720,
        march_flags::NONE,
    );
    let with_dda = Prepared::new(
        &device, &shader, "march", &tree, world_from_clip, eye, 1280, 720,
        march_flags::DDA,
    );

    let (a1, b, a2) = compare_aba(&device, &queue, &baseline, &with_dda);
    println!("DDA: baseline {a1:.2} / {a2:.2} ms, with DDA {b:.2} ms");
    assert!(
        (a1 - a2).abs() < a1 * 0.15,
        "baseline drifted {a1:.2} to {a2:.2}; the comparison is not trustworthy"
    );
}
```

Record the numbers here. A result inside the baseline spread is a result: it
says this optimisation does not pay on this hardware and this scene, which is
worth knowing before the next one is built on top of it.

- [ ] **Step 6: Commit**

```bash
git add crates
git commit -m "perf(render): step brick children with DDA" -m "Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>"
```

---

### Task 4: Bitmask filter

`bevox_core::mask_table` has held the reachability masks since milestone 1,
tested and unused. This is what they were for.

**Files:**
- Modify: `crates/bevox_render/src/upload.rs`, `crates/bevox_render/src/pipeline.rs`
- Modify: `crates/bevox_render/assets/shaders/march.wgsl`, both test files

**Interfaces:**
- Produces: binding 5, a storage buffer of 512 masks as `array<vec2<u32>>` (low and high halves), from `build_direction_masks()`.

> **A bind group change touches three places that move together**: the layout in
> `init_march_pipeline`, the entries in `dispatch_march`, and the harness. The
> storage texture moves to binding 5 in the shader and every entry point.

- [ ] **Step 1: Write the bit-identity test**

```rust
/// The filter decides which bricks are entered, never what is found inside one.
/// Combinations are tested too: an optimisation can be individually sound and
/// wrong in company, and the app runs them together.
#[test]
fn the_mask_filter_leaves_output_bit_identical() {
    let Some((device, queue)) = gpu_device() else {
        eprintln!("no GPU adapter available, skipping");
        return;
    };
    let tree = parity_scene();
    let gpu_volume = GpuVolume::from_contree(&tree);
    let (width, height) = (128u32, 128u32);
    let eye = Vec3::new(-30.0, 40.0, -30.0);
    let view = Mat4::look_at_rh(eye, Vec3::new(32.0, 12.0, 32.0), Vec3::Y);
    let projection = Mat4::perspective_rh(0.9, 1.0, 0.1, 500.0);
    let world_from_clip = (projection * view).inverse();
    let shader = std::fs::read_to_string("assets/shaders/march.wgsl").expect("shader missing");

    let reference = run_march_flagged(
        &device, &queue, &shader, "march_identity", world_from_clip, eye,
        &tree, &gpu_volume, width, height, march_flags::NONE,
    );

    for flags in [
        march_flags::MASK_FILTER,
        march_flags::DDA | march_flags::MASK_FILTER,
    ] {
        let got = run_march_flagged(
            &device, &queue, &shader, "march_identity", world_from_clip, eye,
            &tree, &gpu_volume, width, height, flags,
        );
        let differing = reference.iter().zip(&got).filter(|(a, b)| a != b).count();
        assert_eq!(differing, 0, "flags {flags:#b}: {differing} bytes differ");
    }
}
```

- [ ] **Step 2: Upload the table**

`GpuSceneData` gains `pub direction_masks: Vec<[u32; 2]>`, filled from
`bevox_core::mask_table::build_direction_masks()` split into halves. It is
constant, so it is built once in `Default` rather than per scene.

- [ ] **Step 3: Implement the filter**

Before descending into a child, AND its occupancy mask with the reachability mask
for the ray's entry cell and direction octant. A zero result means the ray cannot
reach anything inside, and the child is skipped without being entered.

```wgsl
fn direction_mask(cell: u32, octant: u32) -> vec2<u32> {
    return direction_masks[cell * 8u + octant];
}

/// Whether this ray can reach any occupied child of `node` from `cell`.
fn brick_reachable(node: vec4<u32>, cell: u32, octant: u32) -> bool {
    let m = direction_mask(cell, octant);
    return (node_mask_lo(node) & m.x) != 0u || (node_mask_hi(node) & m.y) != 0u;
}
```

The octant comes from the ray direction's sign bits, matching
`bevox_core::mask_table::octant_index`, whose tests already pin that a zero
component counts as positive.

- [ ] **Step 4: Run tests, then measure**

Run: `cargo test -p bevox_render`, then the A/B/A benchmark with
`MASK_FILTER` and with `DDA | MASK_FILTER`. Record both.

- [ ] **Step 5: Commit**

```bash
git add crates
git commit -m "perf(render): skip bricks a ray cannot reach" -m "Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>"
```

---

### Task 5: Beam prepass

The one aimed squarely at the measured problem: cost rises when geometry fills
the screen, and this makes full-resolution rays start where a coarse pass already
proved there is nothing.

**Files:**
- Modify: `crates/bevox_render/assets/shaders/march.wgsl`, `crates/bevox_render/src/pipeline.rs`, `crates/bevox_render/src/upload.rs`, both test files

**Interfaces:**
- Produces: a second storage texture at one-eighth resolution per axis, `R32Float`; entry point `beam_prepass`; a second dispatch before the main one.

- [ ] **Step 1: Write the conservativeness test**

This one is not a plain bit-identity check, because the prepass is only safe when
its seed distance can never overshoot geometry. The test renders at several
camera positions, including ones where thin geometry sits between beam rays.

```rust
/// Thin geometry is what a too-aggressive seed eats first: single-voxel walls
/// and isolated voxels, which can hide between beam samples.
fn thin_scene() -> Contree {
    let extent = 64u32;
    let mut voxels = Vec::new();
    // Single-voxel-thick walls on two axes.
    for a in 0..extent {
        for b in 0..extent {
            voxels.push((UVec3::new(a, b, 32), MaterialId(1)));
            voxels.push((UVec3::new(32, a, b), MaterialId(2)));
        }
    }
    // Isolated voxels scattered off the walls.
    let mut rng = bevox_core::testing::XorShift64::new(4242);
    for _ in 0..200 {
        voxels.push((
            UVec3::new(rng.next_below(extent), rng.next_below(extent), rng.next_below(extent)),
            MaterialId(1),
        ));
    }
    Contree::from_voxels(extent, &voxels)
}

/// A beam prepass seeds full-resolution rays with a distance a coarse pass
/// proved empty. Seeding even slightly too far skips thin geometry, and the
/// symptom is holes that appear from some angles and not others — so this
/// sweeps angles rather than trusting one view.
#[test]
fn the_beam_prepass_never_skips_geometry() {
    let Some((device, queue)) = gpu_device() else {
        eprintln!("no GPU adapter available, skipping");
        return;
    };
    let tree = thin_scene();
    let gpu_volume = GpuVolume::from_contree(&tree);
    let (width, height) = (128u32, 128u32);
    let centre = Vec3::splat(32.0);
    let shader = std::fs::read_to_string("assets/shaders/march.wgsl").expect("shader missing");

    for step in 0..8u32 {
        let angle = step as f32 * std::f32::consts::TAU / 8.0;
        let eye = centre + Vec3::new(angle.cos() * 90.0, 30.0, angle.sin() * 90.0);
        let view = Mat4::look_at_rh(eye, centre, Vec3::Y);
        let projection = Mat4::perspective_rh(0.9, 1.0, 0.1, 500.0);
        let world_from_clip = (projection * view).inverse();

        let reference = run_march_flagged(
            &device, &queue, &shader, "march_identity", world_from_clip, eye,
            &tree, &gpu_volume, width, height, march_flags::NONE,
        );
        let seeded = run_march_flagged(
            &device, &queue, &shader, "march_identity", world_from_clip, eye,
            &tree, &gpu_volume, width, height, march_flags::BEAM,
        );

        let differing = reference.iter().zip(&seeded).filter(|(a, b)| a != b).count();
        assert_eq!(differing, 0, "angle {step}: {differing} bytes differ; the seed overshot");
    }
}
```

- [ ] **Step 2: Implement the prepass**

> **This adds two bindings**, so the layout, the dispatch entries and the harness
> move together, and the storage texture shifts again in every entry point. The
> beam texture is bound twice: written by the prepass, read by the main pass.

```wgsl
@group(0) @binding(5) var beam_out: texture_storage_2d<r32float, write>;
@group(0) @binding(6) var beam_in: texture_storage_2d<r32float, read>;
```

```wgsl
/// Distance at which one voxel shrinks to the spacing between beam rays.
///
/// Past this, a voxel can sit entirely between two beams and be missed, so the
/// seed is capped here whatever the beam actually found. Two adjacent beam
/// directions give the spacing directly, which avoids passing a field of view
/// through the uniform and going stale when the projection changes.
fn beam_safe_distance(size: vec2<u32>) -> f32 {
    let a = primary_ray(vec3<u32>(0u, 0u, 0u), size);
    let b = primary_ray(vec3<u32>(1u, 0u, 0u), size);
    let spread = length(b - a);
    if spread <= 1e-9 {
        return 1e30;
    }
    return 1.0 / spread;
}

/// Coarse pass: how far each beam travelled before meeting anything.
@compute @workgroup_size(8, 8, 1)
fn beam_prepass(@builtin(global_invocation_id) id: vec3<u32>) {
    let size = textureDimensions(beam_out);
    if id.x >= size.x || id.y >= size.y { return; }

    let hit = traverse(view.camera_position.xyz, primary_ray(id, size), max_ray_distance());
    let cap = beam_safe_distance(size);
    var seed = cap;
    if hit.hit {
        seed = min(hit.t, cap);
    }
    textureStore(beam_out, vec2<i32>(id.xy), vec4<f32>(seed, 0.0, 0.0, 0.0));
}

/// Smallest distance any surrounding beam reported.
///
/// The minimum is what makes this safe: any of the four may be the beam that
/// saw the geometry this pixel is about to hit.
fn beam_seed(id: vec3<u32>) -> f32 {
    let bs = textureDimensions(beam_in);
    let base = vec2<u32>(id.x / 8u, id.y / 8u);
    var m = 1e30;
    for (var dy = 0u; dy < 2u; dy = dy + 1u) {
        for (var dx = 0u; dx < 2u; dx = dx + 1u) {
            let s = min(base + vec2<u32>(dx, dy), bs - vec2<u32>(1u));
            m = min(m, textureLoad(beam_in, vec2<i32>(s)).r);
        }
    }
    return m;
}
```

Each full-resolution entry point then starts its ray further along the same line,
rather than `traverse` growing a start parameter:

```wgsl
    let dir = primary_ray(id, size);
    var t_seed = 0.0;
    if flag_enabled(4u) {
        t_seed = beam_seed(id);
    }
    let hit = traverse(
        view.camera_position.xyz + dir * t_seed,
        dir,
        max_ray_distance() - t_seed,
    );
    // hit.t is measured from the offset origin; add t_seed wherever an absolute
    // distance is used.
```

Offsetting the origin keeps `traverse` untouched, which matters: it is the
function every parity test pins, and a seed parameter threaded through it would
put the optimisation inside the thing being verified rather than outside it.

- [ ] **Step 3: Run tests, then measure**

Measure `BEAM` alone and `DDA | MASK_FILTER | BEAM`. The close-to-geometry camera
is the case that motivated this work, so measure that one specifically, not only
the framed-back view.

- [ ] **Step 4: Commit**

```bash
git add crates
git commit -m "perf(render): seed full resolution rays from a beam prepass" -m "Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>"
```

---

### Task 6: Choose defaults and re-measure the real scenes

**Files:**
- Modify: `crates/bevox/src/main.rs`, `crates/bevox_render/src/upload.rs`
- Modify: `docs/superpowers/plans/2026-09-13-bevox-scene-composition.md`

- [ ] **Step 1: Set the defaults from the measurements**

`march_flags::DEFAULT` becomes the combination that measured fastest while
staying bit-identical. An optimisation that did not pay stays implemented and
off, with the measurement recorded next to it saying why.

- [ ] **Step 2: Re-measure the app against the composed scenes**

Run each file in `assets/` and record frame times both framed back and close to
geometry, the same two positions the original measurement used:

```bash
cargo run -p bevox --release -- assets/Church_Of_St_Sophia.vox
```

Frame rate is capped at 60, so a framed-back reading of 60 fps says only "at
least as fast as before". The close reading is the one that moves.

- [ ] **Step 3: Record the result**

Update the measurements table in the scene composition plan with a second column,
so the before and after sit together rather than in separate documents.

- [ ] **Step 4: Run the whole suite**

Run: `cargo test`
Expected: PASS, including every bit-identity test with the default flags set.

- [ ] **Step 5: Commit**

```bash
git add crates docs
git commit -m "perf(render): enable the optimisations that measured faster" -m "Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>"
```

---

## Milestone check

**Milestone 8** — DDA, bitmask filtering and the beam prepass, each measured
A/B/A against a baseline in the same run, and each proven to leave output
bit-identical.

## What this plan deliberately does not do

No LODs, no empty-space distance fields, no GPU timestamp queries. No changes to
the data structure: everything here is traversal, so the parity tests stay valid
throughout.

No re-architecture if an optimisation disappoints. A measurement showing that DDA
does not pay on this hardware is a finding to record, not a reason to keep
rewriting until the number moves.

## Subsequent work

Milestone 7 — the sphere brush wired to mouse input with dirty-range upload — is
the last one left in the spec, and is independent of everything here.

---

## GPU timestamps, added afterwards

The spec asked for timestamp queries wrapping each dispatch. They live in the
headless harness (`Prepared::dispatch_timed`), not in the app: that is where the
A/B/A discipline is, and it needs no device-feature negotiation in the binary.
An adapter without `TIMESTAMP_QUERY` still runs every test and simply reports no
GPU time.

Measured at 1280x720, extent 1024, close to geometry, GTX 1650:

```
      dda:  22.02 ms vs scan 40.57/39.29 (drift 1.28)  gain 17.91 ms (44.9%)  [gpu 20.70 ms]
     mask:  34.42 ms vs scan 39.48/39.94 (drift 0.46)  gain  5.28 ms (13.3%)  [gpu 34.29 ms]
 dda+mask:  18.50 ms vs scan 40.37/40.13 (drift 0.24)  gain 21.75 ms (54.0%)  [gpu 17.73 ms]
     beam:  29.31 ms vs scan 39.84/40.55 (drift 0.71)  gain 10.88 ms (27.1%)  [gpu 30.78 ms = beam 0.93 + main 29.86]
      all:  15.44 ms vs scan 39.89/40.75 (drift 0.86)  gain 24.88 ms (61.7%)  [gpu 15.78 ms = beam 0.30 + main 15.49]
```

Two things worth keeping from this.

**The wall-clock numbers were honest.** GPU time tracks them within about a
millisecond at every flag setting, so submit and driver overhead was never
hiding inside the measurements the optimisation decisions were made from. That
was worth checking rather than assuming.

**The beam prepass is nearly free, and gets cheaper in company.** 0.93 ms on its
own, 0.30 ms with DDA and the mask filter on -- the prepass is itself a march,
so it benefits from the same optimisations it feeds. Its cost was never the
question; what it hands the main pass is. Splitting the two passes is the one
thing wall-clock timing could not have told us.
