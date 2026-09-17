# Rigid Body Rendering Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Render a voxel volume placed by a rigid transform, composed with the static world by nearest hit, so a body can be seen rotating before any physics exists.

**Architecture:** A body is a small `Contree` plus a rigid transform. Its nodes and voxels are packed into the existing storage buffers behind a per-body base offset, so no new bindings are needed for the geometry. The shader gains a body table at one new binding, transforms the ray into each body's local frame, marches with the existing `traverse`, and keeps the nearest hit. Because the transform is rigid, `t` means the same thing in both frames and the comparison is exact.

**Tech Stack:** Rust, Bevy 0.19.1, wgpu 29.0.4, glam 0.32, WGSL.

**Spec:** `docs/superpowers/specs/2026-09-15-bevox-rigid-bodies-design.md`

## Global Constraints

- Native desktop only. No web build, ever — permanently out of scope.
- `bevox_core` has no Bevy and no GPU dependency, and gains no new dependencies.
- **Body transforms are rigid: rotation and translation, no scale.** This is load-bearing, not a simplification — it is what makes `t` comparable between frames and the nearest-hit composition exact.
- Voxel data on the GPU is budgeted at **512 MB maximum**, checked at upload as an error rather than an allocation attempt. Body geometry counts against it.
- No new dependencies in any crate.
- Tests use a seeded `bevox_core::testing::XorShift64`; no test framework may be added.
- Every performance claim comes from interleaved A/B/A within one session, drift reported.
- **A scene with zero bodies must render bit-identically to the same scene before this work.** The composition loop changing a pixel of a body-free scene is a defect.
- Verify with `cargo test --workspace`.

## Facts that are not obvious from the code

**`traverse` already works in volume space.** It takes a world-space origin and direction today only because the volume sits at the origin with identity transform. Handing it a ray already transformed into a body's local frame needs no change to `traverse` itself — which matters, because `traverse` is the function every parity test pins.

**`bevox_core::march::march` has taken `volume_to_world: Affine3A` since milestone 2** and been passed `IDENTITY` everywhere. It transforms the ray in and marches. The GPU should mirror that shape exactly, so the CPU stays a usable reference for bodies too.

**A rigid transform preserves `t`.** `world_to_local` rotates and translates; it does not scale, so a distance along the transformed ray equals the distance along the original. Do not normalise the transformed direction — that would silently rescale `t` and break the comparison. The direction comes out of a rotation already unit-length.

**The normal comes back rotated, not transformed.** A surface normal under a rigid transform rotates with the rotation part and ignores the translation. Using the full affine on a normal adds the translation and points it somewhere meaningless.

**Node and voxel indices are already shifted by one for the root.** `GpuVolume::buffer_nodes()` puts the root at index 0 and the arena after it. A body packed at base `B` therefore has its root at `B` and its arena slot `n` at `B + 1 + n`. Getting this wrong renders one body's geometry while walking another's tree.

## File Structure

- `crates/bevox_core/src/body.rs` — **new**. `Body`: a `Contree` plus a rigid transform, and the world/local conversions. No Bevy, no GPU. Its own file because it is the data structure the whole subsystem hangs off and it must stay testable without a device.
- `crates/bevox_core/src/lib.rs` — declare the module.
- `crates/bevox_render/src/upload.rs` — pack bodies into `GpuSceneData` and build the body table.
- `crates/bevox_render/src/pipeline.rs` — binding 8, the layout entry, the budget.
- `crates/bevox_render/assets/shaders/march.wgsl` — the composition loop, behind `FLAG_BODIES`.
- `crates/bevox/src/main.rs` — one body with a hardcoded spin, so it can be seen.
- `crates/bevox_render/tests/{common/mod.rs,gpu_parity.rs,gpu_bench.rs}` — harness binding, gates, measurement.

---

### Task 1: A body and its frames, on the CPU

**Files:**
- Create: `crates/bevox_core/src/body.rs`
- Modify: `crates/bevox_core/src/lib.rs`

**Interfaces:**
- Produces: `#[derive(Clone, Debug)]` on `Contree` and `#[derive(Clone)]` on `NodeArena`; `pub struct Body { pub volume: Contree, pub position: Vec3, pub orientation: Quat }` with `pub fn new(volume: Contree, position: Vec3, orientation: Quat) -> Self`, `pub fn local_from_world(&self) -> Affine3A`, `pub fn world_from_local(&self) -> Affine3A`, `pub fn march_world(&self, origin: Vec3, dir: Vec3, max_dist: f32, stats: &mut MarchStats) -> Option<Hit>`.
- Consumes: `bevox_core::contree::Contree`, `bevox_core::march::{march, Hit, MarchStats}`.

- [ ] **Step 1: Write the failing tests**

Create `crates/bevox_core/src/body.rs` with this test module and nothing else yet:

```rust
//! A voxel volume placed in the world by a rigid transform.
//!
//! Rigid means rotation and translation and nothing else. That is what keeps a
//! distance along a ray meaning the same thing inside the body's frame as
//! outside it, which is what lets the renderer compose bodies with the static
//! world by simply keeping the nearest hit.

#[cfg(test)]
mod tests {
    use super::*;
    use crate::dense::DenseVolume;
    use crate::material::MaterialId;
    use glam::UVec3;

    /// A solid 16-voxel cube in a 64 volume, centred on the volume's middle.
    fn cube() -> Contree {
        let mut dense = DenseVolume::new(64).unwrap();
        for z in 24..40 {
            for y in 24..40 {
                for x in 24..40 {
                    dense.set(UVec3::new(x, y, z), MaterialId(1));
                }
            }
        }
        dense.into_contree()
    }

    /// An untransformed body is marched exactly as the bare volume is, or the
    /// composition would perturb a scene that has not moved.
    #[test]
    fn an_identity_body_matches_marching_the_volume_directly() {
        let body = Body::new(cube(), Vec3::ZERO, Quat::IDENTITY);
        let origin = Vec3::new(32.0, 32.0, -40.0);
        let dir = Vec3::Z;

        let mut a = MarchStats::default();
        let direct = march(&body.volume, Affine3A::IDENTITY, origin, dir, 500.0, false, &mut a);
        let mut b = MarchStats::default();
        let through = body.march_world(origin, dir, 500.0, &mut b);

        assert_eq!(direct.map(|h| h.voxel), through.map(|h| h.voxel));
        assert_eq!(direct.map(|h| h.t), through.map(|h| h.t));
    }

    /// Translating the body moves where the ray meets it by exactly the same
    /// amount, in the opposite sense along the ray.
    #[test]
    fn translating_the_body_shifts_the_hit_distance() {
        let origin = Vec3::new(32.0, 32.0, -40.0);
        let dir = Vec3::Z;
        let mut stats = MarchStats::default();

        let still = Body::new(cube(), Vec3::ZERO, Quat::IDENTITY)
            .march_world(origin, dir, 500.0, &mut stats)
            .expect("the ray should meet the cube");
        let moved = Body::new(cube(), Vec3::new(0.0, 0.0, 10.0), Quat::IDENTITY)
            .march_world(origin, dir, 500.0, &mut stats)
            .expect("the ray should still meet the cube");

        assert!(
            (moved.t - still.t - 10.0).abs() < 1e-3,
            "moving the body 10 along the ray changed the hit distance by {}",
            moved.t - still.t
        );
    }

    /// A rigid transform preserves distance. A quarter turn about the cube's
    /// own centre leaves a ray down the axis meeting it at the same distance,
    /// because a cube is symmetric under that rotation.
    #[test]
    fn a_rigid_rotation_preserves_the_hit_distance() {
        let centre = Vec3::splat(32.0);
        let origin = Vec3::new(32.0, 32.0, -40.0);
        let dir = Vec3::Z;
        let mut stats = MarchStats::default();

        let still = Body::new(cube(), Vec3::ZERO, Quat::IDENTITY)
            .march_world(origin, dir, 500.0, &mut stats)
            .expect("hit");
        // Rotate about the volume's own centre: translate the centre to the
        // origin, turn, and put it back.
        let turned = Body::new(
            cube(),
            centre - Quat::from_rotation_y(std::f32::consts::FRAC_PI_2) * centre,
            Quat::from_rotation_y(std::f32::consts::FRAC_PI_2),
        )
        .march_world(origin, dir, 500.0, &mut stats)
        .expect("hit");

        assert!(
            (turned.t - still.t).abs() < 1e-3,
            "a quarter turn of a symmetric cube changed the hit distance from {} to {}",
            still.t,
            turned.t
        );
    }

    /// The two transforms must actually invert each other, or every conversion
    /// in the renderer is subtly wrong in a way that looks like a physics bug.
    #[test]
    fn the_frames_are_inverses() {
        let body = Body::new(
            cube(),
            Vec3::new(5.0, -3.0, 11.0),
            Quat::from_euler(glam::EulerRot::XYZ, 0.3, -0.7, 1.1),
        );
        let p = Vec3::new(17.0, 4.0, -9.0);
        let round_trip = body.world_from_local().transform_point3(
            body.local_from_world().transform_point3(p),
        );
        assert!(
            (round_trip - p).length() < 1e-3,
            "a point round-tripped through both frames landed at {round_trip:?}, not {p:?}"
        );
    }

    /// Rotation must not rescale the ray, or `t` stops meaning the same thing
    /// in both frames and the renderer's nearest-hit comparison silently
    /// prefers whichever body happens to be scaled smaller.
    #[test]
    fn the_transform_does_not_rescale_a_direction() {
        let body = Body::new(
            cube(),
            Vec3::new(5.0, -3.0, 11.0),
            Quat::from_euler(glam::EulerRot::XYZ, 0.3, -0.7, 1.1),
        );
        let dir = Vec3::new(0.3, -0.9, 0.4).normalize();
        let local = body.local_from_world().transform_vector3(dir);
        assert!(
            (local.length() - 1.0).abs() < 1e-5,
            "a unit direction came out of the transform with length {}",
            local.length()
        );
    }
}
```

- [ ] **Step 2: Run the tests to verify they fail**

Run: `cargo test -p bevox_core --lib body`
Expected: FAIL to compile — `cannot find type Body`.

- [ ] **Step 3: Make `Contree` cloneable**

`Contree` derives neither `Clone` nor `Debug` today, so `Body` cannot derive them
and a test cannot hold two bodies built from the same volume. Add `Clone` to
`NodeArena` (its fields are all `Vec` and `Range`, so it derives cleanly) and
`Clone, Debug` to `Contree`. `NodeArena` already derives `Debug`.

This is the smallest change that unblocks the milestone, and `Debug` in
particular has been a friction point before — an earlier test had to use
`matches!` on a `Result` because `Contree` could not be formatted.

Cloning a `Contree` copies its whole arena, so it is not something to do per
frame. Bodies are small and are cloned when created, not when moved.

- [ ] **Step 4: Write the implementation**

Add above the test module:

```rust
use crate::contree::Contree;
use crate::march::{Hit, MarchStats, march};
use glam::{Affine3A, Quat, Vec3};

/// A voxel volume placed in the world by a rigid transform.
#[derive(Clone, Debug)]
pub struct Body {
    pub volume: Contree,
    pub position: Vec3,
    pub orientation: Quat,
}

impl Body {
    pub fn new(volume: Contree, position: Vec3, orientation: Quat) -> Self {
        Self { volume, position, orientation }
    }

    /// Volume space to world space.
    pub fn world_from_local(&self) -> Affine3A {
        Affine3A::from_rotation_translation(self.orientation, self.position)
    }

    /// World space to volume space.
    ///
    /// The inverse of a rigid transform is rigid, so this rescales nothing and
    /// a distance along the ray survives the trip unchanged.
    pub fn local_from_world(&self) -> Affine3A {
        self.world_from_local().inverse()
    }

    /// Marches a world-space ray through this body.
    ///
    /// `march` already takes the placement and does the conversion, which is
    /// what keeps the CPU a usable reference for bodies rather than only for
    /// the static world.
    pub fn march_world(
        &self,
        origin: Vec3,
        dir: Vec3,
        max_dist: f32,
        stats: &mut MarchStats,
    ) -> Option<Hit> {
        march(&self.volume, self.world_from_local(), origin, dir, max_dist, false, stats)
    }
}
```

Add `pub mod body;` to `crates/bevox_core/src/lib.rs`.

- [ ] **Step 5: Run the tests to verify they pass**

Run: `cargo test -p bevox_core`
Expected: PASS.

> If `a_rigid_rotation_preserves_the_hit_distance` fails, check whether `march` normalises `local_dir`. It must not: `t` is expressed in units of `dir`'s length, and renormalising after a rotation would be a no-op for a unit input but would hide a scale bug for any other.

- [ ] **Step 6: Commit**

```bash
git add crates/bevox_core
git commit -m "feat(core): a voxel volume placed by a rigid transform" -m "Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>"
```

---

### Task 2: Pack bodies into the GPU buffers

**Files:**
- Modify: `crates/bevox_render/src/upload.rs`, `crates/bevox_render/src/pipeline.rs`
- Modify: `crates/bevox_render/tests/common/mod.rs`

**Interfaces:**
- Produces: `#[repr(C)] pub struct GpuBody { pub local_from_world: [[f32; 4]; 4], pub rotation: [[f32; 4]; 4], pub node_base: u32, pub voxel_base: u32, pub depth: u32, pub extent: u32 }` (144 bytes), `GpuSceneData` gains `pub bodies: Vec<GpuBody>`, `MARCH_BINDING_COUNT` becomes 9, binding 8 is `array<GpuBody>`.
- Consumes: `bevox_core::body::Body`, `bevox_core::gpu::GpuVolume`.

> **A binding change touches four places that move together**: the layout tuple in `init_march_pipeline`, `MARCH_BINDING_COUNT`, the bind group entries in `dispatch_march`, and the harness's own layout in `tests/common/mod.rs`. The layout gate parses the shader for `@group(0) @binding(N)` and compares the set for exact equality, so **declare the shader binding in this task even though nothing reads it until Task 3**, or the gate fails at this commit.

- [ ] **Step 1: Write the failing tests**

Add to the test module in `crates/bevox_render/src/upload.rs`:

```rust
    #[test]
    fn a_gpu_body_is_the_size_the_shader_expects() {
        // Two mat4x4 (64 each) plus four u32 rounded to a 16-byte boundary.
        assert_eq!(size_of::<GpuBody>(), 144);
        assert_eq!(align_of::<GpuBody>(), 4);
    }

    #[test]
    fn packing_a_body_shifts_its_arena_past_the_static_world() {
        let world = Contree::empty(3);
        let body = bevox_core::body::Body::new(
            Contree::empty(2),
            Vec3::new(1.0, 2.0, 3.0),
            Quat::IDENTITY,
        );
        let packed = pack_bodies(&world, std::slice::from_ref(&body));

        let world_nodes = GpuVolume::from_contree(&world).buffer_nodes().len() as u32;
        assert_eq!(
            packed.bodies[0].node_base, world_nodes,
            "the body's root must sit immediately after the static world's nodes"
        );
        assert_eq!(
            packed.nodes.len() as u32,
            world_nodes + GpuVolume::from_contree(&body.volume).buffer_nodes().len() as u32,
            "the packed buffer must hold both volumes end to end"
        );
    }

    #[test]
    fn a_packed_body_carries_the_inverse_of_its_placement() {
        // The shader transforms rays world-to-local, so that is what it needs.
        let body = bevox_core::body::Body::new(
            Contree::empty(2),
            Vec3::new(4.0, 0.0, 0.0),
            Quat::IDENTITY,
        );
        let packed = pack_bodies(&Contree::empty(3), std::slice::from_ref(&body));
        let m = Mat4::from_cols_array_2d(&packed.bodies[0].local_from_world);
        let there_and_back = m.transform_point3(Vec3::new(4.0, 0.0, 0.0));
        assert!(
            there_and_back.length() < 1e-4,
            "the body's own position should map to its local origin, got {there_and_back:?}"
        );
    }

    #[test]
    fn a_scene_with_no_bodies_packs_exactly_the_static_world() {
        // The zero-body case must be byte-identical, because the whole feature
        // is required to leave a body-free scene untouched.
        let world = Contree::empty(3);
        let packed = pack_bodies(&world, &[]);
        let plain = GpuVolume::from_contree(&world);
        assert_eq!(packed.nodes, plain.buffer_nodes());
        assert_eq!(packed.voxels, plain.voxels);
        assert!(packed.bodies.is_empty());
    }
```

- [ ] **Step 2: Run them to verify they fail**

Run: `cargo test -p bevox_render --lib pack_bodies`
Expected: FAIL — `cannot find function pack_bodies`.

- [ ] **Step 3: Implement the packing**

In `crates/bevox_render/src/upload.rs`:

```rust
/// One body as the shader reads it.
///
/// Carries the inverse placement because that is the direction a ray travels —
/// world into local — and the rotation separately because a normal rotates
/// back without the translation.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, Pod, Zeroable)]
pub struct GpuBody {
    pub local_from_world: [[f32; 4]; 4],
    pub rotation: [[f32; 4]; 4],
    /// Index of this body's root in the shared node buffer. Its arena slot `n`
    /// is at `node_base + 1 + n`, matching how the static world is laid out.
    pub node_base: u32,
    pub voxel_base: u32,
    pub depth: u32,
    pub extent: u32,
}

/// The static world and every body, packed end to end into shared buffers.
pub struct PackedScene {
    pub nodes: Vec<GpuNode>,
    pub voxels: Vec<u32>,
    pub bodies: Vec<GpuBody>,
}

/// Packs the static world followed by each body.
///
/// One set of buffers rather than one per body: a body is small, and a second
/// pair of bindings per body would cap the count at whatever the device allows
/// rather than at what the frame budget allows.
pub fn pack_bodies(world: &Contree, bodies: &[bevox_core::body::Body]) -> PackedScene {
    let world_volume = GpuVolume::from_contree(world);
    let mut nodes = world_volume.buffer_nodes();
    let mut voxels = world_volume.voxels;
    let mut out = Vec::with_capacity(bodies.len());

    for body in bodies {
        let volume = GpuVolume::from_contree(&body.volume);
        let node_base = nodes.len() as u32;
        let voxel_base = voxels.len() as u32;
        nodes.extend_from_slice(&volume.buffer_nodes());
        voxels.extend_from_slice(&volume.voxels);

        out.push(GpuBody {
            local_from_world: Mat4::from(body.local_from_world()).to_cols_array_2d(),
            rotation: Mat4::from_quat(body.orientation).to_cols_array_2d(),
            node_base,
            voxel_base,
            depth: body.volume.depth(),
            extent: body.volume.extent(),
        });
    }

    PackedScene { nodes, voxels, bodies: out }
}
```

`GpuSceneData` gains `pub bodies: Vec<GpuBody>`; `Default` gives it an empty vec. `build_gpu_scene` calls `pack_bodies` and stores all three results.

In `crates/bevox_render/src/pipeline.rs`: raise `MARCH_BINDING_COUNT` to 9, add `storage_buffer_read_only_sized(false, NonZero::new(size_of::<GpuBody>() as u64))` as the ninth layout entry, add a `bodies: Buffer` to `MarchBuffers` with `STORAGE | COPY_DST`, bind it at 8, and add its bytes to `budget_bytes`. A zero-length storage buffer is invalid, so an empty body list uploads one zeroed `GpuBody` and the shader is told the count is zero.

`MarchUniform` gains the body count. It has a spare slot: put it in `volume_params.w`, which is currently an unused zero, rather than growing the struct again.

Declare the binding in `march.wgsl` now, unread:

```wgsl
struct GpuBody {
    local_from_world: mat4x4<f32>,
    rotation: mat4x4<f32>,
    node_base: u32,
    voxel_base: u32,
    depth: u32,
    extent: u32,
};
@group(0) @binding(8) var<storage, read> bodies: array<GpuBody>;
```

Mirror the layout and bind group in `crates/bevox_render/tests/common/mod.rs`.

- [ ] **Step 4: Run the tests**

Run: `cargo test --workspace`
Expected: PASS, including the layout gate and every existing parity test — nothing reads the binding yet, so no pixel may change.

- [ ] **Step 5: Commit**

```bash
git add crates
git commit -m "feat(render): pack rigid bodies into the scene buffers" -m "Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>"
```

---

### Task 3: Compose bodies into the march

**Files:**
- Modify: `crates/bevox_render/assets/shaders/march.wgsl`, `crates/bevox_render/src/upload.rs`

**Interfaces:**
- Produces: `march_flags::BODIES = 16`; `traverse_at` — `traverse` parameterised by the node and voxel bases so it can walk a body's tree.

- [ ] **Step 1: Write the gates**

Two tests, and they check opposite things. Add both to `crates/bevox_render/tests/gpu_parity.rs`:

```rust
/// A scene with no bodies must be untouched by the composition loop.
///
/// This is the one that protects everything built before this milestone: the
/// loop runs, finds nothing, and must leave every pixel exactly as it was.
#[test]
fn a_scene_with_no_bodies_is_bit_identical_with_bodies_enabled() {
    let Some((device, queue)) = gpu_device() else {
        eprintln!("no GPU adapter available, skipping");
        return;
    };
    let tree = parity_scene();
    let gpu_volume = GpuVolume::from_contree(&tree);
    let (width, height) = (96u32, 96u32);
    let eye = Vec3::new(-30.0, 40.0, -30.0);
    let view = Mat4::look_at_rh(eye, Vec3::new(32.0, 12.0, 32.0), Vec3::Y);
    let projection = Mat4::perspective_rh(0.9, 1.0, 0.1, 500.0);
    let world_from_clip = (projection * view).inverse();
    let shader = std::fs::read_to_string("assets/shaders/march.wgsl").expect("shader missing");

    for entry in ["march_identity", "march_voxel_id", "march_normal", "march"] {
        let without = run_march_flagged(
            &device, &queue, &shader, entry, world_from_clip, eye, &tree, &gpu_volume, width,
            height, march_flags::DEFAULT,
        );
        let with = run_march_flagged(
            &device, &queue, &shader, entry, world_from_clip, eye, &tree, &gpu_volume, width,
            height, march_flags::DEFAULT | march_flags::BODIES,
        );
        let differing = without.chunks(4).zip(with.chunks(4)).filter(|(a, b)| a != b).count();
        assert_eq!(
            differing, 0,
            "{entry}: enabling bodies changed {differing} pixels of a scene that has none"
        );
    }
}

/// A body must actually appear, and appear where its transform puts it.
///
/// The zero-body test above passes trivially if the loop never runs. This is
/// the one that proves it does: the same body at two different placements must
/// produce two different images, and both must differ from the empty scene.
#[test]
fn a_placed_body_appears_where_its_transform_puts_it() {
    let Some((device, queue)) = gpu_device() else {
        eprintln!("no GPU adapter available, skipping");
        return;
    };
    let (width, height) = (96u32, 96u32);
    let eye = Vec3::new(32.0, 32.0, -60.0);
    let view = Mat4::look_at_rh(eye, Vec3::splat(32.0), Vec3::Y);
    let projection = Mat4::perspective_rh(0.9, 1.0, 0.1, 500.0);
    let world_from_clip = (projection * view).inverse();
    let shader = std::fs::read_to_string("assets/shaders/march.wgsl").expect("shader missing");

    // An empty world so only the body can be seen.
    let world = Contree::empty(3);
    let cube = body_cube();

    let empty = run_bodies(
        &device, &queue, &shader, world_from_clip, eye, &world, &[], width, height,
    );
    let left = run_bodies(
        &device, &queue, &shader, world_from_clip, eye, &world,
        &[Body::new(cube.clone(), Vec3::new(-12.0, 0.0, 0.0), Quat::IDENTITY)],
        width, height,
    );
    let right = run_bodies(
        &device, &queue, &shader, world_from_clip, eye, &world,
        &[Body::new(cube, Vec3::new(12.0, 0.0, 0.0), Quat::IDENTITY)],
        width, height,
    );

    assert_ne!(empty, left, "a body was placed but nothing was drawn");
    assert_ne!(empty, right, "a body was placed but nothing was drawn");
    assert_ne!(left, right, "moving the body did not move what was drawn");
}
```

`body_cube()` is a 16-voxel solid cube in a 64 volume, as in Task 1. `run_bodies` is a harness helper mirroring `run_march_flagged` but taking a body list; it needs `Prepared::new` to accept bodies and pack them with `pack_bodies`.

- [ ] **Step 2: Run them to verify they fail**

Run: `cargo test -p bevox_render --test gpu_parity bodies`
Expected: FAIL — `march_flags::BODIES` does not exist.

- [ ] **Step 3: Implement the composition**

Add `pub const BODIES: u32 = 16;` to `march_flags`, leaving `DEFAULT` alone until Task 5 measures it.

`traverse` currently reads `nodes[...]` and the voxel array with the static world's layout baked in. Parameterise it:

```wgsl
/// `traverse`, told where a volume's nodes and voxels begin.
///
/// The static world is this with both bases zero. A body's root sits at
/// `node_base` and its arena slot n at `node_base + 1 + n`, which is the same
/// root-first layout the static world uses, just offset.
fn traverse_at(
    origin: vec3<f32>, dir: vec3<f32>, max_dist: f32,
    node_base: u32, voxel_base: u32, depth: u32, extent: u32,
) -> Hit { ... }
```

Keep `traverse(origin, dir, max_dist)` as a thin wrapper calling `traverse_at(..., 0u, 0u, view.volume_params.x, view.volume_params.y)`, so every existing call site and every parity test is untouched.

Then the composition, called from `primary_hit` after the static march:

```wgsl
/// The nearest of the static world's hit and every body's.
///
/// `t` is directly comparable because body transforms are rigid: the ray is
/// rotated and translated into the body's frame, never scaled, so a distance
/// means the same thing in both. The normal comes back through the rotation
/// alone -- putting a normal through the full affine would add the translation
/// and point it nowhere.
fn compose_bodies(origin: vec3<f32>, dir: vec3<f32>, world_hit: Hit, max_dist: f32) -> Hit {
    var best = world_hit;
    var limit = max_dist;
    if best.hit { limit = best.t; }

    let count = view.volume_params.w;
    for (var i = 0u; i < count; i = i + 1u) {
        let b = bodies[i];
        let local_origin = (b.local_from_world * vec4<f32>(origin, 1.0)).xyz;
        let local_dir = (b.local_from_world * vec4<f32>(dir, 0.0)).xyz;

        let h = traverse_at(local_origin, local_dir, limit, b.node_base, b.voxel_base, b.depth, b.extent);
        if h.hit && h.t < limit {
            best = h;
            best.face_normal = (b.rotation * vec4<f32>(h.face_normal, 0.0)).xyz;
            limit = h.t;
        }
    }
    return best;
}
```

Guard the whole loop with `flag_enabled(FLAG_BODIES)` so a body-free scene with the flag off runs exactly the code it ran before.

> The implicit normal is computed from neighbour occupancy in the static volume. A body's hit must use its own volume's neighbours, or its shading is read out of the wrong tree. For this milestone the body's face normal alone is acceptable — say so in the report, and note it for milestone 3.

- [ ] **Step 4: Run the tests**

Run: `cargo test --workspace`
Expected: PASS.

> **If `a_scene_with_no_bodies_is_bit_identical` fails**, the wrapper is not exactly equivalent to the old `traverse` — check the depth and extent it passes. **If `a_placed_body_appears` fails with all three images equal**, the loop is not running: check the count in `volume_params.w`. **If left and right are equal but differ from empty**, the transform is not being applied — check that the inverse, not the forward, placement was packed.

- [ ] **Step 5: Commit**

```bash
git add crates
git commit -m "perf(render): compose rigid bodies into the march" -m "Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>"
```

---

### Task 4: A spinning body in the app

**Files:**
- Modify: `crates/bevox/src/main.rs`, `crates/bevox_render/src/upload.rs`

- [ ] **Step 1: Add bodies to the scene resource**

`VoxelScene` gains `pub bodies: Vec<Body>`. `build_gpu_scene` packs them. Because a body's transform changes every frame while its geometry does not, **the body table is re-uploaded every frame and the geometry is not** — the table is a handful of 144-byte entries, so this is cheap, and conflating the two would drag the whole node buffer through a rebuild on every rotation.

State plainly in a comment that a body whose *geometry* changes needs the generation bump, and a body that merely moves does not.

- [ ] **Step 2: Spawn one and spin it**

In `crates/bevox/src/main.rs`, spawn a single cube body above the scene and rotate it:

```rust
/// Turns the demo body, so the composition can be seen working before any
/// physics exists. Milestone 2 replaces this with integration.
fn spin_bodies(time: Res<Time>, mut scene: ResMut<VoxelScene>) {
    let turn = Quat::from_rotation_y(time.delta_secs() * 0.7)
        * Quat::from_rotation_x(time.delta_secs() * 0.3);
    for body in &mut scene.bodies {
        body.orientation = (turn * body.orientation).normalize();
    }
}
```

Renormalising every step is not optional: a quaternion accumulated by repeated multiplication drifts off unit length, and the drift shows up as a body that slowly shears.

Set `march_flags::DEFAULT` to include `BODIES`.

- [ ] **Step 3: Build**

Run: `cargo build -p bevox --release`
Expected: builds with no warnings.

Do **not** run the app — it opens a window a human must close. Leave visual confirmation to the human and say so in the report.

- [ ] **Step 4: Commit**

```bash
git add crates
git commit -m "feat(bevox): spin a demo rigid body" -m "Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>"
```

---

### Task 5: Measure what a body costs

**Files:**
- Modify: `crates/bevox_render/tests/gpu_bench.rs`, this plan file

- [ ] **Step 1: Measure**

Add a benchmark that renders the bench scene with 0, 1, 4 and 16 bodies, A/B/A interleaved, reporting wall clock and GPU time. The question it answers is how the cost scales with body count, because the composition is `1 + N` marches per ray and the cap depends on the answer.

Run: `cargo test --release -p bevox_render --test gpu_bench bodies -- --nocapture`

- [ ] **Step 2: Record and decide the cap**

Add a Measurements section to this plan with the numbers, and set a body-count cap from them rather than from taste. State the per-body cost in milliseconds and what count fits a 16.7 ms frame alongside the static world.

> If a single body costs more than about 10% of the static world's march, say so plainly — it would mean the composition is too expensive for the milestones that follow and the design needs a bounding-volume test before the per-body march, not more bodies.

- [ ] **Step 3: Run the whole suite**

Run: `cargo test --workspace`
Expected: PASS.

- [ ] **Step 4: Commit**

```bash
git add crates docs
git commit -m "perf(render): measure what composing a body costs" -m "Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>"
```

---

## Milestone check

A voxel volume placed by a rigid transform renders, composed with the static world by nearest hit, at a measured cost — and a scene with no bodies renders bit-identically to before.

## What this plan deliberately does not do

No physics. The body spins from a hardcoded rotation; gravity, mass properties and contacts are milestone 2.

No implicit normals for bodies. A body shades from its face normal, which is flat where the static world is smooth. Milestone 3, alongside contacts.

No bounding-volume rejection before the per-body march. Task 5 measures whether it is needed rather than assuming it; adding it first would be optimising something unmeasured.

No body-vs-body anything. Nothing here needs two bodies to interact.

## Measurements

2026-09-17, NVIDIA GeForce GTX 1650, headless, release build. `bodies_are_measured_against_none` in `crates/bevox_render/tests/gpu_bench.rs`:

```
cargo test --release -p bevox_render --test gpu_bench bodies -- --ignored --nocapture
```

Bench scene (extent 1024 floor and columns), 1280x720, bench camera (close to geometry, looking along the floor), `march_flags::DEFAULT`. Bodies are the parity tests' 16-voxel cube (25..41 in a 64 volume), turned off-axis, in a 4x4 grid 120 voxels ahead in the open corridor; each count takes the first N. Every row is A/B/A against the zero-body scene, three rounds, medians: wall clock over batches of 30 dispatches, and GPU timestamps (median of 7 per reading). "px changed" diffs each count's image against the zero-body image, proving the bodies are on screen. The last row puts all 16 behind the camera, where no ray enters any body's box; it is not in the slope.

Raw output, verbatim:

```
scene: extent 1024, 1280x720, bench camera, flags DEFAULT, 16-voxel cube bodies
 0 bodies: wall  11.79 ms vs 11.80/11.79 (drift 0.00) =  -0.00 ms | gpu  11.75 ms vs 11.75/11.74 (drift 0.01) =  +0.00 ms |      0 px changed
 1 bodies: wall  15.03 ms vs 11.78/11.78 (drift 0.00) =  +3.25 ms | gpu  15.02 ms vs 11.75/11.75 (drift 0.00) =  +3.27 ms |  15663 px changed
 4 bodies: wall  24.39 ms vs 11.80/11.80 (drift 0.01) = +12.59 ms | gpu  24.39 ms vs 11.78/11.78 (drift 0.01) = +12.62 ms |  62257 px changed
16 bodies: wall  62.49 ms vs 11.82/11.83 (drift 0.01) = +50.67 ms | gpu  62.37 ms vs 11.79/11.79 (drift 0.01) = +50.58 ms | 268334 px changed
16 bodies behind the camera: wall  57.07 ms vs 11.78/11.78 (drift 0.00) = +45.29 ms | gpu  56.99 ms vs 11.75/11.73 (drift 0.02) = +45.25 ms |      0 px changed
static world: wall 11.80 ms, gpu 11.75 ms
per body (slope over 0/1/4/16): wall 3.165 ms = 26.8% of the static march, gpu 3.159 ms = 26.9%
bodies that fit a 16.7 ms frame beside the static world: wall 1.5, gpu 1.6
```

An earlier run in the same session, before the behind-the-camera row existed, gave a slope of 3.170 ms wall and 3.168 ms GPU against a static 11.78 / 11.75 ms.

**Per body: 3.17 ms.** Linear in the count: 3.25, 3.15 and 3.17 ms per body at 1, 4 and 16. Wall clock and GPU time agree to within 0.1 ms, so this is shader time, not submission overhead.

**Cap: one body.** 11.80 ms of static world plus 3.17 ms a body leaves room for 1.5 bodies in 16.7 ms. `MAX_BODIES = 1` in `crates/bevox_render/src/pipeline.rs`; `prepare_march_buffers` clamps the uniform's body count to it through `marched_body_count`, with `error_once!` when a scene has more. Bodies past the cap are still packed and uploaded, and simply not marched. The test harness writes its own uniform and is not capped, which is how this benchmark measures past it.

**One body costs 27% of the static world's march, well over the 10% line.** Composition as built is too expensive for the milestones that follow: at this camera on this card, the linear fit predicts two bodies at about 18.1 ms, past 16.7 ms. Two bodies were not measured. The cap should not go up.

**Where the cost is, and what the rejection has to beat.** Sixteen bodies that no ray comes near still cost 45.3 ms, 89% of the 50.7 ms that sixteen in view cost, so the cost is not per voxel visited. That row does not show it is the call, though. For every body, a pixel whose ray cannot hit it still pays, in `compose_bodies` and `traverse_at`:

1. a read of the whole `bodies[i]` from the storage buffer, both 4x4 matrices included, with a bounds check;
2. two matrix-vector multiplies, one for the origin and one for the direction, before any rejection;
3. the call into `traverse_at`: three divides for `1.0 / dir`, the root slab test, and a `Hit` returned by value. naga also declares every `var` at the top of the function, including the roughly 450-byte `stack`, which wgpu zeroes.

The benchmark cannot separate these. The root slab test in step 3 already is a bounding-volume test: the body's own box, in its own frame. Another test of the same kind in front of the march would still pay steps 1 and 2 and save only the call-entry share, which was not measured. What the evidence supports is rejecting a body before it is read and transformed, with a rejection cheaper than the read, transform and slab test it replaces, not merely placed in front of the march. The candidates, roughly from cheapest per pixel, none implemented:

- **CPU frustum cull.** Bodies outside the view never reach the uniform's count; the behind-the-camera row would drop to zero. It does nothing for a body on screen.
- **Per-body screen-space rectangle** tested against the pixel coordinate: a few integer compares, primary rays only.
- **World-space sphere or box** tested against the world ray, before the transform. It also serves shadow rays once they compose bodies.
- **Transforming the ray origin once per body per frame on the CPU.** For primary rays it is the camera position for every pixel, which removes one of the two multiplies.
- **Hoisting the slab test into `compose_bodies`.** This saves only the call overhead.

Neither where the time goes among steps 1 to 3, nor how much any candidate recovers, is measured here. Both belong to the task that adds one, re-measured with this benchmark.

What these numbers do not show. They are dispatch timings at one camera, not frame rates: no present, no vsync, no CPU frame work. One scene, one resolution, one GPU. Because most of the per-body cost is paid per pixel, it likely scales with resolution, so a cap chosen at 1280x720 depends on that resolution. One body shape, and a loose one: a 16-voxel cube inside a 64-voxel volume, so rays that enter the volume and miss the cube are in the count; a tighter volume would change the in-view cost, though by the behind-the-camera row not most of it. Primary rays only: shadow rays do not compose bodies yet, so bodies casting shadows would add to this rather than share it. The cap test proves `marched_body_count` clamps; it does not prove `prepare_march_buffers` calls it, which needs a render device no unit test has.
