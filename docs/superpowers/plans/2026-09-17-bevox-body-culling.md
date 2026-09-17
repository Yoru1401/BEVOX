# Body Culling Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Stop paying for rigid bodies a pixel cannot see, so the body cap can rise above 1 before physics needs several bodies.

**Architecture:** Two complementary rejections, both ahead of the work the measurement blamed. A CPU frustum cull removes whole bodies from the per-frame table, so an off-screen body costs nothing at all. A per-body screen-space rectangle, uploaded as its own compact array, lets the shader skip a visible body's read and transform for every pixel outside its footprint. Both are held to bit-identity and measured A/B/A, and the cap is then raised from the numbers.

**Tech Stack:** Rust, Bevy 0.19.1, wgpu 29.0.4, glam 0.32, WGSL.

**Spec:** `docs/superpowers/specs/2026-09-15-bevox-rigid-bodies-design.md`. The measurement this plan exists to answer is the "Measurements" section of `docs/superpowers/plans/2026-09-15-bevox-body-rendering.md`.

**Branch:** `feat/body-culling`, built on `feat/rigid-body-rendering`. **Physics work is not merged to master until Flori says so.** Finish this branch by keeping it and pushing it for testing, not by merging.

## Global Constraints

- Native desktop only. No web build, ever — permanently out of scope.
- `bevox_core` has no Bevy and no GPU dependency, and gains no new dependencies.
- No new dependencies in any crate.
- **Body transforms are rigid: rotation and translation, no scale.** Load-bearing — it keeps `t` comparable between a body's frame and the world's.
- Voxel data on the GPU is budgeted at **512 MB maximum**, checked at upload as an error rather than an allocation attempt.
- Tests use a seeded `bevox_core::testing::XorShift64`; no test framework may be added.
- Every performance claim comes from interleaved A/B/A within one session, drift reported.
- **An optimisation that changes a pixel is a defect, not a trade-off.** The only accepted exception is the documented grazing allowance on shadow-ray outputs. Culling must be bit-identical for primary rays.
- **A zero-body scene must still render bit-identically.**
- Verify with `cargo test --workspace` (about 2 minutes).

## What the measurement actually says

This plan is argued from a measurement, so read it first. At 1280x720 on a GTX 1650, each body costs 3.17 ms against an 11.80 ms static world — 27% per body — so `MAX_BODIES` is 1.

**Sixteen bodies behind the camera, reachable by no ray, cost 89% of what sixteen visible ones cost.** For every pixel and every body, `compose_bodies` reads the whole 144-byte `bodies[i]` and does two matrix-vector multiplies *before* `traverse_at` gets to its root slab test. The cost is paid whether the ray can hit the body or not.

That rules out the obvious fix. **`traverse_at`'s root slab test already is a bounding-volume test** — the body's own box, in its own frame. Adding another bounding test in front of the march would still pay the read and the transforms. A rejection only helps if it is **cheaper than the read and transform it replaces, and runs before them.**

Nobody measured how the cost splits between the read, the transforms and the call. So each rejection here is behind its own flag and measured separately; the plan does not assume which recovers most.

## Facts that are not obvious from the code

**The app and the tests use different projection conventions.** Test cameras use `Mat4::perspective_rh`: standard depth, near maps to 0, a finite far plane. The app's camera is Bevy's default, `perspective_infinite_reverse_rh`: **reverse-Z**, near maps to 1, and the far plane is at infinity. So the near and far planes you would pull straight out of a clip matrix mean opposite things in the two, and the far plane does not exist in the app at all. **A cull tested only against the harness's cameras can pass every test and be wrong in the running app.** Two consequences, both mandatory:
- Cull only against the four **side** planes — left, right, bottom, top. They come from `row3 ± row0` and `row3 ± row1`, involve only x, y and w, and are identical under both conventions.
- Replace the near plane with a **behind-the-camera** test using the camera position and its forward direction. Never cull against a far plane: not culling distant bodies is merely conservative.

**The forward direction is safe to unproject in both conventions, but only one way.** The shader's `primary_ray` unprojects clip `z = 1`. Under standard depth that is the far plane; under reverse-Z it is the near plane. Both points lie in front of the camera, so `normalize(unproject(0, 0, 1) - position)` is the true forward direction in both. Unprojecting `z = 0` is not safe: under reverse-Z that is the plane at infinity.

**The harness has its own path, and this project has shipped a bug through that before.** The GPU test harness builds its body table itself instead of using the render world's staging. A culling function the app calls but the harness does not would leave every GPU gate testing the unculled path. **The culling decision must be one pure function that the render world and the harness both call.** Two rounds of the previous milestone went into closing exactly this gap for the buffer-reuse decision.

**The body table is written in two places in the render world.** `prepare_march_buffers` writes it on the reuse path from `SceneUpdate::bodies` and on the rebuild path from `GpuSceneData::bodies`. Both must cull. Extract one helper that both call.

**A culled table is compacted, so every per-body array must be compacted identically.** The shader loops over `count` entries. Each `GpuBody` carries its own `node_base` and `voxel_base`, so dropping entries is safe for the geometry. But the rectangle array added in Task 3 is indexed in step with the table, and must be built by the same function, in the same order, or body `i` is tested against another body's rectangle.

**Frustum culling is wrong for shadows, and must not be reused for them.** Bodies do not cast shadows yet. When they do, a body just outside the view can shadow geometry inside it. Say so in a comment on the cull, so shadow composition does not inherit it.

**The pixel mapping must match `primary_ray` exactly, or the rectangle is off by half a pixel.** `primary_ray` takes pixel `id` to NDC as `x = (id.x + 0.5) / width * 2 - 1` and `y = 1 - (id.y + 0.5) / height * 2`. The rectangle inverts that, rounds **outward** (floor the minimum, ceil the maximum), and pads by one pixel. A rectangle that rounds inward drops a column of a body's edge.

## File Structure

- `crates/bevox_core/src/body.rs` — gains `occupied_bounds`, a body's tightest node-granular box in its own frame. Pure; no GPU.
- `crates/bevox_render/src/cull.rs` — **new**. The frustum cull and the screen rectangle, as pure functions over a camera and a body's world-space bound. Its own file because it is the one piece of this milestone testable without a GPU, and both the render world and the harness must call the same code.
- `crates/bevox_render/src/lib.rs` — declare the module.
- `crates/bevox_render/src/upload.rs` — carry each body's local bound into `GpuSceneData`.
- `crates/bevox_render/src/pipeline.rs` — cull at both table-write sites; binding 9 for rectangles; the cap on visible bodies.
- `crates/bevox_render/assets/shaders/march.wgsl` — the rectangle early-out in `compose_bodies`.
- `crates/bevox_render/tests/{common/mod.rs,gpu_parity.rs,gpu_bench.rs}` — the harness calls the same cull; gates; measurement.

---

### Task 1: A body's occupied bounds

A cull is only as good as its bound. The previous measurement used a 16-voxel cube inside a 64-voxel volume; bounding by the volume would make that body look fifteen times larger on screen than it is.

**Files:**
- Modify: `crates/bevox_core/src/body.rs`

**Interfaces:**
- Produces: `pub fn occupied_bounds(tree: &Contree) -> Option<(UVec3, UVec3)>` — the minimum corner inclusive and the maximum corner exclusive, in the volume's own voxel coordinates; `None` for an empty volume.
- Consumes: `bevox_core::contree::{Contree, level_extent}`, `bevox_core::node::{Node, CHILDREN}`, `NodeArena::node`.

- [ ] **Step 1: Write the failing tests**

Add to the test module in `crates/bevox_core/src/body.rs`:

```rust
    #[test]
    fn an_empty_volume_has_no_bounds() {
        assert_eq!(occupied_bounds(&Contree::empty(3)), None);
    }

    /// The bound must contain every occupied voxel. That is the whole promise:
    /// a cull that trusts a bound missing a voxel removes a body that is on
    /// screen, and the pixels change.
    #[test]
    fn the_bounds_contain_every_occupied_voxel() {
        let mut rng = crate::testing::XorShift64::new(9_17);
        let mut voxels = Vec::new();
        for _ in 0..60 {
            voxels.push((
                UVec3::new(rng.next_below(64), rng.next_below(64), rng.next_below(64)),
                MaterialId(1),
            ));
        }
        let tree = Contree::from_voxels(64, &voxels);
        let (lo, hi) = occupied_bounds(&tree).expect("the volume has voxels");
        for (p, _) in &voxels {
            assert!(
                p.cmpge(lo).all() && p.cmplt(hi).all(),
                "voxel {p:?} lies outside the bounds {lo:?}..{hi:?}"
            );
        }
    }

    /// Node-granular, so conservative by at most one brick (4 voxels) per side
    /// -- and no looser, or a small body is bounded as a large one and the
    /// rectangle cull stops helping.
    ///
    /// Builds its own cube rather than using `cube()`. That helper fills 24..40,
    /// which is aligned to the brick grid and so collapses to uniform nodes whose
    /// boxes happen to be exact -- it would not exercise the rounding at all.
    /// 25..41 is deliberately off-grid, so the bound must round outward.
    #[test]
    fn the_bounds_are_tight_to_within_a_brick() {
        let mut dense = DenseVolume::new(64).unwrap();
        for z in 25..41 {
            for y in 25..41 {
                for x in 25..41 {
                    dense.set(UVec3::new(x, y, z), MaterialId(1));
                }
            }
        }
        let tree = dense.into_contree();
        let (lo, hi) = occupied_bounds(&tree).expect("the cube has voxels");
        for axis in 0..3 {
            assert!(lo[axis] <= 25 && lo[axis] + 4 > 25, "min {lo:?} is looser than a brick");
            assert!(hi[axis] >= 41 && hi[axis] < 41 + 4, "max {hi:?} is looser than a brick");
        }
    }
```

- [ ] **Step 2: Run to verify they fail**

Run: `cargo test -p bevox_core --lib occupied_bounds`
Expected: FAIL to compile — `cannot find function occupied_bounds`.

- [ ] **Step 3: Implement**

Add to `crates/bevox_core/src/body.rs`, above the test module:

```rust
/// The tightest box holding a volume's geometry, in its own voxel coordinates:
/// minimum inclusive, maximum exclusive. `None` for an empty volume.
///
/// Node-granular: it stops at the first node that is uniform or no larger than
/// a brick, so it can be up to one brick loose per side. That is conservative,
/// which is the only direction a cull may err in, and tight enough that a small
/// body is not bounded as its whole volume.
pub fn occupied_bounds(tree: &Contree) -> Option<(UVec3, UVec3)> {
    let mut lo = UVec3::splat(u32::MAX);
    let mut hi = UVec3::ZERO;
    let mut any = false;
    grow_bounds(tree, tree.root(), tree.depth() - 1, UVec3::ZERO, &mut lo, &mut hi, &mut any);
    any.then_some((lo, hi))
}

fn grow_bounds(
    tree: &Contree,
    node: crate::node::Node,
    level: u32,
    origin: UVec3,
    lo: &mut UVec3,
    hi: &mut UVec3,
    any: &mut bool,
) {
    if node.is_empty() {
        return;
    }
    let extent = crate::contree::level_extent(level);
    // A brick-sized or uniform node takes its whole box. Level 0 is a brick, so
    // this also stops the walk before voxel children, which live in another
    // arena and have no `Node` to recurse on.
    if level == 0 || !node.is_subdivided() {
        *lo = lo.min(origin);
        *hi = hi.max(origin + UVec3::splat(extent));
        *any = true;
        return;
    }
    let step = crate::contree::level_extent(level - 1);
    for i in 0..crate::node::CHILDREN {
        if let Some(slot) = node.child_slot(i) {
            // Inverse of child_index: x + y * 4 + z * 16.
            let c = UVec3::new(i % 4, (i / 4) % 4, i / 16);
            grow_bounds(tree, tree.arena().node(slot), level - 1, origin + c * step, lo, hi, any);
        }
    }
}
```

- [ ] **Step 4: Run to verify they pass, and prove the containment test can fail**

Run: `cargo test -p bevox_core`
Expected: PASS.

Then break it: change `*hi = hi.max(origin + UVec3::splat(extent));` to `*hi = hi.max(origin);`. Confirm `the_bounds_contain_every_occupied_voxel` FAILS. Restore. Report both states.

- [ ] **Step 5: Commit**

```bash
git add crates/bevox_core
git commit -m "feat(core): a volume's occupied bounds, for culling" -m "Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>"
```

---

### Task 2: Cull whole bodies the camera cannot see

**Files:**
- Create: `crates/bevox_render/src/cull.rs`
- Modify: `crates/bevox_render/src/lib.rs`, `crates/bevox_render/src/upload.rs`, `crates/bevox_render/src/pipeline.rs`
- Modify: `crates/bevox_render/tests/common/mod.rs`, `crates/bevox_render/tests/gpu_parity.rs`

**Interfaces:**
- Produces: `pub struct BodyBound { pub centre: Vec3, pub radius: f32 }`; `pub fn world_bound(local: (UVec3, UVec3), body: &Body) -> BodyBound`; `pub struct Frustum`; `impl Frustum { pub fn from_camera(world_from_clip: Mat4, position: Vec3) -> Self; pub fn sees(&self, bound: BodyBound) -> bool }`; `pub fn visible_bodies(table: &[GpuBody], bounds: &[BodyBound], frustum: &Frustum) -> Vec<GpuBody>`; `march_flags::CULL_BODIES = 32`.
- Consumes: `bevox_core::body::{Body, occupied_bounds}` from Task 1.

`GpuSceneData` gains `pub body_local_bounds: Vec<Option<(UVec3, UVec3)>>`, filled once per rebuild from `occupied_bounds`, in the same order as `bodies`. It is geometry, so it changes only on a rebuild; the world-space bound is recomputed every frame from the live transform.

- [ ] **Step 1: Write the failing tests**

Create `crates/bevox_render/src/cull.rs` with this test module:

```rust
//! Deciding, on the CPU, which bodies a camera could possibly see.
//!
//! Only primary rays. Frustum culling is wrong for shadows: a body just outside
//! the view can shadow geometry inside it. Shadow composition must not reuse
//! this.

#[cfg(test)]
mod tests {
    use super::*;

    fn standard(eye: Vec3, target: Vec3) -> Mat4 {
        let view = Mat4::look_at_rh(eye, target, Vec3::Y);
        let projection = Mat4::perspective_rh(0.9, 16.0 / 9.0, 0.1, 500.0);
        (projection * view).inverse()
    }

    /// Bevy's default projection: reverse-Z, far plane at infinity. The app
    /// uses this; the GPU harness does not.
    fn reverse_z(eye: Vec3, target: Vec3) -> Mat4 {
        let view = Mat4::look_at_rh(eye, target, Vec3::Y);
        let projection = Mat4::perspective_infinite_reverse_rh(0.9, 16.0 / 9.0, 0.1);
        (projection * view).inverse()
    }

    #[test]
    fn a_body_straight_ahead_is_seen() {
        let eye = Vec3::ZERO;
        for world_from_clip in [standard(eye, -Vec3::Z), reverse_z(eye, -Vec3::Z)] {
            let f = Frustum::from_camera(world_from_clip, eye);
            assert!(f.sees(BodyBound { centre: Vec3::new(0.0, 0.0, -50.0), radius: 5.0 }));
        }
    }

    #[test]
    fn a_body_behind_the_camera_is_culled() {
        let eye = Vec3::ZERO;
        for world_from_clip in [standard(eye, -Vec3::Z), reverse_z(eye, -Vec3::Z)] {
            let f = Frustum::from_camera(world_from_clip, eye);
            assert!(!f.sees(BodyBound { centre: Vec3::new(0.0, 0.0, 50.0), radius: 5.0 }));
        }
    }

    #[test]
    fn a_body_far_to_the_side_is_culled() {
        let eye = Vec3::ZERO;
        for world_from_clip in [standard(eye, -Vec3::Z), reverse_z(eye, -Vec3::Z)] {
            let f = Frustum::from_camera(world_from_clip, eye);
            assert!(!f.sees(BodyBound { centre: Vec3::new(500.0, 0.0, -50.0), radius: 5.0 }));
        }
    }

    /// Straddling an edge is seen. A cull that removed a body half on screen
    /// would change pixels, which is the one thing it must never do.
    #[test]
    fn a_body_straddling_a_side_plane_is_seen() {
        let eye = Vec3::ZERO;
        for world_from_clip in [standard(eye, -Vec3::Z), reverse_z(eye, -Vec3::Z)] {
            let f = Frustum::from_camera(world_from_clip, eye);
            // At z = -50 the horizontal half-width is 50 * tan(0.45) * 16/9 ~ 43.
            assert!(f.sees(BodyBound { centre: Vec3::new(45.0, 0.0, -50.0), radius: 5.0 }));
        }
    }

    /// A body around the camera itself, straddling the behind-the-camera plane,
    /// is seen.
    #[test]
    fn a_body_enclosing_the_camera_is_seen() {
        let eye = Vec3::ZERO;
        for world_from_clip in [standard(eye, -Vec3::Z), reverse_z(eye, -Vec3::Z)] {
            let f = Frustum::from_camera(world_from_clip, eye);
            assert!(f.sees(BodyBound { centre: Vec3::new(0.0, 0.0, 3.0), radius: 5.0 }));
        }
    }

    /// The two conventions must agree on every body, because the app runs one
    /// and every GPU gate runs the other. A disagreement is a cull that passes
    /// all the tests and breaks the running app.
    ///
    /// The two frustums come from two different matrices, each re-inverted, so
    /// their planes differ by float error. A body sitting within that error of a
    /// plane can legitimately land on either side. If this test ever fails, check
    /// the margin first: exclude only bodies whose signed distance to the nearest
    /// plane is within 1e-3 of their radius, and never widen the cull itself to
    /// make it pass.
    #[test]
    fn standard_and_reverse_z_agree_everywhere() {
        let mut rng = bevox_core::testing::XorShift64::new(17);
        let eye = Vec3::new(3.0, 7.0, -2.0);
        let target = Vec3::new(40.0, -10.0, -90.0);
        let a = Frustum::from_camera(standard(eye, target), eye);
        let b = Frustum::from_camera(reverse_z(eye, target), eye);
        let mut seen = 0;
        for _ in 0..4000 {
            let c = Vec3::new(
                rng.next_below(400) as f32 - 200.0,
                rng.next_below(400) as f32 - 200.0,
                rng.next_below(400) as f32 - 200.0,
            );
            let bound = BodyBound { centre: c, radius: rng.next_below(20) as f32 + 1.0 };
            assert_eq!(a.sees(bound), b.sees(bound), "conventions disagree about {bound:?}");
            seen += a.sees(bound) as u32;
        }
        // Not vacuous: some bodies must be seen and some culled.
        assert!(seen > 100 && seen < 3900, "{seen} of 4000 seen; the sample does not exercise the cull");
    }
}
```

Add the far-plane caveat as a test comment, not an assertion: under reverse-Z there is no far plane, so the standard-Z frustum must not cull on its far plane either, or `standard_and_reverse_z_agree_everywhere` fails for distant bodies.

- [ ] **Step 2: Run to verify they fail**

Run: `cargo test -p bevox_render --lib cull`
Expected: FAIL to compile.

- [ ] **Step 3: Implement the cull**

Above the tests in `cull.rs`:

```rust
use crate::upload::GpuBody;
use bevox_core::body::Body;
use glam::{Mat4, UVec2, UVec3, Vec2, Vec3, Vec4, Vec4Swizzles};

/// A body's bounding sphere in world space.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct BodyBound {
    pub centre: Vec3,
    pub radius: f32,
}

/// Places a body's local occupied box in the world as a sphere.
///
/// A sphere because a rigid transform leaves its radius unchanged, so it needs
/// no re-fitting when the body turns; the box's half-diagonal bounds it in
/// every orientation.
pub fn world_bound(local: (UVec3, UVec3), body: &Body) -> BodyBound {
    let lo = local.0.as_vec3();
    let hi = local.1.as_vec3();
    BodyBound {
        centre: body.world_from_local().transform_point3((lo + hi) * 0.5),
        radius: (hi - lo).length() * 0.5,
    }
}

/// The four side planes and a behind-the-camera plane.
///
/// No near or far plane from the matrix: the app's camera is reverse-Z with the
/// far plane at infinity, the test harness's is standard-Z, and those planes
/// mean opposite things in the two. The side planes use only x, y and w, so
/// they are the same in both.
pub struct Frustum {
    /// Inward-facing, as (normal, distance); a point p is inside when
    /// dot(normal, p) + distance >= 0.
    sides: [Vec4; 4],
    position: Vec3,
    forward: Vec3,
}

impl Frustum {
    pub fn from_camera(world_from_clip: Mat4, position: Vec3) -> Self {
        let clip_from_world = world_from_clip.inverse();
        let r0 = clip_from_world.row(0);
        let r1 = clip_from_world.row(1);
        let r3 = clip_from_world.row(3);
        let normalise = |p: Vec4| p / p.xyz().length();

        // z = 1 is in front of the camera in both conventions: the far plane
        // under standard-Z, the near plane under reverse-Z. z = 0 is not -- under
        // reverse-Z it is the plane at infinity.
        let ahead = world_from_clip * Vec4::new(0.0, 0.0, 1.0, 1.0);
        let forward = (ahead.xyz() / ahead.w - position).normalize();

        Self {
            sides: [
                normalise(r3 + r0),
                normalise(r3 - r0),
                normalise(r3 + r1),
                normalise(r3 - r1),
            ],
            position,
            forward,
        }
    }

    /// Whether any part of the sphere could be on screen. Errs towards yes.
    pub fn sees(&self, bound: BodyBound) -> bool {
        if (bound.centre - self.position).dot(self.forward) < -bound.radius {
            return false;
        }
        self.sides
            .iter()
            .all(|p| p.xyz().dot(bound.centre) + p.w >= -bound.radius)
    }
}

/// The table entries for the bodies this frustum could see, in table order.
///
/// Compacted: each `GpuBody` carries its own geometry bases, so dropping entries
/// leaves the rest valid. Any other per-body array must be built from the same
/// kept indices, in the same order.
pub fn visible_bodies(table: &[GpuBody], bounds: &[BodyBound], frustum: &Frustum) -> Vec<GpuBody> {
    table
        .iter()
        .zip(bounds)
        .filter(|(_, b)| frustum.sees(**b))
        .map(|(g, _)| *g)
        .collect()
}
```

Declare `pub mod cull;` in `crates/bevox_render/src/lib.rs`.

- [ ] **Step 4: Wire it in, in all three places**

1. **`GpuSceneData`** gains `pub body_local_bounds: Vec<Option<(UVec3, UVec3)>>`, filled by `pack_bodies` from `occupied_bounds`, same order as `bodies`. A body with no voxels is culled unconditionally.
2. **`prepare_march_buffers`**: extract one helper that takes the placed table, the local bounds and the camera, computes world bounds, and — when `march_flags::DEFAULT` includes `CULL_BODIES` — returns `visible_bodies(...)`. Call it at **both** table-write sites, the reuse path and the rebuild path. The uniform's body count is `marched_body_count(visible.len())`, so the cap applies to visible bodies.
3. **The test harness**: `Prepared::new` takes the flags already. When they include `CULL_BODIES`, build the table through the **same helper**, using the harness camera. Do not reimplement the cull in the harness.

Add `pub const CULL_BODIES: u32 = 32;` to `march_flags`. It is CPU-only; the shader never reads it. Say so in its doc comment. Leave `DEFAULT` unchanged until Task 4 measures.

- [ ] **Step 5: The GPU gate**

Add to `crates/bevox_render/tests/gpu_parity.rs` a test rendering `march_identity`, `march_normal` and `march` with bodies placed **fully in view, straddling each side of the frustum, and fully behind the camera**, with and without `CULL_BODIES`. Assert every pixel identical. Assert at least one body was genuinely culled and at least one genuinely kept, by calling the cull directly on the same inputs — or the test proves nothing about the cull.

- [ ] **Step 6: Run, then prove each gate can fail**

Run: `cargo test --workspace`
Expected: PASS.

Break the cull two ways, confirming a test fails each time, and restore:
- Change `>= -bound.radius` to `>= 0.0` in `sees` (drops straddling bodies). The straddling unit test and the GPU gate must FAIL.
- Build the frustum with `row2` as a near plane instead of the behind-the-camera test. `standard_and_reverse_z_agree_everywhere` must FAIL.

Report both.

- [ ] **Step 7: Commit**

```bash
git add crates
git commit -m "perf(render): cull rigid bodies outside the view on the CPU" -m "Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>"
```

---

### Task 3: Skip a visible body's read and transform outside its screen footprint

A visible body is still read and transformed for every pixel on screen. Its footprint is usually a small fraction of that.

**Files:**
- Modify: `crates/bevox_render/src/cull.rs`, `crates/bevox_render/src/pipeline.rs`, `crates/bevox_render/src/upload.rs`
- Modify: `crates/bevox_render/assets/shaders/march.wgsl`
- Modify: `crates/bevox_render/tests/common/mod.rs`, `crates/bevox_render/tests/gpu_parity.rs`

**Interfaces:**
- Produces: `#[repr(C)] pub struct GpuBodyRect { pub min: [u32; 2], pub max: [u32; 2] }` (16 bytes); `pub fn screen_rect(local: (UVec3, UVec3), body: &Body, world_from_clip: Mat4, size: UVec2) -> GpuBodyRect`; binding 9, `array<GpuBodyRect>`; `march_flags::BODY_RECT = 64`; `MARCH_BINDING_COUNT` becomes 10.
- Consumes: Task 2's cull helper, so rectangles are built from the same kept bodies in the same order.

> **Why a separate binding and not a field on `GpuBody`.** The measurement could not tell whether reading one field of a storage-buffer struct loads the whole struct; that is up to the driver. A separate 16-byte array makes the cheap test's read cost certain. Growing `GpuBody` instead would leave the result depending on something this project cannot observe.

> **A binding change touches four places that move together**: the layout tuple in `init_march_pipeline`, `MARCH_BINDING_COUNT`, the bind group entries in `dispatch_march`, and the harness's separate layout in `tests/common/mod.rs`. The layout gate catches the first two disagreeing with the shader; nothing catches the harness drifting, so check it by hand.

- [ ] **Step 1: Write the failing tests**

In `cull.rs`'s test module:

```rust
    fn cube_body(position: Vec3) -> (Body, (UVec3, UVec3)) {
        let volume = bevox_core::contree::Contree::empty(3);
        (Body::new(volume, position, glam::Quat::IDENTITY), (UVec3::splat(24), UVec3::splat(40)))
    }

    /// The rectangle must cover every pixel whose ray could hit the body. It is
    /// checked against the same pixel-to-ray mapping `primary_ray` uses, so a
    /// half-pixel slip is caught here rather than as a missing column on screen.
    #[test]
    fn the_rectangle_covers_every_pixel_that_sees_the_body() {
        let size = UVec2::new(160, 90);
        let eye = Vec3::new(32.0, 32.0, -80.0);
        for world_from_clip in [standard(eye, Vec3::splat(32.0)), reverse_z(eye, Vec3::splat(32.0))] {
            let (body, local) = cube_body(Vec3::ZERO);
            let rect = screen_rect(local, &body, world_from_clip, size);
            let bound = world_bound(local, &body);

            let mut inside = 0;
            for y in 0..size.y {
                for x in 0..size.x {
                    let ndc = Vec2::new(
                        (x as f32 + 0.5) / size.x as f32 * 2.0 - 1.0,
                        1.0 - (y as f32 + 0.5) / size.y as f32 * 2.0,
                    );
                    let far = world_from_clip * Vec4::new(ndc.x, ndc.y, 1.0, 1.0);
                    let dir = (far.xyz() / far.w - eye).normalize();
                    // A ray passing within the sphere may hit the body.
                    let to = bound.centre - eye;
                    let along = to.dot(dir);
                    let near_miss = (to - dir * along).length() <= bound.radius && along > 0.0;
                    if near_miss {
                        inside += 1;
                        assert!(
                            x >= rect.min[0] && x <= rect.max[0] && y >= rect.min[1] && y <= rect.max[1],
                            "pixel ({x}, {y}) can see the body but lies outside the rectangle {rect:?}"
                        );
                    }
                }
            }
            assert!(inside > 50, "only {inside} pixels see the body; the test is not exercising the rectangle");
        }
    }

    /// Straddling the camera plane must not produce a garbage rectangle from a
    /// corner behind the camera.
    #[test]
    fn a_body_around_the_camera_gets_the_whole_screen() {
        let size = UVec2::new(160, 90);
        let eye = Vec3::new(32.0, 32.0, 32.0);
        let (body, local) = cube_body(Vec3::ZERO);
        let rect = screen_rect(local, &body, standard(eye, Vec3::new(32.0, 32.0, 100.0)), size);
        assert_eq!(rect.min, [0, 0]);
        assert_eq!(rect.max, [size.x - 1, size.y - 1]);
    }
```

- [ ] **Step 2: Run to verify they fail**

Run: `cargo test -p bevox_render --lib screen_rect`
Expected: FAIL to compile.

- [ ] **Step 3: Implement the rectangle**

```rust
/// A body's footprint on screen, in pixels, inclusive at both ends.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, bytemuck::Pod, bytemuck::Zeroable)]
pub struct GpuBodyRect {
    pub min: [u32; 2],
    pub max: [u32; 2],
}

/// The pixels a body can cover: its eight world-space corners projected and
/// rounded outward, padded by one.
///
/// Inverts `primary_ray`'s mapping exactly. A corner at or behind the camera
/// has no meaningful projection, so the body gets the whole screen -- only
/// conservative answers are allowed.
pub fn screen_rect(local: (UVec3, UVec3), body: &Body, world_from_clip: Mat4, size: UVec2) -> GpuBodyRect {
    let whole = GpuBodyRect { min: [0, 0], max: [size.x - 1, size.y - 1] };
    let clip_from_world = world_from_clip.inverse();
    let world_from_local = body.world_from_local();
    let (lo, hi) = (local.0.as_vec3(), local.1.as_vec3());

    let mut min = Vec2::splat(f32::INFINITY);
    let mut max = Vec2::splat(f32::NEG_INFINITY);
    for i in 0..8 {
        let corner = Vec3::new(
            if i & 1 == 0 { lo.x } else { hi.x },
            if i & 2 == 0 { lo.y } else { hi.y },
            if i & 4 == 0 { lo.z } else { hi.z },
        );
        let clip = clip_from_world * world_from_local.transform_point3(corner).extend(1.0);
        if clip.w <= 1e-6 {
            return whole;
        }
        let ndc = clip.xy() / clip.w;
        let px = Vec2::new(
            (ndc.x + 1.0) * 0.5 * size.x as f32 - 0.5,
            (1.0 - ndc.y) * 0.5 * size.y as f32 - 0.5,
        );
        min = min.min(px);
        max = max.max(px);
    }

    let clamp = |v: f32, n: u32| (v.max(0.0) as u32).min(n - 1);
    GpuBodyRect {
        min: [clamp(min.x.floor() - 1.0, size.x), clamp(min.y.floor() - 1.0, size.y)],
        max: [clamp(max.x.ceil() + 1.0, size.x), clamp(max.y.ceil() + 1.0, size.y)],
    }
}
```

- [ ] **Step 4: The shader early-out**

Add binding 9 and test the rectangle **before** reading `bodies[i]`:

```wgsl
struct GpuBodyRect { min: vec2<u32>, max: vec2<u32> };
@group(0) @binding(9) var<storage, read> body_rects: array<GpuBodyRect>;
const FLAG_BODY_RECT: u32 = 64u;
```

`compose_bodies` needs the pixel, so give it `id: vec2<u32>` and pass it from `primary_hit`. At the top of the loop body:

```wgsl
        // Cheaper than the read and the two transforms below, and ahead of
        // them. `traverse_at`'s slab test already rejects a ray that misses the
        // body's box, but only after that work has been paid.
        if flag_enabled(FLAG_BODY_RECT) {
            let r = body_rects[i];
            if id.x < r.min.x || id.x > r.max.x || id.y < r.min.y || id.y > r.max.y {
                continue;
            }
        }
```

Build the rectangles in the same helper as Task 2's cull, from the **same kept bodies in the same order**, and upload them alongside the table at both write sites. An empty list uploads one zeroed rectangle, as the body table does. Mirror the binding in the harness.

- [ ] **Step 5: The GPU gate**

Render bodies near and far, partly off each edge, and rotated, through `march_identity`, `march_normal` and `march`, with and without `BODY_RECT`. Assert every pixel identical. Assert, by counting, that the rectangle actually excludes some pixels from each body — or the gate cannot tell a working early-out from one that never fires.

- [ ] **Step 6: Run, then prove the gate can fail**

Run: `cargo test --workspace`
Expected: PASS.

Break it: drop the one-pixel pad and round inward (`ceil` the minimum, `floor` the maximum). The GPU gate or the covering unit test must FAIL. Restore. Report both.

- [ ] **Step 7: Commit**

```bash
git add crates
git commit -m "perf(render): skip a body's read and transform outside its screen footprint" -m "Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>"
```

---

### Task 4: Measure, and raise the cap from the numbers

**Files:**
- Modify: `crates/bevox_render/tests/gpu_bench.rs`, `crates/bevox_render/src/upload.rs`, `crates/bevox_render/src/pipeline.rs`
- Modify: this plan file

- [ ] **Step 1: Measure every combination, A/B/A**

Extend the existing body benchmark. Keep its 0/1/4/16 counts and its behind-the-camera row, and run each against the zero-body baseline with: neither, `CULL_BODIES` alone, `BODY_RECT` alone, and both. Report wall clock, GPU time and drift for every row.

The behind-the-camera row is the direct test of Task 2: with `CULL_BODIES` it should cost almost nothing, since those bodies never reach the table. If it does not, say so — it would mean the remaining cost is not in the per-body loop at all.

Add one row the previous milestone lacked: **a tight body.** Its measurement used a 16-voxel cube in a 64-voxel volume, which Task 1's occupied bounds now handle. Report whether `BODY_RECT`'s gain depends on body size.

Run: `cargo test --release -p bevox_render --test gpu_bench bodies -- --ignored --nocapture`

- [ ] **Step 2: Set the defaults and the cap**

Add `CULL_BODIES` and `BODY_RECT` to `DEFAULT` only if each measured faster than drift, and say which in `DEFAULT`'s doc comment.

Then raise `MAX_BODIES` from the measured per-visible-body cost: how many visible bodies of the benchmark's size fit in a 16.7 ms frame alongside the 11.80 ms static world. Cite the number in the doc comment. **If it is still 1, leave it at 1 and say so plainly** — that is a finding, not a failure, and it points at the next rejection on the list: a world-space sphere tested against the world ray, which would also serve shadow rays.

- [ ] **Step 3: Record**

Add a Measurements section to this plan: the raw numbers verbatim, GTX 1650, the date, and what they do and do not show. They are dispatch timings at one camera and resolution, not frame rates, and the cap is resolution-dependent because most of the per-body cost is paid per pixel.

- [ ] **Step 4: Run the whole suite**

Run: `cargo test --workspace`
Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add crates docs
git commit -m "perf(render): measure body culling and raise the body cap" -m "Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>"
```

---

## Milestone check

An off-screen body costs nothing, an on-screen body is read and transformed only for pixels near its footprint, both are bit-identical, and `MAX_BODIES` is set from a measurement.

## What this plan deliberately does not do

No shadow composition for bodies, and the frustum cull must never be reused for it — an off-screen body can shadow on-screen geometry.

No world-space sphere test against the ray in the shader. It is the next rejection if this one leaves the cap at 1, and it is the one that would also serve shadow rays, but it adds a per-ray test for every body where the rectangle adds a per-pixel integer compare.

No level of detail, no body sleeping, no occlusion culling. A body hidden behind the static world still costs its footprint.

No merge to master. Physics stays on its own branch until Flori says otherwise.
