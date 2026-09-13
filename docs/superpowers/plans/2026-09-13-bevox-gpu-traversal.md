# BEVOX GPU traversal — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Put the voxel scene on screen, ray marched by our own WGSL compute shader, and prove that shader agrees with the CPU reference marcher pixel for pixel.

**Architecture:** `bevox_core` gains a GPU byte layout: nodes flattened to four `u32`s each (WGSL has no portable `u64`), leaf voxel bytes packed four per word. A Bevy plugin uploads those buffers, dispatches a compute shader into a storage-texture `Image`, and displays that image with an ordinary sprite, so Bevy does the compositing and no custom blit is needed. Correctness is established by a headless wgpu harness that runs the same shader outside Bevy and compares its output against `bevox_core::march`.

**Tech Stack:** Bevy 0.19.1, wgpu (via Bevy), WGSL, bytemuck, pollster for the headless harness.

**Spec:** `docs/superpowers/specs/2026-09-13-bevox-raymarcher-core-design.md`
**Predecessor:** `docs/superpowers/plans/2026-09-13-bevox-core-and-reference-marcher.md` (milestones 1–2, complete)

## Global Constraints

- Rust edition 2024, `rust-version = "1.95"`. Native desktop only; never add a `wasm32` path.
- Bevy 0.19.1. **The render-graph registration API is the one piece of this plan not verified against documentation.** Task 4 Step 1 is a discovery step that reads it from the installed crate before any code depends on it. If what you find disagrees with the code below, the discovery wins — fix the plan, do not bend the code around a guess.
- `MaterialId(0)` is empty, everywhere.
- WGSL has no portable `u64` and no byte-addressable storage. Every 64-bit mask crosses as two `u32`s, every voxel byte as a quarter of a `u32`. This is a translation layer, not a redesign: `bevox_core`'s in-memory types do not change.
- The CPU reference marcher is ground truth. When GPU and CPU disagree, the GPU is wrong until proven otherwise.
- Every WGSL loop carries a hard iteration bound. An unbounded loop in a compute shader hangs the GPU and takes the desktop with it — strictly worse than the wrong-answer failure the same bug caused on the CPU in Plan 1.
- Shaders are authored as `.wgsl` files under `crates/bevox_render/assets/shaders/`, loaded by path, never inlined as strings.

## File Structure

| File | Responsibility |
|---|---|
| `crates/bevox_core/src/gpu.rs` | GPU byte layout: `GpuNode`, packing, `GpuVolume` |
| `crates/bevox_render/Cargo.toml` | Bevy plugin crate manifest |
| `crates/bevox_render/src/lib.rs` | `BevoxRenderPlugin`, public wiring |
| `crates/bevox_render/src/camera.rs` | Fly camera controller |
| `crates/bevox_render/src/upload.rs` | Extract + prepare: buffers and uniforms into the render world |
| `crates/bevox_render/src/pipeline.rs` | Bind group layout, compute pipeline, dispatch system |
| `crates/bevox_render/assets/shaders/march.wgsl` | The traversal shader |
| `crates/bevox_render/tests/gpu_parity.rs` | Headless wgpu harness and the parity test |
| `crates/bevox/Cargo.toml` | Binary manifest |
| `crates/bevox/src/main.rs` | App: window, camera, scene, plugin |

---

### Task 1: Workspace crates and a window

**Files:**
- Modify: `Cargo.toml`
- Create: `crates/bevox_render/Cargo.toml`, `crates/bevox_render/src/lib.rs`
- Create: `crates/bevox/Cargo.toml`, `crates/bevox/src/main.rs`

**Interfaces:**
- Consumes: nothing from earlier plans yet.
- Produces: `BevoxRenderPlugin` implementing `bevy::prelude::Plugin`; marker resource `BevoxReady` proving the plugin ran.

- [ ] **Step 1: Add the crates to the workspace**

`Cargo.toml` members become:

```toml
members = ["crates/bevox_core", "crates/bevox_render", "crates/bevox"]
```

Create the two manifests, then add Bevy:

```bash
cargo add bevy@0.19.1 -p bevox_render
cargo add bevy@0.19.1 -p bevox
cargo add bevox_core --path crates/bevox_core -p bevox_render
cargo add bevox_render --path crates/bevox_render -p bevox
cargo tree -d
```

Expected: `cargo tree -d` reports no duplicate `glam` — Bevy and `bevox_core` must agree on it. If they do not, align `bevox_core`'s glam to the version Bevy pulls and re-run.

- [ ] **Step 2: Write the failing test**

`crates/bevox_render/src/lib.rs`:

```rust
//! Bevy plugin: uploads voxel data and ray marches it on the GPU.

use bevy::prelude::*;

/// Inserted by the plugin so tests can prove it built.
#[derive(Resource, Debug, PartialEq, Eq)]
pub struct BevoxReady;

pub struct BevoxRenderPlugin;

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_plugin_builds_on_a_minimal_app() {
        let mut app = App::new();
        // MinimalPlugins deliberately has no RenderApp, which is the point of
        // this test. InputPlugin is added because the camera system needs it.
        app.add_plugins(MinimalPlugins)
            .add_plugins(bevy::input::InputPlugin)
            .add_plugins(BevoxRenderPlugin);
        app.update();
        assert!(app.world().get_resource::<BevoxReady>().is_some());
    }
}
```

- [ ] **Step 3: Run the test to verify it fails**

Run: `cargo test -p bevox_render`
Expected: FAIL — `BevoxRenderPlugin` does not implement `Plugin`.

- [ ] **Step 4: Implement the plugin**

Add to `crates/bevox_render/src/lib.rs`, above the test module:

```rust
impl Plugin for BevoxRenderPlugin {
    fn build(&self, app: &mut App) {
        app.insert_resource(BevoxReady);
    }
}
```

- [ ] **Step 5: Run the test to verify it passes**

Run: `cargo test -p bevox_render`
Expected: PASS, 1 test.

- [ ] **Step 6: Write the binary**

`crates/bevox/src/main.rs`:

```rust
use bevy::prelude::*;
use bevox_render::BevoxRenderPlugin;

fn main() {
    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "BEVOX".into(),
                resolution: (1280u32, 720u32).into(),
                ..default()
            }),
            ..default()
        }))
        .add_plugins(BevoxRenderPlugin)
        .add_systems(Startup, setup)
        .run();
}

fn setup(mut commands: Commands) {
    commands.spawn(Camera2d);
}
```

- [ ] **Step 7: Confirm the window opens**

Run: `cargo run -p bevox --release`
Expected: a 1280x720 window titled BEVOX opens and stays open. Close it to end the run.

- [ ] **Step 8: Commit**

```bash
git add Cargo.toml Cargo.lock crates
git commit -m "feat(render): add bevy plugin and binary crates" -m "Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>"
```

---

### Task 2: GPU byte layout in `bevox_core`

**Files:**
- Create: `crates/bevox_core/src/gpu.rs`
- Modify: `crates/bevox_core/src/lib.rs`, `crates/bevox_core/Cargo.toml`

**Interfaces:**
- Consumes: `Contree`, `Node`, `NodeArena` from Plan 1.
- Produces: `GpuNode { mask_lo, mask_hi, child_base, material }` (16 bytes, `Pod`); `GpuVolume::from_contree(&Contree) -> GpuVolume` with fields `nodes: Vec<GpuNode>`, `voxels: Vec<u32>`, `root: GpuNode`, `depth: u32`; methods `node_bytes(&self) -> &[u8]`, `voxel_bytes(&self) -> &[u8]`, `buffer_nodes(&self) -> Vec<GpuNode>` (root first, arena shifted by one); free functions `pack_voxels(&[u8]) -> Vec<u32>` and `unpack_voxel(&[u32], usize) -> u8`.

- [ ] **Step 1: Add bytemuck**

```bash
cargo add bytemuck --features derive -p bevox_core
```

- [ ] **Step 2: Write the failing tests**

`crates/bevox_core/src/gpu.rs`:

```rust
//! Translation to GPU-friendly layout.
//!
//! WGSL has no portable 64-bit integer and no byte-addressable storage, so
//! masks cross as two u32 halves and voxel bytes are packed four per word.
//! This is a translation layer only: the in-memory types are unchanged.

use crate::contree::Contree;
use crate::node::Node;
use bytemuck::{Pod, Zeroable};

#[repr(C)]
#[derive(Clone, Copy, Debug, PartialEq, Eq, Pod, Zeroable, Default)]
pub struct GpuNode {
    pub mask_lo: u32,
    pub mask_hi: u32,
    pub child_base: u32,
    pub material: u32,
}

#[derive(Clone, Debug)]
pub struct GpuVolume {
    pub nodes: Vec<GpuNode>,
    pub voxels: Vec<u32>,
    pub root: GpuNode,
    pub depth: u32,
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::dense::DenseVolume;
    use crate::material::MaterialId;
    use glam::UVec3;

    #[test]
    fn gpu_node_is_sixteen_bytes() {
        assert_eq!(size_of::<GpuNode>(), 16);
    }

    #[test]
    fn a_mask_survives_the_split_into_halves() {
        let node = Node::subdivided(0xDEAD_BEEF_1234_5678, 42);
        let gpu = GpuNode::from(node);
        assert_eq!(gpu.mask_lo, 0x1234_5678);
        assert_eq!(gpu.mask_hi, 0xDEAD_BEEF);
        assert_eq!(gpu.child_base, 42);
        assert_eq!(
            (gpu.mask_hi as u64) << 32 | gpu.mask_lo as u64,
            0xDEAD_BEEF_1234_5678
        );
    }

    #[test]
    fn the_top_and_bottom_mask_bits_both_survive() {
        let node = Node::subdivided(1u64 << 63 | 1, 0);
        let gpu = GpuNode::from(node);
        assert_eq!(gpu.mask_lo, 1);
        assert_eq!(gpu.mask_hi, 1 << 31);
    }

    #[test]
    fn voxels_pack_four_to_a_word_and_unpack_again() {
        let bytes: Vec<u8> = (0..10u8).map(|i| i * 7 + 1).collect();
        let packed = pack_voxels(&bytes);
        assert_eq!(packed.len(), 3, "10 bytes need 3 words");
        for (i, b) in bytes.iter().enumerate() {
            assert_eq!(unpack_voxel(&packed, i), *b, "byte {i}");
        }
    }

    #[test]
    fn packing_an_empty_slice_yields_no_words() {
        assert!(pack_voxels(&[]).is_empty());
    }

    #[test]
    fn a_volume_converts_with_its_arena_intact() {
        let mut dense = DenseVolume::new(16).unwrap();
        dense.set(UVec3::new(3, 4, 5), MaterialId(9));
        dense.set(UVec3::new(9, 1, 2), MaterialId(4));
        let tree = Contree::from_dense(&dense);
        let gpu = GpuVolume::from_contree(&tree);

        assert_eq!(gpu.depth, tree.depth());
        assert_eq!(gpu.nodes.len(), tree.arena().nodes().len());
        assert_eq!(gpu.root, GpuNode::from(tree.root()));
        // Every arena node survived the conversion.
        for (i, n) in tree.arena().nodes().iter().enumerate() {
            assert_eq!(gpu.nodes[i], GpuNode::from(*n), "node {i}");
        }
        // Every voxel byte is recoverable from the packed words.
        for (i, b) in tree.arena().voxels().iter().enumerate() {
            assert_eq!(unpack_voxel(&gpu.voxels, i), *b, "voxel {i}");
        }
    }

    #[test]
    fn byte_views_have_the_expected_lengths() {
        let tree = Contree::from_dense(&DenseVolume::new(16).unwrap());
        let gpu = GpuVolume::from_contree(&tree);
        assert_eq!(gpu.node_bytes().len(), gpu.nodes.len() * 16);
        assert_eq!(gpu.voxel_bytes().len(), gpu.voxels.len() * 4);
    }
}
```

- [ ] **Step 3: Run the tests to verify they fail**

Run: `cargo test -p bevox_core gpu`
Expected: FAIL — no `From<Node>` for `GpuNode`, no `pack_voxels`, no `unpack_voxel`, no `from_contree`.

- [ ] **Step 4: Write the implementation**

Insert into `crates/bevox_core/src/gpu.rs`, above the test module:

```rust
impl From<Node> for GpuNode {
    fn from(node: Node) -> Self {
        GpuNode {
            mask_lo: node.mask as u32,
            mask_hi: (node.mask >> 32) as u32,
            child_base: node.child_base,
            material: node.material,
        }
    }
}

/// Packs voxel bytes four to a word, little-endian within the word.
pub fn pack_voxels(bytes: &[u8]) -> Vec<u32> {
    let mut words = vec![0u32; bytes.len().div_ceil(4)];
    for (i, b) in bytes.iter().enumerate() {
        words[i / 4] |= (*b as u32) << ((i % 4) * 8);
    }
    words
}

/// Reads back a single voxel byte from packed words.
pub fn unpack_voxel(words: &[u32], index: usize) -> u8 {
    ((words[index / 4] >> ((index % 4) * 8)) & 0xFF) as u8
}

impl GpuVolume {
    pub fn from_contree(tree: &Contree) -> Self {
        GpuVolume {
            nodes: tree.arena().nodes().iter().copied().map(GpuNode::from).collect(),
            voxels: pack_voxels(tree.arena().voxels()),
            root: GpuNode::from(tree.root()),
            depth: tree.depth(),
        }
    }

    pub fn node_bytes(&self) -> &[u8] {
        bytemuck::cast_slice(&self.nodes)
    }

    pub fn voxel_bytes(&self) -> &[u8] {
        bytemuck::cast_slice(&self.voxels)
    }
}
```

Add to `crates/bevox_core/src/lib.rs`:

```rust
pub mod gpu;
```

- [ ] **Step 5: Run the tests to verify they pass**

Run: `cargo test -p bevox_core`
Expected: PASS — 6 new tests, plus all 64 from Plan 1 still green.

- [ ] **Step 6: Commit**

```bash
git add crates/bevox_core Cargo.lock
git commit -m "feat(core): add GPU byte layout for nodes and packed voxels" -m "Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>"
```

---

### Task 3: Fly camera

**Files:**
- Create: `crates/bevox_render/src/camera.rs`
- Modify: `crates/bevox_render/src/lib.rs`

**Interfaces:**
- Consumes: `BevoxRenderPlugin`.
- Produces: `FlyCamera { speed: f32, sensitivity: f32, yaw: f32, pitch: f32 }` (a `Component`, `Default`); `apply_look(yaw: f32, pitch: f32, delta: Vec2, sensitivity: f32) -> (f32, f32)`; `movement_vector(yaw: f32, forward: f32, right: f32, up: f32) -> Vec3`.

The maths is separated from the Bevy wiring so it can be tested without a window.

- [ ] **Step 1: Write the failing tests**

`crates/bevox_render/src/camera.rs`:

```rust
//! Fly camera. The arithmetic lives in free functions so it is testable
//! without a window, an input device or a running app.

use bevy::prelude::*;
use std::f32::consts::FRAC_PI_2;

/// Slightly under a right angle, so looking straight up never flips the basis.
const PITCH_LIMIT: f32 = FRAC_PI_2 - 0.001;

#[derive(Component, Debug, Clone, Copy)]
pub struct FlyCamera {
    pub speed: f32,
    pub sensitivity: f32,
    pub yaw: f32,
    pub pitch: f32,
}

impl Default for FlyCamera {
    fn default() -> Self {
        Self { speed: 24.0, sensitivity: 0.003, yaw: 0.0, pitch: 0.0 }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn looking_up_is_clamped_below_a_right_angle() {
        let (_, pitch) = apply_look(0.0, 0.0, Vec2::new(0.0, -10_000.0), 0.003);
        assert!(pitch <= PITCH_LIMIT, "pitch {pitch} exceeded the limit");
        assert!(pitch > 0.0);
    }

    #[test]
    fn looking_down_is_clamped_above_the_negative_right_angle() {
        let (_, pitch) = apply_look(0.0, 0.0, Vec2::new(0.0, 10_000.0), 0.003);
        assert!(pitch >= -PITCH_LIMIT, "pitch {pitch} exceeded the limit");
        assert!(pitch < 0.0);
    }

    #[test]
    fn yaw_accumulates_and_is_unbounded() {
        let (yaw, _) = apply_look(1.0, 0.0, Vec2::new(100.0, 0.0), 0.01);
        assert!((yaw - 1.0).abs() > 1e-6, "yaw should have moved off its input");
        // Turning right then left returns to the start.
        let (back, _) = apply_look(yaw, 0.0, Vec2::new(-100.0, 0.0), 0.01);
        assert!((back - 1.0).abs() < 1e-5, "yaw did not return, got {back}");
    }

    #[test]
    fn forward_movement_at_zero_yaw_points_down_negative_z() {
        let v = movement_vector(0.0, 1.0, 0.0, 0.0);
        assert!((v - Vec3::NEG_Z).length() < 1e-5, "got {v:?}");
    }

    #[test]
    fn a_quarter_turn_of_yaw_sends_forward_along_negative_x() {
        let v = movement_vector(FRAC_PI_2, 1.0, 0.0, 0.0);
        assert!((v - Vec3::NEG_X).length() < 1e-5, "got {v:?}");
    }

    #[test]
    fn vertical_movement_ignores_yaw() {
        let a = movement_vector(0.0, 0.0, 0.0, 1.0);
        let b = movement_vector(2.3, 0.0, 0.0, 1.0);
        assert!((a - Vec3::Y).length() < 1e-5);
        assert!((a - b).length() < 1e-5, "up should not depend on yaw");
    }

    #[test]
    fn no_input_produces_no_movement() {
        assert_eq!(movement_vector(1.2, 0.0, 0.0, 0.0), Vec3::ZERO);
    }
}
```

- [ ] **Step 2: Run the tests to verify they fail**

Run: `cargo test -p bevox_render camera`
Expected: FAIL — `apply_look` and `movement_vector` not found.

- [ ] **Step 3: Write the implementation**

Insert into `crates/bevox_render/src/camera.rs`, above the test module:

```rust
/// Applies mouse motion to a yaw/pitch pair. Pitch is clamped; yaw is free.
pub fn apply_look(yaw: f32, pitch: f32, delta: Vec2, sensitivity: f32) -> (f32, f32) {
    let new_yaw = yaw - delta.x * sensitivity;
    let new_pitch = (pitch - delta.y * sensitivity).clamp(-PITCH_LIMIT, PITCH_LIMIT);
    (new_yaw, new_pitch)
}

/// Movement direction in world space for the given yaw and per-axis inputs.
/// Not normalised: the caller scales by speed and delta time.
pub fn movement_vector(yaw: f32, forward: f32, right: f32, up: f32) -> Vec3 {
    let rotation = Quat::from_rotation_y(yaw);
    let f = rotation * Vec3::NEG_Z;
    let r = rotation * Vec3::X;
    let v = f * forward + r * right + Vec3::Y * up;
    if v.length_squared() > 1e-6 { v.normalize() } else { Vec3::ZERO }
}

/// Reads input and moves any entity carrying a [`FlyCamera`].
pub fn fly_camera_system(
    time: Res<Time>,
    keys: Res<ButtonInput<KeyCode>>,
    mouse: Res<ButtonInput<MouseButton>>,
    mut motion: MessageReader<bevy::input::mouse::MouseMotion>,
    mut query: Query<(&mut Transform, &mut FlyCamera)>,
) {
    let mut delta = Vec2::ZERO;
    for m in motion.read() {
        delta += m.delta;
    }

    for (mut transform, mut cam) in &mut query {
        if mouse.pressed(MouseButton::Right) {
            let (yaw, pitch) = apply_look(cam.yaw, cam.pitch, delta, cam.sensitivity);
            cam.yaw = yaw;
            cam.pitch = pitch;
        }
        transform.rotation = Quat::from_rotation_y(cam.yaw) * Quat::from_rotation_x(cam.pitch);

        let forward = axis(&keys, KeyCode::KeyW, KeyCode::KeyS);
        let right = axis(&keys, KeyCode::KeyD, KeyCode::KeyA);
        let up = axis(&keys, KeyCode::Space, KeyCode::ShiftLeft);
        let boost = if keys.pressed(KeyCode::ControlLeft) { 4.0 } else { 1.0 };

        let motion = movement_vector(cam.yaw, forward, right, up);
        transform.translation += motion * cam.speed * boost * time.delta_secs();
    }
}

fn axis(keys: &ButtonInput<KeyCode>, positive: KeyCode, negative: KeyCode) -> f32 {
    (keys.pressed(positive) as i32 as f32) - (keys.pressed(negative) as i32 as f32)
}
```

> **Verify before trusting:** `MessageReader` and `bevy::input::mouse::MouseMotion` are the 0.19 spellings of what older Bevy called `EventReader`/`MouseMotion` events. If the crate disagrees, run
> `cargo doc -p bevy_input --no-deps --open` and use what is actually there. The free functions above are pure maths and are unaffected either way.

Register it in `crates/bevox_render/src/lib.rs`:

```rust
pub mod camera;

impl Plugin for BevoxRenderPlugin {
    fn build(&self, app: &mut App) {
        app.insert_resource(BevoxReady)
            .add_systems(Update, camera::fly_camera_system);
    }
}
```

- [ ] **Step 4: Run the tests to verify they pass**

Run: `cargo test -p bevox_render`
Expected: PASS, 8 tests (7 camera + the plugin smoke test).

- [ ] **Step 5: Commit**

```bash
git add crates/bevox_render
git commit -m "feat(render): add fly camera with testable movement maths" -m "Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>"
```

---

### Task 4: Compute pipeline writing a flat colour — milestone 3

This is the plumbing task. Its first step exists because the render-graph
registration API is the one thing in this plan not verified from documentation.

**Files:**
- Create: `crates/bevox_render/src/upload.rs`, `crates/bevox_render/src/pipeline.rs`
- Create: `crates/bevox_render/assets/shaders/march.wgsl`
- Modify: `crates/bevox_render/src/lib.rs`, `crates/bevox/src/main.rs`

**Interfaces:**
- Consumes: `GpuVolume` from Task 2.
- Produces: `VoxelScene { tree: Contree }` (a `Resource`); `MarchTarget { image: Handle<Image> }` (a `Resource`); `MarchUniform` (`Pod`, 96 bytes) with fields `world_from_clip: [[f32; 4]; 4]`, `camera_position: [f32; 4]`, `volume_params: [u32; 4]`; `MarchPipeline` (a `Resource`) holding `layout: BindGroupLayout` and `pipeline: CachedComputePipelineId`.

- [ ] **Step 1: Discover the render-graph API before writing code against it**

```bash
cargo doc -p bevy_render --no-deps
```

Then find how a compute dispatch is registered in this exact version. Search the installed source rather than guessing:

```bash
rg -n "RenderGraph" --type rust "$(cargo metadata --format-version 1 | python -c "import json,sys;print(json.load(sys.stdin)['packages'][0]['manifest_path'])" | xargs dirname)" | head
rg -n "add_systems\(RenderGraph" ~/.cargo/registry/src -g '*.rs' | head -20
```

Record the answer in this task before continuing: the schedule label, the
ordering label for the camera driver, and the import paths. The code in Step 4
assumes `render_app.add_systems(RenderGraph, dispatch.before(camera_driver))`,
which matches a documented 0.19 example, but the imports must be confirmed.

- [ ] **Step 2: Write the failing tests**

`crates/bevox_render/src/upload.rs`:

```rust
//! Moving voxel data and camera parameters into the render world.

use bevy::prelude::*;
use bevy::render::extract_resource::ExtractResource;
use bevox_core::contree::Contree;
use bytemuck::{Pod, Zeroable};

/// The scene the renderer draws. Replacing it re-uploads on the next frame.
#[derive(Resource)]
pub struct VoxelScene {
    pub tree: Contree,
    /// Bumped whenever `tree` changes, so the render world knows to re-upload.
    pub generation: u32,
}

/// Camera and volume parameters, as the shader sees them.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, Pod, Zeroable)]
pub struct MarchUniform {
    pub world_from_clip: [[f32; 4]; 4],
    pub camera_position: [f32; 4],
    /// `[depth, extent, 0, 0]`.
    pub volume_params: [u32; 4],
}

/// The storage texture the compute shader writes and a sprite displays.
#[derive(Resource, Clone, ExtractResource)]
pub struct MarchTarget {
    pub image: Handle<Image>,
    pub width: u32,
    pub height: u32,
}

#[cfg(test)]
mod tests {
    use super::*;
    use bevox_core::contree::Contree;

    #[test]
    fn the_uniform_is_the_size_the_shader_expects() {
        // mat4x4 (64) + vec4 (16) + uvec4 (16)
        assert_eq!(size_of::<MarchUniform>(), 96);
        assert_eq!(align_of::<MarchUniform>(), 4);
    }

    #[test]
    fn volume_params_carry_depth_and_extent() {
        let tree = Contree::empty(3);
        let u = march_uniform(Mat4::IDENTITY, Vec3::ZERO, &tree);
        assert_eq!(u.volume_params[0], 3);
        assert_eq!(u.volume_params[1], 64);
    }

    #[test]
    fn the_camera_position_survives_into_the_uniform() {
        let tree = Contree::empty(2);
        let u = march_uniform(Mat4::IDENTITY, Vec3::new(1.0, 2.0, 3.0), &tree);
        assert_eq!(u.camera_position[0], 1.0);
        assert_eq!(u.camera_position[1], 2.0);
        assert_eq!(u.camera_position[2], 3.0);
    }

    #[test]
    fn the_matrix_is_stored_column_major_as_wgsl_expects() {
        let m = Mat4::from_translation(Vec3::new(5.0, 6.0, 7.0));
        let tree = Contree::empty(2);
        let u = march_uniform(m, Vec3::ZERO, &tree);
        // glam is column-major, and to_cols_array_2d yields columns.
        assert_eq!(u.world_from_clip[3][0], 5.0);
        assert_eq!(u.world_from_clip[3][1], 6.0);
        assert_eq!(u.world_from_clip[3][2], 7.0);
    }
}
```

- [ ] **Step 3: Run the tests to verify they fail**

Run: `cargo test -p bevox_render upload`
Expected: FAIL — `march_uniform` not found.

- [ ] **Step 4: Write the uniform builder**

Insert into `crates/bevox_render/src/upload.rs`, above the test module:

```rust
/// Builds the shader uniform from a camera and the volume being drawn.
///
/// `world_from_clip` is the inverse view-projection: the shader multiplies a
/// clip-space point by it to get a world-space ray target.
pub fn march_uniform(world_from_clip: Mat4, camera_position: Vec3, tree: &Contree) -> MarchUniform {
    MarchUniform {
        world_from_clip: world_from_clip.to_cols_array_2d(),
        camera_position: camera_position.extend(0.0).to_array(),
        volume_params: [tree.depth(), tree.extent(), 0, 0],
    }
}
```

- [ ] **Step 5: Run the tests to verify they pass**

Run: `cargo test -p bevox_render upload`
Expected: PASS, 4 tests.

- [ ] **Step 6: Write the flat-colour shader**

`crates/bevox_render/assets/shaders/march.wgsl`:

```wgsl
// Ray marcher. This revision writes a flat colour so the plumbing can be
// verified before any traversal logic exists.

struct MarchUniform {
    world_from_clip: mat4x4<f32>,
    camera_position: vec4<f32>,
    // [depth, extent, 0, 0]
    volume_params: vec4<u32>,
};

@group(0) @binding(0) var<uniform> view: MarchUniform;
@group(0) @binding(1) var<storage, read> nodes: array<vec4<u32>>;
@group(0) @binding(2) var<storage, read> voxels: array<u32>;
@group(0) @binding(3) var output: texture_storage_2d<rgba8unorm, write>;

@compute @workgroup_size(8, 8, 1)
fn march(@builtin(global_invocation_id) id: vec3<u32>) {
    let size = textureDimensions(output);
    if id.x >= size.x || id.y >= size.y {
        return;
    }

    // Flat colour, plus a faint gradient so a stuck frame is obvious.
    let uv = vec2<f32>(f32(id.x) / f32(size.x), f32(id.y) / f32(size.y));
    textureStore(output, vec2<i32>(id.xy), vec4<f32>(0.15, 0.35 + uv.y * 0.2, 0.6, 1.0));
}
```

- [ ] **Step 7: Write the pipeline and dispatch**

`crates/bevox_render/src/pipeline.rs`:

```rust
//! Bind group layout, compute pipeline and the dispatch that runs it.

use crate::upload::{MarchTarget, MarchUniform};
use bevy::prelude::*;
use bevy::render::render_asset::RenderAssets;
use bevy::render::render_resource::binding_types::{
    storage_buffer_read_only, texture_storage_2d, uniform_buffer,
};
use bevy::render::render_resource::*;
use bevy::render::renderer::{RenderDevice, RenderQueue};
use bevy::render::texture::GpuImage;

pub const SHADER_PATH: &str = "shaders/march.wgsl";
pub const WORKGROUP: u32 = 8;

#[derive(Resource)]
pub struct MarchPipeline {
    pub layout: BindGroupLayout,
    pub pipeline: CachedComputePipelineId,
}

/// GPU-side buffers for the current scene.
#[derive(Resource)]
pub struct MarchBuffers {
    pub uniform: Buffer,
    pub nodes: Buffer,
    pub voxels: Buffer,
    pub generation: u32,
}

pub fn init_march_pipeline(
    mut commands: Commands,
    device: Res<RenderDevice>,
    asset_server: Res<AssetServer>,
    pipeline_cache: Res<PipelineCache>,
) {
    let layout = device.create_bind_group_layout(
        "bevox_march_layout",
        &BindGroupLayoutEntries::sequential(
            ShaderStages::COMPUTE,
            (
                uniform_buffer::<MarchUniform>(false),
                storage_buffer_read_only::<Vec<u32>>(false),
                storage_buffer_read_only::<Vec<u32>>(false),
                texture_storage_2d(TextureFormat::Rgba8Unorm, StorageTextureAccess::WriteOnly),
            ),
        ),
    );

    let shader = asset_server.load(SHADER_PATH);
    let pipeline = pipeline_cache.queue_compute_pipeline(ComputePipelineDescriptor {
        label: Some("bevox_march".into()),
        layout: vec![layout.clone()],
        shader,
        entry_point: Some("march".into()),
        ..default()
    });

    commands.insert_resource(MarchPipeline { layout, pipeline });
}

/// Runs inside the render graph. Skips the frame if anything is not ready,
/// rather than panicking: a pipeline still compiling is normal, not an error.
pub fn dispatch_march(
    pipeline: Option<Res<MarchPipeline>>,
    buffers: Option<Res<MarchBuffers>>,
    target: Option<Res<MarchTarget>>,
    images: Res<RenderAssets<GpuImage>>,
    pipeline_cache: Res<PipelineCache>,
    device: Res<RenderDevice>,
    queue: Res<RenderQueue>,
) {
    let (Some(pipeline), Some(buffers), Some(target)) = (pipeline, buffers, target) else {
        return;
    };
    let Some(gpu_image) = images.get(&target.image) else {
        return;
    };
    let Some(compute) = pipeline_cache.get_compute_pipeline(pipeline.pipeline) else {
        return;
    };

    let bind_group = device.create_bind_group(
        "bevox_march_bind_group",
        &pipeline.layout,
        &BindGroupEntries::sequential((
            buffers.uniform.as_entire_binding(),
            buffers.nodes.as_entire_binding(),
            buffers.voxels.as_entire_binding(),
            &gpu_image.texture_view,
        )),
    );

    let mut encoder = device.create_command_encoder(&CommandEncoderDescriptor {
        label: Some("bevox_march_encoder"),
    });
    {
        let mut pass = encoder.begin_compute_pass(&ComputePassDescriptor {
            label: Some("bevox_march_pass"),
            timestamp_writes: None,
        });
        pass.set_pipeline(compute);
        pass.set_bind_group(0, &bind_group, &[]);
        pass.dispatch_workgroups(
            target.width.div_ceil(WORKGROUP),
            target.height.div_ceil(WORKGROUP),
            1,
        );
    }
    queue.submit([encoder.finish()]);
}
```

- [ ] **Step 8: Add the extracted scene data**

Append to `crates/bevox_render/src/upload.rs`:

```rust
use bevox_core::gpu::{GpuNode, GpuVolume};

/// The scene as the render world sees it.
///
/// ponytail: cloned into the render world every frame. Fine at 64^3 (a few
/// hundred KB); switch to uploading only when `generation` changes once editing
/// lands and scenes get large.
#[derive(Resource, Clone, ExtractResource)]
pub struct GpuSceneData {
    pub nodes: Vec<GpuNode>,
    pub voxels: Vec<u32>,
    pub depth: u32,
    pub extent: u32,
    pub generation: u32,
}

/// Rebuilds the GPU-side representation whenever the scene changes.
pub fn build_gpu_scene(mut commands: Commands, scene: Res<VoxelScene>) {
    if !scene.is_changed() {
        return;
    }
    let volume = GpuVolume::from_contree(&scene.tree);
    commands.insert_resource(GpuSceneData {
        nodes: volume.buffer_nodes(),
        voxels: volume.voxels,
        depth: scene.tree.depth(),
        extent: scene.tree.extent(),
        generation: scene.generation,
    });
}
```

- [ ] **Step 9: Create the target image and the sprite that shows it**

Append to `crates/bevox_render/src/upload.rs`:

```rust
use bevy::asset::RenderAssetUsages;
use bevy::render::render_resource::{Extent3d, TextureDimension, TextureFormat, TextureUsages};

/// Creates the storage texture the shader writes, and a sprite to display it.
pub fn create_march_target(
    mut commands: Commands,
    mut images: ResMut<Assets<Image>>,
    windows: Query<&Window>,
) {
    let (width, height) = match windows.single() {
        Ok(window) => (window.physical_width().max(1), window.physical_height().max(1)),
        Err(_) => (1280, 720),
    };

    let mut image = Image::new_fill(
        Extent3d { width, height, depth_or_array_layers: 1 },
        TextureDimension::D2,
        &[0, 0, 0, 255],
        TextureFormat::Rgba8Unorm,
        RenderAssetUsages::RENDER_WORLD,
    );
    // STORAGE_BINDING is the one that matters: without it the bind group is
    // rejected at creation, and the symptom is a black window with a validation
    // error rather than an obvious failure.
    image.texture_descriptor.usage =
        TextureUsages::TEXTURE_BINDING | TextureUsages::COPY_DST | TextureUsages::STORAGE_BINDING;

    let handle = images.add(image);
    commands.spawn(Sprite::from_image(handle.clone()));
    commands.insert_resource(MarchTarget { image: handle, width, height });
}
```

- [ ] **Step 10: Write the render-world prepare system**

Append to `crates/bevox_render/src/pipeline.rs`:

```rust
use crate::upload::{GpuSceneData, march_uniform};
use bevy::render::render_resource::util::BufferInitDescriptor;
use bevy::render::render_resource::BufferUsages;

/// Uploads scene buffers and the per-frame uniform into the render world.
pub fn prepare_march_buffers(
    mut commands: Commands,
    device: Res<RenderDevice>,
    queue: Res<RenderQueue>,
    scene: Option<Res<GpuSceneData>>,
    camera: Option<Res<ExtractedMarchCamera>>,
    existing: Option<Res<MarchBuffers>>,
) {
    let (Some(scene), Some(camera)) = (scene, camera) else {
        return;
    };

    let uniform_value = MarchUniform {
        world_from_clip: camera.world_from_clip.to_cols_array_2d(),
        camera_position: camera.position.extend(0.0).to_array(),
        volume_params: [scene.depth, scene.extent, 0, 0],
    };

    // Rebuild storage buffers only when the scene changes; the uniform is
    // rewritten every frame because the camera moves every frame.
    if let Some(buffers) = existing
        && buffers.generation == scene.generation
    {
        queue.write_buffer(&buffers.uniform, 0, bytemuck::bytes_of(&uniform_value));
        return;
    }

    // A zero-length storage buffer is invalid, so an empty scene gets padding.
    let node_bytes = if scene.nodes.is_empty() {
        vec![0u8; 16]
    } else {
        bytemuck::cast_slice(&scene.nodes).to_vec()
    };
    let voxel_bytes = if scene.voxels.is_empty() {
        vec![0u8; 4]
    } else {
        bytemuck::cast_slice(&scene.voxels).to_vec()
    };

    commands.insert_resource(MarchBuffers {
        uniform: device.create_buffer_with_data(&BufferInitDescriptor {
            label: Some("bevox_march_uniform"),
            contents: bytemuck::bytes_of(&uniform_value),
            usage: BufferUsages::UNIFORM | BufferUsages::COPY_DST,
        }),
        nodes: device.create_buffer_with_data(&BufferInitDescriptor {
            label: Some("bevox_nodes"),
            contents: &node_bytes,
            usage: BufferUsages::STORAGE,
        }),
        voxels: device.create_buffer_with_data(&BufferInitDescriptor {
            label: Some("bevox_voxels"),
            contents: &voxel_bytes,
            usage: BufferUsages::STORAGE,
        }),
        generation: scene.generation,
    });
}
```

And in `crates/bevox_render/src/upload.rs`, the camera side:

```rust
/// The active camera, as the render world needs it.
#[derive(Resource, Clone, ExtractResource)]
pub struct ExtractedMarchCamera {
    pub world_from_clip: Mat4,
    pub position: Vec3,
}

/// Reads the active 3D camera in the main world so it can be extracted.
pub fn track_march_camera(
    mut commands: Commands,
    camera: Query<(&GlobalTransform, &Projection), With<Camera3d>>,
) {
    let Ok((transform, projection)) = camera.single() else {
        return;
    };
    let view = transform.compute_matrix().inverse();
    let clip_from_world = projection.get_clip_from_view() * view;
    commands.insert_resource(ExtractedMarchCamera {
        world_from_clip: clip_from_world.inverse(),
        position: transform.translation(),
    });
}
```

- [ ] **Step 11: Register everything in the plugin**

Replace the `Plugin` impl in `crates/bevox_render/src/lib.rs`:

```rust
pub mod camera;
pub mod pipeline;
pub mod upload;

use bevy::render::extract_resource::ExtractResourcePlugin;
use bevy::render::graph::CameraDriverLabel;
use bevy::render::render_graph::RenderGraph;
use bevy::render::{Render, RenderApp, RenderStartup, RenderSystems};

impl Plugin for BevoxRenderPlugin {
    fn build(&self, app: &mut App) {
        app.insert_resource(BevoxReady)
            .add_plugins(ExtractResourcePlugin::<upload::MarchTarget>::default())
            .add_plugins(ExtractResourcePlugin::<upload::GpuSceneData>::default())
            .add_plugins(ExtractResourcePlugin::<upload::ExtractedMarchCamera>::default())
            .add_systems(Startup, upload::create_march_target)
            .add_systems(
                Update,
                (
                    camera::fly_camera_system,
                    upload::build_gpu_scene,
                    upload::track_march_camera,
                ),
            );

        // MinimalPlugins has no RenderApp, which is what keeps the Task 1 smoke
        // test working.
        let Some(render_app) = app.get_sub_app_mut(RenderApp) else {
            return;
        };
        render_app
            .add_systems(RenderStartup, pipeline::init_march_pipeline)
            .add_systems(
                Render,
                pipeline::prepare_march_buffers.in_set(RenderSystems::Prepare),
            )
            .add_systems(RenderGraph, pipeline::dispatch_march.before(CameraDriverLabel));
    }
}
```

> **This is the line Step 1 exists for.** `add_systems(RenderGraph, ...)`
> ordered before the camera driver matches a documented 0.19 example, but the
> label type and import path (`RenderGraph` as a schedule label,
> `CameraDriverLabel` as the ordering target) are the part not verified. If the
> installed crate disagrees, use what it says and correct this plan.

- [ ] **Step 12: Point the app at a 3D camera**

Replace `setup` in `crates/bevox/src/main.rs`:

```rust
fn setup(mut commands: Commands) {
    // 2D camera composites the sprite that shows the marched image.
    commands.spawn(Camera2d);

    // 3D camera exists only to supply view and projection matrices to the
    // shader; it renders nothing itself.
    commands.spawn((
        Camera3d::default(),
        Camera { order: -1, is_active: false, ..default() },
        Transform::from_xyz(-30.0, 40.0, -30.0).looking_at(Vec3::new(32.0, 12.0, 32.0), Vec3::Y),
        bevox_render::camera::FlyCamera::default(),
    ));
}
```

- [ ] **Step 13: Confirm the deliverable**

Run: `cargo run -p bevox --release`
Expected: the window fills with the blue gradient the shader writes — proof that
our compute shader ran, wrote a storage texture, and reached the screen. A black
window means the dispatch or the bind group failed; check the log for validation
errors before proceeding.

- [ ] **Step 14: Commit**

```bash
git add crates/bevox_render crates/bevox
git commit -m "feat(render): dispatch a compute shader into a displayed storage texture" -m "Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>"
```

---

### Task 5: Headless wgpu harness

A test harness that runs a compute shader with no Bevy, no window and no swap
chain, so shader output can be compared against the CPU reference in CI.

**Files:**
- Create: `crates/bevox_render/tests/gpu_parity.rs`
- Modify: `crates/bevox_render/Cargo.toml`

**Interfaces:**
- Produces: `fn gpu_device() -> Option<(wgpu::Device, wgpu::Queue)>`; `fn run_march(device, queue, shader_src, uniform, nodes, voxels, width, height) -> Vec<u8>` returning RGBA bytes.

- [ ] **Step 1: Add the harness dependencies**

```bash
cargo add wgpu --dev -p bevox_render
cargo add pollster --dev -p bevox_render
cargo add bytemuck --features derive --dev -p bevox_render
cargo tree -d -p bevox_render
```

Expected: exactly one `wgpu` version in the tree — the harness must use the same
one Bevy does, or the shader under test is not the shader that ships.

- [ ] **Step 2: Write the failing test**

`crates/bevox_render/tests/gpu_parity.rs`:

```rust
//! Runs the real shader on a headless device and compares it with the CPU
//! reference marcher. Skips cleanly when no adapter is available, so a machine
//! without a usable GPU reports a skip rather than a false failure.

const FLAT_SHADER: &str = r#"
@group(0) @binding(0) var output: texture_storage_2d<rgba8unorm, write>;

@compute @workgroup_size(8, 8, 1)
fn main(@builtin(global_invocation_id) id: vec3<u32>) {
    let size = textureDimensions(output);
    if id.x >= size.x || id.y >= size.y { return; }
    textureStore(output, vec2<i32>(id.xy), vec4<f32>(1.0, 0.0, 0.0, 1.0));
}
"#;

#[test]
fn the_harness_can_run_a_trivial_shader() {
    let Some((device, queue)) = gpu_device() else {
        eprintln!("no GPU adapter available, skipping");
        return;
    };
    let pixels = run_flat(&device, &queue, FLAT_SHADER, 16, 16);
    assert_eq!(pixels.len(), 16 * 16 * 4);
    // Every pixel is opaque red.
    for px in pixels.chunks_exact(4) {
        assert_eq!(px, [255, 0, 0, 255], "unexpected pixel");
    }
}
```

- [ ] **Step 3: Run the test to verify it fails**

Run: `cargo test -p bevox_render --test gpu_parity`
Expected: FAIL — `gpu_device` and `run_flat` not found.

- [ ] **Step 4: Implement the harness**

Add to `crates/bevox_render/tests/gpu_parity.rs`:

```rust
use wgpu::util::DeviceExt;

fn gpu_device() -> Option<(wgpu::Device, wgpu::Queue)> {
    pollster::block_on(async {
        let instance = wgpu::Instance::default();
        let adapter = instance
            .request_adapter(&wgpu::RequestAdapterOptions::default())
            .await
            .ok()?;
        adapter
            .request_device(&wgpu::DeviceDescriptor {
                label: Some("bevox_test_device"),
                ..Default::default()
            })
            .await
            .ok()
    })
}

/// Runs a shader whose only binding is the output storage texture, and reads
/// the result back as RGBA bytes.
fn run_flat(
    device: &wgpu::Device,
    queue: &wgpu::Queue,
    source: &str,
    width: u32,
    height: u32,
) -> Vec<u8> {
    let module = device.create_shader_module(wgpu::ShaderModuleDescriptor {
        label: Some("test_shader"),
        source: wgpu::ShaderSource::Wgsl(source.into()),
    });

    let texture = device.create_texture(&wgpu::TextureDescriptor {
        label: Some("test_target"),
        size: wgpu::Extent3d { width, height, depth_or_array_layers: 1 },
        mip_level_count: 1,
        sample_count: 1,
        dimension: wgpu::TextureDimension::D2,
        format: wgpu::TextureFormat::Rgba8Unorm,
        usage: wgpu::TextureUsages::STORAGE_BINDING | wgpu::TextureUsages::COPY_SRC,
        view_formats: &[],
    });
    let view = texture.create_view(&wgpu::TextureViewDescriptor::default());

    let pipeline = device.create_compute_pipeline(&wgpu::ComputePipelineDescriptor {
        label: Some("test_pipeline"),
        layout: None,
        module: &module,
        entry_point: Some("main"),
        compilation_options: Default::default(),
        cache: None,
    });

    let bind_group = device.create_bind_group(&wgpu::BindGroupDescriptor {
        label: None,
        layout: &pipeline.get_bind_group_layout(0),
        entries: &[wgpu::BindGroupEntry { binding: 0, resource: wgpu::BindingResource::TextureView(&view) }],
    });

    // Readback requires rows padded to 256 bytes.
    let unpadded = width * 4;
    let padded = unpadded.div_ceil(256) * 256;
    let readback = device.create_buffer(&wgpu::BufferDescriptor {
        label: Some("readback"),
        size: (padded * height) as u64,
        usage: wgpu::BufferUsages::COPY_DST | wgpu::BufferUsages::MAP_READ,
        mapped_at_creation: false,
    });

    let mut encoder = device.create_command_encoder(&Default::default());
    {
        let mut pass = encoder.begin_compute_pass(&Default::default());
        pass.set_pipeline(&pipeline);
        pass.set_bind_group(0, &bind_group, &[]);
        pass.dispatch_workgroups(width.div_ceil(8), height.div_ceil(8), 1);
    }
    encoder.copy_texture_to_buffer(
        texture.as_image_copy(),
        wgpu::TexelCopyBufferInfo {
            buffer: &readback,
            layout: wgpu::TexelCopyBufferLayout {
                offset: 0,
                bytes_per_row: Some(padded),
                rows_per_image: Some(height),
            },
        },
        wgpu::Extent3d { width, height, depth_or_array_layers: 1 },
    );
    queue.submit([encoder.finish()]);

    let slice = readback.slice(..);
    slice.map_async(wgpu::MapMode::Read, |_| {});
    device.poll(wgpu::PollType::Wait).expect("device poll failed");

    let data = slice.get_mapped_range();
    let mut out = Vec::with_capacity((unpadded * height) as usize);
    for row in 0..height {
        let start = (row * padded) as usize;
        out.extend_from_slice(&data[start..start + unpadded as usize]);
    }
    drop(data);
    readback.unmap();
    out
}
```

> **Verify before trusting:** wgpu's `request_adapter`, `request_device`, `poll`
> and the `TexelCopy*` type names have all moved across recent versions. If the
> installed version disagrees, run `cargo doc -p wgpu --no-deps --open` and
> follow what is there. The structure — create texture, dispatch, copy to a
> 256-byte-aligned buffer, map, read — is version independent.

- [ ] **Step 5: Run the test to verify it passes**

Run: `cargo test -p bevox_render --test gpu_parity -- --nocapture`
Expected: PASS. On a machine with no adapter it prints the skip line and still passes.

- [ ] **Step 6: Commit**

```bash
git add crates/bevox_render Cargo.lock
git commit -m "test(render): add headless wgpu harness for shader testing" -m "Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>"
```

---

### Task 6: WGSL traversal proven against the CPU reference — milestone 4

**Files:**
- Modify: `crates/bevox_render/assets/shaders/march.wgsl`
- Modify: `crates/bevox_render/tests/gpu_parity.rs`

**Interfaces:**
- Consumes: `GpuVolume`, `march_uniform`, the harness from Task 5, `bevox_core::march::march`.
- Produces: a `march.wgsl` whose `march` entry point ray marches the bound volume and writes each hit voxel's material as a colour; a parity test comparing GPU output with CPU output per pixel.

The comparison is on **material identity**, not shading: the GPU writes the hit
material index into the red channel and a hit flag into the green channel. Two
implementations agreeing on which voxel each ray hit is the invariant that
matters; lighting is Plan 3's concern and would only add float noise here.

- [ ] **Step 1: Write the failing parity test**

Add to `crates/bevox_render/tests/gpu_parity.rs`:

```rust
use bevox_core::contree::Contree;
use bevox_core::dense::DenseVolume;
use bevox_core::gpu::GpuVolume;
use bevox_core::march::{MarchStats, march};
use bevox_core::material::MaterialId;
use glam::{Affine3A, Mat4, UVec3, Vec3};

/// Small scene with a floor, a column and a carved sphere: collapsed uniform
/// regions, subdivided nodes and empty space all in one.
fn parity_scene() -> Contree {
    let mut dense = DenseVolume::new(64).unwrap();
    for z in 0..64 {
        for x in 0..64 {
            for y in 0..6 {
                dense.set(UVec3::new(x, y, z), MaterialId(1));
            }
        }
    }
    for z in 28..36 {
        for y in 6..26 {
            for x in 28..36 {
                dense.set(UVec3::new(x, y, z), MaterialId(2));
            }
        }
    }
    let mut tree = dense.into_contree();
    tree.apply_sphere(Vec3::new(32.0, 18.0, 32.0), 5.0, MaterialId::EMPTY);
    tree
}

#[test]
fn the_gpu_traversal_agrees_with_the_cpu_reference() {
    let Some((device, queue)) = gpu_device() else {
        eprintln!("no GPU adapter available, skipping");
        return;
    };

    let tree = parity_scene();
    let gpu_volume = GpuVolume::from_contree(&tree);

    let (width, height) = (64u32, 64u32);
    let eye = Vec3::new(-30.0, 40.0, -30.0);
    let view = Mat4::look_at_rh(eye, Vec3::new(32.0, 12.0, 32.0), Vec3::Y);
    let projection = Mat4::perspective_rh(0.9, width as f32 / height as f32, 0.1, 500.0);
    let world_from_clip = (projection * view).inverse();

    let shader = std::fs::read_to_string("assets/shaders/march.wgsl")
        .expect("shader file missing");
    let gpu_pixels = run_march(
        &device,
        &queue,
        &shader,
        world_from_clip,
        eye,
        &tree,
        &gpu_volume,
        width,
        height,
    );

    let mut stats = MarchStats::default();
    let mut mismatches = 0usize;
    for y in 0..height {
        for x in 0..width {
            let dir = ray_direction(world_from_clip, eye, x, y, width, height);
            let cpu = march(&tree, Affine3A::IDENTITY, eye, dir, 1000.0, false, &mut stats);

            let i = ((y * width + x) * 4) as usize;
            let gpu_hit = gpu_pixels[i + 1] > 0;
            let gpu_material = gpu_pixels[i];

            match cpu {
                Some(hit) => {
                    if !gpu_hit || gpu_material != hit.material.0 {
                        mismatches += 1;
                    }
                }
                None => {
                    if gpu_hit {
                        mismatches += 1;
                    }
                }
            }
        }
    }

    assert_eq!(stats.overruns, 0, "the CPU reference overran its step budget");
    assert_eq!(mismatches, 0, "{mismatches} pixels disagreed between GPU and CPU");
}

/// Same ray construction the shader performs, so both sides march the same rays.
fn ray_direction(world_from_clip: Mat4, eye: Vec3, x: u32, y: u32, w: u32, h: u32) -> Vec3 {
    let ndc_x = (x as f32 + 0.5) / w as f32 * 2.0 - 1.0;
    let ndc_y = 1.0 - (y as f32 + 0.5) / h as f32 * 2.0;
    let far = world_from_clip * glam::Vec4::new(ndc_x, ndc_y, 1.0, 1.0);
    (far.truncate() / far.w - eye).normalize()
}
```

- [ ] **Step 2: Extend the harness to bind the full layout**

Add `run_march` to `crates/bevox_render/tests/gpu_parity.rs`. It mirrors
`run_flat` but binds four resources in the order Task 4 established: uniform,
node storage buffer, voxel storage buffer, output texture.

```rust
#[repr(C)]
#[derive(Clone, Copy, bytemuck::Pod, bytemuck::Zeroable)]
struct TestUniform {
    world_from_clip: [[f32; 4]; 4],
    camera_position: [f32; 4],
    volume_params: [u32; 4],
}

#[allow(clippy::too_many_arguments)]
fn run_march(
    device: &wgpu::Device,
    queue: &wgpu::Queue,
    source: &str,
    world_from_clip: Mat4,
    eye: Vec3,
    tree: &Contree,
    volume: &GpuVolume,
    width: u32,
    height: u32,
) -> Vec<u8> {
    let uniform = TestUniform {
        world_from_clip: world_from_clip.to_cols_array_2d(),
        camera_position: eye.extend(0.0).to_array(),
        volume_params: [tree.depth(), tree.extent(), 0, 0],
    };

    let uniform_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
        label: Some("uniform"),
        contents: bytemuck::bytes_of(&uniform),
        usage: wgpu::BufferUsages::UNIFORM,
    });
    // A zero-length storage buffer is invalid, so empty volumes get one padding word.
    let node_bytes = if volume.nodes.is_empty() { vec![0u8; 16] } else { volume.node_bytes().to_vec() };
    let voxel_bytes = if volume.voxels.is_empty() { vec![0u8; 4] } else { volume.voxel_bytes().to_vec() };
    let node_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
        label: Some("nodes"),
        contents: &node_bytes,
        usage: wgpu::BufferUsages::STORAGE,
    });
    let voxel_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
        label: Some("voxels"),
        contents: &voxel_bytes,
        usage: wgpu::BufferUsages::STORAGE,
    });

    let module = device.create_shader_module(wgpu::ShaderModuleDescriptor {
        label: Some("march"),
        source: wgpu::ShaderSource::Wgsl(source.into()),
    });
    let texture = device.create_texture(&wgpu::TextureDescriptor {
        label: Some("march_target"),
        size: wgpu::Extent3d { width, height, depth_or_array_layers: 1 },
        mip_level_count: 1,
        sample_count: 1,
        dimension: wgpu::TextureDimension::D2,
        format: wgpu::TextureFormat::Rgba8Unorm,
        usage: wgpu::TextureUsages::STORAGE_BINDING | wgpu::TextureUsages::COPY_SRC,
        view_formats: &[],
    });
    let view = texture.create_view(&wgpu::TextureViewDescriptor::default());

    let pipeline = device.create_compute_pipeline(&wgpu::ComputePipelineDescriptor {
        label: Some("march_pipeline"),
        layout: None,
        module: &module,
        entry_point: Some("march"),
        compilation_options: Default::default(),
        cache: None,
    });
    let bind_group = device.create_bind_group(&wgpu::BindGroupDescriptor {
        label: None,
        layout: &pipeline.get_bind_group_layout(0),
        entries: &[
            wgpu::BindGroupEntry { binding: 0, resource: uniform_buffer.as_entire_binding() },
            wgpu::BindGroupEntry { binding: 1, resource: node_buffer.as_entire_binding() },
            wgpu::BindGroupEntry { binding: 2, resource: voxel_buffer.as_entire_binding() },
            wgpu::BindGroupEntry { binding: 3, resource: wgpu::BindingResource::TextureView(&view) },
        ],
    });

    let unpadded = width * 4;
    let padded = unpadded.div_ceil(256) * 256;
    let readback = device.create_buffer(&wgpu::BufferDescriptor {
        label: Some("readback"),
        size: (padded * height) as u64,
        usage: wgpu::BufferUsages::COPY_DST | wgpu::BufferUsages::MAP_READ,
        mapped_at_creation: false,
    });

    let mut encoder = device.create_command_encoder(&Default::default());
    {
        let mut pass = encoder.begin_compute_pass(&Default::default());
        pass.set_pipeline(&pipeline);
        pass.set_bind_group(0, &bind_group, &[]);
        pass.dispatch_workgroups(width.div_ceil(8), height.div_ceil(8), 1);
    }
    encoder.copy_texture_to_buffer(
        texture.as_image_copy(),
        wgpu::TexelCopyBufferInfo {
            buffer: &readback,
            layout: wgpu::TexelCopyBufferLayout {
                offset: 0,
                bytes_per_row: Some(padded),
                rows_per_image: Some(height),
            },
        },
        wgpu::Extent3d { width, height, depth_or_array_layers: 1 },
    );
    queue.submit([encoder.finish()]);

    let slice = readback.slice(..);
    slice.map_async(wgpu::MapMode::Read, |_| {});
    device.poll(wgpu::PollType::Wait).expect("device poll failed");
    let data = slice.get_mapped_range();
    let mut out = Vec::with_capacity((unpadded * height) as usize);
    for row in 0..height {
        let start = (row * padded) as usize;
        out.extend_from_slice(&data[start..start + unpadded as usize]);
    }
    drop(data);
    readback.unmap();
    out
}
```

- [ ] **Step 3: Run the test to verify it fails**

Run: `cargo test -p bevox_render --test gpu_parity`
Expected: FAIL — the shader still writes a flat colour, so every pixel disagrees.
The failure message reports the mismatch count, which should be most of 4096.

- [ ] **Step 4: Write the traversal shader**

Replace the body of `crates/bevox_render/assets/shaders/march.wgsl`:

```wgsl
// Voxel ray marcher.
//
// A direct port of the CPU reference in bevox_core::march: recursive descent
// expressed as an explicit stack, ray-box per child, children visited near to
// far. Optimisations belong in a later plan; this revision exists to be
// *correct*, and its correctness is pinned by a parity test against the CPU.

struct MarchUniform {
    world_from_clip: mat4x4<f32>,
    camera_position: vec4<f32>,
    volume_params: vec4<u32>,  // [depth, extent, 0, 0]
};

@group(0) @binding(0) var<uniform> view: MarchUniform;
@group(0) @binding(1) var<storage, read> nodes: array<vec4<u32>>;
@group(0) @binding(2) var<storage, read> voxels: array<u32>;
@group(0) @binding(3) var output: texture_storage_2d<rgba8unorm, write>;

const BRICK_EDGE: u32 = 4u;
const CHILDREN: u32 = 64u;
const MAX_STEPS: u32 = 4096u;
const MAX_DEPTH: u32 = 8u;

fn node_mask_lo(n: vec4<u32>) -> u32 { return n.x; }
fn node_mask_hi(n: vec4<u32>) -> u32 { return n.y; }
fn node_child_base(n: vec4<u32>) -> u32 { return n.z; }
fn node_material(n: vec4<u32>) -> u32 { return n.w; }

fn is_subdivided(n: vec4<u32>) -> bool {
    return (node_mask_lo(n) | node_mask_hi(n)) != 0u;
}
fn is_empty(n: vec4<u32>) -> bool {
    return !is_subdivided(n) && node_material(n) == 0u;
}
fn is_uniform_solid(n: vec4<u32>) -> bool {
    return !is_subdivided(n) && node_material(n) != 0u;
}

/// Whether child `i` exists, reading the mask as two 32-bit halves.
fn has_child(n: vec4<u32>, i: u32) -> bool {
    if i < 32u {
        return (node_mask_lo(n) & (1u << i)) != 0u;
    }
    return (node_mask_hi(n) & (1u << (i - 32u))) != 0u;
}

/// Popcount of the mask bits below `i` — the same addressing the CPU uses.
fn child_slot(n: vec4<u32>, i: u32) -> u32 {
    var prefix: u32;
    if i < 32u {
        prefix = countOneBits(node_mask_lo(n) & ((1u << i) - 1u));
    } else {
        let low = countOneBits(node_mask_lo(n));
        var high_mask: u32 = 0u;
        if i > 32u {
            high_mask = (1u << (i - 32u)) - 1u;
        }
        prefix = low + countOneBits(node_mask_hi(n) & high_mask);
    }
    return node_child_base(n) + prefix;
}

fn voxel_byte(index: u32) -> u32 {
    return (voxels[index / 4u] >> ((index % 4u) * 8u)) & 0xFFu;
}

fn level_extent(level: u32) -> u32 {
    return 1u << (2u * (level + 1u));
}

struct Slab { hit: bool, t_enter: f32, t_exit: f32 };

fn ray_box(origin: vec3<f32>, inv_dir: vec3<f32>, lo: vec3<f32>, hi: vec3<f32>) -> Slab {
    let t0 = (lo - origin) * inv_dir;
    let t1 = (hi - origin) * inv_dir;
    let near = min(t0, t1);
    let far = max(t0, t1);
    let t_enter = max(max(near.x, near.y), near.z);
    let t_exit = min(min(far.x, far.y), far.z);
    return Slab(t_enter <= t_exit && t_exit >= 0.0, t_enter, t_exit);
}

struct Frame {
    node: vec4<u32>,
    origin: vec3<u32>,
    level: u32,
    t_enter: f32,
    t_exit: f32,
    // Children of this frame already descended into, as two 32-bit halves.
    visited_lo: u32,
    visited_hi: u32,
};

struct Hit { hit: bool, material: u32, t: f32 };

/// Explicit-stack descent. Children are visited in near-to-far order by
/// selecting the unvisited candidate with the smallest entry distance each
/// iteration, which costs a scan of 64 but needs no sorting storage.
fn traverse(origin: vec3<f32>, dir: vec3<f32>, max_dist: f32) -> Hit {
    // Float division by zero yields infinity in WGSL, which is exactly what the
    // CPU reference stores for a zero direction component. Writing it as a plain
    // divide keeps both sides bit-comparable instead of substituting 1e30.
    let inv_dir = vec3<f32>(1.0) / dir;

    let depth = view.volume_params.x;
    let extent = f32(view.volume_params.y);
    let root_slab = ray_box(origin, inv_dir, vec3<f32>(0.0), vec3<f32>(extent));
    if !root_slab.hit {
        return Hit(false, 0u, 0.0);
    }

    // Element 0 of `nodes` is the root written by the uploader, so every arena
    // slot n lives at index n + 1.
    var stack: array<Frame, MAX_DEPTH>;
    var sp: u32 = 0u;
    var active: bool = true;
    stack[0] = Frame(
        nodes[0],
        vec3<u32>(0u),
        depth - 1u,
        max(root_slab.t_enter, 0.0),
        root_slab.t_exit,
        0u,
        0u,
    );

    var steps: u32 = 0u;

    loop {
        if !active { break; }
        steps = steps + 1u;
        if steps > MAX_STEPS { break; }

        let frame = stack[sp];

        if is_empty(frame.node) {
            if sp == 0u { active = false; } else { sp = sp - 1u; }
            continue;
        }
        if is_uniform_solid(frame.node) {
            return Hit(true, node_material(frame.node), frame.t_enter);
        }

        // Nearest child this ray crosses that this frame has not descended into.
        var best_i: u32 = CHILDREN;
        var best_t: f32 = 1e30;
        var best_exit: f32 = 0.0;
        var best_origin = vec3<u32>(0u);

        var step_size: u32 = 1u;
        if frame.level > 0u {
            step_size = level_extent(frame.level - 1u);
        }

        for (var i: u32 = 0u; i < CHILDREN; i = i + 1u) {
            if !has_child(frame.node, i) { continue; }

            // Branch rather than select: select evaluates both arms, and the
            // high arm would shift by (i - 32) and underflow when i < 32.
            var taken: bool;
            if i < 32u {
                taken = (frame.visited_lo & (1u << i)) != 0u;
            } else {
                taken = (frame.visited_hi & (1u << (i - 32u))) != 0u;
            }
            if taken { continue; }

            let cx = i % BRICK_EDGE;
            let cy = (i / BRICK_EDGE) % BRICK_EDGE;
            let cz = i / (BRICK_EDGE * BRICK_EDGE);
            let child_origin = frame.origin + vec3<u32>(cx, cy, cz) * step_size;
            let lo = vec3<f32>(child_origin);
            let hi = lo + vec3<f32>(f32(step_size));
            let slab = ray_box(origin, inv_dir, lo, hi);
            if !slab.hit { continue; }
            if slab.t_exit < frame.t_enter || slab.t_enter > frame.t_exit { continue; }

            let t = max(slab.t_enter, frame.t_enter);
            if t < best_t {
                best_t = t;
                best_i = i;
                best_exit = slab.t_exit;
                best_origin = child_origin;
            }
        }

        if best_i == CHILDREN || best_t > max_dist {
            if sp == 0u { active = false; } else { sp = sp - 1u; }
            continue;
        }

        // Mark it visited in the stored frame, not the local copy.
        if best_i < 32u {
            stack[sp].visited_lo = stack[sp].visited_lo | (1u << best_i);
        } else {
            stack[sp].visited_hi = stack[sp].visited_hi | (1u << (best_i - 32u));
        }

        let slot = child_slot(frame.node, best_i);

        if frame.level == 0u {
            return Hit(true, voxel_byte(slot), best_t);
        }

        sp = sp + 1u;
        stack[sp] = Frame(
            nodes[slot + 1u],
            best_origin,
            frame.level - 1u,
            best_t,
            min(best_exit, frame.t_exit),
            0u,
            0u,
        );
    }

    return Hit(false, 0u, 0.0);
}

@compute @workgroup_size(8, 8, 1)
fn march(@builtin(global_invocation_id) id: vec3<u32>) {
    let size = textureDimensions(output);
    if id.x >= size.x || id.y >= size.y { return; }

    let ndc = vec2<f32>(
        (f32(id.x) + 0.5) / f32(size.x) * 2.0 - 1.0,
        1.0 - (f32(id.y) + 0.5) / f32(size.y) * 2.0,
    );
    let far = view.world_from_clip * vec4<f32>(ndc, 1.0, 1.0);
    let eye = view.camera_position.xyz;
    let dir = normalize(far.xyz / far.w - eye);

    let hit = traverse(eye, dir, 1000.0);

    // Material identity in red, hit flag in green: what the parity test reads.
    var colour = vec4<f32>(0.0, 0.0, 0.0, 1.0);
    if hit.hit {
        colour = vec4<f32>(f32(hit.material) / 255.0, 1.0, 0.0, 1.0);
    }
    textureStore(output, vec2<i32>(id.xy), colour);
}
```

> **Note on the root node.** The shader reads the root from `nodes[0]` and
> offsets every arena slot by one. The uploader must therefore write the root
> `GpuNode` first and the arena after it. Task 6 Step 5 adds the test that pins
> this; if you would rather pass the root in the uniform, change both sides
> together and keep the parity test green.

- [ ] **Step 5: Upload the root-first buffer**

`GpuVolume::buffer_nodes()` and its test landed in Task 2 — this plan originally
introduced them here, which was a forward reference, since Task 4's uploader
already calls the method.

Only the wiring remains: change `run_march` in the parity test to upload
`volume.buffer_nodes()` rather than `volume.nodes`, and confirm Task 4's
uploader does the same.

- [ ] **Step 6: Run the parity test**

Run: `cargo test -p bevox_render --test gpu_parity -- --nocapture`
Expected: PASS — zero mismatching pixels across the 64x64 image.

If mismatches remain, do not adjust the tolerance. Print the first disagreeing
pixel's ray and march it on both sides: the CPU path is debuggable and the GPU
path is not, which is the entire reason the reference exists.

- [ ] **Step 7: Run the whole suite**

Run: `cargo test`
Expected: PASS — Plan 1's tests, the core GPU-layout tests, the camera tests and both GPU tests.

- [ ] **Step 8: Commit**

```bash
git add crates
git commit -m "feat(render): port voxel traversal to WGSL with CPU parity test" -m "Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>"
```

---

### Task 7: The scene on screen

**Files:**
- Modify: `crates/bevox_render/src/lib.rs`, `crates/bevox_render/src/upload.rs`, `crates/bevox/src/main.rs`

**Interfaces:**
- Consumes: everything above.
- Produces: a running app drawing the voxel scene from a movable camera.

- [ ] **Step 1: Split the shader into a display entry point and a test entry point**

The parity test must keep reading material identity after the display output
becomes cosmetic, so the two concerns get separate entry points over one shared
`traverse`. Replace the single `march` entry point in
`crates/bevox_render/assets/shaders/march.wgsl` with both of these:

```wgsl
/// Ray direction for a pixel. Shared so both entry points march identical rays.
fn primary_ray(id: vec3<u32>, size: vec2<u32>) -> vec3<f32> {
    let ndc = vec2<f32>(
        (f32(id.x) + 0.5) / f32(size.x) * 2.0 - 1.0,
        1.0 - (f32(id.y) + 0.5) / f32(size.y) * 2.0,
    );
    let far = view.world_from_clip * vec4<f32>(ndc, 1.0, 1.0);
    return normalize(far.xyz / far.w - view.camera_position.xyz);
}

/// Material identity in red, hit flag in green. Read by the parity test only.
@compute @workgroup_size(8, 8, 1)
fn march_identity(@builtin(global_invocation_id) id: vec3<u32>) {
    let size = textureDimensions(output);
    if id.x >= size.x || id.y >= size.y { return; }

    let hit = traverse(view.camera_position.xyz, primary_ray(id, size), 1000.0);

    var colour = vec4<f32>(0.0, 0.0, 0.0, 1.0);
    if hit.hit {
        colour = vec4<f32>(f32(hit.material) / 255.0, 1.0, 0.0, 1.0);
    }
    textureStore(output, vec2<i32>(id.xy), colour);
}

/// Palette lookup. A real material table arrives with milestone 5; two colours
/// are enough to prove the scene reads correctly.
fn material_colour(material: u32) -> vec3<f32> {
    if material == 1u { return vec3<f32>(0.55, 0.55, 0.58); }
    if material == 2u { return vec3<f32>(0.70, 0.35, 0.27); }
    return vec3<f32>(1.0, 0.0, 1.0);  // unmapped materials are obvious
}

/// What the window shows.
@compute @workgroup_size(8, 8, 1)
fn march(@builtin(global_invocation_id) id: vec3<u32>) {
    let size = textureDimensions(output);
    if id.x >= size.x || id.y >= size.y { return; }

    let hit = traverse(view.camera_position.xyz, primary_ray(id, size), 1000.0);

    var colour = vec3<f32>(0.35, 0.47, 0.70);  // sky
    if hit.hit {
        // Distance falloff only. This is deliberately not lighting: normals and
        // shadows are milestone 5, and faking them here would hide their absence.
        let fade = clamp(1.0 - hit.t / 160.0, 0.25, 1.0);
        colour = material_colour(hit.material) * fade;
    }
    textureStore(output, vec2<i32>(id.xy), vec4<f32>(colour, 1.0));
}
```

- [ ] **Step 2: Point the parity test at the identity entry point**

In `crates/bevox_render/tests/gpu_parity.rs`, change `run_march`'s pipeline
descriptor:

```rust
        entry_point: Some("march_identity"),
```

- [ ] **Step 3: Build the demo scene in the app**

Replace `crates/bevox/src/main.rs`:

```rust
use bevy::prelude::*;
use bevox_core::dense::DenseVolume;
use bevox_core::material::MaterialId;
use bevox_render::camera::FlyCamera;
use bevox_render::upload::VoxelScene;
use bevox_render::BevoxRenderPlugin;
use glam::UVec3;

fn main() {
    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "BEVOX".into(),
                resolution: (1280u32, 720u32).into(),
                ..default()
            }),
            ..default()
        }))
        .add_plugins(BevoxRenderPlugin)
        .add_systems(Startup, setup)
        .run();
}

fn setup(mut commands: Commands) {
    commands.spawn(Camera2d);
    commands.spawn((
        Camera3d::default(),
        Camera { order: -1, is_active: false, ..default() },
        Transform::from_xyz(-30.0, 40.0, -30.0).looking_at(Vec3::new(32.0, 12.0, 32.0), Vec3::Y),
        FlyCamera::default(),
    ));
    commands.insert_resource(VoxelScene { tree: demo_scene(), generation: 0 });
}

/// The same floor, column and carved sphere the parity test uses, so what is on
/// screen is what the test proved correct.
fn demo_scene() -> bevox_core::contree::Contree {
    let mut dense = DenseVolume::new(64).unwrap();
    for z in 0..64 {
        for x in 0..64 {
            for y in 0..6 {
                dense.set(UVec3::new(x, y, z), MaterialId(1));
            }
        }
    }
    for z in 28..36 {
        for y in 6..26 {
            for x in 28..36 {
                dense.set(UVec3::new(x, y, z), MaterialId(2));
            }
        }
    }
    let mut tree = dense.into_contree();
    tree.apply_sphere(Vec3::new(32.0, 18.0, 32.0), 5.0, MaterialId::EMPTY);
    tree
}
```

- [ ] **Step 4: Confirm the deliverable**

Run: `cargo run -p bevox --release`
Expected: the voxel scene renders, and right-drag plus WASD flies the camera
around it. The carved sphere is visible in the column.

- [ ] **Step 5: Run the whole suite**

Run: `cargo test`
Expected: PASS, including the parity test against `march_identity`.

- [ ] **Step 6: Commit**

```bash
git add crates
git commit -m "feat(render): draw the voxel scene from a movable camera" -m "Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>"
```

---

## Milestone check

- **Milestone 3** — a Bevy shell with a window, a fly camera, and a compute pass of ours writing a texture that reaches the screen (Tasks 1, 3, 4).
- **Milestone 4** — a WGSL traversal that agrees with the CPU reference on every pixel of a scene containing collapsed uniform regions, subdivided nodes and empty space (Tasks 5, 6, 7).

## What this plan deliberately does not do

No DDA, no bitmask filtering, no beam prepass. The shader here is the slow,
obvious port whose only job is to be provably correct. Optimising it is
milestone 8's work, and the parity test built here is what will keep those
optimisations honest: each one must leave the output bit-identical.

No lighting beyond a fixed direction, no shadow ray, no normals — that is
milestone 5, in Plan 3.

## Subsequent plans

- **Plan 3 — milestone 5 and milestone 6.** Implicit normals and the sun shadow ray in WGSL, then `.vox` loading through `dot_vox`.
- **Plan 4 — milestones 7 and 8.** The brush wired to input with dirty-range upload, then DDA, bitmask filtering and the beam prepass, each measured A/B/A and each required to be bit-identical.
