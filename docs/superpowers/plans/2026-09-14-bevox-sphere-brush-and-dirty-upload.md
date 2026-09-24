---
type: Implementation Plan
title: 'Sphere Brush and Dirty-Range Upload'
description: 'Edit the voxel scene at runtime with a sphere brush, uploading only the arena ranges the edit touched rather than the whole volume.'
tags: [editing, upload, brush]
generated: { by: claude-opus-5/claude-code, at: 2026-09-14T00:00:00Z }
---

# Sphere Brush and Dirty-Range Upload Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Edit the voxel scene at runtime with a sphere brush, uploading only the arena ranges the edit touched rather than the whole volume.

**Architecture:** `Contree::apply_sphere` and the arena's dirty-range tracking already exist from milestone 1 and are tested. What is missing is everything between them and the GPU: a main-world system that drains the dirty ranges into a small staging resource each frame, persistent GPU buffers with spare capacity that accept partial `write_buffer` calls, a picking function that turns a screen point into a world position via the CPU marcher, and mouse input in the app. The full-rebuild path stays as the fallback for scene loads and for edits that outgrow the buffers.

**Tech Stack:** Rust, Bevy 0.19.1, wgpu 29.0.4, glam 0.32.

**Spec:** `docs/superpowers/specs/2026-09-13-bevox-raymarcher-core-design.md` — milestone 7, "Sphere brush and dirty-range upload / Editing without full re-upload".

## Global Constraints

- Native desktop only. No web build, ever — this is out of scope permanently, not deferred.
- `bevox_core` has no Bevy and no GPU dependency. It is developed test-first and must stay that way; nothing in this plan adds a dependency to it.
- Voxel data on the GPU is budgeted at **512 MB maximum**. The budget is checked at upload and exceeding it is an error, never an allocation attempt.
- Root volume is up to 4096 per axis (raised from the spec's 1024 during milestone 6; `bevox_core::vox::MAX_EXTENT = 4096`).
- No new dependencies in any crate.
- Tests use a seeded `bevox_core::testing::XorShift64`, never a random seed and never an added test framework.
- Every performance claim comes from interleaved A/B/A runs within a single session. A number measured today is never compared against one recorded yesterday.
- ~~`cargo test --workspace` is currently broken in this repo: under `resolver = "3"` it unifies `bevox_render`'s `wgpu`/`glam` dev-dependencies differently from a per-crate build and then fails to load the bevy artifact it asks for.~~ **That diagnosis was wrong and is superseded.** The real cause was `link.exe` running out of memory (LNK1102) while linking several large Bevy test binaries at once, each carrying full debug info; an OOM-killed link left a truncated artifact, and the *next* build reported "can't find crate for bevy", which is what sent the diagnosis down the wrong path. Fixed by cutting debug info in `Cargo.toml`'s dev profile. `cargo test --workspace` works. The per-crate `Run:` steps below still work and were how this plan was executed.

## Facts the implementer needs that are not obvious from the code

Read these before starting. Each one is a bug waiting to happen.

**The root node is not in the arena.** `GpuVolume::buffer_nodes()` writes the root at buffer index 0 and the arena after it, so **arena slot `n` lives at buffer index `n + 1`**. A dirty arena range `a..b` therefore writes to buffer byte offset `(a + 1) * 16`. Separately, `Contree::set_root` replaces the root on nearly every edit, and the root is in no dirty range because it is in no arena — **index 0 must be rewritten on every edit regardless of what the dirty ranges say.**

**Voxels are packed four bytes to a word.** `bevox_core::gpu::pack_voxels` packs the arena's `&[u8]` into `Vec<u32>`, little-endian within the word. A dirty *byte* range `a..b` overlaps words `a / 4 .. b.div_ceil(4)`, and the bytes sharing those words that were not dirty must still be written correctly — which they are, because the staging system re-reads them from the live arena rather than remembering old values. Round the byte range outward to word boundaries; never write a partial word.

**`free_nodes` does not mark anything dirty, and that is correct.** Freed slots are unreachable from the root, so whatever stale bytes sit in them on the GPU are never read. Only `alloc_nodes`, `set_node`, `alloc_voxels` and `set_voxel` mark dirty.

**An edit can allocate past the end of the buffers.** `alloc_nodes` grows the arena. If the new high-water mark exceeds the buffer's capacity, a partial write is impossible and the whole volume must be rebuilt into larger buffers. That fallback is required for correctness, not an optimisation.

**`write_buffer` offsets and sizes must be 4-byte aligned.** Node entries are 16 bytes so node ranges align naturally. Work with voxels in whole words and they align too.

## File Structure

- `crates/bevox_render/src/upload.rs` — gains `SceneUpdate` (the staged per-frame delta) and `stage_scene_update` (drains the arena's dirty ranges). This file already owns "moving voxel data into the render world", so the staging belongs here rather than in a new module.
- `crates/bevox_render/src/pipeline.rs` — `MarchBuffers` gains capacities; `prepare_march_buffers` learns to write ranges instead of rebuilding, with the rebuild kept as the fallback.
- `crates/bevox_render/src/pick.rs` — **new**. One pure function turning a camera and a screen point into a world-space hit. Its own file because it is the only piece here that is pure geometry and testable without a GPU or a window, and mixing it into `upload.rs` would bury that.
- `crates/bevox/src/main.rs` — mouse buttons and brush radius keys.
- `crates/bevox_render/tests/gpu_parity.rs` — the equivalence test that is this milestone's real gate.
- `crates/bevox_render/tests/gpu_bench.rs` — edit and upload cost.

---

### Task 1: Stage an edit's dirty ranges

The arena already records what an edit touched. This turns that record into a small, cloneable delta the render world can consume, and proves the ranges are exactly right before any GPU code depends on them.

**Files:**
- Modify: `crates/bevox_render/src/upload.rs`
- Test: `crates/bevox_render/src/upload.rs` (the existing `#[cfg(test)] mod tests` at the bottom)

**Interfaces:**
- Produces: `pub struct NodeWrite { pub start: u32, pub nodes: Vec<GpuNode> }`, `pub struct VoxelWrite { pub start_word: u32, pub words: Vec<u32> }`, `pub struct SceneUpdate { pub root: GpuNode, pub nodes: Vec<NodeWrite>, pub voxels: Vec<VoxelWrite>, pub node_high_water: u32, pub voxel_word_high_water: u32, pub generation: u32 }`, and `pub fn stage_scene_update(scene: &mut VoxelScene) -> SceneUpdate`.
- Consumes: `bevox_core::arena::NodeArena::{dirty_nodes, dirty_voxels, clear_dirty, nodes, voxels}`, `bevox_core::gpu::{GpuNode, pack_voxels}`, `bevox_core::contree::Contree::{arena, arena_mut, root}`.

- [ ] **Step 1: Write the failing tests**

Add to the `mod tests` block at the bottom of `crates/bevox_render/src/upload.rs`:

```rust
    use bevox_core::contree::Contree;
    use bevox_core::material::MaterialId;
    use glam::Vec3;

    /// A scene with something in it, so an edit has existing nodes to rewrite
    /// rather than only allocating fresh ones.
    fn edit_scene() -> VoxelScene {
        let mut tree = Contree::empty(3);
        tree.apply_sphere(Vec3::new(32.0, 32.0, 32.0), 12.0, MaterialId(1));
        // The initial build is not an edit: clear it so a test sees only what
        // the edit under test touched.
        tree.arena_mut().clear_dirty();
        VoxelScene { tree, materials: MaterialTable::new(), generation: 1 }
    }

    #[test]
    fn staging_an_untouched_scene_produces_no_writes() {
        let mut scene = edit_scene();
        let update = stage_scene_update(&mut scene);
        assert!(update.nodes.is_empty(), "nothing was edited, yet nodes were staged");
        assert!(update.voxels.is_empty(), "nothing was edited, yet voxels were staged");
    }

    #[test]
    fn the_root_is_staged_even_when_it_is_in_no_dirty_range() {
        // The root lives at buffer index 0, outside the arena, so no dirty
        // range can ever name it -- and nearly every edit replaces it.
        let mut scene = edit_scene();
        let before = GpuNode::from(scene.tree.root());
        scene.tree.apply_sphere(Vec3::new(32.0, 32.0, 32.0), 20.0, MaterialId(2));
        let update = stage_scene_update(&mut scene);
        assert_eq!(update.root, GpuNode::from(scene.tree.root()));
        assert_ne!(update.root, before, "this edit should have changed the root");
    }

    #[test]
    fn staged_nodes_carry_the_bytes_the_arena_holds_now() {
        let mut scene = edit_scene();
        scene.tree.apply_sphere(Vec3::new(20.0, 20.0, 20.0), 6.0, MaterialId(3));
        let update = stage_scene_update(&mut scene);
        assert!(!update.nodes.is_empty(), "an edit staged no node writes");
        let arena = scene.tree.arena();
        for write in &update.nodes {
            for (i, staged) in write.nodes.iter().enumerate() {
                let slot = write.start as usize + i;
                assert_eq!(
                    *staged,
                    GpuNode::from(arena.nodes()[slot]),
                    "staged node at arena slot {slot} does not match the arena"
                );
            }
        }
    }

    #[test]
    fn staged_voxel_words_match_a_full_pack_of_the_arena() {
        let mut scene = edit_scene();
        scene.tree.apply_sphere(Vec3::new(30.0, 30.0, 30.0), 4.0, MaterialId(4));
        let update = stage_scene_update(&mut scene);
        let whole = bevox_core::gpu::pack_voxels(scene.tree.arena().voxels());
        assert!(!update.voxels.is_empty(), "an edit staged no voxel writes");
        for write in &update.voxels {
            for (i, word) in write.words.iter().enumerate() {
                let w = write.start_word as usize + i;
                assert_eq!(*word, whole[w], "staged voxel word {w} differs from a full pack");
            }
        }
    }

    #[test]
    fn staging_clears_the_dirty_record_so_the_next_frame_stages_nothing() {
        let mut scene = edit_scene();
        scene.tree.apply_sphere(Vec3::new(32.0, 32.0, 32.0), 8.0, MaterialId(1));
        let first = stage_scene_update(&mut scene);
        assert!(!first.nodes.is_empty());
        let second = stage_scene_update(&mut scene);
        assert!(second.nodes.is_empty(), "the same edit staged twice");
        assert!(second.voxels.is_empty(), "the same edit staged twice");
    }

    #[test]
    fn the_high_water_marks_cover_the_whole_arena() {
        // The buffers must be large enough for every slot, not merely for the
        // dirty ones: a slot allocated by an earlier edit is still read.
        let mut scene = edit_scene();
        scene.tree.apply_sphere(Vec3::new(40.0, 40.0, 40.0), 10.0, MaterialId(2));
        let update = stage_scene_update(&mut scene);
        assert_eq!(update.node_high_water, scene.tree.arena().nodes().len() as u32);
        assert_eq!(
            update.voxel_word_high_water,
            scene.tree.arena().voxels().len().div_ceil(4) as u32
        );
    }
```

- [ ] **Step 2: Run the tests to verify they fail**

Run: `cargo test -p bevox_render --lib staging`
Expected: FAIL — `cannot find function stage_scene_update in this scope`.

- [ ] **Step 3: Write the implementation**

Add to `crates/bevox_render/src/upload.rs`, above the `#[cfg(test)]` block:

```rust
/// A contiguous run of nodes to write, addressed by arena slot.
#[derive(Clone, Debug)]
pub struct NodeWrite {
    /// First arena slot. The buffer index is this plus one: index 0 is the root.
    pub start: u32,
    pub nodes: Vec<GpuNode>,
}

/// A contiguous run of packed voxel words to write.
#[derive(Clone, Debug)]
pub struct VoxelWrite {
    pub start_word: u32,
    pub words: Vec<u32>,
}

/// One frame's worth of scene changes.
///
/// Cloned into the render world every frame like the rest of the extracted
/// state, which is only affordable because it is empty on frames with no edit.
#[derive(Resource, Clone, Debug, ExtractResource)]
pub struct SceneUpdate {
    /// Always present. The root lives outside the arena, so no dirty range can
    /// name it, and nearly every edit replaces it.
    pub root: GpuNode,
    pub nodes: Vec<NodeWrite>,
    pub voxels: Vec<VoxelWrite>,
    /// Arena slots in use. The node buffer must hold this many plus the root.
    pub node_high_water: u32,
    /// Packed voxel words in use.
    pub voxel_word_high_water: u32,
    /// Bumped by a full scene replacement, never by an edit.
    pub generation: u32,
}

impl Default for SceneUpdate {
    fn default() -> Self {
        Self {
            root: GpuNode::default(),
            nodes: Vec::new(),
            voxels: Vec::new(),
            node_high_water: 0,
            voxel_word_high_water: 0,
            generation: 0,
        }
    }
}

/// Drains the arena's dirty ranges into a delta the render world can write.
///
/// Reads the current arena rather than remembering old values, which is what
/// makes the voxel path correct: a dirty byte range is rounded outward to whole
/// words, and the untouched bytes sharing those words are re-read as they are.
pub fn stage_scene_update(scene: &mut VoxelScene) -> SceneUpdate {
    let node_ranges = scene.tree.arena().dirty_nodes();
    let voxel_ranges = scene.tree.arena().dirty_voxels();

    let arena = scene.tree.arena();
    let nodes = node_ranges
        .iter()
        .map(|r| NodeWrite {
            start: r.start,
            nodes: arena.nodes()[r.start as usize..r.end as usize]
                .iter()
                .map(|n| GpuNode::from(*n))
                .collect(),
        })
        .collect();

    let bytes = arena.voxels();
    let voxels = voxel_ranges
        .iter()
        .map(|r| {
            let first = r.start / 4;
            let last = r.end.div_ceil(4);
            let lo = (first * 4) as usize;
            // The final word may run past the arena; pack_voxels zero-pads it,
            // and those bytes belong to no voxel.
            let hi = ((last * 4) as usize).min(bytes.len());
            VoxelWrite {
                start_word: first,
                words: bevox_core::gpu::pack_voxels(&bytes[lo..hi]),
            }
        })
        .collect();

    let update = SceneUpdate {
        root: GpuNode::from(scene.tree.root()),
        nodes,
        voxels,
        node_high_water: arena.nodes().len() as u32,
        voxel_word_high_water: arena.voxels().len().div_ceil(4) as u32,
        generation: scene.generation,
    };

    scene.tree.arena_mut().clear_dirty();
    update
}
```

Add `use bevox_core::gpu::{GpuNode, GpuVolume};` to the imports if `GpuNode` is not already there, and `use bevy::render::extract_resource::ExtractResource;` is already present.

- [ ] **Step 4: Run the tests to verify they pass**

Run: `cargo test -p bevox_render --lib`
Expected: PASS, 23 tests (17 existing plus the 6 new).

- [ ] **Step 5: Commit**

```bash
git add crates/bevox_render/src/upload.rs
git commit -m "feat(render): stage an edit's dirty ranges for upload" -m "Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>"
```

---

### Task 2: Write only the staged ranges

**Files:**
- Modify: `crates/bevox_render/src/pipeline.rs`
- Modify: `crates/bevox_render/src/lib.rs` (register the resource and system)
- Test: `crates/bevox_render/src/pipeline.rs` (new `#[cfg(test)] mod tests`)

**Interfaces:**
- Consumes: `SceneUpdate`, `NodeWrite`, `VoxelWrite` from Task 1.
- Produces: `MarchBuffers` gains `pub node_capacity: u32` and `pub voxel_word_capacity: u32`; `pub fn buffer_capacity_for(high_water: u32) -> u32`; `pub const VOXEL_BUDGET_BYTES: u64 = 512 * 1024 * 1024`.

> **The node buffer needs `COPY_DST`.** It is currently created with `BufferUsages::STORAGE` alone, and `write_buffer` on a buffer without `COPY_DST` is a validation error. The same applies to the voxel buffer.

- [ ] **Step 1: Write the failing tests**

Add a new test module at the bottom of `crates/bevox_render/src/pipeline.rs`:

```rust
#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn capacity_leaves_room_to_grow() {
        // Growth headroom exists so that a small edit does not force a full
        // rebuild; the factor is arbitrary but the property is not.
        assert!(buffer_capacity_for(1000) > 1000);
        assert!(buffer_capacity_for(1000) <= 4000, "headroom should be bounded, not unbounded");
    }

    #[test]
    fn capacity_is_never_zero() {
        // A zero-length storage buffer is invalid, so an empty scene still gets
        // room for something.
        assert!(buffer_capacity_for(0) >= 1);
    }

    #[test]
    fn capacity_is_monotonic() {
        let mut last = 0;
        for hw in [0u32, 1, 10, 1_000, 100_000, 1_000_000] {
            let c = buffer_capacity_for(hw);
            assert!(c >= hw, "capacity {c} cannot hold {hw} entries");
            assert!(c >= last, "capacity went backwards as the scene grew");
            last = c;
        }
    }

    #[test]
    fn the_voxel_budget_is_the_number_the_spec_states() {
        assert_eq!(VOXEL_BUDGET_BYTES, 512 * 1024 * 1024);
    }

    #[test]
    fn a_scene_over_the_budget_is_rejected_rather_than_allocated() {
        // Checked at upload and exceeding it is an error, never an allocation
        // attempt -- so the check must be on the capacity actually requested.
        let over = (VOXEL_BUDGET_BYTES / 4) as u32 + 1;
        assert!(!within_voxel_budget(buffer_capacity_for(over)));
        assert!(within_voxel_budget(buffer_capacity_for(1000)));
    }
}
```

- [ ] **Step 2: Run the tests to verify they fail**

Run: `cargo test -p bevox_render --lib capacity`
Expected: FAIL — `cannot find function buffer_capacity_for in this scope`.

- [ ] **Step 3: Write the implementation**

In `crates/bevox_render/src/pipeline.rs`, add near the other constants:

```rust
/// Voxel data budget on the GPU. Checked at upload; exceeding it is an error,
/// never an allocation attempt.
pub const VOXEL_BUDGET_BYTES: u64 = 512 * 1024 * 1024;

/// Entries to allocate for a scene currently using `high_water` of them.
///
/// The headroom is what lets an edit allocate new nodes without forcing the
/// whole volume to be rebuilt. Doubling is bounded and monotonic; growing by a
/// fixed slack would stop helping once scenes got large.
pub fn buffer_capacity_for(high_water: u32) -> u32 {
    high_water.saturating_mul(2).max(1024)
}

/// Whether a voxel-word capacity fits the budget.
pub fn within_voxel_budget(word_capacity: u32) -> bool {
    u64::from(word_capacity) * 4 <= VOXEL_BUDGET_BYTES
}
```

Add the two capacity fields to `MarchBuffers`:

```rust
    pub node_capacity: u32,
    pub voxel_word_capacity: u32,
```

Add `COPY_DST` to both storage buffers where they are created:

```rust
            usage: BufferUsages::STORAGE | BufferUsages::COPY_DST,
```

Replace the early-return branch of `prepare_march_buffers`. The signature gains `update: Option<Res<SceneUpdate>>`:

```rust
pub fn prepare_march_buffers(
    mut commands: Commands,
    device: Res<RenderDevice>,
    queue: Res<RenderQueue>,
    scene: Option<Res<GpuSceneData>>,
    update: Option<Res<SceneUpdate>>,
    camera: Option<Res<ExtractedMarchCamera>>,
    existing: Option<Res<MarchBuffers>>,
) {
```

and, in place of the existing `if let Some(buffers) = existing && buffers.generation == scene.generation { ... }` block:

```rust
    // Reuse the buffers unless the scene was replaced outright or an edit grew
    // past the room they have. Both fall through to the rebuild below.
    if let Some(buffers) = existing
        && buffers.generation == scene.generation
        && update.as_ref().is_none_or(|u| {
            u.node_high_water < buffers.node_capacity
                && u.voxel_word_high_water <= buffers.voxel_word_capacity
        })
    {
        queue.write_buffer(&buffers.uniform, 0, bytemuck::bytes_of(&uniform_value));

        if let Some(update) = update {
            // Index 0 is the root, which lives outside the arena and so appears
            // in no dirty range. Arena slot n is therefore at index n + 1.
            queue.write_buffer(&buffers.nodes, 0, bytemuck::bytes_of(&update.root));
            for write in &update.nodes {
                let offset = u64::from(write.start + 1) * size_of::<GpuNode>() as u64;
                queue.write_buffer(&buffers.nodes, offset, bytemuck::cast_slice(&write.nodes));
            }
            for write in &update.voxels {
                let offset = u64::from(write.start_word) * 4;
                queue.write_buffer(&buffers.voxels, offset, bytemuck::cast_slice(&write.words));
            }
        }
        return;
    }
```

In the rebuild path below, allocate at capacity rather than exactly, and check the budget. Replace the `node_bytes` / `voxel_bytes` preparation with:

```rust
    let node_capacity = buffer_capacity_for(scene.nodes.len() as u32);
    let voxel_word_capacity = buffer_capacity_for(scene.voxels.len() as u32);
    if !within_voxel_budget(voxel_word_capacity) {
        error!(
            "voxel data needs {} MB, over the {} MB budget; scene not uploaded",
            u64::from(voxel_word_capacity) * 4 / (1024 * 1024),
            VOXEL_BUDGET_BYTES / (1024 * 1024)
        );
        return;
    }

    let mut node_bytes = bytemuck::cast_slice(&scene.nodes).to_vec();
    node_bytes.resize(node_capacity as usize * size_of::<GpuNode>(), 0);
    let mut voxel_bytes = bytemuck::cast_slice(&scene.voxels).to_vec();
    voxel_bytes.resize(voxel_word_capacity as usize * 4, 0);
```

and add the two fields to the `MarchBuffers` the function inserts:

```rust
        node_capacity,
        voxel_word_capacity,
```

In `crates/bevox_render/src/lib.rs`, register the resource and the staging system. Add to the main-world block alongside the other `ExtractResourcePlugin` lines:

```rust
            .init_resource::<upload::SceneUpdate>()
            .add_plugins(ExtractResourcePlugin::<upload::SceneUpdate>::default())
```

and add `upload::stage_scene_update_system` to the same `Update` set the other upload systems run in, ordered after `upload::build_gpu_scene`. Add the wrapper to `upload.rs`:

```rust
/// Drains the scene's dirty ranges once per frame.
///
/// Runs every frame, not only on edits: the resource it writes must be empty on
/// a quiet frame, or the render world would rewrite the last edit forever.
pub fn stage_scene_update_system(
    mut commands: Commands,
    scene: Option<ResMut<VoxelScene>>,
) {
    let Some(mut scene) = scene else {
        return;
    };
    commands.insert_resource(stage_scene_update(&mut scene));
}
```

- [ ] **Step 4: Run the tests to verify they pass**

Run: `cargo test -p bevox_render --lib`
Expected: PASS, 28 tests.

- [ ] **Step 5: Commit**

```bash
git add crates/bevox_render/src/pipeline.rs crates/bevox_render/src/lib.rs crates/bevox_render/src/upload.rs
git commit -m "feat(render): upload only the ranges an edit touched" -m "Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>"
```

---

### Task 3: Prove an incrementally uploaded scene renders identically

This is the milestone's gate. Everything above is plumbing; this is the test that says the plumbing is right. It follows milestone 8's discipline: the optimised path must be **bit-identical** to the straightforward one, and a differing pixel is a defect rather than a trade-off.

**Files:**
- Modify: `crates/bevox_render/tests/common/mod.rs`
- Modify: `crates/bevox_render/tests/gpu_parity.rs`

**Interfaces:**
- Consumes: `SceneUpdate`, `stage_scene_update`, `NodeWrite`, `VoxelWrite` from Task 1; `buffer_capacity_for` from Task 2; `Prepared` from the existing harness.
- Produces: `Prepared::new_with_capacity(...)` and `Prepared::apply_update(&self, queue, &SceneUpdate)`.

- [ ] **Step 1: Extend the harness**

In `crates/bevox_render/tests/common/mod.rs`, the node and voxel buffers are created with `wgpu::BufferUsages::STORAGE`. Add `| wgpu::BufferUsages::COPY_DST` to both, keep handles to them in `Prepared`, and allocate them at capacity so an edit has room:

```rust
pub struct Prepared {
    pipeline: wgpu::ComputePipeline,
    beam_pipeline: Option<wgpu::ComputePipeline>,
    bind_group: wgpu::BindGroup,
    texture: wgpu::Texture,
    /// Kept so a test can apply an incremental update the way the app does.
    node_buffer: wgpu::Buffer,
    voxel_buffer: wgpu::Buffer,
    width: u32,
    height: u32,
}
```

Pad the two byte vectors to capacity before creating the buffers, mirroring the app:

```rust
        let node_capacity = bevox_render::pipeline::buffer_capacity_for(nodes.len() as u32);
        let mut node_bytes = node_bytes;
        node_bytes.resize(node_capacity as usize * 16, 0);

        let voxel_capacity =
            bevox_render::pipeline::buffer_capacity_for(volume.voxels.len() as u32);
        let mut voxel_bytes = voxel_bytes;
        voxel_bytes.resize(voxel_capacity as usize * 4, 0);
```

and add the method that replays an update:

```rust
    /// Writes a staged update exactly the way `prepare_march_buffers` does.
    ///
    /// Duplicating the offset arithmetic here would let the test agree with a
    /// bug, so this mirrors the app's rule explicitly: index 0 is the root,
    /// arena slot n is at index n + 1.
    pub fn apply_update(
        &self,
        queue: &wgpu::Queue,
        update: &bevox_render::upload::SceneUpdate,
    ) {
        queue.write_buffer(&self.node_buffer, 0, bytemuck::bytes_of(&update.root));
        for write in &update.nodes {
            let offset = u64::from(write.start + 1) * 16;
            queue.write_buffer(&self.node_buffer, offset, bytemuck::cast_slice(&write.nodes));
        }
        for write in &update.voxels {
            let offset = u64::from(write.start_word) * 4;
            queue.write_buffer(&self.voxel_buffer, offset, bytemuck::cast_slice(&write.words));
        }
    }
```

Return the two buffers in the `Self { ... }` construction.

- [ ] **Step 2: Write the failing test**

Add to `crates/bevox_render/tests/gpu_parity.rs`:

```rust
/// An edited scene uploaded by ranges must render exactly like the same scene
/// uploaded whole.
///
/// This is the milestone's gate. A partial upload that misses a range produces
/// a scene that is *nearly* right, which is the hardest kind of wrong to see by
/// eye -- so it is checked pixel for pixel rather than looked at. Several edits
/// run in sequence because the second one rewrites nodes the first allocated,
/// which is where a stale offset shows up.
#[test]
fn an_incrementally_uploaded_edit_renders_identically() {
    let Some((device, queue)) = gpu_device() else {
        eprintln!("no GPU adapter available, skipping");
        return;
    };
    let shader = std::fs::read_to_string("assets/shaders/march.wgsl").expect("shader missing");
    let (width, height) = (96u32, 96u32);
    let eye = Vec3::new(-30.0, 40.0, -30.0);
    let view = Mat4::look_at_rh(eye, Vec3::new(32.0, 12.0, 32.0), Vec3::Y);
    let projection = Mat4::perspective_rh(0.9, 1.0, 0.1, 500.0);
    let world_from_clip = (projection * view).inverse();

    let edits = [
        (Vec3::new(32.0, 20.0, 32.0), 7.0, MaterialId(2)),
        (Vec3::new(20.0, 10.0, 40.0), 5.0, MaterialId(3)),
        // Erasing is the case that frees nodes rather than allocating them.
        (Vec3::new(32.0, 20.0, 32.0), 4.0, MaterialId::EMPTY),
        (Vec3::new(45.0, 14.0, 20.0), 9.0, MaterialId(1)),
    ];

    // One tree is edited and uploaded incrementally; the other is edited the
    // same way and uploaded from scratch each time.
    let mut incremental = VoxelScene {
        tree: parity_scene(),
        materials: parity_materials_table(),
        generation: 1,
    };
    incremental.tree.arena_mut().clear_dirty();
    let mut whole = parity_scene();

    let volume = GpuVolume::from_contree(&incremental.tree);
    let prepared = Prepared::new(
        &device, &shader, "march_identity", &incremental.tree, &volume, world_from_clip, eye,
        width, height, march_flags::DEFAULT,
    );

    for (i, (centre, radius, material)) in edits.iter().enumerate() {
        incremental.tree.apply_sphere(*centre, *radius, *material);
        whole.apply_sphere(*centre, *radius, *material);

        let update = bevox_render::upload::stage_scene_update(&mut incremental);
        assert!(
            update.node_high_water < bevox_render::pipeline::buffer_capacity_for(
                volume.nodes.len() as u32
            ),
            "edit {i} outgrew the buffers; the test needs a bigger starting capacity"
        );
        prepared.apply_update(&queue, &update);
        let got = prepared.read_back(&device, &queue);

        let whole_volume = GpuVolume::from_contree(&whole);
        let reference = run_march_flagged(
            &device, &queue, &shader, "march_identity", world_from_clip, eye, &whole,
            &whole_volume, width, height, march_flags::DEFAULT,
        );

        let differing = reference.chunks(4).zip(got.chunks(4)).filter(|(a, b)| a != b).count();
        assert_eq!(
            differing, 0,
            "after edit {i}, {differing} of {} pixels differ between the incremental \
             upload and a full one",
            width * height
        );
    }
}
```

The test needs a `MaterialTable` matching the harness palette. Add next to `parity_materials` in `crates/bevox_render/tests/common/mod.rs`:

```rust
/// The same colours `parity_materials` produces, as a MaterialTable.
///
/// Shared rather than duplicated so a test cannot pass by lighting the scene
/// differently from the thing it is checking.
///
/// `push` assigns ids in order from 1, so the nth pushed material gets
/// MaterialId(n) -- which is what makes this agree with `parity_materials`,
/// whose index 0 is the empty slot.
pub fn parity_materials_table() -> bevox_core::material::MaterialTable {
    let mut table = bevox_core::material::MaterialTable::new();
    for rgba in parity_materials().iter().skip(1) {
        let color = [
            (rgba[0] * 255.0) as u8,
            (rgba[1] * 255.0) as u8,
            (rgba[2] * 255.0) as u8,
            (rgba[3] * 255.0) as u8,
        ];
        table.push(bevox_core::material::Material { color });
    }
    table
}
```

> The table only has to agree with `parity_materials()`. If `push` returns
> `None` the table is full, which cannot happen for the handful of parity
> colours -- ignore the return rather than unwrapping it in a loop.

- [ ] **Step 3: Run the test to verify it fails**

Run: `cargo test -p bevox_render --test gpu_parity an_incrementally`
Expected: FAIL — `no method named apply_update` before Step 1 is done; after Step 1, it compiles and must pass.

- [ ] **Step 4: Run the test to verify it passes**

Run: `cargo test -p bevox_render --test gpu_parity`
Expected: PASS, 13 tests.

> **If it fails on every pixel**, the root is not being written — check that index 0 is rewritten unconditionally. **If it fails on a patch of pixels that grows with each edit**, the `+ 1` slot-to-index shift is missing or doubled. **If it fails only after the erasing edit**, freed ranges are being treated as dirty, or a voxel word is being written from a stale byte range.

- [ ] **Step 5: Commit**

```bash
git add crates/bevox_render/tests
git commit -m "test(render): pin incremental upload against a full one" -m "Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>"
```

---

### Task 4: Turn a screen point into a world position

**Files:**
- Create: `crates/bevox_render/src/pick.rs`
- Modify: `crates/bevox_render/src/lib.rs` (add `pub mod pick;`)

**Interfaces:**
- Produces: `pub struct Pick { pub voxel: UVec3, pub position: Vec3, pub normal: Vec3 }` and `pub fn pick_voxel(tree: &Contree, world_from_clip: Mat4, eye: Vec3, ndc: Vec2) -> Option<Pick>`.
- Consumes: `bevox_core::march::{march, MarchStats}`.

- [ ] **Step 1: Write the failing tests**

Create `crates/bevox_render/src/pick.rs`:

```rust
//! Turning a point on screen into a voxel in the world.
//!
//! The CPU reference marcher does the work. One ray per click is nothing next
//! to a frame of them, and it avoids a GPU readback with its latency and its
//! second copy of the traversal to keep in agreement.

use bevox_core::contree::Contree;
use bevox_core::march::{MarchStats, march};
use glam::{Affine3A, Mat4, UVec3, Vec2, Vec3};

/// Where a ray through the screen met the volume.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Pick {
    pub voxel: UVec3,
    /// The point on the surface, in world space.
    pub position: Vec3,
    /// The face the ray entered through.
    pub normal: Vec3,
}

#[cfg(test)]
mod tests {
    use super::*;
    use bevox_core::dense::DenseVolume;
    use bevox_core::material::MaterialId;

    /// A single solid block in the middle of an otherwise empty volume.
    fn block_scene() -> Contree {
        let mut dense = DenseVolume::new(64).unwrap();
        for z in 28..36 {
            for y in 28..36 {
                for x in 28..36 {
                    dense.set(UVec3::new(x, y, z), MaterialId(1));
                }
            }
        }
        dense.into_contree()
    }

    fn camera(eye: Vec3, target: Vec3) -> Mat4 {
        let view = Mat4::look_at_rh(eye, target, Vec3::Y);
        let projection = Mat4::perspective_rh(0.9, 1.0, 0.1, 500.0);
        (projection * view).inverse()
    }

    #[test]
    fn the_centre_of_the_screen_hits_what_the_camera_looks_at() {
        let tree = block_scene();
        let eye = Vec3::new(32.0, 32.0, -40.0);
        let hit = pick_voxel(&tree, camera(eye, Vec3::splat(32.0)), eye, Vec2::ZERO)
            .expect("a camera pointed at the block should hit it");
        assert!(
            (28..36).contains(&hit.voxel.x)
                && (28..36).contains(&hit.voxel.y)
                && (28..36).contains(&hit.voxel.z),
            "picked {:?}, which is outside the block",
            hit.voxel
        );
    }

    #[test]
    fn the_normal_faces_the_camera() {
        let tree = block_scene();
        let eye = Vec3::new(32.0, 32.0, -40.0);
        let hit = pick_voxel(&tree, camera(eye, Vec3::splat(32.0)), eye, Vec2::ZERO).unwrap();
        // Approaching along +z, the entered face points back along -z.
        assert_eq!(hit.normal, Vec3::new(0.0, 0.0, -1.0));
    }

    #[test]
    fn a_ray_into_empty_space_picks_nothing() {
        let tree = block_scene();
        let eye = Vec3::new(32.0, 32.0, -40.0);
        // Look away from the block entirely.
        let away = camera(eye, eye + Vec3::new(0.0, 1.0, -1.0));
        assert!(pick_voxel(&tree, away, eye, Vec2::ZERO).is_none());
    }

    #[test]
    fn the_position_lies_on_the_picked_voxel() {
        let tree = block_scene();
        let eye = Vec3::new(32.0, 32.0, -40.0);
        let hit = pick_voxel(&tree, camera(eye, Vec3::splat(32.0)), eye, Vec2::ZERO).unwrap();
        let lo = hit.voxel.as_vec3();
        // On the surface, so a coordinate may sit exactly on a face.
        assert!(
            hit.position.cmpge(lo - Vec3::splat(0.001)).all()
                && hit.position.cmple(lo + Vec3::splat(1.001)).all(),
            "position {:?} is not on voxel {:?}",
            hit.position,
            hit.voxel
        );
    }

    #[test]
    fn an_off_centre_point_picks_a_different_voxel_than_the_centre() {
        // Guards against ignoring the ndc argument entirely, which would make
        // every click land wherever the camera happens to point.
        let tree = block_scene();
        let eye = Vec3::new(32.0, 32.0, -40.0);
        let world_from_clip = camera(eye, Vec3::splat(32.0));
        let centre = pick_voxel(&tree, world_from_clip, eye, Vec2::ZERO).unwrap();
        let offset = pick_voxel(&tree, world_from_clip, eye, Vec2::new(0.35, 0.0)).unwrap();
        assert_ne!(centre.voxel, offset.voxel);
    }
}
```

- [ ] **Step 2: Run the tests to verify they fail**

Run: `cargo test -p bevox_render --lib pick`
Expected: FAIL — `cannot find function pick_voxel in this scope`.

- [ ] **Step 3: Write the implementation**

Add above the test module in `crates/bevox_render/src/pick.rs`:

```rust
/// The voxel under a point in normalised device coordinates.
///
/// `ndc` is -1 to 1 on each axis, y up, matching what the shader's `primary_ray`
/// builds from a pixel — so a pick agrees with what was drawn there.
pub fn pick_voxel(
    tree: &Contree,
    world_from_clip: Mat4,
    eye: Vec3,
    ndc: Vec2,
) -> Option<Pick> {
    let far = world_from_clip * glam::Vec4::new(ndc.x, ndc.y, 1.0, 1.0);
    let dir = (far.truncate() / far.w - eye).normalize();

    // Generous: the ray has to cross the whole volume from outside it.
    let max_dist = (tree.extent() as f32 * 8.0).max(1000.0);
    let mut stats = MarchStats::default();
    let hit = march(tree, Affine3A::IDENTITY, eye, dir, max_dist, false, &mut stats)?;

    Some(Pick {
        voxel: hit.voxel,
        position: eye + dir * hit.t,
        normal: hit.face_normal,
    })
}
```

Add `pub mod pick;` to `crates/bevox_render/src/lib.rs`.

- [ ] **Step 4: Run the tests to verify they pass**

Run: `cargo test -p bevox_render --lib`
Expected: PASS, 33 tests.

- [ ] **Step 5: Commit**

```bash
git add crates/bevox_render/src/pick.rs crates/bevox_render/src/lib.rs
git commit -m "feat(render): pick the voxel under a point on screen" -m "Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>"
```

---

### Task 5: Wire the brush to the mouse

**Files:**
- Modify: `crates/bevox/src/main.rs`

**Interfaces:**
- Consumes: `bevox_render::pick::pick_voxel`, `VoxelScene`, `Contree::apply_sphere`.
- Produces: `BrushSettings { radius: f32, material: MaterialId }` and `brush_input` system.

> **The fly camera already captures the mouse for looking.** Clicks must not fight it. Read `crates/bevox/src/main.rs` to see how `fly_camera_system` consumes input, and place the brush system after it in the same `Update` set.

- [ ] **Step 1: Write the failing test**

Brush behaviour that does not need a window goes in `bevox_core`, where the sphere already lives. Add to the `mod tests` block of `crates/bevox_core/src/edit.rs`:

```rust
    #[test]
    fn painting_then_erasing_the_same_sphere_restores_the_tree() {
        // The property the brush rests on: erase is paint with EMPTY, and the
        // two compose back to where they started. If they do not, repeated
        // editing drifts and the arena grows without bound.
        let mut dense = DenseVolume::new(64).unwrap();
        for z in 0..64 {
            for x in 0..64 {
                for y in 0..6 {
                    dense.set(UVec3::new(x, y, z), MaterialId(1));
                }
            }
        }
        let tree = dense.into_contree();
        let before = tree.to_dense();

        let mut edited = tree;
        edited.apply_sphere(Vec3::new(32.0, 3.0, 32.0), 5.0, MaterialId(2));
        assert_ne!(edited.to_dense(), before, "the paint did nothing");

        // Erase exactly what was painted, then repaint the floor it removed.
        edited.apply_sphere(Vec3::new(32.0, 3.0, 32.0), 5.0, MaterialId::EMPTY);
        for z in 0..64 {
            for x in 0..64 {
                for y in 0..6 {
                    let p = UVec3::new(x, y, z);
                    if before.get(p) != MaterialId::EMPTY && edited.get(p) == MaterialId::EMPTY {
                        edited.apply_sphere(p.as_vec3() + Vec3::splat(0.5), 0.4, MaterialId(1));
                    }
                }
            }
        }
        assert_eq!(edited.to_dense(), before, "paint then erase did not round trip");
        edited.check_canonical().expect("the tree stopped being canonical");
    }
```

- [ ] **Step 2: Run the test to verify it fails or passes**

Run: `cargo test -p bevox_core --lib painting_then_erasing`
Expected: PASS if `apply_sphere` is already correct, which it should be — this pins existing behaviour the brush now depends on. If it FAILS, stop: the brush cannot be wired to an edit that does not round trip, and the bug is in `edit.rs`, not in the app.

- [ ] **Step 3: Write the app wiring**

In `crates/bevox/src/main.rs`, add:

```rust
/// What the brush paints and how big it is.
#[derive(Resource)]
struct BrushSettings {
    radius: f32,
    material: MaterialId,
}

impl Default for BrushSettings {
    fn default() -> Self {
        Self { radius: 4.0, material: MaterialId(1) }
    }
}

/// Left click paints, right click erases, the wheel resizes the brush.
///
/// The pick runs against the same tree the renderer draws, so what is clicked
/// is what was seen. Placing the sphere at the hit point rather than at the
/// voxel centre keeps the brush from stepping in whole voxels as the camera
/// turns.
fn brush_input(
    buttons: Res<ButtonInput<MouseButton>>,
    mut wheel: EventReader<bevy::input::mouse::MouseWheel>,
    mut brush: ResMut<BrushSettings>,
    mut scene: ResMut<VoxelScene>,
    camera: Query<(&GlobalTransform, &Projection), With<Camera3d>>,
) {
    for event in wheel.read() {
        brush.radius = (brush.radius + event.y).clamp(1.0, 32.0);
    }

    let paint = buttons.just_pressed(MouseButton::Left);
    let erase = buttons.just_pressed(MouseButton::Right);
    if !paint && !erase {
        return;
    }
    let Ok((transform, projection)) = camera.single() else {
        return;
    };

    let eye = transform.translation();
    let world_from_clip =
        (projection.get_clip_from_view() * transform.to_matrix().inverse()).inverse();
    // The crosshair, not the cursor: the fly camera holds the pointer captive
    // for looking, so the centre of the screen is where the user is aiming.
    let Some(hit) = pick_voxel(&scene.tree, world_from_clip, eye, Vec2::ZERO) else {
        return;
    };

    let material = if erase { MaterialId::EMPTY } else { brush.material };
    // Paint on the near side of the surface so a click adds material in front
    // of what was hit rather than burying it inside.
    let centre = if erase {
        hit.position
    } else {
        hit.position + hit.normal * brush.radius
    };
    scene.tree.apply_sphere(centre, brush.radius, material);
    // Deliberately not bumped: an edit is uploaded by range, and bumping the
    // generation is what asks for a full rebuild.
    let _ = scene.generation;
}
```

Register it:

```rust
        .init_resource::<BrushSettings>()
```

and add `brush_input` to the `Update` systems, after the fly camera system.

- [ ] **Step 4: Build and run the app**

Run: `cargo build -p bevox --release`
Expected: builds with no warnings.

Then run it and edit a scene by hand:

```bash
cargo run -p bevox --release -- assets/sponza.vox
```

Expected: left click adds a ball of voxels in front of the crosshair, right click carves one out, the wheel changes the size, and the frame rate does not collapse after an edit. **Frame rate is not the measurement here** — it is capped at 60 and proves nothing about cost. It is being watched only for a collapse, which would mean edits are forcing full rebuilds.

- [ ] **Step 5: Commit**

```bash
git add crates/bevox/src/main.rs crates/bevox_core/src/edit.rs
git commit -m "feat(bevox): paint and erase voxels with the mouse" -m "Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>"
```

---

### Task 6: Measure what an edit costs

**Files:**
- Modify: `crates/bevox_render/tests/gpu_bench.rs`

**Interfaces:**
- Consumes: `bench_scene` from the existing bench file, and `stage_scene_update` from Task 1. No GPU: this counts bytes staged, and a dispatch would measure the renderer rather than the edit.

- [ ] **Step 1: Write the measurement**

Add to `crates/bevox_render/tests/gpu_bench.rs`:

```rust
/// What an edit actually costs: the CPU rewrite, and the bytes it uploads.
///
/// The claim this milestone makes is "editing without full re-upload", and the
/// number that supports it is the ratio of bytes written to bytes the scene
/// occupies. Wall-clock time for the write is dominated by queue submission at
/// these sizes, so bytes are the honest measure and are reported as such.
#[test]
#[ignore]
fn an_edit_uploads_a_fraction_of_the_scene() {
    let (tree, extent) = bench_scene();
    let mut scene = bevox_render::upload::VoxelScene {
        tree,
        materials: bevox_core::material::MaterialTable::new(),
        generation: 1,
    };
    scene.tree.arena_mut().clear_dirty();

    let whole_nodes = scene.tree.arena().nodes().len() * 16;
    let whole_voxels = scene.tree.arena().voxels().len();
    println!(
        "scene: extent {extent}, {} node bytes, {} voxel bytes",
        whole_nodes, whole_voxels
    );

    for radius in [2.0f32, 8.0, 32.0] {
        let centre = Vec3::splat(extent as f32 * 0.5);
        let started = std::time::Instant::now();
        scene.tree.apply_sphere(centre, radius, bevox_core::material::MaterialId(3));
        let edit_ms = started.elapsed().as_secs_f32() * 1000.0;

        let staged = std::time::Instant::now();
        let update = bevox_render::upload::stage_scene_update(&mut scene);
        let stage_ms = staged.elapsed().as_secs_f32() * 1000.0;

        let node_bytes: usize = update.nodes.iter().map(|w| w.nodes.len() * 16).sum();
        let voxel_bytes: usize = update.voxels.iter().map(|w| w.words.len() * 4).sum();
        let total = node_bytes + voxel_bytes;
        println!(
            "radius {radius:>5}: edit {edit_ms:6.2} ms, stage {stage_ms:5.2} ms, \
             upload {total:>9} bytes ({:.3}% of the scene) in {} ranges",
            total as f64 / (whole_nodes + whole_voxels) as f64 * 100.0,
            update.nodes.len() + update.voxels.len()
        );
        assert!(
            total < whole_nodes + whole_voxels,
            "radius {radius} uploaded the whole scene; the dirty ranges are not narrowing anything"
        );
    }
}
```

- [ ] **Step 2: Run it**

Run: `cargo test --release -p bevox_render --test gpu_bench an_edit_uploads -- --ignored --nocapture`
Expected: PASS, printing three lines. The percentage should be small for radius 2 and grow with radius. **If radius 2 uploads more than a percent or two of the scene**, the dirty ranges are being merged too aggressively or the whole arena is being staged — investigate before recording the number.

- [ ] **Step 3: Record the result**

Add a section to this plan's file under a `## Measurements` heading with the three lines as printed, the GPU, and the date. State plainly what the number does and does not show: it is bytes uploaded, not frame time, and it was measured in one process.

- [ ] **Step 4: Run the whole suite**

Run: `cargo test -p bevox_core && cargo test -p bevox_render && cargo test -p bevox`
Expected: PASS. Note the total count and compare it against the 134 that passed at the end of milestone 8, plus the tests this plan adds.

- [ ] **Step 5: Commit**

```bash
git add crates/bevox_render/tests/gpu_bench.rs docs/superpowers/plans
git commit -m "test(render): measure what an edit uploads" -m "Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>"
```

---

## Measurements

Ran 2026-09-14 on the GTX 1650 (the machine's recorded GPU for this project's
other measurements). This particular test touches no GPU and no device -- it
stages updates on the CPU and counts bytes -- so the byte counts themselves are
hardware-independent; the machine is noted for the record, not because it
could have changed the result.

`cargo test --release -p bevox_render --test gpu_bench an_edit_uploads -- --ignored --nocapture`:

```
scene: extent 1024, 1158784 node bytes, 2134016 voxel bytes
radius     2: edit   0.06 ms, stage  0.02 ms, upload       800 bytes (0.024% of the scene) in 2 ranges
radius     8: edit   0.08 ms, stage  0.07 ms, upload      3328 bytes (0.101% of the scene) in 9 ranges
radius    32: edit   0.98 ms, stage  0.43 ms, upload     60960 bytes (1.851% of the scene) in 10 ranges
test result: ok. 1 passed; 0 failed; 0 ignored; 0 measured; 3 filtered out; finished in 0.18s
```

(Edits are cumulative: each radius is applied to the scene the previous one
left behind, so the byte counts are not three independent measurements of the
same starting state.)

This is a byte count, not a timing: it shows what fraction of the scene's
node and voxel storage a `stage_scene_update` call actually serialises after
an edit, measured once in a single process. It does not show frame time,
GPU upload latency, or anything about the render path. The `edit_ms` and
`stage_ms` figures printed alongside it are single-shot CPU wall-clock
readings from one process, not the interleaved A/B/A comparisons this
project otherwise requires for a performance claim -- treat them as
indicative only, not as a benchmark result.

The radius-2 line is the strongest evidence for the milestone's "editing
without full re-upload" claim: a minimal edit stages 800 bytes against a
3.2 MB scene (0.024%), in only 2 ranges, which is what "not a full
re-upload" has to mean in bytes. The radius-32 line (1.851%) shows the
fraction growing sensibly with edit size while staying well under the
scene total, so the dirty-range mechanism is not merging ranges into
something coarser than the edit.

### Render-world extraction, measured after the fact

`ExtractResourcePlugin::<GpuSceneData>` cloned the whole payload into the render
world every frame. `extract_gpu_scene` clones only when the scene actually
changed. A/B/A interleaved in one process, release build, 2026-09-14:

```
          bench_scene (extent 1024,     3.15 MB): clone  0.860/ 0.869 ms  saved  0.865 ms (  5.2% of a 16.7 ms frame, drift 0.010)
  Church_Of_St_Sophia (extent 4096,    28.55 MB): clone  7.611/ 7.369 ms  saved  7.490 ms ( 44.9%, drift 0.242)
               castle (extent 4096,    28.29 MB): clone  7.663/ 7.650 ms  saved  7.657 ms ( 45.9%, drift 0.013)
               custom (extent  256,     4.28 MB): clone  0.958/ 0.939 ms  saved  0.948 ms (  5.7%, drift 0.019)
                 nuke (extent 4096,    40.32 MB): clone 10.818/11.042 ms  saved 10.930 ms ( 65.6%, drift 0.224)
               sponza (extent 1024,     8.91 MB): clone  2.314/ 2.465 ms  saved  2.390 ms ( 14.3%, drift 0.152)
```

Every row beats its drift by more than an order of magnitude. `nuke` was
spending two thirds of a 60 Hz frame budget copying a scene that had not
changed, and the render world reads only `depth`, `extent` and `generation`
from that copy on a frame with no edit.

This measures the clone in isolation, **not** end-to-end frame time, and the
saving should not be read as a frame-rate claim. The clone sat in the extract
schedule between the main world and the render world, so the time is genuinely
off the frame -- but nothing here measured what the frame does with it. The app
is vsync-capped at 60, and this project has already been burned by a frame
counter reading a healthy 60 while the renderer drew nothing.

## Milestone check

**Milestone 7** — a sphere brush adds and removes voxels at runtime, and only the affected GPU buffer ranges are re-uploaded. The gate is Task 3: an incrementally uploaded edit renders bit-identically to the same scene uploaded whole, across a sequence of edits including one that erases.

## What this plan deliberately does not do

No undo, no brush shapes beyond the sphere, no material picker beyond a default, and no UI. The spec's milestone is the brush and the upload path; everything else is a separate decision.

No GPU-side picking. A CPU ray per click costs nothing next to a frame of them, and a readback would add latency plus a second copy of the traversal to keep in agreement with the first.

No change to the arena's allocator. The spec is explicit that the free list per size class is to be replaced "only if fragmentation is demonstrated by measurement", and nothing here measures it. If Task 6 shows an edit uploading far more than it should, that is a finding to record, not licence to rewrite the allocator.

No fix for `cargo test --workspace`. It is broken for reasons that predate this work and it deserves its own task; the per-crate form is the workaround throughout. *(Done afterwards: the cause was linker memory, not feature resolution. See the dev-profile comment in `Cargo.toml`.)*

## Subsequent work

This is the last milestone in the spec. What follows it is not planned: the deferred items from the original scoping — LODs, empty-space distance fields, GPU timestamp queries, streaming volumes larger than 4096 — remain open, with the exception of a web build, which is permanently out of scope.
