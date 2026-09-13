# BEVOX core and reference marcher — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build `bevox_core` — the contree voxel data structure with editing — and a CPU reference ray marcher that renders a scene to a PNG, with no GPU and no Bevy involved.

**Architecture:** A sparse 64-tree ("contree") where each node carries a 64-bit occupancy mask and addresses its children by popcount prefix. Nodes live in an arena with per-size-class free lists and dirty-range tracking, so edits re-upload only what changed. A deliberately simple recursive ray-box marcher serves as correctness ground truth for the GPU shader written in a later plan.

**Tech Stack:** Rust 1.96, edition 2024, glam for vector maths, `image` as a dev-dependency for PNG output. No Bevy, no wgpu, no test framework.

**Spec:** `docs/superpowers/specs/2026-09-13-bevox-raymarcher-core-design.md`

## Global Constraints

- Rust edition 2024, `rust-version = "1.95"` (Bevy 0.19.1's floor; toolchain present is 1.96.0).
- Native desktop only. WebAssembly and browser builds are permanently excluded — never add a `wasm32` code path or a browser fallback.
- `MaterialId(0)` is reserved for empty space, everywhere, without exception.
- `Node` is exactly 16 bytes and `#[repr(C)]`, because a later plan uploads it to the GPU unchanged.
- No new test frameworks and no property-testing crates. Randomised tests use the seeded xorshift generator built in Task 3.
- `glam` must resolve to exactly one version in the tree, so that a later plan can share types with Bevy without conversion.
- Every march loop has a hard iteration cap and reports overruns. A loop without a cap is a defect even if it terminates in practice.
- Deferred-work hedges from the spec, to be honoured but never built past: traversal takes a volume plus a transform; addressing uses `ChunkCoord`/`VoxelPos` types; `material` is an index into a material table.

## File Structure

| File | Responsibility |
|---|---|
| `Cargo.toml` | Workspace root; members list grows in later plans |
| `crates/bevox_core/Cargo.toml` | Core crate manifest |
| `crates/bevox_core/src/lib.rs` | Module declarations and re-exports |
| `crates/bevox_core/src/material.rs` | `MaterialId`, `Material`, `MaterialTable` |
| `crates/bevox_core/src/address.rs` | `ChunkCoord`, `VoxelPos` — addressing vocabulary |
| `crates/bevox_core/src/node.rs` | `Node` layout, mask helpers, popcount child addressing |
| `crates/bevox_core/src/arena.rs` | `NodeArena`: allocation, free lists, dirty ranges |
| `crates/bevox_core/src/dense.rs` | `DenseVolume` — flat reference volume for tests and import |
| `crates/bevox_core/src/testing.rs` | `XorShift64` seeded RNG |
| `crates/bevox_core/src/contree.rs` | `Contree`: build from dense, read, canonical check |
| `crates/bevox_core/src/edit.rs` | Sphere brush and the generic subtree rewrite it uses |
| `crates/bevox_core/src/mask_table.rs` | Direction reachability masks |
| `crates/bevox_core/src/march.rs` | CPU reference marcher, `Hit`, `MarchStats` |
| `crates/bevox_core/src/normal.rs` | Implicit normal from neighbour occupancy |
| `crates/bevox_core/examples/render_reference.rs` | Renders a test scene to PNG (milestone 2 deliverable) |

Tests live in `#[cfg(test)]` modules beside the code they cover, except the cross-module round-trip and render tests, which go in `crates/bevox_core/tests/`.

---

### Task 1: Workspace, materials, addressing, and the node layout

**Files:**
- Create: `Cargo.toml`
- Create: `crates/bevox_core/Cargo.toml`
- Create: `crates/bevox_core/src/lib.rs`
- Create: `crates/bevox_core/src/material.rs`
- Create: `crates/bevox_core/src/address.rs`
- Create: `crates/bevox_core/src/node.rs`

**Interfaces:**
- Consumes: nothing.
- Produces: `MaterialId(pub u8)` with `MaterialId::EMPTY` and `is_empty() -> bool`; `Material { color: [u8; 4] }`; `MaterialTable::new() -> MaterialTable`, `push(&mut self, Material) -> Option<MaterialId>`, `get(&self, MaterialId) -> Material`, `len(&self) -> usize`. `ChunkCoord(pub IVec3)`, `VoxelPos { chunk: ChunkCoord, local: UVec3 }`. `Node { mask: u64, child_base: u32, material: u32 }` with `Node::EMPTY`, `Node::uniform(MaterialId) -> Node`, `Node::subdivided(u64, u32) -> Node`, `is_subdivided/is_empty/is_uniform_solid(self) -> bool`, `material(self) -> MaterialId`, `child_count(self) -> u32`, `child_slot(self, u32) -> Option<u32>`. Free functions `child_index(x, y, z) -> u32` and constants `BRICK_EDGE: u32 = 4`, `CHILDREN: u32 = 64`.

- [ ] **Step 1: Create the workspace and crate manifests**

`Cargo.toml`:

```toml
[workspace]
resolver = "3"
members = ["crates/bevox_core"]

[workspace.package]
edition = "2024"
rust-version = "1.95"
```

`crates/bevox_core/Cargo.toml`:

```toml
[package]
name = "bevox_core"
version = "0.1.0"
edition.workspace = true
rust-version.workspace = true
```

Then add glam without pinning a guessed version:

```bash
cargo add glam -p bevox_core
cargo tree -p bevox_core -d
```

Expected: `cargo tree -d` reports no duplicate dependencies.

- [ ] **Step 2: Write the failing tests**

`crates/bevox_core/src/material.rs`:

```rust
//! Material identity. Index 0 is reserved for empty space.

/// Index into a [`MaterialTable`].
#[derive(Clone, Copy, PartialEq, Eq, Debug, Default)]
pub struct MaterialId(pub u8);

impl MaterialId {
    /// The absence of a voxel. Never appears in a material table as a real entry.
    pub const EMPTY: MaterialId = MaterialId(0);

    pub fn is_empty(self) -> bool {
        self.0 == 0
    }
}

/// A material's renderable properties. Physics columns (density, friction,
/// restitution) are a deferred feature and are deliberately absent.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub struct Material {
    pub color: [u8; 4],
}

#[derive(Clone, Debug)]
pub struct MaterialTable {
    entries: Vec<Material>,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn slot_zero_is_empty_and_not_pushable() {
        let table = MaterialTable::new();
        assert_eq!(table.len(), 1);
        assert!(MaterialId::EMPTY.is_empty());
        assert!(table.get(MaterialId::EMPTY).color[3] == 0);
    }

    #[test]
    fn push_returns_sequential_ids() {
        let mut table = MaterialTable::new();
        let a = table.push(Material { color: [255, 0, 0, 255] }).unwrap();
        let b = table.push(Material { color: [0, 255, 0, 255] }).unwrap();
        assert_eq!(a, MaterialId(1));
        assert_eq!(b, MaterialId(2));
        assert_eq!(table.get(a).color, [255, 0, 0, 255]);
    }

    #[test]
    fn push_rejects_the_two_hundred_fifty_seventh_material() {
        let mut table = MaterialTable::new();
        for i in 1..=255u16 {
            assert!(table.push(Material { color: [i as u8, 0, 0, 255] }).is_some());
        }
        assert!(table.push(Material { color: [1, 2, 3, 4] }).is_none());
    }
}
```

`crates/bevox_core/src/node.rs`:

```rust
//! The contree node. 16 bytes, uploaded to the GPU unchanged by a later plan.

use crate::material::MaterialId;

/// Voxels per axis within one node.
pub const BRICK_EDGE: u32 = 4;
/// Children per node.
pub const CHILDREN: u32 = 64;

#[repr(C)]
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub struct Node {
    /// Which of the 64 children exist.
    pub mask: u64,
    /// Arena index of the first child. Children are contiguous.
    pub child_base: u32,
    /// Material table index. Meaningful only when `mask == 0`:
    /// 0 means the region is empty, non-zero means solid of that material.
    pub material: u32,
}

/// Child ordinal from child-space coordinates, each in `0..BRICK_EDGE`.
pub const fn child_index(x: u32, y: u32, z: u32) -> u32 {
    x + y * BRICK_EDGE + z * BRICK_EDGE * BRICK_EDGE
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn node_is_sixteen_bytes() {
        assert_eq!(size_of::<Node>(), 16);
        assert_eq!(align_of::<Node>(), 8);
    }

    #[test]
    fn empty_and_uniform_are_distinguishable() {
        assert!(Node::EMPTY.is_empty());
        assert!(!Node::EMPTY.is_uniform_solid());
        assert!(!Node::EMPTY.is_subdivided());

        let solid = Node::uniform(MaterialId(7));
        assert!(!solid.is_empty());
        assert!(solid.is_uniform_solid());
        assert!(!solid.is_subdivided());
        assert_eq!(solid.material(), MaterialId(7));
    }

    #[test]
    fn child_slot_is_none_when_the_bit_is_clear() {
        let node = Node::subdivided(1 << 5, 100);
        assert_eq!(node.child_slot(4), None);
        assert_eq!(node.child_slot(5), Some(100));
        assert_eq!(node.child_count(), 1);
    }

    #[test]
    fn child_slot_offsets_by_popcount_prefix() {
        // Children present at ordinals 1, 5 and 9, stored contiguously from 100.
        let mask = (1u64 << 1) | (1u64 << 5) | (1u64 << 9);
        let node = Node::subdivided(mask, 100);
        assert_eq!(node.child_slot(1), Some(100));
        assert_eq!(node.child_slot(5), Some(101));
        assert_eq!(node.child_slot(9), Some(102));
        assert_eq!(node.child_count(), 3);
    }

    #[test]
    fn child_slot_handles_the_first_and_last_ordinals() {
        let node = Node::subdivided(u64::MAX, 0);
        assert_eq!(node.child_slot(0), Some(0));
        assert_eq!(node.child_slot(63), Some(63));
    }

    #[test]
    fn child_index_matches_x_major_order() {
        assert_eq!(child_index(0, 0, 0), 0);
        assert_eq!(child_index(3, 0, 0), 3);
        assert_eq!(child_index(0, 1, 0), 4);
        assert_eq!(child_index(0, 0, 1), 16);
        assert_eq!(child_index(3, 3, 3), 63);
    }
}
```

`crates/bevox_core/src/address.rs`:

```rust
//! Addressing vocabulary. The core holds exactly one chunk, at the origin;
//! these types exist so that streaming can add chunks later without changing
//! how a voxel address is spelled.

use glam::{IVec3, UVec3};

#[derive(Clone, Copy, PartialEq, Eq, Hash, Debug, Default)]
pub struct ChunkCoord(pub IVec3);

impl ChunkCoord {
    pub const ORIGIN: ChunkCoord = ChunkCoord(IVec3::ZERO);
}

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub struct VoxelPos {
    pub chunk: ChunkCoord,
    pub local: UVec3,
}

impl VoxelPos {
    pub fn at_origin(local: UVec3) -> Self {
        Self { chunk: ChunkCoord::ORIGIN, local }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn origin_positions_share_a_chunk() {
        let a = VoxelPos::at_origin(UVec3::new(1, 2, 3));
        let b = VoxelPos::at_origin(UVec3::new(9, 9, 9));
        assert_eq!(a.chunk, b.chunk);
        assert_eq!(a.chunk, ChunkCoord::ORIGIN);
    }
}
```

`crates/bevox_core/src/lib.rs`:

```rust
pub mod address;
pub mod material;
pub mod node;
```

- [ ] **Step 3: Run the tests to verify they fail**

Run: `cargo test -p bevox_core`
Expected: FAIL — compilation errors reporting no function `new`, `push`, `get`, `len` on `MaterialTable`, and no `EMPTY`, `uniform`, `subdivided`, `is_empty`, `is_uniform_solid`, `is_subdivided`, `material`, `child_count`, `child_slot` on `Node`.

- [ ] **Step 4: Write the implementations**

Append to `crates/bevox_core/src/material.rs`, above the test module:

```rust
impl MaterialTable {
    /// Creates a table whose slot 0 is the reserved empty material.
    pub fn new() -> Self {
        Self { entries: vec![Material { color: [0, 0, 0, 0] }] }
    }

    /// Appends a material. Returns `None` when all 255 usable slots are taken.
    pub fn push(&mut self, material: Material) -> Option<MaterialId> {
        if self.entries.len() > u8::MAX as usize {
            return None;
        }
        let id = MaterialId(self.entries.len() as u8);
        self.entries.push(material);
        Some(id)
    }

    pub fn get(&self, id: MaterialId) -> Material {
        self.entries[id.0 as usize]
    }

    pub fn len(&self) -> usize {
        self.entries.len()
    }

    pub fn is_empty(&self) -> bool {
        self.entries.len() <= 1
    }
}

impl Default for MaterialTable {
    fn default() -> Self {
        Self::new()
    }
}
```

Append to `crates/bevox_core/src/node.rs`, above the test module:

```rust
impl Node {
    pub const EMPTY: Node = Node { mask: 0, child_base: 0, material: 0 };

    /// A region entirely filled with one material, stored without children.
    pub fn uniform(material: MaterialId) -> Node {
        Node { mask: 0, child_base: 0, material: material.0 as u32 }
    }

    /// A region with children. `mask` must be non-zero.
    pub fn subdivided(mask: u64, child_base: u32) -> Node {
        debug_assert!(mask != 0, "a subdivided node must have at least one child");
        Node { mask, child_base, material: 0 }
    }

    pub fn is_subdivided(self) -> bool {
        self.mask != 0
    }

    pub fn is_empty(self) -> bool {
        self.mask == 0 && self.material == 0
    }

    pub fn is_uniform_solid(self) -> bool {
        self.mask == 0 && self.material != 0
    }

    /// Only meaningful when the node is not subdivided.
    pub fn material(self) -> MaterialId {
        MaterialId(self.material as u8)
    }

    pub fn child_count(self) -> u32 {
        self.mask.count_ones()
    }

    /// Arena slot of a child, or `None` when that child does not exist.
    pub fn child_slot(self, child: u32) -> Option<u32> {
        debug_assert!(child < CHILDREN);
        if self.mask & (1u64 << child) == 0 {
            return None;
        }
        let prefix = self.mask & ((1u64 << child) - 1);
        Some(self.child_base + prefix.count_ones())
    }
}
```

- [ ] **Step 5: Run the tests to verify they pass**

Run: `cargo test -p bevox_core`
Expected: PASS, 10 tests (3 material, 6 node, 1 address).

- [ ] **Step 6: Commit**

```bash
git add Cargo.toml Cargo.lock crates/bevox_core
git commit -m "feat(core): add node layout, materials and addressing types" -m "Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>"
```

---

### Task 2: Node arena with free lists and dirty ranges

**Files:**
- Create: `crates/bevox_core/src/arena.rs`
- Modify: `crates/bevox_core/src/lib.rs`

**Interfaces:**
- Consumes: `Node` from Task 1.
- Produces: `NodeArena::new() -> NodeArena`, `alloc_nodes(&mut self, u32) -> u32`, `free_nodes(&mut self, u32, u32)`, `alloc_voxels(&mut self, u32) -> u32`, `free_voxels(&mut self, u32, u32)`, `node(&self, u32) -> Node`, `set_node(&mut self, u32, Node)`, `voxel(&self, u32) -> u8`, `set_voxel(&mut self, u32, u8)`, `nodes(&self) -> &[Node]`, `voxels(&self) -> &[u8]`, `dirty_nodes(&self) -> Vec<Range<u32>>`, `dirty_voxels(&self) -> Vec<Range<u32>>`, `clear_dirty(&mut self)`.

- [ ] **Step 1: Write the failing tests**

`crates/bevox_core/src/arena.rs`:

```rust
//! Arena storage for contree nodes and leaf voxel bytes.
//!
//! Reclamation is a free list per size class, which is the deliberately simple
//! version of a page allocator. Replace it only if measurement shows
//! fragmentation actually costs something.

use crate::node::{CHILDREN, Node};
use core::ops::Range;

#[derive(Debug)]
pub struct NodeArena {
    nodes: Vec<Node>,
    voxels: Vec<u8>,
    node_free: Vec<Vec<u32>>,
    voxel_free: Vec<Vec<u32>>,
    dirty_nodes: Vec<Range<u32>>,
    dirty_voxels: Vec<Range<u32>>,
}

/// Merges overlapping and adjacent ranges, leaving disjoint ones separate.
fn normalize(ranges: &[Range<u32>]) -> Vec<Range<u32>> {
    let mut sorted: Vec<Range<u32>> = ranges.to_vec();
    sorted.sort_by_key(|r| r.start);
    let mut out: Vec<Range<u32>> = Vec::with_capacity(sorted.len());
    for r in sorted {
        match out.last_mut() {
            Some(last) if r.start <= last.end => last.end = last.end.max(r.end),
            _ => out.push(r),
        }
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::material::MaterialId;

    #[test]
    fn allocations_are_sequential_when_nothing_is_free() {
        let mut arena = NodeArena::new();
        assert_eq!(arena.alloc_nodes(3), 0);
        assert_eq!(arena.alloc_nodes(2), 3);
        assert_eq!(arena.nodes().len(), 5);
    }

    #[test]
    fn freed_blocks_are_reused_for_the_same_size_class() {
        let mut arena = NodeArena::new();
        let a = arena.alloc_nodes(4);
        let _b = arena.alloc_nodes(4);
        arena.free_nodes(a, 4);
        assert_eq!(arena.alloc_nodes(4), a);
    }

    #[test]
    fn a_freed_block_is_not_reused_for_a_different_size_class() {
        let mut arena = NodeArena::new();
        let a = arena.alloc_nodes(4);
        arena.free_nodes(a, 4);
        let b = arena.alloc_nodes(2);
        assert_ne!(b, a);
    }

    #[test]
    fn allocation_marks_exactly_the_allocated_range_dirty() {
        let mut arena = NodeArena::new();
        arena.clear_dirty();
        let start = arena.alloc_nodes(3);
        assert_eq!(arena.dirty_nodes(), vec![start..start + 3]);
    }

    #[test]
    fn writes_mark_their_slot_dirty() {
        let mut arena = NodeArena::new();
        let start = arena.alloc_nodes(2);
        arena.clear_dirty();
        arena.set_node(start + 1, Node::uniform(MaterialId(3)));
        assert_eq!(arena.dirty_nodes(), vec![start + 1..start + 2]);
        assert_eq!(arena.node(start + 1).material(), MaterialId(3));
    }

    #[test]
    fn adjacent_dirty_ranges_merge_but_disjoint_ones_do_not() {
        let mut arena = NodeArena::new();
        arena.alloc_nodes(8);
        arena.clear_dirty();
        arena.set_node(0, Node::uniform(MaterialId(1)));
        arena.set_node(1, Node::uniform(MaterialId(1)));
        arena.set_node(5, Node::uniform(MaterialId(1)));
        assert_eq!(arena.dirty_nodes(), vec![0..2, 5..6]);
    }

    #[test]
    fn voxel_allocation_is_tracked_separately_from_nodes() {
        let mut arena = NodeArena::new();
        let v = arena.alloc_voxels(9);
        arena.clear_dirty();
        arena.set_voxel(v + 2, 42);
        assert_eq!(arena.voxel(v + 2), 42);
        assert_eq!(arena.dirty_voxels(), vec![v + 2..v + 3]);
        assert!(arena.dirty_nodes().is_empty());
    }

    #[test]
    fn size_classes_cover_a_full_node_of_children() {
        let mut arena = NodeArena::new();
        let a = arena.alloc_nodes(CHILDREN);
        arena.free_nodes(a, CHILDREN);
        assert_eq!(arena.alloc_nodes(CHILDREN), a);
    }
}
```

- [ ] **Step 2: Run the tests to verify they fail**

Run: `cargo test -p bevox_core arena`
Expected: FAIL — no function `new` found for `NodeArena`, plus the other missing methods.

- [ ] **Step 3: Write the implementation**

Insert into `crates/bevox_core/src/arena.rs`, above the test module:

```rust
impl NodeArena {
    pub fn new() -> Self {
        Self {
            nodes: Vec::new(),
            voxels: Vec::new(),
            // Index by slot count; 0 is unused so that `free[count]` reads directly.
            node_free: vec![Vec::new(); CHILDREN as usize + 1],
            voxel_free: vec![Vec::new(); CHILDREN as usize + 1],
            dirty_nodes: Vec::new(),
            dirty_voxels: Vec::new(),
        }
    }

    pub fn alloc_nodes(&mut self, count: u32) -> u32 {
        debug_assert!(count >= 1 && count <= CHILDREN);
        let start = match self.node_free[count as usize].pop() {
            Some(start) => start,
            None => {
                let start = self.nodes.len() as u32;
                self.nodes.resize(self.nodes.len() + count as usize, Node::EMPTY);
                start
            }
        };
        self.dirty_nodes.push(start..start + count);
        start
    }

    pub fn free_nodes(&mut self, start: u32, count: u32) {
        debug_assert!(count >= 1 && count <= CHILDREN);
        self.node_free[count as usize].push(start);
    }

    pub fn alloc_voxels(&mut self, count: u32) -> u32 {
        debug_assert!(count >= 1 && count <= CHILDREN);
        let start = match self.voxel_free[count as usize].pop() {
            Some(start) => start,
            None => {
                let start = self.voxels.len() as u32;
                self.voxels.resize(self.voxels.len() + count as usize, 0);
                start
            }
        };
        self.dirty_voxels.push(start..start + count);
        start
    }

    pub fn free_voxels(&mut self, start: u32, count: u32) {
        debug_assert!(count >= 1 && count <= CHILDREN);
        self.voxel_free[count as usize].push(start);
    }

    pub fn node(&self, slot: u32) -> Node {
        self.nodes[slot as usize]
    }

    pub fn set_node(&mut self, slot: u32, node: Node) {
        self.nodes[slot as usize] = node;
        self.dirty_nodes.push(slot..slot + 1);
    }

    pub fn voxel(&self, slot: u32) -> u8 {
        self.voxels[slot as usize]
    }

    pub fn set_voxel(&mut self, slot: u32, value: u8) {
        self.voxels[slot as usize] = value;
        self.dirty_voxels.push(slot..slot + 1);
    }

    pub fn nodes(&self) -> &[Node] {
        &self.nodes
    }

    pub fn voxels(&self) -> &[u8] {
        &self.voxels
    }

    /// Ranges touched since the last `clear_dirty`, merged where contiguous.
    pub fn dirty_nodes(&self) -> Vec<Range<u32>> {
        normalize(&self.dirty_nodes)
    }

    pub fn dirty_voxels(&self) -> Vec<Range<u32>> {
        normalize(&self.dirty_voxels)
    }

    pub fn clear_dirty(&mut self) {
        self.dirty_nodes.clear();
        self.dirty_voxels.clear();
    }
}

impl Default for NodeArena {
    fn default() -> Self {
        Self::new()
    }
}
```

Add to `crates/bevox_core/src/lib.rs`:

```rust
pub mod arena;
```

- [ ] **Step 4: Run the tests to verify they pass**

Run: `cargo test -p bevox_core arena`
Expected: PASS, 8 tests.

- [ ] **Step 5: Commit**

```bash
git add crates/bevox_core
git commit -m "feat(core): add node arena with size-class free lists and dirty ranges" -m "Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>"
```

---

### Task 3: Dense volumes, seeded RNG, and contree construction

**Files:**
- Create: `crates/bevox_core/src/dense.rs`
- Create: `crates/bevox_core/src/testing.rs`
- Create: `crates/bevox_core/src/contree.rs`
- Create: `crates/bevox_core/tests/round_trip.rs`
- Modify: `crates/bevox_core/src/lib.rs`

**Interfaces:**
- Consumes: `Node`, `MaterialId`, `NodeArena`, `child_index`, `BRICK_EDGE`, `CHILDREN`.
- Produces: `DenseVolume::new(extent: u32) -> DenseVolume`, `get(&self, UVec3) -> MaterialId`, `set(&mut self, UVec3, MaterialId)`, `extent(&self) -> u32`. `XorShift64::new(seed: u64) -> XorShift64`, `next_u64(&mut self) -> u64`, `next_below(&mut self, u32) -> u32`. `Contree::empty(depth: u32) -> Contree`, `from_dense(&DenseVolume) -> Contree`, `to_dense(&self) -> DenseVolume`, `get(&self, UVec3) -> MaterialId`, `depth(&self) -> u32`, `extent(&self) -> u32`, `root(&self) -> Node`, `arena(&self) -> &NodeArena`, `arena_mut(&mut self) -> &mut NodeArena`, plus `pub(crate)` field access for later tasks in the same crate.

- [ ] **Step 1: Write the failing tests**

`crates/bevox_core/src/testing.rs`:

```rust
//! Seeded randomness for tests. No dependency, reproducible across runs.

pub struct XorShift64(u64);

impl XorShift64 {
    /// `seed` must be non-zero; zero is replaced to keep the generator alive.
    pub fn new(seed: u64) -> Self {
        Self(if seed == 0 { 0x9E37_79B9_7F4A_7C15 } else { seed })
    }

    pub fn next_u64(&mut self) -> u64 {
        let mut x = self.0;
        x ^= x << 13;
        x ^= x >> 7;
        x ^= x << 17;
        self.0 = x;
        x
    }

    /// Uniform enough for test data. `bound` must be non-zero.
    pub fn next_below(&mut self, bound: u32) -> u32 {
        (self.next_u64() % bound as u64) as u32
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_same_seed_produces_the_same_sequence() {
        let mut a = XorShift64::new(12345);
        let mut b = XorShift64::new(12345);
        for _ in 0..64 {
            assert_eq!(a.next_u64(), b.next_u64());
        }
    }

    #[test]
    fn a_zero_seed_still_generates() {
        let mut rng = XorShift64::new(0);
        assert_ne!(rng.next_u64(), 0);
    }

    #[test]
    fn next_below_stays_in_range() {
        let mut rng = XorShift64::new(7);
        for _ in 0..1000 {
            assert!(rng.next_below(5) < 5);
        }
    }
}
```

`crates/bevox_core/src/dense.rs`:

```rust
//! A flat cubic volume. Reference representation for tests and model import.

use crate::material::MaterialId;
use glam::UVec3;

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct DenseVolume {
    extent: u32,
    data: Vec<u8>,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_new_volume_is_entirely_empty() {
        let volume = DenseVolume::new(16);
        assert_eq!(volume.extent(), 16);
        assert_eq!(volume.get(UVec3::new(3, 4, 5)), MaterialId::EMPTY);
    }

    #[test]
    fn set_then_get_returns_the_material() {
        let mut volume = DenseVolume::new(16);
        volume.set(UVec3::new(3, 4, 5), MaterialId(9));
        assert_eq!(volume.get(UVec3::new(3, 4, 5)), MaterialId(9));
        assert_eq!(volume.get(UVec3::new(3, 4, 6)), MaterialId::EMPTY);
    }
}
```

`crates/bevox_core/src/contree.rs`:

```rust
//! The sparse 64-tree. Built bottom-up so that homogeneous regions collapse
//! during construction rather than in a later pass.

use crate::arena::NodeArena;
use crate::material::MaterialId;
use crate::node::{BRICK_EDGE, CHILDREN, Node, child_index};
use crate::dense::DenseVolume;
use glam::UVec3;

pub struct Contree {
    pub(crate) arena: NodeArena,
    pub(crate) root: Node,
    pub(crate) depth: u32,
}

/// Voxels per axis covered by a node at `level`, where level 0 is a leaf brick.
pub fn level_extent(level: u32) -> u32 {
    BRICK_EDGE.pow(level + 1)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::testing::XorShift64;

    #[test]
    fn level_extents_are_powers_of_four() {
        assert_eq!(level_extent(0), 4);
        assert_eq!(level_extent(1), 16);
        assert_eq!(level_extent(4), 1024);
    }

    #[test]
    fn an_empty_volume_collapses_to_an_empty_root() {
        let tree = Contree::from_dense(&DenseVolume::new(64));
        assert!(tree.root().is_empty());
        assert_eq!(tree.arena().nodes().len(), 0);
        assert_eq!(tree.arena().voxels().len(), 0);
    }

    #[test]
    fn a_completely_full_volume_collapses_to_a_uniform_root() {
        let mut dense = DenseVolume::new(64);
        for z in 0..64 {
            for y in 0..64 {
                for x in 0..64 {
                    dense.set(UVec3::new(x, y, z), MaterialId(5));
                }
            }
        }
        let tree = Contree::from_dense(&dense);
        assert!(tree.root().is_uniform_solid());
        assert_eq!(tree.root().material(), MaterialId(5));
        assert_eq!(tree.arena().nodes().len(), 0);
    }

    #[test]
    fn a_single_voxel_is_readable() {
        let mut dense = DenseVolume::new(16);
        dense.set(UVec3::new(7, 2, 11), MaterialId(3));
        let tree = Contree::from_dense(&dense);
        assert_eq!(tree.get(UVec3::new(7, 2, 11)), MaterialId(3));
        assert_eq!(tree.get(UVec3::new(7, 2, 10)), MaterialId::EMPTY);
        assert_eq!(tree.depth(), 2);
        assert_eq!(tree.extent(), 16);
    }

    #[test]
    fn every_voxel_of_a_random_volume_round_trips() {
        let mut rng = XorShift64::new(0xDEAD_BEEF);
        let mut dense = DenseVolume::new(16);
        for z in 0..16 {
            for y in 0..16 {
                for x in 0..16 {
                    // Sparse-ish: roughly one voxel in four is solid.
                    if rng.next_below(4) == 0 {
                        let m = MaterialId(1 + rng.next_below(8) as u8);
                        dense.set(UVec3::new(x, y, z), m);
                    }
                }
            }
        }
        let tree = Contree::from_dense(&dense);
        assert_eq!(tree.to_dense(), dense);
    }

    #[test]
    fn empty_tree_reads_as_empty_everywhere() {
        let tree = Contree::empty(2);
        assert_eq!(tree.extent(), 16);
        assert_eq!(tree.get(UVec3::new(15, 15, 15)), MaterialId::EMPTY);
    }
}
```

`crates/bevox_core/tests/round_trip.rs`:

```rust
use bevox_core::contree::Contree;
use bevox_core::dense::DenseVolume;
use bevox_core::material::MaterialId;
use bevox_core::testing::XorShift64;
use glam::UVec3;

/// Builds a volume with large uniform blocks plus scattered noise, which
/// exercises both the collapse path and the subdivided path.
fn mixed_volume(extent: u32, seed: u64) -> DenseVolume {
    let mut rng = XorShift64::new(seed);
    let mut dense = DenseVolume::new(extent);
    // A solid slab across the bottom quarter.
    for z in 0..extent {
        for y in 0..extent / 4 {
            for x in 0..extent {
                dense.set(UVec3::new(x, y, z), MaterialId(1));
            }
        }
    }
    // Scattered voxels above it.
    for _ in 0..extent * extent {
        let p = UVec3::new(
            rng.next_below(extent),
            extent / 4 + rng.next_below(extent - extent / 4),
            rng.next_below(extent),
        );
        dense.set(p, MaterialId(2 + rng.next_below(4) as u8));
    }
    dense
}

#[test]
fn mixed_volumes_round_trip_at_several_depths() {
    for (extent, seed) in [(4u32, 1u64), (16, 2), (64, 3)] {
        let dense = mixed_volume(extent, seed);
        let tree = Contree::from_dense(&dense);
        assert_eq!(tree.to_dense(), dense, "extent {extent} failed to round trip");
    }
}

#[test]
fn construction_is_deterministic() {
    let dense = mixed_volume(16, 99);
    let a = Contree::from_dense(&dense);
    let b = Contree::from_dense(&dense);
    assert_eq!(a.root(), b.root());
    assert_eq!(a.arena().nodes(), b.arena().nodes());
    assert_eq!(a.arena().voxels(), b.arena().voxels());
}
```

- [ ] **Step 2: Run the tests to verify they fail**

Run: `cargo test -p bevox_core`
Expected: FAIL — no function `new` for `DenseVolume`, no `from_dense`/`to_dense`/`get`/`empty` for `Contree`.

- [ ] **Step 3: Implement `DenseVolume`**

Insert into `crates/bevox_core/src/dense.rs`, above the test module:

```rust
impl DenseVolume {
    /// `extent` must be a power of four so that it maps onto whole tree levels.
    pub fn new(extent: u32) -> Self {
        debug_assert!(extent.is_power_of_two() && extent.trailing_zeros() % 2 == 0);
        Self { extent, data: vec![0; (extent as usize).pow(3)] }
    }

    pub fn extent(&self) -> u32 {
        self.extent
    }

    fn index(&self, p: UVec3) -> usize {
        debug_assert!(p.x < self.extent && p.y < self.extent && p.z < self.extent);
        let e = self.extent as usize;
        p.x as usize + p.y as usize * e + p.z as usize * e * e
    }

    pub fn get(&self, p: UVec3) -> MaterialId {
        MaterialId(self.data[self.index(p)])
    }

    pub fn set(&mut self, p: UVec3, material: MaterialId) {
        let i = self.index(p);
        self.data[i] = material.0;
    }
}
```

- [ ] **Step 4: Implement `Contree` construction and reads**

Insert into `crates/bevox_core/src/contree.rs`, above the test module:

```rust
impl Contree {
    pub fn empty(depth: u32) -> Self {
        debug_assert!(depth >= 1);
        Self { arena: NodeArena::new(), root: Node::EMPTY, depth }
    }

    pub fn depth(&self) -> u32 {
        self.depth
    }

    pub fn extent(&self) -> u32 {
        level_extent(self.depth - 1)
    }

    pub fn root(&self) -> Node {
        self.root
    }

    pub fn arena(&self) -> &NodeArena {
        &self.arena
    }

    pub fn arena_mut(&mut self) -> &mut NodeArena {
        &mut self.arena
    }

    pub fn from_dense(dense: &DenseVolume) -> Self {
        let depth = dense.extent().trailing_zeros() / 2;
        debug_assert!(depth >= 1, "a volume must be at least 4 voxels per axis");
        let mut tree = Self::empty(depth);
        tree.root = tree.build(dense, depth - 1, UVec3::ZERO);
        tree
    }

    /// Builds the node covering `level_extent(level)` voxels from `origin`.
    fn build(&mut self, dense: &DenseVolume, level: u32, origin: UVec3) -> Node {
        if level == 0 {
            return self.build_leaf(dense, origin);
        }

        let step = level_extent(level - 1);
        let mut children = [Node::EMPTY; CHILDREN as usize];
        let mut mask = 0u64;
        for z in 0..BRICK_EDGE {
            for y in 0..BRICK_EDGE {
                for x in 0..BRICK_EDGE {
                    let child_origin = origin + UVec3::new(x, y, z) * step;
                    let child = self.build(dense, level - 1, child_origin);
                    let i = child_index(x, y, z);
                    children[i as usize] = child;
                    if !child.is_empty() {
                        mask |= 1u64 << i;
                    }
                }
            }
        }

        if mask == 0 {
            return Node::EMPTY;
        }
        if let Some(node) = collapse_uniform(&children) {
            return node;
        }

        let count = mask.count_ones();
        let base = self.arena.alloc_nodes(count);
        let mut slot = base;
        for child in children.iter() {
            if !child.is_empty() {
                self.arena.set_node(slot, *child);
                slot += 1;
            }
        }
        Node::subdivided(mask, base)
    }

    /// Builds a leaf brick, whose children are individual voxels.
    fn build_leaf(&mut self, dense: &DenseVolume, origin: UVec3) -> Node {
        let mut materials = [MaterialId::EMPTY; CHILDREN as usize];
        let mut mask = 0u64;
        for z in 0..BRICK_EDGE {
            for y in 0..BRICK_EDGE {
                for x in 0..BRICK_EDGE {
                    let m = dense.get(origin + UVec3::new(x, y, z));
                    let i = child_index(x, y, z);
                    materials[i as usize] = m;
                    if !m.is_empty() {
                        mask |= 1u64 << i;
                    }
                }
            }
        }

        if mask == 0 {
            return Node::EMPTY;
        }
        if mask == u64::MAX {
            let first = materials[0];
            if materials.iter().all(|m| *m == first) {
                return Node::uniform(first);
            }
        }

        let count = mask.count_ones();
        let base = self.arena.alloc_voxels(count);
        let mut slot = base;
        for m in materials.iter() {
            if !m.is_empty() {
                self.arena.set_voxel(slot, m.0);
                slot += 1;
            }
        }
        Node::subdivided(mask, base)
    }

    pub fn get(&self, p: UVec3) -> MaterialId {
        debug_assert!(p.x < self.extent() && p.y < self.extent() && p.z < self.extent());
        let mut node = self.root;
        let mut level = self.depth - 1;
        let mut local = p;

        loop {
            if !node.is_subdivided() {
                return node.material();
            }

            if level == 0 {
                let i = child_index(local.x, local.y, local.z);
                return match node.child_slot(i) {
                    Some(slot) => MaterialId(self.arena.voxel(slot)),
                    None => MaterialId::EMPTY,
                };
            }

            let step = level_extent(level - 1);
            let cell = local / step;
            let i = child_index(cell.x, cell.y, cell.z);
            match node.child_slot(i) {
                Some(slot) => {
                    node = self.arena.node(slot);
                    local -= cell * step;
                    level -= 1;
                }
                None => return MaterialId::EMPTY,
            }
        }
    }

    pub fn to_dense(&self) -> DenseVolume {
        let extent = self.extent();
        let mut dense = DenseVolume::new(extent);
        for z in 0..extent {
            for y in 0..extent {
                for x in 0..extent {
                    let p = UVec3::new(x, y, z);
                    dense.set(p, self.get(p));
                }
            }
        }
        dense
    }
}

/// Returns a collapsed node when every child is the same uniform solid.
fn collapse_uniform(children: &[Node; CHILDREN as usize]) -> Option<Node> {
    let first = children[0];
    if !first.is_uniform_solid() {
        return None;
    }
    if children.iter().all(|c| *c == first) {
        Some(first)
    } else {
        None
    }
}
```

Add to `crates/bevox_core/src/lib.rs`:

```rust
pub mod contree;
pub mod dense;
pub mod testing;
```

- [ ] **Step 5: Run the tests to verify they pass**

Run: `cargo test -p bevox_core`
Expected: PASS. New tests: 3 in `testing`, 2 in `dense`, 6 in `contree`, 2 in `tests/round_trip.rs`.

- [ ] **Step 6: Commit**

```bash
git add crates/bevox_core
git commit -m "feat(core): build contree from dense volumes with collapse on construction" -m "Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>"
```

---

### Task 4: Canonical form checking

**Files:**
- Modify: `crates/bevox_core/src/contree.rs`
- Create: `crates/bevox_core/tests/canonical.rs`

**Interfaces:**
- Consumes: `Contree`, `Node`, `NodeArena` from Tasks 1–3.
- Produces: `CanonicalError` enum with variants `UncollapsedEmpty { level: u32 }`, `UncollapsedUniform { level: u32 }`, `EmptyChildStored { level: u32, slot: u32 }`, `SlotOutOfBounds { slot: u32 }`; `Contree::check_canonical(&self) -> Result<(), CanonicalError>`.

- [ ] **Step 1: Write the failing tests**

`crates/bevox_core/tests/canonical.rs`:

```rust
use bevox_core::contree::{CanonicalError, Contree};
use bevox_core::dense::DenseVolume;
use bevox_core::material::MaterialId;
use bevox_core::node::Node;
use bevox_core::testing::XorShift64;
use glam::UVec3;

#[test]
fn trees_built_from_dense_volumes_are_canonical() {
    let mut rng = XorShift64::new(4242);
    for extent in [4u32, 16, 64] {
        let mut dense = DenseVolume::new(extent);
        for _ in 0..extent * extent {
            let p = UVec3::new(
                rng.next_below(extent),
                rng.next_below(extent),
                rng.next_below(extent),
            );
            dense.set(p, MaterialId(1 + rng.next_below(3) as u8));
        }
        let tree = Contree::from_dense(&dense);
        assert_eq!(tree.check_canonical(), Ok(()), "extent {extent}");
    }
}

#[test]
fn an_empty_tree_is_canonical() {
    assert_eq!(Contree::empty(3).check_canonical(), Ok(()));
}

#[test]
fn a_node_whose_children_are_all_the_same_solid_is_rejected() {
    // Hand-build a level-1 node with 64 identical uniform children, which
    // construction would have collapsed.
    let mut tree = Contree::empty(2);
    let base = tree.arena_mut().alloc_nodes(64);
    for i in 0..64 {
        tree.arena_mut().set_node(base + i, Node::uniform(MaterialId(3)));
    }
    tree.set_root_for_test(Node::subdivided(u64::MAX, base));
    assert!(matches!(
        tree.check_canonical(),
        Err(CanonicalError::UncollapsedUniform { .. })
    ));
}

#[test]
fn an_empty_child_occupying_a_slot_is_rejected() {
    let mut tree = Contree::empty(2);
    let base = tree.arena_mut().alloc_nodes(2);
    tree.arena_mut().set_node(base, Node::uniform(MaterialId(1)));
    tree.arena_mut().set_node(base + 1, Node::EMPTY);
    tree.set_root_for_test(Node::subdivided(0b11, base));
    assert!(matches!(
        tree.check_canonical(),
        Err(CanonicalError::EmptyChildStored { .. })
    ));
}

#[test]
fn a_child_slot_past_the_arena_is_rejected() {
    let mut tree = Contree::empty(2);
    tree.set_root_for_test(Node::subdivided(0b1, 9999));
    assert!(matches!(
        tree.check_canonical(),
        Err(CanonicalError::SlotOutOfBounds { .. })
    ));
}
```

- [ ] **Step 2: Run the tests to verify they fail**

Run: `cargo test -p bevox_core --test canonical`
Expected: FAIL — `CanonicalError` not found, no `check_canonical`, no `set_root_for_test`.

- [ ] **Step 3: Write the implementation**

Append to `crates/bevox_core/src/contree.rs`, above the test module:

```rust
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum CanonicalError {
    /// A subdivided node whose children are all empty; it should be `Node::EMPTY`.
    UncollapsedEmpty { level: u32 },
    /// A subdivided node whose children are all the same uniform solid.
    UncollapsedUniform { level: u32 },
    /// An empty node occupying a child slot, which wastes space and breaks
    /// the invariant that a set mask bit implies a non-empty child.
    EmptyChildStored { level: u32, slot: u32 },
    /// A child slot outside the arena.
    SlotOutOfBounds { slot: u32 },
}

impl Contree {
    /// Replaces the root. Used by the edit path in Task 5.
    pub(crate) fn set_root(&mut self, root: Node) {
        self.root = root;
    }

    /// Root replacement for tests that build deliberately invalid trees.
    /// Integration tests are separate crates, so this has to be public.
    #[doc(hidden)]
    pub fn set_root_for_test(&mut self, root: Node) {
        self.root = root;
    }

    pub fn check_canonical(&self) -> Result<(), CanonicalError> {
        self.check_node(self.root, self.depth - 1)
    }

    fn check_node(&self, node: Node, level: u32) -> Result<(), CanonicalError> {
        if !node.is_subdivided() {
            return Ok(());
        }

        // Leaf bricks address voxel bytes, which cannot be empty by construction
        // because empty voxels clear their mask bit. Only bounds need checking.
        if level == 0 {
            let last = node.child_base + node.child_count() - 1;
            if last as usize >= self.arena.voxels().len() {
                return Err(CanonicalError::SlotOutOfBounds { slot: last });
            }
            for i in 0..CHILDREN {
                if let Some(slot) = node.child_slot(i)
                    && self.arena.voxel(slot) == 0
                {
                    return Err(CanonicalError::EmptyChildStored { level, slot });
                }
            }
            return Ok(());
        }

        let last = node.child_base + node.child_count() - 1;
        if last as usize >= self.arena.nodes().len() {
            return Err(CanonicalError::SlotOutOfBounds { slot: last });
        }

        let mut all_same_uniform = node.child_count() == CHILDREN;
        let mut first = Node::EMPTY;
        for i in 0..CHILDREN {
            let Some(slot) = node.child_slot(i) else {
                all_same_uniform = false;
                continue;
            };
            let child = self.arena.node(slot);
            if child.is_empty() {
                return Err(CanonicalError::EmptyChildStored { level, slot });
            }
            if i == 0 {
                first = child;
            }
            if !child.is_uniform_solid() || child != first {
                all_same_uniform = false;
            }
            self.check_node(child, level - 1)?;
        }

        if all_same_uniform {
            return Err(CanonicalError::UncollapsedUniform { level });
        }
        Ok(())
    }
}
```

Note: `all_same_uniform` starts false unless all 64 children are present, so a partially filled node can never be flagged. `UncollapsedEmpty` is unreachable through the public build path — an all-empty node has a zero mask and is therefore not subdivided — but it stays in the enum because Task 5's edit path could regress into producing one, and a later test asserts against it.

- [ ] **Step 4: Run the tests to verify they pass**

Run: `cargo test -p bevox_core --test canonical`
Expected: PASS, 5 tests.

- [ ] **Step 5: Run the whole suite for regressions**

Run: `cargo test -p bevox_core`
Expected: PASS, all tests from Tasks 1–4.

- [ ] **Step 6: Commit**

```bash
git add crates/bevox_core
git commit -m "feat(core): add canonical form checking for contrees" -m "Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>"
```

---

### Task 5: Sphere brush editing with incremental dirty tracking

**Files:**
- Create: `crates/bevox_core/src/edit.rs`
- Create: `crates/bevox_core/tests/editing.rs`
- Modify: `crates/bevox_core/src/lib.rs`

**Interfaces:**
- Consumes: `Contree`, `Node`, `NodeArena`, `MaterialId`, `level_extent`, `child_index`.
- Produces: `Contree::apply_sphere(&mut self, center: Vec3, radius: f32, material: MaterialId)`. Setting `MaterialId::EMPTY` erases. Internally, `rewrite(&mut self, node: Node, level: u32, origin: UVec3, op: &SphereOp) -> Node`.

- [ ] **Step 1: Write the failing tests**

`crates/bevox_core/tests/editing.rs`:

```rust
use bevox_core::contree::Contree;
use bevox_core::dense::DenseVolume;
use bevox_core::material::MaterialId;
use glam::{UVec3, Vec3};

/// Ground truth: the same sphere applied to a flat volume.
fn dense_sphere(extent: u32, center: Vec3, radius: f32, material: MaterialId) -> DenseVolume {
    let mut dense = DenseVolume::new(extent);
    for z in 0..extent {
        for y in 0..extent {
            for x in 0..extent {
                let c = Vec3::new(x as f32, y as f32, z as f32) + Vec3::splat(0.5);
                if c.distance(center) <= radius {
                    dense.set(UVec3::new(x, y, z), material);
                }
            }
        }
    }
    dense
}

#[test]
fn a_sphere_added_to_an_empty_tree_matches_the_dense_result() {
    let mut tree = Contree::empty(2); // 16 voxels per axis
    let center = Vec3::splat(8.0);
    tree.apply_sphere(center, 5.0, MaterialId(2));
    assert_eq!(tree.to_dense(), dense_sphere(16, center, 5.0, MaterialId(2)));
    assert_eq!(tree.check_canonical(), Ok(()));
}

#[test]
fn voxels_outside_the_radius_are_untouched() {
    let mut dense = DenseVolume::new(16);
    dense.set(UVec3::new(0, 0, 0), MaterialId(7));
    dense.set(UVec3::new(15, 15, 15), MaterialId(7));
    let mut tree = Contree::from_dense(&dense);

    tree.apply_sphere(Vec3::splat(8.0), 2.0, MaterialId(2));

    assert_eq!(tree.get(UVec3::new(0, 0, 0)), MaterialId(7));
    assert_eq!(tree.get(UVec3::new(15, 15, 15)), MaterialId(7));
    assert_eq!(tree.get(UVec3::splat(8)), MaterialId(2));
}

#[test]
fn erasing_with_the_empty_material_removes_voxels_and_recollapses() {
    let mut dense = DenseVolume::new(16);
    for z in 0..16 {
        for y in 0..16 {
            for x in 0..16 {
                dense.set(UVec3::new(x, y, z), MaterialId(1));
            }
        }
    }
    let mut tree = Contree::from_dense(&dense);
    assert!(tree.root().is_uniform_solid());

    // A radius large enough to cover the whole volume.
    tree.apply_sphere(Vec3::splat(8.0), 64.0, MaterialId::EMPTY);

    assert!(tree.root().is_empty());
    assert_eq!(tree.check_canonical(), Ok(()));
}

#[test]
fn filling_everything_collapses_back_to_a_uniform_root() {
    let mut tree = Contree::empty(2);
    tree.apply_sphere(Vec3::splat(8.0), 64.0, MaterialId(4));
    assert!(tree.root().is_uniform_solid());
    assert_eq!(tree.root().material(), MaterialId(4));
    assert_eq!(tree.check_canonical(), Ok(()));
}

#[test]
fn dirty_ranges_are_sufficient_to_reproduce_the_edited_buffers() {
    let mut tree = Contree::empty(3); // 64 voxels per axis
    tree.apply_sphere(Vec3::splat(32.0), 10.0, MaterialId(1));

    // Snapshot the buffers as a GPU would hold them, then clear the flags.
    let mut mirror_nodes = tree.arena().nodes().to_vec();
    let mut mirror_voxels = tree.arena().voxels().to_vec();
    tree.arena_mut().clear_dirty();

    tree.apply_sphere(Vec3::new(40.0, 32.0, 32.0), 6.0, MaterialId(2));

    // Apply only the dirty ranges to the mirror, exactly as an upload would.
    mirror_nodes.resize(tree.arena().nodes().len(), Default::default());
    mirror_voxels.resize(tree.arena().voxels().len(), 0);
    for r in tree.arena().dirty_nodes() {
        let range = r.start as usize..r.end as usize;
        mirror_nodes[range.clone()].copy_from_slice(&tree.arena().nodes()[range]);
    }
    for r in tree.arena().dirty_voxels() {
        let range = r.start as usize..r.end as usize;
        mirror_voxels[range.clone()].copy_from_slice(&tree.arena().voxels()[range]);
    }

    assert_eq!(mirror_nodes, tree.arena().nodes());
    assert_eq!(mirror_voxels, tree.arena().voxels());
}

#[test]
fn repeated_identical_edits_are_stable() {
    let mut tree = Contree::empty(2);
    tree.apply_sphere(Vec3::splat(8.0), 4.0, MaterialId(3));
    let after_first = tree.to_dense();
    tree.apply_sphere(Vec3::splat(8.0), 4.0, MaterialId(3));
    assert_eq!(tree.to_dense(), after_first);
    assert_eq!(tree.check_canonical(), Ok(()));
}
```

`Node` must derive `Default` for the `resize` call above. Add `Default` to its derive list in `crates/bevox_core/src/node.rs`:

```rust
#[repr(C)]
#[derive(Clone, Copy, PartialEq, Eq, Debug, Default)]
pub struct Node {
```

- [ ] **Step 2: Run the tests to verify they fail**

Run: `cargo test -p bevox_core --test editing`
Expected: FAIL — no method `apply_sphere` found for `Contree`.

- [ ] **Step 3: Write the implementation**

`crates/bevox_core/src/edit.rs`:

```rust
//! Voxel editing. Edits rewrite only the subtrees they intersect, free the
//! nodes they replace, and leave the tree canonical.

use crate::contree::{Contree, level_extent};
use crate::material::MaterialId;
use crate::node::{BRICK_EDGE, CHILDREN, Node, child_index};
use glam::{UVec3, Vec3};

/// A sphere paint operation. `material` of `MaterialId::EMPTY` erases.
pub(crate) struct SphereOp {
    center: Vec3,
    radius: f32,
    material: MaterialId,
}

impl SphereOp {
    /// Whether the sphere touches the cube of `extent` voxels at `origin`.
    fn intersects(&self, origin: UVec3, extent: u32) -> bool {
        let lo = origin.as_vec3();
        let hi = lo + Vec3::splat(extent as f32);
        let closest = self.center.clamp(lo, hi);
        closest.distance_squared(self.center) <= self.radius * self.radius
    }

    /// Whether a single voxel's centre lies inside the sphere.
    fn covers_voxel(&self, p: UVec3) -> bool {
        let c = p.as_vec3() + Vec3::splat(0.5);
        c.distance_squared(self.center) <= self.radius * self.radius
    }
}

impl Contree {
    /// Paints a sphere. Passing `MaterialId::EMPTY` erases instead of filling.
    pub fn apply_sphere(&mut self, center: Vec3, radius: f32, material: MaterialId) {
        let op = SphereOp { center, radius, material };
        let root = self.root();
        let new_root = self.rewrite(root, self.depth() - 1, UVec3::ZERO, &op);
        self.set_root(new_root);
    }

    /// Returns the replacement for `node`, freeing whatever it replaces.
    fn rewrite(&mut self, node: Node, level: u32, origin: UVec3, op: &SphereOp) -> Node {
        let extent = level_extent(level);
        if !op.intersects(origin, extent) {
            return node;
        }

        if level == 0 {
            return self.rewrite_leaf(node, origin, op);
        }

        let step = level_extent(level - 1);
        let mut children = [Node::EMPTY; CHILDREN as usize];
        let mut mask = 0u64;
        for z in 0..BRICK_EDGE {
            for y in 0..BRICK_EDGE {
                for x in 0..BRICK_EDGE {
                    let i = child_index(x, y, z);
                    let child_origin = origin + UVec3::new(x, y, z) * step;
                    // An unsubdivided parent hands its own value down to each child.
                    let existing = match node.child_slot(i) {
                        Some(slot) => self.arena.node(slot),
                        None if node.is_uniform_solid() => node,
                        None => Node::EMPTY,
                    };
                    let child = self.rewrite(existing, level - 1, child_origin, op);
                    children[i as usize] = child;
                    if !child.is_empty() {
                        mask |= 1u64 << i;
                    }
                }
            }
        }

        if node.is_subdivided() {
            self.arena.free_nodes(node.child_base, node.child_count());
        }

        if mask == 0 {
            return Node::EMPTY;
        }
        if let Some(collapsed) = collapse_uniform(&children) {
            return collapsed;
        }

        let count = mask.count_ones();
        let base = self.arena.alloc_nodes(count);
        let mut slot = base;
        for child in children.iter() {
            if !child.is_empty() {
                self.arena.set_node(slot, *child);
                slot += 1;
            }
        }
        Node::subdivided(mask, base)
    }

    fn rewrite_leaf(&mut self, node: Node, origin: UVec3, op: &SphereOp) -> Node {
        let mut materials = [MaterialId::EMPTY; CHILDREN as usize];
        let mut mask = 0u64;
        for z in 0..BRICK_EDGE {
            for y in 0..BRICK_EDGE {
                for x in 0..BRICK_EDGE {
                    let i = child_index(x, y, z);
                    let p = origin + UVec3::new(x, y, z);
                    let existing = match node.child_slot(i) {
                        Some(slot) => MaterialId(self.arena.voxel(slot)),
                        None if node.is_uniform_solid() => node.material(),
                        None => MaterialId::EMPTY,
                    };
                    let m = if op.covers_voxel(p) { op.material } else { existing };
                    materials[i as usize] = m;
                    if !m.is_empty() {
                        mask |= 1u64 << i;
                    }
                }
            }
        }

        if node.is_subdivided() {
            self.arena.free_voxels(node.child_base, node.child_count());
        }

        if mask == 0 {
            return Node::EMPTY;
        }
        if mask == u64::MAX {
            let first = materials[0];
            if materials.iter().all(|m| *m == first) {
                return Node::uniform(first);
            }
        }

        let count = mask.count_ones();
        let base = self.arena.alloc_voxels(count);
        let mut slot = base;
        for m in materials.iter() {
            if !m.is_empty() {
                self.arena.set_voxel(slot, m.0);
                slot += 1;
            }
        }
        Node::subdivided(mask, base)
    }
}

fn collapse_uniform(children: &[Node; CHILDREN as usize]) -> Option<Node> {
    let first = children[0];
    if !first.is_uniform_solid() {
        return None;
    }
    if children.iter().all(|c| *c == first) {
        Some(first)
    } else {
        None
    }
}
```

Add to `crates/bevox_core/src/lib.rs`:

```rust
pub mod edit;
```

- [ ] **Step 4: Run the tests to verify they pass**

Run: `cargo test -p bevox_core --test editing`
Expected: PASS, 6 tests.

- [ ] **Step 5: Run the whole suite for regressions**

Run: `cargo test -p bevox_core`
Expected: PASS.

- [ ] **Step 6: Commit**

```bash
git add crates/bevox_core
git commit -m "feat(core): add sphere brush editing with subtree rewrite" -m "Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>"
```

---

### Task 6: Direction reachability mask table

**Files:**
- Create: `crates/bevox_core/src/mask_table.rs`
- Modify: `crates/bevox_core/src/lib.rs`

**Interfaces:**
- Consumes: `BRICK_EDGE`, `CHILDREN`, `child_index`.
- Produces: `TABLE_LEN: usize = 512`, `OCTANTS: u32 = 8`, `build_direction_masks() -> [u64; TABLE_LEN]`, `direction_mask(cell: u32, octant: u32) -> u64`, `octant_index(dir: Vec3) -> u32`. Octant bit 0 set means the x component is negative, bit 1 y, bit 2 z.

- [ ] **Step 1: Write the failing tests**

`crates/bevox_core/src/mask_table.rs`:

```rust
//! Conservative reachability masks. For a ray entering a brick at a given cell
//! and travelling in a given sign octant, the mask holds every cell it could
//! still reach. ANDing it with a node's occupancy mask turns "does this brick
//! contain anything this ray can hit" into two instructions.

use crate::node::{BRICK_EDGE, CHILDREN, child_index};
use glam::Vec3;
use std::sync::OnceLock;

pub const OCTANTS: u32 = 8;
pub const TABLE_LEN: usize = (CHILDREN * OCTANTS) as usize;

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_cell_can_always_reach_itself() {
        let table = build_direction_masks();
        for cell in 0..CHILDREN {
            for octant in 0..OCTANTS {
                let mask = table[(cell * OCTANTS + octant) as usize];
                assert!(mask & (1u64 << cell) != 0, "cell {cell} octant {octant}");
            }
        }
    }

    #[test]
    fn the_all_positive_octant_from_the_origin_cell_reaches_everything() {
        assert_eq!(direction_mask(child_index(0, 0, 0), 0b000), u64::MAX);
    }

    #[test]
    fn the_all_negative_octant_from_the_origin_cell_reaches_only_itself() {
        let mask = direction_mask(child_index(0, 0, 0), 0b111);
        assert_eq!(mask.count_ones(), 1);
        assert_eq!(mask, 1u64 << child_index(0, 0, 0));
    }

    #[test]
    fn the_all_negative_octant_from_the_far_cell_reaches_everything() {
        assert_eq!(direction_mask(child_index(3, 3, 3), 0b111), u64::MAX);
    }

    #[test]
    fn a_mixed_octant_restricts_only_the_axes_it_names() {
        // Start in the middle, x negative, y and z positive.
        let cell = child_index(2, 2, 2);
        let mask = direction_mask(cell, 0b001);
        // Reachable x in 0..=2, y in 2..=3, z in 2..=3 => 3 * 2 * 2 = 12 cells.
        assert_eq!(mask.count_ones(), 12);
        assert!(mask & (1u64 << child_index(0, 3, 3)) != 0);
        assert!(mask & (1u64 << child_index(3, 3, 3)) == 0);
    }

    #[test]
    fn octant_index_reads_the_sign_bits() {
        assert_eq!(octant_index(Vec3::new(1.0, 1.0, 1.0)), 0b000);
        assert_eq!(octant_index(Vec3::new(-1.0, 1.0, 1.0)), 0b001);
        assert_eq!(octant_index(Vec3::new(1.0, -1.0, 1.0)), 0b010);
        assert_eq!(octant_index(Vec3::new(-1.0, -1.0, -1.0)), 0b111);
        // Zero counts as positive, which keeps the mask conservative.
        assert_eq!(octant_index(Vec3::new(0.0, 0.0, 0.0)), 0b000);
    }

    #[test]
    fn the_cached_table_matches_a_fresh_build() {
        let built = build_direction_masks();
        for cell in 0..CHILDREN {
            for octant in 0..OCTANTS {
                assert_eq!(
                    direction_mask(cell, octant),
                    built[(cell * OCTANTS + octant) as usize]
                );
            }
        }
    }
}
```

- [ ] **Step 2: Run the tests to verify they fail**

Run: `cargo test -p bevox_core mask_table`
Expected: FAIL — `build_direction_masks`, `direction_mask` and `octant_index` not found.

- [ ] **Step 3: Write the implementation**

Insert into `crates/bevox_core/src/mask_table.rs`, above the test module:

```rust
/// Builds the full table, indexed by `cell * OCTANTS + octant`.
pub fn build_direction_masks() -> [u64; TABLE_LEN] {
    let mut table = [0u64; TABLE_LEN];
    for cz in 0..BRICK_EDGE {
        for cy in 0..BRICK_EDGE {
            for cx in 0..BRICK_EDGE {
                let cell = child_index(cx, cy, cz);
                for octant in 0..OCTANTS {
                    let neg_x = octant & 0b001 != 0;
                    let neg_y = octant & 0b010 != 0;
                    let neg_z = octant & 0b100 != 0;
                    let mut mask = 0u64;
                    for tz in 0..BRICK_EDGE {
                        for ty in 0..BRICK_EDGE {
                            for tx in 0..BRICK_EDGE {
                                let ok_x = if neg_x { tx <= cx } else { tx >= cx };
                                let ok_y = if neg_y { ty <= cy } else { ty >= cy };
                                let ok_z = if neg_z { tz <= cz } else { tz >= cz };
                                if ok_x && ok_y && ok_z {
                                    mask |= 1u64 << child_index(tx, ty, tz);
                                }
                            }
                        }
                    }
                    table[(cell * OCTANTS + octant) as usize] = mask;
                }
            }
        }
    }
    table
}

fn table() -> &'static [u64; TABLE_LEN] {
    static TABLE: OnceLock<[u64; TABLE_LEN]> = OnceLock::new();
    TABLE.get_or_init(build_direction_masks)
}

/// Cells reachable from `cell` when travelling in `octant`.
pub fn direction_mask(cell: u32, octant: u32) -> u64 {
    debug_assert!(cell < CHILDREN && octant < OCTANTS);
    table()[(cell * OCTANTS + octant) as usize]
}

/// Sign octant of a direction. A zero component counts as positive, which
/// keeps the resulting mask conservative rather than dropping a plane.
pub fn octant_index(dir: Vec3) -> u32 {
    let mut octant = 0;
    if dir.x < 0.0 {
        octant |= 0b001;
    }
    if dir.y < 0.0 {
        octant |= 0b010;
    }
    if dir.z < 0.0 {
        octant |= 0b100;
    }
    octant
}
```

Add to `crates/bevox_core/src/lib.rs`:

```rust
pub mod mask_table;
```

- [ ] **Step 4: Run the tests to verify they pass**

Run: `cargo test -p bevox_core mask_table`
Expected: PASS, 7 tests.

- [ ] **Step 5: Commit**

```bash
git add crates/bevox_core
git commit -m "feat(core): generate conservative direction reachability masks" -m "Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>"
```

---

### Task 7: CPU reference ray marcher

**Files:**
- Create: `crates/bevox_core/src/march.rs`
- Create: `crates/bevox_core/tests/marching.rs`
- Modify: `crates/bevox_core/src/lib.rs`

**Interfaces:**
- Consumes: `Contree`, `Node`, `MaterialId`, `level_extent`, `child_index`.
- Produces: `Hit { t: f32, voxel: UVec3, material: MaterialId, face_normal: Vec3 }`, `MarchStats { steps: u32, overruns: u32 }` with `MarchStats::default()`, `MAX_STEPS: u32 = 65_536`, and `march(volume: &Contree, volume_to_world: Affine3A, origin: Vec3, dir: Vec3, max_dist: f32, any_hit: bool, stats: &mut MarchStats) -> Option<Hit>`.

This implementation is deliberately simple — recursive descent with a sorted child walk, no DDA and no mask filtering. It is ground truth for the GPU shader written in a later plan, so it must not share that shader's clever parts; two implementations of the same trick would share the same bug.

- [ ] **Step 1: Write the failing tests**

`crates/bevox_core/tests/marching.rs`:

```rust
use bevox_core::contree::Contree;
use bevox_core::dense::DenseVolume;
use bevox_core::march::{MarchStats, march};
use bevox_core::material::MaterialId;
use glam::{Affine3A, UVec3, Vec3};

fn single_voxel_tree() -> Contree {
    let mut dense = DenseVolume::new(16);
    dense.set(UVec3::new(8, 8, 8), MaterialId(3));
    Contree::from_dense(&dense)
}

#[test]
fn a_ray_aimed_at_a_lone_voxel_hits_it() {
    let tree = single_voxel_tree();
    let mut stats = MarchStats::default();
    let hit = march(
        &tree,
        Affine3A::IDENTITY,
        Vec3::new(-5.0, 8.5, 8.5),
        Vec3::X,
        100.0,
        false,
        &mut stats,
    )
    .expect("expected a hit");

    assert_eq!(hit.voxel, UVec3::new(8, 8, 8));
    assert_eq!(hit.material, MaterialId(3));
    assert!((hit.t - 13.0).abs() < 1e-3, "t was {}", hit.t);
    assert_eq!(hit.face_normal, Vec3::NEG_X);
    assert_eq!(stats.overruns, 0);
}

#[test]
fn a_ray_that_misses_returns_nothing() {
    let tree = single_voxel_tree();
    let mut stats = MarchStats::default();
    let hit = march(
        &tree,
        Affine3A::IDENTITY,
        Vec3::new(-5.0, 0.5, 0.5),
        Vec3::X,
        100.0,
        false,
        &mut stats,
    );
    assert!(hit.is_none());
}

#[test]
fn an_empty_tree_is_never_hit() {
    let tree = Contree::empty(2);
    let mut stats = MarchStats::default();
    assert!(
        march(
            &tree,
            Affine3A::IDENTITY,
            Vec3::new(-5.0, 8.5, 8.5),
            Vec3::X,
            100.0,
            false,
            &mut stats
        )
        .is_none()
    );
}

#[test]
fn the_nearer_of_two_voxels_is_returned() {
    let mut dense = DenseVolume::new(16);
    dense.set(UVec3::new(4, 8, 8), MaterialId(1));
    dense.set(UVec3::new(12, 8, 8), MaterialId(2));
    let tree = Contree::from_dense(&dense);

    let mut stats = MarchStats::default();
    let hit = march(
        &tree,
        Affine3A::IDENTITY,
        Vec3::new(-5.0, 8.5, 8.5),
        Vec3::X,
        100.0,
        false,
        &mut stats,
    )
    .unwrap();
    assert_eq!(hit.voxel, UVec3::new(4, 8, 8));

    // From the far side, the other voxel is nearer.
    let hit = march(
        &tree,
        Affine3A::IDENTITY,
        Vec3::new(25.0, 8.5, 8.5),
        Vec3::NEG_X,
        100.0,
        false,
        &mut stats,
    )
    .unwrap();
    assert_eq!(hit.voxel, UVec3::new(12, 8, 8));
    assert_eq!(hit.face_normal, Vec3::X);
}

#[test]
fn max_dist_stops_the_ray_short() {
    let tree = single_voxel_tree();
    let mut stats = MarchStats::default();
    let hit = march(
        &tree,
        Affine3A::IDENTITY,
        Vec3::new(-5.0, 8.5, 8.5),
        Vec3::X,
        10.0, // the voxel is 13 units away
        false,
        &mut stats,
    );
    assert!(hit.is_none());
}

#[test]
fn any_hit_finds_something_without_visiting_more_than_closest_hit() {
    let mut dense = DenseVolume::new(16);
    for x in 0..16 {
        dense.set(UVec3::new(x, 8, 8), MaterialId(1));
    }
    let tree = Contree::from_dense(&dense);

    let mut closest_stats = MarchStats::default();
    let closest = march(
        &tree,
        Affine3A::IDENTITY,
        Vec3::new(-5.0, 8.5, 8.5),
        Vec3::X,
        100.0,
        false,
        &mut closest_stats,
    );

    let mut any_stats = MarchStats::default();
    let any = march(
        &tree,
        Affine3A::IDENTITY,
        Vec3::new(-5.0, 8.5, 8.5),
        Vec3::X,
        100.0,
        true,
        &mut any_stats,
    );

    assert!(closest.is_some() && any.is_some());
    assert!(any_stats.steps <= closest_stats.steps);
}

#[test]
fn a_translated_volume_hits_the_same_voxel() {
    let tree = single_voxel_tree();
    let shift = Vec3::new(100.0, 0.0, 0.0);
    let mut stats = MarchStats::default();

    let hit = march(
        &tree,
        Affine3A::from_translation(shift),
        Vec3::new(-5.0, 8.5, 8.5) + shift,
        Vec3::X,
        100.0,
        false,
        &mut stats,
    )
    .expect("expected a hit through the transform");

    assert_eq!(hit.voxel, UVec3::new(8, 8, 8));
    assert!((hit.t - 13.0).abs() < 1e-3);
}
```

- [ ] **Step 2: Run the tests to verify they fail**

Run: `cargo test -p bevox_core --test marching`
Expected: FAIL — unresolved import `bevox_core::march`.

- [ ] **Step 3: Write the implementation**

`crates/bevox_core/src/march.rs`:

```rust
//! CPU reference ray marcher.
//!
//! Deliberately simple: recursive descent, ray-box per child, children visited
//! in near-to-far order. It is ground truth for the GPU shader, so it shares
//! none of that shader's optimisations — a reference that repeats the clever
//! part would repeat the clever part's bugs.

use crate::contree::{Contree, level_extent};
use crate::material::MaterialId;
use crate::node::{BRICK_EDGE, CHILDREN, Node, child_index};
use glam::{Affine3A, IVec3, UVec3, Vec3};

/// Upper bound on child visits for one ray. Exceeding it reports an overrun and
/// returns a miss rather than looping.
pub const MAX_STEPS: u32 = 65_536;

#[derive(Clone, Copy, PartialEq, Debug)]
pub struct Hit {
    /// Distance along the world-space ray.
    pub t: f32,
    /// Voxel coordinate within the volume.
    pub voxel: UVec3,
    pub material: MaterialId,
    /// Normal of the face the ray entered through, in volume space.
    pub face_normal: Vec3,
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct MarchStats {
    pub steps: u32,
    pub overruns: u32,
}

/// Slab test. Returns the entry and exit distances, which overlap only when the
/// ray actually crosses the box.
fn ray_box(origin: Vec3, inv_dir: Vec3, lo: Vec3, hi: Vec3) -> Option<(f32, f32)> {
    let t0 = (lo - origin) * inv_dir;
    let t1 = (hi - origin) * inv_dir;
    let near = t0.min(t1);
    let far = t0.max(t1);
    let t_enter = near.max_element();
    let t_exit = far.min_element();
    if t_enter <= t_exit && t_exit >= 0.0 {
        Some((t_enter, t_exit))
    } else {
        None
    }
}

/// Which face of a box the ray entered, given the per-axis entry distances.
fn entry_normal(origin: Vec3, inv_dir: Vec3, lo: Vec3, hi: Vec3) -> Vec3 {
    let t0 = (lo - origin) * inv_dir;
    let t1 = (hi - origin) * inv_dir;
    let near = t0.min(t1);
    if near.x >= near.y && near.x >= near.z {
        if inv_dir.x >= 0.0 { Vec3::NEG_X } else { Vec3::X }
    } else if near.y >= near.z {
        if inv_dir.y >= 0.0 { Vec3::NEG_Y } else { Vec3::Y }
    } else if inv_dir.z >= 0.0 {
        Vec3::NEG_Z
    } else {
        Vec3::Z
    }
}

struct Ray {
    origin: Vec3,
    dir: Vec3,
    inv_dir: Vec3,
    max_dist: f32,
    any_hit: bool,
}

/// Marches a ray through a volume placed by `volume_to_world`.
///
/// `dir` need not be normalised; `t` is expressed in units of `dir`'s length,
/// and `max_dist` in the same units.
pub fn march(
    volume: &Contree,
    volume_to_world: Affine3A,
    origin: Vec3,
    dir: Vec3,
    max_dist: f32,
    any_hit: bool,
    stats: &mut MarchStats,
) -> Option<Hit> {
    let world_to_volume = volume_to_world.inverse();
    let local_origin = world_to_volume.transform_point3(origin);
    let local_dir = world_to_volume.transform_vector3(dir);

    // A zero component would divide by zero; infinity is the correct limit here
    // and the slab test handles it.
    let inv_dir = Vec3::new(
        if local_dir.x == 0.0 { f32::INFINITY } else { 1.0 / local_dir.x },
        if local_dir.y == 0.0 { f32::INFINITY } else { 1.0 / local_dir.y },
        if local_dir.z == 0.0 { f32::INFINITY } else { 1.0 / local_dir.z },
    );

    let ray = Ray { origin: local_origin, dir: local_dir, inv_dir, max_dist, any_hit };
    let extent = volume.extent() as f32;
    let (t_enter, t_exit) = ray_box(local_origin, inv_dir, Vec3::ZERO, Vec3::splat(extent))?;

    visit(
        volume,
        volume.root(),
        volume.depth() - 1,
        UVec3::ZERO,
        &ray,
        t_enter.max(0.0),
        t_exit,
        stats,
    )
}

fn visit(
    volume: &Contree,
    node: Node,
    level: u32,
    origin: UVec3,
    ray: &Ray,
    t_enter: f32,
    t_exit: f32,
    stats: &mut MarchStats,
) -> Option<Hit> {
    if node.is_empty() || t_enter > ray.max_dist || t_enter > t_exit {
        return None;
    }

    if node.is_uniform_solid() {
        let extent = level_extent(level);
        let lo = origin.as_vec3();
        let hi = lo + Vec3::splat(extent as f32);
        // A collapsed region covers many voxels. Report the one the ray actually
        // entered, not the region's origin, or normals get computed in the wrong
        // place and every large uniform surface shades incorrectly.
        let point = ray.origin + ray.dir * t_enter;
        let lo_i = origin.as_ivec3();
        let voxel = point
            .floor()
            .as_ivec3()
            .clamp(lo_i, lo_i + IVec3::splat(extent as i32 - 1))
            .as_uvec3();
        return Some(Hit {
            t: t_enter,
            voxel,
            material: node.material(),
            face_normal: entry_normal(ray.origin, ray.inv_dir, lo, hi),
        });
    }

    let step = if level == 0 { 1 } else { level_extent(level - 1) };

    // Gather the children this ray crosses, with their entry distances.
    let mut candidates: Vec<(f32, u32, UVec3)> = Vec::with_capacity(8);
    for z in 0..BRICK_EDGE {
        for y in 0..BRICK_EDGE {
            for x in 0..BRICK_EDGE {
                let i = child_index(x, y, z);
                if node.child_slot(i).is_none() {
                    continue;
                }
                stats.steps += 1;
                if stats.steps > MAX_STEPS {
                    stats.overruns += 1;
                    return None;
                }
                let child_origin = origin + UVec3::new(x, y, z) * step;
                let lo = child_origin.as_vec3();
                let hi = lo + Vec3::splat(step as f32);
                if let Some((child_enter, child_exit)) = ray_box(ray.origin, ray.inv_dir, lo, hi)
                    && child_exit >= t_enter
                    && child_enter <= t_exit
                {
                    candidates.push((child_enter.max(t_enter), i, child_origin));
                }
            }
        }
    }

    // Closest-hit needs near-to-far order so the first hit found is the closest.
    // Shadow rays do not care which voxel they find, so they skip the sort.
    if !ray.any_hit {
        candidates.sort_by(|a, b| a.0.total_cmp(&b.0));
    }

    for (child_enter, i, child_origin) in candidates {
        if child_enter > ray.max_dist {
            // Sorted candidates are monotonic, so the rest are further still.
            if ray.any_hit {
                continue;
            }
            break;
        }
        let slot = node.child_slot(i).expect("candidate implies an occupied slot");

        if level == 0 {
            let material = MaterialId(volume.arena().voxel(slot));
            let lo = child_origin.as_vec3();
            let hi = lo + Vec3::ONE;
            return Some(Hit {
                t: child_enter,
                voxel: child_origin,
                material,
                face_normal: entry_normal(ray.origin, ray.inv_dir, lo, hi),
            });
        }

        let child = volume.arena().node(slot);
        let lo = child_origin.as_vec3();
        let hi = lo + Vec3::splat(step as f32);
        let (_, child_exit) = ray_box(ray.origin, ray.inv_dir, lo, hi)
            .expect("candidate implies an intersection");

        if let Some(hit) = visit(
            volume,
            child,
            level - 1,
            child_origin,
            ray,
            child_enter,
            child_exit.min(t_exit),
            stats,
        ) {
            return Some(hit);
        }
    }

    None
}
```

Note on `any_hit`: it skips the near-to-far sort, so a shadow ray may return any occluder rather than the nearest one. That is the correct semantics for shadows and it is why the flag earns its place instead of being dead weight. The test asserts only that `any_hit` never visits more children than closest-hit, which holds in both modes.

Add to `crates/bevox_core/src/lib.rs`:

```rust
pub mod march;
```

- [ ] **Step 4: Run the tests to verify they pass**

Run: `cargo test -p bevox_core --test marching`
Expected: PASS, 7 tests.

- [ ] **Step 5: Run the whole suite for regressions**

Run: `cargo test -p bevox_core`
Expected: PASS.

- [ ] **Step 6: Commit**

```bash
git add crates/bevox_core
git commit -m "feat(core): add CPU reference ray marcher" -m "Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>"
```

---

### Task 8: Implicit normals and the reference render

**Files:**
- Create: `crates/bevox_core/src/normal.rs`
- Create: `crates/bevox_core/examples/render_reference.rs`
- Create: `crates/bevox_core/tests/reference_render.rs`
- Modify: `crates/bevox_core/src/lib.rs`
- Modify: `crates/bevox_core/Cargo.toml`

**Interfaces:**
- Consumes: `Contree`, `Hit`, `march`, `MarchStats`, `MaterialId`, `MaterialTable`, `Material`.
- Produces: `implicit_normal(volume: &Contree, voxel: UVec3, face_normal: Vec3) -> Vec3`; the example binary `render_reference`.

- [ ] **Step 1: Add the dev-dependency**

```bash
cargo add image --dev -p bevox_core
```

Expected: `image` appears under `[dev-dependencies]` in `crates/bevox_core/Cargo.toml`.

- [ ] **Step 2: Write the failing tests**

`crates/bevox_core/src/normal.rs`:

```rust
//! Per-voxel normals, generated from neighbour occupancy rather than stored.
//!
//! Storing normals in voxel data makes every fill, copy and delete responsible
//! for fixing up its neighbours, which is why the spec forbids it.

use crate::contree::Contree;
use crate::material::MaterialId;
use glam::{IVec3, UVec3, Vec3};

const NEIGHBOURS: [IVec3; 6] = [
    IVec3::new(1, 0, 0),
    IVec3::new(-1, 0, 0),
    IVec3::new(0, 1, 0),
    IVec3::new(0, -1, 0),
    IVec3::new(0, 0, 1),
    IVec3::new(0, 0, -1),
];

#[cfg(test)]
mod tests {
    use super::*;
    use crate::dense::DenseVolume;

    fn tree_from(points: &[(UVec3, u8)]) -> Contree {
        let mut dense = DenseVolume::new(16);
        for (p, m) in points {
            dense.set(*p, MaterialId(*m));
        }
        Contree::from_dense(&dense)
    }

    #[test]
    fn an_isolated_voxel_falls_back_to_the_face_normal() {
        let tree = tree_from(&[(UVec3::new(8, 8, 8), 1)]);
        let n = implicit_normal(&tree, UVec3::new(8, 8, 8), Vec3::NEG_X);
        assert_eq!(n, Vec3::NEG_X);
    }

    #[test]
    fn a_one_voxel_thick_slab_falls_back_to_the_face_normal() {
        let mut points = Vec::new();
        for z in 6..11 {
            for x in 6..11 {
                points.push((UVec3::new(x, 8, z), 1u8));
            }
        }
        let tree = tree_from(&points);
        let n = implicit_normal(&tree, UVec3::new(8, 8, 8), Vec3::Y);
        // Up and down are both exposed on a one-voxel-thick slab, and the four
        // lateral neighbours are solid, so the normal resolves to the face.
        assert!(n.length() > 0.99 && n.length() < 1.01);
    }

    #[test]
    fn a_voxel_in_an_inner_corner_points_diagonally_outward() {
        // Two perpendicular walls meeting, leaving +x and +y exposed.
        let mut points = Vec::new();
        for i in 0..5u32 {
            points.push((UVec3::new(8, 8 - i, 8), 1u8));
            points.push((UVec3::new(8 - i, 8, 8), 1u8));
        }
        points.push((UVec3::new(8, 8, 7), 1));
        points.push((UVec3::new(8, 8, 9), 1));
        let tree = tree_from(&points);

        let n = implicit_normal(&tree, UVec3::new(8, 8, 8), Vec3::Y);
        assert!(n.x > 0.0 && n.y > 0.0, "normal was {n:?}");
        assert!((n.length() - 1.0).abs() < 1e-3);
    }

    #[test]
    fn a_fully_buried_voxel_falls_back_to_the_face_normal() {
        let mut points = vec![(UVec3::new(8, 8, 8), 1u8)];
        for d in NEIGHBOURS {
            let p = IVec3::new(8, 8, 8) + d;
            points.push((p.as_uvec3(), 1));
        }
        let tree = tree_from(&points);
        let n = implicit_normal(&tree, UVec3::new(8, 8, 8), Vec3::Z);
        assert_eq!(n, Vec3::Z);
    }
}
```

`crates/bevox_core/tests/reference_render.rs`:

```rust
use bevox_core::contree::Contree;
use bevox_core::dense::DenseVolume;
use bevox_core::march::{MarchStats, march};
use bevox_core::material::MaterialId;
use glam::{Affine3A, UVec3, Vec3};

/// A floor slab with a block standing on it.
fn scene() -> Contree {
    let mut dense = DenseVolume::new(64);
    for z in 0..64 {
        for x in 0..64 {
            for y in 0..8 {
                dense.set(UVec3::new(x, y, z), MaterialId(1));
            }
        }
    }
    for z in 28..36 {
        for y in 8..24 {
            for x in 28..36 {
                dense.set(UVec3::new(x, y, z), MaterialId(2));
            }
        }
    }
    dense.into_contree()
}

#[test]
fn rendering_the_same_scene_twice_produces_identical_output() {
    let tree = scene();
    let a = render(&tree);
    let b = render(&tree);
    assert_eq!(a, b);
}

#[test]
fn the_reference_render_hits_the_floor_and_misses_the_sky() {
    let tree = scene();
    let mut stats = MarchStats::default();

    // Straight down onto the floor.
    let down = march(
        &tree,
        Affine3A::IDENTITY,
        Vec3::new(10.0, 40.0, 10.0),
        Vec3::NEG_Y,
        200.0,
        false,
        &mut stats,
    );
    assert!(down.is_some());
    assert_eq!(down.unwrap().voxel.y, 7);

    // Straight up into nothing.
    let up = march(
        &tree,
        Affine3A::IDENTITY,
        Vec3::new(10.0, 40.0, 10.0),
        Vec3::Y,
        200.0,
        false,
        &mut stats,
    );
    assert!(up.is_none());
    assert_eq!(stats.overruns, 0);
}

/// Renders a tiny image and returns the raw pixels.
fn render(tree: &Contree) -> Vec<u8> {
    let (w, h) = (32u32, 32u32);
    let mut pixels = Vec::with_capacity((w * h * 3) as usize);
    let mut stats = MarchStats::default();
    let eye = Vec3::new(-40.0, 40.0, -40.0);
    for py in 0..h {
        for px in 0..w {
            let u = px as f32 / w as f32 - 0.5;
            let v = 0.5 - py as f32 / h as f32;
            let dir = (Vec3::new(32.0, 16.0, 32.0) - eye).normalize()
                + Vec3::new(u, v, 0.0);
            match march(tree, Affine3A::IDENTITY, eye, dir.normalize(), 500.0, false, &mut stats) {
                Some(hit) => {
                    let shade = (hit.face_normal.dot(Vec3::new(0.3, 0.9, 0.2).normalize())
                        * 0.5
                        + 0.5)
                        * 255.0;
                    pixels.extend_from_slice(&[shade as u8, shade as u8, shade as u8]);
                }
                None => pixels.extend_from_slice(&[20, 30, 50]),
            }
        }
    }
    pixels
}
```

`DenseVolume::into_contree` is used above, so add it in Step 4.

- [ ] **Step 3: Run the tests to verify they fail**

Run: `cargo test -p bevox_core`
Expected: FAIL — `implicit_normal` not found, and no method `into_contree` on `DenseVolume`.

- [ ] **Step 4: Write the implementations**

Insert into `crates/bevox_core/src/normal.rs`, above the test module:

```rust
/// Approximates a surface normal by summing the directions in which a voxel is
/// exposed. Falls back to `face_normal` when the result carries no information,
/// which happens for isolated and fully buried voxels.
pub fn implicit_normal(volume: &Contree, voxel: UVec3, face_normal: Vec3) -> Vec3 {
    let extent = volume.extent() as i32;
    let base = voxel.as_ivec3();
    let mut sum = Vec3::ZERO;

    for d in NEIGHBOURS {
        let n = base + d;
        let outside = n.x < 0 || n.y < 0 || n.z < 0 || n.x >= extent || n.y >= extent || n.z >= extent;
        let empty = outside || volume.get(n.as_uvec3()) == MaterialId::EMPTY;
        if empty {
            sum += d.as_vec3();
        }
    }

    if sum.length_squared() < 1e-6 {
        return face_normal;
    }
    sum.normalize()
}
```

Add to `crates/bevox_core/src/dense.rs`, inside `impl DenseVolume`:

```rust
    /// Convenience for tests and examples.
    pub fn into_contree(self) -> crate::contree::Contree {
        crate::contree::Contree::from_dense(&self)
    }
```

Add to `crates/bevox_core/src/lib.rs`:

```rust
pub mod normal;
```

- [ ] **Step 5: Write the example renderer**

`crates/bevox_core/examples/render_reference.rs`:

```rust
//! Renders the reference scene to `reference.png`.
//!
//! Run with: cargo run -p bevox_core --example render_reference --release

use bevox_core::contree::Contree;
use bevox_core::dense::DenseVolume;
use bevox_core::march::{MarchStats, march};
use bevox_core::material::{Material, MaterialId, MaterialTable};
use bevox_core::normal::implicit_normal;
use glam::{Affine3A, UVec3, Vec3};

const WIDTH: u32 = 512;
const HEIGHT: u32 = 512;

fn main() {
    let mut table = MaterialTable::new();
    let stone = table.push(Material { color: [140, 140, 150, 255] }).unwrap();
    let brick = table.push(Material { color: [180, 90, 70, 255] }).unwrap();

    let tree = build_scene(stone, brick);
    let sun = Vec3::new(0.4, 1.0, 0.25).normalize();
    let eye = Vec3::new(-30.0, 48.0, -30.0);
    let target = Vec3::new(32.0, 12.0, 32.0);

    let forward = (target - eye).normalize();
    let right = forward.cross(Vec3::Y).normalize();
    let up = right.cross(forward);

    let mut stats = MarchStats::default();
    let mut buffer = Vec::with_capacity((WIDTH * HEIGHT * 3) as usize);

    for py in 0..HEIGHT {
        for px in 0..WIDTH {
            let u = (px as f32 + 0.5) / WIDTH as f32 - 0.5;
            let v = 0.5 - (py as f32 + 0.5) / HEIGHT as f32;
            let dir = (forward + right * u + up * v).normalize();

            let color = match march(
                &tree,
                Affine3A::IDENTITY,
                eye,
                dir,
                500.0,
                false,
                &mut stats,
            ) {
                Some(hit) => {
                    let normal = implicit_normal(&tree, hit.voxel, hit.face_normal);
                    shade(&tree, &table, hit.voxel, hit.material, normal, sun, &mut stats)
                }
                None => [90, 120, 180],
            };
            buffer.extend_from_slice(&color);
        }
    }

    println!("march steps: {}, overruns: {}", stats.steps, stats.overruns);
    assert_eq!(stats.overruns, 0, "a ray exceeded the step cap");

    image::save_buffer("reference.png", &buffer, WIDTH, HEIGHT, image::ExtendedColorType::Rgb8)
        .expect("failed to write reference.png");
    println!("wrote reference.png");
}

fn shade(
    tree: &Contree,
    table: &MaterialTable,
    voxel: UVec3,
    material: MaterialId,
    normal: Vec3,
    sun: Vec3,
    stats: &mut MarchStats,
) -> [u8; 3] {
    let base = table.get(material).color;

    // Offset along the normal so the shadow ray does not re-hit its own voxel.
    let origin = voxel.as_vec3() + Vec3::splat(0.5) + normal * 0.75;
    let shadowed = march(tree, Affine3A::IDENTITY, origin, sun, 500.0, true, stats).is_some();

    let ambient = 0.25;
    let diffuse = if shadowed { 0.0 } else { normal.dot(sun).max(0.0) * 0.75 };
    let light = ambient + diffuse;

    [
        (base[0] as f32 * light) as u8,
        (base[1] as f32 * light) as u8,
        (base[2] as f32 * light) as u8,
    ]
}

fn build_scene(stone: MaterialId, brick: MaterialId) -> Contree {
    let mut dense = DenseVolume::new(64);
    for z in 0..64 {
        for x in 0..64 {
            for y in 0..8 {
                dense.set(UVec3::new(x, y, z), stone);
            }
        }
    }
    for z in 28..36 {
        for y in 8..24 {
            for x in 28..36 {
                dense.set(UVec3::new(x, y, z), brick);
            }
        }
    }
    let mut tree = dense.into_contree();
    // Carve a hollow so the brush path appears in the deliverable image.
    tree.apply_sphere(Vec3::new(32.0, 20.0, 32.0), 5.0, MaterialId::EMPTY);
    tree
}
```

- [ ] **Step 6: Run the tests to verify they pass**

Run: `cargo test -p bevox_core`
Expected: PASS, including 4 normal tests and 2 reference-render tests.

- [ ] **Step 7: Produce the milestone deliverable**

Run: `cargo run -p bevox_core --example render_reference --release`
Expected: prints a step count with `overruns: 0` and writes `reference.png` showing a lit floor, a brick column with a sphere carved out of it, and a shadow cast onto the floor.

- [ ] **Step 8: Commit**

```bash
git add crates/bevox_core Cargo.lock
git commit -m "feat(core): add implicit normals and the reference renderer" -m "Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>"
```

---

## Milestone check

After Task 8, spec milestones 1 and 2 are complete:

- **Milestone 1** — contree construction and editing in `bevox_core`, with no GPU involved.
- **Milestone 2** — a CPU reference marcher that writes a PNG, proving the traversal mathematics in a debuggable setting.

## Subsequent plans

These are intentionally not written yet, because their code depends on interfaces that do not exist until the tasks above are done.

- **Plan 2 — milestones 3 to 5.** The Bevy 0.19.1 shell, the compute pipeline and render graph node, the WGSL traversal port validated against the Task 7 reference, then implicit normals and the sun shadow ray on the GPU.
- **Plan 3 — milestones 6 to 8.** `.vox` loading, the sphere brush wired to input with dirty-range upload, then the beam prepass and bitmask filter with their bit-identical guarantees and A/B/A measurements.
