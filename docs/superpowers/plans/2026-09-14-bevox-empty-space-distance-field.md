# Empty-Space Distance Field Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Skip empty space by advancing a ray's start through a coarse distance field before it enters the tree, and keep that field correct under the sphere brush.

**Architecture:** A coarse grid stores the Chebyshev distance from each cell to the nearest solid voxel, in cells, **capped at 16**. Before traversal, a ray repeatedly samples the field and jumps to the far face of the empty cube the sample proves around it — sphere tracing on an L∞ metric. The field is built on the CPU at load and, when the brush paints, lowered in a bounded box around the edit; erasing needs no update at all. The shader reads it at a new binding behind a flag, so it can be measured A/B/A against the existing path and held to the same bit-identity standard as every other optimisation.

**Tech Stack:** Rust, Bevy 0.19.1, wgpu 29.0.4, glam 0.32, WGSL.

**Spec:** `docs/superpowers/specs/2026-09-13-bevox-raymarcher-core-design.md` — this is deferred scope from that spec's "what this does not do", now being taken up. The spec's Measurement and Testing sections bind it.

## Global Constraints

- Native desktop only. No web build, ever — permanently out of scope, not deferred.
- `bevox_core` has no Bevy and no GPU dependency. The field's construction belongs there and must stay dependency-free.
- Voxel data on the GPU is budgeted at **512 MB maximum**, checked at upload, an error rather than an allocation attempt. The field counts against it: `within_budget` in `crates/bevox_render/src/pipeline.rs` must grow to include it.
- Root volume up to 4096 per axis.
- No new dependencies in any crate.
- Tests use a seeded `bevox_core::testing::XorShift64`, never a random seed, and no test framework may be added.
- Every performance claim comes from interleaved A/B/A within a single session, with drift reported. A number measured today is never compared against one from yesterday.
- **An optimisation that changes a pixel is a defect, not a trade-off.** The one accepted exception is the documented graze convention on shadow rays, already capped at 4 pixels in `the_beam_prepass_never_skips_geometry`.
- Verify with `cargo test --workspace`. It works again as of `19100cc`.

## The three facts this design rests on

Read these before starting. Each one is load-bearing and none is obvious from the code.

**Chebyshev, not Euclidean.** The field stores the L∞ distance to the nearest solid voxel. A value of `d` at cell `c` means every cell within the axis-aligned cube of radius `d` around `c` is empty. That is what lets a ray advance safely *in any direction* — with a Euclidean field the safe advance depends on direction and the arithmetic stops being cheap.

**The cap at 16 is what makes editing bounded.** Painting a voxel at cell `X` lowers the true distance of every cell `Y` to at most `chebyshev(X, Y)`. That is unbounded in principle, so a naive invalidation would have to walk the whole field. But if no stored value ever exceeds `MAX_DISTANCE = 16`, then any `Y` further than 16 cells from `X` already stores a value `≤ 16 ≤ chebyshev(X, Y)` and is *already conservative without being touched*. So a paint only has to lower a 33×33×33 box. Raising the cap raises the invalidation cost cubically.

**Erasing never needs an update.** Removing voxels only increases true distances, so a stale field under-estimates. Under-estimating costs speed, never correctness. Only paint invalidates.

## File Structure

- `crates/bevox_core/src/distance_field.rs` — **new**. `DistanceField`: construction from a `Contree`, sampling, and the bounded lowering an edit needs. Pure data structure, no Bevy, no GPU; its own file because it is a self-contained algorithm with its own invariants, and it is where the correctness of the whole feature lives.
- `crates/bevox_core/src/lib.rs` — declare the module.
- `crates/bevox_render/src/upload.rs` — carry the field into `GpuSceneData`, and its dirty cells into `SceneUpdate`.
- `crates/bevox_render/src/pipeline.rs` — binding 7, the layout entry, the budget, and the partial write.
- `crates/bevox_render/assets/shaders/march.wgsl` — sample the field and advance the ray, behind `FLAG_DISTANCE_FIELD`.
- `crates/bevox/src/main.rs` — lower the field where the brush paints.
- `crates/bevox_render/tests/{common/mod.rs,gpu_parity.rs,gpu_bench.rs}` — harness binding, identity gate, measurement.

---

### Task 1: The distance field, on the CPU

Everything downstream trusts this to never over-estimate. It is built and tested with no GPU in sight.

**Files:**
- Create: `crates/bevox_core/src/distance_field.rs`
- Modify: `crates/bevox_core/src/lib.rs`

**Interfaces:**
- Produces: `pub const CELL_VOXELS: u32 = 16`, `pub const MAX_DISTANCE: u8 = 16`, `pub struct DistanceField { cells: Vec<u8>, edge: u32 }` with `pub fn build(tree: &Contree) -> Self`, `pub fn edge(&self) -> u32`, `pub fn cells(&self) -> &[u8]`, `pub fn get(&self, cell: UVec3) -> u8`, `pub fn sample_voxel(&self, p: UVec3) -> u8`, and `pub fn lower_around(&mut self, centre: Vec3, radius: f32) -> Range<u32>`.
- Consumes: `bevox_core::contree::Contree::{extent, get}`, `bevox_core::material::MaterialId`.

- [ ] **Step 1: Write the failing tests**

Create `crates/bevox_core/src/distance_field.rs` with this test module and nothing else yet:

```rust
//! A coarse Chebyshev distance field, for skipping empty space.
//!
//! Each cell holds the L-infinity distance, in cells, from itself to the
//! nearest solid voxel, capped at `MAX_DISTANCE`. A value of `d` at cell `c`
//! promises that every cell within the cube of radius `d` around `c` is empty,
//! which is what lets a ray advance by `d` cells in any direction at once.
//!
//! The promise is one-directional on purpose: the field may under-estimate and
//! cost speed, but it must never over-estimate, because a ray would then jump
//! over geometry. Every operation here preserves that.

#[cfg(test)]
mod tests {
    use super::*;
    use crate::dense::DenseVolume;
    use crate::material::MaterialId;
    use glam::UVec3;

    /// Sparse on purpose: a dense volume at extent 512 would allocate 134 MB
    /// just to hold one voxel.
    fn one_voxel_at(extent: u32, p: UVec3) -> Contree {
        Contree::from_voxels(extent, &[(p, MaterialId(1))])
    }

    /// The cell containing a solid voxel is at distance zero. Anything else
    /// would let a ray skip the voxel it is standing on.
    #[test]
    fn a_cell_holding_geometry_is_zero() {
        let tree = one_voxel_at(64, UVec3::new(20, 20, 20));
        let field = DistanceField::build(&tree);
        assert_eq!(field.get(UVec3::new(20 / CELL_VOXELS, 20 / CELL_VOXELS, 20 / CELL_VOXELS)), 0);
    }

    /// An empty volume is empty everywhere, so every cell saturates.
    #[test]
    fn an_empty_volume_is_uniformly_the_cap() {
        let tree = Contree::empty(3);
        let field = DistanceField::build(&tree);
        assert!(field.cells().iter().all(|d| *d == MAX_DISTANCE));
    }

    /// Distance grows with separation, and stops at the cap rather than
    /// running away.
    #[test]
    fn distance_grows_away_from_geometry_and_saturates() {
        // Extent 512 is 32 cells per axis, so the saturation probe below is
        // inside the grid; at 256 it would fall outside and read as zero.
        let tree = one_voxel_at(512, UVec3::splat(8));
        let field = DistanceField::build(&tree);
        let solid = UVec3::ZERO;
        assert_eq!(field.get(solid), 0);
        assert_eq!(field.get(solid + UVec3::new(1, 0, 0)), 1);
        assert_eq!(field.get(solid + UVec3::new(3, 0, 0)), 3);
        // Chebyshev, not Euclidean: a diagonal neighbour is as near as an
        // axial one.
        assert_eq!(field.get(solid + UVec3::new(3, 3, 3)), 3);
        assert_eq!(field.get(solid + UVec3::splat(MAX_DISTANCE as u32 + 4)), MAX_DISTANCE);
    }

    /// The invariant the whole feature rests on, checked exhaustively against
    /// the tree itself on a scene with awkward geometry.
    #[test]
    fn the_field_never_over_estimates() {
        let mut rng = crate::testing::XorShift64::new(20260914);
        let extent = 128u32;
        let mut dense = DenseVolume::new(extent).unwrap();
        for _ in 0..300 {
            dense.set(
                UVec3::new(
                    rng.next_below(extent),
                    rng.next_below(extent),
                    rng.next_below(extent),
                ),
                MaterialId(1),
            );
        }
        let tree = dense.into_contree();
        let field = DistanceField::build(&tree);

        let cells = extent / CELL_VOXELS;
        for cz in 0..cells {
            for cy in 0..cells {
                for cx in 0..cells {
                    let c = UVec3::new(cx, cy, cz);
                    let claimed = field.get(c);
                    // Every cell strictly inside the claimed cube must be
                    // empty, or the claim is a lie a ray would act on.
                    for dz in 0..claimed as u32 {
                        for dy in 0..claimed as u32 {
                            for dx in 0..claimed as u32 {
                                let n = c + UVec3::new(dx, dy, dz);
                                if n.x >= cells || n.y >= cells || n.z >= cells {
                                    continue;
                                }
                                assert!(
                                    cell_is_empty(&tree, n),
                                    "cell {c:?} claims {claimed} but {n:?} holds geometry"
                                );
                            }
                        }
                    }
                }
            }
        }
    }

    fn cell_is_empty(tree: &Contree, cell: UVec3) -> bool {
        let base = cell * CELL_VOXELS;
        for z in 0..CELL_VOXELS {
            for y in 0..CELL_VOXELS {
                for x in 0..CELL_VOXELS {
                    if tree.get(base + UVec3::new(x, y, z)) != MaterialId::EMPTY {
                        return false;
                    }
                }
            }
        }
        true
    }

    /// Painting lowers the field near the edit, and lowering is all an edit
    /// ever does -- a value that went up would be a ray jumping over the new
    /// geometry.
    #[test]
    fn painting_only_lowers_the_field() {
        let tree = Contree::empty(3);
        let mut field = DistanceField::build(&tree);
        let before: Vec<u8> = field.cells().to_vec();
        field.lower_around(Vec3::splat(32.0), 6.0);
        for (after, before) in field.cells().iter().zip(before.iter()) {
            assert!(after <= before, "a cell rose from {before} to {after}");
        }
        assert!(field.cells().iter().zip(before.iter()).any(|(a, b)| a < b), "nothing changed");
    }

    /// After a paint, the field must still never over-estimate -- checked
    /// against the tree the paint actually produced.
    #[test]
    fn the_field_is_still_conservative_after_a_paint() {
        let mut tree = Contree::empty(3);
        let mut field = DistanceField::build(&tree);

        let centre = Vec3::new(30.0, 30.0, 30.0);
        tree.apply_sphere(centre, 5.0, MaterialId(1));
        field.lower_around(centre, 5.0);

        let cells = tree.extent() / CELL_VOXELS;
        for cz in 0..cells {
            for cy in 0..cells {
                for cx in 0..cells {
                    let c = UVec3::new(cx, cy, cz);
                    let claimed = field.get(c) as u32;
                    for dz in 0..claimed {
                        for dy in 0..claimed {
                            for dx in 0..claimed {
                                let n = c + UVec3::new(dx, dy, dz);
                                if n.x >= cells || n.y >= cells || n.z >= cells {
                                    continue;
                                }
                                assert!(
                                    cell_is_empty(&tree, n),
                                    "after painting, cell {c:?} claims {claimed} but {n:?} is solid"
                                );
                            }
                        }
                    }
                }
            }
        }
    }

    /// The lowered range is what gets re-uploaded, so it must cover every cell
    /// the paint actually changed.
    #[test]
    fn the_returned_range_covers_every_changed_cell() {
        let tree = Contree::empty(3);
        let mut field = DistanceField::build(&tree);
        let before: Vec<u8> = field.cells().to_vec();
        let range = field.lower_around(Vec3::splat(32.0), 4.0);
        for (i, (a, b)) in field.cells().iter().zip(before.iter()).enumerate() {
            if a != b {
                let i = i as u32;
                assert!(
                    range.contains(&i),
                    "cell {i} changed but is outside the reported range {range:?}"
                );
            }
        }
    }
}
```

- [ ] **Step 2: Run the tests to verify they fail**

Run: `cargo test -p bevox_core --lib distance_field`
Expected: FAIL to compile — `cannot find type DistanceField`.

- [ ] **Step 3: Write the implementation**

Add above the test module:

```rust
use crate::contree::Contree;
use crate::material::MaterialId;
use glam::{UVec3, Vec3};
use std::ops::Range;

/// Voxels per field cell, per axis.
///
/// Sixteen is two levels of the contree's 4x subdivision, so a cell maps onto
/// whole nodes. At extent 4096 the field is 256 cells per axis: 16 MB at one
/// byte each, against a 512 MB budget.
pub const CELL_VOXELS: u32 = 16;

/// The largest distance a cell may store, in cells.
///
/// This is not an arbitrary cap. Painting a voxel lowers the true distance of
/// every cell within `chebyshev` reach of it, which is unbounded -- but a cell
/// further than `MAX_DISTANCE` away already stores a value no greater than
/// `MAX_DISTANCE`, and so is already conservative without being touched. The
/// cap is what turns invalidation from a whole-field walk into a bounded box,
/// and the box grows with its cube.
pub const MAX_DISTANCE: u8 = 16;

/// Chebyshev distance to the nearest solid voxel, per coarse cell.
#[derive(Clone, Debug)]
pub struct DistanceField {
    cells: Vec<u8>,
    edge: u32,
}

impl DistanceField {
    /// Cells per axis.
    pub fn edge(&self) -> u32 {
        self.edge
    }

    /// The raw grid, row major: `x + y * edge + z * edge * edge`.
    pub fn cells(&self) -> &[u8] {
        &self.cells
    }

    pub fn get(&self, cell: UVec3) -> u8 {
        if cell.x >= self.edge || cell.y >= self.edge || cell.z >= self.edge {
            return 0;
        }
        self.cells[self.index(cell) as usize]
    }

    /// The field value covering a voxel coordinate.
    pub fn sample_voxel(&self, p: UVec3) -> u8 {
        self.get(p / CELL_VOXELS)
    }

    fn index(&self, cell: UVec3) -> u32 {
        cell.x + cell.y * self.edge + cell.z * self.edge * self.edge
    }

    /// Builds the field for a whole tree.
    ///
    /// Two sweeps of a chamfer transform with all-one weights, which is exactly
    /// the Chebyshev distance: forward over increasing indices taking the
    /// already-computed neighbours below, then backward taking those above.
    pub fn build(tree: &Contree) -> Self {
        let edge = (tree.extent() / CELL_VOXELS).max(1);
        let mut cells = vec![MAX_DISTANCE; (edge * edge * edge) as usize];

        let mut field = Self { cells, edge };
        mark_solid(tree, tree.root(), tree.depth() - 1, UVec3::ZERO, &mut field);
        field.sweep();
        field
    }

    /// Propagates distances outward from the zeroes, in both directions.
    fn sweep(&mut self) {
        let edge = self.edge as i32;
        // Forward: every neighbour with a smaller index is already final.
        for z in 0..edge {
            for y in 0..edge {
                for x in 0..edge {
                    self.relax(x, y, z, &[(-1, 0, 0), (0, -1, 0), (0, 0, -1),
                        (-1, -1, 0), (-1, 0, -1), (0, -1, -1), (-1, -1, -1)]);
                }
            }
        }
        // Backward: the same for larger indices.
        for z in (0..edge).rev() {
            for y in (0..edge).rev() {
                for x in (0..edge).rev() {
                    self.relax(x, y, z, &[(1, 0, 0), (0, 1, 0), (0, 0, 1),
                        (1, 1, 0), (1, 0, 1), (0, 1, 1), (1, 1, 1)]);
                }
            }
        }
    }

    fn relax(&mut self, x: i32, y: i32, z: i32, offsets: &[(i32, i32, i32)]) {
        let edge = self.edge as i32;
        let here = self.index(UVec3::new(x as u32, y as u32, z as u32)) as usize;
        let mut best = self.cells[here];
        for (dx, dy, dz) in offsets {
            let (nx, ny, nz) = (x + dx, y + dy, z + dz);
            if nx < 0 || ny < 0 || nz < 0 || nx >= edge || ny >= edge || nz >= edge {
                // Outside the volume holds no geometry, so the cube's promise
                // is vacuously true out there and the neighbour contributes
                // nothing. Treating outside as solid instead would clamp every
                // border cell to 1 and make the field useless on small
                // volumes -- at extent 64 the grid is only 4 cells wide.
                // A ray that skips out of the volume is terminated by the root
                // slab test in `traverse`, so nothing is lost.
                continue;
            }
            let n = self.index(UVec3::new(nx as u32, ny as u32, nz as u32)) as usize;
            best = best.min(self.cells[n].saturating_add(1));
        }
        self.cells[here] = best.min(MAX_DISTANCE);
    }

    /// Lowers the field to account for a sphere of new geometry.
    ///
    /// Returns the contiguous cell range touched, for upload. Only lowering is
    /// ever needed: a cell beyond `MAX_DISTANCE` of the edit already stores a
    /// value no larger than its true distance.
    pub fn lower_around(&mut self, centre: Vec3, radius: f32) -> Range<u32> {
        let reach = radius / CELL_VOXELS as f32 + MAX_DISTANCE as f32 + 1.0;
        let lo = ((centre / CELL_VOXELS as f32) - Vec3::splat(reach))
            .max(Vec3::ZERO)
            .as_uvec3();
        let hi = ((centre / CELL_VOXELS as f32) + Vec3::splat(reach))
            .min(Vec3::splat((self.edge - 1) as f32))
            .as_uvec3();

        let solid_lo = ((centre - Vec3::splat(radius)) / CELL_VOXELS as f32).floor();
        let solid_hi = ((centre + Vec3::splat(radius)) / CELL_VOXELS as f32).ceil();

        for cz in lo.z..=hi.z {
            for cy in lo.y..=hi.y {
                for cx in lo.x..=hi.x {
                    let c = Vec3::new(cx as f32, cy as f32, cz as f32);
                    // Chebyshev distance from this cell to the painted box,
                    // which is what the new geometry can promise at worst.
                    let outside = (solid_lo - c).max(c - solid_hi).max(Vec3::ZERO);
                    let d = outside.x.max(outside.y).max(outside.z).floor() as u32;
                    let d = d.min(MAX_DISTANCE as u32) as u8;
                    let i = self.index(UVec3::new(cx, cy, cz)) as usize;
                    self.cells[i] = self.cells[i].min(d);
                }
            }
        }

        let first = self.index(lo);
        let last = self.index(hi) + 1;
        first..last
    }
}

/// Zeroes every cell a non-empty node covers.
///
/// Walks the tree rather than probing every voxel. Probing would be
/// `cells x 4096` lookups -- about a billion at extent 1024 -- where the walk
/// is linear in occupied nodes and stops as soon as a node fits inside a cell.
fn mark_solid(
    tree: &Contree,
    node: crate::node::Node,
    level: u32,
    origin: UVec3,
    field: &mut DistanceField,
) {
    if node.is_empty() {
        return;
    }
    let extent = crate::contree::level_extent(level);

    // A node no larger than a cell, or one with no structure left to descend
    // into, marks the cells it covers and stops. Levels 0 and 1 have extents 4
    // and 16, so this also guarantees the walk never reaches voxel children,
    // which live in a different arena and have no `Node` to recurse on.
    if extent <= CELL_VOXELS || !node.is_subdivided() {
        let lo = origin / CELL_VOXELS;
        let hi = (origin + UVec3::splat(extent - 1)) / CELL_VOXELS;
        for z in lo.z..=hi.z {
            for y in lo.y..=hi.y {
                for x in lo.x..=hi.x {
                    let i = field.index(UVec3::new(x, y, z)) as usize;
                    field.cells[i] = 0;
                }
            }
        }
        return;
    }

    let step = crate::contree::level_extent(level - 1);
    for i in 0..crate::node::CHILDREN {
        if let Some(slot) = node.child_slot(i) {
            // Inverse of child_index: x + y * 4 + z * 16.
            let c = UVec3::new(i % 4, (i / 4) % 4, i / 16);
            mark_solid(tree, tree.arena().node(slot), level - 1, origin + c * step, field);
        }
    }
}
```

Add `pub mod distance_field;` to `crates/bevox_core/src/lib.rs`.

- [ ] **Step 4: Run the tests to verify they pass**

Run: `cargo test -p bevox_core`
Expected: PASS. The exhaustive conservativeness tests are the ones that matter; if either fails, the sweep or the lowering is wrong and nothing downstream is worth writing.

> `build` is O(cells x 4096) because `cell_empty` walks every voxel of every cell. At extent 4096 that is 16.7M cells x 4096 voxel lookups, which is far too slow. **If the build takes more than a second or two at extent 1024, stop and report it** — the fix is to walk the tree's occupied nodes instead of probing every voxel, and that is a change worth making deliberately rather than discovering under a benchmark.

- [ ] **Step 5: Commit**

```bash
git add crates/bevox_core
git commit -m "feat(core): a coarse Chebyshev distance field for empty space" -m "Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>"
```

---

### Task 2: Carry the field to the GPU

**Files:**
- Modify: `crates/bevox_render/src/upload.rs`, `crates/bevox_render/src/pipeline.rs`
- Modify: `crates/bevox_render/tests/common/mod.rs`

**Interfaces:**
- Produces: `GpuSceneData` gains `pub distance_field: Vec<u32>` and `pub field_edge: u32`; `MARCH_BINDING_COUNT` becomes 8; binding 7 is `array<u32>`, four cells per word.
- Consumes: `bevox_core::distance_field::{DistanceField, CELL_VOXELS, MAX_DISTANCE}`.

> **A binding change touches four places that move together**: the layout tuple in `init_march_pipeline`, `MARCH_BINDING_COUNT`, the bind group entries in `dispatch_march`, and the harness's own layout in `tests/common/mod.rs`. `the_layout_declares_every_binding_the_shader_uses` catches the first two disagreeing with the shader; nothing catches the harness drifting, so check it by hand.

- [ ] **Step 1: Write the failing test**

Add to the test module in `crates/bevox_render/src/upload.rs`:

```rust
    #[test]
    fn the_field_packs_four_cells_to_a_word() {
        let field = bevox_core::distance_field::DistanceField::build(&Contree::empty(3));
        let packed = pack_field(&field);
        assert_eq!(packed.len(), field.cells().len().div_ceil(4));
        // Little-endian within the word, matching pack_voxels.
        let first = packed[0];
        for i in 0..4 {
            assert_eq!(
                ((first >> (i * 8)) & 0xFF) as u8,
                field.cells()[i as usize],
                "cell {i} is not in byte {i} of word 0"
            );
        }
    }
```

- [ ] **Step 2: Run it to verify it fails**

Run: `cargo test -p bevox_render --lib the_field_packs`
Expected: FAIL — `cannot find function pack_field`.

- [ ] **Step 3: Implement the upload**

In `crates/bevox_render/src/upload.rs`:

```rust
/// The field packed four cells to a word, matching `pack_voxels`.
pub fn pack_field(field: &bevox_core::distance_field::DistanceField) -> Vec<u32> {
    bevox_core::gpu::pack_voxels(field.cells())
}
```

Add to `GpuSceneData`:

```rust
    /// Chebyshev distance to the nearest solid voxel per coarse cell, packed
    /// four to a word.
    pub distance_field: Vec<u32>,
    /// Cells per axis, so the shader can index the grid.
    pub field_edge: u32,
```

`Default` gets `distance_field: vec![0], field_edge: 1` — a single zero cell, which claims no empty space anywhere and is therefore the safe empty-scene value. `build_gpu_scene` fills both from `DistanceField::build(&scene.tree)`.

In `crates/bevox_render/src/pipeline.rs`: raise `MARCH_BINDING_COUNT` to 8, add `storage_buffer_read_only_sized(false, NonZero::new(4))` as the eighth layout entry, add a `field: Buffer` to `MarchBuffers` created with `STORAGE | COPY_DST`, bind it at 7, and include it in `budget_bytes`:

```rust
pub fn budget_bytes(node_capacity: u32, voxel_word_capacity: u32, field_words: u32) -> u64 {
    u64::from(node_capacity) * size_of::<GpuNode>() as u64
        + u64::from(voxel_word_capacity) * 4
        + u64::from(field_words) * 4
}
```

Update `within_budget` and its three tests to take the new argument; pass `0` where a test is only exercising one array.

Mirror all of it in `crates/bevox_render/tests/common/mod.rs`: a `storage_entry(7, 4)` layout entry, a field buffer built from `pack_field(&DistanceField::build(tree))`, and a bind group entry at 7.

- [ ] **Step 4: Run the tests**

Run: `cargo test --workspace`
Expected: PASS, including `the_layout_declares_every_binding_the_shader_uses` — which will still pass because the shader has no binding 7 yet and `MARCH_BINDING_COUNT` is 8. **That is a false pass and the test cannot catch it.** Note it and move on; Task 3 adds the shader side and the test becomes meaningful again.

- [ ] **Step 5: Commit**

```bash
git add crates
git commit -m "feat(render): upload the distance field" -m "Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>"
```

---

### Task 3: Advance the ray through empty space

**Files:**
- Modify: `crates/bevox_render/assets/shaders/march.wgsl`
- Modify: `crates/bevox_render/src/upload.rs` (the flag)

**Interfaces:**
- Produces: `march_flags::DISTANCE_FIELD = 8`; `@group(0) @binding(7) var<storage, read> distance_field: array<u32>;`

- [ ] **Step 1: Write the identity test**

Add to `crates/bevox_render/tests/gpu_parity.rs`:

```rust
/// Skipping empty space must not change what is hit.
///
/// The field promises a cube of emptiness around each cell; a ray that jumps
/// further than the promise passes through geometry, and the symptom is holes
/// that open from some angles and not others. This sweeps angles, and runs the
/// thin scene, whose single-voxel walls are the thinnest thing a jump can
/// straddle.
#[test]
fn the_distance_field_leaves_output_bit_identical() {
    let Some((device, queue)) = gpu_device() else {
        eprintln!("no GPU adapter available, skipping");
        return;
    };
    let tree = thin_scene();
    let gpu_volume = GpuVolume::from_contree(&tree);
    let (width, height) = (96u32, 96u32);
    let centre = Vec3::splat(32.0);
    let shader = std::fs::read_to_string("assets/shaders/march.wgsl").expect("shader missing");

    for step in 0..8u32 {
        let angle = step as f32 * std::f32::consts::TAU / 8.0;
        let eye = centre + Vec3::new(angle.cos() * 90.0, 30.0, angle.sin() * 90.0);
        let view = Mat4::look_at_rh(eye, centre, Vec3::Y);
        let projection = Mat4::perspective_rh(0.9, 1.0, 0.1, 500.0);
        let world_from_clip = (projection * view).inverse();

        for entry in ["march_identity", "march_voxel_id", "march_normal", "march"] {
            let reference = run_march_flagged(
                &device, &queue, &shader, entry, world_from_clip, eye, &tree, &gpu_volume, width,
                height, march_flags::NONE,
            );
            for flags in [
                march_flags::DISTANCE_FIELD,
                march_flags::DEFAULT | march_flags::DISTANCE_FIELD,
            ] {
                let got = run_march_flagged(
                    &device, &queue, &shader, entry, world_from_clip, eye, &tree, &gpu_volume,
                    width, height, flags,
                );
                let differing =
                    reference.chunks(4).zip(got.chunks(4)).filter(|(a, b)| a != b).count();
                // Shaded output is allowed the documented grazing-shadow pixels.
                let allowed = if entry == "march" { 4 } else { 0 };
                assert!(
                    differing <= allowed,
                    "angle {step}, {entry}, flags {flags:#b}: {differing} of {} pixels differ",
                    width * height
                );
            }
        }
    }
}
```

- [ ] **Step 2: Run it to verify it fails**

Run: `cargo test -p bevox_render --test gpu_parity the_distance_field`
Expected: FAIL — `march_flags::DISTANCE_FIELD` does not exist.

- [ ] **Step 3: Implement the advance**

Add `pub const DISTANCE_FIELD: u32 = 8;` to `march_flags` in `crates/bevox_render/src/upload.rs`, leaving `DEFAULT` alone until Task 5 measures it.

In `march.wgsl`:

```wgsl
const FLAG_DISTANCE_FIELD: u32 = 8u;

@group(0) @binding(7) var<storage, read> distance_field: array<u32>;

/// Voxels per field cell, per axis. Must match bevox_core's CELL_VOXELS.
const FIELD_CELL: f32 = 16.0;

fn field_at(cell: vec3<i32>) -> u32 {
    let edge = i32(view.field_params.x);
    if cell.x < 0 || cell.y < 0 || cell.z < 0
        || cell.x >= edge || cell.y >= edge || cell.z >= edge {
        return 0u;
    }
    let i = u32(cell.x + cell.y * edge + cell.z * edge * edge);
    return (distance_field[i / 4u] >> ((i % 4u) * 8u)) & 0xFFu;
}

/// Advances `t` past empty space the field can prove is empty.
///
/// The field holds a Chebyshev distance, so a value of `d` at the ray's cell
/// promises the whole cube of `d` cells around it is empty. Advancing to that
/// cube's exit plane is therefore safe in any direction, which is the property
/// a Euclidean field would not give.
///
/// Conservative by construction: it never advances past the cube the field
/// promised, so it cannot skip geometry unless the field itself lied.
fn skip_empty_space(origin: vec3<f32>, dir: vec3<f32>, inv_dir: vec3<f32>, start: f32, max_dist: f32) -> f32 {
    var t = start;
    // A ray crosses a bounded number of cubes before it either hits something
    // or leaves; the bound stops a degenerate direction spinning here.
    for (var i = 0u; i < 64u; i = i + 1u) {
        if t > max_dist { return t; }
        let p = origin + dir * t;
        let cell = vec3<i32>(floor(p / FIELD_CELL));
        let d = field_at(cell);
        if d == 0u { return t; }

        // Exit plane of the cube of `d` cells around this one.
        let lo = (vec3<f32>(cell) - vec3<f32>(f32(d) - 1.0)) * FIELD_CELL;
        let hi = (vec3<f32>(cell) + vec3<f32>(f32(d))) * FIELD_CELL;
        let t0 = (lo - origin) * inv_dir;
        let t1 = (hi - origin) * inv_dir;
        let far = max(t0, t1);
        let exit = min(min(far.x, far.y), far.z);
        // Nudge past the boundary, or the next sample lands on the same cell
        // and the loop makes no progress.
        if exit <= t { return t; }
        t = exit + 1e-3;
    }
    return t;
}
```

`MarchUniform` gains `field_params: vec4<u32>` carrying `[field_edge, 0, 0, 0]`; add the matching `pub field_params: [u32; 4]` to the Rust struct, bump the size assertion from 112 to 128, and set it in both places that build the uniform (`march_uniform` and `prepare_march_buffers`) **and in `TestUniform` in the harness**, or the layout's minimum binding size will reject it.

Call it from `primary_hit`, before the beam seed is applied, so the two compose:

```wgsl
    var t_seed = 0.0;
    if flag_enabled(FLAG_BEAM) {
        t_seed = beam_seed(id, size);
    }
    if flag_enabled(FLAG_DISTANCE_FIELD) {
        t_seed = skip_empty_space(view.camera_position.xyz, dir, vec3<f32>(1.0) / dir, t_seed, max_ray_distance());
    }
```

and from `traverse_any` for shadow rays, which are half the cost of a lit frame.

- [ ] **Step 4: Run the tests**

Run: `cargo test -p bevox_render --test gpu_parity`
Expected: PASS.

> **If it fails on scattered single pixels**, the nudge `1e-3` is too small at large `t` and the loop is re-sampling the same cell; scale it with `t`. **If it fails on whole regions**, the cube bounds are wrong — check the `d - 1` in `lo`, which is there because the promise covers the cell itself plus `d - 1` around it. **If it fails only with `DEFAULT | DISTANCE_FIELD`**, the interaction with the beam seed is at fault, not the field: the beam's seed is already past some geometry-free distance and the field must advance *from* it, not replace it.

- [ ] **Step 5: Commit**

```bash
git add crates
git commit -m "perf(render): skip empty space with the distance field" -m "Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>"
```

---

### Task 4: Keep the field correct under the brush

**Files:**
- Modify: `crates/bevox_render/src/upload.rs`, `crates/bevox/src/main.rs`
- Modify: `crates/bevox_render/src/pipeline.rs`

**Interfaces:**
- Produces: `SceneUpdate` gains `pub field: Vec<FieldWrite>` where `pub struct FieldWrite { pub start_word: u32, pub words: Vec<u32> }`; `VoxelScene` gains `pub field: DistanceField`.

- [ ] **Step 1: Write the failing test**

Add to the test module in `crates/bevox_render/src/upload.rs`:

```rust
    /// Painting must update the field in the same frame as the voxels, or the
    /// GPU skips empty space that is no longer empty.
    #[test]
    fn a_paint_stages_the_field_cells_it_lowered() {
        let mut scene = edit_scene();
        scene.tree.apply_sphere(Vec3::splat(32.0), 6.0, MaterialId(2));
        scene.field.lower_around(Vec3::splat(32.0), 6.0);

        let update = stage_scene_update(&mut scene);
        assert!(!update.field.is_empty(), "a paint staged no field cells");

        let whole = pack_field(&scene.field);
        for write in &update.field {
            for (i, word) in write.words.iter().enumerate() {
                let w = write.start_word as usize + i;
                assert_eq!(*word, whole[w], "staged field word {w} differs from a full pack");
            }
        }
    }

    /// Erasing needs no field update at all: removing geometry only increases
    /// true distances, so a stale field under-estimates, which costs speed and
    /// never correctness.
    #[test]
    fn erasing_stages_no_field_cells() {
        let mut scene = edit_scene();
        scene.tree.apply_sphere(Vec3::splat(32.0), 6.0, MaterialId::EMPTY);
        let update = stage_scene_update(&mut scene);
        assert!(update.field.is_empty(), "an erase staged field cells it did not need to");
    }
```

- [ ] **Step 2: Run it to verify it fails**

Run: `cargo test -p bevox_render --lib a_paint_stages`
Expected: FAIL — `VoxelScene` has no field `field`.

- [ ] **Step 3: Implement it**

`VoxelScene` gains `pub field: DistanceField`, built alongside the tree wherever a scene is constructed (`crates/bevox/src/main.rs`, and the test helpers). `lower_around` returns the cell range it touched; `stage_scene_update` converts that to whole words the same way voxel ranges are converted, and clears it.

Keep the lowered range on the scene rather than recomputing it: add `pub field_dirty: Option<Range<u32>>` to `VoxelScene`, set by the brush, drained by `stage_scene_update`. An erase leaves it `None`, which is what makes the second test pass.

`brush_input` in `crates/bevox/src/main.rs` calls `scene.field.lower_around(centre, brush.radius)` **only when painting**, and records the returned range.

`prepare_march_buffers` writes `update.field` into the field buffer at `start_word * 4`, exactly as it does for voxels.

- [ ] **Step 4: Run the tests**

Run: `cargo test --workspace`
Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add crates
git commit -m "feat(render): keep the distance field correct under the brush" -m "Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>"
```

---

### Task 5: Measure it, then decide

**Files:**
- Modify: `crates/bevox_render/tests/gpu_bench.rs`, `crates/bevox_render/src/upload.rs`
- Modify: this plan file

- [ ] **Step 1: Add the flag to the benchmarks**

Add `("field", march_flags::DISTANCE_FIELD)` and `("all+field", march_flags::DEFAULT | march_flags::DISTANCE_FIELD)` to the flag list in `optimisations_are_measured_against_the_baseline`, and to `the_dispatches_are_timed_by_the_gpu`.

- [ ] **Step 2: Measure**

Run: `cargo test --release -p bevox_render --test gpu_bench optimisations -- --nocapture`
Run: `cargo test --release -p bevox_render --test gpu_bench the_dispatches_are_timed -- --ignored --nocapture`
Run: `cargo test --release -p bevox_render --test gpu_bench the_real_scenes -- --ignored --nocapture`

Record all three verbatim.

- [ ] **Step 3: Decide the default, honestly**

`march_flags::DEFAULT` gains `DISTANCE_FIELD` **only if** it measures faster than drift on the close-to-geometry camera, which is the case this work exists for. If it does not, leave it off and record the number saying so.

> This is the step most likely to go badly. The concern raised before this plan was written stands: nothing has profiled where the remaining 15 ms goes, and close to geometry rays hit quickly, so there may be little empty space left to skip. **A measurement showing no gain is the correct outcome to record, not a reason to keep tuning until the number moves.** Note also that the field costs memory and an invalidation path on every paint — if it does not pay, say plainly that it should be reverted rather than shipped switched off.

- [ ] **Step 4: Run the whole suite**

Run: `cargo test --workspace`
Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add crates docs
git commit -m "perf(render): measure the distance field and set the default" -m "Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>"
```

---

## Milestone check

Empty space is skipped by a conservative coarse field that survives editing, measured A/B/A against the existing path, and held to bit-identity except for the documented grazing-shadow pixels.

## What this plan deliberately does not do

No GPU-side field construction. It is built on the CPU at load, where it is testable without a device, and the build cost is measured rather than assumed.

No field for erasing. Removing geometry only makes the field conservative, and a rebuild would be pure cost for no correctness gain.

No LODs and no streaming. Those remain deferred.

No raising `MAX_DISTANCE` to chase a number. The cap is what bounds invalidation, and the box around a paint grows with its cube.
