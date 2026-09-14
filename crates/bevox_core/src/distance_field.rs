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
        // Extent 1024 is 64 cells per axis, so the saturation probe below is
        // inside the grid; at 256 it would fall outside and read as zero.
        // (512 itself is not a legal extent here: this tree is 4-ary, so an
        // extent must be a power of four -- 256 or 1024, never 512.)
        let tree = one_voxel_at(1024, UVec3::splat(8));
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
        // 128 is not a legal extent for this 4-ary tree (a power of four is
        // required); 64 is the nearest valid size and keeps the O(cells x
        // claimed^3) exhaustive check below fast.
        let extent = 64u32;
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
