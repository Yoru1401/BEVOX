//! How full of solid voxels each coarse cell is, for ambient occlusion.
//!
//! After Dwyer's devlog #15. A point's surroundings are judged by how much
//! solid sits around it: a flat surface has half its neighbourhood full, an
//! inner corner about three quarters. He samples that by counting the voxels in
//! each 16-voxel cube and blending between cube centres, which is what this
//! grid holds. The shader does the blending.
//!
//! The same cells as the distance field, so both grids index alike and travel
//! in one buffer.

use crate::contree::{Contree, level_extent};
use crate::distance_field::CELL_VOXELS;
use crate::node::Node;
use glam::{IVec3, UVec3};
use std::ops::Range;

/// Voxels in a full cell.
const CELL_VOXELS_CUBED: u32 = CELL_VOXELS * CELL_VOXELS * CELL_VOXELS;

/// Solid voxels per coarse cell, as a fraction scaled to a byte.
#[derive(Clone, Debug)]
pub struct Fullness {
    cells: Vec<u8>,
    edge: u32,
}

impl Fullness {
    /// Cells per axis.
    pub fn edge(&self) -> u32 {
        self.edge
    }

    /// The raw grid, row major: `x + y * edge + z * edge * edge`.
    pub fn cells(&self) -> &[u8] {
        &self.cells
    }

    /// 0 for empty, 255 for solid. Outside the grid reads empty: beyond the
    /// world there is nothing to occlude anything.
    pub fn get(&self, cell: UVec3) -> u8 {
        if cell.x >= self.edge || cell.y >= self.edge || cell.z >= self.edge {
            return 0;
        }
        self.cells[self.index(cell) as usize]
    }

    fn index(&self, cell: UVec3) -> u32 {
        cell.x + cell.y * self.edge + cell.z * self.edge * self.edge
    }

    /// Counts the whole tree.
    pub fn build(tree: &Contree) -> Self {
        let edge = (tree.extent() / CELL_VOXELS).max(1);
        let mut counts = vec![0u32; (edge * edge * edge) as usize];
        count(tree, tree.root(), tree.depth() - 1, UVec3::ZERO, edge, None, &mut counts);
        let cells = counts.iter().map(|&n| as_byte(n)).collect();
        Self { cells, edge }
    }

    /// Recounts every cell the box `lo..=hi`, in voxels, touches, and returns
    /// the cell range to upload.
    ///
    /// An edit changes what is solid, and fullness has no safe direction to err
    /// in: too full darkens what should be lit, too empty lights what should be
    /// dark. So the cells it touched are counted again from the tree.
    pub fn recount(&mut self, tree: &Contree, lo: IVec3, hi: IVec3) -> Range<u32> {
        let cell = CELL_VOXELS as i32;
        let last = self.edge as i32 - 1;
        let lo = (lo.div_euclid(IVec3::splat(cell))).clamp(IVec3::ZERO, IVec3::splat(last));
        let hi = (hi.div_euclid(IVec3::splat(cell))).clamp(IVec3::ZERO, IVec3::splat(last));
        let (lo, hi) = (lo.as_uvec3(), hi.as_uvec3());

        let mut counts = vec![0u32; (self.edge * self.edge * self.edge) as usize];
        count(tree, tree.root(), tree.depth() - 1, UVec3::ZERO, self.edge, Some((lo, hi)), &mut counts);
        for z in lo.z..=hi.z {
            for y in lo.y..=hi.y {
                for x in lo.x..=hi.x {
                    let i = self.index(UVec3::new(x, y, z)) as usize;
                    self.cells[i] = as_byte(counts[i]);
                }
            }
        }
        self.index(lo)..self.index(hi) + 1
    }
}

/// A cell's count as the byte the shader reads.
fn as_byte(count: u32) -> u8 {
    (count * 255 / CELL_VOXELS_CUBED) as u8
}

/// Adds every solid voxel of `node` to the cell holding it, skipping nodes
/// outside `only` when it is given.
///
/// A node is aligned to its own extent and every extent is a power of four, so
/// a node either sits inside one cell or covers whole cells: there is never a
/// node straddling a cell boundary to split between two.
fn count(
    tree: &Contree,
    node: Node,
    level: u32,
    origin: UVec3,
    edge: u32,
    only: Option<(UVec3, UVec3)>,
    counts: &mut [u32],
) {
    if node.is_empty() {
        return;
    }
    let extent = level_extent(level);
    let lo = origin / CELL_VOXELS;
    let hi = (origin + UVec3::splat(extent - 1)) / CELL_VOXELS;
    if let Some((only_lo, only_hi)) = only
        && (lo.cmpgt(only_hi).any() || hi.cmplt(only_lo).any())
    {
        return;
    }

    let index = |c: UVec3| (c.x + c.y * edge + c.z * edge * edge) as usize;
    if !node.is_subdivided() {
        // Uniform solid: inside one cell it adds its own volume, and covering
        // whole cells it fills each of them.
        let per_cell = extent.min(CELL_VOXELS).pow(3);
        for z in lo.z..=hi.z {
            for y in lo.y..=hi.y {
                for x in lo.x..=hi.x {
                    counts[index(UVec3::new(x, y, z))] += per_cell;
                }
            }
        }
        return;
    }
    if level == 0 {
        // A brick's children are voxels, and its extent of 4 fits in a cell.
        counts[index(lo)] += node.child_count();
        return;
    }
    let step = level_extent(level - 1);
    for child in 0..64u32 {
        let Some(slot) = node.child_slot(child) else {
            continue;
        };
        let at = UVec3::new(child % 4, (child / 4) % 4, child / 16);
        count(tree, tree.arena().node(slot), level - 1, origin + at * step, edge, only, counts);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::dense::DenseVolume;
    use crate::material::MaterialId;
    use crate::testing::XorShift64;
    use glam::Vec3;

    /// Counts the cells the slow way, one voxel at a time.
    fn brute(tree: &Contree, edge: u32) -> Vec<u8> {
        let mut counts = vec![0u32; (edge * edge * edge) as usize];
        for (p, _) in tree.voxels() {
            let c = p / CELL_VOXELS;
            counts[(c.x + c.y * edge + c.z * edge * edge) as usize] += 1;
        }
        counts.iter().map(|&n| as_byte(n)).collect()
    }

    /// A solid cell reads full, an empty one reads empty, and a half-filled one
    /// reads about half.
    #[test]
    fn a_cell_reads_how_full_it_is() {
        let mut dense = DenseVolume::new(64).unwrap();
        for z in 0..16 {
            for y in 0..16 {
                for x in 0..16 {
                    dense.set(UVec3::new(x, y, z), MaterialId(1));
                }
            }
        }
        // Half of the next cell along x.
        for z in 0..16 {
            for y in 0..8 {
                for x in 16..32 {
                    dense.set(UVec3::new(x, y, z), MaterialId(1));
                }
            }
        }
        let full = Fullness::build(&dense.into_contree());
        assert_eq!(full.get(UVec3::new(0, 0, 0)), 255);
        assert_eq!(full.get(UVec3::new(2, 0, 0)), 0);
        let half = full.get(UVec3::new(1, 0, 0));
        assert!((120..=130).contains(&half), "a half-full cell reads {half}");
        assert_eq!(full.get(UVec3::new(99, 0, 0)), 0, "outside the grid is not empty");
    }

    /// The tree walk counts what counting voxels one at a time counts, on
    /// scenes with uniform nodes of every size and loose voxels between them.
    #[test]
    fn the_walk_counts_what_the_voxels_say() {
        let mut rng = XorShift64::new(0x000a_0cc1);
        for case in 0..40 {
            let mut dense = DenseVolume::new(64).unwrap();
            // Blocks of 16, blocks of 4, then loose voxels: every node size.
            for _ in 0..6 {
                let at = UVec3::new(rng.next_below(4), rng.next_below(4), rng.next_below(4)) * 16;
                for z in 0..16 {
                    for y in 0..16 {
                        for x in 0..16 {
                            dense.set(at + UVec3::new(x, y, z), MaterialId(1));
                        }
                    }
                }
            }
            for _ in 0..40 {
                let at = UVec3::new(rng.next_below(16), rng.next_below(16), rng.next_below(16)) * 4;
                for z in 0..4 {
                    for y in 0..4 {
                        for x in 0..4 {
                            dense.set(at + UVec3::new(x, y, z), MaterialId(2));
                        }
                    }
                }
            }
            for _ in 0..400 {
                let at = UVec3::new(rng.next_below(64), rng.next_below(64), rng.next_below(64));
                dense.set(at, MaterialId(1));
            }
            let tree = dense.into_contree();
            let full = Fullness::build(&tree);
            assert_eq!(full.cells(), brute(&tree, full.edge()), "case {case}");
        }
    }

    /// After an edit, a recount of what it touched matches a fresh build, and
    /// says which cells to upload.
    #[test]
    fn a_recount_matches_a_fresh_build() {
        let mut dense = DenseVolume::new(64).unwrap();
        for z in 0..64 {
            for y in 0..20 {
                for x in 0..64 {
                    dense.set(UVec3::new(x, y, z), MaterialId(1));
                }
            }
        }
        let mut tree = dense.into_contree();
        let mut full = Fullness::build(&tree);

        let (centre, radius) = (Vec3::new(30.0, 19.0, 30.0), 6.0);
        tree.apply_sphere(centre, radius, MaterialId::EMPTY);
        let reach = IVec3::splat(radius.ceil() as i32 + 1);
        let range = full.recount(&tree, centre.as_ivec3() - reach, centre.as_ivec3() + reach);
        assert_eq!(full.cells(), Fullness::build(&tree).cells(), "the recount disagrees with a build");
        assert!(range.end > range.start, "the recount uploaded nothing");

        // And painting it back.
        tree.apply_sphere(centre, radius, MaterialId(2));
        full.recount(&tree, centre.as_ivec3() - reach, centre.as_ivec3() + reach);
        assert_eq!(full.cells(), Fullness::build(&tree).cells(), "the recount after painting disagrees");
    }
}
