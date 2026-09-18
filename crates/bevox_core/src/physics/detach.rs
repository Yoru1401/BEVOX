//! Terrain that an edit cuts free becomes a body.
//!
//! After Dwyer's devlog #12: a depth-first search over face neighbours, in which
//! a piece that cannot reach the ground is no longer part of the world. His
//! search walks the tree's uniform nodes, which is much faster on a large
//! volume; this one walks voxels and gives up past a budget. The spec's
//! Detachment section says why, and what it would take to change.

use crate::contree::Contree;
use crate::physics::classify::solid_at;
use glam::{IVec3, UVec3};
use std::collections::HashMap;

const NEIGHBOURS: [IVec3; 6] =
    [IVec3::X, IVec3::NEG_X, IVec3::Y, IVec3::NEG_Y, IVec3::Z, IVec3::NEG_Z];

/// The pieces of `tree` around the box `lo..=hi` that no longer reach the
/// ground, each sorted.
///
/// A piece is grounded when it holds a voxel at `y == 0`, or when it is larger
/// than `budget`, at which point the search gives up rather than walking a
/// mountain. Both leave the piece in the world, which is the safe direction.
pub fn loose_pieces(tree: &Contree, lo: IVec3, hi: IVec3, budget: usize) -> Vec<Vec<UVec3>> {
    // Which walk reached each voxel first. A later walk that runs into one of an
    // earlier walk's voxels is on that walk's piece, which was not loose, or it
    // would have been walked to the end and every voxel of it claimed.
    let mut seen: HashMap<IVec3, usize> = HashMap::new();
    let mut pieces = Vec::new();
    let mut walks = 0;
    for z in lo.z..=hi.z {
        for y in lo.y..=hi.y {
            for x in lo.x..=hi.x {
                let seed = IVec3::new(x, y, z);
                if !solid_at(tree, seed) || seen.contains_key(&seed) {
                    continue;
                }
                walks += 1;
                if let Some(piece) = walk(tree, seed, budget, walks, &mut seen) {
                    pieces.push(piece);
                }
            }
        }
    }
    pieces
}

/// Walks the piece holding `seed` as walk number `id`. `None` when it is
/// grounded, too big, or joins a piece an earlier walk already gave up on.
fn walk(
    tree: &Contree,
    seed: IVec3,
    budget: usize,
    id: usize,
    seen: &mut HashMap<IVec3, usize>,
) -> Option<Vec<UVec3>> {
    let mut stack = vec![seed];
    let mut piece = Vec::new();
    seen.insert(seed, id);
    let mut grounded = false;
    while let Some(p) = stack.pop() {
        if p.y == 0 {
            grounded = true;
        }
        piece.push(p.as_uvec3());
        if piece.len() > budget {
            return None;
        }
        for step in NEIGHBOURS {
            let n = p + step;
            if !solid_at(tree, n) {
                continue;
            }
            match seen.get(&n) {
                None => {
                    seen.insert(n, id);
                    stack.push(n);
                }
                // Stopped short of the end by an earlier walk that gave up: this
                // is that walk's piece, so it is not loose either.
                Some(&other) if other != id => grounded = true,
                Some(_) => {}
            }
        }
    }
    if grounded {
        return None;
    }
    piece.sort_unstable_by_key(|p| (p.z, p.y, p.x));
    Some(piece)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::material::MaterialId;
    use crate::physics::BUDGET;

    /// A floor with a column on it. Cutting the column's base leaves the rest of
    /// the column reaching nothing: that is the piece that falls.
    fn floor_and_column() -> Contree {
        let mut voxels = Vec::new();
        for z in 0..16 {
            for x in 0..16 {
                voxels.push((UVec3::new(x, 0, z), MaterialId(1)));
            }
        }
        for y in 1..10 {
            voxels.push((UVec3::new(8, y, 8), MaterialId(2)));
        }
        Contree::from_voxels(16, &voxels)
    }

    #[test]
    fn cutting_a_column_frees_what_is_above_the_cut() {
        let mut tree = floor_and_column();
        tree.clear_voxels(&[UVec3::new(8, 1, 8), UVec3::new(8, 2, 8)]);
        let pieces = loose_pieces(&tree, IVec3::new(7, 0, 7), IVec3::new(9, 3, 9), BUDGET);
        assert_eq!(pieces.len(), 1, "{pieces:?}");
        assert_eq!(pieces[0].len(), 7, "the freed column is 7 voxels: {:?}", pieces[0]);
        assert!(pieces[0].iter().all(|p| p.y >= 3), "the floor came along: {:?}", pieces[0]);
    }

    /// A notch frees what is above it and nothing below; a hole in the floor
    /// frees nothing at all.
    #[test]
    fn a_notch_frees_only_what_it_cuts_off() {
        let mut tree = floor_and_column();
        tree.clear_voxels(&[UVec3::new(8, 5, 8)]);
        let pieces = loose_pieces(&tree, IVec3::new(7, 4, 7), IVec3::new(9, 6, 9), BUDGET);
        assert_eq!(pieces.len(), 1);
        assert!(pieces[0].iter().all(|p| p.y > 5), "grounded voxels came along: {:?}", pieces[0]);

        let mut tree = floor_and_column();
        tree.clear_voxels(&[UVec3::new(2, 0, 2)]);
        assert!(loose_pieces(&tree, IVec3::new(1, 0, 1), IVec3::new(3, 1, 3), BUDGET).is_empty());
    }

    /// Two pieces cut free at once are two pieces.
    #[test]
    fn two_cuts_free_two_pieces() {
        let mut voxels = Vec::new();
        for z in 0..16 {
            for x in 0..16 {
                voxels.push((UVec3::new(x, 0, z), MaterialId(1)));
            }
        }
        for (x, z) in [(4, 4), (12, 12)] {
            for y in 1..5 {
                voxels.push((UVec3::new(x, y, z), MaterialId(2)));
            }
        }
        let mut tree = Contree::from_voxels(16, &voxels);
        tree.clear_voxels(&[UVec3::new(4, 1, 4), UVec3::new(12, 1, 12)]);
        let pieces = loose_pieces(&tree, IVec3::new(3, 0, 3), IVec3::new(13, 2, 13), BUDGET);
        assert_eq!(pieces.len(), 2, "{pieces:?}");
        for piece in &pieces {
            assert_eq!(piece.len(), 3);
        }
    }

    /// Past the budget the search gives up and calls the piece grounded, which is
    /// the safe direction: nothing is detached that should not be.
    #[test]
    fn a_piece_past_the_budget_is_left_alone() {
        let mut voxels = Vec::new();
        // A slab floating with no floor: every voxel is loose, and there are
        // more of them than a small budget allows.
        for z in 0..16 {
            for y in 4..8 {
                for x in 0..16 {
                    voxels.push((UVec3::new(x, y, z), MaterialId(1)));
                }
            }
        }
        let tree = Contree::from_voxels(16, &voxels);
        assert!(loose_pieces(&tree, IVec3::new(0, 4, 0), IVec3::new(2, 6, 2), 100).is_empty());
        assert_eq!(
            loose_pieces(&tree, IVec3::new(0, 4, 0), IVec3::new(2, 6, 2), BUDGET).len(),
            1,
            "with room to finish, the slab is one loose piece"
        );
    }

    /// Diagonals do not hold a piece up: only faces join voxels, as in Dwyer's
    /// devlog #12.
    #[test]
    fn a_diagonal_touch_does_not_count_as_joined() {
        let mut voxels = Vec::new();
        for x in 0..4 {
            voxels.push((UVec3::new(x, 0, 0), MaterialId(1)));
        }
        voxels.push((UVec3::new(4, 1, 1), MaterialId(2)));
        let tree = Contree::from_voxels(16, &voxels);
        let pieces = loose_pieces(&tree, IVec3::new(3, 0, 0), IVec3::new(4, 1, 1), BUDGET);
        assert_eq!(pieces, vec![vec![UVec3::new(4, 1, 1)]]);
    }
}
