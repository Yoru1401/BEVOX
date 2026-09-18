//! Terrain that an edit cuts free becomes a body.
//!
//! After Dwyer's devlog #12: a depth-first search over face neighbours, in which
//! a piece that cannot reach the ground is no longer part of the world. His
//! search walks the tree's uniform nodes, which is much faster on a large
//! volume; this one walks voxels and gives up past a budget. The spec's
//! Detachment section says why, and what it would take to change.

use super::BUDGET;
use crate::body::Body;
use crate::contree::Contree;
use crate::material::{MaterialId, MaterialTable};
use crate::physics::classify::solid_at;
use glam::{IVec3, Quat, UVec3};
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

/// Turns the pieces an edit cut free into bodies, removing them from the world.
///
/// `room` is how many more bodies the scene can draw. Past it, the largest
/// pieces are taken and the rest stay in the world: a piece that vanished
/// because the cap was full would look like a bug, not a budget.
pub fn detach(
    tree: &mut Contree,
    materials: &MaterialTable,
    lo: IVec3,
    hi: IVec3,
    room: usize,
) -> Vec<Body> {
    let mut pieces = loose_pieces(tree, lo, hi, BUDGET);
    pieces.sort_by_key(|p| std::cmp::Reverse(p.len()));
    pieces.truncate(room);

    let mut bodies = Vec::new();
    for piece in pieces {
        let voxels: Vec<_> = piece.iter().map(|&p| (p, tree.get(p))).collect();
        tree.clear_voxels(&piece);
        if let Some(body) = body_from(&voxels, materials) {
            bodies.push(body);
        }
    }
    bodies
}

/// Builds a body holding `voxels`, placed where they were.
///
/// The volume is the smallest power of four that fits the piece, because that
/// is what a `Contree`'s extent must be, and the piece sits at its corner.
fn body_from(voxels: &[(UVec3, MaterialId)], materials: &MaterialTable) -> Option<Body> {
    let (first, _) = *voxels.first()?;
    let mut lo = first;
    let mut hi = first;
    for (p, _) in voxels {
        lo = lo.min(*p);
        hi = hi.max(*p);
    }
    let span = (hi - lo + UVec3::ONE).max_element();
    let mut extent = 4;
    while extent < span {
        extent *= 4;
    }
    let local: Vec<_> = voxels.iter().map(|(p, m)| (*p - lo, *m)).collect();
    let mut body = Body::new(Contree::from_voxels(extent, &local), lo.as_vec3(), Quat::IDENTITY);
    body.recompute(materials).then_some(body)
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

    /// The freed column leaves the world and arrives as a body, with its voxels
    /// in the same places and its mass computed.
    #[test]
    fn a_freed_piece_becomes_a_body_where_it_stood() {
        let materials = crate::physics::fixtures::materials();
        let mut tree = floor_and_column();
        tree.clear_voxels(&[UVec3::new(8, 1, 8), UVec3::new(8, 2, 8)]);
        let bodies = detach(&mut tree, &materials, IVec3::new(7, 0, 7), IVec3::new(9, 3, 9), 16);

        assert_eq!(bodies.len(), 1);
        let body = &bodies[0];
        assert!(body.mass.mass > 0.0, "the body has no mass");
        for y in 3..10 {
            assert!(tree.get(UVec3::new(8, y, 8)).is_empty(), "voxel {y} is still in the world");
        }
        assert!(!tree.get(UVec3::new(8, 0, 8)).is_empty(), "the floor went with it");

        let mut world: Vec<[i32; 3]> = body
            .volume
            .voxels()
            .iter()
            .map(|(p, _)| {
                let at = body.world_from_local().transform_point3(p.as_vec3() + 0.5);
                (at - 0.5).round().as_ivec3().to_array()
            })
            .collect();
        world.sort_unstable();
        let want: Vec<[i32; 3]> = (3..10).map(|y| [8, y, 8]).collect();
        assert_eq!(world, want, "the piece moved when it became a body");
    }

    /// With no room for another body, the piece stays in the world: geometry
    /// that cannot be drawn must not disappear.
    #[test]
    fn a_piece_stays_in_the_world_when_there_is_no_room() {
        let materials = crate::physics::fixtures::materials();
        let mut tree = floor_and_column();
        tree.clear_voxels(&[UVec3::new(8, 1, 8), UVec3::new(8, 2, 8)]);
        let bodies = detach(&mut tree, &materials, IVec3::new(7, 0, 7), IVec3::new(9, 3, 9), 0);
        assert!(bodies.is_empty());
        assert!(!tree.get(UVec3::new(8, 5, 8)).is_empty(), "the column vanished with nowhere to go");
    }

    /// The biggest pieces go first when there is not room for all of them.
    #[test]
    fn the_biggest_pieces_go_first() {
        let mut voxels = Vec::new();
        for z in 0..16 {
            for x in 0..16 {
                voxels.push((UVec3::new(x, 0, z), MaterialId(1)));
            }
        }
        for y in 1..3 {
            voxels.push((UVec3::new(4, y, 4), MaterialId(2)));
        }
        for y in 1..8 {
            voxels.push((UVec3::new(12, y, 12), MaterialId(2)));
        }
        let mut tree = Contree::from_voxels(16, &voxels);
        tree.clear_voxels(&[UVec3::new(4, 1, 4), UVec3::new(12, 1, 12)]);
        let materials = crate::physics::fixtures::materials();
        let bodies = detach(&mut tree, &materials, IVec3::new(3, 0, 3), IVec3::new(13, 2, 13), 1);
        assert_eq!(bodies.len(), 1);
        assert_eq!(bodies[0].volume.voxels().len(), 6, "the shorter column was taken instead");
        assert!(!tree.get(UVec3::new(4, 2, 4)).is_empty(), "the shorter column vanished");
    }
}
