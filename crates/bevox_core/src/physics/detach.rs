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

/// Face neighbours. Down is last, so it is popped first: the walk dives for the
/// floor, and most cuts are into ground that reaches it within a few steps.
const NEIGHBOURS: [IVec3; 6] =
    [IVec3::Y, IVec3::X, IVec3::NEG_X, IVec3::Z, IVec3::NEG_Z, IVec3::NEG_Y];

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
    while let Some(p) = stack.pop() {
        // Grounded: stop at once. The rest of this piece stays unwalked, and any
        // later walk that reaches it runs into what this one claimed and stops
        // too. Walking on would cost the whole budget for every cut into the
        // ground, which is most cuts.
        if p.y == 0 {
            return None;
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
                // An earlier walk's voxel: this is that walk's piece, which was
                // grounded or too big, or it would have been walked to the end
                // and every voxel of it claimed.
                Some(&other) if other != id => return None,
                Some(_) => {}
            }
        }
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

    /// The search against the plainest possible reference, over random scenes
    /// and budgets small enough to cut walks short.
    ///
    /// The reference labels every component around the box completely, with no
    /// budget and no early exit, then keeps the ones that never touch the floor
    /// and fit the budget. Any walk that stops early and leaves a later walk
    /// fenced in by its claims, and so reporting a scrap of a bigger piece as
    /// loose, disagrees with it.
    #[test]
    fn the_search_agrees_with_labelling_every_piece_in_full() {
        let mut rng = crate::testing::XorShift64::new(0xde7a_c4);
        for case in 0..300 {
            let mut voxels = Vec::new();
            for z in 0..16 {
                for y in 0..16 {
                    for x in 0..16 {
                        if rng.next_below(100) < 38 {
                            voxels.push((UVec3::new(x, y, z), MaterialId(1)));
                        }
                    }
                }
            }
            let tree = Contree::from_voxels(16, &voxels);
            let lo = IVec3::new(
                rng.next_below(12) as i32,
                rng.next_below(12) as i32,
                rng.next_below(12) as i32,
            );
            let hi = lo + IVec3::splat(3);
            let budget = [3, 8, 25, 60, BUDGET][case % 5];

            // Pieces are each sorted already; order the list of them by content.
            let key = |piece: &Vec<UVec3>| piece.iter().map(|p| p.to_array()).collect::<Vec<_>>();
            let mut got: Vec<_> = loose_pieces(&tree, lo, hi, budget).iter().map(key).collect();
            got.sort();
            let mut want: Vec<_> = reference(&tree, lo, hi, budget).iter().map(key).collect();
            want.sort();
            assert_eq!(got, want, "case {case}, budget {budget}, box {lo}..={hi}");
        }
    }

    fn reference(tree: &Contree, lo: IVec3, hi: IVec3, budget: usize) -> Vec<Vec<UVec3>> {
        let mut labelled: std::collections::HashSet<IVec3> = std::collections::HashSet::new();
        let mut out = Vec::new();
        for z in lo.z..=hi.z {
            for y in lo.y..=hi.y {
                for x in lo.x..=hi.x {
                    let seed = IVec3::new(x, y, z);
                    if !solid_at(tree, seed) || labelled.contains(&seed) {
                        continue;
                    }
                    let mut stack = vec![seed];
                    let mut piece = vec![];
                    labelled.insert(seed);
                    while let Some(p) = stack.pop() {
                        piece.push(p);
                        for step in [IVec3::X, IVec3::NEG_X, IVec3::Y, IVec3::NEG_Y, IVec3::Z, IVec3::NEG_Z] {
                            let n = p + step;
                            if solid_at(tree, n) && labelled.insert(n) {
                                stack.push(n);
                            }
                        }
                    }
                    if piece.iter().all(|p| p.y != 0) && piece.len() <= budget {
                        let mut piece: Vec<UVec3> = piece.iter().map(|p| p.as_uvec3()).collect();
                        piece.sort_unstable_by_key(|p| (p.z, p.y, p.x));
                        out.push(piece);
                    }
                }
            }
        }
        out
    }

    /// What a detachment costs. Not a gate; its numbers go in the plan's
    /// Measurements section.
    #[test]
    #[ignore]
    fn a_detach_is_timed() {
        let materials = crate::physics::fixtures::materials();
        let median = |mut v: Vec<f64>| {
            v.sort_by(|a, b| a.partial_cmp(b).unwrap());
            v[v.len() / 2]
        };

        // A column 8 by 8 and 40 tall on a floor, cut at its foot: the piece
        // is 2,560 voxels, all of which the walk visits and the body receives.
        let column = || {
            let mut voxels = Vec::new();
            for z in 0..64 {
                for x in 0..64 {
                    voxels.push((UVec3::new(x, 0, z), MaterialId(1)));
                }
            }
            for y in 1..42 {
                for z in 28..36 {
                    for x in 28..36 {
                        voxels.push((UVec3::new(x, y, z), MaterialId(2)));
                    }
                }
            }
            let mut tree = Contree::from_voxels(64, &voxels);
            let cut: Vec<UVec3> = (28..36)
                .flat_map(|z| (28..36).map(move |x| UVec3::new(x, 1, z)))
                .collect();
            tree.clear_voxels(&cut);
            tree
        };
        let freed = median(
            (0..9)
                .map(|_| {
                    let mut tree = column();
                    let start = std::time::Instant::now();
                    let bodies = detach(
                        &mut tree,
                        &materials,
                        IVec3::new(27, 0, 27),
                        IVec3::new(36, 2, 36),
                        16,
                    );
                    let ms = start.elapsed().as_secs_f64() * 1000.0;
                    assert_eq!(bodies.len(), 1);
                    ms
                })
                .collect(),
        );

        // A hole dug into solid ground 32 deep: nothing comes free, and the
        // question is what finding that out costs.
        let mut voxels = Vec::new();
        for z in 0..64 {
            for y in 0..32 {
                for x in 0..64 {
                    voxels.push((UVec3::new(x, y, z), MaterialId(1)));
                }
            }
        }
        let mut ground = Contree::from_voxels(64, &voxels);
        ground.apply_sphere(glam::Vec3::new(32.0, 31.0, 32.0), 5.0, MaterialId::EMPTY);
        let grounded = median(
            (0..9)
                .map(|_| {
                    let start = std::time::Instant::now();
                    let pieces =
                        loose_pieces(&ground, IVec3::new(26, 25, 26), IVec3::new(38, 32, 38), BUDGET);
                    let ms = start.elapsed().as_secs_f64() * 1000.0;
                    assert!(pieces.is_empty());
                    ms
                })
                .collect(),
        );
        println!("detach, median ms: 2560-voxel column freed {freed:.3}, hole in ground {grounded:.3}");
    }
}
