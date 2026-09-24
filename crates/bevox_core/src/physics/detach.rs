//! Terrain that an edit cuts free becomes a body.
//!
//! After Dwyer's devlog #12: a depth-first search over face neighbours, in which
//! a piece that cannot reach the ground is no longer part of the world. The
//! search walks the tree's uniform nodes, so a step crosses a whole region of
//! solid terrain rather than one voxel, and gives up past a budget.

use super::BUDGET;
use crate::body::Body;
use crate::contree::{Contree, level_extent};
use crate::material::{MaterialId, MaterialTable};
use crate::node::child_index;
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
    // Which walk reached each cell first, by the cell's origin. A later walk
    // that runs into one of an earlier walk's cells is on that walk's piece,
    // which was not loose, or it would have been walked to the end and every
    // cell of it claimed.
    let mut seen: HashMap<IVec3, usize> = HashMap::new();
    let mut pieces = Vec::new();
    let mut walks = 0;
    for z in lo.z..=hi.z {
        for y in lo.y..=hi.y {
            for x in lo.x..=hi.x {
                let seed = IVec3::new(x, y, z);
                let Some(cell) = cell_at(tree, seed) else {
                    continue;
                };
                if seen.contains_key(&cell.origin) {
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

/// One step of the walk: a box of solid voxels the tree holds as a single
/// node, or one voxel where it is subdivided to the leaf.
///
/// A node's box is aligned to its own size, so the box spans
/// `origin .. origin + size` on each axis and `origin.y == 0` is exactly "on
/// the ground".
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
struct Cell {
    origin: IVec3,
    size: i32,
}

impl Cell {
    fn voxels(self) -> usize {
        let n = self.size as usize;
        n * n * n
    }

    /// The far corner, one past the box.
    fn end(self) -> IVec3 {
        self.origin + IVec3::splat(self.size)
    }
}

/// The cell holding `p`: the uniform solid node it lands in, or its own voxel.
/// `None` where the tree is empty there, or `p` is outside it.
///
/// The descent is `Contree::get`'s, keeping the box each step narrows to.
fn cell_at(tree: &Contree, p: IVec3) -> Option<Cell> {
    let extent = tree.extent() as i32;
    if p.cmplt(IVec3::ZERO).any() || p.cmpge(IVec3::splat(extent)).any() {
        return None;
    }
    let mut node = tree.root();
    let mut level = tree.depth() - 1;
    let mut origin = IVec3::ZERO;
    loop {
        if !node.is_subdivided() {
            let size = level_extent(level) as i32;
            return node.is_uniform_solid().then_some(Cell { origin, size });
        }
        let local = (p - origin).as_uvec3();
        if level == 0 {
            let i = child_index(local.x, local.y, local.z);
            return node.child_slot(i).map(|_| Cell { origin: p, size: 1 });
        }
        let step = level_extent(level - 1);
        let cell = local / step;
        let i = child_index(cell.x, cell.y, cell.z);
        let slot = node.child_slot(i)?;
        node = tree.arena().node(slot);
        origin += (cell * step).as_ivec3();
        level -= 1;
    }
}

/// Every cell touching `cell` across its six faces.
///
/// Each face is scanned column by column just outside the box, and each hit
/// skips the width of the cell it found, so the scan costs one lookup per
/// neighbour rather than one per voxel of the face. Against a wall of single
/// voxels that is one lookup each, which is what the voxel walk paid anyway.
fn neighbours(tree: &Contree, cell: Cell, out: &mut Vec<Cell>) {
    out.clear();
    let (lo, hi) = (cell.origin, cell.end());
    for axis in 0..3 {
        let (u, v) = ([1, 0, 0][axis], [2, 2, 1][axis]);
        for side in [lo[axis] - 1, hi[axis]] {
            let mut a = lo[u];
            while a < hi[u] {
                let mut step_u = cell.size;
                // Whether this column had a hole. Skipping ahead is only safe
                // when it did not: past a hole, the next column may hold a
                // neighbour this one never saw.
                let mut gap = false;
                let mut b = lo[v];
                while b < hi[v] {
                    let mut at = IVec3::ZERO;
                    at[axis] = side;
                    at[u] = a;
                    at[v] = b;
                    match cell_at(tree, at) {
                        Some(found) => {
                            out.push(found);
                            // Skip what this neighbour covers on both axes: on
                            // `v` now, and on `u` no further than it reaches.
                            step_u = step_u.min(found.end()[u] - a);
                            b = found.end()[v];
                        }
                        None => {
                            gap = true;
                            b += 1;
                        }
                    }
                }
                a += if gap { 1 } else { step_u.max(1) };
            }
        }
    }
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
    let start = cell_at(tree, seed)?;
    let mut stack = vec![start];
    let mut cells = Vec::new();
    let mut voxels = 0usize;
    let mut found = Vec::new();
    seen.insert(start.origin, id);
    while let Some(cell) = stack.pop() {
        // Grounded: stop at once. The rest of this piece stays unwalked, and any
        // later walk that reaches it runs into what this one claimed and stops
        // too. Walking on would cost the whole budget for every cut into the
        // ground, which is most cuts.
        if cell.origin.y == 0 {
            return None;
        }
        cells.push(cell);
        voxels += cell.voxels();
        if voxels > budget {
            return None;
        }
        neighbours(tree, cell, &mut found);
        for n in &found {
            match seen.get(&n.origin) {
                None => {
                    seen.insert(n.origin, id);
                    stack.push(*n);
                }
                // An earlier walk's cell: this is that walk's piece, which was
                // grounded or too big, or it would have been walked to the end
                // and every cell of it claimed.
                Some(&other) if other != id => return None,
                Some(_) => {}
            }
        }
    }
    let mut piece = Vec::with_capacity(voxels);
    for cell in cells {
        for z in cell.origin.z..cell.end().z {
            for y in cell.origin.y..cell.end().y {
                for x in cell.origin.x..cell.end().x {
                    piece.push(IVec3::new(x, y, z).as_uvec3());
                }
            }
        }
    }
    piece.sort_unstable_by_key(|p| (p.z, p.y, p.x));
    Some(piece)
}

/// Every face-connected piece of `tree`, each sorted, largest first.
///
/// No floor and no budget: inside a body every piece is loose, and a body is
/// small enough to label in full. This is what decides whether an erase cut a
/// body in two.
pub fn components(tree: &Contree) -> Vec<Vec<UVec3>> {
    let mut seen: std::collections::HashSet<IVec3> = std::collections::HashSet::new();
    let mut pieces = Vec::new();
    for (voxel, _) in tree.voxels() {
        let seed = voxel.as_ivec3();
        if !seen.insert(seed) {
            continue;
        }
        let mut stack = vec![seed];
        let mut piece = Vec::new();
        while let Some(p) = stack.pop() {
            piece.push(p.as_uvec3());
            for step in NEIGHBOURS {
                let n = p + step;
                if solid_at(tree, n) && seen.insert(n) {
                    stack.push(n);
                }
            }
        }
        piece.sort_unstable_by_key(|p| (p.z, p.y, p.x));
        pieces.push(piece);
    }
    pieces.sort_by_key(|p| std::cmp::Reverse(p.len()));
    pieces
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
        if let Some(mut body) = body_from(&voxels, materials) {
            // It came out of the terrain, so it may go back into it once it
            // settles out of sight. See `physics::merge`.
            body.from_terrain = true;
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

    /// What detachment frees came out of the terrain, and says so: only such a
    /// body may merge back into it.
    #[test]
    fn a_freed_piece_knows_it_came_from_the_terrain() {
        let materials = crate::physics::fixtures::materials();
        let mut tree = floor_and_column();
        tree.clear_voxels(&[UVec3::new(8, 1, 8), UVec3::new(8, 2, 8)]);
        let freed = detach(&mut tree, &materials, IVec3::new(7, 0, 7), IVec3::new(9, 3, 9), 16);
        assert!(!freed.is_empty(), "nothing came free, so this proves nothing");
        assert!(freed.iter().all(|b| b.from_terrain), "a freed piece does not know it was terrain");
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
        let mut rng = crate::testing::XorShift64::new(0x00de_7ac4);
        for case in 0..600 {
            // Half the cases are loose voxels, where every cell of the walk is
            // one voxel; half are whole 4-blocks, which the tree keeps as
            // uniform nodes, so the walk steps over boxes of 64 voxels and
            // crosses between sizes. Only the second kind exercises that.
            let blocky = case % 2 == 1;
            let mut voxels = Vec::new();
            if blocky {
                for z in (0..16).step_by(4) {
                    for y in (0..16).step_by(4) {
                        for x in (0..16).step_by(4) {
                            if rng.next_below(100) < 45 {
                                continue;
                            }
                            for dz in 0..4 {
                                for dy in 0..4 {
                                    for dx in 0..4 {
                                        voxels.push((UVec3::new(x + dx, y + dy, z + dz), MaterialId(1)));
                                    }
                                }
                            }
                        }
                    }
                }
                // Loose voxels among the blocks, so cells of both sizes meet
                // and a neighbour may touch a face anywhere, not only at the
                // corner the scan starts from.
                for z in 0..16 {
                    for y in 0..16 {
                        for x in 0..16 {
                            if rng.next_below(100) < 12 {
                                voxels.push((UVec3::new(x, y, z), MaterialId(1)));
                            }
                        }
                    }
                }
                voxels.sort_unstable_by_key(|(p, _)| (p.z, p.y, p.x));
                voxels.dedup_by_key(|(p, _)| *p);
            } else {
                for z in 0..16 {
                    for y in 0..16 {
                        for x in 0..16 {
                            if rng.next_below(100) < 38 {
                                voxels.push((UVec3::new(x, y, z), MaterialId(1)));
                            }
                        }
                    }
                }
            }
            let tree = Contree::from_voxels(16, &voxels);
            if blocky {
                let sizes = voxels.iter().filter_map(|(p, _)| cell_at(&tree, p.as_ivec3()));
                assert!(
                    sizes.map(|c| c.size).max().unwrap_or(1) > 1,
                    "case {case} holds no uniform node, so the walk never steps over a box"
                );
            }
            let lo = IVec3::new(
                rng.next_below(12) as i32,
                rng.next_below(12) as i32,
                rng.next_below(12) as i32,
            );
            let hi = lo + IVec3::splat(3);
            let budget = [3, 8, 25, 60, 200, BUDGET][case % 6];

            // Pieces are each sorted already; order the list of them by content.
            let key = |piece: &Vec<UVec3>| piece.iter().map(|p| p.to_array()).collect::<Vec<_>>();
            let mut got: Vec<_> = loose_pieces(&tree, lo, hi, budget).iter().map(key).collect();
            got.sort();
            let mut want: Vec<_> = reference(&tree, lo, hi, budget).iter().map(key).collect();
            want.sort();
            assert_eq!(got, want, "case {case}, budget {budget}, box {lo}..={hi}");
            // And against the walk it replaced, voxel for voxel.
            let mut voxelwise: Vec<_> =
                loose_pieces_by_voxel(&tree, lo, hi, budget).iter().map(key).collect();
            voxelwise.sort();
            assert_eq!(got, voxelwise, "the cell walk and the voxel walk disagree, case {case}");
        }
    }

    /// The voxel walk the cell walk replaced, kept to measure against: the same
    /// early exit and the same budget, one voxel a step.
    fn loose_pieces_by_voxel(tree: &Contree, lo: IVec3, hi: IVec3, budget: usize) -> Vec<Vec<UVec3>> {
        fn walk_voxels(
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
                        Some(&other) if other != id => return None,
                        Some(_) => {}
                    }
                }
            }
            piece.sort_unstable_by_key(|p| (p.z, p.y, p.x));
            Some(piece)
        }

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
                    if let Some(piece) = walk_voxels(tree, seed, budget, walks, &mut seen) {
                        pieces.push(piece);
                    }
                }
            }
        }
        pieces
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

    #[test]
    fn components_are_labelled_largest_first() {
        let mut voxels = Vec::new();
        for x in 0..5 {
            voxels.push((UVec3::new(x, 0, 0), MaterialId(1)));
        }
        for x in 8..10 {
            voxels.push((UVec3::new(x, 0, 0), MaterialId(1)));
        }
        let tree = Contree::from_voxels(16, &voxels);
        let pieces = components(&tree);
        assert_eq!(pieces.iter().map(Vec::len).collect::<Vec<_>>(), vec![5, 2]);
        assert!(components(&Contree::empty(2)).is_empty());
    }

    /// Diagonals do not join a body's pieces either.
    #[test]
    fn components_do_not_join_diagonals() {
        let tree = Contree::from_voxels(
            4,
            &[(UVec3::new(0, 0, 0), MaterialId(1)), (UVec3::new(1, 1, 1), MaterialId(1))],
        );
        assert_eq!(components(&tree).len(), 2);
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
        let time = |walk: fn(&Contree, IVec3, IVec3, usize) -> Vec<Vec<UVec3>>,
                    tree: &Contree,
                    lo: IVec3,
                    hi: IVec3,
                    expect: usize| {
            median(
                (0..9)
                    .map(|_| {
                        let start = std::time::Instant::now();
                        let pieces = walk(tree, lo, hi, BUDGET);
                        let ms = start.elapsed().as_secs_f64() * 1000.0;
                        assert_eq!(pieces.len(), expect);
                        ms
                    })
                    .collect(),
            )
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
        // The same two scenes, cell walk against the voxel walk it replaced,
        // interleaved in this one run.
        let column = column();
        let (cut_lo, cut_hi) = (IVec3::new(27, 0, 27), IVec3::new(36, 2, 36));
        let (hole_lo, hole_hi) = (IVec3::new(26, 25, 26), IVec3::new(38, 32, 38));
        let mut rows = Vec::new();
        for _ in 0..3 {
            rows.push((
                time(loose_pieces, &column, cut_lo, cut_hi, 1),
                time(loose_pieces_by_voxel, &column, cut_lo, cut_hi, 1),
                time(loose_pieces, &ground, hole_lo, hole_hi, 0),
                time(loose_pieces_by_voxel, &ground, hole_lo, hole_hi, 0),
            ));
        }
        println!("detach, median ms: 2560-voxel column freed {freed:.3}, hole in ground {grounded:.3}");
        for (cells_column, voxels_column, cells_hole, voxels_hole) in rows {
            println!(
                "  A/B column: cells {cells_column:.3} vs voxels {voxels_column:.3} | \
                 hole: cells {cells_hole:.3} vs voxels {voxels_hole:.3}"
            );
        }
    }
}
