//! Contacts between a body and the static world.
//!
//! After Dwyer's devlogs #20 and #26. Body corners are tested against world
//! voxels, world corners against body voxels, and edges against edges, all as
//! cubes rounded at their corners and edges. The exact formulas, the reach and
//! the ownership rules are this design's own; see the spec's Provenance section.

use super::classify::{Shape, classify, solid_at};
use crate::body::{Body, BodyId, occupied_bounds};
use crate::contree::Contree;
use crate::distance_field::{CELL_VOXELS, DistanceField};
use crate::material::{MaterialTable, combine_friction, combine_restitution};
use glam::{IVec3, UVec3, Vec3};
use std::collections::HashMap;

/// The rounding radius of a voxel's corners and edges.
pub const RADIUS: f32 = 0.5;

const AXES: [Vec3; 3] = [Vec3::X, Vec3::Y, Vec3::Z];

/// Which voxel of which body touched which voxel of what.
///
/// Stable from tick to tick while the touch persists, which is what warm
/// starting keys on. `other` names the world or the body on the far side, so an
/// impulse cannot be carried over to a different neighbour.
#[derive(Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Debug)]
pub struct ContactKey {
    /// The world, or the other body.
    pub other: BodyId,
    /// The voxel of the body this contact belongs to. Plain coordinates
    /// rather than a `UVec3`, which is not ordered, and the solver sorts its
    /// contacts so that a tick is reproducible.
    pub mine: [u32; 3],
    /// The voxel of `other`. Against the world, its world-space coordinate,
    /// which is never negative where a contact can be.
    pub theirs: [u32; 3],
}

#[derive(Clone, Copy, Debug)]
pub struct Contact {
    pub key: ContactKey,
    /// World space, pointing from the world into the body: the way the body is
    /// pushed.
    pub normal: Vec3,
    /// Signed gap at detection. Negative is penetration; positive is speculative.
    pub separation: f32,
    /// The contact point relative to the centre of mass, in the body's own axes.
    pub anchor: Vec3,
    /// Where that point was in the world at detection.
    pub world_point: Vec3,
    /// The contact point relative to the OTHER body's centre of mass, in its
    /// own axes. Zero against the world, which does not move.
    pub other_anchor: Vec3,
    /// Where that point was in the world at detection. Zero against the world.
    pub other_point: Vec3,
    /// Coulomb friction between the two voxels that touch.
    pub friction: f32,
    /// How much of the approach speed a bounce keeps.
    pub restitution: f32,
}

/// Owned ranges are half-open, so a point on the boundary between two voxels
/// belongs to exactly one of them.
fn owns(x: f32) -> bool {
    (-RADIUS..RADIUS).contains(&x)
}

/// A rounded corner at `q` against the solid voxel `shape` centred at `c`.
///
/// Returns the separation and the normal pointing from that voxel toward `q`.
/// Returns `None` when this voxel does not own the touch: a face owns only the
/// sphere over it, and an edge only the one alongside it, so a sphere within
/// reach of many voxels is reported once.
pub fn sphere_vs_voxel(q: Vec3, c: Vec3, shape: Shape) -> Option<(f32, Vec3)> {
    let w = q - c;
    match shape {
        Shape::Corner => {
            let d = w.length();
            (d > 1e-6).then(|| (d - 2.0 * RADIUS, w / d))
        }
        Shape::Edge(axis) => {
            if !owns(w[axis]) {
                return None;
            }
            let mut across = w;
            across[axis] = 0.0;
            let d = across.length();
            (d > 1e-6).then(|| (d - 2.0 * RADIUS, across / d))
        }
        Shape::Face { axis, open_plus, open_minus } => {
            if !owns(w[(axis + 1) % 3]) || !owns(w[(axis + 2) % 3]) {
                return None;
            }
            let side = if open_plus && (!open_minus || w[axis] >= 0.0) {
                1.0
            } else if open_minus {
                -1.0
            } else {
                return None;
            };
            let height = w[axis] * side;
            if height < 0.0 {
                // Behind the face: the voxel on the open side owns this touch.
                return None;
            }
            let mut n = Vec3::ZERO;
            n[axis] = side;
            Some((height - 2.0 * RADIUS, n))
        }
        Shape::Interior => None,
    }
}

/// A rounded body edge through `p` along unit `d`, against a world edge voxel
/// centred at `c` along unit axis `e`.
///
/// Returns the separation, the normal from the world edge toward the body edge,
/// and the midpoint between them. Returns `None` unless the closest approach
/// lies within both voxels' length; past the end, the neighbouring voxel along
/// the edge owns the touch.
pub fn edge_vs_edge(p: Vec3, d: Vec3, c: Vec3, e: Vec3) -> Option<(f32, Vec3, Vec3)> {
    let r = p - c;
    let b = d.dot(e);
    let denom = 1.0 - b * b;
    let (t, s) = if denom < 1e-4 {
        // Parallel: the body voxel's own centre, projected onto the world edge.
        (0.0, r.dot(e))
    } else {
        let (dd, ee) = (d.dot(r), e.dot(r));
        ((b * ee - dd) / denom, (ee - b * dd) / denom)
    };
    if !owns(t) || !owns(s) {
        return None;
    }
    let (on_body, on_world) = (p + d * t, c + e * s);
    let w = on_body - on_world;
    let dist = w.length();
    (dist > 1e-6).then(|| (dist - 2.0 * RADIUS, w / dist, (on_body + on_world) * 0.5))
}

/// The body's world-space box: its occupied bounds carried through its
/// transform, grown by `margin` and a voxel's rounding.
pub fn world_box(body: &Body, margin: f32) -> Option<(Vec3, Vec3)> {
    let (lo, hi) = occupied_bounds(&body.volume)?;
    let (lo, hi) = (lo.as_vec3(), hi.as_vec3());
    let m = body.world_from_local();
    let mut min = Vec3::splat(f32::INFINITY);
    let mut max = Vec3::splat(f32::NEG_INFINITY);
    for i in 0..8 {
        let corner = Vec3::new(
            if i & 1 == 0 { lo.x } else { hi.x },
            if i & 2 == 0 { lo.y } else { hi.y },
            if i & 4 == 0 { lo.z } else { hi.z },
        );
        let p = m.transform_point3(corner);
        min = min.min(p);
        max = max.max(p);
    }
    Some((min - (margin + RADIUS), max + (margin + RADIUS)))
}

/// Whether the distance field promises that no world voxel lies in the box.
///
/// One lookup, at the cell holding the box's centre. A cell `d` cells from the
/// nearest solid cell promises that every cell within `d - 1` of it is empty.
pub fn field_clears(field: &DistanceField, min: Vec3, max: Vec3) -> bool {
    let size = CELL_VOXELS as f32;
    let cell = ((min + max) * 0.5 / size).floor();
    if cell.cmplt(Vec3::ZERO).any() {
        return false;
    }
    let d = field.get(cell.as_uvec3()) as f32;
    if d == 0.0 {
        return false;
    }
    let free_min = (cell - (d - 1.0)) * size;
    let free_max = (cell + d) * size;
    min.cmpge(free_min).all() && max.cmple(free_max).all()
}

/// Every voxel within `reach` of the one containing `q`.
fn around(q: Vec3, reach: i32) -> impl Iterator<Item = IVec3> {
    let centre = q.floor().as_ivec3();
    (-reach..=reach).flat_map(move |z| {
        (-reach..=reach)
            .flat_map(move |y| (-reach..=reach).map(move |x| centre + IVec3::new(x, y, z)))
    })
}

/// Every world voxel in the box, clipped to the world.
fn box_voxels(min: Vec3, max: Vec3, extent: u32) -> impl Iterator<Item = IVec3> {
    let lo = min.floor().as_ivec3().max(IVec3::ZERO);
    let hi = max.ceil().as_ivec3().min(IVec3::splat(extent as i32));
    (lo.z..hi.z).flat_map(move |z| {
        (lo.y..hi.y).flat_map(move |y| (lo.x..hi.x).map(move |x| IVec3::new(x, y, z)))
    })
}

/// Records a contact unless it is past the margin. When two loops find the same
/// voxel pair, the first stands.
#[allow(clippy::too_many_arguments)]
fn keep(
    found: &mut HashMap<ContactKey, Contact>,
    body: &Body,
    margin: f32,
    key: ContactKey,
    normal: Vec3,
    separation: f32,
    world_point: Vec3,
    coefficients: (f32, f32),
) {
    if separation > margin {
        return;
    }
    let anchor = body.local_from_world().transform_point3(world_point) - body.com;
    let (friction, restitution) = coefficients;
    found.entry(key).or_insert(Contact {
        key,
        normal,
        separation,
        anchor,
        world_point,
        // The world does not move, so it has no anchor to track.
        other_anchor: Vec3::ZERO,
        other_point: Vec3::ZERO,
        friction,
        restitution,
    });
}

/// The coefficients for a touch between a body voxel and a world voxel: each
/// voxel's own material, combined. A body half on ice drags on one side only.
///
/// `k` is always inside the world here: every caller has tested it already.
fn coefficients(
    materials: &MaterialTable,
    body: &Body,
    u: UVec3,
    tree: &Contree,
    k: IVec3,
) -> (f32, f32) {
    let a = materials.get(body.volume.get(u));
    let b = materials.get(tree.get(k.as_uvec3()));
    (combine_friction(a.friction, b.friction), combine_restitution(a.restitution, b.restitution))
}

/// Every contact between `body` and the static world with a separation of at
/// most `margin`, sorted by key so the solver is deterministic.
///
/// The reach grows with the margin. A contact at `margin` needs voxels up to
/// `ceil(margin + 0.5)` away, and a floor exactly one voxel below a resting
/// corner sits on a 2x2x2 lookup's tie boundary.
pub fn detect(
    body: &Body,
    tree: &Contree,
    field: &DistanceField,
    materials: &MaterialTable,
    margin: f32,
) -> Vec<Contact> {
    let Some((min, max)) = world_box(body, margin) else {
        return Vec::new();
    };
    if field_clears(field, min, max) {
        return Vec::new();
    }
    let world_from_local = body.world_from_local();
    let local_from_world = body.local_from_world();
    let reach = (margin + RADIUS).ceil() as i32;
    let world_solid = |p: IVec3| solid_at(tree, p);
    let body_solid = |p: IVec3| solid_at(&body.volume, p);
    let mut found = HashMap::new();

    // A body corner against every world voxel within reach.
    for &u in &body.features.corners {
        let q = world_from_local.transform_point3(u.as_vec3() + 0.5);
        for k in around(q, reach) {
            if !world_solid(k) {
                continue;
            }
            if let Some((sep, n)) = sphere_vs_voxel(q, k.as_vec3() + 0.5, classify(world_solid, k))
            {
                let point = q - n * (RADIUS + sep * 0.5);
                let pair = coefficients(materials, body, u, tree, k);
                keep(
                    &mut found,
                    body,
                    margin,
                    ContactKey { other: BodyId::WORLD, mine: u.to_array(), theirs: k.as_uvec3().to_array() },
                    n,
                    sep,
                    point,
                    pair,
                );
            }
        }
    }

    // A world corner in the box against every body voxel within reach. This runs
    // in the body's frame, where its voxels are axis-aligned.
    for k in box_voxels(min, max, tree.extent()) {
        if !world_solid(k) || classify(world_solid, k) != Shape::Corner {
            continue;
        }
        let q = local_from_world.transform_point3(k.as_vec3() + 0.5);
        for u in around(q, reach) {
            if !body_solid(u) {
                continue;
            }
            if let Some((sep, n_local)) =
                sphere_vs_voxel(q, u.as_vec3() + 0.5, classify(body_solid, u))
            {
                // `n_local` points from the body voxel toward the world corner;
                // the body is pushed the other way.
                let point = world_from_local
                    .transform_point3(q - n_local * (RADIUS + sep * 0.5));
                let pair = coefficients(materials, body, u.as_uvec3(), tree, k);
                keep(
                    &mut found,
                    body,
                    margin,
                    ContactKey { other: BodyId::WORLD, mine: u.as_uvec3().to_array(), theirs: k.as_uvec3().to_array() },
                    -(body.orientation * n_local),
                    sep,
                    point,
                    pair,
                );
            }
        }
    }

    // A body edge against every world edge within reach.
    for &(u, axis) in &body.features.edges {
        let p = world_from_local.transform_point3(u.as_vec3() + 0.5);
        let d = body.orientation * AXES[axis];
        for k in around(p, reach) {
            if !world_solid(k) {
                continue;
            }
            let Shape::Edge(world_axis) = classify(world_solid, k) else { continue };
            if let Some((sep, n, point)) =
                edge_vs_edge(p, d, k.as_vec3() + 0.5, AXES[world_axis])
            {
                let pair = coefficients(materials, body, u, tree, k);
                keep(
                    &mut found,
                    body,
                    margin,
                    ContactKey { other: BodyId::WORLD, mine: u.to_array(), theirs: k.as_uvec3().to_array() },
                    n,
                    sep,
                    point,
                    pair,
                );
            }
        }
    }

    let mut contacts: Vec<Contact> = found.into_values().collect();
    contacts.sort_unstable_by_key(|c| c.key);
    contacts
}

/// Whether two world-space boxes overlap.
pub fn boxes_overlap(a: (Vec3, Vec3), b: (Vec3, Vec3)) -> bool {
    a.0.cmple(b.1).all() && b.0.cmple(a.1).all()
}

/// Every contact between two bodies within `margin`, belonging to `a`.
///
/// Runs in `b`'s frame, where `b`'s voxels are axis-aligned, exactly as
/// `detect` runs in the world's. Normals point from `b` into `a`.
pub fn detect_pair(a: &Body, b: &Body, materials: &MaterialTable, margin: f32) -> Vec<Contact> {
    let (Some(box_a), Some(box_b)) = (world_box(a, margin), world_box(b, margin)) else {
        return Vec::new();
    };
    if !boxes_overlap(box_a, box_b) {
        return Vec::new();
    }
    let b_from_a = b.local_from_world() * a.world_from_local();
    let a_from_b = a.local_from_world() * b.world_from_local();
    let reach = (margin + RADIUS).ceil() as i32;
    let a_solid = |p: IVec3| solid_at(&a.volume, p);
    let b_solid = |p: IVec3| solid_at(&b.volume, p);
    let mut found = HashMap::new();

    // A's corners against B's voxels.
    for &u in &a.features.corners {
        let q = b_from_a.transform_point3(u.as_vec3() + 0.5);
        for k in around(q, reach) {
            if !b_solid(k) {
                continue;
            }
            if let Some((sep, n_b)) = sphere_vs_voxel(q, k.as_vec3() + 0.5, classify(b_solid, k)) {
                let point_b = q - n_b * (RADIUS + sep * 0.5);
                keep_pair(&mut found, a, b, materials, margin, u, k.as_uvec3(), n_b, sep, point_b);
            }
        }
    }

    // B's corners against A's voxels. The pair test answers in A's frame with
    // the normal pointing from A into B, so it is reversed and carried into B's.
    for &w in &b.features.corners {
        let q = a_from_b.transform_point3(w.as_vec3() + 0.5);
        for k in around(q, reach) {
            if !a_solid(k) {
                continue;
            }
            if let Some((sep, n_a)) = sphere_vs_voxel(q, k.as_vec3() + 0.5, classify(a_solid, k)) {
                let point_a = q - n_a * (RADIUS + sep * 0.5);
                let point_b = b_from_a.transform_point3(point_a);
                let n_b = b_from_a.transform_vector3(-n_a);
                keep_pair(&mut found, a, b, materials, margin, k.as_uvec3(), w, n_b, sep, point_b);
            }
        }
    }

    // A's edges against B's edges.
    for &(u, axis) in &a.features.edges {
        let p = b_from_a.transform_point3(u.as_vec3() + 0.5);
        let d = b_from_a.transform_vector3(AXES[axis]);
        for k in around(p, reach) {
            if !b_solid(k) {
                continue;
            }
            let Shape::Edge(b_axis) = classify(b_solid, k) else { continue };
            if let Some((sep, n_b, point_b)) = edge_vs_edge(p, d, k.as_vec3() + 0.5, AXES[b_axis]) {
                keep_pair(&mut found, a, b, materials, margin, u, k.as_uvec3(), n_b, sep, point_b);
            }
        }
    }

    let mut contacts: Vec<Contact> = found.into_values().collect();
    contacts.sort_unstable_by_key(|c| c.key);
    contacts
}

/// Records a contact between two bodies. `normal` and `point` arrive in B's
/// frame, where the pair tests ran, and are carried into the world here.
#[allow(clippy::too_many_arguments)]
fn keep_pair(
    found: &mut HashMap<ContactKey, Contact>,
    a: &Body,
    b: &Body,
    materials: &MaterialTable,
    margin: f32,
    mine: UVec3,
    theirs: UVec3,
    normal: Vec3,
    separation: f32,
    point: Vec3,
) {
    if separation > margin {
        return;
    }
    let key = ContactKey { other: b.id, mine: mine.to_array(), theirs: theirs.to_array() };
    let world_point = b.world_from_local().transform_point3(point);
    let ma = materials.get(a.volume.get(mine));
    let mb = materials.get(b.volume.get(theirs));
    found.entry(key).or_insert(Contact {
        key,
        normal: b.orientation * normal,
        separation,
        anchor: a.local_from_world().transform_point3(world_point) - a.com,
        world_point,
        other_anchor: point - b.com,
        other_point: world_point,
        friction: combine_friction(ma.friction, mb.friction),
        restitution: combine_restitution(ma.restitution, mb.restitution),
    });
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::material::MaterialId;
    use crate::physics::fixtures::{cube, cube_of, materials, placed, slab};
    use glam::Quat;

    const TOLERANCE: f32 = 1e-4;

    #[test]
    fn two_corner_spheres_are_a_sphere_pair() {
        let (sep, n) = sphere_vs_voxel(Vec3::new(1.2, 0.0, 0.0), Vec3::ZERO, Shape::Corner).unwrap();
        assert!((sep - 0.2).abs() < TOLERANCE, "separation {sep}");
        assert!((n - Vec3::X).length() < TOLERANCE, "normal {n:?}");
    }

    /// An edge owns only the sphere alongside it, not one past its end: the next
    /// voxel along the edge owns that one.
    #[test]
    fn an_edge_owns_only_the_sphere_alongside_it() {
        let (sep, n) =
            sphere_vs_voxel(Vec3::new(0.3, 0.9, 0.0), Vec3::ZERO, Shape::Edge(0)).unwrap();
        assert!((sep + 0.1).abs() < TOLERANCE, "separation {sep}");
        assert!((n - Vec3::Y).length() < TOLERANCE, "normal {n:?}");
        assert!(sphere_vs_voxel(Vec3::new(0.6, 0.9, 0.0), Vec3::ZERO, Shape::Edge(0)).is_none());
    }

    /// A face owns only the sphere over it, and pushes out of its open side.
    #[test]
    fn a_face_owns_only_the_sphere_over_it() {
        let top = Shape::Face { axis: 1, open_plus: true, open_minus: false };
        let (sep, n) = sphere_vs_voxel(Vec3::new(0.2, 1.0, -0.4), Vec3::ZERO, top).unwrap();
        assert!(sep.abs() < TOLERANCE, "separation {sep}");
        assert_eq!(n, Vec3::Y);
        assert!(
            sphere_vs_voxel(Vec3::new(0.5, 1.0, 0.0), Vec3::ZERO, top).is_none(),
            "the next voxel owns x = 0.5"
        );
        assert!(
            sphere_vs_voxel(Vec3::new(0.0, -1.0, 0.0), Vec3::ZERO, top).is_none(),
            "the closed side pushes nothing"
        );
        assert!(sphere_vs_voxel(Vec3::new(0.0, 1.0, 0.0), Vec3::ZERO, Shape::Interior).is_none());
    }

    #[test]
    fn crossed_edges_meet_where_their_lines_do() {
        let (sep, n, point) =
            edge_vs_edge(Vec3::new(0.0, 1.1, 0.0), Vec3::X, Vec3::ZERO, Vec3::Z).unwrap();
        assert!((sep - 0.1).abs() < TOLERANCE, "separation {sep}");
        assert!((n - Vec3::Y).length() < TOLERANCE, "normal {n:?}");
        assert!((point - Vec3::new(0.0, 0.55, 0.0)).length() < TOLERANCE, "point {point:?}");
        assert!(
            edge_vs_edge(Vec3::new(0.0, 1.1, 0.7), Vec3::X, Vec3::ZERO, Vec3::Z).is_none(),
            "past the world edge's end"
        );
        let (sep, n, _) =
            edge_vs_edge(Vec3::new(0.2, 1.0, 0.0), Vec3::X, Vec3::ZERO, Vec3::X).unwrap();
        assert!(sep.abs() < TOLERANCE && (n - Vec3::Y).length() < TOLERANCE, "parallel: {sep} {n:?}");
    }

    /// Dwyer's devlog #20: a box sitting on the ground touches it only at its
    /// corners. Turning the box about the vertical must not change that.
    #[test]
    fn a_cube_resting_on_the_floor_touches_it_at_four_corners() {
        let world = slab(64, 0..8);
        let field = DistanceField::build(&world);
        for angle in [0.0f32, std::f32::consts::FRAC_PI_6] {
            let body = placed(cube(4, 4), Vec3::new(32.0, 10.0, 32.0), Quat::from_rotation_y(angle));
            let contacts = detect(&body, &world, &field, &materials(), 0.1);
            assert_eq!(contacts.len(), 4, "at {angle} rad: {contacts:#?}");
            for c in &contacts {
                assert!(c.separation.abs() < TOLERANCE, "at {angle} rad, separation {}", c.separation);
                assert!((c.normal - Vec3::Y).length() < TOLERANCE, "at {angle} rad, normal {:?}", c.normal);
                assert!(c.key.mine[1] == 0, "a contact came from voxel {:?}, not the bottom", c.key.mine);
            }
        }
    }

    /// A body face resting on a lone world pillar: no corner of the body is
    /// there, so only the world-corner loop can find it.
    #[test]
    fn a_world_corner_meets_a_body_face() {
        let mut voxels = Vec::new();
        for y in 0..8 {
            voxels.push((UVec3::new(32, y, 32), MaterialId(1)));
        }
        let world = Contree::from_voxels(64, &voxels);
        let field = DistanceField::build(&world);
        // A 4-cube centred over the pillar: the pillar's top meets the middle of
        // its bottom face.
        let body = placed(cube(4, 4), Vec3::new(32.5, 10.0, 32.5), Quat::IDENTITY);
        let contacts = detect(&body, &world, &field, &materials(), 0.1);
        assert!(!contacts.is_empty(), "the pillar was not found");
        for c in &contacts {
            assert_eq!(c.key.theirs, [32, 7, 32], "contact with {:?}", c.key.theirs);
            assert!((c.normal - Vec3::Y).length() < TOLERANCE, "normal {:?}", c.normal);
            assert!(c.separation.abs() < TOLERANCE, "separation {}", c.separation);
        }
    }

    /// A bar lying across a bar touches it mid-span, where neither has a corner.
    #[test]
    fn crossed_bars_touch_through_their_edges() {
        let mut voxels = Vec::new();
        for z in 20..44 {
            voxels.push((UVec3::new(32, 8, z), MaterialId(1)));
        }
        let world = Contree::from_voxels(64, &voxels);
        let field = DistanceField::build(&world);
        let bar: Vec<_> = (0..16).map(|x| (UVec3::new(x, 0, 0), MaterialId(1))).collect();
        let body = placed(Contree::from_voxels(16, &bar), Vec3::new(32.5, 9.5, 32.5), Quat::IDENTITY);
        let contacts = detect(&body, &world, &field, &materials(), 0.1);
        assert!(
            contacts.iter().any(|c| (c.key.mine, c.key.theirs) == ([8, 0, 0], [32, 8, 32])
                && c.separation.abs() < TOLERANCE),
            "no edge contact at the crossing: {contacts:#?}"
        );
    }

    /// The field clears a box only inside the cube its cell promises.
    #[test]
    fn the_field_clears_only_what_it_promises() {
        let world = slab(64, 0..8);
        let field = DistanceField::build(&world);
        // Cell y = 2 (voxels 32..48) is two cells from the floor's cell row:
        // free from voxel 16 up.
        assert!(field_clears(&field, Vec3::new(20.0, 38.0, 20.0), Vec3::new(28.0, 46.0, 28.0)));
        // Centred in cell y = 1 (16..32), one cell from the floor, so it
        // promises only its own cell. This box reaches below 16.
        assert!(!field_clears(&field, Vec3::new(20.0, 14.0, 20.0), Vec3::new(28.0, 20.0, 28.0)));
        assert!(!field_clears(&field, Vec3::new(20.0, 4.0, 20.0), Vec3::new(28.0, 10.0, 28.0)));
    }

    /// The coefficients come from the two voxels that touch, not from the
    /// body: a cube resting half on ice and half on stone drags on one side.
    #[test]
    fn a_contact_takes_the_coefficients_of_both_voxels() {
        let materials = materials();
        // Floor of stone (1), with the far half ice (3).
        let mut voxels = Vec::new();
        for z in 0..64 {
            for y in 0..8 {
                for x in 0..64 {
                    let m = if x >= 32 { MaterialId(3) } else { MaterialId(1) };
                    voxels.push((UVec3::new(x, y, z), m));
                }
            }
        }
        let world = Contree::from_voxels(64, &voxels);
        let field = DistanceField::build(&world);
        // A bouncy cube (4) straddling the seam.
        let body = placed(cube_of(4, 4, MaterialId(4)), Vec3::new(32.0, 10.0, 32.0), Quat::IDENTITY);
        let contacts = detect(&body, &world, &field, &materials, 0.1);
        assert_eq!(contacts.len(), 4);

        let on_ice: Vec<_> = contacts.iter().filter(|c| c.key.theirs[0] >= 32).collect();
        let on_stone: Vec<_> = contacts.iter().filter(|c| c.key.theirs[0] < 32).collect();
        assert_eq!(on_ice.len(), 2, "the cube did not straddle the seam");
        assert_eq!(on_stone.len(), 2);
        for c in on_ice {
            assert_eq!(c.friction, 0.0, "ice is frictionless");
            assert!((c.restitution - 0.8).abs() < 1e-6, "the bouncy cube bounces on ice");
        }
        for c in on_stone {
            assert!((c.friction - 0.6).abs() < 1e-6, "stone against a gripping cube");
            assert!((c.restitution - 0.8).abs() < 1e-6);
        }
    }

    /// Two cubes stacked exactly touch across the upper one's bottom face: at
    /// its four corners, and along the rims, where both cubes' edge voxels meet.
    ///
    /// More contacts than a cube on a floor, which touches at four corners only.
    /// A floor's top is all face voxels, which corners alone can reach; a small
    /// cube's top is rim, and rim meets rim.
    #[test]
    fn a_cube_resting_on_a_cube_touches_across_its_bottom_face() {
        let materials = materials();
        let lower = placed(cube(4, 4), Vec3::new(32.0, 10.0, 32.0), Quat::IDENTITY);
        let upper = placed(cube(4, 4), Vec3::new(32.0, 14.0, 32.0), Quat::IDENTITY);
        let contacts = detect_pair(&upper, &lower, &materials, 0.1);
        for c in &contacts {
            assert_eq!(c.key.other, lower.id);
            assert!((c.normal - Vec3::Y).length() < TOLERANCE, "normal {:?}", c.normal);
            assert!(c.separation.abs() < TOLERANCE, "separation {}", c.separation);
            assert_eq!(c.key.mine[1], 0, "the upper body touched with voxel {:?}", c.key.mine);
            assert_eq!(c.key.theirs[1], 3, "the lower body touched with voxel {:?}", c.key.theirs);
        }
        for corner in [[0, 0, 0], [3, 0, 0], [0, 0, 3], [3, 0, 3]] {
            assert!(
                contacts.iter().any(|c| c.key.mine == corner),
                "no contact at the upper cube's corner {corner:?}: {contacts:#?}"
            );
        }
        // Four corners and the rims between them, each reported once.
        assert_eq!(contacts.len(), 12, "{contacts:#?}");
    }

    /// Turning the pair as a whole turns the contact with it: the normal is the
    /// same in the bodies' shared frame, whatever the world's axes are.
    #[test]
    fn a_turned_pair_touches_the_same_way() {
        let materials = materials();
        let turn = Quat::from_rotation_z(0.9);
        let centre = Vec3::new(32.0, 20.0, 32.0);
        let lower = placed(cube(4, 4), centre, turn);
        let upper = placed(cube(4, 4), centre + turn * Vec3::new(0.0, 4.0, 0.0), turn);
        let contacts = detect_pair(&upper, &lower, &materials, 0.1);
        // The same twelve contacts as the flat pair, named by the same voxels.
        assert_eq!(contacts.len(), 12, "{contacts:#?}");
        for corner in [[0, 0, 0], [3, 0, 0], [0, 0, 3], [3, 0, 3]] {
            assert!(
                contacts.iter().any(|c| c.key.mine == corner),
                "no contact at the upper cube's corner {corner:?}"
            );
        }
        for c in &contacts {
            assert!((c.normal - turn * Vec3::Y).length() < 1e-3, "normal {:?}", c.normal);
            assert!(c.separation.abs() < 1e-3, "separation {}", c.separation);
        }
    }

    /// A narrow pillar of a body standing on a wide one touches where the
    /// pillar's bottom corner meets the flat top: the other body has no corner
    /// there, so only this body's corners can find it. The mirror of the test
    /// below, and between them they pin both corner loops: each alone would
    /// still find every corner-to-corner touch, because both loops see those.
    #[test]
    fn a_corner_of_this_body_meets_a_face() {
        let materials = materials();
        let pillar_voxels: Vec<_> =
            (0..4).map(|y| (UVec3::new(0, y, 0), MaterialId(1))).collect();
        let pillar = placed(
            Contree::from_voxels(4, &pillar_voxels),
            Vec3::new(32.0, 14.0, 32.0),
            Quat::IDENTITY,
        );
        let base = placed(cube(4, 4), Vec3::new(32.0, 10.0, 32.0), Quat::IDENTITY);
        let contacts = detect_pair(&pillar, &base, &materials, 0.1);
        assert_eq!(contacts.len(), 1, "{contacts:#?}");
        let c = contacts[0];
        assert_eq!(c.key.mine, [0, 0, 0], "the pillar touched with {:?}", c.key.mine);
        assert!(
            c.key.theirs[1] == 3 && c.key.theirs != [0, 3, 0],
            "the base touched with a corner, so this proves nothing: {:?}",
            c.key.theirs
        );
        assert!((c.normal - Vec3::Y).length() < TOLERANCE, "normal {:?}", c.normal);
        assert!(c.separation.abs() < TOLERANCE, "separation {}", c.separation);
    }

    /// A body resting on a narrow pillar of a body touches it where the pillar's
    /// top corner meets the flat underside: there is no corner of the upper body
    /// at that point, so only the other body's corners can find it. The normal
    /// still has to point up into the upper body.
    #[test]
    fn a_corner_of_the_other_body_meets_a_face() {
        let materials = materials();
        let pillar_voxels: Vec<_> =
            (0..4).map(|y| (UVec3::new(0, y, 0), MaterialId(1))).collect();
        let pillar = placed(
            Contree::from_voxels(4, &pillar_voxels),
            Vec3::new(32.0, 10.0, 32.0),
            Quat::IDENTITY,
        );
        // Centred over the pillar, so its top meets the middle of the underside.
        let top = placed(cube(4, 4), Vec3::new(32.0, 14.0, 32.0), Quat::IDENTITY);
        let contacts = detect_pair(&top, &pillar, &materials, 0.1);
        assert_eq!(contacts.len(), 1, "{contacts:#?}");
        let c = contacts[0];
        assert_eq!(c.key.theirs, [0, 3, 0], "the pillar touched with {:?}", c.key.theirs);
        assert!(
            c.key.mine[1] == 0 && c.key.mine != [0, 0, 0],
            "the upper body touched with a corner, so this proves nothing: {:?}",
            c.key.mine
        );
        assert!((c.normal - Vec3::Y).length() < TOLERANCE, "normal {:?}", c.normal);
        assert!(c.separation.abs() < TOLERANCE, "separation {}", c.separation);
    }

    /// Bodies that do not overlap are not compared voxel by voxel.
    #[test]
    fn far_apart_bodies_have_no_contacts_and_no_overlap() {
        let materials = materials();
        let a = placed(cube(4, 4), Vec3::new(32.0, 10.0, 32.0), Quat::IDENTITY);
        let b = placed(cube(4, 4), Vec3::new(32.0, 40.0, 32.0), Quat::IDENTITY);
        assert!(!boxes_overlap(world_box(&a, 0.1).unwrap(), world_box(&b, 0.1).unwrap()));
        assert!(detect_pair(&a, &b, &materials, 0.1).is_empty());
    }

    /// A pair's coefficients come from the two voxels, exactly as against the
    /// world: an icy body on a gripping one is slippery.
    #[test]
    fn a_pair_takes_the_coefficients_of_both_voxels() {
        let materials = materials();
        let lower = placed(cube(4, 4), Vec3::new(32.0, 10.0, 32.0), Quat::IDENTITY);
        let icy = placed(cube_of(4, 4, MaterialId(3)), Vec3::new(32.0, 14.0, 32.0), Quat::IDENTITY);
        let contacts = detect_pair(&icy, &lower, &materials, 0.1);
        assert!(!contacts.is_empty());
        for c in contacts {
            assert_eq!(c.friction, 0.0, "ice on stone should slide");
        }
    }

    #[test]
    fn a_body_in_open_air_has_no_contacts() {
        let world = slab(64, 0..8);
        let field = DistanceField::build(&world);
        let body = placed(cube(4, 4), Vec3::new(32.0, 40.0, 32.0), Quat::IDENTITY);
        assert!(detect(&body, &world, &field, &materials(), 1.35).is_empty());
    }
}
