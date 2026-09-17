//! Contacts between a body and the static world.
//!
//! After Dwyer's devlogs #20 and #26. Body corners are tested against world
//! voxels, world corners against body voxels, and edges against edges, all as
//! cubes rounded at their corners and edges. The exact formulas, the reach and
//! the ownership rules are this design's own; see the spec's Provenance section.

use super::classify::{Shape, classify, solid_at};
use crate::body::{Body, occupied_bounds};
use crate::contree::Contree;
use crate::distance_field::{CELL_VOXELS, DistanceField};
use glam::{IVec3, UVec3, Vec3};
use std::collections::HashMap;

/// The rounding radius of a voxel's corners and edges.
pub const RADIUS: f32 = 0.5;

const AXES: [Vec3; 3] = [Vec3::X, Vec3::Y, Vec3::Z];

/// Which body voxel touched which world voxel. Stable from tick to tick while
/// the touch persists, which is what warm starting keys on.
pub type ContactKey = (UVec3, IVec3);

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
fn keep(
    found: &mut HashMap<ContactKey, Contact>,
    body: &Body,
    margin: f32,
    key: ContactKey,
    normal: Vec3,
    separation: f32,
    world_point: Vec3,
) {
    if separation > margin {
        return;
    }
    let anchor = body.local_from_world().transform_point3(world_point) - body.com;
    found.entry(key).or_insert(Contact { key, normal, separation, anchor, world_point });
}

/// Every contact between `body` and the static world with a separation of at
/// most `margin`, sorted by key so the solver is deterministic.
///
/// The reach grows with the margin. A contact at `margin` needs voxels up to
/// `ceil(margin + 0.5)` away, and a floor exactly one voxel below a resting
/// corner sits on a 2x2x2 lookup's tie boundary.
pub fn detect(body: &Body, tree: &Contree, field: &DistanceField, margin: f32) -> Vec<Contact> {
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
                keep(&mut found, body, margin, (u, k), n, sep, q - n * (RADIUS + sep * 0.5));
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
                keep(
                    &mut found,
                    body,
                    margin,
                    (u.as_uvec3(), k),
                    -(body.orientation * n_local),
                    sep,
                    point,
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
                keep(&mut found, body, margin, (u, k), n, sep, point);
            }
        }
    }

    let mut contacts: Vec<Contact> = found.into_values().collect();
    contacts.sort_by_key(|c| (c.key.0.to_array(), c.key.1.to_array()));
    contacts
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::material::MaterialId;
    use crate::physics::fixtures::{cube, placed, slab};
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
        for angle in [0.0f32, 0.5236] {
            let body = placed(cube(4, 4), Vec3::new(32.0, 10.0, 32.0), Quat::from_rotation_y(angle));
            let contacts = detect(&body, &world, &field, 0.1);
            assert_eq!(contacts.len(), 4, "at {angle} rad: {contacts:#?}");
            for c in &contacts {
                assert!(c.separation.abs() < TOLERANCE, "at {angle} rad, separation {}", c.separation);
                assert!((c.normal - Vec3::Y).length() < TOLERANCE, "at {angle} rad, normal {:?}", c.normal);
                assert!(c.key.0.y == 0, "a contact came from voxel {:?}, not the bottom", c.key.0);
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
        let contacts = detect(&body, &world, &field, 0.1);
        assert!(!contacts.is_empty(), "the pillar was not found");
        for c in &contacts {
            assert_eq!(c.key.1, IVec3::new(32, 7, 32), "contact with {:?}", c.key.1);
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
        let contacts = detect(&body, &world, &field, 0.1);
        assert!(
            contacts.iter().any(|c| c.key == (UVec3::new(8, 0, 0), IVec3::new(32, 8, 32))
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

    #[test]
    fn a_body_in_open_air_has_no_contacts() {
        let world = slab(64, 0..8);
        let field = DistanceField::build(&world);
        let body = placed(cube(4, 4), Vec3::new(32.0, 40.0, 32.0), Quat::IDENTITY);
        assert!(detect(&body, &world, &field, 1.35).is_empty());
    }
}
