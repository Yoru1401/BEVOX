//! The brush on a body: painting grows it, erasing shrinks it, and an erase
//! that cuts it in two leaves two bodies.
//!
//! Each piece of a split carries on as that part of the body was moving: its
//! centre of mass keeps the velocity it had as a point of the parent, and it
//! keeps the parent's spin. Summed over the pieces, that is the parent's
//! momentum, which is what makes a split look like the body came apart rather
//! than like new bodies appeared.

use super::detach::components;
use crate::body::Body;
use crate::contree::Contree;
use crate::material::{MaterialId, MaterialTable};
use glam::{UVec3, Vec3};

/// Paints or erases a sphere, given in world space, on `bodies[index]`.
///
/// An erase that cuts the body into pieces splits it, unless that would take
/// the scene past `max_bodies`, in which case the pieces stay together: a body
/// past the cap is not drawn, and a rigid body can hold disconnected voxels. A
/// body erased to nothing is removed.
pub fn sculpt(
    bodies: &mut Vec<Body>,
    index: usize,
    centre: Vec3,
    radius: f32,
    material: MaterialId,
    materials: &MaterialTable,
    max_bodies: usize,
) {
    let body = &mut bodies[index];
    if !material.is_empty() {
        paint(body, centre, radius, material, materials);
        return;
    }

    // The rigid motion before the cut, which every piece inherits.
    let (com, velocity, spin) = (body.position, body.velocity, body.angular_velocity());
    let local = body.local_from_world().transform_point3(centre);
    body.volume.apply_sphere(local, radius, MaterialId::EMPTY);

    let pieces = components(&body.volume);
    if pieces.is_empty() {
        bodies.remove(index);
        return;
    }
    let would_have = bodies.len() - 1 + pieces.len();
    if pieces.len() == 1 || would_have > max_bodies {
        let body = &mut bodies[index];
        body.recompute(materials);
        carry_motion(body, com, velocity, spin);
        return;
    }

    // The largest piece stays in the parent, keeping its identity; every other
    // piece leaves as a new body, placed exactly where its voxels were.
    let parent = &bodies[index];
    let corner = parent.position - parent.orientation * parent.com;
    let (orientation, extent) = (parent.orientation, parent.volume.extent());
    let mut split_off = Vec::new();
    for piece in &pieces[1..] {
        let voxels: Vec<_> = piece.iter().map(|&p| (p, parent.volume.get(p))).collect();
        let mut body = Body::new(Contree::from_voxels(extent, &voxels), corner, orientation);
        // A piece of terrain cut in two is still terrain, and a piece of a
        // spawned body is not.
        body.from_terrain = parent.from_terrain;
        if body.recompute(materials) {
            carry_motion(&mut body, com, velocity, spin);
            split_off.push(body);
        }
    }

    let body = &mut bodies[index];
    let others: Vec<UVec3> = pieces[1..].iter().flatten().copied().collect();
    body.volume.clear_voxels(&others);
    body.recompute(materials);
    carry_motion(body, com, velocity, spin);
    bodies.extend(split_off);
}

/// Paints a sphere onto a body, growing its volume first if the paint would
/// reach past the edge, where it would otherwise be dropped.
fn paint(body: &mut Body, centre: Vec3, radius: f32, material: MaterialId, materials: &MaterialTable) {
    let local = body.local_from_world().transform_point3(centre);
    body.grow_to_fit(
        (local - radius).floor().as_ivec3(),
        (local + radius).ceil().as_ivec3(),
    );
    // Growing moved the grid under the body, so the centre is carried again.
    let local = body.local_from_world().transform_point3(centre);
    body.volume.apply_sphere(local, radius, material);
    body.recompute(materials);
}

/// Gives a piece the motion its centre of mass had as a point of the parent,
/// which moved with `velocity` at `com` and turned at `spin`.
fn carry_motion(body: &mut Body, com: Vec3, velocity: Vec3, spin: Vec3) {
    body.velocity = velocity + spin.cross(body.position - com);
    body.set_angular_velocity(spin);
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::physics::fixtures::{cube, materials, placed};
    use glam::{EulerRot, Quat};

    fn world_voxels(body: &Body) -> Vec<[i32; 3]> {
        let mut out: Vec<[i32; 3]> = body
            .volume
            .voxels()
            .iter()
            .map(|(p, _)| {
                (body.world_from_local().transform_point3(p.as_vec3() + 0.5) - 0.5)
                    .round()
                    .as_ivec3()
                    .to_array()
            })
            .collect();
        out.sort_unstable();
        out
    }

    /// Painting onto a body adds voxels where the sphere is and leaves the old
    /// ones where they were, even past the edge of its volume.
    #[test]
    fn painting_grows_a_body_past_its_volume() {
        let materials = materials();
        let mut bodies = vec![placed(cube(4, 4), Vec3::new(20.0, 20.0, 20.0), Quat::IDENTITY)];
        let before = world_voxels(&bodies[0]);
        let mass = bodies[0].mass.mass;
        // Beyond the +x face: the 4-extent volume cannot hold it as it is.
        sculpt(&mut bodies, 0, Vec3::new(23.5, 20.0, 20.0), 1.5, MaterialId(2), &materials, 16);

        let after = world_voxels(&bodies[0]);
        assert!(after.len() > before.len(), "nothing was painted");
        assert!(bodies[0].mass.mass > mass);
        for v in &before {
            assert!(after.contains(v), "painting moved or lost voxel {v:?}");
        }
        assert!(after.iter().any(|v| v[0] >= 22), "no paint landed past the old volume");
    }

    /// Painting below the volume's corner shifts the grid; nothing may move.
    #[test]
    fn painting_past_the_low_corner_moves_nothing() {
        let materials = materials();
        let turn = Quat::from_euler(EulerRot::XYZ, 0.2, -0.7, 0.4);
        let mut bodies = vec![placed(cube(4, 4), Vec3::new(20.0, 20.0, 20.0), turn)];
        let before = world_voxels(&bodies[0]);
        let low = bodies[0].world_from_local().transform_point3(Vec3::new(-1.0, 1.0, 1.0));
        sculpt(&mut bodies, 0, low, 1.5, MaterialId(2), &materials, 16);
        let after = world_voxels(&bodies[0]);
        assert!(after.len() > before.len(), "nothing was painted");
        for v in &before {
            assert!(after.contains(v), "painting moved or lost voxel {v:?}");
        }
    }

    /// A split piece is terrain if its parent was, and is not if it was not:
    /// what may go back into the world is what came out of it.
    #[test]
    fn a_split_piece_keeps_where_its_parent_came_from() {
        let materials = materials();
        let bar: Vec<_> = (0..12)
            .flat_map(|x| (0..2).map(move |z| (UVec3::new(x, 0, z), MaterialId(1))))
            .collect();
        for from_terrain in [false, true] {
            let mut body =
                placed(Contree::from_voxels(16, &bar), Vec3::new(30.0, 30.0, 30.0), Quat::IDENTITY);
            body.from_terrain = from_terrain;
            let middle = body.world_from_local().transform_point3(Vec3::new(6.0, 0.5, 1.0));
            let mut bodies = vec![body];
            sculpt(&mut bodies, 0, middle, 1.2, MaterialId::EMPTY, &materials, 16);
            assert_eq!(bodies.len(), 2, "the bar did not split");
            for b in &bodies {
                assert_eq!(b.from_terrain, from_terrain, "a split piece forgot where it came from");
            }
        }
    }

    /// Erasing a bar through its middle leaves two bodies, the same voxels in
    /// the same places, and the same momentum, whatever the bar was doing.
    #[test]
    fn erasing_a_bar_in_two_splits_it() {
        let materials = materials();
        let bar: Vec<_> = (0..12)
            .flat_map(|x| (0..2).map(move |z| (UVec3::new(x, 0, z), MaterialId(1))))
            .collect();
        let turn = Quat::from_euler(EulerRot::XYZ, 0.3, 0.8, -0.2);
        let mut body = placed(Contree::from_voxels(16, &bar), Vec3::new(30.0, 30.0, 30.0), turn);
        body.velocity = Vec3::new(4.0, -1.0, 2.0);
        let spin = Vec3::new(0.5, 1.5, -0.8);
        body.set_angular_velocity(spin);
        let id = body.id;
        let (com, velocity) = (body.position, body.velocity);
        let momentum = body.velocity * body.mass.mass;
        let middle = body.world_from_local().transform_point3(Vec3::new(6.0, 0.5, 1.0));
        let before = world_voxels(&body);
        let mut bodies = vec![body];

        sculpt(&mut bodies, 0, middle, 1.2, MaterialId::EMPTY, &materials, 16);

        assert_eq!(bodies.len(), 2, "the bar did not split");
        assert_eq!(bodies[0].id, id, "the piece left behind lost the parent's identity");
        let after: Vec<[i32; 3]> = bodies.iter().flat_map(world_voxels).collect();
        assert!(after.len() < before.len(), "nothing was erased");
        for v in &after {
            assert!(before.contains(v), "a piece gained voxel {v:?}");
        }
        // The cut is symmetric, so what is left has the old centre of mass, and
        // the momentum left is the old momentum's share by mass.
        let total: Vec3 = bodies.iter().map(|b| b.velocity * b.mass.mass).sum();
        let expected = momentum * (after.len() as f32 / before.len() as f32);
        assert!(
            (total - expected).length() < 0.01 * expected.length(),
            "momentum {total:?}, expected {expected:?}"
        );
        for b in &bodies {
            assert!((b.angular_velocity() - spin).length() < 1e-3, "a piece lost the spin");
        }

        // The stronger statement: every voxel of every piece moves exactly as it
        // did as part of the bar. The total above cannot see a piece missing its
        // share of the spin, because on a symmetric cut those shares cancel.
        for b in &bodies {
            let (v, w) = (b.velocity, b.angular_velocity());
            for (p, _) in b.volume.voxels() {
                let at = b.world_from_local().transform_point3(p.as_vec3() + 0.5);
                let now = v + w.cross(at - b.position);
                let then = velocity + spin.cross(at - com);
                assert!((now - then).length() < 1e-3, "a voxel changed speed: {now:?} against {then:?}");
            }
        }
    }

    /// With no room for another body, the pieces stay together.
    #[test]
    fn a_split_with_no_room_keeps_the_pieces_together() {
        let materials = materials();
        let bar: Vec<_> = (0..12).map(|x| (UVec3::new(x, 0, 0), MaterialId(1))).collect();
        let mut bodies = vec![placed(
            Contree::from_voxels(16, &bar),
            Vec3::new(30.0, 30.0, 30.0),
            Quat::IDENTITY,
        )];
        let middle = bodies[0].world_from_local().transform_point3(Vec3::new(6.0, 0.5, 0.5));
        sculpt(&mut bodies, 0, middle, 1.2, MaterialId::EMPTY, &materials, 1);
        assert_eq!(bodies.len(), 1, "the split went past the cap");
        assert_eq!(components(&bodies[0].volume).len(), 2, "the erase did not cut the bar");
    }

    #[test]
    fn a_body_erased_to_nothing_is_removed() {
        let materials = materials();
        let mut bodies = vec![placed(cube(4, 4), Vec3::new(20.0, 20.0, 20.0), Quat::IDENTITY)];
        sculpt(&mut bodies, 0, Vec3::new(20.0, 20.0, 20.0), 10.0, MaterialId::EMPTY, &materials, 16);
        assert!(bodies.is_empty());
    }
}
