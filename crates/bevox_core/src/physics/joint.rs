//! Joints: two bodies, or a body and the world, held together at a pivot.
//!
//! After Dwyer's devlog #30: each joint is a constraint solved as an impulse
//! alongside the contacts. A ball joint keeps two points together; a hinge also
//! keeps two axes aligned, so the bodies turn only about it.

use super::BIAS;
use super::classify::solid_at;
use crate::body::{Body, BodyId};
use glam::{Mat2, Mat3, Vec2, Vec3};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum JointKind {
    /// The two pivot points stay together; rotation is free.
    Ball,
    /// As a ball, and the two bodies turn only about the axis.
    Hinge,
}

/// A joint between body `a` and body `b`, or the world when `b` is `None`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Joint {
    pub kind: JointKind,
    pub a: BodyId,
    pub b: Option<BodyId>,
    /// The pivot in `a`'s volume coordinates.
    pub anchor_a: Vec3,
    /// The pivot in `b`'s volume coordinates, or in the world.
    pub anchor_b: Vec3,
    /// The hinge axis in `a`'s own axes.
    pub axis_a: Vec3,
    /// The hinge axis in `b`'s own axes, or in the world's.
    pub axis_b: Vec3,
    /// Accumulated impulses for warm starting, in world axes. World axes rather
    /// than the rows' own, because a hinge's rows are rebuilt from its axis
    /// every substep and their basis turns with it.
    pub linear: Vec3,
    pub angular: Vec3,
}

impl Joint {
    /// Joins `a` to `b`, or to the world, at the world point `pivot`. Nothing
    /// moves: `b` is fastened wherever it is. `axis` is the hinge axis in world
    /// axes; a ball joint ignores it.
    pub fn new(kind: JointKind, a: &Body, b: Option<&Body>, pivot: Vec3, axis: Vec3) -> Self {
        let axis = axis.normalize();
        let (anchor_b, axis_b) = match b {
            Some(b) => {
                (b.local_from_world().transform_point3(pivot), b.orientation.inverse() * axis)
            }
            None => (pivot, axis),
        };
        Self {
            kind,
            a: a.id,
            b: b.map(|b| b.id),
            anchor_a: a.local_from_world().transform_point3(pivot),
            anchor_b,
            axis_a: a.orientation.inverse() * axis,
            axis_b,
            linear: Vec3::ZERO,
            angular: Vec3::ZERO,
        }
    }

    /// The pivot as each side holds it, in the world.
    pub fn pivots(&self, a: &Body, b: Option<&Body>) -> (Vec3, Vec3) {
        let pa = a.world_from_local().transform_point3(self.anchor_a);
        let pb = match b {
            Some(b) => b.world_from_local().transform_point3(self.anchor_b),
            None => self.anchor_b,
        };
        (pa, pb)
    }
}

/// The cross-product matrix: `skew(r) * x == r.cross(x)`.
fn skew(r: Vec3) -> Mat3 {
    Mat3::from_cols(
        Vec3::new(0.0, r.z, -r.y),
        Vec3::new(-r.z, 0.0, r.x),
        Vec3::new(r.y, -r.x, 0.0),
    )
}

/// How a point `r` from the centre of mass answers an impulse there: its
/// velocity changes by this matrix times the impulse, `1/m - [r] I^-1 [r]`.
fn point_mass(body: &Body, r: Vec3) -> Mat3 {
    let s = skew(r);
    Mat3::from_diagonal(Vec3::splat(body.mass.inverse_mass())) - s * body.world_inverse_inertia() * s
}

/// Applies a linear impulse at the pivot and an angular one, to `a`, and the
/// opposite to `b`.
fn apply(a: &mut Body, b: Option<&mut Body>, ra: Vec3, rb: Vec3, linear: Vec3, angular: Vec3) {
    a.velocity += linear * a.mass.inverse_mass();
    a.angular_momentum += ra.cross(linear) + angular;
    if let Some(b) = b {
        b.velocity -= linear * b.mass.inverse_mass();
        b.angular_momentum -= rb.cross(linear) + angular;
    }
}

/// The pivot's offset from each body's centre of mass. Zero for the world.
fn levers(a: &Body, b: Option<&Body>, pa: Vec3, pb: Vec3) -> (Vec3, Vec3) {
    (pa - a.position, b.map_or(Vec3::ZERO, |b| pb - b.position))
}

/// Re-applies what the joint carried before.
pub(crate) fn warm_start(a: &mut Body, b: Option<&mut Body>, joint: &Joint) {
    let (pa, pb) = joint.pivots(a, b.as_deref());
    let (ra, rb) = levers(a, b.as_deref(), pa, pb);
    apply(a, b, ra, rb, joint.linear, joint.angular);
}

/// One iteration on one joint. With `use_bias`, a fraction of the drift is
/// corrected too, through velocity; the relax pass after integrating leaves it
/// out, so the correction does not become motion.
pub(crate) fn solve(a: &mut Body, mut b: Option<&mut Body>, joint: &mut Joint, inv_h: f32, use_bias: bool) {
    // The pivots must move together: one 3x3 solve, because an impulse along one
    // axis moves the pivot along the others too once the body turns. Solving the
    // three axes as separate rows ignores that coupling. Measured: no current
    // gate can tell the two apart, because the substeps correct what one pass
    // misses; the full solve stays because it is the correct one, at no cost.
    let (pa, pb) = joint.pivots(a, b.as_deref());
    let (ra, rb) = levers(a, b.as_deref(), pa, pb);
    let va = a.point_velocity(ra);
    let vb = b.as_deref().map_or(Vec3::ZERO, |b| b.point_velocity(rb));
    let mut k = point_mass(a, ra);
    if let Some(b) = b.as_deref() {
        k += point_mass(b, rb);
    }
    let bias = if use_bias { (pa - pb) * (BIAS * inv_h) } else { Vec3::ZERO };
    let lambda = -(k.inverse() * (va - vb + bias));
    joint.linear += lambda;
    apply(a, b.as_deref_mut(), ra, rb, lambda, Vec3::ZERO);

    if joint.kind == JointKind::Hinge {
        solve_hinge(a, b, joint, inv_h, use_bias);
    }
}

/// A hinge's two angular rows: the bodies may turn relative to each other only
/// about the axis, and a fraction of any misalignment is turned back.
///
/// The rows are the two directions across the axis, rebuilt from the axis every
/// time, which is why the accumulated impulse is kept in world axes.
fn solve_hinge(a: &mut Body, b: Option<&mut Body>, joint: &mut Joint, inv_h: f32, use_bias: bool) {
    let wa = a.orientation * joint.axis_a;
    let wb = b.as_deref().map_or(joint.axis_b, |b| b.orientation * joint.axis_b);
    let (t1, t2) = wa.any_orthonormal_pair();
    let inverse = a.world_inverse_inertia()
        + b.as_deref().map_or(Mat3::ZERO, |b| b.world_inverse_inertia());
    let spin = a.angular_velocity() - b.as_deref().map_or(Vec3::ZERO, |b| b.angular_velocity());
    let cdot = Vec2::new(spin.dot(t1), spin.dot(t2));
    // Turning `a` about `wa x wb` brings its axis toward `b`'s, so the bias asks
    // for relative spin along it, in proportion to the misalignment.
    let error = wa.cross(wb);
    let bias = if use_bias {
        -Vec2::new(error.dot(t1), error.dot(t2)) * (BIAS * inv_h)
    } else {
        Vec2::ZERO
    };
    let k = Mat2::from_cols(
        Vec2::new(t1.dot(inverse * t1), t2.dot(inverse * t1)),
        Vec2::new(t1.dot(inverse * t2), t2.dot(inverse * t2)),
    );
    let lambda = -(k.inverse() * (cdot + bias));
    let impulse = t1 * lambda.x + t2 * lambda.y;
    joint.angular += impulse;
    apply(a, b, Vec3::ZERO, Vec3::ZERO, Vec3::ZERO, impulse);
}

/// After an edit, moves each joint to whichever piece now holds its pivot, and
/// removes a joint whose pivot, first body or second body is gone.
///
/// The first body's anchor sits inside the voxel that was clicked, so "the
/// piece that holds the pivot" is the one holding that voxel. A split keeps
/// volume coordinates, since every piece is built in the parent's frame, so the
/// voxel has the same coordinates in the piece; checking that the piece also
/// puts it at the same place in the world rules out an unrelated body that
/// happens to have a voxel there. The second body was fastened wherever it was
/// clicked, which need not be inside it, so it stays with whichever piece keeps
/// its identity.
pub fn follow(joints: &mut Vec<Joint>, bodies: &[Body]) {
    let find = |id: BodyId| bodies.iter().find(|b| b.id == id);
    joints.retain_mut(|joint| {
        if let Some(b) = joint.b
            && find(b).is_none()
        {
            return false;
        }
        let Some(a) = find(joint.a) else {
            return false;
        };
        let voxel = joint.anchor_a.floor().as_ivec3();
        if solid_at(&a.volume, voxel) {
            return true;
        }
        let at = a.world_from_local().transform_point3(joint.anchor_a);
        let holder = bodies.iter().find(|piece| {
            piece.id != a.id
                && solid_at(&piece.volume, voxel)
                && (piece.world_from_local().transform_point3(joint.anchor_a) - at).length() < 1e-3
        });
        match holder {
            Some(piece) => {
                joint.a = piece.id;
                true
            }
            None => false,
        }
    });
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::material::MaterialId;
    use crate::physics::fixtures::{cube, materials, placed};
    use crate::physics::sculpt::sculpt;
    use glam::{Quat, UVec3};

    /// Cut a bar in two, and a joint pinned to its far end follows the piece
    /// that end is on.
    #[test]
    fn a_joint_follows_its_pivot_across_a_split() {
        let materials = materials();
        let bar: Vec<_> = (0..12).map(|x| (UVec3::new(x, 0, 0), MaterialId(1))).collect();
        let mut bodies = vec![placed(
            crate::contree::Contree::from_voxels(16, &bar),
            Vec3::new(30.0, 30.0, 30.0),
            Quat::IDENTITY,
        )];
        let end = bodies[0].world_from_local().transform_point3(Vec3::new(11.5, 0.5, 0.5));
        let mut joints = vec![Joint::new(JointKind::Ball, &bodies[0], None, end, Vec3::Y)];
        // Near the far end, so the joint's end is on the smaller piece, which
        // leaves as a new body.
        let cut = bodies[0].world_from_local().transform_point3(Vec3::new(8.5, 0.5, 0.5));
        sculpt(&mut bodies, 0, cut, 1.2, MaterialId::EMPTY, &materials, 16);
        assert_eq!(bodies.len(), 2, "the bar did not split");
        follow(&mut joints, &bodies);
        assert_eq!(joints.len(), 1);
        assert_eq!(joints[0].a, bodies[1].id, "the joint stayed on the piece without its pivot");
    }

    /// Erase the pivot, or the body, and the joint goes.
    #[test]
    fn a_joint_without_its_pivot_is_removed() {
        let materials = materials();
        let mut bodies = vec![placed(cube(4, 4), Vec3::new(30.0, 30.0, 30.0), Quat::IDENTITY)];
        let corner = bodies[0].world_from_local().transform_point3(Vec3::splat(0.01));
        let mut joints = vec![Joint::new(JointKind::Ball, &bodies[0], None, corner, Vec3::Y)];
        sculpt(&mut bodies, 0, corner, 0.9, MaterialId::EMPTY, &materials, 16);
        assert!(!bodies.is_empty(), "the whole body went, so this tests the wrong thing");
        follow(&mut joints, &bodies);
        assert!(joints.is_empty(), "a joint outlived its pivot voxel");

        let mut joints = vec![Joint::new(JointKind::Ball, &bodies[0], None, corner, Vec3::Y)];
        bodies.clear();
        follow(&mut joints, &bodies);
        assert!(joints.is_empty(), "a joint outlived its body");
    }

    /// A joint whose pivot is untouched stays exactly as it was.
    #[test]
    fn an_untouched_joint_is_kept() {
        let bodies = vec![placed(cube(4, 4), Vec3::new(30.0, 30.0, 30.0), Quat::IDENTITY)];
        let corner = bodies[0].world_from_local().transform_point3(Vec3::splat(0.01));
        let mut joints = vec![Joint::new(JointKind::Ball, &bodies[0], None, corner, Vec3::Y)];
        let before = joints.clone();
        follow(&mut joints, &bodies);
        assert_eq!(joints, before);
    }

    /// Another body with a voxel at the same coordinates, somewhere else in the
    /// world, is not the piece that holds the pivot: its voxel is not where the
    /// pivot was.
    #[test]
    fn a_joint_does_not_jump_to_an_unrelated_body() {
        let materials = materials();
        let mut bodies = vec![
            placed(cube(4, 4), Vec3::new(30.0, 30.0, 30.0), Quat::IDENTITY),
            placed(cube(4, 4), Vec3::new(50.0, 30.0, 30.0), Quat::IDENTITY),
        ];
        let corner = bodies[0].world_from_local().transform_point3(Vec3::splat(0.01));
        let mut joints = vec![Joint::new(JointKind::Ball, &bodies[0], None, corner, Vec3::Y)];
        sculpt(&mut bodies, 0, corner, 0.9, MaterialId::EMPTY, &materials, 16);
        assert!(solid_at(&bodies[1].volume, glam::IVec3::ZERO), "the other body lacks the voxel");
        follow(&mut joints, &bodies);
        assert!(joints.is_empty(), "the joint jumped to a body that never held its pivot");
    }
}
