//! Joints: two bodies, or a body and the world, held together at a pivot.
//!
//! After Dwyer's devlog #30: each joint is a constraint solved as an impulse
//! alongside the contacts. A ball joint keeps two points together; a hinge also
//! keeps two axes aligned, so the bodies turn only about it.

use super::BIAS;
use crate::body::{Body, BodyId};
use glam::{Mat3, Vec3};

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
}
