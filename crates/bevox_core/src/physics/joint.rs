//! Joints: two bodies, or a body and the world, held together.
//!
//! After Dwyer's devlog #30. A joint is one linear part and one angular part,
//! sixteen types in all, and every part is solved with the one expression he
//! derives,
//!
//! ```text
//! λ = −(J M⁻¹ Jᵀ)⁻¹ (J·V + bias)
//! ```
//!
//! over its one to three rows at once. The bias, a fraction of the drift put
//! right each substep, is this project's; he shows none.

use super::{BIAS, GRAB_MAX_FORCE, GRAB_MAX_TORQUE};
use super::classify::solid_at;
use crate::body::{Body, BodyId};
use glam::{Mat3, Quat, Vec3};

/// What a joint does with the two anchors.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Linear {
    /// Nothing.
    Free,
    /// They stay together.
    Point,
    /// The first stays on the line through the second, along the axis.
    Line,
    /// They stay at most this many voxels apart: a rope, which pulls and never
    /// pushes.
    Distance(f32),
}

/// What a joint does with the two bodies' orientations.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Angular {
    /// Nothing.
    Free,
    /// They keep the relative orientation they had when joined.
    Locked,
    /// They turn relative to each other only about the axis.
    Axis,
    /// Their axes stay within this many radians of each other; twist is free.
    Cone(f32),
}

/// The most a joint's friction resists the motion it leaves free: a force on
/// the anchors, a torque on the turn. Zero is none.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Friction {
    pub force: f32,
    pub torque: f32,
}

/// What a motor asks of its motion: along a line in voxels, about an axis in
/// radians.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Drive {
    /// A relative speed.
    Speed(f32),
    /// A position, reached as a servo reaches it.
    Target(f32),
}

/// A motor on a joint's one free motion, with the most force (or torque) it
/// has.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Motor {
    pub drive: Drive,
    pub max: f32,
}

/// What a joint carried last substep, for warm starting. In world axes: a
/// part's rows turn with the bodies, so a number per row would mean nothing by
/// the next substep.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Carried {
    pub linear: Vec3,
    pub angular: Vec3,
    pub linear_friction: Vec3,
    pub angular_friction: Vec3,
    /// Along the line or about the axis the motor drives.
    pub motor: f32,
}

/// A joint between body `a` and body `b`, or the world when `b` is `None`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Joint {
    pub a: BodyId,
    pub b: Option<BodyId>,
    pub linear: Linear,
    pub angular: Angular,
    /// `a`'s anchor, in its volume coordinates.
    pub anchor_a: Vec3,
    /// `b`'s anchor, in its volume coordinates, or in the world.
    pub anchor_b: Vec3,
    /// The line, hinge or cone axis, in `a`'s own axes.
    pub axis_a: Vec3,
    /// The same axis in `b`'s own axes, or the world's.
    pub axis_b: Vec3,
    /// `a`'s orientation relative to `b`'s when the joint was made.
    pub rest: Quat,
    pub friction: Friction,
    pub motor: Option<Motor>,
    /// The most the linear part pulls with, and the angular part turns with.
    /// Unlimited unless set.
    pub max_force: f32,
    pub max_torque: f32,
    pub carried: Carried,
}

impl Joint {
    /// Joins `a`'s point `at_a` to `b`'s point `at_b`, both given in the world,
    /// `b` being the world when it is `None`. Nothing moves: each body is
    /// fastened where it is. `axis`, in world axes, is the line, hinge or cone
    /// axis; the other parts ignore it.
    pub fn new(
        a: &Body,
        b: Option<&Body>,
        linear: Linear,
        angular: Angular,
        at_a: Vec3,
        at_b: Vec3,
        axis: Vec3,
    ) -> Self {
        let axis = axis.normalize();
        let anchor_b = match b {
            Some(b) => b.local_from_world().transform_point3(at_b),
            None => at_b,
        };
        Self {
            a: a.id,
            b: b.map(|b| b.id),
            linear,
            angular,
            anchor_a: a.local_from_world().transform_point3(at_a),
            anchor_b,
            axis_a: a.orientation.inverse() * axis,
            axis_b: orientation(b).inverse() * axis,
            rest: orientation(b).inverse() * a.orientation,
            friction: Friction::default(),
            motor: None,
            max_force: f32::INFINITY,
            max_torque: f32::INFINITY,
            carried: Carried::default(),
        }
    }

    /// The mouse grab: `body` held at the world point `at`, in the orientation
    /// it has now, by a joint to the world whose force and torque are capped.
    /// The target is `anchor_b`; move it to move the body.
    pub fn grab(body: &Body, at: Vec3) -> Self {
        Self {
            max_force: GRAB_MAX_FORCE,
            max_torque: GRAB_MAX_TORQUE,
            ..Self::new(body, None, Linear::Point, Angular::Locked, at, at, Vec3::Y)
        }
    }

    /// The same joint, with friction on what it leaves free.
    pub fn with_friction(mut self, friction: Friction) -> Self {
        self.friction = friction;
        self
    }

    /// The same joint, with a motor on its one free motion: along its line, or
    /// about its axis.
    ///
    /// Panics unless it has exactly one of a Line part and an Axis part. On any
    /// other joint a motor would have no single motion to drive, or two.
    pub fn with_motor(mut self, motor: Motor) -> Self {
        let line = self.linear == Linear::Line;
        let axis = self.angular == Angular::Axis;
        assert!(
            line != axis,
            "a motor needs exactly one of a Line part and an Axis part, not {:?} and {:?}",
            self.linear,
            self.angular
        );
        self.motor = Some(motor);
        self
    }

    /// The two anchors in the world.
    pub fn pivots(&self, a: &Body, b: Option<&Body>) -> (Vec3, Vec3) {
        let pa = a.world_from_local().transform_point3(self.anchor_a);
        let pb = match b {
            Some(b) => b.world_from_local().transform_point3(self.anchor_b),
            None => self.anchor_b,
        };
        (pa, pb)
    }

    /// The axis as each side holds it, in the world.
    pub fn axes(&self, a: &Body, b: Option<&Body>) -> (Vec3, Vec3) {
        (a.orientation * self.axis_a, orientation(b) * self.axis_b)
    }

    /// How far `a` has turned about the axis, relative to `b`, since the joint
    /// was made: radians, from −π to π.
    pub fn twist(&self, a: &Body, b: Option<&Body>) -> f32 {
        let turn = (orientation(b) * self.rest).inverse() * a.orientation;
        wrap(2.0 * turn.xyz().dot(self.axis_a).atan2(turn.w))
    }

    /// How far `a`'s anchor lies along the line from `b`'s, in voxels.
    pub fn slide(&self, a: &Body, b: Option<&Body>) -> f32 {
        let (pa, pb) = self.pivots(a, b);
        (pa - pb).dot(self.axes(a, b).1)
    }
}

/// A side's orientation: the world's is none.
fn orientation(b: Option<&Body>) -> Quat {
    b.map_or(Quat::IDENTITY, |b| b.orientation)
}

/// An angle brought into −π..π.
fn wrap(angle: f32) -> f32 {
    use std::f32::consts::{PI, TAU};
    (angle + PI).rem_euclid(TAU) - PI
}

/// The cross-product matrix: `skew(r) * x == r.cross(x)`.
fn skew(r: Vec3) -> Mat3 {
    Mat3::from_cols(
        Vec3::new(0.0, r.z, -r.y),
        Vec3::new(-r.z, 0.0, r.x),
        Vec3::new(r.y, -r.x, 0.0),
    )
}

/// `n nᵀ`: the projector onto `n`.
fn outer(n: Vec3) -> Mat3 {
    Mat3::from_cols(n * n.x, n * n.y, n * n.z)
}

/// How a point `r` from the centre of mass answers an impulse there: its
/// velocity changes by this matrix times the impulse, `1/m − [r] I⁻¹ [r]`.
fn point_mass(body: &Body, r: Vec3) -> Mat3 {
    let s = skew(r);
    Mat3::from_diagonal(Vec3::splat(body.mass.inverse_mass())) - s * body.world_inverse_inertia() * s
}

/// How the two points answer an impulse pair: both bodies resist.
fn linear_mass(a: &Body, b: Option<&Body>, ra: Vec3, rb: Vec3) -> Mat3 {
    point_mass(a, ra) + b.map_or(Mat3::ZERO, |b| point_mass(b, rb))
}

/// How the relative spin answers an angular impulse pair.
fn angular_mass(a: &Body, b: Option<&Body>) -> Mat3 {
    a.world_inverse_inertia() + b.map_or(Mat3::ZERO, |b| b.world_inverse_inertia())
}

/// Dwyer's `λ = −(J M⁻¹ Jᵀ)⁻¹ (J·V + bias)`, for a part's rows at once.
///
/// The rows are the directions `rows` projects onto: all three axes, the two
/// across an axis, or the one along it. `k` is how a point, or a spin, answers
/// an impulse in every direction, so `rows·k·rows` is `J M⁻¹ Jᵀ`. Padding the
/// directions the rows leave out keeps it invertible and keeps them out of the
/// answer. `error` is `J·V + bias`, before projecting.
///
/// The padding is `k`'s own scale, not 1. A body's `k` is about 1e-5, and a
/// matrix mixing 1e-5 with 1 loses its small eigenvalues in f32: measured, the
/// inverse came back wrong by more than itself, and a hinge between two bodies
/// blew up.
fn block(k: Mat3, rows: Mat3, error: Vec3) -> Vec3 {
    let scale = (k.x_axis.x + k.y_axis.y + k.z_axis.z) / 3.0;
    let padded = rows * k * rows + (Mat3::IDENTITY - rows) * scale;
    -(padded.inverse() * (rows * error))
}

/// A body's lever to the point `at`. Zero for the world.
fn lever(body: Option<&Body>, at: Vec3) -> Vec3 {
    body.map_or(Vec3::ZERO, |b| at - b.position)
}

/// Where a linear part acts on `b`: at `b`'s anchor, except for a line. A
/// line's offset is measured at `a`'s anchor, so `b` is pushed at the point of
/// it that lies there; pushing it anywhere else would twist the pair and make
/// angular momentum out of nothing.
fn b_point(joint: &Joint, pa: Vec3, pb: Vec3) -> Vec3 {
    if joint.linear == Linear::Line { pa } else { pb }
}

/// How fast `a`'s point moves away from `b`'s.
fn relative_velocity(a: &Body, b: Option<&Body>, ra: Vec3, rb: Vec3) -> Vec3 {
    a.point_velocity(ra) - b.map_or(Vec3::ZERO, |b| b.point_velocity(rb))
}

/// How fast `a` turns relative to `b`.
fn relative_spin(a: &Body, b: Option<&Body>) -> Vec3 {
    a.angular_velocity() - b.map_or(Vec3::ZERO, |b| b.angular_velocity())
}

/// Applies `linear` at `ra` and `angular` to `a`, and the opposite to `b`, at
/// `rb`.
fn apply(a: &mut Body, b: Option<&mut Body>, ra: Vec3, rb: Vec3, linear: Vec3, angular: Vec3) {
    a.velocity += linear * a.mass.inverse_mass();
    a.angular_momentum += ra.cross(linear) + angular;
    if let Some(b) = b {
        b.velocity -= linear * b.mass.inverse_mass();
        b.angular_momentum -= rb.cross(linear) + angular;
    }
}

/// How much of an equality's drift is put right per second: `BIAS` of it per
/// substep, or none in the relax pass.
fn bias_rate(use_bias: bool, inv_h: f32) -> f32 {
    if use_bias { BIAS * inv_h } else { 0.0 }
}

/// The bias of a row that resists only one way, as a contact's does. While it
/// is slack, `c < 0`, the bodies may close exactly the gap this substep; past
/// it, `BIAS` of the excess is put right.
fn one_sided_bias(c: f32, inv_h: f32, use_bias: bool) -> f32 {
    if c < 0.0 { c * inv_h } else { c * bias_rate(use_bias, inv_h) }
}

/// Re-applies what the joint carried before.
pub(crate) fn warm_start(a: &mut Body, mut b: Option<&mut Body>, joint: &Joint) {
    let (pa, pb) = joint.pivots(a, b.as_deref());
    let (wa, wb) = joint.axes(a, b.as_deref());
    let c = &joint.carried;
    let ra = pa - a.position;
    let (motor_linear, motor_angular) = match joint.motor {
        Some(_) if joint.linear == Linear::Line => (wb * c.motor, Vec3::ZERO),
        Some(_) => (Vec3::ZERO, wa * c.motor),
        None => (Vec3::ZERO, Vec3::ZERO),
    };
    let rb = lever(b.as_deref(), b_point(joint, pa, pb));
    apply(a, b.as_deref_mut(), ra, rb, c.linear, c.angular + c.angular_friction + motor_angular);
    // Friction and a line's motor act on `b` at `a`'s anchor.
    let rb = lever(b.as_deref(), pa);
    apply(a, b, ra, rb, c.linear_friction + motor_linear, Vec3::ZERO);
}

/// One iteration on one joint. With `use_bias`, a fraction of the drift is
/// put right too; the relax pass after integrating leaves it out, so the
/// correction does not become motion.
pub(crate) fn solve(
    a: &mut Body,
    mut b: Option<&mut Body>,
    joint: &mut Joint,
    inv_h: f32,
    use_bias: bool,
) {
    // Velocity goals first, as in Box2D v3: they run in the relax pass too,
    // because they are not drift corrections.
    solve_motor(a, b.as_deref_mut(), joint, inv_h);
    solve_friction(a, b.as_deref_mut(), joint, inv_h);
    // One-sided rows before equalities, so the hard constraints have the last
    // word: a cone before a point, a rope before a lock.
    if matches!(joint.angular, Angular::Cone(_)) && !matches!(joint.linear, Linear::Distance(_)) {
        solve_angular(a, b.as_deref_mut(), joint, inv_h, use_bias);
        solve_linear(a, b, joint, inv_h, use_bias);
    } else {
        solve_linear(a, b.as_deref_mut(), joint, inv_h, use_bias);
        solve_angular(a, b, joint, inv_h, use_bias);
    }
}

/// The linear part: Dwyer's `x₂+r₂−x₁−r₁`, all of it for a point, across the
/// axis for a line, along it for a rope.
fn solve_linear(a: &mut Body, b: Option<&mut Body>, joint: &mut Joint, inv_h: f32, use_bias: bool) {
    let (pa, pb) = joint.pivots(a, b.as_deref());
    let (ra, rb) = (pa - a.position, lever(b.as_deref(), b_point(joint, pa, pb)));
    let k = linear_mass(a, b.as_deref(), ra, rb);
    let v = relative_velocity(a, b.as_deref(), ra, rb);
    let d = pa - pb;
    let before = joint.carried.linear;
    let limit = joint.max_force / inv_h;
    joint.carried.linear = match joint.linear {
        Linear::Free => return,
        Linear::Point => (before + block(k, Mat3::IDENTITY, v + d * bias_rate(use_bias, inv_h)))
            .clamp_length_max(limit),
        Linear::Line => {
            let n = joint.axes(a, b.as_deref()).1;
            (before + block(k, Mat3::IDENTITY - outer(n), v + d * bias_rate(use_bias, inv_h)))
                .clamp_length_max(limit)
        }
        Linear::Distance(max) => {
            let length = d.length();
            if length < 1e-6 {
                return;
            }
            let n = d / length;
            let lambda =
                block(k, outer(n), v + n * one_sided_bias(length - max, inv_h, use_bias)).dot(n);
            // A rope pulls, never pushes.
            n * (before.dot(n) + lambda).clamp(-limit, 0.0)
        }
    };
    apply(a, b, ra, rb, joint.carried.linear - before, Vec3::ZERO);
}

/// The angular part: Dwyer's `θ₂ − θ₁` for a lock, `a₁·b₂` and `a₁·c₂` for a
/// hinge, and the angle between the axes for a cone.
fn solve_angular(a: &mut Body, b: Option<&mut Body>, joint: &mut Joint, inv_h: f32, use_bias: bool) {
    let k = angular_mass(a, b.as_deref());
    let spin = relative_spin(a, b.as_deref());
    let (wa, wb) = joint.axes(a, b.as_deref());
    let before = joint.carried.angular;
    let limit = joint.max_torque / inv_h;
    joint.carried.angular = match joint.angular {
        Angular::Free => return,
        Angular::Locked => {
            // How far `a` has turned from where `rest` puts it, as a rotation
            // vector.
            let off = a.orientation * (orientation(b.as_deref()) * joint.rest).inverse();
            let off = if off.w < 0.0 { -off } else { off };
            (before + block(k, Mat3::IDENTITY, spin + off.xyz() * (2.0 * bias_rate(use_bias, inv_h))))
                .clamp_length_max(limit)
        }
        Angular::Axis => {
            // Turning `a` about `wa × wb` brings its axis toward `b`'s.
            let error = spin + wb.cross(wa) * bias_rate(use_bias, inv_h);
            (before + block(k, Mat3::IDENTITY - outer(wa), error)).clamp_length_max(limit)
        }
        Angular::Cone(max) => {
            // Relative spin along `wb × wa` opens the angle between the axes.
            let across = wb.cross(wa);
            let sin = across.length();
            if sin < 1e-6 {
                return;
            }
            let u = across / sin;
            let apart = wa.dot(wb).clamp(-1.0, 1.0).acos();
            let lambda =
                block(k, outer(u), spin + u * one_sided_bias(apart - max, inv_h, use_bias)).dot(u);
            // A cone only ever pushes the axes back together.
            u * (before.dot(u) + lambda).clamp(-limit, 0.0)
        }
    };
    apply(a, b, Vec3::ZERO, Vec3::ZERO, Vec3::ZERO, joint.carried.angular - before);
}

/// Adds `delta` to an accumulated impulse, keeps the total within `limit` as a
/// vector, and returns what was added in the end.
fn clamp_add(total: &mut Vec3, delta: Vec3, limit: f32) -> Vec3 {
    let before = *total;
    *total = (before + delta).clamp_length_max(limit);
    *total - before
}

/// The same, for one row.
fn clamp_scalar(total: &mut f32, delta: f32, limit: f32) -> f32 {
    let before = *total;
    *total = (before + delta).clamp(-limit, limit);
    *total - before
}

/// The motor, on the joint's one free motion: toward a speed, or toward a
/// target as a servo. The target's error is wrapped on a hinge, so it takes
/// the short way round. The accumulated impulse is held within `max` times
/// the substep, so a motor stalls against a load beyond `max`.
fn solve_motor(a: &mut Body, b: Option<&mut Body>, joint: &mut Joint, inv_h: f32) {
    let Some(motor) = joint.motor else {
        return;
    };
    let limit = motor.max / inv_h;
    let (wa, wb) = joint.axes(a, b.as_deref());
    if joint.linear == Linear::Line {
        let pa = joint.pivots(a, b.as_deref()).0;
        let (ra, rb) = (pa - a.position, lever(b.as_deref(), pa));
        let want = match motor.drive {
            Drive::Speed(v) => v,
            Drive::Target(x) => (x - joint.slide(a, b.as_deref())) * BIAS * inv_h,
        };
        let speed = relative_velocity(a, b.as_deref(), ra, rb).dot(wb);
        let k = linear_mass(a, b.as_deref(), ra, rb);
        let lambda = block(k, outer(wb), wb * (speed - want)).dot(wb);
        let applied = clamp_scalar(&mut joint.carried.motor, lambda, limit);
        apply(a, b, ra, rb, wb * applied, Vec3::ZERO);
    } else {
        let want = match motor.drive {
            Drive::Speed(v) => v,
            Drive::Target(x) => wrap(x - joint.twist(a, b.as_deref())) * BIAS * inv_h,
        };
        let speed = relative_spin(a, b.as_deref()).dot(wa);
        let lambda = block(angular_mass(a, b.as_deref()), outer(wa), wa * (speed - want)).dot(wa);
        let applied = clamp_scalar(&mut joint.carried.motor, lambda, limit);
        apply(a, b, Vec3::ZERO, Vec3::ZERO, Vec3::ZERO, wa * applied);
    }
}

/// Friction on what the joint leaves free: a relative velocity of zero asked
/// for, the accumulated impulse held within the force (or torque) times the
/// substep, as a vector, as contact friction is. A motor replaces friction on
/// the motion it drives.
fn solve_friction(a: &mut Body, mut b: Option<&mut Body>, joint: &mut Joint, inv_h: f32) {
    let Friction { force, torque } = joint.friction;
    let driven = joint.motor.is_some();
    let (wa, wb) = joint.axes(a, b.as_deref());
    let linear_rows = match joint.linear {
        Linear::Free | Linear::Distance(_) => Some(Mat3::IDENTITY),
        Linear::Line if !driven => Some(outer(wb)),
        _ => None,
    };
    if let Some(rows) = linear_rows.filter(|_| force > 0.0) {
        let pa = joint.pivots(a, b.as_deref()).0;
        let (ra, rb) = (pa - a.position, lever(b.as_deref(), pa));
        let v = relative_velocity(a, b.as_deref(), ra, rb);
        let lambda = block(linear_mass(a, b.as_deref(), ra, rb), rows, v);
        let applied = clamp_add(&mut joint.carried.linear_friction, lambda, force / inv_h);
        apply(a, b.as_deref_mut(), ra, rb, applied, Vec3::ZERO);
    }
    let angular_rows = match joint.angular {
        Angular::Free | Angular::Cone(_) => Some(Mat3::IDENTITY),
        Angular::Axis if !driven => Some(outer(wa)),
        _ => None,
    };
    if let Some(rows) = angular_rows.filter(|_| torque > 0.0) {
        let lambda = block(angular_mass(a, b.as_deref()), rows, relative_spin(a, b.as_deref()));
        let applied = clamp_add(&mut joint.carried.angular_friction, lambda, torque / inv_h);
        apply(a, b, Vec3::ZERO, Vec3::ZERO, Vec3::ZERO, applied);
    }
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
    use crate::contree::Contree;
    use crate::distance_field::DistanceField;
    use crate::material::{MaterialId, MaterialTable};
    use crate::physics::GRAVITY;
    use crate::physics::fixtures::{cube, cube_of, energy, materials, placed};
    use crate::physics::sculpt::sculpt;
    use crate::physics::solver::step;
    use glam::{Quat, UVec3};

    const DT: f32 = 1.0 / 64.0;
    const LINEARS: [Linear; 4] = [Linear::Free, Linear::Point, Linear::Line, Linear::Distance(3.0)];
    const ANGULARS: [Angular; 4] = [Angular::Free, Angular::Locked, Angular::Axis, Angular::Cone(0.5)];

    /// An empty world to step bodies in.
    struct Space {
        world: Contree,
        field: DistanceField,
        materials: MaterialTable,
    }

    impl Space {
        fn new() -> Self {
            let world = Contree::empty(3);
            let field = DistanceField::build(&world);
            Self { world, field, materials: materials() }
        }

        fn step(&self, bodies: &mut Vec<Body>, joints: &mut [Joint], gravity: Vec3) {
            step(bodies, &self.world, &self.field, &self.materials, gravity, DT, None, joints);
        }

        fn step_holding(&self, bodies: &mut Vec<Body>, grab: &mut Joint, gravity: Vec3) {
            step(bodies, &self.world, &self.field, &self.materials, gravity, DT, Some(grab), &mut []);
        }
    }

    /// A ball joint: a Point with the angular part free.
    fn ball(a: &Body, b: Option<&Body>, at: Vec3) -> Joint {
        Joint::new(a, b, Linear::Point, Angular::Free, at, at, Vec3::Y)
    }

    /// The two bodies `j` joins, as they are now.
    fn sides<'a>(bodies: &'a [Body], j: &Joint) -> (&'a Body, Option<&'a Body>) {
        let find = |id: BodyId| bodies.iter().find(|b| b.id == id).unwrap();
        (find(j.a), j.b.map(find))
    }

    /// How far each part of `j` is from holding: (linear in voxels, angular in
    /// radians). Zero while it holds.
    fn violation(bodies: &[Body], j: &Joint) -> (f32, f32) {
        let (a, b) = sides(bodies, j);
        let (pa, pb) = j.pivots(a, b);
        let (wa, wb) = j.axes(a, b);
        let d = pa - pb;
        let linear = match j.linear {
            Linear::Free => 0.0,
            Linear::Point => d.length(),
            Linear::Line => (d - wb * d.dot(wb)).length(),
            Linear::Distance(max) => (d.length() - max).max(0.0),
        };
        let apart = wa.dot(wb).clamp(-1.0, 1.0).acos();
        let angular = match j.angular {
            Angular::Free => 0.0,
            Angular::Locked => {
                let held = b.map_or(Quat::IDENTITY, |b| b.orientation) * j.rest;
                held.angle_between(a.orientation)
            }
            Angular::Axis => apart,
            Angular::Cone(max) => (apart - max).max(0.0),
        };
        (linear, angular)
    }

    /// How far `j` has let its body move where it should be free: (linear,
    /// angular). `start` is where the body began.
    fn freedom(bodies: &[Body], j: &Joint, start: (Vec3, Quat)) -> (f32, f32) {
        let (a, b) = sides(bodies, j);
        let (pa, pb) = j.pivots(a, b);
        let (wa, wb) = j.axes(a, b);
        let linear = match j.linear {
            Linear::Free => (a.position - start.0).length(),
            Linear::Point => 0.0,
            Linear::Line => j.slide(a, b).abs(),
            Linear::Distance(max) => max - (pa - pb).length(),
        };
        let angular = match j.angular {
            Angular::Free => a.orientation.angle_between(start.1),
            Angular::Locked => 0.0,
            Angular::Axis => j.twist(a, b).abs(),
            Angular::Cone(_) => wa.dot(wb).clamp(-1.0, 1.0).acos(),
        };
        (linear, angular)
    }

    /// Every one of the sixteen joints holds what it constrains and frees what
    /// it does not.
    ///
    /// A cube is joined to the world by its top and kicked so that every free
    /// motion is used: along the line, x; up past a rope's anchor, so the rope
    /// goes slack; spinning on every axis, which includes the hinge's. A part
    /// that gives way shows as a violation; a part that holds too much shows as
    /// a free motion that never happened.
    #[test]
    fn every_joint_holds_what_it_should_and_frees_the_rest() {
        let space = Space::new();
        for linear in LINEARS {
            for angular in ANGULARS {
                let name = format!("{linear:?} + {angular:?}");
                let mut bodies = vec![placed(cube(4, 4), Vec3::new(30.0, 30.0, 30.0), Quat::IDENTITY)];
                let at_a = Vec3::new(30.0, 31.5, 30.0);
                let at_b = match linear {
                    Linear::Distance(max) => at_a + Vec3::Y * max,
                    _ => at_a,
                };
                let mut joints = vec![Joint::new(&bodies[0], None, linear, angular, at_a, at_b, Vec3::X)];
                bodies[0].velocity = Vec3::new(4.0, 20.0, -2.0);
                bodies[0].set_angular_velocity(Vec3::new(2.0, -3.0, 1.5));
                let start = (bodies[0].position, bodies[0].orientation);
                let (mut worst, mut freest) = ((0.0f32, 0.0f32), (0.0f32, 0.0f32));
                for _ in 0..1000 {
                    space.step(&mut bodies, &mut joints, Vec3::ZERO);
                    let (l, a) = violation(&bodies, &joints[0]);
                    worst = (worst.0.max(l), worst.1.max(a));
                    let (l, a) = freedom(&bodies, &joints[0], start);
                    freest = (freest.0.max(l), freest.1.max(a));
                }
                assert!(worst.0 < 0.05, "{name}: the linear part gave way by {}", worst.0);
                assert!(worst.1 < 0.02, "{name}: the angular part gave way by {} rad", worst.1);
                if linear != Linear::Point {
                    assert!(freest.0 > 0.5, "{name}: the linear part held what it frees ({})", freest.0);
                }
                if angular != Angular::Locked {
                    assert!(freest.1 > 0.3, "{name}: the angular part held what it frees ({})", freest.1);
                }
            }
        }
    }

    /// A joint between two free bodies moves momentum between them, linear
    /// and angular, and never makes or destroys any, whatever its type.
    /// Angular momentum is about the origin: orbit plus spin.
    #[test]
    fn every_joint_conserves_momentum() {
        let space = Space::new();
        let momentum = |bodies: &[Body]| -> (Vec3, Vec3) {
            let linear: Vec3 = bodies.iter().map(|b| b.velocity * b.mass.mass).sum();
            let angular: Vec3 = bodies
                .iter()
                .map(|b| b.position.cross(b.velocity * b.mass.mass) + b.angular_momentum)
                .sum();
            (linear, angular)
        };
        for linear in LINEARS {
            for angular in ANGULARS {
                let name = format!("{linear:?} + {angular:?}");
                let mut bodies = vec![
                    placed(cube(4, 4), Vec3::new(30.0, 30.0, 30.0), Quat::IDENTITY),
                    placed(cube_of(4, 4, MaterialId(2)), Vec3::new(34.5, 30.0, 30.0), Quat::IDENTITY),
                ];
                // Anchors apart where the part allows it, and `b` kicked across
                // the line between them: friction then pulls across that line,
                // and pushing `b` at the wrong point would twist the pair.
                let at_a = Vec3::new(31.5, 30.5, 30.5);
                let at_b = match linear {
                    Linear::Free | Linear::Distance(_) => Vec3::new(33.0, 30.5, 30.5),
                    _ => at_a,
                };
                let axis = Vec3::new(0.3, 1.0, 0.2);
                let mut joint = Joint::new(&bodies[0], Some(&bodies[1]), linear, angular, at_a, at_b, axis)
                    // Strong enough to matter: at 1e5 a misplaced friction
                    // lever cost 5e-5 of the angular momentum, inside the
                    // tolerance, and the gate could not see it.
                    .with_friction(Friction { force: 1.0e7, torque: 1.0e7 });
                if (linear == Linear::Line) != (angular == Angular::Axis) {
                    joint = joint.with_motor(Motor { drive: Drive::Speed(1.0), max: 1.0e6 });
                }
                let mut joints = vec![joint];
                bodies[0].velocity = Vec3::new(0.0, 6.0, -3.0);
                bodies[0].set_angular_velocity(Vec3::new(1.0, 2.0, 0.0));
                bodies[1].velocity = Vec3::new(-2.0, 0.0, 5.0);
                let (p0, l0) = momentum(&bodies);
                for _ in 0..200 {
                    space.step(&mut bodies, &mut joints, Vec3::ZERO);
                }
                let (p, l) = momentum(&bodies);
                assert!((p - p0).length() < 1e-3 * p0.length(), "{name}: momentum {p:?}, was {p0:?}");
                assert!(
                    (l - l0).length() < 1e-3 * l0.length(),
                    "{name}: angular momentum {l:?}, was {l0:?}"
                );
            }
        }
    }

    /// Nothing damps a joint without friction, so a swinging body never stops.
    /// What must hold is that it never gains energy, which is how an unstable
    /// solver shows itself.
    ///
    /// The Free linear part is left out: with gravity, its body just falls.
    #[test]
    fn no_joint_gains_energy() {
        let space = Space::new();
        for linear in [Linear::Point, Linear::Line, Linear::Distance(3.0)] {
            for angular in ANGULARS {
                let name = format!("{linear:?} + {angular:?}");
                let mut bodies = vec![placed(cube(4, 4), Vec3::new(30.0, 30.0, 30.0), Quat::IDENTITY)];
                // Off the top's middle, so gravity swings the body as well as
                // pulling it down.
                let at_a = Vec3::new(31.5, 31.5, 30.0);
                let at_b = match linear {
                    Linear::Distance(max) => at_a + Vec3::Y * max,
                    _ => at_a,
                };
                let mut joints = vec![Joint::new(&bodies[0], None, linear, angular, at_a, at_b, Vec3::X)];
                bodies[0].velocity = Vec3::new(3.0, 0.0, 2.0);
                let start = energy(&bodies);
                let scale = bodies[0].mass.mass * -GRAVITY.y * 4.0;
                let mut highest = f32::NEG_INFINITY;
                for _ in 0..3000 {
                    space.step(&mut bodies, &mut joints, GRAVITY);
                    highest = highest.max(energy(&bodies));
                }
                assert!(highest - start < 0.02 * scale, "{name}: gained energy, {start} rose to {highest}");
            }
        }
    }

    /// A door 8 wide, 12 tall and 1 thick, centred at `centre`.
    fn door(centre: Vec3) -> Body {
        let voxels: Vec<_> = (0..8)
            .flat_map(|x| (0..12).map(move |y| (UVec3::new(x, y, 0), MaterialId(1))))
            .collect();
        placed(Contree::from_voxels(16, &voxels), centre, Quat::IDENTITY)
    }

    /// A bar 8 long, from x = 30 to 38, hinged at its left end about z.
    fn hinged_bar() -> (Vec<Body>, Joint) {
        let voxels: Vec<_> = (0..8).map(|x| (UVec3::new(x, 0, 0), MaterialId(1))).collect();
        let bar = placed(Contree::from_voxels(16, &voxels), Vec3::new(34.0, 30.5, 30.5), Quat::IDENTITY);
        let end = Vec3::new(30.5, 30.5, 30.5);
        let joint = Joint::new(&bar, None, Linear::Point, Angular::Axis, end, end, Vec3::Z);
        (vec![bar], joint)
    }

    /// A door swinging on a hinge with friction slows and stops; without it,
    /// it would swing on.
    #[test]
    fn friction_stops_a_swinging_door() {
        let space = Space::new();
        let mut bodies = vec![door(Vec3::new(34.0, 36.0, 30.5))];
        let hinge = Vec3::new(30.5, 36.0, 30.5);
        let mut joints = vec![
            Joint::new(&bodies[0], None, Linear::Point, Angular::Axis, hinge, hinge, Vec3::Y)
                .with_friction(Friction { force: 0.0, torque: 1.0e7 }),
        ];
        // Swinging about the hinge: the centre of mass moves at
        // `ω × (com − hinge)`. A spin about the centre alone would mostly be
        // taken out by the hinge, leaving too little swing to see.
        let spin = Vec3::Y * 2.0;
        bodies[0].set_angular_velocity(spin);
        bodies[0].velocity = spin.cross(bodies[0].position - hinge);
        for _ in 0..300 {
            space.step(&mut bodies, &mut joints, GRAVITY);
        }
        assert!(bodies[0].orientation.angle_between(Quat::IDENTITY) > 0.1, "it never swung");
        let spin = bodies[0].angular_velocity().length();
        assert!(spin < 1e-3, "still turning at {spin}");
    }

    /// Joint friction resists up to its force and no further. A block on a
    /// friction joint to the world, with less friction than its weight, falls
    /// at `g − F/m`; with more, it hangs where it is.
    #[test]
    fn joint_friction_holds_up_to_its_force() {
        let space = Space::new();
        let fall = |share: f32, ticks: u32| -> Body {
            let mut bodies = vec![placed(cube(4, 4), Vec3::new(30.0, 60.0, 30.0), Quat::IDENTITY)];
            let weight = bodies[0].mass.mass * -GRAVITY.y;
            let at = bodies[0].position;
            let mut joints = vec![
                Joint::new(&bodies[0], None, Linear::Free, Angular::Free, at, at, Vec3::Y)
                    .with_friction(Friction { force: share * weight, torque: 0.0 }),
            ];
            for _ in 0..ticks {
                space.step(&mut bodies, &mut joints, GRAVITY);
            }
            bodies.remove(0)
        };
        let weak = fall(0.5, 64);
        let want = 0.5 * GRAVITY.y;
        assert!(
            (weak.velocity.y - want).abs() < 0.02 * want.abs(),
            "fell at {} voxels/s after a second, want {want}",
            weak.velocity.y
        );
        let strong = fall(2.0, 300);
        assert!((strong.position.y - 60.0).abs() < 0.01, "slid to {}", strong.position.y);
    }

    /// A speed motor drives its hinge at its speed when it is strong enough,
    /// and stalls when the load is more than its torque.
    #[test]
    fn a_speed_motor_drives_and_stalls() {
        let space = Space::new();
        let run = |max: f32| -> (f32, f32) {
            let (mut bodies, joint) = hinged_bar();
            let mut joints = vec![joint.with_motor(Motor { drive: Drive::Speed(2.0), max })];
            let mut highest = f32::NEG_INFINITY;
            for _ in 0..300 {
                space.step(&mut bodies, &mut joints, GRAVITY);
                highest = highest.max(joints[0].twist(&bodies[0], None));
            }
            (bodies[0].angular_velocity().z, highest)
        };
        // The bar weighs 8000; held level, gravity turns it with 8000 · 98 · 4,
        // about 3.1e6. 5%: the point rows, solved after the motor, move the
        // spin a little each pass while gravity pulls on the bar.
        let (spin, _) = run(1.0e8);
        assert!((spin - 2.0).abs() < 0.1, "a strong motor turns at {spin}, not 2");
        let (_, highest) = run(1.0e6);
        assert!(highest < 0.5, "a weak motor lifted the bar to {highest} rad");
    }

    /// A target motor turns its hinge to the target and holds it there, and
    /// brings it back when it is knocked away.
    #[test]
    fn a_target_motor_holds_its_angle() {
        let space = Space::new();
        let target = std::f32::consts::FRAC_PI_4;
        let (mut bodies, joint) = hinged_bar();
        // Over three times the bar's pull at 45 degrees, about 2.2e6, yet weak
        // enough that a knock visibly moves it: 1e8 stops the knock within two
        // substeps.
        let mut joints = vec![joint.with_motor(Motor { drive: Drive::Target(target), max: 1.0e7 })];
        for _ in 0..300 {
            space.step(&mut bodies, &mut joints, GRAVITY);
        }
        let at = joints[0].twist(&bodies[0], None);
        assert!((at - target).abs() < 0.02, "held at {at}, not {target}");

        // Knocked about the hinge, as the door is pushed: a spin about the
        // centre alone would mostly be taken out by the hinge.
        let spin = Vec3::Z * -5.0;
        let end = bodies[0].world_from_local().transform_point3(joints[0].anchor_a);
        bodies[0].set_angular_velocity(spin);
        bodies[0].velocity = spin.cross(bodies[0].position - end);
        let mut lowest = f32::INFINITY;
        for _ in 0..300 {
            space.step(&mut bodies, &mut joints, GRAVITY);
            lowest = lowest.min(joints[0].twist(&bodies[0], None));
        }
        assert!(lowest < target - 0.1, "the knock never moved it");
        let at = joints[0].twist(&bodies[0], None);
        assert!((at - target).abs() < 0.02, "came back to {at}, not {target}");
    }

    /// From 3 radians to −3, a target motor goes the short way, through π, not
    /// six radians back through 0.
    #[test]
    fn a_target_motor_takes_the_short_way_round() {
        let space = Space::new();
        let (mut bodies, joint) = hinged_bar();
        let mut joints = vec![joint.with_motor(Motor { drive: Drive::Target(3.0), max: 1.0e8 })];
        for _ in 0..300 {
            space.step(&mut bodies, &mut joints, Vec3::ZERO);
        }
        assert!((joints[0].twist(&bodies[0], None) - 3.0).abs() < 0.02, "never reached 3");
        joints[0].motor = Some(Motor { drive: Drive::Target(-3.0), max: 1.0e8 });
        let mut nearest_zero = f32::INFINITY;
        for _ in 0..300 {
            space.step(&mut bodies, &mut joints, Vec3::ZERO);
            nearest_zero = nearest_zero.min(joints[0].twist(&bodies[0], None).abs());
        }
        assert!(nearest_zero > 2.5, "went the long way, through {nearest_zero}");
        assert!((joints[0].twist(&bodies[0], None) + 3.0).abs() < 0.02, "never reached -3");
    }

    /// A motor needs exactly one motion to drive.
    #[test]
    #[should_panic(expected = "a motor needs")]
    fn a_motor_on_a_joint_without_one_free_motion_panics() {
        let body = placed(cube(4, 4), Vec3::new(30.0, 30.0, 30.0), Quat::IDENTITY);
        let _ = ball(&body, None, body.position).with_motor(Motor { drive: Drive::Speed(1.0), max: 1.0 });
    }

    /// Grabbed by a corner and moved, a body follows with that corner and keeps
    /// the pose it was grabbed in, rather than hanging from the corner.
    #[test]
    fn a_grab_holds_the_point_and_the_pose() {
        use crate::physics::GRAB_MAX_FORCE;
        let space = Space::new();
        let mut bodies = vec![placed(cube(4, 4), Vec3::new(30.0, 30.0, 30.0), Quat::IDENTITY)];
        let corner = bodies[0].world_from_local().transform_point3(Vec3::new(0.01, 3.99, 0.01));
        let mut grab = Joint::grab(&bodies[0], corner);
        assert_eq!(grab.max_force, GRAB_MAX_FORCE);
        grab.anchor_b = corner + Vec3::new(2.0, 1.0, 0.0);
        for _ in 0..600 {
            space.step_holding(&mut bodies, &mut grab, GRAVITY);
        }
        let held = bodies[0].world_from_local().transform_point3(grab.anchor_a);
        assert!((held - grab.anchor_b).length() < 0.05, "held at {held:?}, target {:?}", grab.anchor_b);
        let turned = bodies[0].orientation.angle_between(Quat::IDENTITY);
        assert!(turned < 0.02, "the body turned {turned} rad in the grab");
    }

    /// A light body is lifted to the target; one heavier than the grab's force
    /// stays on the floor.
    #[test]
    fn a_grab_lifts_a_light_body_but_not_a_heavy_one() {
        let materials = materials();
        let world = crate::physics::fixtures::slab(64, 0..8);
        let field = DistanceField::build(&world);
        let lift = |volume: Contree, centre: Vec3| -> f32 {
            let mut bodies = vec![placed(volume, centre, Quat::IDENTITY)];
            let mut grab = Joint::grab(&bodies[0], centre);
            grab.anchor_b = centre + Vec3::Y * 5.0;
            for _ in 0..300 {
                step(&mut bodies, &world, &field, &materials, GRAVITY, DT, Some(&mut grab), &mut []);
            }
            bodies[0].position.y - centre.y
        };
        // 64 voxels at 1000 weigh 64,000; the grab lifts 2,000,000.
        let light = lift(cube(4, 4), Vec3::new(20.0, 10.0, 32.0));
        assert!((light - 5.0).abs() < 0.1, "a light body rose {light}, not 5");
        // 1000 voxels at 3000 weigh 3,000,000.
        let heavy = lift(cube_of(10, 16, MaterialId(2)), Vec3::new(40.0, 13.0, 32.0));
        assert!(heavy.abs() < 0.5, "a heavy body rose {heavy}");
    }

    /// A target across the world pulls with the grab's force and no more: one
    /// tick reaches at most `F dt / m`.
    #[test]
    fn a_far_target_pulls_with_the_capped_force() {
        use crate::physics::GRAB_MAX_FORCE;
        let space = Space::new();
        let mut bodies = vec![placed(cube(4, 4), Vec3::new(30.0, 30.0, 30.0), Quat::IDENTITY)];
        let mut grab = Joint::grab(&bodies[0], bodies[0].position);
        grab.anchor_b = Vec3::new(3000.0, 30.0, 30.0);
        space.step_holding(&mut bodies, &mut grab, Vec3::ZERO);
        let speed = bodies[0].velocity.length();
        let most = GRAB_MAX_FORCE * DT / bodies[0].mass.mass;
        assert!(speed <= most * 1.001, "one tick reached {speed}, the cap allows {most}");
        assert!(speed > 0.5 * most, "the grab barely pulled: {speed}");
    }

    /// Letting go keeps the momentum the grab gave: a body carried sideways and
    /// released keeps going, as a flick would throw it.
    #[test]
    fn letting_go_keeps_the_momentum() {
        let space = Space::new();
        let mut bodies = vec![placed(cube(4, 4), Vec3::new(30.0, 30.0, 30.0), Quat::IDENTITY)];
        let mut grab = Joint::grab(&bodies[0], bodies[0].position);
        for tick in 0..128 {
            grab.anchor_b = Vec3::new(30.0 + tick as f32 * 0.1, 30.0, 30.0);
            space.step_holding(&mut bodies, &mut grab, Vec3::ZERO);
        }
        // The target moved 0.1 a tick: 6.4 voxels a second. The body ends a
        // tick slower than that: the target jumps once a tick and the four
        // substeps close 20% of the gap each, so the last one moves at
        // `0.2 · 0.8³ · L / h`, with `L = 0.1 / (1 − 0.8⁴)`: 4.44, as measured.
        // Relaxing the grab would leave nearly nothing.
        let carried = bodies[0].velocity.x;
        assert!(carried > 0.5 * 6.4 && carried < 6.4, "carried at {carried}, the target at 6.4");
        space.step(&mut bodies, &mut [], Vec3::ZERO);
        assert_eq!(bodies[0].velocity.x, carried, "letting go changed the velocity");
    }

    /// A grab on a body that is gone does nothing, rather than panicking.
    #[test]
    fn a_grab_on_a_missing_body_does_nothing() {
        let space = Space::new();
        let gone = placed(cube(4, 4), Vec3::new(30.0, 30.0, 30.0), Quat::IDENTITY);
        let mut grab = Joint::grab(&gone, gone.position);
        let mut bodies = vec![placed(cube(4, 4), Vec3::new(40.0, 30.0, 30.0), Quat::IDENTITY)];
        space.step_holding(&mut bodies, &mut grab, Vec3::ZERO);
        assert_eq!(bodies[0].velocity, Vec3::ZERO);
    }

    /// Cut a bar in two, and a joint pinned to its far end follows the piece
    /// that end is on.
    #[test]
    fn a_joint_follows_its_pivot_across_a_split() {
        let materials = materials();
        let bar: Vec<_> = (0..12).map(|x| (UVec3::new(x, 0, 0), MaterialId(1))).collect();
        let mut bodies = vec![placed(Contree::from_voxels(16, &bar), Vec3::new(30.0, 30.0, 30.0), Quat::IDENTITY)];
        let end = bodies[0].world_from_local().transform_point3(Vec3::new(11.5, 0.5, 0.5));
        let mut joints = vec![ball(&bodies[0], None, end)];
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
        let mut joints = vec![ball(&bodies[0], None, corner)];
        sculpt(&mut bodies, 0, corner, 0.9, MaterialId::EMPTY, &materials, 16);
        assert!(!bodies.is_empty(), "the whole body went, so this tests the wrong thing");
        follow(&mut joints, &bodies);
        assert!(joints.is_empty(), "a joint outlived its pivot voxel");

        let mut joints = vec![ball(&bodies[0], None, corner)];
        bodies.clear();
        follow(&mut joints, &bodies);
        assert!(joints.is_empty(), "a joint outlived its body");
    }

    /// A joint whose pivot is untouched stays exactly as it was.
    #[test]
    fn an_untouched_joint_is_kept() {
        let bodies = vec![placed(cube(4, 4), Vec3::new(30.0, 30.0, 30.0), Quat::IDENTITY)];
        let corner = bodies[0].world_from_local().transform_point3(Vec3::splat(0.01));
        let mut joints = vec![ball(&bodies[0], None, corner)];
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
        let mut joints = vec![ball(&bodies[0], None, corner)];
        sculpt(&mut bodies, 0, corner, 0.9, MaterialId::EMPTY, &materials, 16);
        assert!(solid_at(&bodies[1].volume, glam::IVec3::ZERO), "the other body lacks the voxel");
        follow(&mut joints, &bodies);
        assert!(joints.is_empty(), "the joint jumped to a body that never held its pivot");
    }
}
