//! Temporal Gauss-Seidel over the whole scene. Contacts are detected once per
//! tick, body against world and body against body; then each substep applies
//! gravity, warm starts, solves the velocity constraints, integrates and
//! relaxes.
//!
//! The world is the case where a contact has no second body: every term of its
//! is zero, so there is one code path rather than two.
//!
//! The detect-once-then-substep shape and warm starting by voxel pair are from
//! Dwyer's devlog #26. The unbiased relax solve after integrating is from Erin
//! Catto's soft-step solver (Box2D v3): it removes the velocity the push-out
//! bias added, so pushing a body out of the floor does not make it bounce.

use super::contact::{Contact, RADIUS, detect, detect_pair, world_box};
use super::joint::{self, Joint};
use super::{
    BASE_MARGIN, BIAS, GRAB_DAMPING, GRAB_FREQUENCY, GRAB_MAX_ACCEL, GRAB_SPIN_DAMPING, MAX_PUSH,
    MAX_TRAVEL, RESTITUTION_SWEEPS, SLOP, SUBSTEPS,
};
use crate::body::{Body, BodyId, occupied_bounds};
use crate::contree::Contree;
use crate::distance_field::DistanceField;
use crate::material::MaterialTable;
use glam::{Mat3, Quat, Vec2, Vec3};
use std::collections::{HashMap, HashSet};

/// What a contact carried last tick, for warm starting.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct ContactImpulse {
    pub normal: f32,
    /// Along the contact's two tangents, in their order.
    pub tangent: Vec2,
}

/// A body held by the mouse: a damped spring from `target` to the point
/// `anchor` of the body with id `body`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Grab {
    pub body: BodyId,
    /// The grabbed point, in the body's volume coordinates, so that recomputing
    /// the body's centre of mass after an edit does not move it.
    pub anchor: Vec3,
    /// Where the spring pulls the grabbed point, in the world.
    pub target: Vec3,
}

impl Grab {
    /// Grabs `body` at the world point `at`, which is also where it is held
    /// until the target moves.
    pub fn new(body: &Body, at: Vec3) -> Self {
        Self { body: body.id, anchor: body.local_from_world().transform_point3(at), target: at }
    }
}

/// A contact and the bodies it joins: the one it belongs to, and the one it is
/// against, unless that is the world.
struct Joined {
    contact: Contact,
    body: usize,
    other: Option<usize>,
}

/// Advances every body by one tick of `dt` seconds, against the static world and
/// against each other.
///
/// Bodies entirely below the world are removed first. Returns whether any were:
/// the body list changed, so the caller must rebuild what is packed from it. A
/// body with no mass is not simulated.
#[allow(clippy::too_many_arguments)]
pub fn step(
    bodies: &mut Vec<Body>,
    tree: &Contree,
    field: &DistanceField,
    materials: &MaterialTable,
    gravity: Vec3,
    dt: f32,
    grab: Option<&Grab>,
    joints: &mut [Joint],
) -> bool {
    let before = bodies.len();
    bodies.retain(|b| world_box(b, 0.0).is_none_or(|(_, max)| max.y >= 0.0));
    let h = dt / SUBSTEPS as f32;
    let inv_h = 1.0 / h;

    // Speeds are capped before anything is detected: a pair's margin depends on
    // how far both of its bodies can travel.
    let radii: Vec<f32> = bodies.iter().map(radius).collect();
    let travel: Vec<f32> = bodies
        .iter_mut()
        .zip(&radii)
        .map(|(b, &r)| {
            if b.mass.mass > 0.0 { cap_speed(b, world_inverse_inertia(b), r, dt) } else { 0.0 }
        })
        .collect();

    // Which bodies each joint joins, by index, this tick. A joint whose body is
    // gone, or has no mass, sits the tick out; `follow` is what removes it.
    let index: HashMap<BodyId, usize> = bodies.iter().enumerate().map(|(i, b)| (b.id, i)).collect();
    let alive = |id: BodyId| index.get(&id).copied().filter(|&i| bodies[i].mass.mass > 0.0);
    let links: Vec<Option<(usize, Option<usize>)>> = joints
        .iter()
        .map(|j| {
            let a = alive(j.a)?;
            match j.b {
                None => Some((a, None)),
                Some(id) => Some((a, Some(alive(id)?))),
            }
        })
        .collect();
    // Jointed pairs do not collide: a hinge where a door meets its frame would
    // otherwise have the contact pushing out while the joint pulls back.
    let jointed: HashSet<(BodyId, BodyId)> =
        joints.iter().filter_map(|j| j.b.map(|b| pair_key(j.a, b))).collect();

    let joined = collect_contacts(bodies, tree, field, materials, &travel, &jointed);
    let mut impulses: Vec<ContactImpulse> = joined
        .iter()
        .map(|j| bodies[j.body].warm.get(&j.contact.key).copied().unwrap_or_default())
        .collect();
    // The speed the bodies arrive with, which the substeps are about to destroy.
    // A bounce is written in terms of it, so it is recorded here.
    let approach: Vec<f32> = joined
        .iter()
        .map(|j| {
            let other = j.other.map(|o| &bodies[o]);
            normal_velocity(&bodies[j.body], other, &j.contact)
        })
        .collect();

    for _ in 0..SUBSTEPS {
        for (b, &r) in bodies.iter_mut().zip(&radii).filter(|(b, _)| b.mass.mass > 0.0) {
            b.velocity += gravity * h;
            if let Some(grab) = grab.filter(|g| g.body == b.id) {
                pull(b, grab, h);
            }
            cap_speed(b, world_inverse_inertia(b), r, dt);
        }

        for (j, &impulse) in joined.iter().zip(&impulses) {
            let (a, b) = pair_mut(bodies, j.body, j.other);
            warm_start(a, b, &j.contact, impulse);
        }
        for (joint, link) in joints.iter().zip(&links) {
            if let Some((a, b)) = *link {
                let (a, b) = pair_mut(bodies, a, b);
                joint::warm_start(a, b, joint);
            }
        }
        solve_joints(bodies, joints, &links, inv_h, true);
        for (j, impulse) in joined.iter().zip(impulses.iter_mut()) {
            let (a, mut b) = pair_mut(bodies, j.body, j.other);
            solve(a, b.as_deref_mut(), &j.contact, impulse, inv_h, true);
            solve_friction(a, b, &j.contact, impulse);
        }

        for b in bodies.iter_mut().filter(|b| b.mass.mass > 0.0) {
            integrate(b, h);
        }

        solve_joints(bodies, joints, &links, inv_h, false);
        for (j, impulse) in joined.iter().zip(impulses.iter_mut()) {
            let (a, mut b) = pair_mut(bodies, j.body, j.other);
            solve(a, b.as_deref_mut(), &j.contact, impulse, inv_h, false);
            solve_friction(a, b, &j.contact, impulse);
        }
    }

    // What the contacts carried while holding the bodies up, which is what warm
    // starting wants next tick. The bounce below is a one-off: feeding it back
    // would have the next tick's warm start kick the body again, and a body
    // would keep almost all its speed instead of `restitution` of it.
    for b in bodies.iter_mut() {
        b.warm.clear();
    }
    for (j, &impulse) in joined.iter().zip(&impulses) {
        bodies[j.body].warm.insert(j.contact.key, impulse);
    }

    apply_restitution(bodies, &joined, &mut impulses, &approach);
    bodies.len() != before
}

/// Every contact in the scene: each body against the world, then each pair of
/// bodies once, with the lower index first.
fn collect_contacts(
    bodies: &[Body],
    tree: &Contree,
    field: &DistanceField,
    materials: &MaterialTable,
    travel: &[f32],
    jointed: &HashSet<(BodyId, BodyId)>,
) -> Vec<Joined> {
    let mut joined = Vec::new();
    for (i, body) in bodies.iter().enumerate() {
        if body.mass.mass <= 0.0 {
            continue;
        }
        for contact in detect(body, tree, field, materials, travel[i] + BASE_MARGIN) {
            joined.push(Joined { contact, body: i, other: None });
        }
    }
    for i in 0..bodies.len() {
        for j in (i + 1)..bodies.len() {
            if bodies[i].mass.mass <= 0.0 || bodies[j].mass.mass <= 0.0 {
                continue;
            }
            if jointed.contains(&pair_key(bodies[i].id, bodies[j].id)) {
                continue;
            }
            let margin = travel[i].max(travel[j]) + BASE_MARGIN;
            for contact in detect_pair(&bodies[i], &bodies[j], materials, margin) {
                joined.push(Joined { contact, body: i, other: Some(j) });
            }
        }
    }
    joined
}

/// Restitution, after the substeps have removed the approach speed. Solving it
/// inside them would fight the push-out bias.
///
/// Swept several times rather than once, and each contact may give impulse back
/// down to what it carried before. A single sweep has every contact of a flat
/// landing push the whole body to the target on its own, four corners stacking
/// to more than the body arrived with; sweeping lets the later ones take that
/// back out.
fn apply_restitution(
    bodies: &mut [Body],
    joined: &[Joined],
    impulses: &mut [ContactImpulse],
    approach: &[f32],
) {
    let base: Vec<f32> = impulses.iter().map(|i| i.normal).collect();
    for _ in 0..RESTITUTION_SWEEPS {
        for ((j, impulse), (&approach, &base)) in
            joined.iter().zip(impulses.iter_mut()).zip(approach.iter().zip(&base))
        {
            let c = &j.contact;
            // A contact that never carried load is one the body never reached.
            //
            // No speed threshold here. One was tried, on the usual reasoning
            // that slow contacts must not bounce or a body buzzes forever, and
            // it changed nothing measurable: a bounce is written against the
            // speed the body ARRIVED with, and a body in sustained contact
            // arrives at nearly zero. Even at a restitution of 0.99 the body
            // settles. It goes back in the day a test shows creep without it.
            if c.restitution <= 0.0 || base == 0.0 {
                continue;
            }
            let (a, mut b) = pair_mut(bodies, j.body, j.other);
            let vn = normal_velocity(a, b.as_deref(), c);
            let target = -c.restitution * approach;
            let k = effective_mass(a, b.as_deref(), c);
            let total = (impulse.normal + (target - vn) / k).max(base);
            let delta = total - impulse.normal;
            impulse.normal = total;
            push(a, b.as_deref_mut(), c, c.normal * delta);
        }
    }
}

/// Pulls the grabbed point toward the target with a damped spring, applied at
/// the point, so the body turns as well as moves; and damps the body's spin,
/// which the spring alone does nothing about.
///
/// The spring is an acceleration scaled by the body's mass, so every body is
/// held alike. It runs once per substep, which keeps `w h` small enough for an
/// explicit spring to stay stable.
fn pull(body: &mut Body, grab: &Grab, h: f32) {
    let w = std::f32::consts::TAU * GRAB_FREQUENCY;
    let point = body.world_from_local().transform_point3(grab.anchor);
    let r = point - body.position;
    let spring = w * w * (grab.target - point) - 2.0 * GRAB_DAMPING * w * point_velocity(body, r);
    let impulse = spring.clamp_length_max(GRAB_MAX_ACCEL) * body.mass.mass * h;
    body.velocity += impulse * body.mass.inverse_mass();
    body.angular_momentum += r.cross(impulse);
    body.angular_momentum *= (1.0 - GRAB_SPIN_DAMPING * h).max(0.0);
}

/// The same key for a pair whichever way round it is named.
fn pair_key(a: BodyId, b: BodyId) -> (BodyId, BodyId) {
    (a.min(b), a.max(b))
}

/// One iteration over every joint whose bodies are here.
fn solve_joints(
    bodies: &mut [Body],
    joints: &mut [Joint],
    links: &[Option<(usize, Option<usize>)>],
    inv_h: f32,
    use_bias: bool,
) {
    for (joint, link) in joints.iter_mut().zip(links) {
        if let Some((a, b)) = *link {
            let (a, b) = pair_mut(bodies, a, b);
            joint::solve(a, b, joint, inv_h, use_bias);
        }
    }
}

/// The two bodies a contact joins, borrowed at once.
fn pair_mut(bodies: &mut [Body], i: usize, j: Option<usize>) -> (&mut Body, Option<&mut Body>) {
    match j {
        None => (&mut bodies[i], None),
        Some(j) => {
            debug_assert_ne!(i, j, "a contact cannot join a body to itself");
            let (lo, hi) = bodies.split_at_mut(i.max(j));
            if i < j { (&mut lo[i], Some(&mut hi[0])) } else { (&mut hi[0], Some(&mut lo[j])) }
        }
    }
}

/// The inverse inertia tensor in world axes.
fn world_inverse_inertia(body: &Body) -> Mat3 {
    body.world_inverse_inertia()
}

/// How far the body's furthest voxel lies from its centre of mass.
fn radius(body: &Body) -> f32 {
    let Some((lo, hi)) = occupied_bounds(&body.volume) else {
        return RADIUS;
    };
    (lo.as_vec3() - body.com).abs().max((hi.as_vec3() - body.com).abs()).length()
}

/// Scales velocity and angular momentum together, so that a tick's travel
/// (linear, plus the furthest voxel's swing) is at most `MAX_TRAVEL`. Returns
/// that travel, which is how far detection must look ahead.
fn cap_speed(body: &mut Body, inv_inertia: Mat3, radius: f32, dt: f32) -> f32 {
    let spin = (inv_inertia * body.angular_momentum).length();
    let travel = (body.velocity.length() + spin * radius) * dt;
    if travel > MAX_TRAVEL {
        let scale = MAX_TRAVEL / travel;
        body.velocity *= scale;
        body.angular_momentum *= scale;
        return MAX_TRAVEL;
    }
    travel
}

/// How fast a point fixed to the body, `r` from its centre of mass, is moving.
fn point_velocity(body: &Body, r: Vec3) -> Vec3 {
    body.point_velocity(r)
}

/// How fast the two surfaces are moving apart, as a vector. The world
/// contributes nothing, because it does not move.
fn relative_velocity(a: &Body, b: Option<&Body>, c: &Contact) -> Vec3 {
    let va = point_velocity(a, a.orientation * c.anchor);
    let vb = b.map_or(Vec3::ZERO, |b| point_velocity(b, b.orientation * c.other_anchor));
    va - vb
}

/// The closing speed along the contact normal. Negative is approaching.
fn normal_velocity(a: &Body, b: Option<&Body>, c: &Contact) -> f32 {
    relative_velocity(a, b, c).dot(c.normal)
}

/// How much impulse one unit of velocity change costs at this contact, along
/// `dir`. Both bodies resist; the world's terms are zero.
fn effective_mass_along(a: &Body, b: Option<&Body>, c: &Contact, dir: Vec3) -> f32 {
    let term = |body: &Body, anchor: Vec3| {
        let r = body.orientation * anchor;
        let inv = world_inverse_inertia(body);
        body.mass.inverse_mass() + (inv * r.cross(dir)).cross(r).dot(dir)
    };
    term(a, c.anchor) + b.map_or(0.0, |b| term(b, c.other_anchor))
}

fn effective_mass(a: &Body, b: Option<&Body>, c: &Contact) -> f32 {
    effective_mass_along(a, b, c, c.normal)
}

/// The gap now: the gap at detection, plus how far the two anchors have moved
/// apart along the normal since.
fn separation(a: &Body, b: Option<&Body>, c: &Contact) -> f32 {
    let moved_a = a.position + a.orientation * c.anchor - c.world_point;
    let moved_b =
        b.map_or(Vec3::ZERO, |b| b.position + b.orientation * c.other_anchor - c.other_point);
    c.separation + (moved_a - moved_b).dot(c.normal)
}

/// The contact's two tangent directions.
///
/// Derived from the normal only: a basis that depended on velocity would turn
/// between ticks, and last tick's stored tangent impulse would mean nothing.
fn tangents(normal: Vec3) -> (Vec3, Vec3) {
    normal.any_orthonormal_pair()
}

/// Applies an impulse at the contact point: the body is pushed along it, and
/// whatever it is touching is pushed the other way.
fn push(a: &mut Body, b: Option<&mut Body>, c: &Contact, impulse: Vec3) {
    let ra = a.orientation * c.anchor;
    a.velocity += impulse * a.mass.inverse_mass();
    a.angular_momentum += ra.cross(impulse);
    if let Some(b) = b {
        let rb = b.orientation * c.other_anchor;
        b.velocity -= impulse * b.mass.inverse_mass();
        b.angular_momentum -= rb.cross(impulse);
    }
}

/// Re-applies what the contact carried before, normal and friction together.
fn warm_start(a: &mut Body, b: Option<&mut Body>, c: &Contact, impulse: ContactImpulse) {
    let (t1, t2) = tangents(c.normal);
    let p = c.normal * impulse.normal + t1 * impulse.tangent.x + t2 * impulse.tangent.y;
    push(a, b, c, p);
}

/// One sequential-impulse iteration on one contact.
///
/// The accumulated impulse never goes negative: contacts push, never pull. A
/// contact that is still apart lets the bodies approach by exactly the gap. A
/// penetrating one, with `use_bias`, pushes them apart by a fraction of the
/// depth beyond the slop.
fn solve(
    a: &mut Body,
    b: Option<&mut Body>,
    c: &Contact,
    impulse: &mut ContactImpulse,
    inv_h: f32,
    use_bias: bool,
) {
    let vn = normal_velocity(a, b.as_deref(), c);
    let k = effective_mass(a, b.as_deref(), c);
    let separation = separation(a, b.as_deref(), c);
    let bias = if separation > 0.0 {
        separation * inv_h
    } else if use_bias {
        (BIAS * (separation + SLOP).min(0.0) * inv_h).max(-MAX_PUSH)
    } else {
        0.0
    };

    let total = (impulse.normal - (vn + bias) / k).max(0.0);
    let delta = total - impulse.normal;
    impulse.normal = total;
    push(a, b, c, c.normal * delta);
}

/// Coulomb friction along the contact's two tangents.
///
/// The accumulated tangent impulse is clamped as a vector rather than per axis:
/// clamping each axis alone would let the total reach sqrt(2) times the limit,
/// and a body pushed diagonally would slide further than one pushed along an
/// axis.
fn solve_friction(
    a: &mut Body,
    b: Option<&mut Body>,
    c: &Contact,
    impulse: &mut ContactImpulse,
) {
    if c.friction <= 0.0 {
        impulse.tangent = Vec2::ZERO;
        return;
    }
    let (t1, t2) = tangents(c.normal);
    let v = relative_velocity(a, b.as_deref(), c);
    let mut delta = Vec2::ZERO;
    for (i, t) in [t1, t2].iter().enumerate() {
        delta[i] = -v.dot(*t) / effective_mass_along(a, b.as_deref(), c, *t);
    }
    let limit = c.friction * impulse.normal;
    let mut total = impulse.tangent + delta;
    if total.length() > limit {
        total = total.normalize_or_zero() * limit;
    }
    let applied = total - impulse.tangent;
    impulse.tangent = total;
    push(a, b, c, t1 * applied.x + t2 * applied.y);
}

/// Moves and turns the body by one substep.
///
/// `omega` is in world axes, so the spin quaternion multiplies on the left.
/// Dwyer lost three weeks to the other order.
fn integrate(body: &mut Body, h: f32) {
    body.position += body.velocity * h;
    let omega = world_inverse_inertia(body) * body.angular_momentum;
    let spin = Quat::from_xyzw(omega.x, omega.y, omega.z, 0.0) * body.orientation;
    body.orientation = (body.orientation + spin * (0.5 * h)).normalize();
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::material::MaterialId;
    use crate::physics::GRAVITY;
    use crate::physics::fixtures::{cube, cube_of, energy, materials, placed, slab, slab_of};
    use crate::physics::joint::{Angular, Joint, Linear};
    use glam::EulerRot;

    const DT: f32 = 1.0 / 64.0;

    /// A ball joint: a Point with the angular part free.
    fn ball(a: &Body, b: Option<&Body>, at: Vec3) -> Joint {
        Joint::new(a, b, Linear::Point, Angular::Free, at, at, Vec3::Y)
    }

    fn run(
        bodies: &mut Vec<Body>,
        world: &Contree,
        field: &DistanceField,
        materials: &MaterialTable,
        gravity: Vec3,
        ticks: u32,
    ) {
        for _ in 0..ticks {
            step(bodies, world, field, materials, gravity, DT, None, &mut []);
        }
    }

    /// With no gravity and nothing to touch, momentum is conserved exactly, not
    /// approximately: velocity and angular momentum are stored, and nothing may
    /// write them.
    #[test]
    fn momentum_is_conserved_exactly_in_free_flight() {
        let world = Contree::empty(3);
        let field = DistanceField::build(&world);
        let mut body =
            placed(cube(4, 4), Vec3::splat(500.0), Quat::from_euler(EulerRot::XYZ, 0.3, 0.9, -0.4));
        body.velocity = Vec3::new(3.0, -1.0, 2.0);
        body.angular_momentum = Vec3::new(20_000.0, -45_000.0, 10_000.0);
        let (v, l, q) = (body.velocity, body.angular_momentum, body.orientation);
        let mut bodies = vec![body];
        run(&mut bodies, &world, &field, &materials(), Vec3::ZERO, 10_000);
        assert_eq!(bodies[0].velocity, v);
        assert_eq!(bodies[0].angular_momentum, l);
        assert!(
            bodies[0].orientation.angle_between(q) > 0.01,
            "it never turned, so this proves nothing"
        );
    }

    /// Semi-implicit Euler per substep has a closed form:
    /// y_N = y_0 - g h^2 N (N + 1) / 2 after N substeps.
    #[test]
    fn a_free_fall_matches_its_closed_form() {
        let world = Contree::empty(3);
        let field = DistanceField::build(&world);
        let mut bodies = vec![placed(cube(4, 4), Vec3::new(32.0, 200.0, 32.0), Quat::IDENTITY)];
        let ticks = 32;
        run(&mut bodies, &world, &field, &materials(), GRAVITY, ticks);
        let h = DT / SUBSTEPS as f32;
        let n = (ticks * SUBSTEPS) as f32;
        let want = 200.0 + GRAVITY.y * h * h * n * (n + 1.0) * 0.5;
        let got = bodies[0].position.y;
        assert!((got - want).abs() < 1e-3 * (200.0 - want), "fell to {got}, want {want}");
    }

    /// A body turning about the world's y axis turns about the world's y axis,
    /// whatever its orientation. Swapping the quaternion product, the bug
    /// Dwyer's devlog #12 spent three weeks on, turns it about a body axis.
    #[test]
    fn a_spin_turns_about_the_world_axis_it_names() {
        let world = Contree::empty(3);
        let field = DistanceField::build(&world);
        let start = Quat::from_euler(EulerRot::XYZ, 0.3, 0.9, -0.4);
        let mut body = placed(cube(4, 4), Vec3::splat(500.0), start);
        // A cube's inertia is the same on every axis, so momentum and spin are
        // parallel.
        let rate = 2.0;
        body.angular_momentum = Vec3::Y * rate / body.mass.inverse_inertia.y_axis.y;
        let mut bodies = vec![body];
        run(&mut bodies, &world, &field, &materials(), Vec3::ZERO, 1);
        let (axis, angle) = (bodies[0].orientation * start.inverse()).to_axis_angle();
        assert!(axis.y.abs() > 0.999, "turned about {axis:?}");
        assert!((angle - rate * DT).abs() < 0.01 * rate * DT, "turned {angle}, want {}", rate * DT);
    }

    /// The milestone in one test: carve the support out and the piece falls,
    /// then rests on the floor.
    #[test]
    fn a_cut_column_falls_and_lands() {
        let materials = materials();
        let mut voxels = slab(64, 0..8).voxels();
        for y in 8..20 {
            voxels.push((glam::UVec3::new(32, y, 32), MaterialId(2)));
        }
        let mut world = Contree::from_voxels(64, &voxels);
        let field = DistanceField::build(&world);

        // Cut the column's base.
        world.clear_voxels(&[glam::UVec3::new(32, 8, 32), glam::UVec3::new(32, 9, 32)]);
        let mut bodies = crate::physics::detach::detach(
            &mut world,
            &materials,
            glam::IVec3::new(31, 7, 31),
            glam::IVec3::new(33, 10, 33),
            16,
        );
        assert_eq!(bodies.len(), 1, "the column did not come free");
        let started_at = bodies[0].position.y;

        run(&mut bodies, &world, &field, &materials, GRAVITY, 200);
        let landed_at = bodies[0].position.y;
        assert!(landed_at < started_at - 1.0, "it never fell: {started_at} to {landed_at}");
        assert!(bodies[0].velocity.length() < 0.5, "it never settled: {:?}", bodies[0].velocity);
        assert!(landed_at > 8.0, "it fell through the floor to {landed_at}");
    }

    /// Held at its centre and moved, a body springs to the target and settles
    /// there, hanging the little gravity asks for.
    #[test]
    fn a_grabbed_body_settles_at_the_target() {
        let materials = materials();
        let world = Contree::empty(3);
        let field = DistanceField::build(&world);
        let mut bodies = vec![placed(cube(4, 4), Vec3::new(30.0, 30.0, 30.0), Quat::IDENTITY)];
        let mut grab = Grab::new(&bodies[0], Vec3::new(30.0, 30.0, 30.0));
        grab.target = Vec3::new(34.0, 32.0, 30.0);
        for _ in 0..300 {
            step(&mut bodies, &world, &field, &materials, GRAVITY, DT, Some(&grab), &mut []);
        }
        let held = bodies[0].world_from_local().transform_point3(grab.anchor);
        assert!((held - grab.target).length() < 0.3, "held at {held:?}, target {:?}", grab.target);
        assert!(bodies[0].velocity.length() < 0.05, "still moving at {:?}", bodies[0].velocity);
    }

    /// Held by a corner, a body hangs straight down from it, and the swing dies
    /// away rather than going on forever.
    #[test]
    fn a_body_held_by_its_corner_hangs_below_it() {
        let materials = materials();
        let world = Contree::empty(3);
        let field = DistanceField::build(&world);
        let mut bodies = vec![placed(cube(4, 4), Vec3::new(30.0, 30.0, 30.0), Quat::IDENTITY)];
        let corner = bodies[0].world_from_local().transform_point3(Vec3::ZERO);
        let grab = Grab::new(&bodies[0], corner);
        for _ in 0..900 {
            step(&mut bodies, &world, &field, &materials, GRAVITY, DT, Some(&grab), &mut []);
        }
        let com = bodies[0].position;
        let off = com - grab.target;
        assert!(off.y < -2.0, "the body is not hanging below the grab: {off:?}");
        assert!(Vec3::new(off.x, 0.0, off.z).length() < 0.2, "the body hangs askew: {off:?}");
        assert!(bodies[0].angular_velocity().length() < 0.05, "still swinging");
    }

    /// A target across the world accelerates the body hard, but no harder than
    /// the grab's cap: no yank to full speed in a single tick.
    #[test]
    fn a_far_target_cannot_yank_a_body() {
        let materials = materials();
        let world = Contree::empty(3);
        let field = DistanceField::build(&world);
        let mut bodies = vec![placed(cube(4, 4), Vec3::new(30.0, 30.0, 30.0), Quat::IDENTITY)];
        let mut grab = Grab::new(&bodies[0], Vec3::new(30.0, 30.0, 30.0));
        grab.target = Vec3::new(3000.0, 30.0, 30.0);
        step(&mut bodies, &world, &field, &materials, Vec3::ZERO, DT, Some(&grab), &mut []);
        let speed = bodies[0].velocity.length();
        assert!(speed <= GRAB_MAX_ACCEL * DT * 1.001, "one tick reached {speed}");
        assert!(speed > 0.5 * GRAB_MAX_ACCEL * DT, "the grab barely pulled: {speed}");
    }

    /// A grab on a body that is gone does nothing, rather than panicking.
    #[test]
    fn a_grab_on_a_missing_body_does_nothing() {
        let materials = materials();
        let world = Contree::empty(3);
        let field = DistanceField::build(&world);
        let gone = placed(cube(4, 4), Vec3::new(30.0, 30.0, 30.0), Quat::IDENTITY);
        let grab = Grab::new(&gone, Vec3::new(30.0, 30.0, 30.0));
        let mut bodies = vec![placed(cube(4, 4), Vec3::new(40.0, 30.0, 30.0), Quat::IDENTITY)];
        step(&mut bodies, &world, &field, &materials, Vec3::ZERO, DT, Some(&grab), &mut []);
        assert_eq!(bodies[0].velocity, Vec3::ZERO);
    }

    /// How far apart a joint's two sides have come.
    fn opening(bodies: &[Body], j: &Joint) -> f32 {
        let a = bodies.iter().find(|b| b.id == j.a).unwrap();
        let b = j.b.map(|id| bodies.iter().find(|b| b.id == id).unwrap());
        let (pa, pb) = j.pivots(a, b);
        (pa - pb).length()
    }

    /// Hung from its centre top, a body stays exactly where it is: the joint
    /// holds its weight.
    #[test]
    fn a_body_hung_from_its_top_stays_put() {
        let materials = materials();
        let world = Contree::empty(3);
        let field = DistanceField::build(&world);
        let mut bodies = vec![placed(cube(4, 4), Vec3::new(30.0, 30.0, 30.0), Quat::IDENTITY)];
        let top = bodies[0].world_from_local().transform_point3(Vec3::new(2.0, 3.99, 2.0));
        let mut joints = vec![ball(&bodies[0], None, top)];
        for _ in 0..1000 {
            step(&mut bodies, &world, &field, &materials, GRAVITY, DT, None, &mut joints);
        }
        assert!(opening(&bodies, &joints[0]) < 0.05, "the joint gave way");
        assert!(bodies[0].velocity.length() < 0.01, "it moved: {:?}", bodies[0].velocity);
    }

    /// Pinned by a corner, a body swings as a pendulum. Nothing damps a joint,
    /// so it never stops; what must hold is that the pivot stays shut all the
    /// while, and that the swing never gains energy, which is how an unstable
    /// solver shows itself.
    #[test]
    fn a_pendulum_keeps_its_pivot_and_gains_no_energy() {
        let materials = materials();
        let world = Contree::empty(3);
        let field = DistanceField::build(&world);
        let mut bodies = vec![placed(cube(4, 4), Vec3::new(30.0, 30.0, 30.0), Quat::IDENTITY)];
        let corner = bodies[0].world_from_local().transform_point3(Vec3::new(0.01, 3.99, 0.01));
        let mut joints = vec![ball(&bodies[0], None, corner)];
        let start = energy(&bodies);
        let scale = bodies[0].mass.mass * -GRAVITY.y * 4.0;
        let (mut widest, mut fastest, mut highest) = (0.0f32, 0.0f32, f32::NEG_INFINITY);
        for _ in 0..3000 {
            step(&mut bodies, &world, &field, &materials, GRAVITY, DT, None, &mut joints);
            widest = widest.max(opening(&bodies, &joints[0]));
            fastest = fastest.max(bodies[0].velocity.length());
            highest = highest.max(energy(&bodies));
        }
        assert!(fastest > 1.0, "it never swung, so this proves nothing");
        assert!(widest < 0.05, "the pivot opened by {widest}");
        assert!(
            highest - start < 0.02 * scale,
            "the pendulum gained energy: {start} rose to {highest}"
        );
    }

    /// A chain of three, hanging and then knocked sideways, stays together and
    /// gains no energy.
    #[test]
    fn a_knocked_chain_stays_together() {
        let materials = materials();
        let world = Contree::empty(3);
        let field = DistanceField::build(&world);
        // Each cube hangs from the bottom of the one above; the top one from the
        // world.
        let mut bodies: Vec<Body> = (0..3)
            .map(|i| placed(cube(4, 4), Vec3::new(30.0, 38.0 - i as f32 * 4.0, 30.0), Quat::IDENTITY))
            .collect();
        let mut joints = vec![ball(&bodies[0], None, Vec3::new(30.0, 39.99, 30.0))];
        for i in 1..3 {
            let meet = Vec3::new(30.0, 39.99 - i as f32 * 4.0, 30.0);
            joints.push(ball(&bodies[i], Some(&bodies[i - 1]), meet));
        }
        bodies[2].velocity = Vec3::new(6.0, 0.0, 2.0);
        let start = energy(&bodies);
        let scale = bodies.iter().map(|b| b.mass.mass).sum::<f32>() * -GRAVITY.y * 4.0;
        let (mut widest, mut highest) = (0.0f32, f32::NEG_INFINITY);
        for _ in 0..5000 {
            step(&mut bodies, &world, &field, &materials, GRAVITY, DT, None, &mut joints);
            for j in &joints {
                widest = widest.max(opening(&bodies, j));
            }
            highest = highest.max(energy(&bodies));
        }
        assert!(widest < 0.1, "a joint opened by {widest}");
        assert!(highest - start < 0.02 * scale, "the chain gained energy: {start} rose to {highest}");
    }

    /// Two jointed bodies that overlap slightly do not push each other apart:
    /// jointed pairs have no contacts. The overlap is shallow on purpose; a deep
    /// one puts each body's corners in the other's interior, where no contact
    /// is ever made, and the gate would pass with the contacts left on.
    #[test]
    fn jointed_bodies_do_not_collide() {
        let materials = materials();
        let world = Contree::empty(3);
        let field = DistanceField::build(&world);
        let mut bodies = vec![
            placed(cube(4, 4), Vec3::new(30.0, 30.0, 30.0), Quat::IDENTITY),
            placed(cube(4, 4), Vec3::new(33.7, 30.0, 30.0), Quat::IDENTITY),
        ];
        let pivot = Vec3::new(31.85, 30.0, 30.0);
        let mut joints = vec![ball(&bodies[1], Some(&bodies[0]), pivot)];
        for _ in 0..30 {
            step(&mut bodies, &world, &field, &materials, Vec3::ZERO, DT, None, &mut joints);
        }
        for b in &bodies {
            assert!(b.velocity.length() < 1e-3, "an overlap pushed a jointed body: {:?}", b.velocity);
        }
    }

    /// A door hinged to the world about a vertical axis turns only about that
    /// axis: pushed, it swings in the horizontal plane, its hinge stays put, and
    /// gravity, pulling on the far edge, does not tip it.
    #[test]
    fn a_hinged_door_turns_only_about_its_axis() {
        let materials = materials();
        let world = Contree::empty(3);
        let field = DistanceField::build(&world);
        // A door 8 wide, 12 tall, 1 thick, from x = 30 to 38.
        let door: Vec<_> = (0..8)
            .flat_map(|x| (0..12).map(move |y| (glam::UVec3::new(x, y, 0), MaterialId(1))))
            .collect();
        let mut bodies = vec![placed(
            Contree::from_voxels(16, &door),
            Vec3::new(34.0, 36.0, 30.5),
            Quat::IDENTITY,
        )];
        let hinge = Vec3::new(30.01, 36.0, 30.5);
        let mut joints = vec![Joint::new(&bodies[0], None, Linear::Point, Angular::Axis, hinge, hinge, Vec3::Y)];
        bodies[0].velocity = Vec3::new(0.0, 0.0, 8.0);
        for _ in 0..300 {
            step(&mut bodies, &world, &field, &materials, GRAVITY, DT, None, &mut joints);
        }
        let door = &bodies[0];
        let up = door.orientation * Vec3::Y;
        assert!(up.y > 0.999, "the door tipped: its up is {up:?}");
        let spin = door.angular_velocity();
        assert!(
            Vec3::new(spin.x, 0.0, spin.z).length() < 0.02,
            "the door turns about more than its axis: {spin:?}"
        );
        assert!(door.orientation.angle_between(Quat::IDENTITY) > 0.2, "the push never swung it");
        assert!(opening(&bodies, &joints[0]) < 0.05, "the hinge moved");
    }

    /// A moving cube hitting a still one of equal mass gives its motion away:
    /// momentum is conserved, and neither passes through the other.
    #[test]
    fn a_cube_knocks_another_along() {
        let materials = materials();
        let world = Contree::empty(3);
        let field = DistanceField::build(&world);
        let mut hitter = placed(cube(4, 4), Vec3::new(20.0, 32.0, 32.0), Quat::IDENTITY);
        hitter.velocity = Vec3::new(20.0, 0.0, 0.0);
        let sitter = placed(cube(4, 4), Vec3::new(30.0, 32.0, 32.0), Quat::IDENTITY);
        let before = hitter.velocity * hitter.mass.mass;
        let mut bodies = vec![hitter, sitter];
        run(&mut bodies, &world, &field, &materials, Vec3::ZERO, 120);

        let after: Vec3 = bodies.iter().map(|b| b.velocity * b.mass.mass).sum();
        assert!(
            (after - before).length() < 0.05 * before.length(),
            "momentum {after:?}, was {before:?}"
        );
        assert!(
            bodies[1].velocity.x > 1.0,
            "the still cube was not knocked along: {:?}",
            bodies[1].velocity
        );
        // Neither material bounces, so equal masses should end up travelling
        // together. They do not quite: the push-out bias adds a little
        // separation, measured at 4.55 voxels/s against a 20 voxels/s impact.
        // The bound sits above that and below what a wrong effective mass or a
        // separation that ignores the other body gives (6.14 and 6.60), which
        // is what makes those breaks visible here.
        let parting = bodies[1].velocity.x - bodies[0].velocity.x;
        assert!(
            parting < 5.5,
            "the cubes parted at {parting} voxels/s; with no bounce they should travel together"
        );
        assert!(
            bodies[0].position.x + 3.5 < bodies[1].position.x,
            "the cubes ended overlapping: {} and {}",
            bodies[0].position.x,
            bodies[1].position.x
        );
    }

    /// Three cubes stacked stay stacked, at the height they settled at. A stack
    /// is how an impulse solver fails: the bottom sinks, or the whole thing
    /// shivers. A single body cannot show either.
    #[test]
    fn a_stack_of_three_stands_still() {
        let materials = materials();
        let world = slab(64, 0..8);
        let field = DistanceField::build(&world);
        let mut bodies: Vec<Body> = (0..3)
            .map(|i| placed(cube(4, 4), Vec3::new(32.0, 10.2 + i as f32 * 4.0, 32.0), Quat::IDENTITY))
            .collect();
        run(&mut bodies, &world, &field, &materials, GRAVITY, 2000);
        let settled: Vec<f32> = bodies.iter().map(|b| b.position.y).collect();
        for (i, y) in settled.iter().enumerate() {
            let want = 10.0 + i as f32 * 4.0;
            assert!((y - want).abs() < 0.2, "body {i} settled at {y}, want about {want}");
        }

        run(&mut bodies, &world, &field, &materials, GRAVITY, 8000);
        for (i, (b, was)) in bodies.iter().zip(&settled).enumerate() {
            assert!(
                (b.position.y - was).abs() < 5e-3,
                "body {i} drifted from {was} to {}",
                b.position.y
            );
            assert!(b.velocity.length() < 2e-2, "body {i} still moving at {:?}", b.velocity);
        }
    }

    /// A body resting on another is held up by it, not by the floor: take the
    /// lower one away and the upper one falls.
    #[test]
    fn taking_the_lower_body_away_drops_the_upper() {
        let materials = materials();
        let world = slab(64, 0..8);
        let field = DistanceField::build(&world);
        let mut bodies = vec![
            placed(cube(4, 4), Vec3::new(32.0, 10.2, 32.0), Quat::IDENTITY),
            placed(cube(4, 4), Vec3::new(32.0, 14.2, 32.0), Quat::IDENTITY),
        ];
        run(&mut bodies, &world, &field, &materials, GRAVITY, 1000);
        assert!(bodies[1].velocity.y.abs() < 0.05, "the upper body never settled");
        let held_at = bodies[1].position.y;
        assert!(held_at > 13.0, "the upper body sank into the lower one: {held_at}");

        bodies.remove(0);
        run(&mut bodies, &world, &field, &materials, GRAVITY, 10);
        assert!(
            bodies[0].velocity.y < -1.0,
            "the upper body hung in the air: {:?}",
            bodies[0].velocity
        );
    }

    /// Dropped flat from half a voxel up, a cube lands and stays put. Jitter is
    /// the characteristic failure of impulse solvers and hides from short tests,
    /// so the last thousand ticks of ten thousand are watched.
    #[test]
    fn a_body_at_rest_stays_at_rest() {
        let world = slab(64, 0..8);
        let field = DistanceField::build(&world);
        let mut bodies = vec![placed(cube(4, 4), Vec3::new(32.0, 10.5, 32.0), Quat::IDENTITY)];
        run(&mut bodies, &world, &field, &materials(), GRAVITY, 1000);
        let settled = bodies[0].position.y;
        assert!((settled - 10.0).abs() < 0.1, "settled at {settled}, not on the floor at 10");

        run(&mut bodies, &world, &field, &materials(), GRAVITY, 8000);
        let (mut low, mut high) = (f32::INFINITY, f32::NEG_INFINITY);
        for _ in 0..1000 {
            step(&mut bodies, &world, &field, &materials(), GRAVITY, DT, None, &mut []);
            low = low.min(bodies[0].position.y);
            high = high.max(bodies[0].position.y);
        }
        let body = &bodies[0];
        assert!(high - low < 1e-4, "height wandered {low}..{high} over the last 1000 ticks");
        assert!(
            (body.position.y - settled).abs() < 1e-3,
            "drifted from {settled} to {}",
            body.position.y
        );
        assert!(body.velocity.length() < 1e-2, "still moving at {:?}", body.velocity);

        // At rest the contacts carry exactly the weight: one tick's normal
        // impulse is m |g| dt.
        assert!(body.warm.len() >= 4, "resting on {} contacts", body.warm.len());
        let per_tick: f32 = body.warm.values().map(|i| i.normal).sum::<f32>() * SUBSTEPS as f32;
        let weight = body.mass.mass * -GRAVITY.y * DT;
        assert!(
            (per_tick - weight).abs() < 0.05 * weight,
            "normal impulse {per_tick} per tick, weight {weight}"
        );
    }

    /// Sliding on stone stops in the distance Coulomb friction predicts,
    /// v^2 / (2 mu g); on ice it keeps going.
    #[test]
    fn a_sliding_body_stops_on_stone_and_slides_on_ice() {
        let materials = materials();
        // Near the edge it starts from, so 56 voxels of sliding on ice still
        // land on the floor rather than off it.
        let start = Vec3::new(4.0, 10.0, 32.0);
        let slide = |floor: MaterialId, ticks: u32| -> (Vec3, Vec3) {
            let world = slab_of(64, 0..8, floor);
            let field = DistanceField::build(&world);
            let mut body = placed(cube(4, 4), start, Quat::IDENTITY);
            body.velocity = Vec3::new(30.0, 0.0, 0.0);
            let mut bodies = vec![body];
            run(&mut bodies, &world, &field, &materials, GRAVITY, ticks);
            (bodies[0].position, bodies[0].velocity)
        };

        let (stone_at, stone_v) = slide(MaterialId(1), 120);
        let expected = 30.0 * 30.0 / (2.0 * 0.6 * -GRAVITY.y);
        assert!(stone_v.length() < 0.2, "still sliding on stone at {stone_v:?}");
        let travelled = stone_at.x - start.x;
        assert!(
            (travelled - expected).abs() < 0.2 * expected,
            "slid {travelled} voxels on stone, Coulomb says about {expected}"
        );

        let (ice_at, ice_v) = slide(MaterialId(3), 120);
        assert!(ice_v.x > 29.0, "ice slowed the body to {ice_v:?}");
        assert!(ice_at.x - start.x > 50.0, "only {} voxels on ice", ice_at.x - start.x);
    }

    /// Friction obeys the Coulomb cone in every direction: a body pushed
    /// diagonally decelerates at the same rate as one pushed along an axis.
    #[test]
    fn friction_is_the_same_in_every_direction() {
        let materials = materials();
        let world = slab_of(64, 0..8, MaterialId(1));
        let field = DistanceField::build(&world);
        let speed = |v: Vec3| -> f32 {
            let mut body = placed(cube(4, 4), Vec3::new(32.0, 10.0, 32.0), Quat::IDENTITY);
            body.velocity = v;
            let mut bodies = vec![body];
            run(&mut bodies, &world, &field, &materials, GRAVITY, 20);
            bodies[0].velocity.length()
        };
        let axis = speed(Vec3::new(30.0, 0.0, 0.0));
        let diagonal = speed(Vec3::new(30.0, 0.0, 30.0).normalize() * 30.0);
        assert!(axis > 1.0, "the body already stopped, so this proves nothing");
        assert!(
            (axis - diagonal).abs() < 0.05 * axis,
            "axis-aligned kept {axis}, diagonal kept {diagonal}"
        );
    }

    /// Friction must not disturb a body that is already still.
    #[test]
    fn friction_leaves_a_resting_body_alone() {
        let materials = materials();
        let world = slab_of(64, 0..8, MaterialId(1));
        let field = DistanceField::build(&world);
        let mut bodies = vec![placed(cube(4, 4), Vec3::new(32.0, 10.5, 32.0), Quat::IDENTITY)];
        run(&mut bodies, &world, &field, &materials, GRAVITY, 1000);
        let settled = bodies[0].position;
        run(&mut bodies, &world, &field, &materials, GRAVITY, 5000);
        assert!(
            (bodies[0].position - settled).length() < 1e-3,
            "crept from {settled:?} to {:?}",
            bodies[0].position
        );
    }

    /// Balanced on an edge but past the tipping point, a cube falls onto a face.
    /// That needs the angular terms of the normal impulse.
    #[test]
    fn a_cube_on_its_edge_tips_onto_a_face() {
        let world = slab(64, 0..8);
        let field = DistanceField::build(&world);
        let tilt = Quat::from_rotation_z(50f32.to_radians());
        let alignment =
            |q: Quat| [Vec3::X, Vec3::Y, Vec3::Z].iter().map(|a| (q * *a).y.abs()).fold(0.0, f32::max);
        assert!(alignment(tilt) < 0.8, "the start is not on an edge, so this proves nothing");

        let mut bodies = vec![placed(cube(4, 4), Vec3::new(32.0, 11.0, 32.0), tilt)];
        run(&mut bodies, &world, &field, &materials(), GRAVITY, 640);
        let body = &bodies[0];
        assert!(alignment(body.orientation) > 0.999, "ended at {:?}, not flat", body.orientation);
        let spin = body.mass.inverse_inertia.y_axis.y * body.angular_momentum.length();
        assert!(
            spin < 0.05 && body.velocity.length() < 0.05,
            "still moving: spin {spin}, {:?}",
            body.velocity
        );
    }

    /// A bounce keeps `e` of the approach speed, so the body returns to about
    /// `e^2` of its drop height. A dead floor keeps nothing.
    #[test]
    fn a_bouncy_body_returns_to_a_quarter_of_its_height() {
        let materials = materials();
        let apex = |floor: MaterialId, body_material: MaterialId| -> f32 {
            let world = slab_of(64, 0..8, floor);
            let field = DistanceField::build(&world);
            let mut bodies = vec![placed(
                cube_of(4, 4, body_material),
                Vec3::new(32.0, 14.0, 32.0),
                Quat::IDENTITY,
            )];
            let mut top: f32 = 0.0;
            let mut landed = false;
            for _ in 0..400 {
                step(&mut bodies, &world, &field, &materials, GRAVITY, DT, None, &mut []);
                let y = bodies[0].position.y;
                landed |= y < 10.2;
                if landed {
                    top = top.max(y - 10.0);
                }
            }
            assert!(landed, "the body never reached the floor");
            top
        };

        // Dropped 4 voxels onto a floor that keeps 0.8 of the approach speed:
        // the first bounce reaches about 0.64 of 4 voxels.
        let bounced = apex(MaterialId(4), MaterialId(1));
        assert!((bounced - 2.56).abs() < 0.6, "bounced to {bounced}, expected about 2.56");

        let dead = apex(MaterialId(1), MaterialId(1));
        assert!(dead < 0.1, "a dead floor bounced the body {dead} voxels");
    }

    /// Below the threshold there is no bounce, or a bouncy body would buzz on
    /// the floor forever instead of settling.
    #[test]
    fn a_bouncy_body_still_settles() {
        let materials = materials();
        let world = slab_of(64, 0..8, MaterialId(4));
        let field = DistanceField::build(&world);
        let mut bodies =
            vec![placed(cube_of(4, 4, MaterialId(4)), Vec3::new(32.0, 12.0, 32.0), Quat::IDENTITY)];
        run(&mut bodies, &world, &field, &materials, GRAVITY, 2000);
        let settled = bodies[0].position.y;
        assert!((settled - 10.0).abs() < 0.1, "settled at {settled}, not on the floor");
        let (mut low, mut high) = (f32::INFINITY, f32::NEG_INFINITY);
        for _ in 0..1000 {
            step(&mut bodies, &world, &field, &materials, GRAVITY, DT, None, &mut []);
            low = low.min(bodies[0].position.y);
            high = high.max(bodies[0].position.y);
        }
        assert!(high - low < 1e-3, "a bouncy body buzzed between {low} and {high}");
    }

    /// A nearly elastic body settles rather than trading micro-bounces forever.
    ///
    /// This is the threshold's gate: at 0.99, a bounce keeps almost everything,
    /// so without a floor on what counts as an impact the body would still be
    /// hopping thousands of ticks later.
    #[test]
    fn a_nearly_elastic_body_settles_rather_than_hopping_forever() {
        let materials = materials();
        let world = slab_of(64, 0..8, MaterialId(5));
        let field = DistanceField::build(&world);
        let mut bodies =
            vec![placed(cube_of(4, 4, MaterialId(5)), Vec3::new(32.0, 11.0, 32.0), Quat::IDENTITY)];
        run(&mut bodies, &world, &field, &materials, GRAVITY, 3000);
        let (mut low, mut high) = (f32::INFINITY, f32::NEG_INFINITY);
        for _ in 0..500 {
            step(&mut bodies, &world, &field, &materials, GRAVITY, DT, None, &mut []);
            low = low.min(bodies[0].position.y);
            high = high.max(bodies[0].position.y);
        }
        assert!(high - low < 1e-3, "still hopping between {low} and {high}");
        assert!((low - 10.0).abs() < 0.1, "settled at {low}, not on the floor");
    }

    /// At the speed cap, a body falls onto a floor one voxel thick and does not
    /// pass through it: the speculative margin covers a tick's travel.
    ///
    /// The start height is chosen so that it would tunnel without the margin.
    /// Capped, the body falls exactly `MAX_TRAVEL` per tick, and without the
    /// margin a corner is only caught while its centre is within 1.1 voxels
    /// above the floor voxel's centre -- a window narrower than the step, so
    /// most phases still catch it by luck. From 40.65 the steps straddle the
    /// window, and the break check below lands the body under the floor.
    #[test]
    fn a_body_at_the_speed_cap_does_not_tunnel_through_a_thin_floor() {
        let world = slab(64, 10..11);
        let field = DistanceField::build(&world);
        let mut body = placed(cube(4, 4), Vec3::new(32.0, 40.65, 32.0), Quat::IDENTITY);
        body.velocity = Vec3::new(0.0, -MAX_TRAVEL / DT, 0.0);
        let mut bodies = vec![body];
        run(&mut bodies, &world, &field, &materials(), GRAVITY, 120);
        let y = bodies[0].position.y;
        assert!((y - 13.0).abs() < 0.2, "ended at {y}, not on the floor at 13");
    }

    /// Detection reads the live tree, so erasing the support drops a resting
    /// body on the very next tick.
    #[test]
    fn erasing_the_support_drops_a_resting_body() {
        let mut world = slab(64, 0..8);
        let field = DistanceField::build(&world);
        let mut bodies = vec![placed(cube(4, 4), Vec3::new(32.0, 10.5, 32.0), Quat::IDENTITY)];
        run(&mut bodies, &world, &field, &materials(), GRAVITY, 300);
        assert!(bodies[0].velocity.y.abs() < 0.05, "not at rest before the edit");
        // Erasing leaves the field under-estimating, which is its safe direction.
        world.apply_sphere(Vec3::new(32.0, 6.0, 32.0), 6.0, MaterialId::EMPTY);
        step(&mut bodies, &world, &field, &materials(), GRAVITY, DT, None, &mut []);
        assert!(bodies[0].velocity.y < -1.0, "still held up: {:?}", bodies[0].velocity);
    }

    /// Warm starting depends on a resting contact keeping its key.
    #[test]
    fn a_resting_body_keeps_its_contact_keys() {
        let world = slab(64, 0..8);
        let field = DistanceField::build(&world);
        let mut bodies = vec![placed(cube(4, 4), Vec3::new(32.0, 10.5, 32.0), Quat::IDENTITY)];
        run(&mut bodies, &world, &field, &materials(), GRAVITY, 300);
        let keys = |b: &Body| {
            let mut k: Vec<_> = b.warm.keys().copied().collect();
            k.sort_unstable();
            k
        };
        let before = keys(&bodies[0]);
        step(&mut bodies, &world, &field, &materials(), GRAVITY, DT, None, &mut []);
        assert!(!before.is_empty());
        assert_eq!(keys(&bodies[0]), before);
    }

    #[test]
    fn a_body_below_the_world_is_removed() {
        let world = slab(64, 0..8);
        let field = DistanceField::build(&world);
        let mut bodies = vec![placed(cube(4, 4), Vec3::new(32.0, -60.0, 32.0), Quat::IDENTITY)];
        assert!(step(&mut bodies, &world, &field, &materials(), GRAVITY, DT, None, &mut []));
        assert!(bodies.is_empty());
        let mut bodies = vec![placed(cube(4, 4), Vec3::new(32.0, 40.0, 32.0), Quat::IDENTITY)];
        assert!(!step(&mut bodies, &world, &field, &materials(), GRAVITY, DT, None, &mut []));
        assert_eq!(bodies.len(), 1);
    }

    /// A body built with `new` and never recomputed has no mass and is not
    /// simulated.
    #[test]
    fn a_massless_body_does_not_move() {
        let world = slab(64, 0..8);
        let field = DistanceField::build(&world);
        let mut bodies = vec![Body::new(cube(4, 4), Vec3::new(32.0, 40.0, 32.0), Quat::IDENTITY)];
        run(&mut bodies, &world, &field, &materials(), GRAVITY, 10);
        assert_eq!(bodies[0].position, Vec3::new(32.0, 40.0, 32.0));
    }

    /// CPU time for one tick. Not a gate; its numbers go in the plan's
    /// Measurements section.
    #[test]
    #[ignore]
    fn a_tick_is_timed() {
        let world = slab(64, 0..8);
        let field = DistanceField::build(&world);
        let grid = |y: f32| -> Vec<Body> {
            (0..16)
                .map(|i| {
                    placed(
                        cube(4, 4),
                        Vec3::new(8.0 + (i % 4) as f32 * 14.0, y, 8.0 + (i / 4) as f32 * 14.0),
                        Quat::IDENTITY,
                    )
                })
                .collect()
        };
        let materials = materials();
        let time = |bodies: &mut Vec<Body>, ticks: u32| -> Vec<f64> {
            (0..ticks)
                .map(|_| {
                    let start = std::time::Instant::now();
                    step(bodies, &world, &field, &materials, GRAVITY, DT, None, &mut []);
                    start.elapsed().as_secs_f64() * 1000.0
                })
                .collect()
        };
        let median = |mut v: Vec<f64>| {
            v.sort_by(|a, b| a.partial_cmp(b).unwrap());
            v[v.len() / 2]
        };
        let mut falling = grid(40.0);
        let fall = median(time(&mut falling, 30));
        let mut resting = grid(10.5);
        time(&mut resting, 300);
        let rest = median(time(&mut resting, 1000));
        // Four stacks of four: every body touching another, and 120 pairs to
        // consider.
        let stacks = || -> Vec<Body> {
            (0..16)
                .map(|i| {
                    placed(
                        cube(4, 4),
                        Vec3::new(16.0 + (i % 4) as f32 * 12.0, 10.2 + (i / 4) as f32 * 4.0, 32.0),
                        Quat::IDENTITY,
                    )
                })
                .collect()
        };
        let mut piled = stacks();
        time(&mut piled, 300);
        let pile = median(time(&mut piled, 1000));
        println!("16 bodies, median ms/tick: falling {fall:.4}, resting {rest:.4}, stacked {pile:.4}");
    }
}
