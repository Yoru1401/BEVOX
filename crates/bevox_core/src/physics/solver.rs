//! Temporal Gauss-Seidel. Contacts are detected once per tick; then each
//! substep applies gravity, warm starts, solves the velocity constraints,
//! integrates and relaxes.
//!
//! The detect-once-then-substep shape and warm starting by voxel pair are from
//! Dwyer's devlog #26. The unbiased relax solve after integrating is from Erin
//! Catto's soft-step solver (Box2D v3): it removes the velocity the push-out
//! bias added, so pushing a body out of the floor does not make it bounce.

use super::contact::{Contact, RADIUS, detect, world_box};
use super::{BASE_MARGIN, BIAS, MAX_PUSH, MAX_TRAVEL, SLOP, SUBSTEPS};
use crate::body::{Body, occupied_bounds};
use crate::contree::Contree;
use crate::distance_field::DistanceField;
use crate::material::MaterialTable;
use glam::{Mat3, Quat, Vec2, Vec3};

/// Advances every body by one tick of `dt` seconds against the static world.
///
/// Bodies entirely below the world are removed first. Returns whether any were:
/// the body list changed, so the caller must rebuild what is packed from it. A
/// body with no mass is not simulated.
pub fn step(
    bodies: &mut Vec<Body>,
    tree: &Contree,
    field: &DistanceField,
    materials: &MaterialTable,
    gravity: Vec3,
    dt: f32,
) -> bool {
    let before = bodies.len();
    bodies.retain(|b| world_box(b, 0.0).is_none_or(|(_, max)| max.y >= 0.0));
    for body in bodies.iter_mut().filter(|b| b.mass.mass > 0.0) {
        step_body(body, tree, field, materials, gravity, dt);
    }
    bodies.len() != before
}

/// What a contact carried last tick, for warm starting.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct ContactImpulse {
    pub normal: f32,
    /// Along the contact's two tangents, in their order.
    pub tangent: Vec2,
}

/// The contact's two tangent directions.
///
/// Derived from the normal only: a basis that depended on velocity would turn
/// between ticks, and last tick's stored tangent impulse would mean nothing.
fn tangents(normal: Vec3) -> (Vec3, Vec3) {
    normal.any_orthonormal_pair()
}

fn step_body(
    body: &mut Body,
    tree: &Contree,
    field: &DistanceField,
    materials: &MaterialTable,
    gravity: Vec3,
    dt: f32,
) {
    let h = dt / SUBSTEPS as f32;
    let inv_h = 1.0 / h;
    let inv_mass = body.mass.inverse_mass();
    let radius = radius(body);

    let travel = cap_speed(body, world_inverse_inertia(body), radius, dt);
    let contacts = detect(body, tree, field, materials, travel + BASE_MARGIN);
    let mut impulses: Vec<ContactImpulse> =
        contacts.iter().map(|c| body.warm.get(&c.key).copied().unwrap_or_default()).collect();

    for _ in 0..SUBSTEPS {
        body.velocity += gravity * h;
        let inv_inertia = world_inverse_inertia(body);
        cap_speed(body, inv_inertia, radius, dt);

        for (c, &impulse) in contacts.iter().zip(&impulses) {
            warm_start(body, inv_mass, c, impulse);
        }
        for (c, impulse) in contacts.iter().zip(impulses.iter_mut()) {
            solve(body, inv_mass, inv_inertia, c, impulse, inv_h, true);
            solve_friction(body, inv_mass, inv_inertia, c, impulse);
        }

        integrate(body, inv_inertia, h);

        let inv_inertia = world_inverse_inertia(body);
        for (c, impulse) in contacts.iter().zip(impulses.iter_mut()) {
            solve(body, inv_mass, inv_inertia, c, impulse, inv_h, false);
            solve_friction(body, inv_mass, inv_inertia, c, impulse);
        }
    }

    body.warm = contacts.iter().zip(&impulses).map(|(c, &i)| (c.key, i)).collect();
}

/// The inverse inertia tensor in world axes.
fn world_inverse_inertia(body: &Body) -> Mat3 {
    let r = Mat3::from_quat(body.orientation);
    r * body.mass.inverse_inertia * r.transpose()
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

/// The contact point's offset from the centre of mass, in world axes.
fn lever(body: &Body, c: &Contact) -> Vec3 {
    body.orientation * c.anchor
}

/// Applies an impulse at the contact point.
fn push(body: &mut Body, inv_mass: f32, c: &Contact, impulse: Vec3) {
    body.velocity += impulse * inv_mass;
    body.angular_momentum += lever(body, c).cross(impulse);
}

/// Re-applies what the contact carried before, normal and friction together.
fn warm_start(body: &mut Body, inv_mass: f32, c: &Contact, impulse: ContactImpulse) {
    let (t1, t2) = tangents(c.normal);
    let p = c.normal * impulse.normal + t1 * impulse.tangent.x + t2 * impulse.tangent.y;
    push(body, inv_mass, c, p);
}

/// Coulomb friction along the contact's two tangents.
///
/// The accumulated tangent impulse is clamped as a vector rather than per axis:
/// clamping each axis alone would let the total reach sqrt(2) times the limit,
/// and a body pushed diagonally would slide further than one pushed along an
/// axis.
fn solve_friction(
    body: &mut Body,
    inv_mass: f32,
    inv_inertia: Mat3,
    c: &Contact,
    impulse: &mut ContactImpulse,
) {
    if c.friction <= 0.0 {
        impulse.tangent = Vec2::ZERO;
        return;
    }
    let (t1, t2) = tangents(c.normal);
    let r = lever(body, c);
    let omega = inv_inertia * body.angular_momentum;
    let v = body.velocity + omega.cross(r);
    let mut delta = Vec2::ZERO;
    for (i, t) in [t1, t2].iter().enumerate() {
        let k = inv_mass + (inv_inertia * r.cross(*t)).cross(r).dot(*t);
        delta[i] = -v.dot(*t) / k;
    }
    let limit = c.friction * impulse.normal;
    let mut total = impulse.tangent + delta;
    if total.length() > limit {
        total = total.normalize_or_zero() * limit;
    }
    let applied = total - impulse.tangent;
    impulse.tangent = total;
    push(body, inv_mass, c, t1 * applied.x + t2 * applied.y);
}

/// One sequential-impulse iteration on one contact.
///
/// The accumulated impulse never goes negative: contacts push, never pull. A
/// contact that is still apart lets the body approach by exactly the gap. A
/// penetrating one, with `use_bias`, pushes the body out by a fraction of the
/// depth beyond the slop.
fn solve(
    body: &mut Body,
    inv_mass: f32,
    inv_inertia: Mat3,
    c: &Contact,
    impulse: &mut ContactImpulse,
    inv_h: f32,
    use_bias: bool,
) {
    let r = lever(body, c);
    let omega = inv_inertia * body.angular_momentum;
    let vn = (body.velocity + omega.cross(r)).dot(c.normal);
    let k = inv_mass + (inv_inertia * r.cross(c.normal)).cross(r).dot(c.normal);

    // The gap at detection, moved by how far the contact point has travelled
    // along the normal since. The world does not move.
    let separation = c.separation + (body.position + r - c.world_point).dot(c.normal);
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
    push(body, inv_mass, c, c.normal * delta);
}

/// Moves and turns the body by one substep.
///
/// `omega` is in world axes, so the spin quaternion multiplies on the left.
/// Dwyer lost three weeks to the other order.
fn integrate(body: &mut Body, inv_inertia: Mat3, h: f32) {
    body.position += body.velocity * h;
    let omega = inv_inertia * body.angular_momentum;
    let spin = Quat::from_xyzw(omega.x, omega.y, omega.z, 0.0) * body.orientation;
    body.orientation = (body.orientation + spin * (0.5 * h)).normalize();
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::material::MaterialId;
    use crate::physics::GRAVITY;
    use crate::physics::fixtures::{cube, materials, placed, slab, slab_of};
    use glam::EulerRot;

    const DT: f32 = 1.0 / 64.0;

    fn run(
        bodies: &mut Vec<Body>,
        world: &Contree,
        field: &DistanceField,
        materials: &MaterialTable,
        gravity: Vec3,
        ticks: u32,
    ) {
        for _ in 0..ticks {
            step(bodies, world, field, materials, gravity, DT);
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
            step(&mut bodies, &world, &field, &materials(), GRAVITY, DT);
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
        step(&mut bodies, &world, &field, &materials(), GRAVITY, DT);
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
            let mut k: Vec<_> = b.warm.keys().map(|(u, w)| (u.to_array(), w.to_array())).collect();
            k.sort_unstable();
            k
        };
        let before = keys(&bodies[0]);
        step(&mut bodies, &world, &field, &materials(), GRAVITY, DT);
        assert!(!before.is_empty());
        assert_eq!(keys(&bodies[0]), before);
    }

    #[test]
    fn a_body_below_the_world_is_removed() {
        let world = slab(64, 0..8);
        let field = DistanceField::build(&world);
        let mut bodies = vec![placed(cube(4, 4), Vec3::new(32.0, -60.0, 32.0), Quat::IDENTITY)];
        assert!(step(&mut bodies, &world, &field, &materials(), GRAVITY, DT));
        assert!(bodies.is_empty());
        let mut bodies = vec![placed(cube(4, 4), Vec3::new(32.0, 40.0, 32.0), Quat::IDENTITY)];
        assert!(!step(&mut bodies, &world, &field, &materials(), GRAVITY, DT));
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
                    step(bodies, &world, &field, &materials, GRAVITY, DT);
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
        println!("16 bodies: falling {fall:.4} ms/tick, resting {rest:.4} ms/tick (median)");
    }
}
