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
use super::fracture::{self, Fracture};
use super::joint::{self, Joint};
use super::sleep;
use super::{
    Air, BASE_MARGIN, BIAS, MAX_PUSH, MAX_SPEED, RESTITUTION_SWEEPS, ROLLING, SLOP, SUBSTEPS, Tuning,
};
use bevox_core::body::{Body, BodyId, occupied_bounds};
use bevox_core::contree::Contree;
use bevox_core::distance_field::DistanceField;
use bevox_core::material::MaterialTable;
use glam::{Mat3, Quat, UVec3, Vec2, Vec3};
use std::collections::{HashMap, HashSet};
pub use bevox_core::body::ContactImpulse;

/// What a tick produced besides new positions.
#[derive(Clone, Debug, Default)]
pub struct StepOutcome {
    /// A body left the world, so the packed buffers must be rebuilt.
    pub rebuild: bool,
    /// Contacts that carried more than their material could take. The caller
    /// applies them: this crate knows nothing about the scene they belong to.
    pub fractures: Vec<Fracture>,
    /// The largest blow any contact carried this tick -- the same `peak[at]`
    /// the fracture threshold reads, before any strength is applied. Zero when
    /// nothing touched.
    ///
    /// Reported because a strength is only meaningful against a measured load,
    /// and the load is otherwise invisible from outside a tick:
    /// `solver::tests::what_the_tick_rate_does_to_the_blow` is what reads it.
    pub peak_impulse: f32,
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
/// body with no mass is not simulated. `grab`, the mouse's joint to the world,
/// is solved with the other joints.
#[allow(clippy::too_many_arguments)]
pub fn step(
    bodies: &mut Vec<Body>,
    tree: &Contree,
    field: &DistanceField,
    materials: &MaterialTable,
    air: Air,
    dt: f32,
    grab: Option<&mut Joint>,
    joints: &mut [Joint],
) -> StepOutcome {
    step_with(bodies, tree, field, materials, air, dt, grab, joints, Tuning::default())
}

/// `step`, with the solver's sweep counts exposed. `step` is this with
/// `Tuning::default()`, which is 1 and 1 -- bit for bit what `step` has always
/// done.
#[allow(clippy::too_many_arguments)]
pub fn step_with(
    bodies: &mut Vec<Body>,
    tree: &Contree,
    field: &DistanceField,
    materials: &MaterialTable,
    air: Air,
    dt: f32,
    grab: Option<&mut Joint>,
    joints: &mut [Joint],
    tuning: Tuning,
) -> StepOutcome {
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

    // Before anything is detected, wake whatever something awake could move, so
    // no contact or joint below is ever solved against a sleeper.
    sleep::wake(bodies, joints, grab.as_ref().map(|g| g.a), &travel);

    // Jointed pairs do not collide: a hinge where a door meets its frame would
    // otherwise have the contact pushing out while the joint pulls back.
    let jointed: HashSet<(BodyId, BodyId)> =
        joints.iter().filter_map(|j| j.b.map(|b| pair_key(j.a, b))).collect();
    // The grab is one more joint, to the world, last in the list, so the relax
    // pass can leave it out.
    let scene_joints = joints.len();
    let mut joints: Vec<&mut Joint> = joints.iter_mut().chain(grab).collect();

    // Which bodies each joint joins, by index, this tick. A joint whose body is
    // gone, or has no mass, sits the tick out; `follow` is what removes it. So
    // does one whose body sleeps, which after the wake pass means a joint to the
    // world with nothing awake to move it.
    let index: HashMap<BodyId, usize> = bodies.iter().enumerate().map(|(i, b)| (b.id, i)).collect();
    let alive =
        |id: BodyId| index.get(&id).copied().filter(|&i| bodies[i].mass.mass > 0.0 && !bodies[i].asleep);
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

    let joined = collect_contacts(bodies, tree, field, materials, &travel, &jointed);
    let mut impulses: Vec<ContactImpulse> = joined
        .iter()
        .map(|j| bodies[j.body].warm.get(&j.contact.key).copied().unwrap_or_default())
        .collect();
    // The most any contact carried at any point in the tick. The accumulated
    // impulse is a running total that a separating contact drives back to zero
    // before the tick ends, so it cannot answer "did these surfaces ever press
    // on each other" -- which is what tells a collision from a speculative
    // contact that closed and never touched. `break_what_gave_way` needs that.
    let mut peak: Vec<f32> = vec![0.0; joined.len()];
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
        for (b, &r) in bodies.iter_mut().zip(&radii).filter(|(b, _)| b.mass.mass > 0.0 && !b.asleep) {
            // Drag before the cap, and the cap almost never fires: what limits
            // a fall is a force, so the body eases into its terminal speed
            // instead of having its acceleration switched off in one tick.
            // Gravity and drag are accelerations, so they skip the mass;
            // whatever gameplay pushed with is a force and does not.
            let pushed = b.force * b.mass.inverse_mass();
            b.velocity += (air.gravity + pushed - air.drag * b.velocity * b.velocity.length()) * h;
            b.angular_momentum += b.torque * h;
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
        for _ in 0..tuning.velocity_iterations {
            solve_joints(bodies, &mut joints, &links, inv_h, true);
            for ((j, impulse), peak) in joined.iter().zip(impulses.iter_mut()).zip(peak.iter_mut()) {
                let (a, mut b) = pair_mut(bodies, j.body, j.other);
                solve(a, b.as_deref_mut(), &j.contact, impulse, inv_h, true);
                solve_friction(a, b.as_deref_mut(), &j.contact, impulse);
                solve_rolling(a, b, &j.contact, *impulse);
                *peak = peak.max(impulse.normal);
            }
        }

        for b in bodies.iter_mut().filter(|b| b.mass.mass > 0.0 && !b.asleep) {
            integrate(b, h);
        }

        // The grab is not relaxed. Its target moves, and the bias is how that
        // motion reaches the body: relaxing it would stop the body dead every
        // substep, and letting go would throw nothing.
        for _ in 0..tuning.relaxation_iterations {
            solve_joints(bodies, &mut joints[..scene_joints], &links[..scene_joints], inv_h, false);
            for ((j, impulse), peak) in joined.iter().zip(impulses.iter_mut()).zip(peak.iter_mut()) {
                let (a, mut b) = pair_mut(bodies, j.body, j.other);
                solve(a, b.as_deref_mut(), &j.contact, impulse, inv_h, false);
                solve_friction(a, b.as_deref_mut(), &j.contact, impulse);
                solve_rolling(a, b, &j.contact, *impulse);
                *peak = peak.max(impulse.normal);
            }
        }
    }

    // What the contacts carried while holding the bodies up, which is what warm
    // starting wants next tick. The bounce below is a one-off: feeding it back
    // would have the next tick's warm start kick the body again, and a body
    // would keep almost all its speed instead of `restitution` of it.
    // A sleeper keeps what it carried, for the tick it wakes.
    for b in bodies.iter_mut().filter(|b| !b.asleep) {
        b.warm.clear();
    }
    for (j, &impulse) in joined.iter().zip(&impulses) {
        bodies[j.body].warm.insert(j.contact.key, impulse);
    }

    // Spent: a force acts for the tick it was given for, and no longer. A
    // caller that wants to keep pushing says so every tick.
    for b in bodies.iter_mut() {
        b.force = Vec3::ZERO;
        b.torque = Vec3::ZERO;
    }

    apply_restitution(bodies, &joined, &mut impulses, &approach);
    let fractures = break_what_gave_way(bodies, tree, materials, &joined, &peak);
    sleep::settle(bodies, &joints, &radii, dt);
    StepOutcome {
        rebuild: bodies.len() != before,
        fractures,
        peak_impulse: peak.iter().copied().fold(0.0, f32::max),
    }
}

/// Which contacts carried more than their material could take, and the impulse
/// handed back to the bodies for the ones that did.
///
/// Both sides of a contact are tested: a crate dropped on ice can break the
/// crate, the ice, or both, and both are asked about the same blow.
fn break_what_gave_way(
    bodies: &mut [Body],
    tree: &Contree,
    materials: &MaterialTable,
    joined: &[Joined],
    peak: &[f32],
) -> Vec<Fracture> {
    let mut fractures = Vec::new();
    let mut give_back = Vec::new();
    for (at, j) in joined.iter().enumerate() {
        let c = &j.contact;
        // The largest `over` any side reported, which is the weakest side's:
        // `over` is the blow over that side's own threshold, so the biggest one
        // names the lowest threshold. That is the threshold breaking cost, and
        // the rest of the impulse is the contact's to give back.
        let mut weakest_over = 0.0f32;
        // The most the contact carried at any point in the tick, which is what
        // a material's strength is written in.
        //
        // Dwyer's devlog 28, and his argument against a force is kept: a
        // collision resolves inside one tick, so the force it reports depends
        // on the tick rate, and the impulse it exchanges does not.
        // `the_same_collision_breaks_at_any_tick_rate` holds that.
        //
        // The impulse rather than the closing speed, which this used to be,
        // because a closing speed cannot see a crush. The grab is a joint with
        // an enormous force limit driving toward a velocity goal, so a body
        // pressed into something leans on it with up to `GRAB_MAX_FORCE` while
        // the contact holds both surfaces still: the approach speed is ~0
        // however hard the press. `a_slow_crush_breaks_what_it_presses` is
        // that gate.
        //
        // The units are why the first attempt at this failed. An impulse grows
        // with the mass a contact holds up, so a settled stack carries a large
        // one for standing there -- warm starting even seeds each tick from the
        // last -- and `strength` in the tens broke everything.
        // `resting_weight_breaks_nothing` is that gate, and every strength in
        // the tables is now calibrated above the resting load it measures.
        // It is read as the impulse itself, never divided by a mass: a contact
        // belongs to whichever body the scene lists first, so a quotient would
        // read one collision two ways, which
        // `a_collision_breaks_the_same_things_whichever_body_is_listed_first`
        // holds against.
        let blow = peak[at];
        // Inert for any positive strength, and kept for two smaller reasons.
        //
        // It used to carry the argument that a speculative contact which never
        // touched must not be read as a blow. With the impulse that argument
        // is `over_strength`'s and it holds there: `peak` is a running maximum
        // from 0, so a contact that never touched gives exactly 0, which is
        // past no positive strength. What is left is a *negative* strength,
        // where `0 <= strength` is false and `over_strength` would report a
        // break for a contact that never happened -- and the two material
        // lookups and bounds checks below, which this skips for every
        // speculative contact in the scene.
        if peak[at] <= 0.0 {
            continue;
        }
        // The struck voxel's material, scaled by the size of the body that
        // holds it: `voxels` is that body's whole volume, not anything local to
        // the impact. Dwyer's rule, and it is why a chip of glass gives way
        // under a load a sheet of it holds.
        let mut side = |voxel: [u32; 3], body: Option<BodyId>, strength: f32, voxels: u32| {
            if let Some(over) = fracture::over_strength(blow, strength, voxels) {
                weakest_over = weakest_over.max(over);
                fractures.push(Fracture {
                    at: c.world_point,
                    voxel: UVec3::from(voxel),
                    body,
                    impulse: peak[at],
                    blow,
                    over,
                });
            }
        };

        let mine = &bodies[j.body];
        side(
            c.key.mine,
            Some(mine.id),
            materials.get(mine.volume.get(UVec3::from(c.key.mine))).strength,
            mine.voxel_count,
        );
        match j.other {
            // The static world's own voxel, in world coordinates. It has no
            // body and so no voxel count, and it takes `SIZE_CAP` -- saturated,
            // because the world is the largest thing in the scene. Reading the
            // missing count as 1 would give terrain the threshold of a single
            // chip, the most fragile thing anywhere, and nothing in the suite
            // asks about the world's size, so nothing would catch it.
            None => side(
                c.key.theirs,
                None,
                materials.get(tree.get(UVec3::from(c.key.theirs))).strength,
                fracture::SIZE_CAP,
            ),
            Some(o) => {
                let other = &bodies[o];
                let material = other.volume.get(UVec3::from(c.key.theirs));
                side(
                    c.key.theirs,
                    Some(other.id),
                    materials.get(material).strength,
                    other.voxel_count,
                );
            }
        }
        if weakest_over > 1.0 {
            // What the contact carried past the weakest threshold it met.
            // `over` is the blow over that threshold, so the threshold is the
            // blow over `over`, and the excess is what is left.
            give_back.push((at, blow - blow / weakest_over));
        }
    }

    // Separately, because the scan holds the bodies immutably. Breaking
    // something costs less speed than bouncing off it: the contact keeps what
    // breaking took and returns the rest, which is `fracture::hand_back`.
    //
    // Against the peak, not the running total: a collision that resolved and
    // let go inside the tick ends it carrying nothing, and giving back a share
    // of nothing would leave the striking body stopped dead at a hole it made.
    for (at, excess) in give_back {
        let j = &joined[at];
        let (a, b) = pair_mut(bodies, j.body, j.other);
        push(a, b, &j.contact, -j.contact.normal * fracture::hand_back(excess, peak[at]));
    }
    fractures
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
        if body.mass.mass <= 0.0 || body.asleep {
            continue;
        }
        for contact in detect(body, tree, field, materials, travel[i] + BASE_MARGIN) {
            joined.push(Joined { contact, body: i, other: None });
        }
    }
    for i in 0..bodies.len() {
        for j in (i + 1)..bodies.len() {
            if bodies[i].mass.mass <= 0.0 || bodies[j].mass.mass <= 0.0 || bodies[i].asleep || bodies[j].asleep {
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
            let (a, b) = pair_mut(bodies, j.body, j.other);
            let vn = normal_velocity(a, b.as_deref(), c);
            let target = -c.restitution * approach;
            let k = effective_mass(a, b.as_deref(), c);
            let total = (impulse.normal + (target - vn) / k).max(base);
            let delta = total - impulse.normal;
            impulse.normal = total;
            push(a, b, c, c.normal * delta);
        }
    }
}

/// The same key for a pair whichever way round it is named.
fn pair_key(a: BodyId, b: BodyId) -> (BodyId, BodyId) {
    (a.min(b), a.max(b))
}

/// One iteration over every joint whose bodies are here.
fn solve_joints(
    bodies: &mut [Body],
    joints: &mut [&mut Joint],
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
/// (linear, plus the furthest voxel's swing) is at most `MAX_SPEED`. Returns
/// that travel, which is how far detection must look ahead.
fn cap_speed(body: &mut Body, inv_inertia: Mat3, radius: f32, dt: f32) -> f32 {
    let spin = (inv_inertia * body.angular_momentum).length();
    let speed = body.velocity.length() + spin * radius;
    if speed > MAX_SPEED {
        let scale = MAX_SPEED / speed;
        body.velocity *= scale;
        body.angular_momentum *= scale;
        return MAX_SPEED * dt;
    }
    speed * dt
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

/// Resistance to rolling, which sliding friction cannot provide.
///
/// A rolling contact barely slips, so `solve_friction` finds almost no relative
/// velocity to oppose and a sphere rolls for ever. This opposes the spin
/// itself, with an angular impulse limited by what the contact is pressing
/// with -- `ROLLING * friction * normal impulse * RADIUS` -- and never more
/// than the spin that is there, so it can slow a roll to a stop and never
/// reverse one or add energy.
fn solve_rolling(a: &mut Body, b: Option<&mut Body>, c: &Contact, impulse: ContactImpulse) {
    if c.friction <= 0.0 || impulse.normal <= 0.0 {
        return;
    }
    let relative = a.angular_velocity() - b.as_deref().map_or(Vec3::ZERO, |b| b.angular_velocity());
    let Some(axis) = relative.try_normalize() else {
        return;
    };
    // The lever a voxel contact turns on, so the limit is a torque.
    let limit = ROLLING * c.friction * impulse.normal * RADIUS;
    let resist = -axis * limit.min(relative.length() / inverse_inertia_about(a, b.as_deref(), axis));
    a.angular_momentum += resist;
    if let Some(b) = b {
        b.angular_momentum -= resist;
    }
}

/// How freely the pair turns about `axis`, which is what converts an angular
/// impulse into a change in spin.
fn inverse_inertia_about(a: &Body, b: Option<&Body>, axis: Vec3) -> f32 {
    let of = |body: &Body| axis.dot(world_inverse_inertia(body) * axis);
    (of(a) + b.map_or(0.0, of)).max(1e-12)
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
    use bevox_core::material::MaterialId;
    use crate::GRAVITY;
    use crate::TERMINAL_SPEED;
    use crate::mass::recompute;
    use crate::fixtures::{cube, cube_of, energy, materials, placed, slab, slab_of};
    use crate::joint::{Angular, Joint, Linear};
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
            step(bodies, world, field, materials, Air::vacuum(gravity), DT, None, &mut []);
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
        let mut bodies = crate::detach::detach(
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
            step(&mut bodies, &world, &field, &materials, Air::VACUUM, DT, None, &mut joints);
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
            step(&mut bodies, &world, &field, &materials, Air::VACUUM, DT, None, &mut joints);
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
            step(&mut bodies, &world, &field, &materials, Air::VACUUM, DT, None, &mut joints);
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
            step(&mut bodies, &world, &field, &materials, Air::STILL, DT, None, &mut joints);
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
            step(&mut bodies, &world, &field, &materials, Air::VACUUM, DT, None, &mut joints);
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

    /// `step` and `step_with(.., Tuning::default())` are the same function:
    /// the three-body stack, run 2000 ticks through each from identical
    /// starting state, ends bit for bit identical. Not a tolerance -- the
    /// default tuning must change nothing, because the loops it adds run
    /// exactly once.
    #[test]
    fn a_tick_is_unchanged_by_the_default_tuning() {
        let materials = materials();
        let world = slab(64, 0..8);
        let field = DistanceField::build(&world);
        let start: Vec<Body> = (0..3)
            .map(|i| placed(cube(4, 4), Vec3::new(32.0, 10.2 + i as f32 * 4.0, 32.0), Quat::IDENTITY))
            .collect();

        let mut via_step = start.clone();
        for _ in 0..2000 {
            step(&mut via_step, &world, &field, &materials, Air::vacuum(GRAVITY), DT, None, &mut []);
        }

        let mut via_step_with = start.clone();
        for _ in 0..2000 {
            step_with(
                &mut via_step_with,
                &world,
                &field,
                &materials,
                Air::vacuum(GRAVITY),
                DT,
                None,
                &mut [],
                crate::Tuning::default(),
            );
        }

        for (a, b) in via_step.iter().zip(&via_step_with) {
            assert_eq!(a.position, b.position, "position diverged under the default tuning");
            assert_eq!(a.orientation, b.orientation, "orientation diverged under the default tuning");
            assert_eq!(a.velocity, b.velocity, "velocity diverged under the default tuning");
            assert_eq!(
                a.angular_momentum, b.angular_momentum,
                "angular_momentum diverged under the default tuning"
            );
        }
    }

    /// Spending more velocity iterations may not make a resting contact
    /// *worse*. The three-cube stack of `a_stack_of_three_stands_still`,
    /// settled 2000 ticks at 8 velocity iterations, sits no deeper below its
    /// settled reference than the same stack at `Tuning::default()`.
    ///
    /// This is the one monotonic property the 2026-10-01 measurement found
    /// (0.0003 against 0.0017 -- see
    /// `docs/concepts/solver-convergence-is-a-setting.md`). Deeper
    /// penetration under more work would be the signature of a mis-scaled
    /// push-out bias, and nothing else in the suite looks at penetration as a
    /// function of the iteration count.
    #[test]
    fn more_iterations_do_not_deepen_a_resting_contact() {
        let materials = materials();
        let world = slab(64, 0..8);
        let field = DistanceField::build(&world);
        let fixture = || -> Vec<Body> {
            (0..3)
                .map(|i| placed(cube(4, 4), Vec3::new(32.0, 10.2 + i as f32 * 4.0, 32.0), Quat::IDENTITY))
                .collect()
        };

        // How far the bottom cube sits from the 10.0 it settles at in
        // `a_stack_of_three_stands_still`, after the same 2000 ticks.
        //
        // `.abs()`, not `.max(0.0)`: clamping at zero reads a body floating
        // *above* its reference as a perfect contact, and the thing this gate
        // watches for is a mis-scaled push-out bias, which shows up as travel
        // in either direction.
        let settle = |tuning: crate::Tuning| -> f32 {
            let mut bodies = fixture();
            for _ in 0..2000 {
                step_with(
                    &mut bodies,
                    &world,
                    &field,
                    &materials,
                    Air::vacuum(GRAVITY),
                    DT,
                    None,
                    &mut [],
                    tuning,
                );
            }
            (10.0 - bodies[0].position.y).abs()
        };

        let one = settle(crate::Tuning::default());
        let eight = settle(crate::Tuning { velocity_iterations: 8, relaxation_iterations: 1 });
        assert!(
            eight <= one + 1e-4,
            "eight velocity iterations penetrate {eight} against the default's {one}: more work made the resting contact worse"
        );
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

        // Taking a body away is an edit, and wakes what it held up, as the app
        // does through `wake_near`.
        let (lo, hi) = world_box(&bodies[0], 1.0).unwrap();
        bodies.remove(0);
        crate::sleep::wake_near(&mut bodies, lo, hi);
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
            step(&mut bodies, &world, &field, &materials(), Air::VACUUM, DT, None, &mut []);
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
                step(&mut bodies, &world, &field, &materials, Air::VACUUM, DT, None, &mut []);
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
            step(&mut bodies, &world, &field, &materials, Air::VACUUM, DT, None, &mut []);
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
            step(&mut bodies, &world, &field, &materials, Air::VACUUM, DT, None, &mut []);
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
    /// Capped, the body falls exactly `MAX_SPEED * dt` per tick, and without the
    /// margin a corner is only caught while its centre is within 1.1 voxels
    /// above the floor voxel's centre -- a window narrower than the step, so
    /// most phases still catch it by luck. From 40.65 the steps straddle the
    /// window, and the break check below lands the body under the floor.
    #[test]
    fn a_body_at_the_speed_cap_does_not_tunnel_through_a_thin_floor() {
        let world = slab(64, 10..11);
        let field = DistanceField::build(&world);
        // At every rate, because the travel per tick is what the speculative
        // contacts have to cover and a slower tick covers more of it: at 32 Hz
        // the ceiling is eight voxels a tick against a floor one voxel thick.
        for hz in [32.0f32, 64.0, 128.0] {
            let dt = 1.0 / hz;
            let mut body = placed(cube(4, 4), Vec3::new(32.0, 40.65, 32.0), Quat::IDENTITY);
            body.velocity = Vec3::new(0.0, -MAX_SPEED, 0.0);
            let mut bodies = vec![body];
            for _ in 0..(2.0 / dt) as usize {
                step(&mut bodies, &world, &field, &materials(), Air::VACUUM, dt, None, &mut []);
            }
            assert!(!bodies.is_empty(), "at {hz} Hz the body fell out of the world");
            let y = bodies[0].position.y;
            assert!((y - 13.0).abs() < 0.3, "at {hz} Hz it ended at {y}, not on the floor at 13");
        }
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
        // At rest this long the body sleeps, and a sleeper is woken by an edit
        // only through `wake_near`, as the app's edits call it.
        crate::sleep::wake_near(&mut bodies, Vec3::new(25.0, -1.0, 25.0), Vec3::new(39.0, 13.0, 39.0));
        step(&mut bodies, &world, &field, &materials(), Air::VACUUM, DT, None, &mut []);
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
        step(&mut bodies, &world, &field, &materials(), Air::VACUUM, DT, None, &mut []);
        assert!(!before.is_empty());
        assert_eq!(keys(&bodies[0]), before);
    }

    /// What looking for fractures costs a tick that finds none, and what a tick
    /// that finds one costs on top.
    ///
    /// A and B interleaved in one run, on the same sixteen resting bodies: the
    /// first with materials that hold, the second with materials that give way
    /// at any touch, so every contact in the scene raises an event and hands
    /// impulse back. The second is far past anything a game would do -- it is
    /// the ceiling, not a case.
    ///
    /// Run with `cargo test --release -p bevox_core --lib fracture_is_timed --
    /// --ignored --nocapture`.
    #[test]
    #[ignore]
    fn fracture_is_timed() {
        let world = slab(64, 0..8);
        let field = DistanceField::build(&world);
        let holds = materials();
        let mut gives = MaterialTable::new();
        for i in 1..=7u8 {
            let mut m = holds.get(MaterialId(i));
            m.strength = 0.0;
            gives.push(m).unwrap();
        }

        let scene = || {
            (0..16)
                .map(|i| {
                    let (x, z) = ((i % 4) as f32 * 6.0 + 20.0, (i / 4) as f32 * 6.0 + 20.0);
                    let mut b = placed(cube(4, 4), Vec3::new(x, 10.0, z), Quat::IDENTITY);
                    recompute(&mut b, &holds);
                    b
                })
                .collect::<Vec<_>>()
        };

        let time = |materials: &MaterialTable| {
            let mut bodies = scene();
            // Settle first, so the timing is a steady tick rather than a drop.
            for _ in 0..30 {
                step(&mut bodies, &world, &field, materials, Air::VACUUM, DT, None, &mut []);
            }
            let mut ticks = Vec::new();
            let mut broke = 0;
            for _ in 0..200 {
                // Awake on both sides of the comparison. Bodies that keep
                // breaking never settle, so leaving the others asleep would
                // time sleeping against solving and call the difference
                // fracture.
                for b in bodies.iter_mut() {
                    b.asleep = false;
                }
                let at = std::time::Instant::now();
                let out =
                    step(&mut bodies, &world, &field, materials, Air::VACUUM, DT, None, &mut []);
                ticks.push(at.elapsed().as_secs_f64() * 1000.0);
                broke += out.fractures.len();
            }
            ticks.sort_by(f64::total_cmp);
            (ticks[ticks.len() / 2], broke)
        };

        let (a1, none) = time(&holds);
        let (b, raised) = time(&gives);
        let (a2, _) = time(&holds);
        println!(
            "16 bodies resting but awake, median of 200 ticks: holds {a1:.4} / {a2:.4} ms ({none} \
             fractures), gives way {b:.4} ms ({raised} fractures)"
        );
        assert_eq!(none, 0, "the materials that hold broke {none} times");
        assert!(raised > 0, "the materials that give way broke nothing");
    }

    /// A lone voxel nudged along the floor comes to rest, and then sleeps.
    ///
    /// A lone voxel classifies as a `Corner`, which is a sphere: it rolls, and
    /// a rolling contact barely slips, so sliding friction never stops it.
    /// Before `ROLLING` it was still moving at 3.3 voxels a second after eight
    /// seconds and `still_for` never left zero, so it never slept and never
    /// merged. The same goes for a tower one voxel wide, whose voxels are
    /// cylinders standing on a sphere.
    ///
    /// The ice half is the discriminating one: the resistance comes from the
    /// contact's friction, so a voxel spinning on frictionless ice must keep
    /// spinning. A damper that ignored friction would stop that too and pass
    /// every other assertion here.
    ///
    /// It has to be a *spin* on the ice rather than a roll: with no friction
    /// the voxel slides without turning, and a test of sliding cannot tell a
    /// contact force from a global one.
    #[test]
    fn a_lone_voxel_stops_rolling_and_sleeps() {
        let world = slab(64, 0..8);
        let ice = slab_of(64, 0..8, MaterialId(3));
        let field = DistanceField::build(&world);
        let materials = materials();
        let tower = {
            let voxels: Vec<_> = (0..6u32).map(|y| (UVec3::new(0, y, 0), MaterialId(1))).collect();
            Contree::from_voxels(16, &voxels)
        };

        let roll = |floor: &Contree, volume: Contree, seconds: f32| {
            let mut body = placed(volume, Vec3::new(32.0, 9.0, 32.0), Quat::IDENTITY);
            assert!(recompute(&mut body, &materials));
            body.velocity = Vec3::new(4.0, 0.0, 0.0);
            let mut bodies = vec![body];
            for _ in 0..(seconds / DT) as usize {
                step(&mut bodies, floor, &field, &materials, Air::EARTH, DT, None, &mut []);
            }
            (bodies[0].velocity.length(), bodies[0].asleep)
        };

        let (speed, asleep) = roll(&world, cube_of(1, 4, MaterialId(1)), 3.0);
        assert!(asleep, "a lone voxel was still awake after three seconds, moving at {speed}");
        let (speed, asleep) = roll(&world, tower, 5.0);
        assert!(asleep, "a one-wide tower was still awake after five seconds, moving at {speed}");

        let mut body = placed(cube_of(1, 4, MaterialId(1)), Vec3::new(32.0, 8.5, 32.0), Quat::IDENTITY);
        assert!(recompute(&mut body, &materials));
        let spun = 6.0;
        body.angular_momentum = (world_inverse_inertia(&body).inverse()) * Vec3::new(0.0, spun, 0.0);
        let mut bodies = vec![body];
        for _ in 0..(3.0 / DT) as usize {
            step(&mut bodies, &ice, &field, &materials, Air::EARTH, DT, None, &mut []);
        }
        let left = bodies[0].angular_velocity().length();
        assert!(
            left > spun * 0.9,
            "a voxel spinning on frictionless ice slowed from {spun} to {left}: the resistance              is not coming from the contact's friction"
        );
    }

    /// A force acts for the tick it was given for, and then it is spent.
    ///
    /// Held down for a second it changes speed by `f/m`; given once it is gone
    /// the next tick. A force that was never cleared would accelerate a body
    /// for ever from a single push, which is the bug this shape prevents.
    #[test]
    fn a_force_acts_for_its_tick_and_no_longer() {
        let world = slab(64, 0..8);
        let field = DistanceField::build(&world);
        let materials = materials();
        let body = || {
            let mut b = placed(cube(4, 4), Vec3::new(32.0, 40.0, 32.0), Quat::IDENTITY);
            assert!(recompute(&mut b, &materials));
            b
        };

        // Held for a second, in free flight, with nothing else acting.
        let mut bodies = vec![body()];
        let push = bodies[0].mass.mass * 10.0;
        for _ in 0..(1.0 / DT) as usize {
            bodies[0].add_force(Vec3::new(push, 0.0, 0.0));
            step(&mut bodies, &world, &field, &materials, Air::STILL, DT, None, &mut []);
        }
        let held = bodies[0].velocity.x;
        assert!(
            (held - 10.0).abs() < 0.1,
            "a force of ten times the mass, held for a second, left it at {held} not 10"
        );

        // Given once, then nothing.
        let mut bodies = vec![body()];
        bodies[0].add_force(Vec3::new(push, 0.0, 0.0));
        step(&mut bodies, &world, &field, &materials, Air::STILL, DT, None, &mut []);
        let after_one = bodies[0].velocity.x;
        for _ in 0..20 {
            step(&mut bodies, &world, &field, &materials, Air::STILL, DT, None, &mut []);
        }
        assert!(
            (bodies[0].velocity.x - after_one).abs() < 1e-4,
            "one push kept pushing: {after_one} became {}",
            bodies[0].velocity.x
        );
        assert!(after_one > 0.0, "the push did nothing at all");
    }

    /// The ceiling is a speed, so it is the same speed at every tick rate.
    ///
    /// As voxels per tick it was 256 at 64 Hz and 512 at 128: changing the
    /// rate changed how fast anything in the game could go.
    #[test]
    fn the_ceiling_is_the_same_speed_at_every_rate() {
        let world = slab(64, 0..8);
        let field = DistanceField::build(&world);
        let materials = materials();
        for hz in [32.0f32, 64.0, 128.0, 240.0] {
            let dt = 1.0 / hz;
            let mut body = placed(cube(4, 4), Vec3::new(32.0, 200.0, 32.0), Quat::IDENTITY);
            assert!(recompute(&mut body, &materials));
            body.velocity = Vec3::new(0.0, -10_000.0, 0.0);
            let mut bodies = vec![body];
            step(&mut bodies, &world, &field, &materials, Air::STILL, dt, None, &mut []);
            let capped = bodies[0].velocity.length();
            assert!(
                (capped - MAX_SPEED).abs() < 1.0,
                "at {hz} Hz the ceiling is {capped}, not {MAX_SPEED}"
            );
        }
    }

    /// The same scene at four tick rates lands in about the same place.
    ///
    /// A discrete solver is not tick-invariant and this does not pretend
    /// otherwise: it pins a **bound**, measured before the physics crate
    /// existed, so a later change cannot widen the spread without saying so.
    /// What was measured then: a slide stopping within 0.38% across 32 to 240
    /// Hz, and a fall landing within 0.02%.
    ///
    /// The spread is also required to be non-zero, because a solver that
    /// ignored `dt` entirely would pass a bound and be far more wrong.
    #[test]
    fn the_same_scene_lands_alike_at_every_tick_rate() {
        let world = slab(64, 0..8);
        let field = DistanceField::build(&world);
        let materials = materials();
        let at = |dt: f32, seconds: f32, setup: &dyn Fn(&mut Body)| {
            let mut body = placed(cube(4, 4), Vec3::new(20.0, 10.0, 32.0), Quat::IDENTITY);
            assert!(recompute(&mut body, &materials));
            setup(&mut body);
            let mut bodies = vec![body];
            for _ in 0..(seconds / dt) as usize {
                step(&mut bodies, &world, &field, &materials, Air::EARTH, dt, None, &mut []);
            }
            bodies[0].position
        };

        let rates = [32.0f32, 64.0, 128.0, 240.0];
        let nudge = |b: &mut Body| b.velocity = Vec3::new(30.0, 0.0, 0.0);
        let slides: Vec<f32> = rates.iter().map(|&hz| at(1.0 / hz, 4.0, &nudge).x).collect();
        let rests: Vec<f32> = rates.iter().map(|&hz| at(1.0 / hz, 3.0, &|_| {}).y).collect();
        // A tenth of a second in, while it is genuinely still sliding. Resting
        // positions alone are blind to time running at the wrong rate: halve
        // every substep and the body traces the same path, only slower, and
        // every resting place is exactly where it always was.
        let midway: Vec<f32> = rates.iter().map(|&hz| at(1.0 / hz, 0.1, &nudge).x).collect();

        let spread = |v: &[f32]| {
            let (lo, hi) = v.iter().fold((f32::MAX, f32::MIN), |(l, h), &x| (l.min(x), h.max(x)));
            (hi - lo) / (v.iter().sum::<f32>() / v.len() as f32).abs()
        };
        let (slide, rest, half) = (spread(&slides), spread(&rests), spread(&midway));
        eprintln!(
            "slide stop {slides:?} -> {:.3}%, rest {rests:?} -> {:.4}%, at 0.1s {midway:?} -> {:.3}%",
            slide * 100.0,
            rest * 100.0,
            half * 100.0
        );
        assert!(
            half < 0.02,
            "a tenth of a second in, the four rates are {:.2}% apart: the simulation is not              advancing at the same rate per second",
            half * 100.0
        );
        assert!(slide < 0.01, "the slide's stopping point spreads {:.2}% across rates", slide * 100.0);
        assert!(rest < 0.001, "the resting height spreads {:.3}% across rates", rest * 100.0);
        assert!(
            slide > 0.0,
            "every rate stopped the slide at exactly the same place, which a discrete solver \
             cannot do: this is measuring nothing"
        );
    }

    /// A long fall eases into its terminal speed instead of having its
    /// acceleration switched off.
    ///
    /// The old hard cap was visible from the ground: a body accelerated for
    /// 0.82 seconds, reached 80 voxels a second exactly, and fell the rest of
    /// the way at a flat rate. Drag gives the asymptote that falling actually
    /// has -- always some acceleration left, never quite at the limit.
    ///
    /// Dropped where nothing can be hit, so this is the integrator alone.
    #[test]
    fn a_long_fall_eases_into_terminal_speed() {
        let world = slab(64, 0..8);
        let field = DistanceField::build(&world);
        let materials = materials();
        let mut body = placed(cube(4, 4), Vec3::new(32.0, 4000.0, 32.0), Quat::IDENTITY);
        assert!(recompute(&mut body, &materials));
        let mut bodies = vec![body];

        let (mut speeds, mut fastest_gain) = (Vec::new(), 0.0f32);
        let mut last = 0.0f32;
        for _ in 0..(3.0 / DT) as usize {
            step(&mut bodies, &world, &field, &materials, Air::EARTH, DT, None, &mut []);
            let speed = -bodies[0].velocity.y;
            fastest_gain = fastest_gain.max(speed - last);
            speeds.push(speed);
            last = speed;
        }

        let terminal = *speeds.last().unwrap();
        assert!(
            terminal > TERMINAL_SPEED * 0.9 && terminal < TERMINAL_SPEED,
            "after three seconds the fall is at {terminal}, not approaching {TERMINAL_SPEED} \
             from below"
        );
        assert!(
            speeds.windows(2).all(|w| w[1] > w[0]),
            "the fall stopped accelerating somewhere: something is clamping it"
        );
        // A clamp shows up here: every gain equal until one is zero. Drag makes
        // each gain smaller than the last.
        let gains: Vec<f32> = speeds.windows(2).map(|w| w[1] - w[0]).collect();
        assert!(
            gains.windows(2).all(|w| w[1] <= w[0] + 1e-4),
            "the acceleration rose again partway down, which drag cannot do"
        );
        assert!(
            *gains.last().unwrap() < fastest_gain * 0.25,
            "the last tick gained {} against a first of {fastest_gain}: the fall is still in \
             free flight after three seconds",
            gains.last().unwrap()
        );
        assert!(
            terminal < MAX_SPEED,
            "the fall reached {terminal} and the cap is {}: a falling body is being clamped, \
             which is what drag is here to prevent",
            MAX_SPEED
        );
    }

    /// One body of `material` driven straight down into a floor at `speed`,
    /// simulated for a sixth of a second at `dt` a tick. Returns every fracture
    /// the collision raised and how fast the body was still falling at the end.
    ///
    /// No gravity: the approach speed is the one thing under test, and letting
    /// gravity add to it would make the collision depend on how long the flight
    /// took -- which is exactly what the tick-rate gate must not measure.
    fn slam(material: MaterialId, speed: f32, dt: f32) -> (Vec<Fracture>, f32) {
        let world = slab(64, 0..8);
        let field = DistanceField::build(&world);
        let materials = materials();
        // Inside the contact margin already, so the collision lands in the
        // first tick. Further out, the speculative contact appears a tick early
        // and bleeds the approach speed away over two ticks, which halves the
        // blow and is a property of the detector rather than of fracture.
        let mut body = placed(cube_of(4, 4, material), Vec3::new(32.0, 11.0, 32.0), Quat::IDENTITY);
        assert!(recompute(&mut body, &materials));
        body.velocity = Vec3::new(0.0, -speed, 0.0);
        let mut bodies = vec![body];
        let mut fractures = Vec::new();
        let mut left = 0.0;
        for tick in 0..(1.0 / (6.0 * dt)).round() as usize {
            let out = step(&mut bodies, &world, &field, &materials, Air::STILL, dt, None, &mut []);
            fractures.extend(out.fractures);
            // The collision is the first tick, and what it left the body doing
            // is the question. Later ticks only settle it, and with no gravity
            // they settle everything to the same near-zero.
            if tick == 0 {
                left = -bodies[0].velocity.y;
            }
        }
        (fractures, left)
    }

    /// Hit something hard enough and it breaks; land gently and it does not.
    #[test]
    fn a_hard_landing_breaks_and_a_soft_one_does_not() {
        let (hard, _) = slam(MaterialId(6), 120.0, DT);
        assert!(!hard.is_empty(), "a body driven into the floor at 120 broke nothing");
        assert!(
            hard.iter().any(|f| f.body.is_some()),
            "the floor broke but the thing that hit it did not"
        );

        let (soft, _) = slam(MaterialId(6), 1.0, DT);
        assert!(soft.is_empty(), "a body arriving at 1 voxel a second broke {} things", soft.len());
    }

    /// The same collision, twice, differing only in what the body is made of.
    #[test]
    fn what_breaks_depends_on_the_material() {
        let (brittle, _) = slam(MaterialId(6), 60.0, DT);
        let (tough, _) = slam(MaterialId(7), 60.0, DT);
        assert!(
            brittle.iter().any(|f| f.body.is_some()),
            "the brittle body survived a blow at 60"
        );
        assert!(
            !tough.iter().any(|f| f.body.is_some()),
            "the unbreakable body broke, so strength is not being read"
        );
    }

    /// A narrow hammer of unbreakable material driven straight down onto
    /// `target`, which sits on an unbreakable floor. Returns every fracture,
    /// the largest blow any contact carried, and how fast the hammer was still
    /// descending when the strike tick ended.
    ///
    /// The hammer is unbreakable, which is what makes that last number worth
    /// reading: it never comes apart, so the body whose speed is measured is
    /// the same body before and after, and what changed it is the contact
    /// alone.
    ///
    /// The floor takes the load, so what a contact reads is the hammer's
    /// momentum rather than the target's own mass. That is the whole point of
    /// the fixture: a small target and a large one can be struck *identically*,
    /// which is impossible when the target is the thing supplying the momentum
    /// -- a one-voxel body exchanges a sixty-fourth of what a four-cube does at
    /// the same speed, which would swamp any size term.
    ///
    /// No gravity: the strike is the only thing under test.
    fn hammered(target: Contree, speed: f32) -> (Vec<Fracture>, f32, f32) {
        let world = slab_of(64, 0..8, MaterialId(7));
        let field = DistanceField::build(&world);
        let materials = materials();
        // The struck voxel spans the same world box whatever the target's size:
        // both fixtures are placed so that the voxel under the hammer is
        // 31.5..32.5 across and 8..9 up. Contact geometry is therefore not what
        // separates the two runs.
        let at = Vec3::new(32.0, 8.5, 32.0);
        let mut bodies = vec![
            placed(target, at, Quat::IDENTITY),
            // A one-voxel footprint, so it strikes exactly one voxel of either
            // target. Just inside the contact margin, so the strike lands on
            // the first tick instead of being bled over two.
            placed(block(1, 4, 1, 4, MaterialId(7)), Vec3::new(32.0, 11.05, 32.0), Quat::IDENTITY),
        ];
        bodies[1].velocity = Vec3::new(0.0, -speed, 0.0);

        let mut fractures = Vec::new();
        let mut hardest = 0.0f32;
        // The strike is the first tick; later ticks only settle what it left.
        let mut left = 0.0;
        for tick in 0..8 {
            let out =
                step(&mut bodies, &world, &field, &materials, Air::STILL, DT, None, &mut []);
            hardest = hardest.max(out.peak_impulse);
            fractures.extend(out.fractures);
            if tick == 0 {
                left = -bodies[1].velocity.y;
            }
        }
        (fractures, hardest, left)
    }

    /// Dwyer's rule, and the reason it exists: a small thing breaks under a
    /// blow a large thing of the same material shrugs off.
    ///
    /// Both targets are glass and both are struck by the same hammer at the
    /// same speed on the same voxel. The only difference is how many voxels the
    /// struck voxel's body has -- one against twenty-seven, which is `SIZE_CAP`
    /// and so the full span of the size term: a third of `GLASS_STRENGTH`
    /// against the whole of it, 116,667 against 350,000.
    #[test]
    fn a_small_body_breaks_before_a_large_one() {
        // A 1x1x27 bar rather than a 3x3x3 cube: the cube would present a nine
        // voxel face to the hammer and split the blow nine ways, which would
        // make the large target survive for a reason that has nothing to do
        // with its size. The bar's middle voxel is the only one struck, and it
        // sits exactly where the small target's single voxel does.
        let (small, small_blow, _) = hammered(cube_of(1, 4, MaterialId(6)), HAMMER_SPEED);
        let (large, large_blow, _) = hammered(size_cap_bar(), HAMMER_SPEED);
        // Not equal blows -- the bar's own mass means the same hammer leaves a
        // *larger* impulse on it, 638,388 against 387,507 -- and that is the
        // stronger statement: the large target survives a harder blow than the
        // one that broke the small one, so what saved it cannot be a gentler
        // hit. The only way to pass this with a flat threshold is for neither
        // to break, which the first assertion forbids.
        assert!(
            large_blow >= small_blow,
            "the large target was struck at {large_blow:.0} and the small one at \
             {small_blow:.0}: it survived for want of a blow, not for its size"
        );
        assert!(
            small.iter().any(|f| f.body.is_some()),
            "a one-voxel glass body took a blow of {small_blow:.0} against a third of the {} \
             its material holds at `SIZE_CAP`, and did not break",
            crate::fixtures::GLASS_STRENGTH
        );
        assert!(
            !large.iter().any(|f| f.body.is_some()),
            "a twenty-seven-voxel glass body broke under the same blow of {large_blow:.0}: the \
             size term is not reaching the threshold"
        );
    }

    /// A one-voxel-thick bar of exactly `SIZE_CAP` glass voxels: the smallest
    /// body the size term treats as large, so its threshold is the material's
    /// own strength unscaled.
    ///
    /// Derived from the constant rather than written out, so a retune of
    /// `SIZE_CAP` carries the fixture with it instead of silently leaving a
    /// body that no longer spans the term.
    fn size_cap_bar() -> Contree {
        assert!(
            fracture::SIZE_CAP % 2 == 1 && fracture::SIZE_CAP <= 64,
            "a `SIZE_CAP` of {} does not fit this fixture: an even one leaves the bar no \
             middle voxel, and the claim that the struck voxel sits exactly where the \
             single-voxel target's does depends on there being one",
            fracture::SIZE_CAP
        );
        block(1, 1, fracture::SIZE_CAP, 64, MaterialId(6))
    }

    /// A strike that passes the threshold and almost nothing more: `over` is
    /// 1.069, a blow of 373,976 against `GLASS_STRENGTH`.
    const BARELY_OVER_SPEED: f32 = 100.0;

    /// A strike far past it: `over` is 2.687, a blow of 940,572.
    ///
    /// The two speeds bracket 2.5, which is where a fixed 0.6 hand-back and
    /// the excess coincide -- `1 - 1/2.5 == 0.6`. A fixed fraction is therefore
    /// too generous on one of these strikes and too mean on the other, and
    /// cannot be right about both.
    const FAR_OVER_SPEED: f32 = 250.0;

    /// The hand-back is the excess, so it scales with how far past strength the
    /// blow went: a hammer barely over the threshold is left nearly stopped,
    /// and one far over carries most of its speed on through.
    ///
    /// This is what a fixed fraction cannot do, and it is the gate the fixed
    /// fraction `REBOUND = 0.6` had to fail. `breaking_something_does_not_stop_
    /// you` only asks that *something* comes back, which 0.6 satisfied for the
    /// whole life of the old rule; this asks that the amount be the right one
    /// for the collision.
    ///
    /// Both strikes use the same unbreakable hammer on the same `SIZE_CAP` bar,
    /// so the only thing that differs is how far over strength the blow landed.
    /// The hammer never breaks, so the body being measured is the same body
    /// throughout and nothing detaches out from under the reading.
    ///
    /// Read as a fraction of the approach speed rather than as an impulse: the
    /// two strikes carry different momentum, and the question is what share of
    /// it survives the contact.
    #[test]
    fn the_hand_back_scales_with_how_far_past_strength_the_blow_went() {
        let (barely, barely_blow, barely_left) = hammered(size_cap_bar(), BARELY_OVER_SPEED);
        let (far, far_blow, far_left) = hammered(size_cap_bar(), FAR_OVER_SPEED);
        let over_of = |f: &[Fracture]| f.iter().map(|f| f.over).fold(0.0f32, f32::max);
        let (barely_over, far_over) = (over_of(&barely), over_of(&far));

        // Both must break, or there is no hand-back to measure.
        assert!(
            barely_over > 1.0,
            "the gentler strike left {barely_blow:.0} and broke nothing: it is under the \
             threshold, so this gate is measuring two hand-backs of zero"
        );
        assert!(
            far_over > 2.5 && barely_over < 2.5,
            "the two strikes read {barely_over:.3} and {far_over:.3} times over strength and \
             do not bracket 2.5, where a fixed 0.6 and the excess agree: a fixed fraction \
             could sit between them and pass"
        );

        let (barely_share, far_share) =
            (barely_left / BARELY_OVER_SPEED, far_left / FAR_OVER_SPEED);
        assert!(
            barely_share < 0.08,
            "a hammer {barely_over:.3} times over the threshold kept {:.1}% of its speed \
             ({barely_left:.2} of {BARELY_OVER_SPEED}): breaking something it barely had the \
             momentum to break cost it almost nothing, so the hand-back is a fixed share \
             rather than the excess",
            barely_share * 100.0
        );
        assert!(
            far_share > 0.35,
            "a hammer {far_over:.3} times over the threshold kept only {:.1}% of its speed \
             ({far_left:.2} of {FAR_OVER_SPEED}), blow {far_blow:.0}: a blow far past what the \
             material could take is being charged most of itself for the break",
            far_share * 100.0
        );
        assert!(
            far_share > barely_share * 10.0,
            "the far strike kept {:.1}% of its speed and the barely-over one {:.1}%: the \
             hand-back is not scaling with how far past strength the blow went, which is \
             what a fixed fraction looks like",
            far_share * 100.0,
            barely_share * 100.0
        );
    }

    /// Chosen so both blows land between a single voxel's threshold and a
    /// capped body's, which is what leaves the size term as the only thing that
    /// can decide the outcome: the small target reads 164,915 against 116,667
    /// and the large one 260,656 against 350,000.
    const HAMMER_SPEED: f32 = 70.0;

    /// Dwyer's devlog 28, and the reason the threshold is not on force: what
    /// breaks must not depend on how often the physics ticks.
    ///
    /// The discriminating half is the second one. A blow **under** the
    /// material's strength must leave it whole at every rate, and a threshold
    /// read as a force cannot manage that: dividing by the tick makes the same
    /// collision look sixty-four times worse at 64 Hz and a hundred and
    /// twenty-eight times at 128, so it shatters everything at every rate.
    ///
    /// The blow is the accumulated contact impulse, and the gate asks only that
    /// the *outcome* match, not the number: the peak impulse of one collision
    /// is close at the two rates but not equal -- 1,047,814 at 64 Hz against
    /// 968,836 at 128 for the 60-voxel slam, because a collision the detector
    /// sees coming is spread over more ticks at a finer rate. Both land on the
    /// same side of every strength in the table, which is what matters. A
    /// force would not: dividing by the tick moves it by the ratio of the
    /// rates.
    #[test]
    fn the_same_collision_breaks_at_any_tick_rate() {
        let broke = |speed: f32, dt: f32| !slam(MaterialId(6), speed, dt).0.is_empty();
        assert!(broke(60.0, DT), "a blow well over strength did not break at 64 Hz");
        assert!(broke(60.0, DT / 2.0), "the same blow did not break at 128 Hz");
        assert!(!broke(8.0, DT), "a blow under strength broke something at 64 Hz");
        assert!(
            !broke(8.0, DT / 2.0),
            "a blow under strength broke something at 128 Hz but not at 64: the threshold is \
             reading a force, which is the one thing that changes with the tick"
        );
    }

    /// Breaking through something costs less speed than bouncing off it.
    #[test]
    fn breaking_something_does_not_stop_you() {
        let (_, through) = slam(MaterialId(6), 120.0, DT);
        let (_, off) = slam(MaterialId(7), 120.0, DT);
        assert!(
            through > off + 1.0,
            "a body that broke what it hit was still falling at {through} and one that did not at {off}: \
             the impulse is not being handed back, so the pieces fall out of a hole nothing \
             went through"
        );
    }

    /// One collision between two bodies, run with the two listed in either
    /// order, breaks the same things by the same amount.
    ///
    /// A contact belongs to whichever body the scene lists first, which has
    /// nothing to do with which of them struck the other. Reading the blow as
    /// the contact's impulse divided by *that* body's mass would therefore make
    /// the answer depend on the list order: the impulse two bodies exchange is
    /// set by their reduced mass, so naming the light one reports nearly the
    /// whole closing speed and naming the heavy one a twenty-fourth of it. The
    /// same glass cube would then shatter or survive according to where in a
    /// `Vec` it sat.
    ///
    /// The raw impulse is symmetric, so there is no owner to pick.
    #[test]
    fn a_collision_breaks_the_same_things_whichever_body_is_listed_first() {
        let materials = materials();
        let world = Contree::empty(3);
        let field = DistanceField::build(&world);

        // A brittle cube thrown at a heavier one. The gap is well outside the
        // contact margin, so the collision happens during the run rather than
        // being resolved away as a speculative contact on the first substep.
        let collide = |swapped: bool| -> Vec<Fracture> {
            let anvil =
                placed(cube_of(4, 4, MaterialId(2)), Vec3::new(32.0, 32.0, 32.0), Quat::IDENTITY);
            let mut pebble =
                placed(cube_of(2, 4, MaterialId(6)), Vec3::new(32.0, 40.0, 32.0), Quat::IDENTITY);
            pebble.velocity = Vec3::new(0.0, -100.0, 0.0);
            let mut bodies =
                if swapped { vec![pebble, anvil] } else { vec![anvil, pebble] };
            let mut broke = Vec::new();
            for _ in 0..16 {
                let out =
                    step(&mut bodies, &world, &field, &materials, Air::STILL, DT, None, &mut []);
                broke.extend(out.fractures);
            }
            broke
        };

        let (heavy_first, light_first) = (collide(false), collide(true));
        let hardest = |f: &[Fracture]| f.iter().map(|f| f.blow).fold(0.0f32, f32::max);
        assert!(
            !heavy_first.is_empty(),
            "a brittle cube thrown at 100 broke nothing with the heavy body listed first; the \
             blow is being divided by that body's mass"
        );
        assert!(!light_first.is_empty(), "it broke nothing with the light body listed first");
        // Relative, because the blow is an impulse in the hundreds of
        // thousands: one part in a thousand of it is the same statement that an
        // absolute voxel a second was of a speed.
        let (a, b) = (hardest(&heavy_first), hardest(&light_first));
        assert!(
            (a - b).abs() < 1e-3 * a.max(b),
            "the same collision read as {a:.1} one way round and {b:.1} the other"
        );
        assert!(
            a > crate::fixtures::GLASS_STRENGTH,
            "the blow came out at {a:.1}, at or under the {} it had to exceed to be reported \
             at all, so this is reading something other than the impulse",
            crate::fixtures::GLASS_STRENGTH
        );
    }

    /// Two bodies of different strength in one collision: the weaker gives way
    /// and the stronger does not, and the fracture reports an `over` measured
    /// against its *own* threshold.
    ///
    /// Flori read this as a bug, and it is not one. A contact is one impulse
    /// shared by a pair, but a threshold belongs to a voxel in a body, so one
    /// blow tested twice can come out two ways -- which is the whole reason
    /// `break_what_gave_way` asks both sides rather than picking an owner. The
    /// gate is what makes that explicable instead of surprising.
    #[test]
    fn the_weaker_side_breaks_and_the_stronger_does_not() {
        let materials = materials();
        let world = Contree::empty(3);
        let field = DistanceField::build(&world);

        // The same collision as the list-order gate: a glass pebble thrown at a
        // heavier cube of the default material, which is eight times stronger.
        let anvil =
            placed(cube_of(4, 4, MaterialId(2)), Vec3::new(32.0, 32.0, 32.0), Quat::IDENTITY);
        let mut pebble =
            placed(cube_of(2, 4, MaterialId(6)), Vec3::new(32.0, 40.0, 32.0), Quat::IDENTITY);
        pebble.velocity = Vec3::new(0.0, -100.0, 0.0);
        let (strong, weak_id) = (anvil.id, pebble.id);
        let mut bodies = vec![anvil, pebble];

        let mut broke = Vec::new();
        for _ in 0..16 {
            let out = step(&mut bodies, &world, &field, &materials, Air::STILL, DT, None, &mut []);
            broke.extend(out.fractures);
        }

        assert!(
            broke.iter().any(|f| f.body == Some(weak_id)),
            "the glass pebble survived a collision it reported {} fractures from",
            broke.len()
        );
        assert!(
            !broke.iter().any(|f| f.body == Some(strong)),
            "the stronger body broke too, so the threshold is not being read per side"
        );

        // The pebble's eight voxels put its threshold at two thirds of
        // `GLASS_STRENGTH`, and `over` is the blow over *that* -- not over the
        // anvil's strength, and not over the raw table figure.
        let want = crate::fixtures::GLASS_STRENGTH * fracture::size_factor(8);
        for f in broke.iter().filter(|f| f.body == Some(weak_id)) {
            let over = f.blow / want;
            assert!(
                (f.over - over).abs() < 1e-4 * over,
                "a fracture reported {} times over on a blow of {:.0}, and its own threshold of \
                 {want:.0} makes that {over}",
                f.over,
                f.blow
            );
            assert!(f.over > 1.0, "a reported fracture was not past its threshold");
        }
    }

    /// Weight alone breaks nothing, however long it leans.
    ///
    /// The binding constraint on every strength in the table, because the blow
    /// is an impulse and an impulse grows with the mass a contact holds up:
    /// warm starting seeds each tick from the last, so a stack carries a large
    /// one while doing nothing much. This stack's bottom contact reaches a
    /// measured 331,306, which is why `GLASS_STRENGTH` -- the weakest material
    /// in the fixture table -- is 350,000 and not the 20 it was when the blow
    /// was a closing speed. At 20 this fails by 61,134 fractures.
    ///
    /// **The 331,306 is the stack *landing*, not its weight.** It peaks at
    /// tick 5: the cubes start 0.2 voxels above where they settle, and
    /// `peak[at]` counts the push-out velocity the bias adds. The load the
    /// stack then holds is 18,798, seventeen times less. See
    /// `what_the_tick_rate_does_to_the_blow` and
    /// `docs/concepts/fracture-load-window.md`.
    ///
    /// Glass deliberately: the weakest material makes this the binding case.
    #[test]
    fn resting_weight_breaks_nothing() {
        let materials = materials();
        let world = slab_of(64, 0..8, MaterialId(6));
        let field = DistanceField::build(&world);
        let mut bodies: Vec<Body> = (0..3)
            .map(|i| {
                placed(
                    cube_of(4, 4, MaterialId(6)),
                    Vec3::new(32.0, 10.2 + i as f32 * 4.0, 32.0),
                    Quat::IDENTITY,
                )
            })
            .collect();

        let mut broke = Vec::new();
        for _ in 0..1200 {
            let out =
                step(&mut bodies, &world, &field, &materials, Air::VACUUM, DT, None, &mut []);
            broke.extend(out.fractures);
        }
        assert_eq!(bodies.len(), 3, "the stack came apart");
        assert!(
            broke.is_empty(),
            "a glass stack standing still broke {} things, the heaviest blow {:.3} against a \
             strength of {}",
            broke.len(),
            broke.iter().map(|f| f.blow).fold(0.0f32, f32::max),
            crate::fixtures::GLASS_STRENGTH,
        );
    }

    /// Pressing a held body into brittle material breaks it, however slowly it
    /// is pressed.
    ///
    /// The symptom this whole rule change exists for. The mouse grab is a joint
    /// with an enormous force limit driving toward a *velocity* goal, so a
    /// grabbed body leaning on something presses with up to `GRAB_MAX_FORCE`
    /// while the contact holds it still: the two surfaces are closing at
    /// essentially nothing. A threshold on closing speed therefore reads a
    /// slow crush as no blow at all, and a player can lean a rock through a
    /// window without marking it.
    #[test]
    fn a_slow_crush_breaks_what_it_presses() {
        let materials = materials();
        // Glass floor, unbreakable body: only one of the two can give way, so
        // what breaks is not in question.
        let world = slab_of(64, 0..8, MaterialId(6));
        let field = DistanceField::build(&world);
        let centre = Vec3::new(32.0, 10.0, 32.0);
        let mut bodies = vec![placed(cube_of(4, 4, MaterialId(7)), centre, Quat::IDENTITY)];
        assert!(recompute(&mut bodies[0], &materials));
        let mut grab = Joint::grab(&bodies[0], centre);

        let mut broke = Vec::new();
        let mut fastest = 0.0f32;
        for tick in 0..400 {
            // The target creeps down into the floor: 0.002 a tick is an eighth
            // of a voxel a second, far below anything that reads as an impact.
            grab.anchor_b = centre - Vec3::Y * (tick as f32 * 0.002);
            // Before the step, and only while nothing has broken: the tick
            // that breaks the floor hands impulse back and drops the body into
            // the hole, and neither says how gently it was pressing.
            if broke.is_empty() {
                fastest = fastest.max(bodies[0].velocity.length());
            }
            let out = step(
                &mut bodies,
                &world,
                &field,
                &materials,
                Air::VACUUM,
                DT,
                Some(&mut grab),
                &mut [],
            );
            broke.extend(out.fractures);
        }

        assert!(
            fastest < 1.0,
            "the body moved at {fastest} voxels a second before anything broke, so this is an \
             impact and not a crush"
        );
        assert!(
            !broke.is_empty(),
            "a body pressed into glass with up to {} of force, at under {fastest:.3} voxels a \
             second, broke nothing",
            crate::GRAB_MAX_FORCE
        );
        assert!(
            broke.iter().any(|f| f.body.is_none()),
            "something broke, but not the glass floor being pressed on"
        );
    }

    #[test]
    fn a_body_below_the_world_is_removed() {
        let world = slab(64, 0..8);
        let field = DistanceField::build(&world);
        let mut bodies = vec![placed(cube(4, 4), Vec3::new(32.0, -60.0, 32.0), Quat::IDENTITY)];
        assert!(step(&mut bodies, &world, &field, &materials(), Air::VACUUM, DT, None, &mut []).rebuild);
        assert!(bodies.is_empty());
        let mut bodies = vec![placed(cube(4, 4), Vec3::new(32.0, 40.0, 32.0), Quat::IDENTITY)];
        assert!(!step(&mut bodies, &world, &field, &materials(), Air::VACUUM, DT, None, &mut []).rebuild);
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
                    step(bodies, &world, &field, &materials, Air::VACUUM, DT, None, &mut []);
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

    /// An arbitrary box of voxels. `cube`/`cube_of` from the shared fixtures
    /// can only build actual cubes, and the 240:1 load needs a one-voxel-thick
    /// plate.
    fn block(nx: u32, ny: u32, nz: u32, extent: u32, material: MaterialId) -> Contree {
        let mut voxels = Vec::new();
        for z in 0..nz {
            for y in 0..ny {
                for x in 0..nx {
                    voxels.push((UVec3::new(x, y, z), material));
                }
            }
        }
        Contree::from_voxels(extent, &voxels)
    }

    /// `placed`, with the material table passed explicitly. `fixtures::placed`
    /// calls `fixtures::materials()` itself, whose fixed densities cannot
    /// reach a 240:1 ratio inside `u16`.
    fn placed_with(volume: Contree, centre: Vec3, orientation: Quat, materials: &MaterialTable) -> Body {
        let mut body = Body::new(volume, Vec3::ZERO, orientation);
        assert!(recompute(&mut body, materials), "a fixture body must have mass");
        body.position = centre;
        body
    }

    /// The 240:1 load's two materials: density 200 for the base cube and the
    /// plate, 48,000 for the heavy body -- exactly 240x, and inside `u16`. The
    /// ratio comes from density alone, at equal volume, not from extra voxels.
    fn load_materials() -> MaterialTable {
        use bevox_core::material::{DEFAULT_STRENGTH, Material};
        let mut table = MaterialTable::new();
        table
            .push(Material { color: [160, 160, 160, 255], density: 200, friction: 60, restitution: 0, strength: DEFAULT_STRENGTH })
            .unwrap();
        table
            .push(Material { color: [120, 40, 40, 255], density: 48_000, friction: 60, restitution: 0, strength: DEFAULT_STRENGTH })
            .unwrap();
        table
    }

    /// The 240:1 load fixture, on `slab(64, 0..8)`: a `cube(4, 4)` at
    /// `y = 10.2`, a 4x1x4 one-voxel plate at `y = 12.2`, and a 4x1x4 heavy
    /// body at `y = 13.2` whose density gives it 240 times the plate's mass.
    ///
    /// Not a gate fixture -- the original was deleted on 2026-09-28 when it
    /// was found to diverge -- so it is rebuilt here from the description of
    /// what it was. Shared by `what_the_iteration_counts_cost` and
    /// `the_240_to_1_load_collapses_through_the_floor`.
    fn load_fixture(materials: &MaterialTable) -> Vec<Body> {
        vec![
            placed_with(block(4, 4, 4, 4, MaterialId(1)), Vec3::new(32.0, 10.2, 32.0), Quat::IDENTITY, materials),
            placed_with(block(4, 1, 4, 4, MaterialId(1)), Vec3::new(32.0, 12.2, 32.0), Quat::IDENTITY, materials),
            placed_with(block(4, 1, 4, 4, MaterialId(2)), Vec3::new(32.0, 13.2, 32.0), Quat::IDENTITY, materials),
        ]
    }

    /// How low a body's `world_box` may bottom out while it is still standing
    /// on `slab(64, 0..8)`.
    ///
    /// The slab's voxels fill 0..8, so its top surface is y = 8.0, and
    /// `world_box` is a corner-to-corner box around voxel extents: a body
    /// resting on the slab reads **7.5**, half a voxel low, and wobbles to
    /// about 7.31 while still standing. One voxel of it inside the slab is
    /// 7.0, which is 250x `SLOP` below the resting figure and well clear of
    /// that wobble -- a resting contact's own penetration (0.0017) cannot
    /// reach it, and a collapse passes straight through it.
    const FLOOR_BREACH: f32 = 7.0;

    /// Whether the load fixture has left the regime a stability number can be
    /// read from: any body a voxel into the slab, or any body gone --
    /// `step_with`'s first statement deletes anything whose world box falls
    /// below `y = 0`.
    fn breached_the_slab(bodies: &[Body], expected: usize) -> bool {
        bodies.len() != expected
            || bodies
                .iter()
                .any(|b| world_box(b, 0.0).is_some_and(|(min, _)| min.y < FLOOR_BREACH))
    }

    /// **A characterisation of a known defect, not a property anyone wants.**
    /// The 240:1 load does not settle: it falls through `slab(64, 0..8)`.
    /// This test asserts *today's* behaviour so the defect survives a clone
    /// and is reproducible from the repository rather than from a deleted
    /// trace -- the number in the write-up was previously only recorded in
    /// `.superpowers/`, which is git-ignored.
    ///
    /// **It is expected to fail the day the collapse is fixed**, and the fix
    /// is to invert it: assert that the fixture is still standing after 200
    /// ticks and delete the breach numbers. A failure here means the
    /// behaviour moved -- read the new numbers it prints before deciding
    /// whether that is the fix or a different regression.
    ///
    /// `#[ignore]`d: it runs 55 ticks of a three-body stack, which is cheap,
    /// but it is a record of a bug and not a gate the suite should enforce.
    ///
    /// Measured 2026-10-02 at `Tuning::default()`: the breach is at **tick
    /// 55**, with the heavy body falling at **about -22.1 v/s** and the lowest
    /// world box bottomed at **6.59**, a voxel and a half into a slab whose
    /// top surface is 8.0. See
    /// `docs/concepts/solver-convergence-is-a-setting.md`.
    #[test]
    #[ignore]
    fn the_240_to_1_load_collapses_through_the_floor() {
        let materials = load_materials();
        let world = slab(64, 0..8);
        let field = DistanceField::build(&world);
        let mut bodies = load_fixture(&materials);
        let expected = bodies.len();

        let mut breach = None;
        for tick in 1..=200u32 {
            step_with(&mut bodies, &world, &field, &materials, Air::vacuum(GRAVITY), DT, None, &mut [], crate::Tuning::default());
            if breached_the_slab(&bodies, expected) {
                let down = bodies.iter().map(|b| b.velocity.y).fold(f32::INFINITY, f32::min);
                let low = bodies
                    .iter()
                    .filter_map(|b| world_box(b, 0.0))
                    .map(|(min, _)| min.y)
                    .fold(f32::INFINITY, f32::min);
                breach = Some((tick, down, low));
                break;
            }
        }

        let Some((tick, down, low)) = breach else {
            panic!(
                "the 240:1 load stood for 200 ticks: the collapse this test characterises is gone. \
                 If that is the fix, invert this test -- see its doc comment."
            );
        };
        println!("the 240:1 load breached the slab at tick {tick}, falling at {down:.4} v/s, lowest box bottom {low:.4}");

        // A window, not an equality: this is a characterisation, so it pins
        // the behaviour closely enough to notice a change and loosely enough
        // that a float's last bit is not the gate.
        assert!(
            (50..=60).contains(&tick),
            "the collapse moved to tick {tick}, from the 55 recorded on 2026-10-02"
        );
        assert!(
            down <= -20.0,
            "the collapse is at {down} v/s, not the -22.1 recorded on 2026-10-02: it got gentler, which may be a partial fix"
        );
        assert!(
            low < FLOOR_BREACH,
            "breach reported with the lowest box at {low}, which is not below {FLOOR_BREACH}"
        );
    }

    /// The two iteration counts are **1 by measurement**, and something must
    /// fail if that changes.
    ///
    /// `peak[at]`, the accumulated normal impulse `break_what_gave_way` reads,
    /// is taken inside both solve loops, so the counts are an input to every
    /// `strength` in the material table. Raising either one silently rescales
    /// what a fracture threshold means, which is why this is pinned rather
    /// than left to a comment.
    #[test]
    fn the_solver_iteration_counts_are_one() {
        assert_eq!(
            (crate::VELOCITY_ITERATIONS, crate::RELAXATION_ITERATIONS),
            (1, 1),
            "the solver's iteration counts are 1 by measurement, not by default: \
             raising either rescales `peak`, which the fracture threshold reads, \
             so Part 2's `strength` calibration goes with it. \
             Read docs/concepts/solver-convergence-is-a-setting.md before changing this."
        );
    }

    /// What raising the solver's two per-substep sweep counts costs, and what
    /// they do to a known divergence, interleaved across one invocation so
    /// cross-run drift cannot be mistaken for a tuning's effect. Not a gate:
    /// it prints a table.
    ///
    /// **The design does not separate the two counts.** Four of the seven
    /// variants raise `velocity_iterations` while holding
    /// `relaxation_iterations` at 1, so they vary the *ratio* of biased to
    /// unbiased sweeps and not the biased count alone -- the relax pass exists
    /// to remove the velocity the bias added. `(2,2)` and `(4,4)` are the
    /// balanced rows and the only ones that speak about convergence alone.
    /// Read the two axes separately; see
    /// `docs/concepts/solver-convergence-is-a-setting.md`.
    #[test]
    #[ignore]
    fn what_the_iteration_counts_cost() {
        let load_materials = load_materials();

        // The ratio the brief asks for, checked against what the engine's own
        // mass computation produced, not hand arithmetic.
        let sample = load_fixture(&load_materials);
        let (plate_mass, heavy_mass) = (sample[1].mass.mass, sample[2].mass.mass);
        assert!(
            (heavy_mass / plate_mass - 240.0).abs() < 1e-3,
            "the load fixture's mass ratio is {} / {} = {}, not 240",
            heavy_mass,
            plate_mass,
            heavy_mass / plate_mass
        );

        let resting_materials = materials();
        let resting_fixture = || -> Vec<Body> {
            (0..3)
                .map(|i| placed(cube(4, 4), Vec3::new(32.0, 10.2 + i as f32 * 4.0, 32.0), Quat::IDENTITY))
                .collect()
        };

        let world = slab(64, 0..8);
        let field = DistanceField::build(&world);

        // Two axes, not one. The (n,1) rows hold the relax count at 1 while
        // the biased count rises, so what they vary is the bias:relax ratio:
        // `solve`'s bias is a *velocity* target and positions are not
        // integrated inside the velocity loop, so more biased sweeps converge
        // toward delivering more push-out velocity. The balanced rows raise
        // both together.
        let tunings = [
            crate::Tuning { velocity_iterations: 1, relaxation_iterations: 1 },
            crate::Tuning { velocity_iterations: 2, relaxation_iterations: 1 },
            crate::Tuning { velocity_iterations: 4, relaxation_iterations: 1 },
            crate::Tuning { velocity_iterations: 8, relaxation_iterations: 1 },
            crate::Tuning { velocity_iterations: 2, relaxation_iterations: 2 },
            crate::Tuning { velocity_iterations: 4, relaxation_iterations: 4 },
            // Repeated last: if this does not land within 15% of the first
            // (1,1), the run is drift and none of its numbers may be used.
            crate::Tuning { velocity_iterations: 1, relaxation_iterations: 1 },
        ];

        let median = |mut v: Vec<f64>| {
            v.sort_by(|a, b| a.partial_cmp(b).unwrap());
            v[v.len() / 2]
        };

        // Median-of-seven ms/tick against both fixtures, variants interleaved
        // round by round so a slow round lands on every variant equally
        // rather than being read as one tuning's cost.
        //
        // **No warmup.** These are ticks 1..=SAMPLES of fresh fixtures, so the
        // "resting" column is a stack still settling, not a settled one --
        // `the_cost_of_sixteen_bodies` in this same file warms 300 ticks first
        // and this does not. The columns compare variants against each other on
        // identical state; neither is a resting-cost figure.
        let mut load_states: Vec<Vec<Body>> = tunings.iter().map(|_| load_fixture(&load_materials)).collect();
        let mut resting_states: Vec<Vec<Body>> = tunings.iter().map(|_| resting_fixture()).collect();
        let mut load_times: Vec<Vec<f64>> = (0..tunings.len()).map(|_| Vec::new()).collect();
        let mut resting_times: Vec<Vec<f64>> = (0..tunings.len()).map(|_| Vec::new()).collect();
        const SAMPLES: u32 = 7;
        for _round in 0..SAMPLES {
            for (i, tuning) in tunings.iter().enumerate() {
                let start = std::time::Instant::now();
                step_with(
                    &mut load_states[i],
                    &world,
                    &field,
                    &load_materials,
                    Air::vacuum(GRAVITY),
                    DT,
                    None,
                    &mut [],
                    *tuning,
                );
                load_times[i].push(start.elapsed().as_secs_f64() * 1000.0);

                let start = std::time::Instant::now();
                step_with(
                    &mut resting_states[i],
                    &world,
                    &field,
                    &resting_materials,
                    Air::vacuum(GRAVITY),
                    DT,
                    None,
                    &mut [],
                    *tuning,
                );
                resting_times[i].push(start.elapsed().as_secs_f64() * 1000.0);
            }
        }

        // The load fixture's divergence: the worst upward velocity any body
        // reaches, each variant from a fresh copy of the fixture.
        //
        // **The scan stops at the first breach.** This fixture collapses, and
        // `step_with`'s first statement deletes any body whose world box falls
        // below y = 0, so ticks past the collapse measure a shrinking scene
        // rather than a stack: a maximum over the whole 200 is a maximum over
        // an invalid regime. The peak's tick and the breach tick are printed
        // so the window each number came from is visible.
        let bodies_expected = load_fixture(&load_materials).len();
        let mut worst_up: Vec<f32> = Vec::new();
        let mut peak_tick: Vec<u32> = Vec::new();
        let mut breach_tick: Vec<Option<u32>> = Vec::new();
        for tuning in &tunings {
            let mut bodies = load_fixture(&load_materials);
            let mut worst = f32::NEG_INFINITY;
            let (mut at, mut breach) = (0u32, None);
            let start = std::time::Instant::now();
            for tick in 1..=200u32 {
                step_with(&mut bodies, &world, &field, &load_materials, Air::vacuum(GRAVITY), DT, None, &mut [], *tuning);
                if breached_the_slab(&bodies, bodies_expected) {
                    breach = Some(tick);
                    break;
                }
                let up = bodies.iter().map(|b| b.velocity.y).fold(f32::NEG_INFINITY, f32::max);
                if up > worst {
                    worst = up;
                    at = tick;
                }
            }
            let elapsed = start.elapsed().as_secs_f64();
            assert!(
                elapsed < 2.0,
                "tuning {tuning:?} took {elapsed:.3}s for the load fixture's scan, over the 2s bound"
            );
            worst_up.push(worst);
            peak_tick.push(at);
            breach_tick.push(breach);
        }

        // The resting fixture's penetration after 2000 ticks: how far the
        // bottom cube sits below its settled reference of 10.0, from
        // `a_stack_of_three_stands_still`.
        let mut penetration: Vec<f32> = Vec::new();
        for tuning in &tunings {
            let mut bodies = resting_fixture();
            for _ in 0..2000 {
                step_with(&mut bodies, &world, &field, &resting_materials, Air::vacuum(GRAVITY), DT, None, &mut [], *tuning);
            }
            // `.abs()`, as in `more_iterations_do_not_deepen_a_resting_contact`:
            // a clamp at zero would read a body floating above its reference
            // as a perfect contact.
            penetration.push((10.0 - bodies[0].position.y).abs());
        }

        println!(
            "load fixture masses: plate {plate_mass:.1}, heavy {heavy_mass:.1}, ratio {:.2}",
            heavy_mass / plate_mass
        );
        println!(
            "tuning       load ms/tick   resting ms/tick   worst upward v/s   at tick   breach tick   resting penetration"
        );
        for (i, tuning) in tunings.iter().enumerate() {
            println!(
                "({:>2},{:>2})     {:>10.4}   {:>13.4}     {:>14.4}   {:>7}   {:>11}   {:>18.4}",
                tuning.velocity_iterations,
                tuning.relaxation_iterations,
                median(load_times[i].clone()),
                median(resting_times[i].clone()),
                worst_up[i],
                peak_tick[i],
                breach_tick[i].map_or_else(|| "none".to_string(), |t| t.to_string()),
                penetration[i]
            );
        }

        let first = median(load_times[0].clone());
        let repeat = median(load_times[tunings.len() - 1].clone());
        let drift = (repeat - first).abs() / first;
        println!("drift check: first (1,1) {first:.4} ms, repeat (1,1) {repeat:.4} ms, drift {:.1}%", drift * 100.0);
        assert!(
            drift < 0.15,
            "the repeated (1,1) differs from the first by {:.1}%: this run is drift, its numbers may not be used",
            drift * 100.0
        );
    }

    /// **What the tick rate does to the blow.** Not a gate: it prints a table.
    ///
    /// The threshold `break_what_gave_way` reads is `peak[at]`, the largest
    /// *accumulated* normal impulse a contact reached inside the tick. For a
    /// collision that resolves inside one tick that is near enough the
    /// momentum exchanged, which is rate-independent -- Dwyer's argument
    /// against a force. For a load that is *held*, it is force x dt, and a
    /// quantity proportional to `dt` halves at every doubling of the rate.
    ///
    /// Which of the two every column is, is the question. A column
    /// proportional to `dt` falls by 8 from 64 Hz to 512; a rate-independent
    /// one is flat. The same wall-clock duration is simulated at each rate,
    /// not the same number of ticks, and the rates are interleaved within this
    /// one invocation with 64 Hz repeated last, so the repeat is a determinism
    /// check on the whole table rather than a number from an earlier run.
    ///
    /// Every material here is the unbreakable fixture (7), so nothing fractures
    /// and no measured load is perturbed by the scene coming apart. It is glass
    /// in every other respect -- same density, friction and restitution.
    #[test]
    #[ignore]
    fn what_the_tick_rate_does_to_the_blow() {
        // Same wall clock at every rate: the tick counts the earlier
        // measurements used, divided by the 64 Hz they used them at.
        const REST_SECONDS: f32 = 1200.0 / 64.0;
        const CRUSH_SECONDS: f32 = 400.0 / 64.0;
        const SLAM_SECONDS: f32 = 1.0 / 6.0;
        /// The crush target creeps at a rate per *second*, not per tick:
        /// 0.002 a tick at 64 Hz is an eighth of a voxel a second.
        const CREEP: f32 = 0.002 * 64.0;
        const SLAM_SPEED: f32 = 60.0;

        let ticks = |seconds: f32, dt: f32| (seconds / dt).round() as u32;
        let unbreakable = MaterialId(7);

        // The three-cube stack of `resting_weight_breaks_nothing`, in the
        // unbreakable material. Returns the largest blow any contact carried
        // over the run, the last non-zero one (the settled load, after the
        // drop from 10.2 has gone), and the normal impulse the bottom body's
        // contacts deliver in a tick -- which is `m g dt` and is therefore a
        // known-proportional-to-dt control column.
        let resting = |dt: f32| -> (f32, u32, f32, f32) {
            let materials = materials();
            let world = slab_of(64, 0..8, unbreakable);
            let field = DistanceField::build(&world);
            let mut bodies: Vec<Body> = (0..3)
                .map(|i| {
                    placed(
                        cube_of(4, 4, unbreakable),
                        Vec3::new(32.0, 10.2 + i as f32 * 4.0, 32.0),
                        Quat::IDENTITY,
                    )
                })
                .collect();
            let (mut peak, mut at, mut settled) = (0.0f32, 0u32, 0.0f32);
            for tick in 1..=ticks(REST_SECONDS, dt) {
                let out =
                    step(&mut bodies, &world, &field, &materials, Air::VACUUM, dt, None, &mut []);
                assert!(out.fractures.is_empty(), "the unbreakable fixture broke");
                if out.peak_impulse > peak {
                    peak = out.peak_impulse;
                    at = tick;
                }
                if out.peak_impulse > 0.0 {
                    settled = out.peak_impulse;
                }
            }
            assert_eq!(bodies.len(), 3, "the stack came apart");
            // What the bottom body's contacts actually delivered, as
            // `a_body_at_rest_stays_at_rest` reads it: the floor contacts plus
            // the one to the cube above, so `m g dt` for the whole stack plus
            // the two cubes above it. Summed in a fixed order -- `warm` is a
            // hash map, and float addition is not associative, so iteration
            // order would move the last bit between runs.
            let mut carried: Vec<f32> = bodies[0].warm.values().map(|i| i.normal).collect();
            carried.sort_by(|a, b| a.partial_cmp(b).unwrap());
            let delivered: f32 = carried.iter().sum::<f32>() * SUBSTEPS as f32;
            (peak, at, settled, delivered)
        };

        // `a_slow_crush_breaks_what_it_presses`, with the floor unbreakable so
        // the press runs the whole way instead of stopping at the first break.
        let crush = |dt: f32| -> f32 {
            let materials = materials();
            let world = slab_of(64, 0..8, unbreakable);
            let field = DistanceField::build(&world);
            let centre = Vec3::new(32.0, 10.0, 32.0);
            let mut bodies = vec![placed(cube_of(4, 4, unbreakable), centre, Quat::IDENTITY)];
            assert!(recompute(&mut bodies[0], &materials));
            let mut grab = Joint::grab(&bodies[0], centre);
            let mut peak = 0.0f32;
            for tick in 0..ticks(CRUSH_SECONDS, dt) {
                grab.anchor_b = centre - Vec3::Y * (tick as f32 * dt * CREEP);
                let out = step(
                    &mut bodies,
                    &world,
                    &field,
                    &materials,
                    Air::VACUUM,
                    dt,
                    Some(&mut grab),
                    &mut [],
                );
                assert!(out.fractures.is_empty(), "the unbreakable fixture broke");
                peak = peak.max(out.peak_impulse);
            }
            peak
        };

        // One slam, as `slam` stages it: no gravity, so the approach speed is
        // the only thing in it. Returns the largest `peak[at]` of the
        // collision, the normal impulse summed over every tick of it, and the
        // momentum the body actually gave up -- `m dv`, which is the exchanged
        // momentum by conservation and so the reference the other two are read
        // against.
        let slam_at = |dt: f32| -> (f32, f32, f32) {
            let materials = materials();
            let world = slab_of(64, 0..8, unbreakable);
            let field = DistanceField::build(&world);
            let mut body =
                placed(cube_of(4, 4, unbreakable), Vec3::new(32.0, 11.0, 32.0), Quat::IDENTITY);
            assert!(recompute(&mut body, &materials));
            body.velocity = Vec3::new(0.0, -SLAM_SPEED, 0.0);
            let mass = body.mass.mass;
            let mut bodies = vec![body];
            let (mut peak, mut summed) = (0.0f32, 0.0f32);
            for _ in 0..ticks(SLAM_SECONDS, dt) {
                let out =
                    step(&mut bodies, &world, &field, &materials, Air::STILL, dt, None, &mut []);
                assert!(out.fractures.is_empty(), "the unbreakable fixture broke");
                peak = peak.max(out.peak_impulse);
                // The blow summed over however many ticks the detector spread
                // the collision across. With no gravity the body rests on a
                // contact carrying nothing once the collision is over, so this
                // sums the collision and not the window.
                //
                // Not `warm` x `SUBSTEPS`: that identity holds for a contact at
                // equilibrium, which is exactly what a collision is not -- it
                // read 11.0e6 against a true 3.84e6 at 64 Hz and 17e3 at 256,
                // where the contact had already separated by the tick's end.
                summed += out.peak_impulse;
            }
            (peak, summed, mass * (bodies[0].velocity.y + SLAM_SPEED))
        };

        // 64 Hz last as well as first: these are deterministic simulations, so
        // the repeat must come back bit-identical, and a table whose repeat
        // moved is a table that measured something other than the rate.
        let rates = [64.0f32, 128.0, 256.0, 512.0, 64.0];
        let mut rows = Vec::new();
        for &hz in &rates {
            let dt = 1.0 / hz;
            let (rest_peak, rest_tick, rest_settled, weight) = resting(dt);
            let crush_peak = crush(dt);
            let (slam_peak, slam_summed, slam_exchanged) = slam_at(dt);
            rows.push([
                hz,
                rest_peak,
                rest_tick as f32,
                rest_settled,
                weight,
                crush_peak,
                slam_peak,
                slam_summed,
                slam_exchanged,
            ]);
        }

        println!(
            "  Hz |   rest max | at tick | rest settled |   m g dt |  crush peak |   slam peak |              slam sum | slam m dv"
        );
        for r in &rows {
            println!(
                "{:>4} | {:>10.0} | {:>7.0} | {:>12.0} | {:>8.0} | {:>11.0} | {:>11.0} |                  {:>8.0} | {:>9.0}",
                r[0], r[1], r[2], r[3], r[4], r[5], r[6], r[7], r[8]
            );
        }

        // How far each column moved over the three doublings. 8.0 is exactly
        // proportional to `dt`; 1.0 is rate-independent.
        let names =
            ["rest max", "rest settled", "m g dt", "crush peak", "slam peak", "slam sum", "slam m dv"];
        let columns = [1usize, 3, 4, 5, 6, 7, 8];
        let moved: Vec<String> = names
            .iter()
            .zip(columns)
            .map(|(name, c)| {
                let (a, b) = (rows[0][c], rows[3][c]);
                format!("{name} {:.2}x", if b == 0.0 { f32::INFINITY } else { a / b })
            })
            .collect();
        println!(
            "64 Hz / 512 Hz -- {}  (8.00 = proportional to dt, 1.00 = rate-independent)",
            moved.join(", ")
        );

        // Deterministic simulations, so the repeat must land on the same
        // numbers. A relative window rather than equality: nothing here sums a
        // hash map any more, but a measurement is not the place to make a
        // float's last bit the gate.
        for (c, name) in columns.iter().zip(names) {
            let (first, repeat) = (rows[0][*c], rows[4][*c]);
            assert!(
                (first - repeat).abs() <= 1e-4 * first.abs().max(1.0),
                "the repeated 64 Hz row's {name} is {repeat} against the first row's {first}:                  this table did not measure the rate"
            );
        }
    }

    /// **A characterisation of a known defect, not a property anyone wants.**
    /// A four-high glass stack destroys itself as it settles.
    ///
    /// `resting_weight_breaks_nothing` stands three cubes and `GLASS_STRENGTH`
    /// is 350,000, calibrated just above the 331,306 that stack carries. A
    /// fourth cube carries 356,751 -- measured 2026-10-02 and recorded in
    /// `docs/concepts/fracture-load-window.md` -- which is *above* the shipped
    /// strength. So the cliff is at **four cubes, not past four**: the page
    /// said six, and the arithmetic in its own table said four.
    ///
    /// **What it is not is weight.** The break lands at tick 5, and
    /// `what_the_tick_rate_does_to_the_blow` measures the three-cube stack's
    /// *settled* blow at 18,798 against a transient maximum of 331,306 at
    /// tick 5 -- the same tick. Both tables' "resting load" is the stack
    /// landing from the 0.2 voxels the fixture starts above the floor, and the
    /// load it then holds is twenty times smaller. The same four cubes started
    /// at their settled heights do not break, which this test prints.
    ///
    /// **It is expected to fail until the rule separates a steady load from a
    /// newly-taken one**, which is the fix the concept page names -- or until
    /// the blow stops counting the bias's push-out, which is what inflates a
    /// landing's `peak[at]` over the momentum it exchanges. Raising
    /// `GLASS_STRENGTH` is neither: 350,000 is already within 5% of the
    /// 365,906 a grab can press with, so buying headroom here spends the crush
    /// gate.
    ///
    /// `#[ignore]`d for the same reason as
    /// `the_240_to_1_load_collapses_through_the_floor`: it is a record of a
    /// bug, not a gate the suite should enforce. The day it passes, drop the
    /// `#[ignore]` and the defect half of this comment.
    #[test]
    #[ignore]
    fn a_four_high_glass_stack_stands() {
        let materials = materials();
        let world = slab_of(64, 0..8, MaterialId(6));
        let field = DistanceField::build(&world);
        let mut bodies: Vec<Body> = (0..4)
            .map(|i| {
                placed(
                    cube_of(4, 4, MaterialId(6)),
                    Vec3::new(32.0, 10.2 + i as f32 * 4.0, 32.0),
                    Quat::IDENTITY,
                )
            })
            .collect();

        let mut broke = Vec::new();
        let mut heaviest = 0.0f32;
        let mut first = None;
        for tick in 1..=1200u32 {
            let out =
                step(&mut bodies, &world, &field, &materials, Air::VACUUM, DT, None, &mut []);
            heaviest = heaviest.max(out.peak_impulse);
            if first.is_none() && !out.fractures.is_empty() {
                first = Some(tick);
            }
            broke.extend(out.fractures);
        }
        // The tick matters as much as the count. `rest settled` in
        // `what_the_tick_rate_does_to_the_blow` is 18,798 for three cubes, two
        // orders below the strength, so a break in the first few ticks is the
        // stack *landing* from the 0.2 voxels the fixture starts above the
        // floor -- not its weight.
        println!(
            "four-high glass: {} fractures, first at tick {:?}, heaviest blow {heaviest:.0}              against a strength of {}",
            broke.len(),
            first,
            crate::fixtures::GLASS_STRENGTH
        );
        // The same four cubes, started where they settle instead of 0.2
        // voxels up: no landing, so what is left is the weight. Printed, not
        // asserted -- it is the control that says which of the two the
        // assertion below is about.
        let mut settled: Vec<Body> = (0..4)
            .map(|i| {
                placed(
                    cube_of(4, 4, MaterialId(6)),
                    Vec3::new(32.0, 10.0 + i as f32 * 4.0, 32.0),
                    Quat::IDENTITY,
                )
            })
            .collect();
        let (mut settled_broke, mut settled_heaviest) = (0usize, 0.0f32);
        for _ in 0..1200 {
            let out =
                step(&mut settled, &world, &field, &materials, Air::VACUUM, DT, None, &mut []);
            settled_heaviest = settled_heaviest.max(out.peak_impulse);
            settled_broke += out.fractures.len();
        }
        println!(
            "four-high glass started at its settled heights: {settled_broke} fractures,              heaviest blow {settled_heaviest:.0}"
        );

        assert!(
            broke.is_empty(),
            "a four-high glass stack settling onto the floor broke {} things; the heaviest blow \
             any contact carried was {heaviest:.0} against a strength of {}. Started at its \
             settled heights instead, the same stack broke {settled_broke} with a heaviest blow \
             of {settled_heaviest:.0}",
            broke.len(),
            crate::fixtures::GLASS_STRENGTH
        );
        assert_eq!(bodies.len(), 4, "the stack came apart");
    }
}
