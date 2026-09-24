//! Sleeping: a body at rest stops costing anything until something could move
//! it. After Dwyer's devlog #13, and Box2D.
//!
//! Waking runs to a fixed point at the start of a tick, before anything is
//! detected, so no contact or joint is ever solved against a sleeper and a
//! sleeper never has to stand in as a static body.
//!
//! Bodies fall asleep together or not at all, as Box2D's islands do: every
//! group of awake bodies touching or jointed together sleeps in the same tick,
//! once each of them has been still for `SLEEP_AFTER`. So a pile never has some
//! of its bodies zeroed while their neighbours still press on them. Waking
//! restarts a body's count, which is what the grouping makes safe: one body at
//! a time, two resting neighbours would take turns waking each other forever.

use super::contact::{boxes_overlap, world_box};
use super::joint::Joint;
use super::{BASE_MARGIN, SLEEP_AFTER, SLEEP_SPEED};
use bevox_core::body::{Body, BodyId};
use glam::Vec3;

/// Wakes every sleeper that something awake could move, until nothing
/// changes.
///
/// - An awake body whose box, grown by how far it can travel this tick and the
///   margin, overlaps a sleeper's box wakes it. So a stack a body lands on
///   wakes from the top down, within the tick.
/// - A joint wakes its other side when either side is awake. A joint to the
///   world wakes nothing: the world is never awake.
/// - `grabbed`, the body the mouse holds, is always awake.
pub(crate) fn wake(bodies: &mut [Body], joints: &[Joint], grabbed: Option<BodyId>, travel: &[f32]) {
    if let Some(id) = grabbed {
        for b in bodies.iter_mut().filter(|b| b.id == id) {
            b.asleep = false;
            b.still_for = 0.0;
        }
    }
    let boxes: Vec<Option<(Vec3, Vec3)>> =
        bodies.iter().zip(travel).map(|(b, t)| world_box(b, t + BASE_MARGIN)).collect();
    let find = |bodies: &[Body], id: BodyId| bodies.iter().position(|b| b.id == id);
    loop {
        let mut changed = false;
        for i in 0..bodies.len() {
            if bodies[i].asleep || bodies[i].mass.mass <= 0.0 {
                continue;
            }
            for j in 0..bodies.len() {
                if !bodies[j].asleep {
                    continue;
                }
                if let (Some(a), Some(b)) = (boxes[i], boxes[j])
                    && boxes_overlap(a, b)
                {
                    bodies[j].asleep = false;
                    bodies[j].still_for = 0.0;
                    changed = true;
                }
            }
        }
        for joint in joints {
            let Some(other) = joint.b else {
                continue;
            };
            if let (Some(i), Some(j)) = (find(bodies, joint.a), find(bodies, other))
                && bodies[i].asleep != bodies[j].asleep
            {
                for k in [i, j] {
                    bodies[k].asleep = false;
                    bodies[k].still_for = 0.0;
                }
                changed = true;
            }
        }
        if !changed {
            break;
        }
    }
}

/// After a tick: counts how long each body has been still, and puts to sleep
/// every group of awake bodies, touching or jointed together, that has been
/// still for `SLEEP_AFTER` as a whole. Their motion is zeroed, so nothing is
/// left to creep. A sleeper counts on, which is how long it has slept.
///
/// Still means both a body's centre of mass and its farthest voxel, `radii`
/// from that centre, move slower than `SLEEP_SPEED`: a body spinning in place
/// is not at rest. A body a motor drives never sleeps, or a servo that reached
/// its target would ignore the next one.
pub(crate) fn settle(bodies: &mut [Body], joints: &[&mut Joint], radii: &[f32], dt: f32) {
    let driven: Vec<BodyId> = joints
        .iter()
        .filter(|j| j.motor.is_some())
        .flat_map(|j| std::iter::once(j.a).chain(j.b))
        .collect();
    for (b, &r) in bodies.iter_mut().zip(radii) {
        if b.mass.mass <= 0.0 {
            continue;
        }
        if b.asleep {
            b.still_for += dt;
            continue;
        }
        let spin = b.angular_velocity().length() * r;
        let still = b.velocity.length() < SLEEP_SPEED && spin < SLEEP_SPEED;
        if still && !driven.contains(&b.id) {
            b.still_for += dt;
        } else {
            b.still_for = 0.0;
        }
    }

    // Groups of awake bodies, by union-find over touching boxes and joints.
    let awake: Vec<usize> =
        (0..bodies.len()).filter(|&i| !bodies[i].asleep && bodies[i].mass.mass > 0.0).collect();
    let mut group: Vec<usize> = (0..bodies.len()).collect();
    fn root(group: &mut [usize], mut i: usize) -> usize {
        while group[i] != i {
            group[i] = group[group[i]];
            i = group[i];
        }
        i
    }
    let boxes: Vec<Option<(Vec3, Vec3)>> = bodies.iter().map(|b| world_box(b, BASE_MARGIN)).collect();
    for (n, &i) in awake.iter().enumerate() {
        for &j in &awake[n + 1..] {
            if let (Some(a), Some(b)) = (boxes[i], boxes[j])
                && boxes_overlap(a, b)
            {
                let (ri, rj) = (root(&mut group, i), root(&mut group, j));
                group[ri] = rj;
            }
        }
    }
    let position = |id: BodyId| bodies.iter().position(|b| b.id == id);
    let pairs: Vec<(usize, usize)> =
        joints.iter().filter_map(|j| Some((position(j.a)?, position(j.b?)?))).collect();
    for (i, j) in pairs {
        let (ri, rj) = (root(&mut group, i), root(&mut group, j));
        group[ri] = rj;
    }
    let mut ready = vec![true; bodies.len()];
    for &i in &awake {
        let r = root(&mut group, i);
        ready[r] &= bodies[i].still_for >= SLEEP_AFTER;
    }
    for &i in &awake {
        if ready[root(&mut group, i)] {
            let b = &mut bodies[i];
            b.asleep = true;
            b.velocity = Vec3::ZERO;
            b.angular_momentum = Vec3::ZERO;
        }
    }
}

/// Wakes every body whose box overlaps the world-space box `lo..hi`, and
/// restarts its count. For edits: an erase that takes a sleeper's support, or
/// paint on or beside one, must let it move.
pub fn wake_near(bodies: &mut [Body], lo: Vec3, hi: Vec3) {
    for b in bodies {
        if world_box(b, BASE_MARGIN).is_some_and(|bx| boxes_overlap(bx, (lo, hi))) {
            b.asleep = false;
            b.still_for = 0.0;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::Air;
    use bevox_core::contree::Contree;
    use bevox_core::distance_field::DistanceField;
    use bevox_core::material::{MaterialId, MaterialTable};
    use crate::fixtures::{cube, materials, placed, slab, slab_of};
    use crate::joint::{Angular, Linear};
    use crate::solver::step;
    use glam::Quat;

    const DT: f32 = 1.0 / 64.0;

    struct Scene {
        world: Contree,
        field: DistanceField,
        materials: MaterialTable,
    }

    impl Scene {
        fn floor() -> Self {
            Self::of(slab(64, 0..8))
        }

        fn of(world: Contree) -> Self {
            let field = DistanceField::build(&world);
            Self { world, field, materials: materials() }
        }

        fn run(&self, bodies: &mut Vec<Body>, joints: &mut [Joint], ticks: u32) {
            for _ in 0..ticks {
                step(bodies, &self.world, &self.field, &self.materials, Air::VACUUM, DT, None, joints);
            }
        }
    }

    /// A body at rest falls asleep, and then does not move at all: its height
    /// ten thousand ticks later is the height it slept at, bit for bit. Woken,
    /// it has gathered nothing while asleep: a sleeper that kept taking gravity
    /// would wake with all of it at once.
    #[test]
    fn a_body_at_rest_falls_asleep_and_stays_put() {
        let scene = Scene::floor();
        let mut bodies = vec![placed(cube(4, 4), Vec3::new(32.0, 10.5, 32.0), Quat::IDENTITY)];
        scene.run(&mut bodies, &mut [], 1000);
        assert!(bodies[0].asleep, "a body at rest for a thousand ticks is still awake");
        let slept_at = bodies[0].position;
        scene.run(&mut bodies, &mut [], 10_000);
        assert!(bodies[0].asleep);
        assert_eq!(bodies[0].position, slept_at, "a sleeping body moved");
        wake_near(&mut bodies, slept_at - 3.0, slept_at + 3.0);
        scene.run(&mut bodies, &mut [], 1);
        let speed = bodies[0].velocity.length();
        assert!(speed < 1.0, "the body woke moving at {speed}: it gathered speed while asleep");
    }

    /// A stack of three falls asleep and does not creep.
    #[test]
    fn a_stack_sleeps_without_creeping() {
        let scene = Scene::floor();
        let mut bodies: Vec<Body> = (0..3)
            .map(|i| placed(cube(4, 4), Vec3::new(32.0, 10.2 + i as f32 * 4.0, 32.0), Quat::IDENTITY))
            .collect();
        scene.run(&mut bodies, &mut [], 2000);
        assert!(bodies.iter().all(|b| b.asleep), "the stack never fell asleep");
        let slept_at: Vec<Vec3> = bodies.iter().map(|b| b.position).collect();
        scene.run(&mut bodies, &mut [], 8000);
        for (b, was) in bodies.iter().zip(&slept_at) {
            assert_eq!(b.position, *was, "a sleeping body in the stack crept");
        }
    }

    /// A body dropped on a sleeping stack wakes all of it, down to the bottom,
    /// in the tick it arrives: the stack takes its weight rather than letting
    /// it through.
    #[test]
    fn a_body_dropped_on_a_sleeping_stack_wakes_it_all() {
        let scene = Scene::floor();
        let mut bodies: Vec<Body> = (0..3)
            .map(|i| placed(cube(4, 4), Vec3::new(32.0, 10.2 + i as f32 * 4.0, 32.0), Quat::IDENTITY))
            .collect();
        scene.run(&mut bodies, &mut [], 2000);
        assert!(bodies.iter().all(|b| b.asleep), "the stack never fell asleep");
        bodies.push(placed(cube(4, 4), Vec3::new(32.0, 30.0, 32.0), Quat::IDENTITY));
        // The tick each of the stack first woke in.
        let mut woke: [Option<u32>; 3] = [None; 3];
        for tick in 0..120 {
            scene.run(&mut bodies, &mut [], 1);
            for (w, b) in woke.iter_mut().zip(&bodies) {
                if w.is_none() && !b.asleep {
                    *w = Some(tick);
                }
            }
        }
        assert!(woke.iter().all(Option::is_some), "part of the stack slept through a body landing on it: {woke:?}");
        assert!(woke.iter().all(|w| *w == woke[0]), "the stack woke a body at a time, not together: {woke:?}");
        // The stack's top face is at 20, so a 4-voxel cube rests at 22.
        let y = bodies[3].position.y;
        assert!((y - 22.0).abs() < 0.2, "the dropped body did not come to rest on the stack: {y}");
    }

    /// CPU time for a tick with sixteen bodies resting on the floor, asleep
    /// against the same sixteen kept awake. Not a gate; its numbers go in the
    /// plan's Measurements section.
    #[test]
    #[ignore]
    fn sleeping_is_timed() {
        let scene = Scene::floor();
        let grid = || -> Vec<Body> {
            (0..16)
                .map(|i| {
                    placed(
                        cube(4, 4),
                        Vec3::new(8.0 + (i % 4) as f32 * 14.0, 10.5, 8.0 + (i / 4) as f32 * 14.0),
                        Quat::IDENTITY,
                    )
                })
                .collect()
        };
        let median = |mut v: Vec<f64>| {
            v.sort_by(|a, b| a.partial_cmp(b).unwrap());
            v[v.len() / 2]
        };
        let time = |bodies: &mut Vec<Body>, ticks: u32, keep_awake: bool| -> f64 {
            let ms: Vec<f64> = (0..ticks)
                .map(|_| {
                    if keep_awake {
                        for b in bodies.iter_mut() {
                            b.asleep = false;
                            b.still_for = 0.0;
                        }
                    }
                    let start = std::time::Instant::now();
                    scene.run(bodies, &mut [], 1);
                    start.elapsed().as_secs_f64() * 1000.0
                })
                .collect();
            median(ms)
        };

        let mut bodies = grid();
        time(&mut bodies, 1000, false);
        assert!(bodies.iter().all(|b| b.asleep), "the grid never fell asleep");
        let asleep = time(&mut bodies, 1000, false);
        let mut bodies = grid();
        time(&mut bodies, 300, true);
        let awake = time(&mut bodies, 1000, true);
        println!("16 bodies at rest, median ms/tick: asleep {asleep:.4}, awake {awake:.4}");
    }

    /// Two bodies that come to rest at different times, touching, fall asleep
    /// together. The first sleeps; the second lands beside it, wakes it, and
    /// settles; then both sleep. One body at a time, with waking restarting
    /// the count, they would take turns waking each other forever.
    ///
    /// Beside, not on top: a tower of four struck from above was measured
    /// rocking for 80 seconds before it slept, a solver matter, not a sleeping
    /// one.
    #[test]
    fn neighbours_at_rest_at_different_times_sleep_together() {
        let scene = Scene::floor();
        let mut bodies = vec![placed(cube(4, 4), Vec3::new(30.0, 10.5, 32.0), Quat::IDENTITY)];
        scene.run(&mut bodies, &mut [], 200);
        assert!(bodies[0].asleep, "the first body never slept");
        // A hair clear of the first, so the two boxes overlap within the margin.
        bodies.push(placed(cube(4, 4), Vec3::new(34.05, 11.0, 32.0), Quat::IDENTITY));
        let mut woke = false;
        for _ in 0..300 {
            scene.run(&mut bodies, &mut [], 1);
            woke |= !bodies[0].asleep;
        }
        assert!(woke, "the second body never woke the first, so this tests nothing");
        assert!(bodies.iter().all(|b| b.asleep), "the two never slept together");
    }

    /// The mouse grab wakes what it holds, and lifts it.
    #[test]
    fn a_grab_wakes_a_sleeper() {
        let scene = Scene::floor();
        let mut bodies = vec![placed(cube(4, 4), Vec3::new(32.0, 10.5, 32.0), Quat::IDENTITY)];
        scene.run(&mut bodies, &mut [], 1000);
        assert!(bodies[0].asleep);
        let rest = bodies[0].position;
        let mut grab = Joint::grab(&bodies[0], rest);
        grab.anchor_b = rest + Vec3::Y * 3.0;
        for _ in 0..120 {
            step(&mut bodies, &scene.world, &scene.field, &scene.materials, Air::VACUUM, DT, Some(&mut grab), &mut []);
        }
        assert!(bodies[0].position.y > rest.y + 2.0, "the grab did not wake and lift the body");
    }

    /// A joint wakes its other side: kick one of two jointed sleepers and the
    /// other wakes the same tick.
    #[test]
    fn a_joint_wakes_its_partner() {
        let scene = Scene::floor();
        let mut bodies = vec![
            placed(cube(4, 4), Vec3::new(20.0, 10.0, 32.0), Quat::IDENTITY),
            placed(cube(4, 4), Vec3::new(40.0, 10.0, 32.0), Quat::IDENTITY),
        ];
        for b in &mut bodies {
            b.asleep = true;
        }
        let at = Vec3::new(30.0, 11.0, 32.0);
        let mut joints = vec![Joint::new(&bodies[0], Some(&bodies[1]), Linear::Point, Angular::Free, at, at, Vec3::Y)];
        bodies[0].asleep = false;
        bodies[0].velocity = Vec3::new(-10.0, 0.0, 0.0);
        scene.run(&mut bodies, &mut joints, 1);
        assert!(!bodies[1].asleep, "a joint left its partner asleep");
    }

    /// An erase under a sleeper, with `wake_near` over it as the app calls it,
    /// drops the sleeper. Without the wake it would hang in the air.
    #[test]
    fn waking_near_an_erase_drops_the_sleeper() {
        let mut scene = Scene::floor();
        let mut bodies = vec![placed(cube(4, 4), Vec3::new(32.0, 10.5, 32.0), Quat::IDENTITY)];
        scene.run(&mut bodies, &mut [], 1000);
        assert!(bodies[0].asleep);
        let (centre, radius) = (Vec3::new(32.0, 6.0, 32.0), 6.0);
        scene.world.apply_sphere(centre, radius, MaterialId::EMPTY);
        scene.run(&mut bodies, &mut [], 10);
        assert!(bodies[0].asleep, "the sleeper woke without being told, so this tests nothing");
        wake_near(&mut bodies, centre - radius - 1.0, centre + radius + 1.0);
        scene.run(&mut bodies, &mut [], 10);
        assert!(bodies[0].velocity.y < -1.0, "the sleeper hung in the air: {:?}", bodies[0].velocity);
    }

    /// Moving bodies never fall asleep: one sliding on ice, and one spinning in
    /// place with its centre of mass still.
    #[test]
    fn moving_bodies_stay_awake() {
        let scene = Scene::of(slab_of(64, 0..8, MaterialId(3)));
        let mut sliding = vec![placed(cube(4, 4), Vec3::new(4.0, 10.0, 32.0), Quat::IDENTITY)];
        sliding[0].velocity = Vec3::new(1.0, 0.0, 0.0);
        let mut ever = false;
        for _ in 0..600 {
            scene.run(&mut sliding, &mut [], 1);
            ever |= sliding[0].asleep;
        }
        assert!(!ever, "a body sliding at 1 voxel/s fell asleep");

        let space = Scene::of(Contree::empty(3));
        let mut spinning = vec![placed(cube(4, 4), Vec3::splat(500.0), Quat::IDENTITY)];
        spinning[0].set_angular_velocity(Vec3::Y * 0.1);
        for _ in 0..600 {
            step(&mut spinning, &space.world, &space.field, &space.materials, Air::STILL, DT, None, &mut []);
            ever |= spinning[0].asleep;
        }
        assert!(!ever, "a body spinning in place fell asleep");
    }
}
