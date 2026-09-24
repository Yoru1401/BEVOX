//! The scenes the number keys load: the demo scene, and a scene of joints to
//! try by hand.

use bevox_core::body::Body;
use bevox_core::contree::Contree;
use bevox_core::dense::DenseVolume;
use bevox_core::material::{Material, MaterialId, MaterialTable};
use bevox_core::physics::GRAVITY;
use bevox_core::physics::joint::{Angular, Drive, Friction, Joint, Linear, Motor};
use bevox_render::camera::start_camera;
use bevy::prelude::*;
use std::f32::consts::{FRAC_PI_4, FRAC_PI_6};

/// Which scene is loaded.
#[derive(Resource, Clone, Copy, Debug, Default, PartialEq, Eq)]
pub(crate) enum SceneKind {
    #[default]
    Demo,
    Joints,
}

impl SceneKind {
    pub(crate) fn build(self) -> Scene {
        match self {
            SceneKind::Demo => demo(),
            SceneKind::Joints => joints(),
        }
    }

    pub(crate) fn name(self) -> &'static str {
        match self {
            SceneKind::Demo => "demo",
            SceneKind::Joints => "joints",
        }
    }
}

/// Everything a scene starts with.
pub(crate) struct Scene {
    pub tree: Contree,
    pub materials: MaterialTable,
    pub bodies: Vec<Body>,
    pub joints: Vec<Joint>,
    /// Where the camera starts, and what it looks at.
    pub eye: Vec3,
    pub look_at: Vec3,
}

/// The demo world, with a brick cube dropped in view.
pub(crate) fn demo() -> Scene {
    let (tree, materials) = demo_scene();
    with_falling_body(tree, materials)
}

/// Any world, with the camera where `start_camera` puts it and a brick cube in
/// view: fourteen voxels ahead, ten short of the surface the camera faces, and
/// six to the right of the line of sight, so it does not hide what the camera
/// is looking at. With mass, it falls onto whatever is below.
pub(crate) fn with_falling_body(tree: Contree, materials: MaterialTable) -> Scene {
    let (eye, look_at) = start_camera(&tree);
    let forward = (look_at - eye).normalize();
    let right = forward.cross(Vec3::Y).normalize_or_zero();
    let mut body = demo_body(eye + forward * 14.0 + right * 6.0, Quat::IDENTITY);
    body.recompute(&materials);
    Scene { tree, materials, bodies: vec![body], joints: vec![], eye, look_at }
}

/// A six-voxel cube of material 2 -- brick, in the demo palette -- whose middle
/// is at `centre`, turned by `orientation`. Call `recompute` before simulating
/// it.
///
/// Off the 4-voxel brick grid (5..11 in a 16 volume) so its bricks are partial
/// and it owns real voxel bytes, like the body the parity tests prove, rather
/// than collapsing to uniform nodes.
pub(crate) fn demo_body(centre: Vec3, orientation: Quat) -> Body {
    let mut dense = DenseVolume::new(16).unwrap();
    for z in 5..11 {
        for y in 5..11 {
            for x in 5..11 {
                dense.set(UVec3::new(x, y, z), MaterialId(2));
            }
        }
    }
    Body::new(dense.into_contree(), centre - orientation * Vec3::splat(8.0), orientation)
}

/// The same floor, column and carved sphere the parity test uses, so what is on
/// screen is what the test proved correct, together with the palette it is
/// drawn from.
pub(crate) fn demo_scene() -> (Contree, MaterialTable) {
    let mut dense = DenseVolume::new(64).unwrap();
    for z in 0..64 {
        for x in 0..64 {
            for y in 0..6 {
                dense.set(UVec3::new(x, y, z), MaterialId(1));
            }
        }
    }
    // A slippery strip and a bouncy patch in the floor's top layer, so a
    // dropped body shows friction and restitution without any editing.
    for z in 0..64 {
        for x in 8..24 {
            dense.set(UVec3::new(x, 5, z), MaterialId(3)); // ice
        }
    }
    for z in 40..56 {
        for x in 40..56 {
            dense.set(UVec3::new(x, 5, z), MaterialId(4)); // rubber
        }
    }
    for z in 28..36 {
        for y in 6..26 {
            for x in 28..36 {
                dense.set(UVec3::new(x, y, z), MaterialId(2));
            }
        }
    }
    let mut tree = dense.into_contree();
    tree.apply_sphere(Vec3::new(32.0, 18.0, 32.0), 5.0, MaterialId::EMPTY);
    (tree, palette())
}

/// Stone, brick, ice and rubber.
pub(crate) fn palette() -> MaterialTable {
    let mut materials = MaterialTable::new();
    materials
        .push(Material { color: [140, 140, 150, 255], density: 2600, friction: 60, restitution: 5, strength: 45 })
        .unwrap(); // 1: stone
    materials
        .push(Material { color: [180, 90, 70, 255], density: 1900, friction: 70, restitution: 5, strength: 25 })
        .unwrap(); // 2: brick
    materials
        .push(Material { color: [170, 210, 235, 255], density: 900, friction: 4, restitution: 10, strength: 12 })
        .unwrap(); // 3: ice
    materials
        .push(Material {
            color: [40, 40, 45, 255],
            density: 1100,
            friction: 80,
            restitution: 80,
            strength: bevox_core::material::UNBREAKABLE,
        })
        .unwrap(); // 4: rubber
    materials
}

/// The terrain's material: stone, in the palette.
const STONE: MaterialId = MaterialId(1);
/// Every body's material: brick.
const BRICK: MaterialId = MaterialId(2);

/// How hard the door's hinge resists turning. The door weighs about 1.8e5 and
/// turns about its hinge with an inertia of about 3.9e6, so this stops a swing
/// of one radian a second in about a second.
const DOOR_FRICTION: f32 = 4.0e6;
/// The crank's speed, in radians a second, and its motor's torque: about
/// twenty times what turning the crank and rod against gravity takes.
const CRANK_SPEED: f32 = 2.0;
const CRANK_TORQUE: f32 = 2.0e8;
/// The servo arm's target, and its motor's torque: about fifteen times what
/// holding the arm level takes.
const SERVO_ANGLE: f32 = FRAC_PI_4;
const SERVO_TORQUE: f32 = 1.0e8;
/// The rope's length, in voxels, and the cone's half-angle.
const ROPE: f32 = 10.0;
const CONE: f32 = FRAC_PI_6;

/// A station for every kind of joint, to try each one by hand.
///
/// - Twelve bodies, which leaves four of the sixteen the renderer draws for `F`
///   drops and cut-off pieces.
/// - Every linear part and every angular part appears at least once.
/// - A joint to the world does not stop its body colliding with the world, so
///   every station hangs at least a voxel clear of the terrain it is fastened to.
pub(crate) fn joints() -> Scene {
    let tree = joint_world();
    let materials = palette();
    let m = &materials;
    let mut bodies = Vec::new();
    let mut joints = Vec::new();

    // A door hinged at its left edge beside the wall, with friction in the
    // hinge: push it and it swings, slows and stops.
    let door = brick(UVec3::new(8, 12, 1), Vec3::new(8.0, 7.0, 11.0), m);
    let hinge = Vec3::new(8.5, 13.0, 11.5);
    joints.push(
        Joint::new(&door, None, Linear::Point, Angular::Axis, hinge, hinge, Vec3::Y)
            .with_friction(Friction { force: 0.0, torque: DOOR_FRICTION }),
    );
    bodies.push(door);

    // A shelf welded to the wall: rigid until its pivot voxel is erased.
    let shelf = brick(UVec3::new(6, 1, 4), Vec3::new(40.0, 20.0, 11.0), m);
    let weld = Vec3::new(43.5, 20.5, 11.5);
    joints.push(Joint::new(&shelf, None, Linear::Point, Angular::Locked, weld, weld, Vec3::Y));
    bodies.push(shelf);

    // A servo arm on the wall, holding itself 45 degrees up: push it and it
    // comes back.
    let arm = brick(UVec3::new(8, 1, 1), Vec3::new(50.0, 26.0, 11.0), m);
    let shoulder = Vec3::new(50.5, 26.5, 11.5);
    joints.push(
        Joint::new(&arm, None, Linear::Point, Angular::Axis, shoulder, shoulder, Vec3::Z)
            .with_motor(Motor { drive: Drive::Target(SERVO_ANGLE), max: SERVO_TORQUE }),
    );
    bodies.push(arm);

    // A chain of three links from the beam, each hung from the bottom of the
    // one above.
    let links: Vec<Body> = (0..3)
        .map(|i| brick(UVec3::new(2, 4, 2), Vec3::new(12.0, 39.0 - 4.0 * i as f32, 44.0), m))
        .collect();
    let top = Vec3::new(13.0, 42.5, 45.0);
    joints.push(Joint::new(&links[0], None, Linear::Point, Angular::Free, top, top, Vec3::Y));
    for i in 1..3 {
        let meet = Vec3::new(13.0, 42.9 - 4.0 * i as f32, 45.0);
        joints.push(Joint::new(
            &links[i],
            Some(&links[i - 1]),
            Linear::Point,
            Angular::Free,
            meet,
            meet,
            Vec3::Y,
        ));
    }
    bodies.extend(links);

    // A bob on a rope from the beam, let go 45 degrees out: it swings, and
    // goes slack when lifted.
    let anchor = Vec3::new(31.5, 43.5, 45.0);
    let middle = anchor + Vec3::new(1.0, -1.0, 0.0).normalize() * ROPE;
    let bob = brick(UVec3::splat(3), middle - Vec3::splat(1.5), m);
    joints.push(Joint::new(&bob, None, Linear::Distance(ROPE), Angular::Free, middle, anchor, Vec3::Y));
    bodies.push(bob);

    // A pendulum kept within 30 degrees of hanging straight, pushed hard
    // enough to swing into the limit.
    let mut pendulum = brick(UVec3::new(2, 6, 2), Vec3::new(50.0, 37.0, 44.0), m);
    let pivot = Vec3::new(51.0, 42.5, 45.0);
    joints.push(Joint::new(&pendulum, None, Linear::Point, Angular::Cone(CONE), pivot, pivot, Vec3::NEG_Y));
    // Swinging about the pivot, the centre of mass moves at `ω × (com − pivot)`.
    let spin = Vec3::Z * 3.6;
    pendulum.set_angular_velocity(spin);
    pendulum.velocity = spin.cross(pendulum.position - pivot);
    bodies.push(pendulum);

    // A crank turned by a motor, driving a slider along a line through a rod
    // pinned to both.
    let crank = brick(UVec3::new(6, 1, 1), Vec3::new(20.0, 20.0, 26.0), m);
    let rod = brick(UVec3::new(12, 1, 1), Vec3::new(25.0, 20.0, 27.0), m);
    let slider = brick(UVec3::splat(3), Vec3::new(35.0, 19.0, 25.0), m);
    let hub = Vec3::new(20.5, 20.5, 26.5);
    joints.push(
        Joint::new(&crank, None, Linear::Point, Angular::Axis, hub, hub, Vec3::Z)
            .with_motor(Motor { drive: Drive::Speed(CRANK_SPEED), max: CRANK_TORQUE }),
    );
    let crank_pin = Vec3::new(25.5, 20.5, 27.5);
    joints.push(Joint::new(&rod, Some(&crank), Linear::Point, Angular::Free, crank_pin, crank_pin, Vec3::Y));
    let slider_pin = Vec3::new(36.5, 20.5, 27.5);
    joints.push(Joint::new(&rod, Some(&slider), Linear::Point, Angular::Free, slider_pin, slider_pin, Vec3::Y));
    let rail = Vec3::new(36.5, 20.5, 26.5);
    joints.push(Joint::new(&slider, None, Linear::Line, Angular::Locked, rail, rail, Vec3::X));
    bodies.extend([crank, rod, slider]);

    // A block on a friction joint to the world, with twice its weight in
    // friction: it hovers, and stays wherever it is pushed.
    let block = brick(UVec3::splat(3), Vec3::new(54.0, 14.0, 24.0), m);
    let held = Vec3::new(55.5, 15.5, 25.5);
    let weight = block.mass.mass * -GRAVITY.y;
    joints.push(
        Joint::new(&block, None, Linear::Free, Angular::Free, held, held, Vec3::Y)
            .with_friction(Friction { force: 2.0 * weight, torque: 0.0 }),
    );
    bodies.push(block);

    Scene {
        tree,
        materials,
        bodies,
        joints,
        eye: Vec3::new(32.0, 26.0, 63.0),
        look_at: Vec3::new(32.0, 20.0, 24.0),
    }
}

/// A floor; a wall along the back, z 8..10; and a beam across the front at
/// height 44 on two posts, z 44..46.
fn joint_world() -> Contree {
    let mut dense = DenseVolume::new(64).unwrap();
    let mut fill = |lo: [u32; 3], hi: [u32; 3]| {
        for z in lo[2]..hi[2] {
            for y in lo[1]..hi[1] {
                for x in lo[0]..hi[0] {
                    dense.set(UVec3::new(x, y, z), STONE);
                }
            }
        }
    };
    fill([0, 0, 0], [64, 6, 64]);
    fill([4, 6, 8], [60, 34, 10]);
    fill([4, 6, 44], [6, 44, 46]);
    fill([58, 6, 44], [60, 44, 46]);
    fill([4, 44, 44], [60, 46, 46]);
    dense.into_contree()
}

/// A solid brick box of `size` voxels, its low corner at `corner`, unturned,
/// with its mass computed.
fn brick(size: UVec3, corner: Vec3, materials: &MaterialTable) -> Body {
    let mut voxels = Vec::new();
    for z in 0..size.z {
        for y in 0..size.y {
            for x in 0..size.x {
                voxels.push((UVec3::new(x, y, z), BRICK));
            }
        }
    }
    let mut body = Body::new(Contree::from_voxels(16, &voxels), corner, Quat::IDENTITY);
    assert!(body.recompute(materials), "a scene body must have mass");
    body
}

#[cfg(test)]
mod tests {
    use super::*;
    use bevox_core::physics::Air;
    use bevox_core::distance_field::DistanceField;
    use bevox_core::physics::joint::follow;
    use bevox_core::physics::solver::step;
    use std::f32::consts::{PI, TAU};

    /// Every station of the joint scene does its job, headless, for ten
    /// seconds: no joint gives way, no body or joint is lost, the crank drives
    /// the slider, the servo holds its angle, and the friction block hovers.
    #[test]
    fn the_joint_scene_works() {
        let Scene { tree, materials, mut bodies, mut joints, .. } = joints();
        assert_eq!(bodies.len(), 12);
        let field = DistanceField::build(&tree);
        let index = |bodies: &[Body], id| bodies.iter().position(|b| b.id == id).unwrap();
        let slider = joints.iter().position(|j| j.linear == Linear::Line).unwrap();
        let servo = joints
            .iter()
            .position(|j| matches!(j.motor, Some(Motor { drive: Drive::Target(_), .. })))
            .unwrap();
        let hover = joints.iter().position(|j| j.linear == Linear::Free).unwrap();
        let crank = joints
            .iter()
            .position(|j| matches!(j.motor, Some(Motor { drive: Drive::Speed(_), .. })))
            .unwrap();
        let hover_y = bodies[index(&bodies, joints[hover].a)].position.y;
        let (mut low, mut high) = (f32::INFINITY, f32::NEG_INFINITY);
        // How far the crank has turned in all, unwrapped. Gravity alone swings
        // the crank and rod back and forth, which moves the slider too, but
        // from level it can never carry the crank over the top: only the motor
        // turns it round and round.
        let (mut turned, mut last) = (0.0f32, 0.0f32);
        for _ in 0..640 {
            step(&mut bodies, &tree, &field, &materials, Air::VACUUM, 1.0 / 64.0, None, &mut joints);
            follow(&mut joints, &bodies);
            let x = bodies[index(&bodies, joints[slider].a)].position.x;
            (low, high) = (low.min(x), high.max(x));
            let now = joints[crank].twist(&bodies[index(&bodies, joints[crank].a)], None);
            turned += (now - last + PI).rem_euclid(TAU) - PI;
            last = now;
        }
        assert!(turned > 2.0 * TAU, "the crank turned only {turned} rad in ten seconds");
        assert_eq!(bodies.len(), 12, "a body was lost");
        assert_eq!(joints.len(), 13, "a joint was dropped");
        for b in &bodies {
            assert!(b.position.is_finite() && b.orientation.is_finite(), "a body went to NaN");
        }
        for j in joints.iter().filter(|j| j.linear == Linear::Point) {
            let a = &bodies[index(&bodies, j.a)];
            let b = j.b.map(|id| &bodies[index(&bodies, id)]);
            let (pa, pb) = j.pivots(a, b);
            assert!((pa - pb).length() < 0.1, "a point joint opened by {}", (pa - pb).length());
        }
        assert!(high - low > 5.0, "the crank moved the slider only {} voxels", high - low);
        let twist = joints[servo].twist(&bodies[index(&bodies, joints[servo].a)], None);
        assert!((twist - SERVO_ANGLE).abs() < 0.05, "the servo is at {twist}, not {SERVO_ANGLE}");
        let y = bodies[index(&bodies, joints[hover].a)].position.y;
        assert!((y - hover_y).abs() < 0.05, "the friction block fell from {hover_y} to {y}");
    }

    /// CPU time for one tick of the joint scene. Not a gate; its number goes in
    /// the plan's Measurements section.
    #[test]
    #[ignore]
    fn a_joint_scene_tick_is_timed() {
        let Scene { tree, materials, mut bodies, mut joints, .. } = joints();
        let field = DistanceField::build(&tree);
        let mut tick = || {
            let start = std::time::Instant::now();
            step(&mut bodies, &tree, &field, &materials, Air::VACUUM, 1.0 / 64.0, None, &mut joints);
            start.elapsed().as_secs_f64() * 1000.0
        };
        for _ in 0..200 {
            tick();
        }
        let mut ms: Vec<f64> = (0..1000).map(|_| tick()).collect();
        ms.sort_by(|a, b| a.partial_cmp(b).unwrap());
        println!("joint scene, 12 bodies and 13 joints: median {:.4} ms/tick", ms[500]);
    }
}
