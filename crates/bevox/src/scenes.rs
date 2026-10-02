//! The scenes the number keys load: the demo scene, and a scene of joints to
//! try by hand.

use bevox_core::body::Body;
use bevox_core::contree::Contree;
use bevox_core::dense::DenseVolume;
use bevox_core::material::{Material, MaterialId, MaterialTable};
use bevox_physics::GRAVITY;
use bevox_physics::joint::{Angular, Drive, Friction, Joint, Linear, Motor};
use bevox_render::camera::start_camera;
use bevy::prelude::*;
use std::f32::consts::{FRAC_PI_4, FRAC_PI_6};
use bevox_physics::mass::recompute;

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
    recompute(&mut body, &materials);
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
///
/// **Strengths are impulses**, as `Material::strength` has been since fracture
/// stopped thresholding a closing speed. These were 45, 25 and 12 — speeds in
/// voxels a second — and nothing in the suite reads this palette, so they
/// survived the change in units and turned the demo into a scene where a body
/// landing anywhere set off a chain reaction that took the map apart.
///
/// They are rescaled by a single factor, which puts the weakest of them
/// exactly on `fixtures::GLASS_STRENGTH` — the one strength in this project
/// measured against what a stack actually carries — and leaves stone and brick
/// in the same proportion to it they always had.
///
/// The factor was `350_000 / 12`. It is now `500_000 / 12`: `GLASS_STRENGTH`
/// rose by 10/7 when the crush ceiling stopped binding it, and ice is pinned to
/// that constant by construction, so all three moved with it — 1_312_500 →
/// 1_875_000, 729_167 → 1_041_667, 350_000 → 500_000. The ordering ice < brick
/// < stone is what the proportion is for.
/// `the_demo_palette_outlasts_a_resting_stack` is the gate that was missing.
pub(crate) fn palette() -> MaterialTable {
    palette_with_crush(CRUSH_STONE, CRUSH_BRICK, CRUSH_ICE)
}

/// `palette`, with the crush column passed in, so a test can move one column
/// without touching the other.
fn palette_with_crush(stone: f32, brick: f32, ice: f32) -> MaterialTable {
    let mut materials = MaterialTable::new();
    materials
        .push(Material { color: [140, 140, 150, 255], density: 2600, friction: 60, restitution: 5, strength: 1_875_000.0, crush: stone })
        .unwrap(); // 1: stone
    materials
        .push(Material { color: [180, 90, 70, 255], density: 1900, friction: 70, restitution: 5, strength: 1_041_667.0, crush: brick })
        .unwrap(); // 2: brick
    materials
        .push(Material { color: [170, 210, 235, 255], density: 900, friction: 4, restitution: 10, strength: 500_000.0, crush: ice })
        .unwrap(); // 3: ice
    materials
        .push(Material {
            color: [40, 40, 45, 255],
            density: 1100,
            friction: 80,
            restitution: 80,
            strength: bevox_core::material::UNBREAKABLE,
            // Rubber is the one material here unbreakable in both columns,
            // which is what makes it the presser in `press_into`.
            crush: bevox_core::material::UNBREAKABLE,
        })
        .unwrap(); // 4: rubber
    materials
}

/// The force that crushes the demo's stone, measured 2026-10-02 as the geometric
/// middle of a four-high stone stack's held force (62,164,992) and what a mouse
/// grab presses into stone with (94,487,080) -- 1.23x each way, from a gap only
/// 1.52x wide.
pub(crate) const CRUSH_STONE: f32 = 76_640_000.0;

/// **Brick and ice cannot be calibrated yet, and this is a blocked item, not a
/// choice.** Both stay `UNBREAKABLE`, so a slow crush does not reach them.
///
/// The measurement that blocks it, from
/// `what_the_crush_column_must_sit_between`:
///
/// | | four-high held force | press ceiling | gap |
/// |---|---|---|---|
/// | stone | 62,164,992 | 94,487,080 | 1.52x |
/// | brick | 142,913,184 | 95,169,968 | **0.67x -- inverted** |
/// | ice | 43,003,908 | 116,325,592 | 2.71x |
///
/// Brick's floor sits *above* its ceiling: a four-high brick stack holds more
/// than a grab can press with, so any crush that reaches brick destroys a brick
/// stack standing still. And **none of the three stacks settle** -- 0 of 4
/// asleep in every row -- so these are not held loads at all but swinging piles,
/// which is the same stacking defect that keeps
/// `an_eight_high_glass_stack_stands` ignored and failing.
///
/// A threshold calibrated against a diverging pile bakes the divergence in.
/// Stone's number is taken only because its gap survives the noise; brick and
/// ice wait for stacking. Re-run the diagnostic when it is fixed.
pub(crate) const CRUSH_BRICK: f32 = f32::INFINITY;

/// See `CRUSH_BRICK`: blocked on the same measurement.
pub(crate) const CRUSH_ICE: f32 = f32::INFINITY;

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
    assert!(recompute(&mut body, materials), "a scene body must have mass");
    body
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Every breakable material the player actually meets outlasts a stack of
    /// itself standing still.
    ///
    /// This gate exists because its absence shipped a broken game. `strength`
    /// became an impulse when fracture stopped thresholding a closing speed,
    /// `fixtures::materials()` was recalibrated with it, and this palette was
    /// not — because no test read it. In the app, 45 against an impulse of
    /// 331_306 meant a body landing anywhere took the whole map apart, which is
    /// exactly what Flori saw and the suite did not.
    ///
    /// The floor is the resting load itself, measured on 2026-10-02 against a
    /// clean three-cube glass stack and recorded in
    /// `docs/concepts/fracture-load-window.md`. It is spelled here because
    /// `bevox_physics::fixtures` is `pub(crate)` and this crate cannot reach
    /// `GLASS_STRENGTH`; if the measurement is ever redone, both move together
    /// or this gate fires, which is the outcome worth having.
    #[test]
    fn the_demo_palette_outlasts_a_resting_stack() {
        /// The worst landing transient any glass stack reaches, measured: an
        /// eight-high stack, 414,166. A landing rather than a weight -- see
        /// `docs/concepts/fracture-load-window.md` -- but it is still the load
        /// a material must survive to be placed in a scene at all.
        ///
        /// **Raised from 331_306**, a three-cube stack, with
        /// `GLASS_STRENGTH`. The series is sub-linear in the number of cubes
        /// (about `n^0.55`: 126,129 / 246,227 / 331,306 / 356,751 / 391,195 /
        /// 414,166 for n = 1, 2, 3, 4, 6, 8), so clearing n = 8 clears stacks
        /// far taller than eight and there is no need to pick a height per
        /// scene. Three cubes was the figure only because it was the tallest
        /// stack that stood.
        ///
        /// **This gate is a floor and there is no ceiling.** A grab can press
        /// with at most about 5.9e5, so every material here except ice is
        /// silently uncrushable and nothing fails for it.
        ///
        /// **And it compares the raw table strength, which is the effective
        /// threshold only at or past `fracture::SIZE_CAP`; smaller bodies
        /// carry proportionally less.** Since Task 3 a sub-cap body's
        /// threshold is as low as a third of the table figure, so a body of
        /// eight voxels clears this floor on paper and not in the solver. The
        /// number stays as it is on purpose: self-weight load scales with
        /// voxel count while the threshold scales with its cube root, so a
        /// small body gets *safer* under its own weight, not more fragile.
        /// The sub-cap case this would have to cover is a small body holding
        /// up a large one, which no scene here builds.
        const RESTING_LOAD: f32 = 414_166.0;
        let materials = palette();
        let floor = RESTING_LOAD;
        for id in 1..=4 {
            let m = materials.get(MaterialId(id));
            if m.strength == bevox_core::material::UNBREAKABLE {
                continue;
            }
            assert!(
                m.strength >= floor,
                "material {id} has strength {} -- below the {floor} a resting stack already \
                 carries. A strength under that breaks under its own weight and the whole scene \
                 comes apart. Did the units change again?",
                m.strength,
            );
        }
    }
    /// The tick the app runs at, and the rate every strength and crush in
    /// this palette is calibrated at.
    const DT: f32 = 1.0 / 64.0;

    /// A slab of `material` filling y in 0..8, the floor a press leans on.
    fn floor_of(material: MaterialId) -> Contree {
        let mut dense = DenseVolume::new(64).unwrap();
        for z in 0..64 {
            for y in 0..8 {
                for x in 0..64 {
                    dense.set(UVec3::new(x, y, z), material);
                }
            }
        }
        dense.into_contree()
    }

    /// A four-voxel cube of `material` with its mass computed, centred at
    /// `centre`.
    ///
    /// Mass is computed at the origin and the position set afterwards, which is
    /// what `bevox_physics::fixtures::placed` does and the reason it matters:
    /// `recompute` reads the volume to find the centre of mass, so computing it
    /// at the final position leaves `com` offset by that position and the body
    /// lands nowhere near where the caller asked. Doing it the other way round
    /// put a four-high stack 2.2 voxels in the air and the pressed body
    /// entirely out of contact, which made a first attempt at the measurement
    /// below report a stack that broke at every crush and a press that broke at
    /// none.
    fn cube_of(material: MaterialId, centre: Vec3, materials: &MaterialTable) -> Body {
        let mut voxels = Vec::new();
        for z in 0..4 {
            for y in 0..4 {
                for x in 0..4 {
                    voxels.push((UVec3::new(x, y, z), material));
                }
            }
        }
        let mut body = Body::new(Contree::from_voxels(4, &voxels), Vec3::ZERO, Quat::IDENTITY);
        assert!(recompute(&mut body, materials), "a pressed body must have mass");
        body.position = centre;
        body
    }

    /// A rubber body held by a grab and crept down into a `floor` of the demo
    /// palette, for the same 6.25 seconds `bevox_physics`' own crush fixture
    /// runs. Returns what broke and the fastest the body moved before anything
    /// did.
    ///
    /// Rubber because it is the one material in this palette that is
    /// unbreakable in **both** columns, so only the floor can give way and what
    /// broke is never in question.
    ///
    /// The target creeps at a rate per *second*, so the press travels the same
    /// distance in the same time whatever `DT` is.
    fn press_into(floor: MaterialId, crush: f32) -> (Vec<bevox_physics::fracture::Fracture>, f32) {
        /// The 6.25 seconds `bevox_physics`' crush fixture covers.
        const SECONDS: f32 = 400.0 / 64.0;
        /// 0.002 a tick at 64 Hz, as a rate per second.
        const CREEP: f32 = 0.002 * 64.0;
        /// Rubber: `UNBREAKABLE` in both columns.
        const RUBBER: MaterialId = MaterialId(4);

        let materials = palette_with_crush(crush, crush, crush);
        let world = floor_of(floor);
        let field = DistanceField::build(&world);
        let centre = Vec3::new(32.0, 10.0, 32.0);
        let mut bodies = vec![cube_of(RUBBER, centre, &materials)];
        let mut grab = Joint::grab(&bodies[0], centre);

        let mut broke = Vec::new();
        let mut fastest = 0.0f32;
        for tick in 0..(SECONDS / DT).round() as u32 {
            grab.anchor_b = centre - Vec3::Y * (tick as f32 * DT * CREEP);
            // Only while nothing has broken: the tick that breaks the floor
            // hands impulse back and drops the body into the hole, which says
            // nothing about how gently it was pressing.
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
        (broke, fastest)
    }

    /// A four-high stack of `material`, settling onto a floor of itself, with
    /// `crush` on every material. Returns what broke and how many of the four
    /// ended asleep.
    ///
    /// Four cubes because that is the tallest glass stack `bevox_physics`
    /// measures that actually *settles* — six and eight never do, and a
    /// threshold calibrated against a diverging pile bakes the divergence in.
    fn stack_of(material: MaterialId, crush: f32) -> (Vec<bevox_physics::fracture::Fracture>, usize) {
        let materials = palette_with_crush(crush, crush, crush);
        let world = floor_of(material);
        let field = DistanceField::build(&world);
        let mut bodies: Vec<Body> = (0..4)
            .map(|i| {
                cube_of(material, Vec3::new(32.0, 10.2 + i as f32 * 4.0, 32.0), &materials)
            })
            .collect();
        let mut broke = Vec::new();
        for _ in 0..1200 {
            let out = step(
                &mut bodies,
                &world,
                &field,
                &materials,
                Air::VACUUM,
                DT,
                None,
                &mut [],
            );
            broke.extend(out.fractures);
        }
        let asleep = bodies.iter().filter(|b| b.asleep).count();
        (broke, asleep)
    }

    /// What the demo palette's crush column has to sit between, per material,
    /// measured rather than extrapolated from the fixture glass.
    ///
    /// Prints two bounds for each of stone, brick and ice:
    ///
    /// - **The floor**, the largest crush a four-high stack of it still breaks
    ///   under as it settles. The column must sit *above* this or the demo
    ///   terrain crushes itself the moment a scene loads.
    /// - **The ceiling**, the smallest crush a saturated grab pressing on it
    ///   can no longer break. The column must sit *below* this or a lean does
    ///   nothing, which is the whole defect `a_slow_crush_breaks_stone` exists
    ///   to catch.
    ///
    /// Both by bisection on the boolean "did anything break", because the force
    /// itself is only visible through `bevox_physics`' `#[cfg(test)]`
    /// `solver::trace`, which this crate cannot reach. A boolean bisection is
    /// the same measurement at one bit per step and it needs no instrument.
    ///
    /// `#[ignore]`d: it prints, asserts nothing, and takes about a minute.
    #[test]
    #[ignore]
    fn what_the_palette_crush_column_has_to_sit_between() {
        /// Halvings of a 1e10 bracket: 20 lands inside 0.01%.
        const STEPS: u32 = 20;
        let bisect = |breaks: &dyn Fn(f32) -> bool| -> f32 {
            let (mut lo, mut hi) = (1.0f32, 1.0e10f32);
            for _ in 0..STEPS {
                let mid = (lo * hi).sqrt();
                if breaks(mid) { lo = mid } else { hi = mid }
            }
            lo
        };
        println!(
            "{:7} | {:>16} | {:>16} | {:>7}",
            "mat", "stack floor", "press ceiling", "window"
        );
        for (name, id) in [("stone", STONE), ("brick", BRICK), ("ice", MaterialId(3))] {
            // Both brackets must straddle a real transition, or a bisection
            // reports an endpoint and it reads like a measurement. An
            // uncrushable stack must not break -- if it does, it is breaking on
            // the *impact* branch and the floor below is not about crush at
            // all -- and a press must break something at a crush of 1.
            assert!(
                stack_of(id, f32::INFINITY).0.is_empty(),
                "an uncrushable four-high {name} stack broke, so the floor below would be \
                 measuring the impact branch"
            );
            assert!(
                !press_into(id, 1.0).0.is_empty(),
                "a press did not break {name} at a crush of 1, so the ceiling below would be \
                 measuring whether the press touches at all"
            );
            let floor = bisect(&|c| !stack_of(id, c).0.is_empty());
            let ceiling = bisect(&|c| !press_into(id, c).0.is_empty());
            println!(
                "{name:7} | {floor:16.0} | {ceiling:16.0} | {:7.2}x",
                ceiling / floor
            );
            let (_, asleep) = stack_of(id, f32::INFINITY);
            println!("        (a four-high {name} stack: {asleep} of 4 asleep)");
        }
    }

    /// A player leaning a held body on the demo palette's **stone** crushes it.
    ///
    /// The user-visible half of the two-regime split. `bevox_physics`'
    /// `a_slow_crush_breaks_what_it_presses` proves the rule against the
    /// fixture glass, which no player ever meets; this proves it against the
    /// material the demo terrain is actually made of.
    ///
    /// It could not pass before the regime split and it is worth being precise
    /// about why. A crush used to be judged as an impulse against `strength`,
    /// and a press delivers a measured 365,906 as an impulse at 64 Hz. Stone's
    /// strength is 1,875,000 — five times that — so no press could ever mark
    /// it, and the only material in this palette a lean could break was ice.
    /// The press is now judged as a **force** against `crush`, which is a
    /// separate column calibrated against what a press actually delivers.
    #[test]
    /// Throwaway: prints the forces the crush column must sit between.
    ///
    /// A crush of 1.0 breaks everything, so every `Fracture::blow` on the held
    /// branch is the force that contact carried.
    #[test]
    #[ignore]
    fn what_the_crush_column_must_sit_between() {
        for (name, id) in [("stone", STONE), ("brick", MaterialId(2)), ("ice", MaterialId(3))] {
            let (pressed, _) = press_into(id, 1.0);
            let ceiling = pressed.iter().map(|f| f.blow).fold(0.0f32, f32::max);
            let (stacked, asleep) = stack_of(id, 1.0);
            let floor = stacked.iter().map(|f| f.blow).fold(0.0f32, f32::max);
            println!(
                "{name}: four-high held floor {floor:.0} ({asleep} asleep), press ceiling \
                 {ceiling:.0}, ratio {:.2}x, geometric middle {:.0}",
                ceiling / floor.max(1.0),
                (floor.max(1.0) * ceiling).sqrt(),
            );
        }
    }

    #[test]
    fn a_slow_crush_breaks_stone() {
        let (broke, fastest) = press_into(STONE, CRUSH_STONE);
        assert!(
            fastest < 1.0,
            "the body moved at {fastest} voxels a second before anything broke, so this is an \
             impact and not a crush"
        );
        assert!(
            !broke.is_empty(),
            "a body pressed into the demo palette's stone with up to {} of force, at under \
             {fastest:.3} voxels a second, broke nothing. Stone's `crush` is {} — is the \
             palette's crush column still `UNBREAKABLE`?",
            bevox_physics::GRAB_MAX_FORCE,
            palette().get(STONE).crush,
        );
    }

    use bevox_physics::Air;
    use bevox_core::distance_field::DistanceField;
    use bevox_physics::joint::follow;
    use bevox_physics::solver::step;
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
