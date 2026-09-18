# Dwyer's Joints Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking. **Flori prefers inline execution for this project.**

**Goal:** Rebuild joints after Dwyer's devlog #30: sixteen types, friction, motors, the mouse grab as a joint, and a key-swapped scene to try them all, with the J tool removed.

**Architecture:**
- A joint is one linear part (Free, Point, Line, Distance) and one angular part (Free, Locked, Axis, Cone).
- Every part is solved with Dwyer's one expression, `λ = −(J M⁻¹ Jᵀ)⁻¹ (J·V + bias)`, over its one to three rows at once.
- Friction and motors are further rows whose accumulated impulses are clamped.
- The grab is a capped Point + Locked joint to the world, solved with the others.
- The app loses the J tool and gains `1` and `2` to load and reset the demo and joint scenes.

**Tech Stack:** Rust stable 1.96, Bevy 0.19.1, glam 0.32. No new dependencies.

**Spec:** `docs/superpowers/specs/2026-09-15-bevox-rigid-bodies-design.md`, section "Joints and the mouse grab", milestone 7.

## Global Constraints

- Native desktop only. No web build, ever.
- `bevox_core` has no Bevy and no GPU dependency, and gains no new dependencies. No new dependencies in any crate.
- **Body transforms are rigid: rotation and translation, no scale.**
- Tests use a seeded `bevox_core::testing::XorShift64` wherever they need randomness. No test framework may be added.
- This plan changes no shader. **Bodies still cast no shadows**; that is a separate, later item and is not started here.
- **Physics stays off master.** Commit on `feat/rigid-body-physics`; push to its GitHub copy only when Flori asks; never delete that copy.
- Verify with `cargo test --workspace`. Redirect output to a file and check `$?`; never pipe cargo through `tee` or `tail`. A transient LNK1102 or LNK1104 gets one retry.
- Do not run the app. Flori tests it.
- Every correctness gate is proven by a deliberate break, which is then reverted.

## Decisions (Flori, 2026-09-18)

- **Every one of Dwyer's joint types, with motors:** a linear part from {free, point, line, distance} and an angular part from {free, locked, axis, cone}, plus motors driven to a speed or a target, with a maximum force.
- **Friction on every joint**, on the motion it leaves free. Free and Free with friction is the friction joint.
- **The grab is a joint, as Dwyer's is:** Point and Locked, capped absolutely so heavy bodies sag. It replaces the spring.
- **`1` and `2` swap scenes; pressing either again resets it.**
- **The J tool is removed.** Joints come only from scenes.
- **Approach A, revised:** each part is one block solve of Dwyer's expression. That keeps the 3×3 point solve.

## Facts that are not obvious from the code

**One function solves every part.** `block(k, rows, error)` takes:
- `k`, how a point (or a spin) answers an impulse in every direction;
- `rows`, a projector onto the part's one to three row directions;
- `error`, which is `J·V + bias` before projecting.

It solves `rows·k·rows`, padded with the identity on the directions the rows leave out. That padded matrix is block-diagonal: the solve on the rows' subspace is exactly `(J M⁻¹ Jᵀ)⁻¹`, and nothing comes out in the other directions. The projectors are:
- the identity, for 3 rows;
- `I − n nᵀ`, for 2 rows across `n`;
- `n nᵀ`, for 1 row along `n`.

**A Line pushes `b` at `a`'s anchor, not at its own.** The line's offset is measured at `a`'s anchor. Pushing `b` anywhere else makes the impulse pair twist the two bodies, which creates angular momentum out of nothing. Joint friction and a line's motor act on `b` at `a`'s anchor for the same reason. The momentum gate is what checks this.

**One-sided rows are speculative, as contacts are.** A rope or cone that is slack by `c` may close exactly `c` this substep. Past its limit, `BIAS` of the excess is corrected. Their accumulated impulse only ever resists: `≤ 0` along the row.

**Warm-start impulses are world vectors.** A part's rows turn with the bodies, so a number per row would mean nothing by the next substep. This is unchanged from milestone 6.

**The grab is left out of the relax pass.** Its target moves, and the bias is how that motion reaches the body. Relaxing it would stop the body dead every substep, and letting go would throw nothing. Everything else is relaxed as before.

**The grab's torque cap is its force at two voxels.** That is enough to hold the demo cube level by a corner: `4.1e5 · 98 · 4.2 ≈ 1.7e8`, against a cap of `3.9e8`. It is weak enough that a grab more than two voxels from a hinge swings the door instead of locking it.

**A joint to the world still collides with the world.** Only body–body pairs skip contacts. The scene hangs every station at least a voxel clear of the terrain it is fastened to.

**Mass units are density per voxel.** A voxel of density 1000 weighs 1000. `GRAB_MAX_FORCE` is the weight of 2,000,000, which is 2,000 voxels of density 1000:
- the demo cube (216 voxels at 1900, 4.1e5) lifts;
- a column you cut free (1280 voxels at 2600, 3.3e6) does not.

**The 16-type gate kicks each body by hand, not at random.** Each kick is chosen so every free motion is exercised; a random kick might not exercise one, and the gate would then prove nothing. No randomness is used, so no XorShift64.

## File Structure

- `crates/bevox_core/src/physics/joint.rs`: rewritten.
  - The parts, `Joint`, the block solve, friction, motors and the grab constructor.
  - `follow`, unchanged.
  - The new gates.
- `crates/bevox_core/src/physics/solver.rs`:
  - `step` takes `grab: Option<&mut Joint>`, and the grab is solved with the joints.
  - The spring grab is deleted.
  - The ported joint tests.
- `crates/bevox_core/src/physics/mod.rs`: `GRAB_MAX_FORCE` and `GRAB_MAX_TORQUE` replace the spring constants; `energy` moves into the fixtures.
- `crates/bevox/src/scenes.rs` (**new**): `SceneKind`, `Scene`, the demo scene (moved from `main.rs`) and the joint scene.
- `crates/bevox/src/main.rs`:
  - the J tool is removed;
  - the grab is held as a joint;
  - `scene_keys` is added.

---

### Task 1: Remove the J tool

**Files:**
- Modify: `crates/bevox/src/main.rs`

**Interfaces:**
- Produces: `fn title(grab: &GrabState) -> String`. Task 5 extends it with the scene.

- [ ] **Step 1: Delete the tool.** In `crates/bevox/src/main.rs`:
  - Remove `.init_resource::<JointState>()` and the two `add_systems` lines for `toggle_joint_mode` and `joint_input`.
  - Delete `JointState`, its `Default` impl, `toggle_joint_mode`, `joint_click` and `joint_input`.
  - Delete the tests `two_clicks_make_a_joint` and `joint_and_grab_modes_exclude_each_other`, and the `world.init_resource::<JointState>();` line in `g_toggles_grab_mode`.
  - In `brush_input`, remove the `joint: Res<JointState>` parameter, and change the paint line to:

    ```rust
        // In grab mode the left mouse grabs; only the erase is left here.
        let paint = buttons.just_pressed(MouseButton::Left) && !grab.enabled;
    ```
  - Replace `toggle_grab_mode` and `title` with:

    ```rust
    fn toggle_grab_mode(
        keys: Res<ButtonInput<KeyCode>>,
        mut state: ResMut<GrabState>,
        mut windows: Query<&mut Window>,
    ) {
        if !keys.just_pressed(KeyCode::KeyG) {
            return;
        }
        state.enabled = !state.enabled;
        state.held = None;
        if let Ok(mut window) = windows.single_mut() {
            window.title = title(&state);
        }
    }

    /// The window title, naming the tool in use.
    fn title(grab: &GrabState) -> String {
        if grab.enabled { "BEVOX \u{2014} grab mode (G)".into() } else { "BEVOX".into() }
    }
    ```
  - Imports: drop `JointKind` and `Hit`, and `BodyId` if the compiler reports it unused.

- [ ] **Step 2: Run the tests.**

Run: `cargo test --workspace > $SCRATCH/t1.txt 2>&1; echo $?`
Expected: `0`, with no warnings. Joints still exist (`Joints`, `follow`), but nothing in the app makes one.

- [ ] **Step 3: Commit.**

```bash
git add crates/bevox/src/main.rs
git commit -m "refactor(app): remove the J tool; joints will come from scenes"
```

---

### Task 2: Sixteen joint types, solved with Dwyer's expression

**Files:**
- Modify: `crates/bevox_core/src/physics/joint.rs` (everything above `follow`, and the tests)
- Modify: `crates/bevox_core/src/physics/solver.rs` (the ported tests)
- Modify: `crates/bevox_core/src/physics/mod.rs` (`energy` moves into the fixtures)

**Interfaces:**
- Produces:
  - `pub enum Linear { Free, Point, Line, Distance(f32) }`
  - `pub enum Angular { Free, Locked, Axis, Cone(f32) }`
  - `pub struct Carried { pub linear: Vec3, pub angular: Vec3 }`
  - `pub struct Joint { pub a, pub b, pub linear, pub angular, pub anchor_a, pub anchor_b, pub axis_a, pub axis_b, pub rest: Quat, pub carried: Carried }`
  - `Joint::new(a: &Body, b: Option<&Body>, linear: Linear, angular: Angular, at_a: Vec3, at_b: Vec3, axis: Vec3) -> Joint`
  - `Joint::pivots(&self, a, b) -> (Vec3, Vec3)`, `Joint::axes(&self, a, b) -> (Vec3, Vec3)`, `Joint::twist(&self, a, b) -> f32`, `Joint::slide(&self, a, b) -> f32`
  - `pub(crate) fn warm_start(a, b, &Joint)`, `pub(crate) fn solve(a, b, &mut Joint, inv_h, use_bias)`, `pub fn follow` (unchanged)
  - `fixtures::energy(bodies: &[Body]) -> f32`
- `JointKind` is deleted.

- [ ] **Step 1: Move `energy` into the fixtures.** Cut `energy` from `solver.rs`'s tests and add it to `fixtures` in `mod.rs`:

```rust
    /// Total mechanical energy: motion, spin, and height in gravity.
    pub fn energy(bodies: &[Body]) -> f32 {
        bodies
            .iter()
            .map(|b| {
                0.5 * b.mass.mass * b.velocity.length_squared()
                    + 0.5 * b.angular_velocity().dot(b.angular_momentum)
                    + b.mass.mass * -crate::physics::GRAVITY.y * b.position.y
            })
            .sum()
    }
```

In `solver.rs`'s tests, add `energy` to the `use crate::physics::fixtures::{...}` list.

- [ ] **Step 2: Write the failing gates**, in `joint.rs`'s tests module. Its `use` block becomes:

```rust
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
    /// motion is used:
    /// - along the line, x;
    /// - up past a rope's anchor, so the rope goes slack;
    /// - spinning on every axis, which includes the hinge's.
    ///
    /// Two kinds of failure are caught:
    /// - a part that gives way: any violation over the limit;
    /// - a part that holds too much: a free motion that never happened.
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
                let at_a = Vec3::new(31.5, 30.5, 30.5);
                let at_b = match linear {
                    Linear::Distance(_) => Vec3::new(33.0, 30.5, 30.5),
                    _ => at_a,
                };
                let axis = Vec3::new(0.3, 1.0, 0.2);
                let mut joints =
                    vec![Joint::new(&bodies[0], Some(&bodies[1]), linear, angular, at_a, at_b, axis)];
                bodies[0].velocity = Vec3::new(0.0, 6.0, -3.0);
                bodies[0].set_angular_velocity(Vec3::new(1.0, 2.0, 0.0));
                bodies[1].velocity = Vec3::new(-2.0, 0.0, 1.0);
                let (p0, l0) = momentum(&bodies);
                for _ in 0..200 {
                    space.step(&mut bodies, &mut joints, Vec3::ZERO);
                }
                let (p, l) = momentum(&bodies);
                assert!((p - p0).length() < 1e-3 * p0.length(), "{name}: momentum {p:?}, was {p0:?}");
                assert!((l - l0).length() < 1e-3 * l0.length(), "{name}: angular momentum {l:?}, was {l0:?}");
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
                assert!(
                    highest - start < 0.02 * scale,
                    "{name}: gained energy, {start} rose to {highest}"
                );
            }
        }
    }
```

Port the existing `follow` tests to the new constructor. In each one, replace `Joint::new(JointKind::Ball, &bodies[0], None, X, Vec3::Y)` with `ball(&bodies[0], None, X)`.

- [ ] **Step 3: Run them to see them fail.**

Run: `cargo test -p bevox_core --lib joint > $SCRATCH/t2a.txt 2>&1; echo $?`
Expected: a compile failure, because `Linear`, `Angular` and the new `Joint::new` do not exist.

- [ ] **Step 4: Rewrite `joint.rs` above `follow`.** Keep `follow` as it is, and its doc comment:

```rust
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

use super::BIAS;
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

/// What a joint carried last substep, for warm starting. In world axes: a
/// part's rows turn with the bodies, so a number per row would mean nothing by
/// the next substep.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Carried {
    pub linear: Vec3,
    pub angular: Vec3,
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
            carried: Carried::default(),
        }
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
/// directions the rows leave out with the identity keeps it invertible and
/// keeps them out of the answer. `error` is `J·V + bias`, before projecting.
fn block(k: Mat3, rows: Mat3, error: Vec3) -> Vec3 {
    let padded = rows * k * rows + (Mat3::IDENTITY - rows);
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
pub(crate) fn warm_start(a: &mut Body, b: Option<&mut Body>, joint: &Joint) {
    let (pa, pb) = joint.pivots(a, b.as_deref());
    let rb = lever(b.as_deref(), b_point(joint, pa, pb));
    apply(a, b, pa - a.position, rb, joint.carried.linear, joint.carried.angular);
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
    joint.carried.linear = match joint.linear {
        Linear::Free => return,
        Linear::Point => before + block(k, Mat3::IDENTITY, v + d * bias_rate(use_bias, inv_h)),
        Linear::Line => {
            let n = joint.axes(a, b.as_deref()).1;
            before + block(k, Mat3::IDENTITY - outer(n), v + d * bias_rate(use_bias, inv_h))
        }
        Linear::Distance(max) => {
            let length = d.length();
            if length < 1e-6 {
                return;
            }
            let n = d / length;
            let lambda = block(k, outer(n), v + n * one_sided_bias(length - max, inv_h, use_bias)).dot(n);
            // A rope pulls, never pushes.
            n * (before.dot(n) + lambda).min(0.0)
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
    joint.carried.angular = match joint.angular {
        Angular::Free => return,
        Angular::Locked => {
            // How far `a` has turned from where `rest` puts it, as a rotation
            // vector.
            let off = a.orientation * (orientation(b.as_deref()) * joint.rest).inverse();
            let off = if off.w < 0.0 { -off } else { off };
            before + block(k, Mat3::IDENTITY, spin + off.xyz() * (2.0 * bias_rate(use_bias, inv_h)))
        }
        Angular::Axis => {
            // Turning `a` about `wa × wb` brings its axis toward `b`'s.
            before + block(k, Mat3::IDENTITY - outer(wa), spin + wb.cross(wa) * bias_rate(use_bias, inv_h))
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
            let lambda = block(k, outer(u), spin + u * one_sided_bias(apart - max, inv_h, use_bias)).dot(u);
            // A cone only ever pushes the axes back together.
            u * (before.dot(u) + lambda).min(0.0)
        }
    };
    apply(a, b, Vec3::ZERO, Vec3::ZERO, Vec3::ZERO, joint.carried.angular - before);
}
```

- [ ] **Step 5: Port the solver's joint tests.** In `solver.rs`'s tests module, add:

```rust
    use crate::physics::joint::{Angular, Joint, Linear};

    /// A ball joint: a Point with the angular part free.
    fn ball(a: &Body, b: Option<&Body>, at: Vec3) -> Joint {
        Joint::new(a, b, Linear::Point, Angular::Free, at, at, Vec3::Y)
    }
```

Then:
- Remove every `use crate::physics::joint::{Joint, JointKind};` inside a test.
- Replace `Joint::new(JointKind::Ball, A, B, P, Vec3::Y)` with `ball(A, B, P)`.
- Replace `Joint::new(JointKind::Hinge, &bodies[0], None, hinge, Vec3::Y)` with `Joint::new(&bodies[0], None, Linear::Point, Angular::Axis, hinge, hinge, Vec3::Y)`.
- Change `opening`'s parameter type to `&Joint`.
- Delete `a_joint_conserves_momentum`: `every_joint_conserves_momentum` covers it, angular momentum included.

- [ ] **Step 6: Run the tests.**

Run: `cargo test --workspace > $SCRATCH/t2b.txt 2>&1; echo $?`
Expected: `0`. That means all 16 combinations, momentum, energy, the ported pendulum, chain, door and no-collision tests, and `follow`.

- [ ] **Step 7: The deliberate breaks.** Each one on its own, reverted before the next.
  1. For the Line rows, use one row instead of two: `outer(n.any_orthonormal_vector())` in place of `Mat3::IDENTITY - outer(n)`. `every_joint_holds_what_it_should_and_frees_the_rest` must FAIL, on the Line linear part.
  2. Make `Angular::Free` solve as `Locked`: change `Angular::Free => return,` to fall through to the Locked arm. The same gate must FAIL, on "held what it frees".
  3. In `apply`, leave `b` out: delete its `if let Some(b)` block. `every_joint_conserves_momentum` must FAIL.
  4. Make `b_point` return `pb` for a line too. `every_joint_conserves_momentum` must FAIL on angular momentum for Line.
  5. In `step`'s relax pass, call `solve_joints(..., true)` instead of `false`. `no_joint_gains_energy` should FAIL. If it passes, record this break as blind in "What changed during execution" and try break 6.
  6. Only if break 5 was blind: in `warm_start`, apply the carried impulses twice. `no_joint_gains_energy` must FAIL.

- [ ] **Step 8: Commit.**

```bash
git add crates/bevox_core/src/physics/joint.rs crates/bevox_core/src/physics/solver.rs crates/bevox_core/src/physics/mod.rs
git commit -m "feat(core): sixteen joint types, each part solved with Dwyer's expression"
```

---

### Task 3: Joint friction and motors

**Files:**
- Modify: `crates/bevox_core/src/physics/joint.rs`

**Interfaces:**
- Consumes: Task 2's `Joint`, `block`, `apply`, `lever`, `linear_mass`, `angular_mass`, `relative_velocity`, `relative_spin`, `wrap`.
- Produces:
  - `pub struct Friction { pub force: f32, pub torque: f32 }` (Default is none)
  - `pub enum Drive { Speed(f32), Target(f32) }`
  - `pub struct Motor { pub drive: Drive, pub max: f32 }`
  - `Carried` gains `linear_friction: Vec3`, `angular_friction: Vec3` and `motor: f32`.
  - `Joint` gains `pub friction: Friction` and `pub motor: Option<Motor>`.
  - `Joint::with_friction(self, Friction) -> Self` and `Joint::with_motor(self, Motor) -> Self`. `with_motor` panics unless the joint has exactly one of a Line and an Axis part.

- [ ] **Step 1: Write the failing gates**, in `joint.rs`'s tests:

```rust
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
        // about 3.1e6.
        // 5%: the point rows, solved after the motor, move the spin a little
        // each pass while gravity pulls on the bar.
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
        let mut joints = vec![joint.with_motor(Motor { drive: Drive::Target(target), max: 1.0e8 })];
        for _ in 0..300 {
            space.step(&mut bodies, &mut joints, GRAVITY);
        }
        let at = joints[0].twist(&bodies[0], None);
        assert!((at - target).abs() < 0.02, "held at {at}, not {target}");

        bodies[0].set_angular_velocity(Vec3::Z * -5.0);
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
        let _ = ball(&body, None, body.position)
            .with_motor(Motor { drive: Drive::Speed(1.0), max: 1.0 });
    }
```

In `every_joint_conserves_momentum`, give every joint friction and, where it can have one, a motor, so that both are proven internal:

```rust
                let mut joint = Joint::new(&bodies[0], Some(&bodies[1]), linear, angular, at_a, at_b, axis)
                    .with_friction(Friction { force: 1.0e5, torque: 1.0e5 });
                if (linear == Linear::Line) != (angular == Angular::Axis) {
                    joint = joint.with_motor(Motor { drive: Drive::Speed(1.0), max: 1.0e6 });
                }
                let mut joints = vec![joint];
```

- [ ] **Step 2: Run them to see them fail.**

Run: `cargo test -p bevox_core --lib joint > $SCRATCH/t3a.txt 2>&1; echo $?`
Expected: a compile failure, because `Friction`, `Motor` and `Drive` do not exist.

- [ ] **Step 3: Add the types**, after `Angular`:

```rust
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
```

Extend `Carried`:

```rust
pub struct Carried {
    pub linear: Vec3,
    pub angular: Vec3,
    pub linear_friction: Vec3,
    pub angular_friction: Vec3,
    /// Along the line or about the axis the motor drives.
    pub motor: f32,
}
```

Add `pub friction: Friction,` and `pub motor: Option<Motor>,` to `Joint`, after `rest`. In `Joint::new`, add `friction: Friction::default(),` and `motor: None,`. Then add to `impl Joint`:

```rust
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
```

- [ ] **Step 4: Solve them.** Replace `warm_start`, and add the motor and friction solves ahead of the parts in `solve`:

```rust
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
```

In `solve`, before the part order:

```rust
    // Velocity goals first, as in Box2D v3: they run in the relax pass too,
    // because they are not drift corrections.
    solve_motor(a, b.as_deref_mut(), joint, inv_h);
    solve_friction(a, b.as_deref_mut(), joint, inv_h);
```

And add:

```rust
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
        let lambda = block(linear_mass(a, b.as_deref(), ra, rb), outer(wb), wb * (speed - want)).dot(wb);
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
```

- [ ] **Step 5: Run the tests.**

Run: `cargo test --workspace > $SCRATCH/t3b.txt 2>&1; echo $?`
Expected: `0`.

- [ ] **Step 6: The deliberate breaks.** Each one on its own, reverted.
  1. The door's friction torque set to `0.0` in `friction_stops_a_swinging_door`: it must FAIL on "still turning".
  2. `clamp_add`'s limit ignored (`f32::INFINITY`): `joint_friction_holds_up_to_its_force` must FAIL, because the weak block hangs.
  3. `clamp_scalar`'s limit ignored: `a_speed_motor_drives_and_stalls` must FAIL, because the weak motor lifts the bar.
  4. `wrap(...)` removed from the Target arm: `a_target_motor_takes_the_short_way_round` must FAIL.
  5. In `solve_friction`, pass `pb` in place of `pa` to `lever` for `b`: `every_joint_conserves_momentum` must FAIL on angular momentum, for Distance.

- [ ] **Step 7: Commit.**

```bash
git add crates/bevox_core/src/physics/joint.rs
git commit -m "feat(core): joint friction and motors, speed and target"
```

---

### Task 4: The grab as a joint

**Files:**
- Modify: `crates/bevox_core/src/physics/joint.rs`, `crates/bevox_core/src/physics/solver.rs`, `crates/bevox_core/src/physics/mod.rs`
- Modify: `crates/bevox/src/main.rs`
- Modify: `docs/superpowers/specs/2026-09-15-bevox-rigid-bodies-design.md` (one bullet)

**Interfaces:**
- Produces:
  - `Joint` gains `pub max_force: f32` and `pub max_torque: f32`. `new` sets both to `f32::INFINITY`.
  - `Joint::grab(body: &Body, at: Vec3) -> Joint`. The target is `anchor_b`, a world point.
  - `step(bodies, tree, field, materials, gravity, dt, grab: Option<&mut Joint>, joints: &mut [Joint]) -> bool`
  - `physics::GRAB_MAX_FORCE`, `physics::GRAB_MAX_TORQUE`
  - `GrabState { enabled: bool, held: Option<Joint>, distance: f32 }`
- Deleted:
  - `solver::Grab` and `solver::pull`;
  - `GRAB_FREQUENCY`, `GRAB_DAMPING`, `GRAB_SPIN_DAMPING` and `GRAB_MAX_ACCEL`.

- [ ] **Step 1: Write the failing gates.** In `joint.rs`'s tests, add a grab-aware step to `Space`:

```rust
        fn step_holding(&self, bodies: &mut Vec<Body>, grab: &mut Joint, gravity: Vec3) {
            step(bodies, &self.world, &self.field, &self.materials, gravity, DT, Some(grab), &mut []);
        }
```

and the gates:

```rust
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
        // The target moved 0.1 a tick: 6.4 voxels a second.
        let carried = bodies[0].velocity.x;
        assert!((carried - 6.4).abs() < 0.2 * 6.4, "carried at {carried}, the target at 6.4");
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
```

In `solver.rs`'s tests, delete the four spring tests:
- `a_grabbed_body_settles_at_the_target`
- `a_body_held_by_its_corner_hangs_below_it`
- `a_far_target_cannot_yank_a_body`
- `a_grab_on_a_missing_body_does_nothing`

The joint versions above replace them.

- [ ] **Step 2: Run them to see them fail.**

Run: `cargo test -p bevox_core --lib joint > $SCRATCH/t4a.txt 2>&1; echo $?`
Expected: a compile failure, because `Joint::grab` does not exist.

- [ ] **Step 3: The caps and the grab**, in `joint.rs`:
  - Add `pub max_force: f32,` and `pub max_torque: f32,` to `Joint`, after `motor`, with the doc comment `/// The most the linear part pulls with, and the angular part turns with. Unlimited unless set.`
  - `new` sets both to `f32::INFINITY`.
  - Add to `impl Joint`:

```rust
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
```

with `use super::{BIAS, GRAB_MAX_FORCE, GRAB_MAX_TORQUE};`. In `solve_linear`, cap every arm's total:
- Point: `(before + block(...)).clamp_length_max(joint.max_force / inv_h)`
- Line: the same.
- Distance: `n * (before.dot(n) + lambda).clamp(-joint.max_force / inv_h, 0.0)`

In `solve_angular`, do the same with `max_torque`, for Locked, Axis and Cone.

- [ ] **Step 4: The constants**, in `mod.rs`. Replace the four `GRAB_*` spring constants with:

```rust
/// The most force the mouse grab pulls with: the weight of 2,000 voxels of
/// density 1000. A body heavier than that sags and drags rather than lifts.
pub const GRAB_MAX_FORCE: f32 = 2_000.0 * 1000.0 * 9.81 / VOXEL_METRES;

/// The most torque the grab turns a body with: its force at two voxels.
/// Enough to hold the demo cube level by a corner, and weak enough that a grab
/// more than two voxels from a hinge swings the door rather than locking it.
pub const GRAB_MAX_TORQUE: f32 = GRAB_MAX_FORCE * 2.0;
```

- [ ] **Step 5: The solver.** In `solver.rs`:
  - Delete `Grab`, its `impl`, and `pull`. Drop the spring constants from the imports.
  - Change `step`'s parameter to `grab: Option<&mut Joint>`.
  - Drop the `pull` call in the substep loop.
  - Build the joint list with the grab last, after `jointed` is computed and before `links`:

```rust
    // The grab is one more joint, to the world, last in the list, so the relax
    // pass can leave it out.
    let scene_joints = joints.len();
    let mut joints: Vec<&mut Joint> = joints.iter_mut().chain(grab).collect();
```

  - `links` maps over `joints.iter()`.
  - `solve_joints` takes `joints: &mut [&mut Joint]`.
  - The relax call becomes:

```rust
        // The grab is not relaxed. Its target moves, and the bias is how that
        // motion reaches the body: relaxing it would stop the body dead every
        // substep, and letting go would throw nothing.
        solve_joints(bodies, &mut joints[..scene_joints], &links[..scene_joints], inv_h, false);
```

- [ ] **Step 6: The app.** In `crates/bevox/src/main.rs`:
  - Import `bevox_core::physics::joint::{Joint, follow}` and `bevox_core::physics::solver::step`.
  - `GrabState.held` becomes `Option<Joint>`.
  - In `grab_input`:
    - the held-body check reads `held.a`;
    - the click makes `Joint::grab(&scene.bodies[i], hit.position)`;
    - the drag sets `held.anchor_b = target`.
  - Its doc comment: "a joint holds the clicked point at a point on the cursor's ray, as far away as it was when clicked, and keeps the body's orientation. The wheel moves it nearer or farther; letting go drops it, with whatever momentum it has."
  - `physics_system` takes `mut grab: ResMut<GrabState>` and passes `grab.held.as_mut()`.
  - Tests:
    - `g_toggles_grab_mode` sets `held` to `Some(Joint::grab(&body, Vec3::ZERO))`, where `body` is a `demo_body` recomputed with `demo_scene().1`.
    - `the_physics_system_pulls_a_held_body` builds `let mut grab = Joint::grab(&body, body.position); grab.anchor_b = body.position + Vec3::new(10.0, 0.0, 0.0);`.

- [ ] **Step 7: The spec.** In "Joints and the mouse grab", add to the grab's bullets:

```markdown
- It is left out of the relax pass. Its target moves, and the bias is how that
  motion reaches the body; relaxing it would stop the body dead every substep,
  and letting go would throw nothing.
```

- [ ] **Step 8: Run the tests.**

Run: `cargo test --workspace > $SCRATCH/t4b.txt 2>&1; echo $?`
Expected: `0`, with no warnings.

- [ ] **Step 9: The deliberate breaks.** Each one on its own, reverted.
  1. `Joint::grab` with `Angular::Free`: `a_grab_holds_the_point_and_the_pose` must FAIL on "turned".
  2. `GRAB_MAX_FORCE` ignored (`max_force: f32::INFINITY` in `grab`): `a_grab_lifts_a_light_body_but_not_a_heavy_one` and `a_far_target_pulls_with_the_capped_force` must FAIL.
  3. The relax pass over all of `joints`, grab included: `letting_go_keeps_the_momentum` must FAIL.

- [ ] **Step 10: Commit.**

```bash
git add crates/bevox_core/src/physics crates/bevox/src/main.rs docs/superpowers/specs/2026-09-15-bevox-rigid-bodies-design.md
git commit -m "feat: the mouse grab is a capped joint holding point and pose"
```

---

### Task 5: The scenes, and the keys that swap them

**Files:**
- Create: `crates/bevox/src/scenes.rs`
- Modify: `crates/bevox/src/main.rs`

**Interfaces:**
- Consumes: `Joint::new`, `with_friction`, `with_motor`, `Linear`, `Angular`, `Friction`, `Motor`, `Drive` and `Joint::twist` (Tasks 2 and 3).
- Produces:
  - `pub(crate) enum SceneKind { Demo, Joints }` (Resource), with `build(self) -> Scene` and `name(self) -> &'static str`
  - `pub(crate) struct Scene { tree, materials, bodies, joints, eye, look_at }`
  - `scenes::{demo, joints, with_falling_body, demo_scene, demo_body, palette}`
  - `fn scene_keys(...)` (system) and `fn title(grab: &GrabState, scene: SceneKind) -> String`

- [ ] **Step 1: Move the demo scene.**
  - Move `demo_scene` and `demo_body` from `main.rs` into a new `crates/bevox/src/scenes.rs`, both as `pub(crate)`.
  - Split the materials out of `demo_scene` into `pub(crate) fn palette() -> MaterialTable`, which `demo_scene` calls.
  - In `main.rs`, add `mod scenes;` and `use scenes::{Scene, SceneKind, demo_body};`. The tests module adds `use crate::scenes::demo_scene;`.

- [ ] **Step 2: Write the failing gates.** In `scenes.rs`:

```rust
#[cfg(test)]
mod tests {
    use super::*;
    use bevox_core::distance_field::DistanceField;
    use bevox_core::physics::joint::follow;
    use bevox_core::physics::solver::step;

    /// Every station of the joint scene does its job, headless, for ten
    /// seconds:
    /// - no joint gives way;
    /// - no body or joint is lost;
    /// - the crank drives the slider;
    /// - the servo holds its angle;
    /// - the friction block hovers.
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
        let hover_y = bodies[index(&bodies, joints[hover].a)].position.y;
        let (mut low, mut high) = (f32::INFINITY, f32::NEG_INFINITY);
        for _ in 0..640 {
            step(&mut bodies, &tree, &field, &materials, GRAVITY, 1.0 / 64.0, None, &mut joints);
            follow(&mut joints, &bodies);
            let x = bodies[index(&bodies, joints[slider].a)].position.x;
            (low, high) = (low.min(x), high.max(x));
        }
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
            step(&mut bodies, &tree, &field, &materials, GRAVITY, 1.0 / 64.0, None, &mut joints);
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
```

In `main.rs`'s tests:

```rust
    /// `2` loads the joint scene, and pressing it again resets it; `1` goes back
    /// to the demo. Each load asks for a rebuild and lets go of any grab.
    #[test]
    fn number_keys_load_and_reset_scenes() {
        let mut world = World::new();
        let demo = scenes::demo();
        let field = DistanceField::build(&demo.tree);
        let held = Joint::grab(&demo.bodies[0], demo.bodies[0].position);
        world.insert_resource(VoxelScene {
            tree: demo.tree,
            materials: demo.materials,
            generation: 1,
            field,
            field_dirty: None,
            bodies: demo.bodies,
        });
        world.insert_resource(GrabState { enabled: true, held: Some(held), distance: 10.0 });
        world.init_resource::<Joints>();
        world.init_resource::<SceneKind>();
        let keys = world.register_system(scene_keys);
        let press = |world: &mut World, key: KeyCode| {
            let mut input = ButtonInput::<KeyCode>::default();
            input.press(key);
            world.insert_resource(input);
            world.run_system(keys).unwrap();
        };

        press(&mut world, KeyCode::Digit2);
        assert_eq!(world.resource::<VoxelScene>().bodies.len(), 12);
        assert_eq!(world.resource::<VoxelScene>().generation, 2, "loading did not ask for a rebuild");
        assert_eq!(world.resource::<Joints>().0.len(), 13);
        assert!(world.resource::<GrabState>().held.is_none(), "a grab outlived its scene");
        assert_eq!(*world.resource::<SceneKind>(), SceneKind::Joints);

        let fresh = scenes::joints().bodies[0].position;
        world.resource_mut::<VoxelScene>().bodies[0].position += Vec3::splat(5.0);
        press(&mut world, KeyCode::Digit2);
        assert_eq!(world.resource::<VoxelScene>().bodies[0].position, fresh, "2 did not reset");
        assert_eq!(world.resource::<VoxelScene>().generation, 3);

        press(&mut world, KeyCode::Digit1);
        assert_eq!(world.resource::<VoxelScene>().bodies.len(), 1);
        assert!(world.resource::<Joints>().0.is_empty());
        assert_eq!(*world.resource::<SceneKind>(), SceneKind::Demo);
    }
```

- [ ] **Step 3: Run them to see them fail.**

Run: `cargo test -p bevox > $SCRATCH/t5a.txt 2>&1; echo $?`
Expected: a compile failure, because `joints`, `Scene` and `scene_keys` do not exist.

- [ ] **Step 4: Write the scenes.** At the top of `scenes.rs`:

```rust
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
/// view: fourteen voxels ahead, ten short of the surface the camera faces,
/// and six to the right of the line of sight, so it does not hide what the
/// camera is looking at. With mass, it falls onto whatever is below.
pub(crate) fn with_falling_body(tree: Contree, materials: MaterialTable) -> Scene {
    let (eye, look_at) = start_camera(&tree);
    let forward = (look_at - eye).normalize();
    let right = forward.cross(Vec3::Y).normalize_or_zero();
    let mut body = demo_body(eye + forward * 14.0 + right * 6.0, Quat::IDENTITY);
    body.recompute(&materials);
    Scene { tree, materials, bodies: vec![body], joints: vec![], eye, look_at }
}
```

Then, after the moved `demo_scene`, `palette` and `demo_body`:

```rust
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
        joints.push(Joint::new(&links[i], Some(&links[i - 1]), Linear::Point, Angular::Free, meet, meet, Vec3::Y));
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
```

- [ ] **Step 5: The keys.** In `main.rs`:
  - `.init_resource::<SceneKind>()`
  - `.add_systems(Update, scene_keys.before(build_gpu_scene).before(grab_input).before(brush_input))`
  - `toggle_grab_mode` takes `kind: Res<SceneKind>` and sets `title(&state, *kind)`.
  - `setup` becomes the following; the falling-body code has moved into `with_falling_body` (Step 4):

```rust
fn setup(mut commands: Commands) {
    // 2D camera composites the sprite showing the marched image.
    commands.spawn(Camera2d);

    let scene = match std::env::args().nth(1) {
        Some(path) => match bevox_core::vox::load_scene(std::path::Path::new(&path)) {
            Ok((tree, materials)) => {
                info!(
                    "loaded {path}: extent {}, {} arena nodes",
                    tree.extent(),
                    tree.arena().nodes().len()
                );
                scenes::with_falling_body(tree, materials)
            }
            Err(e) => {
                // A bad path is a typo, not a crash: say so and show the demo.
                error!("could not load {path}: {e}");
                scenes::demo()
            }
        },
        None => scenes::demo(),
    };

    // 3D camera exists only to supply view and projection matrices to the
    // shader; it renders nothing itself.
    commands.spawn((
        Camera3d::default(),
        Camera { order: -1, is_active: false, ..default() },
        Transform::from_translation(scene.eye),
        // Yaw and pitch must agree with the intended direction: the fly camera
        // rewrites the transform's rotation from them every frame.
        FlyCamera::looking_at(scene.eye, scene.look_at),
    ));

    let field = DistanceField::build(&scene.tree);
    commands.insert_resource(VoxelScene {
        tree: scene.tree,
        materials: scene.materials,
        generation: 1,
        field,
        field_dirty: None,
        bodies: scene.bodies,
    });
    commands.insert_resource(Joints(scene.joints));
}
```

```rust
/// The window title: the scene, and the tool in use.
fn title(grab: &GrabState, scene: SceneKind) -> String {
    let mode = if grab.enabled { ", grab mode (G)" } else { "" };
    format!("BEVOX \u{2014} {} scene (1, 2){mode}", scene.name())
}

/// `1` loads the demo scene and `2` the joint scene; pressing either again
/// resets it.
///
/// Loading replaces the world, the bodies and the joints, lets go of any grab,
/// asks for a rebuild, and puts the camera where the scene starts it.
fn scene_keys(
    keys: Res<ButtonInput<KeyCode>>,
    mut kind: ResMut<SceneKind>,
    mut scene: ResMut<VoxelScene>,
    mut joints: ResMut<Joints>,
    mut grab: ResMut<GrabState>,
    mut camera: Query<(&mut Transform, &mut FlyCamera), With<Camera3d>>,
    mut windows: Query<&mut Window>,
) {
    let chosen = if keys.just_pressed(KeyCode::Digit1) {
        SceneKind::Demo
    } else if keys.just_pressed(KeyCode::Digit2) {
        SceneKind::Joints
    } else {
        return;
    };
    let built = chosen.build();
    if let Ok((mut transform, mut fly)) = camera.single_mut() {
        transform.translation = built.eye;
        let facing = FlyCamera::looking_at(built.eye, built.look_at);
        fly.yaw = facing.yaw;
        fly.pitch = facing.pitch;
    }
    let field = DistanceField::build(&built.tree);
    let generation = scene.generation + 1;
    *scene = VoxelScene {
        tree: built.tree,
        materials: built.materials,
        generation,
        field,
        field_dirty: None,
        bodies: built.bodies,
    };
    joints.0 = built.joints;
    grab.held = None;
    *kind = chosen;
    if let Ok(mut window) = windows.single_mut() {
        window.title = title(&grab, chosen);
    }
}
```

- [ ] **Step 6: Run the tests.**

Run: `cargo test --workspace > $SCRATCH/t5b.txt 2>&1; echo $?`
Expected: `0`, with no warnings. If `the_joint_scene_works` fails on a station, fix the station's numbers, not the gate. Record every such change in "What changed during execution".

- [ ] **Step 7: The deliberate breaks.** Each one on its own, reverted.
  1. `scene_keys` keeps the old generation: `number_keys_load_and_reset_scenes` must FAIL.
  2. `CRANK_TORQUE` set to `0.0`: `the_joint_scene_works` must FAIL on the slider.
  3. `SERVO_ANGLE` in the joint changed to `0.0`, leaving the constant the test reads: it must FAIL on the servo.
  4. The block's friction set to `0.0`: it must FAIL on the friction block.

- [ ] **Step 8: Measure.**

Run: `cargo test -p bevox --release a_joint_scene_tick_is_timed -- --ignored --nocapture > $SCRATCH/t5c.txt 2>&1; echo $?`
Record the median in Measurements below. It is a single figure, not a comparison, so no A/B/A is needed.

- [ ] **Step 9: Commit.**

```bash
git add crates/bevox/src
git commit -m "feat(app): 1 and 2 load and reset the demo and the joint scene"
```

---

### Task 6: Record it

- [ ] **Step 1:** Fill in Measurements and "What changed during execution" below.
- [ ] **Step 2:** Update memory `physics-stays-off-master.md`: milestone 7 done, local, not pushed and not yet tested by Flori.
- [ ] **Step 3: Commit.**

```bash
git add docs/superpowers/plans/2026-09-18-bevox-physics-dwyer-joints.md
git commit -m "docs: Dwyer's joints done"
```

## Measurements

- **A tick of the joint scene** (12 bodies, 13 joints, release build): median 0.290, 0.286 and 0.288 ms over three runs of 1000 ticks each, after 200 ticks of warm-up. This is a single figure, not a comparison, so no A/B/A was needed.
- **The workspace:** 327 tests pass and 9 are ignored, with no warnings.

## What changed during execution

1. **`block` pads at `k`'s own scale, not with the identity.**
   - The cause: a body's `k` is about 1e-5. A matrix mixing 1e-5 with 1 loses its small eigenvalues in f32.
   - What it looked like: `padded · padded⁻¹` came out nowhere near the identity (entries up to 14), and a Free + Axis joint between two bodies grew its carried impulse about fivefold per tick until the bodies were stopped by the speed cap.
   - Padding with `trace(k)/3` leaves the answer unchanged and brought the residual to 7e-8. The momentum gate caught it.
2. **`no_joint_gains_energy` is blind to three breaks and catches two.**
   - Blind: the relax pass run with bias, warm starting applied twice, and the bias ×10.
   - Caught: λ ×3, and the effective mass's angular term with its sign flipped.
   - It is kept as a guard against over-correction and a wrong effective mass, and nothing more.
3. **The line-lever break is caught through a blow-up.** Pushing `b` at its own anchor makes the pair diverge, and the momentum gate fails on linear momentum, not on the torque the plan predicted.
4. **The momentum gate was made able to see a misplaced friction lever.**
   - At the plan's friction of 1e5, the lever break cost 5e-5 of the angular momentum, inside the 1e-3 tolerance.
   - The fix: friction 1e7; anchors set apart for Free and Distance; `b` kicked across the line between them.
   - Even then, breaking the lever in the friction solve alone stays blind (4.5e-5). Warm starting reapplies most of the impulse every substep at the correct lever, so the break that fails the gate moves both.
5. **The target-motor knock:**
   - A servo of 1e8 stopped a knock within two substeps, and a spin about the centre is mostly taken out by the hinge.
   - The test now uses a servo of 1e7, still over three times the load, and knocks the bar about its hinge, as the door test does.
6. **The release speed is 4.44, not the target's 6.4.**
   - The target jumps once a tick, and the four substeps close 20% of the gap each, so the last substep moves at `0.2 · 0.8³ · L / h`, with `L = 0.1 / (1 − 0.8⁴)`: exactly 4.44.
   - The gate now asks for more than half the target's speed, and for letting go to leave the speed unchanged.
   - Relaxing the grab leaves 0, which is the break.
7. **The joint scene gate was blind to a dead crank motor.** Gravity alone swings the linkage and moves the slider more than 5 voxels. The gate now also asks that the crank turn more than two full turns in ten seconds; with no motor, it turned −2.96 rad.
8. **Breaks, all reverted.** Each one below fails its gate:
   - Line with one row; Free solved as Locked; `apply` without `b`; Line pushing `b` at `pb`;
   - door friction 0; friction unclamped; motor unclamped; no wrap; the friction lever at `pb` in both warm start and solve;
   - grab Free; grab uncapped (two gates); grab relaxed;
   - no generation bump; crank torque 0; servo target 0; block friction 0.
9. **Unplanned, but needed:** moving the demo scene left five imports in `main.rs` unused, and they were removed.
