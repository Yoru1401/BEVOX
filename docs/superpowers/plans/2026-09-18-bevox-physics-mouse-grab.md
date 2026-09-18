# Mouse Grab Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking. **Flori prefers inline execution for this project.**

**Goal:** Pick up a body with the mouse. A damped spring pulls the exact point you clicked toward the cursor, and the body keeps simulating: it swings, collides, and flies off with its own momentum when released.

**Architecture:**
- **The spring.** A grab is a body id, the grabbed point in the body's volume coordinates, and a target in the world. Every solver substep, a damped spring accelerates the grabbed point toward the target and applies that as an impulse at the point, so the body turns as well as moves.
- **Tuning.** Stiffness and damping are given as a frequency and a damping ratio and scaled by mass, so every body feels the same.
- **The app.** `G` toggles a grab mode in which the left mouse grabs instead of painting.

**Tech Stack:** Rust stable 1.96, Bevy 0.19.1, glam 0.32. No new dependencies.

**Spec:** `docs/superpowers/specs/2026-09-15-bevox-rigid-bodies-design.md`, milestone 5, as revised in Task 1 below.

## Global Constraints

- Native desktop only. No web build, ever.
- `bevox_core` has no Bevy and no GPU dependency, and gains no new dependencies. No new dependencies in any crate.
- **Body transforms are rigid: rotation and translation, no scale.**
- Tests use a seeded `bevox_core::testing::XorShift64`. No test framework may be added.
- A scene with zero bodies must render bit-identically. This plan changes no shader.
- **Physics stays off master.** Commit on `feat/rigid-body-physics`; push to its GitHub copy only when Flori asks; never delete that copy.
- Verify with `cargo test --workspace`. Redirect output to a file and check `$?`; never pipe cargo through `tee` or `tail`. A transient LNK1102 or LNK1104 gets one retry.
- Do not run the app. It opens a window a human must close; Flori tests it.
- Every correctness gate is proven by a deliberate break.

## Decisions (Flori, 2026-09-18)

- **The grab is a spring, not a joint.** It works like the oscillator in Joe Binns' *Get Me Out*: a damped spring toward a target in front of the camera, with the body still fully simulated.
- **It pulls at the clicked point,** not the centre of mass. That is Flori's choice, and it differs from *Get Me Out*. A cube held by its corner hangs from that corner.
- **Input:** `G` toggles grab mode. In grab mode, holding left mouse grabs and releasing lets go. The wheel moves the held body nearer or farther. Right-click erase and `F` drop are unchanged.
- **Joints move to milestone 6.**

## Facts that are not obvious from the code

**Stiffness is a frequency, so mass cancels.** The spring's acceleration is `ω² (target − point) − 2ζω (point velocity)`, and the impulse is that acceleration times the body's mass times the substep. A heavy body and a light one are held the same way, which is what an oscillator-style grab wants.

**Stability comes from the substeps.** Dwyer's explicit spring was unstable. This one runs inside the solver's four substeps, so `ω·h` stays small. At 4 Hz that is `2π·4 / 256 ≈ 0.1`, and still under 0.3 with the extra stiffness a corner grab adds through rotation. Explicit integration is stable well below 2.

**A far target must not yank.** The spring's acceleration is capped, so dragging the cursor across the sky accelerates the body hard but not instantly to the speed cap.

**A grab on a point does not damp rotation about that point.** With only the spring, a body hanging from its corner swings like a frictionless pendulum and never stops. Physics-gun grabs damp the held body's spin for this reason. This one does too, with `GRAB_SPIN_DAMPING`, and only while it is held.

**The grabbed point is stored in volume coordinates.** `world_from_local` maps them to the world whatever `recompute` does to the centre of mass, so an erase on a held body does not move the handle. Painting that grows the volume shifts those coordinates; a held body being painted on is an edge case this plan accepts.

**Releasing is just not grabbing.** The body has the velocity the spring gave it, so a flick throws it.

**Gravity still acts on a held body,** so it hangs `g / ω²` below the target: about 0.16 voxel at 4 Hz.

## File Structure

- `crates/bevox_core/src/physics/mod.rs`: the grab constants.
- `crates/bevox_core/src/physics/solver.rs`: `Grab`, and the spring inside the substeps. `step` gains `grab: Option<&Grab>`.
- `crates/bevox_render/src/pick.rs`: `cursor_ray`, made public for the grab's target.
- `crates/bevox/src/main.rs`: `GrabState`, the `G` toggle, `grab_input`, and the mode in `brush_input` and `physics_system`.
- `docs/superpowers/specs/2026-09-15-bevox-rigid-bodies-design.md`: milestone 5 becomes the spring grab.

---

### Task 1: The grab spring

**Files:**
- Modify: `crates/bevox_core/src/physics/mod.rs`, `crates/bevox_core/src/physics/solver.rs`
- Modify: `crates/bevox/src/main.rs` (the new `step` argument)
- Modify: the spec

**Interfaces:**
- Produces:
  - `pub struct Grab { pub body: BodyId, pub anchor: Vec3, pub target: Vec3 }`
  - `Grab::new(body: &Body, at: Vec3) -> Grab`
  - `step(bodies, tree, field, materials, gravity, dt, grab: Option<&Grab>) -> bool`
  - the constants `GRAB_FREQUENCY`, `GRAB_DAMPING`, `GRAB_SPIN_DAMPING`, `GRAB_MAX_ACCEL`

- [ ] **Step 1: Write the failing tests**

In `solver.rs`'s tests:

```rust
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
            step(&mut bodies, &world, &field, &materials, GRAVITY, DT, Some(&grab));
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
            step(&mut bodies, &world, &field, &materials, GRAVITY, DT, Some(&grab));
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
        step(&mut bodies, &world, &field, &materials, Vec3::ZERO, DT, Some(&grab));
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
        step(&mut bodies, &world, &field, &materials, Vec3::ZERO, DT, Some(&grab));
        assert_eq!(bodies[0].velocity, Vec3::ZERO);
    }
```

Every existing `step(..)` call passes `None` as the new last argument, and so does `run`.

- [ ] **Step 2: Run to verify they fail**, then **Step 3: implement.**

In `physics/mod.rs`:

```rust
/// How fast a held body springs toward the cursor, in hertz. The spring is a
/// frequency rather than a stiffness so that every body, heavy or light, is
/// held the same way.
pub const GRAB_FREQUENCY: f32 = 4.0;

/// The grab spring's damping ratio. Below 1 it overshoots a little, which is
/// the swing that makes a held body feel held rather than attached.
pub const GRAB_DAMPING: f32 = 0.7;

/// How fast a held body's spin dies away, per second. The spring pins one point
/// and does nothing about rotation around it, so without this a body held by its
/// corner would swing forever.
pub const GRAB_SPIN_DAMPING: f32 = 4.0;

/// The most acceleration the grab may apply, in voxels per second squared, so
/// a target far across the world pulls hard rather than instantly.
pub const GRAB_MAX_ACCEL: f32 = 600.0;
```

In `solver.rs`:

```rust
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
```

`step` takes `grab: Option<&Grab>` last. Inside the substep loop, after gravity and before the speed cap, it finds the held body by id (skipping a body with no mass) and calls:

```rust
/// Pulls the grabbed point toward the target with a damped spring, applied at
/// the point, so the body turns as well as moves; and damps the body's spin,
/// which the spring alone does nothing about.
fn pull(body: &mut Body, grab: &Grab, h: f32) {
    let w = std::f32::consts::TAU * GRAB_FREQUENCY;
    let point = body.world_from_local().transform_point3(grab.anchor);
    let r = point - body.position;
    let accel = (w * w * (grab.target - point) - 2.0 * GRAB_DAMPING * w * point_velocity(body, r))
        .clamp_length_max(GRAB_MAX_ACCEL);
    let impulse = accel * body.mass.mass * h;
    body.velocity += impulse * body.mass.inverse_mass();
    body.angular_momentum += r.cross(impulse);
    body.angular_momentum *= (1.0 - GRAB_SPIN_DAMPING * h).max(0.0);
}
```

In `main.rs`, `physics_system` passes `None` for now.

Update the spec:
- The milestone table: 5 becomes "Mouse grab: a damped spring from the cursor to the clicked point", and 6 becomes "Joints".
- The "Joints and the mouse grab" section: the grab is a spring by Flori's choice, following *Get Me Out*'s oscillator but pulling at the grabbed point. Why it is stable here: the substeps and the acceleration cap. And the spin damping.
- Provenance: move "mouse grab as a joint" from what this design builds to what Dwyer does, and list the spring grab as Flori's design.

- [ ] **Step 4: Run to verify they pass.** `cargo test --workspace`, `0`, no warnings. If the corner hang is still swinging, raise `GRAB_SPIN_DAMPING` and record the value; do not loosen the gate.

- [ ] **Step 5: Break checks.**
1. No damping term (`GRAB_DAMPING` term removed): `a_grabbed_body_settles_at_the_target` must FAIL.
2. No spin damping: `a_body_held_by_its_corner_hangs_below_it` must FAIL.
3. No acceleration cap: `a_far_target_cannot_yank_a_body` must FAIL.
4. Apply the impulse at the centre of mass (drop the `r.cross` line): the corner test must FAIL, because nothing then turns the body to hang.

Restore each.

- [ ] **Step 6: Commit** — `feat(core): a damped spring grab at the clicked point`.

---

### Task 2: Grab mode in the app

**Files:**
- Modify: `crates/bevox_render/src/pick.rs` (`cursor_ray` public)
- Modify: `crates/bevox/src/main.rs`

**Interfaces:**
- Consumes: `Grab`, `step(.., grab)`, `pick`, `Target`

- [ ] **Step 1: Write the failing tests**

In `main.rs`'s tests:

```rust
    /// `G` switches grab mode on and off, and switching it off lets go.
    #[test]
    fn g_toggles_grab_mode() {
        let mut world = World::new();
        world.init_resource::<GrabState>();
        let mut keys = ButtonInput::<KeyCode>::default();
        keys.press(KeyCode::KeyG);
        world.insert_resource(keys);
        let toggle = world.register_system(toggle_grab_mode);
        world.run_system(toggle).unwrap();
        assert!(world.resource::<GrabState>().enabled);

        let mut keys = ButtonInput::<KeyCode>::default();
        keys.press(KeyCode::KeyG);
        world.insert_resource(keys);
        world.resource_mut::<GrabState>().held = Some(Grab {
            body: bevox_core::body::BodyId(7),
            anchor: Vec3::ZERO,
            target: Vec3::ZERO,
        });
        world.run_system(toggle).unwrap();
        let state = world.resource::<GrabState>();
        assert!(!state.enabled);
        assert!(state.held.is_none(), "leaving grab mode did not let go");
    }

    /// A held body is pulled by the physics tick.
    #[test]
    fn the_physics_system_pulls_a_held_body() {
        let mut world = World::new();
        let mut time = Time::<()>::default();
        time.advance_by(std::time::Duration::from_secs_f64(1.0 / 64.0));
        world.insert_resource(time);
        let (tree, materials) = demo_scene();
        let field = DistanceField::build(&tree);
        let mut body = demo_body(Vec3::new(20.0, 40.0, 20.0), Quat::IDENTITY);
        assert!(body.recompute(&materials));
        let mut grab = Grab::new(&body, body.position);
        grab.target = body.position + Vec3::new(10.0, 0.0, 0.0);
        world.insert_resource(GrabState { enabled: true, held: Some(grab), distance: 10.0 });
        world.insert_resource(VoxelScene {
            tree,
            materials,
            generation: 1,
            field,
            field_dirty: None,
            bodies: vec![body],
        });
        let physics = world.register_system(physics_system);
        world.run_system(physics).unwrap();
        assert!(world.resource::<VoxelScene>().bodies[0].velocity.x > 0.5, "the grab did not pull");
    }
```

- [ ] **Step 2: Run to verify they fail**, then **Step 3: implement.**

In `pick.rs`, rename `ray` to `pub fn cursor_ray` and update its two callers.

In `main.rs`:

```rust
/// The mouse grab: whether left mouse grabs instead of painting, what it holds,
/// and how far in front of the camera the held point is kept.
#[derive(Resource, Default)]
struct GrabState {
    enabled: bool,
    held: Option<Grab>,
    distance: f32,
}
```

Then:
- `.init_resource::<GrabState>()`.
- **`toggle_grab_mode`** (Update, before `brush_input`): on `KeyG` just pressed, flip `enabled`, clear `held`, and set the window title to `"BEVOX — grab mode (G)"` or back to `"BEVOX"` if a window exists.
- **`grab_input`** (Update, after `fly_camera_system`), when `enabled`:
  - left pressed this frame: `pick` against the bodies; on a body, `held = Some(Grab::new(body, hit.position))` and `distance = (hit.position - eye).length()`;
  - left held: `target = eye + cursor_ray(..) * distance`, and the wheel changes `distance` by 1 per notch, clamped to 2..=200;
  - left released: `held = None`;
  - if the held body's id is no longer in the scene: `held = None`.
- **`brush_input`**, when grab mode is on: ignores left mouse, and ignores the wheel while something is held. Right-click erase is unchanged.
- **`physics_system`** takes `Res<GrabState>` and passes `state.held.as_ref()` to `step`.

- [ ] **Step 4: Run to verify they pass.** `cargo test --workspace`, `0`, no warnings.

- [ ] **Step 5: Break checks.**
1. `toggle_grab_mode` keeps `held` when switching off: the toggle test must FAIL.
2. `physics_system` passes `None`: the pull test must FAIL.

Restore each.

- [ ] **Step 6: Commit** — `feat(app): G toggles a grab mode, and left mouse picks bodies up`.

- [ ] **Step 7: Hand over.** Tell Flori what to try:
  - `G` for grab mode (the window title says so);
  - hold left mouse on a body to pick it up, the wheel to move it nearer or farther;
  - flick and release to throw it;
  - grab a cube by its corner and it hangs from it.

  Do not merge; push only if asked.

---

## Milestone check

A grabbed body springs to the cursor and settles; one held by its corner hangs straight down from it and stops swinging; a far target pulls hard but not instantly; a released body keeps its momentum. `G` switches grab mode, and nothing else about editing changes.

## What this plan deliberately does not do

- **No joints.** Milestone 6.
- **No rotating a held body with the mouse.**
- **No grabbing the world.** Only bodies can be picked up.
