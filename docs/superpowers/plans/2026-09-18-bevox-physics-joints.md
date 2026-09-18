# Joints Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking. **Flori prefers inline execution for this project.**

**Goal:** Join bodies to each other or to the world with ball and hinge joints, made by clicking in the app: a door hinged to a wall, a chain hanging from a beam.

**Architecture:**
- A joint is solved inside the TGS substeps alongside the contacts, as velocity constraints with a soft correction for drift, warm started from its own accumulated impulses. This follows Dwyer's constraint formulation in devlog #30.
- A **ball** joint keeps two pivot points together: a 3×3 solve.
- A **hinge** joint adds two angular rows that keep the bodies' axes aligned.
- Jointed bodies do not collide with each other.
- After any edit, each joint follows the piece that holds its pivot voxel, or is removed.

**Tech Stack:** Rust stable 1.96, Bevy 0.19.1, glam 0.32. No new dependencies.

**Spec:** `docs/superpowers/specs/2026-09-15-bevox-rigid-bodies-design.md`, milestone 6.

## Global Constraints

- Native desktop only. No web build, ever.
- `bevox_core` has no Bevy and no GPU dependency, and gains no new dependencies. No new dependencies in any crate.
- **Body transforms are rigid: rotation and translation, no scale.**
- Tests use a seeded `bevox_core::testing::XorShift64`. No test framework may be added.
- This plan changes no shader. **Bodies still cast no shadows**; that is a separate, later item and is not started here.
- **Physics stays off master.** Commit on `feat/rigid-body-physics`; push to its GitHub copy only when Flori asks; never delete that copy.
- Verify with `cargo test --workspace`. Redirect output to a file and check `$?`; never pipe cargo through `tee` or `tail`. A transient LNK1102 or LNK1104 gets one retry.
- Do not run the app. Flori tests it.
- Every correctness gate is proven by a deliberate break.

## Decisions (Flori, 2026-09-18)

- **A joint tool in the app, not just a demo.** `J` toggles joint mode; `H` switches between ball and hinge.
- **Ball and hinge only.**
- **The pivot is the first click; nothing snaps.** The first click, on a body, sets the pivot. For a hinge, the normal of the clicked face is the axis. The second click chooses what to fasten it to, a body or the world, at that same pivot. This is Garry's Mod's convention.
- **Joints are invisible.** They go away with their body or their pivot voxel. There is no separate remove action yet.

## Facts that are not obvious from the code

**Anchors are volume coordinates, as the grab's is.** `world_from_local` maps them to the world whatever `recompute` does to the centre of mass. The first body's anchor is nudged 0.01 inside the clicked face, so it lies in the clicked voxel, which is what "the piece that holds the pivot" is checked against.

**A split keeps volume coordinates.** Every piece `sculpt` splits off is built at the parent's corner, with its extent and orientation, so a voxel has the same coordinates in the piece as it had in the parent. Following a joint across a split is therefore only a matter of finding which body now holds the voxel. The same world point is checked too, so an unrelated body that happens to have a voxel at those coordinates is not mistaken for the piece.

**The world side of a joint is a fixed point.** When the joint is to the world, `anchor_b` is a world point and `axis_b` a world axis, and every term of the world's is zero, exactly as with contacts.

**Warm-start impulses are stored in world axes.** The hinge's two angular rows are built from the current axis each substep, so their basis turns. The accumulated angular impulse is therefore kept as a world vector rather than two numbers in a basis that no longer exists.

**Jointed bodies must not collide with each other.** A hinge at the edge where a door meets its frame would otherwise have the contact pushing the door out while the joint pulls it back.

**The ball constraint's effective mass is a matrix.** Moving the pivot by an impulse `P` changes its velocity by `(1/m − [r]× I⁻¹ [r]×) P`, summed over both bodies, where `[r]×` is the cross-product matrix. The solve inverts that 3×3 matrix. One row per axis solved separately would converge slowly on a chain.

## File Structure

- `crates/bevox_core/src/physics/joint.rs` (**new**): `Joint`, `JointKind`, the ball and hinge solves, and `follow`.
- `crates/bevox_core/src/physics/solver.rs`: joints inside the substeps, and no contacts between jointed pairs. `step` gains `joints: &mut [Joint]`.
- `crates/bevox_core/src/physics/mod.rs`: declare the module.
- `crates/bevox/src/main.rs`: `JointState`, the `J` and `H` keys, the two clicks, and `follow` after every tick.

---

### Task 1: Ball joints

**Files:**
- Create: `crates/bevox_core/src/physics/joint.rs`
- Modify: `crates/bevox_core/src/physics/mod.rs`, `crates/bevox_core/src/physics/solver.rs`, `crates/bevox/src/main.rs` (the new `step` argument)

**Interfaces:**
- Produces:
  - `pub enum JointKind { Ball, Hinge }`
  - `pub struct Joint { pub kind, pub a: BodyId, pub b: Option<BodyId>, pub anchor_a: Vec3, pub anchor_b: Vec3, pub axis_a: Vec3, pub axis_b: Vec3, pub linear: Vec3, pub angular: Vec3 }`
  - `Joint::new(kind, a: &Body, b: Option<&Body>, pivot: Vec3, axis: Vec3) -> Joint`
  - `step(bodies, tree, field, materials, gravity, dt, grab, joints: &mut [Joint]) -> bool`

- [ ] **Step 1: Write the failing tests**, in `solver.rs`'s tests:

```rust
    /// A body pinned to the world by a ball joint at its corner hangs straight
    /// below the pivot, and the pivot does not drift.
    #[test]
    fn a_body_pinned_by_its_corner_hangs_below_the_pivot() {
        let materials = materials();
        let world = Contree::empty(3);
        let field = DistanceField::build(&world);
        let mut bodies = vec![placed(cube(4, 4), Vec3::new(30.0, 30.0, 30.0), Quat::IDENTITY)];
        let pivot = bodies[0].world_from_local().transform_point3(Vec3::splat(0.01));
        let mut joints = vec![Joint::new(JointKind::Ball, &bodies[0], None, pivot, Vec3::Y)];
        for _ in 0..3000 {
            step(&mut bodies, &world, &field, &materials, GRAVITY, DT, None, &mut joints);
        }
        let held = bodies[0].world_from_local().transform_point3(joints[0].anchor_a);
        assert!((held - pivot).length() < 0.05, "the pivot drifted by {}", (held - pivot).length());
        let off = bodies[0].position - pivot;
        assert!(Vec3::new(off.x, 0.0, off.z).length() < 0.3, "hangs askew: {off:?}");
        assert!(off.y < -2.0, "not hanging: {off:?}");
    }

    /// A chain of three hangs from the world and stays together: every joint
    /// holds its two points within a small distance, and nothing is still
    /// moving thousands of ticks later.
    #[test]
    fn a_chain_of_three_hangs_together() {
        let materials = materials();
        let world = Contree::empty(3);
        let field = DistanceField::build(&world);
        let mut bodies: Vec<Body> = (0..3)
            .map(|i| placed(cube(4, 4), Vec3::new(30.0 + i as f32 * 4.0, 40.0, 30.0), Quat::IDENTITY))
            .collect();
        // The first hangs from the world at its left edge; each next one is
        // joined to the one before where they meet.
        let top = Vec3::new(28.01, 40.0, 30.0);
        let mut joints = vec![Joint::new(JointKind::Ball, &bodies[0], None, top, Vec3::Y)];
        for i in 1..3 {
            let meet = Vec3::new(28.01 + i as f32 * 4.0, 40.0, 30.0);
            joints.push(Joint::new(JointKind::Ball, &bodies[i], Some(&bodies[i - 1]), meet, Vec3::Y));
        }
        for _ in 0..5000 {
            step(&mut bodies, &world, &field, &materials, GRAVITY, DT, None, &mut joints);
        }
        for (i, j) in joints.iter().enumerate() {
            let a = bodies.iter().find(|b| b.id == j.a).unwrap();
            let pa = a.world_from_local().transform_point3(j.anchor_a);
            let pb = match j.b {
                None => j.anchor_b,
                Some(id) => {
                    let b = bodies.iter().find(|b| b.id == id).unwrap();
                    b.world_from_local().transform_point3(j.anchor_b)
                }
            };
            assert!((pa - pb).length() < 0.1, "joint {i} opened by {}", (pa - pb).length());
        }
        for b in &bodies {
            assert!(b.velocity.length() < 0.1, "the chain is still moving: {:?}", b.velocity);
        }
    }

    /// A joint between two free bodies moves momentum between them but never
    /// makes or destroys it.
    #[test]
    fn a_joint_conserves_momentum() {
        let materials = materials();
        let world = Contree::empty(3);
        let field = DistanceField::build(&world);
        let mut bodies = vec![
            placed(cube(4, 4), Vec3::new(30.0, 30.0, 30.0), Quat::IDENTITY),
            placed(cube_of(4, 4, MaterialId(2)), Vec3::new(34.0, 30.0, 30.0), Quat::IDENTITY),
        ];
        bodies[0].velocity = Vec3::new(0.0, 6.0, -3.0);
        let before: Vec3 = bodies.iter().map(|b| b.velocity * b.mass.mass).sum();
        let mut joints =
            vec![Joint::new(JointKind::Ball, &bodies[1], Some(&bodies[0]), Vec3::new(32.0, 30.0, 30.0), Vec3::Y)];
        for _ in 0..200 {
            step(&mut bodies, &world, &field, &materials, Vec3::ZERO, DT, None, &mut joints);
        }
        let after: Vec3 = bodies.iter().map(|b| b.velocity * b.mass.mass).sum();
        assert!((after - before).length() < 1e-3 * before.length(), "momentum {after:?}, was {before:?}");
        assert!(bodies[1].velocity.length() > 0.5, "the joint moved nothing, so this proves nothing");
    }

    /// Two jointed bodies that overlap do not push each other apart: jointed
    /// pairs have no contacts.
    #[test]
    fn jointed_bodies_do_not_collide() {
        let materials = materials();
        let world = Contree::empty(3);
        let field = DistanceField::build(&world);
        let mut bodies = vec![
            placed(cube(4, 4), Vec3::new(30.0, 30.0, 30.0), Quat::IDENTITY),
            placed(cube(4, 4), Vec3::new(32.0, 30.0, 30.0), Quat::IDENTITY),
        ];
        let mut joints =
            vec![Joint::new(JointKind::Ball, &bodies[1], Some(&bodies[0]), Vec3::new(31.0, 30.0, 30.0), Vec3::Y)];
        for _ in 0..30 {
            step(&mut bodies, &world, &field, &materials, Vec3::ZERO, DT, None, &mut joints);
        }
        for b in &bodies {
            assert!(b.velocity.length() < 1e-3, "an overlap pushed a jointed body: {:?}", b.velocity);
        }
    }
```

Every existing `step` call passes `&mut []` as the new last argument.

- [ ] **Step 2: Run to verify they fail**, then **Step 3: implement.**

`joint.rs`:

```rust
//! Joints: two bodies, or a body and the world, held together at a pivot.
//!
//! After Dwyer's devlog #30: each joint is a constraint solved as an impulse
//! alongside the contacts, the same expression for every kind. A ball joint
//! keeps two points together; a hinge also keeps two axes aligned.

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
    /// Accumulated impulses for warm starting, in world axes.
    pub linear: Vec3,
    pub angular: Vec3,
}

impl Joint {
    /// Joins `a` to `b`, or to the world, at the world point `pivot`. Nothing
    /// moves: `b` is fastened wherever it is. `axis` is the hinge axis, in world
    /// axes; a ball joint ignores it.
    pub fn new(kind: JointKind, a: &Body, b: Option<&Body>, pivot: Vec3, axis: Vec3) -> Self {
        let axis = axis.normalize();
        let (anchor_b, axis_b) = match b {
            Some(b) => (b.local_from_world().transform_point3(pivot), b.orientation.inverse() * axis),
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
}
```

The solve lives in `joint.rs` too, taking `(a: &mut Body, b: Option<&mut Body>, joint: &mut Joint, inv_h, use_bias)`:

```rust
/// The cross-product matrix: `skew(r) * x == r.cross(x)`.
fn skew(r: Vec3) -> Mat3 {
    Mat3::from_cols(Vec3::new(0.0, r.z, -r.y), Vec3::new(-r.z, 0.0, r.x), Vec3::new(r.y, -r.x, 0.0))
}

/// How the pivot's velocity answers an impulse there: `1/m - [r] I^-1 [r]`.
fn point_mass(body: &Body, r: Vec3) -> Mat3 {
    let s = skew(r);
    Mat3::from_diagonal(Vec3::splat(body.mass.inverse_mass())) - s * world_inverse_inertia(body) * s
}
```

The ball solve:
- `ra = pa − a.position`, and `rb` likewise for a body.
- `cdot = point velocity of a at ra − point velocity of b at rb`.
- `k = point_mass(a, ra) + point_mass(b, rb)`.
- `bias = if use_bias { BIAS · (pa − pb) · inv_h } else { 0 }`.
- `λ = −k.inverse() · (cdot + bias)`; `joint.linear += λ`.
- Apply `+λ` at `ra` to `a` and `−λ` at `rb` to `b`, as the contact `push` does.

`world_inverse_inertia` and `point_velocity` move from `solver.rs` into `Body` as methods (`Body::world_inverse_inertia` already exists privately; make it `pub(crate)`), so both files share them.

In `solver.rs`:
- `step` takes `joints: &mut [Joint]` last and builds an id-to-index map.
- `collect_contacts` skips any pair joined by a joint.
- In each substep, after warm-starting the contacts, warm start each joint: apply `joint.linear` at the pivots, and `joint.angular` as angular momentum, `+` to `a` and `−` to `b`. Solve the joints with bias before the contacts. After integrating, solve them again without bias, in the relax pass.
- A joint whose bodies are not both present, or have no mass, is skipped.

- [ ] **Step 4: Run to verify they pass.** `cargo test --workspace`, `0`, no warnings. If the chain opens or keeps moving, try, one at a time and recording each: a second joint iteration per substep, then more substeps for joints only. Do not loosen the gate.

- [ ] **Step 5: Break checks.**
1. Solve the ball as three independent rows, one axis at a time, ignoring the off-diagonal terms: the chain gate should FAIL, or the pinned gate drift. If neither does, record that the matrix solve is not needed, keep it anyway (it is the correct formulation), and say so.
2. Apply `+λ` to both bodies: `a_joint_conserves_momentum` must FAIL.
3. Let jointed pairs collide: `jointed_bodies_do_not_collide` must FAIL.
4. Drop the bias: the pinned gate must FAIL on drift.

Restore each.

- [ ] **Step 6: Commit** — `feat(core): ball joints, solved with the contacts`.

---

### Task 2: Hinge joints

**Files:**
- Modify: `crates/bevox_core/src/physics/joint.rs`

- [ ] **Step 1: Write the failing test**

```rust
    /// A door hinged to the world about a vertical axis swings only about that
    /// axis: pushed, it turns in the horizontal plane and its hinge edge stays
    /// put, and gravity does not tip it over.
    #[test]
    fn a_hinged_door_turns_only_about_its_axis() {
        let materials = materials();
        let world = Contree::empty(3);
        let field = DistanceField::build(&world);
        // A door 8 wide, 12 tall, 1 thick.
        let door: Vec<_> = (0..8)
            .flat_map(|x| (0..12).map(move |y| (UVec3::new(x, y, 0), MaterialId(1))))
            .collect();
        let mut bodies =
            vec![placed(Contree::from_voxels(16, &door), Vec3::new(34.0, 36.0, 30.5), Quat::IDENTITY)];
        let hinge = Vec3::new(30.01, 36.0, 30.5);
        let mut joints = vec![Joint::new(JointKind::Hinge, &bodies[0], None, hinge, Vec3::Y)];
        bodies[0].velocity = Vec3::new(0.0, 0.0, 8.0);
        for _ in 0..300 {
            step(&mut bodies, &world, &field, &materials, GRAVITY, DT, None, &mut joints);
        }
        let door = &bodies[0];
        let spin = door.angular_velocity();
        let tilt = (door.orientation * Vec3::Y).dot(Vec3::Y);
        assert!(tilt > 0.999, "the door tipped: its up is {:?}", door.orientation * Vec3::Y);
        assert!(
            Vec3::new(spin.x, 0.0, spin.z).length() < 0.02,
            "the door turns about more than its axis: {spin:?}"
        );
        assert!(door.orientation.angle_between(Quat::IDENTITY) > 0.2, "the push never swung it");
        let held = door.world_from_local().transform_point3(joints[0].anchor_a);
        assert!((held - hinge).length() < 0.05, "the hinge moved by {}", (held - hinge).length());
    }
```

- [ ] **Step 2: Run to verify it fails**, then **Step 3: implement.**

After the ball rows, for `JointKind::Hinge`:
- `wa = a.orientation * joint.axis_a`, and `wb` from `b`, or the world axis.
- `(t1, t2) = wa.any_orthonormal_pair()`.
- `cdot = (ωa − ωb)` projected on `t1` and `t2`, with `ωb = 0` for the world.
- `error = wa.cross(wb)`, projected likewise. The bias makes the relative spin turn `wa` toward `wb`: `bias_i = −BIAS · error_i · inv_h` when `use_bias`.
- `k` is the 2×2 `t_i · (Ia⁻¹ + Ib⁻¹) · t_j`.
- `λ = −k⁻¹ (cdot + bias)`.
- `impulse = t1 λ1 + t2 λ2`; `joint.angular += impulse`; `a.L += impulse`; `b.L −= impulse`.

- [ ] **Step 4: Run to verify it passes.** Then **Step 5: break checks.**
1. Drop the angular rows: the door must tip, and FAIL.
2. Flip the bias sign: the door must tip or twist, and FAIL.

Restore each.

- [ ] **Step 6: Commit** — `feat(core): hinge joints`.

---

### Task 3: Joints follow their pivots

**Files:**
- Modify: `crates/bevox_core/src/physics/joint.rs`

**Interfaces:**
- Produces: `pub fn follow(joints: &mut Vec<Joint>, bodies: &[Body])`

- [ ] **Step 1: Write the failing tests**

```rust
    /// Cut a bar in two, and a joint pinned to its far end follows the piece
    /// that end is on.
    #[test]
    fn a_joint_follows_its_pivot_across_a_split() {
        let materials = materials();
        let bar: Vec<_> = (0..12).map(|x| (UVec3::new(x, 0, 0), MaterialId(1))).collect();
        let mut bodies = vec![placed(Contree::from_voxels(16, &bar), Vec3::new(30.0, 30.0, 30.0), Quat::IDENTITY)];
        let end = bodies[0].world_from_local().transform_point3(Vec3::new(11.5, 0.5, 0.5));
        let mut joints = vec![Joint::new(JointKind::Ball, &bodies[0], None, end, Vec3::Y)];
        // Erase near the far end, so the joint's end is the smaller piece.
        let cut = bodies[0].world_from_local().transform_point3(Vec3::new(8.5, 0.5, 0.5));
        crate::physics::sculpt::sculpt(&mut bodies, 0, cut, 1.2, MaterialId::EMPTY, &materials, 16);
        assert_eq!(bodies.len(), 2);
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
        let mut joints = vec![Joint::new(JointKind::Ball, &bodies[0], None, corner, Vec3::Y)];
        crate::physics::sculpt::sculpt(&mut bodies, 0, corner, 0.9, MaterialId::EMPTY, &materials, 16);
        follow(&mut joints, &bodies);
        assert!(joints.is_empty(), "a joint outlived its pivot voxel");

        let mut joints = vec![Joint::new(JointKind::Ball, &bodies[0], None, corner, Vec3::Y)];
        bodies.clear();
        follow(&mut joints, &bodies);
        assert!(joints.is_empty(), "a joint outlived its body");
    }
```

- [ ] **Step 2: Run to verify they fail**, then **Step 3: implement `follow`.** For each joint:
1. **Side `a`:** the pivot voxel is `anchor_a.floor()`.
   - If the body with id `a` holds it, keep the joint.
   - Otherwise find the body that holds that voxel and maps `anchor_a` to the same world point, within 1e-3, as the old body did, and give the joint its id.
   - If there is none, the joint goes.
2. **Side `b`,** when it is a body: if its id is gone, the joint goes. The second body is fastened wherever it was clicked, which need not be inside it, so it stays with whichever piece keeps its identity.

Record the old body's world point *before* it is lost. Pieces keep the parent's frame at the moment of the split, so compare against the pieces' own `world_from_local`. That transform is the same for the old and new holder of the voxel, which is what makes this check meaningful.

- [ ] **Step 4: Run to verify they pass.** Then **Step 5: break checks.**
1. Never retarget, keeping the id: the split test must FAIL.
2. Never remove: the pivot-removal test must FAIL.

Restore each.

- [ ] **Step 6: Commit** — `feat(core): joints follow their pivots across edits`.

---

### Task 4: The joint tool

**Files:**
- Modify: `crates/bevox/src/main.rs`

- [ ] **Step 1: Write the failing tests**

```rust
    /// Two clicks make a joint: the first on a body sets the pivot, the second
    /// on the world fastens it there.
    #[test]
    fn two_clicks_make_a_joint() {
        let materials = demo_scene().1;
        let mut body = demo_body(Vec3::new(20.0, 40.0, 20.0), Quat::IDENTITY);
        assert!(body.recompute(&materials));
        let bodies = vec![body];
        let mut state = JointState { enabled: true, kind: JointKind::Hinge, pending: None };
        let mut joints = Joints::default();
        let on_body = Hit { target: Target::Body(0), position: Vec3::new(20.0, 43.0, 20.0), normal: Vec3::Y };
        joint_click(&mut state, &mut joints, &bodies, on_body);
        assert!(joints.0.is_empty() && state.pending.is_some(), "the first click made a joint");
        let on_world = Hit { target: Target::World, position: Vec3::new(20.0, 6.0, 20.0), normal: Vec3::Y };
        joint_click(&mut state, &mut joints, &bodies, on_world);
        assert_eq!(joints.0.len(), 1);
        assert_eq!(joints.0[0].kind, JointKind::Hinge);
        assert!(joints.0[0].b.is_none(), "not fastened to the world");
        assert!(state.pending.is_none());
    }

    /// `J` and `G` are exclusive: one tool at a time.
    #[test]
    fn joint_and_grab_modes_exclude_each_other() {
        let mut world = World::new();
        world.init_resource::<GrabState>();
        world.init_resource::<JointState>();
        world.resource_mut::<GrabState>().enabled = true;
        let mut keys = ButtonInput::<KeyCode>::default();
        keys.press(KeyCode::KeyJ);
        world.insert_resource(keys);
        let toggle = world.register_system(toggle_joint_mode);
        world.run_system(toggle).unwrap();
        assert!(world.resource::<JointState>().enabled);
        assert!(!world.resource::<GrabState>().enabled, "grab mode stayed on");
    }
```

- [ ] **Step 2: Run to verify they fail**, then **Step 3: implement.**
- **Resources:**
  - `#[derive(Resource, Default)] struct Joints(Vec<Joint>)`;
  - `#[derive(Resource)] struct JointState { enabled: bool, kind: JointKind, pending: Option<(BodyId, Vec3, Vec3)> }`, with `Default` giving `Ball`.
- **`toggle_joint_mode`:**
  - `J` flips `enabled`, clears `pending`, turns grab mode off (and lets go), and sets the title;
  - `H`, in joint mode, switches the kind;
  - the title reads `BEVOX — joint mode: ball (J, H)` or `hinge`.
- **`toggle_grab_mode`:** also turns joint mode off.
- **`joint_click(state, joints, bodies, hit)`** is a pure function:
  - with nothing pending, a body hit sets `pending = (id, hit.position − hit.normal · 0.01, hit.normal)`;
  - with something pending, a body hit that is not the same body, or a world hit, makes `Joint::new(kind, a, b, pivot, axis)` and clears `pending`;
  - a click on the same body again cancels the pending joint.
- **`joint_input`:** calls `pick` on a left press in joint mode, then `joint_click`.
- **`brush_input`:** ignores the left mouse in joint mode, as in grab mode.
- **`physics_system`:** takes `ResMut<Joints>`, passes `&mut joints.0` to `step`, then calls `follow(&mut joints.0, &scene.bodies)`.

- [ ] **Step 4: Run to verify they pass.** Then **Step 5: break check.** Have `toggle_joint_mode` leave grab mode on: the exclusion test must FAIL. Restore.

- [ ] **Step 6: Commit** — `feat(app): a joint tool: J, then click a body and what to fasten it to`.

- [ ] **Step 7: Update the spec's milestone table:** 6 is done. Add a "later" line for body shadows, which Flori flagged. Then commit.

- [ ] **Step 8: Hand over.** Tell Flori:
  - `J` for joint mode, and `H` for hinge;
  - click a body, then the world, to hang it or hinge it; or click a body, then another body, to chain them;
  - the grab (`G`) is the way to test joints by pulling on them.

  Do not merge; push only if asked.

---

## What changed during execution

- **Joints are undamped, so the "settles" gates were wrong as planned.** A body
  pinned by its corner is a frictionless pendulum and never stops, and neither
  does a knocked chain. The gates now check what must hold instead: a joint
  stays shut the whole time, and total energy never grows, which is how an
  unstable solver shows itself. A body hung from its centre top, already in
  equilibrium, must stay put.
- **Solving the ball constraint one axis at a time passes every gate too.**
  The substeps correct what one pass misses. The full 3x3 solve is kept, as the
  correct formulation at the same cost, and the source says the gates cannot
  tell the two apart.
- **Two gates were blind and were rebuilt.**
  - Two deeply overlapping jointed cubes never collided anyway, because each
    one's corners sat in the other's interior, where no contact is made. The
    gate uses a shallow overlap.
  - Nothing checked that `follow` refuses an unrelated body that happens to
    hold a voxel at the same coordinates. A second cube elsewhere now does.

## Milestone check

A body pinned by its corner hangs below the pivot. A three-link chain hangs together and comes to rest. A hinged door turns only about its axis and does not tip. Joints conserve momentum. Jointed bodies do not fight each other. A joint follows its pivot across a split and goes with its pivot.

## What this plan deliberately does not do

- **No weld, rope, prismatic or cone joints.** Ball and hinge only, by Flori's choice.
- **No drawing of joints.**
- **No joint limits, motors or breaking.**
- **No body shadows.** That is a separate item for later.
