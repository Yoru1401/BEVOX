---
type: Implementation Plan
title: 'Body Editing'
description: 'The brush edits bodies as it edits the world.'
tags: [physics, rigid-bodies, editing]
generated: { by: claude-opus-5/claude-code, at: 2026-09-18T00:00:00Z }
---

# Body Editing Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking. **Flori prefers inline execution for this project.**

**Goal:** The brush edits bodies as it edits the world. Painting onto a body grows it, erasing shrinks it, and erasing a body in two leaves two bodies that carry on moving as the halves were.

**Architecture:**
- **Picking** gains bodies: the nearest hit among the world and every body decides what the stroke edits.
- **Editing a body** happens in its own frame. The stroke's centre is carried into the body's voxel grid; painting that would reach past the volume first grows the volume, without moving any voxel.
- **After an erase,** the body's voxels are labelled into connected pieces. Every piece past the first becomes a new body, with the velocity that point of the parent had, so momentum is conserved.

**Tech Stack:** Rust stable 1.96, Bevy 0.19.1, glam 0.32. No new dependencies.

**Spec:** `docs/superpowers/specs/2026-09-15-bevox-rigid-bodies-design.md`, milestone 4's second half.

## Global Constraints

- Native desktop only. No web build, ever.
- `bevox_core` has no Bevy and no GPU dependency, and gains no new dependencies. No new dependencies in any crate.
- **Body transforms are rigid: rotation and translation, no scale.**
- Tests use a seeded `bevox_core::testing::XorShift64`. No test framework may be added.
- Every performance claim comes from interleaved A/B/A within one session, with drift reported.
- A scene with zero bodies must render bit-identically. This plan changes no shader.
- **Physics stays off master.** Commit on `feat/rigid-body-physics`; push to its GitHub copy only when Flori asks; never delete that copy.
- Verify with `cargo test --workspace` (about 2 minutes). Redirect output to a file and check `$?`; never pipe cargo through `tee` or `tail`. A transient LNK1102 or LNK1104 gets one retry.
- Do not run the app. It opens a window a human must close; Flori tests it.
- Every correctness gate is proven by a deliberate break.

## What exists now

- `Body::march_world(origin, dir, max_dist, stats) -> Option<Hit>`, where `Hit` has `t` (world distance, preserved because the transform is rigid), `voxel` and `face_normal` in the body's own frame.
- `pick::pick_voxel(tree, world_from_clip, eye, ndc) -> Option<Pick>`, world only.
- `Body::recompute(&materials)`, which moves the pivot to the centre of mass without moving a voxel and returns `false` for an empty volume.
- `Contree::apply_sphere`, `Contree::voxels`, `Contree::from_voxels`, `Contree::clear_voxels`.
- `physics::detach::{loose_pieces, detach}`, and a test-only full labelling of pieces.
- The app's `brush_input`: left click paints, right click erases and detaches, both against the world only.

## Facts that are not obvious from the code

**A stroke edits what the cursor is on, and only that.** A sphere that overlaps both a body and the ground behind it edits the body. Editing both would carve the ground under a body the user was sculpting.

**Painting can outgrow a body's volume.** A detached piece sits in the smallest power-of-four volume that fits it, so painting onto its outside usually reaches past the volume's edge, where `apply_sphere` would silently drop the paint. The volume is rebuilt larger first, and its contents shifted by whole voxels. Shifting the contents by `s` and the centre of mass by `s` together leaves `world_from_local` unchanged, so no voxel moves.

**A split is a rigid-body identity, not a guess.** Just before the split, every point of the body moves with `v + ω × (p − c)`. Each piece is given exactly the velocity its own centre of mass had, and the same spin. Summing `m_i (v + ω × (c_i − c))` over the pieces gives `M v`, because the pieces' mass-weighted centres average to `c`. That is momentum conservation, and it is a gate.

**The largest piece keeps the parent's identity.** Its id, and so its place in anything keyed on it, carries over. The others are new bodies.

**No room means no split.** Past `MAX_BODIES` a body is not drawn, so a split that would exceed the cap leaves the pieces together as one body. A rigid body can hold disconnected voxels; nothing breaks, and nothing vanishes.

**A body erased to nothing is removed.**

**Every body edit changes packed geometry,** so it bumps the scene's generation, as adding or removing a body does.

## File Structure

- `crates/bevox_core/src/physics/detach.rs`: `components`, the full labelling, promoted from the test into the module.
- `crates/bevox_core/src/body.rs`: `angular_velocity` and `set_angular_velocity`, and `grow_to_fit`.
- `crates/bevox_core/src/physics/sculpt.rs` (**new**): painting, erasing and splitting a body.
- `crates/bevox_render/src/pick.rs`: `pick`, which sees bodies.
- `crates/bevox/src/main.rs`: strokes route to the world or to a body.

---

### Task 1: Labelling a body's pieces, and its spin

**Files:**
- Modify: `crates/bevox_core/src/physics/detach.rs`
- Modify: `crates/bevox_core/src/body.rs`

**Interfaces:**
- Produces:
  - `pub fn components(tree: &Contree) -> Vec<Vec<UVec3>>`, every face-connected piece, each sorted, largest first
  - `Body::angular_velocity(&self) -> Vec3`
  - `Body::set_angular_velocity(&mut self, omega: Vec3)`

- [ ] **Step 1: Write the failing tests**

In `detach.rs`:

```rust
    #[test]
    fn components_are_labelled_largest_first() {
        let mut voxels = Vec::new();
        for x in 0..5 {
            voxels.push((UVec3::new(x, 0, 0), MaterialId(1)));
        }
        for x in 8..10 {
            voxels.push((UVec3::new(x, 0, 0), MaterialId(1)));
        }
        let tree = Contree::from_voxels(16, &voxels);
        let pieces = components(&tree);
        assert_eq!(pieces.iter().map(Vec::len).collect::<Vec<_>>(), vec![5, 2]);
        assert!(components(&Contree::empty(2)).is_empty());
    }
```

In `body.rs`:

```rust
    /// Spin set is spin read back, for a body turned any way: the world
    /// inertia rotates with it.
    #[test]
    fn angular_velocity_round_trips() {
        let mut voxels = Vec::new();
        for x in 0..6 {
            voxels.push((UVec3::new(x, 0, 0), MaterialId(1)));
        }
        voxels.push((UVec3::new(0, 1, 0), MaterialId(2)));
        let materials = crate::physics::fixtures::materials();
        let mut body = Body::new(
            Contree::from_voxels(16, &voxels),
            Vec3::ZERO,
            Quat::from_euler(glam::EulerRot::XYZ, 0.4, 1.2, -0.3),
        );
        assert!(body.recompute(&materials));
        let omega = Vec3::new(0.7, -1.1, 0.4);
        body.set_angular_velocity(omega);
        assert!((body.angular_velocity() - omega).length() < 1e-4, "{:?}", body.angular_velocity());
    }
```

- [ ] **Step 2: Run to verify they fail**, then **Step 3: implement**.

`components` walks every solid voxel of the tree, face neighbours only, with no budget and no floor: inside a body, every piece is loose. Sort each piece with `sort_unstable_by_key(|p| (p.z, p.y, p.x))`, and the list by length, largest first.

`the_search_agrees_with_labelling_every_piece_in_full` keeps its own reference. It exists to be independent of the code it checks, so it does not switch to `components`.

In `body.rs`:

```rust
    /// The inverse inertia tensor in world axes.
    fn world_inverse_inertia(&self) -> glam::Mat3 {
        let r = glam::Mat3::from_quat(self.orientation);
        r * self.mass.inverse_inertia * r.transpose()
    }

    /// How fast the body turns, in world axes.
    pub fn angular_velocity(&self) -> Vec3 {
        self.world_inverse_inertia() * self.angular_momentum
    }

    /// Sets the spin, by setting the angular momentum that produces it.
    pub fn set_angular_velocity(&mut self, omega: Vec3) {
        self.angular_momentum = self.world_inverse_inertia().inverse() * omega;
    }
```

- [ ] **Step 4: Run to verify they pass.** `cargo test --workspace`, `0`, no warnings.

- [ ] **Step 5: Break checks.** Make `components` add diagonal neighbours: the test must FAIL. Make `set_angular_velocity` skip the rotation (`self.mass.inverse_inertia.inverse() * omega`): the round trip must FAIL. Restore each.

- [ ] **Step 6: Commit** — `feat(core): label a body's pieces, and read and set its spin`.

---

### Task 2: Painting and erasing a body

**Files:**
- Create: `crates/bevox_core/src/physics/sculpt.rs`
- Modify: `crates/bevox_core/src/physics/mod.rs`, `crates/bevox_core/src/body.rs`

**Interfaces:**
- Produces:
  - `Body::grow_to_fit(&mut self, lo: IVec3, hi: IVec3)`, which makes the volume cover the local box `lo..=hi` without moving any voxel
  - `pub fn sculpt(bodies: &mut Vec<Body>, index: usize, centre: Vec3, radius: f32, material: MaterialId, materials: &MaterialTable, max_bodies: usize)`: paints or erases a sphere, given in world space, on `bodies[index]`, and splits or removes it as needed

- [ ] **Step 1: Write the failing tests**

In `sculpt.rs`:

```rust
#[cfg(test)]
mod tests {
    use super::*;
    use crate::physics::fixtures::{cube, materials, placed};
    use glam::{EulerRot, Quat};

    fn world_voxels(body: &Body) -> Vec<[i32; 3]> {
        let mut out: Vec<[i32; 3]> = body
            .volume
            .voxels()
            .iter()
            .map(|(p, _)| {
                (body.world_from_local().transform_point3(p.as_vec3() + 0.5) - 0.5)
                    .round()
                    .as_ivec3()
                    .to_array()
            })
            .collect();
        out.sort_unstable();
        out
    }

    /// Painting onto a body adds voxels where the sphere is and leaves the old
    /// ones where they were, even past the edge of its volume.
    #[test]
    fn painting_grows_a_body_past_its_volume() {
        let materials = materials();
        let mut bodies = vec![placed(cube(4, 4), Vec3::new(20.0, 20.0, 20.0), Quat::IDENTITY)];
        let before = world_voxels(&bodies[0]);
        let mass = bodies[0].mass.mass;
        // Beyond the +x face: the 4-extent volume cannot hold it as it is.
        sculpt(&mut bodies, 0, Vec3::new(23.5, 20.0, 20.0), 1.5, MaterialId(2), &materials, 16);

        let after = world_voxels(&bodies[0]);
        assert!(after.len() > before.len(), "nothing was painted");
        assert!(bodies[0].mass.mass > mass);
        for v in &before {
            assert!(after.contains(v), "painting moved or lost voxel {v:?}");
        }
        assert!(after.iter().any(|v| v[0] >= 22), "no paint landed past the old volume");
    }

    /// Erasing a bar through its middle leaves two bodies, the same voxels in
    /// the same places, and the same momentum, whatever the bar was doing.
    #[test]
    fn erasing_a_bar_in_two_splits_it() {
        let materials = materials();
        let bar: Vec<_> = (0..12)
            .flat_map(|x| (0..2).map(move |z| (UVec3::new(x, 0, z), MaterialId(1))))
            .collect();
        let turn = Quat::from_euler(EulerRot::XYZ, 0.3, 0.8, -0.2);
        let mut body = placed(Contree::from_voxels(16, &bar), Vec3::new(30.0, 30.0, 30.0), turn);
        body.velocity = Vec3::new(4.0, -1.0, 2.0);
        body.set_angular_velocity(Vec3::new(0.5, 1.5, -0.8));
        let id = body.id;
        let momentum = body.velocity * body.mass.mass;
        let middle = body.world_from_local().transform_point3(Vec3::new(6.0, 0.5, 1.0));
        let before = world_voxels(&body);
        let mut bodies = vec![body];

        sculpt(&mut bodies, 0, middle, 1.2, MaterialId::EMPTY, &materials, 16);

        assert_eq!(bodies.len(), 2, "the bar did not split");
        assert_eq!(bodies[0].id, id, "the larger piece lost the parent's identity");
        let after: Vec<[i32; 3]> = bodies.iter().flat_map(world_voxels).collect();
        for v in &after {
            assert!(before.contains(v), "a piece gained voxel {v:?}");
        }
        let total: Vec3 = bodies.iter().map(|b| b.velocity * b.mass.mass).sum();
        let lost = before.len() - after.len();
        let erased_share = lost as f32 / before.len() as f32;
        // The erased voxels took their share of the momentum with them.
        let expected = momentum * (1.0 - erased_share);
        assert!(
            (total - expected).length() < 0.05 * expected.length() + 1e-3 * momentum.length(),
            "momentum {total:?}, expected about {expected:?}"
        );
        for b in &bodies {
            assert!(
                (b.angular_velocity() - Vec3::new(0.5, 1.5, -0.8)).length() < 1e-3,
                "a piece lost the spin: {:?}",
                b.angular_velocity()
            );
        }
    }

    /// With no room for another body, the pieces stay together.
    #[test]
    fn a_split_with_no_room_keeps_the_pieces_together() {
        let materials = materials();
        let bar: Vec<_> = (0..12).map(|x| (UVec3::new(x, 0, 0), MaterialId(1))).collect();
        let mut bodies =
            vec![placed(Contree::from_voxels(16, &bar), Vec3::new(30.0, 30.0, 30.0), Quat::IDENTITY)];
        let middle = bodies[0].world_from_local().transform_point3(Vec3::new(6.0, 0.5, 0.5));
        sculpt(&mut bodies, 0, middle, 1.2, MaterialId::EMPTY, &materials, 1);
        assert_eq!(bodies.len(), 1, "the split went past the cap");
        assert!(bodies[0].mass.mass > 0.0);
    }

    #[test]
    fn a_body_erased_to_nothing_is_removed() {
        let materials = materials();
        let mut bodies = vec![placed(cube(4, 4), Vec3::new(20.0, 20.0, 20.0), Quat::IDENTITY)];
        sculpt(&mut bodies, 0, Vec3::new(20.0, 20.0, 20.0), 10.0, MaterialId::EMPTY, &materials, 16);
        assert!(bodies.is_empty());
    }
}
```

- [ ] **Step 2: Run to verify they fail**, then **Step 3: implement**.

`Body::grow_to_fit(lo, hi)`:
- If `lo >= 0` and `hi < extent` on every axis, return.
- Otherwise `shift = (-lo).max(ZERO)` as `UVec3`, and `extent` the smallest power of four, at least the current extent, with `hi + shift < extent`.
- Rebuild the volume from `voxels()` with every coordinate plus `shift`.
- `com += shift`, so `world_from_local` is unchanged.
- Keep mass and features: the caller recomputes after the stroke anyway.

`sculpt`:
1. `let body = &mut bodies[index]`. Carry the centre into the body's grid: `let local = body.local_from_world().transform_point3(centre)`.
2. **Paint** (material not empty):
   - `grow_to_fit((local - radius).floor(), (local + radius).ceil())`;
   - recompute `local`, because growing moved the grid;
   - `apply_sphere(local, radius, material)`;
   - `recompute`. Done.
3. **Erase:**
   1. Before touching anything, record the rigid motion: the centre of mass in the world (`position`), `velocity`, and `angular_velocity()`.
   2. `apply_sphere(local, radius, MaterialId::EMPTY)`.
   3. `let pieces = components(&body.volume)`.
   4. **No pieces:** remove the body; done.
   5. **One piece,** or more pieces than there is room for (`bodies.len() - 1 + pieces.len() > max_bodies`): `recompute` and keep the motion; done.
   6. **Otherwise,** for every piece past the first:
      - build a volume of the same extent from its voxels and materials;
      - make `Body::new(volume, placement, orientation)` with `placement = position - orientation * com`, the parent's volume corner, so its voxels land where they were;
      - `recompute`;
      - set `velocity = v + ω × (new.position − c)` and `set_angular_velocity(ω)`;
      - push it.
   7. **The first piece** stays in the parent: clear the other pieces' voxels from its volume, `recompute`, then set its velocity and spin by the same formula.

- [ ] **Step 4: Run to verify they pass.** `cargo test --workspace`, `0`, no warnings.

- [ ] **Step 5: Break checks.**
1. Skip `grow_to_fit`: the painting gate must FAIL, because no paint lands past the old volume.
2. Give every piece the parent's `velocity`, with no `ω × r` term: the split gate must FAIL on momentum, because the bar spins.
3. Ignore `max_bodies`: the no-room gate must FAIL.
4. In `grow_to_fit`, forget `com += shift`: the painting gate must FAIL, because every old voxel moves.

Restore each.

- [ ] **Step 6: Commit** — `feat(core): paint and erase bodies, and split what an erase cuts in two`.

---

### Task 3: Picking bodies

**Files:**
- Modify: `crates/bevox_render/src/pick.rs`

**Interfaces:**
- Produces:
  - `pub enum Target { World, Body(usize) }`
  - `pub struct Hit { pub target: Target, pub position: Vec3, pub normal: Vec3 }`, where `normal` is in world space
  - `pub fn pick(tree: &Contree, bodies: &[Body], world_from_clip: Mat4, eye: Vec3, ndc: Vec2) -> Option<Hit>`

- [ ] **Step 1: Write the failing tests**

```rust
    /// A body in front of the world is what the cursor is on.
    #[test]
    fn a_body_in_front_of_the_world_is_picked() {
        let tree = block_scene();
        let eye = Vec3::new(32.0, 32.0, -20.0);
        let world_from_clip = camera(eye, Vec3::splat(32.0));
        let volume = bevox_core::contree::Contree::from_voxels(
            4,
            &(0..64).map(|i| (UVec3::new(i % 4, (i / 4) % 4, i / 16), MaterialId(2))).collect::<Vec<_>>(),
        );
        let turned = glam::Quat::from_rotation_y(0.6);
        let body = Body::new(volume, Vec3::new(30.0, 30.0, 5.0), turned);
        let hit = pick(&tree, std::slice::from_ref(&body), world_from_clip, eye, Vec2::ZERO).unwrap();
        assert_eq!(hit.target, Target::Body(0));
        assert!(hit.position.z < 10.0, "hit {:?}, behind the body", hit.position);
        assert!((hit.normal.length() - 1.0).abs() < 1e-4);
        assert!(hit.normal.dot(eye - hit.position) > 0.0, "the normal faces away from the eye");

        let far = Body::new(bevox_core::contree::Contree::empty(1), Vec3::new(30.0, 30.0, 5.0), turned);
        let hit = pick(&tree, std::slice::from_ref(&far), world_from_clip, eye, Vec2::ZERO).unwrap();
        assert_eq!(hit.target, Target::World);
    }
```

`camera` and `block_scene` are this module's existing test helpers; check their names, and match the eye to them.

- [ ] **Step 2: Run to verify it fails**, then **Step 3: implement.**
- Factor the ray out of `pick_voxel` into `fn ray(world_from_clip, eye, ndc) -> Vec3`, and have `pick_voxel` use it.
- `pick` marches the world, then every body with `march_world`, and keeps the nearest `t`.
- A body hit's normal is `body.orientation * hit.face_normal`, since the face normal comes back in the body's own frame.
- `position` is `eye + dir * t` for either kind of hit.

- [ ] **Step 4: Run to verify it passes**, then **Step 5: break check.** Leave the body's normal in its own frame: the normal assertion must FAIL for the turned body. Restore.

- [ ] **Step 6: Commit** — `feat(render): pick bodies as well as the world`.

---

### Task 4: The app routes strokes

**Files:**
- Modify: `crates/bevox/src/main.rs`

- [ ] **Step 1: Write the failing test**

```rust
    /// A stroke on a body edits the body, not the world behind it, and asks for
    /// a rebuild because packed geometry changed.
    #[test]
    fn a_stroke_on_a_body_edits_the_body() {
        let (tree, materials) = demo_scene();
        let field = DistanceField::build(&tree);
        let mut body = demo_body(Vec3::new(10.0, 12.0, 50.0), Quat::IDENTITY);
        assert!(body.recompute(&materials));
        let voxels = body.volume.voxels().len();
        let world = tree.voxels().len();
        let mut scene = VoxelScene { tree, materials, generation: 1, field, field_dirty: None, bodies: vec![body] };
        stroke(&mut scene, Target::Body(0), Vec3::new(10.0, 12.0, 50.0), 2.0, MaterialId::EMPTY);
        assert!(scene.bodies[0].volume.voxels().len() < voxels, "the body was not edited");
        assert_eq!(scene.tree.voxels().len(), world, "the world was edited too");
        assert_eq!(scene.generation, 2);
    }
```

- [ ] **Step 2: Run to verify it fails**, then **Step 3: implement.**
- Add `fn stroke(scene, target, centre, radius, material)`:
  - for the world, it does what `brush_input` does today: paint with `apply_brush`, or erase with `erase_and_detach`;
  - for a body, it calls `sculpt(&mut scene.bodies, i, centre, radius, material, &scene.materials, MAX_BODIES)` and bumps `generation`.
- `brush_input` calls `pick` with `&scene.bodies`, works out the centre as today (paint on the near side of the surface), and calls `stroke`.
- The comment in `setup` that says picking does not see bodies goes.

- [ ] **Step 4: Run to verify it passes.** `cargo test --workspace`, `0`, no warnings.

- [ ] **Step 5: Break check.** Route body strokes to the world: the test must FAIL. Restore.

- [ ] **Step 6: Commit** — `feat(app): the brush paints and erases bodies`.

- [ ] **Step 7: Hand over.** Tell Flori what to try:
  - drop a cube with `F`;
  - right-click through its middle, and it splits into two that keep moving;
  - left-click its side, and it grows.

  Do not merge; push only if asked.

---

## Milestone check

Painting a body grows it without moving a voxel, even past its volume's edge. Erasing a body in two leaves two bodies that conserve the parent's momentum and spin. A body erased to nothing is removed, and a split with no room stays whole.

## What this plan deliberately does not do

- **No joints and no grab.** That is milestone 5.
- **No detaching what a paint connects.** Painting a body into the ground does not weld it to the world; it stays a body. Welding is merge-back, deferred.
- **No stroke that edits the world and a body at once.**
