# Body Against Body Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking. **Flori prefers inline execution for this project.**

**Goal:** Bodies collide with each other as they do with the world: a thrown cube knocks another along, and a stack of cubes stands still instead of sinking into itself.

**Architecture:** The tick stops being per body. Contacts are collected for every body against the world and for every overlapping pair of bodies, into one list that names which bodies each contact joins. The substep loop then integrates every body, warm starts every contact, solves every contact and relaxes, exactly as now, but each impulse is applied to both sides. Detection between two bodies reuses the same rounded-voxel pair tests, run in the second body's frame.

**Tech Stack:** Rust stable 1.96, Bevy 0.19.1, glam 0.32. No new dependencies.

**Spec:** `docs/superpowers/specs/2026-09-15-bevox-rigid-bodies-design.md`. This is the second half of its milestone 3.

## Global Constraints

- Native desktop only. No web build, ever.
- `bevox_core` has no Bevy and no GPU dependency, and gains no new dependencies. No new dependencies in any crate.
- **Body transforms are rigid: rotation and translation, no scale.**
- Tests use a seeded `bevox_core::testing::XorShift64`. No test framework may be added.
- Every performance claim comes from interleaved A/B/A within one session, with drift reported.
- A scene with zero bodies must render bit-identically. This plan changes no shader.
- **Physics stays on the local branch `feat/rigid-body-physics`: no push, no merge to master.**
- Verify with `cargo test --workspace` (about 2 minutes). Redirect output to a file and check `$?`; never pipe cargo through `tee` or `tail`. A transient LNK1102 or LNK1104 gets one retry.
- Do not run the app. It opens a window a human must close; Flori tests it.
- Every correctness gate is proven by a deliberate break.

## What exists now

- `physics::contact::detect(body, tree, field, materials, margin) -> Vec<Contact>`, sorted by key, where `Contact` is `{ key: (UVec3, IVec3), normal, separation, anchor, world_point, friction, restitution }`. `anchor` is relative to the body's centre of mass in its own axes, and `normal` points from the world into the body.
- `sphere_vs_voxel(q, c, shape)` and `edge_vs_edge(p, d, c, e)`, both pure.
- `physics::solver::step(bodies, tree, field, materials, gravity, dt) -> bool`, which per body: caps speed, detects, warm starts, runs `SUBSTEPS` substeps of gravity, biased solve, friction, integrate, relax, then sweeps restitution `RESTITUTION_SWEEPS` times.
- `Body::warm: HashMap<ContactKey, ContactImpulse>`, `ContactImpulse { normal, tangent }`.
- Constants: `GRAVITY`, `SUBSTEPS`, `SLOP`, `BIAS`, `MAX_PUSH`, `MAX_TRAVEL`, `BASE_MARGIN`, `RESTITUTION_SWEEPS`.

## Facts that are not obvious from the code

**A body needs an identity that outlives its index.** Warm starting keys on which two voxels touch, and once the other side is a body rather than the world, on which body. Indices move when a body is removed, so `Body` gains an `id` from a counter, and contacts key on it. The world keeps the reserved id 0.

**Both sides of a pair contact move, so the separation update needs both.** Against the world, the current separation is the gap at detection plus how far the body's anchor has travelled along the normal. For a pair it is the gap plus the difference between the two anchors' travel.

**The effective mass sums both bodies' terms.** `k = inv_mass_a + inv_mass_b + n·((Ia⁻¹(ra×n))×ra) + n·((Ib⁻¹(rb×n))×rb)`. Against the world, the second body's terms are zero, which is exactly today's formula: keeping one code path means the world is a body whose inverse mass and inverse inertia are zero.

**Impulses go on opposite ways.** The normal points from body B into body A, so A receives `+P` and B receives `-P`.

**Detection between two bodies runs in B's frame.** A's candidate voxel centre goes through `B.local_from_world() * A.world_from_local()`, which is one rigid transform, and B's voxels are then axis-aligned. The normal that comes back is in B's frame and rotates into the world by B's orientation.

**Symmetry has to be broken, or a pair is detected twice.** Every pair is visited once, with the lower index as A.

**The per-body speed cap must run before any detection,** because a pair's margin depends on both bodies' travel. Cap every body first, then detect everything.

**Body pairs need their own broadphase.** The distance field describes the static world only. For pairs, the two world boxes must overlap; at a cap of 16 bodies that is 120 box tests, which is nothing.

**This is where the resting gates earn their keep.** A stack is the classic way an impulse solver fails: the bottom body sinks, or the stack shivers. Milestone 2's single-body gates cannot see it, so this plan adds stack gates with the same shape: height now against height after ten thousand ticks.

## File Structure

- `crates/bevox_core/src/body.rs`: `BodyId`, `Body::id`, and `warm` keyed by the new key.
- `crates/bevox_core/src/physics/contact.rs`: `ContactKey` gains the other body; `Contact` gains the second anchor; `detect_pair`.
- `crates/bevox_core/src/physics/solver.rs`: one contact list for the whole scene, impulses applied to both sides.
- `crates/bevox/src/main.rs`: the `F` drop already stacks bodies; no change expected beyond what compiles.

---

### Task 1: Body identity

**Files:**
- Modify: `crates/bevox_core/src/body.rs`
- Modify: `crates/bevox_core/src/physics/contact.rs` (key), `crates/bevox_core/src/physics/solver.rs` (compile)

**Interfaces:**
- Produces:
  - `pub struct BodyId(pub u64)` with `pub const WORLD: BodyId = BodyId(0);`
  - `Body::id: BodyId`, assigned by `Body::new` from an atomic counter starting at 1
  - `ContactKey { pub other: BodyId, pub mine: UVec3, pub theirs: UVec3 }`, `Hash + Eq + Ord`

- [ ] **Step 1: Write the failing tests**

In `body.rs`'s test module:

```rust
    /// Two bodies are never the same body, and a clone is the same body: the
    /// warm-start cache keys on this, and the render tests clone bodies.
    #[test]
    fn every_body_gets_its_own_id() {
        let a = Body::new(cube(), Vec3::ZERO, Quat::IDENTITY);
        let b = Body::new(cube(), Vec3::ZERO, Quat::IDENTITY);
        assert_ne!(a.id, b.id);
        assert_eq!(a.id, a.clone().id);
        assert_ne!(a.id, BodyId::WORLD, "the world's id is reserved");
    }
```

- [ ] **Step 2: Run to verify it fails**

Run: `cargo test -p bevox_core every_body_gets > b.txt 2>&1; echo $?`
Expected: non-zero; `BodyId` does not exist.

- [ ] **Step 3: Implement**

In `body.rs`:

```rust
/// Which body a contact is against. The static world is `WORLD`.
///
/// An identity rather than an index: bodies are removed from the middle of the
/// scene's list, and a warm-start impulse must not follow the body that takes
/// the vacated slot.
#[derive(Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Debug)]
pub struct BodyId(pub u64);

impl BodyId {
    pub const WORLD: BodyId = BodyId(0);
}

fn next_id() -> BodyId {
    use std::sync::atomic::{AtomicU64, Ordering};
    static NEXT: AtomicU64 = AtomicU64::new(1);
    BodyId(NEXT.fetch_add(1, Ordering::Relaxed))
}
```

Add `pub id: BodyId` to `Body`, set from `next_id()` in `new`, and document that a clone keeps the id because it is the same body.

In `contact.rs`, replace the tuple key:

```rust
/// Which voxel of which body touched which voxel of what.
///
/// Stable from tick to tick while the touch persists, which is what warm
/// starting keys on.
#[derive(Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Debug)]
pub struct ContactKey {
    /// The world, or the other body.
    pub other: BodyId,
    /// The voxel of the body this contact belongs to.
    pub mine: UVec3,
    /// The voxel of `other`. For the world, its world-space voxel coordinate,
    /// which is never negative where a contact can be.
    pub theirs: UVec3,
}
```

`detect` builds keys with `other: BodyId::WORLD` and `theirs: k.as_uvec3()`. The world-corner loop already has `k` non-negative, because every caller tested it with `world_solid`.

Sorting by key becomes `contacts.sort_unstable_by_key(|c| c.key)`.

Update the tests that read `c.key.0` and `c.key.1` to `c.key.mine` and `c.key.theirs`, and the solver's key-stability test likewise.

- [ ] **Step 4: Run to verify it passes**

Run: `cargo test --workspace > t.txt 2>&1; echo $?`
Expected: `0`, with no warnings.

- [ ] **Step 5: Break check**

Make `next_id` return `BodyId(1)` always. `every_body_gets_its_own_id` must FAIL. Restore.

- [ ] **Step 6: Commit**

```bash
git add crates
git commit -m "feat(core): give every body an identity, for contacts between bodies" -m "Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>"
```

---

### Task 2: Contacts between two bodies

**Files:**
- Modify: `crates/bevox_core/src/physics/contact.rs`

**Interfaces:**
- Consumes: `BodyId` (Task 1)
- Produces:
  - `Contact` gains `pub other_anchor: Vec3` and `pub other_point: Vec3`; the world's contacts leave both at `Vec3::ZERO`
  - `pub fn boxes_overlap(a: (Vec3, Vec3), b: (Vec3, Vec3)) -> bool`
  - `pub fn detect_pair(a: &Body, b: &Body, materials: &MaterialTable, margin: f32) -> Vec<Contact>`, whose contacts belong to `a`, carry `key.other == b.id`, and whose normals point from `b` into `a`

- [ ] **Step 1: Write the failing tests**

In `contact.rs`'s test module:

```rust
    /// Two cubes stacked exactly touch at the four corners of the upper one's
    /// bottom face, with the normal pointing up into the upper body.
    #[test]
    fn a_cube_resting_on_a_cube_touches_at_four_corners() {
        let materials = materials();
        let lower = placed(cube(4, 4), Vec3::new(32.0, 10.0, 32.0), Quat::IDENTITY);
        let upper = placed(cube(4, 4), Vec3::new(32.0, 14.0, 32.0), Quat::IDENTITY);
        let contacts = detect_pair(&upper, &lower, &materials, 0.1);
        assert_eq!(contacts.len(), 4, "{contacts:#?}");
        for c in &contacts {
            assert_eq!(c.key.other, lower.id);
            assert!((c.normal - Vec3::Y).length() < TOLERANCE, "normal {:?}", c.normal);
            assert!(c.separation.abs() < TOLERANCE, "separation {}", c.separation);
            assert_eq!(c.key.mine.y, 0, "the upper body touched with voxel {:?}", c.key.mine);
            assert_eq!(c.key.theirs.y, 3, "the lower body touched with voxel {:?}", c.key.theirs);
        }
    }

    /// Turning the pair as a whole turns the contact with it: the normal is the
    /// same in the bodies' shared frame, whatever the world's axes.
    #[test]
    fn a_turned_pair_touches_the_same_way() {
        let materials = materials();
        let turn = Quat::from_rotation_z(0.9);
        let lower = placed(cube(4, 4), Vec3::new(32.0, 20.0, 32.0), turn);
        let upper = placed(cube(4, 4), turn * Vec3::new(0.0, 4.0, 0.0) + Vec3::new(32.0, 20.0, 32.0), turn);
        let contacts = detect_pair(&upper, &lower, &materials, 0.1);
        assert_eq!(contacts.len(), 4, "{contacts:#?}");
        for c in &contacts {
            assert!((c.normal - turn * Vec3::Y).length() < 1e-3, "normal {:?}", c.normal);
            assert!(c.separation.abs() < 1e-3, "separation {}", c.separation);
        }
    }

    /// Bodies that do not overlap are not compared voxel by voxel.
    #[test]
    fn far_apart_bodies_have_no_contacts_and_no_overlap() {
        let materials = materials();
        let a = placed(cube(4, 4), Vec3::new(32.0, 10.0, 32.0), Quat::IDENTITY);
        let b = placed(cube(4, 4), Vec3::new(32.0, 40.0, 32.0), Quat::IDENTITY);
        assert!(!boxes_overlap(world_box(&a, 0.1).unwrap(), world_box(&b, 0.1).unwrap()));
        assert!(detect_pair(&a, &b, &materials, 0.1).is_empty());
    }

    /// The pair's coefficients come from the two voxels, exactly as against the
    /// world: an icy body on a gripping one is slippery.
    #[test]
    fn a_pair_takes_the_coefficients_of_both_voxels() {
        let materials = materials();
        let lower = placed(cube(4, 4), Vec3::new(32.0, 10.0, 32.0), Quat::IDENTITY);
        let icy = placed(cube_of(4, 4, MaterialId(3)), Vec3::new(32.0, 14.0, 32.0), Quat::IDENTITY);
        for c in detect_pair(&icy, &lower, &materials, 0.1) {
            assert_eq!(c.friction, 0.0, "ice on stone should slide");
        }
    }
```

- [ ] **Step 2: Run to verify they fail**

Run: `cargo test -p bevox_core contact > b.txt 2>&1; echo $?`
Expected: non-zero; `detect_pair` does not exist.

- [ ] **Step 3: Implement**

In `contact.rs`:

1. `Contact` gains the second side:

   ```rust
       /// The contact point relative to the OTHER body's centre of mass, in its
       /// own axes. Zero against the world, which does not move.
       pub other_anchor: Vec3,
       /// Where that point was in the world at detection. Zero against the world.
       pub other_point: Vec3,
   ```

   `keep` sets both to `Vec3::ZERO`; `detect_pair` fills them.

2. The pair broadphase:

   ```rust
   /// Whether two world-space boxes overlap.
   pub fn boxes_overlap(a: (Vec3, Vec3), b: (Vec3, Vec3)) -> bool {
       a.0.cmple(b.1).all() && b.0.cmple(a.1).all()
   }
   ```

3. `detect_pair`, which mirrors `detect` with `b` standing in for the world:

   ```rust
   /// Every contact between two bodies within `margin`, belonging to `a`.
   ///
   /// Runs in `b`'s frame, where `b`'s voxels are axis-aligned, exactly as
   /// `detect` runs in the world's. Normals point from `b` into `a`.
   pub fn detect_pair(a: &Body, b: &Body, materials: &MaterialTable, margin: f32) -> Vec<Contact> {
       let (Some(box_a), Some(box_b)) = (world_box(a, margin), world_box(b, margin)) else {
           return Vec::new();
       };
       if !boxes_overlap(box_a, box_b) {
           return Vec::new();
       }
       let b_from_a = b.local_from_world() * a.world_from_local();
       let a_from_b = a.local_from_world() * b.world_from_local();
       let reach = (margin + RADIUS).ceil() as i32;
       let a_solid = |p: IVec3| solid_at(&a.volume, p);
       let b_solid = |p: IVec3| solid_at(&b.volume, p);
       let mut found = HashMap::new();

       // A's corners against B's voxels.
       for &u in &a.features.corners {
           let q = b_from_a.transform_point3(u.as_vec3() + 0.5);
           for k in around(q, reach) {
               if !b_solid(k) {
                   continue;
               }
               if let Some((sep, n_b)) = sphere_vs_voxel(q, k.as_vec3() + 0.5, classify(b_solid, k)) {
                   let point_b = q - n_b * (RADIUS + sep * 0.5);
                   keep_pair(&mut found, a, b, materials, margin, u, k.as_uvec3(), n_b, sep, point_b);
               }
           }
       }

       // B's corners against A's voxels. The normal comes back pointing from A
       // into B, so it is reversed before it is stored.
       for &w in &b.features.corners {
           let q = a_from_b.transform_point3(w.as_vec3() + 0.5);
           for k in around(q, reach) {
               if !a_solid(k) {
                   continue;
               }
               if let Some((sep, n_a)) = sphere_vs_voxel(q, k.as_vec3() + 0.5, classify(a_solid, k)) {
                   let point_a = q - n_a * (RADIUS + sep * 0.5);
                   let point_b = b_from_a.transform_point3(point_a);
                   let n_b = b_from_a.transform_vector3(-n_a);
                   keep_pair(&mut found, a, b, materials, margin, k.as_uvec3(), w, n_b, sep, point_b);
               }
           }
       }

       // A's edges against B's edges.
       for &(u, axis) in &a.features.edges {
           let p = b_from_a.transform_point3(u.as_vec3() + 0.5);
           let d = b_from_a.transform_vector3(AXES[axis]);
           for k in around(p, reach) {
               if !b_solid(k) {
                   continue;
               }
               let Shape::Edge(b_axis) = classify(b_solid, k) else { continue };
               if let Some((sep, n_b, point_b)) = edge_vs_edge(p, d, k.as_vec3() + 0.5, AXES[b_axis]) {
                   keep_pair(&mut found, a, b, materials, margin, u, k.as_uvec3(), n_b, sep, point_b);
               }
           }
       }

       let mut contacts: Vec<Contact> = found.into_values().collect();
       contacts.sort_unstable_by_key(|c| c.key);
       contacts
   }

   /// Records a pair contact. `normal` and `point` arrive in B's frame, where
   /// the pair tests ran, and are carried into the world here.
   #[allow(clippy::too_many_arguments)]
   fn keep_pair(
       found: &mut HashMap<ContactKey, Contact>,
       a: &Body,
       b: &Body,
       materials: &MaterialTable,
       margin: f32,
       mine: UVec3,
       theirs: UVec3,
       normal: Vec3,
       separation: f32,
       point: Vec3,
   ) {
       if separation > margin {
           return;
       }
       let key = ContactKey { other: b.id, mine, theirs };
       let world_point = b.world_from_local().transform_point3(point);
       let ma = materials.get(a.volume.get(mine));
       let mb = materials.get(b.volume.get(theirs));
       found.entry(key).or_insert(Contact {
           key,
           normal: b.orientation * normal,
           separation,
           anchor: a.local_from_world().transform_point3(world_point) - a.com,
           world_point,
           other_anchor: point - b.com,
           other_point: world_point,
           friction: combine_friction(ma.friction, mb.friction),
           restitution: combine_restitution(ma.restitution, mb.restitution),
       });
   }
   ```

   `Affine3A::transform_vector3` rotates without translating, which is what a normal and an edge direction need.

- [ ] **Step 4: Run to verify they pass**

Run: `cargo test -p bevox_core contact > t.txt 2>&1; echo $?`
Expected: `0`.

- [ ] **Step 5: Break checks**

1. Leave B's corner loop's normal unreversed (`b_from_a.transform_vector3(n_a)`). `a_cube_resting_on_a_cube_touches_at_four_corners` must FAIL: half the contacts point down.
2. Skip the frame change, using `u.as_vec3() + 0.5` as `q` in A's corner loop. `a_turned_pair_touches_the_same_way` must FAIL.
3. Make `boxes_overlap` always true. `far_apart_bodies_have_no_contacts_and_no_overlap` must FAIL.

Restore each and re-run.

- [ ] **Step 6: Commit**

```bash
git add crates
git commit -m "feat(core): rounded-voxel contacts between two bodies" -m "Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>"
```

---

### Task 3: One contact list for the scene

**Files:**
- Modify: `crates/bevox_core/src/physics/solver.rs`

**Interfaces:**
- Consumes: `detect_pair`, `Contact::other_anchor` (Task 2)
- Produces: `step` unchanged in signature, but solving every body and pair together

- [ ] **Step 1: Write the failing tests**

In `solver.rs`'s test module:

```rust
    /// A moving cube hitting a still one of equal mass stops it dead and sends
    /// the other on: momentum is conserved and the collision is inelastic.
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

        let after = bodies.iter().map(|b| b.velocity * b.mass.mass).sum::<Vec3>();
        assert!((after - before).length() < 0.02 * before.length(), "momentum {after:?}, was {before:?}");
        assert!(bodies[1].velocity.x > 1.0, "the still cube was not knocked along: {:?}", bodies[1].velocity);
        assert!(bodies[0].velocity.x < bodies[1].velocity.x + 0.5, "the hitter passed through");
        assert!(bodies[0].position.x + 4.5 < bodies[1].position.x, "the cubes overlap");
    }

    /// Three cubes stacked stay stacked, and stay at the height they settled at.
    /// A stack is how an impulse solver fails: the bottom sinks, or it shivers.
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
            assert!((y - want).abs() < 0.15, "body {i} settled at {y}, want about {want}");
        }

        run(&mut bodies, &world, &field, &materials, GRAVITY, 8000);
        for (i, (b, was)) in bodies.iter().zip(&settled).enumerate() {
            assert!((b.position.y - was).abs() < 5e-3, "body {i} drifted from {was} to {}", b.position.y);
            assert!(b.velocity.length() < 1e-2, "body {i} still moving at {:?}", b.velocity);
        }
    }

    /// A body resting on another is held up by it, not by the floor: removing
    /// the lower one drops the upper.
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
        bodies.remove(0);
        run(&mut bodies, &world, &field, &materials, GRAVITY, 10);
        assert!(bodies[0].velocity.y < -1.0, "the upper body hung in the air: {:?}", bodies[0].velocity);
    }
```

- [ ] **Step 2: Run to verify they fail**

Run: `cargo test -p bevox_core solver > b.txt 2>&1; echo $?`
Expected: non-zero. The cubes pass through each other, so `a_cube_knocks_another_along` fails.

- [ ] **Step 3: Implement**

Restructure `step`:

```rust
/// Which bodies a contact joins: the one it belongs to, and the one it is
/// against, if that is a body rather than the world.
struct Joined {
    contact: Contact,
    body: usize,
    other: Option<usize>,
}
```

`step` becomes:

1. Remove bodies below the world, as now.
2. Cap every body's speed, and remember each body's travel.
3. Collect contacts:
   - per body, `detect(body, tree, field, materials, travel + BASE_MARGIN)`, with `other: None`;
   - per pair `i < j`, `detect_pair(&bodies[i], &bodies[j], materials, travel_i.max(travel_j) + BASE_MARGIN)`, with `body: i, other: Some(j)`.

   Both loops skip a body with no mass.
4. Seed impulses from `bodies[joined.body].warm`.
5. Record each contact's approach speed, using both bodies.
6. Substeps, over the whole scene each time:
   1. gravity for every body with mass;
   2. warm start every contact;
   3. solve every contact: normal, then friction;
   4. integrate every body;
   5. relax every contact.
7. Write each body's `warm` back from the contacts that belong to it.
8. Sweep restitution.

The per-contact helpers take two bodies. Because both sides may be the same `Vec`, they take indices and use `split_at_mut`, or take `(&mut Body, Option<&mut Body>)` obtained by index sorting. A helper keeps it readable:

```rust
/// The two bodies a contact joins, borrowed at once.
fn pair(bodies: &mut [Body], i: usize, j: Option<usize>) -> (&mut Body, Option<&mut Body>) {
    match j {
        None => (&mut bodies[i], None),
        Some(j) => {
            debug_assert_ne!(i, j);
            let (lo, hi) = bodies.split_at_mut(i.max(j));
            if i < j { (&mut lo[i], Some(&mut hi[0])) } else { (&mut hi[0], Some(&mut lo[j])) }
        }
    }
}
```

The velocity at a contact, its effective mass and its separation all gain the other side:

```rust
/// The closing speed along the contact normal: how fast the two surfaces
/// approach. The world contributes nothing.
fn normal_velocity(a: &Body, b: Option<&Body>, c: &Contact) -> f32 {
    let va = point_velocity(a, a.orientation * c.anchor);
    let vb = b.map_or(Vec3::ZERO, |b| point_velocity(b, b.orientation * c.other_anchor));
    (va - vb).dot(c.normal)
}

fn point_velocity(body: &Body, r: Vec3) -> Vec3 {
    body.velocity + (world_inverse_inertia(body) * body.angular_momentum).cross(r)
}

/// How much impulse one unit of velocity change costs at this contact.
fn effective_mass(a: &Body, b: Option<&Body>, c: &Contact) -> f32 {
    let term = |body: &Body, anchor: Vec3| {
        let r = body.orientation * anchor;
        let inv = world_inverse_inertia(body);
        body.mass.inverse_mass() + (inv * r.cross(c.normal)).cross(r).dot(c.normal)
    };
    term(a, c.anchor) + b.map_or(0.0, |b| term(b, c.other_anchor))
}

/// The gap now: the gap at detection, plus how far the two anchors have moved
/// apart along the normal since.
fn separation(a: &Body, b: Option<&Body>, c: &Contact) -> f32 {
    let moved_a = a.position + a.orientation * c.anchor - c.world_point;
    let moved_b = b.map_or(Vec3::ZERO, |b| b.position + b.orientation * c.other_anchor - c.other_point);
    c.separation + (moved_a - moved_b).dot(c.normal)
}
```

`push` applies `+P` to the body and `-P` to the other:

```rust
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
```

`solve`, `solve_friction`, `warm_start` and the restitution sweep take `(a, b, c, ...)` and use these helpers. Their arithmetic is otherwise unchanged: the world is the case where `b` is `None` and every term of its is zero.

**Do not change any constant.** If a gate fails, the restructure is wrong.

- [ ] **Step 4: Run to verify they pass**

Run: `cargo test --workspace > t.txt 2>&1; echo $?`
Expected: `0`, with no warnings. **Every milestone 2 and 3a gate must still pass**: they are the same code path with `b = None`.

If `a_stack_of_three_stands_still` fails, the likely causes, in order: contacts solved in an order that starves the bottom body (sort by key, and keep the scene's contact order stable); the separation update missing the other body's motion; warm starting seeded from the wrong body's cache.

- [ ] **Step 5: Break checks**

1. In `push`, apply `+P` to both sides. `a_cube_knocks_another_along` must FAIL on momentum.
2. In `separation`, drop the other body's term. `a_stack_of_three_stands_still` must FAIL.
3. In `effective_mass`, drop the other body's term. `a_cube_knocks_another_along` must FAIL, because the hitter over-pushes.
4. Skip pair detection entirely (return early from the pair loop). `taking_the_lower_body_away_drops_the_upper` must FAIL.

Restore each and re-run.

- [ ] **Step 6: Commit**

```bash
git add crates
git commit -m "feat(core): solve every body and pair in one list, so bodies collide" -m "Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>"
```

---

### Task 4: Measure, and the demo

**Files:**
- Modify: `crates/bevox_core/src/physics/solver.rs` (the timing test covers pairs)
- Modify: this plan (Measurements)

- [ ] **Step 1: Extend the timing test**

In `a_tick_is_timed`, add a third case: sixteen bodies in a pile, four stacks of four, settled for 300 ticks and then timed for 1000. Print it beside the falling and resting numbers.

```rust
        let stacks = || -> Vec<Body> {
            (0..16)
                .map(|i| {
                    placed(
                        cube(4, 4),
                        Vec3::new(
                            16.0 + (i % 4) as f32 * 12.0,
                            10.2 + (i / 4) as f32 * 4.0,
                            32.0,
                        ),
                        Quat::IDENTITY,
                    )
                })
                .collect()
        };
        let mut piled = stacks();
        time(&mut piled, 300);
        let pile = median(time(&mut piled, 1000));
```

- [ ] **Step 2: Run it**

Run: `cargo test --release -p bevox_core a_tick_is_timed -- --ignored --nocapture --test-threads=1 > m.txt 2>&1; echo $?`

- [ ] **Step 3: Record**

Add a Measurements section to this plan: the printed line verbatim, the date and the CPU, and the comparison with 3a's `0.0071` falling and `0.8394` resting, stating plainly that those came from another process. Say what the pair cost is per pair, and whether the `MAX_BODIES = 16` cap is still the binding limit or CPU time is.

- [ ] **Step 4: Commit**

```bash
git add crates docs
git commit -m "perf(core): measure a tick with bodies piled on each other" -m "Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>"
```

- [ ] **Step 5: Hand over**

Tell Flori what to try: `cargo run -p bevox`, then `F` several times in the same place to stack cubes, and once at a sliding angle to knock a stack over. Do not merge or push.

---

## Milestone check

A cube thrown at another knocks it along and stops, momentum conserved. Three cubes stack and stay still over ten thousand ticks. Removing a body from under another drops it. Every milestone 2 and 3a gate still passes, because the world is the same code path with no second body.

## What this plan deliberately does not do

- **No sleeping and no islands.** The measurement in Task 4 is what would justify them.
- **No rolling resistance, no spinning friction.**
- **No fracture.** Dwyer has it; it needs a damage model this project has not specified.
- **No joints.** That is milestone 5, and it is what the mouse grab is built on.
