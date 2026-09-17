# Friction and Restitution Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking. **Flori prefers inline execution for this project.**

**Goal:** A body slides to a stop on stone, keeps sliding on ice, and bounces on rubber, with friction and restitution taken per voxel from the materials that touch.

**Architecture:** Detection already knows which two voxels touch, so it reads their materials and combines them into a friction and a restitution coefficient per contact. The solver gains a friction impulse along two tangents, clamped to the Coulomb cone, and a restitution pass after the substeps that restores a fraction of the speed the body arrived with. Warm starting carries the tangent impulses too.

**Tech Stack:** Rust stable 1.96, Bevy 0.19.1, glam 0.32. No new dependencies.

**Spec:** `docs/superpowers/specs/2026-09-15-bevox-rigid-bodies-design.md`. This is the first half of its milestone 3; body against body is the second half and is **not** in this plan.

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
- Every correctness gate is proven by a deliberate break: break the guarded code, watch the gate FAIL, restore, watch it pass.

## What exists after milestone 2

- `physics::contact::detect(body, tree, field, margin) -> Vec<Contact>`, where `Contact` carries `key: (UVec3, IVec3)`, `normal`, `separation`, `anchor` and `world_point`.
- `physics::solver::step(bodies, tree, field, gravity, dt)`, a TGS tick: detect once, then `SUBSTEPS` substeps of gravity, warm start, biased solve, integrate, relax.
- `Body::warm: HashMap<ContactKey, f32>`, the accumulated normal impulse per contact.
- `Material { color, density }`, with `DEFAULT_DENSITY`.
- Constants in `physics`: `GRAVITY`, `SUBSTEPS`, `SLOP`, `BIAS`, `MAX_PUSH`, `MAX_TRAVEL`, `BASE_MARGIN`, `VOXEL_METRES`.

## Facts that are not obvious from the code

**Coefficients are per contact, not per body.** Dwyer's devlog #26 makes friction and restitution per voxel, so one body slides differently depending on which of its voxels is touching. Detection already has both voxel coordinates, so it looks up both materials and stores the combined coefficients on the `Contact`.

**Tangent directions must be reproducible.** A friction impulse is warm started like a normal impulse, so the two tangents a contact used last tick must be the same two this tick. They are derived from the normal alone, by `Vec3::any_orthonormal_pair`, never from the velocity, which changes.

**Friction is clamped as a vector, not per axis.** Clamping each tangent separately would let the total reach `sqrt(2)` times the Coulomb limit and make a body slide diagonally faster than straight.

**Restitution runs after the substeps, not inside them.** A bounce needs the speed the body arrived with, which the substeps have already destroyed. The solver records the approach speed per contact when it prepares them, and a final pass adds the impulse that turns `v` into `-e * approach`. Box2D v3 does the same, and for the same reason.

**A resting body must not bounce.** With no threshold, gravity's per-substep velocity becomes a small approach speed and a bouncy body buzzes forever. Contacts approaching slower than `RESTITUTION_THRESHOLD` get no restitution. This is why milestone 2's resting gate is re-run on a bouncy material in this plan.

**Friction must not break resting.** The resting gate from milestone 2 stays as it is, and it now runs with friction active. If it fails, friction is wrong, not the gate.

## File Structure

- `crates/bevox_core/src/material.rs`: `friction` and `restitution` columns, and the combination rules.
- `crates/bevox_core/src/physics/mod.rs`: `RESTITUTION_THRESHOLD`; fixtures gain materials with friction and bounce.
- `crates/bevox_core/src/physics/contact.rs`: `Contact` carries the combined coefficients.
- `crates/bevox_core/src/body.rs`: `warm` holds a normal and a tangent impulse.
- `crates/bevox_core/src/physics/solver.rs`: friction impulses and the restitution pass.
- `crates/bevox/src/main.rs`: an ice strip and a rubber patch in the demo scene.
- `crates/bevox_core/src/vox.rs`, `crates/bevox_core/examples/render_reference.rs`, `crates/bevox_render/tests/common/mod.rs`: `Material` literals gain the two columns.

---

### Task 1: Material columns and the combination rules

**Files:**
- Modify: `crates/bevox_core/src/material.rs`
- Modify: `crates/bevox_core/src/vox.rs`, `crates/bevox_core/examples/render_reference.rs`, `crates/bevox_render/tests/common/mod.rs`, `crates/bevox/src/main.rs`
- Modify: `crates/bevox_core/src/physics/mod.rs` (fixtures)

**Interfaces:**
- Produces:
  - `Material { pub color: [u8; 4], pub density: u16, pub friction: u8, pub restitution: u8 }`
  - `pub const DEFAULT_FRICTION: u8 = 60;` and `pub const DEFAULT_RESTITUTION: u8 = 5;`
  - `pub fn combine_friction(a: u8, b: u8) -> f32`
  - `pub fn combine_restitution(a: u8, b: u8) -> f32`

- [ ] **Step 1: Write the failing tests**

In `material.rs`'s test module:

```rust
    /// Both columns are hundredths, so 60 is a coefficient of 0.6.
    #[test]
    fn a_material_keeps_its_friction_and_bounce() {
        let mut table = MaterialTable::new();
        let id = table
            .push(Material { color: [1, 2, 3, 255], density: 900, friction: 5, restitution: 80 })
            .unwrap();
        assert_eq!(table.get(id).friction, 5);
        assert_eq!(table.get(id).restitution, 80);
    }

    /// Friction combines as the geometric mean, so ice against stone is
    /// slippery rather than the average of the two. Restitution takes the
    /// larger, so a bouncy ball bounces off a dead floor.
    #[test]
    fn coefficients_combine_the_way_two_surfaces_do() {
        assert!((combine_friction(100, 100) - 1.0).abs() < 1e-6);
        assert!((combine_friction(4, 100) - 0.2).abs() < 1e-6, "ice against stone");
        assert_eq!(combine_friction(0, 100), 0.0, "frictionless against anything is frictionless");
        assert!((combine_restitution(80, 5) - 0.8).abs() < 1e-6);
        assert_eq!(combine_restitution(0, 0), 0.0);
    }
```

- [ ] **Step 2: Run to verify they fail**

Run: `cargo test -p bevox_core material > b.txt 2>&1; echo $?`
Expected: non-zero; the fields and functions do not exist.

- [ ] **Step 3: Implement**

In `material.rs`, extend the struct and its doc, and add the rules:

```rust
/// A material's properties: how it is drawn, how heavy it is, and how it
/// behaves on contact.
///
/// The physics columns are hundredths, and integers so `Material` stays `Eq`:
/// `friction` 60 is a coefficient of 0.6, `restitution` 80 is 0.8. Friction may
/// exceed 1; restitution above 1 would add energy on every bounce, so it is
/// clamped where it is used.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub struct Material {
    pub color: [u8; 4],
    pub density: u16,
    pub friction: u8,
    pub restitution: u8,
}

/// What a material's friction is when its source says nothing: like dry stone.
pub const DEFAULT_FRICTION: u8 = 60;

/// Barely bouncy, which is what most solids are.
pub const DEFAULT_RESTITUTION: u8 = 5;

/// The friction between two surfaces: the geometric mean, so the slipperier
/// one dominates and anything against a frictionless surface slides free.
pub fn combine_friction(a: u8, b: u8) -> f32 {
    (a as f32 * b as f32).sqrt() / 100.0
}

/// The bounce between two surfaces: the larger, so one bouncy surface is enough.
/// Clamped to 1, because more would add energy with every bounce.
pub fn combine_restitution(a: u8, b: u8) -> f32 {
    (a.max(b) as f32 / 100.0).min(1.0)
}
```

Update `MaterialTable::new`'s empty slot to `Material { color: [0, 0, 0, 0], density: 0, friction: 0, restitution: 0 }`.

Add `friction: DEFAULT_FRICTION, restitution: DEFAULT_RESTITUTION` to every other `Material` literal in `material.rs`'s tests, `vox.rs`, `examples/render_reference.rs` and `tests/common/mod.rs`, importing the constants where needed.

In `main.rs`'s `demo_scene`, give the palette real values and two new materials:

```rust
    materials
        .push(Material { color: [140, 140, 150, 255], density: 2600, friction: 60, restitution: 5 })
        .unwrap(); // 1: stone
    materials
        .push(Material { color: [180, 90, 70, 255], density: 1900, friction: 70, restitution: 5 })
        .unwrap(); // 2: brick
    materials
        .push(Material { color: [170, 210, 235, 255], density: 900, friction: 4, restitution: 10 })
        .unwrap(); // 3: ice
    materials
        .push(Material { color: [40, 40, 45, 255], density: 1100, friction: 80, restitution: 80 })
        .unwrap(); // 4: rubber
```

In `physics/mod.rs`'s fixtures, give material 1 ordinary friction and no bounce, material 2 the same, and add two more:

```rust
    /// Material 1 weighs 1000 and grips; 2 weighs 3000 and grips; 3 is
    /// frictionless ice; 4 is bouncy.
    pub fn materials() -> MaterialTable {
        let mut table = MaterialTable::new();
        let mut push = |color, density, friction, restitution| {
            table.push(Material { color, density, friction, restitution }).unwrap()
        };
        push([200, 200, 200, 255], 1000, 60, 0);
        push([90, 90, 90, 255], 3000, 60, 0);
        push([170, 210, 235, 255], 1000, 0, 0);
        push([40, 40, 45, 255], 1000, 60, 80);
        table
    }
```

Check the closure borrows: if `push` fights the borrow checker, write the four `table.push(...)` calls out instead.

The fixtures' `cube` and `slab` take material 1 today. Give both a material parameter:

```rust
    /// A solid `n`-cubed block of `material` at the origin of an `extent` volume.
    pub fn cube_of(n: u32, extent: u32, material: MaterialId) -> Contree { ... }

    /// A world whose layers `ys` are solid `material` across the whole volume.
    pub fn slab_of(extent: u32, ys: std::ops::Range<u32>, material: MaterialId) -> Contree { ... }
```

Keep `cube(n, extent)` and `slab(extent, ys)` as wrappers passing `MaterialId(1)`, so milestone 2's tests are untouched.

- [ ] **Step 4: Run to verify they pass**

Run: `cargo test --workspace > t.txt 2>&1; echo $?`
Expected: `0`, with no warnings.

- [ ] **Step 5: Break check**

Change `combine_friction` to `(a as f32 + b as f32) / 200.0`, an average. Run `cargo test -p bevox_core coefficients_combine > b.txt 2>&1; echo $?`: expected non-zero, and the ice case FAILS (0.52 instead of 0.2). Restore and re-run: `0`.

- [ ] **Step 6: Commit**

```bash
git add crates
git commit -m "feat(core): per-material friction and restitution" -m "Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>"
```

---

### Task 2: Contacts carry the coefficients of the two voxels that touch

**Files:**
- Modify: `crates/bevox_core/src/physics/contact.rs`

**Interfaces:**
- Consumes: `combine_friction`, `combine_restitution`, `MaterialTable` (Task 1)
- Produces:
  - `Contact` gains `pub friction: f32` and `pub restitution: f32`
  - `detect(body, tree, field, materials: &MaterialTable, margin: f32) -> Vec<Contact>` — note the new parameter, before `margin`

- [ ] **Step 1: Write the failing test**

In `contact.rs`'s test module:

```rust
    /// The coefficients come from the two voxels that touch, not from the body:
    /// a cube resting half on ice and half on stone drags on one side only.
    #[test]
    fn a_contact_takes_the_coefficients_of_both_voxels() {
        let materials = crate::physics::fixtures::materials();
        // Floor of stone (1), with the far half ice (3).
        let mut voxels = Vec::new();
        for z in 0..64 {
            for y in 0..8 {
                for x in 0..64 {
                    let m = if x >= 32 { MaterialId(3) } else { MaterialId(1) };
                    voxels.push((UVec3::new(x, y, z), m));
                }
            }
        }
        let world = Contree::from_voxels(64, &voxels);
        let field = DistanceField::build(&world);
        // A bouncy cube (4) straddling the seam.
        let body = placed(
            crate::physics::fixtures::cube_of(4, 4, MaterialId(4)),
            Vec3::new(32.0, 10.0, 32.0),
            Quat::IDENTITY,
        );
        let contacts = detect(&body, &world, &field, &materials, 0.1);
        assert_eq!(contacts.len(), 4);

        let on_ice: Vec<_> = contacts.iter().filter(|c| c.key.1.x >= 32).collect();
        let on_stone: Vec<_> = contacts.iter().filter(|c| c.key.1.x < 32).collect();
        assert_eq!(on_ice.len(), 2, "the cube did not straddle the seam");
        assert_eq!(on_stone.len(), 2);
        for c in on_ice {
            assert_eq!(c.friction, 0.0, "ice is frictionless");
            assert!((c.restitution - 0.8).abs() < 1e-6, "the bouncy cube bounces on ice");
        }
        for c in on_stone {
            assert!((c.friction - 0.6).abs() < 1e-6, "stone against a gripping cube");
            assert!((c.restitution - 0.8).abs() < 1e-6);
        }
    }
```

- [ ] **Step 2: Run to verify it fails**

Run: `cargo test -p bevox_core contact > b.txt 2>&1; echo $?`
Expected: non-zero; `detect` takes four arguments and `Contact` has no `friction`.

- [ ] **Step 3: Implement**

In `contact.rs`:

1. Add to `Contact`:

   ```rust
       /// Coulomb friction between the two voxels that touch.
       pub friction: f32,
       /// How much of the approach speed a bounce keeps.
       pub restitution: f32,
   ```

2. Give `keep` two more parameters, `friction: f32` and `restitution: f32`, and store them.

3. Add the parameter `materials: &MaterialTable` to `detect`, before `margin`, and a helper:

   ```rust
   /// The coefficients for a touch between a body voxel and a world voxel.
   fn coefficients(
       materials: &MaterialTable,
       body: &Body,
       u: UVec3,
       tree: &Contree,
       k: IVec3,
   ) -> (f32, f32) {
       let a = materials.get(body.volume.get(u));
       let b = materials.get(tree.get(k.as_uvec3()));
       (combine_friction(a.friction, b.friction), combine_restitution(a.restitution, b.restitution))
   }
   ```

   `k` is always inside the world where this is called, because every caller has already tested it with `world_solid`.

4. At each of the three `keep` calls, compute `let (friction, restitution) = coefficients(materials, body, u, tree, k);` and pass them. In the world-corner loop the body voxel is `u.as_uvec3()`.

5. Import `use crate::material::{MaterialTable, combine_friction, combine_restitution};`.

Update milestone 2's `detect` calls in `contact.rs`'s and `solver.rs`'s tests to pass `&materials()` — and `solver::step` gains the same parameter in Task 3, so leave `solver.rs`'s call site for now by passing `&crate::material::MaterialTable::new()` there temporarily. **Do not leave that temporary in place:** Task 3 replaces it.

- [ ] **Step 4: Run to verify it passes**

Run: `cargo test -p bevox_core > t.txt 2>&1; echo $?`
Expected: `0`.

- [ ] **Step 5: Break check**

In `coefficients`, swap the body lookup for the world's: `let a = materials.get(tree.get(k.as_uvec3()));`. Run `cargo test -p bevox_core a_contact_takes_the_coefficients > b.txt 2>&1; echo $?`: expected non-zero, because the ice contacts then report the floor's own bounce rather than the cube's. Restore and re-run: `0`.

- [ ] **Step 6: Commit**

```bash
git add crates
git commit -m "feat(core): contacts carry the friction and bounce of both voxels" -m "Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>"
```

---

### Task 3: Friction impulses

**Files:**
- Modify: `crates/bevox_core/src/body.rs` (`warm` holds two impulses)
- Modify: `crates/bevox_core/src/physics/solver.rs`

**Interfaces:**
- Consumes: `Contact::friction` (Task 2)
- Produces:
  - `physics::solver::ContactImpulse { pub normal: f32, pub tangent: Vec2 }`, `Default`
  - `Body::warm: HashMap<ContactKey, ContactImpulse>`
  - `step(bodies, tree, field, materials: &MaterialTable, gravity, dt) -> bool` — note the new parameter

- [ ] **Step 1: Write the failing tests**

In `solver.rs`'s test module:

```rust
    /// Sliding on stone stops in the distance Coulomb friction predicts,
    /// v^2 / (2 mu g); on ice it keeps going.
    #[test]
    fn a_sliding_body_stops_on_stone_and_slides_on_ice() {
        let materials = materials();
        // Near the world's edge it starts from, so 56 voxels of sliding on ice
        // still land inside the floor rather than off it.
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
```

Update every existing solver test to pass `&materials()` to `run` and `step`; `run` gains the parameter too.

- [ ] **Step 2: Run to verify they fail**

Run: `cargo test -p bevox_core solver > b.txt 2>&1; echo $?`
Expected: non-zero; `step` takes five arguments and `slab_of` is not imported.

- [ ] **Step 3: Implement**

In `body.rs`, change the field to `pub warm: HashMap<ContactKey, crate::physics::solver::ContactImpulse>`, and keep `HashMap::new()` in `new`.

In `solver.rs`:

1. The impulse record:

   ```rust
   /// What a contact carried last tick, for warm starting.
   #[derive(Clone, Copy, Debug, Default, PartialEq)]
   pub struct ContactImpulse {
       pub normal: f32,
       /// Along the contact's two tangents, in their order.
       pub tangent: Vec2,
   }
   ```

   Import `glam::Vec2`.

2. Tangents from the normal alone, so warm starting stays meaningful:

   ```rust
   /// The contact's two tangent directions. Derived from the normal only: a
   /// basis that depended on velocity would rotate between ticks and make last
   /// tick's stored tangent impulse meaningless.
   fn tangents(normal: Vec3) -> (Vec3, Vec3) {
       normal.any_orthonormal_pair()
   }
   ```

3. `step` and `step_body` take `materials: &MaterialTable`, and `detect` is called with it.

4. Warm start applies both impulses, so `push` takes a vector impulse:

   ```rust
   /// Applies an impulse at the contact point.
   fn push(body: &mut Body, inv_mass: f32, c: &Contact, impulse: Vec3) {
       body.velocity += impulse * inv_mass;
       body.angular_momentum += lever(body, c).cross(impulse);
   }
   ```

   The normal solve then calls `push(body, inv_mass, c, c.normal * delta)`, and warm start applies
   `c.normal * stored.normal + t1 * stored.tangent.x + t2 * stored.tangent.y`.

5. The friction solve, run after the normal solve in the same loop, for both the biased and the relax pass:

   ```rust
   /// Coulomb friction along the contact's two tangents.
   ///
   /// The accumulated tangent impulse is clamped as a vector rather than per
   /// axis: clamping each axis alone would let the total reach sqrt(2) times the
   /// limit, and a body pushed diagonally would slide further than one pushed
   /// along an axis.
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
   ```

6. In `step_body`, replace the `Vec<f32>` of impulses with `Vec<ContactImpulse>`, seeded from `body.warm`, and call `solve_friction` after each contact's normal solve, in both the biased and the relax loop. Store the whole record back into `body.warm` at the end.

- [ ] **Step 4: Run to verify they pass**

Run: `cargo test --workspace > t.txt 2>&1; echo $?`
Expected: `0`, with no warnings. Milestone 2's `a_body_at_rest_stays_at_rest` and `a_cube_on_its_edge_tips_onto_a_face` must still pass; friction changes how the tipping cube's edge slides, so if the tipping gate fails, check the friction clamp before touching the gate.

- [ ] **Step 5: Break checks**

1. Clamp per axis instead: replace the vector clamp with `total = total.clamp(Vec2::splat(-limit), Vec2::splat(limit));`. `friction_is_the_same_in_every_direction` must FAIL.
2. Use the velocity for the tangent basis: `let t1 = v.reject_from(c.normal).normalize_or(Vec3::X);`. Warm starting then means nothing; `a_sliding_body_stops_on_stone_and_slides_on_ice` or `friction_leaves_a_resting_body_alone` must FAIL. If neither does, say so and leave the deterministic basis in place; it is still required for the warm-start contract.
3. Drop the `c.friction <= 0.0` guard and use `limit = 1.0 * impulse.normal`. The ice half of the sliding test must FAIL.

Restore each and re-run.

- [ ] **Step 6: Commit**

```bash
git add crates
git commit -m "feat(core): Coulomb friction at contacts" -m "Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>"
```

---

### Task 4: Restitution

**Files:**
- Modify: `crates/bevox_core/src/physics/mod.rs` (`RESTITUTION_THRESHOLD`)
- Modify: `crates/bevox_core/src/physics/solver.rs`

**Interfaces:**
- Consumes: `Contact::restitution` (Task 2), `ContactImpulse` (Task 3)
- Produces: `pub const RESTITUTION_THRESHOLD: f32 = 10.0;` in `physics`

- [ ] **Step 1: Write the failing tests**

In `solver.rs`'s test module:

```rust
    /// A bounce keeps `e` of the approach speed, so the body returns to about
    /// `e^2` of its drop height. A dead floor keeps nothing.
    #[test]
    fn a_bouncy_body_returns_to_a_quarter_of_its_height() {
        let materials = materials();
        let apex = |floor: MaterialId, body_material: MaterialId| -> f32 {
            let world = slab_of(64, 0..8, floor);
            let field = DistanceField::build(&world);
            let mut bodies =
                vec![placed(cube_of(4, 4, body_material), Vec3::new(32.0, 14.0, 32.0), Quat::IDENTITY)];
            let mut top: f32 = 0.0;
            let mut landed = false;
            for _ in 0..400 {
                step(&mut bodies, &world, &field, &materials, GRAVITY, DT);
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
            step(&mut bodies, &world, &field, &materials, GRAVITY, DT);
            low = low.min(bodies[0].position.y);
            high = high.max(bodies[0].position.y);
        }
        assert!(high - low < 1e-3, "a bouncy body buzzed between {low} and {high}");
    }
```

- [ ] **Step 2: Run to verify they fail**

Run: `cargo test -p bevox_core solver > b.txt 2>&1; echo $?`
Expected: non-zero. `a_bouncy_body_returns_to_a_quarter_of_its_height` fails because nothing bounces yet.

- [ ] **Step 3: Implement**

In `physics/mod.rs`:

```rust
/// Contacts approaching slower than this, in voxels per second, do not bounce.
///
/// Without it, the speed gravity adds in one substep is enough to make a bouncy
/// body buzz on the floor forever instead of settling.
pub const RESTITUTION_THRESHOLD: f32 = 10.0;
```

In `solver.rs`, inside `step_body`:

1. Before the substep loop, record each contact's approach speed:

   ```rust
   // The speed the body arrives with, which the substeps are about to destroy.
   // A bounce is written in terms of it, so it is recorded here.
   let approach: Vec<f32> = contacts
       .iter()
       .map(|c| {
           let r = lever(body, c);
           let omega = world_inverse_inertia(body) * body.angular_momentum;
           (body.velocity + omega.cross(r)).dot(c.normal)
       })
       .collect();
   ```

2. After the substep loop, the restitution pass:

   ```rust
   // Restitution last, after the substeps have removed the approach speed.
   // Solving it inside them would fight the push-out bias.
   let inv_inertia = world_inverse_inertia(body);
   for ((c, impulse), &approach) in contacts.iter().zip(impulses.iter_mut()).zip(&approach) {
       if c.restitution <= 0.0 || approach > -RESTITUTION_THRESHOLD || impulse.normal == 0.0 {
           continue;
       }
       let r = lever(body, c);
       let omega = inv_inertia * body.angular_momentum;
       let vn = (body.velocity + omega.cross(r)).dot(c.normal);
       let target = -c.restitution * approach;
       if vn >= target {
           continue;
       }
       let k = inv_mass + (inv_inertia * r.cross(c.normal)).cross(r).dot(c.normal);
       let total = (impulse.normal + (target - vn) / k).max(0.0);
       let delta = total - impulse.normal;
       impulse.normal = total;
       push(body, inv_mass, c, c.normal * delta);
   }
   ```

   `impulse.normal == 0.0` skips a contact that never carried load: a speculative one the body never reached.

- [ ] **Step 4: Run to verify they pass**

Run: `cargo test --workspace > t.txt 2>&1; echo $?`
Expected: `0`, with no warnings.

**If `a_bouncy_body_returns_to_a_quarter_of_its_height` lands outside its tolerance,** check the drop height against the apex formula before touching the tolerance: a 4-voxel drop with `e = 0.8` gives `0.64 * 4 = 2.56`. **If `a_bouncy_body_still_settles` fails,** raise `RESTITUTION_THRESHOLD` and record every value tried in the Measurements section.

- [ ] **Step 5: Break checks**

1. Remove the threshold test (`approach > -RESTITUTION_THRESHOLD`). `a_bouncy_body_still_settles` must FAIL.
2. Move the restitution pass inside the substep loop, at the end of the body. `a_bouncy_body_returns_to_a_quarter_of_its_height` must FAIL or the body must gain height; report what happens either way.
3. Use the current normal velocity instead of the recorded approach (`let target = -c.restitution * vn;`). `a_bouncy_body_returns_to_a_quarter_of_its_height` must FAIL, because by then the substeps have removed the approach.

Restore each and re-run.

- [ ] **Step 6: Commit**

```bash
git add crates
git commit -m "feat(core): restitution, so a bouncy body bounces and still settles" -m "Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>"
```

---

### Task 5: The demo, and the numbers

**Files:**
- Modify: `crates/bevox/src/main.rs`
- Modify: this plan (Measurements)

**Interfaces:**
- Consumes: `step` with materials (Task 3)

- [ ] **Step 1: Write the failing test**

In `main.rs`'s tests:

```rust
    /// The demo floor has an ice strip and a rubber patch, so dropping bodies
    /// shows friction and bounce without editing anything.
    #[test]
    fn the_demo_scene_has_something_slippery_and_something_bouncy() {
        let (tree, materials) = demo_scene();
        let has = |m: MaterialId| {
            (0..64).any(|x| (0..64).any(|z| tree.get(UVec3::new(x, 5, z)) == m))
        };
        assert!(has(MaterialId(3)), "no ice on the floor");
        assert!(has(MaterialId(4)), "no rubber on the floor");
        assert_eq!(materials.get(MaterialId(3)).friction, 4);
        assert_eq!(materials.get(MaterialId(4)).restitution, 80);
    }
```

- [ ] **Step 2: Run to verify it fails**

Run: `cargo test -p bevox > b.txt 2>&1; echo $?`
Expected: non-zero; the floor is all stone.

- [ ] **Step 3: Implement**

In `demo_scene`, after the floor loop and before the column, lay the two surfaces into the floor's top layer:

```rust
    // A slippery strip and a bouncy patch, so a dropped body shows friction and
    // restitution without any editing.
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
```

In `physics_system`, pass the materials: `step(&mut scene.bodies, &scene.tree, &scene.field, &scene.materials, GRAVITY, time.delta_secs())`. The borrow splits because `scene` is already destructured as `&mut *scene`.

- [ ] **Step 4: Run to verify it passes**

Run: `cargo test --workspace > t.txt 2>&1; echo $?`
Expected: `0`, with no warnings.

- [ ] **Step 5: Break check**

Change the ice strip's material to `MaterialId(1)`. Run `cargo test -p bevox the_demo_scene_has > b.txt 2>&1; echo $?`: expected non-zero, FAILS. Restore and re-run: `0`.

- [ ] **Step 6: Measure**

Run: `cargo test --release -p bevox_core a_tick_is_timed -- --ignored --nocapture --test-threads=1 > m.txt 2>&1; echo $?`

Add a `## Measurements` section to this plan with the printed line verbatim, the date and the CPU, and the comparison against milestone 2's 0.0052 ms falling / 0.8074 ms resting for 16 bodies. Those came from a different process, so state that plainly: it says whether friction and restitution changed the cost by a lot, not by how much. Record any tuning of `RESTITUTION_THRESHOLD`.

- [ ] **Step 7: Commit**

```bash
git add crates docs
git commit -m "feat(app): an ice strip and a bouncy patch in the demo scene" -m "Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>"
```

- [ ] **Step 8: Hand over**

Tell Flori what to try: `cargo run -p bevox`, then `F` to drop cubes. A cube pushed onto the ice strip keeps going; one dropped on the rubber patch bounces. Do not merge or push.

---

## Milestone check

A body slides to a stop on stone in the distance Coulomb friction predicts, keeps sliding on ice, bounces to about `e^2` of its drop height on rubber, and still settles rather than buzzing. Every gate is proven by a deliberate break.

## What this plan deliberately does not do

- **No body against body.** That is milestone 3's second half: it restructures the solver so two moving bodies share a contact, and it needs body identity in the warm-start cache.
- **No rolling resistance and no spinning friction.** A ball would roll forever; nothing in the scene is a ball yet.
- **No static-versus-dynamic friction split.** One coefficient per pair, as Dwyer has.
- **No sleeping.** Milestone 2 measured resting bodies at 0.05 ms each; that is the number that would justify it.
