# Rigid-Body Physics Milestone 2 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking. **Flori prefers inline execution for this project.**

**Goal:** A voxel body dropped into the scene lands on the static voxel world, tumbles if it lands off balance, and comes to rest without jitter.

**Architecture:** The simulation is a pure `bevox_core::physics` module with no Bevy, split into four files:
- `mass`: density-weighted mass properties;
- `classify`: corner, edge, face and interior voxels;
- `contact`: rounded-voxel pair tests and detection against the world;
- `solver`: a temporal Gauss-Seidel tick.

`Body` carries its own physics state. The app calls `solver::step` once per `FixedUpdate` tick and adds an `F` key that drops a body.

**Tech Stack:** Rust stable 1.96, Bevy 0.19.1, glam 0.32. No new dependencies.

**Spec:** `docs/superpowers/specs/2026-09-15-bevox-rigid-bodies-design.md` (revised at 695b30f). Read its Collision detection, Integration and solving, and Provenance sections before starting.

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

## Facts that are not obvious from the code

**`Body::new` must keep placing a volume exactly as it does today.** About 25 call sites in the render crate's tests build bodies with `Body::new(volume, origin, orientation)` and compare GPU output against CPU marches. `position` becomes the centre of mass in world space, but `com` starts at zero and only `recompute` moves it. `world_from_local` is written as `from_rotation_translation(q, position - q * com)`, so a body that was never recomputed has exactly the old matrix, and nothing in the render crate changes.

**Recompute moves `position`, not the voxels.** When the centre of mass moves by `Δ` in local space, `position` moves by `orientation * Δ`, and every voxel stays where it was in the world. Brush-editing bodies in milestone 4 depends on this.

**The contact lookup must reach further than the spec's "2x2x2".** Voxel `k` is centred at `k + 0.5`. A corner sphere resting exactly on a floor has its centre exactly one voxel above the floor voxel's centre. That puts the pair on the 2x2x2 lookup's tie boundary, so the contact would appear and vanish from tick to tick, and warm starting would lose it. A speculative contact separated by more than zero is never seen at all. Detection therefore looks at every voxel within `reach = ceil(margin + 0.5)` of the voxel containing the centre: 3x3x3 at rest, 5x5x5 at the speed cap. Task 4 updates the spec.

**Each touch is owned by one voxel.** A sphere over a floor is within reach of up to 25 floor voxels. The pair test reports it only for the face voxel under the sphere's centre, using half-open ranges `[-0.5, 0.5)`, and only for the edge voxel alongside it. Otherwise every contact would be counted many times.

**Contacts are sorted by key before solving.** `HashMap` iteration order is random per process, and sequential impulses depend on order, so an unsorted solver makes the resting gate non-reproducible.

**The tick is Box2D v3's soft step, and that part is not from Dwyer.** Each substep runs gravity, warm start, a biased solve, integrate, then an unbiased "relax" solve. Dwyer's devlog #26 gives detect-once-then-substep and warm starting, but not the relax pass. The relax pass removes the velocity the bias added, so push-out does not become bounce. This follows Erin Catto's published soft-step design and is listed in the spec's Provenance as a fill-in.

**The accumulated impulse is per substep.** Warm starting applies it again every substep, so at rest the sum of a body's accumulated normal impulses is `mass * |g| * h`, and one tick's total is `SUBSTEPS` times that. The resting gate checks exactly this.

**Physics runs in `FixedUpdate`, which Bevy runs before `Update` every frame.** A physics tick that only moves bodies needs nothing more: the body table is re-uploaded every frame. A tick that removes a body bumps `generation`, and that is still ahead of `build_gpu_scene` in `Update`, as `VoxelScene`'s ordering rule requires.

## File Structure

- `crates/bevox_core/src/material.rs`: `Material` gains `density`.
- `crates/bevox_core/src/contree.rs`: `Contree::voxels`, the inverse of `from_voxels`.
- `crates/bevox_core/src/physics/mod.rs` (**new**): constants, module declarations, test fixtures.
- `crates/bevox_core/src/physics/mass.rs` (**new**): `MassProperties`, `mass_properties`.
- `crates/bevox_core/src/physics/classify.rs` (**new**): `Shape`, `classify`, `solid_at`, `Features`, `features`.
- `crates/bevox_core/src/physics/contact.rs` (**new**): `Contact`, `ContactKey`, pair tests, `world_box`, `field_clears`, `detect`.
- `crates/bevox_core/src/physics/solver.rs` (**new**): `step`.
- `crates/bevox_core/src/body.rs`: physics fields, centre-of-mass pivot, `recompute`.
- `crates/bevox_core/src/lib.rs`: `pub mod physics;`.
- `crates/bevox_core/src/vox.rs`, `crates/bevox_core/examples/render_reference.rs`, `crates/bevox_render/tests/common/mod.rs`: `Material` literals gain a density.
- `crates/bevox/src/main.rs`: physics system, the `F` drop, demo densities; `spin_bodies` removed.
- `docs/superpowers/specs/2026-09-15-bevox-rigid-bodies-design.md`: lookup reach and relax pass.

---

### Task 1: Material density and listing a tree's voxels

**Files:**
- Modify: `crates/bevox_core/src/material.rs`
- Modify: `crates/bevox_core/src/contree.rs`
- Modify: `crates/bevox_core/src/vox.rs:121`, `crates/bevox_core/src/vox.rs:162`
- Modify: `crates/bevox_core/examples/render_reference.rs:17-18`
- Modify: `crates/bevox_render/tests/common/mod.rs:38-39`
- Modify: `crates/bevox/src/main.rs:271-272`

**Interfaces:**
- Produces:
  - `Material { pub color: [u8; 4], pub density: u16 }`
  - `pub const DEFAULT_DENSITY: u16 = 1000;` in `material.rs`
  - `Contree::voxels(&self) -> Vec<(UVec3, MaterialId)>`

- [ ] **Step 1: Write the failing tests**

In `material.rs`'s test module:

```rust
    /// Density rides along with colour: mass properties read it back by id.
    #[test]
    fn a_material_keeps_its_density() {
        let mut table = MaterialTable::new();
        let id = table.push(Material { color: [1, 2, 3, 255], density: 2600 }).unwrap();
        assert_eq!(table.get(id).density, 2600);
        assert_eq!(table.get(MaterialId::EMPTY).density, 0, "empty space must weigh nothing");
    }
```

In `contree.rs`'s test module:

```rust
    /// Listing a tree's voxels and building a tree from the list must agree,
    /// or mass properties and classification would see a different body from
    /// the one the renderer draws.
    #[test]
    fn voxels_round_trips_through_from_voxels() {
        let mut rng = crate::testing::XorShift64::new(0x5eed_0001);
        let mut input = std::collections::HashMap::new();
        for _ in 0..3000 {
            let p = UVec3::new(rng.next_below(64), rng.next_below(64), rng.next_below(64));
            input.insert(p, MaterialId(1 + rng.next_below(3) as u8));
        }
        // A solid block too, so uniform nodes are expanded rather than skipped.
        for z in 16..32 {
            for y in 16..32 {
                for x in 16..32 {
                    input.insert(UVec3::new(x, y, z), MaterialId(2));
                }
            }
        }
        let list: Vec<_> = input.iter().map(|(p, m)| (*p, *m)).collect();
        let tree = Contree::from_voxels(64, &list);

        let key = |(p, m): &(UVec3, MaterialId)| (p.z, p.y, p.x, m.0);
        let mut want: Vec<_> = list.iter().map(key).collect();
        let mut got: Vec<_> = tree.voxels().iter().map(key).collect();
        want.sort_unstable();
        got.sort_unstable();
        assert_eq!(got, want);
    }
```

- [ ] **Step 2: Run to verify they fail**

Run: `cargo test -p bevox_core material contree > build.txt 2>&1; echo $?`
Expected: non-zero. The compile fails: `density` is not a field, and `voxels` is not a method.

- [ ] **Step 3: Implement**

In `material.rs`, replace the `Material` doc comment and struct, and the empty slot in `new`:

```rust
/// A material's properties: how it is drawn and how heavy it is.
///
/// Density is relative: only ratios between voxels, and later against a
/// joint's force, are observable. `u16` rather than a float so `Material`
/// stays `Eq`. Friction and restitution arrive with milestone 3.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub struct Material {
    pub color: [u8; 4],
    pub density: u16,
}

/// The density a material gets when its source says nothing about mass, such
/// as a MagicaVoxel palette.
pub const DEFAULT_DENSITY: u16 = 1000;
```

```rust
    pub fn new() -> Self {
        Self { entries: vec![Material { color: [0, 0, 0, 0], density: 0 }] }
    }
```

Add `, density: DEFAULT_DENSITY` to every other `Material { color: ... }` literal:
- the tests in `material.rs`;
- both literals in `vox.rs`, with `use crate::material::DEFAULT_DENSITY;` or the full path;
- `examples/render_reference.rs`;
- `tests/common/mod.rs`, using `bevox_core::material::DEFAULT_DENSITY`.

In `main.rs`'s `demo_scene`, use real values:

```rust
    materials.push(Material { color: [140, 140, 150, 255], density: 2600 }).unwrap(); // 1: stone
    materials.push(Material { color: [180, 90, 70, 255], density: 1900 }).unwrap(); // 2: brick
```

In `contree.rs`, add to `impl Contree`, next to `from_voxels`:

```rust
    /// Every solid voxel and its material, in no particular order: the inverse
    /// of `from_voxels`.
    ///
    /// Uniform nodes are expanded voxel by voxel, so this is linear in the
    /// body's volume. That is fine for bodies, which are small; do not call it
    /// on the static world.
    pub fn voxels(&self) -> Vec<(UVec3, MaterialId)> {
        let mut out = Vec::new();
        self.collect_voxels(self.root, self.depth - 1, UVec3::ZERO, &mut out);
        out
    }

    fn collect_voxels(&self, node: Node, level: u32, origin: UVec3, out: &mut Vec<(UVec3, MaterialId)>) {
        if node.is_empty() {
            return;
        }
        if !node.is_subdivided() {
            let extent = level_extent(level);
            let m = node.material();
            for z in 0..extent {
                for y in 0..extent {
                    for x in 0..extent {
                        out.push((origin + UVec3::new(x, y, z), m));
                    }
                }
            }
            return;
        }
        for i in 0..CHILDREN {
            let Some(slot) = node.child_slot(i) else { continue };
            // Inverse of child_index: x + y * 4 + z * 16.
            let c = UVec3::new(i % 4, (i / 4) % 4, i / 16);
            if level == 0 {
                // A brick's children are voxels, in the voxel arena.
                out.push((origin + c, MaterialId(self.arena.voxel(slot))));
            } else {
                let step = level_extent(level - 1);
                self.collect_voxels(self.arena.node(slot), level - 1, origin + c * step, out);
            }
        }
    }
```

Check what `contree.rs` already imports (`Node`, `CHILDREN`, `level_extent`, `MaterialId`), and add only what is missing.

- [ ] **Step 4: Run to verify they pass**

Run: `cargo test --workspace > t1.txt 2>&1; echo $?`
Expected: `0`, with no warnings. Check with `grep -c "^warning" t1.txt`.

- [ ] **Step 5: Break check**

In `collect_voxels`, change `origin + c * step` to `origin + c` and run `cargo test -p bevox_core voxels_round_trips > b1.txt 2>&1; echo $?`. Expected: non-zero, the round-trip FAILS. Restore it and re-run: `0`.

- [ ] **Step 6: Commit**

```bash
git add crates
git commit -m "feat(core): material density, and listing a tree's voxels" -m "Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>"
```

---

### Task 2: Mass properties and the centre-of-mass pivot

**Files:**
- Create: `crates/bevox_core/src/physics/mod.rs`
- Create: `crates/bevox_core/src/physics/mass.rs`
- Modify: `crates/bevox_core/src/lib.rs`
- Modify: `crates/bevox_core/src/body.rs`

**Interfaces:**
- Consumes: `Contree::voxels`, `Material::density` (Task 1)
- Produces:
  - `physics::{VOXEL_METRES, GRAVITY, SUBSTEPS, SLOP, BIAS, MAX_PUSH, MAX_TRAVEL, BASE_MARGIN}`
  - `physics::mass::MassProperties { pub mass: f32, pub inverse_inertia: Mat3 }`, with `inverse_mass(&self) -> f32`
  - `physics::mass::mass_properties(&[(UVec3, MaterialId)], &MaterialTable) -> Option<(MassProperties, Vec3)>`
  - `Body` fields `com: Vec3`, `mass: MassProperties`, `velocity: Vec3`, `angular_momentum: Vec3`
  - `Body::recompute(&mut self, &MaterialTable) -> bool`
  - `#[cfg(test)] physics::fixtures::{materials, cube, placed}`

- [ ] **Step 1: Create the module with constants and fixtures**

`crates/bevox_core/src/physics/mod.rs`:

```rust
//! Rigid-body physics for voxel bodies, after Douglas Dwyer's engine: voxels
//! classified as corners, edges and faces; rounded-voxel contacts; a temporal
//! Gauss-Seidel solver. See `docs/superpowers/specs/2026-09-15-bevox-rigid-bodies-design.md`,
//! whose Provenance section separates what his devlogs show from what is
//! filled in here.
//!
//! Units are voxels and seconds.

pub mod mass;

use glam::Vec3;

/// How long one voxel is, in metres. A tuning knob: gravity is expressed in
/// voxels, so this sets how heavy a fall feels.
pub const VOXEL_METRES: f32 = 0.1;

/// Downward acceleration, in voxels per second squared.
pub const GRAVITY: Vec3 = Vec3::new(0.0, -9.81 / VOXEL_METRES, 0.0);

/// Velocity-solver substeps per tick. Contacts are detected once per tick and
/// reused by every substep.
pub const SUBSTEPS: u32 = 4;

/// Penetration left alone, in voxels, so a resting contact is not pushed out
/// and fallen back into every substep.
pub const SLOP: f32 = 0.02;

/// Fraction of the penetration beyond `SLOP` removed per substep, as velocity.
pub const BIAS: f32 = 0.2;

/// The fastest the bias may push a body out, in voxels per second, so a body
/// buried by an edit floats out rather than being fired out.
pub const MAX_PUSH: f32 = 20.0;

/// The furthest a body may move in one tick, in voxels: linear travel plus
/// the swing of its furthest voxel. Speeds are capped to it, which is what
/// lets speculative contacts stand in for continuous collision detection.
pub const MAX_TRAVEL: f32 = 1.25;

/// Speculative margin every body gets even at rest, in voxels.
pub const BASE_MARGIN: f32 = 0.1;

#[cfg(test)]
pub(crate) mod fixtures {
    use crate::body::Body;
    use crate::contree::Contree;
    use crate::material::{Material, MaterialId, MaterialTable};
    use glam::{Quat, UVec3, Vec3};

    /// Material 1 weighs 1000, material 2 weighs 3000.
    pub fn materials() -> MaterialTable {
        let mut table = MaterialTable::new();
        table.push(Material { color: [200, 200, 200, 255], density: 1000 }).unwrap();
        table.push(Material { color: [90, 90, 90, 255], density: 3000 }).unwrap();
        table
    }

    /// A solid `n`-cubed block of material 1 at the origin of an `extent` volume.
    pub fn cube(n: u32, extent: u32) -> Contree {
        let mut voxels = Vec::new();
        for z in 0..n {
            for y in 0..n {
                for x in 0..n {
                    voxels.push((UVec3::new(x, y, z), MaterialId(1)));
                }
            }
        }
        Contree::from_voxels(extent, &voxels)
    }

    /// A body with its mass properties computed and its centre of mass at
    /// `centre` in the world.
    pub fn placed(volume: Contree, centre: Vec3, orientation: Quat) -> Body {
        let mut body = Body::new(volume, Vec3::ZERO, orientation);
        assert!(body.recompute(&materials()), "a fixture body must have mass");
        body.position = centre;
        body
    }
}
```

In `lib.rs`, add `pub mod physics;` in alphabetical order, after `pub mod normal;`.

- [ ] **Step 2: Write the failing tests**

`crates/bevox_core/src/physics/mass.rs`, tests first:

```rust
#[cfg(test)]
mod tests {
    use super::*;
    use crate::physics::fixtures::{cube, materials};

    /// Summing unit-cube inertias with the parallel-axis theorem is exact for a
    /// block, so this is equality with the continuous formula, not an estimate:
    /// I = m * n^2 / 6 about each axis.
    #[test]
    fn a_solid_cube_has_the_textbook_inertia() {
        let (props, com) = mass_properties(&cube(4, 4).voxels(), &materials()).unwrap();
        assert_eq!(props.mass, 64_000.0);
        assert_eq!(com, Vec3::splat(2.0));
        let want = 6.0 / (64_000.0 * 16.0);
        let got = props.inverse_inertia;
        for (i, col) in [got.x_axis, got.y_axis, got.z_axis].iter().enumerate() {
            for (j, value) in col.to_array().iter().enumerate() {
                let expected = if i == j { want } else { 0.0 };
                assert!(
                    (value - expected).abs() <= want * 1e-5,
                    "inverse inertia [{i}][{j}] is {value}, want {expected}"
                );
            }
        }
    }

    /// A voxel of material 2 is three times as heavy, so the centre of mass
    /// sits three quarters of the way toward it.
    #[test]
    fn the_centre_of_mass_leans_toward_the_denser_voxel() {
        let voxels = [(UVec3::new(0, 0, 0), MaterialId(1)), (UVec3::new(1, 0, 0), MaterialId(2))];
        let (props, com) = mass_properties(&voxels, &materials()).unwrap();
        assert_eq!(props.mass, 4000.0);
        assert!((com - Vec3::new(1.25, 0.5, 0.5)).length() < 1e-6, "centre of mass {com:?}");
    }

    /// Nothing to simulate: no voxels, or voxels that weigh nothing.
    #[test]
    fn weightless_voxels_have_no_mass_properties() {
        assert!(mass_properties(&[], &materials()).is_none());
        let air = [(UVec3::ZERO, MaterialId::EMPTY)];
        assert!(mass_properties(&air, &materials()).is_none());
    }
}
```

In `body.rs`'s test module:

```rust
    /// Recomputing moves the pivot, never the voxels. Milestone 4's brush edits
    /// recompute a body in mid-air and must not make it jump.
    #[test]
    fn recompute_keeps_every_voxel_where_it_was() {
        let materials = crate::physics::fixtures::materials();
        // An L, so the centre of mass is nowhere near the volume's corner or middle.
        let mut voxels = Vec::new();
        for x in 0..8 {
            voxels.push((UVec3::new(x, 0, 0), MaterialId(1)));
        }
        for y in 1..6 {
            voxels.push((UVec3::new(0, y, 0), MaterialId(2)));
        }
        let orientation = Quat::from_euler(glam::EulerRot::XYZ, 0.4, -1.1, 0.7);
        let mut body = Body::new(Contree::from_voxels(16, &voxels), Vec3::new(5.0, -3.0, 9.0), orientation);

        let world = |b: &Body, list: &[(UVec3, MaterialId)]| -> Vec<Vec3> {
            list.iter().map(|(p, _)| b.world_from_local().transform_point3(p.as_vec3() + 0.5)).collect()
        };
        let before = world(&body, &voxels);
        assert!(body.recompute(&materials));
        assert_ne!(body.com, Vec3::ZERO, "the pivot did not move, so this proves nothing");
        for (a, b) in before.iter().zip(world(&body, &voxels)) {
            assert!((*a - b).length() < 1e-4, "recompute moved a voxel from {a:?} to {b:?}");
        }

        // Erase the upright of the L. The pivot moves again; the rest stays put.
        let base: Vec<_> = voxels.iter().copied().filter(|(p, _)| p.y == 0).collect();
        let before = world(&body, &base);
        let com = body.com;
        body.volume = Contree::from_voxels(16, &base);
        assert!(body.recompute(&materials));
        assert_ne!(body.com, com, "the edit did not move the pivot, so this proves nothing");
        for (a, b) in before.iter().zip(world(&body, &base)) {
            assert!((*a - b).length() < 1e-4, "an edit moved a voxel from {a:?} to {b:?}");
        }
    }

    #[test]
    fn a_body_with_no_voxels_does_not_recompute() {
        let mut body = Body::new(Contree::empty(2), Vec3::ZERO, Quat::IDENTITY);
        assert!(!body.recompute(&crate::physics::fixtures::materials()));
        assert_eq!(body.mass.mass, 0.0);
    }

    /// Every render test builds bodies with `new` and never recomputes them.
    /// Their placement must be the exact matrix it was before bodies had mass.
    #[test]
    fn a_body_never_recomputed_is_placed_exactly_as_before() {
        let orientation = Quat::from_euler(glam::EulerRot::XYZ, 0.3, 0.9, -0.4);
        let position = Vec3::new(12.5, -4.0, 30.0);
        let body = Body::new(cube(), position, orientation);
        assert_eq!(body.world_from_local(), Affine3A::from_rotation_translation(orientation, position));
    }
```

Make sure `body.rs`'s test module imports `MaterialId` (it already does), and `Contree` and `UVec3` through `super::*`.

- [ ] **Step 3: Run to verify they fail**

Run: `cargo test -p bevox_core > t2.txt 2>&1; echo $?`
Expected: non-zero. The compile fails: `mass_properties`, `recompute` and `com` are not defined.

- [ ] **Step 4: Implement mass properties**

At the top of `mass.rs`, above the tests:

```rust
//! Mass, centre of mass and inertia from voxel occupancy and material density.

use crate::material::{MaterialId, MaterialTable};
use glam::{DMat3, DVec3, Mat3, UVec3, Vec3};

/// How heavy a body is and how it resists turning. The centre of mass it is
/// measured about lives on `Body`, because it also places the body.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MassProperties {
    pub mass: f32,
    /// Inverse inertia tensor about the centre of mass, in the body's own axes.
    pub inverse_inertia: Mat3,
}

impl Default for MassProperties {
    /// No mass: a body that is not simulated.
    fn default() -> Self {
        Self { mass: 0.0, inverse_inertia: Mat3::ZERO }
    }
}

impl MassProperties {
    /// Zero for a body with no mass.
    pub fn inverse_mass(&self) -> f32 {
        if self.mass > 0.0 { 1.0 / self.mass } else { 0.0 }
    }
}

/// Mass properties of `voxels`, and their centre of mass in volume
/// coordinates. `None` when they weigh nothing.
///
/// Each voxel is a unit cube: its own inertia about its centre, `m / 6` on
/// each axis, is carried to the centre of mass by the parallel-axis theorem.
/// That makes the result exact for any union of voxels. Accumulated in f64,
/// because a 64-cubed body sums hundreds of thousands of terms.
pub fn mass_properties(voxels: &[(UVec3, MaterialId)], materials: &MaterialTable) -> Option<(MassProperties, Vec3)> {
    let mut mass = 0.0f64;
    let mut moment = DVec3::ZERO;
    for &(p, m) in voxels {
        let rho = materials.get(m).density as f64;
        mass += rho;
        moment += (p.as_dvec3() + 0.5) * rho;
    }
    if mass <= 0.0 {
        return None;
    }
    let com = moment / mass;

    let mut inertia = DMat3::ZERO;
    for &(p, m) in voxels {
        let rho = materials.get(m).density as f64;
        let d = p.as_dvec3() + 0.5 - com;
        let own = DMat3::from_diagonal(DVec3::splat(rho / 6.0));
        let carried = (DMat3::from_diagonal(DVec3::splat(d.length_squared())) - outer(d, d)) * rho;
        inertia += own + carried;
    }
    let props = MassProperties { mass: mass as f32, inverse_inertia: inertia.inverse().as_mat3() };
    Some((props, com.as_vec3()))
}

/// `a * bᵀ`.
fn outer(a: DVec3, b: DVec3) -> DMat3 {
    DMat3::from_cols(a * b.x, a * b.y, a * b.z)
}
```

- [ ] **Step 5: Implement the body fields and `recompute`**

In `body.rs`, replace the struct, `new` and `world_from_local`:

```rust
use crate::material::MaterialTable;
use crate::physics::mass::{MassProperties, mass_properties};

/// A voxel volume placed in the world by a rigid transform, and the state that
/// moves it.
#[derive(Clone, Debug)]
pub struct Body {
    pub volume: Contree,
    /// Where `com` is in the world: the point the body turns about.
    pub position: Vec3,
    pub orientation: Quat,
    /// Centre of mass in volume coordinates.
    ///
    /// Zero until `recompute`, which leaves a body built by `new` placed
    /// exactly as it was before bodies had mass.
    pub com: Vec3,
    /// Zero until `recompute`. A body with no mass is not simulated.
    pub mass: MassProperties,
    /// Voxels per second.
    pub velocity: Vec3,
    /// About `com`, in world axes.
    ///
    /// Stored instead of angular velocity because, with no torque, it is
    /// exactly constant, which is what lets the conservation gate demand
    /// exact equality.
    pub angular_momentum: Vec3,
}

impl Body {
    pub fn new(volume: Contree, position: Vec3, orientation: Quat) -> Self {
        debug_assert!(
            orientation.is_normalized(),
            "body orientation {orientation:?} is not a unit quaternion; it would scale the ray, \
             so `t` would stop meaning the same distance in the body's frame and the renderer's \
             nearest-hit composition would pick the wrong surface"
        );
        Self {
            volume,
            position,
            orientation,
            com: Vec3::ZERO,
            mass: MassProperties::default(),
            velocity: Vec3::ZERO,
            angular_momentum: Vec3::ZERO,
        }
    }

    /// Volume space to world space: about `com`, which sits at `position`.
    ///
    /// Written as one rotation and one translation rather than composed with a
    /// translation by `-com`. With `com` zero it is then exactly the matrix a
    /// body had before it had mass, which every render test's placement
    /// depends on.
    pub fn world_from_local(&self) -> Affine3A {
        Affine3A::from_rotation_translation(self.orientation, self.position - self.orientation * self.com)
    }
```

Keep `local_from_world` and `march_world` unchanged. Add after `march_world`:

```rust
    /// Recomputes mass properties after the volume changed, or for the first
    /// time. The pivot moves to the new centre of mass and `position` moves
    /// with it, so every voxel stays where it was in the world.
    ///
    /// Returns `false`, and leaves the body massless, when nothing in the
    /// volume weighs anything: such a body should be removed.
    pub fn recompute(&mut self, materials: &MaterialTable) -> bool {
        let voxels = self.volume.voxels();
        let Some((mass, com)) = mass_properties(&voxels, materials) else {
            self.mass = MassProperties::default();
            return false;
        };
        self.position += self.orientation * (com - self.com);
        self.com = com;
        self.mass = mass;
        true
    }
```

In `mod.rs`, check that `pub mod mass;` is present.

- [ ] **Step 6: Run to verify they pass**

Run: `cargo test --workspace > t2.txt 2>&1; echo $?`
Expected: `0`, with no warnings. The render crate's GPU tests pass unchanged; `a_body_never_recomputed_is_placed_exactly_as_before` is the reason.

- [ ] **Step 7: Break checks**

1. In `recompute`, delete `self.position += self.orientation * (com - self.com);`. Run `cargo test -p bevox_core recompute_keeps > b2a.txt 2>&1; echo $?`. Expected: non-zero, FAILS. Restore.
2. In `mass_properties`, change `rho / 6.0` to `rho / 12.0`. Run `cargo test -p bevox_core textbook_inertia > b2b.txt 2>&1; echo $?`. Expected: non-zero, FAILS. Restore.

Re-run both tests after restoring: `0`.

- [ ] **Step 8: Commit**

```bash
git add crates
git commit -m "feat(core): mass properties, and bodies that turn about their centre of mass" -m "Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>"
```

---

### Task 3: Voxel classification

**Files:**
- Create: `crates/bevox_core/src/physics/classify.rs`
- Modify: `crates/bevox_core/src/physics/mod.rs` (add `pub mod classify;`)
- Modify: `crates/bevox_core/src/body.rs` (`features` field, filled by `recompute`)

**Interfaces:**
- Consumes: `Contree::voxels`, `Body::recompute` (Tasks 1 and 2)
- Produces:
  - `enum Shape { Corner, Edge(usize), Face { axis: usize, open_plus: bool, open_minus: bool }, Interior }`
  - `classify(solid: impl Fn(IVec3) -> bool, p: IVec3) -> Shape`
  - `solid_at(tree: &Contree, p: IVec3) -> bool`
  - `struct Features { pub corners: Vec<UVec3>, pub edges: Vec<(UVec3, usize)> }`
  - `features(tree: &Contree, voxels: &[(UVec3, MaterialId)]) -> Features`
  - `Body::features: Features`

- [ ] **Step 1: Write the failing tests**

At the bottom of `classify.rs`:

```rust
#[cfg(test)]
mod tests {
    use super::*;
    use crate::physics::fixtures::cube;

    fn shapes_of(tree: &Contree) -> Vec<Shape> {
        tree.voxels().iter().map(|(p, _)| classify(|q| solid_at(tree, q), p.as_ivec3())).collect()
    }

    /// Only corners and edges can be touched first, so only they are kept.
    #[test]
    fn a_cube_keeps_its_eight_corners_and_twelve_runs_of_edges() {
        for n in [2u32, 4, 5] {
            let tree = cube(n, 16);
            let f = features(&tree, &tree.voxels());
            assert_eq!(f.corners.len(), 8, "{n}-cube corners");
            assert_eq!(f.edges.len(), 12 * (n as usize - 2), "{n}-cube edges");
        }
    }

    /// Every voxel gets exactly one label, and the counts are the textbook ones.
    #[test]
    fn every_voxel_of_a_cube_is_labelled_once() {
        let n = 5usize;
        let shapes = shapes_of(&cube(n as u32, 16));
        let count = |f: fn(&Shape) -> bool| shapes.iter().filter(|s| f(s)).count();
        assert_eq!(count(|s| matches!(s, Shape::Corner)), 8);
        assert_eq!(count(|s| matches!(s, Shape::Edge(_))), 12 * (n - 2));
        assert_eq!(count(|s| matches!(s, Shape::Face { .. })), 6 * (n - 2) * (n - 2));
        assert_eq!(count(|s| matches!(s, Shape::Interior)), (n - 2).pow(3));
    }

    #[test]
    fn a_bar_has_corner_ends_and_an_edge_along_it() {
        let tree = Contree::from_voxels(
            4,
            &[(UVec3::new(0, 1, 1), MaterialId(1)), (UVec3::new(1, 1, 1), MaterialId(1)), (UVec3::new(2, 1, 1), MaterialId(1))],
        );
        let solid = |q| solid_at(&tree, q);
        assert_eq!(classify(solid, IVec3::new(0, 1, 1)), Shape::Corner);
        assert_eq!(classify(solid, IVec3::new(1, 1, 1)), Shape::Edge(0));
        assert_eq!(classify(solid, IVec3::new(2, 1, 1)), Shape::Corner);
    }

    /// A one-voxel plate is a face open on both sides, which is how a thin
    /// floor is seen from above and below.
    #[test]
    fn a_plate_is_open_on_both_sides() {
        let mut voxels = Vec::new();
        for z in 0..3 {
            for x in 0..3 {
                voxels.push((UVec3::new(x, 1, z), MaterialId(1)));
            }
        }
        let tree = Contree::from_voxels(4, &voxels);
        assert_eq!(
            classify(|q| solid_at(&tree, q), IVec3::new(1, 1, 1)),
            Shape::Face { axis: 1, open_plus: true, open_minus: true }
        );
    }

    /// Outside the volume is empty, so a voxel on the volume's boundary is
    /// exposed there.
    #[test]
    fn outside_the_volume_is_empty() {
        let tree = cube(4, 4);
        assert!(!solid_at(&tree, IVec3::new(-1, 0, 0)));
        assert!(!solid_at(&tree, IVec3::new(0, 4, 0)));
        assert!(solid_at(&tree, IVec3::new(3, 3, 3)));
    }
}
```

In `body.rs`'s test module:

```rust
    #[test]
    fn recompute_classifies_the_voxels() {
        let mut body = Body::new(crate::physics::fixtures::cube(4, 4), Vec3::ZERO, Quat::IDENTITY);
        assert!(body.features.corners.is_empty());
        assert!(body.recompute(&crate::physics::fixtures::materials()));
        assert_eq!(body.features.corners.len(), 8);
        assert_eq!(body.features.edges.len(), 24);
    }
```

- [ ] **Step 2: Run to verify they fail**

Run: `cargo test -p bevox_core > t3.txt 2>&1; echo $?`
Expected: non-zero. The compile fails because `classify` does not exist.

- [ ] **Step 3: Implement**

At the top of `classify.rs`:

```rust
//! Which voxels of a body can be touched first, and in what shape.
//!
//! After Dwyer's devlogs #20 and #26: a voxel is labelled by how many axes it
//! has solid neighbours on both sides. Only corners and edges ever need
//! testing against another volume. A face voxel is only touched where a
//! corner of the other volume reaches it, and an interior voxel never is.

use crate::contree::Contree;
use crate::material::MaterialId;
use glam::{IVec3, UVec3};

const AXES: [IVec3; 3] = [IVec3::X, IVec3::Y, IVec3::Z];

/// A solid voxel's collision shape: a cube rounded at its corners and edges,
/// full across its faces.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Shape {
    /// Solid on both sides along no axis: a sphere of radius 0.5.
    Corner,
    /// Along exactly the named axis: a cylinder of radius 0.5 along it.
    Edge(usize),
    /// Along two axes: flat across `axis`, on whichever sides are open.
    Face { axis: usize, open_plus: bool, open_minus: bool },
    /// Along all three: never touched first.
    Interior,
}

/// Whether `p` is a solid voxel of `tree`. Outside the volume is empty.
pub fn solid_at(tree: &Contree, p: IVec3) -> bool {
    let n = tree.extent() as i32;
    p.cmpge(IVec3::ZERO).all() && p.cmplt(IVec3::splat(n)).all() && !tree.get(p.as_uvec3()).is_empty()
}

/// The shape of the solid voxel at `p`, from its six neighbours.
pub fn classify(solid: impl Fn(IVec3) -> bool, p: IVec3) -> Shape {
    let enclosed = AXES.map(|e| solid(p + e) && solid(p - e));
    match enclosed.iter().filter(|&&b| b).count() {
        0 => Shape::Corner,
        1 => Shape::Edge(enclosed.iter().position(|&b| b).unwrap()),
        2 => {
            let axis = enclosed.iter().position(|&b| !b).unwrap();
            Shape::Face { axis, open_plus: !solid(p + AXES[axis]), open_minus: !solid(p - AXES[axis]) }
        }
        _ => Shape::Interior,
    }
}

/// A body's corner and edge voxels, the only ones tested against the world.
#[derive(Clone, Debug, Default)]
pub struct Features {
    pub corners: Vec<UVec3>,
    /// With the axis each edge runs along.
    pub edges: Vec<(UVec3, usize)>,
}

/// Classifies every one of `voxels`, which must be `tree`'s own.
pub fn features(tree: &Contree, voxels: &[(UVec3, MaterialId)]) -> Features {
    let mut out = Features::default();
    for &(p, _) in voxels {
        match classify(|q| solid_at(tree, q), p.as_ivec3()) {
            Shape::Corner => out.corners.push(p),
            Shape::Edge(axis) => out.edges.push((p, axis)),
            Shape::Face { .. } | Shape::Interior => {}
        }
    }
    out
}
```

In `mod.rs`, add `pub mod classify;`.

In `body.rs`:
- Import `use crate::physics::classify::{Features, features};`.
- Add the field:

  ```rust
      /// Corner and edge voxels, the only ones tested for contact. Empty until
      /// `recompute`.
      pub features: Features,
  ```

- Initialise `features: Features::default()` in `new`.
- In `recompute`, after `self.mass = mass;`, add `self.features = features(&self.volume, &voxels);`.
- In the massless early return, add `self.features = Features::default();`.

- [ ] **Step 4: Run to verify they pass**

Run: `cargo test --workspace > t3.txt 2>&1; echo $?`
Expected: `0`, with no warnings.

- [ ] **Step 5: Break check**

In `classify`, change `solid(p + e) && solid(p - e)` to `solid(p + e) || solid(p - e)`. Run `cargo test -p bevox_core classify > b3.txt 2>&1; echo $?`. Expected: non-zero, and the cube counts FAIL. Restore it and re-run: `0`.

- [ ] **Step 6: Commit**

```bash
git add crates
git commit -m "feat(core): classify body voxels as corners, edges, faces and interior" -m "Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>"
```

---

### Task 4: Contacts against the static world

**Files:**
- Create: `crates/bevox_core/src/physics/contact.rs`
- Modify: `crates/bevox_core/src/physics/mod.rs` (add `pub mod contact;` and the `slab` fixture)
- Modify: `crates/bevox_core/src/body.rs` (`warm` field)
- Modify: `docs/superpowers/specs/2026-09-15-bevox-rigid-bodies-design.md`

**Interfaces:**
- Consumes: `Shape`, `classify`, `solid_at`, `Features` (Task 3); `Body::{world_from_local, local_from_world, com, orientation, features}`; `occupied_bounds`; `DistanceField::get`; `CELL_VOXELS`.
- Produces:
  - `pub const RADIUS: f32 = 0.5;`
  - `pub type ContactKey = (UVec3, IVec3);`
  - `struct Contact { pub key: ContactKey, pub normal: Vec3, pub separation: f32, pub anchor: Vec3, pub world_point: Vec3 }`
  - `sphere_vs_voxel(q: Vec3, c: Vec3, shape: Shape) -> Option<(f32, Vec3)>`
  - `edge_vs_edge(p: Vec3, d: Vec3, c: Vec3, e: Vec3) -> Option<(f32, Vec3, Vec3)>`
  - `world_box(body: &Body, margin: f32) -> Option<(Vec3, Vec3)>`
  - `field_clears(field: &DistanceField, min: Vec3, max: Vec3) -> bool`
  - `detect(body: &Body, tree: &Contree, field: &DistanceField, margin: f32) -> Vec<Contact>` (sorted by key)
  - `Body::warm: HashMap<ContactKey, f32>`
  - fixture `slab(extent: u32, ys: std::ops::Range<u32>) -> Contree`

- [ ] **Step 1: Add the fixture**

In `mod.rs`'s `fixtures`:

```rust
    /// A world whose layers `ys` are solid material 1 across the whole volume.
    pub fn slab(extent: u32, ys: std::ops::Range<u32>) -> Contree {
        let mut voxels = Vec::new();
        for z in 0..extent {
            for y in ys.clone() {
                for x in 0..extent {
                    voxels.push((UVec3::new(x, y, z), MaterialId(1)));
                }
            }
        }
        Contree::from_voxels(extent, &voxels)
    }
```

- [ ] **Step 2: Write the failing tests**

At the bottom of `contact.rs`:

```rust
#[cfg(test)]
mod tests {
    use super::*;
    use crate::material::MaterialId;
    use crate::physics::fixtures::{cube, placed, slab};
    use glam::Quat;

    const TOLERANCE: f32 = 1e-4;

    #[test]
    fn two_corner_spheres_are_a_sphere_pair() {
        let (sep, n) = sphere_vs_voxel(Vec3::new(1.2, 0.0, 0.0), Vec3::ZERO, Shape::Corner).unwrap();
        assert!((sep - 0.2).abs() < TOLERANCE, "separation {sep}");
        assert!((n - Vec3::X).length() < TOLERANCE, "normal {n:?}");
    }

    /// An edge owns only the sphere alongside it, not one past its end: the
    /// next voxel along the edge owns that one.
    #[test]
    fn an_edge_owns_only_the_sphere_alongside_it() {
        let (sep, n) = sphere_vs_voxel(Vec3::new(0.3, 0.9, 0.0), Vec3::ZERO, Shape::Edge(0)).unwrap();
        assert!((sep + 0.1).abs() < TOLERANCE, "separation {sep}");
        assert!((n - Vec3::Y).length() < TOLERANCE, "normal {n:?}");
        assert!(sphere_vs_voxel(Vec3::new(0.6, 0.9, 0.0), Vec3::ZERO, Shape::Edge(0)).is_none());
    }

    /// A face owns only the sphere over it, and pushes out of its open side.
    #[test]
    fn a_face_owns_only_the_sphere_over_it() {
        let top = Shape::Face { axis: 1, open_plus: true, open_minus: false };
        let (sep, n) = sphere_vs_voxel(Vec3::new(0.2, 1.0, -0.4), Vec3::ZERO, top).unwrap();
        assert!(sep.abs() < TOLERANCE, "separation {sep}");
        assert_eq!(n, Vec3::Y);
        assert!(sphere_vs_voxel(Vec3::new(0.5, 1.0, 0.0), Vec3::ZERO, top).is_none(), "the next voxel owns x = 0.5");
        assert!(sphere_vs_voxel(Vec3::new(0.0, -1.0, 0.0), Vec3::ZERO, top).is_none(), "the closed side pushes nothing");
        assert!(sphere_vs_voxel(Vec3::new(0.0, 1.0, 0.0), Vec3::ZERO, Shape::Interior).is_none());
    }

    #[test]
    fn crossed_edges_meet_where_their_lines_do() {
        let (sep, n, point) = edge_vs_edge(Vec3::new(0.0, 1.1, 0.0), Vec3::X, Vec3::ZERO, Vec3::Z).unwrap();
        assert!((sep - 0.1).abs() < TOLERANCE, "separation {sep}");
        assert!((n - Vec3::Y).length() < TOLERANCE, "normal {n:?}");
        assert!((point - Vec3::new(0.0, 0.55, 0.0)).length() < TOLERANCE, "point {point:?}");
        assert!(edge_vs_edge(Vec3::new(0.0, 1.1, 0.7), Vec3::X, Vec3::ZERO, Vec3::Z).is_none(), "past the world edge's end");
        let (sep, n, _) = edge_vs_edge(Vec3::new(0.2, 1.0, 0.0), Vec3::X, Vec3::ZERO, Vec3::X).unwrap();
        assert!(sep.abs() < TOLERANCE && (n - Vec3::Y).length() < TOLERANCE, "parallel: {sep} {n:?}");
    }

    /// Dwyer's devlog #20: a box sitting on the ground touches it only at its
    /// corners. Turning the box about the vertical must not change that.
    #[test]
    fn a_cube_resting_on_the_floor_touches_it_at_four_corners() {
        let world = slab(64, 0..8);
        let field = DistanceField::build(&world);
        for angle in [0.0f32, 0.5236] {
            let body = placed(cube(4, 4), Vec3::new(32.0, 10.0, 32.0), Quat::from_rotation_y(angle));
            let contacts = detect(&body, &world, &field, 0.1);
            assert_eq!(contacts.len(), 4, "at {angle} rad: {contacts:#?}");
            for c in &contacts {
                assert!(c.separation.abs() < TOLERANCE, "at {angle} rad, separation {}", c.separation);
                assert!((c.normal - Vec3::Y).length() < TOLERANCE, "at {angle} rad, normal {:?}", c.normal);
                assert!(c.key.0.y == 0, "a contact came from voxel {:?}, not the bottom", c.key.0);
            }
        }
    }

    /// A body face resting on a lone world pillar: no corner of the body is
    /// there, so only the world-corner loop can find it.
    #[test]
    fn a_world_corner_meets_a_body_face() {
        let mut voxels = Vec::new();
        for y in 0..8 {
            voxels.push((UVec3::new(32, y, 32), MaterialId(1)));
        }
        let world = Contree::from_voxels(64, &voxels);
        let field = DistanceField::build(&world);
        // A 4-cube centred over the pillar: the pillar's top meets the middle of its bottom face.
        let body = placed(cube(4, 4), Vec3::new(32.5, 10.0, 32.5), Quat::IDENTITY);
        let contacts = detect(&body, &world, &field, 0.1);
        assert!(!contacts.is_empty(), "the pillar was not found");
        for c in &contacts {
            assert_eq!(c.key.1, IVec3::new(32, 7, 32), "contact with {:?}", c.key.1);
            assert!((c.normal - Vec3::Y).length() < TOLERANCE, "normal {:?}", c.normal);
            assert!(c.separation.abs() < TOLERANCE, "separation {}", c.separation);
        }
    }

    /// A bar lying across a bar touches it mid-span, where neither has a corner.
    #[test]
    fn crossed_bars_touch_through_their_edges() {
        let mut voxels = Vec::new();
        for z in 20..44 {
            voxels.push((UVec3::new(32, 8, z), MaterialId(1)));
        }
        let world = Contree::from_voxels(64, &voxels);
        let field = DistanceField::build(&world);
        let bar: Vec<_> = (0..16).map(|x| (UVec3::new(x, 0, 0), MaterialId(1))).collect();
        let body = placed(Contree::from_voxels(16, &bar), Vec3::new(32.5, 9.5, 32.5), Quat::IDENTITY);
        let contacts = detect(&body, &world, &field, 0.1);
        assert!(
            contacts.iter().any(|c| c.key == (UVec3::new(8, 0, 0), IVec3::new(32, 8, 32)) && c.separation.abs() < TOLERANCE),
            "no edge contact at the crossing: {contacts:#?}"
        );
    }

    /// The field clears a box only inside the cube its cell promises.
    #[test]
    fn the_field_clears_only_what_it_promises() {
        let world = slab(64, 0..8);
        let field = DistanceField::build(&world);
        // Cell y = 2 (voxels 32..48) is two cells from the floor's cell row:
        // free from voxel 16 up.
        assert!(field_clears(&field, Vec3::new(20.0, 38.0, 20.0), Vec3::new(28.0, 46.0, 28.0)));
        // Centred in cell y = 1 (16..32), which is one cell from the floor, so
        // it promises only its own cell. This box reaches below 16.
        assert!(!field_clears(&field, Vec3::new(20.0, 14.0, 20.0), Vec3::new(28.0, 20.0, 28.0)));
        assert!(!field_clears(&field, Vec3::new(20.0, 4.0, 20.0), Vec3::new(28.0, 10.0, 28.0)));
    }

    #[test]
    fn a_body_in_open_air_has_no_contacts() {
        let world = slab(64, 0..8);
        let field = DistanceField::build(&world);
        let body = placed(cube(4, 4), Vec3::new(32.0, 40.0, 32.0), Quat::IDENTITY);
        assert!(detect(&body, &world, &field, 1.35).is_empty());
    }
}
```

- [ ] **Step 3: Run to verify they fail**

Run: `cargo test -p bevox_core contact > t4.txt 2>&1; echo $?`
Expected: non-zero, because `contact` is not defined.

- [ ] **Step 4: Implement**

At the top of `contact.rs`:

```rust
//! Contacts between a body and the static world.
//!
//! After Dwyer's devlogs #20 and #26. Body corners are tested against world
//! voxels, world corners against body voxels, and edges against edges, all as
//! cubes rounded at their corners and edges. The exact formulas, the reach and
//! the ownership rules are this design's own; see the spec's Provenance section.

use super::classify::{Shape, classify, solid_at};
use crate::body::{Body, occupied_bounds};
use crate::contree::Contree;
use crate::distance_field::{CELL_VOXELS, DistanceField};
use glam::{IVec3, UVec3, Vec3};
use std::collections::HashMap;

/// The rounding radius of a voxel's corners and edges.
pub const RADIUS: f32 = 0.5;

const AXES: [Vec3; 3] = [Vec3::X, Vec3::Y, Vec3::Z];

/// Which body voxel touched which world voxel. Stable from tick to tick while
/// the touch persists, which is what warm starting keys on.
pub type ContactKey = (UVec3, IVec3);

#[derive(Clone, Copy, Debug)]
pub struct Contact {
    pub key: ContactKey,
    /// World space, pointing from the world into the body: the way the body is pushed.
    pub normal: Vec3,
    /// Signed gap at detection. Negative is penetration; positive is speculative.
    pub separation: f32,
    /// The contact point relative to the centre of mass, in the body's own axes.
    pub anchor: Vec3,
    /// Where the contact point was in the world at detection.
    pub world_point: Vec3,
}

/// Owned ranges are half-open, so a point on the boundary between two voxels
/// belongs to exactly one of them.
fn owns(x: f32) -> bool {
    (-RADIUS..RADIUS).contains(&x)
}

/// A rounded corner at `q` against the solid voxel `shape` centred at `c`.
///
/// Returns the separation and the normal pointing from that voxel toward `q`.
/// Returns `None` when this voxel does not own the touch: a face owns only the
/// sphere over it, and an edge only the one alongside it, so a sphere within
/// reach of many voxels is reported once.
pub fn sphere_vs_voxel(q: Vec3, c: Vec3, shape: Shape) -> Option<(f32, Vec3)> {
    let w = q - c;
    match shape {
        Shape::Corner => {
            let d = w.length();
            (d > 1e-6).then(|| (d - 2.0 * RADIUS, w / d))
        }
        Shape::Edge(axis) => {
            if !owns(w[axis]) {
                return None;
            }
            let mut across = w;
            across[axis] = 0.0;
            let d = across.length();
            (d > 1e-6).then(|| (d - 2.0 * RADIUS, across / d))
        }
        Shape::Face { axis, open_plus, open_minus } => {
            if !owns(w[(axis + 1) % 3]) || !owns(w[(axis + 2) % 3]) {
                return None;
            }
            let side = if open_plus && (!open_minus || w[axis] >= 0.0) {
                1.0
            } else if open_minus {
                -1.0
            } else {
                return None;
            };
            let height = w[axis] * side;
            if height < 0.0 {
                // Behind the face: the voxel on the open side owns it.
                return None;
            }
            let mut n = Vec3::ZERO;
            n[axis] = side;
            Some((height - 2.0 * RADIUS, n))
        }
        Shape::Interior => None,
    }
}

/// A rounded body edge through `p` along unit `d`, against a world edge
/// voxel centred at `c` along unit axis `e`.
///
/// Returns the separation, the normal from the world edge toward the body
/// edge, and the midpoint between them. Returns `None` unless the closest
/// approach lies within both voxels' length; past the end, the neighbouring
/// voxel along the edge owns the touch.
pub fn edge_vs_edge(p: Vec3, d: Vec3, c: Vec3, e: Vec3) -> Option<(f32, Vec3, Vec3)> {
    let r = p - c;
    let b = d.dot(e);
    let denom = 1.0 - b * b;
    let (t, s) = if denom < 1e-4 {
        // Parallel: the body voxel's own centre, projected onto the world edge.
        (0.0, r.dot(e))
    } else {
        let (dd, ee) = (d.dot(r), e.dot(r));
        ((b * ee - dd) / denom, (ee - b * dd) / denom)
    };
    if !owns(t) || !owns(s) {
        return None;
    }
    let (on_body, on_world) = (p + d * t, c + e * s);
    let w = on_body - on_world;
    let dist = w.length();
    (dist > 1e-6).then(|| (dist - 2.0 * RADIUS, w / dist, (on_body + on_world) * 0.5))
}

/// The body's world-space box: its occupied bounds carried through its
/// transform, grown by `margin` and a voxel's rounding.
pub fn world_box(body: &Body, margin: f32) -> Option<(Vec3, Vec3)> {
    let (lo, hi) = occupied_bounds(&body.volume)?;
    let (lo, hi) = (lo.as_vec3(), hi.as_vec3());
    let m = body.world_from_local();
    let mut min = Vec3::splat(f32::INFINITY);
    let mut max = Vec3::splat(f32::NEG_INFINITY);
    for i in 0..8 {
        let corner = Vec3::new(
            if i & 1 == 0 { lo.x } else { hi.x },
            if i & 2 == 0 { lo.y } else { hi.y },
            if i & 4 == 0 { lo.z } else { hi.z },
        );
        let p = m.transform_point3(corner);
        min = min.min(p);
        max = max.max(p);
    }
    Some((min - (margin + RADIUS), max + (margin + RADIUS)))
}

/// Whether the distance field promises that no world voxel lies in the box.
///
/// One lookup, at the cell holding the box's centre. A cell `d` cells from the
/// nearest solid cell promises that every cell within `d - 1` of it is empty.
pub fn field_clears(field: &DistanceField, min: Vec3, max: Vec3) -> bool {
    let size = CELL_VOXELS as f32;
    let cell = ((min + max) * 0.5 / size).floor();
    if cell.cmplt(Vec3::ZERO).any() {
        return false;
    }
    let d = field.get(cell.as_uvec3()) as f32;
    if d == 0.0 {
        return false;
    }
    let free_min = (cell - (d - 1.0)) * size;
    let free_max = (cell + d) * size;
    min.cmpge(free_min).all() && max.cmple(free_max).all()
}

/// Every voxel within `reach` of the one containing `q`.
fn around(q: Vec3, reach: i32) -> impl Iterator<Item = IVec3> {
    let centre = q.floor().as_ivec3();
    (-reach..=reach).flat_map(move |z| {
        (-reach..=reach).flat_map(move |y| (-reach..=reach).map(move |x| centre + IVec3::new(x, y, z)))
    })
}

/// Every world voxel in the box, clipped to the world.
fn box_voxels(min: Vec3, max: Vec3, extent: u32) -> impl Iterator<Item = IVec3> {
    let lo = min.floor().as_ivec3().max(IVec3::ZERO);
    let hi = max.ceil().as_ivec3().min(IVec3::splat(extent as i32));
    (lo.z..hi.z).flat_map(move |z| (lo.y..hi.y).flat_map(move |y| (lo.x..hi.x).map(move |x| IVec3::new(x, y, z))))
}

/// Records a contact unless it is past the margin. When two loops find the
/// same voxel pair, the first stands.
fn keep(
    found: &mut HashMap<ContactKey, Contact>,
    body: &Body,
    margin: f32,
    key: ContactKey,
    normal: Vec3,
    separation: f32,
    world_point: Vec3,
) {
    if separation > margin {
        return;
    }
    let anchor = body.local_from_world().transform_point3(world_point) - body.com;
    found.entry(key).or_insert(Contact { key, normal, separation, anchor, world_point });
}

/// Every contact between `body` and the static world with a separation of at
/// most `margin`, sorted by key so the solver is deterministic.
///
/// The reach grows with the margin. A contact at `margin` needs voxels up to
/// `ceil(margin + 0.5)` away, and a floor exactly one voxel below a resting
/// corner sits on a 2x2x2 lookup's tie boundary.
pub fn detect(body: &Body, tree: &Contree, field: &DistanceField, margin: f32) -> Vec<Contact> {
    let Some((min, max)) = world_box(body, margin) else {
        return Vec::new();
    };
    if field_clears(field, min, max) {
        return Vec::new();
    }
    let world_from_local = body.world_from_local();
    let local_from_world = body.local_from_world();
    let reach = (margin + RADIUS).ceil() as i32;
    let world_solid = |p: IVec3| solid_at(tree, p);
    let body_solid = |p: IVec3| solid_at(&body.volume, p);
    let mut found = HashMap::new();

    // A body corner against every world voxel within reach.
    for &u in &body.features.corners {
        let q = world_from_local.transform_point3(u.as_vec3() + 0.5);
        for k in around(q, reach) {
            if !world_solid(k) {
                continue;
            }
            if let Some((sep, n)) = sphere_vs_voxel(q, k.as_vec3() + 0.5, classify(world_solid, k)) {
                keep(&mut found, body, margin, (u, k), n, sep, q - n * (RADIUS + sep * 0.5));
            }
        }
    }

    // A world corner in the box against every body voxel within reach. This
    // runs in the body's frame, where its voxels are axis-aligned.
    for k in box_voxels(min, max, tree.extent()) {
        if !world_solid(k) || classify(world_solid, k) != Shape::Corner {
            continue;
        }
        let q = local_from_world.transform_point3(k.as_vec3() + 0.5);
        for u in around(q, reach) {
            if !body_solid(u) {
                continue;
            }
            if let Some((sep, n_local)) = sphere_vs_voxel(q, u.as_vec3() + 0.5, classify(body_solid, u)) {
                // `n_local` points from the body voxel toward the world corner;
                // the body is pushed the other way.
                let point = world_from_local.transform_point3(q - n_local * (RADIUS + sep * 0.5));
                keep(&mut found, body, margin, (u.as_uvec3(), k), -(body.orientation * n_local), sep, point);
            }
        }
    }

    // A body edge against every world edge within reach.
    for &(u, axis) in &body.features.edges {
        let p = world_from_local.transform_point3(u.as_vec3() + 0.5);
        let d = body.orientation * AXES[axis];
        for k in around(p, reach) {
            if !world_solid(k) {
                continue;
            }
            let Shape::Edge(world_axis) = classify(world_solid, k) else { continue };
            if let Some((sep, n, point)) = edge_vs_edge(p, d, k.as_vec3() + 0.5, AXES[world_axis]) {
                keep(&mut found, body, margin, (u, k), n, sep, point);
            }
        }
    }

    let mut contacts: Vec<Contact> = found.into_values().collect();
    contacts.sort_by_key(|c| (c.key.0.to_array(), c.key.1.to_array()));
    contacts
}
```

In `mod.rs`, add `pub mod contact;`.

In `body.rs`:
- Import `use crate::physics::contact::ContactKey;` and `use std::collections::HashMap;`.
- Add the field:

  ```rust
      /// Each contact's accumulated normal impulse from the last tick, for
      /// warm starting. Cleared by `recompute`, whose voxels may have moved.
      pub warm: HashMap<ContactKey, f32>,
  ```

- Initialise `warm: HashMap::new()` in `new`.
- Add `self.warm.clear();` at the top of `recompute`.

**Borrow note:** `classify(world_solid, k)` passes the closure by value. `world_solid` is `Copy`, because it captures only a shared reference, so this compiles. If the compiler disagrees, pass `&world_solid`, because `&F` implements `Fn`.

- [ ] **Step 5: Run to verify they pass**

Run: `cargo test --workspace > t4.txt 2>&1; echo $?`
Expected: `0`, with no warnings.

- [ ] **Step 6: Break checks**

Run each break, confirm it FAILS, then restore it:
1. Replace the world-corner loop's body with `continue;`. `a_world_corner_meets_a_body_face` must FAIL.
2. Replace the edge loop's body with `continue;`. `crossed_bars_touch_through_their_edges` must FAIL.
3. In the corner loop, use `u.as_vec3() + 0.5` as `q` without `world_from_local`. `a_cube_resting_on_the_floor_touches_it_at_four_corners` must FAIL.
4. In `field_clears`, change `(d - 1.0)` to `d`. `the_field_clears_only_what_it_promises` must FAIL.
5. In `owns`, change the range to `-RADIUS..=RADIUS`. `a_face_owns_only_the_sphere_over_it` must FAIL.

Run each with `cargo test -p bevox_core <test name> > b4-N.txt 2>&1; echo $?`. After restoring all five, `cargo test -p bevox_core contact` must return `0`.

- [ ] **Step 7: Update the spec**

In `docs/superpowers/specs/2026-09-15-bevox-rigid-bodies-design.md`, replace step 3 of Collision detection:

```markdown
3. **Lookup.** A candidate voxel's centre is transformed into the other side's
   grid, and every voxel within `ceil(margin + 0.5)` of the voxel containing
   it is examined: 3x3x3 at rest, 5x5x5 at the speed cap. Dwyer's devlog says
   the nearest 8. That misses speculative contacts, and a corner resting
   exactly one voxel above a floor sits on its tie boundary, so the contact
   would come and go from tick to tick.
```

In the Provenance fill-in list, add these three entries: "the lookup reach"; "the ownership rules that report each touch once"; "the unbiased relax solve after each substep's position update, after Erin Catto's soft-step solver in Box2D v3".

- [ ] **Step 8: Commit**

```bash
git add crates docs
git commit -m "feat(core): rounded-voxel contacts between a body and the world" -m "Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>"
```

---

### Task 5: The TGS solver

**Files:**
- Create: `crates/bevox_core/src/physics/solver.rs`
- Modify: `crates/bevox_core/src/physics/mod.rs` (add `pub mod solver;`)

**Interfaces:**
- Consumes: `detect`, `world_box`, `Contact`, `RADIUS` (Task 4); `Body` physics fields; the constants in `physics` (Task 2)
- Produces: `pub fn step(bodies: &mut Vec<Body>, tree: &Contree, field: &DistanceField, gravity: Vec3, dt: f32) -> bool`, which returns whether any body was removed

- [ ] **Step 1: Write the failing tests**

At the bottom of `solver.rs`:

```rust
#[cfg(test)]
mod tests {
    use super::*;
    use crate::material::MaterialId;
    use crate::physics::fixtures::{cube, placed, slab};
    use crate::physics::GRAVITY;
    use glam::EulerRot;

    const DT: f32 = 1.0 / 64.0;

    fn run(bodies: &mut Vec<Body>, world: &Contree, field: &DistanceField, gravity: Vec3, ticks: u32) {
        for _ in 0..ticks {
            step(bodies, world, field, gravity, DT);
        }
    }

    /// With no gravity and nothing to touch, momentum is exactly conserved,
    /// not approximately: velocity and angular momentum are stored, and
    /// nothing may write them.
    #[test]
    fn momentum_is_conserved_exactly_in_free_flight() {
        let world = Contree::empty(3);
        let field = DistanceField::build(&world);
        let mut body = placed(cube(4, 4), Vec3::splat(500.0), Quat::from_euler(EulerRot::XYZ, 0.3, 0.9, -0.4));
        body.velocity = Vec3::new(3.0, -1.0, 2.0);
        body.angular_momentum = Vec3::new(20_000.0, -45_000.0, 10_000.0);
        let (v, l, q) = (body.velocity, body.angular_momentum, body.orientation);
        let mut bodies = vec![body];
        run(&mut bodies, &world, &field, Vec3::ZERO, 10_000);
        assert_eq!(bodies[0].velocity, v);
        assert_eq!(bodies[0].angular_momentum, l);
        assert!(bodies[0].orientation.angle_between(q) > 0.01, "it never turned, so this proves nothing");
    }

    /// Semi-implicit Euler per substep has a closed form:
    /// y_N = y_0 - g h^2 N (N + 1) / 2 after N substeps.
    #[test]
    fn a_free_fall_matches_its_closed_form() {
        let world = Contree::empty(3);
        let field = DistanceField::build(&world);
        let mut bodies = vec![placed(cube(4, 4), Vec3::new(32.0, 200.0, 32.0), Quat::IDENTITY)];
        let ticks = 32;
        run(&mut bodies, &world, &field, GRAVITY, ticks);
        let h = DT / SUBSTEPS as f32;
        let n = (ticks * SUBSTEPS) as f32;
        let want = 200.0 + GRAVITY.y * h * h * n * (n + 1.0) * 0.5;
        let got = bodies[0].position.y;
        assert!((got - want).abs() < 1e-3 * (200.0 - want), "fell to {got}, want {want}");
    }

    /// A body turning about the world's y axis turns about the world's y axis,
    /// whatever its orientation. Swapping the quaternion product, the bug
    /// Dwyer's devlog #12 spent three weeks on, turns it about a body axis
    /// instead.
    #[test]
    fn a_spin_turns_about_the_world_axis_it_names() {
        let world = Contree::empty(3);
        let field = DistanceField::build(&world);
        let start = Quat::from_euler(EulerRot::XYZ, 0.3, 0.9, -0.4);
        let mut body = placed(cube(4, 4), Vec3::splat(500.0), start);
        // A cube's inertia is the same on every axis, so momentum and spin are parallel.
        let rate = 2.0;
        body.angular_momentum = Vec3::Y * rate / body.mass.inverse_inertia.y_axis.y;
        let mut bodies = vec![body];
        run(&mut bodies, &world, &field, Vec3::ZERO, 1);
        let (axis, angle) = (bodies[0].orientation * start.inverse()).to_axis_angle();
        assert!(axis.y.abs() > 0.999, "turned about {axis:?}");
        assert!((angle - rate * DT).abs() < 0.01 * rate * DT, "turned {angle}, want {}", rate * DT);
    }

    /// Dropped flat from half a voxel up, a cube lands and stays put. Jitter
    /// is the characteristic failure of impulse solvers and hides from short
    /// tests, so the last thousand ticks of ten thousand are watched.
    #[test]
    fn a_body_at_rest_stays_at_rest() {
        let world = slab(64, 0..8);
        let field = DistanceField::build(&world);
        let mut bodies = vec![placed(cube(4, 4), Vec3::new(32.0, 10.5, 32.0), Quat::IDENTITY)];
        run(&mut bodies, &world, &field, GRAVITY, 1000);
        let settled = bodies[0].position.y;
        assert!((settled - 10.0).abs() < 0.1, "settled at {settled}, not on the floor at 10");

        run(&mut bodies, &world, &field, GRAVITY, 8000);
        let (mut low, mut high) = (f32::INFINITY, f32::NEG_INFINITY);
        for _ in 0..1000 {
            step(&mut bodies, &world, &field, GRAVITY, DT);
            low = low.min(bodies[0].position.y);
            high = high.max(bodies[0].position.y);
        }
        let body = &bodies[0];
        assert!(high - low < 1e-4, "height wandered {low}..{high} over the last 1000 ticks");
        assert!((body.position.y - settled).abs() < 1e-3, "drifted from {settled} to {}", body.position.y);
        assert!(body.velocity.length() < 1e-2, "still moving at {:?}", body.velocity);

        // At rest, the contacts carry exactly the weight: one tick's normal
        // impulse is m |g| dt.
        assert!(body.warm.len() >= 4, "resting on {} contacts", body.warm.len());
        let per_tick: f32 = body.warm.values().sum::<f32>() * SUBSTEPS as f32;
        let weight = body.mass.mass * -GRAVITY.y * DT;
        assert!((per_tick - weight).abs() < 0.05 * weight, "normal impulse {per_tick} per tick, weight {weight}");
    }

    /// Balanced on an edge but past the tipping point, a cube falls onto a
    /// face. That needs the angular terms of the normal impulse.
    #[test]
    fn a_cube_on_its_edge_tips_onto_a_face() {
        let world = slab(64, 0..8);
        let field = DistanceField::build(&world);
        let tilt = Quat::from_rotation_z(50f32.to_radians());
        let alignment = |q: Quat| [Vec3::X, Vec3::Y, Vec3::Z].iter().map(|a| (q * *a).y.abs()).fold(0.0, f32::max);
        assert!(alignment(tilt) < 0.8, "the start is not on an edge, so this proves nothing");

        let mut bodies = vec![placed(cube(4, 4), Vec3::new(32.0, 11.0, 32.0), tilt)];
        run(&mut bodies, &world, &field, GRAVITY, 640);
        let body = &bodies[0];
        assert!(alignment(body.orientation) > 0.999, "ended at {:?}, not flat", body.orientation);
        let spin = body.mass.inverse_inertia.y_axis.y * body.angular_momentum.length();
        assert!(spin < 0.05 && body.velocity.length() < 0.05, "still moving: spin {spin}, {:?}", body.velocity);
    }

    /// At the speed cap, a body falls onto a floor one voxel thick and does not
    /// pass through it: the speculative margin covers a tick's travel.
    #[test]
    fn a_body_at_the_speed_cap_does_not_tunnel_through_a_thin_floor() {
        let world = slab(64, 10..11);
        let field = DistanceField::build(&world);
        let mut body = placed(cube(4, 4), Vec3::new(32.0, 40.0, 32.0), Quat::IDENTITY);
        body.velocity = Vec3::new(0.0, -MAX_TRAVEL / DT, 0.0);
        let mut bodies = vec![body];
        run(&mut bodies, &world, &field, GRAVITY, 120);
        let y = bodies[0].position.y;
        assert!((y - 13.0).abs() < 0.2, "ended at {y}, not on the floor at 13");
    }

    /// Detection reads the live tree, so erasing the support drops a resting
    /// body on the very next tick.
    #[test]
    fn erasing_the_support_drops_a_resting_body() {
        let mut world = slab(64, 0..8);
        let field = DistanceField::build(&world);
        let mut bodies = vec![placed(cube(4, 4), Vec3::new(32.0, 10.5, 32.0), Quat::IDENTITY)];
        run(&mut bodies, &world, &field, GRAVITY, 300);
        assert!(bodies[0].velocity.y.abs() < 0.05, "not at rest before the edit");
        // Erasing leaves the field under-estimating, which is its safe direction.
        world.apply_sphere(Vec3::new(32.0, 6.0, 32.0), 6.0, MaterialId::EMPTY);
        step(&mut bodies, &world, &field, GRAVITY, DT);
        assert!(bodies[0].velocity.y < -1.0, "still held up: {:?}", bodies[0].velocity);
    }

    /// Warm starting depends on a resting contact keeping its key.
    #[test]
    fn a_resting_body_keeps_its_contact_keys() {
        let world = slab(64, 0..8);
        let field = DistanceField::build(&world);
        let mut bodies = vec![placed(cube(4, 4), Vec3::new(32.0, 10.5, 32.0), Quat::IDENTITY)];
        run(&mut bodies, &world, &field, GRAVITY, 300);
        let keys = |b: &Body| {
            let mut k: Vec<_> = b.warm.keys().map(|(u, w)| (u.to_array(), w.to_array())).collect();
            k.sort_unstable();
            k
        };
        let before = keys(&bodies[0]);
        step(&mut bodies, &world, &field, GRAVITY, DT);
        assert!(!before.is_empty());
        assert_eq!(keys(&bodies[0]), before);
    }

    #[test]
    fn a_body_below_the_world_is_removed() {
        let world = slab(64, 0..8);
        let field = DistanceField::build(&world);
        let mut bodies = vec![placed(cube(4, 4), Vec3::new(32.0, -60.0, 32.0), Quat::IDENTITY)];
        assert!(step(&mut bodies, &world, &field, GRAVITY, DT));
        assert!(bodies.is_empty());
        let mut bodies = vec![placed(cube(4, 4), Vec3::new(32.0, 40.0, 32.0), Quat::IDENTITY)];
        assert!(!step(&mut bodies, &world, &field, GRAVITY, DT));
        assert_eq!(bodies.len(), 1);
    }

    /// A body built with `new` and never recomputed has no mass and is not
    /// simulated.
    #[test]
    fn a_massless_body_does_not_move() {
        let world = slab(64, 0..8);
        let field = DistanceField::build(&world);
        let mut bodies = vec![Body::new(cube(4, 4), Vec3::new(32.0, 40.0, 32.0), Quat::IDENTITY)];
        run(&mut bodies, &world, &field, GRAVITY, 10);
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
            (0..16).map(|i| placed(cube(4, 4), Vec3::new(8.0 + (i % 4) as f32 * 14.0, y, 8.0 + (i / 4) as f32 * 14.0), Quat::IDENTITY)).collect()
        };
        let time = |bodies: &mut Vec<Body>, ticks: u32| -> Vec<f64> {
            (0..ticks)
                .map(|_| {
                    let start = std::time::Instant::now();
                    step(bodies, &world, &field, GRAVITY, DT);
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
        println!("16 bodies: falling {fall:.4} ms/tick, resting {rest:.4} ms/tick (median)");
    }
}
```

- [ ] **Step 2: Run to verify they fail**

Run: `cargo test -p bevox_core solver > t5.txt 2>&1; echo $?`
Expected: non-zero, because `step` is not defined.

- [ ] **Step 3: Implement**

At the top of `solver.rs`:

```rust
//! Temporal Gauss-Seidel. Contacts are detected once per tick; then each
//! substep applies gravity, warm starts, solves the velocity constraints,
//! integrates and relaxes.
//!
//! The detect-once-then-substep shape and warm starting by voxel pair are from
//! Dwyer's devlog #26. The unbiased relax solve after integrating is from
//! Erin Catto's soft-step solver (Box2D v3): it removes the velocity the push-out
//! bias added, so pushing a body out of the floor does not make it bounce.

use super::contact::{Contact, RADIUS, detect, world_box};
use super::{BASE_MARGIN, BIAS, MAX_PUSH, MAX_TRAVEL, SLOP, SUBSTEPS};
use crate::body::{Body, occupied_bounds};
use crate::contree::Contree;
use crate::distance_field::DistanceField;
use glam::{Mat3, Quat, Vec3};

/// Advances every body by one tick of `dt` seconds against the static world.
///
/// Bodies entirely below the world are removed first. Returns whether any
/// were: the body list changed, so the caller must rebuild what is packed from
/// it. A body with no mass is not simulated.
pub fn step(bodies: &mut Vec<Body>, tree: &Contree, field: &DistanceField, gravity: Vec3, dt: f32) -> bool {
    let before = bodies.len();
    bodies.retain(|b| world_box(b, 0.0).is_none_or(|(_, max)| max.y >= 0.0));
    for body in bodies.iter_mut().filter(|b| b.mass.mass > 0.0) {
        step_body(body, tree, field, gravity, dt);
    }
    bodies.len() != before
}

fn step_body(body: &mut Body, tree: &Contree, field: &DistanceField, gravity: Vec3, dt: f32) {
    let h = dt / SUBSTEPS as f32;
    let inv_h = 1.0 / h;
    let inv_mass = body.mass.inverse_mass();
    let radius = radius(body);

    let travel = cap_speed(body, world_inverse_inertia(body), radius, dt);
    let contacts = detect(body, tree, field, travel + BASE_MARGIN);
    let mut impulses: Vec<f32> = contacts.iter().map(|c| body.warm.get(&c.key).copied().unwrap_or(0.0)).collect();

    for _ in 0..SUBSTEPS {
        body.velocity += gravity * h;
        let inv_inertia = world_inverse_inertia(body);
        cap_speed(body, inv_inertia, radius, dt);

        for (c, &impulse) in contacts.iter().zip(&impulses) {
            push(body, inv_mass, c, impulse);
        }
        for (c, impulse) in contacts.iter().zip(impulses.iter_mut()) {
            solve(body, inv_mass, inv_inertia, c, impulse, inv_h, true);
        }

        integrate(body, inv_inertia, h);

        let inv_inertia = world_inverse_inertia(body);
        for (c, impulse) in contacts.iter().zip(impulses.iter_mut()) {
            solve(body, inv_mass, inv_inertia, c, impulse, inv_h, false);
        }
    }

    body.warm = contacts.iter().zip(&impulses).map(|(c, &i)| (c.key, i)).collect();
}

/// The inverse inertia tensor in world axes.
fn world_inverse_inertia(body: &Body) -> Mat3 {
    let r = Mat3::from_quat(body.orientation);
    r * body.mass.inverse_inertia * r.transpose()
}

/// How far the body's furthest voxel lies from its centre of mass.
fn radius(body: &Body) -> f32 {
    let Some((lo, hi)) = occupied_bounds(&body.volume) else {
        return RADIUS;
    };
    (lo.as_vec3() - body.com).abs().max((hi.as_vec3() - body.com).abs()).length()
}

/// Scales velocity and angular momentum together, so that a tick's travel
/// (linear plus the furthest voxel's swing) is at most `MAX_TRAVEL`. Returns
/// that travel.
fn cap_speed(body: &mut Body, inv_inertia: Mat3, radius: f32, dt: f32) -> f32 {
    let spin = (inv_inertia * body.angular_momentum).length();
    let travel = (body.velocity.length() + spin * radius) * dt;
    if travel > MAX_TRAVEL {
        let scale = MAX_TRAVEL / travel;
        body.velocity *= scale;
        body.angular_momentum *= scale;
        return MAX_TRAVEL;
    }
    travel
}

/// The contact point's offset from the centre of mass, in world axes.
fn lever(body: &Body, c: &Contact) -> Vec3 {
    body.orientation * c.anchor
}

/// Applies a normal impulse at the contact point.
fn push(body: &mut Body, inv_mass: f32, c: &Contact, impulse: f32) {
    let p = c.normal * impulse;
    body.velocity += p * inv_mass;
    body.angular_momentum += lever(body, c).cross(p);
}

/// One sequential-impulse iteration on one contact.
///
/// The accumulated impulse never goes negative: contacts push, never pull.
/// A contact that is still apart lets the body approach by exactly the gap.
/// A penetrating one, with `use_bias`, pushes the body out by a fraction of
/// the depth beyond the slop.
fn solve(body: &mut Body, inv_mass: f32, inv_inertia: Mat3, c: &Contact, accumulated: &mut f32, inv_h: f32, use_bias: bool) {
    let r = lever(body, c);
    let omega = inv_inertia * body.angular_momentum;
    let vn = (body.velocity + omega.cross(r)).dot(c.normal);
    let k = inv_mass + (inv_inertia * r.cross(c.normal)).cross(r).dot(c.normal);

    // The gap at detection, moved by how far the contact point has travelled
    // along the normal since. The world does not move.
    let separation = c.separation + (body.position + r - c.world_point).dot(c.normal);
    let bias = if separation > 0.0 {
        separation * inv_h
    } else if use_bias {
        (BIAS * (separation + SLOP).min(0.0) * inv_h).max(-MAX_PUSH)
    } else {
        0.0
    };

    let total = (*accumulated - (vn + bias) / k).max(0.0);
    let delta = total - *accumulated;
    *accumulated = total;
    push(body, inv_mass, c, delta);
}

/// Moves and turns the body by one substep.
///
/// `ω` is in world axes, so the spin quaternion multiplies on the left. Dwyer
/// lost three weeks to the other order.
fn integrate(body: &mut Body, inv_inertia: Mat3, h: f32) {
    body.position += body.velocity * h;
    let omega = inv_inertia * body.angular_momentum;
    let spin = Quat::from_xyzw(omega.x, omega.y, omega.z, 0.0) * body.orientation;
    body.orientation = (body.orientation + spin * (0.5 * h)).normalize();
}
```

In `mod.rs`, add `pub mod solver;`.

- [ ] **Step 4: Run to verify they pass**

Run: `cargo test -p bevox_core solver > t5.txt 2>&1; echo $?`
Expected: `0`.

**If `a_body_at_rest_stays_at_rest` or `a_cube_on_its_edge_tips_onto_a_face` fails,** tune `SLOP`, `BIAS`, `MAX_PUSH` or `SUBSTEPS` in `mod.rs`, one at a time, and record each value tried and its result in the plan's Measurements section. **Never loosen the gates' numbers.** If no setting passes, stop and report the failing numbers: the solver design itself is then in question.

**If `a_body_at_the_speed_cap_does_not_tunnel_through_a_thin_floor` fails,** look first at `detect`'s reach at `margin = MAX_TRAVEL + BASE_MARGIN`.

- [ ] **Step 5: Break checks**

Run each break with `cargo test -p bevox_core <test> > b5-N.txt 2>&1; echo $?`, confirm FAIL, then restore:

1. In `integrate`, swap the product to `body.orientation * Quat::from_xyzw(...)`. `a_spin_turns_about_the_world_axis_it_names` must FAIL.
2. In `push`, delete `body.angular_momentum += ...`. `a_cube_on_its_edge_tips_onto_a_face` must FAIL.
3. In `step_body`, detect with `BASE_MARGIN` alone instead of `travel + BASE_MARGIN`. `a_body_at_the_speed_cap_does_not_tunnel_through_a_thin_floor` must FAIL.
   - If it passes, the fall's phase happens to land a corner inside the floor voxel. Change the start height in the test by 0.5, re-run the break until it FAILS, and keep that height in the test.
4. In `step_body`, move `integrate(body, inv_inertia, h);` above the biased solve loop. This is the integrate-before-solve order Dwyer's devlog #26 says jitters. `a_body_at_rest_stays_at_rest` must FAIL.
   - If it passes, say so in the Measurements section. The gate then does not detect that ordering, which is a finding; do not weaken anything to hide it.
5. In `momentum_is_conserved_exactly_in_free_flight`'s code path, add `body.angular_momentum = world_inverse_inertia(body).inverse() * (inv_inertia * body.angular_momentum);` at the end of `integrate`. That recomputes momentum from spin through a float round trip. The test must FAIL.
6. In `step`, detect against a tree captured on the first call. Simplest: in the test only, pass the pre-erase clone of `world` to the post-erase step. `erasing_the_support_drops_a_resting_body` must FAIL.

After restoring everything, `cargo test --workspace > t5.txt 2>&1; echo $?` must return `0` with no warnings.

- [ ] **Step 6: Commit**

```bash
git add crates docs
git commit -m "feat(core): a temporal Gauss-Seidel solver, so bodies land, tumble and rest" -m "Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>"
```

---

### Task 6: The app: physics each tick, and dropping bodies

**Files:**
- Modify: `crates/bevox/src/main.rs`
- Modify: this plan (Measurements)

**Interfaces:**
- Consumes: `physics::solver::step`, `physics::GRAVITY`, `Body::recompute`

- [ ] **Step 1: Write the failing test**

Replace `a_spun_body_stays_unit_length_and_in_place` in `main.rs`'s tests with:

```rust
    /// The system steps the scene's bodies, and a body leaving the world bumps
    /// the generation, because the packed buffers hold the body list.
    #[test]
    fn the_physics_system_moves_bodies_and_rebuilds_when_one_leaves() {
        let mut world = World::new();
        let mut time = Time::<()>::default();
        time.advance_by(std::time::Duration::from_secs_f64(1.0 / 64.0));
        world.insert_resource(time);
        let (tree, materials) = demo_scene();
        let field = DistanceField::build(&tree);
        let mut falling = demo_body(Vec3::new(20.0, 40.0, 20.0), Quat::IDENTITY);
        assert!(falling.recompute(&materials));
        let mut gone = demo_body(Vec3::new(20.0, -100.0, 20.0), Quat::IDENTITY);
        assert!(gone.recompute(&materials));
        world.insert_resource(VoxelScene {
            tree,
            materials,
            generation: 1,
            field,
            field_dirty: None,
            bodies: vec![falling, gone],
        });

        let physics = world.register_system(physics_system);
        world.run_system(physics).unwrap();

        let scene = world.resource::<VoxelScene>();
        assert_eq!(scene.bodies.len(), 1, "the body below the world was not removed");
        assert_eq!(scene.generation, 2, "removing a body did not ask for a rebuild");
        assert!(scene.bodies[0].velocity.y < 0.0, "the body did not start to fall");
    }
```

- [ ] **Step 2: Run to verify it fails**

Run: `cargo test -p bevox > t6.txt 2>&1; echo $?`
Expected: non-zero, because `physics_system` does not exist and `demo_body` takes one argument.

- [ ] **Step 3: Implement**

In `main.rs`:

1. Add the imports `use bevox_core::physics::GRAVITY;`, `use bevox_core::physics::solver::step;`, and `EulerRot` from `bevy::math` if the prelude does not already bring it in.
2. Delete `spin_bodies` and its doc comment, and its `.add_systems(Update, spin_bodies.before(build_gpu_scene))` line with the comment above it.
3. Add these systems in `main`, where the spin was:

   ```rust
           // Fixed timestep: physics must not depend on the frame rate, and
           // Bevy runs `FixedUpdate` before `Update`, so a removal's generation
           // bump lands before `build_gpu_scene`.
           .add_systems(FixedUpdate, physics_system)
           .add_systems(Update, drop_body_input.after(fly_camera_system).before(build_gpu_scene))
   ```

4. Replace `demo_body`:

   ```rust
   /// A six-voxel cube of material 2 -- brick, in the demo palette -- whose
   /// middle is at `centre`, turned by `orientation`. Call `recompute` before
   /// simulating it.
   ///
   /// Off the 4-voxel brick grid (5..11 in a 16 volume) so its bricks are partial
   /// and it owns real voxel bytes, like the body the parity tests prove, rather
   /// than collapsing to uniform nodes.
   fn demo_body(centre: Vec3, orientation: Quat) -> Body {
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
   ```

5. In `setup`, replace the body lines. Keep the placement comment, but change its last sentence to say the body now falls:

   ```rust
       let mut body = demo_body(eye + forward * 14.0 + right * 6.0, Quat::IDENTITY);
       body.recompute(&materials);
   ```

   `body` must be computed before `materials` moves into `VoxelScene`, which it already is.

6. Add the two systems:

   ```rust
   /// One physics tick for every body.
   fn physics_system(time: Res<Time>, mut scene: ResMut<VoxelScene>) {
       let scene = &mut *scene;
       if step(&mut scene.bodies, &scene.tree, &scene.field, GRAVITY, time.delta_secs()) {
           // A body left the world. The body list is packed into the scene
           // buffers, so they must be rebuilt.
           scene.generation += 1;
       }
   }

   /// `F` drops a tilted cube in front of the camera, to watch landing and
   /// settling again and again.
   fn drop_body_input(
       keys: Res<ButtonInput<KeyCode>>,
       camera: Query<&GlobalTransform, With<Camera3d>>,
       mut scene: ResMut<VoxelScene>,
   ) {
       if !keys.just_pressed(KeyCode::KeyF) {
           return;
       }
       let Ok(transform) = camera.single() else {
           return;
       };
       let tilt = Quat::from_euler(EulerRot::XYZ, 0.4, 0.7, 0.2);
       let mut body = demo_body(transform.translation() + transform.forward().as_vec3() * 16.0, tilt);
       if body.recompute(&scene.materials) {
           scene.bodies.push(body);
           // A new body adds geometry to the packed buffers.
           scene.generation += 1;
       }
   }
   ```

- [ ] **Step 4: Run to verify it passes**

Run: `cargo test --workspace > t6.txt 2>&1; echo $?`
Expected: `0`, with no warnings.

- [ ] **Step 5: Break check**

In `physics_system`, delete `scene.generation += 1;`. Run `cargo test -p bevox > b6.txt 2>&1; echo $?`: expected non-zero, and the test FAILS on the generation. Restore it and re-run: `0`.

- [ ] **Step 6: Measure**

Run: `cargo test --release -p bevox_core a_tick_is_timed -- --ignored --nocapture --test-threads=1 > m6.txt 2>&1; echo $?`

Add a `## Measurements` section to this plan with:
- the printed line, verbatim;
- the date and the CPU;
- a note that this is CPU time for 16 bodies against a 64-extent slab, not a frame time, and makes no comparison, so no A/B/A applies;
- every tuning value tried in Task 5, and its result.

- [ ] **Step 7: Commit**

```bash
git add crates docs
git commit -m "feat(app): simulate bodies each fixed tick, and drop one with F" -m "Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>"
```

- [ ] **Step 8: Hand over**

Tell Flori how to try it: `cargo run -p bevox`. The startup cube falls. `F` drops a tilted cube in front of the camera. Erasing the ground under a resting cube (right click) drops it. Do not merge or push.

---

## Milestone check

A body dropped into the scene lands on real voxel terrain, tumbles onto a face if it lands off balance, stays at rest without drifting over ten thousand ticks, and falls when the ground under it is erased. Every gate above has been proven by a deliberate break.

## What this plan deliberately does not do

- **No friction or restitution** (milestone 3). A tipping cube slides as it falls, because nothing holds its edge in place.
- **No body against body** (milestone 3). Bodies pass through each other.
- **No detachment, body editing or splitting** (milestone 4). `recompute` exists and is tested for it, but nothing edits a body yet.
- **No joints and no grab** (milestone 5). Picking still sees only the world, so clicking a body edits the world behind it.
- **No sleeping, no merging back into terrain, no multithreading.** Measure first.
- **No render interpolation.** A 64 Hz tick on a 60 Hz display shows slight judder.
