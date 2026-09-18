# Terrain Detachment Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking. **Flori prefers inline execution for this project.**

**Goal:** Carve the support out from under a wall and the wall falls. Erasing terrain that disconnects a piece turns that piece into a rigid body.

**Architecture:** After an erase, a search starts from the solid voxels around the hole and walks face neighbours. A piece that never reaches the world's floor, and fits inside a budget, is disconnected: its voxels are removed from the world through the ordinary edit path, and a body is built from them. Everything else is left alone. The search is on voxels, bounded by a budget, rather than on the tree's uniform nodes.

**Tech Stack:** Rust stable 1.96, Bevy 0.19.1, glam 0.32. No new dependencies.

**Spec:** `docs/superpowers/specs/2026-09-15-bevox-rigid-bodies-design.md`, milestone 4's first half. Brush editing of bodies is the second half and is **not** in this plan.

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

- `Contree::apply_sphere(centre, radius, material)`, the only edit. It rewrites the subtrees it intersects, frees what it replaces, leaves the tree canonical, and marks arena ranges dirty so the render world uploads only the delta.
- `Contree::voxels()`, `Contree::get(p)`, `Contree::from_voxels(extent, &[(UVec3, MaterialId)])`.
- `physics::classify::solid_at(tree, p)`.
- `Body::new(volume, position, orientation)` and `Body::recompute(&materials)`, which moves the pivot to the centre of mass without moving any voxel.
- `physics::solver::step(bodies, tree, field, materials, gravity, dt)`, with body-against-body contact.
- `upload::apply_brush(scene, centre, radius, material)`, which the app's right click calls to erase.
- `MAX_BODIES = 16` in `bevox_render::pipeline`: past it, bodies are uploaded but not marched.

## Facts that are not obvious from the code

**Removing the piece must go through the edit path, not around it.** The arena's dirty ranges are what the render world uploads. A detached piece that vanished from the tree by any other route would keep drawing until the next full rebuild. `apply_sphere` is the only edit today, so this plan generalises the machinery behind it rather than adding a second one.

**"Disconnected" needs an anchor, and the anchor is the world's floor.** A piece is grounded when it contains a voxel at `y == 0`. Nothing else in the scene is distinguished, and a rule like "the biggest piece stays" would drop a mountain the moment a pebble outweighed a slice of it.

**The search is bounded, and the bound is conservative in the safe direction.** A cut into a mountainside would otherwise walk the whole mountain. A search that exceeds `BUDGET` voxels gives up and calls the piece grounded, so nothing is detached that should not be. The spec calls for a walk over uniform nodes instead, which is Dwyer's approach and much faster on big volumes; this plan updates the spec, keeps the simple walk, and leaves the node walk to whenever the measurement asks for it.

**Never remove geometry that cannot be drawn.** The renderer marches at most `MAX_BODIES` bodies. If a detachment would push the scene past that, the piece stays in the world where it is still drawn, frozen. A piece that vanished because the cap was full would look like a bug.

**A detached body's voxels must not move.** The piece is built at the world position of its bounding-box corner, and `recompute` then shifts the pivot to the centre of mass without moving any voxel. The gate checks the voxels' world positions before and after.

**A volume's extent is a power of four.** The piece's box is rarely one, so the volume is the next power of four that fits, and the voxels sit at the box's corner inside it.

**Erasing leaves the distance field under-estimating, which is its safe direction,** so no field work is needed here. `apply_brush` already skips the field on an erase for the same reason.

## File Structure

- `crates/bevox_core/src/edit.rs`: the edit machinery becomes generic over the operation, and gains `Contree::clear_voxels`.
- `crates/bevox_core/src/physics/detach.rs` (**new**): the search and the body building.
- `crates/bevox_core/src/physics/mod.rs`: declare the module, and `BUDGET`.
- `crates/bevox_render/src/upload.rs`: `apply_brush` returns what it erased, so the app can detach from it.
- `crates/bevox/src/main.rs`: the erase path detaches, and the demo column is worth cutting.

---

### Task 1: Erasing an arbitrary set of voxels

**Files:**
- Modify: `crates/bevox_core/src/edit.rs`

**Interfaces:**
- Produces: `Contree::clear_voxels(&mut self, voxels: &[UVec3])`
- Keeps: `Contree::apply_sphere` unchanged in behaviour and signature

- [ ] **Step 1: Write the failing tests**

In `edit.rs`'s test module:

```rust
    /// Clearing takes exactly the voxels named and leaves the rest alone.
    #[test]
    fn clear_voxels_removes_only_what_it_is_given() {
        let mut dense = DenseVolume::new(16).unwrap();
        for z in 0..8 {
            for y in 0..8 {
                for x in 0..8 {
                    dense.set(UVec3::new(x, y, z), MaterialId(1));
                }
            }
        }
        let mut tree = dense.into_contree();
        let gone: Vec<UVec3> = (0..8).map(|x| UVec3::new(x, 3, 3)).collect();
        tree.clear_voxels(&gone);

        for p in &gone {
            assert!(tree.get(*p).is_empty(), "{p:?} survived");
        }
        assert_eq!(tree.voxels().len(), 8 * 8 * 8 - gone.len());
        assert!(tree.check_canonical().is_ok(), "the tree is no longer canonical");
    }

    /// The edit goes through the arena, so the render world can upload a delta
    /// rather than rebuilding: a body that vanished any other way would keep
    /// being drawn.
    #[test]
    fn clear_voxels_marks_the_arena_dirty() {
        let mut dense = DenseVolume::new(16).unwrap();
        for x in 0..8 {
            dense.set(UVec3::new(x, 0, 0), MaterialId(1));
        }
        let mut tree = dense.into_contree();
        tree.arena_mut().clear_dirty();
        tree.clear_voxels(&[UVec3::new(3, 0, 0)]);
        let dirty = tree.arena().dirty_nodes().len() + tree.arena().dirty_voxels().len();
        assert!(dirty > 0, "nothing was marked dirty");
    }

    /// Clearing nothing changes nothing.
    #[test]
    fn clear_voxels_of_nothing_is_a_no_op() {
        let mut tree = Contree::from_voxels(16, &[(UVec3::new(1, 2, 3), MaterialId(1))]);
        tree.clear_voxels(&[]);
        assert_eq!(tree.voxels().len(), 1);
    }
```

- [ ] **Step 2: Run to verify they fail**

Run: `cargo test -p bevox_core clear_voxels > b.txt 2>&1; echo $?`
Expected: non-zero; `clear_voxels` does not exist.

- [ ] **Step 3: Implement**

Read `edit.rs` first: `rewrite`, `rewrite_leaf` and `SphereOp` are what this generalises. The operation becomes a trait, and `SphereOp` one implementation:

```rust
/// What an edit does to the voxels it covers.
///
/// One trait rather than a second copy of the tree rewrite: every edit has to
/// free the nodes it replaces, leave the tree canonical, and mark the arena
/// dirty so the render world can upload a delta.
pub(crate) trait EditOp {
    /// Whether the edit touches the cube of `extent` voxels at `origin`. A node
    /// it does not touch is returned as it was.
    fn touches(&self, origin: UVec3, extent: u32) -> bool;

    /// What the voxel at `p` becomes: `None` leaves it as it is.
    fn material_at(&self, p: UVec3) -> Option<MaterialId>;
}
```

`SphereOp` implements it with its existing `intersects` and `covers_voxel`:

```rust
impl EditOp for SphereOp {
    fn touches(&self, origin: UVec3, extent: u32) -> bool {
        self.intersects(origin, extent)
    }

    fn material_at(&self, p: UVec3) -> Option<MaterialId> {
        self.covers_voxel(p).then_some(self.material)
    }
}
```

The new operation erases a set:

```rust
/// Erases exactly the voxels it was given.
struct ClearOp {
    voxels: HashSet<UVec3>,
    lo: UVec3,
    hi: UVec3,
}

impl EditOp for ClearOp {
    fn touches(&self, origin: UVec3, extent: u32) -> bool {
        let hi = origin + UVec3::splat(extent - 1);
        origin.cmple(self.hi).all() && hi.cmpge(self.lo).all()
    }

    fn material_at(&self, p: UVec3) -> Option<MaterialId> {
        self.voxels.contains(&p).then_some(MaterialId::EMPTY)
    }
}
```

Change `rewrite` and `rewrite_leaf` to take `op: &dyn EditOp` (or a generic `op: &O`), replacing `op.intersects(..)` with `op.touches(..)` and the leaf's `covers_voxel` decision with `material_at`. **Read the existing leaf code and keep its structure**: it decides per voxel what the new material is, which `material_at` now answers, with `None` meaning "unchanged".

Then:

```rust
impl Contree {
    /// Erases exactly `voxels`. Used by detachment, which moves a piece of the
    /// world into a body and must take the same voxels out of the world.
    pub fn clear_voxels(&mut self, voxels: &[UVec3]) {
        let Some(&first) = voxels.first() else {
            return;
        };
        let mut lo = first;
        let mut hi = first;
        for p in voxels {
            lo = lo.min(*p);
            hi = hi.max(*p);
        }
        let op = ClearOp { voxels: voxels.iter().copied().collect(), lo, hi };
        let root = self.root();
        let new_root = self.rewrite(root, self.depth() - 1, UVec3::ZERO, &op);
        self.set_root(new_root);
    }
}
```

- [ ] **Step 4: Run to verify they pass**

Run: `cargo test --workspace > t.txt 2>&1; echo $?`
Expected: `0`, with no warnings. Every existing `apply_sphere` test must still pass: the sphere path is the same rewrite through a different operation.

- [ ] **Step 5: Break checks**

1. In `ClearOp::touches`, return `false` always. `clear_voxels_removes_only_what_it_is_given` must FAIL.
2. In `ClearOp::material_at`, return `Some(MaterialId::EMPTY)` regardless of membership. The same test must FAIL, on the count of what survived.

Restore each and re-run.

- [ ] **Step 6: Commit**

```bash
git add crates
git commit -m "feat(core): erase an arbitrary set of voxels through the edit path" -m "Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>"
```

---

### Task 2: Finding the disconnected piece

**Files:**
- Create: `crates/bevox_core/src/physics/detach.rs`
- Modify: `crates/bevox_core/src/physics/mod.rs`

**Interfaces:**
- Consumes: `solid_at`, `Contree::get`
- Produces:
  - `pub const BUDGET: usize = 20_000;` in `physics`
  - `pub fn loose_pieces(tree: &Contree, lo: IVec3, hi: IVec3, budget: usize) -> Vec<Vec<UVec3>>`, the disconnected pieces touching the box `lo..hi`, each sorted

- [ ] **Step 1: Write the failing tests**

```rust
#[cfg(test)]
mod tests {
    use super::*;
    use crate::material::MaterialId;
    use glam::UVec3;

    /// A floor with a column on it. Cutting the column's base leaves the rest of
    /// the column reaching nothing: that is the piece that falls.
    fn floor_and_column() -> Contree {
        let mut voxels = Vec::new();
        for z in 0..16 {
            for x in 0..16 {
                voxels.push((UVec3::new(x, 0, z), MaterialId(1)));
            }
        }
        for y in 1..10 {
            voxels.push((UVec3::new(8, y, 8), MaterialId(2)));
        }
        Contree::from_voxels(16, &voxels)
    }

    #[test]
    fn cutting_a_column_frees_what_is_above_the_cut() {
        let mut tree = floor_and_column();
        tree.clear_voxels(&[UVec3::new(8, 1, 8), UVec3::new(8, 2, 8)]);
        let pieces = loose_pieces(&tree, IVec3::new(7, 0, 7), IVec3::new(9, 3, 9), BUDGET);
        assert_eq!(pieces.len(), 1, "{pieces:?}");
        assert_eq!(pieces[0].len(), 7, "the freed column is 7 voxels: {:?}", pieces[0]);
        assert!(pieces[0].iter().all(|p| p.y >= 3), "the floor came along: {:?}", pieces[0]);
    }

    /// A cut that leaves everything still standing on the floor frees nothing.
    #[test]
    fn a_notch_frees_nothing() {
        let mut tree = floor_and_column();
        tree.clear_voxels(&[UVec3::new(8, 5, 8)]);
        // The column above the notch is now free; below it is still grounded.
        let pieces = loose_pieces(&tree, IVec3::new(7, 4, 7), IVec3::new(9, 6, 9), BUDGET);
        assert_eq!(pieces.len(), 1);
        assert!(pieces[0].iter().all(|p| p.y > 5));

        // Whereas a hole in the floor frees nothing at all.
        let mut tree = floor_and_column();
        tree.clear_voxels(&[UVec3::new(2, 0, 2)]);
        assert!(loose_pieces(&tree, IVec3::new(1, 0, 1), IVec3::new(3, 1, 3), BUDGET).is_empty());
    }

    /// Two pieces cut free at once are two pieces.
    #[test]
    fn two_cuts_free_two_pieces() {
        let mut voxels = Vec::new();
        for z in 0..16 {
            for x in 0..16 {
                voxels.push((UVec3::new(x, 0, z), MaterialId(1)));
            }
        }
        for (x, z) in [(4, 4), (12, 12)] {
            for y in 1..5 {
                voxels.push((UVec3::new(x, y, z), MaterialId(2)));
            }
        }
        let mut tree = Contree::from_voxels(16, &voxels);
        tree.clear_voxels(&[UVec3::new(4, 1, 4), UVec3::new(12, 1, 12)]);
        let pieces = loose_pieces(&tree, IVec3::new(3, 0, 3), IVec3::new(13, 2, 13), BUDGET);
        assert_eq!(pieces.len(), 2, "{pieces:?}");
        for piece in &pieces {
            assert_eq!(piece.len(), 3);
        }
    }

    /// Past the budget the search gives up and calls the piece grounded, which
    /// is the safe direction: nothing is detached that should not be.
    #[test]
    fn a_piece_past_the_budget_is_left_alone() {
        let mut voxels = Vec::new();
        // A slab floating with no floor at all: every voxel is "loose", and
        // there are more of them than a small budget allows.
        for z in 0..16 {
            for y in 4..8 {
                for x in 0..16 {
                    voxels.push((UVec3::new(x, y, z), MaterialId(1)));
                }
            }
        }
        let tree = Contree::from_voxels(16, &voxels);
        assert!(loose_pieces(&tree, IVec3::new(0, 4, 0), IVec3::new(2, 6, 2), 100).is_empty());
        assert_eq!(
            loose_pieces(&tree, IVec3::new(0, 4, 0), IVec3::new(2, 6, 2), BUDGET).len(),
            1,
            "with room to finish, the slab is one loose piece"
        );
    }

    /// Diagonals do not hold a piece up: only faces join voxels, as in Dwyer's
    /// devlog #12.
    #[test]
    fn a_diagonal_touch_does_not_count_as_joined() {
        let mut voxels = Vec::new();
        for x in 0..4 {
            voxels.push((UVec3::new(x, 0, 0), MaterialId(1)));
        }
        voxels.push((UVec3::new(4, 1, 1), MaterialId(2)));
        let tree = Contree::from_voxels(4, &voxels);
        let pieces = loose_pieces(&tree, IVec3::new(3, 0, 0), IVec3::new(4, 1, 1), BUDGET);
        assert_eq!(pieces.len(), 1);
        assert_eq!(pieces[0], vec![UVec3::new(4, 1, 1)]);
    }
}
```

- [ ] **Step 2: Run to verify they fail**

Run: `cargo test -p bevox_core detach > b.txt 2>&1; echo $?`
Expected: non-zero; the module does not exist.

- [ ] **Step 3: Implement**

`crates/bevox_core/src/physics/detach.rs`:

```rust
//! Terrain that an edit cuts free becomes a body.
//!
//! After Dwyer's devlog #12: a depth-first search over face neighbours, where a
//! piece that cannot reach the ground is no longer part of the world. His search
//! walks the tree's uniform nodes, which is much faster on a large volume; this
//! one walks voxels and gives up past a budget. See the spec's Detachment
//! section for why, and what it would take to change.

use super::BUDGET;
use crate::contree::Contree;
use crate::physics::classify::solid_at;
use glam::{IVec3, UVec3};
use std::collections::HashSet;

const NEIGHBOURS: [IVec3; 6] = [
    IVec3::X,
    IVec3::NEG_X,
    IVec3::Y,
    IVec3::NEG_Y,
    IVec3::Z,
    IVec3::NEG_Z,
];

/// The pieces of `tree` around the box `lo..=hi` that no longer reach the
/// ground, each sorted.
///
/// A piece is grounded when it holds a voxel at `y == 0`, or when it is larger
/// than `budget`, at which point the search gives up rather than walking a
/// mountain. Both leave the piece in the world, which is the safe direction.
pub fn loose_pieces(tree: &Contree, lo: IVec3, hi: IVec3, budget: usize) -> Vec<Vec<UVec3>> {
    let mut seen: HashSet<IVec3> = HashSet::new();
    let mut pieces = Vec::new();
    for z in lo.z..=hi.z {
        for y in lo.y..=hi.y {
            for x in lo.x..=hi.x {
                let seed = IVec3::new(x, y, z);
                if !solid_at(tree, seed) || seen.contains(&seed) {
                    continue;
                }
                if let Some(piece) = walk(tree, seed, budget, &mut seen) {
                    pieces.push(piece);
                }
            }
        }
    }
    pieces
}

/// Walks the piece holding `seed`. `None` when it is grounded or too big; the
/// voxels it visited are marked seen either way, so no piece is walked twice.
fn walk(
    tree: &Contree,
    seed: IVec3,
    budget: usize,
    seen: &mut HashSet<IVec3>,
) -> Option<Vec<UVec3>> {
    let mut stack = vec![seed];
    let mut piece = Vec::new();
    seen.insert(seed);
    let mut grounded = false;
    while let Some(p) = stack.pop() {
        if p.y == 0 {
            grounded = true;
        }
        piece.push(p.as_uvec3());
        if piece.len() > budget {
            return None;
        }
        for step in NEIGHBOURS {
            let n = p + step;
            if solid_at(tree, n) && seen.insert(n) {
                stack.push(n);
            }
        }
    }
    if grounded {
        return None;
    }
    piece.sort_unstable_by_key(|p| (p.z, p.y, p.x));
    Some(piece)
}
```

In `physics/mod.rs`, add the module and:

```rust
/// The most voxels a detachment search will walk before giving up.
///
/// A cut into a mountainside would otherwise walk the mountain. Giving up calls
/// the piece grounded, so a piece larger than this stays in the world rather
/// than becoming a body that could not be afforded anyway.
pub const BUDGET: usize = 20_000;
```

- [ ] **Step 4: Run to verify they pass**

Run: `cargo test -p bevox_core detach > t.txt 2>&1; echo $?`
Expected: `0`.

- [ ] **Step 5: Break checks**

1. Add the eight diagonal steps to `NEIGHBOURS` (for example `IVec3::new(1, 1, 0)`). `a_diagonal_touch_does_not_count_as_joined` must FAIL.
2. Drop the `grounded` check (always `false`). `a_notch_frees_nothing`'s floor case must FAIL.
3. Drop the budget check. `a_piece_past_the_budget_is_left_alone` must FAIL.

Restore each and re-run.

- [ ] **Step 6: Commit**

```bash
git add crates
git commit -m "feat(core): find the pieces an edit cuts free" -m "Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>"
```

---

### Task 3: Turning a piece into a body

**Files:**
- Modify: `crates/bevox_core/src/physics/detach.rs`

**Interfaces:**
- Consumes: `loose_pieces`, `Contree::clear_voxels`, `Body::recompute`
- Produces: `pub fn detach(tree: &mut Contree, materials: &MaterialTable, lo: IVec3, hi: IVec3, room: usize) -> Vec<Body>`

- [ ] **Step 1: Write the failing tests**

```rust
    /// The freed column leaves the world and arrives as a body, with its voxels
    /// in the same places and its mass computed.
    #[test]
    fn a_freed_piece_becomes_a_body_where_it_stood() {
        let materials = crate::physics::fixtures::materials();
        let mut tree = floor_and_column();
        tree.clear_voxels(&[UVec3::new(8, 1, 8), UVec3::new(8, 2, 8)]);
        let bodies = detach(&mut tree, &materials, IVec3::new(7, 0, 7), IVec3::new(9, 3, 9), 16);

        assert_eq!(bodies.len(), 1);
        let body = &bodies[0];
        assert!(body.mass.mass > 0.0, "the body has no mass");
        for y in 3..10 {
            assert!(tree.get(UVec3::new(8, y, 8)).is_empty(), "voxel {y} is still in the world");
        }
        assert!(!tree.get(UVec3::new(8, 0, 8)).is_empty(), "the floor went with it");

        let mut world: Vec<[i32; 3]> = body
            .volume
            .voxels()
            .iter()
            .map(|(p, _)| {
                body.world_from_local().transform_point3(p.as_vec3() + 0.5).round().as_ivec3().to_array()
            })
            .collect();
        world.sort_unstable();
        let want: Vec<[i32; 3]> = (3..10).map(|y| [8, y, 8]).collect();
        assert_eq!(world, want, "the piece moved when it became a body");
    }

    /// With no room for another body, the piece stays in the world: geometry
    /// that cannot be drawn must not disappear.
    #[test]
    fn a_piece_stays_in_the_world_when_there_is_no_room() {
        let materials = crate::physics::fixtures::materials();
        let mut tree = floor_and_column();
        tree.clear_voxels(&[UVec3::new(8, 1, 8), UVec3::new(8, 2, 8)]);
        let bodies = detach(&mut tree, &materials, IVec3::new(7, 0, 7), IVec3::new(9, 3, 9), 0);
        assert!(bodies.is_empty());
        assert!(!tree.get(UVec3::new(8, 5, 8)).is_empty(), "the column vanished with nowhere to go");
    }

    /// The biggest pieces go first when there is not room for all of them.
    #[test]
    fn the_biggest_pieces_go_first() {
        let mut voxels = Vec::new();
        for z in 0..16 {
            for x in 0..16 {
                voxels.push((UVec3::new(x, 0, z), MaterialId(1)));
            }
        }
        for y in 1..3 {
            voxels.push((UVec3::new(4, y, 4), MaterialId(2)));
        }
        for y in 1..8 {
            voxels.push((UVec3::new(12, y, 12), MaterialId(2)));
        }
        let mut tree = Contree::from_voxels(16, &voxels);
        tree.clear_voxels(&[UVec3::new(4, 1, 4), UVec3::new(12, 1, 12)]);
        let materials = crate::physics::fixtures::materials();
        let bodies = detach(&mut tree, &materials, IVec3::new(3, 0, 3), IVec3::new(13, 2, 13), 1);
        assert_eq!(bodies.len(), 1);
        assert_eq!(bodies[0].volume.voxels().len(), 6, "the shorter column was taken instead");
        assert!(!tree.get(UVec3::new(4, 2, 4)).is_empty(), "the shorter column vanished");
    }
```

- [ ] **Step 2: Run to verify they fail**

Run: `cargo test -p bevox_core detach > b.txt 2>&1; echo $?`
Expected: non-zero; `detach` does not exist.

- [ ] **Step 3: Implement**

```rust
/// Turns the pieces an edit cut free into bodies, removing them from the world.
///
/// `room` is how many more bodies the scene can draw. Past it, the largest
/// pieces are taken and the rest stay in the world: a piece that vanished
/// because the cap was full would look like a bug, not a budget.
pub fn detach(
    tree: &mut Contree,
    materials: &MaterialTable,
    lo: IVec3,
    hi: IVec3,
    room: usize,
) -> Vec<Body> {
    let mut pieces = loose_pieces(tree, lo, hi, BUDGET);
    pieces.sort_unstable_by_key(|p| std::cmp::Reverse(p.len()));
    pieces.truncate(room);

    let mut bodies = Vec::new();
    for piece in pieces {
        let materials_of: Vec<_> = piece.iter().map(|&p| (p, tree.get(p))).collect();
        tree.clear_voxels(&piece);
        if let Some(body) = body_from(&materials_of, materials) {
            bodies.push(body);
        }
    }
    bodies
}

/// Builds a body holding `voxels`, placed where they were.
///
/// The volume is the smallest power of four that fits the piece, because that
/// is what a `Contree`'s extent must be, and the piece sits at its corner.
fn body_from(voxels: &[(UVec3, MaterialId)], materials: &MaterialTable) -> Option<Body> {
    let (first, _) = *voxels.first()?;
    let mut lo = first;
    let mut hi = first;
    for (p, _) in voxels {
        lo = lo.min(*p);
        hi = hi.max(*p);
    }
    let span = (hi - lo + UVec3::ONE).max_element();
    let mut extent = 4;
    while extent < span {
        extent *= 4;
    }
    let local: Vec<_> = voxels.iter().map(|(p, m)| (*p - lo, *m)).collect();
    let mut body = Body::new(Contree::from_voxels(extent, &local), lo.as_vec3(), Quat::IDENTITY);
    body.recompute(materials).then_some(body)
}
```

Add the imports this needs: `crate::body::Body`, `crate::material::{MaterialId, MaterialTable}`, `glam::Quat`.

- [ ] **Step 4: Run to verify they pass**

Run: `cargo test --workspace > t.txt 2>&1; echo $?`
Expected: `0`, with no warnings.

- [ ] **Step 5: Add the gate that says why this milestone exists**

In `solver.rs`'s tests, the whole point of the feature:

```rust
    /// The milestone in one test: carve the support out and the piece falls,
    /// then rests on the floor.
    #[test]
    fn a_cut_column_falls_and_lands() {
        let materials = materials();
        let mut world = slab(64, 0..8);
        let mut voxels = world.voxels();
        for y in 8..20 {
            voxels.push((UVec3::new(32, y, 32), MaterialId(2)));
        }
        world = Contree::from_voxels(64, &voxels);
        let field = DistanceField::build(&world);

        // Cut the column's base.
        world.clear_voxels(&[UVec3::new(32, 8, 32), UVec3::new(32, 9, 32)]);
        let mut bodies = crate::physics::detach::detach(
            &mut world,
            &materials,
            IVec3::new(31, 7, 31),
            IVec3::new(33, 10, 33),
            16,
        );
        assert_eq!(bodies.len(), 1, "the column did not come free");
        let started_at = bodies[0].position.y;

        run(&mut bodies, &world, &field, &materials, GRAVITY, 200);
        let landed_at = bodies[0].position.y;
        assert!(landed_at < started_at - 1.0, "it never fell: {started_at} to {landed_at}");
        assert!(bodies[0].velocity.length() < 0.5, "it never settled: {:?}", bodies[0].velocity);
        assert!(landed_at > 8.0, "it fell through the floor to {landed_at}");
    }
```

- [ ] **Step 6: Break checks**

1. In `body_from`, place the body at `Vec3::ZERO` instead of `lo`. `a_freed_piece_becomes_a_body_where_it_stood` must FAIL.
2. In `detach`, clear the voxels but return no bodies. `a_cut_column_falls_and_lands` must FAIL.
3. In `detach`, drop the `truncate(room)`. `a_piece_stays_in_the_world_when_there_is_no_room` must FAIL.

Restore each and re-run.

- [ ] **Step 7: Commit**

```bash
git add crates
git commit -m "feat(core): a piece cut free becomes a body where it stood" -m "Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>"
```

---

### Task 4: The app, the spec, and the numbers

**Files:**
- Modify: `crates/bevox_render/src/upload.rs`, `crates/bevox/src/main.rs`
- Modify: `docs/superpowers/specs/2026-09-15-bevox-rigid-bodies-design.md`
- Modify: this plan (Measurements)

- [ ] **Step 1: Write the failing test**

In `main.rs`'s tests:

```rust
    /// Erasing the base of the demo scene's column drops it: the erase runs
    /// detachment, and the freed piece arrives as a body.
    #[test]
    fn erasing_a_support_spawns_a_body() {
        let (tree, materials) = demo_scene();
        let field = DistanceField::build(&tree);
        let mut scene =
            VoxelScene { tree, materials, generation: 1, field, field_dirty: None, bodies: vec![] };
        // The demo column stands on the floor at x = 28..36, z = 28..36.
        erase_and_detach(&mut scene, Vec3::new(32.0, 7.0, 32.0), 5.0);
        assert_eq!(scene.bodies.len(), 1, "the column did not come free");
        assert!(scene.bodies[0].mass.mass > 0.0);
        assert_eq!(scene.generation, 2, "the body list changed without asking for a rebuild");
    }
```

- [ ] **Step 2: Run to verify it fails**

Run: `cargo test -p bevox erasing_a_support > b.txt 2>&1; echo $?`
Expected: non-zero; `erase_and_detach` does not exist.

- [ ] **Step 3: Implement**

In `main.rs`, beside `brush_input`:

```rust
/// Erases a sphere, then hands whatever it cut free to the physics as bodies.
///
/// The cap is the renderer's: past `MAX_BODIES` a body is uploaded but not
/// marched, so a piece with nowhere to go stays in the world rather than
/// disappearing.
fn erase_and_detach(scene: &mut VoxelScene, centre: Vec3, radius: f32) {
    apply_brush(scene, centre, radius, MaterialId::EMPTY);
    let reach = radius.ceil() as i32 + 1;
    let hit = centre.round().as_ivec3();
    let room = MAX_BODIES.saturating_sub(scene.bodies.len());
    let VoxelScene { tree, materials, bodies, generation, .. } = scene;
    let freed = detach(tree, materials, hit - reach, hit + reach, room);
    if !freed.is_empty() {
        bodies.extend(freed);
        // The body list changed, so the packed buffers must be rebuilt.
        *generation += 1;
    }
}
```

Call it from `brush_input` where the erase is today, keeping the paint path on `apply_brush`. Import `bevox_core::physics::detach::detach` and `bevox_render::pipeline::MAX_BODIES`, exporting the latter if it is not already public.

- [ ] **Step 4: Run to verify it passes**

Run: `cargo test --workspace > t.txt 2>&1; echo $?`
Expected: `0`, with no warnings.

- [ ] **Step 5: Break check**

Have `erase_and_detach` skip the generation bump. `erasing_a_support_spawns_a_body` must FAIL. Restore.

- [ ] **Step 6: Update the spec**

In the Detachment section of `docs/superpowers/specs/2026-09-15-bevox-rigid-bodies-design.md`, replace the claim that the search walks uniform nodes with what is built:

```markdown
Detachment finds the disconnected parts with a depth-first search using
**6-connectivity** (face neighbours only), over **voxels**, bounded by a budget:
a search that exceeds it calls the piece grounded and leaves it alone. A piece
is grounded when it holds a voxel on the world's floor.

Dwyer's devlog #12 walks the tree's **uniform nodes** instead, which is far
faster on a large volume, because a solid node is one graph node rather than
thousands. That is the upgrade when a measurement asks for it; the budget is
what makes the simple walk safe until then.

A piece is never taken when the renderer has no room to draw it: geometry that
vanished because the body cap was full would read as a bug.
```

Add "the voxel-level search and its budget" to the Provenance section's list of fill-ins.

- [ ] **Step 7: Measure**

Add an ignored timing test in `detach.rs` that cuts the base from a 40-voxel-tall column of 8x8 cross-section (2,560 voxels) and times `detach`, and a second that runs the same search against a 64-extent solid slab where the budget stops it. Print both.

Run it, then record in this plan's Measurements: the numbers verbatim, the date, the CPU, what the budget costs when it gives up, and whether the search is anywhere near a frame's budget.

- [ ] **Step 8: Commit**

```bash
git add crates docs
git commit -m "feat(app): erasing terrain drops what it cuts free" -m "Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>"
```

- [ ] **Step 9: Hand over**

Tell Flori what to try: `cargo run -p bevox`, then right-click the base of the column in the demo scene. The column should come free and topple. Do not merge or push.

---

## Measurements

2026-09-18, Intel Core i5-10400, release build,
`cargo test --release -p bevox_core a_detach_is_timed -- --ignored --nocapture --test-threads=1`.
Median of nine runs each.

First version, which walked on after reaching the floor:

```
detach, median ms: 2560-voxel column freed 2.949, hole in ground 18.086
```

After stopping a walk the moment it is grounded, with down tried first:

```
detach, median ms: 2560-voxel column freed 1.765, hole in ground 0.247
```

These are two runs of two builds, not an interleaved A/B/A. The gap on the
hole, 73 times, is far outside any drift this machine has shown. The column's
1.2 ms gain is plausible but not claimed.

**What it says.** The common case, a cut into solid ground that frees nothing,
went from a full budget walk to a few dozen steps, from over a frame's worth of
work to a quarter of a millisecond. Freeing a 2,560-voxel piece costs under
2 ms, most of it building the body's tree and computing its mass. Neither is a
reason to walk uniform nodes yet.

**One gate was found blind and replaced.** The budget test could no longer
produce the fenced-in walk it guarded against once the search order changed, so
breaking that rule went unnoticed. It is now guarded by
`the_search_agrees_with_labelling_every_piece_in_full`, which compares the
search against a plain full labelling over 300 random scenes with budgets down
to 3. Any walk that stops early and fences a later one in disagrees with it.

## Milestone check

Cutting a column's base leaves the column falling as a body, landing and settling on the floor. A cut that leaves everything grounded spawns nothing. A piece too large for the budget, or with no room under the body cap, stays in the world.

## What this plan deliberately does not do

- **No brush editing of bodies.** Painting onto a body, erasing from one, and splitting one in two are milestone 4's second half.
- **No node-level search.** The budget is what makes the voxel walk safe; the node walk waits for a measurement.
- **No merging a settled body back into the terrain.** Dwyer does this to stop paying for debris; it is its own milestone.
- **No fracture on impact.**
