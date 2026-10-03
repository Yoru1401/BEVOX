---
type: Map
title: 'bevox_core on one page'
description: 'The voxel data structure, what each of its sixteen modules is for, how a voxel address descends to a voxel, and the three derived structures that must be kept honest.'
tags: [map, core, contree, data-model, reference]
generated: { by: claude-opus-5/claude-code, at: 2026-09-28T00:00:00Z }
sources:
  - id: spec
    resource: /superpowers/specs/2026-09-13-bevox-raymarcher-core-design.md
    title: BEVOX ray-marcher core — design
---

# bevox_core

The voxel data structure and every algorithm over it. **No Bevy, no GPU, no
window** — so all of it is testable headless, and a Bevy upgrade breaks
plumbing rather than the engine. That split is the reason this crate exists.

# Where it lives

**The data structure itself.**

| | |
|---|---|
| `node` | The 16-byte `Node`: a 64-bit child mask, a child base, a material. Uploaded to the GPU unchanged. |
| `contree` | The sparse 64-tree. Built bottom-up, so uniform regions collapse as they are built rather than in a later pass. |
| `arena` | Where nodes and leaf voxel bytes actually sit, with a free list per size class and the dirty ranges an upload needs. |
| `address` | How a voxel is named: a chunk coordinate and a local position. One chunk today; streaming adds chunks without changing the spelling. |
| `material` | The material table. Colour plus the physics columns, each an integer so `Material` stays `Eq`. |

**Reading and changing the volume.**

| | |
|---|---|
| `edit` | Changing a built tree: the sphere brush, and the voxel-list fill and clear that detachment, merging and fracture are built on. Rebuilds only the subtrees it touches and marks the arena ranges dirty. |
| `march` | The CPU reference ray marcher — the ground truth the WGSL shader is held to, pixel for pixel. |
| `normal` | Per-voxel normals generated from neighbour occupancy. Never stored: see below. |
| `dense` | A plain 3D array, for building a tree and for checking one. Test and import scaffolding, not a runtime structure. |
| `vox` | MagicaVoxel import, whole scene graph included. |
| `body` | A volume placed by a rigid transform, and the state that moves it. |

**Derived structures, rebuilt from the tree.**

| | |
|---|---|
| `distance_field` | How far each 16-voxel cell is from anything solid, so a ray can skip empty space. |
| `fullness` | How full of solid each 16-voxel cell is, for ambient occlusion. |
| `mask_table` | 512 precomputed masks — 64 entry cells × 8 direction octants — that say which children a ray could possibly reach. |

**Support.**

| | |
|---|---|
| `gpu` | Packs nodes and voxel bytes into the words the shader reads. |
| `testing` | A seeded `XorShift64`, so random tests are reproducible and add no dependency. |

# How a voxel is found

Every level divides by four on each axis, so the tree is `BRICK_EDGE`-cubed
groups all the way down and the descent is the same step at every level:

```text
extent 4096  ->  6 levels  ->  a leaf brick is 4x4x4 voxels

  at each level:
    child = (position - origin) / level_extent(level - 1)      which of the 64
    bit   = child.x + child.y * 4 + child.z * 16               its index
    mask & (1 << bit) == 0   ->  nothing here; the ray steps over the whole cube
    slot  = child_base + popcount(mask & ((1 << bit) - 1))     where it lives

  a node with no children is uniform:
    material == 0  ->  empty, and a ray crosses it in one move
    material != 0  ->  solid, and a ray stops at once
```

The 64-bit mask is doing four jobs at once, which is what makes the whole
structure pay: sparse child addressing, empty-space skipping, the bitmask
filter (AND it with `mask_table` and a zero means the ray cannot hit anything
in this brick), and the uniform collapse that keeps construction, editing and
traversal all sub-linear in voxel count.

# Why sixty-four

Four per axis is not a free parameter. **A contree is an octree with two layers
squashed into one** — branching goes from 8 to 8², so a node divides space into
4x4x4 instead of 2x2x2. The name is `tetrahexaconta` (Greek, 64) plus `tree`.

That derivation is what picks the number, and the register argument above is its
*consequence* rather than its reason:

- **four per axis** is 2², the two merged levels;
- **a 64-bit mask** is 8², their combined occupancy — and so exactly one machine
  word, which is why a ray reads it once and then steps on bits;
- **six levels to 4096** instead of twelve.

**Two layers is the largest squash that still works.** Three would give 512
children and a 512-bit mask, which no longer fits a register, and the structure
loses the one trick it is built on. See
[the contree, and where the name comes from](../reference/contree.md).

# Two rules that are easy to break

**Normals are never stored.** They are generated from neighbour occupancy at
upload time. Storing them means every fill, copy and delete must also fix the
normals of everything newly exposed — which made region operations intractable
in Dwyer's engine and forced a rewrite (his devlog 22). `normal` exists so that
nothing is tempted to cache them.

**The two coarse grids do not fail alike.** A stale distance field only
under-estimates, costing speed; stale fullness is wrong on screen, in both
directions. So an erase may skip the field, and **nothing** may skip fullness —
which is why every path that edits the world goes through one method rather
than each caller remembering. See
[no safe stale direction](../concepts/stale-direction.md).

# Constants

| | | |
|---|---|---|
| `BRICK_EDGE` | 4 | Voxels per axis in a leaf brick, and the branching factor at every level. |
| `CHILDREN` | 64 | Children per node, which is `BRICK_EDGE` cubed and the width of the mask. |
| `CELL_VOXELS` | 16 | Voxels per axis in a coarse cell. Both derived grids use it, and they must agree. |
| `MAX_DISTANCE` | 16 | The furthest a distance-field cell counts, in cells. Beyond it the value is already conservative. |
| `OCTANTS` | 8 | Direction sign combinations, one per octant of ray direction. |
| `TABLE_LEN` | 512 | Entries in the direction mask table: `CHILDREN * OCTANTS`, 4 KB uploaded once. |
| `MAX_STEPS` | 65 536 | The reference marcher's step cap. A runaway loop reports an overrun rather than hanging. |
| `MAX_EXTENT` | 4096 | The largest volume an import will build. |
| `DEFAULT_DENSITY` | 1000 | Mass per voxel when the source says nothing, as a MagicaVoxel palette does not. |
| `DEFAULT_FRICTION` | 60 | Hundredths, so 0.6: about dry stone. |
| `DEFAULT_RESTITUTION` | 5 | Hundredths. Barely bouncy, which is what most solids are. |
| `DEFAULT_STRENGTH` | 2 800 000 | The largest contact impulse a material survives. An impulse, so it is read against a body: a four-voxel stone cube breaks terrain arriving at 200 voxels a second, which is terminal, and holds at 120. Deliberately unchanged when `GLASS_STRENGTH` rose — it is calibrated against a thrown body, not a landing — and nearly seven times clear of the 414 166 the worst glass stack lands with. |
| `UNBREAKABLE` | `f32::INFINITY` | A material that never fractures, however hard it is hit. |
| `DEFAULT_CRUSH` | `f32::INFINITY` | The largest contact *force* a material survives while resting, as opposed to `strength`'s impulse on an impact. Nothing reads it yet, so every table sets it to `UNBREAKABLE`. |

# Read next

- [The spec](../superpowers/specs/2026-09-13-bevox-raymarcher-core-design.md) —
  the data model, the memory budget and why the crates are split this way.
- [Dwyer devlog by devlog](../reference/dwyer-devlogs.md) — 17 and 18 are the
  tree and its traversal, 22 is why normals are implicit.
- [No safe stale direction](../concepts/stale-direction.md) — what the derived
  grids owe the tree.
