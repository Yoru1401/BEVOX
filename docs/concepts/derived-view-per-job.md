---
type: Rule
title: 'One canonical tree, and a derived view for each job'
description: 'BEVOX already keeps six representations derived from the Contree, each shaped for one job and each with its own staleness policy; the rule is to add the seventh as a view rather than widening the tree.'
tags: [architecture, core, physics, render, correctness]
generated: { by: claude-opus-5/claude-code, at: 2026-10-01T00:00:00Z }
sources:
  - id: lin
    resource: /reference/john-lin.md
    title: "John Lin's voxel engine, read from his blog"
  - id: stale
    resource: /concepts/stale-direction.md
    title: No safe stale direction
  - id: core
    resource: /map/core.md
    title: bevox_core
---

# The pattern, which was already here

`Contree` is the only thing BEVOX stores. Every other shape the data takes is
**derived from it**, built by a named function, and kept for exactly one job:

| View | Derived by | Kept in | Job |
|---|---|---|---|
| `Features` — corners and edges | `classify::features` | `Body::features` | the points contact detection tests |
| `MassProperties` | `mass::mass_properties` | `Body::mass` | centre of mass and inverse inertia |
| `DistanceField` | `DistanceField::build` | `VoxelScene::field` | skipping empty space before the tree |
| `Fullness` | `Fullness::build` | `VoxelScene::fullness` | ambient occlusion |
| `GpuVolume` | `GpuVolume::from_contree` | the march buffers | what the shader can address |
| Normals | `normal::implicit_normal` | **nowhere** | shading, at the sample |

Six views, six shapes, one source. `VoxelScene` is the pattern written out as a
struct: `tree`, and then the views over it with their invalidation state beside
them.

**This was never decided.** Each view was built by the plan that needed it, and
the shape of the whole only becomes visible when they are listed together.

# Why it is worth a name

[John Lin](/reference/john-lin.md) argues that a voxel engine dies when one
format is chosen for the renderer and every other system is made to live with
it — that sparse voxel octrees are "acceptable (not even great)" at storage and
rendering and bad at everything else. His answer is a canonical minimum format
plus conversion operators registered by name.

**He is not theorising.** Fourteen months before writing that, he described his
own engine's *"unified voxel framework"* — world, physics, ray tracing,
lighting, procedural generation, sound tracing and collision detection all over
the same voxel data — as its central achievement. The argument is a post-mortem
on that engine, written after rewriting it.

BEVOX is his answer already, reached without the argument. The difference is
that his conversions are **declared** and BEVOX's are six unrelated functions
with no shared vocabulary. That costs nothing until someone reaches for the
wrong tool, and the wrong tool here is **widening the tree.**

A knowledge-graph pass on 2026-09-30 found `MaterialId` bridging **37 of 72**
communities at a betweenness of 0.200. That is what the canonical format
touching everything looks like, and it is the price of the pattern working: the
*one* type that genuinely belongs in every system is the one in the tree.
Anything else put there would pay the same cost without the same reason.

# The rule

**When a job wants the voxels in a different shape, derive a view. Do not add a
field to `Node`, and do not add an attribute to the tree.**

The tree holds a material index and an occupancy mask. Both earn their place in
every subsystem. A new per-voxel attribute would not, and would cost 16 bytes a
node across a 512 MB budget to serve one consumer.

Two consequences worth stating, because both have already been paid for:

**A view declares where it may be stale.** This is not optional and it is not
uniform — [no safe stale direction](/concepts/stale-direction.md) exists only
because two views sitting in the same struct, built over the same cells,
uploaded in the same buffer, have **opposite** policies:

- `field_dirty` is `None` after an erase. Removing geometry only raises true
  distances, so a lagging field under-estimates — it costs speed, never
  correctness.
- `fullness_dirty` is never skipped. A lagging fullness darkens or brightens
  the wrong voxels, and there is no direction in which that is safe.

So "derived from the tree" says nothing about when it must be rebuilt. Each
view answers that for itself, in a comment next to its dirty range.

**A view that is cheap enough is not stored at all.** `implicit_normal` is
recomputed at every sample rather than cached, and that is the reason
[F3](/reference/dwyer-drift.md) — per-voxel sun visibility — has nowhere to
live. The pattern does not say "cache everything derivable"; it says each view
chooses its own point on that trade, and not storing is a valid choice.

**That choice has a bill, and F3 is it.** A view recomputed per sample cannot
accumulate anything across frames. Sun visibility is not a function of the
sample — it is a property of the voxel, measured once and reused — so it needs
the one thing this pattern has so far declined to build: a **stored** per-voxel
view. Lin's third engine lists per-voxel material attributes as shipped, so the
blocker is BEVOX's, not the pattern's. The rule above still holds: that store is
a seventh view, not a wider `Node`.

# What this rules out

**A runtime format registry.** Lin's version dispatches conversions by name at
run time, and he concedes it needs the `From, To` combinations pre-coded anyway.
BEVOX is native-only with one renderer and one solver, so a registry would be
an abstraction with one implementation per slot, and the cost would land in
shader codegen — see [the GPU codegen cliff](/concepts/gpu-codegen-cliff.md).

Six functions and a struct are the right size for six views. The rule is about
where the **next** one goes, not about building machinery to hold it.
