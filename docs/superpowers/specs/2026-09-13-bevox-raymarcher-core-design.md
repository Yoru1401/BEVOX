---
type: Design Spec
title: 'BEVOX ray-marcher core'
description: 'A sparse voxel world ray marched on the GPU, with a CPU reference the shader is held to pixel for pixel.'
tags: [voxels, raymarching, gpu, architecture]
generated: { by: claude-opus-5/claude-code, at: 2026-09-13T00:00:00Z }
---

# BEVOX ray-marcher core — design

**Date:** 2026-09-13
**Status:** approved, ready for implementation planning

## Goal

A voxel ray-marching renderer in Rust on Bevy, rendering sparse voxel volumes with no
triangle meshing, with runtime voxel editing. This is the first milestone of a larger
engine, not the engine itself.

The technical approach follows Douglas Dwyer's Octo engine (Voxel Devlog #17, #18, #22),
reimplemented rather than ported — no Octo source is used, only the publicly described
techniques.

## Success criteria

The core is done when all of the following hold:

1. A MagicaVoxel `.vox` scene loads and renders at 1920x1080 on the target hardware.
2. Voxels are shaded with implicitly generated per-voxel normals against a directional
   sun, with hard shadows from a single secondary ray.
3. A sphere brush adds and removes voxels at runtime, and only the affected GPU buffer
   ranges are re-uploaded.
4. The beam prepass and the bitmask filter each produce output bit-identical to the same
   frame rendered with that optimization disabled.
5. Frame time is measured with GPU timestamp queries, and each optimization's effect is
   recorded from interleaved A/B/A runs.

No frame-time target is committed in advance. The reference point is that Dwyer reported
roughly 7 ms for a castle scene with a primary and a shadow ray on a GTX 1660 Ti; the
target hardware here is one tier below that. An actual target will be set from the first
measurement at milestone 5.

## Environment

| Item | Value |
|---|---|
| GPU | NVIDIA GeForce GTX 1650, 4 GB VRAM (Turing, no RT cores) |
| CPU | Intel Core i5-10400, 6 cores / 12 threads |
| Toolchain | rustc / cargo 1.96.0 |
| Bevy | 0.19.1 (requires Rust >= 1.95) |
| Model loading | dot_vox 5.2.0 |
| Platform | Native desktop (Windows) only |

4 GB of VRAM is the binding constraint on this hardware, not compute throughput.

## Scope

### In scope

Contree storage with 64-bit occupancy masks; DDA traversal with a bitmask filter; beam
prepass; implicit per-voxel normals; directional sun with hard shadows; `.vox` loading;
sphere-brush editing with incremental upload; a flying camera; measurement instrumentation.

### Deferred, expected later

Streaming or infinite worlds; levels of detail; physics; multiplayer; global illumination,
ambient occlusion, and path tracing; transparency; modding; non-grid-aligned moving
objects.

These are out of scope for the core, but all are expected to be built afterwards. They are
therefore treated as known direction rather than speculation — see "Design hedges for
deferred work" below.

Accepted consequence: with no LOD system, render distance is bounded by the fixed root
volume extent.

### Permanently excluded

WebAssembly and browser builds. This is a settled decision, not a deferral.

Native desktop may therefore be assumed everywhere: no WebGPU buffer-size ceilings, no
SharedArrayBuffer threading constraints, and native-only wgpu backend features may be used
without maintaining a browser fallback path.

## Architecture

Three crates in one Cargo workspace:

- **`bevox_core`** — all voxel algorithms. No Bevy dependency, no GPU dependency, no
  windowing. Contree construction and mutation, arena allocation, dirty-range tracking,
  buffer serialization, and a CPU reference ray marcher. Fully unit-testable headless.
- **`bevox_render`** — a Bevy plugin. `RenderApp` systems, pipelines, bind groups, the
  render graph node, and the WGSL shaders. Intended to be replaceable.
- **`bevox`** — the binary. Window, camera, input, brush controls.

The split exists because Bevy's rendering internals change across releases. Keeping every
algorithm in a crate that has no knowledge of Bevy means a future Bevy upgrade breaks
plumbing rather than the engine, and means the algorithms can be tested without a GPU or a
window.

## Design hedges for deferred work

Three deferred features would be expensive to retrofit and are nearly free to anticipate.
In each case the hedge is the *shape* of a signature or the *meaning* of a field. No
abstraction layer is built, no machinery is added, and no code exists to support a feature
that does not exist.

1. **Traversal takes a volume handle and a transform.** `traverse()` is defined over a
   volume plus an object-to-world transform, even though the core always passes a single
   volume and the identity transform. Adding non-grid-aligned moving objects later means
   adding a list and a broad phase, not rewriting the shader's addressing.
2. **World addressing is by chunk coordinate, never a hardcoded root node.** The core has
   exactly one chunk, at the origin. Streaming and LODs later add chunks; they do not
   change how a voxel address is expressed.
3. **`material` is an index into a material table, not a colour.** In the core, the table
   holds only a palette colour. Physics later adds density, friction, and restitution
   columns; transparency adds opacity. Neither changes the node layout or any traversal
   code.

Deliberately not hedged, because they are additive passes or separate systems and building
for them now would be speculation: LOD level selection, probe storage for global
illumination, the corner/edge/face voxel classification that physics needs, and any
networking structure.

## Data model

### Node layout

```rust
struct Node {          // 16 bytes, std430-compatible
    mask:       u64,   // which of the 64 children (4x4x4) exist
    child_base: u32,   // arena index of the first child; children are contiguous
    material:   u32,   // material table index; meaningful only when mask == 0:
                       //   0        => the region is empty
                       //   non-zero => the region is solid, of that material
}
```

Children are stored contiguously and addressed by

```text
child_index = child_base + popcount(mask & ((1u64 << i) - 1))
```

so a node with three children occupies three slots rather than sixty-four.

The mask serves four purposes at once:

1. Sparse child addressing, as above.
2. Empty-space skipping: a zero mask with a zero material means a ray steps across the
   whole cube in one move; a zero mask with a non-zero material is an immediate hit.
3. The bitmask filter: AND the mask with a precomputed per-direction reachability mask; a
   zero result means the ray cannot hit anything in this brick and can skip it entirely.
4. Homogeneous-region collapse, which is what makes construction, editing, and traversal
   all sub-linear in voxel count.

### Leaves

Leaf bricks use the same 4x4x4 shape. The mask records which of the 64 voxels are solid,
and the payload is `popcount(mask)` single-byte material table indices. Memory works out to
16 bytes per node plus one byte per solid voxel. `.vox` files and Teardown maps are already
palette-indexed, so per-voxel colour costs one byte rather than four, with up to 256
materials per model. Index 0 is reserved for empty.

### Normals

Per-voxel normals are **never stored**. They are generated at upload time from neighbour
occupancy.

This is a deliberate copy of the correction Dwyer made in Devlog #22. Storing normals
explicitly means every fill, copy, or delete must also recompute the normals of newly
exposed neighbours, which made basic region operations intractable in his engine and
ultimately forced a full rewrite.

### Direction mask table

64 starting cells times 8 direction-sign octants = 512 `u64` values, 4 KB total, generated
at build time and uploaded once. This is the lookup that drives optimization 3 above.

### Root extent and memory budget

The root volume is 1024 cubed, which is five levels of 4x subdivision. Dense, that would
be 1 GB; sparse, a typical scene is tens of megabytes.

Voxel data on the GPU is budgeted at **512 MB maximum**. The budget is checked at upload
and exceeding it is an error, never an allocation attempt.

### Editing

A brush computes the affected bounding box, rebuilds only the subtrees it intersects, and
marks the touched arena ranges dirty. Upload writes only dirty ranges.

Arena reclamation uses a free list per size class. This is the deliberately simple version
of the page-bitstring allocator Dwyer describes in Devlog #7. It is to be replaced only if
fragmentation is demonstrated by measurement.

## Render pipeline

Four stages per frame:

1. **Upload** — in `prepare`, write dirty arena ranges into the storage buffer.
2. **Beam prepass** — a compute dispatch at one-eighth resolution on each axis, writing an
   `R32Float` distance texture. Ray advance is capped so that no voxel can become small enough to fall between
   adjacent beam rays. This cap is what makes the optimization conservative; without it,
   thin geometry develops holes.
3. **Main pass** — a compute dispatch at full resolution. Seeds its starting distance from
   the **minimum** over the 2x2 block of beam texels surrounding it, marches, and on a hit computes the
   implicit normal from neighbour occupancy and casts one shadow ray toward the sun.
   Writes `Rgba8UnormSrgb`.
4. **Composite** — a render graph node blits the storage texture into Bevy's view target,
   ordered before `camera_driver`.

### Bind groups

- Group 0: view uniforms (inverse view-projection, camera position, sun direction).
- Group 1: voxel arena storage buffer, palette, direction mask table.
- Group 2: output storage textures.

Layouts are built with `BindGroupLayoutEntries::sequential`; pipelines are queued through
`PipelineCache::queue_compute_pipeline` and initialized in `RenderStartup`. The exact
access path for Bevy's view uniforms is to be confirmed against the Bevy 0.19.1 API during
implementation planning rather than assumed.

### Traversal

A single WGSL function serves both ray types:

```text
traverse(origin, dir, max_dist, any_hit) -> Hit
```

Shadow rays pass `any_hit = true` and return on the first intersection.

Traversal uses DDA within a brick, where the grid is uniform and stepping costs only
addition and comparison, and ray-box intersection when crossing between tree levels, where
cell sizes differ and DDA is not valid. Descent uses an explicit five-deep stack.

## Error handling

- **Hard step cap in every march loop.** A non-terminating loop in a compute shader hangs
  the GPU and the desktop with it. Every loop has a maximum iteration count, returns a miss
  on overrun, and increments a debug counter. Dwyer lost time to exactly this failure mode
  in Devlog #1, caused by floating-point imprecision letting a ray oscillate between two
  adjacent cells.
- **VRAM budget enforcement.** Model loads and edits that would exceed 512 MB are rejected
  with a surfaced error. The device is never asked for an allocation that could fail.
- **Pipeline compilation failure.** `PipelineCache` reporting a pipeline as not ready
  causes the graph node to skip that frame. It is not a panic.
- **wgpu error scopes** are enabled in debug builds.

## Testing

`bevox_core` has no Bevy and no GPU dependency, so it is developed test-first.

**Core tests**

- Round trip: dense array to contree and back, every voxel compared. Random volumes from a
  seeded xorshift generator; no test framework and no additional dependency.
- Canonical form: no node survives whose 64 children are identical, and popcount indexing
  agrees with the mask on every node.
- Arena: the free list never double-frees and never issues overlapping ranges; dirty ranges
  cover exactly the nodes an edit touched, no more and no less.
- Brush: voxels inside the radius are set, voxels outside are untouched, and the tree
  remains canonical afterwards.

**Reference marcher**

A CPU implementation of the same traversal, in Rust, serves as ground truth for the WGSL
port. For a fixed scene and camera, hit/miss and voxel ID must match exactly; distance
along the ray may differ within a float tolerance. Without this, shader bugs are diagnosed
by eye.

**GPU tests**

Headless wgpu device, fixed scene, output hashed against a golden image. The key
invariant: beam prepass enabled must produce **bit-identical** output to beam prepass
disabled, and likewise for the bitmask filter. An optimization that changes a pixel is a
defect, not a trade-off. The step-cap overrun counter must be zero on all test scenes.

## Measurement

GPU timestamp queries wrap each dispatch. The beam prepass and the bitmask filter have
independent runtime toggles.

Every performance claim comes from interleaved A/B/A runs within a single session. A
measurement taken today is never compared against a number recorded yesterday, because
cross-invocation drift on this hardware is expected to exceed the size of some of the
effects under test.

## Milestones

| # | Deliverable | What it proves |
|---|---|---|
| 1 | Contree construction and editing in `bevox_core` | Data structure is correct, with no GPU involved |
| 2 | CPU reference marcher writing a PNG | Traversal mathematics is correct, in a debugger |
| 3 | Bevy shell: window, fly camera, compute pass writing a flat colour | Plumbing is correct, with no algorithm involved |
| 4 | WGSL traversal | Matches milestone 2 on the golden scene |
| 5 | Implicit normals and sun shadow ray | Meets the stated visual bar |
| 6 | `.vox` loading via dot_vox | Real scenes render |
| 7 | Sphere brush and dirty-range upload | Editing without full re-upload |
| 8 | Beam prepass and bitmask filter | Each measured A/B/A, each bit-identical |

Optimizations come last deliberately: they are only safe once a golden reference exists to
protect them. Milestone 2 precedes milestone 3 so that the difficult algorithm is debugged
on the CPU and the shader becomes a port of code already known to work.

## References

Douglas Dwyer's Voxel Devlog series, in particular:

- #7 — GPU memory allocation and culling
- #15 — cheap ambient occlusion (out of scope here, relevant later)
- #17 — the 64-tree and per-node occupancy masks
- #18 — DDA, bitmask filtering, and the beam optimization
- #22 — why per-voxel normals should be implicit
