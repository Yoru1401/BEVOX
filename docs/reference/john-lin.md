---
type: Reference
title: "John Lin's voxel engine, read from his blog"
description: "A second voxel developer, unrelated to Dwyer, whose argument is that one voxel format chosen for the renderer poisons every other system; his answer is a canonical format plus registered conversion operators."
tags: [reference, architecture, voxel, rendering, vulkan]
generated: { by: claude-opus-5/claude-code, at: 2026-10-01T00:00:00Z }
sources:
  - id: perfect
    resource: https://voxely.net/blog/the-perfect-voxel-engine/
    title: The Perfect Voxel Engine
    author: human:john-lin
  - id: ecs
    resource: https://voxely.net/blog/object-oriented-entity-component-system-design/
    title: Object-Oriented Entity-Component-System Design
    author: human:john-lin
  - id: channel
    resource: https://www.youtube.com/@johnlin9665
    title: John Lin on YouTube
    author: human:john-lin
  - id: views
    resource: /concepts/derived-view-per-job.md
    title: 'One canonical tree, and a derived view for each job'
---

# Who this is, and who he is not

**Not Douglas Dwyer.** A separate developer with a separate engine, found
2026-10-01. Nothing here belongs in
[the drift ledger](dwyer-drift.md) — BEVOX is not modelled on this engine and
has never diverged from it.

He is worth a reference page for one reason: his central argument is an
objection to the architecture BEVOX is built on, written by someone who tried
the alternative.

# What exists to read

| | |
|---|---|
| Blog | `voxely.net` — **three posts.** OO-ECS Design (2021-07-27), The Perfect Voxel Engine (2021-09-18), RNN for 2D Tile Map Synthesis (2023-08-21) |
| YouTube | `@johnlin9665` — 41.8K subscribers, **14 videos**, newest five years old |
| Source | **None for this engine.** `github.com/Lin20/BinaryMeshFitting` (MIT, C++) is an earlier dual-marching-cubes and manifold-dual-contouring mesher, a different project |

**The renderer post was never written.** "The Perfect Voxel Engine" closes on
"in the next post, we'll (hopefully for real) dive into the rendering
architecture"; the next post, two years later, is about RNNs. So everything
below the format discussion is a sketch he gave in passing, not a description
of a shipped renderer, and the project has been silent since.

Treat the whole page accordingly: **this is a design argument, not a measured
engine.** There are no numbers in it, no benchmark, and no code to run. That is
the opposite of how everything else in this bundle is sourced.

# The argument

**A voxel engine fails because the data format is chosen to satisfy the
renderer, and then every other system has to live with it.**

His list of what an engine must also do, which the format has to serve:
lighting, state serialisation and network synchronisation, collision detection,
AI and path finding, dynamic objects — and then terrain features, trees,
vegetation, water, structures, procedural generation, per-voxel physical
interaction, and voxel characters.

His verdict on the structure BEVOX uses, verbatim:

> As it turns out for sparse voxel octrees, storage and rendering are the only
> things they are acceptable (not even great) at.

The sharp form of it is a thought experiment. Given a ray-tracing API over
`{vec3 position; vec3 normal;}`, adding colour has two answers: widen the
struct and bake the attribute into the leaves, or accept any stride and offset
and bake **indices**, leaving each consumer to decode what it needs. He picks
the second, then says: replace Vertex with Voxel, and the first answer **is**
Efficient Sparse Voxel Octrees.

His second point is that the problem is already solved in mesh pipelines. A
library like `assimp` imports anything into one common minimum format —
vertices and indices — which physics and ray-tracing libraries then convert to
whatever they need, with supplementary attributes carried alongside rather than
inside.

# The design: one minimum format, plus registered conversions

Three stages, which he calls **Allocation, Tagging, Conversion.**

**Allocation.** Named allocators hand out a buffer and take it back, so no
caller manages lifetime. `"cpu_recycled"` for scratch work; a different
allocator when the data must land on the GPU or on disk. The allocator is
swapped by name, not by type.

**Tagging.** An attribute is `{name, bits_per_element, type,
total_size_in_bytes, optional data pointer}`. The `type` enum mirrors GLSL's
primitive types, or is `"custom"`. Common ones are preloaded — `albedo` as
`u8vec4`, `normal` as `vec3`. Two things fall out: a modder can add an
attribute without the engine knowing about it, and an attribute exists **only
in the volumes that need it**. His example is not storing vegetation-growth
state deep underground where nothing grows.

**Conversion.** Operators are registered by source and destination format
*name* and looked up at run time — `get_conversion("terrain_cell", "default")`.
Type coercion is template specialisations of `attribute_converter<From, To>`;
his `u8vec4 -> vec4` applies the `1/255` scale that a plain cast would miss. He
frames operators as black boxes and lists what they can be: mesh voxelisation,
Minecraft map import, CSG-as-building-system, compressors, **collision-data
generation**, procedural terrain voxelisation, and vegetation grown from seed
data.

**He names his own weak spot, unprompted**, and it is the real hole:

> Admittedly, this part is a little weak because it requires pre-coding the
> supported type combos.

The runtime function pointer can only select among `From, To` combinations
someone anticipated well enough to force the compiler to emit. So the
"arbitrary attributes" claim is bounded by a compile-time list after all.

# The renderer, as far as he describes it

Vulkan ray tracing. Which shaders run is determined by the BLAS geometry
configuration, the SBT offset on the top-level instance, the shader binding
table's contents, and which geometry was hit. He builds **per-format
intersection shaders**, plus **callable shaders** that decode attributes a
pipeline asks for. The engine assembles the SBT; the user links a format to its
shaders.

That is the whole of it. No measurements, no resolution, no frame time.

# The engine around it

From the earlier post: a hybrid he calls **OO-ECS**, on **Flecs**. An *engine*
hosts contexts and domains; a *context* creates and operates on components; a
*domain* is a collection of components that can itself act as an entity, a
system, or a component. C++ core for rendering and physics, C function pointers
at the boundary, C# bindings for high-level logic — deliberately
language-agnostic, so content work can happen in a managed language.

# What BEVOX should and should not take

**The criticism lands, and BEVOX has already measured it.** A knowledge-graph
pass over this repository on 2026-09-30 put `MaterialId` at a betweenness
centrality of **0.200**, bridging **37 of 72** detected communities — core
storage, physics contacts, GPU packing, editing and import. That is the "one
format leaks into every system" failure he predicts, arriving as a number
rather than an opinion.

**Two of his specific objections do not bite here**, and the reason matters:

- *Baked per-voxel attributes.* BEVOX stores a material index against a
  256-entry palette and **generates** normals at the sample —
  `normal::implicit_normal`. The problem he names needs stored per-voxel
  attributes, and BEVOX has none. The same choice is why
  [F3](dwyer-drift.md) is blocked: per-voxel sun visibility has nowhere to live.
- *Collision detection.* BEVOX does not solve against the tree. It classifies
  voxels into shapes (`physics::classify`), walks cells to detach, and keeps
  two coarse grids. Which is his own answer, reached piece by piece.

That last point is the whole value of reading him, and it is written up as
[one canonical tree, and a derived view for each job](/concepts/derived-view-per-job.md).

**What not to take: the runtime format registry.** He admits it needs
pre-coded type combinations; BEVOX is native-only with one renderer and one
solver; and an abstraction with one implementation is not an abstraction. The
cost of his flexibility would be paid in shader codegen, which is where this
project has already been bitten twice —
[the GPU codegen cliff](/concepts/gpu-codegen-cliff.md).

# A caution about the source itself

An argument that is widely admired and never shipped is weak evidence. The
comments under the post are five years of people asking where the next one
went. He may be right about sparse octrees and still not have built the thing
that proves it, and **"Lin rejected sparse voxel octrees" is not a measurement**
— the same caution the drift ledger already carries about Dwyer's rejections.
