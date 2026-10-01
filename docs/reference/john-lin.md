---
type: Reference
title: "John Lin's voxel engine, read from his blog"
description: "A second voxel developer, unrelated to Dwyer, who built three voxel engines and then argued that one format chosen for the renderer poisons every other system; his video descriptions date the rewrites and show the argument is a post-mortem on his own unified-format engine."
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
| YouTube | `@johnlin9665` — 41.8K subscribers, **14 videos**, 2020-03-06 to 2021-05-13. **The descriptions are the better source than the blog**: dated, specific, and full of numbers |
| Source | **None for this engine.** `github.com/Lin20/BinaryMeshFitting` (MIT, C++) is an earlier dual-marching-cubes and manifold-dual-contouring mesher, a different project |

**The renderer post was never written.** "The Perfect Voxel Engine" closes on
"in the next post, we'll (hopefully for real) dive into the rendering
architecture"; the next post, two years later, is about RNNs. So everything
below the format discussion is a sketch he gave in passing, not a description
of a shipped renderer, and the project has been silent since.

Treat the blog post accordingly: **the format argument is a design argument,
not a measured one.** The post has no numbers in it and there is no code to run.
The video descriptions are a different matter — they are dated, specific, and
carry the only figures he published. They are collected below.

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

# Three renderers, not one

**This corrects an earlier version of this page**, which took the blog's Vulkan
ray-tracing sketch for a description of his renderer. The video descriptions
show he had already built and discarded two by the time he wrote it.

| When | Renderer | Why it changed |
|---|---|---|
| to 2020-06 | **Hardware RTX** path tracing. Noiseless and *"20x faster (60fps HD!)"* by 2020-03 | — |
| 2020-06-16 | **Custom ray tracer in Vulkan Compute.** *"The engine is no longer using RTX"* | Hardware RT was *"not quite as high"* in raw performance, but his own structure is *"an extremely lightweight acceleration structure that actually allows for this type of dynamic scene. I think in the end it's a net win"* |
| 2021-05-13 | **Third engine, rewritten from scratch over four months.** 10x faster than the second | A *"innocent attempt at increasing the view distance"* that grew |

**The middle row is the one BEVOX should read.** He dropped *hardware* ray
tracing — the fast path, on the card that advertises it — because a BLAS is too
heavy to rebuild for a scene that changes. BEVOX reached the same place by never
having the option: a compute shader marching its own tree, with no acceleration
structure to rebuild.

So the blog's BLAS/SBT/intersection-shader design is either a return to
hardware RT for the third engine or forward-looking design that was never
shipped. The third engine's description says high detail would be *"more likely
than not reserved for RTX graphics cards"*, which points at the former.
[Likely — the two statements are his, the reconciliation is mine.]

# The numbers he published

Not many, but more than the blog has, and all from the descriptions:

| | |
|---|---|
| Hardware | i7-8700K, 2080 Ti (stated for the fluid sim) |
| Fluid simulation | **never exceeded 8 ms** on 4 CPU threads; *"99% multithreaded, with a 1% critical section"* |
| Fluid method | hybrid Lagrangian-Eulerian after Hu et al. 2018 (MLS-MPM), **AVX2**, on the main thread; CPU-GPU transfer *"actually ha[s] a big impact on frame times"* |
| Physics | contact-based, **PGS solver**, one CPU thread, *"without any SSE trickery"*, parallel-ready |
| Third engine | **8x (512 cubic) detail increase**; **5 path-traced bounces** from sun, atmosphere and emissives; **10x rendering speedup**; effective world **256K cubed**; ~1 minute to generate the island at startup |
| World height | **y 0-4095**, a hard boundary |
| Building system | grid and default shape size **16 cubed** voxels, switchable off, down to a single voxel |
| Storage | a whole player-built scene **under 3 MB** |

# What he shipped that BEVOX has not

- **Per-voxel material attributes**, stated as a headline feature of the third
  engine. BEVOX stores a material index and generates normals, which is why
  [F3](dwyer-drift.md) — per-voxel sun visibility — has nowhere to live. He
  solved the storage problem BEVOX's F3 is blocked on, and did not say how.
- **Volumetric fracture of terrain and objects** (2020-10-10), before BEVOX's
  own fracture and from a different direction.
- **Rigid bodies coupled to a volumetric fluid** — one-way, buoyancy sampled
  volumetrically against the fluid velocity field where an object collides with
  it.
- **Constraints and motors on player-built objects**, with the note that *"mass
  distribution and surface area greatly affect the behavior"* — the same
  observation BEVOX records as mass properties recomputed on every volume
  change.
- **Ray-traced world generation**: assets placed by traced condition — in
  sunlight, on cave walls, in large open spaces.

And one line worth quoting for the opposite reason:

> Don't worry: we don't rotate our voxels here, because we know better.

**BEVOX does rotate its voxels.** A body is placed by a rigid transform and
marched in its own space, which is the whole of
[rigid body rendering](/superpowers/plans/2026-09-15-bevox-body-rendering.md).
He is not wrong that it costs something — it is why a body needs two `mat4`
transforms per ray before `traverse_at` can reject it. It is a deliberate
difference, and his throwaway parenthesis is the strongest outside argument
against it that this project has.

# The reversal, which is what makes his argument evidence

On **2020-07-12**, the unified format is the achievement:

> The whole approach is made possible thanks to a unified voxel framework. The
> world, physics processing, ray tracing, lighting, procedural generation, sound
> tracing and collision detection all use the same voxel data, allowing for some
> nice optimizations.

On **2021-09-18**, one format for every system is the disease, and sparse voxel
octrees are *"acceptable (not even great)"* at only two of those jobs.

Fourteen months and one full engine rewrite apart. [Certain that both are his
words on those dates; Likely that the second is a conclusion drawn from the
first.]

**This matters because it upgrades the blog post from opinion to post-mortem.**
An earlier version of this page cautioned that an argument widely admired and
never shipped is weak evidence. That caution was wrong in its premise: he
shipped three engines, wrote the unified-format approach up as a win, and
reversed it after living with it. The thing he never shipped is the *fix* — the
registry — not the diagnosis.

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
  attributes, and BEVOX has none. **But read that as a dodge, not a win**: it is
  exactly why [F3](dwyer-drift.md) is blocked, and his third engine lists
  per-voxel material attributes as a shipped feature. He solved the storage
  problem BEVOX is stuck behind; he just never said how.
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

Narrowed, because the descriptions falsify the wider version this page carried
first. He is not a theorist who never built anything — he built three engines
and reversed his own architecture after the second. What he never shipped is the
**registry**, the proposed fix, and that is the part to hold at arm's length.

Two things still do not resolve. The renderer post he promised never appeared,
so there is no account of how a dynamic attribute set reaches a GPU beyond the
SBT sketch. And the project has been silent since 2021 with the third engine
unreleased, so **nothing here is evidence that the multi-format design works at
the scale he was aiming at** — only that the unified one stopped working for
him.
