---
type: Reference
title: "Douglas Dwyer's voxel engine, devlog by devlog"
description: 'What Douglas Dwyer built in each of his thirty Voxel Devlogs, dated, and what each one changed from the ones before it.'
tags: [dwyer, voxels, reference, history]
generated: { by: claude-opus-5/claude-code, at: 2026-09-24T00:00:00Z }
sources:
  - id: channel
    resource: https://www.youtube.com/@DouglasDwyer/videos
    title: Douglas Dwyer, Voxel Devlog series
    author: human:douglas-dwyer
    last_modified: 2026-09-11T00:00:00Z
---

# What this is

A chronological record of Douglas Dwyer's voxel engine as he described it, one
section per devlog: what he built, how he said it works, and what it changed
from what came before.

**Where it comes from.** All thirty transcripts, plus the channel's own titles,
video IDs and upload dates read from YouTube on 2026-09-24. Everything here is
what he *says* in the videos. His engine is not open source, so nothing below is
read from his code, and where he gives no number this document gives none.

**How to read the arc.** Five phases, and he doubles back on each of them:

| Devlogs | Dates | What he is doing |
|---|---|---|
| 1-7 | Dec 2021 - Aug 2022 | building the renderer, twice, and choosing its language |
| 8-11 | Nov 2022 - Mar 2023 | distance fields, engine architecture, and the first moving voxels |
| 12-19 | Jun 2023 - Jun 2024 | the world: felling, terrain, ambient occlusion, distance, light |
| 20-24 | Jul 2024 - Aug 2025 | rigid-body physics, modding, a new codebase, GI, UI |
| 25-30 | Oct 2025 - Sep 2026 | the physics rewrite, shattering, a character, joints |

---

# 1. Implementing sparse voxel octrees and the ray caster

**2021-12-24.** [zbmYzugnEC0](https://www.youtube.com/watch?v=zbmYzugnEC0)

**What he built.** The project from nothing: C#, Vulkan, a compute shader that
ray marches a 256³ volume.

- **Why ray tracing at all:** he had been rendering voxels as bounding boxes in
  OpenGL, and wanted many dynamic lights. Shadow mapping means re-rendering the
  scene per light; a ray march answers a light with one more ray.
- **Why Vulkan:** macOS has no OpenGL version with compute shaders, and OpenCL
  meant interop with OpenGL as well. The tutorial alone was 900 lines and took a
  week, plus another week making his codebase object-oriented.
- **The structure:** a sparse voxel octree, chosen for two things at once —
  compression (a 256³ grid at one byte a voxel is 16 MB; his came in under half
  a megabyte) and traversal, because a homogeneous branch is stepped over whole.
- **The march:** per pixel, test each object's bounding box, advance to the
  nearest, read the voxel; on a miss, step the ray out of the octree node it is
  in and repeat.
- **Two floating-point bugs**, both found by stepping through shader code in
  RenderDoc: the ray cycling between two adjacent voxels forever, and the ray
  losing track of which voxel it was in. Fixed by forcing every step to move
  strictly forward, and by snapping the ray's position to the current voxel.

**Where it ended.** 7-10 ms on a GTX 1070. One unexplained cost he flags for
later: enlarging a small shared array of hit objects to 8 or 16 slows the shader
down although the work does not change, and moving it out of shared memory
nearly doubles the frame time.

---

# 2. Textures, lighting, and MUCH faster rendering

**2022-01-08.** [PfaNQLs6E94](https://www.youtube.com/watch?v=PfaNQLs6E94)

**What he built.** Materials, 3D textures, sun shadows, multiple objects, and a
halving of the frame time.

- **Materials:** a 16-bit id per voxel, pointing at a parameter set. For now it
  names a small 3D colour texture, tiled across the volume — three-dimensional
  texturing of a large world without the memory a per-voxel colour would cost.
- **LODs, nearly free:** stop the octree descent early and shade from the node
  you stopped at. He says plainly that it did *not* buy much speed, despite
  being trivial to add.
- **Sun shadows:** one more ray from the voxel toward the sun. "That is the real
  beauty of ray tracing techniques."
- **Per-voxel lighting, not per-pixel:** one ray from the voxel, averaged over
  its exposed faces. He prefers the look — it reads as the mesh's shape rather
  than as a pixel effect.
- **Many objects:** the first version sorted every object a ray might hit, which
  capped a ray at four or eight objects because GPU code cannot allocate. He
  replaced it with stepping: take the shortest step to the next voxel, and if
  another object offers a shorter one, continue in that object instead.

**The optimisation that paid.** His octree packed LOD data, a flags byte, the
full octants' materials and the sparse octants' offsets one after another, so
each descent had to compute where the offsets began. Aligning them instead
removed the arithmetic and the buffer reads, at the cost of 30-50% more memory.
**3.5-4 ms, from 7-10.**

---

# 3. Sparse voxel octree modification and benchmarking

**2022-02-15.** [yjOLx4O634I](https://www.youtube.com/watch?v=yjOLx4O634I)

**What he built.** Editing an octree in place, and the first honest benchmarks.

- **Why not the naive way:** building his 256³ octree from a flat array takes
  500-1000 ms. Rebuilding on every edit is out.
- **The algorithm** is divide and conquer over two octrees, a target and a
  source, pasting the source onto the target and keeping the target wherever the
  source is empty — "like pasting two transparent images on top of one another".
  If the target is wholly inside the source and the source is one homogeneous
  material, overwrite it whole; otherwise split the target into eight and
  recurse on the octants that overlap. It terminates because the worst case is a
  single voxel against a single voxel.
- **The bit trick:** a voxel's octant at level *n* is just the *n*-th bit of
  each of its three coordinates. He moved that to x86 SIMD intrinsics.
- It took three days and he is blunt about the first write-up being a mess,
  `goto` statements included.

**The numbers, and the language.** Placing a tree: 11 ms an iteration in C#.
Generating an octree from a flat array: 80 ms. He then rewrote both in C++ to
see what the runtime cost him — **5 ms and 40 ms, twice as fast** — and notes
that most of his C# was already `unsafe` pointer code, so he blames the runtime
itself rather than bounds checks.

---

# 4. Drawing MILLIONS of voxels on an integrated GPU with parallax ray marching

**2022-05-10.** [h81I8hR56vQ](https://www.youtube.com/watch?v=h81I8hR56vQ)

**The pivot.** He throws away both the language and the renderer.

- **Why:** the compute-shader marcher was memory-bound — it ran 30% faster on a
  1660 Ti than a 1070, which has a third less bandwidth — and it fell over on
  low-end hardware. He had also settled on what he wants to build: a multiplayer
  *platform*, playable in a browser, extensible with mods. That makes
  integrated GPUs a requirement, not a nice-to-have.
- **C# to Rust**, for WebAssembly at near-native speed, and because he did not
  want cross-platform C++. He is candid that coming from object-oriented C# the
  borrow checker took real relearning.
- **Compute shader to rasterisation**, because triangles are the one thing every
  GPU is good at.

**Parallax ray marching**, his name for the technique, after parallax bump
mapping. Rather than meshing each voxel face — greedy meshing only merges
coplanar faces, and he wants detailed surfaces — he **rasterises a bounding box
per 8×8×8 block of voxels and ray marches inside it in the fragment shader**,
using the marcher he already had. Triangle counts fall by an order of magnitude
or more. His mesher only ever emits tight boxes, so no fragment is spent on
empty space.

**The two costs he names.**
- **Depth.** A box's depth is not its voxel's, so he must write depth in the
  fragment shader — which disables the early depth test and made everything
  slow. His fix: draw voxel objects normally with early-Z on, write true depth
  to a *separate* buffer, then overwrite the depth buffer in a second pass.
- **No MSAA**, since he sets each pixel himself. He uses FXAA instead and calls
  it a fair trade.

**Where it ended.** 36 volumes of 256³, over a million surface voxels, lit, at
over 60 fps on Intel UHD integrated graphics.

---

# 5. Adding MULTIPLAYER NETWORKING with WebRTC to my game engine

**2022-07-17.** [bQXM0QSlsqs](https://www.youtube.com/watch?v=bQXM0QSlsqs)

**What he built.** Peer-to-peer multiplayer, and the client/server split that
shapes the engine from here on.

- **Why peer-to-peer:** hosting servers for everyone is expensive and forces
  users to upload their worlds. Direct connections are cheaper and faster.
- **Why WebRTC:** in a browser the only options are HTTP (too slow), WebSockets
  (not peer-to-peer) and WebRTC — the standard behind Discord and Teams video,
  which is exactly peer-to-peer media transport.
- **What that needs:** a signalling server, because clients cannot know each
  other's addresses and must get through firewalls. Two clients tell the
  signalling server they want to connect, it passes each the other's connection
  data, and they meet through ICE. He wrote it in Go, and notes he will need a
  server for accounts anyway.
- **The design decision that outlasts this episode:** all game logic moves to a
  **game-server thread**, and the local player connects to an internal server
  even in single player. Single-player and multiplayer code are then the same
  code. Networked motion is interpolated, so remote players move smoothly.

**Where it ended.** Two browser windows sharing one voxel world, each placing
and destroying spheres in it.

---

# 6. Designing a FLEXIBLE game engine with Rust

**2022-07-23, six days later.** [TRVrjSD6zgA](https://www.youtube.com/watch?v=TRVrjSD6zgA)

**What he built.** The architecture under devlog 5, which he had skipped over.

- **Why the Unity-style design failed:** object-oriented game code is an
  interdependent graph, and Rust requires a tree — one owner per object. He
  tried it, hit the wall, and rebuilt.
- **What he arrived at, after three iterations:** a **hybrid entity-component
  and event system**. A world holds entities, components and shared resources
  such as the clock. Systems act on the world and, unlike a plain ECS, keep
  their *own* state — the graphics system needs somewhere to put GPU handles.
  Each system is a set of event handlers: input broadcasts a click, the network
  system answers it. Built on Rust generics, and **open-sourced** on his GitHub.
- **The octree builder trait:** converting an array to an octree, merging two
  octrees and generating terrain are the same recursion with one question
  swapped — which regions share a material. He abstracted that into a trait, and
  notes that because Rust monomorphises generics, the abstraction is free; an
  inherited type would have put virtual calls in the hottest code he has.
- **Threads in the browser:** browsers forbid native threads, so he uses web
  workers plus `SharedArrayBuffer` and the `wasm_thread` crate to get real
  threads for client and server.

---

# 7. The PERFECT voxel rendering pipeline (and online demo)

**2022-08-30.** [IFUj53VwYvU](https://www.youtube.com/watch?v=IFUj53VwYvU)

**What he built.** Nine months in, the renderer he had prototyped in devlog 4,
made to scale — and a playable browser demo.

- **What was wrong with the prototype:** one texture per 256³ chunk, and a naive
  mesher. Neither scaled to many chunks.
- **A GPU memory allocator.** Every voxel object renders identically, so they
  all share **one big vertex buffer and a few big textures**. The allocator is a
  zero-cost abstraction over "a resource is pages of fixed size" — a segment of
  a buffer, a cube in a 3D texture. Free and filled pages are a bit string, and
  it finds contiguous runs 64 pages at a time with bitwise operations.
- **Generating the data:** descend the octree from the top, mark the 8×8×8
  parallax boxes on a solid region's surface, recurse otherwise. Each box's
  voxels are packed into a *materials map* texture; each visible face costs 16
  bytes — 4 naming its place in the materials map, 12 for four corners at three
  bytes each.
- **Three rounds of CPU-side face culling**, all resting on voxels having only
  six axis-aligned face directions:
  - **frustum culling** of whole 256³ chunks, against the frustum grown by a
    chunk in each direction;
  - **octant culling** — a chunk entirely to one side of the camera can never
    show the faces pointing away, so half of them are dropped;
  - **directional culling** for the faces octant culling misses.
  Together these leave **only 20-40% of loaded triangles drawn**, issued as one
  `glMultiDraw` — which matters most on the web, where each GL call crosses into
  JavaScript.
- **Shadows are recomputed only when the scene changes**, not per frame, so the
  expensive lighting pass is skipped on most frames. Static geometry and small
  dynamic objects are drawn in two passes, which keeps that optimisation alive
  even with moving shadow casters.

He credits an article (linked from the video) for the culling ideas.

---

# 8. GPU-generated DISTANCE FIELDS

**2022-11-26.** [REKcTBgkrsE](https://www.youtube.com/watch?v=REKcTBgkrsE)

**What he built.** A distance field generated by the *rasteriser*, and the
honest conclusion that it did not beat what he already had.

- **The field:** unsigned, and measured in the **infinity norm** rather than the
  Euclidean one — "the biggest possible cube you can draw" — because that is
  what a DDA over voxels can use. Sphere tracing then steps a ray by the field's
  value at its position.
- **Why generation is the hard part**, which is the part other voxel channels
  skip: the naive search is O(n³k³) for a kernel of k. A dynamic-programming
  algorithm gives the exact field in O(n³) and another runs in O(n³ log k), but
  neither parallelises, so neither suits a scene that changes.
- **His inversion:** instead of each empty voxel searching for solid, each
  **solid surface voxel writes its distance into the empty voxels around it**.
  Same answer, one dimension less work — interior voxels can be skipped
  entirely, since the camera is never inside geometry.
- **Why that is a rasterisation problem:** writing a value into every cell of a
  box, combining overlapping boxes, is what a GPU does with quads and blending.
  For each surface voxel he draws the quads of a cube of side 2k−1 into a 3D
  buffer, computes the distance in the fragment shader, and uses **min blending**
  to combine them.

**The result, and the verdict.** 15-30 ms CPU and 10-20 ms GPU per newly loaded
chunk on Intel integrated graphics. But **parallax ray marching still wins in a
significant number of cases**, so he does not adopt the distance-field renderer.
What he keeps is a faster octree traversal he found on the way, which feeds the
mesher.

**What he announces:** a full rewrite, because a cold debug build now takes over
30 seconds — split into a cargo workspace of smaller crates.

---

# 9. Codebase OVERHAUL, new EVENT SYSTEM, LODs, and MORE

**2022-12-27.** [YQ83XfZQHHA](https://www.youtube.com/watch?v=YQ83XfZQHHA)

**What he built.** The rewrite: **over 8,000 lines of Rust** redone or discarded.

- **Why his own event library had to change.** In the devlog-6 design, systems
  could only talk by raising events. That isolation pushed him toward monoliths
  — one graphics system that owned framebuffers *and* meshing *and* drawing.
- **The fix:** systems may now **depend** on other systems, forming a directed
  acyclic graph; a handler can query a system it depends on. A low-level graphics
  system can own the screen resolution, and a voxel renderer and a lighting
  renderer can both ask it. The library, `geese`, went to crates.io with docs,
  and its networking companion `geese_pool` with it.
- **Other changes:** material ids became their own type rather than a `u16`
  ("encode as much as you can in the type system"); the graphics system can be
  reloaded at run time, so toggling vsync no longer restarts the game; the
  server now tracks changed voxel regions itself and syncs them, so a
  server-side edit reaches clients with no networking code at the edit site;
  LODs return; and **client-side prediction** for edits, so your own changes
  appear before the server acknowledges them.
- **A side experiment:** he tried plain greedy meshing again and found it
  "quite comparable" to parallax ray marching — every scene in that video is
  rasterised triangles.

---

# 10. Adding SNOW and MODEL IMPORTS

**2023-01-29.** [XNtdYQLiGbA](https://www.youtube.com/watch?v=XNtdYQLiGbA)

**What he built.** Snow, as a worked example of how a feature enters his engine,
and an importer for other people's voxel models.

- **The split that makes snow cheap:** falling snow is **client-only** and purely
  visual; snow settling on the ground is **server-only** and real. Neither side
  knows the other's half. The client spawns about 6,000 flakes on the CPU —
  ray casting so none falls through terrain — then the GPU moves and draws them
  every frame.
- **The lesson he draws out:** his first ground-snow system placed one flake at
  a random spot per tick and was laggy, because **the engine is built for
  infrequent large voxel updates, not frequent tiny ones**. Batching 20-30
  placements into one update fixed it.
- **Model importing** via `gvox`, Gabe Rundlett's C++ library. Getting a C++
  library with its standard library to compile beside Rust *for the web* took
  the two of them about 20 hours and produced two internal compiler errors. He
  suggests importing Ace of Spades maps at 16× scale, which puts his voxels at
  1/16 of a Minecraft block — at that scale walking speed matches Minecraft's.
- Also: cascaded shadow maps, a settings menu, and a kanban board.

---

# 11. Making voxels MOVE with the separating axis test

**2023-03-18.** [PW1Xwc3zzNc](https://www.youtube.com/watch?v=PW1Xwc3zzNc)

**What he built.** The first physics: collision *detection*, plus transparency.
The episode is framed around two trade-offs he chose deliberately.

- **Trade-off one: his own detection, not PhysX.** A general library brings
  every shape pair out of the box; a bespoke one can be specialised to voxels.
  He took the bespoke path on the theory that voxel-against-voxel could be made
  very fast, and says it paid.
- **The separating axis theorem**, 15 axes for two boxes (faces plus edge
  cases). Because it projects both shapes onto each axis, the *gap* between the
  projections tells him how far they may move before touching — which gives him
  **continuous collision detection**, so nothing tunnels through a wall.
- **The optimisation only voxels allow.** Every voxel sits on the same regular
  grid, so one voxel is another shifted and scaled — and projection onto an axis
  is linear, so the projection shifts and scales with it. He computes the
  projections **once for a voxel at the origin** and reuses them for every other
  voxel, leaving **one or two matrix multiplies per voxel-voxel test**. Large
  empty octree nodes are skipped whole.
- **Trade-off two: order-independent transparency.** Correct transparency needs
  per-pixel sorting; Minecraft-style per-triangle sorting is cheaper but still
  expensive with many concave objects. He chose **weighted blending** — average
  the transparent colours by alpha, blend once into the scene — and says the
  result is close enough that he cannot tell them apart, for one extra pass.

---

# 12. Chopping trees DOWN

**2023-06-27, three months after the last.** [5e8ut4NgF-8](https://www.youtube.com/watch?v=5e8ut4NgF-8)

**What he built.** The first working rigid-body physics — and the feature the
whole engine is pointed at: **cut a tree's base and the tree falls**. It works
for anything, not just trees: disconnect any structure from the ground and it
becomes its own entity.

He frames the episode as breaking one unfathomable problem into two:

**1. Finding what came loose** — connected component labelling, by depth-first
search. Two voxels are connected if they share a **face**; diagonals do not
count, which he says is both simpler and more sensible.
- The image-processing algorithms on Wikipedia are faster per pixel (they scan
  whole rows, with better cache behaviour) but all are **linear in the number of
  voxels**.
- **His improvement: walk the octree's homogeneous nodes as graph nodes, not
  voxels.** A solid 8×8×8 node is known to be internally connected, so it is one
  node instead of 512 with all their edges. Nodes seen are written into a dense
  bitmap, so "is this node in the component" is constant time, and the component
  is then removed from the source volume.

**2. Collision response and rotation.** Devlog 11 left him able to detect, not to
respond. Rotations are quaternions, for compactness and interpolation.
- **Resting rotation is the hard part.** A phone on a table corner needs, in
  principle, the infinite set of points where the two touch. His approximation:
  gather **a discrete list of contact points** during detection (the separating
  axis test does not hand them to you), sum each one's torque, multiply by the
  time step, add to angular velocity.
- **Then it did not work, for three weeks.** Objects "sat up and started
  spinning in the air, gaining infinite rotational velocity, as though they were
  possessed." He fixed it by the same method as the feature: disable rotation on
  all axes but one, print between every stage, and compare. The tell was two
  objects with identical angular velocity spinning opposite ways. **He had
  swapped the two quaternions in the multiply** — non-commutative, as he knew.
  One line.

**What he says about the result.** It is **not physically accurate, because each
object is processed individually** — a tall stack of boxes will not topple
properly. "If I were to redo the physics engine in the future I would want to
learn more about the constraint-based physics techniques which are what engines
like Teardown use." He does exactly that, thirteen devlogs later.

---

# 13. OPTIMIZING my physics engine

**2023-08-04.** [b_d-0EyOuVg](https://www.youtube.com/watch?v=b_d-0EyOuVg)

**What he built.** Three things that make the physics affordable rather than
merely working.

- **Despawning, as merging.** Debris left by chopping and digging never gets
  interacted with again. Rather than vanish it, he takes a commenter's
  suggestion (credited to "majitec") and **fuses it back into the terrain**: an
  affine transform, then re-rasterise into the main voxel grid. **It only
  happens with the player's back turned**, so the change from entity to terrain
  is never seen.
- **Sleeping.** He records when each object was last changed by physics; if the
  object did not move last tick and nothing around it moved in the last few
  ticks, the result this tick is the same, so it is skipped. The numbers he
  gives: collision detection for one of his trees costs **1-2 ms**, and after
  chopping 10-20 trees the loop took **14-15 ms against a 25 ms tick budget**.
  With sleeping, a fallen tree at rest costs nothing at all.
- **Multi-threading**, twice over. A thread pool hands objects to cores: ten
  resting trees took **23 ms of thread time but 9 ms of wall time** on four
  threads. And `geese` itself became multi-threaded — two systems that are not
  each other's dependencies cannot observe each other, so their handlers may run
  at once while still appearing sequential.

**An aside he puts to the audience:** event-driven architecture like his versus
an ECS like Bevy's. He prefers events for one-off things like a player logging
in, and keeps an ECS inside a single system.

---

# 14. GORGEOUS, speedy terrain generation

**2023-12-04, four months later.** [m4-toCACxKU](https://www.youtube.com/watch?v=m4-toCACxKU)

**What he built.** Terrain generation moved to the GPU and made into data
instead of code.

- **What was wrong:** the old generator was multi-threaded CPU work, which stole
  cores from everything else, and it was **hardcoded** — compiled in, so no user
  could ever change it.
- **Terrain generators are now bytecode.** A generator is a list of low-level
  operations (add vectors, multiply, random). Each operation has a text snippet;
  he concatenates the snippets into shader source and compiles it. The bytecode
  serialises, so generators can be made and changed at run time — he wants a
  node editor, or a small language, in front of it later.
- **Interval arithmetic, which is what makes it fast.** A generator maps a
  position to a number, and the sign of that number picks the material. For many
  functions, a *range* of inputs gives a predictable range of outputs — `2x` over
  10..20 gives 20..40, `eˣ` over a..b gives `eᵃ`..`eᵇ`, and it generalises to
  logical operations. So he compiles **two** shaders: an interval version that
  proves a whole region homogeneous, and a "fine" one that samples voxel by
  voxel only where the interval version could not decide.
- **The measured effect:** for surface chunks with trees, rocks and grass, **no
  more than 50% of voxels are sampled individually, usually 20-30%**; sky and
  underground chunks are skipped wholesale.
- **Imported models inside that scheme.** Interval arithmetic cannot predict a
  3D texture, so he **precomputes a distance field for each model**: if the field
  at a box's minimum corner exceeds the box's side, the whole box shares that
  corner's value. He computes it on a 16× downscaled copy, since fields are slow
  to build, and uploads both resolutions, packing homogeneous octree regions out
  of GPU memory entirely.

This is the first episode with a sponsor.

---

# 15. Adding ambient occlusion to my game engine

**2023-12-25.** [3WaLMBiezMU](https://www.youtube.com/watch?v=3WaLMBiezMU)

**What he built.** Ambient occlusion, "after three attempts", by a method he
believes is his own.

- **Why the two standard approaches failed him:**
  - **Minecraft's per-block AO** looks only at directly adjacent blocks. His
    voxels are far smaller, and he wants shadows reaching **four to eight
    voxels**.
  - **SSAO**, which he has implemented professionally, fights his renderer twice
    over: it shades smoothly per pixel, while his lighting is one solid colour
    per voxel, and it is stochastic, so taking a single rounded sample per voxel
    left the scene noisy and flickering. Averaging every pixel on a voxel (tried
    in December, nine months after the first attempt) did not fix it either.
- **The model he replaced them with.** Ideally, draw a sphere of about eight
  voxels around each voxel and measure **what fraction of it is solid**. On a
  flat surface that is exactly 50%, and nothing should darken. In the crease
  where floor meets wall it is about 75%, and the excess over half is the
  darkening. He is explicit that this is neither perfect nor physically
  accurate — it is a good estimate of the effect he wants.
- **The approximation of the approximation**, which is what ships: divide the
  world into **16×16×16 cubes** and store how many voxels each contains. That
  value is the fullness at the cube's *centre*; everywhere else is **linear
  interpolation between neighbouring cube centres**. He notes the interpolation
  **never over-estimates**, so no shadow appears where none belongs.
- **Why it is cheap.** The count costs nothing extra on the CPU, because he is
  already walking the volume to greedy-mesh it, and on the GPU the blend is
  **one hardware-filtered texture read** — the filtering is the hardware's,
  so there is no interpolation code at all. Against SSAO's extra pass and eight
  or more random texture reads, he calls it "cheap as dirt".

**Also announced:** the graphics overhaul from OpenGL ("which is trash") to
WebGPU, which gave him the compute shaders of devlog 14 and vertex pulling,
cutting triangle counts another 15-20%.

---

# 16. How I tripled the render distance in my game engine

**2024-01-28.** [74M-IxtSVMg](https://www.youtube.com/watch?v=74M-IxtSVMg)

**What he built.** Levels of detail done properly — generated, not downsampled —
plus a sky and a fix to his transparency.

- **What was wrong:** the server used to load chunks at **full quality** and
  downsample them, so anything a client wanted to *see* the server had to load
  and simulate, including places no player was.
- **The change:** terrain generators already worked at any scale (quietly added
  in devlog 14), so an LOD is now **generated directly at its own resolution**.
  The client arranges the world as an octree of cubes, each drawn with the same
  number of voxels, subdivided more finely near the player.
- **The problem that costs the bookkeeping:** generating an LOD from scratch
  loses **player edits** — build a tower, fly away, and the generator does not
  know about it. His fix: cache generated LODs to disk; mark edited regions
  dirty and propagate the flag up the LOD octree; when a coarse LOD is requested,
  load it from the save, pull the finer data for any dirty sub-octant,
  downsample it and recombine. Zoom out, and the tower is still there.
- **Order-independent transparency, corrected.** His devlog-11 version averaged
  fragments with no weighting, so a red pane in front of a green one looked
  identical to the reverse. Adding a **weight that rises for fragments nearer the
  camera** recovers most of the look of back-to-front sorting while staying
  order-independent.

He closes by saying he has pushed greedy meshing and voxel rasterisation "as far
as I can possibly go".

---

# 17. Adding ray tracing (back) to my game engine

**2024-03-29.** [aY4Zet_C9Zs](https://www.youtube.com/watch?v=aY4Zet_C9Zs)

**What he built.** The second great reversal: he deletes the rasteriser and goes
back to ray marching, two years after leaving it. **Over 11,000 lines removed.**

- **Why:** his materials worked like Minecraft blocks — a palette, a 16×16
  texture each — so distant terrain was flat grey, and imported models looked
  bad because their colours had nowhere to map. What he wants is **per-voxel
  colours and per-voxel normals**, which he calls the secret sauce behind the
  engines he admires. A ray marcher reads voxel data directly; a rasteriser
  would need triangles per unique voxel.
- **The data structure, which is the heart of the episode.** GPU memory is slow,
  and an octree descent costs eight or more reads per step. So he replaces the
  octree with a tree that splits each cube into **64 children, four per axis** —
  he does not know a name for it and calls it a **brick tree**.
- **The bit mask that makes it fast.** Each node carries a **64-bit mask** of
  which children are occupied. A ray reads that mask once, into a register, then
  steps from child to child testing **bits, not memory** — so it can take **up
  to ten ray steps with no further memory read**.
- **What that buys:** a Teardown castle map at **7 ms a frame with a primary and
  a shadow ray on a GTX 1660 Ti**, with no optimisation of the marching loop and
  no depth prepass — "all just coming from the empty space skipping and this
  bit mask acceleration structure". He suspects he is compute-bound.
- **Integrated GPUs are not abandoned:** a ray marcher has a dial a rasteriser
  does not — render at lower resolution. He shows the engine running on
  integrated graphics that way.

He also changes how he makes the videos, deliberately looser, because "I don't
want my YouTube channel to be about the videos".

---

# 18. Doubling the speed of my game's graphics

**2024-04-26.** [P2bGF6GPmfc](https://www.youtube.com/watch?v=P2bGF6GPmfc)

**What he built.** Three optimisations to the new ray marcher, together a **2×**.
All numbers are on his Intel integrated GPU, one scene, start to finish
**120 ms → 60 ms**; he says his 1660 Ti gained similarly.

- **DDA instead of ray-box tests: 120 → 100 ms.** The old step recomputed, for
  all three components, the distance to each side of the voxel's box, with a
  division. DDA instead carries the distance to the next X, Y and Z plane, steps
  along whichever is nearest, and adds one increment back — **one component per
  loop, addition only**. The catch, and he is open about it: DDA wants a fixed
  grid, so he uses it **only inside a 4×4×4 brick** and falls back to ray-box
  tests when crossing levels. He asks the audience whether anyone knows how to
  DDA between levels.
- **Bit-mask culling of a brick: 100 → 80 ms.** A ray entering a brick can only
  ever reach a subset of its 64 cells — one travelling up and left can never
  meet the cells down and right. He **precomputes, at compile time, a mask per
  starting cell and set of cardinal directions**, and ANDs it with the brick's
  occupancy mask. Zero means the ray cannot hit anything in this brick, so it
  skips the whole traversal in a couple of instructions.
- **The beam optimisation: 80 → 60 ms.** Render the scene first at low
  resolution, then start each full-resolution ray at the distance its
  neighbours reached. The danger is a small voxel slipping between the coarse
  rays; his answer is that **a voxel large enough on screen must hit one of the
  surrounding coarse rays**, so each fine ray takes the **minimum** of its four
  or six neighbours, and the coarse pass is stopped before voxels shrink below
  that size. The result is **guaranteed conservative — the image is identical
  with the optimisation off**.

---

# 19. Emissive voxels and fancy lighting

**2024-06-26.** [VPetAcm1heI](https://www.youtube.com/watch?v=VPetAcm1heI)

**What he built.** Real-time path-traced indirect light, and the pipeline that
makes it hold still.

- **What the path tracer is for:** ambient occlusion, now as an actual light
  calculation rather than the fullness trick of devlog 15, and **emissive
  voxels**. He picks his own radiance function rather than a physical one: a ray
  that reaches the sky returns a constant sky colour; one that hits geometry
  returns an ambient colour with **exponential decay**, so surfaces close
  together darken. One Monte Carlo sample per pixel, working in a day or two.
- **Emissive light came for three or four lines.** A third branch: if the ray
  hits a light-emitting voxel, return that light's colour. Unlike rasterisation,
  **area lights are free**.
- **The noise problem, and his unusual answer.** One sample per pixel is grainy,
  and the standard denoisers assume per-pixel shading, which would destroy his
  one-colour-per-voxel look. So he built **a per-voxel hash map on the GPU**,
  with atomic compare-exchange: each frame records the visible voxels as keys,
  each pixel adds its lighting into its voxel's average, and a final pass
  combines unlit colour with per-voxel light.
- **What that bought beyond denoising:** the previous frame's hash map seeds the
  current one, giving **temporal accumulation**; and because the visible voxels
  are already enumerated, **direct sunlight costs one ray per voxel instead of
  one per pixel** — worth 1-2 ms on the 1660 Ti even with path tracing off.
- **The remaining flicker at distance** — too few samples once a voxel is small
  — he fixes with **à-trous wavelet filtering**: blur before accumulating, but
  never across object boundaries, so edges stay sharp.

**The pipeline, in his order:** beam pass → primary pass (unlit colour, hash-map
keys) → seed each slot from last frame → clear last frame's keys → direct
lighting, one ray per voxel → indirect lighting at half resolution → four
à-trous iterations → accumulate into the hash map → composite → upscale and
anti-alias.

---

# 20. Coding rigid body physics for voxels

**2024-07-31.** [byP6cA71Cgw](https://www.youtube.com/watch?v=byP6cA71Cgw)

**What he built.** The **second** physics engine, replacing the one from devlogs
11-13. His verdict on the first: "laggy, buggy, and worst of all physically
inaccurate" — it conserved neither energy nor momentum, so a thrown box would
not push the box it hit.

**Collision detection, after Teardown.** He credits a public tech talk by
Teardown's author Dennis Gustafsson.
- The old way tested every nearby pair of voxels with a full separating axis
  test, so a box resting on the ground produced a contact for **every voxel on
  its bottom face**.
- The insight he takes: between two polyhedra, a touch is always **edge to edge
  or vertex to face**. So he classifies voxels once, when an object spawns, into
  a **physics acceleration structure**: a voxel is a **corner** if along some
  axis it is not flanked on both sides, and an **edge** if it is flanked on both
  sides along exactly one axis. Detection then tests one object's corners
  against the other's voxels, both ways round, and edges against edges. The box
  on the ground now produces **four contacts, at its corners**.
- **And the per-voxel test gets cheaper too.** Instead of a box-box separating
  axis test, he transforms one voxel's centre into the other volume's frame and
  checks **the eight voxels nearest that point**. This ignores the individual
  voxel's rotation — "voxels end up behaving kind of like tiny little spheres" —
  which he says is not even a bad thing, since objects roll and slide more.

**The solver.** The old engine simulated **one object at a time**, treating
everything else as fixed, so no force ever transferred: a single voxel resting
on a tree pinned the whole tree to the ground.
- He learned the replacement from **Ian Millington's *Game Physics Engine
  Development*** (he notes he is not sponsored) — a **two-phase iterative
  solver**, position and then velocity, both proceeding the same way.
- Each step resolves the **worst remaining contact**, then updates every other
  contact for the movement it caused, and repeats until the list is resolved.
  The mathematics is ordinary conservation of momentum and energy; the
  difficulty is that contacts interact.

**His advice, from having done it twice:** build the 2D version first. "It's
honestly what I should have done the first time."

---

# 21. Creating a modding system with Rust and WebAssembly

**2024-08-30.** [fvxOI0nQsTA](https://www.youtube.com/watch?v=fvxOI0nQsTA)

**What he built.** Mods as WebAssembly modules, written in Rust and compiled to
wasm.

- **Why wasm:** it is **sandboxed**, so a mod cannot read memory or touch the
  file system — he names the Minecraft mods that turned out to be malware — and
  it is **platform independent**, so one upload runs on every client including
  the browser.
- **How a mod is written:** the engine defines traits on the host (his example
  is frame timing); macros let a wasm module call them. A mod is a Rust project
  of a few lines.
- **Hot reloading:** recompile the mod and the engine notices and swaps it live.
- **What mods can do so far:** read input and draw UI. **The player controller
  is now a mod** — movement and look are no longer engine code. Actions are
  declared by the mod with default bindings, and the engine owns rebinding, so
  mods never deal with settings. Gamepads work. UI is drawn with `egui`.

**Note what is missing**, because it drives the next episode: **no API for
creating or editing voxel data**, in a voxel engine.

---

# 22. Grass, textures, and a new codebase

**2025-02-16, five and a half months later.** [YTZBFz3Et40](https://www.youtube.com/watch?v=YTZBFz3Et40)

**What he built.** A third rewrite. This is the episode where he explains what
went wrong with the previous design, and it is two things.

- **WebAssembly was the wrong boundary.** Debugging compiled wasm is poor, so
  mod authors get bad feedback; and a wasm module lives in **its own address
  space**, so host and mod can only exchange numbers and byte arrays. Sharing an
  object or a resource, and getting it collected properly, was cumbersome —
  which is exactly why the voxel-editing API never got written.
- **Explicit per-voxel normals were the wrong data.** Storing a normal in each
  voxel made every edit a nightmare: delete a cubic region and you must fix the
  normals of everything newly exposed. Defining what should happen on a fill, or
  a copy between volumes, was "such a huge headache that I didn't want to deal
  with it". **His engine had no copy-region operation at all**, and that is why
  the building system he kept promising never appeared.

**What he changed.**

- **Rust core, C# front end.** The voxel data, graphics and physics stay in Rust
  for speed and SIMD; entities, networking and game logic move to a **C# API**.
  Mods then live in the same address space, and he and mod authors use the same
  API. He compares it to Unity's C++ back end with a C# scripting front.
- **Normals are implicit**, generated at run time instead of stored. Easier
  authoring, simpler data, less memory.
- **The structure has a name.** "In my previous engine I used a structure called
  a 64-tree, or I think commonly we're now calling it a **contree**" — 4x4x4
  groups merged into a larger group. Ripping the stored normals out of it freed
  room for other data. "Contrees are the best voxel data structure that I've
  found for this kind of thing."
- **The editing operations finally exist:** fill a region, copy between or
  within volumes, and a small CSG library that rasterises spheres, cylinders and
  tori, with an optional mask. About **4,000-5,000 lines of Rust**, because none
  of them may iterate voxel by voxel — they all operate on sparse groups.
- **Rendering additions:** per-voxel normals computed at upload time, then
  **triplanar mapping** of real PBR textures ("I'm a programmer, I don't do
  art"), baked and cached on the GPU; **displacement** on CSG surfaces, for
  bricks; **grass and leaves as decorations** — a surface voxel of the right
  material generates blades or leaf clusters into an auxiliary buffer, which are
  rasterised and composited over the ray-marched image, and waved by wind while
  no voxel moves; absorptive transparency; and basic volumetric light shafts.

---

# 23. Adding global illumination to my game engine w/ DDGI

**2025-06-28.** [L1vhle74AEU](https://www.youtube.com/watch?v=L1vhle74AEU)

**What he built.** Indirect light again, but this time **DDGI** — dynamic
diffuse global illumination — chosen over the path tracing of devlog 19 because
it is less noisy and more efficient.

- **The idea:** probes scattered near surfaces store incoming light in every
  direction; a surface samples the probes around it and blends.
- **Light leaking, and the fix:** a point behind a wall may otherwise take light
  from a probe in front of it. Each probe therefore stores **a depth map** as
  well, and a probe whose nearest surface in that direction is closer than the
  point being shaded is excluded.
- **Infinite bounces, cheaply:** each update casts random rays and shades what
  they hit **with the previous iteration's DDGI output**.
- **Probe placement is where the work went.** Models are split into a grid of
  **16³ voxel cells**, one probe per cell, offset within it. The placement walks
  the **contree** breadth-first to find the largest empty leaf, preferring
  subnodes near the cell's centre, and places the probe at its centre — an empty
  cell gets a central probe, a partly full one gets a probe pushed to the side,
  a full one gets none. The result is downsampled into **four LODs**.
- **Per frame:** a compute shader marks probes active only if there is geometry
  in their own or the six adjacent cells, or an unaligned object's bounds
  overlap, and pushes survivors to a work list with GPU atomics. **A fixed ray
  budget is then divided among the active probes, so cost stays flat** however
  many probes exist. Rays follow a Fibonacci sphere.
- **What a probe stores:** irradiance, encoded by **octahedral mapping** into
  2D textures, plus average depth the same way. Shading a voxel takes the eight
  nearest cells, trilinear weights, then extra weights for line of sight and for
  probes behind the surface.

He calls it "one of the most complicated things I've implemented and certainly
the most complicated thing to explain", and cites three papers.

---

# 24. Adding UI to my hybrid C# game engine was surprisingly tricky

**2025-08-16.** [LLKCnN5a_FY](https://www.youtube.com/watch?v=LLKCnN5a_FY)

**What he built.** A month on a UI system — deliberately a side quest, told as a
casual rant.

- **The bind:** business logic is C# now, and for a custom game engine the only
  real C# option is an ImGui binding. He rejects it on two grounds — **the API
  is unsafe**, and undefined behaviour is unacceptable in an engine that runs
  untrusted mods, and **it hangs off one global state object**, a footgun in an
  engine with a server thread and a client thread.
- **What he wanted:** `egui`, which he calls a staple of the Rust ecosystem and
  an exemplar of a foolproof API — a window takes a closure, so the code's scope
  *is* the UI's scope, and you cannot forget to call `end`. His general rule:
  **minimise the number of ways to make a mistake, by enforcing the constraint
  statically**.
- **The problem:** `egui` has over 200 types and 2,000 methods. By hand at ten
  bindings an hour that is 200 hours — two and a half months at his rate.
- **How he generated them instead**, in two parts:
  - **Talking across the boundary:** `egui`'s types are mostly plain data and
    implement serde's `Serialize`, so he passes them as serialised bytes, and
    uses the `serde-generate` crate to emit the C# type definitions.
  - **Finding the functions:** Rust has no reflection, so he takes **rustdoc's
    JSON output** as the metadata and generates from it — "a little bit jank",
    and good for about **80%** of the API; the remaining 400-500 he wrote by
    hand.
- The result is open source as **egui.net**, offered back to the C# community.

---

# 25. Prototyping physics and procedural generation

**2025-10-13.** [pY7Y2pSCnGo](https://www.youtube.com/watch?v=pY7Y2pSCnGo)

**What he built.** Two months of prototyping, shown as a progress report rather
than a feature.

- **What was wrong with the second physics engine** — his own list, and it is
  the brief for devlog 26: the simulation **was not stable**; only one or two
  boxes could be stacked before they jittered apart; and a pile of objects in a
  corner would jitter, lag, and eventually **explode and clip through each
  other**, the solver failing outright. "Teardown doesn't have these problems.
  So I knew I could do better, too."
- **His method: a toy 2D engine first**, called **Rigid Pixels**, open source, so
  he can swap collision detectors and solvers modularly and see which is stable
  — the advice he gave at the end of devlog 20, now taken.
- **The test scene that convinced him**, which he calls *the tumbler*: 49 small
  cubes in a larger hollow cube fixed in place so it can only rotate. Stacked
  heavily, the simulation does not explode; the cubes settle and stay; rotating
  the tumbler or pulling one cube propagates force through the layers.
- **Terrain generation**, restarted, with curves tuned to match Minecraft's —
  hills, ravines, 3D overhangs. Most of the work is API design, because he wants
  other people writing generators.
- **Someone else's contribution:** another voxel developer, Kelvin, built a
  Minecraft-style build system in the engine's C# layer and a whole house with
  it — in one night — with sub-metre fence posts, tinted voxels, a reflective
  floor and sculpted bushes.

---

# 26. My voxel physics engine was BROKEN - here's every fix

**2026-05-03, nearly seven months later.** [R9bror0oqR0](https://www.youtube.com/watch?v=R9bror0oqR0)

**What he built.** The **third** physics engine — the one BEVOX's design is
drawn from. He organises it as features, then the detector, then the solver.

**Features.**
- **A fulcrum scene:** two heavy objects balanced on a beam over a box; removing
  one topples it. He is explicit about why it pleased him — balancing shows
  forces propagating and cancelling correctly, and toppling shows torque.
- **Stacking:** about **six boxes** of just over half a metre; the seventh
  wobbles. Mixed piles hold together, nothing explodes — and he closes a GitHub
  issue from a user (Delayeth) about objects clipping through walls when piled
  into a corner.
- **Per-material attributes: density, friction and restitution, per voxel.**
  Previously friction was a single global coefficient. A single-voxel object can
  now slide or bounce differently depending which part of it lands. He expects
  density to matter most once there is fluid.
- **Fracturing** — a thrown box splinters. Work in progress; the goal is
  "immersive destruction on par with Teardown".

**The collision detector.**
- Voxels are labelled **corner, edge, face or interior**, because a collision can
  only be edge-edge, or corner against corner, face or edge. That narrows the
  search.
- **What was janky before:** to test two nearby voxels he translated one centre
  into the other's frame and compared **axis-aligned bounding boxes** — which is
  **not invariant under rotation**, so turning an object slightly changed the
  box and produced wrong normals and jitter.
- **What Teardown does:** treat the voxels as **spheres** for that final test,
  since spheres are rotation-invariant, clamping the normal outward for a face
  voxel. Its cost, which he demonstrates in his 2D toy: objects **visibly sink
  into one another** as one body's spheres drop into the gaps between the
  other's, and the unfilled volume adds **excess friction**.
- **His improvement: round off the corners and edges of a voxel but leave its
  faces full.** The mathematics is barely heavier than sphere-sphere — a few
  extra plane-sphere tests, plus a cylinder-cylinder case for edge against edge.

**The solver.**
- **The old loop** was split impulses: apply forces, integrate velocity **and
  position**, detect collisions, fix velocities, then fix positions. Its flaw is
  structural — a box standing still is pushed down by gravity and back up every
  frame, and in floating point that is jitter.
- **The fix in principle: integrate position only after the velocity constraints
  are solved.** Then a resting box has zero velocity when its position updates,
  so its transform does not change at all.
- **The structure he chose, after trying PGS and NGS, is TGS** — temporal
  Gauss-Seidel. Detect collisions between body pairs **once**, then loop several
  times over: integrate forces, solve velocity constraints against the contacts
  already found, integrate positions. Several steps in one, reusing the
  contacts, converging faster.
- **Warm starting**, the other convergence win: begin each contact's normal
  force from **last frame's value** rather than zero. He notes this is awkward in
  a conventional engine because each contact needs a stable identity — "but with
  voxels this is very easy. You can just use the coordinate pair of the two
  colliding voxels as the key in your force hash map." It makes the difference
  for stacking.
- **Performance work**, listed but deferred to a future video: a hand-written
  multithreading library with a **lock-free scheduler**, **graph colouring** for
  contacts, and damage accumulation for fracturing.

**Elsewhere in the engine**, since devlog 24: the core voxel data structure was
refactored and every modification algorithm rewritten, more composable and **2×
faster**; Kelvin's build system landed; **voxel objects can now be edited like
the main grid**; and there are more unit tests.

---

# 27. Rayon is NOT for games - use this instead

**2026-06-03.** [QFQkqFSg8Z4](https://www.youtube.com/watch?v=QFQkqFSg8Z4)

**What he built.** His own lock-free thread pool, **micropool**, after Rayon gave
him lag spikes. He opens by admitting the title is clickbait and that Rayon is
mature and useful.

- **The systemic complaint: Rayon is designed for throughput, not latency.** The
  main thread schedules 5 ms of physics, every worker is busy with world
  generation, and the main thread waits 20 ms. For a command-line tool churning
  data that is fine; for a game it is the difference between smooth and
  stuttering.
- **Two mechanisms behind it.** An external thread that schedules work **does not
  participate in it**, so there is no bound on how long it waits, and it is
  likely to be context-switched out. And a worker waiting on a sub-iterator will
  **pick up any other task**, including a 50 ms world-generation job.
- **The measurement.** 1,000 settled boxes, in Tracy: **12 ms a tick with Rayon**,
  with the main thread idle the whole time after dispatching its islands, and
  one worker stuck with the single large island. Adding another worker does not
  fix it — the main thread has other work to do, and oversaturating the CPU
  brings its own lag.
- **What micropool does differently:** the scheduling thread **helps complete the
  work**, even when it is external; and a thread waiting on results will only
  take work **spawned from the same root task**, so physics can never be stalled
  by world generation. An atomic bit mask separates time-critical jobs from
  background ones.
- **Lock-free, after trying not to be.** His first version was a tree of jobs
  behind a read-write lock, and the profile was full of contention that worsened
  with every added thread. The rewrite is atomics throughout: a fixed array of
  job slots, each with an **atomic count of unstarted work units** — a worker
  claims one with a `fetch_sub` and checks the sign — and a second counter of
  unfinished units that reaching zero tells the scheduler the job is done.
  Bookkeeping of free slots and root tasks is a bit-packed atomic array.
- **The result: 12 ms → 8 ms**, by swapping the thread pool alone. Unit tested,
  and run clean under Miri.

---

# 28. How I made voxels SHATTER on impact

**2026-06-30.** [lsTHpbEN0dE](https://www.youtube.com/watch?v=lsTHpbEN0dE)

**What he built.** Fracturing, the feature devlog 26 showed as work in progress.
Two halves: deciding where a crack happens, and applying a pattern.

**Fracture events.**
- Inside the solver, every contact point is checked against the materials of the
  voxels touching there; **an impulse over the material's threshold raises a
  fracture event**.
- **It must be impulse, not force**, and he spends real time on why. A collision
  resolves within one tick, so the same 4 N·s at 10 ms a tick reads as 400 N and
  at 5 ms as 800 N: **the force depends on the frame rate, the impulse does
  not**. He notes the irony — real materials break on maximum *force*, following
  a stress-strain curve, but he does not simulate deformation, so force is not
  available to him.
- **The impulse at those contacts is then reduced**, so the bodies keep some
  velocity into the next tick — the rock must continue through the window it
  just broke. He calls this momentum preservation "what makes or breaks the
  realism here".

**Applying the pattern.**
- His first designs planned the pattern, labelled the pieces and copied them into
  new volumes. Then he realised the engine already had the machinery: **the
  neighbourhood disconnector**, which spawns a rigid body whenever voxels come
  loose from the grid — the thing that fells trees.
- So fracturing is just **drawing cracks by setting voxels empty** around the
  impact, and letting the disconnector produce the pieces. Modular, and the
  fastest option he tried, since the disconnector runs anyway.
- **The patterns are data, and customisable.** A pattern is a **boolean voxel
  volume** — true means delete — so a modder can author one. Each material
  carries a **fracture table**, and the engine picks a pattern from it by impulse
  strength plus randomness. Each pattern is expanded into **six cached copies,
  one per cardinal direction**. Stock helpers generate patterns from planes and
  **Worley noise**; rock breaks chunky, ice into shards. All of it is in the
  public C# API.

**Also:** the collision detector is now exposed to C#, which lets him stop
players placing voxels inside a physics object — previously an easy way to break
the game — and is the groundwork for a character controller.

---

# 29. Coding a character controller

**2026-07-31.** [HlCVvG7_HFY](https://www.youtube.com/watch?v=HlCVvG7_HFY)

**What he built.** A kinematic character controller, and Minecraft's auto-jump
"on steroids", because a voxel world has no slopes — only jagged cubes, and a
20-voxel-tall character must not stop dead at a one-voxel bump.

- **Why kinematic, not rigid body:** custom motion such as stair-stepping is hard
  to get out of Newtonian dynamics; and his rigid-body simulation **runs only on
  the server**, while responsive movement must run on the client. A controller
  built from collision *queries* can.
- **The core loop:** the player's hitbox is fed to the collision detector, which
  returns contacts; each contact is a plane, and the desired velocity must be
  **clipped to be tangent to or away from every plane** — "an interesting
  quadratic programming problem", solved with a couple of loops. Then sweep the
  hitbox along that velocity in steps just under a voxel, clip again on contact,
  and repeat until the time step is spent.
- **Stairs, from Quake's source**, which he went and read as "old, but very
  battle tested and simple to understand". Quake tries the plain slide *and* a
  move up by the step height, across, then down, and takes the latter if it ends
  on solid ground having travelled further.
- **His two modifications**, because he cannot control the geometry users build:
  try **several step heights in one-voxel increments**, not just the full one —
  otherwise the character overshoots a low doorway and hits its head — and
  **stop at the first result that preserves the full velocity** rather than
  comparing all of them, which cuts collision queries.
- **Stepping down comes free:** every motion, including the one that steps
  nowhere, ends with an attempt to move down a full step height.
- **Friction and restitution per material** come straight from the rigid-body
  code.
- **A first-person camera made him nauseous** — the eye teleports voxel by voxel
  — so the camera's height follows a **critically damped spring**, which by
  construction never overshoots.
- **The API is the point.** He spent extra days turning the prototype into a
  clean one: an **immutable `CharacterMover` value type**, constructed fresh each
  frame from the world and last frame's state, whose helpers (friction,
  restitution, sweep, step) each **return a copy** rather than mutating. That
  makes the multi-path stepping algorithm easy to write, and users extend it
  through C# extension methods. All of it was written **using only the engine's
  public C# API** — which he treats as validation that the architecture works.

Also the episode where he starts a Patreon.

---

# 30. Adding joints to my physics engine

**2026-09-11.** [RvhYKj9kEP8](https://www.youtube.com/watch?v=RvhYKj9kEP8)

**What he built.** Joints, under **one uniform solve** — the most directly
reusable piece of mathematics in the series.

- **The taxonomy:** joints remove one to three degrees of freedom. Linear — fixed
  to a point (weld), to a line (prismatic), or within a radius (distance).
  Angular — about one axis (revolute), or within a cone. **His engine supports 16
  types, each a combination of one linear and one angular component.**
- **A joint is an error function `C`.** It takes the constrained bodies'
  positions and rotations and returns how violated the constraint is; the joint
  is satisfied when `C = 0`. His worked example is a box held 16 voxels from the
  origin: `C = sqrt(x² + y²) − 16`.
- **He points out `C` is an SDF** — implicitly describing the allowed
  configurations as an SDF describes a surface — **with one difference: `C` needs
  no units**. `x² + y² − 16²`, with no square root, is an equally valid
  constraint. Any smooth function will do, and it may be written against the
  components of a body's quaternion as readily as its position.
- **The derivation, which is the heart of the episode:**
  - `J`, the gradient of `C`, points perpendicular to the level set, so the body
    may move freely in any direction perpendicular to `J`.
  - `J · V` is the rate of change of the constraint error, and must be **zero**.
  - The constraint force acts along `J`, so `F = Jᵀλ`.
  - Substituting `F = ma` as a change in velocity, multiplying through by `J`,
    and cancelling `J · V_final = 0`, gives **λ = −(J M⁻¹ Jᵀ)⁻¹ (J · V)**.
  - Written with transposes and inverses because it generalises: with many
    bodies and many constraints the `J`s and `M`s are matrices, and `J` is the
    **Jacobian**, the multivariate gradient.
- **Every joint in the video is solved by that one expression.** "The only
  difference between them is that `C` and `J` are computed differently for each
  constraint."

**What he shows it doing:** ragdolls; doors on hinges with friction, so a swung
door slows; **mouse dragging as a joint** — previously an explicit Euler spring,
which made grabbed objects bounce, and now a joint constraining both position
and rotation, stable enough to stack boxes with; and motors driving mechanical
linkages — a four-bar linkage, a crank and slider, a Scotch yoke, a radial
engine of three pistons, and a clock whose joints drive to the current time.

**The voxel-specific part**, easy to miss: **a joint follows the body it is
attached to.** Cut the piece of world holding a door's hinge and the joint
transfers to the rigid body that detachment just spawned, so the door stays
hung; cut the hinge out of the door itself and the two become separate objects.

---

# What BEVOX takes from which devlog

Pointers only, so a reader of this bundle can get from a devlog to the thing it
produced here. No comparison is intended; see the plan behind each for what was
actually built and measured.

| Devlog | Taken up in |
|---|---|
| 12, 13 — connected components over octree nodes; merging debris back into terrain | [detachment over nodes](../superpowers/plans/2026-09-24-bevox-detachment-over-nodes.md), [sleeping and merging](../superpowers/plans/2026-09-19-bevox-sleep-and-merge.md), [terrain-only merging](../concepts/terrain-only-merging.md) |
| 15 — the fullness grid | [ambient occlusion](../superpowers/plans/2026-09-24-bevox-ambient-occlusion.md) |
| 17, 18 — the 64-tree, its child mask, DDA inside a brick, the beam optimisation | [GPU traversal](../superpowers/plans/2026-09-13-bevox-gpu-traversal.md), [traversal optimisation](../superpowers/plans/2026-09-13-bevox-traversal-optimisation.md) |
| 20 — corner and edge voxels, voxels as rounded shapes | [milestone 2](../superpowers/plans/2026-09-18-bevox-physics-milestone-2.md) |
| 26 — TGS, warm starting keyed by voxel pair, per-material density, friction, restitution | [milestone 2](../superpowers/plans/2026-09-18-bevox-physics-milestone-2.md), [friction and restitution](../superpowers/plans/2026-09-18-bevox-physics-friction-restitution.md) |
| 30 — `C`, `J`, and one λ for sixteen joint types | [Dwyer's joints](../superpowers/plans/2026-09-18-bevox-physics-dwyer-joints.md), [joint block conditioning](../concepts/joint-block-conditioning.md) |
| 28 — fracture by impulse, cracks drawn as empty voxels, the disconnector doing the rest | not built |
