---
type: Reference
title: "Douglas Dwyer's voxel engine, as a design"
description: "What his engine is and how its parts work, organised by system rather than by devlog: the data structure, two renderers, four lighting systems, three physics engines, the architecture, and the method."
tags: [dwyer, reference, architecture, rendering, physics, lighting]
generated: { by: claude-opus-5/claude-code, at: 2026-10-03T00:00:00Z }
sources:
  - id: devlogs
    resource: /reference/dwyer-devlogs.md
    title: "Douglas Dwyer's voxel engine, devlog by devlog"
  - id: rigid_pixels
    resource: /reference/rigid-pixels.md
    title: "Dwyer's rigid_pixels, read from source"
---

# What this is

A description of Douglas Dwyer's voxel engine as a **design** — what each system
does and why he says he built it that way — assembled from thirty devlogs and one
open-source prototype, and arranged by system rather than by date.

**Everything here is what he states publicly.** The engine is not open source, so
nothing is read from its code; where he gives no number, none is given. His 2D
physics prototype `rigid_pixels` is public and is the one place his actual source
can be read.

**The chronological record is [the devlog file](dwyer-devlogs.md).** This document
is the other cut through the same material: what the engine *is*, rather than the
order it arrived in.

---

# 1. The product

He is building a **multiplayer voxel platform, playable in a browser, extensible
with mods** — not a game, and not a renderer. That decision is from devlog 4, and
almost every later choice follows from it:

- **Integrated GPUs are a requirement**, not a courtesy, because the browser is
  the target. This is what killed his first renderer.
- **Peer-to-peer**, because hosting servers for everyone is expensive and forces
  users to upload their worlds.
- **Mods are first-class**, and the safety of running untrusted code shapes the
  language boundary twice over.
- **Other people must be able to author things** — terrain generators, fracture
  patterns, build systems, character controllers — so API design is a recurring
  subject in its own right, not an afterthought.

He says the engine exists to make one thing possible: **cut the base of a
structure and it falls**. Trees are the demo; the mechanism is general.

---

# 2. The voxel data structure

The one piece he has rewritten least and praises most.

## The sparse voxel octree, and why he left it

He began with an SVO, chosen for two properties at once: **compression** — a 256³
grid at a byte a voxel is 16 MB, and his came in under half a megabyte — and
**traversal**, because a homogeneous branch is stepped over whole.

He abandoned it for one reason: **an octree descent costs eight or more memory
reads per step**, and GPU memory is slow.

## The contree

Its replacement splits each cube into **64 children, four per axis**. He first
called it a brick tree, having no name for it; by devlog 22 he says "I think
commonly we're now calling it a **contree**", and calls contrees "the best voxel
data structure that I've found for this kind of thing".

**What makes it fast is a 64-bit occupancy mask per node.** A ray reads that mask
once, into a register, and then steps from child to child **testing bits rather
than memory** — up to ten ray steps with no further read.

**The layout lesson from the octree carried over.** His SVO packed LOD data, a
flags byte, the full octants' materials and the sparse octants' offsets one after
another, so every descent had to compute where the offsets began. Aligning them
instead removed both the arithmetic and the buffer reads, and took the frame from
7–10 ms to 3.5–4 ms, at the cost of 30–50% more memory.

## Editing it

Rebuilding is not an option: constructing his 256³ octree from a flat array took
500–1000 ms.

**The paste algorithm** is divide and conquer over two volumes, a target and a
source — "like pasting two transparent images on top of one another". Where the
target lies wholly inside the source and the source is one homogeneous material,
overwrite it whole; otherwise split into children and recurse on the overlapping
ones. It terminates because the worst case is a single voxel against a single
voxel.

A useful detail: **a voxel's octant at level n is the n-th bit of each of its
three coordinates**, which he moved to SIMD intrinsics.

He later noticed that converting an array to a tree, merging two trees, and
generating terrain are **the same recursion with one question swapped** — which
regions share a material — and abstracted that into a trait. Because Rust
monomorphises generics, the abstraction costs nothing; an inherited type would
have put virtual calls in the hottest code he has.

## What the engine stores per voxel, and what it does not

- **A material id**, which he made its own type rather than a `u16` on the
  principle of encoding as much as possible in the type system.
- **Per-material attributes: density, friction, restitution**, and a fracture
  table. Previously friction was one global coefficient.
- **Colours per voxel**, which is what drove the return to ray marching.
- **Normals are generated at run time, not stored.** He tried storing them and
  removed them: a stored normal makes every edit a nightmare, because deleting a
  region means fixing the normals of everything newly exposed, and defining the
  behaviour for a fill or a copy was "such a huge headache that I didn't want to
  deal with it". **His engine had no copy-region operation at all** until he
  removed stored normals — and that absence is why the building system he kept
  promising did not appear for years.

The editing operations that eventually landed — fill a region, copy between or
within volumes, and a CSG library rasterising spheres, cylinders and tori with an
optional mask — are **4,000–5,000 lines of Rust**, because none of them may
iterate voxel by voxel. They all work on sparse groups.

---

# 3. Rendering, which he reversed twice

## Ray marching, abandoned (devlogs 1–3)

C#, Vulkan, a compute shader marching a 256³ volume. He chose ray tracing because
he wanted many dynamic lights, and shadow mapping costs a scene re-render per
light while a ray march costs one more ray. 7–10 ms on a GTX 1070.

**Why he left it:** it was memory-bound — it ran 30% *faster* on a 1660 Ti than on
a 1070, which has a third less bandwidth — and it fell over on low-end hardware,
which the browser target made unacceptable.

## Parallax ray marching (devlogs 4–16)

His name for the technique, after parallax bump mapping, and his answer to
"rasterise voxels without meshing every face": **rasterise a bounding box per
8×8×8 block and ray march inside it in the fragment shader**. Triangle counts fall
by an order of magnitude or more, and his mesher only emits tight boxes, so no
fragment is spent on empty space.

**Two costs he names.** A box's depth is not its voxel's, so he must write depth
in the fragment shader — which disables early-Z and made everything slow. His fix:
draw normally with early-Z on, write true depth to a separate buffer, then
overwrite the depth buffer in a second pass. And **no MSAA**, since he sets each
pixel himself; he uses FXAA instead.

**At scale** this needed a **GPU memory allocator**, because every voxel object
renders identically and so they all share one big vertex buffer and a few big
textures. Free and filled pages are a bit string, scanned 64 at a time with
bitwise operations. Each visible face costs **16 bytes** — 4 naming its place in a
materials map, 12 for four corners at three bytes each.

**Three rounds of CPU-side culling**, all resting on voxels having only six
axis-aligned face directions: frustum culling of whole chunks against a frustum
grown by a chunk in each direction; **octant culling**, since a chunk entirely to
one side of the camera can never show the faces pointing away, dropping half of
them; and directional culling for what octant culling misses. Together these leave
**20–40% of loaded triangles drawn**, issued as one multi-draw — which matters
most on the web, where each GL call crosses into JavaScript.

He also found, trying it again, that **plain greedy meshing was "quite
comparable"** to parallax ray marching.

## Ray marching again (devlogs 17 onward)

He deleted the rasteriser — **over 11,000 lines** — and went back.

**Why:** his materials worked like Minecraft blocks, a palette with a 16×16
texture each, so distant terrain was flat grey and imported models had nowhere to
map their colours. What he wanted was **per-voxel colours and per-voxel normals**,
which he calls the secret sauce of the engines he admires. A ray marcher reads
voxel data directly; a rasteriser would need triangles per unique voxel.

**The result**, with the contree's bit mask and no optimisation of the marching
loop and no depth prepass: a Teardown castle map at **7 ms a frame, primary and
shadow ray, on a GTX 1660 Ti**. He suspects he is compute-bound. Integrated GPUs
are served by a dial a rasteriser does not have — render at lower resolution.

**Then three optimisations, together a 2×** (120 ms → 60 ms on his integrated GPU,
one scene):

- **DDA instead of ray-box tests, 120 → 100.** The old step recomputed, for all
  three components, the distance to each side of the voxel's box, with a division.
  DDA carries the distance to the next X, Y and Z plane, steps along whichever is
  nearest, and adds one increment back: **one component per loop, addition only**.
  The catch he is open about — DDA wants a fixed grid, so he uses it **only inside
  a 4×4×4 brick** and falls back to ray-box tests between levels. He asks the
  audience whether anyone knows how to DDA across levels.
- **Bit-mask culling of a brick, 100 → 80.** A ray entering a brick can only reach
  a subset of its 64 cells — one travelling up and left can never meet the cells
  down and right. He **precomputes at compile time a mask per starting cell and
  set of cardinal directions** and ANDs it with the occupancy mask. Zero means the
  brick cannot be hit, skipped in a couple of instructions.
- **The beam optimisation, 80 → 60.** Render first at low resolution, then start
  each full-resolution ray at the distance its neighbours reached. The danger is a
  small voxel slipping between coarse rays; his answer is that a voxel large
  enough on screen must hit one of the surrounding coarse rays, so each fine ray
  takes the **minimum** of its four or six neighbours and the coarse pass stops
  before voxels shrink below that size. **Guaranteed conservative — the image is
  identical with it off.**

## Everything else in the frame

**Transparency** is order-independent, by weighted blending: average the
transparent colours by alpha and blend once. His first version weighted nothing,
so a red pane in front of a green one looked identical to the reverse; adding a
**weight that rises for fragments nearer the camera** recovered most of the look
of back-to-front sorting.

**Grass and leaves are decorations**: a surface voxel of the right material
generates blades or leaf clusters into an auxiliary buffer, which are rasterised
and composited over the ray-marched image, and waved by wind while no voxel moves.

Also **triplanar mapping** of real PBR textures, baked and cached on the GPU
("I'm a programmer, I don't do art"); **displacement** on CSG surfaces for bricks;
absorptive transparency; and basic volumetric light shafts.

---

# 4. Lighting, four systems deep

**Per-voxel, not per-pixel, throughout.** One ray from the voxel, averaged over
its exposed faces. He prefers the look — it reads as the shape of the thing rather
than as a pixel effect — and the choice constrains every later lighting decision,
because standard denoisers assume per-pixel shading.

**Sun shadows** are one more ray from the voxel toward the sun. "That is the real
beauty of ray tracing techniques."

## Ambient occlusion by fullness (VVAO)

He rejected both standard approaches by name. **Minecraft's per-block AO** looks
only at directly adjacent blocks, and his voxels are far smaller — he wants
shadows reaching **four to eight voxels**. **SSAO**, which he has implemented
professionally, fights his renderer twice: it shades smoothly per pixel while his
lighting is one colour per voxel, and it is stochastic, so a single rounded sample
per voxel left the scene noisy and flickering.

**The model:** ideally, draw a sphere of about eight voxels around each voxel and
measure **what fraction is solid**. On a flat surface that is exactly 50% and
nothing darkens; in the crease where floor meets wall it is about 75%, and the
excess over half is the darkening. He is explicit that this is an estimate of an
effect, not physical accuracy.

**What ships is an approximation of that approximation:** divide the world into
**16×16×16 cubes** and store how many voxels each contains. That value is the
fullness at the cube's centre, and everywhere else is **linear interpolation
between neighbouring centres** — which he notes **never over-estimates**, so no
shadow appears where none belongs.

**Why it is cheap:** the count costs nothing extra on the CPU, because he is
already walking the volume to mesh it, and on the GPU the blend is **one
hardware-filtered texture read** — the filtering is the hardware's, so there is no
interpolation code at all. Against SSAO's extra pass and eight-plus random reads
he calls it "cheap as dirt".

## Path tracing, and the hash map

Real-time indirect light, one Monte Carlo sample per pixel, working in a day or
two. He picks his **own radiance function** rather than a physical one: a ray
reaching the sky returns a constant sky colour; one hitting geometry returns an
ambient colour with **exponential decay**, so surfaces close together darken.
**Emissive voxels came for three or four lines** — a third branch returning the
light's colour — and unlike rasterisation, **area lights are free**.

**The denoiser is the unusual part.** Standard denoisers would destroy the
one-colour-per-voxel look, so he built **a per-voxel hash map on the GPU** with
atomic compare-exchange: each frame records the visible voxels as keys, each pixel
adds its lighting into its voxel's average, and a final pass combines unlit colour
with per-voxel light.

Two things fell out of that beyond denoising. The previous frame's hash map seeds
the current one, giving **temporal accumulation**. And because the visible voxels
are already enumerated, **direct sunlight costs one ray per voxel instead of one
per pixel** — worth 1–2 ms on the 1660 Ti even with path tracing off.

Remaining flicker at distance, where a voxel is small and samples are few, he
fixes with **à-trous wavelet filtering**: blur before accumulating, never across
object boundaries.

**The pipeline, in his order:** beam pass → primary pass (unlit colour, hash-map
keys) → seed each slot from last frame → clear last frame's keys → direct
lighting, one ray per voxel → indirect at half resolution → four à-trous
iterations → accumulate into the hash map → composite → upscale and anti-alias.

## DDGI

He later replaced the path-traced indirect light with **dynamic diffuse global
illumination**, as less noisy and more efficient. Probes near surfaces store
incoming light in every direction; a surface samples the probes around it.

- **Light leaking** is handled by each probe also storing **a depth map**; a probe
  whose nearest surface in a direction is closer than the point being shaded is
  excluded.
- **Infinite bounces come cheaply**: each update casts random rays and shades what
  they hit **with the previous iteration's output**.
- **Probe placement is where the work went.** Models split into **16³ voxel
  cells**, one probe each, offset within the cell. Placement walks the contree
  breadth-first for the largest empty leaf, preferring subnodes near the centre —
  an empty cell gets a central probe, a partly full one gets a probe pushed to the
  side, a full one gets none. Downsampled into four LODs.
- **Per frame** a compute shader marks a probe active only if there is geometry in
  its own or the six adjacent cells, or an unaligned object overlaps, and pushes
  survivors to a work list with GPU atomics. **A fixed ray budget is divided among
  the active probes, so cost stays flat** however many exist. Rays follow a
  Fibonacci sphere.
- **A probe stores** irradiance encoded by **octahedral mapping** into 2D
  textures, plus average depth the same way.

He calls it the most complicated thing he has implemented and the most complicated
to explain, and cites three papers (Majercik et al. twice, and Rohacek).

---

# 5. The world: terrain, LODs, and what grows on it

**Terrain generators are bytecode, not code.** A generator is a list of low-level
operations — add vectors, multiply, random — each with a text snippet; he
concatenates the snippets into shader source and compiles it. The bytecode
serialises, so generators can be authored and changed at run time. The old
generator was hardcoded CPU work, which stole cores from everything else and which
no user could ever change.

**Interval arithmetic is what makes it fast.** A generator maps a position to a
number whose sign picks the material. For many functions a *range* of inputs gives
a predictable range of outputs — `2x` over 10..20 gives 20..40, `eˣ` over a..b
gives `eᵃ..eᵇ` — and it generalises to logical operations. So he compiles **two**
shaders: an interval version that proves a region homogeneous, and a fine one that
samples voxel by voxel only where the interval version could not decide. Measured:
**no more than 50% of voxels sampled individually, usually 20–30%**, with sky and
underground chunks skipped wholesale.

**Imported models inside that scheme** need different treatment, because interval
arithmetic cannot predict a 3D texture. He **precomputes a distance field per
model**: if the field at a box's minimum corner exceeds the box's side, the whole
box shares that value. Computed on a 16× downscaled copy, with both resolutions
uploaded.

**LODs are generated, not downsampled.** The server used to load chunks at full
quality and downsample, so anything a client wanted to *see* the server had to load
and simulate. Since generators work at any scale, an LOD is now generated directly
at its own resolution, and the client arranges the world as an octree of cubes
each drawn with the same number of voxels, subdivided more finely near the player.

**The cost of that is bookkeeping about edits**, because generating an LOD from
scratch loses them — build a tower, fly away, and the generator does not know.
His fix: cache generated LODs to disk; mark edited regions dirty and propagate the
flag up the LOD octree; when a coarse LOD is requested, load it from the save,
pull finer data for any dirty sub-octant, downsample and recombine.

**Snow is his worked example of how a feature enters the engine**, and the split is
instructive: falling snow is **client-only and purely visual**, about 6,000 flakes
spawned on the CPU (ray cast so none falls through terrain) then moved and drawn
by the GPU; snow settling on the ground is **server-only and real**. Neither side
knows the other's half. His first ground-snow system placed one flake at a random
spot per tick and was laggy, because **the engine is built for infrequent large
voxel updates, not frequent tiny ones**; batching 20–30 placements per update
fixed it.

---

# 6. Physics, rebuilt three times

## The first engine: detection, then response (devlogs 11–13)

He wrote his own rather than taking PhysX, on the theory that voxel-against-voxel
could be made very fast, and says it paid.

**Detection by the separating axis theorem**, 15 axes for two boxes. Because it
projects both shapes onto each axis, the **gap** between projections says how far
they may move before touching — which gives **continuous collision detection**, so
nothing tunnels.

**The optimisation only voxels allow:** every voxel sits on the same regular grid,
so one voxel is another shifted and scaled, and projection onto an axis is linear.
He computes the projections **once for a voxel at the origin** and reuses them,
leaving one or two matrix multiplies per voxel-voxel test. Large empty nodes are
skipped whole.

**Finding what came loose** is connected-component labelling by depth-first search,
with two voxels connected if they share a **face** — diagonals do not count, which
he calls both simpler and more sensible. The image-processing algorithms are faster
per pixel but all **linear in the number of voxels**; his improvement is to **walk
the tree's homogeneous nodes as graph nodes**, since a solid 8×8×8 node is known to
be internally connected and so is one node rather than 512 with all their edges.

**Response and rotation** is where it went wrong. Resting rotation needs, in
principle, the infinite set of points where two bodies touch; his approximation
gathers **a discrete list of contact points** during detection, sums each one's
torque, multiplies by the time step. Then it did not work for three weeks —
objects "sat up and started spinning in the air, gaining infinite rotational
velocity, as though they were possessed". The cause was **two quaternions swapped
in a multiply**. One line.

**He states its limit plainly:** it is not physically accurate, because **each
object is processed individually** — a tall stack will not topple properly.

**Making it affordable** took three things. **Despawning as merging**: debris is
fused back into the terrain by an affine transform and a re-rasterise, and **only
with the player's back turned**. **Sleeping**: if an object did not move last tick
and nothing around it moved recently, skip it — collision detection for one of his
trees costs 1–2 ms, and after chopping 10–20 trees the loop took 14–15 ms against
a 25 ms budget. **Multi-threading**: ten resting trees took 23 ms of thread time
but 9 ms of wall time on four threads.

## The second engine (devlog 20)

His verdict on the first: "laggy, buggy, and worst of all physically inaccurate" —
it conserved neither energy nor momentum, so a thrown box would not push the box
it hit.

**Detection after Teardown**, credited to a tech talk by Dennis Gustafsson. The old
way tested every nearby voxel pair with a full SAT, so a box resting on the ground
produced a contact for **every voxel on its bottom face**. The insight: between two
polyhedra a touch is always **edge to edge or vertex to face**. So voxels are
classified once, at spawn, into a **physics acceleration structure** — a voxel is a
**corner** if along some axis it is not flanked on both sides, an **edge** if
flanked on both sides along exactly one axis. The box on the ground now produces
**four contacts, at its corners**.

**The per-voxel test got cheaper too**: transform one voxel's centre into the
other's frame and check **the eight voxels nearest that point**. This ignores the
voxel's own rotation — "voxels end up behaving kind of like tiny little spheres" —
which he says is not even a bad thing, since objects roll and slide more.

**The solver** came from Ian Millington's *Game Physics Engine Development*: a
**two-phase iterative solver**, position then velocity, each step resolving the
**worst remaining contact** and then updating every other contact for the movement
it caused.

**His advice at the end**, which he takes five devlogs later: build the 2D version
first.

## The third engine (devlogs 25–26)

**His own list of what was wrong:** the simulation was not stable; only one or two
boxes could be stacked before jittering apart; a pile in a corner would jitter,
lag, and eventually explode and clip through each other. "Teardown doesn't have
these problems. So I knew I could do better, too."

**His method was a toy 2D engine**, `rigid_pixels`, open source, so detectors and
solvers could be swapped modularly and compared. **His validation scene is the
tumbler**: 49 small cubes in a larger hollow cube fixed so it can only rotate.

**The detector's fix.** The old final test translated one voxel centre into the
other's frame and compared **axis-aligned bounding boxes**, which is **not
invariant under rotation** — turning an object slightly changed the box and
produced wrong normals and jitter. Teardown treats the voxels as **spheres**, which
are rotation-invariant, clamping the normal outward for a face voxel; its cost,
which he demonstrates in the 2D toy, is that objects **visibly sink into one
another** as one body's spheres drop into the gaps between the other's, and the
unfilled volume adds excess friction. **His improvement: round off the corners and
edges of a voxel but leave its faces full** — barely heavier than sphere-sphere, a
few extra plane-sphere tests plus a cylinder-cylinder case for edge against edge.

Voxels are labelled **corner, edge, face or interior**, because a collision can
only be edge-edge, or corner against corner, face or edge.

**The solver's fix is structural.** The old loop applied forces, integrated
velocity **and position**, detected collisions, fixed velocities, then fixed
positions — so a box standing still is pushed down by gravity and back up every
frame, which in floating point is jitter. **The fix in principle: integrate
position only after the velocity constraints are solved**, so a resting box has
zero velocity when its position updates and its transform does not change at all.

**He chose TGS** — temporal Gauss-Seidel — after trying PGS and NGS: detect
collisions between body pairs **once**, then loop several times over integrate
forces, solve velocity constraints against the contacts already found, integrate
positions.

**Warm starting** is the other convergence win: begin each contact's normal force
from **last frame's value** rather than zero. He notes this is awkward in a
conventional engine because each contact needs a stable identity — "but with voxels
this is very easy. You can just use the coordinate pair of the two colliding voxels
as the key in your force hash map." It makes the difference for stacking.

**What it achieves:** about **six boxes** of just over half a metre stack before
the seventh wobbles; a fulcrum scene balances two heavy objects on a beam and
topples when one is removed; mixed piles hold together.

Also in this engine: **per-material density, friction and restitution, per voxel**,
where friction had been one global coefficient.

## Fracture

**An impulse over the material's threshold raises a fracture event**, checked
inside the solver at every contact point against the materials of the voxels
touching there.

**It must be impulse, not force**, and he spends real time on why: a collision
resolves within one tick, so the same 4 N·s at 10 ms a tick reads as 400 N and at
5 ms as 800 N — **the force depends on the frame rate, the impulse does not**. He
notes the irony that real materials break on maximum *force*, following a
stress-strain curve, but he does not simulate deformation so force is not
available to him.

**The impulse at those contacts is then reduced**, so the bodies keep some velocity
into the next tick — the rock must continue through the window it just broke. He
calls this momentum preservation "what makes or breaks the realism here".

**Applying the pattern reuses what the engine already had.** His first designs
planned a pattern, labelled the pieces and copied them into new volumes; then he
realised **the neighbourhood disconnector** — the thing that fells trees — already
spawns a rigid body whenever voxels come loose. So fracturing is **drawing cracks
by setting voxels empty** around the impact and letting the disconnector produce
the pieces. Modular, and the fastest option he tried, since the disconnector runs
anyway.

**The patterns are data.** A pattern is a **boolean voxel volume** where true means
delete, so a modder can author one; each material carries a **fracture table** and
the engine picks by impulse strength plus randomness; each pattern is expanded into
**six cached copies, one per cardinal direction**. Stock helpers generate patterns
from planes and **Worley noise** — rock breaks chunky, ice into shards.

## Joints

**One uniform solve for sixteen types.** Joints remove one to three degrees of
freedom: linear — fixed to a point (weld), to a line (prismatic), or within a
radius (distance); angular — about one axis (revolute), or within a cone. Each of
his sixteen is a combination of one linear and one angular component.

**A joint is an error function `C`** of the bodies' positions and rotations,
satisfied when `C = 0`. His example is a box held 16 voxels from the origin:
`C = sqrt(x² + y²) − 16`. He points out **`C` is an SDF** — implicitly describing
the allowed configurations as an SDF describes a surface — **with one difference:
`C` needs no units.** `x² + y² − 16²`, without the square root, is an equally valid
constraint. Any smooth function will do, and it may be written against the
components of a quaternion as readily as a position.

**The derivation:** `J`, the gradient of `C`, points perpendicular to the level
set, so a body may move freely perpendicular to `J`. `J · V` is the rate of change
of the constraint error and must be zero. The constraint force acts along `J`, so
`F = Jᵀλ`. Substituting `F = ma` as a change in velocity, multiplying through by
`J` and cancelling `J · V_final = 0` gives

> **λ = −(J M⁻¹ Jᵀ)⁻¹ (J · V)**

written with transposes and inverses because it generalises: with many bodies and
constraints the `J`s and `M`s are matrices and `J` is the Jacobian. **Every joint
is solved by that one expression** — "the only difference between them is that `C`
and `J` are computed differently for each constraint."

**The voxel-specific part, easy to miss:** a joint follows the body it is attached
to across a split.

**What he shows it doing:** ragdolls; doors on hinges with friction, so a swung
door slows; **mouse dragging as a joint**, previously an explicit Euler spring
which made grabbed objects bounce and now a joint constraining both position and
rotation, stable enough to stack boxes with; and motors driving a four-bar linkage,
a crank and slider, a Scotch yoke, a radial engine of three pistons, and a clock
whose joints drive to the current time.

## The character controller

**Kinematic, not rigid body**, for two reasons: custom motion such as
stair-stepping is hard to get out of Newtonian dynamics, and **his rigid-body
simulation runs only on the server** while responsive movement must run on the
client. A controller built from collision *queries* can.

**The core loop:** feed the player's hitbox to the collision detector, treat each
contact as a plane, and **clip the desired velocity to be tangent to or away from
every plane** — "an interesting quadratic programming problem", solved with a
couple of loops. Then sweep the hitbox along that velocity in steps just under a
voxel, clip again on contact, repeat until the time step is spent.

**Stairs come from Quake's source**, which he read as "old, but very battle tested
and simple to understand": try the plain slide *and* a move up by the step height,
across, then down, and take the latter if it ends on solid ground having travelled
further. **His two modifications**, because he cannot control the geometry users
build: try **several step heights in one-voxel increments** so the character does
not overshoot a low doorway and hit its head, and **stop at the first result that
preserves the full velocity** rather than comparing all of them. **Stepping down
comes free** — every motion ends with an attempt to move down a full step height.

A first-person camera made him nauseous, because the eye teleports voxel by voxel,
so the camera's height follows a **critically damped spring**, which by
construction never overshoots.

---

# 7. Architecture

## Client and server, always

All game logic lives on a **game-server thread**, and the local player connects to
an internal server even in single player — so single-player and multiplayer are the
same code. Networked motion is interpolated.

**Peer-to-peer over WebRTC**, because in a browser the options are HTTP (too slow),
WebSockets (not peer-to-peer) and WebRTC — the standard behind Discord and Teams
video. It needs a **signalling server**, since clients cannot know each other's
addresses and must get through firewalls; two clients tell it they want to connect
and it passes each the other's connection data, and they meet through ICE. He wrote
it in Go.

**Browsers forbid native threads**, so he uses web workers plus `SharedArrayBuffer`
to get real threads for client and server.

## `geese`, the event system

He tried a Unity-style design and hit a wall: **object-oriented game code is an
interdependent graph, and Rust requires a tree** — one owner per object.

What he arrived at after three iterations is a **hybrid entity-component and event
system**. A world holds entities, components and shared resources such as the
clock. **Systems act on the world and, unlike a plain ECS, keep their own state** —
the graphics system needs somewhere to put GPU handles. Each system is a set of
event handlers.

**The revision that mattered:** in the first version systems could only talk by
raising events, and that isolation pushed him toward monoliths — one graphics
system owning framebuffers *and* meshing *and* drawing. So systems may now
**depend** on other systems, forming a directed acyclic graph, and a handler can
query a system it depends on. It is open source on crates.io, with a networking
companion `geese_pool`.

**It became multi-threaded** on a neat argument: two systems that are not each
other's dependencies cannot observe each other, so their handlers may run at once
while still appearing sequential.

On events versus ECS, he prefers events for one-off things like a player logging
in, and keeps an ECS inside a single system.

## The language boundary, moved twice

**C# to Rust** (devlog 4), for WebAssembly at near-native speed and because he did
not want cross-platform C++. He had measured the cost of the C# runtime first:
placing a tree took **11 ms in C# against 5 ms in C++**, generating an octree from
a flat array **80 ms against 40** — and notes most of his C# was already `unsafe`
pointer code, so he blames the runtime rather than bounds checks.

**Mods as WebAssembly** (devlog 21), because wasm is **sandboxed** — a mod cannot
read memory or touch the file system, and he names the Minecraft mods that turned
out to be malware — and **platform independent**, so one upload runs everywhere
including the browser. The player controller became a mod; actions are declared by
the mod with default bindings and the engine owns rebinding, so mods never deal
with settings.

**Then wasm was abandoned as the boundary** (devlog 22), for two reasons he states
directly. **Debugging compiled wasm is poor**, so mod authors get bad feedback. And
**a wasm module lives in its own address space**, so host and mod can only exchange
numbers and byte arrays; sharing an object or a resource and getting it collected
properly was cumbersome — which is **exactly why the voxel-editing API never got
written**.

**What replaced it: a Rust core with a C# front end.** Voxel data, graphics and
physics stay in Rust for speed and SIMD; entities, networking and game logic move
to a C# API, so mods live in the same address space and he and mod authors use the
same API. He compares it to Unity's C++ back end with a C# scripting front.

**He treats "written using only the public API" as validation** — his character
controller was built that way deliberately.

## UI

A month on UI, told as a side quest. The only real C# option for a custom engine is
an ImGui binding, which he rejects on two grounds: **the API is unsafe**, and
undefined behaviour is unacceptable in an engine running untrusted mods; and **it
hangs off one global state object**, a footgun with a server thread and a client
thread.

What he wanted was `egui`, which he calls an exemplar of a foolproof API — a window
takes a closure, so the code's scope *is* the UI's scope and you cannot forget to
call `end`. His general rule: **minimise the number of ways to make a mistake by
enforcing the constraint statically**.

The problem was 200+ types and 2,000 methods — 200 hours by hand. He generated them
in two parts: `egui`'s types are mostly plain data implementing serde's
`Serialize`, so he passes them as serialised bytes and uses `serde-generate` to
emit the C# definitions; and since Rust has no reflection, he takes **rustdoc's
JSON output** as the metadata. Good for about **80%**; the remaining 400–500 he
wrote by hand. Released as **egui.net**.

---

# 8. Performance engineering

**His own thread pool, `micropool`**, after Rayon gave him lag spikes. The systemic
complaint: **Rayon is designed for throughput, not latency**. Two mechanisms — an
external thread that schedules work **does not participate in it**, so there is no
bound on how long it waits; and a worker waiting on a sub-iterator will **pick up
any other task**, including a 50 ms world-generation job.

Measured on 1,000 settled boxes in Tracy: **12 ms a tick with Rayon**, the main
thread idle after dispatching, one worker stuck with the single large island.
Adding another worker does not fix it.

**What micropool does differently:** the scheduling thread **helps complete the
work**, even when external; and a thread waiting on results takes only work
**spawned from the same root task**, so physics can never be stalled by world
generation. An atomic bit mask separates time-critical jobs from background ones.

**Lock-free, after trying not to be.** His first version was a tree of jobs behind
a read-write lock and the profile was full of contention that worsened with every
thread. The rewrite is atomics throughout: a fixed array of job slots, each with an
**atomic count of unstarted work units** — a worker claims one with `fetch_sub` and
checks the sign — plus a second counter of unfinished units whose reaching zero
tells the scheduler the job is done. **12 ms → 8 ms by swapping the thread pool
alone.** Unit tested and run clean under Miri.

**Shadows are recomputed only when the scene changes**, not per frame. Static
geometry and small dynamic objects are drawn in two passes, which keeps that
optimisation alive even with moving shadow casters.

---

# 9. The things he tried and rejected

Worth listing on their own, because the reasons are specific:

- **Distance fields for rendering.** Unsigned, in the **infinity norm** rather than
  Euclidean — "the biggest possible cube you can draw" — because that is what a DDA
  over voxels can use. Generation is the hard part, which he notes other channels
  skip: the naive search is O(n³k³). **His inversion** is that instead of each
  empty voxel searching for solid, each **solid surface voxel writes its distance
  into the empty voxels around it** — one dimension less work, and interior voxels
  skip entirely since the camera is never inside geometry. **And that is a
  rasterisation problem**: writing a value into every cell of a box and combining
  overlapping boxes is what a GPU does with quads and blending, so he draws the
  quads of a cube of side 2k−1 into a 3D buffer and combines with **min blending**.
  15–30 ms CPU and 10–20 ms GPU per newly loaded chunk. **He did not adopt it** —
  parallax ray marching still won in a significant number of cases. What he kept
  was a faster octree traversal found on the way.
- **Early LODs by stopping the descent.** Nearly free to add, and he says plainly
  it did **not** buy much speed.
- **Sorting objects along a ray.** His first multi-object marcher sorted every
  object a ray might hit, which **capped a ray at four or eight objects because GPU
  code cannot allocate**. Replaced by stepping: take the shortest step to the next
  voxel, and if another object offers a shorter one, continue in that object.
- **Storing per-voxel normals.** Covered above; removed, and their removal is what
  made region copying possible.
- **ImGui, and Rayon**, both covered above.

---

# 10. How he works

The method is consistent enough across thirty devlogs to be part of the design.

- **He rewrites rather than patches**, and says so: over 8,000 lines redone in one
  overhaul, over 11,000 deleted in another, three physics engines, two renderers,
  three codebases.
- **He measures before switching languages or structures** — the C#/C++ comparison,
  the 1070-versus-1660 Ti bandwidth test, Tracy on the thread pool.
- **He reports what did not work**, with numbers, and keeps the parts that did —
  the distance-field episode ends in rejection and a kept traversal improvement.
- **He debugs by bisection and printing**: disable rotation on all axes but one,
  print between every stage, compare two objects that should agree. That is how the
  swapped quaternion was found after three weeks.
- **He reads other people's work and says whose**: Teardown's tech talk for the
  contact classification and the sphere test, Millington's book for the first real
  solver, Quake's source for stair-stepping, three named papers for DDGI, a
  commenter for debris merging, `gvox` for model import.
- **He builds the small version first when a problem is hard**, and regrets not
  doing it earlier: "build the 2D version first — it's honestly what I should have
  done the first time."
- **API design is treated as engineering**, not packaging — the immutable
  `CharacterMover`, the closure-scoped UI, enforcing constraints statically,
  encoding what you can in the type system.

Two things he has never published: **any constant from the engine** — not a
strength, a margin or a mass — and **the engine's source**. The numbers in this
document are frame times, counts and sizes. The one place his code can be read is
the 2D prototype.
