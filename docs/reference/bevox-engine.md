---
type: Reference
title: 'BEVOX as a design'
description: "What the engine is and how its parts work, by system: one sparse tree marched on the GPU, six views derived from it, a rigid-body tick at a fixed rate, the disciplines that keep its numbers honest, and what is measured and still open."
tags: [architecture, core, render, physics, reference]
generated: { by: claude-opus-5/claude-code, at: 2026-10-03T00:00:00Z }
sources:
  - id: map
    resource: /map/index.md
    title: The engine on one page
  - id: core
    resource: /map/core.md
    title: bevox_core on one page
  - id: physics
    resource: /map/physics.md
    title: bevox_physics on one page
  - id: render
    resource: /map/render.md
    title: bevox_render on one page
---

# What this is

A description of BEVOX as a **design** — what each system does and why it is
built that way — assembled from the code and the gated map pages, and arranged by
system rather than by the order it was built.

**The map pages are the short form and are checked against the source**; this is
the long form, and its prose is not checked by anything, so a claim here names the
code that makes it true wherever it can.

---

# 1. What it is

**A pure-Rust sparse-voxel ray marcher with voxel rigid bodies. Everything is
marched on the GPU from one data structure, and there is no triangle mesh
anywhere.**

Native only. WebAssembly is excluded permanently, which is a live decision rather
than a default — it is why the engine asks the adapter for what it actually has
instead of a portable baseline.

The thing the whole engine is pointed at: **cut the support out from under a
structure and the structure falls, breaks, settles, and eventually becomes part of
the world again.** Every subsystem is in service of that one sentence — the tree
so it can be edited cheaply, the derived grids so it can be drawn and simulated
after the edit, the detachment walk so the loose part is found, the solver so it
falls believably, and merging so the debris stops costing anything.

---

# 2. The voxel data structure

## The tree

A **sparse 64-tree**, called a contree — *tetrahexaconta* plus *tree*, and
structurally an octree with [two layers squashed into one](contree.md). Every
level divides by four on each axis, so a node has **64 children** and a leaf brick
is 4×4×4 voxels. The root extent
goes to **4096³** — dense that would be 69 GB; sparse, a scene is tens of
megabytes, because nothing walks empty space.

A `Node` is **16 bytes**: a 64-bit child mask, a child base, a material. It is
uploaded to the GPU unchanged, which is the point — the CPU and the shader read
the same bytes.

**The descent is the same step at every level:**

```text
child = (position - origin) / level_extent(level - 1)   which of the 64
bit   = child.x + child.y * 4 + child.z * 16            its index
mask & (1 << bit) == 0  ->  nothing here; step over the whole cube
slot  = child_base + popcount(mask & ((1 << bit) - 1))  where it lives
```

A node with no children is **uniform**: material zero is empty and a ray crosses
it in one move; anything else is solid and a ray stops at once.

**The 64-bit mask does four jobs at once**, and that is what makes the structure
pay:

- sparse child addressing, through the popcount prefix;
- empty-space skipping, because a clear bit is a whole cube the ray steps over;
- the bitmask filter — AND it with a precomputed direction mask and a zero means
  the ray cannot hit anything in this brick at all;
- the uniform collapse, which keeps construction, editing and traversal all
  sub-linear in voxel count.

The tree is **built bottom-up**, so uniform regions collapse as they are built
rather than in a later pass.

## Where the bytes live

An **arena** holds nodes and leaf voxel bytes, with a free list per size class and
the **dirty ranges** an upload needs. An edit rebuilds only the subtrees it
touches and marks those ranges; the frame then stages only what changed.

Addresses are a **chunk coordinate plus a local position**. There is one chunk
today, and the spelling is already the one streaming would need.

## Materials

A material table of up to 256 entries: colour, plus the physics columns —
density, friction, restitution, an impact strength and a crush force. Slot zero is
transparent and not pushable.

**Two break thresholds, not one**, because they are different quantities:
`strength` is the largest **contact impulse** a material survives on an impact,
and `crush` the largest **force** it survives while held. Section 6 says why.

## Editing

The sphere brush, and the voxel-list fill and clear that detachment, merging and
fracture are all built on. Edits are sparse: they rebuild the subtrees they touch,
collapse what became uniform, and mark the arena dirty.

**Normals are never stored.** They are generated from neighbour occupancy at
upload time by `normal::implicit_normal`. Storing them would mean every fill, copy
and delete must also fix the normals of everything newly exposed, and defining
that for a copy between volumes is where region operations become intractable. The
module exists so nothing is tempted to cache them.

The cost of that choice is named rather than hidden: a view recomputed at every
sample cannot accumulate anything across frames, which is why per-voxel sun
visibility has nowhere to live.

---

# 3. One canonical tree, and a derived view for each job

`Contree` is the only thing stored. **Six representations are derived from it**,
each shaped for one job, each with its own staleness policy:

| View | Built by | Kept in | For |
|---|---|---|---|
| `Features` — corners and edges | `classify::features` | `Body::features` | the points contact detection tests |
| `MassProperties` | `mass::mass_properties` | `Body::mass` | centre of mass and inverse inertia |
| `DistanceField` | `DistanceField::build` | `VoxelScene::field` | skipping empty space before the tree |
| `Fullness` | `Fullness::build` | `VoxelScene::fullness` | ambient occlusion |
| `GpuVolume` | `GpuVolume::from_contree` | the march buffers | what the shader can address |
| Normals | `normal::implicit_normal` | **nowhere** | shading, at the sample |

`VoxelScene` is that pattern written as a struct: the tree, then the views with
their invalidation state beside them.

**The rule: when a job wants the voxels in a different shape, derive a view — do
not add a field to `Node`.** The tree holds a material index and an occupancy
mask, and both earn their place in every subsystem. A new per-voxel attribute
would not, and would cost 16 bytes a node across a 512 MB budget to serve one
consumer.

**Each view declares where it may be stale, and they do not agree.** The two
coarse grids sit in the same struct, cover the same cells and upload in the same
buffer, and have **opposite** policies:

- `field_dirty` is `None` after an erase. Removing geometry only raises true
  distances, so a lagging field under-estimates — it costs speed, never
  correctness.
- `fullness_dirty` is never skipped. A lagging fullness darkens or brightens the
  wrong voxels, and there is no direction in which that is safe.

So every path that edits the world goes through one method, rather than each
caller remembering which grid forgives what.

---

# 4. Rendering

A Bevy plugin that uploads the scene and marches it in a compute shader. The
renderer depends on the core and **never on the physics** — a manifest gate keeps
it that way, because the renderer draws bodies and must not need the simulation to
do it.

It is also the deliberately replaceable crate. Bevy's rendering internals move
between releases, so everything that would break lives here rather than in the
core, and every algorithm stays testable with no GPU and no window.

## One frame

```text
  MAIN WORLD, in Update
    1  stage what changed: dirty arena ranges, lowered field cells, recounted
       fullness cells  -- a body that only MOVED needs none of this

  RENDER WORLD, in Prepare
    2  cull bodies the camera cannot see; give the rest a screen rectangle
    3  build the uniform: camera, sun, extents, flags, body count

  BEAM PREPASS, one dispatch at 1/8 resolution per axis
    4  march one ray per coarse pixel, capped at the distance where a voxel
       could slip between two beams

  MAIN PASS, one dispatch at full resolution
    5  seed the ray at the MINIMUM of the 3x3 beam neighbourhood
    6  skip the empty space the distance field proves clear
    7  march the static world, then each body, keeping the nearest hit
    8  on a hit: implicit normal, one shadow ray to the sun, ambient darkened
       by the fullness around the voxel
    9  write the pixel

  PRESENT
   10  a window-sized Sprite under a Camera2d draws the storage texture
```

**Per ray that is `1 + N` marches**, one per body, which is why the body count is
capped and why most of a body's cost is paid per pixel.

## What makes the march cheap

Four accelerations, each a flag that must be **bit-identical** on and off:

- **DDA inside a brick.** Stepping by addition — carry the distance to the next X,
  Y and Z plane and step along whichever is nearest — instead of recomputing
  ray-box distances with a division. It wants a fixed grid, so it runs inside a
  4×4×4 brick and ray-box tests take over between levels.
- **The mask filter.** 512 precomputed masks — 64 entry cells × 8 direction
  octants, 4 KB uploaded once — say which children a ray entering there could
  possibly reach. ANDed with the brick's occupancy mask, a zero skips the whole
  brick.
- **The beam prepass.** March at 1/8 resolution, then start each full ray at the
  **minimum** of the 3×3 coarse neighbourhood around it, because a pixel sits
  anywhere inside its beam's cell and the geometry it is about to meet may have
  been seen by either side. Conservative by construction.
- **The distance field.** Advance a ray through space a coarse grid proves empty,
  before it enters the tree at all.

**Bit-identity is the rule, not an aspiration.** A flag joins `DEFAULT` only once
it has measured faster *while changing no pixel*. One that changes a pixel is a
defect, not a trade-off.

## Bodies

A body is a volume placed by a **rigid transform — no scale, ever**, which is what
makes nearest-hit composition exact: a rigid transform preserves `t`, so distances
compare directly across spaces.

Three things keep bodies affordable. **Culling** drops bodies the camera cannot
see. **Screen rectangles** let a pixel outside a body's 2D footprint pay nothing.
And **shadow casters are a separate list from the marched table**, because a body
just off screen can still shadow what is on it — each caster carries a world
bounding sphere a ray rejects before any transform.

## What the shader reads

Ten bindings, eight of them storage buffers. That was the ceiling for a long time
because the engine asked for the portable baseline, which allows eight — and it is
why the fullness grid rides packed behind the distance field in one buffer.

The engine now asks the adapter for **sixteen**, having established that eight was
a portability number and not a hardware one; the adapter offers 524,288. A grid
that wants *filtering* should become a sampled 3D texture rather than a ninth
storage buffer, which is a different budget again.

---

# 5. Lighting

Deliberately small, and currently the least developed system.

- **One sun shadow ray** from the hit toward the sun.
- **Ambient occlusion from fullness**: a coarse grid counts how full of solid each
  16-voxel cell is, and the ambient term is darkened by the fullness around the
  voxel. The count costs nothing extra, because the grid is already built.
- **Implicit normals** from neighbour occupancy, generated at the sample.

There is no global illumination, no emissive material, and no denoiser, because
there is nothing stochastic to denoise.

**One known defect sits here:** the palette is sRGB values labelled as linear, so
the shadow term multiplies in the wrong space and a shadowed voxel slides toward
grey instead of losing lightness while holding its hue.

---

# 6. Physics

Units are **voxels and seconds** throughout. A body's *state* lives on `Body` in
the core, because the renderer needs it; this crate is what computes and advances
it.

## One tick

**64 Hz, fixed.** The system runs in `FixedUpdate`, so the step is the schedule's
and never the frame's — a gate pins both the schedule and the rate, because every
material constant is calibrated against 64 Hz and nothing else would notice if
that changed.

```text
  ONCE, before anything is solved
    1  drop any body wholly below the world
    2  cap every speed, and record how far each body can travel
    3  wake whatever an awake body could reach -- to a fixed point
    4  DETECT every contact                    -- once; substeps reuse these
    5  read each contact's approach speed

  FOUR TIMES, with h = dt/4
    6  gravity, drag, and whatever gameplay pushed with
    7  re-apply what each contact and joint carried   -- warm starting
    8  solve joints, with drift correction
    9  solve contacts: normal, then friction, then rolling
   10  INTEGRATE position and orientation
   11  solve again with no drift correction           -- the relax pass

  ONCE, after the substeps
   12  store what each contact carried, for next tick
   13  apply restitution, swept several times
   14  raise a fracture event for every contact that was too hard
   15  count how long each body has been still, and sleep the still groups
```

**Contacts are found once and reused by every substep.** That is what makes the
solver temporal Gauss-Seidel rather than plainly iterative.

**Three choices are load-bearing and easy to undo by accident:**

- **Position is integrated only after velocities are solved** (10 after 9). A
  resting body therefore has no velocity left when it moves, so it does not move
  at all. Correcting position in a pass of its own jitters at float precision.
- **The relax pass carries no bias** (11). Pushing a body out of the floor would
  otherwise leave the push as real velocity, and it would bounce.
- **The mouse grab is left out of the relax pass.** Its target moves, and the bias
  is how that motion reaches the body; relaxing it would stop the body dead every
  substep, and letting go would throw nothing.

**Both solve passes run exactly once per substep**, and that is a measured choice:
raising the counts was tried, cost up to 3× a tick, and did not buy stability on a
heavily loaded stack.

## What a voxel is on contact

A cube **rounded at its corners and edges, flat across its faces**. Pure spheres
let bodies sink into each other's gaps; axis-aligned boxes are not invariant under
rotation and give wrong normals when a body turns.

Which shape a voxel is depends on how many axes have a solid neighbour on **both**
sides:

| Axes enclosed | Shape | Geometry | Tested against |
|---|---|---|---|
| 0 | `Corner` | sphere | any voxel of the other side |
| 1 | `Edge(axis)` | cylinder along that axis | other edges only |
| 2 | `Face{axis}` | flat slab on the exposed axis | nothing; corners cover it |
| 3 | `Interior` | none | never touched first |

A voxel alone in space is enclosed on no axis, so **it is a sphere**, and a
one-voxel tower is cylinders standing on one. That is not an approximation to
tighten — it is what the scheme asks for, and it is why contacts need a force that
specifically opposes **rolling**, since sliding friction cannot stop a sphere.

Classification **re-runs on every edit**, not once at spawn, because bodies here
are brushed and fractured.

## Fracture, in two regimes

A contact breaks a voxel when it carries more than the material can take — and
*what* is measured depends on what kind of contact it is, because **one quantity
cannot separate the two cases.**

A stack landing and a body being crushed slowly deliver **1.06× apart in impulse**
and **18,000× apart in closing speed**. So:

| Regime | Test | Blow | Threshold |
|---|---|---|---|
| **Impact** | closing speed above `IMPACT_SPEED` | accumulated normal impulse | `Material::strength` |
| **Held** | otherwise | end-of-tick impulse × `SUBSTEPS / dt`, a force | `Material::crush` |

`IMPACT_SPEED` is a discriminator, not a dial: the two events sit four orders of
magnitude apart, so any value across that range gives the same answer, and a gate
asserts it.

**Each side is tested against its own threshold**, scaled by its own size —
`cbrt(min(voxels, SIZE_CAP) / SIZE_CAP)`, so small bodies break sooner and
anything at or past the cap is unchanged. The static world saturates at the cap,
because it is unbounded and reading it as one voxel would make terrain the most
fragile thing in the scene.

**The contact hands back exactly the excess** over the weaker side's threshold,
clamped to what it actually carried. A body that breaks something must carry on
through it, and a fixed fraction returns too much for a glancing break and too
little for a hard one.

Cracks are then drawn as empty voxels, and the piece falls out through the
detachment path — the same machinery that fells a cut structure, so fracture adds
no separate piece-finding code.

## Joints

Sixteen types under one solve. A joint is an error function `C` that is zero when
satisfied, `J` is its gradient, and every joint resolves to the same expression
with only `C` and `J` differing. Friction and motors ride on it, and the mouse
grab is a joint rather than a spring, so a grabbed body does not bounce.

A joint **follows its pivot voxel across a split**: cut a door off its frame and
the joint stays with the piece that kept the pivot.

## Detachment, sleeping, merging

**Detachment** walks the tree for terrain an edit cut loose from the ground and
turns each piece into a body. It walks **uniform nodes rather than voxels**, so a
cut into a large volume costs a few steps rather than thousands, and it tries the
downward face first — a search for ground should dive, not spread.

**Sleeping** wakes anything an awake body could reach, to a fixed point, then puts
still groups to sleep together. A sleeper costs the tick nothing.

**Merging** writes a settled body back into the world and frees its slot — but
only terrain debris, and only out of view. A body a player deliberately placed
vanishing into the terrain would read as a bug.

---

# 7. Architecture

```text
              bevox_core          the data: tree, arena, materials, and Body
                 ^   ^            itself. No Bevy, no GPU, no window.
                /     \
      bevox_render   bevox_physics    neither depends on the other, and a gate
               ^       ^              on the manifest keeps it that way
                \     /
                 bevox              the binary: window, input, brush, scenes
```

**Why `Body` is in the core and not in physics.** The renderer marches bodies and
must never need the simulation to do it. So what a body *is* — transform, mass
properties, features, warm-start impulses — is core, and everything that
*computes* any of it is physics.

**Why the renderer is the replaceable one.** Keeping every algorithm in a crate
that knows nothing about Bevy means an engine upgrade breaks plumbing rather than
the engine, and means the algorithms are testable headless.

**The two loops.** The tick runs in `FixedUpdate` at 64 Hz and never depends on
the frame rate. The frame runs in `Update` as fast as it goes. Bevy runs
`FixedUpdate` first, which is what lets a tick remove a body and have the frame
rebuild the packed buffers in the same iteration.

**A CPU reference marcher** exists in the core and is the ground truth the WGSL
shader is held to, pixel for pixel. The shader is loaded from disk at run time,
which is what makes it possible to A/B two shader *sources* in one session.

---

# 8. The disciplines

These are enforced, not aspirational, and two of them have caught defects that no
feature test would have.

**Every gate is proven by a deliberate break.** A test that has never failed is
not known to test anything. Break the thing it guards, watch it fail, restore, and
record both. This has caught **blind gates that passed for the wrong reason**, and
twice it has caught a test that *could not fail* — one that compared two values
both moved by the change, and one that searched its own source for a string its
own assertion contained.

**Every performance claim is interleaved A/B/A in one invocation**, with the drift
reported beside the difference. Cross-run drift on this hardware is routinely
larger than the effect under test, so a number from an earlier run is not
evidence. A measurement whose repeated baseline disagrees with its first is
discarded rather than reported.

**An inference is not an observation.** A claim about behaviour names the code that
makes it true — the function, ideally the line. The gates check names and counts;
they check no prose at all, and cannot, so the prose half of the documentation is
where errors live and must be written to be checkable in one hop.

**A known defect is recorded as a failing characterisation test**, `#[ignore]`d,
documented with what it records and the prediction that fixing the real cause
makes it pass. A defect described only in prose is one nobody can reproduce.

**The map pages are gated.** Their factual half — every module, constant and
shader flag they name — is checked against the source by a test, and they are
capped at a page so they stay readable.

---

# 9. The numbers

Measured at **1280×720 on a GTX 1650**, interleaved, unless stated.

| | |
|---|---|
| Static world, everything on | **13.23 ms** of a 16.7 ms frame — 79% before a body exists |
| Mean traversal steps per primary ray | **9.0**, against a shader cap of 4096 |
| A visible body | **0.629 ms**, split almost exactly 50/50 primary and shadow |
| Sixteen bodies | 23.30 ms, or 17.83 with body shadows off |
| Workgroup size | 8. 16 was inside the drift; 32 was a **+2.45 ms regression** |
| Detachment of a freed piece | 0.138 ms, from 0.26 |
| Sixteen bodies asleep | 0.0039 ms a tick |
| Solver iteration counts | 1 and 1, measured: raising them cost up to 3× a tick and did not buy stability |

**What the step count rules out.** Nine steps against a 4096 cap means the marcher
is **not step-bound** — four accelerations have already reduced the walk to almost
nothing, and work aimed at reducing steps is aimed at nine of them. The remaining
time is memory latency, the per-pixel setup every ray pays before it walks, the
shadow ray, and ambient occlusion's reads at the hit.

**The caps, and why they are where they are.** 16 bodies is a measured figure, not
an aspiration, and it moves with resolution because most of a body's cost is per
pixel. 512 MB is checked at upload, so exceeding it is an error rather than an
allocation that might fail. 4096³ is the largest volume an import will build.

---

# 10. What is open

Stated because a design document that lists only what works is not a description
of the engine.

**Stacks do not settle.** Four-high stacks of the demo materials end a run with
none of their bodies asleep, and six- and eight-high stacks of a test material end
with late speeds of 2.03 and 4.12 voxels a second and held loads swinging by a
factor of two. This is the blocking defect: it makes any load measured on a stack
a measurement of a swinging pile, which is why two of the three demo materials have
no crush threshold and why a taller-stack gate ships failing.

**An extreme mass ratio collapses.** A 240:1 ratio through a one-voxel plate falls
through the floor at about −23.6 voxels a second by tick 55. Characterised by a
test, cause not established.

**The fracture discriminator has a structural blind spot.** It switches on closing
speed, and a stack's *internal* contacts are stationary during a landing — the
contact between the bottom two cubes transmits the whole landing at a closing speed
of exactly zero, because the two are falling together. No threshold reaches zero,
and no per-tick impulse reading separates it either. The fix would be a quantity
steady over several ticks, and nothing keeps that history.

**The blow is one contact's share of a statically indeterminate split**, and the
share moves 1.7× with body size alone. That bounds the accuracy of any threshold.

**Per-voxel sun visibility has nowhere to live**, because no view is stored per
voxel — the cost of generating normals rather than caching them.

**The palette is sRGB labelled as linear**, so shading multiplies in the wrong
space.
