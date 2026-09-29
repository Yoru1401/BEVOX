---
type: Reference
title: 'How far BEVOX has drifted from Dwyer'
description: 'All thirty devlogs against what BEVOX actually does: what was taken, what was adapted with a reason, what diverged with no reason recorded, and what was never in scope.'
tags: [dwyer, provenance, drift, reference, audit]
generated: { by: claude-opus-5/claude-code, at: 2026-09-29T00:00:00Z }
sources:
  - id: devlogs
    resource: /reference/dwyer-devlogs.md
    title: "Douglas Dwyer's voxel engine, devlog by devlog"
  - id: raymarcher
    resource: /superpowers/specs/2026-09-13-bevox-raymarcher-core-design.md
    title: BEVOX ray-marcher core — design
  - id: bodies
    resource: /superpowers/specs/2026-09-15-bevox-rigid-bodies-design.md
    title: BEVOX Rigid Bodies — Design
---

# The verdict

**BEVOX is not a reimplementation of Dwyer's engine, and the drift is not evenly
spread.**

| | |
|---|---|
| Devlogs BEVOX draws on at all | **16 of 30** |
| …substantially | **10**: 12, 13, 15, 17, 18, 20, 22, 26, 28, 30 |
| Never in scope — a different product or a direction he abandoned | **12** |
| Deviations from what he shows, **with a reason on record** | **10** |
| Deviations with **no reason recorded anywhere** | **7** |

- **The physics tracks him closely.** Devlogs 20, 26, 28 and 30 are followed
  structure for structure: the classification, the rounded shapes, TGS, warm
  starting by voxel pair, one λ for every joint, cracks as deleted voxels. Every
  place BEVOX overrules him there now has its reason written down.
- **The data structure tracks him closely.** 17, 18 and 22 are taken whole.
- **The renderer has drifted furthest, and silently.** Six of the seven
  unrecorded divergences are in rendering, and **one of them is the direct cause
  of the body cap.** See below.
- Most of what looks like divergence is not: twelve devlogs are about
  multiplayer, mods, a browser build, a C# API, UI, terrain generation, global
  illumination, a character controller or his own thread pool. BEVOX's specs
  exclude or defer all of it deliberately.

# The seven divergences with no reason on record

These are the drift. Each is a place BEVOX does something different from what he
demonstrated, where no spec, plan or concept says why.

## 1. Many objects are marched separately, not stepped together — devlog 2

**The most consequential item on this page.** His first renderer sorted the
objects a ray might hit, which **capped a ray at four or eight objects because
GPU code cannot allocate**. He replaced it with *stepping*: take the shortest
step to the next voxel, and if another object offers a shorter one, continue in
that object instead. One interleaved traversal over every volume.

BEVOX marches the static world and then **each body in turn, keeping the nearest
hit** — the sorted-per-object shape he abandoned — and visits them in **table
order**, so the running nearest hit tightens late and most traversals only
narrow an answer another one already had.

**Corrected 2026-09-29, after reading the shader rather than the flag docs.**
This entry first said "`1 + N` unaccelerated marches from the ray origin", and
that overstates it. Three of the four things one would reach for are already
built: `traverse_at` seeds its first frame at the body's **bounding-box entry**,
not the camera; `compose_bodies`'s `limit` tightens on every hit so a body behind
the nearest one exits on its first step; and `BODY_RECT` skips a body for pixels
outside its screen rectangle before anything is read or transformed.

What is genuinely missing is narrower, and one half of it is not Dwyer's at all:

- **Ray order**, which is his devlog-2 point, and the early break that makes cost
  sublinear in the body count.
- **A bounding-sphere reject on the primary path.** `GpuBody::bound` exists and
  its own comment says it is "filled in only for the shadow casters; the marched
  table leaves it zero". So an optimisation this project built, measured at **20
  ms a frame**, and documented is applied to one of its two ray types. That is
  an asymmetry inside BEVOX, not a divergence from him, and it is the cheaper
  half.

Both are specified in
[body composition in ray order](../superpowers/specs/2026-09-29-bevox-body-composition-design.md).

## 2. Sunlight costs a ray per pixel, not a ray per voxel — devlog 19

Because his per-voxel hash map already enumerates the visible voxels, he shades
**direct sunlight once per voxel instead of once per pixel**, and measured it at
**1–2 ms on a 1660 Ti even with path tracing off**. BEVOX casts one shadow ray
per pixel, then deliberately snaps the ray's origin to the voxel centre so a
whole face is lit or shadowed together — the same *look*, at per-pixel cost.

So BEVOX already pays for a per-voxel result and gets it per pixel. Unrecorded.

## 3. Shadows are recomputed every frame — devlog 7

He recomputes shadows **only when the scene changes**, and splits static geometry
from small dynamic objects so that optimisation survives moving casters. BEVOX
recomputes everything every frame. Defensible for a marcher rather than a
rasteriser, but never written down as a choice.

## 4. No 2D prototype, and a far lighter validation scene — devlogs 20, 25

He ends devlog 20 with "build the 2D version first — it's honestly what I should
have done the first time", and in devlog 25 he takes his own advice: a 2D toy
called *Rigid Pixels*, so detectors and solvers can be swapped and compared for
stability.

BEVOX went straight to 3D. And his validation is *the tumbler* — 49 cubes in a
hollow rotating cube, which must settle and propagate force without exploding.
BEVOX's heaviest standing gate is **three** cubes (`a_stack_of_three_stands_still`),
against his six-boxes-before-it-wobbles bar in devlog 26. The solver limit found
on 2026-09-28 — a 240:1 mass ratio diverging — is exactly the class of failure a
tumbler scene would surface.

## 5. BEVOX uses the distance field he measured and rejected — devlog 8

He built a GPU distance field, measured it, and **did not adopt it**: parallax ray
marching still won in a significant number of cases. BEVOX's distance field is
worth a measured 12.8% on top of the other three optimisations.

Not a mistake — an inversion worth recording, because it shows his verdicts are
tied to his renderer and do not transfer. BEVOX keeps the infinity-norm shape of
his field (a cube of promised-free space) but builds it on the CPU, per 16-voxel
cell rather than per voxel.

## 6. No copy-region operation — devlog 22

He names the absence of region copy as what blocked the building system he kept
promising, and devlog 22's editing API is fill, copy between or within volumes,
and a CSG library. BEVOX has `apply_sphere`, `fill_voxels` and `clear_voxels`,
and no copy at all. Nothing says whether that is deferred or unwanted.

## 7. Editing is a brush, not a two-tree paste — devlog 3

His edit is divide-and-conquer over **two** octrees — paste a source onto a
target, keeping the target where the source is empty, overwriting whole where the
target is inside a homogeneous source. BEVOX's edit path rebuilds the subtrees a
sphere touches. The same complexity class, a narrower operation, and unrecorded
as a departure.

# All thirty, against what BEVOX does

`taken` follows him. `adapted` deviates with a reason on record. `drifted` is one
of the seven above. `scope` was never BEVOX's to build.

| # | What he built | What BEVOX does | |
|---|---|---|---|
| 1 | SVO, compute-shader marcher, C#/Vulkan. Two float bugs: a ray cycling between voxels, and losing its cell | A 64-tree, WGSL. `MAX_STEPS` caps every loop for exactly his reason | taken |
| 2 | Palette materials, LODs, sun shadow ray, **per-voxel** lighting, many objects by **stepping** | Palette ✓, shadow ray ✓, per-voxel look ✓. LODs absent. **Stepping not taken** | **drifted** |
| 3 | Octree edit by divide-and-conquer over two trees; SIMD octant bit trick | Sphere brush rebuilding touched subtrees | **drifted** |
| 4 | Pivot to rasterisation + parallax ray marching; C# → Rust | Never rasterises. Follows his devlog-17 return instead | scope |
| 5 | WebRTC peer-to-peer multiplayer, internal server even in single player | None. Deferred in the spec | scope |
| 6 | `geese` ECS-plus-events, open sourced; generic octree-builder trait | Bevy ECS | scope |
| 7 | GPU page allocator over a bit string; frustum, octant and directional face culling; **shadows cached until the scene changes** | Free list per size class, stated as the deliberately simple version. Frustum cull for bodies. **Shadows every frame** | adapted + **drifted** |
| 8 | GPU distance field, infinity norm, min-blended; **measured and not adopted** | A CPU distance field per 16-voxel cell, infinity norm, kept and worth 12.8% | **drifted** |
| 9 | `geese` systems as a DAG; material id as its own type; client-side prediction | `MaterialId` newtype, same lesson | scope |
| 10 | Snow; `gvox` model import; the lesson that the engine wants infrequent large updates | `dot_vox` import. Dirty-range upload has the same shape | taken |
| 11 | SAT collision, **continuous** detection, projections reused across voxels | Rejects both. Rounded shapes, speculative contacts, a speed cap | adapted |
| 12 | Connected components over homogeneous nodes, 6-connected, **dense bitmap** of visited; the quaternion-order bug that cost three weeks | Node walk ✓, 6-connected ✓, quaternion order gated. **Hash map, not a bitmap** — a dense array is impossible at extent 4096 | adapted |
| 13 | Sleeping; merging debris **out of view**; multithreading | Sleeping ✓, merging ✓ but only `from_terrain` (Flori's call, recorded). Multithreading deferred | adapted |
| 14 | GPU terrain generation as bytecode; interval arithmetic | No terrain generation at all | scope |
| 15 | AO from **16³ fullness cubes**, trilinear between centres, never over-estimating, **one hardware-filtered read** | Same grid, same half-is-flat rule. Blend done by hand in WGSL — no sampler, because the grid shares the field's buffer | adapted |
| 16 | LODs generated at their own resolution, cached, dirty flags up the tree | None. Render distance bounded by the root extent, stated in the spec | scope |
| 17 | The **brick tree / contree**: 64 children, a 64-bit mask, ten steps with no memory read. 7 ms castle on a 1660 Ti | Taken whole, including the name | taken |
| 18 | DDA inside a brick; compile-time per-cell direction masks; the **beam** prepass, min of four or six neighbours, guaranteed conservative | All three, bit-identical. Beam takes the min of a **3×3** neighbourhood | adapted |
| 19 | Path-traced GI; per-voxel hash map as denoiser; temporal accumulation; à-trous; **direct light one ray per voxel** | No GI. **One shadow ray per pixel** | scope + **drifted** |
| 20 | Corner and edge voxels; a voxel centre into the other frame checking the **nearest 8**; Millington's iterative solver; mouse dragging; disconnected terrain becomes bodies | Classification ✓, dragging ✓, detachment ✓. Reach is `ceil(margin + 0.5)`, since 8 misses speculative contacts. Solver is TGS, as he himself moved to in 26 | adapted |
| 21 | WebAssembly mods; the player controller as a mod | None. WASM permanently excluded in the spec | scope |
| 22 | Rust core + C# front end; **implicit normals**; the name "contree"; fill, **copy**, CSG | Implicit normals ✓, name ✓. **No copy-region operation** | taken + **drifted** |
| 23 | DDGI probes, octahedral irradiance, 16³ cells, contree BFS placement, fixed ray budget | None | scope |
| 24 | `egui` bound to C# via rustdoc JSON codegen | No UI | scope |
| 25 | **A 2D toy engine first** (*Rigid Pixels*); the **tumbler** scene, 49 cubes | Straight to 3D. Heaviest stack gate is three cubes | **drifted** |
| 26 | TGS: detect once, substep, integrate position last; warm start keyed by voxel pair; corner/edge/face/interior; rounded corners and edges with full faces; per-material density, friction, restitution; no separate position pass | All of it | taken |
| 27 | `micropool`, a lock-free thread pool: 12 ms → 8 ms on 1,000 boxes | Single-threaded. Deferred until a measurement asks | scope |
| 28 | Fracture on an **impulse** threshold; impulse partly handed back; cracks as **deleted voxels** reusing the disconnector; authored boolean patterns per material, six cached orientations, Worley noise | Mechanism ✓, hand-back ✓. Threshold is a **closing speed**, because `strength` is a `u16` and his rule shatters a resting stack. Cracks are random planes, since there is no modding API to author patterns for | adapted |
| 29 | Kinematic character controller; Quake stair-stepping; damped-spring camera; immutable `CharacterMover` | Fly camera only | scope |
| 30 | Joints as `C`, `J`, one λ; sixteen types; friction and motors; the grab as a joint; joints follow a body after a split | All of it, plus a bias per substep that he shows none of | adapted |

# What the measurement says about 1 to 3

`where_a_body_s_cost_goes`, 2026-09-29, GTX 1650, 1280x720, bench camera, every
configuration interleaved in one invocation:

| | GPU ms |
|---|---|
| static world, `DEFAULT` | 13.23 |
| sixteen bodies | **23.30** |
| sixteen bodies, no body shadows | 17.83 |

**A body costs 0.629 ms and it splits down the middle** — 0.316 ms of primary
march against 0.314 ms of shadow-ray caster tests. So divergence 1 and
divergences 2-3 are worth *the same amount*, which was not knowable before.

Three consequences for the plan:

- **Neither fix alone gets sixteen bodies inside the frame.** Removing all the
  primary cost leaves 18.25 ms; removing all the shadow cost leaves 18.28. The
  budget is 16.7.
- **Ordering helps the primary half far more than the shadow half**, contrary to
  what this page first said. A shadow ray already rejects each caster by its
  bounding sphere and returns on the *first* hit rather than the nearest, so any
  occluder will do and distance is a poor proxy for which to test first. The
  remaining shadow cost is rays that genuinely pass near bodies and must traverse
  them; ordering does not reduce that. Divergence 1 is still the better first
  move, because it is cheaper and lower-risk, not because it serves both halves.
- **The static march is the real ceiling.** 13.23 ms of a 16.7 ms frame before a
  single body exists, at this camera. No work on body composition moves it, and
  any target above roughly five bodies has to face it.

Honest limits of this measurement: the per-body figures at **one** and **four**
bodies sit inside the noise — spreads of 1.0 to 3.0 ms against effects of 0.3 —
and the one-body shadow delta came out *negative*, which is impossible and is
drift. Only the sixteen-body decomposition is above the noise floor, so the claim
is the 50/50 split at sixteen, not a per-body slope.

# Divergences 4 to 7, weighed

Divergences 1 to 3 are the renderer's limit and are being worked on. These four
are not blocking anything, so each is weighed rather than scheduled.

**6 and 7 turn out to be one thing.** Both want the same missing primitive: a
node-aware operation that copies one tree's region into another. His paste is
that with a mask; a region copy is that without one. Neither exists here.

| # | Impact | Better | Why |
|---|---|---|---|
| 4 | **High, already realised** | **His** | His validation finds instability; ours find scenarios |
| 5 | None — ours wins | **Ours** | He rejected it for a renderer BEVOX does not have |
| 6+7 | **Medium, and measurable** | **His** | Detachment finds nodes and then throws them away |

## 4. No stress scene — his tumbler against our three cubes

**Impact: high, and it has already cost something.** On 2026-09-28 a 240:1 mass
ratio through a one-voxel plate made a stack accelerate *upward* at 23 voxels a
second by tick 33 — four substeps of sequential impulses cannot hold that ratio.
Nothing in the suite would have found it, because every physics gate here proves
a **named scenario**: a cube lands, a stack of three stands, a chain holds, a
lone voxel sleeps. His tumbler proves **stability under load**, which is a
different property and the one that actually broke.

**The change, and it is not a 2D engine.** His "build the 2D version first"
advice only pays before the 3D solver exists, and it exists and works; taking it
now means rebuilding what passes 117 tests. What transfers is the *scene*: some
dozens of bodies in a closed container that can only rotate, required to settle,
stay settled, and pass force through the pile when the container turns. No new
engine code — a fixture and three assertions. `MAX_BODIES` does not constrain it,
because the cap is the renderer's and this is a headless physics test.

**Better: his, clearly.** Ours is not wrong, it is incomplete: named scenarios
cannot catch a class of failure nobody named.

## 5. A distance field he measured and rejected

**Impact: none, and ours is the better call.** He built the field, measured it,
and kept parallax ray marching instead. BEVOX's field is worth a measured 12.8%
on top of the other three optimisations, because BEVOX has no rasteriser for it
to lose to.

**The change: one line in the core spec**, recording that the field is kept
against his verdict and why. Nothing in code.

**Better: ours, for this renderer.** The general lesson is worth more than the
item: **his verdicts are tied to his architecture and do not transfer.** "Dwyer
rejected X" is not evidence against X here, and this page should not be read as
though it were.

## 6 and 7. The missing two-tree operation

**Impact: medium, and it is a speed cost today, not a missing feature.**
Detachment was made 4.5× faster on 2026-09-24 by walking the tree's uniform
nodes instead of single voxels — and then `walk` **expands every cell it found
back into individual voxels**:

```rust
for cell in cells {
    for z in .. { for y in .. { for x in .. { piece.push(..) } } }
}
```

`detach` then calls `tree.clear_voxels(&piece)` and
`Contree::from_voxels(extent, &local)`. So a 2,560-voxel column is found in a
handful of cells and then handled as 2,560 entries, twice. The node structure the
walk worked to discover is thrown away at the door.

**The change:** `Contree::copy_region`, node-aware, plus a node-aware clear, and
then detachment hands over cells rather than voxels. Measurable directly against
the current path in `a_detach_is_timed`, which already interleaves walks.

**Where it does *not* help, and this matters:** `merge` writes a body into the
world voxel by voxel because the body is at an **arbitrary rotation** — each
voxel snaps to the world cell under its own centre, and no node-aware copy can
do that. Merge stays as it is. So the win is detachment and fracture, not
everything that looks like a copy.

**Better: his**, and it subsumes ours — the sphere brush becomes a paste of a
generated sphere. But that generality is worth nothing today, since every edit
BEVOX makes is a sphere. Build the primitive because detachment needs the speed;
leave the brush alone until something needs the mask.

# What to do with this

- **Before raising `MAX_BODIES`**, read divergence 1. His answer to that exact
  cap is in devlog 2 and BEVOX never took it.
- **Before optimising shadows**, read 2 and 3, which are one piece of work: sun
  visibility is per voxel, and BEVOX pays per pixel for a per-voxel result.
- **4 is the cheapest real win on this page** — a fixture, no engine code, and it
  covers a failure class the suite cannot currently see.
- **5 needs a sentence, not a change.**
- **6+7 is a `bevox_core` primitive** whose justification is detachment's speed,
  not API completeness.
- The ten adaptations are settled. They are in the rigid-bodies spec's Provenance
  section and in the concepts, each with the measurement or the constraint that
  forced it.

See [Dwyer devlog by devlog](dwyer-devlogs.md) for what each episode actually
shows; this page only compares.
