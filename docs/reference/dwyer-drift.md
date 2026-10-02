---
type: Reference
title: 'How far BEVOX has drifted from Dwyer'
description: "Every difference between BEVOX and Douglas Dwyer's engine, why each one happened, and whether it was a good idea. Six causes; only one of them is drift."
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
  - id: inference
    resource: /concepts/an-inference-is-not-an-observation.md
    title: An inference is not an observation
---

# The verdict

**BEVOX is not a reimplementation of his engine, and most of what looks like
drift is not.** Thirty-two differences, sorted by *why* rather than by devlog,
because the cause is what says whether a difference needs fixing.

| Cause | Count | Verdict |
|---|---|---|
| **A — different product.** He is building a moddable multiplayer browser platform | 6 | Right, and cheap to be right about |
| **B — different architecture.** His verdicts are tied to a renderer BEVOX does not have | 3 | Right, and it skipped two of his rewrites |
| **C — different constraints.** A `u16`, an extent of 4096, eight storage buffers, a ten-per-cent fracture load window | 5 | Forced, and correct under the constraint |
| **D — not reached yet.** Deferred, not rejected | 6 | Fine, except one |
| **E — improvements on him** | 5 | Ours is better, mostly because he taught us the problem |
| **F — oversight.** Nobody decided | 7 | **This is the drift.** Five looked worth fixing; F7 is measured and resolved, leaving four |

**Seven real problems out of thirty-two differences**, five of which looked worth
fixing. **F7 is now resolved rather than fixed** — the setting was exposed,
measured, and left where it was — which leaves **F1 to F4** worth fixing. Of
those four, **three are in the renderer** (F1 the missing bounding-sphere reject,
F2 and F3); **F4 is the stress scene**, which is physics. The seventh, F7, was a
solver setting nobody knew existed.

(The sentence here used to read "Four are in the renderer … leaves four worth
fixing", which equated two different fours: the four renderer items as counted
before F7 existed, and the four that remain after F7. They are not the same
set.)

# A — Different product

He is building a multiplayer, moddable, browser-playable platform. BEVOX is a
native single-player engine. None of this is drift; all of it is scope, and the
specs say so.

| Difference | Devlog | Why | Good idea? |
|---|---|---|---|
| No multiplayer, no client/server split | 5 | His internal-server-even-in-single-player design exists so one code path serves both. BEVOX has one player | **Yes.** That split shapes every system; paying it for nothing would be the most expensive kind of foresight |
| No mods, no WebAssembly sandbox | 21 | He sandboxes mods because Minecraft mods have shipped malware. BEVOX runs no untrusted code | **Yes**, and it is a *permanent* exclusion in the spec, not a deferral |
| No C# front end | 22 | He split a Rust core from C# logic **so modders and he share one API**. With no modders the split buys nothing and costs a boundary | **Yes** |
| No UI | 24 | He spent a month generating `egui` bindings for C#. BEVOX has function keys | **Yes** |
| No browser build | 4, 6 | Permanently excluded | **Yes, and it pays**: no WebGPU buffer ceilings, no `SharedArrayBuffer` threading, native-only wgpu features usable with no fallback |
| Bevy ECS, not his own `geese` | 6, 9 | He wrote `geese` because Rust's one-owner rule fought his Unity-style design, then rewrote it when isolation pushed him toward monoliths | **Yes**, with a hedge: Bevy's internals churn, which is exactly why `bevox_core` has no Bevy dependency and the renderer is the replaceable crate |

# B — Different architecture

He spent devlogs 4 to 16 rasterising with parallax ray marching, and returned to
ray marching in 17. BEVOX started where he ended up.

| Difference | Devlog | Why | Good idea? |
|---|---|---|---|
| Never rasterised; no parallax ray marching, no greedy meshing | 4, 7, 9, 16 | BEVOX began after devlog 17 existed, so it could read his conclusion instead of walking his journey | **Yes, and the biggest win on this page.** It skipped a language change, two renderer rewrites and the stored-normals dead end |
| Normals were never stored | 22 | Same reason. His explicit per-voxel normals made every fill and copy intractable and forced a rewrite | **Yes** |
| Keeps a distance field he measured and **rejected** | 8 | He dropped it because parallax ray marching beat it. BEVOX has no rasteriser for it to lose to, and measured it at **12.8%** on top of the other three optimisations | **Yes** — and the lesson matters more than the item: *"Dwyer rejected X" is not evidence against X here.* This page must not be read as though it were |

# C — Different constraints

Each is forced by something his engine does not face. None is a preference.

| Difference | Devlog | Why | Good idea? |
|---|---|---|---|
| The size term on the fracture threshold is calibrated at **cap-sized**, not at one voxel | 28 | His scaling runs `sqrt(min(area, 7))` up from a single unit. Every strength in the table was measured against bodies at or past the cap, so a term that multiplies up from one voxel triples all of them. Two gates then failed at any `SIZE_CAP` above 1 | **Forced, and the same rule.** `size_factor` is `cbrt(min(voxels, SIZE_CAP) / SIZE_CAP)`, running 1/3 → 1: identical behaviour for anything at or past the cap, where every strength was measured, and small bodies weaker, which is all his comment on the rule claims. **The reason has changed and so has the size of it.** The original was a ten-per-cent window between a three-cube stack's landing transient (331,306) and the load a mouse grab presses with (365,906) — one number serving both a struck contact and a held one. The regime split sent the press to `Material::crush` as a force, so that bound left `strength` altogether; `GLASS_STRENGTH` rose from 350,000 to 500,000 and the window is now 414,166 (the worst landing in the series, at eight cubes) to 607,581, **47 per cent**. A multiplicative term spanning three still needs three hundred, so the normalisation stands — but it now rests on the *collision* bound alone: an eight-voxel glass pebble carrying 405,054 against a `size_factor(8)` of two thirds. That is the one measurement to redo if anyone ever wants the rule he actually describes — [the load window](../concepts/fracture-load-window.md) |
| Cracks are random planes, not authored boolean pattern volumes | 28 | His patterns are a data format for a modding API. BEVOX has no modders to author them | **Yes for now**, and reversible: the generator is one function behind one call |
| The ambient-occlusion blend runs in the shader, not through a hardware sampler | 15 | The fullness grid rides in the distance field's buffer because the compute stage is at **wgpu's default of eight storage buffers**. No sampler, so eight reads and nine weights in WGSL | **Neutral.** It measured inside drift, so nothing was lost — but the sampler is exactly what he calls the reason his version is "cheap as dirt", and giving it up went unrecorded until 2026-09-28 |
| Visited cells are a hash map, not his **dense bitmap** | 12 | He names constant-time membership as what makes walking nodes worth doing. His world is chunked; this tree reaches **4096**, so a dense array is 6.9×10¹⁰ entries and a walk may roam anywhere within `BUDGET` | **Forced, and handled**: a cheap integer hasher recovered 17-26% of it on 2026-09-28 |
| Only `from_terrain` debris merges back | 13 | He merges any settled body. Flori's call: a body the player spawned vanishing would read as a bug | **Yes** — his rule is right for debris and wrong for anything a player put there on purpose |

# D — Not reached yet

Deferred, not rejected. Five are fine. One has started costing.

| Difference | Devlog | Why | Good idea? |
|---|---|---|---|
| No global illumination — no path tracing, no DDGI, no probes | 19, 23 | Two whole subsystems, and BEVOX has one shadow ray | **Fine.** But see F3: his per-voxel hash map is what makes his *direct* light cheap, and that part is being paid for without being collected |
| No levels of detail | 2, 16 | Accepted consequence, stated in the core spec: render distance is bounded by the root extent | **Fine**, and honestly recorded. He says plainly that LODs "did not buy much speed" when he first tried them |
| No terrain generation | 14 | BEVOX loads `.vox` files and hand-built scenes | **Fine** |
| No character controller | 29 | BEVOX has a fly camera | **Fine** |
| Single-threaded | 13, 27 | He got **12 ms → 8 ms** from his own lock-free pool, on **1,000** settled boxes. BEVOX's cap is 16, and sixteen resting bodies cost 0.0039 ms asleep | **Fine, and the numbers say so.** Threading physics here would be optimising 0.93 ms inside a 23.30 ms frame |
| **No copy-region operation** | 3, 22 | Never needed until detachment, merging and fracture each grew their own region-shaped code | **No — build this one.** Detachment was made 4.5× faster by walking uniform nodes and then **expands every cell back into single voxels** before building the body. A 2,560-voxel column is found in a handful of cells and handled as 2,560 entries, twice |

# E — Improvements on him

Places BEVOX is better, usually because he documented the problem first.

| Difference | Devlog | Why | Good idea? |
|---|---|---|---|
| Voxel classification re-runs on **every edit** | 20 | He classifies once, when an object spawns — enough for an engine whose bodies were not yet editable. BEVOX edits bodies with a brush and fractures them | **Yes**, and forced by having features he did not yet have |
| The speed ceiling is a speed, not a distance per tick | — | Our own `MAX_TRAVEL` was per-tick, so it meant a different speed at 64 Hz and 128 | **Half right, and being replaced.** Per-second was the right fix to a bug we made. But **he has no ceiling at all**: he bounds tunnelling by substepping *detection* per pair on a fixed distance margin, so cost is linear in speed where BEVOX's margin-widened lookup is cubic. [The respec](../superpowers/specs/2026-09-30-bevox-fracture-and-detection-design.md) removes the cap. **And his own name for that is continuous collision detection** — the 2026-02-20 sneak peek says so of the TGS engine, so substepping detection is not a departure from him but an implementation of what he calls CCD |
| Angular **momentum** stored, not angular velocity | — | With no torque it is exactly constant, so the conservation gate can demand exact equality rather than a tolerance | **Yes** |
| A bias term on joints | 30 | His `λ = −(J M⁻¹ Jᵀ)⁻¹ (J·V)` shows no drift correction at all | **Yes**, and it is left out of the relax pass, after Box2D v3 |
| Every gate proven by a deliberate break | — | He debugs by disabling axes and printing between stages — effective, and not a standing discipline. BEVOX has found **six blind gates** this way | **Yes**, and it is the practice most responsible for the physics being trustworthy |

# F — Oversight: the actual drift

Seven differences nobody decided on. Five looked worth fixing; **F7 is now
resolved**, leaving four.

## F1. The primary path has no bounding-sphere reject — **bad**

**What differs.** `GpuBody::bound` carries a body's world bounding sphere, and its
own comment says it is *"filled in only for the shadow casters; the marched table
leaves it zero"*.

**Why it drifted.** The reject was built for shadow rays, where it was measured at
**20 ms a frame**, and nobody carried it across. This is not a difference from
Dwyer at all — it is an asymmetry inside BEVOX between its own two ray types.

**Good idea?** No. A primary ray inside a body's screen rectangle but nowhere near
the body still pays a 160-byte read, two `mat4` transforms, a `ray_box` and a
stack frame. A screen rectangle is a 2D footprint: it says the body is *somewhere
along* this ray, not that the ray touches it.

## F2. Bodies are marched in table order, not ray order — **bad**

**What differs.** His devlog 2 replaced a sorted per-object list — which "capped a
ray at four or eight objects because GPU code cannot allocate" — with stepping:
take the shortest step, and if another object offers a shorter one, continue in
that object instead. `compose_bodies` iterates the table as the cull built it.

**Why it drifted.** Composition was written when there was one body, and `limit`
tightening on every hit made it look sufficient. In arbitrary order only about
`ln(N)` of `N` traversals narrow an answer the rest already had.

**Good idea?** No, and it is part of why `MAX_BODIES` is what it is.

> **Corrected 2026-09-29.** This entry first claimed "`1 + N` unaccelerated
> marches from the ray origin", which overstated it and cost a day. Three of the
> four things one would reach for are already built: `traverse_at` seeds its first
> frame at the body's **bounding-box entry**, `limit` tightens on every hit, and
> `BODY_RECT` skips a body for pixels outside its screen rectangle. The claim came
> from a doc comment whose premise was true and whose conclusion was never
> checked — see [an inference is not an observation](../concepts/an-inference-is-not-an-observation.md).

F1 and F2 are both specified in
[body composition in ray order](../superpowers/specs/2026-09-29-bevox-body-composition-design.md).

## F3. Sunlight costs a ray per pixel, not per visible voxel — **bad**

**What differs.** His per-voxel hash map already enumerates the visible voxels, so
he shades direct sunlight **once per voxel**, measured at **1-2 ms on a 1660 Ti
even with path tracing off**.

**Why it drifted.** The per-voxel store was part of his *denoiser* (devlog 19), and
BEVOX has no path tracer to denoise — so the store was never built, and the
optimisation riding on it was never separated from the feature that motivated it.

**Good idea?** No, and it is the sharpest item here: BEVOX deliberately snaps the
shadow ray's origin to the voxel centre so a whole face is lit or shadowed
together. **It already pays for a per-voxel result and collects it per pixel.**

## F4. No stress scene, and no 2D prototype — **the stress scene is done; the 2D repo is not**

**What differs.** He ends devlog 20 with "build the 2D version first — it's
honestly what I should have done the first time", and in devlog 25 he does it:
*Rigid Pixels*, so detectors and solvers can be swapped and compared. His
validation is **the tumbler** — 49 cubes in a hollow rotating cube.

**Why it drifted.** Every gate here was written for a *feature*, so nobody wrote
the one that is not about a feature. A cube lands, three stand, a chain holds, a
lone voxel sleeps — all named scenarios.

**Good idea?** No. On 2026-09-28 a 240:1 mass ratio through a one-voxel plate made
a stack accelerate **upward at 23 voxels a second** by tick 33. Nothing in 117
tests would have caught it, because stability under load is not a scenario.

**Two pieces, and Flori wants both** — my "the 2D half is no longer actionable"
was overruled on 2026-09-30 and the shape settled on 2026-10-01.

- **The stress scene is done**, 2026-10-02:
  `solver::tests::forty_nine_cubes_tumble_without_escaping_diverging_or_sinking`
  — 49 glass 2x2x2 cubes in a closed hollow box with two-voxel walls, gravity
  swept through a full turn in the xy plane over 800 ticks at constant
  magnitude. **Gravity rotates because the drum cannot**: the static world is a
  `Contree` with no kinematic bodies, so the test gates stacking stability
  under a load direction that never stops changing, and **not** his tumbler —
  a real drum also drags its contents tangentially through wall friction, and
  this does not. Three assertions, as specified: nothing escapes (all 49 ids
  survive, every centre inside the cavity), nothing diverges (no speed past
  1.5x the 66.63 v/s a free fall across the cavity diagonal reaches), nothing
  sinks (no overlap past `SLOP + BASE_MARGIN`, sampled every 16 ticks rather
  than at the final frame alone). **It passed on arrival**: worst speed 45.56
  v/s against a bound of 99.94, worst penetration 0.0259 against 0.1200, one
  fracture at 244,950 against an 8-voxel threshold of 233,333, nothing asleep.
  `#[ignore]`d at 99 seconds in a debug build; run it by name.

  **What the scene revealed is how little it loads the solver.** Of four
  deliberate breaks, only `BASE_MARGIN = 0` fails it (penetration 0.0268
  against a bound that falls to 0.0200 with it). `MAX_PUSH` 20 → 2000 changes
  the run **not at all, bit for bit** — the bias here only ever pushes at about
  0.08 v/s, so the clamp is inert in this scene. `RELAXATION_ITERATIONS = 0`
  passes (42.09 v/s, 0.0200). `BIAS = 0` passes at 0.1025, 85% of the way to
  the bound and 4x the baseline. So the speed assertion has a **2.2x margin**
  and is a regression tripwire rather than a sensitive instrument: a uniform
  pile of equal-density cubes loads the solver far less than the 240:1 ratio
  that bit, and a future calibration checked only against this scene would not
  be checked hard. The scene was **not** bent until it was green — it was green
  first, and the three assertions are the ones the ledger specified.
- **The 2D simulation gets its own repository.** It is a *bench*, not a feature —
  somewhere detectors and solvers can be swapped and compared the way
  `rigid_pixels` is, which is the whole reason he built one. Keeping it out of
  this tree means it can carry throwaway solvers without them becoming BEVOX's
  API, and it never pays this repo's gate discipline for code that exists to be
  deleted.

## F7. One velocity pass per substep, where his is a count — **resolved, not fixed**

**What differs.** His solver config carries `velocity_iterations` and
`relaxation_iterations` as *counts*. **BEVOX now carries them too**, as
`bevox_physics::Tuning` through `solver::step_with`, and **both are 1** — so the
structural difference is gone and the effective behaviour is unchanged.

**Why it drifted.** BEVOX's solver was ported from what devlog 26 describes, and
he describes the *structure* — detect, substep, integrate last — not the iteration
counts inside it. They were only visible once `rigid_pixels` could be read.

**Good idea?** The difference was worth closing; **the reason given for closing
it was wrong.** This entry said more velocity iterations per substep is "the
standard lever for stacking stability" and that BEVOX "has been running the
solver at its least stable setting". Measured by
`solver::tests::what_the_iteration_counts_cost` (2026-10-01, re-run 2026-10-02):
**raising the velocity count while holding the relax count at 1 bought no
stability on a 240:1 load** — worst upward velocity 12.83 v/s at `(1,1)` against
18.02 at `(8,1)`, and `(8,1)` breached the floor on the same tick 55 that
`(1,1)` did, for 2.6x the tick cost. The one gain on that axis was shallower
resting penetration, 0.0017 → 0.0003.

**What the measurement cannot say.** Those four variants hold
`relaxation_iterations` at 1, so they vary the **bias:relax ratio** and not the
biased count alone — the relax pass exists to remove the velocity the bias
added, and the ratio is an unexcluded explanation of every number in them. The
two **balanced** variants point the other way: `(2,2)` and `(4,4)` keep the
fixture standing to tick **105** and **123** against `(1,1)`'s 55, for 1.9x and
3.0x the tick. So the balanced axis is **not** closed, and the earlier claims
that this axis offered nothing and that the question was closed are withdrawn.

So **the counts stay at 1** — a cost decision on a fixture that collapses at
every setting measured, plus the fact that the counts rescale `peak` and
therefore Part 2's `strength` calibration, pinned by
`solver::tests::the_solver_iteration_counts_are_one`. Full write-up:
[solver convergence is a setting, and the spec reached for the wrong one](../concepts/solver-convergence-is-a-setting.md).
Two things it left behind: the 2026-09-28 divergence **did not reproduce** as
recorded, and the rebuilt fixture instead collapses through the floor at tick
55 — an open defect, now characterised by the `#[ignore]`d
`solver::tests::the_240_to_1_load_collapses_through_the_floor`, which passes
today and is written to be inverted when the collapse is fixed. F4's stress
scene is now built and **does not settle this**: a uniform 49-cube pile passes
every assertion with a 2.2x speed margin and survives `RELAXATION_ITERATIONS
= 0` and `BIAS = 0`, so what makes an extreme mass ratio collapse is still
unmeasured, and the tumbler is not the fixture that will measure it.

## F5. Shadows are recomputed every frame — **too early to say**

**What differs.** He recomputes shadows only when the scene changes, and splits
static from dynamic geometry so the optimisation survives moving casters.

**Why it drifted.** His scheme is a rasteriser's — a shadow pass whose output is
reusable. A marcher has no pass to skip.

**Good idea?** Unclear, and probably the wrong question. For a marcher, sun
visibility is a property of a **voxel**, so the right form of his idea is F3's
per-voxel cache rather than his per-frame one. F3 subsumes this.

## F6. Editing is a sphere brush, not a two-tree paste — **neutral**

**What differs.** His edit is divide-and-conquer over two trees: paste a source
onto a target, keep the target where the source is empty, overwrite whole where
the target lies inside a homogeneous source.

**Why it drifted.** BEVOX only ever needed spheres.

**Good idea?** Neutral, and it collapses into D's copy-region item: both want the
same missing node-aware primitive, his paste being that with a mask. Build it for
detachment's speed; leave the brush alone until something needs the mask.

# Where the measurement points

| | GPU ms |
|---|---|
| static world, `DEFAULT` | 13.23 |
| sixteen bodies | 23.30 |
| sixteen bodies, no body shadows | 17.83 |

A body costs **0.629 ms**, split 0.316 primary against 0.314 shadow. So F1+F2 and
F3 are worth **the same amount**, and neither alone gets sixteen bodies inside a
16.7 ms frame — 18.25 and 18.28 respectively.

**And the static march is 13.23 ms on its own**, 79% of the frame before a body
exists. That is the real ceiling, it appears nowhere on this page, and it is where
his most striking number sits: **7 ms for a Teardown castle on a 1660 Ti**, primary
and shadow ray, against 13.23 ms here on a 1650 for the bench scene. Whether that
gap is the hardware, the scene or the traversal is unmeasured, and it is the most
interesting open question in the engine.

# Order of work

1. **F1**, which mirrors code that exists and is already measured on the other ray
   type.
2. **F2**, his devlog 2, if F1 leaves the primary half worth attacking.
3. **F4's stress scene — done** 2026-10-02, a fixture and no engine code, as
   predicted. Its 2D-repository half is not.
4. **F3**, which needs a per-voxel store BEVOX does not have.
5. **D's copy-region**, justified by detachment's speed rather than API
   completeness.
6. F5 and F6 need a sentence each, not a change.
