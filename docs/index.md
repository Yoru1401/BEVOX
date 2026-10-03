---
okf_version: "0.2"
---

# BEVOX knowledge bundle

A pure-Rust sparse-voxel ray marcher, with a rigid-body physics subsystem
modelled on Douglas Dwyer's voxel engine. Everything here is a design spec or
the plan that carried one out; a plan's Measurements section holds the numbers
the decision was made on, and its Breaks section the deliberate breaks each
gate was proven with.

Read a spec for what the engine is meant to be, a concept for a fact worth
carrying into the next piece of work, and a plan for why one piece of it is the
way it is and what it cost.

# Start here

* [The engine on one page](map/) - the four crates, the two loops, the numbers
  everything sits inside, and where to go to change a given thing.
* [bevox_core](map/core.md) - the voxel data structure and the algorithms over it.
* [bevox_physics](map/physics.md) - bodies, contacts, joints, fracture, sleeping.
  Its tunables are on [their own page](map/physics-constants.md).
* [bevox_render](map/render.md) - the Bevy plugin, the shader, and one frame.

A map page says what is true now, and its factual half is checked against the
source by `tests/map.rs` in each crate. Everything below says how it came to be
true, and is never rewritten to match.

# Concepts

Start here. Each one is a rule, a measured number, a decision or a post-mortem,
lifted out of the plan that produced it.

* [Concepts](concepts/) - what was learned building this, out of the plans that learned it.
* [Every gate is proven by a deliberate break](concepts/deliberate-breaks.md) - a test that has never failed is not known to test anything.
* [No safe stale direction](concepts/stale-direction.md) - the distance field may lag, fullness may not, so every editing path recounts it.
* [The GPU codegen cliff](concepts/gpu-codegen-cliff.md) - code the shader never runs can still cost tens of percent; re-bench every march.wgsl edit.
* [One canonical tree, and a derived view for each job](concepts/derived-view-per-job.md) - six representations are derived from the Contree, each shaped for one job and each declaring where it may be stale.
* [An inference is not an observation](concepts/an-inference-is-not-an-observation.md) - a claim about behaviour names the code that makes it true, because no gate can read prose.
* [The body cap, and what a body costs](concepts/body-cap.md) - 0.629 ms per visible body, and why MAX_BODIES is 16.
* [The frame budget, and where it goes](concepts/frame-budget.md) - sixteen bodies with shadows take 22.6 ms against 16.7.
* [Nine steps a ray, so the walk is not the cost](concepts/nine-steps-a-ray.md) - a primary ray takes nine steps, so the static march is bound by what a step costs, not how many.
* [Solver convergence is a setting, and the spec reached for the wrong one](concepts/solver-convergence-is-a-setting.md) - raising the biased count alone bought no stability on a 240:1 load, and the design cannot separate convergence from the bias:relax ratio; both counts stay at 1.
* [Both coarse grids ride in one storage buffer](concepts/coarse-grids-share-one-buffer.md) *(premise corrected)* - eight was wgpu's browser baseline, not the hardware's limit; the adapter allows 524,288.
* [Only terrain debris merges back](concepts/terrain-only-merging.md) - what may return to the world, and why it must be out of view.
* [Sliding friction cannot stop a roll](concepts/rolling-needs-its-own-resistance.md) - a lone voxel is a sphere, and nothing was slowing it down.
* [Padding a joint block at its own scale](concepts/joint-block-conditioning.md) - identity padding against a 1e-5 block destroys the inverse in f32.

# Reference

* [The contree, and where the name comes from](reference/contree.md) - tetrahexaconta plus tree: a contree is an octree with two layers squashed into one, which is why four per axis and why a 64-bit mask. With the one outside measurement of what squashing costs.
* [BEVOX as a design](reference/bevox-engine.md) - what this engine is and how its parts work, by system: one sparse tree marched on the GPU, six views derived from it, a rigid-body tick at a fixed rate, the disciplines that keep its numbers honest, and what is measured and still open.
* [Douglas Dwyer's voxel engine, as a design](reference/dwyer-engine.md) - what his engine is and how its parts work, by system rather than by date: the contree, two renderers, four lighting systems, three physics engines, the architecture, what he rejected and why, and how he works. No comparison with anything here.
* [Douglas Dwyer's voxel engine, devlog by devlog](reference/dwyer-devlogs.md) - what he built in each of his thirty Voxel Devlogs, dated, and what each changed from the ones before, plus the four videos that are not devlogs.
* [Dwyer's rigid_pixels, read from source](reference/rigid-pixels.md) - the 2D prototype behind devlog 26, which settles what the devlogs leave open.
* [John Lin's voxel engine, read from his blog and videos](reference/john-lin.md) - a second developer, unrelated to Dwyer. Three engines in fifteen months, including dropping hardware ray tracing because a BLAS is too heavy for a dynamic scene, and a reversal on unified voxel formats that makes his objection a post-mortem rather than an opinion.
* [How far BEVOX has drifted from Dwyer](reference/dwyer-drift.md) - thirty-one differences sorted by why each happened and whether it was a good idea. Six causes; only one is drift.

# Design specs

* [BEVOX ray-marcher core](superpowers/specs/2026-09-13-bevox-raymarcher-core-design.md) - A sparse voxel world ray marched on the GPU, with a CPU reference the shader is held to pixel for pixel.
* [Body composition in ray order](superpowers/specs/2026-09-29-bevox-body-composition-design.md) - cut the primary half of a body's cost by rejecting with the bounding sphere the shadow path already has, then visiting bodies in ray order.
* [Sun visibility per voxel, not per pixel](superpowers/specs/2026-10-03-bevox-per-voxel-sun-design.md) - the shadow ray is a pure function of (voxel, face) and costs 8 ms of a 26 ms frame at 1080p. Compute it once per voxel face, bit-identically.
* [Fracture in two regimes](superpowers/specs/2026-10-02-bevox-fracture-regimes-design.md) - a landing and a crush are 1.06x apart in impulse and 18,000x apart in closing speed, so switch on the speed and threshold a force for a held contact.
* [Fracture and detection, as he built them](superpowers/specs/2026-09-30-bevox-fracture-and-detection-design.md) - expose the solver's iteration counts, revert the closing-speed threshold to a size-scaled impulse, and replace the speed cap with substepped detection. In that order, because the iteration count sets the impulse the threshold reads.
* [BEVOX Rigid Bodies](superpowers/specs/2026-09-15-bevox-rigid-bodies-design.md) - Voxel chunks that detach from the static world, fall, tumble and come to rest, rendered by the same ray marcher that draws everything else.

# Implementation plans

* [BEVOX core and reference marcher](superpowers/plans/2026-09-13-bevox-core-and-reference-marcher.md) - Build `bevox_core` — the contree voxel data structure with editing — and a CPU reference ray marcher that renders a scene to a PNG, with no GPU and no Bevy involved.
* [BEVOX GPU traversal](superpowers/plans/2026-09-13-bevox-gpu-traversal.md) - Put the voxel scene on screen, ray marched by our own WGSL compute shader, and prove that shader agrees with the CPU reference marcher pixel for pixel.
* [BEVOX MagicaVoxel scene composition](superpowers/plans/2026-09-13-bevox-scene-composition.md) - Load a whole MagicaVoxel scene — every model, placed and rotated by its scene graph — instead of one model out of dozens.
* [BEVOX shading and model import](superpowers/plans/2026-09-13-bevox-shading-and-vox.md) - Light the voxel scene with implicitly generated per-voxel normals and a sun shadow ray, then load real MagicaVoxel models into it.
* [BEVOX traversal optimisation](superpowers/plans/2026-09-13-bevox-traversal-optimisation.md) - Make the ray marcher fast enough to fly through a composed scene, without changing a single pixel it produces.
* [Empty-Space Distance Field](superpowers/plans/2026-09-14-bevox-empty-space-distance-field.md) - Skip empty space by advancing a ray's start through a coarse distance field before it enters the tree, and keep that field correct under the sphere brush.
* [Sphere Brush and Dirty-Range Upload](superpowers/plans/2026-09-14-bevox-sphere-brush-and-dirty-upload.md) - Edit the voxel scene at runtime with a sphere brush, uploading only the arena ranges the edit touched rather than the whole volume.
* [Rigid Body Rendering](superpowers/plans/2026-09-15-bevox-body-rendering.md) - Render a voxel volume placed by a rigid transform, composed with the static world by nearest hit, so a body can be seen rotating before any physics exists.
* [Body Culling](superpowers/plans/2026-09-17-bevox-body-culling.md) - Stop paying for rigid bodies a pixel cannot see, so the body cap can rise above 1 before physics needs several bodies.
* [Body Shadows](superpowers/plans/2026-09-18-bevox-body-shadows.md) - Rigid bodies cast shadows: on the static world, on each other, and on themselves.
* [Body Editing](superpowers/plans/2026-09-18-bevox-physics-body-editing.md) - The brush edits bodies as it edits the world.
* [Body Against Body](superpowers/plans/2026-09-18-bevox-physics-body-vs-body.md) - Bodies collide with each other as they do with the world: a thrown cube knocks another along, and a stack of cubes stands still instead of sinking into itself.
* [Terrain Detachment](superpowers/plans/2026-09-18-bevox-physics-detachment.md) - Carve the support out from under a wall and the wall falls.
* [Dwyer's Joints](superpowers/plans/2026-09-18-bevox-physics-dwyer-joints.md) - Rebuild joints after Dwyer's devlog #30: sixteen types, friction, motors, the mouse grab as a joint, and a key-swapped scene to try them all, with the J tool removed.
* [Friction and Restitution](superpowers/plans/2026-09-18-bevox-physics-friction-restitution.md) - A body slides to a stop on stone, keeps sliding on ice, and bounces on rubber, with friction and restitution taken per voxel from the materials that touch.
* [Joints](superpowers/plans/2026-09-18-bevox-physics-joints.md) *(deprecated)* - Join bodies to each other or to the world with ball and hinge joints, made by clicking in the app: a door hinged to a wall, a chain hanging from a beam.
* [Rigid-Body Physics Milestone 2](superpowers/plans/2026-09-18-bevox-physics-milestone-2.md) - A voxel body dropped into the scene lands on the static voxel world, tumbles if it lands off balance, and comes to rest without jitter.
* [Mouse Grab](superpowers/plans/2026-09-18-bevox-physics-mouse-grab.md) *(superseded)* - Pick up a body with the mouse, as a damped spring. Milestone 7 rebuilt the grab as a joint; its constants and gates are gone.
* [Sleeping and Merging Debris](superpowers/plans/2026-09-19-bevox-sleep-and-merge.md) - Bodies at rest stop costing physics time, and a body that came out of the terrain and settled out of view goes back into it and frees its slot, as in Dwyer's devlog #13.
* [A Physics Crate, Forces, and a Tick-Free Ceiling](superpowers/plans/2026-09-24-bevox-physics-crate.md) - physics moves to its own crate, bodies take forces and impulses from outside, and the speed ceiling stops depending on the tick rate.
* [Sun visibility per voxel](superpowers/plans/2026-10-03-bevox-per-voxel-sun.md) - build the store and measure its own cost before splitting the march: insert first and read nothing, then add the sun pass and collect the win.
* [Fracture in two regimes](superpowers/plans/2026-10-02-bevox-fracture-regimes.md) - give `Material` a crush force, branch on the closing speed, then raise the impact strengths the crush ceiling was holding down.
* [Fracture on a size-scaled impulse](superpowers/plans/2026-10-02-bevox-fracture-on-impulse.md) - widen `Material::strength` to `f32`, threshold the accumulated normal impulse so a slow crush breaks what it presses, scale each side by its own size, and hand back exactly the excess.
* [Fracture](superpowers/plans/2026-09-24-bevox-fracture.md) - Bodies and terrain crack where a collision is too hard for the material; cracks are drawn as empty voxels and detachment produces the pieces.
* [Debug Views](superpowers/plans/2026-09-24-bevox-debug-views.md) - Nine debug views selectable in the app from the function keys, eight of them free and one, the ray-step heatmap, measured before it is kept.
* [Ambient Occlusion](superpowers/plans/2026-09-24-bevox-ambient-occlusion.md) - Creases and corners darken, so a body sitting on the floor reads as sitting on it, following Dwyer's devlog #15.
* [Detachment Over Tree Nodes](superpowers/plans/2026-09-24-bevox-detachment-over-nodes.md) - The detachment search walks the tree's uniform nodes instead of single voxels, as Dwyer's devlog #12 does, so a cut into a large volume costs a few steps rather than thousands.
