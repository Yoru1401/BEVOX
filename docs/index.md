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

# Concepts

Start here. Each one is a rule, a measured number, a decision or a post-mortem,
lifted out of the plan that produced it.

* [Concepts](concepts/) - what was learned building this, out of the plans that learned it.
* [Every gate is proven by a deliberate break](concepts/deliberate-breaks.md) - a test that has never failed is not known to test anything.
* [No safe stale direction](concepts/stale-direction.md) - the distance field may lag, fullness may not, so every editing path recounts it.
* [The GPU codegen cliff](concepts/gpu-codegen-cliff.md) - code the shader never runs can still cost tens of percent; re-bench every march.wgsl edit.
* [The body cap, and what a body costs](concepts/body-cap.md) - 0.265 ms per visible body, and why MAX_BODIES is 16.
* [The frame budget, and where it goes](concepts/frame-budget.md) - sixteen bodies with shadows take 22.6 ms against 16.7.
* [Both coarse grids ride in one storage buffer](concepts/coarse-grids-share-one-buffer.md) - wgpu's eight-storage-buffer limit, and what it forces.
* [Only terrain debris merges back](concepts/terrain-only-merging.md) - what may return to the world, and why it must be out of view.
* [Padding a joint block at its own scale](concepts/joint-block-conditioning.md) - identity padding against a 1e-5 block destroys the inverse in f32.

# Reference

* [Douglas Dwyer's voxel engine, devlog by devlog](reference/dwyer-devlogs.md) - what he built in each of his thirty Voxel Devlogs, dated, and what each changed from the ones before.

# Design specs

* [BEVOX ray-marcher core](superpowers/specs/2026-09-13-bevox-raymarcher-core-design.md) - A sparse voxel world ray marched on the GPU, with a CPU reference the shader is held to pixel for pixel.
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
* [Mouse Grab](superpowers/plans/2026-09-18-bevox-physics-mouse-grab.md) - Pick up a body with the mouse.
* [Sleeping and Merging Debris](superpowers/plans/2026-09-19-bevox-sleep-and-merge.md) - Bodies at rest stop costing physics time, and a body that came out of the terrain and settled out of view goes back into it and frees its slot, as in Dwyer's devlog #13.
* [Fracture](superpowers/plans/2026-09-24-bevox-fracture.md) - Bodies and terrain crack where a collision is too hard for the material; cracks are drawn as empty voxels and detachment produces the pieces.
* [Debug Views](superpowers/plans/2026-09-24-bevox-debug-views.md) - Nine debug views selectable in the app from the function keys, eight of them free and one, the ray-step heatmap, measured before it is kept.
* [Ambient Occlusion](superpowers/plans/2026-09-24-bevox-ambient-occlusion.md) - Creases and corners darken, so a body sitting on the floor reads as sitting on it, following Dwyer's devlog #15.
* [Detachment Over Tree Nodes](superpowers/plans/2026-09-24-bevox-detachment-over-nodes.md) - The detachment search walks the tree's uniform nodes instead of single voxels, as Dwyer's devlog #12 does, so a cut into a large volume costs a few steps rather than thousands.
