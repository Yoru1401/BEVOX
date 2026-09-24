# Concepts

What was learned building this, taken out of the plans that learned it. A plan
tells the story of one piece of work; these are the facts worth carrying into
the next one.

# Rules

* [Every gate is proven by a deliberate break](deliberate-breaks.md) - a test that has never failed is not known to test anything.
* [No safe stale direction](stale-direction.md) - the distance field may lag, fullness may not, so every editing path recounts it.
* [The GPU codegen cliff](gpu-codegen-cliff.md) - code the shader never runs can still cost tens of percent; re-bench every march.wgsl edit.

# Measurements

* [The body cap, and what a body costs](body-cap.md) - 0.265 ms per visible body, and why MAX_BODIES is 16.
* [The frame budget, and where it goes](frame-budget.md) - sixteen bodies with shadows take 22.6 ms against 16.7.

# Decisions

* [Both coarse grids ride in one storage buffer](coarse-grids-share-one-buffer.md) - wgpu's eight-storage-buffer limit, and what it forces.
* [Only terrain debris merges back](terrain-only-merging.md) - what may return to the world, and why it must be out of view.

# Post-mortems

* [Sliding friction cannot stop a roll](rolling-needs-its-own-resistance.md) - a lone voxel is a sphere, and nothing was slowing it down.

* [Padding a joint block at its own scale](joint-block-conditioning.md) - identity padding against a 1e-5 block destroys the inverse in f32.
