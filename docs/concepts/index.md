# Concepts

What was learned building this, taken out of the plans that learned it. A plan
tells the story of one piece of work; these are the facts worth carrying into
the next one.

# Rules

* [Every gate is proven by a deliberate break](deliberate-breaks.md) - a test that has never failed is not known to test anything.
* [No safe stale direction](stale-direction.md) - the distance field may lag, fullness may not, so every editing path recounts it.
* [The GPU codegen cliff](gpu-codegen-cliff.md) - code the shader never runs can still cost tens of percent; re-bench every march.wgsl edit.
* [One canonical tree, and a derived view for each job](derived-view-per-job.md) - six views are derived from the Contree and each declares its own staleness; the next job gets a view, not a wider node.
* [An inference is not an observation](an-inference-is-not-an-observation.md) - five of six doc errors in one audit were true premises with unchecked conclusions; the gates cannot read prose.

# Measurements

* [The body cap, and what a body costs](body-cap.md) - 0.629 ms per visible body, and why MAX_BODIES is 16.
* [The frame budget, and where it goes](frame-budget.md) - sixteen bodies with shadows take 22.6 ms against 16.7.
* [Nine steps a ray, so the walk is not the cost](nine-steps-a-ray.md) - the traversal has nothing left to give; workgroup size is not the lever either.
* [The impulse threshold separates a crush from a lean by ten per cent](fracture-load-window.md) - fracture reads the accumulated impulse so it can see a crush; resting load 331,306 against a crush load that saturates at 365,906, and past four cubes the ordering inverts.
* [Solver convergence is a setting, and the spec reached for the wrong one](solver-convergence-is-a-setting.md) - raising the biased count alone bought no stability on a 240:1 load, the balanced axis is open, and that fixture's collapse is a characterised open defect.

# Decisions

* [Both coarse grids ride in one storage buffer](coarse-grids-share-one-buffer.md) *(premise corrected)* - eight was wgpu's browser baseline, not the hardware's limit.
* [Only terrain debris merges back](terrain-only-merging.md) - what may return to the world, and why it must be out of view.

# Post-mortems

* [Sliding friction cannot stop a roll](rolling-needs-its-own-resistance.md) - a lone voxel is a sphere, and nothing was slowing it down.

* [Padding a joint block at its own scale](joint-block-conditioning.md) - identity padding against a 1e-5 block destroys the inverse in f32.
