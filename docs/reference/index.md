# Reference

Material about things outside this repository, kept because the engine is
built from it.

* [The contree, and where the name comes from](contree.md) - tetrahexaconta plus tree: a contree is an octree with two layers squashed into one, which is why four per axis and why a 64-bit mask. With the one outside measurement of what squashing costs.
* [BEVOX as a design](bevox-engine.md) - what this engine is and how its parts work, by system: one sparse tree marched on the GPU, six views derived from it, a rigid-body tick at a fixed rate, the disciplines that keep its numbers honest, and what is measured and still open.
* [Douglas Dwyer's voxel engine, as a design](dwyer-engine.md) - what his engine is and how its parts work, by system rather than by date: the contree, two renderers, four lighting systems, three physics engines, the architecture, what he rejected and why, and how he works. No comparison with anything here.
* [Douglas Dwyer's voxel engine, devlog by devlog](dwyer-devlogs.md) - what he built in each of his thirty Voxel Devlogs, dated, and what each changed from the ones before, plus the four videos that are not devlogs. Dates verified mechanically and descriptions swept on 2026-10-01: nothing contradicted.
* [Dwyer's rigid_pixels, read from source](rigid-pixels.md) - his open-source 2D prototype: detection substepped instead of capped, fracture on a size-scaled impulse, and the solver knobs BEVOX collapsed. No licence: read for design, never copy.
* [John Lin's voxel engine, read from his blog and videos](john-lin.md) - a second developer, unrelated to Dwyer. Three renderers in fifteen months, one of them abandoning hardware ray tracing for a lightweight structure a dynamic scene can afford; the numbers he published; and his reversal on unified voxel formats. No source, and silent since 2021.
* [How far BEVOX has drifted from Dwyer](dwyer-drift.md) - thirty-one differences, each with why it happened and whether it was a good idea. Six causes: different product, different architecture, different constraints, not reached yet, improvements, and oversight.
