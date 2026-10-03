---
type: Reference
title: 'The contree, and where the name comes from'
description: "A contree is an octree with two layers squashed into one: branching 8 to 8 squared, 4x4x4 per node, from tetrahexaconta plus tree. With the one outside measurement of what squashing costs in space and saves in encode time."
tags: [core, contree, data-model, reference]
generated: { by: claude-opus-5/claude-code, at: 2026-10-03T00:00:00Z }
sources:
  - id: eisenwave
    resource: https://eisenwave.github.io/voxel-compression-docs/svo/svo.html
    title: 'Sparse Voxel Octrees — voxel-compression-docs'
  - id: core
    resource: /map/core.md
    title: bevox_core on one page
---

# What the name means

**`tetrahexaconta`** — Greek for sixty-four — **plus `tree`.** Contracted, a
tetrahexacontree is a contree.

Found by Flori on 2026-10-03: the term appears on screen in Dwyer's devlog 22,
where he says only that it is "commonly" what the structure is now called, and
the definition is in
[eisenwave's voxel-compression documentation](https://eisenwave.github.io/voxel-compression-docs/svo/svo.html),
which gives the etymology and the derivation together.

# The derivation, which is more useful than the name

Their framing is **the squashed octree**: take an octree and merge two layers
into one. Branching goes from 8 to **8² = 64**, and each node divides space into
a **4x4x4** cube instead of 2x2x2.

So four-per-axis is not a design parameter anyone chose. It falls out:

| | |
|---|---|
| four per axis | 2², the two merged levels |
| a 64-bit occupancy mask | 8², their combined occupancy |
| half the depth | six levels to 4096 rather than twelve |

**And it is why the mask fits a register.** One `u64` covers a whole node, so a
ray reads it once and then steps from child to child on bits rather than memory.
That is usually given as the reason for choosing 64; it is better read as the
reason the squash *stops* at two layers.

**Three layers would not work.** 8³ = 512 children and a 512-bit mask, which no
machine word holds, and the structure loses the only trick it is built on. Two is
the largest squash that keeps it.

# What squashing costs, measured by someone else

The one outside number available on this structure, from the same source, on
their Ragged Cluster model:

| | |
|---|---|
| Regular SVO, encode | 1700–1800 ms |
| **Squashed SVO, encode** | **1048–1288 ms** |
| Space cost of squashing | **+0.196% in bits** |

Roughly **1.4x faster to build, for two tenths of a percent of space.**

That number is worth keeping because it contradicts the obvious intuition. A node
storing occupancy for 64 cells rather than 8 looks like it should waste bits on
absent children in sparse regions — and at realistic sparsity it does not,
measurably. Nothing in this repository measures that, and nothing here is likely
to; it is the kind of claim that wants a corpus of models rather than one scene.

# Vocabulary for traversal

The same source names three strategies by what they must keep:

| | Needs |
|---|---|
| Depth-first | a stack of nodes |
| Breadth-first | a queue |
| Their accelerated depth-first | a stack of node *lists* |

**BEVOX's marcher is none of these exactly.** It carries a stack of descent
*frames* — a node, the ray's position within it, and the DDA state — because it
is walking along a ray rather than enumerating a volume. The vocabulary is still
the right one for describing it, and "stack of node lists" is the nearest
neighbour.

# What this does not settle

The etymology and the derivation are settled. **The byte layout is not.** Dwyer
has never published one — not his node size, not his leaf representation, not how
he encodes a voxel's colour — and this page does not guess. What is recorded
elsewhere is the one layout lesson he did publish, from his octree days: aligning
a node's fields rather than packing them removed both arithmetic and buffer
reads, taking a frame from 7–10 ms to 3.5–4, for 30–50% more memory.
