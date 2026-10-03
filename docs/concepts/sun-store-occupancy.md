---
type: Measurement
title: 'Seven pixels a voxel face, and three slots a face'
description: 'The sun shadow ray is recomputed 7.5 times a voxel face at 1080p, so the redundancy is there to collect; but a lockstep insert claims 3.2 slots for each face, which is what would be dispatched over and spends most of the win before it is taken.'
tags: [performance, gpu, lighting, atomics]
generated: { by: claude-opus-5/claude-code, at: 2026-10-03T00:00:00Z }
sources:
  - id: design
    resource: /superpowers/specs/2026-10-03-bevox-per-voxel-sun-design.md
    title: 'Sun visibility per voxel, not per pixel'
  - id: cliff
    resource: /concepts/gpu-codegen-cliff.md
    title: The GPU codegen cliff
---

# The numbers

`the_sun_store_occupancy_is_reported` and `what_the_sun_store_costs` in
`tests/gpu_bench.rs`, GTX 1650, bench scene and camera, `DEFAULT`, no bodies,
2^21 slots. Two invocations, the spread between them in the last column.

| | 1280x720 | 1920x1080 | between runs |
|---|---|---|---|
| pixels that hit the static world and inserted | 417 497 | 886 596 | <1% |
| distinct `(voxel, face)` keys | 94 084 | 118 215 | <0.01% |
| **pixels a distinct face — the redundancy** | **4.44** | **7.50** | <1% |
| slots occupied | 251 912 | 377 697 | <1% |
| **slots a face — the duplication** | **2.68** | **3.20** | <2% |
| pixels an occupied slot | 1.66 | 2.33 | |

**The redundancy is real and it grows with resolution**, which is the whole
premise: 2.25x the pixels found only 1.26x the faces, because a voxel covers
more pixels the closer the camera's rays are packed. Nothing here is near 1.

# The duplication, which is the finding

**Each face holds three slots, not one.** Correctness is untouched — every
duplicate carries the same key and the same answer, and a reader verifying the
key finds one of them — but a sun pass dispatched over *occupied slots* would
march 377 697 shadow rays where 118 215 distinct answers exist. That takes the
collectable factor from **7.50 down to 2.33**, and with it about 2.4 ms of the
8.36 ms ceiling, before the pass is written.

The cause is the gap the design names and does not close. A slot is claimed with
a compare-exchange on a 32-bit frame stamp and the 39-bit key is written after,
because core WGSL has no 64-bit atomic. The key write is not ordered against
another invocation's read of the stamp. Sixty-four lanes of a workgroup cover
about seven faces, so for each face several lanes reach the same slot in
lockstep: one wins the exchange, and the losers re-read the stamp and the key at
the same instruction step the winner is writing it. They see this frame's stamp
beside an older key, conclude the slot is someone else's, and probe on to claim
another for the same face.

**So a dedup *before* the insert is worth more than any tuning of the insert.** A
quad or subgroup vote, or one insert per workgroup per distinct face, attacks the
lockstep directly; a retry inside the insert is a race against the same
instruction step.

# What the store costs when nothing reads it

Five invocations, each A/B/A interleaved, insert on against off, GPU clock:

| | readings | taken |
|---|---|---|
| 1280x720 | +0.24, +0.42, +0.54, +0.54, +0.58 | **about +0.5 ms** |
| 1920x1080 | +0.20, +0.69, +0.95, +1.10, +2.68 | **about +0.9 ms** |

The 1080p readings scatter: two of the five came with a baseline drift larger
than the difference (2.32 and 1.18 ms) and are not results by this project's own
rule. The three that stand give +0.69, +0.95 and +1.10, and the wall clock,
tighter, gives +0.48 to +1.17. **Call it 8 to 12% of the 8.36 ms ceiling**, which
is affordable. Collisions, at a 4.5% load factor in faces, cost almost nothing:
the key guard rescues about 2 100 faces of 94 000 at 720p, a birthday count.

# The collision that the stored key guards

`forcing_every_key_to_match_shares_slots_between_voxels` reports every key as
equal, and the store then records 92 005 faces against 94 084 — **2 077 voxel
faces took a slot another voxel owns**, against the 2 110 a birthday count
predicts. Few, because the table is twenty times oversized in faces, and at
1080p four times as many. The guard is load-bearing, not defensive: without it
those faces read another voxel's answer.
