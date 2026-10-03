---
type: Measurement
title: 'Nine pixels a voxel face, and one slot each'
description: 'The sun shadow ray is recomputed 9.22 times a voxel face at 1080p, so the redundancy is there to collect; the insert claims exactly one slot per face, because the frame stamp and the key tag share one word and so one compare-exchange settles both.'
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
2^21 slots.

| | 1280x720 | 1920x1080 |
|---|---|---|
| pixels that hit the static world and inserted | 485 361 | 1 092 019 |
| distinct `(voxel, face)` keys | 94 124 | 118 423 |
| **pixels a distinct face — the redundancy** | **5.16** | **9.22** |
| pixels a face counting sky pixels too | 9.79 | 17.51 |
| slots occupied | 94 124 | 118 423 |
| **slots a face** | **1.000** | **1.000** |
| load factor | 0.045 | 0.057 |

**The redundancy is real and it grows with resolution**, which is the whole
premise: 2.25x the pixels found only 1.26x the faces, because a voxel covers
more pixels the closer the camera's rays are packed. Nothing here is near 1, and
one slot a face means a pass over the occupied slots collects all of it.

The factor over *inserting* pixels is the honest one. The factor over every
pixel, 17.51 at 1080p, counts sky pixels that never marched a shadow ray.

# One slot a face, and the race that cost three

The first form of the insert claimed a slot with a compare-exchange on a 32-bit
frame stamp and wrote the 39-bit key after, because core WGSL has
`atomic<u32>` and `atomic<i32>` and nothing wider. **That key write is not
ordered against another invocation's read of the stamp.** Sixty-four lanes of a
workgroup cover about seven faces, so several lanes reach one slot in lockstep:
one wins the exchange, and the losers read the key at the instruction step the
winner writes it, see this frame's stamp beside a stale key, call the slot
someone else's, and probe on to claim another for the same face.

Measured, that form took **2.68 slots a face at 720p and 3.20 at 1080p**, and
19% of hit pixels came away with no slot at all, having spent all eight probes
thrashing. Every duplicate was correct and all but one was waste: a pass over
occupied slots would have marched each of them, collecting 2.33 pixels an answer
instead of 9.22 — about 2.4 ms of the 8.36 ms the shadow march is worth.

**The fix is to put the key's tag in the claim word.** 8 bits of frame stamp
over 24 bits of a hash of the key, so one compare-exchange is atomic over both
"is this slot free" and "is it mine", and **nothing is written after the claim
for a losing lane to half-read**. A loser re-reads the word and sees the winner's
stamp and tag together. One slot a face, exactly, at both resolutions.

Two things that fall out of it:

- **A matching claim word returns the slot without reading the key**, because
  reading the key is what the race was. So two faces whose 24-bit tags collide
  *and* whose slots collide share one slot, holding whichever key its claimer
  wrote — about one pair in sixteen million. **The full key in `sun_table` is
  what tells them apart, and it is checked by whoever reads the store**, who
  marches its own ray on a mismatch. A wrong answer is not reachable through it.
- **The tag must be a second, independent mix.** A tag cut from the same hash as
  the slot index shares 21 of its 24 bits with it, so two faces in one slot would
  already agree on all but three and collide one time in eight.

# The stamp is 8 bits, and that is a wrap to think about

`pipeline::next_sun_stamp` cycles 1 to 255 and never returns 0, because a
zero-initialised claim word is a slot that was never used and a stamp of 0 would
read as free and as this frame's at once. Narrowing the stamp to 8 bits moves
that wrap from 2.3 years to 4.25 seconds, so it is now a case that happens rather
than one that does not.

A slot still carrying a stamp from exactly 255 frames ago reads as this frame's.
With a different tag it is skipped, costing one slot of capacity out of two
million. With the same tag — one in sixteen million again — it is taken without
its key being rewritten, and the key verification on read is what makes that
safe. `the_stamp_cycles_without_ever_being_zero` and
`the_shader_and_the_stamp_agree_on_the_split` are the gates; the second exists
because the width is spelled as a shift in `march.wgsl` and a bit count in
`pipeline.rs`, and a stamp wider than the shader's field would overwrite the tag
with no picture to show it.

**Both of those are now exercised.** The read side is `march_composite` and its
key check is an image break — see [the sun pass was
lane-bound](sun-pass-is-lane-bound.md), which also carries what the store
collected once something read it.

# What the store costs when nothing reads it

Five invocations, each A/B/A interleaved, insert on against off, GPU clock,
after the fix:

| | readings | taken |
|---|---|---|
| 1280x720 | +0.22, +0.26, +0.28, +0.31, +0.38 | **about +0.28 ms** |
| 1920x1080 | +0.30, +0.34, +0.43, +0.60 | **about +0.4 ms** |

One 1080p reading, +0.27, came with a baseline drift of 0.81 ms and is not a
result. Drift was 0.00-0.29 ms on all the rest, against 0.42-2.32 for the raced
form, which cost **+0.5 ms at 720p and about +0.9 at 1080p** — so the fix made
the store cheaper as well as tighter, because it does a third of the claims and
no key reads. **About 5% of the ceiling at 1080p.**

# The break, and what the cliff did to it

`reintroducing_the_insert_race_costs_slots_per_face` puts the old form back by
text and reads the store twice in one invocation: 1.000 slots a face against
**2.676**, which is the 2.68 the first form measured on its own. The gate asserts
the fixed form is under 1.02 and the raced one over 1.5, so neither half can pass
by accident.

Getting that reproduction to agree took one more edit than expected, and it is
the codegen cliff again. Putting the key read back while **leaving the tag-less
form's now-dead "this frame, another face" branch in the loop** reproduced only
1.218 slots a face — less than half the defect. Removing that one unreachable
branch took it to 2.676. A branch that cannot fire still changed the severity of
a race by a factor of two, which is why a break here is measured and not argued.
