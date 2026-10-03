---
type: Measurement
title: 'Nine pixels a voxel face, and one slot each'
description: 'The sun shadow ray is recomputed 9.22 times a voxel face at 1080p on the near camera, so the redundancy is there to collect, and the insert claims exactly one slot per face because the frame stamp and the key tag share one word. At the horizon the factor is 1.01-1.37 and the store is a 4.4-7.0 ms regression, because break-even needs about 2.2.'
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

# The worst case: the horizon, where the factor is 1 and the store loses

**`the_worst_case_is_measured_against_the_parent` is a failed gate and the
number is on this page because it is a design input, not a bug to tune away.**

`distant_camera` stands high and near one edge of the bench scene, aimed
down-range at the far edge of the floor, so the frame is terrain at 800 to 1200
voxels rather than half sky. A vertical field of 0.9 rad over 1080 rows is
8.33e-4 rad a pixel, so a voxel subtends about a pixel at that distance — which
is the condition the store has nothing to offer.

| | 1280x720 | 1920x1080 |
|---|---|---|
| pixels that inserted | 306 403 | 689 285 |
| distinct `(voxel, face)` keys | 304 731 | 503 636 |
| **the redundancy** | **1.01** | **1.37** |
| four passes against `34f2d45`, GPU clock | **+4.36 to +4.58 ms** | **+6.79 to +7.05 ms** |
| the same against `4560d58`, which already inserts | +3.63 to +4.45 | +5.24 to +6.83 |
| drift on those readings | 0.06-0.44 | 0.01-0.30 |

Three invocations, each A/B/A interleaved in one command, across two source
files rather than within one module. **No reading's drift came near its effect.**
`4560d58` is this branch's parent and already pays the insert, so it is the
flattering baseline of the two; `34f2d45` has no store at all and is the honest
one.

**Where it goes, at 1920x1080, GPU ms, median of seven:**

| one dispatch | | four dispatches | |
|---|---|---|---|
| `march` | 21.76 | `march_primary` | 15.51 |
| | | `sun_compact` | 0.12 |
| | | `sun_pass` | 10.08 |
| | | `march_composite` | 1.37 |

Taking the shadow march out of the primary saved 6.25 ms. Putting it back as a
sun pass over 503 636 slots cost 10.08. **The store did not fail to help; it
actively spent 1.6x what it saved, before the compaction and the composite.**

# Why 1 is not the break-even point: 2.2 is

The sun pass marches **20.0 ns a ray** here against **9.1 ns** in the primary it
came out of — the ray incoherence already measured in [the sun pass was
lane-bound](sun-pass-is-lane-bound.md), where it is 18.2 against 7.5 on the near
camera. A per-slot ray costs about 2.2x a per-pixel one, so the store has to
share each answer **2.2 ways just to break even on the marching**, and more than
that to pay for the insert, the compaction entry, the 7-word record and the
composite's extra read.

**So the premise "any redundancy above 1 is a win" is wrong, and the correct
threshold is a measured ratio rather than a bound.** 9.22 clears 2.2 four times
over, which is why the near camera collects 5.3 ms of an 8.36 ms ceiling. 1.37
does not clear it at all. The same bench prints the ratio per resolution; at
720p it read 1.64, where the primary's own reading is noisier.

**Looking at distant terrain is something players do constantly, so this is not
a corner.** Whether the store takes a distance cutoff, or the work list is
ordered by voxel locality to attack the 2.2x itself, is an open decision and is
not made here.

# What it takes to overflow the table

`shrinking_the_table_until_it_overflows_is_measured`, bench camera. The slot
count travels in the uniform, so this shrinks it with no shader edit and no
smaller buffer — one compiled module against itself, which is the only way to
read an occupancy change without the codegen cliff in the way.

Pixels that came away with a slot, against the 485 361 and 1 092 019 that do at
2^21:

| slots | 720p | 1080p | load in faces at 1080p |
|---|---|---|---|
| 2^20 | 100% | 100% | 0.11 |
| 2^19 | 100% | 100% | 0.23 |
| **2^18** | **99.8%** | **99.1%** | 0.45 |
| **2^17** | **91.9%** | **79.7%** | 0.83 |
| 2^16 | 46.1% | 32.6% | 1.00 |
| 2^14 | 14.4% | 13.8% | 1.00 |

**The first failed insert appears at 2^19 and it is 25 pixels of 485 361.** The
table is twenty times the face count at 2^21, so the first four halvings cost
nothing at all; overflow becomes a real fraction of the screen at **2^18**, and
bites at **2^17**, a 0.83 load factor in faces. Eight probes is what buys that:
a linear probe at a 0.45 load finds a free slot almost always and at 1.00 never.

**The image did not move at any of it.**
`overflowing_the_table_changes_no_pixel` runs the composite at **2^14 slots**,
where 86% of pixels give up and the rest mostly collide, against the per-pixel
`march` at both resolutions with no bodies and with sixteen: **0 differing pixels
in all four**. It asserts the overflow happened before it asserts the image,
because a bit-identity gate on a table that was never full proves nothing.

Only the frame moved: **+3.01 ms at 720p and +7.04 at 1080p** at 2^16, +2.17 and
+6.98 at 2^14, drift 0.44-0.77. That is the saving being handed back as pixels
return to marching their own rays while still paying for the store — which is
the same arithmetic as the horizon above, reached from the other direction.

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
