---
type: Measurement
title: 'Nine pixels a voxel face, and one slot each'
description: 'The sun shadow ray is recomputed 9.22 times a voxel face at 1080p on the near camera and 1.37 at the horizon, and break-even is the per-slot ray cost ratio rather than 1. A hash that indexes an 8x8 face patch in its low bits makes the compaction emit a surface-ordered work list for free, halving the per-slot ray to 9.9 ns and taking break-even from 2.2 to 1.2. The insert costs +1.0 ms at 1080p, not +0.4: a fixed harness stamp let every lane find its own last dispatch already claimed.'
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

**That figure is now gated, and it was not before.** Every use of
`pixels_with_slot` was a `println!`, and the one test that reported it was
`#[ignore]`d behind `distinct_keys > 0` -- so the metric that caught both of
this work's silent defects could not fail a suite run.
`nearly_every_hit_pixel_gets_a_slot`, not ignored, asserts that at least 95% of
the pixels recording a static-world hit came away with a slot, at both cameras
and both resolutions. Measured: **100.0% at three of the four, and 689,320 of
689,356 at the distant camera at 1080p**. The break is the flat-axis failure
below in one line -- the local index taken from `(x, y)` instead of the two
tangent axes -- and it reads **43.8%**, so the threshold is known to be able to
fail.

# The worst case: the horizon, and what a coherent hash did to it

`the_worst_case_is_measured_against_the_parent`. `distant_camera` stands high
and near one edge of the bench scene, aimed down-range at the far edge of the
floor, so the frame is terrain at 800 to 1200 voxels rather than half sky. A
vertical field of 0.9 rad over 1080 rows is 8.33e-4 rad a pixel, so a voxel
subtends about a pixel at that distance -- the condition where the store has
nothing to offer.

| | 1280x720 | 1920x1080 |
|---|---|---|
| pixels that inserted | 306 403 | 689 321 |
| distinct `(voxel, face)` keys | 304 731 | 503 650 |
| **the redundancy** | **1.01** | **1.37** |
| slots a face | 1.0000 | 1.0000 |

**With the scrambling hash this was a 4.4 to 7.1 ms regression**, measured
against `34f2d45` -- the revision before any of this work, whose shader has no
store at all. The store did not merely fail to help: at 1080p, taking the shadow
march out of the primary saved 6.25 ms and putting it back as a sun pass over
503 650 slots cost **10.08**.

**The spatially coherent hash below closed most of it.** Three invocations, each
A/B/A interleaved in one command across two source files, GPU clock, positive is
slower:

| against `34f2d45` | 1280x720 | 1920x1080 |
|---|---|---|
| scrambling hash | +4.36, +4.43, +4.58 | +6.79, +6.83, +7.05 |
| **coherent hash** | **+1.03, +1.32, +1.56** | **+0.43, +0.58, +0.59** |
| drift | 0.04-0.21 | 0.01-0.24 |

And the near field did not pay for it -- the bench camera went **-2.1 to -2.6 ms
at 720p and about -5.4 at 1080p** against the same baseline, which is the +5.3 ms
Task 2 collected, intact.

**It is still slower at the horizon at 1080p, and the honest figure is +0.75 ms
rather than +0.5.** The +0.43/+0.58/+0.59 above was taken with the harness
holding one fixed frame stamp, which is a store state the app never has -- see
[what a fixed stamp was hiding](#what-a-fixed-stamp-was-hiding) below. Re-read
with the stamp advancing per submit, three invocations:

| against `34f2d45`, stamp advancing | 1280x720 | 1920x1080 |
|---|---|---|
| **coherent hash** | **+1.70, +1.62, +1.62** | **+0.87, +0.78, +0.60** |
| drift | 0.04-0.23 | 0.03-0.06 |

It is a larger residue and a far better-conditioned reading: +0.75 ms against
drift near 0.05 is a fifteenfold margin, where the figure it replaces was the
thinnest on this page. The magnitudes are not comparable across sessions and
only the within-session A/B/A stands, but the *conditioning* is the point --
this number is now worth holding.

**The two baselines still disagree on the sign, and that is not an artifact.**
Against `4560d58`, the parent that inserts and reads nothing, the same
configuration reads **-1.67 to -2.12 ms**, i.e. faster, where it read -0.15 to
-0.50 with the fixed stamp. The gap grew rather than cancelling, because
`4560d58` pays the insert in full and collects nothing for it: it is strictly a
worse revision than either side of the comparison, and the distance between the
two baselines is the insert's real cost at 503,650 distinct faces. **The
disagreement is the question each baseline answers, not noise in either.**

**Whether that residue wants a distance cutoff is a design decision and is not
made here.**

# Why 1 is not the break-even point, and why it is now 1.2 rather than 2.2

The sun pass's ray is incoherent where the per-pixel ray is not, and that ratio
-- not 1 -- is what the store must share an answer across before it saves
anything:

| per-slot ray, 1920x1080 distant | ns a ray | break-even redundancy |
|---|---|---|
| scrambling hash | 19.4 | **2.21** |
| **brick-coherent hash** | **9.9** | **1.20-1.22** |

Three invocations give 1.22, 1.21 and 1.20 at that camera, which is the
best-conditioned of the four configurations: the comparator is a difference of
two separately-measured pass times and is noisy at 720p, where the same
calculation ranged 0.81 to 1.20.

**So the premise "any redundancy above 1 is a win" was wrong, and the threshold
is a measured ratio.** 9.22 clears 2.2 four times over, which is why the near
camera collected 5.3 ms even before the hash. 1.37 did not clear 2.2 and does
clear 1.2 -- which is exactly why the horizon went from a 7 ms loss to roughly
break-even rather than to a win.

# The hash: a face-patch index below, a scrambled key above

**`sun_hash` puts the low `SUN_BRICK_BITS * 2` bits of the slot index at the
face's position within an 8x8 patch of the surface it lies on, and a finaliser
over everything else in the key above that.** Faces near each other on a surface
land near each other in the table, deliberately.

The compaction already walks the table in index order, so **it emits a roughly
surface-ordered work list for free** -- no sort, no extra pass, no run-time cost
at all. A run of 64 consecutive slots is one 8x8 patch of one surface, which is
one 64-lane workgroup marching 64 near-identical rays. That is the whole
mechanism, and it halved the per-slot ray.

**Two things had to be right, and getting either wrong cost half the inserts.**

1. **The local index is two-dimensional, because a face is flat.** The first form
   used a 3D brick index -- 9 bits over all three axes of an 8x8x8 brick -- and a
   floor is flat in y, so every floor face in a brick shared one value of
   `y & 7` and only 64 of that run's 512 slots were ever reachable. Four patches
   a run each wanting the same 64 of 512 saturated it: **51% of hit pixels on the
   distant camera came away with no slot**, and 17% on the near one. Indexing by
   the two axes *tangent* to the face uses every slot in the run, and both
   figures went to zero. The position *along* the normal moves into the scrambled
   part rather than being dropped, or two parallel faces one voxel apart would
   collide in every run they tried.
2. **The probe must advance by a run, not by a slot.** A collision means another
   patch holds this run, and the slots just past it are that patch's too, so
   linear probing cannot escape a run in `SUN_PROBES` tries. Adding
   `SUN_BRICK_SLOTS` tries eight *different* runs, and because the width is a
   power of two it leaves the low bits alone -- so a face keeps its own local
   offset in whichever run takes it, and the coherence survives the probe.

**The clustering did not cost the primary pass, which is where the probes are
paid and where the cost would have landed.** Twelve readings across three
invocations and four configurations gave +0.44 to -0.73 ms, swinging both ways
for the *same* configuration between invocations (bench 1080p read -0.22, -0.18
and +0.22). That is noise, not an effect, and the honest statement is that no
probe cost was measurable -- plausibly because the insert gets the same cache
locality the sun pass does.

**Slots a face stayed 1.0000** at both cameras and both resolutions, which was
the gate that outranked any speed result: two faces of one patch cannot collide
at all, since their local indices differ by construction. What collides is two
*patches* landing on one run, and that is what the probe and the load factor are
for.

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

# What a fixed stamp was hiding

`tests/common/mod.rs` held `SUN_STAMP` at 1 for every dispatch, and every timing
path -- `pass_times`, `aba_both`, `time_dispatches` -- takes a median over seven
or more submits of one `Prepared`, whose claim words persist between them.
**From submit 2 onward every lane hit `held == want` on its first probe: no
compare-exchange, no key write, no inter-lane contention.** The app advances the
stamp every frame and re-claims every slot, so the harness was measuring a store
state that exists for exactly one frame of the app's life.

Every submit now writes `ao_params.z` through `pipeline::next_sun_stamp` -- four
bytes against a dispatch of milliseconds, paid on both sides of every
comparison. What moved:

| insert on against off, bench camera, GPU clock | fixed stamp | stamp advancing |
|---|---|---|
| 1280x720 | +0.28 ms | **+0.39, +0.50, +0.87** |
| 1920x1080 | about +0.4 ms | **+0.91, +0.95, +1.06** |

**The insert costs about +1.0 ms at 1080p, not +0.4 -- 12% of the 8.36 ms
ceiling rather than 5%.** Drift 0.02-0.39 on all six readings. Taking the claim
loop out of the measurement took three fifths of its cost with it, which is what
a compare-exchange that never has to exchange is worth.

It also explains the baselines above. `4560d58` inserts and reads nothing, so
the whole of that understated cost sat on it and on the four-pass side at once
and cancelled; priced honestly, both pay it and the horizon's two baselines are
2.5 ms apart rather than 1.0.

# What the store costs when nothing reads it

Five invocations, each A/B/A interleaved, insert on against off, GPU clock,
after the fix and **with the harness's fixed frame stamp**, which is why these
are not the numbers above:

| | readings | taken |
|---|---|---|
| 1280x720 | +0.22, +0.26, +0.28, +0.31, +0.38 | **about +0.28 ms** |
| 1920x1080 | +0.30, +0.34, +0.43, +0.60 | **about +0.4 ms** |

One 1080p reading, +0.27, came with a baseline drift of 0.81 ms and is not a
result. Drift was 0.00-0.29 ms on all the rest, against 0.42-2.32 for the raced
form, which cost **+0.5 ms at 720p and about +0.9 at 1080p** — so the fix made
the store cheaper as well as tighter, because it does a third of the claims and
no key reads.

# The break, and what the cliff did to it

`reintroducing_the_insert_race_costs_slots_per_face` puts the old form back by
text and reads the store twice in one invocation: 1.000 slots a face against the
raced form's. The gate asserts the fixed form is under 1.02 and the raced one
over 1.5, so neither half can pass by accident.

**The raced form's own figure depends on the hash, and it moved when the hash
did: 2.676 with the scrambling hash, 1.676 with the coherent one.** That is the
defect behaving as described rather than a weaker reproduction — the race is
lanes of one workgroup reaching one slot in lockstep, and a coherent hash sends
a workgroup's faces to *different* slots in the same run by construction, so
fewer of them collide in the first place. It leaves the >1.5 threshold with
little margin, which is worth knowing before anyone changes
`SUN_BRICK_BITS`: the gate would start failing as a false alarm rather than
silently passing, which is the safe direction, but it would still need its
number re-read rather than its threshold lowered.

Getting that reproduction to agree took one more edit than expected, and it is
the codegen cliff again. Putting the key read back while **leaving the tag-less
form's now-dead "this frame, another face" branch in the loop** reproduced only
1.218 slots a face — less than half the defect. Removing that one unreachable
branch took it to 2.676. A branch that cannot fire still changed the severity of
a race by a factor of two, which is why a break here is measured and not argued.
