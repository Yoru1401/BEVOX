---
type: Measurement
title: 'The sun pass was lane-bound, not ray-bound'
description: 'Marching one shadow ray per slot over a 5.65%-occupied table cost 15.01 ms for 118,423 rays against 8.14 ms for 1,092,019; the waste was idle lanes, and compacting the slots into a dense work list took it to 2.15 ms and collected 5.0 of the 8.36 ms ceiling. The 2.4x a ray left over was ray incoherence, and a surface-coherent hash halved it for free.'
tags: [performance, gpu, lighting, dispatch]
generated: { by: claude-opus-5/claude-code, at: 2026-10-03T00:00:00Z }
sources:
  - id: design
    resource: /superpowers/specs/2026-10-03-bevox-per-voxel-sun-design.md
    title: 'Sun visibility per voxel, not per pixel'
  - id: occupancy
    resource: /concepts/sun-store-occupancy.md
    title: 'Nine pixels a voxel face, and one slot each'
  - id: cliff
    resource: /concepts/gpu-codegen-cliff.md
    title: The GPU codegen cliff
---

# What was collected

GTX 1650, bench scene and camera, `DEFAULT`, no bodies. The lit path is four
dispatches where it was one, A/B/A interleaved in one invocation against the
single-pass `march`, GPU clock.

| | 1280x720 | 1920x1080 |
|---|---|---|
| the ceiling: what the shadow march costs | 4.67-4.82 ms | **7.98-8.36 ms** |
| **collected** | **+1.4 ms** | **+5.3 ms** |
| drift on those readings | 0.01-0.33 | 0.03-0.51 |

Four invocations at 1080p gave +5.00, +5.13, +5.23 and +5.56 on the GPU clock
and +4.80 to +5.17 on the wall; at 720p, +1.14, +1.45, +1.47 and +1.52. **About
64% of the ceiling at 1080p and 30% at 720p**, and the gap between the two is
the redundancy itself: 9.22 pixels a face at 1080p against 5.16 at 720p. Two of
the four 1080p readings are cross-file against the parent revision's shader
rather than against `march` inside the same module, and they agree.

Per pass, GPU ms, median of seven, at 1920x1080:

| one dispatch | | four dispatches | |
|---|---|---|---|
| beam | 0.78 | beam | 0.75 |
| `march` | 25.00 | `march_primary` | 16.71 |
| | | `sun_compact` | 0.12 |
| | | `sun_pass` | 2.15 |
| | | `march_composite` | 0.59 |
| **total** | **25.78** | **total** | **20.33** |

A second invocation gave 25.59 against 20.18, the same 5.4 ms.

**The pixel side behaves exactly as the spec predicted.** `march_primary` is
`march` minus the shadow march — 16.71 against 25.00 is 8.29 ms, and the ceiling
is 7.98-8.36 — and the composite that reads the answer back costs 0.59 ms.
Nothing in that half was a surprise.

# The surprise: 118,423 rays cost more than 1,092,019

The first form dispatched the sun pass over the whole table, one invocation per
slot, which is what the spec describes. It cost **15.01 ms at 1080p** and made
the whole change **7.42 ms slower than doing nothing**.

The same pass with every slot skipped costs **0.05 ms over 32,768 workgroups**,
so none of the 15 ms was the empty invocations' own instructions. It was all in
the marching:

| | rays | ms | ns a ray |
|---|---|---|---|
| per pixel, in `march` | 1 092 019 | 8.14 | 7.5 |
| per slot, whole table | 118 423 | 15.01 | **127** |
| per slot, compacted | 118 423 | 2.15 | 18.2 |

**17x the cost a ray, and the table's load factor is 5.65%.** A 64-lane
workgroup of consecutive slots holds about 3.6 occupied ones, a workgroup costs
what its slowest lane costs, and 32/1.8 occupied per 32-lane warp is 17.8. The
two numbers are the same number. **The waste was idle lanes, not work.**

So compacting the occupied slots into a dense list is not a tuning option, which
is how the spec left it: without it the whole change is a regression. It is one
extra dispatch — a scan of the table appending each occupied slot through one
`atomicAdd` — and it costs **0.12 ms**, against the 12.9 ms it recovers.

**The remaining 2.4x a ray is ray incoherence and is not a bug.** Neighbouring
pixels march near-identical shadow rays; neighbouring slots are unrelated voxels
scattered by the hash, so no two rays in a warp share a cache line or a path
length. 18.2 ns a ray against 7.5 is what that costs, and it is already priced
into the +5.0 ms. Ordering the work list by voxel locality is the only lever
left on this pass, and it has not been measured.

# That 2.4x is also the break-even redundancy, and a coherent hash halved it

A per-slot ray costing 2.2-2.4x a per-pixel one means the store must share each
answer that many ways before it saves anything. **The break-even redundancy is
this ratio, not 1.** On the bench camera 9.22 clears it four times over; at the
horizon, where a voxel covers about a pixel, 1.37 did not clear it and the four
passes were **6.8 to 7.1 ms slower than doing nothing at 1080p**.

**This page's closing line above -- that ordering the work list is the only lever
left -- was right, and the lever turned out to cost nothing to pull.** The
compaction already walks the table in index order, so a hash whose low bits are a
face's position in an 8x8 patch of its surface makes the work list come out
surface-ordered **with no sort and no extra pass**:

| 1920x1080, distant camera | sun pass | ns a ray | break-even |
|---|---|---|---|
| scrambling hash | 9.47-9.75 ms | 19.4 | 2.21 |
| **face-patch hash** | **4.96-5.06 ms** | **9.9** | **1.20-1.22** |

**The incoherence was about half the per-slot ray's cost, and it was
addressable.** On the bench camera the same change took the sun pass from 2.19 to
1.26 ms. The measurement, the two ways of getting the index wrong, and the
probe-sequence change it needs are in [nine pixels a voxel
face](sun-store-occupancy.md).

What that leaves is **9.9 ns a slot ray against about 8.2 ns a pixel ray** -- a
ratio near 1.2 rather than 2.4, and most of what remains is no longer ordering.

# What the pixel side needed that the spec did not say

The composite cannot re-derive the hit: marching the primary ray twice would
cost more than the shadow ray is worth. So the primary pass leaves a **7-word
record per pixel** — slot, the full 39-bit key, material, the shading normal and
the ambient occlusion — and the composite reads it. The normal is stored because
`implicit_normal` is six tree descents; the shadow origin is **not** stored,
because a static-world origin is `vec3(voxel) + 0.5 + face_normal * 0.75` and the
key gives both back exactly.

That region is 232 MB at 3840x2160, up from 33 MB, and it changes what a window
larger than the capacity means. Before, a pixel past it simply lost the saving;
now it would have no shading at all, so **`dispatch_march` falls back to the
single-pass `march` for a window past `SUN_PIXEL_CAPACITY`**. The fallback is in
Rust and not a branch in the composite, because a branch a benchmark never takes
has twice cost this project tens of percent of a frame.

**The sky and body hits are shaded in the primary pass and marked done.** A body
keeps the per-pixel path whole — its `voxel` is a coordinate in the body's own
volume, so a key built from it would alias onto the static voxel at the same
coordinates — and finishing it in the primary leaves the composite with one
fallback path rather than two.

# The cliff, a fourth time

Splitting the pass cost the **old** path, which is byte for byte unedited, **+0.17
to +0.67 ms at 1080p** against the parent revision, across three invocations with
drift 0.12-0.40. The only thing that touched it is the insert being split into
`sun_insert` and `sun_claim` so the primary pass can pass a key it already holds,
and the per-pixel write gaining a stride. `march` is now the fallback and the
baseline, so this is not on the shipped path — but it is the fourth time an edit
that cannot change a pixel has moved this shader by tenths of a millisecond, and
the end-to-end number above is measured across the two source files for exactly
that reason.

# The read side is where the key guard lives now

`sun_claim` returns a slot on a matching claim word without reading the key,
because reading it is what the insert race was. **`march_composite` compares the
full 39-bit key and marches its own ray on a mismatch**, and that is the only
thing standing between a tag-and-slot collision and a pixel reading another
voxel's sun.

`the_key_check_is_what_keeps_two_faces_apart` is the break, and it is an image
break — which Task 1's could not be, because nothing read the store then. With
every tag reported equal, thousands of faces land in a slot another voxel owns:
**0 pixels differ with the key check in place and 4,389 without it**, of 921,600.
The check must sit in a later pass than the insert: done in the primary it would
read the key at the instruction step another lane writes it, which is the race
again, and every loser would fall back to marching.

`the_sun_pass_is_what_the_image_depends_on` is the other half — a sun pass
writing a constant moves 380,435 pixels, one switched off moves 16,264, and the
two differ from each other, which is what rules out a composite that quietly
marched its own ray all along.
