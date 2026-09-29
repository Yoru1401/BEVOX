---
type: Rule
title: 'An inference is not an observation'
description: 'Five of six doc errors found in one audit were true premises with unchecked conclusions bolted on; the gates cannot catch prose, so a claim about behaviour names the code that makes it true.'
tags: [process, documentation, correctness, audit]
generated: { by: claude-opus-5/claude-code, at: 2026-09-29T00:00:00Z }
sources:
  - id: bodies
    resource: /superpowers/specs/2026-09-29-bevox-body-composition-design.md
    title: Body composition in ray order
  - id: breaks
    resource: /concepts/deliberate-breaks.md
    title: Every gate is proven by a deliberate break
---

# What happened

A day of work was planned on a sentence in `gpu_bench.rs`:

> Composition is `1 + N` marches per ray, and a body march gets neither the beam
> seed nor the distance-field skip — both know only the static world — **so each
> is an unaccelerated march over the whole ray.**

The clause before the dash is true and checkable. The clause after it is an
**inference**, it was never checked, and it is false: `traverse_at` seeds its
first frame at `max(root_slab.t_enter, t_start)`, so a body march begins at the
body's bounding box and never walks the space in front of it.

On the strength of that sentence, a drift ledger entry and a plan were written
around rebuilding the traversal. Reading forty lines of the shader replaced both
with two much smaller changes.

# The rule

**A claim about what the code does names the code that makes it true.** Not the
module — the function, and ideally the line that decides it. A reader who cannot
get from the claim to the mechanism in one hop cannot check it, and an unchecked
claim is how this one survived.

**An inference gets marked as one, or gets checked.** "So each is X" is a
conclusion. Either verify it and state it as observed, or write "which should
mean X" so the next reader knows the load it can bear.

# Why the gates do not help

`crates/bevox/tests/map.rs` checks that every module, constant and march flag on
a map page exists and that every page fits a page. It checks **no prose at all**,
and cannot: no test holds "the tick integrates position after velocities" or
"a body march starts at its bounding box".

So the gated half of the docs is now reliable and the prose half is not, and the
prose half is where the decisions come from. That asymmetry is the thing to
remember, not the individual fix.

# What the audit found

Six errors, on 2026-09-29. Five were written the day before, in the map pages,
by the same hand that wrote the gates:

| Claim | Truth |
|---|---|
| `camera` owns the clip-to-world matrix | It is `upload::offset_from_clip` |
| `fracture` runs when voxels change | `solver` asks it on every contact, every tick |
| Staging runs in the render world's prepare | It runs in the main world's `Update`, before the extract boundary |
| A body costs 0.265 ms | 0.629 ms, re-measured once shadows and AO shipped |
| `edit` is the sphere brush | Also the fill and clear that detachment, merging and fracture stand on |
| A body march is unaccelerated over the whole ray | It starts at the body's bounding box |

**The pattern in all six is the same:** a true thing was written, and then
something that sounded like its consequence. The clip matrix *belongs* with the
camera, so it was described as living there. `fracture` *is* about editing
voxels, so it was grouped with the edit-time modules. 0.265 ms *was* measured —
before two features landed on top of it.

# The specs and plans, swept mechanically

The map pages were audited by reading. 22,763 lines of specs and plans cannot be,
so they were swept for the one thing a machine can check: **every backticked
identifier that appears nowhere in the code.** 89 of them.

Most are correctly historical and need nothing — Dwyer's own `geese`, `egui`,
`gvox` and `CharacterMover`; Bevy APIs from older versions; `.vox` chunk names;
and constants a plan records *removing*, such as `RESTITUTION_THRESHOLD`, which
the solver's own comment says was tried and dropped.

One category was not fine: **a plan naming a gate that does not exist.** A Breaks
table is an index into the test suite — its whole value is that a reader can go
run the gate that caught a break — so a name that does not resolve is worthless.
Three were found, all renamed on the way in:

| Planned | Shipped |
|---|---|
| `a_notch_frees_nothing` | `a_notch_frees_only_what_it_cuts_off` |
| `body_shadows_off_leave_every_entry_bit_identical_to_no_bodies_casting` | `body_shadows_off_leave_the_image_as_with_no_caster` |
| `a_cube_resting_on_a_cube_touches_at_four_corners` | `a_cube_resting_on_a_cube_touches_across_its_bottom_face` |

The third changed its **claim** with its name: the plan expected four contacts,
one per corner; the gate asserts twelve, because a cube resting squarely on
another meets it across the whole face and the corner spheres are joined by the
edge pairs along the rim. The planned number was wrong about the geometry.

Each plan now carries a "Names as shipped" map. The planned names stay in the
text as the record of what was intended.

**And one plan was superseded without being marked.** The Mouse Grab plan builds
a damped spring; milestone 7 replaced it with a joint, and nothing in that plan
still exists — not `GRAB_DAMPING`, `GRAB_FREQUENCY`, `GRAB_MAX_ACCEL`,
`GRAB_SPIN_DAMPING`, nor any of its three gates. The index listed it plainly
while the joints plan it replaced carried *(deprecated)*. Both are marked now.

**The rule this leaves:** when a name changes during execution, the plan's index
into the suite is updated or mapped in the same commit. The narrative is history
and stays; the index is a pointer and must resolve.

# The caution that came out of the audit itself

While checking whether `dense` is really only test and import scaffolding, a
`grep | head -5` truncated its output before the `vox.rs` matches, and a *true*
claim was nearly "corrected" into a false one.

**Verifying a claim with truncated evidence is not verifying it.** If the check
is worth running, it is worth reading all of.
