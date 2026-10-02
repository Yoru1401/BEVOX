---
type: Measurement
title: 'The impulse threshold is a force threshold for everything it was built for'
description: 'Fracture thresholds the accumulated contact impulse, which is what sees a slow crush. Measured: the crush load is proportional to dt and so is a force wearing an impulse''s units, the "resting load" the strengths were calibrated against is a settling impact and not weight, the landing peak owes nothing to the bias and the press is clamped at 100%, and past five cubes a stack''s landing is above the ceiling a grab can reach, so the window is empty and only the closing speed separates the regimes.'
tags: [physics, fracture, dwyer, measurement, open-defect]
generated: { by: claude-opus-5/claude-code, at: 2026-10-02T00:00:00Z }
sources:
  - id: plan
    resource: /superpowers/plans/2026-10-02-bevox-fracture-on-impulse.md
    title: Fracture on a size-scaled impulse
  - id: fracture
    resource: /superpowers/plans/2026-09-24-bevox-fracture.md
    title: Fracture Implementation Plan
---

# The headline

**The blow is the accumulated contact impulse, not the closing speed.** A
closing speed cannot see a crush: the mouse grab is a joint with a 1.962e8
force limit driving toward a *velocity* goal, so a grabbed body leaning on
something presses hard while the contact holds both surfaces still. The
approach speed is then under 0.011 voxels a second however hard the press, and
a player could lean a rock through a window without marking it.
`a_slow_crush_breaks_what_it_presses` is that gate.

**And for a crush, that impulse is a force threshold in an impulse's units.**
Measured over four tick rates on 2026-10-02: the crush load is proportional to
`dt`, so the one case this rule exists for is rate-dependent, which is the
property Dwyer's argument for the impulse rejects. Dwyer's argument still holds
for a *collision* -- the slam columns below are flat.

# The rate sweep, measured 2026-10-02

`solver::tests::what_the_tick_rate_does_to_the_blow`, `#[ignore]`d. The same
wall-clock duration at every rate, rates interleaved in one invocation with 64
Hz repeated last as a determinism check, every material the unbreakable
fixture so no measured load is perturbed by the scene coming apart.

| Hz | rest max (at tick) | rest settled | `m g dt` control | crush peak | slam peak | slam sum | slam `m dv` |
| --- | --- | --- | --- | --- | --- | --- | --- |
| 64 | 331,306 (5) | 18,798 | 490,430 | 365,906 | 1,047,814 | 1,517,884 | 3,841,039 |
| 128 | 311,196 (9) | 10,028 | 245,242 | 308,430 | 666,791 | 1,208,314 | 3,855,816 |
| 256 | 322,637 (17) | 5,157 | 122,622 | 177,597 | 1,333,582 | 1,374,443 | 3,889,906 |
| 512 | 294,375 (33) | 3,122 | 61,309 | 93,512 | 1,238,326 | 1,314,284 | 3,863,532 |
| **64/512** | **1.13x** | **6.02x** | **8.00x** | **3.91x** | **0.85x** | **1.15x** | **0.99x** |

8.00x is exactly proportional to `dt`; 1.00x is rate-independent. The `m g dt`
control is the bottom body's own delivered normal impulse, the quantity
`a_body_at_rest_stays_at_rest` already pins at `m g dt`, and it lands on 8.00x
to two decimals -- so the sweep can tell the two regimes apart.

**Proportional to `dt`:** the settled resting load, and **the crush load**.
**Flat:** the landing transient, the slam's `peak[at]`, the slam's blow summed
over the ticks the detector spread it across, and the momentum the slam
actually exchanged.

## What that costs, at the shipped 350,000

The crush load at 128 Hz is **308,430, below `GLASS_STRENGTH`**. So
`a_slow_crush_breaks_what_it_presses` -- which runs only at `DT` = 1/64 s --
**stops breaking anything the moment the tick rate doubles**, while the
landing transient the strength is calibrated above does not move. The window
does not merely narrow above 64 Hz; it closes.

The slam columns are the other half of the result and they are good news: a
collision's exchanged momentum is flat to 1.3% across a factor of eight in
rate. `peak[at]` for a collision tracks it loosely (0.85x, and
non-monotonically, because at a finer rate the detector sees the collision
coming and spreads it over more ticks). The rule is sound for impacts. It is
the held load that is a force.

# The "resting load" is a landing, not weight

**Every "resting load" number in this page's history is a settling impact.**
The fixture starts its cubes 0.2 voxels above where they settle, so the
measured maximum lands on **tick 5 at 64 Hz and tick 33 at 512** -- the same
78 ms -- and is **17x the load the stack then holds**:

This page used to add "and `peak[at]` includes the push-out velocity the
solver's bias adds". **That is false, measured 2026-10-02** by
`where_a_landing_peak_comes_from`: the bias is **0.0%** of the landing peak.
`solve` takes the `BIAS` branch only when the separation is negative, and then
uses `(separation + SLOP).min(0.0)`; the deepest separation anywhere in the
landing is **-0.0135**, shallower than `SLOP`, so the term is identically
zero for every contact of it. The landing peak is momentum and nothing else --
the four floor contacts' end-of-substep impulses sum over the tick to
**1,805,350** against a `sum m dv + m g dt` of **1,805,350**, to the last
printed digit.

| Stack of 4-voxel cubes, at 64 Hz | Landing maximum | Settled load |
| --- | --- | --- |
| 3 -- the gate's fixture | 331,306 | 18,798 |

That is why the landing column is flat across the rates and the settled column
halves: one is a collision and the other is weight. **`GLASS_STRENGTH` is
calibrated against a landing on one side and a rate-dependent joint clamp on
the other.** Neither of the two numbers it sits between is resting weight.

# The open defects

## A four-high glass stack destroys itself

`a_four_high_glass_stack_stands`, `#[ignore]`d, **fails in the shipped build**:
2 fractures, the first at **tick 5**, heaviest blow **356,751** against a
strength of 350,000. The same four cubes started at their settled heights
instead break **nothing**, with a heaviest blow of 56,776.

**So the cliff is at four cubes, not "past four"** -- this page previously put
it at six -- and what it is is the *landing*, not the weight. Raising
`GLASS_STRENGTH` is not the fix: 350,000 is already within 5% of the 365,906 a
grab can press with at 64 Hz, and above 350,000 the crush gate is spent.

## The window is empty, not narrow, measured 2026-10-02

`how_the_landing_and_the_held_load_scale`, 1200 ticks a stack, unbreakable
glass so nothing comes apart:

| n | landing peak | at tick | held load | landing/held | `m g h n / 4` | late speed | sleeping |
|---|---|---|---|---|---|---|---|
| 1 | 126,129 | 4 | 10,231 | 12.3 | 6,131 | 0.0000 | 1 |
| 2 | 246,227 | 5 | 14,177 | 17.4 | 12,263 | 0.0000 | 2 |
| 3 | 331,306 | 5 | 18,798 | 17.6 | 18,394 | 0.0000 | 3 |
| 4 | 356,751 | 5 | 39,308 | 9.1 | 24,525 | 0.0000 | 4 |
| 6 | 391,195 | 5 | 118,320 | 3.3 | 36,788 | 2.0251 | 0 |
| 8 | 414,166 | 5 | 176,647 | 2.3 | 49,050 | 4.1189 | 0 |

**The six-high and eight-high landings, 391,195 and 414,166, are both above
the 365,906 a saturated grab can reach.** So there is no value of
`GLASS_STRENGTH` at which a six-high glass stack settles and a player can
still crush glass. The window is not narrow at four cubes and gone at some
future height -- past five cubes there is nothing to tune.

The landing is **sub-linear**: 3.28x the peak for 8x the mass, about
`n^0.55`, because a taller stack's cubes separate in flight and arrive over
several ticks. The held load is roughly **linear** where it means anything --
`m g h n / 4` is the fair share of weight at four corner contacts, and n = 3
sits at 1.02x of it. **n = 6 and n = 8 never settle**: late speeds of 2.03 and
4.12 v/s with the load swinging 59,686-133,272 and 86,716-217,383 and nothing
asleep, so those two "held load" figures are not weight. That is a stacking
defect, not a fracture one, and it is the first thing the ladder found that
`the_240_to_1_load_collapses_through_the_floor` does not already name.

## The blow is one contact's share of an indeterminate split

Measured 2026-10-02 by `where_a_landing_peak_comes_from` and
`what_the_sweep_counts_do_to_the_landing_peak`. The four floor contacts of a
cube landing flat are geometrically symmetric, so a fair split of the 744,901
that substep 0 of the four-high landing delivers would be 186,225 each, under
`GLASS_STRENGTH`. The measured split is **356,751 / 158,882 / 113,937 /
115,332**: 47.9% on whichever contact the sweep reaches first.

**Not under-convergence.** Through `step_with`, balanced sweeps 1 -> 16 (the
shipped counts untouched), the peak goes 356,751 -> 399,486 -> 459,965 ->
580,848 -> 622,623 and the share stays 47.9% -> 47.5% -> 45.0% -> 45.4% ->
44.8%. More sweeps make it **worse**, by killing more of the arriving velocity
inside substep 0 rather than spreading it over four. A redundant four-point
normal contact has no unique impulse distribution; sequential impulses pick an
uneven one and keep it.

So **a palette strength is compared against an arbitrary fraction of a load,
and the fraction moves by 1.7x with body size alone** -- 77% of the clamp for
a 2-voxel cube against 45% for an 8-voxel one, in the press. This is not the
cause of the load window: the regimes are 1.06x apart however the impulse is
counted. It is an accuracy bound on whatever threshold replaces this one, and
nothing in the suite would notice it changing.

## The slow crush works on ice, and on nothing else

The crush load **saturates** between about **3.4e5 and 5.9e5** -- near-flat in
the pressed body's mass, and flat in how long and how far the target is driven
(0.002 a tick for 400 ticks and for 1500 give the identical 365,906):

| Pressed body, 64 Hz | Mass | Crush load |
| --- | --- | --- |
| 2-voxel cube | 8,000 | 588,636 |
| 4-voxel cube | 64,000 | 365,906 |
| 8-voxel cube | 512,000 | 343,399 |
| 10-voxel cube | 1,000,000 | 395,677 |

Against the demo palette: **ice is 350,000, brick is 729,167, stone is
1,312,500, and `DEFAULT_STRENGTH` is 2,800,000.** A grab cannot reach any of
them but ice.

**So the slow crush this whole rule change exists for works on ice and on
nothing else.** Not on terrain, not on stone, not on brick. A player can lean
a rock on a brick wall with 1.962e8 of force behind it for as long as they
like and it will never mark.

And nothing checks that. `the_demo_palette_outlasts_a_resting_stack` asserts
only a **floor** -- that no material is weaker than the load a stack already
carries. There is no ceiling assertion, so **a material above the crush load
is silently uncrushable** and the suite stays green.

## Why the press stops where it does

This page used to say the press "settles into a penetration equilibrium rather
than running up to its force limit", comparing the measured 365,906 against
"its 1.962e8 force limit". **That is an impulse against a force** -- the exact
units confusion this change exists to fix.

`joint.rs`'s `solve_linear` clamps the joint's *accumulated impulse*, not its
force: `let limit = joint.max_force / inv_h`, with `inv_h = SUBSTEPS / dt`. So
the real ceiling is `GRAB_MAX_FORCE * dt / SUBSTEPS`, and it moves with the
rate:

| Hz | clamp = `GRAB_MAX_FORCE dt / SUBSTEPS` | measured crush | fraction |
| --- | --- | --- | --- |
| 64 | 766,406 | 365,906 | 48% |
| 128 | 383,203 | 308,430 | 81% |
| 256 | 191,602 | 177,597 | 93% |
| 512 | 95,801 | 93,512 | 98% |

**At 256 Hz and above the press is simply clamped**, which is why the crush
column tracks `dt`.

**And at 64 Hz it is clamped too. The penetration-equilibrium reading is
dead.** Measured 2026-10-02 by `what_stops_the_press`: the grab's accumulated
linear impulse is at **100.0% of its clamp from tick 40 onward**, flat to the
end of a 400-tick press. What the "48%" measures is not the press -- it is
**how the clamp divides among the contacts**. At equilibrium the four contacts
against the world carry 1 / 363,231 / 283,028 / 144,664, summing to
**790,924** against the grab's clamped **766,406** plus the body's own
`m g h` of 24,525 -- seven parts in 790,000. The heaviest takes **47.4%**, and
`766,406 x 0.477` is the 365,906.

So the crush load is **the clamp divided by a contact count**, which is also
why the mass table above is near-flat: a 2-voxel cube has fewer contacts to
divide by and reaches 77% of the clamp, an 8-voxel cube has more and reaches
45%.

# What would actually fix this

Two candidates, neither done:

- **Read the load a contact has carried *steadily* apart from the load it just
  took.** The steady load is exactly what resting weight is and exactly what a
  crush is not. This is the one that separates the three regimes the sweep
  found instead of pricing them against each other.
- ~~**Stop counting the bias's push-out in the blow.**~~ **Withdrawn,
  measured 2026-10-02: the bias is 0.0% of the landing peak** (see above), so
  this would change it by nothing.
- **Tell the regimes apart by the closing speed.** `approach[at]` is already
  computed per contact and fracture does not read it. Measured 2026-10-02 by
  `what_the_closing_speed_says_in_each_regime`: **-5.3639 v/s** for the
  four-high landing, **+0.0003** for the same stack settled, **-0.0003** for a
  saturated grab. Four orders of magnitude, against **1.06x** between the two
  loads as impulses.

Raising `GLASS_STRENGTH` is not a candidate: it walks into the crush ceiling
from the other side.

**And no way of counting the impulse separates the two, measured
2026-10-02.** One substep's normal impulse summed over every floor contact is
**744,901** for the four-high landing and **790,924** for the saturated press
-- 1.06x apart, no better than the 1.03x their maxima already are. The two
events deliver the same momentum per substep; only the closing speed differs.
Summing a contact patch instead of taking its maximum therefore buys nothing,
and the discriminator above is the only candidate left.

# The window is too narrow for a size term, measured 2026-10-02

The size scaling that followed -- `fracture::SIZE_CAP`, a material's strength
scaled by its body's volume after Dwyer -- had to be **normalised** because of
this window, and that is the first thing the window has cost rather than
merely threatened.

The plan asked for `cbrt(min(voxels, SIZE_CAP))`, running 1 -> 3 with the
calibration point at one voxel. Every strength in every table was measured
against bodies at or past the cap -- the fixture cubes are 4x4x4, 64 voxels --
so that term does not leave those numbers alone, it triples them. Two gates
then fail, and **no value of `SIZE_CAP` above 1 saves them**:

| gate | blow | struck body | largest factor it tolerates |
|---|---|---|---|
| `a_slow_crush_breaks_what_it_presses` | 365,906 | static world, saturated at `SIZE_CAP` | 1.046 |
| `a_collision_breaks_the_same_things_whichever_body_is_listed_first` | 405,054 | 8-voxel glass pebble | 1.157 |
| `the_same_collision_breaks_at_any_tick_rate`, at 60 | 1,047,814 | 64-voxel glass cube | 2.994 |

The first two are 4.5% and 15.7% of headroom over 350,000. Both re-measured
2026-10-02 after review; the pebble's blow was first recorded as 375,223 and
7.2%, which is wrong — the contact carries 405,054, and the raw rule's factor
on eight voxels is `cbrt(8) = 2`, so the gate fails either way. A multiplicative
term spanning three needs three hundred per cent of it, and this window is
ten.

So `size_factor` is `cbrt(min(voxels, SIZE_CAP) / SIZE_CAP)`, running 1/3 -> 1:
the calibration point moves to the cap, which is where it was measured.
Nothing at or past the cap moves at all, and only small bodies get weaker --
which is the whole of what Dwyer's comment about the rule claims. The static
world still saturates at `SIZE_CAP`, so it still reads the toughest factor
there is, which is the point of saturating it.

**This is a symptom, not a second defect.** Either fix above widens the window
and lets the term run 1 -> 3 if that is ever wanted. Until then, a palette
number means what a cap-sized body of that material takes.

# What an impulse means for the player

`strength` is no longer a speed a material survives. **A bigger body breaks the
same material at a lower speed**, because the impulse it carries scales with
its mass. That is what an impulse threshold is, not a flaw in it, but every
number on a palette now has to be read against a body -- and, until the held
load is separated out, against a tick rate.
