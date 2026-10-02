---
type: Design Spec
title: 'Fracture in two regimes'
description: "No impulse threshold can separate a stack landing from a slow crush -- they are 1.06x apart in impulse and 18,000x apart in closing speed. Switch on the closing speed and threshold a force for a held contact."
tags: [physics, fracture, dwyer, design]
generated: { by: claude-opus-5/claude-code, at: 2026-10-02T00:00:00Z }
sources:
  - id: window
    resource: /concepts/fracture-load-window.md
    title: The fracture load window
  - id: devlogs
    resource: /reference/dwyer-devlogs.md
    title: "Douglas Dwyer's voxel engine, devlog by devlog"
---

# Fracture in two regimes — design

**Date:** 2026-10-02
**Status:** awaiting review

# The measurement this rests on

One substep's normal impulse, summed over every floor contact:

| | impulse | closing speed |
|---|---|---|
| a four-cube stack **landing** | 744,901 | **−5.3639 v/s** |
| a mouse grab **crushing**, saturated | 790,924 | **−0.0003 v/s** |

**1.06x apart in impulse. 18,000x apart in closing speed.**

That is the whole argument. The usable strength window is 331,306 → 365,906, ten
per cent wide, and a four-high stack lands outside it at 356,751 — so a glass
stack destroys itself on arrival. **No way of counting the impulse can separate
these two events**, because they deliver the same momentum. The closing speed
separates them by four orders of magnitude.

Three things the diagnosis ruled out, each measured rather than argued:

- **Not a solver bug.** The tick-5 exchange is 1,805,350 and `sum m dv + m g dt`
  is 1,805,350 — agreement to the last digit.
- **Not the push-out bias.** It contributes **0.0%**: the landing never
  penetrates deeper than 0.0135 and `SLOP` is 0.02, so the term is identically
  zero.
- **Not under-convergence.** Balanced sweeps 1 → 16 make the peak *worse*
  (356,751 → 622,623) at a constant 45–48% share. That is static indeterminacy,
  not a solver that has not finished.

And the ceiling is now explained: the grab sits at **100.0% of its clamp** from
tick 40 to 399, and the clamp is `GRAB_MAX_FORCE * dt / SUBSTEPS` = 766,406. The
heaviest of four contacts takes 47.4% of it. **The ceiling is a clamp divided by
a contact count**, which is why it is flat in mass.

Scaling closes the case: the landing transient is **sub-linear** (`~n^0.55`;
126,129 → 414,166 from one cube to eight) while held load is linear. The n=6 and
n=8 landings, 391,195 and 414,166, are **above the crush ceiling entirely**. No
single constant can serve both, at any value.

# What this is, and is not, a departure from

Dwyer's devlog 28 argues the threshold **must be an impulse, not a force**:

> A collision resolves within one tick, so the same 4 N·s at 10 ms a tick reads
> as 400 N and at 5 ms as 800 N: the force depends on the frame rate, the impulse
> does not. He notes the irony — real materials break on maximum *force*,
> following a stress-strain curve, but he does not simulate deformation, so force
> is not available to him.

Read the scope: ***"a collision resolves within one tick."*** His argument is
about **impacts**, and it is right about them. He never considers a sustained
contact because nothing in his engine presses indefinitely — his mouse grab
(devlog 30) arrives two devlogs after his fracture (devlog 28).

For a held contact the argument inverts exactly: the impulse is force × dt and
the force is not. **So this applies his own stated preference in the regime where
it is available**, rather than extending his conclusion past the case he argued
it for. It is not a departure.

# The design

Keep the impulse as the magnitude. Switch on `approach[at]`, the closing speed at
detection — already computed per contact, and currently unused by fracture since
it stopped being the blow.

| Regime | Test | Blow | Threshold |
|---|---|---|---|
| **Impact** | `-approach[at] >= IMPACT_SPEED` | `peak[at]`, the accumulated normal impulse | `Material::strength`, as today |
| **Held** | otherwise | `peak[at] * SUBSTEPS / dt`, a force | `Material::crush`, new |

`IMPACT_SPEED` sits in the 0.05–1 v/s region, where the diagnosis found nothing
at all: the two events are 5.36 and 0.0003, so any value in four orders of
magnitude separates them. **It is not a tuned constant** and the spec says so —
pick 0.1 and gate that moving it across the whole region changes no outcome.

`Material` gains `crush: f32`, a force. `UNBREAKABLE` remains the sentinel for
both.

# What it fixes, and what it does not

**Fixes.** The window disappears — the two regimes stop competing for one number,
so the landing is judged as an impact against the strength it already has, and a
press is judged as a force. The four-cube landing stops breaking. The crush
reaches stone and brick instead of ice alone, because a force threshold is not
confined to a clamp-divided-by-contact-count ceiling. And the rate-dependence
goes with it: `peak * SUBSTEPS / dt` is flat in `dt` where `peak` was not, so
`a_slow_crush_breaks_what_it_presses` stops depending on a 64 Hz tick.

**Does not fix, and must not be claimed to.**

- **The n=6 and n=8 stacks never settle** — late speeds 2.03 and 4.12 v/s, held
  load swinging 59k–133k and 87k–217k. That is a stacking defect, it is
  independent of fracture, and it is the next thing after this.
- **The blow is one contact's share of a statically indeterminate split**, and
  the share moves **1.7x with body size alone** (77% of the clamp for a 2-voxel
  cube, 45% for an 8-voxel one). That is an accuracy bound on any threshold, this
  one included. It is not a cause and this change does not improve it.

# Gates

| Gate | What it proves |
|---|---|
| A four-high glass stack lands and stands | The defect this exists for. `a_four_high_glass_stack_stands` stops being `#[ignore]`d and failing |
| An eight-high stack lands without breaking | The landing transient is judged as an impact at every height, not only inside a window |
| `a_slow_crush_breaks_what_it_presses` at `DT` **and** `DT/2` **and** `DT/4` | The held branch is rate-independent, which the impulse branch could never be |
| A slow crush breaks **stone**, not only ice | The ceiling is gone, which is the user-visible half |
| `resting_weight_breaks_nothing` | Must still pass. A resting stack presses with its weight and nothing more |
| `the_same_collision_breaks_at_any_tick_rate` | Must still pass. The impact branch is unchanged |
| Moving `IMPACT_SPEED` across 0.05 → 1 changes no outcome | The discriminator is not a tuned constant |

Breaks required: force the impact branch always (the crush gate must fail); force
the held branch always (the landing gate must fail); set `IMPACT_SPEED` to 100
(everything becomes a held contact) and to 0 (everything an impact) — each must
fail a different gate.

# Cost

Every material needs a second number, and both need calibrating. That is the real
price and it is palette-wide: `fixtures::materials()`, `scenes::palette()`, and
`DEFAULT_STRENGTH`'s companion. `the_demo_palette_outlasts_a_resting_stack` gains
a sibling for the crush column.

# Rejected

- **Suppressing the landing transient.** Dead on the bias measurement: the bias
  contributes 0.0% and the impulse matches the momentum exactly. There is nothing
  artefactual to suppress.
- **Raising the crush ceiling at the grab.** A correct diagnosis of the ceiling
  and insufficient: the landing floor outruns any clamp by n=8.
- **One quantity, better calibrated.** Ruled out by measurement, not taste. The
  two events are 1.06x apart.
