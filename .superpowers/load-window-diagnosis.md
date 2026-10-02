# The fracture load window: where the numbers actually come from

Diagnosis only. No engine behaviour changed, no constant retuned. Branch
`feat/interleaved-stepping`, on top of `d01703a`. Every number below is
printed by an `#[ignore]`d diagnostic in `crates/bevox_physics/src/solver.rs`,
at `DT = 1/64`, with the unbreakable twin of glass (`MaterialId(7)`) wherever
fracture would otherwise hand impulse back and perturb the thing being
measured. The three breaking figures the defect is written in -- 331,306,
356,751, 365,906 -- reproduce exactly against the unbreakable fixture, so the
substitution costs nothing.

**What is measured** is what a diagnostic printed. **What is inferred** says so.

## The instrument

`solver::trace`, `#[cfg(test)]` only, inert until a test arms it. It records
every `solve` call on every contact -- substep, pass, `vn`, `k`, separation,
the bias term, and the accumulated impulse before and after -- plus the
tick's contact layout and each contact's `approach[at]`. It is handed values
`solve` had already computed for its own use; no arithmetic on any path the
game runs was touched, and the ordinary suite is 127 passed, 0 failed, as
before.

Diagnostics, all `#[ignore]`d:

| test | what it answers |
|---|---|
| `where_a_landing_peak_comes_from` | 1, 2 |
| `what_the_sweep_counts_do_to_the_landing_peak` | 2 |
| `what_stops_the_press` | 3 |
| `how_the_landing_and_the_held_load_scale` | 4 |
| `what_the_closing_speed_says_in_each_regime` | the recommendation |

## The identity everything else is read against

A contact's accumulated normal impulse is not reset inside a tick, but it is
**re-applied from scratch every substep**: `warm_start` pushes the stored
value and `solve` then drives the total to whatever cancels the normal
velocity. So the value a contact holds at the end of a substep is exactly the
momentum that contact delivered in that substep.

**Measured, and exactly:** for the four-high landing tick, the four floor
contacts' end-of-substep values sum over the four substeps to **1,805,350**,
and the momentum the bodies lost over that tick, `sum m dv + m g dt`, is
**1,805,350**. Agreement to the last printed digit. `peak[at]` is therefore
momentum in the units of momentum, and is directly comparable with `m dv`.

## 1. How far it falls, and whether the impulse is consistent

**Measured.** 4-voxel cube, mass 64,000 each.

| | three-high | four-high |
|---|---|---|
| free-fall bound on 0.2 voxels | 6.2642 v/s | 6.2642 v/s |
| arrival speeds, tick before the peak | -3.866, -4.283, -3.056 | -4.062, -4.476, -4.801, -3.997 |
| `m v` the whole stack carries, at the bound | 1,202,723 | 1,603,631 |
| momentum arriving (`sum m abs(v_down)`) | 717,137 | 1,109,537 |
| momentum the contacts removed that tick | 1,099,445 | 1,805,350 |
| `peak[at]` | **331,306** | **356,751** |
| peak as a fraction of the tick's whole exchange | 0.30x | 0.20x |
| floor contacts | 4 | 4 |

The peak lands on **tick 5**, which is where a 0.2-voxel fall arrives:
`sqrt(2 * 0.2 / 98.1)` is 0.0639 s, 4.09 ticks. Arrival is below the free-fall
bound because the speculative margin starts bleeding the closing speed a tick
early.

**There is no solver bug here.** The impulse is not merely within what the
arrival momentum allows, it is a *fifth* of it: 356,751 is one of four floor
contacts' shares of the 744,901 that the floor took in substep 0 alone, and
the tick's total exchange is 1,805,350, which conservation accounts for
exactly. A 0.2-voxel drop of 256,000 of mass can deliver this and much more.

## 2. Where the peak comes from

**Measured.** Four-high stack, peak tick 5, **substep 0**, the **relax
(unbiased) pass**, contact `at = 0`, body 0 against the world:

```
vn -2.8275   1/k 23862   sep -0.0135   bias 0.0000
before 289280   after 356751   bias-free 356751
```

- **The bias contributes 0.0% of the landing peak, measured.** Not a small
  share: zero. `solve` takes the `BIAS` branch only when `separation < 0`, and
  then uses `(separation + SLOP).min(0.0)`. The deepest separation anywhere in
  the landing is **-0.0135**, shallower than `SLOP = 0.02`, so the clamped term
  is identically zero for every contact of the landing. `MAX_PUSH` never comes
  near: the depth at which it would bind is -0.4106.
- **So two statements in the record are false.**
  `docs/concepts/fracture-load-window.md` and the comments on
  `resting_weight_breaks_nothing` and `a_four_high_glass_stack_stands` say
  "`peak[at]` counts the push-out velocity the bias adds" and name "stop
  counting the bias's push-out" as a candidate fix. The bias adds nothing to
  count. Candidate **(a) is dead on the measurement**, and the fix it proposes
  would change the landing peak by zero.
- **The peak is front-loaded, not spread.** Floor sum by substep, four-high:
  744,901 / 679,644 / 358,559 / 22,246. Substep 0 takes 41% of the tick.
- **The blow is one contact's share of a statically indeterminate split.** The
  four floor contacts of a cube landing flat are geometrically symmetric; a
  fair split of substep 0 would be 186,225 each, under `GLASS_STRENGTH`.
  Measured split: **356,751 / 158,882 / 113,937 / 115,332** -- 47.9% on
  whichever contact the sweep reaches first.

  Not an under-convergence artefact. `what_the_sweep_counts_do_to_the_landing_peak`,
  through `step_with` (shipped counts untouched):

  | balanced sweeps | peak | share of its substep's floor total | split |
  |---|---|---|---|
  | 1 | 356,751 | 47.9% | 356751 / 158882 / 113937 / 115332 |
  | 2 | 399,486 | 47.5% | 399486 / 184377 / 122499 / 134082 |
  | 4 | 459,965 | 45.0% | 459965 / 228977 / 181228 / 151945 |
  | 8 | 580,848 | 45.4% | 580848 / 289365 / 220616 / 188765 |
  | 16 | 622,623 | 44.8% | 622623 / 322437 / 241720 / 202682 |

  More sweeps make the peak **worse**, not better -- they kill more of the
  arriving velocity inside substep 0 instead of spreading it over four -- and
  the concentration is stable at 45-48%. A redundant four-point normal contact
  has no unique impulse distribution, and sequential impulses pick an uneven
  one and keep it.

## 3. Why the crush ceiling is low

**Measured, and it is not penetration equilibrium.** `what_stops_the_press`,
unbreakable floor, 400 ticks of creep:

| tick | blow peak | grab carried | % of clamp | deepest sep | peak bias |
|---|---|---|---|---|---|
| 40 | 293,826 | 766,406 | **100.0%** | -0.0060 | 0.000 |
| 200 | 322,513 | 766,406 | **100.0%** | -0.0097 | 0.000 |
| 399 | 363,271 | 766,406 | **100.0%** | -0.0117 | 0.000 |

**The grab is saturated at its clamp from tick 40 onward.** The press is not
held below its limit by anything; it is *at* its limit, and what the clamp
buys is then divided among the contacts. At equilibrium, last substep, every
contact against the world:

```
at 0  sep -0.0117  after        1  ( 0.0% of the total,  0.0% of the clamp)
at 1  sep -0.0114  after  363231  (45.9% of the total, 47.4% of the clamp)
at 2  sep -0.0114  after  283028  (35.8% of the total, 36.9% of the clamp)
at 3  sep -0.0111  after  144664  (18.3% of the total, 18.9% of the clamp)
contacts' total 790924   grab accumulated 766406   clamp 766406
```

`790,924 - 766,406 = 24,518`, and `m g h` for this body is **24,525**: the
contacts carry the grab's whole clamped pull plus the body's own weight, to
seven parts in 790,000.

So the ceiling is **`GRAB_MAX_FORCE * dt / SUBSTEPS`, times the share the
heaviest of the contacts takes**: `766,406 * 0.477 = 365,6xx` against the
measured 365,906. Nothing else is involved. The concept page's surviving
"penetration-equilibrium reading, for 64 Hz only" is wrong, and the page's own
unexplained mass table falls out of this too -- a 2-voxel cube has fewer
contacts to divide by and reaches 588,636 (77% of the clamp), an 8-voxel cube
has more and reaches 343,399 (45%). The crush load is **the clamp divided by a
contact count**, which is why it is near-flat in mass and exactly proportional
to `dt`.

## 4. Scaling

**Measured.** `how_the_landing_and_the_held_load_scale`, 1200 ticks each.

| n | landing peak | at tick | held load | landing/held | `m g h n / 4` | late min | late max | late speed | sleeping |
|---|---|---|---|---|---|---|---|---|---|
| 1 | 126,129 | 4 | 10,231 | 12.3 | 6,131 | 0 | 0 | 0.0000 | 1 |
| 2 | 246,227 | 5 | 14,177 | 17.4 | 12,263 | 0 | 0 | 0.0000 | 2 |
| 3 | 331,306 | 5 | 18,798 | 17.6 | 18,394 | 0 | 0 | 0.0000 | 3 |
| 4 | **356,751** | 5 | 39,308 | 9.1 | 24,525 | 0 | 0 | 0.0000 | 4 |
| 6 | **391,195** | 5 | 118,320 | 3.3 | 36,788 | 59,686 | 133,272 | 2.0251 | 0 |
| 8 | **414,166** | 5 | 176,647 | 2.3 | 49,050 | 86,716 | 217,383 | 4.1189 | 0 |

- **The landing is strongly sub-linear**: 3.28x the peak for 8x the mass,
  about `n^0.55`. It saturates because a taller stack arrives staggered -- the
  cubes separate in flight and the momentum reaches the floor over several
  ticks instead of one.
- **The held load is roughly linear** where it means anything. For n <= 4 the
  stack sleeps, and the last load before sleep sits within 1.0-1.7x of
  `m g h n / 4`, the fair share of weight at four corner contacts; n = 3 is
  1.02x. The 1.6-1.7x at n = 1 and 4 is the same contact-share concentration
  as everywhere else.
- **n = 6 and n = 8 never settle at all** -- late speed 2.03 and 4.12 v/s,
  late load swinging 59,686-133,272 and 86,716-217,383, nothing asleep. Their
  "held load" is **not weight** and must not be read as the linear term. This
  is a stacking-stability defect, not a fracture one, and it is a second
  thing the fixture ladder shows.
- **The window is already empty, measured.** The n = 6 landing, 391,195, and
  the n = 8 landing, 414,166, are both **above the 365,906 ceiling a saturated
  grab can reach**. There is no value of `GLASS_STRENGTH` that lets a six-high
  glass stack settle and still lets a player crush glass. So a single constant
  cannot serve both, and no retune, however clever, recovers one.

## Recommendation: (c), with `approach[at]` as the discriminator

**(c), and nothing else is left standing.** The regimes must be told apart,
and the discriminator should be `approach[at]`, the closing speed at
detection, which is already computed per contact and already unused by
fracture.

The one measurement that most supports it -- `what_the_closing_speed_says_in_each_regime`
and the substep sums above:

| regime | total normal impulse, one substep, all floor contacts | heaviest contact's blow | its `approach[at]` |
|---|---|---|---|
| four-high stack landing | **744,901** | 356,751 | **-5.3639 v/s** |
| the same stack settled | -- | 39,308 | **+0.0003 v/s** |
| saturated grab pressing | **790,924** | 365,906 | **-0.0003 v/s** |

**The two loads are 1.06x apart as impulses and 1.8 x 10^4 apart as closing
speeds.** That single row decides the whole question:

- It kills every accounting fix, including the one I would otherwise have
  proposed. Summing a contact patch instead of taking its maximum changes the
  landing to 744,901 and the press to 790,924 -- still 1.06x apart, no better
  than the 1.03x the maxima already are. The two events genuinely deliver the
  same momentum per substep. **No way of counting the impulse can separate
  them.**
- It kills (a) outright: the bias is 0.0% of the landing peak.
- It leaves (b) half-right and insufficient. The diagnosis in (b) is correct
  and worth recording -- the ceiling is the grab's clamp times a contact
  share, measured, not an equilibrium -- but raising the clamp does not fix
  fracture. The floor rises with stack height and the landing at n = 8 is
  already 414,166; chasing it with a bigger clamp buys a window that the next
  taller stack closes again.
- Only the closing speed separates them, and it separates them by four orders
  of magnitude.

### What I would build

1. **Keep the impulse as the magnitude.** It is rate-independent for a
   collision, measured in the existing sweep (slam `m dv` flat to 1.3% over
   64-512 Hz), and that half of the rule is sound.
2. **Switch on `approach[at]`.** Above a closing speed in the region of
   0.05-1 v/s -- four orders of margin either side of the measured
   -5.36 / -0.0003 -- the contact is an *impact* and the impulse threshold
   applies as it does today. At or below it, the contact is a *sustained load*
   and what it carries is a force wearing an impulse's units, so threshold
   `peak[at] * SUBSTEPS / dt` against a separate, rate-independent limit. The
   existing sweep already showed the held and crush loads are proportional to
   `dt`; dividing by `h` is what makes that constant mean the same thing at
   every rate, and it is what finally lets the crush gate survive 128 Hz.
3. **Then re-derive the two constants independently**, which is only possible
   once they are two: the impact limit above the n = 8 landing, the sustained
   limit between a stack's weight and what a grab can press with as a force.
4. **Correct the fixture ladder's reading.** The n >= 6 "held load" numbers are
   a stack that never settles, and the n <= 4 ones carry the 1.6x contact-share
   concentration. Any calibration taken from them inherits both.

### What I would also record, and what the candidate list missed

**(e) The blow is one contact's share of a statically indeterminate split, and
the share is a function of how many voxels touch.** Measured: 47.9% of its
substep for a 4-voxel cube's four-point landing, stable at 44.8-47.9% from 1
to 16 balanced sweeps, 45.9% for the press, and -- from the concept page's own
mass table read through the clamp -- 77% for a 2-voxel cube against 45% for an
8-voxel one. So a palette strength is being compared against an arbitrary
fraction of a load, and the fraction moves by 1.7x with body size alone.

This is **not** the cause of the defect: the regimes are 1.06x apart however
the impulse is counted, and (c) is still the fix. It is an accuracy bound on
whatever threshold (c) ends up with, it explains three loose numbers in the
record at once, and nothing in the suite would notice it changing.

**(f) The six- and eight-high stacks do not settle.** Separate defect, found
in passing, evidence in the table above.

## Constraints honoured

- No engine behaviour changed. `solver::trace` is `#[cfg(test)]`, inert until
  armed, and is handed values `solve` already computed.
- No constant retuned: no strength, `BIAS`, `SLOP`, `MAX_PUSH`, `BASE_MARGIN`,
  `SUBSTEPS`, or iteration count. The sweep-count measurement goes through
  `step_with`, which exists for it.
- All five diagnostics `#[ignore]`d. `cargo test -p bevox_physics --lib`:
  **127 passed, 0 failed, 13 ignored**, unchanged from `d01703a` except for the
  five new ignored tests.
- `a_four_high_glass_stack_stands` left ignored and still failing by design.
