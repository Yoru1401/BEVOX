# Update Log

## 2026-10-03
* **Creation**: [the contree, and where the name comes from](reference/contree.md).
  **Flori found it**: the term is on screen in devlog 22, and it is
  `tetrahexaconta` — Greek for 64 — plus `tree`. The derivation matters more than
  the etymology: **a contree is an octree with two layers squashed into one**, so
  branching goes 8 to 8² and a node divides 4x4x4 instead of 2x2x2. Four per axis
  is therefore not a parameter anyone chose — it is 2², and the 64-bit mask is 8².
  The register argument this bundle has been giving as the *reason* for 64 is
  really its *consequence*, and the right statement is that **two layers is the
  largest squash that still fits a machine word**: three would be 512 children and
  a 512-bit mask, and the structure loses the only trick it is built on.
* **Creation**: the one outside measurement of the structure, from the same
  source — squashing cuts encode time from 1700-1800 ms to **1048-1288**, about
  1.4x, for **+0.196% in bits**. Worth keeping because it contradicts the obvious
  intuition that a node covering 64 cells rather than 8 must waste bits on absent
  children in sparse regions. Nothing here measures that and nothing here is
  likely to.

## 2026-10-02
* **Update**: a slow crush now breaks **stone** — the user-visible half of the
  two-regime change, and the thing a crush could not do when one constant had to
  serve both regimes. `CRUSH_STONE` is 76,640,000, the geometric middle of a
  four-high stone stack's held force (62,164,992) and what a grab presses into
  stone with (94,487,080): **1.23x each way, from a gap only 1.52x wide.**
* **Blocked**: **brick and ice cannot be calibrated and stay `UNBREAKABLE`.**
  Measured per material, four-high held force against press ceiling: stone
  62,164,992 / 94,487,080 (1.52x), brick **142,913,184 / 95,169,968 — inverted**,
  ice 43,003,908 / 116,325,592 (2.71x). Brick's floor sits *above* its ceiling, so
  any crush reaching brick destroys a brick stack standing still. And **none of
  the three stacks settle** — 0 of 4 asleep in every row — so these are not held
  loads but swinging piles. **The stacking defect that keeps
  `an_eight_high_glass_stack_stands` failing reaches the demo palette at four
  cubes**, and a threshold calibrated against a diverging pile bakes the
  divergence in. Stone's number is taken only because its gap survives the noise.

* **Update**: the impact strengths rose, the ceiling having left them —
  [The impulse threshold is a force threshold for everything it was built for](concepts/fracture-load-window.md).
  `GLASS_STRENGTH` sat at **350,000** only to stay under the **365,906** a
  saturated grab presses with; the regime split sends a press to
  `Material::crush` as a force, so that ceiling no longer applies to `strength`
  at all. It is now **500,000**, the geometric middle of the **414,166** an
  eight-high stack lands with (the worst of a landing series that is sub-linear
  at about `n^0.55`, so clearing eight clears far taller) and the **607,581**
  the two-body collision gate allows — an eight-voxel pebble carrying 405,054
  against a `size_factor(8)` of two thirds. The window went from **1.10x to
  1.47x**. `a_four_high_glass_stack_stands` is **un-`#[ignore]`d and passing**
  after failing for the whole life of the one-number threshold. The demo palette
  moved with it, its ice being pinned to that constant: 1,312,500 / 729,167 /
  350,000 → **1,875,000 / 1,041,667 / 500,000**, and the palette gate's floor
  rose from the three-cube 331,306 to 414,166.
* **Creation**: `default_strength_breaks_at_terminal_speed_and_holds_below_it`,
  because scaling every strength by the same 10/7 nearly shipped terrain nothing
  could break. `DEFAULT_STRENGTH` is calibrated against what a **thrown** body
  delivers, not a landing — it is already seven times clear of those — and the
  measured peak curve is 1,047,814 / 2,476,651 / 2,071,706 / 3,429,210 /
  6,096,372 at arrival speeds 60 / 120 / 150 / 200 / 256. At 10/7 it becomes
  4,000,000, **above the 3,429,210 a terminal arrival lands**, so default terrain
  would survive every fall there is and break only at `MAX_SPEED`. **The whole
  suite stayed green on that value.** It stays at 2,800,000 and the new gate
  pins the window 2,476,651 to 3,429,210. The curve is also **not monotonic** —
  150 lands less than 120, the substep the contact is first seen in moving with
  the speed — so the old doc comment's "arrives at about 150" was never true.
* **Update**: raising a threshold shrinks `fracture::over`, and `over` sizes the
  crack pattern. Every blow now reads **0.7x** the `over` it did. The visible
  pattern barely moves: `planes_of` was already at its clamp floor of 2 for
  every blow in the suite before and after, and `reach_of` loses one voxel on
  exactly one of them (the eight-voxel pebble, 3 → 2). What did shrink is the
  **top** of the range — the hardest blow the engine can land on a cap-sized
  glass body is **963,237**, capped by `MAX_SPEED` and not by any speed a test
  asks for, and that reads **1.926** over instead of 2.75. So
  `the_hand_back_scales_with_how_far_past_strength_the_blow_went` lost the
  bracket it was built on: `1 - 1/2.5 == 0.6` pinned the fixed `REBOUND` the
  rule replaced, 2.5 is no longer reachable, and the bracket is now 1.5, which
  rules out a fixed third instead. **The one place this change made a gate
  weaker.** A heavier hammer would buy 2.5 back, since the cap is on speed and
  the blow is an impulse.
* **Update**: the fracture load window, taken apart by measurement —
  [The impulse threshold is a force threshold for everything it was built for](concepts/fracture-load-window.md).
  Three of the page's claims were wrong and are now corrected. The bias
  contributes **0.0%** of the landing peak, not the 17x inflation the page
  blamed it for: the landing never penetrates deeper than 0.0135 and `SLOP` is
  0.02, so the clamped term is identically zero. The slow crush is **not** held
  below its limit by a penetration equilibrium; the grab sits at **100% of its
  impulse clamp from tick 40**, and the 48% is how that clamp divides among the
  four contacts. And the window is **empty, not narrow**: a six-high landing is
  391,195 and an eight-high 414,166, both above the 365,906 a saturated grab
  reaches, so past five cubes no strength satisfies both gates. What survives is
  the closing speed as the discriminator — -5.3639 v/s landing against -0.0003
  pressing, four orders of magnitude, while the two loads are **1.06x** apart as
  impulses, which is why no way of counting the impulse can separate them. Five
  `#[ignore]`d diagnostics and a `#[cfg(test)]` `solver::trace` are the
  instrument; no engine behaviour and no constant moved.
* **Creation**: `the_physics_runs_at_the_rate_the_tests_assume`, which pins the
  fact that keeps BEVOX's rate-dependence latent. `physics_system` sits in
  `FixedUpdate`, and inside that schedule Bevy rebinds the generic clock to
  `Time<Fixed>` — so the `time.delta_secs()` the solver receives is **64 Hz, not
  the frame delta**. Every material strength is calibrated at that rate, and the
  fracture blow for a held contact is force x dt (365,906 at 64 Hz, 93,512 at
  512). Move the system to `Update` or call `Time::<Fixed>::from_hz`, and the
  calibration stops describing the app **while every test keeps passing**,
  because the tests pass their own `DT` and never ask what the app uses.
  `the_same_collision_breaks_at_any_tick_rate` guards the latent property; this
  guards the thing that keeps it latent.
* **Fix**: and the gate's first draft was broken, caught by its own deliberate
  break. It searched `include_str!("main.rs")` for
  `add_systems(FixedUpdate, physics_system)` — a string that **appears literally
  in the assertion itself**, so the file always contained it and the gate passed
  with the system moved to `Update`. It now searches only the source before
  `#[cfg(test)]`. A self-referential source check is a gate that cannot fail, and
  nothing but the break would have shown it.

* **Creation**: the stress scene, drift F4's first half —
  `solver::tests::forty_nine_cubes_tumble_without_escaping_diverging_or_sinking`.
  49 glass 2x2x2 cubes in a closed hollow box with two-voxel walls, gravity
  swept through a full turn over 800 ticks at constant magnitude, because the
  static world is a `Contree` and cannot be a rotating drum. It gates stacking
  stability under a continuously changing load direction, **not** Dwyer's
  tumbler — a real drum drags its contents through wall friction and this does
  not. **Passed on arrival**: worst speed 45.56 v/s against a bound of 99.94
  (1.5x a 66.63 v/s free fall across the cavity diagonal), worst penetration
  0.0259 against `SLOP + BASE_MARGIN` = 0.1200, one fracture at 244,950 against
  an 8-voxel threshold of 233,333, nothing ever asleep. `#[ignore]`d at 99
  seconds in a debug build —
  [how far BEVOX has drifted from Dwyer](reference/dwyer-drift.md).
* **Update**: what that scene revealed is **how little it loads the solver**.
  Four deliberate breaks, one fails: `BASE_MARGIN = 0` (0.0268 against a bound
  that falls to 0.0200 with it). `MAX_PUSH` 20 → 2000 changes the run **bit for
  bit not at all** — the bias only ever pushes at about 0.08 v/s here, so the
  clamp is inert. `RELAXATION_ITERATIONS = 0` passes at 42.09 v/s and 0.0200;
  `BIAS = 0` passes at 0.1025, 85% of the bound and 4x baseline. So the speed
  assertion is a regression tripwire, not a sensitive instrument, and a
  calibration checked only against a uniform pile is not checked hard. The
  earlier claim that F4's scene would say what makes an extreme mass ratio
  collapse is withdrawn — it does not —
  [how far BEVOX has drifted from Dwyer](reference/dwyer-drift.md).
* **Update**: the hand-back has a gate of its own.
  `the_hand_back_scales_with_how_far_past_strength_the_blow_went` strikes the
  same `SIZE_CAP` glass bar twice with the same unbreakable hammer, once
  1.069 times over strength and once 2.687, and asks what share of its
  approach speed the hammer keeps: 2% and 55%. The two speeds bracket 2.5,
  where `1 - 1/over` equals the retired `REBOUND = 0.6`, so a fixed fraction
  is too generous on one strike and too mean on the other. **Restoring
  `0.6 * peak[at]` fails it at 52.2% on the gentler strike and fails nothing
  else in the suite**, which is why it had to exist —
  `breaking_something_does_not_stop_you` only ever asked that *something* come
  back.
* **Correction**: the eight-voxel pebble in
  `a_collision_breaks_the_same_things_whichever_body_is_listed_first` carries
  **405,054**, 15.7% over `GLASS_STRENGTH`, not the 375,223 and 7.2% recorded
  earlier today. Re-measured; the crush figure (365,906, 4.5%) reproduced
  exactly. The conclusion is untouched — the raw rule's factor on eight voxels
  is `cbrt(8) = 2`, well past either bound.
* **Correction**: substituting the plan's raw `cbrt(min(voxels, SIZE_CAP))` at
  `ff269c5` leaves the suite at **117 passed / 7 failed**, not the 120 / 4
  recorded in the task report. The four calibration gates that report named do
  all fail; the three it missed are `a_small_body_breaks_before_a_large_one`
  and the two `fracture` unit tests, all three written by that same task
  against the normalised arithmetic and so unable to pass under the raw form.
  The count was measured before those gates existed and reported against the
  commit that has them. **The deviation itself still holds** — it rests on the
  four gates, and they fail.
* **Correction**: 331,306 is a **landing transient**, not a load a resting
  stack carries. `docs/reference/dwyer-drift.md` and
  `bevox_core::material::DEFAULT_STRENGTH` both said the latter; the load the
  same stack then holds is 18,798, and the rest of the tree already recorded
  it that way.
* **Update**: a material's strength is scaled by the size of the body that holds
  it, after Dwyer's `sqrt(min(area, 7))` — `cbrt(min(voxels, SIZE_CAP))` with
  `SIZE_CAP = 27`, so a chip of glass gives way under a load a sheet of it
  holds. `Body::voxel_count` caches the count in `mass::recompute`, which
  already holds the voxel slice; `break_what_gave_way` runs per contact per tick
  and must not allocate there. The static world has no body and saturates at
  `SIZE_CAP`, because reading the missing count as one voxel would make terrain
  the most fragile thing in the scene.
  **The factor had to be normalised to run 1/3 → 1 rather than 1 → 3**, and that
  was measured rather than chosen: every strength in every table was calibrated
  against bodies at or past the cap, the crush gate has 4.5% of headroom over
  `GLASS_STRENGTH` and the list-order gate's eight-voxel pebble 15.7%, and no
  `SIZE_CAP` above 1 passes both. The ten-per-cent fracture load window is the
  real constraint — [the load window](concepts/fracture-load-window.md) has the
  table — and it is an open defect, not this change's to fix.
  [Fracture on a size-scaled impulse](superpowers/plans/2026-10-02-bevox-fracture-on-impulse.md).
* **Update**: a breaking contact hands back **exactly the excess** it carried
  past the weaker side's threshold, clamped to what it actually carried, and
  `fracture::REBOUND = 0.6` is retired. The fixed share was a guess at the thing
  the excess is. The clamp matters because the excess is measured over the
  *weaker* side while the impulse belongs to the pair, so an unclamped hand-back
  could add energy the scene never had;
  `the_hand_back_never_exceeds_what_the_contact_carried` is the gate and it fails
  by 500 units unclamped.
* **Creation**: `the_weaker_side_breaks_and_the_stronger_does_not`, which
  **passed on arrival** and was kept. One impulse tested against two thresholds
  may come out two ways; Flori read that as a bug and it is the design. The gate
  is what makes it explicable, and a gate that passes the day it is written is
  still a record of the claim.
* **Fix**: the window's size was read once, at startup, and never again —
  `create_march_target` is a `Startup` system, so the storage texture, the size
  `dispatch_march` turns into a workgroup count, and the sprite's `custom_size`
  all kept whatever the window was when the app opened. **Flori found it on
  Hyprland**, where launching from a terminal tiles the window to half the screen
  and going fullscreen then leaves the march drawn in the old rectangle with the
  rest of the window black. `resize_march_target` moves all three together, and
  `a_resized_window_takes_the_march_target_with_it` asserts all three, because
  fixing one alone is a different bug rather than a fix: the texture alone leaves
  the borders, the sprite alone stretches a stale low-resolution image, and the
  recorded size alone dispatches over pixels nothing wrote. Proven by two
  separate breaks. The beam buffer needed nothing — `BEAM_CAPACITY` is sized for
  7680x4320.

* **Fix**: the demo palette was never rescaled when `strength` became an impulse,
  and **Flori found it by playing**: a body landing anywhere set off a chain
  reaction that took the whole map apart. `fixtures::materials()` was
  recalibrated with the units change; `scenes::palette()` was not, because
  **nothing in the suite reads it** — so stone, brick and ice stayed at 45, 25
  and 12, speeds in voxels a second, against a resting impulse of **331,306**.
  Everything broke on contact with anything. Rescaled by one factor,
  `350_000 / 12`, which puts the weakest exactly on the measured floor and keeps
  the three in the proportion they always had.
* **Creation**: `the_demo_palette_outlasts_a_resting_stack`, the gate whose
  absence shipped that. It asserts every breakable material the player actually
  meets is stronger than the load a resting stack carries, and its deliberate
  break prints *"below the 331306 a resting stack already carries… Did the units
  change again?"* **The lesson is not the number.** The physics crate's fixtures
  and the app's palette are two material tables, only one of them is tested, and
  the tested one is not the one the game runs on.

## 2026-10-02
* **Fix**: two overclaims the review wave's own fix introduced, both in
  [the convergence page](concepts/solver-convergence-is-a-setting.md), one
  mirrored into `lib.rs`. The page had blamed the non-monotone wander
  (12.83 → 15.75 → 14.81 → 18.02) on the invalid 200-tick window, then said
  seven lines later that correcting the window left every figure unchanged —
  **which refutes the first claim**. The wander is real inside the valid window.
  And "the one place a convergence reading is not confounded" was false: the
  balanced rows raise **total work** as well as the ratio, so numerical damping
  is an unexcluded reason for a later breach. Time-to-collapse cannot separate
  "better conditioned" from "bleeding energy faster".
* **Update**: the sharpest comparison in the table, which the original write-up
  missed. `(2,2)` is four sweeps a substep and `(4,1)` is five — **roughly
  matched total work** — and they breach at **105 against 67**. Nearly the same
  solving, nearly twice the standing time, with only the bias:relax ratio
  between them. That argues the ratio better than "monotone in the count" did,
  and it argues against damping being the whole story, since the row doing *more*
  total work collapses sooner.

## 2026-10-01
* **Correction, 2026-10-02**: the negative result below was stated wider than
  its experiment could carry, and is **narrowed**. Four of the six variants
  raised `velocity_iterations` while holding `relaxation_iterations` at 1, so
  they vary the **bias:relax ratio** and not the biased count alone — the relax
  pass exists to remove the velocity the bias added. Three sentences are
  withdrawn in that form: "made a 240:1 load *less* stable", "no setting on this
  axis", "the question is closed". `(2,2)` was added and the measurement re-run:
  on the **balanced** axis the fixture stands **roughly twice and three times as
  long** (breach at tick 105 at (2,2) and 123 at (4,4), against 55 at (1,1)), so
  the balanced axis is **open**, pointing the other way. The stability scan also
  took its maximum over 200 unconditional ticks of a fixture that breaches at 55
  and a `step_with` that deletes fallen bodies — it now stops at the first breach
  and prints the peak and breach ticks. The headline survives: **the lever F7
  reached for — the biased count alone — did not deliver.**
* **Measurement, and a negative result**: the solver's two iteration counts are
  exposed, measured, and **staying at 1**, pinned by
  `the_solver_iteration_counts_are_one`. F7 called raising the velocity count
  "the standard lever for stacking stability"; `what_the_iteration_counts_cost`
  says it is **not that in this engine, on the axis the spec named** — a 240:1
  load's worst upward velocity goes **12.83 v/s at (1,1) to 18.02 v/s at (8,1)**
  for 2.6x the tick cost, and (8,1) breaches the floor on the same tick 55 that
  (1,1) does. The one monotonic gain is resting penetration, 0.0017 → 0.0003, now
  gated by `more_iterations_do_not_deepen_a_resting_contact`. The loops
  themselves are free: bit-identical at the default, proven by
  `a_tick_is_unchanged_by_the_default_tuning`. F7 is **resolved, not fixed**, and
  the claim about what raising the counts buys is withdrawn —
  [solver convergence is a setting, and the spec reached for the wrong one](concepts/solver-convergence-is-a-setting.md).
  Also recorded there, because it was found on the way: **zeroing
  `velocity_iterations` does not drop a body through the floor.** The unbiased
  relax pass calls the same `solve`/`solve_friction`/`solve_rolling`, so
  non-penetration survives; what breaks is eleven tests, mostly joints. The
  biased pass is not what holds a stack up.
* **Open defect, now characterised**: the rebuilt 240:1 fixture **collapses
  through the floor at tick 55** — peaking at 12.83 v/s *upward* at tick 11, then
  a voxel and a half into the slab and falling at **−22.08 v/s**. Reproducible
  from a clone by `the_240_to_1_load_collapses_through_the_floor` (`#[ignore]`d),
  which **passes today** and is written to be inverted the day the collapse is
  fixed. The 2026-09-28 record ("23 v/s upward by tick 33") **does not
  reproduce**: different magnitude, direction and timing, and at tick 33 the
  fixture is settling. This is independent of the iteration counts, nothing was
  adjusted to chase the recorded number, and **no cause is established** — two
  hypotheses are recorded and marked as hypotheses (`MAX_PUSH` being a
  mass-independent velocity clamp, `[Likely]`; contacts reused across four
  substeps against a one-voxel plate, `[Guessing]`), neither tested.
* **Update**: [the fracture and detection spec](superpowers/specs/2026-09-30-bevox-fracture-and-detection-design.md)
  revised, and **reordered**. F7 — the solver's iteration counts — was listed out
  of scope and is now **Part 1**, because `peak[at]`, the accumulated normal
  impulse the fracture threshold reads, is taken **inside** the two solve loops
  F7 turns into loops. A sequential-impulse solver converges its accumulated
  impulse upward, so a lower iteration count means a smaller impulse and a
  fracture that fires late. The count is therefore an input to every `strength`
  in the material table: calibrate first and change the count after, and the
  whole table is wrong. Both counts default to 1 so the first commit is
  bit-identical.
* **Update**: Part 3's provenance firmed up. His 2026-02-20 sneak peek calls the
  TGS engine's detection *"continuous collision detection"*, the same phrase
  devlog 11 earns from SAT projection gaps — so substepped speculative detection
  is what **he** calls CCD, and Part 3 implements his design rather than
  diverging from it.
* **Update**: every Dwyer video **description** swept, and the record checked a
  second time. All **thirty upload dates compared mechanically** against the
  channel metadata — **all thirty match** — and the descriptions **contradicted
  nothing**. Two independent checks have now failed to find an error in
  [the devlog record](reference/dwyer-devlogs.md), which is as much confidence as
  that file can earn without his source.
* **Creation**: [the four videos that are not devlogs](reference/dwyer-devlogs.md),
  all four missing from the record until now. One matters: the **2026-02-20
  sneak peek**, ten weeks before devlog 26, whose description dates **TGS,
  continuous collision detection and material-specific properties to before 26
  explained them** — so 25 → 26 is not the jump it reads as. **And he calls his
  detection continuous**, the same word devlog 11 earns from SAT projection gaps,
  which puts `rigid_pixels`'s substepped `Speculative` detector under his own
  umbrella term. The [E2 respec](superpowers/specs/2026-09-30-bevox-fracture-and-detection-design.md)
  is therefore implementing what he calls CCD, not departing from him. The ledger
  row is updated.
* **Update**: **the one resolution he states anywhere.** Devlog 22's description
  says its scenes were captured *"in real-time 1080p on my GTX 1660 TI"* —
  eleven months after the 7 ms of devlog 17 and with textures, foliage,
  transparency and volumetrics added since. So the standing comparison is
  **1920x1080 against this bench's 1280x720, 2.25x the rays**. That **widens**
  the gap rather than explaining it, and 1080p is the figure to beat —
  [nine steps a ray](concepts/nine-steps-a-ray.md).
* **Update**: three smaller facts the transcripts do not carry — his own name for
  his ambient occlusion, **VVAO, volumetric voxel ambient occlusion** (15); the
  three DDGI papers named, Majercik et al. twice and Rohacek (23); and his
  *"market research"* video reviewing the building systems of frozein's engine,
  **Teardown** and `_rey`'s, the only place Teardown is a subject rather than a
  benchmark.
* **Fix**: [John Lin](reference/john-lin.md) corrected from his video
  descriptions, which contradict what the page was written with. **He built three
  renderers in fifteen months**, and on 2020-06-16 abandoned *hardware* ray
  tracing — *"the engine is no longer using RTX"* — for a custom Vulkan Compute
  ray tracer, because a BLAS is too heavy to rebuild for a dynamic scene and his
  own *"extremely lightweight acceleration structure"* was a net win despite
  being slower in raw terms. The blog's BLAS/SBT sketch is therefore not "his
  renderer". Also: the claim that his argument carries no measurements was wrong
  — the descriptions carry fluid simulation under **8 ms on 4 threads**, a
  **10x** rendering speedup, **5** path-traced bounces, a **256K cubed** world,
  **y 0-4095**, and a player-built scene under **3 MB**.
* **Update**: and the reason to trust his argument at all. On **2020-07-12** he
  presented his engine's *"unified voxel framework"* — world, physics, ray
  tracing, lighting, procedural generation, sound tracing and collision detection
  over the same voxel data — as the achievement that made everything possible. On
  **2021-09-18** one format for every system is the disease. Fourteen months and
  one rewrite apart, which makes the blog post a **post-mortem on his own
  engine**, not a position. The earlier caution on that page — that an argument
  never shipped is weak evidence — had a false premise and is narrowed to the
  part that is still true: he never shipped the *fix*.
* **Update**: [the derived-view rule](concepts/derived-view-per-job.md) now names
  the bill for its own cheapest case. Normals are recomputed per sample and never
  stored, and a per-sample view cannot accumulate across frames — which is
  exactly why F3, per-voxel sun visibility, is blocked. Lin's third engine lists
  per-voxel material attributes as shipped, so that blocker is BEVOX's and not
  the pattern's. He also says, in passing, *"we don't rotate our voxels here,
  because we know better"* — BEVOX does, and that parenthesis is the strongest
  outside argument against it this project has.
* **Creation**: [John Lin's voxel engine, read from his blog](reference/john-lin.md) —
  a second voxel developer, **unrelated to Dwyer**, whose argument is that one
  format chosen for the renderer poisons every other system: sparse voxel octrees
  are *"acceptable (not even great)"* at storage and rendering and bad at
  collision, lighting, path finding and new attributes. His answer is a canonical
  minimum format plus conversion operators registered by name. **No source, no
  measurements, and the renderer post he promised was never written** — the
  project has been silent since 2021, so this is a design argument and is sourced
  nothing like the rest of this bundle.
* **Creation**: [one canonical tree, and a derived view for each job](concepts/derived-view-per-job.md) —
  the pattern BEVOX was already following, now named. **Six** representations are
  derived from the `Contree` — `Features`, `MassProperties`, `DistanceField`,
  `Fullness`, `GpuVolume`, and normals that are never stored at all — each shaped
  for one job and each declaring its own staleness. The rule: the next job gets a
  view, not a wider `Node`. It also explains why two grids over the same cells in
  the same struct have opposite dirty policies.
* **Fix**: the body cap read **0.265 ms per visible body** in both
  [the bundle index](index.md) and [the concepts index](concepts/index.md), while
  `body-cap.md` itself and `map/render.md` carry the 0.629 ms re-measured on
  2026-09-29. This is the sixth row of the
  [inference audit](concepts/an-inference-is-not-an-observation.md)'s own table
  surviving in the two files that audit did not re-read — found by a knowledge-graph
  pass, not by a gate, because `tests/map.rs` checks names and not numbers.

## 2026-09-30
* **Creation**: [Dwyer's rigid_pixels, read from source](reference/rigid-pixels.md) —
  his open-source 2D prototype settles three things the devlogs leave open. **He
  has no speed cap at all**: tunnelling is bounded by substepping *detection* per
  pair against a fixed 0.4 distance, so cost is linear in speed where BEVOX's
  margin-widened lookup is cubic. His fracture threshold is an **impulse scaled
  by object size**, tested **per side**, handing back **exactly the excess**. And
  his solver exposes `velocity_iterations` and `relaxation_iterations` as counts
  where BEVOX runs one of each — the standard lever for stacking stability, which
  is what broke on 2026-09-28.
* **Creation**: [fracture and detection, as he built them](superpowers/specs/2026-09-30-bevox-fracture-and-detection-design.md) —
  the respec. The closing-speed threshold goes back to an impulse: **the units
  were the problem and the physics was changed to fix them**, and it cost crush
  fracture, which is the symptom Flori found with the mouse grab.
* **Update**: devlog 17 checked line by line against the reference file — date,
  id, the 64 children, the brick-tree name, the 64-bit mask, eight-or-more reads
  per octree step, ten ray steps without a memory read, 7 ms, primary and shadow
  ray, 1660 Ti, compute-bound, and the 11,000 lines removed. **Every checkable
  claim holds.**
* **Update**: the static march measured rather than guessed at. **A primary ray
  takes nine steps**, so the marcher is not step-bound and Dwyer's four
  accelerations have already taken the walk to almost nothing — no traversal
  work can close the gap to his 7 ms castle. Workgroup size measured for the
  first time: 16x16 within drift, 32x32 a 2.45 ms regression, so 8 stays and the
  question is closed — [nine steps a ray](concepts/nine-steps-a-ray.md).

## 2026-09-29
* **Fix**: the engine stops asking for wgpu's defaults, which are the **WebGPU
  spec baseline** — eight storage buffers per stage where the GTX 1650 offers
  524,288, and 128 MB per binding where it offers 2,047. BEVOX excludes
  WebAssembly permanently, so it had been paying a browser's ceiling for nothing,
  and had written a rule telling future work to keep paying it.
  `pipeline::device_limits` asks for what the engine needs plus a margin. It also
  closes a latent bug: the 512 MB voxel budget was checked as a **sum** while one
  binding capped at 128 MB, so a large scene passed the check and would have died
  at buffer creation —
  [both coarse grids ride in one storage buffer](concepts/coarse-grids-share-one-buffer.md).
* **Fix**: the specs and plans swept for dead references — 89 backticked
  identifiers that appear nowhere in the code. Most are correctly historical;
  three were **gates a Breaks table names and the suite does not have**, all
  renamed on the way in, and one had changed its claim as well as its name. Each
  plan now maps planned names to shipped ones, and the Mouse Grab plan is marked
  superseded, which it has been since milestone 7 without saying so.
* **Fix**: an audit of the map pages against the code, prompted by a day of work
  planned on a false sentence. Six errors, five of them a day old and in the map
  pages: `camera` did not own the clip matrix, `fracture` runs every tick and not
  only on an edit, staging is main-world and not render-world, `edit` is more
  than the brush, the body cost was stale, and a body march does **not** walk the
  whole ray. All six were a true premise with an unchecked conclusion attached —
  [an inference is not an observation](concepts/an-inference-is-not-an-observation.md).
* **Update**: a visible body costs **0.629 ms, not 0.265** — the cap's figure
  predated shadows and AO by a day and a week. It splits evenly between the
  primary march and the shadow-ray caster tests, so the two renderer divergences
  are worth the same; and the static march alone is 13.23 ms of a 16.7 ms frame —
  [the body cap](concepts/body-cap.md).
* **Update**: the drift ledger rebuilt around **why** each difference happened and
  whether it was a good idea. Thirty-one differences, six causes, and only one of
  them is drift: six oversights, four worth fixing, four of those in the renderer.
  Everything else is a different product, a different renderer, a forced
  constraint, something not reached yet, or a place BEVOX is better —
  [the ledger](reference/dwyer-drift.md).
* **Creation**: [how far BEVOX has drifted from Dwyer](reference/dwyer-drift.md) —
  all thirty devlogs set against what the engine does. Ten deviations have a
  reason on record; **seven do not**, six of those in the renderer. The first is
  that his answer to the many-objects cap (devlog 2's interleaved stepping) was
  never taken, which is why `MAX_BODIES` is 16.

## 2026-09-28
* **Creation**: [the map](map/) — five one-page overviews, after Stone Librande's
  GDC talk on one-page designs: the engine, then a page per crate, plus the
  physics tunables. Each is capped at a page, and the half of each that is
  derivable from the source is checked against it by `crates/bevox/tests/map.rs`,
  so a page cannot quietly stop being true. The bundle's front door is now the
  map; the specs and plans behind it say how it came to be true and are never
  rewritten to match.
* **Fix**: a fracture's blow is the speed the two surfaces met at, not the
  contact impulse divided by the owning body's mass. The owner is whichever
  body the scene lists first, so the same collision read two ways -- and a
  resting stack could shatter under weight that never moved. Two gates, each
  proven by the alternative it rules out -
  [Fracture](superpowers/plans/2026-09-24-bevox-fracture.md).
* **Fix**: the detachment walk dives for the floor again, and its visited map
  stopped being hashed with SipHash. A cut into terrain went from 0.26 ms to
  0.138 -
  [Detachment Over Tree Nodes](superpowers/plans/2026-09-24-bevox-detachment-over-nodes.md).
* **Update**: the specs' Provenance section gained devlog 28, which fracture was
  built from but which it never listed, and the five places this engine departs
  from Dwyer that were not written down: the blow as a speed, rolling
  resistance, the ambient-occlusion blend done without a sampler, the visited
  map, and classification on every edit -
  [BEVOX Rigid Bodies](superpowers/specs/2026-09-15-bevox-rigid-bodies-design.md).
* **Update**: rolling resistance is recorded as a consequence of Dwyer's rounded
  corners and edges rather than an invention -- a lone voxel is entirely corner,
  so his shapes make it a sphere - and the two plans that deferred it with
  "nothing in the scene is a ball yet" say so -
  [Sliding friction cannot stop a roll](concepts/rolling-needs-its-own-resistance.md).
* **Update**: claims the code does not support are corrected or marked unbuilt --
  the contact lookup reach, the beam seed's neighbourhood, the output format, the
  composite stage, the render resolution, the platform, and the GPU's absent
  step-cap counter and error scopes -
  [BEVOX ray-marcher core](superpowers/specs/2026-09-13-bevox-raymarcher-core-design.md).

## 2026-09-24
* **Creation**: `bevox_physics`, a force and impulse API on `Body`, and a
  ceiling in voxels per second -
  [A Physics Crate, Forces, and a Tick-Free Ceiling](superpowers/plans/2026-09-24-bevox-physics-crate.md).
* **Update**: a falling body is limited by drag rather than a clamp, and
  contacts resist rolling so a lone voxel settles -
  [Sliding friction cannot stop a roll](concepts/rolling-needs-its-own-resistance.md).
* **Creation**: fracture, for bodies and terrain, after Dwyer's devlog 28 -
  [Fracture](superpowers/plans/2026-09-24-bevox-fracture.md).
* **Creation**: ten views on the function keys, and what they cost the lit
  frame (nothing measurable) - [Debug Views](superpowers/plans/2026-09-24-bevox-debug-views.md).
* **Creation**: a devlog-by-devlog record of Douglas Dwyer's engine, from all
  thirty transcripts — [Dwyer's devlogs](reference/dwyer-devlogs.md).
* **Creation**: eight [concepts](concepts/) — the rules, measured numbers,
  decisions and post-mortems that were buried in the plans.
* **Creation**: `docs/` became an OKF v0.2 knowledge bundle — every spec and plan
  carries frontmatter, and [the index](index.md) lists them.
* **Update**: ambient occlusion, after Dwyer's fullness grid, and the two paths
  that were leaving it stale — [Ambient Occlusion](superpowers/plans/2026-09-24-bevox-ambient-occlusion.md).
* **Update**: detachment walks the tree's uniform nodes, 4.5x faster on freeing —
  [Detachment Over Tree Nodes](superpowers/plans/2026-09-24-bevox-detachment-over-nodes.md).

## 2026-09-19
* **Creation**: bodies sleep, and debris that came out of the terrain merges back
  into it — [Sleeping and Merging Debris](superpowers/plans/2026-09-19-bevox-sleep-and-merge.md).

## 2026-09-18
* **Creation**: Dwyer's sixteen joint types, friction and motors, the mouse grab
  as a joint — [Dwyer's Joints](superpowers/plans/2026-09-18-bevox-physics-dwyer-joints.md).
* **Deprecation**: the first joints, made with a tool in the app, were replaced by
  those — [Joints](superpowers/plans/2026-09-18-bevox-physics-joints.md).
* **Creation**: bodies cast shadows on the world, each other and themselves —
  [Body Shadows](superpowers/plans/2026-09-18-bevox-body-shadows.md).
