# Update Log

## 2026-09-29
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
