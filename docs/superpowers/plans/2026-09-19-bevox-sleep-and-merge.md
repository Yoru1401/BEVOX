# Sleeping and Merging Debris Implementation Plan

> Executed inline on 2026-09-19 and rebuilt on 2026-09-24 after the work was deleted by accident. This is the record of what was built.

**Goal:** Bodies at rest stop costing physics time, and a body that came out of the terrain and settled out of view goes back into it and frees its slot, as in Dwyer's devlog #13.

**Architecture:**
- **Sleeping** is state on `Body`, managed by `physics::sleep`, which `step` calls: the wake pass at the start of a tick, the settle pass at the end. Sleepers take no part in anything between.
- **Merging** is `physics::merge`: it writes a body's voxels into the world at their snapped places, through the new `Contree::fill_voxels`.
- The app decides **when** to merge, in `merge_settled`.

**Spec:** `docs/superpowers/specs/2026-09-15-bevox-rigid-bodies-design.md`, milestone 9.

## Decisions (Flori)

- **2026-09-19:** merge only when asleep and out of view, as Dwyer does, so the snap to the grid is never seen; a rotated body snaps; a jointed body never merges.
- **2026-09-24:** **only a body that came out of the terrain may merge.** Detachment marks what it frees, a split inherits the mark, and anything spawned outright — the `F` drop, a scene's own bodies — keeps its slot however long it sleeps.

## What was built

| Piece | Where |
|---|---|
| `asleep`, `still_for`, `from_terrain` | `body.rs` |
| `SLEEP_SPEED` 0.05, `SLEEP_AFTER` 0.5 s, `MERGE_AFTER` 3 s | `physics/mod.rs` |
| `wake`, `settle`, `wake_near` | `physics/sleep.rs` |
| `merge` | `physics/merge.rs` |
| `fill_voxels` | `edit.rs` |
| the mark on what it frees | `physics/detach.rs`, `physics/sculpt.rs` |
| `merge_settled`, `merge_system`, the wake after every edit | `bevox/src/main.rs` |

## Facts that are not obvious from the code

- **Waking runs to a fixed point before anything is detected,** so no contact or joint is ever solved against a sleeper, and a sleeper never stands in as a static body.
- **Bodies sleep in groups**, as Box2D's islands do: everything touching or jointed together sleeps in the same tick, once every member has been still. Waking restarts a body's count, and the grouping is what makes that safe: one at a time, two resting neighbours take turns waking each other forever.
- **A body a motor drives never sleeps**, or a servo that reached its target would ignore the next one.
- **Edits wake from the app,** through `sleep::wake_near` over the edit's box widened by a voxel. Two core tests that edit the world directly call it too, and say why.
- **A merge only adds geometry,** so the distance field is lowered over the merged body's sphere, as painting lowers it; stale, it would over-estimate and rays would skip the new voxels.

## Measurements

- **A tick with sixteen bodies resting on the floor** (release, GTX 1650 machine, medians of 1000 ticks, three runs): **asleep 0.0039 ms**, awake 0.9299 / 0.9463 / 0.9358 ms. Sleeping costs about 240 times less.
- **The workspace:** 350 tests pass, 10 ignored, no warnings.

## Gates, each proven by a deliberate break

| Gate | Break that fails it |
|---|---|
| A body at rest sleeps, stays put bit for bit, and wakes without gathered speed | sleepers take gravity |
| A stack sleeps without creeping | — |
| A body dropped on a sleeping stack wakes all of it in the same tick | wake does not propagate |
| Neighbours that settle at different times sleep together | sleep one body at a time |
| A grab, and a joint partner, wake a sleeper | — |
| `wake_near` over an erased support drops the sleeper | no wake after an edit |
| Moving bodies, sliding or spinning in place, never sleep | settle ignores spin |
| An aligned body merges voxel for voxel; a turned one lands under its voxel centres | round instead of floor |
| A merge never overwrites the world and stays inside it | overwrite the world |
| A settled body out of view merges, frees its slot, and leaves the field safe | merge in view; field not lowered |
| No merge while in view, jointed, grabbed, too recent, awake, or never terrain | merge ignores where a body came from |
| What detachment frees, and what splits off it, is marked as terrain | detach forgets; a split piece forgets |

## What changed during execution

1. **Two flaws in the first design, found by the gates.**
   - Waking without restarting the count let a touched stack re-sleep in the same tick with its velocity zeroed.
   - A servo that reached its target fell asleep and then ignored a new target and a knock.
   - Both fixed by group sleeping, a restarted count, and never sleeping a driven body.
2. **A claim in the module comment was removed as untrue.** It said a sinking body had been measured; the measurement it referred to (21.96) was a correct resting height. Group sleeping is kept for the reasoned ping-pong failure, not a measured sink.
3. **A gate that proved nothing was rewritten.** The stack-waking test accepted bodies waking on later ticks; it now requires the same tick.
4. **Two blind breaks, fixed by moving the scene.**
   - The merged body had to sleep in open air: on the floor the distance field already reads zero, so a stale field could not over-estimate.
   - The group-sleep gate needed two bodies that settle at different times; a stack settles all at once.
5. **An unrelated solver weakness, found and flagged rather than fixed:** a four-high stack struck from above rocks for about 80 seconds and walks 0.4 voxels before it is still. Sleeping made it visible. Raised as its own task.
6. **Rebuilt on 2026-09-24** after the work was deleted, with Flori's new rule added: only bodies that came out of the terrain merge.
