---
type: Implementation Plan
title: 'Detachment Over Tree Nodes'
description: 'The detachment search walks the tree''s uniform nodes instead of single voxels, as Dwyer''s devlog #12 does, so a cut into a large volume costs a few steps rather than thousands.'
tags: [physics, detachment, performance]
generated: { by: claude-opus-5/claude-code, at: 2026-09-24T00:00:00Z }
---

# Detachment Over Tree Nodes Implementation Plan

**Goal:** The detachment search walks the tree's uniform nodes instead of single voxels, as Dwyer's devlog #12 does, so a cut into a large volume costs a few steps rather than thousands.

**Decision (Flori, 2026-09-24):** speed first. The pieces that detach must not change at all, and the 20,000-voxel budget stays. Whether to raise it is a separate decision, made on the measurement this produces.

**Spec:** `docs/superpowers/specs/2026-09-15-bevox-rigid-bodies-design.md`, Detachment.

## Constraints

- `bevox_core` only; no new dependencies.
- Physics stays off master; commit on `feat/rigid-body-physics`, push only when asked.
- Same pieces, voxel for voxel, as the voxel walk gives.
- Every gate proven by a deliberate break; the timing claim measured A/B in one run.

## Design

A walk step is a **cell**: a maximal uniform-solid node, or a single voxel where the tree is subdivided to leaf level. Two cells are joined when their boxes meet face to face, which is the same 6-connectivity the voxel walk uses.

- `cell_at(tree, p) -> Option<Cell>`: descend from the root; stop at a uniform solid node and return its box, or at a leaf voxel. `None` when empty. This is `Contree::get`'s descent, keeping the box it stopped at.
- **Neighbours:** for each of a cell's six faces, scan the voxel columns just outside it, and for each one take `cell_at`. After each hit, skip the width of the cell found, so the scan costs one lookup per distinct neighbour, not one per voxel of the face.
- **Grounded:** a cell whose box touches `y == 0`. Same rule, same early exit.
- **Budget:** a cell's volume counts as its voxel count, so "bigger than the budget stays in the world" keeps its meaning.
- `components`, used inside a body, stays voxel-based: a body is small, and the spec says so.

## Tasks

1. **The cell walk.** Add `cell_at` and the neighbour scan; rewrite `loose_pieces`'s walk over cells. Keep the voxel walk as a test-only reference.
2. **Gates.** The existing property test (`the_search_agrees_with_labelling_every_piece_in_full`, 300 random scenes against an independent reference) must pass unchanged, plus a new test that the cell walk and the old voxel walk return identical pieces on those scenes. Breaks: a cell's neighbours scanned only at its corner; grounded tested on the cell's origin rather than its whole box.
3. **Measure.** Extend `a_detach_is_timed` to run both walks on the same scenes in one invocation, medians, and record them.
4. **Record.** Spec's Detachment section, plan Measurements, memory.

## Measurements

`a_detach_is_timed`, release, the cell walk interleaved with the voxel walk it replaced, three rounds in one run:

| Scene | Cell walk | Voxel walk |
|---|---|---|
| A 2,560-voxel column cut free | 0.232 / 0.234 / 0.234 ms | 1.029 / 1.076 / 1.122 ms |
| A hole dug in solid ground, nothing freed | 0.260 / 0.268 / 0.269 ms | 0.223 / 0.235 / 0.274 ms |

- **Freeing a piece: about 4.5 times faster.**
- **The grounded case is unchanged**, and may be a hair slower: the early exit already ended it after a few steps, so there was nothing to win. That is the common case for a cut into terrain.
- The workspace: 350 tests pass, 11 ignored, no warnings.

### 2026-09-28: the grounded case had something to win after all

The reading above was wrong about *why* the grounded case did not improve. The
rewrite had silently dropped the walk's face order. `NEIGHBOURS` put down last
so the stack popped it first and the walk dived for the floor; the cell walk
scanned axis by axis instead, which pushes `+z` last and dives sideways. Since
a grounded walk stops the moment it touches `y == 0`, the order *is* the early
exit's speed.

Restored, and the visited map's hasher changed with it (see below), measured
interleaved A/B/A in one invocation of `a_detach_is_timed`, medians of nine:

| | Dive for the floor | Scan by axis |
|---|---|---|
| Hole dug in solid ground, nothing freed | **0.136 / 0.138 / 0.139 ms** | 0.184 / 0.188 / 0.215 ms |
| A 2,560-voxel column cut free | 0.194 / 0.195 / 0.215 ms | 0.194 / 0.197 / 0.196 ms |

- **The grounded case: about 27% faster**, with a spread of 0.003 ms against a
  difference of 0.05. Real.
- **The freed case is unchanged**, as it should be: nothing in it is grounded,
  so no order finds a floor sooner.

**The visited map's hasher.** Dwyer keeps the visited set as a dense bitmap
(devlog 12) and names constant-time membership as what makes walking nodes worth
doing. A dense array is not available at this extent -- the tree reaches 4096,
so one entry per voxel coordinate is 6.9e10 of them, and a walk may roam
anywhere within `BUDGET` of the cut -- so the map stays and the hashing was made
cheap instead. The keys are three integers this code computed itself; SipHash's
resistance to adversarial keys buys nothing here.

| | `CellHasher` | SipHash |
|---|---|---|
| Column freed | **0.195 / 0.196 / 0.195 ms** | 0.233 / 0.236 / 0.233 ms |
| Hole in ground | **0.139 / 0.212 / 0.138 ms** | 0.188 / 0.188 / 0.189 ms |

About 17% on the freed case with a spread of 0.001, and about 26% on the
grounded one. Together with the face order, a cut into terrain went from the
0.260-0.269 ms recorded above to **0.138**.

Both changes are pinned by the existing property test, which now compares three
walks on all 600 scenes: the cell walk, the voxel walk it replaced, and the cell
walk with the old face order. Breaks: the two face orders must agree on every
scene, and they do -- the order is a speed, never a result.

## What changed during execution

1. **A real bug, caught by the property test once its scenes mixed cell sizes.** The face scan skipped ahead by the narrowest neighbour found in a column, but a column with an empty gap can have a different neighbour one step along, which was then never visited. Skipping is now only done for a column with no hole in it.
2. **The property test's scenes were the weak point, twice.**
   - As written they were 38% random noise, where every cell is a single voxel, so the whole point of the change went untested. Half the cases are now blocks the tree keeps as uniform nodes.
   - Blocks alone were still not enough: with everything aligned, a neighbour always covers the corner column, and the break that scanned only the corner passed. Loose voxels are now scattered among the blocks.
3. **The property test also pins the two walks against each other**, voxel for voxel, not only against the full-labelling reference.
4. **A non-vacuity check was fixed** rather than deleted: it demanded a uniform node at the origin, which scattering voxels can remove; it now asks that some cell in the scene is bigger than a voxel.
5. **The budget was left at 20,000 voxels**, as decided. Whether to raise it is now a decision with numbers behind it.
