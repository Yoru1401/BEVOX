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

(Filled in during execution.)

## What changed during execution

(Filled in during execution.)
