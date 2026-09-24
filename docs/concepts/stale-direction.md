---
type: Rule
title: 'No safe stale direction'
description: 'A stale distance field only under-estimates and costs speed; stale fullness is wrong on screen, so every path that edits the world recounts it.'
tags: [ambient-occlusion, distance-field, editing, correctness]
generated: { by: claude-opus-5/claude-code, at: 2026-09-24T00:00:00Z }
sources:
  - id: ao
    resource: /superpowers/plans/2026-09-24-bevox-ambient-occlusion.md
    title: Ambient Occlusion Implementation Plan
---

# The asymmetry

The two coarse grids do not fail alike.

| Grid | Left stale after adding solid | Left stale after removing solid |
|---|---|---|
| distance field | **rays skip the new geometry** | under-estimates: costs speed only |
| fullness | lights a crease that is now there | darkens air that is now open |

So an erase may skip the field entirely — that is why `apply_brush` lowers the
field only when painting — but **nothing** may skip fullness.

# Where it went wrong

The recount lived in `apply_brush` alone, which covers the brush's own
neighbourhood. Two other paths change the world:

- **Detachment** frees pieces that can reach far outside the brush — the bridge
  that falls when its one support is erased. Its old cubes stayed counted full:
  a dark crease in open air.
- **Merging** writes a body into the world nowhere near a brush, and contributed
  no occlusion at all: a merged pile lit as if it were not there.

Both were permanent until the next scene reload, and no test failed.

# The shape of the fix

One method, `VoxelScene::world_changed(lo, hi)`, which every editing path calls.
The dirty ranges widen rather than replace, so two edits between two uploads
cannot lose the first. Three gates hold it: the grid must equal a fresh build
after a detachment and after a merge, and every packed word two edits changed
must be staged.

**The general rule:** when a derived structure has no safe direction to be wrong
in, its update belongs at the one place the source changes, not at each caller
that remembers to ask. See [deliberate breaks](deliberate-breaks.md) — all three
gates were proven by removing the calls.
