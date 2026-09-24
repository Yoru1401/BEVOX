---
type: Rule
title: 'Every gate is proven by a deliberate break'
description: 'A test that has never failed is not known to test anything; six plans record gates that passed a break and had to be rebuilt.'
tags: [testing, process, correctness]
generated: { by: claude-opus-5/claude-code, at: 2026-09-24T00:00:00Z }
sources:
  - id: joints
    resource: /superpowers/plans/2026-09-18-bevox-physics-dwyer-joints.md
    title: "Dwyer's Joints Implementation Plan"
  - id: bodies
    resource: /superpowers/plans/2026-09-18-bevox-physics-body-vs-body.md
    title: Body Against Body Implementation Plan
  - id: sleep
    resource: /superpowers/plans/2026-09-19-bevox-sleep-and-merge.md
    title: Sleeping and Merging Debris Implementation Plan
---

# The rule

For every gate: break the thing it guards on purpose, run it, and require it to
fail. Restore, and record both the break and what it did. A gate that survives
its break is **blind** and is rebuilt or replaced — never patched over, and never
kept on the grounds that it passes.

# Why this is not paranoia here

Blind gates were found in six separate plans. A sample of what survived a break:

- **Energy gate, three breaks.** `no_joint_gains_energy` catches a tripled
  lambda and a flipped mass sign, and is blind to three others. Breaking the
  lever in the friction solve alone stays blind at 4.5e-5, because warm starting
  reapplies most of the impulse each substep at the correct lever. The break
  that fails the gate has to move both the warm start and the solve.
- **The joint scene, blind to a dead crank motor.** Gravity alone swings the
  linkage far enough to move the slider past the threshold. The gate now also
  demands the crank turn more than two full turns in ten seconds; with no motor
  it turned -2.96 rad.
- **The merge gate, blind to a field left un-lowered**, until the body was moved
  aloft — on the floor the field already reads zero, which hid it.
- **The face scan, blind to a corner-only break**, until loose voxels were
  scattered among the blocks.
- **The tunnelling test** in milestone 2, and two gates each in the first joints
  plan and the body-versus-body plan.

# What a blind gate teaches

In every case the fix was a stronger scene, not a stronger assertion: move the
body where the stale value is visible, add the geometry the shortcut skips, ask
for the thing only the feature under test can produce. **Where the fixture is
weak, no threshold is strong enough.**

Record blind breaks honestly in the plan's "What changed during execution". A
gate whose limits are written down is worth more than one assumed to be total.
