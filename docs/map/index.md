# The engine on one page

A pure-Rust sparse-voxel ray marcher with voxel rigid bodies. Everything is
marched on the GPU from one data structure; there is no triangle mesh anywhere.

Start here, then take the page for whatever you are changing.

* [bevox_core](core.md) — the voxel data structure and every algorithm over it.
* [bevox_physics](physics.md) — rigid bodies, contacts, joints, fracture.
  Its tunables are on [their own page](physics-constants.md).
* [bevox_render](render.md) — the Bevy plugin, the shader and the frame.

# How the crates sit

```text
              bevox_core          the data: the tree, the arena, materials,
                 ^   ^            and Body itself. No Bevy, no GPU, no window.
                /     \
               /       \
      bevox_render   bevox_physics     neither depends on the other, and a
               ^       ^               gate on the manifest keeps it that way
                \     /
                 \   /
                 bevox             the binary: window, input, brush, scenes
```

**Why `Body` lives in the core and not in physics.** The renderer marches bodies
and must never need the simulation to do it. So what a body *is* — its
transform, mass properties, features, warm-start impulses — is in `bevox_core`,
and everything that *computes* any of that is in `bevox_physics`.

**Why the renderer is the replaceable one.** Bevy's rendering internals change
between releases. Keeping every algorithm in a crate that knows nothing about
Bevy means an upgrade breaks plumbing, not the engine, and means the algorithms
are testable with no GPU and no window.

# The two loops

| | Runs | Does |
|---|---|---|
| **The tick** | `FixedUpdate`, 64 Hz | One physics step: detect, solve, integrate, fracture, sleep. Never depends on the frame rate. |
| **The frame** | `Update`, as fast as it goes | Input, the brush, merging settled debris, then staging whatever changed and dispatching the march. |

Bevy runs `FixedUpdate` before `Update`, which is what lets a tick remove a body
and have the frame rebuild the packed buffers in the same iteration.

# The numbers everything else sits inside

| | | |
|---|---|---|
| Volume extent | up to 4096³ | The root volume. Dense that would be 69 GB; sparse a scene is tens of megabytes, because nothing walks empty space. |
| GPU voxel budget | 512 MB | Checked at upload. Exceeding it is an error, never an allocation that might fail. |
| Bodies marched | 16 | Per ray the shader does `1 + N` marches, so this is a measured cap, not an aspiration — and it moves with resolution. |
| Frame target | 16.7 ms | At 1280x720 on a GTX 1650. The worst case is over it by design; [the frame budget](../concepts/frame-budget.md) says why. |
| Tick rate | 64 Hz | 4 substeps each, so `h = dt/4`. Speeds are per second, so changing the rate does not change what anything can do. |

# To change something, start here

| You want to change | Go to |
|---|---|
| How a voxel looks, or anything the shader does | [render.md](render.md), then `march.wgsl` — **but read [the codegen cliff](../concepts/gpu-codegen-cliff.md) first** |
| How bodies fall, collide, settle or break | [physics.md](physics.md) |
| What a material does | [core.md](core.md), and `bevox_core::material` |
| How the volume is stored, built or edited | [core.md](core.md) |
| What a key does, or what a scene contains | `crates/bevox/src/main.rs` and `scenes.rs` |
| How many bodies can be drawn at once | [the body cap](../concepts/body-cap.md) |

# Two rules that outrank convenience

**Every gate is proven by a deliberate break.** A test that has never failed is
not known to test anything. Break the thing a test guards, watch it fail,
restore, and record both — [the rule, and the six blind gates it
caught](../concepts/deliberate-breaks.md).

**Every performance claim is interleaved A/B/A in one session**, with the drift
reported beside the difference. Cross-run drift on this hardware is routinely
larger than the effect under test, so a number from an earlier invocation is not
evidence.

# The rest of the bundle

These pages are the map. The territory is in
[the specs, plans and concepts](../index.md): a **spec** for what a subsystem is
meant to be, a **plan** for what one piece of work cost and what it measured, and
a **concept** for a rule, a number or a post-mortem worth carrying forward.

A map page says what is true now. A plan says what it took to get there, and is
never rewritten to match.
