# BEVOX Rigid Bodies — Design

Voxel chunks that detach from the static world, fall, tumble and come to rest,
rendered by the same ray marcher that draws everything else.

This is new scope. The milestone-8 spec deliberately excluded physics, so nothing
here is a deferred item being picked up. It is a second subsystem built on the
first.

> **Revised 2026-09-17.** Collision, integration, materials and the milestones
> were rebuilt to follow Douglas Dwyer's voxel physics engine, as shown in his
> devlogs, at Flori's request. The Provenance section at the end separates what
> his devlogs show from what this design fills in. The first version's
> surface-voxel queries and plain semi-implicit Euler are replaced.

## What this is for

Carve the supports out from under a wall, and the wall should fall over rather
than hang in the air. Bodies are always voxels: a piece of terrain that an edit
disconnects becomes a body, as in Dwyer's engine. Everything below serves that.

Two interactions come later but constrain the design now:

- **Picking up bodies.** A joint holds the clicked point at the cursor and keeps
  the body's orientation, and the body keeps simulating.
- **Editing bodies with the brush.** Painting grows a body, erasing shrinks it,
  and pieces that end up disconnected split into separate bodies.

## The one property that makes this tractable

**A body's transform is rigid: rotation and translation, no scale.**

The CPU marcher has taken `volume_to_world: Affine3A` since milestone 2 and been
handed `IDENTITY` at every call site. That hedge was placed for exactly this.
Marching a transformed volume means transforming the ray into the volume's
local frame, marching, and transforming the hit back.

Because the transform is rigid, **`t` is preserved exactly** between world and
local space: a distance along the ray means the same thing in both. That is what
lets the renderer compose the static world and any number of bodies by keeping
the nearest hit, with no depth buffer, no re-projection and no epsilon.

Allow scale and that property dies. `t` would scale per body, the comparison
would need a division, and every existing parity test's notion of distance would
stop being shared. Scale is therefore out permanently, not deferred.

## Rendering

The shader marches the static world, then each body in turn, and keeps the
nearest hit. Normals return through the rotation only.

Per ray this is `1 + N` marches, so **the body count is capped**, and the cap is a
measured number rather than an aspiration. Body culling (milestone 1b) made a
body cheap:

- **Off-screen bodies:** a CPU frustum cull drops them entirely.
- **On-screen bodies:** a per-pixel screen rectangle skips a body's read and
  transform outside its footprint.

`MAX_BODIES` is 16 at 1280x720 on a GTX 1650. The measurements are in
`docs/superpowers/plans/2026-09-17-bevox-body-culling.md`.

**Bodies cast shadows** (2026-09-18) on the world, on each other and on
themselves.
- A shadow ray the static world leaves clear tests every body, up to the cap,
  stopping at the first hit.
- Those bodies are a caster list, never the culled table and with no screen
  rectangle, because a body just off screen can shadow what is on it.
- Each caster carries its world bounding sphere, so a ray that passes nowhere
  near a body skips it before any transform.
- Bodies shade like terrain: with the implicit normal, which blends the
  directions a voxel is open to, probed in the body's own volume and rotated
  into the world. So a body's edges and corners shade apart from its faces
  rather than every face being one colour.
- Shadows fall in voxels on bodies, as on terrain. A body hit's shadow ray
  starts from the centre of the voxel that was hit, in the body's frame, 0.75
  along its face, so a whole voxel face is lit or shadowed together.
- Measured: +4.85 ms for sixteen bodies in view, for 22.6 ms in all, and free
  for bodies the rays miss. Flori chose to keep shadows on and the cap at
  sixteen. The plan
  `docs/superpowers/plans/2026-09-18-bevox-body-shadows.md` has the numbers.

Each body owns its own `Contree`, packed into the shared node and voxel buffers
behind a per-body base offset.

## Materials

`Material` gains physics columns as the milestones need them:

- **`density`** (milestone 2). A `u16`, so `Material` can still derive `Eq`. A
  voxel's mass is its material's density, in relative units: only ratios
  between bodies, and against a joint's force later, are observable.
- **`friction` and `restitution`** (milestone 3). Per material, so one body can
  slide or bounce differently depending on which of its voxels touches.

## Mass properties

Computed from voxel occupancy, weighted by density, by a function that can be
**re-run whenever the volume changes**, not only at creation:

- **Mass:** the sum of voxel densities.
- **Centre of mass:** the density-weighted mean of voxel centres. It becomes the
  point the body rotates about:
  `world_from_local = T(position) · R(orientation) · T(-com)`, where `position`
  is the centre of mass in world space.
- **Inertia tensor:** summed per voxel about the centre of mass, from the exact
  unit-cube inertia plus the parallel-axis term. It is inverted once and stored.

A body whose centre of mass is not its pivot tumbles wrongly in a way that looks
almost right, which is the worst kind of wrong.

**Recomputing after an edit keeps the body's voxels where they are in the
world.** If the centre of mass moves by `Δ` in local space, `position` moves by
`orientation · Δ`. A body left with no voxels is removed.

## Voxel classification

Each solid voxel of a body is labelled by counting the axes along which it has
solid neighbours on **both** sides:

| Axes | Label | Collision shape |
|---|---|---|
| 0 | corner | sphere of radius 0.5 |
| 1 | edge | cylinder of radius 0.5 along that axis, one voxel long |
| 2 | face | full slab on its exposed axis |
| 3 | interior | none: it can never be touched first |

The labels are computed with the mass properties and re-run on every edit.
Static-world voxels are classified on demand, from their neighbours in the
tree, and only inside a body's bounding box: the world is too large to label in
advance.

## Collision detection

Once per physics tick, for every pair that can touch:

1. **Broadphase.**
   - Take the world-space axis-aligned bounding box of the body's rotated
     occupied bounds, grown by the speculative margin.
   - Against the world, if one distance-field cell's promised free cube contains
     that box, the body touches nothing this tick.
   - Otherwise the box bounds which world voxels are considered.
   - Between bodies (milestone 3), the two boxes must overlap.
2. **Candidates.** Collisions between two voxel volumes happen only between a
   corner and any voxel, or between two edges. So the pairs tested are:
   - corners of A against voxels of B;
   - corners of B against voxels of A;
   - edges of A against edges of B.

   Faces never test against faces. The corners around a face cover that
   contact.
3. **Lookup.** A candidate voxel's centre is transformed into the other side's
   grid, and every voxel within `ceil(margin + 0.5)` of the voxel containing it
   is examined: 3x3x3 at rest, 5x5x5 at the speed cap. Dwyer's devlog says the
   nearest 8. That misses speculative contacts, and a corner resting exactly one
   voxel above a floor sits on its tie boundary, so the contact would come and
   go from tick to tick and warm starting would lose it.
4. **Pair test.** The two rounded shapes from the classification table are
   tested: sphere against sphere, plane or cylinder, and cylinder against
   cylinder. Each test gives:
   - **signed separation:** negative is penetration; positive up to the margin
     makes a speculative contact;
   - **normal:** not restricted to the grid axes;
   - **anchor point** on each body.
5. **Key.** Each contact is keyed by the pair of voxel coordinates, which is
   what makes warm starting possible.

Rounding corners and edges while keeping faces flat avoids both failure modes
Dwyer describes:

- **Axis-aligned boxes** are not rotation-invariant, which gives wrong normals
  and jitter.
- **Pure spheres** let bodies sink into each other's gaps and add false
  friction.

## Integration and solving

Temporal Gauss-Seidel (TGS), on Bevy's `FixedUpdate` at 64 Hz. One tick:

1. **Detect** contacts once (above).
2. **Warm start.** Apply each contact's accumulated impulse from the previous
   tick where its key matches, and drop the rest.
3. **Substeps.** Repeat `SUBSTEPS` times (4 to start) with `h = dt / SUBSTEPS`,
   **without re-detecting**:
   1. Apply external forces: `v += g·h`.
   2. Solve velocity constraints. For each contact, compute the normal impulse
      including the angular terms (`r × n` through the world inverse inertia),
      clamp the accumulated total to be non-negative, and apply it to linear
      velocity and angular momentum. Each substep recomputes a contact's current
      separation from how far its anchors have moved since detection.
   3. Integrate positions: `x += v·h`, and orientation from
      `ω = R·I⁻¹·Rᵀ·L`, renormalised.

**State.** A body stores linear velocity and **world-space angular momentum
`L`**, not angular velocity. With no torque, `L` is exactly constant, so the
conservation gate can demand exact conservation. Physics state lives in `Body`
itself, not in a parallel array, so splitting or removing a body cannot
desynchronise the two.

**No separate position-correction pass.** Pushing a resting body up after
gravity moves it down makes it jitter at float precision. Penetration is instead
removed by a soft bias inside the velocity constraint, above a small slop.

**Tunnelling.** No continuous collision detection. Speculative contacts give an
approaching body a constraint before it arrives, and a velocity cap keeps a
tick's travel within the margin. `SUBSTEPS`, the slop, the bias factor, the
margin and the cap are named constants, tuned against the resting gate.

**Units.** Voxels and seconds. `GRAVITY = 9.81 / VOXEL_METRES` voxels/s², where
`VOXEL_METRES` (0.1 to start) is a tuning knob for how heavy falls feel.

**Leaving the world.** Voxels outside the static tree's extent are empty to
detection. A body entirely below the world is removed.

**Edits to the world** need no special handling. Detection reads the live tree
every tick, so erasing the ground under a resting body drops it on the next
tick.

## Detachment and body edits

An edit that removes voxels can disconnect part of a structure, in the static
world or in a body. Detachment finds the disconnected parts with a depth-first
search using **6-connectivity** (face neighbours only), starting from the solid
voxels around the edit.

- **Grounded** means reaching the world's floor (`y == 0`). A walk stops the
  moment it does, and dives downward first, so a cut into the ground costs a
  few dozen steps rather than a full search.
- **Too big** means outgrowing a budget. A walk that does gives up and calls the
  piece grounded, so a cut into a mountainside never walks the mountain.
- A later walk that runs into an earlier walk's voxels is on that earlier
  walk's piece, and takes its verdict.

Each part that neither reaches the floor nor outgrows the budget becomes its own
body. Its voxels are removed from the source through the ordinary edit path, so
the render world uploads a delta, and its mass properties are computed.

The search walks the tree's **uniform nodes**, as Dwyer's devlog #12 does: a
step crosses a whole solid region rather than one voxel. A step is a cell —
a uniform solid node, or a single voxel where the tree is subdivided to the
leaf — and two cells are joined when their boxes meet face to face, which is
the same 6-connectivity walking voxels gave. Each face is scanned column by
column just outside the box, skipping the width of each neighbour found, so a
face costs one lookup per neighbour rather than one per voxel; a column with a
hole in it cannot skip, because past the hole the next column may hold a
neighbour this one never saw.

Measured (release, interleaved against the voxel walk in one run): freeing a
2,560-voxel column takes 0.23 ms against 1.03-1.12, about 4.5 times faster. A
cut into solid ground, where the early exit already ends the walk in a few
steps, is unchanged at about 0.26 ms. The budget still counts voxels, so what
detaches is exactly what detached before.

A piece is never taken when the renderer has no room to draw it: past the body
cap, the largest pieces go first and the rest stay in the world, still drawn.

Body edits go through the same path:

- **Painting onto a body** grows it and recomputes it.
- **Erasing from a body** recomputes it and runs the same split, so a body cut in
  two becomes two bodies.

Detachment is coupled to edit throughput. A single paint already stages about
2.2 MB of distance-field data at extent 4096, so the search is bounded to the
affected region.

## Joints and the mouse grab

> **Revised 2026-09-18, milestone 7.** Joints were rebuilt after Dwyer's devlog
> #30: sixteen types, friction, motors, and the mouse grab as a joint. This
> replaces milestone 5's spring grab and milestone 6's two joint types and its
> J tool.

**One equation for every joint, Dwyer's.** A joint is an error function `C` of
the two bodies' transforms, satisfied when `C = 0`, and `J` is its Jacobian.
Asking that `J·V₂ = 0`, with the force along `J` as `F = Jᵀλ`, Newton's law
gives

```
λ = −(J M⁻¹ Jᵀ)⁻¹ (J·V + bias)
```

Each part of a joint is solved as one block of its one to three rows, so a
point's `J M⁻¹ Jᵀ` is the 3×3 `1/m − [r]× I⁻¹ [r]×` summed over both bodies.
The bias, `BIAS` of `C` per substep, is this design's; Dwyer shows none. It is
left out of the relax pass, as it is for contacts.

**Sixteen types**, each one linear part and one angular part. `pa` and `pb` are
the two anchors in the world, `wa` and `wb` the two axes.

| Part | `C` | Rows |
|---|---|---|
| Linear free | none | 0 |
| Point | `pa − pb` | 3 |
| Line | `pa − pb` along two normals to the line | 2 |
| Distance(`L`) | `abs(pa − pb) − L ≤ 0`: a rope, which pulls and never pushes | 1, one-sided |
| Angular free | none | 0 |
| Locked | the relative turn away from `rest` | 3 |
| Axis | `wa × wb` across the axis (Dwyer: `a₁·b₂`, `a₁·c₂`) | 2 |
| Cone(`θ`) | `angle(wa, wb) − θ ≤ 0` | 1, one-sided |

A one-sided row behaves like a contact: while it is slack, the bodies may close
exactly the gap. A joint has two anchors, so a rope can reach from a beam to a
body, and `rest`, the bodies' relative orientation when it was made. Anchors
are in volume coordinates, as the grab's always were, or in the world on the
world's side.

**Friction** acts on the motion a joint leaves free, asking for a relative
velocity of zero. Its accumulated impulse is clamped as a vector, as contact
friction is, to `force·h` for linear motion and `torque·h` for angular.

- Linear: Free and Distance on three axes at the pivot, Line along the line,
  Point on nothing.
- Angular: Free and Cone on three axes, Axis about the axis, Locked on nothing.

Free and Free with friction is Box2D's friction joint: a capped drag between
two bodies.

**Motors** drive the one free motion of an Axis or a Line part, and creating one
on any other part panics.

- `Speed(v)` asks for the relative velocity `v`.
- `Target(x)` asks for `(x − current)·BIAS/h`: a servo. On a hinge the angle
  error is wrapped to ±π, so it takes the short way round.
- The impulse is clamped to `±max·h`, so a motor stalls against a load beyond
  `max`.
- On its own motion a motor replaces friction, and `Speed(0)` is a brake.

**Order in each pass**, after Box2D v3, so the hard constraints have the last
word: motors, then friction, then the one-sided rows, then the equality blocks.
Motors and friction run in the relax pass too, because they are velocity goals
and not drift corrections. Joints are solved before contacts.

**Jointed bodies do not collide with each other.** A joint to the world does
not stop its body colliding with the world, so a door is hung a voxel's gap from
its post.

**A joint follows its pivot.** After any edit, a joint moves to whichever piece
holds its first anchor's voxel at the same place in the world. It is removed if
no piece does, or if either of its bodies is gone.

**The mouse grab is a joint**, as in Dwyer's engine: Point and Locked, from the
clicked point to a target on the cursor's ray, holding the orientation the body
had when it was grabbed.

- A body grabbed by its corner keeps its pose rather than hanging from the
  corner, which is what makes stacking easy.
- Its force and torque are capped absolutely, by `GRAB_MAX_FORCE` and
  `GRAB_MAX_TORQUE`, and not scaled by mass. A heavy body sags and drags, and
  the grab cannot force a body through the terrain, since contacts are solved
  after it.
- Letting go removes the joint, and the body keeps its momentum, so a flick
  throws it.
- It is left out of the relax pass. Its target moves, and the bias is how that
  motion reaches the body; relaxing it would stop the body dead every substep,
  and letting go would throw nothing.
- `G` toggles grab mode, and the wheel moves the held point nearer or farther.

This replaces milestone 5's grab, a damped spring at the clicked point that was
Flori's design after Joe Binns' *Get Me Out*.

**Joints are made by scenes, not by a tool.** `1` loads the demo scene and `2`
the joint scene, and pressing either again resets it. The joint scene holds
twelve bodies, under the cap of sixteen: a door, a chain, a rope, a cone, a
shelf, a crank and slider, a servo arm and a friction block. Every linear part
and every angular part appears in it at least once.

## Testing

The project's existing discipline applies unchanged:

- **CPU reference first.** Mass properties, classification, pair tests and the
  solver are pure functions, tested without a GPU or a window.
- **Bit-identity.** A scene with zero bodies must render bit-identically to the
  same scene before this work existed.
- **Conservation as a gate.** A body with no gravity and no contacts must
  conserve linear and angular momentum over thousands of ticks.
- **A body at rest must stay at rest.** Drop a body on a floor. Its height after
  ten thousand ticks must equal its height after one thousand, and at rest the
  normal impulse per tick must match its weight. Jitter hides from short tests.
- **Every performance claim is interleaved A/B/A in one session**, with drift
  reported. Any change to the ray-march shader re-runs the static-world bench,
  because a driver codegen cliff once cost 48% with identical pixels.

One lesson this project paid for many times over: **a test whose scene or camera
cannot reach the code it names is worse than no test**, because it reads as
coverage. Every gate states what it would fail against, and the ones guarding
correctness are checked by deliberately breaking the code they guard.

## Milestones

| # | Deliverable | What it proves |
|---|---|---|
| 1 | A transformed body volume rendered, composed with the static world | The renderer can do this at all |
| 1b | Body culling; cap raised to 16 | Bodies are cheap enough to have several |
| 2 | Material density, recomputable mass properties, voxel classification, rounded-voxel contacts against the world, TGS solver with angular response and warm starting, gravity | A body lands, tumbles and rests on real terrain |
| 3 | Friction and restitution per material; body against body | Bodies slide, bounce and pile |
| 4 | Detachment from the world, and brush editing of bodies with splitting | The thing this was for |
| 5 | Mouse grab: a damped spring from the cursor to the clicked point (a joint since 7) | Bodies can be picked up |
| 6 | Ball and hinge joints, made with a tool in the app (the tool removed in 7) | Hinges, chains, hanging things |
| 7 | Dwyer's joints: sixteen types, friction and motors; the grab as a joint; a joint scene | Doors, ropes and machines; bodies held steady |
| 8 | Body shadows, from every body including those off screen | Bodies sit in the scene rather than on it |
| 9 | Sleeping, and merging settled terrain debris back into the world | Debris stops costing anything, and gives its slot back |

**Sleeping and merging** (milestone 9). A group of bodies touching or jointed
together falls asleep once every one of them has been still for half a second,
and takes no part in a tick until something that could move it wakes it: an
awake body, a joint, the grab, or an edit through `sleep::wake_near`. Measured:
sixteen resting bodies cost 0.0039 ms a tick asleep against 0.93 ms awake.

A sleeper that **came out of the terrain** — detachment cut it loose, or it
split off something that did — merges back into the world once it has slept
three seconds out of view, and frees its slot. Its voxels snap to the world
grid, which is why it must be out of view; the field is lowered as painting
lowers it. A body spawned outright never merges, nor does one a joint names or
the mouse holds.

Deferred until a measurement asks for them: fracture on hard impacts, and
multithreading. Dwyer's engine has both.

## What this deliberately does not do

- **Scale on body transforms, ever.** See above; it is load-bearing, not a
  simplification.
- **Soft bodies and fluids.**
- **Continuous collision detection.** Speculative contacts and a velocity cap
  are the answer until something demonstrates otherwise.
- **Sleeping and islands, until measurement shows they are needed.**

## Provenance

What Douglas Dwyer's devlogs show, and where. His engine is not open source, so
this list is all that is known of it here:

- **#20, [Coding rigid body physics for voxels](https://www.youtube.com/watch?v=byP6cA71Cgw).**
  - Corner and edge voxels, with collisions only corner against voxel and edge
    against edge, after Teardown's approach.
  - A voxel centre transformed into the other volume, checking the nearest 8
    voxels.
  - An iterative contact solver after Millington's *Game Physics Engine
    Development*.
  - Mouse dragging.
  - Disconnected terrain becoming bodies.
- **#26, [My voxel physics engine was BROKEN](https://www.youtube.com/watch?v=R9bror0oqR0).**
  - Corner, edge, face and interior labels.
  - Rounded corners and edges with full faces, using sphere, plane and cylinder
    tests, after spheres and axis-aligned boxes both failed.
  - A TGS loop: detect, then substeps of forces, velocity constraints and
    position integration, reusing contacts.
  - Warm starting keyed by the colliding voxel coordinates.
  - A separate position pass dropped for causing jitter.
  - Per-material density, friction and restitution.
  - Fracturing.
  - Voxel objects that can be edited and break apart.
- **#30, [Adding joints to my physics engine](https://www.youtube.com/watch?v=RvhYKj9kEP8).**
  - Joints as an error function `C`, Jacobian `J` and impulse `λ`, with one
    expression for every joint, `λ = −(J M⁻¹ Jᵀ)⁻¹ J·V₁/Δt`, shown at 7:25.
  - The constraint forms, shown at 4:36:
    - contact penetration, `(x₂+r₂−x₁−r₁)·n₁`;
    - point, `x₂+r₂−x₁−r₁`;
    - line, that offset along two normals;
    - locked, `θ₂ − θ₁` on each axis;
    - hinge, `a₁·b₂` and `a₁·c₂`.
  - Sixteen joint types, each one linear part (point, line, radius) and one
    angular part (one axis, cone).
  - Friction on a door, and motors with a maximum force, driven to a speed or
    to a target (a clock).
  - The mouse grab moved from an unstable explicit-Euler spring to a joint that
    constrains position and rotation.
  - Joints following the body that holds them after a split.
- **#12, [Chopping trees DOWN](https://www.youtube.com/watch?v=5e8ut4NgF-8).**
  - Connected-component labelling by depth-first search, 6-connected, over the
    homogeneous nodes of his sparse voxel octree, with a dense bitmap of visited
    nodes.
- **#13, [OPTIMIZING my physics engine](https://www.youtube.com/watch?v=b_d-0EyOuVg).**
  - Sleeping.
  - Merging debris back into the terrain when out of view.
  - Multithreading.

Filled in by this design, **not** shown in his devlogs:

- the exact pair-test formulas and rounding radii;
- the detachment search's budget, its early exit, and the face scan that
  skips a neighbour's width;
- the free and locked parts that make his list sixteen types, which is a
  reading of it;
- the distance joint as a rope, which he describes as "within a radius";
- the cone's formula, joint friction and motors as rows, and their order;
- the bias on joints, and the grab's absolute caps;
- the lookup reach;
- the ownership rules that report each touch once;
- the unbiased relax solve after each substep's position update, after Erin
  Catto's soft-step solver in Box2D v3;
- the soft bias and slop;
- speculative contacts and the velocity cap;
- the substep count;
- the distance-field broadphase;
- storing angular momentum instead of angular velocity;
- the centre-of-mass pivot formula;
- `u16` density;
- the tick rate.
