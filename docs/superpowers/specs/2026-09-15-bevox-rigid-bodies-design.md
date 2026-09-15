# BEVOX Rigid Bodies — Design

Voxel chunks that detach from the static world, fall, tumble, and come to rest —
rendered by the same ray marcher that draws everything else.

This is new scope. The milestone-8 spec deliberately excluded physics; nothing
here is a deferred item being picked up. It is a second subsystem built on the
first.

## What this is for

Carve the supports out from under a wall and the wall should fall over, not hang
in the air. That is the whole goal. Everything below serves it.

## The one property that makes this tractable

**A body's transform is rigid: rotation and translation, no scale.**

The CPU marcher has taken `volume_to_world: Affine3A` since milestone 2 and been
handed `IDENTITY` at every call site. That hedge was placed for exactly this.
Marching a transformed volume means transforming the ray into the volume's local
frame, marching, and transforming the hit back.

Because the transform is rigid, **`t` is preserved exactly** between world and
local space. A distance along the ray means the same thing in both. That is what
lets the renderer compose the static world and any number of bodies by simply
keeping the nearest hit — no depth buffer, no re-projection, no epsilon.

Allow scale and that property dies: `t` scales per body, the comparison needs a
division, and every existing parity test's notion of distance stops being shared.
Scale is therefore out, permanently, not deferred.

## Rendering

The shader marches the static world, then each body in turn, and keeps the
nearest hit.

```
hit = traverse(world_ray)
for each body:
    local = body.world_to_local * world_ray
    body_hit = traverse_body(local)
    if body_hit.t < hit.t: hit = body_hit (normal rotated back to world)
```

Per ray this is `1 + N` marches, so **the body count is capped** and the cap is a
number in the uniform, not an aspiration. Bodies are small — a collapsed wall
section is hundreds of voxels, not millions — so each body march terminates
quickly, but the cap is what stops a pathological scene from quadratic blowup.

Shadow rays go through the same composition, or bodies cast no shadows and the
lie is visible immediately.

Each body owns its own `Contree`. Bodies are small, so their arenas are small,
and packing them into the existing node and voxel buffers behind a per-body base
offset costs one indirection and avoids a second set of bindings.

## Mass properties

Computed from voxel occupancy, once, when a body is created:

- **mass** — voxel count times a per-material density
- **centre of mass** — the occupancy-weighted mean, which becomes the body's
  local origin so the transform rotates about the right point
- **inertia tensor** — summed per voxel with the parallel-axis theorem, then
  inverted once and stored

A body whose centre of mass is not its local origin tumbles wrongly in a way
that looks almost right, which is the worst kind of wrong. Re-centring at
creation is not an optimisation.

## Integration

Semi-implicit Euler. Orientation as a quaternion, renormalised every step;
angular velocity in world space with the inertia tensor rotated into it each
step. Fixed timestep with an accumulator, because a variable step makes
restitution and resting contact frame-rate dependent, and a voxel scene's frame
time varies by 3x depending on where the camera looks.

## Collision

Against the static world first; body against body last, because it is the part
most easily deferred and the least visible.

A body's **surface voxels** are the only candidates — an interior voxel cannot
touch anything an exterior one does not touch first. They are found once at
creation.

For each surface voxel, its world position queries the static volume. The
distance field already built for empty-space skipping is a conservative
free-radius oracle: a body whose whole bounding sphere sits inside one field
cell's promised cube cannot be touching anything, and that rejects most bodies
most frames for the cost of one lookup.

Contacts resolve with impulses: restitution along the normal, Coulomb friction
in the tangent plane, and a positional correction for penetration that does not
feed energy back in. Resting contact is where naive impulse solvers jitter, and
the honest fix is a small penetration slop plus a bias, not more iterations.

## Detachment

An edit that removes voxels can disconnect part of the structure. Finding the
disconnected part is a connected-components pass over the affected region, which
is why it is the last milestone rather than the first: it is the piece most
coupled to edit throughput, and a single paint already stages about 2.2 MB of
distance-field data at extent 4096.

Until then bodies are spawned deliberately, which is enough to build and test
everything else.

## Testing

The project's existing discipline applies unchanged and is what this design is
shaped around:

- **CPU reference first.** Mass properties, integration and contact resolution
  are pure functions of numbers and are tested without a GPU or a window.
- **Bit-identity.** A scene with zero bodies must render bit-identically to the
  same scene before this work existed. The composition loop is a defect if it
  changes a pixel of a body-free scene.
- **Conservation as a gate.** A body with no gravity and no contacts must
  conserve linear and angular momentum to float tolerance over thousands of
  steps. This is the test that catches an integrator that looks right.
- **A body at rest must stay at rest.** Drop a body on a floor and its height
  after ten thousand steps must equal its height after one thousand. Jitter is
  the characteristic failure of every impulse solver and it hides from short
  tests.
- **Every performance claim is interleaved A/B/A in one session**, drift
  reported, as everywhere else in this project.

And one lesson this project paid for four times over: **a test whose scene or
camera cannot reach the code it names is worse than no test**, because it reads
as coverage. Every gate here states what it would fail against, and the ones
guarding correctness are checked by deliberately breaking the code they guard.

## Milestones

| # | Deliverable | What it proves |
|---|---|---|
| 1 | A transformed body volume rendered, composed with the static world | The renderer can do this at all, and cheaply |
| 2 | Mass properties, integration, gravity, and a body resting on the floor | The physics is right in isolation |
| 3 | Full contact resolution: friction, restitution, angular response | A body tumbles and settles convincingly |
| 4 | Detachment — carving a support spawns a body | The thing this was for |
| 5 | Body against body | Piles, not just single objects |

Milestone 1 first because the rendering is the part that could prove
unaffordable, and everything else is wasted if it does. It carries a hardcoded
spin so it can be seen working before any physics exists.

## What this deliberately does not do

No scale on body transforms, ever — see above; it is load-bearing, not a
simplification.

No soft bodies, no fluids, no constraints or joints. No sleeping or islands
until measurement shows they are needed. No continuous collision detection: a
fast enough body will tunnel, and the fixed timestep plus a velocity cap is the
answer until something demonstrates otherwise.

No re-voxelisation of a settled body back into the static world. It is the
obvious way to stop paying for a body that has stopped moving, and it is a
milestone of its own once there is something to measure.
