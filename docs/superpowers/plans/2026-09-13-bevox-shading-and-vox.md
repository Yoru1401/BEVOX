# BEVOX shading and model import — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Light the voxel scene with implicitly generated per-voxel normals and a sun shadow ray, then load real MagicaVoxel models into it.

**Architecture:** `traverse` already agrees with the CPU reference, so it is extended rather than rewritten: it gains the hit voxel coordinate and entry face normal that shading needs. Normals are summed from neighbour occupancy in the shader exactly as `bevox_core::normal` does on the CPU, shadows are a second `traverse` call with an any-hit early out, and colours come from a palette buffer that `.vox` import fills. Every addition is pinned by a parity test against the CPU implementation of the same thing.

**Tech Stack:** Bevy 0.19.1, wgpu 29.0.4, WGSL, dot_vox 5.2.0, bytemuck.

**Spec:** `docs/superpowers/specs/2026-09-13-bevox-raymarcher-core-design.md`
**Predecessors:** `2026-09-13-bevox-core-and-reference-marcher.md` (milestones 1–2), `2026-09-13-bevox-gpu-traversal.md` (milestones 3–4)

## Global Constraints

- Rust edition 2024, `rust-version = "1.95"`. Native desktop only.
- `MaterialId(0)` is empty. `glam` stays at 0.32 to match Bevy.
- **The CPU is ground truth.** Every GPU feature added here has a CPU counterpart in `bevox_core`, and a parity test comparing them. When they disagree, the GPU is wrong until proven otherwise.
- `march_identity` and its existing parity test must keep passing unchanged. It is the regression net for every edit to `traverse`.
- WGSL reserved words bite: `active` is one. Prefer plain names and let the compiler object.
- Storage buffer minimum binding sizes must match the shader's element type — `array<vec4<u32>>` needs 16, `array<u32>` needs 4. Use `storage_buffer_read_only_sized`; the un-sized generic form silently declares 4 and fails at dispatch.
- Bind group changes touch three places that must move together: `init_march_pipeline`'s layout, `dispatch_march`'s entries, and `run_march` in the parity test.
- Every WGSL loop keeps a hard iteration bound.

## File Structure

| File | Responsibility |
|---|---|
| `crates/bevox_render/assets/shaders/march.wgsl` | Traversal, point sampling, normals, shadows, entry points |
| `crates/bevox_core/src/vox.rs` | MagicaVoxel import: `DotVoxData` to `DenseVolume` + `MaterialTable` |
| `crates/bevox_core/src/material.rs` | Gains `MaterialTable::to_gpu` for palette upload |
| `crates/bevox_render/src/upload.rs` | Palette in `GpuSceneData` |
| `crates/bevox_render/src/pipeline.rs` | Palette binding |
| `crates/bevox_render/tests/gpu_parity.rs` | Normal, shadow and palette parity tests |
| `crates/bevox/src/main.rs` | Loads a `.vox` path from the command line |

---

### Task 1: Hit voxel and face normal from the GPU traversal

Shading needs to know *which* voxel was hit and *which face* the ray entered
through. The CPU `Hit` already carries both; the WGSL one carries neither.

**Files:**
- Modify: `crates/bevox_render/assets/shaders/march.wgsl`
- Modify: `crates/bevox_render/tests/gpu_parity.rs`

**Interfaces:**
- Consumes: `traverse`, `run_march` from Plan 2.
- Produces: WGSL `Hit { hit: bool, material: u32, t: f32, voxel: vec3<u32>, face_normal: vec3<f32> }`; entry point `march_voxel_id` writing the hit voxel coordinate.

- [ ] **Step 1: Write the failing parity test**

Add to `crates/bevox_render/tests/gpu_parity.rs`:

```rust
/// The hit voxel coordinate must match the CPU exactly. A collapsed uniform
/// region is the case that breaks: the region's origin is not the voxel the ray
/// entered, and shading a normal at the wrong coordinate is invisible in a flat
/// material but wrong everywhere a surface turns.
#[test]
fn the_gpu_reports_the_same_hit_voxel_as_the_cpu() {
    let Some((device, queue)) = gpu_device() else {
        eprintln!("no GPU adapter available, skipping");
        return;
    };

    let tree = parity_scene();
    let gpu_volume = GpuVolume::from_contree(&tree);
    let (width, height) = (64u32, 64u32);
    let eye = Vec3::new(-30.0, 40.0, -30.0);
    let view = Mat4::look_at_rh(eye, Vec3::new(32.0, 12.0, 32.0), Vec3::Y);
    let projection = Mat4::perspective_rh(0.9, width as f32 / height as f32, 0.1, 500.0);
    let world_from_clip = (projection * view).inverse();

    let shader = std::fs::read_to_string("assets/shaders/march.wgsl").expect("shader file missing");
    let pixels = run_march(
        &device, &queue, &shader, "march_voxel_id",
        world_from_clip, eye, &tree, &gpu_volume, width, height,
    );

    let mut stats = MarchStats::default();
    let mut mismatches = 0usize;
    let mut first = String::new();

    for y in 0..height {
        for x in 0..width {
            let dir = ray_direction(world_from_clip, eye, x, y, width, height);
            let cpu = march(&tree, Affine3A::IDENTITY, eye, dir, 1000.0, false, &mut stats);
            let i = ((y * width + x) * 4) as usize;

            // Volume extent is 64, so each axis fits in one byte.
            let gpu_hit = pixels[i + 3] > 0;
            let gpu_voxel = UVec3::new(pixels[i] as u32, pixels[i + 1] as u32, pixels[i + 2] as u32);

            let bad = match cpu {
                Some(hit) => !gpu_hit || gpu_voxel != hit.voxel,
                None => gpu_hit,
            };
            if bad {
                if mismatches == 0 {
                    first = format!(
                        "at ({x},{y}) cpu={:?} gpu_hit={gpu_hit} gpu_voxel={gpu_voxel:?}",
                        cpu.map(|h| h.voxel)
                    );
                }
                mismatches += 1;
            }
        }
    }

    assert_eq!(mismatches, 0, "{mismatches} voxel coordinates disagreed; first {first}");
}
```

- [ ] **Step 2: Run the test to verify it fails**

Run: `cargo test -p bevox_render --test gpu_parity the_gpu_reports_the_same_hit_voxel`
Expected: FAIL — the shader has no `march_voxel_id` entry point, so pipeline creation reports an invalid entry point.

- [ ] **Step 3: Extend the shader's Hit and traversal**

In `crates/bevox_render/assets/shaders/march.wgsl`, replace the `Hit` struct:

```wgsl
struct Hit {
    hit: bool,
    material: u32,
    t: f32,
    voxel: vec3<u32>,
    face_normal: vec3<f32>,
};
```

Add the entry-face helper next to `ray_box`, mirroring `entry_normal` in
`bevox_core::march`:

```wgsl
/// Which face of a box the ray entered, from the per-axis entry distances.
fn entry_normal(origin: vec3<f32>, inv_dir: vec3<f32>, lo: vec3<f32>, hi: vec3<f32>) -> vec3<f32> {
    let t0 = (lo - origin) * inv_dir;
    let t1 = (hi - origin) * inv_dir;
    let near = min(t0, t1);
    if near.x >= near.y && near.x >= near.z {
        if inv_dir.x >= 0.0 { return vec3<f32>(-1.0, 0.0, 0.0); }
        return vec3<f32>(1.0, 0.0, 0.0);
    }
    if near.y >= near.z {
        if inv_dir.y >= 0.0 { return vec3<f32>(0.0, -1.0, 0.0); }
        return vec3<f32>(0.0, 1.0, 0.0);
    }
    if inv_dir.z >= 0.0 { return vec3<f32>(0.0, 0.0, -1.0); }
    return vec3<f32>(0.0, 0.0, 1.0);
}
```

Then fill both new fields at the two places `traverse` returns a hit. The
uniform-solid return becomes:

```wgsl
        if is_uniform_solid(frame.node) {
            let region = f32(level_extent(frame.level));
            let lo = vec3<f32>(frame.origin);
            let hi = lo + vec3<f32>(region);
            // A collapsed region covers many voxels: report the one actually
            // entered, not the region's origin. bevox_core::march does the same,
            // and a parity test pins it.
            let point = origin + dir * frame.t_enter;
            let clamped = clamp(
                floor(point),
                lo,
                lo + vec3<f32>(region - 1.0),
            );
            return Hit(
                true,
                node_material(frame.node),
                frame.t_enter,
                vec3<u32>(clamped),
                entry_normal(origin, inv_dir, lo, hi),
            );
        }
```

and the leaf return becomes:

```wgsl
        if frame.level == 0u {
            let lo = vec3<f32>(best_origin);
            let hi = lo + vec3<f32>(1.0);
            return Hit(
                true,
                voxel_byte(slot),
                best_t,
                best_origin,
                entry_normal(origin, inv_dir, lo, hi),
            );
        }
```

Both `Hit(false, ...)` returns become:

```wgsl
    return Hit(false, 0u, 0.0, vec3<u32>(0u), vec3<f32>(0.0));
```

`traverse` needs `dir` for the uniform-region point, so it is already a
parameter — no signature change.

- [ ] **Step 4: Add the voxel-id entry point**

```wgsl
/// Hit voxel coordinate in RGB, hit flag in alpha. Read by the parity test only.
@compute @workgroup_size(8, 8, 1)
fn march_voxel_id(@builtin(global_invocation_id) id: vec3<u32>) {
    let size = textureDimensions(output);
    if id.x >= size.x || id.y >= size.y { return; }

    let hit = traverse(view.camera_position.xyz, primary_ray(id, size), 1000.0);

    var colour = vec4<f32>(0.0, 0.0, 0.0, 0.0);
    if hit.hit {
        // Volumes in these tests are at most 256 per axis, so a byte each.
        colour = vec4<f32>(
            f32(hit.voxel.x) / 255.0,
            f32(hit.voxel.y) / 255.0,
            f32(hit.voxel.z) / 255.0,
            1.0,
        );
    }
    textureStore(output, vec2<i32>(id.xy), colour);
}
```

- [ ] **Step 5: Run the tests to verify they pass**

Run: `cargo test -p bevox_render --test gpu_parity`
Expected: PASS, 4 tests. `march_identity` parity must still be green — if it
broke, the `Hit` change altered traversal behaviour and that is the bug.

- [ ] **Step 6: Commit**

```bash
git add crates/bevox_render
git commit -m "feat(render): report hit voxel and face normal from GPU traversal" -m "Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>"
```

---

### Task 2: Point sampling and implicit normals in WGSL

**Files:**
- Modify: `crates/bevox_render/assets/shaders/march.wgsl`
- Modify: `crates/bevox_render/tests/gpu_parity.rs`

**Interfaces:**
- Consumes: the extended `Hit` from Task 1, `bevox_core::normal::implicit_normal`.
- Produces: WGSL `material_at(p: vec3<i32>) -> u32` and `implicit_normal(voxel: vec3<u32>, face_normal: vec3<f32>) -> vec3<f32>`; entry point `march_normal`.

- [ ] **Step 1: Write the failing parity test**

```rust
/// Normals are summed from neighbour occupancy, so they exercise point sampling
/// at six coordinates around every hit — a completely different path through the
/// tree than the ray march that found the voxel.
#[test]
fn the_gpu_normals_match_the_cpu() {
    let Some((device, queue)) = gpu_device() else {
        eprintln!("no GPU adapter available, skipping");
        return;
    };

    let tree = parity_scene();
    let gpu_volume = GpuVolume::from_contree(&tree);
    let (width, height) = (64u32, 64u32);
    let eye = Vec3::new(-30.0, 40.0, -30.0);
    let view = Mat4::look_at_rh(eye, Vec3::new(32.0, 12.0, 32.0), Vec3::Y);
    let projection = Mat4::perspective_rh(0.9, width as f32 / height as f32, 0.1, 500.0);
    let world_from_clip = (projection * view).inverse();

    let shader = std::fs::read_to_string("assets/shaders/march.wgsl").expect("shader file missing");
    let pixels = run_march(
        &device, &queue, &shader, "march_normal",
        world_from_clip, eye, &tree, &gpu_volume, width, height,
    );

    let mut stats = MarchStats::default();
    let mut mismatches = 0usize;
    let mut worst = 0.0f32;
    let mut first = String::new();

    for y in 0..height {
        for x in 0..width {
            let dir = ray_direction(world_from_clip, eye, x, y, width, height);
            let Some(hit) = march(&tree, Affine3A::IDENTITY, eye, dir, 1000.0, false, &mut stats)
            else {
                continue;
            };

            let cpu_n = bevox_core::normal::implicit_normal(&tree, hit.voxel, hit.face_normal);
            let i = ((y * width + x) * 4) as usize;
            let gpu_n = Vec3::new(
                pixels[i] as f32 / 255.0 * 2.0 - 1.0,
                pixels[i + 1] as f32 / 255.0 * 2.0 - 1.0,
                pixels[i + 2] as f32 / 255.0 * 2.0 - 1.0,
            );

            // One byte per component quantises to steps of 2/255, so allow a
            // little over one step before calling it a disagreement.
            let delta = (gpu_n - cpu_n).length();
            if delta > worst {
                worst = delta;
            }
            if delta > 0.02 {
                if mismatches == 0 {
                    first = format!("at ({x},{y}) cpu={cpu_n:?} gpu={gpu_n:?} delta={delta}");
                }
                mismatches += 1;
            }
        }
    }

    assert_eq!(mismatches, 0, "{mismatches} normals disagreed (worst {worst}); first {first}");
}
```

- [ ] **Step 2: Run the test to verify it fails**

Run: `cargo test -p bevox_render --test gpu_parity the_gpu_normals_match_the_cpu`
Expected: FAIL — no `march_normal` entry point.

- [ ] **Step 3: Add point sampling to the shader**

`traverse` walks the tree along a ray; this walks it to a single coordinate.
The descent mirrors `Contree::get`.

```wgsl
/// Material at a voxel coordinate, or 0 outside the volume.
///
/// This is the tree descent from Contree::get: at each level pick the child
/// containing the coordinate, stop early at an unsubdivided node.
fn material_at(p: vec3<i32>) -> u32 {
    let extent = i32(view.volume_params.y);
    if p.x < 0 || p.y < 0 || p.z < 0 || p.x >= extent || p.y >= extent || p.z >= extent {
        return 0u;
    }

    var node = nodes[0];
    var level = view.volume_params.x - 1u;
    var local = vec3<u32>(p);

    // Bounded by tree depth; the volume is never deeper than MAX_DEPTH.
    for (var guard: u32 = 0u; guard < MAX_DEPTH; guard = guard + 1u) {
        if !is_subdivided(node) {
            return node_material(node);
        }

        if level == 0u {
            let i = local.x + local.y * BRICK_EDGE + local.z * BRICK_EDGE * BRICK_EDGE;
            if !has_child(node, i) {
                return 0u;
            }
            return voxel_byte(child_slot(node, i));
        }

        let step_size = level_extent(level - 1u);
        let cell = local / step_size;
        let i = cell.x + cell.y * BRICK_EDGE + cell.z * BRICK_EDGE * BRICK_EDGE;
        if !has_child(node, i) {
            return 0u;
        }
        node = nodes[child_slot(node, i) + 1u];
        local = local - cell * step_size;
        level = level - 1u;
    }
    return 0u;
}

/// Sums the directions in which a voxel is exposed, falling back to the entry
/// face when that carries no information. Mirrors bevox_core::normal.
fn implicit_normal(voxel: vec3<u32>, face_normal: vec3<f32>) -> vec3<f32> {
    let base = vec3<i32>(voxel);
    var sum = vec3<f32>(0.0);

    if material_at(base + vec3<i32>(1, 0, 0)) == 0u { sum += vec3<f32>(1.0, 0.0, 0.0); }
    if material_at(base + vec3<i32>(-1, 0, 0)) == 0u { sum += vec3<f32>(-1.0, 0.0, 0.0); }
    if material_at(base + vec3<i32>(0, 1, 0)) == 0u { sum += vec3<f32>(0.0, 1.0, 0.0); }
    if material_at(base + vec3<i32>(0, -1, 0)) == 0u { sum += vec3<f32>(0.0, -1.0, 0.0); }
    if material_at(base + vec3<i32>(0, 0, 1)) == 0u { sum += vec3<f32>(0.0, 0.0, 1.0); }
    if material_at(base + vec3<i32>(0, 0, -1)) == 0u { sum += vec3<f32>(0.0, 0.0, -1.0); }

    if dot(sum, sum) < 1e-6 {
        return face_normal;
    }
    return normalize(sum);
}

/// Normal encoded into unsigned bytes, hit flag in alpha.
@compute @workgroup_size(8, 8, 1)
fn march_normal(@builtin(global_invocation_id) id: vec3<u32>) {
    let size = textureDimensions(output);
    if id.x >= size.x || id.y >= size.y { return; }

    let hit = traverse(view.camera_position.xyz, primary_ray(id, size), 1000.0);

    var colour = vec4<f32>(0.0, 0.0, 0.0, 0.0);
    if hit.hit {
        let n = implicit_normal(hit.voxel, hit.face_normal);
        colour = vec4<f32>(n * 0.5 + vec3<f32>(0.5), 1.0);
    }
    textureStore(output, vec2<i32>(id.xy), colour);
}
```

- [ ] **Step 4: Run the tests to verify they pass**

Run: `cargo test -p bevox_render --test gpu_parity`
Expected: PASS, 5 tests.

If normals disagree only at region boundaries, suspect `material_at`'s early
exit at unsubdivided nodes: a uniform *solid* region must report its material,
an empty one must report 0, and conflating them makes interior faces look
exposed.

- [ ] **Step 5: Commit**

```bash
git add crates/bevox_render
git commit -m "feat(render): generate implicit normals in the shader" -m "Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>"
```

---

### Task 3: Sun shadow ray

**Files:**
- Modify: `crates/bevox_render/assets/shaders/march.wgsl`
- Modify: `crates/bevox_render/src/upload.rs`, `crates/bevox_render/src/pipeline.rs`
- Modify: `crates/bevox_render/tests/gpu_parity.rs`

**Interfaces:**
- Produces: `MarchUniform.sun_direction: [f32; 4]` (uniform grows to 112 bytes); WGSL `traverse_any(origin, dir, max_dist) -> bool`; entry point `march_shadow`.

- [ ] **Step 1: Widen the uniform**

In `crates/bevox_render/src/upload.rs`:

```rust
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, Pod, Zeroable)]
pub struct MarchUniform {
    pub world_from_clip: [[f32; 4]; 4],
    pub camera_position: [f32; 4],
    /// Normalised direction *toward* the sun.
    pub sun_direction: [f32; 4],
    /// `[depth, extent, 0, 0]`.
    pub volume_params: [u32; 4],
}

/// The sun direction the renderer and the parity tests share.
pub const SUN_DIRECTION: Vec3 = Vec3::new(0.4, 1.0, 0.25);
```

and set it in `march_uniform`:

```rust
        sun_direction: SUN_DIRECTION.normalize().extend(0.0).to_array(),
```

Update the size test — the layout is now 64 + 16 + 16 + 16:

```rust
    #[test]
    fn the_uniform_is_the_size_the_shader_expects() {
        assert_eq!(size_of::<MarchUniform>(), 112);
        assert_eq!(align_of::<MarchUniform>(), 4);
    }
```

`init_march_pipeline` needs no edit: it already sizes the binding from
`size_of::<MarchUniform>()`.

- [ ] **Step 2: Run the size test to verify it fails**

Run: `cargo test -p bevox_render upload`
Expected: FAIL — 96 does not equal 112 until the field is added, which is the
point: the binding size and the struct cannot drift apart silently.

- [ ] **Step 3: Mirror the field in the shader and the test harness**

In `march.wgsl`:

```wgsl
struct MarchUniform {
    world_from_clip: mat4x4<f32>,
    camera_position: vec4<f32>,
    sun_direction: vec4<f32>,
    volume_params: vec4<u32>,
};
```

In `gpu_parity.rs`, `TestUniform` gains the same field in the same position:

```rust
#[repr(C)]
#[derive(Clone, Copy, bytemuck::Pod, bytemuck::Zeroable)]
struct TestUniform {
    world_from_clip: [[f32; 4]; 4],
    camera_position: [f32; 4],
    sun_direction: [f32; 4],
    volume_params: [u32; 4],
}
```

and `run_march` fills it:

```rust
        sun_direction: bevox_render::upload::SUN_DIRECTION.normalize().extend(0.0).to_array(),
```

- [ ] **Step 4: Write the failing shadow parity test**

```rust
/// Shadow rays start offset along the normal and stop at the first occluder, so
/// they exercise the any-hit path and the self-intersection offset together.
#[test]
fn the_gpu_shadows_match_the_cpu() {
    let Some((device, queue)) = gpu_device() else {
        eprintln!("no GPU adapter available, skipping");
        return;
    };

    let tree = parity_scene();
    let gpu_volume = GpuVolume::from_contree(&tree);
    let (width, height) = (64u32, 64u32);
    let eye = Vec3::new(-30.0, 40.0, -30.0);
    let view = Mat4::look_at_rh(eye, Vec3::new(32.0, 12.0, 32.0), Vec3::Y);
    let projection = Mat4::perspective_rh(0.9, width as f32 / height as f32, 0.1, 500.0);
    let world_from_clip = (projection * view).inverse();
    let sun = bevox_render::upload::SUN_DIRECTION.normalize();

    let shader = std::fs::read_to_string("assets/shaders/march.wgsl").expect("shader file missing");
    let pixels = run_march(
        &device, &queue, &shader, "march_shadow",
        world_from_clip, eye, &tree, &gpu_volume, width, height,
    );

    let mut stats = MarchStats::default();
    let mut mismatches = 0usize;
    let mut lit = 0usize;
    let mut shadowed = 0usize;

    for y in 0..height {
        for x in 0..width {
            let dir = ray_direction(world_from_clip, eye, x, y, width, height);
            let Some(hit) = march(&tree, Affine3A::IDENTITY, eye, dir, 1000.0, false, &mut stats)
            else {
                continue;
            };

            let n = bevox_core::normal::implicit_normal(&tree, hit.voxel, hit.face_normal);
            let origin = hit.voxel.as_vec3() + Vec3::splat(0.5) + n * 0.75;
            let cpu_shadowed =
                march(&tree, Affine3A::IDENTITY, origin, sun, 500.0, true, &mut stats).is_some();

            let i = ((y * width + x) * 4) as usize;
            let gpu_shadowed = pixels[i] > 127;

            if cpu_shadowed { shadowed += 1 } else { lit += 1 }
            if cpu_shadowed != gpu_shadowed {
                mismatches += 1;
            }
        }
    }

    // A test where nothing is shadowed would pass trivially.
    assert!(shadowed > 0, "no shadowed pixels; the scene or sun makes this test vacuous");
    assert!(lit > 0, "every pixel shadowed; the scene or sun makes this test vacuous");
    assert_eq!(mismatches, 0, "{mismatches} shadow decisions disagreed");
}
```

- [ ] **Step 5: Implement the shadow ray**

Add to `march.wgsl`. `traverse_any` is a thin wrapper rather than a second
traversal: duplicating the walk would let the two drift apart.

```wgsl
/// Whether anything is hit within `max_dist`. Shadow rays do not care which
/// voxel occludes them, only that one does.
fn traverse_any(origin: vec3<f32>, dir: vec3<f32>, max_dist: f32) -> bool {
    return traverse(origin, dir, max_dist).hit;
}

/// Shadow flag in red: 255 shadowed, 0 lit.
@compute @workgroup_size(8, 8, 1)
fn march_shadow(@builtin(global_invocation_id) id: vec3<u32>) {
    let size = textureDimensions(output);
    if id.x >= size.x || id.y >= size.y { return; }

    let hit = traverse(view.camera_position.xyz, primary_ray(id, size), 1000.0);

    var colour = vec4<f32>(0.0, 0.0, 0.0, 0.0);
    if hit.hit {
        let n = implicit_normal(hit.voxel, hit.face_normal);
        // Offset along the normal so the ray does not immediately re-hit its own
        // voxel. 0.75 matches bevox_core's reference renderer.
        let origin = vec3<f32>(hit.voxel) + vec3<f32>(0.5) + n * 0.75;
        var shadowed = 0.0;
        if traverse_any(origin, view.sun_direction.xyz, 500.0) {
            shadowed = 1.0;
        }
        colour = vec4<f32>(shadowed, 0.0, 0.0, 1.0);
    }
    textureStore(output, vec2<i32>(id.xy), colour);
}
```

- [ ] **Step 6: Run the tests to verify they pass**

Run: `cargo test -p bevox_render`
Expected: PASS — 6 parity tests plus the crate's unit tests.

If shadows disagree on surfaces facing the sun, the offset is the first
suspect: too small and surfaces shadow themselves, too large and thin geometry
is stepped over.

- [ ] **Step 7: Commit**

```bash
git add crates/bevox_render
git commit -m "feat(render): add the sun shadow ray" -m "Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>"
```

---

### Task 4: Palette buffer and lit display output — milestone 5

**Files:**
- Modify: `crates/bevox_core/src/material.rs`
- Modify: `crates/bevox_render/src/upload.rs`, `crates/bevox_render/src/pipeline.rs`
- Modify: `crates/bevox_render/assets/shaders/march.wgsl`
- Modify: `crates/bevox_render/tests/gpu_parity.rs`

**Interfaces:**
- Produces: `MaterialTable::to_gpu(&self) -> Vec<[f32; 4]>` (linear RGBA, 256 entries); `GpuSceneData.palette: Vec<[f32; 4]>`; binding 4 in the layout, entries and harness.

- [ ] **Step 1: Write the failing palette test**

In `crates/bevox_core/src/material.rs`:

```rust
    #[test]
    fn the_gpu_palette_is_always_two_hundred_and_fifty_six_entries() {
        let mut table = MaterialTable::new();
        table.push(Material { color: [255, 128, 0, 255] }).unwrap();
        let gpu = table.to_gpu();
        assert_eq!(gpu.len(), 256, "the shader indexes this by a byte");
    }

    #[test]
    fn palette_entries_are_normalised_and_slot_zero_is_transparent() {
        let mut table = MaterialTable::new();
        let id = table.push(Material { color: [255, 128, 0, 255] }).unwrap();
        let gpu = table.to_gpu();

        assert_eq!(gpu[0], [0.0, 0.0, 0.0, 0.0], "slot 0 is empty space");
        let e = gpu[id.0 as usize];
        assert!((e[0] - 1.0).abs() < 1e-6, "red was {}", e[0]);
        assert!((e[1] - 128.0 / 255.0).abs() < 1e-6, "green was {}", e[1]);
        assert!((e[3] - 1.0).abs() < 1e-6, "alpha was {}", e[3]);
    }

    #[test]
    fn unset_palette_slots_are_zero() {
        let table = MaterialTable::new();
        let gpu = table.to_gpu();
        assert!(gpu.iter().all(|e| *e == [0.0, 0.0, 0.0, 0.0]));
    }
```

- [ ] **Step 2: Run to verify it fails**

Run: `cargo test -p bevox_core material`
Expected: FAIL — no method `to_gpu`.

- [ ] **Step 3: Implement the palette conversion**

```rust
    /// The palette as the shader indexes it: 256 linear RGBA entries, whatever
    /// the table's current length, because a voxel byte can name any slot.
    pub fn to_gpu(&self) -> Vec<[f32; 4]> {
        let mut out = vec![[0.0f32; 4]; 256];
        for (i, m) in self.entries.iter().enumerate().skip(1) {
            out[i] = [
                m.color[0] as f32 / 255.0,
                m.color[1] as f32 / 255.0,
                m.color[2] as f32 / 255.0,
                m.color[3] as f32 / 255.0,
            ];
        }
        out
    }
```

- [ ] **Step 4: Thread the palette through the render crate**

> **This step changes the bind group layout, so three places move together or
> nothing works: `init_march_pipeline`'s entries, `dispatch_march`'s bind group,
> and `run_march` in the parity harness. It also changes `VoxelScene`, so
> `main.rs` is updated here rather than in Task 6 — leaving the workspace
> uncompilable between tasks is not acceptable.**

In `crates/bevox_render/src/upload.rs`:

```rust
use bevox_core::material::MaterialTable;

#[derive(Resource)]
pub struct VoxelScene {
    pub tree: Contree,
    pub materials: MaterialTable,
    pub generation: u32,
}
```

`GpuSceneData` gains the palette:

```rust
#[derive(Resource, Clone, ExtractResource)]
pub struct GpuSceneData {
    pub nodes: Vec<GpuNode>,
    pub voxels: Vec<u32>,
    pub palette: Vec<[f32; 4]>,
    pub depth: u32,
    pub extent: u32,
    pub generation: u32,
}

impl Default for GpuSceneData {
    fn default() -> Self {
        Self {
            nodes: vec![GpuNode::default()],
            voxels: Vec::new(),
            palette: MaterialTable::new().to_gpu(),
            depth: 1,
            extent: 4,
            generation: 0,
        }
    }
}
```

and `build_gpu_scene` fills it:

```rust
    commands.insert_resource(GpuSceneData {
        nodes: volume.buffer_nodes(),
        voxels: volume.voxels,
        palette: scene.materials.to_gpu(),
        depth: scene.tree.depth(),
        extent: scene.tree.extent(),
        generation: scene.generation,
    });
```

In `crates/bevox_render/src/pipeline.rs`, the layout gains a fourth buffer
binding, pushing the storage texture to binding 4:

```rust
    let entries = BindGroupLayoutEntries::sequential(
        ShaderStages::COMPUTE,
        (
            uniform_buffer_sized(false, NonZero::new(size_of::<MarchUniform>() as u64)),
            storage_buffer_read_only_sized(false, NonZero::new(16)),
            storage_buffer_read_only_sized(false, NonZero::new(4)),
            // Palette: array<vec4<f32>>, so one element is 16 bytes.
            storage_buffer_read_only_sized(false, NonZero::new(16)),
            texture_storage_2d(TextureFormat::Rgba8Unorm, StorageTextureAccess::WriteOnly),
        ),
    );
```

`MarchBuffers` gains `pub palette: Buffer`, built alongside the others:

```rust
        palette: device.create_buffer_with_data(&BufferInitDescriptor {
            label: Some("bevox_palette"),
            contents: bytemuck::cast_slice(&scene.palette),
            usage: BufferUsages::STORAGE,
        }),
```

and `dispatch_march` binds it in the same position:

```rust
        &BindGroupEntries::sequential((
            buffers.uniform.as_entire_binding(),
            buffers.nodes.as_entire_binding(),
            buffers.voxels.as_entire_binding(),
            buffers.palette.as_entire_binding(),
            &gpu_image.texture_view,
        )),
```

In `crates/bevox/src/main.rs`, `demo_scene` returns its palette too, so the
built-in scene and imported models take the same path:

```rust
use bevox_core::material::{Material, MaterialTable};

/// The demo scene with the palette it is drawn from.
fn demo_scene() -> (Contree, MaterialTable) {
    let mut dense = DenseVolume::new(64).unwrap();
    for z in 0..64 {
        for x in 0..64 {
            for y in 0..6 {
                dense.set(UVec3::new(x, y, z), MaterialId(1));
            }
        }
    }
    for z in 28..36 {
        for y in 6..26 {
            for x in 28..36 {
                dense.set(UVec3::new(x, y, z), MaterialId(2));
            }
        }
    }
    let mut tree = dense.into_contree();
    tree.apply_sphere(Vec3::new(32.0, 18.0, 32.0), 5.0, MaterialId::EMPTY);

    let mut materials = MaterialTable::new();
    materials.push(Material { color: [140, 140, 150, 255] }).unwrap(); // 1: stone
    materials.push(Material { color: [180, 90, 70, 255] }).unwrap();   // 2: brick
    (tree, materials)
}
```

and the insertion becomes:

```rust
    let (tree, materials) = demo_scene();
    commands.insert_resource(VoxelScene { tree, materials, generation: 1 });
```

- [ ] **Step 5: Update the shader bindings and light the display output**

```wgsl
@group(0) @binding(0) var<uniform> view: MarchUniform;
@group(0) @binding(1) var<storage, read> nodes: array<vec4<u32>>;
@group(0) @binding(2) var<storage, read> voxels: array<u32>;
@group(0) @binding(3) var<storage, read> palette: array<vec4<f32>>;
@group(0) @binding(4) var output: texture_storage_2d<rgba8unorm, write>;
```

and the display entry point becomes:

```wgsl
@compute @workgroup_size(8, 8, 1)
fn march(@builtin(global_invocation_id) id: vec3<u32>) {
    let size = textureDimensions(output);
    if id.x >= size.x || id.y >= size.y { return; }

    let hit = traverse(view.camera_position.xyz, primary_ray(id, size), 1000.0);

    var colour = vec3<f32>(0.35, 0.47, 0.70);  // sky
    if hit.hit {
        let n = implicit_normal(hit.voxel, hit.face_normal);
        let sun = view.sun_direction.xyz;

        let origin = vec3<f32>(hit.voxel) + vec3<f32>(0.5) + n * 0.75;
        var diffuse = max(dot(n, sun), 0.0) * 0.75;
        if traverse_any(origin, sun, 500.0) {
            diffuse = 0.0;
        }

        let base = palette[hit.material].rgb;
        colour = base * (0.25 + diffuse);
    }
    textureStore(output, vec2<i32>(id.xy), vec4<f32>(colour, 1.0));
}
```

`material_colour` is deleted: the palette replaces it.

Every other entry point keeps `binding(4)` for the texture, so update the
`textureStore` target declaration only — their bodies are unchanged.

- [ ] **Step 6: Update the harness for five bindings**

`run_march` builds the palette itself rather than taking one, so the ten-argument
signature the Task 1–3 tests already call is unchanged. Those tests read voxel
ids, normals and shadow flags, none of which depend on colour; a test that needs
real colours can take a palette parameter when one exists.

In `crates/bevox_render/tests/gpu_parity.rs`, inside `run_march`, alongside the
other buffers:

```rust
    // The identity, normal and shadow entry points never read the palette, so a
    // zeroed one is sufficient and keeps this signature stable.
    let palette = bevox_core::material::MaterialTable::new().to_gpu();
    let palette_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
        label: Some("palette"),
        contents: bytemuck::cast_slice(&palette),
        usage: wgpu::BufferUsages::STORAGE,
    });
```

and the bind group entries become five, with the texture last at binding 4:

```rust
        entries: &[
            wgpu::BindGroupEntry { binding: 0, resource: uniform_buffer.as_entire_binding() },
            wgpu::BindGroupEntry { binding: 1, resource: node_buffer.as_entire_binding() },
            wgpu::BindGroupEntry { binding: 2, resource: voxel_buffer.as_entire_binding() },
            wgpu::BindGroupEntry { binding: 3, resource: palette_buffer.as_entire_binding() },
            wgpu::BindGroupEntry {
                binding: 4,
                resource: wgpu::BindingResource::TextureView(&view),
            },
        ],
```

- [ ] **Step 7: Run everything**

Run: `cargo test`
Expected: PASS. All six parity tests, including `march_identity`, which must be
unaffected by shading changes.

- [ ] **Step 8: Confirm the deliverable**

Run: `cargo run -p bevox --release`
Expected: the column casts a shadow onto the floor, surfaces facing the sun are
brighter than those facing away, and the carved sphere's interior is visibly
darker than the outer faces.

- [ ] **Step 9: Commit**

```bash
git add crates
git commit -m "feat(render): light the scene with palette colours and shadows" -m "Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>"
```

---

### Task 5: MagicaVoxel import in `bevox_core`

**Files:**
- Create: `crates/bevox_core/src/vox.rs`
- Modify: `crates/bevox_core/src/lib.rs`, `crates/bevox_core/Cargo.toml`

**Interfaces:**
- Produces: `VoxError` (`NoModels`, `ModelOutOfRange { index, count }`, `TooLarge { extent }`); `import_model(data: &DotVoxData, index: usize) -> Result<(DenseVolume, MaterialTable), VoxError>`; `load_vox(path: &Path) -> Result<(DenseVolume, MaterialTable), VoxError>`.

Taking `DotVoxData` rather than a path is what makes this testable: the tests
build models in memory and never touch the filesystem.

- [ ] **Step 1: Add the dependency**

```bash
cargo add dot_vox@5.2 -p bevox_core
```

- [ ] **Step 2: Write the failing tests**

`crates/bevox_core/src/vox.rs`:

```rust
//! MagicaVoxel import.
//!
//! Two conventions differ from ours and both are handled here rather than at
//! call sites: MagicaVoxel is Z-up where we are Y-up, and its model sizes are
//! arbitrary where our volumes are powers of four.

use crate::dense::DenseVolume;
use crate::material::{Material, MaterialId, MaterialTable};
use dot_vox::DotVoxData;
use glam::UVec3;

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum VoxError {
    NoModels,
    ModelOutOfRange { index: usize, count: usize },
    /// Larger than the biggest volume the engine addresses.
    TooLarge { extent: u32 },
}

impl core::fmt::Display for VoxError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            VoxError::NoModels => write!(f, "the file contains no models"),
            VoxError::ModelOutOfRange { index, count } => {
                write!(f, "model {index} requested but the file has {count}")
            }
            VoxError::TooLarge { extent } => {
                write!(f, "model needs an extent of {extent}, beyond the 1024 limit")
            }
        }
    }
}

impl core::error::Error for VoxError {}

/// Smallest power of four that fits `size`, or `None` beyond 1024.
pub fn fitting_extent(size: u32) -> Option<u32> {
    let mut extent = 4u32;
    while extent < size {
        extent *= 4;
        if extent > 1024 {
            return None;
        }
    }
    Some(extent)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn model(size: (u32, u32, u32), voxels: &[(u8, u8, u8, u8)]) -> DotVoxData {
        DotVoxData {
            version: 150,
            models: vec![dot_vox::Model {
                size: dot_vox::Size { x: size.0, y: size.1, z: size.2 },
                voxels: voxels
                    .iter()
                    .map(|(x, y, z, i)| dot_vox::Voxel { x: *x, y: *y, z: *z, i: *i })
                    .collect(),
            }],
            palette: (0..256)
                .map(|i| dot_vox::Color { r: i as u8, g: 0, b: 0, a: 255 })
                .collect(),
            materials: Vec::new(),
            scenes: Vec::new(),
            layers: Vec::new(),
        }
    }

    #[test]
    fn extents_round_up_to_powers_of_four() {
        assert_eq!(fitting_extent(1), Some(4));
        assert_eq!(fitting_extent(4), Some(4));
        assert_eq!(fitting_extent(5), Some(16));
        assert_eq!(fitting_extent(64), Some(64));
        assert_eq!(fitting_extent(126), Some(256));
        assert_eq!(fitting_extent(1024), Some(1024));
        assert_eq!(fitting_extent(1025), None);
    }

    #[test]
    fn a_file_with_no_models_is_an_error() {
        let mut data = model((1, 1, 1), &[]);
        data.models.clear();
        assert_eq!(import_model(&data, 0), Err(VoxError::NoModels));
    }

    #[test]
    fn asking_for_a_model_beyond_the_file_is_an_error() {
        let data = model((1, 1, 1), &[]);
        assert_eq!(
            import_model(&data, 3),
            Err(VoxError::ModelOutOfRange { index: 3, count: 1 })
        );
    }

    #[test]
    fn z_up_becomes_y_up() {
        // A single voxel high on MagicaVoxel's z axis must land high on our y.
        let data = model((4, 4, 4), &[(1, 2, 3, 7)]);
        let (volume, _) = import_model(&data, 0).unwrap();
        assert_eq!(volume.get(UVec3::new(1, 3, 2)), MaterialId(7));
        assert_eq!(volume.get(UVec3::new(1, 2, 3)), MaterialId::EMPTY);
    }

    #[test]
    fn the_volume_is_padded_to_a_power_of_four() {
        let data = model((5, 5, 5), &[(4, 4, 4, 1)]);
        let (volume, _) = import_model(&data, 0).unwrap();
        assert_eq!(volume.extent(), 16);
        assert_eq!(volume.get(UVec3::new(4, 4, 4)), MaterialId(1));
    }

    #[test]
    fn the_palette_is_carried_across() {
        let data = model((4, 4, 4), &[(0, 0, 0, 5)]);
        let (_, materials) = import_model(&data, 0).unwrap();
        // dot_vox palette index i holds red = i in this fixture.
        assert_eq!(materials.get(MaterialId(5)).color[0], 5);
        assert_eq!(materials.get(MaterialId::EMPTY).color[3], 0, "slot 0 stays empty");
    }

    #[test]
    fn an_oversized_model_is_rejected() {
        let data = model((2000, 4, 4), &[]);
        assert_eq!(import_model(&data, 0), Err(VoxError::TooLarge { extent: 2000 }));
    }
}
```

- [ ] **Step 3: Run to verify it fails**

Run: `cargo test -p bevox_core vox`
Expected: FAIL — `import_model` not found.

- [ ] **Step 4: Implement the import**

```rust
/// Converts one model to a padded volume plus its palette.
pub fn import_model(
    data: &DotVoxData,
    index: usize,
) -> Result<(DenseVolume, MaterialTable), VoxError> {
    if data.models.is_empty() {
        return Err(VoxError::NoModels);
    }
    let model = data
        .models
        .get(index)
        .ok_or(VoxError::ModelOutOfRange { index, count: data.models.len() })?;

    // Our axes are Y-up, MagicaVoxel's are Z-up, so its y and z swap.
    let longest = model.size.x.max(model.size.y).max(model.size.z);
    let extent = fitting_extent(longest).ok_or(VoxError::TooLarge { extent: longest })?;

    let mut volume = DenseVolume::new(extent).expect("fitting_extent returns a legal extent");
    for v in &model.voxels {
        volume.set(
            UVec3::new(v.x as u32, v.z as u32, v.y as u32),
            MaterialId(v.i),
        );
    }

    let mut materials = MaterialTable::new();
    for c in data.palette.iter().take(255) {
        materials
            .push(Material { color: [c.r, c.g, c.b, c.a] })
            .expect("at most 255 entries are pushed");
    }

    Ok((volume, materials))
}

/// Reads a `.vox` file and imports its first model.
pub fn load_vox(path: &std::path::Path) -> Result<(DenseVolume, MaterialTable), VoxError> {
    let data = dot_vox::load(path.to_str().unwrap_or_default()).map_err(|_| VoxError::NoModels)?;
    import_model(&data, 0)
}
```

> **Verify before trusting:** `dot_vox`'s type names (`Model`, `Size`, `Voxel`,
> `Color`, and the `DotVoxData` field list) and `dot_vox::load`'s signature are
> taken from its 5.2 API. If the compiler disagrees, run
> `cargo doc -p dot_vox --no-deps --open` and follow what is there; the mapping
> logic — axis swap, padding, palette copy — is unaffected.

The `load_vox` error mapping is deliberately coarse: a parse failure and an
empty file are both "this file gave us no model". Refine it when a caller needs
to tell them apart.

- [ ] **Step 5: Run the tests to verify they pass**

Run: `cargo test -p bevox_core`
Expected: PASS, 7 new tests.

- [ ] **Step 6: Commit**

```bash
git add crates/bevox_core Cargo.lock
git commit -m "feat(core): import MagicaVoxel models" -m "Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>"
```

---

### Task 6: Load a model from the command line — milestone 6

**Files:**
- Modify: `crates/bevox/src/main.rs`

**Interfaces:**
- Consumes: `load_vox`, `VoxelScene`.
- Produces: an app that renders `cargo run -p bevox -- model.vox`, falling back to the built-in scene.

- [ ] **Step 1: Load the model when a path is given**

```rust
fn setup(mut commands: Commands) {
    commands.spawn(Camera2d);

    let (tree, materials) = match std::env::args().nth(1) {
        Some(path) => match bevox_core::vox::load_vox(std::path::Path::new(&path)) {
            Ok((volume, materials)) => {
                info!("loaded {path}: extent {}", volume.extent());
                (volume.into_contree(), materials)
            }
            Err(e) => {
                // A bad path is the user's typo, not a crash: say so and show
                // the demo scene instead.
                error!("could not load {path}: {e}");
                demo_scene()
            }
        },
        None => demo_scene(),
    };

    let centre = Vec3::splat(tree.extent() as f32 * 0.5);
    let eye = centre + Vec3::new(-1.0, 1.2, -1.0) * tree.extent() as f32 * 0.7;

    commands.spawn((
        Camera3d::default(),
        Camera { order: -1, is_active: false, ..default() },
        Transform::from_translation(eye),
        FlyCamera::looking_at(eye, centre),
    ));

    commands.insert_resource(VoxelScene { tree, materials, generation: 1 });
}
```

`demo_scene` already returns `(Contree, MaterialTable)` from Task 4, so both
branches of the match yield the same pair and the renderer sees no difference
between a built-in scene and an imported model.

The camera is framed from the volume's extent rather than hardcoded, because an
imported model may be 16 or 1024 voxels across and a fixed position would put it
off screen or inside the geometry.

- [ ] **Step 2: Confirm the fallback still renders**

Run: `cargo run -p bevox --release`
Expected: the demo scene, lit and shadowed exactly as at the end of Task 4.

- [ ] **Step 3: Confirm a bad path does not crash**

Run: `cargo run -p bevox --release -- nope.vox`
Expected: an error line naming the file, then the demo scene.

- [ ] **Step 4: Confirm a real model renders**

Download any MagicaVoxel `.vox` model, then:

Run: `cargo run -p bevox --release -- path/to/model.vox`
Expected: the model renders with its own palette colours, framed in view, with
sun shading and shadows. Fly around it to confirm the geometry is solid from all
sides rather than correct from one angle.

- [ ] **Step 5: Run the whole suite**

Run: `cargo test`
Expected: PASS.

- [ ] **Step 6: Commit**

```bash
git add crates
git commit -m "feat: render MagicaVoxel models from the command line" -m "Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>"
```

---

## Milestone check

- **Milestone 5** — implicit per-voxel normals and a sun shadow ray, each proven against its CPU counterpart pixel by pixel (Tasks 1–4).
- **Milestone 6** — `.vox` models load, with their palettes, and render (Tasks 5–6).

## What this plan deliberately does not do

No ambient occlusion, no global illumination, no transparency. No DDA, no
bitmask filtering, no beam prepass — the shader stays the slow obvious version
whose correctness is pinned, because the optimisations in milestone 8 are only
safe on top of a reference that already agrees.

No editing from input, no streaming, no LODs.

## Subsequent plans

- **Plan 4 — milestone 7.** The sphere brush wired to mouse input, with dirty-range upload rather than whole-volume re-upload.
- **Plan 5 — milestone 8.** DDA within bricks, the bitmask filter, and the beam prepass. Each measured A/B/A with GPU timestamp queries, and each required to leave the parity tests bit-identical. That requirement is why they come last.
