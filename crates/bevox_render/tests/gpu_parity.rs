//! Runs the real shader on a headless device and compares it with the CPU
//! reference marcher. Skips cleanly when no adapter is available, so a machine
//! without a usable GPU reports a skip rather than a false failure.

use bevox_core::body::Body;
use bevox_core::contree::Contree;
use bevox_core::dense::DenseVolume;
use bevox_core::gpu::GpuVolume;
use bevox_core::march::{MarchStats, march};
use bevox_render::cull::GpuBodyRect;
use bevox_render::upload::{VoxelScene, march_flags};
use bevox_core::material::MaterialId;
use glam::{Affine3A, Mat4, Quat, UVec3, Vec3, Vec4};

mod common;
use common::*;


/// Small scene with a floor, a column and a carved sphere: collapsed uniform
/// regions, subdivided nodes and empty space all in one.
fn parity_scene() -> Contree {
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
    tree
}



/// Same ray construction the shader performs, operation for operation, so both
/// sides march the same rays bit for bit: camera-relative, nothing subtracted,
/// and the pixel centre as an exact integer times an explicit reciprocal.
fn ray_direction(offset_from_clip: Mat4, x: u32, y: u32, w: u32, h: u32) -> Vec3 {
    let ndc_x = ((2 * x + 1) as f32 - w as f32) * (1.0 / w as f32);
    let ndc_y = (h as f32 - (2 * y + 1) as f32) * (1.0 / h as f32);
    let p = offset_from_clip * Vec4::new(ndc_x, ndc_y, 1.0, 1.0);
    (p.truncate() / p.w).normalize()
}

/// Renders one frame with the given traversal flags and reads it back.
#[allow(clippy::too_many_arguments)]
fn run_march_flagged(
    device: &wgpu::Device,
    queue: &wgpu::Queue,
    source: &str,
    entry_point: &str,
    offset_from_clip: Mat4,
    eye: Vec3,
    tree: &Contree,
    width: u32,
    height: u32,
    flags: u32,
) -> Vec<u8> {
    Prepared::new(
        device, source, entry_point, tree, offset_from_clip, eye, width, height, flags,
        &[],
    )
    .read_back(device, queue)
}

/// Renders one frame with the shared configuration and reads it back.
#[allow(clippy::too_many_arguments)]
fn run_march(
    device: &wgpu::Device,
    queue: &wgpu::Queue,
    source: &str,
    entry_point: &str,
    offset_from_clip: Mat4,
    eye: Vec3,
    tree: &Contree,
    width: u32,
    height: u32,
) -> Vec<u8> {
    Prepared::new(
        device, source, entry_point, tree, offset_from_clip, eye, width, height, 0, &[],
    )
    .read_back(device, queue)
}

/// Renders one frame with the given bodies packed after the static world, with
/// body composition and body shadows on. Mirrors `run_march_flagged`, but the
/// flags the loops need are nonnegotiable, so callers only choose geometry and
/// entry point.
#[allow(clippy::too_many_arguments)]
fn run_bodies(
    device: &wgpu::Device,
    queue: &wgpu::Queue,
    source: &str,
    entry_point: &str,
    offset_from_clip: Mat4,
    eye: Vec3,
    world: &Contree,
    bodies: &[Body],
    width: u32,
    height: u32,
) -> Vec<u8> {
    Prepared::new(
        device, source, entry_point, world, offset_from_clip, eye, width, height,
        march_flags::DEFAULT | march_flags::BODIES | march_flags::BODY_SHADOWS, bodies,
    )
    .read_back(device, queue)
}

/// Material of the body-test floor, distinct from `body_cube`'s, so a test can
/// tell body pixels from static ones in `march_identity`.
const FLOOR: MaterialId = MaterialId(2);

/// A static world for body tests: a floor three voxels thick at extent 256.
///
/// Non-empty and a different extent from the body's 64 on purpose: a body
/// marched with the static world's depth and extent would otherwise pass.
/// Three thick, not four, so the floor's bricks do not collapse to uniform
/// nodes and it really owns voxel bytes that a body read from the static
/// world's base would pick up. `extra` adds static voxels on top.
fn body_world(extra: &[(UVec3, MaterialId)]) -> Contree {
    let mut voxels = extra.to_vec();
    for z in 0..256 {
        for x in 0..256 {
            for y in 0..3 {
                voxels.push((UVec3::new(x, y, z), FLOOR));
            }
        }
    }
    let world = Contree::from_voxels(256, &voxels);
    let probe = Body::new(body_cube(), Vec3::ZERO, Quat::IDENTITY);
    let packed = bevox_render::upload::pack_bodies(&world, std::slice::from_ref(&probe));
    assert!(
        packed.bodies[0].voxel_base > 0 && packed.bodies[0].extent != world.extent(),
        "the body world no longer separates the body's layout from the static world's"
    );
    world
}

/// The display entry point shares `traverse` with `march_identity`, but writes
/// colours rather than identities. This proves it renders a scene — distinct
/// sky, floor and column — rather than a uniform field, which is what a
/// silently-skipped dispatch or a missed volume would produce.
#[test]
fn the_display_entry_point_renders_a_scene() {
    let Some((device, queue)) = gpu_device() else {
        eprintln!("no GPU adapter available, skipping");
        return;
    };

    let tree = parity_scene();

    let (width, height) = (64u32, 64u32);
    let eye = Vec3::new(-30.0, 40.0, -30.0);
    let view = Mat4::look_at_rh(Vec3::ZERO, Vec3::new(32.0, 12.0, 32.0) - eye, Vec3::Y);
    let projection = Mat4::perspective_rh(0.9, width as f32 / height as f32, 0.1, 500.0);
    let offset_from_clip = (projection * view).inverse();

    let shader = std::fs::read_to_string("assets/shaders/march.wgsl").expect("shader file missing");
    let pixels = run_march(
        &device,
        &queue,
        &shader,
        "march",
        offset_from_clip,
        eye,
        &tree,
        width,
        height,
    );

    let mut distinct = std::collections::HashSet::new();
    for px in pixels.chunks_exact(4) {
        distinct.insert([px[0], px[1], px[2]]);
    }
    assert!(
        distinct.len() > 2,
        "expected sky, floor and column to differ; got {} distinct colours",
        distinct.len()
    );

    // The sky colour must appear: some rays miss the volume entirely.
    let sky = [(0.35 * 255.0) as u8, (0.47 * 255.0) as u8, (0.70 * 255.0) as u8];
    let near_sky = distinct
        .iter()
        .any(|c| c.iter().zip(sky).all(|(a, b)| a.abs_diff(b) <= 2));
    assert!(near_sky, "no sky-coloured pixel found; every ray hit something");
}

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
    let (width, height) = (64u32, 64u32);
    let eye = Vec3::new(-30.0, 40.0, -30.0);
    let view = Mat4::look_at_rh(Vec3::ZERO, Vec3::new(32.0, 12.0, 32.0) - eye, Vec3::Y);
    let projection = Mat4::perspective_rh(0.9, width as f32 / height as f32, 0.1, 500.0);
    let offset_from_clip = (projection * view).inverse();

    let shader = std::fs::read_to_string("assets/shaders/march.wgsl").expect("shader file missing");
    let pixels = run_march(
        &device,
        &queue,
        &shader,
        "march_voxel_id",
        offset_from_clip,
        eye,
        &tree,
        width,
        height,
    );

    let mut stats = MarchStats::default();
    let mut mismatches = 0usize;
    let mut first = String::new();

    for y in 0..height {
        for x in 0..width {
            let dir = ray_direction(offset_from_clip, x, y, width, height);
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
    let (width, height) = (64u32, 64u32);
    let eye = Vec3::new(-30.0, 40.0, -30.0);
    let view = Mat4::look_at_rh(Vec3::ZERO, Vec3::new(32.0, 12.0, 32.0) - eye, Vec3::Y);
    let projection = Mat4::perspective_rh(0.9, width as f32 / height as f32, 0.1, 500.0);
    let offset_from_clip = (projection * view).inverse();

    let shader = std::fs::read_to_string("assets/shaders/march.wgsl").expect("shader file missing");
    let pixels = run_march(
        &device,
        &queue,
        &shader,
        "march_normal",
        offset_from_clip,
        eye,
        &tree,
        width,
        height,
    );

    let mut stats = MarchStats::default();
    let mut mismatches = 0usize;
    let mut worst = 0.0f32;
    let mut first = String::new();
    let mut compared = 0usize;

    for y in 0..height {
        for x in 0..width {
            let dir = ray_direction(offset_from_clip, x, y, width, height);
            let Some(hit) = march(&tree, Affine3A::IDENTITY, eye, dir, 1000.0, false, &mut stats)
            else {
                continue;
            };
            compared += 1;

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

    assert!(compared > 500, "only {compared} pixels hit geometry; the test is vacuous");
    assert_eq!(mismatches, 0, "{mismatches} normals disagreed (worst {worst}); first {first}");
}

/// Shadow rays start offset along the normal and stop at the first occluder, so
/// they exercise the any-hit path and the self-intersection offset together.
#[test]
fn the_gpu_shadows_match_the_cpu() {
    let Some((device, queue)) = gpu_device() else {
        eprintln!("no GPU adapter available, skipping");
        return;
    };

    let tree = parity_scene();
    let (width, height) = (64u32, 64u32);
    let eye = Vec3::new(-30.0, 40.0, -30.0);
    let view = Mat4::look_at_rh(Vec3::ZERO, Vec3::new(32.0, 12.0, 32.0) - eye, Vec3::Y);
    let projection = Mat4::perspective_rh(0.9, width as f32 / height as f32, 0.1, 500.0);
    let offset_from_clip = (projection * view).inverse();
    let sun = bevox_render::upload::SUN_DIRECTION.normalize();

    let shader = std::fs::read_to_string("assets/shaders/march.wgsl").expect("shader file missing");
    let pixels = run_march(
        &device,
        &queue,
        &shader,
        "march_shadow",
        offset_from_clip,
        eye,
        &tree,
        width,
        height,
    );

    let mut stats = MarchStats::default();
    let mut mismatches = 0usize;
    let mut lit = 0usize;
    let mut shadowed = 0usize;
    let mut first = String::new();

    for y in 0..height {
        for x in 0..width {
            let dir = ray_direction(offset_from_clip, x, y, width, height);
            let Some(hit) = march(&tree, Affine3A::IDENTITY, eye, dir, 1000.0, false, &mut stats)
            else {
                continue;
            };

            // Face normal, matching the shader: a blended corner normal is
            // shorter than the voxel's half-extent on every axis.
            let origin = hit.voxel.as_vec3() + Vec3::splat(0.5) + hit.face_normal * 0.75;
            let cpu_shadowed =
                march(&tree, Affine3A::IDENTITY, origin, sun, 500.0, true, &mut stats).is_some();

            let i = ((y * width + x) * 4) as usize;
            let gpu_shadowed = pixels[i] > 127;

            if cpu_shadowed {
                shadowed += 1;
            } else {
                lit += 1;
            }
            if cpu_shadowed != gpu_shadowed {
                if mismatches == 0 {
                    first = format!("at ({x},{y}) cpu={cpu_shadowed} gpu={gpu_shadowed}");
                }
                mismatches += 1;
            }
        }
    }

    // A run where nothing is shadowed, or everything is, would pass trivially.
    assert!(shadowed > 0, "no shadowed pixels; the scene or sun makes this test vacuous");
    assert!(lit > 0, "every pixel shadowed; the scene or sun makes this test vacuous");
    assert_eq!(mismatches, 0, "{mismatches} shadow decisions disagreed; first {first}");
}

/// A large volume viewed from where the app frames its camera.
///
/// The ray budget used to be a hardcoded 1000 units. At extent 4096 the camera
/// sits ~4500 units out, so every ray died before reaching the geometry and the
/// screen showed nothing but sky — while still reporting a healthy 60 fps,
/// because missing everything is cheap. Frame rate cannot detect this; hit
/// counts can.
#[test]
fn a_distant_camera_on_a_large_volume_still_reaches_the_geometry() {
    let Some((device, queue)) = gpu_device() else {
        eprintln!("no GPU adapter available, skipping");
        return;
    };

    let extent = 4096u32;
    let centre = extent / 2;
    // Large enough to subtend several pixels from the camera distance below:
    // at 4500 units with a 0.9 rad fov, a 64x64 image resolves ~63 units per
    // pixel, so a small block would be sub-pixel and legitimately invisible.
    let half = 64u32;

    // A solid block at the centre of an otherwise empty 4096 volume.
    let mut voxels = Vec::new();
    for z in (centre - half)..(centre + half) {
        for y in (centre - half)..(centre + half) {
            for x in (centre - half)..(centre + half) {
                voxels.push((UVec3::new(x, y, z), MaterialId(1)));
            }
        }
    }
    let tree = Contree::from_voxels(extent, &voxels);

    // Framed exactly as the app frames it.
    let centre_f = Vec3::splat(extent as f32 * 0.5);
    let eye = centre_f + Vec3::new(-1.0, 1.2, -1.0).normalize() * extent as f32 * 1.1;

    let (width, height) = (256u32, 256u32);
    let view = Mat4::look_at_rh(Vec3::ZERO, centre_f - eye, Vec3::Y);
    let projection = Mat4::perspective_rh(0.9, 1.0, 0.1, 20_000.0);
    let offset_from_clip = (projection * view).inverse();

    let shader = std::fs::read_to_string("assets/shaders/march.wgsl").expect("shader file missing");
    let pixels = run_march(
        &device,
        &queue,
        &shader,
        "march_identity",
        offset_from_clip,
        eye,
        &tree,
        width,
        height,
    );

    let hits = pixels.chunks_exact(4).filter(|px| px[1] > 0).count();

    // The CPU reference from the same camera. If it finds the geometry and the
    // GPU does not, the fault is in the shader; if neither does, it is in the
    // tree or the camera.
    let mut stats = MarchStats::default();
    let mut cpu_hits = 0usize;
    for y in 0..height {
        for x in 0..width {
            let dir = ray_direction(offset_from_clip, x, y, width, height);
            if march(&tree, Affine3A::IDENTITY, eye, dir, 100_000.0, false, &mut stats).is_some() {
                cpu_hits += 1;
            }
        }
    }

    assert!(
        cpu_hits > 20,
        "the CPU reference found only {cpu_hits} hits, so the tree or camera is at fault, \
         not the shader (overruns: {})",
        stats.overruns
    );
    assert!(
        hits > 20,
        "GPU reached {hits} pixels but the CPU reached {cpu_hits}; the shader is at fault"
    );
}

/// Flags must reach the shader without disturbing it. With no optimisation
/// implemented yet, setting every bit must change nothing — and this is the
/// shape every later bit-identity test takes, so it is worth having green
/// before anything depends on it.
#[test]
fn the_mask_filter_and_beam_together_leave_output_identical() {
    // MASK_FILTER | BEAM: the one pairing the other two identity tests do not
    // cover between them. Neither touches shadow rays, so this one can stay
    // exact where the beam test has to allow a grazing pixel.
    let Some((device, queue)) = gpu_device() else {
        eprintln!("no GPU adapter available, skipping");
        return;
    };
    let tree = parity_scene();
    let (width, height) = (64u32, 64u32);
    let eye = Vec3::new(-30.0, 40.0, -30.0);
    let view = Mat4::look_at_rh(Vec3::ZERO, Vec3::new(32.0, 12.0, 32.0) - eye, Vec3::Y);
    let projection = Mat4::perspective_rh(0.9, 1.0, 0.1, 500.0);
    let offset_from_clip = (projection * view).inverse();
    let shader = std::fs::read_to_string("assets/shaders/march.wgsl").expect("shader missing");

    for entry in ["march_identity", "march_normal", "march_shadow", "march"] {
        let off = run_march_flagged(
            &device, &queue, &shader, entry, offset_from_clip, eye, &tree, width,
            height, 0,
        );
        let on = run_march_flagged(
            &device, &queue, &shader, entry, offset_from_clip, eye, &tree, width,
            height, 0b110,
        );
        assert_eq!(off, on, "{entry}: the mask filter and beam together changed output");
    }
}

#[test]
fn the_gpu_traversal_agrees_with_the_cpu_reference() {
    let Some((device, queue)) = gpu_device() else {
        eprintln!("no GPU adapter available, skipping");
        return;
    };

    let tree = parity_scene();

    let (width, height) = (64u32, 64u32);
    let eye = Vec3::new(-30.0, 40.0, -30.0);
    let view = Mat4::look_at_rh(Vec3::ZERO, Vec3::new(32.0, 12.0, 32.0) - eye, Vec3::Y);
    let projection = Mat4::perspective_rh(0.9, width as f32 / height as f32, 0.1, 500.0);
    let offset_from_clip = (projection * view).inverse();

    let shader = std::fs::read_to_string("assets/shaders/march.wgsl").expect("shader file missing");
    let gpu_pixels = run_march(
        &device,
        &queue,
        &shader,
        "march_identity",
        offset_from_clip,
        eye,
        &tree,
        width,
        height,
    );

    let mut stats = MarchStats::default();
    let mut mismatches = 0usize;
    let mut first: Option<(u32, u32, String)> = None;

    for y in 0..height {
        for x in 0..width {
            let dir = ray_direction(offset_from_clip, x, y, width, height);
            let cpu = march(&tree, Affine3A::IDENTITY, eye, dir, 1000.0, false, &mut stats);

            let i = ((y * width + x) * 4) as usize;
            let gpu_hit = gpu_pixels[i + 1] > 0;
            let gpu_material = gpu_pixels[i];

            let bad = match cpu {
                Some(hit) => !gpu_hit || gpu_material != hit.material.0,
                None => gpu_hit,
            };
            if bad {
                mismatches += 1;
                if first.is_none() {
                    first = Some((
                        x,
                        y,
                        format!(
                            "cpu={:?} gpu_hit={gpu_hit} gpu_material={gpu_material}",
                            cpu.map(|h| h.material.0)
                        ),
                    ));
                }
            }
        }
    }

    assert_eq!(stats.overruns, 0, "the CPU reference overran its step budget");
    assert_eq!(
        mismatches, 0,
        "{mismatches} of {} pixels disagreed; first at {:?}",
        width * height,
        first
    );
}

const FLAT_SHADER: &str = r#"
@group(0) @binding(0) var output: texture_storage_2d<rgba8unorm, write>;

@compute @workgroup_size(8, 8, 1)
fn main(@builtin(global_invocation_id) id: vec3<u32>) {
    let size = textureDimensions(output);
    if id.x >= size.x || id.y >= size.y { return; }
    textureStore(output, vec2<i32>(id.xy), vec4<f32>(1.0, 0.0, 0.0, 1.0));
}
"#;




/// Runs a shader whose only binding is the output storage texture.
fn run_flat(
    device: &wgpu::Device,
    queue: &wgpu::Queue,
    source: &str,
    width: u32,
    height: u32,
) -> Vec<u8> {
    let module = device.create_shader_module(wgpu::ShaderModuleDescriptor {
        label: Some("test_shader"),
        source: wgpu::ShaderSource::Wgsl(source.into()),
    });

    let texture = storage_target(device, width, height);
    let view = texture.create_view(&wgpu::TextureViewDescriptor::default());

    let pipeline = device.create_compute_pipeline(&wgpu::ComputePipelineDescriptor {
        label: Some("test_pipeline"),
        layout: None,
        module: &module,
        entry_point: Some("main"),
        compilation_options: Default::default(),
        cache: None,
    });

    let bind_group = device.create_bind_group(&wgpu::BindGroupDescriptor {
        label: None,
        layout: &pipeline.get_bind_group_layout(0),
        entries: &[wgpu::BindGroupEntry {
            binding: 0,
            resource: wgpu::BindingResource::TextureView(&view),
        }],
    });

    let mut encoder = device.create_command_encoder(&Default::default());
    {
        let mut pass = encoder.begin_compute_pass(&Default::default());
        pass.set_pipeline(&pipeline);
        pass.set_bind_group(0, &bind_group, &[]);
        pass.dispatch_workgroups(width.div_ceil(8), height.div_ceil(8), 1);
    }

    read_texture(device, queue, encoder, &texture, width, height)
}

#[test]
fn the_harness_can_run_a_trivial_shader() {
    let Some((device, queue)) = gpu_device() else {
        eprintln!("no GPU adapter available, skipping");
        return;
    };
    let pixels = run_flat(&device, &queue, FLAT_SHADER, 16, 16);
    assert_eq!(pixels.len(), 16 * 16 * 4);
    // Every pixel is opaque red.
    for px in pixels.chunks_exact(4) {
        assert_eq!(px, [255, 0, 0, 255], "unexpected pixel");
    }
}

/// The whole point of DDA: fewer child tests, byte-for-byte the same image.
///
/// Stepping emits cells in ray order; the scan emitted them in nearest-entry
/// order. Those agree only while cells are equal sized and disjoint — true
/// within one node, which is why this can be an identity and not a tolerance.
/// The cameras are chosen to attack that: axis-aligned directions put the ray
/// exactly on cell boundaries, where `floor` can pick the neighbour the scan
/// would not have, and a zero direction component makes an exit distance
/// undefined unless it is special-cased.
#[test]
fn dda_leaves_output_bit_identical() {
    let Some((device, queue)) = gpu_device() else {
        eprintln!("no GPU adapter available, skipping");
        return;
    };
    let tree = parity_scene();
    let (width, height) = (96u32, 96u32);
    let shader = std::fs::read_to_string("assets/shaders/march.wgsl").expect("shader missing");

    let cameras = [
        // Oblique: the ordinary case, every direction component non-zero.
        (Vec3::new(-30.0, 40.0, -30.0), Vec3::new(32.0, 12.0, 32.0)),
        // Straight down one axis: two direction components are exactly zero.
        (Vec3::new(32.0, 18.0, -40.0), Vec3::new(32.0, 18.0, 32.0)),
        // Straight down, through the sphere the scene carves out.
        (Vec3::new(32.0, 90.0, 32.0001), Vec3::new(32.0, 0.0, 32.0)),
        // Negative direction on every axis, so the DDA steps backwards.
        (Vec3::new(96.0, 60.0, 96.0), Vec3::new(32.0, 12.0, 32.0)),
        // Inside the volume, grazing the floor: rays start mid-node, so most
        // frames are entered somewhere other than at a corner.
        (Vec3::new(10.0, 7.0, 10.0), Vec3::new(60.0, 7.5, 60.0)),
    ];

    for (i, (eye, target)) in cameras.iter().enumerate() {
        let view = Mat4::look_at_rh(Vec3::ZERO, *target - *eye, Vec3::Y);
        let projection = Mat4::perspective_rh(0.9, 1.0, 0.1, 500.0);
        let offset_from_clip = (projection * view).inverse();

        for entry in ["march_identity", "march_normal", "march_shadow", "march"] {
            let off = run_march_flagged(
                &device, &queue, &shader, entry, offset_from_clip, *eye, &tree, width,
                height, march_flags::NONE,
            );
            let on = run_march_flagged(
                &device, &queue, &shader, entry, offset_from_clip, *eye, &tree, width,
                height, march_flags::DDA,
            );
            let differing = off
                .chunks(4)
                .zip(on.chunks(4))
                .filter(|(a, b)| a != b)
                .count();
            assert_eq!(
                differing, 0,
                "camera {i}, {entry}: DDA changed {differing} of {} pixels",
                width * height
            );
        }
    }
}

/// The filter decides which bricks are entered, never what is found inside one.
///
/// Combinations are tested too: an optimisation can be individually sound and
/// wrong in company, and the app runs them together. The cameras are the same
/// awkward set the DDA test uses, for the same reason — the filter reads the
/// cell a ray enters a child at, so it inherits every boundary case DDA has.
#[test]
fn the_mask_filter_leaves_output_bit_identical() {
    let Some((device, queue)) = gpu_device() else {
        eprintln!("no GPU adapter available, skipping");
        return;
    };
    let tree = parity_scene();
    let (width, height) = (96u32, 96u32);
    let shader = std::fs::read_to_string("assets/shaders/march.wgsl").expect("shader missing");

    let cameras = [
        (Vec3::new(-30.0, 40.0, -30.0), Vec3::new(32.0, 12.0, 32.0)),
        (Vec3::new(32.0, 18.0, -40.0), Vec3::new(32.0, 18.0, 32.0)),
        (Vec3::new(32.0, 90.0, 32.0001), Vec3::new(32.0, 0.0, 32.0)),
        (Vec3::new(96.0, 60.0, 96.0), Vec3::new(32.0, 12.0, 32.0)),
        (Vec3::new(10.0, 7.0, 10.0), Vec3::new(60.0, 7.5, 60.0)),
    ];

    for (i, (eye, target)) in cameras.iter().enumerate() {
        let view = Mat4::look_at_rh(Vec3::ZERO, *target - *eye, Vec3::Y);
        let projection = Mat4::perspective_rh(0.9, 1.0, 0.1, 500.0);
        let offset_from_clip = (projection * view).inverse();

        for entry in ["march_identity", "march_normal", "march_shadow", "march"] {
            let reference = run_march_flagged(
                &device, &queue, &shader, entry, offset_from_clip, *eye, &tree, width,
                height, march_flags::NONE,
            );
            for flags in [
                march_flags::MASK_FILTER,
                march_flags::DDA | march_flags::MASK_FILTER,
            ] {
                let got = run_march_flagged(
                    &device, &queue, &shader, entry, offset_from_clip, *eye, &tree,
                    width, height, flags,
                );
                let differing =
                    reference.chunks(4).zip(got.chunks(4)).filter(|(a, b)| a != b).count();
                assert_eq!(
                    differing, 0,
                    "camera {i}, {entry}, flags {flags:#b}: {differing} of {} pixels differ",
                    width * height
                );
            }
        }
    }
}

/// Thin geometry is what a too-aggressive seed eats first: single-voxel walls
/// and isolated voxels, which can hide between beam samples.
fn thin_scene() -> Contree {
    let extent = 64u32;
    let mut voxels = Vec::new();
    // Single-voxel-thick walls on two axes.
    for a in 0..extent {
        for b in 0..extent {
            voxels.push((UVec3::new(a, b, 32), MaterialId(1)));
            voxels.push((UVec3::new(32, a, b), MaterialId(2)));
        }
    }
    // Isolated voxels scattered off the walls.
    let mut rng = bevox_core::testing::XorShift64::new(4242);
    for _ in 0..200 {
        voxels.push((
            UVec3::new(rng.next_below(extent), rng.next_below(extent), rng.next_below(extent)),
            MaterialId(1),
        ));
    }
    Contree::from_voxels(extent, &voxels)
}

/// A beam prepass seeds full-resolution rays with a distance a coarse pass
/// proved empty. Seeding even slightly too far skips thin geometry, and the
/// symptom is holes that appear from some angles and not others — so this
/// sweeps angles rather than trusting one view.
///
/// The combination is tested alongside it because that is what ships: the seed
/// changes where a ray starts, and both other optimisations change how it walks
/// from there.
///
/// Primary-ray output is exact. Shaded output is allowed a couple of pixels,
/// and only because of one characterised difference: the scan's slab test is
/// inclusive, so a ray touching a cell at exactly one point still descends into
/// it, while a walk can only visit cells it passes through. Shadow rays all
/// share one direction, so grid-locked origins reach those exact boundaries
/// systematically where an arbitrary camera direction never does. It is a
/// convention difference at a zero-measure graze, not lost geometry — which is
/// what `dda_matches_the_scan_from_inside_the_volume` exists to show.
#[test]
fn the_beam_prepass_never_skips_geometry() {
    let Some((device, queue)) = gpu_device() else {
        eprintln!("no GPU adapter available, skipping");
        return;
    };
    let tree = thin_scene();
    let (width, height) = (128u32, 128u32);
    let centre = Vec3::splat(32.0);
    let shader = std::fs::read_to_string("assets/shaders/march.wgsl").expect("shader missing");

    for step in 0..8u32 {
        let angle = step as f32 * std::f32::consts::TAU / 8.0;
        let eye = centre + Vec3::new(angle.cos() * 90.0, 30.0, angle.sin() * 90.0);
        let view = Mat4::look_at_rh(Vec3::ZERO, centre - eye, Vec3::Y);
        let projection = Mat4::perspective_rh(0.9, 1.0, 0.1, 500.0);
        let offset_from_clip = (projection * view).inverse();

        for entry in ["march_identity", "march_voxel_id", "march_normal", "march_shadow", "march"]
        {
            let reference = run_march_flagged(
                &device, &queue, &shader, entry, offset_from_clip, eye, &tree, width,
                height, march_flags::NONE,
            );
            for flags in [
                march_flags::BEAM,
                march_flags::DDA | march_flags::MASK_FILTER | march_flags::BEAM,
            ] {
                let seeded = run_march_flagged(
                    &device, &queue, &shader, entry, offset_from_clip, eye, &tree,
                    width, height, flags,
                );
                let differing =
                    reference.chunks(4).zip(seeded.chunks(4)).filter(|(a, b)| a != b).count();
                // Shadow rays are the only place the graze convention shows.
                let allowed = if entry == "march" || entry == "march_shadow" { 4 } else { 0 };
                assert!(
                    differing <= allowed,
                    "angle {step}, {entry}, flags {flags:#b}: {differing} of {} pixels differ,                      more than the {allowed} a grazing shadow ray explains",
                    width * height
                );
            }
        }
    }
}

/// A floor and a few columns at extent 256, leaving large open volumes.
///
/// The distance field is coarse -- one cell per 16 voxels -- so a scene with
/// scattered isolated voxels saturates it to zero and the skip never runs.
/// This one has genuine emptiness for the field to find.
fn open_scene() -> Contree {
    let extent = 256u32;
    let mut voxels = Vec::new();
    for z in 0..extent {
        for x in 0..extent {
            for y in 0..4 {
                voxels.push((UVec3::new(x, y, z), MaterialId(1)));
            }
        }
    }
    for (cx, cz) in [(60u32, 60u32), (180, 70), (100, 190)] {
        for y in 4..90 {
            for dz in 0..24 {
                for dx in 0..24 {
                    voxels.push((UVec3::new(cx + dx, y, cz + dz), MaterialId(2)));
                }
            }
        }
    }
    Contree::from_voxels(extent, &voxels)
}

/// Skipping empty space must not change what is hit.
///
/// The field promises a cube of emptiness around each cell; a ray that jumps
/// further than the promise passes through geometry, and the symptom is holes
/// that open from some angles and not others. This sweeps angles over a scene
/// with large open volumes -- a scene of only scattered single voxels
/// saturates the coarse field to zero everywhere and would exercise none of
/// the jump arithmetic, which is why the field-wide non-vacuity guard below
/// exists.
///
/// That guard only proves the field has large values *somewhere*; it says
/// nothing about the cell each camera actually starts in. The cameras used to
/// orbit at radius 360 around a scene of extent 256 -- entirely outside the
/// volume, where `field_at` reads 0 for every cell -- so `skip_empty_space`
/// returned on iteration 0 for every ray and none of the jump arithmetic ever
/// ran, while the field-wide guard stayed green throughout. The per-camera
/// `at_eye >= 2` assert below is what actually catches that: it fails the
/// moment a camera's own starting cell cannot produce a jump.
#[test]
fn the_distance_field_leaves_output_bit_identical() {
    let Some((device, queue)) = gpu_device() else {
        eprintln!("no GPU adapter available, skipping");
        return;
    };
    let tree = open_scene();
    let field = bevox_core::distance_field::DistanceField::build(&tree);
    let max_claimed = field.cells().iter().copied().max().unwrap_or(0);
    assert!(
        max_claimed >= 4,
        "the field's largest value is {max_claimed}, so skip_empty_space returns on its \
         first iteration and this test exercises none of the jump arithmetic"
    );
    eprintln!("distance field max claimed: {max_claimed}");
    let (width, height) = (96u32, 96u32);
    // A point in the open space open_scene() provides: above the floor
    // (y 0..4), below every column top (y 90), and clear of all three
    // 24-wide columns. The cameras orbit this point instead of the scene's
    // outer edge, so a ray actually starts inside the volume the field
    // covers.
    let centre = Vec3::new(128.0, 40.0, 128.0);
    // Radius 32, with the ring rotated 20 degrees off the compass
    // directions. The three columns sit at roughly 45-degree intervals
    // around this point, so an unrotated ring of 8 (0, 45, 90, ...) aims two
    // of the eight cameras straight down a column wall, where the per-camera
    // guard below fails. Found by scanning radius and phase for one that
    // keeps every camera's own cell reading at least 2 -- 22.5 degrees, the
    // midpoint of the safe window, also clears that guard but then lands one
    // ray exactly on a voxel boundary the field-skip's origin nudge resolves
    // a ULP to the other side of from the unskipped scan (a one-cell graze
    // in march_voxel_id, not a lost hit), so this is 20 rather than the
    // midpoint. Confirmed clean -- zero differing pixels across every angle,
    // entry point and flag combination -- before committing to it.
    let radius = 32.0f32;
    let phase = 20.0f32.to_radians();
    let shader = std::fs::read_to_string("assets/shaders/march.wgsl").expect("shader missing");

    for step in 0..8u32 {
        let angle = phase + step as f32 * std::f32::consts::TAU / 8.0;
        let eye = centre + Vec3::new(angle.cos() * radius, 0.0, angle.sin() * radius);

        let eye_cell = (eye / bevox_core::distance_field::CELL_VOXELS as f32).as_uvec3();
        let at_eye = field.get(eye_cell);
        assert!(
            at_eye >= 2,
            "angle {step}: the field reads {at_eye} at the camera's own cell, so \
             skip_empty_space returns on its first iteration and this test exercises \
             none of the jump arithmetic"
        );

        let view = Mat4::look_at_rh(Vec3::ZERO, centre - eye, Vec3::Y);
        let projection = Mat4::perspective_rh(0.9, 1.0, 0.1, 500.0);
        let offset_from_clip = (projection * view).inverse();

        for entry in ["march_identity", "march_voxel_id", "march_normal", "march"] {
            let reference = run_march_flagged(
                &device, &queue, &shader, entry, offset_from_clip, eye, &tree, width,
                height, march_flags::NONE,
            );
            for flags in [
                march_flags::DISTANCE_FIELD,
                march_flags::DEFAULT | march_flags::DISTANCE_FIELD,
            ] {
                let got = run_march_flagged(
                    &device, &queue, &shader, entry, offset_from_clip, eye, &tree,
                    width, height, flags,
                );
                let differing =
                    reference.chunks(4).zip(got.chunks(4)).filter(|(a, b)| a != b).count();
                // Shaded output is allowed the documented grazing-shadow pixels.
                let allowed = if entry == "march" { 4 } else { 0 };
                assert!(
                    differing <= allowed,
                    "angle {step}, {entry}, flags {flags:#b}: {differing} of {} pixels differ",
                    width * height
                );
            }
        }
    }
}

/// Rays that start inside the volume, among voxels with nothing adjacent.
///
/// A shadow ray is a primary ray with its origin on a voxel surface, and that
/// is the one input the outside-the-volume cameras never produce. An isolated
/// voxel is the hardest thing for a walk to catch: it occupies one cell of one
/// brick, and missing the cell means missing the voxel entirely.
#[test]
fn dda_matches_the_scan_from_inside_the_volume() {
    let Some((device, queue)) = gpu_device() else {
        eprintln!("no GPU adapter available, skipping");
        return;
    };
    let mut voxels = Vec::new();
    let mut rng = bevox_core::testing::XorShift64::new(99);
    for _ in 0..400 {
        voxels.push((
            UVec3::new(rng.next_below(64), rng.next_below(64), rng.next_below(64)),
            MaterialId(1),
        ));
    }
    let tree = Contree::from_voxels(64, &voxels);
    let (width, height) = (128u32, 128u32);
    let shader = std::fs::read_to_string("assets/shaders/march.wgsl").expect("shader missing");

    // Origins offset from voxel centres the way a shadow ray's is.
    let eyes = [
        Vec3::new(32.5, 32.5, 31.75),
        Vec3::new(20.5, 8.5, 44.25),
        Vec3::new(5.25, 40.5, 40.5),
    ];
    let mut total = 0usize;
    for (i, eye) in eyes.iter().enumerate() {
        for step in 0..4u32 {
            let angle = step as f32 * std::f32::consts::TAU / 4.0;
            let target = *eye + Vec3::new(angle.cos(), 0.6, angle.sin()) * 20.0;
            let view = Mat4::look_at_rh(Vec3::ZERO, target - *eye, Vec3::Y);
            let projection = Mat4::perspective_rh(0.9, 1.0, 0.1, 500.0);
            let offset_from_clip = (projection * view).inverse();
            for entry in ["march_identity", "march_voxel_id"] {
                let scan = run_march_flagged(
                    &device, &queue, &shader, entry, offset_from_clip, *eye, &tree,
                    width, height, march_flags::NONE,
                );
                let dda = run_march_flagged(
                    &device, &queue, &shader, entry, offset_from_clip, *eye, &tree,
                    width, height, march_flags::DDA,
                );
                let differing =
                    scan.chunks(4).zip(dda.chunks(4)).filter(|(a, b)| a != b).count();
                if differing != 0 {
                    println!("eye {i} step {step} {entry}: {differing} differ");
                }
                total += differing;
            }
        }
    }
    assert_eq!(total, 0, "DDA lost {total} pixels among isolated voxels");
}

/// An edited scene uploaded by ranges must render exactly like the same scene
/// uploaded whole.
///
/// This is the milestone's gate. A partial upload that misses a range produces
/// a scene that is *nearly* right, which is the hardest kind of wrong to see by
/// eye -- so it is checked pixel for pixel rather than looked at. Several edits
/// run in sequence because the second one rewrites nodes the first allocated,
/// which is where a stale offset shows up.
#[test]
fn an_incrementally_uploaded_edit_renders_identically() {
    let Some((device, queue)) = gpu_device() else {
        eprintln!("no GPU adapter available, skipping");
        return;
    };
    let shader = std::fs::read_to_string("assets/shaders/march.wgsl").expect("shader missing");
    let (width, height) = (96u32, 96u32);
    let eye = Vec3::new(-30.0, 40.0, -30.0);
    let view = Mat4::look_at_rh(Vec3::ZERO, Vec3::new(32.0, 12.0, 32.0) - eye, Vec3::Y);
    let projection = Mat4::perspective_rh(0.9, 1.0, 0.1, 500.0);
    let offset_from_clip = (projection * view).inverse();

    let edits = [
        (Vec3::new(32.0, 20.0, 32.0), 7.0, MaterialId(2)),
        (Vec3::new(20.0, 10.0, 40.0), 5.0, MaterialId(3)),
        // Erasing is the case that frees nodes rather than allocating them.
        (Vec3::new(32.0, 20.0, 32.0), 4.0, MaterialId::EMPTY),
        (Vec3::new(45.0, 14.0, 20.0), 9.0, MaterialId(1)),
    ];

    // One tree is edited and uploaded incrementally; the other is edited the
    // same way and uploaded from scratch each time. Both go through
    // `apply_brush`, not `tree.apply_sphere` directly -- that is the only
    // path that lowers the field and marks `field_dirty`, and staying off it
    // is exactly what let this test pass with `update.field` wired up to
    // nothing at all: painting a tree via `apply_sphere` alone never dirties
    // a field range, so `stage_scene_update` always staged an empty
    // `update.field`, and this test could not have told a correct write from
    // no write.
    let tree = parity_scene();
    let field = bevox_core::distance_field::DistanceField::build(&tree);
    let mut incremental = VoxelScene {
        tree,
        materials: parity_materials(),
        generation: 1,
        field,
        field_dirty: None,
        bodies: Vec::new(),
    };
    incremental.tree.arena_mut().clear_dirty();

    let whole_tree = parity_scene();
    let whole_field = bevox_core::distance_field::DistanceField::build(&whole_tree);
    let mut whole = VoxelScene {
        tree: whole_tree,
        materials: parity_materials(),
        generation: 1,
        field: whole_field,
        field_dirty: None,
        bodies: Vec::new(),
    };

    let volume = GpuVolume::from_contree(&incremental.tree);
    let prepared = Prepared::new(
        &device, &shader, "march_identity", &incremental.tree, offset_from_clip, eye,
        width, height, march_flags::DEFAULT, &[],
    );

    for (i, (centre, radius, material)) in edits.iter().enumerate() {
        bevox_render::upload::apply_brush(&mut incremental, *centre, *radius, *material);
        bevox_render::upload::apply_brush(&mut whole, *centre, *radius, *material);

        let update = bevox_render::upload::stage_scene_update(&mut incremental);
        assert!(
            update.node_high_water < bevox_render::pipeline::buffer_capacity_for(
                volume.nodes.len() as u32
            ),
            "edit {i} outgrew the buffers; the test needs a bigger starting capacity"
        );
        prepared.apply_update(&queue, &update);
        let got = prepared.read_back(&device, &queue);

        let reference = run_march_flagged(
            &device, &queue, &shader, "march_identity", offset_from_clip, eye, &whole.tree,
            width, height, march_flags::DEFAULT,
        );

        let differing = reference.chunks(4).zip(got.chunks(4)).filter(|(a, b)| a != b).count();
        assert_eq!(
            differing, 0,
            "after edit {i}, {differing} of {} pixels differ between the incremental \
             upload and a full one",
            width * height
        );
    }
}

/// A scene with no bodies must render the same with body composition on as off.
///
/// Both sides run the same shader, so this proves one narrow thing: the
/// composition loop, at a body count of zero, changes no pixel of any entry
/// point. It cannot see a change both sides share -- the `traverse` wrapper
/// over `traverse_at`, or `shadow_origin` factored out of the shading entry
/// points -- because the flag-off side runs that code too. That a body-free
/// scene still matches the shader from before bodies existed was checked once,
/// in the final branch review, by rendering against that shader; nothing in
/// the suite re-checks it.
#[test]
fn a_scene_with_no_bodies_is_bit_identical_with_bodies_enabled() {
    let Some((device, queue)) = gpu_device() else {
        eprintln!("no GPU adapter available, skipping");
        return;
    };
    let tree = parity_scene();
    let (width, height) = (96u32, 96u32);
    let eye = Vec3::new(-30.0, 40.0, -30.0);
    let view = Mat4::look_at_rh(Vec3::ZERO, Vec3::new(32.0, 12.0, 32.0) - eye, Vec3::Y);
    let projection = Mat4::perspective_rh(0.9, 1.0, 0.1, 500.0);
    let offset_from_clip = (projection * view).inverse();
    let shader = std::fs::read_to_string("assets/shaders/march.wgsl").expect("shader missing");

    for entry in ["march_identity", "march_voxel_id", "march_normal", "march_shadow", "march"] {
        // Masked out explicitly: `DEFAULT` carries `BODIES`, so comparing it
        // against `DEFAULT | BODIES` would compare a build with itself.
        let without = run_march_flagged(
            &device, &queue, &shader, entry, offset_from_clip, eye, &tree, width,
            height, march_flags::DEFAULT & !march_flags::BODIES,
        );
        // With the rectangles and body shadows on too: both loops run zero
        // times, and the buffers they would read are zeroed entries.
        let with = run_march_flagged(
            &device, &queue, &shader, entry, offset_from_clip, eye, &tree, width,
            height,
            march_flags::DEFAULT | march_flags::BODIES | march_flags::BODY_RECT
                | march_flags::BODY_SHADOWS,
        );
        let differing = without.chunks(4).zip(with.chunks(4)).filter(|(a, b)| a != b).count();
        assert_eq!(
            differing, 0,
            "{entry}: enabling bodies changed {differing} pixels of a scene that has none"
        );
    }
}

/// A body must actually appear, and appear where its transform puts it.
///
/// The zero-body test above passes trivially if the loop never runs. This is
/// the one that proves it does, and that placement has the right sense: images
/// that merely differ would also pass with the body mirrored, axis-swapped or
/// read from a garbage base, so the body's pixels are counted per image half.
#[test]
fn a_placed_body_appears_where_its_transform_puts_it() {
    let Some((device, queue)) = gpu_device() else {
        eprintln!("no GPU adapter available, skipping");
        return;
    };
    let (width, height) = (96u32, 96u32);
    let eye = Vec3::new(32.0, 32.0, -60.0);
    let view = Mat4::look_at_rh(Vec3::ZERO, Vec3::splat(32.0) - eye, Vec3::Y);
    let projection = Mat4::perspective_rh(0.9, 1.0, 0.1, 500.0);
    let offset_from_clip = (projection * view).inverse();
    let shader = std::fs::read_to_string("assets/shaders/march.wgsl").expect("shader missing");

    // Every ray that reaches the cube (y 25..41, seen from y 32) stays far above
    // the floor (y 0..3), so the floor shows at the bottom of the frame without
    // ever standing in front of the body.
    let world = body_world(&[]);
    let cube = body_cube();

    // Body pixels in the [left, right] image halves. `march_identity` writes the
    // material in red and the hit flag in green; the floor is another material.
    let halves = |bodies: &[Body]| {
        let pixels = run_bodies(
            &device, &queue, &shader, "march_identity", offset_from_clip, eye, &world, bodies,
            width, height,
        );
        let mut sides = [0usize; 2];
        for (i, px) in pixels.chunks_exact(4).enumerate() {
            if px[1] > 0 && px[0] == 1 {
                sides[usize::from(i as u32 % width >= width / 2)] += 1;
            }
        }
        sides
    };

    // look_at_rh looking down +Z with Y up gives a camera right of
    // forward x up = (0,0,1) x (0,1,0) = -X, and pixel x grows to the right.
    // So a body shifted toward world -X lands in the right half of the image.
    let [empty_left, empty_right] = halves(&[]);
    let [neg_left, neg_right] =
        halves(&[Body::new(cube.clone(), Vec3::new(-12.0, 0.0, 0.0), Quat::IDENTITY)]);
    let [pos_left, pos_right] =
        halves(&[Body::new(cube, Vec3::new(12.0, 0.0, 0.0), Quat::IDENTITY)]);

    assert_eq!(empty_left + empty_right, 0, "body pixels were drawn with no body placed");
    for (side, count) in [("-X", neg_left + neg_right), ("+X", pos_left + pos_right)] {
        assert!(count > 100, "a body was placed at {side} but only {count} pixels drew it");
    }
    assert!(
        neg_right > neg_left,
        "a body at world -X belongs in the right half; left {neg_left}, right {neg_right}"
    );
    assert!(
        pos_left > pos_right,
        "a body at world +X belongs in the left half; left {pos_left}, right {pos_right}"
    );
}

/// A rotated body's normal must come back through its rotation, not its inverse
/// and not unrotated.
///
/// Both placements above are unrotated, so the packed rotation is the identity
/// and dropping it, or packing the inverse in its place, renders the same image.
/// A non-identity orientation checked per pixel against the CPU reference is
/// what pins it.
///
/// The camera also sits inside the static world, in open space the distance
/// field skips, with the body nearer than that skip. A body march started from
/// the static world's seeded origin instead of the camera loses the body here.
///
/// And the static world holds a decoy at the body's *local* voxel coordinates,
/// out of view behind the camera: planes x = 0 mod 3 through (24..42)^3. A body
/// hit shaded by probing the static tree at its local voxel reads those planes
/// and gets a +-X normal; a shadow ray started from its local voxel starts
/// among them and is shadowed. Without the decoy both probes read empty space
/// and fall back to the right answer, which is how that bug hid before.
///
/// And a static wall stands between the camera and the body's world +X side,
/// the image's left. Composition keeps the nearest `t`, so there the wall must
/// show, though the body lies behind it. A body march not bounded by the world
/// hit draws the body through the wall, and no other test would notice: every
/// other body test has nothing between the camera and the body.
#[test]
fn a_rotated_body_is_shaded_with_its_rotated_normal() {
    let Some((device, queue)) = gpu_device() else {
        eprintln!("no GPU adapter available, skipping");
        return;
    };
    let (width, height) = (96u32, 96u32);
    // Field cell 5 on z, two cells short of the wall's cell 7, so the camera's
    // own cell still reads as open space.
    let eye = Vec3::new(100.0, 60.0, 88.0);
    let centre = Vec3::new(100.0, 60.0, 140.0);
    let view = Mat4::look_at_rh(Vec3::ZERO, centre - eye, Vec3::Y);
    let projection = Mat4::perspective_rh(0.9, 1.0, 0.1, 500.0);
    let offset_from_clip = (projection * view).inverse();
    let shader = std::fs::read_to_string("assets/shaders/march.wgsl").expect("shader missing");

    let mut decoy = Vec::new();
    for z in 24..42 {
        for y in 24..42 {
            for x in (24..42).filter(|x| x % 3 == 0) {
                decoy.push((UVec3::new(x, y, z), FLOOR));
            }
        }
    }
    // In front of the body, whose nearest corner is at z ~127, over world
    // x >= 100 only: half the body is hidden and half is not.
    let mut wall = Vec::new();
    for z in 116..119 {
        for y in 40..80 {
            for x in 100..130 {
                wall.push((UVec3::new(x, y, z), FLOOR));
            }
        }
    }
    let world = body_world(&[decoy, wall].concat());
    let field = bevox_core::distance_field::DistanceField::build(&world);
    let at_eye = field.get((eye / bevox_core::distance_field::CELL_VOXELS as f32).as_uvec3());
    assert!(
        at_eye >= 2,
        "the field reads {at_eye} at the camera, so nothing is skipped and a seeded body march \
         would pass here"
    );

    // Turned about the cube's own centre (33 in its volume), which is placed at
    // `centre`, so the whole cube stays in view.
    let orientation = Quat::from_euler(glam::EulerRot::XYZ, 0.4, 0.8, -0.3);
    let body = Body::new(body_cube(), centre - orientation * Vec3::splat(33.0), orientation);

    let render = |entry: &str| {
        run_bodies(
            &device, &queue, &shader, entry, offset_from_clip, eye, &world,
            std::slice::from_ref(&body), width, height,
        )
    };
    let pixels = render("march_normal");
    let shadows = render("march_shadow");
    let sun = bevox_render::upload::SUN_DIRECTION.normalize();

    // Every pixel, both ways: a pixel the CPU says hits nothing must miss on the
    // GPU too, and one where the static world is nearer must carry its normal,
    // not a body's. Checking only body pixels would let extra GPU hits through.
    let mut stats = MarchStats::default();
    let mut body_pixels = 0usize;
    // Pixels where the body is on the ray but the static world is nearer, and
    // how many of those the GPU got wrong.
    let mut occluded = 0usize;
    let mut occluded_bad = 0usize;
    let mut mismatches = 0usize;
    let mut worst = 0.0f32;
    let mut first = String::new();
    for y in 0..height {
        for x in 0..width {
            let dir = ray_direction(offset_from_clip, x, y, width, height);
            let i = ((y * width + x) * 4) as usize;
            let gpu_hit = pixels[i + 3] > 0;
            let got = Vec3::new(
                pixels[i] as f32 / 255.0 * 2.0 - 1.0,
                pixels[i + 1] as f32 / 255.0 * 2.0 - 1.0,
                pixels[i + 2] as f32 / 255.0 * 2.0 - 1.0,
            );

            let mut behind = false;
            let (expected, cpu_shadowed, gpu_shadowed) =
                match cpu_composed(&world, &body, eye, dir, &mut stats) {
                    None => (None, false, false),
                    Some((hit, false)) => {
                        behind = body.march_world(eye, dir, 1000.0, &mut stats).is_some();
                        let n =
                            bevox_core::normal::implicit_normal(&world, hit.voxel, hit.face_normal);
                        (Some(n), false, false)
                    }
                    Some((hit, true)) => {
                        body_pixels += 1;
                        // The CPU hit's face normal is in the body's frame.
                        let n = orientation * hit.face_normal;
                        // From the world-space hit point lifted 0.25 off the
                        // surface, toward the static world and the body itself:
                        // a face turned from the sun is shadowed by its own body.
                        let origin = eye + dir * hit.t + n * 0.25;
                        let cpu_shadowed = cpu_shadowed(&world, std::slice::from_ref(&body), origin, sun, &mut stats);
                        (Some(n), cpu_shadowed, shadows[i] > 127)
                    }
                };

            // One byte per channel quantises the decoded normal to steps of
            // 2/255, so a correct normal is off by at most half of that.
            let bad = match expected {
                None => gpu_hit,
                Some(n) => {
                    let delta = (got - n).abs().max_element();
                    worst = worst.max(delta);
                    !gpu_hit || delta > 2.0 / 255.0 + 1e-4 || cpu_shadowed != gpu_shadowed
                }
            };
            if bad {
                if mismatches == 0 {
                    first = format!(
                        "at ({x},{y}) expected={expected:?} gpu_hit={gpu_hit} gpu={got:?} \
                         cpu_shadowed={cpu_shadowed} gpu_shadowed={gpu_shadowed}"
                    );
                }
                mismatches += 1;
            }
            if behind {
                occluded += 1;
                occluded_bad += usize::from(bad);
            }
        }
    }

    eprintln!(
        "{body_pixels} body pixels, {occluded} with the wall in front of the body \
         ({occluded_bad} wrong), {mismatches} mismatches in all"
    );
    assert!(body_pixels > 200, "only {body_pixels} pixels hit the body; the test is vacuous");
    assert!(
        occluded > 200,
        "only {occluded} pixels have the static world in front of the body, so nothing tests \
         that the nearer hit wins"
    );
    assert_eq!(
        mismatches, 0,
        "{mismatches} of {} pixels disagreed ({body_pixels} on the body; {occluded_bad} of \
         {occluded} where the wall is in front of the body; worst channel {worst}); first {first}",
        width * height
    );
}

/// What one pixel should show: the nearer of the static world's hit and the
/// body's, and whether the body won. The body must be strictly nearer, as in
/// `compose_bodies`.
fn cpu_composed(
    world: &Contree,
    body: &Body,
    eye: Vec3,
    dir: Vec3,
    stats: &mut MarchStats,
) -> Option<(bevox_core::march::Hit, bool)> {
    let static_hit = march(world, Affine3A::IDENTITY, eye, dir, 1000.0, false, stats);
    match body.march_world(eye, dir, 1000.0, stats) {
        Some(b) if static_hit.is_none_or(|s| b.t < s.t) => Some((b, true)),
        _ => static_hit.map(|s| (s, false)),
    }
}

/// Whether the ray toward the sun from `origin` is blocked, as `shadowed` in the
/// shader decides it: the static world, then every body.
fn cpu_shadowed(world: &Contree, bodies: &[Body], origin: Vec3, sun: Vec3, stats: &mut MarchStats) -> bool {
    march(world, Affine3A::IDENTITY, origin, sun, 500.0, true, stats).is_some()
        || bodies.iter().any(|b| b.march_world(origin, sun, 1000.0, stats).is_some())
}

/// The nearest of the static world's hit and every body's, and which body won,
/// if one did. A body must be strictly nearer, as in `compose_bodies`.
fn cpu_composed_all(
    world: &Contree,
    bodies: &[Body],
    eye: Vec3,
    dir: Vec3,
    stats: &mut MarchStats,
) -> Option<(bevox_core::march::Hit, Option<usize>)> {
    let mut best = march(world, Affine3A::IDENTITY, eye, dir, 1000.0, false, stats).map(|h| (h, None));
    for (i, body) in bodies.iter().enumerate() {
        if let Some(h) = body.march_world(eye, dir, 1000.0, stats)
            && best.as_ref().is_none_or(|(b, _)| h.t < b.t)
        {
            best = Some((h, Some(i)));
        }
    }
    best
}

/// An L in a 64 volume: a slab, and a wall standing on its +X edge. With the sun
/// toward +X, the wall shadows the slab's top: a body shadowing itself on a
/// face turned toward the sun.
fn l_body() -> Contree {
    let mut dense = DenseVolume::new(64).unwrap();
    for z in 20..44 {
        for x in 20..44 {
            for y in 20..28 {
                dense.set(UVec3::new(x, y, z), MaterialId(1));
            }
        }
        for x in 36..44 {
            for y in 28..44 {
                dense.set(UVec3::new(x, y, z), MaterialId(1));
            }
        }
    }
    dense.into_contree()
}

/// Bodies cast shadows on the world, on each other and on themselves, and the
/// GPU agrees with the CPU on every pixel.
///
/// A turned L floats over a cube, which stands over the floor. The reference
/// marches, from the same origin as the shader, the static world and then every
/// body. Each kind of shadow must actually occur, or the agreement proves
/// nothing: floor shadowed only by a body, one body shadowed by the other, and
/// the L shadowed by its own wall on a face turned toward the sun.
#[test]
fn bodies_cast_shadows_that_match_the_cpu() {
    let Some((device, queue)) = gpu_device() else {
        eprintln!("no GPU adapter available, skipping");
        return;
    };
    let (width, height) = (128u32, 128u32);
    // From the side the shadows fall toward, -X -Z, so they are not hidden
    // behind the bodies that cast them.
    let eye = Vec3::new(40.0, 160.0, 50.0);
    let view = Mat4::look_at_rh(Vec3::ZERO, Vec3::new(100.0, 20.0, 100.0) - eye, Vec3::Y);
    let projection = Mat4::perspective_rh(0.9, 1.0, 0.1, 500.0);
    let offset_from_clip = (projection * view).inverse();
    let shader = std::fs::read_to_string("assets/shaders/march.wgsl").expect("shader missing");
    let world = body_world(&[]);
    let sun = bevox_render::upload::SUN_DIRECTION.normalize();

    let turn = Quat::from_rotation_y(0.3);
    let l = Body::new(l_body(), Vec3::new(112.0, 60.0, 112.0) - turn * Vec3::splat(32.0), turn);
    // Under the L, where its shadow falls: the cube spans 25..41 of its volume.
    let cube = Body::new(body_cube(), Vec3::new(104.0, 20.0, 106.0) - Vec3::splat(33.0), Quat::IDENTITY);
    let bodies = [l, cube];

    let pixels = run_bodies(
        &device, &queue, &shader, "march_shadow", offset_from_clip, eye, &world, &bodies, width, height,
    );

    let mut stats = MarchStats::default();
    let (mut floor_by_body, mut by_other, mut by_itself, mut mismatches) = (0usize, 0usize, 0usize, 0usize);
    let mut first = String::new();
    for y in 0..height {
        for x in 0..width {
            let dir = ray_direction(offset_from_clip, x, y, width, height);
            let i = ((y * width + x) * 4) as usize;
            let (gpu_hit, gpu_shadowed) = (pixels[i + 3] > 0, pixels[i] > 127);
            let Some((hit, owner)) = cpu_composed_all(&world, &bodies, eye, dir, &mut stats) else {
                if gpu_hit {
                    mismatches += 1;
                }
                continue;
            };
            let (origin, normal) = match owner {
                None => (hit.voxel.as_vec3() + Vec3::splat(0.5) + hit.face_normal * 0.75, hit.face_normal),
                Some(k) => {
                    let n = bodies[k].orientation * hit.face_normal;
                    (eye + dir * hit.t + n * 0.25, n)
                }
            };
            let expected = cpu_shadowed(&world, &bodies, origin, sun, &mut stats);
            let by_world = march(&world, Affine3A::IDENTITY, origin, sun, 500.0, true, &mut stats).is_some();
            let blockers: Vec<usize> = (0..bodies.len())
                .filter(|&j| bodies[j].march_world(origin, sun, 1000.0, &mut stats).is_some())
                .collect();
            match owner {
                None if !by_world && !blockers.is_empty() => floor_by_body += 1,
                Some(k) if blockers.iter().any(|&j| j != k) => by_other += 1,
                Some(k) if blockers.contains(&k) && normal.dot(sun) > 0.0 && !by_world => by_itself += 1,
                _ => {}
            }
            if !gpu_hit || gpu_shadowed != expected {
                if mismatches == 0 {
                    first = format!("at ({x},{y}) owner={owner:?} cpu={expected} gpu_hit={gpu_hit} gpu={gpu_shadowed}");
                }
                mismatches += 1;
            }
        }
    }
    eprintln!("{floor_by_body} floor pixels shadowed by a body, {by_other} by the other body, {by_itself} by itself; {mismatches} mismatches");
    assert!(floor_by_body > 100, "only {floor_by_body} floor pixels are shadowed by a body; the test is vacuous");
    assert!(by_other > 20, "only {by_other} pixels of one body are shadowed by the other");
    assert!(by_itself > 20, "only {by_itself} sunward pixels of a body are shadowed by itself");
    assert_eq!(mismatches, 0, "{mismatches} pixels disagreed; first {first}");
}

/// The camera and body for the off-screen caster tests: a cube high above and
/// behind the camera's view, whose shadow lands on the floor in the middle of
/// it.
fn off_screen_caster() -> (Vec3, Mat4, Body) {
    let eye = Vec3::new(100.0, 40.0, 140.0);
    let view = Mat4::look_at_rh(Vec3::ZERO, Vec3::new(100.0, 0.0, 100.0) - eye, Vec3::Y);
    let projection = Mat4::perspective_rh(0.9, 1.0, 0.1, 500.0);
    let offset_from_clip = (projection * view).inverse();
    // A hundred voxels up the sun's direction from the floor the camera faces.
    let above = Vec3::new(100.0, 3.0, 100.0) + bevox_render::upload::SUN_DIRECTION.normalize() * 100.0;
    let body = Body::new(body_cube(), above - Vec3::splat(33.0), Quat::IDENTITY);
    (eye, offset_from_clip, body)
}

/// A body the camera cannot see still shadows what it can.
///
/// The cull must drop the body, or this tests nothing; then more of the floor
/// is shadowed with the body than without it.
#[test]
fn an_off_screen_body_shadows_what_is_on_screen() {
    let Some((device, queue)) = gpu_device() else {
        eprintln!("no GPU adapter available, skipping");
        return;
    };
    let (width, height) = (96u32, 96u32);
    let (eye, offset_from_clip, body) = off_screen_caster();
    let shader = std::fs::read_to_string("assets/shaders/march.wgsl").expect("shader missing");
    let world = body_world(&[]);
    let flags = march_flags::DEFAULT | march_flags::BODIES | march_flags::CULL_BODIES | march_flags::BODY_SHADOWS;

    let packed = bevox_render::upload::pack_bodies(&world, std::slice::from_ref(&body));
    let (table, _) = bevox_render::cull::bodies_to_march(
        &packed.bodies,
        &packed.body_local_bounds,
        &bevox_render::upload::ExtractedMarchCamera { offset_from_clip, position: eye },
        flags,
        glam::UVec2::new(width, height),
    );
    assert!(table.is_empty(), "the cull keeps the body, so it is not off screen and this proves nothing");

    let shadowed = |bodies: &[Body]| {
        let pixels = Prepared::new(
            &device, &shader, "march_shadow", &world, offset_from_clip, eye, width, height, flags, bodies,
        )
        .read_back(&device, &queue);
        pixels.chunks(4).filter(|p| p[3] > 0 && p[0] > 127).count()
    };
    let (without, with) = (shadowed(&[]), shadowed(std::slice::from_ref(&body)));
    eprintln!("{without} pixels shadowed without the body, {with} with it");
    assert!(with > without + 50, "an off-screen body shadowed only {} more pixels", with - without);
}

/// With body shadows off, a body casts nothing: the off-screen caster's scene
/// renders exactly as the scene without it, on every entry point that shades.
#[test]
fn body_shadows_off_leave_the_image_as_with_no_caster() {
    let Some((device, queue)) = gpu_device() else {
        eprintln!("no GPU adapter available, skipping");
        return;
    };
    let (width, height) = (96u32, 96u32);
    let (eye, offset_from_clip, body) = off_screen_caster();
    let shader = std::fs::read_to_string("assets/shaders/march.wgsl").expect("shader missing");
    let world = body_world(&[]);
    let flags = (march_flags::DEFAULT | march_flags::BODIES | march_flags::CULL_BODIES) & !march_flags::BODY_SHADOWS;
    for entry in ["march_shadow", "march"] {
        let render = |bodies: &[Body]| {
            Prepared::new(&device, &shader, entry, &world, offset_from_clip, eye, width, height, flags, bodies)
                .read_back(&device, &queue)
        };
        let (without, with) = (render(&[]), render(std::slice::from_ref(&body)));
        let differing = without.chunks(4).zip(with.chunks(4)).filter(|(a, b)| a != b).count();
        assert_eq!(differing, 0, "{entry}: with body shadows off, a body changed {differing} pixels");
    }
}

/// A body whose content reaches its volume's max faces, seen from past them.
///
/// `extent` only bounds a body's root box, which makes a wrong one look
/// harmless. It is not. A static-world extent (256) around a 64 body puts a
/// camera just past the body's +X face *inside* the oversized box, so the
/// root slab enters at t = 0; with DDA the start cell is clamped from that
/// point and the march descends into edge cells the ray never crosses, with t
/// near 0 -- phantom hits that beat anything in front of them. Measured in
/// review on a body filling 25..64: 1116 phantom hits and 8091 of 8100 voxels
/// wrong from past +X, 478 phantom hits from an oblique view. A centred body
/// with empty edge cells, or a camera on the min side, shows none of it.
///
/// Checked both ways, per pixel, against the CPU: every hit must match, and
/// every miss must miss, which is what catches a phantom.
#[test]
fn a_body_reaching_its_volume_edge_matches_the_cpu_from_past_that_edge() {
    let Some((device, queue)) = gpu_device() else {
        eprintln!("no GPU adapter available, skipping");
        return;
    };
    let (width, height) = (96u32, 96u32);
    let shader = std::fs::read_to_string("assets/shaders/march.wgsl").expect("shader missing");
    let world = body_world(&[]);

    // Off the brick grid on the min side, flush with the volume on the max side.
    let mut dense = DenseVolume::new(64).unwrap();
    for z in 25..64 {
        for y in 25..64 {
            for x in 25..64 {
                dense.set(UVec3::new(x, y, z), MaterialId(1));
            }
        }
    }
    // Unrotated and on integer coordinates, so body-local voxels compare exactly.
    let body = Body::new(dense.into_contree(), Vec3::new(80.0, 40.0, 80.0), Quat::IDENTITY);

    // Local camera positions past the max faces: beyond 64, inside 256.
    let mut failures = Vec::new();
    for local_eye in [Vec3::new(110.0, 44.0, 44.0), Vec3::new(100.0, 110.0, 120.0)] {
        let eye = body.position + local_eye;
        let view = Mat4::look_at_rh(Vec3::ZERO, (body.position + Vec3::splat(44.0)) - eye, Vec3::Y);
        let projection = Mat4::perspective_rh(0.9, 1.0, 0.1, 500.0);
        let offset_from_clip = (projection * view).inverse();

        let pixels = run_bodies(
            &device, &queue, &shader, "march_voxel_id", offset_from_clip, eye, &world,
            std::slice::from_ref(&body), width, height,
        );

        let mut stats = MarchStats::default();
        let mut body_pixels = 0usize;
        let mut phantoms = 0usize;
        let mut wrong = 0usize;
        let mut first = String::new();
        for y in 0..height {
            for x in 0..width {
                let dir = ray_direction(offset_from_clip, x, y, width, height);
                let i = ((y * width + x) * 4) as usize;
                let gpu_hit = pixels[i + 3] > 0;
                let gpu_voxel =
                    UVec3::new(pixels[i] as u32, pixels[i + 1] as u32, pixels[i + 2] as u32);

                let expected = cpu_composed(&world, &body, eye, dir, &mut stats);
                if matches!(expected, Some((_, true))) {
                    body_pixels += 1;
                }
                let bad = match expected {
                    None if gpu_hit => {
                        phantoms += 1;
                        true
                    }
                    None => false,
                    Some((hit, _)) if !gpu_hit || gpu_voxel != hit.voxel => {
                        wrong += 1;
                        true
                    }
                    Some(_) => false,
                };
                if bad && first.is_empty() {
                    first = format!(
                        "at ({x},{y}) cpu={:?} gpu_hit={gpu_hit} gpu_voxel={gpu_voxel:?}",
                        expected.map(|(h, from_body)| (h.voxel, from_body))
                    );
                }
            }
        }

        eprintln!(
            "camera {local_eye}: {body_pixels} body pixels, {phantoms} phantom hits, {wrong} wrong"
        );
        // Collected rather than asserted here, so a failure reports every camera.
        if body_pixels <= 200 {
            failures.push(format!(
                "camera {local_eye}: only {body_pixels} pixels hit the body; the test is vacuous"
            ));
        }
        if phantoms + wrong > 0 {
            failures.push(format!(
                "camera {local_eye}: {phantoms} phantom hits and {wrong} wrong or missing hits \
                 ({body_pixels} body pixels); first {first}"
            ));
        }
    }
    assert!(failures.is_empty(), "{}", failures.join("\n"));
}

/// Culling bodies the camera cannot see must change no pixel.
///
/// Seven turned cubes: one fully in view, one straddling each of the four side
/// planes, one wholly past a side plane, one behind the camera. Each straddler's
/// centre is outside its plane, so a cull that tested the centre rather than
/// the whole sphere drops it while part of it is still on screen; the test
/// checks that is really so, rather than trusting the placement arithmetic.
///
/// Identical images alone would also pass a cull that never removed anything,
/// so the cull is called directly on the same inputs and must keep exactly the
/// five bodies on screen.
#[test]
fn culling_bodies_outside_the_view_leaves_output_bit_identical() {
    let Some((device, queue)) = gpu_device() else {
        eprintln!("no GPU adapter available, skipping");
        return;
    };
    let (width, height) = (96u32, 96u32);
    let eye = Vec3::new(128.0, 60.0, 40.0);
    let view = Mat4::look_at_rh(Vec3::ZERO, Vec3::Z, Vec3::Y);
    let projection = Mat4::perspective_rh(0.9, 1.0, 0.1, 500.0);
    let offset_from_clip = (projection * view).inverse();
    let shader = std::fs::read_to_string("assets/shaders/march.wgsl").expect("shader missing");
    let world = body_world(&[]);

    // Square, so the half-width and half-height at depth d are both
    // d * tan(0.45). A centre 4 past a side plane there is 3.6 outside it,
    // and the cube reaches 8 either side of its centre however it is turned.
    let d = 60.0;
    let edge = d * 0.45f32.tan() + 4.0;
    let turned = Quat::from_euler(glam::EulerRot::XYZ, 0.4, 0.8, -0.3);
    let place =
        |offset: Vec3| Body::new(body_cube(), eye + offset - turned * Vec3::splat(33.0), turned);
    let bodies = [
        place(Vec3::new(0.0, 0.0, d)),
        place(Vec3::new(edge, 0.0, d)),
        place(Vec3::new(-edge, 0.0, d)),
        place(Vec3::new(0.0, edge, d)),
        place(Vec3::new(0.0, -edge, d)),
        place(Vec3::new(edge + 40.0, 0.0, d)),
        place(Vec3::new(0.0, 0.0, -40.0)),
    ];
    let cull = march_flags::DEFAULT | march_flags::CULL_BODIES | march_flags::BODY_SHADOWS;
    let no_cull = (march_flags::DEFAULT & !march_flags::CULL_BODIES) | march_flags::BODY_SHADOWS;

    let packed = bevox_render::upload::pack_bodies(&world, &bodies);
    let camera = bevox_render::upload::ExtractedMarchCamera { offset_from_clip, position: eye };
    let kept: Vec<u32> = bevox_render::cull::bodies_to_march(
        &packed.bodies, &packed.body_local_bounds, &camera, cull, glam::UVec2::new(width, height),
    )
    .0
    .iter()
    .map(|g| g.node_base)
    .collect();
    let on_screen: Vec<u32> = packed.bodies[..5].iter().map(|g| g.node_base).collect();
    assert_eq!(kept, on_screen, "the cull must keep the five bodies on screen and drop the others");

    // Each straddler is genuine: its centre outside the frustum, its voxels
    // still hit by some pixel's ray.
    let frustum =
        bevox_render::cull::Frustum::from_camera(offset_from_clip, offset_from_clip.inverse(), eye);
    let mut stats = MarchStats::default();
    for i in 1..5 {
        let local = packed.body_local_bounds[i].expect("the cube has voxels");
        let centre = bevox_render::cull::world_bound(local, &packed.bodies[i]).centre;
        assert!(
            !frustum.sees(bevox_render::cull::BodyBound { centre, radius: 0.0 }),
            "body {i}'s centre is inside the frustum, so it does not straddle a side plane"
        );
        let hit = (0..width * height).any(|p| {
            let dir = ray_direction(offset_from_clip, p % width, p / width, width, height);
            bodies[i].march_world(eye, dir, 1000.0, &mut stats).is_some()
        });
        assert!(hit, "body {i} is meant to straddle a side plane, but no pixel sees it");
    }

    for entry in ["march_identity", "march_normal", "march"] {
        let prepare = |flags| {
            Prepared::new(
                &device, &shader, entry, &world, offset_from_clip, eye, width, height, flags,
                &bodies,
            )
        };
        let (without, with) = (prepare(no_cull), prepare(cull));
        assert_eq!(
            (without.body_count, with.body_count),
            (bodies.len() as u32, kept.len() as u32),
            "{entry}: the uniform's body count is not the length of the table written"
        );
        let (without, with) = (without.read_back(&device, &queue), with.read_back(&device, &queue));
        if entry == "march_identity" {
            // Material 1 is the cube's; the floor is another.
            let body_pixels = without.chunks(4).filter(|px| px[1] > 0 && px[0] == 1).count();
            assert!(body_pixels > 100, "only {body_pixels} body pixels; the comparison is vacuous");
        }
        let differing = without.chunks(4).zip(with.chunks(4)).filter(|(a, b)| a != b).count();
        assert_eq!(differing, 0, "{entry}: culling bodies changed {differing} pixels");
    }
}

/// Skipping a body for pixels outside its screen rectangle must change no pixel.
///
/// Turned cubes near, far, and straddling each of the four screen edges, plus
/// one behind the camera first in the list: with culling on as well, the table
/// is compacted, and rectangles built apart from it would pair each body with
/// its neighbour's. The rectangles are mostly disjoint, so a body tested against
/// another's loses pixels.
///
/// Identical images alone would also pass an early-out that never fires. So
/// each visible body's rectangle must exclude pixels the loop would otherwise
/// have marched it for, and shrinking every uploaded rectangle to one pixel
/// must change the image: the shader really reads them and skips.
#[test]
fn skipping_bodies_outside_their_screen_rectangles_leaves_output_bit_identical() {
    let Some((device, queue)) = gpu_device() else {
        eprintln!("no GPU adapter available, skipping");
        return;
    };
    // Not square, so a transposed axis in the rectangle cannot hide.
    let (width, height) = (128u32, 80u32);
    let eye = Vec3::new(128.0, 60.0, 40.0);
    let view = Mat4::look_at_rh(Vec3::ZERO, Vec3::Z, Vec3::Y);
    let projection = Mat4::perspective_rh(0.9, 1.6, 0.1, 500.0);
    let offset_from_clip = (projection * view).inverse();
    let shader = std::fs::read_to_string("assets/shaders/march.wgsl").expect("shader missing");
    let world = body_world(&[]);

    // Looking down +Z with Y up, world +X is screen left and +Y is up. At depth
    // d the half-height is d * tan(0.45) and the half-width 1.6 times that, so
    // each straddler's centre sits on its edge.
    let d = 60.0;
    let half_h = d * 0.45f32.tan();
    let half_w = half_h * 1.6;
    let cube = tight_cube();
    let turned = Quat::from_euler(glam::EulerRot::XYZ, 0.4, 0.8, -0.3);
    let place =
        |offset: Vec3| Body::new(cube.clone(), eye + offset - turned * Vec3::splat(32.0), turned);
    let bodies = [
        place(Vec3::new(0.0, 0.0, -40.0)),
        place(Vec3::new(-10.0, 0.0, 50.0)),
        place(Vec3::new(half_w, 0.0, d)),
        place(Vec3::new(0.0, half_h, d)),
        place(Vec3::new(0.0, -half_h, d)),
        place(Vec3::new(-half_w, 0.0, d)),
        place(Vec3::new(15.0, -5.0, 200.0)),
    ];
    let mut stats = MarchStats::default();
    for (i, body) in bodies.iter().enumerate().skip(1) {
        let seen = (0..width * height).any(|p| {
            let dir = ray_direction(offset_from_clip, p % width, p / width, width, height);
            body.march_world(eye, dir, 1000.0, &mut stats).is_some()
        });
        assert!(seen, "body {i} is meant to be on screen, but no pixel sees it");
    }

    let base = march_flags::DEFAULT & !march_flags::CULL_BODIES & !march_flags::BODY_RECT;
    let rect = base | march_flags::BODY_RECT;
    let rect_and_cull = rect | march_flags::CULL_BODIES;
    let prepare = |entry: &str, flags| {
        Prepared::new(
            &device, &shader, entry, &world, offset_from_clip, eye, width, height, flags, &bodies,
        )
    };

    // The straddlers really reach their edges. Unculled, table index i is body i.
    let r = prepare("march_identity", rect).body_rects;
    assert_eq!(
        (r[2].min[0], r[3].min[1], r[4].max[1], r[5].max[0]),
        (0, 0, height - 1, width - 1),
        "the edge bodies do not reach the left, top, bottom and right edges: {r:?}"
    );
    assert_eq!(
        r[0],
        GpuBodyRect::whole(glam::UVec2::new(width, height)),
        "the body behind the camera did not get the whole screen"
    );

    for entry in ["march_identity", "march_normal", "march"] {
        let without = prepare(entry, base).read_back(&device, &queue);
        if entry == "march_identity" {
            // Material 1 is the cube's; the floor is another.
            let body_pixels = without.chunks(4).filter(|px| px[1] > 0 && px[0] == 1).count();
            assert!(body_pixels > 100, "only {body_pixels} body pixels; the comparison is vacuous");
        }
        for flags in [rect, rect_and_cull] {
            let prepared = prepare(entry, flags);
            let rects = &prepared.body_rects;
            assert_eq!(rects.len(), prepared.body_count as usize, "{entry}: one rectangle per body");
            // Unculled, the body behind the camera leads the table with the
            // whole screen, as it must.
            let visible = if flags & march_flags::CULL_BODIES != 0 { &rects[..] } else { &rects[1..] };
            assert_eq!(visible.len(), bodies.len() - 1);
            let excluded: Vec<u32> = visible
                .iter()
                .map(|r| width * height - (r.max[0] + 1 - r.min[0]) * (r.max[1] + 1 - r.min[1]))
                .collect();
            eprintln!("{entry} (flags {flags}): pixels each visible body is skipped for: {excluded:?}");
            for (i, (r, n)) in visible.iter().zip(&excluded).enumerate() {
                assert!(
                    *n > 0,
                    "{entry}: visible body {i}'s rectangle {r:?} excludes no pixel, so the \
                     early-out never fires for it"
                );
            }
            let with = prepared.read_back(&device, &queue);
            let differing = without.chunks(4).zip(with.chunks(4)).filter(|(a, b)| a != b).count();
            assert_eq!(
                differing, 0,
                "{entry} (flags {flags}): skipping bodies outside their rectangles changed \
                 {differing} pixels"
            );

            if entry == "march_identity" {
                let one_pixel = vec![GpuBodyRect::default(); rects.len()];
                prepared.overwrite_body_rects(&queue, &one_pixel);
                let shrunk = prepared.read_back(&device, &queue);
                let changed = without.chunks(4).zip(shrunk.chunks(4)).filter(|(a, b)| a != b).count();
                assert!(
                    changed > 100,
                    "flags {flags}: one-pixel rectangles changed only {changed} pixels, so the \
                     shader is not skipping on them"
                );
            }
        }
    }
}

/// A 16-voxel cube on the brick grid (24..40), so its occupied bounds are its
/// voxels exactly and a rectangle a pixel short of them drops body pixels.
/// `body_cube` is off the grid and bounded up to three voxels loose, which hides
/// a short rectangle.
fn tight_cube() -> Contree {
    let mut dense = DenseVolume::new(64).unwrap();
    for z in 24..40 {
        for y in 24..40 {
            for x in 24..40 {
                dense.set(UVec3::new(x, y, z), MaterialId(1));
            }
        }
    }
    let cube = dense.into_contree();
    assert_eq!(
        bevox_core::body::occupied_bounds(&cube),
        Some((UVec3::splat(24), UVec3::splat(40))),
        "the cube's bounds are loose, so a rectangle short of the body could pass"
    );
    cube
}

/// Rectangles and the cull must change no pixel where precision runs out: the
/// app's projection (Bevy's default: reverse-Z, infinite, FOV pi/4, near 0.1) at
/// 2560x1440, the physical size of a 1280x720 window on a 2x display, about 2000
/// from the origin.
///
/// Rays unprojected through an absolute f32 view-projection drift a few pixels
/// here, past the rectangles' one-pixel pad; at the other gates, near the origin
/// at small sizes, they do not. At 1280x720 on the test GPU the drift at these
/// silhouettes stayed inside the pad, so that size could not tell the absolute
/// rays from the relative ones. The camera is yawed and pitched, as a player's
/// is: looking straight down an axis keeps the matrices sparse, and the drift
/// with them. Cubes turned to face it square-on lay whole silhouette edges along
/// their rectangles' edges, so a drift shows along a side rather than only at a
/// corner. One cube is turned further.
#[test]
fn body_rectangles_and_culling_hold_far_from_the_origin_with_the_app_camera() {
    let Some((device, queue)) = gpu_device() else {
        eprintln!("no GPU adapter available, skipping");
        return;
    };
    let (width, height) = (2560u32, 1440u32);
    // The static world sits at the origin, well outside the view: only bodies
    // draw.
    let eye = Vec3::new(1300.0, 700.0, -1350.0);
    assert!((eye.length() - 2000.0).abs() < 5.0);
    let projection =
        Mat4::perspective_infinite_reverse_rh(std::f32::consts::FRAC_PI_4, 16.0 / 9.0, 0.1);
    let view = Mat4::look_at_rh(Vec3::ZERO, Vec3::new(0.45, -0.25, 0.85), Vec3::Y);
    let offset_from_clip = (projection * view).inverse();
    let facing = Quat::from_mat4(&view.inverse()).normalize();
    let shader = std::fs::read_to_string("assets/shaders/march.wgsl").expect("shader missing");
    let world = body_world(&[]);

    let cube = tight_cube();
    let turned = Quat::from_euler(glam::EulerRot::XYZ, 0.4, 0.8, -0.3);
    // At `right`, `up` and `depth` in the camera's frame.
    let place = |right: f32, up: f32, depth: f32, orientation: Quat| {
        let centre = eye + facing * Vec3::new(right, up, -depth);
        Body::new(cube.clone(), centre - orientation * Vec3::splat(32.0), orientation)
    };
    // A grid across the view, so silhouette edges fall in many columns and rows,
    // and one cube turned further.
    let mut bodies: Vec<Body> = (0..4)
        .flat_map(|i| (0..3).map(move |j| (i, j)))
        .map(|(i, j)| place(i as f32 * 26.0 - 39.0, j as f32 * 24.0 - 24.0, 90.0, facing))
        .collect();
    bodies.push(place(0.0, 0.0, 50.0, facing * turned));

    let off = march_flags::DEFAULT & !march_flags::CULL_BODIES & !march_flags::BODY_RECT;
    let on = off | march_flags::BODY_RECT | march_flags::CULL_BODIES;
    let prepare = |entry: &str, flags| {
        Prepared::new(
            &device, &shader, entry, &world, offset_from_clip, eye, width, height, flags, &bodies,
        )
    };

    for entry in ["march_identity", "march_normal", "march"] {
        let without = prepare(entry, off).read_back(&device, &queue);
        let prepared = prepare(entry, on);
        assert_eq!(prepared.body_count as usize, bodies.len(), "{entry}: the cull dropped a body in plain view");
        if entry == "march_identity" {
            // Each body is drawn inside its rectangle (material 1, hit flag),
            // and each rectangle leaves pixels out.
            for (i, r) in prepared.body_rects.iter().enumerate() {
                let drawn = (r.min[1]..=r.max[1])
                    .flat_map(|y| (r.min[0]..=r.max[0]).map(move |x| (y * width + x) as usize * 4))
                    .filter(|&p| without[p + 1] > 0 && without[p] == 1)
                    .count();
                let area = (r.max[0] + 1 - r.min[0]) * (r.max[1] + 1 - r.min[1]);
                assert!(drawn > 1000, "body {i} drew only {drawn} pixels in {r:?}; the gate is vacuous");
                assert!(area < width * height, "body {i}'s rectangle {r:?} excludes no pixel");
            }
        }
        let with = prepared.read_back(&device, &queue);
        let differing = without.chunks(4).zip(with.chunks(4)).filter(|(a, b)| a != b).count();
        assert_eq!(
            differing, 0,
            "{entry}: rectangles and culling changed {differing} pixels about 2000 from the origin"
        );
    }
}

/// Entry points that write one component of `primary_ray` as its raw bits, a
/// byte per channel, so a test can read the GPU's own ray exactly.
const RAY_BITS: &str = r#"
fn store_bits(id: vec3<u32>, v: u32) {
    let b = vec4<f32>(vec4<u32>(v & 255u, (v >> 8u) & 255u, (v >> 16u) & 255u, v >> 24u));
    textureStore(output, vec2<i32>(id.xy), b / 255.0);
}
@compute @workgroup_size(8, 8, 1)
fn ray_bits_x(@builtin(global_invocation_id) id: vec3<u32>) {
    let s = textureDimensions(output);
    if id.x < s.x && id.y < s.y { store_bits(id, bitcast<u32>(primary_ray(id, s).x)); }
}
@compute @workgroup_size(8, 8, 1)
fn ray_bits_y(@builtin(global_invocation_id) id: vec3<u32>) {
    let s = textureDimensions(output);
    if id.x < s.x && id.y < s.y { store_bits(id, bitcast<u32>(primary_ray(id, s).y)); }
}
@compute @workgroup_size(8, 8, 1)
fn ray_bits_z(@builtin(global_invocation_id) id: vec3<u32>) {
    let s = textureDimensions(output);
    if id.x < s.x && id.y < s.y { store_bits(id, bitcast<u32>(primary_ray(id, s).z)); }
}
"#;

/// At a voxel edge the ray crosses exactly, DDA must visit every cell it
/// touches there, in the order the scan path and the CPU reference do: nearest
/// first, lowest child index at equal distance. Stepping diagonally past a
/// grazed cell, or starting a frame past the cell grazed on entry, picks another
/// voxel or misses, and every seeded or accelerated walk must agree too.
///
/// The ties are placed, not hoped for. Looking straight down +Z at a square
/// target, a pixel on either diagonal has |dir.x| == |dir.y| exactly, so from an
/// eye the same distance from an edge on both axes its ray crosses that edge
/// exactly. Whether it does is read from the GPU's own ray, bit for bit, and the
/// CPU reference marches that same ray. Three placements:
///
/// - a floor edge on a brick boundary, seen from above: the grazed voxel is in
///   the brick the ray is leaving, which the diagonal step leaves first;
/// - a floor edge inside a brick: the grazed voxel has the lower index;
/// - a block's top edge seen from below, on a brick boundary: the ray enters the
///   next brick exactly on an inner plane, grazing the voxel below it, and
///   without it misses the block entirely.
///
/// Each placement must produce grazed hits, or the gate cannot tell a DDA that
/// visits them from one that does not.
#[test]
fn a_ray_crossing_a_voxel_edge_exactly_hits_what_the_scan_hits() {
    let Some((device, queue)) = gpu_device() else {
        eprintln!("no GPU adapter available, skipping");
        return;
    };
    let (width, height) = (96u32, 96u32);
    let projection = Mat4::perspective_rh(0.9, 1.0, 0.1, 500.0);
    let offset_from_clip = (projection * Mat4::look_at_rh(Vec3::ZERO, Vec3::Z, Vec3::Y)).inverse();
    let source = std::fs::read_to_string("assets/shaders/march.wgsl").expect("shader missing");
    let shader = format!("{source}\n{RAY_BITS}");

    let solid = |hi_x: u32, lo_x: u32, hi_y: u32| {
        let mut dense = DenseVolume::new(64).unwrap();
        for z in 0..64 {
            for y in 0..hi_y {
                for x in lo_x..hi_x {
                    dense.set(UVec3::new(x, y, z), MaterialId(1));
                }
            }
        }
        dense.into_contree()
    };
    let floor = solid(64, 0, 6);
    let block = solid(64, 20, 10);
    // (label, world, eye, the edge's y, the grazed voxel's (x, y) for a ray
    // crossing the edge at x going the given way along x)
    type Grazed = fn(f32, bool) -> (u32, u32);
    let placements: [(&str, &Contree, Vec3, f32, Grazed); 4] = [
        ("floor edge on a brick boundary", &floor, Vec3::new(32.0, 18.0, -40.0), 6.0, |x, pos| {
            (if pos { x as u32 - 1 } else { x as u32 }, 5)
        }),
        ("floor edge inside a brick", &floor, Vec3::new(30.0, 18.0, -40.0), 6.0, |x, pos| {
            (if pos { x as u32 - 1 } else { x as u32 }, 5)
        }),
        ("block edge entered from below", &block, Vec3::new(14.0, 4.0, -40.0), 10.0, |x, _| {
            (x as u32, 9)
        }),
        // Eye inside the volume, in open space above the floor: unlike the
        // other placements it sits in a distance-field cell the floor never
        // touches (field cells are 16 voxels, and y=20 is outside the y<16
        // cell the floor marks zero), so the DISTANCE_FIELD-only flag set
        // below actually exercises skip_empty_space's advance instead of
        // returning immediately, as it does when the eye starts outside the
        // volume entirely.
        ("floor edge with the eye inside the volume", &floor, Vec3::new(30.0, 20.0, 32.0), 6.0, |x, pos| {
            (if pos { x as u32 - 1 } else { x as u32 }, 5)
        }),
    ];

    let flag_sets = [
        march_flags::DDA,
        march_flags::MASK_FILTER,
        march_flags::DDA | march_flags::MASK_FILTER,
        march_flags::BEAM,
        march_flags::DISTANCE_FIELD,
        march_flags::DEFAULT,
    ];
    let mut failures = Vec::new();
    for (label, world, eye, edge_y, grazed) in placements {
        let axis = |entry: &str| -> Vec<f32> {
            let px = Prepared::new(
                &device, &shader, entry, world, offset_from_clip, eye, width, height, 0, &[],
            )
            .read_back(&device, &queue);
            px.chunks(4).map(|c| f32::from_bits(u32::from_le_bytes([c[0], c[1], c[2], c[3]]))).collect()
        };
        let (xs, ys, zs) = (axis("ray_bits_x"), axis("ray_bits_y"), axis("ray_bits_z"));
        let render = |flags| {
            Prepared::new(
                &device, &shader, "march_voxel_id", world, offset_from_clip, eye, width, height,
                flags, &[],
            )
            .read_back(&device, &queue)
        };
        let scan = render(march_flags::NONE);

        let mut stats = MarchStats::default();
        let (mut ties, mut grazed_hits, mut wrong) = (0, 0, 0);
        let d = edge_y - eye.y;
        for p in 0..(width * height) as usize {
            let dir = Vec3::new(xs[p], ys[p], zs[p]);
            // The same distances the shader computes: plane minus origin, times
            // the reciprocal. The edge is `d` from the eye along x, on the side
            // the ray is heading.
            let edge_x = eye.x + d.abs() * dir.x.signum();
            let inv = Vec3::ONE / dir;
            if (edge_x - eye.x) * inv.x != (edge_y - eye.y) * inv.y {
                continue;
            }
            let Some(hit) = march(world, Affine3A::IDENTITY, eye, dir, 1000.0, false, &mut stats)
            else {
                continue;
            };
            ties += 1;
            let at_edge = hit.t == (edge_y - eye.y) * inv.y;
            grazed_hits +=
                usize::from(at_edge && (hit.voxel.x, hit.voxel.y) == grazed(edge_x, dir.x > 0.0));
            let got = &scan[p * 4..p * 4 + 4];
            if got[3] == 0 || UVec3::new(got[0] as u32, got[1] as u32, got[2] as u32) != hit.voxel {
                wrong += 1;
            }
        }
        eprintln!("{label}: {ties} tied rays that hit, {grazed_hits} on the grazed voxel, {wrong} scan mismatches");
        if ties == 0 {
            failures.push(format!("{label}: no tied ray reached a hit; the placement is vacuous"));
        }
        if grazed_hits < 3 {
            failures.push(format!("{label}: only {grazed_hits} rays hit the grazed voxel; the tie is not placed"));
        }
        if wrong > 0 {
            failures.push(format!("{label}: the scan path disagrees with the CPU on {wrong} tied rays"));
        }
        for flags in flag_sets {
            let got = render(flags);
            let differing = scan.chunks(4).zip(got.chunks(4)).filter(|(a, b)| a != b).count();
            if differing > 0 {
                failures.push(format!("{label}: flags {flags:#b} changed {differing} pixels"));
            }
        }
    }
    assert!(failures.is_empty(), "{}", failures.join("\n"));
}
