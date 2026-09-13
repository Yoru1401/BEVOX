//! Runs the real shader on a headless device and compares it with the CPU
//! reference marcher. Skips cleanly when no adapter is available, so a machine
//! without a usable GPU reports a skip rather than a false failure.

use bevox_core::contree::Contree;
use bevox_core::dense::DenseVolume;
use bevox_core::gpu::GpuVolume;
use bevox_core::march::{MarchStats, march};
use bevox_core::material::MaterialId;
use glam::{Affine3A, Mat4, UVec3, Vec3, Vec4};

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



/// Same ray construction the shader performs, so both sides march the same rays.
fn ray_direction(world_from_clip: Mat4, eye: Vec3, x: u32, y: u32, w: u32, h: u32) -> Vec3 {
    let ndc_x = (x as f32 + 0.5) / w as f32 * 2.0 - 1.0;
    let ndc_y = 1.0 - (y as f32 + 0.5) / h as f32 * 2.0;
    let far = world_from_clip * Vec4::new(ndc_x, ndc_y, 1.0, 1.0);
    (far.truncate() / far.w - eye).normalize()
}

/// Renders one frame with the given traversal flags and reads it back.
#[allow(clippy::too_many_arguments)]
fn run_march_flagged(
    device: &wgpu::Device,
    queue: &wgpu::Queue,
    source: &str,
    entry_point: &str,
    world_from_clip: Mat4,
    eye: Vec3,
    tree: &Contree,
    volume: &GpuVolume,
    width: u32,
    height: u32,
    flags: u32,
) -> Vec<u8> {
    Prepared::new(
        device, source, entry_point, tree, volume, world_from_clip, eye, width, height, flags,
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
    world_from_clip: Mat4,
    eye: Vec3,
    tree: &Contree,
    volume: &GpuVolume,
    width: u32,
    height: u32,
) -> Vec<u8> {
    Prepared::new(
        device, source, entry_point, tree, volume, world_from_clip, eye, width, height, 0,
    )
    .read_back(device, queue)
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
    let gpu_volume = GpuVolume::from_contree(&tree);

    let (width, height) = (64u32, 64u32);
    let eye = Vec3::new(-30.0, 40.0, -30.0);
    let view = Mat4::look_at_rh(eye, Vec3::new(32.0, 12.0, 32.0), Vec3::Y);
    let projection = Mat4::perspective_rh(0.9, width as f32 / height as f32, 0.1, 500.0);
    let world_from_clip = (projection * view).inverse();

    let shader = std::fs::read_to_string("assets/shaders/march.wgsl").expect("shader file missing");
    let pixels = run_march(
        &device,
        &queue,
        &shader,
        "march",
        world_from_clip,
        eye,
        &tree,
        &gpu_volume,
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
    let gpu_volume = GpuVolume::from_contree(&tree);
    let (width, height) = (64u32, 64u32);
    let eye = Vec3::new(-30.0, 40.0, -30.0);
    let view = Mat4::look_at_rh(eye, Vec3::new(32.0, 12.0, 32.0), Vec3::Y);
    let projection = Mat4::perspective_rh(0.9, width as f32 / height as f32, 0.1, 500.0);
    let world_from_clip = (projection * view).inverse();

    let shader = std::fs::read_to_string("assets/shaders/march.wgsl").expect("shader file missing");
    let pixels = run_march(
        &device,
        &queue,
        &shader,
        "march_voxel_id",
        world_from_clip,
        eye,
        &tree,
        &gpu_volume,
        width,
        height,
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
        &device,
        &queue,
        &shader,
        "march_normal",
        world_from_clip,
        eye,
        &tree,
        &gpu_volume,
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
            let dir = ray_direction(world_from_clip, eye, x, y, width, height);
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
    let gpu_volume = GpuVolume::from_contree(&tree);
    let (width, height) = (64u32, 64u32);
    let eye = Vec3::new(-30.0, 40.0, -30.0);
    let view = Mat4::look_at_rh(eye, Vec3::new(32.0, 12.0, 32.0), Vec3::Y);
    let projection = Mat4::perspective_rh(0.9, width as f32 / height as f32, 0.1, 500.0);
    let world_from_clip = (projection * view).inverse();
    let sun = bevox_render::upload::SUN_DIRECTION.normalize();

    let shader = std::fs::read_to_string("assets/shaders/march.wgsl").expect("shader file missing");
    let pixels = run_march(
        &device,
        &queue,
        &shader,
        "march_shadow",
        world_from_clip,
        eye,
        &tree,
        &gpu_volume,
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
    let gpu_volume = GpuVolume::from_contree(&tree);

    // Framed exactly as the app frames it.
    let centre_f = Vec3::splat(extent as f32 * 0.5);
    let eye = centre_f + Vec3::new(-1.0, 1.2, -1.0).normalize() * extent as f32 * 1.1;

    let (width, height) = (256u32, 256u32);
    let view = Mat4::look_at_rh(eye, centre_f, Vec3::Y);
    let projection = Mat4::perspective_rh(0.9, 1.0, 0.1, 20_000.0);
    let world_from_clip = (projection * view).inverse();

    let shader = std::fs::read_to_string("assets/shaders/march.wgsl").expect("shader file missing");
    let pixels = run_march(
        &device,
        &queue,
        &shader,
        "march_identity",
        world_from_clip,
        eye,
        &tree,
        &gpu_volume,
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
            let dir = ray_direction(world_from_clip, eye, x, y, width, height);
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
fn setting_flags_does_not_change_output_yet() {
    let Some((device, queue)) = gpu_device() else {
        eprintln!("no GPU adapter available, skipping");
        return;
    };
    let tree = parity_scene();
    let gpu_volume = GpuVolume::from_contree(&tree);
    let (width, height) = (64u32, 64u32);
    let eye = Vec3::new(-30.0, 40.0, -30.0);
    let view = Mat4::look_at_rh(eye, Vec3::new(32.0, 12.0, 32.0), Vec3::Y);
    let projection = Mat4::perspective_rh(0.9, 1.0, 0.1, 500.0);
    let world_from_clip = (projection * view).inverse();
    let shader = std::fs::read_to_string("assets/shaders/march.wgsl").expect("shader missing");

    for entry in ["march_identity", "march_normal", "march_shadow", "march"] {
        let off = run_march_flagged(
            &device, &queue, &shader, entry, world_from_clip, eye, &tree, &gpu_volume, width,
            height, 0,
        );
        let on = run_march_flagged(
            &device, &queue, &shader, entry, world_from_clip, eye, &tree, &gpu_volume, width,
            height, 0b111,
        );
        assert_eq!(off, on, "{entry}: flags changed output before any optimisation exists");
    }
}

#[test]
fn the_gpu_traversal_agrees_with_the_cpu_reference() {
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
    let gpu_pixels = run_march(
        &device,
        &queue,
        &shader,
        "march_identity",
        world_from_clip,
        eye,
        &tree,
        &gpu_volume,
        width,
        height,
    );

    let mut stats = MarchStats::default();
    let mut mismatches = 0usize;
    let mut first: Option<(u32, u32, String)> = None;

    for y in 0..height {
        for x in 0..width {
            let dir = ray_direction(world_from_clip, eye, x, y, width, height);
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
