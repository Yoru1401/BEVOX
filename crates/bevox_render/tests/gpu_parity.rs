//! Runs the real shader on a headless device and compares it with the CPU
//! reference marcher. Skips cleanly when no adapter is available, so a machine
//! without a usable GPU reports a skip rather than a false failure.

use bevox_core::contree::Contree;
use bevox_core::dense::DenseVolume;
use bevox_core::gpu::GpuVolume;
use bevox_core::march::{MarchStats, march};
use bevox_render::upload::march_flags;
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
    // 0b110 only: DDA has landed and owns its own identity test below. The
    // remaining bits must still be inert, which is what keeps a half-finished
    // optimisation from silently altering output.
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
            height, 0b110,
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
    let gpu_volume = GpuVolume::from_contree(&tree);
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
        let view = Mat4::look_at_rh(*eye, *target, Vec3::Y);
        let projection = Mat4::perspective_rh(0.9, 1.0, 0.1, 500.0);
        let world_from_clip = (projection * view).inverse();

        for entry in ["march_identity", "march_normal", "march_shadow", "march"] {
            let off = run_march_flagged(
                &device, &queue, &shader, entry, world_from_clip, *eye, &tree, &gpu_volume, width,
                height, march_flags::NONE,
            );
            let on = run_march_flagged(
                &device, &queue, &shader, entry, world_from_clip, *eye, &tree, &gpu_volume, width,
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
    let gpu_volume = GpuVolume::from_contree(&tree);
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
        let view = Mat4::look_at_rh(*eye, *target, Vec3::Y);
        let projection = Mat4::perspective_rh(0.9, 1.0, 0.1, 500.0);
        let world_from_clip = (projection * view).inverse();

        for entry in ["march_identity", "march_normal", "march_shadow", "march"] {
            let reference = run_march_flagged(
                &device, &queue, &shader, entry, world_from_clip, *eye, &tree, &gpu_volume, width,
                height, march_flags::NONE,
            );
            for flags in [
                march_flags::MASK_FILTER,
                march_flags::DDA | march_flags::MASK_FILTER,
            ] {
                let got = run_march_flagged(
                    &device, &queue, &shader, entry, world_from_clip, *eye, &tree, &gpu_volume,
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
    let gpu_volume = GpuVolume::from_contree(&tree);
    let (width, height) = (128u32, 128u32);
    let centre = Vec3::splat(32.0);
    let shader = std::fs::read_to_string("assets/shaders/march.wgsl").expect("shader missing");

    for step in 0..8u32 {
        let angle = step as f32 * std::f32::consts::TAU / 8.0;
        let eye = centre + Vec3::new(angle.cos() * 90.0, 30.0, angle.sin() * 90.0);
        let view = Mat4::look_at_rh(eye, centre, Vec3::Y);
        let projection = Mat4::perspective_rh(0.9, 1.0, 0.1, 500.0);
        let world_from_clip = (projection * view).inverse();

        for entry in ["march_identity", "march_voxel_id", "march_normal", "march_shadow", "march"]
        {
            let reference = run_march_flagged(
                &device, &queue, &shader, entry, world_from_clip, eye, &tree, &gpu_volume, width,
                height, march_flags::NONE,
            );
            for flags in [
                march_flags::BEAM,
                march_flags::DDA | march_flags::MASK_FILTER | march_flags::BEAM,
            ] {
                let seeded = run_march_flagged(
                    &device, &queue, &shader, entry, world_from_clip, eye, &tree, &gpu_volume,
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
    let gpu_volume = GpuVolume::from_contree(&tree);
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
            let view = Mat4::look_at_rh(*eye, target, Vec3::Y);
            let projection = Mat4::perspective_rh(0.9, 1.0, 0.1, 500.0);
            let world_from_clip = (projection * view).inverse();
            for entry in ["march_identity", "march_voxel_id"] {
                let scan = run_march_flagged(
                    &device, &queue, &shader, entry, world_from_clip, *eye, &tree, &gpu_volume,
                    width, height, march_flags::NONE,
                );
                let dda = run_march_flagged(
                    &device, &queue, &shader, entry, world_from_clip, *eye, &tree, &gpu_volume,
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
