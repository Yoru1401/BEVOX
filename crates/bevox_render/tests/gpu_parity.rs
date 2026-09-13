//! Runs the real shader on a headless device and compares it with the CPU
//! reference marcher. Skips cleanly when no adapter is available, so a machine
//! without a usable GPU reports a skip rather than a false failure.

use bevox_core::contree::Contree;
use bevox_core::dense::DenseVolume;
use bevox_core::gpu::GpuVolume;
use bevox_core::march::{MarchStats, march};
use bevox_core::material::MaterialId;
use glam::{Affine3A, Mat4, UVec3, Vec3, Vec4};
use wgpu::util::DeviceExt;

#[repr(C)]
#[derive(Clone, Copy, bytemuck::Pod, bytemuck::Zeroable)]
struct TestUniform {
    world_from_clip: [[f32; 4]; 4],
    camera_position: [f32; 4],
    volume_params: [u32; 4],
}

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

/// Binds the full four-resource layout the real shader declares.
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
    let uniform = TestUniform {
        world_from_clip: world_from_clip.to_cols_array_2d(),
        camera_position: eye.extend(0.0).to_array(),
        volume_params: [tree.depth(), tree.extent(), 0, 0],
    };

    // Root first, arena shifted by one: the layout the shader indexes.
    let nodes = volume.buffer_nodes();
    let node_bytes: Vec<u8> = if nodes.is_empty() {
        vec![0u8; 16]
    } else {
        bytemuck::cast_slice(&nodes).to_vec()
    };
    // A zero-length storage buffer is invalid, so an empty volume gets padding.
    let voxel_bytes: Vec<u8> = if volume.voxels.is_empty() {
        vec![0u8; 4]
    } else {
        bytemuck::cast_slice(&volume.voxels).to_vec()
    };

    let uniform_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
        label: Some("uniform"),
        contents: bytemuck::bytes_of(&uniform),
        usage: wgpu::BufferUsages::UNIFORM,
    });
    let node_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
        label: Some("nodes"),
        contents: &node_bytes,
        usage: wgpu::BufferUsages::STORAGE,
    });
    let voxel_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
        label: Some("voxels"),
        contents: &voxel_bytes,
        usage: wgpu::BufferUsages::STORAGE,
    });

    let module = device.create_shader_module(wgpu::ShaderModuleDescriptor {
        label: Some("march"),
        source: wgpu::ShaderSource::Wgsl(source.into()),
    });
    let texture = storage_target(device, width, height);
    let view = texture.create_view(&wgpu::TextureViewDescriptor::default());

    let pipeline = device.create_compute_pipeline(&wgpu::ComputePipelineDescriptor {
        label: Some("march_pipeline"),
        layout: None,
        module: &module,
        entry_point: Some(entry_point),
        compilation_options: Default::default(),
        cache: None,
    });
    let bind_group = device.create_bind_group(&wgpu::BindGroupDescriptor {
        label: None,
        layout: &pipeline.get_bind_group_layout(0),
        entries: &[
            wgpu::BindGroupEntry { binding: 0, resource: uniform_buffer.as_entire_binding() },
            wgpu::BindGroupEntry { binding: 1, resource: node_buffer.as_entire_binding() },
            wgpu::BindGroupEntry { binding: 2, resource: voxel_buffer.as_entire_binding() },
            wgpu::BindGroupEntry {
                binding: 3,
                resource: wgpu::BindingResource::TextureView(&view),
            },
        ],
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

fn gpu_device() -> Option<(wgpu::Device, wgpu::Queue)> {
    pollster::block_on(async {
        let instance = wgpu::Instance::default();
        let adapter = instance
            .request_adapter(&wgpu::RequestAdapterOptions::default())
            .await
            .ok()?;
        adapter
            .request_device(&wgpu::DeviceDescriptor {
                label: Some("bevox_test_device"),
                ..Default::default()
            })
            .await
            .ok()
    })
}

/// Copies a texture back as tightly packed RGBA bytes.
///
/// Readback rows must be padded to 256 bytes, so the padding is stripped here
/// rather than leaking into every comparison.
fn read_texture(
    device: &wgpu::Device,
    queue: &wgpu::Queue,
    encoder: wgpu::CommandEncoder,
    texture: &wgpu::Texture,
    width: u32,
    height: u32,
) -> Vec<u8> {
    let unpadded = width * 4;
    let padded = unpadded.div_ceil(256) * 256;
    let readback = device.create_buffer(&wgpu::BufferDescriptor {
        label: Some("readback"),
        size: (padded * height) as u64,
        usage: wgpu::BufferUsages::COPY_DST | wgpu::BufferUsages::MAP_READ,
        mapped_at_creation: false,
    });

    let mut encoder = encoder;
    encoder.copy_texture_to_buffer(
        texture.as_image_copy(),
        wgpu::TexelCopyBufferInfo {
            buffer: &readback,
            layout: wgpu::TexelCopyBufferLayout {
                offset: 0,
                bytes_per_row: Some(padded),
                rows_per_image: Some(height),
            },
        },
        wgpu::Extent3d { width, height, depth_or_array_layers: 1 },
    );
    queue.submit([encoder.finish()]);

    let slice = readback.slice(..);
    slice.map_async(wgpu::MapMode::Read, |_| {});
    device
        .poll(wgpu::PollType::wait_indefinitely())
        .expect("device poll failed");

    let data = slice.get_mapped_range();
    let mut out = Vec::with_capacity((unpadded * height) as usize);
    for row in 0..height {
        let start = (row * padded) as usize;
        out.extend_from_slice(&data[start..start + unpadded as usize]);
    }
    drop(data);
    readback.unmap();
    out
}

fn storage_target(device: &wgpu::Device, width: u32, height: u32) -> wgpu::Texture {
    device.create_texture(&wgpu::TextureDescriptor {
        label: Some("test_target"),
        size: wgpu::Extent3d { width, height, depth_or_array_layers: 1 },
        mip_level_count: 1,
        sample_count: 1,
        dimension: wgpu::TextureDimension::D2,
        format: wgpu::TextureFormat::Rgba8Unorm,
        usage: wgpu::TextureUsages::STORAGE_BINDING | wgpu::TextureUsages::COPY_SRC,
        view_formats: &[],
    })
}

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
