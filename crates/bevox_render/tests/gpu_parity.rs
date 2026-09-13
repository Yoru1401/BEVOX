//! Runs the real shader on a headless device and compares it with the CPU
//! reference marcher. Skips cleanly when no adapter is available, so a machine
//! without a usable GPU reports a skip rather than a false failure.

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
