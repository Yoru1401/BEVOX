//! Shared GPU test scaffolding.
//!
//! One definition of the bind group layout, the buffers and the pipeline, used
//! by both the parity tests and the benchmark. Two copies would drift, and a
//! harness that stops matching the layout the app ships stops testing it.

// Each integration test binary compiles this module separately, so items only
// one of them uses look dead to the other.
#![allow(dead_code)]

use bevox_core::contree::Contree;
use bevox_core::gpu::GpuVolume;
use bevox_core::material::{Material, MaterialTable};
use glam::{Mat4, Vec3};
use wgpu::util::DeviceExt;

#[repr(C)]
#[derive(Clone, Copy, bytemuck::Pod, bytemuck::Zeroable)]
pub struct TestUniform {
    pub world_from_clip: [[f32; 4]; 4],
    pub camera_position: [f32; 4],
    pub sun_direction: [f32; 4],
    /// `[depth, extent, flags, 0]`.
    pub volume_params: [u32; 4],
}

/// Colours for the parity scene's two materials. A zeroed palette would render
/// every surface black, which is indistinguishable from a broken traversal.
pub fn parity_materials() -> MaterialTable {
    let mut table = MaterialTable::new();
    table.push(Material { color: [140, 140, 150, 255] }).unwrap(); // 1: stone
    table.push(Material { color: [180, 90, 70, 255] }).unwrap(); // 2: brick
    table
}

/// One read-only storage binding of the given minimum element size.
pub fn storage_entry(binding: u32, min_size: u64) -> wgpu::BindGroupLayoutEntry {
    wgpu::BindGroupLayoutEntry {
        binding,
        visibility: wgpu::ShaderStages::COMPUTE,
        ty: wgpu::BindingType::Buffer {
            ty: wgpu::BufferBindingType::Storage { read_only: true },
            has_dynamic_offset: false,
            min_binding_size: wgpu::BufferSize::new(min_size),
        },
        count: None,
    }
}

pub fn gpu_device() -> Option<(wgpu::Device, wgpu::Queue)> {
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

pub fn storage_target(device: &wgpu::Device, width: u32, height: u32) -> wgpu::Texture {
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

/// Copies a texture back as tightly packed RGBA bytes, stripping the 256-byte
/// row padding the copy requires.
pub fn read_texture(
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

/// One ready-to-dispatch configuration: pipeline, bindings, target and size.
///
/// Built once and reused, so a timed loop measures the shader rather than
/// resource creation.
/// Full-resolution pixels per beam sample, per axis. Must match BEAM_SCALE.
pub const BEAM_SCALE: u32 = 8;

pub struct Prepared {
    pipeline: wgpu::ComputePipeline,
    /// Present only when the beam flag is set, so the suite does not compile a
    /// prepass for the hundred dispatches that never run one.
    beam_pipeline: Option<wgpu::ComputePipeline>,
    bind_group: wgpu::BindGroup,
    texture: wgpu::Texture,
    /// Kept so a test can apply an incremental update the way the app does.
    node_buffer: wgpu::Buffer,
    voxel_buffer: wgpu::Buffer,
    width: u32,
    height: u32,
}

impl Prepared {
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        device: &wgpu::Device,
        source: &str,
        entry_point: &str,
        tree: &Contree,
        volume: &GpuVolume,
        world_from_clip: Mat4,
        eye: Vec3,
        width: u32,
        height: u32,
        flags: u32,
    ) -> Self {
        let uniform = TestUniform {
            world_from_clip: world_from_clip.to_cols_array_2d(),
            camera_position: eye.extend(0.0).to_array(),
            sun_direction: bevox_render::upload::SUN_DIRECTION
                .normalize()
                .extend(0.0)
                .to_array(),
            volume_params: [tree.depth(), tree.extent(), flags, 0],
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

        let node_capacity = bevox_render::pipeline::buffer_capacity_for(nodes.len() as u32);
        let mut node_bytes = node_bytes;
        node_bytes.resize(node_capacity as usize * 16, 0);

        let voxel_capacity =
            bevox_render::pipeline::buffer_capacity_for(volume.voxels.len() as u32);
        let mut voxel_bytes = voxel_bytes;
        voxel_bytes.resize(voxel_capacity as usize * 4, 0);

        let uniform_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("uniform"),
            contents: bytemuck::bytes_of(&uniform),
            usage: wgpu::BufferUsages::UNIFORM,
        });
        let node_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("nodes"),
            contents: &node_bytes,
            usage: wgpu::BufferUsages::STORAGE | wgpu::BufferUsages::COPY_DST,
        });
        let voxel_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("voxels"),
            contents: &voxel_bytes,
            usage: wgpu::BufferUsages::STORAGE | wgpu::BufferUsages::COPY_DST,
        });
        let palette = parity_materials().to_gpu();
        let beam_dims = (width.div_ceil(BEAM_SCALE).max(1), height.div_ceil(BEAM_SCALE).max(1));
        let beam_buffer = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("beam"),
            size: u64::from(beam_dims.0 * beam_dims.1) * 4,
            usage: wgpu::BufferUsages::STORAGE,
            mapped_at_creation: false,
        });
        let mask_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("masks"),
            contents: bytemuck::cast_slice(&bevox_render::upload::gpu_direction_masks()),
            usage: wgpu::BufferUsages::STORAGE,
        });
        let palette_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("palette"),
            contents: bytemuck::cast_slice(&palette),
            usage: wgpu::BufferUsages::STORAGE,
        });

        let module = device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("march"),
            source: wgpu::ShaderSource::Wgsl(source.into()),
        });
        let texture = storage_target(device, width, height);
        let view = texture.create_view(&wgpu::TextureViewDescriptor::default());

        // An explicit layout, not an auto-derived one. With `layout: None` naga
        // derives the layout from the bindings an entry point actually uses, so
        // an entry point that ignores the palette gets four bindings while one
        // that reads it gets five. Declaring it here mirrors init_march_pipeline,
        // so the harness tests the layout the app really ships.
        let layout = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
            label: Some("march_layout"),
            entries: &[
                storage_entry(1, 16),
                storage_entry(2, 4),
                storage_entry(3, 16),
                wgpu::BindGroupLayoutEntry {
                    binding: 0,
                    visibility: wgpu::ShaderStages::COMPUTE,
                    ty: wgpu::BindingType::Buffer {
                        ty: wgpu::BufferBindingType::Uniform,
                        has_dynamic_offset: false,
                        min_binding_size: wgpu::BufferSize::new(size_of::<TestUniform>() as u64),
                    },
                    count: None,
                },
                storage_entry(5, 8),
                wgpu::BindGroupLayoutEntry {
                    binding: 6,
                    visibility: wgpu::ShaderStages::COMPUTE,
                    ty: wgpu::BindingType::Buffer {
                        ty: wgpu::BufferBindingType::Storage { read_only: false },
                        has_dynamic_offset: false,
                        min_binding_size: wgpu::BufferSize::new(4),
                    },
                    count: None,
                },
                wgpu::BindGroupLayoutEntry {
                    binding: 4,
                    visibility: wgpu::ShaderStages::COMPUTE,
                    ty: wgpu::BindingType::StorageTexture {
                        access: wgpu::StorageTextureAccess::WriteOnly,
                        format: wgpu::TextureFormat::Rgba8Unorm,
                        view_dimension: wgpu::TextureViewDimension::D2,
                    },
                    count: None,
                },
            ],
        });
        let pipeline_layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
            label: Some("march_pipeline_layout"),
            bind_group_layouts: &[Some(&layout)],
            immediate_size: 0,
        });

        let pipeline = device.create_compute_pipeline(&wgpu::ComputePipelineDescriptor {
            label: Some("march_pipeline"),
            layout: Some(&pipeline_layout),
            module: &module,
            entry_point: Some(entry_point),
            compilation_options: Default::default(),
            cache: None,
        });
        let beam_pipeline = (flags & bevox_render::upload::march_flags::BEAM != 0).then(|| {
            device.create_compute_pipeline(&wgpu::ComputePipelineDescriptor {
                label: Some("beam_prepass_pipeline"),
                layout: Some(&pipeline_layout),
                module: &module,
                entry_point: Some("beam_prepass"),
                compilation_options: Default::default(),
                cache: None,
            })
        });
        let bind_group = device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: None,
            layout: &layout,
            entries: &[
                wgpu::BindGroupEntry { binding: 0, resource: uniform_buffer.as_entire_binding() },
                wgpu::BindGroupEntry { binding: 1, resource: node_buffer.as_entire_binding() },
                wgpu::BindGroupEntry { binding: 2, resource: voxel_buffer.as_entire_binding() },
                wgpu::BindGroupEntry { binding: 3, resource: palette_buffer.as_entire_binding() },
                wgpu::BindGroupEntry { binding: 5, resource: mask_buffer.as_entire_binding() },
                wgpu::BindGroupEntry { binding: 6, resource: beam_buffer.as_entire_binding() },
                wgpu::BindGroupEntry {
                    binding: 4,
                    resource: wgpu::BindingResource::TextureView(&view),
                },
            ],
        });

        Self { pipeline, beam_pipeline, bind_group, texture, node_buffer, voxel_buffer, width, height }
    }

    /// The prepass, when there is one, then the main pass. Separate passes, so
    /// the beam writes are visible to the reads that follow.
    fn encode(&self, encoder: &mut wgpu::CommandEncoder) {
        if let Some(beam) = &self.beam_pipeline {
            let mut pass = encoder.begin_compute_pass(&Default::default());
            pass.set_pipeline(beam);
            pass.set_bind_group(0, &self.bind_group, &[]);
            pass.dispatch_workgroups(
                self.width.div_ceil(BEAM_SCALE).max(1).div_ceil(8),
                self.height.div_ceil(BEAM_SCALE).max(1).div_ceil(8),
                1,
            );
        }
        let mut pass = encoder.begin_compute_pass(&Default::default());
        pass.set_pipeline(&self.pipeline);
        pass.set_bind_group(0, &self.bind_group, &[]);
        pass.dispatch_workgroups(self.width.div_ceil(8), self.height.div_ceil(8), 1);
    }

    /// Submits one dispatch without waiting; the caller polls once per batch.
    pub fn dispatch(&self, device: &wgpu::Device, queue: &wgpu::Queue) {
        let mut encoder = device.create_command_encoder(&Default::default());
        self.encode(&mut encoder);
        queue.submit([encoder.finish()]);
    }

    /// Dispatches once and reads the result back as RGBA bytes.
    pub fn read_back(&self, device: &wgpu::Device, queue: &wgpu::Queue) -> Vec<u8> {
        let mut encoder = device.create_command_encoder(&Default::default());
        self.encode(&mut encoder);
        read_texture(device, queue, encoder, &self.texture, self.width, self.height)
    }

    /// Writes a staged update exactly the way `prepare_march_buffers` does.
    ///
    /// Duplicating the offset arithmetic here would let the test agree with a
    /// bug, so this mirrors the app's rule explicitly: index 0 is the root,
    /// arena slot n is at index n + 1.
    pub fn apply_update(
        &self,
        queue: &wgpu::Queue,
        update: &bevox_render::upload::SceneUpdate,
    ) {
        queue.write_buffer(&self.node_buffer, 0, bytemuck::bytes_of(&update.root));
        for write in &update.nodes {
            let offset = u64::from(write.start + 1) * 16;
            queue.write_buffer(&self.node_buffer, offset, bytemuck::cast_slice(&write.nodes));
        }
        for write in &update.voxels {
            let offset = u64::from(write.start_word) * 4;
            queue.write_buffer(&self.voxel_buffer, offset, bytemuck::cast_slice(&write.words));
        }
    }
}
