//! Shared GPU test scaffolding.
//!
//! One definition of the bind group layout, the buffers and the pipeline, used
//! by both the parity tests and the benchmark. Two copies would drift, and a
//! harness that stops matching the layout the app ships stops testing it.

// Each integration test binary compiles this module separately, so items only
// one of them uses look dead to the other.
#![allow(dead_code)]

use std::sync::OnceLock;
use bevox_core::contree::Contree;
use bevox_core::dense::DenseVolume;
use bevox_core::gpu::GpuNode;
use bevox_core::material::{
    DEFAULT_DENSITY, DEFAULT_FRICTION, DEFAULT_RESTITUTION, Material, MaterialId, MaterialTable,
};
use bevox_render::cull::GpuBodyRect;
use glam::{Mat4, UVec3, Vec3};
use wgpu::util::DeviceExt;

#[repr(C)]
#[derive(Clone, Copy, bytemuck::Pod, bytemuck::Zeroable)]
pub struct TestUniform {
    /// Clip space to the offset from the eye, as `ExtractedMarchCamera` carries
    /// it: build it from a view with no translation, never from an absolute one.
    pub offset_from_clip: [[f32; 4]; 4],
    pub camera_position: [f32; 4],
    pub sun_direction: [f32; 4],
    /// `[depth, extent, flags, 0]`.
    pub volume_params: [u32; 4],
    /// `[field_edge, field_cell_size, 0, 0]`.
    pub field_params: [u32; 4],
    pub ao_params: [u32; 4],
}

/// Colours for the parity scene's two materials. A zeroed palette would render
/// every surface black, which is indistinguishable from a broken traversal.
pub fn parity_materials() -> MaterialTable {
    let mut table = MaterialTable::new();
    // 1: stone
    table
        .push(Material {
            color: [140, 140, 150, 255],
            density: DEFAULT_DENSITY,
            friction: DEFAULT_FRICTION,
            restitution: DEFAULT_RESTITUTION,
            strength: bevox_core::material::DEFAULT_STRENGTH,
        })
        .unwrap();
    // 2: brick
    table
        .push(Material {
            color: [180, 90, 70, 255],
            density: DEFAULT_DENSITY,
            friction: DEFAULT_FRICTION,
            restitution: DEFAULT_RESTITUTION,
            strength: bevox_core::material::DEFAULT_STRENGTH,
        })
        .unwrap();
    table
}

/// A solid 16-voxel cube in a 64 volume, as in Task 1's body tests, but one
/// voxel off the 4-voxel brick grid (25..41, not 24..40).
///
/// Aligned, every brick is full and collapses to a uniform node, so the body
/// owns no voxel bytes and a shader that ignored its `voxel_base` would still
/// read the right material. Off the grid, every surface brick is partial and
/// its material comes from the body's own voxel bytes.
pub fn body_cube() -> Contree {
    let mut dense = DenseVolume::new(64).unwrap();
    for z in 25..41 {
        for y in 25..41 {
            for x in 25..41 {
                dense.set(UVec3::new(x, y, z), MaterialId(1));
            }
        }
    }
    dense.into_contree()
}

/// Whether this device can report GPU time.
pub fn timestamps_supported(device: &wgpu::Device) -> bool {
    device.features().contains(wgpu::Features::TIMESTAMP_QUERY)
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
    // One device for every test in this binary, created on first use.
    //
    // Each test used to build its own `Instance`, adapter and `Device`, and the
    // test harness runs them on parallel threads, so a binary brought up a
    // dozen Vulkan devices against one GPU at once. That deadlocked in the
    // driver about one run in ten: a stress test of 20 runs at
    // --test-threads=16 hung twice, once before a single test finished and
    // once with twelve tests blocked at the same time, including the trivial
    // shader test that does nothing but acquire a device. The stuck process
    // burned CPU for hours and survived a session reset.
    //
    // `Device` and `Queue` are cheap handles that wgpu makes `Clone`, so every
    // test gets its own clone of the one device. Each test still builds its
    // own pipelines and buffers, so nothing a test checks is shared.
    static DEVICE: OnceLock<Option<(wgpu::Device, wgpu::Queue)>> = OnceLock::new();
    DEVICE
        .get_or_init(|| {
            pollster::block_on(async {
                let instance = wgpu::Instance::default();
                let adapter = instance
                    .request_adapter(&wgpu::RequestAdapterOptions::default())
                    .await
                    .ok()?;
                // Timestamps are requested when the adapter has them and silently
                // dropped when it does not, so a device without the feature still
                // runs every test -- it just cannot report GPU time.
                // `timestamps_supported` is how a caller finds out which it got.
                let features = adapter.features() & wgpu::Features::TIMESTAMP_QUERY;
                adapter
                    .request_device(&wgpu::DeviceDescriptor {
                        label: Some("bevox_test_device"),
                        required_features: features,
                        ..Default::default()
                    })
                    .await
                    .ok()
            })
        })
        .clone()
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
    field_buffer: wgpu::Buffer,
    /// Timestamps around each pass, when the device supports them. Four slots:
    /// beam begin/end then main begin/end, so a beam-less configuration simply
    /// leaves the first pair unwritten.
    timestamps: Option<Timestamps>,
    /// The body count the uniform carries, read back from the uniform itself.
    ///
    /// No pixel shows it: past the culled table the buffer holds zeroed
    /// entries, which draw nothing, so a count from the packed list renders
    /// the same image while marching bodies the cull removed -- and a
    /// benchmark would still pay for them. A gate compares this with the cull.
    pub body_count: u32,
    /// The rectangles uploaded, one per marched body, in table order.
    pub body_rects: Vec<GpuBodyRect>,
    body_rect_buffer: wgpu::Buffer,
    width: u32,
    height: u32,
}

/// Query set plus the two buffers a timestamp readback needs.
struct Timestamps {
    set: wgpu::QuerySet,
    /// Written by `resolve_query_set`; cannot be mapped directly.
    resolve: wgpu::Buffer,
    /// Mappable copy of `resolve`.
    readback: wgpu::Buffer,
}

/// Timestamp slots this configuration writes: two per pass that runs.
fn used_slots(prepared: &Prepared) -> u32 {
    if prepared.beam_pipeline.is_some() { 4 } else { 2 }
}

/// GPU time for one dispatched frame, in milliseconds.
#[derive(Clone, Copy, Debug, Default)]
pub struct GpuTime {
    /// The beam prepass, or 0.0 when the configuration has none.
    pub beam: f32,
    pub main: f32,
}

impl GpuTime {
    pub fn total(&self) -> f32 {
        self.beam + self.main
    }
}

impl Prepared {
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        device: &wgpu::Device,
        source: &str,
        entry_point: &str,
        tree: &Contree,
        offset_from_clip: Mat4,
        eye: Vec3,
        width: u32,
        height: u32,
        flags: u32,
        bodies: &[bevox_core::body::Body],
    ) -> Self {
        let field = bevox_core::distance_field::DistanceField::build(tree);
        // The static world packed with every body, in the same buffers and the
        // same layout `pack_bodies` gives the real render pipeline. An empty
        // body list packs out to exactly the static world, so this is also the
        // path every pre-bodies test still runs.
        let packed = bevox_render::upload::pack_bodies(tree, bodies);
        let texture = storage_target(device, width, height);
        let view = texture.create_view(&wgpu::TextureViewDescriptor::default());
        // Through the app's own cull, so the gates test what the app runs. The
        // count below is this table's length, never the packed one's: culled,
        // the table is compacted and shorter. The rectangles are sized from the
        // texture the shader writes, as the app sizes them.
        let (table, body_rects) = bevox_render::cull::bodies_to_march(
            &packed.bodies,
            &packed.body_local_bounds,
            &bevox_render::upload::ExtractedMarchCamera { offset_from_clip, position: eye },
            flags,
            glam::UVec2::new(texture.width(), texture.height()),
        );
        // The shadow casters are every body, laid out after the table's room,
        // as the app lays them out.
        let casters =
            bevox_render::pipeline::shadow_casters(&packed.bodies, &packed.body_local_bounds);
        let room = bevox_render::pipeline::body_room(packed.bodies.len());
        let uniform = TestUniform {
            offset_from_clip: offset_from_clip.to_cols_array_2d(),
            camera_position: eye.extend(0.0).to_array(),
            sun_direction: bevox_render::upload::SUN_DIRECTION
                .normalize()
                .extend(0.0)
                .to_array(),
            volume_params: [tree.depth(), tree.extent(), flags, table.len() as u32],
            // The cell size travels in the uniform rather than a matching
            // shader-side constant, so the shader cannot silently disagree
            // with `bevox_core::distance_field::CELL_VOXELS`.
            field_params: [
                field.edge(),
                bevox_core::distance_field::CELL_VOXELS,
                casters.len() as u32,
                room as u32,
            ],
            ao_params: [bevox_render::upload::field_words(&field), 0, 0, 0],
        };

        // Root first, arena shifted by one: the layout the shader indexes.
        let nodes = packed.nodes;
        let node_bytes: Vec<u8> = if nodes.is_empty() {
            vec![0u8; 16]
        } else {
            bytemuck::cast_slice(&nodes).to_vec()
        };
        // A zero-length storage buffer is invalid, so an empty volume gets padding.
        let voxel_bytes: Vec<u8> = if packed.voxels.is_empty() {
            vec![0u8; 4]
        } else {
            bytemuck::cast_slice(&packed.voxels).to_vec()
        };

        let node_capacity = bevox_render::pipeline::buffer_capacity_for(nodes.len() as u32);
        let mut node_bytes = node_bytes;
        node_bytes.resize(node_capacity as usize * size_of::<GpuNode>(), 0);

        let voxel_capacity =
            bevox_render::pipeline::buffer_capacity_for(packed.voxels.len() as u32);
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
        let fullness = bevox_core::fullness::Fullness::build(tree);
        let field_words = bevox_render::upload::pack_grids(&field, &fullness);
        let field_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("field"),
            contents: bytemuck::cast_slice(&field_words),
            usage: wgpu::BufferUsages::STORAGE | wgpu::BufferUsages::COPY_DST,
        });
        // Sized for every packed body, as the app sizes it, with the culled
        // table at the front and zeroes after. A zero-length storage buffer is
        // invalid, so a body-free scene still uploads room for one zeroed
        // GpuBody; the uniform's count, not the buffer length, is what the
        // shader loop actually reads.
        let body_bytes: Vec<u8> = bytemuck::cast_slice(
            &bevox_render::pipeline::body_buffer_contents(&table, &casters, room),
        )
        .to_vec();
        let body_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("bodies"),
            contents: &body_bytes,
            usage: wgpu::BufferUsages::STORAGE,
        });
        // Sized and zero-filled like the body table.
        let mut rect_bytes: Vec<u8> = bytemuck::cast_slice(&body_rects).to_vec();
        rect_bytes.resize(packed.bodies.len().max(1) * size_of::<GpuBodyRect>(), 0);
        let body_rect_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("body_rects"),
            contents: &rect_bytes,
            usage: wgpu::BufferUsages::STORAGE | wgpu::BufferUsages::COPY_DST,
        });

        let module = device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("march"),
            source: wgpu::ShaderSource::Wgsl(source.into()),
        });

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
                storage_entry(7, 4),
                storage_entry(8, size_of::<bevox_render::upload::GpuBody>() as u64),
                storage_entry(9, size_of::<GpuBodyRect>() as u64),
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
                wgpu::BindGroupEntry { binding: 7, resource: field_buffer.as_entire_binding() },
                wgpu::BindGroupEntry { binding: 8, resource: body_buffer.as_entire_binding() },
                wgpu::BindGroupEntry { binding: 9, resource: body_rect_buffer.as_entire_binding() },
                wgpu::BindGroupEntry {
                    binding: 4,
                    resource: wgpu::BindingResource::TextureView(&view),
                },
            ],
        });

        let timestamps = timestamps_supported(device).then(|| {
            let slots = 4;
            Timestamps {
                set: device.create_query_set(&wgpu::QuerySetDescriptor {
                    label: Some("march_timestamps"),
                    ty: wgpu::QueryType::Timestamp,
                    count: slots,
                }),
                resolve: device.create_buffer(&wgpu::BufferDescriptor {
                    label: Some("march_timestamp_resolve"),
                    size: u64::from(slots) * 8,
                    usage: wgpu::BufferUsages::QUERY_RESOLVE | wgpu::BufferUsages::COPY_SRC,
                    mapped_at_creation: false,
                }),
                readback: device.create_buffer(&wgpu::BufferDescriptor {
                    label: Some("march_timestamp_readback"),
                    size: u64::from(slots) * 8,
                    usage: wgpu::BufferUsages::COPY_DST | wgpu::BufferUsages::MAP_READ,
                    mapped_at_creation: false,
                }),
            }
        });

        Self {
            pipeline,
            beam_pipeline,
            bind_group,
            texture,
            node_buffer,
            voxel_buffer,
            field_buffer,
            timestamps,
            body_count: uniform.volume_params[3],
            body_rects,
            body_rect_buffer,
            width,
            height,
        }
    }

    /// The prepass, when there is one, then the main pass. Separate passes, so
    /// the beam writes are visible to the reads that follow.
    fn encode(&self, encoder: &mut wgpu::CommandEncoder, timed: bool) {
        let writes = |begin: u32, end: u32| {
            self.timestamps.as_ref().filter(|_| timed).map(|t| wgpu::ComputePassTimestampWrites {
                query_set: &t.set,
                beginning_of_pass_write_index: Some(begin),
                end_of_pass_write_index: Some(end),
            })
        };

        // Slots are packed, not fixed: resolving a query that was never
        // written is a validation error, so a configuration without a beam
        // pass must not leave a hole at the start of the set.
        let (beam_slots, main_slots) = if self.beam_pipeline.is_some() {
            ((0u32, 1u32), (2u32, 3u32))
        } else {
            ((0, 1), (0, 1))
        };

        if let Some(beam) = &self.beam_pipeline {
            let mut pass = encoder.begin_compute_pass(&wgpu::ComputePassDescriptor {
                label: Some("beam_pass"),
                timestamp_writes: writes(beam_slots.0, beam_slots.1),
            });
            pass.set_pipeline(beam);
            pass.set_bind_group(0, &self.bind_group, &[]);
            pass.dispatch_workgroups(
                self.width.div_ceil(BEAM_SCALE).max(1).div_ceil(8),
                self.height.div_ceil(BEAM_SCALE).max(1).div_ceil(8),
                1,
            );
        }
        let mut pass = encoder.begin_compute_pass(&wgpu::ComputePassDescriptor {
            label: Some("march_pass"),
            timestamp_writes: writes(main_slots.0, main_slots.1),
        });
        pass.set_pipeline(&self.pipeline);
        pass.set_bind_group(0, &self.bind_group, &[]);
        pass.dispatch_workgroups(self.width.div_ceil(8), self.height.div_ceil(8), 1);
        drop(pass);

        if timed && let Some(t) = &self.timestamps {
            let used = if self.beam_pipeline.is_some() { 4 } else { 2 };
            encoder.resolve_query_set(&t.set, 0..used, &t.resolve, 0);
            encoder.copy_buffer_to_buffer(&t.resolve, 0, &t.readback, 0, u64::from(used) * 8);
        }
    }

    /// Dispatches once and reads back what the GPU says each pass took.
    ///
    /// Returns None when the adapter has no timestamp support, which is a
    /// normal state rather than a failure -- the wall-clock timings still work.
    ///
    /// This is a whole round trip per call: submit, wait, map, unmap. It is for
    /// reporting a number, not for use inside a timed loop, where the waiting
    /// would dominate the thing being measured.
    pub fn dispatch_timed(&self, device: &wgpu::Device, queue: &wgpu::Queue) -> Option<GpuTime> {
        let t = self.timestamps.as_ref()?;
        let period = queue.get_timestamp_period();

        let mut encoder = device.create_command_encoder(&Default::default());
        self.encode(&mut encoder, true);
        queue.submit([encoder.finish()]);

        t.readback.map_async(wgpu::MapMode::Read, ..u64::from(used_slots(self)) * 8, |_| {});
        device.poll(wgpu::PollType::wait_indefinitely()).expect("poll");

        let used = if self.beam_pipeline.is_some() { 4usize } else { 2 };
        let ticks: Vec<u64> = {
            let view = t.readback.get_mapped_range(..u64::from(used as u32) * 8);
            bytemuck::cast_slice::<u8, u64>(&view).to_vec()
        };
        t.readback.unmap();

        // Ticks are a monotonic counter; a pass that never ran leaves its pair
        // untouched, and saturating_sub keeps that at zero rather than wrapping
        // into a nonsense duration.
        let ms = |a: u64, b: u64| b.saturating_sub(a) as f32 * period / 1_000_000.0;
        if self.beam_pipeline.is_some() {
            Some(GpuTime { beam: ms(ticks[0], ticks[1]), main: ms(ticks[2], ticks[3]) })
        } else {
            Some(GpuTime { beam: 0.0, main: ms(ticks[0], ticks[1]) })
        }
    }

    /// Submits one dispatch without waiting; the caller polls once per batch.
    pub fn dispatch(&self, device: &wgpu::Device, queue: &wgpu::Queue) {
        let mut encoder = device.create_command_encoder(&Default::default());
        self.encode(&mut encoder, false);
        queue.submit([encoder.finish()]);
    }

    /// Dispatches once and reads the result back as RGBA bytes.
    pub fn read_back(&self, device: &wgpu::Device, queue: &wgpu::Queue) -> Vec<u8> {
        let mut encoder = device.create_command_encoder(&Default::default());
        self.encode(&mut encoder, false);
        read_texture(device, queue, encoder, &self.texture, self.width, self.height)
    }

    /// Replaces the uploaded rectangles, so a gate can show the shader reads
    /// them. `body_rects` is left as built.
    pub fn overwrite_body_rects(&self, queue: &wgpu::Queue, rects: &[GpuBodyRect]) {
        queue.write_buffer(&self.body_rect_buffer, 0, bytemuck::cast_slice(rects));
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
            let offset = u64::from(write.start + 1) * size_of::<GpuNode>() as u64;
            queue.write_buffer(&self.node_buffer, offset, bytemuck::cast_slice(&write.nodes));
        }
        for write in &update.voxels {
            let offset = u64::from(write.start_word) * 4;
            queue.write_buffer(&self.voxel_buffer, offset, bytemuck::cast_slice(&write.words));
        }
        for write in &update.field {
            let offset = u64::from(write.start_word) * 4;
            queue.write_buffer(&self.field_buffer, offset, bytemuck::cast_slice(&write.words));
        }
    }
}
