//! Bind group layout, compute pipeline and the dispatch that runs it.

use crate::upload::{
    ExtractedMarchCamera, GpuSceneData, MarchTarget, MarchUniform, SceneUpdate, march_flags,
};
use bevy::prelude::*;
use bevy::material::bind_group_layout_entries::BindGroupLayoutEntries;
use bevy::material::bind_group_layout_entries::binding_types::{
    storage_buffer_read_only_sized, storage_buffer_sized, texture_storage_2d,
    uniform_buffer_sized,
};
use bevy::material::descriptor::BindGroupLayoutDescriptor;
use bevy::render::render_asset::RenderAssets;
use bevy::render::render_resource::*;
use core::num::NonZero;
use bevy::render::renderer::{RenderDevice, RenderQueue};
use bevy::render::texture::GpuImage;
use bevox_core::gpu::GpuNode;

pub const SHADER_PATH: &str = "shaders/march.wgsl";
pub const WORKGROUP: u32 = 8;
/// Full-resolution pixels per beam sample, per axis. Must match BEAM_SCALE in
/// the shader.
pub const BEAM_SCALE: u32 = 8;
/// Coarse pixels the beam buffer holds: enough for 7680x4320. The buffer is
/// built once with the scene, while the window size is only known later, so it
/// is sized for the largest window rather than resized. 2 MB.
pub const BEAM_CAPACITY: u32 = (7680 / BEAM_SCALE) * (4320 / BEAM_SCALE);

/// Bindings the shader declares, and therefore the number of entries
/// `init_march_pipeline`'s layout tuple must contain.
///
/// Adding a binding to the shader without adding a layout entry is not a
/// compile error and no parity test catches it -- the test harness builds its
/// own layout, so it kept passing while the app could not create its pipeline
/// at all. `the_layout_declares_every_binding_the_shader_uses` is the gate.
pub const MARCH_BINDING_COUNT: usize = 8;

/// Voxel data budget on the GPU. Checked at upload; exceeding it is an error,
/// never an allocation attempt.
pub const VOXEL_BUDGET_BYTES: u64 = 512 * 1024 * 1024;

/// Entries to allocate for a scene currently using `high_water` of them.
///
/// The headroom is what lets an edit allocate new nodes without forcing the
/// whole volume to be rebuilt. Doubling is bounded and monotonic; growing by a
/// fixed slack would stop helping once scenes got large.
pub fn buffer_capacity_for(high_water: u32) -> u32 {
    high_water.saturating_mul(2).max(1024)
}

/// Whether the volume's GPU arrays fit the budget.
///
/// Both arrays, not only the voxels: the node arena is as much of the volume's
/// data as the voxel bytes are, and at 16 bytes an entry it is usually the
/// larger of the two. Checking voxels alone left a scene free to allocate an
/// unbounded node buffer.
pub fn within_budget(node_capacity: u32, voxel_word_capacity: u32, field_words: u32) -> bool {
    budget_bytes(node_capacity, voxel_word_capacity, field_words) <= VOXEL_BUDGET_BYTES
}

/// Bytes the three storage buffers would occupy at these capacities.
pub fn budget_bytes(node_capacity: u32, voxel_word_capacity: u32, field_words: u32) -> u64 {
    u64::from(node_capacity) * size_of::<GpuNode>() as u64
        + u64::from(voxel_word_capacity) * 4
        + u64::from(field_words) * 4
}

#[derive(Resource)]
pub struct MarchPipeline {
    /// The created layout, used when building bind groups.
    pub layout: BindGroupLayout,
    pub pipeline: CachedComputePipelineId,
    /// Coarse pass, dispatched before the main one when the beam flag is set.
    pub beam: CachedComputePipelineId,
}

/// GPU-side buffers for the current scene.
#[derive(Resource)]
pub struct MarchBuffers {
    pub uniform: Buffer,
    pub nodes: Buffer,
    pub voxels: Buffer,
    pub palette: Buffer,
    pub direction_masks: Buffer,
    pub beam: Buffer,
    pub field: Buffer,
    pub generation: u32,
    pub node_capacity: u32,
    pub voxel_word_capacity: u32,
}

pub fn init_march_pipeline(
    mut commands: Commands,
    device: Res<RenderDevice>,
    asset_server: Res<AssetServer>,
    pipeline_cache: Res<PipelineCache>,
) {
    // MarchUniform is a plain Pod struct rather than a ShaderType, so the size
    // is stated directly. The upload test pins it at 128 bytes, which is what
    // keeps this number honest.
    let entries = BindGroupLayoutEntries::sequential(
        ShaderStages::COMPUTE,
        (
            uniform_buffer_sized(false, NonZero::new(size_of::<MarchUniform>() as u64)),
            // The shader declares these as array<vec4<u32>> and array<u32>, so
            // the minimum binding sizes are one element of each: 16 and 4. A
            // smaller minimum is rejected as "shader requirements against the
            // pipeline" at dispatch time, not at layout creation.
            storage_buffer_read_only_sized(false, NonZero::new(16)),
            storage_buffer_read_only_sized(false, NonZero::new(4)),
            // Palette: array<vec4<f32>>, so one element is 16 bytes.
            storage_buffer_read_only_sized(false, NonZero::new(16)),
            texture_storage_2d(TextureFormat::Rgba8Unorm, StorageTextureAccess::WriteOnly),
            storage_buffer_read_only_sized(false, NonZero::new(8)),
            // Beam prepass results: array<f32>, written by the prepass and read
            // by the main pass, so read_write rather than read_only.
            storage_buffer_sized(false, NonZero::new(4)),
            // Distance field: array<u32>, four cells packed per word.
            storage_buffer_read_only_sized(false, NonZero::new(4)),
        ),
    );

    // Two forms of the same layout: the pipeline descriptor takes a descriptor,
    // while building a bind group needs a created layout.
    let layout = device.create_bind_group_layout("bevox_march_layout", &entries);
    let layout_descriptor = BindGroupLayoutDescriptor::new("bevox_march_layout", &entries);

    let shader = asset_server.load(SHADER_PATH);
    let shader_beam = shader.clone();
    let layout_descriptor_beam = BindGroupLayoutDescriptor::new("bevox_march_layout", &entries);
    let pipeline = pipeline_cache.queue_compute_pipeline(ComputePipelineDescriptor {
        label: Some("bevox_march".into()),
        layout: vec![layout_descriptor],
        shader,
        entry_point: Some("march".into()),
        ..default()
    });

    let beam = pipeline_cache.queue_compute_pipeline(ComputePipelineDescriptor {
        label: Some("bevox_beam_prepass".into()),
        layout: vec![layout_descriptor_beam],
        shader: shader_beam,
        entry_point: Some("beam_prepass".into()),
        ..default()
    });

    commands.insert_resource(MarchPipeline { layout, pipeline, beam });
}

/// Uploads scene buffers and the per-frame uniform into the render world.
pub fn prepare_march_buffers(
    mut commands: Commands,
    device: Res<RenderDevice>,
    queue: Res<RenderQueue>,
    scene: Option<Res<GpuSceneData>>,
    update: Option<Res<SceneUpdate>>,
    camera: Option<Res<ExtractedMarchCamera>>,
    existing: Option<Res<MarchBuffers>>,
) {
    let (Some(scene), Some(camera)) = (scene, camera) else {
        return;
    };

    let uniform_value = MarchUniform {
        world_from_clip: camera.world_from_clip.to_cols_array_2d(),
        camera_position: camera.position.extend(0.0).to_array(),
        sun_direction: crate::upload::SUN_DIRECTION.normalize().extend(0.0).to_array(),
        volume_params: [scene.depth, scene.extent, crate::upload::march_flags::DEFAULT, 0],
        field_params: [scene.field_edge, 0, 0, 0],
    };

    // Reuse the buffers unless the scene was replaced outright or an edit grew
    // past the room they have. Both fall through to the rebuild below.
    if let Some(buffers) = existing
        && buffers.generation == scene.generation
        && update.as_ref().is_none_or(|u| {
            u.node_high_water < buffers.node_capacity
                && u.voxel_word_high_water <= buffers.voxel_word_capacity
        })
    {
        queue.write_buffer(&buffers.uniform, 0, bytemuck::bytes_of(&uniform_value));

        if let Some(update) = update {
            // Index 0 is the root, which lives outside the arena and so appears
            // in no dirty range. Arena slot n is therefore at index n + 1.
            queue.write_buffer(&buffers.nodes, 0, bytemuck::bytes_of(&update.root));
            for write in &update.nodes {
                let offset = u64::from(write.start + 1) * size_of::<GpuNode>() as u64;
                queue.write_buffer(&buffers.nodes, offset, bytemuck::cast_slice(&write.nodes));
            }
            for write in &update.voxels {
                let offset = u64::from(write.start_word) * 4;
                queue.write_buffer(&buffers.voxels, offset, bytemuck::cast_slice(&write.words));
            }
            for write in &update.field {
                let offset = u64::from(write.start_word) * 4;
                queue.write_buffer(&buffers.field, offset, bytemuck::cast_slice(&write.words));
            }
        }
        return;
    }

    let node_capacity = buffer_capacity_for(scene.nodes.len() as u32);
    let voxel_word_capacity = buffer_capacity_for(scene.voxels.len() as u32);
    let field_words = scene.distance_field.len() as u32;
    if !within_budget(node_capacity, voxel_word_capacity, field_words) {
        // Once, not every frame. The rejection returns without replacing the
        // buffers, so the condition holds again next frame and an `error!`
        // here would repeat at frame rate until the scene changed.
        error_once!(
            "volume needs {} MB ({} MB of nodes, {} MB of voxels, {} MB of field), over the {} MB budget;              scene not uploaded",
            budget_bytes(node_capacity, voxel_word_capacity, field_words) / (1024 * 1024),
            u64::from(node_capacity) * size_of::<GpuNode>() as u64 / (1024 * 1024),
            u64::from(voxel_word_capacity) * 4 / (1024 * 1024),
            u64::from(field_words) * 4 / (1024 * 1024),
            VOXEL_BUDGET_BYTES / (1024 * 1024)
        );
        return;
    }

    let mut node_bytes = bytemuck::cast_slice(&scene.nodes).to_vec();
    node_bytes.resize(node_capacity as usize * size_of::<GpuNode>(), 0);
    let mut voxel_bytes = bytemuck::cast_slice(&scene.voxels).to_vec();
    voxel_bytes.resize(voxel_word_capacity as usize * 4, 0);

    commands.insert_resource(MarchBuffers {
        uniform: device.create_buffer_with_data(&BufferInitDescriptor {
            label: Some("bevox_march_uniform"),
            contents: bytemuck::bytes_of(&uniform_value),
            usage: BufferUsages::UNIFORM | BufferUsages::COPY_DST,
        }),
        nodes: device.create_buffer_with_data(&BufferInitDescriptor {
            label: Some("bevox_nodes"),
            contents: &node_bytes,
            usage: BufferUsages::STORAGE | BufferUsages::COPY_DST,
        }),
        voxels: device.create_buffer_with_data(&BufferInitDescriptor {
            label: Some("bevox_voxels"),
            contents: &voxel_bytes,
            usage: BufferUsages::STORAGE | BufferUsages::COPY_DST,
        }),
        beam: device.create_buffer(&BufferDescriptor {
            label: Some("bevox_beam"),
            // One distance per coarse pixel. Sized for the largest target the
            // window can be, because it is built once and the window is not.
            //
            // Left zeroed, which is load-bearing: wgpu zero-initialises buffers,
            // and a zero seed means no offset. So the frames before the prepass
            // pipeline finishes compiling render correctly and merely slowly,
            // rather than seeding from uninitialised memory and dropping
            // geometry.
            size: u64::from(BEAM_CAPACITY) * 4,
            usage: BufferUsages::STORAGE,
            mapped_at_creation: false,
        }),
        direction_masks: device.create_buffer_with_data(&BufferInitDescriptor {
            label: Some("bevox_direction_masks"),
            contents: bytemuck::cast_slice(&scene.direction_masks),
            usage: BufferUsages::STORAGE,
        }),
        palette: device.create_buffer_with_data(&BufferInitDescriptor {
            label: Some("bevox_palette"),
            contents: bytemuck::cast_slice(&scene.palette),
            usage: BufferUsages::STORAGE,
        }),
        field: device.create_buffer_with_data(&BufferInitDescriptor {
            label: Some("bevox_field"),
            contents: bytemuck::cast_slice(&scene.distance_field),
            usage: BufferUsages::STORAGE | BufferUsages::COPY_DST,
        }),
        generation: scene.generation,
        node_capacity,
        voxel_word_capacity,
    });
}

/// Runs inside the render graph. Skips the frame if anything is not ready,
/// rather than panicking: a pipeline still compiling is normal, not an error.
pub fn dispatch_march(
    pipeline: Option<Res<MarchPipeline>>,
    buffers: Option<Res<MarchBuffers>>,
    target: Option<Res<MarchTarget>>,
    images: Res<RenderAssets<GpuImage>>,
    pipeline_cache: Res<PipelineCache>,
    device: Res<RenderDevice>,
    queue: Res<RenderQueue>,
) {
    let (Some(pipeline), Some(buffers), Some(target)) = (pipeline, buffers, target) else {
        return;
    };
    let Some(gpu_image) = images.get(&target.image) else {
        return;
    };
    let Some(compute) = pipeline_cache.get_compute_pipeline(pipeline.pipeline) else {
        return;
    };

    let bind_group = device.create_bind_group(
        "bevox_march_bind_group",
        &pipeline.layout,
        &BindGroupEntries::sequential((
            buffers.uniform.as_entire_binding(),
            buffers.nodes.as_entire_binding(),
            buffers.voxels.as_entire_binding(),
            buffers.palette.as_entire_binding(),
            &gpu_image.texture_view,
            buffers.direction_masks.as_entire_binding(),
            buffers.beam.as_entire_binding(),
            buffers.field.as_entire_binding(),
        )),
    );

    let mut encoder = device.create_command_encoder(&CommandEncoderDescriptor {
        label: Some("bevox_march_encoder"),
    });
    // Separate passes, so the prepass writes are visible to the reads that
    // follow. Skipped entirely when the flag is off: at one pixel in 64 it is
    // cheap, but not free.
    if march_flags::DEFAULT & march_flags::BEAM != 0
        && let Some(beam) = pipeline_cache.get_compute_pipeline(pipeline.beam)
    {
        let mut pass = encoder.begin_compute_pass(&ComputePassDescriptor {
            label: Some("bevox_beam_pass"),
            timestamp_writes: None,
        });
        pass.set_pipeline(beam);
        pass.set_bind_group(0, &bind_group, &[]);
        pass.dispatch_workgroups(
            target.width.div_ceil(BEAM_SCALE).max(1).div_ceil(WORKGROUP),
            target.height.div_ceil(BEAM_SCALE).max(1).div_ceil(WORKGROUP),
            1,
        );
    }
    {
        let mut pass = encoder.begin_compute_pass(&ComputePassDescriptor {
            label: Some("bevox_march_pass"),
            timestamp_writes: None,
        });
        pass.set_pipeline(compute);
        pass.set_bind_group(0, &bind_group, &[]);
        pass.dispatch_workgroups(
            target.width.div_ceil(WORKGROUP),
            target.height.div_ceil(WORKGROUP),
            1,
        );
    }
    queue.submit([encoder.finish()]);
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The app's bind group layout must cover every binding `march.wgsl`
    /// declares.
    ///
    /// This exists because it failed: the beam prepass added
    /// `@group(0) @binding(6)` and nothing added the matching layout entry.
    /// The whole suite stayed green -- the GPU harness declares its own
    /// layout -- and the app died on startup with "Shader global
    /// ResourceBinding { group: 0, binding: 6 } is not available in the
    /// pipeline layout".
    #[test]
    fn the_layout_declares_every_binding_the_shader_uses() {
        let source = std::fs::read_to_string(concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/assets/shaders/march.wgsl"
        ))
        .expect("shader missing");

        let mut bindings: Vec<usize> = source
            .lines()
            .filter_map(|line| line.split("@group(0) @binding(").nth(1))
            .filter_map(|rest| rest.split(')').next())
            .filter_map(|n| n.trim().parse().ok())
            .collect();
        bindings.sort_unstable();
        bindings.dedup();

        assert!(!bindings.is_empty(), "no bindings parsed; the scan is broken, not the layout");
        assert_eq!(
            bindings,
            (0..MARCH_BINDING_COUNT).collect::<Vec<_>>(),
            "march.wgsl declares bindings {bindings:?}, but the layout in              init_march_pipeline is built for {MARCH_BINDING_COUNT}. Add the missing              entry to the tuple and update MARCH_BINDING_COUNT together."
        );
    }

    #[test]
    fn capacity_leaves_room_to_grow() {
        // Growth headroom exists so that a small edit does not force a full
        // rebuild; the factor is arbitrary but the property is not.
        assert!(buffer_capacity_for(1000) > 1000);
        assert!(buffer_capacity_for(1000) <= 4000, "headroom should be bounded, not unbounded");
    }

    #[test]
    fn capacity_is_never_zero() {
        // A zero-length storage buffer is invalid, so an empty scene still gets
        // room for something.
        assert!(buffer_capacity_for(0) >= 1);
    }

    #[test]
    fn capacity_is_monotonic() {
        let mut last = 0;
        for hw in [0u32, 1, 10, 1_000, 100_000, 1_000_000] {
            let c = buffer_capacity_for(hw);
            assert!(c >= hw, "capacity {c} cannot hold {hw} entries");
            assert!(c >= last, "capacity went backwards as the scene grew");
            last = c;
        }
    }

    #[test]
    fn the_voxel_budget_is_the_number_the_spec_states() {
        assert_eq!(VOXEL_BUDGET_BYTES, 512 * 1024 * 1024);
    }

    #[test]
    fn a_scene_over_the_budget_is_rejected_rather_than_allocated() {
        // Checked at upload and exceeding it is an error, never an allocation
        // attempt -- so the check must be on the capacity actually requested.
        let over_on_voxels = (VOXEL_BUDGET_BYTES / 4) as u32 + 1;
        assert!(!within_budget(1, buffer_capacity_for(over_on_voxels), 0));
        assert!(within_budget(buffer_capacity_for(1000), buffer_capacity_for(1000), 0));
    }

    #[test]
    fn nodes_count_against_the_budget_too() {
        // A node is 16 bytes, so the node arena is usually the larger array.
        // Budgeting only the voxels let it grow without a bound.
        let over_on_nodes = (VOXEL_BUDGET_BYTES / size_of::<GpuNode>() as u64) as u32 + 1;
        assert!(!within_budget(buffer_capacity_for(over_on_nodes), 1, 0));
    }

    #[test]
    fn the_budget_counts_both_arrays_together() {
        // Each half fits on its own; together they do not.
        let half = (VOXEL_BUDGET_BYTES / 2) as u32;
        let nodes = half / size_of::<GpuNode>() as u32;
        let words = half / 4;
        assert!(within_budget(nodes, 1, 0));
        assert!(within_budget(1, words, 0));
        assert!(!within_budget(nodes + 1, words + 1, 0));
    }
}
