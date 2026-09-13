//! Bind group layout, compute pipeline and the dispatch that runs it.

use crate::upload::{ExtractedMarchCamera, GpuSceneData, MarchTarget, MarchUniform};
use bevy::prelude::*;
use bevy::material::bind_group_layout_entries::BindGroupLayoutEntries;
use bevy::material::bind_group_layout_entries::binding_types::{
    storage_buffer_read_only_sized, texture_storage_2d, uniform_buffer_sized,
};
use bevy::material::descriptor::BindGroupLayoutDescriptor;
use bevy::render::render_asset::RenderAssets;
use bevy::render::render_resource::*;
use core::num::NonZero;
use bevy::render::renderer::{RenderDevice, RenderQueue};
use bevy::render::texture::GpuImage;

pub const SHADER_PATH: &str = "shaders/march.wgsl";
pub const WORKGROUP: u32 = 8;

#[derive(Resource)]
pub struct MarchPipeline {
    /// The created layout, used when building bind groups.
    pub layout: BindGroupLayout,
    pub pipeline: CachedComputePipelineId,
}

/// GPU-side buffers for the current scene.
#[derive(Resource)]
pub struct MarchBuffers {
    pub uniform: Buffer,
    pub nodes: Buffer,
    pub voxels: Buffer,
    pub palette: Buffer,
    pub generation: u32,
}

pub fn init_march_pipeline(
    mut commands: Commands,
    device: Res<RenderDevice>,
    asset_server: Res<AssetServer>,
    pipeline_cache: Res<PipelineCache>,
) {
    // MarchUniform is a plain Pod struct rather than a ShaderType, so the size
    // is stated directly. The upload test pins it at 96 bytes, which is what
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
        ),
    );

    // Two forms of the same layout: the pipeline descriptor takes a descriptor,
    // while building a bind group needs a created layout.
    let layout = device.create_bind_group_layout("bevox_march_layout", &entries);
    let layout_descriptor = BindGroupLayoutDescriptor::new("bevox_march_layout", &entries);

    let shader = asset_server.load(SHADER_PATH);
    let pipeline = pipeline_cache.queue_compute_pipeline(ComputePipelineDescriptor {
        label: Some("bevox_march".into()),
        layout: vec![layout_descriptor],
        shader,
        entry_point: Some("march".into()),
        ..default()
    });

    commands.insert_resource(MarchPipeline { layout, pipeline });
}

/// Uploads scene buffers and the per-frame uniform into the render world.
pub fn prepare_march_buffers(
    mut commands: Commands,
    device: Res<RenderDevice>,
    queue: Res<RenderQueue>,
    scene: Option<Res<GpuSceneData>>,
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
        volume_params: [scene.depth, scene.extent, 0, 0],
    };

    // Rebuild storage buffers only when the scene changes; the uniform is
    // rewritten every frame because the camera moves every frame.
    if let Some(buffers) = existing
        && buffers.generation == scene.generation
    {
        queue.write_buffer(&buffers.uniform, 0, bytemuck::bytes_of(&uniform_value));
        return;
    }

    // A zero-length storage buffer is invalid, so an empty scene gets padding.
    let node_bytes = if scene.nodes.is_empty() {
        vec![0u8; 16]
    } else {
        bytemuck::cast_slice(&scene.nodes).to_vec()
    };
    let voxel_bytes = if scene.voxels.is_empty() {
        vec![0u8; 4]
    } else {
        bytemuck::cast_slice(&scene.voxels).to_vec()
    };

    commands.insert_resource(MarchBuffers {
        uniform: device.create_buffer_with_data(&BufferInitDescriptor {
            label: Some("bevox_march_uniform"),
            contents: bytemuck::bytes_of(&uniform_value),
            usage: BufferUsages::UNIFORM | BufferUsages::COPY_DST,
        }),
        nodes: device.create_buffer_with_data(&BufferInitDescriptor {
            label: Some("bevox_nodes"),
            contents: &node_bytes,
            usage: BufferUsages::STORAGE,
        }),
        voxels: device.create_buffer_with_data(&BufferInitDescriptor {
            label: Some("bevox_voxels"),
            contents: &voxel_bytes,
            usage: BufferUsages::STORAGE,
        }),
        palette: device.create_buffer_with_data(&BufferInitDescriptor {
            label: Some("bevox_palette"),
            contents: bytemuck::cast_slice(&scene.palette),
            usage: BufferUsages::STORAGE,
        }),
        generation: scene.generation,
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
        )),
    );

    let mut encoder = device.create_command_encoder(&CommandEncoderDescriptor {
        label: Some("bevox_march_encoder"),
    });
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
