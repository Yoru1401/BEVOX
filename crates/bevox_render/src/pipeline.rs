//! Bind group layout, compute pipeline and the dispatch that runs it.

use crate::cull::GpuBodyRect;
use crate::upload::{
    ExtractedMarchCamera, GpuBody, GpuSceneData, MarchTarget, MarchUniform, SceneUpdate,
    WorldRegion, march_flags,
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
pub const MARCH_BINDING_COUNT: usize = 10;

/// Voxel data budget on the GPU. Checked at upload; exceeding it is an error,
/// never an allocation attempt.
pub const VOXEL_BUDGET_BYTES: u64 = 512 * 1024 * 1024;

/// Bodies the march composes. Past this, bodies are still packed and uploaded
/// but not marched: the count the shader loops over is clamped to it, after the
/// cull, so it caps visible bodies.
///
/// Sixteen, measured rather than extrapolated. In
/// `bodies_are_measured_against_none` in `tests/gpu_bench.rs` (GTX 1650,
/// 2026-09-17, 1280x720, bench camera, at 8b59104) the static world marched in
/// 12.02 ms wall / 11.83 ms GPU, and sixteen visible bodies under `DEFAULT`'s
/// cull and rectangles in 16.33 / 16.12 ms, under a 16.7 ms frame. The slope
/// over 0, 1, 4 and 16 bodies is 0.265 ms wall / 0.273 ms GPU per body, which
/// would fit 17.7 / 17.9; sixteen is the largest count actually run.
///
/// This is a dispatch budget, not a frame: no present, no CPU frame work, and no
/// headroom for either. Most of the per-body cost is paid per pixel, so the
/// arithmetic is tied to 1280x720 and to bodies about 15,600 pixels on screen.
/// The plan's Measurements section has the numbers.
///
/// The test harness writes its own uniform and is not capped, which is what
/// lets the benchmark measure past this.
pub const MAX_BODIES: usize = 16;

/// How many of `bodies` the shader marches: all of them, up to `MAX_BODIES`.
///
/// Clamped, not rejected: the scene still renders, minus the bodies past the
/// cap. Reported once, like the voxel budget, because this runs every frame and
/// the condition holds until the scene changes.
pub fn marched_body_count(bodies: usize) -> u32 {
    if bodies > MAX_BODIES {
        error_once!("{bodies} bodies, over the cap of {MAX_BODIES}; marching only the first {MAX_BODIES}");
    }
    bodies.min(MAX_BODIES) as u32
}

/// The bodies a shadow ray tests: every placed body with voxels, up to
/// `MAX_BODIES`, each with its world bounding sphere filled in.
///
/// Never the culled table. The cull keeps what the camera can see, and a body
/// just off screen can shadow what is on it: taken from the table, its shadow
/// would pop in as the body came into view. A body with no voxels has no bound
/// and can shadow nothing, so it is left out.
pub fn shadow_casters(placed: &[GpuBody], local_bounds: &[Option<(UVec3, UVec3)>]) -> Vec<GpuBody> {
    placed
        .iter()
        .zip(local_bounds)
        .filter_map(|(g, local)| {
            local.map(|l| {
                let b = crate::cull::world_bound(l, g);
                GpuBody { bound: [b.centre.x, b.centre.y, b.centre.z, b.radius], ..*g }
            })
        })
        .take(MAX_BODIES)
        .collect()
}

/// Room for this many bodies in each half of the body buffer: the placed
/// count, and at least one, because a zero-length storage buffer is invalid.
pub fn body_room(placed: usize) -> usize {
    placed.max(1)
}

/// What the body buffer holds: the marched table from index 0, padded to
/// `room`, then the shadow casters from index `room`, padded to twice that.
///
/// One buffer, so shadows need no binding of their own. The uniform's
/// `field_params.zw` say how many casters there are and where they start.
pub fn body_buffer_contents(table: &[GpuBody], casters: &[GpuBody], room: usize) -> Vec<GpuBody> {
    debug_assert!(table.len() <= room && casters.len() <= room);
    let mut contents = table.to_vec();
    contents.resize(room, GpuBody::default());
    contents.extend_from_slice(casters);
    contents.resize(2 * room, GpuBody::default());
    contents
}

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
pub fn within_budget(
    node_capacity: u32,
    voxel_word_capacity: u32,
    field_words: u32,
    body_count: u32,
) -> bool {
    budget_bytes(node_capacity, voxel_word_capacity, field_words, body_count) <= VOXEL_BUDGET_BYTES
}

/// Bytes the scene's storage buffers would occupy at these capacities. A body
/// is two table entries, one marched and one casting shadows, and its screen
/// rectangle.
pub fn budget_bytes(
    node_capacity: u32,
    voxel_word_capacity: u32,
    field_words: u32,
    body_count: u32,
) -> u64 {
    u64::from(node_capacity) * size_of::<GpuNode>() as u64
        + u64::from(voxel_word_capacity) * 4
        + u64::from(field_words) * 4
        + u64::from(body_count) * BODY_BYTES
}

/// GPU bytes per body: its two table entries and its screen rectangle.
const BODY_BYTES: u64 = (2 * size_of::<GpuBody>() + size_of::<GpuBodyRect>()) as u64;

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
    pub bodies: Buffer,
    /// One `GpuBodyRect` per entry of `bodies`, in the same order.
    pub body_rects: Buffer,
    pub built_from: BuiltFrom,
}

/// The layout a set of buffers was built with.
///
/// One value, not two fields, so the reuse check takes the buffers' region
/// whole and its call site has no region to name -- and so none to confuse
/// with the scene's.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct BuiltFrom {
    pub generation: u32,
    /// The world's share of `nodes` and `voxels`, as the scene these buffers
    /// were built from laid it out. An incremental world write is allowed only
    /// inside it; past it are the bodies.
    pub world_region: WorldRegion,
}

/// Whether this frame's changes can be written into buffers built as
/// `built_from`, rather than rebuilding them from `scene`.
///
/// No if the scene was replaced outright or an edit grew the world past the
/// *buffers'* region. Not the scene's: on the frame an edit outgrows the region,
/// `build_gpu_scene` has already re-packed the bodies past a larger one, so the
/// fresh `scene` says the edit fits while the buffers still hold a body right
/// where it lands. This is the render world's half of what keeps a paint out
/// of the bodies; `build_gpu_scene` is the other.
pub fn can_reuse_buffers(
    built_from: BuiltFrom,
    scene: &GpuSceneData,
    update: Option<&SceneUpdate>,
) -> bool {
    built_from.generation == scene.generation
        && update.is_none_or(|u| {
            built_from.world_region.fits(u.node_high_water, u.voxel_word_high_water)
        })
}

/// This frame's uniform, the body table it marches, each body's screen
/// rectangle and the shadow casters: `camera` looking at `scene` through a
/// target of `size`, with `placed` culled under `flags` and at most
/// `MAX_BODIES` of what is left marched. The casters are not culled.
///
/// Returned together so the count, the table and the rectangles cannot come
/// from different lists. The cull compacts the table; a count taken from
/// `placed` would march past the entries written, into stale ones, and a
/// rectangle list built apart from it would pair a body with another's. Write
/// what this returns and nothing else. The cap applies after the cull, to
/// visible bodies.
pub fn frame_uniform(
    scene: &GpuSceneData,
    camera: &ExtractedMarchCamera,
    placed: &[GpuBody],
    flags: u32,
    size: UVec2,
) -> (MarchUniform, Vec<GpuBody>, Vec<GpuBodyRect>, Vec<GpuBody>) {
    let (table, rects) =
        crate::cull::bodies_to_march(placed, &scene.body_local_bounds, camera, flags, size);
    let casters = shadow_casters(placed, &scene.body_local_bounds);
    let uniform = MarchUniform {
        offset_from_clip: camera.offset_from_clip.to_cols_array_2d(),
        camera_position: camera.position.extend(0.0).to_array(),
        sun_direction: crate::upload::SUN_DIRECTION.normalize().extend(0.0).to_array(),
        volume_params: [scene.depth, scene.extent, flags, marched_body_count(table.len())],
        // The cell size travels with the edge count rather than a matching
        // shader-side constant, so `march.wgsl` cannot silently disagree with
        // `bevox_core::distance_field::CELL_VOXELS` about how big a cell is.
        field_params: [
            scene.field_edge,
            bevox_core::distance_field::CELL_VOXELS,
            casters.len() as u32,
            body_room(placed.len()) as u32,
        ],
    };
    (uniform, table, rects, casters)
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
            // Rigid bodies: array<GpuBody>, one element minimum.
            storage_buffer_read_only_sized(false, NonZero::new(size_of::<GpuBody>() as u64)),
            // Body screen rectangles: array<GpuBodyRect>, one element minimum.
            // The compute stage's eighth storage buffer, and wgpu's default
            // max_storage_buffers_per_shader_stage is 8: no room for another
            // storage binding without raising the device limits.
            storage_buffer_read_only_sized(false, NonZero::new(size_of::<GpuBodyRect>() as u64)),
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
    target: Option<Res<MarchTarget>>,
    images: Res<RenderAssets<GpuImage>>,
) {
    let (Some(scene), Some(camera)) = (scene, camera) else {
        return;
    };
    // The rectangles are in the pixels of the texture `dispatch_march` binds,
    // which the shader measures with `textureDimensions`: read from that same
    // `GpuImage`, not from `MarchTarget`'s recorded size, which could lag it.
    // Image assets are prepared before this set runs and not again before the
    // dispatch, so both see the same texture this frame. With no image yet,
    // nothing is dispatched this frame, so the rectangles written are never
    // read; they are rewritten every frame. Not a return: that would drop this
    // frame's edits.
    let size = target
        .and_then(|t| images.get(&t.image))
        .map_or(UVec2::ONE, |i| UVec2::new(i.texture.width(), i.texture.height()));

    // Anything `can_reuse_buffers` refuses falls through to the rebuild below,
    // which re-packs the bodies past a larger region.
    if let Some(buffers) = existing
        && can_reuse_buffers(buffers.built_from, &scene, update.as_deref())
    {
        // Every frame, edit or not: a body that moved changed only this table,
        // so it is rewritten here while its geometry stays put. Culled, it may
        // be shorter than the buffer, which was built for every body; entries
        // past it are stale, and the uniform's count stops before them. The
        // rectangles too, and even for a body that did not move: they follow
        // the camera.
        let placed = update.as_deref().map_or(&scene.bodies, |u| &u.bodies);
        let (uniform_value, table, rects, casters) =
            frame_uniform(&scene, &camera, placed, march_flags::DEFAULT, size);
        queue.write_buffer(&buffers.uniform, 0, bytemuck::bytes_of(&uniform_value));
        if !table.is_empty() {
            queue.write_buffer(&buffers.bodies, 0, bytemuck::cast_slice(&table));
            queue.write_buffer(&buffers.body_rects, 0, bytemuck::cast_slice(&rects));
        }
        // After the table's room, which is the room this buffer was built with:
        // a change in the number of bodies bumps the generation and rebuilds.
        if !casters.is_empty() {
            let at = (body_room(placed.len()) * size_of::<GpuBody>()) as u64;
            queue.write_buffer(&buffers.bodies, at, bytemuck::cast_slice(&casters));
        }

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

    // Sized as laid out: the world's region, then the bodies packed past it.
    // With no bodies the pack is just the world, and this is the region -- the
    // same capacity a body-free scene has always had. Bodies get no headroom of
    // their own: changing one's geometry bumps the generation and rebuilds.
    let node_capacity = scene.world_region.nodes.max(scene.nodes.len() as u32);
    let voxel_word_capacity = scene.world_region.voxel_words.max(scene.voxels.len() as u32);
    let field_words = scene.distance_field.len() as u32;
    // Room for every body, not only the ones visible now: the buffer outlives
    // this frame, and a later one may see more. A zero-length storage buffer is
    // invalid, so an empty body list still uploads room for one (zeroed)
    // GpuBody; the uniform's count stays at zero.
    let body_capacity = body_room(scene.bodies.len()) as u32;
    if !within_budget(node_capacity, voxel_word_capacity, field_words, body_capacity) {
        // Once, not every frame. The rejection returns without replacing the
        // buffers, so the condition holds again next frame and an `error!`
        // here would repeat at frame rate until the scene changed.
        error_once!(
            "volume needs {} MB ({} MB of nodes, {} MB of voxels, {} MB of field, {} MB of bodies), over the {} MB budget;              scene not uploaded",
            budget_bytes(node_capacity, voxel_word_capacity, field_words, body_capacity) / (1024 * 1024),
            u64::from(node_capacity) * size_of::<GpuNode>() as u64 / (1024 * 1024),
            u64::from(voxel_word_capacity) * 4 / (1024 * 1024),
            u64::from(field_words) * 4 / (1024 * 1024),
            u64::from(body_capacity) * BODY_BYTES / (1024 * 1024),
            VOXEL_BUDGET_BYTES / (1024 * 1024)
        );
        return;
    }

    let mut node_bytes = bytemuck::cast_slice(&scene.nodes).to_vec();
    node_bytes.resize(node_capacity as usize * size_of::<GpuNode>(), 0);
    let mut voxel_bytes = bytemuck::cast_slice(&scene.voxels).to_vec();
    voxel_bytes.resize(voxel_word_capacity as usize * 4, 0);
    let (uniform_value, table, rects, casters) =
        frame_uniform(&scene, &camera, &scene.bodies, march_flags::DEFAULT, size);
    let body_bytes = bytemuck::cast_slice(&body_buffer_contents(
        &table,
        &casters,
        body_capacity as usize,
    ))
    .to_vec();
    let mut rect_bytes = bytemuck::cast_slice(&rects).to_vec();
    rect_bytes.resize(body_capacity as usize * size_of::<GpuBodyRect>(), 0);

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
        bodies: device.create_buffer_with_data(&BufferInitDescriptor {
            label: Some("bevox_bodies"),
            contents: &body_bytes,
            usage: BufferUsages::STORAGE | BufferUsages::COPY_DST,
        }),
        body_rects: device.create_buffer_with_data(&BufferInitDescriptor {
            label: Some("bevox_body_rects"),
            contents: &rect_bytes,
            usage: BufferUsages::STORAGE | BufferUsages::COPY_DST,
        }),
        built_from: BuiltFrom { generation: scene.generation, world_region: scene.world_region },
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
            buffers.bodies.as_entire_binding(),
            buffers.body_rects.as_entire_binding(),
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
        assert!(!within_budget(1, buffer_capacity_for(over_on_voxels), 0, 0));
        assert!(within_budget(buffer_capacity_for(1000), buffer_capacity_for(1000), 0, 0));
    }

    #[test]
    fn nodes_count_against_the_budget_too() {
        // A node is 16 bytes, so the node arena is usually the larger array.
        // Budgeting only the voxels let it grow without a bound.
        let over_on_nodes = (VOXEL_BUDGET_BYTES / size_of::<GpuNode>() as u64) as u32 + 1;
        assert!(!within_budget(buffer_capacity_for(over_on_nodes), 1, 0, 0));
    }

    #[test]
    fn bodies_count_against_the_budget_too() {
        // Body geometry counts against the budget the same as the static world.
        let over_on_bodies = (VOXEL_BUDGET_BYTES / size_of::<GpuBody>() as u64) as u32 + 1;
        assert!(!within_budget(1, 1, 0, over_on_bodies));
        // And the rectangles beside them: this many table entries alone fit.
        let tables_fit = (VOXEL_BUDGET_BYTES / size_of::<GpuBody>() as u64) as u32;
        assert!(!within_budget(0, 0, 0, tables_fit), "the rectangle buffer is not budgeted");
    }

    #[test]
    fn bodies_past_the_cap_are_not_marched() {
        assert_eq!(marched_body_count(0), 0);
        assert_eq!(marched_body_count(MAX_BODIES), MAX_BODIES as u32);
        assert_eq!(
            marched_body_count(MAX_BODIES + 1),
            MAX_BODIES as u32,
            "one body past the cap was marched"
        );
    }

    const BUILT: BuiltFrom =
        BuiltFrom { generation: 3, world_region: WorldRegion { nodes: 100, voxel_words: 50 } };

    fn scene_at(generation: u32, world_region: WorldRegion) -> GpuSceneData {
        GpuSceneData { generation, world_region, ..default() }
    }

    fn update_at(node_high_water: u32, voxel_word_high_water: u32) -> SceneUpdate {
        SceneUpdate { node_high_water, voxel_word_high_water, ..default() }
    }

    #[test]
    fn buffers_are_reused_while_the_update_fits_their_region() {
        let scene = scene_at(BUILT.generation, BUILT.world_region);
        assert!(can_reuse_buffers(BUILT, &scene, None), "a frame with no update rebuilt");
        assert!(
            can_reuse_buffers(BUILT, &scene, Some(&update_at(99, 50))),
            "an update at the region's last legal marks rebuilt"
        );
    }

    /// The overflow frame. `build_gpu_scene` has already re-packed with a larger
    /// region that fits the edit, so the scene's copy says it fits; the buffers
    /// still hold body 0 where the edit lands. Judged by the scene's region,
    /// the paint is written into that body, and the new body table over the
    /// old geometry.
    #[test]
    fn an_update_that_fits_only_the_new_scene_region_rebuilds() {
        let scene = scene_at(BUILT.generation, WorldRegion { nodes: 400, voxel_words: 200 });
        for (nodes, words) in [(100, 50), (99, 51)] {
            assert!(
                scene.world_region.fits(nodes, words) && !BUILT.world_region.fits(nodes, words),
                "({nodes}, {words}) must fit the scene's region and not the buffers', or this \
                 case tests nothing"
            );
            assert!(
                !can_reuse_buffers(BUILT, &scene, Some(&update_at(nodes, words))),
                "({nodes}, {words}) outgrew the buffers' region, yet the buffers were reused"
            );
        }
    }

    #[test]
    fn a_new_generation_rebuilds() {
        let scene = scene_at(BUILT.generation + 1, BUILT.world_region);
        assert!(!can_reuse_buffers(BUILT, &scene, None));
        assert!(!can_reuse_buffers(BUILT, &scene, Some(&update_at(1, 1))));
    }

    /// A reverse-Z camera at the origin looking down -Z, an 8-voxel body 50
    /// ahead of it and the same body 50 behind, and that body's local bounds.
    fn camera_and_bodies() -> (ExtractedMarchCamera, GpuBody, GpuBody, (UVec3, UVec3)) {
        let view = Mat4::look_at_rh(Vec3::ZERO, -Vec3::Z, Vec3::Y);
        let projection = Mat4::perspective_infinite_reverse_rh(0.9, 16.0 / 9.0, 0.1);
        let camera = ExtractedMarchCamera {
            offset_from_clip: (projection * view).inverse(),
            position: Vec3::ZERO,
        };
        let at = |z: f32| {
            let body = bevox_core::body::Body::new(
                bevox_core::contree::Contree::empty(2),
                Vec3::new(-4.0, -4.0, z),
                Quat::IDENTITY,
            );
            GpuBody::default().placed(&body)
        };
        (camera, at(-50.0), at(50.0), (UVec3::ZERO, UVec3::splat(8)))
    }

    /// `bodies_past_the_cap_are_not_marched` proves the clamp; this proves the
    /// uniform the app uploads, with the app's flags, goes through it.
    #[test]
    fn the_uploaded_uniform_marches_no_more_than_the_cap() {
        let (camera, ahead, _, cube) = camera_and_bodies();
        let bodies = vec![ahead; MAX_BODIES + 1];
        let scene = GpuSceneData { body_local_bounds: vec![Some(cube); bodies.len()], ..default() };
        let (uniform, table, _, _) =
            frame_uniform(&scene, &camera, &bodies, march_flags::DEFAULT, UVec2::new(160, 90));
        // Every body is in view, so whether or not `DEFAULT` culls, only the
        // cap can stop the count short of them all.
        assert_eq!(table.len(), bodies.len(), "a body in view was culled; this case no longer reaches past the cap");
        assert_eq!(uniform.volume_params[3], MAX_BODIES as u32);
    }

    /// The count the shader loops over is the length of the culled table, not
    /// of the bodies placed. Counted from the placed list, a culled scene marches
    /// entries past the ones written.
    #[test]
    fn the_uniform_counts_the_culled_table_and_caps_after_the_cull() {
        let (camera, ahead, behind, cube) = camera_and_bodies();
        let scene = GpuSceneData { body_local_bounds: vec![Some(cube); 2 * MAX_BODIES + 1], ..default() };
        let cull = march_flags::DEFAULT | march_flags::CULL_BODIES;

        let size = UVec2::new(160, 90);
        let (uniform, table, rects, _) = frame_uniform(&scene, &camera, &[behind], cull, size);
        assert!(table.is_empty(), "a body behind the camera was kept");
        assert_eq!(uniform.volume_params[3], 0, "the count includes a body the cull removed");
        assert!(rects.is_empty(), "a rectangle was kept for a body the cull removed");

        // A cap's worth behind first, then one more than the cap ahead, so the
        // case reaches past the cap at any value. Capped before the cull, every
        // marched slot would go to a body the cull then drops, and nothing would
        // be drawn; capped after it, the cap's worth of bodies ahead are.
        let placed = [vec![behind; MAX_BODIES], vec![ahead; MAX_BODIES + 1]].concat();
        let (uniform, table, rects, _) = frame_uniform(&scene, &camera, &placed, cull, size);
        let kept = vec![ahead; MAX_BODIES + 1];
        assert_eq!(bytemuck::cast_slice::<GpuBody, u8>(&table), bytemuck::cast_slice::<GpuBody, u8>(&kept));
        let own =
            crate::cull::screen_rect(cube, &ahead, camera.offset_from_clip.inverse(), Vec3::ZERO, size);
        assert_eq!(rects, vec![own; MAX_BODIES + 1]);
        assert_eq!(uniform.volume_params[3], MAX_BODIES as u32);
    }

    /// Shadow rays need every body, not only the ones the camera sees: the
    /// casters include bodies the cull dropped, stop at the cap, and start at
    /// the table's room, which the uniform carries.
    #[test]
    fn the_shadow_casters_are_every_body_not_the_culled_table() {
        let bytes = |b: &[GpuBody]| bytemuck::cast_slice::<GpuBody, u8>(b).to_vec();
        let (camera, ahead, behind, cube) = camera_and_bodies();
        let scene = GpuSceneData { body_local_bounds: vec![Some(cube); 2 * MAX_BODIES + 1], ..default() };
        let cull = march_flags::DEFAULT | march_flags::CULL_BODIES;
        let size = UVec2::new(160, 90);

        let (uniform, table, _, casters) = frame_uniform(&scene, &camera, &[behind, ahead], cull, size);
        assert_eq!(table.len(), 1, "the cull no longer drops the body behind the camera");
        let placements = |b: &[GpuBody]| b.iter().map(|g| g.local_from_world).collect::<Vec<_>>();
        assert_eq!(placements(&casters), placements(&[behind, ahead]), "a body the cull dropped casts no shadow");
        for (c, g) in casters.iter().zip([behind, ahead]) {
            let b = crate::cull::world_bound(cube, &g);
            assert_eq!(c.bound, [b.centre.x, b.centre.y, b.centre.z, b.radius], "a caster's sphere is not its bound");
        }
        assert_eq!(uniform.field_params[2], 2);
        assert_eq!(uniform.field_params[3], 2, "the casters do not start past the table's room");

        let placed = vec![behind; 2 * MAX_BODIES + 1];
        let (uniform, _, _, casters) = frame_uniform(&scene, &camera, &placed, cull, size);
        assert_eq!(casters.len(), MAX_BODIES, "the casters are not capped");
        assert_eq!(uniform.field_params[2], MAX_BODIES as u32);
        assert_eq!(uniform.field_params[3], placed.len() as u32);

        let contents = body_buffer_contents(&table, &casters, placed.len());
        assert_eq!(contents.len(), 2 * placed.len());
        assert_eq!(bytes(&contents[..1]), bytes(&table));
        assert_eq!(bytes(&contents[placed.len()..placed.len() + MAX_BODIES]), bytes(&casters));

        // A body with no voxels has nothing to cast.
        let empty = GpuSceneData { body_local_bounds: vec![None, Some(cube)], ..default() };
        let (uniform, _, _, casters) = frame_uniform(&empty, &camera, &[behind, ahead], cull, size);
        assert_eq!(placements(&casters), placements(&[ahead]));
        assert_eq!(uniform.field_params[2], 1);
    }

    #[test]
    fn the_budget_counts_both_arrays_together() {
        // Each half fits on its own; together they do not.
        let half = (VOXEL_BUDGET_BYTES / 2) as u32;
        let nodes = half / size_of::<GpuNode>() as u32;
        let words = half / 4;
        assert!(within_budget(nodes, 1, 0, 0));
        assert!(within_budget(1, words, 0, 0));
        assert!(!within_budget(nodes + 1, words + 1, 0, 0));
    }
}
