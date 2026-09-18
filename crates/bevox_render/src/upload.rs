//! Moving voxel data and camera parameters into the render world.

use crate::pipeline::buffer_capacity_for;
use bevy::asset::RenderAssetUsages;
use bevy::prelude::*;
use bevy::render::Extract;
use bevy::render::extract_resource::ExtractResource;
use bevy::render::render_resource::{Extent3d, TextureDimension, TextureFormat, TextureUsages};
use bevox_core::body::{Body, occupied_bounds};
use bevox_core::contree::Contree;
use bevox_core::distance_field::DistanceField;
use bevox_core::gpu::{GpuNode, GpuVolume};
use bevox_core::mask_table::build_direction_masks;
use bevox_core::material::{MaterialId, MaterialTable};
use bytemuck::{Pod, Zeroable};
use std::ops::Range;

/// The scene the renderer draws. Replacing it re-uploads on the next frame.
#[derive(Resource)]
pub struct VoxelScene {
    pub tree: Contree,
    /// Colours the voxel material indices name.
    pub materials: MaterialTable,
    /// Bumped whenever `tree` changes, so the render world knows to re-upload.
    pub generation: u32,
    /// Coarse distance-to-solid grid, kept in step with `tree` by the brush.
    pub field: DistanceField,
    /// Cell range the brush lowered since the last `stage_scene_update`, if any.
    ///
    /// `None` after an erase: removing geometry only raises true distances, so
    /// a stale field merely under-estimates -- costing speed, never
    /// correctness -- and needs no upload at all.
    pub field_dirty: Option<Range<u32>>,
    /// Rigid bodies, packed past the world's region of the shared node and
    /// voxel buffers.
    ///
    /// A body that only moves -- a new `position` or `orientation` -- needs
    /// nothing: its table entry is rebuilt and re-uploaded every frame, and
    /// that table is a few 160-byte entries. A body whose *geometry* changes,
    /// or a body added or removed, needs `generation` bumped, because that
    /// moves bytes in the large node and voxel buffers. Do not bump the
    /// generation to move a body: that rebuilds the whole scene every frame.
    ///
    /// Ordering, for any system that writes this -- physics included. One that
    /// changes a body's geometry, or adds, removes or reorders bodies, must
    /// bump `generation` and run before `build_gpu_scene`. One that only
    /// changes a transform must run before `stage_scene_update_system`. The
    /// staged table pairs `GpuSceneData::bodies` with these by index, so
    /// breaking either rule draws a transform on the wrong body's geometry for
    /// a frame.
    pub bodies: Vec<Body>,
}

/// Camera and volume parameters, as the shader sees them.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, Pod, Zeroable)]
pub struct MarchUniform {
    /// Clip space to the world-space offset from `camera_position`. See
    /// `ExtractedMarchCamera::offset_from_clip`.
    pub offset_from_clip: [[f32; 4]; 4],
    pub camera_position: [f32; 4],
    /// Normalised direction *toward* the sun.
    pub sun_direction: [f32; 4],
    /// `[depth, extent, march_flags, 0]`.
    pub volume_params: [u32; 4],
    /// `[field_edge, field_cell_size, 0, 0]`.
    pub field_params: [u32; 4],
}

/// Traversal optimisations, carried in `volume_params.z`.
///
/// One shader renders both sides of every comparison, so a bit-identity test
/// cannot accidentally compare two different builds.
pub mod march_flags {
    pub const NONE: u32 = 0;
    pub const DDA: u32 = 1;
    pub const MASK_FILTER: u32 = 2;
    pub const BEAM: u32 = 4;
    /// Advance the ray through empty space the distance field can prove clear.
    pub const DISTANCE_FIELD: u32 = 8;
    /// Compose every rigid body into the march alongside the static world.
    pub const BODIES: u32 = 16;
    /// Leave bodies the camera cannot see out of the body table.
    ///
    /// CPU-only: the shader never reads this bit. It changes which bodies are
    /// uploaded and the count beside them, not how any of them is marched.
    pub const CULL_BODIES: u32 = 32;
    /// Skip a body for pixels outside its screen rectangle, before its table
    /// entry is read or the ray transformed. Read by the shader, unlike
    /// `CULL_BODIES`.
    pub const BODY_RECT: u32 = 64;
    /// Shadow rays test every body as well as the static world, so bodies cast
    /// shadows. Read by the shader. The bodies it tests are
    /// `pipeline::shadow_casters`, never the culled table.
    pub const BODY_SHADOWS: u32 = 128;
    /// What the app runs. Each optimisation joins this only once it has measured
    /// faster while staying bit-identical.
    ///
    /// All four earned it. A/B/A at 1280x720, extent 1024, close to geometry:
    /// scan 34.6 ms, mask 30.0, field 22.0, beam 24.5, DDA 19.7, the first
    /// three 14.0, all four 12.25 -- a 64.6% gain against 0.01 ms of drift.
    /// The field is worth 12.8% on top of the other three, which is why it is
    /// here rather than reverted.
    ///
    /// `BODIES` is here on different grounds: it is a feature, not an
    /// optimisation, and without it no body renders at all. It costs a
    /// body-free scene nothing -- the composition loop runs zero times, and
    /// the zero-body parity gate proves such a scene bit-identical with it on.
    /// What each body costs is measured separately.
    ///
    /// `CULL_BODIES` and `BODY_RECT` both earned it in
    /// `bodies_are_measured_against_none` (GTX 1650, 1280x720, bench camera,
    /// 2026-09-17, at 8b59104), each A/B/A directly against neither on the same
    /// sixteen bodies, wall / GPU. The cull saves 48.04 / 48.40 ms on bodies
    /// behind the camera (drift 0.15 / 1.60) and is within drift where it culls
    /// nothing. The rectangle saves 48.49 / 48.35 ms on loose bodies in view
    /// (drift 0.19 / 0.09) and 47.31 / 47.89 ms on tight ones (drift 0.26 /
    /// 0.27). Alone it saves nothing behind the camera, where a body gets the
    /// whole screen; the cull removes those. Neither changes a pixel, and a
    /// body-free scene runs neither.
    ///
    /// `BODY_SHADOWS` is a feature, as `BODIES` is, and Flori chose it on
    /// knowing its cost (2026-09-18). In `body_shadows_are_measured` (GTX 1650,
    /// 1280x720, bench camera, A/B/A against the same flags without it, wall /
    /// GPU): a body-free scene +0.04 / -0.00 ms, so no codegen cliff; sixteen
    /// bodies behind the camera -0.23 / -0.34 ms, thanks to the sphere test;
    /// sixteen in view, shadowing 88,171 pixels, +4.85 / +5.31 ms, for 22.6 ms
    /// in all against a 16.7 ms frame. That last is the bench's worst case, a
    /// wall of cubes across the view.
    pub const DEFAULT: u32 = DDA
        | MASK_FILTER
        | BEAM
        | DISTANCE_FIELD
        | BODIES
        | CULL_BODIES
        | BODY_RECT
        | BODY_SHADOWS;
}

/// The sun direction the renderer and the parity tests share.
///
/// Shared rather than duplicated so a test can never pass by lighting the scene
/// differently from the thing it is checking.
pub const SUN_DIRECTION: Vec3 = Vec3::new(0.4, 1.0, 0.25);

/// The storage texture the compute shader writes and a sprite displays.
#[derive(Resource, Clone, ExtractResource)]
pub struct MarchTarget {
    pub image: Handle<Image>,
    pub width: u32,
    pub height: u32,
}

/// One body as the shader reads it.
///
/// Carries the inverse placement because that is the direction a ray travels —
/// world into local — and the rotation separately because a normal rotates
/// back without the translation.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, Pod, Zeroable)]
pub struct GpuBody {
    pub local_from_world: [[f32; 4]; 4],
    pub rotation: [[f32; 4]; 4],
    /// Index of this body's root in the shared node buffer. Its arena slot `n`
    /// is at `node_base + 1 + n`, matching how the static world is laid out.
    pub node_base: u32,
    pub voxel_base: u32,
    pub depth: u32,
    pub extent: u32,
    /// The body's world bounding sphere, centre then radius, so a shadow ray
    /// can pass a body it goes nowhere near before transforming anything.
    /// Filled in only for the shadow casters (`pipeline::shadow_casters`); the
    /// marched table leaves it zero.
    pub bound: [f32; 4],
}

impl GpuBody {
    /// This entry's geometry, placed where `body` is now.
    ///
    /// The one spelling of a body's transform, shared by the full pack and the
    /// per-frame table refresh so the two cannot disagree.
    pub fn placed(self, body: &Body) -> Self {
        // Here as well as in `Body::new`: the fields are public, and this is
        // where every orientation passes on its way to the shader.
        debug_assert!(
            body.orientation.is_normalized(),
            "body orientation {:?} is not a unit quaternion; it would scale `t` in the body's \
             frame and break the nearest-hit composition",
            body.orientation
        );
        Self {
            local_from_world: Mat4::from(body.local_from_world()).to_cols_array_2d(),
            rotation: Mat4::from_quat(body.orientation).to_cols_array_2d(),
            ..self
        }
    }
}

/// How much of the shared buffers belongs to the static world: its data and
/// the headroom its edits grow into. Bodies are packed past it.
///
/// Carried from the pack through `GpuSceneData` into the render buffers rather
/// than recomputed, so the check that allows an incremental world write and
/// the layout that check protects cannot disagree.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct WorldRegion {
    /// Node entries, the root included.
    pub nodes: u32,
    pub voxel_words: u32,
}

impl WorldRegion {
    /// The region for a world currently using this many entries of each.
    pub fn around(nodes: u32, voxel_words: u32) -> Self {
        Self { nodes: buffer_capacity_for(nodes), voxel_words: buffer_capacity_for(voxel_words) }
    }

    /// Whether a world grown to these high-water marks is still inside the
    /// region. False means rebuild: an incremental write past the region would
    /// land in the first body. `node_high_water` counts arena slots, and slot
    /// `n` is at index `n + 1`, hence the strict comparison.
    pub fn fits(&self, node_high_water: u32, voxel_word_high_water: u32) -> bool {
        node_high_water < self.nodes && voxel_word_high_water <= self.voxel_words
    }
}

/// The static world and every body, packed into shared buffers.
pub struct PackedScene {
    pub nodes: Vec<GpuNode>,
    pub voxels: Vec<u32>,
    pub bodies: Vec<GpuBody>,
    /// Each body's `occupied_bounds`, in the same order as `bodies`.
    pub body_local_bounds: Vec<Option<(UVec3, UVec3)>>,
    pub world_region: WorldRegion,
}

/// Packs the static world followed by each body.
///
/// One set of buffers rather than one per body: a body is small, and a second
/// pair of bindings per body would cap the count at whatever the device allows
/// rather than at what the frame budget allows.
///
/// Bodies start past the world's whole region, not at the end of its current
/// data. An edit grows the world by appending to its arena, and those writes go
/// out incrementally; packed tight, the first allocating paint would overwrite
/// the first body. With no bodies there is nothing to protect, so the world is
/// not padded and a body-free scene packs exactly as it always has.
pub fn pack_bodies(world: &Contree, bodies: &[Body]) -> PackedScene {
    let world_volume = GpuVolume::from_contree(world);
    let mut nodes = world_volume.buffer_nodes();
    let mut voxels = world_volume.voxels;
    let world_region = WorldRegion::around(nodes.len() as u32, voxels.len() as u32);
    if !bodies.is_empty() {
        nodes.resize(world_region.nodes as usize, GpuNode::default());
        voxels.resize(world_region.voxel_words as usize, 0);
    }
    let mut out = Vec::with_capacity(bodies.len());
    let body_local_bounds = bodies.iter().map(|b| occupied_bounds(&b.volume)).collect();

    for body in bodies {
        let volume = GpuVolume::from_contree(&body.volume);
        let node_base = nodes.len() as u32;
        let voxel_base = voxels.len() as u32;
        nodes.extend_from_slice(&volume.buffer_nodes());
        voxels.extend_from_slice(&volume.voxels);

        out.push(
            GpuBody {
                node_base,
                voxel_base,
                depth: body.volume.depth(),
                extent: body.volume.extent(),
                ..default()
            }
            .placed(body),
        );
    }

    PackedScene { nodes, voxels, bodies: out, body_local_bounds, world_region }
}

/// The scene as the render world sees it.
///
/// Extracted by `extract_gpu_scene` rather than `ExtractResourcePlugin`, which
/// would clone the whole thing every frame. The render world keeps its copy
/// across frames and only needs a fresh one when the scene is actually rebuilt.
#[derive(Resource, Clone)]
pub struct GpuSceneData {
    pub nodes: Vec<GpuNode>,
    pub voxels: Vec<u32>,
    /// Every rigid body in the scene, packed past `world_region` in `nodes`
    /// and `voxels`. The transforms are as of the last rebuild; the layout --
    /// bases, depth, extent -- is what `SceneUpdate::bodies` re-places each
    /// frame.
    pub bodies: Vec<GpuBody>,
    /// Each body's `occupied_bounds`, in the same order as `bodies`: geometry,
    /// so it changes only on a rebuild. The world-space bound is recomputed
    /// from the live transform every frame.
    pub body_local_bounds: Vec<Option<(UVec3, UVec3)>>,
    pub palette: Vec<[f32; 4]>,
    /// Reachability masks as low/high halves: WGSL has no 64-bit integer.
    /// Constant, so it is built once rather than per scene.
    pub direction_masks: Vec<[u32; 2]>,
    pub depth: u32,
    pub extent: u32,
    /// Chebyshev distance to the nearest solid voxel per coarse cell, packed
    /// four to a word.
    pub distance_field: Vec<u32>,
    /// Cells per axis, so the shader can index the grid.
    pub field_edge: u32,
    pub generation: u32,
    /// The world's share of `nodes` and `voxels`, headroom included.
    pub world_region: WorldRegion,
}

impl Default for GpuSceneData {
    /// A single empty root node: a valid volume that every ray misses.
    ///
    /// This resource always exists, so the dispatch is never gated on a scene
    /// being loaded. An empty world must render sky, not a black screen.
    fn default() -> Self {
        Self {
            nodes: vec![GpuNode::default()],
            voxels: Vec::new(),
            bodies: Vec::new(),
            body_local_bounds: Vec::new(),
            palette: MaterialTable::new().to_gpu(),
            direction_masks: gpu_direction_masks(),
            // Depth must be at least 1: the shader starts at level `depth - 1`.
            depth: 1,
            extent: 4,
            // A single zero cell claims no empty space anywhere, the safe
            // value for an empty scene.
            distance_field: vec![0],
            field_edge: 1,
            generation: 0,
            world_region: WorldRegion::around(1, 0),
        }
    }
}

/// Whether the render world's copy of the scene is stale.
///
/// `changed` is the main-world resource's change flag, which `build_gpu_scene`
/// raises only when it actually rebuilds. That covers both of its triggers --
/// a replaced scene and an edit outgrowing the buffers -- where comparing
/// generations alone would miss the second, since an overflow rebuild
/// deliberately leaves the generation where it is.
///
/// The generation comparison is belt and braces for the one failure that would
/// be worst: a missed change tick on the load path leaving the render world
/// showing nothing at all.
pub fn scene_copy_is_stale(
    changed: bool,
    render_generation: Option<u32>,
    main_generation: u32,
) -> bool {
    match render_generation {
        // The render world has no copy yet, so anything is news.
        None => true,
        Some(g) => changed || g != main_generation,
    }
}

/// Copies the scene into the render world, but only when it changed.
///
/// The per-frame cost of the blanket plugin was the whole payload: at extent
/// 1024 the benchmark scene is 1.16 MB of nodes and 2.13 MB of voxels, and a
/// composed `.vox` scene at extent 4096 runs to tens of megabytes. None of it
/// changes on a frame where nothing was edited, and the render world reads only
/// `depth`, `extent` and `generation` on such frames anyway.
pub fn extract_gpu_scene(
    mut commands: Commands,
    scene: Extract<Res<GpuSceneData>>,
    existing: Option<Res<GpuSceneData>>,
) {
    if !scene_copy_is_stale(
        scene.is_changed(),
        existing.map(|e| e.generation),
        scene.generation,
    ) {
        return;
    }
    commands.insert_resource((*scene).clone());
}

/// The active camera, as the render world needs it.
#[derive(Resource, Clone, ExtractResource)]
pub struct ExtractedMarchCamera {
    /// Clip space to the world-space offset from `position`: the camera's
    /// rotation times the inverse projection, with no translation anywhere in
    /// it.
    ///
    /// Not an absolute inverse view-projection. In f32 every entry of that
    /// carries about `|position| * epsilon` of error, and a primary ray built by
    /// unprojecting the near plane (0.1 away under reverse-Z) and subtracting
    /// the eye amplifies it by `1 / near`: about 2 pixels at 2560x1440 a
    /// thousand units from the origin, which moved body edges outside their
    /// screen rectangles. Built from rotation and projection only, the error is
    /// relative and does not grow with distance from the origin.
    pub offset_from_clip: Mat4,
    pub position: Vec3,
}

/// The reachability table split into halves the shader can index.
pub fn gpu_direction_masks() -> Vec<[u32; 2]> {
    build_direction_masks()
        .iter()
        .map(|m| [*m as u32, (*m >> 32) as u32])
        .collect()
}

/// Rebuilds the GPU-side representation whenever the scene is replaced.
///
/// Gated on `generation`, not on change-detection: `VoxelScene` is mutated by
/// every brush edit, which would otherwise flag it as changed and trigger a
/// full CPU rebuild of the whole volume on every click, defeating the dirty-
/// range upload path entirely. An edit deliberately leaves `generation`
/// alone; only a full scene load bumps it. Do not restore the `is_changed`
/// guard here -- that is the bug this comment exists to prevent.
///
/// A generation match is not enough on its own, though: `prepare_march_buffers`
/// also rebuilds -- from this resource -- whenever an edit's high-water marks
/// have outgrown the render buffers' capacity. If this resource stayed a
/// frozen load-time snapshot, that rebuild would re-upload stale data forever
/// (the new capacity is computed from the same stale snapshot, so the overflow
/// never clears). So a live tree that no longer fits the snapshot's
/// `world_region` forces a refresh too -- the same region and the same
/// `WorldRegion::fits` the render world checks, so the two cannot drift.
///
/// The world region, not the snapshot's whole length: with bodies, `nodes`
/// also holds the region's headroom and every body, and a capacity measured
/// from that would let the world grow into the first body without a rebuild.
pub fn build_gpu_scene(
    mut commands: Commands,
    scene: Option<Res<VoxelScene>>,
    existing: Option<Res<GpuSceneData>>,
) {
    // No scene inserted yet is a normal state, not an error.
    let Some(scene) = scene else {
        return;
    };
    let (node_high_water, voxel_word_high_water) = world_high_water(&scene.tree);
    if let Some(existing) = &existing
        && existing.generation == scene.generation
        && existing.world_region.fits(node_high_water, voxel_word_high_water)
    {
        return;
    }
    // Bodies are geometry here. A body that only moved does not reach this
    // line -- the generation gate above lets it through untouched -- and its
    // new transform goes out in `SceneUpdate::bodies` instead.
    let packed = pack_bodies(&scene.tree, &scene.bodies);
    // The scene's own field, not a fresh build. `VoxelScene` builds it once at
    // load and `apply_brush` keeps it current, so rebuilding here would both
    // discard that work and stall: 930 ms at extent 4096, on a path that also
    // runs when an edit outgrows the buffers mid-session.
    commands.insert_resource(GpuSceneData {
        nodes: packed.nodes,
        voxels: packed.voxels,
        bodies: packed.bodies,
        body_local_bounds: packed.body_local_bounds,
        palette: scene.materials.to_gpu(),
        direction_masks: gpu_direction_masks(),
        depth: scene.tree.depth(),
        extent: scene.tree.extent(),
        field_edge: scene.field.edge(),
        distance_field: pack_field(&scene.field),
        generation: scene.generation,
        world_region: packed.world_region,
    });
}

/// The field packed four cells to a word, matching `pack_voxels`.
pub fn pack_field(field: &bevox_core::distance_field::DistanceField) -> Vec<u32> {
    bevox_core::gpu::pack_voxels(field.cells())
}

/// Clip space to the world-space offset from the eye: the camera's rotation
/// times the inverse projection, with no translation anywhere in it. See
/// `ExtractedMarchCamera::offset_from_clip` for why never the absolute
/// view-projection.
pub fn offset_from_clip(rotation: Quat, clip_from_view: Mat4) -> Mat4 {
    Mat4::from_quat(rotation) * clip_from_view.inverse()
}

/// Reads the active 3D camera in the main world so it can be extracted.
pub fn track_march_camera(
    mut commands: Commands,
    camera: Query<(&GlobalTransform, &Projection), With<Camera3d>>,
) {
    let Ok((transform, projection)) = camera.single() else {
        return;
    };
    commands.insert_resource(ExtractedMarchCamera {
        offset_from_clip: offset_from_clip(transform.rotation(), projection.get_clip_from_view()),
        position: transform.translation(),
    });
}

/// Creates the storage texture the shader writes, and a sprite to display it.
pub fn create_march_target(
    mut commands: Commands,
    mut images: ResMut<Assets<Image>>,
    windows: Query<&Window>,
) {
    let (width, height) = match windows.single() {
        Ok(window) => (window.physical_width().max(1), window.physical_height().max(1)),
        Err(_) => (1280, 720),
    };

    let mut image = Image::new_fill(
        Extent3d { width, height, depth_or_array_layers: 1 },
        TextureDimension::D2,
        &[0, 0, 0, 255],
        TextureFormat::Rgba8Unorm,
        // MAIN_WORLD is kept deliberately: dropping it discards the asset from
        // Assets<Image>, and the sprite then has no dimensions to size itself
        // from, so it draws nothing at all.
        RenderAssetUsages::MAIN_WORLD | RenderAssetUsages::RENDER_WORLD,
    );
    // STORAGE_BINDING is the one that matters: without it the bind group is
    // rejected at creation, and the symptom is a black window with a validation
    // error rather than an obvious failure.
    image.texture_descriptor.usage =
        TextureUsages::TEXTURE_BINDING | TextureUsages::COPY_DST | TextureUsages::STORAGE_BINDING;

    let handle = images.add(image);
    commands.spawn(Sprite {
        image: handle.clone(),
        // Stated explicitly rather than inferred, so the sprite covers the
        // window regardless of when the image's size becomes known.
        custom_size: Some(Vec2::new(width as f32, height as f32)),
        ..default()
    });
    commands.insert_resource(MarchTarget { image: handle, width, height });
}

/// Builds the shader uniform from a camera and the volume being drawn.
///
/// `offset_from_clip` is camera-relative, as `ExtractedMarchCamera` describes:
/// the shader multiplies a clip-space point by it to get a ray direction.
///
/// Takes the field rather than recomputing its edge count from `tree.extent()`:
/// that formula already lives in `DistanceField::build`, and repeating it here
/// is a second spelling of the same number that could silently drift from it.
pub fn march_uniform(
    offset_from_clip: Mat4,
    camera_position: Vec3,
    tree: &Contree,
    field: &DistanceField,
    flags: u32,
) -> MarchUniform {
    MarchUniform {
        offset_from_clip: offset_from_clip.to_cols_array_2d(),
        camera_position: camera_position.extend(0.0).to_array(),
        sun_direction: SUN_DIRECTION.normalize().extend(0.0).to_array(),
        volume_params: [tree.depth(), tree.extent(), flags, 0],
        // The cell size travels in the uniform rather than as a matching
        // shader-side constant, which is exactly the duplication that let
        // `march.wgsl`'s copy silently disagree with this one.
        field_params: [field.edge(), bevox_core::distance_field::CELL_VOXELS, 0, 0],
    }
}

/// A contiguous run of nodes to write, addressed by arena slot.
#[derive(Clone, Debug)]
pub struct NodeWrite {
    /// First arena slot. The buffer index is this plus one: index 0 is the root.
    pub start: u32,
    pub nodes: Vec<GpuNode>,
}

/// A contiguous run of packed voxel words to write.
#[derive(Clone, Debug)]
pub struct VoxelWrite {
    pub start_word: u32,
    pub words: Vec<u32>,
}

/// A contiguous run of packed distance-field words to write.
#[derive(Clone, Debug)]
pub struct FieldWrite {
    pub start_word: u32,
    pub words: Vec<u32>,
}

/// One frame's worth of scene changes.
///
/// Cloned into the render world every frame like the rest of the extracted
/// state, which is only affordable because it is empty on frames with no edit.
#[derive(Resource, Clone, Debug, ExtractResource)]
pub struct SceneUpdate {
    /// Always present. The root lives outside the arena, so no dirty range can
    /// name it, and nearly every edit replaces it.
    pub root: GpuNode,
    pub nodes: Vec<NodeWrite>,
    pub voxels: Vec<VoxelWrite>,
    /// The field cells the brush lowered this frame. Empty on every frame with
    /// no paint, including one that only erased.
    pub field: Vec<FieldWrite>,
    /// Arena slots in use. The node buffer must hold this many plus the root.
    pub node_high_water: u32,
    /// Packed voxel words in use.
    pub voxel_word_high_water: u32,
    /// The whole body table, every frame, placed where each body is now.
    ///
    /// This is how a body moves without a rebuild: its geometry stays where the
    /// last rebuild packed it, and only this table is rewritten.
    pub bodies: Vec<GpuBody>,
}

impl Default for SceneUpdate {
    fn default() -> Self {
        Self {
            root: GpuNode::default(),
            nodes: Vec::new(),
            voxels: Vec::new(),
            field: Vec::new(),
            node_high_water: 0,
            voxel_word_high_water: 0,
            bodies: Vec::new(),
        }
    }
}

/// Applies one brush stroke, updating the distance field only when it must.
///
/// This is where the paint/erase asymmetry lives, and it lives here rather
/// than in the app so it can be tested. Painting adds geometry and lowers true
/// distances, so a field left stale would over-estimate and rays would skip
/// the new geometry. Erasing only raises true distances, leaving the field
/// under-estimating, which costs a little speed and nothing else.
pub fn apply_brush(scene: &mut VoxelScene, centre: Vec3, radius: f32, material: MaterialId) {
    scene.tree.apply_sphere(centre, radius, material);
    if !material.is_empty() {
        scene.field_dirty = Some(scene.field.lower_around(centre, radius));
    }
}

/// The world's high-water marks: arena node slots in use, and packed voxel
/// words in use.
///
/// The one spelling of both. `build_gpu_scene` tests them against the snapshot's
/// region to decide on a rebuild, and `stage_scene_update` hands them to the
/// render world to test against its buffers' region. Were the two computed
/// separately, one side could rebuild while the other wrote into a body.
pub fn world_high_water(tree: &Contree) -> (u32, u32) {
    let arena = tree.arena();
    (arena.nodes().len() as u32, arena.voxels().len().div_ceil(4) as u32)
}

/// Drains the arena's dirty ranges into a delta the render world can write.
///
/// Reads the current arena rather than remembering old values, which is what
/// makes the voxel path correct: a dirty byte range is rounded outward to whole
/// words, and the untouched bytes sharing those words are re-read as they are.
pub fn stage_scene_update(scene: &mut VoxelScene) -> SceneUpdate {
    let (node_high_water, voxel_word_high_water) = world_high_water(&scene.tree);
    let node_ranges = scene.tree.arena().dirty_nodes();
    let voxel_ranges = scene.tree.arena().dirty_voxels();

    let arena = scene.tree.arena();
    let nodes = node_ranges
        .iter()
        .map(|r| NodeWrite {
            start: r.start,
            nodes: arena.nodes()[r.start as usize..r.end as usize]
                .iter()
                .map(|n| GpuNode::from(*n))
                .collect(),
        })
        .collect();

    let bytes = arena.voxels();
    let voxels = voxel_ranges
        .iter()
        .map(|r| {
            let first = r.start / 4;
            let last = r.end.div_ceil(4);
            let lo = (first * 4) as usize;
            // The final word may run past the arena; pack_voxels zero-pads it,
            // and those bytes belong to no voxel.
            let hi = ((last * 4) as usize).min(bytes.len());
            VoxelWrite {
                start_word: first,
                words: bevox_core::gpu::pack_voxels(&bytes[lo..hi]),
            }
        })
        .collect();

    // Rounded outward to whole words exactly like the voxel path, and read
    // from the live field rather than remembered values so a cell sharing a
    // boundary word with the untouched range still comes out right. `None`
    // after an erase, which is what makes that path stage nothing at all.
    let cells = scene.field.cells();
    let field = scene
        .field_dirty
        .take()
        .map(|r| {
            let first = r.start / 4;
            let last = r.end.div_ceil(4);
            let lo = (first * 4) as usize;
            let hi = ((last * 4) as usize).min(cells.len());
            FieldWrite { start_word: first, words: bevox_core::gpu::pack_voxels(&cells[lo..hi]) }
        })
        .into_iter()
        .collect();

    let update = SceneUpdate {
        root: GpuNode::from(scene.tree.root()),
        nodes,
        voxels,
        field,
        node_high_water,
        voxel_word_high_water,
        bodies: Vec::new(),
    };

    scene.tree.arena_mut().clear_dirty();
    update
}

/// Drains the scene's dirty ranges once per frame, and re-places every body.
///
/// Runs every frame, not only on edits: the resource it writes must be empty on
/// a quiet frame, or the render world would rewrite the last edit forever.
///
/// The body table takes its layout from `gpu` -- where the last rebuild put
/// each body's geometry -- and its transforms from the live scene. That is why
/// a body that only moves needs no generation bump. Bodies are matched by
/// index, so one added or removed without a bump is not in the table until the
/// rebuild that bump asks for.
pub fn stage_scene_update_system(
    mut commands: Commands,
    scene: Option<ResMut<VoxelScene>>,
    gpu: Res<GpuSceneData>,
) {
    let Some(mut scene) = scene else {
        return;
    };
    let mut update = stage_scene_update(&mut scene);
    update.bodies = gpu.bodies.iter().zip(&scene.bodies).map(|(g, b)| g.placed(b)).collect();
    commands.insert_resource(update);
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_mask_halves_reassemble_into_the_originals() {
        let split = gpu_direction_masks();
        let source = build_direction_masks();
        assert_eq!(split.len(), source.len());
        for (got, want) in split.iter().zip(source.iter()) {
            assert_eq!(u64::from(got[0]) | (u64::from(got[1]) << 32), *want);
        }
    }

    #[test]
    fn a_render_world_without_a_copy_always_extracts() {
        assert!(scene_copy_is_stale(false, None, 7));
    }

    #[test]
    fn an_unchanged_scene_is_not_re_extracted() {
        // The whole point: no clone on a quiet frame.
        assert!(!scene_copy_is_stale(false, Some(7), 7));
    }

    #[test]
    fn a_rebuilt_scene_is_extracted_even_at_the_same_generation() {
        // An edit that outgrows the buffers rebuilds without bumping the
        // generation, so the change flag is the only signal that it happened.
        assert!(scene_copy_is_stale(true, Some(7), 7));
    }

    #[test]
    fn a_generation_mismatch_is_extracted_even_without_the_change_flag() {
        assert!(scene_copy_is_stale(false, Some(7), 8));
    }

    #[test]
    fn the_field_packs_four_cells_to_a_word() {
        let field = bevox_core::distance_field::DistanceField::build(&Contree::empty(3));
        let packed = pack_field(&field);
        assert_eq!(packed.len(), field.cells().len().div_ceil(4));
        // Little-endian within the word, matching pack_voxels.
        let first = packed[0];
        for i in 0..4 {
            assert_eq!(
                ((first >> (i * 8)) & 0xFF) as u8,
                field.cells()[i as usize],
                "cell {i} is not in byte {i} of word 0"
            );
        }
    }

    #[test]
    fn the_uniform_is_the_size_the_shader_expects() {
        // mat4x4 (64) + vec4 (16) + vec4 sun (16) + uvec4 (16) + uvec4 field (16)
        assert_eq!(size_of::<MarchUniform>(), 128);
        assert_eq!(align_of::<MarchUniform>(), 4);
    }

    #[test]
    fn volume_params_carry_depth_and_extent() {
        let tree = Contree::empty(3);
        let field = DistanceField::build(&tree);
        let u = march_uniform(Mat4::IDENTITY, Vec3::ZERO, &tree, &field, march_flags::NONE);
        assert_eq!(u.volume_params[0], 3);
        assert_eq!(u.volume_params[1], 64);
        // The cell size travels with the edge count rather than a shader-side
        // constant, so this is the one place that number is spelled out.
        assert_eq!(u.field_params[0], field.edge());
        assert_eq!(u.field_params[1], bevox_core::distance_field::CELL_VOXELS);
    }

    #[test]
    fn the_camera_position_survives_into_the_uniform() {
        let tree = Contree::empty(2);
        let field = DistanceField::build(&tree);
        let u =
            march_uniform(Mat4::IDENTITY, Vec3::new(1.0, 2.0, 3.0), &tree, &field, march_flags::NONE);
        assert_eq!(u.camera_position[0], 1.0);
        assert_eq!(u.camera_position[1], 2.0);
        assert_eq!(u.camera_position[2], 3.0);
    }

    #[test]
    fn flags_land_where_the_shader_reads_them() {
        let tree = Contree::empty(2);
        let field = DistanceField::build(&tree);
        let u = march_uniform(
            Mat4::IDENTITY,
            Vec3::ZERO,
            &tree,
            &field,
            march_flags::DDA | march_flags::BEAM,
        );
        assert_eq!(u.volume_params[2], 0b101);
    }

    #[test]
    fn the_matrix_is_stored_column_major_as_wgsl_expects() {
        let m = Mat4::from_translation(Vec3::new(5.0, 6.0, 7.0));
        let tree = Contree::empty(2);
        let field = DistanceField::build(&tree);
        let u = march_uniform(m, Vec3::ZERO, &tree, &field, march_flags::NONE);
        // glam is column-major, and to_cols_array_2d yields columns.
        assert_eq!(u.offset_from_clip[3][0], 5.0);
        assert_eq!(u.offset_from_clip[3][1], 6.0);
        assert_eq!(u.offset_from_clip[3][2], 7.0);
    }

    #[test]
    fn offset_from_clip_matches_the_inverse_view_projection_at_the_eye() {
        // Yaw and pitch together, so no axis is left at its identity value.
        let rotation = Quat::from_euler(EulerRot::YXZ, 0.6, 0.3, 0.0);
        let projections = [
            Mat4::perspective_rh(0.9, 16.0 / 9.0, 0.1, 500.0),
            Mat4::perspective_infinite_reverse_rh(0.9, 16.0 / 9.0, 0.1),
        ];
        for clip_from_view in projections {
            let got = offset_from_clip(rotation, clip_from_view);
            // The textbook route: build the view matrix for a camera at the
            // origin facing `rotation`, compose with the projection, and
            // invert the whole thing back to a clip-to-world-offset matrix.
            let forward = rotation * -Vec3::Z;
            let up = rotation * Vec3::Y;
            let want = (clip_from_view * Mat4::look_to_rh(Vec3::ZERO, forward, up)).inverse();
            let (got, want) = (got.to_cols_array(), want.to_cols_array());
            for i in 0..16 {
                assert!(
                    (got[i] - want[i]).abs() < 1e-5,
                    "entry {i}: got {}, want {}",
                    got[i],
                    want[i]
                );
            }
        }
    }

    use bevox_core::contree::Contree;
    use bevox_core::material::MaterialId;
    use glam::{EulerRot, Mat4, Quat, UVec3, Vec3};

    /// A scene with something in it, so an edit has existing nodes to rewrite
    /// rather than only allocating fresh ones.
    fn edit_scene() -> VoxelScene {
        let mut tree = Contree::empty(3);
        tree.apply_sphere(Vec3::new(32.0, 32.0, 32.0), 12.0, MaterialId(1));
        // The initial build is not an edit: clear it so a test sees only what
        // the edit under test touched.
        tree.arena_mut().clear_dirty();
        let field = DistanceField::build(&tree);
        VoxelScene {
            tree,
            materials: MaterialTable::new(),
            generation: 1,
            field,
            field_dirty: None,
            bodies: Vec::new(),
        }
    }

    /// Painting must update the field in the same frame as the voxels, or the
    /// GPU skips empty space that is no longer empty.
    #[test]
    fn a_paint_stages_the_field_cells_it_lowered() {
        let mut scene = edit_scene();
        apply_brush(&mut scene, Vec3::splat(32.0), 6.0, MaterialId(2));

        let update = stage_scene_update(&mut scene);
        assert!(!update.field.is_empty(), "a paint staged no field cells");

        let whole = pack_field(&scene.field);
        for write in &update.field {
            for (i, word) in write.words.iter().enumerate() {
                let w = write.start_word as usize + i;
                assert_eq!(*word, whole[w], "staged field word {w} differs from a full pack");
            }
        }
    }

    /// Erasing needs no field update at all: removing geometry only increases
    /// true distances, so a stale field under-estimates, which costs speed and
    /// never correctness.
    #[test]
    fn erasing_stages_no_field_cells() {
        let mut scene = edit_scene();
        apply_brush(&mut scene, Vec3::splat(32.0), 6.0, MaterialId::EMPTY);
        let update = stage_scene_update(&mut scene);
        assert!(update.field.is_empty(), "an erase staged field cells it did not need to");
    }

    #[test]
    fn staging_an_untouched_scene_produces_no_writes() {
        let mut scene = edit_scene();
        let update = stage_scene_update(&mut scene);
        assert!(update.nodes.is_empty(), "nothing was edited, yet nodes were staged");
        assert!(update.voxels.is_empty(), "nothing was edited, yet voxels were staged");
    }

    #[test]
    fn the_root_is_staged_even_when_it_is_in_no_dirty_range() {
        // The root lives at buffer index 0, outside the arena, so no dirty
        // range can ever name it -- and nearly every edit replaces it.
        let mut scene = edit_scene();
        let before = GpuNode::from(scene.tree.root());
        scene.tree.apply_sphere(Vec3::new(32.0, 32.0, 32.0), 20.0, MaterialId(2));
        let update = stage_scene_update(&mut scene);
        assert_eq!(update.root, GpuNode::from(scene.tree.root()));
        assert_ne!(update.root, before, "this edit should have changed the root");
    }

    #[test]
    fn staged_nodes_carry_the_bytes_the_arena_holds_now() {
        let mut scene = edit_scene();
        scene.tree.apply_sphere(Vec3::new(20.0, 20.0, 20.0), 6.0, MaterialId(3));
        let update = stage_scene_update(&mut scene);
        assert!(!update.nodes.is_empty(), "an edit staged no node writes");
        let arena = scene.tree.arena();
        for write in &update.nodes {
            for (i, staged) in write.nodes.iter().enumerate() {
                let slot = write.start as usize + i;
                assert_eq!(
                    *staged,
                    GpuNode::from(arena.nodes()[slot]),
                    "staged node at arena slot {slot} does not match the arena"
                );
            }
        }
    }

    #[test]
    fn staged_voxel_words_match_a_full_pack_of_the_arena() {
        let mut scene = edit_scene();
        scene.tree.apply_sphere(Vec3::new(30.0, 30.0, 30.0), 4.0, MaterialId(4));
        let update = stage_scene_update(&mut scene);
        let whole = bevox_core::gpu::pack_voxels(scene.tree.arena().voxels());
        assert!(!update.voxels.is_empty(), "an edit staged no voxel writes");
        for write in &update.voxels {
            for (i, word) in write.words.iter().enumerate() {
                let w = write.start_word as usize + i;
                assert_eq!(*word, whole[w], "staged voxel word {w} differs from a full pack");
            }
        }
    }

    #[test]
    fn staging_clears_the_dirty_record_so_the_next_frame_stages_nothing() {
        let mut scene = edit_scene();
        scene.tree.apply_sphere(Vec3::new(32.0, 32.0, 32.0), 8.0, MaterialId(1));
        let first = stage_scene_update(&mut scene);
        assert!(!first.nodes.is_empty());
        let second = stage_scene_update(&mut scene);
        assert!(second.nodes.is_empty(), "the same edit staged twice");
        assert!(second.voxels.is_empty(), "the same edit staged twice");
    }

    #[test]
    fn the_high_water_marks_cover_the_whole_arena() {
        // The buffers must be large enough for every slot, not merely for the
        // dirty ones: a slot allocated by an earlier edit is still read.
        let mut scene = edit_scene();
        scene.tree.apply_sphere(Vec3::new(40.0, 40.0, 40.0), 10.0, MaterialId(2));
        let update = stage_scene_update(&mut scene);
        assert_eq!(update.node_high_water, scene.tree.arena().nodes().len() as u32);
        assert_eq!(
            update.voxel_word_high_water,
            scene.tree.arena().voxels().len().div_ceil(4) as u32
        );
    }

    #[test]
    fn an_edit_does_not_rebuild_gpu_scene_data_but_a_replacement_does() {
        // Regression guard for the generation gate in `build_gpu_scene`: it
        // must reject `is_changed()` (which every edit trips) and accept only
        // a generation bump. Comparing the resource's node bytes before and
        // after -- not just its `generation` field, which a buggy rebuild
        // would also set to the unchanged scene generation -- is what makes
        // this fail if that guard regresses.
        let mut app = App::new();
        app.add_plugins(MinimalPlugins)
            .init_resource::<GpuSceneData>()
            .add_systems(Update, build_gpu_scene);

        let mut tree = Contree::empty(3);
        tree.apply_sphere(Vec3::new(32.0, 32.0, 32.0), 10.0, MaterialId(1));
        tree.arena_mut().clear_dirty();
        let field = DistanceField::build(&tree);
        app.insert_resource(VoxelScene {
            tree,
            materials: MaterialTable::new(),
            generation: 1,
            field,
            field_dirty: None,
            bodies: Vec::new(),
        });
        app.update();

        let loaded_nodes = app.world().resource::<GpuSceneData>().nodes.clone();
        assert!(loaded_nodes.len() > 1, "the initial load should have produced real geometry");

        // An edit: mutates the tree (so `is_changed()` would trip) but leaves
        // `generation` alone, as every real brush edit does.
        app.world_mut()
            .resource_mut::<VoxelScene>()
            .tree
            .apply_sphere(Vec3::new(5.0, 5.0, 5.0), 4.0, MaterialId(2));
        app.update();
        assert_eq!(
            app.world().resource::<GpuSceneData>().nodes,
            loaded_nodes,
            "an edit left generation unchanged, so GpuSceneData must not have been rebuilt"
        );

        // A replacement: bumping generation must rebuild, reflecting the tree
        // as it stands now (already grown by the edit above).
        app.world_mut().resource_mut::<VoxelScene>().generation = 2;
        app.update();
        assert_ne!(
            app.world().resource::<GpuSceneData>().nodes,
            loaded_nodes,
            "bumping generation must trigger a rebuild"
        );
    }

    #[test]
    fn an_edit_that_outgrows_the_buffers_forces_a_rebuild_from_the_live_tree() {
        // Regression guard for the capacity-overflow path: `prepare_march_buffers`
        // falls back to rebuilding from `GpuSceneData` once an edit's high-water
        // mark outgrows the render buffers. If `build_gpu_scene` never refreshes
        // that snapshot for anything but a generation bump, that fallback reads
        // a permanently stale tree.
        let mut app = App::new();
        app.add_plugins(MinimalPlugins)
            .init_resource::<GpuSceneData>()
            .add_systems(Update, build_gpu_scene);

        let mut tree = Contree::empty(3);
        tree.apply_sphere(Vec3::new(32.0, 32.0, 32.0), 10.0, MaterialId(1));
        tree.arena_mut().clear_dirty();
        let field = DistanceField::build(&tree);
        app.insert_resource(VoxelScene {
            tree,
            materials: MaterialTable::new(),
            generation: 1,
            field,
            field_dirty: None,
            bodies: Vec::new(),
        });
        app.update();

        let loaded_nodes = app.world().resource::<GpuSceneData>().nodes.clone();
        let node_capacity = buffer_capacity_for(loaded_nodes.len() as u32);

        // Paint one isolated voxel per leaf block (extent 4, so a 64-extent tree
        // has 16^3 of them) across distinct blocks until the live arena has
        // outgrown the capacity the loaded snapshot implies. Each block starts
        // untouched, so a single covered voxel among its 64 is never uniform
        // and always forces a fresh node allocation up the chain to the root.
        {
            let mut scene = app.world_mut().resource_mut::<VoxelScene>();
            'grid: for bx in 0..16u32 {
                for by in 0..16u32 {
                    for bz in 0..16u32 {
                        if scene.tree.arena().nodes().len() as u32 + 1 >= node_capacity {
                            break 'grid;
                        }
                        let centre = Vec3::new(
                            (bx * 4) as f32 + 0.5,
                            (by * 4) as f32 + 0.5,
                            (bz * 4) as f32 + 0.5,
                        );
                        scene.tree.apply_sphere(centre, 0.6, MaterialId(2));
                    }
                }
            }
            assert!(
                scene.tree.arena().nodes().len() as u32 + 1 >= node_capacity,
                "test setup failed to outgrow the buffers -- widen the grid"
            );
        }
        app.update();

        let rebuilt_nodes = app.world().resource::<GpuSceneData>().nodes.clone();
        assert_ne!(
            rebuilt_nodes, loaded_nodes,
            "outgrowing the buffers must rebuild GpuSceneData, not keep serving the load-time snapshot"
        );
        let expected = GpuVolume::from_contree(&app.world().resource::<VoxelScene>().tree).buffer_nodes();
        assert_eq!(
            rebuilt_nodes, expected,
            "the forced rebuild must reflect the tree as it stands now, not some other snapshot"
        );
    }

    #[test]
    fn a_gpu_body_is_the_size_the_shader_expects() {
        // Two mat4x4 (64 each), four u32 (16), then the bounding sphere's vec4,
        // which WGSL aligns to 16 and which lands at 144 without padding.
        assert_eq!(size_of::<GpuBody>(), 160);
        assert_eq!(std::mem::offset_of!(GpuBody, bound), 144);
        assert_eq!(align_of::<GpuBody>(), 4);
    }

    #[test]
    fn packing_a_body_shifts_its_arena_past_the_static_world() {
        let world = Contree::empty(3);
        let body = bevox_core::body::Body::new(
            Contree::empty(2),
            Vec3::new(1.0, 2.0, 3.0),
            Quat::IDENTITY,
        );
        let packed = pack_bodies(&world, std::slice::from_ref(&body));

        let world_volume = GpuVolume::from_contree(&world);
        let region = WorldRegion::around(
            world_volume.buffer_nodes().len() as u32,
            world_volume.voxels.len() as u32,
        );
        assert_eq!(packed.world_region, region);
        assert_eq!(
            (packed.bodies[0].node_base, packed.bodies[0].voxel_base),
            (region.nodes, region.voxel_words),
            "the body must start past the world's region, not at its current end"
        );
        assert_eq!(
            packed.nodes.len() as u32,
            region.nodes + GpuVolume::from_contree(&body.volume).buffer_nodes().len() as u32,
            "the packed buffer must hold the world's region and then the body"
        );
    }

    #[test]
    fn a_packed_body_carries_the_inverse_of_its_placement() {
        // The shader transforms rays world-to-local, so that is what it needs.
        let body = bevox_core::body::Body::new(
            Contree::empty(2),
            Vec3::new(4.0, 0.0, 0.0),
            Quat::IDENTITY,
        );
        let packed = pack_bodies(&Contree::empty(3), std::slice::from_ref(&body));
        let m = Mat4::from_cols_array_2d(&packed.bodies[0].local_from_world);
        let there_and_back = m.transform_point3(Vec3::new(4.0, 0.0, 0.0));
        assert!(
            there_and_back.length() < 1e-4,
            "the body's own position should map to its local origin, got {there_and_back:?}"
        );
    }

    #[test]
    fn a_scene_with_no_bodies_packs_exactly_the_static_world() {
        // The zero-body case must be byte-identical, because the whole feature
        // is required to leave a body-free scene untouched.
        let world = Contree::empty(3);
        let packed = pack_bodies(&world, &[]);
        let plain = GpuVolume::from_contree(&world);
        assert_eq!(packed.nodes, plain.buffer_nodes());
        assert_eq!(packed.voxels, plain.voxels);
        assert!(packed.bodies.is_empty());
    }

    /// A body that moves must reach the table without the geometry being
    /// rebuilt. Rebuilding would also put the new transform in the table, so
    /// the table alone cannot tell the right path from the expensive one: the
    /// change tick on `GpuSceneData` is what says no rebuild happened.
    #[test]
    fn a_moved_body_refreshes_the_table_without_a_geometry_rebuild() {
        let mut app = App::new();
        app.add_plugins(MinimalPlugins)
            .init_resource::<GpuSceneData>()
            .add_systems(
                Update,
                (build_gpu_scene, stage_scene_update_system.after(build_gpu_scene)),
            );

        let tree = Contree::empty(3);
        let field = DistanceField::build(&tree);
        let body = bevox_core::body::Body::new(
            Contree::empty(2),
            Vec3::new(4.0, 5.0, 6.0),
            Quat::IDENTITY,
        );
        app.insert_resource(VoxelScene {
            tree,
            materials: MaterialTable::new(),
            generation: 1,
            field,
            field_dirty: None,
            bodies: vec![body],
        });
        app.update();
        let built = app.world().get_resource_change_ticks::<GpuSceneData>().unwrap().changed;

        // Move it and turn it, and leave the generation alone.
        {
            let mut scene = app.world_mut().resource_mut::<VoxelScene>();
            scene.bodies[0].position = Vec3::new(9.0, 1.0, 2.0);
            scene.bodies[0].orientation = Quat::from_euler(glam::EulerRot::XYZ, 0.4, 0.9, -0.2);
        }
        app.update();

        let gpu = app.world().resource::<GpuSceneData>();
        assert_eq!(gpu.generation, 1);
        assert_eq!(
            app.world().get_resource_change_ticks::<GpuSceneData>().unwrap().changed,
            built,
            "moving a body rebuilt GpuSceneData; only the body table should have changed"
        );

        let scene = app.world().resource::<VoxelScene>();
        let expected = pack_bodies(&scene.tree, &scene.bodies).bodies;
        let table = &app.world().resource::<SceneUpdate>().bodies;
        assert_eq!(
            bytemuck::cast_slice::<GpuBody, u8>(table),
            bytemuck::cast_slice::<GpuBody, u8>(&expected),
            "the body table does not place the body where it is now"
        );
    }

    /// `fits` at its exact edges, each limit failing on its own.
    ///
    /// Nodes are strict. The pack puts the root at index 0 and arena slot `n` at
    /// `n + 1`, so `h` slots reach index `h`, and a region of `nodes` entries
    /// holds `nodes - 1` of them: one more would sit on the first body's root.
    /// Voxel words have no root, so a region of `voxel_words` holds exactly that
    /// many, and one more is the first body's first word.
    #[test]
    fn the_world_region_fits_exactly_what_its_layout_holds() {
        let region = WorldRegion { nodes: 100, voxel_words: 50 };
        assert!(region.fits(99, 50), "both sides at their last legal value must fit");
        assert!(!region.fits(100, 50), "slot 99 would be at index 100, the first body's root");
        assert!(!region.fits(99, 51), "word 50 is the first body's first voxel word");
    }

    /// Two copies of a small cube, off the brick grid so each owns voxel bytes
    /// as well as nodes, and so both halves of the overlap check have something
    /// to find.
    fn two_bodies() -> Vec<Body> {
        let mut cube = Vec::new();
        for z in 5..11 {
            for y in 5..11 {
                for x in 5..11 {
                    cube.push((UVec3::new(x, y, z), MaterialId(3)));
                }
            }
        }
        let volume = Contree::from_voxels(16, &cube);
        assert!(
            !GpuVolume::from_contree(&volume).voxels.is_empty(),
            "the body owns no voxel bytes, so the voxel half of the check is vacuous"
        );
        vec![
            Body::new(volume.clone(), Vec3::ZERO, Quat::IDENTITY),
            Body::new(volume, Vec3::splat(20.0), Quat::IDENTITY),
        ]
    }

    /// Where painting stopped fitting.
    #[derive(Debug)]
    struct Outgrown {
        region: WorldRegion,
        node_high_water: u32,
        voxel_word_high_water: u32,
        /// Strokes that fitted before the one that did not.
        fitted: usize,
    }

    /// Paints `strokes` into `scene`, with two bodies added, until the world
    /// outgrows its region -- through `apply_brush` and the real systems, as the
    /// app does.
    ///
    /// Every frame that still fits must stage no write inside any body's nodes
    /// or voxels, and must not rebuild. The frame that no longer fits must
    /// rebuild -- judged by `GpuSceneData`'s change tick, which is what
    /// `build_gpu_scene`'s trigger drives -- with the bodies moved past a region
    /// the grown world fits. The render world's half, `can_reuse_buffers`,
    /// checks the same `fits` against its buffers' region; its own tests in
    /// `pipeline.rs` pin that, since no render device is available here.
    fn paint_until_the_world_outgrows_its_region(
        mut scene: VoxelScene,
        strokes: impl IntoIterator<Item = (Vec3, f32, MaterialId)>,
    ) -> Outgrown {
        let mut app = App::new();
        app.add_plugins(MinimalPlugins)
            .init_resource::<GpuSceneData>()
            .add_systems(
                Update,
                (build_gpu_scene, stage_scene_update_system.after(build_gpu_scene)),
            );

        scene.bodies = two_bodies();
        let extents: Vec<(u32, u32)> = scene
            .bodies
            .iter()
            .map(|b| {
                let v = GpuVolume::from_contree(&b.volume);
                (v.buffer_nodes().len() as u32, v.voxels.len() as u32)
            })
            .collect();
        let first = world_high_water(&scene.tree);
        app.insert_resource(scene);
        app.update();

        for (fitted, (centre, radius, material)) in strokes.into_iter().enumerate() {
            let (region, bodies) = {
                let gpu = app.world().resource::<GpuSceneData>();
                (gpu.world_region, gpu.bodies.clone())
            };
            let tick = app.world().get_resource_change_ticks::<GpuSceneData>().unwrap().changed;

            apply_brush(&mut app.world_mut().resource_mut::<VoxelScene>(), centre, radius, material);
            app.update();

            let update = app.world().resource::<SceneUpdate>();
            let (nodes, words) = (update.node_high_water, update.voxel_word_high_water);
            let rebuild_tick =
                app.world().get_resource_change_ticks::<GpuSceneData>().unwrap().changed;

            if !region.fits(nodes, words) {
                assert_ne!(
                    rebuild_tick, tick,
                    "the world outgrew its region at {centre:?}, but GpuSceneData was not rebuilt"
                );
                let gpu = app.world().resource::<GpuSceneData>();
                assert!(
                    gpu.world_region.fits(nodes, words),
                    "the rebuild did not make room for the world that forced it"
                );
                for body in &gpu.bodies {
                    assert!(
                        body.node_base >= gpu.world_region.nodes
                            && body.voxel_base >= gpu.world_region.voxel_words,
                        "the rebuild packed a body inside the new world region"
                    );
                }
                assert!(
                    nodes > first.0 || words > first.1,
                    "the world never grew, so nothing here was tested"
                );
                return Outgrown { region, node_high_water: nodes, voxel_word_high_water: words, fitted };
            }

            assert_eq!(
                rebuild_tick, tick,
                "the world still fits its region at {centre:?}, yet GpuSceneData was rebuilt"
            );
            let overlaps = |a: &Range<u32>, b: &Range<u32>| a.start < b.end && b.start < a.end;
            let mut offending = Vec::new();
            for (i, (body, (body_nodes, body_words))) in bodies.iter().zip(&extents).enumerate() {
                let node_range = body.node_base..body.node_base + body_nodes;
                let word_range = body.voxel_base..body.voxel_base + body_words;
                for write in &update.nodes {
                    // Arena slot n is buffer index n + 1.
                    let at = write.start + 1..write.start + 1 + write.nodes.len() as u32;
                    if overlaps(&at, &node_range) {
                        offending.push(format!("node write {at:?} into body {i} nodes {node_range:?}"));
                    }
                }
                for write in &update.voxels {
                    let at = write.start_word..write.start_word + write.words.len() as u32;
                    if overlaps(&at, &word_range) {
                        offending.push(format!("voxel write {at:?} into body {i} words {word_range:?}"));
                    }
                }
            }
            assert!(
                offending.is_empty(),
                "stroke {fitted} at {centre:?} wrote into a body: {}",
                offending.join("; ")
            );
        }
        panic!("the strokes ran out before the world outgrew its region -- paint more");
    }

    /// Painting grows the world into its headroom without touching a body, and
    /// outgrowing that headroom rebuilds. One isolated voxel per leaf block of
    /// the sphere scene forces fresh nodes up to the root each time, so this is
    /// the case where the node side runs out first.
    #[test]
    fn painting_the_world_never_writes_into_a_body() {
        let strokes = (0..16u32).flat_map(|bx| {
            (0..16u32).flat_map(move |by| {
                (0..16u32).map(move |bz| {
                    let centre = UVec3::new(bx, by, bz).as_vec3() * 4.0 + Vec3::splat(0.5);
                    (centre, 0.6, MaterialId(2))
                })
            })
        });
        let out = paint_until_the_world_outgrows_its_region(edit_scene(), strokes);
        assert!(out.fitted > 0, "the first stroke outgrew the region, so no write was checked: {out:?}");
        assert!(
            out.node_high_water >= out.region.nodes
                && out.voxel_word_high_water <= out.region.voxel_words,
            "expected the node side alone to run out: {out:?}"
        );
    }

    /// The same, where the voxel side runs out first.
    ///
    /// Carving one voxel out of a uniform brick of a solid slab turns zero voxel
    /// bytes into 63 while the node count holds: the parent's child array is
    /// freed and re-allocated at the same size, so the free list hands the same
    /// block back. Only the first carve in each 16-voxel region adds nodes.
    #[test]
    fn carving_solid_bricks_never_writes_into_a_body() {
        let mut dense = bevox_core::dense::DenseVolume::new(64).unwrap();
        for z in 0..64 {
            for y in 0..32 {
                for x in 0..64 {
                    dense.set(UVec3::new(x, y, z), MaterialId(1));
                }
            }
        }
        let mut tree = dense.into_contree();
        tree.arena_mut().clear_dirty();
        let field = DistanceField::build(&tree);
        let scene = VoxelScene {
            tree,
            materials: MaterialTable::new(),
            generation: 1,
            field,
            field_dirty: None,
            bodies: Vec::new(),
        };
        let strokes = (0..16u32).flat_map(|bz| {
            (0..8u32).flat_map(move |by| {
                (0..16u32).map(move |bx| {
                    let centre = UVec3::new(bx, by, bz).as_vec3() * 4.0 + Vec3::splat(0.5);
                    (centre, 0.6, MaterialId::EMPTY)
                })
            })
        });
        let out = paint_until_the_world_outgrows_its_region(scene, strokes);
        assert!(out.fitted > 0, "the first stroke outgrew the region, so no write was checked: {out:?}");
        assert!(
            out.voxel_word_high_water > out.region.voxel_words
                && out.node_high_water < out.region.nodes,
            "expected the voxel side alone to run out: {out:?}"
        );
    }
}
