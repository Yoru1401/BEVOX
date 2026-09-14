//! Moving voxel data and camera parameters into the render world.

use crate::pipeline::buffer_capacity_for;
use bevy::asset::RenderAssetUsages;
use bevy::prelude::*;
use bevy::render::Extract;
use bevy::render::extract_resource::ExtractResource;
use bevy::render::render_resource::{Extent3d, TextureDimension, TextureFormat, TextureUsages};
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
}

/// Camera and volume parameters, as the shader sees them.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, Pod, Zeroable)]
pub struct MarchUniform {
    pub world_from_clip: [[f32; 4]; 4],
    pub camera_position: [f32; 4],
    /// Normalised direction *toward* the sun.
    pub sun_direction: [f32; 4],
    /// `[depth, extent, march_flags, 0]`.
    pub volume_params: [u32; 4],
    /// `[field_edge, 0, 0, 0]`.
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
    /// What the app runs. Each optimisation joins this only once it has measured
    /// faster while staying bit-identical.
    ///
    /// All three earned it. A/B/A at 1280x720, extent 1024, close to geometry:
    /// scan 44.5 ms, mask 39.1, beam 31.2, DDA 22.3, all three 16.1 -- a 63.8%
    /// gain against 0.2 ms of drift.
    pub const DEFAULT: u32 = DDA | MASK_FILTER | BEAM;
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

/// The scene as the render world sees it.
///
/// Extracted by `extract_gpu_scene` rather than `ExtractResourcePlugin`, which
/// would clone the whole thing every frame. The render world keeps its copy
/// across frames and only needs a fresh one when the scene is actually rebuilt.
#[derive(Resource, Clone)]
pub struct GpuSceneData {
    pub nodes: Vec<GpuNode>,
    pub voxels: Vec<u32>,
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
    pub world_from_clip: Mat4,
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
/// never clears). So a live tree that no longer fits the capacity implied by
/// the existing snapshot forces a refresh too, using the same
/// `buffer_capacity_for` the render world uses so the two worlds cannot drift.
pub fn build_gpu_scene(
    mut commands: Commands,
    scene: Option<Res<VoxelScene>>,
    existing: Option<Res<GpuSceneData>>,
) {
    // No scene inserted yet is a normal state, not an error.
    let Some(scene) = scene else {
        return;
    };
    if let Some(existing) = &existing {
        // +1: the root, which lives at buffer index 0, outside the arena.
        let nodes_needed = scene.tree.arena().nodes().len() as u32 + 1;
        let node_capacity = buffer_capacity_for(existing.nodes.len() as u32);
        let voxel_words_needed = scene.tree.arena().voxels().len().div_ceil(4) as u32;
        let voxel_word_capacity = buffer_capacity_for(existing.voxels.len() as u32);
        if existing.generation == scene.generation
            && nodes_needed <= node_capacity
            && voxel_words_needed <= voxel_word_capacity
        {
            return;
        }
    }
    let volume = GpuVolume::from_contree(&scene.tree);
    let field = bevox_core::distance_field::DistanceField::build(&scene.tree);
    commands.insert_resource(GpuSceneData {
        nodes: volume.buffer_nodes(),
        voxels: volume.voxels,
        palette: scene.materials.to_gpu(),
        direction_masks: gpu_direction_masks(),
        depth: scene.tree.depth(),
        extent: scene.tree.extent(),
        field_edge: field.edge(),
        distance_field: pack_field(&field),
        generation: scene.generation,
    });
}

/// The field packed four cells to a word, matching `pack_voxels`.
pub fn pack_field(field: &bevox_core::distance_field::DistanceField) -> Vec<u32> {
    bevox_core::gpu::pack_voxels(field.cells())
}

/// Reads the active 3D camera in the main world so it can be extracted.
pub fn track_march_camera(
    mut commands: Commands,
    camera: Query<(&GlobalTransform, &Projection), With<Camera3d>>,
) {
    let Ok((transform, projection)) = camera.single() else {
        return;
    };
    let view = transform.to_matrix().inverse();
    let clip_from_world = projection.get_clip_from_view() * view;
    commands.insert_resource(ExtractedMarchCamera {
        world_from_clip: clip_from_world.inverse(),
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
/// `world_from_clip` is the inverse view-projection: the shader multiplies a
/// clip-space point by it to get a world-space ray target.
pub fn march_uniform(
    world_from_clip: Mat4,
    camera_position: Vec3,
    tree: &Contree,
    flags: u32,
) -> MarchUniform {
    let field_edge = (tree.extent() / bevox_core::distance_field::CELL_VOXELS).max(1);
    MarchUniform {
        world_from_clip: world_from_clip.to_cols_array_2d(),
        camera_position: camera_position.extend(0.0).to_array(),
        sun_direction: SUN_DIRECTION.normalize().extend(0.0).to_array(),
        volume_params: [tree.depth(), tree.extent(), flags, 0],
        field_params: [field_edge, 0, 0, 0],
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

/// Drains the arena's dirty ranges into a delta the render world can write.
///
/// Reads the current arena rather than remembering old values, which is what
/// makes the voxel path correct: a dirty byte range is rounded outward to whole
/// words, and the untouched bytes sharing those words are re-read as they are.
pub fn stage_scene_update(scene: &mut VoxelScene) -> SceneUpdate {
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
        node_high_water: arena.nodes().len() as u32,
        voxel_word_high_water: arena.voxels().len().div_ceil(4) as u32,
    };

    scene.tree.arena_mut().clear_dirty();
    update
}

/// Drains the scene's dirty ranges once per frame.
///
/// Runs every frame, not only on edits: the resource it writes must be empty on
/// a quiet frame, or the render world would rewrite the last edit forever.
pub fn stage_scene_update_system(
    mut commands: Commands,
    scene: Option<ResMut<VoxelScene>>,
) {
    let Some(mut scene) = scene else {
        return;
    };
    commands.insert_resource(stage_scene_update(&mut scene));
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
        let u = march_uniform(Mat4::IDENTITY, Vec3::ZERO, &tree, march_flags::NONE);
        assert_eq!(u.volume_params[0], 3);
        assert_eq!(u.volume_params[1], 64);
    }

    #[test]
    fn the_camera_position_survives_into_the_uniform() {
        let tree = Contree::empty(2);
        let u = march_uniform(Mat4::IDENTITY, Vec3::new(1.0, 2.0, 3.0), &tree, march_flags::NONE);
        assert_eq!(u.camera_position[0], 1.0);
        assert_eq!(u.camera_position[1], 2.0);
        assert_eq!(u.camera_position[2], 3.0);
    }

    #[test]
    fn flags_land_where_the_shader_reads_them() {
        let tree = Contree::empty(2);
        let u = march_uniform(
            Mat4::IDENTITY,
            Vec3::ZERO,
            &tree,
            march_flags::DDA | march_flags::BEAM,
        );
        assert_eq!(u.volume_params[2], 0b101);
    }

    #[test]
    fn the_matrix_is_stored_column_major_as_wgsl_expects() {
        let m = Mat4::from_translation(Vec3::new(5.0, 6.0, 7.0));
        let tree = Contree::empty(2);
        let u = march_uniform(m, Vec3::ZERO, &tree, march_flags::NONE);
        // glam is column-major, and to_cols_array_2d yields columns.
        assert_eq!(u.world_from_clip[3][0], 5.0);
        assert_eq!(u.world_from_clip[3][1], 6.0);
        assert_eq!(u.world_from_clip[3][2], 7.0);
    }

    use bevox_core::contree::Contree;
    use bevox_core::material::MaterialId;
    use glam::Vec3;

    /// A scene with something in it, so an edit has existing nodes to rewrite
    /// rather than only allocating fresh ones.
    fn edit_scene() -> VoxelScene {
        let mut tree = Contree::empty(3);
        tree.apply_sphere(Vec3::new(32.0, 32.0, 32.0), 12.0, MaterialId(1));
        // The initial build is not an edit: clear it so a test sees only what
        // the edit under test touched.
        tree.arena_mut().clear_dirty();
        let field = DistanceField::build(&tree);
        VoxelScene { tree, materials: MaterialTable::new(), generation: 1, field, field_dirty: None }
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
}
