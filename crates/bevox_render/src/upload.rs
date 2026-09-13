//! Moving voxel data and camera parameters into the render world.

use bevy::asset::RenderAssetUsages;
use bevy::prelude::*;
use bevy::render::extract_resource::ExtractResource;
use bevy::render::render_resource::{Extent3d, TextureDimension, TextureFormat, TextureUsages};
use bevox_core::contree::Contree;
use bevox_core::gpu::{GpuNode, GpuVolume};
use bevox_core::material::MaterialTable;
use bytemuck::{Pod, Zeroable};

/// The scene the renderer draws. Replacing it re-uploads on the next frame.
#[derive(Resource)]
pub struct VoxelScene {
    pub tree: Contree,
    /// Colours the voxel material indices name.
    pub materials: MaterialTable,
    /// Bumped whenever `tree` changes, so the render world knows to re-upload.
    pub generation: u32,
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
    /// What the app runs. Each optimisation joins this only once it has measured
    /// faster while staying bit-identical.
    pub const DEFAULT: u32 = NONE;
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
/// ponytail: cloned into the render world every frame. Fine at 64^3 (a few
/// hundred KB); switch to uploading only when `generation` changes once editing
/// lands and scenes get large.
#[derive(Resource, Clone, ExtractResource)]
pub struct GpuSceneData {
    pub nodes: Vec<GpuNode>,
    pub voxels: Vec<u32>,
    pub palette: Vec<[f32; 4]>,
    pub depth: u32,
    pub extent: u32,
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
            // Depth must be at least 1: the shader starts at level `depth - 1`.
            depth: 1,
            extent: 4,
            generation: 0,
        }
    }
}

/// The active camera, as the render world needs it.
#[derive(Resource, Clone, ExtractResource)]
pub struct ExtractedMarchCamera {
    pub world_from_clip: Mat4,
    pub position: Vec3,
}

/// Rebuilds the GPU-side representation whenever the scene changes.
pub fn build_gpu_scene(mut commands: Commands, scene: Option<Res<VoxelScene>>) {
    // No scene inserted yet is a normal state, not an error.
    let Some(scene) = scene else {
        return;
    };
    if !scene.is_changed() {
        return;
    }
    let volume = GpuVolume::from_contree(&scene.tree);
    commands.insert_resource(GpuSceneData {
        nodes: volume.buffer_nodes(),
        voxels: volume.voxels,
        palette: scene.materials.to_gpu(),
        depth: scene.tree.depth(),
        extent: scene.tree.extent(),
        generation: scene.generation,
    });
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
    MarchUniform {
        world_from_clip: world_from_clip.to_cols_array_2d(),
        camera_position: camera_position.extend(0.0).to_array(),
        sun_direction: SUN_DIRECTION.normalize().extend(0.0).to_array(),
        volume_params: [tree.depth(), tree.extent(), flags, 0],
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_uniform_is_the_size_the_shader_expects() {
        // mat4x4 (64) + vec4 (16) + vec4 sun (16) + uvec4 (16)
        assert_eq!(size_of::<MarchUniform>(), 112);
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
}
