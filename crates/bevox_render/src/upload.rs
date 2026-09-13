//! Moving voxel data and camera parameters into the render world.

use bevy::asset::RenderAssetUsages;
use bevy::prelude::*;
use bevy::render::extract_resource::ExtractResource;
use bevy::render::render_resource::{Extent3d, TextureDimension, TextureFormat, TextureUsages};
use bevox_core::contree::Contree;
use bevox_core::gpu::{GpuNode, GpuVolume};
use bytemuck::{Pod, Zeroable};

/// The scene the renderer draws. Replacing it re-uploads on the next frame.
#[derive(Resource)]
pub struct VoxelScene {
    pub tree: Contree,
    /// Bumped whenever `tree` changes, so the render world knows to re-upload.
    pub generation: u32,
}

/// Camera and volume parameters, as the shader sees them.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, Pod, Zeroable)]
pub struct MarchUniform {
    pub world_from_clip: [[f32; 4]; 4],
    pub camera_position: [f32; 4],
    /// `[depth, extent, 0, 0]`.
    pub volume_params: [u32; 4],
}

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
    pub depth: u32,
    pub extent: u32,
    pub generation: u32,
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
        RenderAssetUsages::RENDER_WORLD,
    );
    // STORAGE_BINDING is the one that matters: without it the bind group is
    // rejected at creation, and the symptom is a black window with a validation
    // error rather than an obvious failure.
    image.texture_descriptor.usage =
        TextureUsages::TEXTURE_BINDING | TextureUsages::COPY_DST | TextureUsages::STORAGE_BINDING;

    let handle = images.add(image);
    commands.spawn(Sprite::from_image(handle.clone()));
    commands.insert_resource(MarchTarget { image: handle, width, height });
}

/// Builds the shader uniform from a camera and the volume being drawn.
///
/// `world_from_clip` is the inverse view-projection: the shader multiplies a
/// clip-space point by it to get a world-space ray target.
pub fn march_uniform(world_from_clip: Mat4, camera_position: Vec3, tree: &Contree) -> MarchUniform {
    MarchUniform {
        world_from_clip: world_from_clip.to_cols_array_2d(),
        camera_position: camera_position.extend(0.0).to_array(),
        volume_params: [tree.depth(), tree.extent(), 0, 0],
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_uniform_is_the_size_the_shader_expects() {
        // mat4x4 (64) + vec4 (16) + uvec4 (16)
        assert_eq!(size_of::<MarchUniform>(), 96);
        assert_eq!(align_of::<MarchUniform>(), 4);
    }

    #[test]
    fn volume_params_carry_depth_and_extent() {
        let tree = Contree::empty(3);
        let u = march_uniform(Mat4::IDENTITY, Vec3::ZERO, &tree);
        assert_eq!(u.volume_params[0], 3);
        assert_eq!(u.volume_params[1], 64);
    }

    #[test]
    fn the_camera_position_survives_into_the_uniform() {
        let tree = Contree::empty(2);
        let u = march_uniform(Mat4::IDENTITY, Vec3::new(1.0, 2.0, 3.0), &tree);
        assert_eq!(u.camera_position[0], 1.0);
        assert_eq!(u.camera_position[1], 2.0);
        assert_eq!(u.camera_position[2], 3.0);
    }

    #[test]
    fn the_matrix_is_stored_column_major_as_wgsl_expects() {
        let m = Mat4::from_translation(Vec3::new(5.0, 6.0, 7.0));
        let tree = Contree::empty(2);
        let u = march_uniform(m, Vec3::ZERO, &tree);
        // glam is column-major, and to_cols_array_2d yields columns.
        assert_eq!(u.world_from_clip[3][0], 5.0);
        assert_eq!(u.world_from_clip[3][1], 6.0);
        assert_eq!(u.world_from_clip[3][2], 7.0);
    }
}
