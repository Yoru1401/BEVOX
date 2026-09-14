//! Bevy plugin: uploads voxel data and ray marches it on the GPU.

pub mod camera;
pub mod pick;
pub mod pipeline;
pub mod upload;

use bevy::prelude::*;
use bevy::render::extract_resource::ExtractResourcePlugin;
use bevy::render::renderer::{RenderGraph, RenderGraphSystems};
use bevy::render::{ExtractSchedule, Render, RenderApp, RenderStartup, RenderSystems};

/// Inserted by the plugin so tests can prove it built.
#[derive(Resource, Debug, PartialEq, Eq)]
pub struct BevoxReady;

/// Requires `InputPlugin`, which `DefaultPlugins` provides. On a hand-built app
/// it must be added, or the camera system fails parameter validation at runtime.
pub struct BevoxRenderPlugin;

impl Plugin for BevoxRenderPlugin {
    fn build(&self, app: &mut App) {
        app.insert_resource(BevoxReady)
            // Always present, so the renderer draws an empty world rather than
            // going dark when no scene has been inserted.
            .init_resource::<upload::GpuSceneData>()
            .init_resource::<upload::SceneUpdate>()
            .add_plugins(ExtractResourcePlugin::<upload::MarchTarget>::default())
            .add_plugins(ExtractResourcePlugin::<upload::SceneUpdate>::default())
            .add_plugins(ExtractResourcePlugin::<upload::ExtractedMarchCamera>::default())
            .add_systems(
                Update,
                (
                    camera::fly_camera_system,
                    upload::build_gpu_scene,
                    upload::stage_scene_update_system.after(upload::build_gpu_scene),
                    upload::track_march_camera,
                ),
            );

        // No RenderApp means no renderer, so there is nothing to draw into and
        // nothing to dispatch. This is also what keeps the MinimalPlugins smoke
        // test working.
        if app.get_sub_app(RenderApp).is_none() {
            return;
        }
        app.add_systems(Startup, upload::create_march_target);

        let render_app = app.sub_app_mut(RenderApp);
        render_app
            .add_systems(RenderStartup, pipeline::init_march_pipeline)
            .add_systems(ExtractSchedule, upload::extract_gpu_scene)
            .add_systems(
                Render,
                pipeline::prepare_march_buffers.in_set(RenderSystems::Prepare),
            )
            .add_systems(
                RenderGraph,
                pipeline::dispatch_march.in_set(RenderGraphSystems::Begin),
            );
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_plugin_builds_on_a_minimal_app() {
        let mut app = App::new();
        // MinimalPlugins deliberately has no RenderApp, which is the point of
        // this test. InputPlugin is added because the camera system needs it.
        app.add_plugins(MinimalPlugins)
            .add_plugins(bevy::input::InputPlugin)
            .add_plugins(BevoxRenderPlugin);
        app.update();
        assert!(app.world().get_resource::<BevoxReady>().is_some());
    }
}
