//! Bevy plugin: uploads voxel data and ray marches it on the GPU.

pub mod camera;

use bevy::prelude::*;

/// Inserted by the plugin so tests can prove it built.
#[derive(Resource, Debug, PartialEq, Eq)]
pub struct BevoxReady;

/// Requires `InputPlugin`, which `DefaultPlugins` provides. On a hand-built app
/// it must be added, or the camera system fails parameter validation at runtime.
pub struct BevoxRenderPlugin;

impl Plugin for BevoxRenderPlugin {
    fn build(&self, app: &mut App) {
        app.insert_resource(BevoxReady)
            .add_systems(Update, camera::fly_camera_system);
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
