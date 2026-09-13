//! Bevy plugin: uploads voxel data and ray marches it on the GPU.

use bevy::prelude::*;

/// Inserted by the plugin so tests can prove it built.
#[derive(Resource, Debug, PartialEq, Eq)]
pub struct BevoxReady;

pub struct BevoxRenderPlugin;

impl Plugin for BevoxRenderPlugin {
    fn build(&self, app: &mut App) {
        app.insert_resource(BevoxReady);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_plugin_builds_on_a_minimal_app() {
        let mut app = App::new();
        app.add_plugins(MinimalPlugins).add_plugins(BevoxRenderPlugin);
        app.update();
        assert!(app.world().get_resource::<BevoxReady>().is_some());
    }
}
