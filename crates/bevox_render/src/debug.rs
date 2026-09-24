//! Which view the window shows: the lit scene, or one of the buffers behind it.
//!
//! Every view is its own compute entry point in `march.wgsl`, and the render
//! world holds one pipeline per view. Nothing branches per pixel: choosing a
//! view chooses a pipeline, so the lit view runs exactly the shader it would
//! run if this module did not exist.

use bevy::prelude::*;
use bevy::render::extract_resource::ExtractResource;

/// What the window draws. `Lit` is the game; the rest are for looking inside it.
#[derive(Resource, ExtractResource, Clone, Copy, PartialEq, Eq, Debug, Default)]
pub enum DebugView {
    #[default]
    Lit,
    /// The implicit shading normal, `n * 0.5 + 0.5` in RGB.
    Normal,
    /// Distance to the hit against the volume's own size, grey.
    Depth,
    /// Where on the hit voxel's face the ray landed, in red and green.
    Uv,
    /// Material index in red. Flat bands, one per material.
    Material,
    /// The hit voxel's coordinate in RGB. A body's is in its own volume.
    VoxelId,
    /// Palette colour with no light on it at all.
    Unlit,
    /// Red where the sun is blocked.
    Shadows,
    /// Red where ambient occlusion shuts the light out.
    Occlusion,
    /// Traversal steps the pixel cost, grey. Bright is expensive.
    Steps,
}

impl DebugView {
    /// Every view, in the order their keys run. `Lit` is first because it is
    /// the one to come back to.
    pub const ALL: [DebugView; 10] = [
        DebugView::Lit,
        DebugView::Normal,
        DebugView::Depth,
        DebugView::Uv,
        DebugView::Material,
        DebugView::VoxelId,
        DebugView::Unlit,
        DebugView::Shadows,
        DebugView::Occlusion,
        DebugView::Steps,
    ];

    /// Where this view sits in `ALL`, which is the order the render world
    /// queues its pipelines in.
    pub fn index(self) -> usize {
        DebugView::ALL.iter().position(|v| *v == self).unwrap_or(0)
    }

    /// The compute entry point that draws this view.
    ///
    /// `the_views_name_entry_points_that_exist` reads the shader source and
    /// holds this table to it, because a typo here is a black window and no
    /// other sign.
    pub fn entry_point(self) -> &'static str {
        match self {
            DebugView::Lit => "march",
            DebugView::Normal => "march_normal",
            DebugView::Depth => "march_depth",
            DebugView::Uv => "march_uv",
            DebugView::Material => "march_identity",
            DebugView::VoxelId => "march_voxel_id",
            DebugView::Unlit => "march_unlit",
            DebugView::Shadows => "march_shadow",
            DebugView::Occlusion => "march_ao",
            DebugView::Steps => "march_steps",
        }
    }

    /// What to call it in a log line.
    pub fn name(self) -> &'static str {
        match self {
            DebugView::Lit => "lit",
            DebugView::Normal => "normals",
            DebugView::Depth => "depth",
            DebugView::Uv => "voxel face UV",
            DebugView::Material => "material id",
            DebugView::VoxelId => "voxel id",
            DebugView::Unlit => "unlit",
            DebugView::Shadows => "sun shadows",
            DebugView::Occlusion => "ambient occlusion",
            DebugView::Steps => "ray steps",
        }
    }

    /// The key that selects it: F1 for the lit scene, then along the row.
    ///
    /// The app reads this rather than keeping its own table, so a view can
    /// never be added in one place and forgotten in the other.
    pub fn key(self) -> KeyCode {
        match self {
            DebugView::Lit => KeyCode::F1,
            DebugView::Normal => KeyCode::F2,
            DebugView::Depth => KeyCode::F3,
            DebugView::Uv => KeyCode::F4,
            DebugView::Material => KeyCode::F5,
            DebugView::VoxelId => KeyCode::F6,
            DebugView::Unlit => KeyCode::F7,
            DebugView::Shadows => KeyCode::F8,
            DebugView::Occlusion => KeyCode::F9,
            DebugView::Steps => KeyCode::F10,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A view whose entry point is not in the shader draws nothing, and the
    /// only symptom is a black window -- wgpu reports the missing entry point
    /// while creating the pipeline, in a log line easily lost among others.
    #[test]
    fn the_views_name_entry_points_that_exist() {
        let source = std::fs::read_to_string("assets/shaders/march.wgsl")
            .expect("the shader is beside the crate");
        for view in DebugView::ALL {
            let declaration = format!("fn {}(", view.entry_point());
            assert!(
                source.contains(&declaration),
                "{:?} names `{}`, which the shader does not declare",
                view,
                view.entry_point()
            );
        }
    }

    /// Two views sharing a key, a name or an entry point is a view you cannot
    /// reach, and the mistake reads as a working table.
    #[test]
    fn every_view_is_distinct() {
        let count = DebugView::ALL.len();
        for (what, mut seen) in [
            ("keys", DebugView::ALL.map(|v| format!("{:?}", v.key())).to_vec()),
            ("entry points", DebugView::ALL.map(|v| v.entry_point().to_string()).to_vec()),
            ("names", DebugView::ALL.map(|v| v.name().to_string()).to_vec()),
        ] {
            seen.sort();
            seen.dedup();
            assert_eq!(seen.len(), count, "two views share one of the {what}");
        }
    }

    /// Every view indexes to itself. A selection that collapses to one index
    /// shows every view as the lit scene, which reads as the keys not working.
    #[test]
    fn a_view_indexes_to_itself() {
        for view in DebugView::ALL {
            assert_eq!(DebugView::ALL[view.index()], view, "{view:?} indexes elsewhere");
        }
    }

    /// `ALL` is the list everything else iterates: the pipelines, the keys and
    /// the tests above. A variant missing from it is a view that exists in the
    /// type and nowhere else.
    #[test]
    fn all_holds_every_variant() {
        // Bevy's derive gives no variant count, so this is the one place that
        // has to be kept by hand -- and it fails loudly rather than silently.
        assert_eq!(DebugView::ALL.len(), 10, "ALL and the enum have parted");
        assert_eq!(DebugView::default(), DebugView::Lit);
    }
}
