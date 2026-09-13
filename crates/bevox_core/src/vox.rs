//! MagicaVoxel import.
//!
//! Three conventions differ from ours and all are handled here rather than at
//! call sites: MagicaVoxel is Z-up where we are Y-up, its model sizes are
//! arbitrary where our volumes are powers of four, and its in-memory palette
//! indices are zero-based where ours reserve zero for empty space.

use crate::dense::DenseVolume;
use crate::material::{Material, MaterialId, MaterialTable};
use dot_vox::{DotVoxData, Frame, SceneNode};
use glam::{IVec3, UVec3};

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum VoxError {
    /// The file could not be read or parsed at all.
    ReadFailed,
    NoModels,
    ModelOutOfRange { index: usize, count: usize },
    /// Larger than the biggest volume the engine addresses.
    TooLarge { extent: u32 },
}

impl core::fmt::Display for VoxError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            VoxError::ReadFailed => write!(f, "the file could not be read as MagicaVoxel data"),
            VoxError::NoModels => write!(f, "the file contains no models"),
            VoxError::ModelOutOfRange { index, count } => {
                write!(f, "model {index} requested but the file has {count}")
            }
            VoxError::TooLarge { extent } => {
                write!(f, "model needs an extent of {extent}, beyond the 1024 limit")
            }
        }
    }
}

impl core::error::Error for VoxError {}

/// Smallest power of four that fits `size`, or `None` beyond 1024.
pub fn fitting_extent(size: u32) -> Option<u32> {
    let mut extent = 4u32;
    while extent < size {
        extent *= 4;
        if extent > 1024 {
            return None;
        }
    }
    Some(extent)
}

/// Converts one model to a padded volume plus its palette.
pub fn import_model(
    data: &DotVoxData,
    index: usize,
) -> Result<(DenseVolume, MaterialTable), VoxError> {
    if data.models.is_empty() {
        return Err(VoxError::NoModels);
    }
    let model = data
        .models
        .get(index)
        .ok_or(VoxError::ModelOutOfRange { index, count: data.models.len() })?;

    let longest = model.size.x.max(model.size.y).max(model.size.z);
    let extent = fitting_extent(longest).ok_or(VoxError::TooLarge { extent: longest })?;

    let mut volume = DenseVolume::new(extent).expect("fitting_extent returns a legal extent");
    for v in &model.voxels {
        // Our axes are Y-up, MagicaVoxel's are Z-up, so its y and z swap.
        let p = UVec3::new(v.x as u32, v.z as u32, v.y as u32);
        // File data is untrusted: a malformed model can name a voxel outside the
        // size it declared, and that must not panic the importer.
        if p.x >= extent || p.y >= extent || p.z >= extent {
            continue;
        }
        // dot_vox's in-memory palette indices are zero-based, ours reserve zero
        // for empty, so everything shifts up by one. Without this, every voxel
        // using the first palette colour would silently become empty space.
        volume.set(p, MaterialId(v.i.saturating_add(1)));
    }

    let mut materials = MaterialTable::new();
    for c in data.palette.iter().take(255) {
        materials
            .push(Material { color: [c.r, c.g, c.b, c.a] })
            .expect("at most 255 entries are pushed");
    }

    Ok((volume, materials))
}

/// One model, positioned by the scene graph.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PlacedModel {
    pub model_id: u32,
    /// Translation in MagicaVoxel's Z-up space, of the model's centre.
    pub translation: IVec3,
    /// Signed permutation matrix, columns as dot_vox reports them.
    pub rotation: [[f32; 3]; 3],
}

/// Upper bound on nodes visited, so a malformed or cyclic graph terminates.
const MAX_SCENE_NODES: usize = 1 << 20;

/// Flattens the scene graph into placed models.
///
/// A file with no graph yields every model at the origin, so single-model files
/// travel the same path as scenes.
pub fn placed_models(data: &DotVoxData) -> Vec<PlacedModel> {
    if data.scenes.is_empty() {
        return (0..data.models.len() as u32)
            .map(|model_id| PlacedModel {
                model_id,
                translation: IVec3::ZERO,
                rotation: identity_rotation(),
            })
            .collect();
    }

    let mut out = Vec::new();
    // Explicit stack rather than recursion: these graphs are file data and can
    // be deep or malformed.
    let mut stack = vec![(0u32, IVec3::ZERO, identity_rotation())];
    let mut visited = 0usize;

    while let Some((index, translation, rotation)) = stack.pop() {
        visited += 1;
        if visited > MAX_SCENE_NODES {
            break;
        }
        let Some(node) = data.scenes.get(index as usize) else {
            continue;
        };

        match node {
            SceneNode::Transform { frames, child, .. } => {
                // Only the first frame is used: animation is out of scope, and a
                // static import wants the model's resting position.
                let (t, r) = frames
                    .first()
                    .map(frame_transform)
                    .unwrap_or((IVec3::ZERO, identity_rotation()));
                let combined_rotation = multiply(rotation, r);
                let combined_translation = translation + apply(rotation, t);
                stack.push((*child, combined_translation, combined_rotation));
            }
            SceneNode::Group { children, .. } => {
                // Reversed so children are emitted in file order once popped.
                for child in children.iter().rev() {
                    stack.push((*child, translation, rotation));
                }
            }
            SceneNode::Shape { models, .. } => {
                for m in models {
                    out.push(PlacedModel { model_id: m.model_id, translation, rotation });
                }
            }
        }
    }

    out
}

/// Places one of a model's voxels into scene space, still Z-up.
///
/// MagicaVoxel's translations position the model's *centre*, not its corner, so
/// the voxel is measured from that centre before being rotated and moved.
/// Reversing the two shifts every model by half its own size, which reads as a
/// scene that is slightly wrong rather than obviously broken.
pub fn place_voxel(local: UVec3, size: UVec3, placed: &PlacedModel) -> IVec3 {
    let centred = local.as_ivec3() - (size.as_ivec3() / 2);
    placed.translation + apply(placed.rotation, centred)
}

fn identity_rotation() -> [[f32; 3]; 3] {
    [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
}

fn frame_transform(frame: &Frame) -> (IVec3, [[f32; 3]; 3]) {
    let t = frame.position().map(|p| IVec3::new(p.x, p.y, p.z)).unwrap_or(IVec3::ZERO);
    let r = frame
        .orientation()
        .map(|o| o.to_cols_array_2d())
        .unwrap_or_else(identity_rotation);
    (t, r)
}

/// Column-major matrix product, matching dot_vox's column convention.
fn multiply(a: [[f32; 3]; 3], b: [[f32; 3]; 3]) -> [[f32; 3]; 3] {
    let mut out = [[0.0f32; 3]; 3];
    for c in 0..3 {
        for r in 0..3 {
            out[c][r] = (0..3).map(|k| a[k][r] * b[c][k]).sum();
        }
    }
    out
}

/// Applies a signed permutation matrix to an integer vector.
fn apply(m: [[f32; 3]; 3], v: IVec3) -> IVec3 {
    let f = v.as_vec3();
    IVec3::new(
        (m[0][0] * f.x + m[1][0] * f.y + m[2][0] * f.z).round() as i32,
        (m[0][1] * f.x + m[1][1] * f.y + m[2][1] * f.z).round() as i32,
        (m[0][2] * f.x + m[1][2] * f.y + m[2][2] * f.z).round() as i32,
    )
}

/// Reads a `.vox` file and imports its first model.
pub fn load_vox(path: &std::path::Path) -> Result<(DenseVolume, MaterialTable), VoxError> {
    // A missing file and a file with no models are different faults, and saying
    // "contains no models" about a path that does not exist sends the reader
    // looking in the wrong place.
    let data = dot_vox::load(path.to_str().unwrap_or_default()).map_err(|_| VoxError::ReadFailed)?;
    import_model(&data, 0)
}

#[cfg(test)]
mod tests {
    use super::*;
    use dot_vox::{Dict, Frame, SceneNode, ShapeModel};
    use glam::IVec3;

    /// A transform node carrying a translation, wrapping `child`.
    fn transform_node(child: u32, t: (i32, i32, i32)) -> SceneNode {
        let mut attributes = Dict::new();
        attributes.insert("_t".to_string(), format!("{} {} {}", t.0, t.1, t.2));
        SceneNode::Transform {
            attributes: Dict::new(),
            frames: vec![Frame::new(attributes)],
            child,
            layer_id: 0,
        }
    }

    fn group_node(children: Vec<u32>) -> SceneNode {
        SceneNode::Group { attributes: Dict::new(), children }
    }

    fn shape_node(model_id: u32) -> SceneNode {
        SceneNode::Shape {
            attributes: Dict::new(),
            models: vec![ShapeModel { model_id, attributes: Dict::new() }],
        }
    }

    #[test]
    fn a_file_with_no_scene_graph_places_every_model_at_the_origin() {
        let mut data = model((4, 4, 4), &[(0, 0, 0, 0)]);
        data.models.push(vox_model((4, 4, 4), &[(0, 0, 0, 0)]));
        data.scenes.clear();

        let placed = placed_models(&data);
        assert_eq!(placed.len(), 2, "both models should be placed");
        assert!(placed.iter().all(|p| p.translation == IVec3::ZERO));
    }

    #[test]
    fn a_transform_above_a_shape_places_that_model() {
        let mut data = model((4, 4, 4), &[(0, 0, 0, 0)]);
        data.scenes = vec![transform_node(1, (10, 20, 30)), shape_node(0)];

        let placed = placed_models(&data);
        assert_eq!(placed.len(), 1);
        assert_eq!(placed[0].model_id, 0);
        assert_eq!(placed[0].translation, IVec3::new(10, 20, 30));
    }

    #[test]
    fn nested_transforms_accumulate() {
        let mut data = model((4, 4, 4), &[(0, 0, 0, 0)]);
        // root transform -> group -> transform -> shape
        data.scenes = vec![
            transform_node(1, (100, 0, 0)),
            group_node(vec![2]),
            transform_node(3, (5, 7, 0)),
            shape_node(0),
        ];

        let placed = placed_models(&data);
        assert_eq!(placed.len(), 1);
        assert_eq!(placed[0].translation, IVec3::new(105, 7, 0));
    }

    #[test]
    fn a_group_places_each_of_its_children() {
        let mut data = model((4, 4, 4), &[(0, 0, 0, 0)]);
        data.models.push(vox_model((4, 4, 4), &[(0, 0, 0, 0)]));
        data.scenes = vec![
            group_node(vec![1, 3]),
            transform_node(2, (10, 0, 0)),
            shape_node(0),
            transform_node(4, (-10, 0, 0)),
            shape_node(1),
        ];

        let placed = placed_models(&data);
        assert_eq!(placed.len(), 2);
        assert_eq!(placed[0].translation, IVec3::new(10, 0, 0));
        assert_eq!(placed[1].translation, IVec3::new(-10, 0, 0));
    }

    fn placed_at(t: (i32, i32, i32)) -> PlacedModel {
        PlacedModel {
            model_id: 0,
            translation: IVec3::new(t.0, t.1, t.2),
            rotation: identity_rotation(),
        }
    }

    #[test]
    fn an_untransformed_model_keeps_its_shape_around_the_origin() {
        let size = UVec3::new(4, 4, 4);
        // The centre voxel of a 4-cube sits at local (2,2,2), which maps to 0.
        assert_eq!(place_voxel(UVec3::new(2, 2, 2), size, &placed_at((0, 0, 0))), IVec3::ZERO);
        assert_eq!(
            place_voxel(UVec3::new(0, 0, 0), size, &placed_at((0, 0, 0))),
            IVec3::new(-2, -2, -2)
        );
    }

    #[test]
    fn translation_moves_the_whole_model() {
        let size = UVec3::new(4, 4, 4);
        assert_eq!(
            place_voxel(UVec3::new(2, 2, 2), size, &placed_at((10, 20, 30))),
            IVec3::new(10, 20, 30)
        );
    }

    #[test]
    fn a_quarter_turn_about_z_swaps_x_and_y() {
        // Signed permutation: x <- -y, y <- x.
        let rotation = [[0.0, 1.0, 0.0], [-1.0, 0.0, 0.0], [0.0, 0.0, 1.0]];
        let placed = PlacedModel { model_id: 0, translation: IVec3::ZERO, rotation };
        let size = UVec3::new(4, 4, 4);

        // Local (3,2,2) is +1 on x from the centre; after the turn it is +1 on y.
        assert_eq!(place_voxel(UVec3::new(3, 2, 2), size, &placed), IVec3::new(0, 1, 0));
    }

    #[test]
    fn rotation_happens_before_translation() {
        let rotation = [[0.0, 1.0, 0.0], [-1.0, 0.0, 0.0], [0.0, 0.0, 1.0]];
        let placed = PlacedModel { model_id: 0, translation: IVec3::new(100, 0, 0), rotation };
        let size = UVec3::new(4, 4, 4);
        // Rotating first then translating puts this at (100, 1, 0); translating
        // first would put it at (0, 101, 0).
        assert_eq!(place_voxel(UVec3::new(3, 2, 2), size, &placed), IVec3::new(100, 1, 0));
    }

    #[test]
    fn a_cycle_in_the_graph_terminates() {
        // Malformed files exist. A child index pointing back at an ancestor must
        // not hang the importer.
        let mut data = model((4, 4, 4), &[(0, 0, 0, 0)]);
        data.scenes = vec![transform_node(1, (1, 0, 0)), group_node(vec![0])];

        let placed = placed_models(&data);
        assert!(placed.len() < 100, "walk did not terminate, produced {}", placed.len());
    }

    #[test]
    fn a_child_index_past_the_end_is_ignored() {
        let mut data = model((4, 4, 4), &[(0, 0, 0, 0)]);
        data.scenes = vec![transform_node(99, (1, 0, 0))];
        assert!(placed_models(&data).is_empty());
    }

    /// dot_vox's Model does not implement Clone, so tests build their own.
    fn vox_model(size: (u32, u32, u32), voxels: &[(u8, u8, u8, u8)]) -> dot_vox::Model {
        dot_vox::Model {
            size: dot_vox::Size { x: size.0, y: size.1, z: size.2 },
            voxels: voxels
                .iter()
                .map(|(x, y, z, i)| dot_vox::Voxel { x: *x, y: *y, z: *z, i: *i })
                .collect(),
        }
    }

    /// `i` here is dot_vox's in-memory palette index, which is already one less
    /// than the number stored in the file.
    fn model(size: (u32, u32, u32), voxels: &[(u8, u8, u8, u8)]) -> DotVoxData {
        DotVoxData {
            version: 150,
            index_map: Vec::new(),
            models: vec![vox_model(size, voxels)],
            // Palette entry k is identifiable by its red channel being k.
            palette: (0..256)
                .map(|i| dot_vox::Color { r: i as u8, g: 0, b: 0, a: 255 })
                .collect(),
            materials: Vec::new(),
            scenes: Vec::new(),
            layers: Vec::new(),
        }
    }

    #[test]
    fn extents_round_up_to_powers_of_four() {
        assert_eq!(fitting_extent(1), Some(4));
        assert_eq!(fitting_extent(4), Some(4));
        assert_eq!(fitting_extent(5), Some(16));
        assert_eq!(fitting_extent(64), Some(64));
        assert_eq!(fitting_extent(126), Some(256));
        assert_eq!(fitting_extent(1024), Some(1024));
        assert_eq!(fitting_extent(1025), None);
    }

    #[test]
    fn a_file_with_no_models_is_an_error() {
        let mut data = model((4, 4, 4), &[]);
        data.models.clear();
        assert_eq!(import_model(&data, 0).unwrap_err(), VoxError::NoModels);
    }

    #[test]
    fn asking_for_a_model_beyond_the_file_is_an_error() {
        let data = model((4, 4, 4), &[]);
        assert_eq!(
            import_model(&data, 3).unwrap_err(),
            VoxError::ModelOutOfRange { index: 3, count: 1 }
        );
    }

    #[test]
    fn z_up_becomes_y_up() {
        // A voxel high on MagicaVoxel's z axis must land high on our y.
        let data = model((4, 4, 4), &[(1, 2, 3, 6)]);
        let (volume, _) = import_model(&data, 0).unwrap();
        assert_eq!(volume.get(UVec3::new(1, 3, 2)), MaterialId(7));
        assert_eq!(volume.get(UVec3::new(1, 2, 3)), MaterialId::EMPTY);
    }

    /// dot_vox stores palette indices zero-based, and our zero means empty, so a
    /// direct copy would delete every voxel using the first palette colour.
    #[test]
    fn palette_index_zero_is_a_real_voxel_not_empty_space() {
        let data = model((4, 4, 4), &[(0, 0, 0, 0)]);
        let (volume, materials) = import_model(&data, 0).unwrap();

        let id = volume.get(UVec3::new(0, 0, 0));
        assert_ne!(id, MaterialId::EMPTY, "the voxel vanished");
        assert_eq!(id, MaterialId(1));
        // Slot 1 holds dot_vox palette entry 0, whose red channel is 0.
        assert_eq!(materials.get(id).color[0], 0);
    }

    #[test]
    fn the_volume_is_padded_to_a_power_of_four() {
        let data = model((5, 5, 5), &[(4, 4, 4, 1)]);
        let (volume, _) = import_model(&data, 0).unwrap();
        assert_eq!(volume.extent(), 16);
        assert_eq!(volume.get(UVec3::new(4, 4, 4)), MaterialId(2));
    }

    #[test]
    fn the_palette_is_carried_across_with_the_index_shift() {
        let data = model((4, 4, 4), &[(0, 0, 0, 5)]);
        let (_, materials) = import_model(&data, 0).unwrap();
        // dot_vox index 5 becomes our MaterialId(6), holding palette entry 5.
        assert_eq!(materials.get(MaterialId(6)).color[0], 5);
        assert_eq!(materials.get(MaterialId::EMPTY).color[3], 0, "slot 0 stays empty");
    }

    #[test]
    fn an_oversized_model_is_rejected() {
        let data = model((2000, 4, 4), &[]);
        assert_eq!(import_model(&data, 0).unwrap_err(), VoxError::TooLarge { extent: 2000 });
    }
}
