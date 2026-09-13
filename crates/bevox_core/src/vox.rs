//! MagicaVoxel import.
//!
//! Three conventions differ from ours and all are handled here rather than at
//! call sites: MagicaVoxel is Z-up where we are Y-up, its model sizes are
//! arbitrary where our volumes are powers of four, and its in-memory palette
//! indices are zero-based where ours reserve zero for empty space.

use crate::dense::DenseVolume;
use crate::material::{Material, MaterialId, MaterialTable};
use dot_vox::DotVoxData;
use glam::UVec3;

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

    /// `i` here is dot_vox's in-memory palette index, which is already one less
    /// than the number stored in the file.
    fn model(size: (u32, u32, u32), voxels: &[(u8, u8, u8, u8)]) -> DotVoxData {
        DotVoxData {
            version: 150,
            index_map: Vec::new(),
            models: vec![dot_vox::Model {
                size: dot_vox::Size { x: size.0, y: size.1, z: size.2 },
                voxels: voxels
                    .iter()
                    .map(|(x, y, z, i)| dot_vox::Voxel { x: *x, y: *y, z: *z, i: *i })
                    .collect(),
            }],
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
