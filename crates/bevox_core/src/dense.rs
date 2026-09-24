//! A flat cubic volume. Reference representation for tests and model import.

use crate::material::MaterialId;
use glam::UVec3;

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct DenseVolume {
    extent: u32,
    data: Vec<u8>,
}

/// Why a volume could not be created.
///
/// This is a real error rather than an assertion because model import feeds
/// extents straight from files, and MagicaVoxel models are arbitrary sizes.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum VolumeError {
    /// Extents must be 4, 16, 64, 256 ... so that they map onto whole tree levels.
    InvalidExtent { extent: u32 },
}

impl core::fmt::Display for VolumeError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            VolumeError::InvalidExtent { extent } => write!(
                f,
                "volume extent {extent} is not a power of four of at least 4"
            ),
        }
    }
}

impl core::error::Error for VolumeError {}

impl DenseVolume {
    /// Creates an empty volume.
    ///
    /// `extent` must be a power of four, at least 4, so that it maps onto whole
    /// tree levels. Anything else is rejected here, at the boundary, which is
    /// what lets [`crate::contree::Contree::from_dense`] be infallible: a
    /// `DenseVolume` that exists is proof its extent is legal.
    pub fn new(extent: u32) -> Result<Self, VolumeError> {
        if extent < crate::node::BRICK_EDGE
            || !extent.is_power_of_two()
            || !extent.trailing_zeros().is_multiple_of(2)
        {
            return Err(VolumeError::InvalidExtent { extent });
        }
        Ok(Self { extent, data: vec![0; (extent as usize).pow(3)] })
    }

    pub fn extent(&self) -> u32 {
        self.extent
    }

    fn index(&self, p: UVec3) -> usize {
        debug_assert!(p.x < self.extent && p.y < self.extent && p.z < self.extent);
        let e = self.extent as usize;
        p.x as usize + p.y as usize * e + p.z as usize * e * e
    }

    pub fn get(&self, p: UVec3) -> MaterialId {
        MaterialId(self.data[self.index(p)])
    }

    pub fn set(&mut self, p: UVec3, material: MaterialId) {
        let i = self.index(p);
        self.data[i] = material.0;
    }

    /// Convenience for tests and examples.
    pub fn into_contree(self) -> crate::contree::Contree {
        crate::contree::Contree::from_dense(&self)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn extents_that_are_not_powers_of_four_are_rejected() {
        for bad in [0u32, 1, 2, 8, 15, 32, 100] {
            assert_eq!(
                DenseVolume::new(bad),
                Err(VolumeError::InvalidExtent { extent: bad }),
                "extent {bad} should have been rejected"
            );
        }
        for good in [4u32, 16, 64, 256] {
            assert!(DenseVolume::new(good).is_ok(), "extent {good} should be legal");
        }
    }

    #[test]
    fn a_new_volume_is_entirely_empty() {
        let volume = DenseVolume::new(16).unwrap();
        assert_eq!(volume.extent(), 16);
        assert_eq!(volume.get(UVec3::new(3, 4, 5)), MaterialId::EMPTY);
    }

    #[test]
    fn set_then_get_returns_the_material() {
        let mut volume = DenseVolume::new(16).unwrap();
        volume.set(UVec3::new(3, 4, 5), MaterialId(9));
        assert_eq!(volume.get(UVec3::new(3, 4, 5)), MaterialId(9));
        assert_eq!(volume.get(UVec3::new(3, 4, 6)), MaterialId::EMPTY);
    }
}
