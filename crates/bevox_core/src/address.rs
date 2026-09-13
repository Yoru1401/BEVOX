//! Addressing vocabulary. The core holds exactly one chunk, at the origin;
//! these types exist so that streaming can add chunks later without changing
//! how a voxel address is spelled.

use glam::{IVec3, UVec3};

#[derive(Clone, Copy, PartialEq, Eq, Hash, Debug, Default)]
pub struct ChunkCoord(pub IVec3);

impl ChunkCoord {
    pub const ORIGIN: ChunkCoord = ChunkCoord(IVec3::ZERO);
}

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub struct VoxelPos {
    pub chunk: ChunkCoord,
    pub local: UVec3,
}

impl VoxelPos {
    pub fn at_origin(local: UVec3) -> Self {
        Self { chunk: ChunkCoord::ORIGIN, local }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn origin_positions_share_a_chunk() {
        let a = VoxelPos::at_origin(UVec3::new(1, 2, 3));
        let b = VoxelPos::at_origin(UVec3::new(9, 9, 9));
        assert_eq!(a.chunk, b.chunk);
        assert_eq!(a.chunk, ChunkCoord::ORIGIN);
    }
}
