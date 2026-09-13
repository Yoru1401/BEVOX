//! Translation to GPU-friendly layout.
//!
//! WGSL has no portable 64-bit integer and no byte-addressable storage, so
//! masks cross as two u32 halves and voxel bytes are packed four per word.
//! This is a translation layer only: the in-memory types are unchanged.

use crate::contree::Contree;
use crate::node::Node;
use bytemuck::{Pod, Zeroable};

#[repr(C)]
#[derive(Clone, Copy, Debug, PartialEq, Eq, Pod, Zeroable, Default)]
pub struct GpuNode {
    pub mask_lo: u32,
    pub mask_hi: u32,
    pub child_base: u32,
    pub material: u32,
}

#[derive(Clone, Debug)]
pub struct GpuVolume {
    pub nodes: Vec<GpuNode>,
    pub voxels: Vec<u32>,
    pub root: GpuNode,
    pub depth: u32,
}

impl From<Node> for GpuNode {
    fn from(node: Node) -> Self {
        GpuNode {
            mask_lo: node.mask as u32,
            mask_hi: (node.mask >> 32) as u32,
            child_base: node.child_base,
            material: node.material,
        }
    }
}

/// Packs voxel bytes four to a word, little-endian within the word.
pub fn pack_voxels(bytes: &[u8]) -> Vec<u32> {
    let mut words = vec![0u32; bytes.len().div_ceil(4)];
    for (i, b) in bytes.iter().enumerate() {
        words[i / 4] |= (*b as u32) << ((i % 4) * 8);
    }
    words
}

/// Reads back a single voxel byte from packed words.
pub fn unpack_voxel(words: &[u32], index: usize) -> u8 {
    ((words[index / 4] >> ((index % 4) * 8)) & 0xFF) as u8
}

impl GpuVolume {
    pub fn from_contree(tree: &Contree) -> Self {
        GpuVolume {
            nodes: tree.arena().nodes().iter().copied().map(GpuNode::from).collect(),
            voxels: pack_voxels(tree.arena().voxels()),
            root: GpuNode::from(tree.root()),
            depth: tree.depth(),
        }
    }

    pub fn node_bytes(&self) -> &[u8] {
        bytemuck::cast_slice(&self.nodes)
    }

    pub fn voxel_bytes(&self) -> &[u8] {
        bytemuck::cast_slice(&self.voxels)
    }

    /// Root first, then the arena — the layout the shader expects, where arena
    /// slot `n` lives at buffer index `n + 1`. The shader reads the root from
    /// index 0, so traversal starts without a special case for it.
    pub fn buffer_nodes(&self) -> Vec<GpuNode> {
        let mut out = Vec::with_capacity(self.nodes.len() + 1);
        out.push(self.root);
        out.extend_from_slice(&self.nodes);
        out
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::dense::DenseVolume;
    use crate::material::MaterialId;
    use glam::UVec3;

    #[test]
    fn gpu_node_is_sixteen_bytes() {
        assert_eq!(size_of::<GpuNode>(), 16);
    }

    #[test]
    fn a_mask_survives_the_split_into_halves() {
        let node = Node::subdivided(0xDEAD_BEEF_1234_5678, 42);
        let gpu = GpuNode::from(node);
        assert_eq!(gpu.mask_lo, 0x1234_5678);
        assert_eq!(gpu.mask_hi, 0xDEAD_BEEF);
        assert_eq!(gpu.child_base, 42);
        assert_eq!(
            (gpu.mask_hi as u64) << 32 | gpu.mask_lo as u64,
            0xDEAD_BEEF_1234_5678
        );
    }

    #[test]
    fn the_top_and_bottom_mask_bits_both_survive() {
        let node = Node::subdivided(1u64 << 63 | 1, 0);
        let gpu = GpuNode::from(node);
        assert_eq!(gpu.mask_lo, 1);
        assert_eq!(gpu.mask_hi, 1 << 31);
    }

    #[test]
    fn voxels_pack_four_to_a_word_and_unpack_again() {
        let bytes: Vec<u8> = (0..10u8).map(|i| i * 7 + 1).collect();
        let packed = pack_voxels(&bytes);
        assert_eq!(packed.len(), 3, "10 bytes need 3 words");
        for (i, b) in bytes.iter().enumerate() {
            assert_eq!(unpack_voxel(&packed, i), *b, "byte {i}");
        }
    }

    #[test]
    fn packing_an_empty_slice_yields_no_words() {
        assert!(pack_voxels(&[]).is_empty());
    }

    #[test]
    fn a_volume_converts_with_its_arena_intact() {
        let mut dense = DenseVolume::new(16).unwrap();
        dense.set(UVec3::new(3, 4, 5), MaterialId(9));
        dense.set(UVec3::new(9, 1, 2), MaterialId(4));
        let tree = Contree::from_dense(&dense);
        let gpu = GpuVolume::from_contree(&tree);

        assert_eq!(gpu.depth, tree.depth());
        assert_eq!(gpu.nodes.len(), tree.arena().nodes().len());
        assert_eq!(gpu.root, GpuNode::from(tree.root()));
        // Every arena node survived the conversion.
        for (i, n) in tree.arena().nodes().iter().enumerate() {
            assert_eq!(gpu.nodes[i], GpuNode::from(*n), "node {i}");
        }
        // Every voxel byte is recoverable from the packed words.
        for (i, b) in tree.arena().voxels().iter().enumerate() {
            assert_eq!(unpack_voxel(&gpu.voxels, i), *b, "voxel {i}");
        }
    }

    #[test]
    fn byte_views_have_the_expected_lengths() {
        let tree = Contree::from_dense(&DenseVolume::new(16).unwrap());
        let gpu = GpuVolume::from_contree(&tree);
        assert_eq!(gpu.node_bytes().len(), gpu.nodes.len() * 16);
        assert_eq!(gpu.voxel_bytes().len(), gpu.voxels.len() * 4);
    }

    #[test]
    fn buffer_layout_puts_the_root_first_and_shifts_the_arena_by_one() {
        let mut dense = DenseVolume::new(16).unwrap();
        dense.set(UVec3::new(1, 1, 1), MaterialId(2));
        let tree = Contree::from_dense(&dense);
        let gpu = GpuVolume::from_contree(&tree);
        let buffer = gpu.buffer_nodes();

        assert_eq!(buffer.len(), gpu.nodes.len() + 1);
        assert_eq!(buffer[0], gpu.root);
        for (i, n) in gpu.nodes.iter().enumerate() {
            assert_eq!(buffer[i + 1], *n, "arena slot {i} should sit at index {}", i + 1);
        }
    }
}
