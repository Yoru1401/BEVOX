//! The contree node. 16 bytes, uploaded to the GPU unchanged by a later plan.

use crate::material::MaterialId;

/// Voxels per axis within one node.
pub const BRICK_EDGE: u32 = 4;
/// Children per node.
pub const CHILDREN: u32 = 64;

#[repr(C)]
#[derive(Clone, Copy, PartialEq, Eq, Debug, Default)]
pub struct Node {
    /// Which of the 64 children exist.
    pub mask: u64,
    /// Arena index of the first child. Children are contiguous.
    pub child_base: u32,
    /// Material table index. Meaningful only when `mask == 0`:
    /// 0 means the region is empty, non-zero means solid of that material.
    pub material: u32,
}

/// Child ordinal from child-space coordinates, each in `0..BRICK_EDGE`.
pub const fn child_index(x: u32, y: u32, z: u32) -> u32 {
    x + y * BRICK_EDGE + z * BRICK_EDGE * BRICK_EDGE
}

impl Node {
    pub const EMPTY: Node = Node { mask: 0, child_base: 0, material: 0 };

    /// A region entirely filled with one material, stored without children.
    pub fn uniform(material: MaterialId) -> Node {
        Node { mask: 0, child_base: 0, material: material.0 as u32 }
    }

    /// A region with children. `mask` must be non-zero.
    pub fn subdivided(mask: u64, child_base: u32) -> Node {
        debug_assert!(mask != 0, "a subdivided node must have at least one child");
        Node { mask, child_base, material: 0 }
    }

    pub fn is_subdivided(self) -> bool {
        self.mask != 0
    }

    pub fn is_empty(self) -> bool {
        self.mask == 0 && self.material == 0
    }

    pub fn is_uniform_solid(self) -> bool {
        self.mask == 0 && self.material != 0
    }

    /// Only meaningful when the node is not subdivided.
    pub fn material(self) -> MaterialId {
        MaterialId(self.material as u8)
    }

    pub fn child_count(self) -> u32 {
        self.mask.count_ones()
    }

    /// Arena slot of a child, or `None` when that child does not exist.
    pub fn child_slot(self, child: u32) -> Option<u32> {
        debug_assert!(child < CHILDREN);
        if self.mask & (1u64 << child) == 0 {
            return None;
        }
        let prefix = self.mask & ((1u64 << child) - 1);
        Some(self.child_base + prefix.count_ones())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn node_is_sixteen_bytes() {
        assert_eq!(size_of::<Node>(), 16);
        assert_eq!(align_of::<Node>(), 8);
    }

    #[test]
    fn empty_and_uniform_are_distinguishable() {
        assert!(Node::EMPTY.is_empty());
        assert!(!Node::EMPTY.is_uniform_solid());
        assert!(!Node::EMPTY.is_subdivided());

        let solid = Node::uniform(MaterialId(7));
        assert!(!solid.is_empty());
        assert!(solid.is_uniform_solid());
        assert!(!solid.is_subdivided());
        assert_eq!(solid.material(), MaterialId(7));
    }

    #[test]
    fn child_slot_is_none_when_the_bit_is_clear() {
        let node = Node::subdivided(1 << 5, 100);
        assert_eq!(node.child_slot(4), None);
        assert_eq!(node.child_slot(5), Some(100));
        assert_eq!(node.child_count(), 1);
    }

    #[test]
    fn child_slot_offsets_by_popcount_prefix() {
        // Children present at ordinals 1, 5 and 9, stored contiguously from 100.
        let mask = (1u64 << 1) | (1u64 << 5) | (1u64 << 9);
        let node = Node::subdivided(mask, 100);
        assert_eq!(node.child_slot(1), Some(100));
        assert_eq!(node.child_slot(5), Some(101));
        assert_eq!(node.child_slot(9), Some(102));
        assert_eq!(node.child_count(), 3);
    }

    #[test]
    fn child_slot_handles_the_first_and_last_ordinals() {
        let node = Node::subdivided(u64::MAX, 0);
        assert_eq!(node.child_slot(0), Some(0));
        assert_eq!(node.child_slot(63), Some(63));
    }

    #[test]
    fn child_index_matches_x_major_order() {
        assert_eq!(child_index(0, 0, 0), 0);
        assert_eq!(child_index(3, 0, 0), 3);
        assert_eq!(child_index(0, 1, 0), 4);
        assert_eq!(child_index(0, 0, 1), 16);
        assert_eq!(child_index(3, 3, 3), 63);
    }
}
