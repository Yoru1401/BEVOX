//! The sparse 64-tree. Built bottom-up so that homogeneous regions collapse
//! during construction rather than in a later pass.

use crate::arena::NodeArena;
use crate::dense::DenseVolume;
use crate::material::MaterialId;
use crate::node::{BRICK_EDGE, CHILDREN, Node, child_index};
use glam::UVec3;

pub struct Contree {
    pub(crate) arena: NodeArena,
    pub(crate) root: Node,
    pub(crate) depth: u32,
}

/// Voxels per axis covered by a node at `level`, where level 0 is a leaf brick.
pub fn level_extent(level: u32) -> u32 {
    BRICK_EDGE.pow(level + 1)
}

impl Contree {
    pub fn empty(depth: u32) -> Self {
        debug_assert!(depth >= 1);
        Self { arena: NodeArena::new(), root: Node::EMPTY, depth }
    }

    pub fn depth(&self) -> u32 {
        self.depth
    }

    pub fn extent(&self) -> u32 {
        level_extent(self.depth - 1)
    }

    pub fn root(&self) -> Node {
        self.root
    }

    pub fn arena(&self) -> &NodeArena {
        &self.arena
    }

    pub fn arena_mut(&mut self) -> &mut NodeArena {
        &mut self.arena
    }

    pub fn from_dense(dense: &DenseVolume) -> Self {
        let depth = dense.extent().trailing_zeros() / 2;
        debug_assert!(depth >= 1, "a volume must be at least 4 voxels per axis");
        let mut tree = Self::empty(depth);
        tree.root = tree.build(dense, depth - 1, UVec3::ZERO);
        tree
    }

    /// Builds the node covering `level_extent(level)` voxels from `origin`.
    fn build(&mut self, dense: &DenseVolume, level: u32, origin: UVec3) -> Node {
        if level == 0 {
            return self.build_leaf(dense, origin);
        }

        let step = level_extent(level - 1);
        let mut children = [Node::EMPTY; CHILDREN as usize];
        let mut mask = 0u64;
        for z in 0..BRICK_EDGE {
            for y in 0..BRICK_EDGE {
                for x in 0..BRICK_EDGE {
                    let child_origin = origin + UVec3::new(x, y, z) * step;
                    let child = self.build(dense, level - 1, child_origin);
                    let i = child_index(x, y, z);
                    children[i as usize] = child;
                    if !child.is_empty() {
                        mask |= 1u64 << i;
                    }
                }
            }
        }

        if mask == 0 {
            return Node::EMPTY;
        }
        if let Some(node) = collapse_uniform(&children) {
            return node;
        }

        let count = mask.count_ones();
        let base = self.arena.alloc_nodes(count);
        let mut slot = base;
        for child in children.iter() {
            if !child.is_empty() {
                self.arena.set_node(slot, *child);
                slot += 1;
            }
        }
        Node::subdivided(mask, base)
    }

    /// Builds a leaf brick, whose children are individual voxels.
    fn build_leaf(&mut self, dense: &DenseVolume, origin: UVec3) -> Node {
        let mut materials = [MaterialId::EMPTY; CHILDREN as usize];
        let mut mask = 0u64;
        for z in 0..BRICK_EDGE {
            for y in 0..BRICK_EDGE {
                for x in 0..BRICK_EDGE {
                    let m = dense.get(origin + UVec3::new(x, y, z));
                    let i = child_index(x, y, z);
                    materials[i as usize] = m;
                    if !m.is_empty() {
                        mask |= 1u64 << i;
                    }
                }
            }
        }

        if mask == 0 {
            return Node::EMPTY;
        }
        if mask == u64::MAX {
            let first = materials[0];
            if materials.iter().all(|m| *m == first) {
                return Node::uniform(first);
            }
        }

        let count = mask.count_ones();
        let base = self.arena.alloc_voxels(count);
        let mut slot = base;
        for m in materials.iter() {
            if !m.is_empty() {
                self.arena.set_voxel(slot, m.0);
                slot += 1;
            }
        }
        Node::subdivided(mask, base)
    }

    pub fn get(&self, p: UVec3) -> MaterialId {
        debug_assert!(p.x < self.extent() && p.y < self.extent() && p.z < self.extent());
        let mut node = self.root;
        let mut level = self.depth - 1;
        let mut local = p;

        loop {
            if !node.is_subdivided() {
                return node.material();
            }

            if level == 0 {
                let i = child_index(local.x, local.y, local.z);
                return match node.child_slot(i) {
                    Some(slot) => MaterialId(self.arena.voxel(slot)),
                    None => MaterialId::EMPTY,
                };
            }

            let step = level_extent(level - 1);
            let cell = local / step;
            let i = child_index(cell.x, cell.y, cell.z);
            match node.child_slot(i) {
                Some(slot) => {
                    node = self.arena.node(slot);
                    local -= cell * step;
                    level -= 1;
                }
                None => return MaterialId::EMPTY,
            }
        }
    }

    pub fn to_dense(&self) -> DenseVolume {
        let extent = self.extent();
        let mut dense = DenseVolume::new(extent)
            .expect("a tree's extent is a power of four by construction");
        for z in 0..extent {
            for y in 0..extent {
                for x in 0..extent {
                    let p = UVec3::new(x, y, z);
                    dense.set(p, self.get(p));
                }
            }
        }
        dense
    }
}

/// Returns a collapsed node when every child is the same uniform solid.
fn collapse_uniform(children: &[Node; CHILDREN as usize]) -> Option<Node> {
    let first = children[0];
    if !first.is_uniform_solid() {
        return None;
    }
    if children.iter().all(|c| *c == first) {
        Some(first)
    } else {
        None
    }
}

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum CanonicalError {
    /// A subdivided node whose children are all empty; it should be `Node::EMPTY`.
    UncollapsedEmpty { level: u32 },
    /// A subdivided node whose children are all the same uniform solid.
    UncollapsedUniform { level: u32 },
    /// An empty node occupying a child slot, which wastes space and breaks
    /// the invariant that a set mask bit implies a non-empty child.
    EmptyChildStored { level: u32, slot: u32 },
    /// A child slot outside the arena.
    SlotOutOfBounds { slot: u32 },
}

impl Contree {
    /// Replaces the root. Used by the edit path.
    pub(crate) fn set_root(&mut self, root: Node) {
        self.root = root;
    }

    /// Root replacement for tests that build deliberately invalid trees.
    /// Integration tests are separate crates, so this has to be public.
    #[doc(hidden)]
    pub fn set_root_for_test(&mut self, root: Node) {
        self.root = root;
    }

    pub fn check_canonical(&self) -> Result<(), CanonicalError> {
        self.check_node(self.root, self.depth - 1)
    }

    fn check_node(&self, node: Node, level: u32) -> Result<(), CanonicalError> {
        if !node.is_subdivided() {
            return Ok(());
        }

        // Leaf bricks address voxel bytes, which cannot be empty by construction
        // because empty voxels clear their mask bit. Only bounds need checking.
        if level == 0 {
            let last = node.child_base + node.child_count() - 1;
            if last as usize >= self.arena.voxels().len() {
                return Err(CanonicalError::SlotOutOfBounds { slot: last });
            }
            for i in 0..CHILDREN {
                if let Some(slot) = node.child_slot(i)
                    && self.arena.voxel(slot) == 0
                {
                    return Err(CanonicalError::EmptyChildStored { level, slot });
                }
            }
            return Ok(());
        }

        let last = node.child_base + node.child_count() - 1;
        if last as usize >= self.arena.nodes().len() {
            return Err(CanonicalError::SlotOutOfBounds { slot: last });
        }

        let mut all_same_uniform = node.child_count() == CHILDREN;
        let mut first = Node::EMPTY;
        for i in 0..CHILDREN {
            let Some(slot) = node.child_slot(i) else {
                all_same_uniform = false;
                continue;
            };
            let child = self.arena.node(slot);
            if child.is_empty() {
                return Err(CanonicalError::EmptyChildStored { level, slot });
            }
            if i == 0 {
                first = child;
            }
            if !child.is_uniform_solid() || child != first {
                all_same_uniform = false;
            }
            self.check_node(child, level - 1)?;
        }

        if all_same_uniform {
            return Err(CanonicalError::UncollapsedUniform { level });
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::testing::XorShift64;

    #[test]
    fn level_extents_are_powers_of_four() {
        assert_eq!(level_extent(0), 4);
        assert_eq!(level_extent(1), 16);
        assert_eq!(level_extent(4), 1024);
    }

    #[test]
    fn an_empty_volume_collapses_to_an_empty_root() {
        let tree = Contree::from_dense(&DenseVolume::new(64).unwrap());
        assert!(tree.root().is_empty());
        assert_eq!(tree.arena().nodes().len(), 0);
        assert_eq!(tree.arena().voxels().len(), 0);
    }

    #[test]
    fn a_completely_full_volume_collapses_to_a_uniform_root() {
        let mut dense = DenseVolume::new(64).unwrap();
        for z in 0..64 {
            for y in 0..64 {
                for x in 0..64 {
                    dense.set(UVec3::new(x, y, z), MaterialId(5));
                }
            }
        }
        let tree = Contree::from_dense(&dense);
        assert!(tree.root().is_uniform_solid());
        assert_eq!(tree.root().material(), MaterialId(5));
        assert_eq!(tree.arena().nodes().len(), 0);
    }

    #[test]
    fn a_single_voxel_is_readable() {
        let mut dense = DenseVolume::new(16).unwrap();
        dense.set(UVec3::new(7, 2, 11), MaterialId(3));
        let tree = Contree::from_dense(&dense);
        assert_eq!(tree.get(UVec3::new(7, 2, 11)), MaterialId(3));
        assert_eq!(tree.get(UVec3::new(7, 2, 10)), MaterialId::EMPTY);
        assert_eq!(tree.depth(), 2);
        assert_eq!(tree.extent(), 16);
    }

    #[test]
    fn every_voxel_of_a_random_volume_round_trips() {
        let mut rng = XorShift64::new(0xDEAD_BEEF);
        let mut dense = DenseVolume::new(16).unwrap();
        for z in 0..16 {
            for y in 0..16 {
                for x in 0..16 {
                    // Sparse-ish: roughly one voxel in four is solid.
                    if rng.next_below(4) == 0 {
                        let m = MaterialId(1 + rng.next_below(8) as u8);
                        dense.set(UVec3::new(x, y, z), m);
                    }
                }
            }
        }
        let tree = Contree::from_dense(&dense);
        assert_eq!(tree.to_dense(), dense);
    }

    #[test]
    fn empty_tree_reads_as_empty_everywhere() {
        let tree = Contree::empty(2);
        assert_eq!(tree.extent(), 16);
        assert_eq!(tree.get(UVec3::new(15, 15, 15)), MaterialId::EMPTY);
    }
}
