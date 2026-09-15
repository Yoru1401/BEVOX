//! Arena storage for contree nodes and leaf voxel bytes.
//!
//! Reclamation is a free list per size class, which is the deliberately simple
//! version of a page allocator. Replace it only if measurement shows
//! fragmentation actually costs something.

use crate::node::{CHILDREN, Node};
use core::ops::Range;

#[derive(Clone, Debug)]
pub struct NodeArena {
    nodes: Vec<Node>,
    voxels: Vec<u8>,
    node_free: Vec<Vec<u32>>,
    voxel_free: Vec<Vec<u32>>,
    dirty_nodes: Vec<Range<u32>>,
    dirty_voxels: Vec<Range<u32>>,
}

/// Merges overlapping and adjacent ranges, leaving disjoint ones separate.
fn normalize(ranges: &[Range<u32>]) -> Vec<Range<u32>> {
    let mut sorted: Vec<Range<u32>> = ranges.to_vec();
    sorted.sort_by_key(|r| r.start);
    let mut out: Vec<Range<u32>> = Vec::with_capacity(sorted.len());
    for r in sorted {
        match out.last_mut() {
            Some(last) if r.start <= last.end => last.end = last.end.max(r.end),
            _ => out.push(r),
        }
    }
    out
}

impl NodeArena {
    pub fn new() -> Self {
        Self {
            nodes: Vec::new(),
            voxels: Vec::new(),
            // Index by slot count; 0 is unused so that `free[count]` reads directly.
            node_free: vec![Vec::new(); CHILDREN as usize + 1],
            voxel_free: vec![Vec::new(); CHILDREN as usize + 1],
            dirty_nodes: Vec::new(),
            dirty_voxels: Vec::new(),
        }
    }

    pub fn alloc_nodes(&mut self, count: u32) -> u32 {
        debug_assert!(count >= 1 && count <= CHILDREN);
        let start = match self.node_free[count as usize].pop() {
            Some(start) => start,
            None => {
                let start = self.nodes.len() as u32;
                self.nodes.resize(self.nodes.len() + count as usize, Node::EMPTY);
                start
            }
        };
        self.dirty_nodes.push(start..start + count);
        start
    }

    pub fn free_nodes(&mut self, start: u32, count: u32) {
        debug_assert!(count >= 1 && count <= CHILDREN);
        self.node_free[count as usize].push(start);
    }

    pub fn alloc_voxels(&mut self, count: u32) -> u32 {
        debug_assert!(count >= 1 && count <= CHILDREN);
        let start = match self.voxel_free[count as usize].pop() {
            Some(start) => start,
            None => {
                let start = self.voxels.len() as u32;
                self.voxels.resize(self.voxels.len() + count as usize, 0);
                start
            }
        };
        self.dirty_voxels.push(start..start + count);
        start
    }

    pub fn free_voxels(&mut self, start: u32, count: u32) {
        debug_assert!(count >= 1 && count <= CHILDREN);
        self.voxel_free[count as usize].push(start);
    }

    pub fn node(&self, slot: u32) -> Node {
        self.nodes[slot as usize]
    }

    pub fn set_node(&mut self, slot: u32, node: Node) {
        self.nodes[slot as usize] = node;
        self.dirty_nodes.push(slot..slot + 1);
    }

    pub fn voxel(&self, slot: u32) -> u8 {
        self.voxels[slot as usize]
    }

    pub fn set_voxel(&mut self, slot: u32, value: u8) {
        self.voxels[slot as usize] = value;
        self.dirty_voxels.push(slot..slot + 1);
    }

    pub fn nodes(&self) -> &[Node] {
        &self.nodes
    }

    pub fn voxels(&self) -> &[u8] {
        &self.voxels
    }

    /// Ranges touched since the last `clear_dirty`, merged where contiguous.
    pub fn dirty_nodes(&self) -> Vec<Range<u32>> {
        normalize(&self.dirty_nodes)
    }

    pub fn dirty_voxels(&self) -> Vec<Range<u32>> {
        normalize(&self.dirty_voxels)
    }

    pub fn clear_dirty(&mut self) {
        self.dirty_nodes.clear();
        self.dirty_voxels.clear();
    }
}

impl Default for NodeArena {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::material::MaterialId;

    #[test]
    fn allocations_are_sequential_when_nothing_is_free() {
        let mut arena = NodeArena::new();
        assert_eq!(arena.alloc_nodes(3), 0);
        assert_eq!(arena.alloc_nodes(2), 3);
        assert_eq!(arena.nodes().len(), 5);
    }

    #[test]
    fn freed_blocks_are_reused_for_the_same_size_class() {
        let mut arena = NodeArena::new();
        let a = arena.alloc_nodes(4);
        let _b = arena.alloc_nodes(4);
        arena.free_nodes(a, 4);
        assert_eq!(arena.alloc_nodes(4), a);
    }

    #[test]
    fn a_freed_block_is_not_reused_for_a_different_size_class() {
        let mut arena = NodeArena::new();
        let a = arena.alloc_nodes(4);
        arena.free_nodes(a, 4);
        let b = arena.alloc_nodes(2);
        assert_ne!(b, a);
    }

    #[test]
    fn allocation_marks_exactly_the_allocated_range_dirty() {
        let mut arena = NodeArena::new();
        arena.clear_dirty();
        let start = arena.alloc_nodes(3);
        assert_eq!(arena.dirty_nodes(), vec![start..start + 3]);
    }

    #[test]
    fn writes_mark_their_slot_dirty() {
        let mut arena = NodeArena::new();
        let start = arena.alloc_nodes(2);
        arena.clear_dirty();
        arena.set_node(start + 1, Node::uniform(MaterialId(3)));
        assert_eq!(arena.dirty_nodes(), vec![start + 1..start + 2]);
        assert_eq!(arena.node(start + 1).material(), MaterialId(3));
    }

    #[test]
    fn adjacent_dirty_ranges_merge_but_disjoint_ones_do_not() {
        let mut arena = NodeArena::new();
        arena.alloc_nodes(8);
        arena.clear_dirty();
        arena.set_node(0, Node::uniform(MaterialId(1)));
        arena.set_node(1, Node::uniform(MaterialId(1)));
        arena.set_node(5, Node::uniform(MaterialId(1)));
        assert_eq!(arena.dirty_nodes(), vec![0..2, 5..6]);
    }

    #[test]
    fn voxel_allocation_is_tracked_separately_from_nodes() {
        let mut arena = NodeArena::new();
        let v = arena.alloc_voxels(9);
        arena.clear_dirty();
        arena.set_voxel(v + 2, 42);
        assert_eq!(arena.voxel(v + 2), 42);
        assert_eq!(arena.dirty_voxels(), vec![v + 2..v + 3]);
        assert!(arena.dirty_nodes().is_empty());
    }

    #[test]
    fn size_classes_cover_a_full_node_of_children() {
        let mut arena = NodeArena::new();
        let a = arena.alloc_nodes(CHILDREN);
        arena.free_nodes(a, CHILDREN);
        assert_eq!(arena.alloc_nodes(CHILDREN), a);
    }
}
