//! Voxel editing. Edits rewrite only the subtrees they intersect, free the
//! nodes they replace, and leave the tree canonical.

use crate::contree::{Contree, level_extent};
use crate::material::MaterialId;
use crate::node::{BRICK_EDGE, CHILDREN, Node, child_index};
use glam::{UVec3, Vec3};

/// A sphere paint operation. `material` of `MaterialId::EMPTY` erases.
pub(crate) struct SphereOp {
    center: Vec3,
    radius: f32,
    material: MaterialId,
}

impl SphereOp {
    /// Whether the sphere touches the cube of `extent` voxels at `origin`.
    fn intersects(&self, origin: UVec3, extent: u32) -> bool {
        let lo = origin.as_vec3();
        let hi = lo + Vec3::splat(extent as f32);
        let closest = self.center.clamp(lo, hi);
        closest.distance_squared(self.center) <= self.radius * self.radius
    }

    /// Whether a single voxel's centre lies inside the sphere.
    fn covers_voxel(&self, p: UVec3) -> bool {
        let c = p.as_vec3() + Vec3::splat(0.5);
        c.distance_squared(self.center) <= self.radius * self.radius
    }
}

impl Contree {
    /// Paints a sphere. Passing `MaterialId::EMPTY` erases instead of filling.
    pub fn apply_sphere(&mut self, center: Vec3, radius: f32, material: MaterialId) {
        let op = SphereOp { center, radius, material };
        let root = self.root();
        let new_root = self.rewrite(root, self.depth() - 1, UVec3::ZERO, &op);
        self.set_root(new_root);
    }

    /// Returns the replacement for `node`, freeing whatever it replaces.
    fn rewrite(&mut self, node: Node, level: u32, origin: UVec3, op: &SphereOp) -> Node {
        let extent = level_extent(level);
        if !op.intersects(origin, extent) {
            return node;
        }

        if level == 0 {
            return self.rewrite_leaf(node, origin, op);
        }

        let step = level_extent(level - 1);
        let mut children = [Node::EMPTY; CHILDREN as usize];
        let mut mask = 0u64;
        for z in 0..BRICK_EDGE {
            for y in 0..BRICK_EDGE {
                for x in 0..BRICK_EDGE {
                    let i = child_index(x, y, z);
                    let child_origin = origin + UVec3::new(x, y, z) * step;
                    // An unsubdivided parent hands its own value down to each child.
                    let existing = match node.child_slot(i) {
                        Some(slot) => self.arena.node(slot),
                        None if node.is_uniform_solid() => node,
                        None => Node::EMPTY,
                    };
                    let child = self.rewrite(existing, level - 1, child_origin, op);
                    children[i as usize] = child;
                    if !child.is_empty() {
                        mask |= 1u64 << i;
                    }
                }
            }
        }

        if node.is_subdivided() {
            self.arena.free_nodes(node.child_base, node.child_count());
        }

        if mask == 0 {
            return Node::EMPTY;
        }
        if let Some(collapsed) = collapse_uniform(&children) {
            return collapsed;
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

    fn rewrite_leaf(&mut self, node: Node, origin: UVec3, op: &SphereOp) -> Node {
        let mut materials = [MaterialId::EMPTY; CHILDREN as usize];
        let mut mask = 0u64;
        for z in 0..BRICK_EDGE {
            for y in 0..BRICK_EDGE {
                for x in 0..BRICK_EDGE {
                    let i = child_index(x, y, z);
                    let p = origin + UVec3::new(x, y, z);
                    let existing = match node.child_slot(i) {
                        Some(slot) => MaterialId(self.arena.voxel(slot)),
                        None if node.is_uniform_solid() => node.material(),
                        None => MaterialId::EMPTY,
                    };
                    let m = if op.covers_voxel(p) { op.material } else { existing };
                    materials[i as usize] = m;
                    if !m.is_empty() {
                        mask |= 1u64 << i;
                    }
                }
            }
        }

        if node.is_subdivided() {
            self.arena.free_voxels(node.child_base, node.child_count());
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
}

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
