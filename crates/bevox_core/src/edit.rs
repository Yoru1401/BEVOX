//! Voxel editing. Edits rewrite only the subtrees they intersect, free the
//! nodes they replace, and leave the tree canonical.

use crate::contree::{Contree, level_extent};
use crate::material::MaterialId;
use crate::node::{BRICK_EDGE, CHILDREN, Node, child_index};
use glam::{UVec3, Vec3};
use std::collections::HashSet;

/// What an edit does to the voxels it covers.
///
/// One trait rather than a second copy of the tree rewrite: every edit has to
/// free the nodes it replaces, leave the tree canonical, and mark the arena
/// dirty, or the render world cannot upload a delta.
pub(crate) trait EditOp {
    /// Whether the edit touches the cube of `extent` voxels at `origin`. A node
    /// it does not touch is returned as it was.
    fn touches(&self, origin: UVec3, extent: u32) -> bool;

    /// What the voxel at `p` becomes. `None` leaves it as it is.
    fn material_at(&self, p: UVec3) -> Option<MaterialId>;
}

/// A sphere paint operation. `material` of `MaterialId::EMPTY` erases.
pub(crate) struct SphereOp {
    center: Vec3,
    radius: f32,
    material: MaterialId,
}

impl EditOp for SphereOp {
    fn touches(&self, origin: UVec3, extent: u32) -> bool {
        self.intersects(origin, extent)
    }

    fn material_at(&self, p: UVec3) -> Option<MaterialId> {
        self.covers_voxel(p).then_some(self.material)
    }
}

/// Erases exactly the voxels it was given: what detachment takes out of the
/// world when a piece becomes a body.
struct ClearOp {
    voxels: HashSet<UVec3>,
    lo: UVec3,
    hi: UVec3,
}

impl EditOp for ClearOp {
    fn touches(&self, origin: UVec3, extent: u32) -> bool {
        let hi = origin + UVec3::splat(extent - 1);
        origin.cmple(self.hi).all() && hi.cmpge(self.lo).all()
    }

    fn material_at(&self, p: UVec3) -> Option<MaterialId> {
        self.voxels.contains(&p).then_some(MaterialId::EMPTY)
    }
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

    /// Erases exactly `voxels`. Detachment moves a piece of the world into a
    /// body, and takes the same voxels out of the world with this.
    pub fn clear_voxels(&mut self, voxels: &[UVec3]) {
        let Some(&first) = voxels.first() else {
            return;
        };
        let mut lo = first;
        let mut hi = first;
        for p in voxels {
            lo = lo.min(*p);
            hi = hi.max(*p);
        }
        let op = ClearOp { voxels: voxels.iter().copied().collect(), lo, hi };
        let root = self.root();
        let new_root = self.rewrite(root, self.depth() - 1, UVec3::ZERO, &op);
        self.set_root(new_root);
    }

    /// Returns the replacement for `node`, freeing whatever it replaces.
    fn rewrite(&mut self, node: Node, level: u32, origin: UVec3, op: &dyn EditOp) -> Node {
        let extent = level_extent(level);
        if !op.touches(origin, extent) {
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

    fn rewrite_leaf(&mut self, node: Node, origin: UVec3, op: &dyn EditOp) -> Node {
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
                    let m = op.material_at(p).unwrap_or(existing);
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
