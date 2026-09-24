//! Merging: a body that came out of the terrain goes back into it and frees
//! its slot, as in Dwyer's devlog #13. The app decides when; this is what it
//! does.

use bevox_core::body::Body;
use bevox_core::contree::Contree;
use glam::{IVec3, UVec3, Vec3};
use std::collections::HashSet;

/// Writes `body`'s voxels into `tree`, each at the world voxel under its
/// centre, and returns the cells written.
///
/// Only where the world is empty: a merge adds and never overwrites. Only
/// inside the world: a voxel past its edge is dropped. A body at an angle is
/// not on the world's grid, so its voxels snap, and its shape can change by up
/// to a voxel; that is why the app merges only bodies it cannot see. Two voxels
/// that snap to one cell keep the first.
pub fn merge(body: &Body, tree: &mut Contree) -> Vec<UVec3> {
    let to_world = body.world_from_local();
    let extent = IVec3::splat(tree.extent() as i32);
    let mut taken = HashSet::new();
    let mut cells = Vec::new();
    for (p, material) in body.volume.voxels() {
        let at = to_world.transform_point3(p.as_vec3() + Vec3::splat(0.5)).floor().as_ivec3();
        if at.cmplt(IVec3::ZERO).any() || at.cmpge(extent).any() {
            continue;
        }
        let at = at.as_uvec3();
        if tree.get(at).is_empty() && taken.insert(at) {
            cells.push((at, material));
        }
    }
    tree.fill_voxels(&cells);
    cells.into_iter().map(|(p, _)| p).collect()
}

#[cfg(test)]
mod tests {
    use super::*;
    use bevox_core::material::MaterialId;
    use crate::fixtures::cube_of;
    use glam::{EulerRot, Quat};

    /// An unturned body on whole coordinates merges voxel for voxel, each with
    /// its own material.
    #[test]
    fn an_aligned_body_merges_voxel_for_voxel() {
        let mut world = Contree::empty(3);
        let body = Body::new(cube_of(4, 4, MaterialId(2)), Vec3::new(10.0, 20.0, 10.0), Quat::IDENTITY);
        let written = merge(&body, &mut world);
        assert_eq!(written.len(), 64);
        for z in 10..14 {
            for y in 20..24 {
                for x in 10..14 {
                    assert_eq!(world.get(UVec3::new(x, y, z)), MaterialId(2), "({x}, {y}, {z})");
                }
            }
        }
        assert_eq!(world.voxels().len(), 64, "voxels landed outside the body");
    }

    /// A turned body snaps: the world voxel under every one of its voxel
    /// centres ends up solid.
    #[test]
    fn a_turned_body_lands_under_its_voxel_centres() {
        let mut world = Contree::empty(3);
        let turn = Quat::from_euler(EulerRot::XYZ, 0.4, 0.7, -0.2);
        let body = Body::new(cube_of(4, 4, MaterialId(2)), Vec3::new(30.0, 30.0, 30.0), turn);
        let written = merge(&body, &mut world);
        let to_world = body.world_from_local();
        for (p, _) in body.volume.voxels() {
            let under = to_world.transform_point3(p.as_vec3() + Vec3::splat(0.5)).floor().as_uvec3();
            assert!(!world.get(under).is_empty(), "nothing landed under body voxel {p:?}");
        }
        assert!(written.len() > 48, "only {} of 64 voxels found a cell of their own", written.len());
    }

    /// A merge adds and never overwrites, and drops what lies past the world.
    #[test]
    fn merging_keeps_the_world_and_stays_inside_it() {
        let mut world = Contree::from_voxels(64, &[(UVec3::new(11, 21, 11), MaterialId(1))]);
        let body = Body::new(cube_of(4, 4, MaterialId(2)), Vec3::new(10.0, 20.0, 10.0), Quat::IDENTITY);
        let written = merge(&body, &mut world);
        assert_eq!(world.get(UVec3::new(11, 21, 11)), MaterialId(1), "the merge overwrote the world");
        assert_eq!(written.len(), 63);

        // Half past the world's far edge at x = 64.
        let mut world = Contree::empty(3);
        let body = Body::new(cube_of(4, 4, MaterialId(2)), Vec3::new(62.0, 20.0, 10.0), Quat::IDENTITY);
        let written = merge(&body, &mut world);
        assert_eq!(written.len(), 32, "voxels past the edge were not dropped");
        assert!(written.iter().all(|p| p.x < 64));
    }
}
