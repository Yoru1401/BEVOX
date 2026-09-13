//! Per-voxel normals, generated from neighbour occupancy rather than stored.
//!
//! Storing normals in voxel data makes every fill, copy and delete responsible
//! for fixing up its neighbours, which is why the spec forbids it.

use crate::contree::Contree;
use crate::material::MaterialId;
use glam::{IVec3, UVec3, Vec3};

const NEIGHBOURS: [IVec3; 6] = [
    IVec3::new(1, 0, 0),
    IVec3::new(-1, 0, 0),
    IVec3::new(0, 1, 0),
    IVec3::new(0, -1, 0),
    IVec3::new(0, 0, 1),
    IVec3::new(0, 0, -1),
];

/// Approximates a surface normal by summing the directions in which a voxel is
/// exposed. Falls back to `face_normal` when the result carries no information,
/// which happens for isolated and fully buried voxels.
pub fn implicit_normal(volume: &Contree, voxel: UVec3, face_normal: Vec3) -> Vec3 {
    let extent = volume.extent() as i32;
    let base = voxel.as_ivec3();
    let mut sum = Vec3::ZERO;

    for d in NEIGHBOURS {
        let n = base + d;
        let outside =
            n.x < 0 || n.y < 0 || n.z < 0 || n.x >= extent || n.y >= extent || n.z >= extent;
        let empty = outside || volume.get(n.as_uvec3()) == MaterialId::EMPTY;
        if empty {
            sum += d.as_vec3();
        }
    }

    if sum.length_squared() < 1e-6 {
        return face_normal;
    }
    sum.normalize()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::dense::DenseVolume;

    fn tree_from(points: &[(UVec3, u8)]) -> Contree {
        let mut dense = DenseVolume::new(16).unwrap();
        for (p, m) in points {
            dense.set(*p, MaterialId(*m));
        }
        Contree::from_dense(&dense)
    }

    #[test]
    fn an_isolated_voxel_falls_back_to_the_face_normal() {
        let tree = tree_from(&[(UVec3::new(8, 8, 8), 1)]);
        let n = implicit_normal(&tree, UVec3::new(8, 8, 8), Vec3::NEG_X);
        assert_eq!(n, Vec3::NEG_X);
    }

    #[test]
    fn a_one_voxel_thick_slab_falls_back_to_the_face_normal() {
        let mut points = Vec::new();
        for z in 6..11 {
            for x in 6..11 {
                points.push((UVec3::new(x, 8, z), 1u8));
            }
        }
        let tree = tree_from(&points);
        let n = implicit_normal(&tree, UVec3::new(8, 8, 8), Vec3::Y);
        // Up and down are both exposed on a one-voxel-thick slab, and the four
        // lateral neighbours are solid, so the normal resolves to the face.
        assert!(n.length() > 0.99 && n.length() < 1.01);
    }

    #[test]
    fn a_voxel_in_an_inner_corner_points_diagonally_outward() {
        // Two perpendicular walls meeting, leaving +x and +y exposed.
        let mut points = Vec::new();
        for i in 0..5u32 {
            points.push((UVec3::new(8, 8 - i, 8), 1u8));
            points.push((UVec3::new(8 - i, 8, 8), 1u8));
        }
        points.push((UVec3::new(8, 8, 7), 1));
        points.push((UVec3::new(8, 8, 9), 1));
        let tree = tree_from(&points);

        let n = implicit_normal(&tree, UVec3::new(8, 8, 8), Vec3::Y);
        assert!(n.x > 0.0 && n.y > 0.0, "normal was {n:?}");
        assert!((n.length() - 1.0).abs() < 1e-3);
    }

    #[test]
    fn a_fully_buried_voxel_falls_back_to_the_face_normal() {
        let mut points = vec![(UVec3::new(8, 8, 8), 1u8)];
        for d in NEIGHBOURS {
            let p = IVec3::new(8, 8, 8) + d;
            points.push((p.as_uvec3(), 1));
        }
        let tree = tree_from(&points);
        let n = implicit_normal(&tree, UVec3::new(8, 8, 8), Vec3::Z);
        assert_eq!(n, Vec3::Z);
    }
}
