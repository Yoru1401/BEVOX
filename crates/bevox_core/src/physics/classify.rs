//! Which voxels of a volume can be touched first, and in what shape.
//!
//! After Dwyer's devlogs #20 and #26: a voxel is labelled by how many axes it
//! has solid neighbours on both sides. Only corners and edges ever need testing
//! against another volume. A face voxel is touched only where a corner of the
//! other volume reaches it, and an interior voxel never is.

use crate::contree::Contree;
use crate::material::MaterialId;
use glam::{IVec3, UVec3};

const AXES: [IVec3; 3] = [IVec3::X, IVec3::Y, IVec3::Z];

/// A solid voxel's collision shape: a cube rounded at its corners and edges,
/// full across its faces.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Shape {
    /// Solid on both sides along no axis: a sphere of radius 0.5.
    Corner,
    /// Along exactly the named axis: a cylinder of radius 0.5 along it.
    Edge(usize),
    /// Along two axes: flat across `axis`, on whichever sides are open.
    Face { axis: usize, open_plus: bool, open_minus: bool },
    /// Along all three: never touched first.
    Interior,
}

/// Whether `p` is a solid voxel of `tree`. Outside the volume is empty.
pub fn solid_at(tree: &Contree, p: IVec3) -> bool {
    let n = tree.extent() as i32;
    p.cmpge(IVec3::ZERO).all()
        && p.cmplt(IVec3::splat(n)).all()
        && !tree.get(p.as_uvec3()).is_empty()
}

/// The shape of the solid voxel at `p`, from its six neighbours.
pub fn classify(solid: impl Fn(IVec3) -> bool, p: IVec3) -> Shape {
    let enclosed = AXES.map(|e| solid(p + e) && solid(p - e));
    match enclosed.iter().filter(|&&b| b).count() {
        0 => Shape::Corner,
        1 => Shape::Edge(enclosed.iter().position(|&b| b).unwrap()),
        2 => {
            let axis = enclosed.iter().position(|&b| !b).unwrap();
            Shape::Face {
                axis,
                open_plus: !solid(p + AXES[axis]),
                open_minus: !solid(p - AXES[axis]),
            }
        }
        _ => Shape::Interior,
    }
}

/// A body's corner and edge voxels, the only ones tested against the world.
#[derive(Clone, Debug, Default)]
pub struct Features {
    pub corners: Vec<UVec3>,
    /// With the axis each edge runs along.
    pub edges: Vec<(UVec3, usize)>,
}

/// Classifies every one of `voxels`, which must be `tree`'s own.
pub fn features(tree: &Contree, voxels: &[(UVec3, MaterialId)]) -> Features {
    let mut out = Features::default();
    for &(p, _) in voxels {
        match classify(|q| solid_at(tree, q), p.as_ivec3()) {
            Shape::Corner => out.corners.push(p),
            Shape::Edge(axis) => out.edges.push((p, axis)),
            Shape::Face { .. } | Shape::Interior => {}
        }
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::physics::fixtures::cube;

    fn shapes_of(tree: &Contree) -> Vec<Shape> {
        tree.voxels()
            .iter()
            .map(|(p, _)| classify(|q| solid_at(tree, q), p.as_ivec3()))
            .collect()
    }

    /// Only corners and edges can be touched first, so only they are kept.
    #[test]
    fn a_cube_keeps_its_eight_corners_and_twelve_runs_of_edges() {
        for n in [2u32, 4, 5] {
            let tree = cube(n, 16);
            let f = features(&tree, &tree.voxels());
            assert_eq!(f.corners.len(), 8, "{n}-cube corners");
            assert_eq!(f.edges.len(), 12 * (n as usize - 2), "{n}-cube edges");
        }
    }

    /// Every voxel gets exactly one label, and the counts are the textbook ones.
    #[test]
    fn every_voxel_of_a_cube_is_labelled_once() {
        let n = 5usize;
        let shapes = shapes_of(&cube(n as u32, 16));
        let count = |f: fn(&Shape) -> bool| shapes.iter().filter(|s| f(s)).count();
        assert_eq!(count(|s| matches!(s, Shape::Corner)), 8);
        assert_eq!(count(|s| matches!(s, Shape::Edge(_))), 12 * (n - 2));
        assert_eq!(count(|s| matches!(s, Shape::Face { .. })), 6 * (n - 2) * (n - 2));
        assert_eq!(count(|s| matches!(s, Shape::Interior)), (n - 2).pow(3));
    }

    #[test]
    fn a_bar_has_corner_ends_and_an_edge_along_it() {
        let tree = Contree::from_voxels(
            4,
            &[
                (UVec3::new(0, 1, 1), MaterialId(1)),
                (UVec3::new(1, 1, 1), MaterialId(1)),
                (UVec3::new(2, 1, 1), MaterialId(1)),
            ],
        );
        let solid = |q| solid_at(&tree, q);
        assert_eq!(classify(solid, IVec3::new(0, 1, 1)), Shape::Corner);
        assert_eq!(classify(solid, IVec3::new(1, 1, 1)), Shape::Edge(0));
        assert_eq!(classify(solid, IVec3::new(2, 1, 1)), Shape::Corner);
    }

    /// A one-voxel plate is a face open on both sides, which is how a thin floor
    /// is seen from above and below.
    #[test]
    fn a_plate_is_open_on_both_sides() {
        let mut voxels = Vec::new();
        for z in 0..3 {
            for x in 0..3 {
                voxels.push((UVec3::new(x, 1, z), MaterialId(1)));
            }
        }
        let tree = Contree::from_voxels(4, &voxels);
        assert_eq!(
            classify(|q| solid_at(&tree, q), IVec3::new(1, 1, 1)),
            Shape::Face { axis: 1, open_plus: true, open_minus: true }
        );
    }

    /// Outside the volume is empty, so a voxel on the volume's boundary is
    /// exposed there.
    #[test]
    fn outside_the_volume_is_empty() {
        let tree = cube(4, 4);
        assert!(!solid_at(&tree, IVec3::new(-1, 0, 0)));
        assert!(!solid_at(&tree, IVec3::new(0, 4, 0)));
        assert!(solid_at(&tree, IVec3::new(3, 3, 3)));
    }
}
