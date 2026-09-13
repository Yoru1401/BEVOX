use bevox_core::contree::Contree;
use bevox_core::dense::DenseVolume;
use bevox_core::material::MaterialId;
use bevox_core::testing::XorShift64;
use glam::UVec3;

/// Builds a volume with large uniform blocks plus scattered noise, which
/// exercises both the collapse path and the subdivided path.
fn mixed_volume(extent: u32, seed: u64) -> DenseVolume {
    let mut rng = XorShift64::new(seed);
    let mut dense = DenseVolume::new(extent).unwrap();
    // A solid slab across the bottom quarter.
    for z in 0..extent {
        for y in 0..extent / 4 {
            for x in 0..extent {
                dense.set(UVec3::new(x, y, z), MaterialId(1));
            }
        }
    }
    // Scattered voxels above it.
    for _ in 0..extent * extent {
        let p = UVec3::new(
            rng.next_below(extent),
            extent / 4 + rng.next_below(extent - extent / 4),
            rng.next_below(extent),
        );
        dense.set(p, MaterialId(2 + rng.next_below(4) as u8));
    }
    dense
}

#[test]
fn mixed_volumes_round_trip_at_several_depths() {
    for (extent, seed) in [(4u32, 1u64), (16, 2), (64, 3)] {
        let dense = mixed_volume(extent, seed);
        let tree = Contree::from_dense(&dense);
        assert_eq!(tree.to_dense(), dense, "extent {extent} failed to round trip");
    }
}

/// Collects the solid voxels of a dense volume, in the order the dense builder
/// would meet them.
fn solid_voxels(dense: &DenseVolume) -> Vec<(UVec3, MaterialId)> {
    let extent = dense.extent();
    let mut out = Vec::new();
    for z in 0..extent {
        for y in 0..extent {
            for x in 0..extent {
                let p = UVec3::new(x, y, z);
                let m = dense.get(p);
                if !m.is_empty() {
                    out.push((p, m));
                }
            }
        }
    }
    out
}

/// The sparse builder must produce exactly what the dense one does. This is the
/// correctness proof for the whole task: the dense path is already trusted, so
/// equivalence transfers that trust rather than asking for new faith.
#[test]
fn the_sparse_builder_matches_the_dense_builder() {
    for (extent, seed) in [(4u32, 11u64), (16, 12), (64, 13)] {
        let dense = mixed_volume(extent, seed);
        let voxels = solid_voxels(&dense);

        let from_dense = Contree::from_dense(&dense);
        let from_voxels = Contree::from_voxels(extent, &voxels);

        assert_eq!(from_voxels.root(), from_dense.root(), "extent {extent} root");
        assert_eq!(
            from_voxels.arena().nodes(),
            from_dense.arena().nodes(),
            "extent {extent} nodes"
        );
        assert_eq!(
            from_voxels.arena().voxels(),
            from_dense.arena().voxels(),
            "extent {extent} voxels"
        );
        assert_eq!(from_voxels.check_canonical(), Ok(()));
    }
}

#[test]
fn an_empty_voxel_list_builds_an_empty_tree() {
    let tree = Contree::from_voxels(64, &[]);
    assert!(tree.root().is_empty());
    assert_eq!(tree.arena().nodes().len(), 0);
}

#[test]
fn later_voxels_win_when_a_coordinate_repeats() {
    let p = UVec3::new(1, 2, 3);
    let tree = Contree::from_voxels(16, &[(p, MaterialId(4)), (p, MaterialId(9))]);
    assert_eq!(tree.get(p), MaterialId(9));
}

#[test]
fn a_fully_solid_volume_collapses_from_the_sparse_path_too() {
    let extent = 16u32;
    let mut voxels = Vec::new();
    for z in 0..extent {
        for y in 0..extent {
            for x in 0..extent {
                voxels.push((UVec3::new(x, y, z), MaterialId(3)));
            }
        }
    }
    let tree = Contree::from_voxels(extent, &voxels);
    assert!(tree.root().is_uniform_solid(), "a solid volume must collapse");
    assert_eq!(tree.arena().nodes().len(), 0);
    assert_eq!(tree.root().material(), MaterialId(3));
}

#[test]
fn the_sparse_builder_is_deterministic() {
    let dense = mixed_volume(16, 77);
    let voxels = solid_voxels(&dense);
    let a = Contree::from_voxels(16, &voxels);
    let b = Contree::from_voxels(16, &voxels);
    assert_eq!(a.arena().nodes(), b.arena().nodes());
    assert_eq!(a.root(), b.root());
}

#[test]
fn construction_is_deterministic() {
    let dense = mixed_volume(16, 99);
    let a = Contree::from_dense(&dense);
    let b = Contree::from_dense(&dense);
    assert_eq!(a.root(), b.root());
    assert_eq!(a.arena().nodes(), b.arena().nodes());
    assert_eq!(a.arena().voxels(), b.arena().voxels());
}
