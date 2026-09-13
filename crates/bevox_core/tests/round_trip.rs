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

#[test]
fn construction_is_deterministic() {
    let dense = mixed_volume(16, 99);
    let a = Contree::from_dense(&dense);
    let b = Contree::from_dense(&dense);
    assert_eq!(a.root(), b.root());
    assert_eq!(a.arena().nodes(), b.arena().nodes());
    assert_eq!(a.arena().voxels(), b.arena().voxels());
}
