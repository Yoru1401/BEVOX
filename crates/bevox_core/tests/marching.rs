use bevox_core::contree::Contree;
use bevox_core::dense::DenseVolume;
use bevox_core::march::{MarchStats, march};
use bevox_core::material::MaterialId;
use glam::{Affine3A, UVec3, Vec3};

fn single_voxel_tree() -> Contree {
    let mut dense = DenseVolume::new(16).unwrap();
    dense.set(UVec3::new(8, 8, 8), MaterialId(3));
    Contree::from_dense(&dense)
}

#[test]
fn a_ray_aimed_at_a_lone_voxel_hits_it() {
    let tree = single_voxel_tree();
    let mut stats = MarchStats::default();
    let hit = march(
        &tree,
        Affine3A::IDENTITY,
        Vec3::new(-5.0, 8.5, 8.5),
        Vec3::X,
        100.0,
        false,
        &mut stats,
    )
    .expect("expected a hit");

    assert_eq!(hit.voxel, UVec3::new(8, 8, 8));
    assert_eq!(hit.material, MaterialId(3));
    assert!((hit.t - 13.0).abs() < 1e-3, "t was {}", hit.t);
    assert_eq!(hit.face_normal, Vec3::NEG_X);
    assert_eq!(stats.overruns, 0);
}

#[test]
fn a_ray_that_misses_returns_nothing() {
    let tree = single_voxel_tree();
    let mut stats = MarchStats::default();
    let hit = march(
        &tree,
        Affine3A::IDENTITY,
        Vec3::new(-5.0, 0.5, 0.5),
        Vec3::X,
        100.0,
        false,
        &mut stats,
    );
    assert!(hit.is_none());
}

#[test]
fn an_empty_tree_is_never_hit() {
    let tree = Contree::empty(2);
    let mut stats = MarchStats::default();
    assert!(
        march(
            &tree,
            Affine3A::IDENTITY,
            Vec3::new(-5.0, 8.5, 8.5),
            Vec3::X,
            100.0,
            false,
            &mut stats
        )
        .is_none()
    );
}

#[test]
fn the_nearer_of_two_voxels_is_returned() {
    let mut dense = DenseVolume::new(16).unwrap();
    dense.set(UVec3::new(4, 8, 8), MaterialId(1));
    dense.set(UVec3::new(12, 8, 8), MaterialId(2));
    let tree = Contree::from_dense(&dense);

    let mut stats = MarchStats::default();
    let hit = march(
        &tree,
        Affine3A::IDENTITY,
        Vec3::new(-5.0, 8.5, 8.5),
        Vec3::X,
        100.0,
        false,
        &mut stats,
    )
    .unwrap();
    assert_eq!(hit.voxel, UVec3::new(4, 8, 8));

    // From the far side, the other voxel is nearer.
    let hit = march(
        &tree,
        Affine3A::IDENTITY,
        Vec3::new(25.0, 8.5, 8.5),
        Vec3::NEG_X,
        100.0,
        false,
        &mut stats,
    )
    .unwrap();
    assert_eq!(hit.voxel, UVec3::new(12, 8, 8));
    assert_eq!(hit.face_normal, Vec3::X);
}

#[test]
fn max_dist_stops_the_ray_short() {
    let tree = single_voxel_tree();
    let mut stats = MarchStats::default();
    let hit = march(
        &tree,
        Affine3A::IDENTITY,
        Vec3::new(-5.0, 8.5, 8.5),
        Vec3::X,
        10.0, // the voxel is 13 units away
        false,
        &mut stats,
    );
    assert!(hit.is_none());
}

#[test]
fn any_hit_finds_something_without_visiting_more_than_closest_hit() {
    let mut dense = DenseVolume::new(16).unwrap();
    for x in 0..16 {
        dense.set(UVec3::new(x, 8, 8), MaterialId(1));
    }
    let tree = Contree::from_dense(&dense);

    let mut closest_stats = MarchStats::default();
    let closest = march(
        &tree,
        Affine3A::IDENTITY,
        Vec3::new(-5.0, 8.5, 8.5),
        Vec3::X,
        100.0,
        false,
        &mut closest_stats,
    );

    let mut any_stats = MarchStats::default();
    let any = march(
        &tree,
        Affine3A::IDENTITY,
        Vec3::new(-5.0, 8.5, 8.5),
        Vec3::X,
        100.0,
        true,
        &mut any_stats,
    );

    assert!(closest.is_some() && any.is_some());
    assert!(any_stats.steps <= closest_stats.steps);
}

#[test]
fn a_translated_volume_hits_the_same_voxel() {
    let tree = single_voxel_tree();
    let shift = Vec3::new(100.0, 0.0, 0.0);
    let mut stats = MarchStats::default();

    let hit = march(
        &tree,
        Affine3A::from_translation(shift),
        Vec3::new(-5.0, 8.5, 8.5) + shift,
        Vec3::X,
        100.0,
        false,
        &mut stats,
    )
    .expect("expected a hit through the transform");

    assert_eq!(hit.voxel, UVec3::new(8, 8, 8));
    assert!((hit.t - 13.0).abs() < 1e-3);
}
