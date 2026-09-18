use bevox_core::contree::Contree;
use bevox_core::dense::DenseVolume;
use bevox_core::material::MaterialId;
use glam::{UVec3, Vec3};

/// Ground truth: the same sphere applied to a flat volume.
fn dense_sphere(extent: u32, center: Vec3, radius: f32, material: MaterialId) -> DenseVolume {
    let mut dense = DenseVolume::new(extent).unwrap();
    for z in 0..extent {
        for y in 0..extent {
            for x in 0..extent {
                let c = Vec3::new(x as f32, y as f32, z as f32) + Vec3::splat(0.5);
                if c.distance(center) <= radius {
                    dense.set(UVec3::new(x, y, z), material);
                }
            }
        }
    }
    dense
}

#[test]
fn a_sphere_added_to_an_empty_tree_matches_the_dense_result() {
    let mut tree = Contree::empty(2); // 16 voxels per axis
    let center = Vec3::splat(8.0);
    tree.apply_sphere(center, 5.0, MaterialId(2));
    assert_eq!(tree.to_dense(), dense_sphere(16, center, 5.0, MaterialId(2)));
    assert_eq!(tree.check_canonical(), Ok(()));
}

#[test]
fn voxels_outside_the_radius_are_untouched() {
    let mut dense = DenseVolume::new(16).unwrap();
    dense.set(UVec3::new(0, 0, 0), MaterialId(7));
    dense.set(UVec3::new(15, 15, 15), MaterialId(7));
    let mut tree = Contree::from_dense(&dense);

    tree.apply_sphere(Vec3::splat(8.0), 2.0, MaterialId(2));

    assert_eq!(tree.get(UVec3::new(0, 0, 0)), MaterialId(7));
    assert_eq!(tree.get(UVec3::new(15, 15, 15)), MaterialId(7));
    assert_eq!(tree.get(UVec3::splat(8)), MaterialId(2));
}

#[test]
fn erasing_with_the_empty_material_removes_voxels_and_recollapses() {
    let mut dense = DenseVolume::new(16).unwrap();
    for z in 0..16 {
        for y in 0..16 {
            for x in 0..16 {
                dense.set(UVec3::new(x, y, z), MaterialId(1));
            }
        }
    }
    let mut tree = Contree::from_dense(&dense);
    assert!(tree.root().is_uniform_solid());

    // A radius large enough to cover the whole volume.
    tree.apply_sphere(Vec3::splat(8.0), 64.0, MaterialId::EMPTY);

    assert!(tree.root().is_empty());
    assert_eq!(tree.check_canonical(), Ok(()));
}

#[test]
fn filling_everything_collapses_back_to_a_uniform_root() {
    let mut tree = Contree::empty(2);
    tree.apply_sphere(Vec3::splat(8.0), 64.0, MaterialId(4));
    assert!(tree.root().is_uniform_solid());
    assert_eq!(tree.root().material(), MaterialId(4));
    assert_eq!(tree.check_canonical(), Ok(()));
}

#[test]
fn dirty_ranges_are_sufficient_to_reproduce_the_edited_buffers() {
    let mut tree = Contree::empty(3); // 64 voxels per axis
    tree.apply_sphere(Vec3::splat(32.0), 10.0, MaterialId(1));

    // Snapshot the buffers as a GPU would hold them, then clear the flags.
    let mut mirror_nodes = tree.arena().nodes().to_vec();
    let mut mirror_voxels = tree.arena().voxels().to_vec();
    tree.arena_mut().clear_dirty();

    tree.apply_sphere(Vec3::new(40.0, 32.0, 32.0), 6.0, MaterialId(2));

    // Apply only the dirty ranges to the mirror, exactly as an upload would.
    mirror_nodes.resize(tree.arena().nodes().len(), Default::default());
    mirror_voxels.resize(tree.arena().voxels().len(), 0);
    for r in tree.arena().dirty_nodes() {
        let range = r.start as usize..r.end as usize;
        mirror_nodes[range.clone()].copy_from_slice(&tree.arena().nodes()[range]);
    }
    for r in tree.arena().dirty_voxels() {
        let range = r.start as usize..r.end as usize;
        mirror_voxels[range.clone()].copy_from_slice(&tree.arena().voxels()[range]);
    }

    assert_eq!(mirror_nodes, tree.arena().nodes());
    assert_eq!(mirror_voxels, tree.arena().voxels());
}

#[test]
fn repeated_identical_edits_are_stable() {
    let mut tree = Contree::empty(2);
    tree.apply_sphere(Vec3::splat(8.0), 4.0, MaterialId(3));
    let after_first = tree.to_dense();
    tree.apply_sphere(Vec3::splat(8.0), 4.0, MaterialId(3));
    assert_eq!(tree.to_dense(), after_first);
    assert_eq!(tree.check_canonical(), Ok(()));
}

#[test]
fn painting_then_erasing_the_same_sphere_restores_the_tree() {
    // The property the brush rests on: erase is paint with EMPTY, and the
    // two compose back to where they started. If they do not, repeated
    // editing drifts and the arena grows without bound.
    let mut dense = DenseVolume::new(64).unwrap();
    for z in 0..64 {
        for x in 0..64 {
            for y in 0..6 {
                dense.set(UVec3::new(x, y, z), MaterialId(1));
            }
        }
    }
    let tree = dense.into_contree();
    let before = tree.to_dense();

    let mut edited = tree;
    edited.apply_sphere(Vec3::new(32.0, 3.0, 32.0), 5.0, MaterialId(2));
    assert_ne!(edited.to_dense(), before, "the paint did nothing");

    // Erase exactly what was painted, then repaint the floor it removed.
    edited.apply_sphere(Vec3::new(32.0, 3.0, 32.0), 5.0, MaterialId::EMPTY);
    for z in 0..64 {
        for x in 0..64 {
            for y in 0..6 {
                let p = UVec3::new(x, y, z);
                if before.get(p) != MaterialId::EMPTY && edited.get(p) == MaterialId::EMPTY {
                    edited.apply_sphere(p.as_vec3() + Vec3::splat(0.5), 0.4, MaterialId(1));
                }
            }
        }
    }
    assert_eq!(edited.to_dense(), before, "paint then erase did not round trip");
    edited.check_canonical().expect("the tree stopped being canonical");
}

/// Clearing takes exactly the voxels named and leaves the rest alone.
#[test]
fn clear_voxels_removes_only_what_it_is_given() {
    let mut dense = DenseVolume::new(16).unwrap();
    for z in 0..8 {
        for y in 0..8 {
            for x in 0..8 {
                dense.set(UVec3::new(x, y, z), MaterialId(1));
            }
        }
    }
    let mut tree = dense.into_contree();
    let gone: Vec<UVec3> = (0..8).map(|x| UVec3::new(x, 3, 3)).collect();
    tree.clear_voxels(&gone);

    for p in &gone {
        assert!(tree.get(*p).is_empty(), "{p:?} survived");
    }
    assert_eq!(tree.voxels().len(), 8 * 8 * 8 - gone.len());
    assert!(tree.check_canonical().is_ok(), "the tree is no longer canonical");
}

/// The edit goes through the arena, so the render world can upload a delta
/// rather than rebuilding. A piece that vanished any other way would keep being
/// drawn.
#[test]
fn clear_voxels_marks_the_arena_dirty() {
    let mut dense = DenseVolume::new(16).unwrap();
    for x in 0..8 {
        dense.set(UVec3::new(x, 0, 0), MaterialId(1));
    }
    let mut tree = dense.into_contree();
    tree.arena_mut().clear_dirty();
    tree.clear_voxels(&[UVec3::new(3, 0, 0)]);
    let dirty = tree.arena().dirty_nodes().len() + tree.arena().dirty_voxels().len();
    assert!(dirty > 0, "nothing was marked dirty");
}

/// Clearing nothing changes nothing.
#[test]
fn clear_voxels_of_nothing_is_a_no_op() {
    let mut tree = Contree::from_voxels(16, &[(UVec3::new(1, 2, 3), MaterialId(1))]);
    tree.clear_voxels(&[]);
    assert_eq!(tree.voxels().len(), 1);
}
