use bevox_core::contree::{CanonicalError, Contree};
use bevox_core::dense::DenseVolume;
use bevox_core::material::MaterialId;
use bevox_core::node::Node;
use bevox_core::testing::XorShift64;
use glam::UVec3;

#[test]
fn trees_built_from_dense_volumes_are_canonical() {
    let mut rng = XorShift64::new(4242);
    for extent in [4u32, 16, 64] {
        let mut dense = DenseVolume::new(extent).unwrap();
        for _ in 0..extent * extent {
            let p = UVec3::new(
                rng.next_below(extent),
                rng.next_below(extent),
                rng.next_below(extent),
            );
            dense.set(p, MaterialId(1 + rng.next_below(3) as u8));
        }
        let tree = Contree::from_dense(&dense);
        assert_eq!(tree.check_canonical(), Ok(()), "extent {extent}");
    }
}

#[test]
fn an_empty_tree_is_canonical() {
    assert_eq!(Contree::empty(3).check_canonical(), Ok(()));
}

#[test]
fn a_node_whose_children_are_all_the_same_solid_is_rejected() {
    // Hand-build a level-1 node with 64 identical uniform children, which
    // construction would have collapsed.
    let mut tree = Contree::empty(2);
    let base = tree.arena_mut().alloc_nodes(64);
    for i in 0..64 {
        tree.arena_mut().set_node(base + i, Node::uniform(MaterialId(3)));
    }
    tree.set_root_for_test(Node::subdivided(u64::MAX, base));
    assert!(matches!(
        tree.check_canonical(),
        Err(CanonicalError::UncollapsedUniform { .. })
    ));
}

#[test]
fn an_empty_child_occupying_a_slot_is_rejected() {
    let mut tree = Contree::empty(2);
    let base = tree.arena_mut().alloc_nodes(2);
    tree.arena_mut().set_node(base, Node::uniform(MaterialId(1)));
    tree.arena_mut().set_node(base + 1, Node::EMPTY);
    tree.set_root_for_test(Node::subdivided(0b11, base));
    assert!(matches!(
        tree.check_canonical(),
        Err(CanonicalError::EmptyChildStored { .. })
    ));
}

#[test]
fn a_child_slot_past_the_arena_is_rejected() {
    let mut tree = Contree::empty(2);
    tree.set_root_for_test(Node::subdivided(0b1, 9999));
    assert!(matches!(
        tree.check_canonical(),
        Err(CanonicalError::SlotOutOfBounds { .. })
    ));
}
