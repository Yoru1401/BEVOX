//! A voxel volume placed in the world by a rigid transform.
//!
//! Rigid means rotation and translation and nothing else. That is what keeps a
//! distance along a ray meaning the same thing inside the body's frame as
//! outside it, which is what lets the renderer compose bodies with the static
//! world by simply keeping the nearest hit.

use crate::contree::Contree;
use crate::march::{Hit, MarchStats, march};
use crate::material::MaterialTable;
use crate::physics::classify::{Features, features};
use crate::physics::contact::ContactKey;
use crate::physics::solver::ContactImpulse;
use crate::physics::mass::{MassProperties, mass_properties};
use glam::{Affine3A, Quat, UVec3, Vec3};
use std::collections::HashMap;

/// Which body a contact is against. The static world is `WORLD`.
///
/// An identity rather than an index: bodies are removed from the middle of the
/// scene's list, and a warm-start impulse must not follow whichever body takes
/// the vacated slot.
#[derive(Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Debug)]
pub struct BodyId(pub u64);

impl BodyId {
    /// The static world, which every body can touch and which never moves.
    pub const WORLD: BodyId = BodyId(0);
}

fn next_id() -> BodyId {
    use std::sync::atomic::{AtomicU64, Ordering};
    static NEXT: AtomicU64 = AtomicU64::new(1);
    BodyId(NEXT.fetch_add(1, Ordering::Relaxed))
}

/// A voxel volume placed in the world by a rigid transform, and the state that
/// moves it.
#[derive(Clone, Debug)]
pub struct Body {
    /// Names this body in a contact. A clone keeps it: a clone is the same body,
    /// which is what the render tests and the GPU packing rely on.
    pub id: BodyId,
    pub volume: Contree,
    /// Where `com` is in the world: the point the body turns about.
    pub position: Vec3,
    pub orientation: Quat,
    /// Centre of mass in volume coordinates.
    ///
    /// Zero until `recompute`, which leaves a body built by `new` placed
    /// exactly as it was before bodies had mass.
    pub com: Vec3,
    /// Zero until `recompute`. A body with no mass is not simulated.
    pub mass: MassProperties,
    /// Voxels per second.
    pub velocity: Vec3,
    /// About `com`, in world axes.
    ///
    /// Stored instead of angular velocity because, with no torque, it is
    /// exactly constant, which is what lets the conservation gate demand exact
    /// equality.
    pub angular_momentum: Vec3,
    /// Corner and edge voxels, the only ones tested for contact. Empty until
    /// `recompute`.
    pub features: Features,
    /// Each contact's accumulated impulse from the last tick, for warm
    /// starting. Cleared by `recompute`, whose voxels may have moved.
    pub warm: HashMap<ContactKey, ContactImpulse>,
}

impl Body {
    pub fn new(volume: Contree, position: Vec3, orientation: Quat) -> Self {
        debug_assert!(
            orientation.is_normalized(),
            "body orientation {orientation:?} is not a unit quaternion; it would scale the ray, \
             so `t` would stop meaning the same distance in the body's frame and the renderer's \
             nearest-hit composition would pick the wrong surface"
        );
        Self {
            id: next_id(),
            volume,
            position,
            orientation,
            com: Vec3::ZERO,
            mass: MassProperties::default(),
            velocity: Vec3::ZERO,
            angular_momentum: Vec3::ZERO,
            features: Features::default(),
            warm: HashMap::new(),
        }
    }

    /// Volume space to world space: about `com`, which sits at `position`.
    ///
    /// Written as one rotation and one translation rather than composed with a
    /// translation by `-com`. With `com` zero it is then exactly the matrix a
    /// body had before it had mass, which every render test's placement
    /// depends on.
    pub fn world_from_local(&self) -> Affine3A {
        Affine3A::from_rotation_translation(
            self.orientation,
            self.position - self.orientation * self.com,
        )
    }

    /// World space to volume space.
    ///
    /// The inverse of a rigid transform is rigid, so this rescales nothing and
    /// a distance along the ray survives the trip unchanged.
    pub fn local_from_world(&self) -> Affine3A {
        self.world_from_local().inverse()
    }

    /// Marches a world-space ray through this body.
    ///
    /// `march` already takes the placement and does the conversion, which is
    /// what keeps the CPU a usable reference for bodies rather than only for
    /// the static world.
    pub fn march_world(
        &self,
        origin: Vec3,
        dir: Vec3,
        max_dist: f32,
        stats: &mut MarchStats,
    ) -> Option<Hit> {
        march(&self.volume, self.world_from_local(), origin, dir, max_dist, false, stats)
    }

    /// The inverse inertia tensor in world axes.
    fn world_inverse_inertia(&self) -> glam::Mat3 {
        let r = glam::Mat3::from_quat(self.orientation);
        r * self.mass.inverse_inertia * r.transpose()
    }

    /// How fast the body turns, in world axes.
    pub fn angular_velocity(&self) -> Vec3 {
        self.world_inverse_inertia() * self.angular_momentum
    }

    /// Sets the spin, by setting the angular momentum that produces it.
    pub fn set_angular_velocity(&mut self, omega: Vec3) {
        self.angular_momentum = self.world_inverse_inertia().inverse() * omega;
    }

    /// Recomputes mass properties after the volume changed, or for the first
    /// time. The pivot moves to the new centre of mass and `position` moves
    /// with it, so every voxel stays where it was in the world.
    ///
    /// Returns `false`, and leaves the body massless, when nothing in the
    /// volume weighs anything: such a body should be removed.
    pub fn recompute(&mut self, materials: &MaterialTable) -> bool {
        self.warm.clear();
        let voxels = self.volume.voxels();
        let Some((mass, com)) = mass_properties(&voxels, materials) else {
            self.mass = MassProperties::default();
            self.features = Features::default();
            return false;
        };
        self.position += self.orientation * (com - self.com);
        self.com = com;
        self.mass = mass;
        self.features = features(&self.volume, &voxels);
        true
    }
}

/// The tightest box holding a volume's geometry, in its own voxel coordinates:
/// minimum inclusive, maximum exclusive. `None` for an empty volume.
///
/// Node-granular: it stops at the first node that is uniform or no larger than
/// a brick, so it can be up to one brick loose per side. That is conservative,
/// which is the only direction a cull may err in, and tight enough that a small
/// body is not bounded as its whole volume.
pub fn occupied_bounds(tree: &Contree) -> Option<(UVec3, UVec3)> {
    let mut lo = UVec3::splat(u32::MAX);
    let mut hi = UVec3::ZERO;
    let mut any = false;
    grow_bounds(tree, tree.root(), tree.depth() - 1, UVec3::ZERO, &mut lo, &mut hi, &mut any);
    any.then_some((lo, hi))
}

fn grow_bounds(
    tree: &Contree,
    node: crate::node::Node,
    level: u32,
    origin: UVec3,
    lo: &mut UVec3,
    hi: &mut UVec3,
    any: &mut bool,
) {
    if node.is_empty() {
        return;
    }
    let extent = crate::contree::level_extent(level);
    // A brick-sized or uniform node takes its whole box. Level 0 is a brick, so
    // this also stops the walk before voxel children, which live in another
    // arena and have no `Node` to recurse on.
    if level == 0 || !node.is_subdivided() {
        *lo = lo.min(origin);
        *hi = hi.max(origin + UVec3::splat(extent));
        *any = true;
        return;
    }
    let step = crate::contree::level_extent(level - 1);
    for i in 0..crate::node::CHILDREN {
        if let Some(slot) = node.child_slot(i) {
            // Inverse of child_index: x + y * 4 + z * 16.
            let c = UVec3::new(i % 4, (i / 4) % 4, i / 16);
            grow_bounds(tree, tree.arena().node(slot), level - 1, origin + c * step, lo, hi, any);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::dense::DenseVolume;
    use crate::material::MaterialId;
    use glam::UVec3;

    /// A solid 16-voxel cube in a 64 volume, centred on the volume's middle.
    fn cube() -> Contree {
        let mut dense = DenseVolume::new(64).unwrap();
        for z in 24..40 {
            for y in 24..40 {
                for x in 24..40 {
                    dense.set(UVec3::new(x, y, z), MaterialId(1));
                }
            }
        }
        dense.into_contree()
    }

    /// Recomputing moves the pivot, never the voxels. Milestone 4's brush edits
    /// recompute a body in mid-air and must not make it jump.
    #[test]
    fn recompute_keeps_every_voxel_where_it_was() {
        let materials = crate::physics::fixtures::materials();
        // An L, so the centre of mass is nowhere near the volume's corner or
        // its middle.
        let mut voxels = Vec::new();
        for x in 0..8 {
            voxels.push((UVec3::new(x, 0, 0), MaterialId(1)));
        }
        for y in 1..6 {
            voxels.push((UVec3::new(0, y, 0), MaterialId(2)));
        }
        let orientation = Quat::from_euler(glam::EulerRot::XYZ, 0.4, -1.1, 0.7);
        let mut body =
            Body::new(Contree::from_voxels(16, &voxels), Vec3::new(5.0, -3.0, 9.0), orientation);

        let world = |b: &Body, list: &[(UVec3, MaterialId)]| -> Vec<Vec3> {
            list.iter()
                .map(|(p, _)| b.world_from_local().transform_point3(p.as_vec3() + 0.5))
                .collect()
        };
        let before = world(&body, &voxels);
        assert!(body.recompute(&materials));
        assert_ne!(body.com, Vec3::ZERO, "the pivot did not move, so this proves nothing");
        for (a, b) in before.iter().zip(world(&body, &voxels)) {
            assert!((*a - b).length() < 1e-4, "recompute moved a voxel from {a:?} to {b:?}");
        }

        // Erase the upright of the L. The pivot moves again; the rest stays put.
        let base: Vec<_> = voxels.iter().copied().filter(|(p, _)| p.y == 0).collect();
        let before = world(&body, &base);
        let com = body.com;
        body.volume = Contree::from_voxels(16, &base);
        assert!(body.recompute(&materials));
        assert_ne!(body.com, com, "the edit did not move the pivot, so this proves nothing");
        for (a, b) in before.iter().zip(world(&body, &base)) {
            assert!((*a - b).length() < 1e-4, "an edit moved a voxel from {a:?} to {b:?}");
        }
    }

    #[test]
    fn recompute_classifies_the_voxels() {
        let mut body =
            Body::new(crate::physics::fixtures::cube(4, 4), Vec3::ZERO, Quat::IDENTITY);
        assert!(body.features.corners.is_empty());
        assert!(body.recompute(&crate::physics::fixtures::materials()));
        assert_eq!(body.features.corners.len(), 8);
        assert_eq!(body.features.edges.len(), 24);
    }

    /// Two bodies are never the same body, and a clone is the same body: the
    /// warm-start cache keys on this.
    #[test]
    fn every_body_gets_its_own_id() {
        let a = Body::new(cube(), Vec3::ZERO, Quat::IDENTITY);
        let b = Body::new(cube(), Vec3::ZERO, Quat::IDENTITY);
        assert_ne!(a.id, b.id);
        assert_eq!(a.id, a.clone().id);
        assert_ne!(a.id, BodyId::WORLD, "the world's id is reserved");
    }

    /// Spin set is spin read back, for a body turned any way: the world inertia
    /// turns with it.
    #[test]
    fn angular_velocity_round_trips() {
        let mut voxels = Vec::new();
        for x in 0..6 {
            voxels.push((UVec3::new(x, 0, 0), MaterialId(1)));
        }
        voxels.push((UVec3::new(0, 1, 0), MaterialId(2)));
        let materials = crate::physics::fixtures::materials();
        let mut body = Body::new(
            Contree::from_voxels(16, &voxels),
            Vec3::ZERO,
            Quat::from_euler(glam::EulerRot::XYZ, 0.4, 1.2, -0.3),
        );
        assert!(body.recompute(&materials));
        let omega = Vec3::new(0.7, -1.1, 0.4);
        body.set_angular_velocity(omega);
        let back = body.angular_velocity();
        assert!((back - omega).length() < 1e-4, "{back:?}");
    }

    #[test]
    fn a_body_with_no_voxels_does_not_recompute() {
        let mut body = Body::new(Contree::empty(2), Vec3::ZERO, Quat::IDENTITY);
        assert!(!body.recompute(&crate::physics::fixtures::materials()));
        assert_eq!(body.mass.mass, 0.0);
    }

    /// Every render test builds bodies with `new` and never recomputes them.
    /// Their placement must be the exact matrix it was before bodies had mass.
    #[test]
    fn a_body_never_recomputed_is_placed_exactly_as_before() {
        let orientation = Quat::from_euler(glam::EulerRot::XYZ, 0.3, 0.9, -0.4);
        let position = Vec3::new(12.5, -4.0, 30.0);
        let body = Body::new(cube(), position, orientation);
        assert_eq!(
            body.world_from_local(),
            Affine3A::from_rotation_translation(orientation, position)
        );
    }

    /// An untransformed body is marched exactly as the bare volume is, or the
    /// composition would perturb a scene that has not moved.
    #[test]
    fn an_identity_body_matches_marching_the_volume_directly() {
        let body = Body::new(cube(), Vec3::ZERO, Quat::IDENTITY);
        let origin = Vec3::new(32.0, 32.0, -40.0);
        let dir = Vec3::Z;

        let mut a = MarchStats::default();
        let direct = march(&body.volume, Affine3A::IDENTITY, origin, dir, 500.0, false, &mut a);
        let mut b = MarchStats::default();
        let through = body.march_world(origin, dir, 500.0, &mut b);

        assert_eq!(direct.map(|h| h.voxel), through.map(|h| h.voxel));
        assert_eq!(direct.map(|h| h.t), through.map(|h| h.t));

        // A cloned body (exercising the `Clone` derive step 3 added to
        // `Contree`/`NodeArena`) must march identically to its original.
        let mut c = MarchStats::default();
        let cloned = body.clone().march_world(origin, dir, 500.0, &mut c);
        assert_eq!(through.map(|h| h.t), cloned.map(|h| h.t));
    }

    /// Translating the body moves where the ray meets it by exactly the same
    /// amount, in the opposite sense along the ray.
    #[test]
    fn translating_the_body_shifts_the_hit_distance() {
        let origin = Vec3::new(32.0, 32.0, -40.0);
        let dir = Vec3::Z;
        let mut stats = MarchStats::default();

        let still = Body::new(cube(), Vec3::ZERO, Quat::IDENTITY)
            .march_world(origin, dir, 500.0, &mut stats)
            .expect("the ray should meet the cube");
        let moved = Body::new(cube(), Vec3::new(0.0, 0.0, 10.0), Quat::IDENTITY)
            .march_world(origin, dir, 500.0, &mut stats)
            .expect("the ray should still meet the cube");

        assert!(
            (moved.t - still.t - 10.0).abs() < 1e-3,
            "moving the body 10 along the ray changed the hit distance by {}",
            moved.t - still.t
        );
    }

    /// A rigid transform preserves distance. A quarter turn about the cube's
    /// own centre leaves a ray down the axis meeting it at the same distance,
    /// because a cube is symmetric under that rotation.
    ///
    /// Distance alone cannot catch a forward/inverse rotation swap here: for a
    /// ray along Z through a 90-degree-about-Y rotation about the cube's own
    /// centre, `R` and `R^-1` send the ray direction to exactly opposite local
    /// axes, so the entry distance comes out identical either way for *any*
    /// x/y offset of the ray -- offsetting the ray off the rotation centre
    /// (below) is still good practice but does not by itself distinguish the
    /// two. What differs is which face the ray enters through: the correct
    /// mapping enters the local +X face, the swapped one enters -X. That is
    /// what `face_normal` pins down.
    #[test]
    fn a_rigid_rotation_preserves_the_hit_distance() {
        let centre = Vec3::splat(32.0);
        let origin = Vec3::new(36.0, 30.0, -40.0);
        let dir = Vec3::Z;
        let mut stats = MarchStats::default();

        let still = Body::new(cube(), Vec3::ZERO, Quat::IDENTITY)
            .march_world(origin, dir, 500.0, &mut stats)
            .expect("hit");
        // Rotate about the volume's own centre: translate the centre to the
        // origin, turn, and put it back.
        let turned = Body::new(
            cube(),
            centre - Quat::from_rotation_y(std::f32::consts::FRAC_PI_2) * centre,
            Quat::from_rotation_y(std::f32::consts::FRAC_PI_2),
        )
        .march_world(origin, dir, 500.0, &mut stats)
        .expect("hit");

        assert!(
            (turned.t - still.t).abs() < 1e-3,
            "a quarter turn of a symmetric cube changed the hit distance from {} to {}",
            still.t,
            turned.t
        );
        // Entry distance is symmetric under this rotation (see comment above),
        // so it cannot catch a forward/inverse swap; the entered face can. A
        // swap enters the opposite local face and flips this to `Vec3::NEG_X`.
        assert_eq!(
            turned.face_normal,
            Vec3::X,
            "entered the wrong local face ({:?}); the rotation direction may be reversed",
            turned.face_normal
        );
    }

    /// The two transforms must actually invert each other, or every conversion
    /// in the renderer is subtly wrong in a way that looks like a physics bug.
    #[test]
    fn the_frames_are_inverses() {
        let body = Body::new(
            cube(),
            Vec3::new(5.0, -3.0, 11.0),
            Quat::from_euler(glam::EulerRot::XYZ, 0.3, -0.7, 1.1),
        );
        let p = Vec3::new(17.0, 4.0, -9.0);
        let round_trip = body.world_from_local().transform_point3(
            body.local_from_world().transform_point3(p),
        );
        assert!(
            (round_trip - p).length() < 1e-3,
            "a point round-tripped through both frames landed at {round_trip:?}, not {p:?}"
        );
    }

    /// Rotation must not rescale the ray, or `t` stops meaning the same thing
    /// in both frames and the renderer's nearest-hit comparison silently
    /// prefers whichever body happens to be scaled smaller.
    #[test]
    fn the_transform_does_not_rescale_a_direction() {
        let body = Body::new(
            cube(),
            Vec3::new(5.0, -3.0, 11.0),
            Quat::from_euler(glam::EulerRot::XYZ, 0.3, -0.7, 1.1),
        );
        let dir = Vec3::new(0.3, -0.9, 0.4).normalize();
        let local = body.local_from_world().transform_vector3(dir);
        assert!(
            (local.length() - 1.0).abs() < 1e-5,
            "a unit direction came out of the transform with length {}",
            local.length()
        );
    }

    #[test]
    fn an_empty_volume_has_no_bounds() {
        assert_eq!(occupied_bounds(&Contree::empty(3)), None);
    }

    /// The bound must contain every occupied voxel. That is the whole promise:
    /// a cull that trusts a bound missing a voxel removes a body that is on
    /// screen, and the pixels change.
    #[test]
    fn the_bounds_contain_every_occupied_voxel() {
        let mut rng = crate::testing::XorShift64::new(9_17);
        let mut voxels = Vec::new();
        for _ in 0..60 {
            voxels.push((
                UVec3::new(rng.next_below(64), rng.next_below(64), rng.next_below(64)),
                MaterialId(1),
            ));
        }
        let tree = Contree::from_voxels(64, &voxels);
        let (lo, hi) = occupied_bounds(&tree).expect("the volume has voxels");
        for (p, _) in &voxels {
            assert!(
                p.cmpge(lo).all() && p.cmplt(hi).all(),
                "voxel {p:?} lies outside the bounds {lo:?}..{hi:?}"
            );
        }
    }

    /// Node-granular, so conservative by at most one brick (4 voxels) per side
    /// -- and no looser, or a small body is bounded as a large one and the
    /// rectangle cull stops helping.
    ///
    /// Builds its own cube rather than using `cube()`. That helper fills 24..40,
    /// which is aligned to the brick grid and so collapses to uniform nodes whose
    /// boxes happen to be exact -- it would not exercise the rounding at all.
    /// 25..41 is deliberately off-grid, so the bound must round outward.
    #[test]
    fn the_bounds_are_tight_to_within_a_brick() {
        let mut dense = DenseVolume::new(64).unwrap();
        for z in 25..41 {
            for y in 25..41 {
                for x in 25..41 {
                    dense.set(UVec3::new(x, y, z), MaterialId(1));
                }
            }
        }
        let tree = dense.into_contree();
        let (lo, hi) = occupied_bounds(&tree).expect("the cube has voxels");
        for axis in 0..3 {
            assert!(lo[axis] <= 25 && lo[axis] + 4 > 25, "min {lo:?} is looser than a brick");
            assert!(hi[axis] >= 41 && hi[axis] < 41 + 4, "max {hi:?} is looser than a brick");
        }
    }

    /// Neither test above can catch a transposed axis in the child-origin
    /// decode: the random-voxel test spans close to the full 0..64 range on
    /// every axis regardless of which range lands on which axis, and the
    /// tightness cube is 25..41 on all three axes, so permuting axes leaves it
    /// unchanged. A box with a different, non-overlapping range per axis makes
    /// a swap between axes show up as a bound on the wrong values.
    #[test]
    fn the_bounds_do_not_transpose_axes() {
        let mut dense = DenseVolume::new(64).unwrap();
        for z in 50..54 {
            for y in 4..8 {
                for x in 25..41 {
                    dense.set(UVec3::new(x, y, z), MaterialId(1));
                }
            }
        }
        let tree = dense.into_contree();
        let (lo, hi) = occupied_bounds(&tree).expect("the box has voxels");
        let within_a_brick = |lo: u32, hi: u32, min: u32, max: u32| {
            lo <= min && lo + 4 > min && hi >= max && hi < max + 4
        };
        assert!(
            within_a_brick(lo.x, hi.x, 25, 41),
            "x bound {lo:?}..{hi:?} is not within a brick of 25..41"
        );
        assert!(
            within_a_brick(lo.y, hi.y, 4, 8),
            "y bound {lo:?}..{hi:?} is not within a brick of 4..8"
        );
        assert!(
            within_a_brick(lo.z, hi.z, 50, 54),
            "z bound {lo:?}..{hi:?} is not within a brick of 50..54"
        );
    }

    /// The frames must point the way their names say. A ray march can hide a
    /// swap when the geometry happens to be symmetric about the rotation; this
    /// cannot.
    #[test]
    fn world_from_local_places_the_local_origin_at_the_body_position() {
        let body = Body::new(
            cube(),
            Vec3::new(5.0, -3.0, 11.0),
            Quat::from_euler(glam::EulerRot::XYZ, 0.3, -0.7, 1.1),
        );

        let placed = body.world_from_local().transform_point3(Vec3::ZERO);
        assert!(
            (placed - body.position).length() < 1e-4,
            "the body's local origin landed at {placed:?}, not at its position {:?}",
            body.position
        );

        let back = body.local_from_world().transform_point3(body.position);
        assert!(
            back.length() < 1e-4,
            "the body's position should map to its local origin, got {back:?}"
        );
    }
}
