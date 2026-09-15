//! A voxel volume placed in the world by a rigid transform.
//!
//! Rigid means rotation and translation and nothing else. That is what keeps a
//! distance along a ray meaning the same thing inside the body's frame as
//! outside it, which is what lets the renderer compose bodies with the static
//! world by simply keeping the nearest hit.

use crate::contree::Contree;
use crate::march::{Hit, MarchStats, march};
use glam::{Affine3A, Quat, Vec3};

/// A voxel volume placed in the world by a rigid transform.
#[derive(Clone, Debug)]
pub struct Body {
    pub volume: Contree,
    pub position: Vec3,
    pub orientation: Quat,
}

impl Body {
    pub fn new(volume: Contree, position: Vec3, orientation: Quat) -> Self {
        Self { volume, position, orientation }
    }

    /// Volume space to world space.
    pub fn world_from_local(&self) -> Affine3A {
        Affine3A::from_rotation_translation(self.orientation, self.position)
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
