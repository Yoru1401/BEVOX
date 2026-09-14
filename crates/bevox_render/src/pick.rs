//! Turning a point on screen into a voxel in the world.
//!
//! The CPU reference marcher does the work. One ray per click is nothing next
//! to a frame of them, and it avoids a GPU readback with its latency and its
//! second copy of the traversal to keep in agreement.

use bevox_core::contree::Contree;
use bevox_core::march::{MarchStats, march};
use glam::{Affine3A, Mat4, UVec3, Vec2, Vec3};

/// Where a ray through the screen met the volume.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Pick {
    pub voxel: UVec3,
    /// The point on the surface, in world space.
    pub position: Vec3,
    /// The face the ray entered through.
    pub normal: Vec3,
}

/// The voxel under a point in normalised device coordinates.
///
/// `ndc` is -1 to 1 on each axis, y up, matching what the shader's `primary_ray`
/// builds from a pixel — so a pick agrees with what was drawn there.
pub fn pick_voxel(
    tree: &Contree,
    world_from_clip: Mat4,
    eye: Vec3,
    ndc: Vec2,
) -> Option<Pick> {
    let far = world_from_clip * glam::Vec4::new(ndc.x, ndc.y, 1.0, 1.0);
    let dir = (far.truncate() / far.w - eye).normalize();

    // Generous: the ray has to cross the whole volume from outside it.
    let max_dist = (tree.extent() as f32 * 8.0).max(1000.0);
    let mut stats = MarchStats::default();
    let hit = march(tree, Affine3A::IDENTITY, eye, dir, max_dist, false, &mut stats)?;

    Some(Pick {
        voxel: hit.voxel,
        position: eye + dir * hit.t,
        normal: hit.face_normal,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use bevox_core::dense::DenseVolume;
    use bevox_core::material::MaterialId;

    /// A single solid block in the middle of an otherwise empty volume.
    fn block_scene() -> Contree {
        let mut dense = DenseVolume::new(64).unwrap();
        for z in 28..36 {
            for y in 28..36 {
                for x in 28..36 {
                    dense.set(UVec3::new(x, y, z), MaterialId(1));
                }
            }
        }
        dense.into_contree()
    }

    fn camera(eye: Vec3, target: Vec3) -> Mat4 {
        let view = Mat4::look_at_rh(eye, target, Vec3::Y);
        let projection = Mat4::perspective_rh(0.9, 1.0, 0.1, 500.0);
        (projection * view).inverse()
    }

    #[test]
    fn the_centre_of_the_screen_hits_what_the_camera_looks_at() {
        let tree = block_scene();
        let eye = Vec3::new(32.0, 32.0, -40.0);
        let hit = pick_voxel(&tree, camera(eye, Vec3::splat(32.0)), eye, Vec2::ZERO)
            .expect("a camera pointed at the block should hit it");
        assert!(
            (28..36).contains(&hit.voxel.x)
                && (28..36).contains(&hit.voxel.y)
                && (28..36).contains(&hit.voxel.z),
            "picked {:?}, which is outside the block",
            hit.voxel
        );
    }

    #[test]
    fn the_normal_faces_the_camera() {
        let tree = block_scene();
        let eye = Vec3::new(32.0, 32.0, -40.0);
        let hit = pick_voxel(&tree, camera(eye, Vec3::splat(32.0)), eye, Vec2::ZERO).unwrap();
        // Approaching along +z, the entered face points back along -z.
        assert_eq!(hit.normal, Vec3::new(0.0, 0.0, -1.0));
    }

    #[test]
    fn a_ray_into_empty_space_picks_nothing() {
        let tree = block_scene();
        let eye = Vec3::new(32.0, 32.0, -40.0);
        // Look away from the block entirely.
        let away = camera(eye, eye + Vec3::new(0.0, 1.0, -1.0));
        assert!(pick_voxel(&tree, away, eye, Vec2::ZERO).is_none());
    }

    #[test]
    fn the_position_lies_on_the_picked_voxel() {
        let tree = block_scene();
        let eye = Vec3::new(32.0, 32.0, -40.0);
        let hit = pick_voxel(&tree, camera(eye, Vec3::splat(32.0)), eye, Vec2::ZERO).unwrap();
        let lo = hit.voxel.as_vec3();
        // On the surface, so a coordinate may sit exactly on a face.
        assert!(
            hit.position.cmpge(lo - Vec3::splat(0.001)).all()
                && hit.position.cmple(lo + Vec3::splat(1.001)).all(),
            "position {:?} is not on voxel {:?}",
            hit.position,
            hit.voxel
        );
    }

    #[test]
    fn an_off_centre_point_picks_a_different_voxel_than_the_centre() {
        // Guards against ignoring the ndc argument entirely, which would make
        // every click land wherever the camera happens to point.
        let tree = block_scene();
        let eye = Vec3::new(32.0, 32.0, -40.0);
        let world_from_clip = camera(eye, Vec3::splat(32.0));
        let centre = pick_voxel(&tree, world_from_clip, eye, Vec2::ZERO).unwrap();
        let offset = pick_voxel(&tree, world_from_clip, eye, Vec2::new(0.1, 0.0)).unwrap();
        assert_ne!(centre.voxel, offset.voxel);
    }
}
