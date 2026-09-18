//! Turning a point on screen into a voxel in the world.
//!
//! The CPU reference marcher does the work. One ray per click is nothing next
//! to a frame of them, and it avoids a GPU readback with its latency and its
//! second copy of the traversal to keep in agreement.

use bevox_core::body::Body;
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

/// The ray through a point in normalised device coordinates.
///
/// `ndc` is -1 to 1 on each axis, y up, matching what the shader's `primary_ray`
/// builds from a pixel, so a pick agrees with what was drawn there.
fn ray(world_from_clip: Mat4, eye: Vec3, ndc: Vec2) -> Vec3 {
    let far = world_from_clip * glam::Vec4::new(ndc.x, ndc.y, 1.0, 1.0);
    (far.truncate() / far.w - eye).normalize()
}

/// How far a pick looks: far enough to cross the whole world from outside it.
fn reach(tree: &Contree) -> f32 {
    (tree.extent() as f32 * 8.0).max(1000.0)
}

/// The voxel of the world under a point in normalised device coordinates.
pub fn pick_voxel(
    tree: &Contree,
    world_from_clip: Mat4,
    eye: Vec3,
    ndc: Vec2,
) -> Option<Pick> {
    let dir = ray(world_from_clip, eye, ndc);
    let mut stats = MarchStats::default();
    let hit = march(tree, Affine3A::IDENTITY, eye, dir, reach(tree), false, &mut stats)?;

    Some(Pick {
        voxel: hit.voxel,
        position: eye + dir * hit.t,
        normal: hit.face_normal,
    })
}

/// What a pick landed on.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Target {
    World,
    /// The body at this index of the scene's list.
    Body(usize),
}

/// Where a ray through the screen met the world or a body.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Hit {
    pub target: Target,
    /// The point on the surface, in world space.
    pub position: Vec3,
    /// The face the ray entered through, in world space.
    pub normal: Vec3,
}

/// Whatever is under a point on screen: the nearest of the world and every body.
///
/// A body's march answers in the body's own frame for its normal, and in world
/// distance for `t`, which the rigid transform preserves, so hits compare
/// directly and the normal needs only the body's rotation.
pub fn pick(
    tree: &Contree,
    bodies: &[Body],
    world_from_clip: Mat4,
    eye: Vec3,
    ndc: Vec2,
) -> Option<Hit> {
    let dir = ray(world_from_clip, eye, ndc);
    let max = reach(tree);
    let mut stats = MarchStats::default();
    let mut best = march(tree, Affine3A::IDENTITY, eye, dir, max, false, &mut stats)
        .map(|h| (h.t, Target::World, h.face_normal));
    for (i, body) in bodies.iter().enumerate() {
        let Some(h) = body.march_world(eye, dir, max, &mut stats) else { continue };
        if best.is_none_or(|(t, _, _)| h.t < t) {
            best = Some((h.t, Target::Body(i), body.orientation * h.face_normal));
        }
    }
    best.map(|(t, target, normal)| Hit { target, position: eye + dir * t, normal })
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

    /// A vertical offset must move the pick vertically, and upward on screen
    /// must mean upward in the world.
    ///
    /// Screen coordinates run y-down and NDC runs y-up, so there is a flip in
    /// the middle of this. Nothing caught a flip while the app only ever
    /// picked at the screen centre; now that it picks at the cursor, a flip
    /// would put every click on the wrong side of what the user aimed at.
    #[test]
    fn an_upward_offset_picks_a_higher_voxel() {
        let tree = block_scene();
        // Level with the block's centre, looking straight at it, so "up on
        // screen" is unambiguously "up in the world".
        let eye = Vec3::new(32.0, 32.0, -40.0);
        let world_from_clip = camera(eye, Vec3::splat(32.0));
        let centre = pick_voxel(&tree, world_from_clip, eye, Vec2::ZERO).unwrap();
        let up = pick_voxel(&tree, world_from_clip, eye, Vec2::new(0.0, 0.1)).unwrap();
        assert!(
            up.voxel.y > centre.voxel.y,
            "an upward offset picked {:?}, not above the centre pick {:?}",
            up.voxel,
            centre.voxel
        );
    }

    /// The two axes must not be swapped.
    ///
    /// `an_off_centre_point_picks_a_different_voxel_than_the_centre` only
    /// proves `ndc` is read at all: feeding y into the x slot would still
    /// produce "a different voxel" and pass it.
    #[test]
    fn the_two_ndc_axes_are_not_swapped() {
        let tree = block_scene();
        let eye = Vec3::new(32.0, 32.0, -40.0);
        let world_from_clip = camera(eye, Vec3::splat(32.0));
        let centre = pick_voxel(&tree, world_from_clip, eye, Vec2::ZERO).unwrap();
        let x_off = pick_voxel(&tree, world_from_clip, eye, Vec2::new(0.1, 0.0)).unwrap();
        let y_off = pick_voxel(&tree, world_from_clip, eye, Vec2::new(0.0, 0.1)).unwrap();

        assert_ne!(x_off.voxel.x, centre.voxel.x, "a horizontal offset did not move x");
        assert_eq!(x_off.voxel.y, centre.voxel.y, "a horizontal offset moved y");
        assert_ne!(y_off.voxel.y, centre.voxel.y, "a vertical offset did not move y");
        assert_eq!(y_off.voxel.x, centre.voxel.x, "a vertical offset moved x");
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

    /// A body in front of the world is what the cursor is on, and its normal
    /// comes back in world space, facing the eye.
    #[test]
    fn a_body_in_front_of_the_world_is_picked() {
        let tree = block_scene();
        let eye = Vec3::new(32.0, 32.0, -20.0);
        let world_from_clip = camera(eye, Vec3::splat(32.0));
        let solid: Vec<_> = (0..64u32)
            .map(|i| (UVec3::new(i % 4, (i / 4) % 4, i / 16), MaterialId(2)))
            .collect();
        let turned = glam::Quat::from_rotation_y(0.6);
        let body = Body::new(Contree::from_voxels(4, &solid), Vec3::new(30.0, 30.0, 5.0), turned);

        let hit = pick(&tree, std::slice::from_ref(&body), world_from_clip, eye, Vec2::ZERO).unwrap();
        assert_eq!(hit.target, Target::Body(0));
        assert!(hit.position.z < 10.0, "hit {:?}, behind the body", hit.position);
        assert!((hit.normal.length() - 1.0).abs() < 1e-4);
        assert!(hit.normal.dot(eye - hit.position) > 0.0, "the normal faces away from the eye");
        assert!(
            (hit.normal - turned * -Vec3::Z).length() < 1e-4
                || (hit.normal - turned * -Vec3::X).length() < 1e-4,
            "the normal {:?} is not one of the turned body's faces",
            hit.normal
        );

        let empty = Body::new(Contree::empty(1), Vec3::new(30.0, 30.0, 5.0), turned);
        let hit = pick(&tree, std::slice::from_ref(&empty), world_from_clip, eye, Vec2::ZERO).unwrap();
        assert_eq!(hit.target, Target::World);
    }
}
