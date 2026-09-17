//! Deciding, on the CPU, which bodies a camera could possibly see.
//!
//! Only primary rays. Frustum culling is wrong for shadows: a body just outside
//! the view can shadow geometry inside it. Shadow composition must not reuse
//! this.

use crate::upload::{ExtractedMarchCamera, GpuBody, march_flags};
use glam::{Mat4, UVec3, Vec3, Vec4, Vec4Swizzles};

/// A body's bounding sphere in world space.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct BodyBound {
    pub centre: Vec3,
    pub radius: f32,
}

/// Places a body's local occupied box in the world as a sphere.
///
/// A sphere because a rigid transform leaves its radius unchanged, so it needs
/// no re-fitting when the body turns; the box's half-diagonal bounds it in
/// every orientation.
///
/// Takes the placed table entry, not the `Body`: the render world has only the
/// table, so this is the one spelling both it and the test harness can share.
/// The entry carries the inverse placement, inverted back here.
pub fn world_bound(local: (UVec3, UVec3), body: &GpuBody) -> BodyBound {
    let lo = local.0.as_vec3();
    let hi = local.1.as_vec3();
    let world_from_local = Mat4::from_cols_array_2d(&body.local_from_world).inverse();
    BodyBound {
        centre: world_from_local.transform_point3((lo + hi) * 0.5),
        radius: (hi - lo).length() * 0.5,
    }
}

/// The four side planes and a behind-the-camera plane.
///
/// No near or far plane from the matrix: the app's camera is reverse-Z with the
/// far plane at infinity, the test harness's is standard-Z, and those planes
/// mean opposite things in the two. The side planes use only x, y and w, so
/// they are the same in both.
pub struct Frustum {
    /// Inward-facing, as (normal, distance); a point p is inside when
    /// dot(normal, p) + distance >= 0.
    sides: [Vec4; 4],
    position: Vec3,
    forward: Vec3,
}

impl Frustum {
    pub fn from_camera(world_from_clip: Mat4, position: Vec3) -> Self {
        let clip_from_world = world_from_clip.inverse();
        let r0 = clip_from_world.row(0);
        let r1 = clip_from_world.row(1);
        let r3 = clip_from_world.row(3);
        let normalise = |p: Vec4| p / p.xyz().length();

        // z = 1 is in front of the camera in both conventions: the far plane
        // under standard-Z, the near plane under reverse-Z. z = 0 is not -- under
        // reverse-Z it is the plane at infinity.
        let ahead = world_from_clip * Vec4::new(0.0, 0.0, 1.0, 1.0);
        let forward = (ahead.xyz() / ahead.w - position).normalize();

        Self {
            sides: [
                normalise(r3 + r0),
                normalise(r3 - r0),
                normalise(r3 + r1),
                normalise(r3 - r1),
            ],
            position,
            forward,
        }
    }

    /// Whether any part of the sphere could be on screen. Errs towards yes.
    pub fn sees(&self, bound: BodyBound) -> bool {
        if (bound.centre - self.position).dot(self.forward) < -bound.radius {
            return false;
        }
        self.sides
            .iter()
            .all(|p| p.xyz().dot(bound.centre) + p.w >= -bound.radius)
    }
}

/// The table entries for the bodies this frustum could see, in table order.
///
/// Compacted: each `GpuBody` carries its own geometry bases, so dropping entries
/// leaves the rest valid. Any other per-body array must be built from the same
/// kept indices, in the same order.
pub fn visible_bodies(table: &[GpuBody], bounds: &[BodyBound], frustum: &Frustum) -> Vec<GpuBody> {
    table
        .iter()
        .zip(bounds)
        .filter(|(_, b)| frustum.sees(**b))
        .map(|(g, _)| *g)
        .collect()
}

/// The body table to march this frame: `placed` whole, or, when `flags` carry
/// `CULL_BODIES`, only the bodies `camera` could see.
///
/// The one cull both the render world and the test harness call, so the gates
/// test what the app runs. Flags are a parameter, not `DEFAULT`, so the harness
/// can render one scene with culling on and off in the same process.
///
/// The shader's body count must be taken from the length of what this returns,
/// never from `placed`: the table is compacted, and a count from the full list
/// would march entries past the ones written. A body with no voxels has no
/// bound and is never marched.
pub fn bodies_to_march(
    placed: &[GpuBody],
    local_bounds: &[Option<(UVec3, UVec3)>],
    camera: &ExtractedMarchCamera,
    flags: u32,
) -> Vec<GpuBody> {
    if flags & march_flags::CULL_BODIES == 0 {
        return placed.to_vec();
    }
    let (kept, bounds): (Vec<GpuBody>, Vec<BodyBound>) = placed
        .iter()
        .zip(local_bounds)
        .filter_map(|(g, local)| local.map(|l| (*g, world_bound(l, g))))
        .unzip();
    visible_bodies(&kept, &bounds, &Frustum::from_camera(camera.world_from_clip, camera.position))
}

#[cfg(test)]
mod tests {
    use super::*;
    use bevox_core::body::Body;
    use glam::Quat;

    fn standard(eye: Vec3, target: Vec3) -> Mat4 {
        let view = Mat4::look_at_rh(eye, target, Vec3::Y);
        let projection = Mat4::perspective_rh(0.9, 16.0 / 9.0, 0.1, 500.0);
        (projection * view).inverse()
    }

    /// Bevy's default projection: reverse-Z, far plane at infinity. The app
    /// uses this; the GPU harness does not.
    fn reverse_z(eye: Vec3, target: Vec3) -> Mat4 {
        let view = Mat4::look_at_rh(eye, target, Vec3::Y);
        let projection = Mat4::perspective_infinite_reverse_rh(0.9, 16.0 / 9.0, 0.1);
        (projection * view).inverse()
    }

    #[test]
    fn a_body_straight_ahead_is_seen() {
        let eye = Vec3::ZERO;
        for world_from_clip in [standard(eye, -Vec3::Z), reverse_z(eye, -Vec3::Z)] {
            let f = Frustum::from_camera(world_from_clip, eye);
            assert!(f.sees(BodyBound { centre: Vec3::new(0.0, 0.0, -50.0), radius: 5.0 }));
        }
    }

    #[test]
    fn a_body_behind_the_camera_is_culled() {
        let eye = Vec3::ZERO;
        for world_from_clip in [standard(eye, -Vec3::Z), reverse_z(eye, -Vec3::Z)] {
            let f = Frustum::from_camera(world_from_clip, eye);
            assert!(!f.sees(BodyBound { centre: Vec3::new(0.0, 0.0, 50.0), radius: 5.0 }));
        }
    }

    #[test]
    fn a_body_far_to_the_side_is_culled() {
        let eye = Vec3::ZERO;
        for world_from_clip in [standard(eye, -Vec3::Z), reverse_z(eye, -Vec3::Z)] {
            let f = Frustum::from_camera(world_from_clip, eye);
            assert!(!f.sees(BodyBound { centre: Vec3::new(500.0, 0.0, -50.0), radius: 5.0 }));
        }
    }

    /// Straddling an edge is seen. A cull that removed a body half on screen
    /// would change pixels, which is the one thing it must never do.
    #[test]
    fn a_body_straddling_a_side_plane_is_seen() {
        let eye = Vec3::ZERO;
        for world_from_clip in [standard(eye, -Vec3::Z), reverse_z(eye, -Vec3::Z)] {
            let f = Frustum::from_camera(world_from_clip, eye);
            // At z = -50 the horizontal half-width is 50 * tan(0.45) * 16/9 ~ 43.
            assert!(f.sees(BodyBound { centre: Vec3::new(45.0, 0.0, -50.0), radius: 5.0 }));
        }
    }

    /// A body around the camera itself, straddling the behind-the-camera plane,
    /// is seen.
    #[test]
    fn a_body_enclosing_the_camera_is_seen() {
        let eye = Vec3::ZERO;
        for world_from_clip in [standard(eye, -Vec3::Z), reverse_z(eye, -Vec3::Z)] {
            let f = Frustum::from_camera(world_from_clip, eye);
            assert!(f.sees(BodyBound { centre: Vec3::new(0.0, 0.0, 3.0), radius: 5.0 }));
        }
    }

    /// The two conventions must agree on every body, because the app runs one
    /// and every GPU gate runs the other. A disagreement is a cull that passes
    /// all the tests and breaks the running app.
    ///
    /// Under reverse-Z there is no far plane, so the standard-Z frustum must not
    /// cull on its far plane either. This sample cannot show that: it lies
    /// within about 370 of the eye, inside the far plane at 500, and a far
    /// plane added to the cull passes it. `nothing_is_culled_for_being_far`
    /// is the test that does.
    ///
    /// The two frustums come from two different matrices, each re-inverted, so
    /// their planes differ by float error. A body sitting within that error of a
    /// plane can legitimately land on either side. If this test ever fails, check
    /// the margin first: exclude only bodies whose signed distance to the nearest
    /// plane is within 1e-3 of their radius, and never widen the cull itself to
    /// make it pass.
    #[test]
    fn standard_and_reverse_z_agree_everywhere() {
        let mut rng = bevox_core::testing::XorShift64::new(17);
        let eye = Vec3::new(3.0, 7.0, -2.0);
        let target = Vec3::new(40.0, -10.0, -90.0);
        let a = Frustum::from_camera(standard(eye, target), eye);
        let b = Frustum::from_camera(reverse_z(eye, target), eye);
        let mut seen = 0;
        for _ in 0..4000 {
            let c = Vec3::new(
                rng.next_below(400) as f32 - 200.0,
                rng.next_below(400) as f32 - 200.0,
                rng.next_below(400) as f32 - 200.0,
            );
            let bound = BodyBound { centre: c, radius: rng.next_below(20) as f32 + 1.0 };
            assert_eq!(a.sees(bound), b.sees(bound), "conventions disagree about {bound:?}");
            seen += a.sees(bound) as u32;
        }
        // Not vacuous: some bodies must be seen and some culled.
        assert!(seen > 100 && seen < 3900, "{seen} of 4000 seen; the sample does not exercise the cull");
    }

    /// Past the standard projection's far plane at 500. The app's reverse-Z
    /// camera has no far plane and draws this body, so the standard frustum the
    /// GPU gates run must see it too.
    #[test]
    fn nothing_is_culled_for_being_far() {
        let eye = Vec3::ZERO;
        for world_from_clip in [standard(eye, -Vec3::Z), reverse_z(eye, -Vec3::Z)] {
            let f = Frustum::from_camera(world_from_clip, eye);
            assert!(f.sees(BodyBound { centre: Vec3::new(0.0, 0.0, -5000.0), radius: 5.0 }));
        }
    }

    /// The bound is recovered from the table entry the render world holds, which
    /// carries only the inverse placement. A rotated, translated body checks
    /// that the inversion puts the centre where the body's own transform does.
    #[test]
    fn the_world_bound_follows_the_placed_transform() {
        let orientation = Quat::from_euler(glam::EulerRot::XYZ, 0.4, 0.8, -0.3);
        let volume = bevox_core::contree::Contree::empty(2);
        let body = Body::new(volume, Vec3::new(9.0, -4.0, 30.0), orientation);
        let local = (UVec3::new(2, 4, 6), UVec3::new(10, 8, 16));
        let bound = world_bound(local, &GpuBody::default().placed(&body));

        let expected = body.world_from_local().transform_point3(Vec3::new(6.0, 6.0, 11.0));
        assert!(
            (bound.centre - expected).length() < 1e-3,
            "centre {:?}, expected {expected:?}",
            bound.centre
        );
        assert!((bound.radius - Vec3::new(8.0, 4.0, 10.0).length() * 0.5).abs() < 1e-5);
    }

    /// Off, every entry is marched as packed. On, an empty body and one out of
    /// view are dropped, and what is kept stays in table order.
    #[test]
    fn bodies_to_march_drops_only_what_the_flag_and_the_view_allow() {
        let eye = Vec3::ZERO;
        let camera =
            ExtractedMarchCamera { world_from_clip: reverse_z(eye, -Vec3::Z), position: eye };
        let at = |z: f32, id: u32| {
            let volume = bevox_core::contree::Contree::empty(2);
            let body = Body::new(volume, Vec3::new(-4.0, -4.0, z), Quat::IDENTITY);
            GpuBody { node_base: id, ..GpuBody::default().placed(&body) }
        };
        let cube = Some((UVec3::ZERO, UVec3::splat(8)));
        let table = [at(-50.0, 0), at(50.0, 1), at(-60.0, 2), at(-70.0, 3)];
        let bounds = [cube, cube, None, cube];

        let ids = |flags| {
            let kept = bodies_to_march(&table, &bounds, &camera, flags);
            kept.iter().map(|g| g.node_base).collect::<Vec<_>>()
        };
        assert_eq!(ids(march_flags::DEFAULT & !march_flags::CULL_BODIES), [0, 1, 2, 3]);
        assert_eq!(ids(march_flags::DEFAULT | march_flags::CULL_BODIES), [0, 3]);
    }
}
