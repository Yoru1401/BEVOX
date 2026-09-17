//! Deciding, on the CPU, which bodies a camera could possibly see.
//!
//! Only primary rays. Frustum culling is wrong for shadows: a body just outside
//! the view can shadow geometry inside it. Shadow composition must not reuse
//! this.

use crate::upload::{ExtractedMarchCamera, GpuBody, march_flags};
use glam::{Mat4, UVec2, UVec3, Vec2, Vec3, Vec4, Vec4Swizzles};

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
///
/// Camera-relative, like the rays: planes and bounds are compared as offsets
/// from the eye, so a camera far from the origin culls as precisely as one at
/// it.
pub struct Frustum {
    /// Inward-facing, as (normal, distance), in camera-relative space; an offset
    /// p from the eye is inside when dot(normal, p) + distance >= 0.
    sides: [Vec4; 4],
    position: Vec3,
    forward: Vec3,
}

impl Frustum {
    /// `offset_from_clip` is `ExtractedMarchCamera`'s, and `clip_from_offset` its
    /// inverse, passed in so a frame inverts it once.
    pub fn from_camera(offset_from_clip: Mat4, clip_from_offset: Mat4, position: Vec3) -> Self {
        let r0 = clip_from_offset.row(0);
        let r1 = clip_from_offset.row(1);
        let r3 = clip_from_offset.row(3);
        let normalise = |p: Vec4| p / p.xyz().length();

        // z = 1 is in front of the camera in both conventions: the far plane
        // under standard-Z, the near plane under reverse-Z. z = 0 is not -- under
        // reverse-Z it is the plane at infinity.
        let ahead = offset_from_clip * Vec4::new(0.0, 0.0, 1.0, 1.0);
        let forward = (ahead.xyz() / ahead.w).normalize();

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
        let centre = bound.centre - self.position;
        if centre.dot(self.forward) < -bound.radius {
            return false;
        }
        self.sides.iter().all(|p| p.xyz().dot(centre) + p.w >= -bound.radius)
    }
}

/// A body's footprint on screen, in pixels, inclusive at both ends.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, bytemuck::Pod, bytemuck::Zeroable)]
pub struct GpuBodyRect {
    pub min: [u32; 2],
    pub max: [u32; 2],
}

impl GpuBodyRect {
    /// Every pixel of a `size` target.
    pub fn whole(size: UVec2) -> Self {
        Self { min: [0, 0], max: [size.x - 1, size.y - 1] }
    }
}

/// The pixels a body can cover: its eight corners, as offsets from the eye,
/// projected and rounded outward, padded by one.
///
/// Inverts `primary_ray`'s mapping exactly. The pad absorbs only float error,
/// which camera-relative projection keeps relative rather than growing with the
/// distance from the origin. A corner at or behind the camera has no meaningful
/// projection, so the body gets the whole screen -- only conservative answers
/// are allowed.
///
/// `clip_from_offset` is the inverse of `ExtractedMarchCamera::offset_from_clip`
/// and `eye` its position. `size` must be the dimensions of the texture the
/// shader writes, which is what `primary_ray` maps pixels with.
pub fn screen_rect(
    local: (UVec3, UVec3),
    body: &GpuBody,
    clip_from_offset: Mat4,
    eye: Vec3,
    size: UVec2,
) -> GpuBodyRect {
    let world_from_local = Mat4::from_cols_array_2d(&body.local_from_world).inverse();
    let (lo, hi) = (local.0.as_vec3(), local.1.as_vec3());

    let mut min = Vec2::splat(f32::INFINITY);
    let mut max = Vec2::splat(f32::NEG_INFINITY);
    for i in 0..8 {
        let corner = Vec3::new(
            if i & 1 == 0 { lo.x } else { hi.x },
            if i & 2 == 0 { lo.y } else { hi.y },
            if i & 4 == 0 { lo.z } else { hi.z },
        );
        let offset = world_from_local.transform_point3(corner) - eye;
        let clip = clip_from_offset * offset.extend(1.0);
        // `w` is the corner's depth along the view. `transform_point3` still
        // passes through an absolute world-space point before `eye` is
        // subtracted, so `w` itself carries about |eye| * epsilon of error --
        // this is not immune to that. It is still safe: a corner misclassified
        // by that error keeps the sign of its true side in `xy / w`, and the
        // clamp turns it into the screen edge either way, so the rectangle
        // stays conservative.
        if clip.w <= 1e-6 {
            return GpuBodyRect::whole(size);
        }
        let ndc = clip.xy() / clip.w;
        let px = Vec2::new(
            (ndc.x + 1.0) * 0.5 * size.x as f32 - 0.5,
            (1.0 - ndc.y) * 0.5 * size.y as f32 - 0.5,
        );
        min = min.min(px);
        max = max.max(px);
    }

    let clamp = |v: f32, n: u32| (v.max(0.0) as u32).min(n - 1);
    GpuBodyRect {
        min: [clamp(min.x.floor() - 1.0, size.x), clamp(min.y.floor() - 1.0, size.y)],
        max: [clamp(max.x.ceil() + 1.0, size.x), clamp(max.y.ceil() + 1.0, size.y)],
    }
}

/// The body table to march this frame, and each kept body's screen rectangle:
/// `placed` whole, or, when `flags` carry `CULL_BODIES`, only the bodies
/// `camera` could see.
///
/// The one cull both the render world and the test harness call, so the gates
/// test what the app runs. Flags are a parameter, not `DEFAULT`, so the harness
/// can render one scene with culling on and off in the same process.
///
/// The table is compacted: each `GpuBody` carries its own geometry bases, so
/// dropping entries leaves the rest valid. The rectangles are built here, from
/// the same kept bodies in the same order, so the shader's body `i` is never
/// tested against another body's rectangle. They are built whatever the flags;
/// only the shader decides whether to read them. A body with no bound gets the
/// whole screen.
///
/// The shader's body count must be taken from the length of what this returns,
/// never from `placed`: a count from the full list would march entries past the
/// ones written. A body with no voxels has no bound and is never marched when
/// culling.
///
/// `size` is the dimensions of the texture this frame's dispatch writes.
pub fn bodies_to_march(
    placed: &[GpuBody],
    local_bounds: &[Option<(UVec3, UVec3)>],
    camera: &ExtractedMarchCamera,
    flags: u32,
    size: UVec2,
) -> (Vec<GpuBody>, Vec<GpuBodyRect>) {
    debug_assert!(placed.len() <= local_bounds.len());
    let cull = flags & march_flags::CULL_BODIES != 0;
    // Inverted once a frame, for the frustum and every rectangle.
    let clip_from_offset = camera.offset_from_clip.inverse();
    let frustum = Frustum::from_camera(camera.offset_from_clip, clip_from_offset, camera.position);
    placed
        .iter()
        .zip(local_bounds)
        .filter_map(|(g, local)| match local {
            Some(l) if cull && !frustum.sees(world_bound(*l, g)) => None,
            Some(l) => Some((*g, screen_rect(*l, g, clip_from_offset, camera.position, size))),
            None if cull => None,
            None => Some((*g, GpuBodyRect::whole(size))),
        })
        .unzip()
}

#[cfg(test)]
mod tests {
    use super::*;
    use bevox_core::body::Body;
    use glam::Quat;

    /// Camera-relative, as `ExtractedMarchCamera::offset_from_clip` is: the view
    /// looks from the origin along `target - eye`, with no translation.
    fn standard(eye: Vec3, target: Vec3) -> Mat4 {
        let view = Mat4::look_at_rh(Vec3::ZERO, target - eye, Vec3::Y);
        let projection = Mat4::perspective_rh(0.9, 16.0 / 9.0, 0.1, 500.0);
        (projection * view).inverse()
    }

    /// Bevy's default projection: reverse-Z, far plane at infinity. The app
    /// uses this; the GPU harness does not.
    fn reverse_z(eye: Vec3, target: Vec3) -> Mat4 {
        let view = Mat4::look_at_rh(Vec3::ZERO, target - eye, Vec3::Y);
        let projection = Mat4::perspective_infinite_reverse_rh(0.9, 16.0 / 9.0, 0.1);
        (projection * view).inverse()
    }

    fn frustum(offset_from_clip: Mat4, eye: Vec3) -> Frustum {
        Frustum::from_camera(offset_from_clip, offset_from_clip.inverse(), eye)
    }

    /// What `primary_ray` computes for pixel (x, y), as a CPU mirror, operation
    /// for operation.
    fn primary_ray(offset_from_clip: Mat4, x: u32, y: u32, size: UVec2) -> Vec3 {
        let ndc = Vec2::new(
            (2 * x + 1) as f32 - size.x as f32,
            size.y as f32 - (2 * y + 1) as f32,
        ) * (Vec2::ONE / size.as_vec2());
        let p = offset_from_clip * Vec4::new(ndc.x, ndc.y, 1.0, 1.0);
        (p.xyz() / p.w).normalize()
    }

    #[test]
    fn a_body_straight_ahead_is_seen() {
        let eye = Vec3::ZERO;
        for offset_from_clip in [standard(eye, -Vec3::Z), reverse_z(eye, -Vec3::Z)] {
            let f = frustum(offset_from_clip, eye);
            assert!(f.sees(BodyBound { centre: Vec3::new(0.0, 0.0, -50.0), radius: 5.0 }));
        }
    }

    #[test]
    fn a_body_behind_the_camera_is_culled() {
        let eye = Vec3::ZERO;
        for offset_from_clip in [standard(eye, -Vec3::Z), reverse_z(eye, -Vec3::Z)] {
            let f = frustum(offset_from_clip, eye);
            assert!(!f.sees(BodyBound { centre: Vec3::new(0.0, 0.0, 50.0), radius: 5.0 }));
        }
    }

    #[test]
    fn a_body_far_to_the_side_is_culled() {
        let eye = Vec3::ZERO;
        for offset_from_clip in [standard(eye, -Vec3::Z), reverse_z(eye, -Vec3::Z)] {
            let f = frustum(offset_from_clip, eye);
            assert!(!f.sees(BodyBound { centre: Vec3::new(500.0, 0.0, -50.0), radius: 5.0 }));
        }
    }

    /// Straddling an edge is seen. A cull that removed a body half on screen
    /// would change pixels, which is the one thing it must never do.
    #[test]
    fn a_body_straddling_a_side_plane_is_seen() {
        let eye = Vec3::ZERO;
        for offset_from_clip in [standard(eye, -Vec3::Z), reverse_z(eye, -Vec3::Z)] {
            let f = frustum(offset_from_clip, eye);
            // At z = -50 the horizontal half-width is 50 * tan(0.45) * 16/9 ~ 43.
            assert!(f.sees(BodyBound { centre: Vec3::new(45.0, 0.0, -50.0), radius: 5.0 }));
        }
    }

    /// A body around the camera itself, straddling the behind-the-camera plane,
    /// is seen.
    #[test]
    fn a_body_enclosing_the_camera_is_seen() {
        let eye = Vec3::ZERO;
        for offset_from_clip in [standard(eye, -Vec3::Z), reverse_z(eye, -Vec3::Z)] {
            let f = frustum(offset_from_clip, eye);
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
        let a = frustum(standard(eye, target), eye);
        let b = frustum(reverse_z(eye, target), eye);
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
        for offset_from_clip in [standard(eye, -Vec3::Z), reverse_z(eye, -Vec3::Z)] {
            let f = frustum(offset_from_clip, eye);
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

    /// A placed table entry and a 16-voxel box as its local bounds.
    fn cube_body(position: Vec3, orientation: Quat) -> (GpuBody, (UVec3, UVec3)) {
        let volume = bevox_core::contree::Contree::empty(3);
        let body = Body::new(volume, position, orientation);
        (GpuBody::default().placed(&body), (UVec3::splat(24), UVec3::splat(40)))
    }

    /// Asserts every pixel whose ray hits the body's box lies inside its
    /// rectangle, and returns how many pixels hit it.
    ///
    /// Rays are built exactly as the shader builds them, and brought into the
    /// body's frame as `compose_bodies` does, so the oracle carries the same
    /// float error the GPU does. The box, not the bounding sphere: the rectangle
    /// is built from the box's corners, and the sphere's footprint is wider.
    fn covered(offset_from_clip: Mat4, eye: Vec3, body: &GpuBody, local: (UVec3, UVec3), size: UVec2) -> u32 {
        let rect = screen_rect(local, body, offset_from_clip.inverse(), eye, size);
        let local_from_world = Mat4::from_cols_array_2d(&body.local_from_world);
        let (lo, hi) = (local.0.as_vec3(), local.1.as_vec3());
        let origin = local_from_world.transform_point3(eye);

        let mut inside = 0;
        for y in 0..size.y {
            for x in 0..size.x {
                let dir = primary_ray(offset_from_clip, x, y, size);
                let inv = local_from_world.transform_vector3(dir).recip();
                let (t0, t1) = ((lo - origin) * inv, (hi - origin) * inv);
                let (enter, exit) = (t0.min(t1).max_element(), t0.max(t1).min_element());
                if enter <= exit && exit >= 0.0 {
                    inside += 1;
                    assert!(
                        x >= rect.min[0] && x <= rect.max[0] && y >= rect.min[1] && y <= rect.max[1],
                        "pixel ({x}, {y}) can see the body but lies outside the rectangle {rect:?}"
                    );
                }
            }
        }
        inside
    }

    /// The rectangle must cover every pixel whose ray hits the body's box, so a
    /// rectangle a pixel short is caught here rather than as a missing column on
    /// screen.
    #[test]
    fn the_rectangle_covers_every_pixel_that_sees_the_body() {
        let size = UVec2::new(160, 90);
        let eye = Vec3::new(32.0, 32.0, -80.0);
        // Turned about the box's centre, so it stays in view.
        let turned = Quat::from_euler(glam::EulerRot::XYZ, 0.4, 0.8, -0.3);
        let bodies = [
            cube_body(Vec3::ZERO, Quat::IDENTITY),
            cube_body(Vec3::splat(32.0) - turned * Vec3::splat(32.0), turned),
        ];
        for offset_from_clip in [standard(eye, Vec3::splat(32.0)), reverse_z(eye, Vec3::splat(32.0))] {
            for (body, local) in &bodies {
                let inside = covered(offset_from_clip, eye, body, *local, size);
                assert!(inside > 50, "only {inside} pixels see the body; the test is not exercising the rectangle");
            }
        }
    }

    /// The app's camera where precision runs out: Bevy's default projection
    /// (reverse-Z, infinite, FOV pi/4, near 0.1) at 2560x1440, the physical size
    /// of a 1280x720 window on a 2x display, about 1450 from the origin.
    ///
    /// Rays unprojected through an absolute matrix drift about two pixels here
    /// (295 pixels of one body outside its rectangle, measured in review). A
    /// check near the origin at a small size cannot see that.
    #[test]
    fn the_rectangle_covers_every_pixel_far_from_the_origin_at_app_resolution() {
        let size = UVec2::new(2560, 1440);
        let projection =
            Mat4::perspective_infinite_reverse_rh(std::f32::consts::FRAC_PI_4, 16.0 / 9.0, 0.1);
        let eye = Vec3::new(1025.0, 60.0, 1025.0);
        assert!((eye.length() - 1450.0).abs() < 5.0);
        let offset_from_clip =
            (projection * Mat4::look_at_rh(Vec3::ZERO, Vec3::new(0.1, -0.05, 1.0), Vec3::Y)).inverse();

        let turned = Quat::from_euler(glam::EulerRot::XYZ, 0.4, 0.8, -0.3);
        let place = |offset: Vec3, orientation: Quat| {
            cube_body(eye + offset - orientation * Vec3::splat(32.0), orientation)
        };
        for (body, local) in
            [place(Vec3::new(-14.0, 6.0, 70.0), turned), place(Vec3::new(20.0, -12.0, 90.0), Quat::IDENTITY)]
        {
            let inside = covered(offset_from_clip, eye, &body, local, size);
            assert!(inside > 10_000, "only {inside} pixels see the body; the test is not exercising the rectangle");
        }
    }

    /// Straddling the camera plane must not produce a garbage rectangle from a
    /// corner behind the camera.
    #[test]
    fn a_body_around_the_camera_gets_the_whole_screen() {
        let size = UVec2::new(160, 90);
        let eye = Vec3::new(32.0, 32.0, 32.0);
        let target = Vec3::new(32.0, 32.0, 100.0);
        let (body, local) = cube_body(Vec3::ZERO, Quat::IDENTITY);
        for offset_from_clip in [standard(eye, target), reverse_z(eye, target)] {
            let rect = screen_rect(local, &body, offset_from_clip.inverse(), eye, size);
            assert_eq!(rect, GpuBodyRect::whole(size));
        }
    }

    /// Off, every entry is marched as packed. On, an empty body and one out of
    /// view are dropped, and what is kept stays in table order. Either way each
    /// rectangle is its own body's.
    #[test]
    fn bodies_to_march_drops_only_what_the_flag_and_the_view_allow() {
        let eye = Vec3::ZERO;
        let camera =
            ExtractedMarchCamera { offset_from_clip: reverse_z(eye, -Vec3::Z), position: eye };
        let at = |z: f32, id: u32| {
            let volume = bevox_core::contree::Contree::empty(2);
            let body = Body::new(volume, Vec3::new(-4.0, -4.0, z), Quat::IDENTITY);
            GpuBody { node_base: id, ..GpuBody::default().placed(&body) }
        };
        let cube = Some((UVec3::ZERO, UVec3::splat(8)));
        let table = [at(-50.0, 0), at(50.0, 1), at(-60.0, 2), at(-70.0, 3)];
        let bounds = [cube, cube, None, cube];

        let size = UVec2::new(160, 90);
        let ids = |flags| {
            let (kept, rects) = bodies_to_march(&table, &bounds, &camera, flags, size);
            let own: Vec<GpuBodyRect> = kept
                .iter()
                .map(|g| match bounds[g.node_base as usize] {
                    Some(local) => {
                        screen_rect(local, g, camera.offset_from_clip.inverse(), eye, size)
                    }
                    None => GpuBodyRect::whole(size),
                })
                .collect();
            assert_eq!(rects, own, "a rectangle is not its own body's");
            kept.iter().map(|g| g.node_base).collect::<Vec<_>>()
        };
        assert_eq!(ids(march_flags::DEFAULT & !march_flags::CULL_BODIES), [0, 1, 2, 3]);
        assert_eq!(ids(march_flags::DEFAULT | march_flags::CULL_BODIES), [0, 3]);
    }
}

