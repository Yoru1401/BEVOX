//! Fly camera. The arithmetic lives in free functions so it is testable
//! without a window, an input device or a running app.

use bevy::math::Affine3A;
use bevy::prelude::*;
use bevox_core::contree::Contree;
use bevox_core::march::{MarchStats, march};
use std::f32::consts::FRAC_PI_2;

/// Slightly under a right angle, so looking straight up never flips the basis.
const PITCH_LIMIT: f32 = FRAC_PI_2 - 0.001;

#[derive(Component, Debug, Clone, Copy)]
pub struct FlyCamera {
    pub speed: f32,
    pub sensitivity: f32,
    pub yaw: f32,
    pub pitch: f32,
}

impl Default for FlyCamera {
    fn default() -> Self {
        Self { speed: 24.0, sensitivity: 0.003, yaw: 0.0, pitch: 0.0 }
    }
}

impl FlyCamera {
    /// Builds a camera already facing `target` from `from`.
    ///
    /// This exists because `fly_camera_system` rewrites the transform's rotation
    /// from yaw and pitch every frame. Spawning with `Transform::looking_at` and
    /// a default `FlyCamera` therefore looks correct for exactly one frame and
    /// is then overwritten with identity — the camera silently turns away from
    /// whatever it was aimed at.
    pub fn looking_at(from: Vec3, target: Vec3) -> Self {
        let (yaw, pitch) = yaw_pitch_towards(target - from);
        Self { yaw, pitch, ..Default::default() }
    }
}

/// Yaw and pitch whose resulting forward vector matches `direction`.
///
/// Inverts `rotation = Y(yaw) * X(pitch)` applied to -Z.
pub fn yaw_pitch_towards(direction: Vec3) -> (f32, f32) {
    let d = direction.normalize_or_zero();
    if d == Vec3::ZERO {
        return (0.0, 0.0);
    }
    let pitch = d.y.clamp(-1.0, 1.0).asin();
    let yaw = (-d.x).atan2(-d.z);
    (yaw, pitch)
}

/// Applies mouse motion to a yaw/pitch pair. Pitch is clamped; yaw is free.
pub fn apply_look(yaw: f32, pitch: f32, delta: Vec2, sensitivity: f32) -> (f32, f32) {
    let new_yaw = yaw - delta.x * sensitivity;
    let new_pitch = (pitch - delta.y * sensitivity).clamp(-PITCH_LIMIT, PITCH_LIMIT);
    (new_yaw, new_pitch)
}

/// Movement direction in world space for the given yaw and per-axis inputs.
/// Normalised, so diagonal movement is not faster than straight movement.
pub fn movement_vector(yaw: f32, forward: f32, right: f32, up: f32) -> Vec3 {
    let rotation = Quat::from_rotation_y(yaw);
    let f = rotation * Vec3::NEG_Z;
    let r = rotation * Vec3::X;
    let v = f * forward + r * right + Vec3::Y * up;
    if v.length_squared() > 1e-6 { v.normalize() } else { Vec3::ZERO }
}

/// Reads input and moves any entity carrying a [`FlyCamera`].
pub fn fly_camera_system(
    time: Res<Time>,
    keys: Res<ButtonInput<KeyCode>>,
    mouse: Res<ButtonInput<MouseButton>>,
    mut motion: MessageReader<bevy::input::mouse::MouseMotion>,
    mut query: Query<(&mut Transform, &mut FlyCamera)>,
) {
    let mut delta = Vec2::ZERO;
    for m in motion.read() {
        delta += m.delta;
    }

    for (mut transform, mut cam) in &mut query {
        // Middle drag looks. Right is the brush's erase button, and left its
        // paint button, so look takes the one control neither edits with.
        if mouse.pressed(MouseButton::Middle) {
            let (yaw, pitch) = apply_look(cam.yaw, cam.pitch, delta, cam.sensitivity);
            cam.yaw = yaw;
            cam.pitch = pitch;
        }
        transform.rotation = Quat::from_rotation_y(cam.yaw) * Quat::from_rotation_x(cam.pitch);

        let forward = axis(&keys, KeyCode::KeyW, KeyCode::KeyS);
        let right = axis(&keys, KeyCode::KeyD, KeyCode::KeyA);
        let up = axis(&keys, KeyCode::Space, KeyCode::ShiftLeft);
        let boost = if keys.pressed(KeyCode::ControlLeft) { 4.0 } else { 1.0 };

        let step = movement_vector(cam.yaw, forward, right, up);
        transform.translation += step * cam.speed * boost * time.delta_secs();
    }
}

fn axis(keys: &ButtonInput<KeyCode>, positive: KeyCode, negative: KeyCode) -> f32 {
    (keys.pressed(positive) as i32 as f32) - (keys.pressed(negative) as i32 as f32)
}

/// Where the camera starts: just in front of the first surface on the way to
/// the volume's centre, rather than framing the whole thing from outside.
///
/// Framing from outside puts the camera where the renderer is cheapest and the
/// scene is least interesting, and it switches the distance field off
/// entirely -- the skip walk gives up on its first sample outside the grid, so
/// a framed start pays the field's load cost and gets none of its benefit
/// until the user flies in.
///
/// The surface is found with the CPU reference marcher rather than guessed, so
/// this lands sensibly whether the model is 16 voxels across or 4096, and
/// whether its geometry sits at the centre or off in a corner.
pub fn start_camera(tree: &Contree) -> (Vec3, Vec3) {
    let extent = tree.extent() as f32;
    let centre = Vec3::splat(extent * 0.5);
    // The old framed position, now only a place to look *from*.
    let outside = centre + Vec3::new(-1.0, 1.2, -1.0).normalize() * extent * 1.1;
    let dir = (centre - outside).normalize();

    let mut stats = MarchStats::default();
    let hit = march(
        tree,
        Affine3A::IDENTITY,
        outside,
        dir,
        extent * 8.0,
        false,
        &mut stats,
    );

    match hit {
        // Stand off the surface by a fixed distance in voxels, not a fraction
        // of the extent: "close" has to mean the same apparent size whether the
        // model is tiny or 4096 across.
        Some(hit) => {
            let surface = outside + dir * hit.t;
            (surface - dir * 24.0, surface)
        }
        // Nothing on that line of sight -- an empty or very sparse scene. Stand
        // at the centre and look along the same direction rather than framing
        // from outside, so the view is still inside the volume.
        None => (centre, centre + dir * extent * 0.25),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn looking_up_is_clamped_below_a_right_angle() {
        let (_, pitch) = apply_look(0.0, 0.0, Vec2::new(0.0, -10_000.0), 0.003);
        assert!(pitch <= PITCH_LIMIT, "pitch {pitch} exceeded the limit");
        assert!(pitch > 0.0);
    }

    #[test]
    fn looking_down_is_clamped_above_the_negative_right_angle() {
        let (_, pitch) = apply_look(0.0, 0.0, Vec2::new(0.0, 10_000.0), 0.003);
        assert!(pitch >= -PITCH_LIMIT, "pitch {pitch} exceeded the limit");
        assert!(pitch < 0.0);
    }

    #[test]
    fn yaw_accumulates_and_is_unbounded() {
        let (yaw, _) = apply_look(1.0, 0.0, Vec2::new(100.0, 0.0), 0.01);
        assert!((yaw - 1.0).abs() > 1e-6, "yaw should have moved off its input");
        // Turning right then left returns to the start.
        let (back, _) = apply_look(yaw, 0.0, Vec2::new(-100.0, 0.0), 0.01);
        assert!((back - 1.0).abs() < 1e-5, "yaw did not return, got {back}");
    }

    #[test]
    fn forward_movement_at_zero_yaw_points_down_negative_z() {
        let v = movement_vector(0.0, 1.0, 0.0, 0.0);
        assert!((v - Vec3::NEG_Z).length() < 1e-5, "got {v:?}");
    }

    #[test]
    fn a_quarter_turn_of_yaw_sends_forward_along_negative_x() {
        let v = movement_vector(FRAC_PI_2, 1.0, 0.0, 0.0);
        assert!((v - Vec3::NEG_X).length() < 1e-5, "got {v:?}");
    }

    #[test]
    fn vertical_movement_ignores_yaw() {
        let a = movement_vector(0.0, 0.0, 0.0, 1.0);
        let b = movement_vector(2.3, 0.0, 0.0, 1.0);
        assert!((a - Vec3::Y).length() < 1e-5);
        assert!((a - b).length() < 1e-5, "up should not depend on yaw");
    }

    #[test]
    fn no_input_produces_no_movement() {
        assert_eq!(movement_vector(1.2, 0.0, 0.0, 0.0), Vec3::ZERO);
    }

    /// The rotation the system builds each frame must actually face the target,
    /// or the camera turns away from whatever it was spawned looking at.
    #[test]
    fn looking_at_produces_a_rotation_that_faces_the_target() {
        let from = Vec3::new(-30.0, 40.0, -30.0);
        let target = Vec3::new(32.0, 12.0, 32.0);
        let cam = FlyCamera::looking_at(from, target);

        // Exactly what fly_camera_system assigns to Transform::rotation.
        let rotation = Quat::from_rotation_y(cam.yaw) * Quat::from_rotation_x(cam.pitch);
        let forward = rotation * Vec3::NEG_Z;
        let wanted = (target - from).normalize();

        assert!(
            (forward - wanted).length() < 1e-4,
            "forward {forward:?} should match {wanted:?}"
        );
    }

    #[test]
    fn looking_straight_down_negative_z_is_the_default_orientation() {
        let (yaw, pitch) = yaw_pitch_towards(Vec3::NEG_Z);
        assert!(yaw.abs() < 1e-6, "yaw was {yaw}");
        assert!(pitch.abs() < 1e-6, "pitch was {pitch}");
    }

    #[test]
    fn a_zero_direction_falls_back_to_the_default_orientation() {
        assert_eq!(yaw_pitch_towards(Vec3::ZERO), (0.0, 0.0));
    }

    /// The camera must start inside the volume, close to something.
    ///
    /// Outside it the distance field is switched off -- the skip walk gives up
    /// on its first out-of-grid sample -- so a framed start pays the field's
    /// load cost and gets none of its benefit.
    #[test]
    fn the_start_camera_lands_inside_the_volume_near_geometry() {
        use bevox_core::dense::DenseVolume;
        use bevox_core::material::MaterialId;
        use glam::UVec3;

        let extent = 256u32;
        let mut dense = DenseVolume::new(extent).unwrap();
        for z in 0..extent {
            for x in 0..extent {
                for y in 0..40 {
                    dense.set(UVec3::new(x, y, z), MaterialId(1));
                }
            }
        }
        let tree = dense.into_contree();
        let (eye, target) = start_camera(&tree);

        let hi = extent as f32;
        assert!(
            eye.cmpge(Vec3::ZERO).all() && eye.cmple(Vec3::splat(hi)).all(),
            "the camera starts at {eye:?}, outside the 0..{hi} volume"
        );
        assert!(
            eye.distance(target) < 40.0,
            "the camera starts {} units from what it looks at, which is not close",
            eye.distance(target)
        );
    }

    /// An empty scene has no surface to stand near, and must still not frame
    /// from outside.
    #[test]
    fn an_empty_scene_starts_at_the_centre() {
        let tree = Contree::empty(3);
        let (eye, target) = start_camera(&tree);
        assert_eq!(eye, Vec3::splat(32.0));
        assert_ne!(eye, target, "the camera looks at its own position");
    }
}
