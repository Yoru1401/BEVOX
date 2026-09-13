//! Fly camera. The arithmetic lives in free functions so it is testable
//! without a window, an input device or a running app.

use bevy::prelude::*;
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
        if mouse.pressed(MouseButton::Right) {
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
}
