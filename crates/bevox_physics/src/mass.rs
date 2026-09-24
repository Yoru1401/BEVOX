//! Mass, centre of mass and inertia, from voxel occupancy and material density.

use bevox_core::body::{Body, Features};
use bevox_core::material::{MaterialId, MaterialTable};
use glam::{DMat3, DVec3, UVec3, Vec3};
pub use bevox_core::body::MassProperties;

/// Mass properties of `voxels`, and their centre of mass in volume
/// coordinates. `None` when they weigh nothing.
///
/// Each voxel is a unit cube: its own inertia about its centre, `m / 6` on each
/// axis, is carried to the centre of mass by the parallel-axis theorem. That
/// makes the result exact for any union of voxels rather than an estimate.
/// Accumulated in f64, because a 64-cubed body sums hundreds of thousands of
/// terms.
pub fn mass_properties(
    voxels: &[(UVec3, MaterialId)],
    materials: &MaterialTable,
) -> Option<(MassProperties, Vec3)> {
    let mut mass = 0.0f64;
    let mut moment = DVec3::ZERO;
    for &(p, m) in voxels {
        let rho = materials.get(m).density as f64;
        mass += rho;
        moment += (p.as_dvec3() + 0.5) * rho;
    }
    if mass <= 0.0 {
        return None;
    }
    let com = moment / mass;

    let mut inertia = DMat3::ZERO;
    for &(p, m) in voxels {
        let rho = materials.get(m).density as f64;
        let d = p.as_dvec3() + 0.5 - com;
        let own = DMat3::from_diagonal(DVec3::splat(rho / 6.0));
        let carried =
            (DMat3::from_diagonal(DVec3::splat(d.length_squared())) - outer(d, d)) * rho;
        inertia += own + carried;
    }
    let props =
        MassProperties { mass: mass as f32, inverse_inertia: inertia.inverse().as_mat3() };
    Some((props, com.as_vec3()))
}

/// `a * b^T`.
fn outer(a: DVec3, b: DVec3) -> DMat3 {
    DMat3::from_cols(a * b.x, a * b.y, a * b.z)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::fixtures::{cube, materials};

    /// Summing unit-cube inertias with the parallel-axis theorem is exact for a
    /// block, so this is equality with the continuous formula rather than an
    /// estimate: I = m * n^2 / 6 about each axis.
    #[test]
    fn a_solid_cube_has_the_textbook_inertia() {
        let (props, com) = mass_properties(&cube(4, 4).voxels(), &materials()).unwrap();
        assert_eq!(props.mass, 64_000.0);
        assert_eq!(com, Vec3::splat(2.0));
        let want = 6.0 / (64_000.0 * 16.0);
        let got = props.inverse_inertia;
        for (i, col) in [got.x_axis, got.y_axis, got.z_axis].iter().enumerate() {
            for (j, value) in col.to_array().iter().enumerate() {
                let expected = if i == j { want } else { 0.0 };
                assert!(
                    (value - expected).abs() <= want * 1e-5,
                    "inverse inertia [{i}][{j}] is {value}, want {expected}"
                );
            }
        }
    }

    /// A voxel of material 2 is three times as heavy, so the centre of mass sits
    /// three quarters of the way toward it.
    #[test]
    fn the_centre_of_mass_leans_toward_the_denser_voxel() {
        let voxels =
            [(UVec3::new(0, 0, 0), MaterialId(1)), (UVec3::new(1, 0, 0), MaterialId(2))];
        let (props, com) = mass_properties(&voxels, &materials()).unwrap();
        assert_eq!(props.mass, 4000.0);
        assert!((com - Vec3::new(1.25, 0.5, 0.5)).length() < 1e-6, "centre of mass {com:?}");
    }

    /// Nothing to simulate: no voxels, or voxels that weigh nothing.
    #[test]
    fn weightless_voxels_have_no_mass_properties() {
        assert!(mass_properties(&[], &materials()).is_none());
        let air = [(UVec3::ZERO, MaterialId::EMPTY)];
        assert!(mass_properties(&air, &materials()).is_none());
    }
}

/// Recomputes mass properties after the volume changed, or for the first
/// time. The pivot moves to the new centre of mass and `position` moves
/// with it, so every voxel stays where it was in the world.
///
/// Returns `false`, and leaves the body massless, when nothing in the
/// volume weighs anything: such a body should be removed.
///
/// A free function rather than a method on `Body`: it computes mass
/// properties and features, and those are computed here. `bevox_core`
/// holds what a body *is*; this crate holds what is done to it.
pub fn recompute(body: &mut Body, materials: &MaterialTable) -> bool {
    body.warm.clear();
    let voxels = body.volume.voxels();
    let Some((mass, com)) = mass_properties(&voxels, materials) else {
        body.mass = MassProperties::default();
        body.features = Features::default();
        return false;
    };
    body.position += body.orientation * (com - body.com);
    body.com = com;
    body.mass = mass;
    body.features = super::classify::features(&body.volume, &voxels);
    true
}

/// What `recompute` does to a body, which used to be tested beside the
/// type and belongs beside the function.
#[cfg(test)]
mod recomputed {
    use super::*;
    use bevox_core::contree::Contree;
    use bevox_core::material::MaterialId;
    use glam::{Affine3A, Quat, UVec3, Vec3};

    /// A four-cubed block in a four volume, as the moved tests had beside them.
    fn cube() -> Contree {
        let voxels: Vec<_> = (0..4)
            .flat_map(|z| (0..4).flat_map(move |y| (0..4).map(move |x| (UVec3::new(x, y, z), MaterialId(1)))))
            .collect();
        Contree::from_voxels(4, &voxels)
    }

    /// Recomputing moves the pivot, never the voxels. Milestone 4's brush edits
    /// recompute a body in mid-air and must not make it jump.
    #[test]
    fn recompute_keeps_every_voxel_where_it_was() {
        let materials = crate::fixtures::materials();
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
        assert!(recompute(&mut body, &materials));
        assert_ne!(body.com, Vec3::ZERO, "the pivot did not move, so this proves nothing");
        for (a, b) in before.iter().zip(world(&body, &voxels)) {
            assert!((*a - b).length() < 1e-4, "recompute moved a voxel from {a:?} to {b:?}");
        }

        // Erase the upright of the L. The pivot moves again; the rest stays put.
        let base: Vec<_> = voxels.iter().copied().filter(|(p, _)| p.y == 0).collect();
        let before = world(&body, &base);
        let com = body.com;
        body.volume = Contree::from_voxels(16, &base);
        assert!(recompute(&mut body, &materials));
        assert_ne!(body.com, com, "the edit did not move the pivot, so this proves nothing");
        for (a, b) in before.iter().zip(world(&body, &base)) {
            assert!((*a - b).length() < 1e-4, "an edit moved a voxel from {a:?} to {b:?}");
        }
    }

    #[test]
    fn recompute_classifies_the_voxels() {
        let mut body =
            Body::new(crate::fixtures::cube(4, 4), Vec3::ZERO, Quat::IDENTITY);
        assert!(body.features.corners.is_empty());
        assert!(recompute(&mut body, &crate::fixtures::materials()));
        assert_eq!(body.features.corners.len(), 8);
        assert_eq!(body.features.edges.len(), 24);
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
        let materials = crate::fixtures::materials();
        let mut body = Body::new(
            Contree::from_voxels(16, &voxels),
            Vec3::ZERO,
            Quat::from_euler(glam::EulerRot::XYZ, 0.4, 1.2, -0.3),
        );
        assert!(recompute(&mut body, &materials));
        let omega = Vec3::new(0.7, -1.1, 0.4);
        body.set_angular_velocity(omega);
        let back = body.angular_velocity();
        assert!((back - omega).length() < 1e-4, "{back:?}");
    }

    #[test]
    fn a_body_with_no_voxels_does_not_recompute() {
        let mut body = Body::new(Contree::empty(2), Vec3::ZERO, Quat::IDENTITY);
        assert!(!recompute(&mut body, &crate::fixtures::materials()));
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
}
