//! Mass, centre of mass and inertia, from voxel occupancy and material density.

use crate::material::{MaterialId, MaterialTable};
use glam::{DMat3, DVec3, Mat3, UVec3, Vec3};

/// How heavy a body is and how it resists turning. The centre of mass it is
/// measured about lives on `Body`, because it also places the body.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MassProperties {
    pub mass: f32,
    /// Inverse inertia tensor about the centre of mass, in the body's own axes.
    pub inverse_inertia: Mat3,
}

impl Default for MassProperties {
    /// No mass: a body that is not simulated.
    fn default() -> Self {
        Self { mass: 0.0, inverse_inertia: Mat3::ZERO }
    }
}

impl MassProperties {
    /// Zero for a body with no mass.
    pub fn inverse_mass(&self) -> f32 {
        if self.mass > 0.0 { 1.0 / self.mass } else { 0.0 }
    }
}

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
    use crate::physics::fixtures::{cube, materials};

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
