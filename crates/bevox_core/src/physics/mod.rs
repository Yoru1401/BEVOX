//! Rigid-body physics for voxel bodies, after Douglas Dwyer's engine: voxels
//! classified as corners, edges and faces; rounded-voxel contacts; a temporal
//! Gauss-Seidel solver.
//!
//! See `docs/superpowers/specs/2026-09-15-bevox-rigid-bodies-design.md`, whose
//! Provenance section separates what his devlogs show from what is filled in
//! here.
//!
//! Units are voxels and seconds.

pub mod classify;
pub mod mass;

use glam::Vec3;

/// How long one voxel is, in metres. A tuning knob: gravity is expressed in
/// voxels, so this sets how heavy a fall feels.
pub const VOXEL_METRES: f32 = 0.1;

/// Downward acceleration, in voxels per second squared.
pub const GRAVITY: Vec3 = Vec3::new(0.0, -9.81 / VOXEL_METRES, 0.0);

/// Velocity-solver substeps per tick. Contacts are detected once per tick and
/// reused by every substep.
pub const SUBSTEPS: u32 = 4;

/// Penetration left alone, in voxels, so a resting contact is not pushed out
/// and fallen back into every substep.
pub const SLOP: f32 = 0.02;

/// Fraction of the penetration beyond `SLOP` removed per substep, as velocity.
pub const BIAS: f32 = 0.2;

/// The fastest the bias may push a body out, in voxels per second, so a body
/// buried by an edit floats out rather than being fired out.
pub const MAX_PUSH: f32 = 20.0;

/// The furthest a body may move in one tick, in voxels: linear travel plus the
/// swing of its furthest voxel. Speeds are capped to it, which is what lets
/// speculative contacts stand in for continuous collision detection.
pub const MAX_TRAVEL: f32 = 1.25;

/// Speculative margin every body gets even at rest, in voxels.
pub const BASE_MARGIN: f32 = 0.1;

#[cfg(test)]
// Each fixture is used by the module that needs it; which ones are dead depends
// on which physics modules a build compiles.
#[allow(dead_code)]
pub(crate) mod fixtures {
    use crate::body::Body;
    use crate::contree::Contree;
    use crate::material::{Material, MaterialId, MaterialTable};
    use glam::{Quat, UVec3, Vec3};

    /// Material 1 weighs 1000, material 2 weighs 3000.
    pub fn materials() -> MaterialTable {
        let mut table = MaterialTable::new();
        table.push(Material { color: [200, 200, 200, 255], density: 1000 }).unwrap();
        table.push(Material { color: [90, 90, 90, 255], density: 3000 }).unwrap();
        table
    }

    /// A solid `n`-cubed block of material 1 at the origin of an `extent` volume.
    pub fn cube(n: u32, extent: u32) -> Contree {
        let mut voxels = Vec::new();
        for z in 0..n {
            for y in 0..n {
                for x in 0..n {
                    voxels.push((UVec3::new(x, y, z), MaterialId(1)));
                }
            }
        }
        Contree::from_voxels(extent, &voxels)
    }

    /// A body with its mass properties computed and its centre of mass at
    /// `centre` in the world.
    pub fn placed(volume: Contree, centre: Vec3, orientation: Quat) -> Body {
        let mut body = Body::new(volume, Vec3::ZERO, orientation);
        assert!(body.recompute(&materials()), "a fixture body must have mass");
        body.position = centre;
        body
    }
}
