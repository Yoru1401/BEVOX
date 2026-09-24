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
pub mod contact;
pub mod detach;
pub mod fracture;
pub mod joint;
pub mod mass;
pub mod merge;
pub mod sculpt;
pub mod sleep;
pub mod solver;

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

/// What a body moves through when nothing is touching it.
///
/// Gravity was already a parameter of `step` rather than a constant, so a test
/// can switch it off and watch one thing at a time. Drag is the same kind of
/// thing and is passed the same way -- which is also what keeps
/// `momentum_is_conserved_exactly_in_free_flight` exact: drag is an external
/// force and would leak momentum out of a gate that exists to prove none
/// leaks. The solver's invariants are tested in `Air::STILL` and `Air::VACUUM`;
/// drag has its own gate.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Air {
    pub gravity: Vec3,
    /// Quadratic, per voxel: the acceleration is `-drag * v * |v|`.
    pub drag: f32,
}

impl Air {
    /// What the game runs in.
    pub const EARTH: Air = Air { gravity: GRAVITY, drag: DRAG };

    /// Gravity with nothing to resist it, for anything whose closed form is
    /// being checked.
    pub const VACUUM: Air = Air { gravity: GRAVITY, drag: 0.0 };

    /// Neither, for watching a collision on its own.
    pub const STILL: Air = Air { gravity: Vec3::ZERO, drag: 0.0 };

    /// A named gravity, still with no drag.
    pub const fn vacuum(gravity: Vec3) -> Air {
        Air { gravity, drag: 0.0 }
    }
}

/// How fast a long fall settles at, in voxels a second: 20 metres a second.
///
/// Drag is what a body is limited by, not a clamp, so it eases into this
/// rather than hitting it. It takes about two seconds and two hundred voxels
/// to get near, which covers any drop worth building.
pub const TERMINAL_SPEED: f32 = 200.0;

/// Quadratic air resistance, per voxel: `a = -DRAG * v * |v|`.
///
/// Set so that drag balances gravity exactly at `TERMINAL_SPEED`. A falling
/// body approaches that speed and never reaches it, which is what falling
/// does; a hard clamp instead stops the acceleration dead in one tick, and
/// that is visible.
pub const DRAG: f32 = -GRAVITY.y / (TERMINAL_SPEED * TERMINAL_SPEED);

/// The furthest a body may move in one tick, in voxels: linear travel plus the
/// swing of its furthest voxel. Speeds are capped to it, which is what lets
/// speculative contacts stand in for continuous collision detection.
///
/// A safety net, not a speed limit. It sits above what drag allows -- 260
/// voxels a second against a terminal 200 -- so a falling body never reaches
/// it, and what does reach it is something thrown or blasted, where being
/// clamped beats tunnelling through a wall. The detection margin is the body's
/// *actual* travel, so a cap this high costs a slow body nothing.
pub const MAX_TRAVEL: f32 = 4.0;

/// How strongly a contact resists a body rolling on it, as a fraction of what
/// it resists sliding with.
///
/// Sliding friction cannot stop a roll. A rolling sphere has almost no slip
/// where it touches, so friction finds nothing to act against, and a lone
/// voxel -- which classifies as a `Corner`, a sphere of radius 0.5 -- rolls
/// across a flat floor for as long as you watch it and never falls asleep.
/// Measured before this existed: nudged at 4 voxels a second, still moving at
/// 3.3 after eight seconds.
///
/// Small, because it is a real force and not a cure: it must not stop a cube
/// tipping onto its face, which is the same angular motion at the same kind of
/// contact.
pub const ROLLING: f32 = 0.08;

/// Sweeps of the restitution pass. One is not enough: every contact of a flat
/// landing would push the whole body to the bounce target by itself.
pub const RESTITUTION_SWEEPS: u32 = 4;

/// The most voxels a detachment search will walk before giving up.
///
/// A cut into a mountainside would otherwise walk the mountain. Giving up calls
/// the piece grounded, so a piece larger than this stays in the world rather
/// than becoming a body that could not be afforded anyway.
pub const BUDGET: usize = 20_000;

/// The most force the mouse grab pulls with: the weight of 2,000 voxels of
/// density 1000. A body heavier than that sags and drags rather than lifts.
pub const GRAB_MAX_FORCE: f32 = 2_000.0 * 1000.0 * 9.81 / VOXEL_METRES;

/// The most torque the grab turns a body with: its force at two voxels.
/// Enough to hold the demo cube level by a corner, and weak enough that a grab
/// more than two voxels from a hinge swings the door rather than locking it.
pub const GRAB_MAX_TORQUE: f32 = GRAB_MAX_FORCE * 2.0;

/// Below this speed, in voxels per second, a body counts as still: its centre
/// of mass, and its farthest voxel as it turns.
pub const SLEEP_SPEED: f32 = 0.05;

/// How long, in seconds, a body must stay still before it falls asleep.
pub const SLEEP_AFTER: f32 = 0.5;

/// How long, in seconds, a body must sleep before it may merge back into the
/// world, out of view. Long enough that a pile a body just landed on has
/// settled, short enough that debris clears while the player looks elsewhere.
pub const MERGE_AFTER: f32 = 3.0;

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

    /// Material 1 weighs 1000 and grips; 2 weighs 3000 and grips; 3 is
    /// frictionless ice; 4 is bouncy; 5 is nearly elastic; 6 is brittle; 7
    /// never breaks.
    pub fn materials() -> MaterialTable {
        let mut table = MaterialTable::new();
        table
            .push(Material {
                color: [200, 200, 200, 255],
                density: 1000,
                friction: 60,
                restitution: 0,
                strength: crate::material::DEFAULT_STRENGTH,
            })
            .unwrap();
        table
            .push(Material {
                color: [90, 90, 90, 255],
                density: 3000,
                friction: 60,
                restitution: 0,
                strength: crate::material::DEFAULT_STRENGTH,
            })
            .unwrap();
        table
            .push(Material {
                color: [170, 210, 235, 255],
                density: 1000,
                friction: 0,
                restitution: 0,
                strength: crate::material::DEFAULT_STRENGTH,
            })
            .unwrap();
        table
            .push(Material {
                color: [40, 40, 45, 255],
                density: 1000,
                friction: 60,
                restitution: 80,
                strength: crate::material::DEFAULT_STRENGTH,
            })
            .unwrap();
        table
            .push(Material {
                color: [250, 240, 120, 255],
                density: 1000,
                friction: 60,
                restitution: 99,
                strength: crate::material::DEFAULT_STRENGTH,
            })
            .unwrap();
        // 6: glass. Breaks at a speed a body reaches falling a voxel or two.
        table
            .push(Material {
                color: [200, 230, 255, 255],
                density: 1000,
                friction: 60,
                restitution: 0,
                strength: 5,
            })
            .unwrap();
        // 7: the same in every way except that it never breaks, so a test can
        // change one thing about a collision and nothing else.
        table
            .push(Material {
                color: [200, 230, 255, 255],
                density: 1000,
                friction: 60,
                restitution: 0,
                strength: crate::material::UNBREAKABLE,
            })
            .unwrap();
        table
    }

    /// A solid `n`-cubed block of material 1 at the origin of an `extent` volume.
    pub fn cube(n: u32, extent: u32) -> Contree {
        cube_of(n, extent, MaterialId(1))
    }

    /// The same, of whichever material is wanted.
    pub fn cube_of(n: u32, extent: u32, material: MaterialId) -> Contree {
        let mut voxels = Vec::new();
        for z in 0..n {
            for y in 0..n {
                for x in 0..n {
                    voxels.push((UVec3::new(x, y, z), material));
                }
            }
        }
        Contree::from_voxels(extent, &voxels)
    }

    /// A world whose layers `ys` are solid material 1 across the whole volume.
    pub fn slab(extent: u32, ys: std::ops::Range<u32>) -> Contree {
        slab_of(extent, ys, MaterialId(1))
    }

    /// The same, of whichever material is wanted.
    pub fn slab_of(extent: u32, ys: std::ops::Range<u32>, material: MaterialId) -> Contree {
        let mut voxels = Vec::new();
        for z in 0..extent {
            for y in ys.clone() {
                for x in 0..extent {
                    voxels.push((UVec3::new(x, y, z), material));
                }
            }
        }
        Contree::from_voxels(extent, &voxels)
    }

    /// Total mechanical energy: motion, spin, and height in gravity.
    pub fn energy(bodies: &[Body]) -> f32 {
        bodies
            .iter()
            .map(|b| {
                0.5 * b.mass.mass * b.velocity.length_squared()
                    + 0.5 * b.angular_velocity().dot(b.angular_momentum)
                    + b.mass.mass * -crate::physics::GRAVITY.y * b.position.y
            })
            .sum()
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
