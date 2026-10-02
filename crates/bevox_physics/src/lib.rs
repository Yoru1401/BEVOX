//! Rigid-body physics for BEVOX for voxel bodies, after Douglas Dwyer's engine: voxels
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

/// Sweeps of the biased contact/joint pass per substep, the one that pushes
/// penetration out.
///
/// **1 by measurement, not by default.** Measured by
/// `solver::tests::what_the_iteration_counts_cost` (2026-10-01, re-run
/// 2026-10-02): raising this count **while holding `RELAXATION_ITERATIONS` at
/// 1** made a 240:1 mass-ratio load no more stable -- worst upward velocity
/// 12.83 v/s at `(1,1)` against 15.75, 14.81 and 18.02 at `(2,1)`, `(4,1)`
/// and `(8,1)`, and the fixture breached the floor at tick 55, 88, 67 and 55 --
/// for up to 2.6x a tick.
///
/// **What that does not settle.** Those variants change the *ratio* of biased
/// to unbiased sweeps, not the biased count alone: the relax pass exists to
/// remove the velocity this pass's bias added, so an unexcluded explanation is
/// the ratio rather than convergence. Raising both together held the fixture up
/// longer on the two balanced rows measured -- tick 105 at `(2,2)`, 123 at
/// `(4,4)`, against 55 -- but at 1.9x and 3.0x the tick, from two samples on one
/// fixture, and with added numerical damping unexcluded as the reason. 1 is kept
/// because the fixture collapses at every setting measured, because the cheap
/// setting is the one that was measured, and because this count rescales `peak`,
/// which the fracture threshold reads.
///
/// Shallower resting penetration is monotonic in it and is held by
/// `solver::tests::more_iterations_do_not_deepen_a_resting_contact`. Read
/// `docs/concepts/solver-convergence-is-a-setting.md` before raising it.
pub const VELOCITY_ITERATIONS: u32 = 1;

/// Sweeps of the unbiased relax pass per substep, the one that removes the
/// velocity the bias added.
///
/// **1 by measurement, not by default.** This is the count the 2026-10-01
/// design failed to vary: four of its six variants held it at 1 while the
/// biased count rose, so they measured the bias:relax ratio. The two balanced
/// variants -- `(2,2)` and `(4,4)`, the second added on 2026-10-02 -- are the
/// ones that kept the 240:1 load standing longer, to tick 105 and 123 against
/// `(1,1)`'s 55, for 1.9x and 3.0x the tick.
///
/// **So the balanced axis is not a closed question**, and 1 is a cost
/// decision on a fixture that collapses either way, not a measured optimum.
/// Same measurement and same concept page as `VELOCITY_ITERATIONS`.
pub const RELAXATION_ITERATIONS: u32 = 1;

/// How many times the solver sweeps its two passes per substep. Both default
/// to 1, which is what makes `step` and `step_with(.., Tuning::default())`
/// bit for bit identical.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Tuning {
    pub velocity_iterations: u32,
    pub relaxation_iterations: u32,
}

impl Default for Tuning {
    fn default() -> Tuning {
        Tuning { velocity_iterations: VELOCITY_ITERATIONS, relaxation_iterations: RELAXATION_ITERATIONS }
    }
}

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

/// The fastest a body may move, in voxels per **second**: linear speed plus
/// the swing of its furthest voxel. Speeds are capped to it, which is what
/// lets speculative contacts stand in for continuous collision detection.
///
/// A safety net, not a speed limit. It sits above what drag allows -- 256
/// voxels a second against a terminal 200 -- so a falling body never reaches
/// it, and what does reach it is something thrown or blasted, where being
/// clamped beats tunnelling through a wall. The detection margin is the body's
/// *actual* travel, so a ceiling this high costs a slow body nothing.
///
/// Per second, not per tick, which is the whole point: as voxels-per-tick it
/// was a different speed at every tick rate -- 256 at 64 Hz, 512 at 128 --
/// so changing the rate changed how fast anything in the game could go.
pub const MAX_SPEED: f32 = 256.0;

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
    use crate::mass::recompute;
    use bevox_core::body::Body;
    use bevox_core::contree::Contree;
    use bevox_core::material::{Material, MaterialId, MaterialTable};
    use glam::{Quat, UVec3, Vec3};

    /// What the brittle fixture material takes, as the impulse a contact on it
    /// may carry.
    ///
    /// Calibrated by measurement, and the window is narrow because an impulse
    /// threshold has to separate a crush from a lean. It sits above the
    /// 331,306 a three-cube glass stack reaches as it settles onto the floor
    /// (`resting_weight_breaks_nothing`) and above the 190,512 a body arriving
    /// at 8 lands (`the_same_collision_breaks_at_any_tick_rate` requires that
    /// to survive). It sits below the 365,906 a grab presses with at 64 Hz
    /// (`a_slow_crush_breaks_what_it_presses`), the 405,054 of the two-body
    /// collision gate, and the 968,836 that arriving at 60 lands.
    ///
    /// 350,000 is the geometric middle of the binding pair, 331,306 and
    /// 365,906 -- five per cent of headroom either way.
    ///
    /// **The crush half of that window is gone, and this number has not moved
    /// yet.** Since the regime split a press is judged as a force against
    /// `GLASS_CRUSH`, so the 365,906 ceiling no longer applies to `strength`
    /// at all and the crush gate does not care what this is. What still binds
    /// it is landings: the series is 126,129 / 246,227 / 331,306 / 356,751 /
    /// 391,195 / 414,166 for stacks of one to eight, and a four-high stack is
    /// already over this at 356,751, which is why
    /// `a_four_high_glass_stack_stands` is `#[ignore]`d and fails. Raising
    /// this is the next change and the measurements above are what it is
    /// against. `solver::tests::what_the_tick_rate_does_to_the_blow` is the
    /// sweep and `docs/concepts/fracture-load-window.md` is what it means.
    pub const GLASS_STRENGTH: f32 = 350_000.0;

    /// What the brittle fixture material takes as a **force**, while a contact
    /// on it is held rather than struck. `GLASS_STRENGTH` is the impact half
    /// and this is the held half; `fracture::IMPACT_SPEED` chooses between
    /// them.
    ///
    /// **Measured, not chosen**, by `solver::tests::what_a_press_delivers_as_a_
    /// force`, in the solver's own force units. The held branch reads the
    /// **end-of-tick** accumulated impulse, so these are the figures that
    /// matter; the figures by `peak` are in the table below only because the
    /// difference between the two columns is the whole argument for reading
    /// the one it does.
    ///
    /// | at 64 Hz | by `peak` | by `settled` |
    /// |---|---|---|
    /// | a saturated grab pressing | 93,671,928 | 93,669,600 |
    /// | a three-cube stack's largest held force | 35,823,928 | 22,236,248 |
    /// | a four-cube stack's | 52,490,500 | 23,909,612 |
    /// | an eight-cube stack's | 102,018,240 | 76,269,120 |
    ///
    /// **The ceiling is 93,669,600** -- the press at its *weakest* rate, 64
    /// Hz, because the crush gate must break at every rate. As a force the
    /// press is bounded by `GRAB_MAX_FORCE` times the share the heaviest
    /// contact takes, so the column rises toward 1.962e8 with the rate rather
    /// than being flat: 157,853,936 at 128, 181,733,184 at 256, 191,411,232 at
    /// 512.
    ///
    /// **The floor is 23,909,612**, a four-cube stack at 64 Hz -- the rate
    /// `resting_weight_breaks_nothing` runs at, and the height
    /// `a_four_high_glass_stack_stands` needs once the impact strengths are
    /// raised. **47,300,000 is the geometric middle** of that and the ceiling.
    ///
    /// | margin | factor |
    /// |---|---|
    /// | above a four-cube stack's held force (64 Hz) | 1.98x |
    /// | above a three-cube stack's | 2.13x |
    /// | below the press at its weakest rate | 1.98x |
    ///
    /// Against the five per cent `GLASS_STRENGTH` has between 331,306 and
    /// 365,906. The window is **4.21x** wide on the three-cube stack that
    /// binds today, against 1.10x for the impulse threshold this replaced.
    ///
    /// **The eight-cube stack is deliberately not cleared**, and this is the
    /// one calibration decision here worth arguing about. Its 76,269,120 is
    /// above this number, so an eight-high glass stack crushes itself. It is
    /// excluded because it **never settles** -- 0 of 8 bodies asleep, late
    /// speed 4.12 voxels a second, held load swinging 86,716-217,383 -- so
    /// that figure is a diverging pile and not a load. Calibrating a constant
    /// against a configuration that is already diverging bakes a known defect
    /// into the constant, and the non-settling is a *stacking* defect that the
    /// fracture plan explicitly does not fix. An honest 1.98x on a settled
    /// stack beats a 1.23x bought from a broken one.
    /// `an_eight_high_glass_stack_stands` records the cost, `#[ignore]`d and
    /// failing.
    ///
    /// **What this does not fix, and must not be read as fixing.** Reading
    /// `settled` instead of `peak` removes neither the landing residue nor the
    /// rate-dependence:
    ///
    /// - **The floor is still a landing, not weight.** A settled three-cube
    ///   stack holds 18,798 as an impulse at 64 Hz, which is **4,812,288** as
    ///   a force -- 4.6x under the 22,236,248 floor. The floor lands at tick 4
    ///   and sits on the contact between the bottom two cubes, at a closing
    ///   speed of exactly **+0.0000**: they are falling together, so that
    ///   contact genuinely is not closing while it transmits the whole
    ///   landing. No value of `fracture::IMPACT_SPEED` reaches zero. That is a
    ///   limit of the discriminator, not of this number, and
    ///   `docs/concepts/fracture-load-window.md` has it as its own section.
    /// - **The floor still climbs with the rate, by 4.7x** from 64 Hz to 512
    ///   (22,236,248 -> 105,040,984) against 6.8x by `peak`, while the ceiling
    ///   moves 2.04x. So this is calibrated at 64 Hz and is not calibrated at
    ///   512, and nothing in the suite would say so.
    pub const GLASS_CRUSH: f32 = 47_300_000.0;

    /// Material 1 weighs 1000 and grips; 2 weighs 3000 and grips; 3 is
    /// frictionless ice; 4 is bouncy; 5 is nearly elastic; 6 is brittle; 7
    /// never breaks.
    pub fn materials() -> MaterialTable {
        materials_with_glass_crush(GLASS_CRUSH)
    }

    /// The same table with the brittle material's `crush` set to whatever a
    /// test needs, which is what lets one put a threshold *between* two
    /// measured loads and assert which side of it the rule lands on.
    /// `a_landing_is_not_read_as_a_held_load` is the one that needs it.
    pub fn materials_with_glass_crush(crush: f32) -> MaterialTable {
        let mut table = MaterialTable::new();
        table
            .push(Material {
                color: [200, 200, 200, 255],
                density: 1000,
                friction: 60,
                restitution: 0,
                strength: bevox_core::material::DEFAULT_STRENGTH,
                crush: bevox_core::material::DEFAULT_CRUSH,
            })
            .unwrap();
        table
            .push(Material {
                color: [90, 90, 90, 255],
                density: 3000,
                friction: 60,
                restitution: 0,
                strength: bevox_core::material::DEFAULT_STRENGTH,
                crush: bevox_core::material::DEFAULT_CRUSH,
            })
            .unwrap();
        table
            .push(Material {
                color: [170, 210, 235, 255],
                density: 1000,
                friction: 0,
                restitution: 0,
                strength: bevox_core::material::DEFAULT_STRENGTH,
                crush: bevox_core::material::DEFAULT_CRUSH,
            })
            .unwrap();
        table
            .push(Material {
                color: [40, 40, 45, 255],
                density: 1000,
                friction: 60,
                restitution: 80,
                strength: bevox_core::material::DEFAULT_STRENGTH,
                crush: bevox_core::material::DEFAULT_CRUSH,
            })
            .unwrap();
        table
            .push(Material {
                color: [250, 240, 120, 255],
                density: 1000,
                friction: 60,
                restitution: 99,
                strength: bevox_core::material::DEFAULT_STRENGTH,
                crush: bevox_core::material::DEFAULT_CRUSH,
            })
            .unwrap();
        // 6: glass. Breaks under a load a little past what a short stack of
        // it leans with (`GLASS_STRENGTH`), and under a press of
        // `GLASS_CRUSH`. The only fixture material with either: the rest keep
        // `DEFAULT_CRUSH`, so no gate here meets the held branch except the
        // two that are about it.
        table
            .push(Material {
                color: [200, 230, 255, 255],
                density: 1000,
                friction: 60,
                restitution: 0,
                strength: GLASS_STRENGTH,
                crush,
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
                strength: bevox_core::material::UNBREAKABLE,
                crush: bevox_core::material::DEFAULT_CRUSH,
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
                    + b.mass.mass * -crate::GRAVITY.y * b.position.y
            })
            .sum()
    }

    /// A body with its mass properties computed and its centre of mass at
    /// `centre` in the world.
    pub fn placed(volume: Contree, centre: Vec3, orientation: Quat) -> Body {
        let mut body = Body::new(volume, Vec3::ZERO, orientation);
        assert!(recompute(&mut body, &materials()), "a fixture body must have mass");
        body.position = centre;
        body
    }
}
