//! Breaking things: which contacts are too hard for the material, and where the
//! cracks go.
//!
//! After Dwyer's devlog 28. The whole of it rests on one observation: a voxel
//! engine already has code that turns disconnected voxels into rigid bodies, so
//! fracture is not a matter of planning pieces and copying volumes. It is a
//! matter of **deleting the right voxels** and letting detachment do what it
//! already does.
//!
//! The threshold is **never on force**, and that part is his. A collision
//! resolves inside one tick, so the same collision at 10 ms a tick and at 5 ms
//! reports twice the force; a force threshold breaks differently at 30 and 60
//! frames a second, which is a physics bug that looks like a gameplay feature.
//! `the_same_collision_breaks_at_any_tick_rate` holds that.
//!
//! **The blow is the accumulated contact impulse**, which is his, and never a
//! quotient of it: dividing by a mass to recover a speed has to choose a mass,
//! and a contact's owner is not it -- a contact belongs to whichever body the
//! scene lists first, so the same collision would read two ways.
//! `a_collision_breaks_the_same_things_whichever_body_is_listed_first` is that
//! gate. The raw impulse is symmetric and needs no owner.
//!
//! This was a closing speed until 2026-10-02, and what killed that is the
//! crush: the mouse grab is a joint with an enormous force limit driving toward
//! a velocity goal, so a grabbed body leaning on a wall presses with up to
//! `GRAB_MAX_FORCE` while the contact holds both surfaces still. The approach
//! speed is then ~0 however hard the press, and a player could lean a rock
//! through a window without marking it.
//! `a_slow_crush_breaks_what_it_presses` is that gate.
//!
//! **What the units cost, measured 2026-10-02 and re-measured across four tick
//! rates the same day.** An impulse grows with the mass a contact holds up, so
//! a stack carries a large one for doing nothing much -- warm starting seeds
//! each tick's impulse from the last. `resting_weight_breaks_nothing` is that
//! gate, and the first attempt at this rule failed it by 61,134 fractures.
//! Every strength is now calibrated above that load, and three things about
//! the window it leaves are open defects rather than solved problems:
//!
//! - **The 331,306 a three-cube glass stack is calibrated against is a
//!   *landing*, not weight.** It peaks at tick 5 -- the fixture starts 0.2
//!   voxels above where it settles, and `peak[at]` counts the bias's push-out.
//!   The load the same stack then holds is 18,798.
//! - **A four-high glass stack destroys itself**, at 356,751 against a
//!   strength of 350,000, on the same landing. `a_four_high_glass_stack_stands`
//!   records it and fails.
//! - **The crush load is proportional to `dt`** -- 365,906 at 64 Hz, 308,430 at
//!   128, 93,512 at 512 -- so for a *held* load this threshold is a force
//!   wearing an impulse's units, which is the one property Dwyer's argument
//!   rejects. Above 64 Hz the crush gate's own case falls under
//!   `GLASS_STRENGTH` and stops breaking. A *collision*'s exchanged momentum is
//!   flat to 1.3% over the same range, so the rule is sound for impacts.
//!
//! `what_the_tick_rate_does_to_the_blow` is the sweep, and
//! `docs/concepts/fracture-load-window.md` has the whole table, what the grab's
//! per-substep impulse clamp has to do with it, and the two candidate fixes.
//! Neither is done.
//!
//! **The threshold is scaled by the size of the body that holds the struck
//! voxel**, which is also his: a chip of glass gives way under a load a sheet of
//! the same glass holds. `size_factor` is the rule and `SIZE_CAP` the volume it
//! stops at. It is normalised to the cap rather than to one voxel, and the
//! narrow window above is why -- the doc comment there has the measurements.
//!
//! **A breaking contact hands back exactly the excess**, which is what it
//! carried past the weaker side's threshold, clamped to what it carried.
//! `hand_back` is that, and it replaced a fixed `REBOUND = 0.6` on 2026-10-02.

use bevox_core::body::BodyId;
use glam::{IVec3, UVec3, Vec3};

/// A contact that carried more than its material could take.
///
/// One side of one contact: a hard landing can raise two, one for the body and
/// one for what it landed on.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Fracture {
    /// Where the contact was, in the world.
    pub at: Vec3,
    /// The voxel that gave way, in the volume `body` names.
    pub voxel: UVec3,
    /// Whose voxel it is. `None` is the static world.
    pub body: Option<BodyId>,
    /// The most the contact carried at any point in the tick, in the solver's
    /// units. The peak rather than what it ended with: a collision that
    /// resolved and let go is back at zero by the end of the tick.
    pub impulse: f32,
    /// What was tested against the material's strength. The solver sets it to
    /// that same peak impulse, so in anything `step` raises the two are equal;
    /// both are kept because `impulse` says what the contact carried and
    /// `blow` says what the rule read, and a rule that stops reading the raw
    /// impulse would part them. The same number whichever side of the contact
    /// is asked about.
    ///
    /// A hand-built `Fracture` -- `main.rs`'s `blow_on` is the one -- may set
    /// them independently, so nothing may *rely* on their being equal.
    pub blow: f32,
    /// How far past that strength it went: 1.0 exactly at the threshold, 3.0
    /// for three times what the material could take. What sizes the cracks.
    pub over: f32,
}

/// The volume, in voxels, past which a body counts as large and the size term
/// stops shrinking its material's strength.
///
/// Dwyer's 2D rule is `sqrt(min(area, 7))`, commented as making small objects
/// break sooner; the 3D analogue of a length from an area is a length from a
/// volume, so the shape of it is `cbrt(min(voxels, SIZE_CAP))`. 27 makes that
/// span a factor of three, the same span his 1 -> 2.65 covers.
///
/// **What ships is that divided by `cbrt(SIZE_CAP)`**, so the factor runs
/// 1/3 -> 1 rather than 1 -> 3 and the span is the only thing the cap sets.
/// `size_factor` is the rule as shipped and its doc has the measurements that
/// forced the normalisation; `docs/map/physics-constants.md` records it the
/// same way.
pub const SIZE_CAP: u32 = 27;

/// What a body of `voxels` voxels multiplies its material's strength by: a
/// third for a single voxel, rising to 1 at `SIZE_CAP` and stopping.
///
/// A chip of glass gives way under a load a window-sized sheet of the same
/// glass holds. Capped because the alternative is terrain -- a mountain of
/// millions of voxels -- being unbreakable by arithmetic rather than by its
/// material.
///
/// **Divided by `cbrt(SIZE_CAP)`, so the factor runs 1/3 -> 1 rather than the
/// 1 -> 3 the plan asks for, and this is a deviation measured into existence
/// rather than chosen.** Every strength in every table is calibrated against
/// bodies at or past the cap -- the fixture cubes are 4x4x4, which is 64
/// voxels -- so a term that runs 1 -> 3 does not leave those numbers alone, it
/// triples them. Two gates then fail, and no value of `SIZE_CAP` above 1 saves
/// them:
///
/// - `a_slow_crush_breaks_what_it_presses` presses the static world's glass
///   floor with a measured 365,906, 4.5% over `GLASS_STRENGTH`. The world
///   saturates at `SIZE_CAP`, so any factor past 1.046 there stops the crush
///   breaking anything.
/// - `a_collision_breaks_the_same_things_whichever_body_is_listed_first` throws
///   an eight-voxel glass pebble, blow 405,054, 15.7% over. Any factor past
///   1.157 on eight voxels stops it, and the raw rule's `cbrt(8)` is 2.
///
/// Normalising puts the calibration point at the cap instead of at one voxel,
/// which is where it was measured. Nothing at or past the cap moves at all;
/// only small bodies get weaker, which is the whole of what Dwyer's comment
/// claims. The window those two bounds leave is 331,306 to 365,906 -- ten per
/// cent wide, documented in `docs/concepts/fracture-load-window.md` as an open
/// defect -- and a multiplicative term spanning three needs three hundred.
pub fn size_factor(voxels: u32) -> f32 {
    (voxels.min(SIZE_CAP) as f32).cbrt() / (SIZE_CAP as f32).cbrt()
}

/// Whether a blow of this impulse breaks that material in a body of `voxels`
/// voxels, and by how much.
///
/// The threshold is the struck voxel's material scaled by the size of the body
/// that holds it, so the same material is tougher in a large body than in a
/// small one. `over` is measured against that scaled threshold, which is what
/// keeps a hit "three times over strength" meaning the same thing whatever the
/// body: it is what sizes the cracks.
///
/// `None` for a material that holds, for `UNBREAKABLE` whatever the blow, and
/// for a non-finite blow: a NaN compares false against everything, so
/// `blow <= threshold` would silently let it through as "did not break" rather
/// than raising the error it actually is.
pub fn over_strength(blow: f32, strength: f32, voxels: u32) -> Option<f32> {
    if !blow.is_finite() || strength == bevox_core::material::UNBREAKABLE {
        return None;
    }
    // A count of zero scales every strength to zero, which makes the body
    // infinitely fragile and `over` an infinity that `reach_of` and `planes_of`
    // then saturate on. Nothing in production reaches here with one -- every
    // constructor calls `mass::recompute` and drops the body when it returns
    // `false` -- but `Body::voxel_count` is 0 until that call, so the ordering
    // is what makes this safe rather than the type.
    debug_assert!(
        voxels > 0 || strength == 0.0,
        "a body with no voxels was tested against a strength of {strength}: its count was \
         never recomputed, so its threshold is zero and any blow at all shatters it"
    );
    // Multiplied, not divided into the blow: `UNBREAKABLE` is an infinity and
    // scaling it stays infinite, so an unbreakable material in a body of any
    // size is still unbreakable. The guard above catches it first; this keeps
    // the arithmetic from depending on that order.
    let threshold = strength * size_factor(voxels);
    if blow <= threshold {
        return None;
    }
    Some(blow / threshold)
}

/// How much of a fracturing contact's impulse is handed back to the bodies:
/// exactly the `excess` it carried past the weaker side's threshold, and never
/// more than it actually `carried`.
///
/// Dwyer's point, and it is what separates fracture that reads as fracture from
/// fracture that reads as a wall: a rock that breaks a window has to carry on
/// through it. Without this the contact stops the rock dead and the pieces fall
/// out of a hole nothing went through. A fixed share of the impulse -- 0.6,
/// until 2026-10-02 -- was a guess at the same thing; the excess is the thing
/// itself. A contact takes what breaking cost and returns the rest.
///
/// **The clamp is not decoration.** The excess is measured over the *weaker*
/// side's threshold while the impulse belongs to the pair, so handing back more
/// than the contact held would add energy to the scene. Today `break_what_gave_
/// way` passes `blow` and `peak[at]`, which are the same number, so the excess
/// is always under it by construction -- but `Fracture`'s own documentation
/// says nothing may rely on `blow` and `impulse` staying equal, and a rule that
/// stopped reading the raw impulse would part them. The clamp is what makes
/// that a change of threshold rather than a change of energy.
/// `the_hand_back_never_exceeds_what_the_contact_carried` is the gate.
pub fn hand_back(excess: f32, carried: f32) -> f32 {
    excess.clamp(0.0, carried.max(0.0))
}

/// Voxels to clear around `at` so that what was solid there comes apart.
///
/// Cracks are the intersections of a few planes through the impact with the
/// ball around it, which is enough to read as shattering and needs no stored
/// pattern. `over` decides how far the damage reaches and how many planes cut
/// it, so a harder hit makes more and smaller pieces.
///
/// Deterministic in `seed`: the same hit twice gives the same cracks, which is
/// what lets a test say anything at all about a random pattern.
pub fn cracks(at: IVec3, over: f32, seed: u64) -> Vec<IVec3> {
    let reach = reach_of(over);
    let planes = planes_of(over);
    let mut rng = Rng::new(seed | 1);
    let normals: Vec<Vec3> = (0..planes).map(|_| rng.direction()).collect();

    let mut out = Vec::new();
    let r = reach as i32;
    for z in -r..=r {
        for y in -r..=r {
            for x in -r..=r {
                let offset = IVec3::new(x, y, z);
                let p = offset.as_vec3();
                if p.length_squared() > (reach * reach) as f32 {
                    continue;
                }
                // Within half a voxel of a plane through the impact is the
                // crack; a full voxel would erase the piece rather than part it.
                if normals.iter().any(|n| n.dot(p).abs() <= 0.5) {
                    out.push(at + offset);
                }
            }
        }
    }
    out
}

/// How far the damage reaches, in voxels, for a hit `over` times the strength.
///
/// Grows with the square root: quadrupling the impulse doubles the radius,
/// which keeps a very hard hit from erasing a whole body.
pub fn reach_of(over: f32) -> u32 {
    (2.0 * over.max(1.0).sqrt()).round().clamp(2.0, 12.0) as u32
}

/// How many planes cut it. More planes, more and smaller pieces.
pub fn planes_of(over: f32) -> u32 {
    (over.max(1.0).sqrt().round() as u32).clamp(2, 6)
}

/// Enough randomness to scatter crack planes, and no more.
///
/// `bevox_core::testing::XorShift64` is the generator; this only adds the one
/// thing cracks need from it.
struct Rng(bevox_core::testing::XorShift64);

impl Rng {
    fn new(seed: u64) -> Self {
        Self(bevox_core::testing::XorShift64::new(seed))
    }

    fn next(&mut self) -> u64 {
        self.0.next_u64()
    }

    /// A unit vector, near enough evenly spread for cracks.
    fn direction(&mut self) -> Vec3 {
        loop {
            let unit = |v: u64| (v % 2001) as f32 / 1000.0 - 1.0;
            let v = Vec3::new(unit(self.next()), unit(self.next()), unit(self.next()));
            let length = v.length();
            if length > 0.1 {
                return v / length;
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use bevox_core::contree::Contree;
    use bevox_core::material::{MaterialId, UNBREAKABLE};
    use crate::detach::loose_pieces;

    /// A cube of solid voxels, which the cracks are cut into.
    fn block(extent: u32) -> Contree {
        let voxels: Vec<_> = (0..extent)
            .flat_map(|z| {
                (0..extent).flat_map(move |y| {
                    (0..extent).map(move |x| (UVec3::new(x, y, z), MaterialId(1)))
                })
            })
            .collect();
        Contree::from_voxels(extent, &voxels)
    }

    /// Below the threshold nothing breaks, above it something does, and no
    /// impulse at all breaks an unbreakable material.
    #[test]
    fn a_material_breaks_only_past_its_strength() {
        // At the cap, where the size term is 1 and the threshold is the
        // material's own strength.
        assert_eq!(over_strength(39.0, 40.0, SIZE_CAP), None);
        assert_eq!(
            over_strength(40.0, 40.0, SIZE_CAP),
            None,
            "exactly at strength is not past it"
        );
        assert_eq!(over_strength(80.0, 40.0, SIZE_CAP), Some(2.0));
        assert_eq!(over_strength(1e30, UNBREAKABLE, 1), None, "unbreakable broke");
        assert_eq!(
            over_strength(1e30, UNBREAKABLE, SIZE_CAP),
            None,
            "unbreakable broke once the size term scaled it"
        );
    }

    /// The same blow against the same material: a single voxel gives way and a
    /// `SIZE_CAP` body does not.
    ///
    /// Dwyer's rule, at the level of the arithmetic. The factor runs 1/3 -> 1
    /// and stops, so a blow between a third of the raw strength and the whole
    /// of it separates the two -- 20 against a strength of 40 below -- and a
    /// bigger body than `SIZE_CAP` is no tougher than one at it.
    #[test]
    fn a_small_body_takes_less_than_a_large_one() {
        assert_eq!(size_factor(SIZE_CAP), 1.0, "a body at the cap is not the unscaled case");
        assert_eq!(size_factor(u32::MAX), 1.0, "the size term is not capped");
        // Zero, which makes a body whose count was never recomputed infinitely
        // fragile rather than merely weak: its threshold is 0 and `over` comes
        // out an infinity. `over_strength` carries a `debug_assert!` against
        // it, and what contains it in release is ordering -- every production
        // constructor calls `mass::recompute` and drops the body when it
        // returns `false`, so no body with a live count of 0 reaches a contact.
        assert_eq!(
            size_factor(0),
            0.0,
            "a body with no voxels should scale to nothing, which is the case              `over_strength`'s debug assertion exists to catch"
        );
        assert!(
            (size_factor(1) - 1.0 / 3.0).abs() < 1e-6,
            "a single voxel scaled by {}, want a third",
            size_factor(1)
        );

        // Half the strength: past a single voxel's third of it, under a capped
        // body's whole.
        assert_eq!(over_strength(20.0, 40.0, SIZE_CAP), None, "a large body broke at half");
        let over = over_strength(20.0, 40.0, 1).expect("a single voxel held at half");
        assert!((over - 1.5).abs() < 1e-5, "a single voxel read {over} times over, want 1.5");
        // `over` is measured against the *scaled* threshold, so "three times
        // over" sizes the cracks the same way whatever the body.
        assert_eq!(over_strength(120.0, 40.0, SIZE_CAP), Some(3.0));
        let over = over_strength(40.0, 40.0, 1).expect("a single voxel held its whole strength");
        assert!(
            (over - 3.0).abs() < 1e-5,
            "a blow at a capped body's threshold read {over} times over on one voxel, want 3"
        );
    }

    /// A NaN blow compares false against everything, so without an explicit
    /// guard `blow <= strength` would read as "did not break" instead of the
    /// error it is.
    #[test]
    fn a_non_finite_blow_never_breaks_anything() {
        assert_eq!(
            over_strength(f32::NAN, 40.0, 1),
            None,
            "a NaN blow slipped through as a break"
        );
        assert_eq!(over_strength(f32::INFINITY, 40.0, 1), None, "an infinite blow slipped through");
        assert_eq!(
            over_strength(f32::NEG_INFINITY, 40.0, 1),
            None,
            "a negative-infinite blow slipped through"
        );
    }

    /// The hand-back is the excess and nothing more.
    ///
    /// The ceiling is what matters: the excess is measured over the weaker
    /// side's threshold while the impulse belongs to the pair, so an unclamped
    /// hand-back can exceed what the contact carried and add energy to the
    /// scene.
    #[test]
    fn the_hand_back_never_exceeds_what_the_contact_carried() {
        assert_eq!(hand_back(400.0, 1000.0), 400.0, "the excess is not handed back whole");
        assert_eq!(
            hand_back(1500.0, 1000.0),
            1000.0,
            "an excess past what the contact carried was handed back in full, which is energy \
             the scene did not have"
        );
        assert_eq!(hand_back(1000.0, 1000.0), 1000.0, "all of it is still allowed");
        assert_eq!(hand_back(-5.0, 1000.0), 0.0, "a negative excess pulled the bodies together");
        assert_eq!(hand_back(400.0, -1.0), 0.0, "a contact that carried nothing handed something back");
    }

    /// Cracks stay inside the reach they claim, and the same seed gives the
    /// same cracks -- without which no test here could say anything.
    #[test]
    fn cracks_are_bounded_and_repeatable() {
        let at = IVec3::new(40, 40, 40);
        let once = cracks(at, 4.0, 7);
        let twice = cracks(at, 4.0, 7);
        assert_eq!(once, twice, "the same hit cracked differently the second time");
        assert!(!once.is_empty(), "a hit four times over strength cracked nothing");
        let reach = reach_of(4.0) as i32;
        for c in &once {
            let d = *c - at;
            assert!(
                d.abs().max_element() <= reach,
                "{c:?} is {} voxels out, past a reach of {reach}",
                d.abs().max_element()
            );
        }
    }

    /// The point of the whole exercise: after the cracks, the block is in
    /// pieces. Counted with the detachment search, which is the same code that
    /// will make the bodies.
    #[test]
    fn cracks_cut_a_solid_block_into_pieces() {
        let pieces = |over: f32, seed: u64| {
            let mut tree = block(16);
            let at = IVec3::splat(8);
            let cut: Vec<UVec3> = cracks(at, over, seed)
                .iter()
                .filter(|p| p.min_element() >= 0 && p.max_element() < 16)
                .map(|p| p.as_uvec3())
                .collect();
            tree.clear_voxels(&cut);
            // Everything still touching the block's floor is one piece; the
            // search counts what came away from the rest.
            loose_pieces(&tree, IVec3::splat(0), IVec3::splat(15), 64).len()
        };

        let hard = pieces(9.0, 11);
        assert!(hard >= 2, "a hit nine times over strength left {hard} pieces");
        let soft = pieces(1.2, 11);
        assert!(
            soft < hard,
            "a hit just over strength left {soft} pieces and a hard one {hard}: the impulse \
             is not reaching the crack pattern"
        );
    }

    /// The reach grows with the hit, but slowly, and it is capped: a body is
    /// meant to come apart, not to evaporate.
    #[test]
    fn a_harder_hit_reaches_further_and_stops_growing() {
        assert!(reach_of(1.0) < reach_of(9.0));
        assert_eq!(reach_of(1e6), 12, "the reach is not capped");
        assert!(planes_of(1.0) < planes_of(36.0));
        assert_eq!(planes_of(1e6), 6, "the plane count is not capped");
    }
}
