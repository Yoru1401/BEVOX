//! Breaking things: which contacts are too hard for the material, and where the
//! cracks go.
//!
//! After Dwyer's devlog 28. The whole of it rests on one observation: a voxel
//! engine already has code that turns disconnected voxels into rigid bodies, so
//! fracture is not a matter of planning pieces and copying volumes. It is a
//! matter of **deleting the right voxels** and letting detachment do what it
//! already does.
//!
//! The threshold is on **impulse, never force**. A collision resolves inside one
//! tick, so the same collision at 10 ms a tick and at 5 ms reports twice the
//! force and the same impulse. A force threshold would break differently at 30
//! and 60 frames a second, which is a physics bug that looks like a gameplay
//! feature. `the_same_collision_breaks_at_any_tick_rate` holds that.

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
    /// What the contact carried, in the solver's units.
    pub impulse: f32,
    /// The speed that took out of the striking body, in voxels a second, which
    /// is what was tested against the material's strength.
    pub blow: f32,
    /// How far past that strength it went: 1.0 exactly at the threshold, 3.0
    /// for three times what the material could take. What sizes the cracks.
    pub over: f32,
}

/// How much of a fracturing contact's impulse is handed back to the bodies.
///
/// Dwyer's point, and it is what separates fracture that reads as fracture from
/// fracture that reads as a wall: a rock that breaks a window has to carry on
/// through it. Without this the contact stops the rock dead and the pieces fall
/// out of a hole nothing went through.
pub const REBOUND: f32 = 0.6;

/// Whether a blow of this speed breaks that material, and by how much.
///
/// `None` for a material that holds, and for `UNBREAKABLE` whatever the blow.
pub fn over_strength(blow: f32, strength: u16) -> Option<f32> {
    if strength == bevox_core::material::UNBREAKABLE || blow <= strength as f32 {
        return None;
    }
    Some(blow / strength as f32)
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
        assert_eq!(over_strength(39.0, 40), None);
        assert_eq!(over_strength(40.0, 40), None, "exactly at strength is not past it");
        assert_eq!(over_strength(80.0, 40), Some(2.0));
        assert_eq!(over_strength(1e30, UNBREAKABLE), None, "unbreakable broke");
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
