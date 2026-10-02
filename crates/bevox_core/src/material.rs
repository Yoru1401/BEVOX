//! Material identity. Index 0 is reserved for empty space.

/// Index into a [`MaterialTable`].
#[derive(Clone, Copy, PartialEq, Eq, Debug, Default)]
pub struct MaterialId(pub u8);

impl MaterialId {
    /// The absence of a voxel. Never appears in a material table as a real entry.
    pub const EMPTY: MaterialId = MaterialId(0);

    pub fn is_empty(self) -> bool {
        self.0 == 0
    }
}

/// A material's properties: how it is drawn, how heavy it is, and how it
/// behaves on contact.
///
/// Density is relative: only ratios between voxels, and later a joint's force
/// against them, are observable. The contact columns are hundredths, kept as
/// integers because that is the precision any source for them actually has —
/// a palette entry's friction and bounce are picked from a small discrete
/// range, never measured to a fraction of a percent: `friction` 60 is a
/// coefficient of 0.6, `restitution` 80 is 0.8. Friction may exceed 1;
/// restitution above 1 would add energy on every bounce, so it is clamped
/// where it is used.
#[derive(Clone, Copy, PartialEq, Debug)]
pub struct Material {
    pub color: [u8; 4],
    pub density: u16,
    pub friction: u8,
    pub restitution: u8,
    /// The impact this material survives, as the largest normal impulse a
    /// contact on it may carry, in the solver's own units.
    /// `UNBREAKABLE` survives anything.
    ///
    /// An impulse, as Dwyer's devlog 28 has it, and never a force: a collision
    /// resolves inside one tick, so the force it reports depends on the tick
    /// rate and the impulse it exchanges does not.
    ///
    /// Two consequences of the units, and both matter when choosing a number.
    /// An impulse grows with the mass a contact holds up, so **every strength
    /// here is calibrated against resting load** -- a settled stack carries a
    /// large impulse for standing still. And it grows with the striking body's
    /// mass, so this is not a speed a material survives: a bigger body breaks
    /// the same material at a lower speed, which is what an impulse means.
    /// `physics::fracture` has the measurements.
    pub strength: f32,
    /// The largest normal force a contact on this material may hold *while
    /// resting*, in the solver's own units. `UNBREAKABLE` survives anything.
    ///
    /// A force, where `strength` is an impulse, because the two gate
    /// different kinds of contact. `strength` judges an impact: a collision
    /// that resolves inside one tick, where the impulse exchanged does not
    /// depend on the tick rate and the force it implies does. `crush` judges
    /// a contact that is *not* an impact -- a held load, several ticks wide,
    /// where no single tick's impulse means anything but the force the
    /// contact carries tick after tick does. `IMPACT_SPEED` is the
    /// closing-speed threshold that tells the two apart; below it a contact
    /// is read as a crush, at or above it as an impact.
    ///
    /// Read by `physics::solver::break_what_gave_way` on any contact whose
    /// closing speed at detection is under `IMPACT_SPEED`. The demo palette
    /// still sets it to `UNBREAKABLE` throughout, so a crush reaches the
    /// physics fixtures' glass and nothing a player can see; the palette is a
    /// later change.
    pub crush: f32,
}

/// The density a material gets when its source says nothing about mass, such
/// as a MagicaVoxel palette.
pub const DEFAULT_DENSITY: u16 = 1000;

/// What a material's friction is when its source says nothing: like dry stone.
pub const DEFAULT_FRICTION: u8 = 60;

/// Barely bouncy, which is what most solids are.
pub const DEFAULT_RESTITUTION: u8 = 5;

/// What a material takes before it cracks when its source says nothing.
///
/// An impulse, as `Material::strength` is, so it has to be read against a body:
/// a four-voxel cube of ordinary stone weighs 64,000 in these units. Driven
/// into terrain, the peak impulse its contact carries is measured
/// 2026-10-02 as:
///
/// | arrival speed | peak impulse |
/// |---|---|
/// | 60 | 1,047,814 |
/// | 120 | 2,476,651 |
/// | 150 | 2,071,706 |
/// | 200 (terminal) | 3,429,210 |
/// | 256 (`MAX_SPEED`) | 6,096,372 |
///
/// 2.8e6 sits between the 120 row and the 200 row, which keeps the meaning the
/// old closing-speed threshold had: "survives any fall a player builds and
/// breaks when it is thrown or blasted", a terminal speed being 200. Default
/// terrain should be the tough case; what is meant to shatter says so.
/// `physics::solver::tests::default_strength_breaks_at_terminal_speed_and_
/// holds_below_it` is the gate, and the window it pins is 2,476,651 to
/// 3,429,210.
///
/// **The curve is not monotonic**: 150 lands a smaller peak than 120, because
/// where inside the substep loop the contact is first seen moves with the
/// speed. An earlier version of this comment said 2.8e6 "is where that body
/// arrives at about 150", which the measurement falsifies -- at 150 the peak
/// is 2,071,706, under the threshold.
///
/// **This did not move when `GLASS_STRENGTH` rose by 10/7**, and the gate above
/// exists because it nearly did. 4,000,000 is above the 3,429,210 a terminal
/// arrival lands, so default terrain would survive every fall there is and
/// break only at `MAX_SPEED`. Nothing in the suite failed on that value until
/// the gate was written. This constant is calibrated against what a *thrown*
/// body delivers and `GLASS_STRENGTH` against what a *stack* lands with; the
/// two are independent and only the second one changed.
///
/// Well clear of resting load, which is the constraint an impulse threshold
/// has and a speed does not: the worst landing transient any glass stack
/// reaches is a measured 414,166, at eight cubes, so this leaves a factor of
/// nearly seven against the worst of it. That figure is a landing and not a
/// weight -- the load a three-cube stack then holds is 18,798 -- but the
/// transient is the number a strength has to clear, which is why it is the one
/// quoted. `docs/concepts/fracture-load-window.md` has both.
pub const DEFAULT_STRENGTH: f32 = 2_800_000.0;

/// A material that never fractures, however hard it is hit.
pub const UNBREAKABLE: f32 = f32::INFINITY;

/// What a material's `crush` is when its source says nothing: unbreakable.
///
/// A material unbreakable by crush is the conservative default -- a wrong
/// number here would have a player's held rock sink through a wall, where this
/// only means it does not. Every table in this codebase still uses it except
/// the physics fixtures' glass, which has a measured `GLASS_CRUSH`; giving the
/// demo palette a real column is a later change, so a crush reaches the
/// fixture glass and nothing a player can see.
pub const DEFAULT_CRUSH: f32 = UNBREAKABLE;

/// The friction between two surfaces: the geometric mean, so the slipperier one
/// dominates and anything against a frictionless surface slides free.
pub fn combine_friction(a: u8, b: u8) -> f32 {
    (a as f32 * b as f32).sqrt() / 100.0
}

/// The bounce between two surfaces: the larger, so one bouncy surface is
/// enough. Clamped to 1, because more would add energy with every bounce.
pub fn combine_restitution(a: u8, b: u8) -> f32 {
    (a.max(b) as f32 / 100.0).min(1.0)
}

#[derive(Clone, Debug)]
pub struct MaterialTable {
    entries: Vec<Material>,
}

impl MaterialTable {
    /// Creates a table whose slot 0 is the reserved empty material.
    pub fn new() -> Self {
        // Unbreakable rather than zero: there is nothing in an empty voxel to
        // break. Nothing reads it -- empty space raises no contacts -- and if
        // anything ever does, it fails toward no fracture rather than toward
        // one on every touch.
        let empty = Material {
            color: [0, 0, 0, 0],
            density: 0,
            friction: 0,
            restitution: 0,
            strength: UNBREAKABLE,
            crush: UNBREAKABLE,
        };
        Self { entries: vec![empty] }
    }

    /// Appends a material. Returns `None` when all 255 usable slots are taken.
    pub fn push(&mut self, material: Material) -> Option<MaterialId> {
        if self.entries.len() > u8::MAX as usize {
            return None;
        }
        let id = MaterialId(self.entries.len() as u8);
        self.entries.push(material);
        Some(id)
    }

    pub fn get(&self, id: MaterialId) -> Material {
        self.entries[id.0 as usize]
    }

    pub fn len(&self) -> usize {
        self.entries.len()
    }

    pub fn is_empty(&self) -> bool {
        self.entries.len() <= 1
    }

    /// The palette as the shader indexes it: 256 linear RGBA entries, whatever
    /// the table's current length, because a voxel byte can name any slot.
    pub fn to_gpu(&self) -> Vec<[f32; 4]> {
        let mut out = vec![[0.0f32; 4]; 256];
        for (i, m) in self.entries.iter().enumerate().skip(1) {
            out[i] = [
                m.color[0] as f32 / 255.0,
                m.color[1] as f32 / 255.0,
                m.color[2] as f32 / 255.0,
                m.color[3] as f32 / 255.0,
            ];
        }
        out
    }
}

impl Default for MaterialTable {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Both contact columns are hundredths, so 60 is a coefficient of 0.6.
    #[test]
    fn a_material_keeps_its_friction_and_bounce() {
        let mut table = MaterialTable::new();
        let id = table
            .push(Material {
                color: [1, 2, 3, 255],
                density: 900,
                friction: 5,
                restitution: 80,
                strength: 25.0,
                crush: DEFAULT_CRUSH,
            })
            .unwrap();
        assert_eq!(table.get(id).friction, 5);
        assert_eq!(table.get(id).restitution, 80);
        assert_eq!(table.get(id).strength, 25.0);
        assert_eq!(
            table.get(MaterialId::EMPTY).strength,
            UNBREAKABLE,
            "empty space must not be breakable; there is nothing there to break"
        );
    }

    /// `strength` is a float, not hundredths like the contact columns: it is
    /// read directly against an accumulated impulse, which runs to the
    /// millions and has no reason to land on an integer.
    #[test]
    fn a_material_keeps_its_strength_as_a_float() {
        let mut table = MaterialTable::new();
        let id = table
            .push(Material {
                color: [1, 2, 3, 255],
                density: 900,
                friction: DEFAULT_FRICTION,
                restitution: DEFAULT_RESTITUTION,
                strength: 150.5,
                crush: DEFAULT_CRUSH,
            })
            .unwrap();
        assert_eq!(table.get(id).strength, 150.5);
    }

    /// `crush`, unlike `strength`, is a force: nothing reads it yet, but it
    /// has to round-trip through the table like every other column.
    #[test]
    fn a_material_keeps_its_crush_force() {
        let mut table = MaterialTable::new();
        let id = table
            .push(Material {
                color: [1, 2, 3, 255],
                density: 900,
                friction: DEFAULT_FRICTION,
                restitution: DEFAULT_RESTITUTION,
                strength: DEFAULT_STRENGTH,
                crush: 1.5e6,
            })
            .unwrap();
        assert_eq!(table.get(id).crush, 1.5e6);
    }

    /// Friction combines as the geometric mean, so ice against stone is
    /// slippery rather than the average of the two. Restitution takes the
    /// larger, so a bouncy ball bounces off a dead floor.
    #[test]
    fn coefficients_combine_the_way_two_surfaces_do() {
        assert!((combine_friction(100, 100) - 1.0).abs() < 1e-6);
        assert!((combine_friction(4, 100) - 0.2).abs() < 1e-6, "ice against stone");
        assert_eq!(combine_friction(0, 100), 0.0, "frictionless against anything is frictionless");
        assert!((combine_restitution(80, 5) - 0.8).abs() < 1e-6);
        assert_eq!(combine_restitution(0, 0), 0.0);
    }

    /// Density rides along with colour: mass properties read it back by id.
    #[test]
    fn a_material_keeps_its_density() {
        let mut table = MaterialTable::new();
        let id = table
     .push(Material {
         color: [1, 2, 3, 255],
         density: 2600,
         friction: DEFAULT_FRICTION,
         restitution: DEFAULT_RESTITUTION,
         strength: DEFAULT_STRENGTH,
         crush: DEFAULT_CRUSH,
     })
     .unwrap();
        assert_eq!(table.get(id).density, 2600);
        assert_eq!(table.get(MaterialId::EMPTY).density, 0, "empty space must weigh nothing");
    }

    #[test]
    fn the_gpu_palette_is_always_two_hundred_and_fifty_six_entries() {
        let mut table = MaterialTable::new();
        table
            .push(Material {
                color: [255, 128, 0, 255],
                density: DEFAULT_DENSITY,
                friction: DEFAULT_FRICTION,
                restitution: DEFAULT_RESTITUTION,
                strength: DEFAULT_STRENGTH,
                crush: DEFAULT_CRUSH,
            })
            .unwrap();
        let gpu = table.to_gpu();
        assert_eq!(gpu.len(), 256, "the shader indexes this by a byte");
    }

    #[test]
    fn palette_entries_are_normalised_and_slot_zero_is_transparent() {
        let mut table = MaterialTable::new();
        let id = table
     .push(Material {
         color: [255, 128, 0, 255],
         density: DEFAULT_DENSITY,
         friction: DEFAULT_FRICTION,
         restitution: DEFAULT_RESTITUTION,
         strength: DEFAULT_STRENGTH,
         crush: DEFAULT_CRUSH,
     })
     .unwrap();
        let gpu = table.to_gpu();

        assert_eq!(gpu[0], [0.0, 0.0, 0.0, 0.0], "slot 0 is empty space");
        let e = gpu[id.0 as usize];
        assert!((e[0] - 1.0).abs() < 1e-6, "red was {}", e[0]);
        assert!((e[1] - 128.0 / 255.0).abs() < 1e-6, "green was {}", e[1]);
        assert!((e[3] - 1.0).abs() < 1e-6, "alpha was {}", e[3]);
    }

    #[test]
    fn unset_palette_slots_are_zero() {
        let table = MaterialTable::new();
        let gpu = table.to_gpu();
        assert!(gpu.iter().all(|e| *e == [0.0, 0.0, 0.0, 0.0]));
    }

    #[test]
    fn slot_zero_is_empty_and_not_pushable() {
        let table = MaterialTable::new();
        assert_eq!(table.len(), 1);
        assert!(MaterialId::EMPTY.is_empty());
        assert!(table.get(MaterialId::EMPTY).color[3] == 0);
    }

    #[test]
    fn push_returns_sequential_ids() {
        let mut table = MaterialTable::new();
        let a = table
     .push(Material {
         color: [255, 0, 0, 255],
         density: DEFAULT_DENSITY,
         friction: DEFAULT_FRICTION,
         restitution: DEFAULT_RESTITUTION,
         strength: DEFAULT_STRENGTH,
         crush: DEFAULT_CRUSH,
     })
     .unwrap();
        let b = table
     .push(Material {
         color: [0, 255, 0, 255],
         density: DEFAULT_DENSITY,
         friction: DEFAULT_FRICTION,
         restitution: DEFAULT_RESTITUTION,
         strength: DEFAULT_STRENGTH,
         crush: DEFAULT_CRUSH,
     })
     .unwrap();
        assert_eq!(a, MaterialId(1));
        assert_eq!(b, MaterialId(2));
        assert_eq!(table.get(a).color, [255, 0, 0, 255]);
    }

    #[test]
    fn push_rejects_the_two_hundred_fifty_seventh_material() {
        let mut table = MaterialTable::new();
        for i in 1..=255u16 {
            assert!(table.push(Material {
                color: [i as u8, 0, 0, 255],
                density: DEFAULT_DENSITY,
                friction: DEFAULT_FRICTION,
                restitution: DEFAULT_RESTITUTION,
                strength: DEFAULT_STRENGTH,
                crush: DEFAULT_CRUSH,
            }).is_some());
        }
        assert!(table.push(Material {
            color: [1, 2, 3, 4],
            density: DEFAULT_DENSITY,
            friction: DEFAULT_FRICTION,
            restitution: DEFAULT_RESTITUTION,
            strength: DEFAULT_STRENGTH,
            crush: DEFAULT_CRUSH,
        }).is_none());
    }
}
