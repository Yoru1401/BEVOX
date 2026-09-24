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
/// against them, are observable. The contact columns are hundredths. Every
/// column is an integer so `Material` stays `Eq`: `friction` 60 is a
/// coefficient of 0.6, `restitution` 80 is 0.8. Friction may exceed 1;
/// restitution above 1 would add energy on every bounce, so it is clamped
/// where it is used.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub struct Material {
    pub color: [u8; 4],
    pub density: u16,
    pub friction: u8,
    pub restitution: u8,
    /// The impact this material survives, as the speed change a contact
    /// imposes on the body that struck it, in voxels a second.
    /// `UNBREAKABLE` survives anything.
    ///
    /// A speed, not an impulse, so that a large body does not shatter under
    /// its own weight: a contact's impulse grows with the mass resting on it,
    /// while the speed it takes away does not. It is derived from an impulse
    /// all the same -- never from a force, which would depend on the tick
    /// rate. `physics::fracture` says why.
    pub strength: u16,
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
/// In voxels a second, as `Material::strength` is. A body resting under
/// gravity changes speed by about 0.15 a tick, and a fall of twenty voxels
/// arrives at about twenty, so forty is "survives a serious fall, breaks when
/// thrown".
pub const DEFAULT_STRENGTH: u16 = 40;

/// A material that never fractures, however hard it is hit.
pub const UNBREAKABLE: u16 = u16::MAX;

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
        let empty =
            Material { color: [0, 0, 0, 0], density: 0, friction: 0, restitution: 0, strength: UNBREAKABLE };
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
                strength: 25,
            })
            .unwrap();
        assert_eq!(table.get(id).friction, 5);
        assert_eq!(table.get(id).restitution, 80);
        assert_eq!(table.get(id).strength, 25);
        assert_eq!(
            table.get(MaterialId::EMPTY).strength,
            UNBREAKABLE,
            "empty space must not be breakable; there is nothing there to break"
        );
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
     })
     .unwrap();
        let b = table
     .push(Material {
         color: [0, 255, 0, 255],
         density: DEFAULT_DENSITY,
         friction: DEFAULT_FRICTION,
         restitution: DEFAULT_RESTITUTION,
         strength: DEFAULT_STRENGTH,
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
            }).is_some());
        }
        assert!(table.push(Material {
            color: [1, 2, 3, 4],
            density: DEFAULT_DENSITY,
            friction: DEFAULT_FRICTION,
            restitution: DEFAULT_RESTITUTION,
            strength: DEFAULT_STRENGTH,
        }).is_none());
    }
}
