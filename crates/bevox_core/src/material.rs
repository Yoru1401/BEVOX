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

/// A material's renderable properties. Physics columns (density, friction,
/// restitution) are a deferred feature and are deliberately absent.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub struct Material {
    pub color: [u8; 4],
}

#[derive(Clone, Debug)]
pub struct MaterialTable {
    entries: Vec<Material>,
}

impl MaterialTable {
    /// Creates a table whose slot 0 is the reserved empty material.
    pub fn new() -> Self {
        Self { entries: vec![Material { color: [0, 0, 0, 0] }] }
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

    #[test]
    fn the_gpu_palette_is_always_two_hundred_and_fifty_six_entries() {
        let mut table = MaterialTable::new();
        table.push(Material { color: [255, 128, 0, 255] }).unwrap();
        let gpu = table.to_gpu();
        assert_eq!(gpu.len(), 256, "the shader indexes this by a byte");
    }

    #[test]
    fn palette_entries_are_normalised_and_slot_zero_is_transparent() {
        let mut table = MaterialTable::new();
        let id = table.push(Material { color: [255, 128, 0, 255] }).unwrap();
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
        let a = table.push(Material { color: [255, 0, 0, 255] }).unwrap();
        let b = table.push(Material { color: [0, 255, 0, 255] }).unwrap();
        assert_eq!(a, MaterialId(1));
        assert_eq!(b, MaterialId(2));
        assert_eq!(table.get(a).color, [255, 0, 0, 255]);
    }

    #[test]
    fn push_rejects_the_two_hundred_fifty_seventh_material() {
        let mut table = MaterialTable::new();
        for i in 1..=255u16 {
            assert!(table.push(Material { color: [i as u8, 0, 0, 255] }).is_some());
        }
        assert!(table.push(Material { color: [1, 2, 3, 4] }).is_none());
    }
}
