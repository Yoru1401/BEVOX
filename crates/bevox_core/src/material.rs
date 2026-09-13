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
