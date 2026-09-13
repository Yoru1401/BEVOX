//! Conservative reachability masks. For a ray entering a brick at a given cell
//! and travelling in a given sign octant, the mask holds every cell it could
//! still reach. ANDing it with a node's occupancy mask turns "does this brick
//! contain anything this ray can hit" into two instructions.

use crate::node::{BRICK_EDGE, CHILDREN, child_index};
use glam::Vec3;
use std::sync::OnceLock;

pub const OCTANTS: u32 = 8;
pub const TABLE_LEN: usize = (CHILDREN * OCTANTS) as usize;

/// Builds the full table, indexed by `cell * OCTANTS + octant`.
pub fn build_direction_masks() -> [u64; TABLE_LEN] {
    let mut table = [0u64; TABLE_LEN];
    for cz in 0..BRICK_EDGE {
        for cy in 0..BRICK_EDGE {
            for cx in 0..BRICK_EDGE {
                let cell = child_index(cx, cy, cz);
                for octant in 0..OCTANTS {
                    let neg_x = octant & 0b001 != 0;
                    let neg_y = octant & 0b010 != 0;
                    let neg_z = octant & 0b100 != 0;
                    let mut mask = 0u64;
                    for tz in 0..BRICK_EDGE {
                        for ty in 0..BRICK_EDGE {
                            for tx in 0..BRICK_EDGE {
                                let ok_x = if neg_x { tx <= cx } else { tx >= cx };
                                let ok_y = if neg_y { ty <= cy } else { ty >= cy };
                                let ok_z = if neg_z { tz <= cz } else { tz >= cz };
                                if ok_x && ok_y && ok_z {
                                    mask |= 1u64 << child_index(tx, ty, tz);
                                }
                            }
                        }
                    }
                    table[(cell * OCTANTS + octant) as usize] = mask;
                }
            }
        }
    }
    table
}

fn table() -> &'static [u64; TABLE_LEN] {
    static TABLE: OnceLock<[u64; TABLE_LEN]> = OnceLock::new();
    TABLE.get_or_init(build_direction_masks)
}

/// Cells reachable from `cell` when travelling in `octant`.
pub fn direction_mask(cell: u32, octant: u32) -> u64 {
    debug_assert!(cell < CHILDREN && octant < OCTANTS);
    table()[(cell * OCTANTS + octant) as usize]
}

/// Sign octant of a direction. A zero component counts as positive, which
/// keeps the resulting mask conservative rather than dropping a plane.
pub fn octant_index(dir: Vec3) -> u32 {
    let mut octant = 0;
    if dir.x < 0.0 {
        octant |= 0b001;
    }
    if dir.y < 0.0 {
        octant |= 0b010;
    }
    if dir.z < 0.0 {
        octant |= 0b100;
    }
    octant
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_cell_can_always_reach_itself() {
        let table = build_direction_masks();
        for cell in 0..CHILDREN {
            for octant in 0..OCTANTS {
                let mask = table[(cell * OCTANTS + octant) as usize];
                assert!(mask & (1u64 << cell) != 0, "cell {cell} octant {octant}");
            }
        }
    }

    #[test]
    fn the_all_positive_octant_from_the_origin_cell_reaches_everything() {
        assert_eq!(direction_mask(child_index(0, 0, 0), 0b000), u64::MAX);
    }

    #[test]
    fn the_all_negative_octant_from_the_origin_cell_reaches_only_itself() {
        let mask = direction_mask(child_index(0, 0, 0), 0b111);
        assert_eq!(mask.count_ones(), 1);
        assert_eq!(mask, 1u64 << child_index(0, 0, 0));
    }

    #[test]
    fn the_all_negative_octant_from_the_far_cell_reaches_everything() {
        assert_eq!(direction_mask(child_index(3, 3, 3), 0b111), u64::MAX);
    }

    #[test]
    fn a_mixed_octant_restricts_only_the_axes_it_names() {
        // Start in the middle, x negative, y and z positive.
        let cell = child_index(2, 2, 2);
        let mask = direction_mask(cell, 0b001);
        // Reachable x in 0..=2, y in 2..=3, z in 2..=3 => 3 * 2 * 2 = 12 cells.
        assert_eq!(mask.count_ones(), 12);
        assert!(mask & (1u64 << child_index(0, 3, 3)) != 0);
        assert!(mask & (1u64 << child_index(3, 3, 3)) == 0);
    }

    #[test]
    fn octant_index_reads_the_sign_bits() {
        assert_eq!(octant_index(Vec3::new(1.0, 1.0, 1.0)), 0b000);
        assert_eq!(octant_index(Vec3::new(-1.0, 1.0, 1.0)), 0b001);
        assert_eq!(octant_index(Vec3::new(1.0, -1.0, 1.0)), 0b010);
        assert_eq!(octant_index(Vec3::new(-1.0, -1.0, -1.0)), 0b111);
        // Zero counts as positive, which keeps the mask conservative.
        assert_eq!(octant_index(Vec3::new(0.0, 0.0, 0.0)), 0b000);
    }

    #[test]
    fn the_cached_table_matches_a_fresh_build() {
        let built = build_direction_masks();
        for cell in 0..CHILDREN {
            for octant in 0..OCTANTS {
                assert_eq!(
                    direction_mask(cell, octant),
                    built[(cell * OCTANTS + octant) as usize]
                );
            }
        }
    }
}
