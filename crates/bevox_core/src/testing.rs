//! Seeded randomness for tests. No dependency, reproducible across runs.

pub struct XorShift64(u64);

impl XorShift64 {
    /// `seed` must be non-zero; zero is replaced to keep the generator alive.
    pub fn new(seed: u64) -> Self {
        Self(if seed == 0 { 0x9E37_79B9_7F4A_7C15 } else { seed })
    }

    pub fn next_u64(&mut self) -> u64 {
        let mut x = self.0;
        x ^= x << 13;
        x ^= x >> 7;
        x ^= x << 17;
        self.0 = x;
        x
    }

    /// Uniform enough for test data. `bound` must be non-zero.
    pub fn next_below(&mut self, bound: u32) -> u32 {
        (self.next_u64() % bound as u64) as u32
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_same_seed_produces_the_same_sequence() {
        let mut a = XorShift64::new(12345);
        let mut b = XorShift64::new(12345);
        for _ in 0..64 {
            assert_eq!(a.next_u64(), b.next_u64());
        }
    }

    #[test]
    fn a_zero_seed_still_generates() {
        let mut rng = XorShift64::new(0);
        assert_ne!(rng.next_u64(), 0);
    }

    #[test]
    fn next_below_stays_in_range() {
        let mut rng = XorShift64::new(7);
        for _ in 0..1000 {
            assert!(rng.next_below(5) < 5);
        }
    }
}
