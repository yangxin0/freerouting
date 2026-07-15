//! A deterministic, fast hasher for the routing hot paths (the rustc /
//! Firefox "Fx" multiply-mix). `std`'s default SipHash is seeded per
//! process, so `HashMap` iteration order — and everything downstream of
//! it — varies run to run; routing must be reproducible to be
//! benchmarkable and debuggable. Not DoS-resistant: for internal keys
//! (coordinates, ids) only, never attacker-controlled input.

use std::hash::{BuildHasherDefault, Hasher};

const SEED: u64 = 0x51_7c_c1_b7_27_22_0a_95;

#[derive(Default)]
pub struct FxHasher {
    hash: u64,
}

impl FxHasher {
    #[inline]
    fn add_to_hash(&mut self, i: u64) {
        self.hash = (self.hash.rotate_left(5) ^ i).wrapping_mul(SEED);
    }
}

impl Hasher for FxHasher {
    #[inline]
    fn finish(&self) -> u64 {
        self.hash
    }

    #[inline]
    fn write(&mut self, bytes: &[u8]) {
        for chunk in bytes.chunks(8) {
            let mut buf = [0u8; 8];
            buf[..chunk.len()].copy_from_slice(chunk);
            self.add_to_hash(u64::from_le_bytes(buf));
        }
    }

    #[inline]
    fn write_u8(&mut self, i: u8) {
        self.add_to_hash(i as u64);
    }
    #[inline]
    fn write_u16(&mut self, i: u16) {
        self.add_to_hash(i as u64);
    }
    #[inline]
    fn write_u32(&mut self, i: u32) {
        self.add_to_hash(i as u64);
    }
    #[inline]
    fn write_u64(&mut self, i: u64) {
        self.add_to_hash(i);
    }
    #[inline]
    fn write_usize(&mut self, i: usize) {
        self.add_to_hash(i as u64);
    }
    #[inline]
    fn write_i32(&mut self, i: i32) {
        self.add_to_hash(i as u32 as u64);
    }
    #[inline]
    fn write_i64(&mut self, i: i64) {
        self.add_to_hash(i as u64);
    }
}

pub type FxBuildHasher = BuildHasherDefault<FxHasher>;
pub type FxHashMap<K, V> = std::collections::HashMap<K, V, FxBuildHasher>;
pub type FxHashSet<T> = std::collections::HashSet<T, FxBuildHasher>;

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn deterministic_across_maps() {
        let mut a: FxHashMap<(i32, i32, usize), usize> = FxHashMap::default();
        let mut b: FxHashMap<(i32, i32, usize), usize> = FxHashMap::default();
        for i in 0..1000 {
            a.insert((i, -i, i as usize), i as usize);
            b.insert((i, -i, i as usize), i as usize);
        }
        let ka: Vec<_> = a.keys().copied().collect();
        let kb: Vec<_> = b.keys().copied().collect();
        assert_eq!(ka, kb, "same inserts must iterate identically");
    }
}
