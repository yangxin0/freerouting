//! Port of `datastructures/BigIntAux.java`.

use num_bigint::BigInt;

/// The determinant of the vectors (x1, y1) and (x2, y2).
pub fn determinant(x1: &BigInt, y1: &BigInt, x2: &BigInt, y2: &BigInt) -> BigInt {
    x1 * y2 - x2 * y1
}

/// Adds two rational coordinate triples (x, y, z) representing
/// (x/z, y/z). Multiplies both denominators when they differ (taking the
/// least common multiple would be optimal, per the Java comment).
pub fn add_rational_coordinates(first: &[BigInt; 3], second: &[BigInt; 3]) -> [BigInt; 3] {
    if first[2] == second[2] {
        // both rational numbers have the same denominator
        [
            &first[0] + &second[0],
            &first[1] + &second[1],
            first[2].clone(),
        ]
    } else {
        [
            &first[0] * &second[2] + &second[0] * &first[2],
            &first[1] * &second[2] + &second[1] * &first[2],
            &first[2] * &second[2],
        ]
    }
}
