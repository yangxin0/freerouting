//! Port of `geometry/planar/Limits.java`.

/// An upper bound (2^25) so that the product of two integers with absolute
/// value at most `CRIT_INT` is contained in the mantissa of a double with some
/// space left for addition.
pub const CRIT_INT: i32 = 33_554_432;

/// The biggest double value (2^53), so that all integers smaller than this
/// value are exactly represented as a double value.
pub const CRIT_DOUBLE: f64 = 9_007_199_254_740_992.0;

pub const SQRT2: f64 = std::f64::consts::SQRT_2;
