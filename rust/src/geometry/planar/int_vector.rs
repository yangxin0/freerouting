//! Port of `geometry/planar/IntVector.java`.

use crate::datastructures::Signum;
use crate::geometry::planar::{FloatPoint, Side};

/// A two-dimensional vector with integer coordinates.
///
/// Coordinates are bounded by `limits::CRIT_INT` (2^25), so products of two
/// coordinates always fit exactly in an `i64`. Where the Java code computes
/// determinants and scalar products in `double`, the port uses exact `i64`
/// arithmetic, which is at least as precise for in-range inputs.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct IntVector {
    pub x: i32,
    pub y: i32,
}

impl IntVector {
    pub const ZERO: IntVector = IntVector { x: 0, y: 0 };

    pub const fn new(x: i32, y: i32) -> Self {
        IntVector { x, y }
    }

    /// Returns true if both coordinates of this vector are 0.
    pub fn is_zero(self) -> bool {
        self.x == 0 && self.y == 0
    }

    pub fn negate(self) -> Self {
        IntVector::new(-self.x, -self.y)
    }

    pub fn is_orthogonal(self) -> bool {
        self.x == 0 || self.y == 0
    }

    pub fn is_diagonal(self) -> bool {
        self.x.abs() == self.y.abs()
    }

    /// The determinant of the matrix consisting of this vector and `other`.
    pub fn determinant(self, other: IntVector) -> i64 {
        self.x as i64 * other.y as i64 - self.y as i64 * other.x as i64
    }

    /// Turns this vector by `factor` times 90 degrees counterclockwise.
    pub fn turn_90_degree(self, factor: i32) -> Self {
        match factor.rem_euclid(4) {
            0 => self,
            1 => IntVector::new(-self.y, self.x),
            2 => IntVector::new(-self.x, -self.y),
            3 => IntVector::new(self.y, -self.x),
            _ => unreachable!(),
        }
    }

    pub fn mirror_at_y_axis(self) -> Self {
        IntVector::new(-self.x, self.y)
    }

    pub fn mirror_at_x_axis(self) -> Self {
        IntVector::new(self.x, -self.y)
    }

    #[allow(clippy::should_implement_trait)]
    pub fn add(self, other: IntVector) -> Self {
        IntVector::new(self.x + other.x, self.y + other.y)
    }

    /// Let L be the line from the zero vector to `other`. Returns which side
    /// of L this vector is on.
    pub fn side_of(self, other: IntVector) -> Side {
        // Java: Side.of((double) other.x * self.y - (double) other.y * self.x).negate()
        Side::of_i64(self.determinant(other)).negate()
    }

    /// The signum of the scalar product of this vector and `other`.
    pub fn projection(self, other: IntVector) -> Signum {
        Signum::of(self.scalar_product(other) as f64)
    }

    pub fn scalar_product(self, other: IntVector) -> i64 {
        self.x as i64 * other.x as i64 + self.y as i64 * other.y as i64
    }

    /// Converts this vector to a [`FloatPoint`].
    pub fn to_float(self) -> FloatPoint {
        FloatPoint::new(self.x as f64, self.y as f64)
    }

    /// Returns an approximation of this vector scaled to length `length`.
    pub fn change_length_approx(self, length: f64) -> Self {
        let new_point = self.to_float().change_size(length);
        let rounded = new_point.round();
        IntVector::new(rounded.x, rounded.y)
    }

    /// Reduces the coordinates by their greatest common divisor, yielding the
    /// normalized direction of this vector (Java: `to_normalized_direction`).
    pub fn to_normalized_direction(self) -> Self {
        let gcd = gcd(self.x.unsigned_abs(), self.y.unsigned_abs()) as i32;
        if gcd > 1 {
            IntVector::new(self.x / gcd, self.y / gcd)
        } else {
            self
        }
    }
}

/// Port of `BigIntAux.binaryGcd` (plain Euclid is fine here).
pub fn gcd(mut a: u32, mut b: u32) -> u32 {
    while b != 0 {
        (a, b) = (b, a % b);
    }
    a
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn determinant_and_side() {
        let a = IntVector::new(1, 0);
        let b = IntVector::new(0, 1);
        assert_eq!(a.determinant(b), 1);
        assert_eq!(b.determinant(a), -1);
        // b is 90° counterclockwise from a: a is on the right of the line 0->b.
        assert_eq!(a.side_of(b), Side::OnTheRight);
        assert_eq!(b.side_of(a), Side::OnTheLeft);
        assert_eq!(a.side_of(a), Side::Collinear);
    }

    #[test]
    fn turn_90() {
        let v = IntVector::new(3, 1);
        assert_eq!(v.turn_90_degree(1), IntVector::new(-1, 3));
        assert_eq!(v.turn_90_degree(2), IntVector::new(-3, -1));
        assert_eq!(v.turn_90_degree(3), IntVector::new(1, -3));
        assert_eq!(v.turn_90_degree(4), v);
        assert_eq!(v.turn_90_degree(-1), v.turn_90_degree(3));
    }

    #[test]
    fn normalized_direction() {
        assert_eq!(
            IntVector::new(6, -4).to_normalized_direction(),
            IntVector::new(3, -2)
        );
        assert_eq!(
            IntVector::new(0, 5).to_normalized_direction(),
            IntVector::new(0, 1)
        );
    }

    #[test]
    fn projection_signum() {
        let a = IntVector::new(2, 1);
        assert_eq!(a.projection(IntVector::new(1, 1)), Signum::Positive);
        assert_eq!(a.projection(IntVector::new(-1, 2)), Signum::Zero);
        assert_eq!(a.projection(IntVector::new(-2, -1)), Signum::Negative);
    }
}
