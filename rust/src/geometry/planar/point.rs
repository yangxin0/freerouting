//! Port of the abstract class `geometry/planar/Point.java` as an enum over
//! the integer and rational implementations.
//!
//! Methods depending on not-yet-ported types (`IntBox`, `IntOctagon`,
//! `Line`) will be added together with those types.

use crate::geometry::planar::{limits, FloatPoint, IntPoint, RationalPoint, Side, Vector};
use num_bigint::BigInt;
use num_traits::{One, Signed, Zero};
use std::cmp::Ordering;

/// A point in the plane: either integer or rational (projective, infinite
/// precision) coordinates.
#[derive(Debug, Clone)]
pub enum Point {
    Int(IntPoint),
    Rational(RationalPoint),
}

impl Point {
    pub const ZERO: Point = Point::Int(IntPoint::ZERO);

    /// Creates a point from integer coordinates, promoting to rational if
    /// they exceed `limits::CRIT_INT` (Java: `Point.get_instance(int, int)`).
    pub fn get_instance(x: i32, y: i32) -> Self {
        let result = IntPoint::new(x, y);
        if x.abs() > limits::CRIT_INT || y.abs() > limits::CRIT_INT {
            Point::Rational(result.into())
        } else {
            Point::Int(result)
        }
    }

    /// Creates a point from projective coordinates (x/z, y/z), collapsing to
    /// an integer point when possible (Java: `Point.get_instance(BigInteger x3)`).
    ///
    /// Note: like `Vector::get_instance_big`, this fixes the Java original's
    /// double `p_x.mod(p_z)` check to test both coordinates.
    pub fn get_instance_big(mut x: BigInt, mut y: BigInt, mut z: BigInt) -> Self {
        if z.is_negative() {
            x = -x;
            y = -y;
            z = -z;
        }
        if !z.is_zero() && (&x % &z).is_zero() && (&y % &z).is_zero() {
            x = &x / &z;
            y = &y / &z;
            z = BigInt::one();
        }
        if z.is_one() {
            let crit = BigInt::from(limits::CRIT_INT);
            if x.abs() <= crit && y.abs() <= crit {
                use num_traits::ToPrimitive;
                return Point::Int(IntPoint::new(x.to_i32().unwrap(), y.to_i32().unwrap()));
            }
        }
        Point::Rational(RationalPoint::new(x, y, z))
    }

    fn to_rational(&self) -> RationalPoint {
        match self {
            Point::Int(p) => (*p).into(),
            Point::Rational(p) => p.clone(),
        }
    }

    pub fn is_infinite(&self) -> bool {
        match self {
            Point::Int(_) => false,
            Point::Rational(p) => p.is_infinite(),
        }
    }

    pub fn to_float(&self) -> FloatPoint {
        match self {
            Point::Int(p) => p.to_float(),
            Point::Rational(p) => p.to_float(),
        }
    }

    /// The translation of this point by `vector`.
    pub fn translate_by(&self, vector: &Vector) -> Self {
        if vector.is_zero() {
            return self.clone();
        }
        match (self, vector) {
            (Point::Int(p), Vector::Int(v)) => Point::Int(p.translate_by(*v)),
            _ => {
                let result = self.to_rational().translate_by(&vector_to_rational(vector));
                Point::Rational(result)
            }
        }
    }

    /// The difference vector of this point and `other` (`self - other`).
    pub fn difference_by(&self, other: &Point) -> Vector {
        match (self, other) {
            (Point::Int(a), Point::Int(b)) => Vector::Int(a.difference_by(*b)),
            _ => Vector::Rational(self.to_rational().difference_by(&other.to_rational())),
        }
    }

    /// Returns `OnTheLeft` if this point is on the left of the line from
    /// `p1` to `p2`, `OnTheRight` if on the right, `Collinear` otherwise.
    pub fn side_of(&self, p1: &Point, p2: &Point) -> Side {
        let v1 = self.difference_by(p1);
        let v2 = p2.difference_by(p1);
        v1.side_of(&v2)
    }

    /// Returns `Greater` if this point has a strictly bigger x coordinate
    /// than `other`, `Equal` if equal, `Less` otherwise.
    pub fn compare_x(&self, other: &Point) -> Ordering {
        match (self, other) {
            (Point::Int(a), Point::Int(b)) => a.x.cmp(&b.x),
            _ => self.to_rational().compare_x(&other.to_rational()),
        }
    }

    /// Returns `Greater` if this point has a strictly bigger y coordinate
    /// than `other`, `Equal` if equal, `Less` otherwise.
    pub fn compare_y(&self, other: &Point) -> Ordering {
        match (self, other) {
            (Point::Int(a), Point::Int(b)) => a.y.cmp(&b.y),
            _ => self.to_rational().compare_y(&other.to_rational()),
        }
    }

    /// Lexicographic comparison: by x, then by y.
    pub fn compare_x_y(&self, other: &Point) -> Ordering {
        self.compare_x(other).then_with(|| self.compare_y(other))
    }

    /// Turns this point by `factor` times 90 degree around `pole`.
    pub fn turn_90_degree(&self, factor: i32, pole: &Point) -> Self {
        let v = self.difference_by(pole).turn_90_degree(factor);
        pole.translate_by(&v)
    }

    /// Mirrors this point at the vertical line through `pole`.
    pub fn mirror_vertical(&self, pole: &Point) -> Self {
        let v = self.difference_by(pole).mirror_at_y_axis();
        pole.translate_by(&v)
    }

    /// Mirrors this point at the horizontal line through `pole`.
    pub fn mirror_horizontal(&self, pole: &Point) -> Self {
        let v = self.difference_by(pole).mirror_at_x_axis();
        pole.translate_by(&v)
    }
}

fn vector_to_rational(vector: &Vector) -> crate::geometry::planar::RationalVector {
    match vector {
        Vector::Int(v) => (*v).into(),
        Vector::Rational(v) => v.clone(),
    }
}

impl From<IntPoint> for Point {
    fn from(p: IntPoint) -> Self {
        Point::Int(p)
    }
}

/// Numeric equality across representations (an integral rational point
/// equals the corresponding int point).
impl PartialEq for Point {
    fn eq(&self, other: &Self) -> bool {
        match (self, other) {
            (Point::Int(a), Point::Int(b)) => a == b,
            _ => self.to_rational() == other.to_rational(),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geometry::planar::RationalVector;

    fn rational(x: i32, y: i32, z: i32) -> Point {
        Point::Rational(RationalPoint::new(
            BigInt::from(x),
            BigInt::from(y),
            BigInt::from(z),
        ))
    }

    #[test]
    fn get_instance_promotes_and_collapses() {
        assert!(matches!(Point::get_instance(5, -3), Point::Int(_)));
        assert!(matches!(
            Point::get_instance(limits::CRIT_INT + 1, 0),
            Point::Rational(_)
        ));
        // (10/2, -4/2) collapses to IntPoint (5, -2).
        let p = Point::get_instance_big(BigInt::from(10), BigInt::from(-4), BigInt::from(2));
        assert!(matches!(p, Point::Int(_)));
        assert_eq!(p, Point::Int(IntPoint::new(5, -2)));
        // (1/2, 0/2) stays rational.
        let q = Point::get_instance_big(BigInt::from(1), BigInt::from(0), BigInt::from(2));
        assert!(matches!(q, Point::Rational(_)));
        // Negative denominator is normalized.
        let r = Point::get_instance_big(BigInt::from(-3), BigInt::from(6), BigInt::from(-3));
        assert_eq!(r, Point::Int(IntPoint::new(1, -2)));
    }

    #[test]
    fn cross_representation_equality() {
        assert_eq!(rational(10, -6, 2), Point::Int(IntPoint::new(5, -3)));
        assert_eq!(rational(1, 2, 4), rational(2, 4, 8));
        assert_ne!(rational(1, 2, 4), rational(1, 2, 3));
    }

    #[test]
    fn translate_and_difference_mixed() {
        let p = Point::Int(IntPoint::new(2, 3));
        let half = Vector::Rational(RationalVector::new(
            BigInt::from(1),
            BigInt::from(1),
            BigInt::from(2),
        ));
        let q = p.translate_by(&half);
        assert_eq!(q, rational(5, 7, 2));
        let diff = q.difference_by(&p);
        assert_eq!(diff, half);
        // Translating back yields the original point.
        assert_eq!(q.translate_by(&half.negate()), p);
    }

    #[test]
    fn side_of_and_compares() {
        let a = Point::Int(IntPoint::new(0, 0));
        let b = Point::Int(IntPoint::new(10, 0));
        assert_eq!(rational(5, 1, 3).side_of(&a, &b), Side::OnTheLeft);
        assert_eq!(rational(5, -1, 3).side_of(&a, &b), Side::OnTheRight);
        assert_eq!(rational(6, 0, 3).side_of(&a, &b), Side::Collinear);

        assert_eq!(
            rational(7, 0, 2).compare_x(&Point::Int(IntPoint::new(3, 0))),
            Ordering::Greater
        );
        assert_eq!(
            rational(6, 5, 2).compare_x_y(&Point::Int(IntPoint::new(3, 2))),
            Ordering::Greater
        );
    }

    #[test]
    fn turns_and_mirrors() {
        let p = Point::Int(IntPoint::new(3, 1));
        let pole = Point::Int(IntPoint::new(1, 1));
        assert_eq!(p.turn_90_degree(1, &pole), Point::Int(IntPoint::new(1, 3)));
        assert_eq!(p.mirror_vertical(&pole), Point::Int(IntPoint::new(-1, 1)));
        assert_eq!(p.mirror_horizontal(&pole), p);
    }

    #[test]
    fn normalized_direction_from_rational() {
        use crate::geometry::planar::direction::Direction;
        use crate::geometry::planar::IntDirection;
        let v = Vector::Rational(RationalVector::new(
            BigInt::from(6),
            BigInt::from(-4),
            BigInt::from(7),
        ));
        assert_eq!(
            v.to_normalized_direction(),
            Direction::Int(IntDirection::from_vector(
                crate::geometry::planar::IntVector::new(3, -2)
            ))
        );
    }
}
