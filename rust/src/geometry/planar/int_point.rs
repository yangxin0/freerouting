//! Port of `geometry/planar/IntPoint.java` (core subset).
//!
//! Methods that depend on not-yet-ported types (`IntBox`, `IntOctagon`,
//! `Line`) will be added together with those types.

use crate::geometry::planar::{FloatPoint, IntVector, Side};

/// A point in the plane with integer coordinates.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct IntPoint {
    pub x: i32,
    pub y: i32,
}

impl IntPoint {
    pub const ZERO: IntPoint = IntPoint { x: 0, y: 0 };

    pub const fn new(x: i32, y: i32) -> Self {
        IntPoint { x, y }
    }

    /// Returns the translation of this point by `vector`.
    pub fn translate_by(self, vector: IntVector) -> Self {
        IntPoint::new(self.x + vector.x, self.y + vector.y)
    }

    /// Returns the difference vector of this point and `other`
    /// (i.e. `self - other`).
    pub fn difference_by(self, other: IntPoint) -> IntVector {
        IntVector::new(self.x - other.x, self.y - other.y)
    }

    /// Which side of the directed line from `line_a` to `line_b` this point
    /// lies on.
    pub fn side_of(self, line_a: IntPoint, line_b: IntPoint) -> Side {
        let v1 = self.difference_by(line_a);
        let v2 = line_b.difference_by(line_a);
        v1.side_of(v2)
    }

    pub fn to_float(self) -> FloatPoint {
        FloatPoint::new(self.x as f64, self.y as f64)
    }

    /// The euclidean distance to `other`.
    pub fn distance(self, other: IntPoint) -> f64 {
        self.to_float().distance(other.to_float())
    }

    /// The squared euclidean distance to `other`, exact.
    pub fn distance_square(self, other: IntPoint) -> i64 {
        let dx = (self.x - other.x) as i64;
        let dy = (self.y - other.y) as i64;
        dx * dx + dy * dy
    }

    /// The determinant of the vectors (x, y) and (other.x, other.y).
    pub fn determinant(self, other: IntPoint) -> i64 {
        self.x as i64 * other.y as i64 - self.y as i64 * other.x as i64
    }

    /// The signed area of the parallelogram spanned by the vectors
    /// `p2 - p1` and `self - p1`.
    pub fn signed_area(self, p1: IntPoint, p2: IntPoint) -> i64 {
        p2.difference_by(p1).determinant(self.difference_by(p1))
    }

    /// The nearest point to this point on the horizontal or vertical line
    /// through `other` (snaps this point onto an orthogonal line).
    pub fn orthogonal_projection(self, other: IntPoint) -> IntPoint {
        let horizontal_distance = (self.x - other.x).abs();
        let vertical_distance = (self.y - other.y).abs();
        if horizontal_distance <= vertical_distance {
            // projection onto the vertical line through other
            IntPoint::new(other.x, self.y)
        } else {
            // projection onto the horizontal line through other
            IntPoint::new(self.x, other.y)
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn translate_and_difference() {
        let p = IntPoint::new(2, 3);
        let v = IntVector::new(-5, 7);
        let q = p.translate_by(v);
        assert_eq!(q, IntPoint::new(-3, 10));
        assert_eq!(q.difference_by(p), v);
    }

    #[test]
    fn side_of_segment() {
        let a = IntPoint::new(0, 0);
        let b = IntPoint::new(10, 0);
        assert_eq!(IntPoint::new(5, 1).side_of(a, b), Side::OnTheLeft);
        assert_eq!(IntPoint::new(5, -1).side_of(a, b), Side::OnTheRight);
        assert_eq!(IntPoint::new(20, 0).side_of(a, b), Side::Collinear);
    }

    #[test]
    fn distances() {
        let a = IntPoint::new(0, 0);
        let b = IntPoint::new(3, 4);
        assert_eq!(a.distance_square(b), 25);
        assert!((a.distance(b) - 5.0).abs() < 1e-12);
    }
}
