//! Port of `geometry/planar/FloatPoint.java` (core subset).
//!
//! `FloatPoint` is used for fast approximate geometry; exact results are
//! computed on the integer/rational types.

use crate::geometry::planar::IntPoint;

/// A point in the plane with `f64` coordinates.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct FloatPoint {
    pub x: f64,
    pub y: f64,
}

impl FloatPoint {
    pub const ZERO: FloatPoint = FloatPoint { x: 0.0, y: 0.0 };

    pub const fn new(x: f64, y: f64) -> Self {
        FloatPoint { x, y }
    }

    /// The square of the distance from this point to the zero point.
    pub fn size_square(self) -> f64 {
        self.x * self.x + self.y * self.y
    }

    /// The distance from this point to the zero point.
    pub fn size(self) -> f64 {
        self.size_square().sqrt()
    }

    pub fn distance_square(self, other: FloatPoint) -> f64 {
        let dx = other.x - self.x;
        let dy = other.y - self.y;
        dx * dx + dy * dy
    }

    pub fn distance(self, other: FloatPoint) -> f64 {
        self.distance_square(other).sqrt()
    }

    /// Rounds this point to the nearest [`IntPoint`].
    pub fn round(self) -> IntPoint {
        IntPoint::new(self.x.round() as i32, self.y.round() as i32)
    }

    /// Scales this point (as a vector from zero) to size `new_size`.
    /// Returns the point unchanged if it is the zero point.
    pub fn change_size(self, new_size: f64) -> Self {
        if self.x == 0.0 && self.y == 0.0 {
            return self;
        }
        let factor = new_size / self.size();
        FloatPoint::new(self.x * factor, self.y * factor)
    }

    /// The point in the middle between this point and `other`.
    pub fn middle_point(self, other: FloatPoint) -> Self {
        FloatPoint::new((self.x + other.x) / 2.0, (self.y + other.y) / 2.0)
    }

    /// The determinant of this point and `other` interpreted as vectors from
    /// zero.
    pub fn determinant(self, other: FloatPoint) -> f64 {
        self.x * other.y - self.y * other.x
    }

    pub fn scalar_product(self, other: FloatPoint) -> f64 {
        self.x * other.x + self.y * other.y
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn size_and_change_size() {
        let p = FloatPoint::new(3.0, 4.0);
        assert!((p.size() - 5.0).abs() < 1e-12);
        let q = p.change_size(10.0);
        assert!((q.x - 6.0).abs() < 1e-12);
        assert!((q.y - 8.0).abs() < 1e-12);
        assert_eq!(FloatPoint::ZERO.change_size(5.0), FloatPoint::ZERO);
    }

    #[test]
    fn round_to_int_point() {
        assert_eq!(FloatPoint::new(1.4, -2.6).round(), IntPoint::new(1, -3));
    }

    #[test]
    fn middle_point() {
        let m = FloatPoint::new(0.0, 0.0).middle_point(FloatPoint::new(4.0, 6.0));
        assert_eq!(m, FloatPoint::new(2.0, 3.0));
    }
}
