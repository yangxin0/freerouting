//! Port of `geometry/planar/FloatPoint.java` (core subset).
//!
//! `FloatPoint` is used for fast approximate geometry; exact results are
//! computed on the integer/rational types.

use crate::geometry::planar::{IntPoint, Side};

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

    /// The point on the ray from this point through `to_point` at distance
    /// `new_length` from this point.
    pub fn change_length(self, to_point: FloatPoint, new_length: f64) -> FloatPoint {
        let dx = to_point.x - self.x;
        let dy = to_point.y - self.y;
        if dx == 0.0 && dy == 0.0 {
            return to_point;
        }
        let length = (dx * dx + dy * dy).sqrt();
        FloatPoint::new(
            self.x + (dx * new_length) / length,
            self.y + (dy * new_length) / length,
        )
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

    #[allow(clippy::should_implement_trait)]
    pub fn add(self, other: FloatPoint) -> Self {
        FloatPoint::new(self.x + other.x, self.y + other.y)
    }

    pub fn substract(self, other: FloatPoint) -> Self {
        FloatPoint::new(self.x - other.x, self.y - other.y)
    }

    /// The scalar product of the vectors from this point to `p1` and from
    /// this point to `p2`.
    pub fn scalar_product(self, p1: FloatPoint, p2: FloatPoint) -> f64 {
        let dx_1 = p1.x - self.x;
        let dx_2 = p2.x - self.x;
        let dy_1 = p1.y - self.y;
        let dy_2 = p2.y - self.y;
        dx_1 * dx_2 + dy_1 * dy_2
    }

    /// Which side of the directed line from `p1` to `p2` this point is on.
    /// Note: unlike `IntVector::side_of`, the Java original computes this
    /// determinant without negation.
    pub fn side_of(self, p1: FloatPoint, p2: FloatPoint) -> Side {
        let d21_x = p2.x - p1.x;
        let d21_y = p2.y - p1.y;
        let d01_x = self.x - p1.x;
        let d01_y = self.y - p1.y;
        Side::of(d21_x * d01_y - d21_y * d01_x)
    }

    /// Rotates this point by `angle` (radians) around `pole`.
    pub fn rotate(self, angle: f64, pole: FloatPoint) -> Self {
        if angle == 0.0 {
            return self;
        }
        let dx = self.x - pole.x;
        let dy = self.y - pole.y;
        let (sin_angle, cos_angle) = angle.sin_cos();
        FloatPoint::new(
            pole.x + dx * cos_angle - dy * sin_angle,
            pole.y + dx * sin_angle + dy * cos_angle,
        )
    }

    /// Turns this point by `factor` times 90 degree around zero.
    pub fn turn_90_degree(self, factor: i32) -> Self {
        match factor.rem_euclid(4) {
            0 => self,
            1 => FloatPoint::new(-self.y, self.x),
            2 => FloatPoint::new(-self.x, -self.y),
            3 => FloatPoint::new(self.y, -self.x),
            _ => unreachable!(),
        }
    }

    /// Turns this point by `factor` times 90 degree around `pole`.
    pub fn turn_90_degree_around(self, factor: i32, pole: FloatPoint) -> Self {
        pole.add(self.substract(pole).turn_90_degree(factor))
    }

    /// Checks if this point is contained in the box spanned by `p1` and `p2`
    /// with the given tolerance.
    pub fn is_contained_in_box(self, p1: FloatPoint, p2: FloatPoint, tolerance: f64) -> bool {
        let (min_x, max_x) = if p1.x < p2.x {
            (p1.x, p2.x)
        } else {
            (p2.x, p1.x)
        };
        if self.x < min_x - tolerance || self.x > max_x + tolerance {
            return false;
        }
        let (min_y, max_y) = if p1.y < p2.y {
            (p1.y, p2.y)
        } else {
            (p2.y, p1.y)
        };
        self.y >= min_y - tolerance && self.y <= max_y + tolerance
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
