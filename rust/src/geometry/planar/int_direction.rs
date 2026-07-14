//! Port of `geometry/planar/Direction.java` + `IntDirection.java`.
//!
//! A `Direction` is an equivalence class of vectors: two vectors define the
//! same direction if they point the same way. Directions are preferred over
//! angles because direction arithmetic is exact.
//!
//! Unlike the Java code, coordinates are always normalized (divided by their
//! gcd) at construction, so derived equality matches Java's
//! collinear-and-same-sense equivalence. `BigIntDirection` will be ported
//! together with the rational point/vector layer.

use crate::datastructures::Signum;
use crate::geometry::planar::{IntPoint, IntVector, Side};
use std::cmp::Ordering;

/// A direction in the plane with integer coordinates, kept gcd-normalized.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct IntDirection {
    x: i32,
    y: i32,
}

impl IntDirection {
    pub const NULL: IntDirection = IntDirection { x: 0, y: 0 };
    /// East.
    pub const RIGHT: IntDirection = IntDirection { x: 1, y: 0 };
    /// Northeast.
    pub const RIGHT45: IntDirection = IntDirection { x: 1, y: 1 };
    /// North.
    pub const UP: IntDirection = IntDirection { x: 0, y: 1 };
    /// Northwest.
    pub const UP45: IntDirection = IntDirection { x: -1, y: 1 };
    /// West.
    pub const LEFT: IntDirection = IntDirection { x: -1, y: 0 };
    /// Southwest.
    pub const LEFT45: IntDirection = IntDirection { x: -1, y: -1 };
    /// South.
    pub const DOWN: IntDirection = IntDirection { x: 0, y: -1 };
    /// Southeast.
    pub const DOWN45: IntDirection = IntDirection { x: 1, y: -1 };

    /// Creates a direction from a vector (Java: `Direction.get_instance(Vector)`).
    pub fn from_vector(vector: IntVector) -> Self {
        let n = vector.to_normalized_direction();
        IntDirection { x: n.x, y: n.y }
    }

    /// The direction from `from` to `to`; `None` if the points are equal
    /// (Java: `Direction.get_instance(Point, Point)`).
    pub fn from_points(from: IntPoint, to: IntPoint) -> Option<Self> {
        if from == to {
            return None;
        }
        Some(Self::from_vector(to.difference_by(from)))
    }

    /// A direction whose angle with the x-axis is nearly `angle` (radians)
    /// (Java: `Direction.get_instance_approx`).
    pub fn from_angle_approx(angle: f64) -> Self {
        const SCALE_FACTOR: f64 = 10_000.0;
        let x = (angle.cos() * SCALE_FACTOR).round() as i32;
        let y = (angle.sin() * SCALE_FACTOR).round() as i32;
        Self::from_vector(IntVector::new(x, y))
    }

    /// Any vector pointing into this direction.
    pub fn get_vector(self) -> IntVector {
        IntVector::new(self.x, self.y)
    }

    /// True if the direction is horizontal or vertical.
    pub fn is_orthogonal(self) -> bool {
        self.x == 0 || self.y == 0
    }

    /// True if the direction is diagonal.
    pub fn is_diagonal(self) -> bool {
        self.x.abs() == self.y.abs()
    }

    /// True if the direction is orthogonal or diagonal.
    pub fn is_multiple_of_45_degree(self) -> bool {
        self.is_orthogonal() || self.is_diagonal()
    }

    pub fn opposite(self) -> Self {
        IntDirection {
            x: -self.x,
            y: -self.y,
        }
    }

    /// Turns the direction by `factor` times 45 degree.
    pub fn turn_45_degree(self, factor: i32) -> Self {
        let (x, y) = (self.x, self.y);
        let (new_x, new_y) = match factor.rem_euclid(8) {
            0 => (x, y),
            1 => (x - y, x + y),
            2 => (-y, x),
            3 => (-x - y, x - y),
            4 => (-x, -y),
            5 => (y - x, -x - y),
            6 => (y, -x),
            7 => (x + y, y - x),
            _ => unreachable!(),
        };
        // Odd multiples of 45° scale diagonal coordinates by 2 (e.g. (1,1)
        // turned 45° is (0,2)); renormalize to keep equality structural.
        Self::from_vector(IntVector::new(new_x, new_y))
    }

    /// Which side of the line from zero towards `other` this direction is on.
    pub fn side_of(self, other: IntDirection) -> Side {
        self.get_vector().side_of(other.get_vector())
    }

    /// The signum of the scalar product of vectors representing this
    /// direction and `other`.
    pub fn projection(self, other: IntDirection) -> Signum {
        self.get_vector().projection(other.get_vector())
    }

    /// An approximation of the direction in the middle of this direction and
    /// `other`.
    pub fn middle_approx(self, other: IntDirection) -> Self {
        let v1 = self.get_vector().to_float();
        let v2 = other.get_vector().to_float();
        let length1 = v1.size();
        let length2 = v2.size();
        let x = v1.x / length1 + v2.x / length2;
        let y = v1.y / length1 + v2.y / length2;
        const SCALE_FACTOR: f64 = 1000.0;
        Self::from_vector(IntVector::new(
            (x * SCALE_FACTOR).round() as i32,
            (y * SCALE_FACTOR).round() as i32,
        ))
    }

    /// Returns `Greater` if the angle between `p1` and this direction is
    /// bigger than the angle between `p2` and this direction, `Equal` if `p1`
    /// equals `p2`, and `Less` otherwise (Java: `compare_from`).
    pub fn compare_from(self, p1: IntDirection, p2: IntDirection) -> Ordering {
        if p1 >= self {
            if p2 >= self {
                p1.cmp(&p2)
            } else {
                Ordering::Less
            }
        } else if p2 >= self {
            Ordering::Greater
        } else {
            p1.cmp(&p2)
        }
    }

    /// An approximation of the signed angle (radians) of this direction.
    pub fn angle_approx(self) -> f64 {
        (self.y as f64).atan2(self.x as f64)
    }
}

/// Orders directions by their angle with the positive x-axis
/// (counterclockwise, starting at `RIGHT`), like Java's `compareTo`.
impl Ord for IntDirection {
    fn cmp(&self, other: &Self) -> Ordering {
        // Angles are ordered in [0, 2π) starting at RIGHT, counterclockwise:
        // the upper half plane sorts before the lower half plane.
        if self.y > 0 {
            if other.y < 0 {
                return Ordering::Less;
            }
            if other.y == 0 {
                return if other.x > 0 {
                    Ordering::Greater
                } else {
                    Ordering::Less
                };
            }
        } else if self.y < 0 {
            if other.y >= 0 {
                return Ordering::Greater;
            }
        } else {
            // self.y == 0
            if self.x > 0 {
                return if other.y != 0 || other.x < 0 {
                    Ordering::Less
                } else {
                    Ordering::Equal
                };
            }
            // self.x < 0 (or NULL, which sorts with LEFT)
            if other.y > 0 || (other.y == 0 && other.x > 0) {
                return Ordering::Greater;
            }
            if other.y < 0 {
                return Ordering::Less;
            }
            return Ordering::Equal;
        }
        // Both in the same open horizontal half plane: compare by determinant.
        // det(other, self) > 0 means self is counterclockwise from other.
        other
            .get_vector()
            .determinant(self.get_vector())
            .cmp(&0)
    }
}

impl PartialOrd for IntDirection {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn normalization_at_construction() {
        assert_eq!(
            IntDirection::from_vector(IntVector::new(0, 7)),
            IntDirection::UP
        );
        assert_eq!(
            IntDirection::from_vector(IntVector::new(-4, -4)),
            IntDirection::LEFT45
        );
    }

    #[test]
    fn turn_45_cycle() {
        let mut d = IntDirection::RIGHT;
        let expected = [
            IntDirection::RIGHT45,
            IntDirection::UP,
            IntDirection::UP45,
            IntDirection::LEFT,
            IntDirection::LEFT45,
            IntDirection::DOWN,
            IntDirection::DOWN45,
            IntDirection::RIGHT,
        ];
        for e in expected {
            d = d.turn_45_degree(1);
            assert_eq!(d, e);
        }
        assert_eq!(IntDirection::UP.turn_45_degree(-2), IntDirection::RIGHT);
        assert_eq!(IntDirection::UP.turn_45_degree(4), IntDirection::DOWN);
    }

    #[test]
    fn angular_ordering() {
        let ccw = [
            IntDirection::RIGHT,
            IntDirection::RIGHT45,
            IntDirection::UP,
            IntDirection::UP45,
            IntDirection::LEFT,
            IntDirection::LEFT45,
            IntDirection::DOWN,
            IntDirection::DOWN45,
        ];
        for w in ccw.windows(2) {
            assert!(w[0] < w[1], "{:?} should sort before {:?}", w[0], w[1]);
        }
        assert_eq!(
            IntDirection::UP.cmp(&IntDirection::from_vector(IntVector::new(0, 3))),
            Ordering::Equal
        );
    }

    #[test]
    fn from_points_and_opposite() {
        let d = IntDirection::from_points(IntPoint::new(1, 1), IntPoint::new(5, 5)).unwrap();
        assert_eq!(d, IntDirection::RIGHT45);
        assert_eq!(d.opposite(), IntDirection::LEFT45);
        assert_eq!(
            IntDirection::from_points(IntPoint::new(1, 1), IntPoint::new(1, 1)),
            None
        );
    }

    #[test]
    fn from_angle_and_angle_approx() {
        let d = IntDirection::from_angle_approx(std::f64::consts::FRAC_PI_2);
        assert_eq!(d, IntDirection::UP);
        assert!((IntDirection::LEFT.angle_approx() - std::f64::consts::PI).abs() < 1e-12);
    }

    #[test]
    fn middle_and_compare_from() {
        let m = IntDirection::RIGHT.middle_approx(IntDirection::UP);
        assert_eq!(m, IntDirection::RIGHT45);
        // Seen from RIGHT45, UP has a smaller angle than DOWN.
        assert_eq!(
            IntDirection::RIGHT45.compare_from(IntDirection::DOWN, IntDirection::UP),
            Ordering::Greater
        );
    }
}
