//! Port of the `Direction` abstraction over `IntDirection` and
//! `BigIntDirection` (`geometry/planar/BigIntDirection.java` and the
//! dispatch parts of `Direction.java`).

use crate::geometry::planar::rational_vector::sign_to_i32;
use crate::geometry::planar::{IntDirection, RationalVector, Vector};
use num_bigint::BigInt;
use num_traits::{One, Signed, Zero};
use std::cmp::Ordering;

/// A direction as a tuple of infinite precision integers, used when the
/// normalized coordinates do not fit into an `IntDirection`.
#[derive(Debug, Clone, PartialEq, Eq, Hash)]
pub struct BigIntDirection {
    pub x: BigInt,
    pub y: BigInt,
}

impl BigIntDirection {
    pub fn new(x: BigInt, y: BigInt) -> Self {
        BigIntDirection { x, y }
    }

    pub fn is_orthogonal(&self) -> bool {
        self.x.is_zero() || self.y.is_zero()
    }

    pub fn is_diagonal(&self) -> bool {
        self.x.abs() == self.y.abs()
    }

    pub fn get_vector(&self) -> RationalVector {
        RationalVector::new(self.x.clone(), self.y.clone(), BigInt::one())
    }

    pub fn opposite(&self) -> Self {
        BigIntDirection::new(-&self.x, -&self.y)
    }

    /// Angular comparison, same ordering as [`IntDirection`].
    fn cmp_big(&self, other: &BigIntDirection) -> Ordering {
        let y1 = sign_to_i32(&self.y);
        let y2 = sign_to_i32(&other.y);
        let x1 = sign_to_i32(&self.x);
        let x2 = sign_to_i32(&other.x);
        if y1 > 0 {
            if y2 < 0 {
                return Ordering::Less;
            }
            if y2 == 0 {
                return if x2 > 0 {
                    Ordering::Greater
                } else {
                    Ordering::Less
                };
            }
        } else if y1 < 0 {
            if y2 >= 0 {
                return Ordering::Greater;
            }
        } else {
            // y1 == 0
            if x1 > 0 {
                return if y2 != 0 || x2 < 0 {
                    Ordering::Less
                } else {
                    Ordering::Equal
                };
            }
            if y2 > 0 || (y2 == 0 && x2 > 0) {
                return Ordering::Greater;
            }
            if y2 < 0 {
                return Ordering::Less;
            }
            return Ordering::Equal;
        }
        // Same open horizontal half plane: det(other, self) > 0 means self is
        // counterclockwise from other.
        (&other.x * &self.y - &other.y * &self.x).sign_cmp()
    }
}

trait SignCmp {
    fn sign_cmp(&self) -> Ordering;
}

impl SignCmp for BigInt {
    fn sign_cmp(&self) -> Ordering {
        sign_to_i32(self).cmp(&0)
    }
}

impl From<IntDirection> for BigIntDirection {
    fn from(d: IntDirection) -> Self {
        let v = d.get_vector();
        BigIntDirection::new(BigInt::from(v.x), BigInt::from(v.y))
    }
}

/// A direction in the plane: an equivalence class of vectors pointing the
/// same way (Java: abstract class `Direction`).
#[derive(Debug, Clone, PartialEq)]
pub enum Direction {
    Int(IntDirection),
    Big(BigIntDirection),
}

impl Direction {
    pub const NULL: Direction = Direction::Int(IntDirection::NULL);

    /// Creates a direction from a vector (Java: `Direction.get_instance`).
    pub fn from_vector(vector: &Vector) -> Self {
        vector.to_normalized_direction()
    }

    pub fn get_vector(&self) -> Vector {
        match self {
            Direction::Int(d) => Vector::Int(d.get_vector()),
            Direction::Big(d) => Vector::Rational(d.get_vector()),
        }
    }

    pub fn is_orthogonal(&self) -> bool {
        match self {
            Direction::Int(d) => d.is_orthogonal(),
            Direction::Big(d) => d.is_orthogonal(),
        }
    }

    pub fn is_diagonal(&self) -> bool {
        match self {
            Direction::Int(d) => d.is_diagonal(),
            Direction::Big(d) => d.is_diagonal(),
        }
    }

    pub fn is_multiple_of_45_degree(&self) -> bool {
        self.is_orthogonal() || self.is_diagonal()
    }

    pub fn opposite(&self) -> Self {
        match self {
            Direction::Int(d) => Direction::Int(d.opposite()),
            Direction::Big(d) => Direction::Big(d.opposite()),
        }
    }

    /// Turns the direction by `factor` times 45 degree. Like the Java
    /// original, this is not implemented for big directions (returns self
    /// unchanged there).
    pub fn turn_45_degree(&self, factor: i32) -> Self {
        match self {
            Direction::Int(d) => Direction::Int(d.turn_45_degree(factor)),
            Direction::Big(_) => self.clone(),
        }
    }
}

impl From<IntDirection> for Direction {
    fn from(d: IntDirection) -> Self {
        Direction::Int(d)
    }
}

impl PartialOrd for Direction {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        let result = match (self, other) {
            (Direction::Int(a), Direction::Int(b)) => a.cmp(b),
            (a, b) => {
                let big_a = a.to_big();
                let big_b = b.to_big();
                big_a.cmp_big(&big_b)
            }
        };
        Some(result)
    }
}

impl Direction {
    fn to_big(&self) -> BigIntDirection {
        match self {
            Direction::Int(d) => BigIntDirection::from(*d),
            Direction::Big(d) => d.clone(),
        }
    }
}
