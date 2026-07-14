//! Port of `geometry/planar/RationalVector.java`.

use crate::datastructures::big_int_aux;
use crate::datastructures::Signum;
use crate::geometry::planar::{FloatPoint, IntVector, Side};
use num_bigint::BigInt;
use num_traits::{One, Signed, Zero};

/// A vector in the plane with projective coordinates (x, y, z) of infinite
/// precision, representing the rational tuple (x/z, y/z). The denominator z
/// is kept non-negative.
#[derive(Debug, Clone)]
pub struct RationalVector {
    pub x: BigInt,
    pub y: BigInt,
    pub z: BigInt,
}

impl RationalVector {
    pub fn new(x: BigInt, y: BigInt, z: BigInt) -> Self {
        if z.is_negative() {
            RationalVector {
                x: -x,
                y: -y,
                z: -z,
            }
        } else {
            RationalVector { x, y, z }
        }
    }

    pub fn is_zero(&self) -> bool {
        self.x.is_zero() && self.y.is_zero()
    }

    pub fn negate(&self) -> Self {
        RationalVector {
            x: -&self.x,
            y: -&self.y,
            z: self.z.clone(),
        }
    }

    pub fn is_orthogonal(&self) -> bool {
        self.x.is_zero() || self.y.is_zero()
    }

    pub fn is_diagonal(&self) -> bool {
        self.x.abs() == self.y.abs()
    }

    /// The determinant of this vector and `other`, ignoring the (positive)
    /// denominators.
    pub fn determinant(&self, other: &RationalVector) -> BigInt {
        &self.x * &other.y - &self.y * &other.x
    }

    /// Same side convention as [`IntVector::side_of`].
    pub fn side_of(&self, other: &RationalVector) -> Side {
        Side::of_sign(sign_to_i32(&self.determinant(other))).negate()
    }

    /// The signum of the scalar product of this vector and `other`
    /// (denominators are positive, so they do not affect the sign).
    pub fn projection(&self, other: &RationalVector) -> Signum {
        Signum::of(sign_to_i32(&(&self.x * &other.x + &self.y * &other.y)) as f64)
    }

    /// An approximation of the scalar product (via float coordinates, like
    /// the Java original).
    pub fn scalar_product(&self, other: &RationalVector) -> f64 {
        let v1 = self.to_float();
        let v2 = other.to_float();
        v1.x * v2.x + v1.y * v2.y
    }

    pub fn to_float(&self) -> FloatPoint {
        let zd = big_to_f64(&self.z);
        FloatPoint::new(big_to_f64(&self.x) / zd, big_to_f64(&self.y) / zd)
    }

    pub fn turn_90_degree(&self, factor: i32) -> Self {
        let (new_x, new_y) = match factor.rem_euclid(4) {
            0 => (self.x.clone(), self.y.clone()),
            1 => (-&self.y, self.x.clone()),
            2 => (-&self.x, -&self.y),
            3 => (self.y.clone(), -&self.x),
            _ => unreachable!(),
        };
        RationalVector::new(new_x, new_y, self.z.clone())
    }

    pub fn mirror_at_y_axis(&self) -> Self {
        RationalVector::new(-&self.x, self.y.clone(), self.z.clone())
    }

    pub fn mirror_at_x_axis(&self) -> Self {
        RationalVector::new(self.x.clone(), -&self.y, self.z.clone())
    }

    pub fn add(&self, other: &RationalVector) -> Self {
        let result = big_int_aux::add_rational_coordinates(
            &[self.x.clone(), self.y.clone(), self.z.clone()],
            &[other.x.clone(), other.y.clone(), other.z.clone()],
        );
        let [x, y, z] = result;
        RationalVector::new(x, y, z)
    }
}

impl From<IntVector> for RationalVector {
    fn from(v: IntVector) -> Self {
        RationalVector {
            x: BigInt::from(v.x),
            y: BigInt::from(v.y),
            z: BigInt::one(),
        }
    }
}

/// Equality of the represented rational tuples (cross-multiplied), not of
/// the raw projective coordinates.
impl PartialEq for RationalVector {
    fn eq(&self, other: &Self) -> bool {
        big_int_aux::determinant(&self.x, &self.z, &other.x, &other.z).is_zero()
            && big_int_aux::determinant(&self.y, &self.z, &other.y, &other.z).is_zero()
    }
}

pub(crate) fn sign_to_i32(value: &BigInt) -> i32 {
    match value.sign() {
        num_bigint::Sign::Plus => 1,
        num_bigint::Sign::Minus => -1,
        num_bigint::Sign::NoSign => 0,
    }
}

pub(crate) fn big_to_f64(value: &BigInt) -> f64 {
    use num_traits::ToPrimitive;
    value.to_f64().unwrap_or(f64::MAX)
}
