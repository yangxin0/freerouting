//! Port of `geometry/planar/RationalPoint.java` (core subset).
//!
//! A point in the projective plane represented by 3 infinite-precision
//! coordinates (x, y, z); the affine point is (x/z, y/z). Points with z = 0
//! form the line at infinity. Methods depending on `IntBox`/`IntOctagon`/
//! `Line` are deferred until those types are ported.

use crate::datastructures::big_int_aux;
use crate::geometry::planar::rational_vector::big_to_f64;
use crate::geometry::planar::{FloatPoint, IntPoint, RationalVector};
use num_bigint::BigInt;
use num_traits::{One, Signed, Zero};
use std::cmp::Ordering;

#[derive(Debug, Clone)]
pub struct RationalPoint {
    pub x: BigInt,
    pub y: BigInt,
    pub z: BigInt,
}

impl RationalPoint {
    /// Creates a point from projective coordinates. The denominator `z` must
    /// be non-negative (Java throws IllegalArgumentException).
    pub fn new(x: BigInt, y: BigInt, z: BigInt) -> Self {
        assert!(
            !z.is_negative(),
            "RationalPoint: z is expected to be >= 0"
        );
        RationalPoint { x, y, z }
    }

    pub fn is_infinite(&self) -> bool {
        self.z.is_zero()
    }

    pub fn to_float(&self) -> FloatPoint {
        if self.z.is_zero() {
            // Java uses Float.MAX_VALUE for points at infinity.
            return FloatPoint::new(f32::MAX as f64, f32::MAX as f64);
        }
        let zd = big_to_f64(&self.z);
        FloatPoint::new(big_to_f64(&self.x) / zd, big_to_f64(&self.y) / zd)
    }

    /// The translation of this point by `vector`.
    pub fn translate_by(&self, vector: &RationalVector) -> Self {
        let [x, y, z] = big_int_aux::add_rational_coordinates(
            &[self.x.clone(), self.y.clone(), self.z.clone()],
            &[vector.x.clone(), vector.y.clone(), vector.z.clone()],
        );
        RationalPoint::new(x, y, z)
    }

    /// The difference vector of this point and `other` (`self - other`).
    pub fn difference_by(&self, other: &RationalPoint) -> RationalVector {
        let [x, y, z] = big_int_aux::add_rational_coordinates(
            &[self.x.clone(), self.y.clone(), self.z.clone()],
            &[-&other.x, -&other.y, other.z.clone()],
        );
        RationalVector::new(x, y, z)
    }

    /// Compares the x coordinates (cross-multiplied; z is non-negative).
    pub fn compare_x(&self, other: &RationalPoint) -> Ordering {
        (&self.x * &other.z).cmp(&(&other.x * &self.z))
    }

    /// Compares the y coordinates.
    pub fn compare_y(&self, other: &RationalPoint) -> Ordering {
        (&self.y * &other.z).cmp(&(&other.y * &self.z))
    }
}

impl From<IntPoint> for RationalPoint {
    fn from(p: IntPoint) -> Self {
        RationalPoint {
            x: BigInt::from(p.x),
            y: BigInt::from(p.y),
            z: BigInt::one(),
        }
    }
}

/// Equality of the represented affine points (cross-multiplied).
impl PartialEq for RationalPoint {
    fn eq(&self, other: &Self) -> bool {
        big_int_aux::determinant(&self.x, &self.z, &other.x, &other.z).is_zero()
            && big_int_aux::determinant(&self.y, &self.z, &other.y, &other.z).is_zero()
    }
}
