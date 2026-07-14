//! Port of the abstract class `geometry/planar/Vector.java` as an enum over
//! the integer and rational implementations.

use crate::datastructures::Signum;
use crate::geometry::planar::direction::{BigIntDirection, Direction};
use crate::geometry::planar::{limits, FloatPoint, IntDirection, IntVector, RationalVector, Side};
use num_bigint::BigInt;
use num_traits::{One, Signed, Zero};

/// A vector in the plane: either integer or rational (infinite precision)
/// coordinates.
#[derive(Debug, Clone)]
pub enum Vector {
    Int(IntVector),
    Rational(RationalVector),
}

impl Vector {
    pub const ZERO: Vector = Vector::Int(IntVector::ZERO);

    /// Creates a vector from integer coordinates, promoting to rational if
    /// they exceed `limits::CRIT_INT` (Java: `Vector.get_instance(int, int)`).
    pub fn get_instance(x: i32, y: i32) -> Self {
        let result = IntVector::new(x, y);
        if x.abs() > limits::CRIT_INT || y.abs() > limits::CRIT_INT {
            Vector::Rational(result.into())
        } else {
            Vector::Int(result)
        }
    }

    /// Creates a vector from projective coordinates (x/z, y/z), collapsing to
    /// an integer vector when possible (Java: `Vector.get_instance(BigInteger x3)`).
    ///
    /// Note: the Java original checks `p_x.mod(p_z)` twice where the second
    /// check should be `p_y.mod(p_z)` (a latent bug that could truncate y);
    /// this port checks both coordinates.
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
                return Vector::Int(IntVector::new(
                    x.to_i32().unwrap(),
                    y.to_i32().unwrap(),
                ));
            }
        }
        Vector::Rational(RationalVector::new(x, y, z))
    }

    fn to_rational(&self) -> RationalVector {
        match self {
            Vector::Int(v) => (*v).into(),
            Vector::Rational(v) => v.clone(),
        }
    }

    pub fn is_zero(&self) -> bool {
        match self {
            Vector::Int(v) => v.is_zero(),
            Vector::Rational(v) => v.is_zero(),
        }
    }

    pub fn negate(&self) -> Self {
        match self {
            Vector::Int(v) => Vector::Int(v.negate()),
            Vector::Rational(v) => Vector::Rational(v.negate()),
        }
    }

    pub fn add(&self, other: &Vector) -> Self {
        match (self, other) {
            (Vector::Int(a), Vector::Int(b)) => Vector::Int(a.add(*b)),
            _ => Vector::Rational(self.to_rational().add(&other.to_rational())),
        }
    }

    /// Let L be the line from the zero vector to `other`; returns which side
    /// of L this vector is on.
    pub fn side_of(&self, other: &Vector) -> Side {
        match (self, other) {
            (Vector::Int(a), Vector::Int(b)) => a.side_of(*b),
            _ => self.to_rational().side_of(&other.to_rational()),
        }
    }

    pub fn is_orthogonal(&self) -> bool {
        match self {
            Vector::Int(v) => v.is_orthogonal(),
            Vector::Rational(v) => v.is_orthogonal(),
        }
    }

    pub fn is_diagonal(&self) -> bool {
        match self {
            Vector::Int(v) => v.is_diagonal(),
            Vector::Rational(v) => v.is_diagonal(),
        }
    }

    pub fn is_multiple_of_45_degree(&self) -> bool {
        self.is_orthogonal() || self.is_diagonal()
    }

    /// The signum of the scalar product of this vector and `other`.
    pub fn projection(&self, other: &Vector) -> Signum {
        match (self, other) {
            (Vector::Int(a), Vector::Int(b)) => a.projection(*b),
            _ => self.to_rational().projection(&other.to_rational()),
        }
    }

    /// An approximation of the scalar product of this vector and `other`.
    pub fn scalar_product(&self, other: &Vector) -> f64 {
        match (self, other) {
            (Vector::Int(a), Vector::Int(b)) => a.scalar_product(*b) as f64,
            _ => self.to_rational().scalar_product(&other.to_rational()),
        }
    }

    pub fn to_float(&self) -> FloatPoint {
        match self {
            Vector::Int(v) => v.to_float(),
            Vector::Rational(v) => v.to_float(),
        }
    }

    pub fn turn_90_degree(&self, factor: i32) -> Self {
        match self {
            Vector::Int(v) => Vector::Int(v.turn_90_degree(factor)),
            Vector::Rational(v) => Vector::Rational(v.turn_90_degree(factor)),
        }
    }

    pub fn mirror_at_x_axis(&self) -> Self {
        match self {
            Vector::Int(v) => Vector::Int(v.mirror_at_x_axis()),
            Vector::Rational(v) => Vector::Rational(v.mirror_at_x_axis()),
        }
    }

    pub fn mirror_at_y_axis(&self) -> Self {
        match self {
            Vector::Int(v) => Vector::Int(v.mirror_at_y_axis()),
            Vector::Rational(v) => Vector::Rational(v.mirror_at_y_axis()),
        }
    }

    /// An approximation of the euclidean length of this vector.
    pub fn length_approx(&self) -> f64 {
        self.to_float().size()
    }

    /// An approximation of the cosine of the angle between this vector and
    /// `other`.
    pub fn cos_angle(&self, other: &Vector) -> f64 {
        self.scalar_product(other) / (self.to_float().size() * other.to_float().size())
    }

    /// An approximation of the signed angle between this vector and `other`.
    pub fn angle_approx_with(&self, other: &Vector) -> f64 {
        let mut result = self.cos_angle(other).acos();
        if self.side_of(other) == Side::OnTheLeft {
            result = -result;
        }
        result
    }

    /// An approximation of the signed angle between this vector and the
    /// x axis.
    pub fn angle_approx(&self) -> f64 {
        Vector::Int(IntVector::new(1, 0)).angle_approx_with(self)
    }

    /// An approximation of this vector with the same direction and length
    /// `length`. Like the Java original, this is not implemented for
    /// rational vectors (returns self unchanged there).
    pub fn change_length_approx(&self, length: f64) -> Self {
        match self {
            Vector::Int(v) => Vector::Int(v.change_length_approx(length)),
            Vector::Rational(_) => self.clone(),
        }
    }

    /// The normalized direction of this vector.
    pub fn to_normalized_direction(&self) -> Direction {
        match self {
            Vector::Int(v) => Direction::Int(IntDirection::from_vector(*v)),
            Vector::Rational(v) => {
                if v.is_zero() {
                    return Direction::NULL;
                }
                let gcd = gcd_big(&v.x, &v.y);
                let dx = &v.x / &gcd;
                let dy = &v.y / &gcd;
                let crit = BigInt::from(limits::CRIT_INT);
                if dx.abs() <= crit && dy.abs() <= crit {
                    use num_traits::ToPrimitive;
                    Direction::Int(IntDirection::from_vector(IntVector::new(
                        dx.to_i32().unwrap(),
                        dy.to_i32().unwrap(),
                    )))
                } else {
                    Direction::Big(BigIntDirection::new(dx, dy))
                }
            }
        }
    }
}

fn gcd_big(a: &BigInt, b: &BigInt) -> BigInt {
    use num_integer::Integer;
    a.gcd(b)
}

impl From<IntVector> for Vector {
    fn from(v: IntVector) -> Self {
        Vector::Int(v)
    }
}

/// Numeric equality across representations (an integral rational vector
/// equals the corresponding int vector).
impl PartialEq for Vector {
    fn eq(&self, other: &Self) -> bool {
        match (self, other) {
            (Vector::Int(a), Vector::Int(b)) => a == b,
            _ => self.to_rational() == other.to_rational(),
        }
    }
}
