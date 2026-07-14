//! Port of `geometry/planar/Side.java`.

/// Which side of a directed line an object lies on.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Side {
    OnTheLeft,
    OnTheRight,
    Collinear,
}

impl Side {
    /// Returns `OnTheLeft` if `value` > 0, `OnTheRight` if `value` < 0 and
    /// `Collinear` if `value` == 0.
    ///
    /// Note the sign convention is inherited from the Java code, where the
    /// caller passes a determinant whose sign is already flipped.
    pub fn of(value: f64) -> Self {
        if value > 0.0 {
            Side::OnTheLeft
        } else if value < 0.0 {
            Side::OnTheRight
        } else {
            Side::Collinear
        }
    }

    /// Same as [`Side::of`] for a sign value (-1, 0, +1), e.g. a `BigInt`
    /// signum.
    pub fn of_sign(sign: i32) -> Self {
        Self::of_i64(sign as i64)
    }

    /// Same as [`Side::of`] for exact integer determinants.
    pub fn of_i64(value: i64) -> Self {
        match value.signum() {
            1 => Side::OnTheLeft,
            -1 => Side::OnTheRight,
            _ => Side::Collinear,
        }
    }

    /// Returns the opposite side.
    pub fn negate(self) -> Self {
        match self {
            Side::OnTheLeft => Side::OnTheRight,
            Side::OnTheRight => Side::OnTheLeft,
            Side::Collinear => Side::Collinear,
        }
    }
}
