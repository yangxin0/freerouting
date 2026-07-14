//! Port of `datastructures/Signum.java`.

/// The mathematical signum function.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Signum {
    Positive,
    Negative,
    Zero,
}

impl Signum {
    /// Returns the signum of `value`.
    pub fn of(value: f64) -> Self {
        if value > 0.0 {
            Signum::Positive
        } else if value < 0.0 {
            Signum::Negative
        } else {
            Signum::Zero
        }
    }

    /// Returns the signum of `value` as an int: +1, 0 or -1.
    pub fn as_int(value: f64) -> i32 {
        if value > 0.0 {
            1
        } else if value < 0.0 {
            -1
        } else {
            0
        }
    }

    /// Returns the opposite signum.
    pub fn negate(self) -> Self {
        match self {
            Signum::Positive => Signum::Negative,
            Signum::Negative => Signum::Positive,
            Signum::Zero => Signum::Zero,
        }
    }
}
