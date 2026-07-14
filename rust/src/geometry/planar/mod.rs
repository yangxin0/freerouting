//! Port of `app.freerouting.geometry.planar`.
//!
//! The Java code models points/vectors as an abstract class with `Int`,
//! `Rational` (BigInteger-backed) implementations. The Rust port starts with
//! the integer implementations; the rational variants will be added as an
//! enum layer when the code that needs them (line intersections) is ported.

pub mod direction;
pub mod float_line;
pub mod float_point;
pub mod int_direction;
pub mod int_point;
pub mod int_vector;
pub mod limits;
pub mod point;
pub mod rational_point;
pub mod rational_vector;
pub mod side;
pub mod vector;

pub use direction::{BigIntDirection, Direction};
pub use float_line::FloatLine;
pub use float_point::FloatPoint;
pub use int_direction::IntDirection;
pub use int_point::IntPoint;
pub use int_vector::IntVector;
pub use point::Point;
pub use rational_point::RationalPoint;
pub use rational_vector::RationalVector;
pub use side::Side;
pub use vector::Vector;
