//! Port of `geometry/planar/Line.java`.
//!
//! The Java class declares `Point` endpoints but warns and casts to
//! `IntPoint` in every computation ("only implemented for IntPoints till
//! now"), so this port stores `IntPoint` endpoints directly. Intersection
//! *results* may still be rational and are returned as [`Point`].
//!
//! `is_on_the_left`/`is_on_the_right` (TileShape-based) will be added with
//! the shape hierarchy.

use crate::datastructures::Signum;
use crate::geometry::planar::{
    limits, FloatLine, FloatPoint, IntDirection, IntPoint, IntVector, Point, RationalPoint, Side,
    Vector,
};
use num_bigint::BigInt;
use num_traits::{One, Signed, Zero};
use std::cmp::Ordering;

/// A directed line in the plane through two integer points.
#[derive(Debug, Clone, Copy)]
pub struct Line {
    pub a: IntPoint,
    pub b: IntPoint,
}

impl Line {
    pub const fn new(a: IntPoint, b: IntPoint) -> Self {
        Line { a, b }
    }

    pub const fn from_coords(a_x: i32, a_y: i32, b_x: i32, b_y: i32) -> Self {
        Line {
            a: IntPoint::new(a_x, a_y),
            b: IntPoint::new(b_x, b_y),
        }
    }

    /// Creates a directed line from a point and a direction.
    pub fn from_direction(a: IntPoint, dir: IntDirection) -> Self {
        Line::new(a, a.translate_by(dir.get_vector()))
    }

    /// The direction of this directed line.
    pub fn direction(&self) -> IntDirection {
        IntDirection::from_vector(self.b.difference_by(self.a))
    }

    /// The sign of the determinant of the raw direction vectors of `self`
    /// and `other` (positive if `other` is counterclockwise from this
    /// line). Scale-invariant, so the gcd normalization of
    /// [`Line::direction`] is skipped.
    pub fn direction_determinant_sign(&self, other: &Line) -> i32 {
        let dx1 = i64::from(self.b.x) - i64::from(self.a.x);
        let dy1 = i64::from(self.b.y) - i64::from(self.a.y);
        let dx2 = i64::from(other.b.x) - i64::from(other.a.x);
        let dy2 = i64::from(other.b.y) - i64::from(other.a.y);
        (dx1 as i128 * dy2 as i128 - dy1 as i128 * dx2 as i128).signum() as i32
    }

    /// Returns `OnTheLeft` if this line is on the left of `point`,
    /// `OnTheRight` if on the right, `Collinear` if the line contains it.
    pub fn side_of(&self, point: &Point) -> Side {
        point.side_of_line(self).negate()
    }

    /// Like [`Line::side_of`] for an [`IntPoint`], fully exact and fast.
    pub fn side_of_int(&self, point: IntPoint) -> Side {
        point.side_of(self.a, self.b).negate()
    }

    /// Returns `Collinear` if `point` is on the line within `tolerance`,
    /// otherwise which side of `point` this line is on.
    pub fn side_of_float(&self, point: FloatPoint, tolerance: f64) -> Side {
        let det = (self.b.y - self.a.y) as f64 * (point.x - self.a.x as f64)
            - (self.b.x - self.a.x) as f64 * (point.y - self.a.y as f64);
        if det - tolerance > 0.0 {
            Side::OnTheLeft
        } else if det + tolerance < 0.0 {
            Side::OnTheRight
        } else {
            Side::Collinear
        }
    }

    /// Returns which side of the intersection of `l1` and `l2` this line is
    /// on, `Collinear` if all 3 lines intersect in exactly one point.
    pub fn side_of_intersection(&self, l1: &Line, l2: &Line) -> Side {
        // Fast approximate check first, exact check only if inconclusive.
        let intersection_approx = l1.intersection_approx(l2);
        let result = self.side_of_float(intersection_approx, 1.0);
        if result == Side::Collinear {
            self.side_of(&l1.intersection(l2))
        } else {
            result
        }
    }

    /// The signed distance of this line from `point`: positive if the line
    /// is on the left of `point`, else negative.
    pub fn signed_distance(&self, point: FloatPoint) -> f64 {
        let dx = (self.b.x - self.a.x) as f64;
        let dy = (self.b.y - self.a.y) as f64;
        // area of the parallelogram spanned by the 3 points
        let det = dy * (point.x - self.a.x as f64) - dx * (point.y - self.a.y as f64);
        det / (dx * dx + dy * dy).sqrt()
    }

    /// True if the 2 lines define the same set of points (directions may be
    /// opposite).
    pub fn overlaps(&self, other: &Line) -> bool {
        self.side_of_int(other.a) == Side::Collinear
            && self.side_of_int(other.b) == Side::Collinear
    }

    /// Alias of [`Line::overlaps`] (Java keeps both).
    pub fn is_equal_or_opposite(&self, other: &Line) -> bool {
        self.overlaps(other)
    }

    /// The line defining the same set of points but with opposite direction.
    pub fn opposite(&self) -> Self {
        Line::new(self.b, self.a)
    }

    /// The exact intersection point of the 2 lines. If the lines are
    /// parallel, `result.is_infinite()` is true.
    pub fn intersection(&self, other: &Line) -> Point {
        let delta_1 = self.b.difference_by(self.a);
        let delta_2 = other.b.difference_by(other.a);

        // Separate handling for orthogonal and 45 degree lines for better
        // performance (and to stay on integer arithmetic).
        if delta_1.x == 0 {
            // this line is vertical
            if delta_2.y == 0 {
                return Point::Int(IntPoint::new(self.a.x, other.a.y));
            }
            if delta_2.x == delta_2.y {
                return Point::Int(IntPoint::new(
                    self.a.x,
                    other.a.y + self.a.x - other.a.x,
                ));
            }
            if delta_2.x == -delta_2.y {
                return Point::Int(IntPoint::new(
                    self.a.x,
                    other.a.y + other.a.x - self.a.x,
                ));
            }
        } else if delta_1.y == 0 {
            // this line is horizontal
            if delta_2.x == 0 {
                return Point::Int(IntPoint::new(other.a.x, self.a.y));
            }
            if delta_2.x == delta_2.y {
                return Point::Int(IntPoint::new(
                    other.a.x + self.a.y - other.a.y,
                    self.a.y,
                ));
            }
            if delta_2.x == -delta_2.y {
                return Point::Int(IntPoint::new(
                    other.a.x + other.a.y - self.a.y,
                    self.a.y,
                ));
            }
        } else if delta_1.x == delta_1.y {
            // this line is right diagonal
            if delta_2.x == 0 {
                return Point::Int(IntPoint::new(
                    other.a.x,
                    self.a.y + other.a.x - self.a.x,
                ));
            }
            if delta_2.y == 0 {
                return Point::Int(IntPoint::new(
                    self.a.x + other.a.y - self.a.y,
                    other.a.y,
                ));
            }
        } else if delta_1.x == -delta_1.y {
            // this line is left diagonal
            if delta_2.x == 0 {
                return Point::Int(IntPoint::new(
                    other.a.x,
                    self.a.y + self.a.x - other.a.x,
                ));
            }
            if delta_2.y == 0 {
                return Point::Int(IntPoint::new(
                    self.a.x + self.a.y - other.a.y,
                    other.a.y,
                ));
            }
        }

        let det_1 = BigInt::from(self.a.determinant(self.b));
        let det_2 = BigInt::from(other.a.determinant(other.b));
        let mut det = BigInt::from(delta_2.determinant(delta_1));
        let mut is_x = &det_1 * BigInt::from(delta_2.x) - &det_2 * BigInt::from(delta_1.x);
        let mut is_y = &det_1 * BigInt::from(delta_2.y) - &det_2 * BigInt::from(delta_1.y);
        if !det.is_zero() {
            if det.is_negative() {
                det = -det;
                is_x = -is_x;
                is_y = -is_y;
            }
            if (&is_x % &det).is_zero() && (&is_y % &det).is_zero() {
                is_x = &is_x / &det;
                is_y = &is_y / &det;
                use num_traits::ToPrimitive;
                let crit = BigInt::from(limits::CRIT_INT);
                if is_x.abs() <= crit && is_y.abs() <= crit {
                    return Point::Int(IntPoint::new(
                        is_x.to_i32().unwrap(),
                        is_y.to_i32().unwrap(),
                    ));
                }
                det = BigInt::one();
            }
        }
        Point::Rational(RationalPoint::new(is_x, is_y, det))
    }

    /// An approximation of the intersection of the 2 lines. If the lines are
    /// parallel, the result coordinates are `i32::MAX` (like Java's
    /// `Integer.MAX_VALUE`).
    pub fn intersection_approx(&self, other: &Line) -> FloatPoint {
        let d1x = (self.b.x - self.a.x) as f64;
        let d1y = (self.b.y - self.a.y) as f64;
        let d2x = (other.b.x - other.a.x) as f64;
        let d2y = (other.b.y - other.a.y) as f64;
        let det_1 = self.a.x as f64 * self.b.y as f64 - self.a.y as f64 * self.b.x as f64;
        let det_2 = other.a.x as f64 * other.b.y as f64 - other.a.y as f64 * other.b.x as f64;
        let det = d2x * d1y - d2y * d1x;
        if det == 0.0 {
            FloatPoint::new(i32::MAX as f64, i32::MAX as f64)
        } else {
            FloatPoint::new(
                (d2x * det_1 - d1x * det_2) / det,
                (d2y * det_1 - d1y * det_2) / det,
            )
        }
    }

    /// The exact perpendicular projection of `point` onto this line.
    pub fn perpendicular_projection(&self, point: &Point) -> Point {
        match point {
            Point::Int(p) => self.perpendicular_projection_int(*p),
            Point::Rational(p) => self.perpendicular_projection_rational(p),
        }
    }

    /// Port of `IntPoint.perpendicular_projection(Line)`.
    fn perpendicular_projection_int(&self, point: IntPoint) -> Point {
        let v = self.b.difference_by(self.a);
        let vxvx = BigInt::from(v.x as i64 * v.x as i64);
        let vyvy = BigInt::from(v.y as i64 * v.y as i64);
        let vxvy = BigInt::from(v.x as i64 * v.y as i64);
        let denominator = &vxvx + &vyvy;
        let det = BigInt::from(self.a.determinant(self.b));
        let point_x = BigInt::from(point.x);
        let point_y = BigInt::from(point.y);

        let proj_x = &vxvx * &point_x + &vxvy * &point_y + &det * BigInt::from(v.y);
        let proj_y = &vxvy * &point_x + &vyvy * &point_y - &det * BigInt::from(v.x);

        // denominator = |v|^2 is positive for a non-degenerate line.
        if !denominator.is_zero()
            && (&proj_x % &denominator).is_zero()
            && (&proj_y % &denominator).is_zero()
        {
            use num_traits::ToPrimitive;
            let x = (&proj_x / &denominator).to_i32();
            let y = (&proj_y / &denominator).to_i32();
            if let (Some(x), Some(y)) = (x, y) {
                return Point::Int(IntPoint::new(x, y));
            }
        }
        Point::Rational(RationalPoint::new(proj_x, proj_y, denominator))
    }

    /// Port of `RationalPoint.perpendicular_projection(Line)`.
    fn perpendicular_projection_rational(&self, point: &RationalPoint) -> Point {
        let v = self.b.difference_by(self.a);
        let vxvx = BigInt::from(v.x as i64 * v.x as i64);
        let vyvy = BigInt::from(v.y as i64 * v.y as i64);
        let vxvy = BigInt::from(v.x as i64 * v.y as i64);
        let mut denominator = &vxvx + &vyvy;
        let det = BigInt::from(self.a.determinant(self.b));

        let mut proj_x = &vxvx * &point.x + &vxvy * &point.y + &det * BigInt::from(v.y) * &point.z;
        let mut proj_y = &vxvy * &point.x + &vyvy * &point.y + &det * BigInt::from(v.x) * &point.z;

        if !denominator.is_zero() {
            if denominator.is_negative() {
                denominator = -denominator;
                proj_x = -proj_x;
                proj_y = -proj_y;
            }
            if (&proj_x % &denominator).is_zero() && (&proj_y % &denominator).is_zero() {
                proj_x = &proj_x / &denominator;
                proj_y = &proj_y / &denominator;
                let crit = BigInt::from(limits::CRIT_INT);
                if proj_x.abs() <= crit && proj_y.abs() <= crit {
                    use num_traits::ToPrimitive;
                    return Point::Int(IntPoint::new(
                        proj_x.to_i32().unwrap(),
                        proj_y.to_i32().unwrap(),
                    ));
                }
                denominator = BigInt::one();
            }
        }
        Point::Rational(RationalPoint::new(proj_x, proj_y, denominator))
    }

    /// Translates the line perpendicular by about `dist`: to the left if
    /// `dist` > 0, else to the right.
    pub fn translate(&self, dist: f64) -> Self {
        let v = self.direction().get_vector();
        let vxvx = v.x as f64 * v.x as f64;
        let vyvy = v.y as f64 * v.y as f64;
        let length = (vxvx + vyvy).sqrt();
        let new_a = if vxvx <= vyvy {
            // translate along the x axis
            let rel_x = ((dist * length) / v.y as f64).round() as i32;
            IntPoint::new(self.a.x - rel_x, self.a.y)
        } else {
            // translate along the y axis
            let rel_y = ((dist * length) / v.x as f64).round() as i32;
            IntPoint::new(self.a.x, self.a.y + rel_y)
        };
        Line::from_direction(new_a, self.direction())
    }

    /// Translates the line by `vector` (integer vectors only keep the line
    /// on integer points).
    pub fn translate_by(&self, vector: IntVector) -> Self {
        if vector.is_zero() {
            return *self;
        }
        Line::new(self.a.translate_by(vector), self.b.translate_by(vector))
    }

    pub fn is_orthogonal(&self) -> bool {
        self.direction().is_orthogonal()
    }

    pub fn is_diagonal(&self) -> bool {
        self.direction().is_diagonal()
    }

    pub fn is_multiple_of_45_degree(&self) -> bool {
        self.direction().is_multiple_of_45_degree()
    }

    pub fn is_parallel(&self, other: &Line) -> bool {
        self.direction().side_of(other.direction()) == Side::Collinear
    }

    pub fn is_perpendicular(&self, other: &Line) -> bool {
        self.direction()
            .get_vector()
            .projection(other.direction().get_vector())
            == Signum::Zero
    }

    /// The cosine of the angle between this line and `other`.
    pub fn cos_angle(&self, other: &Line) -> f64 {
        let v1 = Vector::Int(self.b.difference_by(self.a));
        let v2 = Vector::Int(other.b.difference_by(other.a));
        v1.cos_angle(&v2)
    }

    /// An approximation of the function value of this line at `x` (line must
    /// not be vertical; returns 0 there, like the Java original after a
    /// warning).
    pub fn function_value_approx(&self, x: f64) -> f64 {
        let p1 = self.a.to_float();
        let p2 = self.b.to_float();
        let dx = p2.x - p1.x;
        if dx == 0.0 {
            return 0.0;
        }
        let dy = p2.y - p1.y;
        let det = p1.x * p2.y - p2.x * p1.y;
        (dy * x - det) / dx
    }

    /// An approximation of the x value of this line at `y` (line must not be
    /// horizontal; returns 0 there).
    pub fn function_in_y_value_approx(&self, y: f64) -> f64 {
        let p1 = self.a.to_float();
        let p2 = self.b.to_float();
        let dy = p2.y - p1.y;
        if dy == 0.0 {
            return 0.0;
        }
        let dx = p2.x - p1.x;
        let det = p1.x * p2.y - p2.x * p1.y;
        (dx * y + det) / dy
    }

    /// The direction from `from_point` to the nearest point on this line;
    /// `None` if `from_point` is contained in this line.
    pub fn perpendicular_direction(&self, from_point: IntPoint) -> Option<IntDirection> {
        let line_side = self.side_of_int(from_point);
        if line_side == Side::Collinear {
            return None;
        }
        let dir1 = self.direction().turn_45_degree(2);
        let dir2 = self.direction().turn_45_degree(6);

        let check_point_1 = from_point.translate_by(dir1.get_vector());
        if self.side_of_int(check_point_1) != line_side {
            return Some(dir1);
        }
        let check_point_2 = from_point.translate_by(dir2.get_vector());
        if self.side_of_int(check_point_2) != line_side {
            return Some(dir2);
        }
        let nearest_line_point = self.projection_approx(from_point.to_float());
        if nearest_line_point.distance_square(check_point_1.to_float())
            <= nearest_line_point.distance_square(check_point_2.to_float())
        {
            Some(dir1)
        } else {
            Some(dir2)
        }
    }

    /// An approximation of the perpendicular projection of `point` onto this
    /// line (Java: `FloatPoint.projection_approx(Line)`).
    pub fn projection_approx(&self, point: FloatPoint) -> FloatPoint {
        FloatLine::new(self.a.to_float(), self.b.to_float()).perpendicular_projection(point)
    }

    /// Turns this line by `factor` times 90 degree around `pole`.
    pub fn turn_90_degree(&self, factor: i32, pole: IntPoint) -> Self {
        let rot = |p: IntPoint| pole.translate_by(p.difference_by(pole).turn_90_degree(factor));
        Line::new(rot(self.a), rot(self.b))
    }

    /// Mirrors this line at the vertical line through `pole`. Note the Java
    /// original also swaps the endpoints to keep the direction consistent.
    pub fn mirror_vertical(&self, pole: IntPoint) -> Self {
        let mirror = |p: IntPoint| {
            pole.translate_by(p.difference_by(pole).mirror_at_y_axis())
        };
        Line::new(mirror(self.b), mirror(self.a))
    }

    /// Mirrors this line at the horizontal line through `pole`, swapping the
    /// endpoints like the Java original.
    pub fn mirror_horizontal(&self, pole: IntPoint) -> Self {
        let mirror = |p: IntPoint| {
            pole.translate_by(p.difference_by(pole).mirror_at_x_axis())
        };
        Line::new(mirror(self.b), mirror(self.a))
    }

    /// The distance between the two defining points of this line.
    pub fn length(&self) -> f64 {
        self.a.distance(self.b)
    }
}

/// Two `Line`s are equal if they define the same point set with the same
/// direction.
impl PartialEq for Line {
    fn eq(&self, other: &Self) -> bool {
        self.side_of_int(other.a) == Side::Collinear && self.direction() == other.direction()
    }
}

/// Lines are ordered by their direction angle, like [`IntDirection`].
impl Ord for Line {
    fn cmp(&self, other: &Self) -> Ordering {
        // Same angular order as IntDirection::cmp, but on the raw
        // difference vectors: the order is scale-invariant, so the gcd
        // normalization of direction() is skipped (this comparison
        // dominated the routing profile via Simplex sorting).
        let dx1 = i64::from(self.b.x) - i64::from(self.a.x);
        let dy1 = i64::from(self.b.y) - i64::from(self.a.y);
        let dx2 = i64::from(other.b.x) - i64::from(other.a.x);
        let dy2 = i64::from(other.b.y) - i64::from(other.a.y);
        if dy1 > 0 {
            if dy2 < 0 {
                return Ordering::Less;
            }
            if dy2 == 0 {
                return if dx2 > 0 {
                    Ordering::Greater
                } else {
                    Ordering::Less
                };
            }
        } else if dy1 < 0 {
            if dy2 >= 0 {
                return Ordering::Greater;
            }
        } else {
            // dy1 == 0
            if dx1 > 0 {
                return if dy2 != 0 || dx2 < 0 {
                    Ordering::Less
                } else {
                    Ordering::Equal
                };
            }
            // dx1 <= 0 (a null vector sorts with LEFT)
            if dy2 > 0 || (dy2 == 0 && dx2 > 0) {
                return Ordering::Greater;
            }
            if dy2 < 0 {
                return Ordering::Less;
            }
            return Ordering::Equal;
        }
        // both in the same open horizontal half plane: determinant order
        (dx2 as i128 * dy1 as i128 - dy2 as i128 * dx1 as i128).cmp(&0)
    }
}

impl PartialOrd for Line {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

impl Eq for Line {}

impl IntPoint {
    /// Which side of `line` this point is on.
    pub fn side_of_line(self, line: &Line) -> Side {
        self.side_of(line.a, line.b)
    }
}

impl Point {
    /// Which side of `line` this point is on.
    pub fn side_of_line(&self, line: &Line) -> Side {
        let v1 = self.difference_by(&Point::Int(line.a));
        let v2 = Vector::Int(line.b.difference_by(line.a));
        v1.side_of(&v2)
    }

    /// The exact perpendicular projection of this point onto `line`.
    pub fn perpendicular_projection(&self, line: &Line) -> Point {
        line.perpendicular_projection(self)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn intersection_fast_paths_and_general() {
        let vertical = Line::from_coords(2, 0, 2, 5);
        let horizontal = Line::from_coords(0, 3, 9, 3);
        let right_diag = Line::from_coords(0, 0, 1, 1);
        let general = Line::from_coords(0, 0, 3, 1);

        assert_eq!(
            vertical.intersection(&horizontal),
            Point::Int(IntPoint::new(2, 3))
        );
        assert_eq!(
            vertical.intersection(&right_diag),
            Point::Int(IntPoint::new(2, 2))
        );
        assert_eq!(
            horizontal.intersection(&right_diag),
            Point::Int(IntPoint::new(3, 3))
        );
        // General line vs vertical: y = x/3 meets x = 2 at (2, 2/3),
        // a rational point via the BigInt path.
        let q = general.intersection(&vertical);
        assert!(matches!(q, Point::Rational(_)));
        let qf = q.to_float();
        assert!((qf.x - 2.0).abs() < 1e-12 && (qf.y - 2.0 / 3.0).abs() < 1e-12);
        // General case with a rational result: y = x/3 meets y = 1 - x at
        // x = 3/4.
        let descending = Line::from_coords(0, 1, 1, 0);
        let p = general.intersection(&descending);
        assert!(matches!(p, Point::Rational(_)));
        let f = p.to_float();
        assert!((f.x - 0.75).abs() < 1e-12 && (f.y - 0.25).abs() < 1e-12);
        // Approximate intersection agrees.
        let fa = general.intersection_approx(&descending);
        assert!((fa.x - 0.75).abs() < 1e-9 && (fa.y - 0.25).abs() < 1e-9);
        // Parallel lines: infinite point.
        let parallel = general.translate_by(IntVector::new(0, 5));
        assert!(general.intersection(&parallel).is_infinite());
    }

    #[test]
    fn side_conventions() {
        let l = Line::from_coords(0, 0, 10, 0);
        // Line pointing +x: point above it -> the LINE is on the right of
        // the point.
        assert_eq!(l.side_of_int(IntPoint::new(5, 2)), Side::OnTheRight);
        assert_eq!(l.side_of_int(IntPoint::new(5, -2)), Side::OnTheLeft);
        assert_eq!(l.side_of_int(IntPoint::new(20, 0)), Side::Collinear);
        // The tolerance is in determinant units (it scales with the line
        // length), exactly like the Java original.
        assert_eq!(
            l.side_of_float(FloatPoint::new(5.0, 0.05), 1.0),
            Side::Collinear
        );
        assert_eq!(
            l.side_of_float(FloatPoint::new(5.0, 0.5), 1.0),
            Side::OnTheRight
        );
        assert_eq!(
            l.side_of(&Point::Int(IntPoint::new(5, 2))),
            Side::OnTheRight
        );
    }

    #[test]
    fn equality_and_overlap() {
        let l = Line::from_coords(0, 0, 2, 2);
        let same = Line::from_coords(-1, -1, 5, 5);
        let opposite = same.opposite();
        assert_eq!(l, same);
        assert_ne!(l, opposite);
        assert!(l.overlaps(&opposite));
        assert!(l.is_equal_or_opposite(&opposite));
        assert!(l.is_parallel(&Line::from_coords(0, 5, 2, 7)));
        assert!(l.is_perpendicular(&Line::from_coords(0, 0, 2, -2)));
    }

    #[test]
    fn perpendicular_projection_exact() {
        let l = Line::from_coords(0, 0, 10, 0);
        assert_eq!(
            l.perpendicular_projection(&Point::Int(IntPoint::new(3, 7))),
            Point::Int(IntPoint::new(3, 0))
        );
        // Projection onto the diagonal of (1, 0): rational (1/2, 1/2).
        let diag = Line::from_coords(0, 0, 1, 1);
        let p = diag.perpendicular_projection(&Point::Int(IntPoint::new(1, 0)));
        let f = p.to_float();
        assert!((f.x - 0.5).abs() < 1e-12 && (f.y - 0.5).abs() < 1e-12);
    }

    #[test]
    fn translate_and_transforms() {
        let l = Line::from_coords(0, 0, 10, 0);
        let t = l.translate(2.0);
        // Left of +x is +y.
        assert_eq!(t.a.y, 2);
        assert_eq!(t.direction(), l.direction());

        let turned = l.turn_90_degree(1, IntPoint::new(0, 0));
        assert_eq!(turned.direction(), IntDirection::UP);

        // Vertical mirroring flips the geometry but also swaps the
        // endpoints (like Java), so the direction is preserved for a
        // horizontal line.
        let mirrored = l.mirror_vertical(IntPoint::new(0, 0));
        assert_eq!(mirrored.direction(), l.direction());
        assert_eq!(mirrored.a, IntPoint::new(-10, 0));
    }

    #[test]
    fn function_values_and_distance() {
        let l = Line::from_coords(0, 0, 2, 2);
        assert!((l.function_value_approx(3.0) - 3.0).abs() < 1e-12);
        assert!((l.function_in_y_value_approx(4.0) - 4.0).abs() < 1e-12);
        let d = Line::from_coords(0, 0, 10, 0).signed_distance(FloatPoint::new(5.0, -3.0));
        assert!((d - 3.0).abs() < 1e-12);
    }

    #[test]
    fn perpendicular_direction_and_ordering() {
        let l = Line::from_coords(0, 0, 10, 0);
        // Point above the line: nearest direction is straight down.
        assert_eq!(
            l.perpendicular_direction(IntPoint::new(5, 3)),
            Some(IntDirection::DOWN)
        );
        assert_eq!(l.perpendicular_direction(IntPoint::new(5, 0)), None);

        let steeper = Line::from_coords(0, 0, 1, 2);
        let flatter = Line::from_coords(0, 0, 2, 1);
        assert!(flatter < steeper);
    }

    #[test]
    fn side_of_intersection() {
        let l1 = Line::from_coords(0, 0, 10, 0);
        let l2 = Line::from_coords(5, -5, 5, 5); // meet at (5, 0)
        let through = Line::from_coords(0, -5, 10, 5); // passes through (5, 0)
        assert_eq!(through.side_of_intersection(&l1, &l2), Side::Collinear);
        let above = Line::from_coords(0, 20, 10, 20);
        assert_eq!(
            above.side_of_intersection(&l1, &l2),
            above.side_of_int(IntPoint::new(5, 0))
        );
    }
}
