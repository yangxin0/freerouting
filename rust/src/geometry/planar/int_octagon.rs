//! Port of `geometry/planar/IntOctagon.java` and
//! `FortyfiveDegreeDirection.java`.
//!
//! An octagon with integer coordinates whose border lines are horizontal,
//! vertical or diagonal (45 degrees) — the preferred shape of the
//! autorouter. Simplex/TileShape interop will be added with those types.
//!
//! The shape is the intersection of 8 half planes described by:
//! `left_x`/`right_x` (vertical borders), `bottom_y`/`top_y` (horizontal
//! borders), and the x-axis intersections of the four diagonal borders.

use crate::geometry::planar::{
    limits, FloatPoint, IntBox, IntDirection, IntPoint, IntVector, Line, Side,
};

/// The eight 45-degree directions, in the same order as the Java enum.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum FortyfiveDegreeDirection {
    Right,
    Right45,
    Up,
    Up45,
    Left,
    Left45,
    Down,
    Down45,
}

impl FortyfiveDegreeDirection {
    pub const VALUES: [FortyfiveDegreeDirection; 8] = [
        Self::Right,
        Self::Right45,
        Self::Up,
        Self::Up45,
        Self::Left,
        Self::Left45,
        Self::Down,
        Self::Down45,
    ];

    pub fn get_direction(self) -> IntDirection {
        match self {
            Self::Right => IntDirection::RIGHT,
            Self::Right45 => IntDirection::RIGHT45,
            Self::Up => IntDirection::UP,
            Self::Up45 => IntDirection::UP45,
            Self::Left => IntDirection::LEFT,
            Self::Left45 => IntDirection::LEFT45,
            Self::Down => IntDirection::DOWN,
            Self::Down45 => IntDirection::DOWN45,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct IntOctagon {
    /// x coordinate of the left vertical border
    pub left_x: i32,
    /// y coordinate of the bottom horizontal border
    pub bottom_y: i32,
    /// x coordinate of the right vertical border
    pub right_x: i32,
    /// y coordinate of the top horizontal border
    pub top_y: i32,
    /// x-axis intersection of the upper left diagonal border
    pub upper_left_diagonal_x: i32,
    /// x-axis intersection of the lower right diagonal border
    pub lower_right_diagonal_x: i32,
    /// x-axis intersection of the lower left diagonal border
    pub lower_left_diagonal_x: i32,
    /// x-axis intersection of the upper right diagonal border
    pub upper_right_diagonal_x: i32,
}

impl IntOctagon {
    /// Reusable instance of an empty octagon.
    pub const EMPTY: IntOctagon = IntOctagon::new(
        limits::CRIT_INT,
        limits::CRIT_INT,
        -limits::CRIT_INT,
        -limits::CRIT_INT,
        limits::CRIT_INT,
        -limits::CRIT_INT,
        limits::CRIT_INT,
        -limits::CRIT_INT,
    );

    /// Argument order follows the Java constructor:
    /// (lx, ly, rx, uy, ulx, lrx, llx, urx).
    pub const fn new(
        lx: i32,
        ly: i32,
        rx: i32,
        uy: i32,
        ulx: i32,
        lrx: i32,
        llx: i32,
        urx: i32,
    ) -> Self {
        IntOctagon {
            left_x: lx,
            bottom_y: ly,
            right_x: rx,
            top_y: uy,
            upper_left_diagonal_x: ulx,
            lower_right_diagonal_x: lrx,
            lower_left_diagonal_x: llx,
            upper_right_diagonal_x: urx,
        }
    }

    pub fn is_empty(&self) -> bool {
        *self == Self::EMPTY
    }

    pub fn is_bounded(&self) -> bool {
        true
    }

    pub fn bounding_box(&self) -> IntBox {
        IntBox::from_coords(self.left_x, self.bottom_y, self.right_x, self.top_y)
    }

    pub fn dimension(&self) -> i32 {
        if self.is_empty() {
            return -1;
        }
        if self.right_x > self.left_x
            && self.top_y > self.bottom_y
            && self.lower_right_diagonal_x > self.upper_left_diagonal_x
            && self.upper_right_diagonal_x > self.lower_left_diagonal_x
        {
            2
        } else if self.right_x == self.left_x && self.top_y == self.bottom_y {
            0
        } else {
            1
        }
    }

    pub fn border_line_count(&self) -> usize {
        8
    }

    /// The corners in counterclockwise order, starting with the left corner
    /// of the lower border.
    pub fn corner(&self, no: usize) -> IntPoint {
        IntPoint::new(self.corner_x(no), self.corner_y(no))
    }

    pub fn corner_x(&self, no: usize) -> i32 {
        match no {
            0 => self.lower_left_diagonal_x - self.bottom_y,
            1 => self.lower_right_diagonal_x + self.bottom_y,
            2 | 3 => self.right_x,
            4 => self.upper_right_diagonal_x - self.top_y,
            5 => self.upper_left_diagonal_x + self.top_y,
            6 | 7 => self.left_x,
            _ => panic!("IntOctagon::corner_x: no out of range"),
        }
    }

    pub fn corner_y(&self, no: usize) -> i32 {
        match no {
            0 | 1 => self.bottom_y,
            2 => self.right_x - self.lower_right_diagonal_x,
            3 => self.upper_right_diagonal_x - self.right_x,
            4 | 5 => self.top_y,
            6 => self.left_x - self.upper_left_diagonal_x,
            7 => self.lower_left_diagonal_x - self.left_x,
            _ => panic!("IntOctagon::corner_y: no out of range"),
        }
    }

    /// The area of the octagon via the shoelace formula over its corners.
    pub fn area(&self) -> f64 {
        let mut result = (self.lower_left_diagonal_x - self.bottom_y) as f64
            * (self.bottom_y - self.lower_left_diagonal_x + self.left_x) as f64;
        result += (self.lower_right_diagonal_x + self.bottom_y) as f64
            * (self.right_x - self.lower_right_diagonal_x - self.bottom_y) as f64;
        result += self.right_x as f64
            * (self.upper_right_diagonal_x - 2 * self.right_x - self.bottom_y
                + self.top_y
                + self.lower_right_diagonal_x) as f64;
        result += (self.upper_right_diagonal_x - self.top_y) as f64
            * (self.top_y - self.upper_right_diagonal_x + self.right_x) as f64;
        result += (self.upper_left_diagonal_x + self.top_y) as f64
            * (self.left_x - self.upper_left_diagonal_x - self.top_y) as f64;
        result += self.left_x as f64
            * (self.lower_left_diagonal_x - 2 * self.left_x - self.top_y
                + self.bottom_y
                + self.upper_left_diagonal_x) as f64;
        0.5 * result.abs()
    }

    /// The border lines in counterclockwise order starting with the lower
    /// boundary; the interior is on the point-left of each line.
    pub fn border_line(&self, no: usize) -> Line {
        match no {
            0 => Line::from_coords(0, self.bottom_y, 1, self.bottom_y),
            1 => Line::from_coords(
                self.lower_right_diagonal_x,
                0,
                self.lower_right_diagonal_x + 1,
                1,
            ),
            2 => Line::from_coords(self.right_x, 0, self.right_x, 1),
            3 => Line::from_coords(
                self.upper_right_diagonal_x,
                0,
                self.upper_right_diagonal_x - 1,
                1,
            ),
            4 => Line::from_coords(0, self.top_y, -1, self.top_y),
            5 => Line::from_coords(
                self.upper_left_diagonal_x,
                0,
                self.upper_left_diagonal_x - 1,
                -1,
            ),
            6 => Line::from_coords(self.left_x, 0, self.left_x, -1),
            7 => Line::from_coords(
                self.lower_left_diagonal_x,
                0,
                self.lower_left_diagonal_x + 1,
                -1,
            ),
            _ => panic!("IntOctagon::border_line: no out of range"),
        }
    }

    pub fn translate_by(&self, v: IntVector) -> Self {
        if v.is_zero() {
            return *self;
        }
        IntOctagon::new(
            self.left_x + v.x,
            self.bottom_y + v.y,
            self.right_x + v.x,
            self.top_y + v.y,
            self.upper_left_diagonal_x + v.x - v.y,
            self.lower_right_diagonal_x + v.x - v.y,
            self.lower_left_diagonal_x + v.x + v.y,
            self.upper_right_diagonal_x + v.x + v.y,
        )
    }

    pub fn max_width(&self) -> f64 {
        let width_1 = (self.right_x - self.left_x).max(self.top_y - self.bottom_y) as f64;
        let width_2 = (self.upper_right_diagonal_x - self.lower_left_diagonal_x)
            .max(self.lower_right_diagonal_x - self.upper_left_diagonal_x)
            as f64;
        width_1.max(width_2 / limits::SQRT2)
    }

    pub fn min_width(&self) -> f64 {
        let width_1 = (self.right_x - self.left_x).min(self.top_y - self.bottom_y) as f64;
        let width_2 = (self.upper_right_diagonal_x - self.lower_left_diagonal_x)
            .min(self.lower_right_diagonal_x - self.upper_left_diagonal_x)
            as f64;
        width_1.min(width_2 / limits::SQRT2)
    }

    /// Offsets the octagon by `distance`: outward if positive, inward if
    /// negative; the result is normalized.
    pub fn offset(&self, distance: f64) -> Self {
        let width = distance.round() as i32;
        if width == 0 {
            return *self;
        }
        // Saturating arithmetic: octagons derived from nearly parallel
        // line intersections can carry quasi-infinite coordinates.
        let dia_width = (limits::SQRT2 * distance).round() as i32;
        IntOctagon::new(
            self.left_x.saturating_sub(width),
            self.bottom_y.saturating_sub(width),
            self.right_x.saturating_add(width),
            self.top_y.saturating_add(width),
            self.upper_left_diagonal_x.saturating_sub(dia_width),
            self.lower_right_diagonal_x.saturating_add(dia_width),
            self.lower_left_diagonal_x.saturating_sub(dia_width),
            self.upper_right_diagonal_x.saturating_add(dia_width),
        )
        .normalize()
    }

    pub fn enlarge(&self, offset: f64) -> Self {
        self.offset(offset)
    }

    /// Makes the octagon canonical: every border line touches the shape, so
    /// that equal shapes have equal coordinates. Returns [`Self::EMPTY`] if
    /// the shape is empty.
    pub fn normalize(&self) -> Self {
        if self.left_x > self.right_x
            || self.bottom_y > self.top_y
            || self.lower_left_diagonal_x > self.upper_right_diagonal_x
            || self.upper_left_diagonal_x > self.lower_right_diagonal_x
        {
            return Self::EMPTY;
        }
        let mut lx = self.left_x;
        let mut rx = self.right_x;
        let mut ly = self.bottom_y;
        let mut uy = self.top_y;
        let mut llx = self.lower_left_diagonal_x;
        let mut ulx = self.upper_left_diagonal_x;
        let mut lrx = self.lower_right_diagonal_x;
        let mut urx = self.upper_right_diagonal_x;

        // Pull each straight border onto the shape.
        lx = lx.max(llx - uy).max(ulx + ly);
        rx = rx.min(urx - ly).min(lrx + uy);
        ly = ly.max(lx - lrx).max(llx - rx);
        uy = uy.min(urx - lx).min(rx - ulx);

        // Pull each diagonal border onto the shape.
        if llx - lx < ly {
            llx = lx + ly;
        }
        if rx - lrx < ly {
            lrx = rx - ly;
        }
        if urx - rx > uy {
            urx = uy + rx;
        }
        if lx - ulx > uy {
            ulx = lx - uy;
        }

        // Cut straight borders back to the diagonal intersections.
        let diag_upper_y = ((urx - ulx) as f64 / 2.0).ceil() as i32;
        if uy > diag_upper_y {
            uy = diag_upper_y;
        }
        let diag_lower_y = ((llx - lrx) as f64 / 2.0).floor() as i32;
        if ly < diag_lower_y {
            ly = diag_lower_y;
        }
        let diag_right_x = ((urx + lrx) as f64 / 2.0).ceil() as i32;
        if rx > diag_right_x {
            rx = diag_right_x;
        }
        let diag_left_x = ((llx + ulx) as f64 / 2.0).floor() as i32;
        if lx < diag_left_x {
            lx = diag_left_x;
        }

        if lx > rx || ly > uy || llx > urx || ulx > lrx {
            return Self::EMPTY;
        }
        IntOctagon::new(lx, ly, rx, uy, ulx, lrx, llx, urx)
    }

    pub fn is_normalized(&self) -> bool {
        *self == self.normalize()
    }

    /// True if `point` is contained in this octagon; may be inexact close to
    /// the border because of the float parameter.
    pub fn contains_float(&self, point: FloatPoint) -> bool {
        if (self.left_x as f64) > point.x
            || (self.bottom_y as f64) > point.y
            || (self.right_x as f64) < point.x
            || (self.top_y as f64) < point.y
        {
            return false;
        }
        let tmp_1 = point.x - point.y;
        let tmp_2 = point.x + point.y;
        (self.upper_left_diagonal_x as f64) <= tmp_1
            && (self.lower_right_diagonal_x as f64) >= tmp_1
            && (self.lower_left_diagonal_x as f64) <= tmp_2
            && (self.upper_right_diagonal_x as f64) >= tmp_2
    }

    /// Exact containment for an [`IntPoint`] (border included).
    pub fn contains(&self, point: IntPoint) -> bool {
        for i in 0..8 {
            if self.side_of_border_line(point.x, point.y, i) == Side::OnTheRight {
                return false;
            }
        }
        true
    }

    /// The side of the point (x, y) relative to the border line
    /// `border_line_no` (border lines run counterclockwise).
    pub fn side_of_border_line(&self, x: i32, y: i32, border_line_no: usize) -> Side {
        let tmp = match border_line_no {
            0 => self.bottom_y - y,
            2 => x - self.right_x,
            4 => y - self.top_y,
            6 => self.left_x - x,
            1 => x - y - self.lower_right_diagonal_x,
            3 => x + y - self.upper_right_diagonal_x,
            5 => self.upper_left_diagonal_x + y - x,
            7 => self.lower_left_diagonal_x - x - y,
            _ => panic!("IntOctagon::side_of_border_line: border_line_no out of range"),
        };
        match tmp.cmp(&0) {
            std::cmp::Ordering::Less => Side::OnTheLeft,
            std::cmp::Ordering::Greater => Side::OnTheRight,
            std::cmp::Ordering::Equal => Side::Collinear,
        }
    }

    /// Like [`Self::side_of_border_line`] for a float point with tolerance.
    pub fn border_line_side_of(&self, point: FloatPoint, line_no: usize, tolerance: f64) -> Side {
        let tmp = match line_no {
            0 => (self.bottom_y as f64) - point.y,
            2 => point.x - self.right_x as f64,
            4 => point.y - self.top_y as f64,
            6 => self.left_x as f64 - point.x,
            1 => point.x - point.y - self.lower_right_diagonal_x as f64,
            3 => point.x + point.y - self.upper_right_diagonal_x as f64,
            5 => self.upper_left_diagonal_x as f64 + point.y - point.x,
            7 => self.lower_left_diagonal_x as f64 - point.x - point.y,
            _ => panic!("IntOctagon::border_line_side_of: line_no out of range"),
        };
        if tmp < -tolerance {
            Side::OnTheLeft
        } else if tmp > tolerance {
            Side::OnTheRight
        } else {
            Side::Collinear
        }
    }

    /// The intersection of two normalized octagons (normalized result).
    pub fn intersection(&self, other: IntOctagon) -> IntOctagon {
        IntOctagon::new(
            self.left_x.max(other.left_x),
            self.bottom_y.max(other.bottom_y),
            self.right_x.min(other.right_x),
            self.top_y.min(other.top_y),
            self.upper_left_diagonal_x.max(other.upper_left_diagonal_x),
            self.lower_right_diagonal_x
                .min(other.lower_right_diagonal_x),
            self.lower_left_diagonal_x.max(other.lower_left_diagonal_x),
            self.upper_right_diagonal_x
                .min(other.upper_right_diagonal_x),
        )
        .normalize()
    }

    /// The smallest octagon containing this and `other`.
    pub fn union(&self, other: IntOctagon) -> IntOctagon {
        IntOctagon::new(
            self.left_x.min(other.left_x),
            self.bottom_y.min(other.bottom_y),
            self.right_x.max(other.right_x),
            self.top_y.max(other.top_y),
            self.upper_left_diagonal_x.min(other.upper_left_diagonal_x),
            self.lower_right_diagonal_x
                .max(other.lower_right_diagonal_x),
            self.lower_left_diagonal_x.min(other.lower_left_diagonal_x),
            self.upper_right_diagonal_x
                .max(other.upper_right_diagonal_x),
        )
    }

    pub fn is_contained_in_box(&self, box_: IntBox) -> bool {
        self.left_x >= box_.ll.x
            && self.bottom_y >= box_.ll.y
            && self.right_x <= box_.ur.x
            && self.top_y <= box_.ur.y
    }

    pub fn is_contained_in(&self, other: IntOctagon) -> bool {
        self.left_x >= other.left_x
            && self.bottom_y >= other.bottom_y
            && self.right_x <= other.right_x
            && self.top_y <= other.top_y
            && self.lower_left_diagonal_x >= other.lower_left_diagonal_x
            && self.upper_left_diagonal_x >= other.upper_left_diagonal_x
            && self.lower_right_diagonal_x <= other.lower_right_diagonal_x
            && self.upper_right_diagonal_x <= other.upper_right_diagonal_x
    }

    /// Checks if two normalized octagons intersect.
    pub fn intersects(&self, other: IntOctagon) -> bool {
        self.left_x.max(other.left_x) <= self.right_x.min(other.right_x)
            && self.bottom_y.max(other.bottom_y) <= self.top_y.min(other.top_y)
            && self.lower_left_diagonal_x.max(other.lower_left_diagonal_x)
                <= self
                    .upper_right_diagonal_x
                    .min(other.upper_right_diagonal_x)
            && self.upper_left_diagonal_x.max(other.upper_left_diagonal_x)
                <= self
                    .lower_right_diagonal_x
                    .min(other.lower_right_diagonal_x)
    }

    /// True if this octagon intersects `other` with a 2-dimensional
    /// intersection.
    pub fn overlaps(&self, other: IntOctagon) -> bool {
        self.left_x.max(other.left_x) < self.right_x.min(other.right_x)
            && self.bottom_y.max(other.bottom_y) < self.top_y.min(other.top_y)
            && self.lower_left_diagonal_x.max(other.lower_left_diagonal_x)
                < self
                    .upper_right_diagonal_x
                    .min(other.upper_right_diagonal_x)
            && self.upper_left_diagonal_x.max(other.upper_left_diagonal_x)
                < self
                    .lower_right_diagonal_x
                    .min(other.lower_right_diagonal_x)
    }

    /// The x value of the left boundary at `y`.
    pub fn left_x_value(&self, y: i32) -> i32 {
        self.left_x
            .max(self.upper_left_diagonal_x + y)
            .max(self.lower_left_diagonal_x - y)
    }

    /// The x value of the right boundary at `y`.
    pub fn right_x_value(&self, y: i32) -> i32 {
        self.right_x
            .min(self.upper_right_diagonal_x - y)
            .min(self.lower_right_diagonal_x + y)
    }

    /// The y value of the lower boundary at `x`.
    pub fn lower_y_value(&self, x: i32) -> i32 {
        self.bottom_y
            .max(self.lower_left_diagonal_x - x)
            .max(x - self.lower_right_diagonal_x)
    }

    /// The y value of the upper boundary at `x`.
    pub fn upper_y_value(&self, x: i32) -> i32 {
        self.top_y
            .min(x - self.upper_left_diagonal_x)
            .min(self.upper_right_diagonal_x - x)
    }

    /// Compares the position of the border line `edge_no` of this octagon
    /// with the corresponding border line of `other`.
    pub fn compare(&self, other: IntOctagon, edge_no: usize) -> Side {
        let cmp = |a: i32, b: i32, greater_is_left: bool| {
            if a == b {
                Side::Collinear
            } else if (a > b) == greater_is_left {
                Side::OnTheLeft
            } else {
                Side::OnTheRight
            }
        };
        match edge_no {
            0 => cmp(self.bottom_y, other.bottom_y, true),
            1 => cmp(
                self.lower_right_diagonal_x,
                other.lower_right_diagonal_x,
                false,
            ),
            2 => cmp(self.right_x, other.right_x, false),
            3 => cmp(
                self.upper_right_diagonal_x,
                other.upper_right_diagonal_x,
                false,
            ),
            4 => cmp(self.top_y, other.top_y, false),
            5 => cmp(
                self.upper_left_diagonal_x,
                other.upper_left_diagonal_x,
                true,
            ),
            6 => cmp(self.left_x, other.left_x, true),
            7 => cmp(
                self.lower_left_diagonal_x,
                other.lower_left_diagonal_x,
                true,
            ),
            _ => panic!("IntOctagon::compare: edge_no out of range"),
        }
    }

    /// True if this octagon describes exactly an [`IntBox`].
    pub fn is_int_box(&self) -> bool {
        self.lower_left_diagonal_x == self.left_x + self.bottom_y
            && self.lower_right_diagonal_x == self.right_x - self.bottom_y
            && self.upper_right_diagonal_x == self.right_x + self.top_y
            && self.upper_left_diagonal_x == self.left_x - self.top_y
    }

    /// The border point of this octagon from `point` into the 45-degree
    /// direction `dir`; if that point is not integer, the nearest outside
    /// integer point is returned.
    pub fn border_point(&self, point: IntPoint, dir: FortyfiveDegreeDirection) -> IntPoint {
        use FortyfiveDegreeDirection::*;
        let (result_x, result_y);
        match dir {
            Right => {
                result_x = self
                    .right_x
                    .min(self.upper_right_diagonal_x - point.y)
                    .min(self.lower_right_diagonal_x + point.y);
                result_y = point.y;
            }
            Left => {
                result_x = self
                    .left_x
                    .max(self.upper_left_diagonal_x + point.y)
                    .max(self.lower_left_diagonal_x - point.y);
                result_y = point.y;
            }
            Up => {
                result_x = point.x;
                result_y = self
                    .top_y
                    .min(point.x - self.upper_left_diagonal_x)
                    .min(self.upper_right_diagonal_x - point.x);
            }
            Down => {
                result_x = point.x;
                result_y = self
                    .bottom_y
                    .max(self.lower_left_diagonal_x - point.x)
                    .max(point.x - self.lower_right_diagonal_x);
            }
            Right45 => {
                let x =
                    (0.5 * (point.x - point.y + self.upper_right_diagonal_x) as f64).ceil() as i32;
                result_x = x.min(self.right_x).min(point.x - point.y + self.top_y);
                result_y = point.y - point.x + result_x;
            }
            Up45 => {
                let x =
                    (0.5 * (point.x + point.y + self.upper_left_diagonal_x) as f64).floor() as i32;
                result_x = x.max(self.left_x).max(point.x + point.y - self.top_y);
                result_y = point.y + point.x - result_x;
            }
            Left45 => {
                let x =
                    (0.5 * (point.x - point.y + self.lower_left_diagonal_x) as f64).floor() as i32;
                result_x = x.max(self.left_x).max(point.x - point.y + self.bottom_y);
                result_y = point.y - point.x + result_x;
            }
            Down45 => {
                let x =
                    (0.5 * (point.x + point.y + self.lower_right_diagonal_x) as f64).ceil() as i32;
                result_x = x.min(self.right_x).min(point.x + point.y - self.bottom_y);
                result_y = point.y + point.x - result_x;
            }
        }
        IntPoint::new(result_x, result_y)
    }

    /// The sorted `max_result_points` (at most 8) nearest points on the
    /// border of this octagon in the 45-degree directions from `point`,
    /// which must be located inside the octagon.
    pub fn nearest_border_projections(
        &self,
        point: IntPoint,
        max_result_points: usize,
    ) -> Vec<IntPoint> {
        if max_result_points == 0 || !self.contains(point) {
            return Vec::new();
        }
        let max_result_points = max_result_points.min(8);
        let inside_point = point.to_float();
        let mut candidates: Vec<(f64, IntPoint)> = FortyfiveDegreeDirection::VALUES
            .iter()
            .map(|&dir| {
                let border_point = self.border_point(point, dir);
                (
                    inside_point.distance_square(border_point.to_float()),
                    border_point,
                )
            })
            .collect();
        candidates.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap());
        candidates
            .into_iter()
            .take(max_result_points)
            .map(|(_, p)| p)
            .collect()
    }

    /// Divides `from_box` minus this octagon into up to 8 convex pieces,
    /// 4 boxes and 4 corner octagons, optimized for minimal cumulative
    /// circumference.
    pub fn cutout_from_box(&self, from_box: IntBox) -> Vec<IntOctagon> {
        let c = self.intersection(from_box.to_int_octagon());
        if self.is_empty() || c.dimension() < self.dimension() {
            // there is only an overlap at the border
            return vec![from_box.to_int_octagon()];
        }
        let p_d = from_box;

        let mut boxes = [
            // left box
            IntBox::from_coords(
                p_d.ll.x,
                c.lower_left_diagonal_x - c.left_x,
                c.left_x,
                c.left_x - c.upper_left_diagonal_x,
            ),
            // right box
            IntBox::from_coords(
                c.right_x,
                c.right_x - c.lower_right_diagonal_x,
                p_d.ur.x,
                c.upper_right_diagonal_x - c.right_x,
            ),
            // lower box
            IntBox::from_coords(
                c.lower_left_diagonal_x - c.bottom_y,
                p_d.ll.y,
                c.lower_right_diagonal_x + c.bottom_y,
                c.bottom_y,
            ),
            // upper box
            IntBox::from_coords(
                c.upper_left_diagonal_x + c.top_y,
                c.top_y,
                c.upper_right_diagonal_x - c.top_y,
                p_d.ur.y,
            ),
        ];

        const CRIT: i32 = limits::CRIT_INT;
        let mut octagons = [
            // upper left octagon
            IntOctagon::new(
                p_d.ll.x,
                boxes[0].ur.y,
                boxes[3].ll.x,
                p_d.ur.y,
                -CRIT,
                c.upper_left_diagonal_x,
                -CRIT,
                CRIT,
            )
            .normalize(),
            // lower left octagon
            IntOctagon::new(
                p_d.ll.x,
                p_d.ll.y,
                boxes[2].ll.x,
                boxes[0].ll.y,
                -CRIT,
                CRIT,
                -CRIT,
                c.lower_left_diagonal_x,
            )
            .normalize(),
            // lower right octagon
            IntOctagon::new(
                boxes[2].ur.x,
                p_d.ll.y,
                p_d.ur.x,
                boxes[1].ll.y,
                c.lower_right_diagonal_x,
                CRIT,
                -CRIT,
                CRIT,
            )
            .normalize(),
            // upper right octagon
            IntOctagon::new(
                boxes[3].ur.x,
                boxes[1].ur.y,
                p_d.ur.x,
                p_d.ur.y,
                -CRIT,
                CRIT,
                c.upper_right_diagonal_x,
                CRIT,
            )
            .normalize(),
        ];

        // Optimize the result to minimum cumulative circumference. Each
        // step may move a divide line between a box and its neighbor corner
        // octagon.
        let (b, o) = (boxes[0], octagons[0]);
        if b.ur.x - b.ll.x > o.top_y - o.bottom_y {
            // switch the horizontal upper left divide line to vertical
            boxes[0] = IntBox::from_coords(b.ll.x, b.ll.y, b.ur.x, o.top_y);
            octagons[0] = IntOctagon::new(
                b.ur.x,
                o.bottom_y,
                o.right_x,
                o.top_y,
                o.upper_left_diagonal_x,
                o.lower_right_diagonal_x,
                o.lower_left_diagonal_x,
                o.upper_right_diagonal_x,
            )
            .normalize();
        }
        let (b, o) = (boxes[3], octagons[0]);
        if b.ur.y - b.ll.y > o.right_x - o.left_x {
            // switch the vertical upper left divide line to horizontal
            boxes[3] = IntBox::from_coords(o.left_x, b.ll.y, b.ur.x, b.ur.y);
            octagons[0] = IntOctagon::new(
                o.left_x,
                o.bottom_y,
                o.right_x,
                b.ll.y,
                o.upper_left_diagonal_x,
                o.lower_right_diagonal_x,
                o.lower_left_diagonal_x,
                o.upper_right_diagonal_x,
            )
            .normalize();
        }
        let (b, o) = (boxes[3], octagons[3]);
        if b.ur.y - b.ll.y > o.right_x - o.left_x {
            // switch the vertical upper right divide line to horizontal
            boxes[3] = IntBox::from_coords(b.ll.x, b.ll.y, o.right_x, b.ur.y);
            octagons[3] = IntOctagon::new(
                o.left_x,
                o.bottom_y,
                o.right_x,
                o.top_y,
                o.upper_left_diagonal_x,
                o.lower_right_diagonal_x,
                o.lower_left_diagonal_x,
                o.upper_right_diagonal_x,
            )
            .normalize();
        }
        let (b, o) = (boxes[1], octagons[3]);
        if b.ur.x - b.ll.x > o.top_y - o.bottom_y {
            // switch the horizontal upper right divide line to vertical
            boxes[1] = IntBox::from_coords(b.ll.x, b.ll.y, b.ur.x, o.top_y);
            octagons[3] = IntOctagon::new(
                o.left_x,
                o.bottom_y,
                b.ll.x,
                o.top_y,
                o.upper_left_diagonal_x,
                o.lower_right_diagonal_x,
                o.lower_left_diagonal_x,
                o.upper_right_diagonal_x,
            )
            .normalize();
        }
        let (b, o) = (boxes[1], octagons[2]);
        if b.ur.x - b.ll.x > o.top_y - o.bottom_y {
            // switch the horizontal lower right divide line to vertical
            boxes[1] = IntBox::from_coords(b.ll.x, o.bottom_y, b.ur.x, b.ur.y);
            octagons[2] = IntOctagon::new(
                o.left_x,
                o.bottom_y,
                b.ll.x,
                o.top_y,
                o.upper_left_diagonal_x,
                o.lower_right_diagonal_x,
                o.lower_left_diagonal_x,
                o.upper_right_diagonal_x,
            )
            .normalize();
        }
        let (b, o) = (boxes[2], octagons[2]);
        if b.ur.y - b.ll.y > o.right_x - o.left_x {
            // switch the vertical lower right divide line to horizontal
            boxes[2] = IntBox::from_coords(b.ll.x, b.ll.y, o.right_x, b.ur.y);
            octagons[2] = IntOctagon::new(
                o.left_x,
                b.ur.y,
                o.right_x,
                o.top_y,
                o.upper_left_diagonal_x,
                o.lower_right_diagonal_x,
                o.lower_left_diagonal_x,
                o.upper_right_diagonal_x,
            )
            .normalize();
        }
        let (b, o) = (boxes[2], octagons[1]);
        if b.ur.y - b.ll.y > o.right_x - o.left_x {
            // switch the vertical lower left divide line to horizontal
            boxes[2] = IntBox::from_coords(o.left_x, b.ll.y, b.ur.x, b.ur.y);
            octagons[1] = IntOctagon::new(
                o.left_x,
                b.ur.y,
                o.right_x,
                o.top_y,
                o.upper_left_diagonal_x,
                o.lower_right_diagonal_x,
                o.lower_left_diagonal_x,
                o.upper_right_diagonal_x,
            )
            .normalize();
        }
        let (b, o) = (boxes[0], octagons[1]);
        if b.ur.x - b.ll.x > o.top_y - o.bottom_y {
            // switch the horizontal lower left divide line to vertical
            boxes[0] = IntBox::from_coords(b.ll.x, o.bottom_y, b.ur.x, b.ur.y);
            octagons[1] = IntOctagon::new(
                b.ur.x,
                o.bottom_y,
                o.right_x,
                o.top_y,
                o.upper_left_diagonal_x,
                o.lower_right_diagonal_x,
                o.lower_left_diagonal_x,
                o.upper_right_diagonal_x,
            )
            .normalize();
        }

        boxes
            .iter()
            .map(|b| b.to_int_octagon())
            .chain(octagons)
            .collect()
    }

    /// Divides `from_octagon` minus this octagon into 8 convex pieces
    /// without sharp angles.
    pub fn cutout_from(&self, p_d: IntOctagon) -> Vec<IntOctagon> {
        let c = self.intersection(p_d);
        if self.is_empty() || c.dimension() < self.dimension() {
            // there is only an overlap at the border
            return vec![p_d];
        }

        let mut result = [IntOctagon::EMPTY; 8];

        let tmp = c.lower_left_diagonal_x - c.left_x;
        result[0] = IntOctagon::new(
            p_d.left_x,
            tmp,
            c.left_x,
            c.left_x - c.upper_left_diagonal_x,
            p_d.upper_left_diagonal_x,
            p_d.lower_right_diagonal_x,
            p_d.lower_left_diagonal_x,
            p_d.upper_right_diagonal_x,
        );
        let tmp2 = c.lower_left_diagonal_x - c.bottom_y;
        result[1] = IntOctagon::new(
            p_d.left_x,
            p_d.bottom_y,
            tmp2,
            tmp,
            p_d.upper_left_diagonal_x,
            p_d.lower_right_diagonal_x,
            p_d.lower_left_diagonal_x,
            c.lower_left_diagonal_x,
        );
        let tmp = c.lower_right_diagonal_x + c.bottom_y;
        result[2] = IntOctagon::new(
            tmp2,
            p_d.bottom_y,
            tmp,
            c.bottom_y,
            p_d.upper_left_diagonal_x,
            p_d.lower_right_diagonal_x,
            p_d.lower_left_diagonal_x,
            p_d.upper_right_diagonal_x,
        );
        let tmp2 = c.right_x - c.lower_right_diagonal_x;
        result[3] = IntOctagon::new(
            tmp,
            p_d.bottom_y,
            p_d.right_x,
            tmp2,
            c.lower_right_diagonal_x,
            p_d.lower_right_diagonal_x,
            p_d.lower_left_diagonal_x,
            p_d.upper_right_diagonal_x,
        );
        let tmp = c.upper_right_diagonal_x - c.right_x;
        result[4] = IntOctagon::new(
            c.right_x,
            tmp2,
            p_d.right_x,
            tmp,
            p_d.upper_left_diagonal_x,
            p_d.lower_right_diagonal_x,
            p_d.lower_left_diagonal_x,
            p_d.upper_right_diagonal_x,
        );
        let tmp2 = c.upper_right_diagonal_x - c.top_y;
        result[5] = IntOctagon::new(
            tmp2,
            tmp,
            p_d.right_x,
            p_d.top_y,
            p_d.upper_left_diagonal_x,
            p_d.lower_right_diagonal_x,
            c.upper_right_diagonal_x,
            p_d.upper_right_diagonal_x,
        );
        let tmp = c.upper_left_diagonal_x + c.top_y;
        result[6] = IntOctagon::new(
            tmp,
            c.top_y,
            tmp2,
            p_d.top_y,
            p_d.upper_left_diagonal_x,
            p_d.lower_right_diagonal_x,
            p_d.lower_left_diagonal_x,
            p_d.upper_right_diagonal_x,
        );
        let tmp2 = c.left_x - c.upper_left_diagonal_x;
        result[7] = IntOctagon::new(
            p_d.left_x,
            tmp2,
            tmp,
            p_d.top_y,
            p_d.upper_left_diagonal_x,
            c.upper_left_diagonal_x,
            p_d.lower_left_diagonal_x,
            p_d.upper_right_diagonal_x,
        );

        for piece in result.iter_mut() {
            *piece = piece.normalize();
        }

        // Optimize the divide lines between neighboring pieces for minimal
        // cumulative circumference, walking around the octagon.
        let (curr_1, curr_2) = (result[0], result[7]);
        if !(curr_1.is_empty() || curr_2.is_empty())
            && curr_1.right_x - curr_1.left_x_value(curr_1.top_y)
                > curr_2.upper_y_value(curr_1.right_x) - curr_2.bottom_y
        {
            // switch the horizontal upper left divide line to vertical
            let new_1 = IntOctagon::new(
                curr_1.left_x.min(curr_2.left_x),
                curr_1.bottom_y,
                curr_1.right_x,
                curr_2.top_y,
                curr_2.upper_left_diagonal_x,
                curr_1.lower_right_diagonal_x,
                curr_1.lower_left_diagonal_x,
                curr_2.upper_right_diagonal_x,
            );
            let new_2 = IntOctagon::new(
                new_1.right_x,
                curr_2.bottom_y,
                curr_2.right_x,
                curr_2.top_y,
                curr_2.upper_left_diagonal_x,
                curr_2.lower_right_diagonal_x,
                curr_2.lower_left_diagonal_x,
                curr_2.upper_right_diagonal_x,
            );
            result[0] = new_1.normalize();
            result[7] = new_2.normalize();
        }
        let (curr_1, curr_2) = (result[7], result[6]);
        if !(curr_1.is_empty() || curr_2.is_empty())
            && curr_2.upper_y_value(curr_1.right_x) - curr_2.bottom_y
                > curr_1.right_x - curr_1.left_x_value(curr_2.bottom_y)
        {
            // switch the vertical upper left divide line to horizontal
            let new_2 = IntOctagon::new(
                curr_1.left_x,
                curr_2.bottom_y,
                curr_2.right_x,
                curr_2.top_y.max(curr_1.top_y),
                curr_1.upper_left_diagonal_x,
                curr_2.lower_right_diagonal_x,
                curr_1.lower_left_diagonal_x,
                curr_2.upper_right_diagonal_x,
            );
            let new_1 = IntOctagon::new(
                curr_1.left_x,
                curr_1.bottom_y,
                curr_1.right_x,
                new_2.bottom_y,
                curr_1.upper_left_diagonal_x,
                curr_1.lower_right_diagonal_x,
                curr_1.lower_left_diagonal_x,
                curr_1.upper_right_diagonal_x,
            );
            result[7] = new_1.normalize();
            result[6] = new_2.normalize();
        }
        let (curr_1, curr_2) = (result[6], result[5]);
        if !(curr_1.is_empty() || curr_2.is_empty())
            && curr_2.upper_y_value(curr_1.right_x) - curr_1.bottom_y
                > curr_2.right_x_value(curr_1.bottom_y) - curr_2.left_x
        {
            // switch the vertical upper right divide line to horizontal
            let new_1 = IntOctagon::new(
                curr_1.left_x,
                curr_1.bottom_y,
                curr_2.right_x,
                curr_2.top_y.max(curr_1.top_y),
                curr_1.upper_left_diagonal_x,
                curr_2.lower_right_diagonal_x,
                curr_1.lower_left_diagonal_x,
                curr_2.upper_right_diagonal_x,
            );
            let new_2 = IntOctagon::new(
                curr_2.left_x,
                curr_2.bottom_y,
                curr_2.right_x,
                new_1.bottom_y,
                curr_2.upper_left_diagonal_x,
                curr_2.lower_right_diagonal_x,
                curr_2.lower_left_diagonal_x,
                curr_2.upper_right_diagonal_x,
            );
            result[6] = new_1.normalize();
            result[5] = new_2.normalize();
        }
        let (curr_1, curr_2) = (result[5], result[4]);
        if !(curr_1.is_empty() || curr_2.is_empty())
            && curr_2.right_x_value(curr_2.top_y) - curr_2.left_x
                > curr_1.upper_y_value(curr_2.left_x) - curr_2.top_y
        {
            // switch the horizontal upper right divide line to vertical
            let new_2 = IntOctagon::new(
                curr_2.left_x,
                curr_2.bottom_y,
                curr_2.right_x.max(curr_1.right_x),
                curr_1.top_y,
                curr_1.upper_left_diagonal_x,
                curr_2.lower_right_diagonal_x,
                curr_2.lower_left_diagonal_x,
                curr_1.upper_right_diagonal_x,
            );
            let new_1 = IntOctagon::new(
                curr_1.left_x,
                curr_1.bottom_y,
                new_2.left_x,
                curr_1.top_y,
                curr_1.upper_left_diagonal_x,
                curr_1.lower_right_diagonal_x,
                curr_1.lower_left_diagonal_x,
                curr_1.upper_right_diagonal_x,
            );
            result[5] = new_1.normalize();
            result[4] = new_2.normalize();
        }
        let (curr_1, curr_2) = (result[4], result[3]);
        if !(curr_1.is_empty() || curr_2.is_empty())
            && curr_1.right_x_value(curr_1.bottom_y) - curr_1.left_x
                > curr_1.bottom_y - curr_2.lower_y_value(curr_1.left_x)
        {
            // switch the horizontal lower right divide line to vertical
            let new_1 = IntOctagon::new(
                curr_1.left_x,
                curr_2.bottom_y,
                curr_2.right_x.max(curr_1.right_x),
                curr_1.top_y,
                curr_1.upper_left_diagonal_x,
                curr_2.lower_right_diagonal_x,
                curr_2.lower_left_diagonal_x,
                curr_1.upper_right_diagonal_x,
            );
            let new_2 = IntOctagon::new(
                curr_2.left_x,
                curr_2.bottom_y,
                new_1.left_x,
                curr_2.top_y,
                curr_2.upper_left_diagonal_x,
                curr_2.lower_right_diagonal_x,
                curr_2.lower_left_diagonal_x,
                curr_2.upper_right_diagonal_x,
            );
            result[4] = new_1.normalize();
            result[3] = new_2.normalize();
        }
        let (curr_1, curr_2) = (result[3], result[2]);
        if !(curr_1.is_empty() || curr_2.is_empty())
            && curr_2.top_y - curr_2.lower_y_value(curr_2.right_x)
                > curr_1.right_x_value(curr_2.top_y) - curr_2.right_x
        {
            // switch the vertical lower right divide line to horizontal
            let new_2 = IntOctagon::new(
                curr_2.left_x,
                curr_1.bottom_y.min(curr_2.bottom_y),
                curr_1.right_x,
                curr_2.top_y,
                curr_2.upper_left_diagonal_x,
                curr_1.lower_right_diagonal_x,
                curr_2.lower_left_diagonal_x,
                curr_1.upper_right_diagonal_x,
            );
            let new_1 = IntOctagon::new(
                curr_1.left_x,
                new_2.top_y,
                curr_1.right_x,
                curr_1.top_y,
                curr_1.upper_left_diagonal_x,
                curr_1.lower_right_diagonal_x,
                curr_1.lower_left_diagonal_x,
                curr_1.upper_right_diagonal_x,
            );
            result[3] = new_1.normalize();
            result[2] = new_2.normalize();
        }
        let (curr_1, curr_2) = (result[2], result[1]);
        if !(curr_1.is_empty() || curr_2.is_empty())
            && curr_1.top_y - curr_1.lower_y_value(curr_1.left_x)
                > curr_1.left_x - curr_2.left_x_value(curr_1.top_y)
        {
            // switch the vertical lower left divide line to horizontal
            let new_1 = IntOctagon::new(
                curr_2.left_x,
                curr_1.bottom_y.min(curr_2.bottom_y),
                curr_1.right_x,
                curr_1.top_y,
                curr_2.upper_left_diagonal_x,
                curr_1.lower_right_diagonal_x,
                curr_2.lower_left_diagonal_x,
                curr_1.upper_right_diagonal_x,
            );
            let new_2 = IntOctagon::new(
                curr_2.left_x,
                new_1.top_y,
                curr_2.right_x,
                curr_2.top_y,
                curr_2.upper_left_diagonal_x,
                curr_2.lower_right_diagonal_x,
                curr_2.lower_left_diagonal_x,
                curr_2.upper_right_diagonal_x,
            );
            result[2] = new_1.normalize();
            result[1] = new_2.normalize();
        }
        let (curr_1, curr_2) = (result[1], result[0]);
        if !(curr_1.is_empty() || curr_2.is_empty())
            && curr_2.right_x - curr_2.left_x_value(curr_2.bottom_y)
                > curr_2.bottom_y - curr_1.lower_y_value(curr_2.right_x)
        {
            // switch the horizontal lower left divide line to vertical
            let new_2 = IntOctagon::new(
                curr_2.left_x.min(curr_1.left_x),
                curr_1.bottom_y,
                curr_2.right_x,
                curr_2.top_y,
                curr_2.upper_left_diagonal_x,
                curr_1.lower_right_diagonal_x,
                curr_1.lower_left_diagonal_x,
                curr_2.upper_right_diagonal_x,
            );
            let new_1 = IntOctagon::new(
                new_2.right_x,
                curr_1.bottom_y,
                curr_1.right_x,
                curr_1.top_y,
                curr_1.upper_left_diagonal_x,
                curr_1.lower_right_diagonal_x,
                curr_1.lower_left_diagonal_x,
                curr_1.upper_right_diagonal_x,
            );
            result[1] = new_1.normalize();
            result[0] = new_2.normalize();
        }

        result.to_vec()
    }
}

impl IntBox {
    /// The [`IntOctagon`] defining the same shape.
    pub fn to_int_octagon(&self) -> IntOctagon {
        IntOctagon::new(
            self.ll.x,
            self.ll.y,
            self.ur.x,
            self.ur.y,
            self.ll.x - self.ur.y,
            self.ur.x - self.ll.y,
            self.ll.x + self.ll.y,
            self.ur.x + self.ur.y,
        )
    }

    /// Enlarges the box by `offset`; contrary to `offset()` the result is an
    /// [`IntOctagon`].
    pub fn enlarge(&self, offset: f64) -> IntOctagon {
        self.to_int_octagon().offset(offset)
    }
}

impl IntPoint {
    /// The smallest octagon containing this point.
    pub fn surrounding_octagon(self) -> IntOctagon {
        let tmp_1 = self.x - self.y;
        let tmp_2 = self.x + self.y;
        IntOctagon::new(self.x, self.y, self.x, self.y, tmp_1, tmp_1, tmp_2, tmp_2)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn diamond(radius: i32) -> IntOctagon {
        // |x| + |y| <= radius
        IntOctagon::new(
            -radius, -radius, radius, radius, -radius, radius, -radius, radius,
        )
        .normalize()
    }

    #[test]
    fn box_octagon_roundtrip() {
        let b = IntBox::from_coords(-2, -1, 3, 4);
        let oct = b.to_int_octagon();
        assert!(oct.is_int_box());
        assert!(oct.is_normalized());
        assert_eq!(oct.bounding_box(), b);
        assert!((oct.area() - b.area()).abs() < 1e-12);
        assert_eq!(oct.dimension(), 2);
    }

    #[test]
    fn normalize_clips_redundant_borders() {
        let d = diamond(4);
        // The straight borders get pulled onto the diamond.
        assert_eq!(d.left_x, -4);
        assert_eq!(d.corner(0), IntPoint::new(0, -4));
        assert!((d.area() - 32.0).abs() < 1e-12);
        assert!(d.is_normalized());
        // An inverted octagon normalizes to EMPTY.
        assert!(IntOctagon::new(5, 0, -5, 0, 0, 0, 0, 0)
            .normalize()
            .is_empty());
        assert_eq!(IntOctagon::EMPTY.dimension(), -1);
    }

    #[test]
    fn containment_and_sides() {
        let d = diamond(4);
        assert!(d.contains(IntPoint::new(0, 0)));
        assert!(d.contains(IntPoint::new(2, 2)));
        assert!(!d.contains(IntPoint::new(3, 2)));
        assert!(d.contains_float(FloatPoint::new(1.5, 1.5)));
        assert!(!d.contains_float(FloatPoint::new(3.0, 1.5)));
        // Interior point is on the left of every border line.
        for i in 0..8 {
            assert_eq!(d.side_of_border_line(0, 0, i), Side::OnTheLeft);
            assert_eq!(
                d.border_line(i).side_of_int(IntPoint::new(0, 0)),
                Side::OnTheRight
            );
        }
        // Corner points are collinear on their adjacent borders.
        assert_eq!(d.side_of_border_line(4, 0, 1), Side::Collinear);
        assert_eq!(d.side_of_border_line(4, 0, 3), Side::Collinear);
    }

    #[test]
    fn intersection_union_intersects() {
        let d = diamond(4);
        let b = IntBox::from_coords(0, 0, 10, 10).to_int_octagon();
        let is = d.intersection(b);
        assert!(!is.is_empty());
        assert!(is.is_contained_in(d));
        assert!(is.is_contained_in(b));
        assert!(d.intersects(b));
        assert!(d.overlaps(b));
        let far = IntBox::from_coords(10, 10, 20, 20).to_int_octagon();
        assert!(!d.intersects(far));
        assert!(d.union(far).is_contained_in(diamond(100).union(far)));
        // Touching only at corner (4,0)/(4,4)-ish: intersects but no overlap.
        let touching = IntBox::from_coords(4, 0, 8, 4).to_int_octagon();
        assert!(d.intersects(touching));
        assert!(!d.overlaps(touching));
    }

    #[test]
    fn boundary_value_functions() {
        let d = diamond(4);
        assert_eq!(d.left_x_value(0), -4);
        assert_eq!(d.right_x_value(2), 2);
        assert_eq!(d.lower_y_value(1), -3);
        assert_eq!(d.upper_y_value(-1), 3);
    }

    #[test]
    fn border_points_and_projections() {
        let d = diamond(4);
        let origin = IntPoint::new(0, 0);
        assert_eq!(
            d.border_point(origin, FortyfiveDegreeDirection::Right),
            IntPoint::new(4, 0)
        );
        assert_eq!(
            d.border_point(origin, FortyfiveDegreeDirection::Up45),
            IntPoint::new(-2, 2)
        );
        let projections = d.nearest_border_projections(origin, 8);
        assert_eq!(projections.len(), 8);
        // The diagonal-direction projections (distance^2 = 8) come before
        // the axis ones (distance^2 = 16).
        let d0 = origin.to_float().distance_square(projections[0].to_float());
        let d7 = origin.to_float().distance_square(projections[7].to_float());
        assert!(d0 <= d7);
        assert!((d0 - 8.0).abs() < 1e-12);
        assert!((d7 - 16.0).abs() < 1e-12);
    }

    #[test]
    fn compare_edges() {
        let a = diamond(4);
        let smaller = diamond(3);
        for edge in 0..8 {
            assert_eq!(a.compare(a, edge), Side::Collinear);
            // Every border of the smaller diamond is further inside.
            assert_eq!(smaller.compare(a, edge), Side::OnTheLeft);
            assert_eq!(a.compare(smaller, edge), Side::OnTheRight);
        }
    }

    #[test]
    fn cutout_covers_complement() {
        let hole = diamond(2);
        let outer_box = IntBox::from_coords(-6, -6, 6, 6);
        let pieces = hole.cutout_from_box(outer_box);
        assert_eq!(pieces.len(), 8);
        let pieces_area: f64 = pieces.iter().map(|p| p.area()).sum();
        assert!(
            (pieces_area - (outer_box.area() - hole.area())).abs() < 1e-9,
            "area mismatch: {pieces_area}"
        );
        for p in &pieces {
            assert!(p.is_empty() || !p.overlaps(hole));
            assert!(p.is_contained_in_box(outer_box));
        }

        let outer_oct = diamond(6);
        let pieces = hole.cutout_from(outer_oct);
        assert_eq!(pieces.len(), 8);
        let pieces_area: f64 = pieces.iter().map(|p| p.area()).sum();
        assert!(
            (pieces_area - (outer_oct.area() - hole.area())).abs() < 1e-9,
            "area mismatch: {pieces_area}"
        );
        for p in &pieces {
            assert!(p.is_empty() || !p.overlaps(hole));
            assert!(p.is_empty() || p.is_contained_in(outer_oct));
        }
        // Non-overlapping cut: divide shape returned unchanged.
        let far = diamond(2).translate_by(IntVector::new(20, 0));
        assert_eq!(far.cutout_from(outer_oct), vec![outer_oct]);
    }

    #[test]
    fn offset_and_translate() {
        let d = diamond(4);
        let grown = d.offset(1.0);
        assert!(d.is_contained_in(grown));
        assert!(grown.contains(IntPoint::new(5, 0)));
        let moved = d.translate_by(IntVector::new(3, -2));
        assert!(moved.contains(IntPoint::new(3, -2)));
        assert!(moved.is_normalized());
        assert_eq!(
            IntPoint::new(2, 3).surrounding_octagon().corner(0),
            IntPoint::new(2, 3)
        );
    }
}
