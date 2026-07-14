//! Port of `geometry/planar/IntBox.java` (box/box operations).
//!
//! Methods involving `IntOctagon`, `Simplex`, `TileShape` or
//! `ShapeBoundingDirections` (to_IntOctagon, to_Simplex, enlarge,
//! bounding_octagon, cutout, cross-shape intersections/compares) will be
//! added together with those types.

use crate::geometry::planar::{limits, FloatPoint, IntPoint, IntVector, Line, Side};

/// An orthogonal rectangle in the plane with integer coordinates, defined by
/// its lower left and upper right corners.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct IntBox {
    /// the lower left corner
    pub ll: IntPoint,
    /// the upper right corner
    pub ur: IntPoint,
}

impl IntBox {
    /// Standard implementation of an empty box.
    pub const EMPTY: IntBox = IntBox::from_coords(
        limits::CRIT_INT,
        limits::CRIT_INT,
        -limits::CRIT_INT,
        -limits::CRIT_INT,
    );

    pub const fn new(ll: IntPoint, ur: IntPoint) -> Self {
        IntBox { ll, ur }
    }

    pub const fn from_coords(ll_x: i32, ll_y: i32, ur_x: i32, ur_y: i32) -> Self {
        IntBox {
            ll: IntPoint::new(ll_x, ll_y),
            ur: IntPoint::new(ur_x, ur_y),
        }
    }

    pub fn is_empty(&self) -> bool {
        self.ll.x > self.ur.x || self.ll.y > self.ur.y
    }

    pub fn border_line_count(&self) -> usize {
        4
    }

    /// The horizontal extension of the box.
    pub fn width(&self) -> i32 {
        self.ur.x - self.ll.x
    }

    /// The vertical extension of the box.
    pub fn height(&self) -> i32 {
        self.ur.y - self.ll.y
    }

    pub fn max_width(&self) -> f64 {
        self.width().max(self.height()) as f64
    }

    pub fn min_width(&self) -> f64 {
        self.width().min(self.height()) as f64
    }

    pub fn area(&self) -> f64 {
        self.width() as f64 * self.height() as f64
    }

    pub fn circumference(&self) -> f64 {
        (2 * (self.width() + self.height())) as f64
    }

    /// The corners in counterclockwise order starting at the lower left.
    pub fn corner(&self, no: usize) -> IntPoint {
        match no {
            0 => self.ll,
            1 => IntPoint::new(self.ur.x, self.ll.y),
            2 => self.ur,
            3 => IntPoint::new(self.ll.x, self.ur.y),
            _ => panic!("IntBox::corner: no out of range"),
        }
    }

    /// -1 if the box is empty, 0 for a single point, 1 for a degenerate
    /// line, 2 for a real area.
    pub fn dimension(&self) -> i32 {
        if self.is_empty() {
            -1
        } else if self.ll == self.ur {
            0
        } else if self.ur.x == self.ll.x || self.ll.y == self.ur.y {
            1
        } else {
            2
        }
    }

    /// Checks if `point` is located in this box (border included).
    pub fn contains(&self, point: IntPoint) -> bool {
        point.x >= self.ll.x && point.y >= self.ll.y && point.x <= self.ur.x && point.y <= self.ur.y
    }

    /// Checks if `point` is located in the interior of this box.
    pub fn contains_inside(&self, point: IntPoint) -> bool {
        point.x > self.ll.x && point.x < self.ur.x && point.y > self.ll.y && point.y < self.ur.y
    }

    /// The nearest point of this box to `from_point`.
    pub fn nearest_point(&self, from_point: FloatPoint) -> FloatPoint {
        FloatPoint::new(
            from_point.x.clamp(self.ll.x as f64, self.ur.x as f64),
            from_point.y.clamp(self.ll.y as f64, self.ur.y as f64),
        )
    }

    /// The sorted `max_result_points` (at most 2) nearest points on the
    /// border of this box to `point`, which is assumed to be in the
    /// interior.
    pub fn nearest_border_projections(
        &self,
        point: IntPoint,
        max_result_points: usize,
    ) -> Vec<IntPoint> {
        if max_result_points == 0 {
            return Vec::new();
        }
        let max_result_points = max_result_points.min(2);

        let lower_x_diff = point.x - self.ll.x;
        let upper_x_diff = self.ur.x - point.x;
        let lower_y_diff = point.y - self.ll.y;
        let upper_y_diff = self.ur.y - point.y;

        let (mut min_diff, mut second_min_diff);
        let mut nearest = point;
        let mut second_nearest = point;
        if lower_x_diff <= upper_x_diff {
            min_diff = lower_x_diff;
            second_min_diff = upper_x_diff;
            nearest.x = self.ll.x;
            second_nearest.x = self.ur.x;
        } else {
            min_diff = upper_x_diff;
            second_min_diff = lower_x_diff;
            nearest.x = self.ur.x;
            second_nearest.x = self.ll.x;
        }
        if lower_y_diff < min_diff {
            second_min_diff = min_diff;
            min_diff = lower_y_diff;
            second_nearest = nearest;
            nearest = IntPoint::new(point.x, self.ll.y);
        } else if lower_y_diff < second_min_diff {
            second_min_diff = lower_y_diff;
            second_nearest = IntPoint::new(point.x, self.ll.y);
        }
        if upper_y_diff < min_diff {
            second_nearest = nearest;
            nearest = IntPoint::new(point.x, self.ur.y);
        } else if upper_y_diff < second_min_diff {
            second_nearest = IntPoint::new(point.x, self.ur.y);
        }

        let mut result = vec![nearest];
        if max_result_points > 1 {
            result.push(second_nearest);
        }
        result
    }

    /// The distance of this box to `from_point`.
    pub fn distance(&self, from_point: FloatPoint) -> f64 {
        from_point.distance(self.nearest_point(from_point))
    }

    /// The weighted distance to the box `other`.
    pub fn weighted_distance(
        &self,
        other: IntBox,
        horizontal_weight: f64,
        vertical_weight: f64,
    ) -> f64 {
        let max_ll_x = self.ll.x.max(other.ll.x) as f64;
        let max_ll_y = self.ll.y.max(other.ll.y) as f64;
        let min_ur_x = self.ur.x.min(other.ur.x) as f64;
        let min_ur_y = self.ur.y.min(other.ur.y) as f64;

        if min_ur_x >= max_ll_x {
            (vertical_weight * (max_ll_y - min_ur_y)).max(0.0)
        } else if min_ur_y >= max_ll_y {
            (horizontal_weight * (max_ll_x - min_ur_x)).max(0.0)
        } else {
            let delta_x = (max_ll_x - min_ur_x) * horizontal_weight;
            let delta_y = (max_ll_y - min_ur_y) * vertical_weight;
            (delta_x * delta_x + delta_y * delta_y).sqrt()
        }
    }

    pub fn bounding_box(&self) -> IntBox {
        *self
    }

    pub fn is_bounded(&self) -> bool {
        true
    }

    /// The smallest box containing this box and `other`.
    pub fn union(&self, other: IntBox) -> IntBox {
        IntBox::from_coords(
            self.ll.x.min(other.ll.x),
            self.ll.y.min(other.ll.y),
            self.ur.x.max(other.ur.x),
            self.ur.y.max(other.ur.y),
        )
    }

    /// The intersection of this box with `other`.
    pub fn intersection(&self, other: IntBox) -> IntBox {
        if other.ll.x > self.ur.x
            || other.ll.y > self.ur.y
            || self.ll.x > other.ur.x
            || self.ll.y > other.ur.y
        {
            return IntBox::EMPTY;
        }
        IntBox::from_coords(
            self.ll.x.max(other.ll.x),
            self.ll.y.max(other.ll.y),
            self.ur.x.min(other.ur.x),
            self.ur.y.min(other.ur.y),
        )
    }

    pub fn intersects(&self, other: IntBox) -> bool {
        !(other.ll.x > self.ur.x
            || other.ll.y > self.ur.y
            || self.ll.x > other.ur.x
            || self.ll.y > other.ur.y)
    }

    /// True if this box intersects `other` with a 2-dimensional
    /// intersection.
    pub fn overlaps(&self, other: IntBox) -> bool {
        !(other.ll.x >= self.ur.x
            || other.ll.y >= self.ur.y
            || self.ll.x >= other.ur.x
            || self.ll.y >= other.ur.y)
    }

    pub fn translate_by(&self, vector: IntVector) -> IntBox {
        if vector.is_zero() {
            return *self;
        }
        IntBox::new(self.ll.translate_by(vector), self.ur.translate_by(vector))
    }

    pub fn turn_90_degree(&self, factor: i32, pole: IntPoint) -> IntBox {
        let rot = |p: IntPoint| pole.translate_by(p.difference_by(pole).turn_90_degree(factor));
        let p1 = rot(self.ll);
        let p2 = rot(self.ur);
        IntBox::from_coords(
            p1.x.min(p2.x),
            p1.y.min(p2.y),
            p1.x.max(p2.x),
            p1.y.max(p2.y),
        )
    }

    /// The border lines in counterclockwise order starting with the lower
    /// boundary; each is directed so the box interior is on its left.
    pub fn border_line(&self, no: usize) -> Line {
        match no {
            0 => Line::from_coords(0, self.ll.y, 1, self.ll.y),
            1 => Line::from_coords(self.ur.x, 0, self.ur.x, 1),
            2 => Line::from_coords(0, self.ur.y, -1, self.ur.y),
            3 => Line::from_coords(self.ll.x, 0, self.ll.x, -1),
            _ => panic!("IntBox::border_line: no out of range"),
        }
    }

    /// The box offset by `dist`: outward if `dist` > 0, else inward.
    pub fn offset(&self, dist: f64) -> IntBox {
        if dist == 0.0 || self.is_empty() {
            return *self;
        }
        let dist = dist.round() as i32;
        IntBox::from_coords(
            self.ll.x - dist,
            self.ll.y - dist,
            self.ur.x + dist,
            self.ur.y + dist,
        )
    }

    /// Offsets only the horizontal boundary.
    pub fn horizontal_offset(&self, dist: f64) -> IntBox {
        if dist == 0.0 || self.is_empty() {
            return *self;
        }
        let dist = dist.round() as i32;
        IntBox::from_coords(self.ll.x - dist, self.ll.y, self.ur.x + dist, self.ur.y)
    }

    /// Offsets only the vertical boundary.
    pub fn vertical_offset(&self, dist: f64) -> IntBox {
        if dist == 0.0 || self.is_empty() {
            return *self;
        }
        let dist = dist.round() as i32;
        IntBox::from_coords(self.ll.x, self.ll.y - dist, self.ur.x, self.ur.y + dist)
    }

    /// Shrinks the width and height of the box by `width` on each side; the
    /// box will not vanish completely.
    pub fn shrink(&self, width: i32) -> IntBox {
        let (ll_x, ur_x) = if 2 * width <= self.ur.x - self.ll.x {
            (self.ll.x + width, self.ur.x - width)
        } else {
            let mid = (self.ll.x + self.ur.x) / 2;
            (mid, mid)
        };
        let (ll_y, ur_y) = if 2 * width <= self.ur.y - self.ll.y {
            (self.ll.y + width, self.ur.y - width)
        } else {
            let mid = (self.ll.y + self.ur.y) / 2;
            (mid, mid)
        };
        IntBox::from_coords(ll_x, ll_y, ur_x, ur_y)
    }

    /// Compares the position of the border line `edge_no` of this box with
    /// the corresponding border line of `other` (used by the search trees).
    pub fn compare(&self, other: IntBox, edge_no: usize) -> Side {
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
            0 => cmp(self.ll.y, other.ll.y, true),
            1 => cmp(self.ur.x, other.ur.x, false),
            2 => cmp(self.ur.y, other.ur.y, false),
            3 => cmp(self.ll.x, other.ll.x, true),
            _ => panic!("IntBox::compare: edge_no out of range"),
        }
    }

    pub fn is_contained_in(&self, other: IntBox) -> bool {
        if self.is_empty() {
            return true;
        }
        self.ll.x >= other.ll.x
            && self.ll.y >= other.ll.y
            && self.ur.x <= other.ur.x
            && self.ur.y <= other.ur.y
    }

    /// True if `other` is contained in the interior of this box.
    pub fn contains_in_interior(&self, other: IntBox) -> bool {
        if other.is_empty() {
            return true;
        }
        other.ll.x > self.ll.x
            && other.ll.y > self.ll.y
            && other.ur.x < self.ur.x
            && other.ur.y < self.ur.y
    }

    /// The part of `from_box` which has minimal distance to this box.
    pub fn nearest_part(&self, from_box: IntBox) -> IntBox {
        let ll_x = if from_box.ll.x >= self.ll.x {
            from_box.ll.x
        } else {
            from_box.ur.x.min(self.ll.x)
        };
        let ur_x = if from_box.ur.x <= self.ur.x {
            from_box.ur.x
        } else {
            from_box.ll.x.max(self.ur.x)
        };
        let ll_y = if from_box.ll.y >= self.ll.y {
            from_box.ll.y
        } else {
            from_box.ur.y.min(self.ll.y)
        };
        let ur_y = if from_box.ur.y <= self.ur.y {
            from_box.ur.y
        } else {
            from_box.ll.y.max(self.ur.y)
        };
        IntBox::from_coords(ll_x, ll_y, ur_x, ur_y)
    }

    /// Divides this box into sections of about equal size with width and
    /// height at most `max_section_width`.
    pub fn divide_into_sections(&self, max_section_width: f64) -> Vec<IntBox> {
        if max_section_width <= 0.0 {
            return Vec::new();
        }
        let length = (self.ur.x - self.ll.x) as f64;
        let height = (self.ur.y - self.ll.y) as f64;
        let x_count = (length / max_section_width).ceil() as i32;
        let y_count = (height / max_section_width).ceil() as i32;
        let section_length_x = (length / x_count as f64).ceil() as i32;
        let section_length_y = (height / y_count as f64).ceil() as i32;
        let mut result = Vec::with_capacity((x_count * y_count) as usize);
        for j in 0..y_count {
            let curr_lly = self.ll.y + j * section_length_y;
            let curr_ury = if j == y_count - 1 {
                self.ur.y
            } else {
                curr_lly + section_length_y
            };
            for i in 0..x_count {
                let curr_llx = self.ll.x + i * section_length_x;
                let curr_urx = if i == x_count - 1 {
                    self.ur.x
                } else {
                    curr_llx + section_length_x
                };
                result.push(IntBox::from_coords(curr_llx, curr_lly, curr_urx, curr_ury));
            }
        }
        result
    }

    /// Cuts this box out of `from_box`, dividing the rest into up to 4
    /// boxes; the division is optimized for minimal cumulative
    /// circumference.
    pub fn cutout_from(&self, from_box: IntBox) -> Vec<IntBox> {
        let c = self.intersection(from_box);
        if self.is_empty() || c.dimension() < self.dimension() {
            // there is only an overlap at the border
            return vec![from_box];
        }
        let p_d = from_box;
        let mut result = [
            IntBox::from_coords(p_d.ll.x, p_d.ll.y, c.ur.x, c.ll.y),
            IntBox::from_coords(p_d.ll.x, c.ll.y, c.ll.x, p_d.ur.y),
            IntBox::from_coords(c.ur.x, p_d.ll.y, p_d.ur.x, c.ur.y),
            IntBox::from_coords(c.ll.x, c.ur.y, p_d.ur.x, p_d.ur.y),
        ];

        if c.ll.x - p_d.ll.x > c.ll.y - p_d.ll.y {
            // switch left dividing line to lower
            result[0] = IntBox::from_coords(c.ll.x, result[0].ll.y, result[0].ur.x, result[0].ur.y);
            result[1] = IntBox::from_coords(result[1].ll.x, p_d.ll.y, result[1].ur.x, result[1].ur.y);
        }
        if p_d.ur.y - c.ur.y > c.ll.x - p_d.ll.x {
            // switch upper dividing line to the left
            result[1] = IntBox::from_coords(result[1].ll.x, result[1].ll.y, result[1].ur.x, c.ur.y);
            result[3] = IntBox::from_coords(p_d.ll.x, result[3].ll.y, result[3].ur.x, result[3].ur.y);
        }
        if p_d.ur.x - c.ur.x > p_d.ur.y - c.ur.y {
            // switch right dividing line to upper
            result[2] = IntBox::from_coords(result[2].ll.x, result[2].ll.y, result[2].ur.x, p_d.ur.y);
            result[3] = IntBox::from_coords(result[3].ll.x, result[3].ll.y, c.ur.x, result[3].ur.y);
        }
        if c.ll.y - p_d.ll.y > p_d.ur.x - c.ur.x {
            // switch lower dividing line to the left
            result[0] = IntBox::from_coords(result[0].ll.x, result[0].ll.y, p_d.ur.x, result[0].ur.y);
            result[2] = IntBox::from_coords(result[2].ll.x, c.ll.y, result[2].ur.x, result[2].ur.y);
        }
        result.to_vec()
    }
}

impl IntPoint {
    /// The smallest box containing this point.
    pub fn surrounding_box(self) -> IntBox {
        IntBox::new(self, self)
    }

    pub fn is_contained_in(self, box_: IntBox) -> bool {
        box_.contains(self)
    }
}

impl crate::geometry::planar::RationalPoint {
    /// The smallest integer box containing this point.
    pub fn surrounding_box(&self) -> IntBox {
        let fp = self.to_float();
        IntBox::from_coords(
            fp.x.floor() as i32,
            fp.y.floor() as i32,
            fp.x.ceil() as i32,
            fp.y.ceil() as i32,
        )
    }

    /// Exact containment check via cross-multiplication.
    pub fn is_contained_in(&self, box_: IntBox) -> bool {
        use num_bigint::BigInt;
        let x_ge = |bound: i32| self.x >= BigInt::from(bound) * &self.z;
        let y_ge = |bound: i32| self.y >= BigInt::from(bound) * &self.z;
        let x_le = |bound: i32| self.x <= BigInt::from(bound) * &self.z;
        let y_le = |bound: i32| self.y <= BigInt::from(bound) * &self.z;
        x_ge(box_.ll.x) && y_ge(box_.ll.y) && x_le(box_.ur.x) && y_le(box_.ur.y)
    }
}

impl crate::geometry::planar::Point {
    /// The smallest integer box containing this point.
    pub fn surrounding_box(&self) -> IntBox {
        match self {
            Self::Int(p) => p.surrounding_box(),
            Self::Rational(p) => p.surrounding_box(),
        }
    }

    pub fn is_contained_in(&self, box_: IntBox) -> bool {
        match self {
            Self::Int(p) => p.is_contained_in(box_),
            Self::Rational(p) => p.is_contained_in(box_),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn empty_and_dimension() {
        assert!(IntBox::EMPTY.is_empty());
        assert_eq!(IntBox::EMPTY.dimension(), -1);
        assert_eq!(IntBox::from_coords(1, 1, 1, 1).dimension(), 0);
        assert_eq!(IntBox::from_coords(0, 0, 5, 0).dimension(), 1);
        assert_eq!(IntBox::from_coords(0, 0, 5, 3).dimension(), 2);
    }

    #[test]
    fn union_intersection_overlap() {
        let a = IntBox::from_coords(0, 0, 10, 10);
        let b = IntBox::from_coords(5, 5, 15, 15);
        assert_eq!(a.union(b), IntBox::from_coords(0, 0, 15, 15));
        assert_eq!(a.intersection(b), IntBox::from_coords(5, 5, 10, 10));
        assert!(a.intersects(b));
        assert!(a.overlaps(b));
        // Touching at an edge: intersects but does not overlap.
        let c = IntBox::from_coords(10, 0, 20, 10);
        assert!(a.intersects(c));
        assert!(!a.overlaps(c));
        assert!(a.intersection(IntBox::from_coords(20, 20, 30, 30)).is_empty());
    }

    #[test]
    fn containment_and_nearest() {
        let b = IntBox::from_coords(0, 0, 10, 10);
        assert!(b.contains(IntPoint::new(0, 5)));
        assert!(!b.contains_inside(IntPoint::new(0, 5)));
        assert!(b.contains_inside(IntPoint::new(1, 5)));
        assert!(IntBox::from_coords(2, 2, 8, 8).is_contained_in(b));
        assert!(b.contains_in_interior(IntBox::from_coords(2, 2, 8, 8)));
        assert!(!b.contains_in_interior(IntBox::from_coords(0, 2, 8, 8)));

        let np = b.nearest_point(FloatPoint::new(15.0, 4.0));
        assert_eq!(np, FloatPoint::new(10.0, 4.0));
        assert!((b.distance(FloatPoint::new(13.0, 14.0)) - 5.0).abs() < 1e-12);
        assert_eq!(
            b.nearest_border_projections(IntPoint::new(2, 5), 2),
            vec![IntPoint::new(0, 5), IntPoint::new(2, 0)]
        );
    }

    #[test]
    fn offsets_and_shrink() {
        let b = IntBox::from_coords(0, 0, 10, 10);
        assert_eq!(b.offset(2.0), IntBox::from_coords(-2, -2, 12, 12));
        assert_eq!(b.horizontal_offset(2.0), IntBox::from_coords(-2, 0, 12, 10));
        assert_eq!(b.vertical_offset(2.0), IntBox::from_coords(0, -2, 10, 12));
        assert_eq!(b.shrink(3), IntBox::from_coords(3, 3, 7, 7));
        // Shrinking beyond half the width collapses to the middle.
        assert_eq!(b.shrink(8), IntBox::from_coords(5, 5, 5, 5));
    }

    #[test]
    fn border_lines_have_interior_on_left() {
        let b = IntBox::from_coords(0, 0, 10, 10);
        let inside = IntPoint::new(5, 5);
        for i in 0..4 {
            // An interior point is on the left of each border line, which
            // means each border LINE is on the right of the point.
            assert_eq!(b.border_line(i).side_of_int(inside), Side::OnTheRight);
        }
        assert_eq!(b.corner(1), IntPoint::new(10, 0));
        assert_eq!(b.corner(3), IntPoint::new(0, 10));
    }

    #[test]
    fn compare_edges() {
        let a = IntBox::from_coords(0, 0, 10, 10);
        let higher_lower_edge = IntBox::from_coords(0, 2, 10, 10);
        assert_eq!(a.compare(higher_lower_edge, 0), Side::OnTheRight);
        assert_eq!(higher_lower_edge.compare(a, 0), Side::OnTheLeft);
        assert_eq!(a.compare(a, 1), Side::Collinear);
    }

    #[test]
    fn divide_and_cutout() {
        let b = IntBox::from_coords(0, 0, 10, 10);
        let sections = b.divide_into_sections(5.0);
        assert_eq!(sections.len(), 4);
        let total_area: f64 = sections.iter().map(|s| s.area()).sum();
        assert!((total_area - b.area()).abs() < 1e-12);

        let hole = IntBox::from_coords(4, 4, 6, 6);
        let pieces = hole.cutout_from(b);
        assert_eq!(pieces.len(), 4);
        let pieces_area: f64 = pieces.iter().map(|s| s.area()).sum();
        assert!((pieces_area - (b.area() - hole.area())).abs() < 1e-12);
        for piece in &pieces {
            assert!(!piece.overlaps(hole));
            assert!(piece.is_contained_in(b));
        }
        // Border-only overlap: from_box returned unchanged.
        let outside = IntBox::from_coords(10, 0, 20, 10);
        assert_eq!(outside.cutout_from(b), vec![b]);
    }

    #[test]
    fn transforms_and_distances() {
        let b = IntBox::from_coords(1, 0, 3, 2);
        assert_eq!(
            b.translate_by(IntVector::new(-1, 2)),
            IntBox::from_coords(0, 2, 2, 4)
        );
        assert_eq!(
            b.turn_90_degree(1, IntPoint::new(0, 0)),
            IntBox::from_coords(-2, 1, 0, 3)
        );
        let far = IntBox::from_coords(10, 10, 20, 20);
        assert!((b.weighted_distance(far, 1.0, 1.0) - ((7 * 7 + 8 * 8) as f64).sqrt()).abs() < 1e-12);
        assert_eq!(b.nearest_part(far), IntBox::from_coords(10, 10, 10, 10));
    }
}
