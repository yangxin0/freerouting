//! Port of `geometry/planar/LineSegment.java`.
//!
//! Unlike an (infinite) `Line`, a `LineSegment` has a start and an end
//! point, represented as 3 lines: the segment lives on `middle` and is
//! closed by `start` and `end` (which must not be parallel to `middle`).

use crate::datastructures::Signum;
use crate::geometry::planar::{
    FloatPoint, IntBox, IntOctagon, IntPoint, IntVector, Line, Point, Polyline, Side, Simplex,
    TileShape,
};

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LineSegment {
    start: Line,
    middle: Line,
    end: Line,
}

impl LineSegment {
    /// Creates a line segment from 3 lines: it starts at the intersection
    /// of `start_line` and `middle_line` and ends at the intersection of
    /// `middle_line` and `end_line`.
    pub fn new(start_line: Line, middle_line: Line, end_line: Line) -> Self {
        LineSegment {
            start: start_line,
            middle: middle_line,
            end: end_line,
        }
    }

    /// The `no`-th line segment of `polyline`, for `no` between 1 and
    /// `polyline.arr.len() - 2`.
    pub fn from_polyline(polyline: &Polyline, no: usize) -> Self {
        assert!(
            no >= 1 && no + 1 < polyline.arr.len(),
            "LineSegment from Polyline: no out of range"
        );
        LineSegment {
            start: polyline.arr[no - 1],
            middle: polyline.arr[no],
            end: polyline.arr[no + 1],
        }
    }

    /// The `no`-th border segment of `shape`.
    pub fn from_shape(shape: &TileShape, no: usize) -> Self {
        let line_count = shape.border_line_count();
        assert!(no < line_count, "LineSegment from TileShape: no out of range");
        let start = shape.border_line((no + line_count - 1) % line_count);
        let middle = shape.border_line(no);
        let end = shape.border_line((no + 1) % line_count);
        LineSegment { start, middle, end }
    }

    /// The intersection of the first 2 lines of this segment.
    pub fn start_point(&self) -> Point {
        self.middle.intersection(&self.start)
    }

    /// The intersection of the last 2 lines of this segment.
    pub fn end_point(&self) -> Point {
        self.middle.intersection(&self.end)
    }

    pub fn start_point_approx(&self) -> FloatPoint {
        self.start.intersection_approx(&self.middle)
    }

    pub fn end_point_approx(&self) -> FloatPoint {
        self.end.intersection_approx(&self.middle)
    }

    /// The (infinite) line of this segment.
    pub fn get_line(&self) -> Line {
        self.middle
    }

    pub fn get_start_closing_line(&self) -> Line {
        self.start
    }

    pub fn get_end_closing_line(&self) -> Line {
        self.end
    }

    /// The line segment with the opposite direction.
    pub fn opposite(&self) -> Self {
        LineSegment {
            start: self.end.opposite(),
            middle: self.middle.opposite(),
            end: self.start.opposite(),
        }
    }

    /// Transforms this segment into a polyline of length 3.
    pub fn to_polyline(&self) -> Polyline {
        Polyline::from_lines(vec![self.start, self.middle, self.end])
    }

    /// A 1-dimensional simplex with the same shape as this line segment.
    pub fn to_simplex(&self) -> Simplex {
        let first = if self.end_point().side_of_line(&self.start) == Side::OnTheRight {
            self.start.opposite()
        } else {
            self.start
        };
        let last = if self.start_point().side_of_line(&self.end) == Side::OnTheRight {
            self.end.opposite()
        } else {
            self.end
        };
        Simplex::get_instance(vec![first, self.middle, self.middle.opposite(), last])
    }

    /// Checks if `point` is contained in this line segment.
    pub fn contains(&self, point: IntPoint) -> bool {
        if self.middle.side_of_int(point) != Side::Collinear {
            return false;
        }
        // Create a perpendicular line at the point and check that the two
        // endpoints of this segment are on different sides of it.
        let perpendicular_direction = self.middle.direction().turn_45_degree(2);
        let perpendicular_line = Line::from_direction(point, perpendicular_direction);
        let start_point_side = perpendicular_line.side_of(&self.start_point());
        let end_point_side = perpendicular_line.side_of(&self.end_point());
        start_point_side != end_point_side || start_point_side == Side::Collinear
    }

    /// The smallest surrounding box of this line segment.
    pub fn bounding_box(&self) -> IntBox {
        let start_corner = self.middle.intersection_approx(&self.start);
        let end_corner = self.middle.intersection_approx(&self.end);
        IntBox::from_coords(
            start_corner.x.min(end_corner.x).floor() as i32,
            start_corner.y.min(end_corner.y).floor() as i32,
            start_corner.x.max(end_corner.x).ceil() as i32,
            start_corner.y.max(end_corner.y).ceil() as i32,
        )
    }

    /// The smallest surrounding octagon of this line segment.
    pub fn bounding_octagon(&self) -> IntOctagon {
        let start_corner = self.middle.intersection_approx(&self.start);
        let end_corner = self.middle.intersection_approx(&self.end);
        let start_x_minus_y = start_corner.x - start_corner.y;
        let end_x_minus_y = end_corner.x - end_corner.y;
        let start_x_plus_y = start_corner.x + start_corner.y;
        let end_x_plus_y = end_corner.x + end_corner.y;
        IntOctagon::new(
            start_corner.x.min(end_corner.x).floor() as i32,
            start_corner.y.min(end_corner.y).floor() as i32,
            start_corner.x.max(end_corner.x).ceil() as i32,
            start_corner.y.max(end_corner.y).ceil() as i32,
            start_x_minus_y.min(end_x_minus_y).floor() as i32,
            start_x_minus_y.max(end_x_minus_y).ceil() as i32,
            start_x_plus_y.min(end_x_plus_y).floor() as i32,
            start_x_plus_y.max(end_x_plus_y).ceil() as i32,
        )
        .normalize()
    }

    /// A new segment with the same start and middle line and a new end
    /// line, so that its length is about `new_length`.
    pub fn change_length_approx(&self, new_length: f64) -> Self {
        let new_end_point = self
            .start_point_approx()
            .change_length(self.end_point_approx(), new_length);
        let perpendicular_direction = self.middle.direction().turn_45_degree(2);
        let new_end_line = Line::from_direction(new_end_point.round(), perpendicular_direction);
        LineSegment::new(self.start, self.middle, new_end_line)
    }

    /// The intersections of this segment with `other` as 0, 1 or 2 lines
    /// whose intersections with this segment's line deliver the
    /// intersection points. On overlap, the 2 lines bound the first and
    /// last overlap point.
    pub fn intersection(&self, other: &LineSegment) -> Vec<Line> {
        if !self.bounding_box().intersects(other.bounding_box()) {
            return Vec::new();
        }
        let start_point_side = self.start_point().side_of_line(&other.middle);
        let end_point_side = self.end_point().side_of_line(&other.middle);
        if start_point_side == Side::Collinear && end_point_side == Side::Collinear {
            // there may be an overlap
            let this_sorted = self.sort_endpoints_in_x_y();
            let other_sorted = other.sort_endpoints_in_x_y();
            let (left_line, right_line) = if this_sorted
                .start_point()
                .compare_x_y(&other_sorted.start_point())
                .is_le()
            {
                (this_sorted, other_sorted)
            } else {
                (other_sorted, this_sorted)
            };
            let cmp = left_line.end_point().compare_x_y(&right_line.start_point());
            if cmp.is_lt() {
                // no touch: the segments are disjoint on the common line
                return Vec::new();
            }
            if cmp.is_eq() {
                // they touch at a single point
                return vec![left_line.end];
            }
            // a real overlap
            let second = if right_line
                .end_point()
                .compare_x_y(&left_line.end_point())
                .is_ge()
            {
                left_line.end
            } else {
                right_line.end
            };
            return vec![right_line.start, second];
        }
        if start_point_side == end_point_side
            || other.start_point().side_of_line(&self.middle)
                == other.end_point().side_of_line(&self.middle)
        {
            return Vec::new(); // no intersection possible
        }
        // both pairs of endpoints are on different sides of the other
        // middle line
        vec![other.middle]
    }

    /// Checks if this segment and `other` contain a common point.
    pub fn intersects(&self, other: &LineSegment) -> bool {
        !self.intersection(other).is_empty()
    }

    /// Checks if this segment and `other` share a common segment which is
    /// not reduced to a point.
    pub fn overlaps(&self, other: &LineSegment) -> bool {
        self.intersection(other).len() > 1
    }

    /// An approximation of this segment by orthogonal stairs with integer
    /// coordinates and stair length at most `width`, to the right of the
    /// segment if `to_the_right`, else to the left.
    pub fn stair_approximation(&self, width: f64, to_the_right: bool) -> Vec<IntPoint> {
        let start_point = self.start_point().to_float().round();
        let end_point = self.end_point().to_float().round();
        if start_point == end_point {
            return Vec::new();
        }
        if start_point.x == end_point.x || start_point.y == end_point.y {
            return vec![start_point, end_point];
        }
        let dx = end_point.x - start_point.x;
        let dy = end_point.y - start_point.y;
        let abs_dx = dx.abs();
        let abs_dy = dy.abs();
        // use otherwise a function of y for better numerical stability
        let function_of_x = abs_dx >= abs_dy;

        let (mut stair_width, stair_count);
        if function_of_x {
            stair_width = ((width * abs_dx as f64) / abs_dy as f64).round() as i32;
            stair_count = (abs_dx - 1) / stair_width + 1;
            if end_point.x < start_point.x {
                stair_width = -stair_width;
            }
        } else {
            stair_width = ((width * abs_dy as f64) / abs_dx as f64).round() as i32;
            stair_count = (abs_dy - 1) / stair_width + 1;
            if end_point.y < start_point.y {
                stair_width = -stair_width;
            }
        }
        let mut result = Vec::with_capacity(2 * stair_count as usize + 1);
        result.push(start_point);
        let det = dx as f64 * dy as f64;
        let change_x_first = to_the_right && det > 0.0 || !to_the_right && det < 0.0;

        let mut prev_line_point_x = start_point.x;
        let mut prev_line_point_y = start_point.y;
        for i in 1..stair_count {
            let (curr_line_point_x, curr_line_point_y);
            if function_of_x {
                curr_line_point_x = start_point.x + i * stair_width;
                curr_line_point_y = self
                    .get_line()
                    .function_value_approx(curr_line_point_x as f64)
                    .round() as i32;
            } else {
                curr_line_point_y = start_point.y + i * stair_width;
                curr_line_point_x = self
                    .get_line()
                    .function_in_y_value_approx(curr_line_point_y as f64)
                    .round() as i32;
            }
            if change_x_first {
                result.push(IntPoint::new(curr_line_point_x, prev_line_point_y));
            } else {
                result.push(IntPoint::new(prev_line_point_x, curr_line_point_y));
            }
            result.push(IntPoint::new(curr_line_point_x, curr_line_point_y));
            prev_line_point_x = curr_line_point_x;
            prev_line_point_y = curr_line_point_y;
        }
        if change_x_first {
            result.push(IntPoint::new(end_point.x, prev_line_point_y));
        } else {
            result.push(IntPoint::new(prev_line_point_x, end_point.y));
        }
        result.push(end_point);
        result
    }

    /// An approximation of this segment by 45-degree stairs with integer
    /// coordinates and stair length at most `width`.
    pub fn stair_approximation_45(&self, width: f64, to_the_right: bool) -> Vec<IntPoint> {
        let start_point = self.start_point().to_float().round();
        let end_point = self.end_point().to_float().round();
        if start_point == end_point {
            return Vec::new();
        }
        let delta = end_point.difference_by(start_point);
        if delta.is_diagonal() || delta.is_orthogonal() {
            return vec![start_point, end_point];
        }
        let abs_delta = IntVector::new(delta.x.abs(), delta.y.abs());
        // use otherwise a function of y for better numerical stability
        let function_of_x = abs_delta.x >= abs_delta.y;
        let det = delta.x as f64 * delta.y as f64;
        let (mut stair_width, stair_count);
        if function_of_x {
            stair_width = ((width * abs_delta.x as f64) / abs_delta.y as f64).round() as i32;
            stair_count = (abs_delta.x - 1) / stair_width + 1;
            if end_point.x < start_point.x {
                stair_width = -stair_width;
            }
        } else {
            stair_width = ((width * abs_delta.y as f64) / abs_delta.x as f64).round() as i32;
            stair_count = (abs_delta.y - 1) / stair_width + 1;
            if end_point.y < start_point.y {
                stair_width = -stair_width;
            }
        }
        let mut result = Vec::with_capacity(2 * stair_count as usize + 1);
        result.push(start_point);
        let mut prev_line_point = start_point;
        for i in 1..=stair_count {
            let curr_line_point = if i == stair_count {
                end_point
            } else if function_of_x {
                let curr_x = start_point.x + i * stair_width;
                let curr_y = self
                    .get_line()
                    .function_value_approx(curr_x as f64)
                    .round() as i32;
                IntPoint::new(curr_x, curr_y)
            } else {
                let curr_y = start_point.y + i * stair_width;
                // Note: the Java original calls function_value_approx here
                // (not function_in_y_value_approx), which looks like a bug;
                // ported faithfully.
                let curr_x = self
                    .get_line()
                    .function_value_approx(curr_y as f64)
                    .round() as i32;
                IntPoint::new(curr_x, curr_y)
            };
            let (curr_x, curr_y);
            if function_of_x {
                let diagonal_first =
                    to_the_right && det < 0.0 || !to_the_right && det > 0.0;
                if diagonal_first {
                    curr_x = prev_line_point.x
                        + Signum::as_int(stair_width as f64)
                            * (curr_line_point.y - prev_line_point.y).abs();
                    curr_y = curr_line_point.y;
                } else {
                    // horizontal first
                    curr_x = curr_line_point.x
                        - Signum::as_int(stair_width as f64)
                            * (curr_line_point.y - prev_line_point.y).abs();
                    curr_y = prev_line_point.y;
                }
            } else {
                // function of y
                let diagonal_first =
                    to_the_right && det > 0.0 || !to_the_right && det < 0.0;
                if diagonal_first {
                    curr_x = curr_line_point.x;
                    curr_y = prev_line_point.y
                        + Signum::as_int(stair_width as f64)
                            * (curr_line_point.x - prev_line_point.x).abs();
                } else {
                    curr_x = prev_line_point.x;
                    curr_y = curr_line_point.y
                        - Signum::as_int(stair_width as f64)
                            * (curr_line_point.x - prev_line_point.x).abs();
                }
            }
            result.push(IntPoint::new(curr_x, curr_y));
            result.push(curr_line_point);
            prev_line_point = curr_line_point;
        }
        result
    }

    /// The border line numbers of `shape` intersected by this line segment
    /// (0, 1 or 2 entries; with 2, the one nearest to the start point comes
    /// first). Intersections at an endpoint count only if the segment
    /// enters the interior of the shape.
    pub fn border_intersections(&self, shape: &TileShape) -> Vec<usize> {
        if !self.bounding_box().intersects(shape.bounding_box()) {
            return Vec::new();
        }
        let edge_count = shape.border_line_count();
        let mut prev_line = shape.border_line(edge_count - 1);
        let mut curr_line = shape.border_line(0);
        let mut result: Vec<usize> = Vec::with_capacity(2);
        let mut intersections: Vec<Point> = Vec::with_capacity(2);
        let line_start = self.start_point();
        let line_end = self.end_point();

        for edge_line_no in 0..edge_count {
            let next_line = shape.border_line((edge_line_no + 1) % edge_count);

            let start_point_side = curr_line.side_of(&line_start);
            let end_point_side = curr_line.side_of(&line_end);
            if start_point_side == Side::OnTheLeft && end_point_side == Side::OnTheLeft {
                // both endpoints outside this border line: no intersection
                return Vec::new();
            }
            if start_point_side == Side::Collinear && end_point_side != Side::OnTheRight {
                // touches only, the interior is not entered
                return Vec::new();
            }
            if end_point_side == Side::Collinear && start_point_side != Side::OnTheRight {
                return Vec::new();
            }
            if start_point_side != Side::OnTheRight || end_point_side != Side::OnTheRight {
                // not both points inside the half plane of curr_line
                let is = self.middle.intersection(&curr_line);
                let prev_line_side_of_is = prev_line.side_of(&is);
                let next_line_side_of_is = next_line.side_of(&is);
                if prev_line_side_of_is != Side::OnTheLeft
                    && next_line_side_of_is != Side::OnTheLeft
                {
                    // intersects curr_line between the previous and next
                    // corner of the shape
                    if prev_line_side_of_is == Side::Collinear {
                        // goes through the previous corner: check the
                        // intersection isn't merely a touch
                        let prev_prev_corner =
                            shape.corner((edge_line_no + edge_count - 1) % edge_count);
                        let next_corner = shape.corner((edge_line_no + 1) % edge_count);
                        let prev_prev_corner_side = self.middle.side_of(&prev_prev_corner);
                        let next_corner_side = self.middle.side_of(&next_corner);
                        if prev_prev_corner_side == Side::Collinear
                            || next_corner_side == Side::Collinear
                            || prev_prev_corner_side == next_corner_side
                        {
                            return Vec::new();
                        }
                    }
                    if next_line_side_of_is == Side::Collinear {
                        // goes through the next corner: check the
                        // intersection isn't merely a touch
                        let prev_corner = shape.corner(edge_line_no);
                        let next_next_corner = shape.corner((edge_line_no + 2) % edge_count);
                        let prev_corner_side = self.middle.side_of(&prev_corner);
                        let next_next_corner_side = self.middle.side_of(&next_next_corner);
                        if prev_corner_side == Side::Collinear
                            || next_next_corner_side == Side::Collinear
                            || prev_corner_side == next_next_corner_side
                        {
                            return Vec::new();
                        }
                    }
                    let already_handled = intersections.contains(&is);
                    if !already_handled && result.len() < 2 {
                        result.push(edge_line_no);
                        intersections.push(is);
                    }
                }
            }
            prev_line = curr_line;
            curr_line = next_line;
        }

        if result.len() == 2 {
            // assure the correct order
            let is0 = intersections[0].to_float();
            let is1 = intersections[1].to_float();
            let curr_start = line_start.to_float();
            if curr_start.distance_square(is1) < curr_start.distance_square(is0) {
                result.swap(0, 1);
            }
        }
        result
    }

    /// Inverts the direction of the segment if the start point is bigger
    /// than the end point in (x, y) lexicographic order.
    pub fn sort_endpoints_in_x_y(&self) -> Self {
        if self.start_point().compare_x_y(&self.end_point()).is_gt() {
            LineSegment::new(self.end, self.middle, self.start)
        } else {
            *self
        }
    }
}

impl Polyline {
    /// The box shape around the `no`-th line segment with border distance
    /// `half_width` from the center line.
    pub fn offset_box(&self, half_width: i32, no: usize) -> IntBox {
        LineSegment::from_polyline(self, no + 1)
            .bounding_box()
            .offset(half_width as f64)
    }

    /// True if `point` lies on this polyline.
    pub fn contains(&self, point: IntPoint) -> bool {
        (1..self.arr.len().saturating_sub(1))
            .any(|i| LineSegment::from_polyline(self, i).contains(point))
    }

    /// A perpendicular line segment from `from_point` onto the nearest line
    /// segment of this polyline; `None` if the perpendicular line does not
    /// intersect the nearest segment inside its bounds or `from_point` is
    /// contained in this polyline.
    pub fn projection_line(&self, from_point: IntPoint) -> Option<LineSegment> {
        let from_point_f = from_point.to_float();
        let mut min_distance = f64::MAX;
        let mut result_line: Option<Line> = None;
        let mut nearest_line: Option<Line> = None;
        for i in 1..self.arr.len() - 1 {
            let projection = self.arr[i].projection_approx(from_point_f);
            let curr_distance = projection.distance(from_point_f);
            if curr_distance < min_distance {
                let Some(direction_towards_line) =
                    self.arr[i].perpendicular_direction(from_point)
                else {
                    continue;
                };
                let curr_result_line = Line::from_direction(from_point, direction_towards_line);
                let prev_corner = self.corner(i - 1);
                let next_corner = self.corner(i);
                let prev_corner_side = curr_result_line.side_of(&prev_corner);
                let next_corner_side = curr_result_line.side_of(&next_corner);
                if prev_corner_side == next_corner_side && prev_corner_side != Side::Collinear
                {
                    // the projection point is outside the line segment
                    continue;
                }
                nearest_line = Some(self.arr[i]);
                min_distance = curr_distance;
                result_line = Some(curr_result_line);
            }
        }
        let nearest_line = nearest_line?;
        let start_line = Line::from_direction(from_point, nearest_line.direction());
        Some(LineSegment::new(start_line, result_line?, nearest_line))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn segment(ax: i32, ay: i32, bx: i32, by: i32) -> LineSegment {
        let polyline = Polyline::from_two_points(IntPoint::new(ax, ay), IntPoint::new(bx, by));
        LineSegment::from_polyline(&polyline, 1)
    }

    #[test]
    fn endpoints_and_opposite() {
        let s = segment(0, 0, 10, 5);
        assert_eq!(s.start_point(), Point::Int(IntPoint::new(0, 0)));
        assert_eq!(s.end_point(), Point::Int(IntPoint::new(10, 5)));
        let o = s.opposite();
        assert_eq!(o.start_point(), s.end_point());
        assert_eq!(o.end_point(), s.start_point());
        assert_eq!(s.bounding_box(), IntBox::from_coords(0, 0, 10, 5));
        let p = s.to_polyline();
        assert_eq!(p.corner_count(), 2);
    }

    #[test]
    fn contains_points_on_segment() {
        let s = segment(0, 0, 10, 10);
        assert!(s.contains(IntPoint::new(5, 5)));
        assert!(s.contains(IntPoint::new(0, 0)));
        assert!(s.contains(IntPoint::new(10, 10)));
        assert!(!s.contains(IntPoint::new(11, 11))); // collinear but beyond
        assert!(!s.contains(IntPoint::new(5, 6))); // off the line
    }

    #[test]
    fn segment_intersection_cases() {
        let horizontal = segment(0, 0, 10, 0);
        let crossing = segment(5, -5, 5, 5);
        let is = horizontal.intersection(&crossing);
        assert_eq!(is.len(), 1);
        assert_eq!(
            horizontal.get_line().intersection(&is[0]),
            Point::Int(IntPoint::new(5, 0))
        );
        assert!(horizontal.intersects(&crossing));
        assert!(!horizontal.overlaps(&crossing));

        // Disjoint segments.
        let far = segment(20, 1, 30, 1);
        assert!(!horizontal.intersects(&far));

        // Overlap on the same line.
        let overlapping = segment(5, 0, 15, 0);
        let is = horizontal.intersection(&overlapping);
        assert_eq!(is.len(), 2);
        assert!(horizontal.overlaps(&overlapping));
        let touching = segment(10, 0, 20, 0);
        let is = horizontal.intersection(&touching);
        assert_eq!(is.len(), 1);
        assert!(!horizontal.overlaps(&touching));
    }

    #[test]
    fn to_simplex_shape() {
        let s = segment(0, 0, 10, 0);
        let simplex = s.to_simplex();
        assert_eq!(simplex.dimension(), 1);
        assert!(simplex.contains(&Point::Int(IntPoint::new(5, 0))));
        assert!(simplex.is_outside(&Point::Int(IntPoint::new(5, 1))));
        assert!(simplex.is_outside(&Point::Int(IntPoint::new(11, 0))));
    }

    #[test]
    fn change_length_and_sort() {
        let s = segment(0, 0, 10, 0);
        let shortened = s.change_length_approx(6.0);
        assert_eq!(shortened.end_point(), Point::Int(IntPoint::new(6, 0)));
        let backwards = segment(10, 0, 0, 0);
        let sorted = backwards.sort_endpoints_in_x_y();
        assert_eq!(sorted.start_point(), Point::Int(IntPoint::new(0, 0)));
    }

    #[test]
    fn stair_approximations() {
        let s = segment(0, 0, 10, 4);
        let stairs = s.stair_approximation(2.0, true);
        assert_eq!(*stairs.first().unwrap(), IntPoint::new(0, 0));
        assert_eq!(*stairs.last().unwrap(), IntPoint::new(10, 4));
        // stairs must be orthogonal
        for w in stairs.windows(2) {
            assert!(
                w[0].x == w[1].x || w[0].y == w[1].y,
                "stair step {w:?} not orthogonal"
            );
        }
        // 45-degree stairs: every step orthogonal or diagonal
        let stairs45 = s.stair_approximation_45(2.0, true);
        assert_eq!(*stairs45.first().unwrap(), IntPoint::new(0, 0));
        assert_eq!(*stairs45.last().unwrap(), IntPoint::new(10, 4));
        for w in stairs45.windows(2) {
            let d = w[1].difference_by(w[0]);
            assert!(
                d.is_orthogonal() || d.is_diagonal(),
                "stair step {w:?} not 45-degree"
            );
        }
    }

    #[test]
    fn border_intersections_with_box() {
        let shape = TileShape::Box(IntBox::from_coords(0, 0, 10, 10));
        // Crossing through the whole box: 2 intersections, nearest first.
        let crossing = segment(-5, 5, 15, 5);
        let borders = crossing.border_intersections(&shape);
        assert_eq!(borders.len(), 2);
        assert_eq!(borders, vec![3, 1]); // left border first (nearest to start)
        // Segment ending inside: 1 intersection.
        let entering = segment(-5, 5, 5, 5);
        assert_eq!(entering.border_intersections(&shape).len(), 1);
        // Segment fully inside: no border intersection.
        let inside = segment(2, 2, 8, 8);
        assert!(inside.border_intersections(&shape).is_empty());
        // Segment far away.
        let outside = segment(20, 20, 30, 30);
        assert!(outside.border_intersections(&shape).is_empty());
    }

    #[test]
    fn polyline_closures() {
        let p = Polyline::from_int_points(&[
            IntPoint::new(0, 0),
            IntPoint::new(10, 0),
            IntPoint::new(10, 10),
        ]);
        assert!(p.contains(IntPoint::new(5, 0)));
        assert!(p.contains(IntPoint::new(10, 5)));
        assert!(!p.contains(IntPoint::new(5, 5)));
        assert_eq!(
            p.offset_box(2, 0),
            IntBox::from_coords(-2, -2, 12, 2)
        );
        // From (4, 7) the vertical segment (distance 6) is nearer than the
        // horizontal one (distance 7).
        let proj = p.projection_line(IntPoint::new(4, 7)).unwrap();
        assert_eq!(proj.start_point(), Point::Int(IntPoint::new(4, 7)));
        assert_eq!(proj.end_point(), Point::Int(IntPoint::new(10, 7)));
        // From below the first segment, the projection lands on it.
        let proj = p.projection_line(IntPoint::new(4, -5)).unwrap();
        assert_eq!(proj.end_point(), Point::Int(IntPoint::new(4, 0)));
        // A point on a segment skips that (collinear) segment and projects
        // onto the next nearest one — matching the actual Java behavior
        // (its javadoc claims null, but the code only skips the collinear
        // line).
        let proj = p.projection_line(IntPoint::new(5, 0)).unwrap();
        assert_eq!(proj.end_point(), Point::Int(IntPoint::new(10, 0)));
    }
}
