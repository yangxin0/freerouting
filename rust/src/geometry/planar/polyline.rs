//! Port of `geometry/planar/Polyline.java`.
//!
//! A polyline is a sequence of lines where no 2 consecutive lines are
//! parallel. A polyline of n lines defines a polygon of n-1 intersection
//! points of consecutive lines. Polylines with integer line coordinates are
//! used instead of polygons with rational corners for performance.
//!
//! Deferred until `LineSegment` is ported: `offset_box`, `contains(Point)`,
//! `projection_line`.

use crate::geometry::planar::{
    FloatPoint, IntBox, IntOctagon, IntPoint, IntVector, Line, Point, Polygon, Side, TileShape,
};

#[derive(Debug, Clone, PartialEq)]
pub struct Polyline {
    /// The lines of this polyline (public like the Java field `arr`).
    pub arr: Vec<Line>,
}

impl Polyline {
    /// Creates a polyline from a polygon of at least 2 corners: corner `i`
    /// becomes the intersection of lines `i` and `i+1`, with perpendicular
    /// end lines representing the first and last corner.
    ///
    /// Like the Java original, only integer corners are supported.
    pub fn from_polygon(polygon: &Polygon) -> Self {
        let corners: Vec<IntPoint> = polygon
            .corner_array()
            .iter()
            .map(|p| match p {
                Point::Int(ip) => *ip,
                Point::Rational(_) => {
                    // Java warns "only implemented for IntPoints till now"
                    p.to_float().round()
                }
            })
            .collect();
        Self::from_int_points(&corners)
    }

    /// Creates a polyline through the given corner points.
    pub fn from_int_points(point_arr: &[IntPoint]) -> Self {
        if point_arr.len() < 2 {
            return Polyline { arr: Vec::new() };
        }
        let n = point_arr.len();
        let mut arr = Vec::with_capacity(n + 1);
        // perpendicular line at the start
        let start_dir = direction_between(point_arr[0], point_arr[1]).turn_45_degree(2);
        arr.push(Line::from_direction(point_arr[0], start_dir));
        for i in 1..n {
            arr.push(Line::new(point_arr[i - 1], point_arr[i]));
        }
        // perpendicular line at the end
        let end_dir = direction_between(point_arr[n - 1], point_arr[n - 2]).turn_45_degree(2);
        arr.push(Line::from_direction(point_arr[n - 1], end_dir));
        Polyline { arr }
    }

    /// Creates a polyline of 3 lines between two corners.
    pub fn from_two_points(from_corner: IntPoint, to_corner: IntPoint) -> Self {
        if from_corner == to_corner {
            return Polyline { arr: Vec::new() };
        }
        Self::from_int_points(&[from_corner, to_corner])
    }

    /// Creates a polyline from an array of lines. Lines parallel to the
    /// previous line are skipped, overlaps removed, and the directed lines
    /// normalized so that each points from the previous corner to the next.
    pub fn from_lines(line_arr: Vec<Line>) -> Self {
        let lines = remove_consecutive_parallel_lines(line_arr);
        let mut lines = remove_overlaps(lines);
        if lines.len() < 3 {
            return Polyline { arr: Vec::new() };
        }
        // Turn the direction of the lines so they always point from the
        // previous corner to the next corner.
        for i in 1..lines.len() - 1 {
            let corner = lines[i].intersection_approx(&lines[i + 1]);
            let side_of_line = lines[i - 1].side_of_float(corner, 0.0);
            if side_of_line != Side::Collinear {
                let d0 = lines[i - 1].direction();
                let d1 = lines[i].direction();
                if d0.side_of(d1) != side_of_line {
                    lines[i] = lines[i].opposite();
                }
            }
        }
        Polyline { arr: lines }
    }

    /// The number of corners: the number of lines minus 1.
    pub fn corner_count(&self) -> usize {
        self.arr.len().saturating_sub(1)
    }

    pub fn is_empty(&self) -> bool {
        self.arr.len() < 3
    }

    /// True if this polyline is empty or all corner points are equal.
    pub fn is_point(&self) -> bool {
        if self.arr.len() < 3 {
            return true;
        }
        let first_corner = self.corner(0);
        (1..self.arr.len() - 1).all(|i| self.corner(i) == first_corner)
    }

    pub fn is_orthogonal(&self) -> bool {
        self.arr.iter().all(Line::is_orthogonal)
    }

    pub fn is_multiple_of_45_degree(&self) -> bool {
        self.arr.iter().all(Line::is_multiple_of_45_degree)
    }

    /// The intersection of lines `no` and `no + 1` (clamped like Java).
    pub fn corner(&self, no: usize) -> Point {
        let no = no.min(self.arr.len() - 2);
        self.arr[no].intersection(&self.arr[no + 1])
    }

    pub fn corner_approx(&self, no: usize) -> FloatPoint {
        let no = no.min(self.arr.len() - 2);
        self.arr[no].intersection_approx(&self.arr[no + 1])
    }

    pub fn first_corner(&self) -> Point {
        self.corner(0)
    }

    pub fn last_corner(&self) -> Point {
        self.corner(self.arr.len() - 2)
    }

    pub fn corner_arr(&self) -> Vec<Point> {
        if self.arr.len() < 2 {
            return Vec::new();
        }
        (0..self.arr.len() - 1).map(|i| self.corner(i)).collect()
    }

    pub fn corner_approx_arr(&self) -> Vec<FloatPoint> {
        if self.arr.len() < 2 {
            return Vec::new();
        }
        (0..self.arr.len() - 1)
            .map(|i| self.corner_approx(i))
            .collect()
    }

    /// The polyline with reversed order of lines.
    pub fn reverse(&self) -> Self {
        let reversed = self
            .arr
            .iter()
            .rev()
            .map(Line::opposite)
            .collect();
        Polyline { arr: reversed }
    }

    /// The length of this polyline from corner `from_corner` to `to_corner`.
    pub fn length_approx_between(&self, from_corner: usize, to_corner: usize) -> f64 {
        let to_corner = to_corner.min(self.arr.len() - 2);
        (from_corner..to_corner)
            .map(|i| self.corner_approx(i + 1).distance(self.corner_approx(i)))
            .sum()
    }

    /// The cumulative distance between consecutive corners.
    pub fn length_approx(&self) -> f64 {
        self.length_approx_between(0, self.arr.len() - 2)
    }

    /// For each line in `from_no..to_no` a convex shape around the line
    /// where left and right border have distance `half_width` from the
    /// center line, with dog-ear corners cut off against neighboring
    /// segments.
    pub fn offset_shapes_between(
        &self,
        half_width: i32,
        from_no: usize,
        to_no: usize,
    ) -> Vec<TileShape> {
        let to_no = to_no.min(self.arr.len() - 1);
        let shape_count = (to_no.saturating_sub(from_no)).saturating_sub(1);
        let mut shape_arr = Vec::with_capacity(shape_count);
        if shape_count == 0 {
            return shape_arr;
        }
        let hw = half_width as f64;
        let mut prev_dir = self.arr[from_no].direction();
        let mut curr_dir = self.arr[from_no + 1].direction();
        for i in from_no + 1..to_no {
            let next_dir = self.arr[i + 1].direction();

            let mut lines = Vec::with_capacity(4);
            // current center line translated to the right
            lines.push(self.arr[i].translate(-hw));

            // the front line of the offset shape
            let next_dir_from_curr_dir =
                next_dir.get_vector().side_of(curr_dir.get_vector());
            if next_dir_from_curr_dir == Side::OnTheLeft {
                // next right line
                lines.push(self.arr[i + 1].translate(-hw));
            } else {
                // next left line in opposite direction
                lines.push(self.arr[i + 1].opposite().translate(-hw));
            }

            // current left line in opposite direction
            lines.push(self.arr[i].opposite().translate(-hw));

            // the back line of the offset shape
            let curr_dir_from_prev_dir =
                curr_dir.get_vector().side_of(prev_dir.get_vector());
            if curr_dir_from_prev_dir == Side::OnTheLeft {
                // previous line translated to the right
                lines.push(self.arr[i - 1].translate(-hw));
            } else {
                // previous left line in opposite direction
                lines.push(self.arr[i - 1].opposite().translate(-hw));
            }

            // cut off outstanding corners with following shapes
            let check_dist_square = 2.0 * hw * hw;
            let mut cut_dog_ear_lines: Vec<Line> = Vec::new();
            {
                let mut corner_to_check = FloatPoint::ZERO;
                let mut curr_line = lines[1];
                let check_line = if next_dir_from_curr_dir == Side::OnTheLeft {
                    lines[2]
                } else {
                    lines[0]
                };
                let check_distance_corner = self.corner_approx(i);
                let mut tmp_curr_dir = next_dir;
                let mut direction_changed = false;
                for j in i + 2..self.arr.len().saturating_sub(1) {
                    if self
                        .corner_approx(j - 1)
                        .distance_square(check_distance_corner)
                        > check_dist_square
                    {
                        break;
                    }
                    if !direction_changed {
                        corner_to_check = curr_line.intersection_approx(&check_line);
                    }
                    let tmp_next_dir = self.arr[j].direction();
                    let tmp_next_dir_from_tmp_curr_dir = tmp_next_dir
                        .get_vector()
                        .side_of(tmp_curr_dir.get_vector());
                    direction_changed =
                        tmp_next_dir_from_tmp_curr_dir != next_dir_from_curr_dir;
                    if !direction_changed {
                        let next_border_line =
                            if tmp_next_dir_from_tmp_curr_dir == Side::OnTheLeft {
                                self.arr[j].translate(-hw)
                            } else {
                                self.arr[j].opposite().translate(-hw)
                            };
                        if next_border_line.side_of_float(corner_to_check, 0.0)
                            == Side::OnTheLeft
                            && next_border_line.side_of(&self.corner(i)) == Side::OnTheRight
                            && next_border_line.side_of(&self.corner(i - 1))
                                == Side::OnTheRight
                        {
                            // an outstanding corner
                            cut_dog_ear_lines.push(next_border_line);
                        }
                        tmp_curr_dir = tmp_next_dir;
                        curr_line = next_border_line;
                    }
                }
            }
            // cut off outstanding corners with previous shapes
            {
                let mut corner_to_check = FloatPoint::ZERO;
                let check_distance_corner = self.corner_approx(i - 1);
                let check_line = if curr_dir_from_prev_dir == Side::OnTheLeft {
                    lines[2]
                } else {
                    lines[0]
                };
                let mut curr_line = lines[3];
                let mut tmp_curr_dir = prev_dir;
                let mut direction_changed = false;
                for j in (1..=i.saturating_sub(2)).rev() {
                    if self.corner_approx(j).distance_square(check_distance_corner)
                        > check_dist_square
                    {
                        break;
                    }
                    if !direction_changed {
                        corner_to_check = curr_line.intersection_approx(&check_line);
                    }
                    let tmp_prev_dir = self.arr[j].direction();
                    let tmp_curr_dir_from_tmp_prev_dir = tmp_curr_dir
                        .get_vector()
                        .side_of(tmp_prev_dir.get_vector());
                    direction_changed =
                        tmp_curr_dir_from_tmp_prev_dir != curr_dir_from_prev_dir;
                    if !direction_changed {
                        let prev_border_line =
                            if tmp_curr_dir_from_tmp_prev_dir == Side::OnTheLeft {
                                self.arr[j].translate(-hw)
                            } else {
                                self.arr[j].opposite().translate(-hw)
                            };
                        if prev_border_line.side_of_float(corner_to_check, 0.0)
                            == Side::OnTheLeft
                            && prev_border_line.side_of(&self.corner(i)) == Side::OnTheRight
                            && prev_border_line.side_of(&self.corner(i - 1))
                                == Side::OnTheRight
                        {
                            // an outstanding corner
                            cut_dog_ear_lines.push(prev_border_line);
                        }
                        tmp_curr_dir = tmp_prev_dir;
                        curr_line = prev_border_line;
                    }
                }
            }
            let mut s1 = TileShape::get_instance(lines);
            if !cut_dog_ear_lines.is_empty() {
                s1 = s1.intersection(&TileShape::get_instance(cut_dog_ear_lines));
            }
            // intersect with the offset bounding octagon to keep the shape
            // bounded
            let surr_oct = self.bounding_octagon_between(i - 1, i);
            let bounding_shape = TileShape::Octagon(surr_oct.offset(hw));
            shape_arr.push(bounding_shape.intersection_with_simplify(&s1));

            prev_dir = curr_dir;
            curr_dir = next_dir;
        }
        shape_arr
    }

    /// Offset shapes for all lines of this polyline.
    pub fn offset_shapes(&self, half_width: i32) -> Vec<TileShape> {
        self.offset_shapes_between(half_width, 0, self.arr.len() - 1)
    }

    /// The offset shape around the `no`-th line segment.
    pub fn offset_shape(&self, half_width: i32, no: usize) -> Option<TileShape> {
        if no + 3 > self.arr.len() {
            return None;
        }
        self.offset_shapes_between(half_width, no, no + 2)
            .into_iter()
            .next()
    }

    pub fn translate_by(&self, vector: IntVector) -> Self {
        if vector.is_zero() {
            return self.clone();
        }
        Polyline {
            arr: self.arr.iter().map(|l| l.translate_by(vector)).collect(),
        }
    }

    pub fn turn_90_degree(&self, factor: i32, pole: IntPoint) -> Self {
        Polyline {
            arr: self
                .arr
                .iter()
                .map(|l| l.turn_90_degree(factor, pole))
                .collect(),
        }
    }

    pub fn rotate_approx(&self, angle: f64, pole: FloatPoint) -> Self {
        if angle == 0.0 {
            return self.clone();
        }
        let new_corners: Vec<IntPoint> = (0..self.corner_count())
            .map(|i| self.corner_approx(i).rotate(angle, pole).round())
            .collect();
        Self::from_int_points(&new_corners)
    }

    pub fn mirror_vertical(&self, pole: IntPoint) -> Self {
        Polyline {
            arr: self.arr.iter().map(|l| l.mirror_vertical(pole)).collect(),
        }
    }

    pub fn mirror_horizontal(&self, pole: IntPoint) -> Self {
        Polyline {
            arr: self.arr.iter().map(|l| l.mirror_horizontal(pole)).collect(),
        }
    }

    /// The smallest box containing the corners from `from_corner_no` to
    /// `to_corner_no`.
    pub fn bounding_box_between(&self, from_corner_no: usize, to_corner_no: usize) -> IntBox {
        let to_corner_no = to_corner_no.min(self.arr.len() - 2);
        let (mut llx, mut lly) = (f64::MAX, f64::MAX);
        let (mut urx, mut ury) = (f64::MIN, f64::MIN);
        for i in from_corner_no..=to_corner_no {
            let c = clamp_corner(self.corner_approx(i));
            llx = llx.min(c.x);
            lly = lly.min(c.y);
            urx = urx.max(c.x);
            ury = ury.max(c.y);
        }
        IntBox::from_coords(
            llx.floor() as i32,
            lly.floor() as i32,
            urx.ceil() as i32,
            ury.ceil() as i32,
        )
    }

    pub fn bounding_box(&self) -> IntBox {
        self.bounding_box_between(0, self.corner_count() - 1)
    }

    /// The smallest octagon containing the corners from `from_corner_no` to
    /// `to_corner_no`.
    pub fn bounding_octagon_between(
        &self,
        from_corner_no: usize,
        to_corner_no: usize,
    ) -> IntOctagon {
        let to_corner_no = to_corner_no.min(self.arr.len() - 2);
        let (mut lx, mut ly) = (f64::MAX, f64::MAX);
        let (mut rx, mut uy) = (f64::MIN, f64::MIN);
        let (mut ulx, mut llx) = (f64::MAX, f64::MAX);
        let (mut lrx, mut urx) = (f64::MIN, f64::MIN);
        for i in from_corner_no..=to_corner_no {
            let c = clamp_corner(self.corner_approx(i));
            lx = lx.min(c.x);
            ly = ly.min(c.y);
            rx = rx.max(c.x);
            uy = uy.max(c.y);
            let tmp = c.x - c.y;
            ulx = ulx.min(tmp);
            lrx = lrx.max(tmp);
            let tmp = c.x + c.y;
            llx = llx.min(tmp);
            urx = urx.max(tmp);
        }
        IntOctagon::new(
            lx.floor() as i32,
            ly.floor() as i32,
            rx.ceil() as i32,
            uy.ceil() as i32,
            ulx.floor() as i32,
            lrx.ceil() as i32,
            llx.floor() as i32,
            urx.ceil() as i32,
        )
    }

    /// An approximation of the nearest point on this polyline to
    /// `from_point`.
    pub fn nearest_point_approx(&self, from_point: FloatPoint) -> FloatPoint {
        let corners = self.corner_approx_arr();
        let mut min_distance = f64::MAX;
        let mut nearest_point = corners[0];
        for c in &corners {
            let curr_distance = c.distance(from_point);
            if curr_distance < min_distance {
                min_distance = curr_distance;
                nearest_point = *c;
            }
        }
        let c_tolerance = 1.0;
        for i in 1..self.arr.len() - 1 {
            let projection = self.arr[i].projection_approx(from_point);
            let curr_distance = projection.distance(from_point);
            if curr_distance < min_distance {
                // look if the projection is inside the segment
                let segment_length = corners[i].distance(corners[i - 1]);
                if projection.distance(corners[i]) + projection.distance(corners[i - 1])
                    < segment_length + c_tolerance
                {
                    min_distance = curr_distance;
                    nearest_point = projection;
                }
            }
        }
        nearest_point
    }

    /// The distance of `from_point` to the nearest point on this polyline.
    pub fn distance(&self, from_point: FloatPoint) -> f64 {
        from_point.distance(self.nearest_point_approx(from_point))
    }

    /// Combines the two polylines if they have a common end corner,
    /// preserving the order of lines of this polyline; returns `self`
    /// unchanged if there is no common end corner.
    pub fn combine(&self, other: &Polyline) -> Self {
        if self.arr.len() < 3 || other.arr.len() < 3 {
            return self.clone();
        }
        let (combine_at_start, combine_other_at_start) =
            if self.first_corner() == other.first_corner() {
                (true, true)
            } else if self.first_corner() == other.last_corner() {
                (true, false)
            } else if self.last_corner() == other.first_corner() {
                (false, true)
            } else if self.last_corner() == other.last_corner() {
                (false, false)
            } else {
                return self.clone(); // no common endpoint
            };
        let mut line_arr: Vec<Line> = Vec::with_capacity(self.arr.len() + other.arr.len() - 2);
        if combine_at_start {
            // insert the lines of other in front
            if combine_other_at_start {
                // in reverse order, skipping the first line of other
                line_arr.extend(other.arr.iter().skip(1).rev().map(Line::opposite));
            } else {
                // skipping the last line of other
                line_arr.extend_from_slice(&other.arr[..other.arr.len() - 1]);
            }
            // append this polyline, skipping its first line
            line_arr.extend_from_slice(&self.arr[1..]);
        } else {
            // insert this polyline in front, skipping its last line
            line_arr.extend_from_slice(&self.arr[..self.arr.len() - 1]);
            if combine_other_at_start {
                // skipping the first line of other
                line_arr.extend_from_slice(&other.arr[1..]);
            } else {
                // in reverse order, skipping the last line of other
                line_arr.extend(
                    other.arr[..other.arr.len() - 1]
                        .iter()
                        .rev()
                        .map(Line::opposite),
                );
            }
        }
        Polyline::from_lines(line_arr)
    }

    /// Splits this polyline at the line `line_no` by inserting `end_line`
    /// as concluding line of the first piece and start line of the second.
    /// Returns `None` if nothing was split.
    pub fn split(&self, line_no: usize, end_line: Line) -> Option<[Polyline; 2]> {
        if line_no < 1 || line_no > self.arr.len() - 2 {
            return None;
        }
        if self.arr[line_no].is_parallel(&end_line) {
            return None;
        }
        let new_end_corner = self.arr[line_no].intersection(&end_line);
        if (line_no == 1 && new_end_corner == self.first_corner())
            || (line_no >= self.arr.len() - 2 && new_end_corner == self.last_corner())
        {
            // no split if end_line only touches this polyline at an end
            // point
            return None;
        }
        let first_piece: Vec<Line> = if self.corner(line_no - 1) == new_end_corner {
            // skip line segment of length 0 at the end of the first piece
            self.arr[..=line_no].to_vec()
        } else {
            let mut lines = self.arr[..=line_no].to_vec();
            lines.push(end_line);
            lines
        };
        let second_piece: Vec<Line> = if self.corner(line_no) == new_end_corner {
            // skip line segment of length 0 at the start of the second
            // piece
            self.arr[line_no..].to_vec()
        } else {
            let mut lines = vec![end_line];
            lines.extend_from_slice(&self.arr[line_no..]);
            lines
        };
        let result = [
            Polyline::from_lines(first_piece),
            Polyline::from_lines(second_piece),
        ];
        if result[0].is_point() || result[1].is_point() {
            return None;
        }
        Some(result)
    }

    /// A new polyline skipping the lines from `from_no` to `to_no`
    /// inclusive.
    pub fn skip_lines(&self, from_no: usize, to_no: usize) -> Self {
        if to_no > self.arr.len() - 1 || from_no > to_no {
            return self.clone();
        }
        let mut new_lines = Vec::with_capacity(self.arr.len() - (to_no - from_no + 1));
        new_lines.extend_from_slice(&self.arr[..from_no]);
        new_lines.extend_from_slice(&self.arr[to_no + 1..]);
        Polyline::from_lines(new_lines)
    }

    /// Shortens this polyline to `new_line_count` lines with the last
    /// segment shortened to about `last_segment_length`; the new last
    /// corner is an integer point.
    pub fn shorten(&self, new_line_count: usize, last_segment_length: f64) -> Self {
        let last_corner = self.corner_approx(new_line_count - 2);
        let prev_last_corner = self.corner_approx(new_line_count - 3);
        let new_last_corner = prev_last_corner
            .change_length(last_corner, last_segment_length)
            .round();
        if Point::Int(new_last_corner) == self.corner(self.corner_count() - 2) {
            // skip the last line
            return self.skip_lines(new_line_count - 1, new_line_count - 1);
        }
        let mut new_lines: Vec<Line> = self.arr[..new_line_count - 2].to_vec();
        // create the last 2 lines of the new polyline
        let prev_line = self.arr[new_line_count - 2];
        let first_line_point = if prev_line.a == new_last_corner {
            prev_line.b
        } else {
            prev_line.a
        };
        let new_prev_last_line = Line::new(first_line_point, new_last_corner);
        new_lines.push(new_prev_last_line);
        new_lines.push(Line::from_direction(
            new_last_corner,
            new_prev_last_line.direction().turn_45_degree(6),
        ));
        Polyline { arr: new_lines }
    }
}

/// Clamps a corner approximation into the legal coordinate range:
/// intersections of nearly parallel consecutive lines can be quasi
/// infinite and must not poison bounding shapes (Java wraps silently
/// there).
fn clamp_corner(c: FloatPoint) -> FloatPoint {
    let limit = crate::geometry::planar::limits::CRIT_INT as f64;
    FloatPoint::new(c.x.clamp(-limit, limit), c.y.clamp(-limit, limit))
}

/// The normalized direction from `from` to `to` (must differ).
fn direction_between(from: IntPoint, to: IntPoint) -> crate::geometry::planar::IntDirection {
    crate::geometry::planar::IntDirection::from_points(from, to)
        .expect("Polyline: consecutive corners must differ")
}

fn remove_consecutive_parallel_lines(line_arr: Vec<Line>) -> Vec<Line> {
    if line_arr.len() < 3 {
        return line_arr;
    }
    let mut result: Vec<Line> = Vec::with_capacity(line_arr.len());
    result.push(line_arr[0]);
    for line in &line_arr[1..] {
        if !result.last().unwrap().is_parallel(line) {
            result.push(*line);
        }
    }
    if result.len() < 3 {
        return Vec::new();
    }
    result
}

/// Checks if previous and next line are equal or opposite and removes the
/// resulting overlap.
fn remove_overlaps(line_arr: Vec<Line>) -> Vec<Line> {
    if line_arr.len() < 4 {
        return line_arr;
    }
    let n = line_arr.len();
    let mut tmp_arr: Vec<Line> = Vec::with_capacity(n);
    if !line_arr[0].is_equal_or_opposite(&line_arr[2]) {
        tmp_arr.push(line_arr[0]);
    }
    // else skip the first line
    tmp_arr.push(line_arr[1]);
    for i in 2..n - 2 {
        if tmp_arr
            .last()
            .is_some_and(|last| last.is_equal_or_opposite(&line_arr[i + 1]))
        {
            // skip 2 lines
            tmp_arr.pop();
        } else {
            tmp_arr.push(line_arr[i]);
        }
    }
    tmp_arr.push(line_arr[n - 2]);
    // Guard like the Java original: need at least 2 entries to look back.
    if tmp_arr.len() >= 2
        && !line_arr[n - 1].is_equal_or_opposite(&tmp_arr[tmp_arr.len() - 2])
    {
        tmp_arr.push(line_arr[n - 1]);
    }
    // else skip the last line
    if tmp_arr.len() == n {
        return line_arr; // nothing skipped
    }
    if tmp_arr.len() < 3 {
        return Vec::new();
    }
    tmp_arr
}

#[cfg(test)]
mod tests {
    use super::*;

    fn zigzag() -> Polyline {
        Polyline::from_int_points(&[
            IntPoint::new(0, 0),
            IntPoint::new(10, 0),
            IntPoint::new(10, 10),
            IntPoint::new(20, 10),
        ])
    }

    #[test]
    fn construction_and_corners() {
        let p = zigzag();
        assert_eq!(p.arr.len(), 5);
        assert_eq!(p.corner_count(), 4);
        assert!(!p.is_empty());
        assert!(!p.is_point());
        assert!(p.is_orthogonal());
        assert_eq!(p.first_corner(), Point::Int(IntPoint::new(0, 0)));
        assert_eq!(p.last_corner(), Point::Int(IntPoint::new(20, 10)));
        assert_eq!(p.corner(1), Point::Int(IntPoint::new(10, 0)));
        assert_eq!(p.corner(2), Point::Int(IntPoint::new(10, 10)));
        assert!((p.length_approx() - 30.0).abs() < 1e-9);
        assert_eq!(p.bounding_box(), IntBox::from_coords(0, 0, 20, 10));

        let two = Polyline::from_two_points(IntPoint::new(0, 0), IntPoint::new(3, 4));
        assert_eq!(two.arr.len(), 3);
        assert!((two.length_approx() - 5.0).abs() < 1e-9);
        assert!(Polyline::from_two_points(IntPoint::new(1, 1), IntPoint::new(1, 1)).is_empty());
    }

    #[test]
    fn reverse_roundtrip() {
        let p = zigzag();
        let r = p.reverse();
        assert_eq!(r.first_corner(), p.last_corner());
        assert_eq!(r.last_corner(), p.first_corner());
        assert_eq!(r.corner_count(), p.corner_count());
        assert_eq!(r.reverse().corner_arr(), p.corner_arr());
    }

    #[test]
    fn from_lines_removes_parallel_duplicates() {
        let mut lines = zigzag().arr.clone();
        // duplicate a middle line: parallel consecutive lines are skipped
        lines.insert(2, lines[2]);
        let p = Polyline::from_lines(lines);
        assert_eq!(p.corner_arr(), zigzag().corner_arr());
    }

    #[test]
    fn combine_at_common_corner() {
        let a = Polyline::from_int_points(&[IntPoint::new(0, 0), IntPoint::new(10, 0)]);
        let b = Polyline::from_int_points(&[IntPoint::new(10, 0), IntPoint::new(10, 10)]);
        let combined = a.combine(&b);
        assert_eq!(combined.first_corner(), Point::Int(IntPoint::new(0, 0)));
        assert_eq!(combined.last_corner(), Point::Int(IntPoint::new(10, 10)));
        assert_eq!(combined.corner_count(), 3);
        // no common corner: unchanged
        let c = Polyline::from_int_points(&[IntPoint::new(50, 50), IntPoint::new(60, 50)]);
        assert_eq!(a.combine(&c), a);
    }

    #[test]
    fn split_at_line() {
        let p = zigzag();
        // vertical cut through the middle of the first segment
        let cut = Line::from_coords(5, -10, 5, 10);
        let [first, second] = p.split(1, cut).unwrap();
        assert_eq!(first.first_corner(), Point::Int(IntPoint::new(0, 0)));
        assert_eq!(first.last_corner(), Point::Int(IntPoint::new(5, 0)));
        assert_eq!(second.first_corner(), Point::Int(IntPoint::new(5, 0)));
        assert_eq!(second.last_corner(), Point::Int(IntPoint::new(20, 10)));
        // A cut touching only the start corner does not split.
        let touching = Line::from_coords(0, -10, 0, 10);
        assert!(p.split(1, touching).is_none());
    }

    #[test]
    fn offset_shapes_cover_segments() {
        let p = zigzag();
        let shapes = p.offset_shapes(2);
        assert_eq!(shapes.len(), 3);
        for (i, shape) in shapes.iter().enumerate() {
            assert!(!shape.is_empty(), "shape {i} empty");
            // The shape must contain both corners of its segment.
            assert!(shape.contains(&p.corner(i)), "shape {i} misses corner");
            assert!(shape.contains(&p.corner(i + 1)), "shape {i} misses corner");
            // And a point at distance half_width sideways from the segment
            // middle.
            let mid = p.corner_approx(i).middle_point(p.corner_approx(i + 1));
            assert!(shape.contains_float(mid, 0.01));
            assert!(!shape.contains_float(FloatPoint::new(mid.x + 50.0, mid.y), 0.01));
        }
    }

    #[test]
    fn transforms() {
        let p = zigzag();
        let t = p.translate_by(IntVector::new(5, -5));
        assert_eq!(t.first_corner(), Point::Int(IntPoint::new(5, -5)));
        let turned = p.turn_90_degree(1, IntPoint::new(0, 0));
        assert_eq!(turned.first_corner(), Point::Int(IntPoint::new(0, 0)));
        assert_eq!(turned.last_corner(), Point::Int(IntPoint::new(-10, 20)));
        let mirrored = p.mirror_vertical(IntPoint::new(0, 0));
        assert_eq!(mirrored.last_corner(), Point::Int(IntPoint::new(-20, 10)));
    }

    #[test]
    fn nearest_point_and_distance() {
        let p = zigzag();
        let near = p.nearest_point_approx(FloatPoint::new(5.0, 3.0));
        assert_eq!(near, FloatPoint::new(5.0, 0.0));
        assert!((p.distance(FloatPoint::new(5.0, 3.0)) - 3.0).abs() < 1e-9);
        // beyond the end: nearest is the last corner
        let near_end = p.nearest_point_approx(FloatPoint::new(25.0, 10.0));
        assert_eq!(near_end, FloatPoint::new(20.0, 10.0));
    }

    #[test]
    fn skip_and_shorten() {
        let p = zigzag();
        let shortened = p.shorten(4, 5.0);
        assert_eq!(shortened.corner_count(), 3);
        assert_eq!(
            shortened.last_corner(),
            Point::Int(IntPoint::new(10, 5))
        );
    }
}
