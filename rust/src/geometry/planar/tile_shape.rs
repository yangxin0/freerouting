//! Port of the abstract classes `TileShape.java` / `RegularTileShape.java`
//! (dispatch + generic algorithms) as an enum over the three concrete tile
//! shapes.
//!
//! Deferred: the Polyline/LineSegment-dependent methods (entrance_points,
//! cutout(Polyline), is_intersected_interior_by) follow with those types.

use crate::geometry::planar::{
    FloatPoint, IntBox, IntOctagon, IntPoint, Line, Point, Side, Simplex,
};

/// A convex shape whose border lines are directed so the interior is on
/// their point-left: an axis-parallel box, a 45-degree octagon, or a general
/// simplex.
#[derive(Debug, Clone, PartialEq)]
pub enum TileShape {
    Box(IntBox),
    Octagon(IntOctagon),
    Simplex(Simplex),
}

impl TileShape {
    /// Creates a tile shape as the intersection of the half planes defined
    /// by `lines`, simplified to the most specific kind
    /// (Java: `TileShape.get_instance(Line[])`).
    pub fn get_instance(lines: Vec<Line>) -> Self {
        TileShape::Simplex(Simplex::get_instance(lines)).simplify()
    }

    /// Creates a tile shape from the corners of a convex polygon
    /// (counterclockwise).
    pub fn from_convex_polygon(corners: &[IntPoint]) -> Self {
        let n = corners.len();
        let lines = (0..n)
            .map(|i| Line::new(corners[i], corners[(i + 1) % n]))
            .collect();
        Self::get_instance(lines)
    }

    /// Converts to a simpler physical representation if possible
    /// (Simplex → Box/Octagon, Octagon → Box).
    pub fn simplify(self) -> Self {
        match self {
            TileShape::Simplex(ref s) => {
                if s.is_empty() {
                    TileShape::Simplex(Simplex::EMPTY)
                } else if s.is_int_box() {
                    TileShape::Box(s.bounding_box())
                } else if let Some(oct) = s.to_int_octagon() {
                    TileShape::Octagon(oct)
                } else {
                    self
                }
            }
            TileShape::Octagon(oct) => {
                if oct.is_int_box() {
                    TileShape::Box(oct.bounding_box())
                } else {
                    self
                }
            }
            TileShape::Box(_) => self,
        }
    }

    pub fn is_empty(&self) -> bool {
        match self {
            TileShape::Box(b) => b.is_empty(),
            TileShape::Octagon(o) => o.is_empty(),
            TileShape::Simplex(s) => s.is_empty(),
        }
    }

    pub fn dimension(&self) -> i32 {
        match self {
            TileShape::Box(b) => b.dimension(),
            TileShape::Octagon(o) => o.dimension(),
            TileShape::Simplex(s) => s.dimension(),
        }
    }

    pub fn is_bounded(&self) -> bool {
        match self {
            TileShape::Box(_) | TileShape::Octagon(_) => true,
            TileShape::Simplex(s) => s.is_bounded(),
        }
    }

    pub fn border_line_count(&self) -> usize {
        match self {
            TileShape::Box(_) => 4,
            TileShape::Octagon(_) => 8,
            TileShape::Simplex(s) => s.border_line_count(),
        }
    }

    pub fn border_line(&self, no: usize) -> Line {
        match self {
            TileShape::Box(b) => b.border_line(no),
            TileShape::Octagon(o) => o.border_line(no),
            TileShape::Simplex(s) => s.border_line(no),
        }
    }

    pub fn corner_is_bounded(&self, no: usize) -> bool {
        match self {
            TileShape::Box(_) | TileShape::Octagon(_) => true,
            TileShape::Simplex(s) => s.corner_is_bounded(no),
        }
    }

    pub fn corner(&self, no: usize) -> Point {
        match self {
            TileShape::Box(b) => Point::Int(b.corner(no)),
            TileShape::Octagon(o) => Point::Int(o.corner(no)),
            TileShape::Simplex(s) => s.corner(no),
        }
    }

    pub fn corner_approx(&self, no: usize) -> FloatPoint {
        match self {
            TileShape::Box(b) => b.corner(no).to_float(),
            TileShape::Octagon(o) => o.corner(no).to_float(),
            TileShape::Simplex(s) => s.corner_approx(no),
        }
    }

    pub fn corner_approx_arr(&self) -> Vec<FloatPoint> {
        (0..self.border_line_count())
            .map(|i| self.corner_approx(i))
            .collect()
    }

    /// The area of the shape via the shoelace formula; `f64::MAX` if
    /// unbounded.
    pub fn area(&self) -> f64 {
        match self {
            TileShape::Box(b) => b.area(),
            TileShape::Octagon(o) => o.area(),
            TileShape::Simplex(_) => {
                if !self.is_bounded() {
                    return f64::MAX;
                }
                if self.dimension() < 2 {
                    return 0.0;
                }
                let corner_count = self.border_line_count();
                let mut result = 0.0;
                let mut prev_corner = self.corner_approx(corner_count - 2);
                let mut curr_corner = self.corner_approx(corner_count - 1);
                for i in 0..corner_count {
                    let next_corner = self.corner_approx(i);
                    result += curr_corner.x * (next_corner.y - prev_corner.y);
                    prev_corner = curr_corner;
                    curr_corner = next_corner;
                }
                0.5 * result.abs()
            }
        }
    }

    pub fn centre_of_gravity(&self) -> FloatPoint {
        let corners = self.corner_approx_arr();
        let n = corners.len() as f64;
        let (mut x, mut y) = (0.0, 0.0);
        for c in &corners {
            x += c.x;
            y += c.y;
        }
        FloatPoint::new(x / n, y / n)
    }

    pub fn max_width(&self) -> f64 {
        match self {
            TileShape::Box(b) => b.max_width(),
            TileShape::Octagon(o) => o.max_width(),
            TileShape::Simplex(s) => s.max_width(),
        }
    }

    pub fn min_width(&self) -> f64 {
        match self {
            TileShape::Box(b) => b.min_width(),
            TileShape::Octagon(o) => o.min_width(),
            TileShape::Simplex(s) => s.min_width(),
        }
    }

    pub fn bounding_box(&self) -> IntBox {
        match self {
            TileShape::Box(b) => *b,
            TileShape::Octagon(o) => o.bounding_box(),
            TileShape::Simplex(s) => s.bounding_box(),
        }
    }

    pub fn bounding_octagon(&self) -> Option<IntOctagon> {
        match self {
            TileShape::Box(b) => Some(b.to_int_octagon()),
            TileShape::Octagon(o) => Some(*o),
            TileShape::Simplex(s) => s.bounding_octagon(),
        }
    }

    pub fn to_simplex(&self) -> Simplex {
        match self {
            TileShape::Box(b) => b.to_simplex(),
            TileShape::Octagon(o) => o.to_simplex(),
            TileShape::Simplex(s) => s.clone(),
        }
    }

    pub fn is_int_box(&self) -> bool {
        match self {
            TileShape::Box(_) => true,
            TileShape::Octagon(o) => o.is_int_box(),
            TileShape::Simplex(s) => s.is_int_box(),
        }
    }

    pub fn is_int_octagon(&self) -> bool {
        match self {
            TileShape::Box(_) | TileShape::Octagon(_) => true,
            TileShape::Simplex(s) => s.is_int_octagon(),
        }
    }

    pub fn translate_by(&self, vector: crate::geometry::planar::IntVector) -> Self {
        match self {
            TileShape::Box(b) => TileShape::Box(b.translate_by(vector)),
            TileShape::Octagon(o) => TileShape::Octagon(o.translate_by(vector)),
            TileShape::Simplex(s) => TileShape::Simplex(s.translate_by(vector)),
        }
    }

    pub fn offset(&self, distance: f64) -> Self {
        match self {
            TileShape::Box(b) => TileShape::Box(b.offset(distance)),
            TileShape::Octagon(o) => TileShape::Octagon(o.offset(distance)),
            TileShape::Simplex(s) => TileShape::Simplex(s.offset(distance)),
        }
    }

    /// Enlarges the shape by `offset`; like Java, a box enlarges into an
    /// octagon.
    pub fn enlarge(&self, offset: f64) -> Self {
        match self {
            TileShape::Box(b) => TileShape::Octagon(b.enlarge(offset)),
            TileShape::Octagon(o) => TileShape::Octagon(o.enlarge(offset)),
            TileShape::Simplex(s) => TileShape::Simplex(s.enlarge(offset)),
        }
    }

    // ---- point containment (generic over border lines) ----

    pub fn is_outside(&self, point: &Point) -> bool {
        let line_count = self.border_line_count();
        if line_count == 0 {
            return true;
        }
        (0..line_count).any(|i| self.border_line(i).side_of(point) == Side::OnTheLeft)
    }

    pub fn contains(&self, point: &Point) -> bool {
        !self.is_outside(point)
    }

    pub fn contains_inside(&self, point: &Point) -> bool {
        let line_count = self.border_line_count();
        if line_count == 0 {
            return false;
        }
        (0..line_count).all(|i| self.border_line(i).side_of(point) == Side::OnTheRight)
    }

    /// Containment with tolerance in determinant units (see
    /// `Line::side_of_float`).
    pub fn contains_float(&self, point: FloatPoint, tolerance: f64) -> bool {
        let line_count = self.border_line_count();
        if line_count == 0 {
            return false;
        }
        (0..line_count)
            .all(|i| self.border_line(i).side_of_float(point, tolerance) == Side::OnTheRight)
    }

    /// `Collinear` if `point` is on the border (with tolerance),
    /// `OnTheLeft` if outside, `OnTheRight` if inside.
    pub fn side_of_border(&self, point: FloatPoint, tolerance: f64) -> Side {
        let line_count = self.border_line_count();
        if line_count == 0 {
            return Side::Collinear;
        }
        let mut result = Side::OnTheRight; // point is inside
        for i in 0..line_count {
            match self.border_line(i).side_of_float(point, tolerance) {
                Side::OnTheLeft => return Side::OnTheLeft, // point is outside
                Side::Collinear => result = Side::Collinear,
                Side::OnTheRight => {}
            }
        }
        result
    }

    /// The index of the border line containing `point`, if `point` lies on
    /// the border.
    pub fn contains_on_border_line_no(&self, point: &Point) -> Option<usize> {
        let line_count = self.border_line_count();
        let mut containing_line_no = None;
        for i in 0..line_count {
            match self.border_line(i).side_of(point) {
                Side::OnTheLeft => return None, // outside
                Side::Collinear => containing_line_no = Some(i),
                Side::OnTheRight => {}
            }
        }
        containing_line_no
    }

    pub fn contains_on_border(&self, point: &Point) -> bool {
        self.contains_on_border_line_no(point).is_some()
    }

    /// True if this shape contains `other` completely (exact).
    pub fn contains_tile(&self, other: &TileShape) -> bool {
        (0..other.border_line_count()).all(|i| self.contains(&other.corner(i)))
    }

    /// True if this shape contains `other` completely (approximate).
    pub fn contains_approx(&self, other: &TileShape) -> bool {
        other
            .corner_approx_arr()
            .into_iter()
            .all(|c| self.contains_float(c, 0.0))
    }

    // ---- nearest points and distances ----

    pub fn distance(&self, point: FloatPoint) -> f64 {
        self.nearest_point_approx(point).distance(point)
    }

    pub fn border_distance(&self, point: FloatPoint) -> f64 {
        self.nearest_border_point_approx(point)
            .map(|p| p.distance(point))
            .unwrap_or(f64::MAX)
    }

    pub fn smallest_radius(&self) -> f64 {
        self.border_distance(self.centre_of_gravity())
    }

    pub fn nearest_point_approx(&self, from_point: FloatPoint) -> FloatPoint {
        if self.contains_float(from_point, 0.0) {
            return from_point;
        }
        self.nearest_border_point_approx(from_point)
            .unwrap_or(from_point)
    }

    /// The exact nearest point to `from_point` on the border of the shape.
    pub fn nearest_border_point(&self, from_point: &Point) -> Option<Point> {
        let line_count = self.border_line_count();
        if line_count == 0 {
            return None;
        }
        let from_point_f = from_point.to_float();
        if line_count == 1 {
            return Some(self.border_line(0).perpendicular_projection(from_point));
        }
        let mut min_dist = f64::MAX;
        let mut min_dist_ind = 0;
        // calculate the distance to the nearest corner first
        for i in 0..line_count {
            let curr_dist = self.corner_approx(i).distance_square(from_point_f);
            if curr_dist < min_dist {
                min_dist = curr_dist;
                min_dist_ind = i;
            }
        }
        let mut nearest_point = self.corner(min_dist_ind);

        let mut prev_ind = line_count - 2;
        let mut curr_ind = line_count - 1;
        for next_ind in 0..line_count {
            let projection = self.border_line(curr_ind).perpendicular_projection(from_point);
            if (!self.corner_is_bounded(curr_ind)
                || self.border_line(prev_ind).side_of(&projection) == Side::OnTheRight)
                && (!self.corner_is_bounded(next_ind)
                    || self.border_line(next_ind).side_of(&projection) == Side::OnTheRight)
            {
                let curr_dist = projection.to_float().distance_square(from_point_f);
                if curr_dist < min_dist {
                    min_dist = curr_dist;
                    nearest_point = projection;
                }
            }
            prev_ind = curr_ind;
            curr_ind = next_ind;
        }
        Some(nearest_point)
    }

    pub fn nearest_point(&self, from_point: &Point) -> Option<Point> {
        if !self.is_outside(from_point) {
            return Some(from_point.clone());
        }
        self.nearest_border_point(from_point)
    }

    pub fn nearest_border_point_approx(&self, from_point: FloatPoint) -> Option<FloatPoint> {
        self.nearest_border_points_approx(from_point, 1)
            .into_iter()
            .next()
    }

    /// Approximations of the `count` nearest points to `from_point` on the
    /// border, on different border lines, sorted ascending by distance.
    pub fn nearest_border_points_approx(
        &self,
        from_point: FloatPoint,
        count: usize,
    ) -> Vec<FloatPoint> {
        if count == 0 {
            return Vec::new();
        }
        let line_count = self.border_line_count();
        if line_count == 0 {
            return Vec::new();
        }
        if line_count == 1 {
            return vec![self.border_line(0).projection_approx(from_point)];
        }
        if self.dimension() == 0 {
            return vec![self.corner_approx(0)];
        }
        let result_count = count.min(line_count);
        let mut candidates: Vec<(f64, FloatPoint)> = Vec::new();

        // the bounded corners first
        for i in 0..line_count {
            if self.corner_is_bounded(i) {
                let corner = self.corner_approx(i);
                candidates.push((corner.distance_square(from_point), corner));
            }
        }
        // then the perpendicular projections that lie on their border
        // segment
        let mut prev_ind = line_count - 2;
        let mut curr_ind = line_count - 1;
        for next_ind in 0..line_count {
            let projection = self.border_line(curr_ind).projection_approx(from_point);
            if (!self.corner_is_bounded(curr_ind)
                || self.border_line(prev_ind).side_of_float(projection, 0.0)
                    == Side::OnTheRight)
                && (!self.corner_is_bounded(next_ind)
                    || self.border_line(next_ind).side_of_float(projection, 0.0)
                        == Side::OnTheRight)
            {
                candidates.push((projection.distance_square(from_point), projection));
            }
            prev_ind = curr_ind;
            curr_ind = next_ind;
        }
        candidates.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap());
        candidates
            .into_iter()
            .take(result_count)
            .map(|(_, p)| p)
            .collect()
    }

    /// The number of the corner nearest to `from_point`.
    ///
    /// Note: the Java original initializes its minimum to
    /// `Double.MIN_VALUE`, so its comparison never fires and it always
    /// returns 0; this port implements the evident intent.
    pub fn index_of_nearest_corner(&self, from_point: &Point) -> usize {
        let from_point_f = from_point.to_float();
        let mut result = 0;
        let mut min_dist = f64::MAX;
        for i in 0..self.border_line_count() {
            let curr_dist = self.corner_approx(i).distance(from_point_f);
            if curr_dist < min_dist {
                min_dist = curr_dist;
                result = i;
            }
        }
        result
    }

    /// A line segment between the approximations of corner 0 and the
    /// opposite corner (`corner_count / 2`); `None` if the shape is empty.
    pub fn diagonal_corner_segment(&self) -> Option<crate::geometry::planar::FloatLine> {
        if self.is_empty() {
            return None;
        }
        let first_corner = self.corner_approx(0);
        let last_corner = self.corner_approx(self.border_line_count() / 2);
        Some(crate::geometry::planar::FloatLine::new(
            first_corner,
            last_corner,
        ))
    }

    /// The minimal distance of `line` to this shape, assuming the line is
    /// on the left of the shape; -1 if the line is on the right or
    /// intersects the interior.
    pub fn distance_to_the_left(&self, line: &Line) -> f64 {
        let mut result = f64::from(i32::MAX);
        for i in 0..self.border_line_count() {
            let curr_corner = self.corner_approx(i);
            let mut line_side = line.side_of_float(curr_corner, 1.0);
            if line_side == Side::Collinear {
                line_side = line.side_of(&self.corner(i));
            }
            if line_side == Side::OnTheRight {
                // the corner would be outside the result shape
                return -1.0;
            }
            result = result.min(line.signed_distance(curr_corner));
        }
        result
    }

    /// `Collinear` if `line` intersects the interior of this shape,
    /// otherwise on which side of the line the shape lies.
    pub fn side_of_line(&self, line: &Line) -> Side {
        let mut on_the_left = false;
        let mut on_the_right = false;
        for i in 0..self.border_line_count() {
            match line.side_of(&self.corner(i)) {
                Side::OnTheLeft => on_the_right = true,
                Side::OnTheRight => on_the_left = true,
                Side::Collinear => {}
            }
            if on_the_left && on_the_right {
                return Side::Collinear;
            }
        }
        if on_the_left {
            Side::OnTheLeft
        } else {
            Side::OnTheRight
        }
    }

    /// True if the line segment has a common point with the interior of
    /// this shape.
    pub fn is_intersected_interior_by(&self, segment: &crate::geometry::planar::LineSegment) -> bool {
        let start_point = segment.start_point();
        let end_point = segment.end_point();
        let float_start_point = start_point.to_float();
        let float_end_point = end_point.to_float();
        let n = self.border_line_count();

        let mut start_sides = Vec::with_capacity(n);
        let mut end_sides = Vec::with_capacity(n);
        for i in 0..n {
            let curr_border_line = self.border_line(i);
            let mut side_of_start = curr_border_line.side_of_float(float_start_point, 1.0);
            if side_of_start == Side::Collinear {
                side_of_start = curr_border_line.side_of(&start_point);
            }
            let mut side_of_end = curr_border_line.side_of_float(float_end_point, 1.0);
            if side_of_end == Side::Collinear {
                side_of_end = curr_border_line.side_of(&end_point);
            }
            if side_of_start != Side::OnTheRight && side_of_end != Side::OnTheRight {
                // both endpoints outside this border line
                return false;
            }
            start_sides.push(side_of_start);
            end_sides.push(side_of_end);
        }
        if start_sides.iter().all(|s| *s == Side::OnTheRight) {
            return true; // start point inside
        }
        if end_sides.iter().all(|s| *s == Side::OnTheRight) {
            return true; // end point inside
        }
        let segment_line = segment.get_line();
        // check if the segment crosses a border line
        for i in 0..n {
            if start_sides[i] == end_sides[i] {
                continue;
            }
            if (start_sides[i] == Side::Collinear && end_sides[i] == Side::OnTheLeft)
                || (end_sides[i] == Side::Collinear && start_sides[i] == Side::OnTheLeft)
            {
                // the interior is not intersected
                continue;
            }
            let mut prev_corner_side = segment_line.side_of_float(self.corner_approx(i), 1.0);
            if prev_corner_side == Side::Collinear {
                prev_corner_side = segment_line.side_of(&self.corner(i));
            }
            let next_corner_index = (i + 1) % n;
            let mut next_corner_side =
                segment_line.side_of_float(self.corner_approx(next_corner_index), 1.0);
            if next_corner_side == Side::Collinear {
                next_corner_side = segment_line.side_of(&self.corner(next_corner_index));
            }
            if (prev_corner_side == Side::OnTheLeft && next_corner_side == Side::OnTheRight)
                || (prev_corner_side == Side::OnTheRight && next_corner_side == Side::OnTheLeft)
            {
                return true;
            }
        }
        false
    }

    /// The half plane on the point-left of `line` as a tile shape
    /// (Java: `TileShape.get_instance(Line)`).
    pub fn half_plane(line: Line) -> TileShape {
        TileShape::Simplex(Simplex::get_instance(vec![line]))
    }

    // ---- intersection and cutout ----

    /// The intersection of this shape with `other`, staying in the most
    /// specific representation (box ∩ box = box, regular ∩ regular =
    /// octagon, anything with a simplex = simplex).
    pub fn intersection(&self, other: &TileShape) -> TileShape {
        match (self, other) {
            (TileShape::Box(a), TileShape::Box(b)) => TileShape::Box(a.intersection(*b)),
            (TileShape::Box(a), TileShape::Octagon(b))
            | (TileShape::Octagon(b), TileShape::Box(a)) => {
                TileShape::Octagon(a.to_int_octagon().intersection(*b))
            }
            (TileShape::Octagon(a), TileShape::Octagon(b)) => {
                TileShape::Octagon(a.intersection(*b))
            }
            (a, b) => TileShape::Simplex(a.to_simplex().intersection(&b.to_simplex())),
        }
    }

    pub fn intersection_with_simplify(&self, other: &TileShape) -> TileShape {
        self.intersection(other).simplify()
    }

    pub fn intersects(&self, other: &TileShape) -> bool {
        match (self, other) {
            (TileShape::Box(a), TileShape::Box(b)) => a.intersects(*b),
            (TileShape::Box(a), TileShape::Octagon(b))
            | (TileShape::Octagon(b), TileShape::Box(a)) => {
                a.to_int_octagon().intersects(*b)
            }
            (TileShape::Octagon(a), TileShape::Octagon(b)) => a.intersects(*b),
            (a, b) => a.to_simplex().intersects(&b.to_simplex()),
        }
    }

    /// Divides this shape minus `hole` into convex pieces
    /// (Java: `this.cutout(p_shape)` is "cut `p_shape` out of `this`").
    pub fn cutout(&self, hole: &TileShape) -> Vec<TileShape> {
        match self {
            TileShape::Box(outer) => {
                // Java IntBox.cutout additionally simplifies each piece.
                let pieces: Vec<TileShape> = match hole {
                    TileShape::Box(h) => h
                        .cutout_from(*outer)
                        .into_iter()
                        .map(TileShape::Box)
                        .collect(),
                    TileShape::Octagon(h) => h
                        .cutout_from_box(*outer)
                        .into_iter()
                        .map(TileShape::Octagon)
                        .collect(),
                    TileShape::Simplex(h) => h
                        .cutout_from(&outer.to_simplex())
                        .into_iter()
                        .map(TileShape::Simplex)
                        .collect(),
                };
                pieces.into_iter().map(TileShape::simplify).collect()
            }
            TileShape::Octagon(outer) => match hole {
                TileShape::Box(h) => h
                    .to_int_octagon()
                    .cutout_from(*outer)
                    .into_iter()
                    .map(TileShape::Octagon)
                    .collect(),
                TileShape::Octagon(h) => h
                    .cutout_from(*outer)
                    .into_iter()
                    .map(TileShape::Octagon)
                    .collect(),
                TileShape::Simplex(h) => h
                    .cutout_from(&outer.to_simplex())
                    .into_iter()
                    .map(TileShape::Simplex)
                    .collect(),
            },
            TileShape::Simplex(outer) => hole
                .to_simplex()
                .cutout_from(outer)
                .into_iter()
                .map(TileShape::Simplex)
                .collect(),
        }
    }
}

impl From<IntBox> for TileShape {
    fn from(b: IntBox) -> Self {
        TileShape::Box(b)
    }
}

impl From<IntOctagon> for TileShape {
    fn from(o: IntOctagon) -> Self {
        TileShape::Octagon(o)
    }
}

impl From<Simplex> for TileShape {
    fn from(s: Simplex) -> Self {
        TileShape::Simplex(s)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geometry::planar::{IntDirection, IntVector};

    #[test]
    fn get_instance_simplifies() {
        // Orthogonal lines: collapses to a Box.
        let box_shape = TileShape::from_convex_polygon(&[
            IntPoint::new(0, 0),
            IntPoint::new(10, 0),
            IntPoint::new(10, 10),
            IntPoint::new(0, 10),
        ]);
        assert!(matches!(box_shape, TileShape::Box(_)));
        assert_eq!(box_shape.bounding_box(), IntBox::from_coords(0, 0, 10, 10));

        // A diamond: collapses to an Octagon.
        let diamond = TileShape::from_convex_polygon(&[
            IntPoint::new(4, 0),
            IntPoint::new(8, 4),
            IntPoint::new(4, 8),
            IntPoint::new(0, 4),
        ]);
        assert!(matches!(diamond, TileShape::Octagon(_)));

        // A general triangle stays a Simplex.
        let triangle = TileShape::from_convex_polygon(&[
            IntPoint::new(0, 0),
            IntPoint::new(10, 0),
            IntPoint::new(3, 7),
        ]);
        assert!(matches!(triangle, TileShape::Simplex(_)));
        assert_eq!(triangle.dimension(), 2);
    }

    #[test]
    fn generic_geometry_consistency() {
        let shapes = [
            TileShape::Box(IntBox::from_coords(0, 0, 10, 10)),
            TileShape::Octagon(
                IntOctagon::new(-4, -4, 4, 4, -4, 4, -4, 4)
                    .normalize()
                    .translate_by(IntVector::new(5, 5)),
            ),
            TileShape::from_convex_polygon(&[
                IntPoint::new(0, 0),
                IntPoint::new(10, 0),
                IntPoint::new(5, 9),
            ]),
        ];
        for shape in &shapes {
            let g = shape.centre_of_gravity();
            assert!(shape.contains_float(g, 0.01), "gravity outside {shape:?}");
            assert!(shape.contains(&Point::Int(g.round())));
            assert!(shape.area() > 0.0);
            assert!(shape.smallest_radius() > 0.0);
            // Every corner is on the border.
            for i in 0..shape.border_line_count() {
                let c = shape.corner(i);
                assert!(shape.contains_on_border(&c), "corner {i} of {shape:?}");
            }
            // A far point is outside, its nearest border point is on the
            // border, and distances agree.
            let far = FloatPoint::new(100.0, 50.0);
            let nb = shape.nearest_border_point_approx(far).unwrap();
            assert!((shape.distance(far) - nb.distance(far)).abs() < 1e-9);
            assert_eq!(shape.side_of_border(nb, 0.01), Side::Collinear);
        }
    }

    #[test]
    fn intersection_kind_promotion() {
        let a = TileShape::Box(IntBox::from_coords(0, 0, 10, 10));
        let b = TileShape::Box(IntBox::from_coords(5, 5, 15, 15));
        assert!(matches!(a.intersection(&b), TileShape::Box(_)));
        let oct = TileShape::Octagon(IntBox::from_coords(2, 2, 8, 8).to_int_octagon());
        assert!(matches!(a.intersection(&oct), TileShape::Octagon(_)));
        let tri = TileShape::from_convex_polygon(&[
            IntPoint::new(0, 0),
            IntPoint::new(10, 0),
            IntPoint::new(3, 7),
        ]);
        assert!(matches!(a.intersection(&tri), TileShape::Simplex(_)));
        assert!(a.intersects(&tri));
        // intersection_with_simplify can demote a simplex result.
        let ortho_simplex = TileShape::Simplex(IntBox::from_coords(1, 1, 9, 9).to_simplex());
        assert!(matches!(
            a.intersection_with_simplify(&ortho_simplex),
            TileShape::Box(_)
        ));
    }

    #[test]
    fn containment_of_shapes() {
        let outer = TileShape::Box(IntBox::from_coords(0, 0, 20, 20));
        let inner = TileShape::from_convex_polygon(&[
            IntPoint::new(2, 2),
            IntPoint::new(8, 2),
            IntPoint::new(5, 6),
        ]);
        assert!(outer.contains_tile(&inner));
        assert!(outer.contains_approx(&inner));
        assert!(!inner.contains_tile(&outer));
    }

    #[test]
    fn cutout_dispatch_covers_all_kinds() {
        let outer_kinds = [
            TileShape::Box(IntBox::from_coords(0, 0, 20, 20)),
            TileShape::Octagon(IntBox::from_coords(0, 0, 20, 20).to_int_octagon()),
            TileShape::Simplex(IntBox::from_coords(0, 0, 20, 20).to_simplex()),
        ];
        let hole_kinds = [
            TileShape::Box(IntBox::from_coords(8, 8, 12, 12)),
            TileShape::Octagon(
                IntOctagon::new(6, 6, 14, 14, -4, 24, 16, 24).normalize(),
            ),
            TileShape::from_convex_polygon(&[
                IntPoint::new(8, 8),
                IntPoint::new(12, 8),
                IntPoint::new(10, 13),
            ]),
        ];
        for outer in &outer_kinds {
            for hole in &hole_kinds {
                let pieces = outer.cutout(hole);
                assert!(!pieces.is_empty(), "{outer:?} cutout {hole:?}");
                let hole_clipped = outer.intersection(hole);
                let pieces_area: f64 = pieces.iter().map(|p| p.area()).sum();
                let expected = outer.area() - hole_clipped.area();
                assert!(
                    (pieces_area - expected).abs() < 1e-6,
                    "area mismatch for {outer:?} cutout {hole:?}: {pieces_area} vs {expected}"
                );
                let hole_center = Point::Int(IntPoint::new(10, 10));
                for piece in &pieces {
                    assert!(!piece.contains_inside(&hole_center));
                }
            }
        }
    }

    #[test]
    fn nearest_corner_and_border_points() {
        let b = TileShape::Box(IntBox::from_coords(0, 0, 10, 10));
        assert_eq!(
            b.index_of_nearest_corner(&Point::Int(IntPoint::new(9, 1))),
            1
        );
        let points = b.nearest_border_points_approx(FloatPoint::new(5.0, -3.0), 2);
        assert_eq!(points.len(), 2);
        assert_eq!(points[0], FloatPoint::new(5.0, 0.0));
        // A half plane has one border line: the projection is returned.
        let half = TileShape::Simplex(Simplex::new(vec![Line::from_direction(
            IntPoint::new(0, 0),
            IntDirection::RIGHT,
        )]));
        assert_eq!(
            half.nearest_border_point_approx(FloatPoint::new(3.0, 5.0)),
            Some(FloatPoint::new(3.0, 0.0))
        );
    }
}
