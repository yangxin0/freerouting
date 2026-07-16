//! Port of `geometry/planar/PolygonShape.java` (with the parts of
//! `PolylineShape.java` it uses).
//!
//! A possibly concave polygon shape, normalized at construction to
//! counterclockwise corners starting at the lowest (y, then x) corner.
//! The central algorithm is `split_to_convex`, dividing the polygon into
//! convex tile pieces along axis-parallel division lines constructed at
//! concave corners.

use crate::geometry::planar::{
    FloatPoint, IntBox, IntOctagon, IntPoint, IntVector, Line, Point, Polygon, Side, TileShape,
};

#[derive(Debug, Clone, PartialEq)]
pub struct PolygonShape {
    pub corners: Vec<Point>,
}

/// Deterministic replacement for Java's seeded `Random` used to pick the
/// start corner in `split_to_convex` (the exact choice only affects which
/// valid decomposition is produced).
struct Lcg(u64);

impl Lcg {
    fn next_below(&mut self, bound: usize) -> usize {
        self.0 = self
            .0
            .wrapping_mul(6364136223846793005)
            .wrapping_add(1442695040888963407);
        ((self.0 >> 33) as usize) % bound.max(1)
    }
}

impl PolygonShape {
    pub fn new(corner_arr: Vec<Point>) -> Self {
        Self::from_polygon(&Polygon::new(corner_arr))
    }

    pub fn from_int_points(points: &[IntPoint]) -> Self {
        Self::new(points.iter().map(|p| Point::Int(*p)).collect())
    }

    pub fn from_polygon(polygon: &Polygon) -> Self {
        let polygon = if polygon.winding_number_after_closing() < 0 {
            // the corners are in clockwise sense
            polygon.revert_corners()
        } else {
            polygon.clone()
        };
        let curr_corners = polygon.corner_array();
        let mut last_corner_no = curr_corners.len().saturating_sub(1);
        if last_corner_no > 0 && curr_corners[0] == curr_corners[last_corner_no] {
            last_corner_no -= 1; // skip the repeated closing point
        }
        if last_corner_no >= 2
            && curr_corners[last_corner_no]
                .side_of(&curr_corners[last_corner_no - 1], &curr_corners[0])
                == Side::Collinear
        {
            last_corner_no -= 1; // skip the collinear last point
        }
        let mut first_corner_no = 0;
        if last_corner_no >= 2
            && curr_corners[0].side_of(&curr_corners[1], &curr_corners[last_corner_no])
                == Side::Collinear
        {
            first_corner_no += 1; // skip the collinear first point
        }
        // rotate so the corner with lowest y (then lowest x) comes first
        let mut start_corner_no = first_corner_no;
        let mut start_corner = curr_corners[start_corner_no].to_float();
        for i in start_corner_no + 1..=last_corner_no {
            let curr = curr_corners[i].to_float();
            if curr.y < start_corner.y || (curr.y == start_corner.y && curr.x < start_corner.x) {
                start_corner_no = i;
                start_corner = curr;
            }
        }
        let mut corners = Vec::with_capacity(last_corner_no - first_corner_no + 1);
        corners.extend_from_slice(&curr_corners[start_corner_no..=last_corner_no]);
        corners.extend_from_slice(&curr_corners[first_corner_no..start_corner_no]);
        PolygonShape { corners }
    }

    pub fn corner(&self, no: usize) -> &Point {
        &self.corners[no]
    }

    pub fn border_line_count(&self) -> usize {
        self.corners.len()
    }

    pub fn is_empty(&self) -> bool {
        self.corners.is_empty()
    }

    pub fn is_bounded(&self) -> bool {
        true
    }

    pub fn dimension(&self) -> i32 {
        match self.corners.len() {
            0 => -1,
            1 => 0,
            2 => 1,
            _ => 2,
        }
    }

    fn corner_int(&self, no: usize) -> IntPoint {
        match &self.corners[no] {
            Point::Int(p) => *p,
            Point::Rational(_) => self.corners[no].to_float().round(),
        }
    }

    /// The border line from corner `no` to the next corner.
    pub fn border_line(&self, no: usize) -> Line {
        let next = (no + 1) % self.corners.len();
        Line::new(self.corner_int(no), self.corner_int(next))
    }

    pub fn centre_of_gravity(&self) -> FloatPoint {
        let n = self.corners.len() as f64;
        let (mut x, mut y) = (0.0, 0.0);
        for c in &self.corners {
            let f = c.to_float();
            x += f.x;
            y += f.y;
        }
        FloatPoint::new(x / n, y / n)
    }

    /// The area of this polygon via the shoelace formula.
    ///
    /// Note: the Java original tests `dimension() <= 2` (always true) and
    /// therefore always returns 0; this port implements the evident intent
    /// (`< 2`).
    pub fn area(&self) -> f64 {
        if self.dimension() < 2 {
            return 0.0;
        }
        let n = self.corners.len();
        let mut result = 0.0;
        let mut prev_corner = self.corners[n - 2].to_float();
        let mut curr_corner = self.corners[n - 1].to_float();
        for i in 0..n {
            let next_corner = self.corners[i].to_float();
            result += curr_corner.x * (next_corner.y - prev_corner.y);
            prev_corner = curr_corner;
            curr_corner = next_corner;
        }
        0.5 * result.abs()
    }

    pub fn bounding_box(&self) -> IntBox {
        let (mut llx, mut lly) = (f64::MAX, f64::MAX);
        let (mut urx, mut ury) = (f64::MIN, f64::MIN);
        for c in &self.corners {
            let f = c.to_float();
            llx = llx.min(f.x);
            lly = lly.min(f.y);
            urx = urx.max(f.x);
            ury = ury.max(f.y);
        }
        IntBox::from_coords(
            llx.floor() as i32,
            lly.floor() as i32,
            urx.ceil() as i32,
            ury.ceil() as i32,
        )
    }

    pub fn bounding_octagon(&self) -> IntOctagon {
        let (mut lx, mut ly) = (f64::MAX, f64::MAX);
        let (mut rx, mut uy) = (f64::MIN, f64::MIN);
        let (mut ulx, mut llx) = (f64::MAX, f64::MAX);
        let (mut lrx, mut urx) = (f64::MIN, f64::MIN);
        for c in &self.corners {
            let f = c.to_float();
            lx = lx.min(f.x);
            ly = ly.min(f.y);
            rx = rx.max(f.x);
            uy = uy.max(f.y);
            let tmp = f.x - f.y;
            ulx = ulx.min(tmp);
            lrx = lrx.max(tmp);
            let tmp = f.x + f.y;
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
        .normalize()
    }

    /// True if no corner turns to the right (with an additional
    /// angle-sum check against self-wrapping borders, like Java).
    pub fn is_convex(&self) -> bool {
        if self.corners.len() <= 2 {
            return true;
        }
        let n = self.corners.len();
        let mut prev_point = &self.corners[n - 1];
        let mut curr_point = &self.corners[0];
        for ind in 0..n {
            let next_point = &self.corners[(ind + 1) % n];
            if next_point.side_of(prev_point, curr_point) == Side::OnTheRight {
                return false;
            }
            prev_point = curr_point;
            curr_point = next_point;
        }
        // check that the sum of the interior angles is at most 2 pi
        let first_line = Line::new(self.corner_int(n - 1), self.corner_int(0));
        let mut curr_line = Line::new(self.corner_int(0), self.corner_int(1));
        let first_direction = first_line.direction();
        let mut last_det = first_direction.determinant(curr_line.direction());
        for ind2 in 2..n {
            curr_line = Line::new(curr_line.b, self.corner_int(ind2));
            let curr_det = first_direction.determinant(curr_line.direction());
            if last_det <= 0 && curr_det > 0 {
                return false;
            }
            last_det = curr_det;
        }
        true
    }

    /// The convex hull: repeatedly drops corners that do not turn left.
    pub fn convex_hull(&self) -> PolygonShape {
        let mut current = self.clone();
        'outer: loop {
            if current.corners.len() <= 2 {
                return current;
            }
            let n = current.corners.len();
            let mut prev_point = current.corners[n - 1].clone();
            let mut curr_point = current.corners[0].clone();
            for ind in 0..n {
                let next_point = current.corners[(ind + 1) % n].clone();
                if next_point.side_of(&prev_point, &curr_point) != Side::OnTheLeft {
                    // skip curr_point (which is corners[ind])
                    let mut new_corners = current.corners.clone();
                    new_corners.remove(ind);
                    current = PolygonShape::new(new_corners);
                    continue 'outer;
                }
                prev_point = curr_point;
                curr_point = next_point;
            }
            return current;
        }
    }

    /// The smallest convex tile shape containing this polygon.
    pub fn bounding_tile(&self) -> TileShape {
        let hull = self.convex_hull();
        let n = hull.corners.len();
        let lines: Vec<Line> = (0..n).map(|i| hull.border_line(i)).collect();
        TileShape::get_instance(lines)
    }

    // ---- point containment (via the convex pieces, like Java) ----

    pub fn contains_float(&self, point: FloatPoint) -> bool {
        self.split_to_convex()
            .is_some_and(|pieces| pieces.iter().any(|p| p.contains_float(point, 0.0)))
    }

    pub fn is_outside(&self, point: &Point) -> bool {
        match self.split_to_convex() {
            Some(pieces) => pieces.iter().all(|p| p.is_outside(point)),
            None => true,
        }
    }

    pub fn contains(&self, point: &Point) -> bool {
        !self.is_outside(point)
    }

    // ---- transforms ----

    pub fn translate_by(&self, vector: IntVector) -> Self {
        if vector.is_zero() {
            return self.clone();
        }
        PolygonShape::new(
            self.corners
                .iter()
                .map(|c| c.translate_by(&crate::geometry::planar::Vector::Int(vector)))
                .collect(),
        )
    }

    pub fn turn_90_degree(&self, factor: i32, pole: IntPoint) -> Self {
        let pole = Point::Int(pole);
        PolygonShape::new(
            self.corners
                .iter()
                .map(|c| c.turn_90_degree(factor, &pole))
                .collect(),
        )
    }

    pub fn rotate_approx(&self, angle: f64, pole: FloatPoint) -> Self {
        if angle == 0.0 {
            return self.clone();
        }
        PolygonShape::new(
            self.corners
                .iter()
                .map(|c| Point::Int(c.to_float().rotate(angle, pole).round()))
                .collect(),
        )
    }

    pub fn mirror_vertical(&self, pole: IntPoint) -> Self {
        let pole = Point::Int(pole);
        PolygonShape::new(
            self.corners
                .iter()
                .map(|c| c.mirror_vertical(&pole))
                .collect(),
        )
    }

    pub fn mirror_horizontal(&self, pole: IntPoint) -> Self {
        let pole = Point::Int(pole);
        PolygonShape::new(
            self.corners
                .iter()
                .map(|c| c.mirror_horizontal(&pole))
                .collect(),
        )
    }

    // ---- convex decomposition ----

    /// Splits this polygon into convex tile pieces. The result is not
    /// exact because rounded intersections are used at the division
    /// points. Returns `None` if the split fails (e.g. the polygon has
    /// self-intersections).
    pub fn split_to_convex(&self) -> Option<Vec<TileShape>> {
        if self.corners.is_empty() {
            return Some(Vec::new());
        }
        let mut rng = Lcg(99);
        let pieces = self.split_to_convex_recu(&mut rng)?;
        Some(
            pieces
                .into_iter()
                .map(|piece| {
                    let n = piece.corners.len();
                    let lines: Vec<Line> = (0..n).map(|i| piece.border_line(i)).collect();
                    TileShape::get_instance(lines)
                })
                .collect(),
        )
    }

    fn split_to_convex_recu(&self, rng: &mut Lcg) -> Option<Vec<PolygonShape>> {
        // start at a hashed corner and search the first concave corner
        let n = self.corners.len();
        if n < 3 {
            return Some(vec![self.clone()]);
        }
        let mut start_corner_no = rng.next_below(n);
        let mut curr_corner = &self.corners[start_corner_no];
        let mut prev_corner = if start_corner_no != 0 {
            &self.corners[start_corner_no - 1]
        } else {
            &self.corners[n - 1]
        };
        let mut concave_corner_no: Option<usize> = None;
        for _ in 0..n {
            let next_corner = &self.corners[(start_corner_no + 1) % n];
            if next_corner.side_of(prev_corner, curr_corner) == Side::OnTheRight {
                concave_corner_no = Some(start_corner_no);
                break;
            }
            prev_corner = curr_corner;
            curr_corner = next_corner;
            start_corner_no = (start_corner_no + 1) % n;
        }
        let Some(concave_corner_no) = concave_corner_no else {
            // no concave corner: already convex
            return Some(vec![self.clone()]);
        };
        let (projection, corner_no_after_projection) = self.division_point(concave_corner_no)?;

        // construct the two result pieces
        let mut corner_count = corner_no_after_projection as isize - concave_corner_no as isize;
        if corner_count < 0 {
            corner_count += n as isize;
        }
        let corner_count = corner_count as usize + 1;
        let mut first_arr: Vec<Point> = Vec::with_capacity(corner_count);
        let mut corner_ind = concave_corner_no;
        for _ in 0..corner_count - 1 {
            first_arr.push(self.corners[corner_ind].clone());
            corner_ind = (corner_ind + 1) % n;
        }
        first_arr.push(Point::Int(projection.round()));
        let first_piece = PolygonShape::new(first_arr);

        let mut corner_count = concave_corner_no as isize - corner_no_after_projection as isize;
        if corner_count < 0 {
            corner_count += n as isize;
        }
        let corner_count = corner_count as usize + 2;
        let mut last_arr: Vec<Point> = Vec::with_capacity(corner_count);
        last_arr.push(Point::Int(projection.round()));
        let mut corner_ind = corner_no_after_projection;
        for _ in 1..corner_count {
            last_arr.push(self.corners[corner_ind].clone());
            corner_ind = (corner_ind + 1) % n;
        }
        let last_piece = PolygonShape::new(last_arr);

        let mut result = first_piece.split_to_convex_recu(rng)?;
        result.extend(last_piece.split_to_convex_recu(rng)?);
        Some(result)
    }

    /// At a concave corner, constructs a minimal axis-parallel division
    /// point on the border (Java: inner class `DivisionPoint`). Returns
    /// the projection point and the corner index after it.
    fn division_point(&self, concave_corner_no: usize) -> Option<(FloatPoint, usize)> {
        let n = self.corners.len();
        let concave_corner = self.corners[concave_corner_no].to_float();
        let before_concave_corner = if concave_corner_no != 0 {
            self.corners[concave_corner_no - 1].to_float()
        } else {
            self.corners[n - 1].to_float()
        };
        let after_concave_corner = if concave_corner_no == n - 1 {
            self.corners[0].to_float()
        } else {
            self.corners[concave_corner_no + 1].to_float()
        };

        let search_right =
            before_concave_corner.y > concave_corner.y || concave_corner.y > after_concave_corner.y;
        let search_left =
            before_concave_corner.y < concave_corner.y || concave_corner.y < after_concave_corner.y;
        let search_up =
            before_concave_corner.x < concave_corner.x || concave_corner.x < after_concave_corner.x;
        let search_down =
            before_concave_corner.x > concave_corner.x || concave_corner.x > after_concave_corner.x;

        let mut min_projection_dist = f64::from(i32::MAX);
        let mut min_projection: Option<FloatPoint> = None;
        let mut corner_no_after_min_projection = 0;

        let mut corner_no_after_curr_projection = (concave_corner_no + 2) % n;
        let mut corner_before_curr_projection = if corner_no_after_curr_projection != 0 {
            self.corner_int(corner_no_after_curr_projection - 1)
        } else {
            self.corner_int(n - 1)
        };
        let mut corner_before_projection_approx = corner_before_curr_projection.to_float();

        for _ in 0..n.saturating_sub(2) {
            let corner_after_curr_projection = self.corner_int(corner_no_after_curr_projection);
            let corner_after_projection_approx = corner_after_curr_projection.to_float();
            if corner_before_projection_approx.y != corner_after_projection_approx.y {
                // try a horizontal division
                let (min_y, max_y) =
                    if corner_after_projection_approx.y > corner_before_projection_approx.y {
                        (
                            corner_before_projection_approx.y,
                            corner_after_projection_approx.y,
                        )
                    } else {
                        (
                            corner_after_projection_approx.y,
                            corner_before_projection_approx.y,
                        )
                    };
                if concave_corner.y >= min_y && concave_corner.y <= max_y {
                    let curr_line =
                        Line::new(corner_before_curr_projection, corner_after_curr_projection);
                    let x_intersect = curr_line.function_in_y_value_approx(concave_corner.y);
                    let curr_dist = (x_intersect - concave_corner.x).abs();
                    // the new shape must not be concave at the projection
                    let projection_ok = curr_dist < min_projection_dist
                        && ((search_right
                            && x_intersect > concave_corner.x
                            && concave_corner.y <= corner_after_projection_approx.y)
                            || (search_left
                                && x_intersect < concave_corner.x
                                && concave_corner.y >= corner_after_projection_approx.y));
                    if projection_ok {
                        min_projection_dist = curr_dist;
                        corner_no_after_min_projection = corner_no_after_curr_projection;
                        min_projection = Some(FloatPoint::new(x_intersect, concave_corner.y));
                    }
                }
            }
            if corner_before_projection_approx.x != corner_after_projection_approx.x {
                // try a vertical division
                let (min_x, max_x) =
                    if corner_after_projection_approx.x > corner_before_projection_approx.x {
                        (
                            corner_before_projection_approx.x,
                            corner_after_projection_approx.x,
                        )
                    } else {
                        (
                            corner_after_projection_approx.x,
                            corner_before_projection_approx.x,
                        )
                    };
                if concave_corner.x >= min_x && concave_corner.x <= max_x {
                    let curr_line =
                        Line::new(corner_before_curr_projection, corner_after_curr_projection);
                    let y_intersect = curr_line.function_value_approx(concave_corner.x);
                    let curr_dist = (y_intersect - concave_corner.y).abs();
                    let projection_ok = curr_dist < min_projection_dist
                        && ((search_up
                            && y_intersect > concave_corner.y
                            && concave_corner.x >= corner_after_projection_approx.x)
                            || (search_down
                                && y_intersect < concave_corner.y
                                && concave_corner.x <= corner_after_projection_approx.x));
                    if projection_ok {
                        min_projection_dist = curr_dist;
                        corner_no_after_min_projection = corner_no_after_curr_projection;
                        min_projection = Some(FloatPoint::new(concave_corner.x, y_intersect));
                    }
                }
            }
            corner_before_curr_projection = corner_after_curr_projection;
            corner_before_projection_approx = corner_after_projection_approx;
            corner_no_after_curr_projection = (corner_no_after_curr_projection + 1) % n;
        }
        min_projection.map(|p| (p, corner_no_after_min_projection))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn shape(points: &[(i32, i32)]) -> PolygonShape {
        PolygonShape::from_int_points(
            &points
                .iter()
                .map(|(x, y)| IntPoint::new(*x, *y))
                .collect::<Vec<_>>(),
        )
    }

    #[test]
    fn normalization() {
        // clockwise input gets reverted; starts at lowest y then x
        let s = shape(&[(0, 10), (10, 10), (10, 0), (0, 0)]);
        assert_eq!(s.corners.len(), 4);
        assert_eq!(*s.corner(0), Point::Int(IntPoint::new(0, 0)));
        assert!(s.is_convex());
        assert!((s.area() - 100.0).abs() < 1e-9);
        assert_eq!(s.bounding_box(), IntBox::from_coords(0, 0, 10, 10));
        assert_eq!(s.dimension(), 2);
        // winding number checks
        let ccw = Polygon::new(vec![
            Point::Int(IntPoint::new(0, 0)),
            Point::Int(IntPoint::new(10, 0)),
            Point::Int(IntPoint::new(10, 10)),
        ]);
        assert_eq!(ccw.winding_number_after_closing(), 1);
        assert_eq!(ccw.revert_corners().winding_number_after_closing(), -1);
    }

    #[test]
    fn convexity_and_hull() {
        let l_shape = shape(&[(0, 0), (20, 0), (20, 10), (10, 10), (10, 20), (0, 20)]);
        assert!(!l_shape.is_convex());
        let hull = l_shape.convex_hull();
        assert!(hull.is_convex());
        assert_eq!(hull.corners.len(), 5);
        assert!((hull.area() - 350.0).abs() < 1e-9);
        let tile = l_shape.bounding_tile();
        assert!(tile.contains(&Point::Int(IntPoint::new(15, 12)))); // in hull, not in L
    }

    #[test]
    fn split_l_shape_to_convex() {
        let l_shape = shape(&[(0, 0), (20, 0), (20, 10), (10, 10), (10, 20), (0, 20)]);
        let pieces = l_shape.split_to_convex().unwrap();
        assert!(pieces.len() >= 2);
        let total: f64 = pieces.iter().map(|p| p.area()).sum();
        assert!(
            (total - l_shape.area()).abs() < 1e-6,
            "pieces area {total} != polygon area {}",
            l_shape.area()
        );
        // containment via pieces
        assert!(l_shape.contains(&Point::Int(IntPoint::new(5, 15))));
        assert!(l_shape.contains(&Point::Int(IntPoint::new(15, 5))));
        assert!(l_shape.is_outside(&Point::Int(IntPoint::new(15, 15))));
        assert!(l_shape.contains_float(FloatPoint::new(5.0, 5.0)));
        assert!(!l_shape.contains_float(FloatPoint::new(15.0, 15.0)));
    }

    #[test]
    fn split_u_shape_to_convex() {
        // a U shape with two concave corners
        let u = shape(&[
            (0, 0),
            (30, 0),
            (30, 20),
            (20, 20),
            (20, 10),
            (10, 10),
            (10, 20),
            (0, 20),
        ]);
        assert!(!u.is_convex());
        let pieces = u.split_to_convex().unwrap();
        let total: f64 = pieces.iter().map(|p| p.area()).sum();
        assert!((total - u.area()).abs() < 1e-6, "{total} vs {}", u.area());
        assert!(u.contains(&Point::Int(IntPoint::new(5, 15))));
        assert!(u.contains(&Point::Int(IntPoint::new(25, 15))));
        assert!(u.is_outside(&Point::Int(IntPoint::new(15, 15))));
    }

    #[test]
    fn transforms() {
        let s = shape(&[(0, 0), (10, 0), (10, 10), (0, 10)]);
        let t = s.translate_by(IntVector::new(5, 5));
        assert_eq!(t.bounding_box(), IntBox::from_coords(5, 5, 15, 15));
        let turned = s.turn_90_degree(1, IntPoint::new(0, 0));
        assert_eq!(turned.bounding_box(), IntBox::from_coords(-10, 0, 0, 10));
        let mirrored = s.mirror_vertical(IntPoint::new(0, 0));
        assert_eq!(mirrored.bounding_box(), IntBox::from_coords(-10, 0, 0, 10));
    }
}
