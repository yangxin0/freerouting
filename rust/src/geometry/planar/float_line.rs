//! Port of `geometry/planar/FloatLine.java`.
//!
//! A line in the plane defined by two [`FloatPoint`]s. Calculations with
//! `FloatLine`s are generally not exact; if exactness is needed, the exact
//! `Line` type (to be ported) is used instead.

use crate::geometry::planar::{limits, FloatPoint};

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct FloatLine {
    pub a: FloatPoint,
    pub b: FloatPoint,
}

impl FloatLine {
    pub const fn new(a: FloatPoint, b: FloatPoint) -> Self {
        FloatLine { a, b }
    }

    /// Returns the line with swapped end points.
    pub fn opposite(self) -> Self {
        FloatLine::new(self.b, self.a)
    }

    /// Flips this line if needed so that it points into the same general
    /// direction as `other`.
    pub fn adjust_direction(self, other: FloatLine) -> Self {
        if self.b.side_of(self.a, other.a) == other.b.side_of(self.a, other.a) {
            self
        } else {
            self.opposite()
        }
    }

    /// The intersection of this line with `other`; `None` if the lines are
    /// parallel.
    pub fn intersection(self, other: FloatLine) -> Option<FloatPoint> {
        let d1x = self.b.x - self.a.x;
        let d1y = self.b.y - self.a.y;
        let d2x = other.b.x - other.a.x;
        let d2y = other.b.y - other.a.y;
        let det_1 = self.a.x * self.b.y - self.a.y * self.b.x;
        let det_2 = other.a.x * other.b.y - other.a.y * other.b.x;
        let det = d2x * d1y - d2y * d1x;
        if det == 0.0 {
            return None;
        }
        Some(FloatPoint::new(
            (d2x * det_1 - d1x * det_2) / det,
            (d2y * det_1 - d1y * det_2) / det,
        ))
    }

    /// Translates the line perpendicular by about `dist`: to the left if
    /// `dist` > 0, else to the right.
    pub fn translate(self, dist: f64) -> Self {
        let dx = self.b.x - self.a.x;
        let dy = self.b.y - self.a.y;
        let dxdx = dx * dx;
        let dydy = dy * dy;
        let length = (dxdx + dydy).sqrt();
        let new_a = if dxdx <= dydy {
            // translate along the x axis
            FloatPoint::new(self.a.x - (dist * length) / dy, self.a.y)
        } else {
            // translate along the y axis
            FloatPoint::new(self.a.x, self.a.y + (dist * length) / dx)
        };
        FloatLine::new(new_a, FloatPoint::new(new_a.x + dx, new_a.y + dy))
    }

    /// The signed distance of this line from `point`: positive if the line
    /// is on the left of `point`, else negative.
    pub fn signed_distance(self, point: FloatPoint) -> f64 {
        let dx = self.b.x - self.a.x;
        let dy = self.b.y - self.a.y;
        // area of the parallelogram spanned by the 3 points
        let det = dy * (point.x - self.a.x) - dx * (point.y - self.a.y);
        det / (dx * dx + dy * dy).sqrt()
    }

    /// An approximation of the perpendicular projection of `point` onto this
    /// line.
    pub fn perpendicular_projection(self, point: FloatPoint) -> FloatPoint {
        let dx = self.b.x - self.a.x;
        let dy = self.b.y - self.a.y;
        if dx == 0.0 && dy == 0.0 {
            return self.a;
        }
        let dxdx = dx * dx;
        let dydy = dy * dy;
        let dxdy = dx * dy;
        let denominator = dxdx + dydy;
        let det = self.a.x * self.b.y - self.b.x * self.a.y;
        FloatPoint::new(
            (point.x * dxdx + point.y * dxdy + det * dy) / denominator,
            (point.x * dxdy + point.y * dydy - det * dx) / denominator,
        )
    }

    /// The distance of `point` to the nearest point of this line between
    /// `a` and `b`.
    pub fn segment_distance(self, point: FloatPoint) -> f64 {
        let projection = self.perpendicular_projection(point);
        if projection.is_contained_in_box(self.a, self.b, 0.01) {
            point.distance(projection)
        } else {
            point.distance(self.a).min(point.distance(self.b))
        }
    }

    /// The perpendicular projection of `line_segment` onto this oriented line
    /// segment; `None` if the projection is empty.
    pub fn segment_projection(self, line_segment: FloatLine) -> Option<FloatLine> {
        if self.b.scalar_product(self.a, line_segment.a) < 0.0 {
            return None;
        }
        if self.a.scalar_product(self.b, line_segment.b) < 0.0 {
            return None;
        }
        let projected_a = if self.a.scalar_product(self.b, line_segment.a) < 0.0 {
            self.a
        } else {
            let p = self.perpendicular_projection(line_segment.a);
            if p.x.abs() >= limits::CRIT_INT as f64 || p.y.abs() >= limits::CRIT_INT as f64 {
                return None;
            }
            p
        };
        let projected_b = if self.b.scalar_product(self.a, line_segment.b) < 0.0 {
            self.b
        } else {
            self.perpendicular_projection(line_segment.b)
        };
        if projected_b.x.abs() >= limits::CRIT_INT as f64
            || projected_b.y.abs() >= limits::CRIT_INT as f64
        {
            return None;
        }
        Some(FloatLine::new(projected_a, projected_b))
    }

    /// The projection of `line_segment` onto this oriented line segment by
    /// moving `line_segment` perpendicular into the direction of this line
    /// segment; `None` if the projection is empty or degenerate.
    pub fn segment_projection_2(self, line_segment: FloatLine) -> Option<FloatLine> {
        if line_segment.a.scalar_product(line_segment.b, self.b) <= 0.0 {
            return None;
        }
        if line_segment.b.scalar_product(line_segment.a, self.a) <= 0.0 {
            return None;
        }
        let projected_a = if line_segment.a.scalar_product(line_segment.b, self.a) < 0.0 {
            let perpendicular_line = FloatLine::new(
                line_segment.a,
                line_segment.b.turn_90_degree_around(1, line_segment.a),
            );
            let p = perpendicular_line.intersection(self)?;
            if p.x.abs() >= limits::CRIT_INT as f64 || p.y.abs() >= limits::CRIT_INT as f64 {
                return None;
            }
            p
        } else {
            self.a
        };
        let projected_b = if line_segment.b.scalar_product(line_segment.a, self.b) < 0.0 {
            let perpendicular_line = FloatLine::new(
                line_segment.b,
                line_segment.a.turn_90_degree_around(1, line_segment.b),
            );
            let p = perpendicular_line.intersection(self)?;
            if p.x.abs() >= limits::CRIT_INT as f64 || p.y.abs() >= limits::CRIT_INT as f64 {
                return None;
            }
            p
        } else {
            self.b
        };
        Some(FloatLine::new(projected_a, projected_b))
    }

    /// Shrinks this line on both sides by `offset`. The result will contain
    /// at least the gravity point of the line.
    pub fn shrink_segment(self, offset: f64) -> Self {
        let dx = self.b.x - self.a.x;
        let dy = self.b.y - self.a.y;
        if dx == 0.0 && dy == 0.0 {
            return self;
        }
        let length = (dx * dx + dy * dy).sqrt();
        let offset = offset.min(length / 2.0);
        let new_a = FloatPoint::new(
            self.a.x + (dx * offset) / length,
            self.a.y + (dy * offset) / length,
        );
        let new_length = length - offset;
        let new_b = FloatPoint::new(
            self.a.x + (dx * new_length) / length,
            self.a.y + (dy * new_length) / length,
        );
        FloatLine::new(new_a, new_b)
    }

    /// The nearest point on this line to `from_point` between `a` and `b`.
    pub fn nearest_segment_point(self, from_point: FloatPoint) -> FloatPoint {
        let projection = self.perpendicular_projection(from_point);
        if projection.is_contained_in_box(self.a, self.b, 0.01) {
            return projection;
        }
        // Now the projection is outside the line segment.
        if from_point.distance_square(self.a) <= from_point.distance_square(self.b) {
            self.a
        } else {
            self.b
        }
    }

    /// Divides this line segment into `count` line segments of nearly equal
    /// length.
    pub fn divide_segment_into_sections(self, count: usize) -> Vec<FloatLine> {
        match count {
            0 => return Vec::new(),
            1 => return vec![self],
            _ => {}
        }
        let line_length = self.b.distance(self.a);
        let section_length = line_length / count as f64;
        let dx = self.b.x - self.a.x;
        let dy = self.b.y - self.a.y;
        let mut result = Vec::with_capacity(count);
        let mut curr_a = self.a;
        for i in 0..count {
            let curr_b = if i == count - 1 {
                self.b
            } else {
                let curr_b_dist = (i + 1) as f64 * section_length;
                FloatPoint::new(
                    self.a.x + (dx * curr_b_dist) / line_length,
                    self.a.y + (dy * curr_b_dist) / line_length,
                )
            };
            result.push(FloatLine::new(curr_a, curr_b));
            curr_a = curr_b;
        }
        result
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geometry::planar::Side;

    fn line(ax: f64, ay: f64, bx: f64, by: f64) -> FloatLine {
        FloatLine::new(FloatPoint::new(ax, ay), FloatPoint::new(bx, by))
    }

    #[test]
    fn intersection() {
        let horizontal = line(0.0, 0.0, 10.0, 0.0);
        let diagonal = line(2.0, -2.0, 6.0, 2.0);
        let p = horizontal.intersection(diagonal).unwrap();
        assert!((p.x - 4.0).abs() < 1e-12 && p.y.abs() < 1e-12);
        assert_eq!(horizontal.intersection(line(0.0, 1.0, 10.0, 1.0)), None);
    }

    #[test]
    fn signed_distance_sign_convention() {
        let l = line(0.0, 0.0, 10.0, 0.0);
        // The line is "on the left" of a point below it -> positive.
        assert!((l.signed_distance(FloatPoint::new(5.0, -2.0)) - 2.0).abs() < 1e-12);
        assert!((l.signed_distance(FloatPoint::new(5.0, 3.0)) + 3.0).abs() < 1e-12);
    }

    #[test]
    fn translate_moves_left_for_positive_dist() {
        let l = line(0.0, 0.0, 10.0, 0.0);
        let t = l.translate(2.0);
        // Left of a line pointing +x is +y.
        assert!((t.a.y - 2.0).abs() < 1e-12);
        assert!((t.b.y - 2.0).abs() < 1e-12);
        assert!((t.a.x - l.a.x).abs() < 1e-12);
    }

    #[test]
    fn projections() {
        let l = line(0.0, 0.0, 10.0, 0.0);
        let p = l.perpendicular_projection(FloatPoint::new(3.0, 5.0));
        assert!((p.x - 3.0).abs() < 1e-12 && p.y.abs() < 1e-12);
        assert!((l.segment_distance(FloatPoint::new(3.0, 5.0)) - 5.0).abs() < 1e-12);
        // Beyond the b end: distance to b.
        assert!((l.segment_distance(FloatPoint::new(13.0, 4.0)) - 5.0).abs() < 1e-12);
        assert_eq!(
            l.nearest_segment_point(FloatPoint::new(13.0, 4.0)),
            FloatPoint::new(10.0, 0.0)
        );
    }

    #[test]
    fn segment_projection_clips() {
        let l = line(0.0, 0.0, 10.0, 0.0);
        let s = line(-3.0, 2.0, 5.0, 2.0);
        let p = l.segment_projection(s).unwrap();
        assert_eq!(p.a, FloatPoint::new(0.0, 0.0));
        assert!((p.b.x - 5.0).abs() < 1e-12 && p.b.y.abs() < 1e-12);
        // Entirely behind a: empty.
        assert_eq!(l.segment_projection(line(-9.0, 1.0, -4.0, 1.0)), None);
    }

    #[test]
    fn shrink_and_divide() {
        let l = line(0.0, 0.0, 10.0, 0.0);
        let s = l.shrink_segment(2.0);
        assert!((s.a.x - 2.0).abs() < 1e-12 && (s.b.x - 8.0).abs() < 1e-12);
        let sections = l.divide_segment_into_sections(4);
        assert_eq!(sections.len(), 4);
        assert_eq!(sections[0].a, l.a);
        assert_eq!(sections[3].b, l.b);
        for (i, sec) in sections.iter().enumerate() {
            assert!((sec.a.x - 2.5 * i as f64).abs() < 1e-12);
        }
    }

    #[test]
    fn adjust_direction() {
        let l = line(0.0, 0.0, 10.0, 0.0);
        let same_dir = line(0.0, 5.0, 10.0, 5.0);
        let opposite_dir = same_dir.opposite();
        assert_eq!(l.adjust_direction(same_dir), l);
        assert_eq!(l.adjust_direction(opposite_dir), l.opposite());
    }

    #[test]
    fn float_point_side_of() {
        let a = FloatPoint::new(0.0, 0.0);
        let b = FloatPoint::new(10.0, 0.0);
        assert_eq!(FloatPoint::new(5.0, 1.0).side_of(a, b), Side::OnTheLeft);
        assert_eq!(FloatPoint::new(5.0, -1.0).side_of(a, b), Side::OnTheRight);
        assert_eq!(FloatPoint::new(20.0, 0.0).side_of(a, b), Side::Collinear);
    }
}
