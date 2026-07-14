//! Port of `board/CalcFromSide.java`: determines the border side of a
//! tile shape from which a polyline / line segment / point enters, used
//! by the shove algorithms to pick the shove direction.

use crate::geometry::planar::{FloatPoint, LineSegment, Point, Polyline, TileShape};

/// The border side of a shape an object enters from.
#[derive(Debug, Clone, PartialEq)]
pub struct CalcFromSide {
    /// The number of the border line, `None` if not calculated.
    pub no: Option<usize>,
    pub border_intersection: Option<FloatPoint>,
}

impl CalcFromSide {
    pub const NOT_CALCULATED: CalcFromSide = CalcFromSide {
        no: None,
        border_intersection: None,
    };

    /// Calculates the edge of `shape` where `polyline` enters. Used in the
    /// push trace algorithm to determine the shove direction; `no` is
    /// expected between 1 and `polyline.line_count() - 2` inclusive.
    pub fn from_polyline(polyline: &Polyline, no: usize, shape: &TileShape) -> CalcFromSide {
        // the edge of the shape where the polyline enters
        let mut curr_no = no;
        while curr_no > 0 {
            let curr_seg = LineSegment::from_polyline(polyline, curr_no);
            let intersections = curr_seg.border_intersections(shape);
            if let Some(&fromside_no) = intersections.first() {
                let intersection = curr_seg
                    .get_line()
                    .intersection_approx(&shape.border_line(fromside_no));
                return CalcFromSide {
                    no: Some(fromside_no),
                    border_intersection: Some(intersection),
                };
            }
            curr_no -= 1;
        }
        // The first corner of the polyline is inside the shape: take the
        // nearest intersection point of polyline.arr[1] with the border.
        let from_point = polyline.corner_approx(0);
        let check_line = polyline.arr[1];
        let mut min_dist = f64::MAX;
        let mut fromside_no = None;
        let mut intersection = None;
        for i in 0..shape.border_line_count() {
            let curr_line = shape.border_line(i);
            let curr_intersection = check_line.intersection_approx(&curr_line);
            let curr_dist = curr_intersection.distance(from_point).abs();
            if curr_dist < min_dist {
                fromside_no = Some(i);
                intersection = Some(curr_intersection);
                min_dist = curr_dist;
            }
        }
        CalcFromSide {
            no: fromside_no,
            border_intersection: intersection,
        }
    }

    /// Calculates the nearest border side of `shape` to `from_point`. Used
    /// in the shove-drill-item algorithm to determine the shove direction.
    pub fn from_point(from_point: &Point, shape: &TileShape) -> CalcFromSide {
        let Some(border_projection) = shape.nearest_border_point(from_point) else {
            return CalcFromSide::NOT_CALCULATED;
        };
        let no = shape.contains_on_border_line_no(&border_projection);
        // Java warns when no side is found and keeps a negative side
        CalcFromSide {
            no,
            border_intersection: Some(border_projection.to_float()),
        }
    }

    /// Calculates the side of `shape` at the start of `line_segment`; with
    /// `shove_to_the_left` the front side number is increased by 2, else
    /// decreased by 2 (modulo the border line count).
    pub fn from_segment(
        line_segment: &LineSegment,
        shape: &TileShape,
        shove_to_the_left: bool,
    ) -> CalcFromSide {
        let start_corner = line_segment.start_point_approx();
        let end_corner = line_segment.end_point_approx();
        let border_line_count = shape.border_line_count();
        let check_line = line_segment.get_line();
        let first_corner = shape.corner_approx(0);
        let mut prev_side = check_line.side_of_float(first_corner, 0.0);
        let mut front_side_no = None;
        for i in 1..=border_line_count {
            let next_corner = if i == border_line_count {
                first_corner
            } else {
                shape.corner_approx(i)
            };
            let next_side = check_line.side_of_float(next_corner, 0.0);
            if prev_side != next_side {
                let curr_intersection = shape
                    .border_line(i - 1)
                    .intersection_approx(&check_line);
                if curr_intersection.distance_square(start_corner)
                    < curr_intersection.distance_square(end_corner)
                {
                    front_side_no = Some(i - 1);
                    break;
                }
            }
            prev_side = next_side;
        }
        let Some(front_side_no) = front_side_no else {
            // Java warns "start corner was not found"
            return CalcFromSide::NOT_CALCULATED;
        };
        let no = if shove_to_the_left {
            (front_side_no + 2) % border_line_count
        } else {
            (front_side_no + border_line_count - 2) % border_line_count
        };
        let prev_corner = shape.corner_approx(no);
        let next_corner = shape.corner_approx((no + 1) % border_line_count);
        CalcFromSide {
            no: Some(no),
            border_intersection: Some(prev_corner.middle_point(next_corner)),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geometry::planar::{IntBox, IntPoint};

    fn square() -> TileShape {
        TileShape::Box(IntBox::from_coords(-100, -100, 100, 100))
    }

    #[test]
    fn polyline_entering_from_the_left() {
        // a horizontal polyline entering the square from the left
        let polyline = Polyline::from_int_points(&[
            IntPoint::new(-300, 0),
            IntPoint::new(0, 0),
            IntPoint::new(0, 300),
        ]);
        let from_side = CalcFromSide::from_polyline(&polyline, 1, &square());
        // IntBox border lines: 0 = bottom, 1 = right, 2 = top, 3 = left
        assert_eq!(from_side.no, Some(3));
        let p = from_side.border_intersection.unwrap();
        assert!((p.x - -100.0).abs() < 1e-6 && p.y.abs() < 1e-6);
    }

    #[test]
    fn nearest_side_of_point() {
        let from_side =
            CalcFromSide::from_point(&Point::Int(IntPoint::new(0, -250)), &square());
        assert_eq!(from_side.no, Some(0)); // the bottom side
    }

    #[test]
    fn segment_shove_sides() {
        // On a 4-sided box both shove directions give the same (opposite)
        // side: +-2 mod 4 coincide, like in Java. On an octagon they
        // differ.
        let polyline = Polyline::from_int_points(&[
            IntPoint::new(-300, 0),
            IntPoint::new(300, 0),
        ]);
        let seg = LineSegment::from_polyline(&polyline, 1);
        let left = CalcFromSide::from_segment(&seg, &square(), true);
        let right = CalcFromSide::from_segment(&seg, &square(), false);
        assert!(left.no.is_some());
        assert_eq!(left.no, right.no);

        let octagon = TileShape::Octagon(
            crate::geometry::planar::Circle::new(IntPoint::new(0, 0), 100)
                .bounding_octagon(),
        );
        let left = CalcFromSide::from_segment(&seg, &octagon, true);
        let right = CalcFromSide::from_segment(&seg, &octagon, false);
        assert!(left.no.is_some() && right.no.is_some());
        assert_ne!(left.no, right.no);
    }
}
