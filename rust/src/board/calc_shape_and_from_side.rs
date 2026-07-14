//! Port of `board/CalcShapeAndFromSide.java`: calculates the from-side
//! for pushing and cuts off the dog ears of a trace segment shape in the
//! shove algorithm.

use crate::board::calc_from_side::CalcFromSide;
use crate::geometry::planar::{Line, Polyline, Side, TileShape};

/// The (possibly dog-ear-cut) shape of a trace segment together with the
/// side it is entered from.
#[derive(Debug, Clone)]
pub struct CalcShapeAndFromSide {
    pub shape: TileShape,
    pub from_side: CalcFromSide,
}

impl CalcShapeAndFromSide {
    /// Calculates the shape and from-side of the trace segment `index`
    /// (its search-tree shape `segment_shape`). In the shove check
    /// functions `in_shove_check` is expected true, in the actual shove
    /// functions false. `half_width` is the compensated trace half width.
    pub fn new(
        polyline: &Polyline,
        half_width: i32,
        index: usize,
        segment_shape: &TileShape,
        orthogonal: bool,
        in_shove_check: bool,
    ) -> CalcShapeAndFromSide {
        let mut curr_shape;
        let mut curr_from_side: Option<CalcFromSide> = None;
        if orthogonal {
            curr_shape = TileShape::Box(segment_shape.bounding_box());
        } else {
            // prevent dog ears at the start and the end of the substitute
            // trace
            curr_shape = TileShape::Simplex(segment_shape.to_simplex());
            let mut cut_off_at_start = false;
            let mut cut_off_at_end = false;
            let end_cutline = calc_cutline_at_end(index, polyline, half_width);
            if let Some(end_cutline) = end_cutline {
                let cut_plane = TileShape::half_plane(end_cutline);
                let tmp_shape = curr_shape.intersection(&cut_plane);
                if tmp_shape != curr_shape && !tmp_shape.is_empty() {
                    curr_shape = TileShape::Simplex(tmp_shape.to_simplex());
                    cut_off_at_end = true;
                }
            }
            let start_cutline = calc_cutline_at_start(index, polyline, half_width);
            if let Some(start_cutline) = start_cutline {
                let cut_plane = TileShape::half_plane(start_cutline);
                let tmp_shape = curr_shape.intersection(&cut_plane);
                if tmp_shape != curr_shape && !tmp_shape.is_empty() {
                    curr_shape = TileShape::Simplex(tmp_shape.to_simplex());
                    cut_off_at_start = true;
                }
            }
            let mut from_side_no = None;
            let mut curr_cut_line = None;
            if cut_off_at_start {
                curr_cut_line = start_cutline;
                from_side_no = curr_shape
                    .to_simplex()
                    .border_line_index(&start_cutline.unwrap());
            }
            if from_side_no.is_none() && cut_off_at_end {
                curr_cut_line = end_cutline;
                from_side_no = curr_shape
                    .to_simplex()
                    .border_line_index(&end_cutline.unwrap());
            }
            if let (Some(no), Some(cut_line)) = (from_side_no, curr_cut_line) {
                let border_intersection =
                    cut_line.intersection_approx(&curr_shape.border_line(no));
                curr_from_side = Some(CalcFromSide {
                    no: Some(no),
                    border_intersection: Some(border_intersection),
                });
            }
        }
        if curr_from_side.is_none() && !in_shove_check {
            // in a shove check this calculation may produce an undesired
            // stack level > 1 in ShapeTraceEntries (Java comment)
            curr_from_side = Some(CalcFromSide::from_polyline(polyline, index, &curr_shape));
        }
        CalcShapeAndFromSide {
            shape: curr_shape,
            from_side: curr_from_side.unwrap_or(CalcFromSide::NOT_CALCULATED),
        }
    }
}

/// The cut line preventing a dog ear at the trace end, if the segment is
/// near it.
fn calc_cutline_at_end(index: usize, polyline: &Polyline, half_width: i32) -> Option<Line> {
    let len = polyline.arr.len();
    if index == len - 3
        || polyline
            .corner_approx(len - 2)
            .distance(polyline.corner_approx(index + 1))
            < half_width as f64
    {
        let curr_line = polyline.arr[len - 1];
        let is = polyline.corner_approx(len - 3);
        if curr_line.side_of_float(is, 0.0) == Side::OnTheLeft {
            Some(curr_line.opposite())
        } else {
            Some(curr_line)
        }
    } else {
        None
    }
}

/// The cut line preventing a dog ear at the trace start, if the segment is
/// near it.
fn calc_cutline_at_start(index: usize, polyline: &Polyline, half_width: i32) -> Option<Line> {
    if index == 0
        || polyline
            .corner_approx(0)
            .distance(polyline.corner_approx(index))
            < half_width as f64
    {
        let curr_line = polyline.arr[0];
        let is = polyline.corner_approx(1);
        if curr_line.side_of_float(is, 0.0) == Side::OnTheLeft {
            Some(curr_line.opposite())
        } else {
            Some(curr_line)
        }
    } else {
        None
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geometry::planar::IntPoint;

    #[test]
    fn single_segment_gets_cut_and_from_side() {
        // a single-segment trace: shape gets cut at both ends
        let polyline = Polyline::from_int_points(&[
            IntPoint::new(0, 0),
            IntPoint::new(1000, 0),
        ]);
        let shapes = polyline.offset_shapes(100);
        assert_eq!(shapes.len(), 1);
        let result =
            CalcShapeAndFromSide::new(&polyline, 100, 0, &shapes[0], false, false);
        assert!(!result.shape.is_empty());
        // the dog-ear cut shape stays within the offset shape
        assert!(result
            .shape
            .bounding_box()
            .intersects(shapes[0].bounding_box()));
        // from side must be calculated outside a shove check
        assert!(result.from_side.no.is_some());
    }

    #[test]
    fn middle_segment_keeps_shape() {
        // a long middle segment far from both ends keeps its full shape
        let polyline = Polyline::from_int_points(&[
            IntPoint::new(0, 0),
            IntPoint::new(5000, 0),
            IntPoint::new(5000, 5000),
            IntPoint::new(10000, 5000),
        ]);
        let shapes = polyline.offset_shapes(10);
        assert_eq!(shapes.len(), 3);
        let result =
            CalcShapeAndFromSide::new(&polyline, 10, 1, &shapes[1], false, true);
        // in a shove check without end cuts, the from side stays
        // uncalculated
        assert!(!result.shape.is_empty());
        assert!(result.from_side.no.is_none());
    }
}
