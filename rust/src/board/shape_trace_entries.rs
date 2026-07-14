//! Port of the standalone part of `board/ShapeTraceEntries.java`:
//! `cutout_trace`, which removes the part of a trace inside a shape and
//! reinserts the outside pieces. The entry-point bookkeeping of the full
//! shove workhorse follows with `ShoveTraceAlgo`.

use crate::board::basic_board::{BasicBoard, ItemId};
use crate::board::ItemKind;
use crate::geometry::planar::TileShape;

/// Java: `ShapeTraceEntries.c_offset_add`.
const C_OFFSET_ADD: f64 = 1.0;

/// Cuts out the part of the trace `trace_id` inside `shape` (enlarged by
/// the trace half width plus the clearance to `cl_class`) and reinserts
/// the remaining outside pieces. Returns the ids of the inserted pieces
/// (empty when nothing was cut).
pub fn cutout_trace(
    board: &mut BasicBoard,
    trace_id: ItemId,
    shape: &TileShape,
    cl_class: usize,
) -> Vec<ItemId> {
    let Some(item) = board.get_item(trace_id) else {
        // Java warns "trace is deleted"
        return Vec::new();
    };
    let ItemKind::PolylineTrace(trace) = &item.kind else {
        return Vec::new();
    };
    // enlarge the shape in 2 steps for symmetry reasons (Java comment)
    let cl_offset = board.rules.clearance_matrix.get_value(
        item.base.clearance_class,
        cl_class,
        trace.layer,
        false,
    ) as f64
        + C_OFFSET_ADD;
    let offset_shape = shape.offset(trace.half_width as f64).offset(cl_offset);
    let pieces = offset_shape.cutout_polyline(&trace.polyline);
    if pieces.len() == 1 && pieces[0] == trace.polyline {
        // nothing cut off
        return Vec::new();
    }
    let layer = trace.layer;
    let half_width = trace.half_width;
    let net_nos = item.base.net_nos.clone();
    let clearance_class = item.base.clearance_class;
    board.remove_item(trace_id);
    let mut inserted = Vec::new();
    for piece in pieces {
        if piece.is_empty() {
            continue;
        }
        inserted.push(board.insert_trace(
            piece,
            layer,
            half_width,
            net_nos.clone(),
            clearance_class,
        ));
    }
    inserted
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::board::{Layer, LayerStructure};
    use crate::core::Padstacks;
    use crate::geometry::planar::{IntBox, IntPoint, Polyline};
    use crate::rules::{BoardRules, ClearanceMatrix};

    fn test_board() -> BasicBoard {
        let stack = LayerStructure::new(vec![Layer::new("F.Cu", true)]);
        let matrix = ClearanceMatrix::get_default_instance(stack.clone(), 200);
        let mut rules = BoardRules::new(stack.clone(), matrix);
        rules.get_default_net_class();
        BasicBoard::new(stack, rules, Padstacks::new(1))
    }

    #[test]
    fn cuts_crossing_trace_into_two_pieces() {
        let mut board = test_board();
        let polyline = Polyline::from_int_points(&[
            IntPoint::new(-10000, 0),
            IntPoint::new(10000, 0),
        ]);
        let trace = board.insert_trace(polyline, 0, 100, vec![1], 1);
        let shape = TileShape::Box(IntBox::from_coords(-1000, -1000, 1000, 1000));
        let pieces = cutout_trace(&mut board, trace, &shape, 1);
        assert_eq!(pieces.len(), 2);
        assert!(board.get_item(trace).is_none(), "the original is removed");
        // the pieces end outside the enlarged shape: half width 100 +
        // clearance 200 + 1 = x beyond +-1301
        for id in pieces {
            let item = board.get_item(id).unwrap();
            let ItemKind::PolylineTrace(t) = &item.kind else {
                panic!("piece is a trace");
            };
            for c in t.polyline.corner_approx_arr() {
                assert!(
                    c.x.abs() >= 1300.0,
                    "piece corner {c:?} inside the cut region"
                );
            }
        }
    }

    #[test]
    fn disjoint_trace_is_untouched() {
        let mut board = test_board();
        let polyline = Polyline::from_int_points(&[
            IntPoint::new(5000, 5000),
            IntPoint::new(9000, 5000),
        ]);
        let trace = board.insert_trace(polyline, 0, 100, vec![1], 1);
        let shape = TileShape::Box(IntBox::from_coords(-1000, -1000, 1000, 1000));
        let pieces = cutout_trace(&mut board, trace, &shape, 1);
        assert!(pieces.is_empty());
        assert!(board.get_item(trace).is_some(), "the original stays");
    }
}
