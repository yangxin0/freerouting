//! Depth-1 core of `board/ShoveTraceAlgo.java`: pushes a single trace out
//! of a shove shape by cutting it at the shape and inserting the
//! substitute pieces produced by [`ShapeTraceEntries`], provided the
//! substitutes are free. Recursive shoving (substitutes pushing further
//! traces) follows later; callers fall back to ripup when this fails.

use crate::board::basic_board::{BasicBoard, ItemId};
use crate::board::shape_trace_entries::ShapeTraceEntries;
use crate::board::{CalcFromSide, ItemKind};
use crate::geometry::planar::TileShape;

/// Tries to shove the trace `trace_id` out of `shove_shape` on `layer`
/// while routing `own_net_nos`. Returns true when the trace was cut and
/// its substitute pieces inserted; false leaves the board unchanged.
pub fn shove_aside(
    board: &mut BasicBoard,
    shove_shape: &TileShape,
    layer: usize,
    own_net_nos: &[i32],
    cl_class: usize,
    trace_id: ItemId,
) -> bool {
    let Some(item) = board.get_item(trace_id) else {
        return false;
    };
    let ItemKind::PolylineTrace(trace) = &item.kind else {
        return false;
    };
    if trace.layer != layer || item.base.is_shove_fixed() {
        return false;
    }
    let victim_nets = item.base.net_nos.clone();

    let mut entries = ShapeTraceEntries::new(
        shove_shape.clone(),
        layer,
        own_net_nos.to_vec(),
        cl_class,
        CalcFromSide::NOT_CALCULATED,
    );
    if !entries.store_items(board, &[trace_id], false, false) {
        return false;
    }
    if !entries.shove_via_list.is_empty() {
        return false;
    }
    // collect all substitute pieces up front; every piece must be free
    // before the board is touched
    let mut pieces = Vec::new();
    while let Some(piece) = entries.next_substitute_trace_piece(board) {
        pieces.push(piece);
    }
    if pieces.is_empty() {
        return false;
    }
    for (polyline, piece_layer, half_width, net_nos, piece_cl) in &pieces {
        let clearance = board
            .rules
            .clearance_matrix
            .get_value(*piece_cl, cl_class, *piece_layer, false)
            .max(0) as f64;
        for shape in polyline.offset_shapes(*half_width) {
            let check = shape.offset(clearance);
            // the shoved piece must not collide with anything except the
            // trace being cut (its own net is not an obstacle)
            let blocked = board
                .overlapping_items(&check, Some(*piece_layer))
                .into_iter()
                .any(|id| {
                    if id == trace_id {
                        return false;
                    }
                    board.get_item(id).is_some_and(|other| {
                        if let ItemKind::ObstacleArea(a) = &other.kind {
                            if a.is_conduction {
                                return false;
                            }
                        }
                        !net_nos.iter().any(|n| other.base.contains_net(*n))
                    })
                });
            if blocked {
                return false;
            }
        }
    }
    // commit: cut the victim and insert the substitutes
    entries.cutout_traces(board, &[trace_id]);
    for (polyline, piece_layer, half_width, net_nos, piece_cl) in pieces {
        board.insert_trace(polyline, piece_layer, half_width, net_nos, piece_cl);
    }
    let _ = victim_nets;
    true
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
    fn shoves_a_crossing_trace_and_keeps_it_connected() {
        let mut board = test_board();
        let polyline = Polyline::from_int_points(&[
            IntPoint::new(-10000, 0),
            IntPoint::new(10000, 0),
        ]);
        let victim = board.insert_trace(polyline, 0, 100, vec![2], 1);
        let shape = TileShape::Box(IntBox::from_coords(-1000, -1000, 1000, 1000));
        assert!(shove_aside(&mut board, &shape, 0, &[1], 1, victim));
        // the victim was cut; the substitute keeps net 2 connected around
        // the shape: from one stub end to the other via trace contacts
        let net_items: Vec<ItemId> = board
            .items()
            .filter(|(_, it)| it.base.contains_net(2))
            .map(|(id, _)| *id)
            .collect();
        assert!(net_items.len() >= 3, "stubs + substitute expected");
        let connected = board.get_connected_set(net_items[0], 2);
        assert_eq!(
            connected.len(),
            net_items.len(),
            "the shoved net must stay one connected set"
        );
        // and nothing of net 2 crosses the shove shape interior anymore
        for id in net_items {
            let item = board.get_item(id).unwrap();
            for (s, _) in item.tile_shapes(&board.padstacks) {
                assert!(
                    shape.intersection(s).dimension() < 2,
                    "net 2 copper still inside the shove shape"
                );
            }
        }
    }

    #[test]
    fn refuses_when_the_substitute_is_blocked() {
        let mut board = test_board();
        let polyline = Polyline::from_int_points(&[
            IntPoint::new(-10000, 0),
            IntPoint::new(10000, 0),
        ]);
        let victim = board.insert_trace(polyline, 0, 100, vec![2], 1);
        // a wall of foreign net 3 above and below the shove shape leaves
        // no room for the substitute
        for y in [-2200, 2200] {
            let wall = Polyline::from_int_points(&[
                IntPoint::new(-8000, y),
                IntPoint::new(8000, y),
            ]);
            board.insert_trace(wall, 0, 700, vec![3], 1);
        }
        let shape = TileShape::Box(IntBox::from_coords(-1000, -1000, 1000, 1000));
        let items_before = board.items().count();
        assert!(!shove_aside(&mut board, &shape, 0, &[1], 1, victim));
        assert_eq!(board.items().count(), items_before, "board unchanged");
    }
}
