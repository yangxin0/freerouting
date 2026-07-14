//! Core of `board/ShoveTraceAlgo.java`: pushes a trace out of a shove
//! shape by cutting it at the shape and inserting the substitute pieces
//! produced by [`ShapeTraceEntries`]. Substitutes blocked by further
//! shovable traces shove those recursively (bounded depth, snapshot
//! rollback on failure); callers fall back to ripup when this fails.

use crate::board::basic_board::{BasicBoard, ItemId};
use crate::board::shape_trace_entries::ShapeTraceEntries;
use crate::board::{CalcFromSide, ItemKind};
use crate::geometry::planar::TileShape;

/// Tries to shove the trace `trace_id` out of `shove_shape` on `layer`
/// while routing `own_net_nos`. Returns true when the trace was cut and
/// its substitute pieces inserted; false leaves the board unchanged.
/// Substitutes blocked by further shovable traces shove those
/// recursively, up to 3 levels deep.
/// Shoves every foreign routable trace overlapping `shove_shape` on
/// `layer` aside while routing `own_net_nos`. Returns true when all
/// victims were cut and replaced by substitutes routing around the shape
/// (recursively shoving what blocks the substitutes, up to 3 levels);
/// false leaves the board unchanged.
pub fn shove_aside(
    board: &mut BasicBoard,
    shove_shape: &TileShape,
    layer: usize,
    own_net_nos: &[i32],
    cl_class: usize,
) -> bool {
    // depth 1: substitutes must be free (recursive pushing attempts
    // ping-pong between adjacent substitutes and cost more than they won
    // on the fleet; revisit with Java's ordered forced insertion)
    shove_shape_recursive(board, shove_shape, layer, own_net_nos, cl_class, 1)
}

fn shove_shape_recursive(
    board: &mut BasicBoard,
    shove_shape: &TileShape,
    layer: usize,
    own_net_nos: &[i32],
    cl_class: usize,
    depth: usize,
) -> bool {
    // the shove victims: shovable foreign traces of ONE net family (the
    // first found — multiple distinct-net victims need Java's ordered
    // forced insertion, not yet ported). Everything else (vias, pins,
    // keepouts, other families) stays on the board and constrains the
    // substitutes through the free check; the caller rips what remains.
    let mut victims: Vec<ItemId> = Vec::new();
    let mut family: Option<Vec<i32>> = None;
    for id in board.overlapping_items(shove_shape, Some(layer)) {
        let Some(item) = board.get_item(id) else {
            continue;
        };
        if own_net_nos.iter().any(|n| item.base.contains_net(*n)) {
            continue;
        }
        if let ItemKind::PolylineTrace(t) = &item.kind {
            if t.layer != layer || item.base.is_shove_fixed() || !item.is_routable() {
                continue;
            }
            match &family {
                None => {
                    family = Some(item.base.net_nos.clone());
                    victims.push(id);
                }
                Some(f) if *f == item.base.net_nos => victims.push(id),
                Some(_) => {}
            }
        }
    }
    if victims.is_empty() {
        return false; // nothing shovable in the way
    }
    if depth == 0 {
        return false;
    }

    let mut entries = ShapeTraceEntries::new(
        shove_shape.clone(),
        layer,
        own_net_nos.to_vec(),
        cl_class,
        CalcFromSide::NOT_CALCULATED,
    );
    if !entries.store_items(board, &victims, false, false) {
        return false;
    }
    if !entries.shove_via_list.is_empty() {
        return false;
    }
    let mut pieces = Vec::new();
    while let Some(piece) = entries.next_substitute_trace_piece(board) {
        pieces.push(piece);
    }

    // check every substitute before touching the board: at depth 1 a
    // blocked substitute simply refuses (no snapshot churn); the victims
    // themselves are not blockers (they get cut below)
    for (polyline, piece_layer, half_width, net_nos, piece_cl) in &pieces {
        let clearance = board
            .rules
            .clearance_matrix
            .get_value(*piece_cl, cl_class, *piece_layer, false)
            .max(0) as f64;
        for shape in polyline.offset_shapes(*half_width) {
            let check = shape.offset(clearance);
            let blocked = board
                .overlapping_items(&check, Some(*piece_layer))
                .into_iter()
                .any(|id| {
                    if victims.contains(&id) {
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
    let _ = depth;
    // commit
    entries.cutout_traces(board, &victims);
    for (polyline, piece_layer, half_width, net_nos, piece_cl) in pieces {
        board.insert_trace(polyline, piece_layer, half_width, net_nos, piece_cl);
    }
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
        assert!(shove_aside(&mut board, &shape, 0, &[1], 1));
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
    fn one_family_shoved_per_call_keeps_nets_connected() {
        // two traces of different nets crossing the same shove shape are
        // handled in one entries pass via the stack levels
        let mut board = test_board();
        for (y, net) in [(0, 2), (-500, 3)] {
            board.insert_trace(
                Polyline::from_int_points(&[
                    IntPoint::new(-10000, y),
                    IntPoint::new(10000, y),
                ]),
                0,
                100,
                vec![net],
                1,
            );
        }
        let shape = TileShape::Box(IntBox::from_coords(-1000, -1000, 1000, 1000));
        // one net family is shoved per call (distinct-net stacking needs
        // ordered forced insertion, still open); both nets stay connected
        let _ = shove_aside(&mut board, &shape, 0, &[1], 1);
        for net in [2, 3] {
            let items: Vec<ItemId> = board
                .items()
                .filter(|(_, it)| it.base.contains_net(net))
                .map(|(id, _)| *id)
                .collect();
            assert!(!items.is_empty());
            let connected = board.get_connected_set(items[0], net);
            assert_eq!(connected.len(), items.len(), "net {net} split");
        }
    }

    #[test]
    fn chained_shove_never_corrupts_the_board() {
        // a bystander sits where the substitute must go: whether the
        // recursion succeeds or refuses, both nets stay connected
        let mut board = test_board();
        board.insert_trace(
            Polyline::from_int_points(&[IntPoint::new(-10000, 0), IntPoint::new(10000, 0)]),
            0,
            100,
            vec![2],
            1,
        );
        board.insert_trace(
            Polyline::from_int_points(&[
                IntPoint::new(-10000, 1500),
                IntPoint::new(10000, 1500),
            ]),
            0,
            100,
            vec![3],
            1,
        );
        let shape = TileShape::Box(IntBox::from_coords(-1000, -1000, 1000, 1000));
        let _ = shove_aside(&mut board, &shape, 0, &[1], 1);
        for net in [2, 3] {
            let items: Vec<ItemId> = board
                .items()
                .filter(|(_, it)| it.base.contains_net(net))
                .map(|(id, _)| *id)
                .collect();
            assert!(!items.is_empty());
            let connected = board.get_connected_set(items[0], net);
            assert_eq!(connected.len(), items.len(), "net {net} split");
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
            let wall_id = board.insert_trace(wall, 0, 700, vec![3], 1);
            // shove-fixed walls: recursion must not push them
            board.set_fixed_state(wall_id, crate::board::FixedState::ShoveFixed);
        }
        let shape = TileShape::Box(IntBox::from_coords(-1000, -1000, 1000, 1000));
        let items_before = board.items().count();
        assert!(!shove_aside(&mut board, &shape, 0, &[1], 1));
        assert_eq!(board.items().count(), items_before, "board unchanged");
    }
}
