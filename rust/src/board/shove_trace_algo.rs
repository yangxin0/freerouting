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
/// Shoves the foreign routable traces overlapping `shove_shape` on
/// `layer` aside while routing `own_net_nos` (Java:
/// `ShoveTraceAlgo.insert`). Substitute pieces recursively shove what
/// blocks them, directed by the from-side derived from the substitute
/// geometry, before being inserted. Returns true when the shape is
/// cleared; false leaves the board unchanged (transactional).
/// `forbidden` are the not-yet-inserted shapes of the pending connection
/// (all corridor segments and via footprints, clearance-inflated): the
/// substitutes must avoid them, since the board cannot show them yet.
pub fn shove_aside(
    board: &mut BasicBoard,
    shove_shape: &TileShape,
    layer: usize,
    own_net_nos: &[i32],
    cl_class: usize,
    forbidden: &[(TileShape, usize)],
) -> bool {
    board.generate_snapshot();
    if shove_insert(
        board,
        shove_shape,
        CalcFromSide::NOT_CALCULATED,
        layer,
        own_net_nos,
        cl_class,
        forbidden,
        4,
    ) {
        board.pop_snapshot();
        true
    } else {
        board.undo();
        false
    }
}

/// The recursive worker (Java: `ShoveTraceAlgo.insert`); on failure the
/// board may be partially changed — the caller rolls back.
#[allow(clippy::too_many_arguments)]
fn shove_insert(
    board: &mut BasicBoard,
    trace_shape: &TileShape,
    from_side: CalcFromSide,
    layer: usize,
    own_net_nos: &[i32],
    cl_class: usize,
    forbidden: &[(TileShape, usize)],
    depth: usize,
) -> bool {
    if trace_shape.is_empty() {
        return true;
    }
    // candidates within clearance of the shape
    let max_cl = board.rules.clearance_matrix.max_value(layer).max(0) as f64;
    let query = trace_shape.offset(max_cl);
    let mut obstacles: Vec<ItemId> = Vec::new();
    for id in board.overlapping_items(&query, Some(layer)) {
        let Some(item) = board.get_item(id) else {
            continue;
        };
        if own_net_nos.iter().any(|n| item.base.contains_net(*n)) {
            continue;
        }
        match &item.kind {
            ItemKind::ObstacleArea(a) => {
                if !a.is_conduction
                    && item
                        .tile_shapes(&board.padstacks)
                        .iter()
                        .any(|(s, l)| *l == layer && s.intersection(&query).dimension() >= 2)
                {
                    return false; // keepouts cannot be shoved
                }
            }
            ItemKind::Via(_) => {
                // via shoving (MoveDrillItemAlgo) not yet ported: fail
                // only when the via actually conflicts with the shape
                let conflicts = item
                    .tile_shapes(&board.padstacks)
                    .iter()
                    .any(|(s, l)| *l == layer && s.intersection(&query).dimension() >= 2);
                if conflicts {
                    return false;
                }
            }
            ItemKind::PolylineTrace(t) => {
                if t.layer != layer {
                    continue;
                }
                if item.base.is_shove_fixed() || !item.is_routable() {
                    // fixed traces block when conflicting incl. clearance
                    let conflicts = item
                        .tile_shapes(&board.padstacks)
                        .iter()
                        .any(|(s, l)| *l == layer && s.intersection(&query).dimension() >= 2);
                    if conflicts {
                        return false;
                    }
                    continue;
                }
                obstacles.push(id);
            }
        }
    }
    let mut entries = ShapeTraceEntries::new(
        trace_shape.clone(),
        layer,
        own_net_nos.to_vec(),
        cl_class,
        from_side,
    );
    // copper sharing allowed like Java's insert
    if !entries.store_items(board, &obstacles, false, true) {
        return false;
    }
    if !entries.shove_via_list.is_empty() {
        return false;
    }
    if crate::debug::shove() {
        eprintln!(
            "SHOVE layer {layer} obstacles {} pieces {} depth {depth}",
            obstacles.len(),
            entries.substitute_trace_count()
        );
    }
    if entries.substitute_trace_count() == 0 {
        return true;
    }
    if depth == 0 {
        return false;
    }
    // cut all victims now; the substitutes reconnect them
    entries.cutout_traces(board, &obstacles);
    while let Some((polyline, piece_layer, half_width, net_nos, piece_cl)) =
        entries.next_substitute_trace_piece(board)
    {
        if polyline.is_empty() || polyline.corner_count() < 2 {
            continue;
        }
        // the substitute must avoid the pending connection's own shapes
        // (they are not on the board yet)
        let segment_shapes = polyline.offset_shapes(half_width);
        let hits_forbidden = segment_shapes.iter().any(|seg| {
            forbidden.iter().any(|(f, fl)| {
                *fl == piece_layer && f.intersection(seg).dimension() >= 2
            })
        });
        if hits_forbidden {
            return false;
        }
        // clear the way for this substitute segment by segment, with the
        // from side derived from the substitute geometry (directs the
        // inner shoves away instead of ping-ponging back)
        for (i, segment_shape) in segment_shapes.iter().enumerate() {
            let calc = crate::board::CalcShapeAndFromSide::new(
                &polyline,
                half_width,
                i,
                segment_shape,
                false,
                false,
            );
            if !shove_insert(
                board,
                &calc.shape,
                calc.from_side,
                piece_layer,
                &net_nos,
                piece_cl,
                forbidden,
                depth - 1,
            ) {
                return false;
            }
        }
        crate::board::basic_board::set_birth_tag(2);
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
        assert!(shove_aside(&mut board, &shape, 0, &[1], 1, &[]));
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
        let _ = shove_aside(&mut board, &shape, 0, &[1], 1, &[]);
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
        let _ = shove_aside(&mut board, &shape, 0, &[1], 1, &[]);
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
        assert!(!shove_aside(&mut board, &shape, 0, &[1], 1, &[]));
        assert_eq!(board.items().count(), items_before, "board unchanged");
    }
}
