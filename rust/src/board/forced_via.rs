//! Port of `ForcedViaAlgo.java`: checking and inserting a via at a
//! location after shoving obstacle traces (and vias) aside. Java's pure
//! `check` becomes check-by-doing inside a snapshot transaction.

use crate::board::basic_board::{BasicBoard, ItemId};
use crate::board::shove_trace_algo::shove_aside;
use crate::geometry::planar::{IntBox, IntPoint, IntVector, TileShape};

#[derive(Debug, Clone, Copy)]
pub(crate) struct ViaInsertionPolicy {
    pub attach_allowed: bool,
    pub escape_smd_layer: Option<usize>,
}

/// The via pad shapes of `padstack` translated to `location`, with their
/// layers, plus the start-trace shape when the trace pen is wider than
/// the pad (Java: the `start_trace_circle` handling).
fn forced_shapes(
    board: &BasicBoard,
    via_padstack: usize,
    location: IntPoint,
    trace_half_width: i32,
) -> Vec<(TileShape, usize)> {
    let Some(ps) = board.padstacks.get_by_no(via_padstack) else {
        return Vec::new();
    };
    if !ps.has_shapes_at_transition_endpoints(ps.from_layer(), ps.to_layer()) {
        return Vec::new();
    }
    let mut result = Vec::new();
    for layer in ps.from_layer()..=ps.to_layer() {
        let Some(shape) = ps.get_shape(layer) else {
            continue;
        };
        let pad = shape.translate_by(IntVector::new(location.x, location.y));
        let pad_bb = pad.bounding_box();
        result.push((pad, layer));
        if trace_half_width > 0 {
            // make space for starting a trace in case the trace is wider
            // than the via pad
            let trace_bb = IntBox::from_coords(
                location.x - trace_half_width,
                location.y - trace_half_width,
                location.x + trace_half_width,
                location.y + trace_half_width,
            );
            if trace_bb.max_width() > pad_bb.max_width() {
                result.push((TileShape::Box(trace_bb), layer));
            }
        }
    }
    result
}

/// Shoves space free and inserts a via of `via_padstack` at `location`
/// (Java: `ForcedViaAlgo.insert`). Transactional: on failure the board
/// is unchanged and None is returned.
pub fn insert_forced_via(
    board: &mut BasicBoard,
    via_padstack: usize,
    location: IntPoint,
    net_nos: &[i32],
    cl_class: usize,
    trace_half_width: i32,
    attach_allowed: bool,
) -> Option<ItemId> {
    insert_forced_via_with_escape(
        board,
        via_padstack,
        location,
        net_nos,
        cl_class,
        trace_half_width,
        ViaInsertionPolicy {
            attach_allowed,
            escape_smd_layer: None,
        },
    )
}

/// Escape-aware variant used only by the maze insertion path. The selected
/// ViaInfo's attach bit stays unchanged; `escape_smd_layer` supplies the one
/// layer-scoped same-net SMD exception when the search landed on a pin.
pub(crate) fn insert_forced_via_with_escape(
    board: &mut BasicBoard,
    via_padstack: usize,
    location: IntPoint,
    net_nos: &[i32],
    cl_class: usize,
    trace_half_width: i32,
    policy: ViaInsertionPolicy,
) -> Option<ItemId> {
    let shapes = forced_shapes(board, via_padstack, location, trace_half_width);
    if shapes.is_empty() {
        return None;
    }
    board.generate_snapshot();
    let watermark = board.begin_lineage_drc_transaction();
    for (shape, layer) in &shapes {
        let max_cl = board.rules.clearance_matrix.max_value(*layer).max(0) as f64;
        let corridor = shape.offset(max_cl + 1.0);
        if !shove_aside(board, &corridor, *layer, net_nos, cl_class, &[]) {
            if crate::debug::shove() {
                eprintln!("FORCED VIA shove failed at {location:?} layer {layer}");
            }
            board.discard_lineage_drc_transaction(watermark);
            board.rollback_snapshot();
            return None;
        }
        // the space must actually be free now (unshovable items remain)
        let blocked = board
            .overlapping_items(&corridor, Some(*layer))
            .into_iter()
            .any(|id| {
                board.get_item(id).is_some_and(|it| {
                    !it.base.net_nos.iter().any(|n| net_nos.contains(n))
                        && !matches!(&it.kind, crate::board::ItemKind::ObstacleArea(a) if a.is_conduction && !a.is_obstacle)
                })
            });
        if blocked {
            if crate::debug::shove() {
                eprintln!("FORCED VIA blocked at {location:?} layer {layer}");
            }
            board.discard_lineage_drc_transaction(watermark);
            board.rollback_snapshot();
            return None;
        }
    }
    let id = if let Some(layer) = policy.escape_smd_layer {
        board.insert_escape_via(
            via_padstack,
            location,
            net_nos.to_vec(),
            cl_class,
            policy.attach_allowed,
            layer,
        )
    } else {
        board.insert_via(
            via_padstack,
            location,
            net_nos.to_vec(),
            cl_class,
            policy.attach_allowed,
        )
    };
    // Validate the entire transaction.  Shoving may have inserted foreign
    // substitute traces; checking only the via lets an invalid substitute
    // escape into the committed board.
    if !board.finish_lineage_drc_transaction(watermark) {
        if crate::debug::shove() {
            eprintln!("FORCED VIA violates drill clearance at {location:?}");
        }
        board.rollback_snapshot();
        return None;
    }
    board.pop_snapshot();
    Some(id)
}

/// True if a via of `via_padstack` fits at `location` after shoving
/// (Java: `ForcedViaAlgo.check`). The board is left unchanged.
pub fn check_forced_via(
    board: &mut BasicBoard,
    via_padstack: usize,
    location: IntPoint,
    net_nos: &[i32],
    cl_class: usize,
    trace_half_width: i32,
) -> bool {
    board.generate_snapshot();
    let ok = insert_forced_via(
        board,
        via_padstack,
        location,
        net_nos,
        cl_class,
        trace_half_width,
        false,
    )
    .is_some();
    board.rollback_snapshot();
    ok
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::board::{Layer, LayerStructure};
    use crate::core::Padstacks;
    use crate::geometry::planar::Polyline;
    use crate::rules::{BoardRules, ClearanceMatrix};

    fn test_board() -> BasicBoard {
        let stack = LayerStructure::new(vec![Layer::new("F.Cu", true), Layer::new("B.Cu", true)]);
        let matrix = ClearanceMatrix::get_default_instance(stack.clone(), 200);
        let mut rules = BoardRules::new(stack.clone(), matrix);
        rules.get_default_net_class();
        let mut padstacks = Padstacks::new(2);
        padstacks.add_shape_on_layers(
            TileShape::Box(IntBox::from_coords(-400, -400, 400, 400)),
            0,
            1,
        );
        BasicBoard::new(stack, rules, padstacks)
    }

    #[test]
    fn forced_via_shoves_a_blocking_trace() {
        let mut board = test_board();
        // a foreign trace running straight over the target location
        let blocker = board.insert_trace(
            Polyline::from_int_points(&[IntPoint::new(-20000, 0), IntPoint::new(20000, 0)]),
            0,
            100,
            vec![2],
            1,
        );
        let via = insert_forced_via(&mut board, 1, IntPoint::new(0, 0), &[1], 1, 100, false);
        assert!(via.is_some(), "forced via must succeed by shoving");
        // the blocker net must still be connected (its trace was shoved,
        // possibly replaced by pieces, never just deleted)
        assert!(board.net_is_completely_connected(2));
        let _ = blocker;
        // and the via must keep clearance to every foreign item
        let via_item = board.get_item(via.unwrap()).unwrap().clone();
        for (s, l) in via_item.tile_shapes(&board.padstacks) {
            for oid in board.overlapping_items(&s.offset(400.0), Some(*l)) {
                let other = board.get_item(oid).unwrap();
                if oid == via.unwrap() || other.base.shares_net(&via_item.base) {
                    continue;
                }
                assert!(
                    s.euclidean_distance_to(
                        &other.tile_shapes(&board.padstacks).first().unwrap().0
                    ) >= 199.0,
                    "via must keep clearance after the shove"
                );
            }
        }
    }

    #[test]
    fn forced_via_honors_same_net_drill_rule_beyond_matrix_max() {
        // the review repro: a via_via_same_net of 3000 with a matrix max of
        // 200 used to be accepted here (the blocked check skipped ALL
        // same-net items) and then reported by the authoritative DRC
        let mut board = test_board();
        board.insert_via(1, IntPoint::new(0, 0), vec![1], 1, false);
        // 600-unit edge gap to the existing same-net via: fine at the
        // matrix clearance (200)...
        assert!(
            insert_forced_via(&mut board, 1, IntPoint::new(1400, 0), &[1], 1, 100, false).is_some(),
            "no same-net rule: the ordinary clearance allows the site"
        );
        let mut board = test_board();
        board.insert_via(1, IntPoint::new(0, 0), vec![1], 1, false);
        board.rules.set_same_net_clearance(
            crate::rules::ItemClass::Via,
            crate::rules::ItemClass::Via,
            3000,
        );
        let items_before = board.item_count();
        assert!(
            insert_forced_via(&mut board, 1, IntPoint::new(1400, 0), &[1], 1, 100, false).is_none(),
            "a same-net drill rule beyond the matrix max must reject the site"
        );
        assert_eq!(
            board.item_count(),
            items_before,
            "the rejected insert must leave the board unchanged"
        );
        assert!(crate::drc::check_board(&board).violations.is_empty());
    }

    #[test]
    fn check_leaves_board_unchanged() {
        let mut board = test_board();
        board.insert_trace(
            Polyline::from_int_points(&[IntPoint::new(-20000, 0), IntPoint::new(20000, 0)]),
            0,
            100,
            vec![2],
            1,
        );
        let items_before: Vec<_> = board.items().map(|(id, _)| *id).collect();
        assert!(check_forced_via(
            &mut board,
            1,
            IntPoint::new(0, 0),
            &[1],
            1,
            100
        ));
        let items_after: Vec<_> = board.items().map(|(id, _)| *id).collect();
        assert_eq!(items_before, items_after, "check must not change the board");
    }
}
