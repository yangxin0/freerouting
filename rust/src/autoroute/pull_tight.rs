//! Simplified pull-tight optimization (the corner-elimination core of
//! `board/PullTightAlgoAnyAngle.java`): repeatedly drops trace corners
//! whose direct bypass segment stays clear of foreign items, shortening
//! and smoothing routed traces.

use crate::board::basic_board::{BasicBoard, ItemId};
use crate::board::ItemKind;
use crate::geometry::planar::{IntPoint, Polyline, TileShape};

/// Pulls one trace tight by corner elimination. Returns the id of the
/// (possibly replaced) trace and the number of corners removed.
pub fn pull_tight_trace(board: &mut BasicBoard, id: ItemId) -> (ItemId, usize) {
    // pull-tight reinserts the trace; the diagnostic birth of the
    // ORIGINAL inserting mechanism is preserved (a plain tag here made
    // every trace look pull-tight-born)
    let original_birth = board.get_item(id).map(|i| i.base.birth).unwrap_or(0);
    let Some(item) = board.get_item(id).cloned() else {
        return (id, 0);
    };
    let ItemKind::PolylineTrace(trace) = &item.kind else {
        return (id, 0);
    };
    // Java PullTightAlgo: shove-fixed traces (which includes user-fixed)
    // are never pulled — their geometry is protected
    if item.base.is_shove_fixed() || trace.corner_count() < 3 {
        return (id, 0);
    }
    let net_no = item.base.net_nos.first().copied().unwrap_or(0);
    let layer = trace.layer;
    let half_width = trace.half_width;
    let clearance_class = item.base.clearance_class;
    let clearance_class_explicit = item.base.clearance_class_explicit;
    let mut corners: Vec<IntPoint> = trace
        .polyline
        .corner_approx_arr()
        .iter()
        .map(|c| c.round())
        .collect();

    // remove the trace so it does not block its own bypass segments
    board.remove_item(id);

    let mut removed = 0usize;
    let mut changed = true;
    while changed && corners.len() > 2 {
        changed = false;
        let mut i = 1;
        while i + 1 < corners.len() {
            let (a, c) = (corners[i - 1], corners[i + 1]);
            if a == c {
                corners.remove(i);
                removed += 1;
                changed = true;
                continue;
            }
            let bypass = Polyline::from_two_points(a, c);
            // under an angle restriction only compliant bypasses are
            // allowed (Java: PullTightAlgo45/90 keep the restriction)
            let restriction = board.rules.get_trace_angle_restriction();
            if restriction != crate::board::AngleRestriction::None {
                let (fa, fc) = (
                    crate::geometry::planar::FloatPoint::new(a.x as f64, a.y as f64),
                    crate::geometry::planar::FloatPoint::new(c.x as f64, c.y as f64),
                );
                let dx = (fc.x - fa.x).abs();
                let dy = (fc.y - fa.y).abs();
                let compliant = match restriction {
                    crate::board::AngleRestriction::NinetyDegree => dx == 0.0 || dy == 0.0,
                    _ => dx == 0.0 || dy == 0.0 || dx == dy,
                };
                if !compliant {
                    i += 1;
                    continue;
                }
            }
            // the bypass must keep the clearance, not merely avoid
            // touching (zero-margin bypasses were a DRC leak)
            let max_cl = board.rules.clearance_matrix.max_value(layer).max(0);
            let free = !bypass.is_empty()
                && bypass
                    .offset_shape(half_width + max_cl, 0)
                    .map(|shape: TileShape| !board.is_blocked(&shape, layer, net_no))
                    .unwrap_or(false);
            if free {
                corners.remove(i);
                removed += 1;
                changed = true;
            } else {
                i += 1;
            }
        }
    }

    // the corners are ROUNDED approximations of line intersections:
    // ill-conditioned corners (e.g. combine junctions between nearly
    // collinear traces) can drift 10+ units, moving segments the bypass
    // checks never validated into foreign clearance. Validate the whole
    // rebuilt polyline EXACTLY (both miter directions, like the DRC
    // audit); keep the original geometry when anything is blocked
    // (unchanged geometry = unchanged DRC status).
    let rebuilt = Polyline::from_int_points(&corners);
    // the trace's contacts register at its EXACT end corners; rounding
    // may move a rational endpoint (polyline_path wiring) off the pad or
    // junction it touched, silently disconnecting the net — a rebuild
    // that moves an endpoint is discarded (as is a no-op rebuild)
    let endpoints_kept = removed > 0
        && !rebuilt.is_empty()
        && rebuilt.corner_approx(0) == trace.polyline.corner_approx(0)
        && rebuilt.corner_approx(rebuilt.corner_count() - 1)
            == trace
                .polyline
                .corner_approx(trace.polyline.corner_count() - 1);
    let rebuilt_free = endpoints_kept
        && polyline_keeps_clearance(board, &rebuilt, half_width, layer, net_no, clearance_class);
    let (polyline, removed) = if rebuilt_free {
        (rebuilt, removed)
    } else {
        (trace.polyline.clone(), 0)
    };
    let new_id = {
        let _birth_tag = crate::board::basic_board::birth_tag_scope(3);
        board.insert_trace_with_provenance(
            polyline,
            layer,
            half_width,
            item.base.net_nos.clone(),
            clearance_class,
            clearance_class_explicit,
        )
    };
    if original_birth != 0 {
        board.set_birth(new_id, original_birth);
    }
    if crate::debug::maze() {
        audit_foreign_clearance(
            board,
            new_id,
            &format!("TIGHT(rebuilt-free {rebuilt_free}, removed {removed})"),
        );
    }
    (new_id, removed)
}

/// Pulls all unfixed traces of the board tight until no more corners can
/// be removed (bounded by `max_rounds`). Returns the total number of
/// corners removed.
pub fn pull_tight_all(board: &mut BasicBoard, max_rounds: usize) -> usize {
    let mut total_removed = 0usize;
    for _ in 0..max_rounds.max(1) {
        let trace_ids: Vec<ItemId> = board
            .items()
            .filter(|(_, item)| {
                matches!(item.kind, ItemKind::PolylineTrace(_)) && !item.base.is_shove_fixed()
            })
            .map(|(id, _)| *id)
            .collect();
        let mut removed_this_round = 0usize;
        for id in trace_ids {
            let (_, removed) = pull_tight_trace(board, id);
            removed_this_round += removed;
        }
        total_removed += removed_this_round;
        if removed_this_round == 0 {
            break;
        }
    }
    total_removed
}

/// Combines every trace of the board with its simple-joint neighbours
/// (same net family, layer, width, exactly one trace contact at the
/// corner). Reduces the fragmentation left by junction splitting and
/// shove cutouts. Returns the number of removed items.
pub fn combine_all_traces(board: &mut BasicBoard) -> usize {
    let _birth_tag = crate::board::basic_board::birth_tag_scope(4);
    let before = board.items().count();
    let ids: Vec<crate::board::ItemId> = board
        .items()
        .filter(|(_, it)| matches!(it.kind, ItemKind::PolylineTrace(_)))
        .map(|(id, _)| *id)
        .collect();
    for id in ids {
        // ids removed by earlier combines are skipped inside
        let current = board.combine_trace(id);
        if crate::debug::maze() && current != id {
            audit_foreign_clearance(board, current, "COMBINE");
        }
    }
    // redundant cycles (parallel paths back to the same endpoints) are
    // removed after combining, like Java's normalize pass
    // (BasicBoard.remove_if_cycle); repeated because a removal can
    // expose another cycle
    loop {
        let ids: Vec<crate::board::ItemId> = board
            .items()
            .filter(|(_, it)| matches!(it.kind, ItemKind::PolylineTrace(_)))
            .map(|(id, _)| *id)
            .collect();
        let removed_cycles = ids
            .into_iter()
            .filter(|&id| board.remove_if_cycle(id))
            .count();
        if removed_cycles == 0 {
            break;
        }
    }
    before.saturating_sub(board.items().count())
}

/// True if every segment of `poly` (at `half_width` on `layer`) keeps
/// the pairwise clearance to every foreign item, in BOTH miter
/// directions: inflation is a mitered line-push, so miter(a, cl)
/// missing b does not imply miter(b, cl) misses a at diagonal corners —
/// the DRC audit checks both, so must the pull-tight gate.
fn polyline_keeps_clearance(
    board: &BasicBoard,
    poly: &Polyline,
    half_width: i32,
    layer: usize,
    net_no: i32,
    clearance_class: usize,
) -> bool {
    let max_cl = board.rules.clearance_matrix.max_value(layer).max(0);
    for i in 0..poly.corner_count().saturating_sub(1) {
        let Some(seg) = poly.offset_shape(half_width, i) else {
            return false;
        };
        let query = seg.offset(2.0 * max_cl as f64);
        for oid in board.overlapping_items(&query, Some(layer)) {
            let Some(other) = board.get_item(oid) else {
                continue;
            };
            if other.base.contains_net(net_no) {
                continue;
            }
            if let ItemKind::ObstacleArea(a) = &other.kind {
                if (a.is_conduction && !a.is_obstacle) || a.via_only {
                    continue;
                }
            }
            let cl =
                crate::drc::clearance_for_new_item(board, other, clearance_class, layer) as i32;
            // direction a: our inflated segment vs their copper
            let check = seg.offset(cl.max(0) as f64);
            if other
                .tile_shapes(&board.padstacks)
                .iter()
                .any(|(os, ol)| *ol == layer && os.intersection(&check).dimension() >= 2)
            {
                return false;
            }
            // direction b: their inflated copper vs our segment
            if let Some(inflated) = board.inflated_shapes(oid, cl.max(0)) {
                if inflated.iter().any(|(os, obb, ol)| {
                    *ol == layer
                        && obb.intersects(seg.bounding_box())
                        && os.intersection(&seg).dimension() >= 2
                }) {
                    return false;
                }
            }
        }
    }
    true
}

/// Debug audit: report every foreign item within pairwise clearance of
/// `id`'s copper (used to attribute post-processing DRC violations).
#[doc(hidden)]
pub fn audit_foreign_clearance(board: &BasicBoard, id: ItemId, tag: &str) {
    let Some(item) = board.get_item(id).cloned() else {
        return;
    };
    for (s, l) in item.tile_shapes(&board.padstacks) {
        for oid in board.overlapping_items(&s.offset(10_000.0), Some(*l)) {
            let Some(other) = board.get_item(oid) else {
                continue;
            };
            if oid == id || other.base.shares_net(&item.base) {
                continue;
            }
            if let ItemKind::ObstacleArea(a) = &other.kind {
                if (a.is_conduction && !a.is_obstacle) || a.via_only {
                    continue;
                }
            }
            let cl = crate::drc::required_clearance(board, &item, other, *l).unwrap_or(0.0);
            let check = s.offset(cl - 2.0);
            if other
                .tile_shapes(&board.padstacks)
                .iter()
                .any(|(os, ol)| ol == l && os.intersection(&check).dimension() >= 2)
            {
                let kind = |it: &crate::board::Item| match &it.kind {
                    ItemKind::Via(_) => "via",
                    ItemKind::PolylineTrace(_) => "trace",
                    ItemKind::ObstacleArea(_) => "area",
                };
                eprintln!(
                    "{tag} VIOLATION {} {id} (nets {:?}, bbox {:?}) vs {} {oid} \
                     (nets {:?}, birth {}, bbox {:?}) layer {l}",
                    kind(&item),
                    item.base.net_nos,
                    item.bounding_box(&board.padstacks),
                    kind(other),
                    other.base.net_nos,
                    other.base.birth,
                    other.bounding_box(&board.padstacks),
                );
            }
        }
    }
}

/// The cumulative length of all traces of the board.
pub fn total_trace_length(board: &BasicBoard) -> f64 {
    board
        .items()
        .filter_map(|(_, item)| match &item.kind {
            ItemKind::PolylineTrace(t) => Some(t.get_length()),
            _ => None,
        })
        .sum()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::board::{Layer, LayerStructure};
    use crate::core::Padstacks;
    use crate::geometry::planar::{IntBox, PolygonShape, PolylineArea};
    use crate::rules::{BoardRules, ClearanceMatrix};

    fn test_board() -> BasicBoard {
        let stack = LayerStructure::new(vec![Layer::new("F.Cu", true)]);
        let matrix = ClearanceMatrix::get_default_instance(stack.clone(), 200);
        let mut rules = BoardRules::new(stack.clone(), matrix);
        rules.get_default_net_class();
        let mut padstacks = Padstacks::new(1);
        padstacks.add_shape_on_layers(
            TileShape::Box(IntBox::from_coords(-300, -300, 300, 300)),
            0,
            0,
        );
        BasicBoard::new(stack, rules, padstacks)
    }

    fn zigzag_corners() -> Vec<IntPoint> {
        vec![
            IntPoint::new(0, 0),
            IntPoint::new(2000, 2000),
            IntPoint::new(4000, 0),
            IntPoint::new(6000, 2000),
            IntPoint::new(8000, 0),
        ]
    }

    #[test]
    fn straightens_unobstructed_zigzag() {
        let mut board = test_board();
        let a = board.insert_via(1, IntPoint::new(0, 0), vec![1], 1, false);
        let b = board.insert_via(1, IntPoint::new(8000, 0), vec![1], 1, false);
        let trace = board.insert_trace(
            Polyline::from_int_points(&zigzag_corners()),
            0,
            100,
            vec![1],
            1,
        );
        board.set_item_clearance_class_explicit(trace, true);
        assert!(board.net_is_completely_connected(1));
        let before = total_trace_length(&board);

        let removed = pull_tight_all(&mut board, 4);
        assert!(removed >= 3, "only {removed} corners removed");
        let after = total_trace_length(&board);
        assert!(
            after < before * 0.9,
            "length not reduced: {before} -> {after}"
        );
        // connectivity preserved, endpoints intact
        assert!(board.net_is_completely_connected(1));
        assert!(board.get_item(a).is_some() && board.get_item(b).is_some());
        assert!(board.get_item(trace).is_none(), "trace was replaced");
        assert!(board.items().any(|(_, item)| {
            matches!(item.kind, ItemKind::PolylineTrace(_))
                && item.base.contains_net(1)
                && item.base.clearance_class_explicit
        }));
    }

    #[test]
    fn keeps_detour_around_obstacle() {
        let mut board = test_board();
        board.insert_via(1, IntPoint::new(0, 0), vec![1], 1, false);
        board.insert_via(1, IntPoint::new(8000, 0), vec![1], 1, false);
        // a keepout blocking the straight line between the pads
        let wall = PolylineArea::new(
            PolygonShape::from_int_points(&[
                IntPoint::new(3800, -1000),
                IntPoint::new(4200, -1000),
                IntPoint::new(4200, 1500),
                IntPoint::new(3800, 1500),
            ]),
            vec![],
        );
        board.insert_area(wall, 0, "wall", vec![], 1, false);
        board.insert_trace(
            Polyline::from_int_points(&zigzag_corners()),
            0,
            100,
            vec![1],
            1,
        );
        pull_tight_all(&mut board, 4);
        // still connected, and the remaining trace avoids the wall
        assert!(board.net_is_completely_connected(1));
        let blocked = board.is_blocked(
            &TileShape::Box(IntBox::from_coords(3800, -100, 4200, 100)),
            0,
            1,
        );
        assert!(blocked, "wall region should still be blocked (by the wall)");
        for (_, item) in board.items() {
            if let ItemKind::PolylineTrace(t) = &item.kind {
                // no trace corner inside the wall
                for c in t.polyline.corner_approx_arr() {
                    assert!(
                        !(c.x > 3800.0 && c.x < 4200.0 && c.y > -1000.0 && c.y < 1500.0),
                        "corner inside the wall"
                    );
                }
                // the trace must still detour above the wall
                assert!(t.corner_count() > 2, "detour was flattened away");
            }
        }
    }

    #[test]
    fn fixed_traces_untouched() {
        let mut board = test_board();
        let trace = board.insert_trace(
            Polyline::from_int_points(&zigzag_corners()),
            0,
            100,
            vec![1],
            1,
        );
        board.set_fixed_state(trace, crate::board::FixedState::UserFixed);
        assert_eq!(pull_tight_all(&mut board, 2), 0);
        assert!(board.get_item(trace).is_some());
    }
}
