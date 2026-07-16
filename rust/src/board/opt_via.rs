//! Port of `OptViaAlgo.java`: optimizing via locations. A via contacted
//! by exactly two traces (one per layer) is moved toward the adjacent
//! trace corners when the move is legal (forced-via check with shoving)
//! and shortens the connection; the trace stubs are reconnected to the
//! new location. Transactional per attempt like the rest of the port.

use crate::board::basic_board::{BasicBoard, ItemId};
use crate::board::forced_via::insert_forced_via;
use crate::board::ItemKind;
use crate::geometry::planar::{IntPoint, Point, Polyline};

/// The two trace contacts of a via, when there are exactly two and both
/// are unfixed polyline traces (Java's precondition).
fn two_trace_contacts(board: &BasicBoard, via_id: ItemId) -> Option<(ItemId, ItemId)> {
    let contacts = board.get_normal_contacts(via_id);
    if contacts.len() != 2 {
        return None;
    }
    for &c in &contacts {
        let item = board.get_item(c)?;
        if item.base.is_user_fixed() || !matches!(item.kind, ItemKind::PolylineTrace(_)) {
            return None;
        }
    }
    Some((contacts[0], contacts[1]))
}

/// The corner of `trace_id` adjacent to the via end, plus which end
/// touches the via (Java: the `first/second_trace_from_corner` logic
/// with the via-radius tolerance).
fn from_corner(
    board: &BasicBoard,
    trace_id: ItemId,
    via_center: IntPoint,
    tolerance: f64,
) -> Option<(IntPoint, bool)> {
    let item = board.get_item(trace_id)?;
    let ItemKind::PolylineTrace(t) = &item.kind else {
        return None;
    };
    let n = t.polyline.corner_count();
    if n < 2 {
        return None;
    }
    let close = |p: &Point| p.to_float().distance(via_center.to_float()) <= tolerance;
    if close(&t.first_corner()) {
        Some((t.polyline.corner(1).to_float().round(), true))
    } else if close(&t.last_corner()) {
        Some((t.polyline.corner(n - 2).to_float().round(), false))
    } else {
        None
    }
}

/// Tries to move `via_id` to a better location (Java:
/// `OptViaAlgo.opt_via_location`). Returns true when the via moved.
pub fn opt_via_location(board: &mut BasicBoard, via_id: ItemId, max_recursion: usize) -> bool {
    if max_recursion == 0 {
        return false;
    }
    let Some(item) = board.get_item(via_id).cloned() else {
        return false;
    };
    let ItemKind::Via(via) = &item.kind else {
        return false;
    };
    if item.base.is_user_fixed() || item.base.component_no != 0 {
        return false;
    }
    let contacts = board.get_normal_contacts(via_id);
    if contacts.len() == 1 {
        // plane/fanout via: exactly one trace contact — pull the via
        // along its stub toward the far end (Java:
        // opt_plane_or_fanout_via)
        return opt_single_contact_via(board, via_id, contacts[0]);
    }
    let Some((t1, t2)) = two_trace_contacts(board, via_id) else {
        return false;
    };
    let via_center = via.center;
    let padstack = via.padstack;
    let attach_allowed = via.attach_allowed;
    let net_nos = item.base.net_nos.clone();
    let cl_class = item.base.clearance_class;
    let tolerance = board
        .padstacks
        .get_by_no(padstack)
        .and_then(|p| p.get_shape(p.from_layer()))
        .map(|s| s.bounding_box().min_width() / 2.0 + 1.0)
        .unwrap_or(500.0);
    let Some((c1, end1)) = from_corner(board, t1, via_center, tolerance) else {
        return false;
    };
    let Some((c2, end2)) = from_corner(board, t2, via_center, tolerance) else {
        return false;
    };
    let (hw1, _layer1) = match &board.get_item(t1).unwrap().kind {
        ItemKind::PolylineTrace(t) => (t.half_width, t.layer),
        _ => return false,
    };
    let (hw2, _layer2) = match &board.get_item(t2).unwrap().kind {
        ItemKind::PolylineTrace(t) => (t.half_width, t.layer),
        _ => return false,
    };
    let len_before = c1.to_float().distance(via_center.to_float())
        + c2.to_float().distance(via_center.to_float());
    // candidate targets in Java's spirit: each adjacent corner and their
    // midpoint — the via slides toward the shorter configuration
    let mid = IntPoint::new((c1.x + c2.x) / 2, (c1.y + c2.y) / 2);
    for cand in [c1, c2, mid] {
        if cand == via_center {
            continue;
        }
        let len_after =
            c1.to_float().distance(cand.to_float()) + c2.to_float().distance(cand.to_float());
        if len_after + 2.0 * hw1.max(hw2) as f64 >= len_before {
            continue;
        }
        board.generate_snapshot();
        // detach the via-end stubs so the move has room
        let mut ok = shorten_trace_at(board, t1, end1, cand);
        let t2_now = if ok {
            // t1's replacement may have renumbered t2? ids are stable
            // (shorten replaces only its own trace)
            shorten_trace_at(board, t2, end2, cand)
        } else {
            false
        };
        ok = ok && t2_now;
        if ok {
            board.remove_item(via_id);
            ok = insert_forced_via(
                board,
                padstack,
                cand,
                &net_nos,
                cl_class,
                hw1.max(hw2),
                attach_allowed,
            )
            .is_some();
        }
        // both nets must remain connected AND the reconnected stubs must
        // keep clearance (the stub inserts are not shove-validated)
        let all_connected = ok
            && net_nos
                .iter()
                .all(|&n| board.net_is_completely_connected(n));
        let stubs_clear = all_connected && [t1, t2].iter().all(|_| true) && {
            let mut clear = true;
            'outer: for (id, item) in board.items() {
                if item.base.component_no != 0
                    || !item.base.net_nos.iter().any(|n| net_nos.contains(n))
                {
                    continue;
                }
                for (s, l) in item.tile_shapes(&board.padstacks) {
                    for oid in board.overlapping_items(&s.offset(10_000.0), Some(*l)) {
                        if oid == *id {
                            continue;
                        }
                        let Some(other) = board.get_item(oid) else {
                            continue;
                        };
                        if other.base.shares_net(&item.base) {
                            continue;
                        }
                        if let ItemKind::ObstacleArea(a) = &other.kind {
                            if a.is_conduction {
                                continue;
                            }
                        }
                        let cl = board.rules.clearance_matrix.get_value(
                            item.base.clearance_class,
                            other.base.clearance_class,
                            *l,
                            false,
                        ) as f64;
                        if other.tile_shapes(&board.padstacks).iter().any(|(os, ol)| {
                            ol == l && crate::drc::violates(s.euclidean_distance_to(os), cl)
                        }) {
                            clear = false;
                            break 'outer;
                        }
                    }
                }
            }
            clear
        };
        if stubs_clear {
            board.pop_snapshot();
            return true;
        }
        board.undo();
    }
    false
}

/// The plane/fanout branch (Java: `opt_plane_or_fanout_via`): a via
/// with a single trace contact slides to the trace's adjacent corner,
/// shortening the stub, when a forced via fits there and the net stays
/// connected.
fn opt_single_contact_via(board: &mut BasicBoard, via_id: ItemId, trace_id: ItemId) -> bool {
    let Some(item) = board.get_item(via_id).cloned() else {
        return false;
    };
    let ItemKind::Via(via) = &item.kind else {
        return false;
    };
    if item.base.is_user_fixed() || item.base.component_no != 0 {
        return false;
    }
    let Some(t) = board.get_item(trace_id).cloned() else {
        return false;
    };
    if t.base.is_user_fixed() || !matches!(t.kind, ItemKind::PolylineTrace(_)) {
        return false;
    }
    let via_center = via.center;
    let padstack = via.padstack;
    let attach_allowed = via.attach_allowed;
    let net_nos = item.base.net_nos.clone();
    let cl_class = item.base.clearance_class;
    let tolerance = board
        .padstacks
        .get_by_no(padstack)
        .and_then(|p| p.get_shape(p.from_layer()))
        .map(|s| s.bounding_box().min_width() / 2.0 + 1.0)
        .unwrap_or(500.0);
    let Some((cand, end)) = from_corner(board, trace_id, via_center, tolerance) else {
        return false;
    };
    if cand == via_center {
        return false;
    }
    let hw = match &t.kind {
        ItemKind::PolylineTrace(pt) => pt.half_width,
        _ => return false,
    };
    board.generate_snapshot();
    // when the via reaches the trace's far corner the stub degenerates:
    // remove it entirely — the via then contacts the far item directly
    // (Java deletes the emptied stub too)
    let ok = if shorten_trace_at(board, trace_id, end, cand) {
        true
    } else {
        board.remove_item(trace_id)
    };
    let ok = ok && {
        board.remove_item(via_id);
        insert_forced_via(
            board,
            padstack,
            cand,
            &net_nos,
            cl_class,
            hw,
            attach_allowed,
        )
        .is_some()
    };
    let connected = ok
        && net_nos
            .iter()
            .all(|&n| board.net_is_completely_connected(n));
    if connected {
        board.pop_snapshot();
        true
    } else {
        board.undo();
        false
    }
}

/// Replaces the via-end corner of a trace with `new_end` (the stub
/// follows the moved via). Returns false when the geometry degenerates.
fn shorten_trace_at(
    board: &mut BasicBoard,
    trace_id: ItemId,
    at_first_end: bool,
    new_end: IntPoint,
) -> bool {
    let Some(item) = board.get_item(trace_id).cloned() else {
        return false;
    };
    let ItemKind::PolylineTrace(t) = &item.kind else {
        return false;
    };
    let mut corners: Vec<IntPoint> = t
        .polyline
        .corner_approx_arr()
        .iter()
        .map(|c| c.round())
        .collect();
    if corners.len() < 2 {
        return false;
    }
    if at_first_end {
        corners[0] = new_end;
    } else {
        let n = corners.len();
        corners[n - 1] = new_end;
    }
    corners.dedup();
    if corners.len() < 2 {
        return false;
    }
    let polyline = Polyline::from_int_points(&corners);
    if polyline.is_empty() {
        return false;
    }
    let (layer, half_width) = (t.layer, t.half_width);
    let net_nos = item.base.net_nos.clone();
    let cl = item.base.clearance_class;
    board.remove_item(trace_id);
    crate::board::basic_board::set_birth_tag(3);
    board.insert_trace(polyline, layer, half_width, net_nos, cl);
    true
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::board::{Layer, LayerStructure};
    use crate::core::Padstacks;
    use crate::geometry::planar::{IntBox, TileShape};
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
    fn single_contact_via_pulls_along_its_stub() {
        let mut board = test_board();
        let pad = board.insert_via(1, IntPoint::new(0, 0), vec![1], 1, false);
        board.set_component_no(pad, 1);
        // stub from the pad to a dangling fanout via
        let via = board.insert_via(1, IntPoint::new(12000, 0), vec![1], 1, false);
        board.insert_trace(
            Polyline::from_int_points(&[IntPoint::new(0, 0), IntPoint::new(12000, 0)]),
            0,
            100,
            vec![1],
            1,
        );
        assert!(board.net_is_completely_connected(1));
        let moved = opt_via_location(&mut board, via, 3);
        assert!(moved, "the fanout via should pull toward the pad");
        assert!(board.net_is_completely_connected(1));
    }

    #[test]
    fn dogleg_via_slides_toward_the_corner() {
        let mut board = test_board();
        // pads at the ends
        let a = board.insert_via(1, IntPoint::new(0, 0), vec![1], 1, false);
        let b = board.insert_via(1, IntPoint::new(20000, 10000), vec![1], 1, false);
        board.set_component_no(a, 1);
        board.set_component_no(b, 1);
        // a dogleg: trace on L0 to (10000, 10000), via, trace on L1 to b —
        // the via sits off the straight path
        let via = board.insert_via(1, IntPoint::new(10000, 10000), vec![1], 1, false);
        board.insert_trace(
            Polyline::from_int_points(&[IntPoint::new(0, 0), IntPoint::new(10000, 10000)]),
            0,
            100,
            vec![1],
            1,
        );
        board.insert_trace(
            Polyline::from_int_points(&[IntPoint::new(10000, 10000), IntPoint::new(20000, 10000)]),
            1,
            100,
            vec![1],
            1,
        );
        assert!(board.net_is_completely_connected(1));
        let moved = opt_via_location(&mut board, via, 3);
        assert!(moved, "the dogleg via should find a shorter location");
        assert!(
            board.net_is_completely_connected(1),
            "net must stay connected"
        );
    }
}
