//! Port of the core loop of `BatchAutorouter.java` (single pass, no
//! ripup escalation yet): route the incomplete connections of every net
//! until each net forms one connected set.
//!
//! Incompletes are computed from the board connectivity: the connected
//! components of a net's connectable items; the closest pair of items
//! between two components becomes the next connection to route.

use crate::autoroute::maze_search::{maze_route, MazeRouteRequest};
use crate::board::basic_board::{BasicBoard, ItemId};

#[derive(Debug, Default, PartialEq, Eq)]
pub struct BatchResult {
    pub routed_connections: usize,
    pub failed_connections: usize,
}

/// Parameters for a batch pass.
#[derive(Debug, Clone, Copy)]
pub struct BatchRequest {
    pub trace_half_width: i32,
    pub clearance_class: usize,
    pub via_padstack: usize,
    pub via_cost: f64,
    /// Expansion budget per connection (see
    /// `MazeRouteRequest::max_expansions`).
    pub max_expansions: usize,
}

/// The connected components of the connectable items of `net_no`.
fn net_components(board: &BasicBoard, net_no: i32) -> Vec<Vec<ItemId>> {
    let net_items: Vec<ItemId> = board
        .items()
        .filter(|(_, item)| item.base.contains_net(net_no) && item.is_connectable())
        .map(|(id, _)| *id)
        .collect();
    let mut components: Vec<Vec<ItemId>> = Vec::new();
    let mut assigned: Vec<ItemId> = Vec::new();
    for &item in &net_items {
        if assigned.contains(&item) {
            continue;
        }
        let component = board.get_connected_set(item, net_no);
        assigned.extend(component.iter().copied());
        components.push(component);
    }
    components
}

/// The items of a component usable as connection endpoints: drill items
/// (pads/vias) preferred, because a trace contact requires endpoint
/// equality (trace splitting at junctions is not yet ported).
fn endpoint_candidates(board: &BasicBoard, component: &[ItemId]) -> Vec<ItemId> {
    let drills: Vec<ItemId> = component
        .iter()
        .copied()
        .filter(|id| {
            matches!(
                board.get_item(*id).map(|i| &i.kind),
                Some(crate::board::ItemKind::Via(_))
            )
        })
        .collect();
    if drills.is_empty() {
        component.to_vec()
    } else {
        drills
    }
}

/// The closest pair of items between two components (by bounding-box
/// center distance).
fn closest_pair(
    board: &BasicBoard,
    component_a: &[ItemId],
    component_b: &[ItemId],
) -> Option<(ItemId, ItemId)> {
    let center = |id: ItemId| {
        board.get_item(id).map(|item| {
            let bb = item.bounding_box(&board.padstacks);
            (
                (bb.ll.x as f64 + bb.ur.x as f64) / 2.0,
                (bb.ll.y as f64 + bb.ur.y as f64) / 2.0,
            )
        })
    };
    let mut best: Option<(f64, ItemId, ItemId)> = None;
    for &a in component_a {
        let Some((ax, ay)) = center(a) else { continue };
        for &b in component_b {
            let Some((bx, by)) = center(b) else { continue };
            let dist = (ax - bx).hypot(ay - by);
            if best.is_none_or(|(d, _, _)| dist < d) {
                best = Some((dist, a, b));
            }
        }
    }
    best.map(|(_, a, b)| (a, b))
}

/// Routes all incomplete connections of `net_no`. Returns (routed,
/// failed) counts; stops trying a net after the first failed connection.
pub fn route_net(board: &mut BasicBoard, net_no: i32, request: &BatchRequest) -> BatchResult {
    let mut result = BatchResult::default();
    let mut prev_component_count = usize::MAX;
    loop {
        let components = net_components(board, net_no);
        if components.len() <= 1 {
            break;
        }
        if components.len() >= prev_component_count {
            // a routed connection did not reduce the component count:
            // treat as failure to avoid looping forever
            result.failed_connections += 1;
            break;
        }
        prev_component_count = components.len();
        // route between the first component and the component closest
        // to it
        let first_candidates = endpoint_candidates(board, &components[0]);
        let mut best: Option<(ItemId, ItemId)> = None;
        for other in &components[1..] {
            let other_candidates = endpoint_candidates(board, other);
            if let Some(pair) = closest_pair(board, &first_candidates, &other_candidates) {
                let dist_of = |p: &(ItemId, ItemId)| {
                    let bb_a = board.get_item(p.0).unwrap().bounding_box(&board.padstacks);
                    let bb_b = board.get_item(p.1).unwrap().bounding_box(&board.padstacks);
                    bb_a.weighted_distance(bb_b, 1.0, 1.0)
                };
                if best.is_none() || dist_of(&pair) < dist_of(best.as_ref().unwrap()) {
                    best = Some(pair);
                }
            }
        }
        let Some((start, dest)) = best else {
            break;
        };
        let maze_request = MazeRouteRequest {
            net_no,
            start_item: start,
            dest_item: dest,
            trace_half_width: request.trace_half_width,
            clearance_class: request.clearance_class,
            via_padstack: request.via_padstack,
            via_cost: request.via_cost,
            max_expansions: request.max_expansions,
        };
        if maze_route(board, &maze_request).is_some() {
            result.routed_connections += 1;
        } else {
            result.failed_connections += 1;
            break; // no ripup yet: give up on this net
        }
    }
    result
}

/// Routes all nets with incomplete connections in ascending net-number
/// order (a single batch pass).
pub fn batch_route(board: &mut BasicBoard, request: &BatchRequest) -> BatchResult {
    let net_nos: Vec<i32> = (1..=board.rules.nets.max_net_no()).collect();
    let mut result = BatchResult::default();
    for net_no in net_nos {
        let net_result = route_net(board, net_no, request);
        result.routed_connections += net_result.routed_connections;
        result.failed_connections += net_result.failed_connections;
    }
    result
}

/// Removes up to `max_rips` routable foreign route items (never
/// component pins or fixed items) blocking the straight corridor between
/// the two items of the failed connection, returning the affected nets
/// (a simplified stand-in for MazeSearchAlgo's per-room ripup costs).
fn rip_blocking_items(
    board: &mut BasicBoard,
    net_no: i32,
    start: ItemId,
    dest: ItemId,
    corridor_half_width: i32,
    max_rips: usize,
) -> Vec<i32> {
    use crate::geometry::planar::{IntPoint, Polyline, TileShape};
    let center = |id: ItemId| -> Option<IntPoint> {
        board.get_item(id).map(|item| {
            let bb = item.bounding_box(&board.padstacks);
            IntPoint::new((bb.ll.x + bb.ur.x) / 2, (bb.ll.y + bb.ur.y) / 2)
        })
    };
    let (Some(a), Some(b)) = (center(start), center(dest)) else {
        return Vec::new();
    };
    let polyline = Polyline::from_two_points(a, b);
    let corridor: TileShape = if polyline.is_empty() {
        TileShape::Box(
            crate::geometry::planar::IntBox::new(a, a).offset(corridor_half_width as f64),
        )
    } else {
        match polyline.offset_shape(corridor_half_width, 0) {
            Some(s) => s,
            None => return Vec::new(),
        }
    };
    let mut candidates: Vec<ItemId> = board
        .overlapping_items(&corridor, None)
        .into_iter()
        .filter(|id| {
            board.get_item(*id).is_some_and(|item| {
                !item.base.contains_net(net_no)
                    && item.base.component_no == 0
                    && item.base.net_count() > 0
                    && item.is_routable()
            })
        })
        .collect();
    candidates.truncate(max_rips);
    let mut ripped_nets: Vec<i32> = Vec::new();
    for id in candidates {
        if let Some(item) = board.get_item(id) {
            ripped_nets.extend(item.base.net_nos.iter().copied());
        }
        board.remove_item(id);
    }
    ripped_nets.sort();
    ripped_nets.dedup();
    ripped_nets
}

/// Like [`route_net`], but on failure rips blocking foreign route items
/// along the missing connection and retries once.
pub fn route_net_with_ripup(
    board: &mut BasicBoard,
    net_no: i32,
    request: &BatchRequest,
    max_rips: usize,
) -> BatchResult {
    let mut result = route_net(board, net_no, request);
    if result.failed_connections == 0 || max_rips == 0 {
        return result;
    }
    // find the still-missing connection pair and rip its corridor
    let components = net_components(board, net_no);
    if components.len() <= 1 {
        return result;
    }
    let first_candidates = endpoint_candidates(board, &components[0]);
    let mut pair: Option<(ItemId, ItemId)> = None;
    for other in &components[1..] {
        let other_candidates = endpoint_candidates(board, other);
        if let Some(p) = closest_pair(board, &first_candidates, &other_candidates) {
            pair = Some(p);
            break;
        }
    }
    let Some((a, b)) = pair else {
        return result;
    };
    // Transactional ripup: commit only if this net AND every ripped net
    // end up completely connected, otherwise restore the previous state
    // (Java instead proves the benefit inside the maze search by paying
    // ripup costs).
    board.generate_snapshot();
    let corridor_half_width = 2 * (request.trace_half_width + 400);
    let ripped_nets = rip_blocking_items(board, net_no, a, b, corridor_half_width, max_rips);
    if ripped_nets.is_empty() {
        board.pop_snapshot();
        return result;
    }
    let retry = route_net(board, net_no, request);
    let mut success =
        retry.failed_connections == 0 && board.net_is_completely_connected(net_no);
    if success {
        // reroute the victims immediately; all must recover
        for &ripped in &ripped_nets {
            let r = route_net(board, ripped, request);
            if r.failed_connections > 0 || !board.net_is_completely_connected(ripped) {
                success = false;
                break;
            }
        }
    }
    if success {
        result.routed_connections += retry.routed_connections;
        result.failed_connections = 0;
        board.pop_snapshot();
    } else {
        board.undo();
    }
    result
}

/// The half perimeter of the bounding box of a net's connectable items,
/// used to order nets shortest-first.
fn net_extent(board: &BasicBoard, net_no: i32) -> i64 {
    let mut bb = crate::geometry::planar::IntBox::EMPTY;
    for (_, item) in board.items() {
        if item.base.contains_net(net_no) && item.is_connectable() {
            bb = bb.union(item.bounding_box(&board.padstacks));
        }
    }
    if bb.is_empty() {
        0
    } else {
        bb.width() as i64 + bb.height() as i64
    }
}

/// Routes all nets over several passes: pass 1 in shortest-net-first
/// order, later passes retrying the incomplete nets with a doubled
/// expansion budget each time (a simplified stand-in for Java's
/// ripup-cost pass escalation). Returns the result of the final state.
pub fn batch_route_passes(
    board: &mut BasicBoard,
    request: &BatchRequest,
    passes: usize,
) -> BatchResult {
    let mut net_nos: Vec<i32> = (1..=board.rules.nets.max_net_no()).collect();
    net_nos.sort_by_key(|&n| net_extent(board, n));

    let mut total = BatchResult::default();
    let mut budget = request.max_expansions;
    for pass in 0..passes.max(1) {
        let pass_request = BatchRequest {
            max_expansions: budget,
            ..*request
        };
        let mut failed_this_pass = 0usize;
        // ripup is allowed from the second pass on, with growing allowance
        let max_rips = if pass == 0 { 0 } else { 2 * pass };
        for &net_no in &net_nos {
            if board.net_is_completely_connected(net_no) {
                continue;
            }
            let result = route_net_with_ripup(board, net_no, &pass_request, max_rips);
            total.routed_connections += result.routed_connections;
            failed_this_pass += result.failed_connections;
        }
        if failed_this_pass == 0 {
            break;
        }
        if pass + 1 == passes {
            total.failed_connections += failed_this_pass;
        }
        budget = budget.saturating_mul(2);
    }
    total
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::board::{Layer, LayerStructure};
    use crate::core::Padstacks;
    use crate::geometry::planar::{IntBox, IntPoint, TileShape};
    use crate::rules::{BoardRules, ClearanceMatrix};

    fn test_board(net_count: usize) -> BasicBoard {
        let stack = LayerStructure::new(vec![
            Layer::new("F.Cu", true),
            Layer::new("B.Cu", true),
        ]);
        let matrix = ClearanceMatrix::get_default_instance(stack.clone(), 200);
        let mut rules = BoardRules::new(stack.clone(), matrix);
        rules.get_default_net_class();
        for i in 0..net_count {
            rules.nets.add(format!("net_{i}"), 1, false);
        }
        let mut padstacks = Padstacks::new(2);
        padstacks.add_shape_on_layers(
            TileShape::Box(IntBox::from_coords(-400, -400, 400, 400)),
            0,
            1,
        );
        BasicBoard::new(stack, rules, padstacks)
    }

    fn request() -> BatchRequest {
        BatchRequest {
            trace_half_width: 100,
            clearance_class: 1,
            via_padstack: 1,
            via_cost: 5000.0,
            max_expansions: 100_000,
        }
    }

    #[test]
    fn routes_multiple_nets() {
        let mut board = test_board(3);
        // three horizontal nets stacked vertically, 2 pads each
        for i in 0..3_i32 {
            let y = i * 4000;
            board.insert_via(1, IntPoint::new(0, y), vec![i + 1], 1, false);
            board.insert_via(1, IntPoint::new(12000, y), vec![i + 1], 1, false);
        }
        let result = batch_route(&mut board, &request());
        assert_eq!(result.failed_connections, 0);
        assert_eq!(result.routed_connections, 3);
        for net in 1..=3 {
            assert!(
                board.net_is_completely_connected(net),
                "net {net} incomplete"
            );
        }
    }

    #[test]
    fn routes_net_with_three_pads() {
        let mut board = test_board(1);
        board.insert_via(1, IntPoint::new(0, 0), vec![1], 1, false);
        board.insert_via(1, IntPoint::new(8000, 0), vec![1], 1, false);
        board.insert_via(1, IntPoint::new(4000, 6000), vec![1], 1, false);
        let result = batch_route(&mut board, &request());
        assert_eq!(result.failed_connections, 0);
        // three pads need two connections
        assert_eq!(result.routed_connections, 2);
        assert!(board.net_is_completely_connected(1));
    }

    #[test]
    fn crossing_nets_use_both_layers() {
        let mut board = test_board(2);
        // net 1 horizontal, net 2 vertical, crossing in the middle
        board.insert_via(1, IntPoint::new(-6000, 0), vec![1], 1, false);
        board.insert_via(1, IntPoint::new(6000, 0), vec![1], 1, false);
        board.insert_via(1, IntPoint::new(0, -6000), vec![2], 1, false);
        board.insert_via(1, IntPoint::new(0, 6000), vec![2], 1, false);
        let result = batch_route(&mut board, &request());
        assert_eq!(result.failed_connections, 0);
        assert!(board.net_is_completely_connected(1));
        assert!(board.net_is_completely_connected(2));
        // and the nets are not shorted: no trace carries both nets
        for (_, item) in board.items() {
            assert!(item.base.net_nos.len() <= 1);
        }
    }

    #[test]
    fn ripup_frees_a_contended_gap() {
        use crate::geometry::planar::{PolygonShape, PolylineArea};
        // single signal layer: no via escape
        let stack = LayerStructure::new(vec![Layer::new("F.Cu", true)]);
        let matrix = ClearanceMatrix::get_default_instance(stack.clone(), 200);
        let mut rules = BoardRules::new(stack.clone(), matrix);
        rules.get_default_net_class();
        rules.nets.add("net1", 1, false);
        rules.nets.add("net2", 1, false);
        let mut padstacks = Padstacks::new(1);
        padstacks.add_shape_on_layers(
            TileShape::Box(IntBox::from_coords(-300, -300, 300, 300)),
            0,
            0,
        );
        let mut board = BasicBoard::new(stack, rules, padstacks);

        // a wall at x = 5000 with a gap around y = 0 (tall enough for two
        // traces side by side)
        for (lly, ury) in [(-40000, -2000), (2000, 40000)] {
            let wall = PolylineArea::new(
                PolygonShape::from_int_points(&[
                    IntPoint::new(4800, lly),
                    IntPoint::new(5200, lly),
                    IntPoint::new(5200, ury),
                    IntPoint::new(4800, ury),
                ]),
                vec![],
            );
            board.insert_area(wall, 0, "wall", vec![], 1, false);
        }
        // net 1 spans the gap; its pads are protected component pins
        let n1a = board.insert_via(1, IntPoint::new(0, 0), vec![1], 1, false);
        let n1b = board.insert_via(1, IntPoint::new(10000, 0), vec![1], 1, false);
        // net 2 also must pass the gap
        let n2a = board.insert_via(1, IntPoint::new(1000, -1200), vec![2], 1, false);
        let n2b = board.insert_via(1, IntPoint::new(9000, -1200), vec![2], 1, false);
        for id in [n1a, n1b, n2a, n2b] {
            board.set_component_no(id, 1);
        }

        let request = BatchRequest {
            trace_half_width: 100,
            clearance_class: 1,
            via_padstack: 1,
            via_cost: 5000.0,
            max_expansions: 30_000,
        };
        // route net 1 first: it takes the gap
        let r1 = route_net(&mut board, 1, &request);
        assert_eq!(r1.failed_connections, 0);
        assert!(board.net_is_completely_connected(1));

        // multi-pass with ripup completes both nets
        let result = batch_route_passes(&mut board, &request, 4);
        assert_eq!(
            result.failed_connections, 0,
            "ripup passes did not complete the board"
        );
        assert!(board.net_is_completely_connected(1));
        assert!(board.net_is_completely_connected(2));
    }

    #[test]
    fn reports_failures_for_unroutable_net() {
        let mut board = test_board(1);
        board.insert_via(1, IntPoint::new(0, 0), vec![1], 1, false);
        board.insert_via(1, IntPoint::new(9000, 0), vec![1], 1, false);
        for layer in 0..2 {
            for (from, to) in [
                ((-2000, -2000), (2000, -2000)),
                ((2000, -2000), (2000, 2000)),
                ((2000, 2000), (-2000, 2000)),
                ((-2000, 2000), (-2000, -2000)),
            ] {
                board.insert_trace(
                    crate::geometry::planar::Polyline::from_int_points(&[
                        IntPoint::new(from.0, from.1),
                        IntPoint::new(to.0, to.1),
                    ]),
                    layer,
                    300,
                    vec![9],
                    1,
                );
            }
        }
        let result = batch_route(&mut board, &request());
        assert_eq!(result.routed_connections, 0);
        assert_eq!(result.failed_connections, 1);
        assert!(!board.net_is_completely_connected(1));
    }
}
