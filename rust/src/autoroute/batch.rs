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
pub struct BatchRequest {
    pub trace_half_width: i32,
    pub clearance_class: usize,
    pub via_padstack: usize,
    pub via_cost: f64,
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
