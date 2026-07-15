//! Port of `BatchFanout.java` + `RoutingBoard.fanout()`: escape routing
//! for SMD pins. A fanout connects a single-layer pin to a via so other
//! layers become reachable; the maze runs in fanout mode, completing at
//! the first successful drill.

use crate::autoroute::batch::BatchRequest;
use crate::autoroute::maze_search::{maze_route_with_engine, MazeRouteRequest};
use crate::board::basic_board::{BasicBoard, ItemId};
use crate::board::ItemKind;

/// True if the pin needs a fanout: a single-layer connectable pad whose
/// connected set lies entirely on its own layer and whose net has more
/// to connect (Java: `RoutingBoard.fanout` preconditions).
fn needs_fanout(board: &BasicBoard, pin_id: ItemId) -> Option<i32> {
    let item = board.get_item(pin_id)?;
    if item.base.component_no == 0 || !item.is_connectable() {
        return None;
    }
    if item.base.net_count() != 1 {
        return None;
    }
    let net_no = *item.base.net_nos.first()?;
    let layers: Vec<usize> = item
        .tile_shapes(&board.padstacks)
        .iter()
        .map(|(_, l)| *l)
        .collect();
    let Some(&pin_layer) = layers.first() else {
        return None;
    };
    if layers.iter().any(|&l| l != pin_layer) {
        return None; // through-hole pin reaches every layer already
    }
    // the whole connected set must be stuck on the pin's layer
    let connected = board.get_connected_set(pin_id, net_no);
    for id in &connected {
        let Some(it) = board.get_item(*id) else { continue };
        if it
            .tile_shapes(&board.padstacks)
            .iter()
            .any(|(_, l)| *l != pin_layer)
        {
            return None;
        }
    }
    // and something else of the net must remain unconnected
    let has_unconnected = board.items().any(|(id, it)| {
        it.base.contains_net(net_no) && it.is_connectable() && !connected.contains(id)
    });
    if has_unconnected {
        Some(net_no)
    } else {
        None
    }
}

/// Fans out one pin: routes from the pin in fanout mode and inserts the
/// found escape (trace + via). Returns true when a via was placed.
pub fn fanout_pin(board: &mut BasicBoard, pin_id: ItemId, request: &BatchRequest) -> bool {
    let Some(net_no) = needs_fanout(board, pin_id) else {
        return false;
    };
    // Java (a7cc6e42): the pin's net-class via rule decides the fanout
    // via; nets without one fall back to the board-level via rules
    // (fanout.fallback_to_board_vias, default true). A pin is skipped
    // only when neither yields a via.
    let via_padstack = board
        .rules
        .via_padstack_for_net(net_no)
        .unwrap_or(request.via_padstack);
    if via_padstack == 0 {
        return false;
    }
    let maze_request = MazeRouteRequest {
        net_no,
        start_item: pin_id,
        dest_item: pin_id,
        start_items: vec![pin_id],
        dest_items: Vec::new(),
        trace_half_width: request.trace_half_width,
        clearance_class: request.clearance_class,
        via_padstack,
        via_cost: request.via_cost,
        // fanout escapes are local: a small budget keeps hopeless pins
        // cheap (Java bounds the whole stage with a timeout instead)
        max_expansions: 3_000,
        ripup_penalty: 0.0,
        deadline: request.deadline,
        is_fanout: true,
    };
    let mut engine = crate::autoroute::engine::AutorouteEngine::new_with_clearance(
        net_no,
        false,
        request.clearance_class,
        request.trace_half_width,
    );
    crate::board::basic_board::set_birth_tag(1);
    maze_route_with_engine(board, &mut engine, &maze_request).is_some()
}

/// Fans out every SMD pin that needs it, in passes until no pin
/// improves (Java: `BatchFanout.fanout_board`, outer pins first).
/// Java default `fanout.maxItems` (Integer.MAX_VALUE): a cap on the
/// pins one fanout run may process.
const FANOUT_MAX_ITEMS: usize = usize::MAX;

pub fn fanout_board(
    board: &mut BasicBoard,
    request: &BatchRequest,
    max_passes: usize,
    time_limit: Option<&crate::datastructures::TimeLimit>,
) -> usize {
    let center = {
        let bb = board.bounding_box();
        (
            (bb.ll.x as f64 + bb.ur.x as f64) / 2.0,
            (bb.ll.y as f64 + bb.ur.y as f64) / 2.0,
        )
    };
    let mut total = 0usize;
    for _pass in 0..max_passes.max(1) {
        let mut pins: Vec<(f64, ItemId)> = board
            .items()
            .filter(|(_, it)| {
                it.base.component_no != 0
                    && it.is_connectable()
                    && matches!(it.kind, ItemKind::Via(_))
            })
            .map(|(id, it)| {
                let bb = it.bounding_box(&board.padstacks);
                let (cx, cy) = (
                    (bb.ll.x as f64 + bb.ur.x as f64) / 2.0,
                    (bb.ll.y as f64 + bb.ur.y as f64) / 2.0,
                );
                let d = (cx - center.0).hypot(cy - center.1);
                (d, *id)
            })
            .collect();
        // outer pins first (Java: pinSortingOrder "outer_first")
        pins.sort_by(|a, b| b.0.partial_cmp(&a.0).unwrap_or(std::cmp::Ordering::Equal));
        let mut fanned = 0usize;
        for (_, pin) in pins {
            if time_limit.is_some_and(|t| t.limit_exceeded()) {
                return total + fanned;
            }
            // Java (27e700bc): fanout.maxItems caps the processed pins
            if total + fanned >= FANOUT_MAX_ITEMS {
                return total + fanned;
            }
            if fanout_pin(board, pin, request) {
                fanned += 1;
            }
        }
        total += fanned;
        if fanned == 0 {
            break;
        }
    }
    total
}
