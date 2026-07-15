//! Post-route optimization (the core of Java's `BatchOptRoute`): rip one
//! net's route items and reroute it against the otherwise-complete board,
//! keeping the result only when it improves. Improvement means: a
//! previously incomplete net completes, or the net stays complete with
//! fewer vias, or equal vias and shorter traces (Java compares via count
//! first, then length).

use crate::autoroute::batch::{route_net_with_ripup, BatchRequest};
use crate::board::basic_board::{BasicBoard, ItemId};
use crate::board::ItemKind;

/// The via count and trace length of one net's route items.
fn net_route_cost(board: &BasicBoard, net_no: i32) -> (usize, f64) {
    let mut vias = 0usize;
    let mut length = 0.0f64;
    for (_, item) in board.items() {
        if item.base.component_no != 0 || !item.base.contains_net(net_no) {
            continue;
        }
        match &item.kind {
            ItemKind::Via(_) => vias += 1,
            ItemKind::PolylineTrace(t) => length += t.get_length(),
            _ => {}
        }
    }
    (vias, length)
}

fn rip_net_route_items(board: &mut BasicBoard, net_no: i32) {
    let ids: Vec<ItemId> = board
        .items()
        .filter(|(_, it)| {
            it.base.component_no == 0
                && it.base.contains_net(net_no)
                && it.is_routable()
        })
        .map(|(id, _)| *id)
        .collect();
    for id in ids {
        board.remove_item(id);
    }
}

/// One optimization pass over all nets. Returns the number of improved
/// nets. Transactional per net: kept only when the net completes AND
/// (it was incomplete, or the via count drops, or the via count holds
/// and the length shrinks by more than `min_gain`).
pub fn optimize_route_pass(
    board: &mut BasicBoard,
    request: &BatchRequest,
    time_limit: Option<&crate::datastructures::TimeLimit>,
) -> usize {
    let net_nos: Vec<i32> = (1..=board.rules.nets.max_net_no()).collect();
    let min_gain = 4.0 * request.trace_half_width as f64;
    let mut improved = 0usize;
    for net_no in net_nos {
        if time_limit.is_some_and(|t| t.limit_exceeded()) {
            break;
        }
        let was_complete = board.net_is_completely_connected(net_no);
        let (vias_before, len_before) = net_route_cost(board, net_no);
        if was_complete && vias_before == 0 && len_before == 0.0 {
            continue; // nothing routed (single-pad net or pad-only)
        }
        board.generate_snapshot();
        rip_net_route_items(board, net_no);
        // reroute with a modest per-net budget; in-search ripup enabled
        // so the reroute may push others aside (their recovery is part
        // of the same transaction inside route_net_with_ripup)
        let budget_ms = time_limit
            .map(|t| t.remaining_ms())
            .unwrap_or(u64::MAX)
            .min(5_000);
        let net_request = BatchRequest {
            deadline: Some(crate::datastructures::TimeLimit::new(budget_ms)),
            ..*request
        };
        crate::board::basic_board::set_birth_tag(1);
        if was_complete {
            let _ = crate::autoroute::batch::route_net(board, net_no, &net_request);
        } else {
            // recovery attempt: in-search ripup with a strong penalty
            // (the plain reroute already failed during routing)
            let penalty = request.via_cost.max(20_000.0) * 2.0;
            let _ = route_net_with_ripup(board, net_no, &net_request, penalty);
        }
        let complete_now = board.net_is_completely_connected(net_no);
        let (vias_after, len_after) = net_route_cost(board, net_no);
        let keep = complete_now
            && (!was_complete
                || vias_after < vias_before
                || (vias_after == vias_before && len_after + min_gain < len_before));
        if keep {
            board.pop_snapshot();
            improved += 1;
        } else {
            board.undo();
        }
    }
    improved
}

/// Runs optimization passes until no net improves or time runs out.
/// Returns the total number of improvements.
pub fn optimize_route(
    board: &mut BasicBoard,
    request: &BatchRequest,
    time_limit: Option<&crate::datastructures::TimeLimit>,
) -> usize {
    let mut total = 0usize;
    loop {
        if time_limit.is_some_and(|t| t.limit_exceeded()) {
            break;
        }
        let improved = optimize_route_pass(board, request, time_limit);
        total += improved;
        if improved == 0 {
            break;
        }
    }
    total
}
