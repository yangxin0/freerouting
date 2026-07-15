//! Post-route optimization (the core of Java's `BatchOptRoute`): rip one
//! net's route items and reroute it against the otherwise-complete board,
//! keeping the result only when it improves. Improvement means: a
//! previously incomplete net completes, or the net stays complete with
//! fewer vias, or equal vias and shorter traces (Java compares via count
//! first, then length).

use crate::autoroute::batch::{route_net_with_ripup, BatchRequest};
use crate::board::basic_board::{BasicBoard, ItemId};
use crate::board::ItemKind;

/// The clearance violations touching one net's route items (local DRC:
/// the optimizer must never trade violations for length).
fn net_violations(board: &BasicBoard, net_no: i32) -> usize {
    let mut count = 0usize;
    for (id, item) in board.items() {
        if item.base.component_no != 0 || !item.base.contains_net(net_no) {
            continue;
        }
        for (s, l) in item.tile_shapes(&board.padstacks) {
            for oid in board.overlapping_items(&s.offset(10_000.0), Some(*l)) {
                if oid == *id {
                    continue;
                }
                let Some(other) = board.get_item(oid) else { continue };
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
                let check = s.offset(cl);
                if other.tile_shapes(&board.padstacks).iter().any(|(os, ol)| {
                    ol == l
                        && os.intersection(&check).dimension() >= 2
                        && s.euclidean_distance_to(os) < cl - 1.0
                }) {
                    count += 1;
                }
            }
        }
    }
    count
}

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
    optimize_nets_pass(board, request, &net_nos, time_limit)
}

/// [`optimize_route_pass`] over an explicit net slice (the unit of work
/// of the multithreaded optimizer).
pub fn optimize_nets_pass(
    board: &mut BasicBoard,
    request: &BatchRequest,
    net_nos: &[i32],
    time_limit: Option<&crate::datastructures::TimeLimit>,
) -> usize {
    let net_nos: Vec<i32> = net_nos.to_vec();
    let min_gain = 4.0 * request.trace_half_width as f64;
    let mut improved = 0usize;
    for net_no in net_nos {
        if time_limit.is_some_and(|t| t.limit_exceeded()) {
            break;
        }
        let was_complete = board.net_is_completely_connected(net_no);
        let (vias_before, len_before) = net_route_cost(board, net_no);
        let violations_before = net_violations(board, net_no);
        if was_complete && vias_before == 0 && len_before == 0.0 {
            continue; // nothing routed (single-pad net or pad-only)
        }
        board.generate_snapshot();
        rip_net_route_items(board, net_no);
        // reroute with a modest per-net budget; in-search ripup enabled
        // so the reroute may push others aside (their recovery is part
        // of the same transaction inside route_net_with_ripup)
        // recovery attempts on incomplete nets warrant a bigger budget
        // than improvement reroutes of already-complete nets
        let cap = if was_complete { 5_000 } else { 20_000 };
        let budget_ms = time_limit
            .map(|t| t.remaining_ms())
            .unwrap_or(u64::MAX)
            .min(cap);
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
            && net_violations(board, net_no) <= violations_before
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

/// The multithreaded optimizer (Java: `BatchOptimizerMultiThreaded`):
/// each round clones the board per worker, every worker optimizes its
/// slice of the nets in parallel, and the best-scoring result board is
/// adopted (greedy board update strategy). Requires `threads >= 1`.
pub fn optimize_route_multithreaded(
    board: &mut BasicBoard,
    request: &BatchRequest,
    threads: usize,
    time_limit: Option<&crate::datastructures::TimeLimit>,
) -> usize {
    let threads = threads.max(1);
    if threads == 1 {
        return optimize_route(board, request, time_limit);
    }
    let all_nets: Vec<i32> = (1..=board.rules.nets.max_net_no()).collect();
    let mut total = 0usize;
    loop {
        if time_limit.is_some_and(|t| t.limit_exceeded()) {
            break;
        }
        // partition the nets round-robin across the workers
        let slices: Vec<Vec<i32>> = (0..threads)
            .map(|t| {
                all_nets
                    .iter()
                    .copied()
                    .skip(t)
                    .step_by(threads)
                    .collect()
            })
            .collect();
        let results: Vec<(usize, f64, BasicBoard)> = std::thread::scope(|scope| {
            let handles: Vec<_> = slices
                .iter()
                .map(|slice| {
                    let mut clone = board.clone();
                    scope.spawn(move || {
                        let improved =
                            optimize_nets_pass(&mut clone, request, slice, time_limit);
                        let stats =
                            crate::scoring::BoardStatistics::collect(&clone);
                        let score = stats
                            .normalized_score(&crate::scoring::ScoringSettings::default());
                        (improved, score, clone)
                    })
                })
                .collect();
            handles
                .into_iter()
                .filter_map(|h| h.join().ok())
                .collect()
        });
        let baseline = crate::scoring::BoardStatistics::collect(board)
            .normalized_score(&crate::scoring::ScoringSettings::default());
        let best = results
            .into_iter()
            .filter(|(improved, score, _)| *improved > 0 && *score > baseline)
            .max_by(|a, b| a.1.partial_cmp(&b.1).unwrap_or(std::cmp::Ordering::Equal));
        match best {
            Some((improved, _, winner)) => {
                *board = winner;
                total += improved;
            }
            None => break,
        }
    }
    total
}

/// One via-optimization sweep (Java: OptViaAlgo in the optimizer
/// phase): every route via tries to slide to a shorter legal location.
pub fn optimize_vias(
    board: &mut BasicBoard,
    time_limit: Option<&crate::datastructures::TimeLimit>,
) -> usize {
    let vias: Vec<ItemId> = board
        .items()
        .filter(|(_, it)| {
            it.base.component_no == 0
                && it.base.net_count() > 0
                && matches!(it.kind, ItemKind::Via(_))
        })
        .map(|(id, _)| *id)
        .collect();
    let mut moved = 0usize;
    for via in vias {
        if time_limit.is_some_and(|t| t.limit_exceeded()) {
            break;
        }
        if crate::board::opt_via::opt_via_location(board, via, 3) {
            moved += 1;
        }
    }
    moved
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
        let improved = optimize_route_pass(board, request, time_limit)
            + optimize_vias(board, time_limit);
        total += improved;
        if improved == 0 {
            break;
        }
    }
    total
}
