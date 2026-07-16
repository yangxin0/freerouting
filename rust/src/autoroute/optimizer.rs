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
            let search_radius = board.rules.clearance_matrix.max_value(*l).max(0) as f64;
            for oid in board.overlapping_items(&s.offset(search_radius), Some(*l)) {
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
                    if a.via_only && !matches!(item.kind, ItemKind::Via(_)) {
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

/// The completeness of every net (1-based; index 0 unused), so acceptance can
/// verify that no previously-complete net was broken by the reroute. A bare
/// count of incomplete nets is not enough: a reroute that completes the target
/// while breaking one previously-complete victim leaves the count unchanged and
/// would be wrongly accepted (the same is true of an incomplete-connection
/// count for a symmetric one-for-one swap). Recording the actual set catches it.
fn complete_net_set(board: &BasicBoard) -> Vec<bool> {
    let max = board.rules.nets.max_net_no();
    let mut v = vec![false; (max + 1) as usize];
    for n in 1..=max {
        v[n as usize] = board.net_is_completely_connected(n);
    }
    v
}

fn rip_net_route_items(board: &mut BasicBoard, net_no: i32) {
    let ids: Vec<ItemId> = board
        .items()
        .filter(|(_, it)| {
            it.base.component_no == 0 && it.base.contains_net(net_no) && it.is_routable()
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
    // Java (BatchOptimizer): a pass ends early after too many
    // consecutive non-improving items; the streak resets on improvement
    // (settings.optimizer.maxConsecutiveFailures, default 50)
    let mut consecutive_failures = 0usize;
    // Board-wide completeness set, the primary acceptance gate. Cached across
    // net-steps: a rejected step restores the board via undo, so the set is
    // unchanged and can be reused; only an accepted step (which mutates the
    // board) refreshes it. This keeps the board-wide scan roughly once per
    // acceptance rather than once per net.
    let mut complete_before: Option<Vec<bool>> = None;
    for net_no in net_nos {
        if time_limit.is_some_and(|t| t.limit_exceeded()) {
            break;
        }
        if consecutive_failures >= MAX_CONSECUTIVE_FAILURES {
            break;
        }
        if complete_before.is_none() {
            complete_before = Some(complete_net_set(board));
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
        // Reroute with the net's own class rules (width, clearance class, via
        // padstack), not the base request, so the optimizer does not relay
        // traces under the wrong clearance.
        let net_request = BatchRequest {
            deadline: Some(crate::datastructures::TimeLimit::new(budget_ms)),
            ..crate::autoroute::batch::request_for_net(board, net_no, request)
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
        let local_keep = complete_now
            && net_violations(board, net_no) <= violations_before
            && (!was_complete
                || vias_after < vias_before
                || (vias_after == vias_before && len_after + min_gain < len_before));
        // Board-wide gate (Java `ItemRouteResult.improved`): never accept a
        // reroute that breaks a previously-complete net, even if the target net
        // itself improved. Checking the completeness SET (not just a count)
        // catches the one-for-one swap where the target completes while a victim
        // breaks. Only scanned when the local condition already holds, so
        // rejects stay cheap; the scan early-exits on the first regression.
        let before = complete_before.as_ref().unwrap();
        let broke_a_net = local_keep
            && (1..=board.rules.nets.max_net_no()).any(|m| {
                m != net_no && before[m as usize] && !board.net_is_completely_connected(m)
            });
        let keep = local_keep && !broke_a_net;
        if keep {
            board.pop_snapshot();
            // board changed; refresh the completeness baseline
            complete_before = Some(complete_net_set(board));
            improved += 1;
            consecutive_failures = 0;
        } else {
            board.undo();
            // board restored to its pre-step state; cached set still valid
            consecutive_failures += 1;
        }
    }
    improved
}

/// Java default `optimizer.maxConsecutiveFailures`: an optimization pass
/// exits after this many non-improving items in a row.
const MAX_CONSECUTIVE_FAILURES: usize = 50;

/// Java default `optimizer.optimizationImprovementThreshold` (1%): a
/// pass whose relative score gain falls below this stops the optimizer.
const IMPROVEMENT_THRESHOLD: f64 = 0.01;

fn board_score(board: &BasicBoard) -> f64 {
    crate::scoring::BoardStatistics::collect(board)
        .normalized_score(&crate::scoring::ScoringSettings::default())
}

/// Java (BatchOptimizer.runBatchLoop): stop before a pass when the score
/// is already so close to 1000 that the remaining potential improvement
/// is below the threshold.
fn score_near_maximum(score: f64) -> bool {
    score * (1.0 + IMPROVEMENT_THRESHOLD) >= 1000.0
}

/// How the multithreaded optimizer publishes task results to the master
/// board (Java: `BoardUpdateStrategy`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BoardUpdateStrategy {
    /// Only the pass's single best improvement is adopted, at pass end.
    GlobalOptimal,
    /// Every winning task replaces the master board immediately; later
    /// tasks clone the updated board.
    Greedy,
    /// Alternates between the two per pass, `ratio.0` global-optimal
    /// passes then `ratio.1` greedy passes (Java: hybridRatio "1:1").
    Hybrid,
}

/// The order nets are handed to the optimizer tasks (Java:
/// `ItemSelectionStrategy`). GlobalOptimal passes always run Sequential,
/// like Java.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ItemSelectionStrategy {
    Sequential,
    /// Deterministically shuffled per pass (xorshift seeded by pass no).
    Random,
    /// Nets whose previous-pass results were best come first (Java sorts
    /// prior `ItemRouteResult`s ascending), unseen nets after them.
    Prioritized,
}

/// One net's reroute outcome, ordered like Java's `ItemRouteResult`
/// `compareTo`: fewer incompletes, then fewer vias, then shorter.
#[derive(Debug, Clone, Copy)]
struct NetRouteResult {
    net_no: i32,
    incomplete_after: usize,
    vias_after: usize,
    len_after: f64,
}

impl NetRouteResult {
    fn key(&self) -> (usize, usize, f64) {
        (self.incomplete_after, self.vias_after, self.len_after)
    }
    fn improved_over(&self, other: &NetRouteResult) -> bool {
        self.key()
            .partial_cmp(&other.key())
            .is_some_and(|o| o == std::cmp::Ordering::Less)
    }
}

/// The multithreaded optimizer (Java: `BatchOptimizerMultiThreaded`)
/// with Java's defaults: GREEDY board updates, PRIORITIZED selection.
pub fn optimize_route_multithreaded(
    board: &mut BasicBoard,
    request: &BatchRequest,
    threads: usize,
    time_limit: Option<&crate::datastructures::TimeLimit>,
) -> usize {
    optimize_route_multithreaded_with_strategy(
        board,
        request,
        threads,
        time_limit,
        BoardUpdateStrategy::Greedy,
        ItemSelectionStrategy::Prioritized,
        (1, 1),
    )
}

/// The multithreaded optimizer (Java: `BatchOptimizerMultiThreaded`):
/// each pass spawns one reroute task per net; every task clones the
/// master board, reroutes its net, and reports a [`NetRouteResult`].
/// GREEDY publishes each winning board immediately, GLOBAL_OPTIMAL only
/// the pass's best at pass end, HYBRID alternates. Passes repeat until
/// none improves or time runs out. Requires `threads >= 1`.
pub fn optimize_route_multithreaded_with_strategy(
    board: &mut BasicBoard,
    request: &BatchRequest,
    threads: usize,
    time_limit: Option<&crate::datastructures::TimeLimit>,
    strategy: BoardUpdateStrategy,
    selection: ItemSelectionStrategy,
    hybrid_ratio: (usize, usize),
) -> usize {
    let threads = threads.max(1);
    if threads == 1 {
        return optimize_route(board, request, time_limit);
    }
    let all_nets: Vec<i32> = (1..=board.rules.nets.max_net_no()).collect();
    // the strategy sequence a HYBRID run cycles through per pass
    let hybrid_list: Vec<BoardUpdateStrategy> = {
        let (optimal, greedy) = (hybrid_ratio.0.max(1), hybrid_ratio.1.max(1));
        std::iter::repeat_n(BoardUpdateStrategy::GlobalOptimal, optimal)
            .chain(std::iter::repeat_n(BoardUpdateStrategy::Greedy, greedy))
            .collect()
    };
    let mut prior_results: std::collections::HashMap<i32, NetRouteResult> =
        std::collections::HashMap::new();
    let mut total = 0usize;
    let mut pass_no = 0usize;
    loop {
        if time_limit.is_some_and(|t| t.limit_exceeded()) {
            break;
        }
        let score_before = board_score(board);
        if score_near_maximum(score_before) {
            break;
        }
        let pass_strategy = match strategy {
            BoardUpdateStrategy::Hybrid => hybrid_list[pass_no % hybrid_list.len()],
            s => s,
        };
        // GLOBAL_OPTIMAL forces sequential selection, like Java
        let pass_selection = if pass_strategy == BoardUpdateStrategy::GlobalOptimal {
            ItemSelectionStrategy::Sequential
        } else {
            selection
        };
        let mut order = all_nets.clone();
        match pass_selection {
            ItemSelectionStrategy::Sequential => {}
            ItemSelectionStrategy::Random => {
                // deterministic xorshift shuffle (the crate has no rng)
                let mut state = 0x9e3779b9u64 ^ (pass_no as u64 + 1);
                for i in (1..order.len()).rev() {
                    state ^= state << 13;
                    state ^= state >> 7;
                    state ^= state << 17;
                    order.swap(i, (state % (i as u64 + 1)) as usize);
                }
            }
            ItemSelectionStrategy::Prioritized => {
                let mut seen: Vec<NetRouteResult> = Vec::new();
                let mut unseen: Vec<i32> = Vec::new();
                for &n in &all_nets {
                    match prior_results.get(&n) {
                        Some(r) => seen.push(*r),
                        None => unseen.push(n),
                    }
                }
                seen.sort_by(|a, b| {
                    a.key()
                        .partial_cmp(&b.key())
                        .unwrap_or(std::cmp::Ordering::Equal)
                });
                order = seen.iter().map(|r| r.net_no).collect();
                order.extend(unseen);
            }
        }
        prior_results.clear();

        struct Shared {
            master: BasicBoard,
            next: usize,
            best: Option<(NetRouteResult, BasicBoard)>,
            results: Vec<NetRouteResult>,
            adopted: usize,
        }
        let shared = std::sync::Mutex::new(Shared {
            master: board.clone(),
            next: 0,
            best: None,
            results: Vec::new(),
            adopted: 0,
        });
        std::thread::scope(|scope| {
            for _ in 0..threads.min(order.len()) {
                scope.spawn(|| loop {
                    if time_limit.is_some_and(|t| t.limit_exceeded()) {
                        return;
                    }
                    let (net_no, mut clone) = {
                        let mut s = shared.lock().unwrap();
                        if s.next >= order.len() {
                            return;
                        }
                        let net_no = order[s.next];
                        s.next += 1;
                        // GREEDY tasks copy the live master; GLOBAL tasks
                        // conceptually copy the pass-start board, which is
                        // the same object since GLOBAL never updates it
                        (net_no, s.master.clone())
                    };
                    let improved =
                        optimize_nets_pass(&mut clone, request, &[net_no], time_limit) > 0;
                    let (vias_after, len_after) = net_route_cost(&clone, net_no);
                    let result = NetRouteResult {
                        net_no,
                        incomplete_after: usize::from(!clone.net_is_completely_connected(net_no)),
                        vias_after,
                        len_after,
                    };
                    let mut s = shared.lock().unwrap();
                    s.results.push(result);
                    if improved && s.best.as_ref().is_none_or(|(b, _)| result.improved_over(b)) {
                        if pass_strategy == BoardUpdateStrategy::Greedy {
                            s.master = clone.clone();
                            s.adopted += 1;
                        }
                        s.best = Some((result, clone));
                    }
                });
            }
        });
        let s = shared.into_inner().unwrap();
        for r in &s.results {
            prior_results.insert(r.net_no, *r);
        }
        let improved_this_pass = match pass_strategy {
            BoardUpdateStrategy::Greedy => {
                if s.adopted > 0 {
                    *board = s.master;
                }
                s.adopted
            }
            _ => match s.best {
                Some((_, winner)) => {
                    *board = winner;
                    1
                }
                None => 0,
            },
        };
        total += improved_this_pass;
        pass_no += 1;
        if improved_this_pass == 0 {
            break;
        }
        // Java's threshold stop: a pass gaining less than the threshold
        // relative score improvement ends the optimizer
        let score_after = board_score(board);
        let pass_improvement = if score_before > 0.0 {
            (score_after - score_before) / score_before
        } else {
            0.0
        };
        if pass_improvement < IMPROVEMENT_THRESHOLD {
            break;
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

/// Runs optimization passes with Java's stopping criteria
/// (BatchOptimizer.runBatchLoop): stop when the score is already close
/// to the maximum, and stop when a pass's relative score improvement
/// falls below the threshold (no improvement counts too). Returns the
/// total number of improvements.
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
        let score_before = board_score(board);
        if score_near_maximum(score_before) {
            break;
        }
        let improved =
            optimize_route_pass(board, request, time_limit) + optimize_vias(board, time_limit);
        total += improved;
        if improved == 0 {
            break;
        }
        let score_after = board_score(board);
        let pass_improvement = if score_before > 0.0 {
            (score_after - score_before) / score_before
        } else {
            0.0
        };
        if pass_improvement < IMPROVEMENT_THRESHOLD {
            break;
        }
    }
    total
}
