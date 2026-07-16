//! Port of the core loop of `BatchAutorouter.java` (single pass, no
//! ripup escalation yet): route the incomplete connections of every net
//! until each net forms one connected set.
//!
//! Incompletes are computed from the board connectivity: the connected
//! components of a net's connectable items; the closest pair of items
//! between two components becomes the next connection to route.

use crate::autoroute::maze_search::{
    maze_route_with_engine, maze_route_with_ripup, MazeRouteRequest,
};
use crate::board::basic_board::{BasicBoard, ItemId};

#[derive(Debug, Default, PartialEq, Eq)]
pub struct BatchResult {
    pub routed_connections: usize,
    pub failed_connections: usize,
    /// The nets of items ripped up while routing (need rerouting).
    pub ripped_nets: Vec<i32>,
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
    /// In-search ripup penalty per rippable item (0 = ripup disabled; see
    /// `MazeRouteRequest::ripup_penalty`).
    pub ripup_penalty: f64,
    /// Optional wall-clock deadline honored inside searches and cascades.
    pub deadline: Option<crate::datastructures::TimeLimit>,
}

/// The connected components of the connectable items of `net_no`.
pub fn net_components(board: &BasicBoard, net_no: i32) -> Vec<Vec<ItemId>> {
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
        if crate::debug::maze() {
            for member in &component {
                if assigned.contains(member) {
                    eprintln!(
                        "NET {net_no}: item {member:?} is in several components \
                         (asymmetric contacts; seed {item:?})"
                    );
                }
            }
        }
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
    route_net_with_store(board, net_no, request, &mut None)
}

/// Like [`route_net`], reusing the caller's engine store across calls:
/// the room graph persists across connections AND nets in plain mode
/// (Java: maintain_database), synchronized against board changes and
/// switched between nets before every connection. Ripup-mode requests
/// bypass the store (they rip and shove foreign items mid-connection).
pub fn route_net_with_store(
    board: &mut BasicBoard,
    net_no: i32,
    request: &BatchRequest,
    store: &mut Option<crate::autoroute::engine::AutorouteEngine>,
) -> BatchResult {
    let mut result = BatchResult::default();
    let mut prev_component_count = usize::MAX;
    let mut last_items: Vec<ItemId> = Vec::new();
    let mut use_sets = true;
    loop {
        if request.deadline.is_some_and(|t| t.limit_exceeded()) {
            result.failed_connections += 1;
            break;
        }
        let components = net_components(board, net_no);
        if components.len() <= 1 {
            break;
        }
        if components.len() >= prev_component_count {
            // a routed connection did not reduce the component count:
            // remove its items (junk that only obstructs other nets)
            if std::env::var_os("FR_KEEP_JUNK").is_some() {
                last_items.clear();
                result.failed_connections += 1;
                break;
            }
            for id in last_items.drain(..) {
                board.remove_item(id);
            }
            if use_sets {
                // retry the connection single-pair: set arrivals can pick
                // a target whose contact never registers (stacked pads)
                use_sets = false;
                prev_component_count = usize::MAX;
                continue;
            }
            result.failed_connections += 1;
            break;
        }
        prev_component_count = components.len();
        // route between the two closest components overall (minimum
        // spanning behavior, important for many-pin nets like power)
        let candidate_sets: Vec<Vec<ItemId>> = components
            .iter()
            .map(|c| endpoint_candidates(board, c))
            .collect();
        let dist_of = |p: &(ItemId, ItemId)| {
            let bb_a = board.get_item(p.0).unwrap().bounding_box(&board.padstacks);
            let bb_b = board.get_item(p.1).unwrap().bounding_box(&board.padstacks);
            bb_a.weighted_distance(bb_b, 1.0, 1.0)
        };
        let mut best: Option<(ItemId, ItemId)> = None;
        for i in 0..candidate_sets.len() {
            for j in i + 1..candidate_sets.len() {
                if let Some(pair) = closest_pair(board, &candidate_sets[i], &candidate_sets[j]) {
                    if best.is_none() || dist_of(&pair) < dist_of(best.as_ref().unwrap()) {
                        best = Some(pair);
                    }
                }
            }
        }
        let Some((start, dest)) = best else {
            break;
        };
        // route component to component (Java: p_start_set/p_dest_set):
        // the maze may start from any endpoint-capable item and arrive at
        // ANY connectable item of the destination component — trace
        // arrivals land exactly on the centerline lattice so the junction
        // split registers the contact (power-net taps)
        let start_component = candidate_sets
            .iter()
            .find(|c| c.contains(&start))
            .cloned()
            .unwrap_or_default();
        let dest_component = components
            .iter()
            .find(|c| c.contains(&dest))
            .map(|c| {
                c.iter()
                    .copied()
                    .filter(|id| board.get_item(*id).is_some_and(|it| it.is_connectable()))
                    .collect::<Vec<_>>()
            })
            .unwrap_or_default();
        if crate::debug::maze() {
            eprintln!(
                "ROUTE net {net_no}: connect item {start:?} -> item {dest:?} \
                 ({} components)",
                components.len()
            );
        }
        let maze_request = MazeRouteRequest {
            net_no,
            start_item: start,
            dest_item: dest,
            start_items: if use_sets {
                start_component
            } else {
                Vec::new()
            },
            dest_items: if use_sets { dest_component } else { Vec::new() },
            is_fanout: false,
            trace_half_width: request.trace_half_width,
            clearance_class: request.clearance_class,
            via_padstack: request.via_padstack,
            via_cost: request.via_cost,
            max_expansions: request.max_expansions,
            ripup_penalty: request.ripup_penalty,
            deadline: request.deadline,
        };
        let connection = if request.ripup_penalty > 0.0 {
            maze_route_with_ripup(board, &maze_request)
        } else {
            let usable = matches!(store, Some(e) if !e.allow_ripup
                && e.trace_clearance_class == request.clearance_class
                && e.trace_half_width == request.trace_half_width);
            if !usable {
                if crate::debug::maze() {
                    eprintln!(
                        "ENGINE new for net {net_no} (hw {} class {})",
                        request.trace_half_width, request.clearance_class
                    );
                }
                *store = Some(
                    crate::autoroute::engine::AutorouteEngine::new_with_clearance(
                        net_no,
                        false,
                        request.clearance_class,
                        request.trace_half_width,
                    ),
                );
            }
            let engine = store.as_mut().unwrap();
            engine.sync_board_changes(board);
            engine.switch_net(board, net_no);
            maze_route_with_engine(board, engine, &maze_request)
        };
        if let Some(connection) = connection {
            result.routed_connections += 1;
            last_items = connection.new_items.clone();
            result.ripped_nets.extend(connection.ripped_nets);
        } else {
            result.failed_connections += 1;
            break; // give up on this net (the pass may retry with ripup)
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

/// Like [`route_net`], but on failure retries with in-search ripup:
/// the maze may route through rippable foreign items paying
/// `ripup_penalty` per item, the crossed items are removed, and all
/// victims are rerouted immediately. Transactional: commits only if the
/// failed net and every victim end up completely connected, otherwise
/// the board state is restored.
pub fn route_net_with_ripup(
    board: &mut BasicBoard,
    net_no: i32,
    request: &BatchRequest,
    ripup_penalty: f64,
) -> BatchResult {
    let mut result = route_net(board, net_no, request);
    if result.failed_connections == 0 || ripup_penalty <= 0.0 {
        return result;
    }
    board.generate_snapshot();
    // cap the whole transaction (retry + victim recovery) to a per-net
    // budget: a single hard net otherwise burns a whole pass's wall
    // clock in ripup-mode completions (8088sbc: 45 s for 3 nets)
    let budget_ms = request
        .deadline
        .map(|t| t.remaining_ms())
        .unwrap_or(u64::MAX)
        .min(10_000);
    let sub_deadline = crate::datastructures::TimeLimit::new(budget_ms);
    let rip_request = BatchRequest {
        ripup_penalty,
        deadline: Some(sub_deadline),
        ..*request
    };
    let retry = route_net(board, net_no, &rip_request);
    let mut extra_routed = retry.routed_connections;
    let mut success = retry.failed_connections == 0 && board.net_is_completely_connected(net_no);
    let mut broken_victims = 0usize;
    if success {
        // Reroute the victims immediately without further ripup. At most
        // ONE victim net may stay broken (a 1-for-1 swap): completion
        // stays monotone while a hard failure becomes a failure-set
        // rotation that later passes and the restart fallback can attack
        // from the other side. (A bounded cascading variant was
        // benchmarked at iterations 55-58 and consistently regressed
        // completion by time starvation: 161/173 vs 166/173.)
        for &ripped in &retry.ripped_nets {
            if request.deadline.is_some_and(|t| t.limit_exceeded()) || sub_deadline.limit_exceeded()
            {
                // half-done victim recovery must not commit
                broken_victims = usize::MAX;
                break;
            }
            if board.net_is_completely_connected(ripped) {
                continue;
            }
            let victim_request = BatchRequest {
                deadline: Some(sub_deadline),
                ..*request
            };
            let r = route_net(board, ripped, &victim_request);
            extra_routed += r.routed_connections;
            if r.failed_connections > 0 || !board.net_is_completely_connected(ripped) {
                broken_victims += 1;
                if broken_victims > 1 {
                    break;
                }
            }
        }
        success = broken_victims <= 1;
    }
    if success {
        result.routed_connections += extra_routed;
        // a swap leaves one net broken: report it so the pass loop keeps
        // running; only a full recovery clears the failure count
        result.failed_connections = broken_victims;
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
/// order, later passes retrying the incomplete nets with in-search ripup
/// and a doubled expansion budget each time. Returns the result of the
/// final state.
pub fn batch_route_passes(
    board: &mut BasicBoard,
    request: &BatchRequest,
    passes: usize,
) -> BatchResult {
    batch_route_passes_with_time_limit(board, request, passes, None)
}

/// The request adjusted to `net_no`'s net class rules: its trace half
/// width and via padstack when the imported rules define them.
pub(crate) fn request_for_net(
    board: &BasicBoard,
    net_no: i32,
    base: &BatchRequest,
) -> BatchRequest {
    let class_half_width = board.rules.get_trace_half_width(net_no, 0);
    // Route each net with its own trace clearance class (Java: AutorouteControl
    // takes trace_clearance_class_no from the net class). The CLI/API base
    // request hardcoded class 1, so nets with a tighter or looser clearance
    // rule were routed and DRC-checked against the wrong spacing.
    let class_clearance = board.rules.get_trace_clearance_class(net_no);
    BatchRequest {
        trace_half_width: if class_half_width > 0 {
            class_half_width
        } else {
            base.trace_half_width
        },
        clearance_class: if class_clearance > 0 {
            class_clearance
        } else {
            base.clearance_class
        },
        via_padstack: board
            .rules
            .via_padstack_for_net(net_no)
            .unwrap_or(base.via_padstack),
        ..*base
    }
}

/// The nets of routed items violating the pairwise clearance to another
/// routed foreign item (post-routing audit; planes excepted).
fn violating_nets(board: &BasicBoard) -> Vec<i32> {
    use crate::board::ItemKind;
    let mut nets = Vec::new();
    let routed: Vec<ItemId> = board
        .items()
        .filter(|(_, it)| {
            it.base.component_no == 0
                && it.base.net_count() > 0
                && !matches!(it.kind, ItemKind::ObstacleArea(_))
        })
        .map(|(id, _)| *id)
        .collect();
    for &id in &routed {
        let Some(item) = board.get_item(id) else {
            continue;
        };
        let shapes: Vec<_> = item.tile_shapes(&board.padstacks).to_vec();
        'shapes: for (shape, layer) in shapes {
            let max_cl = board.rules.clearance_matrix.max_value(layer).max(0) as f64;
            for other_id in board.overlapping_items(&shape.offset(max_cl), Some(layer)) {
                if other_id == id {
                    continue;
                }
                let Some(other) = board.get_item(other_id) else {
                    continue;
                };
                if other.base.shares_net(&item.base) || other.base.component_no != 0 {
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
                    layer,
                    false,
                ) as f64;
                let check = shape.offset(cl);
                let conflict = other
                    .tile_shapes(&board.padstacks)
                    .iter()
                    .any(|(s, l)| *l == layer && s.intersection(&check).dimension() >= 2);
                if conflict {
                    nets.extend(item.base.net_nos.iter().copied());
                    nets.extend(other.base.net_nos.iter().copied());
                    break 'shapes;
                }
            }
        }
    }
    nets.sort();
    nets.dedup();
    nets
}

/// Rips and reroutes the nets with clearance violations among routed
/// items; incomplete beats illegal, so failed reroutes stay unrouted.
fn repair_violations(
    board: &mut BasicBoard,
    request: &BatchRequest,
    time_limit: Option<&crate::datastructures::TimeLimit>,
) -> usize {
    let mut repaired = 0;
    let all_nets: Vec<i32> = (1..=board.rules.nets.max_net_no()).collect();
    for _round in 0..2 {
        let nets = violating_nets(board);
        if nets.is_empty() {
            break;
        }
        // transactional: the repair may not trade completion away —
        // kept only when every rerouted net completes again
        let complete_before = all_nets
            .iter()
            .filter(|&&n| board.net_is_completely_connected(n))
            .count();
        board.generate_snapshot();
        let to_remove: Vec<ItemId> = board
            .items()
            .filter(|(_, it)| {
                it.base.component_no == 0
                    && it.is_routable()
                    && it.base.net_nos.iter().any(|n| nets.contains(n))
            })
            .map(|(id, _)| *id)
            .collect();
        for id in to_remove {
            board.remove_item(id);
        }
        let ripup_penalty = request.via_cost.max(20_000.0);
        // the repair routes carry the remaining overall budget so the
        // per-net sub-deadlines inside route_net_with_ripup are honest
        let repair_request = BatchRequest {
            deadline: time_limit.copied().or(request.deadline),
            ..*request
        };
        crate::board::basic_board::set_birth_tag(5);
        for &net_no in &nets {
            if time_limit.is_some_and(|t| t.limit_exceeded()) {
                break;
            }
            let net_request = request_for_net(board, net_no, &repair_request);
            route_net_with_ripup(board, net_no, &net_request, ripup_penalty);
        }
        crate::board::basic_board::set_birth_tag(0);
        let complete_after = all_nets
            .iter()
            .filter(|&&n| board.net_is_completely_connected(n))
            .count();
        if crate::debug::stats() {
            eprintln!(
                "REPAIR round {_round}: {} violating nets {:?}, complete {} -> {} ({})",
                nets.len(),
                nets,
                complete_before,
                complete_after,
                if complete_after >= complete_before {
                    "KEPT"
                } else {
                    "ROLLED BACK"
                }
            );
        }
        if complete_after >= complete_before {
            board.pop_snapshot();
            repaired += nets.len();
        } else {
            board.undo();
            break; // this round's reroutes failed; keep completion
        }
    }
    repaired
}

/// Like [`batch_route_passes`] with an optional wall-clock limit checked
/// between nets (Java: BatchAutorouter's TimeLimit); on expiry the batch
/// stops after the current connection and reports the state so far.
pub fn batch_route_passes_with_time_limit(
    board: &mut BasicBoard,
    request: &BatchRequest,
    passes: usize,
    time_limit: Option<&crate::datastructures::TimeLimit>,
) -> BatchResult {
    let mut net_nos: Vec<i32> = (1..=board.rules.nets.max_net_no()).collect();
    // many-pin (power) nets route FIRST on the open board — they need
    // whole corridor systems and are unroutable into leftover congestion
    // (interf_u's VCC: 22 connections, routable alone in 136 ms, never
    // completable when last); everything else keeps ascending extent,
    // which measured cleanest for signal nets
    let pin_count = |n: i32| -> usize {
        board
            .items()
            .filter(|(_, it)| it.base.contains_net(n) && it.is_connectable())
            .count()
    };
    net_nos.sort_by_key(|&n| {
        let pins = pin_count(n);
        if pins >= 6 {
            (0i64, (usize::MAX - pins) as i64, 0i64)
        } else {
            (1i64, 0i64, net_extent(board, n))
        }
    });
    if crate::debug::route_order_desc() {
        net_nos.reverse(); // experiment: reversed order
    }

    let mut total = BatchResult::default();
    let mut budget = request.max_expansions;
    // reserve part of the wall clock for the restart fallback below: the
    // passes otherwise consume the entire limit and the fallback (which
    // wins the hard nets) never gets to run
    let pass_limit = time_limit.copied().map(|mut t| {
        t.multiply(0.7);
        t
    });
    for pass in 0..passes.max(1) {
        let pass_start = std::time::Instant::now();
        let pass_request = BatchRequest {
            max_expansions: budget,
            deadline: pass_limit.or(request.deadline),
            ..*request
        };
        let mut failed_this_pass = 0usize;
        let mut engine_store: Option<crate::autoroute::engine::AutorouteEngine> = None;
        // in-search ripup is allowed from the second pass on, with the
        // penalty escalating per pass (Java: the ripup costs increase
        // with the pass number, so churny swaps become ever more
        // expensive and the passes converge)
        let ripup_penalty = if pass == 0 {
            0.0
        } else {
            request.via_cost.max(20_000.0) * pass as f64
        };
        let mut out_of_time = false;
        for &net_no in &net_nos {
            if pass_limit.is_some_and(|t| t.limit_exceeded()) {
                out_of_time = true;
                break;
            }
            if board.net_is_completely_connected(net_no) {
                continue;
            }
            let net_request = request_for_net(board, net_no, &pass_request);
            // cross-net room reuse (Java: maintain_database): a clear
            // win on big boards since SRN + obstacle rooms (8088sbc
            // pass 0: 18.9s → 9.8s, 2.6× fewer completions), a small
            // cost on tiny ones — default by board size, FR_CROSS_NET
            // overrides
            let cross_net = std::env::var("FR_CROSS_NET")
                .map(|v| v != "0")
                .unwrap_or_else(|_| board.item_count() >= 400);
            let net_start = std::time::Instant::now();
            let result = if ripup_penalty > 0.0 {
                route_net_with_ripup(board, net_no, &net_request, ripup_penalty)
            } else if cross_net {
                route_net_with_store(board, net_no, &net_request, &mut engine_store)
            } else {
                route_net(board, net_no, &net_request)
            };
            if crate::debug::stats() && net_start.elapsed().as_secs() >= 15 {
                eprintln!(
                    "SLOW NET {net_no} pass {pass}: {:.1?} ({} routed, {} failed)",
                    net_start.elapsed(),
                    result.routed_connections,
                    result.failed_connections
                );
            }
            total.routed_connections += result.routed_connections;
            failed_this_pass += result.failed_connections;
        }
        if crate::debug::stats() {
            let ts = crate::datastructures::min_area_tree::take_tree_stats();
            eprintln!(
                "PASS {pass} done in {:.1?}: {} failed (penalty {ripup_penalty}); \
                 tree {} queries, {} nodes ({:.1} nodes/query)",
                pass_start.elapsed(),
                failed_this_pass,
                ts.queries,
                ts.nodes_visited,
                ts.nodes_visited as f64 / ts.queries.max(1) as f64
            );
        }
        if out_of_time {
            // count the remaining incomplete nets as failures and stop
            total.failed_connections += net_nos
                .iter()
                .filter(|&&n| !board.net_is_completely_connected(n))
                .count();
            break;
        }
        // NOTE: pulling all traces tight between passes was tried here and
        // REVERTED: it regressed the interf_u benchmark from 165/173 in
        // 139 s to 154/173 in 236 s (tightened traces hug obstacles and
        // produce degenerate shapes that poison room completion). Tighten
        // only after routing finishes.
        if failed_this_pass == 0 {
            break;
        }
        if pass + 1 == passes {
            total.failed_connections += failed_this_pass;
        }
        budget = budget.saturating_mul(2);
    }

    // Restart fallback: incomplete nets often fail only because earlier
    // routed nets consumed their corridors (they typically route fine
    // alone). While time remains, repeatedly rip up all route items and
    // route the current failures FIRST (rotated per round for
    // diversity); each round is kept only when strictly more nets
    // complete (transactional via snapshot), so completion is monotonic.
    let mut dry_rounds = 0usize;
    let mut round = 0usize;
    let mut max_dry = 1usize;
    // the restart may not consume the tail of the budget: the final
    // violation-repair guarantee needs wall clock too (coldfire @600s
    // shipped 12 deep violations because a doomed 180s restart round
    // ran to the wire and repair got nothing)
    let restart_limit = time_limit.copied().map(|mut t| {
        t.multiply(0.9);
        t
    });
    while dry_rounds < max_dry && !restart_limit.is_some_and(|t| t.limit_exceeded()) {
        let mut incomplete: Vec<i32> = net_nos
            .iter()
            .copied()
            .filter(|&n| !board.net_is_completely_connected(n))
            .collect();
        if incomplete.is_empty() {
            break;
        }
        // with several failures the rotation gives each round a genuinely
        // different order, so more dry rounds are worth the time; with a
        // single failure the restart is deterministic and one dry round
        // settles it
        max_dry = (incomplete.len() + 1).min(4);
        round += 1;
        let round_start = std::time::Instant::now();
        let rot = round % incomplete.len();
        incomplete.rotate_left(rot);
        let complete_before = net_nos.len() - incomplete.len();
        board.generate_snapshot();
        let to_remove: Vec<ItemId> = board
            .items()
            .filter(|(_, it)| {
                it.base.component_no == 0 && it.base.net_count() > 0 && it.is_routable()
            })
            .map(|(id, _)| *id)
            .collect();
        for id in to_remove {
            board.remove_item(id);
        }
        let order: Vec<i32> = incomplete
            .iter()
            .copied()
            .chain(net_nos.iter().copied().filter(|n| !incomplete.contains(n)))
            .collect();
        let restart_request = BatchRequest {
            max_expansions: budget,
            deadline: restart_limit.or(request.deadline),
            ..*request
        };
        let ripup_penalty = request.via_cost.max(20_000.0);
        let mut restart = BatchResult::default();
        // FR_LOCK_RESTART experiment (corridor negotiation): when an
        // originally-incomplete net completes during the round, lock its
        // route items so later nets cannot rip its corridors back out
        // from under it; all locks release before the keep/rollback
        // decision. Rationale: the coldfire stragglers are the giant
        // power nets — they route fine first but signals erode their
        // corridors during the rest of the round.
        let lock_restart = std::env::var_os("FR_LOCK_RESTART").is_some();
        let mut locked: Vec<ItemId> = Vec::new();
        for &net_no in &order {
            if restart_limit.is_some_and(|t| t.limit_exceeded()) {
                break;
            }
            let net_request = request_for_net(board, net_no, &restart_request);
            let result = route_net_with_ripup(board, net_no, &net_request, ripup_penalty);
            restart.routed_connections += result.routed_connections;
            restart.failed_connections += result.failed_connections;
            if lock_restart
                && incomplete.contains(&net_no)
                && board.net_is_completely_connected(net_no)
            {
                let ids: Vec<ItemId> = board
                    .items()
                    .filter(|(_, it)| it.base.contains_net(net_no) && it.is_routable())
                    .map(|(id, _)| *id)
                    .collect();
                for id in ids {
                    board.set_fixed_state(id, crate::board::FixedState::UserFixed);
                    locked.push(id);
                }
            }
        }
        for id in locked {
            board.set_fixed_state(id, crate::board::FixedState::Unfixed);
        }
        let complete_after = net_nos
            .iter()
            .filter(|&&n| board.net_is_completely_connected(n))
            .count();
        let incomplete_after: Vec<i32> = net_nos
            .iter()
            .copied()
            .filter(|&n| !board.net_is_completely_connected(n))
            .collect();
        if crate::debug::stats() {
            eprintln!(
                "RESTART round {round} in {:.1?}: complete {} -> {}",
                round_start.elapsed(),
                complete_before,
                complete_after
            );
        }
        // a tie that CHANGES the failing-net set is worth taking once:
        // the next round attacks a different net first (with a single
        // stuck net the restart is otherwise deterministic and dry)
        let changed_set = complete_after == complete_before && incomplete_after != incomplete;
        if complete_after > complete_before || (changed_set && dry_rounds + 1 < max_dry) {
            board.pop_snapshot();
            total.routed_connections += restart.routed_connections;
            total.failed_connections = net_nos.len() - complete_after;
            if complete_after > complete_before {
                dry_rounds = 0;
            } else {
                dry_rounds += 1;
            }
        } else {
            board.undo();
            dry_rounds += 1;
        }
    }
    // final guarantee: no clearance violations among routed items —
    // violating nets are ripped and rerouted against the now-complete
    // board (incomplete beats illegal)
    repair_violations(board, request, time_limit);
    total.failed_connections = net_nos
        .iter()
        .filter(|&&n| !board.net_is_completely_connected(n))
        .count();
    if crate::debug::stats() {
        for &n in &net_nos {
            if board.net_is_completely_connected(n) {
                continue;
            }
            let name = board
                .rules
                .nets
                .get_by_no(n)
                .map(|x| x.name.clone())
                .unwrap_or_default();
            let pins = board
                .items()
                .filter(|(_, it)| it.base.contains_net(n) && it.is_connectable())
                .count();
            let fragments = net_components(board, n).len();
            eprintln!("INCOMPLETE net {n} \"{name}\": {pins} pins, {fragments} fragments");
        }
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
        let stack = LayerStructure::new(vec![Layer::new("F.Cu", true), Layer::new("B.Cu", true)]);
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
            ripup_penalty: 0.0,
            deadline: None,
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
        // like every import: configure the default class width (otherwise
        // request_for_net overrides the request width with the class
        // default)
        rules.set_default_trace_half_widths(100);
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
        for (lly, ury) in [(-40000, -3000), (3000, 40000)] {
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
            ripup_penalty: 0.0,
            deadline: None,
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
