//! Port of the core loop of `BatchAutorouter.java` (single pass, no
//! ripup escalation yet): route the incomplete connections of every net
//! until each net forms one connected set.
//!
//! Incompletes are computed from the board connectivity: the connected
//! components of a net's connectable items; the closest pair of items
//! between two components becomes the next connection to route.

use crate::autoroute::maze_search::{
    maze_route_with_engine, maze_route_with_ripup, MazeRouteRequest, RoutedConnection,
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
    /// The clearance class the inserted VIAS carry (Java: the selected
    /// `ViaInfo`'s clearance class, which may be stricter than the trace
    /// class). 0 = fall back to `clearance_class`.
    pub via_clearance_class: usize,
    /// The declared ViaInfo attach bit.  The maze may widen its *search*
    /// permission for a pure-SMD net, but that routing exception must not be
    /// persisted on the inserted via (Java keeps these two bits separate).
    pub via_attach_allowed: bool,
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

impl BatchRequest {
    /// Validates the caller-supplied base request before any maze state is
    /// allocated or board mutation begins. Per-net overrides are derived by
    /// `request_for_net`; this check only covers the fields the caller owns.
    pub fn validate(&self, board: &BasicBoard) -> Result<(), String> {
        if self.trace_half_width <= 0
            || self.trace_half_width > crate::geometry::planar::limits::CRIT_INT
        {
            return Err("trace_half_width must be positive".into());
        }
        if self.clearance_class >= board.rules.clearance_matrix.get_class_count() {
            return Err(format!(
                "clearance_class {} is outside the board matrix",
                self.clearance_class
            ));
        }
        if self.via_clearance_class >= board.rules.clearance_matrix.get_class_count() {
            return Err(format!(
                "via_clearance_class {} is outside the board matrix",
                self.via_clearance_class
            ));
        }
        if self.via_padstack != 0 && board.padstacks.get_by_no(self.via_padstack).is_none() {
            return Err(format!(
                "via_padstack {} is not present in the board library",
                self.via_padstack
            ));
        }
        if !self.via_cost.is_finite() || self.via_cost < 0.0 {
            return Err("via_cost must be finite and nonnegative".into());
        }
        if !self.ripup_penalty.is_finite() || self.ripup_penalty < 0.0 {
            return Err("ripup_penalty must be finite and nonnegative".into());
        }
        if self.max_expansions == 0 {
            return Err("max_expansions must be positive".into());
        }
        Ok(())
    }
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

/// Whether every physical item currently represented for `net_no` is in one
/// connected component. This is deliberately narrower than electrical
/// completeness: unresolved source endpoints remain a final failure, but they
/// must not prevent the router from connecting all geometry it does have.
fn net_routing_is_complete(board: &BasicBoard, net_no: i32) -> bool {
    let components = net_components(board, net_no);
    !components.is_empty() && components.len() == 1
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

enum RouteAttemptResult {
    Progress(RoutedConnection),
    NoProgress,
    Failed,
}

/// Resolves the snapshot opened immediately before one maze attempt. A
/// successful insertion is not progress until it reduces the target net's
/// component count; keeping the snapshot open through that check is required
/// because shove and normalization may have modified pre-existing items.
fn finish_route_attempt(
    board: &mut BasicBoard,
    net_no: i32,
    previous_component_count: usize,
    connection: Option<RoutedConnection>,
    keep_no_progress: bool,
) -> RouteAttemptResult {
    let Some(connection) = connection else {
        board.rollback_snapshot();
        return RouteAttemptResult::Failed;
    };
    if net_components(board, net_no).len() < previous_component_count {
        board.pop_snapshot();
        RouteAttemptResult::Progress(connection)
    } else {
        if keep_no_progress {
            board.pop_snapshot();
        } else {
            board.rollback_snapshot();
        }
        RouteAttemptResult::NoProgress
    }
}

/// Routes all incomplete connections of `net_no` from a base request.
/// Per-net width, clearance, ViaRule, attach policy, active layers and plane
/// via cost are always derived here, so the public single-net entry point has
/// the same rule contract as [`batch_route`] and the pass scheduler.
pub fn route_net(board: &mut BasicBoard, net_no: i32, request: &BatchRequest) -> BatchResult {
    if board.rules.nets.get_by_no(net_no).is_none() || request.validate(board).is_err() {
        return BatchResult {
            failed_connections: 1,
            ..BatchResult::default()
        };
    }
    if net_components(board, net_no).is_empty() {
        return BatchResult {
            failed_connections: crate::ratsnest::routing_failure_count_for_net(board, net_no),
            ..BatchResult::default()
        };
    }
    let net_request = request_for_net(board, net_no, request);
    let mut result = route_net_with_store(board, net_no, &net_request, &mut None);
    result.failed_connections += board.unresolved_net_endpoint_count(net_no);
    result
}

/// Like [`route_net`], reusing the caller's engine store across calls:
/// the room graph persists across connections AND nets in plain mode
/// (Java: maintain_database), synchronized against board changes and
/// switched between nets before every connection. Ripup-mode requests
/// bypass the store (they rip and shove foreign items mid-connection).
pub(crate) fn route_net_with_store(
    board: &mut BasicBoard,
    net_no: i32,
    request: &BatchRequest,
    store: &mut Option<crate::autoroute::engine::AutorouteEngine>,
) -> BatchResult {
    let mut result = BatchResult::default();
    if net_components(board, net_no).is_empty() {
        return result;
    }
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
        // Java plane routing (BatchAutorouter.autoroute_item): an item of a
        // plane net routes from its OWN component toward EVERYTHING not yet
        // connected to it (route_start_set = connected set, route_dest_set =
        // unconnected set) — the pour is therefore ALWAYS among the targets,
        // whichever pair happened to be closest. Orient the chosen pair so
        // the non-pour side is the start, and widen the dest set below.
        let contains_plane = board
            .rules
            .nets
            .get_by_no(net_no)
            .is_some_and(|n| n.contains_plane());
        let side_has_plane = |id: ItemId| {
            components
                .iter()
                .find(|c| c.contains(&id))
                .is_some_and(|c| {
                    c.iter().any(|&i| {
                        matches!(
                            board.get_item(i).map(|it| &it.kind),
                            Some(crate::board::ItemKind::ObstacleArea(a)) if a.is_conduction
                        )
                    })
                })
        };
        let (start, dest) = if contains_plane && side_has_plane(start) && !side_has_plane(dest) {
            (dest, start)
        } else {
            (start, dest)
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
        let dest_component = if contains_plane {
            // plane nets: every OTHER component is a target (the pour
            // included), like Java's unconnected set
            components
                .iter()
                .filter(|c| !c.contains(&start))
                .flat_map(|c| c.iter().copied())
                .filter(|id| board.get_item(*id).is_some_and(|it| it.is_connectable()))
                .collect::<Vec<_>>()
        } else {
            components
                .iter()
                .find(|c| c.contains(&dest))
                .map(|c| {
                    c.iter()
                        .copied()
                        .filter(|id| board.get_item(*id).is_some_and(|it| it.is_connectable()))
                        .collect::<Vec<_>>()
                })
                .unwrap_or_default()
        };
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
            via_clearance_class: request.via_clearance_class,
            via_attach_allowed: request.via_attach_allowed,
            via_cost: request.via_cost,
            max_expansions: request.max_expansions,
            ripup_penalty: request.ripup_penalty,
            deadline: request.deadline,
        };
        // The transaction spans insertion AND the progress check below.
        // insert_connection's inner snapshot only makes insertion failures
        // atomic; committing it early used to leak shove/normalization changes
        // when a nominally successful route did not join two components.
        board.generate_snapshot();
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
                    crate::autoroute::engine::AutorouteEngine::new_with_clearance_synced(
                        board,
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
        let keep_no_progress = std::env::var_os("FR_KEEP_JUNK").is_some();
        match finish_route_attempt(
            board,
            net_no,
            components.len(),
            connection,
            keep_no_progress,
        ) {
            RouteAttemptResult::Progress(connection) => {
                result.routed_connections += 1;
                result.ripped_nets.extend(connection.ripped_nets);
            }
            RouteAttemptResult::NoProgress if use_sets && !keep_no_progress => {
                // Set arrivals can pick a target whose contact never
                // registers (for example stacked pads); retry one exact pair
                // from the fully restored board.
                use_sets = false;
            }
            RouteAttemptResult::NoProgress | RouteAttemptResult::Failed => {
                result.failed_connections += 1;
                break; // give up on this net (the pass may retry with ripup)
            }
        }
    }
    result
}

/// Routes all nets with incomplete connections in ascending net-number
/// order (a single batch pass). Each net routes under its OWN class rules
/// (width, clearance class, via rule, plane via cost) — this public entry
/// derives them like the pass scheduler does, so external callers cannot
/// route with the base request's rules by accident.
pub fn batch_route(board: &mut BasicBoard, request: &BatchRequest) -> BatchResult {
    if request.validate(board).is_err() {
        return BatchResult {
            failed_connections: 1,
            ..BatchResult::default()
        };
    }
    let net_nos: Vec<i32> = (1..=board.rules.nets.max_net_no()).collect();
    let mut result = BatchResult::default();
    for net_no in net_nos {
        let net_result = route_net(board, net_no, request);
        result.routed_connections += net_result.routed_connections;
        result.failed_connections += net_result.failed_connections;
    }
    result.failed_connections = crate::ratsnest::routing_failure_count(board);
    result
}

/// Like [`route_net`], but on failure retries with in-search ripup:
/// the maze may route through rippable foreign items paying
/// `ripup_penalty` per item, the crossed items are removed, and all
/// victims are rerouted immediately. This public entry point is strict: it
/// commits only if the failed net and every victim end up completely
/// connected, otherwise the board state is restored.
///
/// `request` is the BASE request: the per-net rules (width, clearance
/// class, via rule, attach policy, plane via cost) are derived here for
/// the target AND separately for every victim — copying the
/// target-adjusted request onto the victims recreated a strict-clearance
/// victim under the aggressor's weaker class.
pub fn route_net_with_ripup(
    board: &mut BasicBoard,
    net_no: i32,
    request: &BatchRequest,
    ripup_penalty: f64,
) -> BatchResult {
    // Validate before opening a speculative snapshot. `generate_snapshot`
    // intentionally drops an existing user redo branch; an invalid request
    // must be a read-only rejection rather than a history mutation.
    if !valid_ripup_request(board, net_no, request, ripup_penalty) {
        return BatchResult {
            failed_connections: 1,
            ..BatchResult::default()
        };
    }
    if net_components(board, net_no).is_empty() {
        return BatchResult {
            failed_connections: crate::ratsnest::routing_failure_count_for_net(board, net_no),
            ..BatchResult::default()
        };
    }
    // `generate_snapshot` necessarily invalidates an already-exposed user
    // redo branch. Keep a full checkpoint only for that uncommon case so a
    // rejected valid route is observationally read-only; ordinary routing
    // retains the cheaper item-level transaction path.
    let history_checkpoint = board.can_redo().then(|| board.clone());
    let complete_before: Vec<bool> = (0..=board.rules.nets.max_net_no())
        .map(|net| net > 0 && board.net_is_completely_connected(net))
        .collect();
    board.generate_snapshot();
    let mut result = route_net_with_ripup_policy(board, net_no, request, ripup_penalty, false);
    let preserves_completed_nets = (1..=board.rules.nets.max_net_no())
        .all(|net| !complete_before[net as usize] || board.net_is_completely_connected(net));
    if result.failed_connections == 0
        && board.net_is_completely_connected(net_no)
        && preserves_completed_nets
    {
        board.pop_snapshot();
    } else {
        if let Some(checkpoint) = history_checkpoint {
            *board = checkpoint;
        } else {
            board.rollback_snapshot();
        }
        result.routed_connections = 0;
        result.failed_connections = result.failed_connections.max(1);
        result.ripped_nets.clear();
    }
    result
}

fn valid_ripup_request(
    board: &BasicBoard,
    net_no: i32,
    request: &BatchRequest,
    ripup_penalty: f64,
) -> bool {
    board.rules.nets.get_by_no(net_no).is_some()
        && request.validate(board).is_ok()
        && ripup_penalty.is_finite()
        && ripup_penalty >= 0.0
}

/// Internal pass-loop policy. A bounded one-for-one failure-set rotation is
/// useful to the scheduler because a later pass can attack the broken victim;
/// it is never exposed through the public single-net API.
fn route_net_with_ripup_policy(
    board: &mut BasicBoard,
    net_no: i32,
    request: &BatchRequest,
    ripup_penalty: f64,
    allow_one_broken_victim: bool,
) -> BatchResult {
    if !valid_ripup_request(board, net_no, request, ripup_penalty) {
        return BatchResult {
            failed_connections: 1,
            ..BatchResult::default()
        };
    }
    // The preliminary attempt is deliberately non-ripping. The caller's
    // base request may carry a default ripup cost for other APIs; honoring it
    // here would let the early-success return strand an unreported victim.
    let net_request = BatchRequest {
        ripup_penalty: 0.0,
        ..request_for_net(board, net_no, request)
    };
    let mut result = route_net_with_store(board, net_no, &net_request, &mut None);
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
        ..net_request
    };
    let retry = route_net_with_store(board, net_no, &rip_request, &mut None);
    let mut extra_routed = retry.routed_connections;
    let mut success = retry.failed_connections == 0 && net_routing_is_complete(board, net_no);
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
            if net_routing_is_complete(board, ripped) {
                continue;
            }
            // the victim reroutes under ITS OWN net-class rules, derived
            // from the base request — not the target-adjusted one
            let victim_request = BatchRequest {
                deadline: Some(sub_deadline),
                ripup_penalty: 0.0,
                ..request_for_net(board, ripped, request)
            };
            let r = route_net_with_store(board, ripped, &victim_request, &mut None);
            extra_routed += r.routed_connections;
            if r.failed_connections > 0 || !net_routing_is_complete(board, ripped) {
                broken_victims += 1;
                if broken_victims > 1 {
                    break;
                }
            }
        }
        success = if allow_one_broken_victim {
            broken_victims <= 1
        } else {
            broken_victims == 0
        };
    }
    if success {
        result.routed_connections += extra_routed;
        // a swap leaves one net broken: report it so the pass loop keeps
        // running; only a full recovery clears the failure count
        result.failed_connections = broken_victims;
        board.pop_snapshot();
    } else {
        board.rollback_snapshot();
    }
    result
}

/// Optimizer-facing ripup policy. Unlike the public strict entry point, this
/// leaves a partially improving preliminary route in the caller's snapshot so
/// the optimizer's board-wide metric/DRC gate can decide whether to keep it.
/// Victim recovery remains transactional inside the policy itself.
pub(crate) fn route_net_with_ripup_for_optimizer(
    board: &mut BasicBoard,
    net_no: i32,
    request: &BatchRequest,
    ripup_penalty: f64,
) -> BatchResult {
    route_net_with_ripup_policy(board, net_no, request, ripup_penalty, false)
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
    // the maze routes every layer at one width: take the class's maximum
    // over its ACTIVE signal layers, so a layer-dependent width rule is
    // never undercut (sampling only layer 0 could pick the narrow layer)
    let class_half_width = board.rules.get_trace_half_width_max_active(net_no);
    // Route each net with its own trace clearance class (Java: AutorouteControl
    // takes trace_clearance_class_no from the net class). The CLI/API base
    // request hardcoded class 1, so nets with a tighter or looser clearance
    // rule were routed and DRC-checked against the wrong spacing.
    let class_clearance = board.rules.get_trace_clearance_class(net_no);
    // Java plane-net routing (BatchAutorouter.autoroute_item): a net
    // carrying a copper pour uses get_plane_via_costs() — default 5 vs 50,
    // one tenth — so the router prefers a short stub dropping into the
    // plane over long surface traces. This is the contains_plane flag's
    // production consumer.
    let contains_plane = board
        .rules
        .nets
        .get_by_no(net_no)
        .is_some_and(|n| n.contains_plane());
    // the net's via rule selects the VIA INFO (span-aware: the first via
    // whose padstack covers the full routing span, so a blind-first rule
    // does not lock the router onto an unusable via) — it carries the
    // padstack, the via's OWN clearance class and the attach flag
    let last_layer = board.layer_structure.layer_count().saturating_sub(1);
    let selected_via = board
        .rules
        .selected_via_for_net(net_no, &board.padstacks, last_layer);
    let (via_padstack, via_clearance_class) = match selected_via {
        Some(info) => (info.get_padstack(), info.get_clearance_class()),
        None if board.rules.has_bound_via_rule(net_no) => (0, 0),
        None => (base.via_padstack, 0),
    };
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
        via_padstack,
        via_clearance_class,
        // Preserve the selected ViaInfo's declared attach bit. Pure-SMD
        // escape permission is represented separately on the routed via.
        via_attach_allowed: via_attach_allowed_for_net(board, net_no, base.via_padstack),
        via_cost: if contains_plane {
            base.via_cost / 10.0
        } else {
            base.via_cost
        },
        ..*base
    }
}

/// The declared ViaInfo attach bit for the via selected for this net.
/// Pure-SMD escape permission is search state and does not rewrite this rule.
pub(crate) fn via_attach_allowed_for_net(
    board: &BasicBoard,
    net_no: i32,
    base_padstack: usize,
) -> bool {
    // the same span-aware selection as request_for_net, so the attach
    // flag belongs to the via actually being inserted
    let last_layer = board.layer_structure.layer_count().saturating_sub(1);
    let declared_attach = board
        .rules
        .selected_via_for_net(net_no, &board.padstacks, last_layer)
        .map(|info| info.attach_smd_allowed())
        .or_else(|| board.rules.has_bound_via_rule(net_no).then_some(false))
        .unwrap_or_else(|| {
            board.rules.via_at_smd_allowed
                && board
                    .padstacks
                    .get_by_no(base_padstack)
                    .is_some_and(|p| p.attach_allowed)
        });
    declared_attach
}

/// Whether all original component pads of a net are single-layer SMD pads.
/// Route-created traces/vias have `component_no == 0` and are ignored, so the
/// result remains stable after the first fanout connection is inserted.
pub(crate) fn pure_smd_search_relaxation(board: &BasicBoard, net_no: i32) -> bool {
    if board.layer_structure.layer_count() <= 1 {
        return false;
    }
    let mut any_pin = false;
    for (_, item) in board.items() {
        if !item.base.contains_net(net_no) || !item.is_connectable() {
            continue;
        }
        // Component-owned drill items are the original pins.  Ignore
        // route-created items (component_no == 0), but reject a through-hole
        // pin and any board-level connectable area.
        if item.base.component_no == 0 {
            if matches!(item.kind, crate::board::ItemKind::ObstacleArea(_)) {
                return false;
            }
            continue;
        }
        any_pin = true;
        if !matches!(item.kind, crate::board::ItemKind::Via(_))
            || item.first_layer(&board.padstacks) != item.last_layer(&board.padstacks)
        {
            return false;
        }
    }
    any_pin
}

/// The nets involved in repairable clearance violations plus the count
/// of those violations, from the authoritative DRC — one rule set for
/// routing, repair and reporting (previously a hand-rolled audit that
/// skipped pins, conduction flags and same-net drills, and read the
/// transposed matrix cell).
fn violating_nets(board: &BasicBoard) -> (Vec<i32>, usize) {
    let report = crate::drc::check_board(board);
    let mut nets = Vec::new();
    let mut repairable = 0usize;
    for v in &report.violations {
        // only violations touching a route item can be repaired by
        // rerouting; a placement-level overlap (pin vs pin/keepout) is
        // not routable away
        let routable = [v.first_item, v.second_item].iter().any(|&id| {
            board
                .get_item(id)
                .is_some_and(|it| it.base.component_no == 0)
        });
        if !routable {
            continue;
        }
        repairable += 1;
        for id in [v.first_item, v.second_item] {
            if let Some(it) = board.get_item(id) {
                nets.extend(it.base.net_nos.iter().copied());
            }
        }
    }
    nets.sort_unstable();
    nets.dedup();
    nets.retain(|&n| n > 0);
    (nets, repairable)
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
        let (nets, violations_before) = violating_nets(board);
        if nets.is_empty() {
            break;
        }
        // transactional: the repair may not trade completion away — kept
        // only when every net that was complete BEFORE is still complete
        // after. The former scalar count admitted a one-for-one swap
        // (break a previously-complete victim while completing another).
        let complete_before: Vec<i32> = all_nets
            .iter()
            .copied()
            .filter(|&n| board.net_is_completely_connected(n))
            .collect();
        board.generate_snapshot();
        let to_remove: Vec<ItemId> = board
            .items()
            .filter(|(_, it)| {
                it.base.component_no == 0
                    && it.is_routable()
                    && !it.base.is_shove_fixed()
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
        let _birth_tag = crate::board::basic_board::birth_tag_scope(5);
        for &net_no in &nets {
            if time_limit.is_some_and(|t| t.limit_exceeded()) {
                break;
            }
            route_net_with_ripup_policy(board, net_no, &repair_request, ripup_penalty, false);
        }
        let no_net_broken = complete_before
            .iter()
            .all(|&n| board.net_is_completely_connected(n));
        // a repair round is kept only when it made real progress: no
        // previously-complete net broken AND strictly fewer violations — a
        // round that merely preserves the complete-net count while leaving
        // the violations in place (or moving them) is rolled back
        let (_, violations_after) = violating_nets(board);
        let keep = no_net_broken && violations_after < violations_before;
        if crate::debug::stats() {
            eprintln!(
                "REPAIR round {_round}: {} violating nets {:?}, complete {} (broken: {}), \
                 violations {} -> {} ({})",
                nets.len(),
                nets,
                complete_before.len(),
                !no_net_broken,
                violations_before,
                violations_after,
                if keep { "KEPT" } else { "ROLLED BACK" }
            );
        }
        if keep {
            board.pop_snapshot();
            repaired += violations_before - violations_after;
        } else {
            board.rollback_snapshot();
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
    if request.validate(board).is_err() {
        return BatchResult {
            failed_connections: 1,
            ..BatchResult::default()
        };
    }
    let mut net_nos: Vec<i32> = (1..=board.rules.nets.max_net_no())
        .filter(|&net| !net_components(board, net).is_empty())
        .collect();
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
    // Plateau guard: track the best (fewest) failures seen and how many
    // consecutive passes have failed to beat it. Once the pass loop stops
    // completing new nets, its escalating-penalty passes just re-run the same
    // deterministic search (J2: ~9999 identical 2.5 ms no-progress passes ≈
    // 25 s); we stop and let the restart fallback finish instead. The limit is
    // generous — each stalled pass also doubles the expansion budget, so after
    // this many passes the budget is ~2^8x and penalty ~8x the base; a net that
    // completion needs more escalation than that will not finish in the pass
    // loop anyway (the restart fallback is the stronger path).
    const PASS_STALL_LIMIT: usize = 8;
    let mut best_failed = usize::MAX;
    let mut stall = 0usize;
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
            if net_routing_is_complete(board, net_no) {
                continue;
            }
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
                // takes the BASE request: target and victim rules are
                // derived per net inside
                route_net_with_ripup_policy(board, net_no, &pass_request, ripup_penalty, true)
            } else {
                let net_request = request_for_net(board, net_no, &pass_request);
                if cross_net {
                    route_net_with_store(board, net_no, &net_request, &mut engine_store)
                } else {
                    route_net_with_store(board, net_no, &net_request, &mut None)
                }
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
                .filter(|&&n| !net_routing_is_complete(board, n))
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
        // Stop the pass loop once it has stalled (no fewer failures) for
        // several consecutive passes and hand off to the restart fallback,
        // which rips everything and routes the failures first — a stronger
        // completion path. A pass that reduces the failure count resets the
        // counter, so boards making gradual progress are unaffected; only a
        // genuine plateau (like J2's) is cut short.
        if failed_this_pass < best_failed {
            best_failed = failed_this_pass;
            stall = 0;
        } else {
            stall += 1;
            if stall >= PASS_STALL_LIMIT {
                total.failed_connections += failed_this_pass;
                break;
            }
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
            .filter(|&n| !net_routing_is_complete(board, n))
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
        // ShoveFixed route items survive the restart (Java's optimizer
        // never seeds with them; deleting them here would recreate the
        // protected copper Unfixed)
        let to_remove: Vec<ItemId> = board
            .items()
            .filter(|(_, it)| {
                it.base.component_no == 0
                    && it.base.net_count() > 0
                    && it.is_routable()
                    && !it.base.is_shove_fixed()
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
        let mut locked: Vec<(ItemId, crate::board::FixedState)> = Vec::new();
        for &net_no in &order {
            if restart_limit.is_some_and(|t| t.limit_exceeded()) {
                break;
            }
            let result =
                route_net_with_ripup_policy(board, net_no, &restart_request, ripup_penalty, true);
            restart.routed_connections += result.routed_connections;
            restart.failed_connections += result.failed_connections;
            if lock_restart
                && incomplete.contains(&net_no)
                && net_routing_is_complete(board, net_no)
            {
                let ids: Vec<(ItemId, crate::board::FixedState)> = board
                    .items()
                    .filter(|(_, it)| it.base.contains_net(net_no) && it.is_routable())
                    .map(|(id, it)| (*id, it.base.fixed_state))
                    .collect();
                for (id, prior) in ids {
                    board.set_fixed_state(id, crate::board::FixedState::UserFixed);
                    locked.push((id, prior));
                }
            }
        }
        // release to the PRIOR state, not Unfixed — the lock must not strip
        // ShoveFixed protection from items it temporarily pinned
        for (id, prior) in locked {
            board.set_fixed_state(id, prior);
        }
        let complete_after = net_nos
            .iter()
            .filter(|&&n| net_routing_is_complete(board, n))
            .count();
        let incomplete_after: Vec<i32> = net_nos
            .iter()
            .copied()
            .filter(|&n| !net_routing_is_complete(board, n))
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
            board.rollback_snapshot();
            dry_rounds += 1;
        }
    }
    // final guarantee: no clearance violations among routed items —
    // violating nets are ripped and rerouted against the now-complete
    // board (incomplete beats illegal)
    repair_violations(board, request, time_limit);
    total.failed_connections = crate::ratsnest::routing_failure_count(board);
    if crate::debug::stats() {
        for &n in &net_nos {
            if net_routing_is_complete(board, n) {
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
            via_clearance_class: 0,
            via_attach_allowed: false,
            via_cost: 5000.0,
            max_expansions: 100_000,
            ripup_penalty: 0.0,
            deadline: None,
        }
    }

    #[test]
    fn public_single_net_routes_reject_invalid_contracts_without_mutation() {
        let mut board = test_board(1);
        let before_items = board.items().count();

        let missing = route_net(&mut board, 999, &request());
        assert_eq!(missing.routed_connections, 0);
        assert_eq!(missing.failed_connections, 1);
        assert_eq!(board.items().count(), before_items);

        let missing_ripup = route_net_with_ripup(&mut board, 999, &request(), 1.0);
        assert_eq!(missing_ripup.routed_connections, 0);
        assert_eq!(missing_ripup.failed_connections, 1);
        assert_eq!(board.items().count(), before_items);

        // Validation failures are read-only: an existing user redo branch
        // must survive the rejected API call.
        let mut history = test_board(1);
        history.generate_snapshot();
        let transient = history.insert_via(1, IntPoint::new(0, 0), vec![1], 1, false);
        assert!(history.undo());
        assert!(history.get_item(transient).is_none());
        let invalid = route_net_with_ripup(&mut history, 999, &request(), 1.0);
        assert_eq!(invalid.failed_connections, 1);
        assert!(history.redo());
        assert!(history.get_item(transient).is_some());

        let empty = route_net(&mut board, 1, &request());
        assert_eq!(empty.routed_connections, 0);
        assert_eq!(empty.failed_connections, 0);
        let empty_ripup = route_net_with_ripup(&mut board, 1, &request(), 1.0);
        assert_eq!(empty_ripup.failed_connections, 0);
        assert_eq!(board.items().count(), before_items);

        let empty_batch = batch_route_passes(&mut board, &request(), 99);
        assert_eq!(empty_batch.routed_connections, 0);
        assert_eq!(empty_batch.failed_connections, 0);
        assert_eq!(board.items().count(), before_items);

        board.record_unresolved_net_endpoint(
            1,
            crate::board::basic_board::LogicalEndpoint::new("MISSING", "1"),
        );
        assert_eq!(route_net(&mut board, 1, &request()).failed_connections, 1);
        assert_eq!(
            route_net_with_ripup(&mut board, 1, &request(), 1.0).failed_connections,
            1
        );

        // Two unresolved terminals represent one logical connection, not two
        // independently routable failures.  The direct single-net APIs must
        // agree with the batch/ratsnest failure metric in this empty-physical
        // case.
        let mut multiple_unresolved = test_board(1);
        for component in ["MISSING_A", "MISSING_B"] {
            multiple_unresolved.record_unresolved_net_endpoint(
                1,
                crate::board::basic_board::LogicalEndpoint::new(component, "1"),
            );
        }
        assert_eq!(
            route_net(&mut multiple_unresolved, 1, &request()).failed_connections,
            1
        );
        assert_eq!(
            route_net_with_ripup(&mut multiple_unresolved, 1, &request(), 1.0).failed_connections,
            1
        );

        let zero_budget = BatchRequest {
            max_expansions: 0,
            ..request()
        };
        let rejected = route_net(&mut board, 1, &zero_budget);
        assert_eq!(rejected.routed_connections, 0);
        assert_eq!(rejected.failed_connections, 1);
        assert_eq!(board.items().count(), before_items);

        let mut partial = test_board(1);
        partial.insert_via(1, IntPoint::new(0, 0), vec![1], 1, false);
        partial.insert_via(1, IntPoint::new(8_000, 0), vec![1], 1, false);
        partial.record_unresolved_net_endpoint(
            1,
            crate::board::basic_board::LogicalEndpoint::new("MISSING", "1"),
        );
        let partial_result = route_net(&mut partial, 1, &request());
        assert_eq!(partial_result.routed_connections, 1);
        assert_eq!(partial_result.failed_connections, 1);
        assert_eq!(net_components(&partial, 1).len(), 1);
        assert!(!partial.net_is_completely_connected(1));
    }

    #[test]
    fn public_ripup_failure_restores_the_entire_entry_state() {
        use crate::geometry::planar::{PolygonShape, PolylineArea};

        let mut board = test_board(1);
        for point in [
            IntPoint::new(-8_000, 0),
            IntPoint::new(-4_000, 0),
            IntPoint::new(8_000, 0),
        ] {
            let pin = board.insert_via(1, point, vec![1], 1, false);
            board.set_component_no(pin, 1);
        }
        // Enclose the third pin on both layers. The two left pins can route,
        // but the final connection cannot; the public transaction must roll
        // that first successful connection back as well.
        for layer in 0..2 {
            for (x1, y1, x2, y2) in [
                (5_500, -2_500, 10_500, -2_000),
                (5_500, 2_000, 10_500, 2_500),
                (5_500, -2_000, 6_000, 2_000),
                (10_000, -2_000, 10_500, 2_000),
            ] {
                let area = PolylineArea::new(
                    PolygonShape::from_int_points(&[
                        IntPoint::new(x1, y1),
                        IntPoint::new(x2, y1),
                        IntPoint::new(x2, y2),
                        IntPoint::new(x1, y2),
                    ]),
                    Vec::new(),
                );
                board.insert_area(area, layer, "wall", Vec::new(), 1, false);
            }
        }
        let before = board.item_count();

        // The optimizer-facing policy is intentionally not the strict public
        // transaction: it leaves a useful preliminary connection in the
        // caller's snapshot when the final hard connection is impossible.
        let mut optimizer_candidate = board.clone();
        let candidate =
            route_net_with_ripup_for_optimizer(&mut optimizer_candidate, 1, &request(), 20_000.0);
        assert!(candidate.routed_connections > 0);
        assert!(net_components(&optimizer_candidate, 1).len() < 3);
        assert!(!optimizer_candidate.net_is_completely_connected(1));

        let result = route_net_with_ripup(&mut board, 1, &request(), 20_000.0);

        assert_eq!(result.routed_connections, 0);
        assert_eq!(result.failed_connections, 1);
        assert_eq!(board.item_count(), before);
        assert!(!board.items().any(|(_, item)| {
            item.base.component_no == 0
                && matches!(item.kind, crate::board::ItemKind::PolylineTrace(_))
        }));
        assert!(!board.redo(), "a rejected route must not be redoable");

        // A valid speculative call must also preserve a caller's preexisting
        // redo branch. The checkpoint is only needed in this history-bearing
        // case; ordinary rejected routes use rollback-and-discard above.
        let mut history = board.clone();
        history.generate_snapshot();
        let transient = history.insert_via(1, IntPoint::new(30_000, 30_000), Vec::new(), 1, false);
        assert!(history.undo());
        assert!(history.can_redo());
        let history_result = route_net_with_ripup(&mut history, 1, &request(), 20_000.0);
        assert_eq!(history_result.routed_connections, 0);
        assert!(history.redo(), "rejected routing must preserve user redo");
        assert!(history.get_item(transient).is_some());
    }

    #[test]
    fn optimizer_never_rips_shove_fixed_routes() {
        use crate::geometry::planar::Polyline;
        let mut board = test_board(1);
        let a = board.insert_via(1, IntPoint::new(0, 0), vec![1], 1, false);
        board.set_component_no(a, 1);
        let b = board.insert_via(1, IntPoint::new(20000, 10000), vec![1], 1, false);
        board.set_component_no(b, 2);
        // a deliberately dog-legged but PROTECTED route: ShoveFixed items
        // are not optimizer seeds (Java BatchOptimizer skips them), so the
        // detour must survive even though a shorter route exists
        let via = board.insert_via(1, IntPoint::new(0, 10000), vec![1], 1, false);
        let t1 = board.insert_trace(
            Polyline::from_int_points(&[IntPoint::new(0, 0), IntPoint::new(0, 10000)]),
            0,
            100,
            vec![1],
            1,
        );
        let t2 = board.insert_trace(
            Polyline::from_int_points(&[IntPoint::new(0, 10000), IntPoint::new(20000, 10000)]),
            1,
            100,
            vec![1],
            1,
        );
        for id in [via, t1, t2] {
            board.set_fixed_state(id, crate::board::FixedState::ShoveFixed);
        }
        assert!(board.net_is_completely_connected(1));
        crate::autoroute::optimizer::optimize_route_pass(&mut board, &request(), None);
        for id in [via, t1, t2] {
            assert!(
                board.get_item(id).is_some(),
                "ShoveFixed route items must survive the optimizer"
            );
        }
        assert!(board.net_is_completely_connected(1));
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
    fn no_progress_attempt_restores_preexisting_shove_mutations() {
        use crate::geometry::planar::Polyline;

        let mut board = test_board(2);
        board.insert_via(1, IntPoint::new(0, 0), vec![1], 1, false);
        board.insert_via(1, IntPoint::new(10_000, 0), vec![1], 1, false);
        let victim = board.insert_trace(
            Polyline::from_int_points(&[IntPoint::new(-5_000, 2_000), IntPoint::new(5_000, 2_000)]),
            0,
            100,
            vec![2],
            1,
        );
        let victim_before = board.get_item(victim).cloned().unwrap();
        let ids_before: Vec<_> = board.items().map(|(id, _)| *id).collect();
        let components_before = net_components(&board, 1).len();
        assert_eq!(components_before, 2);

        // Model the nested insertion transaction: a shove replaces an item
        // that predates the route attempt, insertion commits its own snapshot,
        // but the new target-net trace does not join the two components.
        board.generate_snapshot(); // route-attempt snapshot
        board.generate_snapshot(); // insert_connection snapshot
        board.remove_item(victim);
        board.insert_trace(
            Polyline::from_int_points(&[IntPoint::new(-5_000, 3_000), IntPoint::new(5_000, 3_000)]),
            0,
            100,
            vec![2],
            1,
        );
        board.insert_trace(
            Polyline::from_int_points(&[IntPoint::new(20_000, 0), IntPoint::new(21_000, 0)]),
            0,
            100,
            vec![1],
            1,
        );
        assert!(board.pop_snapshot());

        let outcome = finish_route_attempt(
            &mut board,
            1,
            components_before,
            Some(RoutedConnection {
                ripped_nets: Vec::new(),
            }),
            false,
        );
        assert!(matches!(outcome, RouteAttemptResult::NoProgress));
        assert_eq!(
            board.get_item(victim),
            Some(&victim_before),
            "rollback must restore the pre-existing shoved item"
        );
        assert_eq!(
            board.items().map(|(id, _)| *id).collect::<Vec<_>>(),
            ids_before,
            "rollback must remove every item created by the attempt"
        );
        assert_eq!(net_components(&board, 1).len(), components_before);
    }

    #[test]
    fn pure_smd_attach_relaxation_survives_route_items() {
        use crate::rules::{ViaInfo, ViaRule};

        let stack = LayerStructure::new(vec![Layer::new("F.Cu", true), Layer::new("B.Cu", true)]);
        let matrix = ClearanceMatrix::get_default_instance(stack.clone(), 200);
        let mut rules = BoardRules::new(stack.clone(), matrix);
        let default_class = rules.get_default_net_class();
        let net_no = rules.nets.add("smd_net", default_class, false);
        let mut padstacks = Padstacks::new(2);
        let via_shape = TileShape::Box(IntBox::from_coords(-400, -400, 400, 400));
        let via_padstack = padstacks.add(
            "via_attach_off",
            vec![Some(via_shape.clone()), Some(via_shape.clone())],
            false,
            false,
        );
        let smd_front = padstacks.add(
            "smd_front",
            vec![Some(via_shape.clone()), None],
            false,
            false,
        );
        let smd_back = padstacks.add("smd_back", vec![None, Some(via_shape)], false, false);
        let via_info = rules
            .via_infos
            .add(ViaInfo::new("via_attach_off", via_padstack, 1, false))
            .unwrap();
        let mut via_rule = ViaRule::new("smd_rule");
        via_rule.append_via(via_info);
        rules.via_rules.push(via_rule);
        rules
            .net_classes
            .get_mut(default_class)
            .set_via_rule(Some(0));
        let mut board = BasicBoard::new(stack, rules, padstacks);

        let pin_a = board.insert_via(smd_front, IntPoint::new(0, 0), vec![net_no], 1, false);
        board.set_component_no(pin_a, 1);
        let pin_b = board.insert_via(smd_back, IntPoint::new(8000, 0), vec![net_no], 1, false);
        board.set_component_no(pin_b, 2);

        assert!(pure_smd_search_relaxation(&board, net_no));
        assert!(!via_attach_allowed_for_net(&board, net_no, via_padstack));

        let net_request = request_for_net(&board, net_no, &request());
        assert!(
            !net_request.via_attach_allowed,
            "ViaInfo declaration stays off"
        );
        let routed = route_net(&mut board, net_no, &request());
        assert_eq!(routed.failed_connections, 0);
        assert!(board.net_is_completely_connected(net_no));

        let escape_vias: Vec<_> = board
            .items()
            .filter_map(|(_, item)| match &item.kind {
                crate::board::ItemKind::Via(via)
                    if item.base.component_no == 0 && via.is_escape_via =>
                {
                    Some(via)
                }
                _ => None,
            })
            .collect();
        assert!(
            !escape_vias.is_empty(),
            "the layer transition must escape a pad"
        );
        assert!(escape_vias
            .iter()
            .all(|via| { !via.attach_allowed && via.escape_smd_layer.is_some() }));
        assert!(crate::drc::check_board(&board).violations.is_empty());

        // A route-created via away from every SMD pin is ordinary and must
        // not inherit the net-wide search relaxation as persistent metadata.
        let ordinary = board.insert_via(
            via_padstack,
            IntPoint::new(4000, 6000),
            vec![net_no],
            1,
            false,
        );
        let crate::board::ItemKind::Via(ordinary) = &board.get_item(ordinary).unwrap().kind else {
            unreachable!()
        };
        assert!(!ordinary.is_escape_via);
        assert_eq!(ordinary.escape_smd_layer, None);

        // A routed via has component_no == 0. It must not turn the original
        // all-SMD pin classification off for the next connection request.
        assert!(pure_smd_search_relaxation(&board, net_no));
        assert!(!via_attach_allowed_for_net(&board, net_no, via_padstack));
    }

    #[test]
    fn bound_empty_via_rule_does_not_fall_back_to_base_via() {
        use crate::rules::ViaRule;

        let mut board = test_board(1);
        board.rules.via_rules.push(ViaRule::new("no_vias"));
        let default_class = board.rules.get_default_net_class();
        board
            .rules
            .net_classes
            .get_mut(default_class)
            .set_via_rule(Some(0));

        let restricted = request_for_net(&board, 1, &request());
        assert_eq!(restricted.via_padstack, 0, "bound-empty means no via");
        assert!(!restricted.via_attach_allowed);

        board
            .rules
            .net_classes
            .get_mut(default_class)
            .set_via_rule(None);
        assert_eq!(
            request_for_net(&board, 1, &request()).via_padstack,
            request().via_padstack,
            "an unbound class retains the legacy request fallback"
        );
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
    fn public_route_net_derives_the_net_class_from_a_base_request() {
        let mut board = test_board(1);
        board.insert_via(1, IntPoint::new(0, 0), vec![1], 1, false);
        board.insert_via(1, IntPoint::new(10_000, 0), vec![1], 1, false);

        // The base request is intentionally much narrower than the default
        // class (100 vs 1500).  The formerly public raw helper inserted the
        // caller width verbatim; the supported single-net API must derive the
        // same per-net request as batch routing.
        let base = request();
        assert_eq!(base.trace_half_width, 100);
        assert_eq!(board.rules.get_trace_half_width_max_active(1), 1_500);
        let result = route_net(&mut board, 1, &base);

        assert_eq!(result.failed_connections, 0);
        assert!(board.items().any(|(_, item)| {
            matches!(
                &item.kind,
                crate::board::ItemKind::PolylineTrace(trace)
                    if item.base.contains_net(1) && trace.half_width == 1_500
            )
        }));
        assert!(!board.items().any(|(_, item)| {
            matches!(
                &item.kind,
                crate::board::ItemKind::PolylineTrace(trace)
                    if item.base.contains_net(1) && trace.half_width == base.trace_half_width
            )
        }));
    }

    #[test]
    fn public_route_net_cannot_land_a_via_on_an_inactive_layer() {
        use crate::rules::{ViaInfo, ViaRule};

        let stack = LayerStructure::new(vec![Layer::new("F.Cu", true), Layer::new("B.Cu", true)]);
        let matrix = ClearanceMatrix::get_default_instance(stack.clone(), 200);
        let mut rules = BoardRules::new(stack.clone(), matrix);
        let class_idx = rules.get_default_net_class();
        rules.set_default_trace_half_widths(100);
        let net_no = rules.nets.add("masked", class_idx, false);
        let shape = TileShape::Box(IntBox::from_coords(-300, -300, 300, 300));
        let mut padstacks = Padstacks::new(2);
        let through = padstacks.add(
            "through",
            vec![Some(shape.clone()), Some(shape.clone())],
            false,
            false,
        );
        let front = padstacks.add("front", vec![Some(shape.clone()), None], false, false);
        let back = padstacks.add("back", vec![None, Some(shape)], false, false);
        let via_info = rules
            .via_infos
            .add(ViaInfo::new("through_info", through, 1, false))
            .unwrap();
        let mut via_rule = ViaRule::new("through_rule");
        via_rule.append_via(via_info);
        rules.via_rules.push(via_rule);
        rules.net_classes.get_mut(class_idx).set_via_rule(Some(0));
        rules
            .net_classes
            .get_mut(class_idx)
            .set_active_routing_layer(1, false);
        let mut board = BasicBoard::new(stack, rules, padstacks);
        let a = board.insert_via(front, IntPoint::new(0, 0), vec![net_no], 1, false);
        let b = board.insert_via(back, IntPoint::new(10_000, 0), vec![net_no], 1, false);
        board.set_component_no(a, 1);
        board.set_component_no(b, 2);
        let before = board.item_count();

        let result = route_net(&mut board, net_no, &request());

        assert_eq!(result.routed_connections, 0);
        assert_eq!(result.failed_connections, 1);
        assert_eq!(board.item_count(), before, "failed route leaked copper");
        assert!(!board.net_is_completely_connected(net_no));
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
            via_clearance_class: 0,
            via_attach_allowed: false,
            via_cost: 5000.0,
            max_expansions: 30_000,
            ripup_penalty: 0.0,
            deadline: None,
        };
        // route net 1 first: it takes the gap
        let r1 = route_net(&mut board, 1, &request);
        assert_eq!(r1.failed_connections, 0);
        assert!(board.net_is_completely_connected(1));

        // A caller may reuse a base request that already carries a ripup
        // cost. The strict public API must still keep its preliminary and
        // victim-recovery phases non-ripping, and it may never return with the
        // previously complete net stranded.
        let base_with_ripup = BatchRequest {
            ripup_penalty: 1.0,
            ..request
        };
        let strict = route_net_with_ripup(&mut board, 2, &base_with_ripup, 20_000.0);
        assert!(board.net_is_completely_connected(1));
        if strict.failed_connections == 0 {
            assert!(board.net_is_completely_connected(2));
        }

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
