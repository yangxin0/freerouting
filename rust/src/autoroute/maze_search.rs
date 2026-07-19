//! Port of the core expansion loop of `MazeSearchAlgo.java` together with
//! simplified `LocateFoundConnectionAlgo`/`InsertFoundConnectionAlgo` and
//! `ExpansionDrill` layer changes.
//!
//! The search is a Dijkstra expansion over door *sections* plus drill
//! steps: each door section can be occupied once, rooms materialize their
//! neighbors lazily on first entry, and layer changes are expanded by
//! drilling at the room entry location where a via fits on all spanned
//! layers (Java generates candidates via DrillPage/DrillPageArray; this
//! port drills at entry locations, documented simplification). Ripup and
//! shove follow later. The found connection is backtracked through the
//! node chain and inserted as per-layer polyline traces joined by vias.

use crate::datastructures::FxHashSet as HashSet;
use std::cmp::Reverse;
use std::collections::BinaryHeap;

thread_local! {
    /// Cumulative search statistics (FR_STATS diagnostics).
    pub static STATS: std::cell::RefCell<SearchStats> =
        std::cell::RefCell::new(SearchStats::default());
}

#[derive(Debug, Default, Clone, Copy)]
pub struct SearchStats {
    pub searches: u64,
    pub expansions: u64,
    pub pushes: u64,
    pub rooms_completed: u64,
}

pub fn take_stats() -> SearchStats {
    STATS.with(|s| std::mem::take(&mut *s.borrow_mut()))
}

use crate::autoroute::engine::AutorouteEngine;
use crate::autoroute::expansion_room::{DoorId, RoomId};
use crate::board::basic_board::{BasicBoard, ItemId};
use crate::geometry::planar::{FloatPoint, IntBox, IntPoint, Polyline, TileShape};

/// One step of the search, kept in an arena for backtracking.
#[derive(Debug, Clone, Copy)]
struct BacktrackNode {
    location: FloatPoint,
    layer: usize,
    parent: Option<usize>,
    /// The room entered at this step (diagnostics).
    room: Option<RoomId>,
    via_choice: Option<ViaChoice>,
}

#[derive(Debug, Clone, Copy, PartialEq)]
enum Step {
    /// Passing through a door section into a room.
    Door { door: DoorId, section: usize },
    /// Drilling to another layer at the entry location.
    Drill,
}

#[derive(Debug, Clone, Copy, PartialEq)]
struct QueueEntry {
    /// The accumulated path cost (g).
    cost: f64,
    /// g plus the estimated remaining distance to the destination
    /// (Java: sorting by cost + DestinationDistance).
    estimate: f64,
    step: Step,
    room_to_enter: RoomId,
    parent: Option<usize>,
    location: FloatPoint,
    via_choice: Option<ViaChoice>,
}

impl Eq for QueueEntry {}
impl PartialOrd for QueueEntry {
    fn partial_cmp(&self, other: &Self) -> Option<std::cmp::Ordering> {
        Some(self.cmp(other))
    }
}
impl Ord for QueueEntry {
    fn cmp(&self, other: &Self) -> std::cmp::Ordering {
        self.estimate
            .partial_cmp(&other.estimate)
            .unwrap_or(std::cmp::Ordering::Equal)
            .then_with(|| self.room_to_enter.cmp(&other.room_to_enter))
    }
}

pub(crate) struct MazeSearchResult {
    /// The corners of the found connection with their layers, from start
    /// to destination. Consecutive corners on different layers are joined
    /// by a via.
    pub corners: Vec<(FloatPoint, usize)>,
    /// The room entered at each corner (diagnostics; aligned with
    /// `corners`, `None` for the appended destination point).
    pub rooms: Vec<Option<RoomId>>,
    /// Via candidate used to reach each corner (only layer-changing corners
    /// carry a value).  Keeping this through backtracking prevents insertion
    /// from silently replacing a blind/buried candidate with the request's
    /// fallback through-via.
    pub(crate) via_choices: Vec<Option<ViaChoice>>,
}

/// Parameters of a maze routing request.
#[derive(Debug, Clone)]
pub(crate) struct MazeRouteRequest {
    pub net_no: i32,
    pub start_item: ItemId,
    pub dest_item: ItemId,
    /// All items of the start/destination connected sets (Java routes
    /// component to component: autoroute_connection takes p_start_set
    /// and p_dest_set). Empty = fall back to the single items above.
    pub start_items: Vec<ItemId>,
    pub dest_items: Vec<ItemId>,
    pub trace_half_width: i32,
    pub clearance_class: usize,
    /// 1-based padstack for layer-change vias.
    pub via_padstack: usize,
    /// The clearance class inserted vias carry (the selected `ViaInfo`'s
    /// class — a via rule may demand stricter spacing than the traces).
    /// 0 = fall back to `clearance_class`.
    pub via_clearance_class: usize,
    /// Vias may land on drillable (SMD) pads of their own net; the
    /// inserted via carries the flag (Java `attach_smd_allowed`).
    pub via_attach_allowed: bool,
    /// Additional cost of a layer change in board units.
    pub via_cost: f64,
    /// Budget for the expansion: the maximum number of queue pops before
    /// the search gives up (Java bounds passes with a TimeLimit instead).
    pub max_expansions: usize,
    /// If > 0, the search may route through rippable foreign route items,
    /// paying this penalty per rippable item in an entered room (Java:
    /// MazeSearchAlgo ripup costs); the items intersecting the inserted
    /// connection are removed.
    pub ripup_penalty: f64,
    /// Optional wall-clock deadline checked periodically during the
    /// expansion.
    pub deadline: Option<crate::datastructures::TimeLimit>,
    /// Fanout mode (Java: AutorouteControl.is_fanout): the search
    /// completes at the FIRST successful drill — the goal of a fanout is
    /// reaching another layer through a via, not a destination item.
    pub is_fanout: bool,
}

/// A concrete ViaInfo candidate for one layer transition.  Via rules are
/// ordered preferences, but the usable candidate depends on the actual
/// `from_layer..to_layer` transition; keeping this value local to the maze
/// avoids collapsing blind/buried and through vias into one board-wide pick.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub(crate) struct ViaChoice {
    padstack: usize,
    clearance_class: usize,
    attach_allowed: bool,
}

/// Search-only permission for placing one concrete ViaRule candidate on a
/// same-net drillable SMD pad.  `MazeRouteRequest::via_attach_allowed` is a
/// summary of the request's fallback/full-span via; a mixed rule can select
/// a different attach-enabled blind or buried candidate for this transition.
/// Pure-SMD escape is an independent relaxation and does not alter the
/// inserted via's declared attach bit.
fn search_attach_allowed_for_choice(choice: ViaChoice, pure_smd_relax: bool) -> bool {
    // `via_attach_allowed` on the request describes only its selected
    // fallback ViaInfo.  A bound rule can expose several candidates with
    // different attach bits; allowing the request-wide bit to leak into a
    // candidate lets an attach-disabled blind/buried via search through an
    // SMD pin and leaves insertion with a route that can never be replayed.
    // Fallback candidates copy their bit into `choice`, so the candidate is
    // the sole rule-level authority here.  Pure-SMD escape is an independent
    // search relaxation and is intentionally retained.
    choice.attach_allowed || pure_smd_relax
}

fn via_choice_supports_transition(
    board: &BasicBoard,
    choice: ViaChoice,
    from_layer: usize,
    to_layer: usize,
) -> bool {
    let Some(padstack) = board.padstacks.get_by_no(choice.padstack) else {
        return false;
    };
    let low = from_layer.min(to_layer);
    let high = from_layer.max(to_layer);
    padstack.from_layer() <= low
        && padstack.to_layer() >= high
        && padstack.has_shapes_at_transition_endpoints(from_layer, to_layer)
}

fn via_choices_for_transition(
    board: &BasicBoard,
    request: &MazeRouteRequest,
    from_layer: usize,
    to_layer: usize,
) -> Vec<ViaChoice> {
    let low = from_layer.min(to_layer);
    let high = from_layer.max(to_layer);
    let mut choices = Vec::new();
    if let Some(net) = board.rules.nets.get_by_no(request.net_no) {
        let class = board.rules.net_classes.get(net.get_class());
        if let Some(rule_id) = class.get_via_rule() {
            if let Some(rule) = board.rules.via_rules.get(rule_id) {
                for &via_id in rule.vias() {
                    let info = board.rules.via_infos.get(via_id);
                    let padstack = info.get_padstack();
                    let Some(ps) = board.padstacks.get_by_no(padstack) else {
                        continue;
                    };
                    let choice = ViaChoice {
                        padstack,
                        clearance_class: if info.get_clearance_class() == 0 {
                            request.clearance_class
                        } else {
                            info.get_clearance_class()
                        },
                        attach_allowed: info.attach_smd_allowed(),
                    };
                    if ps.from_layer() <= low
                        && ps.to_layer() >= high
                        && via_choice_supports_transition(board, choice, from_layer, to_layer)
                    {
                        choices.push(choice);
                    }
                }
            }
        }
    }
    if choices.is_empty() && !board.rules.has_bound_via_rule(request.net_no) {
        if let Some(ps) = board.padstacks.get_by_no(request.via_padstack) {
            let choice = ViaChoice {
                padstack: request.via_padstack,
                clearance_class: request.via_class(),
                attach_allowed: request.via_attach_allowed,
            };
            if ps.from_layer() <= low
                && ps.to_layer() >= high
                && via_choice_supports_transition(board, choice, from_layer, to_layer)
            {
                choices.push(choice);
            }
        }
    }
    choices.dedup();
    choices
}

fn via_choices_for_layer(
    board: &BasicBoard,
    request: &MazeRouteRequest,
    layer: usize,
) -> Vec<ViaChoice> {
    let layer_count = board.layer_structure.layer_count();
    let mut choices = Vec::new();
    // Preserve the ordering of the selected ViaRule across all possible
    // destination layers.  Gathering by destination first changes a rule
    // such as blind, buried, through into a layer-dependent order, which is
    // observably different from Java's ordered ViaInfo preference.
    if let Some(net) = board.rules.nets.get_by_no(request.net_no) {
        let class = board.rules.net_classes.get(net.get_class());
        if let Some(rule_id) = class.get_via_rule() {
            if let Some(rule) = board.rules.via_rules.get(rule_id) {
                for &via_id in rule.vias() {
                    let info = board.rules.via_infos.get(via_id);
                    let padstack = info.get_padstack();
                    let Some(ps) = board.padstacks.get_by_no(padstack) else {
                        continue;
                    };
                    if ps.from_layer() > layer
                        || ps.to_layer() < layer
                        || ps.get_shape(layer).is_none()
                        || !(0..layer_count).any(|target| {
                            target != layer
                                && board.rules.is_active_routing_layer(request.net_no, target)
                                && ps.has_shapes_at_transition_endpoints(layer, target)
                        })
                    {
                        continue;
                    }
                    let choice = ViaChoice {
                        padstack,
                        clearance_class: if info.get_clearance_class() == 0 {
                            request.clearance_class
                        } else {
                            info.get_clearance_class()
                        },
                        attach_allowed: info.attach_smd_allowed(),
                    };
                    if !choices.contains(&choice) {
                        choices.push(choice);
                    }
                }
            }
        }
    }
    if choices.is_empty() && !board.rules.has_bound_via_rule(request.net_no) {
        // Boards without a bound ViaRule retain the request's concrete
        // fallback, but only when its padstack can actually leave this layer.
        if let Some(ps) = board.padstacks.get_by_no(request.via_padstack) {
            if ps.from_layer() <= layer
                && ps.to_layer() >= layer
                && ps.get_shape(layer).is_some()
                && (0..layer_count).any(|target| {
                    target != layer
                        && board.rules.is_active_routing_layer(request.net_no, target)
                        && ps.has_shapes_at_transition_endpoints(layer, target)
                })
            {
                choices.push(ViaChoice {
                    padstack: request.via_padstack,
                    clearance_class: request.via_class(),
                    attach_allowed: request.via_attach_allowed,
                });
            }
        }
    }
    choices
}

impl MazeRouteRequest {
    fn start_set(&self) -> Vec<ItemId> {
        if self.start_items.is_empty() {
            vec![self.start_item]
        } else {
            self.start_items.clone()
        }
    }
    fn dest_set(&self) -> Vec<ItemId> {
        if self.dest_items.is_empty() {
            vec![self.dest_item]
        } else {
            self.dest_items.clone()
        }
    }
    fn is_dest(&self, item: ItemId) -> bool {
        if self.dest_items.is_empty() {
            item == self.dest_item
        } else {
            self.dest_items.contains(&item)
        }
    }
    /// The clearance class inserted vias carry (0 = the trace class).
    fn via_class(&self) -> usize {
        if self.via_clearance_class == 0 {
            self.clearance_class
        } else {
            self.via_clearance_class
        }
    }
}

/// Runs the maze expansion from the start item towards the destination
/// item. Returns the corner list of the found connection.
/// Runs the maze search and corrects both connection endpoints so the route
/// begins and ends exactly at a pin's connection point (the drill center) when
/// it lands inside a pad off-centre. Applied here, on the search result, so
/// every consumer — the inline insertion of `maze_route_with_engine` and the
/// `insert_connection` path alike — sees connection points that reload as
/// genuine connections rather than dangling tracks (finding #2).
pub(crate) fn find_connection(
    board: &BasicBoard,
    engine: &mut AutorouteEngine,
    request: &MazeRouteRequest,
) -> Option<MazeSearchResult> {
    let mut result = find_connection_inner(board, engine, request)?;
    if let Some(&(first, l)) = result.corners.first() {
        if let Some(c) = pin_exit_corner(board, request.net_no, first, l) {
            result.corners.insert(0, (c, l));
            result.rooms.insert(0, None);
            result.via_choices.insert(0, None);
        }
    }
    if let Some(&(last, l)) = result.corners.last() {
        if let Some(c) = pin_exit_corner(board, request.net_no, last, l) {
            result.corners.push((c, l));
            result.rooms.push(None);
            result.via_choices.push(None);
        }
    }
    Some(result)
}

fn find_connection_inner(
    board: &BasicBoard,
    engine: &mut AutorouteEngine,
    request: &MazeRouteRequest,
) -> Option<MazeSearchResult> {
    let mut start_shapes: Vec<(TileShape, usize)> = Vec::new();
    for id in request.start_set() {
        if let Some(item) = board.get_item(id) {
            start_shapes.extend(item.tile_shapes(&board.padstacks).iter().cloned());
        }
    }
    if start_shapes.is_empty() {
        return None;
    }
    // the room geometry already carries the full margin (obstacles are
    // inflated by half width + clearance), so the door shrink only
    // spaces the sections by the trace width
    let offset = request.trace_half_width as f64;
    // destination centers (with their layers) for the remaining-distance
    // estimate. NOTE: distance-to-center is inadmissible for large
    // destinations, but empirically it GUIDES far better than the
    // admissible nearest-bbox-point variant (weighted-A* effect: the
    // bbox estimate regressed interf_u from 112 s/173 to 247 s/172 and
    // coldfire by 2 nets — do not "fix" the admissibility again).
    // When the destination has no shape on the queried layer, one via is
    // unavoidable and its cost joins the estimate.
    let dest_centers: Vec<(FloatPoint, usize)> = request
        .dest_set()
        .iter()
        .filter_map(|id| board.get_item(*id))
        .flat_map(|item| {
            item.tile_shapes(&board.padstacks)
                .iter()
                .map(|(s, l)| (s.centre_of_gravity(), *l))
                .collect::<Vec<_>>()
        })
        .collect();
    let via_cost_for_estimate = request.via_cost;
    let estimate_to_dest = move |p: FloatPoint, layer: usize| -> f64 {
        let dist = dest_centers
            .iter()
            .map(|(d, _)| p.distance(*d))
            .fold(f64::MAX, f64::min)
            .min(1e12);
        // explicit weighting on top (the center distance already behaves
        // like weighted A*; FR_ASTAR_WEIGHT tunes the trade-off)
        let weight = crate::debug::astar_weight();
        if dest_centers.iter().any(|(_, l)| *l == layer) {
            dist * weight
        } else {
            dist * weight + via_cost_for_estimate
        }
    };

    let mut nodes: Vec<BacktrackNode> = Vec::new();
    let mut open: BinaryHeap<Reverse<QueueEntry>> = BinaryHeap::new();
    // A location/layer may be reachable through more than one ViaInfo with
    // the same padstack but different clearance or attach policy.  Keep the
    // complete candidate in the settled-state key so one candidate cannot
    // suppress a later, semantically distinct candidate.
    let mut drilled: HashSet<(i32, i32, usize, ViaChoice)> = HashSet::default();
    // via_free memo: neighbouring room pops re-list the same frontier
    // drill points; the exact 4-layer clearance check ran per pop and
    // reached tens of millions of tree queries per pass
    let mut via_ok: crate::datastructures::FxHashMap<(i32, i32, ViaChoice), bool> =
        crate::datastructures::FxHashMap::default();

    // create and seed the start rooms on every ACTIVE layer of the start
    // item — the net-class use_layer gate applies to planar routing too,
    // not only to drills: two pads on a disabled layer must not route
    // entirely on that layer (Java AutorouteControl.layer_active)
    for (start_shape, layer) in &start_shapes {
        if !board.rules.is_active_routing_layer(request.net_no, *layer) {
            continue;
        }
        let start_center = start_shape.centre_of_gravity();
        let mut start_rooms = engine.create_start_rooms(board, start_shape.clone(), *layer);
        if start_rooms.is_empty() {
            // an earlier layer's expansion may already have completed
            // rooms covering this pad (new rooms must not overlap them);
            // those existing rooms then serve as the start
            start_rooms = engine.rooms_containing(start_center.round(), *layer, board);
        }
        // set FR_DEBUG_MAZE=1 to diagnose instantly failing connections
        if crate::debug::maze() {
            eprintln!(
                "MAZE start item {:?} layer {layer}: {} start rooms",
                request.start_item,
                start_rooms.len()
            );
        }
        for &room in &start_rooms {
            // the start corner must lie inside room ∩ start shape so the
            // first segment cannot leave the room (the pad's centre of
            // gravity may be outside a sliver room — same DRC leak as on
            // the arrival side)
            let room_shape = engine.graph.room(room).shape.clone();
            let start_door = room_shape.intersection(start_shape);
            // Prefer the start pad's connection point (its centre of gravity,
            // = the drill centre for a symmetric pad) when it lies in the room,
            // so the trace BEGINS at the pin connection point — the same
            // electrical-equivalence requirement as the arrival side (#2).
            // Fall back to the door centroid only when the centre is outside the
            // room (else the first segment would cross foreign clearance).
            let sc = crate::geometry::planar::Point::Int(start_center.round());
            let center_in_room =
                room_shape.contains(&sc) || room_shape.to_simplex().offset(2.0).contains(&sc);
            let start_point = if center_in_room {
                start_center
            } else if start_door.dimension() >= 1 {
                start_door.centre_of_gravity()
            } else {
                start_center
            };
            if let Some(t) = engine
                .target_doors(room)
                .iter()
                .find(|t| request.is_dest(t.item))
            {
                let dest_point =
                    destination_point(board, t.item, *layer, Some(&room_shape), start_point);
                // coincident points would insert nothing (stacked pads of
                // one net whose contact never registers): fall through to
                // the search instead of returning a degenerate route
                if dest_point.round() != start_point.round() {
                    return Some(MazeSearchResult {
                        corners: vec![(start_point, *layer), (dest_point, *layer)],
                        rooms: vec![Some(room), None],
                        via_choices: vec![None, None],
                    });
                }
            }
            engine.expand_room(board, room);
            let root = nodes.len();
            nodes.push(BacktrackNode {
                location: start_point,
                layer: *layer,
                parent: None,
                room: Some(room),
                via_choice: None,
            });
            seed_room(
                engine,
                board,
                request,
                room,
                start_point,
                0.0,
                root,
                None,
                offset,
                &mut open,
                &mut drilled,
                &mut via_ok,
                &estimate_to_dest,
            );
        }
    }

    // pre-create rooms around the destination shapes too: in dense pin
    // rows the neighbours' clearance-inflated shapes can kill every free
    // room touching the dest pad, leaving no room with a target door to
    // arrive at. Like start rooms, these keep (a sliver of) the dest
    // shape by the contained-shape privilege and carry its target door.
    for dest_id in request.dest_set() {
        let Some(dest) = board.get_item(dest_id) else {
            continue;
        };
        let dest_shapes: Vec<(TileShape, usize)> = dest.tile_shapes(&board.padstacks).to_vec();
        for (dest_shape, layer) in dest_shapes {
            if !board.rules.is_active_routing_layer(request.net_no, layer) {
                continue;
            }
            engine.create_start_rooms(board, dest_shape, layer);
        }
    }

    STATS.with(|s| s.borrow_mut().searches += 1);
    let mut expansions = 0usize;
    while let Some(Reverse(entry)) = open.pop() {
        STATS.with(|s| s.borrow_mut().expansions += 1);
        expansions += 1;
        if expansions > request.max_expansions {
            return None; // budget exhausted
        }
        if expansions.is_multiple_of(1024) && request.deadline.is_some_and(|t| t.limit_exceeded()) {
            return None; // out of time
        }
        // sections are occupied at push time: every door pop is unique
        if let Step::Door { door, section } = entry.step {
            let sections = &engine.graph.door(door).sections;
            debug_assert!(section < sections.len() && sections[section].is_occupied);
        }
        let room = entry.room_to_enter;
        let layer = engine.graph.room(room).layer;
        let node_id = nodes.len();
        nodes.push(BacktrackNode {
            location: entry.location,
            layer,
            parent: entry.parent,
            room: Some(room),
            via_choice: entry.via_choice,
        });

        // fanout completes at the first drill (Java: MazeSearchAlgo
        // "algorithm completed after the first drill")
        if request.is_fanout && matches!(entry.step, Step::Drill) {
            let mut corners: Vec<(FloatPoint, usize)> = Vec::new();
            let mut rooms: Vec<Option<RoomId>> = Vec::new();
            let mut via_choices: Vec<Option<ViaChoice>> = Vec::new();
            let mut curr = Some(node_id);
            while let Some(i) = curr {
                corners.push((nodes[i].location, nodes[i].layer));
                rooms.push(nodes[i].room);
                via_choices.push(nodes[i].via_choice);
                curr = nodes[i].parent;
            }
            corners.reverse();
            rooms.reverse();
            via_choices.reverse();
            return Some(MazeSearchResult {
                corners,
                rooms,
                via_choices,
            });
        }

        engine.expand_room(board, room);

        let arrival_target = engine
            .target_doors(room)
            .iter()
            .find(|t| request.is_dest(t.item))
            .map(|t| t.item);
        if let Some(arrival_target) = arrival_target {
            // backtrack through the node chain
            let mut corners: Vec<(FloatPoint, usize)> = Vec::new();
            let mut rooms: Vec<Option<RoomId>> = Vec::new();
            let mut via_choices: Vec<Option<ViaChoice>> = Vec::new();
            let mut curr = Some(node_id);
            while let Some(i) = curr {
                corners.push((nodes[i].location, nodes[i].layer));
                rooms.push(nodes[i].room);
                via_choices.push(nodes[i].via_choice);
                curr = nodes[i].parent;
            }
            corners.reverse();
            rooms.reverse();
            via_choices.reverse();
            let arrival_shape = engine.graph.room(room).shape.clone();
            let dest_point = destination_point(
                board,
                arrival_target,
                layer,
                Some(&arrival_shape),
                entry.location,
            );
            corners.push((dest_point, layer));
            rooms.push(None);
            via_choices.push(None);
            return Some(MazeSearchResult {
                corners,
                rooms,
                via_choices,
            });
        }

        seed_room(
            engine,
            board,
            request,
            room,
            entry.location,
            entry.cost,
            node_id,
            match entry.step {
                Step::Door { door, .. } => Some(door),
                Step::Drill => None,
            },
            offset,
            &mut open,
            &mut drilled,
            &mut via_ok,
            &estimate_to_dest,
        );
    }
    if crate::debug::maze() {
        let rooms_with_dest_door = (0..engine.graph.room_count())
            .filter(|&r| {
                engine
                    .target_doors(r)
                    .iter()
                    .any(|t| request.is_dest(t.item))
            })
            .count();
        let dest_kind = board
            .get_item(request.dest_item)
            .map(|i| match i.kind {
                crate::board::ItemKind::Via(_) => "via/pad",
                crate::board::ItemKind::PolylineTrace(_) => "trace",
                crate::board::ItemKind::ObstacleArea(_) => "area",
            })
            .unwrap_or("missing");
        eprintln!(
            "MAZE exhausted after {expansions} expansions \
             (dest {dest_kind}, rooms_with_dest_door={rooms_with_dest_door})"
        );
    }
    None
}

/// Pushes the door sections and drill steps reachable from `room`.
#[allow(clippy::too_many_arguments)]
fn seed_room(
    engine: &mut AutorouteEngine,
    board: &BasicBoard,
    request: &MazeRouteRequest,
    room: RoomId,
    location: FloatPoint,
    base_cost: f64,
    parent: usize,
    entered_through: Option<DoorId>,
    offset: f64,
    open: &mut BinaryHeap<Reverse<QueueEntry>>,
    drilled: &mut HashSet<(i32, i32, usize, ViaChoice)>,
    via_ok: &mut crate::datastructures::FxHashMap<(i32, i32, ViaChoice), bool>,
    estimate_to_dest: &dyn Fn(FloatPoint, usize) -> f64,
) {
    let layer = engine.graph.room(room).layer;
    // door expansions
    let doors = engine.graph.room(room).doors.clone();
    for door in doors {
        if entered_through == Some(door) {
            continue;
        }
        let Some(other) = engine.graph.other_room(door, room) else {
            continue;
        };
        // obstacle rooms are enterable only when ripup is allowed
        let other_item = engine.obstacle_room_item(other);
        if request.ripup_penalty <= 0.0 && other_item.is_some() {
            continue;
        }
        // Java's ALREADY_RIPPED_COSTS: moving between obstacle rooms of
        // the SAME item (consecutive segments of one trace) is free —
        // the rip was charged at first entry
        let already_ripped = other_item.is_some() && other_item == engine.obstacle_room_item(room);
        let segments = engine.graph.door_section_segments(door, offset);
        // shovable traces cost a fraction of the rip penalty (Java:
        // MazeShoveTraceAlgo passages carry no ripup cost; the corridor
        // shove at insert slides them aside)
        let shove_discount = if request.ripup_penalty > 0.0
            && engine.obstacle_room_shovable(
                board,
                other,
                request.trace_half_width,
                request.clearance_class,
            ) {
            0.25
        } else {
            1.0
        };
        let section_count = segments.len();
        for (section, seg) in segments.iter().enumerate() {
            let midpoint = seg.a.middle_point(seg.b);
            // Java's section_ok (MazeShoveTraceAlgo.check_shove_trace_line):
            // a lateral slide is only possible entering through the FIRST
            // or LAST door section — interior sections would need the
            // trace to pass through the entry point
            let section_discount =
                if shove_discount < 1.0 && (section == 0 || section + 1 == section_count) {
                    shove_discount
                } else {
                    1.0
                };
            let ripup_cost = if already_ripped {
                0.0
            } else {
                request.ripup_penalty * engine.rippable_items(other).len() as f64 * section_discount
            };
            let cost = base_cost + location.distance(midpoint) + ripup_cost;
            // occupy ON PUSH (Java: expand_to_door_section sets is_occupied
            // when the element is inserted): each section enters the queue
            // exactly once, from the cheapest frontier element known at that
            // time.
            //
            // NOTE: Java actually settles the section at POP, which is
            // theoretically more optimal (an expensive first discovery can
            // block a cheaper later path here). That correct variant was tried
            // — competing candidates queued, cheapest settled at pop, with a
            // strictly-cheaper prune to bound re-pushes — but occupy-on-push is
            // load-bearing as a frontier pruner, not just a micro-optimization:
            // removing it exploded the search on J2 from ~125 ms to ~21 s
            // (168x; NormalPuzzle earlier hit 69M pushes). The optimality gap
            // is small in practice while the slowdown is catastrophic, so the
            // occupy-on-push tradeoff is retained deliberately.
            match engine.graph.door_mut(door).sections.get_mut(section) {
                Some(s) if !s.is_occupied => {
                    s.is_occupied = true;
                    s.best_cost = cost;
                }
                _ => continue,
            }
            let other_layer = engine.graph.room(other).layer;
            STATS.with(|s| s.borrow_mut().pushes += 1);
            open.push(Reverse(QueueEntry {
                cost,
                estimate: cost + estimate_to_dest(midpoint, other_layer),
                step: Step::Door { door, section },
                room_to_enter: other,
                parent: Some(parent),
                location: midpoint,
                via_choice: None,
            }));
        }
    }
    // drill expansion (Java: ExpansionDrill candidates from DrillPages):
    // try the entry location plus a grid of sample points within the room
    let pure_smd_relax = crate::autoroute::batch::pure_smd_search_relaxation(board, request.net_no);
    for choice in via_choices_for_layer(board, request, layer) {
        // The selected full-span ViaInfo on the request is only a fallback
        // summary.  A ViaRule may offer an attach-enabled blind/buried via
        // alongside an attach-disabled through via, so drill-page and
        // pairwise-site search must use the concrete candidate's bit.  The
        // pure-SMD relaxation remains an independent search-only permission.
        let search_attach_relax = search_attach_allowed_for_choice(choice, pure_smd_relax);
        let Some(padstack) = board.padstacks.get_by_no(choice.padstack) else {
            continue;
        };
        let from = padstack.from_layer();
        let to = padstack.to_layer();
        if layer < from || layer > to || padstack.get_shape(layer).is_none() {
            continue;
        }
        let mut drill_points: Vec<IntPoint> = vec![location.round()];
        {
            // drill pages (Java: DrillPageArray): cached convex free areas;
            // candidates are the page drills whose free area intersects the
            // current room
            let room_shape = engine.graph.room(room).shape.clone();
            let bb = room_shape.bounding_box();
            let via_margin = (from..=to)
                .filter_map(|l| padstack.get_shape(l))
                .map(|s| (s.bounding_box().max_width() / 2.0) as i32)
                .max()
                .unwrap_or(1000)
                + (from..=to)
                    .map(|l| board.rules.clearance_matrix.max_value(l).max(0))
                    .max()
                    .unwrap_or(0)
                + board.rules.max_same_net_clearance().max(0);
            let pages = engine
                .drill_pages
                .entry(choice.padstack)
                .or_insert_with(|| {
                    crate::autoroute::drill_pages::DrillPageArray::new(board, choice.padstack)
                });
            pages.sync_board_changes(board);
            for drill in pages.drills_overlapping(
                board,
                &bb,
                request.net_no,
                via_margin,
                search_attach_relax,
            ) {
                if drill_points.len() >= 17 {
                    break;
                }
                if room_shape.contains(&crate::geometry::planar::Point::Int(drill.location)) {
                    drill_points.push(drill.location);
                }
            }
        }
        let mut candidate_request = request.clone();
        candidate_request.via_padstack = choice.padstack;
        candidate_request.via_clearance_class = choice.clearance_class;
        candidate_request.via_attach_allowed = choice.attach_allowed;
        for drill_point in drill_points {
            let free = *via_ok
                .entry((drill_point.x, drill_point.y, choice))
                .or_insert_with(|| {
                    via_free(board, &candidate_request, drill_point, search_attach_relax)
                });
            if !free {
                continue;
            }
            let drill_cost = base_cost + location.distance(drill_point.to_float());
            for next_layer in from..=to {
                if next_layer == layer {
                    continue;
                }
                if !padstack.has_shapes_at_transition_endpoints(layer, next_layer) {
                    continue;
                }
                // net-class active-layer gate (Java AutorouteControl.layer_active
                // from `(circuit (use_layer ...))`): the via may still span the
                // disabled layer, but the search never routes onto it
                if !board
                    .rules
                    .is_active_routing_layer(request.net_no, next_layer)
                {
                    continue;
                }
                if !drilled.insert((drill_point.x, drill_point.y, next_layer, choice)) {
                    continue;
                }
                // find or create the room on the target layer containing the
                // point
                let target_rooms = engine.rooms_containing(drill_point, next_layer, board);
                for target_room in target_rooms {
                    let ripup_cost =
                        request.ripup_penalty * engine.rippable_items(target_room).len() as f64;
                    let cost = drill_cost + request.via_cost + ripup_cost;
                    open.push(Reverse(QueueEntry {
                        cost,
                        estimate: cost + estimate_to_dest(drill_point.to_float(), next_layer),
                        step: Step::Drill,
                        room_to_enter: target_room,
                        parent: Some(parent),
                        location: drill_point.to_float(),
                        via_choice: Some(choice),
                    }));
                }
            }
        }
    }
}

/// The clearance a via of `request` must keep to `other` on `layer`, or
/// `None` when `other` does not constrain the via site. Mirrors the DRC's
/// same-net drill rule (Java `Via`/`Pin.is_obstacle`): same-net traces and
/// areas never constrain; same-net pins and vias do — unless the via may
/// attach and the pin is a drillable SMD pad — at the `*_same_net` rule
/// value when one exists, the ordinary matrix value otherwise. The search
/// (`via_free`) and the insert gate (`via_site_is_clear`) share this rule,
/// so the maze never picks a site the insert then rejects.
fn via_site_clearance_with_attach(
    board: &BasicBoard,
    request: &MazeRouteRequest,
    other: &crate::board::Item,
    layer: usize,
    attach_allowed: bool,
    escape_smd_layer: Option<usize>,
) -> Option<f64> {
    let mut same_net_required: Option<f64> = None;
    if other.base.contains_net(request.net_no) {
        if !crate::drc::is_drill(&other.kind) {
            return None;
        }
        if (attach_allowed || escape_smd_layer == Some(layer))
            && crate::drc::is_pin(other)
            && crate::drc::drill_allowed(other, &board.padstacks)
        {
            return None;
        }
        let other_class = if !crate::drc::is_pin(other) {
            crate::rules::ItemClass::Via
        } else if crate::drc::drill_allowed(other, &board.padstacks) {
            crate::rules::ItemClass::Smd
        } else {
            crate::rules::ItemClass::Pin
        };
        same_net_required = board
            .rules
            .get_same_net_clearance(crate::rules::ItemClass::Via, other_class)
            .map(|v| v as f64);
    } else if let crate::board::ItemKind::ObstacleArea(a) = &other.kind {
        if a.is_conduction && !a.is_obstacle {
            return None;
        }
    }
    Some(same_net_required.unwrap_or_else(|| {
        // The pending via receives a higher ID than every existing item.
        // Preserve Java's final-DRC order `(existing, new)`; the via's class
        // comes from the selected ViaInfo and may be stricter than the trace.
        crate::drc::clearance_for_new_item(board, other, request.via_class(), layer)
    }))
}

/// True if a via at `point` keeps its clearance on all layers it spans.
fn via_free(
    board: &BasicBoard,
    request: &MazeRouteRequest,
    point: IntPoint,
    search_attach_allowed: bool,
) -> bool {
    let Some(padstack) = board.padstacks.get_by_no(request.via_padstack) else {
        return false;
    };
    if !padstack.has_shapes_at_transition_endpoints(padstack.from_layer(), padstack.to_layer()) {
        return false;
    }
    for layer in padstack.from_layer()..=padstack.to_layer() {
        let Some(shape) = padstack.get_shape(layer) else {
            continue;
        };
        // the via pad must keep the pairwise clearance to every foreign
        // item on every spanned layer (a max-clearance check falsely
        // seals tight pockets)
        let via_shape =
            shape.translate_by(crate::geometry::planar::IntVector::new(point.x, point.y));
        // the query must also reach `*_same_net` rule distances, which live
        // outside the matrix and can exceed its maximum — otherwise a larger
        // same-net drill rule is never discovered here
        let max_cl = board
            .rules
            .clearance_matrix
            .max_value(layer)
            .max(board.rules.max_same_net_clearance())
            .max(0) as f64;
        let query = via_shape.offset(max_cl);
        for id in board.overlapping_items(&query, Some(layer)) {
            let Some(item) = board.get_item(id) else {
                continue;
            };
            // with ripup, rippable (foreign) items do not block drills:
            // the via insertion rips whatever its footprint overlaps
            if request.ripup_penalty > 0.0
                && crate::autoroute::room_completion::is_rippable(item, request.net_no)
            {
                continue;
            }
            let Some(pairwise) = via_site_clearance_with_attach(
                board,
                request,
                item,
                layer,
                search_attach_allowed,
                None,
            ) else {
                continue;
            };
            let check = via_shape.offset(pairwise);
            let conflicts = item
                .tile_shapes(&board.padstacks)
                .iter()
                .any(|(s, l)| *l == layer && s.intersection(&check).dimension() >= 2);
            if conflicts {
                return false;
            }
        }
    }
    true
}

/// Models the legal exit of a connection through a pin: if `corner` (an
/// endpoint of a routed connection) lies inside a same-net drill's pad on this
/// layer but off its connection point, returns that connection point (the drill
/// center). The segment corner→center stays inside the convex same-net pad, so
/// it crosses no foreign clearance — this is NOT a generic stub but the pin exit
/// itself, the part of the route the arrival room had to exclude for clearance.
/// Landing exactly on the connection point is what makes the routed net
/// electrically equivalent on SES reload (finding #2).
fn pin_exit_corner(
    board: &BasicBoard,
    net_no: i32,
    corner: FloatPoint,
    layer: usize,
) -> Option<FloatPoint> {
    let cp = corner.round();
    let query = TileShape::Box(IntBox::from_coords(cp.x - 1, cp.y - 1, cp.x + 1, cp.y + 1));
    for oid in board.overlapping_items(&query, Some(layer)) {
        let Some(item) = board.get_item(oid) else {
            continue;
        };
        if !item.base.contains_net(net_no) {
            continue;
        }
        let crate::board::ItemKind::Via(v) = &item.kind else {
            continue;
        };
        if v.center == cp {
            return None; // already at the connection point
        }
        let in_pad = item
            .tile_shapes(&board.padstacks)
            .iter()
            .any(|(s, l)| *l == layer && s.contains(&crate::geometry::planar::Point::Int(cp)));
        if in_pad {
            return Some(v.center.to_float());
        }
    }
    None
}

/// The point where the connection enters the destination item. The
/// straight segment from the arrival location to this point must stay
/// legal: when the arrival room is known, the point is taken inside
/// room ∩ dest shape, so the segment never leaves the (convex) room.
/// (Using the dest shape's centre of gravity let the final segment
/// cross foreign clearance zones — the last DRC leak.)
fn destination_point(
    board: &BasicBoard,
    dest_item: ItemId,
    layer: usize,
    arrival_room: Option<&TileShape>,
    fallback: FloatPoint,
) -> FloatPoint {
    board
        .get_item(dest_item)
        .and_then(|item| {
            if matches!(item.kind, crate::board::ItemKind::ObstacleArea(_)) {
                return Some(fallback);
            }
            // arriving at a TRACE: land exactly on its centerline lattice
            // so the junction split (and thus the contact) registers
            if let crate::board::ItemKind::PolylineTrace(t) = &item.kind {
                let anchor = arrival_room
                    .map(|room| {
                        let door = room.intersection(
                            item.tile_shapes(&board.padstacks)
                                .iter()
                                .find(|(_, l)| *l == layer)
                                .map(|(s, _)| s)
                                .unwrap_or(&room.clone()),
                        );
                        if door.dimension() >= 1 {
                            door.centre_of_gravity()
                        } else {
                            fallback
                        }
                    })
                    .unwrap_or(fallback);
                let tap = t.polyline.nearest_lattice_point(anchor)?;
                // the tap (and thus the final segment) must stay in the
                // arrival room; a far tap crosses whatever lies between
                if let Some(room) = arrival_room {
                    let pt = crate::geometry::planar::Point::Int(tap);
                    if !room.contains(&pt) && !room.to_simplex().offset(2.0).contains(&pt) {
                        return Some(fallback);
                    }
                }
                return Some(tap.to_float());
            }
            // arriving at a DRILL item (via/pin): prefer its connection point,
            // the drill center, so the trace terminates exactly where Java lands
            // it. Landing at the door centroid (room ∩ pad) instead left the end
            // ~50 um off a small SMD pin, which the lenient in-pad containment
            // rule still counts as connected but a reloaded SES reads as a
            // dangling track (finding #2). Only taken when the center lies in the
            // arrival room and the pad on this layer, so the final segment stays
            // inside the convex room and does not cross foreign clearance;
            // otherwise fall back to the door centroid.
            if let crate::board::ItemKind::Via(v) = &item.kind {
                let center = crate::geometry::planar::Point::Int(v.center);
                let center_in_room = arrival_room.is_none_or(|room| {
                    room.contains(&center) || room.to_simplex().offset(2.0).contains(&center)
                });
                let center_in_pad = item
                    .tile_shapes(&board.padstacks)
                    .iter()
                    .any(|(s, l)| *l == layer && s.contains(&center));
                if center_in_room && center_in_pad {
                    return Some(v.center.to_float());
                }
            }
            let shapes = item.tile_shapes(&board.padstacks);
            let dest_shape = shapes
                .iter()
                .find(|(_, l)| *l == layer)
                .or_else(|| shapes.first())
                .map(|(s, _)| s)?;
            if let Some(room) = arrival_room {
                let door = room.intersection(dest_shape);
                if door.dimension() >= 1 {
                    return Some(door.centre_of_gravity());
                }
            }
            Some(dest_shape.centre_of_gravity())
        })
        .unwrap_or(fallback)
}

/// A successfully inserted connection.
pub(crate) struct RoutedConnection {
    /// The nets of the rippable items removed to make room (empty without
    /// ripup).
    pub ripped_nets: Vec<i32>,
}

/// Runs the maze search and inserts the found connection as per-layer
/// polyline traces joined by vias, ripping the rippable foreign items the
/// connection passes through when `ripup_penalty` > 0. Returns the
/// inserted item ids and the ripped nets.
pub(crate) fn maze_route_with_ripup(
    board: &mut BasicBoard,
    request: &MazeRouteRequest,
) -> Option<RoutedConnection> {
    let allow_ripup = request.ripup_penalty > 0.0;
    let mut engine = AutorouteEngine::new_with_clearance_synced(
        board,
        request.net_no,
        allow_ripup,
        request.clearance_class,
        request.trace_half_width,
    );
    if !allow_ripup {
        return maze_route_with_engine(board, &mut engine, request);
    }
    // with ripup, victims are shoved/removed BEFORE the fallible insertion:
    // make each connection atomic here so a failed insert never leaves
    // rips behind. This path builds a fresh engine per call, so its cache is
    // not reused after either the incremental commit or a rollback.
    board.generate_snapshot();
    match maze_route_with_engine(board, &mut engine, request) {
        Some(connection) => {
            board.pop_snapshot();
            Some(connection)
        }
        None => {
            board.rollback_snapshot();
            None
        }
    }
}

/// Like [`maze_route_with_ripup`], but reusing a caller-owned engine: the
/// expansion-room graph stays valid across the connections of one net
/// (own-net items never restrain rooms; items ripped in between only make
/// the kept rooms conservative). The engine must be fresh whenever the
/// board changes outside this net's routing (e.g. after an undo).
pub(crate) fn maze_route_with_engine(
    board: &mut BasicBoard,
    engine: &mut AutorouteEngine,
    request: &MazeRouteRequest,
) -> Option<RoutedConnection> {
    let allow_ripup = request.ripup_penalty > 0.0;
    // clear the occupation state of the previous search
    engine.graph.reset();
    let result = find_connection(board, engine, request)?;
    // invariant check (diagnostics): the segment between consecutive
    // same-layer corners must lie inside the room entered at the FIRST
    // corner (convex ⇒ checking both endpoints suffices)
    if crate::debug::maze() {
        for k in 0..result.corners.len().saturating_sub(1) {
            let (a, la) = result.corners[k];
            let (b, lb) = result.corners[k + 1];
            if crate::debug::path() {
                eprintln!(
                    "PATH net {} corner {k} ({:.0},{:.0}) layer {la} room {:?}",
                    request.net_no, a.x, a.y, result.rooms[k]
                );
            }
            // the segment a→b always runs on corner a's layer inside the
            // room entered at a: when b is a drill node (lb != la), the
            // travel to the drill point still happens on la and the via
            // sits at b. (Skipping la != lb pairs hid the drill-segment
            // class of illegal inserts entirely.)
            let _ = lb;
            let Some(room) = result.rooms[k] else {
                eprintln!(
                    "INVARIANT SKIP net {} corner {k}: no room for segment \
                     ({:.0},{:.0})→({:.0},{:.0}) layer {la}",
                    request.net_no, a.x, a.y, b.x, b.y
                );
                continue;
            };
            let shape = &engine.graph.room(room).shape;
            let pa = crate::geometry::planar::Point::Int(a.round());
            let pb = crate::geometry::planar::Point::Int(b.round());
            // small tolerance: corners live on borders
            let ok = |p: &crate::geometry::planar::Point| {
                shape.contains(p) || shape.to_simplex().offset(2.0).contains(p)
            };
            if !ok(&pa) || !ok(&pb) {
                eprintln!(
                    "INVARIANT BROKEN net {} at corner {k}: segment ({:?})→({:?}) \
                     layer {la} room {room} (bbox {:?}, layer {}) a-in {} b-in {} \
                     layers ({} -> {})",
                    request.net_no,
                    a,
                    b,
                    shape.bounding_box(),
                    engine.graph.room(room).layer,
                    ok(&pa),
                    ok(&pb),
                    result.corners[k].1,
                    result.corners[k + 1].1,
                );
            } else {
                // cross-check: the room contains the segment, so the
                // segment must be clear of foreign items; if not, the
                // ROOM itself overlaps an obstacle
                let (ra, rb) = (a.round(), b.round());
                if ra != rb {
                    if let Some(seg) =
                        Polyline::from_two_points(ra, rb).offset_shape(request.trace_half_width, 0)
                    {
                        let cl = board
                            .rules
                            .clearance_matrix
                            .get_value(request.clearance_class, request.clearance_class, la, false)
                            .max(0) as f64;
                        let check = seg.offset(cl - 2.0);
                        for id in board.overlapping_items(&check, Some(la)) {
                            let Some(item) = board.get_item(id) else {
                                continue;
                            };
                            if item.base.contains_net(request.net_no) {
                                continue;
                            }
                            if let crate::board::ItemKind::ObstacleArea(ar) = &item.kind {
                                if ar.is_conduction && !ar.is_obstacle {
                                    continue;
                                }
                            }
                            if !item
                                .tile_shapes(&board.padstacks)
                                .iter()
                                .any(|(s, l)| *l == la && s.intersection(&check).dimension() >= 2)
                            {
                                continue;
                            }
                            eprintln!(
                                "ROOM LEAK net {} corner {k} room {room} bbox {:?} \
                                 contains segment ({},{})→({},{}) layer {la} \
                                 but item {id} (nets {:?}, birth {}, bbox {:?}) blocks",
                                request.net_no,
                                shape.bounding_box(),
                                ra.x,
                                ra.y,
                                rb.x,
                                rb.y,
                                item.base.net_nos,
                                item.base.birth,
                                item.bounding_box(&board.padstacks),
                            );
                            // decisive probe: recomplete the room's shape
                            // against the live board; a still-dirty result
                            // means a live collection/restrain bug, a clean
                            // one means the room predates this obstacle
                            let re = crate::autoroute::room_completion::complete_shape_with_ripup(
                                board,
                                &crate::autoroute::room_completion::IncompleteRoom {
                                    shape: engine.graph.room(room).shape.clone(),
                                    layer: la,
                                    contained_shape: engine.graph.room(room).shape.clone(),
                                },
                                request.net_no,
                                None,
                                false,
                                request.clearance_class,
                                request.trace_half_width,
                            );
                            let cl_m = cl + request.trace_half_width as f64 - 2.0;
                            let still = re.iter().any(|p| {
                                item.tile_shapes(&board.padstacks).iter().any(|(s, l)| {
                                    *l == la
                                        && s.offset(cl_m).intersection(&p.shape).dimension() >= 2
                                })
                            });
                            eprintln!("  RECOMPLETE pieces {} still-dirty {still}", re.len());
                            if !still {
                                // stale-looking: capture exact shapes for
                                // offline reproduction
                                let margin = request.trace_half_width
                                    + crate::drc::clearance_for_new_item(
                                        board,
                                        item,
                                        request.clearance_class,
                                        la,
                                    ) as i32
                                    + crate::rules::clearance_matrix::CLEARANCE_SAFETY_MARGIN;
                                eprintln!(
                                    "  LEAKGEOM room {:?} margin {margin} item-shapes {:?}",
                                    engine.graph.room(room).shape.to_simplex(),
                                    item.tile_shapes(&board.padstacks)
                                        .iter()
                                        .filter(|(_, l)| *l == la)
                                        .map(|(s, _)| s.to_simplex())
                                        .collect::<Vec<_>>(),
                                );
                            }
                        }
                    }
                }
            }
        }
    }

    // room-vs-board consistency audit (expensive, FR_AUDIT_ROOMS): every
    // completed room must be clear of every foreign item's inflated shape
    if crate::debug::audit_rooms() && !allow_ripup {
        for &room_id in engine.complete_rooms() {
            let r = engine.graph.room(room_id);
            let room_bbox = r.shape.bounding_box();
            let query = TileShape::Box(room_bbox).offset(8000.0);
            for id in board.overlapping_items(&query, Some(r.layer)) {
                let Some(item) = board.get_item(id) else {
                    continue;
                };
                if item.base.contains_net(request.net_no) {
                    continue;
                }
                if let crate::board::ItemKind::ObstacleArea(ar) = &item.kind {
                    if ar.is_conduction && !ar.is_obstacle {
                        continue;
                    }
                }
                let cl = crate::drc::clearance_for_new_item(
                    board,
                    item,
                    request.clearance_class,
                    r.layer,
                );
                for (os, ol) in item.tile_shapes(&board.padstacks) {
                    if *ol != r.layer {
                        continue;
                    }
                    let infl = os.offset(request.trace_half_width as f64 + cl - 2.0);
                    if infl.intersection(&r.shape).dimension() >= 2 {
                        eprintln!(
                            "DIRTY ROOM net {} room {room_id} layer {} bbox {:?} \
                             overlaps item {id} (nets {:?}, birth {}) shape-bbox {:?} \
                             cl {cl} hw {} isect-bbox {:?}\n  ROOM-GEOM {:?}\n  OBST-GEOM {:?}",
                            request.net_no,
                            r.layer,
                            room_bbox,
                            item.base.net_nos,
                            item.base.birth,
                            os.bounding_box(),
                            request.trace_half_width,
                            infl.intersection(&r.shape).bounding_box(),
                            r.shape.to_simplex(),
                            os.to_simplex(),
                        );
                        // decisive probe: re-complete this exact shape now;
                        // if the piece survives overlapping, the collection
                        // or restrain bug reproduces deterministically
                        let recompleted =
                            crate::autoroute::room_completion::complete_shape_with_ripup(
                                board,
                                &crate::autoroute::room_completion::IncompleteRoom {
                                    shape: r.shape.clone(),
                                    layer: r.layer,
                                    contained_shape: r.shape.clone(),
                                },
                                request.net_no,
                                None,
                                false,
                                request.clearance_class,
                                request.trace_half_width,
                            );
                        let still_dirty = recompleted
                            .iter()
                            .any(|p| infl.intersection(&p.shape).dimension() >= 2);
                        eprintln!(
                            "  RECOMPLETE pieces {} still-dirty {}",
                            recompleted.len(),
                            still_dirty
                        );
                    }
                }
            }
        }
    }

    // with ripup: remove the rippable foreign items intersecting the
    // connection geometry before inserting it
    let mut ripped_nets: Vec<i32> = Vec::new();
    if allow_ripup {
        // the pending connection's own shapes (not yet on the board):
        // shove substitutes must avoid them
        let mut forbidden: Vec<(TileShape, usize)> = Vec::new();
        for (window_index, window) in result.corners.windows(2).enumerate() {
            let ((a, layer_a), (b, layer_b)) = (window[0], window[1]);
            let (pa, pb) = (a.round(), b.round());
            // the travel pa→pb always runs on layer_a (when b is a drill
            // node the via sits at PB) — same semantics as the insert
            if pa != pb {
                let max_cl = board.rules.clearance_matrix.max_value(layer_a).max(0);
                if let Some(shape) = Polyline::from_two_points(pa, pb)
                    .offset_shape(request.trace_half_width + max_cl + 1, 0)
                {
                    forbidden.push((shape, layer_a));
                }
            }
            if layer_a != layer_b {
                let choice = result
                    .via_choices
                    .get(window_index + 1)
                    .and_then(|c| *c)
                    .or_else(|| {
                        via_choices_for_transition(board, request, layer_a, layer_b)
                            .into_iter()
                            .next()
                    });
                if let Some(choice) = choice {
                    if let Some(padstack) = board.padstacks.get_by_no(choice.padstack) {
                        for layer in padstack.from_layer()..=padstack.to_layer() {
                            if let Some(shape) = padstack.get_shape(layer) {
                                let max_cl =
                                    board.rules.clearance_matrix.max_value(layer).max(0) as f64;
                                forbidden.push((
                                    shape
                                        .translate_by(crate::geometry::planar::IntVector::new(
                                            pb.x, pb.y,
                                        ))
                                        .offset(max_cl + 1.0),
                                    layer,
                                ));
                            }
                        }
                    }
                }
            }
        }
        let mut to_rip: Vec<ItemId> = Vec::new();
        // Java's exact rip set: the items whose obstacle rooms the path
        // traversed
        for room in result.rooms.iter().flatten() {
            if let Some(item_id) = engine.obstacle_room_item(*room) {
                // only rip what is actually rippable: obstacle rooms are
                // now created only for rippable items (engine.rs), but
                // guard here too so a component pin or fixed item can
                // never be deleted, matching the corridor/via rip paths
                if board.get_item(item_id).is_some_and(|item| {
                    crate::autoroute::room_completion::is_rippable(item, request.net_no)
                }) {
                    to_rip.push(item_id);
                }
            }
        }
        for (window_index, window) in result.corners.windows(2).enumerate() {
            let ((a, layer_a), (b, layer_b)) = (window[0], window[1]);
            let (pa, pb) = (a.round(), b.round());
            // rip the travel corridor on layer_a for EVERY pair: when b
            // is a drill node the segment pa→pb still runs on layer_a
            // (skipping la≠lb pairs left the pre-via travel unripped —
            // the residual rip-window violation class)
            if pa != pb {
                let polyline = Polyline::from_two_points(pa, pb);
                // rip everything within CLEARANCE of the new copper, not
                // only what touches it (leaving clearance-range items in
                // place was a DRC leak)
                let max_cl = board.rules.clearance_matrix.max_value(layer_a).max(0);
                if let Some(shape) = polyline.offset_shape(request.trace_half_width + max_cl + 1, 0)
                {
                    // prefer shoving the corridor segment's trace victims
                    // aside (they stay connected, no victim reroute
                    // needed); whatever remains rippable afterwards
                    // (vias, refused traces) is ripped as before
                    let _ = crate::board::shove_aside(
                        board,
                        &shape,
                        layer_a,
                        &[request.net_no],
                        request.clearance_class,
                        &forbidden,
                    );
                    for id in board.overlapping_items(&shape, Some(layer_a)) {
                        if board.get_item(id).is_some_and(|item| {
                            crate::autoroute::room_completion::is_rippable(item, request.net_no)
                        }) {
                            to_rip.push(id);
                        }
                    }
                }
            }
            if layer_a != layer_b {
                // the via footprint at the layer change (the via sits at
                // the DRILL node pb, not at the corner before it)
                let choice = result
                    .via_choices
                    .get(window_index + 1)
                    .and_then(|c| *c)
                    .or_else(|| {
                        via_choices_for_transition(board, request, layer_a, layer_b)
                            .into_iter()
                            .next()
                    });
                if let Some(choice) = choice {
                    if let Some(padstack) = board.padstacks.get_by_no(choice.padstack) {
                        for layer in padstack.from_layer()..=padstack.to_layer() {
                            if let Some(shape) = padstack.get_shape(layer) {
                                let max_cl =
                                    board.rules.clearance_matrix.max_value(layer).max(0) as f64;
                                let q = shape
                                    .translate_by(crate::geometry::planar::IntVector::new(
                                        pb.x, pb.y,
                                    ))
                                    .offset(max_cl + 1.0);
                                for id in board.overlapping_items(&q, Some(layer)) {
                                    if board.get_item(id).is_some_and(|item| {
                                        crate::autoroute::room_completion::is_rippable(
                                            item,
                                            request.net_no,
                                        )
                                    }) {
                                        to_rip.push(id);
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
        to_rip.sort();
        to_rip.dedup();
        for id in to_rip {
            if let Some(item) = board.get_item(id) {
                ripped_nets.extend(item.base.net_nos.iter().copied());
            }
            board.remove_item(id);
        }
        ripped_nets.sort();
        ripped_nets.dedup();
    }

    let restriction = board.rules.get_trace_angle_restriction();
    let restricted = restrict_corners(engine, &result, restriction);
    let new_items = insert_connection(board, request, &restricted)?;
    // rooms are reused across a net's connections: make the new items
    // reachable as destinations
    engine.register_new_targets(board, &new_items);
    Some(RoutedConnection { ripped_nets })
}

/// Runs the maze search and inserts the found connection as per-layer
/// polyline traces joined by vias. Returns the inserted item ids.
#[cfg(test)]
pub(crate) fn maze_route(
    board: &mut BasicBoard,
    request: &MazeRouteRequest,
) -> Option<Vec<ItemId>> {
    let mut engine = AutorouteEngine::new_with_clearance_synced(
        board,
        request.net_no,
        false,
        request.clearance_class,
        request.trace_half_width,
    );
    let result = find_connection(board, &mut engine, request)?;
    // the same angle normalization as the engine path: inserting the raw
    // search corners bypassed the board's 45/90-degree restriction
    let restricted = restrict_corners(&engine, &result, board.rules.get_trace_angle_restriction());
    insert_connection(board, request, &restricted)
}

/// The intermediate corner making from→to compliant with the angle
/// restriction (Java: `LocateFoundConnectionAlgo.calculate_additional_corner`
/// with `ninety_degree_corner` / `fortyfive_degree_corner`).
fn calculate_additional_corner(
    from: FloatPoint,
    to: FloatPoint,
    horizontal_first: bool,
    restriction: crate::board::AngleRestriction,
) -> FloatPoint {
    use crate::board::AngleRestriction::*;
    match restriction {
        None => to,
        NinetyDegree => {
            if horizontal_first {
                FloatPoint::new(to.x, from.y)
            } else {
                FloatPoint::new(from.x, to.y)
            }
        }
        FortyfiveDegree => {
            let abs_dx = (to.x - from.x).abs();
            let abs_dy = (to.y - from.y).abs();
            if abs_dx <= abs_dy {
                if horizontal_first {
                    let y = if to.y >= from.y {
                        from.y + abs_dx
                    } else {
                        from.y - abs_dx
                    };
                    FloatPoint::new(to.x, y)
                } else {
                    let y = if to.y > from.y {
                        to.y - abs_dx
                    } else {
                        to.y + abs_dx
                    };
                    FloatPoint::new(from.x, y)
                }
            } else if horizontal_first {
                let x = if to.x > from.x {
                    to.x - abs_dy
                } else {
                    to.x + abs_dy
                };
                FloatPoint::new(x, from.y)
            } else {
                let x = if to.x > from.x {
                    from.x + abs_dy
                } else {
                    from.x - abs_dy
                };
                FloatPoint::new(x, to.y)
            }
        }
    }
}

/// True if from→to satisfies the restriction (axis-parallel for 90°,
/// axis-parallel or diagonal for 45°).
pub(crate) fn segment_is_compliant(
    from: FloatPoint,
    to: FloatPoint,
    restriction: crate::board::AngleRestriction,
) -> bool {
    use crate::board::AngleRestriction::*;
    let dx = (to.x - from.x).round();
    let dy = (to.y - from.y).round();
    match restriction {
        None => true,
        NinetyDegree => dx == 0.0 || dy == 0.0,
        FortyfiveDegree => dx == 0.0 || dy == 0.0 || dx.abs() == dy.abs(),
    }
}

/// Rewrites the found corners so every segment satisfies the angle
/// restriction, inserting intermediate corners chosen to stay inside the
/// segment's room when possible (Java:
/// `LocateFoundConnectionAlgo45Degree.calculate_next_trace_corners`).
fn restrict_corners(
    engine: &AutorouteEngine,
    result: &MazeSearchResult,
    restriction: crate::board::AngleRestriction,
) -> MazeSearchResult {
    if restriction == crate::board::AngleRestriction::None {
        return MazeSearchResult {
            corners: result.corners.clone(),
            rooms: result.rooms.clone(),
            via_choices: result.via_choices.clone(),
        };
    }
    let mut corners: Vec<(FloatPoint, usize)> = Vec::new();
    let mut rooms: Vec<Option<RoomId>> = Vec::new();
    let mut via_choices: Vec<Option<ViaChoice>> = Vec::new();
    for k in 0..result.corners.len() {
        let (b, lb) = result.corners[k];
        if let Some(&(a, la)) = corners.last().filter(|_| k > 0) {
            // The travel a→b always runs on layer `la` — when b is a drill
            // node (lb != la) the segment is the APPROACH to the via,
            // emitted on the old layer by the insert. Restricting only
            // same-layer pairs let noncompliant approach segments through.
            if !segment_is_compliant(a, b, restriction) {
                // choose horizontal_first so the extra corner stays in
                // the segment's room (try true, then false, like Java)
                let room = result.rooms[k - 1].or(result.rooms[k]);
                let mut extra = calculate_additional_corner(a, b, true, restriction);
                if let Some(r) = room {
                    let shape = &engine.graph.room(r).shape;
                    let inside = |p: FloatPoint| {
                        let ip = crate::geometry::planar::Point::Int(p.round());
                        shape.contains(&ip) || shape.to_simplex().offset(2.0).contains(&ip)
                    };
                    if !inside(extra) {
                        let alt = calculate_additional_corner(a, b, false, restriction);
                        if inside(alt) {
                            extra = alt;
                        }
                    }
                }
                let rounded = extra.round().to_float();
                if rounded != a && rounded != b {
                    // the extra corner belongs to the approach segment's
                    // layer `la`, so the run stays on the old layer until
                    // the drill point itself
                    corners.push((rounded, la));
                    rooms.push(result.rooms[k - 1]);
                    via_choices.push(None);
                }
            }
        }
        corners.push((b, lb));
        rooms.push(result.rooms[k]);
        via_choices.push(result.via_choices[k]);
    }
    MazeSearchResult {
        corners,
        rooms,
        via_choices,
    }
}

/// Inserts the found connection as per-layer polyline traces joined by
/// vias and normalizes the junctions.
/// True when a via of the request's padstack at `p` keeps the exact
/// pairwise clearance to every foreign item (mitered pre-filter +
/// Euclidean confirm, like the DRC).
#[cfg(test)]
fn via_site_is_clear(board: &BasicBoard, request: &MazeRouteRequest, p: IntPoint) -> bool {
    via_site_is_clear_with_escape(board, request, p, None)
}

fn via_site_is_clear_with_escape(
    board: &BasicBoard,
    request: &MazeRouteRequest,
    p: IntPoint,
    escape_smd_layer: Option<usize>,
) -> bool {
    let Some(ps) = board.padstacks.get_by_no(request.via_padstack) else {
        // A missing padstack is malformed board state, not an empty via.
        // Failing closed prevents the insert path from treating an invalid
        // candidate as clearance-free.
        return false;
    };
    if !ps.has_shapes_at_transition_endpoints(ps.from_layer(), ps.to_layer()) {
        return false;
    }
    let matrix = &board.rules.clearance_matrix;
    for layer in ps.from_layer()..=ps.to_layer() {
        let Some(shape) = ps.get_shape(layer) else {
            continue;
        };
        let shape = shape.translate_by(crate::geometry::planar::IntVector::new(p.x, p.y));
        // reach `*_same_net` rule distances too (they can exceed the matrix max)
        let max_cl = matrix
            .max_value(layer)
            .max(board.rules.max_same_net_clearance())
            .max(0) as f64;
        for other_id in board.overlapping_items(&shape.offset(max_cl), Some(layer)) {
            let Some(other) = board.get_item(other_id) else {
                continue;
            };
            let Some(cl) = via_site_clearance_with_attach(
                board,
                request,
                other,
                layer,
                request.via_attach_allowed,
                escape_smd_layer,
            ) else {
                continue;
            };
            if other.tile_shapes(&board.padstacks).iter().any(|(os, ol)| {
                *ol == layer
                    && os.intersection(&shape.offset(cl)).dimension() >= 2
                    && crate::drc::violates(shape.euclidean_distance_to(os), cl)
            }) {
                return false;
            }
        }
    }
    true
}

fn via_site_is_clear_for_choice(
    board: &BasicBoard,
    request: &MazeRouteRequest,
    p: IntPoint,
    choice: ViaChoice,
    escape_smd_layer: Option<usize>,
) -> bool {
    let mut candidate = request.clone();
    candidate.via_padstack = choice.padstack;
    candidate.via_clearance_class = choice.clearance_class;
    candidate.via_attach_allowed = choice.attach_allowed;
    via_site_is_clear_with_escape(board, &candidate, p, escape_smd_layer)
}

/// Returns the one SMD layer that justifies a pure-SMD escape via at `p`.
/// The selected ViaInfo must have attach disabled, the net must consist of
/// original single-layer component pins, and the pending via copper must
/// actually overlap one of those pins. A single marker cannot represent two
/// different SMD layers, so an ambiguous stacked-pad site is rejected.
fn escape_smd_layer_for_choice(
    board: &BasicBoard,
    request: &MazeRouteRequest,
    p: IntPoint,
    choice: ViaChoice,
) -> Option<usize> {
    if choice.attach_allowed
        || !crate::autoroute::batch::pure_smd_search_relaxation(board, request.net_no)
    {
        return None;
    }
    board.pure_smd_escape_layer(request.net_no, choice.padstack, p)
}

/// True when a trace run of the request's width keeps the exact
/// pairwise clearance to every foreign item on `layer` (mitered
/// pre-filter + Euclidean confirm, like the via-site gate).
fn trace_run_is_clear(
    board: &BasicBoard,
    request: &MazeRouteRequest,
    polyline: &Polyline,
    layer: usize,
) -> bool {
    let matrix = &board.rules.clearance_matrix;
    let max_cl = matrix.max_value(layer).max(0) as f64;
    for seg in polyline.offset_shapes(request.trace_half_width) {
        for other_id in board.overlapping_items(&seg.offset(max_cl), Some(layer)) {
            let Some(other) = board.get_item(other_id) else {
                continue;
            };
            if other.base.contains_net(request.net_no) {
                continue;
            }
            if let crate::board::ItemKind::ObstacleArea(a) = &other.kind {
                if (a.is_conduction && !a.is_obstacle) || a.via_only {
                    continue;
                }
            }
            // The pending trace receives a higher ID than `other`; use the
            // same `(existing, new)` cell as the final DRC.
            let cl =
                crate::drc::clearance_for_new_item(board, other, request.clearance_class, layer);
            if other.tile_shapes(&board.padstacks).iter().any(|(os, ol)| {
                *ol == layer
                    && os.intersection(&seg.offset(cl)).dimension() >= 2
                    && crate::drc::violates(seg.euclidean_distance_to(os), cl)
            }) {
                if crate::debug::maze() {
                    let d = other
                        .tile_shapes(&board.padstacks)
                        .iter()
                        .filter(|(_, ol)| *ol == layer)
                        .map(|(os, _)| seg.euclidean_distance_to(os))
                        .fold(f64::INFINITY, f64::min);
                    eprintln!(
                        "RUN CONFLICT net {} layer {layer} vs item {other_id} \
                         (nets {:?}): d {d:.1} < cl {cl}",
                        request.net_no, other.base.net_nos
                    );
                }
                return false;
            }
        }
    }
    true
}

/// Inserts one found connection as an atomic board transaction.  The maze
/// insertion may invoke forced-via and shove helpers, which can modify items
/// that predate this call.  A birth-ID cleanup is insufficient for those
/// victims; the board snapshot covers every nested mutation and restores the
/// exact pre-call state on any late failure.
fn insert_connection(
    board: &mut BasicBoard,
    request: &MazeRouteRequest,
    result: &MazeSearchResult,
) -> Option<Vec<ItemId>> {
    board.generate_snapshot();
    let watermark = board.begin_lineage_drc_transaction();
    let inserted = insert_connection_inner(board, request, result);
    match inserted {
        Some(items) => {
            if board.finish_lineage_drc_transaction(watermark) {
                board.pop_snapshot();
                Some(items)
            } else {
                board.rollback_snapshot();
                None
            }
        }
        None => {
            board.discard_lineage_drc_transaction(watermark);
            board.rollback_snapshot();
            None
        }
    }
}

fn insert_connection_inner(
    board: &mut BasicBoard,
    request: &MazeRouteRequest,
    result: &MazeSearchResult,
) -> Option<Vec<ItemId>> {
    let _birth_tag = crate::board::basic_board::birth_tag_scope(1);

    let mut new_items = Vec::new();
    // Correct the two endpoints so the connection begins and ends exactly at the
    // pin connection point when it lands inside a pad off-centre. Doing it here,
    // on the final corner list, covers every path that produced it (the direct
    // same-room route, the general backtrack, both start and dest sides) — the
    // arrival/departure rooms exclude the pad centre for clearance, so the pin
    // exit must be modelled explicitly. The added segment stays inside the
    // convex same-net pad, so `trace_run_is_clear` (which skips same-net items)
    // always accepts it.
    let mut corners = result.corners.clone();
    let mut via_choices = result.via_choices.clone();
    if crate::debug::maze() {
        eprintln!("FOUND net {} corners {:?}", request.net_no, corners);
    }
    if let Some(&(first, l)) = corners.first() {
        if let Some(c) = pin_exit_corner(board, request.net_no, first, l) {
            corners.insert(0, (c, l));
            via_choices.insert(0, None);
        }
    }
    if let Some(&(last, l)) = corners.last() {
        if let Some(c) = pin_exit_corner(board, request.net_no, last, l) {
            corners.push((c, l));
            via_choices.push(None);
        }
    }
    let mut run: Vec<IntPoint> = Vec::new();
    let mut run_layer = corners.first()?.1;
    let flush =
        |board: &mut BasicBoard,
         run: &mut Vec<IntPoint>,
         layer: usize,
         items: &mut Vec<ItemId>|
         -> bool {
            run.dedup();
            if run.len() > 1 {
                let polyline = Polyline::from_int_points(run);
                // A run that folds back on itself (e.g. [a, b, a]) collapses
                // to fewer than two corners: nothing to insert.
                if polyline.is_empty() {
                    if crate::debug::maze() {
                        eprintln!(
                            "DEGENERATE RUN skipped net {} layer {layer}: {run:?}",
                            request.net_no
                        );
                    }
                    return true;
                }
                // corridor runs bypass room geometry where foreign items
                // were ripped or shovable: a conflicted run first shoves
                // the offenders aside (traces and vias), and fails the
                // insert when even that cannot clear it (coldfire: maze
                // traces 998 from pre-existing vias at required 1500)
                if !trace_run_is_clear(board, request, &polyline, layer) {
                    let max_cl = board.rules.clearance_matrix.max_value(layer).max(0) as f64;
                    for seg in polyline.offset_shapes(request.trace_half_width) {
                        let _ = crate::board::shove_trace_algo::shove_aside(
                            board,
                            &seg.offset(max_cl),
                            layer,
                            &[request.net_no],
                            request.clearance_class,
                            &[],
                        );
                    }
                    if !trace_run_is_clear(board, request, &polyline, layer) {
                        if crate::debug::maze() {
                            eprintln!("RUN BLOCKED net {} layer {layer}: {run:?}", request.net_no);
                        }
                        return false;
                    }
                }
                // birth-site validation: the post-rip board must leave
                // every inserted segment its full clearance
                if crate::debug::maze() {
                    for seg in polyline.offset_shapes(request.trace_half_width) {
                        let cl = board
                            .rules
                            .clearance_matrix
                            .get_value(
                                request.clearance_class,
                                request.clearance_class,
                                layer,
                                false,
                            )
                            .max(0) as f64;
                        let check = seg.offset(cl - 2.0);
                        for id in board.overlapping_items(&check, Some(layer)) {
                            let Some(item) = board.get_item(id) else {
                                continue;
                            };
                            if item.base.contains_net(request.net_no) {
                                continue;
                            }
                            if let crate::board::ItemKind::ObstacleArea(a) = &item.kind {
                                if a.is_conduction && !a.is_obstacle {
                                    continue;
                                }
                            }
                            // exact: only report 2D overlaps (the tree
                            // query also returns boundary touches)
                            if !item.tile_shapes(&board.padstacks).iter().any(|(s, l)| {
                                *l == layer && s.intersection(&check).dimension() >= 2
                            }) {
                                continue;
                            }
                            eprintln!(
                                "ILLEGAL INSERT net {} layer {layer} ripup={} \
                                 blocked by item {id} (nets {:?}, birth {}) run {:?}",
                                request.net_no,
                                request.ripup_penalty > 0.0,
                                item.base.net_nos,
                                item.base.birth,
                                run
                            );
                        }
                    }
                }
                let new_id = board.insert_trace(
                    polyline,
                    layer,
                    request.trace_half_width,
                    vec![request.net_no],
                    request.clearance_class,
                );
                if crate::debug::maze() {
                    eprintln!(
                        "INSERTED trace {new_id} net {} layer {layer} run {:?}",
                        request.net_no, run
                    );
                }
                items.push(new_id);
            }
            true
        };
    for (corner_index, (corner, layer)) in corners.iter().enumerate() {
        let p = corner.round();
        if *layer != run_layer {
            // the drill NODE's location is the via site: the travel from
            // the previous corner to the drill point happened on the OLD
            // layer, so finish that run at p before switching. (Putting
            // the via at the previous corner instead moved that segment
            // to the new layer where it was never searched — the display
            // illegal-insert class, exposed by grid-sampled drills.)
            if run.last() != Some(&p) {
                run.push(p);
            }
            if !flush(board, &mut run, run_layer, &mut new_items) {
                return None;
            }
            // Drill-page-validated sites are inserted plainly (the vast
            // majority). Rip-corridor sites bypass the drill-page
            // exclusion and could land too close to surviving foreign
            // vias (coldfire: 1004 apart at required 1500) — a conflicted
            // site goes through Java's forced-via path instead (checked,
            // shoves conflicting items free) and fails the insert when
            // even that cannot clear it.
            let mut choices = Vec::new();
            if let Some(choice) = via_choices
                .get(corner_index)
                .and_then(|c| *c)
                .filter(|choice| via_choice_supports_transition(board, *choice, run_layer, *layer))
            {
                choices.push(choice);
            }
            for choice in via_choices_for_transition(board, request, run_layer, *layer) {
                if !choices.contains(&choice) {
                    choices.push(choice);
                }
            }
            if choices.is_empty() {
                return None;
            }
            let mut inserted_via = None;
            for choice in choices {
                if !via_choice_supports_transition(board, choice, run_layer, *layer) {
                    continue;
                }
                // The via carries the selected ViaInfo's clearance class,
                // not the trace request's.  Try later rule candidates when
                // an earlier candidate is blocked at this concrete site.
                let escape_smd_layer = escape_smd_layer_for_choice(board, request, p, choice);
                if via_site_is_clear_for_choice(board, request, p, choice, escape_smd_layer) {
                    inserted_via = Some(if let Some(layer) = escape_smd_layer {
                        board.insert_escape_via(
                            choice.padstack,
                            p,
                            vec![request.net_no],
                            choice.clearance_class,
                            choice.attach_allowed,
                            layer,
                        )
                    } else {
                        board.insert_via(
                            choice.padstack,
                            p,
                            vec![request.net_no],
                            choice.clearance_class,
                            choice.attach_allowed,
                        )
                    });
                    break;
                }
                if let Some(id) = crate::board::forced_via::insert_forced_via_with_escape(
                    board,
                    choice.padstack,
                    p,
                    &[request.net_no],
                    choice.clearance_class,
                    request.trace_half_width,
                    crate::board::forced_via::ViaInsertionPolicy {
                        attach_allowed: choice.attach_allowed,
                        escape_smd_layer,
                    },
                ) {
                    inserted_via = Some(id);
                    break;
                }
            }
            if let Some(id) = inserted_via {
                new_items.push(id);
            } else {
                if crate::debug::maze() {
                    eprintln!("VIA SITE BLOCKED net {} at {p:?}", request.net_no);
                }
                return None;
            }
            run = vec![p];
            run_layer = *layer;
        }
        if run.last() != Some(&p) {
            run.push(p);
        }
    }
    if !flush(board, &mut run, run_layer, &mut new_items) {
        return None;
    }
    if new_items.is_empty() {
        // nothing was inserted (degenerate/coincident corners): reporting
        // success would let the caller loop forever on "routed"
        // connections that change nothing
        return None;
    }

    // normalize junctions: if an inserted trace endpoint lands in the
    // middle of an existing trace of the net, split that trace so the
    // contact registers (Java: PolylineTrace normalization on insert)
    let endpoints: Vec<(IntPoint, usize)> = new_items
        .iter()
        .filter_map(|id| board.get_item(*id).cloned())
        .filter_map(|item| match item.kind {
            crate::board::ItemKind::PolylineTrace(t) => Some(t),
            _ => None,
        })
        .flat_map(|t| {
            [
                (t.first_corner().to_float().round(), t.layer),
                (t.last_corner().to_float().round(), t.layer),
            ]
        })
        .collect();
    for (point, layer) in endpoints {
        board.split_traces_at(point, layer, request.net_no);
    }

    // also split a same-net trace that runs *underneath* a newly inserted via
    // (no trace endpoint at the via center), on every layer the via spans, so
    // the via registers a contact with the pass-through trace (Java:
    // insert_via calls split_traces across the via's spanned layers). Without
    // this, a via dropped on a pass-through trace leaves the net's connectivity
    // unregistered and completion is over-counted.
    let via_splits: Vec<(IntPoint, usize)> = new_items
        .iter()
        .filter_map(|id| board.get_item(*id).cloned())
        .filter_map(|item| match item.kind {
            crate::board::ItemKind::Via(v) => Some(v),
            _ => None,
        })
        .flat_map(|v| {
            let (from, to) = board
                .padstacks
                .get_by_no(v.padstack)
                .map(|p| (p.from_layer(), p.to_layer()))
                .unwrap_or((0, 0));
            (from..=to).map(move |l| (v.center, l)).collect::<Vec<_>>()
        })
        .collect();
    for (point, layer) in via_splits {
        // split_traces_at cuts one matching trace per call; loop so every
        // same-net trace passing under the via center is split (bounded by the
        // trace count, so a false return terminates it).
        while board.split_traces_at(point, layer, request.net_no) {}
    }

    Some(new_items)
}

#[allow(dead_code)]
fn point_shape(p: IntPoint) -> TileShape {
    TileShape::Box(IntBox::new(p, p))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::board::{Layer, LayerStructure};
    use crate::core::Padstacks;
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

    fn request(start: ItemId, dest: ItemId) -> MazeRouteRequest {
        MazeRouteRequest {
            via_clearance_class: 0,
            via_attach_allowed: false,
            net_no: 1,
            start_items: Vec::new(),
            dest_items: Vec::new(),
            is_fanout: false,
            start_item: start,
            dest_item: dest,
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
    fn trivial_connection_in_one_room() {
        let mut board = test_board();
        let a = board.insert_via(1, IntPoint::new(0, 0), vec![1], 1, false);
        let b = board.insert_via(1, IntPoint::new(5000, 0), vec![1], 1, false);
        let items = maze_route(&mut board, &request(a, b)).expect("route failed");
        assert!(!items.is_empty());
        assert!(board.net_is_completely_connected(1));
    }

    #[test]
    fn missing_via_padstack_is_not_clear() {
        let board = test_board();
        let mut req = request(0, 0);
        req.via_padstack = usize::MAX;
        assert!(
            !via_site_is_clear(&board, &req, IntPoint::new(0, 0)),
            "a malformed via candidate must fail closed"
        );
    }

    #[test]
    fn failed_insert_restores_victim_changed_by_nested_shove() {
        let mut board = test_board();
        let victim_id = board.insert_trace(
            Polyline::from_int_points(&[IntPoint::new(-10_000, 0), IntPoint::new(10_000, 0)]),
            0,
            100,
            vec![2],
            1,
        );
        let victim_before = board.get_item(victim_id).cloned().unwrap();
        let ids_before: Vec<_> = board.items().map(|(id, _)| *id).collect();

        // The first run intersects the victim and successfully shoves it
        // aside.  The subsequent layer transition has no valid padstack, so
        // insertion fails after that nested mutation.
        let result = MazeSearchResult {
            corners: vec![
                (FloatPoint::new(-1_000.0, 0.0), 0),
                (FloatPoint::new(1_000.0, 0.0), 0),
                (FloatPoint::new(1_000.0, 0.0), 1),
            ],
            rooms: vec![None, None, None],
            via_choices: vec![None, None, None],
        };
        let mut req = request(0, 0);
        req.via_padstack = usize::MAX;
        assert!(
            insert_connection(&mut board, &req, &result).is_none(),
            "the malformed later via must fail"
        );

        assert_eq!(board.get_item(victim_id), Some(&victim_before));
        let ids_after: Vec<_> = board.items().map(|(id, _)| *id).collect();
        assert_eq!(ids_after, ids_before, "failed insertion leaked new items");
    }

    #[test]
    fn connection_through_door_graph_around_wall() {
        let mut board = test_board();
        let a = board.insert_via(1, IntPoint::new(0, 0), vec![1], 1, false);
        let b = board.insert_via(1, IntPoint::new(9000, 0), vec![1], 1, false);
        // foreign-net wall between them on BOTH layers, gap below y = -2000
        for layer in 0..2 {
            board.insert_trace(
                Polyline::from_int_points(&[IntPoint::new(4500, -2000), IntPoint::new(4500, 9000)]),
                layer,
                300,
                vec![2],
                1,
            );
        }
        maze_route(&mut board, &request(a, b)).expect("route failed");
        assert!(board.net_is_completely_connected(1));
        // the route detours below the wall on some layer
        let mut detoured = false;
        for (_, item) in board.items() {
            if let crate::board::ItemKind::PolylineTrace(t) = &item.kind {
                if !item.base.contains_net(1) {
                    continue;
                }
                detoured |= t.polyline.corner_approx_arr().windows(2).any(|w| {
                    (w[0].x <= 4500.0 && w[1].x >= 4500.0) && (w[0].y + w[1].y) / 2.0 < -1500.0
                });
            }
        }
        assert!(detoured, "trace did not detour below the wall");
    }

    #[test]
    fn layer_change_through_via() {
        let mut board = test_board();
        let a = board.insert_via(1, IntPoint::new(0, 0), vec![1], 1, false);
        let b = board.insert_via(1, IntPoint::new(9000, 0), vec![1], 1, false);
        // an impassable wall on layer 0 only
        board.insert_trace(
            Polyline::from_int_points(&[IntPoint::new(4500, -50000), IntPoint::new(4500, 50000)]),
            0,
            300,
            vec![2],
            1,
        );
        let items = maze_route(&mut board, &request(a, b)).expect("route failed");
        assert!(board.net_is_completely_connected(1));
        // Both pads span both layers, so the router may route entirely on
        // layer 1; but if the route uses layer 0 it must contain a via.
        // Verify the inserted geometry avoids the wall on layer 0.
        for id in &items {
            if let Some(crate::board::ItemKind::PolylineTrace(t)) =
                board.get_item(*id).map(|i| &i.kind)
            {
                if t.layer == 0 {
                    for w in t.polyline.corner_approx_arr().windows(2) {
                        assert!(
                            !(w[0].x < 4500.0 && w[1].x > 4500.0),
                            "layer-0 trace crosses the wall"
                        );
                    }
                }
            }
        }
    }

    #[test]
    fn drill_approach_segments_honor_the_angle_restriction() {
        // the travel to a drill node runs on the OLD layer; restricting
        // only same-layer corner pairs let noncompliant approaches through
        let engine = AutorouteEngine::new_with_clearance(1, false, 1, 100);
        let result = MazeSearchResult {
            corners: vec![
                (FloatPoint::new(0.0, 0.0), 0),
                // drill node: layer changes 0 -> 1, approach (0,0)->(100,37)
                // is neither axis-parallel nor diagonal
                (FloatPoint::new(100.0, 37.0), 1),
                (FloatPoint::new(100.0, 500.0), 1),
            ],
            rooms: vec![None, None, None],
            via_choices: vec![None, None, None],
        };
        let restricted = restrict_corners(
            &engine,
            &result,
            crate::board::AngleRestriction::FortyfiveDegree,
        );
        for w in restricted.corners.windows(2) {
            let ((a, la), (b, _)) = (w[0], w[1]);
            assert!(
                segment_is_compliant(a, b, crate::board::AngleRestriction::FortyfiveDegree),
                "segment {a:?} -> {b:?} on layer {la} violates the restriction"
            );
        }
        // the inserted corner belongs to the approach segment's OLD layer
        let extra = restricted
            .corners
            .iter()
            .find(|(p, _)| {
                *p != FloatPoint::new(0.0, 0.0)
                    && p.y != 500.0
                    && p.round() != IntPoint::new(100, 37)
            })
            .expect("an extra corner must be inserted for the approach");
        assert_eq!(extra.1, 0, "the extra corner runs on the old layer");
    }

    #[test]
    fn via_rule_candidates_are_selected_per_layer_transition() {
        use crate::board::{Layer, LayerStructure};
        use crate::core::Padstacks;
        use crate::rules::{BoardRules, ClearanceMatrix, ViaInfo, ViaRule};
        let stack =
            LayerStructure::new((0..4).map(|i| Layer::new(format!("L{i}"), true)).collect());
        let matrix = ClearanceMatrix::get_default_instance(stack.clone(), 200);
        let mut rules = BoardRules::new(stack.clone(), matrix);
        rules.get_default_net_class();
        let net = rules.nets.add("N", 1, false);
        let mut pads = Padstacks::new(4);
        let shape = TileShape::Box(IntBox::from_coords(-100, -100, 100, 100));
        let blind = pads.add_shape_on_layers(shape.clone(), 0, 1);
        let buried = pads.add_shape_on_layers(shape.clone(), 1, 2);
        let through = pads.add_shape_on_layers(shape, 0, 3);
        let blind_info = rules
            .via_infos
            .add(ViaInfo::new("blind", blind, 1, false))
            .unwrap();
        let buried_info = rules
            .via_infos
            .add(ViaInfo::new("buried", buried, 1, false))
            .unwrap();
        let through_info = rules
            .via_infos
            .add(ViaInfo::new("through", through, 1, false))
            .unwrap();
        let mut rule = ViaRule::new("mixed");
        rule.append_via(blind_info);
        rule.append_via(buried_info);
        rule.append_via(through_info);
        rules.via_rules.push(rule);
        rules.net_classes.get_mut(0).set_via_rule(Some(0));
        let board = BasicBoard::new(stack, rules, pads);
        let request = MazeRouteRequest {
            net_no: net,
            start_item: 0,
            dest_item: 0,
            start_items: Vec::new(),
            dest_items: Vec::new(),
            trace_half_width: 50,
            clearance_class: 1,
            via_padstack: through,
            via_clearance_class: 1,
            via_attach_allowed: false,
            via_cost: 100.0,
            max_expansions: 10,
            ripup_penalty: 0.0,
            deadline: None,
            is_fanout: false,
        };
        let a = via_choices_for_transition(&board, &request, 0, 1);
        assert_eq!(a.first().map(|v| v.padstack), Some(blind));
        let b = via_choices_for_transition(&board, &request, 1, 2);
        assert_eq!(b.first().map(|v| v.padstack), Some(buried));
        assert!(via_choices_for_transition(&board, &request, 0, 3)
            .iter()
            .any(|v| v.padstack == through));
        let layer_one = via_choices_for_layer(&board, &request, 1);
        assert_eq!(
            layer_one
                .iter()
                .map(|choice| choice.padstack)
                .collect::<Vec<_>>(),
            vec![blind, buried, through],
            "all destination layers must retain ViaRule priority"
        );
    }

    #[test]
    fn mixed_via_rule_uses_each_candidates_attach_permission_during_search() {
        // The request's fallback is an attach-disabled through via, while
        // the rule's first candidate is an attach-enabled blind via.  The
        // blind candidate is legal on the same-net SMD pin; the through
        // candidate is not.  This is the distinction lost when drill-page
        // search used only request.via_attach_allowed for every candidate.
        use crate::rules::{ViaInfo, ViaRule};

        let stack =
            LayerStructure::new((0..3).map(|i| Layer::new(format!("L{i}"), true)).collect());
        let matrix = ClearanceMatrix::get_default_instance(stack.clone(), 200);
        let mut rules = BoardRules::new(stack.clone(), matrix);
        let class_idx = rules.get_default_net_class();
        let net_no = rules.nets.add("N", class_idx, false);
        let shape = TileShape::Box(IntBox::from_coords(-300, -300, 300, 300));
        let mut padstacks = Padstacks::new(3);
        let blind = padstacks.add(
            "blind",
            vec![Some(shape.clone()), Some(shape.clone()), None],
            false,
            false,
        );
        let through = padstacks.add(
            "through",
            vec![
                Some(shape.clone()),
                Some(shape.clone()),
                Some(shape.clone()),
            ],
            false,
            false,
        );
        let smd = padstacks.add("smd", vec![Some(shape), None, None], false, false);
        let blind_info = rules
            .via_infos
            .add(ViaInfo::new("blind_info", blind, 1, true))
            .unwrap();
        let through_info = rules
            .via_infos
            .add(ViaInfo::new("through_info", through, 1, false))
            .unwrap();
        let mut rule = ViaRule::new("mixed_attach");
        rule.append_via(blind_info);
        rule.append_via(through_info);
        rules.via_rules.push(rule);
        rules.net_classes.get_mut(class_idx).set_via_rule(Some(0));
        let mut board = BasicBoard::new(stack, rules, padstacks);
        let pin = board.insert_via(smd, IntPoint::new(0, 0), vec![net_no], 1, false);
        board.set_component_no(pin, 1);

        let request = MazeRouteRequest {
            net_no,
            start_item: pin,
            dest_item: pin,
            start_items: Vec::new(),
            dest_items: Vec::new(),
            trace_half_width: 100,
            clearance_class: 1,
            via_padstack: through,
            via_clearance_class: 1,
            via_attach_allowed: false,
            via_cost: 100.0,
            max_expansions: 100,
            ripup_penalty: 0.0,
            deadline: None,
            is_fanout: false,
        };
        let blind_choice = ViaChoice {
            padstack: blind,
            clearance_class: 1,
            attach_allowed: true,
        };
        let through_choice = ViaChoice {
            padstack: through,
            clearance_class: 1,
            attach_allowed: false,
        };
        let mut blind_request = request.clone();
        blind_request.via_padstack = blind;
        blind_request.via_clearance_class = blind_choice.clearance_class;
        blind_request.via_attach_allowed = blind_choice.attach_allowed;
        // Simulate a request whose selected full-span fallback permits SMD
        // attachment.  That summary must not leak into the rule's
        // attach-disabled through candidate below.
        let mut through_request = request;
        through_request.via_padstack = through;
        through_request.via_clearance_class = through_choice.clearance_class;
        through_request.via_attach_allowed = true;

        assert!(via_free(
            &board,
            &blind_request,
            IntPoint::new(0, 0),
            search_attach_allowed_for_choice(blind_choice, false),
        ));
        assert!(!via_free(
            &board,
            &through_request,
            IntPoint::new(0, 0),
            search_attach_allowed_for_choice(through_choice, false),
        ));
        // The rule itself exposes both candidates, despite the request's
        // attach-disabled full-span fallback.
        let choices = via_choices_for_layer(&board, &blind_request, 0);
        assert_eq!(choices, vec![blind_choice, through_choice]);
    }

    #[test]
    fn duplicate_padstack_via_infos_keep_distinct_attach_semantics() {
        use crate::rules::{ViaInfo, ViaRule};

        let stack = LayerStructure::new(vec![Layer::new("F.Cu", true), Layer::new("B.Cu", true)]);
        let matrix = ClearanceMatrix::get_default_instance(stack.clone(), 200);
        let mut rules = BoardRules::new(stack.clone(), matrix);
        assert!(rules.clearance_matrix.append_class("strict_via"));
        let strict_via = rules.clearance_matrix.get_no("strict_via").unwrap();
        let class_idx = rules.get_default_net_class();
        let net_no = rules.nets.add("N", class_idx, false);
        let shape = TileShape::Box(IntBox::from_coords(-300, -300, 300, 300));
        let mut padstacks = Padstacks::new(2);
        let through = padstacks.add(
            "through",
            vec![Some(shape.clone()), Some(shape.clone())],
            false,
            false,
        );
        let smd = padstacks.add("smd", vec![Some(shape), None], false, false);
        let attach_off = rules
            .via_infos
            .add(ViaInfo::new("through_off", through, 1, false))
            .unwrap();
        let attach_on = rules
            .via_infos
            .add(ViaInfo::new("through_on_strict", through, strict_via, true))
            .unwrap();
        let mut rule = ViaRule::new("same_padstack_different_policy");
        rule.append_via(attach_off);
        rule.append_via(attach_on);
        rules.via_rules.push(rule);
        rules.net_classes.get_mut(class_idx).set_via_rule(Some(0));
        let mut board = BasicBoard::new(stack, rules, padstacks);
        let pin = board.insert_via(smd, IntPoint::new(0, 0), vec![net_no], 1, false);
        board.set_component_no(pin, 1);

        let request = MazeRouteRequest {
            net_no,
            start_item: pin,
            dest_item: pin,
            start_items: Vec::new(),
            dest_items: Vec::new(),
            trace_half_width: 100,
            clearance_class: 1,
            via_padstack: through,
            via_clearance_class: 1,
            via_attach_allowed: false,
            via_cost: 100.0,
            max_expansions: 100,
            ripup_penalty: 0.0,
            deadline: None,
            is_fanout: false,
        };
        let off_choice = ViaChoice {
            padstack: through,
            clearance_class: 1,
            attach_allowed: false,
        };
        let on_choice = ViaChoice {
            clearance_class: strict_via,
            attach_allowed: true,
            ..off_choice
        };

        assert_eq!(
            via_choices_for_transition(&board, &request, 0, 1),
            vec![off_choice, on_choice],
            "ViaInfo policy must not be deduplicated by padstack number"
        );
        assert!(!via_site_is_clear_for_choice(
            &board,
            &request,
            IntPoint::new(0, 0),
            off_choice,
            None,
        ));
        assert!(via_site_is_clear_for_choice(
            &board,
            &request,
            IntPoint::new(0, 0),
            on_choice,
            None,
        ));
    }

    #[test]
    fn bound_empty_or_incompatible_via_rule_fails_closed() {
        use crate::rules::{ViaInfo, ViaRule};

        let stack = LayerStructure::new(vec![Layer::new("F.Cu", true), Layer::new("B.Cu", true)]);
        let matrix = ClearanceMatrix::get_default_instance(stack.clone(), 200);
        let mut rules = BoardRules::new(stack.clone(), matrix);
        let class_idx = rules.get_default_net_class();
        let net_no = rules.nets.add("N", class_idx, false);
        let shape = TileShape::Box(IntBox::from_coords(-100, -100, 100, 100));
        let mut padstacks = Padstacks::new(2);
        let through = padstacks.add_shape_on_layers(shape.clone(), 0, 1);
        let front_only = padstacks.add_shape_on_layers(shape, 0, 0);
        let request = MazeRouteRequest {
            net_no,
            start_item: 0,
            dest_item: 0,
            start_items: Vec::new(),
            dest_items: Vec::new(),
            trace_half_width: 50,
            clearance_class: 1,
            via_padstack: through,
            via_clearance_class: 1,
            via_attach_allowed: false,
            via_cost: 100.0,
            max_expansions: 10,
            ripup_penalty: 0.0,
            deadline: None,
            is_fanout: false,
        };

        rules.via_rules.push(ViaRule::new("empty"));
        rules.net_classes.get_mut(class_idx).set_via_rule(Some(0));
        let mut board = BasicBoard::new(stack, rules, padstacks);
        assert!(via_choices_for_layer(&board, &request, 0).is_empty());
        assert!(via_choices_for_transition(&board, &request, 0, 1).is_empty());

        let blind_info = board
            .rules
            .via_infos
            .add(ViaInfo::new("front_only", front_only, 1, false))
            .unwrap();
        board.rules.via_rules[0].append_via(blind_info);
        assert!(
            via_choices_for_transition(&board, &request, 0, 1).is_empty(),
            "a bound incompatible rule must not widen to the through fallback"
        );

        board
            .rules
            .net_classes
            .get_mut(class_idx)
            .set_via_rule(None);
        assert_eq!(
            via_choices_for_transition(&board, &request, 0, 1)
                .first()
                .map(|choice| choice.padstack),
            Some(through),
            "only an unbound class may use the request fallback"
        );
    }

    #[test]
    fn sparse_via_can_cross_but_not_land_on_a_layer_without_a_pad() {
        use crate::rules::{ViaInfo, ViaRule};

        let stack = LayerStructure::new(vec![
            Layer::new("F.Cu", true),
            Layer::new("In1.Cu", true),
            Layer::new("B.Cu", true),
        ]);
        let matrix = ClearanceMatrix::get_default_instance(stack.clone(), 200);
        let mut rules = BoardRules::new(stack.clone(), matrix);
        let class_idx = rules.get_default_net_class();
        let net_no = rules.nets.add("N", class_idx, false);
        let shape = TileShape::Box(IntBox::from_coords(-300, -300, 300, 300));
        let mut padstacks = Padstacks::new(3);
        let gapped = padstacks.add(
            "gapped",
            vec![Some(shape.clone()), None, Some(shape.clone())],
            false,
            false,
        );
        let front = padstacks.add("front", vec![Some(shape.clone()), None, None], false, false);
        let middle = padstacks.add("middle", vec![None, Some(shape), None], false, false);
        let via_info = rules
            .via_infos
            .add(ViaInfo::new("gapped", gapped, 1, false))
            .unwrap();
        let mut via_rule = ViaRule::new("gapped_rule");
        via_rule.append_via(via_info);
        rules.via_rules.push(via_rule);
        rules.net_classes.get_mut(class_idx).set_via_rule(Some(0));
        let mut board = BasicBoard::new(stack, rules, padstacks);
        let start = board.insert_via(front, IntPoint::new(0, 0), vec![net_no], 1, false);
        let dest = board.insert_via(middle, IntPoint::new(8_000, 0), vec![net_no], 1, false);
        let request = MazeRouteRequest {
            net_no,
            start_item: start,
            dest_item: dest,
            start_items: Vec::new(),
            dest_items: Vec::new(),
            trace_half_width: 100,
            clearance_class: 1,
            via_padstack: gapped,
            via_clearance_class: 1,
            via_attach_allowed: false,
            via_cost: 100.0,
            max_expansions: 20_000,
            ripup_penalty: 0.0,
            deadline: None,
            is_fanout: false,
        };
        let choice = ViaChoice {
            padstack: gapped,
            clearance_class: 1,
            attach_allowed: false,
        };

        assert!(
            via_choices_for_layer(&board, &request, 0)
                .iter()
                .any(|candidate| candidate.padstack == gapped),
            "the plated barrel may cross the padless inner layer"
        );
        assert!(
            via_choices_for_layer(&board, &request, 1).is_empty(),
            "a layer without via copper cannot be a drill source"
        );
        assert!(
            via_choices_for_transition(&board, &request, 0, 1).is_empty(),
            "a layer without via copper cannot be a drill destination"
        );
        assert!(
            via_choices_for_transition(&board, &request, 0, 2)
                .iter()
                .any(|candidate| candidate.padstack == gapped),
            "missing inner annular copper does not break an endpoint-to-endpoint barrel"
        );

        // A stale search result must not bypass the transition gate during
        // replay. This used to insert the malformed via and let a trace end
        // at its centre on a layer where the via has no copper.
        let forged = MazeSearchResult {
            corners: vec![
                (FloatPoint::new(4_000.0, 0.0), 0),
                (FloatPoint::new(4_000.0, 0.0), 1),
            ],
            rooms: vec![None, None],
            via_choices: vec![None, Some(choice)],
        };
        let ids_before: Vec<_> = board.items().map(|(id, _)| *id).collect();
        assert!(insert_connection(&mut board, &request, &forged).is_none());
        assert_eq!(
            board.items().map(|(id, _)| *id).collect::<Vec<_>>(),
            ids_before,
            "rejected replay leaked route items"
        );

        assert!(!board.net_is_completely_connected(net_no));
        assert!(
            maze_route(&mut board, &request).is_none(),
            "the maze must not route onto missing via copper"
        );
        assert!(
            !board.net_is_completely_connected(net_no),
            "the malformed via must not create false connectivity"
        );
    }

    #[test]
    fn net_class_inactive_layer_blocks_drill_expansion() {
        // `(circuit (use_layer ...))`: a disabled routing layer must never
        // be drilled onto (Java AutorouteControl.layer_active). Start pad
        // exists only on F.Cu, dest pad only on B.Cu, so the connection
        // REQUIRES a drill onto B.Cu.
        let stack = LayerStructure::new(vec![Layer::new("F.Cu", true), Layer::new("B.Cu", true)]);
        let matrix = ClearanceMatrix::get_default_instance(stack.clone(), 200);
        let mut rules = BoardRules::new(stack.clone(), matrix);
        let class_idx = rules.get_default_net_class();
        let net_no = rules.nets.add("N1", 1, false);
        assert_eq!(net_no, 1, "request() routes net 1");
        if let Some(net) = rules.nets.get_by_no_mut(net_no) {
            net.set_class(class_idx);
        }
        let pad = TileShape::Box(IntBox::from_coords(-400, -400, 400, 400));
        let mut padstacks = Padstacks::new(2);
        padstacks.add_shape_on_layers(pad.clone(), 0, 1); // 1: through via
        padstacks.add_shape_on_layers(pad.clone(), 0, 0); // 2: F.Cu-only pad
        padstacks.add_shape_on_layers(pad, 1, 1); // 3: B.Cu-only pad
        let mut board = BasicBoard::new(stack, rules, padstacks);
        let a = board.insert_via(2, IntPoint::new(0, 0), vec![net_no], 1, false);
        let b = board.insert_via(3, IntPoint::new(9000, 0), vec![net_no], 1, false);
        // sanity: with every layer active the connection routes via a drill
        let mut open_board = board.clone();
        assert!(
            maze_route(&mut open_board, &request(a, b)).is_some(),
            "the unrestricted board must route"
        );
        // with B.Cu disabled for the class, the drill expansion may not
        // land there and the (B.Cu-only) destination is unreachable
        board
            .rules
            .net_classes
            .get_mut(class_idx)
            .set_active_routing_layer(1, false);
        assert!(
            maze_route(&mut board, &request(a, b)).is_none(),
            "routing onto a disabled net-class layer must be refused"
        );
    }

    #[test]
    fn no_connection_reports_none() {
        let mut board = test_board();
        let a = board.insert_via(1, IntPoint::new(0, 0), vec![1], 1, false);
        let b = board.insert_via(1, IntPoint::new(9000, 0), vec![1], 1, false);
        // a closed box of foreign net around the start pad on both layers
        for layer in 0..2 {
            for (from, to) in [
                ((-2000, -2000), (2000, -2000)),
                ((2000, -2000), (2000, 2000)),
                ((2000, 2000), (-2000, 2000)),
                ((-2000, 2000), (-2000, -2000)),
            ] {
                board.insert_trace(
                    Polyline::from_int_points(&[
                        IntPoint::new(from.0, from.1),
                        IntPoint::new(to.0, to.1),
                    ]),
                    layer,
                    300,
                    vec![2],
                    1,
                );
            }
        }
        assert!(maze_route(&mut board, &request(a, b)).is_none());
    }
}
