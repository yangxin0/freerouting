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

use std::cmp::Reverse;
use std::collections::{BinaryHeap, HashSet};

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

pub struct MazeSearchResult {
    /// The corners of the found connection with their layers, from start
    /// to destination. Consecutive corners on different layers are joined
    /// by a via.
    pub corners: Vec<(FloatPoint, usize)>,
    /// The room entered at each corner (diagnostics; aligned with
    /// `corners`, `None` for the appended destination point).
    pub rooms: Vec<Option<RoomId>>,
}

/// Parameters of a maze routing request.
pub struct MazeRouteRequest {
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
}

/// Runs the maze expansion from the start item towards the destination
/// item. Returns the corner list of the found connection.
pub fn find_connection(
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
    let mut drilled: HashSet<(i32, i32, usize)> = HashSet::new();

    // create and seed the start rooms on every layer of the start item
    for (start_shape, layer) in &start_shapes {
        let start_center = start_shape.centre_of_gravity();
        let mut start_rooms = engine.create_start_rooms(board, start_shape.clone(), *layer);
        if start_rooms.is_empty() {
            // an earlier layer's expansion may already have completed
            // rooms covering this pad (new rooms must not overlap them);
            // those existing rooms then serve as the start
            start_rooms =
                engine.rooms_containing(start_center.round(), *layer, board);
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
            let start_point = if start_door.dimension() >= 1 {
                start_door.centre_of_gravity()
            } else {
                start_center
            };
            if let Some(t) = engine
                .target_doors(room)
                .iter()
                .find(|t| request.is_dest(t.item))
            {
                let dest_point = destination_point(
                    board,
                    t.item,
                    *layer,
                    Some(&room_shape),
                    start_point,
                );
                // coincident points would insert nothing (stacked pads of
                // one net whose contact never registers): fall through to
                // the search instead of returning a degenerate route
                if dest_point.round() != start_point.round() {
                    return Some(MazeSearchResult {
                        corners: vec![(start_point, *layer), (dest_point, *layer)],
                        rooms: vec![Some(room), None],
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
        let Some(dest) = board.get_item(dest_id) else { continue };
        let dest_shapes: Vec<(TileShape, usize)> =
            dest.tile_shapes(&board.padstacks).to_vec();
        for (dest_shape, layer) in dest_shapes {
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
        });

        // fanout completes at the first drill (Java: MazeSearchAlgo
        // "algorithm completed after the first drill")
        if request.is_fanout && matches!(entry.step, Step::Drill) {
            let mut corners: Vec<(FloatPoint, usize)> = Vec::new();
            let mut rooms: Vec<Option<RoomId>> = Vec::new();
            let mut curr = Some(node_id);
            while let Some(i) = curr {
                corners.push((nodes[i].location, nodes[i].layer));
                rooms.push(nodes[i].room);
                curr = nodes[i].parent;
            }
            corners.reverse();
            rooms.reverse();
            return Some(MazeSearchResult { corners, rooms });
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
            let mut curr = Some(node_id);
            while let Some(i) = curr {
                corners.push((nodes[i].location, nodes[i].layer));
                rooms.push(nodes[i].room);
                curr = nodes[i].parent;
            }
            corners.reverse();
            rooms.reverse();
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
            return Some(MazeSearchResult { corners, rooms });
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
    drilled: &mut HashSet<(i32, i32, usize)>,
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
        let already_ripped =
            other_item.is_some() && other_item == engine.obstacle_room_item(room);
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
            let section_discount = if shove_discount < 1.0
                && (section == 0 || section + 1 == section_count)
            {
                shove_discount
            } else {
                1.0
            };
            let ripup_cost = if already_ripped {
                0.0
            } else {
                request.ripup_penalty
                    * engine.rippable_items(other).len() as f64
                    * section_discount
            };
            let cost = base_cost + location.distance(midpoint) + ripup_cost;
            // occupy ON PUSH (Java: expand_to_door_section sets
            // is_occupied when the element is inserted): each section
            // enters the queue exactly once, from the cheapest frontier
            // element known at that time. Occupy-on-pop instead lets
            // every re-entry of a room re-seed all its sections — a
            // relaxation storm (69M pushes on NormalPuzzle).
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
            }));
        }
    }
    // drill expansion (Java: ExpansionDrill candidates from DrillPages):
    // try the entry location plus a grid of sample points within the room
    let Some(padstack) = board.padstacks.get_by_no(request.via_padstack) else {
        return;
    };
    let from = padstack.from_layer();
    let to = padstack.to_layer();
    if layer < from || layer > to {
        return;
    }
    let mut drill_points: Vec<IntPoint> = vec![location.round()];
    {
        // drill pages (Java: DrillPageArray): cached convex free areas;
        // candidates are the page drills whose free area intersects the
        // current room
        let room_shape = engine.graph.room(room).shape.clone();
        let bb = room_shape.bounding_box();
        if engine.drill_pages.is_none() {
            engine.drill_pages = Some(
                crate::autoroute::drill_pages::DrillPageArray::new(
                    board,
                    request.via_padstack,
                ),
            );
        }
        let via_margin = padstack
            .get_shape(from)
            .map(|s| (s.bounding_box().max_width() / 2.0) as i32)
            .unwrap_or(1000)
            + board.rules.clearance_matrix.max_value(layer).max(0);
        let pages = engine.drill_pages.as_mut().unwrap();
        pages.sync_board_changes(board);
        for drill in pages.drills_overlapping(board, &bb, request.net_no, via_margin) {
            if drill_points.len() >= 17 {
                break;
            }
            if room_shape
                .contains(&crate::geometry::planar::Point::Int(drill.location))
            {
                drill_points.push(drill.location);
            }
        }
    }
    for drill_point in drill_points {
        if !via_free(board, request, drill_point) {
            continue;
        }
        let drill_cost = base_cost + location.distance(drill_point.to_float());
        for next_layer in from..=to {
            if next_layer == layer {
                continue;
            }
            if !drilled.insert((drill_point.x, drill_point.y, next_layer)) {
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
                }));
            }
        }
    }
}

/// True if a via at `point` keeps its clearance on all layers it spans.
fn via_free(board: &BasicBoard, request: &MazeRouteRequest, point: IntPoint) -> bool {
    let Some(padstack) = board.padstacks.get_by_no(request.via_padstack) else {
        return false;
    };
    for layer in padstack.from_layer()..=padstack.to_layer() {
        let Some(shape) = padstack.get_shape(layer) else {
            continue;
        };
        // the via pad must keep the pairwise clearance to every foreign
        // item on every spanned layer (a max-clearance check falsely
        // seals tight pockets)
        let via_shape =
            shape.translate_by(crate::geometry::planar::IntVector::new(point.x, point.y));
        let max_cl = board.rules.clearance_matrix.max_value(layer).max(0) as f64;
        let query = via_shape.offset(max_cl);
        for id in board.overlapping_items(&query, Some(layer)) {
            let Some(item) = board.get_item(id) else {
                continue;
            };
            if item.base.contains_net(request.net_no) {
                continue;
            }
            if let crate::board::ItemKind::ObstacleArea(a) = &item.kind {
                if a.is_conduction {
                    continue; // planes get fabrication cutouts
                }
            }
            // with ripup, rippable items do not block drills: the via
            // insertion rips whatever its footprint overlaps
            if request.ripup_penalty > 0.0
                && crate::autoroute::room_completion::is_rippable(item, request.net_no)
            {
                continue;
            }
            let pairwise = board
                .rules
                .clearance_matrix
                .get_value(item.base.clearance_class, request.clearance_class, layer, false)
                .max(0) as f64;
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
                    if !room.contains(&pt)
                        && !room.to_simplex().offset(2.0).contains(&pt)
                    {
                        return Some(fallback);
                    }
                }
                return Some(tap.to_float());
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
pub struct RoutedConnection {
    pub new_items: Vec<ItemId>,
    /// The nets of the rippable items removed to make room (empty without
    /// ripup).
    pub ripped_nets: Vec<i32>,
}

/// Runs the maze search and inserts the found connection as per-layer
/// polyline traces joined by vias, ripping the rippable foreign items the
/// connection passes through when `ripup_penalty` > 0. Returns the
/// inserted item ids and the ripped nets.
pub fn maze_route_with_ripup(
    board: &mut BasicBoard,
    request: &MazeRouteRequest,
) -> Option<RoutedConnection> {
    let allow_ripup = request.ripup_penalty > 0.0;
    let mut engine = AutorouteEngine::new_with_clearance(
        request.net_no,
        allow_ripup,
        request.clearance_class,
        request.trace_half_width,
    );
    maze_route_with_engine(board, &mut engine, request)
}

/// Like [`maze_route_with_ripup`], but reusing a caller-owned engine: the
/// expansion-room graph stays valid across the connections of one net
/// (own-net items never restrain rooms; items ripped in between only make
/// the kept rooms conservative). The engine must be fresh whenever the
/// board changes outside this net's routing (e.g. after an undo).
pub fn maze_route_with_engine(
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
                    if let Some(seg) = Polyline::from_two_points(ra, rb)
                        .offset_shape(request.trace_half_width, 0)
                    {
                        let cl = board
                            .rules
                            .clearance_matrix
                            .get_value(
                                request.clearance_class,
                                request.clearance_class,
                                la,
                                false,
                            )
                            .max(0) as f64;
                        let check = seg.offset(cl - 2.0);
                        for id in board.overlapping_items(&check, Some(la)) {
                            let Some(item) = board.get_item(id) else { continue };
                            if item.base.contains_net(request.net_no) {
                                continue;
                            }
                            if let crate::board::ItemKind::ObstacleArea(ar) = &item.kind {
                                if ar.is_conduction {
                                    continue;
                                }
                            }
                            if !item.tile_shapes(&board.padstacks).iter().any(|(s, l)| {
                                *l == la && s.intersection(&check).dimension() >= 2
                            }) {
                                continue;
                            }
                            eprintln!(
                                "ROOM LEAK net {} corner {k} room {room} bbox {:?} \
                                 contains segment ({},{})→({},{}) layer {la} \
                                 but item {id} (nets {:?}, birth {}, bbox {:?}) blocks",
                                request.net_no,
                                shape.bounding_box(),
                                ra.x, ra.y, rb.x, rb.y,
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
                                        && s.offset(cl_m)
                                            .intersection(&p.shape)
                                            .dimension()
                                            >= 2
                                })
                            });
                            eprintln!("  RECOMPLETE pieces {} still-dirty {still}", re.len());
                            if !still {
                                // stale-looking: capture exact shapes for
                                // offline reproduction
                                let margin = request.trace_half_width
                                    + board
                                        .rules
                                        .clearance_matrix
                                        .get_value(
                                            item.base.clearance_class,
                                            request.clearance_class,
                                            la,
                                            true,
                                        )
                                        .max(0);
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
                let Some(item) = board.get_item(id) else { continue };
                if item.base.contains_net(request.net_no) {
                    continue;
                }
                if let crate::board::ItemKind::ObstacleArea(ar) = &item.kind {
                    if ar.is_conduction {
                        continue;
                    }
                }
                let cl = board
                    .rules
                    .clearance_matrix
                    .get_value(
                        item.base.clearance_class,
                        request.clearance_class,
                        r.layer,
                        false,
                    )
                    .max(0) as f64;
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
        for window in result.corners.windows(2) {
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
                if let Some(padstack) = board.padstacks.get_by_no(request.via_padstack) {
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
        let mut to_rip: Vec<ItemId> = Vec::new();
        // Java's exact rip set: the items whose obstacle rooms the path
        // traversed
        for room in result.rooms.iter().flatten() {
            if let Some(item_id) = engine.obstacle_room_item(*room) {
                to_rip.push(item_id);
            }
        }
        for window in result.corners.windows(2) {
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
                let max_cl = board
                    .rules
                    .clearance_matrix
                    .max_value(layer_a)
                    .max(0);
                if let Some(shape) =
                    polyline.offset_shape(request.trace_half_width + max_cl + 1, 0)
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
            if layer_a != layer_b {
                // the via footprint at the layer change (the via sits at
                // the DRILL node pb, not at the corner before it)
                if let Some(padstack) = board.padstacks.get_by_no(request.via_padstack) {
                    for layer in padstack.from_layer()..=padstack.to_layer() {
                        if let Some(shape) = padstack.get_shape(layer) {
                            let max_cl = board
                                .rules
                                .clearance_matrix
                                .max_value(layer)
                                .max(0) as f64;
                            let q = shape
                                .translate_by(
                                    crate::geometry::planar::IntVector::new(pb.x, pb.y),
                                )
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
    Some(RoutedConnection {
        new_items,
        ripped_nets,
    })
}

/// Runs the maze search and inserts the found connection as per-layer
/// polyline traces joined by vias. Returns the inserted item ids.
pub fn maze_route(board: &mut BasicBoard, request: &MazeRouteRequest) -> Option<Vec<ItemId>> {
    let mut engine = AutorouteEngine::new_with_clearance(
        request.net_no,
        false,
        request.clearance_class,
        request.trace_half_width,
    );
    let result = find_connection(board, &mut engine, request)?;
    insert_connection(board, request, &result)
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
                    let y = if to.y >= from.y { from.y + abs_dx } else { from.y - abs_dx };
                    FloatPoint::new(to.x, y)
                } else {
                    let y = if to.y > from.y { to.y - abs_dx } else { to.y + abs_dx };
                    FloatPoint::new(from.x, y)
                }
            } else if horizontal_first {
                let x = if to.x > from.x { to.x - abs_dy } else { to.x + abs_dy };
                FloatPoint::new(x, from.y)
            } else {
                let x = if to.x > from.x { from.x + abs_dy } else { from.x - abs_dy };
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
        };
    }
    let mut corners: Vec<(FloatPoint, usize)> = Vec::new();
    let mut rooms: Vec<Option<RoomId>> = Vec::new();
    for k in 0..result.corners.len() {
        let (b, lb) = result.corners[k];
        if let Some(&(a, la)) = corners.last().map(|c| c).filter(|_| k > 0) {
            if la == lb && !segment_is_compliant(a, b, restriction) {
                // choose horizontal_first so the extra corner stays in
                // the segment's room (try true, then false, like Java)
                let room = result.rooms[k - 1].or(result.rooms[k]);
                let mut extra = calculate_additional_corner(a, b, true, restriction);
                if let Some(r) = room {
                    let shape = &engine.graph.room(r).shape;
                    let inside = |p: FloatPoint| {
                        let ip = crate::geometry::planar::Point::Int(p.round());
                        shape.contains(&ip)
                            || shape.to_simplex().offset(2.0).contains(&ip)
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
                    corners.push((rounded, lb));
                    rooms.push(result.rooms[k - 1]);
                }
            }
        }
        corners.push((b, lb));
        rooms.push(result.rooms[k]);
    }
    MazeSearchResult { corners, rooms }
}

/// Inserts the found connection as per-layer polyline traces joined by
/// vias and normalizes the junctions.
fn insert_connection(
    board: &mut BasicBoard,
    request: &MazeRouteRequest,
    result: &MazeSearchResult,
) -> Option<Vec<ItemId>> {
    crate::board::basic_board::set_birth_tag(1);

    let mut new_items = Vec::new();
    let mut run: Vec<IntPoint> = Vec::new();
    let mut run_layer = result.corners.first()?.1;
    let flush =
        |board: &mut BasicBoard, run: &mut Vec<IntPoint>, layer: usize, items: &mut Vec<ItemId>| {
            run.dedup();
            if run.len() > 1 {
                let polyline = Polyline::from_int_points(run);
                // birth-site validation: the post-rip board must leave
                // every inserted segment its full clearance
                if crate::debug::maze() {
                    for seg in polyline.offset_shapes(request.trace_half_width) {
                        let cl = board
                            .rules
                            .clearance_matrix
                            .get_value(request.clearance_class, request.clearance_class, layer, false)
                            .max(0) as f64;
                        let check = seg.offset(cl - 2.0);
                        for id in board.overlapping_items(&check, Some(layer)) {
                            let Some(item) = board.get_item(id) else { continue };
                            if item.base.contains_net(request.net_no) {
                                continue;
                            }
                            if let crate::board::ItemKind::ObstacleArea(a) = &item.kind {
                                if a.is_conduction {
                                    continue;
                                }
                            }
                            // exact: only report 2D overlaps (the tree
                            // query also returns boundary touches)
                            if !item.tile_shapes(&board.padstacks).iter().any(|(s, l)| {
                                *l == layer
                                    && s.intersection(&check).dimension() >= 2
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
                    eprintln!("INSERTED trace {new_id} net {} layer {layer}", request.net_no);
                }
                items.push(new_id);
            }
        };
    for (corner, layer) in &result.corners {
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
            flush(board, &mut run, run_layer, &mut new_items);
            new_items.push(board.insert_via(
                request.via_padstack,
                p,
                vec![request.net_no],
                request.clearance_class,
                false,
            ));
            run = vec![p];
            run_layer = *layer;
        }
        if run.last() != Some(&p) {
            run.push(p);
        }
    }
    flush(board, &mut run, run_layer, &mut new_items);
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
        let stack = LayerStructure::new(vec![
            Layer::new("F.Cu", true),
            Layer::new("B.Cu", true),
        ]);
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
    fn connection_through_door_graph_around_wall() {
        let mut board = test_board();
        let a = board.insert_via(1, IntPoint::new(0, 0), vec![1], 1, false);
        let b = board.insert_via(1, IntPoint::new(9000, 0), vec![1], 1, false);
        // foreign-net wall between them on BOTH layers, gap below y = -2000
        for layer in 0..2 {
            board.insert_trace(
                Polyline::from_int_points(&[
                    IntPoint::new(4500, -2000),
                    IntPoint::new(4500, 9000),
                ]),
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
                detoured |= t
                    .polyline
                    .corner_approx_arr()
                    .windows(2)
                    .any(|w| {
                        (w[0].x <= 4500.0 && w[1].x >= 4500.0)
                            && (w[0].y + w[1].y) / 2.0 < -1500.0
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
            Polyline::from_int_points(&[
                IntPoint::new(4500, -50000),
                IntPoint::new(4500, 50000),
            ]),
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
