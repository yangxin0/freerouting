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
}

/// Parameters of a maze routing request.
pub struct MazeRouteRequest {
    pub net_no: i32,
    pub start_item: ItemId,
    pub dest_item: ItemId,
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
}

/// Runs the maze expansion from the start item towards the destination
/// item. Returns the corner list of the found connection.
pub fn find_connection(
    board: &BasicBoard,
    engine: &mut AutorouteEngine,
    request: &MazeRouteRequest,
) -> Option<MazeSearchResult> {
    let start = board.get_item(request.start_item)?;
    let start_shapes: Vec<(TileShape, usize)> = start.tile_shapes(&board.padstacks).to_vec();
    if start_shapes.is_empty() {
        return None;
    }
    let offset = request.trace_half_width as f64;
    // destination centers for the admissible remaining-distance estimate
    let dest_centers: Vec<FloatPoint> = board
        .get_item(request.dest_item)
        .map(|item| {
            item.tile_shapes(&board.padstacks)
                .into_iter()
                .map(|(s, _)| s.centre_of_gravity())
                .collect()
        })
        .unwrap_or_default();
    let estimate_to_dest = move |p: FloatPoint| -> f64 {
        dest_centers
            .iter()
            .map(|d| p.distance(*d))
            .fold(f64::MAX, f64::min)
            .min(1e12)
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
        if std::env::var_os("FR_DEBUG_MAZE").is_some() {
            eprintln!(
                "MAZE start item {:?} layer {layer}: {} start rooms",
                request.start_item,
                start_rooms.len()
            );
        }
        for &room in &start_rooms {
            if engine
                .target_doors(room)
                .iter()
                .any(|t| t.item == request.dest_item)
            {
                let dest_point =
                    destination_point(board, request.dest_item, *layer, start_center);
                return Some(MazeSearchResult {
                    corners: vec![(start_center, *layer), (dest_point, *layer)],
                });
            }
            engine.expand_room(board, room);
            let root = nodes.len();
            nodes.push(BacktrackNode {
                location: start_center,
                layer: *layer,
                parent: None,
            });
            seed_room(
                engine,
                board,
                request,
                room,
                start_center,
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

    let mut expansions = 0usize;
    while let Some(Reverse(entry)) = open.pop() {
        expansions += 1;
        if expansions > request.max_expansions {
            return None; // budget exhausted
        }
        if expansions.is_multiple_of(1024) && request.deadline.is_some_and(|t| t.limit_exceeded()) {
            return None; // out of time
        }
        // occupy the step
        match entry.step {
            Step::Door { door, section } => {
                let sections = &mut engine.graph.door_mut(door).sections;
                if section >= sections.len() || sections[section].is_occupied {
                    continue;
                }
                sections[section].is_occupied = true;
            }
            Step::Drill => {}
        }
        let room = entry.room_to_enter;
        let layer = engine.graph.room(room).layer;
        let node_id = nodes.len();
        nodes.push(BacktrackNode {
            location: entry.location,
            layer,
            parent: entry.parent,
        });

        engine.expand_room(board, room);

        if engine
            .target_doors(room)
            .iter()
            .any(|t| t.item == request.dest_item)
        {
            // backtrack through the node chain
            let mut corners: Vec<(FloatPoint, usize)> = Vec::new();
            let mut curr = Some(node_id);
            while let Some(i) = curr {
                corners.push((nodes[i].location, nodes[i].layer));
                curr = nodes[i].parent;
            }
            corners.reverse();
            let dest_point =
                destination_point(board, request.dest_item, layer, entry.location);
            corners.push((dest_point, layer));
            return Some(MazeSearchResult { corners });
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
    if std::env::var_os("FR_DEBUG_MAZE").is_some() {
        eprintln!("MAZE exhausted after {expansions} expansions");
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
    estimate_to_dest: &dyn Fn(FloatPoint) -> f64,
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
        let segments = engine.graph.door_section_segments(door, offset);
        for (section, seg) in segments.iter().enumerate() {
            if engine
                .graph
                .door(door)
                .sections
                .get(section)
                .is_some_and(|s| s.is_occupied)
            {
                continue;
            }
            let midpoint = seg.a.middle_point(seg.b);
            let ripup_cost =
                request.ripup_penalty * engine.rippable_items(other).len() as f64;
            let cost = base_cost + location.distance(midpoint) + ripup_cost;
            open.push(Reverse(QueueEntry {
                cost,
                estimate: cost + estimate_to_dest(midpoint),
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
        // grid sampling like drill pages: step derived from the via size
        let room_shape = engine.graph.room(room).shape.clone();
        let bb = room_shape.bounding_box();
        let via_extent = padstack
            .get_shape(from)
            .map(|s| s.bounding_box().width().max(1))
            .unwrap_or(1000);
        let step = (2 * via_extent).max(4 * request.trace_half_width);
        let mut count = 0;
        let mut x = bb.ll.x - bb.ll.x.rem_euclid(step) + step;
        while x < bb.ur.x && count < 16 {
            let mut y = bb.ll.y - bb.ll.y.rem_euclid(step) + step;
            while y < bb.ur.y && count < 16 {
                let p = IntPoint::new(x, y);
                if room_shape.contains(&crate::geometry::planar::Point::Int(p)) {
                    drill_points.push(p);
                    count += 1;
                }
                y += step;
            }
            x += step;
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
                    estimate: cost + estimate_to_dest(drill_point.to_float()),
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
        // the via pad must keep the (conservative: largest for its class)
        // clearance to foreign copper on every spanned layer
        let clearance = board
            .rules
            .clearance_matrix
            .max_value_of_class(request.clearance_class, layer)
            .max(request.trace_half_width);
        let query = shape
            .translate_by(crate::geometry::planar::IntVector::new(point.x, point.y))
            .enlarge(clearance as f64);
        if board.is_blocked(&query, layer, request.net_no) {
            return false;
        }
    }
    true
}

/// The centre of the destination item's shape on `layer` (or its first
/// shape).
fn destination_point(
    board: &BasicBoard,
    dest_item: ItemId,
    layer: usize,
    fallback: FloatPoint,
) -> FloatPoint {
    board
        .get_item(dest_item)
        .and_then(|item| {
            let shapes = item.tile_shapes(&board.padstacks);
            shapes
                .iter()
                .find(|(_, l)| *l == layer)
                .or_else(|| shapes.first())
                .map(|(s, _)| s.centre_of_gravity())
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
    );
    let result = find_connection(board, &mut engine, request)?;

    // with ripup: remove the rippable foreign items intersecting the
    // connection geometry before inserting it
    let mut ripped_nets: Vec<i32> = Vec::new();
    if allow_ripup {
        let mut to_rip: Vec<ItemId> = Vec::new();
        for window in result.corners.windows(2) {
            let ((a, layer_a), (b, layer_b)) = (window[0], window[1]);
            let (pa, pb) = (a.round(), b.round());
            if layer_a == layer_b && pa != pb {
                let polyline = Polyline::from_two_points(pa, pb);
                if let Some(shape) =
                    polyline.offset_shape(request.trace_half_width + 1, 0)
                {
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
            } else if layer_a != layer_b {
                // the via footprint at the layer change
                if let Some(padstack) = board.padstacks.get_by_no(request.via_padstack) {
                    for layer in padstack.from_layer()..=padstack.to_layer() {
                        if let Some(shape) = padstack.get_shape(layer) {
                            let q = shape.translate_by(
                                crate::geometry::planar::IntVector::new(pa.x, pa.y),
                            );
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

    let new_items = insert_connection(board, request, &result)?;
    Some(RoutedConnection {
        new_items,
        ripped_nets,
    })
}

/// Runs the maze search and inserts the found connection as per-layer
/// polyline traces joined by vias. Returns the inserted item ids.
pub fn maze_route(board: &mut BasicBoard, request: &MazeRouteRequest) -> Option<Vec<ItemId>> {
    let mut engine =
        AutorouteEngine::new_with_clearance(request.net_no, false, request.clearance_class);
    let result = find_connection(board, &mut engine, request)?;
    insert_connection(board, request, &result)
}

/// Inserts the found connection as per-layer polyline traces joined by
/// vias and normalizes the junctions.
fn insert_connection(
    board: &mut BasicBoard,
    request: &MazeRouteRequest,
    result: &MazeSearchResult,
) -> Option<Vec<ItemId>> {

    let mut new_items = Vec::new();
    let mut run: Vec<IntPoint> = Vec::new();
    let mut run_layer = result.corners.first()?.1;
    let flush =
        |board: &mut BasicBoard, run: &mut Vec<IntPoint>, layer: usize, items: &mut Vec<ItemId>| {
            run.dedup();
            if run.len() > 1 {
                items.push(board.insert_trace(
                    Polyline::from_int_points(run),
                    layer,
                    request.trace_half_width,
                    vec![request.net_no],
                    request.clearance_class,
                ));
            }
        };
    for (corner, layer) in &result.corners {
        let p = corner.round();
        if *layer != run_layer {
            let via_location = *run.last().unwrap_or(&p);
            flush(board, &mut run, run_layer, &mut new_items);
            new_items.push(board.insert_via(
                request.via_padstack,
                via_location,
                vec![request.net_no],
                request.clearance_class,
                false,
            ));
            run = vec![via_location];
            run_layer = *layer;
        }
        if run.last() != Some(&p) {
            run.push(p);
        }
    }
    flush(board, &mut run, run_layer, &mut new_items);

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
