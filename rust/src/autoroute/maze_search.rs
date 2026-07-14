//! Port of the core expansion loop of `MazeSearchAlgo.java` (single-layer,
//! without ripup/shove, which follow later) together with a simplified
//! `LocateFoundConnectionAlgo`/`InsertFoundConnectionAlgo`: the found
//! connection is traced through the door-section midpoints and inserted as
//! a polyline trace.
//!
//! The search is a Dijkstra expansion over door *sections*: each section
//! of each door can be occupied once (`MazeSearchElement`), stores its
//! backtrack door, and expansion enters the room behind the door,
//! lazily materializing its neighbors through the engine's frontier
//! expansion.

use std::cmp::Reverse;
use std::collections::BinaryHeap;

use crate::autoroute::engine::AutorouteEngine;
use crate::autoroute::expansion_room::{DoorId, RoomId};
use crate::board::basic_board::{BasicBoard, ItemId};
use crate::geometry::planar::{FloatPoint, IntBox, IntPoint, Polyline, TileShape};

#[derive(Debug, Clone, Copy, PartialEq)]
struct QueueEntry {
    cost: f64,
    door: DoorId,
    section: usize,
    /// The room this expansion enters through the door.
    room_to_enter: RoomId,
    from: Option<(DoorId, usize)>,
    /// The location this entry expands from, for cost calculation.
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
        self.cost
            .partial_cmp(&other.cost)
            .unwrap_or(std::cmp::Ordering::Equal)
            .then_with(|| self.door.cmp(&other.door))
            .then_with(|| self.section.cmp(&other.section))
    }
}

pub struct MazeSearchResult {
    /// The corners of the found connection, from start to destination.
    pub corners: Vec<FloatPoint>,
    pub layer: usize,
}

/// Runs the maze expansion from `start_item` towards `dest_item` on
/// `layer`. Returns the corner list of the found connection.
pub fn find_connection(
    board: &BasicBoard,
    engine: &mut AutorouteEngine,
    start_item: ItemId,
    dest_item: ItemId,
    layer: usize,
    trace_half_width: i32,
) -> Option<MazeSearchResult> {
    let start = board.get_item(start_item)?;
    let (start_shape, _) = start
        .tile_shapes(&board.padstacks)
        .into_iter()
        .find(|(_, l)| *l == layer)?;
    let start_center = start_shape.centre_of_gravity();

    let start_rooms = engine.create_start_rooms(board, start_shape, layer);
    if start_rooms.is_empty() {
        return None;
    }

    let offset = trace_half_width as f64;
    let mut open: BinaryHeap<Reverse<QueueEntry>> = BinaryHeap::new();

    // check the start rooms for the destination and seed the queue
    for &room in &start_rooms {
        if engine
            .target_doors(room)
            .iter()
            .any(|t| t.item == dest_item)
        {
            // trivially connected inside one room
            let dest_point = destination_point(board, engine, dest_item, room, start_center);
            return Some(MazeSearchResult {
                corners: vec![start_center, dest_point],
                layer,
            });
        }
    }
    for &room in &start_rooms {
        // materialize the neighbors of the start room before seeding
        engine.expand_room(board, room);
        seed_room_doors(
            engine,
            board,
            room,
            start_center,
            0.0,
            None,
            offset,
            &mut open,
        );
    }

    while let Some(Reverse(entry)) = open.pop() {
        {
            let sections = &mut engine.graph.door_mut(entry.door).sections;
            if entry.section >= sections.len() || sections[entry.section].is_occupied {
                continue;
            }
            sections[entry.section].is_occupied = true;
            sections[entry.section].backtrack_door = entry.from;
        }
        let room = entry.room_to_enter;
        // materialize the neighbors of the entered room
        engine.expand_room(board, room);

        // destination reached?
        if engine
            .target_doors(room)
            .iter()
            .any(|t| t.item == dest_item)
        {
            // backtrack: collect the section midpoints
            let mut corners = vec![entry.location];
            let mut curr = entry.from;
            while let Some((door, section)) = curr {
                let segments = engine.graph.door_section_segments(door, offset);
                if let Some(seg) = segments.get(section) {
                    corners.push(seg.a.middle_point(seg.b));
                }
                curr = engine.graph.door(door).sections[section].backtrack_door;
            }
            corners.push(start_center);
            corners.reverse();
            let dest_point = destination_point(board, engine, dest_item, room, entry.location);
            corners.push(dest_point);
            return Some(MazeSearchResult { corners, layer });
        }

        seed_room_doors(
            engine,
            board,
            room,
            entry.location,
            entry.cost,
            Some((entry.door, entry.section)),
            offset,
            &mut open,
        );
    }
    None
}

/// Pushes all unoccupied door sections of `room` onto the queue; each
/// entry's cost is `base_cost` plus the distance from `location` to the
/// section midpoint (Dijkstra accumulation).
#[allow(clippy::too_many_arguments)]
fn seed_room_doors(
    engine: &mut AutorouteEngine,
    _board: &BasicBoard,
    room: RoomId,
    location: FloatPoint,
    base_cost: f64,
    from: Option<(DoorId, usize)>,
    offset: f64,
    open: &mut BinaryHeap<Reverse<QueueEntry>>,
) {
    let doors = engine.graph.room(room).doors.clone();
    for door in doors {
        if from.is_some_and(|(d, _)| d == door) {
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
            let cost = base_cost + location.distance(midpoint);
            open.push(Reverse(QueueEntry {
                cost,
                door,
                section,
                room_to_enter: other,
                from,
                location: midpoint,
            }));
        }
    }
}

/// The point on the destination item's shape nearest to `from`.
fn destination_point(
    board: &BasicBoard,
    engine: &AutorouteEngine,
    dest_item: ItemId,
    room: RoomId,
    from: FloatPoint,
) -> FloatPoint {
    let layer = engine.graph.room(room).layer;
    board
        .get_item(dest_item)
        .map(|item| {
            item.tile_shapes(&board.padstacks)
                .into_iter()
                .filter(|(_, l)| *l == layer)
                .map(|(s, _)| s.centre_of_gravity())
                .next()
                .unwrap_or(from)
        })
        .unwrap_or(from)
}

/// Runs the maze search and inserts the found connection as a polyline
/// trace. Returns the id of the inserted trace.
pub fn maze_route(
    board: &mut BasicBoard,
    net_no: i32,
    start_item: ItemId,
    dest_item: ItemId,
    layer: usize,
    trace_half_width: i32,
    clearance_class: usize,
) -> Option<ItemId> {
    let mut engine = AutorouteEngine::new(net_no);
    let result = find_connection(
        board,
        &mut engine,
        start_item,
        dest_item,
        layer,
        trace_half_width,
    )?;
    // round the corners to integer points, dropping duplicates
    let mut corners: Vec<IntPoint> = Vec::with_capacity(result.corners.len());
    for c in &result.corners {
        let p = c.round();
        if corners.last() != Some(&p) {
            corners.push(p);
        }
    }
    if corners.len() < 2 {
        return None;
    }
    Some(board.insert_trace(
        Polyline::from_int_points(&corners),
        result.layer,
        trace_half_width,
        vec![net_no],
        clearance_class,
    ))
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

    #[test]
    fn trivial_connection_in_one_room() {
        let mut board = test_board();
        let a = board.insert_via(1, IntPoint::new(0, 0), vec![1], 1, false);
        let b = board.insert_via(1, IntPoint::new(5000, 0), vec![1], 1, false);
        let trace = maze_route(&mut board, 1, a, b, 0, 100, 1).expect("route failed");
        assert!(board.get_item(trace).is_some());
        assert!(board.net_is_completely_connected(1));
    }

    #[test]
    fn connection_through_door_graph_around_wall() {
        let mut board = test_board();
        let a = board.insert_via(1, IntPoint::new(0, 0), vec![1], 1, false);
        let b = board.insert_via(1, IntPoint::new(9000, 0), vec![1], 1, false);
        // foreign-net wall between them, gap below y = -2000
        board.insert_trace(
            Polyline::from_int_points(&[
                IntPoint::new(4500, -2000),
                IntPoint::new(4500, 9000),
            ]),
            0,
            300,
            vec![2],
            1,
        );
        let trace = maze_route(&mut board, 1, a, b, 0, 100, 1).expect("route failed");
        let item = board.get_item(trace).unwrap();
        // the trace goes around the wall: at x = 4500 its corner path must
        // be below the wall's lower end
        if let crate::board::ItemKind::PolylineTrace(t) = &item.kind {
            assert!(t.corner_count() >= 2);
            let crosses_below = t
                .polyline
                .corner_approx_arr()
                .windows(2)
                .any(|w| {
                    (w[0].x <= 4500.0 && w[1].x >= 4500.0)
                        && (w[0].y + w[1].y) / 2.0 < -1500.0
                });
            assert!(crosses_below, "trace did not detour below the wall");
        } else {
            panic!("not a trace");
        }
        assert!(board.net_is_completely_connected(1));
    }

    #[test]
    fn no_connection_reports_none() {
        let mut board = test_board();
        let a = board.insert_via(1, IntPoint::new(0, 0), vec![1], 1, false);
        let b = board.insert_via(1, IntPoint::new(9000, 0), vec![1], 1, false);
        // a closed box of foreign net around the start pad
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
                0,
                300,
                vec![2],
                1,
            );
        }
        assert!(maze_route(&mut board, 1, a, b, 0, 100, 1).is_none());
    }
}
