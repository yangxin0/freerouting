//! Port of the core of `AutorouteEngine.java` (room graph maintenance)
//! with a simplified frontier expansion in place of
//! `SortedRoomNeighbours.java`'s sorted-edge-gap algorithm (deferred).
//!
//! The engine owns the room graph for one routed net. Completed
//! free-space rooms never overlap: new rooms are restrained against the
//! shapes of existing complete rooms in addition to the board obstacles
//! (Java achieves the same by inserting complete rooms into the autoroute
//! search tree). Frontier expansion seeds an incomplete room beyond each
//! border edge of a room; edges whose far side is already covered
//! restrain away to nothing, so expansion terminates naturally.

use crate::autoroute::expansion_room::{RoomGraph, RoomId, RoomKind};
use crate::autoroute::room_completion::{restrain_shape, IncompleteRoom};
use crate::board::basic_board::{BasicBoard, ItemId};
use crate::geometry::planar::TileShape;

/// A door to an own-net target item reachable from a room
/// (Java: `TargetItemExpansionDoor`).
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct TargetDoor {
    pub item: ItemId,
    pub shape_index: usize,
}

pub struct AutorouteEngine {
    pub net_no: i32,
    pub graph: RoomGraph,
    /// If true, rippable foreign route items do not restrain rooms; the
    /// maze search pays a penalty to pass through them.
    pub allow_ripup: bool,
    /// The clearance class of the routed trace (obstacles restrain rooms
    /// inflated by the pairwise clearance to this class).
    pub trace_clearance_class: usize,
    /// The half width of the routed trace: part of the room margin.
    pub trace_half_width: i32,
    /// All completed free-space rooms.
    complete_rooms: Vec<RoomId>,
    /// The target doors of each room, indexed by room id.
    target_doors: Vec<Vec<TargetDoor>>,
    /// The rippable foreign items overlapping each room.
    rippable_items: Vec<Vec<ItemId>>,
    /// Rooms whose frontier was already expanded.
    expanded: Vec<bool>,
}

impl AutorouteEngine {
    pub fn new(net_no: i32) -> Self {
        Self::new_with_ripup(net_no, false)
    }

    pub fn new_with_ripup(net_no: i32, allow_ripup: bool) -> Self {
        Self::new_with_clearance(net_no, allow_ripup, 1, 0)
    }

    pub fn new_with_clearance(
        net_no: i32,
        allow_ripup: bool,
        trace_clearance_class: usize,
        trace_half_width: i32,
    ) -> Self {
        AutorouteEngine {
            net_no,
            graph: RoomGraph::new(),
            allow_ripup,
            trace_clearance_class,
            trace_half_width,
            complete_rooms: Vec::new(),
            target_doors: Vec::new(),
            rippable_items: Vec::new(),
            expanded: Vec::new(),
        }
    }

    /// The rippable foreign items overlapping a room.
    pub fn rippable_items(&self, room: RoomId) -> &[ItemId] {
        &self.rippable_items[room]
    }

    /// Registers target doors for own-net items inserted after rooms were
    /// completed (rooms are reused across the connections of a net; the
    /// new items would otherwise be unreachable as destinations).
    pub fn register_new_targets(&mut self, board: &BasicBoard, items: &[ItemId]) {
        for &item_id in items {
            let Some(item) = board.get_item(item_id) else {
                continue;
            };
            if !item.is_connectable() || !item.base.contains_net(self.net_no) {
                continue;
            }
            for (index, (shape, layer)) in
                item.tile_shapes(&board.padstacks).iter().enumerate()
            {
                for &room in &self.complete_rooms {
                    if self.graph.room(room).layer == *layer
                        && shape.intersects(&self.graph.room(room).shape)
                    {
                        self.target_doors[room].push(TargetDoor {
                            item: item_id,
                            shape_index: index,
                        });
                    }
                }
            }
        }
    }

    pub fn complete_rooms(&self) -> &[RoomId] {
        &self.complete_rooms
    }

    pub fn target_doors(&self, room: RoomId) -> &[TargetDoor] {
        &self.target_doors[room]
    }

    /// Completes an incomplete room against the board obstacles and the
    /// already completed rooms, inserting the results into the room graph
    /// with doors to every touching complete room and target doors to
    /// own-net items. Returns the new room ids.
    pub fn complete_room(&mut self, board: &BasicBoard, room: IncompleteRoom) -> Vec<RoomId> {
        // restrain against the board obstacles
        let mut pieces = crate::autoroute::room_completion::complete_shape_with_ripup(
            board,
            &room,
            self.net_no,
            None,
            self.allow_ripup,
            self.trace_clearance_class,
            self.trace_half_width,
        );
        // restrain against the existing complete rooms (they must not
        // overlap); bounding boxes prune the exact overlap tests
        for &existing in &self.complete_rooms {
            if self.graph.room(existing).layer != room.layer {
                continue;
            }
            let existing_shape = self.graph.room(existing).shape.clone();
            let existing_bbox = existing_shape.bounding_box();
            let mut new_pieces = Vec::new();
            for piece in pieces {
                if !piece.shape.bounding_box().intersects(existing_bbox) {
                    new_pieces.push(piece);
                    continue;
                }
                let intersection = piece.shape.intersection(&existing_shape);
                if intersection.dimension() == 2 {
                    new_pieces.extend(restrain_shape(&piece, &existing_shape));
                } else {
                    new_pieces.push(piece);
                }
            }
            pieces = new_pieces;
        }

        let mut new_rooms = Vec::new();
        for piece in pieces {
            if piece.shape.dimension() < 2 {
                continue;
            }
            let room_id =
                self.graph
                    .add_room(piece.shape.clone(), piece.layer, RoomKind::CompleteFreeSpace);
            // doors to touching complete rooms (bounding boxes prune the
            // exact touch tests)
            let piece_bbox = piece.shape.bounding_box();
            for &existing in &self.complete_rooms {
                if self.graph.room(existing).layer != piece.layer {
                    continue;
                }
                let existing_room = self.graph.room(existing);
                if !existing_room.shape.bounding_box().intersects(piece_bbox) {
                    continue;
                }
                let dim = existing_room
                    .shape
                    .intersection(&piece.shape)
                    .dimension();
                if dim >= 1 {
                    self.graph.add_door_with_dimension(existing, room_id, dim);
                }
            }
            // target doors to own-net connectable items intersecting the
            // room, and the rippable foreign items it overlaps
            let mut targets = Vec::new();
            let mut rippables = Vec::new();
            for item_id in board.overlapping_items(&piece.shape, Some(piece.layer)) {
                let Some(item) = board.get_item(item_id) else {
                    continue;
                };
                if self.allow_ripup
                    && crate::autoroute::room_completion::is_rippable(item, self.net_no)
                {
                    rippables.push(item_id);
                }
                if !item.is_connectable() || !item.base.contains_net(self.net_no) {
                    continue;
                }
                for (index, (shape, layer)) in
                    item.tile_shapes(&board.padstacks).iter().enumerate()
                {
                    if *layer == piece.layer && shape.intersects(&piece.shape) {
                        targets.push(TargetDoor {
                            item: item_id,
                            shape_index: index,
                        });
                    }
                }
            }
            self.complete_rooms.push(room_id);
            self.target_doors.push(targets);
            self.rippable_items.push(rippables);
            self.expanded.push(false);
            debug_assert_eq!(self.target_doors.len(), self.graph.room_count());
            new_rooms.push(room_id);
        }
        new_rooms
    }

    /// Expands the frontier of a room: seeds an incomplete room beyond
    /// each border edge and completes it. Edges already covered by other
    /// rooms produce nothing. Returns the newly created rooms.
    pub fn expand_room(&mut self, board: &BasicBoard, room_id: RoomId) -> Vec<RoomId> {
        if self.expanded[room_id] {
            return Vec::new();
        }
        self.expanded[room_id] = true;
        let room_shape = self.graph.room(room_id).shape.clone();
        let layer = self.graph.room(room_id).layer;
        let mut new_rooms = Vec::new();
        for i in 0..room_shape.border_line_count() {
            let border_line = room_shape.border_line(i);
            // the half plane on the far side of this border edge
            let new_room_shape = TileShape::half_plane(border_line.opposite());
            let contained = room_shape.intersection(&new_room_shape);
            if contained.is_empty() {
                continue;
            }
            let incomplete = IncompleteRoom {
                shape: new_room_shape,
                layer,
                contained_shape: contained,
            };
            new_rooms.extend(self.complete_room(board, incomplete));
        }
        new_rooms
    }

    /// The complete rooms containing `point` on `layer`; if none exists
    /// yet, a room is completed around the point (used for drill targets).
    pub fn rooms_containing(
        &mut self,
        point: crate::geometry::planar::IntPoint,
        layer: usize,
        board: &BasicBoard,
    ) -> Vec<RoomId> {
        let p = crate::geometry::planar::Point::Int(point);
        let existing: Vec<RoomId> = self
            .complete_rooms
            .iter()
            .copied()
            .filter(|&r| {
                self.graph.room(r).layer == layer && self.graph.room(r).shape.contains(&p)
            })
            .collect();
        if !existing.is_empty() {
            return existing;
        }
        self.create_start_rooms(
            board,
            TileShape::Box(crate::geometry::planar::IntBox::new(point, point)),
            layer,
        )
    }

    /// Creates and completes the start rooms around a point-like shape
    /// (e.g. the connection shape of the start item).
    pub fn create_start_rooms(
        &mut self,
        board: &BasicBoard,
        contained_shape: TileShape,
        layer: usize,
    ) -> Vec<RoomId> {
        let start = IncompleteRoom {
            shape: TileShape::Box(board.bounding_box().offset(1000.0)),
            layer,
            contained_shape,
        };
        self.complete_room(board, start)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::board::{Layer, LayerStructure};
    use crate::core::Padstacks;
    use crate::geometry::planar::{IntBox, IntPoint, Point};
    use crate::rules::{BoardRules, ClearanceMatrix};
    use std::collections::VecDeque;

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
    fn room_graph_reaches_target_through_doors() {
        let mut board = test_board();
        let _start_pad = board.insert_via(1, IntPoint::new(0, 0), vec![1], 1, false);
        let target_pad = board.insert_via(1, IntPoint::new(9000, 0), vec![1], 1, false);
        // an obstacle wall of a foreign net between them with a gap below
        let wall = board.insert_trace(
            crate::geometry::planar::Polyline::from_int_points(&[
                IntPoint::new(4500, -2000),
                IntPoint::new(4500, 8000),
            ]),
            0,
            300,
            vec![2],
            1,
        );
        assert!(board.get_item(wall).is_some());

        let mut engine = AutorouteEngine::new(1);
        let start_point = IntPoint::new(0, 0);
        let start_rooms = engine.create_start_rooms(
            &board,
            TileShape::Box(IntBox::new(start_point, start_point)),
            0,
        );
        assert!(!start_rooms.is_empty());

        // breadth-first frontier expansion until a room with a target door
        // to the target pad appears
        let mut queue: VecDeque<RoomId> = start_rooms.into_iter().collect();
        let mut found = false;
        let mut steps = 0;
        while let Some(room) = queue.pop_front() {
            if engine
                .target_doors(room)
                .iter()
                .any(|t| t.item == target_pad)
            {
                found = true;
                break;
            }
            steps += 1;
            if steps > 200 {
                break;
            }
            for new_room in engine.expand_room(&board, room) {
                queue.push_back(new_room);
            }
        }
        assert!(found, "room graph never reached the target pad");

        // rooms never overlap each other 2-dimensionally
        let rooms = engine.complete_rooms().to_vec();
        for (a_pos, &a) in rooms.iter().enumerate() {
            for &b in rooms.iter().skip(a_pos + 1) {
                if engine.graph.room(a).layer != engine.graph.room(b).layer {
                    continue;
                }
                let overlap = engine
                    .graph
                    .room(a)
                    .shape
                    .intersection(&engine.graph.room(b).shape);
                assert!(
                    overlap.dimension() < 2,
                    "rooms {a} and {b} overlap 2-dimensionally"
                );
            }
        }

        // rooms never overlap the obstacle wall
        let wall_shapes: Vec<TileShape> = board
            .get_item(wall)
            .unwrap()
            .tile_shapes(&board.padstacks)
            .iter()
            .map(|(s, _)| s.clone())
            .collect();
        for &room in &rooms {
            for ws in &wall_shapes {
                assert!(
                    engine.graph.room(room).shape.intersection(ws).dimension() < 2,
                    "a room overlaps the obstacle"
                );
            }
        }
    }

    #[test]
    fn start_room_has_target_door_for_own_pad() {
        let mut board = test_board();
        let pad = board.insert_via(1, IntPoint::new(0, 0), vec![1], 1, false);
        let mut engine = AutorouteEngine::new(1);
        let rooms = engine.create_start_rooms(
            &board,
            TileShape::Box(IntBox::new(IntPoint::new(0, 0), IntPoint::new(0, 0))),
            0,
        );
        assert_eq!(rooms.len(), 1);
        let targets = engine.target_doors(rooms[0]);
        assert!(targets.iter().any(|t| t.item == pad));
        // the room contains the start point and covers free space
        assert!(engine
            .graph
            .room(rooms[0])
            .shape
            .contains(&Point::Int(IntPoint::new(0, 0))));
    }
}
