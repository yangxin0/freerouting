//! Port of the expansion-room object model of the maze search:
//! `ExpansionRoom.java`, `FreeSpaceExpansionRoom.java` (incomplete /
//! complete), `ObstacleExpansionRoom.java`, `ExpansionDoor.java` and
//! `MazeSearchElement.java`.
//!
//! The Java pointer graph (rooms and doors cross-referencing each other)
//! becomes an arena [`RoomGraph`] with `RoomId`/`DoorId` indices.

use crate::board::basic_board::ItemId;
use crate::geometry::planar::{FloatLine, FloatPoint, TileShape};

/// Tolerance for the accuracy of the traces in the autoroute algorithm
/// (Java: `AutorouteEngine.TRACE_WIDTH_TOLERANCE`).
pub const TRACE_WIDTH_TOLERANCE: f64 = 2.0;

pub type RoomId = usize;
pub type DoorId = usize;

/// Adjustment of a maze search element (Java:
/// `MazeSearchElement.Adjustment`).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Adjustment {
    #[default]
    None,
    Right,
    Left,
}

/// The maze-search state of one section of an expandable object.
#[derive(Debug, Clone)]
pub struct MazeSearchElement {
    /// True if this section is already occupied by the maze expansion.
    pub is_occupied: bool,
    /// The door and section this element was entered from, for
    /// backtracking.
    pub backtrack_door: Option<(DoorId, usize)>,
    pub room_ripped: bool,
    pub adjustment: Adjustment,
    /// The ripup cost paid to enter this door's room; zero when
    /// `room_ripped` is false.
    pub ripup_cost: i32,
    /// The best path cost this section was queued with so far; pushes
    /// that cannot improve it are pruned (the duplicate pushes otherwise
    /// dominate the open heap).
    pub best_cost: f64,
}

impl Default for MazeSearchElement {
    fn default() -> Self {
        MazeSearchElement {
            is_occupied: false,
            backtrack_door: None,
            room_ripped: false,
            adjustment: Adjustment::default(),
            ripup_cost: 0,
            best_cost: f64::INFINITY,
        }
    }
}

impl MazeSearchElement {
    pub fn reset(&mut self) {
        *self = MazeSearchElement::default();
    }
}

/// The kind of an expansion room.
#[derive(Debug, Clone, PartialEq)]
pub enum RoomKind {
    /// A not yet completed free-space room: the shape may still shrink,
    /// but it must keep containing `contained_shape`
    /// (Java: `IncompleteFreeSpaceExpansionRoom`).
    IncompleteFreeSpace { contained_shape: TileShape },
    /// A completed free-space room: maximal and obstacle-free
    /// (Java: `CompleteFreeSpaceExpansionRoom`).
    CompleteFreeSpace,
    /// A room around an obstacle item, used for ripup routing
    /// (Java: `ObstacleExpansionRoom`).
    Obstacle { item: ItemId, shape_index: usize },
}

#[derive(Debug, Clone)]
pub struct ExpansionRoom {
    pub shape: TileShape,
    pub layer: usize,
    pub kind: RoomKind,
    /// The doors to neighbour rooms.
    pub doors: Vec<DoorId>,
    /// False after the room was invalidated (net switch or board change);
    /// dead rooms are skipped by every engine lookup and expansion.
    pub alive: bool,
}

/// A common edge between two expansion rooms.
#[derive(Debug, Clone)]
pub struct ExpansionDoor {
    pub first_room: RoomId,
    pub second_room: RoomId,
    /// 1 for edge doors; 2-dimensional doors only exist between obstacle
    /// rooms (or overlapping free-space rooms at corners).
    pub dimension: i32,
    /// Each section can be expanded separately by the maze search.
    pub sections: Vec<MazeSearchElement>,
    /// Cache of the computed section segments per offset: the door shape
    /// simplification dominated the routing profile when recomputed on
    /// every room seeding.
    cached_segments: Option<(f64, Vec<FloatLine>)>,
}

/// Arena of rooms and doors.
#[derive(Debug, Default)]
pub struct RoomGraph {
    rooms: Vec<ExpansionRoom>,
    doors: Vec<ExpansionDoor>,
}

impl RoomGraph {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn add_room(&mut self, shape: TileShape, layer: usize, kind: RoomKind) -> RoomId {
        self.rooms.push(ExpansionRoom {
            shape,
            layer,
            kind,
            doors: Vec::new(),
            alive: true,
        });
        self.rooms.len() - 1
    }

    /// Marks a room dead and detaches its doors from both sides (the
    /// neighbours keep routing through their remaining doors).
    pub fn remove_room(&mut self, id: RoomId) {
        self.rooms[id].alive = false;
        let doors = std::mem::take(&mut self.rooms[id].doors);
        for door in doors {
            let other = self.other_room(door, id);
            if let Some(other) = other {
                self.rooms[other].doors.retain(|&d| d != door);
            }
        }
    }

    pub fn room(&self, id: RoomId) -> &ExpansionRoom {
        &self.rooms[id]
    }

    pub fn room_mut(&mut self, id: RoomId) -> &mut ExpansionRoom {
        &mut self.rooms[id]
    }

    pub fn room_count(&self) -> usize {
        self.rooms.len()
    }

    pub fn door(&self, id: DoorId) -> &ExpansionDoor {
        &self.doors[id]
    }

    pub fn door_mut(&mut self, id: DoorId) -> &mut ExpansionDoor {
        &mut self.doors[id]
    }

    pub fn door_count(&self) -> usize {
        self.doors.len()
    }

    /// Creates a door between two rooms with the dimension of their shape
    /// intersection, registering it in both rooms
    /// (Java: `ExpansionDoor` constructor + `ExpansionRoom.add_door`).
    pub fn add_door(&mut self, first_room: RoomId, second_room: RoomId) -> DoorId {
        let dimension = self.rooms[first_room]
            .shape
            .intersection(&self.rooms[second_room].shape)
            .dimension();
        self.add_door_with_dimension(first_room, second_room, dimension)
    }

    pub fn add_door_with_dimension(
        &mut self,
        first_room: RoomId,
        second_room: RoomId,
        dimension: i32,
    ) -> DoorId {
        let id = self.doors.len();
        self.doors.push(ExpansionDoor {
            first_room,
            second_room,
            dimension,
            sections: Vec::new(),
            cached_segments: None,
        });
        self.rooms[first_room].doors.push(id);
        self.rooms[second_room].doors.push(id);
        id
    }

    /// The shape of a door: the intersection of its rooms' shapes.
    pub fn door_shape(&self, door: DoorId) -> TileShape {
        let d = &self.doors[door];
        // simplified like Java's Simplex.intersection: redundant border
        // lines survive a plain intersection, and consecutive nearly
        // parallel redundant lines make corner approximations quasi
        // infinite, poisoning the door sections and the routed corners
        self.rooms[d.first_room]
            .shape
            .intersection_with_simplify(&self.rooms[d.second_room].shape)
    }

    /// The other room of the door, if `room` is one of its rooms.
    pub fn other_room(&self, door: DoorId, room: RoomId) -> Option<RoomId> {
        let d = &self.doors[door];
        if d.first_room == room {
            Some(d.second_room)
        } else if d.second_room == room {
            Some(d.first_room)
        } else {
            None
        }
    }

    /// True if the two rooms share a door
    /// (Java: `ExpansionRoom.door_exists`).
    pub fn door_exists(&self, room_1: RoomId, room_2: RoomId) -> bool {
        self.rooms[room_1]
            .doors
            .iter()
            .any(|&d| self.other_room(d, room_1) == Some(room_2))
    }

    /// Calculates the line segments of the sections of the door, allocating
    /// the door's maze search sections
    /// (Java: `ExpansionDoor.get_section_segments`).
    pub fn door_section_segments(&mut self, door: DoorId, offset: f64) -> Vec<FloatLine> {
        let offset = offset + TRACE_WIDTH_TOLERANCE;
        if let Some((cached_offset, segments)) = &self.doors[door].cached_segments {
            if *cached_offset == offset {
                return segments.clone();
            }
        }
        let segments = self.compute_door_section_segments(door, offset);
        self.doors[door].cached_segments = Some((offset, segments.clone()));
        segments
    }

    /// Uncached worker for [`Self::door_section_segments`]; `offset`
    /// already includes the tolerance.
    fn compute_door_section_segments(&mut self, door: DoorId, offset: f64) -> Vec<FloatLine> {
        let door_shape = self.door_shape(door);
        if door_shape.is_empty() {
            return Vec::new();
        }
        let d = &self.doors[door];
        let (door_line_segment, shrinked_line_segment);
        if d.dimension == 1 {
            let Some(segment) = door_shape.diagonal_corner_segment() else {
                return Vec::new();
            };
            door_line_segment = segment;
            shrinked_line_segment = segment.shrink_segment(offset);
        } else if d.dimension == 2
            && matches!(self.rooms[d.first_room].kind, RoomKind::CompleteFreeSpace)
            && matches!(self.rooms[d.second_room].kind, RoomKind::CompleteFreeSpace)
        {
            // Overlapping doors at a corner are possible in case of 90- or
            // 45-degree routing; in free-angle routing the corners are cut
            // off.
            let Some(segment) = self.calc_door_line_segment(door, &door_shape) else {
                // one free-space room inside the other
                return Vec::new();
            };
            if segment.b.distance_square(segment.a) < 4.0 * offset * offset {
                // 2-dimensional small doors are not yet expanded
                return Vec::new();
            }
            door_line_segment = segment;
            shrinked_line_segment = segment.shrink_segment(offset);
        } else {
            let gravity_point = door_shape.centre_of_gravity();
            door_line_segment = FloatLine::new(gravity_point, gravity_point);
            shrinked_line_segment = door_line_segment;
        }
        let max_door_section_width = 10.0 * offset;
        let section_count = (door_line_segment.b.distance(door_line_segment.a)
            / max_door_section_width) as usize
            + 1;
        self.allocate_sections(door, section_count);
        shrinked_line_segment.divide_segment_into_sections(section_count)
    }

    /// A diagonal line of the 2-dimensional door shape representing the
    /// restraint line between the two room shapes.
    fn calc_door_line_segment(&self, door: DoorId, door_shape: &TileShape) -> Option<FloatLine> {
        let d = &self.doors[door];
        let first_room_shape = &self.rooms[d.first_room].shape;
        let second_room_shape = &self.rooms[d.second_room].shape;
        let mut first_corner: Option<crate::geometry::planar::Point> = None;
        let mut second_corner: Option<crate::geometry::planar::Point> = None;
        for i in 0..door_shape.border_line_count() {
            let curr_corner = door_shape.corner(i);
            if !first_room_shape.contains_inside(&curr_corner)
                && !second_room_shape.contains_inside(&curr_corner)
            {
                // the corner is on the border of both room shapes
                match &first_corner {
                    None => first_corner = Some(curr_corner),
                    Some(fc) if *fc != curr_corner => {
                        second_corner = Some(curr_corner);
                        break;
                    }
                    _ => {}
                }
            }
        }
        let (fc, sc) = (first_corner?, second_corner?);
        Some(FloatLine::new(fc.to_float(), sc.to_float()))
    }

    fn allocate_sections(&mut self, door: DoorId, section_count: usize) {
        let sections = &mut self.doors[door].sections;
        if sections.len() != section_count {
            *sections = (0..section_count)
                .map(|_| MazeSearchElement::default())
                .collect();
        }
    }

    /// Resets all maze search state for routing the next connection.
    pub fn reset(&mut self) {
        for door in &mut self.doors {
            for section in &mut door.sections {
                section.reset();
            }
        }
    }

    /// The centre of gravity of a door's shape (used as expansion target
    /// approximation).
    pub fn door_center(&self, door: DoorId) -> FloatPoint {
        self.door_shape(door).centre_of_gravity()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geometry::planar::IntBox;

    fn box_shape(llx: i32, lly: i32, urx: i32, ury: i32) -> TileShape {
        TileShape::Box(IntBox::from_coords(llx, lly, urx, ury))
    }

    #[test]
    fn rooms_and_doors() {
        let mut graph = RoomGraph::new();
        // two free-space rooms sharing the edge x = 100
        let left = graph.add_room(box_shape(0, 0, 100, 100), 0, RoomKind::CompleteFreeSpace);
        let right = graph.add_room(box_shape(100, 0, 200, 100), 0, RoomKind::CompleteFreeSpace);
        let door = graph.add_door(left, right);
        assert_eq!(graph.door(door).dimension, 1);
        assert_eq!(graph.other_room(door, left), Some(right));
        assert_eq!(graph.other_room(door, right), Some(left));
        assert!(graph.door_exists(left, right));
        assert_eq!(graph.room(left).doors, vec![door]);

        // the door shape is the common edge
        let shape = graph.door_shape(door);
        assert_eq!(shape.dimension(), 1);
        assert_eq!(shape.bounding_box(), IntBox::from_coords(100, 0, 100, 100));

        // section segments run along the edge, shrunk by the offset
        let segments = graph.door_section_segments(door, 8.0);
        assert!(!segments.is_empty());
        assert_eq!(graph.door(door).sections.len(), segments.len());
        for seg in &segments {
            assert!((seg.a.x - 100.0).abs() < 1e-9);
            assert!((seg.b.x - 100.0).abs() < 1e-9);
            assert!(seg.a.y >= 9.9 && seg.b.y <= 90.1);
        }

        // occupy a section, then reset
        graph.door_mut(door).sections[0].is_occupied = true;
        graph.door_mut(door).sections[0].backtrack_door = Some((door, 0));
        graph.reset();
        assert!(!graph.door(door).sections[0].is_occupied);
        assert!(graph.door(door).sections[0].backtrack_door.is_none());
    }

    #[test]
    fn two_dimensional_door_between_free_space_rooms() {
        let mut graph = RoomGraph::new();
        // two overlapping free-space rooms (as in 90-degree routing)
        let a = graph.add_room(box_shape(0, 0, 120, 100), 0, RoomKind::CompleteFreeSpace);
        let b = graph.add_room(box_shape(80, 0, 200, 100), 0, RoomKind::CompleteFreeSpace);
        let door = graph.add_door(a, b);
        assert_eq!(graph.door(door).dimension, 2);
        let segments = graph.door_section_segments(door, 5.0);
        // the restraint line runs across the overlap between the borders
        // of both rooms
        assert!(!segments.is_empty());
        let first = segments.first().unwrap();
        let last = segments.last().unwrap();
        let length: f64 = segments.iter().map(|s| s.a.distance(s.b)).sum();
        assert!(length > 0.0);
        // the segment endpoints stay inside the door shape's bounding box
        let bb = graph.door_shape(door).bounding_box();
        for p in [first.a, last.b] {
            assert!(p.x >= bb.ll.x as f64 - 1e-9 && p.x <= bb.ur.x as f64 + 1e-9);
            assert!(p.y >= bb.ll.y as f64 - 1e-9 && p.y <= bb.ur.y as f64 + 1e-9);
        }
    }

    #[test]
    fn point_door_for_obstacle_room() {
        let mut graph = RoomGraph::new();
        let free = graph.add_room(box_shape(0, 0, 100, 100), 0, RoomKind::CompleteFreeSpace);
        let obstacle = graph.add_room(
            box_shape(100, 40, 160, 60),
            0,
            RoomKind::Obstacle {
                item: 1,
                shape_index: 0,
            },
        );
        let door = graph.add_door(free, obstacle);
        assert_eq!(graph.door(door).dimension, 1);
        // door between free and obstacle room with dimension 2 would take
        // the gravity-point branch; force it to check that branch
        let door2 = graph.add_door_with_dimension(free, obstacle, 2);
        let segments = graph.door_section_segments(door2, 5.0);
        assert_eq!(segments.len(), 1);
        assert_eq!(segments[0].a, segments[0].b); // a point segment
    }
}
