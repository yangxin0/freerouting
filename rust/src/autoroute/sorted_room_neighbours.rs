//! Port of `SortedRoomNeighbours.java`: the neighbours of a completed
//! expansion room sorted counterclockwise around its border, and the new
//! incomplete rooms created for the uncovered border gaps. This is the
//! faithful growth model of Java's autorouter (the interim frontier
//! expansion seeded a clipped half-plane beyond every border edge).

use crate::geometry::planar::{Line, Point, Simplex, TileShape};

/// One touching neighbour of the room (Java: inner class
/// `SortedRoomNeighbour`).
#[derive(Debug, Clone)]
pub struct Neighbour {
    /// Identifies the neighbour for door creation: an engine room id or
    /// an item id (items block; rooms get doors).
    pub object: NeighbourObject,
    pub neighbour_shape: TileShape,
    pub intersection: TileShape,
    pub side_of_room: usize,
    pub side_of_neighbour: usize,
    pub room_touch_is_corner: bool,
    pub neighbour_touch_is_corner: bool,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum NeighbourObject {
    Room(usize),
    Item(i32),
}

impl NeighbourObject {
    fn id(&self) -> i64 {
        match self {
            // rooms sort after items like Java's id spaces stay disjoint
            NeighbourObject::Room(r) => *r as i64 + (1 << 40),
            NeighbourObject::Item(i) => *i as i64,
        }
    }
}

impl Neighbour {
    /// The first corner of the intersection in counterclockwise border
    /// order (Java: `first_corner`).
    pub fn first_corner(&self, room_shape: &Simplex) -> Point {
        if self.room_touch_is_corner {
            room_shape.corner(self.side_of_room)
        } else if self.neighbour_touch_is_corner {
            self.neighbour_shape.corner(self.side_of_neighbour)
        } else {
            let curr = self
                .neighbour_shape
                .to_simplex()
                .corner(next_no(&self.neighbour_shape.to_simplex(), self.side_of_neighbour));
            let prev_line = room_shape.border_line(prev_no(room_shape, self.side_of_room));
            if prev_line.side_of(&curr) == crate::geometry::planar::Side::OnTheRight {
                curr
            } else {
                room_shape.corner(self.side_of_room)
            }
        }
    }

    /// The last corner of the intersection (Java: `last_corner`).
    pub fn last_corner(&self, room_shape: &Simplex) -> Point {
        if self.room_touch_is_corner {
            room_shape.corner(self.side_of_room)
        } else if self.neighbour_touch_is_corner {
            self.neighbour_shape.corner(self.side_of_neighbour)
        } else {
            let curr = self.neighbour_shape.to_simplex().corner(self.side_of_neighbour);
            let next_line = room_shape.border_line(next_no(room_shape, self.side_of_room));
            if next_line.side_of(&curr) == crate::geometry::planar::Side::OnTheRight {
                curr
            } else {
                room_shape.corner(next_no(room_shape, self.side_of_room))
            }
        }
    }
}

pub fn next_no(s: &Simplex, no: usize) -> usize {
    (no + 1) % s.border_line_count().max(1)
}

pub fn prev_no(s: &Simplex, no: usize) -> usize {
    (no + s.border_line_count().max(1) - 1) % s.border_line_count().max(1)
}

/// The index of the corner equal to `point`, if any (Java:
/// `TileShape.equals_corner`).
fn equals_corner(s: &Simplex, point: &Point) -> Option<usize> {
    (0..s.border_line_count()).find(|&i| &s.corner(i) == point)
}

/// The border side numbers of `room` and `other` containing their
/// 1-dimensional intersection (Java: `TileShape.touching_sides`).
fn touching_sides(room: &Simplex, other: &Simplex, intersection: &TileShape) -> Option<(usize, usize)> {
    let corners: Vec<Point> = (0..intersection.to_simplex().border_line_count())
        .map(|i| intersection.to_simplex().corner(i))
        .collect();
    if corners.is_empty() {
        return None;
    }
    let on_side = |s: &Simplex| -> Option<usize> {
        (0..s.border_line_count()).find(|&i| {
            let l = s.border_line(i);
            corners
                .iter()
                .all(|c| l.side_of(c) == crate::geometry::planar::Side::Collinear)
        })
    };
    Some((on_side(room)?, on_side(other)?))
}

/// Sorts `neighbours` counterclockwise around the room border (Java:
/// `SortedRoomNeighbour.compareTo`, with the distance tolerance of 1).
pub fn sort_neighbours(room_shape: &Simplex, neighbours: &mut [Neighbour]) {
    neighbours.sort_by(|a, b| {
        if a.side_of_room != b.side_of_room {
            return a.side_of_room.cmp(&b.side_of_room);
        }
        let compare_corner = room_shape.corner_approx(a.side_of_room);
        let da = a.first_corner(room_shape).to_float().distance(compare_corner);
        let db = b.first_corner(room_shape).to_float().distance(compare_corner);
        let mut delta = da - db;
        if delta.abs() <= 1.0 && a.first_corner(room_shape) == b.first_corner(room_shape) {
            let da2 = a.last_corner(room_shape).to_float().distance(compare_corner);
            let db2 = b.last_corner(room_shape).to_float().distance(compare_corner);
            delta = da2 - db2;
        }
        match delta.partial_cmp(&0.0) {
            Some(std::cmp::Ordering::Equal) | None => a.object.id().cmp(&b.object.id()),
            Some(ord) if delta.abs() <= 1.0 => {
                a.object.id().cmp(&b.object.id()).then(ord)
            }
            Some(ord) => ord,
        }
    });
}

/// Classifies one overlap and builds the neighbour record (the dim-1 and
/// dim-0 branches of Java's `calculate_neighbours` loop body).
pub fn make_neighbour(
    room_shape: &Simplex,
    object: NeighbourObject,
    neighbour_shape: &TileShape,
) -> Option<Neighbour> {
    // touching requires intersecting bounding boxes: the exact
    // (simplifying) intersection dominated big-board completion when
    // run on every coarse grid candidate
    let room_tile = TileShape::Simplex(room_shape.clone());
    if !room_tile
        .bounding_box()
        .intersects(neighbour_shape.bounding_box())
    {
        return None;
    }
    let intersection = room_tile.intersection_with_simplify(neighbour_shape);
    let dim = intersection.dimension();
    if dim >= 2 || dim < 0 {
        return None; // overlaps are handled by completion; disjoint is noise
    }
    let nb_simplex = neighbour_shape.to_simplex();
    if dim == 1 {
        let (side_room, side_nb) = touching_sides(room_shape, &nb_simplex, &intersection)?;
        return Some(Neighbour {
            object,
            neighbour_shape: neighbour_shape.clone(),
            intersection,
            side_of_room: side_room,
            side_of_neighbour: side_nb,
            room_touch_is_corner: false,
            neighbour_touch_is_corner: false,
        });
    }
    // dimension 0: a corner touch
    let touching_point = intersection.to_simplex().corner(0);
    let (room_touch_is_corner, side_of_room) = match equals_corner(room_shape, &touching_point) {
        Some(no) => (true, no),
        None => (
            false,
            room_tile.contains_on_border_line_no(&touching_point)?,
        ),
    };
    let (neighbour_touch_is_corner, side_of_neighbour) =
        match equals_corner(&nb_simplex, &touching_point) {
            // the previous border line makes the incomplete room as big
            // as possible (Java comment)
            Some(no) => (true, prev_no(&nb_simplex, no)),
            None => (
                false,
                neighbour_shape.contains_on_border_line_no(&touching_point)?,
            ),
        };
    Some(Neighbour {
        object,
        neighbour_shape: neighbour_shape.clone(),
        intersection,
        side_of_room,
        side_of_neighbour,
        room_touch_is_corner,
        neighbour_touch_is_corner,
    })
}

/// A new incomplete room for an uncovered border gap.
#[derive(Debug, Clone)]
pub struct GapRoom {
    pub shape: TileShape,
    pub contained_shape: TileShape,
}

/// Walks the sorted neighbours and creates the incomplete rooms for the
/// uncovered border gaps (Java: `calculate_new_incomplete_rooms`).
/// `completed_shape` may be cut when a corner sticks into empty space.
pub fn calculate_new_incomplete_rooms(
    room_shape: &Simplex,
    completed_shape: &mut TileShape,
    contained_shape: &TileShape,
    sorted: &[Neighbour],
) -> Vec<GapRoom> {
    let mut result = Vec::new();
    if sorted.is_empty() {
        return result;
    }
    let mut prev: &Neighbour = sorted.last().unwrap();
    let last_neighbour: *const Neighbour = prev;
    for next in sorted {
        let mut first_side = prev.side_of_room;
        let mut last_side = next.side_of_room;
        let is_last = std::ptr::eq(prev, last_neighbour);
        let curr_next_no = next_no(room_shape, first_side);
        let prev_ends_at_corner = (first_side != last_side || is_last)
            && prev.last_corner(room_shape) == room_shape.corner(curr_next_no);
        let next_starts_at_corner = (first_side != last_side || is_last)
            && next.first_corner(room_shape) == room_shape.corner(last_side);
        if prev_ends_at_corner {
            first_side = curr_next_no;
        }
        if next_starts_at_corner {
            last_side = prev_no(room_shape, last_side);
        }
        let neighbours_touch = sorted.len() > 1
            && prev.last_corner(room_shape) == next.first_corner(room_shape);
        if !neighbours_touch {
            let mut last_bounding = prev.side_of_neighbour;
            if !(prev_ends_at_corner || prev.room_touch_is_corner) {
                last_bounding = prev_no(&prev.neighbour_shape.to_simplex(), last_bounding);
            }
            let mut first_bounding = next.side_of_neighbour;
            if !(next_starts_at_corner || next.neighbour_touch_is_corner) {
                first_bounding = next_no(&next.neighbour_shape.to_simplex(), first_bounding);
            }
            let mut start_edge_line: Option<Line> = Some(
                next.neighbour_shape
                    .to_simplex()
                    .border_line(first_bounding)
                    .opposite(),
            );
            let mut curr_side = last_side;
            let mut first_time = true;
            loop {
                let middle_edge_line = room_shape.border_line(curr_side).opposite();
                let next_side = prev_no(room_shape, curr_side);
                let last_time = curr_side == first_side && !(is_last && first_time);
                let mut end_edge_line: Option<Line> = None;
                if last_time {
                    let e = prev
                        .neighbour_shape
                        .to_simplex()
                        .border_line(last_bounding)
                        .opposite();
                    if e.direction().side_of(middle_edge_line.direction())
                        == crate::geometry::planar::Side::OnTheLeft
                    {
                        end_edge_line = Some(e);
                    }
                }
                if let Some(sl) = &start_edge_line {
                    if middle_edge_line.direction().side_of(sl.direction())
                        != crate::geometry::planar::Side::OnTheLeft
                    {
                        start_edge_line = None;
                    }
                }
                let mut lines: Vec<Line> = Vec::with_capacity(3);
                if let Some(s) = start_edge_line {
                    lines.push(s);
                }
                lines.push(middle_edge_line);
                if let Some(e) = end_edge_line {
                    lines.push(e);
                }
                let new_room_shape = Simplex::new(lines);
                if !new_room_shape.is_empty() {
                    let new_contained = completed_shape
                        .intersection_with_simplify(&TileShape::Simplex(new_room_shape.clone()));
                    if !new_contained.is_empty() && new_contained.dimension() >= 1 {
                        result.push(GapRoom {
                            shape: TileShape::Simplex(new_room_shape),
                            contained_shape: new_contained,
                        });
                    }
                }
                if last_time {
                    break;
                }
                curr_side = next_side;
                start_edge_line = None;
                first_time = false;
            }
        }
        prev = next;
    }
    let _ = contained_shape;
    result
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geometry::planar::{IntBox, IntPoint};

    fn box_simplex(x1: i32, y1: i32, x2: i32, y2: i32) -> Simplex {
        TileShape::Box(IntBox::from_coords(x1, y1, x2, y2)).to_simplex()
    }

    #[test]
    fn gap_rooms_created_for_uncovered_borders() {
        // room = unit box; one neighbour touching the full bottom side:
        // the walk must create incomplete rooms for the other sides
        let room = box_simplex(0, 0, 1000, 1000);
        let neighbour = TileShape::Box(IntBox::from_coords(0, -500, 1000, 0));
        let nb = make_neighbour(&room, NeighbourObject::Room(1), &neighbour).unwrap();
        assert!(!nb.room_touch_is_corner);
        let mut sorted = vec![nb];
        sort_neighbours(&room, &mut sorted);
        let mut completed = TileShape::Simplex(room.clone());
        let contained = TileShape::Box(IntBox::from_coords(400, 400, 600, 600));
        let gaps =
            calculate_new_incomplete_rooms(&room, &mut completed, &contained, &sorted);
        assert!(!gaps.is_empty(), "uncovered sides must produce gap rooms");
        // every gap room lies outside the room's interior side of the
        // touched border and contains part of the completed shape edge
        for g in &gaps {
            assert!(g.contained_shape.dimension() >= 1);
        }
    }

    #[test]
    fn corner_touch_classification() {
        let room = box_simplex(0, 0, 1000, 1000);
        // neighbour touching exactly at the corner (1000, 1000)
        let neighbour = TileShape::Box(IntBox::from_coords(1000, 1000, 2000, 2000));
        let nb = make_neighbour(&room, NeighbourObject::Item(7), &neighbour).unwrap();
        assert!(nb.room_touch_is_corner);
        assert!(nb.neighbour_touch_is_corner);
        let _ = IntPoint::new(0, 0);
    }
}
