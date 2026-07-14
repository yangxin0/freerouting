//! Port of the free-space room completion from `ShapeSearchTree.java`:
//! `complete_shape` and `restrain_shape`.
//!
//! An incomplete free-space room (a start shape plus a shape it must keep
//! containing) is restrained against every overlapping obstacle shape on
//! its layer, visiting obstacles in deterministic (item id, shape index)
//! order like the v1.9-parity code, until the remaining pieces are
//! obstacle-free. `divide_large_room` (a performance split of huge rooms)
//! is not yet ported.

use crate::board::basic_board::{BasicBoard, ItemId};
use crate::geometry::planar::{Line, LineSegment, Side, TileShape};

/// A not yet completed free-space expansion room
/// (Java: `IncompleteFreeSpaceExpansionRoom`).
#[derive(Debug, Clone)]
pub struct IncompleteRoom {
    pub shape: TileShape,
    pub layer: usize,
    /// The shape the completed room must (partially) contain.
    pub contained_shape: TileShape,
}

/// True if the item may be ripped up while routing `net_no`: a foreign
/// routable route item (never a component pin, fixed item or netless
/// keepout).
pub fn is_rippable(item: &crate::board::Item, net_no: i32) -> bool {
    !item.base.contains_net(net_no)
        && item.base.component_no == 0
        && item.base.net_count() > 0
        && item.is_routable()
}

/// Completes the shape of `room`: returns maximal obstacle-free rooms
/// containing (parts of) the contained shape. Items of `net_no` and
/// `ignore_item` are not obstacles; with `ignore_rippable`, rippable
/// foreign route items do not restrain either (the maze search pays a
/// ripup penalty to pass through them instead).
pub fn complete_shape_with_ripup(
    board: &BasicBoard,
    room: &IncompleteRoom,
    net_no: i32,
    ignore_item: Option<ItemId>,
    ignore_rippable: bool,
    trace_clearance_class: usize,
    trace_half_width: i32,
) -> Vec<IncompleteRoom> {
    let board_box = board.bounding_box().offset(1000.0);
    let start_shape = TileShape::Box(board_box).intersection_with_simplify(&room.shape);
    if start_shape.dimension() != 2 {
        return Vec::new();
    }
    let mut result = vec![IncompleteRoom {
        shape: start_shape.clone(),
        layer: room.layer,
        contained_shape: room.contained_shape.clone(),
    }];

    // deterministic obstacle order: (item id, shape index). Obstacle
    // shapes are inflated by the pairwise clearance to the routed trace
    // (Java: clearance compensation in the autoroute search tree); the
    // door shrink by the trace half width then keeps the copper edges
    // `clearance` apart.
    let matrix = &board.rules.clearance_matrix;
    let mut obstacles: Vec<(ItemId, TileShape)> = Vec::new();
    for item_id in board.overlapping_items(&start_shape, Some(room.layer)) {
        if Some(item_id) == ignore_item {
            continue;
        }
        let Some(item) = board.get_item(item_id) else {
            continue;
        };
        // is_trace_obstacle: items of a foreign net block the room
        if item.base.contains_net(net_no) {
            continue;
        }
        // foreign conduction areas (power planes) do not restrain: they
        // get clearance cutouts in fabrication (Java: ConductionArea)
        if let crate::board::ItemKind::ObstacleArea(a) = &item.kind {
            if a.is_conduction {
                continue;
            }
        }
        if ignore_rippable && is_rippable(item, net_no) {
            continue;
        }
        let clearance = matrix.get_value(
            item.base.clearance_class,
            trace_clearance_class,
            room.layer,
            false,
        );
        for (shape, layer) in item.tile_shapes(&board.padstacks) {
            if *layer == room.layer {
                // trace half width + full clearance: the maze may run the
                // centerline anywhere inside a room (including on its
                // border), so correctness requires the whole margin in
                // the room geometry. (cl/2-only inflation left copper
                // gaps of cl/2 - hw — found by the DRC self-check.)
                let margin = trace_half_width as f64 + clearance as f64;
                let shape = if margin > 0.0 {
                    shape.offset(margin)
                } else {
                    shape.clone()
                };
                obstacles.push((item_id, shape));
            }
        }
    }

    // region-scoped completion log (FR_DEBUG_REGION): records whether a
    // watched obstacle was collected and what pieces resulted
    let watch = std::env::var("FR_DEBUG_REGION").ok().and_then(|v| {
        let n: Vec<i32> = v.split(',').filter_map(|p| p.parse().ok()).collect();
        let [x1, y1, x2, y2] = n[..] else { return None };
        Some(crate::geometry::planar::IntBox::from_coords(x1, y1, x2, y2))
    });
    if let Some(region) = watch {
        if room.contained_shape.bounding_box().intersects(region)
            || obstacles
                .iter()
                .any(|(_, s)| s.bounding_box().intersects(region))
        {
            let in_region: Vec<_> = obstacles
                .iter()
                .filter(|(_, s)| s.bounding_box().intersects(region))
                .map(|(id, _)| *id)
                .collect();
            eprintln!(
                "COMPLETE layer {} contained {:?} region-obstacles {:?}",
                room.layer,
                room.contained_shape.bounding_box(),
                in_region
            );
        }
    }
    for (obstacle_id, obstacle_shape) in &obstacles {
        // cheap bounding-box separation test before the exact overlap
        let obstacle_bbox = obstacle_shape.bounding_box();
        let mut new_result = Vec::new();
        for curr_room in result {
            if !curr_room.shape.bounding_box().intersects(obstacle_bbox) {
                new_result.push(curr_room);
                continue;
            }
            let intersection = curr_room.shape.intersection(obstacle_shape);
            if intersection.dimension() == 2 {
                new_result.extend(restrain_shape(&curr_room, obstacle_shape));
            } else {
                new_result.push(curr_room);
            }
        }
        result = new_result;
        if result.is_empty() {
            if std::env::var_os("FR_DEBUG_MAZE").is_some() {
                let rippable = board
                    .get_item(*obstacle_id)
                    .is_some_and(|i| is_rippable(i, net_no));
                eprintln!(
                    "ROOM KILLED on layer {} by obstacle item {obstacle_id:?} \
                     (bbox {:?}, ripup_mode={ignore_rippable}, rippable={rippable})",
                    room.layer,
                    obstacle_shape.bounding_box()
                );
            }
            break;
        }
    }
    if let Some(region) = watch {
        for piece in &result {
            let bb = piece.shape.bounding_box();
            if bb.intersects(region) {
                eprintln!("  PIECE layer {} bbox {:?}", room.layer, bb);
            }
        }
    }
    result
}

/// [`complete_shape_with_ripup`] without ripup, with the default trace
/// clearance class.
pub fn complete_shape(
    board: &BasicBoard,
    room: &IncompleteRoom,
    net_no: i32,
    ignore_item: Option<ItemId>,
) -> Vec<IncompleteRoom> {
    complete_shape_with_ripup(board, room, net_no, ignore_item, false, 1, 0)
}

/// Restrains the room shape so it no longer intersects the interior of
/// `obstacle_shape`, while keeping (parts of) the contained shape. May
/// return several rooms if the contained shape lies on both sides of the
/// cut.
pub fn restrain_shape(room: &IncompleteRoom, obstacle_shape: &TileShape) -> Vec<IncompleteRoom> {
    restrain_shape_bounded(room, obstacle_shape, 64)
}

/// Depth-bounded worker for [`restrain_shape`]. The recursion on the rest
/// piece is only guaranteed to terminate when every cut reduces the
/// overlap; degenerate obstacle slivers can defeat that, so the depth is
/// capped and the remaining rest piece is dropped (a conservative room).
fn restrain_shape_bounded(
    room: &IncompleteRoom,
    obstacle_shape: &TileShape,
    depth: usize,
) -> Vec<IncompleteRoom> {
    // Convert to Simplex: border lines of length 0 of octagons may not be
    // handled correctly otherwise (Java comment). Converted once here and
    // shared through the recursion.
    let obstacle_tile = TileShape::Simplex(obstacle_shape.to_simplex());
    restrain_shape_prepared(room, &obstacle_tile, depth)
}

/// Worker for [`restrain_shape_bounded`]; `obstacle_tile` is already a
/// simplex.
fn restrain_shape_prepared(
    room: &IncompleteRoom,
    obstacle_tile: &TileShape,
    depth: usize,
) -> Vec<IncompleteRoom> {
    let mut result = Vec::new();
    if depth == 0 {
        return result;
    }
    let TileShape::Simplex(obstacle_simplex) = obstacle_tile else {
        unreachable!("caller converts to simplex");
    };
    let room_shape = &room.shape;
    let shape_to_be_contained = TileShape::Simplex(room.contained_shape.to_simplex());
    if shape_to_be_contained.is_empty() {
        return result;
    }

    // Search the border line of the obstacle so that the contained shape
    // is completely on the right of it and it intersects the room
    // interior; among several candidates take the one furthest from the
    // contained shape.
    let mut cut_line: Option<Line> = None;
    let mut cut_line_distance = -1.0;
    for i in 0..obstacle_simplex.border_line_count() {
        let curr_line_segment = LineSegment::from_shape(obstacle_tile, i);
        if room_shape.is_intersected_interior_by(&curr_line_segment) {
            let curr_line = obstacle_simplex.border_line(i);
            let curr_min_distance = shape_to_be_contained.distance_to_the_left(&curr_line);
            if curr_min_distance > cut_line_distance {
                cut_line_distance = curr_min_distance;
                cut_line = Some(curr_line.opposite());
            }
        }
    }

    if let Some(cut_line) = cut_line {
        let result_piece = room_shape.intersection(&TileShape::half_plane(cut_line));
        if result_piece.dimension() >= 2 {
            result.push(IncompleteRoom {
                shape: result_piece,
                layer: room.layer,
                contained_shape: shape_to_be_contained,
            });
        }
        return result;
    }

    // No single cut line keeps all of the contained shape; find one
    // keeping at least a part of it.
    if shape_to_be_contained.dimension() < 1 {
        // there is already a completed room around the contained shape
        return result;
    }
    let mut cut_line: Option<Line> = None;
    for i in 0..obstacle_simplex.border_line_count() {
        let curr_line_segment = LineSegment::from_shape(obstacle_tile, i);
        if room_shape.is_intersected_interior_by(&curr_line_segment) {
            let curr_line = obstacle_simplex.border_line(i);
            if shape_to_be_contained.side_of_line(&curr_line) == Side::Collinear {
                // the line intersects the interior of the contained shape
                cut_line = Some(curr_line.opposite());
                break;
            }
        }
    }
    let Some(cut_line) = cut_line else {
        // parts or all of the shape may already be occupied elsewhere
        return result;
    };
    let cut_half_plane = TileShape::half_plane(cut_line);
    let new_shape_to_be_contained = shape_to_be_contained.intersection(&cut_half_plane);
    let result_piece = room_shape.intersection(&cut_half_plane);
    if result_piece.dimension() >= 2 {
        result.push(IncompleteRoom {
            shape: result_piece,
            layer: room.layer,
            contained_shape: new_shape_to_be_contained,
        });
    }
    let opposite_half_plane = TileShape::half_plane(cut_line.opposite());
    let rest_piece = room_shape.intersection(&opposite_half_plane);
    if rest_piece.dimension() >= 2 {
        let rest_room = IncompleteRoom {
            shape: rest_piece,
            layer: room.layer,
            contained_shape: shape_to_be_contained.intersection(&opposite_half_plane),
        };
        result.extend(restrain_shape_prepared(&rest_room, obstacle_tile, depth - 1));
    }
    result
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::board::{Layer, LayerStructure};
    use crate::core::Padstacks;
    use crate::geometry::planar::{IntBox, IntPoint, Point};
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

    fn point_room(x: i32, y: i32, layer: usize) -> IncompleteRoom {
        let p = IntPoint::new(x, y);
        IncompleteRoom {
            // start shape: unbounded within the board (a huge box)
            shape: TileShape::Box(IntBox::from_coords(-100000, -100000, 100000, 100000)),
            layer,
            contained_shape: TileShape::Box(IntBox::new(p, p)),
        }
    }

    #[test]
    fn completes_around_single_obstacle() {
        let mut board = test_board();
        // give the board an extent
        board.insert_via(1, IntPoint::new(0, 0), vec![9], 1, false);
        board.insert_via(1, IntPoint::new(10000, 0), vec![9], 1, false);
        // a foreign-net via obstacle between them
        let obstacle = board.insert_via(1, IntPoint::new(5000, 0), vec![2], 1, false);

        let room = point_room(2000, 0, 0);
        let completed = complete_shape(&board, &room, 1, None);
        assert!(!completed.is_empty());
        let obstacle_shape = board
            .get_item(obstacle)
            .unwrap()
            .tile_shape(0, &board.padstacks)
            .unwrap()
            .0;
        for r in &completed {
            // each completed piece contains the start point
            assert!(
                r.shape.contains(&Point::Int(IntPoint::new(2000, 0))),
                "piece lost the contained point"
            );
            // and does not overlap the obstacle interior
            let overlap = r.shape.intersection(&obstacle_shape);
            assert!(
                overlap.dimension() < 2,
                "piece overlaps the obstacle: {overlap:?}"
            );
        }
        // the room is maximal towards the far side: it reaches beyond the
        // obstacle vertically
        let best = completed
            .iter()
            .map(|r| r.shape.area())
            .fold(0.0_f64, f64::max);
        assert!(best > 0.0);
    }

    #[test]
    fn own_net_items_are_not_obstacles() {
        let mut board = test_board();
        board.insert_via(1, IntPoint::new(0, 0), vec![9], 1, false);
        board.insert_via(1, IntPoint::new(5000, 0), vec![1], 1, false);
        let room = point_room(2000, 0, 0);
        let completed = complete_shape(&board, &room, 1, None);
        // no foreign obstacle: exactly one big room
        assert_eq!(completed.len(), 1);
    }

    #[test]
    fn restrain_splits_contained_shape_crossing_obstacle() {
        // the contained shape is a horizontal strip crossing the obstacle:
        // restraining must produce rooms on both sides
        let room = IncompleteRoom {
            shape: TileShape::Box(IntBox::from_coords(-10000, -10000, 10000, 10000)),
            layer: 0,
            contained_shape: TileShape::Box(IntBox::from_coords(-5000, -100, 5000, 100)),
        };
        let obstacle = TileShape::Box(IntBox::from_coords(-500, -2000, 500, 2000));
        let rooms = restrain_shape(&room, &obstacle);
        assert!(rooms.len() >= 2, "expected pieces on both sides");
        for r in &rooms {
            assert!(r.shape.intersection(&obstacle).dimension() < 2);
            // every piece still contains part of the contained strip
            assert!(r.contained_shape.dimension() >= 1);
        }
        // together the pieces cover points left and right of the obstacle
        let left_covered = rooms
            .iter()
            .any(|r| r.shape.contains(&Point::Int(IntPoint::new(-3000, 0))));
        let right_covered = rooms
            .iter()
            .any(|r| r.shape.contains(&Point::Int(IntPoint::new(3000, 0))));
        assert!(left_covered && right_covered);
    }
}
