//! An interim grid-based A* router over [`BasicBoard`].
//!
//! Not a port of a single Java file: freerouting's real router is the maze
//! expansion engine (`MazeSearchAlgo.java` and friends), which follows
//! incrementally. This router exists so the ported board is routable end
//! to end: it searches a coarse grid with layer changes, honoring the
//! board's exact obstacle queries, and inserts the found connection as
//! polyline traces and vias.

use std::cmp::Reverse;
use std::collections::{BinaryHeap, HashMap};

use crate::board::basic_board::{BasicBoard, ItemId};
use crate::geometry::planar::{IntBox, IntPoint, Polyline, TileShape};

#[derive(Debug, Clone)]
pub struct RouteRequest {
    pub net_no: i32,
    pub from: IntPoint,
    pub from_layer: usize,
    pub to: IntPoint,
    pub to_layer: usize,
    /// grid step of the search (board units)
    pub grid: i32,
    pub trace_half_width: i32,
    /// clearance to keep to foreign items
    pub clearance: i32,
    /// clearance class for the inserted items
    pub clearance_class: usize,
    /// 1-based padstack for layer-change vias
    pub via_padstack: usize,
    /// cost of a layer change, in grid steps
    pub via_cost: i32,
}

#[derive(Debug, PartialEq)]
pub enum RouteResult {
    /// The connection was inserted; the new item ids are returned.
    Routed(Vec<ItemId>),
    NotFound,
}

/// One node of the search graph: a grid coordinate plus layer.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord)]
struct Node {
    x: i32,
    y: i32,
    layer: usize,
}

pub struct SimpleRouter;

impl SimpleRouter {
    /// Routes a connection between two points, inserting traces and vias
    /// into the board on success.
    pub fn route(board: &mut BasicBoard, request: &RouteRequest) -> RouteResult {
        let grid = request.grid.max(1);
        // search area: bounding box of the endpoints, generously enlarged
        let area = IntBox::new(request.from, request.from)
            .union(IntBox::new(request.to, request.to))
            .offset((20 * grid) as f64);

        let start = Node {
            x: request.from.x,
            y: request.from.y,
            layer: request.from_layer,
        };
        let target = Node {
            x: request.to.x,
            y: request.to.y,
            layer: request.to_layer,
        };

        let heuristic = |n: &Node| -> i64 {
            let dx = (n.x - target.x).abs() as i64;
            let dy = (n.y - target.y).abs() as i64;
            // octile-ish admissible estimate plus layer difference
            let layer_diff = (n.layer as i64 - target.layer as i64).abs();
            dx.max(dy) + layer_diff * (request.via_cost as i64 * grid as i64)
        };

        let mut open: BinaryHeap<(Reverse<i64>, i64, Node)> = BinaryHeap::new();
        let mut g_score: HashMap<Node, i64> = HashMap::new();
        let mut came_from: HashMap<Node, Node> = HashMap::new();
        g_score.insert(start, 0);
        open.push((Reverse(heuristic(&start)), 0, start));

        let layer_count = board.layer_structure.layer_count();
        let mut found = false;
        while let Some((_, neg_g, current)) = open.pop() {
            let curr_g = -neg_g;
            if g_score.get(&current) != Some(&curr_g) {
                continue; // stale entry
            }
            // snap-to-target: if we are within one grid step, connect
            if current.layer == target.layer
                && (current.x - target.x).abs() <= grid
                && (current.y - target.y).abs() <= grid
                && Self::segment_free(board, request, current.x, current.y, target.x, target.y, current.layer)
            {
                came_from.insert(target, current);
                found = true;
                break;
            }
            // planar neighbors (8 directions)
            for (dx, dy) in [
                (grid, 0),
                (-grid, 0),
                (0, grid),
                (0, -grid),
                (grid, grid),
                (grid, -grid),
                (-grid, grid),
                (-grid, -grid),
            ] {
                let next = Node {
                    x: current.x + dx,
                    y: current.y + dy,
                    layer: current.layer,
                };
                if next.x < area.ll.x || next.x > area.ur.x || next.y < area.ll.y || next.y > area.ur.y
                {
                    continue;
                }
                let step_cost = if dx != 0 && dy != 0 {
                    (grid as i64 * 3) / 2 // diagonal
                } else {
                    grid as i64
                };
                let tentative = curr_g + step_cost;
                if g_score.get(&next).is_some_and(|&g| g <= tentative) {
                    continue;
                }
                if !Self::segment_free(board, request, current.x, current.y, next.x, next.y, current.layer)
                {
                    continue;
                }
                g_score.insert(next, tentative);
                came_from.insert(next, current);
                open.push((Reverse(tentative + heuristic(&next)), -tentative, next));
            }
            // layer changes via a via
            for next_layer in 0..layer_count {
                if next_layer == current.layer {
                    continue;
                }
                let next = Node {
                    layer: next_layer,
                    ..current
                };
                let tentative = curr_g + request.via_cost as i64 * grid as i64;
                if g_score.get(&next).is_some_and(|&g| g <= tentative) {
                    continue;
                }
                if !Self::via_free(board, request, current.x, current.y) {
                    continue;
                }
                g_score.insert(next, tentative);
                came_from.insert(next, current);
                open.push((Reverse(tentative + heuristic(&next)), -tentative, next));
            }
        }

        if !found {
            return RouteResult::NotFound;
        }

        // reconstruct the path
        let mut path = vec![target];
        let mut curr = target;
        while curr != start {
            curr = came_from[&curr];
            path.push(curr);
        }
        path.reverse();

        // insert the connection: one polyline trace per layer run, a via
        // at each layer change
        let mut new_items = Vec::new();
        let mut run: Vec<IntPoint> = vec![IntPoint::new(path[0].x, path[0].y)];
        let mut run_layer = path[0].layer;
        for w in path.windows(2) {
            let (prev, next) = (w[0], w[1]);
            if next.layer != prev.layer {
                // flush the current run and place a via
                if run.len() > 1 {
                    new_items.push(Self::insert_run(board, request, &run, run_layer));
                }
                new_items.push(board.insert_via(
                    request.via_padstack,
                    IntPoint::new(prev.x, prev.y),
                    vec![request.net_no],
                    request.clearance_class,
                    false,
                ));
                run = vec![IntPoint::new(prev.x, prev.y)];
                run_layer = next.layer;
            } else {
                run.push(IntPoint::new(next.x, next.y));
            }
        }
        if run.len() > 1 {
            new_items.push(Self::insert_run(board, request, &run, run_layer));
        }
        RouteResult::Routed(new_items)
    }

    fn insert_run(
        board: &mut BasicBoard,
        request: &RouteRequest,
        run: &[IntPoint],
        layer: usize,
    ) -> ItemId {
        // drop collinear intermediate points
        let mut corners: Vec<IntPoint> = vec![run[0]];
        for i in 1..run.len() - 1 {
            let prev = *corners.last().unwrap();
            let (a, b) = (run[i], run[i + 1]);
            let collinear = (a.x - prev.x) as i64 * (b.y - prev.y) as i64
                == (a.y - prev.y) as i64 * (b.x - prev.x) as i64;
            if !collinear {
                corners.push(a);
            }
        }
        corners.push(*run.last().unwrap());
        board.insert_trace(
            Polyline::from_int_points(&corners),
            layer,
            request.trace_half_width,
            vec![request.net_no],
            request.clearance_class,
        )
    }

    /// True if a trace segment from (x1, y1) to (x2, y2) keeps its
    /// clearance to all foreign items on `layer`.
    fn segment_free(
        board: &BasicBoard,
        request: &RouteRequest,
        x1: i32,
        y1: i32,
        x2: i32,
        y2: i32,
        layer: usize,
    ) -> bool {
        let r = request.trace_half_width + request.clearance;
        let shape = if x1 == x2 || y1 == y2 || (x1 - x2).abs() == (y1 - y2).abs() {
            // orthogonal or diagonal: use the exact offset shape
            let polyline =
                Polyline::from_two_points(IntPoint::new(x1, y1), IntPoint::new(x2, y2));
            if polyline.is_empty() {
                return Self::via_layer_free(board, request, x1, y1, layer);
            }
            match polyline.offset_shape(r, 0) {
                Some(s) => s,
                None => return false,
            }
        } else {
            // fall back to the bounding box blown up by r
            TileShape::Box(
                IntBox::from_coords(x1.min(x2), y1.min(y2), x1.max(x2), y1.max(y2))
                    .offset(r as f64),
            )
        };
        !board.is_blocked(&shape, layer, request.net_no)
    }

    fn via_layer_free(
        board: &BasicBoard,
        request: &RouteRequest,
        x: i32,
        y: i32,
        layer: usize,
    ) -> bool {
        let r = request.trace_half_width + request.clearance;
        let shape = TileShape::Box(IntBox::from_coords(x - r, y - r, x + r, y + r));
        !board.is_blocked(&shape, layer, request.net_no)
    }

    /// True if a via at (x, y) keeps its clearance on all layers.
    fn via_free(board: &BasicBoard, request: &RouteRequest, x: i32, y: i32) -> bool {
        let Some(padstack) = board.padstacks.get_by_no(request.via_padstack) else {
            return false;
        };
        let from = padstack.from_layer();
        let to = padstack.to_layer();
        for layer in from..=to {
            let Some(shape) = padstack.get_shape(layer) else {
                continue;
            };
            let query = shape
                .translate_by(crate::geometry::planar::IntVector::new(x, y))
                .enlarge(request.clearance as f64);
            if board.is_blocked(&query, layer, request.net_no) {
                return false;
            }
        }
        true
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::board::{Layer, LayerStructure};
    use crate::core::Padstacks;
    use crate::geometry::planar::{PolygonShape, PolylineArea};
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
            TileShape::Box(IntBox::from_coords(-300, -300, 300, 300)),
            0,
            1,
        );
        BasicBoard::new(stack, rules, padstacks)
    }

    fn request(from: (i32, i32, usize), to: (i32, i32, usize)) -> RouteRequest {
        RouteRequest {
            net_no: 1,
            from: IntPoint::new(from.0, from.1),
            from_layer: from.2,
            to: IntPoint::new(to.0, to.1),
            to_layer: to.2,
            grid: 500,
            trace_half_width: 100,
            clearance: 200,
            clearance_class: 1,
            via_padstack: 1,
            via_cost: 10,
        }
    }

    #[test]
    fn routes_straight_connection() {
        let mut board = test_board();
        // pads as vias of net 1
        board.insert_via(1, IntPoint::new(0, 0), vec![1], 1, false);
        board.insert_via(1, IntPoint::new(5000, 0), vec![1], 1, false);
        assert!(!board.net_is_completely_connected(1));

        let result = SimpleRouter::route(&mut board, &request((0, 0, 0), (5000, 0, 0)));
        let RouteResult::Routed(items) = result else {
            panic!("route not found");
        };
        assert!(!items.is_empty());
        assert!(board.net_is_completely_connected(1));
    }

    #[test]
    fn routes_around_keepout() {
        let mut board = test_board();
        board.insert_via(1, IntPoint::new(0, 0), vec![1], 1, false);
        board.insert_via(1, IntPoint::new(8000, 0), vec![1], 1, false);
        // a wall keepout on both layers between the pads, with a passage
        // far below
        for layer in 0..2 {
            let wall = PolylineArea::new(
                PolygonShape::from_int_points(&[
                    IntPoint::new(3800, -3000),
                    IntPoint::new(4200, -3000),
                    IntPoint::new(4200, 9000),
                    IntPoint::new(3800, 9000),
                ]),
                vec![],
            );
            board.insert_area(wall, layer, "wall", vec![], 1, false);
        }

        let result = SimpleRouter::route(&mut board, &request((0, 0, 0), (8000, 0, 0)));
        let RouteResult::Routed(_) = result else {
            panic!("route not found");
        };
        assert!(board.net_is_completely_connected(1));
        // the route must detour below the wall: some trace corner has
        // y < -3000
        let mut detoured = false;
        for (_, item) in board.items() {
            if let crate::board::ItemKind::PolylineTrace(t) = &item.kind {
                for c in t.polyline.corner_approx_arr() {
                    if c.y < -3000.0 {
                        detoured = true;
                    }
                }
            }
        }
        assert!(detoured, "route did not detour around the wall");
    }

    #[test]
    fn routes_with_layer_change() {
        let mut board = test_board();
        board.insert_via(1, IntPoint::new(0, 0), vec![1], 1, false);
        board.insert_via(1, IntPoint::new(6000, 0), vec![1], 1, false);
        // an impassable wall on layer 0 only
        let wall = PolylineArea::new(
            PolygonShape::from_int_points(&[
                IntPoint::new(2800, -20000),
                IntPoint::new(3200, -20000),
                IntPoint::new(3200, 20000),
                IntPoint::new(2800, 20000),
            ]),
            vec![],
        );
        board.insert_area(wall, 0, "wall", vec![], 1, false);

        let result = SimpleRouter::route(&mut board, &request((0, 0, 0), (6000, 0, 0)));
        let RouteResult::Routed(items) = result else {
            panic!("route not found");
        };
        assert!(board.net_is_completely_connected(1));
        // at least one via was inserted for the layer change
        let via_count = items
            .iter()
            .filter(|id| {
                matches!(
                    board.get_item(**id).map(|i| &i.kind),
                    Some(crate::board::ItemKind::Via(_))
                )
            })
            .count();
        assert!(via_count >= 1, "expected a layer-change via");
    }

    #[test]
    fn reports_unroutable() {
        let mut board = test_board();
        board.insert_via(1, IntPoint::new(0, 0), vec![1], 1, false);
        board.insert_via(1, IntPoint::new(6000, 0), vec![1], 1, false);
        // an impassable wall on both layers
        for layer in 0..2 {
            let wall = PolylineArea::new(
                PolygonShape::from_int_points(&[
                    IntPoint::new(2800, -30000),
                    IntPoint::new(3200, -30000),
                    IntPoint::new(3200, 30000),
                    IntPoint::new(2800, 30000),
                ]),
                vec![],
            );
            board.insert_area(wall, layer, "wall", vec![], 1, false);
        }
        let result = SimpleRouter::route(&mut board, &request((0, 0, 0), (6000, 0, 0)));
        assert_eq!(result, RouteResult::NotFound);
        assert!(!board.net_is_completely_connected(1));
    }
}
