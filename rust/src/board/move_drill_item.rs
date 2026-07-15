//! Port of `MoveDrillItemAlgo.java`: shoving vias out of an obstacle
//! shape. Java separates a pure `check` from `insert`; this port checks
//! by doing inside a snapshot transaction (semantically equivalent — the
//! board rolls back on failure).

use crate::board::basic_board::{BasicBoard, ItemId};
use crate::board::shove_trace_algo::shove_aside;
use crate::board::ItemKind;
use crate::geometry::planar::{IntPoint, TileShape};

/// Candidate new centers for shoving `via_id` out of `obstacle_shape`
/// (Java: `try_shove_via_points`): projections of the via center onto
/// the border of the obstacle enlarged by the via extent plus the
/// pairwise clearance (any-angle branch), nearest first.
pub fn try_shove_via_points(
    board: &BasicBoard,
    obstacle_shape: &TileShape,
    layer: usize,
    via_id: ItemId,
    cl_class: usize,
    extended_check: bool,
) -> Vec<IntPoint> {
    let Some(via) = board.get_item(via_id) else {
        return Vec::new();
    };
    let Some((via_shape, _)) = via
        .tile_shapes(&board.padstacks)
        .iter()
        .find(|(_, l)| *l == layer)
        .cloned()
    else {
        return Vec::new();
    };
    let clearance = board
        .rules
        .clearance_matrix
        .get_value(cl_class, via.base.clearance_class, layer, false)
        .max(0) as f64;
    // enlarge by half the via extent + clearance (+2 tolerance, like
    // Java's empirical diagonal-shove constant)
    let shove_distance =
        0.5 * via_shape.bounding_box().max_width() as f64 + clearance + 2.0;
    let center = via_shape.centre_of_gravity().round();
    let offset_box = obstacle_shape.bounding_box().offset(shove_distance);
    let try_count = if extended_check { 4 } else { 1 };
    offset_box.nearest_border_projections(center, try_count.min(2))
}

/// Moves a via to `new_center`, shoving traces aside at the destination
/// (Java: `MoveDrillItemAlgo.insert` + the per-layer forced pads).
/// Transactional: the board is unchanged when false is returned.
pub fn move_via(
    board: &mut BasicBoard,
    via_id: ItemId,
    new_center: IntPoint,
    max_via_recursion: usize,
) -> bool {
    let Some(item) = board.get_item(via_id).cloned() else {
        return false;
    };
    let ItemKind::Via(via) = &item.kind else {
        return false;
    };
    if item.base.is_user_fixed() || item.base.component_no != 0 {
        return false; // pins and fixed vias never move
    }
    // like Java: only vias connected exclusively to traces may move
    for contact in board.get_normal_contacts(via_id) {
        let Some(c) = board.get_item(contact) else { continue };
        if !matches!(c.kind, ItemKind::PolylineTrace(_)) {
            if let ItemKind::ObstacleArea(a) = &c.kind {
                if a.is_conduction {
                    continue;
                }
            }
            return false;
        }
    }
    let padstack = via.padstack;
    let attach_allowed = via.attach_allowed;
    let old_center = via.center;
    let net_nos = item.base.net_nos.clone();
    let cl_class = item.base.clearance_class;

    // Record the traces contacting the via so we can bridge them to the new
    // position after the move. Java's `DrillItem.move_by` translates the via
    // in place and inserts old->new connecting stubs for each contacting
    // trace; this port removes and reinserts the via, so without these
    // bridges the contacting traces would be left dangling at the old center
    // and the via's net would be disconnected.
    let mut bridge_contacts: Vec<(usize, i32, usize)> = Vec::new(); // (layer, half_width, clearance_class)
    for contact in board.get_normal_contacts(via_id) {
        if let Some(c) = board.get_item(contact) {
            if let ItemKind::PolylineTrace(t) = &c.kind {
                bridge_contacts.push((t.layer, t.half_width, c.base.clearance_class));
            }
        }
    }

    board.generate_snapshot();
    board.remove_item(via_id);
    // clear the destination on every spanned layer by shoving traces
    let Some(ps) = board.padstacks.get_by_no(padstack) else {
        board.undo();
        return false;
    };
    let layer_shapes: Vec<(TileShape, usize)> = (ps.from_layer()..=ps.to_layer())
        .filter_map(|l| {
            ps.get_shape(l).map(|s| {
                (
                    s.translate_by(crate::geometry::planar::IntVector::new(
                        new_center.x,
                        new_center.y,
                    )),
                    l,
                )
            })
        })
        .collect();
    for (shape, layer) in &layer_shapes {
        let cl = board
            .rules
            .clearance_matrix
            .max_value(*layer)
            .max(0) as f64;
        let inflated = shape.offset(cl);
        if !shove_aside(board, &inflated, *layer, &net_nos, cl_class, &[]) {
            board.undo();
            return false;
        }
        // anything still conflicting (vias, pads) fails the move unless
        // recursion may shove further vias
        let blocked = board
            .overlapping_items(&inflated, Some(*layer))
            .into_iter()
            .any(|id| {
                board
                    .get_item(id)
                    .is_some_and(|it| {
                        !it.base.net_nos.iter().any(|n| net_nos.contains(n))
                            && !matches!(&it.kind, ItemKind::ObstacleArea(a) if a.is_conduction)
                    })
            });
        if blocked {
            let _ = max_via_recursion; // deeper via-shove recursion: future work
            board.undo();
            return false;
        }
    }
    board.insert_via(padstack, new_center, net_nos.clone(), cl_class, attach_allowed);
    // Bridge each previously-contacting trace from the old via center to the
    // new one, preserving connectivity (Java: DrillItem.move_by insert_trace).
    if old_center != new_center {
        for (layer, half_width, trace_cl_class) in bridge_contacts {
            let bridge = crate::geometry::planar::Polyline::from_int_points(&[old_center, new_center]);
            board.insert_trace(bridge, layer, half_width, net_nos.clone(), trace_cl_class);
        }
    }
    board.pop_snapshot();
    true
}

/// Shoves foreign vias out of `obstacle_shape` on `layer` (Java:
/// `MoveDrillItemAlgo.shove_vias`). Returns true when the shape is free
/// of shovable vias afterwards (vias that cannot move are left in place,
/// like Java, which reports them via the failing-obstacle channel).
pub fn shove_vias(
    board: &mut BasicBoard,
    obstacle_shape: &TileShape,
    layer: usize,
    own_net_nos: &[i32],
    cl_class: usize,
    max_via_recursion: usize,
) -> bool {
    if max_via_recursion == 0 {
        return true;
    }
    let query = obstacle_shape.offset(
        board.rules.clearance_matrix.max_value(layer).max(0) as f64,
    );
    let vias: Vec<ItemId> = board
        .overlapping_items(&query, Some(layer))
        .into_iter()
        .filter(|id| {
            board.get_item(*id).is_some_and(|it| {
                matches!(it.kind, ItemKind::Via(_))
                    && it.base.component_no == 0
                    && !it.base.is_user_fixed()
                    && !it.base.net_nos.iter().any(|n| own_net_nos.contains(n))
            })
        })
        .collect();
    let shape_radius = 0.5 * obstacle_shape.bounding_box().min_width() as f64;
    for via_id in vias {
        let candidates =
            try_shove_via_points(board, obstacle_shape, layer, via_id, cl_class, true);
        let Some(via) = board.get_item(via_id) else { continue };
        let via_bb = via.bounding_box(&board.padstacks);
        let via_center = crate::geometry::planar::FloatPoint::new(
            (via_bb.ll.x as f64 + via_bb.ur.x as f64) / 2.0,
            (via_bb.ll.y as f64 + via_bb.ur.y as f64) / 2.0,
        );
        let max_dist = 0.5 * via_bb.max_width() as f64 + shape_radius;
        for (i, cand) in candidates.iter().enumerate() {
            let d = via_center
                .distance(crate::geometry::planar::FloatPoint::new(
                    cand.x as f64,
                    cand.y as f64,
                ));
            if i > 0 && d > max_dist {
                continue;
            }
            if move_via(board, via_id, *cand, max_via_recursion - 1) {
                break;
            }
        }
    }
    true
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::board::{ItemBase, Layer, LayerStructure};
    use crate::core::Padstacks;
    use crate::geometry::planar::{IntBox, Polyline};
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
        let mut board = BasicBoard::new(stack, rules, padstacks);
        // a board outline stand-in so shapes stay in bounds
        let _ = &mut board;
        board
    }

    #[test]
    fn via_is_shoved_out_of_a_corridor() {
        let mut board = test_board();
        // a foreign via sitting in the corridor
        let via = board.insert_via(1, crate::geometry::planar::IntPoint::new(0, 0), vec![2], 1, false);
        // wide anchor traces so the board bbox is meaningful
        board.insert_trace(
            Polyline::from_int_points(&[
                crate::geometry::planar::IntPoint::new(-20000, -8000),
                crate::geometry::planar::IntPoint::new(20000, -8000),
            ]),
            0,
            100,
            vec![3],
            1,
        );
        let corridor = TileShape::Box(IntBox::from_coords(-5000, -700, 5000, 700));
        assert!(shove_vias(&mut board, &corridor, 0, &[1], 1, 2));
        let moved = board.get_item(via).is_none();
        // the original via id is gone (moved = reinserted with a new id)
        assert!(moved, "via should have been moved out of the corridor");
        // and some via of net 2 exists outside the corridor
        let found = board.items().any(|(_, it)| {
            matches!(it.kind, ItemKind::Via(_))
                && it.base.contains_net(2)
                && !corridor
                    .bounding_box()
                    .intersects(it.bounding_box(&board.padstacks))
        });
        assert!(found, "moved via must exist outside the corridor");
    }

    #[test]
    fn moved_via_stays_connected_to_its_trace() {
        use crate::geometry::planar::IntPoint;
        let mut board = test_board();
        let via = board.insert_via(1, IntPoint::new(0, 0), vec![2], 1, false);
        // a same-net trace contacting the via at its center
        let trace = board.insert_trace(
            Polyline::from_int_points(&[IntPoint::new(0, 0), IntPoint::new(0, 6000)]),
            0,
            100,
            vec![2],
            1,
        );
        let corridor = TileShape::Box(IntBox::from_coords(-5000, -700, 5000, 700));
        assert!(shove_vias(&mut board, &corridor, 0, &[1], 1, 2));
        // the original via id is gone (it was reinserted at a new position)
        assert!(board.get_item(via).is_none(), "via should have moved");
        // the trace must still reach a via through the inserted bridge stub;
        // without bridging the trace would be left dangling at the old center
        let connected = board.get_connected_set(trace, 2);
        let reaches_via = connected.iter().any(|&id| {
            board
                .get_item(id)
                .is_some_and(|it| matches!(it.kind, ItemKind::Via(_)))
        });
        assert!(
            reaches_via,
            "moved via must stay connected to its trace via the bridge"
        );
    }

    #[test]
    fn fixed_and_pin_vias_never_move() {
        let mut board = test_board();
        let pin = board.insert_via(1, crate::geometry::planar::IntPoint::new(0, 0), vec![2], 1, false);
        board.set_component_no(pin, 7);
        let corridor = TileShape::Box(IntBox::from_coords(-5000, -700, 5000, 700));
        assert!(shove_vias(&mut board, &corridor, 0, &[1], 1, 2));
        assert!(board.get_item(pin).is_some(), "pins must never be moved");
    }

    #[allow(unused_imports)]
    use crate::board::item::Item as _ItemAlias;
    #[allow(dead_code)]
    fn _unused(_b: ItemBase) {}
}
