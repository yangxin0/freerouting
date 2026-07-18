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
    // `shove_vias` clears a not-yet-inserted route shape. The moved via is
    // reinserted before that future route item, so the final DRC's canonical
    // cell is `(via class, pending shape class)`. Java's local projection
    // helper queries the opposite role order; use the final gate's order here
    // so an asymmetric matrix cannot admit a route that is rejected on insert.
    let clearance = board
        .rules
        .clearance_matrix
        .get_value(via.base.clearance_class, cl_class, layer, false)
        .max(0) as f64;
    // enlarge by half the via extent + clearance (+2 tolerance, like
    // Java's empirical diagonal-shove constant)
    let shove_distance = 0.5 * via_shape.bounding_box().max_width() + clearance + 2.0;
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
    if item.base.is_shove_fixed() || item.base.component_no != 0 || via.is_escape_via {
        return false; // pins and (shove-)fixed vias never move (Java:
                      // MoveDrillItemAlgo.check gates on is_shove_fixed).
                      // Escape vias also stay pinned to the SMD contact
                      // that justifies their layer-scoped DRC exception.
    }
    // like Java: only vias connected exclusively to traces may move
    for contact in board.get_normal_contacts(via_id) {
        let Some(c) = board.get_item(contact) else {
            continue;
        };
        if !matches!(c.kind, ItemKind::PolylineTrace(_)) {
            if let ItemKind::ObstacleArea(a) = &c.kind {
                if a.is_conduction && !a.is_obstacle {
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
    let via_clearance_class_explicit = item.base.clearance_class_explicit;
    // Everything inserted after this point is part of the move transaction:
    // moving a via can shove foreign traces and create substitute pieces in
    // addition to the replacement via and its bridge stubs.
    let watermark = board.next_item_id();

    // Record the traces contacting the via so we can bridge them to the new
    // position after the move. Java's `DrillItem.move_by` translates the via
    // in place and inserts old->new connecting stubs for each contacting
    // trace; this port removes and reinserts the via, so without these
    // bridges the contacting traces would be left dangling and the via's net
    // disconnected. Java bridges center-to-center because its contacts are
    // always center-exact; here a contact endpoint may sit anywhere inside
    // the pad, so the bridge departs from the ACTUAL trace endpoint.
    let mut bridge_contacts: Vec<(IntPoint, usize, i32, usize, bool, Vec<i32>)> = Vec::new();
    // (endpoint, layer, half_width, clearance_class, explicit provenance,
    // shared net set). A multi-net trace must keep every net it shared with
    // the moved via; carrying only the first one silently disconnects the
    // secondary net after a via move.
    let mut contact_traces: Vec<(ItemId, i32)> = Vec::new(); // (trace, shared net)
    for contact in board.get_normal_contacts(via_id) {
        if let Some(c) = board.get_item(contact) {
            if let ItemKind::PolylineTrace(t) = &c.kind {
                let first = t.first_corner().to_float();
                let last = t.last_corner().to_float();
                let endpoint = if first.distance(old_center.to_float())
                    <= last.distance(old_center.to_float())
                {
                    first.round()
                } else {
                    last.round()
                };
                let mut shared_nets: Vec<i32> = c
                    .base
                    .net_nos
                    .iter()
                    .copied()
                    .filter(|net| net_nos.contains(net))
                    .collect();
                shared_nets.sort_unstable();
                shared_nets.dedup();
                if shared_nets.is_empty() {
                    continue;
                }
                bridge_contacts.push((
                    endpoint,
                    t.layer,
                    t.half_width,
                    c.base.clearance_class,
                    c.base.clearance_class_explicit,
                    shared_nets.clone(),
                ));
                for net in shared_nets {
                    contact_traces.push((contact, net));
                }
            }
        }
    }
    // Deduplicate: several traces can contact the via at the same point on
    // the same layer with identical width/class, and one bridge each
    // suffices — a coincident duplicate bridge is wasteful and can itself
    // create a zero-area overlap.
    bridge_contacts
        .sort_unstable_by_key(|(p, l, hw, cl, _, nets)| (p.x, p.y, *l, *hw, *cl, nets.clone()));
    // Keep one geometric bridge only when its source semantics are identical.
    // One coincident bridge cannot represent both an inherited class and an
    // explicit item override; refuse that ambiguous move instead of rewriting
    // either source's provenance.
    let mut deduped: Vec<(IntPoint, usize, i32, usize, bool, Vec<i32>)> =
        Vec::with_capacity(bridge_contacts.len());
    for contact in bridge_contacts {
        if let Some(last) = deduped.last_mut() {
            if last.0 == contact.0
                && last.1 == contact.1
                && last.2 == contact.2
                && last.3 == contact.3
            {
                if last.4 != contact.4 {
                    return false;
                }
                // Same geometry/provenance can serve the union of all
                // shared nets. This avoids duplicate coincident bridges
                // while retaining multi-net electrical connectivity.
                for net in &contact.5 {
                    if !last.5.contains(net) {
                        last.5.push(*net);
                    }
                }
                last.5.sort_unstable();
                continue;
            }
        }
        deduped.push(contact);
    }
    let bridge_contacts = deduped;
    // every needed bridge must satisfy the board's angle restriction — a
    // 45°/90° board must not gain a free-angle stub from a via move
    let restriction = board.rules.get_trace_angle_restriction();
    if bridge_contacts.iter().any(|(p, _, _, _, _, _)| {
        *p != new_center && !restriction.segment_is_compliant(*p, new_center)
    }) {
        return false;
    }

    board.generate_snapshot();
    board.remove_item(via_id);
    // clear the destination on every spanned layer by shoving traces
    let Some(ps) = board.padstacks.get_by_no(padstack) else {
        board.rollback_snapshot();
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
        let cl = board.rules.clearance_matrix.max_value(*layer).max(0) as f64;
        let inflated = shape.offset(cl);
        if !shove_aside(board, &inflated, *layer, &net_nos, cl_class, &[]) {
            board.rollback_snapshot();
            return false;
        }
        // anything still conflicting (vias, pads) fails the move unless
        // recursion may shove further vias
        let blocked = board
            .overlapping_items(&inflated, Some(*layer))
            .into_iter()
            .any(|id| {
                board.get_item(id).is_some_and(|it| {
                    !it.base.net_nos.iter().any(|n| net_nos.contains(n))
                        && !matches!(&it.kind, ItemKind::ObstacleArea(a) if a.is_conduction && !a.is_obstacle)
                })
            });
        if blocked {
            let _ = max_via_recursion; // deeper via-shove recursion: future work
            board.rollback_snapshot();
            return false;
        }
    }
    let new_via = board.insert_via_with_provenance(
        padstack,
        new_center,
        net_nos.clone(),
        cl_class,
        attach_allowed,
        via_clearance_class_explicit,
    );
    // Bridge each previously-contacting trace from its endpoint to the new
    // center, preserving connectivity (Java: DrillItem.move_by insert_trace).
    let mut new_items = vec![new_via];
    for (endpoint, layer, half_width, trace_cl_class, trace_cl_explicit, bridge_nets) in
        bridge_contacts
    {
        if endpoint == new_center {
            continue;
        }
        let bridge = crate::geometry::planar::Polyline::from_int_points(&[endpoint, new_center]);
        if bridge.is_empty() {
            continue;
        }
        new_items.push(board.insert_trace_with_provenance(
            bridge,
            layer,
            half_width,
            bridge_nets,
            trace_cl_class,
            trace_cl_explicit,
        ));
    }
    // a same-net trace running THROUGH the new center must be split there,
    // or the via's contact never registers (contacts need a trace endpoint
    // at the pad)
    board.split_traces_at_via(new_via);
    // final gate: everything this move created must satisfy the
    // authoritative DRC pairwise rule (incl. same-net drill rules) — the
    // shove corridor above ignores same-net items and never checked the
    // bridge stubs at all
    if !board
        .item_ids_since(watermark)
        .into_iter()
        .all(|id| crate::drc::item_is_clear(board, id))
    {
        board.rollback_snapshot();
        return false;
    }
    // connectivity gate: every previously-contacting trace must still
    // REACH the moved via. The bridge departs from the ROUNDED endpoint,
    // but trace contacts require exact endpoint equality — a rational
    // endpoint (polyline_path wiring, pull-tight output) rounds to a
    // nearby point and the bridge then misses the trace electrically
    // while staying DRC-clean. A trace id that no longer exists was
    // split at the new center; its pieces end at the via by construction.
    let connected = contact_traces.iter().all(|&(tid, net)| {
        board.get_item(tid).is_none() || board.get_connected_set(tid, net).contains(&new_via)
    });
    if !connected {
        board.rollback_snapshot();
        return false;
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
    let query = obstacle_shape.offset(board.rules.clearance_matrix.max_value(layer).max(0) as f64);
    let vias: Vec<ItemId> = board
        .overlapping_items(&query, Some(layer))
        .into_iter()
        .filter(|id| {
            board.get_item(*id).is_some_and(|it| {
                matches!(it.kind, ItemKind::Via(_))
                    && it.base.component_no == 0
                    && !it.base.is_shove_fixed()
                    && !it.base.net_nos.iter().any(|n| own_net_nos.contains(n))
            })
        })
        .collect();
    let shape_radius = 0.5 * obstacle_shape.bounding_box().min_width();
    for via_id in vias {
        let candidates = try_shove_via_points(board, obstacle_shape, layer, via_id, cl_class, true);
        let Some(via) = board.get_item(via_id) else {
            continue;
        };
        let via_bb = via.bounding_box(&board.padstacks);
        let via_center = crate::geometry::planar::FloatPoint::new(
            (via_bb.ll.x as f64 + via_bb.ur.x as f64) / 2.0,
            (via_bb.ll.y as f64 + via_bb.ur.y as f64) / 2.0,
        );
        let max_dist = 0.5 * via_bb.max_width() + shape_radius;
        for (i, cand) in candidates.iter().enumerate() {
            let d = via_center.distance(crate::geometry::planar::FloatPoint::new(
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
        let mut board = BasicBoard::new(stack, rules, padstacks);
        // a board outline stand-in so shapes stay in bounds
        let _ = &mut board;
        board
    }

    #[test]
    fn via_is_shoved_out_of_a_corridor() {
        let mut board = test_board();
        // a foreign via sitting in the corridor
        let via = board.insert_via(
            1,
            crate::geometry::planar::IntPoint::new(0, 0),
            vec![2],
            1,
            false,
        );
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
    fn via_shove_projection_uses_reinserted_via_matrix_orientation() {
        let mut board = test_board();
        assert!(board.rules.clearance_matrix.append_class("strict"));
        let strict = board.rules.clearance_matrix.get_no("strict").unwrap();
        // The moved via is the existing/lower-id side relative to the future
        // route shape. Make the final `(via, pending)` cell large and its
        // transpose zero so the projection visibly changes.
        board.rules.clearance_matrix.set_value(strict, 1, 0, 1_000);
        board.rules.clearance_matrix.set_value(1, strict, 0, 0);
        let via = board.insert_via(
            1,
            crate::geometry::planar::IntPoint::new(0, 0),
            vec![2],
            strict,
            false,
        );
        let obstacle = TileShape::Box(IntBox::from_coords(-1_000, -1_000, 1_000, 1_000));
        let candidates = try_shove_via_points(&board, &obstacle, 0, via, 1, false);
        assert!(!candidates.is_empty());
        // The via pad is 800 units wide and the correct clearance is 1,000;
        // projections must therefore be on the ~2,100-unit expanded border,
        // not the ~1,100-unit border produced by the transposed zero cell.
        assert!(candidates
            .iter()
            .all(|p| p.x.abs() >= 2_000 || p.y.abs() >= 2_000));
    }

    #[test]
    fn moved_via_stays_connected_to_its_trace() {
        use crate::geometry::planar::IntPoint;
        let mut board = test_board();
        let via = board.insert_via(1, IntPoint::new(0, 0), vec![2], 1, false);
        board.set_item_clearance_class_explicit(via, true);
        // a same-net trace contacting the via at its center
        let trace = board.insert_trace(
            Polyline::from_int_points(&[IntPoint::new(0, 0), IntPoint::new(0, 6000)]),
            0,
            100,
            vec![2],
            1,
        );
        board.set_item_clearance_class_explicit(trace, true);
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
        assert!(
            board.items().any(|(_, item)| {
                matches!(item.kind, ItemKind::Via(_))
                    && item.base.contains_net(2)
                    && item.base.clearance_class_explicit
            }),
            "the replacement via must retain explicit clearance provenance"
        );
        assert!(
            board.items().any(|(_, item)| {
                matches!(item.kind, ItemKind::PolylineTrace(_))
                    && item.base.contains_net(2)
                    && item.base.clearance_class_explicit
            }),
            "the bridge trace must retain explicit clearance provenance"
        );
    }

    #[test]
    fn move_refuses_ambiguous_coincident_bridge_provenance() {
        let mut board = test_board();
        let via = board.insert_via(1, IntPoint::new(0, 0), vec![2], 1, false);
        let inherited = board.insert_trace(
            Polyline::from_int_points(&[IntPoint::new(0, 0), IntPoint::new(0, 6_000)]),
            0,
            100,
            vec![2],
            1,
        );
        let explicit = board.insert_trace(
            Polyline::from_int_points(&[IntPoint::new(0, 0), IntPoint::new(0, 6_000)]),
            0,
            100,
            vec![2],
            1,
        );
        board.set_item_clearance_class_explicit(explicit, true);

        assert!(!move_via(&mut board, via, IntPoint::new(3_000, 0), 2));
        assert!(board.get_item(via).is_some());
        assert!(
            !board
                .get_item(inherited)
                .unwrap()
                .base
                .clearance_class_explicit
        );
        assert!(
            board
                .get_item(explicit)
                .unwrap()
                .base
                .clearance_class_explicit
        );
    }

    #[test]
    fn move_is_refused_when_a_rational_endpoint_would_disconnect() {
        use crate::geometry::planar::Line;
        let mut board = test_board();
        let via = board.insert_via(1, IntPoint::new(0, 0), vec![2], 1, false);
        // A trace whose via-end corner is RATIONAL: line0 (y = x/3) meets
        // line1 (through (6000,0) and (5823,1)) at exactly (100, 100/3) —
        // inside the ±400 pad but not at its center. The bridge would
        // depart from the ROUNDED (100, 33) and miss the trace endpoint,
        // leaving the trace electrically dangling with zero DRC violations.
        let polyline = Polyline::from_lines(vec![
            Line::new(IntPoint::new(0, 0), IntPoint::new(3, 1)),
            Line::new(IntPoint::new(6000, 0), IntPoint::new(5823, 1)),
            Line::new(IntPoint::new(5000, 0), IntPoint::new(7000, 0)),
        ]);
        let trace = board.insert_trace(polyline, 0, 100, vec![2], 1);
        assert!(
            board.get_normal_contacts(via).contains(&trace),
            "the rational endpoint inside the pad must register as contact"
        );
        let moved = move_via(&mut board, via, IntPoint::new(0, -3000), 2);
        assert!(
            !moved,
            "a move whose bridge cannot reach the rational trace endpoint must be refused"
        );
        assert!(
            board.get_item(via).is_some(),
            "the refused move leaves the board unchanged"
        );
        assert!(board.net_is_completely_connected(2));
    }

    #[test]
    fn shove_fixed_vias_never_move() {
        // Java MoveDrillItemAlgo.check gates on is_shove_fixed, not only
        // user-fixed: a SHOVE_FIXED via must survive the corridor shove
        let mut board = test_board();
        let via = board.insert_via(
            1,
            crate::geometry::planar::IntPoint::new(0, 0),
            vec![2],
            1,
            false,
        );
        board.set_fixed_state(via, crate::board::FixedState::ShoveFixed);
        let corridor = TileShape::Box(IntBox::from_coords(-5000, -700, 5000, 700));
        assert!(shove_vias(&mut board, &corridor, 0, &[1], 1, 2));
        assert!(
            board.get_item(via).is_some(),
            "a shove-fixed via must never be moved"
        );
    }

    #[test]
    fn fixed_and_pin_vias_never_move() {
        let mut board = test_board();
        let pin = board.insert_via(
            1,
            crate::geometry::planar::IntPoint::new(0, 0),
            vec![2],
            1,
            false,
        );
        board.set_component_no(pin, 7);
        let corridor = TileShape::Box(IntBox::from_coords(-5000, -700, 5000, 700));
        assert!(shove_vias(&mut board, &corridor, 0, &[1], 1, 2));
        assert!(board.get_item(pin).is_some(), "pins must never be moved");
    }

    #[test]
    fn escape_vias_never_move_off_their_smd_contact() {
        let mut board = test_board();
        let via = board.insert_escape_via(
            1,
            crate::geometry::planar::IntPoint::new(0, 0),
            vec![2],
            1,
            false,
            0,
        );
        assert!(!move_via(
            &mut board,
            via,
            crate::geometry::planar::IntPoint::new(0, 3000),
            2,
        ));
        let ItemKind::Via(via_item) = &board.get_item(via).unwrap().kind else {
            unreachable!()
        };
        assert_eq!(
            via_item.center,
            crate::geometry::planar::IntPoint::new(0, 0)
        );
        assert!(via_item.is_escape_via);
        assert_eq!(via_item.escape_smd_layer, Some(0));
    }

    #[allow(unused_imports)]
    use crate::board::item::Item as _ItemAlias;
    #[allow(dead_code)]
    fn _unused(_b: ItemBase) {}
}
