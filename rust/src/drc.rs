//! Port of the `drc/` package: `DesignRulesChecker` collecting clearance
//! violations and unconnected items, and `DrcReport` serializing them in
//! the KiCad DRC v1 JSON format (`https://schemas.kicad.org/drc.v1.json`)
//! exactly like the Java report.

use crate::board::basic_board::{BasicBoard, ItemId, LineageViolationKey, LineageViolationRegion};
use crate::board::ItemKind;
use crate::geometry::planar::{FloatPoint, Side, TileShape};

/// Relative tolerance absorbing floating-point rounding in the Euclidean
/// copper-distance check. Unit-independent: it scales with the required
/// clearance, never with the board's resolution. The router's own
/// clearance gates use the same tolerance, so a route the maze accepts
/// is never a (sub-unit) violation for the DRC.
pub(crate) const DISTANCE_EPS: f64 = 1e-6;

/// True when a measured copper distance violates `required` clearance,
/// beyond floating-point noise. One predicate for the DRC and the
/// router's insert gates.
pub(crate) fn violates(distance: f64, required: f64) -> bool {
    distance < required - required.max(1.0) * DISTANCE_EPS
}

/// A drill item (via or pin — both are `ItemKind::Via` here, as in Java's
/// `DrillItem` hierarchy).
pub(crate) fn is_drill(kind: &ItemKind) -> bool {
    matches!(kind, ItemKind::Via(_))
}

/// A component pin (Java `Pin`): a drill item owned by a component. Routing
/// vias have `component_no == 0`; the DSN/KiCad importers tag pins with a
/// nonzero component. This is the only pin-vs-via proxy in the collapsed model.
pub(crate) fn is_pin(item: &crate::board::Item) -> bool {
    is_drill(&item.kind) && item.base.component_no > 0
}

/// A single-layer (SMD) drill item: Java `DrillItem.drill_allowed()` is
/// `first_layer() == last_layer()`.
pub(crate) fn drill_allowed(item: &crate::board::Item, padstacks: &crate::core::Padstacks) -> bool {
    item.first_layer(padstacks) == item.last_layer(padstacks)
}

/// The clearance the pair (`item`, `other`) must keep on `layer`, or `None`
/// when the two items do not constrain each other — Java's `Item.is_obstacle`
/// collapsed into one predicate shared by the DRC, the optimizer's local gate
/// and the via/move insert gates:
/// - same-net traces and areas never constrain; same-net drill pairs do,
///   at the `*_same_net` rule value when one exists, unless the attach
///   exemption applies (attach-allowed via on a drillable same-net SMD pin,
///   or a marked escape via on its one recorded SMD layer);
/// - two constraint areas do not clear against each other, keepouts skip
///   component pins, non-obstacle conduction areas do not participate, and
///   via keepouts constrain only vias;
/// - the matrix lookup is canonicalized internally to
///   `(lower-id, higher-id)`. Java reaches the same cell by visiting the
///   higher-ID item first and querying `(other, this)`; callers cannot
///   accidentally transpose an asymmetric matrix by swapping arguments.
pub(crate) fn required_clearance(
    board: &BasicBoard,
    item: &crate::board::Item,
    other: &crate::board::Item,
    layer: usize,
) -> Option<f64> {
    let mut same_net_required: Option<f64> = None;
    if other.base.shares_net(&item.base) {
        // Same-net: Java `Trace.is_obstacle` is always false for a same-net
        // item, so a pair involving a trace or an area is never a violation.
        // Only drill-item (via/pin) pairs remain.
        if !(is_drill(&item.kind) && is_drill(&other.kind)) {
            return None;
        }
        // Via<->Via and Pin<->Pin are obstacles to each other; the only Java
        // exception (Via/Pin.is_obstacle) is an attach_allowed via on a
        // same-net drillable SMD pin (fanout).
        let a_pin = is_pin(item);
        let b_pin = is_pin(other);
        let attach =
            |it: &crate::board::Item| matches!(&it.kind, ItemKind::Via(v) if v.attach_allowed);
        let escape_on = |it: &crate::board::Item| {
            matches!(
                &it.kind,
                ItemKind::Via(v)
                    if v.is_escape_via && v.escape_smd_layer == Some(layer)
            )
        };
        let exempt = (!a_pin
            && (attach(item) || escape_on(item))
            && b_pin
            && drill_allowed(other, &board.padstacks))
            || (!b_pin
                && (attach(other) || escape_on(other))
                && a_pin
                && drill_allowed(item, &board.padstacks));
        if exempt {
            return None;
        }
        // classify each drill item (routing via / through-pin / smd pad)
        // and look up the same-net clearance for the pair
        let ic = |it: &crate::board::Item| -> crate::rules::ItemClass {
            if !is_pin(it) {
                crate::rules::ItemClass::Via
            } else if drill_allowed(it, &board.padstacks) {
                crate::rules::ItemClass::Smd
            } else {
                crate::rules::ItemClass::Pin
            }
        };
        same_net_required = board
            .rules
            .get_same_net_clearance(ic(item), ic(other))
            .map(|v| v as f64);
    }
    let item_obstacle = matches!(&item.kind, ItemKind::ObstacleArea(_));
    let other_obstacle = matches!(&other.kind, ItemKind::ObstacleArea(_));
    // Two constraint areas do not clear against each other.
    if item_obstacle && other_obstacle {
        return None;
    }
    // Java `ObstacleArea.is_obstacle` is true only for a foreign Trace or
    // (routing) Via — NOT a component Pin.
    if (item_obstacle && is_pin(other)) || (other_obstacle && is_pin(item)) {
        return None;
    }
    // A conduction area (power plane) participates only when it is also
    // flagged an obstacle (Java `ConductionArea.is_obstacle`); via keepouts
    // constrain via placement only.
    if let ItemKind::ObstacleArea(a) = &item.kind {
        if a.is_conduction && !a.is_obstacle {
            return None;
        }
        if a.via_only && !matches!(other.kind, ItemKind::Via(_)) {
            return None;
        }
    }
    if let ItemKind::ObstacleArea(a) = &other.kind {
        if a.is_conduction && !a.is_obstacle {
            return None;
        }
        if a.via_only && !matches!(item.kind, ItemKind::Via(_)) {
            return None;
        }
    }
    // A `*_same_net` rule value takes precedence for a same-net drill pair;
    // otherwise the ordinary (foreign-net) matrix value applies.
    Some(same_net_required.unwrap_or_else(|| {
        let (lower, higher) = if item.base.id_no <= other.base.id_no {
            (item, other)
        } else {
            (other, item)
        };
        board.rules.clearance_matrix.get_value(
            lower.base.clearance_class,
            higher.base.clearance_class,
            layer,
            false,
        ) as f64
    }))
}

/// Clearance required when `new_class` belongs to an item that is about to
/// be inserted.  Item ids are monotonically increasing, therefore the new
/// item is always the higher-id side of the pair. Java visits that item first
/// (its `Item.compareTo` reverses IDs), then looks up `(other, this)`, so the
/// stable semantic order is `(existing lower-id, new higher-id)`. Keeping this
/// adapter beside `required_clearance` prevents insertion preflights from
/// querying the transposed cell of an asymmetric matrix.
pub(crate) fn clearance_for_new_item(
    board: &BasicBoard,
    existing: &crate::board::Item,
    new_class: usize,
    layer: usize,
) -> f64 {
    board
        .rules
        .clearance_matrix
        .get_value(existing.base.clearance_class, new_class, layer, false)
        .max(0) as f64
}

/// Captures the authoritative severity of every existing violation. Identity
/// alone is insufficient for transactional optimization: a candidate can
/// keep the same pair/layer while moving the copper substantially closer.
/// The positive deficit is stable across the final DRC and is compared before
/// accepting a candidate board.
#[allow(dead_code)]
pub(crate) fn violation_snapshot(
    board: &BasicBoard,
) -> std::collections::HashMap<(ItemId, ItemId, usize), f64> {
    clearance_violations(board)
        .into_iter()
        .map(|v| {
            (
                (v.first_item, v.second_item, v.layer),
                (v.required_clearance - v.actual_distance).max(0.0),
            )
        })
        .collect()
}

#[cfg(test)]
pub(crate) fn violation_keys(
    board: &BasicBoard,
) -> std::collections::HashSet<(ItemId, ItemId, usize)> {
    violation_snapshot(board).into_keys().collect()
}

/// True when `id`'s copper keeps the required pairwise clearance to every
/// other board item — the authoritative DRC rule (`required_clearance`),
/// including same-net drill rules. It is retained as a small diagnostic and
/// unit-test oracle; transactional routing uses the lineage-aware delta gate
/// below so inherited violations remain distinguishable from new ones.
#[allow(dead_code)]
pub(crate) fn item_is_clear(board: &BasicBoard, id: ItemId) -> bool {
    let Some(item) = board.get_item(id) else {
        return true;
    };
    for (s, l) in item.tile_shapes(&board.padstacks) {
        let radius = board
            .rules
            .clearance_matrix
            .max_value(*l)
            .max(board.rules.max_same_net_clearance())
            .max(0) as f64;
        for oid in board.overlapping_items(&s.offset(radius), Some(*l)) {
            if oid == id {
                continue;
            }
            let Some(other) = board.get_item(oid) else {
                continue;
            };
            // `required_clearance` canonicalizes IDs to Java's semantic
            // `(lower-id, higher-id)` cell regardless of traversal order.
            let required = required_clearance(board, item, other, *l);
            let Some(cl) = required else {
                continue;
            };
            let check = s.offset(cl);
            if other.tile_shapes(&board.padstacks).iter().any(|(os, ol)| {
                ol == l
                    && os.intersection(&check).dimension() >= 2
                    && violates(s.euclidean_distance_to(os), cl)
            }) {
                return false;
            }
        }
    }
    true
}

/// One concrete violating shape pair used by replacement transactions. The
/// shape pair records the whole pre-existing contact region; retaining only a
/// lineage-pair key would let a shove relocate a violation elsewhere between
/// the same two ancestors and incorrectly call it inherited, while retaining
/// only one closest point would reject an otherwise unchanged trace split.
pub(crate) struct LineageViolationProbe {
    pub key: LineageViolationKey,
    pub deficit: f64,
    pub first_item: ItemId,
    pub second_item: ItemId,
    pub first_lineage: ItemId,
    pub second_lineage: ItemId,
    pub first_shape: TileShape,
    pub second_shape: TileShape,
    pub first_witness: FloatPoint,
    pub second_witness: FloatPoint,
}

fn closest_witness(first: &TileShape, second: &TileShape) -> (FloatPoint, FloatPoint, f64) {
    if first.intersects(second) {
        let point = first.intersection(second).centre_of_gravity();
        return (point, point, 0.0);
    }
    let first_corners = first.corner_approx_arr();
    let second_corners = second.corner_approx_arr();
    if first_corners.is_empty() || second_corners.is_empty() {
        return (
            first.centre_of_gravity(),
            second.centre_of_gravity(),
            f64::MAX,
        );
    }
    let projection = |point: FloatPoint, a: FloatPoint, b: FloatPoint| {
        let dx = b.x - a.x;
        let dy = b.y - a.y;
        let length_square = dx * dx + dy * dy;
        if length_square <= 0.0 {
            return a;
        }
        let factor =
            (((point.x - a.x) * dx + (point.y - a.y) * dy) / length_square).clamp(0.0, 1.0);
        FloatPoint::new(a.x + factor * dx, a.y + factor * dy)
    };
    let mut best = (first_corners[0], second_corners[0], f64::MAX);
    for (corners, other_corners, reversed) in [
        (&first_corners, &second_corners, false),
        (&second_corners, &first_corners, true),
    ] {
        for &point in corners {
            for index in 0..other_corners.len() {
                let projected = projection(
                    point,
                    other_corners[index],
                    other_corners[(index + 1) % other_corners.len()],
                );
                let distance = point.distance(projected);
                if distance < best.2 {
                    best = if reversed {
                        (projected, point, distance)
                    } else {
                        (point, projected, distance)
                    };
                }
            }
        }
    }
    best
}

/// Finds the concrete violations involving one item. This is local in the
/// spatial tree and is used only when a transaction removes an old item or
/// validates a newly born replacement.
pub(crate) fn lineage_violations_for_item(
    board: &BasicBoard,
    id: ItemId,
) -> Vec<LineageViolationProbe> {
    let Some(item) = board.get_item(id) else {
        return Vec::new();
    };
    let mut result = Vec::new();
    for (shape, layer) in item.tile_shapes(&board.padstacks) {
        let search_radius = board
            .rules
            .clearance_matrix
            .max_value(*layer)
            .max(board.rules.max_same_net_clearance())
            .max(0) as f64;
        for other_id in board.overlapping_items(&shape.offset(search_radius), Some(*layer)) {
            if other_id == id {
                continue;
            }
            let Some(other) = board.get_item(other_id) else {
                continue;
            };
            let Some(required) = required_clearance(board, item, other, *layer) else {
                continue;
            };
            for (other_shape, other_layer) in other.tile_shapes(&board.padstacks) {
                if other_layer != layer
                    || other_shape
                        .intersection(&shape.offset(required))
                        .dimension()
                        < 2
                {
                    continue;
                }
                let (first_witness, second_witness, actual) = closest_witness(shape, other_shape);
                if !violates(actual, required) {
                    continue;
                }
                let first_lineage = item.base.lineage_no;
                let second_lineage = other.base.lineage_no;
                let key = if first_lineage <= second_lineage {
                    (first_lineage, second_lineage, *layer)
                } else {
                    (second_lineage, first_lineage, *layer)
                };
                result.push(LineageViolationProbe {
                    key,
                    deficit: (required - actual).max(0.0),
                    first_item: id,
                    second_item: other_id,
                    first_lineage,
                    second_lineage,
                    first_shape: shape.clone(),
                    second_shape: other_shape.clone(),
                    first_witness,
                    second_witness,
                });
            }
        }
    }
    result
}

fn source_contains_witness(
    source_shapes: &std::collections::HashMap<(ItemId, usize), Vec<TileShape>>,
    lineage: ItemId,
    layer: usize,
    witness: FloatPoint,
) -> bool {
    source_shapes.get(&(lineage, layer)).is_some_and(|shapes| {
        shapes
            .iter()
            .any(|shape| shape.side_of_border(witness, 1e-6) != Side::OnTheLeft)
    })
}

// Witnesses are produced by floating-point projection against integer-grid
// polygons.  Permit one board unit of boundary roundoff (far below a normal
// trace width) so a split piece whose closest point lands exactly on an old
// edge is still recognized as the inherited contact.
const REGION_EPS: f64 = 1.0;

/// Returns the largest original deficit among concrete shape-pair regions
/// containing the current closest pair. A single closest point is too narrow
/// for a trace split: each new piece can legitimately choose a different point
/// along the same inherited overlap. Keeping the deficit per region prevents a
/// mild contact from worsening merely because another contact between the same
/// lineage pair was already more severe.
fn inherited_deficit_limit(
    first: FloatPoint,
    second: FloatPoint,
    regions: &[LineageViolationRegion],
) -> Option<f64> {
    regions
        .iter()
        .filter_map(|(first_shape, second_shape, deficit)| {
            (first_shape.side_of_border(first, REGION_EPS) != Side::OnTheLeft
                && second_shape.side_of_border(second, REGION_EPS) != Side::OnTheLeft)
                .then_some(*deficit)
        })
        .reduce(f64::max)
}

/// Validates all violating contacts involving items born in a replacement
/// transaction. A defect is accepted only when the same lineage pair/layer
/// already violated, its deficit did not increase, and each replacement's
/// current contact witnesses remain inside a pre-existing shape-pair region
/// and on any removed source copper.
pub(crate) fn lineage_delta_is_clear(
    board: &BasicBoard,
    watermark: ItemId,
    baseline: &std::collections::HashMap<LineageViolationKey, f64>,
    source_shapes: &std::collections::HashMap<(ItemId, usize), Vec<TileShape>>,
    baseline_regions: &std::collections::HashMap<LineageViolationKey, Vec<LineageViolationRegion>>,
) -> bool {
    for id in board.item_ids_since(watermark) {
        for violation in lineage_violations_for_item(board, id) {
            let Some(previous) = baseline.get(&violation.key) else {
                return false;
            };
            if violation.deficit > previous + previous.max(1.0) * DISTANCE_EPS {
                return false;
            }
            let (current_first, current_second) =
                if violation.first_lineage <= violation.second_lineage {
                    (violation.first_witness, violation.second_witness)
                } else {
                    (violation.second_witness, violation.first_witness)
                };
            let Some(region_limit) = baseline_regions.get(&violation.key).and_then(|regions| {
                inherited_deficit_limit(current_first, current_second, regions)
            }) else {
                return false;
            };
            if violation.deficit > region_limit + region_limit.max(1.0) * DISTANCE_EPS {
                return false;
            }
            for (item_id, witness) in [
                (violation.first_item, violation.first_witness),
                (violation.second_item, violation.second_witness),
            ] {
                if item_id < watermark {
                    continue;
                }
                let Some(item) = board.get_item(item_id) else {
                    return false;
                };
                if !source_contains_witness(
                    source_shapes,
                    item.base.lineage_no,
                    violation.key.2,
                    witness,
                ) {
                    return false;
                }
            }
        }
    }
    true
}

/// One clearance violation between two items
/// (Java: `ClearanceViolation`/`DrcViolation`).
#[derive(Debug, Clone)]
pub struct DrcViolation {
    pub first_item: ItemId,
    pub second_item: ItemId,
    pub layer: usize,
    pub required_clearance: f64,
    pub actual_distance: f64,
    /// Approximate position of the violation (board units).
    pub x: f64,
    pub y: f64,
}

/// One incomplete net (Java: `NetIncompletes`/unconnected items).
#[derive(Debug, Clone)]
pub struct UnconnectedNet {
    pub net_no: i32,
    pub net_name: String,
}

#[derive(Debug, Default)]
pub struct DrcReport {
    pub violations: Vec<DrcViolation>,
    pub unconnected: Vec<UnconnectedNet>,
}

/// Collects the authoritative clearance violations without also walking the
/// connectivity graph. Optimizer transactions use this narrower scan; the
/// public report adds unconnected nets below.
fn clearance_violations(board: &BasicBoard) -> Vec<DrcViolation> {
    // Every item that carries copper OR constrains it (keepouts, board
    // outline) must be an outer item, so a copper-vs-earlier-obstacle pair is
    // checked from at least one side. Java's `DesignRulesChecker` scans every
    // `board.get_items()` and dedups by sorted-id key afterward; excluding
    // obstacles from the outer loop while keeping the `other_id <= id` skip
    // silently dropped copper-vs-lower-id-obstacle pairs entirely. Netless
    // copper (traces/vias with no net) is a real obstacle in Java
    // (`shares_net` is false against everything), so it must be a candidate
    // too — filtering on `net_count() > 0` dropped netless-vs-copper pairs.
    let candidates: Vec<ItemId> = board
        .items()
        .filter(|(_, it)| {
            matches!(
                &it.kind,
                ItemKind::Via(_) | ItemKind::PolylineTrace(_) | ItemKind::ObstacleArea(_)
            )
        })
        .map(|(id, _)| *id)
        .collect();
    // One violation per (item pair, layer), retaining the minimum actual
    // distance across every convex piece/trace segment of both items.
    let mut violations: std::collections::BTreeMap<(ItemId, ItemId, usize), DrcViolation> =
        std::collections::BTreeMap::new();
    for &id in &candidates {
        let Some(item) = board.get_item(id) else {
            continue;
        };
        let shapes: Vec<_> = item.tile_shapes(&board.padstacks).to_vec();
        for (shape, layer) in shapes {
            // Candidate search radius must cover the largest clearance any
            // counterpart could require on this layer; the fixed 10 000-unit
            // radius missed clearances above 1 mm. Java sizes the search by the
            // item's clearance class; the layer-wide maximum is a safe superset
            // (the matrix is not guaranteed symmetric, so a per-class maximum
            // could under-reach). Same-net clearances live OUTSIDE the matrix,
            // so a `*_same_net` value larger than every matrix entry must widen
            // the radius too, or those pairs are never discovered.
            let search_radius = board
                .rules
                .clearance_matrix
                .max_value(layer)
                .max(board.rules.max_same_net_clearance()) as f64;
            for other_id in board.overlapping_items(&shape.offset(search_radius), Some(layer)) {
                if other_id <= id {
                    continue; // dedup: A-B equals B-A (both sides are outer items)
                }
                let Some(other) = board.get_item(other_id) else {
                    continue;
                };
                // one predicate decides whether (and at what value) the pair
                // constrains: same-net drill rules, keepout/conduction/pin
                // exclusions and Java's (lower-id, higher-id) matrix order
                // live there
                let Some(required) = required_clearance(board, item, other, layer) else {
                    continue;
                };
                let check = shape.offset(required);
                let mut worst: Option<f64> = None;
                for (os, ol) in other.tile_shapes(&board.padstacks).iter() {
                    if *ol != layer || os.intersection(&check).dimension() < 2 {
                        continue;
                    }
                    let d = shape.euclidean_distance_to(os);
                    // Tolerance guards ONLY against floating-point noise in the
                    // Euclidean distance (matrix values and coordinates are
                    // integers). A relative epsilon is unit-independent; the
                    // former fixed `- 1.0` was one board unit — 0.1 µm at
                    // `resolution um 10`, but 25.4 µm at `mil 1`, which let
                    // materially undersized clearances pass.
                    if violates(d, required) {
                        worst = Some(worst.map_or(d, |w: f64| w.min(d)));
                    }
                }
                if let Some(actual) = worst {
                    let bb = shape.bounding_box();
                    let candidate = DrcViolation {
                        first_item: id,
                        second_item: other_id,
                        layer,
                        required_clearance: required,
                        actual_distance: actual,
                        x: (bb.ll.x as f64 + bb.ur.x as f64) / 2.0,
                        y: (bb.ll.y as f64 + bb.ur.y as f64) / 2.0,
                    };
                    let key = (id, other_id, layer);
                    match violations.entry(key) {
                        std::collections::btree_map::Entry::Vacant(entry) => {
                            entry.insert(candidate);
                        }
                        std::collections::btree_map::Entry::Occupied(mut entry)
                            if candidate.actual_distance < entry.get().actual_distance =>
                        {
                            entry.insert(candidate);
                        }
                        std::collections::btree_map::Entry::Occupied(_) => {}
                    }
                }
            }
        }
    }
    violations.into_values().collect()
}

/// Collects all clearance violations and unconnected nets of the board
/// (Java: `DesignRulesChecker.check`). Violations are confirmed with the
/// exact Euclidean copper distance and deduplicated (A-B == B-A).
pub fn check_board(board: &BasicBoard) -> DrcReport {
    let mut report = DrcReport {
        violations: clearance_violations(board),
        ..DrcReport::default()
    };
    for net_no in 1..=board.rules.nets.max_net_no() {
        if !board.net_is_completely_connected(net_no) {
            report.unconnected.push(UnconnectedNet {
                net_no,
                net_name: board
                    .rules
                    .nets
                    .get_by_no(net_no)
                    .map(|n| n.name.clone())
                    .unwrap_or_default(),
            });
        }
    }
    report
}

fn json_escape(s: &str) -> String {
    crate::io::json::escape(s)
}

impl DrcReport {
    /// Serializes the report in the KiCad DRC v1 JSON format, matching
    /// the Java `DrcReport` field for field.
    pub fn to_kicad_json(&self, board: &BasicBoard, source: &str) -> String {
        let scale = 1.0 / board.board_units_per_mm(); // board units → mm (unit-aware)
        let kind = |id: ItemId| -> &'static str {
            match board.get_item(id).map(|i| &i.kind) {
                Some(ItemKind::Via(_)) => "via",
                Some(ItemKind::PolylineTrace(_)) => "track",
                _ => "item",
            }
        };
        // Java `DesignRulesChecker.convertToDrcViolation`: a clearance breach
        // involving a hole (Via or Pin — both are vias here) is reported as
        // `hole_clearance`, not `clearance`. This is a relabel, not a separate
        // geometric pass, so the same violations are detected either way.
        let is_hole =
            |id: ItemId| matches!(board.get_item(id).map(|i| &i.kind), Some(ItemKind::Via(_)));
        let mut out = String::new();
        out.push_str("{\n");
        out.push_str("  \"$schema\": \"https://schemas.kicad.org/drc.v1.json\",\n");
        out.push_str("  \"coordinate_units\": \"mm\",\n");
        out.push_str("  \"kicad_version\": \"N/A\",\n");
        out.push_str(&format!(
            "  \"freerouting_version\": \"{}\",\n",
            env!("CARGO_PKG_VERSION")
        ));
        out.push_str(&format!("  \"source\": \"{}\",\n", json_escape(source)));
        out.push_str("  \"violations\": [\n");
        for (i, v) in self.violations.iter().enumerate() {
            let comma = if i + 1 < self.violations.len() {
                ","
            } else {
                ""
            };
            let (vtype, vlabel) = if is_hole(v.first_item) || is_hole(v.second_item) {
                ("hole_clearance", "Hole clearance")
            } else {
                ("clearance", "Clearance")
            };
            out.push_str(&format!(
                "    {{\"type\": \"{vtype}\", \"severity\": \"error\", \
                 \"description\": \"{vlabel} violation ({:.4} mm < {:.4} mm) on layer {}\", \
                 \"items\": [\
                 {{\"description\": \"{} {}\", \"pos\": {{\"x\": {:.4}, \"y\": {:.4}}}, \"uuid\": \"{}\"}},\
                 {{\"description\": \"{} {}\", \"pos\": {{\"x\": {:.4}, \"y\": {:.4}}}, \"uuid\": \"{}\"}}\
                 ]}}{comma}\n",
                v.actual_distance * scale,
                v.required_clearance * scale,
                v.layer,
                kind(v.first_item),
                v.first_item,
                v.x * scale,
                -v.y * scale,
                v.first_item,
                kind(v.second_item),
                v.second_item,
                v.x * scale,
                -v.y * scale,
                v.second_item,
            ));
        }
        out.push_str("  ],\n");
        out.push_str("  \"unconnected_items\": [\n");
        for (i, u) in self.unconnected.iter().enumerate() {
            let comma = if i + 1 < self.unconnected.len() {
                ","
            } else {
                ""
            };
            out.push_str(&format!(
                "    {{\"type\": \"unconnected_items\", \"severity\": \"warning\", \
                 \"description\": \"Net '{}' is not completely connected\", \"items\": []}}{comma}\n",
                json_escape(&u.net_name)
            ));
        }
        out.push_str("  ],\n");
        out.push_str("  \"schematic_parity\": []\n");
        out.push_str("}\n");
        out
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::board::{Layer, LayerStructure};
    use crate::core::Padstacks;
    use crate::geometry::planar::{IntBox, IntPoint, Polyline, TileShape};
    use crate::rules::{BoardRules, ClearanceMatrix};

    fn test_board() -> BasicBoard {
        let stack = LayerStructure::new(vec![Layer::new("F.Cu", true)]);
        let matrix = ClearanceMatrix::get_default_instance(stack.clone(), 200);
        let mut rules = BoardRules::new(stack.clone(), matrix);
        rules.get_default_net_class();
        let mut padstacks = Padstacks::new(1);
        padstacks.add_shape_on_layers(
            TileShape::Box(IntBox::from_coords(-300, -300, 300, 300)),
            0,
            0,
        );
        BasicBoard::new(stack, rules, padstacks)
    }

    fn trace(points: &[(i32, i32)]) -> Polyline {
        Polyline::from_int_points(
            &points
                .iter()
                .map(|&(x, y)| IntPoint::new(x, y))
                .collect::<Vec<_>>(),
        )
    }

    #[test]
    fn lineage_gate_allows_unchanged_split_of_inherited_violation() {
        let mut board = test_board();
        let source = board.insert_trace(trace(&[(0, 0), (4_000, 0)]), 0, 100, vec![1], 1);
        board.insert_trace(trace(&[(0, 250), (4_000, 250)]), 0, 100, vec![2], 1);
        assert_eq!(check_board(&board).violations.len(), 1);

        let watermark = board.begin_lineage_drc_transaction();
        assert!(board.split_traces_at(IntPoint::new(2_000, 0), 0, 1));
        assert!(
            board.finish_lineage_drc_transaction(watermark),
            "splitting does not create or move the inherited defect"
        );
        assert!(board.get_item(source).is_none());
        assert!(board.item_ids_since(watermark).iter().all(|id| board
            .get_item(*id)
            .unwrap()
            .base
            .lineage_no
            == source));
    }

    #[test]
    fn lineage_gate_survives_normalization_combine_of_replacement_piece() {
        let mut board = test_board();
        let source = board.insert_trace(trace(&[(0, 0), (4_000, 0)]), 0, 100, vec![1], 1);
        board.insert_trace(trace(&[(0, 250), (4_000, 250)]), 0, 100, vec![2], 1);
        assert_eq!(check_board(&board).violations.len(), 1);

        board.generate_snapshot();
        let watermark = board.begin_lineage_drc_transaction();
        board.remove_item(source);
        // Model the normalization path after a shove: two replacement pieces
        // retain the source lineage, then combine_trace joins them.  Before
        // the lineage-preserving combine this produced a fresh lineage and
        // the inherited violation was incorrectly rejected as new.
        let first = board.insert_trace_with_lineage(
            trace(&[(0, 0), (2_000, 0)]),
            0,
            100,
            vec![1],
            1,
            false,
            source,
        );
        board.insert_trace_with_lineage(
            trace(&[(2_000, 0), (4_000, 0)]),
            0,
            100,
            vec![1],
            1,
            false,
            source,
        );
        let combined = board.combine_trace(first);
        assert_eq!(
            board.get_item(combined).unwrap().base.lineage_no,
            source,
            "normalization must retain the source lineage"
        );
        assert!(board.finish_lineage_drc_transaction(watermark));
        assert!(board.pop_snapshot());
        assert_eq!(check_board(&board).violations.len(), 1);
    }

    #[test]
    fn transactional_combine_keeps_distinct_source_lineages_separate() {
        let mut board = test_board();
        let first = board.insert_trace(trace(&[(0, 0), (2_000, 0)]), 0, 100, vec![1], 1);
        let second = board.insert_trace(trace(&[(2_000, 0), (4_000, 0)]), 0, 100, vec![1], 1);
        board.insert_trace(trace(&[(0, 250), (1_900, 250)]), 0, 100, vec![2], 1);
        board.insert_trace(trace(&[(2_100, 250), (4_000, 250)]), 0, 100, vec![3], 1);
        assert!(!lineage_violations_for_item(&board, first).is_empty());
        assert!(!lineage_violations_for_item(&board, second).is_empty());

        let watermark = board.begin_lineage_drc_transaction();
        assert_eq!(board.combine_trace(first), first);
        assert!(board.get_item(first).is_some());
        assert!(board.get_item(second).is_some());
        assert!(
            board.finish_lineage_drc_transaction(watermark),
            "normalization must not collapse two independent provenance regions"
        );
    }

    #[test]
    fn lineage_gate_rejects_a_genuinely_new_violation() {
        let mut board = test_board();
        board.insert_trace(trace(&[(0, 0), (4_000, 0)]), 0, 100, vec![1], 1);

        let watermark = board.begin_lineage_drc_transaction();
        board.insert_trace(trace(&[(0, 250), (4_000, 250)]), 0, 100, vec![2], 1);
        assert!(!board.finish_lineage_drc_transaction(watermark));
    }

    #[test]
    fn lineage_gate_rejects_relocated_violation_with_same_pair_and_deficit() {
        let mut board = test_board();
        let source = board.insert_trace(trace(&[(0, 0), (4_000, 0)]), 0, 100, vec![1], 1);
        board.insert_trace(trace(&[(0, 250), (4_000, 250)]), 0, 100, vec![2], 1);
        assert_eq!(check_board(&board).violations.len(), 1);

        let watermark = board.begin_lineage_drc_transaction();
        board.remove_item(source);
        board.insert_trace_with_lineage(
            trace(&[(0, 500), (4_000, 500)]),
            0,
            100,
            vec![1],
            1,
            false,
            source,
        );
        assert!(
            !board.finish_lineage_drc_transaction(watermark),
            "an equal-severity defect at a different contact is not inherited"
        );
    }

    #[test]
    fn lineage_gate_binds_deficit_to_the_matching_contact_region() {
        let mut board = test_board();
        assert!(board.rules.clearance_matrix.append_class("strict"));
        let strict = board.rules.clearance_matrix.get_no("strict").unwrap();
        board
            .rules
            .clearance_matrix
            .set_value_on_all_layers(1, strict, 300);

        // Two disjoint contacts share the same lineage-pair key. The first is
        // maximally severe (overlapping copper, deficit 200); the second has a
        // 100-unit deficit. Replacing only the second region under a stricter
        // class raises its deficit to 200 without moving either witness. A
        // key-wide max would accept that regression; its own region must not.
        let first_source = board.insert_trace(trace(&[(0, 0), (1_000, 0)]), 0, 100, vec![1], 1);
        let first_other = board.insert_trace(trace(&[(0, 0), (1_000, 0)]), 0, 100, vec![2], 1);
        let second_source =
            board.insert_trace(trace(&[(5_000, 2_000), (6_000, 2_000)]), 0, 100, vec![1], 1);
        let second_other =
            board.insert_trace(trace(&[(5_000, 2_300), (6_000, 2_300)]), 0, 100, vec![2], 1);
        board.set_item_lineage(second_source, first_source);
        board.set_item_lineage(second_other, first_other);
        assert_eq!(check_board(&board).violations.len(), 2);

        let watermark = board.begin_lineage_drc_transaction();
        board.remove_item(first_source);
        board.remove_item(second_source);
        board.insert_trace_with_lineage(
            trace(&[(0, 0), (1_000, 0)]),
            0,
            100,
            vec![1],
            1,
            false,
            first_source,
        );
        board.insert_trace_with_lineage(
            trace(&[(5_000, 2_000), (6_000, 2_000)]),
            0,
            100,
            vec![1],
            strict,
            false,
            first_source,
        );
        assert!(
            !board.finish_lineage_drc_transaction(watermark),
            "a severe contact elsewhere must not authorize this region to worsen"
        );
    }

    #[test]
    fn nested_rollback_keeps_the_outer_lineage_transaction() {
        let mut board = test_board();
        let source = board.insert_trace(trace(&[(0, 0), (4_000, 0)]), 0, 100, vec![1], 1);
        board.insert_trace(trace(&[(0, 250), (4_000, 250)]), 0, 100, vec![2], 1);
        assert_eq!(check_board(&board).violations.len(), 1);

        // The outer replacement removes the first source trace and must keep
        // its inherited violation baseline while nested shove/via attempts
        // speculate and roll back.
        board.generate_snapshot();
        let outer = board.begin_lineage_drc_transaction();
        board.remove_item(source);

        board.generate_snapshot();
        let inner = board.begin_lineage_drc_transaction();
        assert!(
            !board.finish_lineage_drc_transaction(inner + 1),
            "a stale watermark must not consume the nested context"
        );
        board.insert_trace(trace(&[(0, 250), (4_000, 250)]), 0, 100, vec![3], 1);
        assert!(!board.finish_lineage_drc_transaction(inner));
        assert!(board.rollback_snapshot());

        // A successful nested helper commits its snapshot into the outer
        // level; that must preserve the outer baseline just like rollback.
        board.generate_snapshot();
        let committed_inner = board.begin_lineage_drc_transaction();
        assert!(board.finish_lineage_drc_transaction(committed_inner));
        assert!(board.pop_snapshot());

        // If the inner rollback erased the outer context, this valid
        // one-for-one replacement would be rejected as an unexplained new
        // violation. Stable lineage plus the original witness should pass.
        board.insert_trace_with_lineage(
            trace(&[(0, 0), (4_000, 0)]),
            0,
            100,
            vec![1],
            1,
            false,
            source,
        );
        assert!(board.finish_lineage_drc_transaction(outer));
        assert!(board.pop_snapshot());
        assert_eq!(check_board(&board).violations.len(), 1);
    }

    #[test]
    fn inner_rollback_discards_outer_capture_contributions() {
        let mut board = test_board();
        let source = board.insert_trace(trace(&[(0, 0), (4_000, 0)]), 0, 100, vec![1], 1);
        board.insert_trace(trace(&[(0, 250), (4_000, 250)]), 0, 100, vec![2], 1);

        board.generate_snapshot();
        let outer = board.begin_lineage_drc_transaction();
        board.generate_snapshot();
        let inner = board.begin_lineage_drc_transaction();
        // Removing the source records its inherited defect in both active
        // transactions. The inner attempt then fails for a non-DRC reason.
        board.remove_item(source);
        board.discard_lineage_drc_transaction(inner);
        assert!(board.rollback_snapshot());
        assert!(board.get_item(source).is_some());

        // A new duplicate with the same lineage/contact must still be rejected.
        // If the rolled-back inner baseline leaked into the outer transaction,
        // it would incorrectly authorize this new violation as inherited.
        board.insert_trace_with_lineage(
            trace(&[(0, 0), (4_000, 0)]),
            0,
            100,
            vec![1],
            1,
            false,
            source,
        );
        assert!(!board.finish_lineage_drc_transaction(outer));
        assert!(board.rollback_snapshot());
    }

    #[test]
    fn inner_commit_keeps_outer_capture_contributions() {
        let mut board = test_board();
        let source = board.insert_trace(trace(&[(0, 0), (4_000, 0)]), 0, 100, vec![1], 1);
        board.insert_trace(trace(&[(0, 250), (4_000, 250)]), 0, 100, vec![2], 1);

        board.generate_snapshot();
        let outer = board.begin_lineage_drc_transaction();
        board.generate_snapshot();
        let inner = board.begin_lineage_drc_transaction();
        board.remove_item(source);
        board.insert_trace_with_lineage(
            trace(&[(0, 0), (4_000, 0)]),
            0,
            100,
            vec![1],
            1,
            false,
            source,
        );
        assert!(board.finish_lineage_drc_transaction(inner));
        assert!(board.pop_snapshot());
        assert!(
            board.finish_lineage_drc_transaction(outer),
            "committed inner observations must remain in the outer baseline"
        );
        assert!(board.pop_snapshot());
    }

    #[test]
    fn same_net_drill_clearance_overrides_default() {
        let mut board = test_board();
        // two SAME-net vias, edge-to-edge gap 100 (padstack spans -300..300)
        board.insert_via(1, IntPoint::new(0, 0), vec![1], 1, false);
        board.insert_via(1, IntPoint::new(0, 700), vec![1], 1, false);
        // with no same-net rule, same-net drills are checked at the default
        // clearance (200) -> the 100 gap is a violation
        assert_eq!(
            check_board(&board).violations.len(),
            1,
            "same-net drills default to the ordinary clearance"
        );
        // a via_via_same_net of 50 relaxes the requirement below the gap
        board.rules.set_same_net_clearance(
            crate::rules::ItemClass::Via,
            crate::rules::ItemClass::Via,
            50,
        );
        assert!(
            check_board(&board).violations.is_empty(),
            "via_via_same_net (50) < 100 gap -> clean"
        );
    }

    #[test]
    fn same_net_clearance_beyond_matrix_max_is_found() {
        let mut board = test_board();
        // two SAME-net vias with an edge gap (700) beyond the matrix maximum
        // (200): clean without a same-net rule...
        board.insert_via(1, IntPoint::new(0, 0), vec![1], 1, false);
        board.insert_via(1, IntPoint::new(0, 1300), vec![1], 1, false);
        assert!(check_board(&board).violations.is_empty());
        // ...but a via_via_same_net of 1000 must flag it — which requires the
        // candidate-search radius to include same-net values, since they live
        // outside the clearance matrix and can exceed its maximum.
        board.rules.set_same_net_clearance(
            crate::rules::ItemClass::Via,
            crate::rules::ItemClass::Via,
            1000,
        );
        assert_eq!(
            check_board(&board).violations.len(),
            1,
            "same-net rule larger than every matrix value must still be enforced"
        );
    }

    #[test]
    fn same_net_drill_touch_is_a_violation_unless_attach_exempt() {
        // Java Via/Pin.is_obstacle: overlapping same-net drills are a
        // violation; the ONLY exemption is an attach-allowed via on a
        // drillable (SMD) pin.
        let mut board = test_board();
        // two same-net vias stacked at the same spot: violation
        board.insert_via(1, IntPoint::new(0, 0), vec![1], 1, false);
        board.insert_via(1, IntPoint::new(0, 100), vec![1], 1, false);
        assert_eq!(
            check_board(&board).violations.len(),
            1,
            "overlapping same-net vias must be a violation"
        );

        // an attach-allowed via on a same-net SMD pin: exempt
        let mut board = test_board();
        let pin = board.insert_via(1, IntPoint::new(0, 0), vec![1], 1, false);
        board.set_component_no(pin, 1); // single-layer padstack -> SMD pin
        board.insert_via(1, IntPoint::new(0, 100), vec![1], 1, true);
        assert!(
            check_board(&board).violations.is_empty(),
            "attach-allowed via on same-net SMD pin is the fanout exemption"
        );

        // the same via WITHOUT attach: violation
        let mut board = test_board();
        let pin = board.insert_via(1, IntPoint::new(0, 0), vec![1], 1, false);
        board.set_component_no(pin, 1);
        board.insert_via(1, IntPoint::new(0, 100), vec![1], 1, false);
        assert_eq!(
            check_board(&board).violations.len(),
            1,
            "a non-attach via on a same-net pin stays a violation"
        );
    }

    #[test]
    fn escape_via_exception_is_limited_to_its_smd_layer() {
        let stack = LayerStructure::new(vec![Layer::new("F.Cu", true), Layer::new("B.Cu", true)]);
        let matrix = ClearanceMatrix::get_default_instance(stack.clone(), 200);
        let mut rules = BoardRules::new(stack.clone(), matrix);
        rules.get_default_net_class();
        let mut padstacks = Padstacks::new(2);
        let shape = TileShape::Box(IntBox::from_coords(-300, -300, 300, 300));
        let through = padstacks.add(
            "through",
            vec![Some(shape.clone()), Some(shape.clone())],
            false,
            false,
        );
        let front = padstacks.add("front", vec![Some(shape.clone()), None], false, false);
        let back = padstacks.add("back", vec![None, Some(shape)], false, false);
        let mut board = BasicBoard::new(stack, rules, padstacks);

        let front_pin = board.insert_via(front, IntPoint::new(0, 0), vec![1], 1, false);
        board.set_component_no(front_pin, 1);
        let escape = board.insert_escape_via(through, IntPoint::new(0, 0), vec![1], 1, false, 0);
        let ItemKind::Via(escape_item) = &board.get_item(escape).unwrap().kind else {
            unreachable!()
        };
        assert!(escape_item.is_escape_via);
        assert!(
            !escape_item.attach_allowed,
            "ViaInfo declaration is preserved"
        );
        assert!(
            check_board(&board).violations.is_empty(),
            "the marked F.Cu SMD contact is exempt"
        );

        let back_pin = board.insert_via(back, IntPoint::new(0, 0), vec![1], 1, false);
        board.set_component_no(back_pin, 2);
        let report = check_board(&board);
        assert_eq!(report.violations.len(), 1);
        assert_eq!(report.violations[0].layer, 1, "B.Cu remains constrained");
    }

    #[test]
    fn detects_a_violation_and_serializes() {
        let mut board = test_board();
        board.insert_trace(
            Polyline::from_int_points(&[IntPoint::new(0, 0), IntPoint::new(5000, 0)]),
            0,
            100,
            vec![1],
            1,
        );
        // a foreign trace 150 away (need 200): violation
        board.insert_trace(
            Polyline::from_int_points(&[IntPoint::new(0, 350), IntPoint::new(5000, 350)]),
            0,
            100,
            vec![2],
            1,
        );
        let report = check_board(&board);
        assert_eq!(report.violations.len(), 1);
        let v = &report.violations[0];
        assert!(v.actual_distance < 200.0 && v.actual_distance > 100.0);
        let json = report.to_kicad_json(&board, "test.dsn");
        assert!(json.contains("schemas.kicad.org/drc.v1.json"));
        assert!(json.contains("clearance"));
    }

    #[test]
    fn violation_snapshot_distinguishes_same_pair_with_worse_clearance() {
        let board_with_offset = |offset: i32| {
            let mut board = test_board();
            board.insert_trace(
                Polyline::from_int_points(&[IntPoint::new(0, 0), IntPoint::new(5000, 0)]),
                0,
                100,
                vec![1],
                1,
            );
            board.insert_trace(
                Polyline::from_int_points(&[IntPoint::new(0, offset), IntPoint::new(5000, offset)]),
                0,
                100,
                vec![2],
                1,
            );
            board
        };
        let mild = violation_snapshot(&board_with_offset(350));
        let severe = violation_snapshot(&board_with_offset(250));
        assert_eq!(mild.len(), 1);
        assert_eq!(severe.len(), 1);
        let key = *mild.keys().next().expect("one violation");
        assert!(severe.contains_key(&key));
        assert!(severe[&key] > mild[&key]);

        // One item pair can have several trace segments on the same layer.
        // The report must retain the worst segment distance, not whichever
        // segment happened to be visited first.
        let mut multi = test_board();
        multi.insert_trace(
            Polyline::from_int_points(&[
                IntPoint::new(0, 0),
                IntPoint::new(5000, 0),
                IntPoint::new(5000, 600),
                IntPoint::new(0, 600),
            ]),
            0,
            100,
            vec![1],
            1,
        );
        multi.insert_trace(
            Polyline::from_int_points(&[IntPoint::new(0, 350), IntPoint::new(4000, 350)]),
            0,
            100,
            vec![2],
            1,
        );
        let report = check_board(&multi);
        assert_eq!(report.violations.len(), 1);
        assert!(
            report.violations[0].actual_distance < 100.0,
            "the closest segment was not retained: {:?}",
            report.violations[0]
        );
    }

    #[test]
    fn copper_against_earlier_lower_id_keepout_is_checked() {
        use crate::geometry::planar::{PolygonShape, PolylineArea};
        let mut board = test_board();
        // Insert the keepout FIRST so it gets the lower item id — this is the
        // regression: previously it was excluded from the outer loop and the
        // `other_id <= id` skip dropped the pair entirely.
        board.insert_area(
            PolylineArea::new(
                PolygonShape::from_int_points(&[
                    IntPoint::new(0, 250),
                    IntPoint::new(5000, 250),
                    IntPoint::new(5000, 450),
                    IntPoint::new(0, 450),
                ]),
                Vec::new(),
            ),
            0,
            "keepout",
            Vec::new(),
            1,
            false,
        );
        // Foreign-net copper 150 away (needs 200) → must be a violation.
        board.insert_trace(
            Polyline::from_int_points(&[IntPoint::new(0, 0), IntPoint::new(5000, 0)]),
            0,
            100,
            vec![1],
            1,
        );
        let report = check_board(&board);
        assert_eq!(
            report.violations.len(),
            1,
            "copper vs earlier lower-id keepout must be reported"
        );
    }

    #[test]
    fn clean_board_reports_nothing() {
        let mut board = test_board();
        board.insert_trace(
            Polyline::from_int_points(&[IntPoint::new(0, 0), IntPoint::new(5000, 0)]),
            0,
            100,
            vec![1],
            1,
        );
        board.insert_trace(
            Polyline::from_int_points(&[IntPoint::new(0, 500), IntPoint::new(5000, 500)]),
            0,
            100,
            vec![2],
            1,
        );
        let report = check_board(&board);
        assert!(report.violations.is_empty());
    }

    #[test]
    fn new_item_clearance_uses_final_id_order_for_asymmetric_matrix() {
        let mut board = test_board();
        assert!(board.rules.clearance_matrix.append_class("strict"));
        let strict = board.rules.clearance_matrix.get_no("strict").unwrap();
        // Java visits the new (higher-id) strict item first, then queries
        // M(old default, new strict). The transposed cell is deliberately zero.
        board.rules.clearance_matrix.set_value(1, strict, 0, 1000);
        board.rules.clearance_matrix.set_value(strict, 1, 0, 0);
        board.insert_trace(
            Polyline::from_int_points(&[IntPoint::new(0, 0), IntPoint::new(5000, 0)]),
            0,
            100,
            vec![1],
            1,
        );
        let new_id = board.insert_trace(
            Polyline::from_int_points(&[IntPoint::new(0, 700), IntPoint::new(5000, 700)]),
            0,
            100,
            vec![2],
            strict,
        );
        assert!(!crate::drc::item_is_clear(&board, new_id));
        assert_eq!(check_board(&board).violations.len(), 1);
    }
}
