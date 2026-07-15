//! Port of the `drc/` package: `DesignRulesChecker` collecting clearance
//! violations and unconnected items, and `DrcReport` serializing them in
//! the KiCad DRC v1 JSON format (`https://schemas.kicad.org/drc.v1.json`)
//! exactly like the Java report.

use crate::board::basic_board::{BasicBoard, ItemId};
use crate::board::ItemKind;

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

/// Collects all clearance violations and unconnected nets of the board
/// (Java: `DesignRulesChecker.check`). Violations are confirmed with the
/// exact Euclidean copper distance and deduplicated (A-B == B-A).
pub fn check_board(board: &BasicBoard) -> DrcReport {
    let mut report = DrcReport::default();
    let routed: Vec<ItemId> = board
        .items()
        .filter(|(_, it)| it.base.net_count() > 0 && !matches!(&it.kind, ItemKind::ObstacleArea(_)))
        .map(|(id, _)| *id)
        .collect();
    for &id in &routed {
        let Some(item) = board.get_item(id) else { continue };
        let shapes: Vec<_> = item.tile_shapes(&board.padstacks).to_vec();
        for (shape, layer) in shapes {
            for other_id in board.overlapping_items(&shape.offset(10_000.0), Some(layer)) {
                if other_id <= id {
                    continue; // dedup: A-B equals B-A
                }
                let Some(other) = board.get_item(other_id) else { continue };
                if other.base.shares_net(&item.base) {
                    continue;
                }
                if let ItemKind::ObstacleArea(a) = &other.kind {
                    if a.is_conduction {
                        continue;
                    }
                }
                let required = board.rules.clearance_matrix.get_value(
                    item.base.clearance_class,
                    other.base.clearance_class,
                    layer,
                    false,
                ) as f64;
                let check = shape.offset(required);
                let mut worst: Option<f64> = None;
                for (os, ol) in other.tile_shapes(&board.padstacks).iter() {
                    if *ol != layer || os.intersection(&check).dimension() < 2 {
                        continue;
                    }
                    let d = shape.euclidean_distance_to(os);
                    if d < required - 1.0 {
                        worst = Some(worst.map_or(d, |w: f64| w.min(d)));
                    }
                }
                if let Some(actual) = worst {
                    let bb = shape.bounding_box();
                    report.violations.push(DrcViolation {
                        first_item: id,
                        second_item: other_id,
                        layer,
                        required_clearance: required,
                        actual_distance: actual,
                        x: (bb.ll.x as f64 + bb.ur.x as f64) / 2.0,
                        y: (bb.ll.y as f64 + bb.ur.y as f64) / 2.0,
                    });
                }
            }
        }
    }
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
    s.replace('\\', "\\\\").replace('"', "\\\"")
}

impl DrcReport {
    /// Serializes the report in the KiCad DRC v1 JSON format, matching
    /// the Java `DrcReport` field for field.
    pub fn to_kicad_json(&self, board: &BasicBoard, source: &str) -> String {
        let scale = 1.0 / (board.resolution.max(1) as f64 * 1000.0); // board units → mm
        let kind = |id: ItemId| -> &'static str {
            match board.get_item(id).map(|i| &i.kind) {
                Some(ItemKind::Via(_)) => "via",
                Some(ItemKind::PolylineTrace(_)) => "track",
                _ => "item",
            }
        };
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
            let comma = if i + 1 < self.violations.len() { "," } else { "" };
            out.push_str(&format!(
                "    {{\"type\": \"clearance\", \"severity\": \"error\", \
                 \"description\": \"Clearance violation ({:.4} mm < {:.4} mm) on layer {}\", \
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
            let comma = if i + 1 < self.unconnected.len() { "," } else { "" };
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
}
