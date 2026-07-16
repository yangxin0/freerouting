//! The KiCad board JSON writer (Java: `KiCadJsonWriter`): serializes the
//! board in the same schema the reader consumes — layers, net classes,
//! nets, components with pads, and the routed traces and vias.

use crate::board::basic_board::BasicBoard;
use crate::board::ItemKind;

fn esc(s: &str) -> String {
    crate::io::json::escape(s)
}

/// Serializes the board as KiCad board JSON. Coordinates are written in
/// mm (the reader's default unit).
pub fn export_kicad_json(board: &BasicBoard) -> String {
    // Board units → true millimetres. Dividing by the raw resolution was only
    // correct for mm-based boards; a `um`-based DSN (the common case) emitted
    // micrometre-magnitude numbers mislabeled "MM" (1000x off). Emit the
    // matching units-per-mm as the resolution so the reader reconstructs the
    // exact board-unit scale.
    let units_per_mm = board.board_units_per_mm();
    let mm = |v: f64| v / units_per_mm;
    let mut out = String::from("{\n  \"unit\": \"MM\",\n");
    out.push_str(&format!(
        "  \"resolution\": {},\n",
        units_per_mm.round().max(1.0) as i64
    ));
    // layers
    out.push_str("  \"layers\": [\n");
    let layer_count = board.layer_structure.layer_count();
    for (i, layer) in board.layer_structure.arr.iter().enumerate() {
        let comma = if i + 1 < layer_count { "," } else { "" };
        out.push_str(&format!(
            "    {{\"index\": {i}, \"name\": \"{}\", \"type\": \"{}\"}}{comma}\n",
            esc(&layer.name),
            if layer.is_signal { "signal" } else { "plane" }
        ));
    }
    out.push_str("  ],\n");
    // net classes: emit EVERY real class with its own clearance and trace
    // width, plus the cross-class clearance rules, so custom DSN classes
    // survive the round trip instead of collapsing into a single "Default".
    let matrix = &board.rules.clearance_matrix;
    let class_count = board.rules.net_classes.count();
    let net_count = board.rules.nets.max_net_no();
    // net names grouped by their class index
    let mut class_nets: Vec<Vec<String>> = vec![Vec::new(); class_count.max(1)];
    for n in 1..=net_count {
        if let Some(net) = board.rules.nets.get_by_no(n) {
            if let Some(v) = class_nets.get_mut(net.get_class()) {
                v.push(net.name.clone());
            }
        }
    }
    let class_json: Vec<String> = (0..class_count)
        .map(|i| {
            let class = board.rules.net_classes.get(i);
            let tcc = class.get_trace_clearance_class();
            let cl = matrix.get_value(tcc, tcc, 0, false) as f64;
            let hw = class.get_trace_half_width(0) as f64;
            let names: Vec<String> = class_nets[i]
                .iter()
                .map(|nm| format!("\"{}\"", esc(nm)))
                .collect();
            // via dimensions from the class's actual via rule (Java
            // KiCadJsonWriter): the first via's padstack shape width is the
            // diameter, the drill is half of it (the model carries no drill);
            // 0.8/0.4 mm is Java's fallback for classes without a via rule
            let via_diameter = class
                .get_via_rule()
                .and_then(|rule_id| board.rules.via_rules.get(rule_id))
                .filter(|rule| rule.via_count() > 0)
                .map(|rule| board.rules.via_infos.get(rule.get_via(0)).get_padstack())
                .and_then(|ps_no| board.padstacks.get_by_no(ps_no))
                .and_then(|ps| ps.get_shape(ps.from_layer()))
                .map(|s| mm(s.bounding_box().width() as f64))
                .unwrap_or(0.8);
            format!(
                "{{\"name\": \"{}\", \"clearance\": {:.6}, \"traceWidth\": {:.6}, \"viaDiameter\": {:.6}, \"viaDrill\": {:.6}, \"netNames\": [{}]}}",
                esc(class.get_name()),
                mm(cl),
                mm(2.0 * hw),
                via_diameter,
                via_diameter * 0.5,
                names.join(", ")
            )
        })
        .collect();
    out.push_str(&format!("  \"netClasses\": [{}],\n", class_json.join(", ")));
    // cross-class clearance rules: the (max) spacing between two classes
    let mut rule_json: Vec<String> = Vec::new();
    for i in 0..class_count {
        for j in (i + 1)..class_count {
            let ci = board.rules.net_classes.get(i).get_trace_clearance_class();
            let cj = board.rules.net_classes.get(j).get_trace_clearance_class();
            let v = matrix.get_value(ci, cj, 0, false) as f64;
            rule_json.push(format!(
                "{{\"classA\": \"{}\", \"classB\": \"{}\", \"clearance\": {:.6}}}",
                esc(board.rules.net_classes.get(i).get_name()),
                esc(board.rules.net_classes.get(j).get_name()),
                mm(v)
            ));
        }
    }
    out.push_str(&format!(
        "  \"clearanceRules\": [{}],\n",
        rule_json.join(", ")
    ));
    // nets, each tagged with its real class name and whether it carries a
    // copper pour (containsPlane, consumed by the reader). The net's own
    // contains_plane flag is authoritative (Java KiCadJsonWriter reads
    // `net.contains_plane()`); serialized conduction areas are a fallback so
    // a hand-built board without the flag still round-trips.
    let mut plane_nets: std::collections::HashSet<i32> = board
        .items()
        .filter_map(|(_, it)| match &it.kind {
            ItemKind::ObstacleArea(a) if a.is_conduction => it.base.net_nos.first().copied(),
            _ => None,
        })
        .collect();
    for n in 1..=net_count {
        if board
            .rules
            .nets
            .get_by_no(n)
            .is_some_and(|net| net.contains_plane())
        {
            plane_nets.insert(n);
        }
    }
    out.push_str("  \"nets\": [\n");
    for n in 1..=net_count {
        let (name, class_name) = board
            .rules
            .nets
            .get_by_no(n)
            .map(|x| {
                (
                    x.name.clone(),
                    board
                        .rules
                        .net_classes
                        .get(x.get_class())
                        .get_name()
                        .to_string(),
                )
            })
            .unwrap_or_default();
        let comma = if n < net_count { "," } else { "" };
        out.push_str(&format!(
            "    {{\"id\": {n}, \"name\": \"{}\", \"className\": \"{}\", \"containsPlane\": {}}}{comma}\n",
            esc(&name),
            esc(&class_name),
            plane_nets.contains(&n),
        ));
    }
    out.push_str("  ],\n");
    // components: group pins by component_no
    let mut comp_pads: std::collections::BTreeMap<i32, Vec<String>> =
        std::collections::BTreeMap::new();
    for (_, item) in board.items() {
        if item.base.component_no == 0 {
            continue;
        }
        let ItemKind::Via(v) = &item.kind else {
            continue;
        };
        let net_name = item
            .base
            .net_nos
            .first()
            .and_then(|&n| board.rules.nets.get_by_no(n))
            .map(|x| x.name.clone())
            .unwrap_or_default();
        let bb = item.bounding_box(&board.padstacks);
        let (w, h) = ((bb.ur.x - bb.ll.x) as f64, (bb.ur.y - bb.ll.y) as f64);
        let through = board
            .padstacks
            .get_by_no(v.padstack)
            .is_some_and(|p| p.from_layer() == 0 && p.to_layer() + 1 == layer_count);
        // the pad must name EVERY layer of its span: the reader derives the
        // span from the named layers, so a through pad listing one layer
        // came back single-layer (electrically different)
        let layers_json = board
            .padstacks
            .get_by_no(v.padstack)
            .map(|p| {
                (p.from_layer()..=p.to_layer())
                    .filter_map(|l| board.layer_structure.arr.get(l))
                    .map(|l| format!("\"{}\"", esc(&l.name)))
                    .collect::<Vec<_>>()
                    .join(", ")
            })
            .unwrap_or_else(|| "\"F.Cu\"".into());
        comp_pads.entry(item.base.component_no).or_default().push(format!(
            "{{\"name\": \"\", \"netName\": \"{}\", \"shape\": \"rect\", \"size\": {{\"x\": {:.6}, \"y\": {:.6}}}, \"offset\": {{\"x\": {:.6}, \"y\": {:.6}}}, \"drill\": {}, \"layers\": [{}]}}",
            esc(&net_name),
            mm(w),
            mm(h),
            mm(v.center.x as f64),
            mm(-v.center.y as f64),
            if through { 1 } else { 0 },
            layers_json,
        ));
    }
    out.push_str("  \"components\": [\n");
    let n_comps = comp_pads.len();
    for (i, (comp, pads)) in comp_pads.iter().enumerate() {
        let comma = if i + 1 < n_comps { "," } else { "" };
        out.push_str(&format!(
            "    {{\"reference\": \"C{comp}\", \"value\": \"\", \"footprint\": \"\", \"position\": {{\"x\": 0, \"y\": 0}}, \"rotation\": 0, \"layer\": \"F.Cu\", \"pads\": [{}]}}{comma}\n",
            pads.join(", ")
        ));
    }
    out.push_str("  ],\n");
    // outline: the board's bounding box (the exact outline polygon is not
    // reconstructible from the boundary keepout strips)
    let bb = board.bounding_box();
    let outline_corners = if bb.is_empty() {
        String::new()
    } else {
        [
            (bb.ll.x, bb.ll.y),
            (bb.ur.x, bb.ll.y),
            (bb.ur.x, bb.ur.y),
            (bb.ll.x, bb.ur.y),
        ]
        .iter()
        .map(|(x, y)| {
            format!(
                "{{\"x\": {:.6}, \"y\": {:.6}}}",
                mm(*x as f64),
                mm(-*y as f64)
            )
        })
        .collect::<Vec<_>>()
        .join(", ")
    };
    out.push_str(&format!(
        "  \"outline\": {{\"corners\": [{outline_corners}], \"clearance\": 0.2}},\n"
    ));
    // routed traces and vias
    let mut traces = Vec::new();
    let mut vias = Vec::new();
    for (id, item) in board.items() {
        if item.base.component_no != 0 || item.base.net_count() == 0 {
            continue;
        }
        let net_name = item
            .base
            .net_nos
            .first()
            .and_then(|&n| board.rules.nets.get_by_no(n))
            .map(|x| x.name.clone())
            .unwrap_or_default();
        match &item.kind {
            ItemKind::PolylineTrace(t) => {
                let pts: Vec<String> = t
                    .polyline
                    .corner_approx_arr()
                    .iter()
                    .map(|c| format!("{{\"x\": {:.6}, \"y\": {:.6}}}", mm(c.x), mm(-c.y)))
                    .collect();
                traces.push(format!(
                    "    {{\"id\": {id}, \"netName\": \"{}\", \"width\": {:.6}, \"layerIndex\": {}, \"points\": [{}]}}",
                    esc(&net_name),
                    mm(2.0 * t.half_width as f64),
                    t.layer,
                    pts.join(", ")
                ));
            }
            ItemKind::Via(v) => {
                // the via's real layer span and pad diameter, not a
                // hardcoded full-stack 0.6 mm via (blind/buried vias and
                // real sizes must survive the round trip)
                let (from, to, dia) = board
                    .padstacks
                    .get_by_no(v.padstack)
                    .map(|p| {
                        (
                            p.from_layer(),
                            p.to_layer(),
                            p.get_shape(p.from_layer())
                                .map(|s| s.bounding_box().max_width())
                                .unwrap_or(0.0),
                        )
                    })
                    .unwrap_or((0, layer_count - 1, 0.0));
                vias.push(format!(
                    "    {{\"id\": {id}, \"netName\": \"{}\", \"position\": {{\"x\": {:.6}, \"y\": {:.6}}}, \"diameter\": {:.6}, \"drill\": {:.6}, \"startLayerIndex\": {from}, \"endLayerIndex\": {to}}}",
                    esc(&net_name),
                    mm(v.center.x as f64),
                    mm(-v.center.y as f64),
                    mm(dia),
                    mm(dia) / 2.0,
                ));
            }
            _ => {}
        }
    }
    out.push_str(&format!("  \"traces\": [\n{}\n  ],\n", traces.join(",\n")));
    out.push_str(&format!("  \"vias\": [\n{}\n  ],\n", vias.join(",\n")));
    // conduction areas (copper pours), with their obstacle flag
    let mut zones = Vec::new();
    for (_, item) in board.items() {
        let ItemKind::ObstacleArea(a) = &item.kind else {
            continue;
        };
        if !a.is_conduction {
            continue;
        }
        let net_name = item
            .base
            .net_nos
            .first()
            .and_then(|&n| board.rules.nets.get_by_no(n))
            .map(|x| x.name.clone())
            .unwrap_or_default();
        let corners: Vec<String> = a
            .area
            .corner_approx_arr()
            .iter()
            .map(|c| format!("{{\"x\": {:.6}, \"y\": {:.6}}}", mm(c.x), mm(-c.y)))
            .collect();
        if corners.len() < 3 {
            continue;
        }
        zones.push(format!(
            "    {{\"netName\": \"{}\", \"layerIndex\": {}, \"isObstacle\": {}, \"polygon\": [{}]}}",
            esc(&net_name),
            a.layer,
            a.is_obstacle,
            corners.join(", ")
        ));
    }
    out.push_str(&format!(
        "  \"conductionAreas\": [\n{}\n  ]\n}}\n",
        zones.join(",\n")
    ));
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::io::import_dsn;
    use crate::io::kicad_json::import_kicad_json;

    const MINI_DSN: &str = r#"(pcb "mini.dsn"
  (resolution um 10)
  (structure
    (layer F.Cu (type signal))
    (layer B.Cu (type signal))
    (boundary (rect pcb 0 0 100000 100000))
    (rule (width 200) (clearance 200))
  )
  (placement)
  (library
    (padstack "Via[0-1]_600:300_um"
      (shape (circle F.Cu 600 0 0))
      (shape (circle B.Cu 600 0 0))
      (attach off)
    )
  )
  (network (net "N1"))
)"#;

    #[test]
    fn writer_round_trips_through_the_reader() {
        let mut board = import_dsn(MINI_DSN).expect("import");
        board.insert_trace(
            crate::geometry::planar::Polyline::from_int_points(&[
                crate::geometry::planar::IntPoint::new(10000, 10000),
                crate::geometry::planar::IntPoint::new(50000, 10000),
            ]),
            0,
            100,
            vec![1],
            1,
        );
        let json = export_kicad_json(&board);
        let board2 = import_kicad_json(&json).expect("re-import");
        assert_eq!(board2.layer_structure.layer_count(), 2);
        assert_eq!(board2.rules.nets.max_net_no(), 1);
        let traces = board2
            .items()
            .filter(|(_, it)| matches!(it.kind, ItemKind::PolylineTrace(_)))
            .count();
        assert_eq!(traces, 1, "the trace must survive the round trip");
    }

    #[test]
    fn through_pads_vias_and_pours_survive_round_trip() {
        use crate::geometry::planar::{IntPoint, PolygonShape, PolylineArea};
        let mut board = import_dsn(MINI_DSN).expect("import");
        // a through pad (the Via padstack spans both layers)
        let pad = board.insert_via(1, IntPoint::new(20000, 20000), vec![1], 1, false);
        board.set_component_no(pad, 1);
        // a routed via
        board.insert_via(1, IntPoint::new(40000, 40000), vec![1], 1, false);
        // an obstacle-flagged pour
        let area = PolylineArea::new(
            PolygonShape::from_int_points(&[
                IntPoint::new(0, 0),
                IntPoint::new(30000, 0),
                IntPoint::new(30000, 30000),
                IntPoint::new(0, 30000),
            ]),
            Vec::new(),
        );
        let zone = board.insert_area(area, 1, "N1", vec![1], 1, true);
        board.set_area_is_obstacle(zone, true);

        let json = export_kicad_json(&board);
        assert!(json.contains("\"containsPlane\": true"));
        let board2 = import_kicad_json(&json).expect("re-import");
        // the through pad still spans both layers
        let pad2 = board2
            .items()
            .find(|(_, it)| it.base.component_no != 0)
            .map(|(_, it)| it.clone())
            .expect("pad");
        assert_eq!(pad2.first_layer(&board2.padstacks), 0);
        assert_eq!(pad2.last_layer(&board2.padstacks), 1);
        // the routed via kept its real 600 um pad, not a hardcoded 0.6 mm
        // full-stack default (they coincide here; assert the span at least)
        let via2 = board2
            .items()
            .find(|(_, it)| it.base.component_no == 0 && matches!(it.kind, ItemKind::Via(_)))
            .map(|(_, it)| it.clone())
            .expect("via");
        assert_eq!(via2.first_layer(&board2.padstacks), 0);
        assert_eq!(via2.last_layer(&board2.padstacks), 1);
        // the pour survived with its obstacle flag
        let zone2 = board2
            .items()
            .find_map(|(_, it)| match &it.kind {
                ItemKind::ObstacleArea(a) if a.is_conduction => Some(a.clone()),
                _ => None,
            })
            .expect("conduction area");
        assert!(zone2.is_obstacle, "obstacle flag must survive");
    }

    // A DSN with a custom high-clearance net class: export→import must preserve
    // both the class's own clearance and its cross clearance to default. The
    // duplicate-"default" bug (finding #3) previously collapsed the cross
    // clearance back to the default on round trip.
    const CUSTOM_CLASS_DSN: &str = r#"(pcb "hv.dsn"
  (resolution um 10)
  (structure
    (layer F.Cu (type signal))
    (layer B.Cu (type signal))
    (boundary (rect pcb 0 0 100000 100000))
    (rule (width 200) (clearance 200))
  )
  (placement)
  (library
    (padstack "Via[0-1]"
      (shape (circle F.Cu 600 0 0))
      (shape (circle B.Cu 600 0 0))
      (attach off)
    )
  )
  (network
    (net "HV")
    (net "LV")
    (class power "HV" (rule (width 250) (clearance 600)))
  )
)"#;

    #[test]
    fn custom_net_class_clearance_survives_round_trip() {
        let board = import_dsn(CUSTOM_CLASS_DSN).expect("import");
        // sanity: the HV net really has a stricter clearance than default
        let hv0 = board.rules.nets.get_by_name("HV")[0].net_number;
        let hv_cl0 = board.rules.get_trace_clearance_class(hv0);
        assert_eq!(
            board
                .rules
                .clearance_matrix
                .get_value(hv_cl0, hv_cl0, 0, false),
            6000,
            "precondition: HV self-clearance is 600 um = 6000 units"
        );

        let json = export_kicad_json(&board);
        let board2 = import_kicad_json(&json).expect("re-import");

        let hv = board2.rules.nets.get_by_name("HV")[0].net_number;
        let lv = board2.rules.nets.get_by_name("LV")[0].net_number;
        let hv_cl = board2.rules.get_trace_clearance_class(hv);
        let lv_cl = board2.rules.get_trace_clearance_class(lv);
        let m = &board2.rules.clearance_matrix;
        assert_eq!(
            m.get_value(hv_cl, hv_cl, 0, false),
            6000,
            "HV self-clearance must survive the round trip"
        );
        assert_eq!(
            m.get_value(hv_cl, lv_cl, 0, false),
            6000,
            "HV<->LV (default) cross clearance must survive as the max, not collapse to default"
        );
    }
}
