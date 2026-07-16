//! The KiCad board JSON writer (Java: `KiCadJsonWriter`): serializes the
//! board in the same schema the reader consumes — layers, net classes,
//! nets, components with pads, and the routed traces and vias.

use crate::board::basic_board::BasicBoard;
use crate::board::ItemKind;

fn esc(s: &str) -> String {
    s.replace('\\', "\\\\").replace('"', "\\\"")
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
            format!(
                "{{\"name\": \"{}\", \"clearance\": {:.6}, \"traceWidth\": {:.6}, \"viaDiameter\": 0.6, \"viaDrill\": 0.3, \"netNames\": [{}]}}",
                esc(class.get_name()),
                mm(cl),
                mm(2.0 * hw),
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
    // nets, each tagged with its real class name
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
            "    {{\"id\": {n}, \"name\": \"{}\", \"className\": \"{}\", \"containsPlane\": false}}{comma}\n",
            esc(&name),
            esc(&class_name),
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
        let layer_name = board
            .padstacks
            .get_by_no(v.padstack)
            .and_then(|p| board.layer_structure.arr.get(p.from_layer()))
            .map(|l| l.name.clone())
            .unwrap_or_else(|| "F.Cu".into());
        comp_pads.entry(item.base.component_no).or_default().push(format!(
            "{{\"name\": \"\", \"netName\": \"{}\", \"shape\": \"rect\", \"size\": {{\"x\": {:.6}, \"y\": {:.6}}}, \"offset\": {{\"x\": {:.6}, \"y\": {:.6}}}, \"drill\": {}, \"layers\": [\"{}\"]}}",
            esc(&net_name),
            mm(w),
            mm(h),
            mm(v.center.x as f64),
            mm(-v.center.y as f64),
            if through { 1 } else { 0 },
            esc(&layer_name),
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
    out.push_str("  \"outline\": {\"corners\": [], \"clearance\": 0.2},\n");
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
                vias.push(format!(
                    "    {{\"id\": {id}, \"netName\": \"{}\", \"position\": {{\"x\": {:.6}, \"y\": {:.6}}}, \"diameter\": 0.6, \"drill\": 0.3, \"startLayerIndex\": 0, \"endLayerIndex\": {}}}",
                    esc(&net_name),
                    mm(v.center.x as f64),
                    mm(-v.center.y as f64),
                    layer_count - 1
                ));
            }
            _ => {}
        }
    }
    out.push_str(&format!("  \"traces\": [\n{}\n  ],\n", traces.join(",\n")));
    out.push_str(&format!("  \"vias\": [\n{}\n  ],\n", vias.join(",\n")));
    out.push_str("  \"conductionAreas\": []\n}\n");
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
