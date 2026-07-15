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
    let res = board.resolution.max(1) as f64;
    let mm = |v: f64| v / res;
    let mut out = String::from("{\n  \"unit\": \"MM\",\n");
    out.push_str(&format!("  \"resolution\": {},\n", board.resolution));
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
    // net classes (the default class summary)
    let cl = board.rules.clearance_matrix.get_value(1, 1, 0, false) as f64;
    let hw = board.rules.get_min_trace_half_width() as f64;
    out.push_str(&format!(
        "  \"netClasses\": [{{\"name\": \"Default\", \"clearance\": {:.6}, \"traceWidth\": {:.6}, \"viaDiameter\": 0.6, \"viaDrill\": 0.3, \"netNames\": []}}],\n",
        mm(cl),
        mm(2.0 * hw)
    ));
    // nets
    out.push_str("  \"nets\": [\n");
    let net_count = board.rules.nets.max_net_no();
    for n in 1..=net_count {
        let name = board
            .rules
            .nets
            .get_by_no(n)
            .map(|x| x.name.clone())
            .unwrap_or_default();
        let comma = if n < net_count { "," } else { "" };
        out.push_str(&format!(
            "    {{\"id\": {n}, \"name\": \"{}\", \"className\": \"Default\", \"containsPlane\": false}}{comma}\n",
            esc(&name)
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
        let ItemKind::Via(v) = &item.kind else { continue };
        let net_name = item
            .base
            .net_nos
            .first()
            .and_then(|&n| board.rules.nets.get_by_no(n))
            .map(|x| x.name.clone())
            .unwrap_or_default();
        let bb = item.bounding_box(&board.padstacks);
        let (w, h) = (
            (bb.ur.x - bb.ll.x) as f64,
            (bb.ur.y - bb.ll.y) as f64,
        );
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
            mm(v.center.y as f64),
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
                    .map(|c| format!("{{\"x\": {:.6}, \"y\": {:.6}}}", mm(c.x), mm(c.y)))
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
                    mm(v.center.y as f64),
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
    use crate::io::kicad_json::import_kicad_json;
    use crate::io::import_dsn;

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
}
