//! SES session writer (port of the writing half of
//! `io/specctra/SesWriter.java`, simplified): emits the routed wires and
//! vias of a board as a Specctra session file, which KiCad and other
//! tools import back.

use crate::board::basic_board::BasicBoard;
use crate::board::ItemKind;

/// Exports the routed items of the board as a Specctra session file.
/// `design_name` is the name recorded in the session; `resolution` must
/// match the import (internal units per micrometer).
pub fn export_ses(board: &BasicBoard, design_name: &str, resolution: i32) -> String {
    let mut out = String::new();
    out.push_str(&format!("(session \"{design_name}.ses\"\n"));
    out.push_str("  (base_design \"");
    out.push_str(design_name);
    out.push_str(".dsn\")\n");
    out.push_str("  (routes \n");
    // Echo the imported physical unit (Java: SesWriter writes the design's
    // unit); hardcoding `um` relabelled non-um designs.
    out.push_str(&format!("    (resolution {} {resolution})\n", board.unit));
    out.push_str("    (parser\n      (host_cad \"freerouting-rs\")\n    )\n");

    // library_out: the via padstacks referenced by the session (Java:
    // SesWriter.writeLibrary), shapes written per layer in board units
    // like the rest of this writer
    let mut via_padstacks: Vec<usize> = board
        .items()
        .filter(|(_, it)| it.base.component_no == 0 && it.base.net_count() > 0)
        .filter_map(|(_, it)| match &it.kind {
            ItemKind::Via(v) => Some(v.padstack),
            _ => None,
        })
        .collect();
    for info_id in 0..board.rules.via_infos.count() {
        via_padstacks.push(board.rules.via_infos.get(info_id).get_padstack());
    }
    via_padstacks.sort_unstable();
    via_padstacks.dedup();
    out.push_str("    (library_out \n");
    for ps_no in via_padstacks {
        let Some(ps) = board.padstacks.get_by_no(ps_no) else {
            continue;
        };
        out.push_str(&format!("      (padstack \"{}\"\n", ps.name));
        for layer in 0..board.layer_structure.layer_count() {
            let Some(shape) = ps.get_shape(layer) else {
                continue;
            };
            let layer_name = &board.layer_structure.arr[layer].name;
            match shape {
                crate::geometry::planar::TileShape::Box(b) => {
                    out.push_str(&format!(
                        "        (shape (rect {} {} {} {} {}))\n",
                        layer_name, b.ll.x, b.ll.y, b.ur.x, b.ur.y
                    ));
                }
                other => {
                    out.push_str(&format!("        (shape (polygon {} 0", layer_name));
                    for i in 0..other.border_line_count() {
                        let c = other.corner_approx(i);
                        out.push_str(&format!(" {} {}", c.x.round() as i64, c.y.round() as i64));
                    }
                    out.push_str("))\n");
                }
            }
        }
        out.push_str("      )\n");
    }
    out.push_str("    )\n");
    out.push_str("    (network_out \n");

    for net_no in 1..=board.rules.nets.max_net_no() {
        let Some(net) = board.rules.nets.get_by_no(net_no) else {
            continue;
        };
        // collect the routed items of this net
        let mut wires = String::new();
        for (_, item) in board.items() {
            if !item.base.contains_net(net_no) {
                continue;
            }
            match &item.kind {
                ItemKind::PolylineTrace(t) => {
                    let layer_name = &board.layer_structure.arr[t.layer].name;
                    wires.push_str(&format!(
                        "        (wire\n          (path {} {}",
                        layer_name,
                        2 * t.half_width
                    ));
                    for corner in t.polyline.corner_approx_arr() {
                        wires.push_str(&format!(
                            "\n            {} {}",
                            corner.x.round() as i64,
                            corner.y.round() as i64
                        ));
                    }
                    wires.push_str("\n          )\n        )\n");
                }
                ItemKind::Via(v) => {
                    // only autoroute-inserted vias (component pins are part
                    // of the base design and are not written)
                    if item.base.component_no != 0 {
                        continue;
                    }
                    let Some(padstack) = board.padstacks.get_by_no(v.padstack) else {
                        continue;
                    };
                    wires.push_str(&format!(
                        "        (via \"{}\" {} {}\n        )\n",
                        padstack.name, v.center.x, v.center.y
                    ));
                }
                ItemKind::ObstacleArea(_) => {}
            }
        }
        if !wires.is_empty() {
            out.push_str(&format!("      (net \"{}\"\n", net.name));
            out.push_str(&wires);
            out.push_str("      )\n");
        }
    }
    out.push_str("    )\n  )\n)\n");
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::board::{Layer, LayerStructure};
    use crate::core::Padstacks;
    use crate::geometry::planar::{IntBox, IntPoint, Polyline, TileShape};
    use crate::io::dsn::parse_dsn;
    use crate::rules::{BoardRules, ClearanceMatrix};

    #[test]
    fn exports_wires_and_vias() {
        let stack = LayerStructure::new(vec![Layer::new("F.Cu", true), Layer::new("B.Cu", true)]);
        let matrix = ClearanceMatrix::get_default_instance(stack.clone(), 200);
        let mut rules = BoardRules::new(stack.clone(), matrix);
        rules.get_default_net_class();
        rules.nets.add("GND", 1, false);
        let mut padstacks = Padstacks::new(2);
        padstacks.add(
            "Via[0-1]_800:400_um",
            vec![
                Some(TileShape::Box(IntBox::from_coords(-400, -400, 400, 400))),
                Some(TileShape::Box(IntBox::from_coords(-400, -400, 400, 400))),
            ],
            true,
            false,
        );
        let mut board = BasicBoard::new(stack, rules, padstacks);
        board.insert_trace(
            Polyline::from_int_points(&[IntPoint::new(0, 0), IntPoint::new(5000, 0)]),
            0,
            100,
            vec![1],
            1,
        );
        board.insert_via(1, IntPoint::new(5000, 0), vec![1], 1, false);

        let ses = export_ses(&board, "test_board", 10);
        // the output is well-formed S-expression
        let parsed = parse_dsn(&ses).expect("SES not parseable");
        assert_eq!(parsed.name(), Some("session"));
        let routes = parsed.child("routes").expect("routes");
        // the library_out carries the via padstack definitions
        let library = routes.child("library_out").expect("library_out");
        let padstack = library.child("padstack").expect("padstack");
        assert_eq!(padstack.arg(), Some("Via[0-1]_800:400_um"));
        assert!(padstack.child("shape").is_some(), "per-layer shapes");
        let network_out = routes.child("network_out").expect("network_out");
        let net = network_out.child("net").expect("net");
        assert_eq!(net.arg(), Some("GND"));
        let wire = net.child("wire").expect("wire");
        let path = wire.child("path").expect("path");
        assert_eq!(path.arg(), Some("F.Cu"));
        assert_eq!(path.args().nth(1), Some("200")); // width = 2 * half width
        let via = net.child("via").expect("via");
        assert_eq!(via.arg(), Some("Via[0-1]_800:400_um"));
    }
}
