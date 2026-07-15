//! Port of `KiCadJsonReader.java`: building a board from the KiCad
//! plugin's board JSON (layers, net classes, nets, components with pads,
//! outline, pre-routed traces/vias).

use crate::board::basic_board::BasicBoard;
use crate::board::{Item, ItemBase, Layer, LayerStructure};
use crate::core::Padstacks;
use crate::geometry::planar::{IntBox, IntPoint, Polyline, TileShape};
use crate::io::json::{parse_json, Json};
use crate::rules::{BoardRules, ClearanceMatrix};

/// Reads a KiCad board JSON document into a board (Java:
/// `KiCadJsonReader.readBoard`). Coordinates are in the document's unit
/// (default mm), converted at the Java default resolution of 10000
/// board units per mm (0.1 µm).
pub fn import_kicad_json(content: &str) -> Result<BasicBoard, String> {
    let doc = parse_json(content)?;
    let unit = doc.str_or("unit", "MM");
    let resolution = match doc.get("resolution").and_then(|v| v.as_f64()) {
        Some(r) if r > 1.0 => r as i32,
        _ => 10_000, // Java: 0.1 µm default for mm
    };
    let to_units = |v: f64| -> i32 {
        let mm = match unit.as_str() {
            "MIL" => v * 0.0254,
            "UM" => v / 1000.0,
            _ => v,
        };
        (mm * resolution as f64).round() as i32
    };

    // layers
    let mut layers = Vec::new();
    for l in doc.arr("layers") {
        layers.push(Layer::new(
            &l.str_or("name", "?"),
            l.str_or("type", "signal") == "signal",
        ));
    }
    if layers.is_empty() {
        return Err("no layers".into());
    }
    let layer_count = layers.len();
    let stack = LayerStructure::new(layers);

    // rules: default clearance from the first net class
    let default_clearance = doc
        .arr("netClasses")
        .first()
        .map(|c| to_units(c.num("clearance")))
        .filter(|&c| c > 0)
        .unwrap_or(to_units(0.2));
    let matrix = ClearanceMatrix::get_default_instance(stack.clone(), default_clearance);
    let mut rules = BoardRules::new(stack.clone(), matrix);
    rules.get_default_net_class();

    // nets
    for n in doc.arr("nets") {
        let name = n.str_or("name", "");
        if !name.is_empty() {
            rules.nets.add(&name, 1, n.get("containsPlane").map(|v| v == &Json::Bool(true)).unwrap_or(false));
        }
    }
    let net_no_by_name = |rules: &BoardRules, name: &str| -> Option<i32> {
        rules
            .nets
            .get_by_name(name)
            .first()
            .map(|n| n.net_number)
    };

    // net class widths
    for c in doc.arr("netClasses") {
        let width = to_units(c.num("traceWidth"));
        if width > 0 {
            rules.set_default_trace_half_widths(width / 2);
        }
        break; // default class only; per-class assignment follows Java's mapper
    }

    let mut padstacks = Padstacks::new(layer_count);
    // via padstack from the first net class via geometry
    let via_d = doc
        .arr("netClasses")
        .first()
        .map(|c| to_units(c.num("viaDiameter")))
        .filter(|&d| d > 0)
        .unwrap_or(to_units(0.6));
    let r = via_d / 2;
    let via_shapes: Vec<Option<TileShape>> = (0..layer_count)
        .map(|_| Some(TileShape::Box(IntBox::from_coords(-r, -r, r, r))))
        .collect();
    padstacks.add(
        format!("Via[0-{}]_{}:{}_um", layer_count - 1, via_d, via_d / 2),
        via_shapes,
        true,
        true,
    );

    let mut board = BasicBoard::new(stack, rules, padstacks);
    board.resolution = resolution;

    // components and pads: each pad becomes a pin (a via-like drill item
    // tagged with the component)
    let mut component_no = 0i32;
    for comp in doc.arr("components") {
        component_no += 1;
        let cx = to_units(comp.num("position").max(0.0));
        let _ = cx;
        let pos = comp.get("position");
        let (px, py) = (
            pos.map(|p| to_units(p.num("x"))).unwrap_or(0),
            pos.map(|p| to_units(p.num("y"))).unwrap_or(0),
        );
        let rot = comp.num("rotation").to_radians();
        for pad in comp.arr("pads") {
            let off = pad.get("offset");
            let (ox, oy) = (
                off.map(|p| to_units(p.num("x"))).unwrap_or(0) as f64,
                off.map(|p| to_units(p.num("y"))).unwrap_or(0) as f64,
            );
            // rotate the offset by the component rotation
            let (sin, cos) = rot.sin_cos();
            let x = px + (ox * cos - oy * sin).round() as i32;
            let y = py + (ox * sin + oy * cos).round() as i32;
            let size = pad.get("size");
            let (w, h) = (
                size.map(|p| to_units(p.num("x"))).unwrap_or(0).max(2) / 2,
                size.map(|p| to_units(p.num("y"))).unwrap_or(0).max(2) / 2,
            );
            let net = net_no_by_name(&board.rules, &pad.str_or("netName", ""));
            // pad layers: through-hole when drill > 0, else the named layer
            let through = pad.num("drill") > 0.0;
            let (from_layer, to_layer) = if through {
                (0, layer_count - 1)
            } else {
                let lname = pad
                    .arr("layers")
                    .first()
                    .and_then(|l| l.as_str())
                    .unwrap_or("F.Cu");
                let l = board.layer_structure.get_no(lname).unwrap_or(0);
                (l, l)
            };
            let ps_no = board.padstacks.count() + 1;
            board.padstacks.add_shape_on_layers(
                TileShape::Box(IntBox::from_coords(-w, -h, w, h)),
                from_layer,
                to_layer,
            );
            let mut base = ItemBase::new(
                component_no,
                net.map(|n| vec![n]).unwrap_or_default(),
                1,
            );
            base.component_no = component_no;
            let item = Item::new_via(base, ps_no, IntPoint::new(x, y), true);
            board.insert_item(item);
        }
    }

    // pre-routed traces and vias
    for t in doc.arr("traces") {
        let Some(net) = net_no_by_name(&board.rules, &t.str_or("netName", "")) else {
            continue;
        };
        let layer = t.num("layerIndex") as usize;
        let hw = (to_units(t.num("width")) / 2).max(1);
        let corners: Vec<IntPoint> = t
            .arr("points")
            .iter()
            .map(|p| IntPoint::new(to_units(p.num("x")), to_units(p.num("y"))))
            .collect();
        if corners.len() < 2 {
            continue;
        }
        let polyline = Polyline::from_int_points(&corners);
        if !polyline.is_empty() && layer < layer_count {
            board.insert_trace(polyline, layer, hw, vec![net], 1);
        }
    }
    for v in doc.arr("vias") {
        let Some(net) = net_no_by_name(&board.rules, &v.str_or("netName", "")) else {
            continue;
        };
        let pos = v.get("position");
        let (x, y) = (
            pos.map(|p| to_units(p.num("x"))).unwrap_or(0),
            pos.map(|p| to_units(p.num("y"))).unwrap_or(0),
        );
        board.insert_via(1, IntPoint::new(x, y), vec![net], 1, false);
    }
    Ok(board)
}

#[cfg(test)]
mod tests {
    use super::*;

    const MINI: &str = r#"{
      "unit": "MM",
      "layers": [
        {"index": 0, "name": "F.Cu", "type": "signal"},
        {"index": 1, "name": "B.Cu", "type": "signal"}
      ],
      "netClasses": [{"name": "Default", "clearance": 0.2, "traceWidth": 0.25,
                      "viaDiameter": 0.6, "viaDrill": 0.3, "netNames": ["GND"]}],
      "nets": [{"id": 1, "name": "GND", "className": "Default", "containsPlane": false}],
      "components": [{
        "reference": "R1", "value": "10k", "footprint": "R_0603",
        "position": {"x": 10.0, "y": -10.0}, "rotation": 0, "layer": "F.Cu",
        "pads": [
          {"name": "1", "netName": "GND", "shape": "rect",
           "size": {"x": 0.8, "y": 0.8}, "offset": {"x": -0.75, "y": 0},
           "drill": 0, "layers": ["F.Cu"]},
          {"name": "2", "netName": "GND", "shape": "rect",
           "size": {"x": 0.8, "y": 0.8}, "offset": {"x": 0.75, "y": 0},
           "drill": 0, "layers": ["F.Cu"]}
        ]
      }],
      "outline": {"corners": [], "clearance": 0.2},
      "traces": [], "vias": [], "conductionAreas": []
    }"#;

    #[test]
    fn imports_a_kicad_board() {
        let board = import_kicad_json(MINI).expect("import");
        assert_eq!(board.layer_structure.layer_count(), 2);
        assert_eq!(board.rules.nets.max_net_no(), 1);
        let pads = board.items().count();
        assert_eq!(pads, 2, "two pads expected");
        // the two GND pads are unconnected: the router has work
        assert!(!board.net_is_completely_connected(1));
    }
}
