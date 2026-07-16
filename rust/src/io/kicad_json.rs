//! Port of `KiCadJsonReader.java`: building a board from the KiCad
//! plugin's board JSON (layers, net classes with clearance classes and
//! via rules, custom clearance rules, nets, components with shaped pads,
//! conduction areas, pre-routed traces/vias).
//!
//! Like the Java reader, the Y axis is negated on import (KiCad's Y axis
//! points down, the board's points up). Unlike Java — whose writer emits
//! un-negated Y, breaking its own round trip — the Rust writer negates
//! symmetrically.

use crate::board::basic_board::BasicBoard;
use crate::board::{FixedState, Item, ItemBase, Layer, LayerStructure};
use crate::core::Padstacks;
use crate::geometry::planar::{
    IntBox, IntOctagon, IntPoint, Point, PolygonShape, Polyline, PolylineArea, TileShape,
};
use crate::io::json::{parse_json, Json};
use crate::rules::{BoardRules, ClearanceMatrix, ViaInfo, ViaRule};

/// An octagon approximating a stadium/circle pad of half-extents
/// `dx`/`dy` centered at the origin (Java builds the same `IntOctagon`
/// for oval pads: the diagonals cut `(2 - sqrt(2)) * r` off the corners).
fn oval_shape(dx: f64, dy: f64) -> TileShape {
    let (lx, rx) = ((-dx).round() as i32, dx.round() as i32);
    let (ly, uy) = ((-dy).round() as i32, dy.round() as i32);
    let r = dx.min(dy).round() as i32;
    let cut = ((2.0 - std::f64::consts::SQRT_2) * r as f64).round() as i32;
    let oct = IntOctagon::new(
        lx,
        ly,
        rx,
        uy,
        lx - uy + cut,
        rx - ly - cut,
        lx + ly + cut,
        rx + uy - cut,
    );
    TileShape::Octagon(oct.normalize())
}

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
    let to_units = |v: f64| -> f64 {
        let mm = match unit.as_str() {
            "MIL" => v * 0.0254,
            "UM" => v / 1000.0,
            _ => v,
        };
        mm * resolution as f64
    };
    let to_int = |v: f64| to_units(v).round() as i32;
    // KiCad's Y axis points down; the board's points up (Java negates too)
    let point = |p: Option<&Json>| -> IntPoint {
        IntPoint::new(
            p.map(|q| to_int(q.num("x"))).unwrap_or(0),
            p.map(|q| -to_int(q.num("y"))).unwrap_or(0),
        )
    };

    // layers: everything that is not a plane is a signal layer
    let mut layers = Vec::new();
    for l in doc.arr("layers") {
        layers.push(Layer::new(
            l.str_or("name", "?"),
            !l.str_or("type", "signal").eq_ignore_ascii_case("plane"),
        ));
    }
    if layers.is_empty() {
        layers.push(Layer::new("F.Cu", true));
        layers.push(Layer::new("B.Cu", true));
    }
    let layer_count = layers.len();
    let stack = LayerStructure::new(layers);

    // clearance matrix: classes null, default, then one per net class;
    // 0.2 mm is the Java fallback for every unset pair. Each JSON net class
    // maps to a matrix column, REUSING the built-in "null"/"default" columns
    // for classes of those names instead of appending duplicates: a duplicate
    // "default" made the clearanceRules resolve to the first column while nets
    // used the appended one, silently dropping the round-tripped clearance.
    let net_classes = doc.arr("netClasses");
    let mut class_names: Vec<String> = vec!["null".into(), "default".into()];
    let mut matrix_cl: Vec<usize> = Vec::with_capacity(net_classes.len());
    for c in net_classes {
        let name = c.str_or("name", "?");
        let idx = class_names
            .iter()
            .position(|n| n.eq_ignore_ascii_case(&name))
            .unwrap_or_else(|| {
                class_names.push(name.clone());
                class_names.len() - 1
            });
        matrix_cl.push(idx);
    }
    let name_refs: Vec<&str> = class_names.iter().map(|s| s.as_str()).collect();
    let mut matrix = ClearanceMatrix::new(stack.clone(), &name_refs);
    matrix.set_default_value(to_int(match unit.as_str() {
        "MIL" => 0.2 / 0.0254,
        "UM" => 200.0,
        _ => 0.2,
    }));
    for (i, c) in net_classes.iter().enumerate() {
        let cl_no = matrix_cl[i];
        let val = to_int(c.num("clearance"));
        if val > 0 && cl_no >= 1 {
            matrix.set_value_on_all_layers(cl_no, cl_no, val);
            // set BOTH directions: the matrix must stay symmetric like Java's,
            // or a later get_value in the transposed direction reads a stale
            // default — the asymmetry behind finding #5c.
            matrix.set_value_on_all_layers(1, cl_no, val);
            matrix.set_value_on_all_layers(cl_no, 1, val);
        }
    }
    for rule in doc.arr("clearanceRules") {
        let (a, b) = (
            matrix.get_no(&rule.str_or("classA", "")),
            matrix.get_no(&rule.str_or("classB", "")),
        );
        if let (Some(a), Some(b)) = (a, b) {
            let v = to_int(rule.num("clearance"));
            matrix.set_value_on_all_layers(a, b, v);
            matrix.set_value_on_all_layers(b, a, v);
        }
    }
    let mut rules = BoardRules::new(stack.clone(), matrix);

    // net classes: trace widths and clearance classes per class. A "default"
    // (or "null") JSON class updates the built-in default net class in place
    // rather than creating a duplicate; genuinely new classes are appended.
    let default_class = rules.get_default_net_class();
    let mut class_index: Vec<(String, usize)> = vec![("default".to_string(), default_class)];
    let mut net_class_of: Vec<usize> = Vec::with_capacity(net_classes.len());
    for (i, c) in net_classes.iter().enumerate() {
        let name = c.str_or("name", "?");
        let cl_no = matrix_cl[i];
        let hw = to_int(c.num("traceWidth")) / 2;
        let nc = if cl_no <= 1 {
            if hw > 0 {
                rules
                    .net_classes
                    .get_mut(default_class)
                    .set_trace_half_width(hw);
            }
            rules
                .net_classes
                .get_mut(default_class)
                .set_trace_clearance_class(cl_no.max(1));
            default_class
        } else {
            let class = rules.append_net_class(&name);
            if hw > 0 {
                rules.net_classes.get_mut(class).set_trace_half_width(hw);
            }
            rules
                .net_classes
                .get_mut(class)
                .set_trace_clearance_class(cl_no);
            class_index.push((name, class));
            class
        };
        net_class_of.push(nc);
    }
    if let Some(&first) = net_class_of.first() {
        // the default class mirrors the first document class (Java keeps
        // 0.25 mm defaults; mirroring gives unclassed nets sane widths)
        let hw = rules.net_classes.get(first).get_trace_half_width(0);
        rules.set_default_trace_half_widths(hw);
    }

    // via padstacks and rules: a default via plus one per net class
    // (Java fallback: 0.8 mm diameter)
    let mut padstacks = Padstacks::new(layer_count);
    let default_via_mm: f64 = match unit.as_str() {
        "MIL" => 30.0,
        "UM" => 800.0,
        _ => 0.8,
    };
    let def_via_d = net_classes
        .iter()
        .find(|c| c.str_or("name", "").eq_ignore_ascii_case("default"))
        .map(|c| c.num("viaDiameter"))
        .filter(|&d| d > 0.0)
        .map(to_int)
        .unwrap_or_else(|| to_int(default_via_mm));
    let via_shapes = |d: i32| -> Vec<Option<TileShape>> {
        let r = d / 2;
        (0..layer_count)
            .map(|_| Some(TileShape::Box(IntBox::from_coords(-r, -r, r, r))))
            .collect()
    };
    let add_via_rule = |rules: &mut BoardRules,
                        padstacks: &mut Padstacks,
                        name: &str,
                        diameter: i32,
                        class: usize| {
        let ps = padstacks.add(name.to_string(), via_shapes(diameter), true, false);
        let cl = rules
            .net_classes
            .get(class)
            .default_item_clearance_classes
            .get(crate::rules::ItemClass::Via);
        if let Some(info) = rules.via_infos.add(ViaInfo::new(name, ps, cl, true)) {
            let mut rule = ViaRule::new(name);
            rule.append_via(info);
            rules.via_rules.push(rule);
            let rule_id = rules.via_rules.len() - 1;
            rules.net_classes.get_mut(class).set_via_rule(Some(rule_id));
        }
    };
    add_via_rule(
        &mut rules,
        &mut padstacks,
        "default_via",
        def_via_d,
        default_class,
    );
    for (i, c) in net_classes.iter().enumerate() {
        let name = c.str_or("name", "?");
        if name.eq_ignore_ascii_case("default") {
            continue;
        }
        let d = Some(c.num("viaDiameter"))
            .filter(|&d| d > 0.0)
            .map(to_int)
            .unwrap_or(def_via_d);
        let class = net_class_of[i];
        add_via_rule(&mut rules, &mut padstacks, &format!("via_{name}"), d, class);
    }

    // nets, assigned to their class; nets only referenced by pads, zones,
    // traces or vias are registered with the default class (Java does both)
    for n in doc.arr("nets") {
        let name = n.str_or("name", "");
        if name.is_empty() {
            continue;
        }
        let plane = n.get("containsPlane") == Some(&Json::Bool(true));
        let net_no = rules.nets.add(&name, 1, plane);
        let class_name = n.str_or("className", "");
        let class = class_index
            .iter()
            .find(|(n, _)| n.eq_ignore_ascii_case(&class_name))
            .map(|(_, c)| *c)
            .unwrap_or(default_class);
        if let Some(net) = rules.nets.get_by_no_mut(net_no) {
            net.set_class(class);
        }
    }
    let mut referenced: Vec<String> = Vec::new();
    for comp in doc.arr("components") {
        for pad in comp.arr("pads") {
            referenced.push(pad.str_or("netName", ""));
        }
    }
    for section in ["conductionAreas", "traces", "vias"] {
        for e in doc.arr(section) {
            referenced.push(e.str_or("netName", ""));
        }
    }
    for name in referenced {
        if !name.is_empty() && rules.nets.get_by_name(&name).is_empty() {
            let net_no = rules.nets.add(&name, 1, false);
            if let Some(net) = rules.nets.get_by_no_mut(net_no) {
                net.set_class(default_class);
            }
        }
    }
    let net_no_by_name = |rules: &BoardRules, name: &str| -> Option<i32> {
        rules.nets.get_by_name(name).first().map(|n| n.net_number)
    };

    let mut board = BasicBoard::new(stack, rules, padstacks);
    board.resolution = resolution;
    // `to_units` normalized every coordinate onto an mm basis (mm * resolution),
    // so the board's unit is millimetres regardless of the document's declared
    // unit. Persist it, or board_units_per_mm() would later assume micrometres
    // and mis-scale all mm reporting.
    board.unit = "mm".to_string();

    // components and pads: each pad becomes a system-fixed pin
    let mut component_no = 0i32;
    for comp in doc.arr("components") {
        component_no += 1;
        let pos = comp.get("position");
        let (px, py) = (
            pos.map(|p| to_units(p.num("x"))).unwrap_or(0.0),
            pos.map(|p| to_units(p.num("y"))).unwrap_or(0.0),
        );
        let rot = comp.num("rotation").to_radians();
        let (sin, cos) = rot.sin_cos();
        for pad in comp.arr("pads") {
            let off = pad.get("offset");
            let (ox, oy) = (
                off.map(|p| to_units(p.num("x"))).unwrap_or(0.0),
                off.map(|p| to_units(p.num("y"))).unwrap_or(0.0),
            );
            // the pin location in board coordinates: the component applies
            // the rotation to the pad offset, then Y is negated
            let x = (px + ox * cos - oy * sin).round() as i32;
            let y = (-py - ox * sin - oy * cos).round() as i32;
            let size = pad.get("size");
            let (dx, dy) = (
                size.map(|p| to_units(p.num("x"))).unwrap_or(0.0).max(2.0) / 2.0,
                size.map(|p| to_units(p.num("y"))).unwrap_or(0.0).max(2.0) / 2.0,
            );
            let shape = match pad.str_or("shape", "rect").to_ascii_lowercase().as_str() {
                "circle" => {
                    let r = dx.min(dy);
                    oval_shape(r, r)
                }
                "oval" => oval_shape(dx, dy),
                _ => TileShape::Box(IntBox::from_coords(
                    (-dx).round() as i32,
                    (-dy).round() as i32,
                    dx.round() as i32,
                    dy.round() as i32,
                )),
            };
            // pad layer span from the named layers; empty means all layers
            let named: Vec<usize> = pad
                .arr("layers")
                .iter()
                .filter_map(|l| l.as_str())
                .filter_map(|n| board.layer_structure.get_no(n))
                .collect();
            let (from_layer, to_layer) = if named.is_empty() {
                (0, layer_count - 1)
            } else {
                (*named.iter().min().unwrap(), *named.iter().max().unwrap())
            };
            let shapes: Vec<Option<TileShape>> = (0..layer_count)
                .map(|l| (from_layer..=to_layer).contains(&l).then(|| shape.clone()))
                .collect();
            let drillable = pad.num("drill") > 0.0;
            let ps_no = board.padstacks.add(
                format!("padstack_{}", board.padstacks.count() + 1),
                shapes,
                drillable,
                false,
            );
            let net = net_no_by_name(&board.rules, &pad.str_or("netName", ""));
            // a pad on a high-clearance net uses that net's clearance class, not
            // a hardcoded default (finding #3); through-hole pads fall back to
            // the default class, smd pads to the smd class.
            let pad_cl = board
                .rules
                .item_clearance_class_or(net.unwrap_or(0), if drillable { 1 } else { 2 });
            let mut base = ItemBase::new(
                component_no,
                net.map(|n| vec![n]).unwrap_or_default(),
                pad_cl,
            );
            base.component_no = component_no;
            base.fixed_state = FixedState::SystemFixed;
            let item = Item::new_via(base, ps_no, IntPoint::new(x, y), true);
            board.insert_item(item);
        }
    }

    // conduction areas (copper pours)
    for zone in doc.arr("conductionAreas") {
        let corners: Vec<Point> = zone
            .arr("polygon")
            .iter()
            .map(|p| Point::Int(point(Some(p))))
            .collect();
        if corners.len() < 3 {
            continue;
        }
        let layer = zone.num("layerIndex") as usize;
        if layer >= layer_count {
            continue;
        }
        let name = zone.str_or("netName", "");
        let net = net_no_by_name(&board.rules, &name);
        let zone_cl = board.rules.item_clearance_class_or(net.unwrap_or(0), 1);
        let area = PolylineArea::new(PolygonShape::new(corners), Vec::new());
        let id = board.insert_area(
            area,
            layer,
            &name,
            net.map(|n| vec![n]).unwrap_or_default(),
            zone_cl,
            // with a net it is a connectable pour; without, a keepout
            net.is_some(),
        );
        board.set_fixed_state(id, FixedState::UserFixed);
    }

    // pre-routed traces and vias, user-fixed like Java
    for t in doc.arr("traces") {
        let Some(net) = net_no_by_name(&board.rules, &t.str_or("netName", "")) else {
            continue;
        };
        let layer = t.num("layerIndex") as usize;
        let hw = (to_int(t.num("width")) / 2).max(1);
        let corners: Vec<IntPoint> = t.arr("points").iter().map(|p| point(Some(p))).collect();
        if corners.len() < 2 || layer >= layer_count {
            continue;
        }
        let polyline = Polyline::from_int_points(&corners);
        if !polyline.is_empty() {
            // pre-routed copper keeps its net's clearance class (finding #3)
            let cl = board.rules.get_trace_clearance_class(net);
            let id = board.insert_trace(polyline, layer, hw, vec![net], cl);
            board.set_fixed_state(id, FixedState::UserFixed);
        }
    }
    for v in doc.arr("vias") {
        let Some(net) = net_no_by_name(&board.rules, &v.str_or("netName", "")) else {
            continue;
        };
        let center = point(v.get("position"));
        let d = Some(v.num("diameter"))
            .filter(|&d| d > 0.0)
            .map(to_int)
            .unwrap_or(def_via_d);
        let from = (v.num("startLayerIndex") as usize).min(layer_count - 1);
        let to = (v.num("endLayerIndex") as usize)
            .min(layer_count - 1)
            .max(from);
        let r = d / 2;
        let ps = board.padstacks.add_shape_on_layers(
            TileShape::Box(IntBox::from_coords(-r, -r, r, r)),
            from,
            to,
        );
        let cl = board.rules.get_trace_clearance_class(net);
        let id = board.insert_via(ps, center, vec![net], cl, false);
        board.set_fixed_state(id, FixedState::UserFixed);
    }
    Ok(board)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::board::ItemKind;

    const MINI: &str = r#"{
      "unit": "MM",
      "layers": [
        {"index": 0, "name": "F.Cu", "type": "signal"},
        {"index": 1, "name": "B.Cu", "type": "signal"}
      ],
      "netClasses": [{"name": "Default", "clearance": 0.2, "traceWidth": 0.25,
                      "viaDiameter": 0.6, "viaDrill": 0.3, "netNames": ["GND"]},
                     {"name": "Power", "clearance": 0.4, "traceWidth": 0.5,
                      "viaDiameter": 0.8, "viaDrill": 0.4, "netNames": ["VCC"]}],
      "clearanceRules": [{"classA": "Default", "classB": "Power", "clearance": 0.3}],
      "nets": [{"id": 1, "name": "GND", "className": "Default", "containsPlane": false},
               {"id": 2, "name": "VCC", "className": "Power", "containsPlane": true}],
      "components": [{
        "reference": "R1", "value": "10k", "footprint": "R_0603",
        "position": {"x": 10.0, "y": -10.0}, "rotation": 0, "layer": "F.Cu",
        "pads": [
          {"name": "1", "netName": "GND", "shape": "circle",
           "size": {"x": 0.8, "y": 0.8}, "offset": {"x": -0.75, "y": 0},
           "drill": 0, "layers": ["F.Cu"]},
          {"name": "2", "netName": "GND", "shape": "oval",
           "size": {"x": 1.2, "y": 0.8}, "offset": {"x": 0.75, "y": 0},
           "drill": 0, "layers": ["F.Cu"]}
        ]
      }],
      "outline": {"corners": [], "clearance": 0.2},
      "traces": [{"netName": "VCC", "width": 0.5, "layerIndex": 0,
                  "points": [{"x": 1.0, "y": 1.0}, {"x": 2.0, "y": 1.0}]}],
      "vias": [{"netName": "VCC", "position": {"x": 2.0, "y": 1.0},
                "diameter": 0.8, "drill": 0.4, "startLayerIndex": 0, "endLayerIndex": 1}],
      "conductionAreas": [{"netName": "GND", "layerIndex": 1, "isObstacle": true,
                           "polygon": [{"x": 0, "y": 0}, {"x": 30, "y": 0},
                                       {"x": 30, "y": 30}, {"x": 0, "y": 30}]}]
    }"#;

    #[test]
    fn imports_a_kicad_board() {
        let board = import_kicad_json(MINI).expect("import");
        assert_eq!(board.layer_structure.layer_count(), 2);
        assert_eq!(board.rules.nets.max_net_no(), 2);
        let pads = board
            .items()
            .filter(|(_, it)| it.base.component_no != 0)
            .count();
        assert_eq!(pads, 2, "two pads expected");
        // the two GND pads are unconnected by wiring, but the GND pour
        // on layer 1 does not reach them on layer 0
        assert!(!board.net_is_completely_connected(1));
    }

    #[test]
    fn net_class_rules_are_applied() {
        let board = import_kicad_json(MINI).expect("import");
        // VCC is in the Power class: 0.25 mm half width at resolution 10000
        assert_eq!(board.rules.get_trace_half_width(2, 0), 2500);
        assert_eq!(board.rules.get_trace_half_width(1, 0), 1250);
        // the custom rule sets 0.3 mm between the two classes
        let a = board.rules.clearance_matrix.get_no("Default").unwrap();
        let b = board.rules.clearance_matrix.get_no("Power").unwrap();
        assert_eq!(board.rules.clearance_matrix.get_value(a, b, 0, false), 3000);
        // per-class vias resolve through the via rules
        let vcc_via = board.rules.via_padstack_for_net(2).unwrap();
        let d = board.padstacks.get_by_no(vcc_via).unwrap().bounding_box();
        assert_eq!(d.ur.x - d.ll.x, 8000, "0.8 mm Power via");
    }

    #[test]
    fn zones_traces_and_vias_are_imported() {
        let board = import_kicad_json(MINI).expect("import");
        let zones = board
            .items()
            .filter(|(_, it)| matches!(&it.kind, ItemKind::ObstacleArea(a) if a.is_conduction))
            .count();
        assert_eq!(zones, 1, "the GND pour is a conduction area");
        let traces = board
            .items()
            .filter(|(_, it)| matches!(it.kind, ItemKind::PolylineTrace(_)))
            .count();
        assert_eq!(traces, 1);
        let vias = board
            .items()
            .filter(|(_, it)| it.base.component_no == 0 && matches!(it.kind, ItemKind::Via(_)))
            .count();
        assert_eq!(vias, 1);
        // KiCad Y points down: y=1.0 mm lands at board y = -10000
        let (_, via) = board
            .items()
            .find(|(_, it)| it.base.component_no == 0 && matches!(it.kind, ItemKind::Via(_)))
            .unwrap();
        let ItemKind::Via(v) = &via.kind else {
            unreachable!()
        };
        assert_eq!(v.center, IntPoint::new(20000, -10000));
    }
}
