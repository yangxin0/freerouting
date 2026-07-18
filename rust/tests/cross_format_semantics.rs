//! Cross-format integration contract for the semantic subset represented by
//! DSN, `.rules`, SES, and the KiCad JSON interchange format.

use freerouting::board::{BasicBoard, ItemKind};
use freerouting::io::{
    export_dsn, export_kicad_json_checked, export_ses, import_dsn, import_kicad_json, import_ses,
    read_rules, strip_wiring, write_rules,
};
use freerouting::rules::ViaRule;

const DESIGN: &str = r#"(pcb "cross-format.dsn"
  (resolution um 10)
  (structure
    (layer F.Cu (type signal))
    (layer B.Cu (type signal))
    (boundary
      (path signal 0 0 0 4000 0 4000 3000 0 3000)
      (clearance_class edge)
    )
    (rule
      (width 200)
      (clearance 200)
      (clearance 500 (type edge_default))
    )
  )
  (placement)
  (library
    (padstack VIA_PS
      (shape (rect F.Cu -300 -300 300 300))
      (shape (rect B.Cu -300 -300 300 300))
      (attach off)
    )
  )
  (network
    (net SIG)
    (via VIA_INFO VIA_PS strict)
    (via_rule SIG_VIAS VIA_INFO)
    (class strict SIG
      (via_rule SIG_VIAS)
      (rule (width 300) (clearance 350))
    )
  )
  (wiring
    (wire
      (path F.Cu 300 1000 1000 2000 1000)
      (net SIG 1)
    )
    (via VIA_PS 2000 1000
      (net SIG 1)
    )
    (wire
      (path B.Cu 300 2000 1000 3000 1000)
      (net SIG 1)
    )
  )
)"#;

#[derive(Debug, PartialEq, Eq)]
struct RouteSemantics {
    traces: Vec<TraceSemantics>,
    vias: Vec<ViaSemantics>,
    connected_route_items: usize,
}

#[derive(Debug, PartialEq, Eq, PartialOrd, Ord)]
struct TraceSemantics {
    layer: String,
    width_um: i64,
    points_um: Vec<(i64, i64)>,
    clearance_class: String,
    clearance_explicit: bool,
}

#[derive(Debug, PartialEq, Eq, PartialOrd, Ord)]
struct ViaSemantics {
    padstack: String,
    center_um: (i64, i64),
    clearance_class: String,
    layer_span: (usize, usize),
    attach_allowed: bool,
    clearance_explicit: bool,
}

#[derive(Debug, PartialEq, Eq)]
struct ViaInfoSemantics {
    name: String,
    padstack: String,
    clearance_class: String,
    layer_span: (usize, usize),
    attach_allowed: bool,
}

#[derive(Debug, PartialEq, Eq)]
struct ViaRuleSemantics {
    name: String,
    infos: Vec<ViaInfoSemantics>,
}

#[derive(Debug, PartialEq, Eq)]
struct BoardSemantics {
    layers: Vec<(String, bool)>,
    outline: (Vec<(i64, i64)>, i64),
    net_class: String,
    trace_widths_um: Vec<i64>,
    trace_clearance_class: String,
    default_clearance_um: Vec<i64>,
    strict_clearance_um: Vec<i64>,
    edge_to_default_um: Vec<i64>,
    via_rule: ViaRuleSemantics,
    route: RouteSemantics,
}

fn um(board: &BasicBoard, value: f64) -> i64 {
    (value * 1000.0 / board.board_units_per_mm()).round() as i64
}

fn clearance_um(board: &BasicBoard, first: &str, second: &str, layer: usize) -> i64 {
    let first = board
        .rules
        .clearance_matrix
        .get_no(first)
        .unwrap_or_else(|| panic!("missing clearance class {first:?}"));
    let second = board
        .rules
        .clearance_matrix
        .get_no(second)
        .unwrap_or_else(|| panic!("missing clearance class {second:?}"));
    um(
        board,
        board
            .rules
            .clearance_matrix
            .get_value(first, second, layer, false) as f64,
    )
}

fn route_semantics(board: &BasicBoard, net_no: i32) -> RouteSemantics {
    let mut traces = Vec::new();
    let mut vias = Vec::new();
    let mut first_route_id = None;
    let mut route_count = 0;

    for (id, item) in board.items() {
        if item.base.component_no != 0 || !item.base.contains_net(net_no) {
            continue;
        }
        let clearance_name = board
            .rules
            .clearance_matrix
            .get_name(item.base.clearance_class)
            .unwrap_or("?")
            .to_string();
        match &item.kind {
            ItemKind::PolylineTrace(trace) => {
                first_route_id.get_or_insert(*id);
                route_count += 1;
                let mut points = trace
                    .polyline
                    .corner_approx_arr()
                    .into_iter()
                    .map(|point| (um(board, point.x), um(board, point.y)))
                    .collect::<Vec<_>>();
                let mut reversed = points.clone();
                reversed.reverse();
                if reversed < points {
                    points = reversed;
                }
                traces.push(TraceSemantics {
                    layer: board.layer_structure.arr[trace.layer].name.clone(),
                    width_um: um(board, 2.0 * trace.half_width as f64),
                    points_um: points,
                    clearance_class: clearance_name,
                    clearance_explicit: item.base.clearance_class_explicit,
                });
            }
            ItemKind::Via(via) => {
                first_route_id.get_or_insert(*id);
                route_count += 1;
                let padstack = board
                    .padstacks
                    .get_by_no(via.padstack)
                    .expect("route via padstack");
                vias.push(ViaSemantics {
                    padstack: padstack.name.clone(),
                    center_um: (
                        um(board, via.center.x as f64),
                        um(board, via.center.y as f64),
                    ),
                    clearance_class: clearance_name,
                    layer_span: (padstack.from_layer(), padstack.to_layer()),
                    attach_allowed: via.attach_allowed,
                    clearance_explicit: item.base.clearance_class_explicit,
                });
            }
            ItemKind::ObstacleArea(_) => {}
        }
    }
    traces.sort();
    vias.sort();

    let connected_route_items = first_route_id
        .map(|id| {
            board
                .get_connected_set(id, net_no)
                .into_iter()
                .filter(|connected| {
                    board.get_item(*connected).is_some_and(|item| {
                        item.base.component_no == 0
                            && matches!(item.kind, ItemKind::PolylineTrace(_) | ItemKind::Via(_))
                    })
                })
                .count()
        })
        .unwrap_or(0);
    assert_eq!(
        connected_route_items, route_count,
        "route must remain connected"
    );

    RouteSemantics {
        traces,
        vias,
        connected_route_items,
    }
}

fn semantics(board: &BasicBoard) -> BoardSemantics {
    let net = board
        .rules
        .nets
        .get_by_name("SIG")
        .into_iter()
        .next()
        .expect("SIG net");
    let class = board.rules.net_classes.get(net.get_class());
    let trace_clearance_class = board
        .rules
        .clearance_matrix
        .get_name(class.get_trace_clearance_class())
        .expect("trace clearance class")
        .to_string();
    let rule_id = class.get_via_rule().expect("strict via rule");
    let rule = &board.rules.via_rules[rule_id];
    let via_infos = rule
        .vias()
        .iter()
        .map(|id| {
            let info = board.rules.via_infos.get(*id);
            let padstack = board
                .padstacks
                .get_by_no(info.get_padstack())
                .expect("via-info padstack");
            ViaInfoSemantics {
                name: info.get_name().to_string(),
                padstack: padstack.name.clone(),
                clearance_class: board
                    .rules
                    .clearance_matrix
                    .get_name(info.get_clearance_class())
                    .expect("via-info clearance class")
                    .to_string(),
                layer_span: (padstack.from_layer(), padstack.to_layer()),
                attach_allowed: info.attach_smd_allowed(),
            }
        })
        .collect();

    let (outline_points, outline_clearance) = board.outline.as_ref().expect("board outline");
    let mut outline_points: Vec<_> = outline_points
        .iter()
        .map(|point| (um(board, point.x as f64), um(board, point.y as f64)))
        .collect();
    outline_points.sort();
    let layer_count = board.layer_structure.layer_count();

    BoardSemantics {
        layers: board
            .layer_structure
            .arr
            .iter()
            .map(|layer| (layer.name.clone(), layer.is_signal))
            .collect(),
        outline: (outline_points, um(board, *outline_clearance as f64)),
        net_class: class.get_name().to_string(),
        trace_widths_um: (0..layer_count)
            .map(|layer| um(board, 2.0 * class.get_trace_half_width(layer) as f64))
            .collect(),
        trace_clearance_class,
        default_clearance_um: (0..layer_count)
            .map(|layer| clearance_um(board, "default", "default", layer))
            .collect(),
        strict_clearance_um: (0..layer_count)
            .map(|layer| clearance_um(board, "strict", "strict", layer))
            .collect(),
        edge_to_default_um: (0..layer_count)
            .map(|layer| clearance_um(board, "edge", "default", layer))
            .collect(),
        via_rule: ViaRuleSemantics {
            name: rule.name.clone(),
            infos: via_infos,
        },
        route: route_semantics(board, net.net_number),
    }
}

fn scramble_rules_owned_state(board: &mut BasicBoard) {
    let net_no = board
        .rules
        .nets
        .get_by_name("SIG")
        .into_iter()
        .next()
        .expect("SIG net")
        .net_number;
    let default_class = board
        .rules
        .net_classes
        .get_by_name("default")
        .expect("default net class");
    let strict_class = board
        .rules
        .net_classes
        .get_by_name("strict")
        .expect("strict net class");
    let default_clearance = board
        .rules
        .clearance_matrix
        .get_no("default")
        .expect("default clearance class");
    let strict_clearance = board
        .rules
        .clearance_matrix
        .get_no("strict")
        .expect("strict clearance class");
    let edge_clearance = board
        .rules
        .clearance_matrix
        .get_no("edge")
        .expect("edge clearance class");
    let via_info = board
        .rules
        .via_infos
        .get_by_name("VIA_INFO")
        .expect("VIA_INFO");
    let via_rule = board.rules.get_via_rule("SIG_VIAS").expect("SIG_VIAS");

    board
        .rules
        .nets
        .get_by_no_mut(net_no)
        .expect("SIG net")
        .set_class(default_class);
    {
        let class = board.rules.net_classes.get_mut(strict_class);
        class.set_trace_half_width(17);
        class.set_trace_clearance_class(default_clearance);
        class
            .default_item_clearance_classes
            .set_all(default_clearance);
        class.set_via_rule(None);
    }

    let matrix = &mut board.rules.clearance_matrix;
    matrix.set_value_on_all_layers(default_clearance, default_clearance, 42);
    matrix.set_value_on_all_layers(strict_clearance, strict_clearance, 44);
    matrix.set_value_on_all_layers(edge_clearance, default_clearance, 46);
    matrix.set_value_on_all_layers(default_clearance, edge_clearance, 46);

    let info = board.rules.via_infos.get_mut(via_info);
    info.set_padstack(0);
    info.set_clearance_class(default_clearance);
    info.set_attach_smd_allowed(true);
    board.rules.via_rules[via_rule] = ViaRule::new("SIG_VIAS");

    let route_items: Vec<_> = board
        .items()
        .filter_map(|(id, item)| {
            (item.base.component_no == 0 && item.base.contains_net(net_no)).then_some(*id)
        })
        .collect();
    for id in route_items {
        board.set_item_clearance_class(id, default_clearance);
        board.set_item_clearance_class_explicit(id, false);
    }
}

#[test]
fn supported_formats_preserve_common_board_semantics() {
    let original = import_dsn(DESIGN).expect("DSN import");
    let expected = semantics(&original);

    let dsn = export_dsn(&original).expect("DSN export");
    let dsn_reloaded = import_dsn(&dsn).expect("DSN reload");
    assert_eq!(semantics(&dsn_reloaded), expected, "DSN round trip");

    let rules = write_rules(&original, "cross-format").expect("rules export");
    let mut rules_reloaded = import_dsn(DESIGN).expect("rules base DSN");
    scramble_rules_owned_state(&mut rules_reloaded);
    read_rules(&mut rules_reloaded, &rules).expect("rules reload");
    assert_eq!(semantics(&rules_reloaded), expected, "rules round trip");

    let session = export_ses(&original, "cross-format", original.resolution).expect("SES export");
    let mut session_reloaded = import_dsn(&strip_wiring(DESIGN)).expect("SES base DSN");
    let summary = import_ses(&mut session_reloaded, &session).expect("SES reload");
    assert_eq!((summary.wires, summary.vias), (2, 1));
    assert!(summary.unknown_nets.is_empty());
    assert_eq!(semantics(&session_reloaded), expected, "SES round trip");

    let json = export_kicad_json_checked(&original).expect("KiCad JSON export");
    let json_reloaded = import_kicad_json(&json).expect("KiCad JSON reload");
    assert_eq!(semantics(&json_reloaded), expected, "KiCad JSON round trip");
}
