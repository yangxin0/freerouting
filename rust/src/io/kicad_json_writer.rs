//! The KiCad board JSON writer (Java: `KiCadJsonWriter`): serializes the
//! board in the same schema the reader consumes — layers, net classes,
//! nets, components with pads, and the routed traces and vias.

use crate::board::basic_board::BasicBoard;
use crate::board::{AngleRestriction, FixedState, ItemKind};
use crate::rules::ItemClass;

/// A board state that the KiCad JSON schema cannot serialize losslessly.
///
/// The writer validates the complete reference graph before emitting any
/// bytes.  This is intentionally stricter than the reader: a malformed or
/// unsupported in-memory board must fail instead of being repaired with an
/// empty name, a default class, or an omitted item.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum KicadJsonWriteError {
    InvalidReferenceGraph {
        path: String,
        message: String,
    },
    InvalidBoard {
        reason: &'static str,
    },
    InvalidPadstack {
        padstack: usize,
        reason: &'static str,
    },
    DanglingRuleReference {
        owner: String,
        kind: &'static str,
        number: usize,
    },
    DanglingItemReference {
        item_id: crate::board::basic_board::ItemId,
        kind: &'static str,
        number: i64,
    },
    UnrepresentableRule {
        owner: String,
        reason: &'static str,
    },
    UnrepresentableItem {
        item_id: crate::board::basic_board::ItemId,
        reason: &'static str,
    },
}

impl std::fmt::Display for KicadJsonWriteError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::InvalidReferenceGraph { path, message } => {
                write!(f, "invalid board reference graph at {path}: {message}")
            }
            Self::InvalidBoard { reason } => write!(f, "invalid board: {reason}"),
            Self::InvalidPadstack { padstack, reason } => {
                write!(f, "invalid padstack {padstack}: {reason}")
            }
            Self::DanglingRuleReference {
                owner,
                kind,
                number,
            } => write!(f, "{owner} references missing {kind} {number}"),
            Self::DanglingItemReference {
                item_id,
                kind,
                number,
            } => write!(f, "item {item_id} references missing {kind} {number}"),
            Self::UnrepresentableRule { owner, reason } => {
                write!(f, "KiCad JSON cannot represent {owner}: {reason}")
            }
            Self::UnrepresentableItem { item_id, reason } => {
                write!(f, "KiCad JSON cannot represent item {item_id}: {reason}")
            }
        }
    }
}

impl std::error::Error for KicadJsonWriteError {}

fn esc(s: &str) -> String {
    crate::io::json::escape(s)
}

fn fixed_state_token(state: FixedState) -> &'static str {
    match state {
        FixedState::Unfixed => "unfixed",
        FixedState::ShoveFixed => "shove_fixed",
        FixedState::UserFixed => "user_fixed",
        FixedState::SystemFixed => "system_fixed",
    }
}

fn item_class_token(class: ItemClass) -> &'static str {
    match class {
        ItemClass::None => "none",
        ItemClass::Trace => "trace",
        ItemClass::Via => "via",
        ItemClass::Pin => "pin",
        ItemClass::Smd => "smd",
        ItemClass::Area => "area",
    }
}

fn angle_restriction_token(value: AngleRestriction) -> &'static str {
    match value {
        AngleRestriction::None => "none",
        AngleRestriction::FortyfiveDegree => "fortyfive_degree",
        AngleRestriction::NinetyDegree => "ninety_degree",
    }
}

fn validate_board_for_kicad_json(board: &BasicBoard) -> Result<(), KicadJsonWriteError> {
    use std::collections::HashSet;

    let layer_count = board.layer_structure.layer_count();
    if layer_count == 0 {
        return Err(KicadJsonWriteError::InvalidBoard {
            reason: "there are no board layers",
        });
    }
    if board.resolution <= 0 {
        return Err(KicadJsonWriteError::InvalidBoard {
            reason: "the coordinate resolution is not positive",
        });
    }
    if !matches!(
        board.unit.to_ascii_lowercase().as_str(),
        "um" | "mil" | "inch" | "in" | "mm" | "cm"
    ) {
        return Err(KicadJsonWriteError::InvalidBoard {
            reason: "the physical unit is unsupported",
        });
    }

    let mut layer_names = HashSet::new();
    for layer in &board.layer_structure.arr {
        if layer.name.is_empty() || !layer_names.insert(layer.name.to_ascii_lowercase()) {
            return Err(KicadJsonWriteError::InvalidBoard {
                reason: "layer names must be non-empty and unique",
            });
        }
    }

    let matrix = &board.rules.clearance_matrix;
    let clearance_count = matrix.get_class_count();
    if clearance_count < 2 {
        return Err(KicadJsonWriteError::InvalidBoard {
            reason: "the clearance matrix lacks its default class",
        });
    }
    if matrix.get_layer_count() != layer_count {
        return Err(KicadJsonWriteError::InvalidBoard {
            reason: "the clearance matrix layer count does not match the board",
        });
    }
    let mut clearance_names = HashSet::new();
    for index in 0..clearance_count {
        let Some(name) = matrix.get_name(index) else {
            return Err(KicadJsonWriteError::InvalidBoard {
                reason: "the clearance matrix contains an unnamed row",
            });
        };
        if name.is_empty() || !clearance_names.insert(name.to_ascii_lowercase()) {
            return Err(KicadJsonWriteError::InvalidBoard {
                reason: "clearance-class names must be non-empty and unique",
            });
        }
    }

    if board.padstacks.board_layer_count != layer_count {
        return Err(KicadJsonWriteError::InvalidBoard {
            reason: "the padstack library layer count does not match the board",
        });
    }
    let mut padstack_names = HashSet::new();
    for padstack_no in 1..=board.padstacks.count() {
        let padstack = board
            .padstacks
            .get_by_no(padstack_no)
            .expect("padstack numbers are contiguous");
        if padstack.name.is_empty() || !padstack_names.insert(padstack.name.to_ascii_lowercase()) {
            return Err(KicadJsonWriteError::InvalidPadstack {
                padstack: padstack_no,
                reason: "its name is empty or duplicates another padstack",
            });
        }
        let populated: Vec<usize> = (0..layer_count)
            .filter(|&layer| padstack.get_shape(layer).is_some())
            .collect();
        if populated.is_empty() {
            return Err(KicadJsonWriteError::InvalidPadstack {
                padstack: padstack_no,
                reason: "it has no copper shape",
            });
        }
        if padstack.from_layer() != populated[0]
            || padstack.to_layer() != *populated.last().expect("not empty")
            || !padstack
                .has_shapes_at_transition_endpoints(padstack.from_layer(), padstack.to_layer())
        {
            return Err(KicadJsonWriteError::InvalidPadstack {
                padstack: padstack_no,
                reason: "its layer-span endpoints have no copper shape",
            });
        }
        for layer in populated {
            let shape = padstack.get_shape(layer).expect("populated layer");
            let corners = shape.corner_approx_arr();
            if shape.is_empty()
                || !shape.is_bounded()
                || shape.dimension() != 2
                || corners.len() < 3
                || corners
                    .iter()
                    .any(|corner| !corner.x.is_finite() || !corner.y.is_finite())
            {
                return Err(KicadJsonWriteError::InvalidPadstack {
                    padstack: padstack_no,
                    reason: "it contains an empty, unbounded, or degenerate shape",
                });
            }
        }
    }

    let mut via_info_names = HashSet::new();
    for info_id in 0..board.rules.via_infos.count() {
        let info = board.rules.via_infos.get(info_id);
        let owner = format!("viaInfo {:?}", info.get_name());
        if info.get_name().is_empty() || !via_info_names.insert(info.get_name().to_string()) {
            return Err(KicadJsonWriteError::UnrepresentableRule {
                owner,
                reason: "its name is empty or duplicates another viaInfo",
            });
        }
        if board.padstacks.get_by_no(info.get_padstack()).is_none() {
            return Err(KicadJsonWriteError::DanglingRuleReference {
                owner,
                kind: "padstack",
                number: info.get_padstack(),
            });
        }
        if info.get_clearance_class() >= clearance_count {
            return Err(KicadJsonWriteError::DanglingRuleReference {
                owner,
                kind: "clearance class",
                number: info.get_clearance_class(),
            });
        }
    }

    let mut via_rule_names = HashSet::new();
    for (rule_id, rule) in board.rules.via_rules.iter().enumerate() {
        let owner = format!("viaRule {:?}", rule.name);
        if rule.name.is_empty() || !via_rule_names.insert(rule.name.clone()) {
            return Err(KicadJsonWriteError::UnrepresentableRule {
                owner,
                reason: "its name is empty or duplicates another viaRule",
            });
        }
        for &info_id in rule.vias() {
            if info_id >= board.rules.via_infos.count() {
                return Err(KicadJsonWriteError::DanglingRuleReference {
                    owner: format!("viaRule {:?} (index {rule_id})", rule.name),
                    kind: "viaInfo",
                    number: info_id,
                });
            }
        }
    }

    let class_count = board.rules.net_classes.count();
    if class_count == 0 {
        return Err(KicadJsonWriteError::InvalidBoard {
            reason: "there are no net classes",
        });
    }
    let mut class_names = HashSet::new();
    for class_id in 0..class_count {
        let class = board.rules.net_classes.get(class_id);
        let owner = format!("net class {:?}", class.get_name());
        if class.get_name().is_empty() || !class_names.insert(class.get_name().to_ascii_lowercase())
        {
            return Err(KicadJsonWriteError::UnrepresentableRule {
                owner,
                reason: "its name is empty or duplicates another net class",
            });
        }
        if class.layer_count() != layer_count {
            return Err(KicadJsonWriteError::UnrepresentableRule {
                owner,
                reason: "its layer count does not match the board",
            });
        }
        let trace_class = class.get_trace_clearance_class();
        if trace_class >= clearance_count {
            return Err(KicadJsonWriteError::DanglingRuleReference {
                owner,
                kind: "trace clearance class",
                number: trace_class,
            });
        }
        for item_class in [
            ItemClass::Trace,
            ItemClass::Via,
            ItemClass::Pin,
            ItemClass::Smd,
            ItemClass::Area,
        ] {
            let clearance = class.default_item_clearance_classes.get(item_class);
            if clearance >= clearance_count {
                return Err(KicadJsonWriteError::DanglingRuleReference {
                    owner: owner.clone(),
                    kind: "item clearance class",
                    number: clearance,
                });
            }
        }
        if let Some(rule_id) = class.get_via_rule() {
            if rule_id >= board.rules.via_rules.len() {
                return Err(KicadJsonWriteError::DanglingRuleReference {
                    owner: owner.clone(),
                    kind: "viaRule",
                    number: rule_id,
                });
            }
        }
        if class.get_trace_half_width(0) <= 0 {
            return Err(KicadJsonWriteError::UnrepresentableRule {
                owner: owner.clone(),
                reason: "its trace width must be positive",
            });
        }
    }

    let mut net_names = HashSet::new();
    for net in board.rules.nets.iter() {
        if net.name.is_empty() || !net_names.insert(net.name.to_ascii_lowercase()) {
            return Err(KicadJsonWriteError::UnrepresentableRule {
                owner: format!("net {}", net.net_number),
                reason: "net names must be non-empty and unique in KiCad JSON",
            });
        }
        if net.subnet_number != 1 {
            return Err(KicadJsonWriteError::UnrepresentableRule {
                owner: format!("net {:?}", net.name),
                reason: "Specctra subnet identity has no KiCad JSON representation",
            });
        }
        if net.get_class() >= class_count {
            return Err(KicadJsonWriteError::DanglingRuleReference {
                owner: format!("net {:?}", net.name),
                kind: "net class",
                number: net.get_class(),
            });
        }
    }

    if let Some((corners, _)) = &board.outline {
        if corners.len() < 3 {
            return Err(KicadJsonWriteError::InvalidBoard {
                reason: "the preserved outline has fewer than three corners",
            });
        }
    }

    for (&item_id, item) in board.items() {
        if item.base.component_no < 0 {
            return Err(KicadJsonWriteError::UnrepresentableItem {
                item_id,
                reason: "component numbers must not be negative",
            });
        }
        if item.base.clearance_class >= clearance_count {
            return Err(KicadJsonWriteError::DanglingItemReference {
                item_id,
                kind: "clearance class",
                number: item.base.clearance_class as i64,
            });
        }
        if item.base.net_nos.len() > 1 {
            return Err(KicadJsonWriteError::UnrepresentableItem {
                item_id,
                reason: "an item belongs to multiple nets or Specctra subnets",
            });
        }
        for &net_no in &item.base.net_nos {
            if board.rules.nets.get_by_no(net_no).is_none() {
                return Err(KicadJsonWriteError::DanglingItemReference {
                    item_id,
                    kind: "net",
                    number: i64::from(net_no),
                });
            }
        }
        match &item.kind {
            ItemKind::Via(via) => {
                let Some(padstack) = board.padstacks.get_by_no(via.padstack) else {
                    return Err(KicadJsonWriteError::DanglingItemReference {
                        item_id,
                        kind: "padstack",
                        number: via.padstack as i64,
                    });
                };
                if via.is_escape_via != via.escape_smd_layer.is_some() {
                    return Err(KicadJsonWriteError::UnrepresentableItem {
                        item_id,
                        reason: "escape-via marker and SMD layer disagree",
                    });
                }
                if let Some(layer) = via.escape_smd_layer {
                    if layer >= layer_count || padstack.get_shape(layer).is_none() {
                        return Err(KicadJsonWriteError::DanglingItemReference {
                            item_id,
                            kind: "escape SMD layer",
                            number: layer as i64,
                        });
                    }
                }
            }
            ItemKind::PolylineTrace(trace) => {
                if item.base.component_no != 0 {
                    return Err(KicadJsonWriteError::UnrepresentableItem {
                        item_id,
                        reason: "component-owned traces are not supported",
                    });
                }
                if trace.layer >= layer_count {
                    return Err(KicadJsonWriteError::DanglingItemReference {
                        item_id,
                        kind: "layer",
                        number: trace.layer as i64,
                    });
                }
                let corners = trace.polyline.corner_approx_arr();
                if trace.half_width <= 0
                    || corners.len() < 2
                    || corners
                        .iter()
                        .any(|corner| !corner.x.is_finite() || !corner.y.is_finite())
                {
                    return Err(KicadJsonWriteError::UnrepresentableItem {
                        item_id,
                        reason: "the trace has invalid width or geometry",
                    });
                }
            }
            ItemKind::ObstacleArea(area) => {
                if item.base.component_no != 0 {
                    return Err(KicadJsonWriteError::UnrepresentableItem {
                        item_id,
                        reason: "component-owned areas are not supported",
                    });
                }
                if area.layer >= layer_count {
                    return Err(KicadJsonWriteError::DanglingItemReference {
                        item_id,
                        kind: "layer",
                        number: area.layer as i64,
                    });
                }
                if !area.area.holes.is_empty() {
                    return Err(KicadJsonWriteError::UnrepresentableItem {
                        item_id,
                        reason: "polygon holes require a schema extension",
                    });
                }
                let corners = area.area.border_shape.corners.len();
                if area.area.is_empty()
                    || !area.area.is_bounded()
                    || area.area.dimension() != 2
                    || corners < 3
                {
                    return Err(KicadJsonWriteError::UnrepresentableItem {
                        item_id,
                        reason: "the area polygon is empty, unbounded, or degenerate",
                    });
                }
                if area.is_conduction == item.base.net_nos.is_empty() {
                    return Err(KicadJsonWriteError::UnrepresentableItem {
                        item_id,
                        reason: "KiCad JSON derives conduction-area identity from net membership",
                    });
                }
                if area.is_conduction && area.via_only {
                    return Err(KicadJsonWriteError::UnrepresentableItem {
                        item_id,
                        reason: "a conduction area cannot also be a via-only keepout",
                    });
                }
                if !area.is_conduction && area.name == "boundary" && board.outline.is_none() {
                    return Err(KicadJsonWriteError::UnrepresentableItem {
                        item_id,
                        reason: "a boundary strip requires preserved outline geometry",
                    });
                }
                if !area.is_conduction
                    && area.name == "boundary"
                    && item.base.fixed_state != FixedState::SystemFixed
                {
                    return Err(KicadJsonWriteError::UnrepresentableItem {
                        item_id,
                        reason: "outline boundary strips are always system fixed in KiCad JSON",
                    });
                }
            }
        }
    }
    crate::board::validate_board_references(board).map_err(|error| {
        KicadJsonWriteError::InvalidReferenceGraph {
            path: error.path,
            message: error.message,
        }
    })
}

/// Serializes a validated board as KiCad board JSON.
///
/// KiCad's interchange contract is canonical millimetres; source DSN units
/// are converted through the board's unit-aware physical scale before
/// writing. A fixed 0.1um grid keeps the reader/writer pair deterministic even
/// when a source unit has a fractional number of board units per millimetre
/// (for example one mil at resolution 1).
pub fn export_kicad_json_checked(board: &BasicBoard) -> Result<String, KicadJsonWriteError> {
    validate_board_for_kicad_json(board)?;
    Ok(export_kicad_json_unchecked(board))
}

/// Compatibility wrapper for callers that cannot return a serialization
/// error yet. Invalid boards panic instead of silently producing corrupt JSON;
/// production entry points use [`export_kicad_json_checked`] directly.
pub fn export_kicad_json(board: &BasicBoard) -> String {
    export_kicad_json_checked(board).expect("board is not representable as KiCad JSON")
}

fn export_kicad_json_unchecked(board: &BasicBoard) -> String {
    let units_per_mm = board.board_units_per_mm();
    let mm = |v: f64| v / units_per_mm;
    // The reader treats an MM document with resolution 1 as its default
    // 10000 units/mm.  Emit that explicit resolution so the physical scale
    // does not depend on the source board's unit token.
    let mm_to_document = |v: f64| v;
    let mut out = String::from("{\n  \"unit\": \"MM\",\n");
    out.push_str("  \"resolution\": 10000,\n");
    // BoardRules are not part of KiCad's public JSON schema.  Keep the
    // routing switches in a versioned extension so a checked round trip does
    // not silently reset the autorouter's behavior.
    let mut same_net_clearances: Vec<(ItemClass, ItemClass, i32)> = board
        .rules
        .same_net_clearances()
        .filter(|(first, second, _)| first <= second)
        .collect();
    same_net_clearances.sort_by_key(|(first, second, _)| (*first, *second));
    let same_net_json = same_net_clearances
        .iter()
        .map(|(first, second, value)| {
            format!(
                "{{\"first\": \"{}\", \"second\": \"{}\", \"clearance\": {:.6}}}",
                item_class_token(*first),
                item_class_token(*second),
                mm(*value as f64)
            )
        })
        .collect::<Vec<_>>()
        .join(", ");
    out.push_str(&format!(
        "  \"freeroutingRules\": {{\"version\": 1, \"traceAngleRestriction\": \"{}\", \"ignoreConduction\": {}, \"pinEdgeToTurnDistance\": {:.6}, \"useSlowAutorouteAlgorithm\": {}, \"viaAtSmdAllowed\": {}, \"sameNetClearances\": [{}]}},\n",
        angle_restriction_token(board.rules.get_trace_angle_restriction()),
        board.rules.get_ignore_conduction(),
        mm(board.rules.get_pin_edge_to_turn_dist()),
        board.rules.get_use_slow_autoroute_algorithm(),
        board.rules.via_at_smd_allowed,
        same_net_json,
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
                .unwrap_or_else(|| mm_to_document(0.8));
            let via_rule_name = class
                .get_via_rule()
                .and_then(|rule_id| board.rules.via_rules.get(rule_id))
                .map(|rule| format!("\"{}\"", esc(&rule.name)))
                .unwrap_or_else(|| "null".to_string());
            // KiCad's public class schema is scalar. This optional extension
            // preserves the remaining Freerouting semantics while leaving the
            // scalar fields above usable by older readers.
            let item_clearances = [
                (ItemClass::None, "none"),
                (ItemClass::Trace, "trace"),
                (ItemClass::Via, "via"),
                (ItemClass::Pin, "pin"),
                (ItemClass::Smd, "smd"),
                (ItemClass::Area, "area"),
            ]
            .into_iter()
            .map(|(item_class, field)| {
                let clearance = class.default_item_clearance_classes.get(item_class);
                let name = matrix.get_name(clearance).unwrap_or("null");
                format!("\"{field}\": \"{}\"", esc(name))
            })
            .collect::<Vec<_>>()
            .join(", ");
            let trace_half_widths = (0..layer_count)
                .map(|layer| mm(class.get_trace_half_width(layer) as f64).to_string())
                .collect::<Vec<_>>()
                .join(", ");
            let active_routing_layers = (0..layer_count)
                .map(|layer| class.is_active_routing_layer(layer).to_string())
                .collect::<Vec<_>>()
                .join(", ");
            let trace_clearance_name = matrix.get_name(tcc).unwrap_or("null");
            let routing_metadata = format!(
                "{{\"version\": 1, \"traceClearanceClass\": \"{}\", \"itemClearanceClasses\": {{{item_clearances}}}, \"traceHalfWidths\": [{trace_half_widths}], \"activeRoutingLayers\": [{active_routing_layers}], \"ignoredByAutorouter\": {}, \"shoveFixed\": {}, \"pullTight\": {}, \"ignoreCyclesWithAreas\": {}, \"minimumTraceLength\": {}, \"maximumTraceLength\": {}}}",
                esc(trace_clearance_name),
                class.is_ignored_by_autorouter,
                class.is_shove_fixed(),
                class.get_pull_tight(),
                class.get_ignore_cycles_with_areas(),
                mm(class.get_minimum_trace_length()),
                mm(class.get_maximum_trace_length()),
            );
            format!(
                "{{\"name\": \"{}\", \"clearance\": {:.6}, \"traceWidth\": {:.6}, \"viaDiameter\": {:.6}, \"viaDrill\": {:.6}, \"netNames\": [{}], \"viaRuleName\": {}, \"routingMetadata\": {}}}",
                esc(class.get_name()),
                mm(cl),
                mm(2.0 * hw),
                via_diameter,
                via_diameter * 0.5,
                names.join(", "),
                via_rule_name,
                routing_metadata,
            )
        })
        .collect();
    out.push_str(&format!("  \"netClasses\": [{}],\n", class_json.join(", ")));

    // KiCad's public JSON has scalar via dimensions, but those do not carry
    // the ordered ViaInfo list that Freerouting uses for rule selection.  The
    // explicit extension below preserves every named via info (including one
    // that has not been used by a routed via yet) and its resolved padstack.
    // Legacy consumers ignore these fields; the Rust reader uses them when
    // present and falls back to the scalar/default path otherwise.
    let via_info_json: Vec<String> = (0..board.rules.via_infos.count())
        .map(|info_id| {
            let info = board.rules.via_infos.get(info_id);
            let Some(padstack) = board.padstacks.get_by_no(info.get_padstack()) else {
                return format!(
                    "{{\"name\": \"{}\", \"padstackName\": \"\", \"startLayerIndex\": 0, \"endLayerIndex\": 0, \"diameter\": 0, \"layerShapes\": [], \"clearanceClass\": \"{}\", \"clearanceValue\": 0, \"attachAllowed\": {}}}",
                    esc(info.get_name()),
                    esc(matrix.get_name(info.get_clearance_class()).unwrap_or("default")),
                    info.attach_smd_allowed(),
                );
            };
            let layer_shapes = (padstack.from_layer()..=padstack.to_layer())
                .filter_map(|layer| {
                    let shape = padstack.get_shape(layer)?;
                    let corners = shape
                        .corner_approx_arr()
                        .into_iter()
                        .map(|corner| {
                            format!(
                                "{{\"x\": {:.6}, \"y\": {:.6}}}",
                                mm(corner.x),
                                mm(-corner.y)
                            )
                        })
                        .collect::<Vec<_>>()
                        .join(", ");
                    Some(format!(
                        "{{\"layerIndex\": {layer}, \"corners\": [{corners}]}}"
                    ))
                })
                .collect::<Vec<_>>()
                .join(", ");
            let diameter = padstack
                .get_shape(padstack.from_layer())
                .map(|shape| mm(shape.bounding_box().max_width()))
                .unwrap_or(0.0);
            let clearance = matrix.get_value(
                info.get_clearance_class(),
                1,
                padstack.from_layer(),
                false,
            );
            format!(
                "{{\"name\": \"{}\", \"padstackName\": \"{}\", \"startLayerIndex\": {}, \"endLayerIndex\": {}, \"diameter\": {:.6}, \"layerShapes\": [{}], \"clearanceClass\": \"{}\", \"clearanceValue\": {:.6}, \"attachAllowed\": {}}}",
                esc(info.get_name()),
                esc(&padstack.name),
                padstack.from_layer(),
                padstack.to_layer(),
                diameter,
                layer_shapes,
                esc(matrix.get_name(info.get_clearance_class()).unwrap_or("default")),
                mm(clearance as f64),
                info.attach_smd_allowed(),
            )
        })
        .collect();
    out.push_str(&format!(
        "  \"viaInfos\": [{}],\n",
        via_info_json.join(", ")
    ));
    let via_rule_json: Vec<String> = board
        .rules
        .via_rules
        .iter()
        .map(|rule| {
            let names = rule
                .vias()
                .iter()
                .filter_map(|&id| {
                    (id < board.rules.via_infos.count())
                        .then_some(board.rules.via_infos.get(id).get_name())
                })
                .map(|name| format!("\"{}\"", esc(name)))
                .collect::<Vec<_>>();
            format!(
                "{{\"name\": \"{}\", \"viaInfoNames\": [{}]}}",
                esc(&rule.name),
                names.join(", ")
            )
        })
        .collect();
    out.push_str(&format!(
        "  \"viaRules\": [{}],\n",
        via_rule_json.join(", ")
    ));
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
    // Full matrix rows are a Rust-reader extension. Scalar net-class fields
    // remain for KiCad compatibility; this section preserves matrix-only
    // classes and per-layer/asymmetric values exactly.
    let matrix_rows: Vec<String> = (0..matrix.get_class_count())
        .map(|row| {
            let layers: Vec<String> = (0..matrix.get_layer_count())
                .map(|layer| {
                    let values: Vec<String> = (0..matrix.get_class_count())
                        .map(|column| {
                            format!(
                                "{:.6}",
                                mm(matrix.get_value(row, column, layer, false) as f64)
                            )
                        })
                        .collect();
                    format!("[{}]", values.join(", "))
                })
                .collect();
            format!(
                "{{\"name\": \"{}\", \"layers\": [{}]}}",
                esc(matrix.get_name(row).unwrap_or("null")),
                layers.join(", ")
            )
        })
        .collect();
    out.push_str(&format!(
        "  \"clearanceMatrix\": [{}],\n",
        matrix_rows.join(", ")
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
        // `layers` conveys the declared span for compatibility with the
        // public KiCad shape schema.  The optional layerShapes extension
        // carries the actual copper profile so a sparse padstack does not
        // acquire copper on otherwise empty intermediate layers on reload.
        let layer_shapes_json = board
            .padstacks
            .get_by_no(v.padstack)
            .map(|p| {
                (p.from_layer()..=p.to_layer())
                    .filter_map(|layer| {
                        let shape = p.get_shape(layer)?;
                        let corners = shape
                            .corner_approx_arr()
                            .into_iter()
                            .map(|corner| {
                                format!(
                                    "{{\"x\": {:.6}, \"y\": {:.6}}}",
                                    mm(corner.x),
                                    mm(-corner.y)
                                )
                            })
                            .collect::<Vec<_>>()
                            .join(", ");
                        Some(format!(
                            "{{\"layerIndex\": {layer}, \"corners\": [{corners}]}}"
                        ))
                    })
                    .collect::<Vec<_>>()
                    .join(", ")
            })
            .unwrap_or_default();
        comp_pads.entry(item.base.component_no).or_default().push(format!(
            "{{\"name\": \"\", \"netName\": \"{}\", \"shape\": \"rect\", \"size\": {{\"x\": {:.6}, \"y\": {:.6}}}, \"offset\": {{\"x\": {:.6}, \"y\": {:.6}}}, \"drill\": {}, \"layers\": [{}], \"layerShapes\": [{}], \"clearanceClass\": \"{}\", \"clearanceValue\": {:.6}, \"clearanceExplicit\": {}, \"fixedState\": \"{}\"}}",
            esc(&net_name),
            mm(w),
            mm(h),
            mm(v.center.x as f64),
            mm(-v.center.y as f64),
            if through { 1 } else { 0 },
            layers_json,
            layer_shapes_json,
            esc(matrix.get_name(item.base.clearance_class).unwrap_or("default")),
            mm(
                matrix
                    .get_value(
                        item.base.clearance_class,
                        1,
                        item.first_layer(&board.padstacks),
                        false,
                    )
                    as f64
            ),
            item.base.clearance_class_explicit,
            fixed_state_token(item.base.fixed_state),
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
    // outline: the PRESERVED source outline when the import carried one
    // (a DSN boundary or a KiCad outline object) — the exact polygon and
    // its clearance survive the round trip. Only a board without one
    // falls back to the all-item bounding box at the default clearance.
    let default_clearance = matrix.get_value(1, 1, 0, false) as f64;
    let (outline_points, outline_clearance): (Vec<(f64, f64)>, f64) = match &board.outline {
        Some((corners, cl)) => (
            corners.iter().map(|p| (p.x as f64, p.y as f64)).collect(),
            *cl as f64,
        ),
        None => {
            let bb = board.bounding_box();
            if bb.is_empty() {
                (Vec::new(), default_clearance)
            } else {
                (
                    vec![
                        (bb.ll.x as f64, bb.ll.y as f64),
                        (bb.ur.x as f64, bb.ll.y as f64),
                        (bb.ur.x as f64, bb.ur.y as f64),
                        (bb.ll.x as f64, bb.ur.y as f64),
                    ],
                    default_clearance,
                )
            }
        }
    };
    let outline_corners = outline_points
        .iter()
        .map(|(x, y)| format!("{{\"x\": {:.6}, \"y\": {:.6}}}", mm(*x), mm(-*y)))
        .collect::<Vec<_>>()
        .join(", ");
    out.push_str(&format!(
        "  \"outline\": {{\"corners\": [{outline_corners}], \"clearance\": {:.6}}},\n",
        mm(outline_clearance)
    ));
    // routed traces and vias
    let mut traces = Vec::new();
    let mut vias = Vec::new();
    for (id, item) in board.items() {
        if item.base.component_no != 0 {
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
                    "    {{\"id\": {id}, \"netName\": \"{}\", \"width\": {:.6}, \"layerIndex\": {}, \"clearanceClass\": \"{}\", \"clearanceValue\": {:.6}, \"clearanceExplicit\": {}, \"fixedState\": \"{}\", \"points\": [{}]}}",
                    esc(&net_name),
                    mm(2.0 * t.half_width as f64),
                    t.layer,
                    esc(matrix.get_name(item.base.clearance_class).unwrap_or("default")),
                    mm(matrix.get_value(item.base.clearance_class, 1, t.layer, false) as f64),
                    item.base.clearance_class_explicit,
                    fixed_state_token(item.base.fixed_state),
                    pts.join(", ")
                ));
            }
            ItemKind::Via(v) => {
                // the via's real layer span and pad diameter, not a
                // hardcoded full-stack 0.6 mm via (blind/buried vias and
                // real sizes must survive the round trip)
                let (from, to, dia, layer_shapes) = board
                    .padstacks
                    .get_by_no(v.padstack)
                    .map(|p| {
                        let shapes = (p.from_layer()..=p.to_layer())
                            .filter_map(|layer| {
                                let shape = p.get_shape(layer)?;
                                let corners = shape
                                    .corner_approx_arr()
                                    .into_iter()
                                    .map(|corner| {
                                        format!(
                                            "{{\"x\": {:.6}, \"y\": {:.6}}}",
                                            mm(corner.x),
                                            mm(-corner.y)
                                        )
                                    })
                                    .collect::<Vec<_>>()
                                    .join(", ");
                                Some(format!(
                                    "{{\"layerIndex\": {layer}, \"corners\": [{corners}]}}"
                                ))
                            })
                            .collect::<Vec<_>>()
                            .join(", ");
                        (
                            p.from_layer(),
                            p.to_layer(),
                            p.get_shape(p.from_layer())
                                .map(|s| s.bounding_box().max_width())
                                .unwrap_or(0.0),
                            shapes,
                        )
                    })
                    .unwrap_or((0, layer_count - 1, 0.0, String::new()));
                let net_trace_class = item
                    .base
                    .net_nos
                    .first()
                    .map(|&net| board.rules.get_trace_clearance_class(net));
                let via_info_name = (0..board.rules.via_infos.count())
                    .find(|&index| {
                        let info = board.rules.via_infos.get(index);
                        info.get_padstack() == v.padstack
                            && (info.get_clearance_class() == item.base.clearance_class
                                || (info.get_clearance_class() == 0
                                    && net_trace_class == Some(item.base.clearance_class)))
                            && info.attach_smd_allowed() == v.attach_allowed
                    })
                    .map(|index| board.rules.via_infos.get(index).get_name().to_string())
                    .unwrap_or_default();
                let via_rule_name = item
                    .base
                    .net_nos
                    .first()
                    .and_then(|&net| board.rules.nets.get_by_no(net))
                    .and_then(|net| board.rules.net_classes.get(net.get_class()).get_via_rule())
                    .and_then(|rule| board.rules.via_rules.get(rule))
                    .map(|rule| rule.name.clone())
                    .unwrap_or_default();
                vias.push(format!(
                    "    {{\"id\": {id}, \"netName\": \"{}\", \"position\": {{\"x\": {:.6}, \"y\": {:.6}}}, \"diameter\": {:.6}, \"drill\": {:.6}, \"startLayerIndex\": {from}, \"endLayerIndex\": {to}, \"layerShapes\": [{layer_shapes}], \"clearanceClass\": \"{}\", \"clearanceValue\": {:.6}, \"clearanceExplicit\": {}, \"viaInfoName\": \"{}\", \"viaRuleName\": \"{}\", \"attachAllowed\": {}, \"isEscapeVia\": {}, \"escapeSmdLayer\": {}, \"fixedState\": \"{}\"}}",
                    esc(&net_name),
                    mm(v.center.x as f64),
                    mm(-v.center.y as f64),
                    mm(dia),
                    mm(dia) / 2.0,
                    esc(matrix.get_name(item.base.clearance_class).unwrap_or("default")),
                    mm(matrix.get_value(item.base.clearance_class, 1, from, false) as f64),
                    item.base.clearance_class_explicit,
                    esc(&via_info_name),
                    esc(&via_rule_name),
                    v.attach_allowed,
                    v.is_escape_via,
                    v.escape_smd_layer
                        .map(|layer| layer.to_string())
                        .unwrap_or_else(|| "null".to_string()),
                    fixed_state_token(item.base.fixed_state),
                ));
            }
            _ => {}
        }
    }
    out.push_str(&format!("  \"traces\": [\n{}\n  ],\n", traces.join(",\n")));
    out.push_str(&format!("  \"vias\": [\n{}\n  ],\n", vias.join(",\n")));
    // conduction areas (copper pours) with their obstacle flag, AND
    // netless keepouts (physical routing constraints the round trip must
    // not drop — Issue027 carries 16 keepout scopes). The boundary strips
    // are excluded: they ARE the serialized outline.
    let mut zones = Vec::new();
    for (_, item) in board.items() {
        let ItemKind::ObstacleArea(a) = &item.kind else {
            continue;
        };
        if !a.is_conduction && a.name == "boundary" {
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
        // the zone's clearance class BY NAME plus its VALUE (mm): the
        // reader's matrix only contains net-class-derived classes, so a
        // matrix-only class (e.g. a boundary keepout's) must be
        // re-creatable from the zone itself or it reloads as default
        let cl_name = matrix
            .get_name(item.base.clearance_class)
            .unwrap_or("default");
        let cl_value = matrix
            .get_value(item.base.clearance_class, 1, a.layer, false)
            .max(0) as f64;
        zones.push(format!(
            "    {{\"name\": \"{}\", \"netName\": \"{}\", \"layerIndex\": {}, \"isObstacle\": {}, \"viaOnly\": {}, \"clearanceClass\": \"{}\", \"clearanceValue\": {:.6}, \"clearanceExplicit\": {}, \"fixedState\": \"{}\", \"polygon\": [{}]}}",
            esc(&a.name),
            esc(&net_name),
            a.layer,
            a.is_obstacle,
            a.via_only,
            esc(cl_name),
            mm(cl_value),
            item.base.clearance_class_explicit,
            fixed_state_token(item.base.fixed_state),
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
    use crate::rules::BoardRules;

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
    fn netless_route_copper_survives_round_trip() {
        let mut board = import_dsn(MINI_DSN).expect("import");
        board.insert_trace(
            crate::geometry::planar::Polyline::from_int_points(&[
                crate::geometry::planar::IntPoint::new(1_000, 1_000),
                crate::geometry::planar::IntPoint::new(9_000, 1_000),
            ]),
            0,
            100,
            Vec::new(),
            1,
        );
        board.insert_via(
            1,
            crate::geometry::planar::IntPoint::new(9_000, 1_000),
            Vec::new(),
            1,
            false,
        );
        let json = export_kicad_json(&board);
        let reloaded = import_kicad_json(&json).expect("re-import");
        assert_eq!(
            reloaded
                .items()
                .filter(|(_, item)| {
                    item.base.component_no == 0
                        && item.base.net_count() == 0
                        && matches!(item.kind, ItemKind::PolylineTrace(_) | ItemKind::Via(_))
                })
                .count(),
            2
        );
        assert_eq!(
            crate::drc::check_board(&reloaded).violations.len(),
            crate::drc::check_board(&board).violations.len()
        );
    }

    #[test]
    fn sparse_via_layer_shapes_survive_round_trip() {
        let dsn = r#"(pcb "sparse.dsn"
  (resolution um 10)
  (structure
    (layer F.Cu (type signal))
    (layer In1.Cu (type signal))
    (layer B.Cu (type signal))
    (boundary (rect pcb 0 0 100000 100000))
    (rule (width 200) (clearance 200))
  )
  (placement)
  (library
    (padstack "Sparse"
      (shape (circle F.Cu 600 0 0))
      (shape (circle B.Cu 600 0 0))
    )
  )
  (network (net "N"))
)"#;
        let mut board = import_dsn(dsn).expect("import");
        board.insert_via(
            1,
            crate::geometry::planar::IntPoint::new(20_000, 20_000),
            vec![1],
            1,
            false,
        );
        let json = export_kicad_json(&board);
        let reloaded = import_kicad_json(&json).expect("re-import");
        let padstack = reloaded
            .items()
            .find_map(|(_, item)| match &item.kind {
                ItemKind::Via(via) if item.base.component_no == 0 => {
                    reloaded.padstacks.get_by_no(via.padstack)
                }
                _ => None,
            })
            .expect("routed via padstack");
        assert!(padstack.get_shape(0).is_some());
        assert!(padstack.get_shape(1).is_none());
        assert!(padstack.get_shape(2).is_some());
    }

    #[test]
    fn unused_via_rule_members_and_order_survive_round_trip() {
        use crate::geometry::planar::{IntBox, TileShape};

        let mut board = import_dsn(MINI_DSN).expect("import");
        let blind_padstack = board.padstacks.add(
            "BlindOnly",
            vec![
                Some(TileShape::Box(IntBox::from_coords(-300, -300, 300, 300))),
                None,
            ],
            true,
            false,
        );
        let blind_info = board
            .rules
            .via_infos
            .add(crate::rules::ViaInfo::new(
                "blind_candidate",
                blind_padstack,
                1,
                false,
            ))
            .expect("blind ViaInfo");
        let through_info = board
            .rules
            .via_infos
            .add(crate::rules::ViaInfo::new("through_candidate", 1, 1, false))
            .expect("through ViaInfo");
        let mut ordered = crate::rules::ViaRule::new("ordered_candidates");
        // There is deliberately no routed via using blind_candidate. The
        // rule must nevertheless retain it as the first future-routing
        // choice after JSON serialization.
        ordered.append_via(blind_info);
        ordered.append_via(through_info);
        board.rules.via_rules.push(ordered);
        let class = board.rules.append_net_class("ordered_class");
        let net = board.rules.nets.add("ORDERED", 1, false);
        board
            .rules
            .nets
            .get_by_no_mut(net)
            .expect("net")
            .set_class(class);
        board
            .rules
            .net_classes
            .get_mut(class)
            .set_via_rule(Some(board.rules.via_rules.len() - 1));

        let json = export_kicad_json(&board);
        assert!(json.contains("\"viaInfos\""));
        assert!(json.contains("\"viaRules\""));
        let reloaded = import_kicad_json(&json).expect("re-import");
        let class2 = reloaded
            .rules
            .net_classes
            .get_by_name("ordered_class")
            .expect("ordered class");
        let rule_id = reloaded
            .rules
            .net_classes
            .get(class2)
            .get_via_rule()
            .expect("ordered rule binding");
        let rule = &reloaded.rules.via_rules[rule_id];
        let names: Vec<&str> = rule
            .vias()
            .iter()
            .map(|&id| reloaded.rules.via_infos.get(id).get_name())
            .collect();
        assert_eq!(names, ["blind_candidate", "through_candidate"]);
        let blind_id = reloaded
            .rules
            .via_infos
            .get_by_name("blind_candidate")
            .expect("unused blind info");
        let blind_ps = reloaded.rules.via_infos.get(blind_id).get_padstack();
        assert_eq!(
            reloaded.padstacks.get_by_no(blind_ps).unwrap().to_layer(),
            0,
            "the unused blind padstack must remain blind"
        );
    }

    #[test]
    fn sparse_component_pad_layer_shapes_survive_round_trip() {
        let dsn = r#"(pcb "sparse-pad.dsn"
  (resolution um 10)
  (structure
    (layer F.Cu (type signal))
    (layer In1.Cu (type signal))
    (layer B.Cu (type signal))
    (boundary (rect pcb 0 0 100000 100000))
    (rule (width 200) (clearance 200))
  )
  (placement)
  (library
    (padstack "Sparse"
      (shape (circle F.Cu 600 0 0))
      (shape (circle B.Cu 600 0 0))
    )
  )
  (network (net "N"))
)"#;
        let mut board = import_dsn(dsn).expect("import");
        let pad = board.insert_via(
            1,
            crate::geometry::planar::IntPoint::new(20_000, 20_000),
            vec![1],
            1,
            false,
        );
        board.set_component_no(pad, 1);
        let json = export_kicad_json(&board);
        assert!(json.contains("\"layerShapes\": [{\"layerIndex\": 0"));
        assert!(!json.contains("\"layerIndex\": 1, \"corners\""));
        let reloaded = import_kicad_json(&json).expect("re-import");
        let padstack = reloaded
            .items()
            .find_map(|(_, item)| match &item.kind {
                ItemKind::Via(via) if item.base.component_no != 0 => {
                    reloaded.padstacks.get_by_no(via.padstack)
                }
                _ => None,
            })
            .expect("component padstack");
        assert!(padstack.get_shape(0).is_some());
        assert!(padstack.get_shape(1).is_none());
        assert!(padstack.get_shape(2).is_some());
    }

    #[test]
    fn dsn_placements_keep_component_groups_in_json_round_trip() {
        let dsn = r#"(pcb "components.dsn"
  (resolution um 10)
  (structure
    (layer F.Cu (type signal))
    (boundary (rect pcb 0 0 100000 100000))
    (rule (width 200) (clearance 200))
  )
  (placement
    (component "SMD"
      (place C1 20000 20000 front 0)
      (place C2 70000 70000 front 0))
  )
  (library
    (image "SMD" (pin "Pad" 1 0 0))
    (padstack "Pad" (shape (rect F.Cu -500 -500 500 500)) (attach off))
  )
  (network
    (net "N1" (pins C1-1))
    (net "N2" (pins C2-1))
  )
)"#;
        let board = import_dsn(dsn).expect("import");
        let json = export_kicad_json(&board);
        assert_eq!(json.matches("\"reference\"").count(), 2);
        let reloaded = import_kicad_json(&json).expect("re-import");
        let component_ids: std::collections::BTreeSet<i32> = reloaded
            .items()
            .filter(|(_, item)| item.base.component_no != 0)
            .map(|(_, item)| item.base.component_no)
            .collect();
        assert_eq!(component_ids.len(), 2, "pads must retain their two groups");
    }

    #[test]
    fn through_pads_vias_and_pours_survive_round_trip() {
        use crate::geometry::planar::{IntBox, IntPoint, PolygonShape, PolylineArea, TileShape};
        let mut board = import_dsn(MINI_DSN).expect("import");
        // a through pad (the Via padstack spans both layers)
        let pad = board.insert_via(1, IntPoint::new(20000, 20000), vec![1], 1, false);
        board.set_component_no(pad, 1);
        // a routed via
        board.insert_via(1, IntPoint::new(40000, 40000), vec![1], 1, false);
        // a pure-SMD escape via whose layer-scoped exception must not collapse
        // into either unrestricted attach or an ordinary violating via
        let smd_ps = board.padstacks.add(
            "SmdTop",
            vec![
                Some(TileShape::Box(IntBox::from_coords(
                    -1000, -1000, 1000, 1000,
                ))),
                None,
            ],
            false,
            false,
        );
        let smd = board.insert_via(smd_ps, IntPoint::new(60000, 60000), vec![1], 1, false);
        board.set_component_no(smd, 2);
        board.insert_escape_via(1, IntPoint::new(60000, 60000), vec![1], 1, false, 0);
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
        let escape2 = board2
            .items()
            .find_map(|(_, item)| match &item.kind {
                ItemKind::Via(via)
                    if item.base.component_no == 0 && via.center == IntPoint::new(60000, 60000) =>
                {
                    Some(via)
                }
                _ => None,
            })
            .expect("escape via");
        assert!(!escape2.attach_allowed);
        assert!(escape2.is_escape_via);
        assert_eq!(escape2.escape_smd_layer, Some(0));
        assert!(crate::drc::check_board(&board2).violations.is_empty());
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

    #[test]
    fn source_outline_geometry_and_clearance_survive_round_trip() {
        // Issue027: the DSN boundary polygon must be exported verbatim, not
        // fabricated from the whole-board bounding box (which pours expand);
        // Issue649: the KiCad outline clearance (0.5 mm) must survive reload.
        let root = concat!(env!("CARGO_MANIFEST_DIR"), "/..");
        let dsn = std::fs::read_to_string(format!("{root}/fixtures/Issue027-zMRETestFixture.dsn"))
            .expect("fixture missing from checkout");
        let board = import_dsn(&dsn).expect("import");
        let (outline, _) = board.outline.clone().expect("DSN boundary preserved");
        let json = export_kicad_json(&board);
        let board2 = import_kicad_json(&json).expect("re-import");
        let (outline2, _) = board2.outline.clone().expect("outline consumed");
        // same corner count and same extents (units differ only by the
        // mm round trip: compare in mm with a loose epsilon)
        assert_eq!(outline.len(), outline2.len(), "outline polygon preserved");
        let extent = |pts: &[crate::geometry::planar::IntPoint], per_mm: f64| {
            let xs: Vec<f64> = pts.iter().map(|p| p.x as f64 / per_mm).collect();
            let ys: Vec<f64> = pts.iter().map(|p| p.y as f64 / per_mm).collect();
            (
                xs.iter().cloned().fold(f64::MAX, f64::min),
                xs.iter().cloned().fold(f64::MIN, f64::max),
                ys.iter().cloned().fold(f64::MAX, f64::min),
                ys.iter().cloned().fold(f64::MIN, f64::max),
            )
        };
        let a = extent(&outline, board.board_units_per_mm());
        let b = extent(&outline2, board2.board_units_per_mm());
        assert!(
            (a.0 - b.0).abs() < 0.01
                && (a.1 - b.1).abs() < 0.01
                && (a.2 - b.2).abs() < 0.01
                && (a.3 - b.3).abs() < 0.01,
            "outline extents must survive the round trip: {a:?} vs {b:?}"
        );

        let kicad_json = std::fs::read_to_string(format!(
            "{root}/fixtures/Issue649-kicad_ecc83-pp_input_board_v1.json"
        ))
        .expect("fixture missing from checkout");
        let board3 = import_kicad_json(&kicad_json).expect("import");
        let (_, cl) = board3.outline.clone().expect("outline preserved");
        let cl_mm = cl as f64 / board3.board_units_per_mm();
        let json3 = export_kicad_json(&board3);
        let board4 = import_kicad_json(&json3).expect("re-import");
        let (_, cl4) = board4.outline.clone().expect("outline preserved");
        let cl4_mm = cl4 as f64 / board4.board_units_per_mm();
        assert!(
            (cl_mm - cl4_mm).abs() < 1e-6,
            "the outline clearance must survive the round trip ({cl_mm} vs {cl4_mm})"
        );
    }

    #[test]
    fn keepouts_and_outline_clearance_survive_round_trip() {
        use crate::geometry::planar::{IntPoint, PolygonShape, PolylineArea};
        let mut board = import_dsn(MINI_DSN).expect("import");
        // widen the default clearance so the outline clearance is distinctive
        board
            .rules
            .clearance_matrix
            .set_value_on_all_layers(1, 1, 5000); // 0.5 mm at 10 units/um
        let keepout_area = |x0: i32| {
            PolylineArea::new(
                PolygonShape::from_int_points(&[
                    IntPoint::new(x0, 0),
                    IntPoint::new(x0 + 8000, 0),
                    IntPoint::new(x0 + 8000, 8000),
                    IntPoint::new(x0, 8000),
                ]),
                Vec::new(),
            )
        };
        // a netless keepout and a via-only keepout (physical constraints
        // the round trip previously dropped entirely)
        let ko = board.insert_area(keepout_area(10000), 0, "ko", Vec::new(), 1, false);
        let vko = board.insert_area(keepout_area(30000), 0, "vko", Vec::new(), 1, false);
        board.set_area_via_only(vko, true);
        let _ = ko;

        let json = export_kicad_json(&board);
        assert!(
            json.contains("\"clearance\": 0.500000"),
            "the outline carries the real default clearance, not 0.2"
        );
        let board2 = import_kicad_json(&json).expect("re-import");
        let keepouts: Vec<_> = board2
            .items()
            .filter_map(|(_, it)| match &it.kind {
                ItemKind::ObstacleArea(a) if !a.is_conduction && a.name != "boundary" => {
                    Some(a.via_only)
                }
                _ => None,
            })
            .collect();
        assert_eq!(
            keepouts.len(),
            2,
            "both keepouts must survive the round trip"
        );
        assert_eq!(
            keepouts.iter().filter(|v| **v).count(),
            1,
            "the via-only flag must survive"
        );
    }

    #[test]
    fn custom_zone_clearance_class_survives_round_trip() {
        // a keepout on a MATRIX-ONLY class (not among the net classes):
        // the reader used to fail the name lookup and fall back to default
        use crate::geometry::planar::{IntPoint, PolygonShape, PolylineArea};
        let mut board = import_dsn(MINI_DSN).expect("import");
        board.rules.clearance_matrix.append_class("strict_ko");
        let strict = board.rules.clearance_matrix.get_no("strict_ko").unwrap();
        let n = board.rules.clearance_matrix.get_class_count();
        for j in 1..n {
            board
                .rules
                .clearance_matrix
                .set_value_on_all_layers(strict, j, 7000);
            board
                .rules
                .clearance_matrix
                .set_value_on_all_layers(j, strict, 7000);
        }
        let area = PolylineArea::new(
            PolygonShape::from_int_points(&[
                IntPoint::new(10000, 10000),
                IntPoint::new(20000, 10000),
                IntPoint::new(20000, 20000),
                IntPoint::new(10000, 20000),
            ]),
            Vec::new(),
        );
        board.insert_area(area, 0, "ko", Vec::new(), strict, false);
        let json = export_kicad_json(&board);
        let board2 = import_kicad_json(&json).expect("re-import");
        let ko2_cl = board2
            .items()
            .find_map(|(_, it)| match &it.kind {
                ItemKind::ObstacleArea(a) if !a.is_conduction && a.name != "boundary" => {
                    Some(it.base.clearance_class)
                }
                _ => None,
            })
            .expect("keepout reloaded");
        let strict2 = board2
            .rules
            .clearance_matrix
            .get_no("strict_ko")
            .expect("the matrix-only class must be recreated on reload");
        assert_eq!(ko2_cl, strict2, "the keepout keeps its class");
        // the class VALUE survives (mm-rounded within a board unit)
        let v = board2
            .rules
            .clearance_matrix
            .get_value(strict2, 1, 0, false);
        assert!(
            (v - 7000).abs() <= 1,
            "the class clearance value must survive the round trip (got {v})"
        );
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

    #[test]
    fn net_class_routing_semantics_survive_round_trip() {
        let mut board = import_dsn(MINI_DSN).expect("import");
        board.rules.clearance_matrix.append_class("strict-trace");
        board.rules.clearance_matrix.append_class("via-only");
        let strict = board
            .rules
            .clearance_matrix
            .get_no("strict-trace")
            .expect("strict class");
        let via_only = board
            .rules
            .clearance_matrix
            .get_no("via-only")
            .expect("via class");
        let default_class = board.rules.get_default_net_class();
        {
            let class = board.rules.net_classes.get_mut(default_class);
            class.set_trace_clearance_class(strict);
            class
                .default_item_clearance_classes
                .set(ItemClass::Trace, strict);
            class
                .default_item_clearance_classes
                .set(ItemClass::Via, via_only);
            class
                .default_item_clearance_classes
                .set(ItemClass::Pin, BoardRules::default_clearance_class());
            class
                .default_item_clearance_classes
                .set(ItemClass::Smd, via_only);
            class
                .default_item_clearance_classes
                .set(ItemClass::Area, strict);
            class.set_trace_half_width_on_layer(0, 1_234);
            class.set_trace_half_width_on_layer(1, 2_345);
            class.set_active_routing_layer(1, false);
            class.is_ignored_by_autorouter = true;
            class.set_shove_fixed(true);
            class.set_pull_tight(false);
            class.set_ignore_cycles_with_areas(true);
            class.set_minimum_trace_length(12_000.0);
            class.set_maximum_trace_length(98_000.0);
        }

        let json = export_kicad_json_checked(&board).expect("checked export");
        assert!(json.contains("\"routingMetadata\""));
        let reloaded = import_kicad_json(&json).expect("re-import");
        let class = reloaded.rules.net_classes.get(
            reloaded
                .rules
                .net_classes
                .get_by_name("default")
                .expect("default class"),
        );
        let matrix = &reloaded.rules.clearance_matrix;
        let class_name = |index| matrix.get_name(index).expect("clearance class name");

        assert_eq!(
            class_name(class.get_trace_clearance_class()),
            "strict-trace"
        );
        assert_eq!(
            class_name(class.default_item_clearance_classes.get(ItemClass::Trace)),
            "strict-trace"
        );
        assert_eq!(
            class_name(class.default_item_clearance_classes.get(ItemClass::Via)),
            "via-only"
        );
        assert_eq!(
            class_name(class.default_item_clearance_classes.get(ItemClass::Pin)),
            "default"
        );
        assert_eq!(
            class_name(class.default_item_clearance_classes.get(ItemClass::Smd)),
            "via-only"
        );
        assert_eq!(
            class_name(class.default_item_clearance_classes.get(ItemClass::Area)),
            "strict-trace"
        );
        assert_eq!(class.get_trace_half_width(0), 1_234);
        assert_eq!(class.get_trace_half_width(1), 2_345);
        assert!(class.is_active_routing_layer(0));
        assert!(!class.is_active_routing_layer(1));
        assert!(class.is_ignored_by_autorouter);
        assert!(class.is_shove_fixed());
        assert!(!class.get_pull_tight());
        assert!(class.get_ignore_cycles_with_areas());
        assert_eq!(class.get_minimum_trace_length(), 12_000.0);
        assert_eq!(class.get_maximum_trace_length(), 98_000.0);
    }

    #[test]
    fn checked_writer_rejects_dangling_rule_graph_references() {
        let mut board = import_dsn(MINI_DSN).expect("import");
        board.rules.via_infos.get_mut(0).set_padstack(999);
        assert!(matches!(
            export_kicad_json_checked(&board),
            Err(KicadJsonWriteError::DanglingRuleReference {
                kind: "padstack",
                number: 999,
                ..
            })
        ));

        let mut board = import_dsn(MINI_DSN).expect("import");
        let mut invalid_rule = crate::rules::ViaRule::new("invalid");
        invalid_rule.append_via(board.rules.via_infos.count());
        board.rules.via_rules.push(invalid_rule);
        assert!(matches!(
            export_kicad_json_checked(&board),
            Err(KicadJsonWriteError::DanglingRuleReference {
                kind: "viaInfo",
                ..
            })
        ));

        let mut board = import_dsn(MINI_DSN).expect("import");
        let missing_rule = board.rules.via_rules.len() + 10;
        board
            .rules
            .net_classes
            .get_mut(0)
            .set_via_rule(Some(missing_rule));
        assert!(matches!(
            export_kicad_json_checked(&board),
            Err(KicadJsonWriteError::DanglingRuleReference {
                kind: "viaRule",
                number,
                ..
            }) if number == missing_rule
        ));

        let mut board = import_dsn(MINI_DSN).expect("import");
        let missing_class = board.rules.clearance_matrix.get_class_count();
        board
            .rules
            .net_classes
            .get_mut(0)
            .set_trace_clearance_class(missing_class);
        assert!(matches!(
            export_kicad_json_checked(&board),
            Err(KicadJsonWriteError::DanglingRuleReference {
                kind: "trace clearance class",
                number,
                ..
            }) if number == missing_class
        ));

        let mut board = import_dsn(MINI_DSN).expect("import");
        let missing_class = board.rules.clearance_matrix.get_class_count();
        board
            .rules
            .net_classes
            .get_mut(0)
            .default_item_clearance_classes
            .set(ItemClass::Via, missing_class);
        assert!(matches!(
            export_kicad_json_checked(&board),
            Err(KicadJsonWriteError::DanglingRuleReference {
                kind: "item clearance class",
                number,
                ..
            }) if number == missing_class
        ));
    }

    #[test]
    fn checked_writer_rejects_dangling_item_references() {
        use crate::geometry::planar::{IntPoint, Polyline};

        let line = || {
            Polyline::from_int_points(&[
                IntPoint::new(10_000, 10_000),
                IntPoint::new(20_000, 10_000),
            ])
        };

        let mut board = import_dsn(MINI_DSN).expect("import");
        let item_id = board.insert_via(999, IntPoint::new(30_000, 30_000), vec![1], 1, false);
        assert!(matches!(
            export_kicad_json_checked(&board),
            Err(KicadJsonWriteError::DanglingItemReference {
                item_id: id,
                kind: "padstack",
                number: 999,
            }) if id == item_id
        ));

        let mut board = import_dsn(MINI_DSN).expect("import");
        let item_id = board.insert_trace(line(), 0, 100, vec![999], 1);
        assert!(matches!(
            export_kicad_json_checked(&board),
            Err(KicadJsonWriteError::DanglingItemReference {
                item_id: id,
                kind: "net",
                number: 999,
            }) if id == item_id
        ));

        let mut board = import_dsn(MINI_DSN).expect("import");
        let missing_layer = board.layer_structure.layer_count();
        let item_id = board.insert_trace(line(), missing_layer, 100, vec![1], 1);
        assert!(matches!(
            export_kicad_json_checked(&board),
            Err(KicadJsonWriteError::DanglingItemReference {
                item_id: id,
                kind: "layer",
                number,
            }) if id == item_id && number == missing_layer as i64
        ));

        let mut board = import_dsn(MINI_DSN).expect("import");
        let missing_class = board.rules.clearance_matrix.get_class_count();
        let item_id = board.insert_trace(line(), 0, 100, vec![1], missing_class);
        assert!(matches!(
            export_kicad_json_checked(&board),
            Err(KicadJsonWriteError::DanglingItemReference {
                item_id: id,
                kind: "clearance class",
                number,
            }) if id == item_id && number == missing_class as i64
        ));
    }

    #[test]
    fn checked_writer_rejects_empty_padstacks_and_omitted_component_items() {
        use crate::geometry::planar::{IntBox, IntPoint, Polyline, TileShape};

        let mut board = import_dsn(MINI_DSN).expect("import");
        let layer_count = board.layer_structure.layer_count();
        let empty = board
            .padstacks
            .add("empty", vec![None; layer_count], false, false);
        assert!(matches!(
            export_kicad_json_checked(&board),
            Err(KicadJsonWriteError::InvalidPadstack {
                padstack,
                reason: "it has no copper shape",
            }) if padstack == empty
        ));

        let mut board = import_dsn(MINI_DSN).expect("import");
        let layer_count = board.layer_structure.layer_count();
        let mut shapes = vec![None; layer_count];
        shapes[0] = Some(TileShape::Box(IntBox::from_coords(0, 0, 0, 0)));
        let degenerate = board.padstacks.add("point", shapes, false, false);
        assert!(matches!(
            export_kicad_json_checked(&board),
            Err(KicadJsonWriteError::InvalidPadstack {
                padstack,
                reason: "it contains an empty, unbounded, or degenerate shape",
            }) if padstack == degenerate
        ));

        let mut board = import_dsn(MINI_DSN).expect("import");
        let trace = board.insert_trace(
            Polyline::from_int_points(&[
                IntPoint::new(10_000, 10_000),
                IntPoint::new(20_000, 10_000),
            ]),
            0,
            100,
            vec![1],
            1,
        );
        board.set_component_no(trace, 1);
        assert!(matches!(
            export_kicad_json_checked(&board),
            Err(KicadJsonWriteError::UnrepresentableItem {
                item_id,
                reason: "component-owned traces are not supported",
            }) if item_id == trace
        ));
    }

    #[test]
    fn routing_metadata_and_fixed_states_survive_checked_round_trip() {
        use crate::board::{FixedState, Item, ItemBase};
        use crate::geometry::planar::{IntPoint, Point, PolygonShape, Polyline, PolylineArea};

        let mut board = import_dsn(MINI_DSN).expect("import");
        board
            .rules
            .set_trace_angle_restriction(crate::board::AngleRestriction::None);
        board.rules.set_ignore_conduction(false);
        board.rules.set_pin_edge_to_turn_dist(375.0);
        board.rules.set_use_slow_autoroute_algorithm(true);
        board.rules.via_at_smd_allowed = true;
        board
            .rules
            .set_same_net_clearance(ItemClass::Pin, ItemClass::Via, 321);

        let trace = board.insert_trace(
            Polyline::from_int_points(&[
                IntPoint::new(10_000, 10_000),
                IntPoint::new(20_000, 10_000),
            ]),
            0,
            100,
            vec![1],
            1,
        );
        board.set_fixed_state(trace, FixedState::ShoveFixed);

        let via = board.insert_via(1, IntPoint::new(30_000, 30_000), vec![1], 1, false);
        board.set_fixed_state(via, FixedState::Unfixed);

        let area = PolylineArea::new(
            PolygonShape::new(vec![
                Point::Int(IntPoint::new(40_000, 40_000)),
                Point::Int(IntPoint::new(45_000, 40_000)),
                Point::Int(IntPoint::new(45_000, 45_000)),
            ]),
            Vec::new(),
        );
        let keepout = board.insert_area(area, 0, "keepout", Vec::new(), 1, false);
        board.set_fixed_state(keepout, FixedState::Unfixed);

        let mut pin_base = ItemBase::new(0, vec![1], 1);
        pin_base.component_no = 7;
        pin_base.fixed_state = FixedState::ShoveFixed;
        let pin = board.insert_item(Item::new_via(
            pin_base,
            1,
            IntPoint::new(60_000, 60_000),
            true,
        ));

        let json = export_kicad_json_checked(&board).expect("checked export");
        let reloaded = import_kicad_json(&json).expect("re-import");
        assert_eq!(
            reloaded.rules.get_trace_angle_restriction(),
            crate::board::AngleRestriction::None
        );
        assert!(!reloaded.rules.get_ignore_conduction());
        assert_eq!(reloaded.rules.get_pin_edge_to_turn_dist(), 375.0);
        assert!(reloaded.rules.get_use_slow_autoroute_algorithm());
        assert!(reloaded.rules.via_at_smd_allowed);
        assert_eq!(
            reloaded
                .rules
                .get_same_net_clearance(ItemClass::Pin, ItemClass::Via),
            Some(321)
        );

        let state_for = |kind: &ItemKind, component_no: i32| {
            reloaded
                .items()
                .find(|(_, item)| {
                    item.base.component_no == component_no
                        && std::mem::discriminant(&item.kind) == std::mem::discriminant(kind)
                })
                .map(|(_, item)| item.base.fixed_state)
        };
        let original_trace_kind = board.get_item(trace).expect("trace").kind.clone();
        let original_via_kind = board.get_item(via).expect("via").kind.clone();
        assert_eq!(
            state_for(&original_trace_kind, 0),
            Some(FixedState::ShoveFixed)
        );
        assert_eq!(state_for(&original_via_kind, 0), Some(FixedState::Unfixed));
        assert_eq!(
            reloaded
                .items()
                .find(|(_, item)| item.base.component_no != 0)
                .map(|(_, item)| item.base.fixed_state),
            Some(FixedState::ShoveFixed)
        );
        assert_eq!(
            reloaded
                .items()
                .find(|(_, item)| matches!(&item.kind, ItemKind::ObstacleArea(area) if area.name == "keepout"))
                .map(|(_, item)| item.base.fixed_state),
            Some(FixedState::Unfixed)
        );
        assert!(board.get_item(pin).is_some());
    }
}
