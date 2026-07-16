//! Semantic DSN import (port of the reading half of
//! `io/specctra/DsnReader.java` and the Structure/Library/Network scope
//! classes, simplified): maps a parsed DSN tree onto a [`BasicBoard`]
//! with layers, padstacks, placed component pins and nets.
//!
//! Simplifications (documented): pad shapes are converted to boxes /
//! bounding octagons (circles/ovals); back-side placement mirrors pin
//! offsets and pad shapes at the y axis and flips the shape layers;
//! non-quarter-turn rotations only rotate pin offsets, not pad shapes.

use std::collections::HashMap;

use crate::board::basic_board::BasicBoard;
use crate::board::{Layer, LayerStructure};
use crate::core::Padstacks;
use crate::geometry::planar::{Circle, IntBox, IntOctagon, IntPoint, TileShape};
use crate::io::dsn::{parse_dsn, SExpr};
use crate::rules::{BoardRules, ClearanceMatrix};

#[derive(Debug, Clone, PartialEq)]
pub struct ImportError(pub String);

impl std::fmt::Display for ImportError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "DSN import error: {}", self.0)
    }
}

impl std::error::Error for ImportError {}

fn err(message: impl Into<String>) -> ImportError {
    ImportError(message.into())
}

/// Maps a DSN item-class token to the [`ItemClass`] enum (for `*_same_net`
/// rules). `wire` maps to a trace; unknown names return `None`.
fn item_class_of(name: &str) -> Option<crate::rules::ItemClass> {
    use crate::rules::ItemClass;
    match name {
        "via" => Some(ItemClass::Via),
        "pin" => Some(ItemClass::Pin),
        "smd" => Some(ItemClass::Smd),
        "area" => Some(ItemClass::Area),
        "wire" => Some(ItemClass::Trace),
        _ => None,
    }
}

/// Resolves a DSN clearance-class name to a clearance-matrix class index,
/// creating the class on demand (Java `Structure.append_clearance_class` /
/// `Network.get_clearance_class`). `wire` and `default` map to the default
/// class; a newly created class copies the default row (`append_class`), and
/// the standard item names via/pin/smd/area additionally point the default net
/// class's per-item clearance classes at the new class.
fn resolve_clearance_class(rules: &mut BoardRules, name: &str) -> usize {
    let lname = name.to_ascii_lowercase();
    match lname.as_str() {
        "wire" | "default" => return BoardRules::default_clearance_class(),
        "null" => return BoardRules::clearance_class_none(),
        _ => {}
    }
    if let Some(idx) = rules.clearance_matrix.get_no(name) {
        return idx;
    }
    rules.clearance_matrix.append_class(name);
    let idx = rules
        .clearance_matrix
        .get_no(name)
        .unwrap_or_else(BoardRules::default_clearance_class);
    let item = match lname.as_str() {
        "via" => Some(crate::rules::ItemClass::Via),
        "pin" => Some(crate::rules::ItemClass::Pin),
        "smd" => Some(crate::rules::ItemClass::Smd),
        "area" => Some(crate::rules::ItemClass::Area),
        _ => None,
    };
    if let Some(ic) = item {
        let default_nc = rules.get_default_net_class();
        rules
            .net_classes
            .get_mut(default_nc)
            .default_item_clearance_classes
            .set(ic, idx);
    }
    idx
}

struct ImagePin {
    padstack_name: String,
    pin_name: String,
    dx: f64,
    dy: f64,
}

/// A keepout declared INSIDE an `(image ...)` footprint, in image-relative
/// (board-unit) coordinates. Instantiated as an obstacle area at every
/// placement of the image, transformed like the pins (Java `Package.Keepout`).
struct ImageKeepout {
    /// relative shape, already scaled to board units
    area: crate::geometry::planar::PolygonShape,
    /// a named layer, or `None` for signal/all (every layer)
    layer: Option<usize>,
    /// a `(via_keepout ...)`: blocks vias only
    via_only: bool,
}

/// A parsed `(image ...)` footprint: its pins and its keepouts.
struct Image {
    pins: Vec<ImagePin>,
    keepouts: Vec<ImageKeepout>,
}

/// Imports a DSN file into a board: layers, default rules, padstacks,
/// component pins (as drill items carrying their nets) and nets.
pub fn import_dsn(content: &str) -> Result<BasicBoard, ImportError> {
    let mut board = import_dsn_inner(content)?;
    // retain the document without its wiring for DSN export (the router
    // only changes the wiring section): remove exactly the balanced
    // `(wiring ...)` span, keeping everything before and after it
    board.dsn_source = Some(strip_wiring(content));
    Ok(board)
}

/// The document with its `(wiring ...)` sections removed. Spans are found
/// by paren counting (quoted strings have no escapes, matching the
/// parser); an unbalanced span leaves the document untouched.
fn strip_wiring(content: &str) -> String {
    let mut out = String::with_capacity(content.len());
    let mut rest = content;
    while let Some(pos) = rest.find("(wiring") {
        // require a real section name: "(wiring" then a delimiter
        let after = rest[pos + "(wiring".len()..].bytes().next();
        let is_section = after.is_none_or(|c| c.is_ascii_whitespace() || c == b')' || c == b'(');
        let mut end = None;
        if is_section {
            let bytes = rest.as_bytes();
            let mut depth = 0usize;
            let mut i = pos;
            while i < bytes.len() {
                match bytes[i] {
                    b'"' => {
                        // parser rule: a lone quote before whitespace/`)`
                        // is an atom, otherwise a quoted string (no escapes)
                        let next = bytes.get(i + 1);
                        if next.is_some_and(|c| !c.is_ascii_whitespace() && *c != b')') {
                            match bytes[i + 1..].iter().position(|&c| c == b'"') {
                                Some(q) => i += q + 1,
                                None => break, // unterminated string
                            }
                        }
                    }
                    b'(' => depth += 1,
                    b')' => {
                        depth -= 1;
                        if depth == 0 {
                            end = Some(i + 1);
                            break;
                        }
                    }
                    _ => {}
                }
                i += 1;
            }
        }
        match end {
            Some(end) => {
                out.push_str(rest[..pos].trim_end_matches([' ', '\t']));
                rest = &rest[end..];
            }
            None => {
                // not a wiring section (or unbalanced): keep it verbatim
                out.push_str(&rest[..pos + "(wiring".len()]);
                rest = &rest[pos + "(wiring".len()..];
            }
        }
    }
    out.push_str(rest);
    out
}

fn import_dsn_inner(content: &str) -> Result<BasicBoard, ImportError> {
    let pcb = parse_dsn(content).map_err(|e| err(e.to_string()))?;
    if !pcb.name().is_some_and(|n| n.eq_ignore_ascii_case("pcb")) {
        return Err(err("root node is not (pcb ...)"));
    }
    let structure = pcb.child("structure").ok_or_else(|| err("no structure"))?;

    // resolution: `(resolution <unit> <value>)`. File coordinates are
    // multiplied by <value> to get integer board units; <unit> is the
    // physical unit, which must be preserved for a faithful SES round-trip
    // (a `mil` design was previously relabelled `um` on export).
    let resolution_node = pcb.child("resolution");
    let unit: String = resolution_node
        .and_then(|r| r.args().next())
        .map(|a| a.to_string())
        .unwrap_or_else(|| "um".to_string());
    let resolution: f64 = resolution_node
        .and_then(|r| r.args().nth(1))
        .and_then(|a| a.parse().ok())
        .unwrap_or(1.0);
    let scale = |v: f64| -> i32 { (v * resolution).round() as i32 };

    // layers
    let mut layers: Vec<(String, bool, i64)> = Vec::new();
    for layer_node in structure.children("layer") {
        let name = layer_node.arg().ok_or_else(|| err("layer without name"))?;
        let is_signal = layer_node
            .child("type")
            .and_then(|t| t.arg())
            .is_none_or(|t| t.eq_ignore_ascii_case("signal"));
        let index = layer_node
            .child("property")
            .and_then(|p| p.child("index"))
            .and_then(|i| i.arg())
            .and_then(|a| a.parse().ok())
            .unwrap_or(layers.len() as i64);
        layers.push((name.to_string(), is_signal, index));
    }
    if layers.is_empty() {
        return Err(err("no layers"));
    }
    layers.sort_by_key(|(_, _, index)| *index);
    let layer_structure = LayerStructure::new(
        layers
            .iter()
            .map(|(name, is_signal, _)| Layer::new(name.clone(), *is_signal))
            .collect(),
    );
    let layer_no = |name: &str| -> Option<usize> { layer_structure.get_no(name) };
    let layer_count = layer_structure.layer_count();

    // default rules (`(clear ...)` is a Specctra alias for `(clearance ...)`)
    let default_clearance = structure
        .child("rule")
        .and_then(|r| r.child("clearance").or_else(|| r.child("clear")))
        .and_then(|c| c.arg_f64())
        .map(&scale)
        .unwrap_or(200);
    let default_width = structure
        .child("rule")
        .and_then(|r| r.child("width"))
        .and_then(|w| w.arg_f64())
        .map(&scale)
        .unwrap_or(250);
    // clearance classes: base "null" (0), "default" (1), "smd" (2). Typed
    // clearance rules `(clear V (type A_B))` then refine or create per-item and
    // named clearance classes once the rules exist.
    let clearance_matrix =
        ClearanceMatrix::new(layer_structure.clone(), &["null", "default", "smd"]);
    let mut rules = BoardRules::new(layer_structure.clone(), clearance_matrix);
    rules.clearance_matrix.set_default_value(default_clearance);
    // (structure (control (via_at_smd on))): vias may attach to SMD pads
    // (Java Structure.read_control_scope -> via_at_smd_allowed, default off)
    rules.via_at_smd_allowed = structure
        .child("control")
        .and_then(|c| c.child("via_at_smd"))
        .and_then(|v| v.arg())
        .is_some_and(|v| v.eq_ignore_ascii_case("on"));
    let default_nc = rules.get_default_net_class();
    // The default net class keeps SMD pads on the tight smd class (2), so plain
    // (unclassed) nets' smd pads stay at the smd clearance. Classed nets that
    // carry a clearance rule override this via set_all below. append_net_class
    // inherits these, so this must run before any class is appended.
    rules
        .net_classes
        .get_mut(default_nc)
        .default_item_clearance_classes
        .set(crate::rules::ItemClass::Smd, 2);

    // typed clearance rules `(clear V (type A_B))` (Java Structure.set_clearance_rule):
    // A and B resolve to clearance-matrix classes — "wire"/"default" map to the
    // default class, the item classes via/pin/smd/area also point the default
    // net class's item clearance classes at their class, any other name is
    // created on demand. `smd_to_turn_gap` sets the pin-edge-to-turn distance;
    // `A_B_same_net` records a same-net clearance for the DRC. Splitting is at
    // the FIRST '_' (Java `split("_", 2)`), so the second name may itself carry
    // underscores (e.g. `wire_kicad_default`); a two-token `(type NAME1 NAME2)`
    // is ONE pair split across tokens (Java's quoted-pair form, the second
    // token optionally led by the '_' separator). Hyphen spellings (`smd-smd`)
    // are normalized to underscore.
    if let Some(rule) = structure.child("rule") {
        // if any wire pair appears, pre-create the four default item classes
        // (Java create_default_clearance_classes)
        let has_wire_pair = rule
            .children("clearance")
            .chain(rule.children("clear"))
            .filter_map(|c| c.child("type"))
            .flat_map(|t| t.args())
            .any(|k| {
                let k = k.to_ascii_lowercase().replace('-', "_");
                k.starts_with("wire_") || k.ends_with("_wire")
            });
        if has_wire_pair {
            for name in ["via", "smd", "pin", "area"] {
                resolve_clearance_class(&mut rules, name);
            }
        }
        for clearance_node in rule.children("clearance").chain(rule.children("clear")) {
            let Some(value) = clearance_node.arg_f64().map(&scale) else {
                continue;
            };
            let Some(type_node) = clearance_node.child("type") else {
                continue; // the untyped default, already applied
            };
            let tokens: Vec<String> = type_node
                .args()
                .map(|t| t.to_ascii_lowercase().replace('-', "_"))
                .collect();
            let mut pairs: Vec<(String, String)> = Vec::new();
            if tokens.len() == 2 {
                // one pair split across two tokens
                let b = tokens[1].strip_prefix('_').unwrap_or(&tokens[1]);
                pairs.push((tokens[0].clone(), b.to_string()));
            } else {
                for kind in &tokens {
                    if kind == "smd_to_turn_gap" {
                        rules.set_pin_edge_to_turn_dist(value as f64);
                        continue;
                    }
                    // `A_B_same_net`: the clearance required between two
                    // SAME-NET item-class items (a drill-breakout rule). Java
                    // parses but never applies these; the DRC uses them.
                    if let Some(base) = kind.strip_suffix("_same_net") {
                        if let Some((a, b)) = base.split_once('_') {
                            if let (Some(ica), Some(icb)) = (item_class_of(a), item_class_of(b)) {
                                rules.set_same_net_clearance(ica, icb, value);
                            }
                        }
                        continue;
                    }
                    // split at the FIRST '_': the second name may contain
                    // underscores (a single-token type has no pair to set)
                    if let Some((a, b)) = kind.split_once('_') {
                        pairs.push((a.to_string(), b.to_string()));
                    }
                }
            }
            for (a, b) in pairs {
                let ci = resolve_clearance_class(&mut rules, &a);
                let cj = resolve_clearance_class(&mut rules, &b);
                rules
                    .clearance_matrix
                    .set_value_on_all_layers(ci, cj, value);
                rules
                    .clearance_matrix
                    .set_value_on_all_layers(cj, ci, value);
            }
        }
    }
    rules.set_default_trace_half_widths((default_width / 2).max(1));

    // library: padstacks and images
    let mut padstacks = Padstacks::new(layer_count);
    let mut padstack_nos: HashMap<String, usize> = HashMap::new();
    let mut images: HashMap<String, Image> = HashMap::new();
    for library in pcb.children("library") {
        for padstack_node in library.children("padstack") {
            let name = padstack_node
                .arg()
                .ok_or_else(|| err("padstack without name"))?;
            let mut shapes: Vec<Option<TileShape>> = vec![None; layer_count];
            for shape_node in padstack_node.children("shape") {
                let Some(inner) = shape_node.as_list().and_then(|l| l.get(1)) else {
                    continue;
                };
                let Some((shape, layer_name)) = read_pad_shape(inner, &scale) else {
                    continue;
                };
                if let Some(l) = layer_no(&layer_name) {
                    shapes[l] = Some(shape);
                }
            }
            // Java read_padstack_scope: attach defaults ON when the
            // `(attach ...)` scope is omitted; only an explicit `off`
            // forbids attaching to SMD pads.
            let attach = padstack_node
                .child("attach")
                .and_then(|a| a.arg())
                .is_none_or(|v| !v.eq_ignore_ascii_case("off"));
            let no = padstacks.add(name, shapes, attach, false);
            padstack_nos.insert(padstacks.get_by_no(no).unwrap().name.clone(), no);
        }
        for image_node in library.children("image") {
            let name = image_node.arg().ok_or_else(|| err("image without name"))?;
            let mut pins = Vec::new();
            for pin_node in image_node.children("pin") {
                let mut args = pin_node.args();
                let padstack_name = args.next().unwrap_or_default().to_string();
                // optional (rotate ...) etc. are lists, args() skips them;
                // the remaining atoms are pin name and the two offsets
                let rest: Vec<&str> = args.collect();
                if rest.len() < 3 {
                    continue;
                }
                let pin_name = rest[rest.len() - 3].to_string();
                let dx: f64 = rest[rest.len() - 2].parse().unwrap_or(0.0);
                let dy: f64 = rest[rest.len() - 1].parse().unwrap_or(0.0);
                pins.push(ImagePin {
                    padstack_name,
                    pin_name,
                    dx,
                    dy,
                });
            }
            // keepouts declared inside the footprint (Java Package.read_scope);
            // place_keepout is placement-only and ignored by routing, like the
            // structure-level place_keepout the importer already skips.
            let mut keepouts = Vec::new();
            for kind in ["keepout", "via_keepout"] {
                for ko in image_node.children(kind) {
                    let Some(shape_node) = ko
                        .child("polygon")
                        .or_else(|| ko.child("rect"))
                        .or_else(|| ko.child("circle"))
                        .or_else(|| ko.child("path"))
                    else {
                        continue;
                    };
                    let layer = shape_node.arg().and_then(&layer_no);
                    let corners = keepout_corners(shape_node, &scale);
                    if corners.len() < 3 {
                        continue;
                    }
                    keepouts.push(ImageKeepout {
                        area: crate::geometry::planar::PolygonShape::new(corners),
                        layer,
                        via_only: kind == "via_keepout",
                    });
                }
            }
            images.insert(name.to_string(), Image { pins, keepouts });
        }
    }

    // network: pin reference "COMP-PIN" -> net number (split at the last
    // '-', like Java)
    let mut pin_nets: HashMap<String, i32> = HashMap::new();
    for network in pcb.children("network") {
        for net_node in network.children("net") {
            let net_name = net_node.arg().unwrap_or_default();
            let net_no = rules.nets.add(net_name, 1, false);
            for pins_node in net_node.children("pins") {
                for pin_ref in pins_node.args() {
                    pin_nets.insert(pin_ref.to_string(), net_no);
                }
            }
        }
    }

    // network classes: per-class trace width, clearance and via
    // ((class NAME net... (circuit (use_via V)) (rule (width W) ...)))
    //
    // Each distinct net-class clearance value becomes its own dynamic clearance
    // matrix class, so a net keeps its class's spacing instead of the hardcoded
    // default. Classes whose clearance equals the board default reuse class 1,
    // so single-class boards are unchanged. `(clear ...)` is a Specctra alias
    // for `(clearance ...)`.
    let clearance_child = |rule: &SExpr| -> Option<f64> {
        rule.child("clearance")
            .or_else(|| rule.child("clear"))
            .and_then(|c| c.arg_f64())
    };
    // Value-keyed dedup among classes that carry a clearance rule: two classes
    // with the same clearance value share one matrix class (identical to Java's
    // per-name classes since the values are equal). NOT seeded with the default
    // value — a named class whose clearance equals the board default still gets
    // its own matrix class so set_all can point its pins at the class value
    // (2002) rather than the tighter smd class (500), matching Java.
    let mut class_for_clearance: HashMap<i32, usize> = HashMap::new();

    // named via infos and via rules declared in the network scope
    // ((via NAME PADSTACK [CLEARANCE_CLASS] [attach]) and
    //  (via_rule NAME VIA_INFO...)); net classes reference the rule by name.
    let mut via_rule_ids: HashMap<String, usize> = HashMap::new();
    for network in pcb.children("network") {
        for via_node in network.children("via") {
            let mut a = via_node.args();
            let (Some(name), Some(padstack_name)) = (a.next(), a.next()) else {
                continue;
            };
            let Some(&padstack_no) = padstack_nos.get(padstack_name) else {
                continue;
            };
            let rest: Vec<&str> = a.collect();
            let attach = rest.iter().any(|t| t.eq_ignore_ascii_case("attach"));
            // the optional clearance class is the non-"attach" trailing token
            let cl = rest
                .iter()
                .find(|t| !t.eq_ignore_ascii_case("attach"))
                .map(|n| resolve_clearance_class(&mut rules, n))
                .unwrap_or_else(BoardRules::default_clearance_class);
            let _ = rules
                .via_infos
                .add(crate::rules::ViaInfo::new(name, padstack_no, cl, attach));
        }
        for rule_node in network.children("via_rule") {
            let mut a = rule_node.args();
            let Some(rule_name) = a.next() else {
                continue;
            };
            let mut via_rule = crate::rules::ViaRule::new(rule_name);
            let mut any = false;
            for via_name in a {
                if let Some(id) = rules.via_infos.get_by_name(via_name) {
                    via_rule.append_via(id);
                    any = true;
                }
            }
            if any {
                // Java add_via_rule: a redeclared rule REPLACES the existing
                // one (in place, so indices already bound to net classes stay
                // valid) rather than accumulating orphan duplicates.
                if let Some(&existing) = via_rule_ids.get(rule_name) {
                    rules.via_rules[existing] = via_rule;
                } else {
                    rules.via_rules.push(via_rule);
                    via_rule_ids.insert(rule_name.to_string(), rules.via_rules.len() - 1);
                }
            }
        }
    }

    for network in pcb.children("network") {
        for class_node in network.children("class") {
            let mut class_args = class_node.args();
            let Some(class_name) = class_args.next() else {
                continue;
            };
            let member_nets: Vec<&str> = class_args.collect();
            let is_default_descriptor = member_nets.is_empty();
            let half_width = class_node
                .child("rule")
                .and_then(|r| r.child("width"))
                .and_then(|w| w.arg_f64())
                .map(&scale)
                .map(|w| (w / 2).max(1));
            // the net class's own trace clearance (scaled board units)
            let class_clearance: Option<i32> = class_node
                .child("rule")
                .and_then(&clearance_child)
                .map(&scale);
            let clearance_class_idx = match class_clearance {
                Some(c) => {
                    if is_default_descriptor && c == default_clearance {
                        // the (class ... (rule (clearance <default>))) descriptor
                        // for the default net class: keep the shared default
                        // class and leave the default net class's item classes
                        // (so plain-net smd pads stay tight); no set_all below.
                        BoardRules::default_clearance_class()
                    } else if let Some(&idx) = class_for_clearance.get(&c) {
                        idx
                    } else {
                        // create a dynamic matrix class holding this spacing to
                        // every class (including itself)
                        let name = format!("cl_{c}");
                        rules.clearance_matrix.append_class(&name);
                        let idx = rules
                            .clearance_matrix
                            .get_no(&name)
                            .unwrap_or_else(BoardRules::default_clearance_class);
                        // Java parity (Network.add_clearance_rule): the new
                        // class's clearance to every existing class is the
                        // MAXIMUM of its own value and the existing entry, so a
                        // stricter class is never under-cleared next to a looser
                        // one regardless of class-creation order. append_class
                        // already copied the (progressively elevated) default
                        // row, so max() preserves accumulated cross-class
                        // spacing. Start at class 1 to leave "null" (0) at zero.
                        let n = rules.clearance_matrix.get_class_count();
                        let layers = rules.clearance_matrix.get_layer_count();
                        for j in 1..n {
                            for layer in 0..layers {
                                let curr = rules
                                    .clearance_matrix
                                    .get_value(idx, j, layer, false)
                                    .max(c);
                                rules.clearance_matrix.set_value(idx, j, layer, curr);
                                rules.clearance_matrix.set_value(j, idx, layer, curr);
                            }
                        }
                        // the class's clearance to itself is exactly its own value
                        rules.clearance_matrix.set_value_on_all_layers(idx, idx, c);
                        class_for_clearance.insert(c, idx);
                        idx
                    }
                }
                // no inline clearance rule: honor a `(clearance_class NAME)`
                // reference (Java insert_net_class) — the trace clearance class
                // becomes the named class; item classes stay at their defaults
                // (no set_all), unlike an inline clearance rule.
                None => match class_node.child("clearance_class").and_then(|c| c.arg()) {
                    Some(name) => resolve_clearance_class(&mut rules, name),
                    None => BoardRules::default_clearance_class(),
                },
            };
            let via_padstack = class_node
                .child("circuit")
                .and_then(|c| c.child("use_via"))
                .and_then(|u| u.arg())
                .and_then(|name| padstack_nos.get(name).copied());
            // the class listing no nets describes the default rules; a named
            // class is appended inheriting the default net class's item classes
            // (so an smd pad on a rule-less class stays on the smd class).
            let class_idx = if is_default_descriptor {
                rules.get_default_net_class()
            } else {
                rules.append_net_class(class_name)
            };
            {
                let class = rules.net_classes.get_mut(class_idx);
                if let Some(hw) = half_width {
                    class.set_trace_half_width(hw);
                }
                class.set_trace_clearance_class(clearance_class_idx);
                // Java (Network.add_clearance_rule): default_item_clearance_classes
                // .set_all(class_no) — pins (incl. smd), vias and areas of a
                // named class that carries a clearance rule use its clearance
                // class too, not just traces, EVEN when the value equals the
                // board default. Never applied to the default net class (the
                // folded default descriptor), which must keep smd pads tight.
                if !is_default_descriptor && class_clearance.is_some() {
                    class
                        .default_item_clearance_classes
                        .set_all(clearance_class_idx);
                }
            }
            // a `(via_rule NAME)` reference binds the class to a named via rule
            // declared in the network scope (Java insert_net_class); it wins
            // over the `(circuit (use_via ...))` fallback below.
            let named_via_rule = class_node
                .child("via_rule")
                .and_then(|v| v.arg())
                .and_then(|name| via_rule_ids.get(name).copied());
            if let Some(rule_id) = named_via_rule {
                rules
                    .net_classes
                    .get_mut(class_idx)
                    .set_via_rule(Some(rule_id));
            } else if let Some(padstack_no) = via_padstack {
                // Java create_via_rule reuses the existing via info for the
                // padstack; only when none was declared is one created, with
                // the default attach rule (via_at_smd && padstack attach).
                let existing = (0..rules.via_infos.count())
                    .find(|&i| rules.via_infos.get(i).get_padstack() == padstack_no);
                let via_info_id = existing.or_else(|| {
                    let attach = rules.via_at_smd_allowed
                        && padstacks
                            .get_by_no(padstack_no)
                            .is_some_and(|p| p.attach_allowed);
                    rules.via_infos.add(crate::rules::ViaInfo::new(
                        format!("via::{class_name}"),
                        padstack_no,
                        clearance_class_idx,
                        attach,
                    ))
                });
                if let Some(via_info_id) = via_info_id {
                    let mut via_rule = crate::rules::ViaRule::new(class_name);
                    via_rule.append_via(via_info_id);
                    rules.via_rules.push(via_rule);
                    let rule_id = rules.via_rules.len() - 1;
                    rules
                        .net_classes
                        .get_mut(class_idx)
                        .set_via_rule(Some(rule_id));
                }
            }
            for net_name in member_nets {
                let numbers: Vec<i32> = rules
                    .nets
                    .get_by_name(net_name)
                    .iter()
                    .map(|n| n.net_number)
                    .collect();
                for no in numbers {
                    if let Some(net) = rules.nets.get_by_no_mut(no) {
                        net.set_class(class_idx);
                    }
                }
            }
        }
    }

    let mut board = BasicBoard::new(layer_structure, rules, padstacks);
    board.resolution = resolution.round() as i32;
    board.unit = unit;

    // power planes: conduction areas connecting their net's pins
    // ((plane NET (polygon LAYER aperture x y ...)))
    for plane_node in structure.children("plane") {
        let Some(net_name) = plane_node.arg() else {
            continue;
        };
        let Some(polygon) = plane_node.child("polygon") else {
            continue;
        };
        let Some(layer) = polygon.arg().and_then(|n| board.layer_structure.get_no(n)) else {
            continue;
        };
        let nums: Vec<f64> = polygon
            .args()
            .skip(2)
            .filter_map(|a| a.parse().ok())
            .collect();
        let corners: Vec<crate::geometry::planar::Point> = nums
            .chunks_exact(2)
            .map(|c| crate::geometry::planar::Point::Int(IntPoint::new(scale(c[0]), scale(c[1]))))
            .collect();
        if corners.len() < 3 {
            continue;
        }
        let net_nos: Vec<i32> = board
            .rules
            .nets
            .get_by_name(net_name)
            .iter()
            .map(|n| n.net_number)
            .collect();
        let area = crate::geometry::planar::PolylineArea::new(
            crate::geometry::planar::PolygonShape::new(corners),
            Vec::new(),
        );
        // the plane uses its net class's Area item clearance class (Java
        // Network insert plane -> get(Area)), so a high-clearance net's copper
        // pour keeps its spacing (#2/#4)
        let plane_cl = board.rules.item_clearance_class_for(
            net_nos.first().copied().unwrap_or(0),
            crate::rules::ItemClass::Area,
        );
        board.insert_area(area, layer, net_name, net_nos, plane_cl, true);
    }

    // keepout areas ((keepout ...) traces+vias, (via_keepout ...) vias
    // only; Java: Structure read scope -> insert_obstacle /
    // insert_via_obstacle). place_keepout affects component placement
    // only and is skipped, like routing ignores it in Java.
    for kind in ["keepout", "via_keepout"] {
        for node in structure.children(kind) {
            let Some(shape_node) = node
                .child("polygon")
                .or_else(|| node.child("rect"))
                .or_else(|| node.child("circle"))
                .or_else(|| node.child("path"))
            else {
                continue;
            };
            let layer_arg = shape_node.arg().unwrap_or("signal");
            let layers: Vec<usize> = match board.layer_structure.get_no(layer_arg) {
                Some(l) => vec![l],
                // "signal", "all", "pcb": every layer
                None => (0..board.layer_structure.layer_count()).collect(),
            };
            let corners = keepout_corners(shape_node, &scale);
            if corners.len() < 3 {
                continue;
            }
            let area = crate::geometry::planar::PolylineArea::new(
                crate::geometry::planar::PolygonShape::new(corners),
                Vec::new(),
            );
            let name = node.arg().unwrap_or(kind);
            // a keepout may name its clearance class (Java uses the keepout's
            // clearance class); default when absent.
            let keepout_cl = node
                .child("clearance_class")
                .and_then(|c| c.arg())
                .map(|n| resolve_clearance_class(&mut board.rules, n))
                .unwrap_or_else(BoardRules::default_clearance_class);
            for layer in layers {
                let mut item = crate::board::Item::new_obstacle_area(
                    crate::board::ItemBase::new(0, Vec::new(), keepout_cl),
                    area.clone(),
                    layer,
                    name,
                    false,
                );
                if kind == "via_keepout" {
                    if let crate::board::ItemKind::ObstacleArea(a) = &mut item.kind {
                        a.via_only = true;
                    }
                }
                let id = board.insert_item(item);
                board.set_fixed_state(id, crate::board::FixedState::SystemFixed);
            }
        }
    }

    // boundary: keepout strips along the outline edges on all layers so
    // routes stay inside the board (Java: BoardOutline tree shapes)
    if let Some(boundary) = structure.child("boundary") {
        if let Some(path) = boundary.child("path") {
            let coords: Vec<f64> = path.args().skip(2).filter_map(|a| a.parse().ok()).collect();
            let corners: Vec<IntPoint> = coords
                .chunks_exact(2)
                .map(|c| IntPoint::new(scale(c[0]), scale(c[1])))
                .collect();
            insert_boundary_keepouts(&mut board, &corners, default_clearance / 2);
        } else if let Some(rect) = boundary.child("rect") {
            // (boundary (rect <layer> x1 y1 x2 y2)): a rectangular outline.
            // Previously only `path` boundaries produced keepouts, so
            // rect-outline boards were unconfined and routes could escape the
            // board. Java reads `rect` as a first-class boundary shape.
            let coords: Vec<f64> = rect.args().skip(1).filter_map(|a| a.parse().ok()).collect();
            if coords.len() >= 4 {
                let (xmin, xmax) = (coords[0].min(coords[2]), coords[0].max(coords[2]));
                let (ymin, ymax) = (coords[1].min(coords[3]), coords[1].max(coords[3]));
                let corners = vec![
                    IntPoint::new(scale(xmin), scale(ymin)),
                    IntPoint::new(scale(xmax), scale(ymin)),
                    IntPoint::new(scale(xmax), scale(ymax)),
                    IntPoint::new(scale(xmin), scale(ymax)),
                ];
                insert_boundary_keepouts(&mut board, &corners, default_clearance / 2);
            }
        }
    }

    // placement: instantiate the image pins per component place
    // cache of placed padstack variants per (padstack, quadrant, side):
    // quarter turns rotate the shapes, back-side placement additionally
    // mirrors them at the vertical axis and flips their layers.
    // Flip style (Java: Components.flip_style_rotate_first): by default
    // back-side pins mirror BEFORE the rotation; with
    // (place_control (flip_style rotate_first)) they rotate first.
    let rotate_first = structure
        .child("place_control")
        .and_then(|pc| pc.child("flip_style"))
        .and_then(|fs| fs.arg())
        .is_some_and(|v| v.eq_ignore_ascii_case("rotate_first"));
    let mut placed_padstacks: HashMap<(usize, i32, bool), usize> = HashMap::new();
    for placement in pcb.children("placement") {
        for component in placement.children("component") {
            let image_name = component.arg().unwrap_or_default();
            let Some(image) = images.get(image_name) else {
                continue;
            };
            let image_pins = &image.pins;
            for place in component.children("place") {
                let args: Vec<&str> = place.args().collect();
                if args.len() < 4 {
                    continue;
                }
                let refdes = args[0];
                let x: f64 = args[1].parse().unwrap_or(0.0);
                let y: f64 = args[2].parse().unwrap_or(0.0);
                let on_front = args[3].eq_ignore_ascii_case("front");
                let rotation_deg: f64 = args.get(4).and_then(|a| a.parse().ok()).unwrap_or(0.0);
                let rotation = rotation_deg.to_radians();
                let (sin, cos) = rotation.sin_cos();
                // component rotations in quarter turns rotate the pad
                // shapes too (a shared padstack gets a rotated variant);
                // other angles only rotate the pin offsets, like before
                let quarter = {
                    let r = rotation_deg.rem_euclid(360.0) / 90.0;
                    if (r - r.round()).abs() < 1e-9 {
                        (r.round() as i32).rem_euclid(4)
                    } else {
                        0
                    }
                };
                // per-pin clearance-class overrides in the placement
                // (Java Network.insert_component: `(pin N (clearance_class NAME))`)
                let mut pin_overrides: HashMap<&str, &str> = HashMap::new();
                for pin_node in place.children("pin") {
                    if let (Some(pin_name), Some(cc)) = (
                        pin_node.arg(),
                        pin_node.child("clearance_class").and_then(|c| c.arg()),
                    ) {
                        pin_overrides.insert(pin_name, cc);
                    }
                }
                for pin in image_pins {
                    let Some(&padstack_no) = padstack_nos.get(&pin.padstack_name) else {
                        continue;
                    };
                    let padstack_no = if quarter != 0 || !on_front {
                        match placed_padstacks.get(&(padstack_no, quarter, on_front)) {
                            Some(&no) => no,
                            None => {
                                let origin = IntPoint::new(0, 0);
                                let (name, shapes, attach) = {
                                    let p = board.padstacks.get_by_no(padstack_no).unwrap();
                                    let n = p.board_layer_count();
                                    let mut shapes: Vec<Option<TileShape>> = vec![None; n];
                                    for l in 0..n {
                                        let Some(s) = p.get_shape(l) else {
                                            continue;
                                        };
                                        // back side: mirror the shape and
                                        // flip its layer, like the pin
                                        // offsets; mirror before or after
                                        // the rotation per the flip style
                                        let (s, target) = if on_front {
                                            (s.turn_90_degree(quarter, origin), l)
                                        } else if rotate_first {
                                            (
                                                s.turn_90_degree(quarter, origin)
                                                    .mirror_vertical(origin),
                                                n - 1 - l,
                                            )
                                        } else {
                                            (
                                                s.mirror_vertical(origin)
                                                    .turn_90_degree(quarter, origin),
                                                n - 1 - l,
                                            )
                                        };
                                        shapes[target] = Some(s);
                                    }
                                    (
                                        format!(
                                            "{}::rot{}{}",
                                            p.name,
                                            quarter * 90,
                                            if on_front { "" } else { "::back" }
                                        ),
                                        shapes,
                                        p.attach_allowed,
                                    )
                                };
                                let no = board.padstacks.add(name, shapes, attach, false);
                                placed_padstacks.insert((padstack_no, quarter, on_front), no);
                                no
                            }
                        }
                    } else {
                        padstack_no
                    };
                    // pin offset: mirror at the y axis for the back side
                    // (before or after the rotation per the flip style)
                    let px = if on_front || rotate_first {
                        pin.dx
                    } else {
                        -pin.dx
                    };
                    let (mut dx, dy) = (px * cos - pin.dy * sin, px * sin + pin.dy * cos);
                    if !on_front && rotate_first {
                        dx = -dx;
                    }
                    let center = IntPoint::new(scale(x + dx), scale(y + dy));
                    let pin_ref = format!("{refdes}-{}", pin.pin_name);
                    let net_nos = pin_nets.get(&pin_ref).map(|n| vec![*n]).unwrap_or_default();
                    let (attach_allowed, smd) = board
                        .padstacks
                        .get_by_no(padstack_no)
                        // single-layer padstacks are SMD pads
                        .map(|p| (p.attach_allowed, p.from_layer() == p.to_layer()))
                        .unwrap_or((false, false));
                    // resolve the pin's clearance class. A per-pin
                    // `(clearance_class NAME)` override in the placement wins
                    // (Java Network.insert_component: pin_info.clearance_class);
                    // otherwise use the net class's per-item-type default
                    // (smd pad -> Smd, through-pin -> Pin), so a classed net's
                    // smd pad uses the class clearance while a plain net's smd
                    // pad stays on the tight smd class (#2).
                    let clearance_class = match pin_overrides.get(pin.pin_name.as_str()) {
                        Some(cc) => resolve_clearance_class(&mut board.rules, cc),
                        None => {
                            let item_class = if smd {
                                crate::rules::ItemClass::Smd
                            } else {
                                crate::rules::ItemClass::Pin
                            };
                            board.rules.item_clearance_class_for(
                                net_nos.first().copied().unwrap_or(0),
                                item_class,
                            )
                        }
                    };
                    let id = board.insert_via(
                        padstack_no,
                        center,
                        net_nos,
                        clearance_class,
                        attach_allowed,
                    );
                    // pins belong to their component: protected from ripup
                    // and not written to session files
                    board.set_component_no(id, 1);
                }
                // component keepouts (finding #1): instantiate each image
                // keepout at this placement, transformed exactly like the pins
                // (Java Package.Keepout via ObstacleArea.get_area).
                for ko in &image.keepouts {
                    use crate::geometry::planar::{FloatPoint, IntVector, PolylineArea};
                    let origin = IntPoint::new(0, 0);
                    let mut area = PolylineArea::new(ko.area.clone(), Vec::new());
                    // default flip style mirrors BEFORE the rotation
                    if !on_front && !rotate_first {
                        area = area.mirror_vertical(origin);
                    }
                    if quarter != 0 {
                        area = area.turn_90_degree(quarter, origin);
                    } else if rotation != 0.0 {
                        area = area.rotate_approx(rotation, FloatPoint::new(0.0, 0.0));
                    }
                    if !on_front && rotate_first {
                        area = area.mirror_vertical(origin);
                    }
                    area = area.translate_by(IntVector::new(scale(x), scale(y)));
                    // back-side keepout on a named layer flips to the mirror layer
                    let layers: Vec<usize> = match ko.layer {
                        Some(l) => vec![if on_front { l } else { layer_count - 1 - l }],
                        None => (0..layer_count).collect(),
                    };
                    for layer in layers {
                        let mut item = crate::board::Item::new_obstacle_area(
                            crate::board::ItemBase::new(0, Vec::new(), 1),
                            area.clone(),
                            layer,
                            "",
                            false,
                        );
                        if ko.via_only {
                            if let crate::board::ItemKind::ObstacleArea(a) = &mut item.kind {
                                a.via_only = true;
                            }
                        }
                        let id = board.insert_item(item);
                        board.set_fixed_state(id, crate::board::FixedState::SystemFixed);
                    }
                }
            }
        }
    }
    // wiring: pre-routed wires and vias (KiCad exports mark existing
    // routes with (type protect))
    for wiring in pcb.children("wiring") {
        for wire_node in wiring.children("wire") {
            // (path <layer> <width> x y ...) lists corners;
            // (polyline_path <layer> <width> x1 y1 x2 y2 ...) lists groups
            // of four numbers, each a line through two points — corners are
            // the intersections of consecutive lines (Java PolylinePath,
            // freerouting's own non-compat wiring export format).
            let (path, is_polyline_path) = match wire_node.child("path") {
                Some(p) => (p, false),
                None => match wire_node.child("polyline_path") {
                    Some(p) => (p, true),
                    None => continue,
                },
            };
            let Some(layer) = path.arg().and_then(|n| board.layer_structure.get_no(n)) else {
                continue;
            };
            let nums: Vec<f64> = path.args().skip(1).filter_map(|a| a.parse().ok()).collect();
            if nums.len() < 5 {
                continue;
            }
            let half_width = (scale(nums[0]) / 2).max(1);
            let net_nos = wire_node
                .child("net")
                .and_then(|n| n.arg())
                .and_then(|name| {
                    board
                        .rules
                        .nets
                        .get_by_name(name)
                        .first()
                        .map(|n| n.net_number)
                })
                .map(|n| vec![n])
                .unwrap_or_default();
            let protected = wire_node
                .child("type")
                .and_then(|t| t.arg())
                .is_some_and(|t| {
                    t.eq_ignore_ascii_case("protect") || t.eq_ignore_ascii_case("fix")
                });
            let polyline = if is_polyline_path {
                let lines: Vec<crate::geometry::planar::Line> = nums[1..]
                    .chunks_exact(4)
                    .filter_map(|c| {
                        let a = IntPoint::new(scale(c[0]), scale(c[1]));
                        let b = IntPoint::new(scale(c[2]), scale(c[3]));
                        (a != b).then(|| crate::geometry::planar::Line::new(a, b))
                    })
                    .collect();
                crate::geometry::planar::Polyline::from_lines(lines)
            } else {
                let corners: Vec<IntPoint> = nums[1..]
                    .chunks_exact(2)
                    .map(|c| IntPoint::new(scale(c[0]), scale(c[1])))
                    .collect();
                crate::geometry::planar::Polyline::from_int_points(&corners)
            };
            if polyline.is_empty() {
                continue;
            }
            // pre-routed wiring keeps its net's clearance class, not the default
            let clearance_class = net_nos
                .first()
                .map(|&n| board.rules.get_trace_clearance_class(n))
                .unwrap_or_else(BoardRules::default_clearance_class);
            let id = board.insert_trace(polyline, layer, half_width, net_nos, clearance_class);
            if protected {
                board.set_fixed_state(id, crate::board::FixedState::UserFixed);
            }
        }
        for via_node in wiring.children("via") {
            let args: Vec<&str> = via_node.args().collect();
            if args.len() < 3 {
                continue;
            }
            let Some(&padstack_no) = padstack_nos.get(args[0]) else {
                continue;
            };
            let (Ok(x), Ok(y)) = (args[1].parse::<f64>(), args[2].parse::<f64>()) else {
                continue;
            };
            let net_nos = via_node
                .child("net")
                .and_then(|n| n.arg())
                .and_then(|name| {
                    board
                        .rules
                        .nets
                        .get_by_name(name)
                        .first()
                        .map(|n| n.net_number)
                })
                .map(|n| vec![n])
                .unwrap_or_default();
            let protected = via_node
                .child("type")
                .and_then(|t| t.arg())
                .is_some_and(|t| {
                    t.eq_ignore_ascii_case("protect") || t.eq_ignore_ascii_case("fix")
                });
            let clearance_class = net_nos
                .first()
                .map(|&n| board.rules.get_trace_clearance_class(n))
                .unwrap_or_else(BoardRules::default_clearance_class);
            // Java Wiring.read_via_scope: attach_allowed =
            // via_at_smd_allowed && padstack.attach_allowed
            let attach = board.rules.via_at_smd_allowed
                && board
                    .padstacks
                    .get_by_no(padstack_no)
                    .is_some_and(|p| p.attach_allowed);
            let id = board.insert_via(
                padstack_no,
                IntPoint::new(scale(x), scale(y)),
                net_nos,
                clearance_class,
                attach,
            );
            if protected {
                board.set_fixed_state(id, crate::board::FixedState::UserFixed);
            }
        }
    }
    // Register the contacts of pre-routed vias landing mid-trace: a
    // contact needs a trace ENDPOINT at the pad, so split the same-net
    // traces at each routing via's center (raw insert_via does not).
    let via_ids: Vec<crate::board::basic_board::ItemId> = board
        .items()
        .filter(|(_, it)| {
            it.base.component_no == 0 && matches!(it.kind, crate::board::ItemKind::Via(_))
        })
        .map(|(id, _)| *id)
        .collect();
    for id in via_ids {
        board.split_traces_at_via(id);
    }
    Ok(board)
}

/// Inserts thin keepout strips along the closed outline given by
/// `corners` on every layer, so routes cannot cross the board boundary
/// (Java: the tree shapes of `BoardOutline`).
fn insert_boundary_keepouts(board: &mut BasicBoard, corners: &[IntPoint], half_width: i32) {
    use crate::geometry::planar::{PolygonShape, PolylineArea};
    if corners.len() < 2 {
        return;
    }
    let half_width = half_width.max(1);
    let layer_count = board.layer_structure.layer_count();
    let mut edges: Vec<(IntPoint, IntPoint)> = corners
        .windows(2)
        .map(|w| (w[0], w[1]))
        .filter(|(a, b)| a != b)
        .collect();
    // close the outline if the file did not repeat the first corner
    if corners.first() != corners.last() {
        edges.push((*corners.last().unwrap(), corners[0]));
    }
    for (a, b) in edges {
        // a thin rectangle strip around the edge
        let line = crate::geometry::planar::FloatLine::new(a.to_float(), b.to_float());
        let left = line.translate(half_width as f64);
        let right = line.translate(-(half_width as f64));
        let strip = PolygonShape::from_int_points(&[
            left.a.round(),
            left.b.round(),
            right.b.round(),
            right.a.round(),
        ]);
        if strip.dimension() < 2 {
            continue;
        }
        let area = PolylineArea::new(strip, vec![]);
        for layer in 0..layer_count {
            board.insert_area(area.clone(), layer, "boundary", vec![], 1, false);
        }
    }
}

/// The polygon corners of a keepout shape node: polygons stay exact,
/// rects become their four corners, circles and paths their bounding
/// octagon corners.
fn keepout_corners(
    node: &SExpr,
    scale: &dyn Fn(f64) -> i32,
) -> Vec<crate::geometry::planar::Point> {
    use crate::geometry::planar::Point;
    let kind = node.name().unwrap_or("");
    let nums: Vec<f64> = node.args().skip(1).filter_map(|a| a.parse().ok()).collect();
    let octagon_corners = |oct: IntOctagon| -> Vec<Point> {
        let t = TileShape::Octagon(oct.normalize());
        (0..t.border_line_count()).map(|i| t.corner(i)).collect()
    };
    if kind.eq_ignore_ascii_case("polygon") {
        // (polygon LAYER aperture x1 y1 ...)
        nums[1..]
            .chunks_exact(2)
            .map(|c| Point::Int(IntPoint::new(scale(c[0]), scale(c[1]))))
            .collect()
    } else if kind.eq_ignore_ascii_case("rect") && nums.len() >= 4 {
        let (x0, y0) = (scale(nums[0].min(nums[2])), scale(nums[1].min(nums[3])));
        let (x1, y1) = (scale(nums[0].max(nums[2])), scale(nums[1].max(nums[3])));
        vec![
            Point::Int(IntPoint::new(x0, y0)),
            Point::Int(IntPoint::new(x1, y0)),
            Point::Int(IntPoint::new(x1, y1)),
            Point::Int(IntPoint::new(x0, y1)),
        ]
    } else if kind.eq_ignore_ascii_case("circle") && !nums.is_empty() {
        let cx = nums.get(1).copied().unwrap_or(0.0);
        let cy = nums.get(2).copied().unwrap_or(0.0);
        let circle = Circle::new(
            IntPoint::new(scale(cx), scale(cy)),
            scale(nums[0] / 2.0).max(1),
        );
        octagon_corners(circle.bounding_octagon())
    } else if kind.eq_ignore_ascii_case("path") && nums.len() >= 5 {
        let radius = nums[0] / 2.0;
        let mut oct: Option<IntOctagon> = None;
        for pair in nums[1..].chunks_exact(2) {
            let c = Circle::new(
                IntPoint::new(scale(pair[0]), scale(pair[1])),
                scale(radius).max(1),
            );
            let o = c.bounding_octagon();
            oct = Some(match oct {
                Some(prev) => prev.union(o),
                None => o,
            });
        }
        oct.map(octagon_corners).unwrap_or_default()
    } else {
        Vec::new()
    }
}

/// Reads a single pad shape node (circle / rect / path / polygon),
/// returning the tile shape (relative to the pad center) and its layer
/// name.
fn read_pad_shape(node: &SExpr, scale: &dyn Fn(f64) -> i32) -> Option<(TileShape, String)> {
    let kind = node.name()?;
    let layer_name = node.arg()?.to_string();
    let nums: Vec<f64> = node.args().skip(1).filter_map(|a| a.parse().ok()).collect();
    let shape = if kind.eq_ignore_ascii_case("circle") {
        // (circle LAYER diameter [cx cy])
        let diameter = *nums.first()?;
        let cx = nums.get(1).copied().unwrap_or(0.0);
        let cy = nums.get(2).copied().unwrap_or(0.0);
        let circle = Circle::new(
            IntPoint::new(scale(cx), scale(cy)),
            scale(diameter / 2.0).max(1),
        );
        circle.bounding_tile()
    } else if kind.eq_ignore_ascii_case("rect") {
        // (rect LAYER x1 y1 x2 y2)
        if nums.len() < 4 {
            return None;
        }
        TileShape::Box(IntBox::from_coords(
            scale(nums[0].min(nums[2])),
            scale(nums[1].min(nums[3])),
            scale(nums[0].max(nums[2])),
            scale(nums[1].max(nums[3])),
        ))
    } else if kind.eq_ignore_ascii_case("path") {
        // (path LAYER width x1 y1 x2 y2 ...): an oval / thick segment,
        // approximated by the union of the corner circles' bounding
        // octagons. (A plain bounding box kept the sharp corners and
        // strangled the routing corridors between neighbouring pads.)
        if nums.len() < 5 {
            return None;
        }
        let radius = nums[0] / 2.0;
        let mut oct: Option<IntOctagon> = None;
        for pair in nums[1..].chunks_exact(2) {
            let c = Circle::new(
                IntPoint::new(scale(pair[0]), scale(pair[1])),
                scale(radius).max(1),
            );
            let o = c.bounding_octagon();
            oct = Some(match oct {
                Some(prev) => prev.union(o),
                None => o,
            });
        }
        TileShape::Octagon(oct?)
    } else if kind.eq_ignore_ascii_case("polygon") {
        // (polygon LAYER aperture x1 y1 ...): bounding box approximation
        if nums.len() < 5 {
            return None;
        }
        let xs: Vec<f64> = nums[1..].iter().step_by(2).copied().collect();
        let ys: Vec<f64> = nums[2..].iter().step_by(2).copied().collect();
        TileShape::Box(IntBox::from_coords(
            scale(xs.iter().cloned().fold(f64::MAX, f64::min)),
            scale(ys.iter().cloned().fold(f64::MAX, f64::min)),
            scale(xs.iter().cloned().fold(f64::MIN, f64::max)),
            scale(ys.iter().cloned().fold(f64::MIN, f64::max)),
        ))
    } else {
        return None;
    };
    Some((shape, layer_name))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn per_net_class_clearance_becomes_a_distinct_class() {
        // two net classes: "sig" at the board default clearance, "hv" larger.
        let dsn = r#"(pcb "cc.dsn"
  (resolution um 10)
  (structure
    (layer F.Cu (type signal))
    (layer B.Cu (type signal))
    (boundary (rect pcb 0 0 100000 100000))
    (rule (width 200) (clearance 200))
  )
  (placement)
  (library)
  (network
    (net "SIG1")
    (net "HV1")
    (class sig "SIG1" (rule (width 200) (clearance 200)))
    (class hv "HV1" (rule (width 400) (clear 800)))
  )
  (wiring
    (wire (path F.Cu 400 10000 10000 20000 20000) (net "HV1") (type route))
  )
)"#;
        let board = import_dsn(dsn).expect("import");
        let sig = board.rules.nets.get_by_name("SIG1")[0].net_number;
        let hv = board.rules.nets.get_by_name("HV1")[0].net_number;
        let sig_cc = board.rules.get_trace_clearance_class(sig);
        let hv_cc = board.rules.get_trace_clearance_class(hv);
        // the larger-clearance net class must get its OWN clearance class
        assert_ne!(
            sig_cc, hv_cc,
            "hv net class must not reuse the default class"
        );
        // and that class must carry the hv spacing (800 um * resolution 10)
        assert_eq!(
            board
                .rules
                .clearance_matrix
                .get_value(hv_cc, hv_cc, 0, false),
            8000,
            "hv clearance class value"
        );
        // the sig net class carries a clearance rule (value == board default);
        // per Java it STILL gets its own clearance class (so its pins/smd use
        // the class, not the tighter smd class), whose value equals the default.
        assert_ne!(sig_cc, BoardRules::default_clearance_class());
        assert_ne!(sig_cc, hv_cc);
        assert_eq!(
            board
                .rules
                .clearance_matrix
                .get_value(sig_cc, sig_cc, 0, false),
            2000,
            "sig clearance class carries the default value"
        );
        // the pre-routed HV1 wire inherits the hv clearance class, not the default
        let wire_cc = board
            .items()
            .find_map(|(_, it)| match &it.kind {
                crate::board::ItemKind::PolylineTrace(_) if it.base.contains_net(hv) => {
                    Some(it.base.clearance_class)
                }
                _ => None,
            })
            .expect("pre-routed HV1 wire");
        assert_eq!(
            wire_cc, hv_cc,
            "wiring must propagate the net's clearance class"
        );
    }

    #[test]
    fn imports_keepout_areas() {
        let dsn = r#"(pcb "k.dsn"
  (resolution um 10)
  (structure
    (layer F.Cu (type signal))
    (layer B.Cu (type signal))
    (boundary (rect pcb 0 0 100000 100000))
    (keepout "cutout" (polygon F.Cu 0 40000 40000 60000 40000 60000 60000 40000 60000))
    (via_keepout (rect signal 10000 10000 20000 20000))
    (rule (width 200) (clearance 200))
  )
  (placement)
  (library)
  (network (net "N1"))
)"#;
        let board = import_dsn(dsn).expect("import");
        use crate::board::ItemKind;
        let keepouts: Vec<_> = board
            .items()
            .filter_map(|(_, it)| match &it.kind {
                ItemKind::ObstacleArea(a) if !a.is_conduction && a.name != "boundary" => {
                    Some((a.layer, a.via_only))
                }
                _ => None,
            })
            .collect();
        // the full keepout sits on F.Cu only; the via keepout ("signal")
        // lands on both layers
        assert_eq!(
            keepouts.iter().filter(|(_, via_only)| !via_only).count(),
            1,
            "one full keepout"
        );
        assert_eq!(
            keepouts.iter().filter(|(_, via_only)| *via_only).count(),
            2,
            "via keepout on both signal layers"
        );
        // the rect boundary must produce confining keepout strips (previously
        // only `path` boundaries did, leaving rect-outline boards unconfined)
        let boundary_strips = board
            .items()
            .filter(|(_, it)| matches!(&it.kind, ItemKind::ObstacleArea(a) if a.name == "boundary"))
            .count();
        assert!(
            boundary_strips > 0,
            "rect boundary must produce confining keepouts"
        );
    }

    #[test]
    fn imports_component_image_keepouts() {
        // a keepout declared INSIDE an (image ...) is instantiated as an
        // obstacle area at each placement of that image (finding #1).
        let dsn = r#"(pcb "ik.dsn"
  (resolution um 10)
  (structure
    (layer F.Cu (type signal))
    (layer B.Cu (type signal))
    (boundary (rect pcb 0 0 200000 200000))
    (rule (width 200) (clearance 200))
  )
  (placement
    (component "FP" (place C1 50000 50000 front 0))
  )
  (library
    (image "FP"
      (pin "Pad" 1 0 0)
      (keepout "" (polygon F.Cu 0 -3000 -3000 3000 -3000 3000 3000 -3000 3000))
    )
    (padstack "Pad" (shape (rect F.Cu -500 -500 500 500)) (attach off))
  )
  (network (net "N1" (pins C1-1)))
)"#;
        let board = import_dsn(dsn).expect("import");
        use crate::board::ItemKind;
        let keepouts = board
            .items()
            .filter(|(_, it)| {
                matches!(&it.kind, ItemKind::ObstacleArea(a) if !a.is_conduction && a.name != "boundary")
            })
            .count();
        assert_eq!(
            keepouts, 1,
            "the image keepout must be instantiated at the placement"
        );
    }

    #[test]
    fn classed_net_smd_pad_uses_class_clearance() {
        // A named class whose clearance equals the board default still gives its
        // SMD pads the class clearance (Java default_item_clearance_classes.
        // set_all), while a PLAIN net's SMD pad stays on the tight smd class.
        let dsn = r#"(pcb "s.dsn"
  (resolution um 10)
  (structure
    (layer F.Cu (type signal))
    (boundary (rect pcb 0 0 200000 200000))
    (rule (width 200) (clearance 200) (clearance 50 (type smd_smd)))
  )
  (placement
    (component "SMD" (place C1 50000 50000 front 0))
    (component "SMD" (place C2 80000 50000 front 0))
  )
  (library
    (image "SMD" (pin "Pad" 1 0 0))
    (padstack "Pad" (shape (rect F.Cu -500 -500 500 500)) (attach off))
  )
  (network
    (net "CLASSED" (pins C1-1))
    (net "PLAIN" (pins C2-1))
    (class hv "CLASSED" (rule (width 200) (clearance 200)))
  )
)"#;
        let board = import_dsn(dsn).expect("import");
        use crate::board::ItemKind;
        let pad_cc = |net_name: &str| -> usize {
            let net = board.rules.nets.get_by_name(net_name)[0].net_number;
            board
                .items()
                .find_map(|(_, it)| match &it.kind {
                    ItemKind::Via(_) if it.base.component_no != 0 && it.base.contains_net(net) => {
                        Some(it.base.clearance_class)
                    }
                    _ => None,
                })
                .expect("pad")
        };
        let classed_cc = pad_cc("CLASSED");
        let plain_cc = pad_cc("PLAIN");
        let m = &board.rules.clearance_matrix;
        // plain net's smd pad -> smd class, value 500
        assert_eq!(
            m.get_value(plain_cc, plain_cc, 0, false),
            500,
            "plain net's smd pad keeps the smd clearance"
        );
        // classed net's smd pad -> the class clearance (2000), NOT smd (500)
        assert_ne!(classed_cc, plain_cc);
        assert_eq!(
            m.get_value(classed_cc, classed_cc, 0, false),
            2000,
            "classed net's smd pad uses the class clearance"
        );
    }

    #[test]
    fn named_and_item_clearance_classes() {
        // Part B: typed item-class pairs (pin_via, via_via), wire->default,
        // arbitrary named classes (power), a net-class `(clearance_class NAME)`
        // reference, and a per-pin `(clearance_class NAME)` override.
        let dsn = r#"(pcb "b.dsn"
  (resolution um 10)
  (structure
    (layer F.Cu (type signal))
    (layer B.Cu (type signal))
    (boundary (rect pcb 0 0 200000 200000))
    (rule
      (clearance 200)
      (clearance 300 (type pin_via))
      (clearance 400 (type via_via))
      (clearance 500 (type wire_via))
      (clearance 600 (type power_default))
      (clearance 70 (type via_via_same_net))
      (clearance 800 (type wire_kicad_default))
      (clearance 900 (type shield _via))
    )
  )
  (placement
    (component "FP"
      (place C1 50000 50000 front 0 (pin 1 (clearance_class power)))
    )
  )
  (library
    (image "FP" (pin "Pad" 1 0 0))
    (padstack "Pad" (shape (rect F.Cu -500 -500 500 500)) (attach off))
  )
  (network
    (net "N1" (pins C1-1))
    (class hv "N1" (clearance_class power) (rule (width 250)))
  )
)"#;
        let board = import_dsn(dsn).expect("import");
        let m = &board.rules.clearance_matrix;
        let via = m.get_no("via").expect("via class created");
        let pin = m
            .get_no("pin")
            .expect("pin class created (wire pair triggers it)");
        let power = m.get_no("power").expect("named power class created");
        // item-class pairs
        assert_eq!(m.get_value(pin, via, 0, false), 3000, "pin_via");
        assert_eq!(m.get_value(via, via, 0, false), 4000, "via_via");
        // wire maps to the default class (1)
        assert_eq!(
            m.get_value(1, via, 0, false),
            5000,
            "wire_via -> default_via"
        );
        // arbitrary named class paired with default
        assert_eq!(m.get_value(power, 1, 0, false), 6000, "power_default");
        // the SECOND name may itself contain underscores (Java split("_", 2))
        let kd = m
            .get_no("kicad_default")
            .expect("underscored second class name");
        assert_eq!(m.get_value(1, kd, 0, false), 8000, "wire_kicad_default");
        // two-token pair form `(type NAME1 _NAME2)` (Java's quoted-pair form)
        let shield = m.get_no("shield").expect("two-token pair class");
        assert_eq!(m.get_value(shield, via, 0, false), 9000, "shield _via");
        // a *_same_net rule is stored as a same-net clearance, not a matrix class
        use crate::rules::ItemClass;
        assert_eq!(
            board
                .rules
                .get_same_net_clearance(ItemClass::Via, ItemClass::Via),
            Some(700),
            "via_via_same_net stored"
        );
        assert!(
            m.get_no("via_same_net").is_none(),
            "no junk 'via_same_net' matrix class is created"
        );
        // net class hv references the named `power` class for its traces
        let n1 = board.rules.nets.get_by_name("N1")[0].net_number;
        assert_eq!(board.rules.get_trace_clearance_class(n1), power);
        // the per-pin override puts pin C1-1 on the `power` class
        use crate::board::ItemKind;
        let pin_cc = board
            .items()
            .find_map(|(_, it)| match &it.kind {
                ItemKind::Via(_) if it.base.component_no != 0 => Some(it.base.clearance_class),
                _ => None,
            })
            .expect("pad");
        assert_eq!(pin_cc, power, "per-pin (clearance_class power) override");
    }

    #[test]
    fn named_via_rule_reference() {
        // (via ...) + (via_rule ...) declarations, referenced by a net class.
        let dsn = r#"(pcb "v.dsn"
  (resolution um 10)
  (structure
    (layer F.Cu (type signal))
    (layer B.Cu (type signal))
    (boundary (rect pcb 0 0 200000 200000))
    (rule (width 200) (clearance 200))
  )
  (placement)
  (library
    (padstack "Via1"
      (shape (circle F.Cu 600 0 0))
      (shape (circle B.Cu 600 0 0))
      (attach off)
    )
  )
  (network
    (net "HV")
    (via "HVVia" "Via1" Power attach)
    (via "HVVia2" "Via1" Power)
    (via_rule Power "HVVia")
    (via_rule Power "HVVia2")
    (class hv "HV" (via_rule Power) (rule (width 250)))
  )
)"#;
        let board = import_dsn(dsn).expect("import");
        let hv = board.rules.nets.get_by_name("HV")[0].net_number;
        let class_idx = board.rules.nets.get_by_no(hv).unwrap().get_class();
        let rule_id = board
            .rules
            .net_classes
            .get(class_idx)
            .get_via_rule()
            .expect("net class binds the named via rule");
        let rule = &board.rules.via_rules[rule_id];
        assert_eq!(rule.name, "Power");
        // a redeclared rule REPLACES the first (Java add_via_rule) — no orphan
        assert_eq!(
            board
                .rules
                .via_rules
                .iter()
                .filter(|r| r.name == "Power")
                .count(),
            1,
            "duplicate via_rule declaration must replace, not accumulate"
        );
        let via_info = board.rules.via_infos.get(rule.vias()[0]);
        assert_eq!(via_info.get_name(), "HVVia2", "the redeclaration wins");
        // via-info attribute parsing is covered by the first declaration
        let first = board
            .rules
            .via_infos
            .get_by_name("HVVia")
            .expect("HVVia info");
        let first = board.rules.via_infos.get(first);
        assert!(first.attach_smd_allowed(), "attach token honored");
        let power = board
            .rules
            .clearance_matrix
            .get_no("Power")
            .expect("named Power clearance class");
        assert_eq!(first.get_clearance_class(), power);
    }

    #[test]
    fn imports_real_fixture() {
        let root = concat!(env!("CARGO_MANIFEST_DIR"), "/..");
        let path = format!("{root}/fixtures/Issue093-interf_u.dsn");
        let content = std::fs::read_to_string(&path).expect("fixture missing from checkout");
        let board = import_dsn(&content).expect("import failed");
        assert_eq!(board.layer_structure.layer_count(), 2);
        assert_eq!(board.layer_structure.arr[0].name, "top_copper");
        assert!(board.padstacks.count() > 10);
        assert!(board.rules.nets.max_net_no() > 10);
        // every pin landed as an item; the interf_u board has hundreds
        let pin_count = board
            .items()
            .filter(|(_, i)| matches!(i.kind, crate::board::ItemKind::Via(_)))
            .count();
        assert!(pin_count > 100, "only {pin_count} pins imported");
        // a known net exists and is not yet routed
        let ack = board
            .rules
            .nets
            .get_by_name("/ACK")
            .first()
            .map(|n| n.net_number)
            .expect("/ACK net missing");
        let ack_items: Vec<_> = board
            .items()
            .filter(|(_, item)| item.base.contains_net(ack))
            .collect();
        // 2 component pins plus the fixture's 4 pre-routed wires
        let pins = ack_items
            .iter()
            .filter(|(_, i)| i.base.component_no != 0)
            .count();
        assert_eq!(pins, 2);
        assert!(ack_items.len() >= 6, "wiring not imported");
        // pins carry shapes usable by the search tree
        let (_, item) = ack_items[0];
        assert!(item.tile_shape_count(&board.padstacks) >= 1);
    }

    #[test]
    fn imports_empty_board_with_boundary_keepouts() {
        use crate::board::ItemKind;
        let root = concat!(env!("CARGO_MANIFEST_DIR"), "/..");
        let path = format!("{root}/fixtures/empty_board.dsn");
        let content = std::fs::read_to_string(&path).expect("fixture missing from checkout");
        let board = import_dsn(&content).expect("import failed");
        assert_eq!(board.layer_structure.layer_count(), 2);
        // no pins, but the boundary keepout strips are present
        assert!(board.item_count() > 0);
        assert!(board
            .items()
            .all(|(_, i)| matches!(i.kind, ItemKind::ObstacleArea(_))));
        // the boundary blocks any net at the outline (coordinates from the
        // file, scaled by resolution 10): the left border is x = 1295400
        let on_border = TileShape::Box(IntBox::from_coords(1295300, -800000, 1295500, -799000));
        assert!(board.is_blocked(&on_border, 0, 1));
        // but the interior is free
        let inside = TileShape::Box(IntBox::from_coords(1500000, -800000, 1500200, -799800));
        assert!(!board.is_blocked(&inside, 0, 1));
    }

    #[test]
    fn imports_prerouted_wiring() {
        use crate::board::{FixedState, ItemKind};
        let root = concat!(env!("CARGO_MANIFEST_DIR"), "/..");
        let path = format!("{root}/fixtures/Issue027-zMRETestFixture.dsn");
        let content = std::fs::read_to_string(&path).expect("fixture missing from checkout");
        let board = import_dsn(&content).expect("import failed");
        // the fixture's protected pre-routed wires became fixed traces
        let protected_traces: Vec<_> = board
            .items()
            .filter(|(_, i)| {
                matches!(i.kind, ItemKind::PolylineTrace(_))
                    && i.base.fixed_state == FixedState::UserFixed
            })
            .collect();
        assert!(
            protected_traces.len() > 10,
            "only {} protected traces imported",
            protected_traces.len()
        );
        // they carry their nets
        assert!(protected_traces.iter().all(|(_, i)| i.base.net_count() > 0));
        // fixed traces are not routable (protected from ripup)
        assert!(protected_traces.iter().all(|(_, i)| !i.is_routable()));
    }

    #[test]
    fn imports_polyline_path_wiring() {
        use crate::board::ItemKind;
        let root = concat!(env!("CARGO_MANIFEST_DIR"), "/..");
        let path = format!("{root}/fixtures/Issue187-processor.Z80.dsn");
        let content = std::fs::read_to_string(&path).expect("fixture missing from checkout");
        let board = import_dsn(&content).expect("import failed");
        // the fixture's routing is stored exclusively as (polyline_path ...)
        // wires: 2026 of them, all carrying nets
        let traces: Vec<_> = board
            .items()
            .filter(|(_, i)| matches!(i.kind, ItemKind::PolylineTrace(_)))
            .collect();
        assert!(traces.len() > 1900, "only {} wires imported", traces.len());
        assert!(traces.iter().all(|(_, i)| i.base.net_count() > 0));
    }

    #[test]
    fn rejects_non_pcb() {
        assert!(import_dsn("(session x)").is_err());
        assert!(import_dsn("(pcb x)").is_err()); // no structure
    }
}
