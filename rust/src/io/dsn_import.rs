//! Semantic DSN import (port of the reading half of
//! `io/specctra/DsnReader.java` and the Structure/Library/Network scope
//! classes, simplified): maps a parsed DSN tree onto a [`BasicBoard`]
//! with layers, padstacks, placed component pins and nets.
//!
//! Simplifications (documented): pad shapes are converted to boxes /
//! bounding octagons (circles/ovals); back-side placement mirrors pin
//! offsets and pad shapes at the y axis and flips the shape layers;
//! non-quarter-turn rotations only rotate pin offsets, not pad shapes.

use std::cell::RefCell;
use std::collections::{BTreeMap, BTreeSet, HashMap};

use crate::board::basic_board::{BasicBoard, LogicalEndpoint};
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

/// Applies one `*_same_net` token to the item-class DRC table.  These tokens
/// are consumed before composite clearance-pair expansion so they cannot be
/// interpreted as (or counted alongside) ordinary matrix class pairs.
fn apply_same_net_type_token(rules: &mut BoardRules, token: &str, value: i32) -> bool {
    let normalized = token.to_ascii_lowercase().replace('-', "_");
    let Some(base) = normalized.strip_suffix("_same_net") else {
        return false;
    };
    let Some((first, second)) = base.split_once('_') else {
        return false;
    };
    let (Some(first_class), Some(second_class)) = (item_class_of(first), item_class_of(second))
    else {
        return false;
    };
    rules.set_same_net_clearance(first_class, second_class, value);
    true
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
    let item = match lname.as_str() {
        "via" => Some(crate::rules::ItemClass::Via),
        "pin" => Some(crate::rules::ItemClass::Pin),
        "smd" => Some(crate::rules::ItemClass::Smd),
        "area" => Some(crate::rules::ItemClass::Area),
        _ => None,
    };
    let idx = if let Some(idx) = rules.clearance_matrix.get_no(name) {
        idx
    } else {
        rules.clearance_matrix.append_class(name);
        rules
            .clearance_matrix
            .get_no(name)
            .unwrap_or_else(BoardRules::default_clearance_class)
    };
    // Bind standard item columns whenever they are semantically resolved.
    // The rules reader may have created the matrix symbol earlier solely to
    // satisfy a forward via declaration.
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

/// Looks up a clearance class in a reference position. Unlike a typed rule
/// declaration, an item/class reference must not invent a matrix column when
/// the name is unknown; callers apply the Java-compatible fallback.
fn lookup_clearance_class(rules: &BoardRules, name: &str) -> Option<usize> {
    if let Some(index) = rules.clearance_matrix.get_no(name) {
        return Some(index);
    }
    match name.to_ascii_lowercase().as_str() {
        "wire" | "default" => Some(BoardRules::default_clearance_class()),
        "null" => Some(BoardRules::clearance_class_none()),
        _ => None,
    }
}

/// Extracts one syntactically valid `(clearance_class NAME)` reference.
/// Resolution is deliberately separate because Java's fallback is owned by
/// the containing scope: structure geometry uses `null`, while network,
/// placement and wiring scopes retain their inherited class.
fn clearance_class_reference<'a>(
    node: &'a SExpr,
    context: &str,
) -> Result<Option<&'a str>, ImportError> {
    let Some(scope) = node.child("clearance_class") else {
        return Ok(None);
    };
    let mut args = scope.args();
    let name = args
        .next()
        .ok_or_else(|| err(format!("{context} has an empty clearance_class reference")))?;
    if args.next().is_some() {
        return Err(err(format!(
            "{context} clearance_class must name exactly one class"
        )));
    }
    Ok(Some(name))
}

fn structure_clearance_class(
    rules: &BoardRules,
    node: &SExpr,
    context: &str,
) -> Result<Option<usize>, ImportError> {
    Ok(clearance_class_reference(node, context)?.map(|name| {
        lookup_clearance_class(rules, name).unwrap_or_else(BoardRules::clearance_class_none)
    }))
}

fn inherited_clearance_class(
    rules: &BoardRules,
    node: &SExpr,
    context: &str,
) -> Result<Option<usize>, ImportError> {
    Ok(clearance_class_reference(node, context)?
        .and_then(|name| lookup_clearance_class(rules, name)))
}

/// Resolves a layer token used by a routing keepout.  Only the three
/// Specctra wildcard spellings expand to all layers; an arbitrary unknown
/// name must not be treated as a wildcard.
fn keepout_layer_reference(
    layer_structure: &LayerStructure,
    name: &str,
    context: &str,
) -> Result<Option<usize>, ImportError> {
    if let Some(layer) = layer_structure.get_no(name) {
        return Ok(Some(layer));
    }
    if name.eq_ignore_ascii_case("signal")
        || name.eq_ignore_ascii_case("all")
        || name.eq_ignore_ascii_case("pcb")
    {
        return Ok(None);
    }
    Err(err(format!("{context} references unknown layer {name:?}")))
}

/// Validates the symbol and layer references of one library padstack before
/// `read_padstack_scope` builds its sparse shape vector.  The low-level
/// reader is shared with sidecar rules and intentionally returns `Option`;
/// the public DSN boundary must distinguish an unsupported shape or unknown
/// layer from a deliberately absent shape on another layer.
fn validate_padstack_scope_references(
    padstack_node: &SExpr,
    layer_structure: &LayerStructure,
) -> Result<(), ImportError> {
    let name = padstack_node
        .arg()
        .ok_or_else(|| err("padstack without name"))?;
    for shape_scope in padstack_node.children("shape") {
        let shape = shape_scope
            .as_list()
            .and_then(|items| items.get(1))
            .ok_or_else(|| err(format!("padstack {name:?} has a shape without geometry")))?;
        let kind = shape
            .name()
            .ok_or_else(|| err(format!("padstack {name:?} has unnamed shape geometry")))?;
        if !matches!(
            kind.to_ascii_lowercase().as_str(),
            "circle" | "rect" | "path" | "polygon"
        ) {
            return Err(err(format!(
                "padstack {name:?} uses unsupported shape {kind:?}"
            )));
        }
        let layer = shape
            .arg()
            .ok_or_else(|| err(format!("padstack {name:?} shape is missing its layer")))?;
        if layer_structure.get_no(layer).is_none()
            && !layer.eq_ignore_ascii_case("signal")
            && !layer.eq_ignore_ascii_case("all")
            && !layer.eq_ignore_ascii_case("pcb")
        {
            return Err(err(format!(
                "padstack {name:?} shape references unknown layer {layer:?}"
            )));
        }
    }
    Ok(())
}

/// Resolves a `(net NAME [SUBNET])` reference.  A missing/zero subnet means
/// "all subnets" in wiring and SES scopes; a positive subnet selects exactly
/// that electrical subnet.  Keeping this in one helper prevents the design,
/// wiring, and session readers from silently collapsing repeated net names.
pub(crate) fn net_numbers_for_reference(
    rules: &BoardRules,
    net_node: &SExpr,
) -> Result<Vec<i32>, String> {
    let mut args = net_node.args();
    let Some(name) = args.next() else {
        return Err("net reference is missing its name".into());
    };
    let subnet = match args.next() {
        None => None,
        Some(value) => {
            let subnet = value
                .parse::<usize>()
                .map_err(|_| format!("net reference subnet {value:?} is not an integer"))?;
            Some(subnet)
        }
    };
    if args.next().is_some() {
        return Err("net reference has too many arguments".into());
    }
    let result = match subnet.filter(|value| *value > 0) {
        Some(subnet) => rules
            .nets
            .get(name, subnet)
            .map(|net| vec![net.net_number])
            .unwrap_or_default(),
        None => rules
            .nets
            .get_by_name(name)
            .into_iter()
            .map(|net| net.net_number)
            .collect(),
    };
    if result.is_empty() {
        return Err(format!("net reference names unknown net {name:?}"));
    }
    Ok(result)
}

fn append_unique_net(target: &mut Vec<i32>, net_no: i32) {
    if !target.contains(&net_no) {
        target.push(net_no);
    }
}

/// Parses the structural `component-pin` identities in a Specctra pin scope.
/// Quoted halves remain separate, so `"A-B"-C` cannot collide with
/// `A-"B-C"`. For an unquoted token Java stops the component at the first
/// hyphen and leaves any later hyphens in the pin name.
fn pin_reference_args(scope: &SExpr) -> Result<Vec<LogicalEndpoint>, ImportError> {
    let tokens: Vec<(&str, bool)> = scope
        .as_list()
        .unwrap_or(&[])
        .iter()
        .skip(1)
        .filter_map(|item| item.as_atom().map(|atom| (atom, item.is_quoted())))
        .collect();
    let mut result = Vec::new();
    let mut index = 0usize;
    while index < tokens.len() {
        let (component, pin, consumed) = if tokens[index].1 {
            if index + 2 < tokens.len() && tokens[index + 1].0 == "-" {
                (tokens[index].0, tokens[index + 2].0, 3)
            } else if index + 1 < tokens.len() && tokens[index + 1].0.starts_with('-') {
                (tokens[index].0, &tokens[index + 1].0[1..], 2)
            } else {
                return Err(err(format!(
                    "{} contains quoted component {:?} without a pin separator",
                    scope.name().unwrap_or("pin scope"),
                    tokens[index].0
                )));
            }
        } else if index + 2 < tokens.len() && tokens[index + 1].0 == "-" {
            (tokens[index].0, tokens[index + 2].0, 3)
        } else if index + 1 < tokens.len() && tokens[index + 1].1 && tokens[index].0.ends_with('-')
        {
            (
                tokens[index].0.strip_suffix('-').unwrap_or_default(),
                tokens[index + 1].0,
                2,
            )
        } else if let Some((component, pin)) = tokens[index].0.split_once('-') {
            (component, pin, 1)
        } else {
            return Err(err(format!(
                "{} pin reference {:?} is missing its component-pin separator",
                scope.name().unwrap_or("pin scope"),
                tokens[index].0
            )));
        };
        if component.is_empty() || pin.is_empty() {
            return Err(err(format!(
                "{} contains an empty component or pin name",
                scope.name().unwrap_or("pin scope")
            )));
        }
        result.push(LogicalEndpoint::new(component, pin));
        index += consumed;
    }
    Ok(result)
}

/// Parses one `(padstack NAME (shape ...) ... [(attach off)])` scope into
/// `padstacks`, returning the new padstack number (Java
/// `Library.read_padstack_scope`). Shared by the DSN importer and the
/// standard `.rules` reader — a rules sidecar may declare via padstacks
/// the design library lacks.
pub(crate) fn read_padstack_scope(
    padstacks: &mut Padstacks,
    layer_structure: &LayerStructure,
    scale: &dyn Fn(f64) -> i32,
    padstack_node: &SExpr,
) -> Option<usize> {
    let name = padstack_node.arg()?;
    let layer_count = layer_structure.layer_count();
    let mut shapes: Vec<Option<TileShape>> = vec![None; layer_count];
    for shape_node in padstack_node.children("shape") {
        let Some(inner) = shape_node.as_list().and_then(|l| l.get(1)) else {
            continue;
        };
        let Some((shape, layer_name)) = read_pad_shape(inner, scale) else {
            continue;
        };
        if let Some(l) = layer_structure.get_no(&layer_name) {
            shapes[l] = Some(shape);
        } else if layer_name.eq_ignore_ascii_case("signal") {
            // wildcard: the shape exists on every signal layer
            // (Java LayerStructure SIGNAL_LAYER handling)
            for (l, present) in shapes.iter_mut().enumerate() {
                if layer_structure.arr[l].is_signal {
                    *present = Some(shape.clone());
                }
            }
        } else if layer_name.eq_ignore_ascii_case("all") || layer_name.eq_ignore_ascii_case("pcb") {
            for present in shapes.iter_mut() {
                *present = Some(shape.clone());
            }
        }
    }
    // Java read_padstack_scope: attach defaults ON when the `(attach ...)`
    // scope is omitted; only an explicit `off` forbids attaching to SMD pads.
    let attach = padstack_node
        .child("attach")
        .and_then(|a| a.arg())
        .is_none_or(|v| !v.eq_ignore_ascii_case("off"));
    Some(padstacks.add(name, shapes, attach, false))
}

/// Shared context for applying network-scope rule nodes — `(via ...)`,
/// `(via_rule ...)` and `(class ...)` — used by the DSN importer and by
/// the standard `.rules` reader, which carries the same grammar (Java
/// RulesReader delegates to the same Network scope parsers).
pub(crate) struct NetworkScopeCtx<'a> {
    pub padstack_nos: &'a HashMap<String, usize>,
    pub padstacks: &'a Padstacks,
    pub layer_structure: &'a LayerStructure,
}

impl NetworkScopeCtx<'_> {
    /// Padstacks are named case-insensitively by the Specctra library. Keep
    /// the exact-name map as the fast path, then fall back to the library's
    /// canonical lookup for differently-cased references.
    pub(crate) fn padstack_no(&self, name: &str) -> Option<usize> {
        self.padstack_nos
            .get(name)
            .copied()
            .or_else(|| self.padstacks.get(name).map(|padstack| padstack.no))
    }
}

/// Applies one `(via NAME PADSTACK [CLEARANCE_CLASS] [attach])`
/// declaration (Java `Network.read_via_info`). Returns false when the
/// referenced padstack is unknown (the declaration cannot bind).
pub(crate) fn apply_via_declaration(
    rules: &mut BoardRules,
    ctx: &NetworkScopeCtx,
    via_node: &SExpr,
) -> bool {
    let mut a = via_node.args();
    let (Some(name), Some(padstack_name)) = (a.next(), a.next()) else {
        return false;
    };
    let Some(padstack_no) = ctx.padstack_no(padstack_name) else {
        return false;
    };
    let rest: Vec<&str> = a.collect();
    let attach = rest.iter().any(|t| t.eq_ignore_ascii_case("attach"));
    // The optional clearance class is the non-"attach" trailing token.  This
    // is an item attribute, not a clearance-class declaration: Java's
    // Network.read_via_info looks up the name and falls back to the default
    // class when it is unknown.  Creating a matrix column here would make a
    // later typed rule mutate a via that Java keeps on the default class.
    let cl = rest
        .iter()
        .find(|t| !t.eq_ignore_ascii_case("attach"))
        .map(|name| {
            lookup_clearance_class(rules, name).unwrap_or_else(BoardRules::default_clearance_class)
        })
        .unwrap_or_else(BoardRules::default_clearance_class);
    // a redeclared via info updates in place (names are unique in Java)
    if let Some(existing) = rules.via_infos.get_by_name(name) {
        let info = rules.via_infos.get_mut(existing);
        info.set_padstack(padstack_no);
        info.set_clearance_class(cl);
        info.set_attach_smd_allowed(attach);
    } else {
        let _ = rules
            .via_infos
            .add(crate::rules::ViaInfo::new(name, padstack_no, cl, attach));
    }
    true
}

/// Applies one `(via_rule NAME VIA...)` declaration (Java
/// `Network.read_via_rule`); a redeclared rule replaces the existing one
/// in place so indices bound to net classes stay valid.
pub(crate) fn apply_via_rule_declaration(
    rules: &mut BoardRules,
    via_rule_ids: &mut HashMap<String, usize>,
    rule_node: &SExpr,
) -> bool {
    let mut a = rule_node.args();
    let Some(rule_name) = a.next() else {
        return false;
    };
    let mut via_rule = crate::rules::ViaRule::new(rule_name);
    let mut all_found = true;
    let mut any_via = false;
    for via_name in a {
        any_via = true;
        if let Some(id) = rules.via_infos.get_by_name(via_name) {
            via_rule.append_via(id);
        } else {
            all_found = false;
        }
    }
    if all_found && any_via {
        if let Some(&existing) = via_rule_ids.get(rule_name) {
            rules.via_rules[existing] = via_rule;
        } else {
            rules.via_rules.push(via_rule);
            via_rule_ids.insert(rule_name.to_string(), rules.via_rules.len() - 1);
        }
        true
    } else {
        false
    }
}

/// Walks one `(rule ...)` scope's clearance children IN DOCUMENT ORDER
/// (`clearance` and `clear` interleaved as written, so later rules
/// overwrite earlier ones like Java's sequential reader). With
/// `apply_untyped`, an untyped clearance sets the whole-matrix default
/// (`Structure.set_clearance_rule` with no type pairs); typed rules apply
/// their class pairs, `smd_to_turn_gap` and `*_same_net` values. Class
/// names are NOT case-folded or hyphen-mutated: quoted names like "A-B"
/// stay intact (only classification keywords compare case-insensitively).
pub(crate) fn apply_rule_scope_clearances(
    rules: &mut BoardRules,
    rule_node: &SExpr,
    scale: &dyn Fn(f64) -> i32,
    apply_untyped: bool,
) -> usize {
    apply_rule_scope_clearances_on_layer(rules, rule_node, scale, apply_untyped, None)
}

/// As [`apply_rule_scope_clearances`], but restricts matrix writes to one
/// layer when `layer` is `Some`.  Keeping the traversal in one function is
/// important: DSN structure rules, layer rules, class rules, and `.rules`
/// sidecars all have the same ordered clearance grammar.
pub(crate) fn apply_rule_scope_clearances_on_layer(
    rules: &mut BoardRules,
    rule_node: &SExpr,
    scale: &dyn Fn(f64) -> i32,
    apply_untyped: bool,
    layer: Option<usize>,
) -> usize {
    let mut applied = 0usize;
    let children = rule_node.as_list().unwrap_or(&[]);
    for clearance_node in children.iter().filter(|c| {
        c.name()
            .is_some_and(|n| n.eq_ignore_ascii_case("clearance") || n.eq_ignore_ascii_case("clear"))
    }) {
        let Some(value) = clearance_node.arg_f64().map(scale) else {
            continue;
        };
        let Some(type_node) = clearance_node.child("type") else {
            if apply_untyped {
                // the untyped default applies to EVERY non-null class pair
                if let Some(layer_no) = layer {
                    rules
                        .clearance_matrix
                        .set_default_value_on_layer(layer_no, value);
                } else {
                    rules.clearance_matrix.set_default_value(value);
                }
                applied += 1;
            }
            continue; // untyped: in DSN import the caller applied it
        };
        // Create the standard item columns at the point the Java reader sees
        // a wire pair, rather than pre-scanning the whole rule (which would
        // let an earlier untyped clearance affect classes declared later).
        let raw_tokens: Vec<(String, bool)> = type_node
            .as_list()
            .unwrap_or(&[])
            .iter()
            .skip(1)
            .filter_map(|t| t.as_atom().map(|s| (s.to_string(), t.is_quoted())))
            .collect();
        if raw_tokens.iter().any(|(name, quoted)| {
            if *quoted {
                return false;
            }
            let lower = name.to_ascii_lowercase();
            lower.starts_with("wire_")
                || lower.starts_with("wire-")
                || lower.ends_with("_wire")
                || lower.ends_with("-wire")
        }) {
            for name in ["via", "smd", "pin", "area"] {
                resolve_clearance_class(rules, name);
            }
        }
        let mut pairs: Vec<(String, String)> = Vec::new();
        for (kind, quoted) in &raw_tokens {
            if *quoted {
                continue;
            }
            let lower = kind.to_ascii_lowercase().replace('-', "_");
            if lower == "smd_to_turn_gap" {
                rules.set_pin_edge_to_turn_dist(value as f64);
                applied += 1;
            } else if apply_same_net_type_token(rules, kind, value) {
                applied += 1;
            }
        }
        pairs.extend(type_pairs(type_node));
        for (a, b) in pairs {
            let ci = resolve_clearance_class(rules, &a);
            let cj = resolve_clearance_class(rules, &b);
            if let Some(layer_no) = layer {
                rules.clearance_matrix.set_value(ci, cj, layer_no, value);
                rules.clearance_matrix.set_value(cj, ci, layer_no, value);
            } else {
                rules
                    .clearance_matrix
                    .set_value_on_all_layers(ci, cj, value);
                rules
                    .clearance_matrix
                    .set_value_on_all_layers(cj, ci, value);
            }
            applied += 1;
        }
    }
    applied
}

/// Converts the lexical contents of a `(type ...)` node into composite class
/// pairs.  The parser retains whether each atom was quoted; this lets us
/// distinguish a quoted class called `A-B` from the unquoted legacy
/// `A-B`/`A_B` pair notation.
fn type_pairs(type_node: &SExpr) -> Vec<(String, String)> {
    let tokens: Vec<(String, bool)> = type_node
        .as_list()
        .unwrap_or(&[])
        .iter()
        .skip(1)
        .filter_map(|t| t.as_atom().map(|s| (s.to_string(), t.is_quoted())))
        .filter(|(s, quoted)| {
            *quoted
                || (!s.eq_ignore_ascii_case("smd_to_turn_gap")
                    && !s.to_ascii_lowercase().ends_with("_same_net")
                    && !s.to_ascii_lowercase().ends_with("-same_net"))
        })
        .collect();
    if tokens.len() == 3 && !tokens[1].1 && (tokens[1].0 == "_" || tokens[1].0 == "-") {
        return vec![(tokens[0].0.clone(), tokens[2].0.clone())];
    }
    if tokens.len() == 2 {
        let mut first = tokens[0].0.clone();
        let mut second = tokens[1].0.clone();
        if !tokens[0].1 {
            first = first.strip_suffix(['_', '-']).unwrap_or(&first).to_string();
        }
        if !tokens[1].1 {
            second = second
                .strip_prefix(['_', '-'])
                .unwrap_or(&second)
                .to_string();
        }
        return vec![(first, second)];
    }
    tokens
        .into_iter()
        .filter(|(_, quoted)| !quoted)
        .filter_map(|(token, _)| split_composite_token(&token))
        .collect()
}

fn rule_has_smd_to_turn_gap(rule_node: &SExpr) -> bool {
    rule_node
        .as_list()
        .unwrap_or(&[])
        .iter()
        .filter(|child| {
            child.name().is_some_and(|name| {
                name.eq_ignore_ascii_case("clearance") || name.eq_ignore_ascii_case("clear")
            })
        })
        .filter_map(|child| child.child("type"))
        .flat_map(|type_node| type_node.args())
        .any(|name| name.eq_ignore_ascii_case("smd_to_turn_gap"))
}

fn rule_has_clearance_declaration(rule_node: &SExpr) -> bool {
    rule_node.as_list().unwrap_or(&[]).iter().any(|child| {
        child.name().is_some_and(|name| {
            (name.eq_ignore_ascii_case("clearance") || name.eq_ignore_ascii_case("clear"))
                && child.arg_f64().is_some()
        })
    })
}

/// Splits the compact one-token spelling used by older Specctra writers.
/// Most pairs are unambiguous at the first `_`/`-` (`wire_kicad_default`),
/// but names themselves may contain underscores.  When both halves are the
/// same, prefer that midpoint (`base_a_base_a`) so the class name is not
/// truncated.  Quoted names and explicit separator tokens are handled by
/// `type_pairs` before this helper.
fn split_composite_token(token: &str) -> Option<(String, String)> {
    let mut separators: Vec<usize> = token
        .char_indices()
        .filter_map(|(index, ch)| matches!(ch, '_' | '-').then_some(index))
        .collect();
    separators.sort_unstable();
    for index in &separators {
        let (left, right_with_separator) = token.split_at(*index);
        let right = &right_with_separator[1..];
        if !left.is_empty() && !right.is_empty() && left.eq_ignore_ascii_case(right) {
            return Some((left.to_string(), right.to_string()));
        }
    }
    separators.first().and_then(|index| {
        let left = &token[..*index];
        let right = &token[*index + 1..];
        (!left.is_empty() && !right.is_empty()).then(|| (left.to_string(), right.to_string()))
    })
}

/// Resolves an item name in the namespace of one net class.  Java keeps
/// separate matrix columns for `class`, `class-via`, `class-pin`, etc.; using
/// the global `via`/`pin` columns here makes a class-scoped typed rule affect
/// unrelated nets.
fn ensure_class_item_clearance_class(
    rules: &mut BoardRules,
    net_class_idx: usize,
    item_name: &str,
    bind_items: bool,
) -> (usize, bool) {
    let class_name = rules.net_classes.get(net_class_idx).get_name().to_string();
    let is_wire =
        item_name.eq_ignore_ascii_case("wire") || item_name.eq_ignore_ascii_case("default");
    let matrix_name = if is_wire {
        class_name.clone()
    } else {
        format!("{class_name}-{item_name}")
    };
    let base = rules
        .net_classes
        .get(net_class_idx)
        .get_trace_clearance_class()
        .max(BoardRules::default_clearance_class());
    let (idx, created) = match rules.clearance_matrix.get_no(&matrix_name) {
        Some(idx) => (idx, false),
        None => {
            if !rules.clearance_matrix.append_class(&matrix_name) {
                return (
                    rules
                        .clearance_matrix
                        .get_no(&matrix_name)
                        .unwrap_or(BoardRules::default_clearance_class()),
                    false,
                );
            }
            let idx = rules
                .clearance_matrix
                .get_no(&matrix_name)
                .unwrap_or(BoardRules::default_clearance_class());
            let n = rules.clearance_matrix.get_class_count();
            let layers = rules.clearance_matrix.get_layer_count();
            for other in 1..n {
                for layer in 0..layers {
                    let value = rules.clearance_matrix.get_value(base, other, layer, false);
                    rules.clearance_matrix.set_value(idx, other, layer, value);
                    rules.clearance_matrix.set_value(other, idx, layer, value);
                }
            }
            (idx, true)
        }
    };
    if let Some(item_class) = item_class_of(&item_name.to_ascii_lowercase()) {
        // Java's `get_clearance_class` only updates a NetClass's per-item
        // defaults for via/pin/smd/area.  A mixed class-pair's `wire` column
        // is a matrix endpoint, not a request to rebind either class's trace
        // clearance (or its Trace item default).
        if bind_items {
            rules
                .net_classes
                .get_mut(net_class_idx)
                .default_item_clearance_classes
                .set(item_class, idx);
        }
    }
    if is_wire && bind_items {
        rules
            .net_classes
            .get_mut(net_class_idx)
            .set_trace_clearance_class(idx);
    }
    (idx, created)
}

/// Applies one class-local rule.  Unlike the board-level applier, every
/// typed class name is resolved in `net_class_idx`'s namespace and the layer
/// argument is honored.  Returns `(settings_applied, smd_gap_seen)`.
fn apply_class_rule_scope(
    rules: &mut BoardRules,
    net_class_idx: usize,
    rule_node: &SExpr,
    scale: &dyn Fn(f64) -> i32,
    layer: Option<usize>,
) -> (usize, bool) {
    let mut applied = 0;
    let mut gap_seen = false;
    let default_class = rules.get_default_net_class();
    for child in rule_node.as_list().unwrap_or(&[]).iter().filter(|c| {
        c.name()
            .is_some_and(|n| n.eq_ignore_ascii_case("clearance") || n.eq_ignore_ascii_case("clear"))
    }) {
        let Some(value) = child.arg_f64().map(scale) else {
            continue;
        };
        let (class_wire, class_wire_created) =
            ensure_class_item_clearance_class(rules, net_class_idx, "wire", true);
        if net_class_idx != default_class || class_wire_created {
            rules
                .net_classes
                .get_mut(net_class_idx)
                .set_trace_clearance_class(class_wire);
        }
        if net_class_idx != default_class && class_wire_created {
            // Java initializes every item class to the newly-created net
            // class, then lets typed pairs refine individual item columns.
            rules
                .net_classes
                .get_mut(net_class_idx)
                .default_item_clearance_classes
                .set_all(class_wire);
        }
        let Some(type_node) = child.child("type") else {
            let class_no = class_wire;
            // An untyped class clearance is the class's minimum spacing to
            // every other non-null class, not merely its diagonal.  Apply it
            // with max semantics so a class read later cannot weaken a
            // stricter pair that was already declared (Java's
            // Network.add_clearance_rule behavior).
            let class_count = rules.clearance_matrix.get_class_count();
            let layers = if let Some(layer_no) = layer {
                layer_no..layer_no + 1
            } else {
                0..rules.clearance_matrix.get_layer_count()
            };
            for other in 1..class_count {
                for layer_no in layers.clone() {
                    if other == class_no {
                        rules
                            .clearance_matrix
                            .set_value(class_no, class_no, layer_no, value);
                        continue;
                    }
                    let current = rules
                        .clearance_matrix
                        .get_value(class_no, other, layer_no, false);
                    let effective = current.max(value);
                    rules
                        .clearance_matrix
                        .set_value(class_no, other, layer_no, effective);
                    rules
                        .clearance_matrix
                        .set_value(other, class_no, layer_no, effective);
                }
            }
            applied += 1;
            continue;
        };
        // Consume special type atoms once, before `type_pairs` expands the
        // remaining matrix-pair atoms.  In particular, a class-scoped
        // `via_via_same_net` must reach the same-net DRC table rather than
        // becoming a stray clearance-matrix class.
        let raw_type_tokens: Vec<(String, bool)> = type_node
            .as_list()
            .unwrap_or(&[])
            .iter()
            .skip(1)
            .filter_map(|t| t.as_atom().map(|s| (s.to_string(), t.is_quoted())))
            .collect();
        for (name, quoted) in &raw_type_tokens {
            if *quoted {
                continue;
            }
            if name.eq_ignore_ascii_case("smd_to_turn_gap") {
                rules.set_pin_edge_to_turn_dist(value as f64);
                gap_seen = true;
                applied += 1;
            } else if apply_same_net_type_token(rules, name, value) {
                applied += 1;
            }
        }
        // A wire pair makes Java create the four standard item columns in the
        // class namespace before applying the remaining pairs.
        if raw_type_tokens.iter().any(|(name, quoted)| {
            if *quoted {
                return false;
            }
            let lower = name.to_ascii_lowercase();
            lower.starts_with("wire_")
                || lower.starts_with("wire-")
                || lower.ends_with("_wire")
                || lower.ends_with("-wire")
        }) {
            for item in ["via", "smd", "pin", "area"] {
                ensure_class_item_clearance_class(rules, net_class_idx, item, true);
            }
        }
        for (a, b) in type_pairs(type_node) {
            // `smd_to_turn_gap` and unquoted `*_same_net` atoms were handled
            // above and are filtered by `type_pairs`; quoted names are valid
            // ordinary matrix classes and must remain untouched.
            let (ia, _) = ensure_class_item_clearance_class(rules, net_class_idx, &a, true);
            let (ib, _) = ensure_class_item_clearance_class(rules, net_class_idx, &b, true);
            if let Some(layer_no) = layer {
                rules.clearance_matrix.set_value(ia, ib, layer_no, value);
                rules.clearance_matrix.set_value(ib, ia, layer_no, value);
            } else {
                rules
                    .clearance_matrix
                    .set_value_on_all_layers(ia, ib, value);
                rules
                    .clearance_matrix
                    .set_value_on_all_layers(ib, ia, value);
            }
            applied += 1;
        }
    }
    (applied, gap_seen)
}

/// Applies one `(class_class (classes A B ...) ...)` scope.  Class-pair
/// rules use matrix columns named after the *net classes*, not whichever
/// scalar/default column happened to exist when the scope was read.  Typed
/// pairs are resolved in each endpoint's class namespace, just as Java's
/// `Network.add_mixed_clearance_rule` does.
pub(crate) fn apply_class_class_scope(
    rules: &mut BoardRules,
    node: &SExpr,
    scale: &dyn Fn(f64) -> i32,
) {
    let names: Vec<String> = node
        .child("classes")
        .map(|c| c.args().map(|s| s.to_string()).collect())
        .unwrap_or_default();
    let class_indices: Vec<usize> = names
        .iter()
        .filter_map(|name| rules.net_classes.get_by_name(name))
        .collect();
    if class_indices.len() < 2 {
        return;
    }
    for i in 0..names.len() {
        for j in (i + 1)..names.len() {
            let (Some(a_idx), Some(b_idx)) = (
                rules.net_classes.get_by_name(&names[i]),
                rules.net_classes.get_by_name(&names[j]),
            ) else {
                continue;
            };
            for rule in node.children("rule") {
                apply_mixed_rule(rules, a_idx, b_idx, rule, scale, None);
            }
            for layer_rule in node.children("layer_rule") {
                let layers: Vec<usize> = layer_rule
                    .args()
                    .filter_map(|name| rules.layer_structure().get_no(name))
                    .collect();
                for layer in layers {
                    for rule in layer_rule.children("rule") {
                        apply_mixed_rule(rules, a_idx, b_idx, rule, scale, Some(layer));
                    }
                }
            }
        }
    }
}

fn apply_mixed_rule(
    rules: &mut BoardRules,
    first_class_idx: usize,
    second_class_idx: usize,
    rule: &SExpr,
    scale: &dyn Fn(f64) -> i32,
    layer: Option<usize>,
) {
    for child in rule.as_list().unwrap_or(&[]).iter().filter(|c| {
        c.name()
            .is_some_and(|n| n.eq_ignore_ascii_case("clearance") || n.eq_ignore_ascii_case("clear"))
    }) {
        let Some(value) = child.arg_f64().map(scale) else {
            continue;
        };
        let Some(type_node) = child.child("type") else {
            let (a, _) = ensure_class_item_clearance_class(rules, first_class_idx, "wire", false);
            let (b, _) = ensure_class_item_clearance_class(rules, second_class_idx, "wire", false);
            set_matrix_pair(rules, a, b, value, layer);
            continue;
        };
        for (a_name, b_name) in type_pairs(type_node) {
            let (a, _) = ensure_class_item_clearance_class(rules, first_class_idx, &a_name, false);
            let (b, _) = ensure_class_item_clearance_class(rules, second_class_idx, &b_name, false);
            let (a_rev, _) =
                ensure_class_item_clearance_class(rules, second_class_idx, &a_name, false);
            let (b_rev, _) =
                ensure_class_item_clearance_class(rules, first_class_idx, &b_name, false);
            set_matrix_pair(rules, a, b, value, layer);
            set_matrix_pair(rules, a_rev, b_rev, value, layer);
        }
    }
}

fn set_matrix_pair(
    rules: &mut BoardRules,
    first: usize,
    second: usize,
    value: i32,
    layer: Option<usize>,
) {
    if let Some(layer_no) = layer {
        rules
            .clearance_matrix
            .set_value(first, second, layer_no, value);
        rules
            .clearance_matrix
            .set_value(second, first, layer_no, value);
    } else {
        rules
            .clearance_matrix
            .set_value_on_all_layers(first, second, value);
        rules
            .clearance_matrix
            .set_value_on_all_layers(second, first, value);
    }
}

/// Applies one `(class NAME [net...] ...)` scope (Java
/// `Network.insert_net_class`): membership, inline clearance rule or
/// `(clearance_class NAME)` reference, `(via_rule NAME)` reference or
/// `(circuit (use_via ...))` fallback, `(circuit (use_layer ...))`,
/// per-class widths and `(shove_fixed ...)`. A class name already on the
/// board updates that class in place (Java looks classes up by name).
pub(crate) fn apply_class_scope(
    rules: &mut BoardRules,
    ctx: &NetworkScopeCtx,
    class_node: &SExpr,
    scale: &dyn Fn(f64) -> i32,
    via_rule_ids: &HashMap<String, usize>,
) {
    let mut class_args = class_node.args();
    let Some(raw_class_name) = class_args.next() else {
        return;
    };
    // A few KiCad 4 exports encode the implicit default class as two empty
    // quoted atoms: `(class '' ...)`. Treat that spelling as the canonical
    // default class instead of creating an unnameable matrix column.
    let class_name = if raw_class_name.is_empty() {
        "default"
    } else {
        raw_class_name
    };
    let member_nets: Vec<&str> = class_args.filter(|name| !name.is_empty()).collect();
    // classes resolve BY NAME (Java Network.insert_net_class): only the
    // class actually named "default" describes the default rules. The
    // former any-empty-class-is-default rule let every memberless named
    // class overwrite the default class — Issue029's 11 classes folded
    // into 5, with default inheriting the LAST empty class's settings.
    let default_idx = rules.get_default_net_class();
    let class_idx = rules
        .net_classes
        .get_by_name(class_name)
        .unwrap_or_else(|| rules.append_net_class(class_name));
    // Apply every rule scope in source order.  Width and clearance are
    // intentionally handled in the same traversal so later declarations
    // have Java's last-write-wins behavior.
    let mut class_has_clearance = false;
    for rule in class_node.children("rule") {
        for w in rule.children("width").filter_map(|w| w.arg_f64()) {
            rules
                .net_classes
                .get_mut(class_idx)
                .set_trace_half_width((scale(w) / 2).max(1));
        }
        apply_class_rule_scope(rules, class_idx, rule, scale, None);
        class_has_clearance |= rule_has_clearance_declaration(rule);
    }
    for layer_rule in class_node.children("layer_rule") {
        let layers: Vec<usize> = layer_rule
            .args()
            .filter_map(|name| ctx.layer_structure.get_no(name))
            .collect();
        for layer in layers {
            for rule in layer_rule.children("rule") {
                for w in rule.children("width").filter_map(|w| w.arg_f64()) {
                    rules
                        .net_classes
                        .get_mut(class_idx)
                        .set_trace_half_width_on_layer(layer, (scale(w) / 2).max(1));
                }
                apply_class_rule_scope(rules, class_idx, rule, scale, Some(layer));
                class_has_clearance |= rule_has_clearance_declaration(rule);
            }
        }
    }
    // A class may use a named matrix class without an inline rule. Java
    // applies this reference before its inline rules; any successfully
    // parsed inline clearance (including typed-only rules such as
    // smd_to_turn_gap) therefore owns the trace class. Checking only for a
    // wire pair would let this reference overwrite a class-local column.
    if !class_has_clearance {
        if let Some(name) = class_node.child("clearance_class").and_then(|c| c.arg()) {
            if let Some(cc) = lookup_clearance_class(rules, name) {
                rules
                    .net_classes
                    .get_mut(class_idx)
                    .set_trace_clearance_class(cc);
            }
        }
    }
    let via_padstacks: Vec<usize> = class_node
        .children("circuit")
        .flat_map(|c| c.children("use_via"))
        .flat_map(|u| u.args())
        .filter_map(|name| ctx.padstack_no(name))
        .collect();
    {
        let class = rules.net_classes.get_mut(class_idx);
        // (circuit (use_layer L ...)): ONLY the listed layers stay
        // active routing layers, and inactive layers get trace
        // width 0 (Java Network.create_active_trace_layers)
        let use_layers: Vec<usize> = class_node
            .children("circuit")
            .flat_map(|c| c.children("use_layer"))
            .flat_map(|u| u.args())
            .filter_map(|n| ctx.layer_structure.get_no(n))
            .collect();
        if class_node
            .children("circuit")
            .any(|c| c.child("use_layer").is_some())
        {
            class.set_all_layers_active(false);
            for &l in &use_layers {
                class.set_active_routing_layer(l, true);
            }
            for l in 0..ctx.layer_structure.layer_count() {
                if !class.is_active_routing_layer(l) {
                    class.set_trace_half_width_on_layer(l, 0);
                }
            }
        }
        // (shove_fixed on|off): recorded on the class (Java parses it
        // in the class scope; its consumer is the interactive stitch
        // route, which this port does not have)
        if let Some(v) = class_node.child("shove_fixed").and_then(|s| s.arg()) {
            class.set_shove_fixed(v.eq_ignore_ascii_case("on"));
        }
        if let Some(v) = class_node.child("pull_tight").and_then(|s| s.arg()) {
            class.set_pull_tight(v.eq_ignore_ascii_case("on"));
        }
        // Specctra orders the two length values as MAX then MIN. Negative
        // max and zero min are the standard "unset" sentinels.
        for circuit in class_node.children("circuit") {
            let Some(length) = circuit.child("length") else {
                continue;
            };
            let values: Vec<f64> = length.args().filter_map(|v| v.parse().ok()).collect();
            if let Some(&max) = values.first() {
                if max > 0.0 {
                    class.set_maximum_trace_length(scale(max) as f64);
                }
            }
            if let Some(&min) = values.get(1) {
                if min > 0.0 {
                    class.set_minimum_trace_length(scale(min) as f64);
                }
            }
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
    } else if !via_padstacks.is_empty() {
        // Java create_via_rule reuses the existing via info for the
        // padstack; only when none was declared is one created, with
        // the default attach rule (via_at_smd && padstack attach).
        let via_class = rules
            .net_classes
            .get(class_idx)
            .default_item_clearance_classes
            .get(crate::rules::ItemClass::Via);
        let mut via_rule = crate::rules::ViaRule::new(class_name);
        for padstack_no in via_padstacks {
            let existing = (0..rules.via_infos.count()).find(|&i| {
                let info = rules.via_infos.get(i);
                info.get_padstack() == padstack_no && info.get_clearance_class() == via_class
            });
            let via_info_id = existing.or_else(|| {
                let attach = rules.via_at_smd_allowed
                    && ctx
                        .padstacks
                        .get_by_no(padstack_no)
                        .is_some_and(|p| p.attach_allowed);
                rules.via_infos.add(crate::rules::ViaInfo::new(
                    format!("via::{class_name}::{padstack_no}"),
                    padstack_no,
                    via_class,
                    attach,
                ))
            });
            if let Some(via_info_id) = via_info_id {
                via_rule.append_via(via_info_id);
            }
        }
        if via_rule.via_count() > 0 {
            rules.via_rules.push(via_rule);
            let rule_id = rules.via_rules.len() - 1;
            rules
                .net_classes
                .get_mut(class_idx)
                .set_via_rule(Some(rule_id));
        }
    } else if class_has_clearance && class_idx != default_idx {
        // Java creates class-specific default ViaInfos when a class has a
        // clearance rule, even if no `(use_via ...)` list is present.  This
        // prevents a strict class from silently reusing a weaker default via.
        let via_class = rules
            .net_classes
            .get(class_idx)
            .default_item_clearance_classes
            .get(crate::rules::ItemClass::Via);
        let mut via_rule = crate::rules::ViaRule::new(class_name);
        for ps_no in 1..=ctx.padstacks.count() {
            let Some(ps) = ctx.padstacks.get_by_no(ps_no) else {
                continue;
            };
            if ps.to_layer() <= ps.from_layer() {
                continue; // one-layer padstacks are pins/SMDs, not vias
            }
            let attach = rules.via_at_smd_allowed && ps.attach_allowed;
            let existing = (0..rules.via_infos.count()).find(|&i| {
                let info = rules.via_infos.get(i);
                info.get_padstack() == ps_no && info.get_clearance_class() == via_class
            });
            let info_id = existing.or_else(|| {
                rules.via_infos.add(crate::rules::ViaInfo::new(
                    format!("{}-{}", ps.name, class_name),
                    ps_no,
                    via_class,
                    attach,
                ))
            });
            if let Some(id) = info_id {
                via_rule.append_via(id);
            }
        }
        if via_rule.via_count() > 0 {
            rules.via_rules.push(via_rule);
            rules
                .net_classes
                .get_mut(class_idx)
                .set_via_rule(Some(rules.via_rules.len() - 1));
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

/// The fixed state of a `(wire ...)`/`(via ...)` wiring node from its
/// `(type ...)` attribute, mapped like Java `Wiring.calc_fixed`:
/// `shove_fixed` → ShoveFixed, `fix` → SystemFixed, `normal` (and absent) →
/// Unfixed; every other explicit token, including KiCad's `route`, is
/// UserFixed. This matches Java `Wiring.calc_fixed`; the writer represents a
/// genuinely unfixed item by omitting `(type ...)`.
fn wiring_fixed_state(node: &SExpr) -> crate::board::FixedState {
    let Some(t) = node.child("type").and_then(|t| t.arg()) else {
        return crate::board::FixedState::Unfixed;
    };
    if t.eq_ignore_ascii_case("shove_fixed") {
        crate::board::FixedState::ShoveFixed
    } else if t.eq_ignore_ascii_case("fix") {
        crate::board::FixedState::SystemFixed
    } else if t.eq_ignore_ascii_case("normal") {
        crate::board::FixedState::Unfixed
    } else {
        crate::board::FixedState::UserFixed
    }
}

/// An explicit wiring-level `(clearance_class NAME)` resolved against the
/// board's clearance matrix (Java `Wiring.read_wire_scope`). An absent scope
/// or an unknown symbol lets the caller retain its net-class default.
fn wiring_clearance_class(
    board: &BasicBoard,
    node: &SExpr,
    context: &str,
) -> Result<Option<usize>, ImportError> {
    inherited_clearance_class(&board.rules, node, context)
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
    /// The image-relative keepout name. Placement scopes use this identity to
    /// override the clearance class for one component instance.
    name: String,
    /// relative shape, already scaled to board units
    area: crate::geometry::planar::PolygonShape,
    /// a named layer, or `None` for signal/all (every layer)
    layer: Option<usize>,
    /// a `(via_keepout ...)`: blocks vias only
    via_only: bool,
    /// resolved clearance class (from a nested or trailing-sibling
    /// `(clearance_class ...)`; the default class when absent)
    clearance_class: usize,
    clearance_class_explicit: bool,
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
    // A parsed tree is not enough to establish a usable board: malformed
    // layers, padstack shapes, net/class references, or item geometry can
    // otherwise survive as plausible defaults and fail much later in the
    // router.  Validate the completed graph before exposing it to callers.
    // Duplicate symbols are deliberately rejected here. Resolving a repeated
    // padstack name by either first- or last-write order changes component
    // geometry, so a public interchange boundary must not guess.
    crate::board::validation::validate_board_references(&board)
        .map_err(|error| err(format!("imported DSN violates board invariants: {error}")))?;
    // retain the document without its wiring for DSN export (the router
    // only changes the wiring section): remove exactly the balanced
    // `(wiring ...)` span, keeping everything before and after it
    board.dsn_source = Some(strip_wiring(content));
    // The exporter preserves every non-wiring section verbatim. Capture the
    // fully imported semantic state now so later routing edits can be
    // distinguished from unsafe rule or geometry edits.
    board.capture_dsn_semantic_baseline();
    Ok(board)
}

/// The document with its top-level `(wiring ...)` sections removed. The
/// structural scan uses the parser's exact quote/comment rules, so text such
/// as `"(wiring fake)"` is never mistaken for copper. Invalid input is left
/// untouched; [`import_dsn`] reports the parse error before retaining it.
pub fn strip_wiring(content: &str) -> String {
    let Ok((_, spans)) = crate::io::dsn::document_structure(content, "wiring") else {
        return content.to_string();
    };
    if spans.is_empty() {
        return content.to_string();
    }
    let mut out = String::with_capacity(content.len());
    let mut cursor = 0usize;
    for span in spans {
        out.push_str(content[cursor..span.start].trim_end_matches([' ', '\t']));
        cursor = span.end;
    }
    out.push_str(&content[cursor..]);
    out
}

/// Reads the Specctra resolution declaration without applying lossy defaults.
/// Coordinates and every rule value are scaled by this value, so accepting a
/// typo here (or coercing a fractional/overflowing value into `i32`) changes
/// the physical board while still producing an apparently valid import.
///
/// The declaration is optional in the format; an omitted declaration keeps
/// the historical Specctra default of one micrometre file unit.  When present,
/// however, it must contain exactly a supported physical unit and a positive
/// integral resolution that fits the board's integer coordinate grid.
fn parse_resolution(pcb: &SExpr) -> Result<(String, i32), ImportError> {
    let mut declarations = pcb.children("resolution");
    let Some(node) = declarations.next() else {
        return Ok(("um".to_string(), 1));
    };
    if declarations.next().is_some() {
        return Err(err(
            "the design contains more than one resolution declaration",
        ));
    }
    let mut args = node.args();
    let unit = args
        .next()
        .ok_or_else(|| err("resolution is missing its physical unit"))?;
    let raw = args
        .next()
        .ok_or_else(|| err("resolution is missing its numeric value"))?;
    if args.next().is_some() {
        return Err(err("resolution must contain exactly a unit and a value"));
    }
    if !matches!(
        unit.to_ascii_lowercase().as_str(),
        "um" | "micron" | "microns" | "mil" | "mm" | "cm" | "inch" | "in"
    ) {
        return Err(err(format!("unsupported resolution unit {unit:?}")));
    }
    let value = raw
        .parse::<f64>()
        .map_err(|_| err(format!("resolution value {raw:?} is not numeric")))?;
    if !value.is_finite() || value <= 0.0 {
        return Err(err(format!(
            "resolution value must be finite and positive, got {raw:?}"
        )));
    }
    if value.fract() != 0.0 {
        return Err(err(format!(
            "resolution value must be an integer, got {raw:?}"
        )));
    }
    if value > f64::from(i32::MAX) {
        return Err(err(format!(
            "resolution value {raw:?} exceeds the board coordinate limit"
        )));
    }
    Ok((unit.to_string(), value as i32))
}

/// Rejects malformed numeric atoms in the DSN scopes this importer actually
/// consumes.  The semantic readers intentionally ignore unknown Specctra
/// extensions for compatibility, but a known coordinate/rule token must not
/// be silently removed by `filter_map` or replaced with zero: either behavior
/// can produce a different, still-plausible board.
fn validate_known_numeric_scopes(pcb: &SExpr) -> Result<(), ImportError> {
    fn finite(atom: &str, context: &str) -> Result<f64, ImportError> {
        let value = atom
            .parse::<f64>()
            .map_err(|_| err(format!("{context} contains non-numeric atom {atom:?}")))?;
        if !value.is_finite() {
            return Err(err(format!("{context} must contain only finite values")));
        }
        Ok(value)
    }

    fn shape(node: &SExpr, context: &str, strict_geometry: bool) -> Result<(), ImportError> {
        let kind = node
            .name()
            .ok_or_else(|| err(format!("{context} is missing its shape kind")))?;
        let args: Vec<&str> = node.args().collect();
        if args.is_empty() {
            return if strict_geometry {
                Err(err(format!("{context} is missing its layer")))
            } else {
                Ok(())
            };
        }
        let numbers = &args[1..];
        for atom in numbers {
            finite(atom, context)?;
        }
        let malformed = match kind.to_ascii_lowercase().as_str() {
            "circle" | "circ" => !matches!(numbers.len(), 1 | 3),
            "rect" => numbers.len() != 4,
            "path" | "polygon" => numbers.len() < 5 || !(numbers.len() - 1).is_multiple_of(2),
            _ => false,
        };
        if malformed && !strict_geometry {
            // Keepouts are optional obstacle hints.  Several legacy exporters
            // emit an empty marker (for example `(polygon F.Cu)`) which Java
            // skips.  Ignore that marker, but still reject non-numeric atoms
            // above so malformed numeric data cannot silently become zero.
            return Ok(());
        }
        match kind.to_ascii_lowercase().as_str() {
            // Centre coordinates are optional, but they are a pair.
            "circle" | "circ" if !matches!(numbers.len(), 1 | 3) => Err(err(format!(
                "{context} circle needs a diameter and optionally an x/y centre"
            ))),
            "rect" if numbers.len() != 4 => Err(err(format!(
                "{context} rect needs exactly two coordinate pairs"
            ))),
            // width/aperture followed by complete coordinate pairs
            "path" | "polygon" if numbers.len() < 5 || !(numbers.len() - 1).is_multiple_of(2) => {
                Err(err(format!(
                    "{context} {kind} needs a width/aperture and complete coordinate pairs"
                )))
            }
            _ => Ok(()),
        }
    }

    fn walk(node: &SExpr, parent: Option<&str>, path: &str) -> Result<(), ImportError> {
        let Some(name) = node.name() else {
            return Ok(());
        };
        let lname = name.to_ascii_lowercase();
        let parent = parent.unwrap_or("");
        match lname.as_str() {
            "width" | "clearance" | "clear" => {
                let args: Vec<&str> = node.args().collect();
                if args.len() != 1 {
                    return Err(err(format!(
                        "{path}/{name} must contain exactly one numeric value"
                    )));
                }
                let value = finite(args[0], &format!("{path}/{name}"))?;
                if value < 0.0 {
                    return Err(err(format!("{path}/{name} must be nonnegative")));
                }
            }
            "length" => {
                let args: Vec<&str> = node.args().collect();
                if args.len() != 2 {
                    return Err(err(format!(
                        "{path}/length must contain exactly two numeric values"
                    )));
                }
                for atom in args {
                    finite(atom, &format!("{path}/length"))?;
                }
            }
            "index" if parent.eq_ignore_ascii_case("property") => {
                let args: Vec<&str> = node.args().collect();
                if args.len() != 1 || args[0].parse::<i64>().is_err() {
                    return Err(err(format!(
                        "{path}/index must contain exactly one integer"
                    )));
                }
            }
            "shape" if parent.eq_ignore_ascii_case("padstack") => {
                let Some(inner) = node.as_list().and_then(|items| items.get(1)) else {
                    return Err(err(format!("{path}/shape is missing its geometry")));
                };
                if inner.name().is_none() {
                    return Err(err(format!("{path}/shape geometry must be a list")));
                }
                shape(inner, &format!("{path}/shape"), true)?;
            }
            "path" | "polygon" | "rect" | "circle" | "circ"
                if matches!(
                    parent.to_ascii_lowercase().as_str(),
                    "boundary" | "plane" | "keepout" | "via_keepout"
                ) =>
            {
                let optional_keepout = matches!(
                    parent.to_ascii_lowercase().as_str(),
                    "keepout" | "via_keepout"
                );
                shape(node, &format!("{path}/{name}"), !optional_keepout)?;
            }
            "pin" if parent.eq_ignore_ascii_case("image") => {
                // Nested `(rotate ...)` is not an atom and is deliberately
                // absent from args(); the remaining grammar is padstack,
                // pin name, dx, dy.
                let args: Vec<&str> = node.args().collect();
                if args.len() != 4 {
                    return Err(err(format!(
                        "{path}/pin needs a padstack, pin name, and x/y offset"
                    )));
                }
                finite(args[2], &format!("{path}/pin offset"))?;
                finite(args[3], &format!("{path}/pin offset"))?;
            }
            "place" if parent.eq_ignore_ascii_case("component") => {
                let args: Vec<&str> = node.args().collect();
                // Java accepts a name-only place as an intentionally
                // unplaced component (`ComponentLocation.coor == null`).
                // It still participates in the logical netlist, but has no
                // physical pins until a later placement is supplied.
                if args.len() != 1 {
                    if !(4..=5).contains(&args.len()) {
                        return Err(err(format!(
                            "{path}/place needs either only a refdes (unplaced) or refdes, x, y, side, and optional rotation"
                        )));
                    }
                    finite(args[1], &format!("{path}/place coordinate"))?;
                    finite(args[2], &format!("{path}/place coordinate"))?;
                    if let Some(rotation) = args.get(4) {
                        finite(rotation, &format!("{path}/place rotation"))?;
                    }
                }
            }
            "net" if parent.eq_ignore_ascii_case("network") => {
                let args: Vec<&str> = node.args().collect();
                if args.is_empty() || args.len() > 2 {
                    return Err(err(format!(
                        "{path}/net needs a name and optional positive subnet"
                    )));
                }
                if let Some(subnet) = args.get(1) {
                    let parsed = subnet.parse::<usize>().map_err(|_| {
                        err(format!("{path}/net subnet {subnet:?} is not an integer"))
                    })?;
                    if parsed == 0 {
                        return Err(err(format!("{path}/net subnet must be positive")));
                    }
                }
            }
            _ => {}
        }
        if let Some(items) = node.as_list() {
            for (index, child) in items.iter().enumerate().skip(1) {
                walk(child, Some(name), &format!("{path}/{name}[{index}]"))?;
            }
        }
        Ok(())
    }

    walk(pcb, None, "pcb")
}

fn import_dsn_inner(content: &str) -> Result<BasicBoard, ImportError> {
    let pcb = parse_dsn(content).map_err(|e| err(e.to_string()))?;
    if !pcb.name().is_some_and(|n| n.eq_ignore_ascii_case("pcb")) {
        return Err(err("root node is not (pcb ...)"));
    }
    validate_known_numeric_scopes(&pcb)?;
    let structure = pcb.child("structure").ok_or_else(|| err("no structure"))?;

    // resolution: `(resolution <unit> <value>)`. File coordinates are
    // multiplied by <value> to get integer board units; <unit> is the
    // physical unit, which must be preserved for a faithful SES round-trip
    // (a `mil` design was previously relabelled `um` on export).
    let (unit, resolution) = parse_resolution(&pcb)?;
    // Float-to-integer casts saturate in Rust.  That is convenient for some
    // numeric code, but unsafe at an interchange boundary: a coordinate just
    // outside the board grid would silently collapse onto i32::MAX/MIN and
    // could still form a plausible-looking polygon.  Keep the importer
    // closures ergonomic while recording the first out-of-range scaled value;
    // the transaction is rejected once all scopes have been parsed.
    let scale_error: RefCell<Option<String>> = RefCell::new(None);
    let scale = |v: f64| -> i32 {
        let scaled = v * f64::from(resolution);
        let limit = f64::from(crate::geometry::planar::limits::CRIT_INT);
        if !scaled.is_finite() || scaled < -limit || scaled > limit {
            let mut error = scale_error.borrow_mut();
            if error.is_none() {
                *error = Some(format!(
                    "scaled geometry/rule value {v} does not fit the board coordinate range"
                ));
            }
            return 0;
        }
        scaled.round() as i32
    };

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

    // Default rules (`(clear ...)` is a Specctra alias for `(clearance ...)`).
    // Start with Java's fallback values, then apply every child in source
    // order.  In particular, a later width/clearance wins; selecting the
    // first rule silently changed Issue029-style files.
    let rule_nodes: Vec<&SExpr> = structure.children("rule").collect();
    let default_clearance = 200;
    let default_width = 250;
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

    rules.set_default_trace_half_widths((default_width / 2).max(1));
    let mut smd_gap_found = false;
    for rule in &rule_nodes {
        for width in rule.children("width").filter_map(|w| w.arg_f64()) {
            rules.set_default_trace_half_widths((scale(width) / 2).max(1));
        }
        apply_rule_scope_clearances(&mut rules, rule, &scale, true);
        smd_gap_found |= rule_has_smd_to_turn_gap(rule);
    }
    // (layer L ... (rule (width W) (clearance C))): per-layer defaults
    // (Java Structure.read_layer_scope) — the layer's width lands on the
    // default net class's layer entry and its clearance on the matrix
    // layer, overriding the global defaults above
    for layer_node in structure.children("layer") {
        let Some(layer) = layer_node.arg().and_then(&layer_no) else {
            continue;
        };
        for rule in layer_node.children("rule") {
            for w in rule
                .children("width")
                .filter_map(|w| w.arg_f64())
                .map(&scale)
            {
                let dc = rules.get_default_net_class();
                rules
                    .net_classes
                    .get_mut(dc)
                    .set_trace_half_width_on_layer(layer, (w / 2).max(1));
            }
            apply_rule_scope_clearances_on_layer(&mut rules, rule, &scale, true, Some(layer));
            smd_gap_found |= rule_has_smd_to_turn_gap(rule);
        }
    }
    if !smd_gap_found {
        rules.set_pin_edge_to_turn_dist(rules.get_min_trace_half_width() as f64);
    }

    // library: padstacks and images
    let mut padstacks = Padstacks::new(layer_count);
    let mut padstack_nos: HashMap<String, usize> = HashMap::new();
    let mut images: HashMap<String, Image> = HashMap::new();
    // Read every padstack before images so an image may legally reference a
    // padstack declared in a later library scope.  Each individual shape is
    // validated first because the shared low-level reader otherwise skips
    // unsupported shapes and unknown layers.
    for library in pcb.children("library") {
        for padstack_node in library.children("padstack") {
            validate_padstack_scope_references(padstack_node, &layer_structure)?;
            let Some(no) =
                read_padstack_scope(&mut padstacks, &layer_structure, &scale, padstack_node)
            else {
                return Err(err("padstack without name"));
            };
            padstack_nos.insert(padstacks.get_by_no(no).unwrap().name.clone(), no);
        }
    }
    for library in pcb.children("library") {
        for image_node in library.children("image") {
            let name = image_node.arg().ok_or_else(|| err("image without name"))?;
            if images.contains_key(name) {
                return Err(err(format!("duplicate image declaration {name:?}")));
            }
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
                if padstacks.get(&padstack_name).is_none() {
                    return Err(err(format!(
                        "image {name:?} pin {pin_name:?} references unknown padstack {padstack_name:?}"
                    )));
                }
                if pins.iter().any(|pin: &ImagePin| pin.pin_name == pin_name) {
                    return Err(err(format!(
                        "image {name:?} declares duplicate pin {pin_name:?}"
                    )));
                }
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
            // Iterated in document order: Eagle exports put the keepout's
            // (clearance_class ...) as the immediately FOLLOWING sibling
            // (Issue143), while standard Specctra nests it inside.
            let mut keepouts = Vec::new();
            let image_items = image_node.as_list().unwrap_or(&[]);
            for (idx, ko) in image_items.iter().enumerate() {
                let Some(ko_kind) = ko.name() else {
                    continue;
                };
                let via_only = ko_kind.eq_ignore_ascii_case("via_keepout");
                if !via_only && !ko_kind.eq_ignore_ascii_case("keepout") {
                    continue;
                }
                let Some(shape_node) = ko
                    .child("polygon")
                    .or_else(|| ko.child("rect"))
                    .or_else(|| ko.child("circle"))
                    .or_else(|| ko.child("circ"))
                    .or_else(|| ko.child("path"))
                else {
                    continue;
                };
                let keepout_name = ko.arg().unwrap_or_default().to_string();
                let layer = match shape_node.arg() {
                    Some(layer_name) => keepout_layer_reference(
                        &layer_structure,
                        layer_name,
                        &format!("image {name:?} keepout"),
                    )?,
                    // Missing layer is part of malformed optional keepout
                    // geometry and is skipped below with the empty shape.
                    None => None,
                };
                let corners = keepout_corners(shape_node, &scale);
                if corners.len() < 3 {
                    continue;
                }
                let polygon = crate::geometry::planar::PolygonShape::new(corners);
                if polygon.dimension() != 2 || polygon.is_empty() {
                    // Some KiCad exports retain an empty keepout marker as a
                    // three-point polygon with all points equal. Do not turn
                    // that marker into a one-point obstacle area.
                    continue;
                }
                let cc_name = ko
                    .child("clearance_class")
                    .and_then(|c| c.arg())
                    .or_else(|| {
                        image_items
                            .get(idx + 1)
                            .filter(|n| {
                                n.name()
                                    .is_some_and(|x| x.eq_ignore_ascii_case("clearance_class"))
                            })
                            .and_then(|n| n.arg())
                    });
                let resolved_clearance =
                    cc_name.and_then(|class_name| lookup_clearance_class(&rules, class_name));
                let clearance_class_explicit = resolved_clearance.is_some();
                let clearance_class = resolved_clearance.unwrap_or_else(|| {
                    rules.item_clearance_class_for(0, crate::rules::ItemClass::Area)
                });
                keepouts.push(ImageKeepout {
                    name: keepout_name,
                    area: polygon,
                    layer,
                    via_only,
                    clearance_class,
                    clearance_class_explicit,
                });
            }
            images.insert(name.to_string(), Image { pins, keepouts });
        }
    }

    // Materialize the placement namespace before reading network pin lists.
    // Component and pin names remain structural. Flattening `A-B`/`C` and
    // `A`/`B-C` into the same string would attach one net to the wrong pad.
    // Counting rather than using a set also catches duplicate placements that
    // would create two physical pads for one logical endpoint.
    let mut physical_pin_ref_counts: HashMap<LogicalEndpoint, usize> = HashMap::new();
    for placement in pcb.children("placement") {
        for component in placement.children("component") {
            let image_name = component
                .arg()
                .ok_or_else(|| err("placement component is missing its image name"))?;
            let image = images.get(image_name).ok_or_else(|| {
                err(format!(
                    "placement component references unknown image {image_name:?}"
                ))
            })?;
            for place in component.children("place") {
                let refdes = place.arg().filter(|name| !name.is_empty()).ok_or_else(|| {
                    err(format!(
                        "component {image_name:?} has a place without refdes"
                    ))
                })?;
                for override_node in place.children("pin") {
                    let pin_name = override_node.arg().ok_or_else(|| {
                        err(format!("placement {refdes:?} has an unnamed pin override"))
                    })?;
                    if !image.pins.iter().any(|pin| pin.pin_name == pin_name) {
                        return Err(err(format!(
                            "placement {refdes:?} overrides unknown image pin {pin_name:?}"
                        )));
                    }
                }
                // A name-only `(place REF)` is a valid Java logical
                // component with no geometry.  Do not put its pins in the
                // physical namespace; network references will be retained as
                // unresolved obligations below.
                let is_physical = place.args().count() >= 4;
                if !is_physical {
                    continue;
                }
                for pin in &image.pins {
                    let pin_ref = LogicalEndpoint::new(refdes, &pin.pin_name);
                    *physical_pin_ref_counts.entry(pin_ref).or_default() += 1;
                }
            }
        }
    }

    // network: structural component/pin reference -> one or more net numbers.
    // Specctra permits repeated net names with
    // explicit subnet numbers and can partition a `(net ...)` with
    // `(fromto ...)` or `(order ...)`; retaining those partitions is required
    // for electrical equivalence and for wiring/session references.
    let mut pin_nets: HashMap<LogicalEndpoint, Vec<i32>> = HashMap::new();
    let mut unresolved_net_endpoints: BTreeMap<i32, BTreeSet<LogicalEndpoint>> = BTreeMap::new();
    for network in pcb.children("network") {
        for net_node in network.children("net") {
            let mut net_args = net_node.args();
            let net_name = net_args.next().unwrap_or_default();
            if net_name.is_empty() {
                return Err(err("network net is missing its name"));
            }
            let first_subnet = net_args
                .next()
                .and_then(|value| value.parse::<usize>().ok())
                .filter(|value| *value > 0)
                .unwrap_or(1);
            // Java accumulates every `(pins ...)` and `(order ...)` scope in
            // document order; any order scope turns that combined list into
            // adjacent two-pin subnets. Explicit `(fromto ...)` groups take
            // precedence but all declarations are still retained for
            // unresolved-endpoint accounting.
            let mut ordered_pins = Vec::new();
            let mut fromto_groups = Vec::new();
            let mut pin_order_found = false;
            for child in net_node
                .as_list()
                .unwrap_or(&[])
                .iter()
                .filter(|child| child.as_list().is_some())
            {
                match child.name().map(str::to_ascii_lowercase).as_deref() {
                    Some("pins") | Some("order") => {
                        if child
                            .name()
                            .is_some_and(|name| name.eq_ignore_ascii_case("order"))
                        {
                            pin_order_found = true;
                        }
                        let refs = pin_reference_args(child)?;
                        ordered_pins.extend(refs);
                    }
                    Some("fromto") => {
                        let refs = pin_reference_args(child)?;
                        if !refs.is_empty() {
                            fromto_groups.push(refs);
                        }
                    }
                    _ => {}
                }
            }
            let groups: Vec<Vec<LogicalEndpoint>> = if !fromto_groups.is_empty() {
                fromto_groups
            } else if pin_order_found {
                if ordered_pins.len() > 1 {
                    ordered_pins.windows(2).map(|pair| pair.to_vec()).collect()
                } else {
                    // Preserve the logical net even for the one-pin order
                    // corner case; dropping it would hide an obligation.
                    vec![ordered_pins]
                }
            } else {
                vec![ordered_pins]
            };
            for (offset, group) in groups.into_iter().enumerate() {
                let subnet = first_subnet.saturating_add(offset);
                let net_no = rules
                    .nets
                    .get(net_name, subnet)
                    .map(|net| net.net_number)
                    .unwrap_or_else(|| rules.nets.add(net_name, subnet, false));
                for pin_ref in group {
                    match physical_pin_ref_counts.get(&pin_ref).copied().unwrap_or(0) {
                        1 => {
                            let entry = pin_nets.entry(pin_ref).or_default();
                            append_unique_net(entry, net_no);
                        }
                        0 => {
                            // Keep the obligation attached to the exact
                            // subnet created for this group. Keying by net
                            // name would broadcast it to every same-name
                            // subnet.
                            unresolved_net_endpoints
                                .entry(net_no)
                                .or_default()
                                .insert(pin_ref);
                        }
                        count => {
                            return Err(err(format!(
                                "network pin reference {pin_ref} is ambiguous across {count} placed pins"
                            )));
                        }
                    }
                }
            }
        }
    }

    // network classes: per-class trace width, clearance and via
    // ((class NAME net... (circuit (use_via V)) (rule (width W) ...)));
    // the application semantics live in the shared appliers, reused by
    // the standard `.rules` reader. Each class with an inline clearance
    // gets a matrix class NAMED after it (Java Network.add_clearance_rule),
    // so typed rules addressing the class name hit the right column.

    // named via infos and via rules declared in the network scope
    // ((via NAME PADSTACK [CLEARANCE_CLASS] [attach]) and
    //  (via_rule NAME VIA_INFO...)); net classes reference the rule by name.
    let mut via_rule_ids: HashMap<String, usize> = HashMap::new();
    let ctx = NetworkScopeCtx {
        padstack_nos: &padstack_nos,
        padstacks: &padstacks,
        layer_structure: &layer_structure,
    };
    for network in pcb.children("network") {
        for via_node in network.children("via") {
            if !apply_via_declaration(&mut rules, &ctx, via_node) {
                return Err(err("malformed or unbound network via declaration"));
            }
        }
        for rule_node in network.children("via_rule") {
            if !apply_via_rule_declaration(&mut rules, &mut via_rule_ids, rule_node) {
                return Err(err("malformed or unbound network via_rule declaration"));
            }
        }
    }

    // Java inserts every network class first, then resolves all class-pair
    // scopes.  Delaying `class_class` application is important when the two
    // classes live in different `(network ...)` scopes: applying a pair while
    // visiting the first network silently drops it because the second class
    // has not been created yet.
    let class_pair_nodes: Vec<&SExpr> = pcb
        .children("network")
        .flat_map(|network| network.children("class_class"))
        .collect();
    for network in pcb.children("network") {
        for class_node in network.children("class") {
            let mut class_args = class_node.args();
            let class_name = class_args
                .next()
                .ok_or_else(|| err("network class is missing its name"))?;
            for circuit in class_node.children("circuit") {
                for use_via in circuit.children("use_via") {
                    for padstack in use_via.args() {
                        if ctx.padstack_no(padstack).is_none() {
                            return Err(err(format!(
                                "network class {class_name:?} references unknown padstack {padstack:?}"
                            )));
                        }
                    }
                }
                for use_layer in circuit.children("use_layer") {
                    for layer in use_layer.args() {
                        if layer_structure.get_no(layer).is_none() {
                            return Err(err(format!(
                                "network class {class_name:?} references unknown layer {layer:?}"
                            )));
                        }
                    }
                }
            }
            for layer_rule in class_node.children("layer_rule") {
                for layer in layer_rule.args() {
                    if layer_structure.get_no(layer).is_none() {
                        return Err(err(format!(
                            "network class {class_name:?} has a rule for unknown layer {layer:?}"
                        )));
                    }
                }
            }
            if let Some(via_rule) = class_node.child("via_rule") {
                let rule_name = via_rule.arg().ok_or_else(|| {
                    err(format!(
                        "network class {class_name:?} has an empty via_rule"
                    ))
                })?;
                if !via_rule_ids.contains_key(rule_name) {
                    return Err(err(format!(
                        "network class {class_name:?} references unknown via_rule {rule_name:?}"
                    )));
                }
            }
            apply_class_scope(&mut rules, &ctx, class_node, &scale, &via_rule_ids);
        }
    }
    // Resolve class-level clearance references after every class declaration
    // has had a chance to create its named matrix column.  This both supports
    // forward references and prevents `apply_class_scope` from silently
    // ignoring an unknown explicit symbol.
    for network in pcb.children("network") {
        for class_node in network.children("class") {
            let Some(raw_name) = class_node.arg() else {
                continue;
            };
            let class_name = if raw_name.is_empty() {
                "default"
            } else {
                raw_name
            };
            let referenced = inherited_clearance_class(
                &rules,
                class_node,
                &format!("network class {class_name:?}"),
            )?;
            let has_inline_clearance = class_node
                .children("rule")
                .any(rule_has_clearance_declaration)
                || class_node.children("layer_rule").any(|layer_rule| {
                    layer_rule
                        .children("rule")
                        .any(rule_has_clearance_declaration)
                });
            if let (Some(clearance_class), false) = (referenced, has_inline_clearance) {
                let class_idx = rules
                    .net_classes
                    .get_by_name(class_name)
                    .ok_or_else(|| err(format!("network class {class_name:?} was not created")))?;
                rules
                    .net_classes
                    .get_mut(class_idx)
                    .set_trace_clearance_class(clearance_class);
            }
        }
    }
    // (class_class (classes A B) (rule (clearance C))): a pairwise
    // clearance between two net classes (Java Network.insert_class_pairs)
    for cc_node in class_pair_nodes {
        let classes = cc_node
            .child("classes")
            .ok_or_else(|| err("network class_class is missing its classes scope"))?;
        let names: Vec<&str> = classes.args().collect();
        if names.len() < 2 || names.iter().any(|name| name.is_empty()) {
            return Err(err("network class_class requires two named classes"));
        }
        for layer_rule in cc_node.children("layer_rule") {
            for layer in layer_rule.args() {
                if layer_structure.get_no(layer).is_none() {
                    return Err(err(format!(
                        "network class_class references unknown layer {layer:?}"
                    )));
                }
            }
        }
        apply_class_class_scope(&mut rules, cc_node, &scale);
    }

    // Java always has a usable default ViaInfo/rule, even when a DSN omits
    // explicit `(via ...)` declarations.  Build those from the library and
    // attach every still-unbound class to the resulting rule.  Class-local
    // rules above already created stricter per-class variants, so they are
    // left untouched.
    let default_net_class = rules.get_default_net_class();
    let default_via_class = rules
        .net_classes
        .get(default_net_class)
        .default_item_clearance_classes
        .get(crate::rules::ItemClass::Via);
    for ps_no in 1..=padstacks.count() {
        let Some(ps) = padstacks.get_by_no(ps_no) else {
            continue;
        };
        if ps.to_layer() <= ps.from_layer() {
            continue;
        }
        let has_default_info = (0..rules.via_infos.count()).any(|i| {
            let info = rules.via_infos.get(i);
            info.get_padstack() == ps_no && info.get_clearance_class() == default_via_class
        });
        if !has_default_info {
            let attach = rules.via_at_smd_allowed && ps.attach_allowed;
            let _ = rules.via_infos.add(crate::rules::ViaInfo::new(
                ps.name.clone(),
                ps_no,
                default_via_class,
                attach,
            ));
        }
    }
    if rules
        .net_classes
        .get(default_net_class)
        .get_via_rule()
        .is_none()
    {
        rules.create_default_via_rule(default_net_class, "default", &padstacks);
    }
    let default_via_rule = rules.net_classes.get(default_net_class).get_via_rule();
    if let Some(default_via_rule) = default_via_rule {
        for class_idx in 0..rules.net_classes.count() {
            if rules.net_classes.get(class_idx).get_via_rule().is_none() {
                rules
                    .net_classes
                    .get_mut(class_idx)
                    .set_via_rule(Some(default_via_rule));
            }
        }
    }

    let mut board = BasicBoard::new(layer_structure, rules, padstacks);
    board.resolution = resolution;
    board.unit = unit;
    for (net_no, endpoints) in unresolved_net_endpoints {
        for endpoint in endpoints {
            board.record_unresolved_net_endpoint(net_no, endpoint);
        }
    }

    // power planes: conduction areas connecting their net's pins
    // ((plane NET (polygon LAYER aperture x y ...)))
    for plane_node in structure.children("plane") {
        let net_name = plane_node
            .arg()
            .ok_or_else(|| err("plane is missing its net reference"))?;
        let polygon = plane_node
            .child("polygon")
            .ok_or_else(|| err(format!("plane {net_name:?} is missing its polygon")))?;
        let layer_name = polygon
            .arg()
            .ok_or_else(|| err(format!("plane {net_name:?} polygon is missing its layer")))?;
        let layer = board.layer_structure.get_no(layer_name).ok_or_else(|| {
            err(format!(
                "plane {net_name:?} references unknown layer {layer_name:?}"
            ))
        })?;
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
        if net_nos.is_empty() {
            return Err(err(format!("plane references unknown net {net_name:?}")));
        }
        // the net now carries a plane (Java DsnFile/Network set_contains_plane):
        // an explicit flag for interchange (KiCad JSON `containsPlane`) instead
        // of re-deriving it from whichever areas happen to be serialized
        for &no in &net_nos {
            if let Some(net) = board.rules.nets.get_by_no_mut(no) {
                net.set_contains_plane(true);
            }
        }
        let area = crate::geometry::planar::PolylineArea::new(
            crate::geometry::planar::PolygonShape::new(corners),
            Vec::new(),
        );
        // the plane uses its net class's Area item clearance class (Java
        // Network insert plane -> get(Area)), so a high-clearance net's copper
        // pour keeps its spacing (#2/#4)
        let explicit_plane_cl =
            structure_clearance_class(&board.rules, plane_node, &format!("plane {net_name:?}"))?;
        let plane_cl = explicit_plane_cl.unwrap_or_else(|| {
            board.rules.item_clearance_class_for(
                net_nos.first().copied().unwrap_or(0),
                crate::rules::ItemClass::Area,
            )
        });
        let id = board.insert_area(area, layer, net_name, net_nos, plane_cl, true);
        if explicit_plane_cl.is_some() {
            board.set_item_clearance_class_explicit(id, true);
        }
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
                .or_else(|| node.child("circ"))
                .or_else(|| node.child("path"))
            else {
                continue;
            };
            let layer_arg = shape_node.arg().unwrap_or("signal");
            let layers: Vec<usize> = match keepout_layer_reference(
                &board.layer_structure,
                layer_arg,
                &format!("structure {kind}"),
            )? {
                Some(layer) => vec![layer],
                None => (0..board.layer_structure.layer_count()).collect(),
            };
            let corners = keepout_corners(shape_node, &scale);
            if corners.len() < 3 {
                continue;
            }
            let polygon = crate::geometry::planar::PolygonShape::new(corners);
            if polygon.dimension() != 2 || polygon.is_empty() {
                continue;
            }
            let area = crate::geometry::planar::PolylineArea::new(polygon, Vec::new());
            let name = node.arg().unwrap_or(kind);
            // a keepout may name its clearance class (Java uses the keepout's
            // clearance class); default when absent.
            let explicit_clearance = structure_clearance_class(
                &board.rules,
                node,
                &format!("structure {kind} {name:?}"),
            )?;
            let keepout_cl = match explicit_clearance {
                Some(class) => class,
                None => board
                    .rules
                    .item_clearance_class_for(0, crate::rules::ItemClass::Area),
            };
            for layer in layers {
                let mut base = crate::board::ItemBase::new(0, Vec::new(), keepout_cl);
                base.clearance_class_explicit = explicit_clearance.is_some();
                let mut item =
                    crate::board::Item::new_obstacle_area(base, area.clone(), layer, name, false);
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
    // routes stay inside the board (Java: BoardOutline tree shapes).
    // EVERY (boundary ...) node is read — a design may declare separate
    // pcb- and signal-boundaries — and a boundary may name its clearance
    // class ((clearance_class ...)); polygons read like paths.
    //
    // The PRESERVED outline prefers the SIGNAL boundary: Java-generated
    // designs put the pcb bounding rectangle first and the actual outline
    // polygon (layer token "signal") after it, so first-wins exported the
    // bounding rectangle. The stored clearance is the boundary's resolved
    // class value, not blindly the default.
    let mut outline_from_signal = false;
    for boundary in structure.children("boundary") {
        let explicit_boundary_cl =
            structure_clearance_class(&board.rules, boundary, "structure boundary")?;
        let boundary_cl = match explicit_boundary_cl {
            Some(class) => class,
            None => board
                .rules
                .item_clearance_class_for(0, crate::rules::ItemClass::Area),
        };
        let boundary_cl_value = board
            .rules
            .clearance_matrix
            .get_value(boundary_cl, BoardRules::default_clearance_class(), 0, false)
            .max(0);
        let keep_outline = |board: &mut BasicBoard,
                            corners: &[IntPoint],
                            layer_token: &str,
                            from_signal: &mut bool| {
            let is_signal = layer_token.eq_ignore_ascii_case("signal");
            if corners.len() >= 3 && (board.outline.is_none() || (is_signal && !*from_signal)) {
                board.outline = Some((corners.to_vec(), boundary_cl_value));
                *from_signal = is_signal;
            }
        };
        if let Some(path) = boundary.child("path").or_else(|| boundary.child("polygon")) {
            let layer_token = path.arg().unwrap_or("").to_string();
            keepout_layer_reference(&board.layer_structure, &layer_token, "structure boundary")?;
            let coords: Vec<f64> = path.args().skip(2).filter_map(|a| a.parse().ok()).collect();
            let corners: Vec<IntPoint> = coords
                .chunks_exact(2)
                .map(|c| IntPoint::new(scale(c[0]), scale(c[1])))
                .collect();
            keep_outline(&mut board, &corners, &layer_token, &mut outline_from_signal);
            // the strip width follows the boundary's RESOLVED clearance
            // value, not blindly the board default
            insert_boundary_keepouts(&mut board, &corners, boundary_cl_value);
        } else if let Some(rect) = boundary.child("rect") {
            // (boundary (rect <layer> x1 y1 x2 y2)): a rectangular outline.
            // Previously only `path` boundaries produced keepouts, so
            // rect-outline boards were unconfined and routes could escape the
            // board. Java reads `rect` as a first-class boundary shape.
            let layer_token = rect.arg().unwrap_or("").to_string();
            keepout_layer_reference(&board.layer_structure, &layer_token, "structure boundary")?;
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
                keep_outline(&mut board, &corners, &layer_token, &mut outline_from_signal);
                insert_boundary_keepouts(&mut board, &corners, boundary_cl_value);
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
    // `ItemBase` stores a compact numeric component identity rather than the
    // DSN refdes.  Allocate one identity per `(place ...)`, not one global
    // sentinel: the KiCad writer groups pads by this field, so reusing `1`
    // would collapse every footprint in a multi-component board into one
    // component on round-trip.
    let mut next_component_no = 1i32;
    for placement in pcb.children("placement") {
        for component in placement.children("component") {
            let image_name = component
                .arg()
                .ok_or_else(|| err("placement component is missing its image name"))?;
            let image = images.get(image_name).ok_or_else(|| {
                err(format!(
                    "placement component references unknown image {image_name:?}"
                ))
            })?;
            let image_pins = &image.pins;
            for place in component.children("place") {
                let component_no = next_component_no;
                next_component_no = next_component_no.saturating_add(1).max(1);
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
                let mut pin_overrides: HashMap<&str, usize> = HashMap::new();
                for pin_node in place.children("pin") {
                    let Some(pin_name) = pin_node.arg() else {
                        continue;
                    };
                    if let Some(clearance_class) = inherited_clearance_class(
                        &board.rules,
                        pin_node,
                        &format!("placement {refdes:?} pin {pin_name:?}"),
                    )? {
                        if pin_overrides.insert(pin_name, clearance_class).is_some() {
                            return Err(err(format!(
                                "placement {refdes:?} has duplicate clearance overrides for pin {pin_name:?}"
                            )));
                        }
                    }
                }
                // Java keeps separate maps for ordinary, via-only, and
                // placement keepouts. The image template supplies the
                // fallback; a named `(keepout NAME (clearance_class C))` on
                // this place overrides only this instance.
                let mut keepout_overrides: HashMap<&str, usize> = HashMap::new();
                for keepout_node in place.children("keepout") {
                    let Some(name) = keepout_node.arg() else {
                        continue;
                    };
                    if let Some(clearance_class) = inherited_clearance_class(
                        &board.rules,
                        keepout_node,
                        &format!("placement {refdes:?} keepout {name:?}"),
                    )? {
                        keepout_overrides.insert(name, clearance_class);
                    }
                }
                let mut via_keepout_overrides: HashMap<&str, usize> = HashMap::new();
                for keepout_node in place.children("via_keepout") {
                    let Some(name) = keepout_node.arg() else {
                        continue;
                    };
                    if let Some(clearance_class) = inherited_clearance_class(
                        &board.rules,
                        keepout_node,
                        &format!("placement {refdes:?} via_keepout {name:?}"),
                    )? {
                        via_keepout_overrides.insert(name, clearance_class);
                    }
                }
                for pin in image_pins {
                    let padstack_no = board
                        .padstacks
                        .get(&pin.padstack_name)
                        .map(|padstack| padstack.no)
                        .ok_or_else(|| {
                            err(format!(
                                "image {image_name:?} pin {:?} references unknown padstack {:?}",
                                pin.pin_name, pin.padstack_name
                            ))
                        })?;
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
                                        // TileShape transforms intentionally
                                        // return a generic simplex, but the
                                        // raw constructor does not sort or
                                        // remove redundant borders. Normalize
                                        // each placed variant before it enters
                                        // the shared padstack library; without
                                        // this, rotated/back pads can look
                                        // two-dimensional while remaining
                                        // unbounded to the search tree.
                                        shapes[target] = Some(normalize_imported_tile_shape(s));
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
                    let pin_ref = LogicalEndpoint::new(refdes, &pin.pin_name);
                    let net_nos = pin_nets.get(&pin_ref).cloned().unwrap_or_default();
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
                    let resolved_explicit = pin_overrides.get(pin.pin_name.as_str()).copied();
                    let clearance_class = match resolved_explicit {
                        Some(cc) => cc,
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
                    board.set_component_no(id, component_no);
                    if resolved_explicit.is_some() {
                        board.set_item_clearance_class_explicit(id, true);
                    }
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
                    let override_clearance = if ko.via_only {
                        via_keepout_overrides.get(ko.name.as_str()).copied()
                    } else {
                        keepout_overrides.get(ko.name.as_str()).copied()
                    };
                    let clearance_class = override_clearance.unwrap_or(ko.clearance_class);
                    let clearance_class_explicit =
                        override_clearance.is_some() || ko.clearance_class_explicit;
                    for layer in layers {
                        let mut base = crate::board::ItemBase::new(0, Vec::new(), clearance_class);
                        base.clearance_class_explicit = clearance_class_explicit;
                        let mut item = crate::board::Item::new_obstacle_area(
                            base,
                            area.clone(),
                            layer,
                            &ko.name,
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
                    None => return Err(err("wiring wire is missing a path")),
                },
            };
            let layer_name = path
                .arg()
                .ok_or_else(|| err("wiring path is missing its layer"))?;
            let layer = board.layer_structure.get_no(layer_name).ok_or_else(|| {
                err(format!(
                    "wiring path references unknown layer {layer_name:?}"
                ))
            })?;
            let raw_numbers: Vec<&str> = path.args().skip(1).collect();
            if raw_numbers.len() < 5 || !(raw_numbers.len() - 1).is_multiple_of(2) {
                return Err(err(
                    "wiring path needs a width and at least two coordinate pairs",
                ));
            }
            let nums: Vec<f64> = raw_numbers
                .iter()
                .map(|atom| {
                    atom.parse::<f64>()
                        .map_err(|_| err(format!("wiring path contains non-numeric atom {atom:?}")))
                })
                .collect::<Result<_, _>>()?;
            if !nums[0].is_finite()
                || nums[0] <= 0.0
                || nums[1..].iter().any(|value| !value.is_finite())
            {
                return Err(err(
                    "wiring path width and coordinates must be finite; width must be positive",
                ));
            }
            if is_polyline_path && !(nums[1..].len()).is_multiple_of(4) {
                return Err(err("wiring polyline_path needs groups of four coordinates"));
            }
            let half_width = (scale(nums[0]) / 2).max(1);
            let net_nos = match wire_node.child("net") {
                Some(node) => net_numbers_for_reference(&board.rules, node).map_err(err)?,
                None => Vec::new(),
            };
            let fixed_state = wiring_fixed_state(wire_node);
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
                // Legacy DSN writers emit zero-length protection segments.
                // Their numeric payload is valid but carries no geometry;
                // preserve Java's compatibility behavior by ignoring only
                // this degenerate case (malformed atoms were rejected above).
                continue;
            }
            // an explicit wiring-level (clearance_class ...) wins (Java
            // Wiring.read_wire_scope); otherwise the wire keeps its net's
            // clearance class, not the default
            let explicit_clearance = wiring_clearance_class(&board, wire_node, "wiring wire")?;
            let clearance_class = explicit_clearance.unwrap_or_else(|| {
                net_nos
                    .first()
                    .map(|&n| board.rules.get_trace_clearance_class(n))
                    .unwrap_or_else(BoardRules::default_clearance_class)
            });
            let id = board.insert_trace(polyline, layer, half_width, net_nos, clearance_class);
            if explicit_clearance.is_some() {
                board.set_item_clearance_class_explicit(id, true);
            }
            board.set_fixed_state(id, fixed_state);
        }
        for via_node in wiring.children("via") {
            let args: Vec<&str> = via_node.args().collect();
            if args.len() != 3 {
                return Err(err(
                    "wiring via needs exactly a padstack name and x/y coordinates",
                ));
            }
            let padstack_no = board
                .padstacks
                .get(args[0])
                .map(|padstack| padstack.no)
                .ok_or_else(|| {
                    err(format!(
                        "wiring via references unknown padstack {:?}",
                        args[0]
                    ))
                })?;
            let x = args[1]
                .parse::<f64>()
                .map_err(|_| err("wiring via x coordinate is not numeric"))?;
            let y = args[2]
                .parse::<f64>()
                .map_err(|_| err("wiring via y coordinate is not numeric"))?;
            if !x.is_finite() || !y.is_finite() {
                return Err(err("wiring via coordinates must be finite"));
            }
            let net_nos = match via_node.child("net") {
                Some(node) => net_numbers_for_reference(&board.rules, node).map_err(err)?,
                None => Vec::new(),
            };
            let fixed_state = wiring_fixed_state(via_node);
            let explicit_clearance = wiring_clearance_class(&board, via_node, "wiring via")?;
            let clearance_class = explicit_clearance.unwrap_or_else(|| {
                net_nos
                    .first()
                    .map(
                        |&n| match board.rules.via_clearance_class_for_padstack(n, padstack_no) {
                            Some(0) => board.rules.get_trace_clearance_class(n),
                            Some(class) => class,
                            None => board
                                .rules
                                .item_clearance_class_for(n, crate::rules::ItemClass::Via),
                        },
                    )
                    .unwrap_or_else(BoardRules::default_clearance_class)
            });
            // A bound ViaInfo is the authoritative attach rule for this net
            // and padstack. Falling back to the board/padstack defaults is
            // only for legacy designs without a named ViaInfo; otherwise an
            // attach-off class silently reloads as attach-on copper.
            let attach = net_nos
                .first()
                .and_then(|&net| board.rules.via_info_for_padstack(net, padstack_no))
                .map(|info| info.attach_smd_allowed())
                .unwrap_or_else(|| {
                    board.rules.via_at_smd_allowed
                        && board
                            .padstacks
                            .get_by_no(padstack_no)
                            .is_some_and(|p| p.attach_allowed)
                });
            let id = board.insert_via(
                padstack_no,
                IntPoint::new(scale(x), scale(y)),
                net_nos,
                clearance_class,
                attach,
            );
            if explicit_clearance.is_some() {
                board.set_item_clearance_class_explicit(id, true);
            }
            board.set_fixed_state(id, fixed_state);
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
    if let Some(error) = scale_error.into_inner() {
        return Err(err(error));
    }
    Ok(board)
}

/// Inserts thin keepout strips along the closed outline given by
/// `corners` on every layer, so routes cannot cross the board boundary
/// (Java: the tree shapes of `BoardOutline`). Shared with the KiCad JSON
/// reader, whose `outline` object is the same closed corner list.
pub(crate) fn insert_boundary_keepouts(
    board: &mut BasicBoard,
    corners: &[IntPoint],
    desired_clearance: i32,
) {
    use crate::geometry::planar::{PolygonShape, PolylineArea};
    if corners.len() < 2 {
        return;
    }
    // Keep the synthetic geometry close to a zero-width outline and carry the
    // requested copper-to-edge distance in its matrix row.  The old code used
    // half the requested clearance as strip geometry *and* the full value in
    // the matrix, enforcing 1.5x the configured spacing and disagreeing with
    // the final DRC near custom-clearance outlines.
    const GEOMETRY_HALF_WIDTH: i32 = 1;
    let desired_clearance = desired_clearance.max(0);
    let matrix_clearance = desired_clearance.saturating_sub(2 * GEOMETRY_HALF_WIDTH);
    let mut class_name = "__boundary_geometry".to_string();
    let mut suffix = 2usize;
    while board.rules.clearance_matrix.get_no(&class_name).is_some() {
        class_name = format!("__boundary_geometry_{suffix}");
        suffix += 1;
    }
    board.rules.clearance_matrix.append_class(&class_name);
    let clearance_class = board
        .rules
        .clearance_matrix
        .get_no(&class_name)
        .unwrap_or_else(BoardRules::default_clearance_class);
    let class_count = board.rules.clearance_matrix.get_class_count();
    for other in 1..class_count {
        board.rules.clearance_matrix.set_value_on_all_layers(
            clearance_class,
            other,
            matrix_clearance,
        );
        board.rules.clearance_matrix.set_value_on_all_layers(
            other,
            clearance_class,
            matrix_clearance,
        );
    }
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
        let left = line.translate(GEOMETRY_HALF_WIDTH as f64);
        let right = line.translate(-(GEOMETRY_HALF_WIDTH as f64));
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
            let id = board.insert_area(
                area.clone(),
                layer,
                "boundary",
                vec![],
                clearance_class,
                false,
            );
            board.set_item_clearance_class_explicit(id, false);
            board.set_fixed_state(id, crate::board::FixedState::SystemFixed);
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
        // A malformed keepout such as `(polygon F.Cu)` has no aperture or
        // coordinates.  Treat it as an unusable shape instead of slicing an
        // empty vector and panicking during import.
        if nums.len() < 3 {
            return Vec::new();
        }
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
    } else if (kind.eq_ignore_ascii_case("circle") || kind.eq_ignore_ascii_case("circ"))
        && !nums.is_empty()
    {
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
        let (x0, x1) = non_degenerate_bounds(scale(nums[0]), scale(nums[2]));
        let (y0, y1) = non_degenerate_bounds(scale(nums[1]), scale(nums[3]));
        TileShape::Box(IntBox::from_coords(
            x0.min(x1),
            y0.min(y1),
            x0.max(x1),
            y0.max(y1),
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
        // Unions of octagons can leave redundant diagonal bounds that make
        // the raw intersection appear one-dimensional even though the
        // stroked path has area. Normalize after the union so tiny/zero-length
        // DSN paths remain valid two-dimensional pad geometry.
        TileShape::Octagon(oct?.normalize())
    } else if kind.eq_ignore_ascii_case("polygon") {
        // (polygon LAYER aperture x1 y1 ...): bounding box approximation
        if nums.len() < 5 {
            return None;
        }
        let xs: Vec<f64> = nums[1..].iter().step_by(2).copied().collect();
        let ys: Vec<f64> = nums[2..].iter().step_by(2).copied().collect();
        let (x0, x1) = non_degenerate_bounds(
            scale(xs.iter().cloned().fold(f64::MAX, f64::min)),
            scale(xs.iter().cloned().fold(f64::MIN, f64::max)),
        );
        let (y0, y1) = non_degenerate_bounds(
            scale(ys.iter().cloned().fold(f64::MAX, f64::min)),
            scale(ys.iter().cloned().fold(f64::MIN, f64::max)),
        );
        TileShape::Box(IntBox::from_coords(
            x0.min(x1),
            y0.min(y1),
            x0.max(x1),
            y0.max(y1),
        ))
    } else {
        return None;
    };
    Some((shape, layer_name))
}

/// Preserve a minimum two-dimensional footprint when a very small DSN shape
/// rounds to a zero-width integer interval at the board resolution. A one
/// board-unit half expansion is the same minimum used for circles/paths and
/// avoids silently turning a physical pad into a line or point.
fn non_degenerate_bounds(a: i32, b: i32) -> (i32, i32) {
    if a != b {
        return (a, b);
    }
    (a.saturating_sub(1), b.saturating_add(1))
}

/// Canonicalizes a shape produced by a component placement transform.
/// `TileShape::turn_90_degree` and `mirror_vertical` preserve the boundary
/// lines but intentionally construct a raw simplex; sorting and removing
/// redundant borders is required before boundedness/dimension queries are
/// meaningful.
fn normalize_imported_tile_shape(shape: TileShape) -> TileShape {
    match shape {
        TileShape::Box(box_shape) => TileShape::Box(box_shape),
        TileShape::Octagon(octagon) => TileShape::Octagon(octagon.normalize()),
        TileShape::Simplex(simplex) => TileShape::get_instance(simplex.border_lines().to_vec()),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn resolution_design(declaration: &str) -> String {
        format!(
            r#"(pcb "resolution.dsn"
  {declaration}
  (structure
    (layer F.Cu (type signal))
    (boundary (rect pcb 0 0 1000 1000))
    (rule (width 20) (clearance 20)))
  (placement)
  (library)
  (network))"#
        )
    }

    #[test]
    fn resolution_metadata_is_validated_before_scaling_the_board() {
        let omitted =
            import_dsn(&resolution_design("")).expect("omitted resolution uses DSN default");
        assert_eq!(omitted.unit, "um");
        assert_eq!(omitted.resolution, 1);

        let integral_decimal = import_dsn(&resolution_design("(resolution MIL 10.0)"))
            .expect("an integral numeric spelling is lossless");
        assert_eq!(integral_decimal.unit, "MIL");
        assert_eq!(integral_decimal.resolution, 10);

        for (declaration, expected) in [
            ("(resolution parsec 10)", "unsupported resolution unit"),
            ("(resolution)", "missing its physical unit"),
            ("(resolution um)", "missing its numeric value"),
            ("(resolution um 10 extra)", "exactly a unit and a value"),
            ("(resolution um not-a-number)", "is not numeric"),
            ("(resolution um NaN)", "finite and positive"),
            ("(resolution um inf)", "finite and positive"),
            ("(resolution um 0)", "finite and positive"),
            ("(resolution um -1)", "finite and positive"),
            ("(resolution um 1.5)", "must be an integer"),
            (
                "(resolution um 2147483648)",
                "exceeds the board coordinate limit",
            ),
        ] {
            let error = import_dsn(&resolution_design(declaration))
                .expect_err("invalid resolution metadata must fail closed")
                .to_string();
            assert!(
                error.contains(expected),
                "{declaration} produced unexpected error: {error}"
            );
        }

        let duplicate = resolution_design("(resolution um 10) (resolution mil 10)");
        let error = import_dsn(&duplicate)
            .expect_err("duplicate resolution scopes must not pick one implicitly")
            .to_string();
        assert!(
            error.contains("more than one resolution declaration"),
            "{error}"
        );
    }

    #[test]
    fn scaled_geometry_outside_board_integer_range_is_rejected() {
        let dsn = resolution_design("(resolution um 10)").replace(
            "(boundary (rect pcb 0 0 1000 1000))",
            "(boundary (rect pcb 0 0 300000000 1000))",
        );
        let error = import_dsn(&dsn)
            .expect_err("scaled coordinates must not saturate into a valid board")
            .to_string();
        assert!(
            error.contains("does not fit the board coordinate range"),
            "{error}"
        );
    }

    #[test]
    fn completed_dsn_graph_is_validated_before_exposure() {
        let duplicate_layers = resolution_design("").replace(
            "(layer F.Cu (type signal))",
            "(layer F.Cu (type signal)) (layer f.cu (type signal))",
        );
        let error = import_dsn(&duplicate_layers)
            .expect_err("duplicate layer identities must not escape the importer")
            .to_string();
        assert!(error.contains("layers[1].name"), "{error}");

        let empty_padstack =
            resolution_design("").replace("(library)", "(library (padstack \"empty\"))");
        let error = import_dsn(&empty_padstack)
            .expect_err("an empty padstack must not escape the importer")
            .to_string();
        assert!(error.contains("padstacks[1].shapes"), "{error}");
    }

    #[test]
    fn structural_pin_identities_preserve_unresolved_obligations() {
        let dsn = r#"(pcb "pins.dsn"
  (resolution um 10)
  (structure
    (layer F.Cu (type signal))
    (boundary (rect pcb 0 0 10000 10000))
    (rule (width 20) (clearance 20)))
  (placement
    (component "FP" (place "A-B" 1000 1000 front 0))
    (component "FP" (place "UNPLACED")))
  (library
    (image "FP" (pin "Pad" C 0 0))
    (padstack "Pad" (shape (rect F.Cu -50 -50 50 50)) (attach off)))
  (network
    (net N1 (pins "A-B"-C MISSING-1))
    (net N2 (pins A-"B-C"))
    (net N3 (pins UNPLACED-C))
    (net N4 (pins SPACED - "P-1"))))"#;
        let board = import_dsn(dsn).expect("Java-compatible unresolved pins must import");
        let n1 = board.rules.nets.get("N1", 1).unwrap().net_number;
        let n2 = board.rules.nets.get("N2", 1).unwrap().net_number;
        let n3 = board.rules.nets.get("N3", 1).unwrap().net_number;
        let n4 = board.rules.nets.get("N4", 1).unwrap().net_number;
        let physical_pin = board
            .items()
            .find(|(_, item)| item.base.component_no != 0)
            .map(|(_, item)| item)
            .expect("placed pin");
        assert_eq!(physical_pin.base.net_nos, vec![n1]);
        let unresolved: Vec<_> = board
            .unresolved_net_endpoints()
            .map(|(net, endpoint)| (net, endpoint.component.as_str(), endpoint.pin.as_str()))
            .collect();
        assert!(unresolved.contains(&(n1, "MISSING", "1")));
        assert!(unresolved.contains(&(n2, "A", "B-C")));
        assert!(unresolved.contains(&(n3, "UNPLACED", "C")));
        assert!(unresolved.contains(&(n4, "SPACED", "P-1")));
        assert_eq!(
            board
                .items()
                .filter(|(_, item)| item.base.component_no != 0)
                .count(),
            1,
            "the name-only place must not instantiate physical pins"
        );
        assert!(!board.net_is_completely_connected(n1));
        assert!(!board.net_is_completely_connected(n2));
        assert!(!board.net_is_completely_connected(n3));
        assert!(!board.net_is_completely_connected(n4));
    }

    #[test]
    fn malformed_known_numeric_scopes_fail_instead_of_defaulting_or_skipping() {
        for (dsn, expected) in [
            (
                resolution_design("").replace("(width 20)", "(width nope)"),
                "width contains non-numeric atom",
            ),
            (
                resolution_design("").replace(
                    "(boundary (rect pcb 0 0 1000 1000))",
                    "(boundary (rect pcb 0 nope 1000 1000))",
                ),
                "rect contains non-numeric atom",
            ),
            (
                resolution_design("").replace(
                    "(library)",
                    "(library (padstack P\n  (shape (rect F.Cu -10 -10 10 10))\n  (shape (rect F.Cu -10 bad 10 10))))",
                ),
                "shape contains non-numeric atom",
            ),
            (
                resolution_design("").replace("(network)", "(network (net N1 nope))"),
                "subnet \"nope\" is not an integer",
            ),
        ] {
            let error = import_dsn(&dsn)
                .expect_err("malformed known numeric scope must fail closed")
                .to_string();
            assert!(error.contains(expected), "unexpected error: {error}");
        }
    }

    #[test]
    fn normalizes_tiny_line_like_pad_shapes_to_area() {
        let scale = |value: f64| value.round() as i32;
        let polygon = parse_dsn("(polygon F.Cu 0 0 -3.5 0 3.5)").expect("polygon");
        let shape = read_pad_shape(&polygon, &scale).expect("polygon shape").0;
        assert_eq!(shape.dimension(), 2);
        assert!(!shape.is_empty());

        let path = parse_dsn("(path F.Cu 1 0 0 0 0)").expect("path");
        let shape = read_pad_shape(&path, &scale).expect("path shape").0;
        assert_eq!(shape.dimension(), 2);
        assert!(!shape.is_empty());
    }

    #[test]
    fn repeated_structure_widths_are_applied_in_document_order() {
        let dsn = r#"(pcb "width-order.dsn"
  (resolution um 10)
  (structure
    (layer F.Cu (type signal)
      (rule (width 300) (width 500)))
    (layer B.Cu (type signal))
    (boundary (rect pcb 0 0 50000 50000))
    (rule (width 100) (width 400) (clearance 200))
  )
  (placement)
  (library)
  (network
    (net "N1")
    (class "C" "N1"
      (rule (width 600) (width 800))
      (layer_rule F.Cu (rule (width 700) (width 900)))))
)"#;
        let board = import_dsn(dsn).expect("import");
        let default = board.rules.net_classes.get(0);
        assert_eq!(default.get_trace_half_width(0), 2500);
        assert_eq!(default.get_trace_half_width(1), 2000);
        let class = board
            .rules
            .net_classes
            .get(board.rules.net_classes.get_by_name("C").expect("class C"));
        assert_eq!(class.get_trace_half_width(0), 4500);
        assert_eq!(class.get_trace_half_width(1), 4000);
    }

    #[test]
    fn outline_prefers_the_signal_boundary() {
        // Issue413 declares the pcb bounding RECTANGLE first and the real
        // signal outline polygon second; first-wins exported the rectangle
        let root = concat!(env!("CARGO_MANIFEST_DIR"), "/..");
        let dsn = std::fs::read_to_string(format!("{root}/fixtures/Issue413-test.dsn"))
            .expect("fixture missing from checkout");
        let board = import_dsn(&dsn).expect("import");
        let (outline, _) = board.outline.clone().expect("outline preserved");
        // the signal outline spans exactly 0..600000 (x600000 file units,
        // scaled x10); the pcb bounding rect is 200 units larger all around
        let min_x = outline.iter().map(|p| p.x).min().unwrap();
        let max_x = outline.iter().map(|p| p.x).max().unwrap();
        assert_eq!(
            (min_x, max_x),
            (0, 6_000_000),
            "the SIGNAL boundary must be preserved, not the pcb rectangle"
        );
    }

    #[test]
    fn uppercase_wiring_is_stripped_and_not_duplicated_on_export() {
        // the review repro (Issue413 shape): a case-sensitive strip left
        // (WIRING ...) in the retained source, and the exporter then
        // emitted the copper twice — doubled copper, self-violations
        let dsn = r#"(pcb "up.dsn"
  (resolution um 10)
  (structure
    (layer F.Cu (type signal))
    (boundary (rect pcb 0 0 100000 100000))
    (rule (width 200) (clearance 200))
  )
  (placement)
  (library)
  (network (net "N1"))
  (WIRING
    (wire (path F.Cu 400 10000 10000 20000 20000) (net "N1") (type route))
  )
)"#;
        let board = import_dsn(dsn).expect("import");
        let traces = |b: &BasicBoard| {
            b.items()
                .filter(|(_, it)| matches!(it.kind, crate::board::ItemKind::PolylineTrace(_)))
                .count()
        };
        assert_eq!(traces(&board), 1, "the uppercase wiring imports once");
        assert!(
            !board
                .dsn_source
                .as_ref()
                .unwrap()
                .to_ascii_lowercase()
                .contains("(wiring"),
            "the retained source must not keep the uppercase wiring section"
        );
        let out = crate::io::dsn_export::export_dsn(&board).expect("export");
        let board2 = import_dsn(&out).expect("re-import");
        assert_eq!(traces(&board2), 1, "re-import must not double the copper");
    }

    #[test]
    fn wiring_surgery_honors_quoted_parentheses_and_trailing_comments() {
        let dsn = r#"(pcb "quoted.dsn"
  (resolution um 10)
  (structure
    (layer F.Cu (type signal))
    (boundary (rect pcb 0 0 100000 100000))
    (rule (width 200) (clearance 200))
  )
  (placement)
  (library)
  (network (net " )rail") (net ' (return)'))
  (wiring
    (wire (path F.Cu 400 10000 10000 20000 10000) (net " )rail") (type route))
    (wire (path F.Cu 400 10000 20000 20000 20000) (net ' (return)') (type route))
  )
)
# a trailing comment may contain a misleading close: )
"#;
        let board = import_dsn(dsn).expect("import");
        let trace_count = |candidate: &BasicBoard| {
            candidate
                .items()
                .filter(|(_, item)| matches!(item.kind, crate::board::ItemKind::PolylineTrace(_)))
                .count()
        };
        assert_eq!(trace_count(&board), 2);
        let out = crate::io::dsn_export::export_dsn(&board).expect("export");
        let reloaded = import_dsn(&out).expect("re-import");
        assert_eq!(trace_count(&reloaded), 2);
        assert!(!reloaded.rules.nets.get_by_name(" )rail").is_empty());
        assert!(!reloaded.rules.nets.get_by_name(" (return)").is_empty());
    }

    #[test]
    fn single_quoted_class_members_join_their_class() {
        // Issue721 shape: (class GND 'GND' ...) — the single-quoted member
        // must resolve to the net, not remain a 'GND' atom in no class
        let dsn = r#"(pcb "sq.dsn"
  (resolution um 10)
  (structure
    (layer F.Cu (type signal))
    (boundary (rect pcb 0 0 100000 100000))
    (rule (width 200) (clearance 200))
  )
  (placement)
  (library)
  (network
    (net 'GND')
    (class GND 'GND' (rule (width 400) (clearance 800)))
  )
)"#;
        let board = import_dsn(dsn).expect("import");
        let gnd = board.rules.nets.get_by_name("GND");
        assert!(!gnd.is_empty(), "single-quoted net name must parse");
        let class_idx = gnd[0].get_class();
        assert_eq!(
            board.rules.net_classes.get(class_idx).get_name(),
            "GND",
            "the single-quoted member must join its class"
        );
    }

    #[test]
    fn wiring_fixed_states_round_trip_through_export() {
        // (type shove_fixed) / (type fix) / (type protect) map to their
        // fixed states on import (Java Wiring.calc_fixed) and are written
        // back by the exporter, surviving the round trip
        let dsn = r#"(pcb "fs.dsn"
  (resolution um 10)
  (structure
    (layer F.Cu (type signal))
    (boundary (rect pcb 0 0 100000 100000))
    (rule (width 200) (clearance 200))
  )
  (placement)
  (library)
  (network (net "N1"))
  (wiring
    (wire (path F.Cu 400 10000 10000 20000 10000) (net "N1") (type shove_fixed))
    (wire (path F.Cu 400 10000 20000 20000 20000) (net "N1") (type protect))
    (wire (path F.Cu 400 10000 30000 20000 30000) (net "N1") (type fix))
    (wire (path F.Cu 400 10000 40000 20000 40000) (net "N1") (type route))
    (wire (path F.Cu 400 10000 50000 20000 50000) (net "N1"))
  )
)"#;
        let states = |b: &BasicBoard| -> Vec<crate::board::FixedState> {
            let mut v: Vec<_> = b
                .items()
                .filter(|(_, it)| matches!(it.kind, crate::board::ItemKind::PolylineTrace(_)))
                .map(|(_, it)| it.base.fixed_state)
                .collect();
            v.sort();
            v
        };
        let board = import_dsn(dsn).expect("import");
        use crate::board::FixedState::*;
        assert_eq!(
            states(&board),
            vec![Unfixed, ShoveFixed, UserFixed, UserFixed, SystemFixed]
        );
        let out = crate::io::dsn_export::export_dsn(&board).expect("export");
        let board2 = import_dsn(&out).expect("re-import");
        assert_eq!(
            states(&board2),
            states(&board),
            "fixed states must survive the DSN round trip"
        );
    }

    #[test]
    fn wiring_level_clearance_class_is_honored() {
        // an explicit (clearance_class ...) on a wire overrides the net's
        // class default (Java Wiring.read_wire_scope)
        let dsn = r#"(pcb "wcc.dsn"
  (resolution um 10)
  (structure
    (layer F.Cu (type signal))
    (boundary (rect pcb 0 0 100000 100000))
    (rule (width 200) (clearance 200))
    (rule (clearance 900 (type special_special)))
  )
  (placement)
  (library)
  (network (net "N1"))
  (wiring
    (wire (path F.Cu 400 10000 10000 20000 20000) (net "N1") (type route)
      (clearance_class special))
  )
)"#;
        let board = import_dsn(dsn).expect("import");
        let special = board
            .rules
            .clearance_matrix
            .get_no("special")
            .expect("typed rule creates the class");
        let wire_cc = board
            .items()
            .find(|(_, it)| matches!(it.kind, crate::board::ItemKind::PolylineTrace(_)))
            .map(|(_, it)| it.base.clearance_class)
            .expect("wire imported");
        assert_eq!(
            wire_cc, special,
            "the wire must carry its explicit clearance class"
        );
        // and the override must survive an export→import round trip: the
        // exporter writes (clearance_class ...) back (dropping it collapsed
        // the wire to the net's default class)
        let out = crate::io::dsn_export::export_dsn(&board).expect("export");
        let board2 = import_dsn(&out).expect("re-import");
        let special2 = board2
            .rules
            .clearance_matrix
            .get_no("special")
            .expect("class recreated on re-import");
        let wire_cc2 = board2
            .items()
            .find(|(_, it)| matches!(it.kind, crate::board::ItemKind::PolylineTrace(_)))
            .map(|(_, it)| it.base.clearance_class)
            .expect("wire re-imported");
        assert_eq!(
            wire_cc2, special2,
            "the explicit clearance class must survive the DSN round trip"
        );
    }

    #[test]
    fn class_use_layer_and_shove_fixed_are_imported() {
        // (circuit (use_layer ...)) leaves ONLY the listed layers active and
        // zeroes the width on inactive layers (Java
        // Network.create_active_trace_layers); the remaining class state is
        // read from the same standard scopes used by `.rules` sidecars.
        let dsn = r#"(pcb "ul.dsn"
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
    (net "TOP1")
    (class toponly "TOP1"
      (circuit (use_layer F.Cu) (length 987.6 123.4))
      (shove_fixed on)
      (pull_tight off)
      (rule (width 300) (clearance 200))
    )
  )
)"#;
        let board = import_dsn(dsn).expect("import");
        let net = &board.rules.nets.get_by_name("TOP1")[0];
        let class = board.rules.net_classes.get(net.get_class());
        assert!(class.is_active_routing_layer(0), "F.Cu stays active");
        assert!(!class.is_active_routing_layer(1), "B.Cu must be inactive");
        assert_eq!(
            class.get_trace_half_width(1),
            0,
            "inactive layers carry width 0"
        );
        assert!(class.get_trace_half_width(0) > 0);
        assert!(class.is_shove_fixed(), "(shove_fixed on) must be recorded");
        assert!(!class.get_pull_tight(), "(pull_tight off) must be recorded");
        assert_eq!(class.get_minimum_trace_length(), 1234.0);
        assert_eq!(class.get_maximum_trace_length(), 9876.0);
        let n = net.net_number;
        assert!(board.rules.is_active_routing_layer(n, 0));
        assert!(!board.rules.is_active_routing_layer(n, 1));
        // the single-width router samples the max over ACTIVE layers
        assert_eq!(board.rules.get_trace_half_width_max_active(n), 1500);
    }

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
    (keepout "cutout"
      (polygon F.Cu 0 40000 40000 60000 40000 60000 60000 40000 60000)
      (clearance_class strict))
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
        let cutout = board
            .items()
            .find(|(_, item)| {
                matches!(&item.kind, ItemKind::ObstacleArea(area) if area.name == "cutout")
            })
            .map(|(_, item)| item)
            .expect("named keepout");
        assert!(
            cutout.base.clearance_class_explicit,
            "an explicit keepout class must remain pinned during sidecar reconciliation"
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
    fn malformed_polygon_keepout_is_ignored_without_panicking() {
        let dsn = r#"(pcb "malformed-keepout.dsn"
  (resolution um 10)
  (structure
    (layer F.Cu (type signal))
    (layer B.Cu (type signal))
    (boundary (rect pcb 0 0 100000 100000))
    (keepout "broken" (polygon F.Cu))
    (rule (width 200) (clearance 200))
  )
  (placement)
  (library)
  (network (net "N1"))
)"#;
        let board = import_dsn(dsn).expect("malformed keepout should not abort the board");
        assert!(!board.items().any(|(_, item)| {
            matches!(&item.kind, crate::board::ItemKind::ObstacleArea(area)
                if area.name == "broken")
        }));
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
    fn placement_keepout_clearance_overrides_are_applied_by_name() {
        let dsn = r#"(pcb "placement-keepout.dsn"
  (resolution um 10)
  (structure
    (layer F.Cu (type signal))
    (layer B.Cu (type signal))
    (boundary (rect pcb 0 0 200000 200000))
    (rule (width 200) (clearance 200))
    (rule (clearance 800 (type default_strict)))
  )
  (placement
    (component "FP"
      (place U1 50000 50000 front 0
        (keepout body (clearance_class strict))
        (via_keepout drill (clearance_class strict)))))
  (library
    (image "FP"
      (pin "P" 1 0 0)
      (keepout body (rect F.Cu -1000 -1000 1000 1000))
      (via_keepout drill (rect signal -500 -500 500 500)))
    (padstack "P"
      (shape (rect F.Cu -500 -500 500 500))
      (attach off)))
  (network (net N1 (pins U1-1))))"#;
        let board = import_dsn(dsn).expect("placement keepout design imports");
        let strict = board
            .rules
            .clearance_matrix
            .get_no("strict")
            .expect("typed rule creates strict class");
        let areas: Vec<_> = board
            .items()
            .filter_map(|(_, item)| match &item.kind {
                crate::board::ItemKind::ObstacleArea(area)
                    if area.name == "body" || area.name == "drill" =>
                {
                    Some((area.name.as_str(), area.via_only, item.base.clearance_class))
                }
                _ => None,
            })
            .collect();
        assert_eq!(areas.len(), 3, "body on F.Cu plus drill on both layers");
        assert!(areas
            .iter()
            .filter(|(name, _, _)| *name == "body")
            .all(|(_, via_only, class)| !via_only && *class == strict));
        assert!(areas
            .iter()
            .filter(|(name, _, _)| *name == "drill")
            .all(|(_, via_only, class)| *via_only && *class == strict));
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
    (rule (clearance 200 (type default_Power)))
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
    fn prerouted_via_uses_bound_via_info_attach_rule() {
        let dsn = r#"(pcb "attach.dsn"
  (resolution um 10)
  (structure
    (layer F.Cu (type signal))
    (layer B.Cu (type signal))
    (boundary (rect pcb 0 0 200000 200000))
    (control (via_at_smd on))
    (rule (width 200) (clearance 200))
  )
  (placement)
  (library
    (padstack "Via1"
      (shape (circle F.Cu 600 0 0))
      (shape (circle B.Cu 600 0 0))
      (attach on)
    )
  )
  (network
    (net "N")
    (via "NoAttach" "Via1" default)
    (via_rule "StrictVias" "NoAttach")
    (class "strict" "N" (via_rule "StrictVias"))
  )
  (wiring
    (via "Via1" 1000 1000 (net "N") (type route))
  )
)"#;
        let board = import_dsn(dsn).expect("import");
        let route_via = board
            .items()
            .find_map(|(_, item)| match &item.kind {
                crate::board::ItemKind::Via(via) if item.base.component_no == 0 => Some(via),
                _ => None,
            })
            .expect("pre-routed via");
        assert!(
            !route_via.attach_allowed,
            "the selected ViaInfo must override the permissive global and padstack defaults"
        );
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
    fn preserves_subnets_fromto_groups_and_wiring_references() {
        let dsn = r#"(pcb "subnets.dsn"
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
    (net "GND" 2
      (fromto A-1 B-1)
      (fromto B-1 C-1))
  )
  (wiring
    (wire (path F.Cu 200 1000 1000 2000 1000) (net "GND" 3)))
)"#;
        let board = import_dsn(dsn).expect("subnet design imports");
        assert!(board.rules.nets.get("GND", 1).is_none());
        let subnet2 = board.rules.nets.get("GND", 2).expect("first fromto subnet");
        let subnet3 = board
            .rules
            .nets
            .get("GND", 3)
            .expect("second fromto subnet");
        let routed = board
            .items()
            .find_map(|(_, item)| match &item.kind {
                crate::board::ItemKind::PolylineTrace(_) => Some(item),
                _ => None,
            })
            .expect("subnet wiring");
        assert_eq!(routed.base.net_nos, vec![subnet3.net_number]);

        let exported = crate::io::dsn_export::export_dsn(&board).expect("export");
        assert!(
            exported.contains("(net \"GND\" 3)"),
            "wiring references must retain the subnet number"
        );
        let reloaded = import_dsn(&exported).expect("re-import");
        let reloaded_trace = reloaded
            .items()
            .find_map(|(_, item)| match &item.kind {
                crate::board::ItemKind::PolylineTrace(_) => Some(item),
                _ => None,
            })
            .expect("reloaded subnet wiring");
        assert_eq!(
            reloaded_trace.base.net_nos,
            vec![reloaded.rules.nets.get("GND", 3).unwrap().net_number]
        );
        assert_eq!(subnet2.subnet_number, 2);
        let unresolved2: Vec<_> = board
            .unresolved_net_endpoints()
            .filter(|(net_no, _)| *net_no == subnet2.net_number)
            .map(|(_, endpoint)| (endpoint.component.as_str(), endpoint.pin.as_str()))
            .collect();
        let unresolved3: Vec<_> = board
            .unresolved_net_endpoints()
            .filter(|(net_no, _)| *net_no == subnet3.net_number)
            .map(|(_, endpoint)| (endpoint.component.as_str(), endpoint.pin.as_str()))
            .collect();
        assert_eq!(unresolved2, vec![("A", "1"), ("B", "1")]);
        assert_eq!(unresolved3, vec![("B", "1"), ("C", "1")]);
        assert_eq!(board.unresolved_net_endpoint_count(subnet2.net_number), 2);
        assert_eq!(board.unresolved_net_endpoint_count(subnet3.net_number), 2);
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
    fn imports_circ_keepouts_with_trailing_clearance_class() {
        use crate::board::ItemKind;
        // Issue143's USB-MINIB images declare their connector keepouts as
        // Eagle-dialect (keepout (circ signal ...)) with the
        // (clearance_class boundary) as the FOLLOWING sibling
        let root = concat!(env!("CARGO_MANIFEST_DIR"), "/..");
        let path = format!("{root}/fixtures/Issue143-rpi_splitter.dsn");
        let content = std::fs::read_to_string(&path).expect("fixture missing from checkout");
        let board = import_dsn(&content).expect("import failed");
        let boundary_class = board
            .rules
            .clearance_matrix
            .get_no("boundary")
            .expect("boundary clearance class");
        // 2 keepouts per USB-MINIB placement x 2 placements, on every layer
        let keepouts: Vec<_> = board
            .items()
            .filter(|(_, it)| {
                matches!(&it.kind, ItemKind::ObstacleArea(a) if !a.is_conduction && a.name != "boundary")
            })
            .collect();
        assert!(
            keepouts.len() >= 4,
            "connector circ keepouts not instantiated (got {})",
            keepouts.len()
        );
        assert!(
            keepouts
                .iter()
                .all(|(_, it)| it.base.clearance_class == boundary_class),
            "keepouts must carry their named clearance class"
        );
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
    fn class_scoped_same_net_clearance_reaches_drc_table_once() {
        use crate::rules::ItemClass;
        let dsn = r#"(pcb "same-net.dsn"
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
    (net "N1")
    (class strict "N1"
      (rule (clearance 70 (type via_via_same_net))))
  )
)"#;
        let board = import_dsn(dsn).expect("import");
        assert_eq!(
            board
                .rules
                .get_same_net_clearance(ItemClass::Via, ItemClass::Via),
            Some(700),
            "class-scoped via_via_same_net must be applied"
        );
        assert!(
            board
                .rules
                .clearance_matrix
                .get_no("via_via_same_net")
                .is_none(),
            "special same-net token must not create a matrix class"
        );
    }

    #[test]
    fn class_pairs_are_resolved_after_all_network_classes() {
        let dsn = r#"(pcb "cross-network-class-pair.dsn"
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
    (net "N1")
    (class A "N1" (rule (clearance 300)))
    (class_class (classes A B) (rule (clearance 900)))
  )
  (network
    (net "N2")
    (class B "N2" (rule (clearance 400)))
  )
)"#;
        let board = import_dsn(dsn).expect("import");
        let a = board.rules.clearance_matrix.get_no("A").expect("A class");
        let b = board.rules.clearance_matrix.get_no("B").expect("B class");
        assert_eq!(
            board.rules.clearance_matrix.get_value(a, b, 0, false),
            9000,
            "class_class must see classes declared in a later network"
        );
    }

    #[test]
    fn mixed_wire_clearance_does_not_rebind_net_class_trace_classes() {
        let dsn = r#"(pcb "mixed-wire.dsn"
  (resolution um 10)
  (structure
    (layer F.Cu (type signal))
    (layer B.Cu (type signal))
    (boundary (rect pcb 0 0 100000 100000))
    (rule
      (width 200)
      (clearance 200)
      (clearance 250 (type base_a_base_a))
      (clearance 350 (type base_b_base_b))
    )
  )
  (placement)
  (library)
  (network
    (net "N1")
    (class A "N1" (clearance_class base_a))
    (net "N2")
    (class B "N2" (clearance_class base_b))
    (class_class (classes A B) (rule (clearance 900 (type wire_wire))))
  )
)"#;
        let board = import_dsn(dsn).expect("import");
        let base_a = board
            .rules
            .clearance_matrix
            .get_no("base_a")
            .expect("base_a class");
        let base_b = board
            .rules
            .clearance_matrix
            .get_no("base_b")
            .expect("base_b class");
        let n1 = board.rules.nets.get_by_name("N1")[0].net_number;
        let n2 = board.rules.nets.get_by_name("N2")[0].net_number;
        assert_eq!(
            board.rules.get_trace_clearance_class(n1),
            base_a,
            "mixed wire rule must not rebind class A's trace clearance"
        );
        assert_eq!(
            board.rules.get_trace_clearance_class(n2),
            base_b,
            "mixed wire rule must not rebind class B's trace clearance"
        );
        let a = board.rules.clearance_matrix.get_no("A").expect("A class");
        let b = board.rules.clearance_matrix.get_no("B").expect("B class");
        assert_eq!(
            board.rules.clearance_matrix.get_value(a, b, 0, false),
            9000,
            "mixed wire pair must still update its matrix endpoints"
        );
    }

    #[test]
    fn typed_class_clearance_precedes_clearance_class_reference() {
        let dsn = r#"(pcb "typed-class-reference.dsn"
  (resolution um 10)
  (structure
    (layer F.Cu (type signal))
    (layer B.Cu (type signal))
    (boundary (rect pcb 0 0 100000 100000))
    (rule
      (width 200)
      (clearance 200)
      (clearance 250 (type base_base))
    )
  )
  (placement)
  (library)
  (network
    (net "N1")
    (class C "N1"
      (clearance_class base)
      (rule (clearance 300 (type smd_to_turn_gap)))
    )
  )
)"#;
        let board = import_dsn(dsn).expect("import");
        let net = board.rules.nets.get_by_name("N1")[0].net_number;
        let trace_class = board.rules.get_trace_clearance_class(net);
        let class_column = board
            .rules
            .clearance_matrix
            .get_no("C")
            .expect("typed class column");
        let referenced = board
            .rules
            .clearance_matrix
            .get_no("base")
            .expect("referenced class column");
        assert_eq!(trace_class, class_column);
        assert_ne!(
            trace_class, referenced,
            "an inline typed rule must outrank the class-level reference"
        );
        assert_eq!(board.rules.get_pin_edge_to_turn_dist(), 3000.0);
    }

    #[test]
    fn network_via_padstack_lookup_is_case_insensitive() {
        let dsn = r#"(pcb "via-case.dsn"
  (resolution um 10)
  (structure
    (layer F.Cu (type signal))
    (layer B.Cu (type signal))
    (boundary (rect pcb 0 0 100000 100000))
    (rule (width 200) (clearance 200))
  )
  (placement)
  (library
    (padstack "ViaPad" (shape (circle F.Cu 600 0 0)) (shape (circle B.Cu 600 0 0)))
  )
  (network
    (net "N1")
    (via "V1" "viapad" default)
    (via_rule "VR" "V1")
    (class C "N1" (via_rule "VR"))
  )
)"#;
        let board = import_dsn(dsn).expect("import");
        let info = board
            .rules
            .via_infos
            .get_by_name("V1")
            .expect("via declaration must bind despite case difference");
        let padstack = board
            .padstacks
            .get_by_no(board.rules.via_infos.get(info).get_padstack())
            .expect("resolved padstack");
        assert_eq!(padstack.name, "ViaPad");
        let class = board.rules.net_classes.get_by_name("C").expect("class C");
        assert!(board.rules.net_classes.get(class).get_via_rule().is_some());
    }

    #[test]
    fn untyped_class_clearance_sets_self_exactly_and_cross_pairs_by_max() {
        let dsn = r#"(pcb "class-clearance-order.dsn"
  (resolution um 10)
  (structure
    (layer F.Cu (type signal))
    (boundary (rect pcb 0 0 100000 100000))
    (rule (width 200) (clearance 200))
  )
  (placement)
  (library)
  (network
    (net "N1")
    (class A "N1"
      (rule (clearance 1200 (type wire_via)))
      (rule (clearance 500)))
  )
)"#;
        let board = import_dsn(dsn).expect("import");
        let a = board.rules.clearance_matrix.get_no("A").expect("A class");
        let via = board
            .rules
            .clearance_matrix
            .get_no("A-via")
            .expect("A-via class");
        assert_eq!(
            board.rules.clearance_matrix.get_value(a, a, 0, false),
            5000,
            "the later untyped class clearance must set its own cell exactly"
        );
        assert_eq!(
            board.rules.clearance_matrix.get_value(a, via, 0, false),
            12000,
            "a stricter existing cross-pair must not be weakened"
        );
    }

    #[test]
    fn rejects_non_pcb() {
        assert!(import_dsn("(session x)").is_err());
        assert!(import_dsn("(pcb x)").is_err()); // no structure
    }
}
