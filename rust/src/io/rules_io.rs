//! Port of `RulesWriter.java` / `RulesReader.java`: persisting the board
//! design rules in the standard Specctra `(rules PCB ...)` format — snap
//! angle, the default width/clearance rule with its typed clearances,
//! via padstacks, `(via ...)`/`(via_rule ...)` declarations, and the net
//! classes with membership, clearance-class/via-rule references and
//! circuit rules. The reader applies the same grammar through the shared
//! network-scope appliers used by the DSN importer, so a standard
//! Freerouting `.rules` file (e.g. Issue593's) restores its classes, via
//! rules and typed clearances instead of only the global defaults.

use std::cell::RefCell;
use std::collections::{HashMap, HashSet};

use crate::board::basic_board::BasicBoard;
use crate::board::AngleRestriction;
use crate::io::dsn::parse_dsn;
use crate::io::dsn_import::{
    apply_class_class_scope, apply_class_scope, apply_rule_scope_clearances,
    apply_rule_scope_clearances_on_layer, apply_via_declaration, apply_via_rule_declaration,
    NetworkScopeCtx,
};
use crate::rules::ItemClass;

fn item_class_token(ic: ItemClass) -> Option<&'static str> {
    match ic {
        ItemClass::Via => Some("via"),
        ItemClass::Pin => Some("pin"),
        ItemClass::Smd => Some("smd"),
        ItemClass::Area => Some("area"),
        ItemClass::Trace => Some("wire"),
        ItemClass::None => None,
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct RulesWriteError {
    identifier: String,
    context: &'static str,
}

impl std::fmt::Display for RulesWriteError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "Specctra cannot represent {} identifier {:?} losslessly",
            self.context, self.identifier
        )
    }
}

impl std::error::Error for RulesWriteError {}

fn unrepresentable(value: &str, context: &'static str) -> RulesWriteError {
    RulesWriteError {
        identifier: value.to_string(),
        context,
    }
}

/// Writes one ordinary Specctra identifier without changing its value. Both
/// quote characters are valid delimiters in Java's scanner, but Specctra has
/// no escape syntax; a value containing both delimiters is therefore rejected
/// instead of being silently changed or written in a private dialect.
fn dsn_identifier(value: &str) -> Result<String, RulesWriteError> {
    let quote = if !value.contains('"') {
        '"'
    } else if !value.contains('\'') {
        '\''
    } else {
        return Err(unrepresentable(value, "ordinary"));
    };
    Ok(format!("{quote}{value}{quote}"))
}

/// Java's clearance-pair reader has a separate raw-string path that only
/// recognizes double-quoted tokens and later strips every double quote. A
/// class name containing `"` cannot round-trip through that grammar.
fn dsn_clearance_identifier(value: &str) -> Result<String, RulesWriteError> {
    if value.contains('"') {
        return Err(unrepresentable(value, "clearance-class"));
    }
    Ok(format!("\"{value}\""))
}

#[derive(Debug, Clone)]
struct MatrixColumn {
    name: String,
    source: usize,
}

const SERIALIZED_ITEM_CLASSES: [(&str, ItemClass); 5] = [
    ("wire", ItemClass::Trace),
    ("via", ItemClass::Via),
    ("pin", ItemClass::Pin),
    ("smd", ItemClass::Smd),
    ("area", ItemClass::Area),
];

fn matrix_sources_are_equivalent(
    matrix: &crate::rules::ClearanceMatrix,
    first: usize,
    second: usize,
) -> bool {
    (0..matrix.get_layer_count()).all(|layer| {
        (0..matrix.get_class_count()).all(|other| {
            matrix.get_value(first, other, layer, false)
                == matrix.get_value(second, other, layer, false)
                && matrix.get_value(other, first, layer, false)
                    == matrix.get_value(other, second, layer, false)
        })
    })
}

/// Builds the matrix namespace written to a rules file. Class-local typed
/// rules can bind item kinds only to the conventional `CLASS-via`,
/// `CLASS-pin`, ... names, so aliases are emitted for arbitrary in-memory
/// bindings and copy the complete source row. A colliding alias with different
/// semantics cannot be represented by the standard grammar and is rejected.
fn serialization_columns(board: &BasicBoard) -> Result<Vec<MatrixColumn>, RulesWriteError> {
    let matrix = &board.rules.clearance_matrix;
    if board.rules.net_classes.count() == 0 {
        return Err(unrepresentable(
            "<board>",
            "the board has no net class to serialize",
        ));
    }
    if matrix.get_class_count() == 0 || matrix.get_layer_count() == 0 {
        return Err(unrepresentable(
            "<board>",
            "the clearance matrix has no classes or layers",
        ));
    }
    let default_net_class_name = board.rules.net_classes.get(0).get_name().to_string();
    for class_idx in 0..board.rules.net_classes.count() {
        let name = board.rules.net_classes.get(class_idx).get_name();
        let reserved = ["wire", "null", "via", "pin", "smd", "area"]
            .iter()
            .any(|reserved| name.eq_ignore_ascii_case(reserved));
        if reserved || (name.eq_ignore_ascii_case("default") && name != default_net_class_name) {
            return Err(unrepresentable(name, "reserved net-class name"));
        }
    }
    // Class 0 is the Specctra `null` class and is deliberately absent from
    // every serialized `(type ...)` pair.  Reject a matrix that gives it any
    // semantic clearance instead of silently dropping that state.
    for layer in 0..matrix.get_layer_count() {
        for other in 0..matrix.get_class_count() {
            if matrix.get_value(0, other, layer, false) != 0
                || matrix.get_value(other, 0, layer, false) != 0
            {
                let other_name = matrix.get_name(other).unwrap_or("?");
                return Err(unrepresentable(
                    &format!("null <-> {other_name} on layer {layer}"),
                    "nonzero null clearance-matrix",
                ));
            }
        }
    }
    if (2..matrix.get_class_count()).any(|source| {
        matrix
            .get_name(source)
            .is_some_and(|name| name.eq_ignore_ascii_case("wire"))
    }) {
        return Err(unrepresentable("wire", "reserved clearance-class name"));
    }
    for layer in 0..matrix.get_layer_count() {
        for first in 0..matrix.get_class_count() {
            for second in (first + 1)..matrix.get_class_count() {
                if matrix.get_value(first, second, layer, false)
                    != matrix.get_value(second, first, layer, false)
                {
                    let first_name = matrix.get_name(first).unwrap_or("?");
                    let second_name = matrix.get_name(second).unwrap_or("?");
                    return Err(unrepresentable(
                        &format!("{first_name} -> {second_name} on layer {layer}"),
                        "asymmetric clearance-matrix",
                    ));
                }
            }
        }
    }

    let mut columns: Vec<MatrixColumn> = (1..matrix.get_class_count())
        .map(|source| MatrixColumn {
            name: matrix.get_name(source).unwrap_or("default").to_string(),
            source,
        })
        .collect();

    for class_idx in 0..board.rules.net_classes.count() {
        let class = board.rules.net_classes.get(class_idx);
        let trace_source = class.get_trace_clearance_class();
        for (token, item_class) in SERIALIZED_ITEM_CLASSES {
            let alias = if item_class == ItemClass::Trace {
                class.get_name().to_string()
            } else {
                format!("{}-{token}", class.get_name())
            };
            let source = if item_class == ItemClass::Trace {
                trace_source
            } else {
                class.default_item_clearance_classes.get(item_class)
            };
            if source >= matrix.get_class_count() {
                return Err(unrepresentable(
                    &format!("{} -> {source}", class.get_name()),
                    "out-of-range item clearance binding",
                ));
            }
            if let Some(existing) = columns
                .iter()
                .find(|column| column.name.eq_ignore_ascii_case(&alias))
            {
                if !matrix_sources_are_equivalent(matrix, existing.source, source) {
                    return Err(unrepresentable(
                        &alias,
                        if item_class == ItemClass::Trace {
                            "colliding net-class trace clearance binding"
                        } else {
                            "colliding net-class item clearance binding"
                        },
                    ));
                }
            } else {
                columns.push(MatrixColumn {
                    name: alias,
                    source,
                });
            }
        }
    }
    Ok(columns)
}

fn write_matrix_pair(
    out: &mut String,
    indent: &str,
    scale_out: &dyn Fn(i32) -> f64,
    value: i32,
    first: &str,
    second: &str,
) -> Result<(), RulesWriteError> {
    out.push_str(&format!(
        "{indent}(clearance {} (type {} {}))\n",
        scale_out(value),
        dsn_clearance_identifier(first)?,
        dsn_clearance_identifier(second)?,
    ));
    Ok(())
}

fn class_item_source(class: &crate::rules::NetClass, item_class: ItemClass) -> usize {
    if item_class == ItemClass::Trace {
        class.get_trace_clearance_class()
    } else {
        class.default_item_clearance_classes.get(item_class)
    }
}

fn item_rule_class(
    rules: &crate::rules::BoardRules,
    net_no: i32,
    item_class: ItemClass,
    via_padstack: Option<usize>,
) -> usize {
    if item_class == ItemClass::Trace {
        rules.get_trace_clearance_class(net_no)
    } else if let Some(padstack) = via_padstack {
        match rules.via_clearance_class_for_padstack(net_no, padstack) {
            Some(0) => rules.get_trace_clearance_class(net_no),
            Some(class) => class,
            None => rules.item_clearance_class_for(net_no, item_class),
        }
    } else {
        rules.item_clearance_class_for(net_no, item_class)
    }
}

fn board_item_class(board: &BasicBoard, item: &crate::board::Item) -> ItemClass {
    match &item.kind {
        crate::board::ItemKind::PolylineTrace(_) => ItemClass::Trace,
        crate::board::ItemKind::ObstacleArea(_) => ItemClass::Area,
        crate::board::ItemKind::Via(_) if item.base.component_no == 0 => ItemClass::Via,
        crate::board::ItemKind::Via(_) => {
            if item.first_layer(&board.padstacks) == item.last_layer(&board.padstacks) {
                ItemClass::Smd
            } else {
                ItemClass::Pin
            }
        }
    }
}

/// Checks the index-based rule references before any text is emitted.
///
/// The Rust model stores ViaInfo, padstack, clearance-class and ViaRule
/// references as integer indices.  Those indices are normally maintained by
/// the mutating APIs, but a board assembled by an importer, a caller, or a
/// future transformation can still contain a stale reference.  The writer
/// must fail closed in that case: blindly indexing a ViaInfos/ViaRules list
/// panics, while the older `get(...).and_then(...)` paths silently omitted a
/// declaration and changed the routing rules on reload.
fn validate_via_references(board: &BasicBoard) -> Result<(), RulesWriteError> {
    let matrix = &board.rules.clearance_matrix;
    let layer_count = board.layer_structure.layer_count();

    let mut via_info_names = HashSet::new();
    for info_id in 0..board.rules.via_infos.count() {
        let info = board.rules.via_infos.get(info_id);
        let info_name = info.get_name();
        if !via_info_names.insert(info_name.to_string()) {
            return Err(unrepresentable(info_name, "duplicate via-info"));
        }

        let padstack_no = info.get_padstack();
        let Some(padstack) = board.padstacks.get_by_no(padstack_no) else {
            return Err(unrepresentable(
                &format!("{info_name} -> padstack {padstack_no}"),
                "via-info padstack reference",
            ));
        };
        if padstack.board_layer_count() != layer_count {
            return Err(unrepresentable(
                &format!(
                    "{info_name} -> padstack {padstack_no} has {} layers (board has {layer_count})",
                    padstack.board_layer_count()
                ),
                "via-info padstack layer span",
            ));
        }
        if padstack.bounding_box().is_empty() {
            return Err(unrepresentable(info_name, "via-info empty padstack"));
        }

        let clearance_class = info.get_clearance_class();
        if matrix.get_name(clearance_class).is_none() {
            return Err(unrepresentable(
                &format!("{info_name} -> clearance class {clearance_class}"),
                "via-info clearance-class reference",
            ));
        }
    }

    let mut via_rule_names = HashSet::new();
    for rule in &board.rules.via_rules {
        if !via_rule_names.insert(rule.name.clone()) {
            return Err(unrepresentable(&rule.name, "duplicate via-rule"));
        }
        if rule.via_count() == 0 {
            return Err(unrepresentable(&rule.name, "empty via-rule"));
        }
        for (position, &via_info_id) in rule.vias().iter().enumerate() {
            if via_info_id >= board.rules.via_infos.count() {
                return Err(unrepresentable(
                    &format!("{}[{position}] -> {via_info_id}", rule.name),
                    "via-rule via-info reference",
                ));
            }
        }
    }

    for class_id in 0..board.rules.net_classes.count() {
        let class = board.rules.net_classes.get(class_id);
        if let Some(rule_id) = class.get_via_rule() {
            if rule_id >= board.rules.via_rules.len() {
                return Err(unrepresentable(
                    &format!("{} -> via_rule {rule_id}", class.get_name()),
                    "net-class via-rule reference",
                ));
            }
        }
    }
    Ok(())
}

/// Reclassifies existing DSN items after a sidecar changes their net class.
/// Only items that still carry their OLD class default are migrated; an
/// explicit per-item DSN clearance override is intentionally preserved.
fn reconcile_existing_item_classes(
    board: &mut BasicBoard,
    previous_rules: &crate::rules::BoardRules,
) {
    let updates: Vec<_> = board
        .items()
        .filter_map(|(id, item)| {
            if item.base.clearance_class_explicit {
                return None;
            }
            let item_class = board_item_class(board, item);
            let via_padstack = match &item.kind {
                crate::board::ItemKind::Via(via) if item.base.component_no == 0 => {
                    Some(via.padstack)
                }
                _ => None,
            };
            let mut nets = item.base.net_nos.iter().copied();
            let Some(first_net) = nets.next() else {
                let old = previous_rules.item_clearance_class_for(0, item_class);
                if item.base.clearance_class != old {
                    return None;
                }
                let new = board.rules.item_clearance_class_for(0, item_class);
                return (new != old).then_some((*id, new));
            };
            let old = item_rule_class(previous_rules, first_net, item_class, via_padstack);
            if item.base.clearance_class != old
                || nets.clone().any(|net| {
                    item_rule_class(previous_rules, net, item_class, via_padstack) != old
                })
            {
                return None;
            }
            let new = item_rule_class(&board.rules, first_net, item_class, via_padstack);
            if nets.any(|net| item_rule_class(&board.rules, net, item_class, via_padstack) != new) {
                return None;
            }
            (new != old).then_some((*id, new))
        })
        .collect();
    for (id, class) in updates {
        board.set_item_clearance_class(id, class);
    }
}

/// Serializes the design rules (Java: `RulesWriter.write`).
pub fn write_rules(board: &BasicBoard, design_name: &str) -> Result<String, RulesWriteError> {
    let scale_out = |v: i32| v as f64 / board.resolution.max(1) as f64;
    let scale_out_f64 = |v: f64| v / board.resolution.max(1) as f64;
    let columns = serialization_columns(board)?;
    validate_via_references(board)?;
    crate::board::validate_board_references(board)
        .map_err(|error| unrepresentable(&error.to_string(), "invalid board reference graph"))?;
    let mut out = String::new();
    out.push_str(&format!("(rules PCB {}\n", dsn_identifier(design_name)?));
    let angle = match board.rules.get_trace_angle_restriction() {
        AngleRestriction::None => "none",
        AngleRestriction::FortyfiveDegree => "fortyfive_degree",
        AngleRestriction::NinetyDegree => "ninety_degree",
    };
    out.push_str(&format!("  (snap_angle {angle})\n"));
    // the default rule: width, untyped clearance, then the typed
    // clearances this board carries (smd_to_turn_gap, same-net drill
    // rules, and every distinct class pair differing from the default —
    // quoted two-token pairs, the form Java also writes)
    let hw = board.rules.get_min_trace_half_width();
    let matrix = &board.rules.clearance_matrix;
    let default_source = if matrix.get_class_count() > 1 { 1 } else { 0 };
    let default_cl = matrix.get_value(default_source, default_source, 0, false);
    out.push_str("  (rule\n");
    out.push_str(&format!(
        "    (width {})\n",
        scale_out_f64(2.0 * f64::from(hw))
    ));
    out.push_str(&format!("    (clearance {})\n", scale_out(default_cl)));
    let turn_gap = board.rules.get_pin_edge_to_turn_dist();
    if turn_gap > 0.0 {
        out.push_str(&format!(
            "    (clearance {} (type smd_to_turn_gap))\n",
            turn_gap / board.resolution.max(1) as f64
        ));
    }
    let mut same_net: Vec<(ItemClass, ItemClass, i32)> = board
        .rules
        .same_net_clearances()
        .filter(|(a, b, _)| a <= b)
        .collect();
    same_net.sort();
    for (a, b, v) in same_net {
        if let (Some(ta), Some(tb)) = (item_class_token(a), item_class_token(b)) {
            out.push_str(&format!(
                "    (clearance {} (type {ta}_{tb}_same_net))\n",
                scale_out(v)
            ));
        }
    }
    // Emit every layer-0 pair, including zero-valued overrides. Besides
    // preserving the exact matrix this forces every named/alias column to be
    // declared even when its row happens to equal the default row.
    for i in 0..columns.len() {
        for j in i..columns.len() {
            write_matrix_pair(
                &mut out,
                "    ",
                &scale_out,
                matrix.get_value(columns[i].source, columns[j].source, 0, false),
                &columns[i].name,
                &columns[j].name,
            )?;
        }
    }
    out.push_str("  )\n");
    // A top-level layer rule resets that layer to its own baseline and then
    // writes only cells that differ from it. The reader applies these scopes
    // in order and therefore reconstructs the complete per-layer matrix.
    for layer in 1..matrix.get_layer_count() {
        let layer_default = matrix.get_value(default_source, default_source, layer, false);
        let differs = columns.iter().enumerate().any(|(i, first)| {
            columns.iter().skip(i).any(|second| {
                matrix.get_value(first.source, second.source, layer, false)
                    != matrix.get_value(first.source, second.source, 0, false)
            })
        });
        if !differs {
            continue;
        }
        let layer_name = board
            .layer_structure
            .arr
            .get(layer)
            .map(|value| value.name.as_str())
            .unwrap_or("signal");
        out.push_str(&format!("  (layer {}\n", dsn_identifier(layer_name)?));
        out.push_str("    (rule\n");
        out.push_str(&format!("      (clearance {})\n", scale_out(layer_default)));
        for i in 0..columns.len() {
            for j in i..columns.len() {
                let value = matrix.get_value(columns[i].source, columns[j].source, layer, false);
                if value != layer_default {
                    write_matrix_pair(
                        &mut out,
                        "      ",
                        &scale_out,
                        value,
                        &columns[i].name,
                        &columns[j].name,
                    )?;
                }
            }
        }
        out.push_str("    )\n");
        out.push_str("  )\n");
    }
    // via padstacks referenced by the via infos, as per-layer rect
    // approximations of their tile shapes (the model keeps resolved
    // shapes, not the pad taxonomy)
    let mut via_padstacks: Vec<usize> = (0..board.rules.via_infos.count())
        .map(|i| board.rules.via_infos.get(i).get_padstack())
        .collect();
    via_padstacks.sort_unstable();
    via_padstacks.dedup();
    for &ps_no in &via_padstacks {
        let Some(ps) = board.padstacks.get_by_no(ps_no) else {
            continue;
        };
        out.push_str(&format!("  (padstack {}\n", dsn_identifier(&ps.name)?));
        for layer in ps.from_layer()..=ps.to_layer() {
            let Some(shape) = ps.get_shape(layer) else {
                continue;
            };
            let bb = shape.bounding_box();
            let layer_name = board
                .layer_structure
                .arr
                .get(layer)
                .map(|l| l.name.as_str())
                .unwrap_or("F.Cu");
            out.push_str(&format!(
                "    (shape (rect {} {} {} {} {}))\n",
                dsn_identifier(layer_name)?,
                scale_out(bb.ll.x),
                scale_out(bb.ll.y),
                scale_out(bb.ur.x),
                scale_out(bb.ur.y),
            ));
        }
        if !ps.attach_allowed {
            out.push_str("    (attach off)\n");
        }
        out.push_str("  )\n");
    }
    // via declarations ((via NAME PADSTACK CLEARANCE_CLASS [attach]))
    for i in 0..board.rules.via_infos.count() {
        let info = board.rules.via_infos.get(i);
        let Some(ps) = board.padstacks.get_by_no(info.get_padstack()) else {
            continue;
        };
        out.push_str(&format!(
            "  (via {} {} {}{})\n",
            dsn_identifier(info.get_name())?,
            dsn_identifier(&ps.name)?,
            dsn_identifier(
                matrix
                    .get_name(info.get_clearance_class())
                    .unwrap_or("default")
            )?,
            if info.attach_smd_allowed() {
                " attach"
            } else {
                ""
            }
        ));
    }
    // via rules ((via_rule NAME VIA...))
    for rule in &board.rules.via_rules {
        out.push_str(&format!("  (via_rule {}", dsn_identifier(&rule.name)?));
        for k in 0..rule.via_count() {
            out.push_str(&format!(
                " {}",
                dsn_identifier(board.rules.via_infos.get(rule.get_via(k)).get_name())?
            ));
        }
        out.push_str(")\n");
    }
    // net classes with their MEMBER NETS (Java Network.write_net_class);
    // a class without members reads back as the default-class descriptor
    for i in 0..board.rules.net_classes.count() {
        let class = board.rules.net_classes.get(i);
        out.push_str(&format!("  (class {}", dsn_identifier(class.get_name())?));
        for n in 1..=board.rules.nets.max_net_no() {
            if let Some(net) = board.rules.nets.get_by_no(n) {
                if net.get_class() == i {
                    out.push_str(&format!(" {}", dsn_identifier(&net.name)?));
                }
            }
        }
        out.push('\n');
        let trace_source = class.get_trace_clearance_class();
        let trace_alias_collision = columns
            .iter()
            .find(|column| column.name.eq_ignore_ascii_case(class.get_name()))
            .is_some_and(|column| {
                !matrix_sources_are_equivalent(matrix, column.source, trace_source)
            });
        if trace_alias_collision {
            if let Some(name) = matrix.get_name(trace_source) {
                out.push_str(&format!(
                    "    (clearance_class {})\n",
                    dsn_identifier(name)?
                ));
            }
        }
        // the via rule by name
        if let Some(rule) = class
            .get_via_rule()
            .and_then(|rule_id| board.rules.via_rules.get(rule_id))
        {
            out.push_str(&format!("    (via_rule {})\n", dsn_identifier(&rule.name)?));
        }
        // The class-local typed rules bind every item kind to its standard
        // class namespace. Their values are copied from the actual bound
        // matrix sources, so arbitrary in-memory bindings survive reload.
        // The widest active-layer width remains the class-wide baseline.
        let hw = (0..class.layer_count())
            .filter(|&l| class.is_active_routing_layer(l))
            .map(|l| class.get_trace_half_width(l))
            .max()
            .unwrap_or(0);
        out.push_str("    (rule\n");
        if hw > 0 {
            out.push_str(&format!(
                "      (width {})\n",
                scale_out_f64(2.0 * f64::from(hw))
            ));
        }
        for (first, &(first_token, first_class)) in SERIALIZED_ITEM_CLASSES.iter().enumerate() {
            for &(second_token, second_class) in SERIALIZED_ITEM_CLASSES.iter().skip(first) {
                if trace_alias_collision
                    && (first_class == ItemClass::Trace || second_class == ItemClass::Trace)
                {
                    continue;
                }
                let value = matrix.get_value(
                    class_item_source(class, first_class),
                    class_item_source(class, second_class),
                    0,
                    false,
                );
                out.push_str(&format!(
                    "      (clearance {} (type {first_token}_{second_token}))\n",
                    scale_out(value)
                ));
            }
        }
        out.push_str("    )\n");
        // Per-layer widths and item-pair values that differ from layer 0.
        let layer_count = board.layer_structure.layer_count();
        for l in 0..layer_count {
            let lw = class.get_trace_half_width(l);
            let pair_differs = (0..SERIALIZED_ITEM_CLASSES.len()).any(|first| {
                (first..SERIALIZED_ITEM_CLASSES.len()).any(|second| {
                    let first_class = SERIALIZED_ITEM_CLASSES[first].1;
                    let second_class = SERIALIZED_ITEM_CLASSES[second].1;
                    if trace_alias_collision
                        && (first_class == ItemClass::Trace || second_class == ItemClass::Trace)
                    {
                        return false;
                    }
                    matrix.get_value(
                        class_item_source(class, first_class),
                        class_item_source(class, second_class),
                        l,
                        false,
                    ) != matrix.get_value(
                        class_item_source(class, first_class),
                        class_item_source(class, second_class),
                        0,
                        false,
                    )
                })
            });
            if (lw > 0 && lw != hw) || pair_differs {
                out.push_str(&format!(
                    "    (layer_rule {}\n",
                    dsn_identifier(&board.layer_structure.arr[l].name)?,
                ));
                out.push_str("      (rule\n");
                if lw > 0 && lw != hw {
                    out.push_str(&format!(
                        "        (width {})\n",
                        scale_out_f64(2.0 * f64::from(lw))
                    ));
                }
                for (first, &(first_token, first_class)) in
                    SERIALIZED_ITEM_CLASSES.iter().enumerate()
                {
                    for &(second_token, second_class) in SERIALIZED_ITEM_CLASSES.iter().skip(first)
                    {
                        if trace_alias_collision
                            && (first_class == ItemClass::Trace || second_class == ItemClass::Trace)
                        {
                            continue;
                        }
                        let value = matrix.get_value(
                            class_item_source(class, first_class),
                            class_item_source(class, second_class),
                            l,
                            false,
                        );
                        let base_value = matrix.get_value(
                            class_item_source(class, first_class),
                            class_item_source(class, second_class),
                            0,
                            false,
                        );
                        if value != base_value {
                            out.push_str(&format!(
                                "        (clearance {} (type {first_token}_{second_token}))\n",
                                scale_out(value)
                            ));
                        }
                    }
                }
                out.push_str("      )\n");
                out.push_str("    )\n");
            }
        }
        // restricted active routing layers ((circuit (use_layer ...))).
        // An EMPTY active mask is still emitted (with no layer names), so
        // it does not silently reload as all-layers-active.
        let active: Vec<&str> = (0..layer_count)
            .filter(|&l| class.is_active_routing_layer(l))
            .map(|l| board.layer_structure.arr[l].name.as_str())
            .collect();
        if active.len() < layer_count {
            out.push_str("    (circuit (use_layer");
            for name in &active {
                out.push_str(&format!(" {}", dsn_identifier(name)?));
            }
            out.push_str("))\n");
        }
        let min_length = class.get_minimum_trace_length();
        let max_length = class.get_maximum_trace_length();
        if min_length > 0.0 || max_length > 0.0 {
            let serialized_max = if max_length > 0.0 {
                max_length / board.resolution.max(1) as f64
            } else {
                -1.0
            };
            let serialized_min = if min_length > 0.0 {
                min_length / board.resolution.max(1) as f64
            } else {
                0.0
            };
            out.push_str(&format!(
                "    (circuit (length {serialized_max} {serialized_min}))\n"
            ));
        }
        if !class.get_pull_tight() {
            out.push_str("    (pull_tight off)\n");
        }
        if class.is_shove_fixed() {
            out.push_str("    (shove_fixed on)\n");
        }
        out.push_str("  )\n");
    }
    out.push_str(")\n");
    Ok(out)
}

/// Applies a `.rules` document to the board (Java: `RulesReader.read`);
/// returns the number of applied settings. Standard grammar: the
/// `(via ...)`, `(via_rule ...)` and `(class ...)` scopes go through the
/// same appliers as the DSN network scope, so class membership,
/// clearance-class and via-rule references, circuit rules and typed
/// clearances all take effect. `(padstack ...)` scopes resolve by name against
/// the design's library or declare a complete new padstack. Malformed known
/// scopes and dangling references reject the transaction; only unknown
/// extension scopes are ignored.
pub fn read_rules(board: &mut BasicBoard, content: &str) -> Result<usize, String> {
    // Sidecar rules are a semantic edit, not a best-effort annotation. Apply
    // them to a private board and commit only after the complete document has
    // parsed and every scaled value fits the board's integer grid.
    let mut candidate = board.clone();
    let applied = read_rules_inner(&mut candidate, content)?;
    crate::board::validation::validate_board_references(&candidate)
        .map_err(|error| format!("rules produced an invalid board: {error}"))?;
    *board = candidate;
    Ok(applied)
}

fn read_rules_inner(board: &mut BasicBoard, content: &str) -> Result<usize, String> {
    let root = parse_dsn(content).map_err(|e| format!("rules parse error: {e:?}"))?;
    validate_rules_numeric_scopes(&root)?;
    let previous_rules = board.rules.clone();
    let resolution = board.resolution.max(1) as f64;
    let scale_error: RefCell<Option<String>> = RefCell::new(None);
    let scale = |v: f64| -> i32 {
        let scaled = v * resolution;
        if !scaled.is_finite() || scaled < f64::from(i32::MIN) || scaled > f64::from(i32::MAX) {
            let mut error = scale_error.borrow_mut();
            if error.is_none() {
                *error = Some(format!(
                    "scaled rules value {v} does not fit the board coordinate range"
                ));
            }
            return 0;
        }
        scaled.round() as i32
    };
    let mut applied = 0usize;
    let mut via_rule_ids: HashMap<String, usize> = board
        .rules
        .via_rules
        .iter()
        .enumerate()
        .map(|(i, r)| (r.name.clone(), i))
        .collect();
    let mut forward_class_shells: HashSet<String> = HashSet::new();
    let nodes = root
        .as_list()
        .ok_or_else(|| "rules document root must be (rules ...)".to_string())?;
    if !nodes
        .first()
        .and_then(|node| node.as_atom())
        .is_some_and(|name| name.eq_ignore_ascii_case("rules"))
    {
        return Err("rules document root must be (rules ...)".to_string());
    }

    // Declaration phase: resolve padstack/via dependencies without applying
    // ordered rule semantics. Via and via-rule placeholders reserve stable
    // IDs only; their clearance class, attach flag and ordered contents are
    // applied at the declaration's actual source position. Class names are
    // likewise created lazily at their own declaration or first forward
    // class-pair reference so inherited defaults reflect that position.
    for node in nodes.iter().skip(1).filter(|n| {
        n.name()
            .is_some_and(|name| name.eq_ignore_ascii_case("padstack"))
    }) {
        let target = node
            .arg()
            .filter(|name| !name.is_empty())
            .ok_or_else(|| "padstack declaration is missing its name".to_string())?;
        let exists = (1..=board.padstacks.count()).any(|no| {
            board
                .padstacks
                .get_by_no(no)
                .is_some_and(|p| p.name.eq_ignore_ascii_case(target))
        });
        if !exists {
            if crate::io::dsn_import::read_padstack_scope(
                &mut board.padstacks,
                &board.layer_structure,
                &scale,
                node,
            )
            .is_none()
            {
                return Err(format!(
                    "padstack {target:?} has no supported, valid layer shape"
                ));
            }
            applied += 1;
        }
    }
    let padstack_nos: HashMap<String, usize> = (1..=board.padstacks.count())
        .filter_map(|no| board.padstacks.get_by_no(no).map(|p| (p.name.clone(), no)))
        .collect();
    let ctx = NetworkScopeCtx {
        padstack_nos: &padstack_nos,
        padstacks: &board.padstacks,
        layer_structure: &board.layer_structure,
    };
    for node in nodes.iter().skip(1).filter(|n| {
        n.name()
            .is_some_and(|name| name.eq_ignore_ascii_case("via"))
    }) {
        let mut args = node.args();
        let (Some(name), Some(padstack_name)) = (args.next(), args.next()) else {
            return Err("via declaration requires a name and padstack".into());
        };
        let Some(padstack_no) = ctx.padstack_no(padstack_name) else {
            return Err(format!(
                "via {name:?} references unknown padstack {padstack_name:?}"
            ));
        };
        if board.rules.via_infos.get_by_name(name).is_none() {
            let _ = board.rules.via_infos.add(crate::rules::ViaInfo::new(
                name,
                padstack_no,
                crate::rules::BoardRules::clearance_class_none(),
                false,
            ));
        }
    }
    for node in nodes.iter().skip(1).filter(|n| {
        n.name()
            .is_some_and(|name| name.eq_ignore_ascii_case("via_rule"))
    }) {
        let mut args = node.args();
        let Some(name) = args.next() else {
            return Err("via_rule declaration is missing its name".into());
        };
        let via_names: Vec<&str> = args.collect();
        if via_names.is_empty() {
            return Err(format!("via_rule {name:?} must contain at least one via"));
        }
        if let Some(missing) = via_names
            .iter()
            .find(|via_name| board.rules.via_infos.get_by_name(via_name).is_none())
        {
            return Err(format!(
                "via_rule {name:?} references unknown via {missing:?}"
            ));
        }
        if via_rule_ids.contains_key(name) {
            continue;
        }
        board.rules.via_rules.push(crate::rules::ViaRule::new(name));
        via_rule_ids.insert(name.to_string(), board.rules.via_rules.len() - 1);
    }
    for node in nodes.iter().skip(1) {
        let Some(name) = node.name() else { continue };
        match name.to_ascii_lowercase().as_str() {
            "snap_angle" => {
                let angle = node
                    .arg()
                    .ok_or_else(|| "snap_angle is missing its value".to_string())?;
                let angle = match angle {
                    "none" => AngleRestriction::None,
                    "ninety_degree" => AngleRestriction::NinetyDegree,
                    "fortyfive_degree" => AngleRestriction::FortyfiveDegree,
                    _ => return Err(format!("unknown snap_angle {angle:?}")),
                };
                board.rules.set_trace_angle_restriction(angle);
                applied += 1;
            }
            "rule" => {
                for w in node.children("width").filter_map(|n| n.arg_f64()) {
                    board
                        .rules
                        .set_default_trace_half_widths((scale(w) / 2).max(1));
                    applied += 1;
                }
                applied += apply_rule_scope_clearances(&mut board.rules, node, &scale, true);
            }
            "layer" => {
                let layer_name = node
                    .arg()
                    .ok_or_else(|| "layer rule is missing its layer name".to_string())?;
                let layer = board
                    .layer_structure
                    .get_no(layer_name)
                    .ok_or_else(|| format!("layer rule references unknown layer {layer_name:?}"))?;
                for rule in node.children("rule") {
                    for w in rule.children("width").filter_map(|n| n.arg_f64()) {
                        board
                            .rules
                            .set_default_trace_half_width_on_layer(layer, (scale(w) / 2).max(1));
                        applied += 1;
                    }
                    applied += apply_rule_scope_clearances_on_layer(
                        &mut board.rules,
                        rule,
                        &scale,
                        true,
                        Some(layer),
                    );
                }
            }
            "padstack" => {
                let target = node
                    .arg()
                    .filter(|name| !name.is_empty())
                    .ok_or_else(|| "padstack declaration is missing its name".to_string())?;
                let exists = (1..=board.padstacks.count()).any(|no| {
                    board
                        .padstacks
                        .get_by_no(no)
                        .is_some_and(|p| p.name.eq_ignore_ascii_case(target))
                });
                if !exists
                    && crate::io::dsn_import::read_padstack_scope(
                        &mut board.padstacks,
                        &board.layer_structure,
                        &scale,
                        node,
                    )
                    .is_none()
                {
                    return Err(format!(
                        "padstack {target:?} has no supported, valid layer shape"
                    ));
                }
            }
            "via" => {
                let padstack_nos: HashMap<String, usize> = (1..=board.padstacks.count())
                    .filter_map(|no| board.padstacks.get_by_no(no).map(|p| (p.name.clone(), no)))
                    .collect();
                let ctx = NetworkScopeCtx {
                    padstack_nos: &padstack_nos,
                    padstacks: &board.padstacks,
                    layer_structure: &board.layer_structure,
                };
                if apply_via_declaration(&mut board.rules, &ctx, node) {
                    applied += 1;
                } else {
                    return Err("malformed or unbound via declaration".into());
                }
            }
            "via_rule" => {
                if apply_via_rule_declaration(&mut board.rules, &mut via_rule_ids, node) {
                    applied += 1;
                } else {
                    return Err("malformed or unbound via_rule declaration".into());
                }
            }
            "class" => {
                let padstack_nos: HashMap<String, usize> = (1..=board.padstacks.count())
                    .filter_map(|no| board.padstacks.get_by_no(no).map(|p| (p.name.clone(), no)))
                    .collect();
                let ctx = NetworkScopeCtx {
                    padstack_nos: &padstack_nos,
                    padstacks: &board.padstacks,
                    layer_structure: &board.layer_structure,
                };
                let mut class_args = node.args();
                let class_name = class_args
                    .next()
                    .ok_or_else(|| "class declaration is missing its name".to_string())?;
                // A sidecar may describe nets that are not present in the
                // current design (for example a reusable project-wide rules
                // file). Membership is therefore retained as a best-effort
                // annotation, while structural references below remain
                // strict because they affect the board graph immediately.
                let _member_nets: Vec<&str> = class_args.filter(|name| !name.is_empty()).collect();
                for circuit in node.children("circuit") {
                    for use_via in circuit.children("use_via") {
                        for padstack in use_via.args() {
                            if ctx.padstack_no(padstack).is_none() {
                                return Err(format!(
                                    "class {class_name:?} references unknown padstack {padstack:?}"
                                ));
                            }
                        }
                    }
                    for use_layer in circuit.children("use_layer") {
                        for layer in use_layer.args() {
                            if board.layer_structure.get_no(layer).is_none() {
                                return Err(format!(
                                    "class {class_name:?} references unknown layer {layer:?}"
                                ));
                            }
                        }
                    }
                }
                for layer_rule in node.children("layer_rule") {
                    for layer in layer_rule.args() {
                        if board.layer_structure.get_no(layer).is_none() {
                            return Err(format!(
                                "class {class_name:?} has a rule for unknown layer {layer:?}"
                            ));
                        }
                    }
                }
                if let Some(via_rule) = node.child("via_rule") {
                    let rule_name = via_rule.arg().ok_or_else(|| {
                        format!("class {class_name:?} has an empty via_rule reference")
                    })?;
                    if !via_rule_ids.contains_key(rule_name) {
                        return Err(format!(
                            "class {class_name:?} references unknown via_rule {rule_name:?}"
                        ));
                    }
                }
                if forward_class_shells.remove(class_name) {
                    if let Some(class_idx) = board.rules.net_classes.get_by_name(class_name) {
                        board.rules.reinitialize_net_class_from_default(class_idx);
                    }
                }
                apply_class_scope(&mut board.rules, &ctx, node, &scale, &via_rule_ids);
                applied += 1;
            }
            "class_class" => {
                // Resolve only the class names needed by this forward pair,
                // at this exact source position. Predeclaring every class at
                // file start freezes the wrong inherited defaults for a class
                // declared after a width/rule change.
                let classes = node
                    .child("classes")
                    .ok_or_else(|| "class_class is missing its classes scope".to_string())?;
                let class_names: Vec<&str> = classes.args().collect();
                if class_names.len() < 2 || class_names.iter().any(|name| name.is_empty()) {
                    return Err("class_class requires at least two named classes".into());
                }
                for class_name in class_names {
                    if board.rules.net_classes.get_by_name(class_name).is_none() {
                        board.rules.append_net_class(class_name);
                        forward_class_shells.insert(class_name.to_string());
                    }
                }
                for layer_rule in node.children("layer_rule") {
                    for layer in layer_rule.args() {
                        if board.layer_structure.get_no(layer).is_none() {
                            return Err(format!(
                                "class_class has a rule for unknown layer {layer:?}"
                            ));
                        }
                    }
                }
                apply_class_class_scope(&mut board.rules, node, &scale);
                applied += 1;
            }
            _ => {}
        }
    }
    reconcile_existing_item_classes(board, &previous_rules);
    if let Some(error) = scale_error.into_inner() {
        return Err(error);
    }
    Ok(applied)
}

/// Validate numeric atoms in scopes understood by the sidecar reader before
/// any applier mutates the candidate board. The low-level shared appliers
/// preserve Java's skip-unknown-scope behavior, but silently skipping a
/// malformed known width or clearance would make a rules file appear to
/// succeed while routing under the old rules.
fn validate_rules_numeric_scopes(root: &crate::io::dsn::SExpr) -> Result<(), String> {
    fn number(
        node: &crate::io::dsn::SExpr,
        label: &str,
        allow_negative: bool,
    ) -> Result<(), String> {
        let args: Vec<&str> = node.args().collect();
        if args.len() != 1 {
            return Err(format!("{label} must contain exactly one numeric value"));
        }
        let value = args[0]
            .parse::<f64>()
            .map_err(|_| format!("{label} is not numeric"))?;
        if !value.is_finite() || (!allow_negative && value < 0.0) {
            return Err(format!("{label} must be finite and nonnegative"));
        }
        Ok(())
    }

    fn walk(node: &crate::io::dsn::SExpr, path: &str) -> Result<(), String> {
        let Some(name) = node.name() else {
            return Ok(());
        };
        match name.to_ascii_lowercase().as_str() {
            "width" | "clearance" | "clear" | "smd_to_turn_gap" => {
                number(node, &format!("{path}/{name}"), false)?;
            }
            "length" => {
                let args: Vec<&str> = node.args().collect();
                if args.len() != 2 {
                    return Err(format!("{path}/length must contain two numeric values"));
                }
                for (index, atom) in args.iter().enumerate() {
                    let value = atom
                        .parse::<f64>()
                        .map_err(|_| format!("{path}/length[{index}] is not numeric"))?;
                    if !value.is_finite() || value < -1.0 {
                        return Err(format!(
                            "{path}/length[{index}] must be finite and at least -1"
                        ));
                    }
                }
            }
            "shape" => {
                // A padstack shape's first atom is a layer name; every
                // remaining atom in the nested shape is numeric geometry.
                if let Some(shape) = node.as_list().and_then(|items| items.get(1)) {
                    for (index, atom) in shape.args().skip(1).enumerate() {
                        let value = atom
                            .parse::<f64>()
                            .map_err(|_| format!("{path}/shape[{index}] is not numeric"))?;
                        if !value.is_finite() {
                            return Err(format!("{path}/shape[{index}] must be finite"));
                        }
                    }
                }
            }
            _ => {}
        }
        if let Some(items) = node.as_list() {
            for (index, child) in items.iter().enumerate().skip(1) {
                walk(child, &format!("{path}/{name}[{index}]"))?;
            }
        }
        Ok(())
    }

    walk(root, "rules")
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geometry::planar::{IntPoint, PolygonShape, PolylineArea};
    use crate::io::import_dsn;

    const MINI: &str = r#"(pcb "mini.dsn"
  (resolution um 10)
  (structure
    (layer F.Cu (type signal))
    (boundary (rect pcb 0 0 50000 50000))
    (rule (width 200) (clearance 200))
  )
  (placement)
  (library)
  (network (net "N1"))
)"#;

    #[test]
    fn class_layer_via_and_shove_settings_round_trip() {
        const TWO_LAYER: &str = r#"(pcb "mini2.dsn"
  (resolution um 10)
  (structure
    (layer F.Cu (type signal))
    (layer B.Cu (type signal))
    (boundary (rect pcb 0 0 50000 50000))
    (rule (width 200) (clearance 200))
  )
  (placement)
  (library
    (padstack "Via[0-1]_600:300_um"
      (shape (circle F.Cu 600 0 0))
      (shape (circle B.Cu 600 0 0))
    )
  )
  (network
    (net "N1")
    (class special "N1" (rule (width 300) (clearance 300)))
  )
)"#;
        let mut board = import_dsn(TWO_LAYER).expect("import");
        let idx = board.rules.net_classes.get_by_name("special").unwrap();
        {
            let class = board.rules.net_classes.get_mut(idx);
            class.set_active_routing_layer(1, false);
            class.set_shove_fixed(true);
            class.set_pull_tight(false);
            class.set_minimum_trace_length(1234.0);
            class.set_maximum_trace_length(9876.0);
            // a layer-dependent width: F.Cu wider than the class rule
            class.set_trace_half_width_on_layer(0, 2500);
        }
        let text = write_rules(&board, "mini2").expect("write");
        assert!(text.contains("use_layer"), "restricted layers persisted");
        assert!(text.contains("shove_fixed on"), "shove_fixed persisted");
        assert!(
            text.contains("(class \"special\" \"N1\""),
            "class net membership persisted: {text}"
        );
        let mut board2 = import_dsn(TWO_LAYER).expect("import");
        read_rules(&mut board2, &text).expect("read");
        let idx2 = board2.rules.net_classes.get_by_name("special").unwrap();
        let class2 = board2.rules.net_classes.get(idx2);
        assert!(class2.is_active_routing_layer(0));
        assert!(
            !class2.is_active_routing_layer(1),
            "layer restriction must survive the .rules round trip"
        );
        assert!(
            class2.is_shove_fixed(),
            "shove_fixed must survive the .rules round trip"
        );
        assert!(
            !class2.get_pull_tight(),
            "pull_tight off must survive the .rules round trip"
        );
        assert_eq!(class2.get_minimum_trace_length(), 1234.0);
        assert_eq!(class2.get_maximum_trace_length(), 9876.0);
        // the per-layer width difference survives instead of collapsing to
        // one class-wide width
        assert_eq!(
            class2.get_trace_half_width(0),
            2500,
            "the wider F.Cu width must survive"
        );
        assert_ne!(
            class2.get_trace_half_width(0),
            class2.get_trace_half_width(1),
            "per-layer widths must not collapse"
        );

        // an EMPTY active mask must round-trip too (it used to reload as
        // all-layers-active because the empty use_layer was omitted)
        let mut board3 = import_dsn(TWO_LAYER).expect("import");
        let idx3 = board3.rules.net_classes.get_by_name("special").unwrap();
        board3
            .rules
            .net_classes
            .get_mut(idx3)
            .set_all_layers_active(false);
        let text3 = write_rules(&board3, "mini2").expect("write");
        let mut board4 = import_dsn(TWO_LAYER).expect("import");
        read_rules(&mut board4, &text3).expect("read");
        let class4 = board4
            .rules
            .net_classes
            .get(board4.rules.net_classes.get_by_name("special").unwrap());
        assert!(
            !class4.is_active_routing_layer(0) && !class4.is_active_routing_layer(1),
            "an empty active mask must survive the round trip"
        );
    }

    #[test]
    fn rules_round_trip() {
        let mut board = import_dsn(MINI).expect("import");
        board
            .rules
            .set_trace_angle_restriction(AngleRestriction::NinetyDegree);
        let text = write_rules(&board, "mini").expect("write");
        assert!(text.contains("snap_angle ninety_degree"));
        let mut board2 = import_dsn(MINI).expect("import");
        assert_ne!(
            board2.rules.get_trace_angle_restriction(),
            AngleRestriction::NinetyDegree
        );
        let applied = read_rules(&mut board2, &text).expect("read");
        assert!(applied >= 2);
        assert_eq!(
            board2.rules.get_trace_angle_restriction(),
            AngleRestriction::NinetyDegree
        );
    }

    #[test]
    fn repeated_widths_are_applied_in_document_order() {
        const TWO_LAYER: &str = r#"(pcb "width-order.dsn"
  (resolution um 10)
  (structure
    (layer F.Cu (type signal))
    (layer B.Cu (type signal))
    (boundary (rect pcb 0 0 50000 50000))
    (rule (width 200) (clearance 200))
  )
  (placement)
  (library)
  (network (net "N1"))
)"#;
        let mut board = import_dsn(TWO_LAYER).expect("import");
        let text = r#"(rules PCB "width-order"
  (rule (width 100) (width 400))
  (layer "F.Cu" (rule (width 300) (width 500)))
  (class "C"
    (rule (width 600) (width 800))
    (layer_rule "F.Cu" (rule (width 700) (width 900))))
)"#;
        read_rules(&mut board, text).expect("read");
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
    fn quoted_identifiers_round_trip_without_corrupting_names() {
        let mut board = import_dsn(MINI).expect("import");
        let class_name = r"power 'low side' \class";
        let net_name = r#"rail "A B" \net"#;
        let design_name = r"board prototype \design";
        let class_idx = board.rules.append_net_class(class_name);
        let net_no = board.rules.nets.add(net_name, 1, false);
        board
            .rules
            .nets
            .get_by_no_mut(net_no)
            .unwrap()
            .set_class(class_idx);

        let text = write_rules(&board, design_name).expect("write representable identifiers");
        let parsed = parse_dsn(&text).expect("writer must emit valid Specctra");
        assert_eq!(parsed.as_list().unwrap()[2].as_atom(), Some(design_name));

        let mut reloaded = import_dsn(MINI).expect("import");
        let reloaded_net = reloaded.rules.nets.add(net_name, 1, false);
        read_rules(&mut reloaded, &text).expect("read own output");
        let reloaded_class = reloaded
            .rules
            .net_classes
            .get_by_name(class_name)
            .expect("class name preserved");
        assert_eq!(
            reloaded
                .rules
                .nets
                .get_by_no(reloaded_net)
                .unwrap()
                .get_class(),
            reloaded_class,
            "quoted net membership must bind to the preserved class name"
        );
        assert_eq!(
            write_rules(&reloaded, design_name).expect("rewrite"),
            text,
            "write-read-write must be byte-stable for either quote and literal backslashes"
        );
    }

    #[test]
    fn unrepresentable_specctra_identifiers_are_rejected() {
        let mut board = import_dsn(MINI).expect("import");
        let error = write_rules(&board, r#"both "double" and 'single'"#)
            .expect_err("Specctra has no quote escape syntax");
        assert!(error.to_string().contains("ordinary identifier"));

        board
            .rules
            .clearance_matrix
            .append_class(r#"power "quoted""#);
        let class_no = board
            .rules
            .clearance_matrix
            .get_no(r#"power "quoted""#)
            .unwrap();
        board
            .rules
            .clearance_matrix
            .set_value(class_no, class_no, 0, 7777);
        let error = write_rules(&board, "mini")
            .expect_err("Java's raw clearance-pair reader cannot preserve a double quote");
        assert!(error.to_string().contains("clearance-class identifier"));
    }

    #[test]
    fn class_class_forward_references_preserve_document_order() {
        let mut board = import_dsn(MINI).expect("import");
        let rules = r#"(rules PCB "mini"
  (class_class (classes "A" "B") (rule (clearance 900)))
  (class "A")
  (class "B")
  (rule (clearance 200))
)"#;
        read_rules(&mut board, rules).expect("read forward class pair");
        let a = board
            .rules
            .clearance_matrix
            .get_no("A")
            .expect("A clearance class");
        let b = board
            .rules
            .clearance_matrix
            .get_no("B")
            .expect("B clearance class");
        assert_eq!(
            board.rules.clearance_matrix.get_value(a, b, 0, false),
            2000,
            "the forward pair must resolve, but a later global rule still wins"
        );
    }

    #[test]
    fn forward_class_shells_inherit_at_their_real_declaration() {
        let mut board = import_dsn(MINI).expect("import");
        let rules = r#"(rules PCB "mini"
  (rule (width 100))
  (class_class (classes "A" "B") (rule (clearance 300)))
  (rule (width 400))
  (class "A")
  (class "B")
)"#;
        read_rules(&mut board, rules).expect("read forward classes");
        let a = board.rules.net_classes.get_by_name("A").unwrap();
        let b = board.rules.net_classes.get_by_name("B").unwrap();
        assert_eq!(
            board.rules.net_classes.get(a).get_trace_half_width(0),
            2000,
            "A must inherit the width active at its class declaration"
        );
        assert_eq!(
            board.rules.net_classes.get(b).get_trace_half_width(0),
            2000,
            "B must inherit the width active at its class declaration"
        );
    }

    #[test]
    fn future_via_symbols_do_not_leak_into_earlier_classes() {
        const VIA_DSN: &str = r#"(pcb "via.dsn"
  (resolution um 10)
  (structure
    (layer F.Cu (type signal))
    (layer B.Cu (type signal))
    (boundary (rect pcb 0 0 50000 50000))
    (rule (width 200) (clearance 200))
  )
  (placement)
  (library
    (padstack "V" (shape (circle F.Cu 600 0 0)) (shape (circle B.Cu 600 0 0)))
  )
  (network (net "N1"))
)"#;
        let mut board = import_dsn(VIA_DSN).expect("import");
        let rules = r#"(rules PCB "via"
  (class "Early")
  (via_rule "FutureRule" "FutureInfo")
  (via "FutureInfo" "V" "via")
  (class "Late")
  (class "Explicit" (via_rule "FutureRule"))
)"#;
        read_rules(&mut board, rules).expect("read via declarations");
        let early = board.rules.net_classes.get_by_name("Early").unwrap();
        let late = board.rules.net_classes.get_by_name("Late").unwrap();
        let explicit = board.rules.net_classes.get_by_name("Explicit").unwrap();
        let via_class = board.rules.clearance_matrix.get_no("via").unwrap();
        assert_ne!(
            board
                .rules
                .net_classes
                .get(early)
                .default_item_clearance_classes
                .get(ItemClass::Via),
            via_class,
            "the future `via` symbol must not alter Early"
        );
        assert_eq!(
            board
                .rules
                .net_classes
                .get(late)
                .default_item_clearance_classes
                .get(ItemClass::Via),
            via_class,
            "Late inherits the semantic declaration now in force"
        );
        assert_eq!(
            board.rules.net_classes.get(explicit).get_via_rule(),
            board
                .rules
                .via_rules
                .iter()
                .position(|r| r.name == "FutureRule"),
            "an explicit forward via_rule reference still resolves"
        );
    }

    #[test]
    fn future_via_clearance_class_is_created_at_its_source_position() {
        const VIA_DSN: &str = r#"(pcb "via-order.dsn"
  (resolution um 10)
  (structure
    (layer F.Cu (type signal))
    (layer B.Cu (type signal))
    (boundary (rect pcb 0 0 50000 50000))
    (rule (width 200) (clearance 200))
  )
  (placement)
  (library
    (padstack "P" (shape (circle F.Cu 600 0 0)) (shape (circle B.Cu 600 0 0)))
  )
  (network (net "N1"))
)"#;
        let mut board = import_dsn(VIA_DSN).expect("import");
        let rules = r#"(rules PCB "via-order"
  (rule (clearance 500 (type "default" "power")))
  (class "C" (via_rule "R"))
  (via "V" "P" "strict")
  (via_rule "R" "V")
)"#;
        read_rules(&mut board, rules).expect("read");
        let power = board
            .rules
            .clearance_matrix
            .get_no("power")
            .expect("power class");
        let strict = board
            .rules
            .clearance_matrix
            .get_no("strict")
            .expect("strict class");
        assert_eq!(
            board
                .rules
                .clearance_matrix
                .get_value(strict, power, 0, false),
            5000,
            "strict must inherit the default-to-power value in force at the via declaration"
        );
        let class = board.rules.net_classes.get_by_name("C").expect("class C");
        assert!(
            board.rules.net_classes.get(class).get_via_rule().is_some(),
            "the symbol-only pass must still resolve forward via-rule IDs"
        );
    }

    #[test]
    fn rules_via_padstack_lookup_is_case_insensitive() {
        const VIA_DSN: &str = r#"(pcb "rules-via-case.dsn"
  (resolution um 10)
  (structure
    (layer F.Cu (type signal))
    (layer B.Cu (type signal))
    (boundary (rect pcb 0 0 50000 50000))
    (rule (width 200) (clearance 200))
  )
  (placement)
  (library
    (padstack "ViaPad" (shape (circle F.Cu 600 0 0)) (shape (circle B.Cu 600 0 0)))
  )
  (network (net "N1"))
)"#;
        let mut board = import_dsn(VIA_DSN).expect("import");
        let rules = r#"(rules PCB "rules-via-case"
  (via "V1" "viapad" default)
  (via_rule "VR" "V1")
  (class "C" "N1" (via_rule "VR"))
)"#;
        read_rules(&mut board, rules).expect("read rules");
        let info = board
            .rules
            .via_infos
            .get_by_name("V1")
            .expect("case-insensitive via padstack reference");
        let padstack = board
            .padstacks
            .get_by_no(board.rules.via_infos.get(info).get_padstack())
            .expect("resolved padstack");
        assert_eq!(padstack.name, "ViaPad");
        let class = board.rules.net_classes.get_by_name("C").expect("class C");
        assert!(board.rules.net_classes.get(class).get_via_rule().is_some());
    }

    #[test]
    fn rules_writer_preserves_layered_item_bindings_and_rejects_asymmetry() {
        const TWO_LAYER: &str = r#"(pcb "matrix.dsn"
  (resolution um 10)
  (structure
    (layer F.Cu (type signal))
    (layer B.Cu (type signal))
    (boundary (rect pcb 0 0 50000 50000))
    (rule (width 200) (clearance 200))
  )
  (placement)
  (library)
  (network (net "N1"))
)"#;
        let mut board = import_dsn(TWO_LAYER).expect("import");
        board.rules.clearance_matrix.append_class("strict");
        let strict = board.rules.clearance_matrix.get_no("strict").unwrap();
        board
            .rules
            .clearance_matrix
            .set_value_on_all_layers(strict, 1, 300);
        board
            .rules
            .clearance_matrix
            .set_value_on_all_layers(1, strict, 300);
        board.rules.clearance_matrix.set_value(strict, 1, 1, 600);
        board.rules.clearance_matrix.set_value(1, strict, 1, 600);
        let class_no = board.rules.append_net_class("strict");
        let class = board.rules.net_classes.get_mut(class_no);
        class.set_trace_clearance_class(strict);
        class.default_item_clearance_classes.set_all(strict);
        board
            .rules
            .nets
            .get_by_no_mut(1)
            .expect("N1")
            .set_class(class_no);
        let text = write_rules(&board, "matrix").expect("write");
        let mut reloaded = import_dsn(TWO_LAYER).expect("reimport");
        read_rules(&mut reloaded, &text).expect("read");
        let strict2 = reloaded
            .rules
            .clearance_matrix
            .get_no("strict")
            .expect("strict class");
        assert_eq!(
            reloaded
                .rules
                .clearance_matrix
                .get_value(strict2, 1, 1, false),
            600
        );
        let class2_no = reloaded
            .rules
            .net_classes
            .get_by_name("strict")
            .expect("strict net class");
        assert_eq!(
            reloaded
                .rules
                .net_classes
                .get(class2_no)
                .get_trace_clearance_class(),
            strict2
        );

        reloaded
            .rules
            .clearance_matrix
            .set_value(strict2, 1, 0, 100);
        let error = write_rules(&reloaded, "matrix")
            .expect_err("asymmetric matrices cannot be represented by .rules");
        assert!(error.to_string().contains("asymmetric"));
    }

    #[test]
    fn rules_writer_rejects_a_trace_alias_that_typed_rules_would_overwrite() {
        let mut board = import_dsn(MINI).expect("import");
        board.rules.clearance_matrix.append_class("strict");
        let strict = board.rules.clearance_matrix.get_no("strict").unwrap();
        board
            .rules
            .clearance_matrix
            .set_value_on_all_layers(strict, 1, 600);
        board
            .rules
            .clearance_matrix
            .set_value_on_all_layers(1, strict, 600);
        let default = board.rules.net_classes.get_mut(0);
        default.set_trace_clearance_class(strict);
        default.default_item_clearance_classes.set_all(strict);

        let error = write_rules(&board, "alias")
            .expect_err("the standard grammar cannot preserve this alias plus typed item rules");
        assert!(error.to_string().contains("trace clearance binding"));
    }

    #[test]
    fn rules_writer_rejects_nonzero_null_class_cells() {
        let mut board = import_dsn(MINI).expect("import");
        board.rules.clearance_matrix.set_value(0, 1, 0, 500);
        board.rules.clearance_matrix.set_value(1, 0, 0, 500);
        let error = write_rules(&board, "mini")
            .expect_err("the rules grammar omits class 0 and must not lose its values");
        assert!(error.to_string().contains("nonzero null"));
    }

    #[test]
    fn rules_writer_rejects_reserved_wire_matrix_class() {
        let mut board = import_dsn(MINI).expect("import");
        board.rules.clearance_matrix.append_class("wire");
        let error = write_rules(&board, "mini")
            .expect_err("the parser reserves wire as an alias of the default class");
        assert!(error.to_string().contains("reserved clearance-class"));
    }

    #[test]
    fn rules_writer_rejects_reserved_net_class_names() {
        let mut board = import_dsn(MINI).expect("import");
        board.rules.append_net_class("wire");
        let error = write_rules(&board, "mini")
            .expect_err("a net class named wire aliases the clearance grammar token");
        assert!(error.to_string().contains("reserved net-class"));
    }

    #[test]
    fn rules_writer_rejects_a_board_without_net_classes_without_panicking() {
        let layers = crate::board::LayerStructure::signal_layers(1);
        let matrix = crate::rules::ClearanceMatrix::get_default_instance(layers.clone(), 200);
        let board = crate::board::BasicBoard::new(
            layers.clone(),
            crate::rules::BoardRules::new(layers, matrix),
            crate::core::Padstacks::new(1),
        );
        let error = write_rules(&board, "empty").expect_err("empty rules are unrepresentable");
        assert!(error.to_string().contains("no net class"));
    }

    #[test]
    fn rules_writer_doubles_large_half_widths_without_overflow() {
        let mut board = import_dsn(MINI).expect("import");
        // CRIT_INT is the largest value accepted by the geometry model.  Its
        // doubled full width still fits in an i32 and must be serialized using
        // floating-point arithmetic rather than an overflowing integer `2 *`.
        board
            .rules
            .set_default_trace_half_widths(crate::geometry::planar::limits::CRIT_INT);
        let text = write_rules(&board, "extreme").expect("wide rule remains serializable");
        assert!(text.contains("(width 6710886.4)"), "{text}");
    }

    #[test]
    fn rules_writer_rejects_dangling_via_info_references() {
        let mut board = import_dsn(MINI).expect("import");
        board.rules.via_infos.add(crate::rules::ViaInfo::new(
            "dangling-padstack",
            usize::MAX,
            1,
            false,
        ));
        let error = write_rules(&board, "mini")
            .expect_err("a via-info with a missing padstack must not be dropped");
        assert!(error.to_string().contains("via-info padstack reference"));

        let mut board = import_dsn(MINI).expect("import");
        let padstack = board.padstacks.add_shape_on_layers(
            crate::geometry::planar::TileShape::Box(crate::geometry::planar::IntBox::from_coords(
                -10, -10, 10, 10,
            )),
            0,
            0,
        );
        board.rules.via_infos.add(crate::rules::ViaInfo::new(
            "dangling-clearance",
            padstack,
            usize::MAX,
            false,
        ));
        let error = write_rules(&board, "mini")
            .expect_err("a via-info with a missing clearance class must not be renamed");
        assert!(error
            .to_string()
            .contains("via-info clearance-class reference"));
    }

    #[test]
    fn rules_writer_rejects_dangling_via_rule_references() {
        let mut board = import_dsn(MINI).expect("import");
        let mut rule = crate::rules::ViaRule::new("dangling-info");
        rule.append_via(usize::MAX);
        board.rules.via_rules.push(rule);
        let error = write_rules(&board, "mini")
            .expect_err("a via rule cannot contain an unknown via-info id");
        assert!(error.to_string().contains("via-rule via-info reference"));

        let mut board = import_dsn(MINI).expect("import");
        board
            .rules
            .net_classes
            .get_mut(0)
            .set_via_rule(Some(usize::MAX));
        let error = write_rules(&board, "mini")
            .expect_err("a class cannot reference an unknown via-rule id");
        assert!(error.to_string().contains("net-class via-rule reference"));
    }

    #[test]
    fn sidecar_reconciles_netless_inherited_items_but_not_explicit_ones() {
        let mut board = import_dsn(MINI).expect("import");
        let area = |x: i32| {
            PolylineArea::new(
                PolygonShape::from_int_points(&[
                    IntPoint::new(x, 1_000),
                    IntPoint::new(x + 500, 1_000),
                    IntPoint::new(x + 500, 1_500),
                    IntPoint::new(x, 1_500),
                ]),
                Vec::new(),
            )
        };
        let inherited = board.insert_area(area(5_000), 0, "inherited", Vec::new(), 1, false);
        let explicit = board.insert_area(area(7_000), 0, "explicit", Vec::new(), 1, false);
        let original_inherited_class = board.get_item(inherited).unwrap().base.clearance_class;
        let original_explicit_class = board.get_item(explicit).unwrap().base.clearance_class;
        board.set_item_clearance_class_explicit(explicit, true);
        let previous = board.rules.clone();
        board.rules.clearance_matrix.append_class("strict_area");
        let strict = board.rules.clearance_matrix.get_no("strict_area").unwrap();
        let default = board.rules.get_default_net_class();
        board
            .rules
            .net_classes
            .get_mut(default)
            .default_item_clearance_classes
            .set(ItemClass::Area, strict);

        reconcile_existing_item_classes(&mut board, &previous);
        assert_eq!(
            board.get_item(explicit).unwrap().base.clearance_class,
            original_explicit_class
        );
        assert_eq!(
            board.get_item(inherited).unwrap().base.clearance_class,
            strict,
            "a netless item inheriting the old area class must follow the sidecar"
        );
        assert_eq!(original_inherited_class, 1);
    }

    #[test]
    fn issue029_rules_classes_padstacks_and_composite_types_apply() {
        // the review repros: 11 classes must stay 11 (empty NAMED classes
        // must not fold into default), the sidecar's 6 padstacks must
        // import, and quoted composite type pairs ("A B"_"C D") must
        // resolve as two names, not fragment at their spaces
        let root = concat!(env!("CARGO_MANIFEST_DIR"), "/..");
        let dsn = std::fs::read_to_string(format!("{root}/fixtures/Issue029-hw48na.dsn"))
            .expect("fixture missing from checkout");
        let rules_text = std::fs::read_to_string(format!("{root}/fixtures/Issue029-hw48na.rules"))
            .expect("fixture missing from checkout");
        let mut board = import_dsn(&dsn).expect("import");
        read_rules(&mut board, &rules_text).expect("read rules");
        // every named class exists (the file declares 11 incl. default)
        for name in [
            "kicad_default",
            "1A EXTERNAL 1oz",
            "2.5A EXTERNAL",
            "3,5A EXT HIGH VOLTAGE",
            "3.5A EXTERNAL 1oz",
            "5A EXTERNAL 1oz",
            "CUSTOM",
            "CUSTOM 0.6",
            "MIN_EXTERN_188A",
            "MIN_EXTERN_241A",
        ] {
            assert!(
                board.rules.net_classes.get_by_name(name).is_some(),
                "class {name:?} must survive (empty classes folded into default before)"
            );
        }
        assert_eq!(
            board.rules.net_classes.get(0).get_name(),
            "default",
            "the default class keeps its identity"
        );
        // the sidecar's padstacks import (the design lacks some of them)
        for ps in [
            "Via[0-1]_1000:400_um",
            "Via[0-1]_1092.2:685.8_um",
            "Via[0-1]_1541.78:1186.18_um",
            "Via[0-1]_600:300_um",
        ] {
            assert!(
                (1..=board.padstacks.count())
                    .any(|no| board.padstacks.get_by_no(no).is_some_and(|p| p.name == ps)),
                "padstack {ps:?} must resolve or import from the sidecar"
            );
        }
        // a quoted composite pair applies at its value: (clear 190.6
        // (type "1A EXTERNAL 1oz"_"2.5A EXTERNAL")) at resolution 10
        let a = board
            .rules
            .clearance_matrix
            .get_no("1A EXTERNAL 1oz")
            .expect("composite class A");
        let b = board
            .rules
            .clearance_matrix
            .get_no("2.5A EXTERNAL")
            .expect("composite class B");
        assert_eq!(
            board.rules.clearance_matrix.get_value(a, b, 0, false),
            1906,
            "the composite pair's clearance must apply un-fragmented"
        );
        // MIN_EXTERN names contain underscores: they must not split
        assert!(
            board
                .rules
                .clearance_matrix
                .get_no("MIN_EXTERN_188A")
                .is_some(),
            "underscored names inside quotes must stay whole"
        );
        assert!(
            board.rules.clearance_matrix.get_no("min").is_none(),
            "no fragment classes may appear"
        );

        // write → re-read must be stable (the malformed output shrank on
        // every pass before)
        let out1 = write_rules(&board, "issue029").expect("write");
        let mut board2 = import_dsn(&dsn).expect("import");
        read_rules(&mut board2, &out1).expect("re-read own output");
        let out2 = write_rules(&board2, "issue029").expect("rewrite");
        assert_eq!(
            out1.lines().count(),
            out2.lines().count(),
            "write→read→write must be stable"
        );
    }

    #[test]
    fn global_clear_alias_applies_to_every_class_pair() {
        // Java's untyped clearance applies to EVERY non-null pair and
        // `clear` is the standard alias — only cell (1,1) was updated
        let mut board = import_dsn(MINI).expect("import");
        // give the board a second real class so the every-pair claim is
        // observable
        board.rules.clearance_matrix.append_class("extra");
        let extra = board.rules.clearance_matrix.get_no("extra").unwrap();
        read_rules(&mut board, "(rules PCB mini (rule (clear 999.0)))").expect("read");
        assert_eq!(board.rules.clearance_matrix.get_value(1, 1, 0, false), 9990);
        assert_eq!(
            board.rules.clearance_matrix.get_value(extra, 1, 0, false),
            9990,
            "the untyped clearance must reach every class pair"
        );
    }

    #[test]
    fn malformed_rules_are_rejected_atomically() {
        let mut board = import_dsn(MINI).expect("import");
        let before_width = board.rules.get_default_trace_half_width(0);
        let before_clearance = board.rules.clearance_matrix.get_value(1, 1, 0, false);
        let before_class_count = board.rules.clearance_matrix.get_class_count();
        let error = read_rules(&mut board, "(rules PCB mini (rule (width 300000000)))")
            .expect_err("a scaled rules value outside the board grid must fail closed");
        assert!(error.contains("scaled rules value"), "{error}");
        assert_eq!(
            board.rules.get_default_trace_half_width(0),
            before_width,
            "a failed sidecar must not leak its width"
        );
        assert_eq!(
            board.rules.clearance_matrix.get_value(1, 1, 0, false),
            before_clearance,
            "a failed sidecar must not leak its matrix values"
        );
        assert_eq!(
            board.rules.clearance_matrix.get_class_count(),
            before_class_count,
            "a failed sidecar must not leak newly-created classes"
        );
    }

    #[test]
    fn rules_reader_rejects_non_rules_root() {
        let mut board = import_dsn(MINI).expect("import");
        let error = read_rules(&mut board, "(pcb mini)")
            .expect_err("a sidecar with a DSN root must be rejected");
        assert!(error.contains("root must be (rules ...)"), "{error}");
    }

    #[test]
    fn standard_freerouting_rules_file_applies_fully() {
        // Issue593's rules file (standard Java RulesWriter output): the
        // via declarations, via rules, class membership and typed
        // clearances must all take effect, not just the global defaults
        let root = concat!(env!("CARGO_MANIFEST_DIR"), "/..");
        let dsn = std::fs::read_to_string(format!("{root}/fixtures/Issue593-BBD_Mars-64.dsn"))
            .expect("fixture missing from checkout");
        let rules_text =
            std::fs::read_to_string(format!("{root}/fixtures/Issue593-BBD_Mars-64.rules"))
                .expect("fixture missing from checkout");
        let mut board = import_dsn(&dsn).expect("import");
        let applied = read_rules(&mut board, &rules_text).expect("read rules");
        assert!(
            applied >= 10,
            "the standard rules file must restore more than the defaults (applied {applied})"
        );
        // the typed clearances from the (rule ...) scope (single-token
        // names without '_' are skipped, exactly like Java)
        assert_eq!(
            board.rules.get_pin_edge_to_turn_dist(),
            1250.0,
            "smd_to_turn_gap 125.0 at resolution 10"
        );
        // the via declarations and rules
        assert!(
            board
                .rules
                .via_infos
                .get_by_name("Via[0-1]_800:400_um")
                .is_some(),
            "via declaration must be applied"
        );
        assert!(
            board
                .rules
                .via_infos
                .get_by_name("Via[0-1]_800:400_um-kicad_default")
                .is_some(),
            "second via declaration must be applied"
        );
        // the kicad_default class: membership + via rule
        let kd = board
            .rules
            .net_classes
            .get_by_name("kicad_default")
            .expect("kicad_default class");
        let gnd = &board.rules.nets.get_by_name("GND");
        assert!(!gnd.is_empty());
        assert_eq!(
            gnd[0].get_class(),
            kd,
            "GND must join kicad_default per the class membership"
        );
        assert!(
            board.rules.net_classes.get(kd).get_via_rule().is_some(),
            "the class's via rule reference must bind"
        );
    }
}
