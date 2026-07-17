//! Port of `RulesWriter.java` / `RulesReader.java`: persisting the board
//! design rules in the standard Specctra `(rules PCB ...)` format — snap
//! angle, the default width/clearance rule with its typed clearances,
//! via padstacks, `(via ...)`/`(via_rule ...)` declarations, and the net
//! classes with membership, clearance-class/via-rule references and
//! circuit rules. The reader applies the same grammar through the shared
//! network-scope appliers used by the DSN importer, so a standard
//! Freerouting `.rules` file (e.g. Issue593's) restores its classes, via
//! rules and typed clearances instead of only the global defaults.

use std::collections::HashMap;

use crate::board::basic_board::BasicBoard;
use crate::board::AngleRestriction;
use crate::io::dsn::parse_dsn;
use crate::io::dsn_import::{
    apply_class_class_scope, apply_class_scope, apply_rule_scope_clearances, apply_via_declaration,
    apply_via_rule_declaration, NetworkScopeCtx,
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

/// Serializes the design rules (Java: `RulesWriter.write`).
pub fn write_rules(board: &BasicBoard, design_name: &str) -> String {
    let scale_out = |v: i32| v as f64 / board.resolution.max(1) as f64;
    let mut out = String::new();
    out.push_str(&format!("(rules PCB {design_name}\n"));
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
    let default_cl = matrix.get_value(1, 1, 0, false);
    out.push_str("  (rule\n");
    out.push_str(&format!("    (width {})\n", scale_out(2 * hw)));
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
    for i in 1..matrix.get_class_count() {
        for j in i..matrix.get_class_count() {
            let v = matrix.get_value(i, j, 0, false);
            if v != default_cl && v > 0 {
                out.push_str(&format!(
                    "    (clearance {} (type \"{}\" \"{}\"))\n",
                    scale_out(v),
                    matrix.get_name(i).unwrap_or("default"),
                    matrix.get_name(j).unwrap_or("default"),
                ));
            }
        }
    }
    out.push_str("  )\n");
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
        out.push_str(&format!("  (padstack \"{}\"\n", ps.name));
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
                "    (shape (rect {layer_name} {} {} {} {}))\n",
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
            "  (via \"{}\" \"{}\" \"{}\"{})\n",
            info.get_name(),
            ps.name,
            matrix
                .get_name(info.get_clearance_class())
                .unwrap_or("default"),
            if info.attach_smd_allowed() {
                " attach"
            } else {
                ""
            }
        ));
    }
    // via rules ((via_rule NAME VIA...))
    for rule in &board.rules.via_rules {
        out.push_str(&format!("  (via_rule \"{}\"", rule.name));
        for k in 0..rule.via_count() {
            out.push_str(&format!(
                " \"{}\"",
                board.rules.via_infos.get(rule.get_via(k)).get_name()
            ));
        }
        out.push_str(")\n");
    }
    // net classes with their MEMBER NETS (Java Network.write_net_class);
    // a class without members reads back as the default-class descriptor
    for i in 0..board.rules.net_classes.count() {
        let class = board.rules.net_classes.get(i);
        out.push_str(&format!("  (class \"{}\"", class.get_name()));
        for n in 1..=board.rules.nets.max_net_no() {
            if let Some(net) = board.rules.nets.get_by_no(n) {
                if net.get_class() == i {
                    out.push_str(&format!(" \"{}\"", net.name));
                }
            }
        }
        out.push('\n');
        // the trace clearance class by NAME (standard reference form)
        if let Some(name) = matrix.get_name(class.get_trace_clearance_class()) {
            out.push_str(&format!("    (clearance_class \"{name}\")\n"));
        }
        // the via rule by name
        if let Some(rule) = class
            .get_via_rule()
            .and_then(|rule_id| board.rules.via_rules.get(rule_id))
        {
            out.push_str(&format!("    (via_rule \"{}\")\n", rule.name));
        }
        // the widest active-layer width (the single-width router's rule)
        let hw = (0..class.layer_count())
            .filter(|&l| class.is_active_routing_layer(l))
            .map(|l| class.get_trace_half_width(l))
            .max()
            .unwrap_or(0);
        if hw > 0 {
            out.push_str(&format!("    (rule (width {}))\n", scale_out(2 * hw)));
        }
        // per-layer widths: a layer whose width differs from the class
        // maximum gets its own (layer_rule L (rule (width ...)))
        let layer_count = board.layer_structure.layer_count();
        for l in 0..layer_count {
            let lw = class.get_trace_half_width(l);
            if lw > 0 && lw != hw {
                out.push_str(&format!(
                    "    (layer_rule \"{}\" (rule (width {})))\n",
                    board.layer_structure.arr[l].name,
                    scale_out(2 * lw)
                ));
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
                out.push_str(&format!(" \"{name}\""));
            }
            out.push_str("))\n");
        }
        if class.is_shove_fixed() {
            out.push_str("    (shove_fixed on)\n");
        }
        out.push_str("  )\n");
    }
    out.push_str(")\n");
    out
}

/// Applies a `.rules` document to the board (Java: `RulesReader.read`);
/// returns the number of applied settings. Standard grammar: the
/// `(via ...)`, `(via_rule ...)` and `(class ...)` scopes go through the
/// same appliers as the DSN network scope, so class membership,
/// clearance-class and via-rule references, circuit rules and typed
/// clearances all take effect. `(padstack ...)` scopes resolve by NAME
/// against the design's library (shapes are not rebuilt); a padstack
/// unknown to the design is skipped.
pub fn read_rules(board: &mut BasicBoard, content: &str) -> Result<usize, String> {
    let root = parse_dsn(content).map_err(|e| format!("rules parse error: {e:?}"))?;
    let mut applied = 0usize;
    if let Some(angle) = root.child("snap_angle").and_then(|n| n.arg()) {
        board.rules.set_trace_angle_restriction(match angle {
            "ninety_degree" => AngleRestriction::NinetyDegree,
            "fortyfive_degree" => AngleRestriction::FortyfiveDegree,
            _ => AngleRestriction::None,
        });
        applied += 1;
    }
    let resolution = board.resolution.max(1) as f64;
    let scale = move |v: f64| -> i32 { (v * resolution).round() as i32 };
    // EVERY top-level (rule ...) scope, in document order — the untyped
    // clearance (clearance/clear alias alike) sets the whole-matrix
    // default, typed rules refine it, and later rules overwrite earlier
    // ones exactly like Java's sequential reader
    for rule in root.children("rule") {
        if let Some(w) = rule
            .child("width")
            .and_then(|n| n.arg())
            .and_then(|v| v.parse::<f64>().ok())
        {
            board.rules.set_default_trace_half_widths(scale(w) / 2);
            applied += 1;
        }
        applied += apply_rule_scope_clearances(&mut board.rules, rule, &scale, true);
    }
    // top-level (padstack ...) scopes: names already in the design
    // resolve to it; a padstack the design LACKS is imported from the
    // sidecar (skipping it silently dropped every dependent via
    // declaration)
    {
        let BasicBoard {
            ref mut padstacks,
            ref layer_structure,
            ..
        } = *board;
        for ps_node in root.children("padstack") {
            let Some(name) = ps_node.arg() else {
                continue;
            };
            let exists = (1..=padstacks.count())
                .any(|no| padstacks.get_by_no(no).is_some_and(|p| p.name == name));
            if exists {
                continue;
            }
            if crate::io::dsn_import::read_padstack_scope(
                padstacks,
                layer_structure,
                &scale,
                ps_node,
            )
            .is_some()
            {
                applied += 1;
            }
        }
    }
    // split the board borrows: the appliers mutate rules while reading
    // the padstack library and layer structure
    let padstack_nos: HashMap<String, usize> = (1..=board.padstacks.count())
        .filter_map(|no| board.padstacks.get_by_no(no).map(|p| (p.name.clone(), no)))
        .collect();
    let mut via_rule_ids: HashMap<String, usize> = board
        .rules
        .via_rules
        .iter()
        .enumerate()
        .map(|(i, r)| (r.name.clone(), i))
        .collect();
    let default_clearance = board.rules.clearance_matrix.get_value(1, 1, 0, false);
    let BasicBoard {
        ref mut rules,
        ref padstacks,
        ref layer_structure,
        ..
    } = *board;
    let ctx = NetworkScopeCtx {
        padstack_nos: &padstack_nos,
        padstacks,
        layer_structure,
        default_clearance,
    };
    for via_node in root.children("via") {
        // only count declarations that actually bind (unknown padstacks
        // must not inflate the applied count)
        if apply_via_declaration(rules, &ctx, via_node) {
            applied += 1;
        }
    }
    for rule_node in root.children("via_rule") {
        apply_via_rule_declaration(rules, &mut via_rule_ids, rule_node);
        applied += 1;
    }
    for class_node in root.children("class") {
        let Some(class_name) = class_node.arg() else {
            continue;
        };
        apply_class_scope(rules, &ctx, class_node, &scale, &via_rule_ids);
        applied += 1;
        let Some(class_idx) = rules.net_classes.get_by_name(class_name).or_else(|| {
            class_node
                .args()
                .nth(1)
                .is_none()
                .then(|| rules.get_default_net_class())
        }) else {
            continue;
        };
        // per-layer width overrides ((layer_rule L (rule (width W))))
        for lr in class_node.children("layer_rule") {
            let Some(layer) = lr.arg().and_then(|n| layer_structure.get_no(n)) else {
                continue;
            };
            if let Some(w) = lr
                .child("rule")
                .and_then(|r| r.child("width"))
                .and_then(|n| n.arg())
                .and_then(|v| v.parse::<f64>().ok())
            {
                rules
                    .net_classes
                    .get_mut(class_idx)
                    .set_trace_half_width_on_layer(layer, (scale(w) / 2).max(1));
                applied += 1;
            }
        }
        // an EMPTY (use_layer) list means NO layer is active; the shared
        // class applier only restricts on non-empty lists
        let has_use_layer = class_node
            .children("circuit")
            .any(|c| c.child("use_layer").is_some());
        let resolved: Vec<usize> = class_node
            .children("circuit")
            .flat_map(|c| c.children("use_layer"))
            .flat_map(|u| u.args())
            .filter_map(|n| layer_structure.get_no(n))
            .collect();
        if has_use_layer && resolved.is_empty() {
            rules
                .net_classes
                .get_mut(class_idx)
                .set_all_layers_active(false);
            applied += 1;
        }
        // legacy private form from earlier rounds of this port:
        // (use_via "PADSTACK") directly in the class scope
        if let Some(padstack_no) = class_node
            .child("use_via")
            .and_then(|u| u.arg())
            .and_then(|name| padstack_nos.get(name).copied())
        {
            let existing = (0..rules.via_infos.count())
                .find(|&i| rules.via_infos.get(i).get_padstack() == padstack_no);
            let tcc = rules.net_classes.get(class_idx).get_trace_clearance_class();
            let attach = rules.via_at_smd_allowed
                && padstacks
                    .get_by_no(padstack_no)
                    .is_some_and(|p| p.attach_allowed);
            let via_info_id = existing.or_else(|| {
                rules.via_infos.add(crate::rules::ViaInfo::new(
                    format!("via::{class_name}"),
                    padstack_no,
                    tcc,
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
                applied += 1;
            }
        }
    }
    // (class_class (classes A B) (rule (clearance C))): pairwise
    // clearances between net classes (Java Network.insert_class_pairs) —
    // applied AFTER the classes so both sides resolve
    for cc_node in root.children("class_class") {
        apply_class_class_scope(rules, cc_node, &scale);
        applied += 1;
    }
    Ok(applied)
}

#[cfg(test)]
mod tests {
    use super::*;
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
            // a layer-dependent width: F.Cu wider than the class rule
            class.set_trace_half_width_on_layer(0, 2500);
        }
        let text = write_rules(&board, "mini2");
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
        let text3 = write_rules(&board3, "mini2");
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
        let text = write_rules(&board, "mini");
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
        let out1 = write_rules(&board, "issue029");
        let mut board2 = import_dsn(&dsn).expect("import");
        read_rules(&mut board2, &out1).expect("re-read own output");
        let out2 = write_rules(&board2, "issue029");
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
