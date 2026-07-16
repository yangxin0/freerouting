//! Port of `RulesWriter.java` / `RulesReader.java`: persisting the board
//! design rules in the Specctra `(rules PCB ...)` format — snap angle,
//! the default width/clearance rule, and the net classes with their
//! rules.

use crate::board::basic_board::BasicBoard;
use crate::board::AngleRestriction;
use crate::io::dsn::parse_dsn;

/// Serializes the design rules (Java: `RulesWriter.write`).
pub fn write_rules(board: &BasicBoard, design_name: &str) -> String {
    let mut out = String::new();
    out.push_str(&format!("(rules PCB {design_name}\n"));
    let angle = match board.rules.get_trace_angle_restriction() {
        AngleRestriction::None => "none",
        AngleRestriction::FortyfiveDegree => "fortyfive_degree",
        AngleRestriction::NinetyDegree => "ninety_degree",
    };
    out.push_str(&format!("  (snap_angle {angle})\n"));
    let hw = board.rules.get_min_trace_half_width();
    let cl = board.rules.clearance_matrix.get_value(1, 1, 0, false);
    out.push_str(&format!(
        "  (rule\n    (width {})\n    (clearance {})\n  )\n",
        (2 * hw) as f64 / board.resolution.max(1) as f64,
        cl as f64 / board.resolution.max(1) as f64,
    ));
    let scale_out = |v: i32| v as f64 / board.resolution.max(1) as f64;
    for i in 0..board.rules.net_classes.count() {
        let class = board.rules.net_classes.get(i);
        out.push_str(&format!("  (class \"{}\"\n", class.get_name()));
        // the widest active-layer width (the single-width router's rule)
        let hw = (0..class.layer_count())
            .filter(|&l| class.is_active_routing_layer(l))
            .map(|l| class.get_trace_half_width(l))
            .max()
            .unwrap_or(0);
        // emit width AND clearance so a custom class's spacing is not discarded
        let tcc = class.get_trace_clearance_class();
        let class_cl = board.rules.clearance_matrix.get_value(tcc, tcc, 0, false);
        out.push_str("    (rule");
        if hw > 0 {
            out.push_str(&format!(" (width {})", scale_out(2 * hw)));
        }
        if class_cl > 0 {
            out.push_str(&format!(" (clearance {})", scale_out(class_cl)));
        }
        out.push_str(")\n");
        // restricted active routing layers ((circuit (use_layer ...)))
        let layer_count = board.layer_structure.layer_count();
        let active: Vec<&str> = (0..layer_count)
            .filter(|&l| class.is_active_routing_layer(l))
            .map(|l| board.layer_structure.arr[l].name.as_str())
            .collect();
        if active.len() < layer_count && !active.is_empty() {
            out.push_str("    (circuit (use_layer");
            for name in &active {
                out.push_str(&format!(" \"{name}\""));
            }
            out.push_str("))\n");
        }
        if class.is_shove_fixed() {
            out.push_str("    (shove_fixed on)\n");
        }
        // the class's via rule, persisted by its first via's padstack name
        if let Some(padstack_name) = class
            .get_via_rule()
            .and_then(|rule_id| board.rules.via_rules.get(rule_id))
            .filter(|rule| rule.via_count() > 0)
            .map(|rule| board.rules.via_infos.get(rule.get_via(0)).get_padstack())
            .and_then(|ps| board.padstacks.get_by_no(ps))
            .map(|ps| ps.name.clone())
        {
            out.push_str(&format!("    (use_via \"{padstack_name}\")\n"));
        }
        out.push_str("  )\n");
    }
    out.push_str(")\n");
    out
}

/// Applies a `.rules` document to the board (Java: `RulesReader.read`);
/// returns the number of applied settings.
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
    let scale = |v: f64| -> i32 { (v * board.resolution.max(1) as f64).round() as i32 };
    if let Some(rule) = root.child("rule") {
        if let Some(w) = rule
            .child("width")
            .and_then(|n| n.arg())
            .and_then(|v| v.parse::<f64>().ok())
        {
            board.rules.set_default_trace_half_widths(scale(w) / 2);
            applied += 1;
        }
        if let Some(c) = rule
            .child("clearance")
            .and_then(|n| n.arg())
            .and_then(|v| v.parse::<f64>().ok())
        {
            let layers = board.rules.clearance_matrix.get_layer_count();
            for layer in 0..layers {
                board
                    .rules
                    .clearance_matrix
                    .set_value(1, 1, layer, scale(c));
            }
            applied += 1;
        }
    }
    // per-class (class NAME (rule (width W) (clearance C))) blocks: apply the
    // width and clearance to the named net class instead of ignoring them.
    for class_node in root.children("class") {
        let Some(class_name) = class_node.arg() else {
            continue;
        };
        let Some(class_idx) = board.rules.net_classes.get_by_name(class_name) else {
            continue;
        };
        let Some(rule) = class_node.child("rule") else {
            continue;
        };
        if let Some(w) = rule
            .child("width")
            .and_then(|n| n.arg())
            .and_then(|v| v.parse::<f64>().ok())
        {
            board
                .rules
                .net_classes
                .get_mut(class_idx)
                .set_trace_half_width((scale(w) / 2).max(1));
            applied += 1;
        }
        // the default class's clearance is carried by the global rule above;
        // apply per-class clearance only to the non-default classes so the
        // shared default clearance class is not perturbed.
        if class_idx != 0 {
            if let Some(c) = rule
                .child("clearance")
                .and_then(|n| n.arg())
                .and_then(|v| v.parse::<f64>().ok())
            {
                board.rules.ensure_net_class_clearance(class_idx, scale(c));
                applied += 1;
            }
        }
        // restricted active routing layers, like the DSN class scope
        let use_layers: Vec<usize> = class_node
            .children("circuit")
            .flat_map(|c| c.children("use_layer"))
            .flat_map(|u| u.args())
            .filter_map(|n| board.layer_structure.get_no(n))
            .collect();
        if !use_layers.is_empty() {
            let class = board.rules.net_classes.get_mut(class_idx);
            class.set_all_layers_active(false);
            for l in use_layers {
                class.set_active_routing_layer(l, true);
            }
            applied += 1;
        }
        if let Some(v) = class_node.child("shove_fixed").and_then(|s| s.arg()) {
            board
                .rules
                .net_classes
                .get_mut(class_idx)
                .set_shove_fixed(v.eq_ignore_ascii_case("on"));
            applied += 1;
        }
        // (use_via "PADSTACK"): bind the class to a via rule over the named
        // padstack, reusing a declared via info when one exists
        if let Some(padstack_no) =
            class_node
                .child("use_via")
                .and_then(|u| u.arg())
                .and_then(|name| {
                    (1..=board.padstacks.count()).find(|&no| {
                        board
                            .padstacks
                            .get_by_no(no)
                            .is_some_and(|p| p.name == name)
                    })
                })
        {
            let existing = (0..board.rules.via_infos.count())
                .find(|&i| board.rules.via_infos.get(i).get_padstack() == padstack_no);
            let tcc = board
                .rules
                .net_classes
                .get(class_idx)
                .get_trace_clearance_class();
            let attach = board.rules.via_at_smd_allowed
                && board
                    .padstacks
                    .get_by_no(padstack_no)
                    .is_some_and(|p| p.attach_allowed);
            let via_info_id = existing.or_else(|| {
                board.rules.via_infos.add(crate::rules::ViaInfo::new(
                    format!("via::{class_name}"),
                    padstack_no,
                    tcc,
                    attach,
                ))
            });
            if let Some(via_info_id) = via_info_id {
                let mut via_rule = crate::rules::ViaRule::new(class_name);
                via_rule.append_via(via_info_id);
                board.rules.via_rules.push(via_rule);
                let rule_id = board.rules.via_rules.len() - 1;
                board
                    .rules
                    .net_classes
                    .get_mut(class_idx)
                    .set_via_rule(Some(rule_id));
                applied += 1;
            }
        }
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
        }
        let text = write_rules(&board, "mini2");
        assert!(text.contains("use_layer"), "restricted layers persisted");
        assert!(text.contains("shove_fixed on"), "shove_fixed persisted");
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
}
