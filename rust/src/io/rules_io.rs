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
    let cl = board
        .rules
        .clearance_matrix
        .get_value(1, 1, 0, false);
    out.push_str(&format!(
        "  (rule\n    (width {})\n    (clearance {})\n  )\n",
        (2 * hw) as f64 / board.resolution.max(1) as f64,
        cl as f64 / board.resolution.max(1) as f64,
    ));
    for i in 0..board.rules.net_classes.count() {
        let class = board.rules.net_classes.get(i);
        out.push_str(&format!("  (class \"{}\"\n", class.get_name()));
        let hw = class.get_trace_half_width(0);
        if hw > 0 {
            out.push_str(&format!(
                "    (rule (width {}))\n",
                (2 * hw) as f64 / board.resolution.max(1) as f64
            ));
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
                board.rules.clearance_matrix.set_value(1, 1, layer, scale(c));
            }
            applied += 1;
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
