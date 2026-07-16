//! Routes a board from scratch, then audits the result: every pair of
//! items of different nets on a shared layer must keep the pairwise
//! clearance of the rule matrix. Prints the violations (if any).
//!
//! Usage: cargo run --release --example drc_check -- board.dsn [seconds]

use freerouting::autoroute::{
    batch_route_passes_with_time_limit, combine_all_traces, pull_tight_all, BatchRequest,
};
use freerouting::board::ItemKind;
use freerouting::datastructures::TimeLimit;
use freerouting::io::import_dsn;

fn main() {
    let path = std::env::args()
        .nth(1)
        .expect("usage: drc_check <dsn> [seconds]");
    let limit_s: u64 = std::env::args()
        .nth(2)
        .and_then(|v| v.parse().ok())
        .unwrap_or(120);
    let content = std::fs::read_to_string(&path).expect("read failed");
    let content = match content.find("  (wiring") {
        Some(pos) => format!("{})", &content[..pos]),
        None => content,
    };
    let mut board = import_dsn(&content).expect("import failed");
    board
        .rules
        .set_trace_angle_restriction(match std::env::var("FR_ANGLE").as_deref() {
            Ok("45") => freerouting::board::AngleRestriction::FortyfiveDegree,
            Ok("90") => freerouting::board::AngleRestriction::NinetyDegree,
            _ => freerouting::board::AngleRestriction::None,
        });

    let all_layers = board.layer_structure.layer_count().saturating_sub(1);
    let via_padstack = (1..=board.padstacks.count())
        .find(|no| {
            board
                .padstacks
                .get_by_no(*no)
                .is_some_and(|p| p.name.starts_with("Via"))
        })
        .or_else(|| {
            (1..=board.padstacks.count()).find(|no| {
                board
                    .padstacks
                    .get_by_no(*no)
                    .is_some_and(|p| p.from_layer() == 0 && p.to_layer() == all_layers)
            })
        })
        .unwrap_or(0);
    let request = BatchRequest {
        trace_half_width: board.rules.get_min_trace_half_width().max(500),
        clearance_class: 1,
        via_padstack,
        via_cost: 50_000.0,
        max_expansions: 100_000,
        ripup_penalty: 0.0,
        deadline: None,
    };
    let limit = TimeLimit::new(limit_s * 1000);
    batch_route_passes_with_time_limit(&mut board, &request, 99, Some(&limit));
    if std::env::var_os("FR_NO_POST").is_none() {
        combine_all_traces(&mut board);
        if std::env::var_os("FR_NO_TIGHT").is_none() {
            pull_tight_all(&mut board, 3);
        }
    }

    // audit: routed items (component 0) vs everything foreign
    let mut violations = 0usize;
    let mut hard_violations = 0usize;
    let mut checked = 0usize;
    let routed: Vec<_> = board
        .items()
        .filter(|(_, it)| {
            it.base.component_no == 0
                && it.base.net_count() > 0
                && !matches!(&it.kind, ItemKind::ObstacleArea(_))
        })
        .map(|(id, _)| *id)
        .collect();
    for &id in &routed {
        let Some(item) = board.get_item(id) else {
            continue;
        };
        let shapes: Vec<_> = item.tile_shapes(&board.padstacks).iter().cloned().collect();
        for (shape, layer) in shapes {
            for other_id in board.overlapping_items(&shape.offset(10_000.0), Some(layer)) {
                if other_id == id {
                    continue;
                }
                let Some(other) = board.get_item(other_id) else {
                    continue;
                };
                if other.base.shares_net(&item.base) {
                    continue;
                }
                if let ItemKind::ObstacleArea(a) = &other.kind {
                    if a.is_conduction {
                        continue; // planes get fabrication cutouts
                    }
                }
                let clearance = board.rules.clearance_matrix.get_value(
                    item.base.clearance_class,
                    other.base.clearance_class,
                    layer,
                    false,
                ) as f64;
                let check = shape.offset(clearance);
                checked += 1;
                // mitered pre-filter (no false negatives), confirmed with
                // the exact Euclidean copper distance (the line-push
                // over-reaches diagonally at corners)
                let conflict = other.tile_shapes(&board.padstacks).iter().any(|(s, l)| {
                    *l == layer
                        && s.intersection(&check).dimension() >= 2
                        && shape.euclidean_distance_to(s) < clearance - 1.0
                });
                // classify: deeper than 2 units = a real violation; the
                // rest is corner-rounding epsilon on exact-touch paths
                let hard = conflict
                    && other.tile_shapes(&board.padstacks).iter().any(|(s, l)| {
                        *l == layer && shape.euclidean_distance_to(s) < clearance - 2.0
                    });
                if hard {
                    hard_violations += 1;
                }
                if conflict {
                    violations += 1;
                    // bisect the violation depth: largest d with overlap
                    // at offset(clearance - d)
                    let (mut lo, mut hi) = (0.0f64, clearance);
                    while hi - lo > 1.0 {
                        let mid = (lo + hi) / 2.0;
                        let deep = other.tile_shapes(&board.padstacks).iter().any(|(s, l)| {
                            *l == layer
                                && s.intersection(&shape.offset(clearance - mid)).dimension() >= 2
                        });
                        if deep {
                            lo = mid;
                        } else {
                            hi = mid;
                        }
                    }
                    println!("DEPTH {lo:.0} of {clearance}");
                    if violations <= 10 {
                        let kind = |it: &freerouting::board::Item| match &it.kind {
                            ItemKind::Via(_) => "via",
                            ItemKind::PolylineTrace(_) => "trace",
                            ItemKind::ObstacleArea(_) => "area",
                        };
                        println!(
                            "VIOLATION: {} {id} (nets {:?}, birth {}) vs {} {other_id} \
                             (nets {:?}, birth {}) layer {layer} req {clearance}",
                            kind(item),
                            item.base.net_nos,
                            item.base.birth,
                            kind(other),
                            other.base.net_nos,
                            other.base.birth,
                        );
                        eprintln!("  at {:?}", shape.bounding_box());
                    }
                }
            }
        }
    }
    let complete = (1..=board.rules.nets.max_net_no())
        .filter(|&n| board.net_is_completely_connected(n))
        .count();
    println!(
        "routed {complete}/{} nets; DRC: {checked} pair checks, {violations} violations \
         ({hard_violations} deeper than 2 units)",
        board.rules.nets.max_net_no()
    );
    std::process::exit(if violations == 0 { 0 } else { 1 });
}
