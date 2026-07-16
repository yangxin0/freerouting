//! Diagnostic: route the fixture briefly, then dump per-trace length and
//! corner statistics to find where the oversized total length comes from.

use freerouting::autoroute::{batch_route_passes_with_time_limit, BatchRequest};
use freerouting::board::ItemKind;
use freerouting::datastructures::TimeLimit;
use freerouting::io::import_dsn;

fn main() {
    let path = std::env::args().nth(1).expect("usage: trace_stats <dsn>");
    let content = std::fs::read_to_string(&path).expect("read failed");
    // strip pre-routed wiring
    let content = match content.find("  (wiring") {
        Some(pos) => format!("{})", &content[..pos]),
        None => content,
    };
    let mut board = import_dsn(&content).expect("import failed");

    let via_padstack = board
        .padstacks
        .get("Via[0-1]_1400:600_um")
        .map(|p| p.no)
        .expect("via padstack missing");
    let request = BatchRequest {
        trace_half_width: board.rules.get_min_trace_half_width().max(500),
        clearance_class: 1,
        via_padstack,
        via_cost: 50_000.0,
        max_expansions: 100_000,
        ripup_penalty: 0.0,
        deadline: None,
    };
    let limit = TimeLimit::new(30_000);
    batch_route_passes_with_time_limit(&mut board, &request, 1, Some(&limit));

    let mut stats: Vec<(f64, f64, usize, i32)> = Vec::new();
    for (_, item) in board.items() {
        if let ItemKind::PolylineTrace(t) = &item.kind {
            if item.base.component_no != 0 {
                continue; // only routed traces
            }
            let corners = t.polyline.corner_approx_arr();
            let max_c = corners
                .iter()
                .map(|c| c.x.abs().max(c.y.abs()))
                .fold(0.0_f64, f64::max);
            stats.push((
                t.get_length(),
                max_c,
                corners.len(),
                item.base.net_nos.first().copied().unwrap_or(0),
            ));
        }
    }
    stats.sort_by(|a, b| b.0.partial_cmp(&a.0).unwrap());
    let total: f64 = stats.iter().map(|s| s.0).sum();
    println!("{} routed traces, total length {:.3e}", stats.len(), total);
    println!("top 10 by length (length, max|corner|, corners, net):");
    for (len, max_c, n, net) in stats.iter().take(10) {
        println!("  len {len:.3e}  max|corner| {max_c:.3e}  corners {n}  net {net}");
    }

    // dump the worst trace's lines and corners
    let worst_net = stats.first().map(|s| s.3).unwrap_or(0);
    for (_, item) in board.items() {
        if let ItemKind::PolylineTrace(t) = &item.kind {
            if item.base.component_no != 0 || item.base.net_nos.first() != Some(&worst_net) {
                continue;
            }
            let corners = t.polyline.corner_approx_arr();
            if corners
                .iter()
                .all(|c| c.x.abs() < 3.4e7 && c.y.abs() < 3.4e7)
            {
                continue;
            }
            println!("worst trace (net {worst_net}), lines:");
            for l in &t.polyline.arr {
                println!("  {l:?}");
            }
            println!("corners:");
            for c in &corners {
                println!("  ({:.1}, {:.1})", c.x, c.y);
            }
            break;
        }
    }
}
