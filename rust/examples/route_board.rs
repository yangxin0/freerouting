//! Demo: import a DSN file, batch-route all nets, export the session.
//!
//! Usage: cargo run --release --example route_board [path/to/board.dsn]

use freerouting::autoroute::{batch_route_passes_with_time_limit, BatchRequest};
use freerouting::datastructures::TimeLimit;
use freerouting::io::{export_ses, import_dsn};
use std::time::Instant;

fn main() {
    let path = std::env::args()
        .nth(1)
        .unwrap_or_else(|| "../fixtures/Issue093-interf_u.dsn".to_string());
    let strip_wiring = std::env::args().any(|a| a == "--strip-wiring");
    let mut content = std::fs::read_to_string(&path).expect("cannot read input file");
    if strip_wiring {
        if let Some(pos) = content.find("  (wiring") {
            content = format!("{})", &content[..pos]);
            println!("pre-routed wiring stripped: routing from scratch");
        }
    }
    let t0 = Instant::now();
    let mut board = import_dsn(&content).expect("import failed");
    println!(
        "imported {} in {:?}: {} layers, {} padstacks, {} nets, {} items",
        path,
        t0.elapsed(),
        board.layer_structure.layer_count(),
        board.padstacks.count(),
        board.rules.nets.max_net_no(),
        board.item_count()
    );

    // pick a via padstack: the first one whose name starts with "Via"
    let via_padstack = (1..=board.padstacks.count())
        .find(|no| {
            board
                .padstacks
                .get_by_no(*no)
                .is_some_and(|p| p.name.starts_with("Via"))
        })
        .expect("no via padstack");
    let request = BatchRequest {
        trace_half_width: board.rules.get_min_trace_half_width().max(500),
        clearance_class: 1,
        via_padstack,
        via_cost: 50_000.0,
        max_expansions: 100_000,
        ripup_penalty: 0.0,
        deadline: None,
    };

    let t1 = Instant::now();
    let net_count = board.rules.nets.max_net_no();
    // bound the batch to 5 minutes of wall clock
    let time_limit = TimeLimit::new(300_000);
    let result =
        batch_route_passes_with_time_limit(&mut board, &request, 4, Some(&time_limit));
    let mut complete_nets = 0usize;
    let mut incomplete_nets = Vec::new();
    for net_no in 1..=net_count {
        if board.net_is_completely_connected(net_no) {
            complete_nets += 1;
        } else {
            let name = board
                .rules
                .nets
                .get_by_no(net_no)
                .map(|n| n.name.clone())
                .unwrap_or_default();
            incomplete_nets.push(name);
        }
    }
    println!(
        "routing finished in {:?}: {} connections routed, {} failed; {}/{} nets complete",
        t1.elapsed(),
        result.routed_connections,
        result.failed_connections,
        complete_nets,
        net_count
    );
    if !incomplete_nets.is_empty() {
        println!("incomplete nets: {}", incomplete_nets.join(", "));
    }
    let ses = export_ses(&board, "routed_board", 10);
    let out = "routed_board.ses";
    std::fs::write(out, &ses).expect("cannot write session");
    println!("session written to {out} ({} bytes)", ses.len());
}
