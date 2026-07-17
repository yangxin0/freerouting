//! Routes a single named net on the (stripped) board and reports.
use freerouting::autoroute::{route_net_with_ripup, BatchRequest};
use freerouting::io::import_dsn;

fn main() {
    let path = std::env::args()
        .nth(1)
        .expect("usage: probe_net <dsn> <net>");
    let name = std::env::args().nth(2).expect("net name");
    let content = std::fs::read_to_string(&path).expect("read failed");
    let content = match content.find("  (wiring") {
        Some(pos) => format!("{})", &content[..pos]),
        None => content,
    };
    let mut board = import_dsn(&content).expect("import failed");
    let net_no = (1..=board.rules.nets.max_net_no())
        .find(|&n| {
            board
                .rules
                .nets
                .get_by_no(n)
                .is_some_and(|net| net.name == name)
        })
        .expect("net not found");
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
        via_clearance_class: 0,
        via_attach_allowed: false,
        via_cost: 50_000.0,
        max_expansions: 100_000,
        ripup_penalty: 0.0,
        deadline: None,
    };
    let t = std::time::Instant::now();
    let result = route_net_with_ripup(&mut board, net_no, &request, 100_000.0);
    println!(
        "net {name} (#{net_no}): routed {} failed {} complete {} in {:?}",
        result.routed_connections,
        result.failed_connections,
        board.net_is_completely_connected(net_no),
        t.elapsed()
    );
}
