//! End-to-end integration: import a real KiCad DSN fixture, batch-route a
//! net, and export the session file.

use freerouting::autoroute::{route_net, BatchRequest};
use freerouting::io::{export_ses, import_dsn, parse_dsn};

#[test]
fn import_route_export_real_board() {
    let root = concat!(env!("CARGO_MANIFEST_DIR"), "/..");
    let path = format!("{root}/fixtures/Issue093-interf_u.dsn");
    let Ok(content) = std::fs::read_to_string(&path) else {
        eprintln!("fixture not present; skipping");
        return;
    };
    // strip the pre-routed wiring so the test exercises actual routing
    let content = match content.find("  (wiring") {
        Some(pos) => format!("{})", &content[..pos]),
        None => content,
    };
    let mut board = import_dsn(&content).expect("import failed");

    // route the two-pin net /ACK
    let ack = board
        .rules
        .nets
        .get_by_name("/ACK")
        .first()
        .map(|n| n.net_number)
        .expect("/ACK missing");
    assert!(!board.net_is_completely_connected(ack));

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
    let result = route_net(&mut board, ack, &request);
    assert_eq!(result.failed_connections, 0, "routing /ACK failed");
    assert_eq!(result.routed_connections, 1);
    assert!(board.net_is_completely_connected(ack));

    // export the session and check it parses with the routed net inside
    let ses = export_ses(&board, "interf_u", 10);
    let parsed = parse_dsn(&ses).expect("SES not parseable");
    assert_eq!(parsed.name(), Some("session"));
    let network_out = parsed
        .child("routes")
        .and_then(|r| r.child("network_out"))
        .expect("network_out");
    let routed_net = network_out
        .children("net")
        .find(|n| n.arg() == Some("/ACK"))
        .expect("/ACK not in session");
    assert!(routed_net.child("wire").is_some(), "no wire for /ACK");

    // the routed wires stay inside the board outline (bounding box of the
    // boundary path in the file, scaled by resolution 10)
    for wire in routed_net.children("wire") {
        let path = wire.child("path").expect("path");
        let coords: Vec<f64> = path.args().skip(2).filter_map(|a| a.parse().ok()).collect();
        for pair in coords.chunks_exact(2) {
            let (x, y) = (pair[0], pair[1]);
            assert!(
                (793750.0..=1949450.0).contains(&x) && (-1424940.0..=-342900.0).contains(&y),
                "wire corner ({x}, {y}) outside the board outline bbox"
            );
        }
    }
}
