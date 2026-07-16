//! Integration coverage for the boundaries the unit tests do not reach:
//! the CLI binary end to end, the SES *import* path, and a full-board DRC
//! run over an actually-routed board. These correspond to the audit's
//! test-parity gaps (no CLI, SES-import, or full-board DRC coverage).

use std::process::Command;

fn root() -> String {
    concat!(env!("CARGO_MANIFEST_DIR"), "/..").to_string()
}

/// A small board that routes to completion quickly, so the CLI test stays
/// fast and deterministic.
const SMALL_FIXTURE: &str = "fixtures/Issue026-J2_reference.dsn";

#[test]
fn cli_routes_a_board_and_writes_a_session() {
    let root = root();
    let dsn = format!("{root}/{SMALL_FIXTURE}");
    assert!(
        std::path::Path::new(&dsn).exists(),
        "required fixture {dsn} missing"
    );
    let out_dir = std::env::temp_dir();
    let ses = out_dir.join("fr_cli_test_j2.ses");
    let _ = std::fs::remove_file(&ses);

    let status = Command::new(env!("CARGO_BIN_EXE_freerouting"))
        .args([
            "-de",
            &dsn,
            "-do",
            ses.to_str().unwrap(),
            "-tl",
            "30",
        ])
        .status()
        .expect("failed to run freerouting binary");

    // J2 routes fully, so the exit code (computed after optimization) is 0
    assert!(
        status.success(),
        "CLI exited with {status:?}; expected success (all nets routed)"
    );
    let ses_text = std::fs::read_to_string(&ses).expect("session file not written");
    assert!(ses_text.contains("(session"), "output is not a session file");
    assert!(
        ses_text.contains("network_out"),
        "session has no routed network"
    );
    let _ = std::fs::remove_file(&ses);
}

#[test]
#[ignore = "electrical equivalence (finding #2) is NOT yet achieved: routed \
traces stop inside a pad rather than at the pin connection point. Enable once \
the maze search terminates connections at the drill connection point."]
fn routed_nets_reach_pin_connection_points() {
    // TARGET PROPERTY (currently unmet — see the ignore reason): every pin of a
    // net the router reports complete must be physically reached at its
    // connection point (drill center) by actual wiring (a trace endpoint or a
    // routing via), the property Java's SES reader enforces. Rust's lenient
    // in-pad containment rule counts a trace that stops ~50 um off center as
    // connected, so such a net reloads with dangling tracks. Pins are excluded
    // from the reached set so a pin cannot trivially prove its own reachedness.
    use freerouting::autoroute::{
        batch_route_passes_with_time_limit, combine_all_traces, pull_tight_all, BatchRequest,
    };
    use freerouting::board::ItemKind;
    use freerouting::datastructures::TimeLimit;
    use freerouting::io::import_dsn;

    let root = root();
    let content = std::fs::read_to_string(format!("{root}/fixtures/SMD-routing-issue-demo.dsn"))
        .expect("SMD fixture missing");
    let mut board = import_dsn(&content).expect("import failed");
    let request = BatchRequest {
        trace_half_width: board.rules.get_min_trace_half_width().max(500),
        clearance_class: 1,
        via_padstack: (1..=board.padstacks.count())
            .find(|no| {
                board
                    .padstacks
                    .get_by_no(*no)
                    .is_some_and(|p| p.name.starts_with("Via"))
            })
            .unwrap_or(1),
        via_cost: 50_000.0,
        max_expansions: 100_000,
        ripup_penalty: 0.0,
        deadline: None,
    };
    let limit = TimeLimit::new(20_000);
    batch_route_passes_with_time_limit(&mut board, &request, 100, Some(&limit));
    // Run the SAME post-processing the CLI does: combine + pull-tight. A prior
    // version verified before this stage and missed that a naive connecting
    // stub is deleted here by cycle removal.
    combine_all_traces(&mut board);
    pull_tight_all(&mut board, 3);

    let mut checked_pins = 0usize;
    for net in 1..=board.rules.nets.max_net_no() {
        if !board.net_is_completely_connected(net) {
            continue;
        }
        // Points reached by actual WIRING: trace endpoints and routing (non-pin)
        // via centers only. Pins are deliberately EXCLUDED — including them would
        // make every pin trivially prove its own reachedness (the vacuous check
        // this test previously had).
        let mut reached: Vec<(i32, i32)> = Vec::new();
        for (_, item) in board.items() {
            if !item.base.contains_net(net) {
                continue;
            }
            match &item.kind {
                ItemKind::PolylineTrace(t) => {
                    for c in [t.first_corner(), t.last_corner()] {
                        let p = c.to_float().round();
                        reached.push((p.x, p.y));
                    }
                }
                ItemKind::Via(v) if item.base.component_no == 0 => {
                    reached.push((v.center.x, v.center.y))
                }
                _ => {}
            }
        }
        // every pin (placed drill item) of this net must be reached at its center
        for (_, item) in board.items() {
            if item.base.component_no == 0 || !item.base.contains_net(net) {
                continue;
            }
            if let ItemKind::Via(pin) = &item.kind {
                let center = (pin.center.x, pin.center.y);
                assert!(
                    reached.contains(&center),
                    "net {net}: pin at {center:?} is not reached at its connection point \
                     by wiring (would reload as a dangling track)"
                );
                checked_pins += 1;
            }
        }
    }
    assert!(checked_pins > 0, "no complete nets with pins were verified");
}

#[test]
fn cli_help_exits_success() {
    let status = Command::new(env!("CARGO_BIN_EXE_freerouting"))
        .arg("--help")
        .status()
        .expect("failed to run freerouting binary");
    assert!(status.success());
}

#[test]
fn ses_import_reconnects_a_routed_net() {
    use freerouting::autoroute::{batch_route_passes_with_time_limit, BatchRequest};
    use freerouting::datastructures::TimeLimit;
    use freerouting::io::{export_ses, import_dsn, import_ses};

    let root = root();
    let content = std::fs::read_to_string(format!("{root}/{SMALL_FIXTURE}"))
        .expect("fixture missing");

    // Route a board via the real batch path, export its SES, then import that
    // SES onto a *fresh* import of the same design and confirm the wiring
    // reconnects the bulk of the nets that were complete in the routed board.
    // The round-trip is NOT exact: the unresolved connection-point issue (see
    // the ignored `routed_nets_reach_pin_connection_points` test) can drop a net
    // whose trace ended off the pin centre, so a small shortfall is tolerated.
    let mut routed = import_dsn(&content).expect("import failed");
    let request = BatchRequest {
        trace_half_width: routed.rules.get_min_trace_half_width().max(500),
        clearance_class: 1,
        via_padstack: (1..=routed.padstacks.count())
            .find(|no| {
                routed
                    .padstacks
                    .get_by_no(*no)
                    .is_some_and(|p| p.name.starts_with("Via"))
            })
            .unwrap_or(1),
        via_cost: 50_000.0,
        max_expansions: 100_000,
        ripup_penalty: 0.0,
        deadline: None,
    };
    let net_nos: Vec<i32> = (1..=routed.rules.nets.max_net_no()).collect();
    let limit = TimeLimit::new(30_000);
    batch_route_passes_with_time_limit(&mut routed, &request, 100, Some(&limit));
    let routed_complete: usize = net_nos
        .iter()
        .filter(|n| routed.net_is_completely_connected(**n))
        .count();
    assert!(routed_complete > 0, "nothing routed to round-trip");
    let ses = export_ses(&routed, "j2", routed.resolution);

    // fresh board (no wiring) + the exported session
    let mut fresh = import_dsn(&content).expect("import failed");
    let summary = import_ses(&mut fresh, &ses).expect("SES import failed");
    assert!(summary.wires > 0, "SES import applied no wires");
    let after: usize = net_nos
        .iter()
        .filter(|n| fresh.net_is_completely_connected(**n))
        .count();
    // the reloaded session must reproduce all but a small residual of the
    // routed board's connectivity (>= 90%), well above the previous "at least
    // one net" bound; the residual is the unresolved connection-point gap
    assert!(after > 0, "SES import reconnected no nets");
    assert!(
        after * 10 >= routed_complete * 9,
        "SES round-trip lost too many nets: routed {routed_complete}, reloaded {after}"
    );
}

#[test]
fn full_board_drc_over_a_routed_board() {
    use freerouting::autoroute::{batch_route_passes_with_time_limit, BatchRequest};
    use freerouting::datastructures::TimeLimit;
    use freerouting::drc::check_board;
    use freerouting::io::import_dsn;

    let root = root();
    let content = std::fs::read_to_string(format!("{root}/{SMALL_FIXTURE}"))
        .expect("fixture missing");
    let mut board = import_dsn(&content).expect("import failed");
    let request = BatchRequest {
        trace_half_width: board.rules.get_min_trace_half_width().max(500),
        clearance_class: 1,
        via_padstack: (1..=board.padstacks.count())
            .find(|no| {
                board
                    .padstacks
                    .get_by_no(*no)
                    .is_some_and(|p| p.name.starts_with("Via"))
            })
            .unwrap_or(1),
        via_cost: 50_000.0,
        max_expansions: 100_000,
        ripup_penalty: 0.0,
        deadline: None,
    };
    // route through the real multi-pass batch path (with ripup), as the CLI does
    let limit = TimeLimit::new(30_000);
    batch_route_passes_with_time_limit(&mut board, &request, 100, Some(&limit));

    // A full-board DRC must run to completion and produce a serializable
    // KiCad report. J2 routes cleanly, so there should be no unconnected
    // nets left; the violation set must be internally consistent (each
    // violation names two distinct real items).
    let report = check_board(&board);
    assert!(
        report.unconnected.is_empty(),
        "J2 should route fully: {} nets left unconnected",
        report.unconnected.len()
    );
    for v in &report.violations {
        assert_ne!(v.first_item, v.second_item, "violation with itself");
        assert!(board.get_item(v.first_item).is_some());
        assert!(board.get_item(v.second_item).is_some());
    }
    let json = report.to_kicad_json(&board, "j2.dsn");
    assert!(json.contains("schemas.kicad.org/drc.v1.json"));
}
