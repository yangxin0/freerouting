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

/// Strict connectivity oracle approximating what a Java SES/DRC reload checks:
/// two connection points are joined ONLY when a wire endpoint sits exactly on
/// each. Union-find keyed by exact (x, y): a trace unions its two endpoints;
/// pins/vias are points joined in when a trace endpoint lands on them. A pin
/// whose centre no wire reaches ends up in its own component — a dangling track.
fn every_real_pin_strictly_wired(board: &freerouting::board::basic_board::BasicBoard) -> Result<usize, String> {
    use freerouting::board::ItemKind;
    use std::collections::HashMap;

    fn find(parent: &mut HashMap<(i32, i32), (i32, i32)>, x: (i32, i32)) -> (i32, i32) {
        let mut r = x;
        while let Some(&p) = parent.get(&r) {
            if p == r {
                break;
            }
            r = p;
        }
        // path-compress
        let mut c = x;
        while let Some(&p) = parent.get(&c) {
            if p == r {
                break;
            }
            parent.insert(c, r);
            c = p;
        }
        r
    }

    let mut checked = 0usize;
    for net in 1..=board.rules.nets.max_net_no() {
        // real pins = placed component drill items of this net
        let pins: Vec<(i32, i32)> = board
            .items()
            .filter(|(_, it)| it.base.component_no != 0 && it.base.contains_net(net))
            .filter_map(|(_, it)| match &it.kind {
                ItemKind::Via(v) => Some((v.center.x, v.center.y)),
                _ => None,
            })
            .collect();
        // only nets that actually need routing between >= 2 pins are meaningful
        if pins.len() < 2 {
            continue;
        }
        // and only nets the router claims to have completed
        if !board.net_is_completely_connected(net) {
            continue;
        }

        let mut parent: HashMap<(i32, i32), (i32, i32)> = HashMap::new();
        for &p in &pins {
            parent.entry(p).or_insert(p);
        }
        for (_, it) in board.items() {
            if !it.base.contains_net(net) {
                continue;
            }
            match &it.kind {
                ItemKind::PolylineTrace(t) => {
                    let a = t.first_corner().to_float().round();
                    let b = t.last_corner().to_float().round();
                    let (a, b) = ((a.x, a.y), (b.x, b.y));
                    parent.entry(a).or_insert(a);
                    parent.entry(b).or_insert(b);
                    let (ra, rb) = (find(&mut parent, a), find(&mut parent, b));
                    if ra != rb {
                        parent.insert(ra, rb);
                    }
                }
                ItemKind::Via(v) => {
                    parent.entry((v.center.x, v.center.y)).or_insert((v.center.x, v.center.y));
                }
                _ => {}
            }
        }
        let root0 = find(&mut parent, pins[0]);
        for &p in &pins[1..] {
            if find(&mut parent, p) != root0 {
                return Err(format!(
                    "net {net}: pin at {p:?} is not strictly wired to the net \
                     (no wire endpoint on its connection point — a dangling track)"
                ));
            }
        }
        checked += pins.len();
    }
    Ok(checked)
}

// Enabled for PLANE-FREE fixtures only: the strict oracle joins connection
// points through trace/via endpoints, but not through conduction-area (plane)
// connections, so on a board with GND/power planes it would flag genuinely
// plane-connected pins. SMD-routing-issue-demo and J2 have no planes, so every
// real pin must be reached by a wire endpoint at its connection point. This is
// the regression guard for the finding #2 fix (maze search lands connections at
// the drill center via pin_exit_corner; cycle removal no longer deletes the sole
// wire reaching a pin center).
#[test]
fn routed_nets_reach_pin_connection_points() {
    use freerouting::autoroute::{
        batch_route_passes_with_time_limit, combine_all_traces, pull_tight_all, BatchRequest,
    };
    use freerouting::datastructures::TimeLimit;
    use freerouting::io::import_dsn;

    let root = root();
    for fixture in [
        "fixtures/SMD-routing-issue-demo.dsn",
        "fixtures/Issue026-J2_reference.dsn",
    ] {
        let content = std::fs::read_to_string(format!("{root}/{fixture}"))
            .unwrap_or_else(|_| panic!("fixture {fixture} missing"));
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
        let limit = TimeLimit::new(30_000);
        batch_route_passes_with_time_limit(&mut board, &request, 100, Some(&limit));
        // the SAME post-processing the CLI does
        combine_all_traces(&mut board);
        pull_tight_all(&mut board, 3);

        let checked = every_real_pin_strictly_wired(&board)
            .unwrap_or_else(|e| panic!("{fixture}: {e}"));
        assert!(checked > 0, "{fixture}: no >=2-pin complete nets to verify");
    }
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
    // KiCad report. J2 routes fully and cleanly, so the report must show no
    // unconnected nets AND no clearance violations; each violation, if any,
    // must also name two distinct real items.
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
    assert!(
        report.violations.is_empty(),
        "J2 should route cleanly, but DRC reported {} clearance violation(s)",
        report.violations.len()
    );
    let json = report.to_kicad_json(&board, "j2.dsn");
    assert!(json.contains("schemas.kicad.org/drc.v1.json"));
}
