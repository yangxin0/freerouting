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

    let output = Command::new(env!("CARGO_BIN_EXE_freerouting"))
        .args(["-de", &dsn, "-do", ses.to_str().unwrap(), "-tl", "30"])
        .output()
        .expect("failed to run freerouting binary");

    // J2 routes to completion, so the CLI exits 0 (the exit code is computed
    // after routing from the completion count — see main.rs). This exercises the
    // whole pipeline end to end and asserts the honest outcome. (Before the
    // finding #1 fix the maze faked 24/24 by deleting blocking pins; the finding
    // #6 investigation later confirmed the router genuinely reaches 24/24.)
    assert!(
        output.status.success(),
        "CLI should route J2 fully (exit 0); got {:?}\nstderr: {}",
        output.status.code(),
        String::from_utf8_lossy(&output.stderr)
    );
    let ses_text = std::fs::read_to_string(&ses).expect("session file not written");
    assert!(
        ses_text.contains("(session"),
        "output is not a session file"
    );
    assert!(
        ses_text.contains("network_out"),
        "session has no routed network"
    );
    let _ = std::fs::remove_file(&ses);
}

/// Strict connectivity oracle approximating what a Java SES/DRC reload checks:
/// two connection points are joined ONLY when a wire endpoint sits exactly on
/// each. Union-find keyed by exact (x, y, layer): a trace unions its two
/// endpoints on its own layer; a via/pin unions the same (x, y) across every
/// layer of its padstack span (only a via bridges layers — two traces meeting
/// at the same (x, y) on different layers with no via are NOT joined). A pin
/// whose centre no wire reaches on a valid layer ends up in its own component —
/// a dangling track.
fn every_real_pin_strictly_wired(
    board: &freerouting::board::basic_board::BasicBoard,
) -> Result<usize, String> {
    use freerouting::board::ItemKind;
    use std::collections::HashMap;

    type Node = (i32, i32, usize);

    fn find(parent: &mut HashMap<Node, Node>, x: Node) -> Node {
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

    fn union(parent: &mut HashMap<Node, Node>, a: Node, b: Node) {
        parent.entry(a).or_insert(a);
        parent.entry(b).or_insert(b);
        let (ra, rb) = (find(parent, a), find(parent, b));
        if ra != rb {
            parent.insert(ra, rb);
        }
    }

    // Union every layer of a drill item's padstack span at (x, y): this is what
    // makes a via a layer bridge. Returns the representative node, or None if
    // the padstack is unknown.
    let span_nodes =
        |parent: &mut HashMap<Node, Node>, padstack: usize, x: i32, y: i32| -> Option<Node> {
            let ps = board.padstacks.get_by_no(padstack)?;
            let (lo, hi) = (ps.from_layer(), ps.to_layer());
            for l in lo..=hi {
                parent.entry((x, y, l)).or_insert((x, y, l));
            }
            for l in lo..hi {
                union(parent, (x, y, l), (x, y, l + 1));
            }
            Some((x, y, lo))
        };

    let mut checked = 0usize;
    for net in 1..=board.rules.nets.max_net_no() {
        // real pins = placed component drill items of this net, with padstack
        let pins: Vec<(usize, i32, i32)> = board
            .items()
            .filter(|(_, it)| it.base.component_no != 0 && it.base.contains_net(net))
            .filter_map(|(_, it)| match &it.kind {
                ItemKind::Via(v) => Some((v.padstack, v.center.x, v.center.y)),
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

        let mut parent: HashMap<Node, Node> = HashMap::new();
        let pin_reps: Vec<Node> = pins
            .iter()
            .filter_map(|&(ps, x, y)| span_nodes(&mut parent, ps, x, y))
            .collect();
        for (_, it) in board.items() {
            if !it.base.contains_net(net) {
                continue;
            }
            match &it.kind {
                ItemKind::PolylineTrace(t) => {
                    let a = t.first_corner().to_float().round();
                    let b = t.last_corner().to_float().round();
                    let l = t.layer;
                    union(&mut parent, (a.x, a.y, l), (b.x, b.y, l));
                }
                ItemKind::Via(v) => {
                    span_nodes(&mut parent, v.padstack, v.center.x, v.center.y);
                }
                _ => {}
            }
        }
        let root0 = find(&mut parent, pin_reps[0]);
        for &p in &pin_reps[1..] {
            if find(&mut parent, p) != root0 {
                return Err(format!(
                    "net {net}: pin at {:?} is not strictly wired to the net \
                     (no wire endpoint on its connection point — a dangling track)",
                    (p.0, p.1)
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
    use freerouting::board::ItemKind;
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

        // Snapshot every component pin BEFORE routing. The router must never
        // delete or move a pin (finding #1: maze rip-up could remove a pin that
        // became an obstacle room). Enumerating pins only from the post-route
        // board would let a deleted pin silently vanish before validation, so
        // we capture identity up front and assert survival afterwards.
        let original_pins: Vec<(freerouting::board::basic_board::ItemId, i32, i32, Vec<i32>)> =
            board
                .items()
                .filter(|(_, it)| it.base.component_no != 0)
                .filter_map(|(id, it)| match &it.kind {
                    ItemKind::Via(v) => {
                        Some((*id, v.center.x, v.center.y, it.base.net_nos.clone()))
                    }
                    _ => None,
                })
                .collect();
        assert!(
            !original_pins.is_empty(),
            "{fixture}: no component pins found before routing"
        );
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
            via_clearance_class: 0,
            via_attach_allowed: false,
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

        // Every pin that existed before routing must still exist afterwards,
        // unmoved and with its nets intact. This is the direct regression guard
        // for finding #1 (a foreign-net pin ripped up as a maze obstacle).
        for (id, cx, cy, nets) in &original_pins {
            match board.get_item(*id).map(|it| (&it.kind, &it.base.net_nos)) {
                Some((ItemKind::Via(v), got_nets))
                    if v.center.x == *cx && v.center.y == *cy && got_nets == nets => {}
                Some(_) => panic!(
                    "{fixture}: component pin (id {id}) at ({cx},{cy}) was moved or its net \
                     changed during routing"
                ),
                None => panic!(
                    "{fixture}: component pin (id {id}) at ({cx},{cy}) was deleted during routing \
                     (maze rip-up removed a pin — finding #1)"
                ),
            }
        }

        let checked =
            every_real_pin_strictly_wired(&board).unwrap_or_else(|e| panic!("{fixture}: {e}"));
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
    let content =
        std::fs::read_to_string(format!("{root}/{SMALL_FIXTURE}")).expect("fixture missing");

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
        via_clearance_class: 0,
        via_attach_allowed: false,
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
    assert!(
        summary.unknown_nets.is_empty(),
        "own session must not name unknown nets"
    );
    let after: usize = net_nos
        .iter()
        .filter(|n| fresh.net_is_completely_connected(**n))
        .count();
    // FULL electrical equivalence on reload: every net the routed board
    // completed must be complete again after the session round trip (the
    // former 90% allowance dated from before the pin-exit and
    // center-anchored-stub work; J2 measures 100%)
    assert_eq!(
        after, routed_complete,
        "SES round-trip must preserve every completed net"
    );
    // DRC equivalence, not just connectivity: the reloaded session must be
    // exactly as clean as the routed board it came from (imported vias
    // carry the real attach_allowed flag, so the same-net drill exemptions
    // survive the round trip)
    let routed_violations = freerouting::drc::check_board(&routed).violations.len();
    let reloaded_violations = freerouting::drc::check_board(&fresh).violations.len();
    assert!(
        reloaded_violations <= routed_violations,
        "SES reload must not introduce violations: routed {routed_violations}, \
         reloaded {reloaded_violations}"
    );

    // an unknown net scope must be skipped (reported), never imported as
    // netless copper that violates against every real net
    let renamed = ses.replacen("(net \"", "(net \"UNKNOWN-", 1);
    assert_ne!(renamed, ses, "fixture session should contain a net scope");
    let mut fresh2 = import_dsn(&content).expect("import failed");
    let summary2 = import_ses(&mut fresh2, &renamed).expect("SES import failed");
    assert_eq!(
        summary2.unknown_nets.len(),
        1,
        "the renamed scope must be reported as unknown"
    );
    assert!(
        fresh2.items().all(|(_, it)| it.base.component_no != 0
            || it.base.net_count() > 0
            || !matches!(
                it.kind,
                freerouting::board::ItemKind::Via(_)
                    | freerouting::board::ItemKind::PolylineTrace(_)
            )),
        "no netless copper may be imported from an unknown net scope"
    );
}

#[test]
fn full_board_drc_over_a_routed_board() {
    use freerouting::autoroute::{batch_route_passes_with_time_limit, BatchRequest};
    use freerouting::datastructures::TimeLimit;
    use freerouting::drc::check_board;
    use freerouting::io::import_dsn;

    let root = root();
    let content =
        std::fs::read_to_string(format!("{root}/{SMALL_FIXTURE}")).expect("fixture missing");
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
        via_clearance_class: 0,
        via_attach_allowed: false,
        via_cost: 50_000.0,
        max_expansions: 100_000,
        ripup_penalty: 0.0,
        deadline: None,
    };
    // route through the real multi-pass batch path (with ripup), as the CLI does
    let limit = TimeLimit::new(30_000);
    batch_route_passes_with_time_limit(&mut board, &request, 100, Some(&limit));

    // A full-board DRC must run to completion and produce a serializable KiCad
    // report. J2 routes fully AND cleanly: every net connected, no clearance
    // violations, and each violation (if any) names two distinct real items.
    // (Before the finding #1 fix the maze faked completion by deleting blocking
    // pins; finding #6 then confirmed the router genuinely reaches 24/24.)
    let report = check_board(&board);
    assert!(
        report.unconnected.is_empty(),
        "J2 should route fully: {} nets unconnected",
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

    // Off-center termination regression bound (the assessed electrical-
    // equivalence gap): after the CLI's own post-processing, a route-trace
    // endpoint inside a same-net drill pad OFF its connection point risks
    // reading as an open on a strict Java reload — UNLESS the trace's
    // other end IS the drill center (a center-anchored stub: the center
    // connection exists, the off-center end is a harmless dangling tail).
    // pin_exit_corner and endpoint-preserving pull-tight keep the risky
    // class at zero on J2; a regression shows up as a jump here.
    freerouting::autoroute::combine_all_traces(&mut board);
    freerouting::autoroute::pull_tight_all(&mut board, 3);
    let mut off_center = 0usize;
    let mut total_ends = 0usize;
    for (_, item) in board.items() {
        let freerouting::board::ItemKind::PolylineTrace(t) = &item.kind else {
            continue;
        };
        if item.base.component_no != 0 {
            continue;
        }
        let ends = [t.first_corner(), t.last_corner()];
        for (i, corner) in ends.iter().enumerate() {
            total_ends += 1;
            let cp = corner.to_float().round();
            let other_end = ends[1 - i].to_float().round();
            for (_, other) in board.items() {
                let freerouting::board::ItemKind::Via(v) = &other.kind else {
                    continue;
                };
                if !other.base.shares_net(&item.base) || v.center == cp {
                    continue;
                }
                let in_pad = other.tile_shapes(&board.padstacks).iter().any(|(s, l)| {
                    *l == t.layer && s.contains(&freerouting::geometry::planar::Point::Int(cp))
                });
                if in_pad && other_end != v.center {
                    off_center += 1;
                    break;
                }
            }
        }
    }
    assert!(total_ends > 0, "routed board has trace ends to audit");
    assert_eq!(
        off_center, 0,
        "off-center pad terminations regressed: {off_center} of {total_ends} trace ends \
         risk reading as opens on a strict (Java) reload (measured 0 since round six)"
    );
}

/// Finding #6: J2 must route to zero unconnected nets (as Java does), not the
/// 22-23/24 the second-round code reached. The gap turned out to be
/// convergence time in the slow debug build, not an algorithmic wall — the
/// router genuinely completes J2. Rust is still slower than Java (a separate
/// performance gap), but the multi-pass ripup loop converges to 24/24 well
/// within the routing budget.
#[test]
fn j2_routes_fully_like_java() {
    let root = root();
    let dsn = format!("{root}/{SMALL_FIXTURE}");
    let ses = std::env::temp_dir().join("fr_j2_fullroute.ses");
    let _ = std::fs::remove_file(&ses);
    let status = Command::new(env!("CARGO_BIN_EXE_freerouting"))
        .args(["-de", &dsn, "-do", ses.to_str().unwrap(), "-tl", "30"])
        .status()
        .expect("failed to run freerouting binary");
    let _ = std::fs::remove_file(&ses);
    // exit 0 requires every net connected (see main.rs completion check)
    assert!(
        status.success(),
        "J2 should route fully (exit 0); got {status:?}"
    );
}
