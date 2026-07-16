//! Port of `SesReader.java`: applying a Specctra session file's routes
//! (wires and vias from `network_out`) to a board imported from the
//! matching design.

use crate::board::basic_board::BasicBoard;
use crate::geometry::planar::{IntPoint, Polyline};
use crate::io::dsn::parse_dsn;

#[derive(Debug)]
pub struct SesImportSummary {
    pub wires: usize,
    pub vias: usize,
    /// `net_out` scopes whose net name does not exist on the board; their
    /// copper is NOT imported (netless copper would be a pure obstacle and
    /// violate against every real net, like Java's SesReader skip).
    pub unknown_nets: Vec<String>,
}

/// Reads the routes of a session file into the board (Java:
/// `SesReader.read`). Coordinates scale from the session's resolution
/// to the board's.
pub fn import_ses(board: &mut BasicBoard, content: &str) -> Result<SesImportSummary, String> {
    let root = parse_dsn(content).map_err(|e| format!("session parse error: {e:?}"))?;
    let routes = root
        .child("routes")
        .ok_or_else(|| "no routes section".to_string())?;
    let resolution_node = routes.child("resolution");
    let ses_resolution: f64 = resolution_node
        .and_then(|r| r.args().nth(1))
        .and_then(|v| v.parse().ok())
        .unwrap_or(board.resolution as f64);
    // The session's `(resolution <unit> <value>)` unit need not match the
    // board's; reconcile through a physical (mm) basis instead of assuming a
    // bare resolution ratio, which is only correct when the units are equal.
    let um_per_unit = |u: &str| -> f64 {
        match u.to_ascii_lowercase().as_str() {
            "mil" => 25.4,
            "inch" | "in" => 25_400.0,
            "mm" => 1000.0,
            "cm" => 10_000.0,
            _ => 1.0, // um (the Specctra default)
        }
    };
    let ses_unit = resolution_node
        .and_then(|r| r.args().next())
        .unwrap_or("um");
    let ses_units_per_mm = ses_resolution.max(1.0) / um_per_unit(ses_unit) * 1000.0;
    let factor = board.board_units_per_mm() / ses_units_per_mm;
    let scale = |v: f64| -> i32 { (v * factor).round() as i32 };
    let network = routes
        .child("network_out")
        .ok_or_else(|| "no network_out section".to_string())?;
    let mut summary = SesImportSummary {
        wires: 0,
        vias: 0,
        unknown_nets: Vec::new(),
    };
    // padstack names -> numbers
    let mut padstack_nos = std::collections::HashMap::new();
    for no in 1..=board.padstacks.count() {
        if let Some(p) = board.padstacks.get_by_no(no) {
            padstack_nos.insert(p.name.clone(), no);
        }
    }
    crate::board::basic_board::set_birth_tag(1);
    for net_node in network.children("net") {
        let Some(net_name) = net_node.arg() else {
            continue;
        };
        let net_nos: Vec<i32> = board
            .rules
            .nets
            .get_by_name(net_name)
            .first()
            .map(|n| vec![n.net_number])
            .unwrap_or_default();
        if net_nos.is_empty() {
            // an unknown session net must not become netless copper (a pure
            // obstacle violating against every real net): skip its routes
            // and report the name
            summary.unknown_nets.push(net_name.to_string());
            continue;
        }
        // propagate the net's own trace clearance class instead of hardcoding
        // the default, so a reloaded session keeps the design's spacing
        let clearance_class = net_nos
            .first()
            .map(|&n| board.rules.get_trace_clearance_class(n))
            .unwrap_or_else(crate::rules::BoardRules::default_clearance_class);
        for wire in net_node.children("wire") {
            let Some(path) = wire.child("path") else {
                continue;
            };
            let Some(layer) = path.arg().and_then(|n| board.layer_structure.get_no(n)) else {
                continue;
            };
            let nums: Vec<f64> = path.args().skip(1).filter_map(|a| a.parse().ok()).collect();
            if nums.len() < 5 {
                continue;
            }
            let half_width = (scale(nums[0]) / 2).max(1);
            let corners: Vec<IntPoint> = nums[1..]
                .chunks_exact(2)
                .map(|c| IntPoint::new(scale(c[0]), scale(c[1])))
                .collect();
            let polyline = Polyline::from_int_points(&corners);
            if polyline.is_empty() {
                continue;
            }
            // Java SesReader inserts session routing USER_FIXED: an
            // imported session is existing copper to preserve, not a
            // draft for the optimizer to rip
            let id = board.insert_trace(
                polyline,
                layer,
                half_width,
                net_nos.clone(),
                clearance_class,
            );
            board.set_fixed_state(id, crate::board::FixedState::UserFixed);
            summary.wires += 1;
        }
        for via in net_node.children("via") {
            let args: Vec<&str> = via.args().collect();
            if args.len() < 3 {
                continue;
            }
            let Some(&padstack_no) = padstack_nos.get(args[0]) else {
                continue;
            };
            let (Ok(x), Ok(y)) = (args[1].parse::<f64>(), args[2].parse::<f64>()) else {
                continue;
            };
            // Java SesReader.processViaScope inserts session vias with
            // attach_allowed = TRUE: the session records legal routing, so
            // a via sitting on its own net's SMD pad keeps the fanout
            // exemption. Hardcoding false made a routed-then-reloaded
            // session fail the same-net drill DRC the original board passed.
            let id = board.insert_via(
                padstack_no,
                IntPoint::new(scale(x), scale(y)),
                net_nos.clone(),
                clearance_class,
                true,
            );
            board.set_fixed_state(id, crate::board::FixedState::UserFixed);
            // register contacts when the via lands mid-trace (a contact
            // needs a trace endpoint at the pad)
            board.split_traces_at_via(id);
            summary.vias += 1;
        }
    }
    crate::board::basic_board::set_birth_tag(0);
    Ok(summary)
}
