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
}

/// Reads the routes of a session file into the board (Java:
/// `SesReader.read`). Coordinates scale from the session's resolution
/// to the board's.
pub fn import_ses(board: &mut BasicBoard, content: &str) -> Result<SesImportSummary, String> {
    let root = parse_dsn(content).map_err(|e| format!("session parse error: {e:?}"))?;
    let routes = root
        .child("routes")
        .ok_or_else(|| "no routes section".to_string())?;
    let ses_resolution: f64 = routes
        .child("resolution")
        .and_then(|r| r.args().nth(1))
        .and_then(|v| v.parse().ok())
        .unwrap_or(board.resolution as f64);
    let factor = board.resolution as f64 / ses_resolution.max(1.0);
    let scale = |v: f64| -> i32 { (v * factor).round() as i32 };
    let network = routes
        .child("network_out")
        .ok_or_else(|| "no network_out section".to_string())?;
    let mut summary = SesImportSummary { wires: 0, vias: 0 };
    // padstack names -> numbers
    let mut padstack_nos = std::collections::HashMap::new();
    for no in 1..=board.padstacks.count() {
        if let Some(p) = board.padstacks.get_by_no(no) {
            padstack_nos.insert(p.name.clone(), no);
        }
    }
    crate::board::basic_board::set_birth_tag(1);
    for net_node in network.children("net") {
        let Some(net_name) = net_node.arg() else { continue };
        let net_nos: Vec<i32> = board
            .rules
            .nets
            .get_by_name(net_name)
            .first()
            .map(|n| vec![n.net_number])
            .unwrap_or_default();
        for wire in net_node.children("wire") {
            let Some(path) = wire.child("path") else { continue };
            let Some(layer) = path.arg().and_then(|n| board.layer_structure.get_no(n))
            else {
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
            board.insert_trace(polyline, layer, half_width, net_nos.clone(), 1);
            summary.wires += 1;
        }
        for via in net_node.children("via") {
            let args: Vec<&str> = via.args().collect();
            if args.len() < 3 {
                continue;
            }
            let Some(&padstack_no) = padstack_nos.get(args[0]) else { continue };
            let (Ok(x), Ok(y)) = (args[1].parse::<f64>(), args[2].parse::<f64>()) else {
                continue;
            };
            board.insert_via(
                padstack_no,
                IntPoint::new(scale(x), scale(y)),
                net_nos.clone(),
                1,
                false,
            );
            summary.vias += 1;
        }
    }
    crate::board::basic_board::set_birth_tag(0);
    Ok(summary)
}
