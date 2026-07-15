//! Command line interface, compatible with the basic flags of the Java
//! jar (`java -jar freerouting.jar -de input.dsn -do output.ses`).
//!
//! The GUI of the Java original is deliberately not ported.

use freerouting::autoroute::{
    batch_route_passes_with_time_limit, pull_tight_all, total_trace_length, BatchRequest,
};
use freerouting::datastructures::TimeLimit;
use freerouting::io::{export_ses, import_dsn};
use std::process::ExitCode;
use std::time::Instant;

const USAGE: &str = "\
freerouting (Rust) — PCB auto-router

Usage: freerouting -de <input.dsn> [options]

Options:
  -de <file.dsn>     design file to route (required)
  -do <file.ses>     session output file (default: input file with .ses)
  -mp <n>            maximum number of ripup passes (default 99)
  -tl <seconds>      wall-clock time limit for routing (default 300)
  --strip-wiring     remove the pre-routed wiring and route from scratch
  --fanout           fan out SMD pins to vias before routing
  --angle <mode>     trace angle restriction: none, 45, 90 (default 45)
  --drc-report <f>   write a KiCad-format DRC report (JSON) after routing
  --export-dsn <f>   write the routed design as a Specctra DSN file
  --import-ses <f>   apply an existing session file before routing
  --threads <n>      optimizer worker threads (default 1)
  --rules <f>        apply a .rules file before routing
  --export-rules <f> write the design rules to a .rules file
  --api-server <p>   run the REST API server on port <p> (no routing)
  --ratsnest <f>     write the unconnected airlines as JSON after routing
  --profile <f>      apply a JSON router-settings profile (maxPasses,
                     viaCosts, timeLimitSeconds, angleRestriction, threads)
  -h, --help         show this help";

fn main() -> ExitCode {
    let args: Vec<String> = std::env::args().skip(1).collect();
    if args.iter().any(|a| a == "-h" || a == "--help") || args.is_empty() {
        println!("{USAGE}");
        return ExitCode::SUCCESS;
    }
    let flag_value = |flag: &str| -> Option<&str> {
        args.iter()
            .position(|a| a == flag)
            .and_then(|i| args.get(i + 1))
            .map(|s| s.as_str())
    };
    if let Some(port) = flag_value("--api-server").and_then(|v| v.parse::<u16>().ok()) {
        let seconds: u64 = flag_value("-tl").and_then(|v| v.parse().ok()).unwrap_or(300);
        return match freerouting::api::serve(port, seconds) {
            Ok(()) => ExitCode::SUCCESS,
            Err(e) => {
                eprintln!("error: {e}");
                ExitCode::FAILURE
            }
        };
    }
    let Some(design) = flag_value("-de") else {
        eprintln!("error: -de <input.dsn> is required\n\n{USAGE}");
        return ExitCode::FAILURE;
    };
    let output = flag_value("-do")
        .map(str::to_string)
        .unwrap_or_else(|| {
            let stem = design.strip_suffix(".dsn").unwrap_or(design);
            format!("{stem}.ses")
        });
    // like the Java jar, passes are effectively unlimited by default and
    // the wall clock (-tl) is the real bound; a JSON profile (Java:
    // RouterSettings) provides defaults that explicit flags override
    let profile = flag_value("--profile")
        .and_then(|p| std::fs::read_to_string(p).ok())
        .and_then(|t| freerouting::io::json::parse_json(&t).ok());
    let prof_num = |key: &str| -> Option<f64> {
        profile.as_ref().and_then(|p| p.get(key)).and_then(|v| v.as_f64())
    };
    let prof_str = |key: &str| -> Option<String> {
        profile
            .as_ref()
            .and_then(|p| p.get(key))
            .and_then(|v| v.as_str())
            .map(str::to_string)
    };
    let max_passes: usize = flag_value("-mp")
        .and_then(|v| v.parse().ok())
        .or(prof_num("maxPasses").map(|v| v as usize))
        .unwrap_or(99);
    let limit_s: u64 = flag_value("-tl")
        .and_then(|v| v.parse().ok())
        .or(prof_num("timeLimitSeconds").map(|v| v as u64))
        .unwrap_or(300);
    let strip_wiring = args.iter().any(|a| a == "--strip-wiring");
    let angle_mode = flag_value("--angle")
        .map(str::to_string)
        .or(prof_str("angleRestriction"))
        .unwrap_or_else(|| "45".to_string());
    let profile_via_costs = prof_num("viaCosts");
    let profile_threads = prof_num("threads").map(|v| v as usize);

    let mut content = match std::fs::read_to_string(design) {
        Ok(c) => c,
        Err(e) => {
            eprintln!("error: cannot read {design}: {e}");
            return ExitCode::FAILURE;
        }
    };
    if strip_wiring {
        if let Some(pos) = content.find("  (wiring") {
            content = format!("{})", &content[..pos]);
        }
    }
    let t0 = Instant::now();
    let mut board = if design.ends_with(".json") {
        match freerouting::io::import_kicad_json(&content) {
            Ok(b) => b,
            Err(e) => {
                eprintln!("error: {e}");
                return ExitCode::FAILURE;
            }
        }
    } else {
        match import_dsn(&content) {
            Ok(b) => b,
            Err(e) => {
                eprintln!("error: {e}");
                return ExitCode::FAILURE;
            }
        }
    };
    board.rules.set_trace_angle_restriction(match angle_mode.as_str() {
        "none" | "any" => freerouting::board::AngleRestriction::None,
        "90" => freerouting::board::AngleRestriction::NinetyDegree,
        _ => freerouting::board::AngleRestriction::FortyfiveDegree,
    });
    println!(
        "imported {design} in {:?}: {} layers, {} nets, {} items",
        t0.elapsed(),
        board.layer_structure.layer_count(),
        board.rules.nets.max_net_no(),
        board.item_count()
    );

    if let Some(rules_path) = flag_value("--rules") {
        match std::fs::read_to_string(rules_path) {
            Ok(text) => match freerouting::io::read_rules(&mut board, &text) {
                Ok(n) => println!("rules applied: {n} settings"),
                Err(e) => eprintln!("error: {e}"),
            },
            Err(e) => eprintln!("error: cannot read {rules_path}: {e}"),
        }
    }
    if let Some(ses_path) = flag_value("--import-ses") {
        match std::fs::read_to_string(ses_path) {
            Ok(text) => match freerouting::io::import_ses(&mut board, &text) {
                Ok(s) => println!("session applied: {} wires, {} vias", s.wires, s.vias),
                Err(e) => eprintln!("error: {e}"),
            },
            Err(e) => eprintln!("error: cannot read {ses_path}: {e}"),
        }
    }

    // via padstack: first named "Via*", else any all-layer padstack
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
        via_cost: profile_via_costs.unwrap_or(50_000.0),
        max_expansions: 100_000,
        ripup_penalty: 0.0,
        deadline: None,
    };

    let t1 = Instant::now();
    let time_limit = TimeLimit::new(limit_s.saturating_mul(1000));
    if args.iter().any(|a| a == "--fanout") {
        let fanned = freerouting::autoroute::fanout_board(&mut board, &request, 20, Some(&time_limit));
        println!("fanout: {fanned} pins fanned out");
    }
    let result =
        batch_route_passes_with_time_limit(&mut board, &request, max_passes, Some(&time_limit));
    let net_count = board.rules.nets.max_net_no() as usize;
    let complete = (1..=net_count)
        .filter(|&n| board.net_is_completely_connected(n as i32))
        .count();
    println!(
        "routed in {:?}: {} connections, {} failed; {complete}/{net_count} nets complete",
        t1.elapsed(),
        result.routed_connections,
        result.failed_connections
    );
    let stats = freerouting::scoring::BoardStatistics::collect(&board);
    let score =
        stats.normalized_score(&freerouting::scoring::ScoringSettings::default());
    println!(
        "score: {score:.2} ({} unrouted, {} violations, {} vias, {:.1} mm)",
        stats.incomplete_count, stats.clearance_violations, stats.via_count, stats.total_length_mm
    );

    let opt_limit = TimeLimit::new(30_000);
    let opt_threads: usize = flag_value("--threads")
        .and_then(|v| v.parse().ok())
        .or(profile_threads)
        .unwrap_or(1);
    let improved = if std::env::var_os("FR_NO_OPT").is_some() {
        0
    } else {
        freerouting::autoroute::optimize_route_multithreaded(
        &mut board,
        &request,
        opt_threads,
        Some(&opt_limit),
    )
    };
    if improved > 0 {
        println!("optimizer: {improved} nets improved");
    }
    let combined = freerouting::autoroute::combine_all_traces(&mut board);
    if combined > 0 {
        println!("normalized: {combined} trace fragments combined");
    }
    let len_before = total_trace_length(&board);
    let removed = pull_tight_all(&mut board, 3);
    let len_after = total_trace_length(&board);
    if len_before > 0.0 {
        println!(
            "pull tight: {removed} corners removed, length {len_before:.0} -> {len_after:.0}"
        );
    }

    if let Some(report_path) = flag_value("--drc-report") {
        let report = freerouting::drc::check_board(&board);
        let json = report.to_kicad_json(&board, design);
        match std::fs::write(report_path, &json) {
            Ok(()) => println!(
                "DRC report written to {report_path}: {} violations, {} unconnected nets",
                report.violations.len(),
                report.unconnected.len()
            ),
            Err(e) => eprintln!("error: cannot write {report_path}: {e}"),
        }
    }
    if let Some(dsn_path) = flag_value("--export-dsn") {
        match freerouting::io::export_dsn(&board) {
            Some(text) => match std::fs::write(dsn_path, &text) {
                Ok(()) => println!("design written to {dsn_path} ({} bytes)", text.len()),
                Err(e) => eprintln!("error: cannot write {dsn_path}: {e}"),
            },
            None => eprintln!("error: no DSN source retained; cannot export"),
        }
    }
    if let Some(rules_out) = flag_value("--export-rules") {
        let text = freerouting::io::write_rules(&board, design);
        match std::fs::write(rules_out, &text) {
            Ok(()) => println!("rules written to {rules_out}"),
            Err(e) => eprintln!("error: cannot write {rules_out}: {e}"),
        }
    }
    if let Some(rn_path) = flag_value("--ratsnest") {
        let json = freerouting::ratsnest::ratsnest_json(&board);
        match std::fs::write(rn_path, &json) {
            Ok(()) => println!("ratsnest written to {rn_path}"),
            Err(e) => eprintln!("error: cannot write {rn_path}: {e}"),
        }
    }
    let design_name = std::path::Path::new(design)
        .file_stem()
        .and_then(|s| s.to_str())
        .unwrap_or(design);
    let ses = export_ses(&board, design_name, board.resolution);
    if let Err(e) = std::fs::write(&output, &ses) {
        eprintln!("error: cannot write {output}: {e}");
        return ExitCode::FAILURE;
    }
    println!("session written to {output} ({} bytes)", ses.len());
    if complete == net_count {
        ExitCode::SUCCESS
    } else {
        ExitCode::from(2)
    }
}
