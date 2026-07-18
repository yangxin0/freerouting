//! Command line interface, compatible with the basic flags of the Java
//! jar (`java -jar freerouting.jar -de input.dsn -do output.ses`).
//!
//! The GUI of the Java original is deliberately not ported.

use freerouting::autoroute::{
    batch_route_passes_with_time_limit, pull_tight_all, total_trace_length, BatchRequest,
};
use freerouting::datastructures::TimeLimit;
use freerouting::io::json::Json;
use freerouting::io::{export_ses, import_dsn};
use std::process::ExitCode;
use std::time::Instant;

const USAGE: &str = "\
freerouting (Rust) — PCB auto-router

Usage: freerouting -de <input.dsn> [options]

Options:
  -de <file.dsn>     design file to route (required)
  -do <file.ses>     session output file (default: input file with .ses)
  -mp <n>            maximum number of ripup passes (default 9999; 0 = no limit)
  -tl <seconds>      wall-clock budget shared by routing + optimization (default 300)
  --strip-wiring     remove the pre-routed wiring and route from scratch
  --fanout           fan out SMD pins to vias before routing
  --angle <mode>     trace angle restriction: none, 45, 90 (default 45)
  --drc-report <f>   write a KiCad-format DRC report (JSON) after routing
  --export-dsn <f>   write the routed design as a Specctra DSN file
  --import-ses <f>   apply an existing session file before routing
  --threads <n>      optimizer worker threads (default: CPU cores - 1)
  --rules <f>        apply a .rules file before routing
  --export-rules <f> write the design rules to a .rules file
  --api-server <p>   run the REST API server on port <p> (no routing)
  --ratsnest <f>     write the unconnected airlines as JSON after routing
  --export-json <f>  write the routed board as KiCad board JSON
  --profile <f>      apply a JSON router-settings profile (maxPasses,
                     viaCosts, timeLimitSeconds, angleRestriction, threads)
  -h, --help         show this help";

#[derive(Default)]
struct RouterProfile {
    max_passes: Option<usize>,
    via_costs: Option<f64>,
    time_limit_seconds: Option<u64>,
    angle_restriction: Option<String>,
    threads: Option<usize>,
}

fn profile_exact_u64(profile: &Json, key: &str) -> Result<Option<u64>, String> {
    let Some(value) = profile.get(key) else {
        return Ok(None);
    };
    if matches!(value, Json::Null) {
        return Ok(None);
    }
    value
        .as_exact_u64()
        .map(Some)
        .ok_or_else(|| format!("profile field {key} must be a non-negative integer"))
}

fn load_profile(path: &str) -> Result<RouterProfile, String> {
    let text =
        std::fs::read_to_string(path).map_err(|e| format!("cannot read profile {path}: {e}"))?;
    let profile = freerouting::io::json::parse_json(&text)
        .map_err(|e| format!("cannot parse profile {path}: {e}"))?;
    if !matches!(profile, Json::Obj(_)) {
        return Err(format!("profile {path} must contain a JSON object"));
    }

    let max_passes = profile_exact_u64(&profile, "maxPasses")?
        .map(|value| {
            usize::try_from(value)
                .map_err(|_| "profile field maxPasses is out of range".to_string())
        })
        .transpose()?;
    let time_limit_seconds = profile_exact_u64(&profile, "timeLimitSeconds")?;
    let threads = profile_exact_u64(&profile, "threads")?
        .map(|value| {
            usize::try_from(value).map_err(|_| "profile field threads is out of range".to_string())
        })
        .transpose()?;
    if threads == Some(0) {
        return Err("profile field threads must be at least 1".to_string());
    }

    let via_costs = match profile.get("viaCosts") {
        None | Some(Json::Null) => None,
        Some(value) => {
            let costs = value
                .as_f64()
                .ok_or_else(|| "profile field viaCosts must be a number".to_string())?;
            if costs <= 0.0 {
                return Err("profile field viaCosts must be greater than 0".to_string());
            }
            Some(costs)
        }
    };
    let angle_restriction = match profile.get("angleRestriction") {
        None | Some(Json::Null) => None,
        Some(value) => {
            let angle = value
                .as_str()
                .ok_or_else(|| "profile field angleRestriction must be a string".to_string())?;
            parse_angle_restriction(angle)
                .map_err(|e| format!("profile field angleRestriction: {e}"))?;
            Some(angle.to_string())
        }
    };

    Ok(RouterProfile {
        max_passes,
        via_costs,
        time_limit_seconds,
        angle_restriction,
        threads,
    })
}

fn parse_angle_restriction(value: &str) -> Result<freerouting::board::AngleRestriction, String> {
    match value {
        "none" | "any" => Ok(freerouting::board::AngleRestriction::None),
        "45" => Ok(freerouting::board::AngleRestriction::FortyfiveDegree),
        "90" => Ok(freerouting::board::AngleRestriction::NinetyDegree),
        _ => Err(format!(
            "invalid angle restriction {value:?}; expected none, 45, or 90"
        )),
    }
}

fn parse_usize_flag(value: Option<&str>, flag: &str) -> Result<Option<usize>, String> {
    value
        .map(|raw| {
            raw.parse::<usize>()
                .map_err(|_| format!("{flag} requires a non-negative integer, got {raw:?}"))
        })
        .transpose()
}

fn parse_u64_flag(value: Option<&str>, flag: &str) -> Result<Option<u64>, String> {
    value
        .map(|raw| {
            raw.parse::<u64>()
                .map_err(|_| format!("{flag} requires a non-negative integer, got {raw:?}"))
        })
        .transpose()
}

fn validate_argument_shape(args: &[String]) -> Result<(), String> {
    const VALUE_FLAGS: &[&str] = &[
        "-de",
        "-do",
        "-mp",
        "-tl",
        "--angle",
        "--drc-report",
        "--export-dsn",
        "--import-ses",
        "--threads",
        "--rules",
        "--export-rules",
        "--api-server",
        "--ratsnest",
        "--export-json",
        "--profile",
        "--opt-strategy",
        "--opt-selection",
        "--hybrid-ratio",
    ];
    const SWITCH_FLAGS: &[&str] = &["--strip-wiring", "--fanout", "-h", "--help"];

    let mut seen = std::collections::HashSet::new();
    let mut index = 0;
    while index < args.len() {
        let flag = args[index].as_str();
        if !seen.insert(flag) {
            return Err(format!("{flag} was specified more than once"));
        }
        if SWITCH_FLAGS.contains(&flag) {
            index += 1;
            continue;
        }
        if VALUE_FLAGS.contains(&flag) {
            let Some(value) = args.get(index + 1) else {
                return Err(format!("{flag} requires a value"));
            };
            if VALUE_FLAGS.contains(&value.as_str()) || SWITCH_FLAGS.contains(&value.as_str()) {
                return Err(format!("{flag} requires a value"));
            }
            index += 2;
            continue;
        }
        return Err(format!("unknown argument {flag:?}"));
    }
    Ok(())
}

fn main() -> ExitCode {
    let args: Vec<String> = std::env::args().skip(1).collect();
    if args.iter().any(|a| a == "-h" || a == "--help") || args.is_empty() {
        println!("{USAGE}");
        return ExitCode::SUCCESS;
    }
    if let Err(e) = validate_argument_shape(&args) {
        eprintln!("error: {e}");
        return ExitCode::FAILURE;
    }
    let flag_value = |flag: &str| -> Option<&str> {
        args.iter()
            .position(|a| a == flag)
            .and_then(|i| args.get(i + 1))
            .map(|s| s.as_str())
    };
    if let Some(raw_port) = flag_value("--api-server") {
        let port = match raw_port.parse::<u16>() {
            Ok(port) => port,
            Err(_) => {
                eprintln!("error: --api-server requires a port in 0..=65535, got {raw_port:?}");
                return ExitCode::FAILURE;
            }
        };
        let seconds = match parse_u64_flag(flag_value("-tl"), "-tl") {
            Ok(seconds) => seconds.unwrap_or(300),
            Err(e) => {
                eprintln!("error: {e}");
                return ExitCode::FAILURE;
            }
        };
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
    let output = flag_value("-do").map(str::to_string).unwrap_or_else(|| {
        let stem = design.strip_suffix(".dsn").unwrap_or(design);
        format!("{stem}.ses")
    });
    // like the Java jar, passes are effectively unlimited by default and
    // the wall clock (-tl) is the real bound; a JSON profile (Java:
    // RouterSettings) provides defaults that explicit flags override
    let profile = match flag_value("--profile") {
        Some(path) => match load_profile(path) {
            Ok(profile) => profile,
            Err(e) => {
                eprintln!("error: {e}");
                return ExitCode::FAILURE;
            }
        },
        None => RouterProfile::default(),
    };
    // Java `RouterSettings`: default maxPasses is 9999 and `-mp 0` means
    // "no limit" (mapped to Integer.MAX_VALUE), with the wall clock as the
    // real bound. Previously `-mp 0` collapsed to a single pass.
    let max_passes: usize = {
        let cli_value = match parse_usize_flag(flag_value("-mp"), "-mp") {
            Ok(value) => value,
            Err(e) => {
                eprintln!("error: {e}");
                return ExitCode::FAILURE;
            }
        };
        let raw = cli_value.or(profile.max_passes).unwrap_or(9999);
        if raw == 0 {
            usize::MAX
        } else {
            raw
        }
    };
    let limit_s = match parse_u64_flag(flag_value("-tl"), "-tl") {
        Ok(value) => value.or(profile.time_limit_seconds).unwrap_or(300),
        Err(e) => {
            eprintln!("error: {e}");
            return ExitCode::FAILURE;
        }
    };
    let strip_wiring = args.iter().any(|a| a == "--strip-wiring");
    let angle_mode = flag_value("--angle")
        .map(str::to_string)
        .or(profile.angle_restriction)
        .unwrap_or_else(|| "45".to_string());
    let angle_restriction = match parse_angle_restriction(&angle_mode) {
        Ok(value) => value,
        Err(e) => {
            eprintln!("error: {e}");
            return ExitCode::FAILURE;
        }
    };
    let profile_via_costs = profile.via_costs;
    let profile_threads = profile.threads;
    // Validate every optimizer option before importing or routing. An invalid
    // explicit setting must not be discovered only after an expensive routing
    // pass has already changed the board.
    let default_threads = std::thread::available_parallelism()
        .map(|n| n.get().saturating_sub(1).max(1))
        .unwrap_or(1);
    let opt_threads = match parse_usize_flag(flag_value("--threads"), "--threads") {
        Ok(value) => value.or(profile_threads).unwrap_or(default_threads),
        Err(e) => {
            eprintln!("error: {e}");
            return ExitCode::FAILURE;
        }
    };
    if opt_threads == 0 {
        eprintln!("error: --threads must be at least 1");
        return ExitCode::FAILURE;
    }
    let strategy = match flag_value("--opt-strategy") {
        None | Some("greedy") => freerouting::autoroute::BoardUpdateStrategy::Greedy,
        Some("global") => freerouting::autoroute::BoardUpdateStrategy::GlobalOptimal,
        Some("hybrid") => freerouting::autoroute::BoardUpdateStrategy::Hybrid,
        Some(value) => {
            eprintln!(
                "error: invalid --opt-strategy {value:?}; expected greedy, global, or hybrid"
            );
            return ExitCode::FAILURE;
        }
    };
    let selection = match flag_value("--opt-selection") {
        None | Some("prioritized") => freerouting::autoroute::ItemSelectionStrategy::Prioritized,
        Some("sequential") => freerouting::autoroute::ItemSelectionStrategy::Sequential,
        Some("random") => freerouting::autoroute::ItemSelectionStrategy::Random,
        Some(value) => {
            eprintln!(
                "error: invalid --opt-selection {value:?}; expected prioritized, sequential, or random"
            );
            return ExitCode::FAILURE;
        }
    };
    let hybrid_ratio = match flag_value("--hybrid-ratio") {
        None => (1, 1),
        Some(value) => {
            let parsed = value.split_once(':').and_then(|(optimal, greedy)| {
                Some((
                    optimal.parse::<usize>().ok()?,
                    greedy.parse::<usize>().ok()?,
                ))
            });
            match parsed {
                Some((optimal, greedy)) if optimal > 0 && greedy > 0 => (optimal, greedy),
                _ => {
                    eprintln!(
                        "error: --hybrid-ratio requires two positive integers as N:M, got {value:?}"
                    );
                    return ExitCode::FAILURE;
                }
            }
        }
    };

    let mut content = match std::fs::read_to_string(design) {
        Ok(c) => c,
        Err(e) => {
            eprintln!("error: cannot read {design}: {e}");
            return ExitCode::FAILURE;
        }
    };
    if strip_wiring {
        // the importer's balanced, case-insensitive strip — the former
        // exact substring truncation missed uppercase (WIRING ...) and
        // dropped everything after the section
        content = freerouting::io::strip_wiring(&content);
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
    board.rules.set_trace_angle_restriction(angle_restriction);
    println!(
        "imported {design} in {:?}: {} layers, {} nets, {} items",
        t0.elapsed(),
        board.layer_structure.layer_count(),
        board.rules.nets.max_net_no(),
        board.item_count()
    );

    // an explicitly requested input that cannot be applied is a failure,
    // not a log line: exiting successfully after ignoring --rules or
    // --import-ses would silently route with the wrong constraints/wiring
    if let Some(rules_path) = flag_value("--rules") {
        match std::fs::read_to_string(rules_path) {
            Ok(text) => match freerouting::io::read_rules(&mut board, &text) {
                Ok(n) => println!("rules applied: {n} settings"),
                Err(e) => {
                    eprintln!("error: {e}");
                    return ExitCode::FAILURE;
                }
            },
            Err(e) => {
                eprintln!("error: cannot read {rules_path}: {e}");
                return ExitCode::FAILURE;
            }
        }
    }
    if let Some(ses_path) = flag_value("--import-ses") {
        match std::fs::read_to_string(ses_path) {
            Ok(text) => match freerouting::io::import_ses(&mut board, &text) {
                Ok(s) => {
                    println!("session applied: {} wires, {} vias", s.wires, s.vias);
                    if !s.unknown_nets.is_empty() {
                        eprintln!(
                            "error: session names {} net(s) unknown to the design: {:?}",
                            s.unknown_nets.len(),
                            s.unknown_nets
                        );
                        return ExitCode::FAILURE;
                    }
                }
                Err(e) => {
                    eprintln!("error: {e}");
                    return ExitCode::FAILURE;
                }
            },
            Err(e) => {
                eprintln!("error: cannot read {ses_path}: {e}");
                return ExitCode::FAILURE;
            }
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
        via_clearance_class: 0,
        via_attach_allowed: false,
        via_cost: profile_via_costs.unwrap_or(50_000.0),
        max_expansions: 100_000,
        ripup_penalty: 0.0,
        deadline: None,
    };

    let t1 = Instant::now();
    let time_limit = TimeLimit::new(limit_s.saturating_mul(1000));
    if args.iter().any(|a| a == "--fanout") {
        let fanned =
            freerouting::autoroute::fanout_board(&mut board, &request, 20, Some(&time_limit));
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
    let score = stats.normalized_score(&freerouting::scoring::ScoringSettings::default());
    println!(
        "score: {score:.2} ({} unrouted, {} violations, {} vias, {:.1} mm)",
        stats.incomplete_count, stats.clearance_violations, stats.via_count, stats.total_length_mm
    );

    // Java uses a single job-wide time budget shared by routing and
    // optimization; reusing the same `time_limit` (which counts from its
    // creation) gives the optimizer whatever remains of `-tl` after routing,
    // so total wall time stays bounded by `-tl` instead of stacking a second
    // budget on top.
    // Java defaults: GREEDY board updates with PRIORITIZED selection.
    let improved = if std::env::var_os("FR_NO_OPT").is_some() {
        0
    } else {
        freerouting::autoroute::optimize_route_multithreaded_with_strategy(
            &mut board,
            &request,
            opt_threads,
            Some(&time_limit),
            strategy,
            selection,
            hybrid_ratio,
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
        println!("pull tight: {removed} corners removed, length {len_before:.0} -> {len_after:.0}");
    }
    if improved > 0 || removed > 0 || combined > 0 {
        let stats = freerouting::scoring::BoardStatistics::collect(&board);
        let score = stats.normalized_score(&freerouting::scoring::ScoringSettings::default());
        println!(
            "final score: {score:.2} ({} unrouted, {} violations, {} vias, {:.1} mm)",
            stats.incomplete_count,
            stats.clearance_violations,
            stats.via_count,
            stats.total_length_mm
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
            Err(e) => {
                eprintln!("error: cannot write {report_path}: {e}");
                return ExitCode::FAILURE;
            }
        }
    }
    if let Some(dsn_path) = flag_value("--export-dsn") {
        match freerouting::io::export_dsn(&board) {
            Ok(text) => match std::fs::write(dsn_path, &text) {
                Ok(()) => println!("design written to {dsn_path} ({} bytes)", text.len()),
                Err(e) => {
                    eprintln!("error: cannot write {dsn_path}: {e}");
                    return ExitCode::FAILURE;
                }
            },
            Err(e) => {
                eprintln!("error: cannot serialize {dsn_path}: {e}");
                return ExitCode::FAILURE;
            }
        }
    }
    if let Some(rules_out) = flag_value("--export-rules") {
        let text = match freerouting::io::write_rules(&board, design) {
            Ok(text) => text,
            Err(e) => {
                eprintln!("error: cannot serialize {rules_out}: {e}");
                return ExitCode::FAILURE;
            }
        };
        match std::fs::write(rules_out, &text) {
            Ok(()) => println!("rules written to {rules_out}"),
            Err(e) => {
                eprintln!("error: cannot write {rules_out}: {e}");
                return ExitCode::FAILURE;
            }
        }
    }
    if let Some(rn_path) = flag_value("--ratsnest") {
        let json = freerouting::ratsnest::ratsnest_json(&board);
        match std::fs::write(rn_path, &json) {
            Ok(()) => println!("ratsnest written to {rn_path}"),
            Err(e) => {
                eprintln!("error: cannot write {rn_path}: {e}");
                return ExitCode::FAILURE;
            }
        }
    }
    if let Some(json_path) = flag_value("--export-json") {
        let text = match freerouting::io::export_kicad_json_checked(&board) {
            Ok(text) => text,
            Err(e) => {
                eprintln!("error: cannot serialize {json_path}: {e}");
                return ExitCode::FAILURE;
            }
        };
        match std::fs::write(json_path, &text) {
            Ok(()) => println!("board JSON written to {json_path} ({} bytes)", text.len()),
            Err(e) => {
                eprintln!("error: cannot write {json_path}: {e}");
                return ExitCode::FAILURE;
            }
        }
    }
    let design_name = std::path::Path::new(design)
        .file_stem()
        .and_then(|s| s.to_str())
        .unwrap_or(design);
    let ses = match export_ses(&board, design_name, board.resolution) {
        Ok(ses) => ses,
        Err(e) => {
            eprintln!("error: cannot serialize {output}: {e}");
            return ExitCode::FAILURE;
        }
    };
    if let Err(e) = std::fs::write(&output, &ses) {
        eprintln!("error: cannot write {output}: {e}");
        return ExitCode::FAILURE;
    }
    println!("session written to {output} ({} bytes)", ses.len());
    // Recompute completion after optimization/normalization: the optimizer's
    // recovery reroutes can finish nets that were incomplete right after the
    // routing pass, and the exit status must reflect the board that was
    // actually written, not the pre-optimization snapshot.
    let complete_final = (1..=net_count)
        .filter(|&n| board.net_is_completely_connected(n as i32))
        .count();
    if complete_final != complete {
        println!("final completion: {complete_final}/{net_count} nets connected");
    }
    // Success requires a DRC-clean result, not just connectivity: a fully
    // connected board with clearance violations is not a usable route.
    // Exit 2 = nets unconnected, exit 3 = connected but violating.
    let violations = freerouting::drc::check_board(&board).violations.len();
    if violations > 0 {
        println!("DRC: {violations} clearance violation(s) remain");
    }
    if complete_final != net_count {
        ExitCode::from(2)
    } else if violations > 0 {
        ExitCode::from(3)
    } else {
        ExitCode::SUCCESS
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn strings(values: &[&str]) -> Vec<String> {
        values.iter().map(|value| value.to_string()).collect()
    }

    #[test]
    fn cli_shape_rejects_unknown_duplicate_and_missing_arguments() {
        assert!(
            validate_argument_shape(&strings(&["-de", "board.dsn", "--fanout", "-mp", "0",]))
                .is_ok()
        );

        for args in [
            strings(&["-de"]),
            strings(&["-de", "--fanout"]),
            strings(&["-de", "a.dsn", "--fanot"]),
            strings(&["-de", "a.dsn", "-de", "b.dsn"]),
            strings(&["a.dsn"]),
        ] {
            assert!(
                validate_argument_shape(&args).is_err(),
                "accepted malformed argument list {args:?}"
            );
        }
    }

    #[test]
    fn numeric_flags_and_angles_reject_lossy_or_unknown_values() {
        assert_eq!(parse_usize_flag(Some("0"), "-mp").unwrap(), Some(0));
        assert_eq!(parse_u64_flag(Some("300"), "-tl").unwrap(), Some(300));
        for value in ["-1", "1.5", "NaN", "999999999999999999999999999"] {
            assert!(parse_usize_flag(Some(value), "-mp").is_err());
            assert!(parse_u64_flag(Some(value), "-tl").is_err());
        }
        for value in ["none", "any", "45", "90"] {
            assert!(parse_angle_restriction(value).is_ok());
        }
        assert!(parse_angle_restriction("forty-five").is_err());
    }

    #[test]
    fn profile_fields_are_checked_instead_of_silently_defaulted() {
        let path = std::env::temp_dir().join(format!(
            "freerouting-cli-profile-validation-{}.json",
            std::process::id()
        ));
        let write = |text: &str| std::fs::write(&path, text).expect("write profile");

        write(
            r#"{
                "maxPasses": 0,
                "viaCosts": 12.5,
                "timeLimitSeconds": 30,
                "angleRestriction": "90",
                "threads": 2,
                "futureJavaSetting": true
            }"#,
        );
        let profile = load_profile(path.to_str().unwrap()).expect("valid profile");
        assert_eq!(profile.max_passes, Some(0));
        assert_eq!(profile.via_costs, Some(12.5));
        assert_eq!(profile.time_limit_seconds, Some(30));
        assert_eq!(profile.angle_restriction.as_deref(), Some("90"));
        assert_eq!(profile.threads, Some(2));

        for invalid in [
            "[]",
            r#"{"maxPasses": 1.5}"#,
            r#"{"maxPasses": -1}"#,
            r#"{"maxPasses": "2"}"#,
            r#"{"timeLimitSeconds": 1e2}"#,
            r#"{"threads": 0}"#,
            r#"{"threads": 2.0}"#,
            r#"{"viaCosts": 0}"#,
            r#"{"viaCosts": "50"}"#,
            r#"{"angleRestriction": 45}"#,
            r#"{"angleRestriction": "diagonal"}"#,
            "{",
        ] {
            write(invalid);
            assert!(
                load_profile(path.to_str().unwrap()).is_err(),
                "accepted invalid profile {invalid}"
            );
        }
        let _ = std::fs::remove_file(path);
    }
}
