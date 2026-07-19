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
use std::io::Write;
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

/// Writes a primary artifact without exposing a truncated destination. The
/// temporary file lives beside the target so the final rename is atomic on
/// the destination filesystem.
fn atomic_write(path: &str, contents: &[u8]) -> std::io::Result<()> {
    let destination = std::path::Path::new(path);
    let parent = destination
        .parent()
        .filter(|parent| !parent.as_os_str().is_empty())
        .unwrap_or_else(|| std::path::Path::new("."));
    let file_name = destination
        .file_name()
        .and_then(|name| name.to_str())
        .unwrap_or("freerouting-output");

    for attempt in 0..100u32 {
        let temporary = parent.join(format!(
            ".{file_name}.{}.{}.tmp",
            std::process::id(),
            attempt
        ));
        let mut file = match std::fs::OpenOptions::new()
            .write(true)
            .create_new(true)
            .open(&temporary)
        {
            Ok(file) => file,
            Err(error) if error.kind() == std::io::ErrorKind::AlreadyExists => continue,
            Err(error) => return Err(error),
        };
        let result = (|| {
            file.write_all(contents)?;
            file.sync_all()?;
            drop(file);
            std::fs::rename(&temporary, destination)
        })();
        if result.is_err() {
            let _ = std::fs::remove_file(&temporary);
        }
        return result;
    }
    Err(std::io::Error::new(
        std::io::ErrorKind::AlreadyExists,
        "could not allocate a temporary output file",
    ))
}

fn output_path_identity(path: &str) -> std::path::PathBuf {
    use std::ffi::OsString;
    use std::path::{Component, Path, PathBuf};

    enum UnresolvedComponent {
        CurDir,
        ParentDir,
        Normal(OsString),
    }

    fn apply_component(path: &mut PathBuf, component: &UnresolvedComponent) {
        match component {
            UnresolvedComponent::CurDir => {}
            UnresolvedComponent::ParentDir => {
                // An absolute path cannot escape its filesystem root.
                let _ = path.pop();
            }
            UnresolvedComponent::Normal(name) => path.push(name),
        }
    }

    let path = Path::new(path);
    if let Ok(canonical) = std::fs::canonicalize(path) {
        return canonical;
    }

    // Build an absolute path without normalizing it.  Filesystem traversal
    // resolves a symlink before applying a following `..`, so normalizing
    // first can compute the wrong destination for a not-yet-created output.
    let absolute = if path.is_absolute() {
        path.to_path_buf()
    } else {
        std::env::current_dir()
            .unwrap_or_else(|_| PathBuf::from("."))
            .join(path)
    };

    // Resolve symlinks and `..` in the longest canonicalizable prefix, then
    // replay the unresolved tail lexically.  This mirrors how the eventual
    // open/rename resolves paths while still identifying nonexistent files.
    let mut existing = absolute.clone();
    let mut tail = Vec::new();
    loop {
        if let Ok(mut identity) = std::fs::canonicalize(&existing) {
            for component in tail.iter().rev() {
                apply_component(&mut identity, component);
            }
            return identity;
        }

        match existing.components().next_back() {
            Some(Component::CurDir) => tail.push(UnresolvedComponent::CurDir),
            Some(Component::ParentDir) => tail.push(UnresolvedComponent::ParentDir),
            Some(Component::Normal(name)) => {
                tail.push(UnresolvedComponent::Normal(name.to_os_string()));
            }
            Some(Component::Prefix(_) | Component::RootDir) | None => break,
        }
        let _ = existing.pop();
    }

    // The current directory or filesystem root should normally provide a
    // canonical prefix.  Retain a deterministic lexical fallback for an
    // environment where even that lookup fails.
    let mut identity = PathBuf::new();
    for component in absolute.components() {
        match component {
            Component::CurDir => {}
            Component::ParentDir => {
                let _ = identity.pop();
            }
            other => identity.push(other.as_os_str()),
        }
    }
    identity
}

#[cfg(unix)]
fn existing_paths_alias(left: &str, right: &str) -> bool {
    use std::os::unix::fs::MetadataExt;

    let (Ok(left), Ok(right)) = (std::fs::metadata(left), std::fs::metadata(right)) else {
        return false;
    };
    left.dev() == right.dev() && left.ino() == right.ino()
}

#[cfg(windows)]
fn existing_paths_alias(left: &str, right: &str) -> bool {
    use std::os::windows::fs::MetadataExt;

    let (Ok(left), Ok(right)) = (std::fs::metadata(left), std::fs::metadata(right)) else {
        return false;
    };
    match (
        (left.volume_serial_number(), left.file_index()),
        (right.volume_serial_number(), right.file_index()),
    ) {
        ((Some(left_volume), Some(left_index)), (Some(right_volume), Some(right_index))) => {
            left_volume == right_volume && left_index == right_index
        }
        _ => false,
    }
}

#[cfg(not(any(unix, windows)))]
fn existing_paths_alias(_left: &str, _right: &str) -> bool {
    false
}

fn paths_alias(
    left_path: &str,
    left_identity: &std::path::Path,
    right_path: &str,
    right_identity: &std::path::Path,
) -> bool {
    left_identity == right_identity || existing_paths_alias(left_path, right_path)
}

/// The session is the mandatory artifact.  Fail before importing/routing when
/// its parent cannot exist, rather than discovering a missing directory only
/// after the expensive route has completed.  Optional artifact destinations
/// intentionally remain best-effort and are checked at their write point.
fn validate_primary_output_destination(path: &str) -> Result<(), String> {
    if path.is_empty() {
        return Err("mandatory session destination must not be empty".into());
    }
    let destination = std::path::Path::new(path);
    let parent = destination
        .parent()
        .filter(|parent| !parent.as_os_str().is_empty())
        .unwrap_or_else(|| std::path::Path::new("."));
    let metadata = std::fs::metadata(parent).map_err(|error| {
        format!("mandatory session parent directory {parent:?} is unavailable: {error}")
    })?;
    if !metadata.is_dir() {
        return Err(format!(
            "mandatory session parent path {parent:?} is not a directory"
        ));
    }
    if std::fs::metadata(destination).is_ok_and(|metadata| metadata.is_dir()) {
        return Err(format!(
            "mandatory session destination {destination:?} is a directory"
        ));
    }
    Ok(())
}

fn validate_distinct_output_paths(outputs: &[(&str, &str)]) -> Result<(), String> {
    let mut seen: Vec<(&str, &str, std::path::PathBuf)> = Vec::new();
    for &(flag, path) in outputs {
        let identity = output_path_identity(path);
        if let Some((previous, _, _)) = seen.iter().find(|(_, seen_path, seen_identity)| {
            paths_alias(path, &identity, seen_path, seen_identity)
        }) {
            return Err(format!(
                "{previous} and {flag} refer to the same output path {path:?}"
            ));
        }
        seen.push((flag, path, identity));
    }
    Ok(())
}

/// Output files must never alias an input file.  Besides losing the source
/// design, truncating `-de` before import can make a failed invocation look
/// like a successful empty-board run.  Compare canonical identities so
/// relative paths, `..`, and existing symlinks cannot bypass the guard.
fn validate_output_paths_against_inputs(
    outputs: &[(&str, &str)],
    inputs: &[(&str, &str)],
) -> Result<(), String> {
    validate_distinct_output_paths(outputs)?;
    let input_ids: Vec<(&str, &str, std::path::PathBuf)> = inputs
        .iter()
        .map(|&(flag, path)| (flag, path, output_path_identity(path)))
        .collect();
    for &(output_flag, output_path) in outputs {
        let output_id = output_path_identity(output_path);
        if let Some((input_flag, _, _)) = input_ids.iter().find(|(_, input_path, input_id)| {
            paths_alias(output_path, &output_id, input_path, input_id)
        }) {
            return Err(format!(
                "{output_flag} output {output_path:?} would overwrite {input_flag} input"
            ));
        }
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
    let mut output_paths = vec![("-do", output.as_str())];
    for flag in [
        "--drc-report",
        "--export-dsn",
        "--export-rules",
        "--ratsnest",
        "--export-json",
    ] {
        if let Some(path) = flag_value(flag) {
            output_paths.push((flag, path));
        }
    }
    let mut input_paths = vec![("-de", design)];
    for flag in ["--rules", "--profile", "--import-ses"] {
        if let Some(path) = flag_value(flag) {
            input_paths.push((flag, path));
        }
    }
    if let Err(error) = validate_output_paths_against_inputs(&output_paths, &input_paths) {
        eprintln!("error: {error}");
        return ExitCode::FAILURE;
    }
    if let Err(error) = validate_primary_output_destination(&output) {
        eprintln!("error: {error}");
        return ExitCode::FAILURE;
    }
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
    // `--angle` controls route generation, not the retained DSN's static rule
    // scopes. Keep the source value so DSN export can splice only wiring while
    // preserving the exact non-wiring semantics it was imported with.
    let source_angle_restriction = board.rules.get_trace_angle_restriction();
    let rules_requested = flag_value("--rules").is_some();
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

    // Validate a sidecar against the retained DSN while the imported angle is
    // still active.  A rules file that changes static DSN semantics (including
    // the angle rule itself) cannot be represented by the route-only DSN
    // exporter and must remain fail-closed.  This check is kept separate from
    // the runtime CLI angle below: a route-only `--angle` override is safe to
    // discard on the private export view, while an actual sidecar mutation is
    // not.
    let sidecar_dsn_preflight = if rules_requested && flag_value("--export-dsn").is_some() {
        freerouting::io::export_dsn(&board)
            .err()
            .map(|error| error.to_string())
    } else {
        None
    };

    // Apply the command-line angle after the sidecar.  This preserves the
    // normal CLI-over-file precedence without contaminating the semantic
    // comparison above.
    board.rules.set_trace_angle_restriction(angle_restriction);

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

    let design_name = std::path::Path::new(design)
        .file_stem()
        .and_then(|s| s.to_str())
        .unwrap_or(design);

    // Validate the mandatory format before entering the expensive routing
    // loop. This catches malformed/pre-routed state and identifier limits up
    // front; normal router output is required to remain representable by the
    // same checked writer.
    if let Err(error) = export_ses(&board, design_name, board.resolution) {
        eprintln!("error: cannot produce the required session output {output}: {error}");
        return ExitCode::FAILURE;
    }

    // The DSN writer deliberately reuses all non-wiring source scopes. Ignore
    // only the runtime route-angle override by restoring the imported value on
    // a private export view. Any sidecar mutation was checked before that
    // override was applied, so a no-op sidecar plus `--angle` remains
    // exportable while real static changes remain fail-closed.
    let dsn_export_preflight = flag_value("--export-dsn").and_then(|_| {
        sidecar_dsn_preflight.clone().or_else(|| {
            let mut export_view = board.clone();
            export_view
                .rules
                .set_trace_angle_restriction(source_angle_restriction);
            freerouting::io::export_dsn(&export_view)
                .err()
                .map(|error| error.to_string())
        })
    });
    if let Some(error) = &dsn_export_preflight {
        eprintln!(
            "error: --export-dsn is incompatible with the current static design state: {error}; \
             routing will continue so the required SES result can still be written"
        );
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

    // The SES is the primary result of a routing invocation. Serialize and
    // atomically publish it before attempting any optional artifact so a
    // secondary writer or path failure cannot erase minutes of routing work.
    let ses = match export_ses(&board, design_name, board.resolution) {
        Ok(ses) => ses,
        Err(e) => {
            eprintln!("error: cannot serialize {output}: {e}");
            return ExitCode::FAILURE;
        }
    };
    if let Err(e) = atomic_write(&output, ses.as_bytes()) {
        eprintln!("error: cannot write {output}: {e}");
        return ExitCode::FAILURE;
    }
    println!("session written to {output} ({} bytes)", ses.len());

    let mut optional_failed = false;
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
                optional_failed = true;
            }
        }
    }
    if let Some(dsn_path) = flag_value("--export-dsn") {
        if let Some(error) = &dsn_export_preflight {
            eprintln!("error: cannot serialize {dsn_path}: {error}");
            optional_failed = true;
        } else {
            let mut export_view = board.clone();
            export_view
                .rules
                .set_trace_angle_restriction(source_angle_restriction);
            let export_result = freerouting::io::export_dsn(&export_view);
            match export_result {
                Ok(text) => match std::fs::write(dsn_path, &text) {
                    Ok(()) => println!("design written to {dsn_path} ({} bytes)", text.len()),
                    Err(e) => {
                        eprintln!("error: cannot write {dsn_path}: {e}");
                        optional_failed = true;
                    }
                },
                Err(e) => {
                    eprintln!("error: cannot serialize {dsn_path}: {e}");
                    optional_failed = true;
                }
            }
        }
    }
    if let Some(rules_out) = flag_value("--export-rules") {
        match freerouting::io::write_rules(&board, design) {
            Ok(text) => match std::fs::write(rules_out, &text) {
                Ok(()) => println!("rules written to {rules_out}"),
                Err(e) => {
                    eprintln!("error: cannot write {rules_out}: {e}");
                    optional_failed = true;
                }
            },
            Err(e) => {
                eprintln!("error: cannot serialize {rules_out}: {e}");
                optional_failed = true;
            }
        }
    }
    if let Some(rn_path) = flag_value("--ratsnest") {
        let json = freerouting::ratsnest::ratsnest_json(&board);
        match std::fs::write(rn_path, &json) {
            Ok(()) => println!("ratsnest written to {rn_path}"),
            Err(e) => {
                eprintln!("error: cannot write {rn_path}: {e}");
                optional_failed = true;
            }
        }
    }
    if let Some(json_path) = flag_value("--export-json") {
        match freerouting::io::export_kicad_json_checked(&board) {
            Ok(text) => match std::fs::write(json_path, &text) {
                Ok(()) => println!("board JSON written to {json_path} ({} bytes)", text.len()),
                Err(e) => {
                    eprintln!("error: cannot write {json_path}: {e}");
                    optional_failed = true;
                }
            },
            Err(e) => {
                eprintln!("error: cannot serialize {json_path}: {e}");
                optional_failed = true;
            }
        }
    }
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
    if optional_failed {
        ExitCode::FAILURE
    } else if complete_final != net_count {
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

    #[test]
    fn primary_output_is_replaced_atomically_without_temp_files() {
        let path = std::env::temp_dir().join(format!(
            "freerouting-atomic-output-{}.ses",
            std::process::id()
        ));
        std::fs::write(&path, b"old").expect("seed destination");
        atomic_write(path.to_str().unwrap(), b"complete session").expect("atomic write");
        assert_eq!(std::fs::read(&path).unwrap(), b"complete session");
        let file_name = path.file_name().unwrap().to_string_lossy();
        let parent = path.parent().unwrap();
        assert!(
            std::fs::read_dir(parent)
                .unwrap()
                .filter_map(Result::ok)
                .all(|entry| !entry
                    .file_name()
                    .to_string_lossy()
                    .starts_with(&format!(".{file_name}.{}.", std::process::id()))),
            "temporary output must not remain after the rename"
        );
        let _ = std::fs::remove_file(path);
    }

    #[test]
    fn outputs_cannot_alias_design_or_sidecar_inputs() {
        let root =
            std::env::temp_dir().join(format!("freerouting-cli-path-guard-{}", std::process::id()));
        std::fs::create_dir_all(&root).expect("create temp directory");
        let design = root.join("board.dsn");
        let rules = root.join("board.rules");
        std::fs::write(&design, "source").expect("write design");
        std::fs::write(&rules, "source").expect("write rules");

        let outputs = [("-do", design.to_str().unwrap())];
        let inputs = [
            ("-de", design.to_str().unwrap()),
            ("--rules", rules.to_str().unwrap()),
        ];
        let error = validate_output_paths_against_inputs(&outputs, &inputs)
            .expect_err("input/output alias must be rejected");
        assert!(error.contains("-de input"));

        let outputs = [("--export-json", rules.to_str().unwrap())];
        let error = validate_output_paths_against_inputs(&outputs, &inputs)
            .expect_err("sidecar/output alias must be rejected");
        assert!(error.contains("--rules input"));

        // A not-yet-created output spelling can normalize to an existing
        // input through a lexical `..` component. Reject the alias
        // conservatively before any writer or later directory creation can
        // turn that spelling into a destructive destination.
        let nested_alias = root.join("missing").join("..").join("board.dsn");
        let outputs = [("-do", nested_alias.to_str().unwrap())];
        let inputs = [("-de", design.to_str().unwrap())];
        let error = validate_output_paths_against_inputs(&outputs, &inputs)
            .expect_err("lexical alias must be rejected");
        assert!(error.contains("-de input"));

        #[cfg(unix)]
        {
            let hard_link = root.join("hard-linked-output.json");
            std::fs::hard_link(&design, &hard_link).expect("create hard link to design input");
            let outputs = [("--export-json", hard_link.to_str().unwrap())];
            let inputs = [("-de", design.to_str().unwrap())];
            let error = validate_output_paths_against_inputs(&outputs, &inputs)
                .expect_err("hard-link input/output alias must be rejected");
            assert!(error.contains("-de input"));
        }
        let _ = std::fs::remove_dir_all(root);
    }

    #[test]
    fn mandatory_output_destination_is_checked_before_routing() {
        let root = std::env::temp_dir().join(format!(
            "freerouting-cli-destination-guard-{}",
            std::process::id()
        ));
        std::fs::create_dir_all(&root).expect("create temp directory");
        let missing_parent = root.join("missing").join("result.ses");
        let error = validate_primary_output_destination(missing_parent.to_str().unwrap())
            .expect_err("missing mandatory output parent must fail early");
        assert!(error.contains("parent directory"));

        let directory_destination = root.join("result.ses");
        std::fs::create_dir(&directory_destination).expect("create directory destination");
        let error = validate_primary_output_destination(directory_destination.to_str().unwrap())
            .expect_err("directory-valued mandatory output must fail early");
        assert!(error.contains("is a directory"));

        let error = validate_primary_output_destination("")
            .expect_err("empty mandatory output must fail early");
        assert!(error.contains("must not be empty"));

        let _ = std::fs::remove_dir_all(root);
    }

    #[cfg(unix)]
    #[test]
    fn outputs_cannot_alias_through_symlink_followed_by_parent_component() {
        use std::os::unix::fs::symlink;

        let root = std::env::temp_dir().join(format!(
            "freerouting-cli-symlink-path-guard-{}",
            std::process::id()
        ));
        let real = root.join("real");
        let nested = real.join("nested");
        std::fs::create_dir_all(&nested).expect("create real output directory");
        let link = root.join("link");
        symlink(&nested, &link).expect("create directory symlink");

        // Neither final output exists.  The kernel resolves `link` to
        // `real/nested` before applying `..`, so both spellings target the
        // same future file under `real`.
        let via_link = link.join("..").join("same-output.ses");
        let direct = real.join("same-output.ses");
        let outputs = [
            ("-do", via_link.to_str().unwrap()),
            ("--export-json", direct.to_str().unwrap()),
        ];
        let error = validate_distinct_output_paths(&outputs)
            .expect_err("symlink/parent output alias must be rejected");
        assert!(error.contains("same output path"));

        let _ = std::fs::remove_dir_all(root);
    }
}
