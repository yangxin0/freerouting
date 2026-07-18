//! Port of the `api/` package's core: the v1 routing-job REST API
//! (Java: `JobControllerV1` + `SystemControllerV1`) on a minimal
//! dependency-free HTTP/1.1 server, together with the `core` job model
//! (`RoutingJob`/`RoutingJobState`).
//!
//! Endpoints (mirroring Java's paths):
//! - `POST /v1/jobs/enqueue`        → create a job, returns its id
//! - `POST /v1/jobs/{id}/input`     → upload the DSN payload
//! - `PUT  /v1/jobs/{id}/start`     → start routing (background thread)
//! - `PUT  /v1/jobs/{id}/cancel`    → request cancellation
//! - `GET  /v1/jobs/{id}`           → job state + score JSON
//! - `GET  /v1/jobs/{id}/output`    → the session file when completed
//! - `GET  /v1/system/status`       → health/version

use crate::autoroute::{batch_route_passes_with_time_limit, BatchRequest};
use crate::datastructures::TimeLimit;
use crate::io::{export_ses, import_dsn};
use std::collections::HashMap;
use std::io::{Read, Write};
use std::net::TcpListener;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::{Arc, Mutex};

/// Java: `RoutingJobState`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum JobState {
    Queued,
    ReadyToStart,
    Running,
    Completed,
    Cancelled,
    Failed,
}

impl JobState {
    fn as_str(self) -> &'static str {
        match self {
            JobState::Queued => "QUEUED",
            JobState::ReadyToStart => "READY_TO_START",
            JobState::Running => "RUNNING",
            JobState::Completed => "COMPLETED",
            JobState::Cancelled => "CANCELLED",
            JobState::Failed => "FAILED",
        }
    }
}

/// Java: `RoutingJob` (the non-GUI essentials).
pub struct RoutingJob {
    pub id: u64,
    pub state: JobState,
    pub input_dsn: Option<String>,
    pub output_ses: Option<String>,
    pub score: Option<f64>,
    pub error: Option<String>,
    pub cancel: Arc<AtomicBool>,
}

type Jobs = Arc<Mutex<HashMap<u64, RoutingJob>>>;

static NEXT_JOB_ID: AtomicU64 = AtomicU64::new(1);

/// Runs the API server on `port`, blocking (Java: the embedded HTTP
/// server of `FreeroutingApplication` in API mode).
pub fn serve(port: u16, route_seconds: u64) -> std::io::Result<()> {
    let listener = TcpListener::bind(("127.0.0.1", port))?;
    println!("API server listening on http://127.0.0.1:{port}");
    let jobs: Jobs = Arc::new(Mutex::new(HashMap::new()));
    for stream in listener.incoming() {
        let Ok(stream) = stream else { continue };
        let jobs = jobs.clone();
        std::thread::spawn(move || {
            let _ = handle(stream, jobs, route_seconds);
        });
    }
    Ok(())
}

fn handle<S: Read + Write>(mut stream: S, jobs: Jobs, route_seconds: u64) -> std::io::Result<()> {
    const MAX_HEADER_BYTES: usize = 64 * 1024;
    const MAX_BODY_BYTES: usize = 64 * 1024 * 1024;
    let mut buf = Vec::new();
    let mut tmp = [0u8; 4096];
    // read until headers complete, then body per content-length
    let (mut method, mut path, mut body) = (String::new(), String::new(), Vec::new());
    loop {
        let n = stream.read(&mut tmp)?;
        if n == 0 {
            if find_headers_end(&buf).is_none() {
                write_http_response(
                    &mut stream,
                    "400 Bad Request",
                    "application/json",
                    "{\"error\": \"incomplete HTTP headers\"}",
                )?;
                return Ok(());
            }
            break;
        }
        buf.extend_from_slice(&tmp[..n]);
        if find_headers_end(&buf).is_none() && buf.len() > MAX_HEADER_BYTES {
            write_http_response(
                &mut stream,
                "431 Request Header Fields Too Large",
                "application/json",
                "{\"error\": \"request headers too large\"}",
            )?;
            return Ok(());
        }
        if buf.len() > MAX_HEADER_BYTES + MAX_BODY_BYTES {
            write_http_response(
                &mut stream,
                "413 Payload Too Large",
                "application/json",
                "{\"error\": \"request too large\"}",
            )?;
            return Ok(());
        }
        if let Some(header_end) = find_headers_end(&buf) {
            if header_end > MAX_HEADER_BYTES {
                write_http_response(
                    &mut stream,
                    "431 Request Header Fields Too Large",
                    "application/json",
                    "{\"error\": \"request headers too large\"}",
                )?;
                return Ok(());
            }
            let headers = match std::str::from_utf8(&buf[..header_end]) {
                Ok(headers) => headers,
                Err(_) => {
                    write_http_response(
                        &mut stream,
                        "400 Bad Request",
                        "application/json",
                        "{\"error\": \"headers must be valid UTF-8\"}",
                    )?;
                    return Ok(());
                }
            };
            let mut lines = headers.lines();
            let Some(request_line) = lines.next() else {
                write_http_response(
                    &mut stream,
                    "400 Bad Request",
                    "application/json",
                    "{\"error\": \"missing request line\"}",
                )?;
                return Ok(());
            };
            let mut parts = request_line.split_whitespace();
            method = parts.next().unwrap_or("").to_string();
            path = parts.next().unwrap_or("").to_string();
            let version = parts.next().unwrap_or("");
            if method.is_empty()
                || path.is_empty()
                || !matches!(version, "HTTP/1.0" | "HTTP/1.1")
                || parts.next().is_some()
            {
                write_http_response(
                    &mut stream,
                    "400 Bad Request",
                    "application/json",
                    "{\"error\": \"malformed request line\"}",
                )?;
                return Ok(());
            }
            let mut content_length = None;
            let mut transfer_encoding = None;
            for line in lines {
                if line.is_empty() {
                    continue;
                }
                let Some((key, value)) = line.split_once(':') else {
                    write_http_response(
                        &mut stream,
                        "400 Bad Request",
                        "application/json",
                        "{\"error\": \"malformed header\"}",
                    )?;
                    return Ok(());
                };
                if key.eq_ignore_ascii_case("content-length") {
                    if content_length.is_some() {
                        write_http_response(
                            &mut stream,
                            "400 Bad Request",
                            "application/json",
                            "{\"error\": \"duplicate Content-Length\"}",
                        )?;
                        return Ok(());
                    }
                    let value = value.trim();
                    let Ok(parsed) = value.parse::<usize>() else {
                        write_http_response(
                            &mut stream,
                            "400 Bad Request",
                            "application/json",
                            "{\"error\": \"invalid Content-Length\"}",
                        )?;
                        return Ok(());
                    };
                    if parsed > MAX_BODY_BYTES {
                        write_http_response(
                            &mut stream,
                            "413 Payload Too Large",
                            "application/json",
                            "{\"error\": \"request body too large\"}",
                        )?;
                        return Ok(());
                    }
                    content_length = Some(parsed);
                } else if key.eq_ignore_ascii_case("transfer-encoding") {
                    transfer_encoding = Some(value.trim());
                }
            }
            if transfer_encoding.is_some() {
                write_http_response(
                    &mut stream,
                    "400 Bad Request",
                    "application/json",
                    "{\"error\": \"Transfer-Encoding is not supported\"}",
                )?;
                return Ok(());
            }
            let content_length = content_length.unwrap_or(0);
            let mut rest = buf[header_end..].to_vec();
            while rest.len() < content_length {
                let n = stream.read(&mut tmp)?;
                if n == 0 {
                    write_http_response(
                        &mut stream,
                        "400 Bad Request",
                        "application/json",
                        "{\"error\": \"request body is shorter than Content-Length\"}",
                    )?;
                    return Ok(());
                }
                rest.extend_from_slice(&tmp[..n]);
            }
            body = rest.into_iter().take(content_length).collect();
            break;
        }
    }
    let (status, content_type, payload) = route(&method, &path, &body, &jobs, route_seconds);
    write_http_response(&mut stream, status, content_type, &payload)?;
    Ok(())
}

fn write_http_response<S: Write>(
    stream: &mut S,
    status: &str,
    content_type: &str,
    payload: &str,
) -> std::io::Result<()> {
    let response = format!(
        "HTTP/1.1 {status}\r\nContent-Type: {content_type}\r\nContent-Length: {}\r\nConnection: close\r\n\r\n",
        payload.len()
    );
    stream.write_all(response.as_bytes())?;
    stream.write_all(payload.as_bytes())
}

fn find_headers_end(buf: &[u8]) -> Option<usize> {
    buf.windows(4).position(|w| w == b"\r\n\r\n").map(|p| p + 4)
}

fn job_json(job: &RoutingJob) -> String {
    format!(
        "{{\"id\": {}, \"state\": \"{}\", \"score\": {}, \"error\": {}}}",
        job.id,
        job.state.as_str(),
        job.score.map_or("null".to_string(), |s| format!("{s:.2}")),
        job.error
            .as_ref()
            .map_or("null".to_string(), |e| json_string(e)),
    )
}

fn route(
    method: &str,
    path: &str,
    body: &[u8],
    jobs: &Jobs,
    route_seconds: u64,
) -> (&'static str, &'static str, String) {
    let not_found = (
        "404 Not Found",
        "application/json",
        "{\"error\": \"not found\"}".to_string(),
    );
    let segments: Vec<&str> = path.trim_matches('/').split('/').collect();
    match (method, segments.as_slice()) {
        ("GET", ["v1", "system", "status"]) => (
            "200 OK",
            "application/json",
            format!(
                "{{\"status\": \"OK\", \"host\": \"freerouting-rs\", \"version\": \"{}\"}}",
                env!("CARGO_PKG_VERSION")
            ),
        ),
        ("POST", ["v1", "jobs", "enqueue"]) => {
            let id = NEXT_JOB_ID.fetch_add(1, Ordering::SeqCst);
            jobs.lock().unwrap().insert(
                id,
                RoutingJob {
                    id,
                    state: JobState::Queued,
                    input_dsn: None,
                    output_ses: None,
                    score: None,
                    error: None,
                    cancel: Arc::new(AtomicBool::new(false)),
                },
            );
            (
                "200 OK",
                "application/json",
                format!("{{\"id\": {id}, \"state\": \"QUEUED\"}}"),
            )
        }
        ("POST", ["v1", "jobs", id, "input"]) => {
            let Ok(id) = id.parse::<u64>() else {
                return not_found;
            };
            {
                let map = jobs.lock().unwrap();
                let Some(job) = map.get(&id) else {
                    return not_found;
                };
                if !matches!(job.state, JobState::Queued | JobState::ReadyToStart) {
                    return (
                        "409 Conflict",
                        "application/json",
                        format!(
                            "{{\"error\": \"job is {} and no longer accepts input\"}}",
                            job.state.as_str()
                        ),
                    );
                }
            }
            if body.is_empty() {
                return (
                    "400 Bad Request",
                    "application/json",
                    "{\"error\": \"dsn input must not be empty\"}".to_string(),
                );
            }
            let input = match std::str::from_utf8(body) {
                Ok(input) => input,
                Err(_) => {
                    return (
                        "400 Bad Request",
                        "application/json",
                        "{\"error\": \"dsn input must be valid UTF-8\"}".to_string(),
                    )
                }
            };
            // Reject malformed designs before they become job state. The
            // worker imports again from the retained source, but deferring
            // this check until start made READY_TO_START mean only "UTF-8",
            // not "a routable design".
            if let Err(error) = import_dsn(input) {
                return (
                    "400 Bad Request",
                    "application/json",
                    format!("{{\"error\": {}}}", json_string(&error.to_string())),
                );
            }
            let mut map = jobs.lock().unwrap();
            let Some(job) = map.get_mut(&id) else {
                return not_found;
            };
            // Input may only be (re)uploaded before the job starts. Check
            // under the same lock used for the state mutation, so a worker
            // cannot start between validation and commit.
            if !matches!(job.state, JobState::Queued | JobState::ReadyToStart) {
                return (
                    "409 Conflict",
                    "application/json",
                    format!(
                        "{{\"error\": \"job is {} and no longer accepts input\"}}",
                        job.state.as_str()
                    ),
                );
            }
            job.input_dsn = Some(input.to_string());
            job.state = JobState::ReadyToStart;
            ("200 OK", "application/json", job_json(job))
        }
        ("PUT", ["v1", "jobs", id, "start"]) => {
            let Ok(id) = id.parse::<u64>() else {
                return not_found;
            };
            let (input, cancel) = {
                let mut map = jobs.lock().unwrap();
                let Some(job) = map.get_mut(&id) else {
                    return not_found;
                };
                if job.state != JobState::ReadyToStart {
                    return (
                        "409 Conflict",
                        "application/json",
                        "{\"error\": \"job has no input or already started\"}".to_string(),
                    );
                }
                job.state = JobState::Running;
                (
                    job.input_dsn.clone().unwrap_or_default(),
                    job.cancel.clone(),
                )
            };
            let jobs_bg = jobs.clone();
            std::thread::spawn(move || {
                let result = run_job(&input, route_seconds, &cancel);
                let mut map = jobs_bg.lock().unwrap();
                if let Some(job) = map.get_mut(&id) {
                    match result {
                        Ok((ses, score, issues)) => {
                            job.output_ses = Some(ses);
                            job.score = Some(score);
                            job.state = if cancel.load(Ordering::SeqCst) {
                                JobState::Cancelled
                            } else if let Some(gate) = issues {
                                // incomplete or violating output is not a
                                // success; the session stays downloadable
                                job.error = Some(gate);
                                JobState::Failed
                            } else {
                                JobState::Completed
                            };
                        }
                        Err(e) => {
                            job.error = Some(e);
                            job.state = JobState::Failed;
                        }
                    }
                }
            });
            (
                "200 OK",
                "application/json",
                format!("{{\"id\": {id}, \"state\": \"RUNNING\"}}"),
            )
        }
        ("PUT", ["v1", "jobs", id, "cancel"]) => {
            let Ok(id) = id.parse::<u64>() else {
                return not_found;
            };
            let mut map = jobs.lock().unwrap();
            let Some(job) = map.get_mut(&id) else {
                return not_found;
            };
            job.cancel.store(true, Ordering::SeqCst);
            // a job with no worker (never started) has nobody to observe
            // the flag: transition it to CANCELLED directly, or it would
            // stay QUEUED/READY_TO_START forever
            if matches!(job.state, JobState::Queued | JobState::ReadyToStart) {
                job.state = JobState::Cancelled;
            }
            ("200 OK", "application/json", job_json(job))
        }
        ("GET", ["v1", "jobs", id]) => {
            let Ok(id) = id.parse::<u64>() else {
                return not_found;
            };
            let map = jobs.lock().unwrap();
            let Some(job) = map.get(&id) else {
                return not_found;
            };
            ("200 OK", "application/json", job_json(job))
        }
        ("GET", ["v1", "jobs", id, "output"]) => {
            let Ok(id) = id.parse::<u64>() else {
                return not_found;
            };
            let map = jobs.lock().unwrap();
            let Some(job) = map.get(&id) else {
                return not_found;
            };
            match &job.output_ses {
                Some(ses) => ("200 OK", "text/plain", ses.clone()),
                None => (
                    "409 Conflict",
                    "application/json",
                    "{\"error\": \"job not completed\"}".to_string(),
                ),
            }
        }
        ("POST", ["v1", "mcp"]) => {
            let text = String::from_utf8_lossy(body).to_string();
            (
                "200 OK",
                "application/json",
                mcp_dispatch(&text, jobs, route_seconds),
            )
        }
        _ => not_found,
    }
}

/// The MCP JSON-RPC 2.0 endpoint (Java: `McpControllerV1`): initialize,
/// tools/list and tools/call over the job API.
fn mcp_dispatch(request: &str, jobs: &Jobs, route_seconds: u64) -> String {
    use crate::io::json::{parse_json, Json};
    let Ok(req) = parse_json(request) else {
        return mcp_error(Json::Null, -32700, "Parse error");
    };
    if !matches!(req, Json::Obj(_)) {
        return mcp_error(Json::Null, -32600, "Invalid Request");
    }
    let id = req.get("id").cloned().unwrap_or(Json::Null);
    if !matches!(id, Json::Null | Json::Num { .. } | Json::Str(_)) {
        return mcp_error(Json::Null, -32600, "Invalid Request");
    }
    if req.get("jsonrpc").and_then(Json::as_str) != Some("2.0") {
        return mcp_error(id, -32600, "Invalid Request");
    }
    let Some(method) = req.get("method").and_then(Json::as_str) else {
        return mcp_error(id, -32600, "Invalid Request");
    };
    match method {
        "initialize" => mcp_result(
            &id,
            &format!(
                "{{\"protocolVersion\": \"2024-11-05\", \"capabilities\": {{\"tools\": {{}}}},                  \"serverInfo\": {{\"name\": \"freerouting-rs\", \"version\": \"{}\"}}}}",
                env!("CARGO_PKG_VERSION")
            ),
        ),
        "notifications/initialized" => String::new(),
        "tools/list" => mcp_result(
            &id,
            r#"{"tools": [
              {"name": "enqueue_job", "description": "Create a new routing job", "inputSchema": {"type": "object", "properties": {}}},
              {"name": "set_job_input", "description": "Upload the Specctra DSN design of a job", "inputSchema": {"type": "object", "properties": {"job_id": {"type": "integer", "minimum": 0}, "dsn": {"type": "string"}}, "required": ["job_id", "dsn"]}},
              {"name": "start_job", "description": "Start routing a job", "inputSchema": {"type": "object", "properties": {"job_id": {"type": "integer", "minimum": 0}}, "required": ["job_id"]}},
              {"name": "get_job_details", "description": "The state and score of a job", "inputSchema": {"type": "object", "properties": {"job_id": {"type": "integer", "minimum": 0}}, "required": ["job_id"]}},
              {"name": "get_job_output", "description": "The routed session file of a completed job", "inputSchema": {"type": "object", "properties": {"job_id": {"type": "integer", "minimum": 0}}, "required": ["job_id"]}},
              {"name": "system_status", "description": "Server health and version", "inputSchema": {"type": "object", "properties": {}}}
            ]}"#,
        ),
        "tools/call" => {
            let Some(Json::Obj(_)) = req.get("params") else {
                return mcp_error(id, -32602, "params must be an object");
            };
            let params = req.get("params").expect("validated params object");
            let Some(name) = params.get("name").and_then(Json::as_str) else {
                return mcp_error(id, -32602, "tool name must be a string");
            };
            let args = match params.get("arguments") {
                Some(value @ Json::Obj(_)) => value,
                Some(_) => return mcp_error(id, -32602, "arguments must be an object"),
                None => {
                    return mcp_error(id, -32602, "arguments must be an object");
                }
            };
            // Validate the original JSON number lexeme. Converting through
            // f64 first would turn 9007199254740993 into 9007199254740992.
            let needs_id = !matches!(name, "enqueue_job" | "system_status");
            let job_id = if let Some(value) = args.get("job_id") {
                match value.as_exact_u64() {
                    Some(value) => value,
                    None => {
                        return mcp_error(id, -32602, "job_id must be a non-negative integer")
                    }
                }
            } else if needs_id {
                return mcp_error(id, -32602, "job_id is required");
            } else {
                0
            };
            let (verb, path, payload): (&str, String, Vec<u8>) = match name {
                "enqueue_job" => ("POST", "/v1/jobs/enqueue".into(), Vec::new()),
                "set_job_input" => {
                    let dsn = match args.get("dsn") {
                        Some(crate::io::json::Json::Str(value)) if !value.is_empty() => value,
                        Some(crate::io::json::Json::Str(_)) => {
                            return mcp_error(id, -32602, "dsn must be a non-empty string")
                        }
                        Some(_) => return mcp_error(id, -32602, "dsn must be a string"),
                        None => return mcp_error(id, -32602, "dsn is required"),
                    };
                    (
                        "POST",
                        format!("/v1/jobs/{job_id}/input"),
                        dsn.as_bytes().to_vec(),
                    )
                }
                "start_job" => ("PUT", format!("/v1/jobs/{job_id}/start"), Vec::new()),
                "get_job_details" => ("GET", format!("/v1/jobs/{job_id}"), Vec::new()),
                "get_job_output" => ("GET", format!("/v1/jobs/{job_id}/output"), Vec::new()),
                "system_status" => ("GET", "/v1/system/status".into(), Vec::new()),
                _ => return mcp_error(id, -32602, "Unknown tool"),
            };
            let (status, _, out) = route(verb, &path, &payload, jobs, route_seconds);
            // a REST failure surfaces as an MCP tool ERROR (isError), not a
            // successful result wrapping the error body
            let is_error = !status.starts_with("2");
            mcp_result(
                &id,
                &format!(
                    "{{\"content\": [{{\"type\": \"text\", \"text\": {}}}], \"isError\": {}}}",
                    json_string(&out),
                    is_error
                ),
            )
        }
        _ => mcp_error(id, -32601, "Unknown method"),
    }
}

fn json_string(s: &str) -> String {
    format!("\"{}\"", crate::io::json::escape(s))
}

fn mcp_result(id: &crate::io::json::Json, result: &str) -> String {
    let id_s = match id {
        crate::io::json::Json::Num { raw, .. } => raw.clone(),
        // a valid string id may contain quotes/backslashes: escape it
        crate::io::json::Json::Str(s) => json_string(s),
        _ => "null".to_string(),
    };
    format!("{{\"jsonrpc\": \"2.0\", \"id\": {id_s}, \"result\": {result}}}")
}

fn mcp_error(id: crate::io::json::Json, code: i32, message: &str) -> String {
    let id_s = match id {
        crate::io::json::Json::Num { raw, .. } => raw,
        crate::io::json::Json::Str(s) => json_string(&s),
        _ => "null".to_string(),
    };
    format!(
        "{{\"jsonrpc\": \"2.0\", \"id\": {id_s}, \"error\": {{\"code\": {code}, \"message\": {}}}}}",
        json_string(message)
    )
}

/// Routes one job. Returns the session output, the score, and — when the
/// routed board is incomplete or violates clearances — a gate message the
/// caller reports as the job error (the output stays downloadable, like
/// the CLI writing its output while exiting 2/3).
fn run_job(
    dsn: &str,
    route_seconds: u64,
    cancel: &AtomicBool,
) -> Result<(String, f64, Option<String>), String> {
    let mut board = import_dsn(dsn).map_err(|e| format!("{e}"))?;
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
    // Route in bounded slices so a cancel request takes effect between
    // slices instead of being ignored until the whole budget is spent
    // (TimeLimit is Copy-plumbed through the router, so the flag cannot
    // ride inside it).
    let limit = TimeLimit::new(route_seconds.saturating_mul(1000));
    let all_connected = |board: &crate::board::basic_board::BasicBoard| {
        (1..=board.rules.nets.max_net_no()).all(|n| board.net_is_completely_connected(n))
    };
    while !cancel.load(Ordering::SeqCst) && !limit.limit_exceeded() && !all_connected(&board) {
        let slice = TimeLimit::new(limit.remaining_ms().clamp(1, 2_000));
        batch_route_passes_with_time_limit(&mut board, &request, 99, Some(&slice));
    }
    if !cancel.load(Ordering::SeqCst) {
        crate::autoroute::combine_all_traces(&mut board);
        crate::autoroute::pull_tight_all(&mut board, 3);
    }
    let stats = crate::scoring::BoardStatistics::collect(&board);
    let score = stats.normalized_score(&crate::scoring::ScoringSettings::default());
    let ses = export_ses(&board, "api_job", board.resolution).map_err(|e| e.to_string())?;
    // final gate (the CLI's exit-2/exit-3 equivalent): a routing result
    // that is incomplete or violates clearances must not read as success
    let report = crate::drc::check_board(&board);
    let issues = if report.unconnected.is_empty() && report.violations.is_empty() {
        None
    } else {
        Some(format!(
            "routing gate: {} unconnected net(s), {} clearance violation(s)",
            report.unconnected.len(),
            report.violations.len()
        ))
    };
    Ok((ses, score, issues))
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::net::Shutdown;
    #[cfg(unix)]
    use std::os::unix::net::UnixStream;

    fn empty_jobs() -> Jobs {
        Arc::new(Mutex::new(HashMap::new()))
    }

    fn queued_jobs(id: u64) -> Jobs {
        let jobs = empty_jobs();
        jobs.lock().unwrap().insert(
            id,
            RoutingJob {
                id,
                state: JobState::Queued,
                input_dsn: None,
                output_ses: None,
                score: None,
                error: None,
                cancel: Arc::new(AtomicBool::new(false)),
            },
        );
        jobs
    }

    #[cfg(unix)]
    fn http_exchange(request: &[u8]) -> String {
        // A Unix stream pair exercises the same blocking read/write contract
        // without consuming a real TCP listener.  This keeps parser tests
        // deterministic in sandboxed CI environments where bind(2) is denied.
        let (mut client, server_stream) = UnixStream::pair().expect("test stream pair");
        let server = std::thread::spawn(move || {
            handle(server_stream, empty_jobs(), 1).expect("handle test request");
        });
        client.write_all(request).expect("write test request");
        client
            .shutdown(Shutdown::Write)
            .expect("finish test request");
        let mut response = String::new();
        client
            .read_to_string(&mut response)
            .expect("read test response");
        server.join().expect("join test server");
        response
    }

    #[cfg(unix)]
    #[test]
    fn http_parser_rejects_ambiguous_or_unsupported_framing() {
        for request in [
            b"GET /v1/system/status HTTP/1.1 extra\r\n\r\n".as_slice(),
            b"POST /v1/mcp HTTP/1.1\r\nContent-Length: 0\r\nContent-Length: 0\r\n\r\n",
            b"POST /v1/mcp HTTP/1.1\r\nTransfer-Encoding: chunked\r\n\r\n",
            b"GET /v1/system/status HTTP/1.1\r\n",
        ] {
            let response = http_exchange(request);
            assert!(response.starts_with("HTTP/1.1 400"), "{response}");
        }
        let response = http_exchange(b"GET /v1/system/status HTTP/1.1\r\n\r\n");
        assert!(response.starts_with("HTTP/1.1 200"), "{response}");

        let mut oversized = b"GET /v1/system/status HTTP/1.1\r\nX-Pad: ".to_vec();
        oversized.extend(std::iter::repeat_n(b'x', 65 * 1024));
        oversized.extend_from_slice(b"\r\n\r\n");
        let response = http_exchange(&oversized);
        assert!(response.starts_with("HTTP/1.1 431"), "{response}");
    }

    #[test]
    fn mcp_preserves_job_ids_above_f64_exact_range() {
        let id = 9_007_199_254_740_993_u64; // 2^53 + 1
        let jobs = empty_jobs();
        jobs.lock().unwrap().insert(
            id,
            RoutingJob {
                id,
                state: JobState::Queued,
                input_dsn: None,
                output_ses: None,
                score: None,
                error: None,
                cancel: Arc::new(AtomicBool::new(false)),
            },
        );
        let request = format!(
            r#"{{"jsonrpc":"2.0","id":1,"method":"tools/call","params":{{"name":"get_job_details","arguments":{{"job_id":{id}}}}}}}"#
        );
        let response = mcp_dispatch(&request, &jobs, 1);
        assert!(
            response.contains("QUEUED"),
            "exact id must resolve: {response}"
        );
        assert!(response.contains("\"isError\": false"));
    }

    #[test]
    fn mcp_rejects_non_integer_job_ids_before_routing() {
        let jobs = empty_jobs();
        for literal in ["-1", "1.0", "1e3", "9007199254740993.0"] {
            let request = format!(
                r#"{{"jsonrpc":"2.0","id":1,"method":"tools/call","params":{{"name":"get_job_details","arguments":{{"job_id":{literal}}}}}}}"#
            );
            let response = mcp_dispatch(&request, &jobs, 1);
            assert!(response.contains("job_id must be a non-negative integer"));
        }
    }

    #[test]
    fn mcp_rejects_missing_non_string_and_empty_dsn_input() {
        let jobs = empty_jobs();
        let requests = [
            (r#"{"job_id": 1}"#, "dsn is required"),
            (r#"{"job_id": 1, "dsn": 42}"#, "dsn must be a string"),
            (
                r#"{"job_id": 1, "dsn": ""}"#,
                "dsn must be a non-empty string",
            ),
        ];
        for (arguments, message) in requests {
            let request = format!(
                r#"{{"jsonrpc":"2.0","id":1,"method":"tools/call","params":{{"name":"set_job_input","arguments":{arguments}}}}}"#
            );
            let response = mcp_dispatch(&request, &jobs, 1);
            assert!(response.contains(message), "{message}: {response}");
        }
    }

    #[test]
    fn mcp_rejects_invalid_json_rpc_envelopes_and_parameter_shapes() {
        let jobs = empty_jobs();
        for request in [
            r#"[]"#,
            r#"{"id":1,"method":"tools/list"}"#,
            r#"{"jsonrpc":"1.0","id":1,"method":"tools/list"}"#,
            r#"{"jsonrpc":"2.0","id":1}"#,
        ] {
            let response = mcp_dispatch(request, &jobs, 1);
            assert!(response.contains("\"code\": -32600"), "{response}");
        }
        for request in [
            r#"{"jsonrpc":"2.0","id":1,"method":"tools/call"}"#,
            r#"{"jsonrpc":"2.0","id":1,"method":"tools/call","params":[]}"#,
            r#"{"jsonrpc":"2.0","id":1,"method":"tools/call","params":{"name":"system_status","arguments":[]}}"#,
        ] {
            let response = mcp_dispatch(request, &jobs, 1);
            assert!(response.contains("\"code\": -32602"), "{response}");
        }
    }

    #[test]
    fn rest_rejects_empty_and_non_utf8_dsn_input() {
        for (body, expected) in [
            (Vec::new(), "dsn input must not be empty"),
            (vec![0xff], "dsn input must be valid UTF-8"),
        ] {
            let jobs = queued_jobs(7);
            let (status, _, payload) = route("POST", "/v1/jobs/7/input", &body, &jobs, 1);
            assert_eq!(status, "400 Bad Request");
            assert!(payload.contains(expected), "{expected}: {payload}");
            assert_eq!(
                jobs.lock().unwrap().get(&7).unwrap().state,
                JobState::Queued
            );
        }
    }

    #[test]
    fn rest_rejects_invalid_dsn_before_changing_job_state() {
        let jobs = queued_jobs(7);
        let (status, _, payload) = route(
            "POST",
            "/v1/jobs/7/input",
            b"(pcb broken (resolution parsec 1) (structure (layer F.Cu)))",
            &jobs,
            1,
        );
        assert_eq!(status, "400 Bad Request");
        assert!(payload.contains("DSN import error"), "{payload}");
        let jobs = jobs.lock().unwrap();
        let job = jobs.get(&7).unwrap();
        assert_eq!(job.state, JobState::Queued);
        assert!(job.input_dsn.is_none());
    }
}
