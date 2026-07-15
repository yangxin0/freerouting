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
use std::net::{TcpListener, TcpStream};
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

fn handle(mut stream: TcpStream, jobs: Jobs, route_seconds: u64) -> std::io::Result<()> {
    let mut buf = Vec::new();
    let mut tmp = [0u8; 4096];
    // read until headers complete, then body per content-length
    let (mut method, mut path, mut body) = (String::new(), String::new(), Vec::new());
    loop {
        let n = stream.read(&mut tmp)?;
        if n == 0 {
            break;
        }
        buf.extend_from_slice(&tmp[..n]);
        if let Some(header_end) = find_headers_end(&buf) {
            let headers = String::from_utf8_lossy(&buf[..header_end]).to_string();
            let mut lines = headers.lines();
            if let Some(request_line) = lines.next() {
                let mut parts = request_line.split_whitespace();
                method = parts.next().unwrap_or("").to_string();
                path = parts.next().unwrap_or("").to_string();
            }
            let content_length: usize = headers
                .lines()
                .find_map(|l| {
                    let (k, v) = l.split_once(':')?;
                    k.eq_ignore_ascii_case("content-length")
                        .then(|| v.trim().parse().ok())?
                })
                .unwrap_or(0);
            let mut rest = buf[header_end..].to_vec();
            while rest.len() < content_length {
                let n = stream.read(&mut tmp)?;
                if n == 0 {
                    break;
                }
                rest.extend_from_slice(&tmp[..n]);
            }
            body = rest;
            break;
        }
    }
    let (status, content_type, payload) = route(&method, &path, &body, &jobs, route_seconds);
    let response = format!(
        "HTTP/1.1 {status}\r\nContent-Type: {content_type}\r\nContent-Length: {}\r\nConnection: close\r\n\r\n",
        payload.len()
    );
    stream.write_all(response.as_bytes())?;
    stream.write_all(payload.as_bytes())?;
    Ok(())
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
            .map_or("null".to_string(), |e| format!("\"{}\"", e.replace('"', "'"))),
    )
}

fn route(
    method: &str,
    path: &str,
    body: &[u8],
    jobs: &Jobs,
    route_seconds: u64,
) -> (&'static str, &'static str, String) {
    let not_found = ("404 Not Found", "application/json", "{\"error\": \"not found\"}".to_string());
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
            let Ok(id) = id.parse::<u64>() else { return not_found };
            let mut map = jobs.lock().unwrap();
            let Some(job) = map.get_mut(&id) else { return not_found };
            job.input_dsn = Some(String::from_utf8_lossy(body).to_string());
            job.state = JobState::ReadyToStart;
            ("200 OK", "application/json", job_json(job))
        }
        ("PUT", ["v1", "jobs", id, "start"]) => {
            let Ok(id) = id.parse::<u64>() else { return not_found };
            let (input, cancel) = {
                let mut map = jobs.lock().unwrap();
                let Some(job) = map.get_mut(&id) else { return not_found };
                if job.state != JobState::ReadyToStart {
                    return (
                        "409 Conflict",
                        "application/json",
                        "{\"error\": \"job has no input or already started\"}".to_string(),
                    );
                }
                job.state = JobState::Running;
                (job.input_dsn.clone().unwrap_or_default(), job.cancel.clone())
            };
            let jobs_bg = jobs.clone();
            std::thread::spawn(move || {
                let result = run_job(&input, route_seconds, &cancel);
                let mut map = jobs_bg.lock().unwrap();
                if let Some(job) = map.get_mut(&id) {
                    match result {
                        Ok((ses, score)) => {
                            job.output_ses = Some(ses);
                            job.score = Some(score);
                            job.state = if cancel.load(Ordering::SeqCst) {
                                JobState::Cancelled
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
            let Ok(id) = id.parse::<u64>() else { return not_found };
            let map = jobs.lock().unwrap();
            let Some(job) = map.get(&id) else { return not_found };
            job.cancel.store(true, Ordering::SeqCst);
            ("200 OK", "application/json", job_json(job))
        }
        ("GET", ["v1", "jobs", id]) => {
            let Ok(id) = id.parse::<u64>() else { return not_found };
            let map = jobs.lock().unwrap();
            let Some(job) = map.get(&id) else { return not_found };
            ("200 OK", "application/json", job_json(job))
        }
        ("GET", ["v1", "jobs", id, "output"]) => {
            let Ok(id) = id.parse::<u64>() else { return not_found };
            let map = jobs.lock().unwrap();
            let Some(job) = map.get(&id) else { return not_found };
            match &job.output_ses {
                Some(ses) => ("200 OK", "text/plain", ses.clone()),
                None => (
                    "409 Conflict",
                    "application/json",
                    "{\"error\": \"job not completed\"}".to_string(),
                ),
            }
        }
        _ => not_found,
    }
}

fn run_job(
    dsn: &str,
    route_seconds: u64,
    _cancel: &AtomicBool,
) -> Result<(String, f64), String> {
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
        via_cost: 50_000.0,
        max_expansions: 100_000,
        ripup_penalty: 0.0,
        deadline: None,
    };
    let limit = TimeLimit::new(route_seconds.saturating_mul(1000));
    batch_route_passes_with_time_limit(&mut board, &request, 99, Some(&limit));
    crate::autoroute::combine_all_traces(&mut board);
    crate::autoroute::pull_tight_all(&mut board, 3);
    let stats = crate::scoring::BoardStatistics::collect(&board);
    let score = stats.normalized_score(&crate::scoring::ScoringSettings::default());
    let ses = export_ses(&board, "api_job", board.resolution);
    Ok((ses, score))
}
