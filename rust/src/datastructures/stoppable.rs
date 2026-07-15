//! Port of `datastructures/Stoppable.java` and `TimeLimit.java`.

use std::time::{Duration, Instant};

/// Interface for stoppable threads/algorithms.
pub trait Stoppable {
    /// Returns true if the algorithm is requested to be stopped.
    fn is_stop_requested(&self) -> bool;
}

/// A time limit for interrupting long-running algorithms.
#[derive(Debug, Clone, Copy)]
pub struct TimeLimit {
    start: Instant,
    limit: Duration,
}

impl TimeLimit {
    /// Creates a time limit of `milliseconds`.
    pub fn new(milliseconds: u64) -> Self {
        TimeLimit {
            start: Instant::now(),
            limit: Duration::from_millis(milliseconds),
        }
    }

    /// Returns true if the time limit is exceeded.
    pub fn limit_exceeded(&self) -> bool {
        self.start.elapsed() >= self.limit
    }

    /// Milliseconds until the limit is exceeded (0 when already over).
    pub fn remaining_ms(&self) -> u64 {
        self.limit
            .saturating_sub(self.start.elapsed())
            .as_millis() as u64
    }

    /// Multiplies the time limit by `factor` (Java: `multiply`).
    pub fn multiply(&mut self, factor: f64) {
        if factor <= 0.0 {
            return;
        }
        self.limit = Duration::from_secs_f64(self.limit.as_secs_f64() * factor);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn time_limit() {
        let t = TimeLimit::new(10_000);
        assert!(!t.limit_exceeded());
        let t = TimeLimit::new(0);
        assert!(t.limit_exceeded());
    }
}
