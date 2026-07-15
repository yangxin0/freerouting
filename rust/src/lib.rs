//! Rust reimplementation of the freerouting PCB auto-router.
//!
//! Ported incrementally from the Java sources under `src/main/java/app/freerouting`.
//! See `rust/PORTING.md` for progress and porting conventions.

pub mod autoroute;
pub mod board;
pub mod core;
pub mod datastructures;
pub mod debug;
pub mod geometry;
pub mod io;
pub mod rules;
