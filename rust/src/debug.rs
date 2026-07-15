//! Process-environment debug flags, read once and cached: `getenv` takes
//! a global lock and showed up at ~25% of samples when the flag checks
//! sat in the routing hot loops.

use crate::geometry::planar::IntBox;
use std::sync::OnceLock;

fn flag(cell: &'static OnceLock<bool>, name: &str) -> bool {
    *cell.get_or_init(|| std::env::var_os(name).is_some())
}

/// FR_DEBUG_MAZE: routing/insert diagnostics on stderr.
pub fn maze() -> bool {
    static C: OnceLock<bool> = OnceLock::new();
    flag(&C, "FR_DEBUG_MAZE")
}

/// FR_DEBUG_PATH: per-corner path dumps.
pub fn path() -> bool {
    static C: OnceLock<bool> = OnceLock::new();
    flag(&C, "FR_DEBUG_PATH")
}

/// FR_DEBUG_SHOVE: shove diagnostics.
pub fn shove() -> bool {
    static C: OnceLock<bool> = OnceLock::new();
    flag(&C, "FR_DEBUG_SHOVE")
}

/// FR_AUDIT_ROOMS: exact room-vs-board audit after every search.
pub fn audit_rooms() -> bool {
    static C: OnceLock<bool> = OnceLock::new();
    flag(&C, "FR_AUDIT_ROOMS")
}

/// FR_ROUTE_ORDER_DESC: reverse the batch net order (experiment knob).
pub fn route_order_desc() -> bool {
    static C: OnceLock<bool> = OnceLock::new();
    flag(&C, "FR_ROUTE_ORDER_DESC")
}

/// FR_DEBUG_REGION: "x1,y1,x2,y2" watch window for region-scoped logs.
pub fn region() -> Option<IntBox> {
    static C: OnceLock<Option<IntBox>> = OnceLock::new();
    *C.get_or_init(|| {
        let v = std::env::var("FR_DEBUG_REGION").ok()?;
        let n: Vec<i32> = v.split(',').filter_map(|p| p.parse().ok()).collect();
        let [x1, y1, x2, y2] = n[..] else { return None };
        Some(IntBox::from_coords(x1, y1, x2, y2))
    })
}

/// FR_ASTAR_WEIGHT: multiplier on the A* estimate (default 1.0).
pub fn astar_weight() -> f64 {
    static C: OnceLock<f64> = OnceLock::new();
    *C.get_or_init(|| {
        std::env::var("FR_ASTAR_WEIGHT")
            .ok()
            .and_then(|v| v.parse().ok())
            .unwrap_or(1.0)
    })
}
