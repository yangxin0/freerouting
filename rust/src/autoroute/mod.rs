//! Port of `app.freerouting.autoroute` (incremental).
//!
//! The faithful maze expansion engine (AutorouteEngine, expansion rooms,
//! MazeSearchAlgo) is the supported routing surface.  The old interim grid
//! router remains only as a unit-test oracle: it accepts a caller-supplied
//! padstack and cannot represent ordered ViaInfo candidates, active-layer
//! masks, candidate-specific clearance/attach policy, or transactional DRC.
//! Exporting it alongside the maze router therefore bypassed the routing
//! contract even though no production caller used it.

pub mod batch;
pub mod drill_pages;
pub mod engine;
pub mod expansion_room;
pub mod fanout;
mod maze_search;
pub mod optimizer;
pub mod pull_tight;
pub mod room_completion;
#[cfg(test)]
mod simple_router;
pub mod sorted_room_neighbours;

pub use batch::{
    batch_route, batch_route_passes, batch_route_passes_with_time_limit, route_net,
    route_net_with_ripup, BatchRequest, BatchResult,
};
pub use engine::{AutorouteEngine, TargetDoor};
pub use expansion_room::{ExpansionDoor, ExpansionRoom, MazeSearchElement, RoomGraph, RoomKind};
pub use fanout::{fanout_board, fanout_pin};
pub use maze_search::take_stats;
pub use optimizer::{
    optimize_route, optimize_route_multithreaded, optimize_route_multithreaded_with_strategy,
    optimize_route_pass, optimize_vias, BoardUpdateStrategy, ItemSelectionStrategy,
};
pub use pull_tight::{combine_all_traces, pull_tight_all, pull_tight_trace, total_trace_length};
pub use room_completion::{complete_shape, restrain_shape, IncompleteRoom};
