//! Port of `app.freerouting.autoroute` (incremental).
//!
//! The faithful maze expansion engine (AutorouteEngine, expansion rooms,
//! MazeSearchAlgo) is ported step by step; `simple_router` provides an
//! interim grid router so the pipeline is routable end to end meanwhile.

pub mod batch;
pub mod drill_pages;
pub mod engine;
pub mod expansion_room;
pub mod maze_search;
pub mod fanout;
pub mod optimizer;
pub mod pull_tight;
pub mod room_completion;
pub mod sorted_room_neighbours;
pub mod simple_router;

pub use batch::{
    batch_route, batch_route_passes, batch_route_passes_with_time_limit, route_net,
    route_net_with_ripup, BatchRequest, BatchResult,
};
pub use engine::{AutorouteEngine, TargetDoor};
pub use expansion_room::{
    ExpansionDoor, ExpansionRoom, MazeSearchElement, RoomGraph, RoomKind,
};
pub use maze_search::{take_stats, 
    find_connection, maze_route, maze_route_with_ripup, MazeSearchResult, RoutedConnection,
};
pub use fanout::{fanout_board, fanout_pin};
pub use optimizer::{optimize_route, optimize_route_pass, optimize_vias};
pub use pull_tight::{combine_all_traces, pull_tight_all, pull_tight_trace, total_trace_length};
pub use room_completion::{complete_shape, restrain_shape, IncompleteRoom};
pub use simple_router::{RouteRequest, RouteResult, SimpleRouter};
