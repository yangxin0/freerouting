//! Port of `app.freerouting.autoroute` (incremental).
//!
//! The faithful maze expansion engine (AutorouteEngine, expansion rooms,
//! MazeSearchAlgo) is ported step by step; `simple_router` provides an
//! interim grid router so the pipeline is routable end to end meanwhile.

pub mod simple_router;

pub use simple_router::{RouteRequest, RouteResult, SimpleRouter};
