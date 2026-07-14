//! Port of `app.freerouting.autoroute` (incremental).
//!
//! The faithful maze expansion engine (AutorouteEngine, expansion rooms,
//! MazeSearchAlgo) is ported step by step; `simple_router` provides an
//! interim grid router so the pipeline is routable end to end meanwhile.

pub mod expansion_room;
pub mod room_completion;
pub mod simple_router;

pub use expansion_room::{
    ExpansionDoor, ExpansionRoom, MazeSearchElement, RoomGraph, RoomKind,
};
pub use room_completion::{complete_shape, restrain_shape, IncompleteRoom};
pub use simple_router::{RouteRequest, RouteResult, SimpleRouter};
