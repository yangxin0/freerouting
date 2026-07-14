//! Port of `app.freerouting.rules` (incremental).

pub mod clearance_matrix;
pub mod net;
pub mod net_class;
pub mod via_rule;

pub use clearance_matrix::ClearanceMatrix;
pub use net::{Net, Nets};
pub use net_class::{DefaultItemClearanceClasses, ItemClass, NetClass, NetClasses};
pub use via_rule::{ViaInfo, ViaInfoId, ViaInfos, ViaRule};
