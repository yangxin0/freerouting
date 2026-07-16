//! Diagnostic: import a DSN and dump the items of one net (or all nets'
//! item counts) with shapes and layers.

use freerouting::autoroute::{route_net, BatchRequest};
use freerouting::io::import_dsn;

fn main() {
    let path = std::env::args()
        .nth(1)
        .expect("usage: net_debug <dsn> [net]");
    let net_name = std::env::args().nth(2);
    let content = std::fs::read_to_string(&path).expect("read failed");
    let content = match content.find("  (wiring") {
        Some(pos) => format!("{})", &content[..pos]),
        None => content,
    };
    let mut board = import_dsn(&content).expect("import failed");

    // --route: try routing the named net alone on the fresh board
    if std::env::args().any(|a| a == "--route") {
        let name = net_name.clone().expect("--route needs a net name");
        let net_no = board
            .rules
            .nets
            .get_by_name(&name)
            .first()
            .map(|n| n.net_number)
            .expect("net not found");
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
        let result = route_net(&mut board, net_no, &request);
        println!(
            "route {name} alone: {} routed, {} failed, connected={}",
            result.routed_connections,
            result.failed_connections,
            board.net_is_completely_connected(net_no)
        );
        return;
    }

    match net_name {
        None => {
            for net_no in 1..=board.rules.nets.max_net_no() {
                let name = board
                    .rules
                    .nets
                    .get_by_no(net_no)
                    .map(|n| n.name.clone())
                    .unwrap_or_default();
                let items: Vec<_> = board
                    .items()
                    .filter(|(_, i)| i.base.contains_net(net_no))
                    .collect();
                let connected = board.net_is_completely_connected(net_no);
                println!(
                    "net {net_no} {name}: {} items, connected={connected}",
                    items.len()
                );
            }
        }
        Some(name) => {
            let net_no = board
                .rules
                .nets
                .get_by_name(&name)
                .first()
                .map(|n| n.net_number)
                .expect("net not found");
            for (id, item) in board.items() {
                if !item.base.contains_net(net_no) {
                    continue;
                }
                println!(
                    "item {id:?} kind {:?} component {} shapes:",
                    std::mem::discriminant(&item.kind),
                    item.base.component_no
                );
                for (shape, layer) in item.tile_shapes(&board.padstacks) {
                    let layer = *layer;
                    println!(
                        "  layer {layer}: dim {} bbox {:?}",
                        shape.dimension(),
                        shape.bounding_box()
                    );
                    // replicate complete_shape's restrain loop step by step
                    use freerouting::autoroute::room_completion::{restrain_shape, IncompleteRoom};
                    let mut pieces = vec![IncompleteRoom {
                        shape: freerouting::geometry::planar::TileShape::Box(
                            board.bounding_box().offset(1000.0),
                        ),
                        layer,
                        contained_shape: shape.clone(),
                    }];
                    for obst_id in board.overlapping_items(&pieces[0].shape, Some(layer)) {
                        let Some(obst) = board.get_item(obst_id) else {
                            continue;
                        };
                        if obst.base.contains_net(net_no) {
                            continue;
                        }
                        for (oshape, olayer) in obst.tile_shapes(&board.padstacks) {
                            if *olayer != layer {
                                continue;
                            }
                            let before = pieces.len();
                            let mut next = Vec::new();
                            for piece in pieces {
                                if piece.shape.intersection(oshape).dimension() == 2 {
                                    next.extend(restrain_shape(&piece, oshape));
                                } else {
                                    next.push(piece);
                                }
                            }
                            pieces = next;
                            if pieces.len() != before {
                                println!(
                                    "    obstacle {obst_id:?} bbox {:?}: pieces {before} -> {}",
                                    oshape.bounding_box(),
                                    pieces.len()
                                );
                            }
                            if pieces.is_empty() {
                                println!("    ROOM KILLED by {obst_id:?}");
                                return;
                            }
                        }
                    }
                    println!("    final pieces: {}", pieces.len());
                }
            }
        }
    }
}
