//! Prints the items, shapes and connectivity of one net (by name).
use freerouting::io::import_dsn;

fn main() {
    let path = std::env::args()
        .nth(1)
        .expect("usage: net_probe <dsn> <net-name>");
    let name = std::env::args().nth(2).expect("net name");
    let content = std::fs::read_to_string(&path).expect("read failed");
    let content = match content.find("  (wiring") {
        Some(pos) => format!("{})", &content[..pos]),
        None => content,
    };
    let mut board = import_dsn(&content).expect("import failed");
    if std::env::args().any(|a| a == "--route") {
        use freerouting::autoroute::{batch_route_passes_with_time_limit, BatchRequest};
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
            via_clearance_class: 0,
            via_attach_allowed: false,
            via_cost: 50_000.0,
            max_expansions: 100_000,
            ripup_penalty: 0.0,
            deadline: None,
        };
        let limit = freerouting::datastructures::TimeLimit::new(30_000);
        batch_route_passes_with_time_limit(&mut board, &request, 99, Some(&limit));
    }
    let board = board;
    let net_no = (1..=board.rules.nets.max_net_no())
        .find(|&n| {
            board
                .rules
                .nets
                .get_by_no(n)
                .is_some_and(|net| net.name == name)
        })
        .expect("net not found");
    println!("net {net_no} = {name}");
    let items: Vec<_> = board
        .items()
        .filter(|(_, it)| it.base.contains_net(net_no))
        .map(|(id, it)| (*id, it.clone()))
        .collect();
    for (id, it) in &items {
        let corners = match &it.kind {
            freerouting::board::ItemKind::PolylineTrace(t) => {
                format!(
                    "ends {:?} / {:?} layer {}",
                    t.first_corner(),
                    t.last_corner(),
                    t.layer
                )
            }
            _ => String::new(),
        };
        println!(
            "item {id}: kind {:?} connectable {} component {} shapes {:?} {corners}",
            std::mem::discriminant(&it.kind),
            it.is_connectable(),
            it.base.component_no,
            it.tile_shapes(&board.padstacks)
                .iter()
                .map(|(s, l)| (s.bounding_box(), *l))
                .collect::<Vec<_>>()
        );
        println!("  contacts: {:?}", board.get_normal_contacts(*id));
    }
    // connectivity
    for (id, _) in &items {
        let set = board.get_connected_set(*id, net_no);
        println!("connected set of {id}: {set:?}");
    }
}
