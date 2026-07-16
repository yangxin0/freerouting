//! Port of the non-GUI core of `interactive/RatsNest.java`: the airlines
//! of the incomplete nets — for every pair of unconnected components,
//! the closest item pair with their connection points.

use crate::autoroute::batch::net_components;
use crate::board::basic_board::BasicBoard;

/// A candidate MST edge between two connected components: (distance,
/// component i, component j, closest point in i, closest point in j).
type CompEdge = (f64, usize, usize, (f64, f64), (f64, f64));

/// One airline (Java: `RatsNest.AirLine`).
#[derive(Debug, Clone)]
pub struct AirLine {
    pub net_no: i32,
    pub net_name: String,
    pub from: (f64, f64),
    pub to: (f64, f64),
}

/// A representative point of an item for airline computation: via/pin
/// centers, both trace endpoints, area centroids (Java: the corners the
/// Delaunay triangulation stores per NetItem).
fn item_points(board: &BasicBoard, id: crate::board::basic_board::ItemId) -> Vec<(f64, f64)> {
    use crate::board::ItemKind;
    let Some(item) = board.get_item(id) else {
        return Vec::new();
    };
    match &item.kind {
        ItemKind::Via(v) => vec![(v.center.x as f64, v.center.y as f64)],
        ItemKind::PolylineTrace(t) => {
            let f = t.polyline.corner_approx(0);
            let l = t.polyline.corner_approx(t.polyline.corner_count() - 1);
            vec![(f.x, f.y), (l.x, l.y)]
        }
        ItemKind::ObstacleArea(_) => {
            let bb = item.bounding_box(&board.padstacks);
            vec![(
                (bb.ll.x as f64 + bb.ur.x as f64) / 2.0,
                (bb.ll.y as f64 + bb.ur.y as f64) / 2.0,
            )]
        }
    }
}

/// The airlines of all incomplete nets, exactly Java's
/// `NetIncompletes` semantics: Kruskal's MST over the net items'
/// representative points, edges ascending by length, one airline per
/// edge joining two different connected sets, endpoints at the actual
/// nearest item points. (Java prunes the candidate edges with a
/// Delaunay triangulation before Kruskal; the triangulation always
/// contains the Euclidean MST, so the complete graph yields the same
/// airlines at ratsnest sizes — the 975-line triangulation is a pure
/// performance device and is not ported.)
pub fn ratsnest(board: &BasicBoard) -> Vec<AirLine> {
    let mut result = Vec::new();
    for net_no in 1..=board.rules.nets.max_net_no() {
        if board.net_is_completely_connected(net_no) {
            continue;
        }
        let net_name = board
            .rules
            .nets
            .get_by_no(net_no)
            .map(|n| n.name.clone())
            .unwrap_or_default();
        let components = net_components(board, net_no);
        if components.len() < 2 {
            continue;
        }
        // item points grouped by connected-set index
        let mut by_comp: Vec<Vec<(f64, f64)>> = vec![Vec::new(); components.len()];
        for (set_idx, comp) in components.iter().enumerate() {
            for &id in comp {
                for pt in item_points(board, id) {
                    by_comp[set_idx].push(pt);
                }
            }
        }
        // The airlines form a minimum spanning tree over the connected
        // COMPONENTS, where the weight between two components is their closest
        // point-pair distance. Instead of materializing all O(P^2) point-pair
        // edges, store only the C^2 component-pair edges (C = number of
        // disconnected pieces): scan point pairs once to find each component
        // pair's closest points, then run Kruskal over those. This gives O(C^2)
        // stored edges — a large win for the common case of a mostly-routed net
        // (few components). NOTE it is NOT a general O(P) method: a fully
        // unrouted net has C = P (every pin its own component), so both the
        // stored edges and the point-pair scan remain O(P^2) in the worst case.
        // A true O(P log P) ratsnest needs the Delaunay pruning Java uses, which
        // is not ported.
        let c = components.len();
        let mut comp_edges: Vec<CompEdge> = Vec::new();
        for i in 0..c {
            for j in (i + 1)..c {
                type Best = Option<(f64, (f64, f64), (f64, f64))>;
                let mut best: Best = None;
                for &pa in &by_comp[i] {
                    for &pb in &by_comp[j] {
                        let d = (pa.0 - pb.0).hypot(pa.1 - pb.1);
                        if best.as_ref().is_none_or(|(bd, _, _)| d < *bd) {
                            best = Some((d, pa, pb));
                        }
                    }
                }
                if let Some((d, from, to)) = best {
                    comp_edges.push((d, i, j, from, to));
                }
            }
        }
        comp_edges.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap_or(std::cmp::Ordering::Equal));
        // union-find over the connected sets
        let mut parent: Vec<usize> = (0..c).collect();
        fn find(parent: &mut [usize], mut x: usize) -> usize {
            while parent[x] != x {
                parent[x] = parent[parent[x]];
                x = parent[x];
            }
            x
        }
        for (_, i, j, from, to) in comp_edges {
            let (a, b) = (find(&mut parent, i), find(&mut parent, j));
            if a == b {
                continue;
            }
            parent[a] = b;
            result.push(AirLine {
                net_no,
                net_name: net_name.clone(),
                from,
                to,
            });
        }
    }
    result
}

/// Serializes the airlines as JSON.
pub fn ratsnest_json(board: &BasicBoard) -> String {
    let scale = 1.0 / (board.resolution.max(1) as f64 * 1000.0);
    let lines = ratsnest(board);
    let mut out = String::from("{\n  \"airlines\": [\n");
    for (i, l) in lines.iter().enumerate() {
        let comma = if i + 1 < lines.len() { "," } else { "" };
        out.push_str(&format!(
            "    {{\"net\": \"{}\", \"from\": [{:.4}, {:.4}], \"to\": [{:.4}, {:.4}]}}{comma}\n",
            l.net_name.replace('"', "'"),
            l.from.0 * scale,
            -l.from.1 * scale,
            l.to.0 * scale,
            -l.to.1 * scale,
        ));
    }
    out.push_str("  ]\n}\n");
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::board::{Layer, LayerStructure};
    use crate::core::Padstacks;
    use crate::geometry::planar::{IntBox, IntPoint, TileShape};
    use crate::rules::{BoardRules, ClearanceMatrix};

    #[test]
    fn airlines_span_the_components() {
        let stack = LayerStructure::new(vec![Layer::new("F.Cu", true)]);
        let matrix = ClearanceMatrix::get_default_instance(stack.clone(), 200);
        let mut rules = BoardRules::new(stack.clone(), matrix);
        rules.get_default_net_class();
        rules.nets.add("N1", 1, false);
        let mut padstacks = Padstacks::new(1);
        padstacks.add_shape_on_layers(
            TileShape::Box(IntBox::from_coords(-300, -300, 300, 300)),
            0,
            0,
        );
        let mut board = BasicBoard::new(stack, rules, padstacks);
        for (x, y) in [(0, 0), (10000, 0), (0, 10000)] {
            let p = board.insert_via(1, IntPoint::new(x, y), vec![1], 1, false);
            board.set_component_no(p, 1);
        }
        let lines = ratsnest(&board);
        assert_eq!(lines.len(), 2, "3 components need 2 airlines");
        assert!(ratsnest_json(&board).contains("\"net\": \"N1\""));
    }
}
