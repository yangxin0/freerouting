//! Port of the non-GUI core of `interactive/RatsNest.java`: the airlines
//! of the incomplete nets — for every pair of unconnected components,
//! the closest item pair with their connection points.

use crate::autoroute::batch::net_components;
use crate::board::basic_board::BasicBoard;

/// One airline (Java: `RatsNest.AirLine`).
#[derive(Debug, Clone)]
pub struct AirLine {
    pub net_no: i32,
    pub net_name: String,
    pub from: (f64, f64),
    pub to: (f64, f64),
}

/// The airlines of all incomplete nets (Java: `RatsNest`'s incompletes,
/// computed with the minimum-spanning chain over component centers).
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
        // component centers
        let centers: Vec<(f64, f64)> = components
            .iter()
            .map(|c| {
                let mut x = 0.0;
                let mut y = 0.0;
                let mut n: f64 = 0.0;
                for id in c {
                    if let Some(item) = board.get_item(*id) {
                        let bb = item.bounding_box(&board.padstacks);
                        x += (bb.ll.x as f64 + bb.ur.x as f64) / 2.0;
                        y += (bb.ll.y as f64 + bb.ur.y as f64) / 2.0;
                        n += 1.0;
                    }
                }
                (x / n.max(1.0), y / n.max(1.0))
            })
            .collect();
        // minimum-spanning chain: connect each unvisited component to the
        // nearest visited one (Prim's algorithm, like Java's incompletes)
        let mut visited = vec![false; centers.len()];
        visited[0] = true;
        for _ in 1..centers.len() {
            let mut best: Option<(f64, usize, usize)> = None;
            for (i, &vi) in visited.iter().enumerate() {
                if !vi {
                    continue;
                }
                for (j, &vj) in visited.iter().enumerate() {
                    if vj {
                        continue;
                    }
                    let d = (centers[i].0 - centers[j].0).hypot(centers[i].1 - centers[j].1);
                    if best.is_none_or(|(bd, _, _)| d < bd) {
                        best = Some((d, i, j));
                    }
                }
            }
            let Some((_, i, j)) = best else { break };
            visited[j] = true;
            result.push(AirLine {
                net_no,
                net_name: net_name.clone(),
                from: centers[i],
                to: centers[j],
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
