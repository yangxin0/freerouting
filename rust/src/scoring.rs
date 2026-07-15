//! Port of `core/scoring` (`BoardStatistics` + the score formula) and the
//! default `ScoringSettings`: the 0..1000 normalized quality score the
//! Java router logs after every pass ("score of 999.99").

use crate::board::basic_board::BasicBoard;
use crate::board::ItemKind;

/// The scoring weights (Java: `ScoringSettings` with the defaults of
/// `DefaultSettings`).
#[derive(Debug, Clone, Copy)]
pub struct ScoringSettings {
    pub unrouted_net_penalty: f64,
    pub clearance_violation_penalty: f64,
    pub bend_penalty: f64,
    pub trace_cost: f64,
    pub via_costs: f64,
}

impl Default for ScoringSettings {
    fn default() -> Self {
        ScoringSettings {
            unrouted_net_penalty: 5_000_000.0,
            clearance_violation_penalty: 1_000_000.0,
            bend_penalty: 10.0,
            trace_cost: 1.0,
            via_costs: 50.0,
        }
    }
}

/// The board statistics feeding the score (Java: `BoardStatistics`).
#[derive(Debug, Default)]
pub struct BoardStatistics {
    /// The number of incomplete nets.
    pub incomplete_count: usize,
    /// The number of nets with at least one connection to make.
    pub maximum_count: usize,
    pub clearance_violations: usize,
    /// Trace corners beyond the two endpoints, summed over all traces.
    pub bends: usize,
    pub via_count: usize,
    pub trace_count: usize,
    /// Total trace length in millimetres (Java normalizes to mm so the
    /// cost term is resolution-independent).
    pub total_length_mm: f64,
}

impl BoardStatistics {
    pub fn collect(board: &BasicBoard) -> Self {
        let mut stats = BoardStatistics::default();
        for net_no in 1..=board.rules.nets.max_net_no() {
            let connectable = board
                .items()
                .filter(|(_, it)| it.base.contains_net(net_no) && it.is_connectable())
                .count();
            if connectable >= 2 {
                stats.maximum_count += 1;
                if !board.net_is_completely_connected(net_no) {
                    stats.incomplete_count += 1;
                }
            }
        }
        for (_, item) in board.items() {
            if item.base.component_no != 0 || item.base.net_count() == 0 {
                continue;
            }
            match &item.kind {
                ItemKind::Via(_) => stats.via_count += 1,
                ItemKind::PolylineTrace(t) => {
                    stats.trace_count += 1;
                    stats.bends += t.corner_count().saturating_sub(2);
                    stats.total_length_mm +=
                        t.get_length() / (board.resolution.max(1) as f64 * 1000.0);
                }
                _ => {}
            }
        }
        stats.clearance_violations = crate::drc::check_board(board).violations.len();
        stats
    }

    /// Java: `calculateScore` — maximum minus penalties minus costs.
    pub fn score(&self, s: &ScoringSettings) -> f64 {
        let maximum = self.maximum_count as f64 * s.unrouted_net_penalty;
        let penalties = self.incomplete_count as f64 * s.unrouted_net_penalty
            + self.clearance_violations as f64 * s.clearance_violation_penalty
            + self.bends as f64 * s.bend_penalty;
        let costs =
            self.total_length_mm * s.trace_cost + self.via_count as f64 * s.via_costs;
        maximum - penalties - costs
    }

    /// Java: `getNormalizedScore` — 0..1000, higher is better; 0 when the
    /// board has no connections (guards NaN propagation, like Java).
    pub fn normalized_score(&self, s: &ScoringSettings) -> f64 {
        let maximum = self.maximum_count as f64 * s.unrouted_net_penalty;
        if maximum <= 0.0 {
            return 0.0;
        }
        (self.score(s) / maximum).max(0.0) * 1000.0
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::board::{Layer, LayerStructure};
    use crate::core::Padstacks;
    use crate::geometry::planar::{IntBox, IntPoint, Polyline, TileShape};
    use crate::rules::{BoardRules, ClearanceMatrix};

    #[test]
    fn score_reflects_completion_and_costs() {
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
        let a = board.insert_via(1, IntPoint::new(0, 0), vec![1], 1, false);
        let b = board.insert_via(1, IntPoint::new(10000, 0), vec![1], 1, false);
        board.set_component_no(a, 1);
        board.set_component_no(b, 1);
        let settings = ScoringSettings::default();
        // unrouted: penalty dominates → score near 0
        let unrouted = BoardStatistics::collect(&board);
        assert_eq!(unrouted.incomplete_count, 1);
        assert!(unrouted.normalized_score(&settings) < 1.0);
        // routed: near-1000
        board.insert_trace(
            Polyline::from_int_points(&[IntPoint::new(0, 0), IntPoint::new(10000, 0)]),
            0,
            100,
            vec![1],
            1,
        );
        let routed = BoardStatistics::collect(&board);
        assert_eq!(routed.incomplete_count, 0);
        let score = routed.normalized_score(&settings);
        assert!(score > 999.0 && score <= 1000.0, "score {score}");
    }
}
