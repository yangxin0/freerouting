//! Port of `rules/BoardRules.java`.
//!
//! The aggregate of all routing rules of a board: clearance matrix, nets,
//! via infos and rules, and net classes. Object references become indices
//! (net class 0 is the default class; via rule 0 the default rule).
//!
//! The two clearance-class maintenance methods that touch board items in
//! Java (`change_clearance_class_no`, `remove_clearance_class`) operate on
//! the rules side here; the board layer is responsible for checking and
//! renumbering its items around them.

use crate::board::{AngleRestriction, LayerStructure};
use crate::core::Padstacks;
use crate::rules::net_class::{ItemClass, NetClasses};
use crate::rules::via_rule::{ViaInfos, ViaRule};
use crate::rules::{ClearanceMatrix, Nets};

const ITEM_CLASSES: [ItemClass; ItemClass::COUNT] = [
    ItemClass::None,
    ItemClass::Trace,
    ItemClass::Via,
    ItemClass::Pin,
    ItemClass::Smd,
    ItemClass::Area,
];

#[derive(Debug, Clone)]
pub struct BoardRules {
    pub clearance_matrix: ClearanceMatrix,
    pub nets: Nets,
    pub via_infos: ViaInfos,
    pub via_rules: Vec<ViaRule>,
    pub net_classes: NetClasses,
    layer_structure: LayerStructure,
    trace_angle_restriction: AngleRestriction,
    /// If true, the router ignores conduction areas.
    ignore_conduction: bool,
    /// The smallest of all default trace half widths.
    min_trace_half_width: i32,
    /// The biggest of all default trace half widths.
    max_trace_half_width: i32,
    /// Minimum distance of the pad border to the first turn of a connected
    /// trace for pins with restricted exit directions (<= 0: no
    /// restriction).
    pin_edge_to_turn_dist: f64,
    use_slow_autoroute_algorithm: bool,
    /// Same-net clearances by item-class pair, from DSN `*_same_net` typed
    /// clearance rules (e.g. `via_via_same_net`). Java parses these but never
    /// applies them; the DRC uses them to require spacing between same-net
    /// drill items. Both orderings of a pair are stored.
    same_net_clearance: std::collections::HashMap<(ItemClass, ItemClass), i32>,
}

impl BoardRules {
    pub fn new(layer_structure: LayerStructure, clearance_matrix: ClearanceMatrix) -> Self {
        BoardRules {
            clearance_matrix,
            nets: Nets::new(),
            via_infos: ViaInfos::new(),
            via_rules: Vec::new(),
            net_classes: NetClasses::new(),
            layer_structure,
            trace_angle_restriction: AngleRestriction::FortyfiveDegree,
            ignore_conduction: true,
            min_trace_half_width: 100_000,
            max_trace_half_width: 100,
            pin_edge_to_turn_dist: 0.0,
            use_slow_autoroute_algorithm: false,
            same_net_clearance: std::collections::HashMap::new(),
        }
    }

    /// Records a same-net clearance between two item classes (both orderings).
    pub fn set_same_net_clearance(&mut self, a: ItemClass, b: ItemClass, value: i32) {
        self.same_net_clearance.insert((a, b), value);
        self.same_net_clearance.insert((b, a), value);
    }

    /// The same-net clearance required between two item classes, if any.
    pub fn get_same_net_clearance(&self, a: ItemClass, b: ItemClass) -> Option<i32> {
        self.same_net_clearance.get(&(a, b)).copied()
    }

    /// The default item clearance class.
    pub fn default_clearance_class() -> usize {
        1
    }

    /// The clearance class for items with no clearances.
    pub fn clearance_class_none() -> usize {
        0
    }

    pub fn layer_structure(&self) -> &LayerStructure {
        &self.layer_structure
    }

    /// The trace half width used for routing the given net on `layer`.
    pub fn get_trace_half_width(&self, net_no: i32, layer: usize) -> i32 {
        let Some(net) = self.nets.get_by_no(net_no) else {
            return 0;
        };
        self.net_classes
            .get(net.get_class())
            .get_trace_half_width(layer)
    }

    /// The trace clearance class of `net_no`'s net class (Java:
    /// `NetClass.get_trace_clearance_class`, used to build `AutorouteControl`).
    /// Falls back to the default clearance class when the net is unknown.
    pub fn get_trace_clearance_class(&self, net_no: i32) -> usize {
        let Some(net) = self.nets.get_by_no(net_no) else {
            return Self::default_clearance_class();
        };
        self.net_classes
            .get(net.get_class())
            .get_trace_clearance_class()
    }

    /// The clearance class an item of the given `ItemClass` (Pin, Via, Smd,
    /// Area, Trace) belonging to `net_no` should use, read from the net class's
    /// `default_item_clearance_classes` (Java `NetClass.get_item_clearance_...`
    /// via `Network.insert_component`/`insert_...`). Unknown nets fall back to
    /// the default net class (index 0).
    pub fn item_clearance_class_for(&self, net_no: i32, item_class: ItemClass) -> usize {
        let class_idx = self
            .nets
            .get_by_no(net_no)
            .map(|n| n.get_class())
            .unwrap_or(0);
        self.net_classes
            .get(class_idx)
            .default_item_clearance_classes
            .get(item_class)
    }

    /// Ensures net class `class_idx` requires clearance `value`: gives it a
    /// dedicated clearance-matrix class (created on first use, keyed by name),
    /// sets that class's clearance to every other class to the maximum of
    /// `value` and the existing entry (Java's cross-class max semantics), sets
    /// its self-clearance to exactly `value`, and points the class's trace and
    /// item clearance classes at it. Used when reading `.rules` files so a
    /// named class's clearance is applied rather than silently dropped.
    pub fn ensure_net_class_clearance(&mut self, class_idx: usize, value: i32) {
        if class_idx >= self.net_classes.count() {
            return;
        }
        let name = format!("rules_cl::{class_idx}::{value}");
        let cl_idx = match self.clearance_matrix.get_no(&name) {
            Some(i) => i,
            None => {
                self.clearance_matrix.append_class(&name);
                let Some(idx) = self.clearance_matrix.get_no(&name) else {
                    return;
                };
                let n = self.clearance_matrix.get_class_count();
                let layers = self.clearance_matrix.get_layer_count();
                for j in 1..n {
                    for layer in 0..layers {
                        let curr = self
                            .clearance_matrix
                            .get_value(idx, j, layer, false)
                            .max(value);
                        self.clearance_matrix.set_value(idx, j, layer, curr);
                        self.clearance_matrix.set_value(j, idx, layer, curr);
                    }
                }
                self.clearance_matrix
                    .set_value_on_all_layers(idx, idx, value);
                idx
            }
        };
        let class = self.net_classes.get_mut(class_idx);
        class.set_trace_clearance_class(cl_idx);
        class.default_item_clearance_classes.set_all(cl_idx);
    }

    /// True if the trace widths for `net_no` differ between layers.
    pub fn trace_widths_are_layer_dependent(&self, net_no: i32) -> bool {
        let compare_width = self.get_trace_half_width(net_no, 0);
        (1..self.layer_structure.layer_count())
            .any(|i| self.get_trace_half_width(net_no, i) != compare_width)
    }

    pub fn get_min_trace_half_width(&self) -> i32 {
        self.min_trace_half_width
    }

    pub fn get_max_trace_half_width(&self) -> i32 {
        self.max_trace_half_width
    }

    /// The index of the default net class, creating it if necessary.
    pub fn get_default_net_class(&mut self) -> usize {
        if self.net_classes.count() == 0 {
            self.create_default_net_class();
        }
        0
    }

    pub fn create_default_net_class(&mut self) {
        let default = self
            .net_classes
            .append("default", &self.layer_structure, false);
        let default_trace_half_width = 1500;
        let class = self.net_classes.get_mut(default);
        class.set_trace_half_width(default_trace_half_width);
        class.set_trace_clearance_class(1);
    }

    /// Changes the default trace half width used for routing on `layer`.
    pub fn set_default_trace_half_width_on_layer(&mut self, layer: usize, value: i32) {
        let default = self.get_default_net_class();
        self.net_classes
            .get_mut(default)
            .set_trace_half_width_on_layer(layer, value);
        self.min_trace_half_width = self.min_trace_half_width.min(value);
        self.max_trace_half_width = self.max_trace_half_width.max(value);
    }

    pub fn get_default_trace_half_width(&mut self, layer: usize) -> i32 {
        let default = self.get_default_net_class();
        self.net_classes.get(default).get_trace_half_width(layer)
    }

    /// Changes the default trace half width on all layers.
    pub fn set_default_trace_half_widths(&mut self, value: i32) {
        if value <= 0 {
            return;
        }
        let default = self.get_default_net_class();
        self.net_classes
            .get_mut(default)
            .set_trace_half_width(value);
        self.min_trace_half_width = self.min_trace_half_width.min(value);
        self.max_trace_half_width = self.max_trace_half_width.max(value);
    }

    /// Appends a new net class initialized from the default class, with a
    /// generated name (Java: `get_new_net_class()` / `append_net_class()`).
    pub fn append_net_class_with_generated_name(&mut self) -> usize {
        let default = self.get_default_net_class();
        let new_class = self
            .net_classes
            .append_with_generated_name(&self.layer_structure);
        self.init_class_from_default(new_class, default);
        new_class
    }

    /// Appends a new net class with `name` initialized from the default
    /// class; if a class with that name exists, it is returned instead.
    pub fn append_net_class(&mut self, name: &str) -> usize {
        if let Some(found) = self.net_classes.get_by_name(name) {
            return found;
        }
        let default = self.get_default_net_class();
        let new_class = self.net_classes.append(name, &self.layer_structure, false);
        let default_item_classes = self
            .net_classes
            .get(default)
            .default_item_clearance_classes
            .clone();
        self.net_classes
            .get_mut(new_class)
            .default_item_clearance_classes = default_item_classes;
        self.init_class_from_default(new_class, default);
        new_class
    }

    fn init_class_from_default(&mut self, new_class: usize, default: usize) {
        let via_rule = self.default_via_rule_id();
        let half_width = self.net_classes.get(default).get_trace_half_width(0);
        let clearance_class = self.net_classes.get(default).get_trace_clearance_class();
        let class = self.net_classes.get_mut(new_class);
        class.set_via_rule(via_rule);
        class.set_trace_half_width(half_width);
        class.set_trace_clearance_class(clearance_class);
    }

    /// The via padstack routing `net_no` should use, from the net's class
    /// via rule (the first via info of the rule).
    pub fn via_padstack_for_net(&self, net_no: i32) -> Option<usize> {
        let net = self.nets.get_by_no(net_no)?;
        let rule_id = self.net_classes.get(net.get_class()).get_via_rule()?;
        let rule = self.via_rules.get(rule_id)?;
        let via_info_id = *rule.vias().first()?;
        Some(self.via_infos.get(via_info_id).get_padstack())
    }

    /// The index of the default via rule, if any.
    pub fn default_via_rule_id(&self) -> Option<usize> {
        if self.via_rules.is_empty() {
            None
        } else {
            Some(0)
        }
    }

    pub fn get_via_rule(&self, name: &str) -> Option<usize> {
        self.via_rules.iter().position(|r| r.name == name)
    }

    /// Creates a default via rule for `net_class` containing all via infos
    /// with the class's default via clearance class; when several via
    /// infos share a layer range, the one with the smallest pad is kept.
    pub fn create_default_via_rule(&mut self, net_class: usize, name: &str, padstacks: &Padstacks) {
        if self.via_infos.count() == 0 {
            return;
        }
        let mut default_rule = ViaRule::new(name);
        let default_via_cl_class = self
            .net_classes
            .get(net_class)
            .default_item_clearance_classes
            .get(ItemClass::Via);
        for i in 0..self.via_infos.count() {
            let curr_via_info = self.via_infos.get(i);
            if curr_via_info.get_clearance_class() != default_via_cl_class {
                continue;
            }
            let Some(curr_padstack) = padstacks.get_by_no(curr_via_info.get_padstack()) else {
                continue;
            };
            let curr_from_layer = curr_padstack.from_layer();
            let curr_to_layer = curr_padstack.to_layer();
            if let Some(existing_via) = default_rule.get_layer_range(
                curr_from_layer,
                curr_to_layer,
                &self.via_infos,
                padstacks,
            ) {
                let new_width = curr_padstack
                    .get_shape(curr_from_layer)
                    .map(|s| s.max_width())
                    .unwrap_or(f64::MAX);
                let existing_width = padstacks
                    .get_by_no(self.via_infos.get(existing_via).get_padstack())
                    .and_then(|p| p.get_shape(curr_from_layer))
                    .map(|s| s.max_width())
                    .unwrap_or(f64::MAX);
                if new_width < existing_width {
                    // the via with the smallest pad shape is preferred
                    default_rule.remove_via(existing_via);
                    default_rule.append_via(i);
                }
            } else {
                default_rule.append_via(i);
            }
        }
        self.via_rules.push(default_rule);
        let rule_id = self.via_rules.len() - 1;
        self.net_classes
            .get_mut(net_class)
            .set_via_rule(Some(rule_id));
    }

    /// True if the clearance class `index` is referenced by any net class
    /// or via info (the board's items must be checked separately).
    pub fn clearance_class_in_use(&self, index: usize) -> bool {
        for i in 0..self.net_classes.count() {
            let net_class = self.net_classes.get(i);
            if net_class.get_trace_clearance_class() == index {
                return true;
            }
            if ITEM_CLASSES
                .iter()
                .any(|&c| net_class.default_item_clearance_classes.get(c) == index)
            {
                return true;
            }
        }
        (0..self.via_infos.count()).any(|i| self.via_infos.get(i).get_clearance_class() == index)
    }

    /// Changes the clearance class of all rules objects from `from_no` to
    /// `to_no` (board items are handled by the board).
    pub fn change_clearance_class_no(&mut self, from_no: usize, to_no: usize) {
        for i in 0..self.net_classes.count() {
            let net_class = self.net_classes.get_mut(i);
            if net_class.get_trace_clearance_class() == from_no {
                net_class.set_trace_clearance_class(to_no);
            }
            for &item_class in &ITEM_CLASSES {
                if net_class.default_item_clearance_classes.get(item_class) == from_no {
                    net_class
                        .default_item_clearance_classes
                        .set(item_class, to_no);
                }
            }
        }
        for i in 0..self.via_infos.count() {
            let via = self.via_infos.get_mut(i);
            if via.get_clearance_class() == from_no {
                via.set_clearance_class(to_no);
            }
        }
    }

    /// Removes the clearance class `index`, renumbering all higher rules
    /// references. Returns false (without changes) if the class is still
    /// in use by the rules; the caller must have verified the board items.
    pub fn remove_clearance_class(&mut self, index: usize) -> bool {
        if self.clearance_class_in_use(index) {
            return false;
        }
        for i in 0..self.net_classes.count() {
            let net_class = self.net_classes.get_mut(i);
            let curr = net_class.get_trace_clearance_class();
            if curr > index {
                net_class.set_trace_clearance_class(curr - 1);
            }
            for &item_class in &ITEM_CLASSES {
                let curr = net_class.default_item_clearance_classes.get(item_class);
                if curr > index {
                    net_class
                        .default_item_clearance_classes
                        .set(item_class, curr - 1);
                }
            }
        }
        for i in 0..self.via_infos.count() {
            let via = self.via_infos.get_mut(i);
            let curr = via.get_clearance_class();
            if curr > index {
                via.set_clearance_class(curr - 1);
            }
        }
        self.clearance_matrix.remove_class(index);
        true
    }

    pub fn get_pin_edge_to_turn_dist(&self) -> f64 {
        self.pin_edge_to_turn_dist
    }

    pub fn set_pin_edge_to_turn_dist(&mut self, value: f64) {
        self.pin_edge_to_turn_dist = value;
    }

    pub fn get_ignore_conduction(&self) -> bool {
        self.ignore_conduction
    }

    pub fn set_ignore_conduction(&mut self, value: bool) {
        self.ignore_conduction = value;
    }

    pub fn get_trace_angle_restriction(&self) -> AngleRestriction {
        self.trace_angle_restriction
    }

    pub fn set_trace_angle_restriction(&mut self, value: AngleRestriction) {
        self.trace_angle_restriction = value;
    }

    /// If true, Simplex shapes are always used in the autorouter; if
    /// false, boxes (90 degree) or octagons (45 degree) are used.
    pub fn get_use_slow_autoroute_algorithm(&self) -> bool {
        self.use_slow_autoroute_algorithm
    }

    pub fn set_use_slow_autoroute_algorithm(&mut self, value: bool) {
        self.use_slow_autoroute_algorithm = value;
    }

    /// The maximum diameter of the default via on its first and last
    /// layer.
    pub fn get_default_via_diameter(&self, padstacks: &Padstacks) -> f64 {
        let Some(rule_id) = self.default_via_rule_id() else {
            return 0.0;
        };
        let rule = &self.via_rules[rule_id];
        if rule.via_count() == 0 {
            return 0.0;
        }
        let via_info = self.via_infos.get(rule.get_via(0));
        let Some(padstack) = padstacks.get_by_no(via_info.get_padstack()) else {
            return 0.0;
        };
        let first = padstack
            .get_shape(padstack.from_layer())
            .map(|s| s.max_width())
            .unwrap_or(0.0);
        let last = padstack
            .get_shape(padstack.to_layer())
            .map(|s| s.max_width())
            .unwrap_or(0.0);
        first.max(last)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geometry::planar::{IntBox, TileShape};
    use crate::rules::ViaInfo;

    fn setup() -> (BoardRules, Padstacks) {
        let stack = LayerStructure::signal_layers(2);
        let matrix = ClearanceMatrix::get_default_instance(stack.clone(), 200);
        let rules = BoardRules::new(stack, matrix);
        let padstacks = Padstacks::new(2);
        (rules, padstacks)
    }

    fn box_shape(hw: i32) -> TileShape {
        TileShape::Box(IntBox::from_coords(-hw, -hw, hw, hw))
    }

    #[test]
    fn default_net_class_and_widths() {
        let (mut rules, _) = setup();
        let default = rules.get_default_net_class();
        assert_eq!(default, 0);
        assert_eq!(rules.net_classes.get(default).get_name(), "default");
        assert_eq!(rules.get_default_trace_half_width(0), 1500);

        rules.set_default_trace_half_widths(800);
        assert_eq!(rules.get_default_trace_half_width(1), 800);
        assert_eq!(rules.get_min_trace_half_width(), 800);
        rules.set_default_trace_half_width_on_layer(1, 900);
        assert_eq!(rules.get_max_trace_half_width(), 900);

        // net trace widths route through the net's class
        let net = rules.nets.add("GND", 1, false);
        assert_eq!(rules.get_trace_half_width(net, 0), 800);
        assert!(rules.trace_widths_are_layer_dependent(net));
    }

    #[test]
    fn append_net_class_inherits_default() {
        let (mut rules, _) = setup();
        rules.get_default_net_class();
        rules.net_classes.get_mut(0).set_trace_clearance_class(1);
        let power = rules.append_net_class("power");
        assert_eq!(rules.net_classes.get(power).get_trace_half_width(0), 1500);
        assert_eq!(rules.net_classes.get(power).get_trace_clearance_class(), 1);
        // existing name returns the existing class
        assert_eq!(rules.append_net_class("power"), power);
        let generated = rules.append_net_class_with_generated_name();
        assert_ne!(generated, power);
    }

    #[test]
    fn default_via_rule_prefers_smallest_pad() {
        let (mut rules, mut padstacks) = setup();
        let default = rules.get_default_net_class();
        let big = padstacks.add_shape_on_layers(box_shape(500), 0, 1);
        let small = padstacks.add_shape_on_layers(box_shape(300), 0, 1);
        rules
            .via_infos
            .add(ViaInfo::new("via_big", big, 1, false))
            .unwrap();
        let small_info = rules
            .via_infos
            .add(ViaInfo::new("via_small", small, 1, false))
            .unwrap();
        rules.create_default_via_rule(default, "default_rule", &padstacks);

        let rule_id = rules.default_via_rule_id().unwrap();
        let rule = &rules.via_rules[rule_id];
        assert_eq!(rule.via_count(), 1);
        assert_eq!(rule.get_via(0), small_info);
        assert_eq!(rules.net_classes.get(default).get_via_rule(), Some(rule_id));
        assert!((rules.get_default_via_diameter(&padstacks) - 600.0).abs() < 1e-9);
        assert_eq!(rules.get_via_rule("default_rule"), Some(rule_id));
    }

    #[test]
    fn clearance_class_maintenance() {
        let (mut rules, _) = setup();
        rules.get_default_net_class();
        rules.clearance_matrix.append_class("extra");
        let extra = rules.clearance_matrix.get_no("extra").unwrap();
        // in use by nothing: removable
        assert!(!rules.clearance_class_in_use(extra));
        // reference it from a via info
        rules
            .via_infos
            .add(ViaInfo::new("v", 1, extra, false))
            .unwrap();
        assert!(rules.clearance_class_in_use(extra));
        assert!(!rules.remove_clearance_class(extra));
        // move the reference away and remove
        rules.change_clearance_class_no(extra, 1);
        assert!(!rules.clearance_class_in_use(extra));
        assert!(rules.remove_clearance_class(extra));
        assert_eq!(rules.clearance_matrix.get_class_count(), 2);
    }
}
