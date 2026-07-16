//! Port of `rules/NetClass.java`, `NetClasses.java` and
//! `DefaultItemClearanceClasses.java`.
//!
//! Java's object references (net class → via rule, net class → clearance
//! matrix) become indices; the matrix/layer data needed for queries is
//! copied in (the signal flags of the layer stack).

use crate::board::LayerStructure;

/// Item classes for default clearance-class lookup
/// (Java: `DefaultItemClearanceClasses.ItemClass`).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum ItemClass {
    None,
    Trace,
    Via,
    Pin,
    Smd,
    Area,
}

impl ItemClass {
    pub const COUNT: usize = 6;

    fn ordinal(self) -> usize {
        match self {
            ItemClass::None => 0,
            ItemClass::Trace => 1,
            ItemClass::Via => 2,
            ItemClass::Pin => 3,
            ItemClass::Smd => 4,
            ItemClass::Area => 5,
        }
    }
}

/// The default clearance class per item class.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct DefaultItemClearanceClasses {
    arr: [usize; ItemClass::COUNT],
}

impl Default for DefaultItemClearanceClasses {
    fn default() -> Self {
        let mut result = DefaultItemClearanceClasses {
            arr: [0; ItemClass::COUNT],
        };
        result.set_all(1);
        result
    }
}

impl DefaultItemClearanceClasses {
    pub fn get(&self, item_class: ItemClass) -> usize {
        self.arr[item_class.ordinal()]
    }

    pub fn set(&mut self, item_class: ItemClass, index: usize) {
        self.arr[item_class.ordinal()] = index;
    }

    /// Sets all item classes except `None` to `index`.
    pub fn set_all(&mut self, index: usize) {
        for v in self.arr.iter_mut().skip(1) {
            *v = index;
        }
    }
}

/// Index of a via rule in the board rules (via rules themselves are ported
/// later).
pub type ViaRuleId = usize;

/// Routing rules for the nets of one class.
#[derive(Debug, Clone, PartialEq)]
pub struct NetClass {
    name: String,
    /// trace half width per layer
    trace_half_width_arr: Vec<i32>,
    /// whether routing is active per layer
    active_routing_layer_arr: Vec<bool>,
    /// which layers of the board are signal layers
    layer_is_signal: Vec<bool>,
    pub default_item_clearance_classes: DefaultItemClearanceClasses,
    pub is_ignored_by_autorouter: bool,
    via_rule: Option<ViaRuleId>,
    trace_clearance_class: usize,
    shove_fixed: bool,
    pull_tight: bool,
    ignore_cycles_with_areas: bool,
    minimum_trace_length: f64,
    maximum_trace_length: f64,
}

impl NetClass {
    pub fn new(
        name: impl Into<String>,
        layer_structure: &LayerStructure,
        is_ignored_by_autorouter: bool,
    ) -> Self {
        let layer_count = layer_structure.layer_count();
        NetClass {
            name: name.into(),
            trace_half_width_arr: vec![0; layer_count],
            active_routing_layer_arr: layer_structure.arr.iter().map(|l| l.is_signal).collect(),
            layer_is_signal: layer_structure.arr.iter().map(|l| l.is_signal).collect(),
            default_item_clearance_classes: DefaultItemClearanceClasses::default(),
            is_ignored_by_autorouter,
            via_rule: None,
            trace_clearance_class: 0,
            shove_fixed: false,
            pull_tight: true,
            ignore_cycles_with_areas: false,
            minimum_trace_length: 0.0,
            maximum_trace_length: 0.0,
        }
    }

    pub fn get_name(&self) -> &str {
        &self.name
    }

    pub fn set_name(&mut self, name: impl Into<String>) {
        self.name = name.into();
    }

    pub fn layer_count(&self) -> usize {
        self.trace_half_width_arr.len()
    }

    /// Sets the trace half width on all layers.
    pub fn set_trace_half_width(&mut self, value: i32) {
        self.trace_half_width_arr.fill(value);
    }

    /// Sets the trace half width on all inner layers.
    pub fn set_trace_half_width_on_inner(&mut self, value: i32) {
        let len = self.trace_half_width_arr.len();
        for v in &mut self.trace_half_width_arr[1..len.saturating_sub(1)] {
            *v = value;
        }
    }

    pub fn set_trace_half_width_on_layer(&mut self, layer: usize, value: i32) {
        self.trace_half_width_arr[layer] = value;
    }

    /// The trace half width on `layer` (0 for out-of-range, like Java).
    pub fn get_trace_half_width(&self, layer: usize) -> i32 {
        self.trace_half_width_arr.get(layer).copied().unwrap_or(0)
    }

    pub fn get_trace_clearance_class(&self) -> usize {
        self.trace_clearance_class
    }

    pub fn set_trace_clearance_class(&mut self, clearance_class_no: usize) {
        self.trace_clearance_class = clearance_class_no;
    }

    pub fn get_via_rule(&self) -> Option<ViaRuleId> {
        self.via_rule
    }

    pub fn set_via_rule(&mut self, via_rule: Option<ViaRuleId>) {
        self.via_rule = via_rule;
    }

    pub fn is_shove_fixed(&self) -> bool {
        self.shove_fixed
    }

    pub fn set_shove_fixed(&mut self, value: bool) {
        self.shove_fixed = value;
    }

    pub fn get_pull_tight(&self) -> bool {
        self.pull_tight
    }

    pub fn set_pull_tight(&mut self, value: bool) {
        self.pull_tight = value;
    }

    pub fn get_ignore_cycles_with_areas(&self) -> bool {
        self.ignore_cycles_with_areas
    }

    pub fn set_ignore_cycles_with_areas(&mut self, value: bool) {
        self.ignore_cycles_with_areas = value;
    }

    /// The minimum trace length; <= 0 means no restriction.
    pub fn get_minimum_trace_length(&self) -> f64 {
        self.minimum_trace_length
    }

    pub fn set_minimum_trace_length(&mut self, value: f64) {
        self.minimum_trace_length = value;
    }

    /// The maximum trace length; <= 0 means no restriction.
    pub fn get_maximum_trace_length(&self) -> f64 {
        self.maximum_trace_length
    }

    pub fn set_maximum_trace_length(&mut self, value: f64) {
        self.maximum_trace_length = value;
    }

    pub fn is_active_routing_layer(&self, layer_no: usize) -> bool {
        self.active_routing_layer_arr
            .get(layer_no)
            .copied()
            .unwrap_or(false)
    }

    pub fn set_active_routing_layer(&mut self, layer_no: usize, active: bool) {
        if let Some(v) = self.active_routing_layer_arr.get_mut(layer_no) {
            *v = active;
        }
    }

    pub fn set_all_layers_active(&mut self, value: bool) {
        self.active_routing_layer_arr.fill(value);
    }

    pub fn set_all_inner_layers_active(&mut self, value: bool) {
        let len = self.active_routing_layer_arr.len();
        for v in &mut self.active_routing_layer_arr[1..len.saturating_sub(1)] {
            *v = value;
        }
    }

    /// True if the trace width differs between signal layers.
    pub fn trace_width_is_layer_dependent(&self) -> bool {
        let compare_value = self.trace_half_width_arr[0];
        (1..self.trace_half_width_arr.len())
            .any(|i| self.layer_is_signal[i] && self.trace_half_width_arr[i] != compare_value)
    }

    /// True if the trace width differs between inner signal layers.
    pub fn trace_width_is_inner_layer_dependent(&self) -> bool {
        let len = self.trace_half_width_arr.len();
        if len <= 3 {
            return false;
        }
        let Some(first_inner) = (1..len).find(|&i| self.layer_is_signal[i]) else {
            return false;
        };
        if first_inner >= len - 1 {
            return false;
        }
        let compare_width = self.trace_half_width_arr[first_inner];
        (first_inner + 1..len - 1)
            .any(|i| self.layer_is_signal[i] && self.trace_half_width_arr[i] != compare_width)
    }
}

/// The array of net classes of the board (Java: `NetClasses`).
#[derive(Debug, Clone, Default)]
pub struct NetClasses {
    class_arr: Vec<NetClass>,
}

impl NetClasses {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn count(&self) -> usize {
        self.class_arr.len()
    }

    pub fn get(&self, index: usize) -> &NetClass {
        &self.class_arr[index]
    }

    pub fn get_mut(&mut self, index: usize) -> &mut NetClass {
        &mut self.class_arr[index]
    }

    pub fn get_by_name(&self, name: &str) -> Option<usize> {
        self.class_arr.iter().position(|c| c.get_name() == name)
    }

    /// Appends a new class with `name`; returns its index.
    pub fn append(
        &mut self,
        name: impl Into<String>,
        layer_structure: &LayerStructure,
        is_ignored_by_autorouter: bool,
    ) -> usize {
        self.class_arr.push(NetClass::new(
            name,
            layer_structure,
            is_ignored_by_autorouter,
        ));
        self.class_arr.len() - 1
    }

    /// Appends a new class with a generated name `classN`.
    pub fn append_with_generated_name(&mut self, layer_structure: &LayerStructure) -> usize {
        let mut index = 0;
        let name = loop {
            index += 1;
            let candidate = format!("class{index}");
            if self.get_by_name(&candidate).is_none() {
                break candidate;
            }
        };
        self.append(name, layer_structure, false)
    }

    /// Finds a class with all trace half widths equal to
    /// `trace_half_width`, the given clearance class and via rule.
    pub fn find(
        &self,
        trace_half_width: i32,
        trace_clearance_class: usize,
        via_rule: Option<ViaRuleId>,
    ) -> Option<usize> {
        self.class_arr.iter().position(|c| {
            c.get_trace_clearance_class() == trace_clearance_class
                && c.get_via_rule() == via_rule
                && (0..c.layer_count()).all(|i| c.get_trace_half_width(i) == trace_half_width)
        })
    }

    /// Finds a class matching the per-layer half widths, clearance class
    /// and via rule.
    pub fn find_with_widths(
        &self,
        trace_half_width_arr: &[i32],
        trace_clearance_class: usize,
        via_rule: Option<ViaRuleId>,
    ) -> Option<usize> {
        self.class_arr.iter().position(|c| {
            c.get_trace_clearance_class() == trace_clearance_class
                && c.get_via_rule() == via_rule
                && trace_half_width_arr.len() == c.layer_count()
                && (0..c.layer_count())
                    .all(|i| c.get_trace_half_width(i) == trace_half_width_arr[i])
        })
    }

    /// Removes the class at `index`. Note: indices of later classes shift.
    pub fn remove(&mut self, index: usize) -> bool {
        if index >= self.class_arr.len() {
            return false;
        }
        self.class_arr.remove(index);
        true
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::board::{Layer, LayerStructure};

    fn stack() -> LayerStructure {
        LayerStructure::new(vec![
            Layer::new("F.Cu", true),
            Layer::new("gnd", false),
            Layer::new("In1.Cu", true),
            Layer::new("B.Cu", true),
        ])
    }

    #[test]
    fn net_class_defaults_and_widths() {
        let stack = stack();
        let mut nc = NetClass::new("default", &stack, false);
        assert_eq!(nc.layer_count(), 4);
        // active layers follow the signal flags
        assert!(nc.is_active_routing_layer(0));
        assert!(!nc.is_active_routing_layer(1));
        assert!(nc.get_pull_tight());
        assert!(!nc.is_shove_fixed());

        nc.set_trace_half_width(100);
        assert!(!nc.trace_width_is_layer_dependent());
        nc.set_trace_half_width_on_layer(2, 150);
        assert!(nc.trace_width_is_layer_dependent());
        assert!(nc.trace_width_is_inner_layer_dependent() || nc.get_trace_half_width(2) == 150);
        // the non-signal layer does not affect layer dependence
        nc.set_trace_half_width(100);
        nc.set_trace_half_width_on_layer(1, 999);
        assert!(!nc.trace_width_is_layer_dependent());

        assert_eq!(nc.default_item_clearance_classes.get(ItemClass::Via), 1);
        nc.default_item_clearance_classes.set(ItemClass::Via, 2);
        assert_eq!(nc.default_item_clearance_classes.get(ItemClass::Via), 2);
        assert_eq!(nc.default_item_clearance_classes.get(ItemClass::None), 0);
    }

    #[test]
    fn net_classes_collection() {
        let stack = stack();
        let mut classes = NetClasses::new();
        let default = classes.append("default", &stack, false);
        classes.get_mut(default).set_trace_half_width(100);
        classes.get_mut(default).set_trace_clearance_class(1);

        let generated = classes.append_with_generated_name(&stack);
        assert_eq!(classes.get(generated).get_name(), "class1");
        assert_eq!(classes.count(), 2);
        assert_eq!(classes.get_by_name("default"), Some(default));

        assert_eq!(classes.find(100, 1, None), Some(default));
        assert_eq!(classes.find(100, 2, None), None);
        assert_eq!(
            classes.find_with_widths(&[100, 100, 100, 100], 1, None),
            Some(default)
        );
        assert_eq!(
            classes.find_with_widths(&[100, 100, 150, 100], 1, None),
            None
        );
        assert!(classes.remove(generated));
        assert_eq!(classes.count(), 1);
    }
}
