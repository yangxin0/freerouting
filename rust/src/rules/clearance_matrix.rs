//! Port of `rules/ClearanceMatrix.java`.
//!
//! An NxN matrix describing the spacing restrictions between N clearance
//! classes on a fixed set of layers. Values are kept even (rounded up),
//! and per-row/per-layer maxima are maintained for fast conservative
//! queries.

use crate::board::LayerStructure;

pub const CLEARANCE_SAFETY_MARGIN: i32 = 16;

#[derive(Debug, Clone, PartialEq, Eq)]
struct MatrixEntry {
    /// clearance value per layer
    layer: Vec<i32>,
}

impl MatrixEntry {
    fn new(layer_count: usize) -> Self {
        MatrixEntry {
            layer: vec![0; layer_count],
        }
    }

    fn is_layer_dependent(&self) -> bool {
        self.layer.iter().any(|&v| v != self.layer[0])
    }
}

#[derive(Debug, Clone)]
struct Row {
    name: String,
    column: Vec<MatrixEntry>,
    max_value: Vec<i32>,
}

impl Row {
    fn new(name: String, class_count: usize, layer_count: usize) -> Self {
        Row {
            name,
            column: (0..class_count)
                .map(|_| MatrixEntry::new(layer_count))
                .collect(),
            max_value: vec![0; layer_count],
        }
    }
}

#[derive(Debug, Clone)]
pub struct ClearanceMatrix {
    layer_structure: LayerStructure,
    /// maximum clearance value for each layer
    max_value_on_layer: Vec<i32>,
    rows: Vec<Row>,
}

impl ClearanceMatrix {
    /// Creates a matrix for the given clearance class names.
    pub fn new(layer_structure: LayerStructure, class_names: &[&str]) -> Self {
        let class_count = class_names.len().max(1);
        let layer_count = layer_structure.layer_count();
        let rows = (0..class_count)
            .map(|i| {
                let name = class_names.get(i).copied().unwrap_or("null").to_string();
                Row::new(name, class_count, layer_count)
            })
            .collect();
        ClearanceMatrix {
            max_value_on_layer: vec![0; layer_count],
            layer_structure,
            rows,
        }
    }

    /// Creates a matrix with the 2 clearance classes "null" and "default",
    /// initialized with `default_value`.
    pub fn get_default_instance(layer_structure: LayerStructure, default_value: i32) -> Self {
        let mut result = ClearanceMatrix::new(layer_structure, &["null", "default"]);
        result.set_default_value(default_value);
        result
    }

    /// The number of the clearance class with `name` (case-insensitive).
    pub fn get_no(&self, name: &str) -> Option<usize> {
        self.rows
            .iter()
            .position(|r| r.name.eq_ignore_ascii_case(name))
    }

    /// The name of the clearance class `cl_class`.
    pub fn get_name(&self, cl_class: usize) -> Option<&str> {
        self.rows.get(cl_class).map(|r| r.name.as_str())
    }

    pub fn get_class_count(&self) -> usize {
        self.rows.len()
    }

    pub fn get_layer_count(&self) -> usize {
        self.layer_structure.layer_count()
    }

    /// Sets the value of all clearance classes with number >= 1 to `value`
    /// on all layers.
    pub fn set_default_value(&mut self, value: i32) {
        for layer in 0..self.get_layer_count() {
            self.set_default_value_on_layer(layer, value);
        }
    }

    /// Sets the value of all clearance classes with number >= 1 to `value`
    /// on `layer`.
    pub fn set_default_value_on_layer(&mut self, layer: usize, value: i32) {
        for i in 1..self.get_class_count() {
            for j in 1..self.get_class_count() {
                self.set_value(i, j, layer, value);
            }
        }
    }

    /// Sets the matrix entry (i, j) to `value` on all layers.
    pub fn set_value_on_all_layers(&mut self, i: usize, j: usize, value: i32) {
        for layer in 0..self.get_layer_count() {
            self.set_value(i, j, layer, value);
        }
    }

    /// Sets the matrix entry (i, j) to `value` on all inner layers.
    pub fn set_inner_value(&mut self, i: usize, j: usize, value: i32) {
        for layer in 1..self.get_layer_count().saturating_sub(1) {
            self.set_value(i, j, layer, value);
        }
    }

    /// Sets the matrix entry (i, j) on `layer`. The value is clamped to be
    /// non-negative and rounded up to an even number (like Java).
    pub fn set_value(&mut self, i: usize, j: usize, layer: usize, value: i32) {
        let mut value = value.max(0);
        if value % 2 != 0 {
            if value == i32::MAX {
                value -= 1;
            } else {
                value += 1;
            }
        }
        let row = &mut self.rows[j];
        row.column[i].layer[layer] = value;
        row.max_value[layer] = row.max_value[layer].max(value);
        self.max_value_on_layer[layer] = self.max_value_on_layer[layer].max(value);
    }

    /// The required spacing of clearance classes `i` and `j` on `layer`
    /// (always even), optionally with the safety margin added. Returns 0
    /// for out-of-bounds requests, like Java.
    pub fn get_value(&self, i: usize, j: usize, layer: usize, add_safety_margin: bool) -> i32 {
        let Some(value) = self
            .rows
            .get(j)
            .and_then(|r| r.column.get(i))
            .and_then(|e| e.layer.get(layer))
        else {
            return 0;
        };
        if add_safety_margin {
            value + CLEARANCE_SAFETY_MARGIN
        } else {
            *value
        }
    }

    /// The maximal required spacing of clearance class `i` to all other
    /// classes on `layer` (indices clamped like Java).
    pub fn max_value_of_class(&self, i: usize, layer: usize) -> i32 {
        let i = i.min(self.get_class_count() - 1);
        let layer = layer.min(self.get_layer_count() - 1);
        self.rows[i].max_value[layer]
    }

    /// The maximal clearance value on `layer`.
    pub fn max_value(&self, layer: usize) -> i32 {
        let layer = layer.min(self.get_layer_count() - 1);
        self.max_value_on_layer[layer]
    }

    /// True if the entry (i, j) differs between layers.
    pub fn is_layer_dependent(&self, i: usize, j: usize) -> bool {
        self.rows[j].column[i].is_layer_dependent()
    }

    /// True if the entry (i, j) differs between inner layers.
    pub fn is_inner_layer_dependent(&self, i: usize, j: usize) -> bool {
        let layer_count = self.get_layer_count();
        if layer_count <= 2 {
            return false; // no inner layers
        }
        let entry = &self.rows[j].column[i];
        let compare_value = entry.layer[1];
        entry.layer[2..layer_count - 1]
            .iter()
            .any(|&v| v != compare_value)
    }

    /// The clearance compensation value of `clearance_class_no` on `layer`:
    /// half the clearance of the class to itself.
    pub fn clearance_compensation_value(&self, clearance_class_no: usize, layer: usize) -> i32 {
        (self.get_value(clearance_class_no, clearance_class_no, layer, false) + 1) / 2
    }

    /// Appends a new clearance class initialized with the values of the
    /// default class (index 1). Returns false if the name already exists.
    pub fn append_class(&mut self, class_name: &str) -> bool {
        if self.get_no(class_name).is_some() {
            return false;
        }
        let old_class_count = self.get_class_count();
        let layer_count = self.get_layer_count();
        for row in &mut self.rows {
            row.column.push(MatrixEntry::new(layer_count));
        }
        self.rows.push(Row::new(
            class_name.to_string(),
            old_class_count + 1,
            layer_count,
        ));
        // set the new matrix elements to default values
        for i in 0..old_class_count {
            for layer in 0..layer_count {
                let default_value = self.get_value(1, i, layer, false);
                self.set_value(old_class_count, i, layer, default_value);
                self.set_value(i, old_class_count, layer, default_value);
            }
        }
        for layer in 0..layer_count {
            let default_value = self.get_value(1, 1, layer, false);
            self.set_value(old_class_count, old_class_count, layer, default_value);
        }
        true
    }

    /// Removes the class with index `index` from the matrix.
    pub fn remove_class(&mut self, index: usize) {
        self.rows.remove(index);
        for row in &mut self.rows {
            row.column.remove(index);
        }
    }

    /// True if all clearance values of class `class_1` equal those of
    /// `class_2` (compared from column 1, like Java).
    pub fn is_equal(&self, class_1: usize, class_2: usize) -> bool {
        if class_1 == class_2 {
            return true;
        }
        if class_1 >= self.get_class_count() || class_2 >= self.get_class_count() {
            return false;
        }
        let row_1 = &self.rows[class_1];
        let row_2 = &self.rows[class_2];
        (1..self.get_class_count()).all(|i| row_1.column[i] == row_2.column[i])
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::board::LayerStructure;

    fn matrix() -> ClearanceMatrix {
        ClearanceMatrix::get_default_instance(LayerStructure::signal_layers(2), 200)
    }

    #[test]
    fn default_instance_values() {
        let m = matrix();
        assert_eq!(m.get_class_count(), 2);
        assert_eq!(m.get_no("default"), Some(1));
        assert_eq!(m.get_no("DEFAULT"), Some(1));
        assert_eq!(m.get_no("nope"), None);
        // class 0 ("null") has no clearance requirements
        assert_eq!(m.get_value(0, 1, 0, false), 0);
        assert_eq!(m.get_value(1, 1, 0, false), 200);
        assert_eq!(m.get_value(1, 1, 0, true), 200 + CLEARANCE_SAFETY_MARGIN);
        // out of bounds requests return 0
        assert_eq!(m.get_value(5, 1, 0, false), 0);
        assert_eq!(m.get_value(1, 1, 9, false), 0);
        assert_eq!(m.max_value(0), 200);
        assert_eq!(m.clearance_compensation_value(1, 0), 100);
    }

    #[test]
    fn values_are_rounded_to_even() {
        let mut m = matrix();
        m.set_value(1, 1, 0, 33);
        assert_eq!(m.get_value(1, 1, 0, false), 34);
        m.set_value(1, 1, 0, -5);
        assert_eq!(m.get_value(1, 1, 0, false), 0);
        m.set_value(1, 1, 0, i32::MAX);
        assert_eq!(m.get_value(1, 1, 0, false), i32::MAX - 1);
    }

    #[test]
    fn append_and_remove_class() {
        let mut m = matrix();
        assert!(m.append_class("power"));
        assert!(!m.append_class("power"));
        assert_eq!(m.get_class_count(), 3);
        let power = m.get_no("power").unwrap();
        // initialized from the default class
        assert_eq!(m.get_value(power, 1, 0, false), 200);
        assert_eq!(m.get_value(1, power, 1, false), 200);
        assert_eq!(m.get_value(power, power, 0, false), 200);
        assert!(m.is_equal(1, power));
        // change it and verify per-layer dependence tracking
        m.set_value(power, power, 0, 400);
        assert!(!m.is_equal(1, power));
        assert!(m.is_layer_dependent(power, power));
        assert_eq!(m.max_value_of_class(power, 0), 400);
        assert_eq!(m.max_value(0), 400);

        m.remove_class(power);
        assert_eq!(m.get_class_count(), 2);
        assert_eq!(m.get_no("power"), None);
        assert_eq!(m.get_value(1, 1, 0, false), 200);
    }
}
