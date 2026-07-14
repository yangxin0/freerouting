//! Port of `board/Layer.java` and `board/LayerStructure.java`.

/// A board layer: a name plus whether it is a signal layer (routing
/// allowed) or not (e.g. power planes).
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Layer {
    pub name: String,
    pub is_signal: bool,
}

impl Layer {
    pub fn new(name: impl Into<String>, is_signal: bool) -> Self {
        Layer {
            name: name.into(),
            is_signal,
        }
    }
}

/// The layer stack of the board.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct LayerStructure {
    pub arr: Vec<Layer>,
}

impl LayerStructure {
    pub fn new(layers: Vec<Layer>) -> Self {
        LayerStructure { arr: layers }
    }

    /// A simple signal-layer-only stack with generated names, useful for
    /// tests and defaults.
    pub fn signal_layers(count: usize) -> Self {
        LayerStructure {
            arr: (0..count)
                .map(|i| Layer::new(format!("layer_{i}"), true))
                .collect(),
        }
    }

    pub fn layer_count(&self) -> usize {
        self.arr.len()
    }

    /// The index of the layer with `name`, if present.
    pub fn get_no(&self, name: &str) -> Option<usize> {
        self.arr.iter().position(|l| l.name == name)
    }

    /// The number of signal layers.
    pub fn signal_layer_count(&self) -> usize {
        self.arr.iter().filter(|l| l.is_signal).count()
    }

    /// The `no`-th signal layer.
    pub fn get_signal_layer(&self, no: usize) -> Option<&Layer> {
        self.arr.iter().filter(|l| l.is_signal).nth(no)
    }

    /// The signal layer number of the layer at `layer_no` (counting only
    /// signal layers below it).
    pub fn get_signal_layer_no(&self, layer_no: usize) -> usize {
        self.arr[..layer_no].iter().filter(|l| l.is_signal).count()
    }

    /// The absolute layer number of the `signal_layer_no`-th signal layer.
    pub fn get_layer_no(&self, signal_layer_no: usize) -> Option<usize> {
        self.arr
            .iter()
            .enumerate()
            .filter(|(_, l)| l.is_signal)
            .nth(signal_layer_no)
            .map(|(i, _)| i)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn signal_layer_mapping() {
        let stack = LayerStructure::new(vec![
            Layer::new("F.Cu", true),
            Layer::new("gnd", false),
            Layer::new("In1.Cu", true),
            Layer::new("B.Cu", true),
        ]);
        assert_eq!(stack.layer_count(), 4);
        assert_eq!(stack.signal_layer_count(), 3);
        assert_eq!(stack.get_no("In1.Cu"), Some(2));
        assert_eq!(stack.get_no("nope"), None);
        assert_eq!(stack.get_signal_layer(1).unwrap().name, "In1.Cu");
        assert_eq!(stack.get_signal_layer_no(2), 1);
        assert_eq!(stack.get_layer_no(1), Some(2));
        assert_eq!(stack.get_layer_no(2), Some(3));
        assert_eq!(stack.get_layer_no(3), None);
    }
}
