//! Port of `rules/ViaInfo.java`, `ViaInfos.java` and `ViaRule.java`.
//!
//! Java's ViaInfo object references become indices into [`ViaInfos`];
//! padstacks are referenced by their 1-based padstack number.

use crate::core::Padstacks;

/// A combination of via padstack, via clearance class and
/// attach-to-SMD-allowed, used in interactive and automatic routing.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ViaInfo {
    name: String,
    /// 1-based padstack number in the board's padstack library.
    padstack: usize,
    clearance_class: usize,
    attach_smd_allowed: bool,
}

impl ViaInfo {
    pub fn new(
        name: impl Into<String>,
        padstack: usize,
        clearance_class: usize,
        attach_smd_allowed: bool,
    ) -> Self {
        ViaInfo {
            name: name.into(),
            padstack,
            clearance_class,
            attach_smd_allowed,
        }
    }

    pub fn get_name(&self) -> &str {
        &self.name
    }

    pub fn set_name(&mut self, name: impl Into<String>) {
        self.name = name.into();
    }

    pub fn get_padstack(&self) -> usize {
        self.padstack
    }

    pub fn set_padstack(&mut self, padstack: usize) {
        self.padstack = padstack;
    }

    pub fn get_clearance_class(&self) -> usize {
        self.clearance_class
    }

    pub fn set_clearance_class(&mut self, clearance_class: usize) {
        self.clearance_class = clearance_class;
    }

    pub fn attach_smd_allowed(&self) -> bool {
        self.attach_smd_allowed
    }

    pub fn set_attach_smd_allowed(&mut self, value: bool) {
        self.attach_smd_allowed = value;
    }
}

/// Index of a [`ViaInfo`] in [`ViaInfos`].
pub type ViaInfoId = usize;

/// The list of via infos usable in interactive and automatic routing.
#[derive(Debug, Clone, Default)]
pub struct ViaInfos {
    list: Vec<ViaInfo>,
}

impl ViaInfos {
    pub fn new() -> Self {
        Self::default()
    }

    /// Adds a via info; returns `None` if the name already exists.
    pub fn add(&mut self, via_info: ViaInfo) -> Option<ViaInfoId> {
        if self.name_exists(via_info.get_name()) {
            return None;
        }
        self.list.push(via_info);
        Some(self.list.len() - 1)
    }

    pub fn count(&self) -> usize {
        self.list.len()
    }

    pub fn get(&self, no: ViaInfoId) -> &ViaInfo {
        &self.list[no]
    }

    pub fn get_mut(&mut self, no: ViaInfoId) -> &mut ViaInfo {
        &mut self.list[no]
    }

    pub fn get_by_name(&self, name: &str) -> Option<ViaInfoId> {
        self.list.iter().position(|v| v.get_name() == name)
    }

    pub fn name_exists(&self, name: &str) -> bool {
        self.get_by_name(name).is_some()
    }
}

/// An ordered list of vias used for routing; vias at the beginning are
/// preferred.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ViaRule {
    pub name: String,
    list: Vec<ViaInfoId>,
}

impl ViaRule {
    pub fn new(name: impl Into<String>) -> Self {
        ViaRule {
            name: name.into(),
            list: Vec::new(),
        }
    }

    /// The empty via rule (Java: `ViaRule.EMPTY`).
    pub fn empty() -> Self {
        ViaRule::new("empty")
    }

    pub fn append_via(&mut self, via: ViaInfoId) {
        self.list.push(via);
    }

    /// Removes `via` from the rule; false if it was not contained.
    pub fn remove_via(&mut self, via: ViaInfoId) -> bool {
        match self.list.iter().position(|&v| v == via) {
            Some(pos) => {
                self.list.remove(pos);
                true
            }
            None => false,
        }
    }

    pub fn via_count(&self) -> usize {
        self.list.len()
    }

    pub fn get_via(&self, index: usize) -> ViaInfoId {
        self.list[index]
    }

    pub fn contains(&self, via_info: ViaInfoId) -> bool {
        self.list.contains(&via_info)
    }

    /// True if this rule contains a via with the given padstack number.
    pub fn contains_padstack(&self, padstack: usize, via_infos: &ViaInfos) -> bool {
        self.list
            .iter()
            .any(|&v| via_infos.get(v).get_padstack() == padstack)
    }

    /// Searches a via with first layer `from_layer` and last layer
    /// `to_layer`.
    pub fn get_layer_range(
        &self,
        from_layer: usize,
        to_layer: usize,
        via_infos: &ViaInfos,
        padstacks: &Padstacks,
    ) -> Option<ViaInfoId> {
        self.list.iter().copied().find(|&v| {
            padstacks
                .get_by_no(via_infos.get(v).get_padstack())
                .is_some_and(|p| p.from_layer() == from_layer && p.to_layer() == to_layer)
        })
    }

    /// Swaps the locations of `via_1` and `via_2` in the rule.
    pub fn swap(&mut self, via_1: ViaInfoId, via_2: ViaInfoId) -> bool {
        let Some(index_1) = self.list.iter().position(|&v| v == via_1) else {
            return false;
        };
        let Some(index_2) = self.list.iter().position(|&v| v == via_2) else {
            return false;
        };
        self.list.swap(index_1, index_2);
        true
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geometry::planar::{IntBox, TileShape};

    fn setup() -> (Padstacks, ViaInfos) {
        let mut padstacks = Padstacks::new(4);
        let shape = TileShape::Box(IntBox::from_coords(-400, -400, 400, 400));
        let through = padstacks.add_shape_on_layers(shape.clone(), 0, 3);
        let blind = padstacks.add_shape_on_layers(shape, 0, 1);
        let mut via_infos = ViaInfos::new();
        via_infos
            .add(ViaInfo::new("via_through", through, 1, false))
            .unwrap();
        via_infos.add(ViaInfo::new("via_blind", blind, 1, true)).unwrap();
        (padstacks, via_infos)
    }

    #[test]
    fn via_infos_add_and_lookup() {
        let (_, mut via_infos) = setup();
        assert_eq!(via_infos.count(), 2);
        assert_eq!(via_infos.get_by_name("via_through"), Some(0));
        assert!(via_infos.name_exists("via_blind"));
        // duplicate name rejected
        assert!(via_infos.add(ViaInfo::new("via_blind", 1, 1, false)).is_none());
        via_infos.get_mut(0).set_clearance_class(2);
        assert_eq!(via_infos.get(0).get_clearance_class(), 2);
    }

    #[test]
    fn via_rule_operations() {
        let (padstacks, via_infos) = setup();
        let mut rule = ViaRule::new("default");
        rule.append_via(0);
        rule.append_via(1);
        assert_eq!(rule.via_count(), 2);
        assert!(rule.contains(0));
        assert!(rule.contains_padstack(1, &via_infos));
        assert!(!rule.contains_padstack(9, &via_infos));

        // layer-range search
        assert_eq!(
            rule.get_layer_range(0, 3, &via_infos, &padstacks),
            Some(0)
        );
        assert_eq!(
            rule.get_layer_range(0, 1, &via_infos, &padstacks),
            Some(1)
        );
        assert_eq!(rule.get_layer_range(1, 2, &via_infos, &padstacks), None);

        // preference order and swap
        assert_eq!(rule.get_via(0), 0);
        assert!(rule.swap(0, 1));
        assert_eq!(rule.get_via(0), 1);
        assert!(!rule.swap(0, 9));

        assert!(rule.remove_via(0));
        assert!(!rule.remove_via(0));
        assert_eq!(rule.via_count(), 1);
        assert_eq!(ViaRule::empty().via_count(), 0);
    }
}
