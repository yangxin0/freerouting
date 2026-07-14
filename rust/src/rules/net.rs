//! Port of `rules/Net.java` and `Nets.java` (rules data only).
//!
//! The board-item queries of the Java `Net` (get_items, get_pins,
//! get_trace_length, …) live on the board and follow with the board item
//! model; here the net is pure rules data. The Java net→net-class object
//! reference becomes an index into [`crate::rules::NetClasses`].

/// The maximum legal net number for nets.
pub const MAX_LEGAL_NET_NO: i32 = 9_999_999;
/// Auxiliary net number for internal use.
pub const HIDDEN_NET_NO: i32 = 10_000_001;

/// Returns false if `net_no` belongs to a net used internally for special
/// purposes.
pub fn is_normal_net_no(net_no: i32) -> bool {
    net_no > 0 && net_no <= MAX_LEGAL_NET_NO
}

/// Properties of an individual electrical net.
#[derive(Debug, Clone, PartialEq)]
pub struct Net {
    /// The name of the net.
    pub name: String,
    /// Used only if a net is divided internally (e.g. because of fromto
    /// rules); 1 for normal nets.
    pub subnet_number: usize,
    /// The unique strictly positive number of the net.
    pub net_number: i32,
    /// Indicates if this net contains a power plane.
    contains_plane: bool,
    /// The index of the routing rule (net class) of this net.
    net_class: usize,
}

impl Net {
    pub fn get_class(&self) -> usize {
        self.net_class
    }

    pub fn set_class(&mut self, net_class: usize) {
        self.net_class = net_class;
    }

    /// Indicates if this net contains a power plane (used by the
    /// autorouter for cheap plane via costs).
    pub fn contains_plane(&self) -> bool {
        self.contains_plane
    }

    pub fn set_contains_plane(&mut self, value: bool) {
        self.contains_plane = value;
    }
}

/// The electrical nets of a board. Net numbers are 1-based like in Java.
#[derive(Debug, Clone, Default)]
pub struct Nets {
    net_arr: Vec<Net>,
}

impl Nets {
    pub fn new() -> Self {
        Self::default()
    }

    /// The biggest net number on the board.
    pub fn max_net_no(&self) -> i32 {
        self.net_arr.len() as i32
    }

    /// The net with the given name (case-insensitive) and subnet number.
    pub fn get(&self, name: &str, subnet_number: usize) -> Option<&Net> {
        self.net_arr
            .iter()
            .find(|n| n.name.eq_ignore_ascii_case(name) && n.subnet_number == subnet_number)
    }

    /// All subnets with the given name (case-insensitive).
    pub fn get_by_name(&self, name: &str) -> Vec<&Net> {
        self.net_arr
            .iter()
            .filter(|n| n.name.eq_ignore_ascii_case(name))
            .collect()
    }

    /// The net with the given 1-based net number.
    pub fn get_by_no(&self, net_no: i32) -> Option<&Net> {
        if net_no < 1 {
            return None;
        }
        self.net_arr.get(net_no as usize - 1)
    }

    pub fn get_by_no_mut(&mut self, net_no: i32) -> Option<&mut Net> {
        if net_no < 1 {
            return None;
        }
        self.net_arr.get_mut(net_no as usize - 1)
    }

    /// Adds a new net with a generated name (Java: `new_net`).
    pub fn new_net(&mut self) -> i32 {
        let name = format!("net#{}", self.net_arr.len() + 1);
        self.add(name, 1, false)
    }

    /// Adds a new net; returns its net number.
    pub fn add(&mut self, name: impl Into<String>, subnet_number: usize, contains_plane: bool) -> i32 {
        let new_net_no = self.net_arr.len() as i32 + 1;
        self.net_arr.push(Net {
            name: name.into(),
            subnet_number,
            net_number: new_net_no,
            contains_plane,
            net_class: 0,
        });
        new_net_no
    }

    pub fn iter(&self) -> impl Iterator<Item = &Net> {
        self.net_arr.iter()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn add_and_lookup() {
        let mut nets = Nets::new();
        assert_eq!(nets.max_net_no(), 0);
        let gnd = nets.add("GND", 1, true);
        let vcc = nets.add("VCC", 1, false);
        assert_eq!(gnd, 1);
        assert_eq!(vcc, 2);
        assert_eq!(nets.max_net_no(), 2);

        assert_eq!(nets.get_by_no(gnd).unwrap().name, "GND");
        assert!(nets.get_by_no(gnd).unwrap().contains_plane());
        assert!(nets.get_by_no(0).is_none());
        assert!(nets.get_by_no(3).is_none());
        assert_eq!(nets.get("gnd", 1).unwrap().net_number, gnd);
        assert!(nets.get("gnd", 2).is_none());

        // subnets share the name
        let gnd2 = nets.add("GND", 2, true);
        assert_eq!(nets.get_by_name("GND").len(), 2);
        assert_eq!(nets.get("GND", 2).unwrap().net_number, gnd2);

        let generated = nets.new_net();
        assert_eq!(nets.get_by_no(generated).unwrap().name, "net#4");

        nets.get_by_no_mut(vcc).unwrap().set_class(3);
        assert_eq!(nets.get_by_no(vcc).unwrap().get_class(), 3);
    }

    #[test]
    fn normal_net_numbers() {
        assert!(is_normal_net_no(1));
        assert!(is_normal_net_no(MAX_LEGAL_NET_NO));
        assert!(!is_normal_net_no(0));
        assert!(!is_normal_net_no(-1));
        assert!(!is_normal_net_no(HIDDEN_NET_NO));
    }
}
