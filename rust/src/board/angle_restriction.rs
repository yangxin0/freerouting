//! Port of `board/AngleRestriction.java`.

/// The angle restriction for traces: none, 45 degree or 90 degree.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum AngleRestriction {
    None,
    FortyfiveDegree,
    NinetyDegree,
}

impl AngleRestriction {
    pub fn value(self) -> u8 {
        match self {
            AngleRestriction::None => 0,
            AngleRestriction::FortyfiveDegree => 1,
            AngleRestriction::NinetyDegree => 2,
        }
    }

    pub fn from_value(value: u8) -> Option<Self> {
        match value {
            0 => Some(AngleRestriction::None),
            1 => Some(AngleRestriction::FortyfiveDegree),
            2 => Some(AngleRestriction::NinetyDegree),
            _ => None,
        }
    }
}
