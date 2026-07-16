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

    /// True if the segment a→b satisfies this restriction: axis-parallel
    /// for 90°, axis-parallel or diagonal for 45°.
    pub fn segment_is_compliant(
        self,
        a: crate::geometry::planar::IntPoint,
        b: crate::geometry::planar::IntPoint,
    ) -> bool {
        let dx = (b.x - a.x) as i64;
        let dy = (b.y - a.y) as i64;
        match self {
            AngleRestriction::None => true,
            AngleRestriction::NinetyDegree => dx == 0 || dy == 0,
            AngleRestriction::FortyfiveDegree => dx == 0 || dy == 0 || dx.abs() == dy.abs(),
        }
    }
}
