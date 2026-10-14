//! How the 2D leg of a bevel decides which corners of a path it acts on.
//!
//! [`BevelCornerMode::Angle`] is the default, not `Select`: a fresh Fillet has to round the whole
//! shape the way it always did, and `Select` starts empty and bevels nothing. The numbers are
//! Select 0, Angle 1, Formula 2, and they are load bearing: the panel builds its tabs in value
//! order. The last two are rules re-read on every rebuild rather than stored selections.

/// Which corners of a path the 2D bevel acts on.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub enum BevelCornerMode {
    /// Only the corners the user picked in the 3D view. None picked bevels nothing - which is
    /// what makes the first click ADD a rounded corner rather than take away three.
    Select = 0,
    /// Every corner that bends more than the operation's corner threshold.
    #[default]
    Angle = 1,
    /// The corners an expression, evaluated once per corner, says yes to.
    Formula = 2,
}

impl BevelCornerMode {
    /// Every member in value order, which is the panel's tab order.
    pub const ALL: [BevelCornerMode; 3] = [
        BevelCornerMode::Select,
        BevelCornerMode::Angle,
        BevelCornerMode::Formula,
    ];

    /// What the panel shows as that tab's tool tip.
    pub fn description(self) -> &'static str {
        match self {
            BevelCornerMode::Select => "Only the corners you click in the 3D view are beveled.",
            BevelCornerMode::Angle => "Every corner sharper than the Bend Threshold is beveled.",
            BevelCornerMode::Formula => {
                "An expression decides each corner. Beveled when the result is not zero."
            }
        }
    }
}
