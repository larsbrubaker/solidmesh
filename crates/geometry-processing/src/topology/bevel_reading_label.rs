//! What a reading calls itself in a chooser, as the facts the label is made of rather than the
//! label. The application puts them into words, which keeps the kernel free of localization.

use super::bevel_edge_group::BevelSelectionKind;

/// The facts a reading's chooser label is made of.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct BevelReadingLabel {
    /// What the reading would store - [`super::BevelEdgeGroup::selection_kind`].
    pub kind: BevelSelectionKind,
    /// How many edges it resolves to.
    pub count: i32,
    /// The flat tolerance it was read at, quoted only when `show_tolerance` says so.
    pub flat_angle_degrees: f64,
    /// Whether the tolerance is worth printing: only for a chain that shares its list with
    /// another chain, because the number exists to tell two chains apart.
    pub show_tolerance: bool,
}

impl BevelReadingLabel {
    /// The label's four facts, in field order.
    pub fn new(
        kind: BevelSelectionKind,
        count: i32,
        flat_angle_degrees: f64,
        show_tolerance: bool,
    ) -> Self {
        BevelReadingLabel {
            kind,
            count,
            flat_angle_degrees,
            show_tolerance,
        }
    }
}
