//! Which profile a level-line band is cut for - the only thing the size means differently between
//! the two.

/// The profile a band is cut for.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub enum BandProfile {
    /// The size is a radius, and the setback is `radius * tan(phi / 2)` at the edge's own angle.
    #[default]
    Fillet,

    /// The size is the setback itself, the same all along the chain.
    Chamfer,
}
