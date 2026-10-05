//! The reading of the MATERIAL at a chain's end corner (`Bevel/Surface/SurfaceFaceCarriesOn.cs`, a
//! static class): whether one of the two faces the bevel walks back carries straight on past that
//! corner, so that the groove has to be closed by a wall of its own rather than notched into a face
//! beyond it. Only its nested reading lands here ([`SurfaceFaceCarriesOnWayOn`]); the reading
//! itself lands with the plan package it was split out of.

/// C# `SurfaceFaceCarriesOn.WayOn` (private): where the way on past a corner falls against one
/// facet of the region standing there.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub enum SurfaceFaceCarriesOnWayOn {
    /// Outside the facet, or not a facet at this corner at all.
    #[default]
    Outside = 0,

    /// Along one of the facet's own two edges at the corner.
    AlongAnEdge,

    /// Into the facet's material.
    Inside,
}
