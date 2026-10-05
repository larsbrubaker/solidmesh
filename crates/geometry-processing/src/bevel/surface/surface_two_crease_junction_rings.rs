//! The three end rings a TWO-crease junction's patch is sewn to, and the grid between them
//! ([`SurfaceTwoCreaseJunctionRings`]). Ports the record of
//! `Bevel/Surface/SurfaceTwoCreaseJunctionRings.cs`; its reading and its grid accessors land with
//! the mixed-corner package's methods.

use std::rc::Rc;

use vector_math::Vector3;

use super::{SurfaceMixedCornerDualFrame, SurfaceMixedCornerRows};

/// C# `SurfaceTwoCreaseJunctionRings` (a sealed class): the three end rings a two-crease
/// junction's patch is sewn to, and the grid between them - read, not emitted. What the patch has
/// left to do with this is win its vertices from the pool and wind its quads
/// (`SurfaceTwoCreaseJunction` reads the canal itself).
///
/// **The grid is TRANSPOSED against a one-crease junction's.** Here the canal turns about the
/// CUT's centre line, so the stations run along the cut's ring and each station's row runs from
/// the wall's contact to the cut's tube. The two ends of the turn are the two BEADS' own square end
/// rings, which is what makes this patch close where a two-edge mixed corner's runs out.
///
/// **Everything is REUSED and nothing is re-derived.** Station `k` stands at the turn the cut's
/// ring rail `k` already names - the rail lies on the cut's tube, so the angle it makes round the
/// cut's centre line IS that station's own turn, to 2e-15 on the nose fixture. Column zero is the
/// upper bead's square end ring and the last column is the lower bead's, checked against where the
/// canal puts them rather than trusted.
///
/// **And the two beads must be sampled alike.** The rows pair by index, so two end rings of
/// different lengths have no pairing at all and are refused by name
/// (`BevelAdvisoryCode::SurfaceJunctionBeadsSampledDifferently`).
pub struct SurfaceTwoCreaseJunctionRings {
    /// The rows the grid's stations are laid with, which it built and owns.
    pub rows: SurfaceMixedCornerRows,
    /// The canal the whole patch stands on - the FIRST arm's dual, which is the whole of it.
    pub canal: Rc<SurfaceMixedCornerDualFrame>,
    /// The CUT's end ring, cut on the canal's own leaning plane rather than on a square section,
    /// running from the first arm's tangency round to the second's.
    pub cut_ring: Vec<Vector3>,
    /// The turn each rail of that ring stands at - one station of the grid each.
    pub stations: Vec<f64>,
    /// The FIRST arm's bead's square end ring, from its wall contact round to its tube contact.
    pub first_bead_ring: Vec<Vector3>,
    /// The SECOND arm's, the same way about - the column the grid closes on.
    pub last_bead_ring: Vec<Vector3>,
}
