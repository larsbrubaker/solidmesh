//! Why a level-line gap was not filled.

/// Why a gap fill refused, or `None` where it filled. The discriminants keep their declaration
/// order, so a refusal's number never changes.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub enum GapFillRefusal {
    /// The gap was filled.
    #[default]
    None,

    /// The gap has an open boundary: an open chain's ends need a fill of their own, which is stage 2.
    OpenChainEnds,

    /// A side did not leave exactly one boundary loop, so there is no pair of rails to zip.
    SideDoesNotLeaveOneLoop,

    /// A loop's chain parameter does not run one way round it (one wrap and no other step
    /// backwards), so the two sides cannot be paired by parameter. The fill result's offending
    /// vertex names where.
    RailIsNotMonotone,

    /// The fill spent its whole work budget without finishing - it was running away, and was
    /// stopped. Nothing is handed back but the refusal; a part-zipped gap is not a mesh.
    WorkBudgetExceeded,

    /// The two loops could not be read as rails running OPPOSITE ways round the chain, which is
    /// what pairing them assumes.
    RailsRunTheSameWay,

    /// The strip the zipper laid does not close: the fill checked its own claim - every boundary
    /// edge of the gap carried by exactly one fill triangle the other way round, and every other
    /// fill edge by two facing triangles - and found an edge that is not. The fill result's open
    /// edge names the first one. Nothing is handed back; an open solid never is.
    FillWouldNotClose,

    /// A corner's hole is not star-shaped about its axis: walked round the axis through the corner
    /// (the normalised sum of the face normals there) its boundary does not turn one way, once, so
    /// no fan to a centre on that axis covers it without folding. The offending vertex names where.
    CornerCapNotStarShaped,

    /// Rows laid past a GROWING neighbour (flow rung 2) could not be clipped on its planes and
    /// capped: a tie or a split would have cut an edge the kept surface stands on, or the drop's
    /// outline was not the crease stretch plus the clip curve, or a cap was not in one of the
    /// neighbour's planes. The strip is taken back.
    GrowingNeighbourClipRefused,

    /// A concave chain's strip would sink into the solid: an arc point stands behind every plane
    /// the band removed on one side, so the fill would take material away where the round adds it
    /// (a rail far round a convex boss). The offending vertex names the point.
    ArcSinksIntoTheSolid,

    /// The strip closed but laid a triangle of no area; a part with one is not handed back. The
    /// offending vertex names a corner of it.
    FillLaysAFlatTriangle,

    /// The one hole filler could not close a hole it was handed; which of its failures it named is
    /// kept by the corner cap, and the offending vertex names a vertex of the hole's boundary.
    HoleFillFailed,

    /// The hole filler's fill leaves the part: an interior fill vertex stands through a face the
    /// band replaced - in front of a convex corner's faces (a bulge out of the part) or behind a
    /// concave corner's (sunk into it), or at a mixed corner both at once. The one validity
    /// refusal of the filler, whichever way the fill went wrong; the offending vertex names a
    /// vertex of the hole's boundary.
    FillLeavesThePart,
}
