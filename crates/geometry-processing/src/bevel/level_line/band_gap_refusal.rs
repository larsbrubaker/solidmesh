//! Why a level-line gap was not built. A size refusal names the room; the others are about the
//! pick or the mesh under it.

/// Why a gap was not built. The discriminants keep their declaration order.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub enum BandGapRefusal {
    /// The gap was built.
    #[default]
    None,

    /// Flooding one side of the chain reached the other side's triangles: one smooth region wraps
    /// both sides, so there are not two sides to set back from.
    ChainDoesNotDivideItsSides,

    /// There is no crease under the chain to bevel: the band's cut took no triangle off the part
    /// and left no boundary to fill. A pick whose two sides are one smooth surface ALL the way
    /// along it has nothing to round at any size, so this is never a statement about size.
    ChainHasNoCreaseToBevel,

    /// The band reached a feature that is not the chain's. The room says what would have fitted.
    Size,

    /// The build spent its whole work budget without finishing - it was running away, and was
    /// stopped rather than left to spin. The result's phase says what was running and its work
    /// spent what it had spent. Nothing is handed back but the refusal: a build stopped part way
    /// has an open mesh, and an open mesh is never a result.
    WorkBudgetExceeded,

    /// A chain edge does not stand on exactly two triangles, or its two triangles do not wind it
    /// consistently. A statement about the MESH under the pick - seen where an earlier loop's fill
    /// has changed the surface a later loop's chain was picked on - refused by name rather than
    /// thrown; the result's offending edge names the edge.
    ChainEdgeNotOnTwoTriangles,

    /// One triangle of the band could not be refined to carry the line however far it was split -
    /// a sliver the halving never brings inside the chord tolerance. The result's offending
    /// triangle names it. A statement about the input, refused rather than thrown.
    RefinementWouldNotSettle,

    /// Two chains of one pick share a vertex or an edge anywhere that is not a CORNER: a shared
    /// edge, or two chains meeting with only two picked edges at the vertex - which is one chain
    /// the linker should have joined. Refused by name rather than built wrong; the result names
    /// the two chains and the vertex they share. The room is ZERO and that zero is NOT a size
    /// statement - no size builds this pick.
    ChainsTouch,

    /// One chain of the pick passes through the same vertex TWICE - a figure-eight rim, two lobes
    /// of one face meeting at a point. Seen on picks nobody typed by hand (a face's rim collected
    /// off the mesh), so refused by name rather than thrown; the result names the chain and the
    /// vertex. The room is ZERO and that zero is NOT a size statement.
    ChainTouchesItself,

    /// Picked chains meet at a vertex where the part has ANOTHER crease that was not picked: the
    /// corner is not where two picked chains turn into each other on one face but where a band
    /// would have to stop against an edge left sharp. That is an end face, refused by name rather
    /// than built wrong; the result names two of the chains and the vertex. The room is ZERO and
    /// that zero is NOT a size statement.
    CornerOnAnUnpickedFace,

    /// A pick of several chains whose cut does not leave each picked side its one loop at the size
    /// asked, NOR at any size the pinch ladder tried below it: no size builds this pick, so the
    /// room is ZERO and NOT a size statement. Seen on `Center Right and Left Side.mcx`. Refused by
    /// name so the user is not told to shrink a size no smaller size fixes.
    SidesLeaveNoLoopAtAnySize,
}
