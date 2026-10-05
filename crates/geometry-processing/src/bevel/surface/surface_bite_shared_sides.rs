//! The pieces ONE bite leaves, read as neighbours of each other ([`SurfaceBiteSharedSides`]): what
//! a piece's side looks like from the piece on the other side of it. Ports the fields of
//! `Bevel/Surface/SurfaceBiteSharedSides.cs`; its constructor and the chain-across reading land
//! with the bite's package as a further `impl` block.

use std::collections::HashMap;

/// C# `SurfaceBiteSharedSides`: the pieces one bite leaves, read as neighbours of each other.
///
/// A side two pieces share has to leave the bite with the same vertex sequence on both of them, or
/// the two walk one directed edge the same way and that is a fold - reported by
/// `SurfaceMixedCornerPatch.NothingCoversNothing` hundreds of faces later, at a place the bite had
/// nothing to do with. Read PIECE BY PIECE that cannot be guaranteed: whether a vertex stands on a
/// side is a measurement against the part's own ruler, and on a sliver whose two sides are one line
/// to within that ruler a vertex the neighbour carries as a corner of some OTHER side measures as
/// standing on this one. Put in here and not there, it claims an edge the neighbour does not have.
///
/// So where the piece across a side is one of this bite's own, the side's sequence is READ OFF IT
/// rather than measured: two pieces wound the same way walk their shared boundary opposite ways, so
/// the chain the neighbour walks from this side's far end back to its near end IS this side,
/// exactly and with no tolerance in it (`SurfaceConvexBite.WithTheVerticesStandingOnIt`). Where no
/// piece of this bite names both of the side's ends - the face across it was never cut - there is
/// nothing to read and the measurement is still the answer.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct SurfaceBiteSharedSides {
    /// The bite's pieces, each a ring of pool vertices.
    pub pieces: Vec<Vec<i32>>,
    /// Which pieces carry each vertex, in piece order, each piece once. Only ever looked up by
    /// vertex, never iterated, so a `HashMap`'s order cannot reach an output.
    pub carried_by: HashMap<i32, Vec<i32>>,
}
