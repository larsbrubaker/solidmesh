//! A run's split vertices read as POINTS OF ITS STRIPS rather than as stations of their own
//! (`Bevel/Sections/SectionAbsorbedSplits.cs`): the per-gap, per-side table
//! ([`SectionAbsorbedSplits`]) and one absorbed point in it ([`AbsorbedPoint`]). Reading them off
//! a run lands with the core's section packages as an `impl` block.
//!
//! A split vertex stands on the straight chain edge between two unsplit stations A and B with the
//! same two surfaces on its sides the whole way, so the strip between A and B is exactly the strip
//! an unsplit mesh would get, and the split vertex is merely a point of that strip's two boundary
//! rows - a strip never subdivided there cannot reverse there.

/// C# `SectionAbsorbedSplits`: for each gap between two consecutive unsplit stations, and for
/// each side, the boundary points the split vertices in that gap contribute to that strip's
/// boundary row, in chain order.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct SectionAbsorbedSplits {
    /// Per gap, the absorbed points on the LEFT side, in chain order.
    pub on_the_left: Vec<Vec<AbsorbedPoint>>,
    /// Per gap, the absorbed points on the RIGHT side, in chain order.
    pub on_the_right: Vec<Vec<AbsorbedPoint>>,
    /// The run's unsplit station indices in chain order. Gap `g` runs from `stations[g]` to
    /// `stations[g + 1]` - and, on a CLOSED run only, the last gap wraps back to `stations[0]`.
    pub stations: Vec<i32>,
    /// Whether the run this was read off closes on itself.
    pub is_closed: bool,
}

/// C# `SectionAbsorbedSplits.AbsorbedPoint`: one absorbed split vertex on one side of one gap -
/// where its boundary point already stands in the pool, which mesh chain vertex it replaces, and
/// how far along the gap it is.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct AbsorbedPoint {
    /// The pool index of the boundary point itself, minted once by the builder.
    pub pool_index: i32,
    /// The mesh chain vertex this point stands at, whose corners the face rebuild substitutes.
    pub chain_vertex: i32,
    /// Where along the gap the split vertex stands - the blend's own parameter, which is also the
    /// order the absorbed points of one gap merge into that gap's boundary row.
    pub parameter: f64,
    /// Whether the point was placed ON the join between the two ends' boundary points - true on a
    /// walked side, where the point is a vertex of the boundary the smooth cut runs along.
    pub is_join_placed: bool,
}
