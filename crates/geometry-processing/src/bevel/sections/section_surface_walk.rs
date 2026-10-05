//! Laying a straight reach ALONG the surface, unfolding across the edges it crosses
//! (`Bevel/Sections/SectionSurfaceWalk.cs`): why a walk stopped short ([`SurfaceWalkRefusal`]),
//! where it crossed one mesh edge ([`SurfaceWalkCrossing`]) and what it came to
//! ([`SurfaceWalkResult`]). The walk itself lands with the core's section packages.

use vector_math::Vector3;

/// C# `SurfaceWalkRefusal`: why a surface walk stopped short of what was asked of it.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub enum SurfaceWalkRefusal {
    /// It did not.
    #[default]
    None = 0,
    /// The reach met a fold the caller asked it to stop at.
    CreaseReached,
    /// The reach met an edge of the surface - nothing to cross into.
    BoundaryReached,
    /// No face of a vertex's fan held the unfolded direction leaving that vertex.
    NoFaceHoldsTheDirection,
    /// The two ends of a join are not joined through non-crease edges within the strip searched.
    FacesNotConnected,
    /// The straight unfolded line between two ends left the strip of faces between them.
    JoinLeftTheStrip,
    /// The walk was asked for from a point or in a direction that names no ray at all.
    NothingToWalk,
}

/// C# `SurfaceWalkCrossing`: where a walk crossed one mesh edge.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct SurfaceWalkCrossing {
    /// The mesh edge crossed.
    pub edge: i32,
    /// Where along that edge, 0 at the graph's vertex 0 and 1 at its vertex 1.
    pub parameter: f64,
    /// The crossing point, which lies in the planes of BOTH the faces the edge joins.
    pub at: Vector3,
}

/// C# `SurfaceWalkResult`: what a walk laid on the surface came to. `Default` is C#'s field
/// initializers: no end edge or vertex (-1), no refusal.
#[derive(Debug, Clone, PartialEq)]
pub struct SurfaceWalkResult {
    /// Where the walk ended, on [`SurfaceWalkResult::end_face`].
    pub end: Vector3,
    /// The face the end point lies in.
    pub end_face: i32,
    /// The direction the walk was travelling when it ended, in the end face's plane.
    pub end_direction: Vector3,
    /// The mesh edges crossed on the way, in the order they were crossed.
    pub crossings: Vec<SurfaceWalkCrossing>,
    /// How far the walk actually went - the distance asked for, unless it was refused.
    pub travelled: f64,
    /// Why it stopped short, where it did.
    pub refusal: SurfaceWalkRefusal,
    /// The mesh edge [`SurfaceWalkResult::end`] lies ON, or -1 where the end lies strictly inside
    /// [`SurfaceWalkResult::end_face`]. A walk that stops on an edge KNOWS it did - refused at a
    /// crease or a boundary, or out of distance exactly at a crossing - without any position
    /// being compared, which is what lets a walked boundary carry an EDGE as its place.
    pub end_edge: i32,
    /// Where along [`SurfaceWalkResult::end_edge`], 0 at the graph's vertex 0 and 1 at its
    /// vertex 1.
    pub end_parameter: f64,
    /// The mesh VERTEX [`SurfaceWalkResult::end`] stands on, or -1 where it stands on no corner.
    /// A walk that runs out at a corner stands on every edge of that vertex's fan at once, so the
    /// vertex - topological, and incident to every fold meeting there - is the one reading that
    /// does not move with the part's placement.
    pub end_vertex: i32,
}

impl Default for SurfaceWalkResult {
    fn default() -> Self {
        Self {
            end: Vector3::default(),
            end_face: 0,
            end_direction: Vector3::default(),
            crossings: Vec::new(),
            travelled: 0.0,
            refusal: SurfaceWalkRefusal::None,
            end_edge: -1,
            end_parameter: 0.0,
            end_vertex: -1,
        }
    }
}
