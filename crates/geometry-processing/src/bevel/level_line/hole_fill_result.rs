//! What the level line's one hole filler hands back: a filled hole, or why it gave no fill
//! ([`HoleFillFailure`], [`HoleFillTriangle`], [`HoleFillResult`]).

use solidmesh_math::Vector3;

/// Why the hole filler gave no fill. The filler never hands back a repaired mesh.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub enum HoleFillFailure {
    /// The hole was filled.
    #[default]
    None,

    /// Fewer than three loop points.
    TooFewPoints,

    /// The loop, cross-tangent and edge-length lists are not all the same length.
    InputCountsDiffer,

    /// Every triangulation of the loop has a zero-area triangle (the loop is degenerate).
    NoValidTriangulation,

    /// Refinement kept splitting past the vertex budget - an edge length far below the hole's size.
    TooManyVertices,

    /// The fairing system had no unique solution.
    SingularFairing,

    /// The finished triangles are not a consistently oriented disk bounded by exactly the loop.
    NotClosed,

    /// Fairing left a triangle of zero area.
    ZeroAreaTriangle,
}

/// One fill triangle, its corners indices into [`HoleFillResult::vertices`].
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub struct HoleFillTriangle {
    pub a: i32,
    pub b: i32,
    pub c: i32,
}

impl HoleFillTriangle {
    /// A triangle from its three corners.
    pub fn new(a: i32, b: i32, c: i32) -> Self {
        HoleFillTriangle { a, b, c }
    }
}

/// A filled hole: `vertices` starts with the loop's own points, in order, followed by the interior
/// vertices the filler added. Every loop edge i -> i+1 appears in exactly one triangle, in that
/// direction.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct HoleFillResult {
    pub failure: HoleFillFailure,
    pub vertices: Vec<Vector3>,
    pub triangles: Vec<HoleFillTriangle>,
}

impl HoleFillResult {
    /// A result from its parts.
    pub fn new(
        failure: HoleFillFailure,
        vertices: Vec<Vector3>,
        triangles: Vec<HoleFillTriangle>,
    ) -> Self {
        HoleFillResult {
            failure,
            vertices,
            triangles,
        }
    }
}
