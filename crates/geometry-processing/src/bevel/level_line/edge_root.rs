//! Where the level line crosses one mesh edge ([`EdgeRoot`]): found once, shared by both
//! triangles on that edge. Ports the record of `Bevel/LevelLine/EdgeRoot.cs` with the bisection's
//! constants ([`BISECTION_STEPS`], [`BRACKET_TOLERANCE`]); the bisection that finds the roots
//! (`EdgeRoots`) lands with the core package that reads the field along edges.

use vector_math::Vector3;

/// C# `EdgeRoot`: where the level line crosses one mesh edge, keyed by the edge's two vertex
/// indices in ascending order.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct EdgeRoot {
    /// The lower vertex index of the edge.
    pub a: i32,
    /// The higher vertex index of the edge.
    pub b: i32,
    /// Where on the edge the line crosses.
    pub point: Vector3,
}

impl EdgeRoot {
    /// C#'s constructor.
    pub fn new(a: i32, b: i32, point: Vector3) -> Self {
        Self { a, b, point }
    }
}

/// C# `EdgeRoots.BisectionSteps`: the worst case, so a pathological field still stops - 60
/// halvings take any edge below 1e-18 of its length.
pub const BISECTION_STEPS: i32 = 60;

/// C# `EdgeRoots.BracketTolerance`: how tight the bracket round the root is taken, as a fraction
/// of the segment's length. Fixed rather than adaptive so the same input always gives the same
/// bits, and far below the 1e-9 the module's answers are held to.
pub const BRACKET_TOLERANCE: f64 = 1e-13;
