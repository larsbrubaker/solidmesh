//! The convex piece of the mesh a least-value query is asked about ([`ChainFieldMeshPiece`]), from
//! `Bevel/LevelLine/ChainField.Queries.cs`, with its constructors and closest point, and the
//! chain's parameter wrap ([`ChainField::wrapped`]). The queries themselves - the other
//! `ChainField` methods of that partial - land with their method package.

use std::rc::Rc;

use vector_math::Vector3;

use super::ChainField;

/// An opaque closest-point function on a piece (C# `Func<Vector3, Vector3>`).
pub type ClosestOnPiece = Rc<dyn Fn(Vector3) -> Vector3>;

/// C# `ChainField.MeshPiece`: the convex piece of the mesh a least-value query is asked about - a
/// segment, a triangle, or - for the exhaustive reference alone - an opaque closest-point
/// function, which has no closed form to reach for.
#[derive(Clone, Default)]
pub struct ChainFieldMeshPiece {
    /// The piece's first corner.
    pub a: Vector3,
    /// The piece's second corner.
    pub b: Vector3,
    /// The piece's third corner; unused on a segment.
    pub c: Vector3,
    /// True for a triangle, false for a segment or a custom piece.
    pub is_triangle: bool,
    /// The custom piece's closest-point function; `None` when the piece's own geometry is known,
    /// so a constant setback can be answered outright.
    pub closest_on_piece: Option<ClosestOnPiece>,
}

impl std::fmt::Debug for ChainFieldMeshPiece {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        // The closest-point function has no Debug; say only whether there is one.
        f.debug_struct("ChainFieldMeshPiece")
            .field("a", &self.a)
            .field("b", &self.b)
            .field("c", &self.c)
            .field("is_triangle", &self.is_triangle)
            .field("custom", &self.closest_on_piece.is_some())
            .finish()
    }
}

impl ChainFieldMeshPiece {
    /// C#'s private constructor.
    pub fn new(
        a: Vector3,
        b: Vector3,
        c: Vector3,
        is_triangle: bool,
        closest_on_piece: Option<ClosestOnPiece>,
    ) -> Self {
        ChainFieldMeshPiece {
            a,
            b,
            c,
            is_triangle,
            closest_on_piece,
        }
    }

    /// C# `MeshPiece.HasClosedForm`: true when the piece's own geometry is known, so a constant
    /// setback can be answered outright.
    pub fn has_closed_form(&self) -> bool {
        self.closest_on_piece.is_none()
    }

    /// C# `MeshPiece.Segment`: the segment a-b.
    pub fn segment(a: Vector3, b: Vector3) -> Self {
        Self::new(a, b, Vector3::default(), false, None)
    }

    /// C# `MeshPiece.Triangle`: the triangle a, b, c.
    pub fn triangle(a: Vector3, b: Vector3, c: Vector3) -> Self {
        Self::new(a, b, c, true, None)
    }

    /// C# `MeshPiece.Custom`: a piece known only by its closest-point function.
    pub fn custom(closest_on_piece: ClosestOnPiece) -> Self {
        Self::new(
            Vector3::default(),
            Vector3::default(),
            Vector3::default(),
            false,
            Some(closest_on_piece),
        )
    }

    /// The piece's closest point to `point`: the custom function where there is one, otherwise
    /// the segment's or the triangle's own closed form.
    pub fn closest_to(&self, point: Vector3) -> Vector3 {
        if let Some(closest_on_piece) = &self.closest_on_piece {
            return closest_on_piece(point);
        }

        if self.is_triangle {
            ChainField::closest_on_triangle(point, self.a, self.b, self.c)
        } else {
            ChainField::closest_on_segment(point, self.a, self.b)
        }
    }
}

impl ChainField {
    /// C# `ChainField.Wrapped`: a parameter read onto the chain: round the loop of a closed chain,
    /// held to the ends of an open one.
    pub fn wrapped(&self, parameter: f64) -> f64 {
        if !self.is_closed {
            return parameter.clamp(0.0, self.length);
        }

        // C#'s `%` on doubles is the truncating remainder, as Rust's.
        let parameter = parameter % self.length;
        if parameter < 0.0 {
            parameter + self.length
        } else {
            parameter
        }
    }

    /// True when no point of the box `min`..`max` can carry a field value at or below `margin`:
    /// the box stands farther from the chain's own box than the greatest setback plus that margin.
    /// The cull a caller that only asks "does this piece dip below the line" uses to skip the
    /// search - it answers from two boxes, so it can never turn a dipping piece away.
    pub fn nothing_reaches_below(&self, min: Vector3, max: Vector3, margin: f64) -> bool {
        let root = &self.nodes[0];
        super::segment_box_tree::box_gap(min, max, root.min, root.max) - self.max_setback > margin
    }
}
