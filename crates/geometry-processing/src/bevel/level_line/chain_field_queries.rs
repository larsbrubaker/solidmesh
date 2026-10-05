//! The convex piece of the mesh a least-value query is asked about ([`ChainFieldMeshPiece`]), from
//! `Bevel/LevelLine/ChainField.Queries.cs`, with its closest point. The queries themselves - the
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
