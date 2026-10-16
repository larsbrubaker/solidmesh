//! One selected corner of a path on a fillet or chamfer, stored as the geometry the corner had
//! when it was picked.
//!
//! The 2D analog of [`super::BevelEdgeGroup`]: a vertex index does not survive a rebuild, so a
//! group carries a geometric anchor in the operation's own 2D space and
//! `BevelCornerGroupResolver` matches it against the rebuilt path. The point says where; the two
//! edge directions say which corner. There is deliberately no stored bend tolerance: which
//! vertices are corners is decided live by the operation's `BendThresholdDegrees`.
//!
//! The JSON is the shape saved designs already hold: PascalCase names, read by name (a saved
//! design lists them in its own property order), every member optional on load.

use serde::{Deserialize, Serialize};
use vector_math::Vector2;

use super::bevel_group::BevelGroup;

/// A path corner anchored by where it was and the two edges meeting at it.
#[derive(Debug, Clone, Copy, Default, PartialEq, Serialize, Deserialize)]
#[serde(default, rename_all = "PascalCase")]
pub struct BevelCornerGroup {
    /// Where the corner was, in the path's own 2D space: the primary signature.
    pub anchor_position: Vector2,
    /// The unit direction from the corner back along the edge to the vertex before it. A
    /// direction rather than the neighbour's position, because a neighbour can move a long way
    /// without the corner itself changing.
    pub anchor_to_previous: Vector2,
    /// The unit direction from the corner on along the edge to the vertex after it. Matching
    /// tries both pairings: a contour wound the other way round swaps these two.
    pub anchor_to_next: Vector2,
}

impl BevelGroup for BevelCornerGroup {
    fn copy(&self) -> Self {
        BevelCornerGroup {
            anchor_position: self.anchor_position,
            anchor_to_previous: self.anchor_to_previous,
            anchor_to_next: self.anchor_to_next,
        }
    }
}
