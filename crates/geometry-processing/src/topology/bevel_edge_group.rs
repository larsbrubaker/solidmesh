//! One selected set of edges on a fillet or chamfer, stored the only way a parametric operation
//! can store a selection: as the geometry of one edge plus the rule that grows the rest from it.
//! The rule is a [`BevelSelectionKind`].
//!
//! Mesh indices do not survive a rebuild (Manifold renumbers everything), so a group carries a
//! geometric anchor in the source's own local space and [`super::bevel_edge_group_resolver`]
//! matches it against the rebuilt mesh. A group is a selection and nothing more: size and
//! segment count belong to the operation.
//!
//! **Contract.** The MCP edge tools and `BevelPickCandidates` rely on this group's JSON shape,
//! which is exactly the one saved designs hold: PascalCase property names, `SelectionKind` by
//! name, `SourceItemID` left out when null, and every member optional on load with the field
//! defaults as defaults. Members are read by name, so their order is free: serde writes them in
//! declaration order, and a saved design lists them in its own property order.

use serde::{Deserialize, Serialize};
use solidmesh_math::{Matrix4X4, Vector3};

use super::bevel_group::BevelGroup;
use super::face_group::FaceGroup;
use super::feature_edge::FeatureEdge;

/// How the edges of a group are derived from the one edge its anchor names. Serialized by name
/// rather than by ordinal, so a member added in the middle later does not re-interpret the groups
/// already saved in people's files.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default, Serialize, Deserialize)]
pub enum BevelSelectionKind {
    /// The anchored edge, and nothing else.
    #[default]
    Edge,
    /// The tangent run the anchored edge lies in - what a CAD package extends a picked edge into.
    TangentChain,
    /// The boundary of the smooth face region on the anchor's first face, flooded at the group's
    /// curve tolerance. This is "bevel the top of this box": one click, four rim edges.
    FaceGroup,
    /// A single point - a cone's tip, a cube's corner - bevelled on its own (point mode). The
    /// point is [`BevelEdgeGroup::anchor_midpoint`]; the direction and normals are unused and
    /// stay zero. Resolved on rebuild to the nearest point candidate, level line only.
    Point,
}

/// A mesh edge selection: the anchored edge's geometry plus the rule that grows the rest.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(default, rename_all = "PascalCase")]
pub struct BevelEdgeGroup {
    /// How the group grows from the anchored edge.
    pub selection_kind: BevelSelectionKind,
    /// The midpoint of the anchored edge, in source-local space: the primary signature.
    pub anchor_midpoint: Vector3,
    /// The unit direction the anchored edge runs in. Matching is sign-insensitive.
    pub anchor_direction: Vector3,
    /// The outward unit normal of the first face along the anchored edge. For
    /// [`BevelSelectionKind::FaceGroup`] this is the side the flood fill runs on.
    pub anchor_normal0: Vector3,
    /// The first face's plane offset: `anchor_normal0 . p` for any point p on that plane.
    pub anchor_plane_distance0: f64,
    /// The outward unit normal of the second face along the anchored edge.
    pub anchor_normal1: Vector3,
    /// The second face's plane offset.
    pub anchor_plane_distance1: f64,
    /// The ID of the DIRECT child of the operation's source container this group's mesh sits
    /// under. `None` means "not recorded" (every file saved before multi-part bevels) and is
    /// left out of the JSON.
    #[serde(rename = "SourceItemID", skip_serializing_if = "Option::is_none")]
    pub source_item_id: Option<String>,
    /// How far the surface may turn and still be walked through, in degrees: the tangent
    /// tolerance for a chain and the flood tolerance for a face group.
    pub curve_tolerance_degrees: f64,
    /// How far the surface had to turn for an edge to be an edge when this group was made. Kept
    /// in the file format; re-read at the operation's threshold before resolving
    /// ([`BevelEdgeGroup::read_at`]).
    pub flat_angle_degrees: f64,
}

impl Default for BevelEdgeGroup {
    fn default() -> Self {
        BevelEdgeGroup {
            selection_kind: BevelSelectionKind::Edge,
            anchor_midpoint: Vector3::default(),
            anchor_direction: Vector3::default(),
            anchor_normal0: Vector3::default(),
            anchor_plane_distance0: 0.0,
            anchor_normal1: Vector3::default(),
            anchor_plane_distance1: 0.0,
            source_item_id: None,
            curve_tolerance_degrees: FaceGroup::DEFAULT_CURVE_TOLERANCE_DEGREES,
            flat_angle_degrees: FeatureEdge::default_flat_angle_degrees(),
        }
    }
}

impl BevelEdgeGroup {
    /// This same selection, read at the operation's bend threshold: a copy whose
    /// `flat_angle_degrees` is that threshold. `curve_tolerance_degrees` is deliberately left
    /// alone - it is how far a chain may keep WALKING, not an answer to "is this an edge".
    pub fn read_at(&self, flat_angle_degrees: f64) -> BevelEdgeGroup {
        let mut copy = self.copy();
        copy.flat_angle_degrees = flat_angle_degrees;
        copy
    }

    /// This same selection, read in another space: the midpoint as a position, the direction as
    /// a vector (renormalised), the normals by the inverse transpose (renormalised), and the
    /// plane offsets re-derived. A point group carries only its midpoint (normalizing its zero
    /// direction would write NaN).
    pub fn transformed(&self, matrix: Matrix4X4) -> BevelEdgeGroup {
        let mut copy = self.copy();

        copy.anchor_midpoint = self.anchor_midpoint.transform(matrix);
        if self.selection_kind == BevelSelectionKind::Point {
            return copy;
        }

        copy.anchor_direction = self.anchor_direction.transform_vector(matrix).get_normal();
        copy.anchor_normal0 = self.anchor_normal0.transform_normal(matrix).get_normal();
        copy.anchor_normal1 = self.anchor_normal1.transform_normal(matrix).get_normal();
        copy.anchor_plane_distance0 = copy.anchor_normal0.dot(copy.anchor_midpoint);
        copy.anchor_plane_distance1 = copy.anchor_normal1.dot(copy.anchor_midpoint);

        copy
    }
}

impl BevelGroup for BevelEdgeGroup {
    fn copy(&self) -> Self {
        BevelEdgeGroup {
            selection_kind: self.selection_kind,
            anchor_midpoint: self.anchor_midpoint,
            anchor_direction: self.anchor_direction,
            anchor_normal0: self.anchor_normal0,
            anchor_plane_distance0: self.anchor_plane_distance0,
            anchor_normal1: self.anchor_normal1,
            anchor_plane_distance1: self.anchor_plane_distance1,
            curve_tolerance_degrees: self.curve_tolerance_degrees,
            flat_angle_degrees: self.flat_angle_degrees,
            source_item_id: self.source_item_id.clone(),
        }
    }
}
