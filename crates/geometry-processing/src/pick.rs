//! Making picks: turning a click on a part into the [`BevelEdgeGroup`](crate::BevelEdgeGroup) a
//! [`BevelPartRequest`](crate::BevelPartRequest) carries.
//!
//! Prepare the part once with [`BevelPickData::build`] (the welded working copy, a ray tracing
//! tree over it, its feature edges and face regions), then for each click:
//!
//! - **an edge**: [`pick_feature_edge`] (or [`pick_feature_edge_measured`], which measures the
//!   tolerance in another space, such as the world's) finds the feature edge nearest the hit
//!   point, and [`interpretations_of_edge`] reads it every way it can mean: the edge alone, its
//!   tangent chain, a face rim. [`index_of_preferred_chain`] says which reading a chain pick
//!   prefers; each [`PickInterpretation`] holds its group, edges and chain;
//! - **a face rim**: [`interpretations_of_face`] reads the rim of the smooth region a face
//!   belongs to;
//! - **a corner**: [`corner_candidates`] lists the sharp corners a point pick can take, and the
//!   group is a [`BevelSelectionKind::Point`](crate::BevelSelectionKind::Point) group anchored
//!   there;
//! - **without a click**: [`group_from_edge`] anchors a group on one feature edge,
//!   [`group_from_face`] on a face region's rim and [`groups_for_all_edges`] makes one group per
//!   straight crease of a mesh.
//!
//! This module re-exports those items from the kernel's own modules under names that read on
//! their own.

pub use crate::bevel::level_line::point_candidates::{
    describe as corner_candidates, PointCandidate,
};
pub use crate::bevel::BevelPickData;
pub use crate::topology::bevel_edge_group_resolver::{
    create_from_edge as group_from_edge, create_from_face_group as group_from_face,
    create_groups_for_all_edges as groups_for_all_edges, IncludeEdge,
};
pub use crate::topology::edge_picking::{pick_feature_edge, pick_feature_edge_measured};
pub use crate::topology::pick_interpretations::{
    build as interpretations_of_edge, build_for_face as interpretations_of_face,
    index_of_preferred_chain,
};
pub use crate::topology::{FeatureEdgePick, PickInterpretation, PickInterpretationKind};
