//! `MatterCAD.GeometryProcessing/Topology`: what a bevel can be applied to. Feature edges
//! ([`FeatureEdge`], [`FeatureEdgeSet`]), runs of them ([`FeatureEdgeChain`]), the logical surface
//! a face belongs to ([`FaceGroup`], cached per mesh by [`FaceRegionCache`]), turning a click
//! into an edge ([`edge_picking`]), and the stored selections of a bevel ([`BevelEdgeGroup`],
//! [`BevelCornerGroup`]) with matching them on a rebuilt mesh ([`bevel_edge_group_resolver`]) and
//! editing their lists ([`bevel_group_edits`]), and the readings a pick (or a stored group) can
//! mean ([`pick_interpretations`], [`bevel_edge_readings`]), read off a set's tangent chains
//! walked once ([`TangentChainCache`]) where a caller prepared them.

mod bevel_corner_group;
pub mod bevel_corner_group_resolver;
mod bevel_corner_mode;
mod bevel_edge_group;
pub mod bevel_edge_group_resolver;
pub mod bevel_edge_readings;
mod bevel_group;
pub mod bevel_group_edits;
mod bevel_reading_label;
pub mod edge_picking;
mod error;
mod face_group;
mod face_region_cache;
mod feature_edge;
mod feature_edge_chain;
mod feature_edge_set;
mod path_corner_set;
mod pick_interpretation;
pub mod pick_interpretations;
mod tangent_chain_cache;

pub use bevel_corner_group::BevelCornerGroup;
pub use bevel_corner_group_resolver::BevelCornerGroupResolution;
pub use bevel_corner_mode::BevelCornerMode;
pub use bevel_edge_group::{BevelEdgeGroup, BevelSelectionKind};
pub use bevel_edge_group_resolver::{BevelEdgeGroupResolution, MatchTolerances};
pub use bevel_group::BevelGroup;
pub use bevel_reading_label::BevelReadingLabel;
pub use edge_picking::FeatureEdgePick;
pub use error::TopologyError;
pub use face_group::FaceGroup;
pub use face_region_cache::FaceRegionCache;
pub use feature_edge::{EdgeConvexity, FeatureEdge};
pub use feature_edge_chain::{ChainEdge, ChainEnd, ChainStopReason, FeatureEdgeChain};
pub use feature_edge_set::FeatureEdgeSet;
pub use path_corner_set::{PathCorner, PathCornerSet};
pub use pick_interpretation::{PickInterpretation, PickInterpretationKind};
pub use tangent_chain_cache::TangentChainCache;
