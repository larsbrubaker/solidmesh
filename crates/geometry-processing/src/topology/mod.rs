//! `MatterCAD.GeometryProcessing/Topology`: what a bevel can be applied to. Feature edges
//! ([`FeatureEdge`], [`FeatureEdgeSet`]), runs of them ([`FeatureEdgeChain`]), the logical surface
//! a face belongs to ([`FaceGroup`], cached per mesh by [`FaceRegionCache`]) and turning a click
//! into an edge ([`edge_picking`]).

pub mod edge_picking;
mod error;
mod face_group;
mod face_region_cache;
mod feature_edge;
mod feature_edge_chain;
mod feature_edge_set;

pub use edge_picking::FeatureEdgePick;
pub use error::TopologyError;
pub use face_group::FaceGroup;
pub use face_region_cache::FaceRegionCache;
pub use feature_edge::{EdgeConvexity, FeatureEdge};
pub use feature_edge_chain::{ChainEdge, ChainEnd, ChainStopReason, FeatureEdgeChain};
pub use feature_edge_set::FeatureEdgeSet;
