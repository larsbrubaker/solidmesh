//! The pick the surface tests on the nose-in-notch part anchor on: the edge group on the feature
//! edge between two known points ([`anchor`]).

use solidmesh::topology::{BevelEdgeGroup, FeatureEdgeSet};
use solidmesh_math::Vector3;

use crate::kernel_cut_helpers::edge_group;

/// A group anchored on the feature edge running between `from` and `to`, either way round
/// (`kernel_cut_helpers::edge_group`).
pub fn anchor(features: &FeatureEdgeSet, from: Vector3, to: Vector3) -> BevelEdgeGroup {
    edge_group(features, from, to)
}
