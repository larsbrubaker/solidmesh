//! A closed loop of the pick the PLANNER would not make a plan for, carried to the sections as the
//! bare ring the loop entry (`SectionBuilder.TryBuildLoop`) takes. Ports
//! `Bevel/Surface/SurfaceBevelPlanlessLoop.cs`.
//!
//! **Why a loop without a plan exists at all.** `SurfaceBevelPlan.TryCreate` plans an edge by
//! rolling a profile between the two faces beside it, and where BOTH of those read curved it
//! refuses by name ([`BevelAdvisoryCode::SurfaceLandsOnTwoCurvedFaces`],
//! `SurfaceRollAcrossTheWall.cs:177`) rather than approximate a construction it does not have. A
//! bore mouth through a tapered wall is exactly that pick - the wall is tessellated and so is the
//! bore - and because the planner refuses the FIRST such edge the whole pass ends with zero plans,
//! so the sections were never asked whether they could build it. They can: the vertex-centric
//! construction never rolls a profile between two faces, it walks each vertex's fan, and a
//! tessellated side is just more facets to walk.
//!
//! **It is not a fallback.** A loop the sections then decline goes back to being the planner's own
//! refusal in the advisory row - the legacy construction has no plan for it and never had one.
//!
//! [`BevelAdvisoryCode::SurfaceLandsOnTwoCurvedFaces`]: crate::bevel::BevelAdvisoryCode::SurfaceLandsOnTwoCurvedFaces

use std::sync::Arc;

use crate::bevel::BevelAdvisory;
use crate::topology::FeatureEdgeSet;

/// C# `SurfaceBevelPlanlessLoop` (a sealed class of init-only properties). The feature set is
/// shared, as the C# reference is.
#[derive(Debug, Clone)]
pub struct SurfaceBevelPlanlessLoop {
    /// The loop's vertices in walk order, one per chain edge - the ring the loop entry takes.
    pub ring: Vec<i32>,

    /// The pick's feature edges, which the loop entry reads each ring pair's crease off.
    pub features: Arc<FeatureEdgeSet>,

    /// The fillet radius asked of this loop.
    pub size: f64,

    /// The refusal the planner gave, which stands again if the sections decline the loop.
    pub planner_refusal: BevelAdvisory,
}
