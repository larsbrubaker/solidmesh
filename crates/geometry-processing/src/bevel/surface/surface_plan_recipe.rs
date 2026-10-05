//! What a surface plan was drawn from, so the same chain can be planned again at another size
//! ([`SurfacePlanRecipe`], `Bevel/Surface/SurfacePlanRecipe.cs`), with its constructor. Planning at
//! a size lands with the core's surface packages.

use std::sync::Arc;

use crate::bevel::BevelSweptChain;
use crate::topology::FaceRegionCache;

/// C# `SurfacePlanRecipe` (a sealed class): everything `SurfaceBevelPlan::try_create` was handed
/// except the SIZE. A plan is drawn at one size and every reading taken off it is a reading at that
/// size; a run-out's is only NEARLY linear in it, so re-measuring at a candidate size means planning
/// the chain again there, and this is what that needs. Owned by the plan it was drawn for.
#[derive(Debug, Clone)]
pub struct SurfacePlanRecipe {
    /// The chain.
    pub chain: BevelSweptChain,
    /// The regions it was resolved on, or `None` (C# null), which plans nothing.
    pub regions: Option<Arc<FaceRegionCache>>,
    /// The curve-tolerance region reading, where the plan had one.
    pub curve_regions: Option<Arc<FaceRegionCache>>,
    /// Whether the profile is a round's arc rather than a chamfer's straight cut.
    pub round_profile: bool,
}

impl SurfacePlanRecipe {
    /// C#'s constructor.
    pub fn new(
        chain: BevelSweptChain,
        regions: Option<Arc<FaceRegionCache>>,
        curve_regions: Option<Arc<FaceRegionCache>>,
        round_profile: bool,
    ) -> Self {
        Self {
            chain,
            regions,
            curve_regions,
            round_profile,
        }
    }
}
