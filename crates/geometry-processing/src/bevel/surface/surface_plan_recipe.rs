//! What a surface plan was drawn from, so the same chain can be planned again at another size
//! ([`SurfacePlanRecipe`]), with its constructor and the chain planned again at a size
//! ([`SurfacePlanRecipe::try_plan_at`]).

use std::rc::Rc;
use std::sync::Arc;

use super::SurfaceBevelPlan;
use crate::bevel::{BevelAdvisory, BevelAdvisoryCode, BevelSweptChain};
use crate::topology::FaceRegionCache;

/// Everything `SurfaceBevelPlan::try_create` was handed except the SIZE. A plan is drawn at one
/// size and every reading taken off it is a reading at that size; a run-out's is only NEARLY linear
/// in it, so re-measuring at a candidate size means planning the chain again there, and this is
/// what that needs. Owned by the plan it was drawn for.
#[derive(Debug, Clone)]
pub struct SurfacePlanRecipe {
    /// The chain.
    pub chain: BevelSweptChain,
    /// The regions it was resolved on, or `None`, which plans nothing.
    pub regions: Option<Arc<FaceRegionCache>>,
    /// The curve-tolerance region reading, where the plan had one.
    pub curve_regions: Option<Arc<FaceRegionCache>>,
    /// Whether the profile is a round's arc rather than a chamfer's straight cut.
    pub round_profile: bool,
}

impl SurfacePlanRecipe {
    /// Records a chain and what it was resolved and planned with.
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

    /// The same chain planned at `size`, or the reason it could not be - which is not this
    /// reading's to report, since the size it was asked about is one nobody has asked for yet. No
    /// regions plans nothing.
    pub fn try_plan_at(&self, size: f64) -> Result<Rc<SurfaceBevelPlan>, BevelAdvisory> {
        let Some(regions) = &self.regions else {
            return Err(BevelAdvisory::new(
                BevelAdvisoryCode::NoEdgesResolved,
                Vec::new(),
            ));
        };

        SurfaceBevelPlan::try_create_with_curve_regions(
            &self.chain,
            regions,
            self.curve_regions.as_ref(),
            size,
            self.round_profile,
        )
    }
}
