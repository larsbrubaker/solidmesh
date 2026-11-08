//! The vertex TWO plans both turn a corner at, refused by name before anything is walked back: one
//! free function.
//!
//! Read off the plans alone ([`super::SurfaceBevelPlan`]), at the top of the build, so that a pick
//! nothing here can close is named at the vertex rather than found out as a seam that would not
//! meet. Two closed rims sharing one vertex, or four picked edges meeting at one, is no corner any
//! construction here builds: the ball's octant wants THREE arms and the junction wants three chain
//! ends with a crease among them. A mitre at either of those corners would have to share a rail 0
//! with bands that are not there, so it is refused here rather than half built. A chain running
//! THROUGH a vertex another plan ends at is cut open there before anything is planned
//! (`SurfaceChainOpensAtTheJunction`), so that case never reaches this reading.

use std::collections::HashMap;
use std::rc::Rc;

use solidmesh_math::Vector3;

use super::SurfaceBevelPlan;
use crate::bevel::{BevelAdvisory, BevelAdvisoryArg, BevelAdvisoryCode};

/// Whether the pass can be built at all, as far as its corners go - `Err` only
/// where one vertex of the part carries two plans' corners, naming where.
pub fn try_survey(plans: &[Rc<SurfaceBevelPlan>]) -> Result<(), BevelAdvisory> {
    // By the mesh's own vertex INDEX, which is exact: the mesh the plans were made on is welded,
    // so two plans standing at one vertex of the part stand at one index (SurfaceStripEnds gathers
    // its ends the same way, and these two readings have to agree about what "the same vertex"
    // means).
    let mut corners: HashMap<i32, Vector3> = HashMap::new();

    for plan in plans {
        let chain = &plan.edges;
        let in_chain = chain.len();

        let mut i = 0;
        while i + 1 < in_chain || (plan.is_closed && i < in_chain) {
            let next = (i + 1) % in_chain;

            // A corner the blend reaches over, or one the convexity turns over at, is not a mitre
            // at all - the construction that owns it closes both of its bands itself.
            if plan.roles().is_swallowed(i as i32)
                || plan.roles().is_swallowed(next as i32)
                || plan.try_mixed_corner_after(i as i32).is_some()
            {
                i += 1;
                continue;
            }

            let vertex = chain[i].edge.to_vertex();

            if let Some(&at) = corners.get(&vertex) {
                return Err(BevelAdvisory::new(
                    BevelAdvisoryCode::SurfaceTwoCornersAtOneVertex,
                    vec![BevelAdvisoryArg::Vector3(at)],
                ));
            }

            corners.insert(vertex, chain[i].to());
            i += 1;
        }
    }

    Ok(())
}
