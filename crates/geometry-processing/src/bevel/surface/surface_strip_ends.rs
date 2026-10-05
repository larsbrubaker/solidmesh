//! Where a strip STOPS: the ring at an open end and the notch it cuts in the face beyond, and the
//! gathering of every free end of every chain into the corners they stand at. Ports
//! `Bevel/Surface/SurfaceStripEnds.cs` (C# `internal static class`; a module of free functions
//! here). What is here is the one ruler every end reads its seam with ([`meets_the_seam`]), the
//! run-off stretch at an end ([`try_the_stretch_at`]) and the junction at a vertex
//! ([`the_junction_at`]) and the open end ([`try_open_end`]); the corner gathering lands with a
//! later package. The third way a strip stops -
//! against the next strip of its own chain - is `SurfaceStripMitre`.

use vector_math::Vector3;

use super::surface_mitre_joint_step::SideSeams;
use super::{
    surface_end_cut, SurfaceEdgePlan, SurfaceStrip, SurfaceStripStation, SurfaceVertexPool,
};
use crate::bevel::{BevelAdvisory, BevelAdvisoryCode};
use crate::topology::EdgeConvexity;

/// C# `MeetsTheSeam`: how near two points of the seam have to come to be the same point - the
/// pool's own [`SurfaceVertexPool::near_enough`], which is ten times its weld because these are
/// float positions read back and crossed with planes, and follows the part's size because a float's
/// ulp does. A seam that is a rounding out is a crack either way.
pub fn meets_the_seam(pool: &SurfaceVertexPool) -> f64 {
    pool.near_enough()
}

/// The one run-off stretch at edge `edge_index`'s start (or end) - from either side's entry of
/// `over_a_stretch` (`None`, C#'s null, is none) - or `None` where neither side runs off there.
/// Refused (at `corner`) where both sides run off at that end.
pub fn try_the_stretch_at(
    over_a_stretch: Option<&[[Option<super::SurfaceBandRunsOffOverAStretch>; 2]]>,
    edge_index: usize,
    at_the_start: bool,
    corner: vector_math::Vector3,
) -> Result<Option<super::SurfaceBandRunsOffOverAStretch>, crate::bevel::BevelAdvisory> {
    let Some(over_a_stretch) = over_a_stretch else {
        return Ok(None);
    };

    let mut stretch = None;

    for side in &over_a_stretch[edge_index] {
        let Some(here) = side
            .as_ref()
            .filter(|here| here.at_the_start == at_the_start)
        else {
            continue;
        };

        if stretch.is_some() {
            return Err(crate::bevel::BevelAdvisory::new(
                crate::bevel::BevelAdvisoryCode::SurfaceStripRunsOffBothSides,
                vec![corner.into()],
            ));
        }

        stretch = Some(here.clone());
    }

    Ok(stretch)
}

/// The junction standing at `vertex`, where one of `junctions` does.
pub fn the_junction_at(
    junctions: Option<&[std::rc::Rc<super::SurfaceMixedJunction>]>,
    vertex: i32,
) -> Option<std::rc::Rc<super::SurfaceMixedJunction>> {
    junctions?
        .iter()
        .find(|junction| junction.vertex == vertex)
        .cloned()
}

/// C# `TryOpenEnd`: a strip that stops at a chain's free end - the ring where every rail crosses
/// the end plane through `corner` and the two seams' end vertices (rail 0 and the last standing on
/// those vertices), and the notch that ring cuts out of the face beyond
/// ([`surface_end_cut::try_cut`], with no die point). Returns the station the stitch sews to.
/// `seams` is C#'s `List<(int, double)>[,]` indexed by `edge_index`; a missing or empty run there,
/// where C# would throw, is refused as an end not supported.
#[allow(clippy::too_many_arguments)]
pub fn try_open_end(
    pool: &mut SurfaceVertexPool,
    faces: &mut Vec<(i32, i32, i32)>,
    strip: &SurfaceStrip,
    edge: &SurfaceEdgePlan,
    seams: &[SideSeams],
    edge_index: i32,
    corner: Vector3,
    at_the_start: bool,
    runs_out_facing: Vector3,
) -> Result<SurfaceStripStation, BevelAdvisory> {
    let not_supported = || {
        BevelAdvisory::new(
            BevelAdvisoryCode::SurfaceStripEndNotSupported,
            vec![corner.into()],
        )
    };
    let seam_end = |side: usize| -> Option<i32> {
        let run = seams.get(edge_index as usize)?[side].as_ref()?;
        let end = if at_the_start {
            run.first()
        } else {
            run.last()
        };
        end.map(|&(vertex, _)| vertex)
    };
    let (Some(on_the_left), Some(on_the_right)) = (seam_end(0), seam_end(1)) else {
        return Err(not_supported());
    };

    let normal =
        (pool.position_of(on_the_left) - corner).cross(pool.position_of(on_the_right) - corner);
    #[allow(clippy::neg_cmp_op_on_partial_ord)] // NaN refuses, as the C#'s `!(x > 0)` does
    let no_plane = !(normal.length_squared() > 0.0);
    if no_plane {
        return Err(not_supported());
    }

    let normal = normal.get_normal();
    let rails = strip.rail_count();
    let mut ring = vec![0; rails.max(0) as usize];
    let mut meets_at = vec![0.0; rails.max(0) as usize];

    for rail in 0..rails {
        let Some(param) = strip.try_cross(rail, corner, normal) else {
            return Err(not_supported());
        };

        meets_at[rail as usize] = param;
        let at = strip.point_on(rail, param);

        if rail == 0 || rail == rails - 1 {
            let vertex = if rail == 0 { on_the_left } else { on_the_right };
            if (pool.position_of(vertex) - at).length() > meets_the_seam(pool) {
                return Err(BevelAdvisory::new(
                    BevelAdvisoryCode::SurfaceStripSeamDidNotMeet,
                    vec![at.into()],
                ));
            }

            ring[rail as usize] = vertex;
            continue;
        }

        ring[rail as usize] = pool.add(at);
    }

    // Which way the chain runs back from this corner.
    let into_the_chain = if at_the_start {
        edge.edge.direction()
    } else {
        -edge.edge.direction()
    };

    // Which end of the ring stands on a wall the bevel rolled along, if either.
    let rolled_wall_end = if edge.left_band.is_some() {
        0
    } else if edge.right_band.is_some() {
        ring.len() as i32 - 1
    } else {
        -1
    };

    // No die point: an end nothing crosses has a profile that is all rim.
    let along_the_profile = surface_end_cut::try_cut(
        pool,
        faces,
        &ring,
        corner,
        edge.convexity() == EdgeConvexity::Concave,
        into_the_chain,
        runs_out_facing,
        rolled_wall_end,
        -1,
    )?;

    Ok(SurfaceStripStation::new(ring, meets_at, along_the_profile))
}
