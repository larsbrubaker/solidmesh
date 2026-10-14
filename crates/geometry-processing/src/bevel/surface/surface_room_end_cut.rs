//! The notch a chain END cuts into the faces standing across its corner, as the room reads it.
//! Which surfaces stand across a region's rim at a corner, the face beyond
//! ([`try_the_face_beyond`]) and the end cut itself ([`try_cut`]).

use std::sync::Arc;

use indexmap::IndexMap;
use vector_math::Vector3;

use super::{
    surface_room_notch_sides, SurfaceBevelPlan, SurfaceEdgePlan, SurfaceRoomCut,
    SurfaceRoomFootprint,
};
use crate::topology::EdgeConvexity;
use crate::topology::{FaceGroup, FaceRegionCache};

/// The surfaces standing on the far side of `region`'s rim at `corner` (within `tolerance`), keyed
/// by region key, less the chain edge's own two (`left`, `right`) and the chain edge itself
/// (`picked_mesh_edge`). In the order the rim names them (an `IndexMap`, so the order is fixed).
#[allow(clippy::too_many_arguments)]
pub fn across_the_rim(
    regions: &FaceRegionCache,
    region: &FaceGroup,
    region_key: i32,
    corner: Vector3,
    tolerance: f64,
    picked_mesh_edge: i32,
    left: i32,
    right: i32,
) -> IndexMap<i32, Arc<FaceGroup>> {
    let mut across = IndexMap::new();

    for rim in region.boundary_edges() {
        if rim.mesh_edge_index == picked_mesh_edge
            || ((rim.start - corner).length() > tolerance
                && (rim.end - corner).length() > tolerance)
        {
            continue;
        }

        let (Some((side0, group0)), Some((side1, group1))) = (
            try_key_of(regions, rim.face0),
            try_key_of(regions, rim.face1),
        ) else {
            continue;
        };

        let other = if side0 == region_key { side1 } else { side0 };

        if other != left && other != right {
            across.insert(other, if side0 == region_key { group1 } else { group0 });
        }
    }

    across
}

/// Which region a face belongs to, as `(its key, the region)`, or `None` where the cache has none
/// for it.
pub fn try_key_of(regions: &FaceRegionCache, face_index: i32) -> Option<(i32, Arc<FaceGroup>)> {
    let region = regions.region(face_index)?;
    let key = SurfaceBevelPlan::key_of(&region);

    Some((key, region))
}

/// How near two points have to be to be the same corner of the rim, relative to the chain edge
/// (at least 1).
pub const VERTEX_TOLERANCE: f64 = 1e-5;

/// The notch a chain's open END cuts into the face standing across its corner, as a room cut of the
/// face beyond - reaching the further of the profile's two setbacks out of the corner, leaning the
/// way the section really leans where that can be read, and keyed back to the end edge it was taken
/// off. `None` for a closed, sizeless or empty plan, an end that runs out or meets a junction, a
/// concave or region-less end edge, or no single face beyond.
pub fn try_cut(plan: Option<&SurfaceBevelPlan>, at_the_start: bool) -> Option<SurfaceRoomCut> {
    let plan = plan?;
    #[allow(clippy::neg_cmp_op_on_partial_ord)] // `!(x > 0)` on purpose: NaN declines
    let no_size = !(plan.size > 0.0);
    if plan.is_closed
        || no_size
        || plan.edges.is_empty()
        || (if at_the_start {
            plan.start_runs_out
        } else {
            plan.end_runs_out
        })
        || plan.try_junction_at(at_the_start).is_some()
    {
        return None;
    }

    let edge = if at_the_start {
        &plan.edges[0]
    } else {
        &plan.edges[plan.edges.len() - 1]
    };

    // (The plan's two regions are never null here: `SurfaceEdgePlan` holds them by value.)
    if edge.convexity() != EdgeConvexity::Convex {
        return None;
    }

    let corner = if at_the_start { edge.from() } else { edge.to() };

    // The profile's reach out of the corner: the further of its two tangent points.
    let reach = vector_math::cs_math::max_f64(edge.left_setback, edge.right_setback);

    #[allow(clippy::neg_cmp_op_on_partial_ord)] // `!(x > 0)` on purpose: NaN declines
    let no_reach = !(reach > 0.0);
    if no_reach {
        return None;
    }

    let (beyond, beyond_region) = try_the_face_beyond(plan, edge, corner)?;

    // Which way the section really leans out of the corner, where the end face is one plane.
    let sides = surface_room_notch_sides::try_the_sides(
        edge,
        corner,
        plan.regions.as_deref(),
        Some(&beyond_region),
        plan.size,
        reach / plan.size,
    );

    let mut cut = SurfaceRoomCut::new(
        beyond,
        corner,
        corner,
        Vector3::ZERO,
        Vector3::ZERO,
        0.0,
        reach / plan.size,
        f64::INFINITY,
    );
    cut.shape = SurfaceRoomFootprint::EndNotch;
    // Keyed back to the edge it was taken off.
    cut.edge_index = if at_the_start {
        0
    } else {
        plan.edges.len() as i32 - 1
    };
    cut.section_side_a = sides.map(|(first, _)| first);
    cut.section_side_b = sides.map(|(_, second)| second);

    Some(cut)
}

/// The one surface standing across BOTH of the end edge's faces' rims at `corner` - its region key
/// and region; `None` where there is none, or more than one.
pub fn try_the_face_beyond(
    plan: &SurfaceBevelPlan,
    edge: &SurfaceEdgePlan,
    corner: Vector3,
) -> Option<(i32, Arc<FaceGroup>)> {
    let regions = plan.regions.as_deref()?;

    let left = edge.left_region_key();
    let right = edge.right_region_key();
    let picked = edge.edge.edge.mesh_edge_index;
    let tolerance =
        vector_math::cs_math::max_f64((edge.to() - edge.from()).length(), 1.0) * VERTEX_TOLERANCE;

    let across_the_left = across_the_rim(
        regions,
        &edge.left_region,
        left,
        corner,
        tolerance,
        picked,
        left,
        right,
    );
    let mut found: Option<(i32, Arc<FaceGroup>)> = None;

    for (key, region) in across_the_rim(
        regions,
        &edge.right_region,
        right,
        corner,
        tolerance,
        picked,
        left,
        right,
    ) {
        if !across_the_left.contains_key(&key) {
            continue;
        }

        if found.is_some() {
            // Two surfaces stand across both faces at this corner.
            return None;
        }

        found = Some((key, region));
    }

    found
}
