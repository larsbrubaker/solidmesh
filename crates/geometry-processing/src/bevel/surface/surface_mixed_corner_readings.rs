//! Every corner of one chain where the convexity turns over, read once when the chain is planned.
//! Ports `Bevel/Surface/SurfaceMixedCornerReadings.cs` (a C# static class): [`try_read_all`].

use std::rc::Rc;

use indexmap::IndexMap;

use polygon_mesh::Mesh;

use super::{
    surface_mixed_corner_cut_edge, surface_mixed_corner_hands_over_at_the_mitre, SurfaceEdgePlan,
    SurfaceEdgeRoles, SurfaceMixedCorner, SurfaceVertexPool,
};
use crate::bevel::{BevelAdvisory, BevelAdvisoryCode};

/// C# `TryReadAll`: reads every corner of the chain `planned` (closed or not) where the convexity
/// turns over - which edge the corner's CUT is, the corner's reading, and where it hands over -
/// into the plan's mixed corners (keyed by the edge the chain ARRIVES on, in walk order) and the
/// edges' roles. `Err` is C#'s `failureReason`: a corner the reading cannot place.
pub fn try_read_all(
    planned: &[SurfaceEdgePlan],
    is_closed: bool,
    size: f64,
    mesh: &Mesh,
) -> Result<(IndexMap<i32, Rc<SurfaceMixedCorner>>, SurfaceEdgeRoles), BevelAdvisory> {
    let mut mixed_corners = IndexMap::new();
    let mut roles = SurfaceEdgeRoles::default();

    let count = planned.len();
    let last = if is_closed { count } else { count - 1 };

    // The part's own ruler, read at the FIRST corner that turns over and not before: it walks the
    // mesh's vertices and most chains have no such corner.
    let mut near_enough = f64::NAN;

    for i in 0..last {
        let here = &planned[i];
        let next = &planned[(i + 1) % count];

        if here.convexity() == next.convexity() {
            continue;
        }

        if near_enough.is_nan() {
            near_enough = SurfaceVertexPool::near_enough_for(&mesh.vertices);
        }

        let not_supported = || {
            BevelAdvisory::new(
                BevelAdvisoryCode::SurfaceCornerNotSupported,
                vec![here.to().into()],
            )
        };

        // Which edge of the chain the CUT is: its own neighbour wherever the shared face is one
        // plane, an edge or more further along a chorded wall (SurfaceMixedCornerCutEdge).
        let cut_is_arriving = here.convexity() == crate::topology::EdgeConvexity::Convex;

        let (mut cut_index, swallowed) = surface_mixed_corner_cut_edge::try_find(
            planned,
            is_closed,
            i as i32,
            cut_is_arriving,
            size,
            near_enough,
        )
        .ok_or_else(not_supported)?;

        // The cut and the bead rather than the arriving edge and the leaving one.
        let touching = &planned[cut_index as usize];

        let mut mixed = SurfaceMixedCorner::try_read(
            touching,
            if cut_is_arriving { next } else { here },
            here.to(),
            size,
            near_enough,
        )?;

        // And WHERE it hands over: on that chord, or on the mitre ring just before it.
        let (carries_the_ring, ring_stops_at) =
            surface_mixed_corner_hands_over_at_the_mitre::try_read(
                planned,
                is_closed,
                cut_index as usize,
                cut_is_arriving,
                &mixed,
            );

        let mut swallowed_now = swallowed.clone();

        if ring_stops_at.is_some() {
            // The chord that touches goes under the blend with the rest of them.
            let mut also_swallowed = swallowed.clone();
            also_swallowed.push(cut_index);

            swallowed_now = also_swallowed;
            cut_index = carries_the_ring as i32;

            mixed = SurfaceMixedCorner::try_read_carried(
                &planned[carries_the_ring],
                touching,
                ring_stops_at,
                here,
                next,
                here.to(),
                size,
                near_enough,
            )?;
        }

        mixed_corners.insert(i as i32, Rc::new(mixed));
        roles.read(
            i as i32,
            cut_index,
            cut_is_arriving,
            if cut_is_arriving {
                ((i + 1) % count) as i32
            } else {
                i as i32
            },
            &swallowed_now,
        );
    }

    Ok((mixed_corners, roles))
}
