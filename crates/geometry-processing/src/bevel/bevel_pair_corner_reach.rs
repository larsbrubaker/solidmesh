//! How far one chain's rolling-ball TUBE has to run past a pair corner to reach the partner's own
//! arc axis - the number `BevelPairCornerTrim` needs and the plain overshoot only happens to give
//! where the two runs meet their faces at the same angle. Ports `Bevel/BevelPairCornerReach.cs`
//! (C# `internal static class`; a module of free functions here). The frame and wedge it reads are
//! [`bevel_edge_tool_builder`]'s; the tube it lengthens is [`super::BevelPartnerTube`].
//!
//! **The geometry, in one line.** Everything cutter A has past the shared vertex lies within a
//! ball's reach of A's own arc axis, and B's tube covers a ball's reach of B's axis. So B's tube
//! covers the whole of A's overreach exactly when it runs far enough to pass A's AXIS - which is a
//! distance measured along B's own last chord, from the vertex to the plane through A's axis.
//! Where both wedges are square that distance is the radius, which is also `max(size, tangent)`,
//! and the tube reached it by accident ([`bevel_edge_tool_builder::overshoot`]). Where A's wedge is
//! acute its ball stands further up its own bisector - `size / sin(half)` - and the accident stops
//! holding: at a 75 degree wedge the axis is 1.30 radii past the vertex and the square partner's
//! tube stopped at 1.00, leaving the far tip of A's overreach untrimmed and a ridge in it.
//!
//! **Why the TUBE may be lengthened and the cutter may not.** A cutter is subtracted from the part,
//! so every millimetre of it past the selection removes material the user did not ask about -
//! which is the whole reason `BevelChainEnds` bounds the overshoot at all. A tube is subtracted
//! from a PARTNER'S CUTTER and never from the part: lengthening it can only hand material back,
//! and the material it hands back is exactly what the rolling ball would have left standing. That
//! asymmetry is why this is read at the sweep rather than folded into the shared ends - the tool,
//! the void wedge and the tube still stop in the same place everywhere else, and this is the one
//! place they must not.
//!
//! **What is left over.** The partner's axis is read from its own FIRST OR LAST CHORD at the shared
//! vertex, so on a partner whose run curves away, the true axis bends off the straight one and the
//! tube falls short by the partner's sagitta over ONE chord rather than over the whole overshoot.
//! That is the same order as the chording error already everywhere in this construction, and it
//! shrinks with the tessellation instead of with the corner's dihedral.
//!
//! **Partners, plural.** More than two runs of one convexity can end at one vertex - three creases
//! of a box corner picked as three groups is the ordinary case - and each of them overreaches past
//! it. The tube has to clear the FURTHEST of their axes, since it is subtracted from all of them,
//! so every selected run of this convexity ending here is measured and the largest is taken. Where
//! there is only one partner, which is every corner the fixtures reach, that is just the partner.
//!
//! A margin goes on top for [`bevel_edge_tool_builder::CONTAINMENT_MARGIN`]'s reason: a tube that
//! ends exactly ON the partner's axis leaves the kernel a tangency to decide about, and erring long
//! here costs nothing at all - past the axis the tube is still inside the ball's reach of it, so it
//! still only hands back material the rolling ball keeps.

use std::collections::HashSet;

use vector_math::{cs_math, math_helper, Vector3};

use super::bevel_edge_tool_builder;
use crate::topology::{ChainEdge, EdgeConvexity, FeatureEdge, FeatureEdgeSet};

/// How far past its own end vertex this chain's tube has to run, or zero where there is nothing to
/// reach - which leaves the sweep with the overshoot it always had.
///
/// `corner` is the vertex this end pairs at (`BevelChainEnds.StartPairCorner`), or `None` where
/// this end is not a recognised pair corner and so has no partner to reach. `end` is this chain's
/// own first or last edge, whichever ends at that vertex; `at_start` is whether that is the chain's
/// START, which leaves by the reverse of its direction. `features` are the feature edges this group
/// was resolved among, for the incidence at the vertex. `selected_mesh_edges` are the mesh edges
/// the whole SELECTION took, which is what says who the partners are - the same set
/// `BevelChainEnds.Resolve` reads the corner off, built once by the caller because this is asked
/// once per end and the incidence at a vertex is walked in full. `size` is the radius every tool of
/// the operation is swept at. A `None` `features` or `selected_mesh_edges` is C#'s null and means
/// no reach.
pub fn to_partner_axis(
    corner: Option<i32>,
    end: &ChainEdge,
    at_start: bool,
    features: Option<&FeatureEdgeSet>,
    selected_mesh_edges: Option<&HashSet<i32>>,
    size: f64,
) -> f64 {
    let (Some(corner), Some(features), Some(selected_mesh_edges)) =
        (corner, features, selected_mesh_edges)
    else {
        return 0.0;
    };

    let outward = if at_start {
        -end.direction()
    } else {
        end.direction()
    };
    let convexity = end.edge.convexity;
    let mut furthest = 0.0;

    for &incident in features.edges_at_vertex(corner) {
        let partner = &features.edges()[incident as usize];

        if partner.mesh_edge_index == end.edge.mesh_edge_index
            || partner.convexity != convexity
            || !selected_mesh_edges.contains(&partner.mesh_edge_index)
        {
            continue;
        }

        if let Some(to_axis) = try_axis_offset(partner, size) {
            // Along THIS tube's own last chord, because that is the direction its end ring travels.
            // A partner whose axis sits back the way this chain came reads negative and is ignored:
            // there is nothing out there for the tube to reach.
            furthest = cs_math::max_f64(furthest, to_axis.dot(outward));
        }
    }

    if furthest > 0.0 {
        furthest * bevel_edge_tool_builder::CONTAINMENT_MARGIN
    } else {
        0.0
    }
}

/// Where one edge's rolling ball sits, as an offset from any point ON that edge: a
/// `size / sin(half)` step down the bisector of its two faces, which is the same centre
/// [`bevel_edge_tool_builder::try_build_cross_section`] draws its arc about.
///
/// Negated against the frame's own y, which points along the OUTWARD bisector: a convex edge's ball
/// rolls inside the material. Only convex edges are ever asked - a tube exists only for a cutter -
/// and a concave one would want the other sign, so the assumption is written into the caller's
/// filter rather than hidden here.
#[allow(clippy::neg_cmp_op_on_partial_ord)]
fn try_axis_offset(edge: &FeatureEdge, size: f64) -> Option<Vector3> {
    let half_angle =
        math_helper::degrees_to_radians(bevel_edge_tool_builder::profile_wedge_angle_degrees(edge))
            / 2.0;
    let sine = half_angle.sin();

    if !(sine > 0.0) || edge.convexity != EdgeConvexity::Convex {
        // A wedge folded shut has no ball anyone can place; the sweep refuses such an edge outright
        // in any case, so this only ever means "no extension" rather than "no tube".
        return None;
    }

    let (_, y_axis, _) =
        bevel_edge_tool_builder::try_build_frame(edge.normal0, edge.normal1, edge.direction)
            .ok()?;

    Some(-y_axis * (size / sine))
}
