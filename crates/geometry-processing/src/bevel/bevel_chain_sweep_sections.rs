//! The SECTIONS half of the chain sweep: one swept segment per chain edge, the tool's cross section
//! drawn in the edge's own frame ([`try_build_segments`]). The chain linker measures a run's back
//! reach with it and the mitre fit sews its joints.

use super::bevel_chain_sweep_mesh::SweptSegment;
use super::{bevel_edge_tool_builder, bevel_profile_polygon};
use super::{BevelAdvisory, BevelAdvisoryCode};
use crate::topology::ChainEdge;

/// One swept segment per chain edge - each edge's tool section drawn in its frame, every section
/// reporting the tool's tangent distance so all stop in the same place. Every profile is then wound
/// the same way, so the faces sewn from them come out facing outward. Refused where the run mixes
/// convexity, an edge has no length, a section cannot be drawn, or a section's point count changes
/// along the run.
#[allow(clippy::neg_cmp_op_on_partial_ord)]
pub fn try_build_segments(
    chain: &[ChainEdge],
    size: f64,
    segments: i32,
    round_profile: bool,
    back_depth: f64,
) -> Result<Vec<SweptSegment>, BevelAdvisory> {
    let mut swept: Vec<SweptSegment> = Vec::with_capacity(chain.len());

    for edge in chain {
        if edge.edge.convexity != chain[0].edge.convexity {
            // A cutter and a bead are mirror images about the edge, so one swept profile cannot be
            // both; the caller splits runs at their convexity changes.
            return Err(BevelAdvisory::new(
                BevelAdvisoryCode::RunMixedConvexity,
                vec![edge.from().into()],
            ));
        }

        let along = edge.to() - edge.from();

        if !(along.length() > 0.0) {
            return Err(BevelAdvisory::new(
                BevelAdvisoryCode::EdgeHasNoLength,
                vec![],
            ));
        }

        let (left_normal, right_normal) = bevel_edge_tool_builder::try_get_profile_normals(
            edge.edge.convexity,
            edge.left_normal(),
            edge.right_normal(),
        )?;

        // Left and right rather than face 0 and face 1, so the same physical surface is the same
        // profile point on every ring.
        let (x_axis, y_axis, z_axis) =
            bevel_edge_tool_builder::try_build_frame(left_normal, right_normal, edge.direction())?;

        let (profile, tangent_distance) = bevel_edge_tool_builder::try_build_cross_section(
            left_normal,
            right_normal,
            x_axis,
            y_axis,
            z_axis,
            size,
            segments,
            round_profile,
            back_depth,
        )?;

        if !swept.is_empty() && profile.len() != swept[0].profile.len() {
            return Err(BevelAdvisory::new(
                BevelAdvisoryCode::CrossSectionChanged,
                vec![],
            ));
        }

        swept.push(SweptSegment::new(
            edge.from(),
            edge.to(),
            z_axis,
            x_axis,
            y_axis,
            profile,
            tangent_distance,
        ));
    }

    // Every ring is walked in the same order, so one polygon's winding decides the whole tool's.
    if bevel_profile_polygon::signed_area(&swept[0].profile) < 0.0 {
        for segment in swept.iter_mut() {
            segment.profile.reverse();
            segment.profile_reversed = true;
        }
    }

    Ok(swept)
}
