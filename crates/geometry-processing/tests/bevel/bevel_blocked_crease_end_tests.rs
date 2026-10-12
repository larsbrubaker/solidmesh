//! Ports `GeometryProcessingTests/BevelBlockedCreaseEndTests.cs`: the far side of
//! `BevelCentreFlowsThroughTests` - a concave crease blocked near its end by something standing on
//! it, with NOTHING on the far side of the block (the part stops 2 mm later). A bead that runs on
//! through an interruption must not run on here, because what it would leave out there is standing
//! in open air past the end of the part. Two guards are stacked: the block's square front face is a
//! perfect capping plane, and no other selected run picks the crease up beyond it. Asserted as the
//! part's own extent. C#'s `[Arguments]` rows are a loop.

use polygon_mesh::Mesh;
use vector_math::Vector3;

use crate::bead_sequence::{clip_and_union, light_tools};
use crate::bevel_obstruction_test_meshes::{create_blocked_crease_end, BLOCKED_AT, BLOCKED_SPAN};
use crate::kernel_cut_helpers::edge_group;
use crate::mesh_measurements::shell_count;

const SEGMENTS: i32 = 8;

/// Where the part ends, which is 2 mm past the block's near face.
const PART_ENDS_AT: f64 = BLOCKED_AT + BLOCKED_SPAN;

/// How far past the part a vertex may sit and still be rounding. A thousandth: the stub this is
/// looking for is millimetres long at every size below, so nothing here is a near miss.
const SLACK: f64 = 0.001;

#[test]
fn a_bead_stops_at_the_block_rather_than_running_on_past_the_end_of_the_part() {
    for radius in [1.0, 3.0, 5.0] {
        let part = create_blocked_crease_end();
        let result = fillet_the_foot_crease(&part, radius)
            .expect("the bead has to build and union onto the part at all");

        assert!(
            result.is_manifold(),
            "the beaded part has to be a closed surface"
        );
        assert_eq!(
            shell_count(&result),
            1,
            "one bead is added to one solid, and a stub cut loose from it would be a second shell"
        );

        let furthest = result
            .vertices
            .iter()
            .map(|vertex| vertex.y as f64)
            .fold(f64::NEG_INFINITY, f64::max);

        assert!(
            furthest <= PART_ENDS_AT + SLACK,
            "the part ends at y = {PART_ENDS_AT}, and a bead of {radius:.3} that ran on through the \
             block would reach {:.3} - a stub standing in open air. The furthest vertex is at \
             {furthest:.3}",
            BLOCKED_AT + (2.0_f64.sqrt() * radius)
        );
    }
}

/// Selects the L's foot crease - the one from the front face to the block - and builds and unions
/// its bead exactly as a rebuild does.
fn fillet_the_foot_crease(part: &Mesh, radius: f64) -> Option<Mesh> {
    let features = geometry_processing::topology::FeatureEdgeSet::create(part);
    let group = edge_group(
        &features,
        Vector3::new(4.0, 0.0, 4.0),
        Vector3::new(4.0, BLOCKED_AT, 4.0),
    );
    let built = light_tools(part, &[group], radius, SEGMENTS)?;

    Some(clip_and_union(part, &built.clippable, built.solids).0)
}
