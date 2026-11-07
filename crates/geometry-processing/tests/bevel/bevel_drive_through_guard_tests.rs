//! The two guards around the drive-through overshoot (`bevel_corner_seam::try_drive_through`) -
//! which SECTION may carry it (only the tool's, never the ball tube's), and the corner shape that
//! must not be given one at all (a ball centre level with or past the corner vertex).

use std::collections::HashSet;

use geometry_processing::bevel::bevel_corner_patch;
use geometry_processing::bevel::bevel_corner_seam::try_drive_through;
use geometry_processing::bevel::bevel_edge_tool_builder::profile_wedge_angle_degrees;
use geometry_processing::bevel::bevel_group_tools::{self, BevelGroupToolsOptions};
use geometry_processing::bevel::{BevelSurfaceLabels, BevelTool};
use geometry_processing::topology::bevel_edge_group_resolver;
use geometry_processing::topology::{BevelSelectionKind, FaceGroup, FeatureEdge, FeatureEdgeSet};
use vector_math::math_helper::degrees_to_radians;
use vector_math::Vector3;

use crate::topology_test_meshes::create_l_shaped_prism;

const RADIUS: f64 = 3.4;

const SEGMENTS: i32 = 8;

/// A run that drives through a three-run corner at one end and turns a PAIR corner at the other
/// still sweeps its ball tube - which is what the pair corner is trimmed with.
///
/// The overshoot's taper reads the TOOL's cross section - tangent, clearance, apex, clearance,
/// tangent, then the arc - so points 1 to 3 of it are the two outer walls. The BALL section is not
/// that shape: it is a plain circle of `4 * segments` points, so points 1 to 3 are three ordinary
/// arc samples and pulling them inward dents the tube.
///
/// A dented tube is worse than no tube, and worse quietly. `BevelPairCornerTrim` subtracts it from
/// the neighbouring cutter to make the mitre, and the tube's sweep failure is SWALLOWED by design -
/// a corner whose tube will not sweep is left as plan 5.3 left it. So the corner would come out
/// domed, with the two cutters' plain union standing where the rolling ball should be, and nothing
/// said.
#[test]
fn a_run_that_drives_through_one_corner_still_sweeps_the_tube_for_its_pair_corner_at_the_other() {
    // The L prism, every edge selected - the same shape BevelMinkowskiOracleTests uses. It carries
    // both corner kinds at once: outer top corners where three convex runs meet, which drive
    // through, and the inner corner whose vertical crease is CONCAVE, which is where two runs pair
    // and the trim needs a tube. A cube cannot stand in for it: its outer corner's crease is
    // convex, so the union of the two cutters is already the mitre and no tube is ever swept.
    let prism = create_l_shaped_prism();
    let features = FeatureEdgeSet::create(&prism);

    let groups: Vec<_> = (0..features.count())
        .map(|i| {
            bevel_edge_group_resolver::create_from_edge(
                &features,
                i as i32,
                BevelSelectionKind::Edge,
                FaceGroup::DEFAULT_CURVE_TOLERANCE_DEGREES,
            )
            .expect("the edge is in the set")
        })
        .collect();

    let (resolutions, _) = bevel_edge_group_resolver::resolve_all(&groups, &prism);
    let eligible: Vec<bool> = resolutions
        .iter()
        .map(|resolution| resolution.is_resolved())
        .collect();
    let patches = bevel_corner_patch::create(&resolutions, &eligible, true, RADIUS, SEGMENTS);
    let selected: Vec<FeatureEdge> = resolutions
        .iter()
        .flat_map(|resolution| resolution.edges.iter().copied())
        .collect();
    let mut surfaces = BevelSurfaceLabels::new(&prism, None);
    let runs_may_drive_through: HashSet<i32> =
        patches.runs_may_drive_through.iter().copied().collect();

    assert!(
        !runs_may_drive_through.is_empty(),
        "the outer top corners have three convex runs each, so the fixture only means anything if \
         some corner really does drive through"
    );

    // The run under test is whichever one both drives through at one end and pairs at the other.
    let mut under_test: Option<BevelTool> = None;
    let mut axis: Option<FeatureEdge> = None;

    for i in 0..resolutions.len() {
        if under_test.is_some() {
            break;
        }

        if !eligible[i] {
            continue;
        }

        let Ok(built) = bevel_group_tools::try_build(
            &resolutions[i],
            true,
            RADIUS,
            SEGMENTS,
            BevelGroupToolsOptions {
                selected_across_groups: Some(&selected),
                patched_corners: Some(&patches.ball_centres),
                surface_labels: Some(&mut surfaces),
                runs_may_drive_through: Some(&runs_may_drive_through),
                patch_seam_rings: Some(&patches.seam_rings),
                ..BevelGroupToolsOptions::default()
            },
        ) else {
            continue;
        };

        for tool in built.tools {
            if !tool.is_bead
                && tool
                    .pair_corners
                    .as_ref()
                    .is_some_and(|corners| !corners.is_empty())
            {
                under_test = Some(tool);
                axis = Some(resolutions[i].edges[0]);
                break;
            }
        }
    }

    assert!(
        under_test.is_some(),
        "the fixture has to yield a run that turns a pair corner, or there is no tube to ask about \
         and the test is measuring nothing"
    );

    let run = under_test.expect("checked above");
    let axis = axis.expect("set with the run");

    assert!(
        run.pair_corners.as_ref().map_or(0, |corners| corners.len()) > 0,
        "this run's far end overlaps another selected run of its own convexity, which is the pair \
         corner the tube is swept for"
    );

    // The tube is the circle the rolling ball leaves: radius `size` about the ARC CENTRE, which
    // sits size / sin(half the profile wedge) from the edge along the outward bisector - not about
    // the edge itself. Every one of its vertices is therefore exactly that far from the centre
    // LINE, end rings included, because a capped ring slides along the axis and keeps its offset.
    // Nothing else in the sweep can move a point off that circle, which makes this a direct read of
    // the dent rather than of the shape it leaves behind.
    let half_wedge = degrees_to_radians(profile_wedge_angle_degrees(&axis)) / 2.0;
    let centre_line =
        axis.start - ((axis.normal0 + axis.normal1).get_normal() * (RADIUS / half_wedge.sin()));

    let off_axis: Vec<f64> = run
        .ball_sweep
        .as_ref()
        .map(|tube| {
            tube.vertices
                .iter()
                .map(|vertex| vertex.as_vector3() - centre_line)
                .map(|offset| (offset - (axis.direction * offset.dot(axis.direction))).length())
                .collect()
        })
        .unwrap_or_default();

    assert!(
        run.ball_sweep.is_some(),
        "the ball tube is what BevelPairCornerTrim takes out of the neighbouring cutter to leave a \
         mitre instead of a dome. None here means the tube's sweep failed - and that failure is \
         swallowed, so the corner would come out domed with nothing reported. The cause to look \
         for is the drive-through overshoot being applied to a section other than the Tool: the \
         taper reads the tool's profile layout, and the ball is a plain circle"
    );

    let dent = off_axis.iter().copied().fold(f64::INFINITY, f64::min);

    // The tube deliberately stands a quarter of a chord sagitta proud of the ball, so the circle
    // it is measured against is a hair over the radius; the taper it is being watched for takes a
    // QUARTER of the radius off, which is two orders larger.
    assert!(
        (dent - RADIUS).abs() <= 0.05,
        "every vertex of the tube is {RADIUS} from the ball's own axis, and the nearest sits at \
         {dent:.4}. A vertex pulled inside that is a DENT, and it comes from the drive-through \
         taper being applied to this section: the taper moves profile points 1 to 3, which on the \
         tool are its two outer walls but on the ball are three ordinary arc samples. The tube \
         still sweeps, so nothing fails - it just trims the neighbouring cutter with the wrong \
         solid and the mitre comes out wrong with nothing reported"
    );
}

/// A corner whose ball centre projects onto the run at or behind the run's own last ring gets no
/// overshoot at all - the plain cap it always had.
///
/// The overshoot runs from the ball centre's plane out to a stand-off short of the corner vertex,
/// so its length is the setback the centre sits back by. Where that setback shrinks below the
/// stand-off, the extra ring would land level with or BEHIND the run's own last ring, and the strip
/// between them would be wound backwards - a self-intersecting operand rather than a shorter
/// overshoot. Asked of `try_drive_through` directly, because a real corner only reaches this as
/// its three faces fold towards flat, and the guard should not depend on how near that is
/// reachable.
#[test]
fn a_corner_whose_ball_centre_is_not_behind_the_vertex_gets_no_overshoot() {
    let outward = Vector3::UNIT_Z;
    let vertex = Vector3::new(0.0, 0.0, 10.0);
    let driving: HashSet<i32> = HashSet::from([7]);

    // Well behind the vertex along the run: the ordinary case, and it drives through.
    let ordinary = try_drive_through(
        false,
        7,
        outward,
        vertex,
        Vector3::new(0.0, 0.0, 6.6),
        Some(&driving),
        None,
    );
    assert!(
        ordinary.is_some(),
        "a ball centre a full radius back from the vertex is the corner this was written for"
    );

    let ordinary = ordinary.expect("checked above");
    assert!(
        ordinary.drive_through_distance > ordinary.plane_distance,
        "the overshoot ring has to sit OUTSIDE the run's own last ring"
    );

    // Level with the vertex, and past it: both would put the extra ring at or behind the last one.
    for centre in [vertex, Vector3::new(0.0, 0.0, 10.5)] {
        let degenerate = try_drive_through(false, 7, outward, vertex, centre, Some(&driving), None);
        assert!(
            degenerate.is_none(),
            "a ball centre at {} against a vertex at {} leaves no room for an overshoot, and the \
             ring would be wound backwards rather than short",
            centre.z,
            vertex.z
        );

        // A declined drive-through hands back no cap at all, so nothing half-built can reach the
        // end.
        assert!(
            !degenerate.is_some_and(|cap| cap.drives_through),
            "declining has to leave the plain cap the end had before, not a half-built one"
        );
    }
}
