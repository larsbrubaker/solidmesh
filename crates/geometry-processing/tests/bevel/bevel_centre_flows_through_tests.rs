//! Port of `R/Tests/GeometryProcessingTests/BevelCentreFlowsThroughTests.cs`: a crease INTERRUPTED
//! by the feature standing on it - the two pieces the nose leaves of the nose-in-notch part's
//! centre crease (`nose_in_notch_test_meshes::centre_flows_through_groups`), both beaded at 6 mm,
//! the selection "Center Flows Through.mcx" carries.
//!
//! The Onshape control's answer is ONE cylinder of radius 6 about (10, 1.5147) running the whole
//! 20 mm height, with nothing at all where the nose interrupts the crease; the nose is kept where it
//! stands proud of it. Neither piece has a selected neighbour at the end the nose stops it, and no
//! face there can cap a bead, so what the end has instead is material standing OVER the crease it is
//! leaving: the bead runs on and is buried in it (`BevelChainEndCap`,
//! `bevel_interrupted_crease`). The rebuild itself is `bead_sequence::light_tools` and
//! `bead_sequence::clip_and_union`.

use geometry_processing::bevel::bevel_interrupted_crease::{short_of_the_far_side, Interruption};
use geometry_processing::bevel::{BevelAdvisoryArg, BevelAdvisoryCode};
use geometry_processing::topology::{ChainEdge, FeatureEdgeSet};
use vector_math::Vector3;

use super::bead_sequence::{clip_and_union, light_tools};
use super::bevel_volume_probes::{probe_cube, volume_inside};
use super::mesh_measurements::{shell_count, signed_volume};
use super::nose_in_notch_test_meshes as meshes;

const RADIUS: f64 = 6.0;

const SEGMENTS: i32 = 8;

/// What the Onshape control measures - the part's 6055.556 plus a bead of 123.216, which is the
/// 20 mm of the sliver between the crease and the cylinder (7.7257 mm^2 of it) less the nose that
/// already fills part of it.
const CONTROL_VOLUME: f64 = 6178.772;

/// How far the rebuild's volume may sit from the control's. The rebuild's arc is 8 chords where
/// the control's is analytic, and an INSCRIBED chord leaves material OUTSIDE the cylinder, so the
/// rebuild is expected to run a few mm^3 heavy: the chorded sliver measures 7.906 mm^2 against the
/// true 7.7257, which over the 20 mm is 3.6. Five leaves room for that and for the nose's own
/// share of it, and is a twenty-fifth of the 123 mm^3 bead being measured.
const VOLUME_TOLERANCE: f64 = 5.0;

#[test]
fn the_bead_runs_through_the_nose_and_swallows_the_whole_centre_crease() {
    let part = meshes::create();
    let features = FeatureEdgeSet::create(&part);
    let (groups, _) = meshes::centre_flows_through_groups(&features);
    let result = fillet(
        &part,
        &groups,
        "the two beads have to build and union onto the part at all",
    );

    assert_eq!(
        shell_count(&result),
        1,
        "the two pieces of crease are one run in the answer, so neither bead may be left floating"
    );

    assert!(
        result.is_manifold(),
        "the two runs' overshoots overlap inside the nose, and a union of two beads sharing the \
         same cylinder is exactly the case a kernel comes back from with a crack"
    );

    let volume = signed_volume(&result);

    assert!(
        (volume - CONTROL_VOLUME).abs() <= VOLUME_TOLERANCE,
        "the control measures {CONTROL_VOLUME:.3} mm^3 against the part's own {:.3}, and this \
         measures {volume:.3}. Materially less is the bead stopping either side of the nose instead \
         of running through - which is 6160.982, the answer the fixture file stores",
        meshes::VOLUME
    );

    // The whole claim, said as geometry rather than as a volume: the corner is GONE. Every one of
    // the control's own vertices is at least 4.24 from the crease line (the nearest are its two
    // tangent lines on the slopes), so a vertex anywhere near it is raw corner the bead failed to
    // swallow.
    let on_the_crease: Vec<Vector3> = result
        .vertices
        .iter()
        .map(|vertex| vertex.as_vector3())
        .filter(|vertex| (vertex.x - 10.0).abs() < 0.1 && (vertex.y - 10.0).abs() < 0.1)
        .collect();

    assert!(
        on_the_crease.is_empty(),
        "the bead runs the whole height of the crease, so no vertex may be left on the crease line \
         itself - and these are: {}",
        on_the_crease
            .iter()
            .map(|v| format!("({:.3}, {:.3}, {:.3})", v.x, v.y, v.z))
            .collect::<Vec<_>>()
            .join(", ")
    );

    // And that the material is actually there, in the thinnest place the bead has to fill: a
    // millimetre cube in the sliver under the nose's lower face at z = 8, where the cylinder stands
    // at y = 7.486 (the chord at x = 10) and the nose's own face starts at y = 8. Empty in the part.
    let under_the_nose = probe_cube(Vector3::new(10.0, 7.75, 8.0), 0.2);
    let probe_volume = 0.2 * 0.2 * 0.2;
    let inside = volume_inside(&result, &under_the_nose);

    assert!(
        (inside - probe_volume).abs() <= probe_volume * 0.02,
        "the sliver between the crease and the cylinder under the nose's lower face has to be \
         filled - it is open air in the part, and it is where a bead that stopped at the nose leaves \
         nothing at all ({inside} of {probe_volume})"
    );
}

/// The same crease picked on ONE side of the nose only, which must NOT run through. Running on is
/// right where the user picked the crease both sides of what interrupts it and wrong where they
/// did not: out past the nose the crease is real, unselected, and in open void, so a bead that ran
/// on there would round 1.8 mm of edge nobody asked for - and on a part that ENDED inside that
/// reach it would stand in air. `bevel_interrupted_crease`'s last reading is what tells the two
/// apart.
#[test]
fn the_bead_stops_at_the_nose_when_only_one_side_of_the_crease_is_picked() {
    let part = meshes::create();
    let features = FeatureEdgeSet::create(&part);
    let (groups, names) = meshes::centre_flows_through_groups(&features);

    assert_eq!(names[0], "below the nose");

    let result = fillet(
        &part,
        &groups[..1],
        "the one bead has to build and union onto the part",
    );

    // THE SAME CUBE the two-sided case has to fill, in the sliver under the nose's lower face at
    // z = 8. It is 1.333 past the end of the picked crease, in open void the nose does not cover,
    // and it is the whole difference between a bead that ran on and one that stopped: with the
    // reading taken out, the bead reaches z = 12.667 - a size past its crease - and fills it.
    let past_the_crease = probe_cube(Vector3::new(10.0, 7.75, 8.0), 0.2);
    let inside = volume_inside(&result, &past_the_crease);

    assert!(
        inside < 0.001,
        "nothing picks this crease up beyond the nose, so the bead stops where the crease it was \
         picked on stops - 15.2 mm^3 of it is what running on adds here ({inside})"
    );
}

/// The reach that has to span the interruption, and the advisory when it does not. Asked of
/// `short_of_the_far_side` directly, with the gap handed to it, because no part in the corpus can
/// be made to show it: the same segment budget that refuses the cap and lets an end run on gets
/// MORE permissive as the crease gets longer, so on this fixture running on wants a size over 3.53
/// while falling short wants one under 3.33, and the two never meet. A part that shows it wants
/// short pieces of crease either side of a wide feature whose face leans - the shape is real,
/// nobody has drawn it, and the arithmetic that would speak up for it is pinned here meanwhile.
fn an_interruption_wider_than_the_two_reaches_is_said_out_loud(gap: f64, expect_advisory: bool) {
    let part = meshes::create();
    let features = FeatureEdgeSet::create(&part);
    let below = centre_crease_below_the_nose(&features);

    // At a right angled crease the tangent distance IS the size, so each end reaches Radius = 6 and
    // the pair spans 12: a 10 mm interruption is closed and a 20 mm one is not.
    let said = short_of_the_far_side(
        Interruption { gap },
        Interruption::default(),
        RADIUS,
        true,
        &below,
        &below,
    );

    assert_eq!(
        said.len(),
        if expect_advisory { 1 } else { 0 },
        "two reaches of {RADIUS} span 12 mm, and this interruption is {gap}"
    );

    if expect_advisory {
        assert_eq!(
            said[0].code(),
            BevelAdvisoryCode::BeadDoesNotSpanInterruption
        );
        assert!(
            matches!(said[0].args()[0], BevelAdvisoryArg::Vector3(at) if at == below.from()),
            "the sentence names the end the bead ran on from"
        );
        assert!(
            matches!(said[0].args()[2], BevelAdvisoryArg::Double(width) if width == gap),
            "and how wide the thing it could not get across is"
        );
    }
}

#[test]
fn an_interruption_wider_than_the_two_reaches_is_said_out_loud_10_false() {
    an_interruption_wider_than_the_two_reaches_is_said_out_loud(10.0, false);
}

#[test]
fn an_interruption_wider_than_the_two_reaches_is_said_out_loud_20_true() {
    an_interruption_wider_than_the_two_reaches_is_said_out_loud(20.0, true);
}

/// The piece of the notch's centre crease below the nose, as one segment of a run.
fn centre_crease_below_the_nose(features: &FeatureEdgeSet) -> ChainEdge {
    for (i, edge) in features.edges().iter().enumerate() {
        if (edge.start - meshes::notch_floor()).length() < 1e-5
            && (edge.end - meshes::lower_crease_end()).length() < 1e-5
        {
            return ChainEdge::new(i as i32, *edge, false);
        }

        if (edge.end - meshes::notch_floor()).length() < 1e-5
            && (edge.start - meshes::lower_crease_end()).length() < 1e-5
        {
            return ChainEdge::new(i as i32, *edge, true);
        }
    }

    panic!("the fixture has no centre crease below the nose");
}

/// C# `FilletAsync`: builds and unions the given groups' beads onto the part, exactly as a rebuild
/// does, every bead trimmed to what it is entitled to occupy.
fn fillet(
    part: &polygon_mesh::Mesh,
    groups: &[geometry_processing::topology::BevelEdgeGroup],
    builds: &str,
) -> polygon_mesh::Mesh {
    let built = light_tools(part, groups, RADIUS, SEGMENTS).expect(builds);
    let (result, tally) = clip_and_union(part, &built.clippable, built.solids);

    assert_eq!(
        tally.unclipped, 0,
        "each bead has to be trimmed to what it is entitled to occupy - a solid that goes on \
         unclipped is one that may stand in air out of the far side of a wall"
    );

    result
}
