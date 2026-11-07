//! The JUNCTION, READ - the L prism's top reflex
//! corner with both of its top rims picked as well as the crease, where three selected edges meet
//! at one vertex and exactly one of them is concave. These pin the geometry the blend there is
//! built from, off the shipped plan; what the build publishes is
//! `surface_mixed_junction_build_tests`.
//!
//! The fixture: `create_l_shaped_grid_prism` at `r = 1`, corner `V = (10, 10, 10)`, faces T (the
//! cap, `z = 10`), S (the step wall, `x = 10`) and W (the void wall, `y = 10`). E1 = T inter S and
//! E2 = T inter W are convex; E3 = S inter W is the concave bead, axis `A = (11, 11, z)`. There is
//! no ball touching all three faces: the octant's ball at `(9, 9, 9)` stands `2 root 2` from the
//! bead's axis. Each cut is tangent to the bead at ONE point (P on S, Q on W), the two cuts never
//! meet, and the filler is one torus with no run-out: C1's end ring carried a quarter turn about
//! A is C2's end ring sample for sample.

use std::rc::Rc;
use std::sync::Arc;

use geometry_processing::bevel::surface::{
    surface_face_walkback, surface_mixed_corner_patch, SurfaceBevelPlan, SurfaceEdgePlan,
    SurfaceLine, SurfaceMixedCorner, SurfaceMixedJunction, SurfaceStrip, SurfaceVertexPool,
};
use geometry_processing::bevel::{BevelAdvisoryArg, BevelAdvisoryCode};
use geometry_processing::topology::{EdgeConvexity, FaceRegionCache, FeatureEdgeSet};
use polygon_mesh::solids::platonic_solids;
use vector_math::{math_helper, Vector3};

use crate::surface_mixed_corner_fixture as fixture;
use crate::surface_walkback_checks::{area_on, vertices_on};
use crate::topology_test_meshes::create_l_shaped_grid_prism;

const RADIUS: f64 = 1.0;

/// Positions are read off float vertices and crossed with planes; a ten-millionth is exact here.
const EXACT: f64 = 1e-6;

/// The L's top reflex corner, where the crease and both top rims meet.
fn the_corner() -> Vector3 {
    Vector3::new(10.0, 10.0, 10.0)
}

fn assert_within(actual: f64, expected: f64, tolerance: f64, because: &str) {
    assert!(
        (actual - expected).abs() <= tolerance,
        "{actual} is not {expected} within {tolerance}: {because}"
    );
}

fn the_first_vector_of(reason: &geometry_processing::bevel::BevelAdvisory) -> Vector3 {
    match reason.args()[0] {
        BevelAdvisoryArg::Vector3(at) => at,
        _ => panic!("the refusal is named at a point - instead {reason:?}"),
    }
}

/// Both readings are of ONE bead: the same axis, the same outer rail centre, the same quarter turn
/// - and the ball the first hands over at is the ball the second starts from.
#[test]
fn the_two_cuts_read_one_bead_with_one_axis_and_one_outer_rail() {
    let (first, second, near_enough) = the_two_readings();

    // One bead, so one axis - and it is the line in the void, not either cut's centre line.
    assert!(first.axis().distance_to(Vector3::new(11.0, 11.0, 0.0)) < EXACT);
    assert!(second.axis().distance_to(Vector3::new(11.0, 11.0, 0.0)) < EXACT);
    assert!((first.axis().direction.dot(second.axis().direction).abs() - 1.0).abs() < EXACT);

    // One cap, so one outer rail: the ball touches T on the arc of radius 2r about where the axis
    // meets it, whichever cut it is read from.
    assert!(
        first
            .outer_rail_centre
            .equals_within(Vector3::new(11.0, 11.0, 10.0), EXACT),
        "instead {:?}",
        first.outer_rail_centre
    );
    assert!((second.outer_rail_centre - first.outer_rail_centre).length() < EXACT);
    assert_within(
        second.outer_rail_radius(),
        first.outer_rail_radius(),
        EXACT,
        "one outer rail",
    );
    assert!(second
        .out_of_the_cap_face
        .equals_within(Vector3::new(0.0, 0.0, 1.0), EXACT));

    // The void wedge is a right angle, so both turns are - and that is the whole turn of the patch.
    assert_within(
        math_helper::radians_to_degrees(first.turn_radians()),
        90.0,
        1e-9,
        "a quarter turn",
    );
    assert!((first.turn_radians() - second.turn_radians()).abs() < 1e-6);

    // Each cut shares a DIFFERENT face with the bead - S for the first, W for the second.
    assert_ne!(first.shared_region_key, second.shared_region_key);
    assert!(
        first
            .tangent_point
            .equals_within(Vector3::new(10.0, 11.0, 9.0), EXACT),
        "C1 touches the bead on S at P - instead {:?}",
        first.tangent_point
    );
    assert!(
        second
            .tangent_point
            .equals_within(Vector3::new(11.0, 10.0, 9.0), EXACT),
        "C2 touches the bead on W at Q - instead {:?}",
        second.tangent_point
    );

    // The consistency the junction is built on: the first reading's ball at the END of its turn
    // is the second reading's ball at the START of its own.
    assert!(
        (first.ball_centre_at(first.turn_radians()) - second.ball_centre_at(0.0)).length()
            < near_enough,
        "instead {:?} against {:?}",
        first.ball_centre_at(first.turn_radians()),
        second.ball_centre_at(0.0)
    );

    assert!(
        first
            .ball_centre_at(0.0)
            .equals_within(Vector3::new(9.0, 11.0, 9.0), EXACT),
        "turn zero stands on C1's own axis - instead {:?}",
        first.ball_centre_at(0.0)
    );
    assert!(
        first
            .ball_centre_at(first.turn_radians())
            .equals_within(Vector3::new(11.0, 9.0, 9.0), EXACT),
        "and the far end on C2's - instead {:?}",
        first.ball_centre_at(first.turn_radians())
    );
}

/// And nothing is owed between the two cuts directly. Their centre lines CROSS at the ball centre
/// the three-convex corner would fan its patch from - but that ball stands `2 root 2` from the
/// bead's axis, so it touches the bead nowhere, and each cut's strip exists only past the OTHER's
/// handover plane anyway.
#[test]
fn neither_cut_touches_the_other_and_the_three_convex_ball_is_nowhere_near_the_bead() {
    let (first, second, _) = the_two_readings();

    // The three-convex corner's ball: on BOTH cuts' centre lines.
    let octant = Vector3::new(9.0, 9.0, 9.0);

    assert!(first.cut_center_line().distance_to(octant) < EXACT);
    assert!(second.cut_center_line().distance_to(octant) < EXACT);
    assert!(
        closest_approach(first.cut_center_line(), second.cut_center_line()) < EXACT,
        "the two centre lines both lie in z = 9 and cross at the octant's own ball"
    );

    // And that ball touches the bead NOWHERE: a ball rolling on the bead stands 2r from its axis.
    assert_within(
        first.axis().distance_to(octant),
        2.0 * 2.0_f64.sqrt(),
        EXACT,
        "2 root 2 from the axis",
    );
    assert!(
        (first.axis().distance_to(octant) - (2.0 * RADIUS)).abs() > 0.5,
        "so no one ball touches T, S and W here and also rolls on the bead"
    );

    // Where the crossing goes: each cut's strip lives only PAST the other's handover plane, and the
    // crossing stands two radii back behind both of them.
    assert_within(
        first.cut_stops_at.signed_distance(octant),
        -2.0 * RADIUS,
        EXACT,
        "behind the first handover",
    );
    assert_within(
        second.cut_stops_at.signed_distance(octant),
        -2.0 * RADIUS,
        EXACT,
        "behind the second handover",
    );
}

/// The patch is one surface: the first cut's end ring, carried the whole turn about the bead's
/// axis, IS the second cut's end ring - sample for sample, to the pool's own ruler.
fn the_second_cuts_end_ring_is_the_firsts_carried_round_the_beads_axis(segments: i32) {
    let (first, second, near_enough) = the_two_readings();
    let (first_strip, second_strip) = the_two_cut_strips(segments);

    let (_, first_ring) =
        surface_mixed_corner_patch::try_cut_the_ring(&first, &first_strip, true, None)
            .unwrap_or_else(|reason| panic!("{reason:?}"));
    let (_, second_ring) =
        surface_mixed_corner_patch::try_cut_the_ring(&second, &second_strip, true, None)
            .unwrap_or_else(|reason| panic!("{reason:?}"));

    assert_eq!(second_ring.len(), first_ring.len());

    for sample in 0..first_ring.len() {
        let carried = first.turned_about(first_ring[sample], first.turn_radians());

        assert!(
            (carried - second_ring[sample]).length() < near_enough,
            "sample {sample}: {:?} carries to {carried:?}, and the second ring has {:?}",
            first_ring[sample],
            second_ring[sample]
        );
    }

    // The four corners of the two rings, named: the cap ends and the tangencies.
    let has = |ring: &[Vector3], at: Vector3| ring.iter().any(|on| on.equals_within(at, 1e-5));

    assert!(
        has(&first_ring, Vector3::new(9.0, 11.0, 10.0)),
        "the first ring reaches the cap at the outer rail's turn zero"
    );
    assert!(
        has(&first_ring, Vector3::new(10.0, 11.0, 9.0)),
        "and the shared face at P"
    );
    assert!(
        has(&second_ring, Vector3::new(11.0, 9.0, 10.0)),
        "the second ring reaches the cap at the outer rail's far end"
    );
    assert!(
        has(&second_ring, Vector3::new(11.0, 10.0, 9.0)),
        "and its own shared face at Q"
    );
}

#[test]
fn the_second_cuts_end_ring_is_the_firsts_carried_round_the_beads_axis_4() {
    the_second_cuts_end_ring_is_the_firsts_carried_round_the_beads_axis(4);
}

#[test]
fn the_second_cuts_end_ring_is_the_firsts_carried_round_the_beads_axis_8() {
    the_second_cuts_end_ring_is_the_firsts_carried_round_the_beads_axis(8);
}

#[test]
fn the_second_cuts_end_ring_is_the_firsts_carried_round_the_beads_axis_16() {
    the_second_cuts_end_ring_is_the_firsts_carried_round_the_beads_axis(16);
}

/// The junction's closed forms, in the cut's own chorded section: a volume that climbs to
/// `2997.87079` from BELOW as the sampling doubles, and a cap of `279.87855` at eight segments.
#[test]
fn the_junctions_closed_forms_climb_to_the_smooth_ones_from_below() {
    use std::f64::consts::PI;

    let smooth_section = 1.0 - (PI / 4.0);
    let smooth_moment = (11.0 / 6.0) - (PI / 2.0);
    let smooth_volume = 3000.0 - (8.0 * smooth_section) - ((PI / 2.0) * smooth_moment);

    assert_within(smooth_section, 0.214602, 1e-6, "the smooth section");
    assert_within(smooth_moment, 0.262537, 1e-6, "its first moment");
    assert_within(smooth_volume, 2997.87079, 1e-5, "the smooth volume");

    let (first, _, _) = the_two_readings();

    let mut last = 0.0;

    for segments in [4, 8, 16] {
        let strip = the_two_cut_strips(segments).0;
        let (section, swept) = fixture::the_corners_own_section_of(&first, &strip);
        let volume = fixture::the_junctions_volume(&first, &strip);

        assert!(
            section > smooth_section,
            "the chorded section at {segments} cuts inside the arc, so it takes more - instead {section}"
        );
        assert!(section - smooth_section < 0.1);

        let steps = (strip.rail_count() - 1) as f64;
        let moment = swept / (steps * (first.turn_radians() / steps).sin());

        assert!(
            moment > smooth_moment,
            "and so does its first moment at {segments} - instead {moment}"
        );

        assert!(
            volume < smooth_volume,
            "so the chorded volume at {segments} stands under the smooth one - instead {volume}"
        );
        assert!(
            volume > last,
            "and climbs as the sampling doubles - {segments} gave {volume} against {last}"
        );

        last = volume;
    }

    // Quadratically: the gap is 0.19 at four segments, 0.048 at eight and 0.012 at sixteen.
    assert!(
        smooth_volume - last < 0.02,
        "sixteen segments is most of the way there - instead {last}"
    );

    // And the cap: 300 less the two 1 by 9 bands, less the chorded quarter disc of radius 2r about
    // the axis, less the unit square of that disc the void bite already had.
    assert_within(
        fixture::the_junctions_cap_area(8),
        279.87855,
        1e-5,
        "the cap",
    );
}

/// The junction READS: three chain ends at one vertex, one crease among them, and the two mixed
/// corners they make of one bead - each cut its own chain's end edge, with no chord swallowed.
#[test]
fn the_junction_reads_as_two_mixed_corners_of_one_bead() {
    let (plans, vertex, near_enough) = the_three_plans(true);

    let junction = SurfaceMixedJunction::try_read(&plans, vertex, the_corner(), near_enough)
        .unwrap_or_else(|reason| panic!("{reason:?}"));

    assert!((junction.corner - the_corner()).length() < EXACT);
    assert_eq!(junction.vertex, vertex);

    // The bead is the crease's chain, and the corner stands at its start - it runs down from there.
    assert_eq!(junction.bead_plan(), 2);
    assert_eq!(junction.bead_index(), 0);
    assert!(junction.bead_corner_at_the_start());
    assert_eq!(junction.bead().convexity(), EdgeConvexity::Concave);

    // Both arms are their chain's end edge with nothing swallowed: the shared faces are flat here,
    // so the cut arriving at the corner is the cut whose cylinder touches the bead.
    for arm in [&junction.first, &junction.second] {
        assert_eq!(
            arm.cut_index, 0,
            "each cut is planned on a chain of one edge"
        );
        assert_eq!(
            arm.swallowed.len(),
            0,
            "a flat shared face swallows no chord"
        );
        assert!(!arm.mixed.hands_over_at_a_mitre);
        assert!(arm.mixed.rolled_section().is_none());
    }

    // The first cut's chain runs INTO the corner and the second's runs out of it.
    assert_eq!(junction.first.plan, 0);
    assert!(!junction.first.corner_at_the_start);
    assert_eq!(junction.second.plan, 1);
    assert!(junction.second.corner_at_the_start);

    // One cap and two different shared faces.
    assert_ne!(
        junction.first.mixed.shared_region_key,
        junction.second.mixed.shared_region_key
    );
    assert_ne!(
        junction.cap_region_key,
        junction.first.mixed.shared_region_key
    );
    assert_ne!(
        junction.cap_region_key,
        junction.second.mixed.shared_region_key
    );

    assert_within(
        math_helper::radians_to_degrees(junction.turn_radians()),
        90.0,
        1e-9,
        "a quarter turn",
    );
    assert!(junction
        .outer_rail_centre()
        .equals_within(Vector3::new(11.0, 11.0, 10.0), EXACT));
    assert!(junction.axis().distance_to(Vector3::new(11.0, 11.0, 0.0)) < EXACT);

    assert!(
        junction
            .first
            .mixed
            .tangent_point
            .equals_within(Vector3::new(10.0, 11.0, 9.0), EXACT),
        "the first arm touches on S at P - instead {:?}",
        junction.first.mixed.tangent_point
    );
    assert!(
        junction
            .second
            .mixed
            .tangent_point
            .equals_within(Vector3::new(11.0, 10.0, 9.0), EXACT),
        "and the second on W at Q - instead {:?}",
        junction.second.mixed.tangent_point
    );
}

/// And it refuses what it does not describe, by name and at the vertex: a corner where only TWO
/// of the three edges were picked.
#[test]
fn two_ends_at_the_vertex_are_no_junction() {
    let (plans, vertex, near_enough) = the_three_plans(true);

    // The two-edge mixed corner: the first cut and the bead, which SurfaceMixedCornerTests builds.
    let two = vec![plans[0].clone(), plans[2].clone()];

    let Err(reason) = SurfaceMixedJunction::try_read(&two, vertex, the_corner(), near_enough)
    else {
        panic!("two ends is no junction");
    };

    assert_eq!(
        reason.code(),
        BevelAdvisoryCode::SurfaceMixedJunctionNotRead,
        "two ends is not four edges, and the count's sentence is kept for the corners that really are crowded"
    );
    assert!((the_first_vector_of(&reason) - the_corner()).length() < EXACT);
}

/// The same three edges CHAMFERED. There is no rolling ball behind a straight profile, and the
/// refusal says that rather than wearing the crowded vertex's sentence.
#[test]
fn a_chamfered_three_edge_corner_is_refused_for_its_profile() {
    let (plans, vertex, near_enough) = the_three_plans(false);

    assert!(
        !plans.iter().any(|plan| plan.edges[0].has_center_line),
        "the fixture only means anything if the chamfer really did plan without a centre line"
    );

    let Err(reason) = SurfaceMixedJunction::try_read(&plans, vertex, the_corner(), near_enough)
    else {
        panic!("a chamfered corner is no junction");
    };

    assert_eq!(
        reason.code(),
        BevelAdvisoryCode::SurfaceMixedJunctionOfChamfers
    );
    assert!((the_first_vector_of(&reason) - the_corner()).length() < EXACT);
}

/// Three CONVEX edges at one vertex - a cube's corner - are the octant and not this corner, so
/// the junction refuses them rather than reading a bead that is not there.
#[test]
fn three_convex_ends_at_one_vertex_are_the_octant_and_not_a_junction() {
    let cube = platonic_solids::create_cube_vector(Vector3::new(20.0, 20.0, 20.0));
    let (regions, features) = fixture::regions_of(&cube);
    let at = Vector3::new(10.0, 10.0, 10.0);
    let mut plans = Vec::new();
    let mut vertex = -1;

    for i in 0..features.count() {
        let edge = &features.edges()[i];

        if (edge.start - at).length() > 1e-6 && (edge.end - at).length() > 1e-6 {
            continue;
        }

        let plan =
            SurfaceBevelPlan::try_create(&fixture::chain_of(&features, i), &regions, RADIUS, true)
                .unwrap_or_else(|reason| panic!("the plan refused the edge: {reason}"));

        vertex = if (edge.start - at).length() < 1e-6 {
            plan.edges[0].edge.from_vertex()
        } else {
            plan.edges[0].edge.to_vertex()
        };
        plans.push(plan);
    }

    assert_eq!(plans.len(), 3, "a cube's corner is where three rims meet");

    let Err(reason) = SurfaceMixedJunction::try_read(
        &plans,
        vertex,
        at,
        SurfaceVertexPool::near_enough_for(&cube.vertices),
    ) else {
        panic!("there is no crease here, so there is no bead to turn about");
    };

    assert_eq!(
        reason.code(),
        BevelAdvisoryCode::SurfaceMixedJunctionNotRead,
        "three edges meet here, not four - what it cannot do is READ them as this corner"
    );
}

/// The WALKBACK takes the junction: handed the corner, the three plans bound their bands on its
/// own planes, and the L is left with exactly the two 9 by 9 walls and a cap short of its two
/// bands - 81, 81 and 282.
#[test]
fn the_walkback_takes_the_junction_and_leaves_both_walls_their_square() {
    let mesh = create_l_shaped_grid_prism();
    let (regions, features) = fixture::regions_of(&mesh);

    let plans = vec![
        plan_of(
            &regions,
            &features,
            fixture::the_convex_edge(&features),
            true,
        ),
        plan_of(
            &regions,
            &features,
            fixture::the_second_convex_edge(&features),
            true,
        ),
        plan_of(
            &regions,
            &features,
            fixture::the_concave_edge(&features),
            true,
        ),
    ];

    let junction = SurfaceMixedJunction::try_read(
        &plans,
        plans[2].edges[0].edge.from_vertex(),
        the_corner(),
        SurfaceVertexPool::near_enough_for(&mesh.vertices),
    )
    .unwrap_or_else(|reason| panic!("{reason:?}"));

    junction
        .try_apply_to(&plans)
        .unwrap_or_else(|applied| panic!("{applied:?}"));

    let walked =
        surface_face_walkback::try_apply(&mesh, &mesh.get_mesh_edge_graph(), &regions, &plans)
            .unwrap_or_else(|reason| panic!("{reason:?}"));

    assert_within(
        area_on(&walked.mesh, |v| (v.x - 10.0).abs() < 1e-5),
        81.0,
        1e-4,
        "the step wall is shared, so the cut takes its top radius and the bead the radius nearest the void",
    );
    assert_within(
        area_on(&walked.mesh, |v| (v.y - 10.0).abs() < 1e-5),
        81.0,
        1e-4,
        "and so is the void wall now that its own top rim is picked - 91 with only two edges picked",
    );
    assert_within(
        area_on(&walked.mesh, |v| (v.z - 10.0).abs() < 1e-5),
        282.0,
        1e-4,
        "the cap loses a 1 by 9 band to each cut, each bounded on that cut's own handover plane",
    );

    // And both tangencies are real vertices of the walls they stand on, which is what the patch's
    // two end rings are sewn to.
    assert!(
        vertices_on(&walked.mesh, |v| (v.x - 10.0).abs() < 1e-5)
            .iter()
            .any(|&at| (at - Vector3::new(10.0, 11.0, 9.0)).length() < 1e-5),
        "P is the corner the step wall is left with"
    );
    assert!(
        vertices_on(&walked.mesh, |v| (v.y - 10.0).abs() < 1e-5)
            .iter()
            .any(|&at| (at - Vector3::new(11.0, 10.0, 9.0)).length() < 1e-5),
        "and Q the corner the void wall is left with"
    );
}

/// The L's three picked edges as three plans of one edge each, in the order cut, cut, bead, with
/// the vertex they meet at and the part's own ruler.
fn the_three_plans(round_profile: bool) -> (Vec<Rc<SurfaceBevelPlan>>, i32, f64) {
    let mesh = create_l_shaped_grid_prism();
    let (regions, features) = fixture::regions_of(&mesh);

    let plans = vec![
        plan_of(
            &regions,
            &features,
            fixture::the_convex_edge(&features),
            round_profile,
        ),
        plan_of(
            &regions,
            &features,
            fixture::the_second_convex_edge(&features),
            round_profile,
        ),
        plan_of(
            &regions,
            &features,
            fixture::the_concave_edge(&features),
            round_profile,
        ),
    ];

    let vertex = plans[2].edges[0].edge.from_vertex();
    let near_enough = SurfaceVertexPool::near_enough_for(&mesh.vertices);

    (plans, vertex, near_enough)
}

pub fn plan_of(
    regions: &Arc<FaceRegionCache>,
    features: &FeatureEdgeSet,
    edge_index: usize,
    round_profile: bool,
) -> Rc<SurfaceBevelPlan> {
    SurfaceBevelPlan::try_create(
        &fixture::chain_of(features, edge_index),
        regions,
        RADIUS,
        round_profile,
    )
    .unwrap_or_else(|reason| panic!("the plan refused the edge: {reason}"))
}

/// The three edges at the corner, each planned on a chain of its OWN - which is how the junction
/// has to read them, because no one chain holds all three.
fn the_two_readings() -> (SurfaceMixedCorner, SurfaceMixedCorner, f64) {
    let (first_cut, second_cut, bead, near_enough) = the_planned_edges();

    let first = SurfaceMixedCorner::try_read(&first_cut, &bead, the_corner(), RADIUS, near_enough)
        .unwrap_or_else(|reason| panic!("the first cut's corner would not read: {reason:?}"));
    let second =
        SurfaceMixedCorner::try_read(&second_cut, &bead, the_corner(), RADIUS, near_enough)
            .unwrap_or_else(|reason| panic!("the second cut's corner would not read: {reason:?}"));

    (first, second, near_enough)
}

/// The two cuts' strips at the sampling the patch would be built at.
fn the_two_cut_strips(segments: i32) -> (SurfaceStrip, SurfaceStrip) {
    let (first_cut, second_cut, _, _) = the_planned_edges();

    (
        strip_of(&first_cut, segments),
        strip_of(&second_cut, segments),
    )
}

pub fn strip_of(edge: &SurfaceEdgePlan, segments: i32) -> SurfaceStrip {
    SurfaceStrip::try_create(edge, segments)
        .unwrap_or_else(|reason| panic!("the strip refused the edge: {reason:?}"))
}

fn the_planned_edges() -> (SurfaceEdgePlan, SurfaceEdgePlan, SurfaceEdgePlan, f64) {
    let mesh = create_l_shaped_grid_prism();
    let (regions, features) = fixture::regions_of(&mesh);

    (
        plan_of(
            &regions,
            &features,
            fixture::the_convex_edge(&features),
            true,
        )
        .edges[0]
            .clone(),
        plan_of(
            &regions,
            &features,
            fixture::the_second_convex_edge(&features),
            true,
        )
        .edges[0]
            .clone(),
        plan_of(
            &regions,
            &features,
            fixture::the_concave_edge(&features),
            true,
        )
        .edges[0]
            .clone(),
        SurfaceVertexPool::near_enough_for(&mesh.vertices),
    )
}

/// How near two skew lines come to each other.
fn closest_approach(first: SurfaceLine, second: SurfaceLine) -> f64 {
    let between = second.point - first.point;
    let across = first.direction.cross(second.direction);

    between.dot(across.get_normal()).abs()
}
