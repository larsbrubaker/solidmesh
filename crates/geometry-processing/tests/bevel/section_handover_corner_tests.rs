//! Port of `SectionHandoverCornerTests.cs`: THE MIXED HANDOVER CORNER of "Every Chamfer Works.mcx"
//! on the part's own mesh ([`crate::handover_corner_test_mesh`]) - the mitred side and the barrel
//! picked together, whose rims hand over at (-10.77, 28, 0) where the bottom cap's rim chord turns
//! straight up the concave seam.
//!
//! The router declines that corner (unequal wedges on a shared smooth side, convex into concave) and
//! legacy builds the pair; the two canals' axes meet in a plane 1.26 radii off the vertex, so the
//! corner is not a mitre; legacy answers it with the cap-riding run-out and refuses the dual canal.
//! Built through the shared `SurfaceTestBuild` port with the sections flag pinned on.
//!
//! Skipped as in C# (`[Skip]`):
//! - `ThePairIsRoutedToTheSectionsAndBuildsClosed`: "RED: the sections have no mixed handover
//!   corner - unequal wedges on a shared smooth side, convex into concave, and no run-out past it
//!   (SurfaceMixedCornerRunOut for legacy)".

use std::rc::Rc;

use geometry_processing::bevel::sections::{section_corner, section_mixed_corner, section_routing};
use geometry_processing::bevel::surface::{
    surface_mixed_corner_readings, surface_mixed_corner_run_out, surface_mixed_corner_which_canal,
    SurfaceBevelPlan, SurfaceMixedCornerDualCanal,
};
use geometry_processing::bevel::BevelAdvisory;
use geometry_processing::topology::{ChainEdge, EdgeConvexity};
use polygon_mesh::CancelToken;
use vector_math::math_helper::radians_to_degrees;
use vector_math::Vector3;

use crate::handover_corner_test_mesh as fixture;
use crate::section_turn_readings::{feature_dihedral_degrees, the_creases_at};
use crate::surface_test_build::{pinned_to_the_sections, station, try_bevel, TryBevelOptions};

/// THE ROUTER'S OWN DECISION at the handover corner, with the flag on: it declines the barrel's
/// plan by clause and by vertex, and the fallback to legacy is what builds the pair.
#[test]
fn the_router_declines_the_handover_corner_and_legacy_builds_the_pair() {
    let built = build_with_the_flag_on();

    assert_eq!(
        built.route.as_deref(),
        Some("legacy: the chain turns 90 degrees at vertex 6"),
        "the router reads the handover corner itself and will not take it to the sections"
    );

    assert_eq!(built.plans.len(), 2, "one rim for each pick");

    let mut counts: Vec<usize> = built.plans.iter().map(|plan| plan.edges.len()).collect();
    counts.sort_unstable();
    assert_eq!(
        counts,
        vec![4, 60],
        "the mitred side's four edge rim and the barrel's sixty"
    );

    if let Err(reason) = &built.result {
        panic!(
            "legacy builds what the sections decline - {:?} at {}",
            reason.as_ref().map(BevelAdvisory::code),
            station(reason.as_ref())
        );
    }
}

/// THE READING of the corner the app's refusal names - every number in the class remarks is
/// asserted here.
#[test]
fn the_corner_is_a_mixed_handover_on_the_barrels_own_facet() {
    let part = fixture::the_part();
    let built = build_with_the_flag_on();

    let (arriving, leaving) = the_turn_at_the_handover_corner(&built.plans);

    assert!(
        (leaving.from() - fixture::THE_HANDOVER_CORNER).length() < 0.005,
        "this is the station the app's three refusals name"
    );

    assert!(
        (section_corner::turn_degrees(&arriving, &leaving) - 90.0).abs() <= 1e-6,
        "the cap's rim chord turns straight up the seam"
    );

    assert!(
        ((arriving.to() - arriving.from()).length() - 1.12115).abs() <= 1e-4,
        "the arriving edge is one chord of the bottom cap's rim"
    );

    assert!(
        ((leaving.to() - leaving.from()).length() - 20.0).abs() <= 1e-6,
        "the leaving edge is the whole seam, floor to top"
    );

    // THE SHARED SIDE IS THE BARREL - one smooth surface, and NOT a flat one.
    assert!(
        (arriving
            .right_normal()
            .get_normal()
            .dot(leaving.right_normal().get_normal())
            - 1.0)
            .abs()
            <= 1e-6,
        "both edges' right side is the same cylinder facet"
    );

    assert!(
        arriving
            .left_normal()
            .get_normal()
            .dot(leaving.left_normal().get_normal())
            .abs()
            <= 1e-6,
        "the two other sides - the floor and the cube's y = 28 wall - are square to each other"
    );

    // THE WEDGES DIFFER, which is the clause that declines the corner.
    let arriving_wedge = arriving
        .left_normal()
        .get_normal()
        .dot(arriving.right_normal().get_normal());
    let leaving_wedge = leaving
        .left_normal()
        .get_normal()
        .dot(leaving.right_normal().get_normal());

    assert!(
        arriving_wedge.abs() <= 1e-6,
        "the cap's rim is a square fold, floor against barrel"
    );

    assert!(
        (leaving_wedge - 0.23345).abs() <= 1e-4,
        "the seam folds 76.5 degrees, cube wall against barrel"
    );

    assert!(
        (arriving_wedge - leaving_wedge).abs() > 1e-4,
        "SectionCorner.TryReadTheSides' SameWedgeToWithin is 1e-4, and this is 0.233 out"
    );

    assert!(
        !section_corner::looks_like_a_corner(&arriving, &leaving),
        "so the corner rule declines it on UNEQUAL WEDGES - one profile cannot serve both"
    );

    assert!(
        section_corner::is_sharp_turn(&arriving, &leaving),
        "and it is a sharp turn, so there is nothing else for the router to call it"
    );

    // THE CREASES LEAVING THE VERTEX: two on the floor and the seam, convex meeting concave.
    let creases = the_creases_at(
        &part,
        leaving.from_vertex(),
        fixture::BEND_THRESHOLD_DEGREES,
    );

    assert_eq!(
        creases.len(),
        3,
        "the cube's own bottom edge, the cap's rim chord and the seam"
    );

    let seams: Vec<_> = creases
        .iter()
        .filter(|crease| crease.direction.get_normal().z.abs() > 0.999)
        .collect();
    assert_eq!(seams.len(), 1);
    let seam = seams[0];

    assert!((feature_dihedral_degrees(seam) - 76.49999).abs() <= 1e-3);

    assert_eq!(
        seam.convexity,
        EdgeConvexity::Concave,
        "the seam is where the cylinder runs INTO the cube - a concave fold"
    );

    for on_the_floor in creases
        .iter()
        .filter(|crease| crease.direction.get_normal().z.abs() < 1e-6)
    {
        assert!((feature_dihedral_degrees(on_the_floor) - 90.0).abs() <= 1e-3);

        assert_eq!(
            on_the_floor.convexity,
            EdgeConvexity::Convex,
            "the two edges on the floor are convex - so the handover is MIXED"
        );
    }
}

/// THE TWO CANALS at the mixed handover corner - read with the sections' own one rule for a centre,
/// convex station and concave alike. Their axes MEET, so their surfaces cut each other in a plane
/// curve; the plane stands 1.26 radii off the vertex and six chords along the arriving edge, so the
/// corner is not a mitre. Every number here is the reading `SectionMixedCorner` carries.
#[test]
fn the_two_canals_axes_meet_and_the_mitre_is_nowhere_near_the_corner() {
    let built = build_with_the_flag_on();
    let (arriving, leaving) = the_turn_at_the_handover_corner(&built.plans);
    let radius = fixture::SIZE;
    let vertex = leaving.from();

    // BOTH BALLS ARE REALLY TANGENT to the three faces - the concave one needs no flipped sign.
    let convex = section_corner::the_axis_at(&arriving, vertex, radius);
    let concave = section_corner::the_axis_at(&leaving, vertex, radius);

    assert!(
        ((convex - vertex).dot(arriving.right_normal().get_normal()) - -radius).abs() <= 1e-9,
        "the convex canal stands a radius inside the barrel"
    );

    assert!(
        ((concave - vertex).dot(leaving.right_normal().get_normal()) - -radius).abs() <= 1e-9,
        "and so does the concave one - the same minus, no branch on convexity"
    );

    assert!(
        ((concave - vertex).dot(leaving.left_normal().get_normal()) - -radius).abs() <= 1e-9,
        "a radius off the cube's wall as well, with its centre across the crease"
    );

    // THE AXES MEET, so the two cylinders cut each other in a PLANE curve.
    assert!(
        section_mixed_corner::how_skew_the_axes_are(&arriving, &leaving, radius) < 1e-9,
        "both axes lie in the one plane standing a radius inside the shared barrel facet"
    );

    let (through, normal) = section_corner::try_the_mitre_plane(&arriving, &leaving, radius)
        .expect("the two canals have a mitre plane");

    assert!(
        ((through - vertex).length() - 6.08159).abs() <= 1e-4,
        "but they meet 1.62 radii away from the corner"
    );

    // AND THE PLANE IS NOWHERE NEAR THE CORNER, which is why this is not a mitre.
    assert!(
        ((vertex - through).dot(normal).abs() - 4.74983).abs() <= 1e-4,
        "the vertex stands 1.26 radii off the plane the two canals cut each other in"
    );

    let slide = -(vertex - through).dot(normal) / arriving.direction().get_normal().dot(normal);

    assert!(
        (slide - 6.71728).abs() <= 1e-4,
        "so the arriving profile would have to slide 6.72 mm to land on it"
    );

    assert!(
        slide > 5.0 * (arriving.to() - arriving.from()).length(),
        "and the arriving edge is 1.12 mm long - the mitre would eat six chords of the rim"
    );

    // THE SHARED SIDE does have one corner point, and it stands on that plane exactly.
    let (on_the_barrel, how_far_apart) =
        section_mixed_corner::try_where_the_shared_side_boundaries_meet(
            &arriving, &leaving, radius,
        )
        .expect("the shared side's two boundaries meet");

    assert!(
        how_far_apart < 1e-9,
        "the two canals' tangency lines on the barrel really do cross"
    );

    assert!((on_the_barrel - Vector3::new(-11.4636, 30.8793, 3.7562)).length() < 1e-3);

    assert!(
        (on_the_barrel - through).dot(normal).abs() < 1e-9,
        "and that crossing lies on the plane the two canals cut each other in"
    );

    // THE FAR ENDS DO NOT MEET: the patch of floor and wall between them is what is owed.
    let (fold, convex_reach, concave_reach) =
        section_mixed_corner::try_where_each_ends_on_the_fold(&arriving, &leaving, radius)
            .expect("each canal ends on the fold");

    assert!(
        (fold.dot(Vector3::new(-1.0, 0.0, 0.0)) - 1.0).abs() <= 1e-9,
        "the floor and the cube's wall fold along the cube's own bottom edge"
    );

    assert!(
        (convex_reach - -3.86289).abs() <= 1e-4,
        "the convex canal's floor tangency crosses the fold at a radius' lean"
    );

    assert!(
        (concave_reach - -2.96112).abs() <= 1e-4,
        "the concave canal's wall tangency at r / tan(wedge / 2) for 76.5 degrees"
    );

    assert!(
        ((convex_reach - concave_reach).abs() - 0.90178).abs() <= 1e-4,
        "0.24 of a radius apart - there is no one point to end both strips at, and that gap is \
         the patch SurfaceMixedCornerDualCanal fades out across"
    );
}

/// WHAT LEGACY ACTUALLY DRAWS at the handover corner, read through its own corner reading rather
/// than guessed from the class names: the cap-riding corner with a 60 degree run-out, the dual canal
/// readable and refused. Every number in the class remarks' three legacy paragraphs is asserted
/// here.
#[test]
fn legacy_answers_the_corner_with_a_cap_riding_run_out_and_refuses_the_dual_canal() {
    let part = fixture::the_part();
    let built = build_with_the_flag_on();

    assert!(built.result.is_ok(), "legacy builds this pair");

    let (barrel, corner) = the_barrel_plan_and_the_handover_index(&built.plans);

    let (mixed_corners, _) =
        surface_mixed_corner_readings::try_read_all(&barrel.edges, true, fixture::SIZE, &part)
            .unwrap_or_else(|why| panic!("{:?}", why.code()));

    let mut keys: Vec<i32> = mixed_corners.keys().copied().collect();
    keys.sort_unstable();
    assert_eq!(
        keys,
        vec![1, 2, 31, 32],
        "the barrel's rim turns over four times, and the handover is the first"
    );

    let mixed = &mixed_corners[&corner];

    // THE CUT IS NOT THE CHORD THAT TOUCHES: the blend hands over on the mitre ring before it.
    assert!(mixed.hands_over_at_a_mitre);

    assert!(
        (mixed.cut.edge.from() - Vector3::new(-9.0902, 24.1221, 0.0)).length() < 1e-3,
        "the cut is the rim chord TWO back - the chords between go under the blend"
    );

    assert_eq!(mixed.bead.convexity(), EdgeConvexity::Concave);
    assert!(
        ((mixed.bead.edge.to() - mixed.bead.edge.from()).length() - 20.0).abs() <= 1e-6,
        "the bead is the whole seam"
    );

    assert!(
        (radians_to_degrees(mixed.turn_radians()) - 67.5).abs() <= 1e-3,
        "67.49998 - the barrel's facets carry the mesh's own float rounding"
    );

    // THE DUAL IS READABLE AND REFUSED - it is a LEANING corner's answer, and this cap does not
    // lean.
    let dual = SurfaceMixedCornerDualCanal::try_read(mixed, f64::INFINITY)
        .unwrap_or_else(|why| panic!("the geometry of the dual exists here - {:?}", why.code()));

    assert!((radians_to_degrees(dual.the_cut_strip_ends_at()) - 90.0).abs() <= 1e-4);
    assert!((radians_to_degrees(dual.dies_at) - 150.0).abs() <= 1e-3);
    assert!(
        (dual.reach_along_the_crease() - 13.54919).abs() <= 1e-4,
        "3.6 radii along a crease running (1, 0, 0) out of the vertex"
    );

    assert!(!mixed.the_cap_leans());
    assert!(
        !surface_mixed_corner_which_canal::the_corner_takes_the_dual(mixed),
        "so legacy publishes none of the dual's rails"
    );

    assert!(
        surface_mixed_corner_which_canal::try_how_far_the_duals_crease_runs(mixed).is_none(),
        "the part carries no edge the way the dual would travel"
    );

    // WHAT IT DOES DRAW: the cap-riding run-out, 60 degrees on, dying r sqrt 3 along the fold.
    let crease = surface_mixed_corner_which_canal::try_the_cap_riding_crease(mixed)
        .expect("the cap-riding crease");

    assert!(
        (crease.direction.dot(Vector3::new(-1.0, 0.0, 0.0)) - 1.0).abs() <= 1e-9,
        "the cap-riding blend runs out along the cube's own bottom fold"
    );

    let reach = surface_mixed_corner_which_canal::try_how_far_the_cap_riding_crease_runs(mixed)
        .expect("how far the cap-riding crease runs");

    assert!(
        (reach - 10.22771).abs() <= 1e-4,
        "and that fold is long enough to take the run-out"
    );

    assert!(
        (radians_to_degrees(surface_mixed_corner_run_out::clears_the_wall()) - 60.0).abs() <= 1e-9,
        "r / cos(phi) reaches 2r at sixty degrees, whatever the wedge"
    );

    assert!(
        (fixture::SIZE * 3f64.sqrt() - 6.50586).abs() <= 1e-4,
        "so the run-out's column closes to a point r sqrt 3 along the fold"
    );
}

/// What C#'s `BuildWithTheFlagOn` hands back through its `out`s.
struct FlagOnBuild {
    plans: Vec<Rc<SurfaceBevelPlan>>,
    /// The mesh, or the advisory (`None` where nothing was named).
    result: Result<polygon_mesh::Mesh, Option<BevelAdvisory>>,
    route: Option<String>,
}

/// C# `BuildWithTheFlagOn`: the two picks built the part builder's own way, with the sections flag
/// on.
fn build_with_the_flag_on() -> FlagOnBuild {
    let part = fixture::the_part();

    let _sections = pinned_to_the_sections();

    let outcome = try_bevel(
        &part,
        &[fixture::the_mitred_side(), fixture::the_barrel()],
        fixture::SIZE,
        fixture::SEGMENTS,
        &CancelToken::new(),
        TryBevelOptions::default(),
    );

    FlagOnBuild {
        plans: outcome.plans_built,
        result: outcome
            .result
            .map(|built| built.mesh)
            .map_err(|failure| failure.advisory().cloned()),
        route: section_routing::last_route(),
    }
}

/// C# `TheBarrelPlanAndTheHandoverIndex`: the barrel's own plan and the index of the turn that
/// stands on the handover corner.
fn the_barrel_plan_and_the_handover_index(
    plans: &[Rc<SurfaceBevelPlan>],
) -> (Rc<SurfaceBevelPlan>, i32) {
    for plan in plans {
        let count = plan.edges.len();

        for i in 0..count {
            if (plan.edges[i].edge.from() - fixture::THE_HANDOVER_CORNER).length() < 0.02 {
                // The corner is INDEXED by the edge that arrives at it, which is the one before.
                return (Rc::clone(plan), ((i + count - 1) % count) as i32);
            }
        }
    }

    panic!(
        "no planned turn stands at {:?}",
        fixture::THE_HANDOVER_CORNER
    );
}

/// C# `TheTurnAtTheHandoverCorner`: the one planned turn that stands on the station the app's
/// refusals name.
fn the_turn_at_the_handover_corner(plans: &[Rc<SurfaceBevelPlan>]) -> (ChainEdge, ChainEdge) {
    for plan in plans {
        let count = plan.edges.len();

        for i in 0..count {
            let arriving = plan.edges[(i + count - 1) % count].edge;
            let leaving = plan.edges[i].edge;

            if (leaving.from() - fixture::THE_HANDOVER_CORNER).length() < 0.02 {
                return (arriving, leaving);
            }
        }
    }

    panic!(
        "no planned turn stands at {:?}",
        fixture::THE_HANDOVER_CORNER
    );
}
