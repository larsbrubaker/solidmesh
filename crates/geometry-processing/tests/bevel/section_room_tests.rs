//! Port of `SectionRoomTests.cs`: the closed-form room reading - the largest fillet radius the
//! sections construction can build on a closed chain, read off the part without building
//! anything, and TIGHT: the build succeeds just under the number and is refused just over it.

use geometry_processing::bevel::sections::{section_assembly, section_room};
use geometry_processing::bevel::surface::surface_mesh_closure;
use geometry_processing::bevel::{BevelAdvisoryArg, BevelAdvisoryCode};
use geometry_processing::topology::{FeatureEdgeChain, FeatureEdgeSet};
use polygon_mesh::{Mesh, MeshEdgeGraph};
use vector_math::{Matrix4X4, Vector3};

use crate::bore_test_meshes::{
    create_plate_with_a_raked_bore, create_plate_with_a_round_bore, RAKED_RAKE_DEGREES, RAKED_SIDES,
};
use crate::surface_bevel_robustness_tests as robustness;
use crate::{mesh_measurements, rigid_motion_meshes};

const SIDES: i32 = 24;
const BORE_RADIUS: f64 = 3.0;
const THICKNESS: f64 = 10.0;
const LAND: f64 = 6.0;
const SEGMENTS: i32 = 8;
const BEND_THRESHOLD_DEGREES: f64 = 20.0;

/// The straight bore's two limits, each one geometry anyone can check by hand.
#[test]
fn the_straight_bores_room_is_the_land_on_top_and_the_thickness_down_the_wall() {
    let (part, graph, chain) = the_straight_bore();

    let (fits, largest, failure) =
        section_room::try_fit(Some(&part), Some(&graph), Some(&chain), 0.4);
    assert!(
        fits,
        "a 0.4 fillet fits in a {LAND} land - instead {failure:?}"
    );

    // The top side is an OFFSET square out of the mouth across the flat face, at a right-angled
    // wedge so the setback is the radius itself: the ray leaves the fan at the plate's rim, a land
    // away, and the rim's own vertex stands on it - so the limit is the land.
    // The wall side SLIDES down the bore's vertical fold with no lean at all (beta = 0), and that
    // fold runs the whole thickness to the FAR mouth's vertex - the far mouth is not picked, so it
    // costs the radius nothing beyond being where the fold ends: the limit is the thickness.
    // The smaller of the two is the land.
    assert!(
        (largest - LAND).abs() <= 1e-4,
        "the land is {LAND} and the thickness {THICKNESS} - it read {largest}"
    );
}

/// The reading is TIGHT: 0.1 percent under it the construction builds and 0.1 percent over it the
/// construction cannot. Without this the closed form could be any conservative number at all and
/// still pass the reading above.
#[test]
fn the_straight_bores_reading_is_tight_to_a_tenth_of_a_percent() {
    let (part, graph, chain) = the_straight_bore();

    let (_, largest, _) = section_room::try_fit(Some(&part), Some(&graph), Some(&chain), 0.0);

    let built = section_assembly::try_build(
        Some(&part),
        Some(&graph),
        Some(&chain),
        0.999 * largest,
        SEGMENTS,
    )
    .unwrap_or_else(|why_not| {
        panic!("a tenth of a percent inside the room the build stands - instead {why_not:?}")
    });

    assert_eq!(
        surface_mesh_closure::open_edge_count(&built).0,
        0,
        "and stands closed, not merely without a refusal"
    );

    let over = 1.001 * largest;

    let (fits, again, refusal) =
        section_room::try_fit(Some(&part), Some(&graph), Some(&chain), over);
    assert!(!fits, "a tenth of a percent outside it does not fit");

    let refusal = refusal.expect("a refusal names why");
    assert_eq!(
        refusal.code(),
        BevelAdvisoryCode::SizeExceedsFaceWidth,
        "a setback wider than its face is the legacy path's own sentence for this"
    );

    let quoted = the_double(&refusal.args()[2]);
    assert!(
        (quoted - largest).abs() <= 1e-9,
        "the advisory quotes the most that fits, which is what the panel shows the user - {quoted} against {largest}"
    );

    assert!((again - largest).abs() <= 1e-9);

    // And the construction itself agrees that the room has run out there - the room reading is
    // not wired into the assembly (the router owns that), so this is two independent readings of
    // one limit rather than one reading twice.
    assert!(
        !stands(&part, &graph, &chain, over),
        "past the room the construction is refused or folds"
    );
}

/// The raked bore, where the wall's fold is longer than the plate is thick (thickness / cos rake)
/// and the lean beta is not zero at most vertices, so the limit is a minimum over the loop and not
/// one number repeated.
#[test]
fn the_raked_bores_reading_is_tight_to_a_tenth_of_a_percent_too() {
    let (part, graph, chain) = the_raked_bore();

    let (fits, largest, failure) =
        section_room::try_fit(Some(&part), Some(&graph), Some(&chain), 0.0);
    assert!(fits, "the raked mouth reads - instead {failure:?}");

    // The binding station is the WALL's slide, not the top face: the bore's fold runs
    // thickness / cos(rake) = 5.6569 from mouth to mouth, and at the shallow end of the ellipse the
    // wedge between the normals is 135 degrees, so the setback is tan(67.5) = 2.4142 per unit of
    // radius and the fold is used up at 5.6569 / 2.4142. The lean there is 1 - that end's fold is
    // square to the way the boundary reaches - and elsewhere round the loop the setback is smaller
    // and the lean costs less than the setback saves, which is why one vertex names the answer.
    assert!(
        (largest - 2.34313).abs() <= 1e-4,
        "the raked wall's fold runs out first - it read {largest}"
    );

    let built = section_assembly::try_build(
        Some(&part),
        Some(&graph),
        Some(&chain),
        0.999 * largest,
        SEGMENTS,
    )
    .unwrap_or_else(|why_not| {
        panic!("a tenth of a percent inside the raked room the build stands - instead {why_not:?}")
    });

    assert_eq!(surface_mesh_closure::open_edge_count(&built).0, 0);

    let over = 1.001 * largest;

    let (fits, _, refusal) = section_room::try_fit(Some(&part), Some(&graph), Some(&chain), over);
    assert!(!fits);

    let refusal = refusal.expect("a refusal names why");
    let quoted = the_double(&refusal.args()[2]);
    assert!(
        (quoted - largest).abs() <= 1e-9,
        "{quoted} against {largest}"
    );

    assert!(
        !stands(&part, &graph, &chain, over),
        "past the raked room the construction is refused or folds"
    );
}

/// Whether the build at `radius` stands: built, closed, one shell and manifold.
fn stands(part: &Mesh, graph: &MeshEdgeGraph, chain: &FeatureEdgeChain, radius: f64) -> bool {
    match section_assembly::try_build(Some(part), Some(graph), Some(chain), radius, SEGMENTS) {
        Ok(past) => {
            surface_mesh_closure::open_edge_count(&past).0 == 0
                && mesh_measurements::shell_count(&past) == 1
                && rigid_motion_meshes::most_faces_on_an_edge(&past) == 2
        }
        Err(_) => false,
    }
}

/// C#'s `(double)refusal.Args[i]`.
fn the_double(arg: &BevelAdvisoryArg) -> f64 {
    match arg {
        BevelAdvisoryArg::Double(value) => *value,
        other => panic!("the argument is a number - instead {other:?}"),
    }
}

fn the_straight_bore() -> (Mesh, MeshEdgeGraph, FeatureEdgeChain) {
    let part = create_plate_with_a_round_bore(SIDES, BORE_RADIUS, THICKNESS, LAND);
    let graph = part.get_mesh_edge_graph();
    let features = FeatureEdgeSet::create_with_graph(&part, &graph, BEND_THRESHOLD_DEGREES);
    let chain = FeatureEdgeChain::propagate(&features, the_upper_mouth(&features))
        .expect("the mouth propagates");

    assert!(
        chain.is_closed() && chain.count() == SIDES as usize,
        "the fixture's mouth walks as a closed loop of {SIDES}, and it walked {} closed {}",
        chain.count(),
        chain.is_closed()
    );

    (part, graph, chain)
}

fn the_raked_bore() -> (Mesh, MeshEdgeGraph, FeatureEdgeChain) {
    // The robustness harness's own dimensions for the ONE raked case that reaches a reading there -
    // 45 degrees and forty sides - so this suite measures the room of a chain that is known to
    // build rather than of one that is refused at the section rule for a different cause.
    let part = create_plate_with_a_raked_bore(
        RAKED_SIDES,
        robustness::RADIUS,
        robustness::THICKNESS,
        RAKED_RAKE_DEGREES,
        6.0,
    );
    let graph = part.get_mesh_edge_graph();
    let features =
        FeatureEdgeSet::create_with_graph(&part, &graph, robustness::BEND_THRESHOLD_DEGREES);
    let chain = FeatureEdgeChain::propagate(
        &features,
        robustness::the_upper_mouth(&features, Matrix4X4::IDENTITY, RAKED_RAKE_DEGREES),
    )
    .expect("the raked mouth propagates");

    assert!(
        chain.is_closed(),
        "the raked mouth walks closed, and it walked {} open",
        chain.count()
    );

    (part, graph, chain)
}

fn the_upper_mouth(features: &FeatureEdgeSet) -> i32 {
    for i in 0..features.count() {
        let edge = &features.edges()[i];

        if on_the_mouth(edge.start) && on_the_mouth(edge.end) {
            return i as i32;
        }
    }

    panic!("the fixture has no upper mouth");
}

fn on_the_mouth(at: Vector3) -> bool {
    (at.z - (THICKNESS / 2.0)).abs() < 1e-6
        && (Vector3::new(at.x, at.y, 0.0).length() - BORE_RADIUS).abs() < 1e-4
}
