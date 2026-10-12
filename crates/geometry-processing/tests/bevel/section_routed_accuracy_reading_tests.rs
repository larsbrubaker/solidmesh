//! Port of `SectionRoutedAccuracyReadingTests.cs`: what the ACCURACY readings that move when
//! `section_routing::use_sections` is on actually read, measured on the routed build. Two are pins
//! of the routed triangulation (the raked mouth's vertex counts); the others were one fault read
//! several ways - the routed blend standing proud of the rolling ball's canal, a station's profile
//! swung as a circle about a lerped centre where the canal is the ellipse two crossing cylinders
//! cut - and read as near zero as a float mesh holds now that `SectionBuilder` places the interior
//! samples on that ellipse.
//!
//! Every test pins the sections router ON (`pinned_to_the_sections`), as C#'s `try` blocks set it.
//! The bore readings are in `surface_bore_mouth_tests.rs`, the eaten chord's build in
//! `surface_mitre_consumes_a_chord_tests.rs`.

use geometry_processing::bevel::sections::section_routing;
use geometry_processing::bevel::surface::SurfaceLine;
use geometry_processing::topology::{
    bevel_edge_group_resolver, BevelSelectionKind, FaceGroup, FeatureEdgeSet,
};
use polygon_mesh::CancelToken;
use vector_math::Vector3;

use crate::bore_test_meshes::{create_plate_with_a_raked_bore, create_plate_with_a_round_bore};
use crate::mesh_measurements::signed_volume;
use crate::surface_bore_mouth_tests::{
    centre_lines, the_raked_bores_facet_planes, the_upper_mouth, the_upper_raked_mouth,
};
use crate::surface_mitre_consumes_a_chord_tests;
use crate::surface_raked_bore_mouth_tests::{RAKED_RADIUS, RAKED_THICKNESS, RAKE_DEGREES};
use crate::surface_test_build::{pinned_to_the_sections, try_bevel, TryBevelOptions};

/// How near the canal a published blend vertex can be read to stand: the mesh keeps its vertices
/// as floats, so a radius of three is only good to about 3e-7 whatever the construction does in
/// double.
const ON_THE_CANAL_AS_A_FLOAT_HOLDS_IT: f64 = 1e-6;

/// The straight bore's mouth, routed: the blend is 280 vertices - forty chords by the profile's
/// seven interior rows - and every one of them stands a radius from its own chord's rolling ball
/// centre line, where the circle-about-a-lerped-centre construction left them up to 3.046e-4 proud.
#[test]
fn the_straight_bores_routed_blend_stands_proud_of_the_canal_by_a_per_row_constant() {
    let _sections = pinned_to_the_sections();

    let part = create_plate_with_a_round_bore(40, 3.0, 10.0, 6.0);
    let features = FeatureEdgeSet::create_with_graph(&part, &part.get_mesh_edge_graph(), 10.0);
    let group = bevel_edge_group_resolver::create_from_edge(
        &features,
        the_upper_mouth(&features),
        BevelSelectionKind::TangentChain,
        FaceGroup::DEFAULT_CURVE_TOLERANCE_DEGREES,
    )
    .expect("the mouth's edge is in the set");

    let outcome = try_bevel(
        &part,
        &[group],
        0.4,
        8,
        &CancelToken::new(),
        TryBevelOptions::default(),
    );
    let mesh = match outcome.mesh_or_reason() {
        Ok(mesh) => mesh,
        Err(reason) => panic!("{:?}", reason.map(|reason| reason.code())),
    };

    assert_eq!(section_routing::last_route().as_deref(), Some("sections"));

    let lines = centre_lines(&features);
    let blend: Vec<Vector3> = mesh
        .vertices
        .iter()
        .map(|vertex| vertex.as_vector3())
        .filter(|at| at.z < 5.0 - 1e-5 && at.z > 5.0 - 0.4 + 1e-5)
        .collect();

    assert_eq!(
        blend.len(),
        280,
        "the routed mouth is one station per chord and the profile's seven interior rows - forty by \
         seven - where the legacy strips published far more"
    );

    // Per ROW, not per vertex: a row's forty radii agree to 1e-6, so any miss would be the whole
    // row's and the reading is of the profile's placement, not of the loft between stations. C#'s
    // GroupBy(Math.Round(z, 5)) keeps first-seen order; only the grouping matters here.
    let mut rows: Vec<(f64, Vec<f64>)> = Vec::new();

    for at in &blend {
        let key = round_to(at.z, 5);
        let radius = Vector3::new(at.x, at.y, 0.0).length();

        match rows.iter_mut().find(|(row, _)| *row == key) {
            Some((_, radii)) => radii.push(radius),
            None => rows.push((key, vec![radius])),
        }
    }

    for (row, radii) in &rows {
        let most = radii.iter().copied().fold(f64::NEG_INFINITY, f64::max);
        let least = radii.iter().copied().fold(f64::INFINITY, f64::min);

        assert!(
            most - least < 1e-5,
            "the row at z {row} is an exact circle, so its miss is the row's own and not a facet's \
             - it spans {:e}",
            most - least
        );
    }

    let worst = blend
        .iter()
        .map(|&at| (nearest(&lines, at).distance_to(at) - 0.4).abs())
        .fold(f64::NEG_INFINITY, f64::max);

    assert!(
        worst < ON_THE_CANAL_AS_A_FLOAT_HOLDS_IT,
        "every interior sample stands a radius from the canal's own axis now that the section is \
         drawn as the ellipse two crossing cylinders cut - it misses by {worst:e}, where the circle \
         bulged 3.0e-4"
    );
}

/// The raked bore's mouth, routed: the blend proper is 254 vertices, not the legacy 1175. A pure
/// pin of the triangulation - the routed mesh is 480 vertices all told where the legacy BAND alone
/// held 1301.
#[test]
fn the_raked_bores_routed_blend_is_a_far_coarser_mesh_than_the_legacy_strips() {
    let _sections = pinned_to_the_sections();

    let part = create_plate_with_a_raked_bore(40, RAKED_RADIUS, RAKED_THICKNESS, RAKE_DEGREES, 6.0);
    let features = FeatureEdgeSet::create_with_graph(&part, &part.get_mesh_edge_graph(), 10.0);
    let group = bevel_edge_group_resolver::create_from_edge(
        &features,
        the_upper_raked_mouth(&features),
        BevelSelectionKind::TangentChain,
        FaceGroup::DEFAULT_CURVE_TOLERANCE_DEGREES,
    )
    .expect("the mouth's edge is in the set");

    let outcome = try_bevel(
        &part,
        &[group],
        0.4,
        8,
        &CancelToken::new(),
        TryBevelOptions::default(),
    );
    let mesh = match outcome.mesh_or_reason() {
        Ok(mesh) => mesh,
        Err(reason) => panic!("{:?}", reason.map(|reason| reason.code())),
    };

    assert_eq!(section_routing::last_route().as_deref(), Some("sections"));
    assert_eq!(
        mesh.vertices.len(),
        480,
        "a triangulation pin and nothing else: the routed mouth's WHOLE mesh is smaller than the \
         legacy band alone"
    );

    let top = RAKED_THICKNESS / 2.0;
    let facets = the_raked_bores_facet_planes(&features);
    let band: Vec<Vector3> = mesh
        .vertices
        .iter()
        .map(|vertex| vertex.as_vector3())
        .filter(|at| at.z < top - 1e-5 && at.z > top - 0.4 + 1e-5)
        .collect();
    let blend = band
        .iter()
        .filter(|&&at| {
            !facets
                .iter()
                .any(|&(point, normal)| (at - point).dot(normal).abs() <= 1e-5)
        })
        .count();

    assert_eq!(
        band.len(),
        273,
        "the routed band, where the legacy one was 1301"
    );
    assert_eq!(
        blend, 254,
        "the routed blend proper, where the legacy one was 1175 - this is the number the flip \
         re-pins ARakedBoresMouthIsRoundedAllTheWayRound to"
    );
}

/// The eaten chord, routed: the staircase mitre holds the same solid as the boolean build to
/// 1.4e-4 of it - the one reading whose sign runs the OTHER way, taking slightly MORE.
#[test]
fn the_eaten_chords_routed_solid_parts_from_the_booleans_by_a_thousandth_of_a_per_cent() {
    let _sections = pinned_to_the_sections();

    let (_, result) = surface_mitre_consumes_a_chord_tests::build(2.0);
    let built = match result {
        Ok(built) => built.mesh,
        Err(failure) => panic!("{:?}", failure.advisory().map(|reason| reason.code())),
    };

    assert_eq!(section_routing::last_route().as_deref(), Some("sections"));

    let solid = signed_volume(&built);

    assert!(
        (solid - 3084.605151).abs() <= 0.005,
        "the routed staircase holds the same solid to 1.4e-4 of it - it reads {solid:.6}"
    );
}

/// The centre line a blend vertex belongs to, which is the one it is nearest a radius from.
fn nearest(lines: &[SurfaceLine], at: Vector3) -> &SurfaceLine {
    lines
        .iter()
        .min_by(|a, b| {
            (a.distance_to(at) - 0.4)
                .abs()
                .total_cmp(&(b.distance_to(at) - 0.4).abs())
        })
        .expect("the mouth has centre lines")
}

/// C#'s `Math.Round(value, digits)`: banker's rounding at a decimal place.
fn round_to(value: f64, digits: i32) -> f64 {
    let scale = 10f64.powi(digits);

    (value * scale).round_ties_even() / scale
}
