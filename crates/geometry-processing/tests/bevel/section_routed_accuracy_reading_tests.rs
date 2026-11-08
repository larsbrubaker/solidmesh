//! What the ACCURACY readings that move when the sections are routed (`BevelRouting::sections`)
//! actually read, measured on the routed build. Two are pins
//! of the routed triangulation (the raked mouth's vertex counts); the others were one fault read
//! several ways - the routed blend standing proud of the rolling ball's canal, a station's profile
//! swung as a circle about a lerped centre where the canal is the ellipse two crossing cylinders
//! cut - and read as near zero as a float mesh holds now that `SectionBuilder` places the interior
//! samples on that ellipse.
//!
//! Every test pins the sections router ON (`pinned_to_the_sections`), and puts it back however it
//! leaves. The bore readings are in `surface_bore_mouth_tests.rs`, the eaten chord's build in
//! `surface_mitre_consumes_a_chord_tests.rs`.

use std::sync::Arc;

use solidmesh::bevel::sections::section_routing;
use solidmesh::bevel::surface::SurfaceLine;
use solidmesh::topology::{
    bevel_edge_group_resolver, BevelSelectionKind, FaceGroup, FeatureEdgeSet,
};
use solidmesh_math::Vector3;
use solidmesh_mesh::CancelToken;

use crate::bore_test_meshes::{create_plate_with_a_raked_bore, create_plate_with_a_round_bore};
use crate::mesh_measurements::signed_volume;
use crate::surface_bore_mouth_tests::{
    centre_lines, the_raked_bores_facet_planes, the_upper_mouth, the_upper_raked_mouth,
};
use crate::surface_cone_rim_tests;
use crate::surface_mitre_consumes_a_chord_tests;
use crate::surface_raked_bore_mouth_tests::{RAKED_RADIUS, RAKED_THICKNESS, RAKE_DEGREES};
use crate::surface_separate_rim_picks_tests;
use crate::surface_test_build::{pinned_to_the_sections, try_bevel, TryBevelOptions};
use crate::topology_test_meshes::create_cylinder;

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
    // row's and the reading is of the profile's placement, not of the loft between stations. Rows
    // group by z rounded to 5 places, in first-seen order; only the grouping matters here.
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

/// The cone rim, routed: the routed rim takes the ring its own tessellated profile sweeps to within
/// a boundary row's miss of the analytic solid.
#[test]
fn the_cones_routed_rim_leaves_material_the_analytic_fillet_takes() {
    let _sections = pinned_to_the_sections();

    let built = surface_cone_rim_tests::build(1.5, true, 8);
    let profile = surface_cone_rim_tests::profile(1.5, true, 8);

    let removed = surface_cone_rim_tests::frustum_volume() - signed_volume(&built);
    let analytic = surface_cone_rim_tests::removed_volume(&profile);

    assert_eq!(section_routing::last_route().as_deref(), Some("sections"));

    assert!(
        (removed - 9.368221).abs() <= 1e-4,
        "the routed rim takes the ring its own tessellated profile sweeps to within {:.6} mm^3 of \
         {analytic:.6}, where the circle-about-a-lerped-centre section left 0.274 standing. What \
         is left is the BOUNDARY rows', not the interior samples' - it reads {removed:.6}",
        analytic - removed
    );
}

/// The cylinder band, routed: the interior samples are ON the canal and what is left is the two
/// BOUNDARY rows, set back from the edge line instead of from where two stations' tangent lines
/// cross.
#[test]
fn the_cylinder_bands_routed_vertices_stand_off_the_rolling_ball_circle_by_the_same_fault() {
    let _sections = pinned_to_the_sections();

    let part = create_cylinder(9.0, 3.0, 24);
    let features = Arc::new(FeatureEdgeSet::create(&part));
    let run: Vec<_> = surface_separate_rim_picks_tests::the_top_rim(&features)
        .into_iter()
        .take(24)
        .collect();

    let (built, _) = surface_separate_rim_picks_tests::built(
        &part,
        &surface_separate_rim_picks_tests::each_on_its_own(&features, &run),
        true,
    );

    assert_eq!(section_routing::last_route().as_deref(), Some("sections"));

    let facets: Vec<Vector3> = run
        .iter()
        .map(surface_separate_rim_picks_tests::facet_normal_under)
        .collect();
    let added = surface_separate_rim_picks_tests::the_bands_own_vertices(&part, &built, &run);

    let worst = added
        .iter()
        .map(|&vertex| {
            facets
                .iter()
                .map(|&normal| {
                    (surface_separate_rim_picks_tests::distance_to_the_balls_centre_line(
                        vertex, normal,
                    ) - 0.5)
                        .abs()
                })
                .fold(f64::INFINITY, f64::min)
        })
        .fold(f64::NEG_INFINITY, f64::max);

    assert_eq!(
        added.len(),
        216,
        "the routed band's own vertices, a triangulation pin"
    );

    assert!(
        (worst - 1.8303e-5).abs() <= 1e-7,
        "the band's worst vertex was 1.0587e-3 off the canal and is now 1.83e-5: the interior \
         samples are ON the canal and what is left is the two BOUNDARY rows - it reads {worst:e}"
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

/// Banker's rounding at a decimal place, so a value exactly halfway rounds to the even digit.
fn round_to(value: f64, digits: i32) -> f64 {
    let scale = 10f64.powi(digits);

    (value * scale).round_ties_even() / scale
}
