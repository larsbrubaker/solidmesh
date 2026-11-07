//! Rust-only checks of the second bevel leaf cluster (`BevelProfilePolygon`,
//! `BevelMixedCornerEnds`, `BevelPairCornerCrease`, `BevelPartnerTube`, `BevelRoundDescriptor`).
//! The whole-build tests also cover them; these pin the leaf behaviour alone.

use std::f64::consts::{FRAC_PI_2, PI};

use geometry_processing::bevel::bevel_pair_corner_crease::{
    lies_between_selected_runs, takes_the_partners_ball,
};
use geometry_processing::bevel::bevel_profile_polygon::{signed_area, try_triangulate};
use geometry_processing::bevel::{
    BevelMixedCornerEnds, BevelPartnerTube, BevelRoundDescriptor, PairFaces,
};
use geometry_processing::topology::{EdgeConvexity, FeatureEdgeSet};
use polygon_mesh::solids::platonic_solids;
use vector_math::{Vector2, Vector3};

#[test]
fn rust_only_an_l_shaped_profile_clips_into_four_counter_clockwise_ears() {
    // Reflex at (1, 1): a fan from vertex 0 would still work, but the ear clip must skip the
    // reflex vertex and keep every triangle counter clockwise.
    let polygon = vec![
        Vector2::new(0.0, 0.0),
        Vector2::new(2.0, 0.0),
        Vector2::new(2.0, 1.0),
        Vector2::new(1.0, 1.0),
        Vector2::new(1.0, 2.0),
        Vector2::new(0.0, 2.0),
    ];
    assert_eq!(signed_area(&polygon), 3.0);

    let triangles = try_triangulate(&polygon).expect("an L is simple");
    assert_eq!(triangles.len(), 12);

    let mut area = 0.0;
    for t in triangles.chunks(3) {
        let tri: Vec<Vector2> = t.iter().map(|&i| polygon[i as usize]).collect();
        let a = signed_area(&tri);
        assert!(a > 0.0, "every ear is counter clockwise");
        area += a;
    }
    assert_eq!(area, 3.0);

    // Wound clockwise there is no ear at all.
    let reversed: Vec<Vector2> = polygon.iter().rev().copied().collect();
    assert_eq!(try_triangulate(&reversed), None);
    assert_eq!(try_triangulate(&polygon[..2]), None);
}

#[test]
fn rust_only_corner_ends_collect_per_vertex_in_first_seen_order() {
    let cube = platonic_solids::create_cube(20.0, 20.0, 20.0);
    let features = FeatureEdgeSet::create(&cube);
    let edges = features.edges();
    let a = edges[0];
    // An edge sharing a's start vertex.
    let b = *edges
        .iter()
        .find(|e| {
            e.mesh_edge_index != a.mesh_edge_index
                && (e.start_vertex == a.start_vertex || e.end_vertex == a.start_vertex)
        })
        .expect("a cube corner has three creases");

    let mut corner = BevelMixedCornerEnds::new(a.start_vertex, a.start);
    corner.add(&a, 0);
    corner.add(&b, 1);
    corner.add(&a, 1);
    assert_eq!(corner.edges().len(), 2, "a picked twice is one edge here");
    assert_eq!(corner.groups(), &[0, 1]);
    assert!(
        corner.shares_a_group(&a, &b),
        "a was also picked into b's group"
    );

    let mut apart = BevelMixedCornerEnds::new(a.start_vertex, a.start);
    apart.add(&a, 0);
    apart.add(&b, 1);
    assert!(!apart.shares_a_group(&a, &b));
}

#[test]
fn rust_only_a_cube_crease_is_between_runs_on_its_own_two_faces() {
    let cube = platonic_solids::create_cube(20.0, 20.0, 20.0);
    let features = FeatureEdgeSet::create(&cube);
    let edge = features.edges()[0];

    assert!(lies_between_selected_runs(
        &edge,
        edge.normal0,
        edge.normal1,
        &[],
        PairFaces::default(),
        None
    ));
    // A run tangent to neither face: the normals refuse it, and with no labelling that is final.
    let other = -edge.normal0;
    assert!(!lies_between_selected_runs(
        &edge,
        other,
        other,
        &[],
        PairFaces::default(),
        None
    ));
    // Labelled as two surfaces, with the run on both faces' surfaces by id, it is recognised.
    let mut labels = vec![0; cube.faces.len()];
    labels[edge.face1 as usize] = 1;
    let mut ask = || Some(labels.as_slice());
    let faces = PairFaces {
        own_face0: edge.face0,
        own_face1: edge.face1,
        neighbour_faces: None,
    };
    assert!(lies_between_selected_runs(
        &edge,
        other,
        other,
        &[],
        faces,
        Some(&mut ask)
    ));
    assert_eq!(edge.convexity, EdgeConvexity::Convex);
    assert!(!takes_the_partners_ball(&edge));
}

#[test]
fn rust_only_a_partner_tube_is_finite_and_its_chords_sit_inside_the_cylinder() {
    let smooth = BevelPartnerTube::spanning(Vector3::ZERO, Vector3::UNIT_Z * 2.0, 5.0, 0.0, 10.0);
    assert_eq!(smooth.axis_direction, Vector3::UNIT_Z);
    assert_eq!(smooth.depth(Vector3::new(5.0, 0.0, 5.0)), 0.0);
    assert_eq!(smooth.depth(Vector3::new(1.0, 0.0, 5.0)), -4.0);
    assert_eq!(smooth.depth(Vector3::new(1.0, 0.0, 11.0)), f64::INFINITY);

    // A quarter-turn arc between faces whose outward normals are +x and +y, in two chords.
    let chorded = BevelPartnerTube::around_the_chorded_arc(
        Vector3::ZERO,
        Vector3::UNIT_Z,
        5.0,
        Vector3::UNIT_X,
        Vector3::UNIT_Y,
        2,
        f64::NEG_INFINITY,
        f64::INFINITY,
    );
    assert!(chorded.is_chorded());
    // Mid-chord on the arc (between the tangent points at -x and -y), the cylinder stands outside
    // the facet plane by the sagitta.
    let mid = Vector3::new(-(PI / 8.0).cos(), -(PI / 8.0).sin(), 0.0) * 5.0;
    let sagitta = 5.0 * (1.0 - (PI / 8.0).cos());
    assert!((chorded.depth(mid) - sagitta).abs() < 1e-12);
    assert_eq!(smooth.depth(mid * 0.5 + Vector3::new(0.0, 0.0, 5.0)), -2.5);
    let at = Vector3::new(-5.0, 0.0, 0.0);
    let crossing = chorded
        .try_cross_plane(
            chorded.angle_of(at),
            Vector3::new(0.0, 0.0, 3.0),
            Vector3::UNIT_Z,
        )
        .expect("the axis crosses a square plane");
    assert!((crossing.z - 3.0).abs() < 1e-12);
    assert!(chorded
        .try_cross_plane(0.0, Vector3::ZERO, Vector3::UNIT_X)
        .is_none());
}

#[test]
fn rust_only_a_round_contains_its_surface_and_merges_axially() {
    let round = BevelRoundDescriptor::new(
        Vector3::ZERO,
        Vector3::UNIT_Z,
        5.0,
        Vector3::UNIT_X,
        Vector3::UNIT_Y,
        0.0,
        10.0,
        8,
    );
    assert!(round.contains(Vector3::new(0.0, 5.0, 5.0), 1e-6, 1e-6));
    assert!(round.contains(
        Vector3::new(
            5.0 * (FRAC_PI_2 / 2.0).cos(),
            5.0 * (FRAC_PI_2 / 2.0).sin(),
            1.0
        ),
        1e-6,
        1e-6
    ));
    assert!(
        !round.contains(Vector3::new(-5.0, 0.0, 5.0), 1e-6, 1e-6),
        "off the span"
    );
    assert!(
        !round.contains(Vector3::new(5.0, 0.0, 11.0), 1e-6, 1e-6),
        "past the end"
    );

    let further = BevelRoundDescriptor::new(
        Vector3::new(0.0, 0.0, 10.0),
        Vector3::UNIT_Z,
        5.0,
        Vector3::UNIT_X,
        Vector3::UNIT_Y,
        0.0,
        4.0,
        8,
    );
    assert!(round.is_the_same_round_as(&further, 1e-9));
    let merged = round.merged(&further);
    assert_eq!((merged.axial_from, merged.axial_to), (0.0, 14.0));
}

#[test]
fn rust_only_a_ball_between_a_round_and_a_square_slope_slides_along_the_axis() {
    // A boss of radius 5 about z beside the plane x = 6 facing -x, square to the axis, so the
    // closed form answers: a ball of radius 1 keeps its centre 6 from the axis and 1 off the plane,
    // at x = 5 and y = sqrt(11) on the anchor's side, in the plane through the anchor.
    let round = BevelRoundDescriptor::with_span(
        Vector3::ZERO,
        Vector3::UNIT_Z,
        5.0,
        Vector3::UNIT_X,
        -PI / 2.0,
        PI / 2.0,
        0.0,
        10.0,
        16,
    );
    let station = round
        .try_find_station(
            Vector3::new(5.5, 1.0, 3.0),
            Vector3::new(6.0, 0.0, 0.0),
            Vector3::new(-1.0, 0.0, 0.0),
            1.0,
        )
        .expect("a ball of radius 1 rolls between the boss and the wall");
    assert_eq!(station.tangent, Vector3::UNIT_Z);
    assert!((station.centre.z - 3.0).abs() < 1e-12);
    assert!(
        (station.centre.x - 5.0).abs() < 1e-12,
        "one radius off the slope"
    );
    assert!(
        (station.centre.y - 11f64.sqrt()).abs() < 1e-12,
        "on the anchor's side"
    );
    assert!(((station.contact - Vector3::new(0.0, 0.0, 3.0)).length() - 5.0).abs() < 1e-12);

    // Too wide a gap for the ball: no station.
    assert!(round
        .try_find_station(
            Vector3::new(5.5, 1.0, 3.0),
            Vector3::new(9.0, 0.0, 0.0),
            Vector3::new(-1.0, 0.0, 0.0),
            1.0,
        )
        .is_none());
}
