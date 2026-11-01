//! A side's closest point and what it lies on (`RollingBallSide::closest`): inside a face, on an
//! edge two of the side's triangles share, on the side's own boundary (a rim), at a corner; and the
//! box tree's answer against every triangle read one by one.

use geometry_processing::bevel::level_line::ChainField;
use geometry_processing::bevel::rolling_ball::{ContactFeature, HalfSpace, RollingBallSide};
use vector_math::seeded_random::SeededRandom;
use vector_math::Vector3;

use crate::surfaces::{rectangle, sphere};

/// A 10 x 10 square at z = 0 made of two triangles, split along its diagonal.
fn square() -> RollingBallSide {
    let mut positions = Vec::new();
    let triangles = rectangle(
        &mut positions,
        Vector3::new(0.0, 0.0, 0.0),
        Vector3::new(10.0, 0.0, 0.0),
        Vector3::new(0.0, 10.0, 0.0),
        1,
        1,
    );
    RollingBallSide::new(&positions, &triangles)
}

#[test]
fn a_point_above_a_face_touches_the_face() {
    let contact = square().closest(Vector3::new(7.0, 2.0, 3.0)).unwrap();
    assert_eq!(contact.point, Vector3::new(7.0, 2.0, 0.0));
    assert_eq!(contact.distance, 3.0);
    assert!(matches!(contact.feature, ContactFeature::Face { .. }));
    assert!(!contact.feature.is_rim());
}

#[test]
fn a_point_above_the_shared_diagonal_touches_an_edge_that_is_no_rim() {
    let contact = square().closest(Vector3::new(4.0, 4.0, 2.0)).unwrap();
    assert!((contact.point - Vector3::new(4.0, 4.0, 0.0)).length() < 1e-12);
    match contact.feature {
        ContactFeature::Edge { rim, .. } => assert!(!rim, "two triangles share the diagonal"),
        other => panic!("expected the diagonal, got {other:?}"),
    }
}

#[test]
fn a_point_past_the_side_touches_its_rim() {
    let contact = square().closest(Vector3::new(13.0, 5.0, 4.0)).unwrap();
    assert_eq!(contact.point, Vector3::new(10.0, 5.0, 0.0));
    assert_eq!(contact.distance, 5.0);
    assert!(matches!(
        contact.feature,
        ContactFeature::Edge { rim: true, .. }
    ));
}

#[test]
fn a_point_past_a_corner_touches_the_corner_vertex_on_the_rim() {
    let contact = square().closest(Vector3::new(-3.0, -4.0, 0.0)).unwrap();
    assert_eq!(contact.point, Vector3::new(0.0, 0.0, 0.0));
    assert_eq!(contact.distance, 5.0);
    assert!(matches!(
        contact.feature,
        ContactFeature::Vertex { rim: true, .. }
    ));
}

/// The tree prunes, so it is held to the plain search: every triangle read in turn.
#[test]
fn the_box_tree_finds_the_closest_triangle_a_plain_search_finds() {
    let mut positions = Vec::new();
    let triangles = sphere(&mut positions, Vector3::new(1.0, -2.0, 3.0), 7.0, 48, 24);
    let side = RollingBallSide::new(&positions, &triangles);
    assert_eq!(side.triangle_count(), triangles.len());

    let mut random = SeededRandom::new(20261031);
    for _ in 0..500 {
        let p = Vector3::new(
            (random.next_double() - 0.5) * 30.0,
            (random.next_double() - 0.5) * 30.0,
            (random.next_double() - 0.5) * 30.0,
        );
        let plain = triangles
            .iter()
            .map(|t| {
                let on = ChainField::closest_on_triangle(
                    p,
                    positions[t[0] as usize],
                    positions[t[1] as usize],
                    positions[t[2] as usize],
                );
                (on - p).length()
            })
            .fold(f64::INFINITY, f64::min);
        let tree = side.closest(p).unwrap().distance;
        assert_eq!(tree, plain, "at {p:?}");
    }
}

#[test]
fn a_cut_through_a_corner_of_the_triangle_reads_the_part_inside() {
    // One triangle whose first corner lies exactly on the cutting plane and whose next corner is
    // across it: the part inside is the triangle from that corner to where the plane crosses the
    // far edge. The point stands above that part, so its foot on the plane is the answer.
    let positions = [
        Vector3::new(0.0, 0.0, 0.0),
        Vector3::new(1.0, 0.0, 0.0),
        Vector3::new(0.0, 1.0, 0.0),
    ];
    let side = RollingBallSide::new(&positions, &[[0, 1, 2]]);
    let half = HalfSpace {
        origin: Vector3::new(0.0, 0.0, 0.0),
        normal: Vector3::new(-1.0, 1.0, 0.0).get_normal(),
    };
    let point = Vector3::new(0.1, 0.5, 1.0);
    for contact in [
        side.closest_in(point, Some(half)).unwrap(),
        side.closest_within(point, &[half]).unwrap(),
        side.closest_tracked(point, &side.closest(point).unwrap(), &[half])
            .unwrap(),
    ] {
        let foot = Vector3::new(0.1, 0.5, 0.0);
        assert!((contact.point - foot).length() <= 1e-12, "{contact:?}");
        assert!((contact.distance - 1.0).abs() <= 1e-12, "{contact:?}");
        assert_eq!(contact.feature, ContactFeature::Face { triangle: 0 });
    }
}

#[test]
fn a_triangle_with_no_area_reads_its_contact_on_its_longest_edge_and_faces_no_way() {
    // Three corners on one line: the triangle is its longest edge, 0 to 2, and a point above the
    // middle of 1 to 2 lies on that edge, not on 0 to 1. With no area it has no front or back, so
    // it cannot say which side of it a point stands on.
    let positions = [
        Vector3::new(0.0, 0.0, 0.0),
        Vector3::new(1.0, 0.0, 0.0),
        Vector3::new(2.0, 0.0, 0.0),
    ];
    let side = RollingBallSide::new(&positions, &[[0, 1, 2]]);
    let point = Vector3::new(1.5, 1.0, 0.0);
    let contact = side.closest(point).unwrap();
    assert_eq!(contact.point, Vector3::new(1.5, 0.0, 0.0), "{contact:?}");
    assert_eq!(
        contact.feature,
        ContactFeature::Edge {
            from: 0,
            to: 2,
            rim: true
        }
    );
    assert_eq!(side.standing_off(&contact, point, false), None);
    assert_eq!(side.standing_off(&contact, point, true), None);
}
