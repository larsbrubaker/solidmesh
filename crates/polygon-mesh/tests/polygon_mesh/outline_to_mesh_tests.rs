//! The kernel's outline entry points on contours in mm: a contour is snapped onto the one-micron
//! grid point for point (half a micron rounding to the even neighbour) with empty contours dropped,
//! and a square with a hole extrudes and revolves to closed, outward solids of the right volume.

use clipper2_rust::Point64;
use solidmesh_math::math_helper::TAU;
use solidmesh_math::Vector2;
use solidmesh_mesh::processors::outline_to_mesh::{
    contours_to_polygons, extrude_contours, revolve_contours, triangulate_faces,
};

use super::solid_checks::assert_closed_outward;

fn contour(points: &[(f64, f64)]) -> Vec<Vector2> {
    points.iter().map(|&(x, y)| Vector2::new(x, y)).collect()
}

/// A 10 mm square at (2, 0) with a 4 mm square hole in its middle, the hole wound clockwise.
fn square_with_hole() -> Vec<Vec<Vector2>> {
    vec![
        contour(&[(2.0, 0.0), (12.0, 0.0), (12.0, 10.0), (2.0, 10.0)]),
        contour(&[(5.0, 3.0), (5.0, 7.0), (9.0, 7.0), (9.0, 3.0)]),
    ]
}

#[test]
fn contours_snap_to_the_micron_grid_half_to_even() {
    let polygons = contours_to_polygons(&[
        contour(&[(0.0005, -0.0015), (1.2344999, 2.0025)]),
        Vec::new(),
        contour(&[(-0.0025, 0.0035)]),
    ]);
    assert_eq!(
        polygons,
        vec![
            vec![Point64::new(0, -2), Point64::new(1234, 2002)],
            vec![Point64::new(-2, 4)],
        ]
    );
}

#[test]
fn a_square_with_a_hole_extrudes_to_a_closed_solid_of_its_volume() {
    let mesh = extrude_contours(&square_with_hole(), 3.0).expect("the outline extrudes");
    assert_closed_outward(&mesh, "extrude");
    let volume = mesh.get_volume();
    assert!((volume - 84.0 * 3.0).abs() < 1e-6, "volume {volume}");
}

#[test]
fn a_square_with_a_hole_revolves_to_a_closed_solid() {
    let mesh =
        revolve_contours(&square_with_hole(), 32, 0.0, TAU, false).expect("the outline revolves");
    assert_closed_outward(&mesh, "revolve");
}

#[test]
fn a_square_with_a_hole_fills_its_area() {
    let mesh = triangulate_faces(&square_with_hole(), None, None, 2.0);
    let area: f64 = mesh
        .faces
        .iter()
        .map(|face| {
            let [a, b, c] = [face.v0, face.v1, face.v2].map(|v| mesh.vertices[v as usize]);
            let (ab, ac) = (b - a, c - a);
            f64::from(ab.x * ac.y - ab.y * ac.x) / 2.0
        })
        .sum();
    assert!((area - 84.0).abs() < 1e-4, "area {area}");
    assert!(mesh.vertices.iter().all(|v| v.z == 2.0));
}
