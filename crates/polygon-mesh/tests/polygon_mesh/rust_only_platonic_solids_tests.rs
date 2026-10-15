//! Rust-only checks of `solids::platonic_solids`, the builders every primitive starts from.
//! Every solid must be a closed, outward-wound, genus-zero mesh with the expected counts.

use polygon_mesh::solids::platonic_solids;
use polygon_mesh::Mesh;

fn assert_closed_outward(mesh: &Mesh, vertices: usize, triangles: usize) {
    assert_eq!(mesh.vertices.len(), vertices);
    assert_eq!(mesh.faces.len(), triangles);
    assert!(mesh.is_manifold());
    // Euler's formula for a closed genus zero triangle mesh: E = 3F / 2, V - E + F = 2.
    assert_eq!(vertices + triangles, 2 + triangles * 3 / 2);
    assert!(mesh.get_volume() > 0.0, "faces must wind outward");
}

#[test]
fn rust_only_dodecahedron_is_a_closed_twelve_face_solid() {
    let phi = (1.0 + 5.0_f64.sqrt()) / 2.0;
    let mesh = platonic_solids::create_dodecahedron(1.0);

    // Twelve pentagons, each fanned into three triangles.
    assert_closed_outward(&mesh, 20, 36);

    // The golden-rectangle vertices reach phi on every axis.
    let bounds = mesh.get_axis_aligned_bounding_box();
    for axis in 0..3 {
        assert!((bounds.min_xyz[axis] + phi).abs() < 1e-6);
        assert!((bounds.max_xyz[axis] - phi).abs() < 1e-6);
    }

    // Edge length 2 / phi; a regular dodecahedron holds (15 + 7 sqrt 5) / 4 * a^3.
    let edge = 2.0 / phi;
    let expected = (15.0 + 7.0 * 5.0_f64.sqrt()) / 4.0 * edge.powi(3);
    assert!((mesh.get_volume() - expected).abs() < 1e-4);

    let scaled = platonic_solids::create_dodecahedron(2.0);
    assert!((scaled.get_volume() - expected * 8.0).abs() < 1e-3);
}

#[test]
fn rust_only_other_platonic_solids_are_closed_and_outward() {
    assert_closed_outward(&platonic_solids::create_tetrahedron(1.0), 4, 4);
    assert_closed_outward(&platonic_solids::create_cube(1.0, 1.0, 1.0), 8, 12);
    assert_closed_outward(&platonic_solids::create_octahedron(1.0), 6, 8);
    assert_closed_outward(&platonic_solids::create_icosahedron(1.0), 12, 20);
}
