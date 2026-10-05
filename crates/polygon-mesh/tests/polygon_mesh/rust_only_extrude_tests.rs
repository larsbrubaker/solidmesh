//! Rust-only checks of `VertexSourceToMesh.Extrude` (agg-sharp has no Extrude test class): the
//! square and bevelled paths make closed, outward-wound solids of the expected size, including
//! the bevel bands tess2 triangulates where C# uses TriangleNet (docs/CSHARP_DIVERGENCES.md 290).

use agg_path::clipper_library::JoinType;
use agg_path::VertexStorage;
use polygon_mesh::processors::vertex_source_to_mesh::{extrude, extrude_bevelled};

fn square(side: f64) -> VertexStorage {
    let mut path = VertexStorage::new();
    path.move_to(0.0, 0.0);
    path.line_to(side, 0.0);
    path.line_to(side, side);
    path.line_to(0.0, side);
    path.close_polygon();
    path
}

#[test]
fn rust_only_a_square_extrudes_to_a_closed_box() {
    let mesh = extrude(&square(10.0), 5.0);

    assert!(mesh.is_manifold());
    assert_eq!(mesh.vertices.len(), 8);
    assert_eq!(mesh.faces.len(), 12);
    assert!(
        (mesh.get_volume() - 500.0).abs() < 1e-6,
        "{}",
        mesh.get_volume()
    );
    let bounds = mesh.get_axis_aligned_bounding_box();
    assert!((bounds.z_size() - 5.0).abs() < 1e-9);
}

#[test]
fn rust_only_a_top_and_bottom_bevelled_square_is_a_closed_outward_solid() {
    let top = [(8.0, -0.5), (10.0, -1.0)];
    let bottom = [(2.0, -0.5), (0.0, -1.0)];
    let mesh = extrude_bevelled(
        &square(10.0),
        10.0,
        Some(&top),
        JoinType::Round,
        Some(&bottom),
        JoinType::Round,
    );

    assert!(
        mesh.is_manifold(),
        "the bevel bands weld to the walls and caps"
    );
    let volume = mesh.get_volume();
    // less than the square prism, more than the prism of the fully inset square
    assert!(volume < 1000.0 && volume > 800.0, "volume {volume}");
    let bounds = mesh.get_axis_aligned_bounding_box();
    assert!((bounds.z_size() - 10.0).abs() < 1e-6);
    assert!((bounds.x_size() - 10.0).abs() < 1e-3);
}
