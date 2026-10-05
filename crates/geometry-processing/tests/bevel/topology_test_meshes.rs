//! Fixture builders ported from `GeometryProcessingTests/Fixtures/TopologyTestMeshes.cs`, added
//! one builder at a time as the test classes that use them land. So far:
//! `CreateIsoscelesPrism` (`BevelArcChordingTests`) and `CreateArcPrism` (`BevelSweepBackDepthTests`).

use polygon_mesh::Mesh;
use vector_math::{math_helper, Vector2, Vector3};

/// A triangular prism whose apex at the origin has the given interior angle, the two long sides
/// running `length` along +x, standing `height` tall centred on z = 0. Welded, so the fixture is
/// about acute angles rather than the repair path.
pub fn create_isosceles_prism(apex_angle_degrees: f64, length: f64, height: f64) -> Mesh {
    let spread = length * (math_helper::degrees_to_radians(apex_angle_degrees) / 2.0).tan();

    // clockwise in xy, which is what makes the side face winding below come out facing outward
    let outline = [
        Vector2::new(0.0, 0.0),
        Vector2::new(length, spread),
        Vector2::new(length, -spread),
    ];

    let bottom = -height / 2.0;
    let top = height / 2.0;

    let mut mesh = Mesh::new();
    let at = Vector3::from_vector2;

    mesh.create_face(&[
        at(outline[0], bottom),
        at(outline[1], bottom),
        at(outline[2], bottom),
    ]);

    mesh.create_face(&[
        at(outline[0], top),
        at(outline[2], top),
        at(outline[1], top),
    ]);

    for i in 0..outline.len() {
        let from = outline[i];
        let to = outline[(i + 1) % outline.len()];

        mesh.create_face(&[at(from, bottom), at(from, top), at(to, top), at(to, bottom)]);
    }

    // CreateFace adds fresh vertices every call, so what is built above is triangle soup.
    // Welding it here keeps this fixture about acute angles - an unwelded one would go down the
    // repair path instead.
    mesh.clean_and_merge();

    mesh
}

/// A half disc of `radius` (its arc chorded into `segments`, closed by its diameter on the x axis),
/// extruded `height` along z.
pub fn create_arc_prism(radius: f64, height: f64, segments: i32) -> Mesh {
    let mut path = agg_path::vertex_storage::VertexStorage::new();
    path.move_to(radius, 0.0);
    for i in 1..=segments {
        let angle = std::f64::consts::PI * f64::from(i) / f64::from(segments);
        path.line_to(radius * angle.cos(), radius * angle.sin());
    }

    path.close_polygon();

    polygon_mesh::processors::vertex_source_to_mesh::extrude(&path, height)
        .expect("a half disc extrudes")
}
