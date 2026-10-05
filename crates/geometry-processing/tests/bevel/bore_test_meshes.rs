//! The plate with a round bore and the split of one of its edges. Ports the two pieces of
//! `Tests/GeometryProcessingTests/Fixtures/BoreTestMeshes.cs` (`CreatePlateWithARoundBore`) and
//! `BoreTestMeshesSplitChords.cs` (`SplitTheEdgeBetween`) that `section_room_split_crease_tests.rs`
//! reads; the fixture's other parts land with the classes that read them.

use agg_path::vertex_storage::VertexStorage;
use polygon_mesh::processors::vertex_source_to_mesh::revolve;
use polygon_mesh::{FaceList, Mesh};
use vector_math::{Vector3, Vector3Float};

/// C# `CreatePlateWithARoundBore`: a plate `thickness` thick, centred on z = 0, with a bore of
/// `radius` drawn with `sides` facets and `land` of plate outside it.
pub fn create_plate_with_a_round_bore(sides: i32, radius: f64, thickness: f64, land: f64) -> Mesh {
    let mut path = VertexStorage::new();
    path.move_to(radius, -thickness / 2.0);
    path.line_to(radius + land, -thickness / 2.0);
    path.line_to(radius + land, thickness / 2.0);
    path.line_to(radius, thickness / 2.0);

    // The closing segment back to (radius, -thickness / 2) is the BORE wall, which is why the
    // profile is left open here.
    revolve(&path, sides, 0.0, std::f64::consts::TAU, true).expect("the plate revolves")
}

/// C# `SplitTheEdgeBetween`: splits the mesh edge between the vertices nearest `from` and `to` at
/// `at` of the way along it, each face on it into two keeping its winding.
pub fn split_the_edge_between(mesh: &mut Mesh, from: Vector3, to: Vector3, at: f64) {
    let nearest = |mesh: &Mesh, point: Vector3| {
        let mut best = -1;
        let mut least = f64::MAX;
        for (i, &v) in mesh.vertices.iter().enumerate() {
            let away = (Vector3::from_vector3_float(v) - point).length_squared();
            if away < least {
                least = away;
                best = i as i32;
            }
        }
        best
    };

    let first = nearest(mesh, from);
    let second = nearest(mesh, to);
    let middle = mesh.vertices.len() as i32;

    let a = Vector3::from_vector3_float(mesh.vertices[first as usize]);
    let b = Vector3::from_vector3_float(mesh.vertices[second as usize]);
    mesh.vertices
        .push(Vector3Float::from_vector3(a + ((b - a) * at)));

    let mut replaced = FaceList::new();
    for face in mesh.faces.iter() {
        let corners = [face.v0, face.v1, face.v2];
        let carries = (0..3).find(|&k| {
            let next = (k + 1) % 3;
            (corners[k] == first && corners[next] == second)
                || (corners[k] == second && corners[next] == first)
        });

        let Some(carries) = carries else {
            replaced.add(face.v0, face.v1, face.v2, &mesh.vertices);
            continue;
        };

        // The two halves of the face, each keeping its winding.
        let opposite = corners[(carries + 2) % 3];
        replaced.add(corners[carries], middle, opposite, &mesh.vertices);
        replaced.add(middle, corners[(carries + 1) % 3], opposite, &mesh.vertices);
    }

    mesh.faces = replaced;
    mesh.mark_as_changed();
}
