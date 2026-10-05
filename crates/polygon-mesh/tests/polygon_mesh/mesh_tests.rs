//! Port of `Agg.PolygonMesh/MeshTests.cs`: STL parsing, face cut lines, face splitting and the
//! face BSP tree. The rest (SDF, slicing/stitching on clipper polygons, dual contouring) wait for
//! their Phase 2 slices and are listed in `docs/PORTING_PLAN.md`.

use polygon_mesh::processors::{face_bsp_tree, stl_processing};
use polygon_mesh::solids::platonic_solids;
use polygon_mesh::{CancelToken, Face, Mesh};
use vector_math::math_helper::TAU;
use vector_math::{Matrix4X4, Plane, Vector2, Vector3, Vector3Float};

#[test]
fn ascii_stl_open_scad_format_parses_to_faces() {
    // Tetrahedron in the exact textual style produced by OpenSCAD: CRLF endings, two-space
    // facet indent, bare integer vertex coordinates.
    let lines = [
        "solid OpenSCAD_Model",
        "  facet normal 0 0 -1",
        "    outer loop",
        "      vertex 0 0 0",
        "      vertex 0 10 0",
        "      vertex 10 0 0",
        "    endloop",
        "  endfacet",
        "  facet normal -1 0 0",
        "    outer loop",
        "      vertex 0 0 0",
        "      vertex 0 0 10",
        "      vertex 0 10 0",
        "    endloop",
        "  endfacet",
        "  facet normal 0 -1 0",
        "    outer loop",
        "      vertex 0 0 0",
        "      vertex 10 0 0",
        "      vertex 0 0 10",
        "    endloop",
        "  endfacet",
        "  facet normal 1 1 1",
        "    outer loop",
        "      vertex 10 0 0",
        "      vertex 0 10 0",
        "      vertex 0 0 10",
        "    endloop",
        "  endfacet",
        "endsolid OpenSCAD_Model",
    ];
    let ascii_stl = lines.join("\r\n") + "\r\n";

    let mesh = stl_processing::load(ascii_stl.as_bytes(), &CancelToken::new(), None)
        .expect("the OpenSCAD STL parses");
    assert_eq!(mesh.faces.len(), 4);
}

#[test]
fn face_cut_wound_correctly() {
    let vertices = vec![
        Vector3Float::from_vector2(Vector2::new(1.0, 0.0).get_rotated(TAU / 3.0 * 0.0), 0.0),
        Vector3Float::from_vector2(Vector2::new(1.0, 0.0).get_rotated(TAU / 3.0 * 1.0), 0.0),
        Vector3Float::from_vector2(Vector2::new(1.0, 0.0).get_rotated(TAU / 3.0 * 2.0), 0.0),
    ];

    let face = Face::with_normal(0, 1, 2, Vector3Float::new(0.0, 0.0, 1.0));

    let check_angle = |angle: f64, distance: f64| {
        let normal = Vector3::from_vector2(Vector2::new(1.0, 0.0).get_rotated(angle), 0.0);
        // C# ignores GetCutLine's return here and reads the zeroed outs on a miss
        let (start, end) = face
            .get_cut_line(&vertices, Plane::new(normal, distance))
            .unwrap_or((Vector3::ZERO, Vector3::ZERO));

        let direction = end - start;
        let y_direction = Vector2::new(direction.x, direction.y).get_rotated(-angle);
        assert!(y_direction.y < 0.0);
    };

    check_angle(TAU / 3.0 * 0.0, 0.5);
    check_angle(TAU / 3.0 * 1.0, 0.5);
    check_angle(TAU / 3.0 * 2.0, 0.5);
}

#[test]
fn cuts_respect_winding_order() {
    let mut cube = platonic_solids::create_cube(10.0, 10.0, 10.0);
    cube.translate_xyz(0.0, 0.0, 5.0); // move bottom to z=0
    let cut_plane = Plane::from_normal_and_point(Vector3::UNIT_Z, Vector3::new(0.0, 0.0, 5.0));

    for face_index in 0..cube.faces.len() {
        let face = cube.faces[face_index];
        if face.normal.z == 0.0 {
            let cut = face.get_cut_line(&cube.vertices, cut_plane);
            assert!(cut.is_some());
            let (start, end) = cut.unwrap_or_default();
            if face.normal.x < 0.0 {
                assert!(start.y > end.y);
            } else if face.normal.y < 0.0 {
                assert!(start.x < end.x);
            } else if face.normal.x > 0.0 {
                assert!(start.y < end.y);
            } else if face.normal.y > 0.0 {
                assert!(start.x > end.x);
            }
        }
    }
}

/// Every face lies wholly on one side (or on) the x = 5 split line.
fn faces_not_crossing_x5(mesh: &Mesh) -> usize {
    let x = |v: i32| mesh.vertices[v as usize].x;
    mesh.faces
        .iter()
        .filter(|f| {
            (x(f.v0) <= 5.0 && x(f.v1) <= 5.0 && x(f.v2) <= 5.0)
                || (x(f.v0) >= 5.0 && x(f.v1) >= 5.0 && x(f.v2) >= 5.0)
        })
        .count()
}

fn count_x(mesh: &Mesh, value: f32) -> usize {
    mesh.vertices.iter().filter(|v| v.x == value).count()
}

#[test]
fn split_face_edge_edge() {
    let test_positions = |p0: usize, p1: usize, p2: usize| {
        let positions = [
            Vector3::ZERO,
            Vector3::new(10.0, 0.0, 0.0),
            Vector3::new(5.0, 20.0, 0.0),
        ];
        let mut mesh = Mesh::new();
        mesh.create_face(&[positions[p0], positions[p1], positions[p2]]);
        assert_eq!(mesh.faces.len(), 1);
        assert_eq!(mesh.vertices.len(), 3);

        // we find a split
        assert!(mesh.split_face(0, Plane::new(Vector3::new(1.0, 0.0, 0.0), 5.0), 0.001));
        // we now have 2 faces
        assert_eq!(mesh.faces.len(), 2);
        // the have all the expected x values
        assert_eq!(mesh.vertices.len(), 4);
        assert_eq!(count_x(&mesh, 0.0), 1);
        assert_eq!(count_x(&mesh, 5.0), 2);
        assert_eq!(count_x(&mesh, 10.0), 1);
        // no face crosses the split line
        assert_eq!(faces_not_crossing_x5(&mesh), 2);
    };

    // test every vertex orientation
    test_positions(0, 1, 2);
    test_positions(0, 2, 1);
    test_positions(1, 0, 2);
    test_positions(1, 2, 0);
    test_positions(2, 0, 1);
    test_positions(2, 1, 0);
}

#[test]
fn split_face_two_edges() {
    let test_positions = |p0: usize, p1: usize, p2: usize| {
        let positions = [
            Vector3::new(0.0, 5.0, 0.0),
            Vector3::ZERO,
            Vector3::new(10.0, 0.0, 0.0),
        ];
        let mut mesh = Mesh::new();
        mesh.create_face(&[positions[p0], positions[p1], positions[p2]]);
        assert_eq!(mesh.faces.len(), 1);
        assert_eq!(mesh.vertices.len(), 3);

        // we find a split
        assert!(mesh.split_face(0, Plane::new(Vector3::new(1.0, 0.0, 0.0), 5.0), 0.001));
        // we now have 3 faces
        assert_eq!(mesh.faces.len(), 3);
        // we now have 5 verts with all the expected x values
        assert_eq!(mesh.vertices.len(), 5);
        assert_eq!(count_x(&mesh, 0.0), 2);
        assert_eq!(count_x(&mesh, 5.0), 2);
        assert_eq!(count_x(&mesh, 10.0), 1);
        // no face crosses the split line
        assert_eq!(faces_not_crossing_x5(&mesh), 3);
    };

    test_positions(0, 1, 2);
    test_positions(0, 2, 1);
    test_positions(1, 0, 2);
    test_positions(1, 2, 0);
    test_positions(2, 0, 1);
    test_positions(2, 1, 0);
}

#[test]
fn create_bsp_face_trees() {
    // a simple list of 3 faces
    //
    // Index 1 ^------------------- z = 3
    //
    // Index 0 ^------------------- z = 2
    //
    // Index 2 ^------------------- z = 1
    let mut test_mesh = Mesh::new();
    test_mesh.create_face(&[
        Vector3::new(0.0, 0.0, 2.0),
        Vector3::new(10.0, 0.0, 2.0),
        Vector3::new(5.0, 5.0, 2.0),
    ]);
    test_mesh.create_face(&[
        Vector3::new(0.0, 0.0, 3.0),
        Vector3::new(10.0, 0.0, 3.0),
        Vector3::new(5.0, 5.0, 3.0),
    ]);
    test_mesh.create_face(&[
        Vector3::new(0.0, 0.0, 1.0),
        Vector3::new(10.0, 0.0, 1.0),
        Vector3::new(5.0, 5.0, 1.0),
    ]);

    // test they are in the right order
    let root = face_bsp_tree::create_default(&test_mesh);

    assert_eq!(root.index, 1);
    let back = root.back_node.as_deref().expect("back node");
    assert_eq!(back.index, 0);
    assert_eq!(back.back_node.as_deref().expect("back back node").index, 2);

    let render_order_list = face_bsp_tree::get_faces_in_visibilty_order(
        &test_mesh,
        &root,
        Matrix4X4::IDENTITY,
        Matrix4X4::IDENTITY,
    );
    assert_eq!(render_order_list[0], 1);
    assert_eq!(render_order_list[1], 0);
    assert_eq!(render_order_list[2], 2);
}
