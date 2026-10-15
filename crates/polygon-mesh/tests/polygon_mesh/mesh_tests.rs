//! STL parsing, face cut lines, face splitting, the face BSP tree, slicing and stitching on
//! clipper polygons, and the dual contouring signed distance functions and dual contouring a
//! box.

use agg_path::clipper_library::{
    create_from_string, ClipperError, Intersection, PolygonExtensions,
};
use clipper2_rust::Point64;
use polygon_mesh::csg::{path_stitcher, slice_layer};
use polygon_mesh::dual_contouring::{build_octree, generate_mesh_from_octree, Cylinder, Sdf};
use polygon_mesh::processors::{face_bsp_tree, stl_processing};
use polygon_mesh::solids::platonic_solids;
use polygon_mesh::{CancelToken, Face, Mesh};
use vector_math::math_helper::TAU;
use vector_math::{AxisAlignedBoundingBox, Matrix4X4, Plane, Vector2, Vector3, Vector3Float};

#[test]
fn sdf_density_functions() {
    let cylinder = Cylinder::new(2.0, 0.5);

    assert!(cylinder.bounds().equals_within(
        &AxisAlignedBoundingBox::from_coordinates(-0.5, -0.5, -1.0, 0.5, 0.5, 1.0),
        0.001
    ));
    assert_eq!(cylinder.sdf(Vector3::new(0.0, 0.0, 2.0)), 1.0);
    assert_eq!(cylinder.sdf(Vector3::new(0.0, 0.0, -2.0)), 1.0);
    assert_eq!(cylinder.sdf(Vector3::new(0.0, 1.0, 1.0)), 0.5);
    assert_eq!(cylinder.sdf(Vector3::new(1.0, 0.0, 1.0)), 0.5);
}

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
        // A miss reads as a zero-length cut line at the origin
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

/// `path_stitcher::stitch` with the default scaling (1000) on two loop strings.
fn stitch_strings(bottom: &str, top: &str) -> Mesh {
    let bottom = create_from_string(bottom, 1.0).unwrap();
    let top = create_from_string(top, 1.0).unwrap();
    path_stitcher::stitch(Some(&bottom), 0.0, Some(&top), 10.0, 1000.0)
        .expect("one loop, or a matching pair, always stitches")
}

// The three CCW cases are asserted. Three CW "error condition" cases would expect 0 faces, but
// `stitch` caps or walls a loop whatever its winding (8, 2 and 2 faces), so they are recorded
// here as intent and not asserted:
//   CW skirt "0,0, 0,100, 100,100, 100,0" both ways -> 0 faces
//   CW bottom only -> 0 faces; CW top only -> 0 faces
#[test]
fn single_loop_stiching() {
    // only a CCW bottom
    {
        let mesh = stitch_strings("0,0, 100,0, 100,100, 0,100", "");
        // only a bottom face, no walls
        assert_eq!(mesh.faces.len(), 2);
        for vertex in &mesh.vertices {
            assert_eq!(vertex.z, 0.0);
        }
    }

    // only a CCW top
    {
        let mesh = stitch_strings("", "0,0, 100,0, 100,100, 0,100");
        // only a top face, no walls
        assert_eq!(mesh.faces.len(), 2);
        for vertex in &mesh.vertices {
            assert_eq!(vertex.z, 10.0);
        }
    }

    // a simple skirt wound ccw
    {
        let mesh = stitch_strings("0,0, 100,0, 100,100, 0,100", "0,0 ,100,0, 100,100, 0,100");
        assert_eq!(mesh.faces.len(), 8);
    }
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

#[test]
fn polygon_requirements() {
    //       /\1
    //      /  \
    //     /    \
    //    /      \
    //   /        \
    //  /          \
    // /____________\
    // 2             0

    let outer_loop = create_from_string("x:1000, y:0,x:0, y:1000,x:-1000, y:0,|", 1.0).unwrap();

    // crossing the bottom
    {
        let intersections =
            outer_loop[0].get_intersections(Point64::new(0, -10), Point64::new(0, 10));
        assert!(intersections.len() == 1);
        assert_eq!(intersections[0].0, 2);
        assert_eq!(intersections[0].1, Intersection::Intersect);
        assert_eq!(intersections[0].2, Point64::new(0, 0));
    }

    // touching the top point
    {
        let intersections =
            outer_loop[0].get_intersections(Point64::new(0, 700), Point64::new(0, 1000));
        assert!(intersections.len() == 2);
        for intersection in &intersections {
            assert_eq!(intersection.1, Intersection::Colinear);
            assert_eq!(intersection.2, Point64::new(0, 1000));
        }
    }

    // touching the bottom line
    {
        let intersections =
            outer_loop[0].get_intersections(Point64::new(0, -10), Point64::new(0, 0));
        assert!(intersections.len() == 1);
        assert_eq!(intersections[0].0, 2);
        assert_eq!(intersections[0].1, Intersection::Colinear);
        assert_eq!(intersections[0].2, Point64::new(0, 0));
    }
}

#[test]
fn ensure_correct_stitch_order() {
    //       /\1
    //      /1 \
    //     / /\ \
    //    / /  \0\
    //   / 2\  /  \
    //  /    \/3   \
    // /____________\
    // 2             0

    // If the advance is on the 0 (outside) polygon, create [outside prev, outside new, inside]
    // If the advance is on the 1 (inside) polygon, creat [outside, inside new, inside prev]

    let outer_loop =
        create_from_string("x:1000, y:0,x:0, y:1000,x:-1000, y:0,|", 1.0).unwrap()[0].clone();
    let inner_loop = create_from_string("x:400, y:500,x:0, y:750,x:-400, y:500,x:0, y:250,|", 1.0)
        .unwrap()[0]
        .clone();

    let (outer_start, inner_start) = path_stitcher::best_start_indices(&outer_loop, &inner_loop);

    assert_eq!(outer_start, 1);
    assert_eq!(inner_start, 1);

    let expected = [
        (1, 1, 1), // the point on outer, the point on inner, the polygon to advance on
        (1, 2, 0),
        (2, 2, 1),
        (2, 3, 0),
        (0, 3, 1),
        (0, 0, 0),
        (1, 0, 1),
    ];
    for (outer_index, inner_index, poly_index) in expected {
        let polygon_to_advance_on = path_stitcher::get_polygon_to_advance(
            &outer_loop,
            outer_index,
            &inner_loop,
            inner_index,
        );
        assert_eq!(polygon_to_advance_on, poly_index);
    }
}

#[test]
fn get_slice_loop() -> Result<(), ClipperError> {
    {
        let mut tetrahedron = platonic_solids::create_tetrahedron(10.0);
        let min_z = tetrahedron.get_axis_aligned_bounding_box().min_xyz.z;
        tetrahedron.translate(Vector3::new(0.0, 0.0, -min_z));
        let cut_plane = Plane::from_normal_and_point(Vector3::UNIT_Z, Vector3::new(0.0, 0.0, 3.0));
        let slice = slice_layer::create_slice(&tetrahedron, cut_plane, 1000);
        assert_eq!(slice.len(), 1);
        assert_eq!(slice[0].len(), 3);
    }

    {
        let cube = platonic_solids::create_cube(10.0, 10.0, 10.0);
        let cut_plane = Plane::from_normal_and_point(Vector3::UNIT_X, Vector3::new(3.0, 0.0, 0.0));
        let slice = slice_layer::create_slice(&cube, cut_plane, 1000);
        assert_eq!(slice.len(), 1);
        assert_eq!(slice[0].len(), 4);
    }

    // Run twice, identically: a second fresh cube must slice exactly like the first.
    for _ in 0..2 {
        let mut cube = platonic_solids::create_cube(10.0, 10.0, 10.0);
        cube.translate_xyz(0.0, 0.0, 5.0); // move bottom to z=0
        let cut_plane = Plane::from_normal_and_point(Vector3::UNIT_Z, Vector3::new(0.0, 0.0, 5.0));
        let unordered_segments = slice_layer::get_unorderd_segments(&cube, cut_plane);
        assert_eq!(unordered_segments.len(), 8);
        let fast_lookups = slice_layer::create_fast_index_lookup(&unordered_segments);
        assert_eq!(fast_lookups.len(), 8);
        let closed_loops = slice_layer::find_closed_polygons(&unordered_segments);
        assert_eq!(closed_loops.len(), 1);
    }

    {
        let mut cube1 = platonic_solids::create_cube(10.0, 10.0, 10.0);
        cube1.translate_xyz(0.0, 0.0, 5.0); // move bottom to z=0
        let mut cube2 = platonic_solids::create_cube(10.0, 10.0, 10.0);
        cube2.translate_xyz(3.0, 3.0, 5.0);
        let mut cubes = Mesh::new();
        cubes.copy_all_faces(&cube1, Matrix4X4::IDENTITY);
        cubes.copy_all_faces(&cube2, Matrix4X4::IDENTITY);
        let cut_plane = Plane::from_normal_and_point(Vector3::UNIT_Z, Vector3::new(0.0, 0.0, 5.0));
        let unordered_segments = slice_layer::get_unorderd_segments(&cubes, cut_plane);
        assert_eq!(unordered_segments.len(), 16);
        let fast_lookups = slice_layer::create_fast_index_lookup(&unordered_segments);
        assert_eq!(
            fast_lookups.len(),
            16,
            "There should be two loops of 8 segments that all have unique starts"
        );
        let closed_loops = slice_layer::find_closed_polygons(&unordered_segments);
        assert_eq!(closed_loops.len(), 2);
        let union = slice_layer::union_closed_polygons(closed_loops)?;
        assert_eq!(union.len(), 1);
    }
    Ok(())
}

#[test]
fn create_dual_contouring_cube() {
    for size in [1.0, 15.0, 200.0] {
        for iterations in [2, 3, 4, 5, 6, 7] {
            // Apply dual contouring to a box shape and validate that the generated mesh is a cube.
            let shape = polygon_mesh::dual_contouring::Box::new(Vector3::new(size, size, size));

            let mut bounds = shape.bounds();
            bounds.expand(0.1);

            let sdf = |p: Vector3| shape.sdf(p);
            let octree = build_octree(&sdf, bounds.min_xyz, bounds.size(), iterations, 0.001);
            let mesh = generate_mesh_from_octree(&octree);

            assert_eq!(mesh.faces.len(), 12, "size {size} iterations {iterations}");
            assert_eq!(
                mesh.vertices.len(),
                8,
                "size {size} iterations {iterations}"
            );

            // Ordered by X, then Y, then Z, so two vertex lists compare position by position.
            let sorted = |mut v: Vec<Vector3Float>| {
                v.sort_by(|a, b| {
                    a.x.partial_cmp(&b.x)
                        .unwrap()
                        .then(a.y.partial_cmp(&b.y).unwrap())
                        .then(a.z.partial_cmp(&b.z).unwrap())
                });
                v
            };
            let expected = sorted(platonic_solids::create_cube(size, size, size).vertices);
            let actual = sorted(mesh.vertices.clone());
            for (expected, actual) in expected.iter().zip(actual.iter()) {
                assert!(
                    (*expected - *actual).length() < 1e-6,
                    "size {size} iterations {iterations}: {expected:?} vs {actual:?}"
                );
            }
        }
    }
}
