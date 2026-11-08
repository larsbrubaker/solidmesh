//! Two-mesh booleans through `csg_operations` and `boolean_processing`.
//!
//! `subtract_has_all_faces` and `subtraction_makes_closed_solid` subtract one centred 10 mm box
//! from another: the boxes come from `create_cube` and the difference from
//! `csg_operations::subtract`.

use solidmesh_math::{Matrix4X4, Vector3};
use solidmesh_mesh::csg::{boolean_processing, csg_operations, BooleanOptions, CsgModes};
use solidmesh_mesh::solids::platonic_solids::{
    create_cube, create_cube_vector, create_icosahedron,
};
use solidmesh_mesh::{CancelToken, Mesh};

fn are_equal(a: f64, b: f64) -> bool {
    let error_range = 0.001;
    a < b + error_range && a > b - error_range
}

fn none() -> CancelToken {
    CancelToken::new()
}

#[test]
fn subtract_works() {
    let centering = Vector3::new(100.0, 100.0, 20.0);
    let mut mesh_a = create_cube(40.0, 40.0, 40.0);
    mesh_a.translate(centering);
    let mut mesh_b = create_cube(40.0, 40.0, 40.0);
    let final_transform = Vector3::new(99.999927784394, 102.400700290798, 16.3588316937214);
    mesh_b.translate(final_transform);

    let result = csg_operations::subtract(&mesh_a, &mesh_b, &none()).expect("subtract runs");

    let a_aabb = mesh_a.get_axis_aligned_bounding_box();
    let intersect_aabb = result.get_axis_aligned_bounding_box();
    assert!(a_aabb.x_size() == 40.0 && a_aabb.y_size() == 40.0 && a_aabb.z_size() == 40.0);
    assert!(
        intersect_aabb.x_size() == 40.0
            && intersect_aabb.y_size() == 40.0
            && intersect_aabb.z_size() == 40.0
    );
}

#[test]
fn subtract_has_all_faces() {
    let x_offset = -0.4;
    let keep_mesh = create_cube(10.0, 10.0, 10.0);
    let mut subtract_mesh = create_cube(10.0, 10.0, 10.0);
    subtract_mesh.translate(Vector3::new(x_offset, -3.0, 2.0));
    let result_mesh =
        csg_operations::subtract(&keep_mesh, &subtract_mesh, &none()).expect("subtract runs");

    let bottom_of_subtract_faces: Vec<_> = result_mesh
        .faces
        .iter()
        .filter(|f| are_equal(f.normal.z as f64, 1.0) && face_at_height(&result_mesh, f, -3.0))
        .collect();

    let mut all_vertices: Vec<Vector3> = Vec::new();
    for face in &bottom_of_subtract_faces {
        for vi in [face.v0, face.v1, face.v2] {
            let position = Vector3::from_vector3_float(result_mesh.vertices[vi as usize]);
            if !all_vertices.contains(&position) {
                all_vertices.push(position);
            }
        }
    }

    assert!(has_position(&all_vertices, Vector3::new(4.6, 2.0, -3.0)));
    assert!(has_position(&all_vertices, Vector3::new(4.6, -5.0, -3.0)));
    assert!(has_position(&all_vertices, Vector3::new(-5.0, 2.0, -3.0)));
    assert!(has_position(&all_vertices, Vector3::new(-5.0, -5.0, -3.0)));

    assert_eq!(keep_mesh.get_non_manifold_edges().len(), 0);
    assert_eq!(subtract_mesh.get_non_manifold_edges().len(), 0);
    assert_eq!(result_mesh.get_non_manifold_edges().len(), 0);
}

fn has_position(all_vertices: &[Vector3], position: Vector3) -> bool {
    all_vertices
        .iter()
        .any(|vertex| vertex.equals_within(position, 0.0001))
}

fn face_at_height(mesh: &Mesh, face: &solidmesh_mesh::Face, height: f64) -> bool {
    [face.v0, face.v1, face.v2]
        .iter()
        .all(|&vi| are_equal(mesh.vertices[vi as usize].z as f64, height))
}

#[test]
fn subtract_icosahedrons_works() {
    let centering = Vector3::new(100.0, 100.0, 20.0);
    let mut mesh_a = create_icosahedron(35.0);
    mesh_a.translate(centering);
    let mut mesh_b = create_icosahedron(35.0);

    let final_transform = Vector3::new(105.240172225344, 92.9716306394062, 18.4619570261172);
    let rot_current = Vector3::new(4.56890223673623, -2.67874102322035, 1.02768848238523);
    let scale_current = Vector3::new(1.07853517569753, 0.964980885267323, 1.09290934544604);
    let transform_b = Matrix4X4::create_scale(scale_current.x, scale_current.y, scale_current.z)
        * Matrix4X4::create_rotation(rot_current)
        * Matrix4X4::create_translation_vector(final_transform);
    mesh_b.transform(transform_b);

    let a_aabb_before = mesh_a.get_axis_aligned_bounding_box();

    let result = csg_operations::subtract(&mesh_a, &mesh_b, &none()).expect("subtract runs");

    let a_aabb = mesh_a.get_axis_aligned_bounding_box();

    // source mesh should not be modified by the subtraction
    assert!(are_equal(a_aabb.x_size(), a_aabb_before.x_size()));
    assert!(are_equal(a_aabb.y_size(), a_aabb_before.y_size()));
    assert!(are_equal(a_aabb.z_size(), a_aabb_before.z_size()));

    // result should have faces and be smaller than or equal to the source
    assert!(!result.faces.is_empty());
    assert!(result.is_manifold());
}

#[test]
fn union_exactly_on_works() {
    let mesh_a = create_cube(40.0, 40.0, 40.0);
    let mesh_b = create_cube(40.0, 40.0, 40.0);

    let result = csg_operations::union(&mesh_a, &mesh_b, &none()).expect("union runs");

    let a_aabb = mesh_a.get_axis_aligned_bounding_box();
    let intersect_aabb = result.get_axis_aligned_bounding_box();
    assert!(a_aabb.x_size() == 40.0 && a_aabb.y_size() == 40.0 && a_aabb.z_size() == 40.0);
    assert!(
        intersect_aabb.x_size() == 40.0
            && intersect_aabb.y_size() == 40.0
            && intersect_aabb.z_size() == 40.0
    );
    assert!(result.is_manifold());
}

#[test]
fn ensure_simple_cube_intersection() {
    // the intersection of 2 cubes
    {
        let mut mesh_a = create_cube_vector(Vector3::new(10.0, 10.0, 10.0));
        mesh_a.translate(Vector3::new(-2.0, -2.0, -2.0));
        let mut mesh_b = create_cube_vector(Vector3::new(10.0, 10.0, 10.0));
        mesh_b.translate(Vector3::new(2.0, 2.0, 2.0));

        let result = csg_operations::intersect(&mesh_a, &mesh_b, &none()).expect("intersect runs");

        let a_aabb = mesh_a.get_axis_aligned_bounding_box();
        let intersect_aabb = result.get_axis_aligned_bounding_box();
        assert!(a_aabb.x_size() == 10.0 && a_aabb.y_size() == 10.0 && a_aabb.z_size() == 10.0);
        assert!(
            intersect_aabb.x_size() == 6.0
                && intersect_aabb.y_size() == 6.0
                && intersect_aabb.z_size() == 6.0
        );
        assert!(mesh_a.is_manifold());
        assert!(mesh_b.is_manifold());
        assert!(result.is_manifold());
    }

    // the intersection of 2 cubes that miss eachother
    {
        let mut mesh_a = create_cube_vector(Vector3::new(10.0, 10.0, 10.0));
        mesh_a.translate(Vector3::new(-5.0, -5.0, -5.0));
        let mut mesh_b = create_cube_vector(Vector3::new(10.0, 10.0, 10.0));
        mesh_b.translate(Vector3::new(5.0, 5.0, 5.0));

        let result = csg_operations::intersect(&mesh_a, &mesh_b, &none()).expect("intersect runs");

        let a_aabb = mesh_a.get_axis_aligned_bounding_box();
        let b_aabb = mesh_b.get_axis_aligned_bounding_box();
        let intersect_aabb = result.get_axis_aligned_bounding_box();

        assert!(a_aabb.x_size() == 10.0 && a_aabb.y_size() == 10.0 && a_aabb.z_size() == 10.0);
        assert!(
            a_aabb.min_xyz == Vector3::new(-10.0, -10.0, -10.0)
                && a_aabb.max_xyz == Vector3::new(0.0, 0.0, 0.0)
        );
        assert!(b_aabb.x_size() == 10.0 && b_aabb.y_size() == 10.0 && b_aabb.z_size() == 10.0);
        assert!(
            b_aabb.min_xyz == Vector3::new(0.0, 0.0, 0.0)
                && b_aabb.max_xyz == Vector3::new(10.0, 10.0, 10.0)
        );
        assert!(
            intersect_aabb.x_size() == 0.0
                && intersect_aabb.y_size() == 0.0
                && intersect_aabb.z_size() == 0.0
        );
    }
}

#[test]
fn ensure_simple_cube_union() {
    let mut mesh_a = create_cube_vector(Vector3::new(10.0, 10.0, 10.0));
    mesh_a.translate(Vector3::new(-2.0, 0.0, 0.0));
    let mut mesh_b = create_cube_vector(Vector3::new(10.0, 10.0, 10.0));
    mesh_b.translate(Vector3::new(2.0, 0.0, 0.0));

    let result = csg_operations::union(&mesh_a, &mesh_b, &none()).expect("union runs");

    let a_aabb = mesh_a.get_axis_aligned_bounding_box();
    let intersect_aabb = result.get_axis_aligned_bounding_box();
    assert!(a_aabb.x_size() == 10.0 && a_aabb.y_size() == 10.0 && a_aabb.z_size() == 10.0);
    assert!(
        intersect_aabb.x_size() == 14.0
            && intersect_aabb.y_size() == 10.0
            && intersect_aabb.z_size() == 10.0
    );
}

#[test]
fn ensure_simple_cube_subtraction() {
    let mut mesh_a = create_cube_vector(Vector3::new(10.0, 10.0, 10.0));
    mesh_a.translate(Vector3::new(-2.0, 0.0, 0.0));
    let mut mesh_b = create_cube_vector(Vector3::new(10.0, 10.0, 10.0));
    mesh_b.translate(Vector3::new(2.0, 0.0, 0.0));

    let result = csg_operations::subtract(&mesh_a, &mesh_b, &none()).expect("subtract runs");

    let a_aabb = mesh_a.get_axis_aligned_bounding_box();
    let intersect_aabb = result.get_axis_aligned_bounding_box();
    assert!(a_aabb.x_size() == 10.0 && a_aabb.y_size() == 10.0 && a_aabb.z_size() == 10.0);
    assert!(
        intersect_aabb.x_size() == 4.0
            && intersect_aabb.y_size() == 10.0
            && intersect_aabb.z_size() == 10.0
    );
}

#[test]
fn subtraction_makes_closed_solid() {
    let x_offset = -0.4;

    let box_combine = create_cube(10.0, 10.0, 10.0);
    let mut subtract = create_cube(10.0, 10.0, 10.0);
    subtract.translate(Vector3::new(x_offset, -3.0, 2.0));
    let result = csg_operations::subtract(&box_combine, &subtract, &none()).expect("subtract runs");

    assert!(result.is_manifold());
}

#[test]
fn boolean_with_empty_mesh_does_not_throw() {
    let empty_mesh = Mesh::new();
    let cube = create_cube(10.0, 10.0, 10.0);
    let options = BooleanOptions::default();
    let run = |a: &Mesh, b: &Mesh, mode| {
        boolean_processing::do_boolean(
            a,
            Matrix4X4::IDENTITY,
            b,
            Matrix4X4::IDENTITY,
            mode,
            &none(),
            &options,
        )
        .expect("a boolean with an empty operand does not fail")
    };

    // Subtract with empty mesh as operand B should return the original mesh unchanged
    assert!(!run(&cube, &empty_mesh, CsgModes::Subtract)
        .vertices
        .is_empty());
    // Subtract with empty mesh as operand A should return empty
    assert_eq!(run(&empty_mesh, &cube, CsgModes::Subtract).faces.len(), 0);
    // Intersect with empty mesh should return empty
    assert_eq!(run(&cube, &empty_mesh, CsgModes::Intersect).faces.len(), 0);
    // Union with empty mesh should return the non-empty mesh
    assert!(!run(&cube, &empty_mesh, CsgModes::Union).vertices.is_empty());
}
