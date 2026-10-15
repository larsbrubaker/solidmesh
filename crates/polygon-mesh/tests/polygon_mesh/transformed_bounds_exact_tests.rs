//! A mesh's transformed bounds are exactly the bounds of every vertex transformed, whatever the
//! order of its vertices or how its faces are triangulated. Scene bounds feed Fit to Bounds, Align
//! and placement on the bed, so a part rebuilt with the same surface but other vertices must land
//! in exactly the same place. The convex hull that speeds the bounds up is built with a tolerance
//! and may leave out a vertex just outside it; that must never move the bounds.

use polygon_mesh::processors::ConvexHull;
use polygon_mesh::Mesh;
use vector_math::{AxisAlignedBoundingBox, Matrix4X4, Vector3, Vector3Float};

/// The bounds of every vertex of `mesh` under `transform`, measured directly.
fn every_vertex_bounds(mesh: &Mesh, transform: Matrix4X4) -> AxisAlignedBoundingBox {
    let mut bounds = AxisAlignedBoundingBox::empty();
    for &v in mesh.vertices.iter() {
        bounds.expand_to_include_point(Vector3::from_vector3_float(v).transform(transform));
    }
    bounds
}

/// A closed prism over a regular `sides`-gon of `radius`, `height` tall. The rim points are so
/// close together that the hull's tolerance leaves many of them out. `reversed` lists the
/// vertices in the opposite order and fans each cap from another corner, so the solid is the
/// same with other vertex indices and other cap triangles.
fn fine_prism(sides: usize, radius: f64, height: f64, reversed: bool) -> Mesh {
    let mut positions = Vec::new();
    for i in 0..sides {
        let angle = std::f64::consts::TAU * i as f64 / sides as f64;
        let (s, c) = angle.sin_cos();
        positions.push(Vector3::new(radius * c, radius * s, 0.0));
    }
    for i in 0..sides {
        let p = positions[i];
        positions.push(Vector3::new(p.x, p.y, height));
    }
    let count = positions.len();
    // where each logical vertex sits in the mesh's vertex list
    let slot = |i: usize| if reversed { count - 1 - i } else { i };
    let mut mesh = Mesh::new();
    let mut ordered = vec![Vector3Float::ZERO; count];
    for (i, &p) in positions.iter().enumerate() {
        ordered[slot(i)] = Vector3Float::from_vector3(p);
    }
    mesh.vertices.extend(ordered);
    let mut add = |a: usize, b: usize, c: usize| {
        let (a, b, c) = (slot(a) as i32, slot(b) as i32, slot(c) as i32);
        let vertices = mesh.vertices.clone();
        mesh.faces.add(a, b, c, &vertices);
    };
    let fan_from = if reversed { sides / 3 } else { 0 };
    for k in 1..sides - 1 {
        let (i, j) = ((fan_from + k) % sides, (fan_from + k + 1) % sides);
        add(fan_from, j, i);
        add(sides + fan_from, sides + i, sides + j);
    }
    for i in 0..sides {
        let j = (i + 1) % sides;
        add(i, j, sides + j);
        add(i, sides + j, sides + i);
    }
    mesh.mark_as_changed();
    mesh
}

fn turns() -> Vec<Matrix4X4> {
    vec![
        Matrix4X4::IDENTITY,
        Matrix4X4::create_rotation_z(0.123),
        Matrix4X4::create_rotation_z(0.7) * Matrix4X4::create_rotation_x(0.31),
        Matrix4X4::create_rotation_y(-1.1)
            * Matrix4X4::create_rotation_z(2.9)
            * Matrix4X4::create_translation(12.5, -3.25, 7.0),
    ]
}

#[test]
fn same_solid_with_other_vertex_order_and_triangles_has_identical_bounds() {
    let forward = fine_prism(400, 20.0, 5.0, false);
    let reversed = fine_prism(400, 20.0, 5.0, true);
    for transform in turns() {
        let a = forward.get_axis_aligned_bounding_box_transformed(transform);
        let b = reversed.get_axis_aligned_bounding_box_transformed(transform);
        assert_eq!(a, b, "bounds differ under {transform:?}");
        assert_eq!(a, every_vertex_bounds(&forward, transform));
    }
}

#[test]
fn a_vertex_the_hull_tolerance_leaves_out_still_bounds_the_mesh() {
    // a 10 mm cube whose top is a very flat pyramid: the apex is 0.005 mm above the top corners,
    // inside the hull's 0.01 mm tolerance
    let mut mesh = Mesh::new();
    let corners = [
        (0.0, 0.0, 0.0),
        (10.0, 0.0, 0.0),
        (10.0, 10.0, 0.0),
        (0.0, 10.0, 0.0),
        (0.0, 0.0, 10.0),
        (10.0, 0.0, 10.0),
        (10.0, 10.0, 10.0),
        (0.0, 10.0, 10.0),
        (5.0, 5.0, 10.005),
    ];
    for (x, y, z) in corners {
        mesh.vertices.push(Vector3Float::new(x, y, z));
    }
    let faces = [
        [0, 2, 1],
        [0, 3, 2],
        [0, 1, 5],
        [0, 5, 4],
        [1, 2, 6],
        [1, 6, 5],
        [2, 3, 7],
        [2, 7, 6],
        [3, 0, 4],
        [3, 4, 7],
        [4, 5, 8],
        [5, 6, 8],
        [6, 7, 8],
        [7, 4, 8],
    ];
    for [a, b, c] in faces {
        let vertices = mesh.vertices.clone();
        mesh.faces.add(a, b, c, &vertices);
    }
    mesh.mark_as_changed();

    // the case this pins: the hull really does leave the apex out
    let apex = Vector3Float::new(5.0, 5.0, 10.005);
    match mesh.get_convex_hull(false) {
        Some(ConvexHull::Hull(hull)) => assert!(
            !hull.vertices.contains(&apex),
            "the hull keeps the apex, so this mesh no longer tests a dropped extreme"
        ),
        other => panic!("expected a built hull, got {other:?}"),
    }

    let bounds = mesh.get_axis_aligned_bounding_box_transformed(Matrix4X4::IDENTITY);
    assert_eq!(bounds.max_xyz.z, f64::from(10.005f32));
    for transform in turns() {
        assert_eq!(
            mesh.get_axis_aligned_bounding_box_transformed(transform),
            every_vertex_bounds(&mesh, transform)
        );
    }
}
