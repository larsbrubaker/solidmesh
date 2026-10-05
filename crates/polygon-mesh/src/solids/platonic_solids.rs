//! The Platonic solids as meshes. Ports agg-sharp `PolygonMesh/Solids/PlatonicSolids.cs`.

use vector_math::{math_helper, Matrix4X4, Vector2, Vector3, Vector3Float};

use crate::mesh::Mesh;

fn add(mesh: &mut Mesh, p: Vector3) {
    mesh.vertices.push(Vector3Float::from_vector3(p));
}

fn add_face(mesh: &mut Mesh, v0: i32, v1: i32, v2: i32) {
    mesh.faces.add(v0, v1, v2, &mesh.vertices);
}

fn scale_if_needed(mesh: &mut Mesh, scale: f64) {
    if scale != 1.0 {
        mesh.transform(Matrix4X4::create_scale_uniform(scale));
    }
}

/// A tetrahedron with its apex at (0, 0, 1) and its base below the origin.
pub fn create_tetrahedron(scale: f64) -> Mesh {
    let mut tetrahedron = Mesh::new();
    let base_point = Vector2::new(1.0, 0.0);
    let base_offset_z = -math_helper::degrees_to_radians(30.0).sin();
    add(
        &mut tetrahedron,
        Vector3::from_vector2(base_point, base_offset_z),
    );
    add(
        &mut tetrahedron,
        Vector3::from_vector2(
            base_point.get_rotated(math_helper::TAU / 3.0),
            base_offset_z,
        ),
    );
    add(
        &mut tetrahedron,
        Vector3::from_vector2(
            base_point.get_rotated(2.0 * math_helper::TAU / 3.0),
            base_offset_z,
        ),
    );
    add(&mut tetrahedron, Vector3::new(0.0, 0.0, 1.0));

    add_face(&mut tetrahedron, 0, 2, 1); // add reversed because we want to see the bottom.
    add_face(&mut tetrahedron, 0, 1, 3);
    add_face(&mut tetrahedron, 1, 2, 3);
    add_face(&mut tetrahedron, 2, 0, 3);

    scale_if_needed(&mut tetrahedron, scale);
    tetrahedron
}

/// A box of the given size centered on the origin.
pub fn create_cube(x_scale: f64, y_scale: f64, z_scale: f64) -> Mesh {
    create_cube_vector(Vector3::new(x_scale, y_scale, z_scale))
}

/// A box of size `scale` centered on the origin.
pub fn create_cube_vector(mut scale: Vector3) -> Mesh {
    scale *= 0.5; // the cube is -1 to 1 and we want it to be -.5 to .5 so it is a unit cube.
    let mut cube = Mesh::new();
    for (x, y, z) in [
        (-1.0, -1.0, 1.0),
        (1.0, -1.0, 1.0),
        (1.0, 1.0, 1.0),
        (-1.0, 1.0, 1.0),
        (-1.0, -1.0, -1.0),
        (1.0, -1.0, -1.0),
        (1.0, 1.0, -1.0),
        (-1.0, 1.0, -1.0),
    ] {
        add(&mut cube, Vector3::new(x, y, z) * scale);
    }

    for (v0, v1, v2) in [
        // front
        (0, 1, 2),
        (0, 2, 3),
        // left
        (4, 0, 3),
        (4, 3, 7),
        // right
        (1, 5, 6),
        (1, 6, 2),
        // back
        (4, 7, 6),
        (4, 6, 5),
        // top
        (3, 2, 6),
        (3, 6, 7),
        // bottom
        (4, 5, 1),
        (4, 1, 0),
    ] {
        add_face(&mut cube, v0, v1, v2);
    }

    cube
}

/// An octahedron with its six vertices on the axes at distance 1.
pub fn create_octahedron(scale: f64) -> Mesh {
    let mut octahedron = Mesh::new();
    for (x, y, z) in [
        (1.0, 0.0, 0.0),
        (-1.0, 0.0, 0.0),
        (0.0, 1.0, 0.0),
        (0.0, -1.0, 0.0),
        (0.0, 0.0, 1.0),
        (0.0, 0.0, -1.0),
    ] {
        add(&mut octahedron, Vector3::new(x, y, z));
    }

    for (v0, v1, v2) in [
        (4, 0, 2),
        (4, 2, 1),
        (4, 1, 3),
        (4, 3, 0),
        (5, 2, 0),
        (5, 1, 2),
        (5, 3, 1),
        (5, 0, 3),
    ] {
        add_face(&mut octahedron, v0, v1, v2);
    }

    scale_if_needed(&mut octahedron, scale);
    octahedron
}

/// A dodecahedron: twelve pentagons, each fanned into three triangles.
pub fn create_dodecahedron(scale: f64) -> Mesh {
    let mut dodecahedron = Mesh::new();

    // Golden ratio for vertex calculations
    let phi = (1.0 + 5.0_f64.sqrt()) / 2.0;
    let inv_phi = 1.0 / phi;

    // Cube vertices
    for i in [-1.0, 1.0] {
        for j in [-1.0, 1.0] {
            for k in [-1.0, 1.0] {
                add(&mut dodecahedron, Vector3::new(i, j, k));
            }
        }
    }

    // Rectangle vertices
    for i in [-1.0, 1.0] {
        add(&mut dodecahedron, Vector3::new(0.0, i * phi, i * inv_phi));
        add(&mut dodecahedron, Vector3::new(i * inv_phi, 0.0, i * phi));
        add(&mut dodecahedron, Vector3::new(i * phi, i * inv_phi, 0.0));
    }

    // The 12 pentagonal faces
    let faces: [[i32; 5]; 12] = [
        [0, 8, 9, 4, 16],
        [0, 16, 17, 2, 10],
        [0, 10, 11, 1, 8],
        [1, 11, 13, 5, 18],
        [1, 18, 19, 3, 9],
        [2, 12, 13, 11, 10],
        [2, 17, 15, 6, 12],
        [3, 14, 15, 17, 16],
        [3, 19, 7, 14, 15],
        [4, 9, 3, 15, 6],
        [4, 6, 12, 13, 5],
        [4, 5, 18, 19, 7],
    ];

    for face in faces {
        for i in 2..face.len() {
            add_face(&mut dodecahedron, face[0], face[i - 1], face[i]);
        }
    }

    scale_if_needed(&mut dodecahedron, scale);
    dodecahedron
}

/// An icosahedron built from three orthogonal golden rectangles.
pub fn create_icosahedron(scale: f64) -> Mesh {
    let mut icosahedron = Mesh::new();

    // Golden ratio for icosahedron construction
    let phi = (1.0 + 5.0_f64.sqrt()) / 2.0;

    // The 12 vertices form three orthogonal rectangles
    for (x, y, z) in [
        (-1.0, phi, 0.0),
        (1.0, phi, 0.0),
        (-1.0, -phi, 0.0),
        (1.0, -phi, 0.0),
        (0.0, -1.0, phi),
        (0.0, 1.0, phi),
        (0.0, -1.0, -phi),
        (0.0, 1.0, -phi),
        (phi, 0.0, -1.0),
        (phi, 0.0, 1.0),
        (-phi, 0.0, -1.0),
        (-phi, 0.0, 1.0),
    ] {
        add(&mut icosahedron, Vector3::new(x, y, z));
    }

    for (v0, v1, v2) in [
        // 5 faces around vertex 0
        (0, 11, 5),
        (0, 5, 1),
        (0, 1, 7),
        (0, 7, 10),
        (0, 10, 11),
        // 5 adjacent faces
        (1, 5, 9),
        (5, 11, 4),
        (11, 10, 2),
        (10, 7, 6),
        (7, 1, 8),
        // 5 faces around vertex 3
        (3, 9, 4),
        (3, 4, 2),
        (3, 2, 6),
        (3, 6, 8),
        (3, 8, 9),
        // 5 adjacent faces
        (4, 9, 5),
        (2, 4, 11),
        (6, 2, 10),
        (8, 6, 7),
        (9, 8, 1),
    ] {
        add_face(&mut icosahedron, v0, v1, v2);
    }

    scale_if_needed(&mut icosahedron, scale);
    icosahedron
}
