//! Port of the test fixture `Fixtures/RigidMotionMeshes.cs`: the rigid motions a bevel reading is
//! taken under to show it does not depend on where the part stands, moving a mesh by one, and the
//! topology readings (signature, most faces on an edge, two-cornered triangles) compared across
//! them.

#![allow(dead_code)]

use std::collections::{BTreeMap, HashMap};
use std::fmt;

use polygon_mesh::Mesh;
use vector_math::{math_helper, Matrix4X4, Vector3, Vector3Float};

use geometry_processing::bevel::surface::surface_mesh_closure;

/// C# `TheHangersOwnStandOff`: the translation the picture hanger's owner file stands its part at.
pub const THE_HANGERS_OWN_STAND_OFF: Vector3 = Vector3::new(-28.5, 23.66, 12.0);

/// C# `TheMotions`: the named rigid motions every invariance reading is taken under, in order.
pub fn the_motions() -> Vec<(&'static str, Matrix4X4)> {
    vec![
        ("identity", Matrix4X4::IDENTITY),
        (
            "stand-off",
            Matrix4X4::create_translation_vector(THE_HANGERS_OWN_STAND_OFF),
        ),
        (
            "far",
            Matrix4X4::create_translation_vector(Vector3::new(250.0, -130.0, 75.0)),
        ),
        ("rotated", turned_by_37_about_123()),
        (
            "rotated and moved",
            turned_by_37_about_123()
                * Matrix4X4::create_translation_vector(THE_HANGERS_OWN_STAND_OFF),
        ),
    ]
}

fn turned_by_37_about_123() -> Matrix4X4 {
    Matrix4X4::create_rotation_axis_angle(
        Vector3::new(1.0, 2.0, 3.0).get_normal(),
        math_helper::degrees_to_radians(37.0),
    )
}

/// C# `Move`: every vertex carried by `motion`, and the face normals recomputed.
pub fn move_mesh(mesh: &mut Mesh, motion: Matrix4X4) {
    for i in 0..mesh.vertices.len() {
        mesh.vertices[i] = Vector3Float::from_vector3(
            Vector3::from_vector3_float(mesh.vertices[i]).transform_position(motion),
        );
    }

    // A stored mesh carries its face normals as data, so a moved mesh whose normals were left
    // behind is a mesh of no placement at all: every reading taken off a normal would be the
    // unmoved part's. Recomputing them here is what makes the moved part the SAME part.
    mesh.calculate_normals();
}

/// C# `TopologySignature`: what a mesh's topology adds up to, compared across motions.
#[derive(Clone, Debug, PartialEq, Eq, Hash)]
pub struct TopologySignature {
    pub vertices: i32,
    pub faces: i32,
    pub open_edges: i32,
    pub valences: String,
}

impl fmt::Display for TopologySignature {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "{} vertices, {} faces, {} open, [{}]",
            self.vertices, self.faces, self.open_edges, self.valences
        )
    }
}

/// C# `SignatureOf`: the counts and the vertex valence histogram of a mesh, or "refused" for none.
pub fn signature_of(mesh: Option<&Mesh>) -> TopologySignature {
    let Some(mesh) = mesh else {
        return TopologySignature {
            vertices: 0,
            faces: 0,
            open_edges: 0,
            valences: "refused".to_string(),
        };
    };

    let mut face_count_of_vertex = vec![0i32; mesh.vertices.len()];

    for face in mesh.faces.iter() {
        face_count_of_vertex[face.v0 as usize] += 1;
        face_count_of_vertex[face.v1 as usize] += 1;
        face_count_of_vertex[face.v2 as usize] += 1;
    }

    let mut groups: BTreeMap<i32, i32> = BTreeMap::new();

    for &count in face_count_of_vertex.iter().filter(|&&count| count > 0) {
        *groups.entry(count).or_insert(0) += 1;
    }

    let histogram = groups
        .iter()
        .map(|(key, count)| format!("{key}x{count}"))
        .collect::<Vec<_>>()
        .join(" ");

    TopologySignature {
        vertices: mesh.vertices.len() as i32,
        faces: mesh.faces.len() as i32,
        open_edges: surface_mesh_closure::open_edge_count(mesh).0,
        valences: histogram,
    }
}

/// C# `MostFacesOnAnEdge`: the most triangles any one edge of the mesh is used by.
pub fn most_faces_on_an_edge(mesh: &Mesh) -> i32 {
    let mut uses: HashMap<(i32, i32), i32> = HashMap::with_capacity(mesh.faces.len() * 3);
    let mut most = 0;

    for face in mesh.faces.iter() {
        for (first, second) in [(face.v0, face.v1), (face.v1, face.v2), (face.v2, face.v0)] {
            let edge = (first.min(second), first.max(second));
            let already = uses.entry(edge).or_insert(0);

            *already += 1;
            most = most.max(*already);
        }
    }

    most
}

/// C# `TwoCorneredTriangleCount`: the triangles that name one vertex twice.
pub fn two_cornered_triangle_count(mesh: &Mesh) -> i32 {
    mesh.faces
        .iter()
        .filter(|face| face.v0 == face.v1 || face.v1 == face.v2 || face.v2 == face.v0)
        .count() as i32
}
