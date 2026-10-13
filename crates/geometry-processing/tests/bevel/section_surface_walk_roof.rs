//! The ROOF fixture of `SectionSurfaceWalkTests.cs` (`ARoof`, `Frame`, `Corners` and the nested
//! `RoofFrame`), which that class shares with `SectionSurfaceWalkFoldTests`: two panels meeting on a
//! convex hinge along y, each 10 wide and dropping 0.65625 over its 10, read back off the mesh's own
//! float vertices so a closed form written from them is exact - with the class's face and hinge
//! readings (`Triangle`, `NormalOf`, `OffTheFace`, `FaceHolding`, `IsTheHinge`) and the moved roof
//! (`ARoofAt`).

#![allow(dead_code)]

use polygon_mesh::{Mesh, MeshEdgeGraph};
use vector_math::{Vector3, Vector3Float};

use crate::mesh_surface_distance::closest_point_on_triangle;

/// C# `RoofFrame`: the roof read back off the MESH.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RoofFrame {
    /// The hinge's low end.
    pub origin: Vector3,
    /// Up the hinge.
    pub along: Vector3,
    /// Down the first panel, square to the hinge.
    pub down_a: Vector3,
    /// Down the second.
    pub down_b: Vector3,
    /// How long a panel is down its slope.
    pub length: f64,
}

impl RoofFrame {
    /// The point on the hinge at the given y.
    pub fn hinge(&self, along: f64) -> Vector3 {
        self.origin + (self.along * (along - self.origin.y))
    }
}

/// C# `ARoof`.
pub fn a_roof() -> Mesh {
    let mut mesh = Mesh::new();
    let drop = 0.65625;
    let f0 = Vector3::new(0.0, -5.0, 0.0);
    let f1 = Vector3::new(0.0, 5.0, 0.0);
    let a0 = Vector3::new(-10.0, -5.0, -drop);
    let a1 = Vector3::new(-10.0, 5.0, -drop);
    let b0 = Vector3::new(10.0, -5.0, -drop);
    let b1 = Vector3::new(10.0, 5.0, -drop);

    mesh.create_face(&[f0, f1, a1]);
    mesh.create_face(&[f0, a1, a0]);
    mesh.create_face(&[f0, b1, f1]);
    mesh.create_face(&[f0, b0, b1]);

    mesh.clean_and_merge();

    mesh
}

/// C# `Frame`: the hinge, the two down-slopes and the panel length as the mesh's own vertices carry
/// them.
pub fn frame(roof: &Mesh) -> RoofFrame {
    let at = |i: usize| Vector3::from_vector3_float(roof.vertices[i]);
    let (mut low, mut high, mut down_a) = (0, 0, 0);

    for i in 0..roof.vertices.len() {
        let here = at(i);

        if here.x.abs() < 1e-9 {
            low = if here.y < at(low).y { i } else { low };
            high = if here.y > at(high).y { i } else { high };
        } else if here.x < 0.0 && here.y < 0.0 {
            down_a = i;
        }
    }

    let origin = at(low);
    let slope = at(down_a) - origin;

    RoofFrame {
        origin,
        along: (at(high) - origin).get_normal(),
        down_a: slope.get_normal(),
        down_b: Vector3::new(-slope.x, slope.y, slope.z).get_normal(),
        length: slope.length(),
    }
}

/// C# `Corners`: a face's three vertex indices.
pub fn corners(mesh: &Mesh, face: usize) -> [i32; 3] {
    let face = mesh.faces[face];
    [face.v0, face.v1, face.v2]
}

/// C# `ARoofAt`: the same roof, moved - a rigid motion moves no diagonal but does move the float
/// grid.
pub fn a_roof_at(by: Vector3) -> Mesh {
    let mut mesh = a_roof();

    for i in 0..mesh.vertices.len() {
        mesh.vertices[i] =
            Vector3Float::from_vector3(Vector3::from_vector3_float(mesh.vertices[i]) + by);
    }

    mesh
}

/// C# `Triangle`: a face's three corners.
pub fn triangle(mesh: &Mesh, face: i32) -> (Vector3, Vector3, Vector3) {
    let drawn = mesh.faces[face as usize];
    (
        Vector3::from_vector3_float(mesh.vertices[drawn.v0 as usize]),
        Vector3::from_vector3_float(mesh.vertices[drawn.v1 as usize]),
        Vector3::from_vector3_float(mesh.vertices[drawn.v2 as usize]),
    )
}

/// C# `NormalOf`: a face's unit normal from its own corners.
pub fn normal_of(mesh: &Mesh, face: i32) -> Vector3 {
    let (a, b, c) = triangle(mesh, face);

    (b - a).cross(c - a).get_normal()
}

/// C# `OffTheFace`: how far a point stands from a face, within the face's own footprint.
pub fn off_the_face(mesh: &Mesh, face: i32, at: Vector3) -> f64 {
    let (a, b, c) = triangle(mesh, face);

    (closest_point_on_triangle(at, a, b, c) - at).length()
}

/// C# `FaceHolding`: the first face a point lies in, or -1.
pub fn face_holding(mesh: &Mesh, at: Vector3) -> i32 {
    (0..mesh.faces.len() as i32)
        .find(|&face| off_the_face(mesh, face, at) < 1e-9)
        .unwrap_or(-1)
}

/// C# `IsTheHinge`: an edge both of whose ends stand on the roof's hinge, x = 0.
pub fn is_the_hinge(mesh: &Mesh, graph: &MeshEdgeGraph, edge: i32) -> bool {
    (mesh.vertices[graph.get_vertex0(edge) as usize].x as f64).abs() < 1e-9
        && (mesh.vertices[graph.get_vertex1(edge) as usize].x as f64).abs() < 1e-9
}
