//! The ROOF fixture of `SectionSurfaceWalkTests.cs` (`ARoof`, `Frame`, `Corners` and the nested
//! `RoofFrame`), which that class shares with `SectionSurfaceWalkFoldTests`: two panels meeting on a
//! convex hinge along y, each 10 wide and dropping 0.65625 over its 10, read back off the mesh's own
//! float vertices so a closed form written from them is exact.

#![allow(dead_code)]

use polygon_mesh::Mesh;
use vector_math::Vector3;

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
