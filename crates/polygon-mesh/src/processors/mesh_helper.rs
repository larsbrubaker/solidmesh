//! agg-sharp `PolygonMesh/Processors/MeshHelper.cs`: a flat two-triangle plane centred on the
//! origin in z = 0, the mesh MatterCAD's `FloorDrawable` gives the bed (mattercad-renderer's
//! `bed_shadow_gpu::bed_quad` draws the same plane) and textures to mint its uvs.

use vector_math::{Vector2, Vector3, Vector3Float};

use crate::Mesh;

/// C# `MeshHelper.CreatePlane(xScale, yScale)`: a plane `x_scale` by `y_scale`, centred on the
/// origin, facing +Z.
pub fn create_plane(x_scale: f64, y_scale: f64) -> Mesh {
    create_plane_vector(Vector2::new(x_scale, y_scale))
}

/// C# `MeshHelper.CreatePlane(Vector2 scaleIn)`.
pub fn create_plane_vector(scale_in: Vector2) -> Mesh {
    // the plane is -1 to 1 and we want it to be -.5 to .5 so it is a unit cube.
    // C# `new Vector3(scaleIn * .5)`: z is 0.
    let half = scale_in * 0.5;
    let scale = Vector3::new(half.x, half.y, 0.0);
    let mut plane = Mesh::new();
    for (x, y) in [(-1.0, -1.0), (1.0, -1.0), (1.0, 1.0), (-1.0, 1.0)] {
        plane
            .vertices
            .push(Vector3Float::from_vector3(Vector3::new(x, y, 0.0) * scale));
    }

    // front
    plane.faces.add(0, 1, 2, &plane.vertices);
    plane.faces.add(0, 2, 3, &plane.vertices);

    plane
}
