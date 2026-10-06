//! Port of the test fixture `Fixtures/BevelObstructionTestMeshes.cs` (its `CreateRibEndingAt`): a
//! 20 mm cube with a rib standing on its top, along y from `rib_ends_at` to the back face, between
//! x = 1 and x = 3 - so the top-left edge's face is obstructed by a crease that does not end it.

#![allow(dead_code)]

use polygon_mesh::Mesh;
use vector_math::Vector3;

/// C# `CreateRibEndingAt`.
pub fn create_rib_ending_at(rib_ends_at: f64) -> Mesh {
    let size = 20.0;
    let near = 1.0;
    let far = 3.0;
    let rib_top = 24.0;
    let v = Vector3::new;

    let mut mesh = Mesh::new();

    // The top, z = size, around the rib's foot.
    mesh.create_face(&[
        v(0.0, 0.0, size),
        v(near, 0.0, size),
        v(near, rib_ends_at, size),
    ]);
    mesh.create_face(&[
        v(0.0, 0.0, size),
        v(near, rib_ends_at, size),
        v(0.0, size, size),
    ]);
    mesh.create_face(&[
        v(0.0, size, size),
        v(near, rib_ends_at, size),
        v(near, size, size),
    ]);
    mesh.create_face(&[
        v(near, 0.0, size),
        v(far, 0.0, size),
        v(far, rib_ends_at, size),
        v(near, rib_ends_at, size),
    ]);
    mesh.create_face(&[
        v(far, 0.0, size),
        v(size, 0.0, size),
        v(far, rib_ends_at, size),
    ]);
    mesh.create_face(&[
        v(far, rib_ends_at, size),
        v(size, 0.0, size),
        v(size, size, size),
    ]);
    mesh.create_face(&[
        v(far, rib_ends_at, size),
        v(size, size, size),
        v(far, size, size),
    ]);

    // The front, y = 0.
    mesh.create_face(&[v(0.0, 0.0, 0.0), v(size, 0.0, 0.0), v(size, 0.0, size)]);
    mesh.create_face(&[v(0.0, 0.0, 0.0), v(size, 0.0, size), v(far, 0.0, size)]);
    mesh.create_face(&[v(0.0, 0.0, 0.0), v(far, 0.0, size), v(near, 0.0, size)]);
    mesh.create_face(&[v(0.0, 0.0, 0.0), v(near, 0.0, size), v(0.0, 0.0, size)]);

    // The back, y = size, with the rib's end rising from it.
    mesh.create_face(&[v(0.0, size, 0.0), v(0.0, size, size), v(near, size, size)]);
    mesh.create_face(&[
        v(0.0, size, 0.0),
        v(near, size, size),
        v(near, size, rib_top),
    ]);
    mesh.create_face(&[
        v(0.0, size, 0.0),
        v(near, size, rib_top),
        v(far, size, rib_top),
    ]);
    mesh.create_face(&[v(0.0, size, 0.0), v(far, size, rib_top), v(far, size, size)]);
    mesh.create_face(&[v(0.0, size, 0.0), v(far, size, size), v(size, size, size)]);
    mesh.create_face(&[v(0.0, size, 0.0), v(size, size, size), v(size, size, 0.0)]);

    // The left, right and bottom.
    mesh.create_face(&[
        v(0.0, 0.0, 0.0),
        v(0.0, 0.0, size),
        v(0.0, size, size),
        v(0.0, size, 0.0),
    ]);
    mesh.create_face(&[
        v(size, 0.0, 0.0),
        v(size, size, 0.0),
        v(size, size, size),
        v(size, 0.0, size),
    ]);
    mesh.create_face(&[
        v(0.0, 0.0, 0.0),
        v(0.0, size, 0.0),
        v(size, size, 0.0),
        v(size, 0.0, 0.0),
    ]);

    // The rib.
    mesh.create_face(&[
        v(near, rib_ends_at, size),
        v(near, rib_ends_at, rib_top),
        v(near, size, rib_top),
        v(near, size, size),
    ]);
    mesh.create_face(&[
        v(far, rib_ends_at, size),
        v(far, size, size),
        v(far, size, rib_top),
        v(far, rib_ends_at, rib_top),
    ]);
    mesh.create_face(&[
        v(near, rib_ends_at, size),
        v(far, rib_ends_at, size),
        v(far, rib_ends_at, rib_top),
        v(near, rib_ends_at, rib_top),
    ]);
    mesh.create_face(&[
        v(near, rib_ends_at, rib_top),
        v(far, rib_ends_at, rib_top),
        v(far, size, rib_top),
        v(near, size, rib_top),
    ]);

    mesh.clean_and_merge();
    mesh
}
