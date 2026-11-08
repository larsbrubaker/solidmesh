//! A part's fillet pinned to the last bit. The 3D bevel, its frames and the matrices that carry a
//! part work out their angles with the pure-Rust `libm` crate rather than std's `sin`, `cos`,
//! `tan`, `acos` and `atan2`, which call each platform's own maths library and differ in the last
//! bits between Windows, Linux, macOS and the browser. With `libm` a part gives the same mesh
//! everywhere, so a digest recorded on one platform holds on every other, and these pins fail on
//! any platform where that stops being true (CI runs them on all of them).
//!
//! The prism's corners are literal numbers, and its angles are none of the round ones, so every
//! wedge, setback and arc point comes through the trig; a rotation carries one build out through
//! the matrix trig as well.

use std::sync::Arc;

use solidmesh::bevel::part::{bevel_part, BevelPartRequest};
use solidmesh::topology::bevel_edge_group_resolver::create_groups_for_all_edges;
use solidmesh_math::{Matrix4X4, Vector3};
use solidmesh_mesh::{CancelToken, Mesh};

/// A triangular prism 12 tall, on a triangle with no round angle.
fn slanted_prism() -> Mesh {
    Mesh::from_arrays(
        &[
            0.0, 0.0, 0.0, //
            30.0, 0.0, 0.0, //
            9.0, 21.0, 0.0, //
            0.0, 0.0, 12.0, //
            30.0, 0.0, 12.0, //
            9.0, 21.0, 12.0,
        ],
        &[
            0, 2, 1, //
            3, 4, 5, //
            0, 1, 4, //
            0, 4, 3, //
            1, 2, 5, //
            1, 5, 4, //
            2, 0, 3, //
            2, 3, 5,
        ],
    )
}

/// The prism's fillet, every edge at `size` with `segments` across the arc, carried by `motion`.
fn filleted(size: f64, segments: i32, motion: Matrix4X4) -> Mesh {
    let prism = slanted_prism();
    let groups = create_groups_for_all_edges(&prism, 20.0, None).expect("a prism has edges");
    let request = BevelPartRequest {
        source_to_operation: motion,
        ..BevelPartRequest::new(Some(Arc::new(prism)), groups, size, segments, true, 20.0)
    };
    let outcome = bevel_part(request, &CancelToken::new(), None).expect("the build runs");
    for row in outcome.group_results.iter().flatten() {
        assert!(!row.failed(), "{:?}", row.failure);
    }
    outcome.mesh.expect("the prism's edges have room")
}

/// FNV-1a over the bits of every vertex coordinate, then every face's corners and normal: any
/// change in any last bit, or in the triangles, moves it.
pub(crate) fn digest(mesh: &Mesh) -> String {
    let mut hash: u64 = 0xcbf2_9ce4_8422_2325;
    let mut mix = |bytes: [u8; 4]| {
        for byte in bytes {
            hash ^= u64::from(byte);
            hash = hash.wrapping_mul(0x0000_0100_0000_01b3);
        }
    };
    for vertex in &mesh.vertices {
        for value in [vertex.x, vertex.y, vertex.z] {
            mix(value.to_bits().to_le_bytes());
        }
    }
    for face in mesh.faces.iter() {
        for corner in [face.v0, face.v1, face.v2] {
            mix(corner.to_le_bytes());
        }
        for value in [face.normal.x, face.normal.y, face.normal.z] {
            mix(value.to_bits().to_le_bytes());
        }
    }
    format!("{hash:016x}")
}

#[test]
fn a_filleted_prism_is_the_same_mesh_on_every_platform() {
    let mesh = filleted(1.5, 6, Matrix4X4::IDENTITY);
    assert_eq!(
        (mesh.vertices.len(), mesh.faces.len(), digest(&mesh)),
        (634, 1264, "1c585fd727d0d336".to_string())
    );
}

#[test]
fn a_filleted_prism_carried_by_a_rotation_is_the_same_mesh_on_every_platform() {
    let motion = Matrix4X4::create_rotation(Vector3::new(0.3, -0.7, 1.1));
    let mesh = filleted(0.75, 5, motion);
    assert_eq!(
        (mesh.vertices.len(), mesh.faces.len(), digest(&mesh)),
        (436, 868, "157113d9386e0de0".to_string())
    );
}
