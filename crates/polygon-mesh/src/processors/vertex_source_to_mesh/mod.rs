//! Paths to meshes. The flat fill every other entry point builds on ([`triangulate_faces`],
//! over [`CachedTesselator`]) is in this file; revolving is in [`revolve`] and extruding with
//! its bevel rings in [`extrude`], split by entry point for the file-size limit.

use agg_path::vertex_data::VertexData;
use agg_path::vertex_source::VertexSource;
use vector_math::{Matrix4X4, Vector2, Vector3};

mod extrude;
mod revolve;

pub use extrude::{extrude, extrude_bevelled};
pub use revolve::{revolve, RevolveError};

use crate::data_converters_2d::CachedTesselator;
use crate::mesh::Mesh;

/// Fills the path's outline at `z_height`, added to `mesh_to_add_to` or a new mesh (see
/// [`triangulate_vertices`]).
pub fn triangulate_faces(
    vertex_source: &dyn VertexSource,
    tesselated_source: Option<&mut CachedTesselator>,
    mesh_to_add_to: Option<Mesh>,
    z_height: f64,
) -> Mesh {
    triangulate_vertices(
        &vertex_source.vertices(),
        tesselated_source,
        mesh_to_add_to,
        z_height,
        None,
    )
}

/// Fills the outline at `z_height`, skipping triangles with two equal corners,
/// and transforms each corner by `in_matrix` unless it is absent or the identity. A caller
/// that passes `tesselated_source` gets the tessellation back in it.
pub fn triangulate_vertices(
    vertex_source: &[VertexData],
    tesselated_source: Option<&mut CachedTesselator>,
    mesh_to_add_to: Option<Mesh>,
    z_height: f64,
    in_matrix: Option<Matrix4X4>,
) -> Mesh {
    let matrix = in_matrix.filter(|m| *m != Matrix4X4::IDENTITY);
    let mut own_tesselator = CachedTesselator::new();
    let tesselator = tesselated_source.unwrap_or(&mut own_tesselator);
    tesselator.send_shape(vertex_source);

    let mut mesh = mesh_to_add_to.unwrap_or_default();
    for_each_triangle(tesselator, |v0, v1, v2| {
        let corners = [v0, v1, v2].map(|v| {
            let position = Vector3::from_vector2(v, z_height);
            match matrix {
                Some(m) => position.transform(m),
                None => position,
            }
        });
        mesh.create_face(&corners);
    });
    mesh
}

/// Calls `f` with the three corners of every tessellated triangle that has three distinct
/// corners (the `v0 == v1 || v1 == v2 || v2 == v0` skip every reader of the cache needs).
pub(crate) fn for_each_triangle(
    tesselator: &CachedTesselator,
    mut f: impl FnMut(Vector2, Vector2, Vector2),
) {
    let cache = &tesselator.indices_cache;
    for i in (0..cache.len().saturating_sub(2)).step_by(3) {
        let position = |k: usize| tesselator.vertices_cache[cache[i + k].index as usize];
        let (v0, v1, v2) = (position(0), position(1), position(2));
        if v0 == v1 || v1 == v2 || v2 == v0 {
            continue;
        }
        f(v0, v1, v2);
    }
}
