//! Mesh projection target (derived from geometry3Sharp): projects a point
//! onto the nearest point of a mesh's surface, found through the mesh's `DMeshAABBTree3`. The
//! `Reducer` (`reducer.rs`) projects onto the original mesh with it when Reduce's "Maintain
//! Surface" is on.

use super::dmesh_aabb_tree3::DMeshAABBTree3;
use super::mesh_constraints::ProjectionTarget;
use super::tri_distance::distance_sqr_closest;

/// Projects onto the mesh of spatial tree `spatial`.
pub(crate) struct MeshProjectionTarget<'a> {
    spatial: &'a DMeshAABBTree3<'a>,
}

impl<'a> MeshProjectionTarget<'a> {
    /// A target over the tree's mesh.
    pub(crate) fn new(spatial: &'a DMeshAABBTree3<'a>) -> Self {
        Self { spatial }
    }
}

impl ProjectionTarget for MeshProjectionTarget<'_> {
    /// The nearest point on the nearest triangle. A point with
    /// no nearest triangle (an empty mesh, or a non-finite point every distance test rejects) is
    /// returned unmoved: Reduce's "Maintain Surface" only projects points the quadrics produced,
    /// and one that cannot be projected is better kept than failing the whole reduce.
    fn project(&self, point: [f64; 3], _identifier: i32) -> [f64; 3] {
        let t_nearest_id = self.spatial.find_nearest_triangle(point, f64::MAX);
        if t_nearest_id < 0 {
            return point;
        }
        let triangle = self.spatial.mesh.get_tri_vertices(t_nearest_id as usize);
        let (_, near_pt) = distance_sqr_closest(&triangle, point);
        near_pt
    }
}
