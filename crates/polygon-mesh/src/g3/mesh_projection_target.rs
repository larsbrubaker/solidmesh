//! geometry3Sharp `MeshProjectionTarget` (`spatial/BasicProjectionTargets.cs`): projects a point
//! onto the nearest point of a mesh's surface, found through the mesh's `DMeshAABBTree3`. The
//! `Reducer` (`reducer.rs`) projects onto the original mesh with it when Reduce's "Maintain
//! Surface" is on. The oriented `Project` overload (with the normal) and the `Auto` factories
//! have no ported caller and are not ported.

use super::dmesh_aabb_tree3::DMeshAABBTree3;
use super::mesh_constraints::ProjectionTarget;
use super::tri_distance::distance_sqr_closest;

/// C# `MeshProjectionTarget` over `mesh` and its spatial tree `spatial`.
pub(crate) struct MeshProjectionTarget<'a> {
    spatial: &'a DMeshAABBTree3<'a>,
}

impl<'a> MeshProjectionTarget<'a> {
    /// C# `new MeshProjectionTarget(tree.Mesh, tree)`: the target is the tree's mesh.
    pub(crate) fn new(spatial: &'a DMeshAABBTree3<'a>) -> Self {
        Self { spatial }
    }
}

impl ProjectionTarget for MeshProjectionTarget<'_> {
    /// C# `Project(vPoint, identifier)`: the nearest point on the nearest triangle. A point with
    /// no nearest triangle (an empty mesh, or a non-finite point every distance test rejects) is
    /// returned unmoved, where C# indexes triangle -1 and throws (divergence 2796).
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
