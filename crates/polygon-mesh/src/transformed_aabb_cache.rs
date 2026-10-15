//! `TransformedAabbCache`: the bounds of a mesh under a transform, remembered per transform.
//! [`crate::Mesh::get_axis_aligned_bounding_box_transformed`] is the entry point.
//!
//! The bounds are exactly those of every vertex transformed, whatever the mesh's vertex order or
//! triangulation. To stay fast they are measured over the vertices on or outside the mesh's
//! convex hull (`processors::hull_bounds_points`), worked out once per mesh change; a vertex
//! inside the hull can never be an extreme.
//!
//! `Mesh::mark_as_changed` clears it ([`TransformedAabbCache::changed`]); the hull and the point
//! set are keyed on [`crate::Mesh::changed_count`].

use std::sync::{Arc, Mutex};

use vector_math::ieee_math::{max_f64, min_f64};
use vector_math::{AxisAlignedBoundingBox, Matrix4X4, Vector3, Vector3Float};

use crate::processors::hull_bounds_points::points_on_or_outside_hull;
use crate::processors::ConvexHull;
use crate::Mesh;

/// Per-transform bounds of one mesh.
#[derive(Debug, Default)]
pub struct TransformedAabbCache {
    /// Bounds per transform; at most 101 entries, looked up by `Matrix4X4` equality.
    cache: Mutex<Vec<(Matrix4X4, AxisAlignedBoundingBox)>>,
    /// The vertices bounds are measured over, tagged with the `changed_count` they were found at.
    points: Mutex<Option<(i32, Arc<Vec<Vector3Float>>)>>,
}

impl Clone for TransformedAabbCache {
    /// A clone starts empty (it is a cache).
    fn clone(&self) -> Self {
        Self::default()
    }
}

impl TransformedAabbCache {
    /// Forgets every cached transform.
    pub fn changed(&self) {
        let mut cache = self.cache.lock().unwrap_or_else(|e| e.into_inner());
        // set the aabbTransform to a bad value so we detect in needs to be recreated
        cache.clear();
        self.points.lock().unwrap_or_else(|e| e.into_inner()).take();
    }

    /// The bounds of every vertex of `mesh` under `transform`.
    pub fn get_axis_aligned_bounding_box(
        &self,
        mesh: &Mesh,
        transform: Matrix4X4,
    ) -> AxisAlignedBoundingBox {
        let mut cache = self.cache.lock().unwrap_or_else(|e| e.into_inner());
        if cache.len() > 100 {
            cache.clear();
        }

        // if we already have the transform with exact bounds than return it
        if let Some((_, aabb)) = cache.iter().find(|(m, _)| *m == transform) {
            // return, the fast cache for this transform is correct
            return *aabb;
        }

        let bounds = match self.bounds_points(mesh) {
            Some(points) => calculate_bounds(&points, transform),
            None => calculate_bounds(&mesh.vertices, transform),
        };
        cache.push((transform, bounds));
        bounds
    }
}

impl TransformedAabbCache {
    /// The vertices on or outside the mesh's hull, found once per mesh change. `None` when the
    /// mesh has no built hull (too few faces, flat, or the hull failed): then every vertex is
    /// measured.
    fn bounds_points(&self, mesh: &Mesh) -> Option<Arc<Vec<Vector3Float>>> {
        let changed_count = mesh.changed_count();
        let mut points = self.points.lock().unwrap_or_else(|e| e.into_inner());
        if let Some((count, cached)) = &*points {
            if *count == changed_count {
                return Some(cached.clone());
            }
        }
        let ConvexHull::Hull(hull) = mesh.get_convex_hull(true)? else {
            return None;
        };
        let found = Arc::new(points_on_or_outside_hull(mesh, &hull));
        *points = Some((changed_count, found.clone()));
        Some(found)
    }
}

fn calculate_bounds(vertices: &[Vector3Float], transform: Matrix4X4) -> AxisAlignedBoundingBox {
    // calculate the aabb for the current transform
    let mut min_xyz = Vector3::new(f64::MAX, f64::MAX, f64::MAX);
    let mut max_xyz = Vector3::new(f64::MIN, f64::MIN, f64::MIN);

    for &position_in in vertices {
        let position = Vector3::from_vector3_float(position_in).transform(transform);

        min_xyz.x = min_f64(min_xyz.x, position.x);
        min_xyz.y = min_f64(min_xyz.y, position.y);
        min_xyz.z = min_f64(min_xyz.z, position.z);

        max_xyz.x = max_f64(max_xyz.x, position.x);
        max_xyz.y = max_f64(max_xyz.y, position.y);
        max_xyz.z = max_f64(max_xyz.z, position.z);
    }

    AxisAlignedBoundingBox::new(min_xyz, max_xyz)
}
