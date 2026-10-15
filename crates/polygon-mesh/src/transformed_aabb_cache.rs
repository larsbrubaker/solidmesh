//! `TransformedAabbCache`: the bounds of a mesh under a transform, computed from its convex hull
//! and remembered per transform. The hull is `processors::mesh_convex_hull`, and
//! [`crate::Mesh::get_axis_aligned_bounding_box_transformed`] is the entry point.
//!
//! `Mesh::mark_as_changed` clears it ([`TransformedAabbCache::changed`]); the hull it reads is
//! itself keyed on [`crate::Mesh::changed_count`].

use std::sync::Mutex;

use vector_math::cs_math::{max_f64, min_f64};
use vector_math::{AxisAlignedBoundingBox, Matrix4X4, Vector3, Vector3Float};

use crate::Mesh;

/// Per-transform bounds of one mesh.
#[derive(Debug, Default)]
pub struct TransformedAabbCache {
    /// Bounds per transform; at most 101 entries, looked up by `Matrix4X4` equality.
    cache: Mutex<Vec<(Matrix4X4, AxisAlignedBoundingBox)>>,
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
    }

    /// The bounds of `mesh` (its convex hull where there is one) under `transform`.
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

        let convex_hull = mesh.get_convex_hull(true);
        let positions = match &convex_hull {
            Some(hull) => &hull.mesh().vertices,
            None => &mesh.vertices,
        };

        let bounds = calculate_bounds(positions, transform);
        cache.push((transform, bounds));
        bounds
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
