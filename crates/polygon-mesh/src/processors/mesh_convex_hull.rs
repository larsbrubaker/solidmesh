//! The convex hull of a mesh's vertices, cached on the mesh, built by
//! `mi_convex_hull` (derived from MIConvexHull).
//! `TransformedAabbCache` (`crate::transformed_aabb_cache`) bounds a transformed mesh by its hull.
//!
//! The mesh owns a typed [`ConvexHullCache`] whose entry carries the [`Mesh::changed_count`] it
//! was built at, so any [`Mesh::mark_as_changed`] makes it stale, without a property bag or an
//! event. Every call is synchronous (a background hull of a mesh the owner may be editing
//! would race), so there is no in-flight marker either.

use std::sync::{Arc, Mutex};

use vector_math::{AxisAlignedBoundingBox, Vector3, Vector3Float};

use super::mi_convex_hull;
use crate::mesh::f32_key;
use crate::Mesh;

/// What [`Mesh::get_convex_hull`] hands back: the cached hull, or - when the mesh is flat or
/// empty - the mesh itself, which is then its own best hull.
#[derive(Debug, Clone)]
pub enum ConvexHull<'a> {
    Hull(Arc<Mesh>),
    Mesh(&'a Mesh),
}

impl ConvexHull<'_> {
    /// The hull mesh.
    pub fn mesh(&self) -> &Mesh {
        match self {
            ConvexHull::Hull(hull) => hull,
            ConvexHull::Mesh(mesh) => mesh,
        }
    }
}

/// The hull cached on a mesh, tagged with the `changed_count` it was built at. Cloning a mesh
/// starts the clone with an empty cache.
#[derive(Debug, Default)]
pub(crate) struct ConvexHullCache(Mutex<Option<(i32, Arc<Mesh>)>>);

impl Clone for ConvexHullCache {
    fn clone(&self) -> Self {
        Self::default()
    }
}

impl ConvexHullCache {
    fn get(&self, changed_count: i32) -> Option<Arc<Mesh>> {
        let guard = self.0.lock().unwrap_or_else(|e| e.into_inner());
        match &*guard {
            Some((count, hull)) if *count == changed_count => Some(hull.clone()),
            _ => None,
        }
    }

    fn set(&self, changed_count: i32, hull: Arc<Mesh>) {
        // last wins - two threads hulling the same (unchanged) mesh produce equivalent hulls
        *self.0.lock().unwrap_or_else(|e| e.into_inner()) = Some((changed_count, hull));
    }
}

impl Mesh {
    /// Gets the convex hull of the mesh, caching it on the mesh (and dropping the cache when the
    /// mesh changes). `None` when the mesh has fewer than 4 faces or the hull cannot be built.
    ///
    /// `_generate_async` is ignored and the hull is always built synchronously: a background
    /// task walking a mesh the owning thread may still be editing would race with the edit.
    pub fn get_convex_hull(&self, _generate_async: bool) -> Option<ConvexHull<'_>> {
        if self.faces.len() < 4 {
            return None;
        }
        let changed_count = self.changed_count();
        if let Some(hull) = self.convex_hull_cache.get(changed_count) {
            return Some(ConvexHull::Hull(hull));
        }
        self.create_hull_mesh(changed_count)
    }

    /// Whether a hull built at the mesh's current [`Mesh::changed_count`] is cached.
    pub fn has_cached_convex_hull(&self) -> bool {
        self.convex_hull_cache.get(self.changed_count()).is_some()
    }

    fn create_hull_mesh(&self, changed_count: i32) -> Option<ConvexHull<'_>> {
        let mut bounds = AxisAlignedBoundingBox::empty();
        // Get the convex hull for the mesh over the distinct vertices (float ==, first kept)
        let mut seen = std::collections::HashSet::new();
        let mut ch_vertex_list: Vec<[f64; 3]> = Vec::new();
        for &position in self.vertices.iter() {
            let has_nan = position.x.is_nan() || position.y.is_nan() || position.z.is_nan();
            // NaN never equals itself, so every NaN position is kept
            if has_nan
                || seen.insert((
                    f32_key(position.x),
                    f32_key(position.y),
                    f32_key(position.z),
                ))
            {
                ch_vertex_list.push([position.x as f64, position.y as f64, position.z as f64]);
                bounds.expand_to_include_point(Vector3::from_vector3_float(position));
            }
        }

        let tollerance = 0.01;

        if ch_vertex_list.is_empty()
            || bounds.x_size() <= tollerance
            || bounds.y_size() <= tollerance
            || bounds.z_size() <= tollerance
            || ch_vertex_list[0][0].is_nan()
        {
            // degenerate (flat or empty) - the mesh is its own best hull, but don't cache it
            return Some(ConvexHull::Mesh(self));
        }

        let faces = mi_convex_hull::create(&ch_vertex_list, tollerance)?;

        // create the mesh from the hull data
        let mut hull_mesh = Mesh::new();
        for face in faces {
            let vertex_count = hull_mesh.vertices.len() as i32;
            for v in face {
                let p = ch_vertex_list[v];
                hull_mesh
                    .vertices
                    .push(Vector3Float::from_vector3(Vector3::new(p[0], p[1], p[2])));
            }
            hull_mesh.faces.add(
                vertex_count,
                vertex_count + 1,
                vertex_count + 2,
                &hull_mesh.vertices,
            );
        }

        let hull = Arc::new(hull_mesh);
        self.convex_hull_cache.set(changed_count, hull.clone());
        Some(ConvexHull::Hull(hull))
    }
}
