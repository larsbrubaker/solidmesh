//! Occluded-triangle removal (derived from geometry3Sharp): removes the triangles on the
//! "inside" of a mesh, for the Repair tool's Advanced remove mode (`advanced_repair`). Also
//! the area-weighted vertex normals, which supply the direction the per-vertex test nudges
//! along, since a `DMesh3` here stores none.
//!
//! Two modes, the ones the Repair tool offers: `FastWindingNumber` (Interior, over
//! `fast_winding`) and `SimpleOcclusionTest` (Occluded, six axis rays through
//! `DMeshAABBTree3::find_nearest_hit_triangle`). Vertices and triangles are tested in id
//! order on one thread, which makes the removal order (and so the ids the mesh frees) the
//! same on every run. Nothing interrupts the test, so there is no cancellation point.

use super::dmesh3::DMesh3;
use super::dmesh_aabb_tree3::DMeshAABBTree3;
use super::fast_winding::FastWinding;
use super::mesh_editor::remove_triangles;
use super::ray_intersection::Ray3d;
use super::vector3d::ZERO_TOLERANCEF;
use super::vector3d::{add, area, dot, normalized, scale, tri_normal, ZERO_TOLERANCE};

/// How "inside" is decided, one mode per Repair tool option.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum CalculationMode {
    /// Inside where the fast winding number exceeds `winding_iso_value`.
    FastWindingNumber,
    /// Inside where a ray along every axis direction hits the mesh.
    SimpleOcclusionTest,
}

/// The occluded-triangle removal's settings and result.
pub(crate) struct RemoveOccludedTriangles {
    /// if true, then we discard tris if any vertex is occluded.
    /// Otherwise we discard based on tri centroids
    pub(crate) per_vertex: bool,
    /// we nudge points out by this amount to try to counteract numerical issues
    pub(crate) normal_offset: f64,
    /// use this as winding isovalue for WindingNumber mode
    pub(crate) winding_iso_value: f64,
    pub(crate) inside_mode: CalculationMode,
    /// indices of removed triangles. List will be empty if nothing removed
    pub(crate) removed_t: Vec<i32>,
    /// Mesh.RemoveTriange() can return false, if that happens, this will be true
    pub(crate) remove_failed: bool,
}

impl RemoveOccludedTriangles {
    /// A removal using `inside_mode`, with the default settings.
    pub(crate) fn new(inside_mode: CalculationMode) -> Self {
        Self {
            per_vertex: false,
            normal_offset: ZERO_TOLERANCE,
            winding_iso_value: 0.5,
            inside_mode,
            removed_t: Vec::new(),
            remove_failed: false,
        }
    }

    /// Always true (there is no cancellation here).
    pub(crate) fn apply(&mut self, mesh: &mut DMesh3) -> bool {
        let removed = {
            let spatial = DMeshAABBTree3::new(mesh);
            let winding = match self.inside_mode {
                CalculationMode::FastWindingNumber => Some(FastWinding::new(&spatial)),
                CalculationMode::SimpleOcclusionTest => None,
            };

            // ray directions
            let ray_dirs: [[f64; 3]; 6] = [
                [1.0, 0.0, 0.0],
                [-1.0, 0.0, 0.0],
                [0.0, 1.0, 0.0],
                [0.0, -1.0, 0.0],
                [0.0, 0.0, 1.0],
                [0.0, 0.0, -1.0],
            ];

            let is_occluded = |pt: [f64; 3]| -> bool {
                if let Some(winding) = &winding {
                    return winding.fast_winding_number(pt) > self.winding_iso_value;
                }
                for dir in ray_dirs {
                    let hit_tid = spatial
                        .find_nearest_hit_triangle(&Ray3d::new(pt, dir), f64::MAX, None)
                        .unwrap_or(-1);
                    if hit_tid == -1 {
                        return false;
                    }
                }
                true
            };

            let mut vertices = Vec::new();
            if self.per_vertex {
                vertices = vec![false; mesh.max_vertex_id().max(0) as usize];
                let normals = compute_vertex_normals(mesh);
                for vid in mesh.vertex_indices() {
                    let mut c = mesh.vertex(vid);
                    let n = normals[vid as usize];
                    c = add(c, scale(n, self.normal_offset));
                    vertices[vid as usize] = is_occluded(c);
                }
            }

            let mut removed = Vec::new();
            for tid in mesh.triangle_indices() {
                let inside = if self.per_vertex {
                    let tri = mesh.get_triangle(tid);
                    vertices[tri[0] as usize]
                        || vertices[tri[1] as usize]
                        || vertices[tri[2] as usize]
                } else {
                    let [v0, v1, v2] = mesh.get_tri_vertices(tid as usize);
                    let c = mesh.get_tri_centroid(tid as usize);
                    let n = tri_normal(v0, v1, v2);
                    is_occluded(add(c, scale(n, self.normal_offset)))
                };
                if inside {
                    removed.push(tid);
                }
            }
            removed
        };

        self.removed_t = removed;
        if !self.removed_t.is_empty() {
            let ok = remove_triangles(mesh, self.removed_t.iter().copied(), true);
            self.remove_failed = !ok;
        }
        true
    }
}

/// Vertex normals: each vertex's normal is the area-weighted sum of its triangles' unit
/// normals, normalized when its squared length exceeds the single-precision zero tolerance.
/// Indexed by vertex id.
fn compute_vertex_normals(mesh: &DMesh3) -> Vec<[f64; 3]> {
    let mut normals = vec![[0.0; 3]; mesh.max_vertex_id().max(0) as usize];
    for ti in mesh.triangle_indices() {
        let tri = mesh.get_triangle(ti);
        let va = mesh.vertex(tri[0]);
        let vb = mesh.vertex(tri[1]);
        let vc = mesh.vertex(tri[2]);
        let n = tri_normal(va, vb, vc);
        let a = area(va, vb, vc);
        for v in tri {
            normals[v as usize] = add(normals[v as usize], scale(n, a));
        }
    }
    for n in &mut normals {
        if dot(*n, *n) > ZERO_TOLERANCEF {
            *n = normalized(*n);
        }
    }
    normals
}
