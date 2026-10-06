//! geometry3Sharp `mesh_ops/RemoveOccludedTriangles.cs`: removes the triangles on the "inside"
//! of a mesh, for the Repair tool's Advanced `RemoveMode` (`advanced_repair`). Also
//! `MeshNormals.Compute` (`mesh/MeshNormals.cs`, the area-weighted face average), which supplies
//! the vertex normals the per-vertex test nudges along, since a `DMesh3` here stores none.
//!
//! The two modes the Repair tool reaches are ported: `FastWindingNumber` (Interior, over
//! `fast_winding`) and `SimpleOcclusionTest` (Occluded, six axis rays through
//! `DMeshAABBTree3::find_nearest_hit_triangle`). C# tests vertices and triangles with
//! `gParallel.ForEach` and collects the removed ids under a lock, so their order varies from run
//! to run; here they are tested in id order, which makes the removal order (and so the ids the
//! mesh frees) deterministic. Nothing sets the C# `Progress` or `ProgressReporter` on this path,
//! so there is no cancellation point.

use super::dmesh3::DMesh3;
use super::dmesh_aabb_tree3::DMeshAABBTree3;
use super::fast_winding::FastWinding;
use super::mesh_editor::remove_triangles;
use super::ray_intersection::Ray3d;
use super::vector3d::ZERO_TOLERANCEF;
use super::vector3d::{add, area, dot, normalized, scale, tri_normal, ZERO_TOLERANCE};

/// C# `RemoveOccludedTriangles.CalculationMode`, the modes the Repair tool uses.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum CalculationMode {
    /// Inside where the fast winding number exceeds `WindingIsoValue`.
    FastWindingNumber,
    /// Inside where a ray along every axis direction hits the mesh.
    SimpleOcclusionTest,
}

/// g3 `RemoveOccludedTriangles`.
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
    /// C# `new RemoveOccludedTriangles(mesh)` with `InsideMode` set.
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

    /// C# `Apply()`: always true (there is no cancellation here).
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

/// C# `MeshNormals.Compute` (`Compute_FaceAvg_AreaWeighted`): each vertex's normal is the
/// area-weighted sum of its triangles' unit normals, normalized when its squared length exceeds
/// `MathUtil.ZeroTolerancef`. Indexed by vertex id.
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
