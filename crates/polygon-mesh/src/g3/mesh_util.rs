//! geometry3Sharp's `MeshUtil` (`mesh/MeshUtil.cs`): the uniform smoother the `Remesher`
//! uses, the discrete Gaussian curvature, and the edge-collapse / edge-flip checks and
//! flip-candidate triangles and normals the hole fillers (`MinimalHoleFill`) use; plus the two
//! `DMesh3` per-element measures they need (`GetTriInternalAngleR`, `VtxOneRingCentroid` from
//! `mesh/DMesh3.cs`). Vector arithmetic is in `vector3d`.

// Consumed by `MinimalHoleFill` (mesh repair), which lands in a later slice; until then only the
// tests reach some of it.
#![allow(dead_code)]

use super::dmesh3::DMesh3;
use super::dmesh3_queries::{find_tri_index, orient_tri_edge};
use super::vector3d::{
    angle_r, cross, dot, fast_normal_direction, normalized, sub, tri_normal, TWO_PI,
};

impl DMesh3 {
    /// C# `GetTriInternalAngleR(tID, i)`: the interior angle at corner `i` in radians.
    pub(crate) fn get_tri_internal_angle_r(&self, tid: i32, i: i32) -> f64 {
        let t = self.triangles[tid as usize];
        let a = self.vertices[t[0] as usize];
        let b = self.vertices[t[1] as usize];
        let c = self.vertices[t[2] as usize];
        if i == 0 {
            angle_r(normalized(sub(b, a)), normalized(sub(c, a)))
        } else if i == 1 {
            angle_r(normalized(sub(a, b)), normalized(sub(c, b)))
        } else {
            angle_r(normalized(sub(a, c)), normalized(sub(b, c)))
        }
    }

    /// C# `VtxOneRingCentroid`: the mean of the one-ring neighbours (zero for a dead or
    /// isolated vertex), summed in the vertex's edge-list order.
    pub(crate) fn vtx_one_ring_centroid(&self, vid: i32) -> [f64; 3] {
        let mut centroid = [0.0; 3];
        if self.vertices_refcount.is_valid(vid) {
            let mut n = 0;
            for eid in self.vertex_edges.iter(vid) {
                let other = self.vertices[self.edge_other_v(eid, vid) as usize];
                centroid[0] += other[0];
                centroid[1] += other[1];
                centroid[2] += other[2];
                n += 1;
            }
            if n > 0 {
                let d = 1.0 / n as f64;
                centroid[0] *= d;
                centroid[1] *= d;
                centroid[2] *= d;
            }
        }
        centroid
    }
}

/// C# `MeshUtil.UniformSmooth`: `(1-t) v + t c` with `c` the one-ring centroid, `t` in [0,1].
pub(crate) fn uniform_smooth(mesh: &DMesh3, vid: i32, t: f64) -> [f64; 3] {
    let mut v = mesh.vertex(vid);
    let c = mesh.vtx_one_ring_centroid(vid);
    let s = 1.0 - t;
    v[0] = s * v[0] + t * c[0];
    v[1] = s * v[1] + t * c[1];
    v[2] = s * v[2] + t * c[2];
    v
}

/// C# `MeshUtil.DiscreteGaussCurvature`: the sum of the interior angles around `vid`, minus
/// 2 pi; zero on flat areas.
pub(crate) fn discrete_gauss_curvature(mesh: &DMesh3, vid: i32) -> f64 {
    let mut angle_sum = 0.0;
    for tid in mesh.vtx_triangles(vid) {
        let et = mesh.get_triangle(tid);
        let idx = find_tri_index(vid, &et);
        angle_sum += mesh.get_tri_internal_angle_r(tid, idx);
    }
    angle_sum - TWO_PI
}

/// C# `MeshUtil.CheckIfCollapseCreatesFlip`: would collapsing `edge_id` to `newv` flip the
/// normal of any surrounding triangle (or hit an invalid neighbourhood)?
pub(crate) fn check_if_collapse_creates_flip(mesh: &DMesh3, edge_id: i32, newv: [f64; 3]) -> bool {
    let edge_info = mesh.get_edge(edge_id);
    let tc = edge_info[2];
    let td = edge_info[3];

    for j in 0..2 {
        let vid = edge_info[j];
        let vother = edge_info[(j + 1) % 2];

        for tid in mesh.vtx_triangles(vid) {
            if tid == tc || tid == td {
                continue;
            }

            let curt = mesh.get_triangle(tid);
            if curt[0] == vother || curt[1] == vother || curt[2] == vother {
                return true; // invalid nbrhood for collapse
            }

            let va = mesh.vertex(curt[0]);
            let vb = mesh.vertex(curt[1]);
            let vc = mesh.vertex(curt[2]);
            let ncur = cross(sub(vb, va), sub(vc, va));
            let sign = if curt[0] == vid {
                let nnew = cross(sub(vb, newv), sub(vc, newv));
                dot(ncur, nnew)
            } else if curt[1] == vid {
                let nnew = cross(sub(newv, va), sub(vc, va));
                dot(ncur, nnew)
            } else if curt[2] == vid {
                let nnew = cross(sub(vb, va), sub(newv, va));
                dot(ncur, nnew)
            } else {
                // C# throws "should never be here!": vtx_triangles only yields triangles
                // that contain vid.
                unreachable!("vtx_triangles yielded a triangle without its vertex")
            };

            if sign <= 0.0 {
                return true;
            }
        }
    }
    false
}

/// C# `MeshUtil.edge_flip_metric`.
fn edge_flip_metric(n0: [f64; 3], n1: [f64; 3], flip_dot_tol: f64) -> f64 {
    if flip_dot_tol == 0.0 {
        dot(n0, n1)
    } else {
        dot(normalized(n0), normalized(n1))
    }
}

/// The oriented `[a, b, c, d]` of an interior edge: `oa, ob` in `t0`'s winding, `c` opposite
/// in `t0`, `d` opposite in `t1`.
fn oriented_flip_quad(mesh: &DMesh3, eid: i32) -> (i32, i32, i32, i32) {
    let einfo = mesh.get_edge(eid);
    let ov = mesh.get_edge_opposing_v(eid);
    let tri_v = mesh.get_triangle(einfo[2]);
    let mut oa = einfo[0];
    let mut ob = einfo[1];
    orient_tri_edge(&mut oa, &mut ob, &tri_v);
    (oa, ob, ov[0], ov[1])
}

/// C# `MeshUtil.CheckIfEdgeFlipCreatesFlip`: after flipping `eid`, does any old/new normal
/// pair have a dot at or below `flip_dot_tol` (C#'s default is 0)?
pub(crate) fn check_if_edge_flip_creates_flip(mesh: &DMesh3, eid: i32, flip_dot_tol: f64) -> bool {
    debug_assert!(!mesh.is_boundary_edge(eid));
    let (oa, ob, c, d) = oriented_flip_quad(mesh, eid);
    let v_c = mesh.vertex(c);
    let v_d = mesh.vertex(d);
    let v_oa = mesh.vertex(oa);
    let v_ob = mesh.vertex(ob);
    let n0 = fast_normal_direction(v_oa, v_ob, v_c);
    let n1 = fast_normal_direction(v_ob, v_oa, v_d);
    let f0 = fast_normal_direction(v_c, v_d, v_ob);
    if edge_flip_metric(n0, f0, flip_dot_tol) <= flip_dot_tol
        || edge_flip_metric(n1, f0, flip_dot_tol) <= flip_dot_tol
    {
        return true;
    }

    let f1 = fast_normal_direction(v_d, v_c, v_oa);
    if edge_flip_metric(n0, f1, flip_dot_tol) <= flip_dot_tol
        || edge_flip_metric(n1, f1, flip_dot_tol) <= flip_dot_tol
    {
        return true;
    }

    false
}

/// C# `MeshUtil.GetEdgeFlipTris`: the edge's two triangles and the two a flip would make,
/// as `(orig_t0, orig_t1, flip_t0, flip_t1)`.
pub(crate) fn get_edge_flip_tris(
    mesh: &DMesh3,
    eid: i32,
) -> ([i32; 3], [i32; 3], [i32; 3], [i32; 3]) {
    let (oa, ob, c, d) = oriented_flip_quad(mesh, eid);
    ([oa, ob, c], [ob, oa, d], [c, d, ob], [d, c, oa])
}

/// C# `MeshUtil.GetEdgeFlipNormals`: the unit normals of the edge's two triangles and of the
/// two a flip would make, as `(n1, n2, on1, on2)`.
pub(crate) fn get_edge_flip_normals(
    mesh: &DMesh3,
    eid: i32,
) -> ([f64; 3], [f64; 3], [f64; 3], [f64; 3]) {
    let (oa, ob, c, d) = oriented_flip_quad(mesh, eid);
    let v_c = mesh.vertex(c);
    let v_d = mesh.vertex(d);
    let v_oa = mesh.vertex(oa);
    let v_ob = mesh.vertex(ob);
    (
        tri_normal(v_oa, v_ob, v_c),
        tri_normal(v_ob, v_oa, v_d),
        tri_normal(v_c, v_d, v_ob),
        tri_normal(v_d, v_c, v_oa),
    )
}
