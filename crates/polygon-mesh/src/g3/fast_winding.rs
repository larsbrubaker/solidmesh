//! geometry3Sharp's fast mesh winding number over a `DMeshAABBTree3`: `FastWindingNumber`,
//! `branch_fast_winding_num`, the per-box far-field cache (`build_fast_winding_cache`,
//! `can_use_fast_winding_cache`, `make_box_fast_winding_cache`) and `box_contains` from
//! `spatial/DMeshAABBTree.cs`; `FastTriWinding.ComputeCoeffs` / `EvaluateOrder2Approx` from
//! `math/FastWindingMath.cs`; `MathUtil.TriSolidAngle`; and `MeshTriInfoCache`
//! (`mesh/MeshCaches.cs`). `remove_occluded_triangles` is the caller.
//!
//! The cache is built when the [`FastWinding`] is made, which is when C# builds it (the first
//! query, which `RemoveOccludedTriangles` makes up front at the origin). C#'s per-box
//! `HashSet<int>` triangle sets are insertion-ordered lists here: every triangle sits in exactly
//! one leaf, so there are no duplicates, and a `HashSet<int>` that is only added to iterates in
//! insertion order - the order the coefficient sums run in.

use std::collections::HashMap;

use super::dmesh3::DMesh3;
use super::dmesh_aabb_tree3::{DMeshAABBTree3, BOX_EPS};
use super::vector3d::{add, dot, length, scale, sub, EPSILON};

/// C# `MathUtil.FourPI`.
const FOUR_PI: f64 = 4.0 * std::f64::consts::PI;
/// C# `FWNBeta`: "is 2.0 in paper".
const FWN_BETA: f64 = 2.0;
/// C# `WINDING_CACHE_THRESH` in `build_fast_winding_cache`: "set this to a larger number to
/// ignore caches if number of triangles is too small".
const WINDING_CACHE_THRESH: i32 = 1;

/// g3 `Matrix3d`, as its three rows.
type Matrix3d = [[f64; 3]; 3];

/// C# `new Matrix3d(ref u, ref v)`: the outer product, `Mij = u_i * v_j`.
fn outer(u: [f64; 3], v: [f64; 3]) -> Matrix3d {
    [
        [u[0] * v[0], u[0] * v[1], u[0] * v[2]],
        [u[1] * v[0], u[1] * v[1], u[1] * v[2]],
        [u[2] * v[0], u[2] * v[1], u[2] * v[2]],
    ]
}

/// C# `Matrix3d.InnerProduct`.
fn inner_product(m1: &Matrix3d, m2: &Matrix3d) -> f64 {
    dot(m1[0], m2[0]) + dot(m1[1], m2[1]) + dot(m1[2], m2[2])
}

/// C# `MathUtil.FastNormalArea`: the unit normal and the area (half the cross product's length,
/// zero when that length is at most `MathUtil.Epsilon`, as `Vector3d.Normalize` returns it).
fn fast_normal_area(v1: [f64; 3], v2: [f64; 3], v3: [f64; 3]) -> ([f64; 3], f64) {
    let edge1 = sub(v2, v1);
    let edge2 = sub(v3, v1);
    let v_cross = super::vector3d::cross(edge1, edge2);
    let len = length(v_cross);
    if len > EPSILON {
        let inv_length = 1.0 / len;
        (scale(v_cross, inv_length), 0.5 * len)
    } else {
        ([0.0; 3], 0.0)
    }
}

/// C# `MathUtil.TriSolidAngle(a, b, c, ref p)`: the solid angle triangle `abc` subtends at `p`.
pub(crate) fn tri_solid_angle(a: [f64; 3], b: [f64; 3], c: [f64; 3], p: [f64; 3]) -> f64 {
    let a = sub(a, p);
    let b = sub(b, p);
    let c = sub(c, p);
    let (la, lb, lc) = (length(a), length(b), length(c));
    let bottom = (la * lb * lc) + dot(a, b) * lc + dot(b, c) * la + dot(c, a) * lb;
    let top = a[0] * (b[1] * c[2] - c[1] * b[2]) - a[1] * (b[0] * c[2] - c[0] * b[2])
        + a[2] * (b[0] * c[1] - c[0] * b[1]);
    2.0 * top.atan2(bottom)
}

/// g3 `MeshTriInfoCache`: each triangle's centroid, unit normal and area (`DMesh3.GetTriInfo`).
/// Sized by the largest triangle id, so a mesh with removed triangles indexes it safely (C#
/// sizes it by the live count; see divergence 2140).
struct MeshTriInfoCache {
    centroids: Vec<[f64; 3]>,
    normals: Vec<[f64; 3]>,
    areas: Vec<f64>,
}

impl MeshTriInfoCache {
    fn new(mesh: &DMesh3) -> Self {
        let n = mesh.max_triangle_id().max(0) as usize;
        let mut cache = Self {
            centroids: vec![[0.0; 3]; n],
            normals: vec![[0.0; 3]; n],
            areas: vec![0.0; n],
        };
        for tid in mesh.triangle_indices() {
            let [v0, v1, v2] = mesh.get_tri_vertices(tid as usize);
            let t = tid as usize;
            cache.centroids[t] = scale(add(add(v0, v1), v2), 1.0 / 3.0);
            let (normal, area) = fast_normal_area(v0, v1, v2);
            cache.normals[t] = normal;
            cache.areas[t] = area;
        }
        cache
    }
}

/// g3 `FWNInfo`: one box's far-field expansion.
struct FwnInfo {
    center: [f64; 3],
    r: f64,
    order1_vec: [f64; 3],
    order2_mat: Matrix3d,
}

/// C# `FastTriWinding.ComputeCoeffs` with a triangle cache.
fn compute_coeffs(mesh: &DMesh3, triangles: &[i32], tri_cache: &MeshTriInfoCache) -> FwnInfo {
    let mut p = [0.0; 3];
    let mut order1 = [0.0; 3];
    let mut order2: Matrix3d = [[0.0; 3]; 3];
    let mut r: f64 = 0.0;

    // compute area-weighted centroid of triangles, we use this as the expansion point
    let mut sum_area = 0.0;
    for &tid in triangles {
        let area = tri_cache.areas[tid as usize];
        sum_area += area;
        p = add(p, scale(tri_cache.centroids[tid as usize], area));
    }
    // C# `p /= sum_area`: each component divided.
    p = [p[0] / sum_area, p[1] / sum_area, p[2] / sum_area];

    // compute first and second-order coefficients of FWN taylor expansion, as well as
    // 'radius' value r, which is max dist from any tri vertex to p
    for &tid in triangles {
        let [p0, p1, p2] = mesh.get_tri_vertices(tid as usize);
        let t = tid as usize;
        let (n, a, c) = (
            tri_cache.normals[t],
            tri_cache.areas[t],
            tri_cache.centroids[t],
        );

        order1 = add(order1, scale(n, a));

        let dcp = sub(c, p);
        let m = outer(dcp, n);
        for (row, m_row) in order2.iter_mut().zip(m) {
            *row = add(*row, scale(m_row, a));
        }

        // this is just for return value...
        let maxdist = dist_sqr(p0, p).max(dist_sqr(p1, p).max(dist_sqr(p2, p)));
        r = r.max(maxdist.sqrt());
    }

    FwnInfo {
        center: p,
        r,
        order1_vec: order1,
        order2_mat: order2,
    }
}

fn dist_sqr(a: [f64; 3], b: [f64; 3]) -> f64 {
    let d = sub(a, b);
    d[0] * d[0] + d[1] * d[1] + d[2] * d[2]
}

/// C# `FastTriWinding.EvaluateOrder2Approx`.
fn evaluate_order2_approx(info: &FwnInfo, q: [f64; 3]) -> f64 {
    let dpq = sub(info.center, q);
    let len = length(dpq);
    let len3 = len * len * len;
    let four_pi_len3 = 1.0 / (FOUR_PI * len3);

    let order1 = four_pi_len3 * dot(info.order1_vec, dpq);

    // second-order hessian \grad^2(G)
    let c = -3.0 / (FOUR_PI * len3 * len * len);

    let hessian: Matrix3d = [
        [
            four_pi_len3 + c * dpq[0] * dpq[0],
            c * dpq[0] * dpq[1],
            c * dpq[0] * dpq[2],
        ],
        [
            c * dpq[1] * dpq[0],
            four_pi_len3 + c * dpq[1] * dpq[1],
            c * dpq[1] * dpq[2],
        ],
        [
            c * dpq[2] * dpq[0],
            c * dpq[2] * dpq[1],
            four_pi_len3 + c * dpq[2] * dpq[2],
        ],
    ];

    let order2 = inner_product(&info.order2_mat, &hessian);

    order1 + order2
}

/// g3 `DMeshAABBTree3.FastWindingNumber` with its cache built: the tree and each internal box's
/// far-field expansion (order 2, C#'s default `FWNApproxOrder`).
pub(crate) struct FastWinding<'t, 'm> {
    tree: &'t DMeshAABBTree3<'m>,
    cache: HashMap<i32, FwnInfo>,
}

impl<'t, 'm> FastWinding<'t, 'm> {
    /// C# `build_fast_winding_cache()`.
    pub(crate) fn new(tree: &'t DMeshAABBTree3<'m>) -> Self {
        let tri_cache = MeshTriInfoCache::new(tree.mesh);
        let mut fw = Self {
            tree,
            cache: HashMap::new(),
        };
        let mut root_hash = None;
        fw.build_cache(
            tree.root_index,
            0,
            WINDING_CACHE_THRESH,
            &mut root_hash,
            &tri_cache,
        );
        fw
    }

    /// C# `FastWindingNumber(p)`: fast approximation of winding number using far-field
    /// approximations.
    pub(crate) fn fast_winding_number(&self, p: [f64; 3]) -> f64 {
        self.branch_fast_winding_num(self.tree.root_index, p)
    }

    /// C# `branch_fast_winding_num`: winding number contribution for all triangles below
    /// `i_box`.
    fn branch_fast_winding_num(&self, i_box: i32, p: [f64; 3]) -> f64 {
        let tree = self.tree;
        let mut branch_sum = 0.0;
        let idx = tree.box_to_index[i_box as usize];
        if idx < tree.triangles_end {
            // triange-list case, array is [N t1 t2 ... tN]
            let idx = idx as usize;
            let num_tris = tree.index_list[idx] as usize;
            for i in 1..=num_tris {
                let ti = tree.index_list[idx + i];
                let [a, b, c] = tree.mesh.get_tri_vertices(ti as usize);
                branch_sum += tri_solid_angle(a, b, c, p) / FOUR_PI;
            }
        } else {
            // internal node, either 1 or 2 child boxes
            let idx = idx as usize;
            let child1 = tree.index_list[idx];
            if child1 < 0 {
                branch_sum += self.child_contribution(-child1 - 1, p);
            } else {
                let child2 = tree.index_list[idx + 1] - 1;
                branch_sum += self.child_contribution(child1 - 1, p);
                branch_sum += self.child_contribution(child2, p);
            }
        }
        branch_sum
    }

    /// One child's share: its cached expansion when the point is outside the box and far
    /// enough away, otherwise the exact descent.
    fn child_contribution(&self, child: i32, p: [f64; 3]) -> f64 {
        let contained = self.box_contains(child, p);
        if !contained {
            if let Some(info) = self.usable_cache(child, p) {
                return evaluate_order2_approx(info, p);
            }
        }
        self.branch_fast_winding_num(child, p)
    }

    /// C# `can_use_fast_winding_cache`.
    fn usable_cache(&self, i_box: i32, q: [f64; 3]) -> Option<&FwnInfo> {
        let info = self.cache.get(&i_box)?;
        let dist_qp = length(sub(info.center, q));
        (dist_qp > FWN_BETA * info.r).then_some(info)
    }

    /// C# `box_contains`: the box widened by `box_eps` (in float).
    fn box_contains(&self, i_box: i32, p: [f64; 3]) -> bool {
        let c = self.tree.box_centers[i_box as usize];
        let e = self.tree.box_extents[i_box as usize];
        (0..3).all(|a| {
            let half = (e[a] + BOX_EPS) as f64;
            let center = c[a] as f64;
            center - half <= p[a] && center + half >= p[a]
        })
    }

    /// C# `build_fast_winding_cache(iBox, depth, tri_count_thresh, out tri_hash, triCache)`.
    fn build_cache(
        &mut self,
        i_box: i32,
        depth: i32,
        tri_count_thresh: i32,
        tri_hash: &mut Option<Vec<i32>>,
        tri_cache: &MeshTriInfoCache,
    ) -> i32 {
        *tri_hash = None;
        let tree = self.tree;
        let idx = tree.box_to_index[i_box as usize];
        if idx < tree.triangles_end {
            // triange-list case, array is [N t1 t2 ... tN]
            return tree.index_list[idx as usize];
        }

        // internal node, either 1 or 2 child boxes
        let idx = idx as usize;
        let child1 = tree.index_list[idx];
        if child1 < 0 {
            // if count in child is large enough, we already built a cache at lower node
            return self.build_cache(
                -child1 - 1,
                depth + 1,
                tri_count_thresh,
                tri_hash,
                tri_cache,
            );
        }

        let child1 = child1 - 1;
        let child2 = tree.index_list[idx + 1] - 1;

        // let each child build its own cache if it wants. If so, it will return the
        // list of its child tris
        let mut child2_hash = None;
        let num_tris_1 = self.build_cache(child1, depth + 1, tri_count_thresh, tri_hash, tri_cache);
        let num_tris_2 = self.build_cache(
            child2,
            depth + 1,
            tri_count_thresh,
            &mut child2_hash,
            tri_cache,
        );
        let build_cache = num_tris_1 + num_tris_2 > tri_count_thresh;

        if depth == 0 {
            return num_tris_1 + num_tris_2; // cannot build cache at level 0...
        }

        // collect up the triangles we need. there are various cases depending on what children
        // already did
        if tri_hash.is_some() || child2_hash.is_some() || build_cache {
            if tri_hash.is_none() && child2_hash.is_some() {
                let mut hash = child2_hash.take().unwrap_or_default();
                self.collect_triangles(child1, &mut hash);
                *tri_hash = Some(hash);
            } else {
                let hash = tri_hash.get_or_insert_with(|| {
                    let mut hash = Vec::new();
                    self.collect_triangles(child1, &mut hash);
                    hash
                });
                match child2_hash {
                    None => self.collect_triangles(child2, hash),
                    Some(child2_hash) => hash.extend(child2_hash),
                }
            }
        }
        if build_cache {
            if let Some(hash) = tri_hash.as_deref() {
                let info = compute_coeffs(tree.mesh, hash, tri_cache);
                self.cache.insert(i_box, info);
            }
        }

        num_tris_1 + num_tris_2
    }

    /// C# `collect_triangles`.
    fn collect_triangles(&self, i_box: i32, triangles: &mut Vec<i32>) {
        let tree = self.tree;
        let idx = tree.box_to_index[i_box as usize];
        if idx < tree.triangles_end {
            let idx = idx as usize;
            let num_tris = tree.index_list[idx] as usize;
            triangles.extend_from_slice(&tree.index_list[idx + 1..=idx + num_tris]);
        } else {
            let idx = idx as usize;
            let child1 = tree.index_list[idx];
            if child1 < 0 {
                self.collect_triangles(-child1 - 1, triangles);
            } else {
                self.collect_triangles(child1 - 1, triangles);
                self.collect_triangles(tree.index_list[idx + 1] - 1, triangles);
            }
        }
    }
}
