//! The part of geometry3Sharp's `DMeshAABBTree3` (`spatial/DMeshAABBTree.cs`) the signed
//! distance grid reaches: the default top-down midpoint build and `FindNearestTriangle`.
//!
//! The layout is g3's: float box centres and extents per box, `box_to_index` into one
//! `index_list` whose leading part holds the leaf triangle lists (`[N t1 .. tN]`) and whose tail
//! holds the internal nodes (one child as `-(child + 1)`, two as `child + 1` pairs). The query
//! reads the boxes padded by g3's `box_eps`, so it prunes exactly where g3 does.

use super::dmesh3::{tri_distance_sqr, AxisAlignedBox3d, DMesh3};

/// g3 `MathUtil.ZeroTolerance`.
const ZERO_TOLERANCE: f64 = 1e-08;
/// g3 `MathUtil.Epsilonf`.
const EPSILON_F: f32 = 1.192_092_9e-7;
/// g3 `box_eps`: `50.0f * MathUtil.Epsilonf`.
const BOX_EPS: f32 = 50.0 * EPSILON_F;
/// g3 `TopDownLeafMaxTriCount` (a leaf holds fewer than this many triangles).
const TOP_DOWN_LEAF_MAX_TRI_COUNT: usize = 4;

/// g3 `AxisAlignedBox3f`, the subset the build needs.
#[derive(Debug, Clone, Copy)]
struct Box3f {
    min: [f32; 3],
    max: [f32; 3],
}

impl Box3f {
    /// C# `AxisAlignedBox3f.Empty`.
    const EMPTY: Box3f = Box3f {
        min: [f32::MAX; 3],
        max: [f32::MIN; 3],
    };

    /// C# `Contain(AxisAlignedBox3f)` / `Contain(AxisAlignedBox3d)` (doubles narrowed first).
    fn contain(&mut self, other: &Box3f) {
        for a in 0..3 {
            self.min[a] = self.min[a].min(other.min[a]);
            self.max[a] = self.max[a].max(other.max[a]);
        }
    }

    fn center(&self) -> [f32; 3] {
        std::array::from_fn(|a| (0.5 * (self.min[a] + self.max[a]) as f64) as f32)
    }

    fn extents(&self) -> [f32; 3] {
        std::array::from_fn(|a| ((self.max[a] - self.min[a]) as f64 * 0.5) as f32)
    }
}

fn narrow(b: &AxisAlignedBox3d) -> Box3f {
    Box3f {
        min: b.min.map(|v| v as f32),
        max: b.max.map(|v| v as f32),
    }
}

/// g3 `boxes_set`.
#[derive(Default)]
struct BoxesSet {
    box_to_index: Vec<i32>,
    box_centers: Vec<[f32; 3]>,
    box_extents: Vec<[f32; 3]>,
    index_list: Vec<i32>,
}

/// g3 `DMeshAABBTree3`, built with `TopDownMidpoint` (C# `new DMeshAABBTree3(mesh, autoBuild: true)`).
pub(crate) struct DMeshAABBTree3<'a> {
    mesh: &'a DMesh3,
    box_to_index: Vec<i32>,
    box_centers: Vec<[f32; 3]>,
    box_extents: Vec<[f32; 3]>,
    index_list: Vec<i32>,
    triangles_end: i32,
    root_index: i32,
}

impl<'a> DMeshAABBTree3<'a> {
    /// C# `new DMeshAABBTree3(mesh, autoBuild: true)`: `build_top_down(false)`.
    pub(crate) fn new(mesh: &'a DMesh3) -> Self {
        let count = mesh.triangle_count();
        // Invalid (non-finite) triangles are skipped but the split still runs over `count`
        // entries, as in g3; the unused tail stays zero.
        let mut triangles = vec![0i32; count];
        let mut centers = vec![[0.0f64; 3]; count];
        let mut i = 0;
        for ti in mesh.triangle_indices() {
            let ti = ti as usize;
            let centroid = mesh.get_tri_centroid(ti);
            let d2 =
                centroid[0] * centroid[0] + centroid[1] * centroid[1] + centroid[2] * centroid[2];
            if d2.is_finite() {
                triangles[i] = ti as i32;
                centers[i] = centroid;
                i += 1;
            }
        }

        let mut tris = BoxesSet::default();
        let mut nodes = BoxesSet::default();
        let mut builder = Builder {
            mesh,
            triangles: &mut triangles,
            centers: &mut centers,
            tris: &mut tris,
            nodes: &mut nodes,
        };
        let (rootnode, _) = builder.split_tri_set_midpoint(0, count, 0);

        let triangles_end = tris.index_list.len() as i32;
        let index_shift = triangles_end;
        let box_shift = tris.box_to_index.len() as i32;
        let mut box_to_index = tris.box_to_index;
        let mut box_centers = tris.box_centers;
        let mut box_extents = tris.box_extents;
        let mut index_list = tris.index_list;
        for b in 0..nodes.box_to_index.len() {
            box_centers.push(nodes.box_centers[b]);
            box_extents.push(nodes.box_extents[b]);
            box_to_index.push(index_shift + nodes.box_to_index[b]);
        }
        for &entry in &nodes.index_list {
            let child_box = if entry < 0 {
                -entry - 1
            } else {
                entry + box_shift
            };
            index_list.push(child_box + 1);
        }

        DMeshAABBTree3 {
            mesh,
            box_to_index,
            box_centers,
            box_extents,
            index_list,
            triangles_end,
            root_index: rootnode + box_shift,
        }
    }

    /// C# `FindNearestTriangle(p, fMaxDist)`: the nearest triangle strictly within `max_dist`
    /// (`f64::MAX` for no limit), or -1.
    pub(crate) fn find_nearest_triangle(&self, p: [f64; 3], max_dist: f64) -> i32 {
        let mut nearest_sqr = if max_dist < f64::MAX {
            max_dist * max_dist
        } else {
            f64::MAX
        };
        let mut tid = -1;
        self.find_nearest_tri(self.root_index, p, &mut nearest_sqr, &mut tid);
        tid
    }

    fn find_nearest_tri(&self, i_box: i32, p: [f64; 3], nearest_sqr: &mut f64, tid: &mut i32) {
        let idx = self.box_to_index[i_box as usize];
        if idx < self.triangles_end {
            let idx = idx as usize;
            let num_tris = self.index_list[idx] as usize;
            for i in 1..=num_tris {
                let ti = self.index_list[idx + i];
                let dist_sqr = tri_distance_sqr(&self.mesh.get_tri_vertices(ti as usize), p);
                if dist_sqr < *nearest_sqr {
                    *nearest_sqr = dist_sqr;
                    *tid = ti;
                }
            }
        } else {
            let idx = idx as usize;
            let child1 = self.index_list[idx];
            if child1 < 0 {
                let child1 = -child1 - 1;
                if self.box_distance_sqr(child1, p) <= *nearest_sqr {
                    self.find_nearest_tri(child1, p, nearest_sqr, tid);
                }
            } else {
                let child1 = child1 - 1;
                let child2 = self.index_list[idx + 1] - 1;
                let dist1 = self.box_distance_sqr(child1, p);
                let dist2 = self.box_distance_sqr(child2, p);
                let (first, first_dist, second, second_dist) = if dist1 < dist2 {
                    (child1, dist1, child2, dist2)
                } else {
                    (child2, dist2, child1, dist1)
                };
                if first_dist < *nearest_sqr {
                    self.find_nearest_tri(first, p, nearest_sqr, tid);
                    if second_dist < *nearest_sqr {
                        self.find_nearest_tri(second, p, nearest_sqr, tid);
                    }
                }
            }
        }
    }

    /// C# `box_distance_sqr`: the box widened by `box_eps` in float, then measured in double.
    fn box_distance_sqr(&self, i_box: i32, p: [f64; 3]) -> f64 {
        let c = self.box_centers[i_box as usize];
        let e = self.box_extents[i_box as usize];
        let half: [f64; 3] = std::array::from_fn(|a| (e[a] + BOX_EPS) as f64);
        let bx = AxisAlignedBox3d {
            min: std::array::from_fn(|a| c[a] as f64 - half[a]),
            max: std::array::from_fn(|a| c[a] as f64 + half[a]),
        };
        bx.distance_squared(p)
    }
}

struct Builder<'b> {
    mesh: &'b DMesh3,
    triangles: &'b mut [i32],
    centers: &'b mut [[f64; 3]],
    tris: &'b mut BoxesSet,
    nodes: &'b mut BoxesSet,
}

impl Builder<'_> {
    /// C# `split_tri_set_midpoint`: returns the box id (a leaf as `-(id + 1)`) and its box.
    fn split_tri_set_midpoint(&mut self, start: usize, count: usize, depth: usize) -> (i32, Box3f) {
        let mut bx = Box3f::EMPTY;
        if count < TOP_DOWN_LEAF_MAX_TRI_COUNT {
            let i_box = self.tris.box_to_index.len() as i32;
            self.tris
                .box_to_index
                .push(self.tris.index_list.len() as i32);
            self.tris.index_list.push(count as i32);
            for i in 0..count {
                let t = self.triangles[start + i];
                self.tris.index_list.push(t);
                bx.contain(&narrow(&self.mesh.get_tri_bounds(t as usize)));
            }
            self.tris.box_centers.push(bx.center());
            self.tris.box_extents.push(bx.extents());
            return (-(i_box + 1), bx);
        }

        let axis = depth % 3;
        let (mut lo, mut hi) = (f64::MAX, -f64::MAX);
        for i in 0..count {
            let d = self.centers[start + i][axis];
            if d < lo {
                lo = d;
            }
            if d > hi {
                hi = d;
            }
        }
        let midpoint = (hi + lo) * 0.5;

        let (n0, n1);
        if (lo - hi).abs() > ZERO_TOLERANCE {
            // Walk in from each side, swapping out-of-order pairs, so centres <= midpoint come
            // first.
            let mut l = 0usize;
            let mut r = count - 1;
            while l < r {
                while self.centers[start + l][axis] <= midpoint {
                    l += 1;
                }
                while self.centers[start + r][axis] > midpoint {
                    r -= 1;
                }
                if l >= r {
                    break;
                }
                self.centers.swap(start + l, start + r);
                self.triangles.swap(start + l, start + r);
            }
            n0 = l;
            n1 = count - n0;
        } else {
            n0 = count / 2;
            n1 = count - n0;
        }

        let (child0, box0) = self.split_tri_set_midpoint(start, n0, depth + 1);
        let (child1, box1) = self.split_tri_set_midpoint(start + n0, n1, depth + 1);
        bx = box0;
        bx.contain(&box1);

        let i_box = self.nodes.box_to_index.len() as i32;
        self.nodes
            .box_to_index
            .push(self.nodes.index_list.len() as i32);
        self.nodes.index_list.push(child0);
        self.nodes.index_list.push(child1);
        self.nodes.box_centers.push(bx.center());
        self.nodes.box_extents.push(bx.extents());
        (i_box, bx)
    }
}
