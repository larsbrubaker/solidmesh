//! geometry3Sharp `DMeshAABBTree3.FindNearestTriangles` (`spatial/DMeshAABBTree.cs`): the
//! nearest pair of triangles between two trees, descending both together and pruning box pairs
//! farther apart than the best pair so far. Only the form `mesh_distance` calls is ported: no
//! `TransformF` (both meshes already in one space) and no `TriangleFilterF`. The tree and its
//! layout are `dmesh_aabb_tree3.rs`; leaf pairs are measured with `dist_triangle3_triangle3`.

use super::dist_triangle3_triangle3::dist_triangle3_triangle3;
use super::dmesh3::AxisAlignedBox3d;
use super::dmesh_aabb_tree3::{DMeshAABBTree3, BOX_EPS};

impl DMeshAABBTree3<'_> {
    /// C# `FindNearestTriangles(otherTree, null, out distance)`: the nearest pair (this tree's
    /// triangle, the other's) and their distance, or `None` (C# `Index2i.Max` and
    /// `double.MaxValue`) when either tree is empty.
    pub(crate) fn find_nearest_triangles(
        &self,
        other: &DMeshAABBTree3<'_>,
    ) -> Option<((i32, i32), f64)> {
        let mut nearest_sqr = f64::MAX;
        let mut nearest_pair = None;
        self.find_nearest_triangles_in(
            self.root_index,
            other,
            other.root_index,
            0,
            &mut nearest_sqr,
            &mut nearest_pair,
        );
        nearest_pair.map(|pair| (pair, nearest_sqr.sqrt()))
    }

    fn find_nearest_triangles_in(
        &self,
        i_box: i32,
        other: &DMeshAABBTree3<'_>,
        o_box: i32,
        depth: i32,
        nearest_sqr: &mut f64,
        nearest_pair: &mut Option<(i32, i32)>,
    ) {
        if self.box_to_index.is_empty() || other.box_to_index.is_empty() {
            return;
        }
        let idx = self.box_to_index[i_box as usize];
        let odx = other.box_to_index[o_box as usize];

        if idx < self.triangles_end && odx < other.triangles_end {
            // ok we are at triangles for both trees, do triangle-level testing
            let num_tris = self.index_list[idx as usize];
            let onum_tris = other.index_list[odx as usize];

            // outer iteration is "other" tris that need to be transformed (more expensive)
            for j in 1..=onum_tris {
                let tj = other.index_list[(odx + j) as usize];
                let otri = other.mesh.get_tri_vertices(tj as usize);

                // inner iteration over "our" triangles
                for i in 1..=num_tris {
                    let ti = self.index_list[(idx + i) as usize];
                    let tri = self.mesh.get_tri_vertices(ti as usize);
                    let dist_sqr = dist_triangle3_triangle3(&otri, &tri).distance_squared;
                    if dist_sqr < *nearest_sqr {
                        *nearest_sqr = dist_sqr;
                        *nearest_pair = Some((ti, tj));
                    }
                }
            }
            return;
        }

        // we either descend "our" tree or the other tree
        //   - if we have hit triangles on "our" tree, we have to descend other
        //   - if we hit triangles on "other", we have to descend ours
        //   - otherwise, we alternate at each depth. This produces wider
        //     branching but is significantly faster (~10x) for both hits and misses
        let mut descend_other = idx < self.triangles_end || depth % 2 == 0;
        if descend_other && odx < other.triangles_end {
            descend_other = false; // can't
        }

        if descend_other {
            // ok we reached triangles on our side but we need to still reach triangles on
            // the other side, so we descend "their" children
            let bounds = self.get_boxd(i_box);
            let o_child1 = other.index_list[odx as usize];
            if o_child1 < 0 {
                // 1 child, descend if nearer than cur min-dist
                let o_child1 = -o_child1 - 1;
                if box_box_distance_sqr(&other.get_boxd(o_child1), &bounds) < *nearest_sqr {
                    self.find_nearest_triangles_in(
                        i_box,
                        other,
                        o_child1,
                        depth + 1,
                        nearest_sqr,
                        nearest_pair,
                    );
                }
            } else {
                // 2 children
                let o_child1 = o_child1 - 1;
                let o_child2 = other.index_list[odx as usize + 1] - 1;
                // descend closer box first
                let d1 = box_box_distance_sqr(&other.get_boxd(o_child1), &bounds);
                let d2 = box_box_distance_sqr(&other.get_boxd(o_child2), &bounds);
                let order = if d2 < d1 {
                    [(o_child2, d2), (o_child1, d1)]
                } else {
                    [(o_child1, d1), (o_child2, d2)]
                };
                for (child, d) in order {
                    if d < *nearest_sqr {
                        self.find_nearest_triangles_in(
                            i_box,
                            other,
                            child,
                            depth + 1,
                            nearest_sqr,
                            nearest_pair,
                        );
                    }
                }
            }
        } else {
            // descend our tree nodes if they intersect w/ current bounds of other tree
            let o_bounds = other.get_boxd(o_box);
            let i_child1 = self.index_list[idx as usize];
            if i_child1 < 0 {
                // 1 child, descend if nearer than cur min-dist
                let i_child1 = -i_child1 - 1;
                if box_box_distance_sqr(&self.get_boxd(i_child1), &o_bounds) < *nearest_sqr {
                    self.find_nearest_triangles_in(
                        i_child1,
                        other,
                        o_box,
                        depth + 1,
                        nearest_sqr,
                        nearest_pair,
                    );
                }
            } else {
                // 2 children
                let i_child1 = i_child1 - 1;
                let i_child2 = self.index_list[idx as usize + 1] - 1;
                // descend closer box first
                let d1 = box_box_distance_sqr(&self.get_boxd(i_child1), &o_bounds);
                let d2 = box_box_distance_sqr(&self.get_boxd(i_child2), &o_bounds);
                let order = if d2 < d1 {
                    [(i_child2, d2), (i_child1, d1)]
                } else {
                    [(i_child1, d1), (i_child2, d2)]
                };
                for (child, d) in order {
                    if d < *nearest_sqr {
                        self.find_nearest_triangles_in(
                            child,
                            other,
                            o_box,
                            depth + 1,
                            nearest_sqr,
                            nearest_pair,
                        );
                    }
                }
            }
        }
    }

    /// C# `get_boxd(iBox)`: the box widened by `box_eps` in float, then held in double.
    fn get_boxd(&self, i_box: i32) -> AxisAlignedBox3d {
        let c = self.box_centers[i_box as usize];
        let e = self.box_extents[i_box as usize];
        let half: [f64; 3] = std::array::from_fn(|a| (e[a] + BOX_EPS) as f64);
        AxisAlignedBox3d {
            min: std::array::from_fn(|a| c[a] as f64 - half[a]),
            max: std::array::from_fn(|a| c[a] as f64 + half[a]),
        }
    }
}

/// C# `AxisAlignedBox3d.DistanceSquared(ref box2)`: `lensqr(max(0, |c1 - c2| - (e1 + e2)))`.
fn box_box_distance_sqr(a: &AxisAlignedBox3d, b: &AxisAlignedBox3d) -> f64 {
    let delta = |axis: usize| {
        let d = ((b.min[axis] + b.max[axis]) - (a.min[axis] + a.max[axis])).abs()
            - ((a.max[axis] - a.min[axis]) + (b.max[axis] - b.min[axis]));
        if d < 0.0 {
            0.0
        } else {
            d
        }
    };
    let (dx, dy, dz) = (delta(0), delta(1), delta(2));
    0.25 * (dx * dx + dy * dy + dz * dz)
}
