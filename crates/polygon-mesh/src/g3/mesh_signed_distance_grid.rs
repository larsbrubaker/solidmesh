//! The narrow-band distance grid (derived from geometry3Sharp's mesh signed distance grid)
//! that `MeshSdfGrid` uses: the grid layout padded around the mesh, the spatial flood fill
//! of unsigned distances near the surface, and Batty's robust `point_in_triangle_2d`, which
//! `MeshSdfGrid` signs with.
//!
//! Every voxel's value is the distance to its nearest triangle, and a voxel joins the next
//! pass from its own distance alone, so the passes visit the same voxels with the same values
//! in any order; they run in order on one thread. The work count feeds a progress callback,
//! and cancellation is polled every 64th voxel.

use std::sync::Arc;

use super::dmesh3::{tri_distance_sqr, DMesh3};
use super::dmesh_aabb_tree3::DMeshAABBTree3;
use crate::sdf::DenseGrid3f;

/// The square root of two.
const SQRT_TWO: f64 = std::f64::consts::SQRT_2;

/// The 26 neighbour offsets of a grid cell: face, edge, then corner neighbours.
const GRID_OFFSETS_26: [[i32; 3]; 26] = [
    [0, 0, -1],
    [0, 0, 1],
    [-1, 0, 0],
    [1, 0, 0],
    [0, -1, 0],
    [0, 1, 0],
    [1, 1, 0],
    [-1, 1, 0],
    [0, 1, 1],
    [0, 1, -1],
    [1, 0, 1],
    [-1, 0, 1],
    [1, 0, -1],
    [-1, 0, -1],
    [1, -1, 0],
    [-1, -1, 0],
    [0, -1, 1],
    [0, -1, -1],
    [1, 1, 1],
    [-1, 1, 1],
    [1, 1, -1],
    [-1, 1, -1],
    [1, -1, 1],
    [-1, -1, 1],
    [1, -1, -1],
    [-1, -1, -1],
];

/// The flood's running work count, reported once per voxel.
pub type FloodProgress = Arc<dyn Fn(i64) + Send + Sync>;

/// A narrow-band unsigned distance grid, padded around the mesh and filled by a spatial flood.
pub(crate) struct NarrowBandDistanceGrid<'a> {
    pub(crate) mesh: &'a DMesh3,
    pub(crate) spatial: &'a DMeshAABBTree3<'a>,
    /// The edge length of a grid cell (stored as float).
    pub(crate) cell_size: f32,
    /// How far from the surface the flood fills distances.
    pub(crate) narrow_band_max_distance: f64,
    /// How far the grid extends past the mesh bounds.
    pub(crate) pad_width: f64,
    /// Polled during the flood; true stops it.
    pub(crate) cancel: &'a dyn Fn() -> bool,
    /// Receives the running work count.
    pub(crate) flood_progress: Option<FloodProgress>,
    /// The number of voxels the flood has processed.
    pub(crate) flood_voxels_processed: i64,
    /// The grid's minimum corner, set by [`NarrowBandDistanceGrid::compute`].
    pub(crate) grid_origin: [f32; 3],
}

impl NarrowBandDistanceGrid<'_> {
    /// Lays out the grid around the mesh bounds padded by `pad_width` and
    /// floods the band. Returns the grid, possibly unfinished when cancelled; the caller
    /// checks its own cancel token before using it.
    pub(crate) fn compute(&mut self) -> DenseGrid3f {
        let bounds = self.mesh.cached_bounds();
        let pad = self.pad_width as f32;
        let min = bounds.min.map(|v| v as f32);
        let max_b = bounds.max.map(|v| v as f32);
        // The float minimum, minus the pad, minus a zero bounds expansion.
        let origin: [f32; 3] = std::array::from_fn(|a| min[a] - pad - 0.0);
        let max: [f32; 3] = std::array::from_fn(|a| max_b[a] + pad + 0.0);
        let ni = ((max[0] - origin[0]) / self.cell_size) as i32 + 1;
        let nj = ((max[1] - origin[1]) / self.cell_size) as i32 + 1;
        let nk = ((max[2] - origin[2]) / self.cell_size) as i32 + 1;
        self.grid_origin = origin;
        self.flood_fill(ni.max(0) as usize, nj.max(0) as usize, nk.max(0) as usize)
    }

    /// The centre of cell `idx` in world space.
    fn cell_center(&self, idx: [i32; 3]) -> [f64; 3] {
        std::array::from_fn(|a| (idx[a] as f32 * self.cell_size + self.grid_origin[a]) as f64)
    }

    /// The exact distance from `p` to triangle `tid`.
    fn distance(&self, p: [f64; 3], tid: i32) -> f64 {
        tri_distance_sqr(&self.mesh.get_tri_vertices(tid as usize), p).sqrt()
    }

    /// The flood fill: unsigned distances for the cells near the surface.
    fn flood_fill(&mut self, ni: usize, nj: usize, nk: usize) -> DenseGrid3f {
        let dx = self.cell_size;
        // The upper bound: (ni + nj + nk) cells, in float.
        let upper_bound = (ni + nj + nk) as i32 as f32 * dx;
        let mut distances = DenseGrid3f::new(ni, nj, nk, upper_bound);
        let size = distances.size();
        let (ox, oy, oz) = (
            self.grid_origin[0] as f64,
            self.grid_origin[1] as f64,
            self.grid_origin[2] as f64,
        );
        let invdx = 1.0 / dx as f64;
        let to_linear =
            |idx: [i32; 3]| idx[0] as usize + ni * (idx[1] as usize + nj * idx[2] as usize);

        let mut queue: Vec<usize> = Vec::new();
        let mut done = vec![false; size];

        // Seed every vertex's voxel with its exact distance.
        let mut abort = false;
        for vid in 0..self.mesh.vertex_count() {
            if vid % 100 == 0 {
                abort = (self.cancel)();
            }
            if abort {
                break;
            }
            let v = self.mesh.get_vertex(vid);
            let fi = (v[0] - ox) * invdx;
            let fj = (v[1] - oy) * invdx;
            let fk = (v[2] - oz) * invdx;
            let idx = [
                (fi as i32).clamp(0, ni as i32 - 1),
                (fj as i32).clamp(0, nj as i32 - 1),
                (fk as i32).clamp(0, nk as i32 - 1),
            ];
            let linear = to_linear(idx);
            if distances.buffer[linear] < upper_bound {
                continue;
            }
            let p = self.cell_center(idx);
            let near_tid = self.spatial.find_nearest_triangle(p, f64::MAX);
            if near_tid < 0 {
                // No triangle at all (an empty mesh): leave the cell at its upper bound.
                continue;
            }
            distances.buffer[linear] = self.distance(p, near_tid) as f32;
            queue.push(linear);
            done[linear] = true;
        }

        if (self.cancel)() {
            return distances;
        }

        let max_dist = self.narrow_band_max_distance;
        let max_query_dist = max_dist + (2.0 * dx as f64 * SQRT_TWO);
        let mut next_queue: Vec<usize> = Vec::new();
        while !queue.is_empty() {
            if abort || (self.cancel)() {
                return distances;
            }
            next_queue.clear();
            for &cur in &queue {
                if abort {
                    break;
                }
                self.flood_voxels_processed += 1;
                let processed = self.flood_voxels_processed;
                if let Some(progress) = &self.flood_progress {
                    progress(processed);
                }
                if processed % 64 == 0 && (self.cancel)() {
                    abort = true;
                    break;
                }

                let cur_idx = [
                    (cur % ni) as i32,
                    ((cur / ni) % nj) as i32,
                    (cur / (ni * nj)) as i32,
                ];
                for offset in GRID_OFFSETS_26 {
                    let nbr = [
                        cur_idx[0] + offset[0],
                        cur_idx[1] + offset[1],
                        cur_idx[2] + offset[2],
                    ];
                    if nbr[0] < 0
                        || nbr[1] < 0
                        || nbr[2] < 0
                        || nbr[0] >= ni as i32
                        || nbr[1] >= nj as i32
                        || nbr[2] >= nk as i32
                    {
                        continue;
                    }
                    let nbr_linear = to_linear(nbr);
                    if done[nbr_linear] {
                        continue;
                    }
                    let p = self.cell_center(nbr);
                    let near_tid = self.spatial.find_nearest_triangle(p, max_query_dist);
                    if near_tid == -1 {
                        done[nbr_linear] = true;
                        continue;
                    }
                    let dist = self.distance(p, near_tid);
                    distances.buffer[nbr_linear] = dist as f32;
                    done[nbr_linear] = true;
                    if dist < max_dist {
                        next_queue.push(nbr_linear);
                    }
                }
            }
            std::mem::swap(&mut queue, &mut next_queue);
        }

        distances
    }
}

/// Twice the signed area of (0,0)-(x1,y1)-(x2,y2), and its sign with a
/// simulation-of-simplicity tie-break (0 only when the two points coincide).
fn orientation(x1: f64, y1: f64, x2: f64, y2: f64) -> (i32, f64) {
    let twice_signed_area = y1 * x2 - x1 * y2;
    let sign = if twice_signed_area > 0.0 {
        1
    } else if twice_signed_area < 0.0 {
        -1
    } else if y2 > y1 {
        1
    } else if y2 < y1 {
        -1
    } else if x1 > x2 {
        1
    } else if x1 < x2 {
        -1
    } else {
        0
    };
    (sign, twice_signed_area)
}

/// Whether (x0, y0) is in the triangle, robustly (a point on a shared
/// edge or vertex is in exactly one of the triangles sharing it), with its barycentric
/// coordinates.
#[allow(clippy::too_many_arguments)]
pub(crate) fn point_in_triangle_2d(
    x0: f64,
    y0: f64,
    x1: f64,
    y1: f64,
    x2: f64,
    y2: f64,
    x3: f64,
    y3: f64,
) -> Option<(f64, f64, f64)> {
    let (x1, x2, x3) = (x1 - x0, x2 - x0, x3 - x0);
    let (y1, y2, y3) = (y1 - y0, y2 - y0, y3 - y0);
    let (signa, a) = orientation(x2, y2, x3, y3);
    if signa == 0 {
        return None;
    }
    let (signb, b) = orientation(x3, y3, x1, y1);
    if signb != signa {
        return None;
    }
    let (signc, c) = orientation(x1, y1, x2, y2);
    if signc != signa {
        return None;
    }
    let sum = a + b + c;
    // With matching nonzero SOS signs a, b and c cannot all be zero.
    assert!(
        sum != 0.0,
        "MakeNarrowBandLevelSet.point_in_triangle_2d: badness!"
    );
    Some((a / sum, b / sum, c / sum))
}
