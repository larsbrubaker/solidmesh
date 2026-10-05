//! geometry3Sharp's `MeshSignedDistanceGrid` (`spatial/MeshSignedDistanceGrid.cs`, Boost Software
//! License 1.0, after Christopher Batty's public-domain SDFGen) in its default configuration, the
//! one agg-sharp's implicit booleans (`BooleanProcessing.GetImplicitFunction`) use:
//! `ComputeMode = NarrowBandOnly`, `ComputeSigns = true`, `InsideMode = ParityCount`,
//! `ExactBandWidth = 1`, no spatial structure (`make_level_set3_parallel`, then
//! `compute_intersections` and `compute_signs`). The flood-fill mode `MeshSdfGrid` uses lives in
//! `mesh_signed_distance_grid`, whose robust `point_in_triangle_2d` this shares.
//!
//! C# spreads the triangles over `gParallel.ForEach` with spin-locked grid writes. Each cell
//! ends at the minimum distance over the triangles whose band covers it and each crossing count
//! is a sum, so the order the triangles land in cannot change the grid: they run in order on
//! one thread here.

use super::dmesh3::DMesh3;
use super::mesh_signed_distance_grid::point_in_triangle_2d;
use crate::sdf::DenseGrid3f;

/// The signed narrow-band grid (C# `MeshSignedDistanceGrid` after `Compute()`): distances are
/// exact within a cell of the surface, `±upper_bound` elsewhere, negative inside.
pub(crate) struct SignedDistanceGrid {
    /// C# `Grid`.
    pub(crate) grid: DenseGrid3f,
    /// C# `GridOrigin`.
    pub(crate) grid_origin: [f32; 3],
    /// C# `CellSize`.
    pub(crate) cell_size: f32,
}

/// C# `ExactBandWidth`'s default.
const EXACT_BAND: i32 = 1;

impl SignedDistanceGrid {
    /// C# `new MeshSignedDistanceGrid(mesh, cellSize)` then `Compute()`.
    pub(crate) fn compute(mesh: &DMesh3, cell_size: f64) -> Self {
        let cell_size = cell_size as f32;
        let bounds = mesh.cached_bounds();
        // C# `fBufferWidth = 2 * ExactBandWidth * CellSize` (float); `ExpandBounds` is zero.
        let buffer_width = (2 * EXACT_BAND) as f32 * cell_size;
        let origin: [f32; 3] = std::array::from_fn(|a| bounds.min[a] as f32 - buffer_width);
        let max: [f32; 3] = std::array::from_fn(|a| bounds.max[a] as f32 + buffer_width);
        let ni = ((max[0] - origin[0]) / cell_size) as i32 + 1;
        let nj = ((max[1] - origin[1]) / cell_size) as i32 + 1;
        let nk = ((max[2] - origin[2]) / cell_size) as i32 + 1;
        let grid = make_level_set3(
            mesh,
            origin,
            cell_size,
            ni.max(0) as usize,
            nj.max(0) as usize,
            nk.max(0) as usize,
        );
        SignedDistanceGrid {
            grid,
            grid_origin: origin,
            cell_size,
        }
    }
}

/// C# `make_level_set3_parallel` with `ComputeSigns = true` in `NarrowBandOnly` mode.
fn make_level_set3(
    mesh: &DMesh3,
    origin: [f32; 3],
    dx: f32,
    ni: usize,
    nj: usize,
    nk: usize,
) -> DenseGrid3f {
    // C# `upper_bound`: `(float)((ni + nj + nk) * CellSize)`.
    let upper_bound = (ni + nj + nk) as i32 as f32 * dx;
    let mut distances = DenseGrid3f::new(ni, nj, nk, upper_bound);
    let mut intersection_count = vec![0i32; ni * nj * nk];
    if ni == 0 || nj == 0 || nk == 0 {
        return distances;
    }
    let (ox, oy, oz) = (origin[0] as f64, origin[1] as f64, origin[2] as f64);
    let invdx = 1.0 / dx as f64;
    let (ni_i, nj_i, nk_i) = (ni as i32, nj as i32, nk as i32);

    for tid in 0..mesh.triangle_count() {
        let [xp, xq, xr] = mesh.get_tri_vertices(tid);
        let fi = |p: [f64; 3]| (p[0] - ox) * invdx;
        let fj = |p: [f64; 3]| (p[1] - oy) * invdx;
        let fk = |p: [f64; 3]| (p[2] - oz) * invdx;
        let (fip, fiq, fir) = (fi(xp), fi(xq), fi(xr));
        let (fjp, fjq, fjr) = (fj(xp), fj(xq), fj(xr));
        let (fkp, fkq, fkr) = (fk(xp), fk(xq), fk(xr));

        // Clamped integer bounding box of the triangle plus the exact band.
        let lo =
            |a: f64, b: f64, c: f64, n: i32| ((min3(a, b, c) as i32) - EXACT_BAND).clamp(0, n - 1);
        let hi = |a: f64, b: f64, c: f64, n: i32| {
            ((max3(a, b, c) as i32) + EXACT_BAND + 1).clamp(0, n - 1)
        };
        let (i0, i1) = (lo(fip, fiq, fir, ni_i), hi(fip, fiq, fir, ni_i));
        let (j0, j1) = (lo(fjp, fjq, fjr, nj_i), hi(fjp, fjq, fjr, nj_i));
        let (k0, k1) = (lo(fkp, fkq, fkr, nk_i), hi(fkp, fkq, fkr, nk_i));

        for k in k0..=k1 {
            for j in j0..=j1 {
                for i in i0..=i1 {
                    // C# builds the cell centre in float, then widens it.
                    let gx = [
                        (i as f32 * dx + origin[0]) as f64,
                        (j as f32 * dx + origin[1]) as f64,
                        (k as f32 * dx + origin[2]) as f64,
                    ];
                    let d = point_triangle_distance(gx, xp, xq, xr) as f32;
                    let linear = i as usize + ni * (j as usize + nj * k as usize);
                    if d < distances.buffer[linear] {
                        distances.buffer[linear] = d;
                    }
                }
            }
        }
    }

    compute_intersections(mesh, origin, dx, ni, nj, nk, &mut intersection_count);
    compute_signs(ni, nj, nk, &mut distances, &intersection_count);
    distances
}

/// C# `compute_intersections` in `ParityCount` mode: each triangle crossed by an x-ray through
/// grid node (j, k) adds one to the cell where it crosses, minus one when it faces +x.
fn compute_intersections(
    mesh: &DMesh3,
    origin: [f32; 3],
    dx: f32,
    ni: usize,
    nj: usize,
    nk: usize,
    intersection_count: &mut [i32],
) {
    let (ox, oy, oz) = (origin[0] as f64, origin[1] as f64, origin[2] as f64);
    let invdx = 1.0 / dx as f64;
    for tid in 0..mesh.triangle_count() {
        let [xp, xq, xr] = mesh.get_tri_vertices(tid);
        // C# `MathUtil.FastNormalDirection`: (q - p) x (r - p), only its x is read.
        let e1 = [xq[0] - xp[0], xq[1] - xp[1], xq[2] - xp[2]];
        let e2 = [xr[0] - xp[0], xr[1] - xp[1], xr[2] - xp[2]];
        let neg_x = e1[1] * e2[2] - e1[2] * e2[1] > 0.0;

        let (fip, fjp, fkp) = (
            (xp[0] - ox) * invdx,
            (xp[1] - oy) * invdx,
            (xp[2] - oz) * invdx,
        );
        let (fiq, fjq, fkq) = (
            (xq[0] - ox) * invdx,
            (xq[1] - oy) * invdx,
            (xq[2] - oz) * invdx,
        );
        let (fir, fjr, fkr) = (
            (xr[0] - ox) * invdx,
            (xr[1] - oy) * invdx,
            (xr[2] - oz) * invdx,
        );

        let j0 = (min3(fjp, fjq, fjr).ceil() as i32).clamp(0, nj as i32 - 1);
        let j1 = (max3(fjp, fjq, fjr).floor() as i32).clamp(0, nj as i32 - 1);
        let k0 = (min3(fkp, fkq, fkr).ceil() as i32).clamp(0, nk as i32 - 1);
        let k1 = (max3(fkp, fkq, fkr).floor() as i32).clamp(0, nk as i32 - 1);

        for k in k0..=k1 {
            for j in j0..=j1 {
                let Some((a, b, c)) =
                    point_in_triangle_2d(j as f64, k as f64, fjp, fkp, fjq, fkq, fjr, fkr)
                else {
                    continue;
                };
                let fi = a * fip + b * fiq + c * fir;
                // The crossing is in (i_interval - 1, i_interval]; crossings beyond +x are dropped.
                let i_interval = fi.ceil() as i32;
                let i = if i_interval < 0 {
                    0
                } else if (i_interval as usize) < ni {
                    i_interval as usize
                } else {
                    continue;
                };
                let linear = i + ni * (j as usize + nj * k as usize);
                intersection_count[linear] += if neg_x { -1 } else { 1 };
            }
        }
    }
}

/// C# `compute_signs` in `ParityCount` mode: along each x row a positive running count is inside.
fn compute_signs(
    ni: usize,
    nj: usize,
    nk: usize,
    distances: &mut DenseGrid3f,
    intersection_count: &[i32],
) {
    for k in 0..nk {
        for j in 0..nj {
            let mut total_count = 0;
            for i in 0..ni {
                let linear = i + ni * (j + nj * k);
                total_count += intersection_count[linear];
                if total_count > 0 {
                    distances.buffer[linear] = -distances.buffer[linear];
                }
            }
        }
    }
}

/// g3 `MathUtil.Min(a, b, c)`.
fn min3(a: f64, b: f64, c: f64) -> f64 {
    a.min(b).min(c)
}

/// g3 `MathUtil.Max(a, b, c)`.
fn max3(a: f64, b: f64, c: f64) -> f64 {
    a.max(b).max(c)
}

fn sub(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}

fn dot(a: [f64; 3], b: [f64; 3]) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

/// g3 `Vector3d.Distance`.
fn distance(a: [f64; 3], b: [f64; 3]) -> f64 {
    let (dx, dy, dz) = (b[0] - a[0], b[1] - a[1], b[2] - a[2]);
    (dx * dx + dy * dy + dz * dz).sqrt()
}

/// C# `point_segment_distance` (double): distance from `x0` to segment `x1`-`x2`.
fn point_segment_distance(x0: [f64; 3], x1: [f64; 3], x2: [f64; 3]) -> f64 {
    let dx = sub(x2, x1);
    let m2 = dot(dx, dx);
    // C# clamps with two ifs; `clamp` matches it, NaN included.
    let s12 = (dot(dx, sub(x2, x0)) / m2).clamp(0.0, 1.0);
    let p = std::array::from_fn(|a| s12 * x1[a] + (1.0 - s12) * x2[a]);
    distance(x0, p)
}

/// C# `point_triangle_distance` (double): distance from `x0` to triangle `x1`-`x2`-`x3`.
fn point_triangle_distance(x0: [f64; 3], x1: [f64; 3], x2: [f64; 3], x3: [f64; 3]) -> f64 {
    // Barycentric coordinates of the closest point on the infinite plane.
    let x13 = sub(x1, x3);
    let x23 = sub(x2, x3);
    let x03 = sub(x0, x3);
    let (m13, m23, d) = (dot(x13, x13), dot(x23, x23), dot(x13, x23));
    let invdet = 1.0 / (m13 * m23 - d * d).max(1e-30);
    let (a, b) = (dot(x13, x03), dot(x23, x03));
    let w23 = invdet * (m23 * a - d * b);
    let w31 = invdet * (m13 * b - d * a);
    let w12 = 1.0 - w23 - w31;
    if w23 >= 0.0 && w31 >= 0.0 && w12 >= 0.0 {
        let p = std::array::from_fn(|i| w23 * x1[i] + w31 * x2[i] + w12 * x3[i]);
        distance(x0, p)
    } else if w23 > 0.0 {
        // This rules out edge 2-3.
        point_segment_distance(x0, x1, x2).min(point_segment_distance(x0, x1, x3))
    } else if w31 > 0.0 {
        // This rules out edge 1-3.
        point_segment_distance(x0, x1, x2).min(point_segment_distance(x0, x2, x3))
    } else {
        // w12 must be > 0, ruling out edge 1-2.
        point_segment_distance(x0, x1, x3).min(point_segment_distance(x0, x2, x3))
    }
}
