//! The one filler for every end and corner hole: Liepa's minimum-weight triangulation of the loop,
//! Liepa's refinement to the strips' edge length, then a discrete bi-Laplacian for the interior
//! with the loop pinned and each loop point's cross-tangent fixing the fill's tangent plane there
//! (`hole_fill_fairing`). Pure: plain points in, plain triangles out, no mesh.

use std::collections::{HashMap, HashSet};

use solidmesh_math::{ieee_math, Vector3};

use super::hole_fill_fairing;
use super::{HoleFillFailure, HoleFillResult, HoleFillTriangle, LevelLineWork, WorkError};

/// Liepa's density factor: a triangle is split when its centroid is this far past both scales
/// (the square root of 2).
pub fn density() -> f64 {
    2f64.sqrt()
}

/// Refinement stops with [`HoleFillFailure::TooManyVertices`] past this many vertices.
pub const VERTEX_BUDGET: usize = 4000;

/// Worst dihedral angles closer than this (radians) count as equal, so the area term decides -
/// otherwise round-off decides between triangulations of a planar loop, and a rigid motion could
/// change the answer.
pub const ANGLE_TIE: f64 = 1e-9;

/// Relative area difference below which two triangulations tie (see [`ANGLE_TIE`]).
pub const AREA_TIE: f64 = 1e-9;

/// Fills the hole bounded by `loop_points`. The loop's order sets the winding: every fill triangle
/// runs its loop edge i -> i+1 forwards, so the fill's normal at loop point i is
/// (next - previous) x cross-tangent. The surface around the hole therefore runs the loop
/// backwards; a caller holding the surrounding surface's boundary passes it reversed.
/// `cross_tangents` gives per loop point a unit direction into the hole, tangent to the surface
/// the fill must meet there; `edge_lengths` per loop point the edge length the fill's interior
/// should approach there.
pub fn fill(
    loop_points: &[Vector3],
    cross_tangents: &[Vector3],
    edge_lengths: &[f64],
) -> Result<HoleFillResult, WorkError> {
    let mut vertices: Vec<Vector3> = loop_points.to_vec();
    let mut triangles: Vec<HoleFillTriangle> = Vec::new();

    let n = loop_points.len();
    if n < 3 {
        return Ok(HoleFillResult::new(
            HoleFillFailure::TooFewPoints,
            vertices,
            triangles,
        ));
    }

    if cross_tangents.len() != n || edge_lengths.len() != n {
        return Ok(HoleFillResult::new(
            HoleFillFailure::InputCountsDiffer,
            vertices,
            triangles,
        ));
    }

    let tiny = tiny_area(loop_points);
    if !triangulate(loop_points, cross_tangents, tiny, &mut triangles)? {
        return Ok(HoleFillResult::new(
            HoleFillFailure::NoValidTriangulation,
            vertices,
            triangles,
        ));
    }

    let mut scale = edge_lengths.to_vec();
    if !refine(&mut vertices, &mut scale, &mut triangles) {
        return Ok(HoleFillResult::new(
            HoleFillFailure::TooManyVertices,
            vertices,
            triangles,
        ));
    }

    if !hole_fill_fairing::fair(&mut vertices, &triangles, n, cross_tangents)? {
        return Ok(HoleFillResult::new(
            HoleFillFailure::SingularFairing,
            vertices,
            triangles,
        ));
    }

    let failure = validate(&vertices, &triangles, n, tiny);
    Ok(HoleFillResult::new(failure, vertices, triangles))
}

/// A triangle this small (relative to the loop's extent) counts as zero-area.
pub fn tiny_area(loop_points: &[Vector3]) -> f64 {
    let mut min = loop_points[0];
    let mut max = loop_points[0];
    for &p in loop_points {
        min = Vector3::component_min(min, p);
        max = Vector3::component_max(max, p);
    }

    let extent = (max - min).length();
    1e-12 * extent * extent
}

/// Liepa's minimum-weight triangulation (O(n^3) dynamic programming). The weight of a
/// triangulation is its worst dihedral angle - between neighbouring fill triangles, and across each
/// loop edge against the surrounding surface's normal, (next - previous) x cross-tangent - with
/// total area breaking ties. `Ok(false)` where no triangulation has every triangle above
/// `tiny_area`.
pub fn triangulate(
    loop_points: &[Vector3],
    cross_tangents: &[Vector3],
    tiny_area: f64,
    triangles: &mut Vec<HoleFillTriangle>,
) -> Result<bool, WorkError> {
    let n = loop_points.len();
    let surface_normal: Vec<Vector3> = (0..n)
        .map(|i| {
            let along = loop_points[(i + 1) % n] - loop_points[(i + n - 1) % n];
            along.cross(cross_tangents[i]).get_normal()
        })
        .collect();

    // the surrounding surface's normal across loop edge i -> i+1
    let edge_normal = |i: usize| (surface_normal[i] + surface_normal[(i + 1) % n]).get_normal();
    let triangle_normal = |a: usize, b: usize, c: usize| {
        (loop_points[b] - loop_points[a])
            .cross(loop_points[c] - loop_points[a])
            .get_normal()
    };

    let at = |i: usize, k: usize| (i * n) + k;
    let mut angle = vec![0.0; n * n];
    let mut area = vec![0.0; n * n];
    let mut middle = vec![-1i64; n * n];

    for span in 2..n {
        let mut i = 0;
        while i + span < n {
            // The whole cubic table is charged once per call, so a large hole would not see a
            // cancel.
            LevelLineWork::poll_cancel()?;
            let k = i + span;
            let mut best_angle = f64::MAX;
            let mut best_area = f64::MAX;
            for m in i + 1..k {
                if (m - i > 1 && middle[at(i, m)] < 0) || (k - m > 1 && middle[at(m, k)] < 0) {
                    continue;
                }

                let cross =
                    (loop_points[m] - loop_points[i]).cross(loop_points[k] - loop_points[i]);
                let triangle_area = cross.length() / 2.0;
                if triangle_area <= tiny_area {
                    continue;
                }

                let normal = cross.get_normal();
                let mut worst = ieee_math::max_f64(angle[at(i, m)], angle[at(m, k)]);
                let left = if m == i + 1 {
                    edge_normal(i)
                } else {
                    triangle_normal(i, middle[at(i, m)] as usize, m)
                };
                worst = ieee_math::max_f64(worst, dihedral(normal, left));
                let right = if k == m + 1 {
                    edge_normal(m)
                } else {
                    triangle_normal(m, middle[at(m, k)] as usize, k)
                };
                worst = ieee_math::max_f64(worst, dihedral(normal, right));
                if span == n - 1 {
                    // the last triangle also carries the closing loop edge n-1 -> 0
                    worst = ieee_math::max_f64(worst, dihedral(normal, edge_normal(n - 1)));
                }

                let total = area[at(i, m)] + area[at(m, k)] + triangle_area;
                // every triangulation of a planar loop has the same area, so area ties too are read
                // with a tolerance and the first middle point in loop order wins - never round-off
                if worst < best_angle - ANGLE_TIE
                    || (worst <= best_angle + ANGLE_TIE && total < best_area * (1.0 - AREA_TIE))
                {
                    best_angle = worst;
                    best_area = total;
                    middle[at(i, k)] = m as i64;
                }
            }

            angle[at(i, k)] = best_angle;
            area[at(i, k)] = best_area;
            i += 1;
        }
    }

    if middle[at(0, n - 1)] < 0 {
        return Ok(false);
    }

    let mut pending = vec![(0usize, n - 1)];
    while let Some((i, k)) = pending.pop() {
        if k - i < 2 {
            continue;
        }

        let m = middle[at(i, k)] as usize;
        triangles.push(HoleFillTriangle::new(i as i32, m as i32, k as i32));
        pending.push((m, k));
        pending.push((i, m));
    }

    Ok(true)
}

/// The angle between two unit normals.
pub fn dihedral(a: Vector3, b: Vector3) -> f64 {
    libm::acos(a.dot(b).clamp(-1.0, 1.0))
}

/// Liepa's refinement: split a triangle at its centroid while the centroid is more than sqrt(2)
/// times both the interpolated edge-length scale and each corner's own scale away from that
/// corner, then relax by edge flips; repeat until nothing splits. New vertices take the mean
/// scale of the triangle they split. False past [`VERTEX_BUDGET`] vertices.
pub fn refine(
    vertices: &mut Vec<Vector3>,
    scale: &mut Vec<f64>,
    triangles: &mut Vec<HoleFillTriangle>,
) -> bool {
    relax(vertices, triangles);
    loop {
        let mut split = false;
        let count = triangles.len();
        for t in 0..count {
            let HoleFillTriangle { a, b, c } = triangles[t];
            let (ua, ub, uc) = (a as usize, b as usize, c as usize);
            let centroid = (vertices[ua] + vertices[ub] + vertices[uc]) / 3.0;
            let centroid_scale = (scale[ua] + scale[ub] + scale[uc]) / 3.0;
            let splits = [ua, ub, uc].iter().all(|&corner| {
                let reach = density() * (centroid - vertices[corner]).length();
                !(reach <= centroid_scale || reach <= scale[corner])
            });
            if !splits {
                continue;
            }

            let v = vertices.len() as i32;
            vertices.push(centroid);
            scale.push(centroid_scale);
            triangles[t] = HoleFillTriangle::new(a, b, v);
            triangles.push(HoleFillTriangle::new(b, c, v));
            triangles.push(HoleFillTriangle::new(c, a, v));
            split = true;
        }

        if !split {
            return true;
        }

        if vertices.len() > VERTEX_BUDGET {
            return false;
        }

        relax(vertices, triangles);
    }
}

/// Flips interior edges whose two opposite angles sum past pi (the Delaunay test, read in each
/// triangle's own plane) until none is left. Loop edges have one triangle and are never flipped.
pub fn relax(vertices: &[Vector3], triangles: &mut [HoleFillTriangle]) {
    let mut owner: HashMap<(i32, i32), usize> = HashMap::new();
    for (t, tri) in triangles.iter().enumerate() {
        owner.insert((tri.a, tri.b), t);
        owner.insert((tri.b, tri.c), t);
        owner.insert((tri.c, tri.a), t);
    }

    let angle = |at: i32, p: i32, q: i32| {
        let u = (vertices[p as usize] - vertices[at as usize]).get_normal();
        let w = (vertices[q as usize] - vertices[at as usize]).get_normal();
        libm::acos(u.dot(w).clamp(-1.0, 1.0))
    };

    // a cap on sweeps; flipping by the Delaunay test terminates, this only guards against
    // round-off cycles
    for _ in 0..100 {
        let mut flipped = false;
        for t in 0..triangles.len() {
            let tri = triangles[t];
            for (a, b, c) in [
                (tri.a, tri.b, tri.c),
                (tri.b, tri.c, tri.a),
                (tri.c, tri.a, tri.b),
            ] {
                let Some(&other) = owner.get(&(b, a)) else {
                    continue;
                };

                let o = triangles[other];
                let d = if o.a != a && o.a != b {
                    o.a
                } else if o.b != a && o.b != b {
                    o.b
                } else {
                    o.c
                };
                if d == c || owner.contains_key(&(c, d)) || owner.contains_key(&(d, c)) {
                    continue;
                }

                if angle(c, a, b) + angle(d, a, b) <= std::f64::consts::PI + 1e-9 {
                    continue;
                }

                owner.remove(&(a, b));
                owner.remove(&(b, a));
                triangles[t] = HoleFillTriangle::new(c, a, d);
                triangles[other] = HoleFillTriangle::new(d, b, c);
                owner.insert((c, a), t);
                owner.insert((a, d), t);
                owner.insert((d, c), t);
                owner.insert((d, b), other);
                owner.insert((b, c), other);
                owner.insert((c, d), other);
                flipped = true;
                break;
            }
        }

        if !flipped {
            return;
        }
    }
}

/// The fill is a consistently oriented disk bounded by exactly the loop (each directed edge used
/// once, each loop edge forwards, every other edge in both directions) with no zero-area triangle.
pub fn validate(
    vertices: &[Vector3],
    triangles: &[HoleFillTriangle],
    n: usize,
    tiny_area: f64,
) -> HoleFillFailure {
    let n = n as i32;
    let mut directed = HashSet::new();
    for t in triangles {
        if !directed.insert((t.a, t.b))
            || !directed.insert((t.b, t.c))
            || !directed.insert((t.c, t.a))
        {
            return HoleFillFailure::NotClosed;
        }
    }

    let mut open = 0;
    for &(a, b) in &directed {
        if !directed.contains(&(b, a)) {
            open += 1;
            if a >= n || b != (a + 1) % n {
                return HoleFillFailure::NotClosed;
            }
        }
    }

    let euler =
        vertices.len() as i64 - ((directed.len() as i64 + n as i64) / 2) + triangles.len() as i64;
    if open != n || euler != 1 {
        return HoleFillFailure::NotClosed;
    }

    for t in triangles {
        let (a, b, c) = (
            vertices[t.a as usize],
            vertices[t.b as usize],
            vertices[t.c as usize],
        );
        if (b - a).cross(c - a).length() / 2.0 <= tiny_area {
            return HoleFillFailure::ZeroAreaTriangle;
        }
    }

    HoleFillFailure::None
}
