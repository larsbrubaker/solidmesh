//! Ports of the measuring helpers the end-face test classes borrow from their C# neighbours:
//! `RoundMouthFilletTests.CrossingPairs` (with `EdgesPierce` and `SegmentPiercesTriangle`),
//! `TopCenterFaceTests.ToTriangle` (with `ToSegment`) and `LevelLineTestGeometry.SquarePocket`
//! (`LevelLine/LevelLineTestGeometry.Solids.cs`), `HoleFillCornerTests.FillWithTheFiller` and
//! `CornerVolumeMeasureTests.Prism`. Used by the end-face test classes.

#![allow(dead_code)]

use geometry_processing::bevel::level_line::corner_cap_hole_fill_switch::set_use_hole_fill_override;
use geometry_processing::bevel::level_line::gap_fill::{self, GapFillError};
use geometry_processing::bevel::level_line::LevelLineMesh;
use geometry_processing::bevel::level_line::{BandGapResult, BandProfile, GapFillResult};
use vector_math::Vector3;

/// C# `RoundMouthFilletTests.CrossingPairs`: the number of pairs of kept triangles, sharing less
/// than an edge, one of whose edges pierces the other's interior.
pub fn crossing_pairs(mesh: &LevelLineMesh) -> i32 {
    let live: Vec<i32> = (0..mesh.triangle_count())
        .filter(|&t| !mesh.is_removed(t))
        .collect();
    let corners: Vec<[i32; 3]> = live
        .iter()
        .map(|&t| {
            let tri = mesh.triangles()[t as usize];
            [tri.a, tri.b, tri.c]
        })
        .collect();
    let boxes: Vec<(Vector3, Vector3)> = corners
        .iter()
        .map(|c| {
            let p: Vec<Vector3> = c.iter().map(|&v| mesh.position(v)).collect();
            let min = |f: fn(&Vector3) -> f64| p.iter().map(f).fold(f64::INFINITY, f64::min);
            let max = |f: fn(&Vector3) -> f64| p.iter().map(f).fold(f64::NEG_INFINITY, f64::max);
            (
                Vector3::new(min(|v| v.x), min(|v| v.y), min(|v| v.z)),
                Vector3::new(max(|v| v.x), max(|v| v.y), max(|v| v.z)),
            )
        })
        .collect();

    let mut crossings = 0;
    for i in 0..live.len() {
        for j in (i + 1)..live.len() {
            let (bi, bj) = (boxes[i], boxes[j]);
            if bi.1.x < bj.0.x
                || bj.1.x < bi.0.x
                || bi.1.y < bj.0.y
                || bj.1.y < bi.0.y
                || bi.1.z < bj.0.z
                || bj.1.z < bi.0.z
            {
                continue;
            }

            let mut shared: Vec<i32> = Vec::new();
            for &v in &corners[i] {
                if corners[j].contains(&v) && !shared.contains(&v) {
                    shared.push(v);
                }
            }
            if shared.len() >= 2 {
                continue;
            }

            if edges_pierce(mesh, &corners[i], &corners[j], &shared)
                || edges_pierce(mesh, &corners[j], &corners[i], &shared)
            {
                crossings += 1;
                if crossings <= 5 {
                    println!(
                        "  crossing {:?} x {:?}",
                        corners[i].map(|v| mesh.position(v)),
                        corners[j].map(|v| mesh.position(v))
                    );
                }
            }
        }
    }

    println!("{crossings} crossing pairs over {} triangles", live.len());
    crossings
}

fn edges_pierce(
    mesh: &LevelLineMesh,
    edges_of: &[i32; 3],
    target: &[i32; 3],
    shared: &[i32],
) -> bool {
    let a = mesh.position(target[0]);
    let b = mesh.position(target[1]);
    let c = mesh.position(target[2]);
    for k in 0..3 {
        let from = edges_of[k];
        let to = edges_of[(k + 1) % 3];
        if shared.contains(&from) || shared.contains(&to) {
            continue;
        }

        if segment_pierces_triangle(mesh.position(from), mesh.position(to), a, b, c) {
            return true;
        }
    }

    false
}

/// Whether the open segment p-q passes through the triangle's interior, with a small relative
/// margin so a touch at a shared seam is not a crossing. Coplanar overlap is not counted.
fn segment_pierces_triangle(p: Vector3, q: Vector3, a: Vector3, b: Vector3, c: Vector3) -> bool {
    const MARGIN: f64 = 1e-6;
    let normal = (b - a).cross(c - a);
    let scale = normal.length();
    if scale < 1e-18 {
        return false;
    }

    let dp = (p - a).dot(normal) / scale;
    let dq = (q - a).dot(normal) / scale;
    let length = (q - p).length();
    if (dp > -MARGIN * length && dq > -MARGIN * length)
        || (dp < MARGIN * length && dq < MARGIN * length)
    {
        return false;
    }

    let hit = p + ((q - p) * (dp / (dp - dq)));
    let u = (b - a).cross(hit - a).dot(normal) / (scale * scale);
    let v = (c - b).cross(hit - b).dot(normal) / (scale * scale);
    let w = (a - c).cross(hit - c).dot(normal) / (scale * scale);
    u > MARGIN && v > MARGIN && w > MARGIN
}

/// C# `TopCenterFaceTests.ToTriangle`: the distance from `p` to the triangle a-b-c.
pub fn to_triangle(p: Vector3, a: Vector3, b: Vector3, c: Vector3) -> f64 {
    let mut n = (b - a).cross(c - a);
    let length = n.length();
    if length > 1e-12 {
        n /= length;
        let h = (p - a).dot(n);
        let q = p - (n * h);
        if (b - a).cross(q - a).dot(n) >= 0.0
            && (c - b).cross(q - b).dot(n) >= 0.0
            && (a - c).cross(q - c).dot(n) >= 0.0
        {
            return h.abs();
        }
    }

    f64::min(
        to_segment(p, a, b),
        f64::min(to_segment(p, b, c), to_segment(p, c, a)),
    )
}

fn to_segment(p: Vector3, a: Vector3, b: Vector3) -> f64 {
    let d = b - a;
    let t = ((p - a).dot(d) / f64::max(d.dot(d), 1e-24)).clamp(0.0, 1.0);
    (p - (a + (d * t))).length()
}

/// A solid with the chain its test bevels: positions, triangles and the chain's vertices.
pub type SolidWithRim = (Vec<Vector3>, Vec<(i32, i32, i32)>, Vec<i32>);

/// C# `LevelLineTestGeometry.SquarePocket(outerHalf = 2, pocketHalf = 1, halfHeight = 1, floorZ =
/// 0)`: a closed block with a square BLIND pocket sunk into its top. The floor's rim is a closed
/// chain of four edges, every one of them concave.
pub fn square_pocket(
    outer_half: f64,
    pocket_half: f64,
    half_height: f64,
    floor_z: f64,
) -> SolidWithRim {
    let corners = [(-1.0, -1.0), (1.0, -1.0), (1.0, 1.0), (-1.0, 1.0)];
    let mut positions = Vec::new();
    for (x, y) in corners {
        positions.push(Vector3::new(x * outer_half, y * outer_half, half_height));
    }
    for (x, y) in corners {
        positions.push(Vector3::new(x * pocket_half, y * pocket_half, half_height));
    }
    for (x, y) in corners {
        positions.push(Vector3::new(x * outer_half, y * outer_half, -half_height));
    }
    for (x, y) in corners {
        positions.push(Vector3::new(x * pocket_half, y * pocket_half, floor_z));
    }

    let top_outer = |k: i32| k % 4;
    let mouth = |k: i32| 4 + (k % 4);
    let low_outer = |k: i32| 8 + (k % 4);
    let floor = |k: i32| 12 + (k % 4);

    let mut indices = Vec::new();
    for k in 0..4 {
        // The top face, an annulus round the mouth wound outward (+z).
        indices.push((top_outer(k), top_outer(k + 1), mouth(k + 1)));
        indices.push((top_outer(k), mouth(k + 1), mouth(k)));

        // The outer wall, facing out of the block.
        indices.push((top_outer(k), low_outer(k), low_outer(k + 1)));
        indices.push((top_outer(k), low_outer(k + 1), top_outer(k + 1)));

        // The pocket's wall, facing into the pocket.
        indices.push((mouth(k), floor(k + 1), floor(k)));
        indices.push((mouth(k), mouth(k + 1), floor(k + 1)));
    }

    // The bottom of the block, wound downward (-z), and the pocket's floor, wound up into the pocket.
    indices.push((low_outer(0), low_outer(2), low_outer(1)));
    indices.push((low_outer(0), low_outer(3), low_outer(2)));
    indices.push((floor(0), floor(1), floor(2)));
    indices.push((floor(0), floor(2), floor(3)));

    (positions, indices, vec![12, 13, 14, 15])
}

/// C# `HoleFillCornerTests.FillWithTheFiller`: fills with the hole filler on for this call only,
/// on this thread.
pub fn fill_with_the_filler(
    gap: &BandGapResult,
    profile: BandProfile,
    segments: i32,
) -> Result<GapFillResult, GapFillError> {
    set_use_hole_fill_override(Some(true));
    let filled = gap_fill::fill(gap, profile, segments);
    set_use_hole_fill_override(None);
    filled
}

/// C# `CornerVolumeMeasureTests.Prism`: the exact fillet prism of radius `r` along the chain -
/// per edge, its length times r^2 (tan(phi / 2) - phi / 2), taken off where the edge is convex and
/// added where it is concave.
pub fn prism(
    positions: &[Vector3],
    indices: &[(i32, i32, i32)],
    chain: &[i32],
    closed: bool,
    r: f64,
) -> f64 {
    let mut total = 0.0;
    let steps = if closed { chain.len() } else { chain.len() - 1 };
    let has = |t: &(i32, i32, i32), v: i32| t.0 == v || t.1 == v || t.2 == v;
    let normal = |t: (i32, i32, i32)| {
        let at = |v: i32| positions[v as usize];
        (at(t.1) - at(t.0)).cross(at(t.2) - at(t.0)).get_normal()
    };
    for i in 0..steps {
        let a = chain[i];
        let b = chain[(i + 1) % chain.len()];
        let on: Vec<(i32, i32, i32)> = indices
            .iter()
            .filter(|t| has(t, a) && has(t, b))
            .copied()
            .collect();
        if on.len() != 2 {
            continue;
        }

        let n1 = normal(on[0]);
        let phi = n1.dot(normal(on[1])).clamp(-1.0, 1.0).acos();
        let third = if on[1].0 != a && on[1].0 != b {
            on[1].0
        } else if on[1].1 != a && on[1].1 != b {
            on[1].1
        } else {
            on[1].2
        };
        let convex = n1.dot(positions[third as usize] - positions[a as usize]) < 0.0;
        total += (if convex { 1.0 } else { -1.0 })
            * (positions[b as usize] - positions[a as usize]).length()
            * r
            * r
            * ((phi / 2.0).tan() - (phi / 2.0));
    }

    total
}
