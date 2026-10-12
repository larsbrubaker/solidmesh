//! Port of `RoundMouthFilletTests.CrossingPairs` (`LevelLine/RoundMouthFilletTests.cs`), the
//! shared count of triangle pairs that pierce each other, which `HoleFillOneLoopCornerTests` and
//! other classes read; the class's own tests are ported separately.

#![allow(dead_code)]

use geometry_processing::bevel::level_line::LevelLineMesh;
use vector_math::Vector3;

/// C# `CrossingPairs`: how many pairs of live triangles not sharing an edge have an edge of one
/// passing through the other's interior.
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
            (
                Vector3::new(
                    p.iter().map(|v| v.x).fold(f64::INFINITY, f64::min),
                    p.iter().map(|v| v.y).fold(f64::INFINITY, f64::min),
                    p.iter().map(|v| v.z).fold(f64::INFINITY, f64::min),
                ),
                Vector3::new(
                    p.iter().map(|v| v.x).fold(f64::NEG_INFINITY, f64::max),
                    p.iter().map(|v| v.y).fold(f64::NEG_INFINITY, f64::max),
                    p.iter().map(|v| v.z).fold(f64::NEG_INFINITY, f64::max),
                ),
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

            // C#'s `corners[i].Intersect(corners[j])`: the distinct shared vertices.
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

/// C# `SegmentPiercesTriangle`: whether the open segment p-q passes through the triangle's
/// interior, with a small relative margin so a touch at a shared seam is not a crossing. Coplanar
/// overlap is not counted.
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
