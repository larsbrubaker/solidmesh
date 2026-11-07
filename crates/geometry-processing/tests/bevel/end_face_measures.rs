//! Measuring helpers the end-face test classes share: the distance to a triangle (`to_triangle`,
//! with its segment form) and the exact fillet prism (`prism`).

#![allow(dead_code)]

use vector_math::Vector3;

/// The distance from `p` to the triangle a-b-c.
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

/// The exact fillet prism of radius `r` along the chain -
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
