//! Re-triangulates a source's flat faces the other way. Every diagonal shared by two coplanar
//! triangles that form a strictly convex quad is swapped for the quad's other diagonal. The part's
//! shape is unchanged; only the triangles across its flat faces differ - which is what a source
//! built by another boolean route hands the level line. Used by
//! `level_line_triangulation_invariance_tests`.

use std::collections::{BTreeMap, HashSet};

use vector_math::Vector3;

/// The source with every flippable coplanar diagonal flipped, and how many were.
/// Each triangle takes part in at most one flip, so the result does not depend on the order of
/// later flips undoing earlier ones. An edge `keep` names (feature and chain edges) is never
/// flipped. The usual plane tolerance is 1e-9.
pub fn flip_all(
    positions: &[Vector3],
    indices: &[(i32, i32, i32)],
    keep: impl Fn(i32, i32) -> bool,
    plane_tolerance: f64,
) -> (Vec<(i32, i32, i32)>, i32) {
    let mut triangles = indices.to_vec();
    let mut touched = vec![false; triangles.len()];
    let mut edges: HashSet<(i32, i32)> = triangles
        .iter()
        .flat_map(|&t| edges_of(t))
        .map(|(a, b)| key(a, b))
        .collect();
    // A sorted map walks the edges in key order, so the flips are the same every run.
    let mut standing: BTreeMap<(i32, i32), Vec<usize>> = BTreeMap::new();
    for (t, &triangle) in triangles.iter().enumerate() {
        for (a, b) in edges_of(triangle) {
            standing.entry(key(a, b)).or_default().push(t);
        }
    }

    let at = |v: i32| positions[v as usize];
    let mut flipped = 0;
    for (&edge, on) in &standing {
        if on.len() != 2 || touched[on[0]] || touched[on[1]] || keep(edge.0, edge.1) {
            continue;
        }

        // Oriented so the first triangle runs a -> b and the second b -> a.
        let (mut first, mut second) = (on[0], on[1]);
        if !runs(triangles[first], edge.0, edge.1) {
            std::mem::swap(&mut first, &mut second);
        }

        let a = edge.0;
        let b = edge.1;
        let c = opposite(triangles[first], a, b);
        let d = opposite(triangles[second], a, b);
        if c == d || edges.contains(&key(c, d)) {
            continue;
        }

        let mut normal = (at(b) - at(a)).cross(at(c) - at(a));
        if normal.length() == 0.0 {
            continue;
        }

        normal = normal.get_normal();
        if normal.dot(at(d) - at(a)).abs() > plane_tolerance {
            continue;
        }

        // (a, b, c) and (b, a, d) become (c, d, b) and (d, c, a); both must face the way the pair
        // did with real area, or the quad is not strictly convex.
        let one = (at(d) - at(c)).cross(at(b) - at(c));
        let two = (at(c) - at(d)).cross(at(a) - at(d));
        let smallest = 1e-12 * (at(c) - at(d)).length_squared();
        if one.dot(normal) <= smallest || two.dot(normal) <= smallest {
            continue;
        }

        triangles[first] = (c, d, b);
        triangles[second] = (d, c, a);
        touched[first] = true;
        touched[second] = true;
        edges.remove(&edge);
        edges.insert(key(c, d));
        flipped += 1;
    }

    (triangles, flipped)
}

fn key(a: i32, b: i32) -> (i32, i32) {
    if a < b {
        (a, b)
    } else {
        (b, a)
    }
}

fn edges_of(t: (i32, i32, i32)) -> [(i32, i32); 3] {
    [(t.0, t.1), (t.1, t.2), (t.2, t.0)]
}

fn runs(t: (i32, i32, i32), from: i32, to: i32) -> bool {
    edges_of(t).contains(&(from, to))
}

fn opposite(t: (i32, i32, i32), a: i32, b: i32) -> i32 {
    if t.0 != a && t.0 != b {
        t.0
    } else if t.1 != a && t.1 != b {
        t.1
    } else {
        t.2
    }
}
