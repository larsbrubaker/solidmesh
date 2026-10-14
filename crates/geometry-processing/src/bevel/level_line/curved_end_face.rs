//! Whether a chain's end face is a facet of a ROUND rather than a flat face - the test the
//! level-line end-face sense asks before it stands an end ray on the face's plane.

use std::collections::{HashMap, HashSet, VecDeque};

use vector_math::Vector3;

const COPLANAR_TOLERANCE: f64 = 1e-6;

/// One triangle by its three corner indices into the positions.
pub type Corners = (i32, i32, i32);

/// Whether the plane of `normal` grown from `seeds` over edge neighbours in that plane meets,
/// across any of its edges, a triangle turned off it by less than `sharp_turn_degrees` - a facet
/// of a round rather than a flat face. Seeds that are not triangles of `indices` are ignored.
///
/// The answer does not depend on the order the flood visits triangles in: the coplanar region is
/// a closure, and the answer is whether any triangle bordering it turns softly.
pub fn bends_on_softly(
    positions: &[Vector3],
    indices: &[Corners],
    normal: Vector3,
    seeds: impl IntoIterator<Item = Corners>,
    sharp_turn_degrees: f64,
) -> bool {
    // Only looked up, never iterated, so the HashMaps' order cannot leak out.
    let mut by_edge: HashMap<(i32, i32), Vec<usize>> = HashMap::new();
    let mut by_corners: HashMap<Corners, usize> = HashMap::new();
    for (i, &t) in indices.iter().enumerate() {
        by_corners.insert(t, i);
        for key in edges_of(t) {
            by_edge.entry(key).or_default().push(i);
        }
    }

    let mut region = HashSet::new();
    let mut open = VecDeque::new();
    for seed in seeds {
        if let Some(&i) = by_corners.get(&seed) {
            if region.insert(i) {
                open.push_back(i);
            }
        }
    }

    let position = |index: i32| positions[index as usize];

    while let Some(current) = open.pop_front() {
        for key in edges_of(indices[current]) {
            for &neighbour in &by_edge[&key] {
                if region.contains(&neighbour) {
                    continue;
                }

                let t = indices[neighbour];
                let face_normal = (position(t.1) - position(t.0))
                    .cross(position(t.2) - position(t.0))
                    .get_normal();
                let cos = face_normal.dot(normal).clamp(-1.0, 1.0);
                if cos >= 1.0 - COPLANAR_TOLERANCE {
                    region.insert(neighbour);
                    open.push_back(neighbour);
                } else if cos.acos() * 180.0 / std::f64::consts::PI < sharp_turn_degrees {
                    return true;
                }
            }
        }
    }

    false
}

fn edges_of(t: Corners) -> [(i32, i32); 3] {
    let (a, b, c) = t;
    [
        if a < b { (a, b) } else { (b, a) },
        if b < c { (b, c) } else { (c, b) },
        if c < a { (c, a) } else { (a, c) },
    ]
}
