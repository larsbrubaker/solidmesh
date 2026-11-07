//! The feature edges of a test mesh: an edge is a feature where it is not two triangles meeting
//! within a fold threshold. Shared by the several-chain fixtures and the room-reading tests that
//! read their feature edges.

#![allow(dead_code)]

use std::collections::HashMap;
use std::rc::Rc;

use geometry_processing::bevel::level_line::seam_edges::FeatureEdgeTest;
use vector_math::Vector3;

/// An edge is a feature where it is not two triangles
/// meeting within `degrees`.
pub fn feature_edges_over(
    positions: &[Vector3],
    indices: &[(i32, i32, i32)],
    degrees: f64,
) -> FeatureEdgeTest {
    let mut standing: HashMap<(i32, i32), Vec<usize>> = HashMap::new();
    for (i, &(a, b, c)) in indices.iter().enumerate() {
        for (p, q) in [(a, b), (b, c), (c, a)] {
            let key = if p < q { (p, q) } else { (q, p) };
            standing
                .entry(key)
                .or_insert_with(|| Vec::with_capacity(2))
                .push(i);
        }
    }

    let normals: Vec<Vector3> = indices
        .iter()
        .map(|&(a, b, c)| {
            let pa = positions[a as usize];
            (positions[b as usize] - pa)
                .cross(positions[c as usize] - pa)
                .get_normal()
        })
        .collect();
    let limit = (degrees * std::f64::consts::PI / 180.0).cos();

    Rc::new(move |a, b| {
        let key = if a < b { (a, b) } else { (b, a) };
        match standing.get(&key) {
            Some(pair) if pair.len() == 2 => normals[pair[0]].dot(normals[pair[1]]) < limit,
            _ => true,
        }
    })
}
