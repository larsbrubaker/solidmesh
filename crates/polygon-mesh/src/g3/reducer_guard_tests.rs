//! Tests for the reducer's distance guard (`reducer_guard.rs`): on a sphere with a needle,
//! whose small faces plain quadric error cuts down cheaply, the guarded reduce keeps the surface
//! closer to the needle, stays closed and repeats to the bit, with and without Maintain Surface.

use super::dmesh3::DMesh3;
use super::reducer::ReduceOptions;
use crate::mesh_distance::surface_deviation;
use crate::Mesh;

/// A UV sphere of radius 1 (`rings` bands, `segments` around) whose north cap is pulled out
/// into a needle: the first ring shrunk to radius `width` at height 1, the pole lifted to
/// `tip`.
fn needle(rings: i32, segments: i32, width: f64, tip: f64) -> Mesh {
    let mut m = DMesh3::default();
    let top = m.append_vertex([0.0, 0.0, tip]);
    for i in 1..rings {
        let phi = std::f64::consts::PI * i as f64 / rings as f64;
        for j in 0..segments {
            let theta = 2.0 * std::f64::consts::PI * j as f64 / segments as f64;
            let (r, z) = if i == 1 {
                (width, 1.0)
            } else {
                (libm::sin(phi), libm::cos(phi))
            };
            m.append_vertex([r * libm::cos(theta), r * libm::sin(theta), z]);
        }
    }
    let bottom = m.append_vertex([0.0, 0.0, -1.0]);
    let ring = |i: i32, j: i32| 1 + (i - 1) * segments + (j % segments);
    for j in 0..segments {
        m.append_triangle(top, ring(1, j), ring(1, j + 1));
        m.append_triangle(bottom, ring(rings - 1, j + 1), ring(rings - 1, j));
    }
    for i in 1..rings - 1 {
        for j in 0..segments {
            let (a, b, c, d) = (
                ring(i, j),
                ring(i, j + 1),
                ring(i + 1, j),
                ring(i + 1, j + 1),
            );
            m.append_triangle(a, c, d);
            m.append_triangle(a, d, b);
        }
    }
    m.to_mesh()
}

#[test]
fn the_guard_keeps_more_of_a_needle_than_plain_quadric_error() {
    let shape = needle(32, 32, 0.05, 1.15);
    let target = 80;
    let plain = shape.reduced_with(
        target,
        ReduceOptions {
            guard: false,
            ..ReduceOptions::default()
        },
    );
    let guarded = shape.reduced_with(target, ReduceOptions::default());
    let d_plain = surface_deviation(&shape, &plain, 50_000, 0.99);
    let d_guarded = surface_deviation(&shape, &guarded, 50_000, 0.99);
    assert!(guarded.faces.len() <= target as usize);
    // Measured: plain quadric error leaves the surface 0.062 of the size from the needle's tip,
    // the guard 0.043.
    assert!(
        d_guarded.max_fraction() < 0.8 * d_plain.max_fraction(),
        "guarded {} against plain {}",
        d_guarded.max_fraction(),
        d_plain.max_fraction()
    );
}

#[test]
fn a_guarded_reduce_repeats_to_the_bit_and_stays_closed() {
    let shape = needle(32, 32, 0.05, 1.15);
    for maintain in [false, true] {
        let first = shape.reduced_to_triangle_count(120, maintain);
        let second = shape.reduced_to_triangle_count(120, maintain);
        assert!(
            first.get_non_manifold_edges().is_empty(),
            "maintain {maintain}"
        );
        let bits = |m: &Mesh| -> Vec<u32> {
            m.vertices
                .iter()
                .flat_map(|v| [v.x.to_bits(), v.y.to_bits(), v.z.to_bits()])
                .chain(
                    m.faces
                        .iter()
                        .flat_map(|f| [f.v0 as u32, f.v1 as u32, f.v2 as u32]),
                )
                .collect()
        };
        assert_eq!(bits(&first), bits(&second), "maintain {maintain}");
    }
}
