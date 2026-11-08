//! The run-out strip's top rim - an extra top point is sewn into the piece, a quad naming one
//! vertex twice is drawn as the triangle it is, and an empty top rim is no top rim at all.

use std::collections::HashMap;

use solidmesh::bevel::surface::{surface_mixed_corner_run_out_strip, SurfaceVertexPool};
use solidmesh_math::Vector3;

fn two_columns() -> (SurfaceVertexPool, [i32; 3], [i32; 3], i32) {
    let mut pool = SurfaceVertexPool::new(&[]);
    let here = [
        pool.add(Vector3::new(0.0, 0.0, 2.0)),
        pool.add(Vector3::new(0.0, 0.0, 1.0)),
        pool.add(Vector3::new(0.0, 0.0, 0.0)),
    ];
    let there = [
        pool.add(Vector3::new(1.0, 0.0, 2.0)),
        pool.add(Vector3::new(1.0, 0.0, 1.0)),
        pool.add(Vector3::new(1.0, 0.0, 0.0)),
    ];
    let extra = pool.add(Vector3::new(0.5, 0.0, 2.5));
    (pool, here, there, extra)
}

const OUTWARD: Vector3 = Vector3::new(0.0, -1.0, 0.0);

#[test]
fn the_extra_top_point_is_sewn_into_the_piece() {
    let (mut pool, here, there, extra) = two_columns();
    let mut faces = Vec::new();

    surface_mixed_corner_run_out_strip::emit(
        &mut pool,
        &mut faces,
        &here,
        &there,
        0,
        0,
        Some(&[extra]),
        OUTWARD,
    );

    let mut walked: HashMap<(i32, i32), i32> = HashMap::new();
    for &(v0, v1, v2) in &faces {
        for side in [(v0, v1), (v1, v2), (v2, v0)] {
            *walked.entry(side).or_default() += 1;
        }
    }

    let twice: Vec<_> = walked
        .iter()
        .filter(|(_, &n)| n > 1)
        .map(|(k, _)| *k)
        .collect();
    assert!(
        twice.is_empty(),
        "no side of the piece is walked the same way twice"
    );

    // The chord between the two tops is INSIDE the piece now.
    let one_way = walked.get(&(there[0], here[0])).copied().unwrap_or(0);
    let the_other = walked.get(&(here[0], there[0])).copied().unwrap_or(0);
    assert_eq!(
        one_way + the_other,
        2,
        "the chord between the two tops is an inside edge of the piece - instead {one_way} and {the_other}"
    );

    let into_the_row = walked.get(&(extra, here[0])).copied().unwrap_or(0);
    let out_of_the_top = walked.get(&(there[0], extra)).copied().unwrap_or(0);
    assert_eq!(
        into_the_row + out_of_the_top,
        2,
        "the top rim is the two edges through the extra point, each walked once"
    );
}

#[test]
fn a_quad_naming_one_vertex_twice_is_drawn_as_the_triangle_it_is() {
    let (mut pool, here, there, _) = two_columns();

    // The two columns share their top, exactly as two columns of one lens do.
    let shared = [here[0], there[1], there[2]];
    let mut faces = Vec::new();
    let stood = pool.vertices().len();

    surface_mixed_corner_run_out_strip::emit(
        &mut pool, &mut faces, &here, &shared, 0, 0, None, OUTWARD,
    );

    assert_eq!(
        pool.vertices().len(),
        stood,
        "no vertex is minted to fan a quad that was really a triangle"
    );
    for &(v0, v1, v2) in &faces {
        assert!(
            !(v0 == v1 || v1 == v2 || v2 == v0),
            "no triangle of the piece names one vertex twice"
        );
    }
}

#[test]
fn without_a_top_rim_the_piece_is_unchanged() {
    let (mut pool, here, there, _) = two_columns();
    let mut with_none = Vec::new();
    let mut with_empty = Vec::new();

    surface_mixed_corner_run_out_strip::emit(
        &mut pool,
        &mut with_none,
        &here,
        &there,
        0,
        0,
        None,
        OUTWARD,
    );
    surface_mixed_corner_run_out_strip::emit(
        &mut pool,
        &mut with_empty,
        &here,
        &there,
        0,
        0,
        Some(&[]),
        OUTWARD,
    );

    // The same faces, in any order.
    let mut a = with_empty.clone();
    let mut b = with_none.clone();
    a.sort_unstable();
    b.sort_unstable();
    assert_eq!(a, b, "an empty top rim is no top rim at all");
    assert!(!with_none.is_empty(), "the piece is drawn either way");
}
