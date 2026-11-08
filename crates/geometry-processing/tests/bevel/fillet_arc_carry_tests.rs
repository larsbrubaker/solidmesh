//! A closed chain's zip row carried round a gentle turn, kept to the mean drawing at a mitre or
//! under an infinite bend threshold, the carry's fade into a mitre's window, and the chain's
//! direction at its seam.

use solidmesh::bevel::level_line::fillet_arc::{carry_weight, tangent_at};
use solidmesh::bevel::level_line::fillet_arc_rows::{along_the_chain, build};
use solidmesh::bevel::level_line::{ChainField, LevelLineMesh};
use solidmesh_math::{ieee_math, Vector3};

#[test]
fn a_row_straddling_a_mitre_evenly_keeps_the_mean_drawing() {
    let (carried, mean) = row(15.0, 10.0);
    assert!(gap(&carried, &mean) < 1e-12);
}

#[test]
fn an_infinite_bend_threshold_keeps_the_mean_drawing() {
    let (carried_finite, mean) = row(5.0, 10.0);
    let (carried_infinite, _) = row(5.0, f64::INFINITY);
    assert!(
        gap(&carried_finite, &mean) > 1e-6,
        "a gentle turn under a finite threshold carries the row"
    );
    assert!(gap(&carried_infinite, &mean) < 1e-12);
}

#[test]
fn rows_fade_smoothly_into_a_mitres_window() {
    let turn = 30.0 * std::f64::consts::PI / 180.0;
    let second = Vector3::new(turn.cos(), turn.sin(), 0.0);
    let points = [
        Vector3::ZERO,
        Vector3::new(10.0, 0.0, 0.0),
        Vector3::new(10.0, 0.0, 0.0) + (second * 10.0),
    ];
    let along = Vector3::new(1.0, 0.0, 0.0);
    let chain =
        ChainField::from_polyline_sharp(&points, false, &[1.0, 1.0, 1.0], 10.0).expect("a chain");

    let mut weights = Vec::new();
    let mut start = 5.0;
    while start <= 9.5 + 1e-9 {
        weights.push(carry_weight(&chain, start, start + 0.5, along, along));
        start += 0.05;
    }

    assert_eq!(weights[0], 1.0);
    assert_eq!(weights[weights.len() - 1], 0.0);
    assert!(
        weights.iter().any(|&w| w > 0.05 && w < 0.95),
        "the fade passes through partial carries"
    );
    for k in 1..weights.len() {
        assert!(
            weights[k] <= weights[k - 1],
            "row {k} nearer the mitre carries no more"
        );
        assert!(
            weights[k - 1] - weights[k] < 0.1,
            "rows {} and {k} carry by nearly the same amount",
            k - 1
        );
    }

    let narrow =
        ChainField::from_polyline_sharp(&points, false, &[1.0, 0.25, 1.0], 10.0).expect("a chain");
    assert_eq!(carry_weight(&narrow, 8.9, 9.4, along, along), 1.0);
}

#[test]
fn the_direction_at_the_seam_is_the_direction_at_zero() {
    let points = [
        Vector3::new(3.0, 0.0, 0.0),
        Vector3::new(1.0, 2.5, 0.0),
        Vector3::new(-2.0, 1.0, 0.0),
        Vector3::new(-1.5, -2.0, 0.0),
        Vector3::new(1.2, -2.2, 0.0),
    ];
    let chain = ChainField::from_polyline_sharp(&points, true, &[1.0; 5], 60.0).expect("a chain");
    let at_zero = tangent_at(&chain, 0.0);
    for parameter in [
        chain.length.next_down(),
        chain.length,
        chain.length.next_up(),
    ] {
        assert!((tangent_at(&chain, parameter) - at_zero).length() < 1e-9);
    }
}

/// A row from just before a joint to just after it, drawn by `along_the_chain` and by the plain
/// `build` about the joint: their interior points.
fn row(turn_degrees: f64, sharp_turn_degrees: f64) -> (Vec<Vector3>, Vec<Vector3>) {
    let turn = turn_degrees * std::f64::consts::PI / 180.0;
    let second = Vector3::new(turn.cos(), turn.sin(), 0.0);
    let points = [
        Vector3::ZERO,
        Vector3::new(10.0, 0.0, 0.0),
        Vector3::new(10.0, 0.0, 0.0) + (second * 10.0),
    ];
    let chain =
        ChainField::from_polyline_sharp(&points, false, &[1.0, 1.0, 1.0], sharp_turn_degrees)
            .expect("a chain");
    let from = chain.point_at(9.5) + Vector3::new(0.0, 0.0, 1.0);
    let to = chain.point_at(10.5) + Vector3::new(-second.y, second.x, 0.0);

    let interior = |draw: &dyn Fn(&mut LevelLineMesh, &mut Vec<i32>) -> Vec<i32>| {
        let mut mesh =
            LevelLineMesh::from_triangles(&[from, to, Vector3::ZERO], &[(0, 1, 2)], None)
                .expect("a mesh");
        let mut added = Vec::new();
        let arc = draw(&mut mesh, &mut added);
        arc[1..arc.len() - 1]
            .iter()
            .map(|&v| mesh.position(v))
            .collect::<Vec<_>>()
    };

    let carried = interior(&|mesh, added| {
        along_the_chain(
            mesh,
            &chain,
            (0, 9.5, (9.5, 9.5), 0.0),
            (1, 10.5, (10.5, 10.5), 0.0),
            8,
            1e-9,
            added,
        )
    });
    let mean = interior(&|mesh, added| build(mesh, 0, 1, chain.point_at(10.0), 8, added));
    (carried, mean)
}

fn gap(a: &[Vector3], b: &[Vector3]) -> f64 {
    a.iter()
        .zip(b)
        .map(|(p, q)| (*p - *q).length())
        .fold(f64::NEG_INFINITY, ieee_math::max_f64)
}
