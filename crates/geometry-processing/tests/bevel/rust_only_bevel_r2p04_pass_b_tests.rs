//! Rust-only checks of the bevel core's R2.P04 readings after the side groups: the step corner's
//! carried piece, and pass B's creases and feature edges off the rounds' mesh. The side tagging
//! and flows are in `rust_only_bevel_r2p04_flow_tests.rs`.

use std::rc::Rc;

use vector_math::Vector3;

#[test]
fn rust_only_the_larger_setback_piece_is_carried_past_the_step() {
    use geometry_processing::bevel::level_line::band_side_groups::group_sides;
    use geometry_processing::bevel::level_line::step_corner_carry::join_the_larger_setback_piece;
    use std::collections::HashSet;
    let positions = vec![
        Vector3::new(0.0, 0.0, 1.0),
        Vector3::new(1.0, 0.0, 1.0),
        Vector3::new(1.0, 0.5, 1.0),
        Vector3::new(1.0, 0.0, 0.0),
    ];
    let indices = vec![(0, 1, 2), (1, 0, 3), (1, 3, 2)];
    let folds = |a: i32, b: i32| a != b;
    let chain_edges = [vec![(0, 1)], vec![(1, 3)]];
    let (tags, mut groups, _) =
        group_sides(&indices, &chain_edges, &folds, false, None, None, None).expect("three groups");
    let setbacks = [vec![1.0, 2.0], vec![1.0, 1.0]];
    let step: HashSet<i32> = [1].into_iter().collect();
    join_the_larger_setback_piece(
        &positions,
        &indices,
        &chain_edges,
        &setbacks,
        &tags,
        &mut groups,
        &step,
    );
    // Piece 0 sets back more at the step: it joins piece 1's own side group, carried there once.
    assert_eq!(groups[2].members, vec![0, 1]);
    assert_eq!(groups[2].side_of_chain.get(&0), Some(&3));
    assert_eq!(groups[2].carried_ends.get(&0), Some(&1));
    // The group both already share is not touched; a second pass counts a second carried end.
    assert!(groups[1].carried_ends.is_empty());
    join_the_larger_setback_piece(
        &positions,
        &indices,
        &chain_edges,
        &setbacks,
        &tags,
        &mut groups,
        &step,
    );
    assert_eq!(groups[2].carried_ends.get(&0), Some(&2));
    // No step corner, or a step within the fraction, carries nothing.
    let (tags, mut plain, _) =
        group_sides(&indices, &chain_edges, &folds, false, None, None, None).expect("three groups");
    let close = [vec![1.0, 1.04], vec![1.0, 1.0]];
    join_the_larger_setback_piece(
        &positions,
        &indices,
        &chain_edges,
        &close,
        &tags,
        &mut plain,
        &step,
    );
    join_the_larger_setback_piece(
        &positions,
        &indices,
        &chain_edges,
        &setbacks,
        &tags,
        &mut plain,
        &HashSet::new(),
    );
    assert!(plain.iter().all(|g| g.carried_ends.is_empty()));
}

#[test]
fn rust_only_pass_bs_creases_and_feature_edges_off_the_rounds() {
    use std::cell::RefCell;

    use geometry_processing::bevel::level_line::band_gap_after_the_rounds::{
        creases_after_the_rounds, feature_edges_after_the_rounds,
    };
    use geometry_processing::bevel::level_line::{GapFillResult, LevelLineMesh, LevelLineTriangle};
    let input = vec![
        Vector3::new(0.0, 0.0, 0.0),
        Vector3::new(1.0, 0.0, 0.0),
        Vector3::new(0.0, 1.0, 0.0),
    ];
    let input_indices = vec![(0, 1, 2)];
    let mut mesh = LevelLineMesh::new();
    mesh.vertices = vec![
        Vector3::new(0.0, 0.0, 0.0),
        Vector3::new(1.0, 0.0, 0.0),
        Vector3::new(0.0, 1.0, 0.0),
        Vector3::new(1.0, 1.0, 1.0),
    ];
    for (i, t) in [
        LevelLineTriangle::new(0, 1, 2, 0),
        LevelLineTriangle::new(1, 3, 2, 0),
    ]
    .into_iter()
    .enumerate()
    {
        mesh.triangles.push(t);
        mesh.removed.push(false);
        mesh.attach(i as i32, t);
    }

    // No knife tip: there are no creases to read again.
    assert!(creases_after_the_rounds(&input, &input_indices, &[vec![0, 1]], None, &mesh).is_none());

    let rounds = GapFillResult {
        mesh: Some(Rc::new(RefCell::new(mesh))),
        added_triangles: vec![0, 1],
        ..GapFillResult::default()
    };
    let only_0_1 = |a: i32, b: i32| (a.min(b), a.max(b)) == (0, 1);
    let feature = feature_edges_after_the_rounds(
        &input,
        &input_indices,
        &[vec![0, 1]],
        None,
        Some(Rc::new(only_0_1)),
        &rounds,
    )
    .expect("a reading");
    // The input's own edges keep their word; an edge of the rounds on one triangle is no feature.
    assert!(feature(0, 1));
    assert!(feature(1, 0));
    assert!(!feature(1, 2));
    assert!(!feature(1, 3));
    // Without a rounds mesh there is nothing to read.
    assert!(feature_edges_after_the_rounds(
        &input,
        &input_indices,
        &[vec![0, 1]],
        None,
        None,
        &GapFillResult::default(),
    )
    .is_none());
}
