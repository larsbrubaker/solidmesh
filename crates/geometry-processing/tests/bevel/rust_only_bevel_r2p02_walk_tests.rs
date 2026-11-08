//! Rust-only checks of the bevel core's R2.P02 loop readings - a group loop split into its chains'
//! pieces, a run-out row cut by the face it ran onto, and a side's boundary walked into loops - on
//! the chain fields of `chain_field_fixtures.rs`. The other rail and fill readings are in
//! `rust_only_bevel_r2p02_fill_tests.rs`.

use std::rc::Rc;

use solidmesh::bevel::level_line::LevelLineMesh;
use solidmesh_math::Vector3;

use crate::chain_field_fixtures::open_chain;

#[test]
fn rust_only_a_mixed_loop_splits_into_pieces_that_take_in_tie_vertices() {
    use solidmesh::bevel::level_line::corner_fill::pieces;
    use solidmesh::bevel::level_line::GapLoop;
    let x_leg = Rc::new(open_chain(
        vec![Vector3::new(0.0, 0.0, 0.0), Vector3::new(4.0, 0.0, 0.0)],
        vec![1.0, 1.0],
    ));
    let y_leg = Rc::new(open_chain(
        vec![Vector3::new(0.0, 0.0, 0.0), Vector3::new(0.0, 4.0, 0.0)],
        vec![1.0, 1.0],
    ));
    let chains = vec![x_leg, y_leg];
    let mut mesh = LevelLineMesh::new();
    // (1, 1) ties the two legs' fields; the others read one leg each.
    mesh.vertices = vec![
        Vector3::new(3.0, 1.0, 0.0),
        Vector3::new(1.0, 1.0, 0.0),
        Vector3::new(1.0, 3.0, 0.0),
        Vector3::new(2.0, 3.0, 0.0),
    ];
    let mut gap_loop = GapLoop::new(
        1,
        true,
        vec![0, 1, 2, 3],
        vec![3.0, 1.0, 3.0, 2.0],
        0,
        true,
        None,
    );
    gap_loop.vertex_owners = Some(vec![0, 0, 1, 1]);
    let split = pieces(&mesh, &chains, &gap_loop).expect("no budget");
    assert_eq!(split.len(), 2);
    assert_eq!((split[0].owner, split[0].vertices.clone()), (0, vec![0, 1]));
    // The y leg's piece takes in the tie vertex (1, 1), read on its own chain at 1.
    assert_eq!(
        (split[1].owner, split[1].vertices.clone()),
        (1, vec![1, 2, 3])
    );
    assert_eq!(split[1].parameters, vec![1.0, 3.0, 2.0]);
    // No owners, no pieces.
    gap_loop.vertex_owners = None;
    assert!(pieces(&mesh, &chains, &gap_loop)
        .expect("no budget")
        .is_empty());
}

#[test]
fn rust_only_a_run_out_row_is_the_section_cut_by_the_face_it_ran_onto() {
    use solidmesh::bevel::level_line::open_gap_fill_cut_section::cut_section_arc;
    use solidmesh::bevel::level_line::{LevelLineTriangle, OpenGapFillOpenRail};
    let chain = open_chain(
        vec![Vector3::new(0.0, 0.0, 0.0), Vector3::new(10.0, 0.0, 0.0)],
        vec![1.0, 1.0],
    );
    // The left rail runs at z = -1 and ran out at A = (4, 0, -1) onto the face x - 2z = 6; the
    // right rail's B = (5, 1, 0) is a setback off the chain at parameter 5.
    let mut mesh = LevelLineMesh::new();
    mesh.vertices = vec![
        Vector3::new(3.0, 0.0, -1.0),
        Vector3::new(4.0, 0.0, -1.0),
        Vector3::new(6.0, 0.0, 0.0),
        Vector3::new(4.0, 1.0, -1.0),
        Vector3::new(5.0, 1.0, 0.0),
    ];
    let t = LevelLineTriangle::new(1, 2, 3, 0);
    mesh.triangles.push(t);
    mesh.removed.push(false);
    mesh.attach(0, t);
    let left = OpenGapFillOpenRail {
        vertices: vec![0, 1],
        parameters: vec![3.0, 4.0],
    };
    let right = OpenGapFillOpenRail {
        vertices: vec![4, 4],
        parameters: vec![4.0, 5.0],
    };
    let cut = cut_section_arc(&mesh, &chain, &left, 1, &right, 1).expect("no budget");
    assert!(!cut.in_the_plane);
    let row = cut.row.expect("the section is cut");
    // The round from the face's plane to B is the quarter circle about (5, 1, -1); it crosses the
    // plane at z = -0.5, and before that the row runs straight back to A.
    let on_plane = row(cut.cut_at);
    assert!((on_plane.z - -0.5).abs() <= 1e-9);
    assert!(((on_plane - Vector3::new(5.0, 1.0, -1.0)).length() - 1.0).abs() <= 1e-9);
    assert!((row(1.0) - Vector3::new(5.0, 1.0, 0.0)).length() <= 1e-12);
    assert!((row(0.0) - Vector3::new(4.0, 0.0, -1.0)).length() <= 1e-12);
}

#[test]
fn rust_only_a_sides_boundary_is_walked_into_one_closed_loop() {
    use solidmesh::bevel::level_line::gap_loop_walk::loops_of_side;
    use solidmesh::bevel::level_line::{BandGroup, FieldSample, LevelLineTriangle, WorkError};
    // A unit square of side 1's triangles: its boundary is one closed loop.
    let mut mesh = LevelLineMesh::new();
    mesh.vertices = vec![
        Vector3::new(0.0, 0.0, 0.0),
        Vector3::new(1.0, 0.0, 0.0),
        Vector3::new(1.0, 1.0, 0.0),
        Vector3::new(0.0, 1.0, 0.0),
    ];
    for (i, t) in [
        LevelLineTriangle::new(0, 1, 2, 1),
        LevelLineTriangle::new(0, 2, 3, 1),
    ]
    .into_iter()
    .enumerate()
    {
        mesh.triangles.push(t);
        mesh.removed.push(false);
        mesh.attach(i as i32, t);
    }

    let group = BandGroup {
        tag: 1,
        members: vec![0],
        side_of_chain: [(0, 1)].into_iter().collect(),
        ..BandGroup::default()
    };
    let sample = |p: Vector3| -> Result<FieldSample, WorkError> {
        Ok(FieldSample {
            parameter: p.x,
            chain: 0,
            ..FieldSample::default()
        })
    };
    let loops = loops_of_side(&mesh, &sample, &group, None, None, None, 0.0).expect("no budget");
    assert_eq!(loops.len(), 1);
    let gap_loop = &loops[0];
    assert!(gap_loop.is_closed && !gap_loop.has_mixed_owners);
    assert_eq!(gap_loop.vertices, vec![0, 1, 2, 3]);
    assert_eq!(gap_loop.parameters, vec![0.0, 1.0, 1.0, 0.0]);
    assert_eq!((gap_loop.side, gap_loop.group_tag), (1, 1));
    assert_eq!(gap_loop.vertex_owners, Some(vec![0; 4]));
    // Another side has no boundary here.
    let other = BandGroup {
        tag: 2,
        members: vec![0],
        side_of_chain: [(0, 2)].into_iter().collect(),
        ..BandGroup::default()
    };
    assert!(loops_of_side(&mesh, &sample, &other, None, None, None, 0.0)
        .expect("no budget")
        .is_empty());
}
