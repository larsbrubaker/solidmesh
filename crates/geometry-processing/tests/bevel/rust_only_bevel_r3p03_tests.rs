//! Rust-only checks of the bevel core's R3.P03 fill pieces on hand-built meshes: the uneven strip's
//! zip. The tests that read these pieces in a whole build are in their own files.

use solidmesh::bevel::level_line::uneven_strip::cell;
use solidmesh::bevel::level_line::LevelLineMesh;
use solidmesh_math::Vector3;

#[test]
fn rust_only_an_uneven_cell_zips_its_arcs_by_length_share() {
    // u: two points along y = 0; v: three along y = 1, its middle a quarter of the way along.
    let mut mesh = LevelLineMesh::new();
    mesh.vertices = vec![
        Vector3::new(0.0, 0.0, 0.0),
        Vector3::new(4.0, 0.0, 0.0),
        Vector3::new(0.0, 1.0, 0.0),
        Vector3::new(1.0, 1.0, 0.0),
        Vector3::new(4.0, 1.0, 0.0),
    ];
    mesh.descends_from = vec![(-1, -1); 5];
    let mut added = Vec::new();
    cell(&mut mesh, &[0, 1], &[2, 3, 4], &mut added).expect("laid");
    // Three triangles: v's quarter comes before u's end, then u, then v's end.
    assert_eq!(added.len(), 3);
    let corners: Vec<(i32, i32, i32)> = added
        .iter()
        .map(|&t| {
            let t = mesh.triangles()[t as usize];
            (t.a, t.b, t.c)
        })
        .collect();
    assert_eq!(corners, vec![(0, 2, 3), (0, 3, 1), (1, 3, 4)]);
}

#[test]
fn rust_only_a_long_rail_edge_is_given_the_other_rails_stations() {
    use solidmesh::bevel::level_line::rail_stations::densify;
    use solidmesh::bevel::level_line::ChainField;
    use std::collections::HashMap;
    let chain = ChainField::from_polyline(
        &[Vector3::new(0.0, 0.0, 0.0), Vector3::new(10.0, 0.0, 0.0)],
        false,
        &[1.0, 1.0],
    )
    .expect("a chain");
    let mut mesh = LevelLineMesh::from_triangles(
        &[
            Vector3::new(0.0, 1.0, 0.0),
            Vector3::new(10.0, 1.0, 0.0),
            Vector3::new(5.0, 3.0, 0.0),
        ],
        &[(0, 1, 2)],
        None,
    )
    .expect("a triangle");
    let mut vertices = vec![0, 1];
    let mut parameters = vec![0.0, 10.0];
    let mut splits = HashMap::new();
    let mut jump_ends = HashMap::new();
    let refused = densify(
        &mut mesh,
        &chain,
        &mut vertices,
        &mut parameters,
        &[2.0, 5.0, 8.0],
        1e-9,
        &mut splits,
        &mut jump_ends,
        2,
    )
    .expect("densified");
    assert_eq!(refused, -1);
    assert_eq!(vertices, vec![0, 3, 4, 5, 1]);
    for (k, x) in [(1, 2.0), (2, 5.0), (3, 8.0)] {
        assert!((parameters[k] - x).abs() <= 1e-6);
        assert!((mesh.position(vertices[k]) - Vector3::new(x, 1.0, 0.0)).length() <= 1e-6);
    }
    assert_eq!(splits.len(), 3);
    assert!(jump_ends.is_empty());

    // Spanning only one of the other rail's vertices is ordinary interleaving: left alone.
    let mut few = vec![0, 3];
    let mut few_parameters = vec![0.0, parameters[1]];
    assert_eq!(
        densify(
            &mut mesh,
            &chain,
            &mut few,
            &mut few_parameters,
            &[1.0],
            1e-9,
            &mut splits,
            &mut jump_ends,
            2
        )
        .ok(),
        Some(-1)
    );
    assert_eq!(few, vec![0, 3]);
}

#[test]
fn rust_only_a_strip_end_is_set_back_to_where_its_chain_ties_the_next() {
    use solidmesh::bevel::level_line::corner_rail_strip::{set_back, WATCH_SETBACKS};
    use solidmesh::bevel::level_line::ChainField;
    use std::cell::RefCell;
    use std::collections::HashMap;
    use std::rc::Rc;
    // Chain 0 along x to (10, 0), chain 1 up from there: their fields tie on the corner's mitre.
    let leg = |from: Vector3, to: Vector3| {
        Rc::new(ChainField::from_polyline(&[from, to], false, &[1.0, 1.0]).expect("a chain"))
    };
    let chains = vec![
        leg(Vector3::new(0.0, 0.0, 0.0), Vector3::new(10.0, 0.0, 0.0)),
        leg(Vector3::new(10.0, 0.0, 0.0), Vector3::new(10.0, 10.0, 0.0)),
    ];
    let mut mesh = LevelLineMesh::from_triangles(
        &[
            Vector3::new(0.0, 1.0, 0.0),
            Vector3::new(9.0, 1.0, 0.0),
            Vector3::new(0.0, -1.0, 0.0),
            Vector3::new(10.0, -1.0, 0.0),
        ],
        &[(2, 3, 1), (2, 1, 0)],
        None,
    )
    .expect("a mesh");
    let seen = Rc::new(RefCell::new(Vec::new()));
    let log = seen.clone();
    WATCH_SETBACKS.with(|w| {
        *w.borrow_mut() = Some(Rc::new(move |densified| log.borrow_mut().push(densified)))
    });

    // Both last ends tie: the setback is the lesser, 9, and the rail running on to 10 is given a
    // station there.
    let (mut av, mut ap) = (vec![0, 1], vec![0.0, 9.0]);
    let (mut bv, mut bp) = (vec![2, 3], vec![0.0, 10.0]);
    let mut splits = HashMap::new();
    let mut jump_ends = HashMap::new();
    let at = set_back(
        &mut mesh,
        &chains,
        0,
        &mut av,
        &mut ap,
        &mut bv,
        &mut bp,
        1e-9,
        &mut splits,
        &mut jump_ends,
        false,
    )
    .expect("set back");
    WATCH_SETBACKS.with(|w| *w.borrow_mut() = None);
    assert!((at.expect("tied") - (9.0 + 1e-9)).abs() <= 1e-12);
    assert_eq!(bv.len(), 3);
    assert!((mesh.position(bv[1]) - Vector3::new(9.0, -1.0, 0.0)).length() <= 1e-6);
    assert_eq!(*seen.borrow(), vec![true]);

    // Neither first end ties: no setback.
    assert_eq!(
        set_back(
            &mut mesh,
            &chains,
            0,
            &mut av,
            &mut ap,
            &mut bv,
            &mut bp,
            1e-9,
            &mut splits,
            &mut jump_ends,
            true,
        )
        .ok(),
        Some(None)
    );
}

#[test]
fn rust_only_a_run_is_cut_at_its_end_parameter() {
    use solidmesh::bevel::level_line::end_plane_trim::cut_at;
    use solidmesh::bevel::level_line::GapLoop;
    let mut mesh = LevelLineMesh::from_triangles(
        &[
            Vector3::new(0.0, 0.0, 0.0),
            Vector3::new(10.0, 0.0, 0.0),
            Vector3::new(5.0, 3.0, 0.0),
        ],
        &[(0, 1, 2)],
        None,
    )
    .expect("a triangle");
    let run = GapLoop::new(1, false, vec![0, 1], vec![0.0, 10.0], 0, false, None);

    // At its head: split at 4, keeping 4..10, the stretch from the cut back to the head.
    let (kept, stretch) = cut_at(&mut mesh, &run, 4.0, true)
        .expect("no budget")
        .expect("a cut");
    assert_eq!(kept.vertices, vec![3, 1]);
    assert_eq!(kept.parameters, vec![4.0, 10.0]);
    assert_eq!(stretch, vec![3, 0]);
    assert!((mesh.position(3) - Vector3::new(4.0, 0.0, 0.0)).length() <= 1e-12);

    // At its tail, within placement noise of the far vertex: that vertex is the cut.
    let (kept, stretch) = cut_at(&mut mesh, &kept, 10.0 - 1e-7, false)
        .expect("no budget")
        .expect("a cut");
    assert_eq!(kept.vertices, vec![3, 1]);
    assert_eq!(stretch, vec![1]);

    // Off the run: refused.
    assert!(cut_at(&mut mesh, &run, 12.0, true)
        .expect("no budget")
        .is_none());
}

#[test]
fn rust_only_the_corner_fill_refuses_what_it_cannot_zip() {
    use solidmesh::bevel::level_line::corner_fill_fill::{fill, CornerFillError};
    use solidmesh::bevel::level_line::{BandGapResult, BandProfile, GapFillRefusal, GapLoop};
    use std::cell::RefCell;
    use std::rc::Rc;
    let mesh = Rc::new(RefCell::new(LevelLineMesh::new()));
    let gap = BandGapResult {
        mesh: Some(mesh),
        loops: vec![GapLoop::new(
            1,
            true,
            vec![0, 1, 2],
            vec![0.0, 1.0, 2.0],
            0,
            false,
            None,
        )],
        ..BandGapResult::default()
    };

    // A fillet of no segments is refused by name.
    assert!(matches!(
        fill(&gap, BandProfile::Fillet, 0),
        Err(CornerFillError::FilletSegments(_))
    ));

    // A loop whose vertices name no owner leaves no one loop per side.
    let refused = fill(&gap, BandProfile::Fillet, 4).expect("a verdict");
    assert_eq!(refused.refusal, GapFillRefusal::SideDoesNotLeaveOneLoop);
    assert!(refused.mesh.is_some());
}

#[test]
fn rust_only_an_end_plane_trim_with_no_end_face_says_why() {
    use solidmesh::bevel::level_line::end_plane_trim_run::{run, trim_end};
    use solidmesh::bevel::level_line::open_gap_fill::{ends_site, set_ends_site};
    use solidmesh::bevel::level_line::{BandProfile, ChainField, GapLoop};
    // A chain along x with no triangle standing at either end: no end plane to trim to.
    let chain = ChainField::from_polyline(
        &[Vector3::new(0.0, 0.0, 0.0), Vector3::new(4.0, 0.0, 0.0)],
        false,
        &[1.0, 1.0],
    )
    .expect("a chain");
    let mut mesh = LevelLineMesh::from_triangles(
        &[
            Vector3::new(1.0, 1.0, 0.0),
            Vector3::new(3.0, 1.0, 0.0),
            Vector3::new(2.0, 2.0, 0.0),
        ],
        &[(0, 1, 2)],
        None,
    )
    .expect("a triangle");
    set_ends_site(None);
    let mut left = GapLoop::new(1, false, vec![0, 1], vec![1.0, 3.0], 0, false, None);
    let mut right = GapLoop::new(2, false, vec![1, 0], vec![3.0, 1.0], 0, false, None);
    let trimmed = trim_end(
        &mut mesh,
        &chain,
        &mut left,
        &mut right,
        true,
        BandProfile::Fillet,
        4,
    )
    .expect("no budget");
    assert!(trimmed.is_none());
    assert_eq!(ends_site().as_deref(), Some(" / trim 1: no end plane"));

    set_ends_site(None);
    assert!(
        run(&mut mesh, &chain, 0, 1, false, BandProfile::Fillet, 4, 0)
            .expect("no budget")
            .is_none()
    );
    assert_eq!(ends_site().as_deref(), Some(" / section: no end plane"));
    set_ends_site(None);
}
