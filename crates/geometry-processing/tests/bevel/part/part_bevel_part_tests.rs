//! The one entry point for a part's fillet or chamfer (`bevel::part::bevel_part`), asked as a
//! library user asks it: one cube edge built and carried out to the operation, a point pick
//! judged by the Point groups, the bar told its phases on one forward-only bar, a cancelled
//! build answered as a cancel, and the rows a build that could not run is shown with.

use std::sync::{Arc, Mutex};

use solidmesh::bevel::part::{
    bevel_part, BevelPartError, BevelPartOutcome, BevelPartRequest, BevelProgress,
    BevelProgressReport,
};
use solidmesh::bevel::BevelAdvisoryCode;
use solidmesh::topology::bevel_edge_group_resolver::create_groups_for_all_edges;
use solidmesh::topology::{BevelEdgeGroup, BevelSelectionKind};
use solidmesh_math::{Matrix4X4, Vector3};
use solidmesh_mesh::solids::platonic_solids::create_cube;
use solidmesh_mesh::{CancelToken, Mesh};

/// The cube's first straight crease as one Edge group.
fn one_edge(cube: &Mesh) -> Vec<BevelEdgeGroup> {
    let groups = create_groups_for_all_edges(cube, 20.0, None).expect("a cube has edges");
    vec![groups[0].clone()]
}

/// A 1 mm fillet of `groups` on a 10 mm cube centered on its origin.
fn fillet_on_a_cube(groups: impl FnOnce(&Mesh) -> Vec<BevelEdgeGroup>) -> BevelPartRequest {
    let cube = create_cube(10.0, 10.0, 10.0);
    let groups = groups(&cube);
    BevelPartRequest::new(Some(Arc::new(cube)), groups, 1.0, 4, true, 20.0)
}

/// One cube edge builds: a mesh, carried out by the request's translation, and a row that built.
#[test]
fn a_cube_edge_is_filleted_and_carried_out_to_the_operation() {
    let request = BevelPartRequest {
        source_to_operation: Matrix4X4::create_translation(125.0, -40.0, 3.0),
        ..fillet_on_a_cube(one_edge)
    };

    let outcome = bevel_part(request, &CancelToken::new(), None).expect("the build runs");

    let row = outcome.group_results[0]
        .as_ref()
        .expect("the group has its row");
    assert!(!row.failed(), "{:?}", row.failure);
    assert!(row.edge_count > 0);
    let mesh = outcome.mesh.expect("a filleted edge is a mesh");
    let bounds = mesh.get_axis_aligned_bounding_box();
    assert!((bounds.min_xyz.x - 120.0).abs() < 1e-4, "{bounds:?}");
    assert!((bounds.max_xyz.x - 130.0).abs() < 1e-4, "{bounds:?}");
    assert!(outcome.failures.is_none());
}

/// A pick in Point mode is judged by the Point groups and gets its own row: a cube's corner, whose
/// three edges are sharp, is refused as such, and a point in the middle of a face is no corner at
/// all. Neither builds, so the part answers no mesh.
#[test]
fn a_point_pick_is_judged_by_the_point_groups() {
    let point = |at: Vector3| BevelEdgeGroup {
        selection_kind: BevelSelectionKind::Point,
        anchor_midpoint: at,
        ..BevelEdgeGroup::default()
    };
    let request = fillet_on_a_cube(|_| {
        vec![
            point(Vector3::new(5.0, 5.0, 5.0)),
            point(Vector3::new(0.0, 0.0, 5.0)),
        ]
    });

    let outcome = bevel_part(request, &CancelToken::new(), None).expect("the build runs");

    let codes: Vec<Option<BevelAdvisoryCode>> = outcome
        .group_results
        .iter()
        .map(|row| {
            row.as_ref()
                .and_then(|row| row.failure.as_ref())
                .map(|failure| failure.code())
        })
        .collect();
    assert_eq!(
        vec![
            Some(BevelAdvisoryCode::PointHasSharpEdges),
            Some(BevelAdvisoryCode::PointIsNoLongerACorner)
        ],
        codes
    );
    assert!(outcome.mesh.is_none());
}

/// The bar a build reports on only moves forward, and ends at its end.
#[test]
fn the_progress_only_moves_forward_and_ends_at_the_end() {
    let ratios = Arc::new(Mutex::new(Vec::new()));
    let told = Arc::clone(&ratios);
    let progress: BevelProgressReport = Arc::new(move |ratio: f64, _: &BevelProgress| {
        told.lock().expect("the test's own lock").push(ratio)
    });

    bevel_part(
        fillet_on_a_cube(one_edge),
        &CancelToken::new(),
        Some(progress),
    )
    .expect("the build runs");

    let ratios = ratios.lock().expect("the test's own lock");
    assert!(!ratios.is_empty());
    assert!(
        ratios.windows(2).all(|pair| pair[0] <= pair[1]),
        "{ratios:?}"
    );
    assert_eq!(Some(&1.0), ratios.last());
}

/// A build whose token is cancelled answers the cancel, not an outcome.
#[test]
fn a_cancelled_build_answers_the_cancel() {
    let cancel = CancelToken::new();
    cancel.cancel();

    let answer = bevel_part(fillet_on_a_cube(one_edge), &cancel, None);

    assert!(
        matches!(answer, Err(BevelPartError::Cancelled)),
        "{answer:?}"
    );
}

/// The rows to show for a build that could not run: every group refused, nothing built.
#[test]
fn a_build_that_could_not_run_refuses_every_group() {
    let cube = create_cube(10.0, 10.0, 10.0);
    let groups = create_groups_for_all_edges(&cube, 20.0, None).expect("edges");

    let outcome = BevelPartOutcome::every_group_refused(&groups[..3]);

    assert!(outcome.mesh.is_none());
    assert_eq!(3, outcome.group_results.len());
    for (i, row) in outcome.group_results.iter().enumerate() {
        let row = row.as_ref().expect("every group has its row");
        assert_eq!(i as i32, row.group_index);
        assert_eq!(
            Some(BevelAdvisoryCode::ConstructionCouldNotBuildThisGroup),
            row.failure.as_ref().map(|failure| failure.code())
        );
    }
}
