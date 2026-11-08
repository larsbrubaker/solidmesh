// Rust-only pins for the surface part builder's whole build (`build`, `try_together`), driven
// without a rebuilding Fillet or Chamfer: a cube's rim built through the one
// pass and carried out by the item matrix, a group never offered refused with its reason, a size that
// does not fit refused with a row and no mesh, a cancelled rebuild leaving by its error, and a
// declined build refused by name.

use std::rc::Rc;
use std::sync::Arc;

use solidmesh::bevel::part::surface_part_builder::{
    BevelSurfacePartBuilder, SurfacePartBuildError,
};
use solidmesh::bevel::surface::{surface_mesh_closure, SurfaceBuildFailure};
use solidmesh::bevel::{BevelAdvisory, BevelAdvisoryCode, BevelRebuildAdvisories};
use solidmesh::topology::{BevelEdgeGroup, BevelEdgeGroupResolution, FeatureEdgeSet};
use solidmesh_math::Color;
use solidmesh_math::Matrix4X4;
use solidmesh_mesh::solids::platonic_solids::create_cube;
use solidmesh_mesh::CancelToken;

/// A 10 mm cube with its top rim picked as group 0 and its bottom rim as group 1; only the groups
/// in `offered` are offered.
fn cube_builder(size: f64, offered: [bool; 2], item_matrix: Matrix4X4) -> BevelSurfacePartBuilder {
    let mesh = create_cube(10.0, 10.0, 10.0);
    let features = Arc::new(FeatureEdgeSet::create(&mesh));
    let resolution = |top: bool| BevelEdgeGroupResolution {
        group: BevelEdgeGroup::default(),
        features: features.clone(),
        failure_reason: None,
        seed_edge_index: 0,
        edges: features
            .edges()
            .iter()
            .filter(|edge| {
                if top {
                    edge.start.z > 0.0 && edge.end.z > 0.0
                } else {
                    edge.start.z < 0.0 && edge.end.z < 0.0
                }
            })
            .copied()
            .collect(),
        chain: None,
        face_group: None,
    };
    let graph = mesh.get_mesh_edge_graph();

    BevelSurfacePartBuilder::new(
        vec![BevelEdgeGroup::default(), BevelEdgeGroup::default()],
        vec![resolution(true), resolution(false)],
        offered.to_vec(),
        Rc::new(BevelRebuildAdvisories::new(None::<Vec<_>>)),
        Arc::new(mesh),
        graph,
        size,
        8,
        true,
        item_matrix,
        Color::default(),
        None,
        None,
        None,
    )
}

#[test]
fn rust_only_a_cube_rim_builds_closed_and_is_carried_out_by_the_item_matrix() {
    let mut builder = cube_builder(
        1.0,
        [true, false],
        Matrix4X4::create_translation(100.0, 0.0, 0.0),
    );

    let part = builder
        .build(&CancelToken::new())
        .expect("nothing cancelled it");
    let mesh = part
        .mesh
        .expect("a 1 mm fillet on a 10 mm cube's top rim builds");

    assert!(
        surface_mesh_closure::is_closed(&mesh).0,
        "the built part is closed"
    );
    let bounds = mesh.get_axis_aligned_bounding_box();
    assert!(
        (bounds.min_xyz.x - 95.0).abs() < 1e-4 && (bounds.max_xyz.x - 105.0).abs() < 1e-4,
        "the part is carried out by the item matrix: {bounds:?}"
    );

    let built = part.group_results[0]
        .as_ref()
        .expect("the built group has a row");
    assert!(!built.failed(), "{:?}", built.failure);
    assert!(built.loop_count >= 1);

    // The group never offered is refused in words, as the boolean path refuses it.
    let skipped = part.group_results[1]
        .as_ref()
        .expect("the skipped group has a row");
    assert!(skipped.failed());
    assert_eq!(
        skipped.failure.as_ref().map(|advisory| advisory.code()),
        Some(BevelAdvisoryCode::NoEdgesResolved)
    );
}

#[test]
fn rust_only_a_size_that_does_not_fit_is_refused_with_a_row_and_no_mesh() {
    let mut builder = cube_builder(20.0, [true, false], Matrix4X4::IDENTITY);

    let part = builder
        .build(&CancelToken::new())
        .expect("nothing cancelled it");

    assert!(
        part.mesh.is_none(),
        "a 20 mm fillet does not fit a 10 mm cube"
    );
    let refused = part.group_results[0]
        .as_ref()
        .expect("the refused group has a row");
    assert!(refused.failed());
    assert!(refused.failure.is_some());
    if let Some(largest) = refused.largest_size_that_fits {
        assert!(largest > 0.0 && largest < 20.0, "offered {largest}");
    }
}

#[test]
fn rust_only_a_cancelled_rebuild_leaves_by_its_error() {
    let mut builder = cube_builder(1.0, [true, true], Matrix4X4::IDENTITY);
    let token = CancelToken::new();
    token.cancel();

    assert!(matches!(
        builder.build(&token),
        Err(SurfacePartBuildError::Cancelled)
    ));
}

/// A sections build that declines with nothing named is refused as the construction not building,
/// naming no edge: a decline is an ordinary answer, and the user is owed a sentence rather than a
/// failed rebuild.
#[test]
fn rust_only_a_declined_build_is_refused_as_not_built() {
    let declined =
        BevelSurfacePartBuilder::the_refusal_a_build_gives(&SurfaceBuildFailure::Declined);

    assert_eq!(
        declined.code(),
        BevelAdvisoryCode::SurfaceConstructionNotBuilt
    );
    assert_eq!(declined.mesh_edge_index(), BevelAdvisory::NO_MESH_EDGE);

    let named = BevelAdvisory::new(BevelAdvisoryCode::SurfaceResultNotClosed, vec![]);
    assert_eq!(
        BevelSurfacePartBuilder::the_refusal_a_build_gives(&SurfaceBuildFailure::Refused(
            named.clone()
        )),
        named
    );
}
