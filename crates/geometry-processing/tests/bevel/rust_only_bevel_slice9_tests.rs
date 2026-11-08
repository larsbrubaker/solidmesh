//! Rust-only tests for bevel slice 9: the rebuild's per-group rows (`BevelGroupResult`,
//! `bevel_group_failures`) and the run-off readings (`surface_band_runs_off_the_face_beyond`).
//! The room measure and the operation exercise them in context (`SurfaceRoomTests`,
//! `BevelGroupResolutionAdvisoryTests`, ...); these hold the pieces to their own contracts.
//! `SurfaceCutClip` has its ported class in `surface_facet_cut_own_ends_tests.rs`.

use std::collections::HashSet;

use geometry_processing::bevel::surface::surface_band_runs_off_the_face_beyond::{
    self as run_off, BandSpan,
};
use geometry_processing::bevel::{
    bevel_group_failures, BevelAdvisory, BevelAdvisoryCode, BevelGroupResult, BevelLoopFailure,
    BevelResultGroup,
};
use geometry_processing::topology::{
    bevel_edge_group_resolver, BevelEdgeGroup, BevelSelectionKind, FaceGroup, FeatureEdgeSet,
};
use polygon_mesh::solids::platonic_solids;
use vector_math::Vector3;

#[test]
fn rust_only_a_row_says_whether_it_failed_partly_refused_or_warned() {
    let group = BevelResultGroup::Edge(BevelEdgeGroup::default());

    let built = BevelGroupResult::new(0, group.clone(), 4, None, Vec::new());
    assert!(!built.failed() && !built.partly_refused() && !built.warned());

    let failed = BevelGroupResult::new(1, group.clone(), 0, Some(refusal()), Vec::new());
    assert!(failed.failed() && !failed.partly_refused());

    let held = BevelAdvisory::new(BevelAdvisoryCode::SurfaceRunStopsSquareAtOneEnd, vec![]);
    let mut partly = BevelGroupResult::new(2, group, 12, None, vec![held]);
    partly.loop_failures = Some(vec![BevelLoopFailure {
        advisory: refusal(),
        largest_size_that_fits: None,
        edge_count: 3,
    }]);
    assert!(partly.partly_refused() && partly.warned());
}

/// A refusal that quotes nothing.
fn refusal() -> BevelAdvisory {
    BevelAdvisory::new(BevelAdvisoryCode::SizeRemovesNothing, vec![])
}

#[test]
fn rust_only_a_chain_that_grew_past_its_pick_keeps_its_refusal() {
    // A cube's edge picked as a tangent chain stops at real corners at every rung, so the
    // refusal the geometry gave stands.
    let cube = platonic_solids::create_cube(10.0, 10.0, 10.0);
    let features = FeatureEdgeSet::create(&cube);
    let seed = features
        .edges()
        .iter()
        .position(|edge| {
            let middle = (edge.start + edge.end) * 0.5;
            (middle - Vector3::new(0.0, -5.0, 5.0)).length() < 1e-9
        })
        .expect("the cube has a front top edge") as i32;
    let group = bevel_edge_group_resolver::create_from_edge(
        &features,
        seed,
        BevelSelectionKind::TangentChain,
        FaceGroup::DEFAULT_CURVE_TOLERANCE_DEGREES,
    )
    .expect("the seed is an edge");
    let resolution = bevel_edge_group_resolver::resolve(&group, &cube);
    assert_eq!(
        resolution.chain.as_ref().map(|chain| chain.count()),
        Some(1)
    );
    let refusal = BevelAdvisory::new(BevelAdvisoryCode::SizeRemovesNothing, vec![]);

    assert_eq!(
        bevel_group_failures::diagnose(&resolution, Some(refusal.clone())),
        Some(refusal)
    );
    assert_eq!(bevel_group_failures::diagnose(&resolution, None), None);
}

/// A 10 x 2 x 2 box's top face and its front top edge, as a band span with `setback`.
fn box_top_band(
    setback: f64,
) -> (
    polygon_mesh::Mesh,
    FaceGroup,
    BandSpan,
    HashSet<i32>,
    HashSet<i32>,
) {
    let mesh = platonic_solids::create_cube(10.0, 2.0, 2.0);
    let top = (0..mesh.faces.len() as i32)
        .find(|&face| mesh.faces[face as usize].normal.z > 0.9)
        .expect("a box has a top");
    let region = FaceGroup::create(&mesh, top).expect("the top floods");
    let front = region
        .boundary_edges()
        .iter()
        .find(|rim| rim.start.y < -0.9 && rim.end.y < -0.9)
        .copied()
        .expect("the top has a front rim");
    let from = if front.start.x < front.end.x {
        front.start
    } else {
        front.end
    };
    let span = BandSpan {
        from,
        along: Vector3::new(1.0, 0.0, 0.0),
        length: 10.0,
        inward: Vector3::new(0.0, 1.0, 0.0),
        setback,
    };

    (
        mesh,
        region,
        span,
        HashSet::from([front.mesh_edge_index]),
        HashSet::from([front.start_vertex, front.end_vertex]),
    )
}

#[test]
fn rust_only_a_band_exactly_as_deep_as_the_face_consumes_it_to_its_rim() {
    for (setback, consumed) in [(2.0, true), (1.5, false), (3.0, false)] {
        let (_mesh, region, span, beveled, chain) = box_top_band(setback);
        assert_eq!(
            run_off::consumes_the_face_to_its_rim(Some(&region), &span, &beveled, &chain, 1e-4),
            consumed,
            "setback {setback}"
        );
    }
}

#[test]
fn rust_only_a_band_on_a_face_it_does_not_take_whole_is_no_run_off() {
    // Shallower than the face: a landing, and nothing runs off.
    let (mesh, region, span, beveled, chain) = box_top_band(1.0);
    assert_eq!(
        run_off::try_read(Some(&mesh), Some(&region), &span, &beveled, &chain),
        Ok(None)
    );

    // Deeper than the face: the tangent line meets the far planes only at the span's own ends or
    // runs parallel to the back face, so this is not the oblique run-off either.
    let (mesh, region, span, beveled, chain) = box_top_band(3.0);
    assert_eq!(
        run_off::try_read(Some(&mesh), Some(&region), &span, &beveled, &chain),
        Ok(None)
    );

    // And no region at all reads nothing.
    assert_eq!(
        run_off::try_read(Some(&mesh), None, &span, &beveled, &chain),
        Ok(None)
    );
}
