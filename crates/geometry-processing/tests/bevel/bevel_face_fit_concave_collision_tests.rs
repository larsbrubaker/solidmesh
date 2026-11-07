//! Two beads meeting across a narrow slot floor are admitted; a round and a bead sharing a face
//! still collide.

use std::rc::Rc;
use std::sync::Arc;

use geometry_processing::bevel::bevel_face_fit::DEFAULT_SURFACE_TOLERANCE_DEGREES;
use geometry_processing::bevel::{BevelAdvisoryCode, BevelFaceFit};
use geometry_processing::topology::{
    bevel_edge_group_resolver, BevelEdgeGroup, BevelSelectionKind, FaceGroup, FeatureEdgeSet,
};
use polygon_mesh::Mesh;
use vector_math::Vector3;

use crate::topology_test_meshes::create_grid_solid;

const RADIUS: f64 = 5.0;
const HALF: f64 = 10.0;
const SLOT_HALF_WIDTH: f64 = 2.0;
const FLOOR_Z: f64 = 2.0;

fn slotted_block() -> Mesh {
    create_grid_solid(
        &[-HALF, -SLOT_HALF_WIDTH, SLOT_HALF_WIDTH, HALF],
        &[-HALF, HALF],
        &[-HALF, FLOOR_Z, HALF],
        |i, j, k| (0..=2).contains(&i) && j == 0 && (0..=1).contains(&k) && !(i == 1 && k == 1),
    )
}

fn fit_of(block: Mesh, groups: &[BevelEdgeGroup]) -> BevelFaceFit {
    let (resolutions, _) = bevel_edge_group_resolver::resolve_all(groups, &block);
    BevelFaceFit::create(
        Arc::new(block),
        Rc::new(resolutions),
        true,
        RADIUS,
        DEFAULT_SURFACE_TOLERANCE_DEGREES,
        None,
    )
}

/// Two beads meeting across a narrow face are admitted.
#[test]
fn two_beads_meeting_across_a_narrow_face_are_admitted() {
    let block = slotted_block();
    assert!(block.is_manifold());
    let features = FeatureEdgeSet::create(&block);
    let groups = [
        group(
            &features,
            Vector3::new(-SLOT_HALF_WIDTH, -HALF, FLOOR_Z),
            Vector3::new(-SLOT_HALF_WIDTH, HALF, FLOOR_Z),
        ),
        group(
            &features,
            Vector3::new(SLOT_HALF_WIDTH, -HALF, FLOOR_Z),
            Vector3::new(SLOT_HALF_WIDTH, HALF, FLOOR_Z),
        ),
    ];
    let fit = fit_of(block, &groups);
    for i in 0..2 {
        assert!(
            fit.try_check_group(i).is_ok(),
            "two fills cannot undercut each other: {:?}",
            fit.try_check_group(i)
        );
    }
}

/// A round and a bead sharing a face still collide.
#[test]
fn a_round_and_a_bead_sharing_a_face_still_collide() {
    let block = slotted_block();
    let features = FeatureEdgeSet::create(&block);
    let groups = [
        group(
            &features,
            Vector3::new(-SLOT_HALF_WIDTH, -HALF, FLOOR_Z),
            Vector3::new(-SLOT_HALF_WIDTH, HALF, FLOOR_Z),
        ),
        group(
            &features,
            Vector3::new(-SLOT_HALF_WIDTH, -HALF, HALF),
            Vector3::new(-SLOT_HALF_WIDTH, HALF, HALF),
        ),
    ];
    let fit = fit_of(block, &groups);
    let reason = fit
        .try_check_group(0)
        .expect_err("one of them cuts, so they collide");
    assert_eq!(reason.code(), BevelAdvisoryCode::SizeCollidesAcrossFace);
}

fn group(features: &FeatureEdgeSet, from: Vector3, to: Vector3) -> BevelEdgeGroup {
    let i = features
        .edges()
        .iter()
        .position(|edge| {
            ((edge.start - from).length() < 1e-5 && (edge.end - to).length() < 1e-5)
                || ((edge.start - to).length() < 1e-5 && (edge.end - from).length() < 1e-5)
        })
        .unwrap_or_else(|| panic!("the fixture has no feature edge from {from:?} to {to:?}"));
    bevel_edge_group_resolver::create_from_edge(
        features,
        i as i32,
        BevelSelectionKind::Edge,
        FaceGroup::DEFAULT_CURVE_TOLERANCE_DEGREES,
    )
    .unwrap()
}
