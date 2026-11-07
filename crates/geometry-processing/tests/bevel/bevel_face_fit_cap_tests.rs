//! A losing wall must use its own admission rule rather than
//! claim the run's cap.

use std::rc::Rc;
use std::sync::Arc;

use geometry_processing::bevel::bevel_chain_end_cap::ResolveOptions;
use geometry_processing::bevel::bevel_face_fit::DEFAULT_SURFACE_TOLERANCE_DEGREES;
use geometry_processing::bevel::{
    bevel_chain_linker, BevelAdvisoryArg, BevelAdvisoryCode, BevelChainEnds, BevelFaceFit,
};
use geometry_processing::topology::{
    bevel_edge_group_resolver, BevelSelectionKind, FaceGroup, FeatureEdgeSet,
};
use vector_math::Vector3;

use crate::topology_test_meshes::create_pyramid_pocket_prism;

/// A losing wall must use its own admission rule rather than claim the run's cap, for a round or
/// a chamfer profile.
fn a_losing_wall_must_use_its_own_admission_rule_rather_than_claim_the_runs_cap(
    round_profile: bool,
) {
    let apex = Vector3::new(2.0, 3.0, 5.0);
    let part = create_pyramid_pocket_prism(4, 10.0, 20.0, apex);
    let features = FeatureEdgeSet::create(&part);
    let rim = Vector3::new(10.0, 0.0, 20.0);
    let seeds: Vec<usize> = (0..features.count())
        .filter(|&i| {
            let edge = &features.edges()[i];
            ((edge.start - apex).length() < 1e-5 && (edge.end - rim).length() < 1e-5)
                || ((edge.end - apex).length() < 1e-5 && (edge.start - rim).length() < 1e-5)
        })
        .collect();
    assert_eq!(seeds.len(), 1);

    let group = bevel_edge_group_resolver::create_from_edge(
        &features,
        seeds[0] as i32,
        BevelSelectionKind::Edge,
        FaceGroup::DEFAULT_CURVE_TOLERANCE_DEGREES,
    )
    .unwrap();
    let resolution = bevel_edge_group_resolver::resolve_all(&[group], &part)
        .0
        .remove(0);
    let runs = bevel_chain_linker::link_resolution(&resolution);
    assert_eq!(runs.len(), 1);
    let run = &runs[0];
    let ends = BevelChainEnds::resolve(
        &run.edges,
        run.is_closed,
        3.0,
        Some(&resolution.features),
        Some(&resolution.edges),
        None,
        ResolveOptions {
            round_profile,
            ..Default::default()
        },
    );

    assert!(part.is_manifold());
    let apex_cap = if (run.edges[0].from() - apex).length() < 1e-5 {
        &ends.start
    } else {
        &ends.end
    };
    assert!((apex_cap.plane_normal - Vector3::new(-5.0, 5.0, -3.0).get_normal()).length() < 1e-6);

    let fit = BevelFaceFit::create(
        Arc::new(part),
        Rc::new(vec![resolution]),
        round_profile,
        3.0,
        DEFAULT_SURFACE_TOLERANCE_DEGREES,
        None,
    );
    let check = fit.try_check_group_fully(0);
    assert_eq!(
        check.fits(),
        round_profile,
        "fit={}, trimmed={}, reason={:?}",
        check.fits(),
        check.trimmed_by_a_wall_it_ends_on,
        check.failure
    );
    if round_profile {
        assert!(!check.standing_on_a_face.is_empty());
    } else {
        let reason = check.failure.unwrap();
        assert_eq!(reason.code(), BevelAdvisoryCode::SizeExceedsFaceWidth);
        match reason.args()[2] {
            BevelAdvisoryArg::Double(fits) => assert!(fits < 3.0),
            ref other => panic!("{other:?}"),
        }
    }
}

#[test]
fn a_losing_wall_must_use_its_own_admission_rule_rather_than_claim_the_runs_cap_chamfer() {
    a_losing_wall_must_use_its_own_admission_rule_rather_than_claim_the_runs_cap(false);
}

#[test]
fn a_losing_wall_must_use_its_own_admission_rule_rather_than_claim_the_runs_cap_round() {
    a_losing_wall_must_use_its_own_admission_rule_rather_than_claim_the_runs_cap(true);
}
