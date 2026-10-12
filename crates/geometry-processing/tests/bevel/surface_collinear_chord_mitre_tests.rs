//! Port of `SurfaceCollinearChordMitreTests.cs`: ONE CREASE THE SOURCE'S OWN BOOLEAN CHORDED,
//! beaded as the two mesh edges it really is - the upper right crease of the nose-in-notch part
//! split at (12, 8, 12), picked whole, and met at that split as a corner the chain turns through
//! NOTHING at. This is what "Center And Right Face.mcx" dies on.
//!
//! **Two collinear pieces of one crease share one tangent line, not two.** The mitre plane's point
//! is where the two beads' tangent lines cross on the face they SHARE, and two pieces of ONE straight
//! crease between the SAME two faces put both lines in the same place - so the corner itself is the
//! point, and the normal is the sum of two equal walk directions: the square cut a straight run
//! wants. C# `[Arguments]` rows are a loop.

use std::sync::Arc;

use geometry_processing::bevel::surface::{
    surface_bevel_builder, surface_chain_join, surface_mesh_closure, SurfaceBevelPlan,
    SurfaceBuildOptions,
};
use geometry_processing::bevel::{bevel_chain_linker, BevelAdvisoryArg, BevelAdvisoryCode};
use geometry_processing::topology::{
    bevel_edge_group_resolver, BevelEdgeGroup, BevelSelectionKind, FaceGroup, FaceRegionCache,
    FeatureEdge, FeatureEdgeSet,
};
use polygon_mesh::{CancelToken, Mesh};
use vector_math::Vector3;

use crate::mesh_measurements::{shell_count, signed_volume};
use crate::nose_in_notch_test_meshes as nose;
use crate::surface_test_build::{try_bevel, TryBevelOptions};

/// Where the boolean under the corpus parts chords the upper right crease - its t = 0.6.
fn the_chord() -> Vector3 {
    Vector3::new(12.0, 8.0, 12.0)
}

/// The chorded crease builds what the whole crease builds - one solid, because chording an edge is a
/// triangulation of the same solid and not a different part - and meets itself on a square cut.
#[test]
fn a_crease_chorded_in_two_builds_what_the_whole_crease_builds() {
    for radius in [2.0, 3.0] {
        let whole = built(&nose::create(), radius, false);
        let chorded = built(
            &nose::create_with_upper_right_crease_split_at(the_chord()),
            radius,
            true,
        );

        let (open, open_at) = surface_mesh_closure::open_edge_count(&chorded);

        assert_eq!(
            open, 0,
            "the chorded crease closes as the whole one does - open at {open_at:?}"
        );
        assert!(chorded.is_manifold());
        assert_eq!(shell_count(&chorded), 1);

        let chorded_volume = signed_volume(&chorded);
        let whole_volume = signed_volume(&whole);

        assert!(
            (chorded_volume - whole_volume).abs() <= 1e-4,
            "the two builds are of one solid - measured 4.2e-06 apart in C#: {chorded_volume} \
             against {whole_volume} (radius {radius})"
        );

        assert!(
            !every(&chorded)
                .iter()
                .any(|&at| (at - the_chord()).length() < 1e-6),
            "the chord's own point stands inside the band the walkback takes off, so nothing of it \
             survives into the answer"
        );

        // And the square cut itself: a plane square to the crease through the chord. The ring the
        // two strips meet along stands in it, and the whole crease's own build has nothing there.
        let along = (nose::upper_crease_end() - nose::right_ridge_end()).get_normal();

        let on_the_square_cut = |built: &Mesh| {
            every(built)
                .iter()
                .filter(|&&at| {
                    (at - the_chord()).dot(along).abs() < 1e-6
                        && (at - the_chord()).length() < radius * 1.5
                })
                .count()
        };

        assert_eq!(
            on_the_square_cut(&whole),
            0,
            "the whole crease turns no corner at the chord, so nothing of it is cut there"
        );

        assert!(
            on_the_square_cut(&chorded) >= 12,
            "and the chorded one meets itself on a ring of one vertex per rail, which at this wedge \
             is 12 - found {} (radius {radius})",
            on_the_square_cut(&chorded)
        );
    }
}

/// THE CHORD DOES NOT RENAME THE REFUSAL. The ridge and the upper right crease together, at radius
/// 2, are refused on the CLEAN solid as `SurfaceMixedCornerRunOutPastTheCrease` at the right ridge
/// end; chording the upper right crease at (12, 8, 12) changes nothing about that reading, and the
/// same name has to come back.
#[test]
fn the_ridge_and_the_upper_right_crease_run_out_past_the_crease_chorded_or_not() {
    for chorded in [false, true] {
        let part = if chorded {
            nose::create_with_upper_right_crease_split_at(the_chord())
        } else {
            nose::create()
        };

        let features = FeatureEdgeSet::create(&part);

        let groups = vec![
            picked_whole(&features, nose::left_ridge_end(), nose::right_ridge_end()),
            picked_whole(
                &features,
                nose::right_ridge_end(),
                if chorded {
                    the_chord()
                } else {
                    nose::upper_crease_end()
                },
            ),
        ];

        let outcome = try_bevel(
            &part,
            &groups,
            2.0,
            16,
            &CancelToken::new(),
            TryBevelOptions::default(),
        );

        assert!(
            !outcome.built(),
            "the run-out reaches 9.169 along a crease that runs 7.8174, so this size has no \
             construction on either mesh (chorded {chorded})"
        );

        let reason = outcome.reason().expect("a refusal names its reason");

        assert_eq!(
            reason.code(),
            BevelAdvisoryCode::SurfaceMixedCornerRunOutPastTheCrease,
            "the chord is the boolean's triangulation and not a different pick - instead {reason:?}"
        );

        let BevelAdvisoryArg::Vector3(at) = reason.args()[0] else {
            panic!("the run-out is named at a place - instead {reason:?}");
        };

        assert!(
            (at - nose::right_ridge_end()).length() < 1e-6,
            "and it is the leaning corner at the right ridge end that runs out, not anything at the \
             chord - {at:?}"
        );
    }
}

/// The bead over the upper right crease, picked whole however many mesh edges it is.
fn built(part: &Mesh, radius: f64, chorded: bool) -> Mesh {
    let features = FeatureEdgeSet::create(part);
    let graph = part.get_mesh_edge_graph();
    let regions = Arc::new(FaceRegionCache::new(
        Arc::new(part.clone()),
        Some(graph.clone()),
        features.flat_angle_degrees(),
    ));

    let wanted: Vec<(Vector3, Vector3)> = if chorded {
        vec![
            (nose::right_ridge_end(), the_chord()),
            (the_chord(), nose::upper_crease_end()),
        ]
    } else {
        vec![(nose::right_ridge_end(), nose::upper_crease_end())]
    };

    let picked: Vec<FeatureEdge> = wanted
        .iter()
        .map(|&(from, to)| {
            let found: Vec<&FeatureEdge> = features
                .edges()
                .iter()
                .filter(|edge| runs_between(edge, from, to))
                .collect();

            assert_eq!(found.len(), 1, "one edge runs from {from:?} to {to:?}");

            *found[0]
        })
        .collect();

    let chains = surface_chain_join::join(&bevel_chain_linker::link(&picked));

    assert!(
        chains.len() == 1 && chains[0].edges.len() == wanted.len(),
        "the crease is one chain of {}, not {} of {:?}",
        wanted.len(),
        chains.len(),
        chains
            .iter()
            .map(|chain| chain.edges.len())
            .collect::<Vec<_>>()
    );

    let plan = SurfaceBevelPlan::try_create(&chains[0], &regions, radius, true)
        .unwrap_or_else(|why| panic!("the plan refused at {radius}: {:?}", why.code()));

    surface_bevel_builder::try_build(
        part,
        &graph,
        &regions,
        &[plan],
        15,
        &CancelToken::new(),
        SurfaceBuildOptions::default(),
    )
    .unwrap_or_else(|refused| {
        panic!(
            "the {} crease refused at {radius}: {refused:?}",
            if chorded { "chorded" } else { "whole" }
        )
    })
    .mesh
}

fn runs_between(edge: &FeatureEdge, from: Vector3, to: Vector3) -> bool {
    ((edge.start - from).length() < 1e-6 && (edge.end - to).length() < 1e-6)
        || ((edge.end - from).length() < 1e-6 && (edge.start - to).length() < 1e-6)
}

/// The Edge-kind group a single click on the feature edge between these two points makes - which
/// resolves to the whole COLLINEAR run it belongs to.
fn picked_whole(features: &FeatureEdgeSet, from: Vector3, to: Vector3) -> BevelEdgeGroup {
    let index = features
        .edges()
        .iter()
        .position(|edge| runs_between(edge, from, to))
        .unwrap_or_else(|| panic!("the part carries no feature edge from {from:?} to {to:?}"));

    bevel_edge_group_resolver::create_from_edge(
        features,
        index as i32,
        BevelSelectionKind::Edge,
        FaceGroup::DEFAULT_CURVE_TOLERANCE_DEGREES,
    )
    .expect("the seed is an edge")
}

/// Every vertex of a build, as points.
fn every(mesh: &Mesh) -> Vec<Vector3> {
    mesh.vertices
        .iter()
        .map(|&vertex| Vector3::from_vector3_float(vertex))
        .collect()
}
