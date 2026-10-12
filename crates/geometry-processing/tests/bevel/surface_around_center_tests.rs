//! Port of `SurfaceAroundCenterTests.cs`: "5mm Bevel Around Center.mcx" asked of Surface mode - the
//! FOUR concave creases where the nose's two faces die on the two slopes of the chevron notch, picked
//! together and rounded in one pass, measured on the nose-in-notch fixture.
//!
//! **The pick is ONE closed loop.** The four creases meet two at a time at the two ridge ends and
//! two at a time on the notch's centre crease, so the join walks the four separate picks into a
//! single closed chain of four edges, and every one of the four corners is an ordinary chain mitre.
//!
//! **What decides which CONSTRUCTION each ridge end takes is where the mitre TIP lands.** The tip
//! stands `1.5495 * radius` from the corner along the slope's own line, which leaves the part at
//! radius 4.5634: below that every rail is mitred; above it the outermost rail of each strip is cut
//! on the face BEYOND the knife-tip edge instead. C# `[Arguments]` rows are a loop.

use std::sync::Arc;

use geometry_processing::bevel::bevel_chain_linker;
use geometry_processing::bevel::surface::{
    surface_bevel_builder, surface_chain_join, surface_mesh_closure, SurfaceBevelPlan,
    SurfaceBuildOptions,
};
use geometry_processing::topology::{FaceRegionCache, FeatureEdge, FeatureEdgeSet};
use polygon_mesh::{CancelToken, Mesh};
use vector_math::Vector3;

use crate::mesh_measurements::{shell_count, signed_volume};
use crate::nose_in_notch_test_meshes as nose_in_notch;
use crate::surface_walkback_checks::{corners, regions_of};

/// The operation's own segment count in the fixture.
const SEGMENTS: i32 = 15;

/// The nose-in-notch part itself: the 20 mm cube less the two slope prisms, plus the nose.
const PART_VOLUME: f64 = 6055.5556;

fn assert_within(actual: f64, expected: f64, tolerance: f64, because: &str) {
    assert!(
        (actual - expected).abs() <= tolerance,
        "{actual} is not {expected} within {tolerance}: {because}"
    );
}

/// Four concave beads of radius 4.5 in one pass, the largest round measured here that keeps both
/// ridge ends' mitre tips on the slopes they are cut against: the loop closes, and what is left of
/// the nose is an ISLAND of its own ridge between x = 8.8678 and 11.1322.
#[test]
fn the_four_beads_close_where_the_mitre_stays_on_the_face() {
    let part = nose_in_notch::create();
    let built = built(&part, 4.5);

    let (open, open_at) = surface_mesh_closure::open_edge_count(&built);
    assert_eq!(
        open, 0,
        "the four beads close into one another round the loop - open at {open_at:?}"
    );
    assert!(built.is_manifold());
    assert_eq!(shell_count(&built), 1);
    assert_eq!(
        folded_onto_a_point(&built),
        0,
        "a triangle with two corners at one point is a fold, not a face"
    );

    // What is left of the ridge, measured along its own line at y = 5, z = 10.
    let ridge = sorted(
        built
            .vertices
            .iter()
            .filter(|v| (v.y as f64 - 5.0).abs() < 1e-3 && (v.z as f64 - 10.0).abs() < 1e-3)
            .map(|v| v.x as f64),
    );

    assert_eq!(
        ridge.len(),
        3,
        "the island is its two ends and the spine between them - found {ridge:?}"
    );
    assert_within(ridge[0], 8.8678, 1e-3, "the island's left end");
    assert_within(ridge[1], 10.0, 1e-3, "the spine");
    assert_within(ridge[2], 11.1322, 1e-3, "the island's right end");

    // And where the two mitres on the notch's centre crease put their rails.
    let centre = sorted(
        built
            .vertices
            .iter()
            .filter(|v| (v.x as f64 - 10.0).abs() < 1e-3 && (v.y as f64 - 10.0).abs() < 1e-3)
            .map(|v| v.z as f64),
    );

    assert_eq!(
        centre.len(),
        4,
        "the crease keeps an end and a run-out at each end - found {centre:?}"
    );
    assert_within(centre[0], 0.0, 1e-3, "the crease's lower end");
    assert_within(
        centre[1],
        3.3797,
        1e-3,
        "the lower pair's mitre runs out here - the r 5 control runs out at 3.0144",
    );
    assert_within(
        centre[2],
        16.6203,
        1e-3,
        "and the upper pair's here - the r 5 control at 16.9856",
    );
    assert_within(centre[3], 20.0, 1e-3, "the crease's upper end");

    assert_within(
        signed_volume(&built).abs(),
        6107.1549,
        1e-3,
        &format!("every one of the four runs is concave, so this fillet only ever adds to the {PART_VOLUME} the part has"),
    );
    assert!(signed_volume(&built).abs() > PART_VOLUME);
}

/// At the radius the fixture was SAVED with, the two ridge ends' mitres stand past the 45 degree
/// knife-tip edges they are cut against, and the pass runs them off onto the faces beyond; the face
/// beyond gains the lens between the two exit curves.
#[test]
fn at_the_saved_radius_the_knife_tip_mitre_runs_off_onto_the_face_beyond() {
    let part = nose_in_notch::create();
    let built = built(&part, 5.0);

    let (open, open_at) = surface_mesh_closure::open_edge_count(&built);
    assert_eq!(
        open, 0,
        "the lens closes the rim the two run-offs opened - open at {open_at:?}"
    );
    assert!(built.is_manifold());
    assert_eq!(shell_count(&built), 1);
    assert_eq!(folded_onto_a_point(&built), 0);

    // Where the two beads run out ON the right knife-tip edge, which is the control's own pair.
    let right_edge = sorted(
        built
            .vertices
            .iter()
            .filter(|v| (v.x as f64 - 20.0).abs() < 1e-3 && (v.y as f64).abs() < 1e-3)
            .map(|v| v.z as f64),
    );

    assert!(
        right_edge.iter().any(|z| (z - 10.3189).abs() < 1e-3),
        "the upper bead runs out here - found {right_edge:?}"
    );
    assert!(
        right_edge.iter().any(|z| (z - 9.6811).abs() < 1e-3),
        "and the lower one here - found {right_edge:?}"
    );

    // And the lens itself, which stands BEYOND that face's old rim: x = 0 exactly, y under zero.
    let lens: Vec<Vector3> = built
        .vertices
        .iter()
        .filter(|v| (v.x as f64).abs() < 1e-6 && (v.y as f64) < -1e-6)
        .map(|&v| Vector3::from_vector3_float(v))
        .collect();

    assert_eq!(
        lens.len(),
        1,
        "the left face gains one turn point past its rim - found {lens:?}"
    );
    assert_within(lens[0].z, 10.0, 1e-3, "the turn point's height");
    assert_within(
        lens[0].y,
        -0.0212,
        1e-3,
        &format!("the turn point stands past the face's rim, where the control's exact cylinder stands 0.0115 past it - {:?}", lens[0]),
    );
    assert!(lens[0].y < 0.0);

    // The nose's island, which the control leaves 1.405 wide about the spine on x = 10.
    let ridge = sorted(
        built
            .vertices
            .iter()
            .filter(|v| (v.y as f64 - 5.0).abs() < 1e-3 && (v.z as f64 - 10.0).abs() < 1e-3)
            .map(|v| v.x as f64),
    );

    assert_eq!(ridge.len(), 3, "found {ridge:?}");
    assert_within(ridge[0], 9.2976, 1e-3, "the island's left end");
    assert_within(ridge[2], 10.7024, 1e-3, "the island's right end");

    // And the two run-outs on the notch's own centre crease, which are the control's exactly.
    let centre = sorted(
        built
            .vertices
            .iter()
            .filter(|v| (v.x as f64 - 10.0).abs() < 1e-3 && (v.y as f64 - 10.0).abs() < 1e-3)
            .map(|v| v.z as f64),
    );

    assert_eq!(centre.len(), 4, "found {centre:?}");
    assert_within(centre[1], 3.0144, 1e-3, "the lower run-out");
    assert_within(centre[2], 16.9856, 1e-3, "the upper run-out");

    assert_within(
        signed_volume(&built).abs(),
        6120.014,
        1.0,
        "the control measures 6120.014, and a chording of the same surface differs by under one cubic millimetre",
    );
}

/// The line the CONSTRUCTION turns on, read from both sides: at 4.55 the mitre tip still stands on
/// the slope and is an ordinary chain mitre, and at 4.6 it has left the slope and runs off onto the
/// face beyond. Both close.
#[test]
fn the_mitre_leaves_the_slope_just_past_four_and_a_half() {
    for (radius, runs_off) in [(4.55, false), (4.6, true)] {
        let part = nose_in_notch::create();
        let built = built(&part, radius);

        let (open, open_at) = surface_mesh_closure::open_edge_count(&built);
        assert_eq!(open, 0, "open at {open_at:?}");
        assert!(built.is_manifold());
        assert_eq!(shell_count(&built), 1);

        // Whether anything stands past the left face's own rim is the whole difference between the
        // two.
        let past_the_rim = built
            .vertices
            .iter()
            .filter(|v| (v.x as f64).abs() < 1e-6 && (v.y as f64) < -1e-6)
            .count();

        assert_eq!(
            past_the_rim,
            if runs_off { 1 } else { 0 },
            "the tip stands {:.4} out from the ridge end, and the slope reaches 7.0711",
            1.5495 * radius
        );
    }
}

fn sorted(values: impl Iterator<Item = f64>) -> Vec<f64> {
    let mut values: Vec<f64> = values.collect();
    values.sort_by(f64::total_cmp);
    values
}

/// The four creases, joined, planned and built at one radius.
fn built(part: &Mesh, radius: f64) -> Mesh {
    let shared = Arc::new(part.clone());
    let (regions, features) = regions_of(&shared);
    let regions = Arc::new(regions);

    let plans = plans_for(&regions, &features, radius);

    surface_bevel_builder::try_build(
        part,
        &part.get_mesh_edge_graph(),
        &regions,
        &plans,
        SEGMENTS,
        &CancelToken::new(),
        SurfaceBuildOptions::default(),
    )
    .unwrap_or_else(|reason| panic!("the four creases have to build at {radius}: {reason:?}"))
    .mesh
}

/// The pick as one plan: the four creases linked, JOINED - which is what makes them one closed loop
/// rather than four runs meeting at four corners - and planned at the one radius.
fn plans_for(
    regions: &Arc<FaceRegionCache>,
    features: &FeatureEdgeSet,
    radius: f64,
) -> Vec<std::rc::Rc<SurfaceBevelPlan>> {
    let chains = surface_chain_join::join(&bevel_chain_linker::link(&four_creases(features)));

    assert!(
        chains.len() == 1 && chains[0].is_closed && chains[0].edges.len() == 4,
        "the four creases are one closed loop of four, not {} chains of {:?}",
        chains.len(),
        chains
            .iter()
            .map(|chain| chain.edges.len())
            .collect::<Vec<_>>()
    );

    let plan = SurfaceBevelPlan::try_create(&chains[0], regions, radius, true)
        .unwrap_or_else(|reason| panic!("the loop has to plan at {radius}: {reason}"));

    vec![plan]
}

/// The four creases the fixture picks, in the order the part names them.
fn four_creases(features: &FeatureEdgeSet) -> Vec<FeatureEdge> {
    let wanted = [
        (
            nose_in_notch::left_ridge_end(),
            nose_in_notch::upper_crease_end(),
        ),
        (
            nose_in_notch::left_ridge_end(),
            nose_in_notch::lower_crease_end(),
        ),
        (
            nose_in_notch::right_ridge_end(),
            nose_in_notch::upper_crease_end(),
        ),
        (
            nose_in_notch::right_ridge_end(),
            nose_in_notch::lower_crease_end(),
        ),
    ];

    wanted
        .iter()
        .map(|&(from, to)| {
            let found: Vec<_> = features
                .edges()
                .iter()
                .filter(|edge| {
                    ((edge.start - from).length() < 1e-6 && (edge.end - to).length() < 1e-6)
                        || ((edge.end - from).length() < 1e-6 && (edge.start - to).length() < 1e-6)
                })
                .cloned()
                .collect();
            assert_eq!(found.len(), 1);
            found.into_iter().next().expect("one crease")
        })
        .collect()
}

/// How many triangles have two of their corners at one point.
fn folded_onto_a_point(mesh: &Mesh) -> usize {
    (0..mesh.faces.len() as i32)
        .filter(|&face| {
            let corners = corners(mesh, face);

            (corners[0] - corners[1]).length() < 1e-9
                || (corners[1] - corners[2]).length() < 1e-9
                || (corners[2] - corners[0]).length() < 1e-9
        })
        .count()
}
