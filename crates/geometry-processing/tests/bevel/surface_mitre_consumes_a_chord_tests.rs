//! A MITRE THAT EATS A WHOLE CHORD ON SOME RAILS STILL
//! BUILDS. The half disc prism's rim turns 85.26 degrees where the diameter meets the arc, and the
//! arc's first chord is only 1.6516 long, so from about 1.41 mm up the two mitres have crossed over
//! on the rails nearest the flat face and the chord's strip is EATEN there. The two rings either
//! side of the chord become staircases: the rails that still have a window end on the real corners,
//! the rails that do not end on the VIRTUAL corner the diameter's and the second chord's lines meet
//! at, (10.281, 0, 20).
//!
//! From about 2.2 mm the diameter's band reaches past the SECOND chord as well, which is a
//! construction of its own and still refused. Legacy-only: every test pins the sections router off
//! (`pinned_to_the_legacy_construction`). The parameter rows are a loop. `half_disc_prism` is
//! this file's, shared with `surface_room_half_disc_rim_tests.rs`.

use std::sync::Arc;

use crate::path_solids::extrude;
use agg_path::vertex_storage::VertexStorage;
use geometry_processing::bevel::surface::{
    surface_bevel_builder, surface_mesh_closure, SurfaceBevelPlan, SurfaceBuildFailure,
    SurfaceBuildOptions, SurfaceBuilt,
};
use geometry_processing::bevel::{bevel_chain_linker, BevelAdvisoryArg, BevelAdvisoryCode};
use geometry_processing::topology::{FaceRegionCache, FeatureEdge, FeatureEdgeSet};
use polygon_mesh::{CancelToken, Mesh};
use vector_math::Vector3;

use crate::mesh_measurements::{shell_count, signed_volume};
use crate::surface_test_build::pinned_to_the_legacy_construction;

pub const FACETS: i32 = 19;

pub const RADIUS: f64 = 10.0;

pub const HEIGHT: f64 = 20.0;

/// The whole half disc prism, before any of it is filleted.
pub const SOURCE_VOLUME: f64 = 3127.315;

/// What the BOOLEAN build of this very rim at size 2 comes back as - the answer the corpus fixture
/// "Edge Loop And Face Should Match.mcx" stores for its Control. The surface build has to be the
/// same solid.
pub const BOOLEAN_VOLUME_AT_TWO: f64 = 3085.094;

/// The size below which no mitre eats anything, kept as a guard: the construction added for the
/// sizes above must not have changed what the ordinary mitre does.
#[test]
fn a_size_the_chord_still_fits_is_untouched() {
    let _pinned = pinned_to_the_legacy_construction();

    for size in [0.5, 1.0] {
        let (mesh, built) = build(size);
        let result =
            built.unwrap_or_else(|reason| panic!("nothing is eaten at {size}: {reason:?}"));

        assert_one_closed_solid(&result.mesh, &mesh, size);
    }
}

/// The sizes the first chord is eaten at on the rails nearest the flat face, which refused with
/// `SurfaceStripSeamDidNotMeet` at the chord's midpoint before the staircase was built.
#[test]
fn the_chord_the_mitres_eat_is_built_as_a_staircase() {
    let _pinned = pinned_to_the_legacy_construction();

    for size in [1.5, 2.0] {
        let (mesh, built) = build(size);
        let result = built.unwrap_or_else(|reason| {
            panic!(
                "the chord's flat rail stands on the corner the two neighbours meet at: {reason:?}"
            )
        });

        assert_one_closed_solid(&result.mesh, &mesh, size);
    }
}

/// And it is the same SOLID the boolean build makes, not merely a closed one: a staircase that
/// stepped the wrong way would still close and would not hold the same material.
#[test]
fn the_eaten_chord_holds_the_boolean_builds_volume() {
    let _pinned = pinned_to_the_legacy_construction();

    let (_, built) = build(2.0);
    let result = built.unwrap_or_else(|reason| panic!("size 2 is the fixture's own: {reason:?}"));
    let volume = signed_volume(&result.mesh);

    assert!(
        (volume - BOOLEAN_VOLUME_AT_TWO).abs() <= 0.02,
        "a tessellated fillet takes a little more than the closed form and nothing like a chord \
         more: the boolean build is {BOOLEAN_VOLUME_AT_TWO:.3} mm^3, this is {volume}"
    );
}

/// TWO chords in a row eaten is not this construction. From about 2.2 mm the diameter's band
/// reaches past the second chord as well, and the walkback leaves it no run either. Recorded so the
/// day it builds the ratchet says so.
#[test]
fn two_chords_in_a_row_are_still_refused() {
    let _pinned = pinned_to_the_legacy_construction();

    let (_, built) = build(3.0);

    let failure = match built {
        Ok(_) => panic!("if it builds now, measure it instead"),
        Err(failure) => failure,
    };
    let reason = failure
        .advisory()
        .unwrap_or_else(|| panic!("a refusal names its reason - instead {failure:?}"));

    assert_eq!(
        reason.code(),
        BevelAdvisoryCode::SurfaceStripSeamDidNotMeet,
        "the second chord has no run to sew to either: {reason:?}"
    );

    let BevelAdvisoryArg::Vector3(at) = reason.args()[0] else {
        panic!("the refusal is named at a place: {reason:?}");
    };

    assert!(
        (at - Vector3::new(9.661, 2.4465, 20.0)).length() < 1e-3,
        "and it is the SECOND chord's midpoint that is named now, not the first's: {reason:?}"
    );
}

/// The size 2 answer, for the kernel to read
/// (`SurfaceResultImportsIntoManifoldTests`).
pub fn the_eaten_chord_built() -> Mesh {
    let (_, built) = build(2.0);

    built
        .unwrap_or_else(|reason| {
            panic!("the eaten chord has to build before it can be imported: {reason:?}")
        })
        .mesh
}

/// The gates every surface answer is held to, whatever it was built out
/// of.
pub fn assert_one_closed_solid(result: &Mesh, mesh: &Mesh, size: f64) {
    let (closed, open_at) = surface_mesh_closure::is_closed(result);

    assert!(
        closed,
        "a fillet of a closed solid stays closed, and this is open at {open_at:?}"
    );
    assert!(result.is_manifold(), "and manifold");
    assert_eq!(
        shell_count(result),
        1,
        "and one solid, with no piece left floating"
    );

    let whole = signed_volume(mesh);
    let left = signed_volume(result);

    assert!(
        (whole - SOURCE_VOLUME).abs() <= 0.01,
        "the source is the same half disc prism every time - {whole}"
    );
    assert!(
        left < whole,
        "a convex fillet of {size} takes material off, and this took none"
    );
    assert!(
        left > whole * 0.97,
        "and it takes a bead off one rim, not a thirtieth of the solid: {whole} became {left}"
    );
}

/// The whole rim of the flat end built as the one closed chain it is - the source mesh
/// and the builder's own answer.
pub fn build(size: f64) -> (Mesh, Result<SurfaceBuilt, SurfaceBuildFailure>) {
    let mesh = half_disc_prism(FACETS, RADIUS, HEIGHT);

    let features = FeatureEdgeSet::create(&mesh);
    let top = mesh
        .vertices
        .iter()
        .map(|vertex| vertex.z as f64)
        .fold(f64::NEG_INFINITY, f64::max);

    let rim: Vec<FeatureEdge> = features
        .edges()
        .iter()
        .filter(|edge| (edge.start.z - top).abs() < 1e-4 && (edge.end.z - top).abs() < 1e-4)
        .cloned()
        .collect();

    assert_eq!(
        rim.len() as i32,
        FACETS + 1,
        "the flat end's rim is the diameter and {FACETS} chords"
    );

    let graph = mesh.get_mesh_edge_graph();
    let regions = Arc::new(FaceRegionCache::new(
        Arc::new(mesh.clone()),
        Some(graph.clone()),
        features.flat_angle_degrees(),
    ));
    let chains = bevel_chain_linker::link(&rim);

    assert_eq!(chains.len(), 1, "the rim is one chain");

    let plan =
        SurfaceBevelPlan::try_create(&chains[0], &regions, size, true).unwrap_or_else(|refused| {
            panic!("the rim has to plan before it can be built: {refused:?}")
        });

    let result = surface_bevel_builder::try_build(
        &mesh,
        &graph,
        &regions,
        &[plan],
        8,
        &CancelToken::new(),
        SurfaceBuildOptions::default(),
    );

    (mesh, result)
}

/// A half disc, drawn with `facets` chords on its arc and closed by its
/// diameter, extruded along z. The corpus fixture's half cylinder in the kernel's own terms.
pub fn half_disc_prism(facets: i32, radius: f64, height: f64) -> Mesh {
    let mut path = VertexStorage::new();

    for i in 0..=facets {
        let angle = std::f64::consts::PI * i as f64 / facets as f64;

        if i == 0 {
            path.move_to(radius, 0.0);
        } else {
            path.line_to(radius * angle.cos(), radius * angle.sin());
        }
    }

    path.close_polygon();

    extrude(&path, height).expect("a half disc extrudes")
}
