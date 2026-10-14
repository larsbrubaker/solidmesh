//! The equality crack between a LANDING and a RUN-OFF - a band whose setback is exactly the width
//! of the face it lands on. One top edge of a 20 x 20 x 2 plate filleted at radius 2: the rolling
//! ball's tangent line on the side wall IS the wall's own bottom rim, and the face is CONSUMED TO
//! ITS RIM - it gives up whole, and the rim's own vertices are the seam.
//!
//! A landing would fail the seam pick in the middle of the edge; a run-off has nothing to cross.
//! Only the third reading has a mesh in it, and it mints no vertex on that side. The two guards
//! either side say the reading turns over at the pool's ruler and not at a size: at 1.9999 the wall
//! keeps a ten-thousandth of itself, and at 2.0001 the ball is refused on room by name. The legacy
//! level-line counterpart is `wall_consumed_to_its_rim_tests.rs`.

use std::sync::Arc;

use geometry_processing::bevel::surface::surface_band_runs_off_the_face_beyond::{
    self as run_off, BandSpan,
};
use geometry_processing::bevel::surface::{
    surface_face_walkback, surface_mesh_closure, SurfaceBevelPlan, SurfaceVertexPool,
};
use geometry_processing::bevel::{bevel_chain_linker, BevelAdvisory, BevelAdvisoryCode};
use geometry_processing::topology::{
    bevel_edge_group_resolver, BevelSelectionKind, FaceGroup, FaceRegionCache, FeatureEdgeSet,
};
use polygon_mesh::solids::platonic_solids;
use polygon_mesh::{CancelToken, Mesh};
use vector_math::Vector3;

use crate::mesh_measurements::{shell_count, signed_volume};
use crate::surface_test_build::{try_bevel, TryBevelOptions};

/// The plate: 20 across each way and 2 thick, so a top edge's side wall is 2 tall.
const PLATE_SIDE: f64 = 20.0;

const PLATE_THICKNESS: f64 = 2.0;

/// The quarter round's chording, which the volume is held to in closed form.
const SEGMENTS: i32 = 16;

/// A mesh keeps its vertices as floats, so a distance of 20 is a distance of 20 to about this.
const NEAR: f64 = 1e-5;

fn plate() -> Mesh {
    platonic_solids::create_cube(PLATE_SIDE, PLATE_SIDE, PLATE_THICKNESS)
}

/// Radius 2 on a 2 mm wall: the band consumes the wall exactly, and the fillet is tangent to the
/// plate's underside along the bottom rim. Closed, manifold, one shell, and the plate less the
/// chorded quarter round over the whole 20 of the edge.
#[test]
fn a_wall_the_band_consumes_exactly_is_given_up_to_its_rim() {
    let built = bevel(&plate(), PLATE_THICKNESS).unwrap_or_else(|refusal| {
        panic!(
            "a ball of 2 on a wall of 2 is tangent at the bottom rim, which is a solid a kernel \
             builds - {}",
            say(refusal.as_ref())
        )
    });

    assert!(
        built.is_manifold(),
        "a bevel of a closed solid stays a closed solid"
    );
    let (closed, open_at) = surface_mesh_closure::is_closed(&built);
    assert!(
        closed,
        "nothing may be left open, and something is at {open_at:?}"
    );
    assert_eq!(shell_count(&built), 1);

    let removed = PLATE_SIDE * removed_section(PLATE_THICKNESS, SEGMENTS);
    let whole = PLATE_SIDE * PLATE_SIDE * PLATE_THICKNESS;
    let volume = signed_volume(&built);

    assert!(
        (volume - (whole - removed)).abs() <= whole * 1e-6,
        "the whole edge gives up one chorded quarter round and nothing else: {volume} against {}",
        whole - removed
    );
}

/// The rim is the SEAM, and the rim's vertices are the plate's own: the two bottom corners of the
/// walled side are still one vertex apiece in the answer.
#[test]
fn the_rims_own_vertices_are_the_strips_rail_zero() {
    let built = bevel(&plate(), PLATE_THICKNESS)
        .unwrap_or_else(|refusal| panic!("{}", say(refusal.as_ref())));

    // The bottom rim of the wall the band consumed: x = +10, z = -1, running the whole 20 in y.
    let on_the_rim = on_the_consumed_walls_rim(&built);

    assert_eq!(
        on_the_rim.len(),
        2,
        "the strip's rail 0 IS the rim, so its two stations are the plate's own two bottom corners \
         and nothing was minted on them - found {on_the_rim:?}"
    );

    for corner in [
        Vector3::new(PLATE_SIDE / 2.0, -PLATE_SIDE / 2.0, -PLATE_THICKNESS / 2.0),
        Vector3::new(PLATE_SIDE / 2.0, PLATE_SIDE / 2.0, -PLATE_THICKNESS / 2.0),
    ] {
        assert_eq!(
            on_the_rim
                .iter()
                .filter(|&&at| (at - corner).length() < NEAR)
                .count(),
            1,
            "and {corner:?} is one vertex of the answer, exactly once"
        );
    }
}

/// The reading itself, and WHERE it turns over: the pool's own ruler and nothing else. The ruler
/// is about 2.8e-4 on this plate, so a ten-thousandth is INSIDE it.
#[test]
fn the_reading_turns_over_at_the_pools_own_ruler() {
    let plate = plate();
    let ruler = SurfaceVertexPool::near_enough_for(&plate.vertices);

    assert!(
        ruler < 1e-3,
        "the whole crack is this wide, and a guard a millimetre out has to clear it - {ruler}"
    );

    for (setback, consumed, why) in [
        (
            PLATE_THICKNESS,
            true,
            "the band reaches exactly as far as the wall does",
        ),
        (
            PLATE_THICKNESS - (ruler / 2.0),
            true,
            "and half a ruler short is the same two points",
        ),
        (
            PLATE_THICKNESS + (ruler / 2.0),
            true,
            "as is half a ruler over, which the room still lets through",
        ),
        (
            PLATE_THICKNESS - (ruler * 4.0),
            false,
            "four rulers of wall is a landing with a tangent line of its own",
        ),
        (
            PLATE_THICKNESS / 2.0,
            false,
            "and half the wall is the ordinary case this must not take",
        ),
    ] {
        assert_eq!(
            the_wall_is_consumed_at(&plate, setback),
            consumed,
            "{why} - at a setback of {setback} on a wall of {PLATE_THICKNESS}"
        );
    }
}

/// Four rulers short of consuming the wall is an ORDINARY landing, and is still built as one.
#[test]
fn a_wall_that_keeps_more_than_a_ruler_of_itself_is_still_an_ordinary_landing() {
    let plate = plate();
    let keeps = SurfaceVertexPool::near_enough_for(&plate.vertices) * 4.0;

    let built = bevel(&plate, PLATE_THICKNESS - keeps).unwrap_or_else(|refusal| {
        panic!(
            "a wall with {keeps} of itself left lands the band on it - {}",
            say(refusal.as_ref())
        )
    });

    assert!(built.is_manifold());
    let (closed, open_at) = surface_mesh_closure::is_closed(&built);
    assert!(
        closed,
        "nothing may be left open, and something is at {open_at:?}"
    );

    assert_eq!(
        on_the_consumed_walls_rim(&built).len(),
        2,
        "the bottom rim is untouched material here - the seam stands four rulers above it"
    );

    let removed = PLATE_SIDE * removed_section(PLATE_THICKNESS - keeps, SEGMENTS);
    let whole = PLATE_SIDE * PLATE_SIDE * PLATE_THICKNESS;
    let volume = signed_volume(&built);

    assert!(
        (volume - (whole - removed)).abs() <= whole * 1e-6,
        "and what it takes is that smaller round, sewn to its own tangent line on the wall: \
         {volume} against {}",
        whole - removed
    );
}

/// And a ten-thousandth over wants more wall than the plate has, which is refused by name.
#[test]
fn a_wall_a_ten_thousandth_too_short_is_refused_on_room() {
    let refusal = match bevel(&plate(), PLATE_THICKNESS + 1e-4) {
        Ok(_) => None,
        Err(refusal) => refusal,
    };
    let refusal = refusal.expect("2.0001 of ball on 2 of wall has nowhere to be tangent");

    assert_eq!(
        refusal.code(),
        BevelAdvisoryCode::SizeExceedsFaceWidth,
        "and the name is the one that says which edit recovers it - {refusal}"
    );
}

/// The reading asked of the plate's 2 mm side wall directly, at a setback of the caller's
/// choosing: the plan is made at that setback, and the side of the walk whose in-face direction
/// runs DOWN the wall is the one asked about.
fn the_wall_is_consumed_at(plate: &Mesh, setback: f64) -> bool {
    let features = FeatureEdgeSet::create(plate);
    let regions = Arc::new(FaceRegionCache::new(
        Arc::new(plate.clone()),
        Some(plate.get_mesh_edge_graph()),
        features.flat_angle_degrees(),
    ));
    let chains =
        bevel_chain_linker::link(&[features.edges()[top_edge_at_plus_x(&features) as usize]]);
    assert_eq!(chains.len(), 1);

    // A square edge, so the fillet's setback IS its radius and the plan can be asked for one
    // directly.
    let plan = SurfaceBevelPlan::try_create(&chains[0], &regions, setback, true).unwrap_or_else(
        |reason| panic!("the plate's top edge would not plan at {setback}: {reason}"),
    );

    let edge = &plan.edges[0];
    let down = edge.left_inward_at_the_edge.get_normal().z < 0.0;

    let (beveled, corners) = surface_face_walkback::the_chains_own(std::slice::from_ref(&plan));

    run_off::consumes_the_face_to_its_rim(
        Some(if down {
            &edge.left_region
        } else {
            &edge.right_region
        }),
        &BandSpan {
            from: edge.from(),
            along: edge.edge.direction(),
            length: (edge.to() - edge.from()).length(),
            inward: if down {
                edge.left_inward_at_the_edge
            } else {
                edge.right_inward_at_the_edge
            }
            .get_normal(),
            setback: if down {
                edge.left_setback
            } else {
                edge.right_setback
            },
        },
        &beveled,
        &corners,
        SurfaceVertexPool::near_enough_for(&plate.vertices),
    )
}

/// The answer's vertices standing on the walled side's bottom rim - x = +10, z = -1.
fn on_the_consumed_walls_rim(built: &Mesh) -> Vec<Vector3> {
    built
        .vertices
        .iter()
        .map(|&vertex| Vector3::from_vector3_float(vertex))
        .filter(|at| {
            (at.x - (PLATE_SIDE / 2.0)).abs() < NEAR
                && (at.z + (PLATE_THICKNESS / 2.0)).abs() < NEAR
        })
        .collect()
}

/// The plate's +x top edge filleted by the surface construction, or whatever refused it (`None`
/// for a build that declined without naming anything).
fn bevel(plate: &Mesh, radius: f64) -> Result<Mesh, Option<BevelAdvisory>> {
    let features = FeatureEdgeSet::create(plate);

    let group = bevel_edge_group_resolver::create_from_edge(
        &features,
        top_edge_at_plus_x(&features),
        BevelSelectionKind::Edge,
        FaceGroup::DEFAULT_CURVE_TOLERANCE_DEGREES,
    )
    .expect("the top edge is a feature edge");

    let outcome = try_bevel(
        plate,
        &[group],
        radius,
        SEGMENTS,
        &CancelToken::new(),
        TryBevelOptions::default(),
    );

    match outcome.mesh_or_reason() {
        Ok(mesh) => Ok(mesh.clone()),
        Err(reason) => Err(reason.cloned()),
    }
}

/// The top edge at +x, which runs the plate's whole 20 in y with the top face on one side and the
/// 2 mm wall on the other.
fn top_edge_at_plus_x(features: &FeatureEdgeSet) -> i32 {
    let found: Vec<usize> = (0..features.count())
        .filter(|&at| {
            let edge = &features.edges()[at];

            (edge.start.z - (PLATE_THICKNESS / 2.0)).abs() < NEAR
                && (edge.end.z - (PLATE_THICKNESS / 2.0)).abs() < NEAR
                && (edge.start.x - (PLATE_SIDE / 2.0)).abs() < NEAR
                && (edge.end.x - (PLATE_SIDE / 2.0)).abs() < NEAR
        })
        .collect();

    assert_eq!(found.len(), 1, "exactly one top edge at +x");

    found[0] as i32
}

/// What a chorded quarter round takes off a square corner per unit of edge: the corner's own
/// `r * r` square less the fan of `segments` chords the profile draws inside it, each a triangle of
/// `r * r * sin(theta) / 2` about the ball centre.
fn removed_section(radius: f64, segments: i32) -> f64 {
    (radius * radius)
        - (segments as f64
            * radius
            * radius
            * (std::f64::consts::PI / (2.0 * segments as f64)).sin()
            / 2.0)
}

fn say(advisory: Option<&BevelAdvisory>) -> String {
    match advisory {
        None => "nothing was refused".to_string(),
        Some(advisory) => advisory.to_string(),
    }
}
