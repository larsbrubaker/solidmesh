//! Stage 2, rung 5 - the room is honest at an END FACE. The band may cut up to the end face's far
//! edges but never across them; a triangular prism's vertical edge tells the honest room from one
//! blind to its end faces.

use geometry_processing::bevel::level_line::band_gap_entry_points::{self, BandGapBuildOptions};
use geometry_processing::bevel::level_line::band_gap_reading;
use geometry_processing::bevel::level_line::chain_on_the_mesh;
use geometry_processing::bevel::level_line::gap_fill;
use geometry_processing::bevel::level_line::seam_edges::FeatureEdgeTest;
use geometry_processing::bevel::level_line::{
    BandGapRefusal, BandGapResult, BandProfile, GapFillRefusal,
};
use vector_math::{Vector2, Vector3};

use crate::level_line_feature_edges::feature_edges_over;
use crate::level_line_room_geometry::SolidWithChain;
use crate::level_line_test_geometry::cube_solid;
use crate::level_line_test_solids::prism_on_outline;
use crate::sweep_mesh_measures;

const FOLD_DEGREES: f64 = 10.0;

/// The cap's altitude from the picked corner - the short end the room must respect.
const ALTITUDE: f64 = 0.3;

/// The room reads a hair UNDER its closed form - a couple of snaps of approach, scaled by the unit
/// setback - and never over it.
const ROOM_SLACK: f64 = 1e-7;

fn feature(positions: &[Vector3], indices: &[(i32, i32, i32)]) -> FeatureEdgeTest {
    feature_edges_over(positions, indices, FOLD_DEGREES)
}

fn unit_setbacks(positions: &[Vector3], indices: &[(i32, i32, i32)], chain: &[i32]) -> Vec<f64> {
    chain_on_the_mesh::unit_setbacks(
        positions,
        indices,
        chain,
        false,
        &chain_on_the_mesh::chain_edges(chain, false),
        BandProfile::Fillet,
    )
    .expect("every chain edge stands on two triangles")
}

fn room_of(positions: &[Vector3], indices: &[(i32, i32, i32)], chain: &[i32]) -> f64 {
    band_gap_reading::room_of(
        positions,
        indices,
        chain,
        false,
        Some(feature(positions, indices)),
        BandProfile::Fillet,
        1e-9,
    )
    .expect("the room reads")
}

fn build(
    positions: &[Vector3],
    indices: &[(i32, i32, i32)],
    chain: &[i32],
    size: f64,
) -> BandGapResult {
    band_gap_entry_points::build(
        positions,
        indices,
        chain,
        false,
        Some(feature(positions, indices)),
        size,
        BandProfile::Fillet,
        &BandGapBuildOptions::default(),
    )
    .expect("the pick builds or refuses by name")
}

/// One top edge of a cube of edge 2 - the side faces' widths and
/// the end faces' far edges agree on a room of 2.
#[test]
fn a_cube_edge_room_is_the_face_width() {
    let (positions, indices) = cube_solid(1.0);
    let chain = [4, 5];
    let unit = unit_setbacks(&positions, &indices, &chain);
    assert!((unit[0] - 1.0).abs() < 1e-12);

    let room = room_of(&positions, &indices, &chain);
    assert!(room <= 2.0, "{room}");
    assert!(2.0 - room < ROOM_SLACK, "{room}");
}

/// The vertical edge at a triangular prism's apex - a fillet's
/// unit setback is 0.15 there, and each cap's far rim edge stands `ALTITUDE` from the chain's ends,
/// so the room is 0.3 / 0.15 = 2.
#[test]
fn a_short_end_face_bounds_the_room() {
    let (positions, indices, chain) = apex_prism();
    let unit = unit_setbacks(&positions, &indices, &chain);
    assert!((unit[0] - 0.15).abs() < 1e-12, "{}", unit[0]);

    let room = room_of(&positions, &indices, &chain);
    assert!(room <= ALTITUDE / 0.15, "{room}");
    assert!((ALTITUDE / 0.15) - room < ROOM_SLACK, "{room}");
}

/// Above the end face's room the
/// build refuses by SIZE, naming that room; under it the band builds and the fill closes.
#[test]
fn an_end_face_pick_refuses_by_size_above_its_room_and_fills_closed_under() {
    let (positions, indices, chain) = apex_prism();
    let room = room_of(&positions, &indices, &chain);

    let over = build(&positions, &indices, &chain, 1.1 * room);
    assert_eq!(over.refusal, BandGapRefusal::Size);
    assert_eq!(over.room, room);

    for fraction in [0.5, 0.9] {
        let under = build(&positions, &indices, &chain, fraction * room);
        assert!(under.built(), "{fraction} of the room: {:?}", under.refusal);
        let fill = gap_fill::fill(
            &under,
            BandProfile::Fillet,
            gap_fill::DEFAULT_FILLET_SEGMENTS,
        )
        .expect("the fill runs");
        assert_eq!(fill.refusal, GapFillRefusal::None, "{fraction} of the room");
        let mesh = fill.mesh.as_ref().expect("a filled mesh").borrow();
        assert!(sweep_mesh_measures::is_closed_and_consistent(
            mesh.vertices(),
            &sweep_mesh_measures::kept(&mesh)
        ));
    }
}

/// A prism 4 tall on the triangle W = (-2, 0), X = (2, 0), V = (0, 0.3); the chain
/// is the vertical edge at V, top to bottom.
fn apex_prism() -> SolidWithChain {
    let outline = [
        Vector2::new(-2.0, 0.0),
        Vector2::new(2.0, 0.0),
        Vector2::new(0.0, ALTITUDE),
    ];
    let (positions, indices, top) = prism_on_outline(&outline, Vector2::new(0.0, 0.1), 4.0);
    let chain = vec![top[2], top[2] + outline.len() as i32];
    (positions, indices, chain)
}
