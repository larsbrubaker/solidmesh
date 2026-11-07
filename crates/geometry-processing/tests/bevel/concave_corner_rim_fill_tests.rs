//! The rim of an L-shaped top face. At the L's inner corner the top face's own level line rounds
//! the INSIDE of a ninety degree turn, so its rail's chain parameter genuinely swings BACK there -
//! the reading the zipper cannot use raw. The monotone envelope in the gap fill is held to it: the
//! fill closes, keeps the solid's winding and moves an amount that is continuous in the size and
//! invariant under a rigid motion. Each test's cases run as a loop inside it.

use geometry_processing::bevel::level_line::band_gap_entry_points::{self, BandGapBuildOptions};
use geometry_processing::bevel::level_line::{
    gap_fill, BandProfile, GapFillRefusal, GapFillResult,
};
use vector_math::{Matrix4X4, Vector2, Vector3};

use crate::cube_edge_union_tests::chorded_fillet_section;
use crate::level_line_feature_edges::feature_edges_over;
use crate::level_line_room_geometry::SolidWithChain;
use crate::level_line_test_geometry::{
    open_edges, signed_volume, signed_volume_of, winding_number,
};
use crate::level_line_test_solids::prism_on_outline;

const FOLD_DEGREES: f64 = 20.0;

/// The length of the L's rim: 6 + 2 + 4 + 3 + 2 + 5.
const RIM_LENGTH: f64 = 22.0;

/// An l shaped rim fills closed.
#[test]
fn an_l_shaped_rim_fills_closed() {
    for size in [0.2, 0.4, 0.6, 0.8] {
        let solid = l_prism();
        let fill = build(&solid, size);

        assert_eq!(fill.refusal, GapFillRefusal::None, "size {size}");
        let mesh = fill.mesh.as_ref().expect("a filled mesh").borrow();
        assert!(open_edges(&mesh).is_empty(), "size {size}");

        // Consistently oriented: a mesh wound inside out would read its volume negative.
        assert!(signed_volume(&mesh) > 0.0, "size {size}");

        // And SHAPE: every rim edge here is convex, so the fill may only take material away - not
        // one point it invents may stand outside the L that went in.
        for &v in &fill.added_vertices {
            let turns = winding_number(&solid.0, &solid.1, mesh.position(v));
            assert!(
                turns > 0.45,
                "the fill put a point outside the L at size {size}"
            );
        }
    }
}

/// The fillet TAKES material - a positive amount,
/// growing smoothly with the size, and never more than rounding every edge in isolation would take
/// (the sum of the chorded edge prisms is an upper bound: the five convex corners overlap, and the
/// inner corner adds back less than they take off).
#[test]
fn the_removed_volume_is_continuous_in_the_size() {
    let solid = l_prism();
    let before = signed_volume_of(&solid.0, &solid.1);

    let at = removed(&solid, before, 0.40);
    let just_over = removed(&solid, before, 0.42);

    assert!(at > 0.0);
    assert!(just_over > at);
    assert!(just_over - at < at * 0.25);
    assert!(at < edge_prisms(0.40));
    assert!(just_over < edge_prisms(0.42));
}

/// The same pick on the same part turned and moved takes
/// the same material away - the two cuts are the SAME cut, vertex for vertex.
#[test]
fn the_fill_is_invariant_under_a_rigid_motion() {
    let solid = l_prism();
    let moved = moved(&solid);

    let here = removed(&solid, signed_volume_of(&solid.0, &solid.1), 0.4);
    let there = removed(&moved, signed_volume_of(&moved.0, &moved.1), 0.4);

    let vertex_count = |solid: &SolidWithChain| {
        build(solid, 0.4)
            .mesh
            .as_ref()
            .expect("a filled mesh")
            .borrow()
            .vertex_count()
    };
    assert_eq!(
        vertex_count(&moved),
        vertex_count(&solid),
        "the same pick rigidly moved is the same cut, vertex for vertex"
    );
    assert!(
        (there - here).abs() < here.abs() * 1e-9,
        "{there} against {here}"
    );
}

/// How much smaller the filled solid is than the one that went in.
fn removed(solid: &SolidWithChain, before: f64, size: f64) -> f64 {
    let fill = build(solid, size);
    let mesh = fill.mesh.as_ref().expect("a filled mesh").borrow();
    before - signed_volume(&mesh)
}

/// What rounding all six rim edges in isolation would take.
fn edge_prisms(size: f64) -> f64 {
    chorded_fillet_section(size, 4) * RIM_LENGTH
}

fn build(solid: &SolidWithChain, size: f64) -> GapFillResult {
    let gap = band_gap_entry_points::build(
        &solid.0,
        &solid.1,
        &solid.2,
        true,
        Some(feature_edges_over(&solid.0, &solid.1, FOLD_DEGREES)),
        size,
        BandProfile::Fillet,
        &BandGapBuildOptions::default(),
    )
    .expect("the pick builds or refuses by name");

    assert!(
        gap.built(),
        "the L rim must leave room at this size: {:?}",
        gap.refusal
    );

    gap_fill::fill(&gap, BandProfile::Fillet, 4).expect("the fill runs")
}

fn moved(solid: &SolidWithChain) -> SolidWithChain {
    let turn = Matrix4X4::create_rotation(Vector3::new(0.3, -0.7, 1.1))
        * Matrix4X4::create_translation(11.5, -3.25, 7.125);
    (
        solid.0.iter().map(|p| p.transform(turn)).collect(),
        solid.1.clone(),
        solid.2.clone(),
    )
}

/// A prism on an L outline, capped top and bottom by a fan from a point that sees
/// every corner of the L, with the top rim as the picked chain. Its one inner corner turns through
/// ninety degrees the concave way.
fn l_prism() -> SolidWithChain {
    let outline = [
        Vector2::new(0.0, 0.0),
        Vector2::new(6.0, 0.0),
        Vector2::new(6.0, 2.0),
        Vector2::new(2.0, 2.0),
        Vector2::new(2.0, 5.0),
        Vector2::new(0.0, 5.0),
    ];

    prism_on_outline(&outline, Vector2::new(1.0, 1.0), 4.0)
}
