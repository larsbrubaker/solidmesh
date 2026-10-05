//! Ports `Tests/GeometryProcessingTests/BevelMitreSplitTests.cs`: a rim that turns a corner the
//! swept tool cannot be mitred through is CUT there rather than refused
//! (`bevel_chain_linker::split_where_the_mitre_cannot_fit`), and a rim whose corners the tool
//! carries perfectly well is left exactly as it was.

use agg_path::vertex_storage::VertexStorage;
use geometry_processing::bevel::bevel_chain_linker::{self, BevelSweptChain};
use geometry_processing::bevel::bevel_chain_sweep::{self, BevelSweptSection};
use geometry_processing::bevel::BevelAdvisoryCode;
use geometry_processing::topology::FeatureEdgeSet;
use polygon_mesh::processors::vertex_source_to_mesh::extrude;
use polygon_mesh::Mesh;

use crate::topology_test_meshes::create_arc_prism;

const RADIUS: f64 = 10.0;

const HEIGHT: f64 = 20.0;

/// The arc's facet count, which puts 9.47 degrees and 1.65 mm in each step at [`RADIUS`].
const ARC_FACETS: i32 = 19;

/// The fillet the fixture asks for, and the one the corner mitre cannot fit.
const SIZE: f64 = 2.0;

const SEGMENTS: i32 = 8;

#[test]
fn the_rim_of_a_half_cylinders_end_is_cut_at_the_two_corners_the_mitre_cannot_fit() {
    let rim = rim(&create_arc_prism(RADIUS, HEIGHT, ARC_FACETS), HEIGHT);

    assert!(
        rim.is_closed,
        "the end face's boundary is one loop, and the linker closes it with no tangent tolerance"
    );
    assert_eq!(
        rim.edges.len(),
        ARC_FACETS as usize + 1,
        "the rim is the arc's facets plus the one straight chord that closes them"
    );

    // The claim this whole split rests on: the sweep really would refuse this run whole.
    let refusal = bevel_chain_sweep::try_create(
        &rim.edges,
        rim.is_closed,
        None,
        SIZE,
        SEGMENTS,
        true,
        BevelSweptSection::Tool,
        0.0,
        0.0,
        None,
        0.0,
        None,
    )
    .expect_err("the fixture means nothing unless the un-cut loop folds");

    assert_eq!(
        refusal.code(),
        BevelAdvisoryCode::SizeFoldsToolAtCurve,
        "and folds for the mitre's sake rather than any other - instead: {refusal:?}"
    );

    let whole = rim.edges.len();
    let pieces = bevel_chain_linker::split_where_the_mitre_cannot_fit(rim, SIZE, SEGMENTS, true);

    assert_eq!(
        pieces.len(),
        2,
        "cut at both corners, a closed rim of chord plus arc is the chord and the arc"
    );

    for piece in &pieces {
        assert!(
            !piece.is_closed,
            "a run cut anywhere is open, and its two ends are corners for the cap and blend machinery"
        );
    }

    assert_eq!(
        pieces.iter().map(|piece| piece.edges.len()).sum::<usize>(),
        whole,
        "cutting a run loses no edge of it"
    );

    let chords: Vec<_> = pieces
        .iter()
        .filter(|piece| piece.edges.len() == 1)
        .collect();
    assert_eq!(chords.len(), 1);
    let chord = &chords[0].edges[0];

    assert!(
        ((chord.to() - chord.from()).length() - 2.0 * RADIUS).abs() <= 1e-6,
        "the piece of one edge is the straight chord, so the cuts landed on the two corners the chord \
         meets the arc at"
    );
}

#[test]
fn a_box_tops_rim_keeps_its_four_corners_because_the_mitre_fits_them() {
    let rim = rim(&box_prism(2.0 * RADIUS, HEIGHT), HEIGHT);

    assert_eq!(
        rim.edges.len(),
        4,
        "a box top is four long edges, which is the case that must not be disturbed"
    );

    let pieces =
        bevel_chain_linker::split_where_the_mitre_cannot_fit(rim.clone(), SIZE, SEGMENTS, true);

    assert_eq!(
        pieces.len(),
        1,
        "a 90 degree corner between two 20 mm edges drags the tool about 2 mm, which those edges carry \
         easily"
    );

    // C#'s IsSameReferenceAs: a run with nothing to cut is handed back untouched rather than
    // rebuilt - here, the very value that went in.
    assert_eq!(pieces[0], rim);
    assert!(
        pieces[0].is_closed,
        "and is still the closed loop it was, which is what lets it weld rather than cap"
    );
}

/// The closed run of the top face's boundary edges, linked the way a face group's rim is.
fn rim(part: &Mesh, top: f64) -> BevelSweptChain {
    let features = FeatureEdgeSet::create(part);

    let on_the_top: Vec<_> = features
        .edges()
        .iter()
        .filter(|edge| (edge.start.z - top).abs() < 1e-5 && (edge.end.z - top).abs() < 1e-5)
        .copied()
        .collect();

    let mut chains = bevel_chain_linker::link(&on_the_top);
    assert_eq!(chains.len(), 1);
    chains.remove(0)
}

/// A square prism, built the same way the arc prism is.
fn box_prism(side: f64, height: f64) -> Mesh {
    let mut path = VertexStorage::new();

    path.move_to(0.0, 0.0);
    path.line_to(side, 0.0);
    path.line_to(side, side);
    path.line_to(0.0, side);
    path.close_polygon();

    extrude(&path, height).expect("a square extrudes")
}
