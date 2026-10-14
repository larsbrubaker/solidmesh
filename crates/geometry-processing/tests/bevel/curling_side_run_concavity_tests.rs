//! "15mm Bevel Top Edge.mcx" - the chevron's left slope top edge, walked from the ridge end (where
//! the notch's nose creases meet it) to the convex corner on the x = 0 wall. The top face's side
//! run curls round the ridge end, so the middle of that run stands on the slope's own plane
//! extended - and the end-face fill read the chain's sense off exactly that vertex, found the
//! convex edge concave by noise, and refused the open chain's ends through the concave end check.
//! Each test's cases run as a loop inside it.

use std::rc::Rc;

use geometry_processing::bevel::level_line::band_gap_entry_points::{self, BandGapBuildOptions};
use geometry_processing::bevel::level_line::gap_fill;
use geometry_processing::bevel::level_line::{
    BandGapRefusal, BandGapResult, BandProfile, GapFillRefusal, LevelLineMesh,
};
use vector_math::Vector3;

use crate::level_line_feature_edges::feature_edges_over;
use crate::level_line_fixture::LevelLineFixture;
use crate::level_line_room_geometry::SolidWithChain;
use crate::level_line_test_geometry::{signed_volume, signed_volume_of};
use crate::nose_in_notch_test_meshes;
use crate::sweep_mesh_measures;

const RIDGE_END: Vector3 = Vector3::new(10.0, 10.0, 20.0);

const CORNER: Vector3 = Vector3::new(0.0, 0.0, 20.0);

fn part() -> SolidWithChain {
    let mut mesh = nose_in_notch_test_meshes::create();
    mesh.clean_and_merge();
    let positions: Vec<Vector3> = mesh
        .vertices
        .iter()
        .map(|&v| Vector3::from_vector3_float(v))
        .collect();
    let indices: Vec<(i32, i32, i32)> = mesh.faces.iter().map(|f| (f.v0, f.v1, f.v2)).collect();
    let find = |p: Vector3| {
        positions
            .iter()
            .position(|&q| (q - p).length() < 1e-9)
            .map_or(-1, |i| i as i32)
    };

    // The file's own walk: ridge end first, corner last.
    let chain = vec![find(RIDGE_END), find(CORNER)];
    (positions, indices, chain)
}

fn cut(size: f64) -> BandGapResult {
    let (positions, indices, chain) = part();
    let options = BandGapBuildOptions {
        chord_tolerance: 0.0,
        snap: 1e-9,
        sharp_turn_degrees: 20.0,
        fillet_segments: 16,
        ..BandGapBuildOptions::default()
    };
    band_gap_entry_points::build_pick(
        &positions,
        &indices,
        &[chain],
        &[false],
        Some(feature_edges_over(&positions, &indices, 20.0)),
        size,
        BandProfile::Fillet,
        &options,
    )
    .expect("the pick is well formed")
}

/// The file's own pick, as the app handed it to the kernel: the sense is read convex, so the fill
/// passes the concave end check and reaches the oblique corner end, which it refuses by name.
#[test]
fn the_files_pick_reads_convex_and_stops_only_at_the_oblique_corner() {
    let fixture = Rc::new(LevelLineFixture::load("fifteen-mm-top-edge-r659"));
    let profile = if fixture.chamfer {
        BandProfile::Chamfer
    } else {
        BandProfile::Fillet
    };
    let options = BandGapBuildOptions {
        chord_tolerance: 0.0,
        snap: 1e-9,
        sharp_turn_degrees: 20.0,
        fillet_segments: fixture.segments,
        ..BandGapBuildOptions::default()
    };
    let features = Rc::clone(&fixture);
    let gap = band_gap_entry_points::build_pick(
        &fixture.positions,
        &fixture.indices,
        &fixture.chains,
        &fixture.closed,
        Some(Rc::new(move |a, b| features.is_feature_edge(a, b))),
        fixture.size,
        profile,
        &options,
    )
    .expect("the pick is well formed");
    assert!(gap.built(), "{:?}, room {:?}", gap.refusal, gap.room);

    let filled = gap_fill::fill(&gap, profile, fixture.segments).expect("the fill runs");
    assert_eq!(
        filled.refusal,
        GapFillRefusal::None,
        "was CornerCapNotStarShaped at the oblique corner end until the end zip ran about the row's axis"
    );
    assert_built_right(
        &filled.mesh.as_ref().expect("a filled mesh").borrow(),
        signed_volume_of(&fixture.positions, &fixture.indices),
    );
}

fn assert_built_right(mesh: &LevelLineMesh, source: f64) {
    let kept = sweep_mesh_measures::kept(mesh);
    assert!(sweep_mesh_measures::is_closed_and_consistent(
        mesh.vertices(),
        &kept
    ));
    assert_eq!(
        sweep_mesh_measures::shell_count(mesh.vertex_count(), &kept),
        1
    );
    let smallest = kept
        .iter()
        .map(|&(a, b, c)| {
            (mesh.position(b) - mesh.position(a))
                .cross(mesh.position(c) - mesh.position(a))
                .length()
                / 2.0
        })
        .fold(f64::INFINITY, f64::min);
    assert!(smallest > 0.0, "no triangle of no area");

    // Wound as the input (the volume keeps its sign) and a convex fillet takes material away.
    let after = signed_volume(mesh);
    assert!(source > 0.0);
    assert!(after > 0.0);
    assert!(after < source);
}

#[test]
fn the_top_edge_fillets_at_every_fraction_of_the_room() {
    for fraction in [0.1, 0.5, 0.9, 1.0] {
        // The room, read by the build itself where the size is past it.
        let past = cut(15.0);
        assert_eq!(past.refusal, BandGapRefusal::Size);
        let size = fraction * past.room;

        let gap = cut(size);
        assert!(gap.built(), "{:?} at {size}", gap.refusal);
        let filled = gap_fill::fill(&gap, BandProfile::Fillet, 16).expect("the fill runs");

        // Builds right or refuses by name - never a part of the wrong shape. Measured: 0.9 builds
        // (takes 131.4); 0.5 and 1.0 refuse OpenChainEnds; 0.1 closed with a triangle of no area
        // and refuses FillLaysAFlatTriangle.
        if filled.refusal != GapFillRefusal::None {
            continue;
        }

        let (positions, indices, _) = part();
        assert_built_right(
            &filled.mesh.as_ref().expect("a filled mesh").borrow(),
            signed_volume_of(&positions, &indices),
        );
    }
}
