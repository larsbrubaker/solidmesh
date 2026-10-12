//! Ports `Tests/GeometryProcessingTests/LevelLine/StairStepChainTests.cs`: the owner file
//! `Stair Step Chain.mcx` as the app route hands it to the kernel - five step edges picked as five
//! open one-edge chains, every chain ending on the two end faces - and the corner fixtures `Inner
//! Right Sides and Center.mcx` (three open chains meeting at one mixed corner) and `Center Right
//! and Left Side.mcx` (that corner with a face rim beside it). C# `[Arguments]` rows are loops over
//! the same-named test.

use std::rc::Rc;

use geometry_processing::bevel::level_line::band_gap_entry_points;
use geometry_processing::bevel::level_line::band_gap_in_turn::{
    build_open_chains_in_turn, takes_open_chains_in_turn,
};
use geometry_processing::bevel::level_line::band_gap_reading;
use geometry_processing::bevel::level_line::corner_cap_hole_fill::last_hole_fill_failure;
use geometry_processing::bevel::level_line::corner_cap_hole_fill_switch::set_use_hole_fill_override;
use geometry_processing::bevel::level_line::{
    gap_fill, BandGapRefusal, BandGapResult, GapFillRefusal, GapFillResult, SeamEdges,
};
use vector_math::Vector3;

use crate::end_face_builds::{as_saved, feature_of};
use crate::fillet_thin_front_fin::profile_of;
use crate::level_line_fixture::LevelLineFixture;
use crate::level_line_test_geometry::{signed_volume, signed_volume_of};
use crate::sweep_mesh_measures::{is_closed_and_consistent, kept, shell_count, smallest_area};

const FIXTURE: &str = "stair-step-chain-op0";

const CORNER_FIXTURE: &str = "inner-right-sides-and-center-op0";

const FRACTIONS: [f64; 4] = [0.1, 0.5, 0.9, 1.0];

/// C# `AnEdgeBetweenTwoChainsEndsStaysAFeature`.
#[test]
fn an_edge_between_two_chains_ends_stays_a_feature() {
    let fixture = Rc::new(LevelLineFixture::load(FIXTURE));
    let feature = SeamEdges::not_the_chains_seam(
        Some(&fixture.positions),
        Some(&fixture.chains),
        Some(&fixture.closed),
        Some(feature_of(&fixture)),
        fixture.seam_epsilon,
    )
    .expect("a feature test");

    // v4 (20, 6, 10) ends chain 0 and v5 (30, 6, 10) ends chain 1: the end face's edge along the
    // tread.
    assert!(fixture.is_feature_edge(4, 5));
    assert!(feature(4, 5));
}

/// C# `TheStairsFiveStepEdgesBuildAtEveryFractionOfTheRoom`.
#[test]
fn the_stairs_five_step_edges_build_at_every_fraction_of_the_room() {
    for fraction in FRACTIONS {
        let fixture = Rc::new(LevelLineFixture::load(FIXTURE));
        let read = build(&fixture, &fixture.positions, fixture.size, None);
        assert_eq!(
            read.refusal,
            BandGapRefusal::None,
            "{:?} chain {}",
            read.refusal,
            read.offending_chain
        );
        let room = read.room;
        assert!(room > 0.0);

        let size = fraction * room;
        let gap = build(&fixture, &fixture.positions, size, None);
        assert!(
            gap.built(),
            "{:?} chain {}, room {:?}",
            gap.refusal,
            gap.offending_chain,
            gap.room
        );
        assert!(takes_open_chains_in_turn(Some(&gap), &fixture.closed));
        let filled = in_turn(&fixture, &fixture.positions, size, room);
        assert_eq!(
            filled.refusal,
            GapFillRefusal::None,
            "{:?} on chain {}",
            filled.refusal,
            filled.offending_chain
        );

        let mesh = filled.mesh.as_ref().expect("a filled mesh").borrow();
        let kept = kept(&mesh);
        assert!(is_closed_and_consistent(&mesh.vertices, &kept));
        assert_eq!(shell_count(mesh.vertices.len() as i32, &kept), 1);
        assert!(smallest_area(&mesh.vertices, &kept) > 0.0);

        let source = signed_volume_of(&fixture.positions, &fixture.indices);
        let result = signed_volume(&mesh);
        assert!(source > 0.0);
        assert!(result > 0.0);

        let mut alone_sum = 0.0;
        for chain in 0..fixture.chains.len() {
            let alone = fill_alone(&fixture, chain, size);
            assert_eq!(alone.refusal, GapFillRefusal::None);
            let moved = signed_volume(&alone.mesh.as_ref().expect("a mesh").borrow()) - source;
            let concave = chain == 0 || chain == 2;
            assert!(
                if concave { moved > 0.0 } else { moved < 0.0 },
                "chain {chain} moved {moved:?}"
            );
            alone_sum += moved;
        }

        assert!(((result - source) - alone_sum).abs() <= 1e-6 * source.abs());
    }
}

/// C# `ThreeOpenChainsAtOneMixedCornerCutTheirBandBelowTheirRoom`.
#[test]
fn three_open_chains_at_one_mixed_corner_cut_their_band_below_their_room() {
    for fraction in FRACTIONS {
        let fixture = Rc::new(LevelLineFixture::load(CORNER_FIXTURE));
        let room = room_of(&fixture);

        // Boolean builds the app's size, 2; the room read 1.67 while it counted chain 2's second
        // edge, running away from the convex chain they share v9 with, as an approach.
        assert!(room > fixture.size);
        let gap = build(&fixture, &fixture.positions, fraction * room, None);
        assert!(
            gap.built(),
            "{:?} in {:?}, room {:?} set by {:?}",
            gap.refusal,
            gap.phase,
            gap.room,
            gap.room_set_by
        );
    }
}

/// C# `ThreeOpenChainsAtOneMixedCornerBuild`.
#[test]
fn three_open_chains_at_one_mixed_corner_build() {
    for fraction in FRACTIONS {
        let fixture = Rc::new(LevelLineFixture::load(CORNER_FIXTURE));
        let room = room_of(&fixture);
        let size = fraction * room;
        let source = signed_volume_of(&fixture.positions, &fixture.indices);
        assert!(source > 0.0);

        let filled = fill_as_the_app_does(&fixture, &fixture.positions, size, room)
            .expect("the band builds");
        println!(
            "stair step corner at {fraction} of room {room:?}: fill {:?} ({:?}), chain {}, vertex {}",
            filled.refusal,
            last_hole_fill_failure(),
            filled.offending_chain,
            filled.offending_vertex
        );
        assert_eq!(
            filled.refusal,
            GapFillRefusal::None,
            "{:?} on chain {}",
            filled.refusal,
            filled.offending_chain
        );

        let mesh = filled.mesh.as_ref().expect("a filled mesh").borrow();
        let kept = kept(&mesh);
        assert!(is_closed_and_consistent(&mesh.vertices, &kept));
        assert_eq!(shell_count(mesh.vertices.len() as i32, &kept), 1);
        assert!(smallest_area(&mesh.vertices, &kept) > 1e-12);

        let result = signed_volume(&mesh);
        assert!(result > 0.0);
        assert_ne!(result, source);

        // Moved the way the chains alone move it.
        let mut alone_sum = 0.0;
        let mut alone_built = 0;
        for chain in 0..fixture.chains.len() {
            let alone = fill_alone(&fixture, chain, size);
            if alone.refusal == GapFillRefusal::None {
                alone_sum += signed_volume(&alone.mesh.as_ref().expect("a mesh").borrow()) - source;
                alone_built += 1;
            }
        }

        println!(
            "stair step corner at {fraction}: volume {result:?} of {source:?} (moved {:?}); chains alone {alone_built}/{}, summed move {alone_sum:?}",
            result - source,
            fixture.chains.len()
        );
        if alone_built == fixture.chains.len() && alone_sum.abs() > 1e-9 * source {
            assert_eq!(sign(result - source), sign(alone_sum));
        }

        // Unchanged under a rigid motion: the same pick turned and moved builds the same volume.
        let moved_positions: Vec<Vector3> = fixture.positions.iter().map(|&p| move_it(p)).collect();
        let moved_fill = fill_as_the_app_does(&fixture, &moved_positions, size, room)
            .expect("the moved band builds");
        assert_eq!(
            moved_fill.refusal,
            GapFillRefusal::None,
            "moved: {:?} on chain {}",
            moved_fill.refusal,
            moved_fill.offending_chain
        );
        let moved_volume = signed_volume(&moved_fill.mesh.as_ref().expect("a mesh").borrow());
        assert!((moved_volume - result).abs() <= 1e-6 * source);
    }
}

/// C# `Math.Sign`.
fn sign(v: f64) -> i32 {
    if v > 0.0 {
        1
    } else if v < 0.0 {
        -1
    } else {
        0
    }
}

/// C# `FillAsTheAppDoes`: the pick filled as the app fills it, with the corner filler on; `None`
/// when the band does not build.
fn fill_as_the_app_does(
    fixture: &Rc<LevelLineFixture>,
    positions: &[Vector3],
    size: f64,
    room: f64,
) -> Option<GapFillResult> {
    let gap = build(fixture, positions, size, None);
    if !gap.built() {
        return None;
    }

    set_use_hole_fill_override(Some(true));
    let filled = if takes_open_chains_in_turn(Some(&gap), &fixture.closed) {
        in_turn(fixture, positions, size, room)
    } else {
        gap_fill::fill(&gap, profile_of(fixture), fixture.segments)
            .expect("the fill fills or refuses by name")
    };
    set_use_hole_fill_override(None);
    Some(filled)
}

/// C# `BandGap.BuildOpenChainsInTurn(..., fixture.Segments, room, snap: 1e-9, seamEpsilon,
/// sharpTurnDegrees)`.
fn in_turn(
    fixture: &Rc<LevelLineFixture>,
    positions: &[Vector3],
    size: f64,
    room: f64,
) -> GapFillResult {
    build_open_chains_in_turn(
        positions,
        &fixture.indices,
        &fixture.chains,
        Some(feature_of(fixture)),
        size,
        profile_of(fixture),
        fixture.segments,
        Some(room),
        1e-9,
        fixture.seam_epsilon,
        fixture.sharp_turn_degrees,
        None,
        None,
    )
    .expect("the pick fills or refuses by name")
}

/// One chain built alone and filled (C#'s `GapFill.Fill(BandGap.Build(...new[] { chain }...))`).
fn fill_alone(fixture: &Rc<LevelLineFixture>, chain: usize, size: f64) -> GapFillResult {
    let gap = band_gap_entry_points::build_pick(
        &fixture.positions,
        &fixture.indices,
        &[fixture.chains[chain].clone()],
        &[fixture.closed[chain]],
        Some(feature_of(fixture)),
        size,
        profile_of(fixture),
        &as_saved(fixture, None),
    )
    .expect("the chain builds or refuses by name");
    gap_fill::fill(&gap, profile_of(fixture), fixture.segments)
        .expect("the fill fills or refuses by name")
}

/// C# `Move`.
fn move_it(p: Vector3) -> Vector3 {
    const ANGLE: f64 = 0.7;
    let (sin, cos) = ANGLE.sin_cos();
    Vector3::new(
        (p.x * cos) - (p.y * sin) + 13.25,
        (p.x * sin) + (p.y * cos) - 7.5,
        p.z + 3.125,
    )
}

/// C# `ARimBesideTheCornerRefusesByNameNotBySize`.
#[test]
fn a_rim_beside_the_corner_refuses_by_name_not_by_size() {
    let fixture = Rc::new(LevelLineFixture::load("center-right-and-left-side-op0"));
    let room = room_of(&fixture);
    assert!(room > fixture.size);
    let gap = build(&fixture, &fixture.positions, fixture.size, Some(room));
    assert_eq!(
        gap.refusal,
        BandGapRefusal::SidesLeaveNoLoopAtAnySize,
        "{:?}, room {:?}",
        gap.refusal,
        gap.room
    );
    assert_eq!(gap.room, 0.0);
}

/// C# `RoomOf`: the pick's room, read as the app reads it.
fn room_of(fixture: &Rc<LevelLineFixture>) -> f64 {
    band_gap_reading::read_room_of_pick(
        &fixture.positions,
        &fixture.indices,
        &fixture.chains,
        &fixture.closed,
        Some(feature_of(fixture)),
        profile_of(fixture),
        1e-9,
        None,
        f64::NAN,
        fixture.seam_epsilon,
        fixture.sharp_turn_degrees,
    )
    .expect("the pick reads")
    .room
}

/// C# `Build`: the whole pick at `size` on `positions`, with the given known room.
fn build(
    fixture: &Rc<LevelLineFixture>,
    positions: &[Vector3],
    size: f64,
    known_room: Option<f64>,
) -> BandGapResult {
    band_gap_entry_points::build_pick(
        positions,
        &fixture.indices,
        &fixture.chains,
        &fixture.closed,
        Some(feature_of(fixture)),
        size,
        profile_of(fixture),
        &as_saved(fixture, known_room),
    )
    .expect("the pick builds or refuses by name")
}
