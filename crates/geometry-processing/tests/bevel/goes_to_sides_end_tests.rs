//! "Fillet Should go to Sides and Stay Within Bounds.mcx" (and its twin "UpperLeftInner should
//! bevel.mcx") - a 14-vertex part whose long straight edges end on SLIVER end faces: the end face y
//! = 95 under c13 is 0.032 tall, from the chain at z = -20 up to a crease at z = -19.968 where
//! slanted faces begin. The Boolean construction builds these picks closed at every size the sweep
//! asks. Parameter rows are loops over the same-named test.
//!
//! Not run here (open work, with the measured reason): `TheChainOnTheEndKnotBuildsClosed` (fixture
//! c11: EndPlaneTrim refuses "trim 1: no end plane" at a nose where the rails meet at one vertex)
//! and `TheSideFaceRimBuildsClosed` (fixtures f0 and f5: room 0 set by Foreign at the rim corner
//! the groove's lower wall touches at the vertex alone).

use std::rc::Rc;

use crate::end_face_measures::to_triangle;
use crate::level_line_fixture::LevelLineFixture;
use crate::round_mouth_fillet_crossings::crossing_pairs;
use crate::sweep_mesh_measures;
use solidmesh::bevel::level_line::band_gap_entry_points::{self, BandGapBuildOptions};
use solidmesh::bevel::level_line::open_gap_fill::ends_site;
use solidmesh::bevel::level_line::{
    gap_fill, BandGapResult, BandProfile, GapFillRefusal, WorkBudget,
};

fn profile_of(fixture: &LevelLineFixture) -> BandProfile {
    if fixture.chamfer {
        BandProfile::Chamfer
    } else {
        BandProfile::Fillet
    }
}

/// One band build of the fixture at `size`, under its known room, seam epsilon, sharp-turn angle
/// and segment count.
fn build(fixture: &Rc<LevelLineFixture>, size: f64) -> BandGapResult {
    let options = BandGapBuildOptions {
        chord_tolerance: 0.0,
        snap: 1e-9,
        known_room: fixture.known_room,
        budget: Some(Rc::new(
            WorkBudget::new(WorkBudget::default_limit(fixture.indices.len() as i32))
                .expect("a positive limit"),
        )),
        seam_epsilon: fixture.seam_epsilon,
        sharp_turn_degrees: fixture.sharp_turn_degrees,
        fillet_segments: fixture.segments,
    };
    let features = Rc::clone(fixture);
    band_gap_entry_points::build_pick(
        &fixture.positions,
        &fixture.indices,
        &fixture.chains,
        &fixture.closed,
        Some(Rc::new(move |a, b| features.is_feature_edge(a, b))),
        size,
        profile_of(fixture),
        &options,
    )
    .expect("the pick is well formed")
}

/// c13's far end stands on the 0.032 sliver. The room used to read the sliver's far crease as a
/// foreign wall - a room of 0.0316 - and at a larger size the cut swallowed the whole sliver and
/// the end cap found no run. Past the sliver the band flows onto the next faces and the end is cut
/// there.
#[test]
fn the_chain_flows_over_the_sliver_end_face() {
    for size in [0.05, 0.1] {
        builds_closed_on_the_source("goes-to-sides-c13-r005", size, 0.5);
    }
}

/// c18 flows over its narrow end face too. The band carries the room it had WITHOUT the flow, so
/// where the flowed band's fill refuses the app answers with that size refusal ("about X fits")
/// rather than no result at all - the flow is an improvement only when it builds.
#[test]
fn a_flowed_band_carries_the_room_it_had_without_the_flow() {
    let fixture = Rc::new(LevelLineFixture::load("goes-to-sides-c18-r005"));
    let gap = build(&fixture, fixture.size);
    assert!(gap.built(), "band {:?}", gap.refusal);
    let unflowed = gap
        .unflowed_room
        .expect("the flowed band carries its unflowed room");
    assert!(unflowed < fixture.size);
    if let Some(known) = fixture.known_room {
        assert_eq!(unflowed, known);
    }
}

/// c17: the top edge of the groove, the 45-degree knife crease from (-64.984, 95.032, 0) to
/// (-45.016, 95.032, 0). Its two ends are mirror images - each stands on a slanted end wall with a
/// 0.035 edge along the top - but one end's top/end-wall crease lay on a FANNED top triangle, so
/// only the upper wall's concave crease was read there, that end alone took the end ray, and the
/// ray read the top's x = -65 crease 0.016 off along it as room 0 at every size. Both ends now read
/// alike, and each flows over its narrow end wall (decision 4): the band builds, carrying the room
/// it had without the flow (0.0065, the top's x = -65 crease read from the end vertex). The end
/// walls lean off square, so a leaning clip would stop the round on them; the pick flows only
/// because it is first built with square clips (`band_gap_entry_points`), and this test and
/// `the_knife_crease_builds_closed` fail without that guard.
#[test]
fn the_knife_crease_flows_over_both_narrow_end_walls() {
    for size in [0.05, 0.1] {
        let fixture = Rc::new(LevelLineFixture::load("goes-to-sides-c17-r005"));
        let gap = build(&fixture, size);
        assert!(
            gap.built(),
            "band {:?}, room {}, set by {:?}",
            gap.refusal,
            gap.room,
            gap.room_set_by
        );
        let unflowed = gap
            .unflowed_room
            .expect("the flowed band carries its unflowed room");
        assert!(unflowed > 0.005);
    }
}

/// c17 filled: closed, one shell, on the source. Each flowed end's hole - the strip's last row
/// against an end run that crosses the flowed end wall and comes back up the x = -65 face -
/// neither turns one way about the end vertex nor lies in a plane, so the old zip refused it
/// CornerCapNotStarShaped; the one filler closes it. The knife crease's faces meet at 45 degrees,
/// so its fillet's middle stands r(1 - sin 22.5) = 0.617 r off them along the whole chain - the
/// bound is that, not the right-angle 0.5 r.
#[test]
fn the_knife_crease_builds_closed() {
    for size in [0.05, 0.1] {
        builds_closed_on_the_source(
            "goes-to-sides-c17-r005",
            size,
            1.0 - (std::f64::consts::PI / 8.0).sin(),
        );
    }
}

fn builds_closed_on_the_source(name: &str, size: f64, reach: f64) {
    let fixture = Rc::new(LevelLineFixture::load(name));
    let gap = build(&fixture, size);
    assert!(
        gap.built(),
        "band {:?}, room {}, set by {:?}",
        gap.refusal,
        gap.room,
        gap.room_set_by
    );

    let filled =
        gap_fill::fill(&gap, profile_of(&fixture), fixture.segments).expect("the fill runs");
    assert_eq!(
        filled.refusal,
        GapFillRefusal::None,
        "{}",
        ends_site().unwrap_or_default()
    );

    let mesh = filled.mesh.as_ref().expect("a filled mesh").borrow();
    let kept = sweep_mesh_measures::kept(&mesh);
    assert!(sweep_mesh_measures::is_closed_and_consistent(
        mesh.vertices(),
        &kept
    ));
    assert_eq!(
        sweep_mesh_measures::shell_count(mesh.vertex_count(), &kept),
        1
    );
    assert_eq!(crossing_pairs(&mesh), 0);

    // Right in shape: a fillet stands at most r(1 - 1/sqrt 3) = 0.42 r off the faces it joins,
    // even where three meet; half r is past every blend, and a cut that went astray stands
    // further. A crease sharper than square stands r(1 - sin(angle / 2)) off, which the caller
    // passes as its reach.
    let mut used: Vec<i32> = kept.iter().flat_map(|&(a, b, c)| [a, b, c]).collect();
    used.sort_unstable();
    used.dedup();
    let at = mesh.vertices();
    let worst = used
        .iter()
        .map(|&v| {
            fixture
                .indices
                .iter()
                .map(|&(a, b, c)| {
                    to_triangle(
                        at[v as usize],
                        fixture.positions[a as usize],
                        fixture.positions[b as usize],
                        fixture.positions[c as usize],
                    )
                })
                .fold(f64::INFINITY, f64::min)
        })
        .fold(f64::NEG_INFINITY, f64::max);
    assert!(worst <= (size * reach) + 1e-9);
}
