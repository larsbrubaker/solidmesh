//! The room at a FADE end is a number, and it is the part's number. A pick of one or three top-rim
//! facet edges of a faceted cylinder fades out at both ends, and what limits it is the cap running
//! out of itself (or, on a short cylinder, the FOREIGN bottom rim). Parameter rows are loops over
//! the same-named test.

use solidmesh::bevel::level_line::band_gap_entry_points::{self, BandGapBuildOptions};
use solidmesh::bevel::level_line::band_gap_reading;
use solidmesh::bevel::level_line::gap_fill;
use solidmesh::bevel::level_line::{
    BandGapRefusal, BandGapResult, BandProfile, RoomReading, RoomReadingKind,
};
use solidmesh_math::Vector3;

use crate::level_line_feature_edges::feature_edges_over;
use crate::open_chain_fixtures;
use crate::several_chains_fixtures;
use crate::sweep_mesh_measures;

/// Low enough that the bottom rim is nearer the pick than anything else on the part.
const SHORT_HEIGHT: f64 = 3.0;

/// The rungs of the room a fade is held to build and fill at - the room itself
/// among them, for a chain that turns as well as for a straight one.
fn rungs_for(_edges: i32) -> [f64; 4] {
    [0.1, 0.5, 0.9, 1.0]
}

/// Every pick the tests run: one and three edges, each as a chamfer and as a fillet.
fn picks() -> Vec<(i32, BandProfile)> {
    let mut picks = Vec::new();
    for edges in [1, 3] {
        for profile in [BandProfile::Chamfer, BandProfile::Fillet] {
            picks.push((edges, profile));
        }
    }

    picks
}

/// One room reading of the fade pick.
fn read(
    positions: &[Vector3],
    indices: &[(i32, i32, i32)],
    edges: i32,
    profile: BandProfile,
) -> RoomReading {
    band_gap_reading::read_room(
        positions,
        indices,
        &open_chain_fixtures::top_rim_run(edges),
        false,
        Some(feature_edges_over(
            positions,
            indices,
            open_chain_fixtures::FOLD_DEGREES,
        )),
        profile,
        1e-9,
        None,
        0.0,
        f64::INFINITY,
    )
    .expect("the room reads")
}

/// One band build of the pick at a size.
fn build(
    positions: &[Vector3],
    indices: &[(i32, i32, i32)],
    edges: i32,
    profile: BandProfile,
    size: f64,
) -> BandGapResult {
    band_gap_entry_points::build(
        positions,
        indices,
        &open_chain_fixtures::top_rim_run(edges),
        false,
        Some(feature_edges_over(
            positions,
            indices,
            open_chain_fixtures::FOLD_DEGREES,
        )),
        size,
        profile,
        &BandGapBuildOptions::default(),
    )
    .expect("the pick builds or refuses by name")
}

/// Whether a build at the size fills closed.
fn fills_closed(
    positions: &[Vector3],
    indices: &[(i32, i32, i32)],
    edges: i32,
    profile: BandProfile,
    size: f64,
) {
    let gap = build(positions, indices, edges, profile, size);
    assert!(
        gap.built(),
        "{edges} {profile:?}: {size} is under the room: {:?}",
        gap.refusal
    );

    let fill =
        gap_fill::fill(&gap, profile, gap_fill::DEFAULT_FILLET_SEGMENTS).expect("the fill runs");
    assert!(
        fill.filled(),
        "{edges} {profile:?} {size}: a size the room holds must fill: {:?}",
        fill.refusal
    );

    let mesh = fill.mesh.as_ref().expect("a filled mesh").borrow();
    let kept = sweep_mesh_measures::kept(&mesh);
    let points: Vec<Vector3> = (0..mesh.vertex_count()).map(|v| mesh.position(v)).collect();
    assert!(sweep_mesh_measures::is_closed_and_consistent(
        &points, &kept
    ));
    assert_eq!(
        sweep_mesh_measures::shell_count(points.len() as i32, &kept),
        1
    );
    assert!(
        sweep_mesh_measures::signed_volume(&points, &kept) > 0.0,
        "the result is wound the way the input was"
    );
}

/// The reading itself - finite, positive, and
/// named by the CAP running out of itself.
#[test]
fn a_fade_end_reads_a_finite_room_set_by_the_foreign_rim() {
    for (edges, profile) in picks() {
        let (positions, indices) = open_chain_fixtures::cylinder();
        let reading = read(&positions, &indices, edges, profile);

        assert!(
            reading.room.is_finite(),
            "a fade is bounded by the part, not unbounded: read {}",
            reading.room
        );
        assert!(reading.room > open_chain_fixtures::facet_length());
        assert_eq!(reading.room_set_by.kind, RoomReadingKind::SelfRoom);

        if edges == 1 {
            // The cap is swallowed when the band reaches the far rim: the 40-gon's width across the
            // flats, measured from a chord of one facet, is 2 R cos(pi / sides).
            let across_the_flats = 2.0
                * open_chain_fixtures::RADIUS
                * (std::f64::consts::PI / open_chain_fixtures::SIDES as f64).cos();
            assert!(
                (reading.room - across_the_flats).abs() <= 1e-6,
                "{profile:?}: {} against {across_the_flats}",
                reading.room
            );
        } else {
            // Three facets read a little less: the VARYING branch's climb, a reading to the self
            // room's stated relative accuracy rather than an exact number.
            assert!(
                (19.7..=19.94).contains(&reading.room),
                "{profile:?}: {}",
                reading.room
            );
        }
    }
}

/// Every rung of the room builds and fills closed,
/// one shell, wound as the input.
#[test]
fn a_fade_builds_and_fills_at_every_rung_of_its_room() {
    for (edges, profile) in picks() {
        let (positions, indices) = open_chain_fixtures::cylinder();
        let room = read(&positions, &indices, edges, profile).room;

        for rung in rungs_for(edges) {
            fills_closed(&positions, &indices, edges, profile, rung * room);
        }
    }
}

/// A hair over the room is refused BY SIZE, and the
/// refusal names that very room.
#[test]
fn over_the_room_a_fade_is_refused_by_size_naming_it() {
    for (edges, profile) in picks() {
        let (positions, indices) = open_chain_fixtures::cylinder();
        let room = read(&positions, &indices, edges, profile).room;

        let gap = build(&positions, &indices, edges, profile, 1.1 * room);
        assert!(!gap.built(), "1.1 of the room is more than the part holds");
        assert_eq!(gap.refusal, BandGapRefusal::Size);

        // ONE reading: the build reads the room the same way this test did - on the input mesh, at
        // the reference size - so the number in the refusal is the number that was read.
        assert!(
            (gap.room - room).abs() <= 1e-9,
            "{edges} {profile:?}: {} against {room}",
            gap.room
        );
    }
}

/// The same part standing somewhere else reads
/// the same room and does the same thing at every rung.
#[test]
fn the_room_and_every_rung_are_unchanged_by_a_rigid_motion() {
    for (edges, profile) in picks() {
        let (positions, indices) = open_chain_fixtures::cylinder();
        let (moved, same_indices) = open_chain_fixtures::moved(several_chains_fixtures::motion());

        let room = read(&positions, &indices, edges, profile).room;
        let moved_room = read(&moved, &same_indices, edges, profile).room;
        assert!(
            (moved_room - room).abs() <= 1e-9 * room,
            "{edges} {profile:?}: {moved_room} against {room}"
        );

        for rung in rungs_for(edges) {
            fills_closed(&moved, &same_indices, edges, profile, rung * room);
        }

        let over = build(&moved, &same_indices, edges, profile, 1.1 * room);
        assert_eq!(over.refusal, BandGapRefusal::Size);
    }
}

/// A FOREIGN edge is what limits a fade where
/// nothing else is near - the bottom rim of a cylinder only 3 tall, less the margin the surviving
/// strip needs.
#[test]
fn a_short_cylinder_reads_the_bottom_rim_as_its_room() {
    for (edges, profile) in picks() {
        let (positions, indices) = open_chain_fixtures::cylinder_of_height(SHORT_HEIGHT);
        let reading = read(&positions, &indices, edges, profile);

        assert_eq!(reading.room_set_by.kind, RoomReadingKind::Foreign);
        let expected = SHORT_HEIGHT - band_gap_reading::room_margin(1.0, 1e-9);
        assert!(
            (reading.room - expected).abs() <= 1e-9,
            "{edges} {profile:?}: {} against {expected}",
            reading.room
        );

        for rung in rungs_for(edges) {
            fills_closed(&positions, &indices, edges, profile, rung * reading.room);
        }

        let over = build(&positions, &indices, edges, profile, 1.1 * reading.room);
        assert_eq!(over.refusal, BandGapRefusal::Size);
    }
}
