//! Port of `LevelLine/SelfRoomVaryingSetbackTests.cs`: where a chain's unit setback VARIES along
//! it, a side's self room must never read HIGH - a room that reads above the size the side really
//! survives to is a size the user is allowed and that then fails. The reading turns the field's
//! greatest point into a room by the swallow size `g(p) = min over c of |p - c| / u(c)` and climbs,
//! rather than dividing by the unit of the additive problem's minimiser (which answered 1.289 and
//! refused its own room). C# `[Arguments]` rows are loops over the same-named test.

use geometry_processing::bevel::level_line::chain_on_the_mesh;
use geometry_processing::bevel::level_line::gap_fill;
use geometry_processing::bevel::level_line::{BandGapRefusal, BandProfile, ChainField, RoomSource};
use vector_math::Vector3;

use crate::level_line_test_geometry::open_edges;
use crate::raked_box_fixture;
use crate::several_chains_fixtures;
use crate::sweep_mesh_measures;

/// The fixture must really vary, or every claim below is about a uniform chain.
#[test]
fn the_rims_unit_setback_varies_between_its_sides() {
    for square_first in [false, true] {
        let (positions, indices, rim) = raked_box_fixture::pick(square_first, None);

        let (lowest, highest) = raked_box_fixture::unit_setbacks(&positions, &indices, &rim);

        assert!(
            highest > lowest,
            "the rake is what makes the fillet's setback vary: {lowest} to {highest}"
        );
        assert!(
            (highest - raked_box_fixture::square_unit()).abs() <= 1e-9,
            "three square edges ask the radius itself"
        );
        assert!(
            (lowest - raked_box_fixture::raked_unit()).abs() <= 1e-9,
            "the 45 degree wall's edge asks tan(22.5 degrees) of it"
        );
    }
}

/// The room read of the raked rim is AT OR UNDER the size the top face really survives to, and
/// close to it. The bound is the whole point: a room above it is a size that refuses nothing and
/// then fails.
#[test]
fn the_room_never_reads_above_what_the_face_survives() {
    for square_first in [false, true] {
        let (positions, indices, rim) = raked_box_fixture::pick(square_first, None);

        let reading = raked_box_fixture::read(&positions, &indices, &rim, BandProfile::Fillet);

        assert!(reading.read, "the rim reads");
        assert_eq!(
            reading.room_set_by,
            RoomSource::self_room(0),
            "the strip's own width is the narrowest thing here: {:?} at {}",
            reading.room_set_by,
            reading.room
        );
        let true_room = raked_box_fixture::true_room();
        assert!(
            reading.room <= true_room,
            "a room over {true_room} is a size that then fails: {}",
            reading.room
        );
        assert!(
            reading.room > true_room * 0.95,
            "and it is a reading, not a shrug - the climb settles within a twentieth of the swallow size, the search's own peak tolerance: {}",
            reading.room
        );
    }
}

/// The same room, whatever the part is turned to - the reading is of the part, not of its pose.
#[test]
fn the_room_is_the_same_under_a_rigid_motion() {
    let straight = raked_box_fixture::pick(false, None);
    let moved = raked_box_fixture::pick(false, Some(several_chains_fixtures::motion()));

    let here = raked_box_fixture::read(&straight.0, &straight.1, &straight.2, BandProfile::Fillet);
    let there = raked_box_fixture::read(&moved.0, &moved.1, &moved.2, BandProfile::Fillet);

    assert!(
        (there.room - here.room).abs() <= 1e-6,
        "here {}, turned {}",
        here.room,
        there.room
    );
    assert!(
        there.room <= raked_box_fixture::true_room(),
        "turned, the bound holds too: {}",
        there.room
    );
}

/// And the room BUILDS: at half of it, at nine tenths of it and AT it the band comes out and the
/// gap fills closed, consistently oriented, one shell. The build is handed the reading as its known
/// room, because a single chain's build otherwise re-reads its own room on the refined mesh at the
/// size asked - a different reading, and the next rung's business. The row at 1.0 is what the old
/// division could not do.
#[test]
fn the_room_builds_and_fills_closed() {
    for fraction in [0.5, 0.9, 1.0] {
        let (positions, indices, rim) = raked_box_fixture::pick(false, None);
        let room = raked_box_fixture::read(&positions, &indices, &rim, BandProfile::Fillet).room;
        let asked = room * fraction;

        let cut = raked_box_fixture::build(
            &positions,
            &indices,
            &rim,
            asked,
            Some(room),
            BandProfile::Fillet,
        );

        assert!(cut.built(), "refusal {:?} at {asked}", cut.refusal);

        let filled = gap_fill::fill(&cut, BandProfile::Fillet, 6).expect("the fill runs");

        assert!(filled.filled(), "refusal {:?}", filled.refusal);
        let mesh = filled.mesh.as_ref().expect("a filled mesh").borrow();
        assert!(open_edges(&mesh).is_empty(), "closed");

        let kept = sweep_mesh_measures::kept(&mesh);
        let points: Vec<Vector3> = (0..mesh.vertex_count()).map(|v| mesh.position(v)).collect();
        assert!(
            sweep_mesh_measures::is_closed_and_consistent(&points, &kept),
            "every directed edge once and its opposite once"
        );
        assert_eq!(
            sweep_mesh_measures::shell_count(mesh.vertex_count(), &kept),
            1,
            "one shell"
        );
    }
}

/// A point standing ON the chain is swallowed at NO size at all, and the reading says zero rather
/// than unbounded. A sliver side - a face collapsed onto its own rim - is the case, and zero is the
/// one answer that refuses every size, where unbounded would allow every size.
#[test]
fn a_point_on_the_chain_is_swallowed_at_zero() {
    let (positions, indices, rim) = raked_box_fixture::pick(false, None);
    let edges: Vec<(i32, i32)> = (0..rim.len())
        .map(|i| (rim[i], rim[(i + 1) % rim.len()]))
        .collect();

    let units = chain_on_the_mesh::unit_setbacks(
        &positions,
        &indices,
        &rim,
        true,
        &edges,
        BandProfile::Fillet,
    )
    .expect("every rim edge stands on two triangles");
    let points: Vec<Vector3> = rim.iter().map(|&v| positions[v as usize]).collect();
    let field = ChainField::from_polyline(&points, true, &units).expect("a valid chain");

    let at = positions[rim[0] as usize];
    assert!(
        field.swallow_scale(at).expect("no budget").abs() <= 1e-12,
        "a corner of the rim is already on the chain"
    );
    assert!(
        field
            .swallow_scale(at + Vector3::new(0.0, 0.0, -1e-9))
            .expect("no budget")
            < 1e-8,
        "and a point a hair off it is swallowed by a hair of a size"
    );
}

/// A hair above the room refuses BY SIZE, which is what the room is for.
#[test]
fn just_above_the_room_refuses_by_size() {
    let (positions, indices, rim) = raked_box_fixture::pick(false, None);
    let room = raked_box_fixture::read(&positions, &indices, &rim, BandProfile::Fillet).room;

    let refused = raked_box_fixture::build(
        &positions,
        &indices,
        &rim,
        room * 1.001,
        Some(room),
        BandProfile::Fillet,
    );

    assert_eq!(
        refused.refusal,
        BandGapRefusal::Size,
        "1.001 of {room} is over the room"
    );
}
