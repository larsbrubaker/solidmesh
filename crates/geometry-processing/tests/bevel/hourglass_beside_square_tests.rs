//! RUNG 5 over a pick of SEVERAL chains, on a fixture built for it
//! (`hourglass_beside_square_fixture.rs`) - when the pick's topology does not stand at the size
//! asked, the pinch ladder runs over the WHOLE pick, so the size the refusal names is a size every
//! chain of the pick built at, and building there really does come back a closed solid. A chain
//! which never pinches (the square boss) is picked alongside one that does (the hourglass), so a
//! ladder that probed only the offending chain, or that stopped at the room, would name a size the
//! pick cannot be built at. Parameter rows are loops over the same-named test.

use solidmesh::bevel::level_line::band_gap_reading::room_margin;
use solidmesh::bevel::level_line::gap_fill;
use solidmesh::bevel::level_line::{BandGapRefusal, BandProfile, RoomSource};
use solidmesh_math::Vector3;

use crate::hourglass_beside_square_fixture::{
    build, hourglass_chain, pick, pick_with, read, NECK_HALF_WIDTH,
};
use crate::level_line_test_geometry::open_edges;
use crate::sweep_mesh_measures;

/// The pick's room is a reading of WIDTH and says nothing of the neck: it is the SELF room of a
/// picked face - how deep a point in it stands from its own rim - and it stands well above the
/// neck's half width, which is why the build below has to find the pinch for itself.
///
/// It is the SQUARE boss's face (chain 1) that sets it, and that is the point of the fixture: the
/// chain that never pinches owns the room. Both faces are 4 across their narrow way - the
/// hourglass's lobes and the square - so both self candidates read the same 2 less the margin, and
/// the least-wins tie falls to the later chain.
#[test]
fn the_picks_room_is_a_width_above_the_necks_half_width() {
    let (positions, indices, chains) = pick();

    let reading = read(&positions, &indices, &chains, BandProfile::Chamfer);

    assert!(reading.read);
    assert!(
        reading.room > NECK_HALF_WIDTH,
        "the width reading never looks at the neck's saddle: {} set by {:?}",
        reading.room,
        reading.room_set_by
    );
    assert_eq!(
        reading.room_set_by,
        RoomSource::self_room(1),
        "a picked face's own depth is the narrowest thing the width reading sees"
    );
    assert!(
        (reading.room - (2.0 - room_margin(1.0, 1e-9))).abs() <= 1e-9,
        "both faces are 4 across their narrow way: {}",
        reading.room
    );
    let least = reading
        .candidates
        .iter()
        .map(|c| c.room)
        .fold(f64::INFINITY, f64::min);
    assert_eq!(least, reading.room, "the room IS the least of the table");
}

/// A size between the neck's half width and the room refuses BY SIZE, and what it names is a size a
/// probe of the WHOLE pick built at: positive, below the size asked, and within the ladder's promise
/// of the true limit - three bisections of a factor-of-two bracket, so never worse than 8/9 of it.
#[test]
fn a_size_above_the_neck_refuses_naming_a_size_the_pick_built_at() {
    let (positions, indices, chains) = pick();
    let room = read(&positions, &indices, &chains, BandProfile::Chamfer).room;
    let neck = NECK_HALF_WIDTH;
    let asked = (neck + room) / 2.0;

    let refused = build(&positions, &indices, &chains, asked, BandProfile::Chamfer);

    assert_eq!(
        refused.refusal,
        BandGapRefusal::Size,
        "at {asked} the hourglass's face is cut in two across its neck"
    );
    assert!(refused.room > 0.0);
    assert!(refused.room < asked);
    assert!(
        refused.room <= neck + 1e-9,
        "no size above the neck's half width builds: {}",
        refused.room
    );
    assert!(
        refused.room * 9.0 / 8.0 >= neck,
        "the ladder's answer is within an eighth of the true limit: {}",
        refused.room
    );
    assert!(
        refused.pinch_probes <= 9,
        "a pinch refusal is bought in a handful of builds: {}",
        refused.pinch_probes
    );
    assert!(!refused.pinches_at_every_size_tried);
}

/// The pinch refusal NAMES the chain that pinched - the hourglass, in either order - so a caller
/// can drop that one loop and build the rest, instead of an unnamed refusal failing the whole pick
/// (Many Holes: 37 port mouths refused together at 0.234 with nobody named).
#[test]
fn the_pinch_refusal_names_the_chain_that_pinched() {
    for square_first in [false, true] {
        let (positions, indices, chains) = pick_with(square_first, false);
        let room = read(&positions, &indices, &chains, BandProfile::Chamfer).room;
        let asked = (NECK_HALF_WIDTH + room) / 2.0;

        let refused = build(&positions, &indices, &chains, asked, BandProfile::Chamfer);

        assert_eq!(refused.refusal, BandGapRefusal::Size);
        assert_eq!(
            refused.room_set_by,
            RoomSource::self_room(hourglass_chain(square_first)),
            "the hourglass's own face closes across its neck, touching no other chain"
        );
    }
}

/// And the size it names BUILDS the whole pick: every (chain, side) leaves its one closed loop,
/// wholly its own, and the gap fills to a closed, consistently oriented, one-piece solid - either
/// profile.
#[test]
fn the_size_the_refusal_names_builds_and_fills_closed() {
    for profile in [BandProfile::Chamfer, BandProfile::Fillet] {
        let (positions, indices, chains) = pick();
        let room = read(&positions, &indices, &chains, profile).room;
        let asked = (NECK_HALF_WIDTH + room) / 2.0;
        let named = build(&positions, &indices, &chains, asked, profile).room;

        assert_builds_and_fills_closed(&positions, &indices, &chains, named, profile);
    }
}

/// Neither the ORDER the two rims were named in nor where the part STANDS is part of the answer:
/// the same size named, and a build there that closes just the same.
///
/// The ladder's sizes are halvings and midpoints of the size asked, so both readings probe the very
/// same sizes; a rigid motion can only move the answer if one of those probes' loop count flips
/// under it, which is a whole bisection step - an eighth of the bracket. That is the tolerance held
/// here, and the size named is then asserted to BUILD where it was named.
#[test]
fn the_order_of_the_rims_and_where_the_part_stands_say_nothing() {
    for (square_first, moved) in [(true, false), (false, true)] {
        let straight = pick();
        let other = pick_with(square_first, moved);

        let room = read(&straight.0, &straight.1, &straight.2, BandProfile::Chamfer).room;
        let asked = (NECK_HALF_WIDTH + room) / 2.0;

        let here = build(
            &straight.0,
            &straight.1,
            &straight.2,
            asked,
            BandProfile::Chamfer,
        );
        let there = build(&other.0, &other.1, &other.2, asked, BandProfile::Chamfer);

        assert_eq!(there.refusal, here.refusal);
        assert!(
            (there.room - here.room).abs() <= (here.room.max(there.room) / 8.0) + 1e-9,
            "the neck is a reading of the part, not of its order or its place: {} / {}",
            here.room,
            there.room
        );

        assert_builds_and_fills_closed(
            &other.0,
            &other.1,
            &other.2,
            there.room,
            BandProfile::Chamfer,
        );
    }
}

/// The whole of what "a size that builds" means for a pick of several: one closed owned loop per
/// (chain, side), and a fill that leaves a closed, consistently oriented, single shell.
fn assert_builds_and_fills_closed(
    positions: &[Vector3],
    indices: &[(i32, i32, i32)],
    chains: &[Vec<i32>],
    size: f64,
    profile: BandProfile,
) {
    let at = build(positions, indices, chains, size, profile);

    assert_eq!(
        at.refusal,
        BandGapRefusal::None,
        "the size the ladder named is a size a probe built at: {size}"
    );
    assert_eq!(at.loops.len(), 4, "two chains, two sides each");
    for gap_loop in &at.loops {
        assert!(gap_loop.is_closed);
        assert!(!gap_loop.has_mixed_owners, "a mixed loop IS the pinch");
    }

    let mut owners: Vec<(i32, i32)> = at.loops.iter().map(|l| (l.chain, l.side)).collect();
    owners.sort_unstable();
    owners.dedup();
    assert_eq!(
        owners.len(),
        4,
        "one loop per (chain, side), not two of one and none of another"
    );

    let filled = gap_fill::fill(&at, profile, 6).expect("the fill runs");

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
