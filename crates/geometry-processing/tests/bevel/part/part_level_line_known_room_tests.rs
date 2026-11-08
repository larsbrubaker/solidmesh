// Which room, if any, the level-line route hands the kernel (`the_room_to_hand_the_build`). The
// build TRUSTS a room it is handed, so a room read off a pick that cannot be built AT ANY SIZE must
// never be handed in. Read at the one place the route makes the decision, with readings the kernel
// really produced, on a cube.

use std::rc::Rc;

use solidmesh::bevel::level_line::band_gap_entry_points::{build_pick, BandGapBuildOptions};
use solidmesh::bevel::level_line::band_gap_reading::read_room_of_pick;
use solidmesh::bevel::level_line::seam_edges::FeatureEdgeTest;
use solidmesh::bevel::level_line::{BandGapRefusal, BandProfile, WorkBudget};
use solidmesh::bevel::part::surface_part_builder::level_line_kernel_handoff::the_room_to_hand_the_build;
use solidmesh_math::Vector3;

/// A unit cube as bare positions and triangles - the smallest thing a chain can be picked on.
fn cube() -> (Vec<Vector3>, Vec<(i32, i32, i32)>) {
    let positions = vec![
        Vector3::new(-1.0, -1.0, -1.0),
        Vector3::new(1.0, -1.0, -1.0),
        Vector3::new(1.0, 1.0, -1.0),
        Vector3::new(-1.0, 1.0, -1.0),
        Vector3::new(-1.0, -1.0, 1.0),
        Vector3::new(1.0, -1.0, 1.0),
        Vector3::new(1.0, 1.0, 1.0),
        Vector3::new(-1.0, 1.0, 1.0),
    ];
    let indices = vec![
        (0, 2, 1),
        (0, 3, 2),
        (4, 5, 6),
        (4, 6, 7),
        (0, 1, 5),
        (0, 5, 4),
        (1, 2, 6),
        (1, 6, 5),
        (2, 3, 7),
        (2, 7, 6),
        (3, 0, 4),
        (3, 4, 7),
    ];
    (positions, indices)
}

fn every_edge(answer: bool) -> FeatureEdgeTest {
    Rc::new(move |_: i32, _: i32| answer)
}

/// A chain drawn across the middle of a face with nothing counted as a fold does not divide its
/// sides. The reading's room is zero and says nothing about size, so the route hands the build NO
/// room - and the build, left to read for itself, refuses by that pick's own name.
#[test]
fn a_pick_that_cannot_be_built_at_any_size_hands_the_build_no_room() {
    let (positions, indices) = cube();
    let across = vec![vec![4, 6]];
    let closed = [false];

    let reading = read_room_of_pick(
        &positions,
        &indices,
        &across,
        &closed,
        Some(every_edge(false)),
        BandProfile::Fillet,
        1e-9,
        None,
        f64::NAN,
        0.0,
        f64::INFINITY,
    )
    .expect("the pick is well formed");

    assert_eq!(
        reading.room, 0.0,
        "the reading of a pick that cannot be built carries a room of nothing"
    );

    let handed = the_room_to_hand_the_build(Some(&reading));

    assert!(
        handed.is_none(),
        "a room of nothing that is not about SIZE must not be handed to a build that trusts it"
    );

    // What handing it in would have cost the user: the refusal the pass falls through on is the
    // pick's own name, and it is only said because the build was left to read the mesh itself.
    let refused = build_pick(
        &positions,
        &indices,
        &across,
        &closed,
        Some(every_edge(false)),
        0.2,
        BandProfile::Fillet,
        &BandGapBuildOptions {
            chord_tolerance: 0.0,
            snap: 1e-9,
            known_room: handed,
            ..BandGapBuildOptions::default()
        },
    )
    .expect("the pick is well formed");

    // An OPEN chain with one region under it and no feature edge of its own has no crease to
    // bevel; ChainDoesNotDivideItsSides is the CLOSED chain's name for the same shape of nothing.
    assert_eq!(refused.refusal, BandGapRefusal::ChainHasNoCreaseToBevel);
}

/// The other half: a pick that CAN be built hands its room straight through, which is what makes
/// a drag cheap.
#[test]
fn a_pick_that_builds_hands_its_room_through() {
    let (positions, indices) = cube();
    let rim = vec![vec![4, 5, 6, 7]];

    let reading = read_room_of_pick(
        &positions,
        &indices,
        &rim,
        &[true],
        Some(every_edge(true)),
        BandProfile::Chamfer,
        1e-9,
        None,
        f64::NAN,
        0.0,
        f64::INFINITY,
    )
    .expect("the pick is well formed");

    let handed = the_room_to_hand_the_build(Some(&reading));

    assert!(handed.is_some());
    assert_eq!(handed.unwrap(), reading.room);
}

/// A reading that never finished knows nothing, and "nothing" is not a room either.
#[test]
fn a_reading_that_ran_out_of_work_hands_the_build_no_room() {
    let (positions, indices) = cube();
    let rim = vec![vec![4, 5, 6, 7]];

    // One unit of work buys nothing: the reading leaves by name rather than answering.
    let reading = read_room_of_pick(
        &positions,
        &indices,
        &rim,
        &[true],
        Some(every_edge(true)),
        BandProfile::Chamfer,
        1e-9,
        Some(Rc::new(WorkBudget::new(1).expect("a positive limit"))),
        f64::NAN,
        0.0,
        f64::INFINITY,
    )
    .expect("the pick is well formed");

    assert!(!reading.read);
    assert!(the_room_to_hand_the_build(Some(&reading)).is_none());
}
