// A pick of many loops settled in a FEW readings rather than one loop a pass - read at the route's
// own settling step (`BevelSurfacePartBuilder::the_settled_pick`) on the shared five-hole plate
// fixture (`five_hole_plate_fixture`).

use std::collections::HashSet;

use solidmesh::bevel::level_line::BandProfile;
use solidmesh::bevel::part::surface_part_builder::BevelSurfacePartBuilder;

use crate::five_hole_plate_fixture as fixture;

/// The five-hole plate at the size the fixture is about: the two end holes are too near the
/// plate's own edge, and the settling names BOTH of them - with the outer rim and the middle three
/// left to build - in a handful of readings.
#[test]
fn every_chain_that_cannot_stay_is_named_at_once() {
    let (positions, indices, chains) = fixture::pick();
    let closed = vec![true; chains.len()];

    let settled = BevelSurfacePartBuilder::the_settled_pick(
        &positions,
        &indices,
        &chains,
        &closed,
        Some(fixture::feature_edges(&positions, &indices)),
        BandProfile::Chamfer,
        fixture::SIZE,
        None,
        0.0,
        f64::INFINITY,
    )
    .expect("the pick is well formed")
    .expect("a pick whose chains do not touch is settled");

    let mut condemned: Vec<i32> = settled.condemned.iter().map(|one| one.chain).collect();
    condemned.sort();
    assert_eq!(
        condemned,
        fixture::TOO_NEAR_THE_EDGE.to_vec(),
        "the two end holes are the chains that cannot be built at this size"
    );

    let mut chains_left = settled.chains.clone();
    chains_left.sort();
    assert_eq!(
        chains_left,
        vec![0, 2, 3, 4],
        "everything that fits stays - the outer rim and the middle three holes"
    );

    assert!(
        settled.fits,
        "what is left really does read at or above the size"
    );

    // Two rounds is what the plate settles in; the bound is the number of readings the route may
    // pay rather than an exact count.
    assert!(
        settled.readings <= 3,
        "a pick of six chains took {} readings to settle",
        settled.readings
    );
}

/// Every condemned chain comes back with the SIZE it fits at beside its neighbours - under the
/// size that was asked and above nothing.
#[test]
fn a_condemned_chain_carries_the_size_it_fits_at() {
    let (positions, indices, chains) = fixture::pick();

    let settled = BevelSurfacePartBuilder::the_settled_pick(
        &positions,
        &indices,
        &chains,
        &vec![true; chains.len()],
        Some(fixture::feature_edges(&positions, &indices)),
        BandProfile::Chamfer,
        fixture::SIZE,
        None,
        0.0,
        f64::INFINITY,
    )
    .expect("the pick is well formed")
    .expect("a pick whose chains do not touch is settled");

    for one in &settled.condemned {
        assert!(
            one.room > 0.0,
            "chain {} was condemned with no number to offer the user",
            one.chain
        );

        assert!(
            one.room < fixture::SIZE,
            "chain {} was condemned, so it cannot fit at the size that was asked",
            one.chain
        );
    }
}

/// A pick where two chains TOUCH is NOT the settling's to judge: it declines, and the pass takes
/// the path it took before this step existed.
#[test]
fn a_pick_where_chains_touch_is_not_settled_at_all() {
    let (positions, indices, chains) = fixture::pick();

    // A run made to touch by giving it one vertex of the middle hole.
    let mut touching = chains.clone();
    let middle = chains[3].clone();
    touching.push(vec![middle[0], middle[1]]);

    let closed: Vec<bool> = (0..touching.len())
        .map(|index| index < chains.len())
        .collect();

    let settled = BevelSurfacePartBuilder::the_settled_pick(
        &positions,
        &indices,
        &touching,
        &closed,
        Some(fixture::feature_edges(&positions, &indices)),
        BandProfile::Chamfer,
        fixture::SIZE,
        None,
        0.0,
        f64::INFINITY,
    )
    .expect("the pick is well formed");

    assert!(
        settled.is_none(),
        "a pick where two chains touch must be handed back to the path it took yesterday"
    );
}

/// The verdict is used only where it falls on WHOLE loops.
#[test]
fn a_verdict_that_names_half_a_loop_is_not_used() {
    let set = |edges: &[i32]| edges.iter().copied().collect::<HashSet<i32>>();
    let loops = [set(&[1, 2, 3]), set(&[4, 5])];

    assert!(
        BevelSurfacePartBuilder::the_loops_the_settling_condemns(&loops, &[set(&[1, 2])]).is_none(),
        "two of a three edge loop is half a loop, and half a loop cannot be dropped or named"
    );

    let condemned_loops =
        BevelSurfacePartBuilder::the_loops_the_settling_condemns(&loops, &[set(&[4, 5])]);
    assert!(
        condemned_loops.is_some(),
        "a chain covering exactly one loop is a verdict the pass can act on"
    );
    assert_eq!(condemned_loops.unwrap(), vec![1]);

    assert!(
        BevelSurfacePartBuilder::the_loops_the_settling_condemns(
            &loops,
            &[set(&[1, 2, 3]), set(&[4, 5])]
        )
        .is_none(),
        "condemning every loop is an answer about the whole pick, which the old path says better"
    );
}

/// The number offered to the user is strictly under the reading it came from.
#[test]
fn the_offered_size_is_under_the_reading() {
    for room in [1.0, 0.4523, 12.75, 0.0031] {
        let offered = BevelSurfacePartBuilder::the_size_to_offer(room);

        assert!(offered.is_some());
        let offered = offered.unwrap();
        assert!(offered < room, "a room of {room} was offered as {offered}");
        assert!(
            offered > room * 0.97,
            "the number offered must still be worth having"
        );
    }

    assert!(BevelSurfacePartBuilder::the_size_to_offer(0.0).is_none());
}
