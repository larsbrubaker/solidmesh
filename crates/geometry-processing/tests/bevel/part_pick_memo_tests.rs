//! What one bevel operation may re-use between the steps of a drag, and what it may not - read at
//! the memo itself (`part::pick_memo`) rather than through a rebuild, since the rules asserted
//! are about WHEN an answer still holds at another size: the content stamp, rooms and their
//! floors, refused sizes, settling re-use, and that the memo crosses threads.

use std::sync::Arc;

use geometry_processing::bevel::level_line::{
    ChainCannotFit, RoomReading, RoomReadingKind, RoomSource,
};
use geometry_processing::bevel::part::pick_memo::zero_room_source;
use geometry_processing::bevel::part::{BevelPickMemo, SettledPick};
use polygon_mesh::solids::platonic_solids::create_cube;
use polygon_mesh::{Face, Mesh};
use vector_math::Vector3Float;
const STAMP: u64 = 7;

const KEY: &str = "stamp|Chamfer|1e-9|closed:1,2,3";

/// Remembers the room, with who set it left at the default.
fn remember_the_room(memo: &BevelPickMemo, stamp: u64, key: &str, room: Option<f64>) {
    memo.remember_the_room(stamp, key, room, zero_room_source());
}

/// Remembers a smaller room, every optional parameter at its default.
fn remember_a_smaller_room(memo: &BevelPickMemo, stamp: u64, key: &str, room: f64) {
    memo.remember_a_smaller_room(stamp, key, room, zero_room_source(), -1, None);
}

fn remember_a_smaller_room_refused_at(memo: &BevelPickMemo, key: &str, room: f64, size: f64) {
    memo.remember_a_smaller_room(STAMP, key, room, zero_room_source(), -1, Some(size));
}

/// The room where the memo knows the pick, `None` where it does not.
fn try_the_room(memo: &BevelPickMemo, key: &str, size: Option<f64>) -> Option<Option<f64>> {
    memo.try_the_room(key, size)
}

/// A mesh of four thousand vertices, which is where the sampled hash's stride exceeds one.
fn a_big_mesh() -> Mesh {
    let mut mesh = Mesh::new();

    for i in 0..4000 {
        mesh.vertices
            .push(Vector3Float::new(i as f64, (i as f32 * 0.5f32) as f64, 0.0));
    }

    let mut i = 0;
    while i + 2 < 300 {
        let face = Face::new(i, i + 1, i + 2, &mesh.vertices);
        mesh.faces.push(face);
        i += 3;
    }

    mesh
}

fn a_settling(condemned: i32) -> Arc<SettledPick> {
    let named = (0..condemned)
        .map(|i| ChainCannotFit {
            chain: i,
            because: zero_room_source(),
            room: 0.5,
            round: 1,
        })
        .collect();

    Arc::new(SettledPick {
        chains: vec![0],
        condemned: named,
        unresolved: Vec::new(),
        last: Some(RoomReading {
            read: true,
            room: 2.0,
            ..RoomReading::default()
        }),
        fits: true,
        readings: 1,
    })
}

/// The room a pick read is handed back to the next rebuild of the same operation.
#[test]
fn a_room_is_read_once_and_handed_back() {
    let memo = BevelPickMemo::new();

    assert!(
        try_the_room(&memo, KEY, None).is_none(),
        "a fresh operation knows nothing about any pick"
    );

    remember_the_room(&memo, STAMP, KEY, Some(4.0));

    let room = try_the_room(&memo, KEY, None);
    assert!(room.is_some());
    assert_eq!(room.unwrap(), Some(4.0));

    assert_eq!(
        memo.rooms_read(),
        1,
        "one reading was paid for, so the drag's count of readings is one"
    );
}

/// A reading that was NOT about size is remembered too, which keeps a drag from re-reading the
/// whole pick at every step.
#[test]
fn a_no_size_statement_is_remembered_as_well() {
    let memo = BevelPickMemo::new();

    remember_the_room(&memo, STAMP, KEY, None);

    let room = try_the_room(&memo, KEY, None);
    assert!(
        room.is_some(),
        "a pick whose reading made no size statement was re-read at the next size"
    );
    assert!(room.unwrap().is_none());
}

/// A build that refused BY SIZE found a neck the reading could not see, and the smaller room it
/// names is what the next rebuild is handed - never a larger one, because the entry is a ceiling.
#[test]
fn a_pinch_refusal_lowers_what_the_next_rebuild_is_handed() {
    let memo = BevelPickMemo::new();

    remember_the_room(&memo, STAMP, KEY, Some(4.0));
    remember_a_smaller_room(&memo, STAMP, KEY, 2.5);

    assert_eq!(
        try_the_room(&memo, KEY, None).flatten(),
        Some(2.5),
        "the refusal proved a smaller ceiling than the reading had"
    );

    remember_a_smaller_room(&memo, STAMP, KEY, 3.5);

    assert_eq!(
        try_the_room(&memo, KEY, None).flatten(),
        Some(2.5),
        "a room may only ever be taken DOWN"
    );
}

/// A pinch's number is a size known to fit, not the most that does, so it never takes the room
/// under a size that has BUILT - whichever of the two is remembered first.
#[test]
fn a_smaller_room_never_sits_under_a_size_that_built() {
    let memo = BevelPickMemo::new();
    remember_the_room(&memo, STAMP, KEY, Some(0.5));

    memo.remember_a_size_that_built(STAMP, KEY, 0.494);
    remember_a_smaller_room(&memo, STAMP, KEY, 0.469);
    assert_eq!(try_the_room(&memo, KEY, None).flatten(), Some(0.494));

    let other = BevelPickMemo::new();
    remember_the_room(&other, STAMP, KEY, Some(0.5));
    remember_a_smaller_room(&other, STAMP, KEY, 0.469);
    other.remember_a_size_that_built(STAMP, KEY, 0.494);
    assert_eq!(try_the_room(&other, KEY, None).flatten(), Some(0.494));
}

/// A pick nothing was ever read for is not INVENTED by a refusal that is not about it.
#[test]
fn a_smaller_room_is_not_written_for_a_pick_that_was_never_read() {
    let memo = BevelPickMemo::new();

    remember_a_smaller_room(&memo, STAMP, KEY, 2.5);

    assert!(try_the_room(&memo, KEY, None).is_none());
}

/// The same pick asked at the SAME size is the same question, and its settling is re-used whole.
#[test]
fn a_settling_is_reused_at_the_same_size() {
    let memo = BevelPickMemo::new();
    let settled = a_settling(2);

    memo.remember_the_settling(STAMP, KEY, 1.0, settled.clone(), Some(0.5));

    let reused = memo.the_settling_to_reuse(KEY, 1.0).expect("re-used");
    assert!(Arc::ptr_eq(&reused, &settled));
}

/// A settling that condemned NOBODY at a size holds at every size at or below its room.
#[test]
fn a_settling_that_condemned_nobody_holds_down_to_its_room() {
    let memo = BevelPickMemo::new();
    let settled = a_settling(0);

    memo.remember_the_settling(STAMP, KEY, 2.0, settled.clone(), Some(2.0));

    assert!(
        memo.the_settling_to_reuse(KEY, 1.0)
            .is_some_and(|reused| Arc::ptr_eq(&reused, &settled)),
        "nobody can be condemned at a size the whole pick already reads above"
    );

    assert!(
        memo.the_settling_to_reuse(KEY, 2.0)
            .is_some_and(|reused| Arc::ptr_eq(&reused, &settled)),
        "at exactly the room no candidate is below the size"
    );

    assert!(
        memo.the_settling_to_reuse(KEY, 2.5).is_none(),
        "above the room a chain may now be condemned, so the pick is settled again"
    );
}

/// A settling that DID condemn somebody says nothing about another size, so it is settled again.
#[test]
fn a_settling_that_condemned_somebody_is_not_reused_at_another_size() {
    let memo = BevelPickMemo::new();

    memo.remember_the_settling(STAMP, KEY, 2.0, a_settling(1), Some(2.0));

    assert!(memo.the_settling_to_reuse(KEY, 1.0).is_none());
}

/// Two operations never share a thing.
#[test]
fn two_operations_never_share_a_reading() {
    let one = BevelPickMemo::new();
    let other = BevelPickMemo::new();

    remember_the_room(&one, STAMP, KEY, Some(4.0));
    one.remember_the_settling(STAMP, KEY, 1.0, a_settling(0), Some(4.0));

    assert!(try_the_room(&other, KEY, None).is_none());
    assert!(other.the_settling_to_reuse(KEY, 1.0).is_none());
    assert_eq!(other.rooms_read(), 0);
}

/// A settling carried onto the survivors' key costs nothing, because nothing was read for it.
#[test]
fn a_settling_carried_to_the_survivors_is_not_counted_as_work() {
    let memo = BevelPickMemo::new();

    memo.carry_the_settling_to(STAMP, "survivors", 1.0, a_settling(0), Some(4.0));

    assert_eq!(memo.settlings_run(), 0);
    assert!(memo.the_settling_to_reuse("survivors", 1.0).is_some());
}

/// The stamp sees an edit the sampled hash cannot: the defect that made this memo hand a part its
/// PRE-EDIT room for the rest of the operation's life.
#[test]
fn the_stamp_sees_an_edit_the_sampled_hash_cannot() {
    let before = a_big_mesh();
    let mut after = a_big_mesh();

    // Index 1 is stepped over by a stride of four (4000 vertices), and no face index moves, so
    // nothing the sampled hash looks at has changed.
    after.vertices[1] = Vector3Float::new(99.0, 99.0, 99.0);

    assert_eq!(
        after.get_long_hash_code(),
        before.get_long_hash_code(),
        "the sampled hash was expected to MISS this edit - without that this test witnesses nothing"
    );

    assert_ne!(
        BevelPickMemo::the_stamp_of(&after),
        BevelPickMemo::the_stamp_of(&before),
        "the memo would hand this part the room read before the edit for the rest of its life"
    );
}

/// A pinch refusal lowers what a SETTLED pick is re-used at too.
#[test]
fn a_pinch_refusal_lowers_what_a_settling_is_reused_at() {
    let memo = BevelPickMemo::new();

    memo.remember_the_settling(STAMP, KEY, 2.0, a_settling(0), Some(2.0));

    assert!(
        memo.the_settling_to_reuse(KEY, 1.5).is_some(),
        "below its room the settling holds"
    );

    remember_a_smaller_room(&memo, STAMP, KEY, 1.0);

    assert!(
        memo.the_settling_to_reuse(KEY, 1.5).is_none(),
        "a build refused at 1.0, so 1.5 is above the room whatever the reading said"
    );

    assert!(
        memo.the_settling_to_reuse(KEY, 0.5).is_some(),
        "under the floor the settling still holds"
    );
}

/// The floor is read before the size the settling was asked at, not after it.
#[test]
fn a_settling_is_not_reused_at_its_own_size_once_a_floor_is_below_it() {
    let memo = BevelPickMemo::new();

    memo.remember_the_settling(STAMP, KEY, 2.0, a_settling(0), Some(2.0));
    remember_a_smaller_room(&memo, STAMP, KEY, 1.0);

    assert!(
        memo.the_settling_to_reuse(KEY, 2.0).is_none(),
        "a build refused this pick at 1.0, so the settling saying everybody fits at 2.0 is stale"
    );
}

/// A later reading may not raise a room a build has already proved smaller.
#[test]
fn a_reading_cannot_raise_a_pinch_lowered_room() {
    let memo = BevelPickMemo::new();

    remember_a_smaller_room(&memo, STAMP, KEY, 1.0);
    remember_the_room(&memo, STAMP, KEY, Some(4.0));

    assert_eq!(
        try_the_room(&memo, KEY, None).flatten(),
        Some(1.0),
        "the build's refusal is the truth and the reading is only the estimate"
    );

    assert_eq!(
        memo.the_room_under_its_floor(KEY, Some(4.0), None),
        Some(1.0)
    );
}

/// The floor remembers the chain its pinch NAMED, not the chain of the room's source.
#[test]
fn a_floor_remembers_only_the_chain_its_pinch_named() {
    let handed_in = RoomSource::new(RoomReadingKind::SelfRoom, 2, -1);

    let named_nobody = BevelPickMemo::new();
    named_nobody.remember_a_smaller_room(STAMP, KEY, 1.0, handed_in, -1, None);

    assert_eq!(
        named_nobody.the_pinched_chain_under_the_floor(KEY, Some(1.5), None),
        -1,
        "a warm refusal from this floor must not drop chain 2, which the pinch never blamed"
    );

    let named_one = BevelPickMemo::new();
    named_one.remember_a_smaller_room(STAMP, KEY, 1.0, handed_in, 1, None);

    assert_eq!(
        named_one.the_pinched_chain_under_the_floor(KEY, Some(1.5), None),
        1,
        "the pinch named chain 1, whatever the room's source says"
    );
}

/// A size that BUILT lifts the floor and clears its pinch mark.
#[test]
fn a_size_that_built_clears_the_pinch_under_the_floor() {
    let memo = BevelPickMemo::new();

    memo.remember_a_smaller_room(
        STAMP,
        KEY,
        1.0,
        RoomSource::new(RoomReadingKind::SelfRoom, 1, -1),
        1,
        None,
    );
    memo.remember_a_size_that_built(STAMP, KEY, 1.2);

    assert_eq!(
        memo.the_room_under_its_floor(KEY, Some(4.0), None),
        Some(1.2)
    );

    assert_eq!(
        memo.the_pinched_chain_under_the_floor(KEY, Some(1.5), None),
        -1,
        "the pick built whole at 1.2, so no chain is to blame for the floor there"
    );
}

/// A floor holds the room down only at or above the size its build was refused at.
#[test]
fn a_floor_holds_only_from_the_size_it_was_refused_at() {
    let memo = BevelPickMemo::new();
    let read = RoomSource::new(RoomReadingKind::Foreign, 3, -1);
    let pinched = RoomSource::new(RoomReadingKind::SelfRoom, 1, -1);

    memo.remember_the_room(STAMP, KEY, Some(4.0), read);
    memo.remember_the_settling(STAMP, KEY, 2.5, a_settling(0), Some(4.0));
    memo.remember_a_smaller_room(STAMP, KEY, 1.0, pinched, 1, Some(2.0));

    assert_eq!(
        try_the_room(&memo, KEY, Some(1.5)).flatten(),
        Some(4.0),
        "1.5 is under the refused 2.0, so the reading is handed as a fresh build is handed it"
    );
    assert_eq!(
        memo.the_room_source(KEY, Some(1.5)).kind,
        RoomReadingKind::Foreign
    );
    assert!(
        memo.the_settling_to_reuse(KEY, 1.5).is_some(),
        "the settling said everybody fits up to 4.0, and nothing refused at 1.5"
    );

    assert_eq!(
        try_the_room(&memo, KEY, Some(2.0)).flatten(),
        Some(1.0),
        "at the size that was refused the floor holds"
    );
    assert_eq!(
        memo.the_room_source(KEY, Some(2.0)).kind,
        RoomReadingKind::SelfRoom
    );
    assert!(memo.the_settling_to_reuse(KEY, 2.0).is_none());

    assert_eq!(
        try_the_room(&memo, KEY, Some(3.0)).flatten(),
        Some(1.0),
        "and above it"
    );
    assert!(
        memo.the_settling_to_reuse(KEY, 2.5).is_none(),
        "not even at the settling's own size, which a build has since refused under"
    );
}

/// A refusal at a larger size never raises the size a floor holds from; a smaller one lowers it.
#[test]
fn the_size_a_floor_holds_from_only_moves_down() {
    let memo = BevelPickMemo::new();

    remember_the_room(&memo, STAMP, KEY, Some(4.0));
    remember_a_smaller_room_refused_at(&memo, KEY, 1.0, 2.0);
    remember_a_smaller_room_refused_at(&memo, KEY, 1.0, 3.0);

    assert_eq!(
        try_the_room(&memo, KEY, Some(2.5)).flatten(),
        Some(1.0),
        "2.0 was refused, and a later refusal at 3.0 does not undo that"
    );

    remember_a_smaller_room_refused_at(&memo, KEY, 1.0, 1.5);
    assert_eq!(
        try_the_room(&memo, KEY, Some(1.7)).flatten(),
        Some(1.0),
        "a refusal at 1.5 holds the floor from there"
    );
}

/// A floor whose refusal size is not known holds at every size, as a floor always did.
#[test]
fn a_floor_with_no_refused_size_holds_at_every_size() {
    let memo = BevelPickMemo::new();

    remember_the_room(&memo, STAMP, KEY, Some(4.0));
    remember_a_smaller_room(&memo, STAMP, KEY, 1.0);

    assert_eq!(try_the_room(&memo, KEY, Some(1.01)).flatten(), Some(1.0));
    assert_eq!(
        memo.the_room_under_its_floor(KEY, Some(4.0), Some(1.01)),
        Some(1.0)
    );
}

/// Forgetting the memo forgets the size a floor held from, so a later floor starts clean.
#[test]
fn forgetting_clears_the_size_a_floor_held_from() {
    let memo = BevelPickMemo::new();

    remember_a_smaller_room_refused_at(&memo, KEY, 1.0, 2.0);
    memo.forget();

    remember_the_room(&memo, STAMP, KEY, Some(4.0));
    remember_a_smaller_room_refused_at(&memo, KEY, 1.0, 3.0);

    assert_eq!(
        try_the_room(&memo, KEY, Some(2.5)).flatten(),
        Some(4.0),
        "the refusal at 2.0 was forgotten, so only 3.0 and above are held under the floor"
    );
}

/// Below the size a floor's pinch refused at, a refusal is the build's own: it does not blame the
/// chain the old pinch named.
#[test]
fn a_pinched_chain_is_blamed_only_where_its_floor_holds() {
    let memo = BevelPickMemo::new();

    memo.remember_a_smaller_room(
        STAMP,
        KEY,
        1.0,
        RoomSource::new(RoomReadingKind::SelfRoom, 1, -1),
        1,
        Some(2.0),
    );

    assert_eq!(
        memo.the_pinched_chain_under_the_floor(KEY, Some(4.0), Some(1.5)),
        -1,
        "at 1.5, under the refused 2.0, the floor is not what this refusal is about"
    );

    assert_eq!(
        memo.the_pinched_chain_under_the_floor(KEY, Some(1.0), Some(2.5)),
        1,
        "at 2.5 the refusal comes from the floor, so it names the chain the pinch named"
    );
}

/// A size that built UNDER the size a pinch refused at leaves the pinch's chain named.
#[test]
fn a_size_that_built_under_the_pinch_keeps_its_chain() {
    let memo = BevelPickMemo::new();

    memo.remember_a_smaller_room(
        STAMP,
        KEY,
        1.0,
        RoomSource::new(RoomReadingKind::SelfRoom, 1, -1),
        1,
        Some(2.0),
    );
    memo.remember_a_size_that_built(STAMP, KEY, 1.8);

    assert_eq!(
        memo.the_pinched_chain_under_the_floor(KEY, Some(1.8), Some(2.5)),
        1,
        "the pick built at 1.8, but 2.0 pinched on chain 1 and 2.5 is above it"
    );

    memo.remember_a_size_that_built(STAMP, KEY, 2.2);

    assert_eq!(
        memo.the_pinched_chain_under_the_floor(KEY, Some(2.2), Some(2.5)),
        -1,
        "once the pick built whole above the pinched size, no chain is to blame"
    );
}

/// A new mesh empties what was read on the old one, so a drag whose mesh changes at every step
/// cannot grow the memo without bound.
#[test]
fn a_new_mesh_evicts_what_the_old_one_read() {
    let memo = BevelPickMemo::new();

    for step in 0u64..50 {
        let key = format!("{step}|{KEY}");
        remember_the_room(&memo, step, &key, Some(4.0));
        remember_a_smaller_room(&memo, step, &key, 2.0);
        memo.remember_the_settling(step, &key, 1.0, a_settling(0), Some(4.0));
    }

    assert!(
        memo.entries() <= 3,
        "fifty steps of a drag left {} entries, each carrying a pick's whole vertex list",
        memo.entries()
    );
}

fn settled(fits: bool, condemned: Vec<ChainCannotFit>) -> Arc<SettledPick> {
    Arc::new(SettledPick {
        chains: vec![0],
        condemned,
        unresolved: Vec::new(),
        last: None,
        fits,
        readings: 1,
    })
}

#[test]
fn rust_only_the_stamp_reads_every_vertex_and_every_face() {
    let mesh = create_cube(10.0, 10.0, 10.0);
    let stamp = BevelPickMemo::the_stamp_of(&mesh);
    assert_eq!(stamp, BevelPickMemo::the_stamp_of(&mesh.clone()));

    let mut moved = mesh.clone();
    moved.vertices[3].x += 0.001;
    assert_ne!(stamp, BevelPickMemo::the_stamp_of(&moved));

    assert_ne!(stamp, BevelPickMemo::the_stamp_of(&Mesh::new()));
}

#[test]
fn rust_only_a_floor_holds_the_room_down_only_at_and_above_the_refused_size() {
    let memo = BevelPickMemo::new();
    memo.remember_the_room(7, "pick", Some(2.0), RoomSource::foreign(3));
    assert_eq!(memo.rooms_read(), 1);
    assert_eq!(memo.try_the_room("pick", None), Some(Some(2.0)));
    assert_eq!(memo.try_the_room("other", None), None);

    // A pinch at 1.5 named 1.2 and chain 4.
    memo.remember_a_smaller_room(7, "pick", 1.2, RoomSource::self_room(4), 4, Some(1.5));
    assert_eq!(memo.try_the_room("pick", Some(1.5)), Some(Some(1.2)));
    assert_eq!(memo.try_the_room("pick", None), Some(Some(1.2)));
    // Below the refused size the floor says nothing and the reading itself is handed over.
    assert_eq!(memo.try_the_room("pick", Some(1.4)), Some(Some(2.0)));
    assert_eq!(
        memo.the_room_source("pick", Some(1.5)),
        RoomSource::self_room(4)
    );
    assert_eq!(
        memo.the_room_source("pick", Some(1.4)),
        RoomSource::foreign(3)
    );
    assert_eq!(memo.the_readings_own_source("pick"), RoomSource::foreign(3));
    assert_eq!(
        memo.the_pinched_chain_under_the_floor("pick", None, Some(1.5)),
        4
    );
    assert_eq!(
        memo.the_pinched_chain_under_the_floor("pick", None, Some(1.4)),
        -1
    );

    // A larger room never raises the floor.
    memo.remember_a_smaller_room(7, "pick", 1.3, RoomSource::nothing(), -1, None);
    assert_eq!(
        memo.the_room_under_its_floor("pick", Some(9.0), None),
        Some(1.2)
    );

    // A size that built lifts the floor to it and, at or above the refused size, clears the chain.
    memo.remember_a_size_that_built(7, "pick", 1.6);
    assert_eq!(
        memo.the_room_under_its_floor("pick", Some(9.0), None),
        Some(1.6)
    );
    assert_eq!(
        memo.the_pinched_chain_under_the_floor("pick", None, None),
        -1
    );

    // A write on another mesh empties the memo; the counters stay.
    memo.remember_the_room(8, "else", None, RoomSource::nothing());
    assert_eq!(memo.try_the_room("pick", None), None);
    assert_eq!(memo.try_the_room("else", None), Some(None));
    assert_eq!(memo.rooms_read(), 2);
    assert_eq!(memo.entries(), 1);
    assert_eq!(memo.the_room_source("missing", None), zero_room_source());

    memo.forget();
    assert_eq!(memo.rooms_read(), 0);
    assert_eq!(memo.entries(), 0);
}

#[test]
fn rust_only_a_settling_is_reused_only_where_it_still_answers_the_question() {
    let memo = BevelPickMemo::new();
    let fitted = settled(true, Vec::new());
    memo.remember_the_settling(1, "rim", 1.0, fitted.clone(), Some(2.0));
    assert_eq!(memo.settlings_run(), 1);

    // The same size, and any size the room still covers when nobody was condemned.
    assert!(memo.the_settling_to_reuse("rim", 1.0).is_some());
    assert!(memo.the_settling_to_reuse("rim", 2.0).is_some());
    assert!(memo.the_settling_to_reuse("rim", 2.5).is_none());

    // A floor proved under the size makes a "fits" settling stale, the same size included.
    memo.remember_a_smaller_room(1, "rim", 0.8, RoomSource::nothing(), -1, Some(0.9));
    assert!(memo.the_settling_to_reuse("rim", 1.0).is_none());
    assert!(memo.the_settling_to_reuse("rim", 0.85).is_some());

    // A settling that condemned somebody answers its own size only.
    let condemned = settled(
        false,
        vec![ChainCannotFit::new(2, RoomSource::foreign(2), 0.4)],
    );
    memo.carry_the_settling_to(1, "survivors", 0.5, condemned, Some(3.0));
    assert_eq!(memo.settlings_run(), 1);
    assert!(memo.the_settling_to_reuse("survivors", 0.5).is_some());
    assert!(memo.the_settling_to_reuse("survivors", 0.6).is_none());
}

#[test]
fn rust_only_the_pick_memo_crosses_threads() {
    fn shared<T: Send + Sync>() {}
    shared::<BevelPickMemo>();
}
