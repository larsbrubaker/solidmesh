//! How a pick memo crosses to another process and back (`part::pick_memo_crossing`): a snapshot
//! taken and restored answers every query the original does, a journaling memo's writes replayed
//! onto another memo leave it exactly where the original is, a memo restored and then journaled
//! journals only what it writes itself, and a real build warmed on one memo hands its room to a
//! build on the memo its journal was replayed onto. The memo's own rules are pinned in
//! `part_pick_memo_tests`.
//!
//! Snapshots are compared by their `Debug` text: a settling's reading carries NaN rooms, which no
//! derived equality reads as equal to themselves, and the text is exact to the bit for every
//! number the memo holds.

use std::sync::Arc;

use solidmesh::bevel::level_line::{ChainCannotFit, RoomCandidate, RoomReading, RoomSource};
use solidmesh::bevel::part::{
    bevel_part, BevelPartRequest, BevelPickMemo, MemoSnapshot, MemoWrite, SettledPick,
};
use solidmesh::topology::bevel_edge_group_resolver::create_groups_for_all_edges;
use solidmesh_math::Vector3;
use solidmesh_mesh::solids::platonic_solids::create_cube;
use solidmesh_mesh::CancelToken;

fn text_of(snapshot: &MemoSnapshot) -> String {
    format!("{snapshot:?}")
}

/// A settling with everything a real one can hold: a condemned chain, an unresolved group and a
/// last reading with candidates, NaN rooms and a crease position among them.
fn a_full_settling(fits: bool) -> Arc<SettledPick> {
    Arc::new(SettledPick {
        chains: vec![0, 2, 3],
        condemned: vec![ChainCannotFit {
            chain: 1,
            because: RoomSource::chain_to_chain(1, 2),
            room: 0.25,
            round: 2,
        }],
        unresolved: vec![vec![4, 5]],
        last: Some(RoomReading {
            read: true,
            room: 1.75,
            room_set_by: RoomSource::growing_crease(1, Vector3::new(1.0, 2.0, 3.0)),
            candidates: vec![
                RoomCandidate::new(RoomSource::foreign(0), 1.75),
                RoomCandidate {
                    room_without_the_other: 2.5,
                    room_without_the_other_at_most: 3.0,
                    ..RoomCandidate::new(RoomSource::chain_to_chain(2, 3), 2.0)
                },
                RoomCandidate {
                    group_members: Some(vec![3]),
                    ..RoomCandidate::new(RoomSource::self_room(3), 4.0)
                },
            ],
            chain_lengths: vec![10.0, 20.0, 30.0],
            ..RoomReading::default()
        }),
        fits,
        readings: 3,
    })
}

/// Writes every map the memo holds, on stamp 9.
fn warm(memo: &BevelPickMemo) {
    memo.remember_the_room(9, "a", Some(4.0), RoomSource::foreign(3));
    memo.remember_the_room(9, "b", None, RoomSource::nothing());
    memo.remember_the_room(9, "c", Some(1.0), RoomSource::self_room(0));
    memo.remember_a_smaller_room(9, "a", 1.2, RoomSource::self_room(4), 4, Some(1.5));
    memo.remember_a_smaller_room(9, "c", 0.5, RoomSource::chain_to_chain(0, 1), -1, None);
    memo.remember_a_size_that_built(9, "a", 1.1);
    memo.remember_a_size_that_built(9, "d", 0.7);
    memo.remember_the_settling(9, "rim", 1.0, a_full_settling(true), Some(2.0));
    memo.carry_the_settling_to(9, "survivors", 0.5, a_full_settling(false), Some(3.0));
}

/// Every answer the memo gives for the keys `warm` writes, at sizes either side of each floor's
/// refused size.
fn every_answer(memo: &BevelPickMemo) -> String {
    let mut answers = String::new();

    for key in ["a", "b", "c", "d", "rim", "survivors", "missing"] {
        for size in [None, Some(0.4), Some(1.0), Some(1.4), Some(1.5), Some(2.5)] {
            answers += &format!(
                "{key} {size:?}: room {:?} under {:?} source {:?} own {:?} pinched {:?} \
                 under-source {:?}\n",
                memo.try_the_room(key, size),
                memo.the_room_under_its_floor(key, Some(9.0), size),
                memo.the_room_source(key, size),
                memo.the_readings_own_source(key),
                memo.the_pinched_chain_under_the_floor(key, None, size),
                memo.the_source_under_its_floor(key, Some(9.0), RoomSource::foreign(7), size),
            );
        }

        for size in [0.5, 1.0, 1.4, 2.0, 2.5] {
            answers += &format!(
                "{key} settling at {size}: {:?}\n",
                memo.the_settling_to_reuse(key, size)
            );
        }
    }

    answers + &format!("entries {}", memo.entries())
}

/// A memo warmed by every kind of write, snapshotted and restored, answers every query as the
/// original does, starts its work counters at zero and snapshots to the same thing again.
#[test]
fn a_restored_snapshot_answers_every_query_as_the_original() {
    let original = BevelPickMemo::new();
    warm(&original);

    let snapshot = original.snapshot();
    assert_eq!(snapshot.the_stamp, Some(9));
    assert_eq!(snapshot.rooms.len(), 3);
    assert_eq!(snapshot.settlings.len(), 2);

    let restored = BevelPickMemo::from_snapshot(snapshot.clone());

    assert_eq!(every_answer(&restored), every_answer(&original));
    assert_eq!(text_of(&restored.snapshot()), text_of(&snapshot));

    assert_eq!(
        (restored.rooms_read(), restored.settlings_run()),
        (0, 0),
        "a restored memo has paid for nothing yet, so its counts are its own from zero"
    );

    // The stamp crossed too: a write on the same mesh keeps what was restored, another empties it.
    restored.remember_the_room(9, "e", Some(1.0), RoomSource::nothing());
    assert_eq!(restored.try_the_room("a", None), Some(Some(1.2)));
    restored.remember_the_room(10, "e", Some(1.0), RoomSource::nothing());
    assert_eq!(restored.try_the_room("a", None), None);
}

/// A snapshot lists each map by key, so two memos holding the same entries snapshot alike
/// whatever order they were written in.
#[test]
fn a_snapshot_does_not_depend_on_the_order_of_the_writes() {
    let forwards = BevelPickMemo::new();
    let backwards = BevelPickMemo::new();
    let keys: Vec<String> = (0..40).map(|i| format!("pick {i}")).collect();

    for key in &keys {
        forwards.remember_the_room(3, key, Some(2.0), RoomSource::foreign(1));
    }
    for key in keys.iter().rev() {
        backwards.remember_the_room(3, key, Some(2.0), RoomSource::foreign(1));
    }

    let snapshot = forwards.snapshot();
    assert_eq!(text_of(&snapshot), text_of(&backwards.snapshot()));
    assert!(snapshot.rooms.windows(2).all(|pair| pair[0].0 < pair[1].0));
}

/// A journaling memo driven through every kind of write - a stamp change that empties it and a
/// forget among them - replays onto a fresh memo, which then holds what the original holds and
/// has counted the same work.
#[test]
fn a_journal_replayed_onto_a_fresh_memo_leaves_it_where_the_original_is() {
    let original = BevelPickMemo::new().journaling();

    warm(&original);
    original.forget();
    warm(&original);
    original.remember_a_smaller_room(9, "rim", 0.8, RoomSource::nothing(), 2, Some(0.9));
    original.remember_a_size_that_built(9, "rim", 0.85);
    // A refused room of nothing is never remembered, so neither is it journaled.
    original.remember_a_smaller_room(9, "rim", 0.0, RoomSource::nothing(), -1, None);

    // A new mesh: everything above is emptied, and only what follows is held.
    original.remember_the_room(11, "a", Some(3.0), RoomSource::foreign(0));
    original.remember_a_smaller_room(11, "a", 2.0, RoomSource::self_room(1), 1, Some(2.2));
    original.remember_the_settling(11, "rim", 1.0, a_full_settling(true), Some(2.0));
    original.carry_the_settling_to(11, "survivors", 0.5, a_full_settling(false), None);
    original.remember_a_size_that_built(11, "a", 1.9);

    let journal = original.take_journal();
    assert_eq!(journal.len(), 2 * 9 + 1 + 2 + 5);
    assert!(matches!(journal[9], MemoWrite::Forget));
    assert!(
        original.take_journal().is_empty(),
        "taking the journal drains it"
    );

    let replayed = BevelPickMemo::new();
    for write in &journal {
        replayed.apply(write);
    }

    assert_eq!(text_of(&replayed.snapshot()), text_of(&original.snapshot()));
    assert_eq!(every_answer(&replayed), every_answer(&original));
    assert_eq!(
        (replayed.rooms_read(), replayed.settlings_run()),
        (original.rooms_read(), original.settlings_run()),
        "a replayed write counts as the write itself would have"
    );
}

/// A worker's memo starts from the page's snapshot and journals only what it writes itself, so
/// the page is never handed back what it already holds; a memo that does not journal has nothing
/// to hand back.
#[test]
fn a_restored_memo_journals_only_its_own_writes() {
    let page = BevelPickMemo::new();
    warm(&page);

    let worker = BevelPickMemo::from_snapshot(page.snapshot()).journaling();
    assert!(worker.take_journal().is_empty());

    worker.remember_the_room(9, "new", Some(5.0), RoomSource::foreign(2));

    let journal = worker.take_journal();
    assert_eq!(journal.len(), 1);
    match &journal[0] {
        MemoWrite::RememberTheRoom {
            stamp,
            key,
            room,
            set_by,
        } => {
            assert_eq!(
                (*stamp, key.as_str(), *room, *set_by),
                (9, "new", Some(5.0), RoomSource::foreign(2))
            );
        }
        other => panic!("journaled {other:?}"),
    }

    page.apply(&journal[0]);
    assert_eq!(text_of(&page.snapshot()), text_of(&worker.snapshot()));

    page.remember_the_room(9, "page only", None, RoomSource::nothing());
    assert!(page.take_journal().is_empty());
}

/// End to end, as MatterCAD's worker uses it: a real build at one size on a journaling memo, its
/// journal replayed onto the page's memo, and a build there at another size reads no room and
/// builds what a build on the worker's own memo builds.
#[test]
fn a_replayed_journal_spares_the_next_build_its_room_reading() {
    let cube = Arc::new(create_cube(10.0, 10.0, 10.0));
    let groups = create_groups_for_all_edges(&cube, 20.0, None).expect("a cube has edges");
    let request = |size: f64, memo: &Arc<BevelPickMemo>| BevelPartRequest {
        pick_memo: Some(memo.clone()),
        ..BevelPartRequest::new(
            Some(cube.clone()),
            vec![groups[0].clone()],
            size,
            4,
            true,
            20.0,
        )
    };
    let build = |size: f64, memo: &Arc<BevelPickMemo>| {
        let outcome =
            bevel_part(request(size, memo), &CancelToken::new(), None).expect("the build runs");
        let mesh = outcome.mesh.expect("a filleted edge is a mesh");
        format!("{:?} {:?}", mesh.vertices, mesh.faces)
    };

    let worker = Arc::new(BevelPickMemo::new().journaling());
    build(1.0, &worker);
    let read_by_the_worker = worker.rooms_read();
    assert!(
        read_by_the_worker > 0,
        "the cold build read the pick's room"
    );

    let page = Arc::new(BevelPickMemo::new());
    for write in worker.take_journal() {
        page.apply(&write);
    }
    assert_eq!(page.rooms_read(), read_by_the_worker);
    assert_eq!(text_of(&page.snapshot()), text_of(&worker.snapshot()));

    let on_the_page = build(0.8, &page);
    assert_eq!(
        page.rooms_read(),
        read_by_the_worker,
        "the page's build re-read a room the worker had already read"
    );
    assert_eq!(on_the_page, build(0.8, &worker));
    assert_eq!(worker.rooms_read(), read_by_the_worker);
}

#[test]
fn rust_only_what_crosses_is_plain_data_that_crosses_threads() {
    fn shared<T: Send + Sync + Clone + std::fmt::Debug + 'static>() {}
    shared::<MemoSnapshot>();
    shared::<MemoWrite>();
}
