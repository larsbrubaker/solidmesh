//! The requirement for naming every too-tight chain in ONE reading, on a real pick of six chains -
//! a plate whose top face carries five holes, two of them too near the plate's edge for the size.
//! One reading names both of those two, and what is left builds and fills closed in ONE build; the
//! read-drop-re-read loop ([`PickThatFits`]) settles on chains that really fit.

use std::collections::{HashMap, HashSet};
use std::rc::Rc;

use geometry_processing::bevel::level_line::band_gap_entry_points::{self, BandGapBuildOptions};
use geometry_processing::bevel::level_line::band_gap_reading::{read_room, read_room_of_pick};
use geometry_processing::bevel::level_line::gap_fill;
use geometry_processing::bevel::level_line::{
    BandGapRefusal, BandProfile, ChainsThatCannotFit, LevelLineMesh, PickThatFits, RoomReading,
    RoomReadingKind, WorkBudget,
};
use vector_math::Vector3;

use crate::five_hole_plate_fixture as fixture;
use crate::gap_fill_tests::shell_count;
use crate::level_line_feature_edges::feature_edges_over;
use crate::level_line_room_geometry::SolidWithChains;
use crate::level_line_test_geometry::{cube_solid, signed_volume_of};
use crate::several_chains_fixtures;

/// The same members, in any order.
fn same_members(a: &[i32], b: &[i32]) -> bool {
    let mut a = a.to_vec();
    let mut b = b.to_vec();
    a.sort();
    b.sort();
    a == b
}

fn budget(triangles: usize) -> Option<Rc<WorkBudget>> {
    Some(Rc::new(
        WorkBudget::new(WorkBudget::default_limit(triangles as i32)).expect("a positive limit"),
    ))
}

fn settle(pick: &SolidWithChains, size: f64) -> PickThatFits {
    let (positions, indices, chains) = pick;
    PickThatFits::read(
        positions,
        indices,
        chains,
        &vec![true; chains.len()],
        Some(fixture::feature_edges(positions, indices)),
        BandProfile::Chamfer,
        size,
        1e-9,
        None,
        0.0,
        f64::INFINITY,
    )
    .expect("the pick reads")
}

/// One build and one fill of `survivors`, checked closed, consistently wound and one shell.
fn build_and_fill_closed(
    positions: &[Vector3],
    indices: &[(i32, i32, i32)],
    survivors: &[Vec<i32>],
    size: f64,
) -> Vec<i32> {
    let gap = band_gap_entry_points::build_pick(
        positions,
        indices,
        survivors,
        &vec![true; survivors.len()],
        Some(fixture::feature_edges(positions, indices)),
        size,
        BandProfile::Chamfer,
        &BandGapBuildOptions::default(),
    )
    .expect("the cut runs");
    assert_eq!(gap.refusal, BandGapRefusal::None);

    let fill = gap_fill::fill(&gap, BandProfile::Chamfer, 6).expect("the fill runs");
    assert!(fill.filled());
    let mesh = fill.mesh.as_ref().expect("the filled mesh").borrow();
    assert!(
        is_closed_and_consistent(&mesh),
        "every directed edge once and its opposite once"
    );
    assert_eq!(shell_count(&mesh), 1);
    fill.triangles_of_chain.clone()
}

/// The fixture is a solid before anything is asked of it.
#[test]
fn the_fixture_is_a_closed_consistently_wound_solid() {
    let (positions, indices, chains) = fixture::pick();

    let mut directed: HashMap<(i32, i32), i32> = HashMap::new();
    for &(a, b, c) in &indices {
        for step in [(a, b), (b, c), (c, a)] {
            *directed.entry(step).or_insert(0) += 1;
        }
    }

    for (step, count) in &directed {
        assert_eq!(*count, 1, "{step:?} is traversed exactly once");
        assert!(
            directed.contains_key(&(step.1, step.0)),
            "{step:?} must stand on a second triangle, wound the other way"
        );
    }

    assert!(signed_volume_of(&positions, &indices) > 0.0);
    assert_eq!(chains.len(), 6);
}

/// ONE asked reading names exactly the two holes that stand too near the plate's edge - and
/// today's unasked reading cannot, because the prune skipped the second pair as unable to lower
/// the room. That is the whole point: the app drops both in one pass instead of one per rebuild.
#[test]
fn one_asked_reading_names_both_holes_where_todays_reading_names_fewer() {
    let (positions, indices, chains) = fixture::pick();
    let size = fixture::SIZE;

    let asked = fixture::read(&positions, &indices, &chains, size);
    let named = ChainsThatCannotFit::read(&asked, size).dropped;

    let named_chains: Vec<i32> = named.iter().map(|d| d.chain).collect();
    assert!(
        same_members(&named_chains, &fixture::TOO_NEAR_THE_EDGE),
        "both end holes are 0.6 from the plate's edge and neither can be built at 0.5: {named_chains:?}"
    );
    for one in &named {
        assert_eq!(one.because.kind, RoomReadingKind::ChainToChain);
        assert_eq!(
            one.because.other_chain, 0,
            "a hole's tight partner is the plate's own outer rim"
        );
        assert!(one.room < size);
    }

    // At THIS size the asked reading costs nothing: the plate's outer rim bounds every hole, so its
    // bounding sphere overlaps each of theirs and no outer-rim pair was ever prunable.
    let today = fixture::read(&positions, &indices, &chains, f64::NAN);
    assert_eq!(
        ChainsThatCannotFit::read(&today, size).dropped.len(),
        named.len()
    );
}

/// The prune's own requirement: at a size big enough that the holes are tight against EACH
/// OTHER, two separate pairs are tight but only one of them sets the room - and a pair that cannot
/// lower the room is skipped unread. The reading asked to name every candidate under the size
/// names them all.
#[test]
fn asking_for_a_size_names_the_tight_pairs_the_prune_would_have_skipped() {
    let (positions, indices, chains) = fixture::pick();

    // Over the 1.5 the 3-wide gaps between neighbouring holes buy, and well over the 0.3 the end
    // holes' own gap to the plate's edge buys.
    let size = 1.6;

    let today = fixture::read(&positions, &indices, &chains, f64::NAN);
    let asked = fixture::read(&positions, &indices, &chains, size);

    assert_eq!(
        asked.room, today.room,
        "asking for more names may not move the room"
    );
    assert_eq!(asked.room_set_by, today.room_set_by);
    assert!(
        asked.candidates.len() > today.candidates.len(),
        "the pairs the prune skipped are read now"
    );

    let named_today: Vec<i32> = ChainsThatCannotFit::read(&today, size)
        .dropped
        .iter()
        .map(|d| d.chain)
        .collect();
    let named_asked: Vec<i32> = ChainsThatCannotFit::read(&asked, size)
        .dropped
        .iter()
        .map(|d| d.chain)
        .collect();
    println!("at {size}: unasked names {named_today:?}; asked names {named_asked:?}");

    assert!(
        named_asked.len() > named_today.len(),
        "a tight pair with no row in the table cannot be named"
    );
    for chain in &named_today {
        assert!(
            named_asked.contains(chain),
            "the asked reading only ever names MORE, never something else"
        );
    }
}

/// What is left after the naming really does fit: a real reading of the survivors alone stands at
/// or above the size, and ONE build and ONE fill of them all together comes back closed,
/// consistently wound and one shell.
#[test]
fn the_survivors_build_and_fill_closed_in_one_build() {
    let (positions, indices, chains) = fixture::pick();
    let size = fixture::SIZE;
    let named =
        ChainsThatCannotFit::read(&fixture::read(&positions, &indices, &chains, size), size);

    let survivors: Vec<Vec<i32>> = chains
        .iter()
        .enumerate()
        .filter(|(k, _)| named.dropped.iter().all(|d| d.chain != *k as i32))
        .map(|(_, chain)| chain.clone())
        .collect();
    assert_eq!(survivors.len(), 4);

    let of_survivors = fixture::read(&positions, &indices, &survivors, f64::NAN);
    assert!(
        of_survivors.room >= size,
        "a pass that leaves something still too tight was wasted"
    );

    let triangles_of_chain = build_and_fill_closed(&positions, &indices, &survivors, size);
    assert!(
        triangles_of_chain.iter().all(|&c| c > 0),
        "every surviving chain laid its own strip"
    );
}

/// The answer is about the PART, not about how the pick was typed: the SAME two holes are named in
/// either order - by identity - and a rigid motion of the plate changes nothing at all.
#[test]
fn the_order_of_the_chains_and_a_rigid_motion_change_nothing() {
    let size = fixture::SIZE;

    let straight = named_at(&fixture::pick(), size);
    let holes_first = named_at(&fixture::pick_with(true, false), size);
    let moved = named_at(&fixture::pick_with(false, true), size);

    assert!(same_members(&straight, &fixture::TOO_NEAR_THE_EDGE));
    assert!(
        same_members(&moved, &fixture::TOO_NEAR_THE_EDGE),
        "a part that is merely moved names the same chains"
    );
    assert!(
        same_members(
            &holes_first,
            &fixture::TOO_NEAR_THE_EDGE_WHEN_HOLES_COME_FIRST
        ),
        "the holes-first order names the same two holes, at the indices it gave them"
    );

    // The same chains by IDENTITY: the vertex lists named are the same sets whichever order was
    // given.
    let rim_first = fixture::pick();
    let other = fixture::pick_with(true, false);
    let named_rim_first: Vec<Vec<i32>> = straight
        .iter()
        .map(|&k| sorted(&rim_first.2[k as usize]))
        .collect();
    let named_holes_first: Vec<Vec<i32>> = holes_first
        .iter()
        .map(|&k| sorted(&other.2[k as usize]))
        .collect();
    assert_eq!(named_holes_first.len(), named_rim_first.len());
    for chain in &named_rim_first {
        assert!(
            named_holes_first.contains(chain),
            "what a user sees dropped cannot depend on the order the loops were picked in"
        );
    }
}

/// Without a size to name candidates under, the reading is what it always was: the same room, the
/// same reading behind it and the same work. The asked reading may only ADD rows.
#[test]
fn the_unasked_reading_is_unchanged_and_the_asked_one_only_gains_rows() {
    for name in ["A", "B", "C"] {
        let (positions, indices, chains) = several_chains_fixtures::named(name);
        let feature =
            feature_edges_over(&positions, &indices, several_chains_fixtures::FOLD_DEGREES);
        let closed = vec![true; chains.len()];

        let today = read_room_of_pick(
            &positions,
            &indices,
            &chains,
            &closed,
            Some(feature.clone()),
            BandProfile::Chamfer,
            1e-9,
            budget(indices.len()),
            f64::NAN,
            0.0,
            f64::INFINITY,
        )
        .expect("the pick reads");

        // A size well above the room: every pair the prune would have skipped is now kept.
        let asked = read_room_of_pick(
            &positions,
            &indices,
            &chains,
            &closed,
            Some(feature),
            BandProfile::Chamfer,
            1e-9,
            budget(indices.len()),
            today.room * 10.0,
            0.0,
            f64::INFINITY,
        )
        .expect("the pick reads");

        assert_eq!(asked.room, today.room, "{name}");
        assert_eq!(asked.room_set_by, today.room_set_by, "{name}");
        assert!(
            asked.candidates.len() >= today.candidates.len(),
            "{name}: asking for more names may only add rows"
        );
    }
}

/// What the extra rows cost, recorded on the fixture the app's own dear picks look like: the asked
/// reading pays one bare pass over the other chain's edges per pair it would have skipped, ONCE
/// per pick - against one whole refuse-drop-rebuild pass per tight pair, which is what it buys.
#[test]
fn the_asked_reading_costs_what_its_extra_pairs_cost() {
    let (positions, indices, chains) = fixture::pick();
    let closed = vec![true; chains.len()];
    let feature = fixture::feature_edges(&positions, &indices);

    let today = read_room_of_pick(
        &positions,
        &indices,
        &chains,
        &closed,
        Some(feature.clone()),
        BandProfile::Chamfer,
        1e-9,
        budget(indices.len()),
        f64::NAN,
        0.0,
        f64::INFINITY,
    )
    .expect("the pick reads");

    // Asked at 1.6, where the holes are tight against each other and the prune really would have
    // skipped those pairs - at the fixture's own 0.5 nothing is skipped and the asked reading is
    // free.
    let asked = read_room_of_pick(
        &positions,
        &indices,
        &chains,
        &closed,
        Some(feature),
        BandProfile::Chamfer,
        1e-9,
        budget(indices.len()),
        1.6,
        0.0,
        f64::INFINITY,
    )
    .expect("the pick reads");

    println!(
        "five-hole plate ({} triangles): {} rows for {} units unasked, {} rows for {} units asked at 1.6",
        indices.len(),
        today.candidates.len(),
        today.work_spent,
        asked.candidates.len(),
        asked.work_spent
    );

    assert_eq!(asked.room, today.room);
    assert!(asked.candidates.len() > today.candidates.len());
}

/// ONE reading is not the answer, and the loop is: at a size where dropping a chain re-tags the
/// sides and makes a survivor's neighbours newly foreign, read-drop-re-read settles on chains that
/// REALLY fit - their own reading stands at or above the size and they build and fill closed in one
/// build. Measured: at 1.0 ONE reading leaves four chains that then read 0.600 and the loop settles
/// on three reading 1.500; at 1.6 one reading leaves three reading 0.600 and the loop settles on two
/// reading 3.000.
#[test]
fn the_read_drop_reread_loop_ends_with_survivors_that_really_fit() {
    for size in [1.0, 1.6] {
        let pick = fixture::pick();
        let (positions, indices, chains) = &pick;

        // ONE reading is not enough: what it leaves standing still does not fit.
        let one_reading =
            ChainsThatCannotFit::read(&fixture::read(positions, indices, chains, size), size);
        let after_one: Vec<Vec<i32>> = chains
            .iter()
            .enumerate()
            .filter(|(k, _)| one_reading.dropped.iter().all(|d| d.chain != *k as i32))
            .map(|(_, chain)| chain.clone())
            .collect();
        let of_after_one: Option<RoomReading> = (!after_one.is_empty())
            .then(|| fixture::read(positions, indices, &after_one, f64::NAN));

        let settled = settle(&pick, size);

        println!(
            "at {size}: one reading leaves {} reading {:?}; the loop settles in {} rounds on {}, room {:?}",
            after_one.len(),
            of_after_one.as_ref().map(|r| r.room),
            settled.rounds,
            settled.chains.len(),
            settled.last.as_ref().map(|r| r.room)
        );

        assert!(
            of_after_one.expect("a reading of what one reading left").room < size,
            "ONE reading is not the answer: dropping a chain re-tags the sides and makes what stood behind it newly foreign"
        );

        assert!(settled.rounds <= chains.len() as i32 + 1);
        assert!(settled.fits);
        assert_eq!(settled.unresolved.len(), 0);
        assert!(
            settled.last.as_ref().expect("a last reading").room >= size,
            "the loop only stops when a reading of what is left names nobody"
        );

        let survivors: Vec<Vec<i32>> = settled
            .chains
            .iter()
            .map(|&k| chains[k as usize].clone())
            .collect();
        build_and_fill_closed(positions, indices, &survivors, size);
    }
}

/// The loop's answer is about the part: the same chains survive by IDENTITY whichever order the
/// pick was given in.
#[test]
fn the_loop_settles_on_the_same_chains_in_either_order() {
    for size in [1.0, 1.6] {
        let one = settled_chains(&fixture::pick(), size);
        let other = settled_chains(&fixture::pick_with(true, false), size);

        assert_eq!(other.len(), one.len());
        for chain in &one {
            assert!(
                other.contains(chain),
                "what survives cannot depend on the order the loops were picked in"
            );
        }
    }
}

/// The self reading is relaxed under an asked size too, not only the pair prune: a group whose own
/// room stands between the pick's room and the size reads UNBOUNDED unasked - a row nobody can be
/// named by - and is measured when the size is asked for.
#[test]
fn the_self_reading_is_measured_under_the_asked_size_too() {
    for (name, size) in [("B", 3.0), ("B", 6.0)] {
        let (positions, indices, chains) = several_chains_fixtures::named(name);
        let feature =
            feature_edges_over(&positions, &indices, several_chains_fixtures::FOLD_DEGREES);
        let closed = vec![true; chains.len()];

        let today = read_room_of_pick(
            &positions,
            &indices,
            &chains,
            &closed,
            Some(feature.clone()),
            BandProfile::Chamfer,
            1e-9,
            None,
            f64::NAN,
            0.0,
            f64::INFINITY,
        )
        .expect("the pick reads");
        let asked = read_room_of_pick(
            &positions,
            &indices,
            &chains,
            &closed,
            Some(feature),
            BandProfile::Chamfer,
            1e-9,
            None,
            size,
            0.0,
            f64::INFINITY,
        )
        .expect("the pick reads");

        let measured = |reading: &RoomReading| {
            reading
                .candidates
                .iter()
                .filter(|c| c.source.kind == RoomReadingKind::SelfRoom && c.room.is_finite())
                .count()
        };

        println!(
            "{name} self rows measured: {} unasked, {} asked at {size}",
            measured(&today),
            measured(&asked)
        );

        assert!(
            measured(&asked) > measured(&today),
            "a self room between the pick's room and the size must be a number, not infinity"
        );
        assert_eq!(
            asked.room, today.room,
            "relaxing the ceiling may not move the room"
        );
        assert_eq!(asked.room_set_by, today.room_set_by);
    }
}

/// Every drop names ORIGINAL chains, including drops from the second round, and each drop says
/// WHICH round it fell in, so a first-reading drop can be told from one the re-tagging caused.
#[test]
fn every_drop_names_original_chains_and_says_which_round_it_fell_in() {
    for (size, holes_first) in [(1.0, false), (1.0, true), (1.6, false), (1.6, true)] {
        let pick = fixture::pick_with(holes_first, false);
        let (positions, _, chains) = &pick;
        let settled = settle(&pick, size);

        assert!(!settled.dropped.is_empty());
        assert!(
            settled.dropped.iter().any(|d| d.round > 1),
            "at these sizes the re-tagging condemns a chain no first reading could"
        );

        for one in &settled.dropped {
            assert!(one.round >= 1);
            assert!(one.round <= settled.rounds);
            assert_eq!(
                one.because.chain, one.chain,
                "a reason is about the chain it condemns, in the caller's own numbering"
            );

            if one.because.kind != RoomReadingKind::ChainToChain {
                continue;
            }

            // The partner is a real chain of the PICK, and it is the one geometrically nearest the
            // chain it condemned.
            let other = one.because.other_chain;
            assert!(other >= 0);
            assert!(other < chains.len() as i32);
            assert_ne!(other, one.chain);
            assert!(
                nearest_chains_to(positions, chains, one.chain).contains(&other),
                "chain {} was condemned by something it stands closest to, not by a hole across the plate",
                one.chain
            );
        }
    }
}

/// A single chain refused by its own SELF reading says Self, not Foreign. A pick of one fills no
/// candidate table, so the loop names it from the reading's own `room_set_by`.
#[test]
fn a_last_chain_standing_says_what_actually_set_its_room() {
    // The cube's top rim alone: nothing foreign bounds its bands within the face, so what limits
    // it is the face running out of ITSELF - a self room of half the cube.
    let (positions, indices) = cube_solid(1.0);
    let feature = feature_edges_over(&positions, &indices, several_chains_fixtures::FOLD_DEGREES);
    let chains = vec![vec![4, 5, 6, 7]];

    let alone = read_room(
        &positions,
        &indices,
        &chains[0],
        true,
        Some(feature.clone()),
        BandProfile::Chamfer,
        1e-9,
        None,
        0.0,
        f64::INFINITY,
    )
    .expect("the chain reads");
    assert_ne!(
        alone.room_set_by.kind,
        RoomReadingKind::None,
        "a single chain's reading says what set its room too"
    );

    let settled = PickThatFits::read(
        &positions,
        &indices,
        &chains,
        &[true],
        Some(feature),
        BandProfile::Chamfer,
        alone.room * 4.0,
        1e-9,
        None,
        0.0,
        f64::INFINITY,
    )
    .expect("the pick reads");

    assert_eq!(settled.chains.len(), 0);
    assert_eq!(settled.dropped.len(), 1);
    assert_eq!(
        settled.dropped[0].because.kind, alone.room_set_by.kind,
        "the reason is the reading's own, not a guess at Foreign"
    );
    assert_eq!(settled.dropped[0].because.chain, 0);
}

/// Which other chains of the pick stand closest to this one, by their vertices - a SET, because the
/// fixture is deliberately regular.
fn nearest_chains_to(positions: &[Vector3], chains: &[Vec<i32>], chain: i32) -> HashSet<i32> {
    let mut apart_from = vec![f64::MAX; chains.len()];
    for other in 0..chains.len() {
        if other as i32 == chain {
            continue;
        }

        for &a in &chains[chain as usize] {
            for &b in &chains[other] {
                apart_from[other] =
                    apart_from[other].min((positions[a as usize] - positions[b as usize]).length());
            }
        }
    }

    let least = apart_from.iter().copied().fold(f64::INFINITY, f64::min);
    (0..chains.len() as i32)
        .filter(|&k| apart_from[k as usize] <= least + 1e-9)
        .collect()
}

fn sorted(chain: &[i32]) -> Vec<i32> {
    let mut chain = chain.to_vec();
    chain.sort();
    chain
}

fn settled_chains(pick: &SolidWithChains, size: f64) -> Vec<Vec<i32>> {
    settle(pick, size)
        .chains
        .iter()
        .map(|&k| sorted(&pick.2[k as usize]))
        .collect()
}

fn named_at(pick: &SolidWithChains, size: f64) -> Vec<i32> {
    let reading = fixture::read(&pick.0, &pick.1, &pick.2, size);
    ChainsThatCannotFit::read(&reading, size)
        .dropped
        .iter()
        .map(|d| d.chain)
        .collect()
}

/// Closed and consistently oriented: every directed edge used once, its opposite once.
fn is_closed_and_consistent(mesh: &LevelLineMesh) -> bool {
    let mut seen: HashMap<(i32, i32), i32> = HashMap::new();
    for t in 0..mesh.triangle_count() {
        if mesh.is_removed(t) {
            continue;
        }

        let tri = mesh.triangles()[t as usize];
        for e in [(tri.a, tri.b), (tri.b, tri.c), (tri.c, tri.a)] {
            *seen.entry(e).or_insert(0) += 1;
        }
    }

    seen.iter()
        .all(|(e, &count)| count == 1 && seen.get(&(e.1, e.0)) == Some(&1))
}
