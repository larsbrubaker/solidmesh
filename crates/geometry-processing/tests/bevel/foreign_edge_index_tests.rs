//! The per-chain FOREIGN reading is about the part NEAR that chain, not about the whole region its
//! chains bound - and it answers the same numbers with the nearest-first index as with a plain pass
//! over every foreign edge. Parameter rows are a loop over the same-named test; every row is run
//! and the failing rows are named together.

use std::rc::Rc;
use std::time::Instant;

use geometry_processing::bevel::level_line::band_gap_reading;
use geometry_processing::bevel::level_line::chain_on_the_mesh;
use geometry_processing::bevel::level_line::foreign_edge_room;
use geometry_processing::bevel::level_line::foreign_room_counters;
use geometry_processing::bevel::level_line::{BandProfile, RoomReading, WorkBudget};
use vector_math::Vector3;

use crate::five_hole_plate_fixture;
use crate::grid_hole_plate_fixture;
use crate::hourglass_beside_square_fixture;
use crate::level_line_feature_edges::feature_edges_over;
use crate::level_line_fixture::LevelLineFixture;
use crate::open_chain_fixtures;
use crate::several_chains_fixtures;

type Pick = (Vec<Vector3>, Vec<(i32, i32, i32)>, Vec<Vec<i32>>);

/// The holes of the grid plate WITHOUT its outer rim, so the plate's outer boundary
/// is foreign to every hole chain.
fn holes_only(holes: i32) -> Pick {
    let (positions, indices, chains) = grid_hole_plate_fixture::pick(holes);
    (positions, indices, chains.into_iter().skip(1).collect())
}

fn read(pick: &Pick, name_every_candidate_under: f64) -> RoomReading {
    let closed = vec![true; pick.2.len()];
    band_gap_reading::read_room_of_pick(
        &pick.0,
        &pick.1,
        &pick.2,
        &closed,
        Some(grid_hole_plate_fixture::feature_edges(&pick.0, &pick.1)),
        BandProfile::Chamfer,
        1e-9,
        None,
        name_every_candidate_under,
        0.0,
        f64::INFINITY,
    )
    .expect("the reading")
}

/// Runs `f` with the index turned off (every foreign edge read), turning
/// it back on afterwards even if `f` panics.
fn reading_every_foreign_edge<R>(f: impl FnOnce() -> R) -> R {
    struct Restore;
    impl Drop for Restore {
        fn drop(&mut self) {
            foreign_edge_room::set_read_every_foreign_edge(false);
        }
    }

    foreign_edge_room::set_read_every_foreign_edge(true);
    let _restore = Restore;
    f()
}

/// The foreign edges read (all edges read less the chain-to-chain ones) and the edges offered.
fn foreign_read_and_offered() -> (i64, i64) {
    foreign_room_counters::with(|t| {
        (
            t.edges_read - t.pair_edges_read,
            t.edges_per_chain
                .as_ref()
                .map_or(0, |v| v.iter().map(|&n| n as i64).sum()),
        )
    })
}

/// A chain's foreign reading is about its own neighbourhood.
#[test]
fn a_chains_foreign_reading_is_about_its_own_neighbourhood() {
    let pick = holes_only(4);
    foreign_room_counters::reset();
    let reading = read(&pick, f64::NAN);
    let (foreign_edges_read, offered) = foreign_read_and_offered();

    // The same reading with the index turned off: the plain pass this replaces.
    foreign_room_counters::reset();
    let plain_edges_read = reading_every_foreign_edge(|| {
        read(&pick, f64::NAN);
        foreign_read_and_offered().0
    });

    println!(
        "holes only, {} chains: room {:?}, foreign offered {offered}, foreign read \
         {foreign_edges_read}, plain pass {plain_edges_read}",
        pick.2.len(),
        reading.room
    );

    // The reading still happened at all - the plate's boundary IS foreign to every hole.
    assert!(offered > 16 * 20, "offered {offered}");
    assert!(
        foreign_edges_read < offered / 2,
        "foreign read {foreign_edges_read}, offered {offered}"
    );
    assert!(
        plain_edges_read >= offered,
        "plain pass {plain_edges_read}, offered {offered}"
    );
}

/// The real Many Holes rim (25 chains on
/// 15,214 triangles) held to three ceilings. With `LEVELLINE_COST=1` the reading is run with the
/// index off as well and both are compared.
#[test]
fn the_witness_does_not_read_the_whole_boundary_per_chain() {
    let directory = format!(
        "{}/tests/bevel/level_line_fixtures",
        env!("CARGO_MANIFEST_DIR")
    );
    let names: Vec<String> = std::fs::read_dir(&directory)
        .expect("the fixture directory")
        .filter_map(|e| e.ok())
        .map(|e| e.file_name().to_string_lossy().into_owned())
        .filter(|n| n.starts_with("many-holes-facerim-") && n.ends_with("chains-r05.txt"))
        .collect();
    assert_eq!(names.len(), 1, "exactly one witness fixture: {names:?}");
    let fixture = Rc::new(LevelLineFixture::load(names[0].trim_end_matches(".txt")));
    let profile = if fixture.chamfer {
        BandProfile::Chamfer
    } else {
        BandProfile::Fillet
    };

    let reading_of = |budget: Rc<WorkBudget>| {
        let features = Rc::clone(&fixture);
        band_gap_reading::read_room_of_pick(
            &fixture.positions,
            &fixture.indices,
            &fixture.chains,
            &fixture.closed,
            Some(Rc::new(move |a, b| features.is_feature_edge(a, b))),
            profile,
            1e-9,
            Some(budget),
            fixture.size,
            0.0,
            f64::INFINITY,
        )
        .expect("the reading")
    };

    foreign_room_counters::reset();
    let indexed_budget = Rc::new(WorkBudget::new(5_000_000).expect("a positive limit"));
    let started = Instant::now();
    let reading = reading_of(indexed_budget);
    let indexed_seconds = started.elapsed().as_secs_f64();
    let (edges_read, offered) = foreign_read_and_offered();
    let (units, search, kept, fallbacks) = foreign_room_counters::with(|t| {
        (
            t.foreign_units,
            t.search_edges_read,
            t.kept_edges,
            t.bracket_fallbacks,
        )
    });
    println!(
        "witness indexed: room {:?} set by {:?}, foreign offered {offered}, read {edges_read} \
         (search {search}, kept {kept}), units {units}, fallbacks {fallbacks}, \
         {indexed_seconds:.1} s",
        reading.room, reading.room_set_by.kind
    );

    if std::env::var("LEVELLINE_COST").as_deref() == Ok("1") {
        foreign_room_counters::reset();
        let plain_budget = Rc::new(WorkBudget::new(5_000_000).expect("a positive limit"));
        let started = Instant::now();
        let plain = reading_every_foreign_edge(|| reading_of(plain_budget));
        let (plain_read, _) = foreign_read_and_offered();
        let plain_units = foreign_room_counters::with(|t| t.foreign_units);
        println!(
            "witness plain: read {plain_read}, units {plain_units}, {:.1} s",
            started.elapsed().as_secs_f64()
        );

        // The real part's own numbers: the same room, named by the same reading, with the same
        // table - for a fraction of the readings.
        assert_eq!(plain.room.to_bits(), reading.room.to_bits());
        assert_eq!(plain.room_set_by, reading.room_set_by);
        assert_eq!(rows(&plain), rows(&reading));
        assert!(plain_read > edges_read);
    }

    assert!(search < 25 * 1_718 / 4, "search edges read {search}");
    assert!(edges_read < 60_000, "foreign edges read {edges_read}");
    assert!(units < 400_000, "foreign units {units}");
}

/// Every number of the reading is the same with the
/// index as without it - the room, which reading named it, and every candidate row in order.
#[test]
fn the_indexed_reading_is_the_unindexed_reading() {
    let mut failures = Vec::new();
    for fixture in [
        "A",
        "B",
        "C",
        "five-hole",
        "hourglass",
        "grid-rim",
        "grid-holes",
        "open-run",
        "sheared-fillet",
        "sheared-fillet-moved",
    ] {
        if let Err(why) = the_indexed_reading_is_the_unindexed_reading_on(fixture) {
            failures.push(format!("{fixture}: {why}"));
        }
    }

    assert!(failures.is_empty(), "{}", failures.join("\n"));
}

fn the_indexed_reading_is_the_unindexed_reading_on(fixture: &str) -> Result<(), String> {
    let (positions, indices, chains, closed, fold, size, profile, varying) = pick(fixture);

    // The branch the index's inequality exists for only runs where a chain's unit setback VARIES
    // along it, so the fixtures meant to reach it are made to prove they do.
    if varying {
        let edges = chain_on_the_mesh::chain_edges(&chains[0], closed[0]);
        let units = chain_on_the_mesh::unit_setbacks(
            &positions, &indices, &chains[0], closed[0], &edges, profile,
        )
        .map_err(|e| format!("unit setbacks: {e:?}"))?;
        let max = units.iter().copied().fold(f64::NEG_INFINITY, f64::max);
        let min = units.iter().copied().fold(f64::INFINITY, f64::min);
        check(max - min > 1e-12 * max, || {
            format!("the unit setback does not vary: {min} to {max}")
        })?;
    }

    for asked in [f64::NAN, size] {
        let reading = || {
            band_gap_reading::read_room_of_pick(
                &positions,
                &indices,
                &chains,
                &closed,
                Some(feature_edges_over(&positions, &indices, fold)),
                profile,
                1e-9,
                None,
                asked,
                0.0,
                f64::INFINITY,
            )
            .expect("the reading")
        };

        foreign_room_counters::reset();
        let indexed = reading();
        if varying {
            // The bracket, the keep filter and the bisection were exercised by this reading.
            let (search, kept) =
                foreign_room_counters::with(|t| (t.search_edges_read, t.kept_edges));
            check(search > 0, || {
                format!("asked {asked}: no search edges read")
            })?;
            check(kept > 0, || format!("asked {asked}: no edges kept"))?;
        }

        let every = reading_every_foreign_edge(reading);

        check(indexed.room.to_bits() == every.room.to_bits(), || {
            format!(
                "asked {asked}: room {:?} against {:?}",
                indexed.room, every.room
            )
        })?;
        check(indexed.room_set_by == every.room_set_by, || {
            format!(
                "asked {asked}: set by {:?} against {:?}",
                indexed.room_set_by, every.room_set_by
            )
        })?;
        check(indexed.candidates.len() == every.candidates.len(), || {
            format!(
                "asked {asked}: {} candidates against {}",
                indexed.candidates.len(),
                every.candidates.len()
            )
        })?;
        for (row, (one, other)) in indexed.candidates.iter().zip(&every.candidates).enumerate() {
            let same = one.source == other.source
                && one.room.to_bits() == other.room.to_bits()
                && one.room_without_the_other.to_bits() == other.room_without_the_other.to_bits()
                && one.room_without_the_other_at_most.to_bits()
                    == other.room_without_the_other_at_most.to_bits()
                && members(&one.group_members) == members(&other.group_members);
            check(same, || {
                format!("asked {asked}: row {row} {one:?} against {other:?}")
            })?;
        }
    }

    Ok(())
}

fn check(holds: bool, why: impl FnOnce() -> String) -> Result<(), String> {
    if holds {
        Ok(())
    } else {
        Err(why())
    }
}

/// The whole candidate table as one string, every number in its exact form.
fn rows(reading: &RoomReading) -> String {
    reading
        .candidates
        .iter()
        .map(|row| {
            format!(
                "{:?} {} {} {:?} {:?} {:?} {}",
                row.source.kind,
                row.source.chain,
                row.source.other_chain,
                row.room,
                row.room_without_the_other,
                row.room_without_the_other_at_most,
                members(&row.group_members)
            )
        })
        .collect::<Vec<_>>()
        .join("\n")
}

/// A group's members as one string, so "no group" and "these members in this order"
/// compare.
fn members(group: &Option<Vec<i32>>) -> String {
    match group {
        None => "none".to_string(),
        Some(g) => g
            .iter()
            .map(|m| m.to_string())
            .collect::<Vec<_>>()
            .join(","),
    }
}

/// The grid plate's holes SHEARED, so a FILLET's unit setback varies along every
/// rim chain; only the holes are picked.
fn sheared_holes(moved: bool) -> Pick {
    let pick = holes_only(2);
    let motion = several_chains_fixtures::motion();
    let positions = pick
        .0
        .iter()
        .map(|p| Vector3::new(p.x + (0.35 * p.z), p.y, p.z))
        .map(|p| if moved { p.transform(motion) } else { p })
        .collect();
    (positions, pick.1, pick.2)
}

type Row = (
    Vec<Vector3>,
    Vec<(i32, i32, i32)>,
    Vec<Vec<i32>>,
    Vec<bool>,
    f64,
    f64,
    BandProfile,
    bool,
);

fn all_closed(pick: Pick, fold: f64, size: f64, profile: BandProfile, varying: bool) -> Row {
    let closed = vec![true; pick.2.len()];
    (pick.0, pick.1, pick.2, closed, fold, size, profile, varying)
}

fn pick(fixture: &str) -> Row {
    match fixture {
        "sheared-fillet" | "sheared-fillet-moved" => all_closed(
            sheared_holes(fixture.ends_with("moved")),
            grid_hole_plate_fixture::FOLD_DEGREES,
            grid_hole_plate_fixture::SIZE,
            BandProfile::Fillet,
            true,
        ),
        "A" | "B" | "C" => all_closed(
            several_chains_fixtures::named(fixture),
            several_chains_fixtures::FOLD_DEGREES,
            0.1,
            BandProfile::Chamfer,
            false,
        ),
        "five-hole" => all_closed(
            five_hole_plate_fixture::pick(),
            five_hole_plate_fixture::FOLD_DEGREES,
            five_hole_plate_fixture::SIZE,
            BandProfile::Chamfer,
            false,
        ),
        "hourglass" => all_closed(
            hourglass_beside_square_fixture::pick(),
            hourglass_beside_square_fixture::FOLD_DEGREES,
            1.0,
            BandProfile::Chamfer,
            false,
        ),
        "grid-rim" => all_closed(
            grid_hole_plate_fixture::pick(2),
            grid_hole_plate_fixture::FOLD_DEGREES,
            grid_hole_plate_fixture::SIZE,
            BandProfile::Chamfer,
            false,
        ),
        "grid-holes" => all_closed(
            holes_only(3),
            grid_hole_plate_fixture::FOLD_DEGREES,
            grid_hole_plate_fixture::SIZE,
            BandProfile::Chamfer,
            false,
        ),
        "open-run" => {
            // A run of the cylinder's top rim: an OPEN chain, so the reading exempts the edges
            // running out of its two ends.
            let (cylinder, faces) = open_chain_fixtures::cylinder();
            (
                cylinder,
                faces,
                vec![open_chain_fixtures::top_rim_run(4)],
                vec![false],
                open_chain_fixtures::FOLD_DEGREES,
                1.0,
                BandProfile::Chamfer,
                false,
            )
        }
        other => panic!("no such fixture: {other}"),
    }
}
