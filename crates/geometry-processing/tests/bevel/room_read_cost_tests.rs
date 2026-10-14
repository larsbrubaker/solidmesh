//! What a room reading is allowed to COST on a chain whose setback varies along it (the Many Holes
//! 243-facet face at r0.25), and that making it cheaper did not move the number it answers.

use std::rc::Rc;

use geometry_processing::bevel::level_line::band_gap_reading::read_room;
use geometry_processing::bevel::level_line::{BandProfile, WorkBudget};

use crate::level_line_fixture::LevelLineFixture;

/// The room this pick has always answered, read off the code before the bracketed search
/// (2026-09-19).
const KNOWN_ROOM: f64 = 1.06568702132;

/// The ceiling on the reading's work: a "the whole shape of this changed" catch, well above the
/// 57,364 units the bracketed reading spends and far below the 1,172,374 of pass-per-trial.
const WORK_CEILING: i64 = 150_000;

#[test]
fn a_varying_setback_room_is_read_without_a_pass_per_trial_size() {
    let fixture = Rc::new(LevelLineFixture::load("many-holes-face243-r025"));

    // Generous, so a reading that regressed to the per-trial cost still ENDS.
    let budget = Rc::new(WorkBudget::new(5_000_000).expect("a positive limit"));

    let profile = if fixture.chamfer {
        BandProfile::Chamfer
    } else {
        BandProfile::Fillet
    };
    let features = Rc::clone(&fixture);
    let reading = read_room(
        &fixture.positions,
        &fixture.indices,
        &fixture.chains[0],
        true,
        Some(Rc::new(move |a, b| features.is_feature_edge(a, b))),
        profile,
        1e-9,
        Some(budget),
        0.0,
        f64::INFINITY,
    )
    .expect("a valid pick");

    assert!(reading.read);
    assert!(
        (reading.room - KNOWN_ROOM).abs() / KNOWN_ROOM < 1e-9,
        "room {} known {}",
        reading.room,
        KNOWN_ROOM
    );
    assert!(
        reading.work_spent < WORK_CEILING,
        "work {}",
        reading.work_spent
    );
}
