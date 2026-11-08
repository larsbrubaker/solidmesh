//! The gate the level-line measurement tests (bridge diagnosis, Many Holes rim cost, pass-one
//! pinch, needle cascade meter) share: whether a measurement is asked for (`LEVELLINE_COST=1`), its
//! cap (`LEVELLINE_COST_CAP`, default three million), the one Many Holes face-rim witness fixture
//! and where a line is said (console plus `LEVELLINE_COST_LOG`). Held once here so the four share
//! one copy.

#![allow(dead_code)]

use std::rc::Rc;

use solidmesh::bevel::level_line::{BandProfile, WorkBudget};

use crate::level_line_fixture::LevelLineFixture;

/// The measurement runs only when `LEVELLINE_COST=1`.
pub fn asked() -> bool {
    std::env::var("LEVELLINE_COST").as_deref() == Ok("1")
}

/// The cap of the measurements whose default is three million.
pub const DEFAULT_CAP: i64 = 3_000_000;

/// `LEVELLINE_COST_CAP` where it parses to a positive count, [`DEFAULT_CAP`] otherwise.
pub fn cap() -> i64 {
    cap_or(DEFAULT_CAP)
}

/// The cap of a measurement with its own default (the Many Holes rim cost reads five million).
pub fn cap_or(default: i64) -> i64 {
    std::env::var("LEVELLINE_COST_CAP")
        .ok()
        .and_then(|asked| asked.parse::<i64>().ok())
        .filter(|&asked| asked > 0)
        .unwrap_or(default)
}

/// A fresh budget of [`cap`] units.
pub fn capped_budget() -> Rc<WorkBudget> {
    budget_of(cap())
}

/// A fresh budget of `limit` units.
pub fn budget_of(limit: i64) -> Rc<WorkBudget> {
    Rc::new(WorkBudget::new(limit).expect("a positive cap"))
}

/// The name (without extension) of the one `many-holes-facerim-*chains-r05.txt`
/// witness fixture.
pub fn witness_name() -> String {
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
    names[0].trim_end_matches(".txt").to_string()
}

/// The fixture's profile.
pub fn profile(fixture: &LevelLineFixture) -> BandProfile {
    if fixture.chamfer {
        BandProfile::Chamfer
    } else {
        BandProfile::Fillet
    }
}

/// The fixture's feature-edge list as the kernel's feature-edge test.
pub fn feature_test(
    fixture: &Rc<LevelLineFixture>,
) -> Option<solidmesh::bevel::level_line::seam_edges::FeatureEdgeTest> {
    let features = Rc::clone(fixture);
    Some(Rc::new(move |a, b| features.is_feature_edge(a, b)))
}

/// The line on the console, and appended to `LEVELLINE_COST_LOG` where one is named (a
/// test host swallows the console).
pub fn say(line: &str) {
    println!("{line}");
    if let Ok(log) = std::env::var("LEVELLINE_COST_LOG") {
        if !log.is_empty() {
            use std::io::Write;
            let mut file = std::fs::OpenOptions::new()
                .create(true)
                .append(true)
                .open(&log)
                .expect("the cost log");
            writeln!(file, "{line}").expect("the cost log");
        }
    }
}

/// An integer with thousands separators, so the cost tables read at a glance.
pub fn n0(value: i64) -> String {
    let digits = value.unsigned_abs().to_string();
    let mut grouped = String::new();
    for (i, digit) in digits.chars().enumerate() {
        if i > 0 && (digits.len() - i).is_multiple_of(3) {
            grouped.push(',');
        }
        grouped.push(digit);
    }
    if value < 0 {
        format!("-{grouped}")
    } else {
        grouped
    }
}
