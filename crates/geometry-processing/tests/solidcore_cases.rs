//! solidcore's user-reported fillet and chamfer recipes, replayed against solidmesh's bevel and
//! graded against their closed-form answers.
//!
//! The cases are solidcore's `fixtures/solidcraft` files that hold a fillet or chamfer, vendored
//! whole and unchanged under `solidcore_cases/fixtures/` and pinned by `solidcore_cases/pin.json`
//! (Apache-2.0, `solidcore_cases/LICENSE-APACHE`; THIRD_PARTY_NOTICES.md). Each case's steps are
//! rebuilt with solidmesh's own primitives, outline extrusions and revolves, booleans and
//! `bevel_part` (`replay.rs`), at three resolutions, and graded by the limit they converge to
//! (`grade.rs`, whose header states the rule).
//!
//! Every case comes to one of: **pass**; **partly graded** (everything it expects that a mesh can
//! be graded on holds, and it also expects something a mesh cannot show); **unported**, naming the
//! op solidmesh has no counterpart for and held in `solidcore_cases/unported.json`;
//! **known-failing**, listed under `solidcore_cases/known_failing/` with why and what it does
//! today (`known_failing.rs`), of which the untriaged ones are counted apart; or **fail**. Every
//! pass with a volume expectation and a blend of its own is graded again with its last fillet or
//! chamfer left out, and must then fail: that is the proof the grading sees the blend. The test
//! fails when any case moves: no case is read at all, an unlisted case fails, a listed one passes
//! or fails differently ("unlist it"), the unported set changes, a listed case no longer exists,
//! or a case is lost by the runner. It
//! prints one line per case and the totals, and with `SOLIDCORE_CASES_RESULTS=<path>` also writes
//! every row (key, status, seen, detail; an unported row's `seen` is its op) as JSON there, which
//! the sync tool reads.

#[path = "solidcore_cases/case_file.rs"]
mod case_file;
#[path = "solidcore_cases/grade.rs"]
mod grade;
#[path = "solidcore_cases/known_failing.rs"]
mod known_failing;
#[path = "solidcore_cases/lf_sha.rs"]
mod lf_sha;
#[path = "bevel/mesh_measurements.rs"]
mod mesh_measurements;
#[path = "solidcore_cases/profile.rs"]
mod profile;
#[path = "solidcore_cases/replay.rs"]
mod replay;

use std::sync::atomic::{AtomicUsize, Ordering};
use std::sync::Mutex;

use case_file::{pinned_fixtures, read_fixture, Case};
use grade::{grade, Graded};
use known_failing::{is_untriaged, read_known_failing, read_unported};

/// One row of the table: printed, and kept for the results file.
fn row(rows: &mut Vec<serde_json::Value>, status: &str, key: &str, seen: &str, detail: &str) {
    let mut line = format!("{status:<13} {key}");
    if !seen.is_empty() {
        line.push_str(&format!("  {seen}"));
    }
    if !detail.is_empty() {
        line.push_str(&format!("  [{detail}]"));
    }
    println!("{line}");
    rows.push(serde_json::json!({
        "key": key,
        "status": status,
        "seen": seen,
        "detail": detail,
    }));
}

/// What one case came to.
enum Outcome {
    /// Passed; whether it is only partly graded, its detail, and whether leaving its last blend
    /// out still passes (which would mean the grading cannot see the blend).
    Pass {
        partly_graded: bool,
        detail: String,
        passes_without_its_blend: bool,
    },
    Unported(String),
    Failing(Graded),
    /// The runner lost the case (a grading thread panicked).
    Lost,
}

fn outcome_of(case: &Case) -> Outcome {
    if let Some(op) = case.unported_op() {
        return Outcome::Unported(op.to_string());
    }
    let graded = grade(case, false);
    if !graded.issues.is_empty() {
        return Outcome::Failing(graded);
    }
    let passes_without_its_blend =
        graded.volume_graded && case.has_blend() && grade(case, true).issues.is_empty();
    Outcome::Pass {
        partly_graded: graded.partly_graded,
        detail: graded.detail,
        passes_without_its_blend,
    }
}

/// Grades every case, spread over the machine's threads, each result kept at its case's index so
/// the table reads in file order whatever finished first.
fn run_all(cases: &[Case]) -> Vec<Outcome> {
    let next = AtomicUsize::new(0);
    let results: Mutex<Vec<Option<Outcome>>> = Mutex::new((0..cases.len()).map(|_| None).collect());
    let threads = std::thread::available_parallelism()
        .map_or(4, |n| n.get())
        .min(8);
    std::thread::scope(|scope| {
        for _ in 0..threads {
            scope.spawn(|| loop {
                let index = next.fetch_add(1, Ordering::Relaxed);
                let Some(case) = cases.get(index) else {
                    break;
                };
                // A case whose grading panics is lost, and reported as such, not the whole run.
                let outcome =
                    std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| outcome_of(case)))
                        .unwrap_or(Outcome::Lost);
                if let Ok(mut results) = results.lock() {
                    results[index] = Some(outcome);
                }
            });
        }
    });
    results
        .into_inner()
        .unwrap_or_default()
        .into_iter()
        .map(|outcome| outcome.unwrap_or(Outcome::Lost))
        .collect()
}

#[test]
fn solidcore_fillet_and_chamfer_cases_hold_their_outcomes() {
    let started = std::time::Instant::now();
    let fixtures = pinned_fixtures().unwrap_or_else(|e| panic!("{e}"));
    let mut cases = Vec::new();
    for path in &fixtures {
        cases.extend(read_fixture(path).unwrap_or_else(|e| panic!("{e}")));
    }
    let mut listed = read_known_failing().unwrap_or_else(|e| panic!("{e}"));
    let mut held_unported = read_unported().unwrap_or_else(|e| panic!("{e}"));
    let outcomes = run_all(&cases);

    let (mut passed, mut partly, mut failed, mut known, mut untriaged, mut unported) =
        (0, 0, 0, 0, 0, 0);
    let mut moved: Vec<String> = Vec::new();
    let mut rows: Vec<serde_json::Value> = Vec::new();
    if cases.is_empty() {
        moved.push("no cases were read: the fixtures folder or pin.json is empty".to_string());
    }
    for (case, outcome) in cases.iter().zip(&outcomes) {
        let key = case.key();
        let entry = listed.remove(&key);
        let held_op = held_unported.remove(&key);
        if entry.as_ref().is_some_and(|e| e.reason.trim().is_empty()) {
            moved.push(format!(
                "{key} is listed as known-failing with no reason: say what goes wrong"
            ));
        }
        if let (Some(op), false) = (&held_op, matches!(outcome, Outcome::Unported(_))) {
            moved.push(format!(
                "{key} is held as unported ({op}) but is not: take it out of unported.json"
            ));
        }
        match outcome {
            Outcome::Lost => {
                failed += 1;
                row(&mut rows, "LOST", &key, "", "");
                moved.push(format!(
                    "{key} was lost by the runner (its grading panicked)"
                ));
            }
            Outcome::Pass {
                partly_graded,
                detail,
                passes_without_its_blend,
            } => {
                if *partly_graded {
                    partly += 1;
                    row(&mut rows, "partly graded", &key, "", detail);
                } else {
                    passed += 1;
                    row(&mut rows, "pass", &key, "", detail);
                }
                if *passes_without_its_blend {
                    moved.push(format!(
                        "{key} also passes with its last blend left out: the grading cannot see \
                         the blend"
                    ));
                }
                if entry.is_some() {
                    moved.push(format!("{key} passes now: unlist it"));
                }
            }
            Outcome::Unported(op) => {
                unported += 1;
                row(
                    &mut rows,
                    "unported",
                    &key,
                    op,
                    &format!("no {op} in solidmesh"),
                );
                if entry.is_some() {
                    moved.push(format!("{key} is unported ({op}): unlist it"));
                }
                match held_op {
                    Some(held) if held == *op => {}
                    Some(held) => moved.push(format!(
                        "{key} is held as unported for {held} but lacks {op}: update unported.json"
                    )),
                    None => moved.push(format!(
                        "{key} is unported ({op}) but not held in unported.json"
                    )),
                }
            }
            Outcome::Failing(graded) => {
                let seen = graded.issues.join("; ");
                match entry {
                    Some(entry) if entry.seen == seen => {
                        if is_untriaged(&entry) {
                            untriaged += 1;
                            row(&mut rows, "untriaged", &key, &seen, &graded.detail);
                        } else {
                            known += 1;
                            row(&mut rows, "known-failing", &key, &seen, &graded.detail);
                        }
                    }
                    Some(entry) => {
                        failed += 1;
                        row(&mut rows, "FAIL", &key, &seen, &graded.detail);
                        moved.push(format!(
                            "{key} fails differently: listed as \"{}\", now \"{seen}\": unlist it \
                             or rewrite its entry",
                            entry.seen
                        ));
                    }
                    None => {
                        failed += 1;
                        row(&mut rows, "FAIL", &key, &seen, &graded.detail);
                        moved.push(format!("{key} fails: {seen}"));
                    }
                }
            }
        }
    }
    for key in listed.keys() {
        moved.push(format!(
            "{key} is listed as known-failing but no such case exists: unlist it"
        ));
    }
    for key in held_unported.keys() {
        moved.push(format!(
            "{key} is held as unported but no such case exists: take it out of unported.json"
        ));
    }
    println!(
        "{} cases: {passed} pass, {partly} partly graded, {failed} fail, {known} known-failing, \
         {untriaged} untriaged, {unported} unported ({:.1} s)",
        cases.len(),
        started.elapsed().as_secs_f64()
    );
    // The sync tool reads the outcomes from here rather than from the printed table.
    if let Ok(path) = std::env::var("SOLIDCORE_CASES_RESULTS") {
        let text = serde_json::to_string_pretty(&rows).unwrap_or_default();
        std::fs::write(&path, format!("{text}\n"))
            .unwrap_or_else(|e| panic!("SOLIDCORE_CASES_RESULTS {path}: {e}"));
    }
    assert!(
        moved.is_empty(),
        "{} cases moved:\n{}",
        moved.len(),
        moved.join("\n")
    );
}
