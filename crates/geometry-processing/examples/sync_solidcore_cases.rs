//! A maintainer tool, not part of the published crate (the crate's `exclude` leaves `examples/`
//! out): the file work behind `scripts/sync_solidcore_cases.sh`, which keeps the vendored solidcore
//! cases (`tests/solidcore_cases/`) in step with solidcore. The script fetches solidcore and
//! extracts `fixtures/solidcraft/` at one commit; this tool does the rest in two passes around a
//! run of the `solidcore_cases` test:
//!
//! - `plan`: compares that folder's files that hold a fillet or chamfer case with the vendored
//!   ones (by the runner's own pin hash, `tests/solidcore_cases/lf_sha.rs`, shared through
//!   `#[path]`), copies in the added and changed files, deletes the vendored files gone upstream
//!   with their known-failing and unported entries, rewrites `pin.json`, and writes the state file
//!   given by `--state` (a scratch file the script keeps in its temporary folder) naming the files
//!   and cases that are new, changed (their `steps` or `expect` differ) or gone. It refuses, and
//!   changes nothing, when upstream holds no case file at all, when a sync would remove more than
//!   half the vendored files (unless `--allow-mass-removal`), or when a file or case name is not
//!   plain (`[A-Za-z0-9._-]`, no `..`), since names become paths here;
//! - `apply`: reads the results file the test wrote (`SOLIDCORE_CASES_RESULTS`, given here as
//!   `--results`) and the state file, writes a known-failing entry for each new or changed case
//!   that fails (`untriaged: new at solidcore <sha>`, or `untriaged: expectation changed at
//!   solidcore <sha>` followed by the entry's old reason where it had one, with the `seen` the run
//!   gave), removes the entry of every listed case that now passes, holds the unported cases in
//!   `unported.json`, and prints the report; `--report <path>` also writes it there as JSON.
//!
//! Run it through the script; `cargo run --example sync_solidcore_cases -- plan|apply ...` takes
//! the arguments the script passes. It only reads the extracted folder and the state and results
//! files, and writes inside this crate's `tests/solidcore_cases/`, the state file and the report.

use std::collections::{BTreeMap, BTreeSet};
use std::fs;
use std::path::{Path, PathBuf};

use serde::{Deserialize, Serialize};
use serde_json::Value;

#[path = "../tests/solidcore_cases/lf_sha.rs"]
mod lf_sha;

use lf_sha::lf_sha256;

type Fallible<T> = Result<T, String>;

/// `pin.json`, in the field order it is written in.
#[derive(Serialize, Deserialize)]
struct Pin {
    source: String,
    license: String,
    commit: String,
    commit_date: String,
    files: BTreeMap<String, String>,
}

/// What `plan` found, for `apply`.
#[derive(Serialize, Deserialize, Default)]
struct State {
    short_sha: String,
    added_files: Vec<String>,
    changed_files: Vec<String>,
    removed_files: Vec<String>,
    new_cases: Vec<String>,
    changed_cases: Vec<String>,
    removed_cases: Vec<String>,
}

/// The sync's report: the state plus what the test run decided.
#[derive(Serialize)]
struct Report<'a> {
    commit: &'a str,
    added_files: &'a [String],
    changed_files: &'a [String],
    removed_files: &'a [String],
    new_cases: &'a [String],
    changed_cases: &'a [String],
    removed_cases: &'a [String],
    newly_passing: Vec<String>,
    newly_failing_untriaged: Vec<String>,
    unported_ops: BTreeMap<String, usize>,
    totals: String,
}

fn cases_folder() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("tests")
        .join("solidcore_cases")
}

fn read_text(path: &Path) -> Fallible<String> {
    fs::read_to_string(path).map_err(|e| format!("{}: {e}", path.display()))
}

fn write_text(path: &Path, text: &str) -> Fallible<()> {
    if let Some(parent) = path.parent() {
        fs::create_dir_all(parent).map_err(|e| format!("{}: {e}", parent.display()))?;
    }
    fs::write(path, text).map_err(|e| format!("{}: {e}", path.display()))
}

fn parse(path: &Path) -> Fallible<Value> {
    serde_json::from_str(&read_text(path)?).map_err(|e| format!("{}: {e}", path.display()))
}

/// The file's cases by name, each as its `steps` and `expect`.
fn cases_of(file: &Value) -> BTreeMap<String, (Value, Value)> {
    file.get("cases")
        .and_then(Value::as_array)
        .map(|cases| {
            cases
                .iter()
                .filter_map(|case| {
                    let name = case.get("name")?.as_str()?.to_string();
                    let steps = case.get("steps").cloned().unwrap_or(Value::Null);
                    let expect = case.get("expect").cloned().unwrap_or(Value::Null);
                    Some((name, (steps, expect)))
                })
                .collect()
        })
        .unwrap_or_default()
}

fn has_blend(file: &Value) -> bool {
    cases_of(file).values().any(|(steps, _)| {
        steps.as_array().is_some_and(|steps| {
            steps.iter().any(|step| {
                matches!(
                    step.get("op").and_then(Value::as_str),
                    Some("fillet") | Some("chamfer")
                )
            })
        })
    })
}

fn json_files(folder: &Path) -> Fallible<Vec<String>> {
    let mut names: Vec<String> = fs::read_dir(folder)
        .map_err(|e| format!("{}: {e}", folder.display()))?
        .filter_map(|entry| entry.ok())
        .filter_map(|entry| entry.file_name().to_str().map(str::to_string))
        .filter(|name| name.ends_with(".json"))
        .collect();
    names.sort();
    Ok(names)
}

fn stem(name: &str) -> &str {
    name.strip_suffix(".json").unwrap_or(name)
}

fn remove_if_present(path: &Path) -> Fallible<()> {
    if path.is_dir() {
        fs::remove_dir_all(path).map_err(|e| format!("{}: {e}", path.display()))
    } else if path.exists() {
        fs::remove_file(path).map_err(|e| format!("{}: {e}", path.display()))
    } else {
        Ok(())
    }
}

fn read_unported() -> Fallible<BTreeMap<String, String>> {
    let path = cases_folder().join("unported.json");
    serde_json::from_str(&read_text(&path)?).map_err(|e| format!("{}: {e}", path.display()))
}

fn write_unported(held: &BTreeMap<String, String>) -> Fallible<()> {
    let text = serde_json::to_string_pretty(held).map_err(|e| e.to_string())?;
    write_text(&cases_folder().join("unported.json"), &format!("{text}\n"))
}

/// Whether a file or case name is safe to make a path of: `[A-Za-z0-9._-]` only, never `..`.
fn is_plain(name: &str) -> bool {
    !name.is_empty()
        && !name.contains("..")
        && name
            .chars()
            .all(|c| c.is_ascii_alphanumeric() || matches!(c, '.' | '_' | '-'))
}

/// The plan's arguments.
struct PlanArgs<'a> {
    upstream: &'a Path,
    commit: &'a str,
    short_sha: &'a str,
    date: &'a str,
    state: &'a Path,
    allow_mass_removal: bool,
}

fn plan(args: PlanArgs) -> Fallible<()> {
    let PlanArgs {
        upstream,
        commit,
        short_sha,
        date,
        state,
        allow_mass_removal,
    } = args;
    let folder = cases_folder();
    let fixtures = folder.join("fixtures");
    let upstream_files = json_files(upstream)?;
    if upstream_files.is_empty() {
        return Err(format!(
            "{} holds no case files at {short_sha}: refusing to sync to an empty upstream",
            upstream.display()
        ));
    }
    let mut wanted: BTreeMap<String, (Vec<u8>, Value)> = BTreeMap::new();
    for name in upstream_files {
        let path = upstream.join(&name);
        let file = parse(&path)?;
        if has_blend(&file) {
            if !is_plain(&name) {
                return Err(format!(
                    "upstream file name {name:?} is not plain: refusing"
                ));
            }
            if let Some(bad) = cases_of(&file).keys().find(|case| !is_plain(case)) {
                return Err(format!("{name}: case name {bad:?} is not plain: refusing"));
            }
            let bytes = fs::read(&path).map_err(|e| format!("{}: {e}", path.display()))?;
            wanted.insert(name, (bytes, file));
        }
    }
    let vendored: BTreeSet<String> = json_files(&fixtures)?.into_iter().collect();
    let removing = vendored
        .iter()
        .filter(|name| !wanted.contains_key(*name))
        .count();
    if wanted.is_empty() || (2 * removing > vendored.len() && !allow_mass_removal) {
        return Err(format!(
            "the sync would remove {removing} of {} vendored files and keep {}: refusing without \
             --allow-mass-removal",
            vendored.len(),
            wanted.len()
        ));
    }
    let mut found = State {
        short_sha: short_sha.to_string(),
        ..State::default()
    };
    let mut unported = read_unported()?;
    for (name, (bytes, file)) in &wanted {
        let target = fixtures.join(name);
        let new_cases = cases_of(file);
        if !vendored.contains(name) {
            found.added_files.push(name.clone());
            for case in new_cases.keys() {
                found.new_cases.push(format!("{}/{case}", stem(name)));
            }
        } else {
            let old_bytes = fs::read(&target).map_err(|e| format!("{name}: {e}"))?;
            if lf_sha256(&old_bytes) == lf_sha256(bytes) {
                continue;
            }
            found.changed_files.push(name.clone());
            let old_cases = cases_of(&parse(&target)?);
            for (case, body) in &new_cases {
                let key = format!("{}/{case}", stem(name));
                match old_cases.get(case) {
                    None => found.new_cases.push(key),
                    Some(old) if old != body => found.changed_cases.push(key),
                    Some(_) => {}
                }
            }
            for case in old_cases
                .keys()
                .filter(|case| !new_cases.contains_key(*case))
            {
                let key = format!("{}/{case}", stem(name));
                remove_if_present(
                    &folder
                        .join("known_failing")
                        .join(stem(name))
                        .join(format!("{case}.json")),
                )?;
                unported.remove(&key);
                found.removed_cases.push(key);
            }
        }
        fs::write(&target, bytes).map_err(|e| format!("{}: {e}", target.display()))?;
    }
    for name in vendored.iter().filter(|name| !wanted.contains_key(*name)) {
        found.removed_files.push(name.clone());
        let old_cases = cases_of(&parse(&fixtures.join(name))?);
        for case in old_cases.keys() {
            let key = format!("{}/{case}", stem(name));
            unported.remove(&key);
            found.removed_cases.push(key);
        }
        remove_if_present(&fixtures.join(name))?;
        remove_if_present(&folder.join("known_failing").join(stem(name)))?;
    }
    write_unported(&unported)?;

    let pin = Pin {
        source: "solidcore fixtures/solidcraft".to_string(),
        license: "Apache-2.0".to_string(),
        commit: commit.to_string(),
        commit_date: date.to_string(),
        files: wanted
            .iter()
            .map(|(name, (bytes, _))| (name.clone(), lf_sha256(bytes)))
            .collect(),
    };
    let text = serde_json::to_string_pretty(&pin).map_err(|e| e.to_string())?;
    write_text(&folder.join("pin.json"), &format!("{text}\n"))?;
    let text = serde_json::to_string_pretty(&found).map_err(|e| e.to_string())?;
    write_text(state, &text)
}

/// One row of the runner's results file.
#[derive(Deserialize)]
struct ResultRow {
    key: String,
    status: String,
    seen: String,
}

fn apply(results_path: &Path, state_path: &Path, report_path: Option<&Path>) -> Fallible<()> {
    let folder = cases_folder();
    let state: State =
        serde_json::from_str(&read_text(state_path)?).map_err(|e| format!("state: {e}"))?;
    let rows: Vec<ResultRow> = serde_json::from_str(&read_text(results_path)?)
        .map_err(|e| format!("{}: {e}", results_path.display()))?;
    if rows.is_empty() {
        return Err("the run's results hold no cases: refusing to apply them".to_string());
    }
    let new: BTreeSet<&String> = state.new_cases.iter().collect();
    let changed: BTreeSet<&String> = state.changed_cases.iter().collect();
    let mut unported = read_unported()?;
    let mut newly_passing = Vec::new();
    let mut newly_failing = Vec::new();
    let mut ops: BTreeMap<String, usize> = BTreeMap::new();
    let mut counts: BTreeMap<String, usize> = BTreeMap::new();
    for row in &rows {
        *counts.entry(row.status.clone()).or_insert(0) += 1;
        let Some((file_stem, case)) = row.key.split_once('/') else {
            return Err(format!("a result row with no case: {}", row.key));
        };
        if !is_plain(file_stem) || !is_plain(case) {
            return Err(format!("result key {:?} is not plain: refusing", row.key));
        }
        let entry = folder
            .join("known_failing")
            .join(file_stem)
            .join(format!("{case}.json"));
        match row.status.as_str() {
            "pass" | "partly graded" if entry.exists() => {
                remove_if_present(&entry)?;
                newly_passing.push(row.key.clone());
            }
            "FAIL" if new.contains(&row.key) || changed.contains(&row.key) => {
                let reason = if new.contains(&row.key) {
                    format!("untriaged: new at solidcore {}", state.short_sha)
                } else {
                    // A listed case whose expectation changed keeps what was known about it.
                    let old: Option<String> = fs::read_to_string(&entry)
                        .ok()
                        .and_then(|text| serde_json::from_str::<Value>(&text).ok())
                        .and_then(|v| v.get("reason")?.as_str().map(str::to_string));
                    match old {
                        Some(old) => format!(
                            "untriaged: expectation changed at solidcore {}; before: {old}",
                            state.short_sha
                        ),
                        None => format!(
                            "untriaged: expectation changed at solidcore {}",
                            state.short_sha
                        ),
                    }
                };
                let text = serde_json::to_string_pretty(
                    &serde_json::json!({ "reason": reason, "seen": row.seen }),
                )
                .map_err(|e| e.to_string())?;
                write_text(&entry, &format!("{text}\n"))?;
                newly_failing.push(row.key.clone());
            }
            "unported" => {
                *ops.entry(row.seen.clone()).or_insert(0) += 1;
                remove_if_present(&entry)?;
                unported.insert(row.key.clone(), row.seen.clone());
            }
            _ => {}
        }
    }
    let unported_now: BTreeSet<&String> = rows
        .iter()
        .filter(|row| row.status == "unported")
        .map(|row| &row.key)
        .collect();
    unported.retain(|key, _| unported_now.contains(key));
    write_unported(&unported)?;

    let totals = counts
        .iter()
        .map(|(status, count)| format!("{count} {status}"))
        .collect::<Vec<_>>()
        .join(", ");
    let report = Report {
        commit: &state.short_sha,
        added_files: &state.added_files,
        changed_files: &state.changed_files,
        removed_files: &state.removed_files,
        new_cases: &state.new_cases,
        changed_cases: &state.changed_cases,
        removed_cases: &state.removed_cases,
        newly_passing,
        newly_failing_untriaged: newly_failing,
        unported_ops: ops,
        totals,
    };
    println!("solidcore sync at {}", report.commit);
    for (label, list) in [
        ("added files", report.added_files),
        ("changed files", report.changed_files),
        ("removed files", report.removed_files),
        ("new cases", report.new_cases),
        ("changed cases", report.changed_cases),
        ("removed cases", report.removed_cases),
        ("newly passing", report.newly_passing.as_slice()),
        (
            "newly failing, untriaged",
            report.newly_failing_untriaged.as_slice(),
        ),
    ] {
        println!("{label}: {}", list.len());
        for item in list {
            println!("  {item}");
        }
    }
    println!("unported ops: {:?}", report.unported_ops);
    println!("the run before this apply: {}", report.totals);
    if let Some(path) = report_path {
        let text = serde_json::to_string_pretty(&report).map_err(|e| e.to_string())?;
        write_text(path, &format!("{text}\n"))?;
    }
    Ok(())
}

fn argument(args: &[String], flag: &str) -> Option<String> {
    args.iter()
        .position(|a| a == flag)
        .and_then(|i| args.get(i + 1).cloned())
}

fn run(args: &[String]) -> Fallible<()> {
    let need = |flag: &str| argument(args, flag).ok_or(format!("missing {flag}"));
    match args.first().map(String::as_str) {
        Some("plan") => plan(PlanArgs {
            upstream: Path::new(&need("--upstream")?),
            commit: &need("--commit")?,
            short_sha: &need("--short")?,
            date: &need("--date")?,
            state: Path::new(&need("--state")?),
            allow_mass_removal: args.iter().any(|a| a == "--allow-mass-removal"),
        }),
        Some("apply") => apply(
            Path::new(&need("--results")?),
            Path::new(&need("--state")?),
            argument(args, "--report").as_deref().map(Path::new),
        ),
        _ => Err(
            "usage: sync_solidcore_cases plan --upstream DIR --commit SHA --short SHA \
                  --date ISO --state FILE [--allow-mass-removal] | apply --results FILE \
                  --state FILE [--report FILE]"
                .to_string(),
        ),
    }
}

fn main() {
    let args: Vec<String> = std::env::args().skip(1).collect();
    if let Err(error) = run(&args) {
        eprintln!("sync_solidcore_cases: {error}");
        std::process::exit(1);
    }
}
