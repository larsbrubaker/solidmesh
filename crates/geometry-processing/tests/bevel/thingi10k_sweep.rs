//! Port of `LevelLine/Thingi10KSweep.cs` (with `Thingi10KCorpus.cs`, `StlReader.cs`, `SweepPick.cs`,
//! `SweepBuild.cs`, `SweepWatchdog.cs`, `SweepRow.cs`, `SweepReport.cs` and `SweepFailureDumps.cs`,
//! each its own file beside this one): the level line's second scoreboard, the bevel built at the
//! KERNEL level on the local Thingi10K corpus - no app, no .mcx, no pick UI. Seeded random models,
//! feature edges by bend angle, random face rims and feature chains, and sizes taken as fractions of
//! the room the construction itself names.
//!
//! Gated on `THINGI_SWEEP=1` and on the dataset being present, because it is an instrument and not a
//! requirement: it reports, it does not assert geometry. The dataset is READ IN PLACE from
//! `THINGI10K_ROOT` (default `~/Development/rust-apps/Thingi10K`) and nothing is copied into this
//! repository. The knobs are C#'s: `THINGI_SEED`, `THINGI_MAX_FACES`, `THINGI_MODELS`,
//! `THINGI_PICKS`, `THINGI_BEND_DEGREES`, `THINGI_BUILD_SECONDS`, `THINGI_MAX_HUNG` and
//! `THINGI_ONLY` (model ids). The report and the failing rows' kernel-fixture dumps go under
//! `TestResults/` in cargo's target tmp directory, outside the repository.
//!
//! The harness shape is `BevelRobustnessSweep`'s, for the same reasons: a build runs on its own
//! thread under a hard wall-clock watchdog, because a kernel defect shows up as a build that never
//! returns; the hang budget is spent once per PICK, so one wedged pick does not burn the budget at
//! five sizes; rows go to disk as they are made, so an early stop keeps everything already learned;
//! and a self-check at the end requires the rows counted to equal the rows written.
//!
//! A long run needs `--profile measure` (`.config/nextest.toml`), which lets it run for an hour:
//! `THINGI_SWEEP=1 cargo nextest run --workspace --test bevel --profile measure --no-capture
//! thingi10k_sweep`. C#'s `[NotInParallel]` is a `threads-required` override there.

use std::io::Write;
use std::path::{Path, PathBuf};
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

use geometry_processing::bevel::level_line::work_budget::{DEFAULT_FLOOR, DEFAULT_PER_TRIANGLE};

use crate::sweep_model::{sweep_model, Sinks, FRACTIONS};
use crate::sweep_report;
use crate::sweep_row::SweepRow;
use crate::sweep_watchdog;
use crate::thingi10k_corpus;

const RUN_VARIABLE: &str = "THINGI_SWEEP";

/// How long to wait for abandoned builds to finish before ending the run early.
const WAIT_FOR_ABANDONED: Duration = Duration::from_secs(60);

/// The sweep's knobs (C#'s `Number` and `Real` environment reads, with their defaults).
#[derive(Debug, Clone)]
pub struct SweepSettings {
    pub seed: i32,
    pub max_faces: i32,
    pub models: i32,
    pub picks_per_model: i32,
    pub bend_degrees: f64,
    pub build_cap: Duration,
    pub max_hung: i32,
}

impl SweepSettings {
    pub fn from_environment() -> Self {
        SweepSettings {
            seed: number("THINGI_SEED", 1),
            max_faces: number("THINGI_MAX_FACES", 20000),
            models: number("THINGI_MODELS", 200),
            picks_per_model: number("THINGI_PICKS", 6),
            bend_degrees: real("THINGI_BEND_DEGREES", 30.0),
            build_cap: Duration::from_secs_f64(real("THINGI_BUILD_SECONDS", 30.0)),
            max_hung: number("THINGI_MAX_HUNG", 12),
        }
    }
}

fn number(name: &str, fallback: i32) -> i32 {
    match std::env::var(name)
        .ok()
        .and_then(|v| v.trim().parse::<i32>().ok())
    {
        Some(value) if value > 0 => value,
        _ => fallback,
    }
}

fn real(name: &str, fallback: f64) -> f64 {
    match std::env::var(name)
        .ok()
        .and_then(|v| v.trim().parse::<f64>().ok())
    {
        Some(value) if value > 0.0 => value,
        _ => fallback,
    }
}

/// What a finished sweep hands back.
pub struct SweepRun {
    pub rows: Vec<SweepRow>,
    pub open_chains: i32,
    pub ended_early: String,
    pub models: usize,
    pub report_path: PathBuf,
    pub summary: String,
}

/// `yyyyMMdd-HHmmss` of now, in UTC (C# writes local time).
fn stamp() -> String {
    let seconds = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map_or(0, |d| d.as_secs()) as i64;
    let (days, time) = (seconds.div_euclid(86_400), seconds.rem_euclid(86_400));
    // Howard Hinnant's civil-from-days.
    let z = days + 719_468;
    let era = z.div_euclid(146_097);
    let doe = z.rem_euclid(146_097);
    let yoe = (doe - doe / 1460 + doe / 36_524 - doe / 146_096) / 365;
    let doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
    let mp = (5 * doy + 2) / 153;
    let day = doy - (153 * mp + 2) / 5 + 1;
    let month = if mp < 10 { mp + 3 } else { mp - 9 };
    let year = yoe + era * 400 + i64::from(month <= 2);
    format!(
        "{year:04}{month:02}{day:02}-{:02}{:02}{:02}",
        time / 3600,
        time % 3600 / 60,
        time % 60
    )
}

/// The sweep itself: draws the models, sweeps each, writes the report under `results`.
pub fn run_sweep(
    root: &Path,
    metadata: &Path,
    settings: &SweepSettings,
    results: &Path,
) -> std::io::Result<SweepRun> {
    sweep_watchdog::set_build_cap(settings.build_cap);
    sweep_watchdog::forget_abandoned();

    let chosen =
        thingi10k_corpus::chosen(metadata, settings.max_faces, settings.seed, settings.models)
            .map_err(std::io::Error::other)?;

    std::fs::create_dir_all(results)?;
    let stamp = stamp();
    let report_path = results.join(format!("thingi-sweep-{}-{}.txt", settings.seed, stamp));
    let failure_root = results.join("thingi-failures");

    let mut rows: Vec<SweepRow> = Vec::new();
    let mut open_chains = 0;
    let mut row_lines = 0;
    let mut ended_early = String::new();

    let mut log = std::io::BufWriter::new(std::fs::File::create(&report_path)?);
    writeln!(log, "thingi sweep {}: root {}", stamp, root.display())?;
    // What the budget IS matters to every reader of a Hung row: over the cap with work to spare is
    // slow, over the cap with the budget gone is a runaway the kernel is already refusing.
    writeln!(
        log,
        "work budget per call: {DEFAULT_FLOOR} units + {DEFAULT_PER_TRIANGLE} per triangle of the mesh going in"
    )?;
    writeln!(
        log,
        "seed {}, models {} of the clean records with faces <= {}, picks {}, bend {} degrees, build cap {}s, max hung {}",
        settings.seed,
        chosen.len(),
        settings.max_faces,
        settings.picks_per_model,
        settings.bend_degrees,
        settings.build_cap.as_secs_f64(),
        settings.max_hung
    )?;
    let fractions: Vec<String> = FRACTIONS.iter().map(|f| f.to_string()).collect();
    writeln!(
        log,
        "sizes are fractions of the room the build names: {}",
        fractions.join(", ")
    )?;
    writeln!(log)?;
    log.flush()?;

    for record in &chosen {
        let before = rows.len();
        let swept = {
            let mut sinks = Sinks {
                rows: &mut rows,
                log: &mut log,
                failure_root: &failure_root,
                row_lines: &mut row_lines,
            };
            sweep_model(
                record,
                root,
                settings.bend_degrees,
                settings.picks_per_model,
                settings.seed,
                &mut sinks,
            )
        };
        match swept {
            Ok(chains) => open_chains += chains,
            Err(error) => {
                row_lines += sweep_report::write(
                    &mut log,
                    SweepRow::model_aside(record.id, &format!("Threw:{}", exception_name(&error))),
                    &mut rows,
                )?;
            }
        }

        writeln!(log, "-- end {}: {} rows", record.id, rows.len() - before)?;
        log.flush()?;

        if sweep_watchdog::alive() >= settings.max_hung {
            writeln!(
                log,
                "-- {} abandoned builds alive (max {}); waiting up to {}s for them",
                sweep_watchdog::alive(),
                settings.max_hung,
                WAIT_FOR_ABANDONED.as_secs_f64()
            )?;
            let waited = Instant::now();
            while waited.elapsed() < WAIT_FOR_ABANDONED
                && sweep_watchdog::alive() >= settings.max_hung
            {
                std::thread::sleep(Duration::from_millis(500));
            }

            if sweep_watchdog::alive() >= settings.max_hung {
                ended_early = format!(
                    "ended early: {} abandoned builds still spinning",
                    sweep_watchdog::alive()
                );
                writeln!(log, "-- {ended_early}")?;
                break;
            }
        }
    }

    let summary = sweep_report::summary(&rows, open_chains, &ended_early);
    writeln!(log)?;
    write!(log, "{summary}")?;
    log.flush()?;

    // The totals are read from a list and the rows are written to a file; if those two ever
    // disagree the report is lying about what happened, which is worse than any row in it.
    if rows.len() as i32 != row_lines {
        panic!(
            "thingi sweep self-check: {} row lines written but the totals count {}",
            row_lines,
            rows.len()
        );
    }

    Ok(SweepRun {
        rows,
        open_chains,
        ended_early,
        models: chosen.len(),
        report_path,
        summary,
    })
}

fn exception_name(error: &std::io::Error) -> &'static str {
    match error.kind() {
        std::io::ErrorKind::Other => "Exception",
        _ => "IOException",
    }
}

fn default_root() -> PathBuf {
    let home = std::env::var_os("HOME").or_else(|| std::env::var_os("USERPROFILE"));
    PathBuf::from(home.unwrap_or_default())
        .join("Development")
        .join("rust-apps")
        .join("Thingi10K")
}

#[test]
fn sweep_thingi10k() {
    if std::env::var(RUN_VARIABLE).as_deref() != Ok("1") {
        println!("thingi sweep: set {RUN_VARIABLE}=1 to run it");
        return;
    }

    let root = match std::env::var("THINGI10K_ROOT") {
        Ok(root) if !root.is_empty() => PathBuf::from(root),
        _ => default_root(),
    };

    let metadata = root.join("docs").join("data").join("models.json");
    if !metadata.exists() {
        println!(
            "thingi sweep: no dataset at {} (set THINGI10K_ROOT); nothing to do",
            root.display()
        );
        return;
    }

    let settings = SweepSettings::from_environment();
    let results = Path::new(env!("CARGO_TARGET_TMPDIR")).join("TestResults");
    let run = run_sweep(&root, &metadata, &settings, &results)
        .expect("the sweep could not write its report");

    println!(
        "thingi sweep: {} builds over {} models -> {}",
        run.rows.len(),
        run.models,
        run.report_path.display()
    );
    println!("{}", run.summary);
}
