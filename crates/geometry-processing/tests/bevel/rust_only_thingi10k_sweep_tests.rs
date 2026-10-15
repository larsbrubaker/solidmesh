//! Rust-only pins of the Thingi10K sweep harness (`Thingi10KSweep` and its parts), which the
//! ported class runs only when the local corpus is present and `THINGI_SWEEP=1`: a synthetic
//! one-cube corpus is written under cargo's target tmp directory and swept, so the zip reader, the
//! corpus draw, the picks, the watchdog, the rows and the report are exercised on every run.

use std::io::Write;
use std::path::{Path, PathBuf};
use std::sync::{mpsc, Mutex};
use std::time::{Duration, Instant};

use crate::stl_reader;
use crate::sweep_row::SweepRow;
use crate::sweep_watchdog;
use crate::thingi10k_corpus;
use crate::thingi10k_sweep::{run_sweep, SweepSettings};

/// The watchdog's build cap and count of abandoned builds are process-wide; the two tests
/// that move them take turns (nextest gives each test a process of its own).
static WATCHDOG: Mutex<()> = Mutex::new(());

const CUBE_CORNERS: [[f32; 3]; 8] = [
    [0.0, 0.0, 0.0],
    [1.0, 0.0, 0.0],
    [1.0, 1.0, 0.0],
    [0.0, 1.0, 0.0],
    [0.0, 0.0, 1.0],
    [1.0, 0.0, 1.0],
    [1.0, 1.0, 1.0],
    [0.0, 1.0, 1.0],
];

/// Outward-wound, two triangles per face.
const CUBE_FACES: [[usize; 3]; 12] = [
    [0, 3, 2],
    [0, 2, 1],
    [4, 5, 6],
    [4, 6, 7],
    [0, 1, 5],
    [0, 5, 4],
    [3, 7, 6],
    [3, 6, 2],
    [0, 4, 7],
    [0, 7, 3],
    [1, 2, 6],
    [1, 6, 5],
];

fn binary_cube() -> Vec<u8> {
    let mut bytes = vec![0u8; 80];
    bytes.extend_from_slice(&(CUBE_FACES.len() as u32).to_le_bytes());
    for face in CUBE_FACES {
        bytes.extend_from_slice(&[0u8; 12]);
        for corner in face {
            for value in CUBE_CORNERS[corner] {
                bytes.extend_from_slice(&value.to_le_bytes());
            }
        }

        bytes.extend_from_slice(&[0u8; 2]);
    }

    bytes
}

fn zipped(name: &str, bytes: &[u8]) -> Vec<u8> {
    let mut archive = zip::ZipWriter::new(std::io::Cursor::new(Vec::new()));
    archive
        .start_file(name, zip::write::SimpleFileOptions::default())
        .unwrap();
    archive.write_all(bytes).unwrap();
    archive.finish().unwrap().into_inner()
}

/// A corpus of one clean cube (id 7, shard 1) and one record the metadata itself calls unclean.
fn synthetic_corpus(name: &str) -> PathBuf {
    let root = Path::new(env!("CARGO_TARGET_TMPDIR")).join(name);
    let _ = std::fs::remove_dir_all(&root);
    std::fs::create_dir_all(root.join("docs/data")).unwrap();
    std::fs::create_dir_all(root.join("meshes/Thingi10K-meshes-1/meshes")).unwrap();
    std::fs::write(
        root.join("docs/data/models.json"),
        r#"[
          {"closed":true,"degenerate_faces":false,"duplicated_faces":false,"edge_manifold":true,
           "faces":12,"format":"stl","id":7,"pwn":true,"repo":1,"single_component":true,
           "vertex_manifold":true},
          {"closed":false,"degenerate_faces":false,"duplicated_faces":false,"edge_manifold":true,
           "faces":12,"format":"stl","id":8,"pwn":true,"repo":1,"single_component":true,
           "vertex_manifold":true},
          {"closed":true,"degenerate_faces":false,"duplicated_faces":false,"edge_manifold":true,
           "faces":12,"format":"obj","id":9,"pwn":true,"repo":1,"single_component":true,
           "vertex_manifold":true}
        ]"#,
    )
    .unwrap();
    std::fs::write(
        root.join("meshes/Thingi10K-meshes-1/meshes/7.stl.zip"),
        zipped("7.stl", &binary_cube()),
    )
    .unwrap();
    root
}

#[test]
fn rust_only_the_zip_reader_welds_a_cube_by_exact_position() {
    let root = synthetic_corpus("thingi_zip_reader");
    let (positions, indices) =
        stl_reader::read_zip(&root.join("meshes/Thingi10K-meshes-1/meshes/7.stl.zip")).unwrap();

    assert_eq!(positions.len(), 8);
    assert_eq!(indices.len(), 12);
    assert!(crate::sweep_mesh_measures::is_closed_and_consistent(
        &positions, &indices
    ));
    assert!((crate::sweep_mesh_measures::signed_volume(&positions, &indices) - 1.0).abs() < 1e-12);
}

#[test]
fn rust_only_the_corpus_draws_only_clean_stl_records() {
    let root = synthetic_corpus("thingi_corpus_draw");
    let metadata = root.join("docs/data/models.json");

    let chosen = thingi10k_corpus::chosen(&metadata, 20000, 1, 200).unwrap();
    assert_eq!(
        chosen.len(),
        1,
        "the unclosed record and the .obj one are not drawn"
    );
    assert_eq!((chosen[0].id, chosen[0].repo, chosen[0].faces), (7, 1, 12));
    assert!(
        root.join("meshes/Thingi10K-meshes-1/meshes/7.stl.zip")
            == thingi10k_corpus::mesh_path(&root, &chosen[0])
    );

    assert!(
        thingi10k_corpus::chosen(&metadata, 11, 1, 200)
            .unwrap()
            .is_empty(),
        "a model over the face cap is not drawn"
    );
}

#[test]
fn rust_only_the_watchdog_names_a_throw_a_panic_and_a_hang() {
    let _turn = WATCHDOG.lock().unwrap_or_else(|e| e.into_inner());
    sweep_watchdog::set_build_cap(Duration::from_millis(300));

    let threw = sweep_watchdog::run(1000, |_| -> Result<(), _> {
        Err(sweep_watchdog::Thrown::new("ArgumentException", "no"))
    });
    assert_eq!(threw.outcome.as_deref(), Some("Threw:ArgumentException"));
    assert_eq!(threw.detail.as_deref(), Some("no"));

    let panicked = sweep_watchdog::run(1000, |_| -> Result<(), _> { panic!("a defect") });
    assert_eq!(panicked.outcome.as_deref(), Some("Threw:Panic"));
    assert_eq!(panicked.detail.as_deref(), Some("a defect"));

    // The builds above answered, so none of them is counted, even if its thread has not exited.
    let before = sweep_watchdog::alive();
    assert_eq!(
        before, 0,
        "a build that answered is not counted as abandoned"
    );

    // A build that never charges the budget cannot be cancelled; this one waits until the test
    // lets it go, so it outlasts the watchdog's grace however the machine schedules it: it is
    // abandoned and still counted.
    let (release, held) = mpsc::channel::<()>();
    let spinner = sweep_watchdog::run(1000, move |_| -> Result<(), _> {
        let _ = held.recv();
        Ok(())
    });
    assert_eq!(spinner.outcome.as_deref(), Some("Hung"));
    assert!(sweep_watchdog::alive() > before);

    // Once it ends on its own it leaves the count; its thread is never joined, so wait for that.
    drop(release);
    let deadline = Instant::now() + Duration::from_secs(10);
    while sweep_watchdog::alive() != before && Instant::now() < deadline {
        std::thread::sleep(Duration::from_millis(10));
    }
    assert_eq!(
        sweep_watchdog::alive(),
        before,
        "the abandoned build ended and is no longer counted"
    );

    sweep_watchdog::set_build_cap(Duration::from_secs(30));
}

#[test]
fn rust_only_a_sweep_of_a_synthetic_corpus_writes_a_report_that_counts_its_rows() {
    let _turn = WATCHDOG.lock().unwrap_or_else(|e| e.into_inner());
    let root = synthetic_corpus("thingi_sweep");
    let settings = SweepSettings {
        seed: 1,
        max_faces: 20000,
        models: 5,
        picks_per_model: 4,
        bend_degrees: 30.0,
        build_cap: Duration::from_secs(30),
        max_hung: 12,
    };
    let results = root.join("TestResults");
    let run = run_sweep(
        &root,
        &root.join("docs/data/models.json"),
        &settings,
        &results,
    )
    .unwrap();

    assert_eq!(run.models, 1);
    assert!(!run.rows.is_empty());
    assert!(run.ended_early.is_empty());
    let report = std::fs::read_to_string(&run.report_path).unwrap();
    assert!(report.contains("TOTALS:"));
    assert_eq!(
        report.lines().filter(|l| l.starts_with("7 pick ")).count(),
        run.rows.len(),
        "every row is one line of the report"
    );
    assert!(
        run.rows
            .iter()
            .all(|r: &SweepRow| !r.outcome.starts_with("Threw:") && r.outcome != "Hung"),
        "{}",
        run.summary
    );
    assert!(
        run.rows.iter().any(|r| r.outcome == "BuiltClosed"),
        "a cube's edges build: {}",
        run.summary
    );
    assert!(run.open_chains >= 0);
}
