//! Real parts' fillets and chamfers, replayed. Each file in `tests/bevel/part_requests` is one
//! [`BevelPartRequest`] a MatterCAD design really made, frozen with no document around it, and
//! what the kernel answered: the mesh to the last bit, each group's row and the part's advisories,
//! as codes. Every one is asked again here and has to answer the same on every platform.
//!
//! The files are written by MatterCAD's request recorder (`bevel_request_recorder_tests` in its
//! `mattercad-model` tests, whose header says how to refresh them). A change that moves one on
//! purpose refreshes the file and says why in its commit. A refused row's offered size is left
//! out: its search runs against a clock.
//!
//! Lars Brubaker made the designs these requests came from and dedicates the files in
//! `tests/bevel/part_requests` to the public domain under CC0 1.0
//! (<https://creativecommons.org/publicdomain/zero/1.0/>).

use std::fs;
use std::path::Path;
use std::sync::Arc;

use serde::Deserialize;
use solidmesh::bevel::part::{bevel_part, BevelPartOutcome, BevelPartRequest, BevelRouting};
use solidmesh::bevel::BevelAdvisory;
use solidmesh::topology::BevelEdgeGroup;
use solidmesh_math::Matrix4X4;
use solidmesh_mesh::{CancelToken, Mesh};

use super::part_bevel_platform_bits_tests::digest;

#[derive(Debug, PartialEq, Deserialize)]
struct ExpectedMesh {
    vertices: usize,
    faces: usize,
    digest: String,
}

#[derive(Debug, PartialEq, Deserialize)]
struct ExpectedRow {
    failure: Option<String>,
    warnings: Vec<String>,
    loop_failures: usize,
}

#[derive(Deserialize)]
struct RecordedRequest {
    design: String,
    vertices: Vec<f64>,
    faces: Vec<i32>,
    source_to_operation: Vec<f64>,
    groups: Vec<BevelEdgeGroup>,
    size: f64,
    segments: i32,
    profile_is_round: bool,
    bend_threshold_degrees: f64,
    routing: [bool; 4],
    expected_mesh: Option<ExpectedMesh>,
    expected_rows: Vec<Option<ExpectedRow>>,
    expected_advisories: Vec<String>,
}

impl RecordedRequest {
    fn request(&self) -> BevelPartRequest {
        let [level_line, sections, corner_hole_fill, flat_ends] = self.routing;
        BevelPartRequest {
            source_to_operation: Matrix4X4::from_slice(&self.source_to_operation),
            routing: BevelRouting {
                level_line,
                sections,
                corner_hole_fill,
                flat_ends,
            },
            ..BevelPartRequest::new(
                Some(Arc::new(Mesh::from_arrays(&self.vertices, &self.faces))),
                self.groups.clone(),
                self.size,
                self.segments,
                self.profile_is_round,
                self.bend_threshold_degrees,
            )
        }
    }

    /// What differs between this recording and `outcome`, in words; `None` when nothing does.
    fn difference(&self, outcome: &BevelPartOutcome) -> Option<String> {
        let mesh = outcome.mesh.as_ref().map(|mesh| ExpectedMesh {
            vertices: mesh.vertices.len(),
            faces: mesh.faces.len(),
            digest: digest(mesh),
        });
        if mesh != self.expected_mesh {
            return Some(format!(
                "the mesh moved: {mesh:?}, recorded {:?}",
                self.expected_mesh
            ));
        }

        let rows: Vec<Option<ExpectedRow>> = outcome
            .group_results
            .iter()
            .map(|row| {
                row.as_ref().map(|row| ExpectedRow {
                    failure: row.failure.as_ref().map(|f| f.code().to_string()),
                    warnings: codes(&row.warnings),
                    loop_failures: row.loop_failures.as_ref().map_or(0, Vec::len),
                })
            })
            .collect();
        if rows != self.expected_rows {
            return Some(format!(
                "the rows moved: {rows:?}, recorded {:?}",
                self.expected_rows
            ));
        }

        let advisories = codes(outcome.advisories.as_deref().unwrap_or_default());
        if advisories != self.expected_advisories {
            return Some(format!(
                "the advisories moved: {advisories:?}, recorded {:?}",
                self.expected_advisories
            ));
        }
        None
    }
}

fn codes(advisories: &[BevelAdvisory]) -> Vec<String> {
    advisories.iter().map(|a| a.code().to_string()).collect()
}

/// Every recording's path, sorted, so a run reads them in one order.
fn recordings() -> Vec<std::path::PathBuf> {
    let directory = Path::new(env!("CARGO_MANIFEST_DIR")).join("tests/bevel/part_requests");
    let mut paths: Vec<_> = fs::read_dir(&directory)
        .unwrap_or_else(|error| panic!("reading {directory:?}: {error}"))
        .map(|entry| entry.expect("a directory entry").path())
        .collect();
    paths.sort();
    paths
}

#[test]
fn every_recorded_part_answers_as_it_was_recorded() {
    let paths = recordings();
    assert!(
        paths.len() > 20,
        "expected the recorded corpus, found {}",
        paths.len()
    );

    let mut moved = Vec::new();
    for path in &paths {
        let text =
            fs::read_to_string(path).unwrap_or_else(|error| panic!("reading {path:?}: {error}"));
        let recorded: RecordedRequest =
            serde_json::from_str(&text).unwrap_or_else(|error| panic!("{path:?}: {error}"));

        let outcome = bevel_part(recorded.request(), &CancelToken::new(), None)
            .unwrap_or_else(|error| panic!("{path:?} ({}): {error}", recorded.design));
        if let Some(difference) = recorded.difference(&outcome) {
            let name = path.file_name().unwrap_or_default().to_string_lossy();
            moved.push(format!("{name} ({}): {difference}", recorded.design));
        }
    }

    assert!(
        moved.is_empty(),
        "{} of {} recorded parts answered differently:\n{}",
        moved.len(),
        paths.len(),
        moved.join("\n")
    );
}
