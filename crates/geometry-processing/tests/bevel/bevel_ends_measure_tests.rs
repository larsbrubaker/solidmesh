//! Flat ends, measured before they were built - every chain END a kernel fixture reaches, which
//! construction closes it today, and the shape of its hole. Gated on `BEVEL_ENDS=1`; prints a table
//! and, when `BEVEL_ENDS_OUT` names a file, writes it there. Changes nothing the build does:
//! `BEVEL_ENDS=1 cargo nextest run --workspace --test bevel --profile measure --no-capture
//! bevel_ends_measure_tests`. It runs alone (a `threads-required` override in
//! `.config/nextest.toml`); its runs can take minutes, hence the `measure` profile.
//!
//! Per end: the construction (`zip` = the corner cap's zip between the strip's last row and the end
//! face's run; `fade`, `dying`, `hull cap`, `strip cap` = the other end fills' watches), the
//! boundary's vertex counts, and three lengths in units of the size r: `runFlat` (the end-face
//! run's largest distance from its own best plane), `rowOff` (the last row's largest distance from
//! that plane) and `st` (the row's two ends' distances from the end vertex V). A fade-type hole is
//! read as a loop: its edge count and its largest distance from its best plane (`flat`).
//!
//! The fixture folder also holds `top-center-face-control-nose.txt`, a control mesh in another
//! format. The sweep leaves it out: read as a fixture it would fail on its first line, and a
//! measurement that fails before it measures anything reads nothing.

use std::rc::Rc;
use std::sync::mpsc;
use std::time::Duration;

use solidmesh::bevel::level_line::band_gap_entry_points::{self, BandGapBuildOptions};
use solidmesh::bevel::level_line::{
    band_gap_in_turn, corner_cap, dying_crease_fill, gap_fill, hull_clip, open_gap_fill,
    strip_clip, BandGapResult, BandProfile, GapFillRefusal, GapFillResult, LevelLineMesh,
};
use solidmesh_math::Vector3;

use crate::end_face_builds::{as_saved, feature_of};
use crate::fillet_thin_front_fin::profile_of;
use crate::level_line_feature_edges::feature_edges_over;
use crate::level_line_fixture::{self, LevelLineFixture};
use crate::{crease_dies_out_fixtures, oblique_end_face_tests, open_chain_fixtures};

const RUN_VARIABLE: &str = "BEVEL_ENDS";

const PER_FIXTURE: Duration = Duration::from_secs(90);

/// The fixtures the measurement reads, by name, in ordinal order: every fixture but Many Holes'
/// 25-chain rim (all closed loops, no end, and it outruns the per-fixture time box).
fn fixture_names() -> Vec<String> {
    level_line_fixture::fixture_names()
        .into_iter()
        .filter(|name| !name.starts_with("many-holes-facerim"))
        .collect()
}

/// Every fixture the ends measurement reads loads as a level-line fixture: the control mesh that
/// would stop it is left out.
#[test]
fn rust_only_every_fixture_the_ends_measurement_reads_loads() {
    let names = fixture_names();
    assert!(names.len() > 50, "{}", names.len());
    for name in names {
        let fixture = LevelLineFixture::load(&name);
        assert!(!fixture.positions.is_empty(), "{name}");
    }
}

/// Measure every end on every kernel fixture.
#[test]
fn measure_every_end_on_every_kernel_fixture() {
    if std::env::var(RUN_VARIABLE).as_deref() != Ok("1") {
        println!("bevel ends: set {RUN_VARIABLE}=1 to run it");
        return;
    }

    let mut table =
        String::from("case | fill | ends (construction: boundary, shape in units of r)\n");
    type Run = Box<dyn FnOnce() -> String + Send>;
    let mut runs: Vec<(String, Run)> = Vec::new();
    for name in fixture_names() {
        let fixture_name = name.clone();
        runs.push((name, Box::new(move || measure(&fixture_name, -1))));
    }

    // EndRayReachTests' chain-alone case: Inner Right Sides and Center's concave chains built one at
    // a time.
    for chain in [0, 2] {
        runs.push((
            format!("inner-right-sides chain {chain} alone"),
            Box::new(move || measure("inner-right-sides-and-center-op0", chain)),
        ));
    }

    // Code-built ends the dumps do not reach: the fade (a cylinder's top-rim run, the crease
    // carrying on past both ends) and ObliqueEndFaceTests' sheared prism (a top edge ending on two
    // oblique end faces).
    for edges in [1, 3] {
        for size in [0.25, 1.0] {
            runs.push((
                format!("cylinder rim fade {edges} edges r{size}"),
                Box::new(move || measure_fade(edges, size)),
            ));
        }
    }

    for fraction in [0.1, 0.5] {
        runs.push((
            format!("sheared prism top edge {fraction} room"),
            Box::new(move || measure_sheared_prism(fraction)),
        ));
    }

    // CreaseDiesOutTests' swollen ridge: a crease that dies out at both ends (one loop round the
    // chain).
    for size in [0.1, 0.5] {
        runs.push((
            format!("swollen ridge dying crease r{size}"),
            Box::new(move || {
                let (positions, indices) = crease_dies_out_fixtures::swollen_ridge_block();
                let gap = crease_dies_out_fixtures::build(&positions, &indices, size);
                watched(&gap, size, || gap_fill::fill(&gap, BandProfile::Fillet, 8))
            }),
        ));
    }

    for (name, run) in runs {
        // A run past its time box is reported and left to finish on its own thread, so one slow
        // fixture cannot hold up the rest.
        let (sender, receiver) = mpsc::channel();
        std::thread::spawn(move || {
            let _ = sender.send(run());
        });
        let line = receiver
            .recv_timeout(PER_FIXTURE)
            .unwrap_or_else(|_| format!("timed out after {} s", PER_FIXTURE.as_secs()));
        table.push_str(&format!("{name} | {line}\n"));
        println!("{name} | {line}");
    }

    if let Ok(output) = std::env::var(format!("{RUN_VARIABLE}_OUT")) {
        if !output.is_empty() {
            std::fs::write(&output, &table).expect("the table writes");
        }
    }
}

/// The fixture's whole pick (`alone` < 0) or one chain of it.
fn measure(name: &str, alone: i32) -> String {
    let fixture = Rc::new(LevelLineFixture::load(name));
    let (chains, closed) = if alone < 0 {
        (fixture.chains.clone(), fixture.closed.clone())
    } else {
        (
            vec![fixture.chains[alone as usize].clone()],
            vec![fixture.closed[alone as usize]],
        )
    };
    let known_room = if alone < 0 { fixture.known_room } else { None };
    let gap = match band_gap_entry_points::build_pick(
        &fixture.positions,
        &fixture.indices,
        &chains,
        &closed,
        Some(feature_of(&fixture)),
        fixture.size,
        profile_of(&fixture),
        &as_saved(&fixture, known_room),
    ) {
        Ok(gap) => gap,
        Err(e) => return format!("threw {e:?} | -"),
    };

    watched(&gap, fixture.size, || {
        if alone < 0 && band_gap_in_turn::takes_open_chains_in_turn(Some(&gap), &closed) {
            band_gap_in_turn::build_open_chains_in_turn(
                &fixture.positions,
                &fixture.indices,
                &chains,
                Some(feature_of(&fixture)),
                fixture.size,
                profile_of(&fixture),
                fixture.segments,
                Some(gap.room),
                1e-9,
                fixture.seam_epsilon,
                fixture.sharp_turn_degrees,
                None,
                None,
            )
            .map_err(|e| format!("{e:?}"))
        } else {
            gap_fill::fill(&gap, profile_of(&fixture), fixture.segments)
                .map_err(|e| format!("{e:?}"))
        }
    })
}

fn measure_fade(edges: i32, size: f64) -> String {
    let (positions, indices) = open_chain_fixtures::cylinder();
    let gap = band_gap_entry_points::build(
        &positions,
        &indices,
        &open_chain_fixtures::top_rim_run(edges),
        false,
        Some(feature_edges_over(
            &positions,
            &indices,
            open_chain_fixtures::FOLD_DEGREES,
        )),
        size,
        BandProfile::Fillet,
        &BandGapBuildOptions::default(),
    )
    .expect("a valid pick");
    watched(&gap, size, || gap_fill::fill(&gap, BandProfile::Fillet, 8))
}

fn measure_sheared_prism(fraction: f64) -> String {
    let size = fraction * oblique_end_face_tests::room_of();
    let positions = oblique_end_face_tests::positions(None);
    let indices = oblique_end_face_tests::indices();
    let gap = band_gap_entry_points::build(
        &positions,
        &indices,
        &oblique_end_face_tests::chain(),
        false,
        Some(feature_edges_over(&positions, &indices, 20.0)),
        size,
        BandProfile::Fillet,
        &BandGapBuildOptions {
            sharp_turn_degrees: 20.0,
            ..BandGapBuildOptions::default()
        },
    )
    .expect("a valid pick");
    watched(&gap, size, || gap_fill::fill(&gap, BandProfile::Fillet, 8))
}

/// Runs one fill with every end watch on and reads each end hole it hands over.
fn watched<E: std::fmt::Debug, F: FnOnce() -> Result<GapFillResult, E>>(
    gap: &BandGapResult,
    r: f64,
    build: F,
) -> String {
    use std::cell::RefCell;

    if !gap.built() {
        return format!("band {:?} | -", gap.refusal);
    }

    let ends: Rc<RefCell<Vec<String>>> = Rc::new(RefCell::new(Vec::new()));
    type Loops = Rc<RefCell<Vec<(&'static str, Vec<(i32, i32)>)>>>;
    let loops: Loops = Rc::new(RefCell::new(Vec::new()));
    let watch = |kind: &'static str| {
        let loops = Rc::clone(&loops);
        Rc::new(move |open: &[(i32, i32)]| loops.borrow_mut().push((kind, open.to_vec())))
    };
    {
        let ends = Rc::clone(&ends);
        corner_cap::WATCH_END_HOLE_AT.with(|w| {
            *w.borrow_mut() = Some(Rc::new(
                move |mesh: &LevelLineMesh, row: &[i32], run: &[i32], v: Vector3| {
                    ends.borrow_mut().push(zip_shape(mesh, row, run, v, r))
                },
            ))
        });
    }
    open_gap_fill::WATCH_FADE_HOLE.with(|w| *w.borrow_mut() = Some(watch("fade")));
    dying_crease_fill::WATCH_DYING_HOLE.with(|w| *w.borrow_mut() = Some(watch("dying")));
    hull_clip::WATCH_CAP_HOLE.with(|w| *w.borrow_mut() = Some(watch("hull cap")));
    strip_clip::WATCH_CAP_HOLE.with(|w| *w.borrow_mut() = Some(watch("strip cap")));

    let mut result = gap.mesh.clone();
    let fill = match build() {
        Ok(filled) => {
            if filled.mesh.is_some() {
                result = filled.mesh.clone();
            }

            if filled.refusal == GapFillRefusal::None {
                "builds".to_string()
            } else {
                format!("{:?}", filled.refusal)
            }
        }
        Err(e) => format!("threw {e:?}"),
    };

    // The watches go off whatever the fill did, so the next fill runs unwatched.
    corner_cap::WATCH_END_HOLE_AT.with(|w| *w.borrow_mut() = None);
    open_gap_fill::WATCH_FADE_HOLE.with(|w| *w.borrow_mut() = None);
    dying_crease_fill::WATCH_DYING_HOLE.with(|w| *w.borrow_mut() = None);
    hull_clip::WATCH_CAP_HOLE.with(|w| *w.borrow_mut() = None);
    strip_clip::WATCH_CAP_HOLE.with(|w| *w.borrow_mut() = None);

    let mut ends = ends.borrow().clone();
    if let Some(mesh) = &result {
        let mesh = mesh.borrow();
        for (kind, edges) in loops.borrow().iter() {
            ends.push(format!("{kind}: {}", loop_shape(&mesh, edges, r)));
        }
    }

    format!(
        "{fill} | {}",
        if ends.is_empty() {
            "no end hole".to_string()
        } else {
            ends.join("; ")
        }
    )
}

/// The zipped end hole - row and run from S to T round the end vertex V.
fn zip_shape(mesh: &LevelLineMesh, row: &[i32], run: &[i32], v: Vector3, r: f64) -> String {
    let run_points: Vec<Vector3> = run.iter().map(|&i| mesh.position(i)).collect();
    let mut with_v = run_points.clone();
    with_v.push(v);
    let (origin, normal) = best_plane(&with_v);
    let run_flat = run_points
        .iter()
        .map(|&p| (p - origin).dot(normal).abs())
        .fold(f64::NEG_INFINITY, f64::max)
        / r;
    let row_off = row
        .iter()
        .map(|&i| (mesh.position(i) - origin).dot(normal).abs())
        .fold(f64::NEG_INFINITY, f64::max)
        / r;
    let s = (mesh.position(row[0]) - v).length() / r;
    let t = (mesh.position(row[row.len() - 1]) - v).length() / r;
    let through_v = run
        .iter()
        .skip(1)
        .take(run.len().saturating_sub(2))
        .any(|&i| (mesh.position(i) - v).length() < 1e-9);
    format!(
        "zip: row {} run {}{}, runFlat {run_flat:.3}, rowOff {row_off:.3}, st {s:.2}/{t:.2}",
        row.len(),
        run.len(),
        if through_v { " (run through V)" } else { "" }
    )
}

/// A fade-type hole read as a loop of directed edges - edge count and flatness.
fn loop_shape(mesh: &LevelLineMesh, edges: &[(i32, i32)], r: f64) -> String {
    let mut vertices = Vec::new();
    for &(a, b) in edges {
        for v in [a, b] {
            if !vertices.contains(&v) && v < mesh.vertex_count() {
                vertices.push(v);
            }
        }
    }

    if vertices.len() < 3 {
        return format!("{} edges", edges.len());
    }

    let points: Vec<Vector3> = vertices.iter().map(|&v| mesh.position(v)).collect();
    let (origin, normal) = best_plane(&points);
    let flat = points
        .iter()
        .map(|&p| (p - origin).dot(normal).abs())
        .fold(f64::NEG_INFINITY, f64::max)
        / r;
    format!("{} edges, flat {flat:.3}", edges.len())
}

/// A plane through the points' centroid with the normal of the largest-area
/// triangle among them - exact for a planar set, and a fair reading of how far a non-planar one
/// departs.
pub(crate) fn best_plane(points: &[Vector3]) -> (Vector3, Vector3) {
    let mut origin = Vector3::ZERO;
    for &p in points {
        origin += p;
    }

    origin /= points.len() as f64;
    let mut best = Vector3::UNIT_Z;
    let mut best_area = 0.0;
    let step = (points.len() / 24).max(1);
    for i in (0..points.len()).step_by(step) {
        for j in (i + 1..points.len()).step_by(step) {
            for k in (j + 1..points.len()).step_by(step) {
                let n = (points[j] - points[i]).cross(points[k] - points[i]);
                if n.length_squared() > best_area {
                    best_area = n.length_squared();
                    best = n;
                }
            }
        }
    }

    (
        origin,
        if best_area > 0.0 {
            best.get_normal()
        } else {
            Vector3::UNIT_Z
        },
    )
}
