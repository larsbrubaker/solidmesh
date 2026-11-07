//! Top center nose control measure tests: how near the kernel's Top Center Face nose lands to the
//! Onshape control, with run-outs off and on - the kernel-side reading of
//! `bevel_top_center_face_regression_tests.rs` in mattercad-model.
//!
//! The control, `top-center-face-control-nose.txt`, is the control mesh that regression test
//! dumps, in its box frame, cut to the triangles with a corner near the nose. Distances are vertex
//! to nearest control triangle and control vertex to nearest built triangle, both ways, reported
//! per region: the two ridge ends (within 3 of each), the apex (within 2.5), and the rest of the
//! nose. Measured 2026-09-26, worst per region: run-outs off 0.163, run-outs on 0.200. What is left
//! is the fill, not the cut: past s = 2.5 along the lower crease the built part has no vertex off
//! the two crease faces, where the control's blend stands 0.57 wide and 0.12 deep at s = 3. The
//! target is the regression test's chord tolerance, 0.05, in every region.
//!
//! There is no test that the nose is within the chord tolerance of the control with run-outs on:
//! the run-out does not yet land within it, so the measured deviations are printed instead.

use std::collections::{BTreeMap, HashSet};
use std::rc::Rc;

use geometry_processing::bevel::level_line::band_gap_entry_points::BandGapBuildOptions;
use geometry_processing::bevel::level_line::{
    gap_fill, turn_run_outs_on, LevelLineWork, WorkBudget,
};
use vector_math::Vector3;

use crate::end_face_builds::{as_saved, build_chains, default_budget};
use crate::fillet_thin_front_fin::profile_of;
use crate::level_line_fixture::LevelLineFixture;
use crate::mesh_surface_distance::closest_point_on_triangle;
use crate::sweep_mesh_measures;

/// Where the fixture's part sits relative to the regression test's box frame.
const OFFSET: Vector3 = Vector3::new(-20.0, -118.5, 0.0);

const LEFT_END: Vector3 = Vector3::new(5.0, 5.0, 10.0);

const RIGHT_END: Vector3 = Vector3::new(15.0, 5.0, 10.0);

fn apex() -> Vector3 {
    Vector3::new(10.0, 10.0, 10.0 + (10.0 / 3.0))
}

/// The worst any region reads with run-outs off, as measured (0.163), plus rounding.
const REACHED_WITH_RUN_OUTS_OFF: f64 = 0.165;

/// The worst any region reads with run-outs on, as measured (0.200), plus rounding.
const REACHED_WITH_RUN_OUTS_ON: f64 = 0.205;

type Triangle = (Vector3, Vector3, Vector3);

/// Prints both readings; asserts that
/// each builds, and that what is reached today, off (the live path) and on, is not lost - not the
/// target, which is the skipped test.
#[test]
fn measure_the_nose_against_the_control_with_run_outs_off_and_on() {
    let (off, off_mesh) = measure(false);
    let (on, on_mesh) = measure(true);
    println!("run-outs off: {}", said(&off));
    println!("run-outs on:  {}", said(&on));
    assert!(off_mesh);
    assert!(on_mesh);

    for (region, worst) in &off {
        assert!(
            *worst <= REACHED_WITH_RUN_OUTS_OFF,
            "{region}: {}",
            said(&off)
        );
    }

    for (region, worst) in &on {
        assert!(
            *worst <= REACHED_WITH_RUN_OUTS_ON,
            "{region}: {}",
            said(&on)
        );
    }
}

fn said(worst: &[(String, f64)]) -> String {
    worst
        .iter()
        .map(|(region, w)| format!("{region} {w:.3}"))
        .collect::<Vec<_>>()
        .join("; ")
}

/// The worst distance per region and way, and whether the nose built and filled.
fn measure(run_outs: bool) -> (Vec<(String, f64)>, bool) {
    let fixture = Rc::new(LevelLineFixture::load("top-center-face-nose-r1"));
    let filled = {
        // Run-outs are turned on (when asked) round both the build and the fill: the gate rides on
        // the budget in force.
        let budget = default_budget(&fixture, false);
        let _work = LevelLineWork::begin(Some(Rc::clone(&budget)));
        let _run_outs = run_outs.then(turn_run_outs_on);
        let options = BandGapBuildOptions {
            budget: Some(Rc::clone(&budget)),
            ..as_saved(&fixture, fixture.known_room)
        };
        let all: Vec<usize> = (0..fixture.chains.len()).collect();
        let gap = build_chains(&fixture, &all, fixture.size, &options);
        if !gap.built() {
            return (Vec::new(), false);
        }

        let triangle_count = gap.mesh.as_ref().map_or(0, |m| m.borrow().triangle_count());
        let fill_budget = Rc::new(
            WorkBudget::new(WorkBudget::default_limit(triangle_count)).expect("a positive limit"),
        );
        fill_budget.run_outs_on.set(budget.run_outs_on.get());
        gap_fill::fill_with_budget(
            &gap,
            profile_of(&fixture),
            fixture.segments,
            Some(fill_budget),
        )
        .expect("the fill runs")
    };

    if !filled.filled() {
        return (Vec::new(), false);
    }

    let mesh = filled.mesh.as_ref().expect("a filled mesh").borrow();
    let built: Vec<Triangle> = sweep_mesh_measures::kept(&mesh)
        .iter()
        .map(|&(a, b, c)| {
            (
                mesh.position(a) - OFFSET,
                mesh.position(b) - OFFSET,
                mesh.position(c) - OFFSET,
            )
        })
        .collect();
    let control = load_control();

    // Ordered by key, so the report reads the same on every run.
    let mut worst: BTreeMap<String, (f64, Vector3)> = BTreeMap::new();
    let mut take = |from: &[Triangle], to: &[Triangle], way: &str| {
        let mut seen = HashSet::new();
        for vertex in from.iter().flat_map(|t| [t.0, t.1, t.2]) {
            if !seen.insert((vertex.x.to_bits(), vertex.y.to_bits(), vertex.z.to_bits()))
                || !in_the_nose(vertex)
            {
                continue;
            }

            let nearest = to
                .iter()
                .map(|t| (closest_point_on_triangle(vertex, t.0, t.1, t.2) - vertex).length())
                .fold(f64::INFINITY, f64::min);
            let key = format!("{} ({way})", region_of(vertex));
            match worst.get(&key) {
                Some(&(was, _)) if nearest <= was => {}
                _ => {
                    worst.insert(key, (nearest, vertex));
                }
            }
        }
    };

    take(&built, &control, "built to control");
    take(&control, &built, "control to built");
    (
        worst
            .into_iter()
            .map(|(key, (value, at))| (format!("{key} at {at:?}"), value))
            .collect(),
        true,
    )
}

/// The nose and what the corner fill touches around it; the control file holds only triangles
/// near it.
fn in_the_nose(v: Vector3) -> bool {
    v.x >= 3.0 && v.x <= 17.0 && v.y >= 3.0 && v.y <= 12.0 && v.z >= 6.0 && v.z <= 15.0
}

fn region_of(v: Vector3) -> &'static str {
    if (v - LEFT_END).length() <= 3.0 {
        "left ridge end"
    } else if (v - RIGHT_END).length() <= 3.0 {
        "right ridge end"
    } else if (v - apex()).length() <= 2.5 {
        "apex"
    } else {
        "rest of the nose"
    }
}

/// Nine numbers a line, a triangle's three corners.
fn load_control() -> Vec<Triangle> {
    let path = format!(
        "{}/tests/bevel/level_line_fixtures/top-center-face-control-nose.txt",
        env!("CARGO_MANIFEST_DIR")
    );
    let text = std::fs::read_to_string(&path).unwrap_or_else(|e| panic!("{path}: {e}"));
    let mut triangles = Vec::new();
    for line in text.lines() {
        if line.starts_with('#') || line.is_empty() {
            continue;
        }

        let c: Vec<f64> = line
            .split(' ')
            .map(|x| {
                x.parse::<f64>()
                    .unwrap_or_else(|e| panic!("{path}: {x}: {e}"))
            })
            .collect();
        triangles.push((
            Vector3::new(c[0], c[1], c[2]),
            Vector3::new(c[3], c[4], c[5]),
            Vector3::new(c[6], c[7], c[8]),
        ));
    }

    triangles
}
