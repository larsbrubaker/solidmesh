//! What a band build of a pick of MANY loops costs PER LOOP, on the n x n hole plate of
//! `grid_hole_plate_fixture` at 5, 17 and 37 chains, and what it costs as the face's triangles grow
//! in sizes across. The cost rows assert only that the pick builds - the numbers are the point,
//! printed and appended to `LEVELLINE_COST_LOG` when it is set; the digest rows pin the built mesh
//! to the bit. The rows share the process-wide meter, so a nextest override in
//! `.config/nextest.toml` keeps them apart. Parameter rows are loops over the same-named test.

use std::cell::RefCell;
use std::rc::Rc;

use indexmap::IndexMap;
use solidmesh::bevel::level_line::band_gap_entry_points::{self, BandGapBuildOptions};
use solidmesh::bevel::level_line::{
    BandProfile, FieldCounters, LevelLineProgress, LevelLineWorkPhase, WorkBudget,
};
use solidmesh_mesh::CancelToken;

use crate::convex_cell_bound_tests::digest;
use crate::grid_hole_plate_fixture;

/// What one build of the pick cost, with enough of the counters to say where
/// the work went.
struct Measurement {
    loops: i32,
    /// How many sizes across a face triangle is - the second cost question.
    span_over_size: f64,
    triangles: i32,
    spent: i64,
    chain_evaluates: i64,
    pick_calls: i64,
    /// Point queries of the PICK's field - each one asks some of the member chains.
    pick_queries: i64,
    phases: Vec<(LevelLineWorkPhase, i64)>,
    mesh_digest: String,
    /// The per-rule meter of this build.
    rule_meter: String,
    /// The same meter split by how many sizes from the line the piece asked about stood.
    reach_meter: String,
    /// What a "no search needed" test would retire.
    no_search_meter: String,
    /// One line per refinement call: which group, which chains, how many candidates.
    refinements: String,
}

impl Measurement {
    fn per_loop(&self) -> f64 {
        self.spent as f64 / self.loops as f64
    }

    /// Member chains read per point query of the pick - the number the fix is about.
    fn chains_per_query(&self) -> f64 {
        if self.pick_queries == 0 {
            0.0
        } else {
            self.chain_evaluates as f64 / self.pick_queries as f64
        }
    }
}

impl std::fmt::Display for Measurement {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let phases: Vec<String> = self
            .phases
            .iter()
            .map(|(phase, spent)| format!("{phase:?} {spent}"))
            .collect();
        write!(
            f,
            "loops {}, span/size {:.1}, triangles {}, spent {}, per loop {:.0}, chain evaluates {}, \
             pick calls {}, pick queries {}, chains per query {:.2} | {} | mesh {}",
            self.loops,
            self.span_over_size,
            self.triangles,
            self.spent,
            self.per_loop(),
            self.chain_evaluates,
            self.pick_calls,
            self.pick_queries,
            self.chains_per_query(),
            phases.join(", "),
            self.mesh_digest
        )
    }
}

/// Switches `FieldCounters`' meter off when dropped (even on a panic): the meter outlives the test,
/// so a build that throws must not leave it on for whatever runs next.
struct MeterOff;

impl Drop for MeterOff {
    fn drop(&mut self) {
        FieldCounters::set_meter_on(false);
    }
}

/// One whole-rim build of the n x n hole plate, with the work budget's phase reports
/// gathered so the spend can be split between the room, the refinement, the cut and the fill.
fn measure(holes: i32, span: f64, size: f64) -> Measurement {
    let (positions, indices, chains) = grid_hole_plate_fixture::pick_with_span(holes, span);

    // A phase change ALWAYS reports (the throttle is only between), so attributing each report's
    // delta to the phase the previous report named splits the spend at the boundaries exactly.
    let per_phase: Rc<RefCell<IndexMap<LevelLineWorkPhase, i64>>> =
        Rc::new(RefCell::new(IndexMap::new()));
    let last: Rc<RefCell<(LevelLineWorkPhase, i64)>> =
        Rc::new(RefCell::new((LevelLineWorkPhase::None, 0)));
    let note = {
        let per_phase = Rc::clone(&per_phase);
        let last = Rc::clone(&last);
        move |progress: LevelLineProgress| {
            let mut last = last.borrow_mut();
            *per_phase.borrow_mut().entry(last.0).or_insert(0) += progress.work_spent - last.1;
            *last = (progress.phase, progress.work_spent);
        }
    };

    let budget = Rc::new(
        WorkBudget::with_cancellation(
            500 * WorkBudget::default_limit(indices.len() as i32),
            CancelToken::default(),
            Some(Rc::new(note)),
        )
        .expect("a positive limit"),
    );

    FieldCounters::reset();
    FieldCounters::set_meter_on(true);
    let (built, meter, reach, no_search, refinements) = {
        let _off = MeterOff;
        let built = band_gap_entry_points::build_pick(
            &positions,
            &indices,
            &chains,
            &vec![true; chains.len()],
            Some(grid_hole_plate_fixture::feature_edges(&positions, &indices)),
            size,
            BandProfile::Chamfer,
            &BandGapBuildOptions {
                chord_tolerance: 0.0,
                snap: 1e-9,
                known_room: None,
                budget: Some(Rc::clone(&budget)),
                ..BandGapBuildOptions::default()
            },
        )
        .expect("a valid pick");

        (
            built,
            FieldCounters::rule_report(),
            FieldCounters::reach_report(),
            FieldCounters::no_search_needed_report(),
            FieldCounters::refinement_report(),
        )
    };

    assert!(
        built.built(),
        "the {holes} x {holes} plate at span {span} size {size} refused by {:?} after {} units of the \
         build's own allowance (a pinch ladder spends on an allowance of its own, not counted here)",
        built.refusal,
        budget.spent.get()
    );

    let (chain_evaluates, pick_calls, pick_queries) = FieldCounters::with(|t| {
        (
            t.evaluate,
            t.max_tri + t.max_along + t.min_tri + t.min_along,
            t.pick_evaluate,
        )
    });
    let mut phases: Vec<(LevelLineWorkPhase, i64)> = per_phase
        .borrow()
        .iter()
        .filter(|(phase, _)| **phase != LevelLineWorkPhase::None)
        .map(|(&phase, &spent)| (phase, spent))
        .collect();
    // A stable sort: equal spends keep their first-reported order.
    phases.sort_by_key(|p| std::cmp::Reverse(p.1));

    let mesh = built.mesh.as_ref().expect("a built mesh").borrow();
    Measurement {
        loops: chains.len() as i32,
        span_over_size: span / size,
        triangles: indices.len() as i32,
        spent: budget.spent.get(),
        chain_evaluates,
        pick_calls,
        pick_queries,
        phases,
        mesh_digest: digest(&mesh),
        rule_meter: meter,
        reach_meter: reach,
        no_search_meter: no_search,
        refinements,
    }
}

/// Prints the line and appends it to `LEVELLINE_COST_LOG` when one is asked for.
fn report(line: &str) {
    println!("{line}");
    if let Ok(log) = std::env::var("LEVELLINE_COST_LOG") {
        if !log.is_empty() {
            use std::io::Write;
            let mut file = std::fs::OpenOptions::new()
                .create(true)
                .append(true)
                .open(&log)
                .expect("the cost log opens");
            writeln!(file, "{line}").expect("the cost log takes the line");
        }
    }
}

/// The cost table, re-measured. It asserts only that the pick builds.
#[test]
fn the_cost_of_a_whole_rim_pick() {
    for holes in [2, 4, 6] {
        let measured = measure(
            holes,
            grid_hole_plate_fixture::DEFAULT_SPAN,
            grid_hole_plate_fixture::SIZE,
        );
        report(&format!(
            "grid {holes}x{holes}: {measured}\n{}\nby distance from the line:\n{}\nno search needed:\n{}\nrefinements:\n{}",
            measured.rule_meter, measured.reach_meter, measured.no_search_meter, measured.refinements
        ));

        assert_eq!(measured.loops, grid_hole_plate_fixture::chain_count(holes));
    }
}

/// THE SECOND COST QUESTION: what a build costs as the face's triangles grow in SIZES ACROSS, with
/// the pick held at 17 loops, with the per-rule meter beside it to say which rule pays it.
#[test]
fn the_cost_against_span_over_size() {
    for (span, size) in [
        (2.0, 0.5),
        (8.0, 0.5),
        (20.0, 0.5),
        (40.0, 0.5),
        (2.0, 0.25),
        (2.0, 0.1),
    ] {
        let measured = measure(4, span, size);
        report(&format!(
            "span {span} size {size}: {measured}\n{}",
            measured.rule_meter
        ));

        assert_eq!(measured.loops, grid_hole_plate_fixture::chain_count(4));
    }
}

/// The mesh a whole-rim build makes, pinned to the bit: the speed work must not move a vertex.
#[test]
fn the_built_mesh_does_not_move() {
    for (holes, expected) in [
        (2, "2F1EC50D4EEE5AAB"),
        (4, "442C79AFA13B3132"),
        (6, "EB360773C1EFA856"),
    ] {
        let measured = measure(
            holes,
            grid_hole_plate_fixture::DEFAULT_SPAN,
            grid_hole_plate_fixture::SIZE,
        );
        assert_eq!(measured.mesh_digest, expected, "{holes} x {holes}");
    }
}
