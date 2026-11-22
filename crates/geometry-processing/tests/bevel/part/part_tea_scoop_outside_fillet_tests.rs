//! "Tea Scoop Outside Fillet": a 1 mm round, 8 segments, bend threshold 13 degrees, over two face
//! rims of a tea scoop - the handle's rims (342 edges in two loops) and the scoop bowl's outer rim
//! (975 edges). The part's mesh and the two picks, as the design saved them, are read from
//! `tests/bevel/fixtures/tea-scoop-outside-fillet.json`. A 1 mm round fits every edge of this part:
//! where the bowl's underside (long strip facets about 1 mm wide, turning under half a degree
//! between neighbours) meets the flat end cap at y = -72.5 the two meet at a right angle, so the
//! whole pick has to build at 1 with nothing refused and give one closed solid.
//!
//! Lars Brubaker made the design this fixture came from and dedicates
//! `tests/bevel/fixtures/tea-scoop-outside-fillet.json` to the public domain under CC0 1.0
//! (<https://creativecommons.org/publicdomain/zero/1.0/>).

use std::cell::RefCell;
use std::rc::Rc;
use std::sync::{Arc, Mutex};
use std::time::Instant;

use serde::Deserialize;
use solidmesh::bevel::level_line::level_line_routing as routing;
use solidmesh::bevel::level_line::{FieldCounters, WorkBudget};
use solidmesh::bevel::part::{
    bevel_part, BevelPartOutcome, BevelPartRequest, BevelProgress, BevelProgressReport,
};
use solidmesh::topology::BevelEdgeGroup;
use solidmesh_mesh::{CancelToken, Mesh};

#[derive(Deserialize)]
struct SavedFillet {
    vertices: Vec<f64>,
    faces: Vec<i32>,
    groups: Vec<BevelEdgeGroup>,
    size: f64,
    segments: i32,
    profile_is_round: bool,
    bend_threshold_degrees: f64,
}

fn the_saved_fillet() -> BevelPartRequest {
    let path = format!(
        "{}/tests/bevel/fixtures/tea-scoop-outside-fillet.json",
        env!("CARGO_MANIFEST_DIR")
    );
    let text = std::fs::read_to_string(&path).unwrap_or_else(|e| panic!("{path}: {e}"));
    let saved: SavedFillet = serde_json::from_str(&text).unwrap_or_else(|e| panic!("{path}: {e}"));
    BevelPartRequest::new(
        Some(Arc::new(Mesh::from_arrays(&saved.vertices, &saved.faces))),
        saved.groups,
        saved.size,
        saved.segments,
        saved.profile_is_round,
        saved.bend_threshold_degrees,
    )
}

/// The request with only group `index` picked.
fn the_saved_fillet_with_only(index: usize) -> BevelPartRequest {
    let mut request = the_saved_fillet();
    request.groups = vec![request.groups[index].clone()];
    request
}

/// Every refused group and loop, in words.
fn refusals(outcome: &BevelPartOutcome) -> Vec<String> {
    let mut words = Vec::new();
    for (index, row) in outcome.group_results.iter().enumerate() {
        let Some(row) = row else {
            words.push(format!("group {index}: no row"));
            continue;
        };
        if let Some(failure) = &row.failure {
            words.push(format!("group {index}: {} {failure:?}", failure.code()));
        }
        for refused in row.loop_failures.iter().flatten() {
            words.push(format!("group {index} loop: {refused:?}"));
        }
    }
    words
}

/// How many separate pieces the mesh is: faces sharing a vertex are one piece.
fn shells(mesh: &Mesh) -> usize {
    let mut parent: Vec<usize> = (0..mesh.vertices.len()).collect();
    fn root(parent: &mut [usize], mut at: usize) -> usize {
        while parent[at] != at {
            parent[at] = parent[parent[at]];
            at = parent[at];
        }
        at
    }
    let mut used = vec![false; mesh.vertices.len()];
    for face in mesh.faces.iter() {
        let corners = [face.v0 as usize, face.v1 as usize, face.v2 as usize];
        for &corner in &corners {
            used[corner] = true;
        }
        for &corner in &corners[1..] {
            let (a, b) = (root(&mut parent, corners[0]), root(&mut parent, corner));
            parent[a] = b;
        }
    }
    let mut roots: Vec<usize> = (0..used.len())
        .filter(|&v| used[v])
        .map(|v| root(&mut parent, v))
        .collect();
    roots.sort_unstable();
    roots.dedup();
    roots.len()
}

#[test]
fn both_rims_round_at_one_as_one_closed_solid() {
    let outcome = bevel_part(the_saved_fillet(), &CancelToken::new(), None)
        .unwrap_or_else(|error| panic!("the build ran: {error}"));
    let refused = refusals(&outcome);
    assert!(refused.is_empty(), "{}", refused.join("\n"));
    let mesh = outcome.mesh.expect("the fillet gives a mesh");
    assert!(mesh.is_manifold(), "the fillet's answer is not closed");
    assert_eq!(shells(&mesh), 1);
}

/// The whole pick and each group alone: how long each takes, the routes taken, the known room the
/// level line was handed and what was refused. Run it with `--ignored --nocapture`.
#[test]
#[ignore = "a probe: prints timings, rooms and refusals, asserts nothing"]
fn probe_the_tea_scoop_fillet() {
    for (name, request) in [
        ("whole pick", the_saved_fillet()),
        ("group 0 alone", the_saved_fillet_with_only(0)),
        ("group 1 alone", the_saved_fillet_with_only(1)),
    ] {
        routing::set_first_route(None);
        routing::set_last_route(None);
        routing::set_watch_room(Some(Arc::new(move |room| {
            println!("  {name}: known room {room:?}")
        })));
        let started = Instant::now();
        let outcome = bevel_part(request, &CancelToken::new(), None);
        let seconds = started.elapsed().as_secs_f64();
        routing::set_watch_room(None);
        match outcome {
            Err(error) => println!("{name}: error {error} in {seconds:.3} s"),
            Ok(outcome) => println!(
                "{name}: {seconds:.3} s, routes first {:?} last {:?}, mesh {:?}\n  {}",
                routing::first_route(),
                routing::last_route(),
                outcome.mesh.as_ref().map(|mesh| (
                    mesh.faces.len(),
                    mesh.is_manifold(),
                    shells(mesh)
                )),
                refusals(&outcome).join("\n  ")
            ),
        }
    }
}

/// The whole pick's wall-clock time per phase the progress bar names, in the order each phase
/// was first named, with each level-line allowance's per-phase work and time, the refinement's
/// per-rule searches and the routes taken. Run it with `--ignored --nocapture`, in a release build
/// for timings that mean anything.
#[test]
#[ignore = "a probe: prints the time per reported phase, asserts nothing"]
fn probe_the_tea_scoop_fillet_phases() {
    // Each phase as the bar names it with the seconds spent while it was the latest one named,
    // and when the last report came.
    type Phases = (Vec<(String, f64)>, Instant);
    let phases: Arc<Mutex<Phases>> = Arc::new(Mutex::new((Vec::new(), Instant::now())));
    let seen = phases.clone();
    let progress: BevelProgressReport = Arc::new(move |_: f64, said: &BevelProgress| {
        let named = format!("{:?}", said.doing);
        let mut seen = seen.lock().expect("one reporter at a time");
        let now = Instant::now();
        let since = now.duration_since(seen.1).as_secs_f64();
        seen.1 = now;
        if let Some(last) = seen.0.last_mut() {
            last.1 += since;
        }
        if seen.0.last().map(|(name, _)| name.as_str()) != Some(named.as_str()) {
            seen.0.push((named, 0.0));
        }
    });
    routing::set_first_route(None);
    routing::set_last_route(None);
    FieldCounters::reset();
    FieldCounters::set_meter_on(true);
    let budgets: Rc<RefCell<Vec<Rc<WorkBudget>>>> = Rc::new(RefCell::new(Vec::new()));
    let gathered = Rc::clone(&budgets);
    routing::set_watch_budget(Some(Rc::new(move |b: &Rc<WorkBudget>| {
        gathered.borrow_mut().push(Rc::clone(b))
    })));
    let started = Instant::now();
    phases.lock().expect("unshared").1 = started;
    let outcome = bevel_part(the_saved_fillet(), &CancelToken::new(), Some(progress));
    let total = started.elapsed().as_secs_f64();
    routing::set_watch_budget(None);
    FieldCounters::set_meter_on(false);
    for (k, budget) in budgets.borrow().iter().enumerate() {
        println!("budget {k}:\n{}", budget.phase_report());
    }
    println!("rules:\n{}", FieldCounters::rule_report());
    println!("refinement:\n{}", FieldCounters::refinement_report());
    println!("{}", FieldCounters::report());
    let mut seen = phases.lock().expect("the build is over");
    let tail = seen.1.elapsed().as_secs_f64();
    if let Some(last) = seen.0.last_mut() {
        last.1 += tail;
    }
    for (name, seconds) in &seen.0 {
        println!("{seconds:8.3} s  {name}");
    }
    println!(
        "total {total:.3} s, routes first {:?} last {:?}",
        routing::first_route(),
        routing::last_route()
    );
    if let Ok(outcome) = outcome {
        println!(
            "mesh {:?}\n  {}",
            outcome
                .mesh
                .as_ref()
                .map(|mesh| (mesh.faces.len(), mesh.is_manifold(), shells(mesh))),
            refusals(&outcome).join("\n  ")
        );
    }
}
