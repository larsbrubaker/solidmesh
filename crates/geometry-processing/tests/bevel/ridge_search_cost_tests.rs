//! What the peak/ridge rule's search is allowed to COST, and that paying less for it does not buy a
//! different answer. Rule (v) of `BandRefinement` hands its firing threshold `max(snap,
//! highestCorner + chordTolerance)` to the bounded search as `atLeast`, so the search discards from
//! its first step every part of the triangle whose Lipschitz bound cannot beat it. The numbers
//! pinned are work units from `WorkBudget.Spent`, which are load-independent; the mesh digests are
//! an empirical pin against drift (re-recorded when the convex cell bound joined the cell bound).
//! Parameter rows are loops over the same-named test.

use std::rc::Rc;

use solidmesh::bevel::level_line::band_gap_entry_points::{self, BandGapBuildOptions};
use solidmesh::bevel::level_line::{BandGapResult, BandProfile, LevelLineMesh, WorkBudget};

use crate::level_line_feature_edges::feature_edges_over;
use crate::level_line_room_geometry::{hourglass_solid, l_prism, SolidWithChain};

const FOLD_DEGREES: f64 = 45.0;

fn part(which: &str) -> SolidWithChain {
    if which == "hourglass" {
        hourglass_solid(10.0, 2.0, 4.0, 4.0)
    } else {
        l_prism()
    }
}

/// What a build may spend, in work units, on each proof - two geometries at two sizes each, the
/// second size a quarter of the first because the rule's cost is what grows as the size falls.
/// Each ceiling sits midway between the measured before/after pair, so every row is RED on the
/// old code and GREEN with room.
#[test]
fn the_ridge_rule_costs_far_less_than_it_did() {
    for (which, size, ceiling) in [
        ("hourglass", 0.4, 145_000i64),
        ("hourglass", 0.1, 533_000),
        ("L", 0.2, 112_000),
        ("L", 0.05, 434_000),
    ] {
        let built = build_and_count(&part(which), size);

        note(&format!("{which} {size}"), &built, "");
        assert!(built.0.built(), "{which} {size}");
        assert!(built.1 < ceiling, "{which} {size}: spent {}", built.1);
    }
}

/// The mesh these builds make, pinned: a change to how the ridge rule's search is PRUNED must not
/// move a vertex without someone re-recording these numbers and saying why. The digest is of the
/// BUILT MESH - vertex count, triangle count, and a plain sum of every coordinate, compared to a
/// billionth of its own size rather than to the bit.
#[test]
fn these_builds_meshes_are_a_pin_against_drift() {
    for (which, size, vertices, triangles, checksum) in [
        ("hourglass", 0.4, 159, 314, 646.4656208235801),
        ("hourglass", 0.1, 163, 322, 720.5857173796824),
        ("L", 0.2, 72, 140, 604.2910959959811),
        ("L", 0.05, 82, 160, 723.8124858573989),
    ] {
        let built = build_and_count(&part(which), size);
        let mesh = built.0.mesh.as_ref().expect("the working mesh").borrow();

        note(
            &format!("{which} {size} mesh"),
            &built,
            &format!(
                "vertices {} triangles {} checksum {:?}",
                mesh.vertex_count(),
                mesh.triangle_count(),
                checksum_of(&mesh)
            ),
        );

        assert_eq!(mesh.vertex_count(), vertices, "{which} {size}");
        assert_eq!(mesh.triangle_count(), triangles, "{which} {size}");
        assert!(
            (checksum_of(&mesh) - checksum).abs() < checksum.abs() * 1e-9,
            "{which} {size}: checksum {:?}",
            checksum_of(&mesh)
        );
    }
}

/// Every coordinate of every vertex added up - one number that moves when any vertex moves
/// (`SeededSearchCostTests` reads it too).
pub fn checksum_of(mesh: &LevelLineMesh) -> f64 {
    let mut total = 0.0;
    for i in 0..mesh.vertex_count() {
        let p = mesh.position(i);
        total += p.x + p.y + p.z;
    }

    total
}

/// Builds the pick under a budget of its own and hands back what it spent. Only the budget is
/// read: a budget is per call and cannot be contaminated by other tests running in parallel.
fn build_and_count(part: &SolidWithChain, size: f64) -> (BandGapResult, i64) {
    let budget = Rc::new(
        WorkBudget::new(WorkBudget::default_limit(part.1.len() as i32)).expect("a positive limit"),
    );
    let result = band_gap_entry_points::build(
        &part.0,
        &part.1,
        &part.2,
        true,
        Some(feature_edges_over(&part.0, &part.1, FOLD_DEGREES)),
        size,
        BandProfile::Fillet,
        &BandGapBuildOptions {
            budget: Some(Rc::clone(&budget)),
            ..BandGapBuildOptions::default()
        },
    )
    .expect("a valid pick");

    let spent = budget.spent.get();
    (result, spent)
}

/// Writes the measured numbers where a reader can find them. A path this process may not write is
/// a bad environment variable, never a failed claim.
fn note(what: &str, built: &(BandGapResult, i64), extra: &str) {
    let line = format!("{what}: spent {} {extra}", built.1);
    println!("{line}");
    append_to_cost_log(&line);
}

/// Appends the line to `LEVELLINE_COST_LOG`, every write failure swallowed: a test
/// runner swallows the console, and these numbers are the whole point of the fixture
/// (`SeededSearchCostTests` writes through it too).
pub fn append_to_cost_log(line: &str) {
    if let Ok(to) = std::env::var("LEVELLINE_COST_LOG") {
        if !to.is_empty() {
            use std::io::Write;
            if let Ok(mut file) = std::fs::OpenOptions::new()
                .create(true)
                .append(true)
                .open(&to)
            {
                let _ = writeln!(file, "{line}");
            }
        }
    }
}
