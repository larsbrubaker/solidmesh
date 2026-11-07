//! What the BUMP rule's and the ISLAND rule's
//! searches are allowed to COST, and that paying less for them does not buy a different mesh - the
//! same move `RidgeSearchCostTests` pins for rule (v), on rules (i, mirrored) and (iv). Both are
//! seeded one chord tolerance BELOW `reading.Snap`, so the searches that find nothing stop being
//! paid for in full. The numbers are work units from `WorkBudget.Spent`, exact integers decided by
//! the algorithm. Each parameter row is one pass of a loop in the test of the same name.

use std::rc::Rc;

use geometry_processing::bevel::level_line::band_gap_entry_points::{self, BandGapBuildOptions};
use geometry_processing::bevel::level_line::{BandGapResult, BandProfile, WorkBudget};

use crate::level_line_feature_edges::feature_edges_over;
use crate::level_line_room_geometry::{box_with_square_hole, hourglass_solid};
use crate::level_line_test_geometry::cube_solid;
use crate::ridge_search_cost_tests::{append_to_cost_log, checksum_of};
use crate::several_chains_fixtures::plate_with_two_bosses;

const FOLD_DEGREES: f64 = 45.0;

/// What a build may spend, in work units: three geometries at two sizes each, the second size a
/// quarter of the first. Each ceiling sits midway between the cost before the seeds and the cost
/// after. The two bosses are one PICK of two chains, so its rows read the seeds through
/// `PickField`.
#[test]
fn the_bump_and_island_rules_cost_far_less_than_they_did() {
    for (which, size, ceiling) in [
        ("hourglass", 0.4, 95_000i64),
        ("hourglass", 0.1, 339_000),
        ("hole", 0.2, 11_400),
        ("hole", 0.05, 47_000),
        ("bosses", 0.2, 51_800),
        ("bosses", 0.05, 269_000),
    ] {
        let built = build_and_count(which, size);

        note(&format!("{which} {size}"), &built, "");
        assert!(built.0.built(), "{which} {size}");
        assert!(built.1 < ceiling, "{which} {size}: spent {}", built.1);
    }
}

/// The seeds change the COST, never the answer: every fixture comes back with the mesh it came
/// back with before them, vertex for vertex (the hourglass rows recorded with
/// `ConvexCellBound` in the cell bound). The last row is the cube picked on its top rim - rule
/// (iv)'s own case.
#[test]
fn seeding_the_searches_leaves_the_same_mesh() {
    for (which, size, vertices, triangles, checksum) in [
        ("hourglass", 0.4, 159, 314, 646.4656208235801),
        ("hourglass", 0.1, 163, 322, 720.5857173796824),
        ("hole", 0.2, 56, 112, 38.400000000000006),
        ("hole", 0.05, 68, 136, 51.6),
        ("bosses", 0.2, 114, 224, 13.20000000000001),
        ("bosses", 0.05, 136, 268, 10.799999999999997),
        ("cube", 0.05, 21, 38, 12.599999999999998),
    ] {
        let built = build_and_count(which, size);
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

/// Builds one named fixture under a budget of its own and hands back what it spent.
fn build_and_count(which: &str, size: f64) -> (BandGapResult, i64) {
    if which == "bosses" {
        let (positions, indices, first, second) = plate_with_two_bosses();
        let budget = Rc::new(
            WorkBudget::new(WorkBudget::default_limit(indices.len() as i32))
                .expect("a positive limit"),
        );
        let result = band_gap_entry_points::build_pick(
            &positions,
            &indices,
            &[first, second],
            &[true, true],
            Some(feature_edges_over(&positions, &indices, FOLD_DEGREES)),
            size,
            BandProfile::Fillet,
            &BandGapBuildOptions {
                budget: Some(Rc::clone(&budget)),
                ..BandGapBuildOptions::default()
            },
        )
        .expect("a valid pick");

        let spent = budget.spent.get();
        return (result, spent);
    }

    let (positions, indices, chain) = match which {
        "hourglass" => hourglass_solid(10.0, 2.0, 4.0, 4.0),
        "cube" => {
            // A cube picked on its TOP RIM: the top face's whole boundary is the chain, so both its
            // triangles have three corners on the line and no outside corner - rule (iv)'s own
            // case, which no other fixture here puts it in.
            let (p, i) = cube_solid(1.0);
            (p, i, vec![4, 5, 6, 7])
        }
        _ => box_with_square_hole(2.0, 0.5, 1.0),
    };

    let budget = Rc::new(
        WorkBudget::new(WorkBudget::default_limit(indices.len() as i32)).expect("a positive limit"),
    );
    let result = band_gap_entry_points::build(
        &positions,
        &indices,
        &chain,
        true,
        Some(feature_edges_over(&positions, &indices, FOLD_DEGREES)),
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

/// Writes the measured numbers where a reader can find them - a test runner swallows the console,
/// and these numbers are the whole point of the fixture.
fn note(what: &str, built: &(BandGapResult, i64), extra: &str) {
    let line = format!(
        "{what}: spent {} built {} {extra}",
        built.1,
        if built.0.built() { "True" } else { "False" }
    );
    println!("{line}");
    append_to_cost_log(&line);
}
