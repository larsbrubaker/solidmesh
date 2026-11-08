//! What the FOREIGN reading of a face rim costs as the rim gains loops, over
//! `grid_hole_plate_fixture`. The reading is per chain, and a chain's own reading must be about the
//! part near that chain, not about how many other loops were picked with it.

use std::rc::Rc;

use solidmesh::bevel::level_line::band_gap_reading;
use solidmesh::bevel::level_line::foreign_room_counters;
use solidmesh::bevel::level_line::{BandProfile, WorkBudget};

use crate::grid_hole_plate_fixture;

/// The ratio of per-chain foreign edges read allowed between the 36-hole plate and the 4-hole one.
const GROWTH_CEILING: f64 = 1.6;

struct Measured {
    work: i64,
    calls: i64,
    edges_read: i64,
    per_chain: f64,
    chains: usize,
    triangles: usize,
    pair_calls: i64,
    pair_edges: i64,
}

fn measure(holes: i32) -> Measured {
    let (positions, indices, chains) = grid_hole_plate_fixture::pick(holes);
    foreign_room_counters::reset();
    let closed = vec![true; chains.len()];

    let budget = Rc::new(WorkBudget::new(200_000_000).expect("a positive limit"));
    let reading = band_gap_reading::read_room_of_pick(
        &positions,
        &indices,
        &chains,
        &closed,
        Some(grid_hole_plate_fixture::feature_edges(&positions, &indices)),
        BandProfile::Chamfer,
        1e-9,
        Some(budget),
        f64::NAN,
        0.0,
        f64::INFINITY,
    )
    .expect("the reading");

    assert!(
        reading.read,
        "the room reading did not finish inside its budget"
    );

    // The FOREIGN reading alone - the chain-to-chain rows are a separate question with its own
    // prune (RoomPairsCostTests), and a total over both would hide which of them grows.
    let (calls, edges_read, pair_calls, pair_edges) = foreign_room_counters::with(|t| {
        (
            t.calls - t.pair_calls,
            t.edges_read - t.pair_edges_read,
            t.pair_calls,
            t.pair_edges_read,
        )
    });
    Measured {
        work: reading.work_spent,
        calls,
        edges_read,
        per_chain: edges_read as f64 / chains.len() as f64,
        chains: chains.len(),
        triangles: indices.len(),
        pair_calls,
        pair_edges,
    }
}

/// The table, written where a run can read it.
/// Not an assertion on the numbers - the requirement is the RATIO below.
#[test]
fn the_foreign_reading_cost_is_measured_over_a_growing_rim() {
    let mut rows = Vec::new();
    for holes in [2, 4, 6] {
        let m = measure(holes);
        rows.push(format!(
            "{} holes: chains {}, tris {}, work {}, foreign calls {}, foreign edges read {}, \
             per chain {:.1}, pair calls {}, pair edges {}",
            holes * holes,
            m.chains,
            m.triangles,
            m.work,
            m.calls,
            m.edges_read,
            m.per_chain,
            m.pair_calls,
            m.pair_edges
        ));
    }

    // A test host swallows the console, so a run that wants the table names a file to have it in.
    if let Ok(log) = std::env::var("LEVELLINE_COST_LOG") {
        if !log.is_empty() {
            use std::io::Write;
            let mut file = std::fs::OpenOptions::new()
                .create(true)
                .append(true)
                .open(&log)
                .expect("the cost log");
            writeln!(file, "{}", rows.join("\n")).expect("the cost log");
        }
    }

    println!("{}", rows.join("\n"));
    assert_eq!(rows.len(), 3);
}

/// A chain's foreign reading must not
/// grow with how many OTHER chains were picked.
#[test]
fn a_chains_foreign_reading_does_not_grow_with_the_rest_of_the_pick() {
    let few = measure(2);
    let many = measure(6);

    assert!(
        many.per_chain / few.per_chain < GROWTH_CEILING,
        "per chain {} at 36 holes against {} at 4",
        many.per_chain,
        few.per_chain
    );
}
