//! The hunt for the Many Holes BRIDGE beyond the two-hole part - a row of four or six spanned rims,
//! a ring turned or re-segmented or slid off the pitch, and the plate's outer boundary picked as a
//! chain of its own. Every one of them is a joint band build that must leave each picked side its
//! own closed loop and must not leave the refined mesh with two vertices in the same place.
//!
//! `bridge_fixture_tests` builds two spanned rims and stays clean at every radius tried, so
//! something the witness has is missing from it. These are that something, one knob at a time, each
//! run at two sizes. The fixture is `hole_row_bridge_fixture`. Parameter rows are a loop over the
//! same-named test.

use std::time::Instant;

use crate::bridge_fixture_tests::spanning_edges;
use crate::hole_row_bridge_fixture::{self, HoleVariation, SIZE};
use crate::level_line_room_geometry::SolidWithChains;
use crate::level_line_test_geometry::coincident_vertices;
use geometry_processing::bevel::level_line::BandGapRefusal;

/// The variants of the row, in the order they were tried.
#[derive(Debug, Clone, Copy)]
enum Variant {
    /// Four plain rings in a row: a middle hole is spanned from BOTH sides at once.
    FourInARow,
    /// Six plain rings, so four of them are spanned twice.
    SixInARow,
    /// Four rings, the second TURNED half a segment against its neighbours.
    TurnedRing,
    /// Four rings, the second cut into 48 segments where its neighbours carry 32.
    MixedSegments,
    /// Four rings, the second slid a third of a segment along the row.
    ShiftedHole,
    /// Four plain rings and the plate's OUTER boundary picked as a fifth chain.
    OuterRimPicked,
    /// Six rings, every asymmetry at once, and the outer rim picked - the witness's shape.
    EveryKnobAtOnce,
}

/// Every row variant, at the size the witness
/// failed at and at half of it. Both sizes, because the bridge is a story about how far the field
/// climbs over a long spanning edge, and halving the size doubles every length the field is
/// measured in.
#[test]
fn a_row_of_spanned_rims_leaves_each_side_its_own_loop() {
    for row in [
        Variant::FourInARow,
        Variant::SixInARow,
        Variant::TurnedRing,
        Variant::MixedSegments,
        Variant::ShiftedHole,
        Variant::OuterRimPicked,
        Variant::EveryKnobAtOnce,
    ] {
        for size in [0.5, 0.25] {
            a_row_of_spanned_rims_leaves_each_side_its_own_loop_row(row, size);
        }
    }
}

fn a_row_of_spanned_rims_leaves_each_side_its_own_loop_row(row: Variant, size: f64) {
    let (positions, indices, chains) = build(row);
    let started = Instant::now();

    let built = hole_row_bridge_fixture::build(&positions, &indices, &chains, size);

    println!(
        "{row:?} at size {size}: {} vertices, {} triangles, {} chains -> {:?}, {} loops, {:.2} s",
        positions.len(),
        indices.len(),
        chains.len(),
        built.refusal,
        built.loops.len(),
        started.elapsed().as_secs_f64()
    );
    let mut rows: Vec<_> = built.side_loops.iter().collect();
    rows.sort_by_key(|r| (r.chain, r.side));
    for side in &rows {
        println!(
            "  chain {} side {} group {}: {} owned, {} closed, {} mixed, met {:?}, owed {}",
            side.chain,
            side.side,
            side.group_tag,
            side.loops,
            side.closed_loops,
            side.mixed_loops,
            side.met_chains,
            side.owed
        );
    }

    let doubled = match &built.mesh {
        Some(mesh) => coincident_vertices(&mesh.borrow(), size * 1e-6),
        None => Vec::new(),
    };
    println!(
        "  coincident vertex pairs in the refined mesh: {}{}",
        doubled.len(),
        doubled
            .first()
            .map(|d| format!(", first {}/{} at {:?}", d.0, d.1, d.2))
            .unwrap_or_default()
    );

    assert_eq!(
        built.refusal,
        BandGapRefusal::None,
        "{row:?} at {size}: every rim has four sizes of room to its neighbour"
    );
    assert_eq!(
        built.side_loops.len(),
        2 * chains.len(),
        "{row:?} at {size}: two picked sides per chain"
    );
    let bridged: Vec<String> = built
        .side_loops
        .iter()
        .filter(|r| !r.owed)
        .map(|r| {
            format!(
                "chain {} side {}: {} owned, {} mixed, met {:?}",
                r.chain, r.side, r.loops, r.mixed_loops, r.met_chains
            )
        })
        .collect();
    assert!(
        bridged.is_empty(),
        "{row:?} at {size}: a side whose loop is shared with another chain is a BRIDGE: {}",
        bridged.join(" | ")
    );
    assert!(
        doubled.is_empty(),
        "{row:?} at {size}: the refinement must not open a crack - coincident unwelded split vertices: {:?}",
        doubled.iter().take(4).collect::<Vec<_>>()
    );
}

/// Every row spans its rims with long edges (rows FourInARow, EveryKnobAtOnce): what each variant's
/// triangulation actually is - the spanning edges it carries and how long they are - so a change to
/// the fixture that quietly stops it spanning fails here rather than going unnoticed in a green
/// build above.
#[test]
fn every_row_spans_its_rims_with_long_edges() {
    for row in [Variant::FourInARow, Variant::EveryKnobAtOnce] {
        let (positions, indices, chains) = build(row);
        let mut spanning: Vec<f64> = spanning_edges(&positions, &indices, &chains)
            .into_iter()
            .map(|e| e / SIZE)
            .collect();
        spanning.sort_by(f64::total_cmp);

        assert!(
            !spanning.is_empty(),
            "{row:?}: with no triangle spanning two rims there is no bridge to reproduce"
        );
        println!(
            "{row:?}: {} vertices, {} triangles, {} chains, {} cross-chain edges, least {:.2} sizes, middling {:.2}, greatest {:.2}",
            positions.len(),
            indices.len(),
            chains.len(),
            spanning.len(),
            spanning[0],
            spanning[spanning.len() / 2],
            spanning[spanning.len() - 1]
        );

        let greatest = spanning[spanning.len() - 1];
        assert!(
            greatest >= 10.0,
            "{row:?}: the witness's bridging edges are 9 to 14 sizes: {greatest}"
        );
    }
}

/// The pick each variant names. The geometry is always quoted in the fixture's
/// own size.
fn build(row: Variant) -> SolidWithChains {
    match row {
        Variant::FourInARow => hole_row_bridge_fixture::pick(4, &[], false),
        Variant::SixInARow => hole_row_bridge_fixture::pick(6, &[], false),
        Variant::TurnedRing => hole_row_bridge_fixture::pick(4, &[HoleVariation::turned(1)], false),
        Variant::MixedSegments => {
            hole_row_bridge_fixture::pick(4, &[HoleVariation::resegmented(1, 48)], false)
        }
        Variant::ShiftedHole => {
            hole_row_bridge_fixture::pick(4, &[HoleVariation::shifted(1)], false)
        }
        Variant::OuterRimPicked => hole_row_bridge_fixture::pick(4, &[], true),
        Variant::EveryKnobAtOnce => hole_row_bridge_fixture::pick(
            6,
            &[
                HoleVariation::turned(1),
                HoleVariation::resegmented(2, 48),
                HoleVariation::shifted(4),
            ],
            true,
        ),
    }
}
