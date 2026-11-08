//! The Many Holes BRIDGE on a synthetic part (`two_hole_bridge_fixture`) - a joint build of two
//! rims that stand four sizes apart, over a top face whose triangles SPAN the two of them, must
//! leave each picked side its own closed loop and must not leave the refined mesh with two vertices
//! in the same place. Each test's cases run as a loop inside it; coincident vertices are read by
//! `level_line_test_geometry::coincident_vertices`.

use std::collections::{HashMap, HashSet};
use std::time::Instant;

use solidmesh_math::Vector3;

use crate::level_line_test_geometry::coincident_vertices;
use crate::two_hole_bridge_fixture::{self, GAP_IN_SIZES, RADIUS_IN_SIZES, SIZE};
use solidmesh::bevel::level_line::BandGapRefusal;

/// What the spanning triangulation is, in sizes -
/// a statement of the fixture's geometry, so it fails loudly if a change to the fixture quietly
/// stops it spanning.
#[test]
fn the_middle_strip_spans_the_two_rims_with_long_edges() {
    let (positions, indices, chains) = two_hole_bridge_fixture::pick_with_radius(RADIUS_IN_SIZES);
    let mut spanning: Vec<f64> = spanning_edges(&positions, &indices, &chains)
        .iter()
        .map(|e| e / SIZE)
        .collect();

    let least = spanning.iter().copied().fold(f64::INFINITY, f64::min);
    let greatest = spanning.iter().copied().fold(f64::NEG_INFINITY, f64::max);
    let count = spanning.len();
    spanning.sort_by(f64::total_cmp);
    println!(
        "bridge fixture: {} vertices, {} triangles, {count} cross-chain edges, least {least:.2} sizes, middling {:.2}, greatest {greatest:.2}",
        positions.len(),
        indices.len(),
        spanning.get(count / 2).copied().unwrap_or(f64::NAN)
    );

    assert!(
        count > 0,
        "with no triangle spanning the two rims there is no bridge to reproduce"
    );
    assert!(
        greatest >= 10.0,
        "the witness's bridging edges are 9 to 14 sizes: {greatest}"
    );
    assert!(
        indices.len() < 1000,
        "the point of the fixture is that the joint build is under a second"
    );
}

/// The joint build of both rims - no refusal,
/// every picked side left exactly one closed loop wholly its own, and no two vertices of the
/// refined mesh in the same place. Swept along the hole radius, which sets the longest spanning
/// chord (`2R + gap`: 14, 20 and 26 sizes, the witness's own range).
#[test]
fn a_build_of_two_spanned_rims_leaves_each_side_its_own_loop() {
    for radius_in_sizes in [5.0, 8.0, 11.0] {
        let (positions, indices, chains) =
            two_hole_bridge_fixture::pick_with_radius(radius_in_sizes);
        let started = Instant::now();

        let built = two_hole_bridge_fixture::build(&positions, &indices, &chains, SIZE);

        let chord = (2.0 * radius_in_sizes) + GAP_IN_SIZES;
        println!(
            "bridge at radius {radius_in_sizes} sizes (longest chord {chord} sizes): {:?}, {} loops, {:.2} s",
            built.refusal,
            built.loops.len(),
            started.elapsed().as_secs_f64()
        );
        let mut rows: Vec<_> = built.side_loops.iter().collect();
        rows.sort_by_key(|r| (r.chain, r.side));
        for row in &rows {
            println!(
                "  chain {} side {} group {}: {} owned, {} closed, {} mixed, met {:?}, owed {}",
                row.chain,
                row.side,
                row.group_tag,
                row.loops,
                row.closed_loops,
                row.mixed_loops,
                row.met_chains,
                row.owed
            );
        }

        let doubled = match &built.mesh {
            Some(mesh) => coincident_vertices(&mesh.borrow(), SIZE * 1e-6),
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
            "both rims have four sizes of room between them: radius {radius_in_sizes}"
        );
        assert_eq!(
            built.side_loops.len(),
            4,
            "two chains, two picked sides each"
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
            "a side whose loop is shared with another chain is a BRIDGE: {}",
            bridged.join(" | ")
        );
        assert!(
            doubled.is_empty(),
            "the refinement must not open a crack - coincident unwelded split vertices: {:?}",
            doubled.iter().take(4).collect::<Vec<_>>()
        );
    }
}

/// Every INPUT edge whose two ends are on DIFFERENT chains of the pick, by
/// length. Shared with `hole_row_bridge_tests`, which reads the same edges.
pub(crate) fn spanning_edges(
    positions: &[Vector3],
    indices: &[(i32, i32, i32)],
    chains: &[Vec<i32>],
) -> Vec<f64> {
    let mut chain_of: HashMap<i32, usize> = HashMap::new();
    for (k, chain) in chains.iter().enumerate() {
        for &v in chain {
            chain_of.entry(v).or_insert(k);
        }
    }

    let mut seen = HashSet::new();
    let mut lengths = Vec::new();
    for &(a, b, c) in indices {
        for (u, v) in [(a, b), (b, c), (c, a)] {
            let (Some(ku), Some(kv)) = (chain_of.get(&u), chain_of.get(&v)) else {
                continue;
            };
            if ku == kv {
                continue;
            }

            if seen.insert((u.min(v), u.max(v))) {
                lengths.push((positions[u as usize] - positions[v as usize]).length());
            }
        }
    }

    lengths
}
