//! Port of `LevelLine/GrazeFixtureTests.cs`: the Many Holes BRIDGE as a single bad EDGE, in under
//! a second - one mesh edge whose two ends are an ordinary Inside/Outside pair but which crosses
//! the line THREE times, because it grazes a second chain's band on the way across. The joint build
//! of both rims must still leave each picked side its own closed loop. Both ways the edge can reach
//! the rules are swept: a graze with HEIGHT under it, at two depths, and one so shallow the triangle
//! on it is a needle below the refinement's no-height floor. The line-to-line shapes
//! (`line_to_line_fixture.rs`) are the same claim for an edge that runs from one chain's band to
//! the other's. C# `[Arguments]` rows are loops over the same-named test.

use std::time::Instant;

use geometry_processing::bevel::level_line::{
    BandGapRefusal, BandGapResult, BandProfile, LevelLineMesh, PickField, VertexFieldReading,
    VertexSide,
};
use vector_math::Vector3;

use crate::graze_fixture::{self, GrazePick, GRAZE_AS_NEEDLE, GRAZE_WITH_HEIGHT};
use crate::level_line_test_geometry::{coincident_vertices, open_edges, signed_volume_of};
use crate::line_to_line_fixture::{self, Defect, LinePick};

/// C# `Math.Sign(double)`.
fn sign(x: f64) -> i32 {
    if x > 0.0 {
        1
    } else if x < 0.0 {
        -1
    } else {
        0
    }
}

/// Forty stations along an edge, read by the pick's own field.
fn stations(field: &PickField, from: Vector3, to: Vector3) -> Vec<f64> {
    (0..=40)
        .map(|k| {
            field
                .evaluate(from + ((to - from) * (k as f64 / 40.0)))
                .expect("the field reads")
                .value
        })
        .collect()
}

fn least(values: &[f64]) -> f64 {
    values.iter().copied().fold(f64::INFINITY, f64::min)
}

fn greatest(values: &[f64]) -> f64 {
    values.iter().copied().fold(f64::NEG_INFINITY, f64::max)
}

/// What the fixture's geometry IS - the numbers the defect depends on. Not a contract, a statement,
/// so it fails loudly if a change to the fixture quietly stops the chord grazing.
#[test]
fn the_grazing_edge_crosses_the_line_three_times() {
    for graze in [GRAZE_WITH_HEIGHT, GRAZE_AS_NEEDLE, 0.15] {
        the_grazing_edge_crosses_the_line_three_times_at(graze);
    }
}

fn the_grazing_edge_crosses_the_line_three_times_at(graze_in_setbacks: f64) {
    let size = graze_fixture::SIZE;
    let pick = graze_fixture::pick(graze_in_setbacks, graze_fixture::RING_SEGMENTS, size);
    let field = graze_fixture::field_of(&pick.chains, &pick.positions, size);
    let from = pick.positions[pick.rim_end as usize];
    let to = pick.positions[pick.clear_end as usize];

    // Forty stations along the edge, read by the pick's own field: the signature is -, +, -, +.
    let values = stations(&field, from, to);
    let crossings = (1..values.len())
        .filter(|&k| sign(values[k]) != sign(values[k - 1]))
        .count();

    let apex_height = height_over_the_edge(from, to, pick.positions[pick.apex as usize]);
    println!(
        "graze at {graze_in_setbacks:.4} setbacks: {} vertices, {} triangles, touch radius {:.4} sizes, \
         edge {:.2} sizes, apex height {:.4e} sizes",
        pick.positions.len(),
        pick.indices.len(),
        pick.graze_radius / size,
        (to - from).length() / size,
        apex_height / size
    );
    println!(
        "  field along it: ends {:.3} and {:.3} sizes, least {:.3}, greatest {:.3}, least INSIDE the \
         span {:.3}, {crossings} sign changes",
        values[0] / size,
        values[values.len() - 1] / size,
        least(&values) / size,
        greatest(&values) / size,
        least(&values[1..values.len() - 1]) / size
    );
    let report: Vec<String> = values.iter().map(|v| format!("{:.2}", v / size)).collect();

    assert!(
        edge_of_the_graze_is_in_the_input(&pick),
        "the graze has to be ONE edge of the input, or there is nothing for a rule to be asked about"
    );
    assert!(values[0] < 0.0, "the near end is a vertex OF the outer rim");
    assert!(
        values[values.len() - 1] > 0.0,
        "the far end stands clear of both rims"
    );
    assert!(
        crossings >= 3,
        "a graze is -, +, -, + - two crossings the ends cannot see: {}",
        report.join(", ")
    );
    assert!(
        pick.indices.len() < 1000,
        "the point of the fixture is that the joint build is under a second"
    );

    // The two ways the same edge reaches the rules: a triangle with height under it, and a needle
    // the refinement's no-height floor - 1e-3 of the chord tolerance, so 2.5e-5 here - reads as
    // flat.
    if graze_in_setbacks >= 0.01 {
        assert!(
            apex_height > 1e-3,
            "this variant is the one with real height under the edge: {apex_height}"
        );
    } else {
        assert!(
            apex_height < 2.5e-5,
            "this variant is the one under the no-height floor: {apex_height}"
        );
    }
}

/// The part itself is a closed solid, so nothing the build reports is an artefact of the input.
#[test]
fn the_fixture_is_a_closed_solid() {
    for graze in [GRAZE_WITH_HEIGHT, GRAZE_AS_NEEDLE] {
        let pick = graze_fixture::pick(graze, graze_fixture::RING_SEGMENTS, graze_fixture::SIZE);
        is_a_closed_solid(
            &pick.positions,
            &pick.indices,
            &format!("graze at {graze:.4}"),
        );
    }
}

fn is_a_closed_solid(positions: &[Vector3], indices: &[(i32, i32, i32)], name: &str) {
    let mesh = LevelLineMesh::from_triangles(positions, indices, None).expect("a valid mesh");
    let open = open_edges(&mesh);
    let volume = signed_volume_of(positions, indices);
    println!(
        "{name}: {} open edges, six times the volume {volume:.4}",
        open.len()
    );

    assert_eq!(
        open.len(),
        0,
        "a tiling with a T-vertex or a hole in it proves nothing: {:?}",
        open.iter().take(4).collect::<Vec<_>>()
    );
    assert!(
        volume > 0.0,
        "every face must be wound outward, or the folds read backwards"
    );
}

/// The joint build of both rims over the grazing edge: no refusal, every picked side left exactly
/// one closed loop wholly its own, and no two vertices of the refined mesh in the same place.
#[test]
fn a_build_over_a_grazing_edge_leaves_each_side_its_own_loop() {
    for graze in [GRAZE_WITH_HEIGHT, GRAZE_AS_NEEDLE, 0.15] {
        a_build_over_a_grazing_edge_leaves_each_side_its_own_loop_at(graze);
    }
}

fn a_build_over_a_grazing_edge_leaves_each_side_its_own_loop_at(graze_in_setbacks: f64) {
    let pick = graze_fixture::pick(
        graze_in_setbacks,
        graze_fixture::RING_SEGMENTS,
        graze_fixture::SIZE,
    );
    let started = Instant::now();
    let built = graze_fixture::build(&pick, graze_fixture::SIZE, BandProfile::Chamfer);
    each_side_its_own_loop(
        &built,
        graze_fixture::SIZE,
        &format!("graze at {graze_in_setbacks:.4} setbacks"),
        started,
    );
}

/// The claims both joint builds make, with C#'s report.
fn each_side_its_own_loop(built: &BandGapResult, size: f64, name: &str, started: Instant) {
    println!(
        "{name}: {:?}, {} loops, {:.2} s",
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

    let doubled = {
        let mesh = built.mesh.as_ref().expect("a refined mesh").borrow();
        coincident_vertices(&mesh, size * 1e-6)
    };
    println!(
        "  coincident vertex pairs in the refined mesh: {}",
        doubled.len()
    );

    assert_eq!(
        built.refusal,
        BandGapRefusal::None,
        "both rims have room to spare at this size: {:?}",
        built.refusal
    );
    assert_eq!(
        built.side_loops.len(),
        4,
        "two chains, two picked sides each"
    );
    assert_eq!(
        built.side_loops.iter().map(|r| r.mixed_loops).sum::<i32>(),
        0,
        "a loop with two owners is the BRIDGE"
    );
    assert!(
        built.side_loops.iter().all(|r| r.owed),
        "a side whose loop is shared with another chain is a BRIDGE"
    );
    assert!(
        built.side_loops.iter().all(|r| r.closed_loops == 1),
        "each picked side of a rim that runs right round wants ONE closed loop: {:?}",
        built
            .side_loops
            .iter()
            .map(|r| (r.chain, r.side, r.closed_loops))
            .collect::<Vec<_>>()
    );
    assert_eq!(
        doubled.len(),
        0,
        "the refinement must not open a crack - coincident unwelded split vertices: {:?}",
        doubled.iter().take(4).collect::<Vec<_>>()
    );
}

/// The mixed edge rule's stated residual: a graze that dips below the snap but not below the chord
/// tolerance. At 0.98 of a setback the chord reads 0.01 Inside at its closest approach, under the
/// 0.025 tolerance (size / 20) every rule splits for - the corner-judged Ridge search wants a rise
/// over highestCorner + tolerance and the mixed edge rule a graze deeper than the tolerance - so the
/// edge's three crossings rest on nothing but the cut's one root.
#[test]
fn a_graze_shallower_than_the_chord_tolerance_still_leaves_each_side_its_own_loop() {
    const UNDER_THE_TOLERANCE: f64 = 0.98;
    the_grazing_edge_crosses_the_line_three_times_at(UNDER_THE_TOLERANCE);
    a_build_over_a_grazing_edge_leaves_each_side_its_own_loop_at(UNDER_THE_TOLERANCE);
}

const DEFECTS: [Defect; 3] = [
    Defect::OnToOn,
    Defect::InsideToInsideNeedle,
    Defect::OnToOnNeedle,
];

fn line_pick(defect: Defect) -> LinePick {
    line_to_line_fixture::pick(
        defect,
        line_to_line_fixture::RING_SEGMENTS,
        line_to_line_fixture::SIZE,
    )
}

/// What the two LINE-TO-LINE shapes are - the ends' sides and the run of field between them -
/// stated so that a change to the fixture that quietly stopped it being the witness's edge fails
/// loudly.
#[test]
fn the_line_to_line_edge_runs_from_one_chains_band_to_the_others() {
    for defect in DEFECTS {
        let size = line_to_line_fixture::SIZE;
        let pick = line_pick(defect);
        let field = line_to_line_fixture::field_of(&pick, size);
        let from = pick.positions[pick.near as usize];
        let to = pick.positions[pick.far as usize];
        let values = stations(&field, from, to);

        // The build's own snap, so "On" here means what VertexFieldReading will say it means.
        const SNAP: f64 = 1e-9;
        let side_near = VertexFieldReading::side_for(values[0], SNAP);
        let side_far = VertexFieldReading::side_for(values[values.len() - 1], SNAP);

        println!(
            "{defect:?}: {} vertices, {} triangles, edge {:.2} sizes",
            pick.positions.len(),
            pick.indices.len(),
            (to - from).length() / size
        );
        println!(
            "  ends read {:.3e} ({side_near:?}) and {:.3e} ({side_far:?}), greatest between {:.2} sizes",
            values[0],
            values[values.len() - 1],
            greatest(&values) / size
        );

        assert!(
            edge_is_in_the_input(&pick.indices, pick.near, pick.far),
            "the bridge has to be ONE edge of the input, or there is nothing for a rule to be asked about"
        );
        assert!(
            greatest(&values) > size,
            "the edge must run right out into the clear between the two bands: {}",
            greatest(&values)
        );

        if defect == Defect::InsideToInsideNeedle {
            assert_eq!(
                side_near,
                VertexSide::Inside,
                "the near end is a vertex OF the hole rim: {}",
                values[0]
            );
            assert_eq!(
                side_far,
                VertexSide::Inside,
                "the far end is a vertex OF the plate rim: {}",
                values[values.len() - 1]
            );
        } else {
            assert_eq!(
                side_near,
                VertexSide::On,
                "the near end stands on chain 0's level line: {}",
                values[0]
            );
            assert_eq!(
                side_far,
                VertexSide::On,
                "the far end stands on chain 1's level line: {}",
                values[values.len() - 1]
            );
        }

        if defect != Defect::OnToOn {
            assert_eq!(
                needles_on_the_edge(&pick),
                2,
                "both triangles on the edge must be under the no-height floor, or a neighbour with \
                 height answers the question and the fixture proves nothing"
            );
        }
    }
}

/// The joint build over an edge that runs from one chain's band to the other's: no refusal, every
/// picked side left exactly one closed loop wholly its own, and no two vertices in the same place.
#[test]
fn a_build_over_a_line_to_line_edge_leaves_each_side_its_own_loop() {
    for defect in DEFECTS {
        let pick = line_pick(defect);
        let started = Instant::now();
        let built =
            line_to_line_fixture::build(&pick, line_to_line_fixture::SIZE, BandProfile::Chamfer);
        each_side_its_own_loop(
            &built,
            line_to_line_fixture::SIZE,
            &format!("{defect:?}"),
            started,
        );
    }
}

/// The line-to-line part is a closed solid, so nothing the build reports is an artefact.
#[test]
fn the_line_to_line_fixture_is_a_closed_solid() {
    for defect in DEFECTS {
        let pick = line_pick(defect);
        is_a_closed_solid(&pick.positions, &pick.indices, &format!("{defect:?}"));
    }
}

/// How many of the input triangles on the bad edge stand under the refinement's no-height floor.
fn needles_on_the_edge(pick: &LinePick) -> i32 {
    // The floor the refinement itself uses at this size: a thousandth of the chord tolerance,
    // which is the size over twenty.
    let floor = 1e-3 * (line_to_line_fixture::SIZE / 20.0);
    let from = pick.positions[pick.near as usize];
    let to = pick.positions[pick.far as usize];
    let mut found = 0;
    for &(a, b, c) in &pick.indices {
        let corners = [a, b, c];
        if !corners.contains(&pick.near) || !corners.contains(&pick.far) {
            continue;
        }

        let third = corners
            .iter()
            .copied()
            .find(|&v| v != pick.near && v != pick.far)
            .expect("a triangle has a third corner");
        if height_over_the_edge(from, to, pick.positions[third as usize]) <= floor {
            found += 1;
        }
    }

    found
}

/// Whether two vertices really are one undivided edge of some input triangle.
fn edge_is_in_the_input(indices: &[(i32, i32, i32)], near: i32, far: i32) -> bool {
    indices.iter().any(|&(a, b, c)| {
        [(a, b), (b, c), (c, a)]
            .iter()
            .any(|&(p, q)| (p == near && q == far) || (p == far && q == near))
    })
}

/// Whether the grazing chord really is one undivided edge of some input triangle.
fn edge_of_the_graze_is_in_the_input(pick: &GrazePick) -> bool {
    edge_is_in_the_input(&pick.indices, pick.rim_end, pick.clear_end)
}

/// How far a point stands off the line through two others - the third corner's height.
fn height_over_the_edge(from: Vector3, to: Vector3, point: Vector3) -> f64 {
    let along = to - from;
    along.cross(point - from).length() / along.length()
}
