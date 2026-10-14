//! A fillet on a straight cube edge is a quarter cylinder about the edge's offset axis - every
//! strip vertex lies on it, and every strip triangle within one chord's sag of it. The cube's faces
//! are two triangles each, so the two rails' stations fall at different places along the edge; a
//! strip whose rows were drawn straight about the mean chain point of a SKEW spoke cut inside the
//! round (0.023 at r 0.2 on rim 0-1-2-3 with edge 0-4) - twisted, faceted shading on a straight
//! round.

use std::collections::BTreeSet;
use std::rc::Rc;

use geometry_processing::bevel::level_line::band_gap_entry_points::{self, BandGapBuildOptions};
use geometry_processing::bevel::level_line::gap_fill::{self, arc_point};
use geometry_processing::bevel::level_line::{BandProfile, WorkBudget};
use vector_math::Vector3;

use crate::cube_edge_union_tests::{link, rim_edges, FACE_RIMS};
use crate::level_line_feature_edges::feature_edges_over;
use crate::level_line_test_geometry::cube_solid;

const FOLD_DEGREES: f64 = 30.0;
const HALF: f64 = 1.0;
const RADIUS: f64 = 0.2;
const SEGMENTS: i32 = 8;

#[test]
fn one_cube_edges_fillet_stays_on_its_cylinder() {
    let edges = BTreeSet::from([(0, 1)]);
    assert!(worst_sag_on_the_cube(&edges) <= sag_allowance());
}

/// A face's rim alone is one closed chain with no corner of three picked edges: the per-chain zip
/// lays it (GapFill), not the corner fill.
#[test]
fn a_face_rim_alone_stays_on_its_cylinders() {
    let edges: BTreeSet<(i32, i32)> = rim_edges(&FACE_RIMS[3]).collect();
    assert!(worst_sag_on_the_cube(&edges) <= sag_allowance());
}

#[test]
fn a_rim_with_an_edge_off_its_corner_stays_on_its_cylinders() {
    let mut edges: BTreeSet<(i32, i32)> = rim_edges(&FACE_RIMS[0]).collect();
    edges.insert((0, 4));
    assert!(worst_sag_on_the_cube(&edges) <= sag_allowance());
}

/// A straight edge split into several collinear segments, as imported meshes often are: no one
/// segment is a whole leg between mitres, so its rows once went back to the midpoint drawing and
/// cut inside the round. The splits are uneven so no row lands square across a split point. The
/// pick is the bottom face's whole rim: a closed chain is laid by the per-chain zip (GapFill), whose
/// rows may be skew; an open chain's fill inserts each rail's stations on the other, so its rows
/// are square already.
#[test]
fn a_collinear_split_edge_stays_on_its_cylinder() {
    let (positions, indices, mut edges) = cube_with_split_edge(&[0.17, 0.4, 0.55, 0.8]);
    edges.extend([(1, 2), (2, 3), (0, 3)]);
    let lines = vec![
        (positions[0], positions[1]),
        (positions[1], positions[2]),
        (positions[2], positions[3]),
        (positions[3], positions[0]),
    ];
    assert!(worst_sag(&positions, &indices, &edges, &lines) <= sag_allowance());
}

type SplitCube = (Vec<Vector3>, Vec<(i32, i32, i32)>, BTreeSet<(i32, i32)>);

/// The cube with its edge 0-1 split at the given fractions: the bottom face's triangle on that edge
/// becomes a fan from vertex 2, the front face's a fan from vertex 5, so the edge is a run of
/// collinear segments and everything else is unchanged. The picked edges are the split edge's
/// segments.
fn cube_with_split_edge(fractions: &[f64]) -> SplitCube {
    let (mut positions, cube_indices) = cube_solid(HALF);
    let mut run = vec![0];
    for &f in fractions {
        run.push(positions.len() as i32);
        positions.push(positions[0] + ((positions[1] - positions[0]) * f));
    }

    run.push(1);
    let mut indices: Vec<(i32, i32, i32)> = cube_indices
        .into_iter()
        .filter(|&t| t != (0, 2, 1) && t != (0, 1, 5))
        .collect();
    let mut edges = BTreeSet::new();
    for i in 0..run.len() - 1 {
        indices.push((run[i], 2, run[i + 1]));
        indices.push((run[i], run[i + 1], 5));
        let edge = if run[i] < run[i + 1] {
            (run[i], run[i + 1])
        } else {
            (run[i + 1], run[i])
        };
        edges.insert(edge);
    }

    (positions, indices, edges)
}

/// How far a point of the strip may stand inside the true round: the arc's rows are drawn by
/// `GapFill.ArcPoint` at even steps of its rational parameter, which are not even in angle, so the
/// allowance is the chord sag r(1 - cos(step / 2)) of the WIDEST step (a triangle's centroid lies
/// inside the chord's midpoint sag). The arc's own vertices must lie on the circle to rounding.
fn sag_allowance() -> f64 {
    let p = Vector3::new(RADIUS, 0.0, 0.0);
    let q = Vector3::new(0.0, RADIUS, 0.0);
    let mut widest = 0.0_f64;
    for s in 0..SEGMENTS {
        let a = arc_point(p, Vector3::ZERO, q, s as f64 / SEGMENTS as f64)
            - Vector3::new(RADIUS, RADIUS, 0.0);
        let b = arc_point(p, Vector3::ZERO, q, (s + 1) as f64 / SEGMENTS as f64)
            - Vector3::new(RADIUS, RADIUS, 0.0);
        widest = widest.max(
            (a.dot(b) / (a.length() * b.length()))
                .clamp(-1.0, 1.0)
                .acos(),
        );
    }

    (RADIUS * (1.0 - (widest / 2.0).cos())) + 1e-9
}

/// -1, 0 or 1 (zero for either zero, unlike `f64::signum`).
fn sign(x: f64) -> i32 {
    if x > 0.0 {
        1
    } else if x < 0.0 {
        -1
    } else {
        0
    }
}

/// The largest amount any vertex or triangle centroid of the built part stands off its picked
/// edge's fillet cylinder, over the points inside that edge's band wedge and more than r from both
/// of its ends (the ends are corner patches, not cylinder); a centroid counts only when its whole
/// triangle is.
fn worst_sag_on_the_cube(edges: &BTreeSet<(i32, i32)>) -> f64 {
    let (positions, indices) = cube_solid(HALF);
    let lines: Vec<(Vector3, Vector3)> = edges
        .iter()
        .map(|&(low, high)| (positions[low as usize], positions[high as usize]))
        .collect();
    worst_sag(&positions, &indices, edges, &lines)
}

/// The same measure on any box-faced part: `edges` are picked, and each of `lines` - a straight
/// cube edge, however many segments it is split into - is measured as one cylinder.
fn worst_sag(
    positions: &[Vector3],
    indices: &[(i32, i32, i32)],
    edges: &BTreeSet<(i32, i32)>,
    lines: &[(Vector3, Vector3)],
) -> f64 {
    let feature = feature_edges_over(positions, indices, FOLD_DEGREES);
    let (chains, closed) = link(positions, indices, edges);
    let options = BandGapBuildOptions {
        chord_tolerance: 0.0,
        snap: 1e-9,
        known_room: None,
        budget: Some(Rc::new(
            WorkBudget::new(WorkBudget::default_limit(indices.len() as i32))
                .expect("a positive limit"),
        )),
        sharp_turn_degrees: FOLD_DEGREES,
        fillet_segments: SEGMENTS,
        ..BandGapBuildOptions::default()
    };
    let gap = band_gap_entry_points::build_pick(
        positions,
        indices,
        &chains,
        &closed,
        Some(feature),
        RADIUS,
        BandProfile::Fillet,
        &options,
    )
    .expect("the pick is well formed");
    assert!(gap.built(), "band {:?}", gap.refusal);

    let fill = gap_fill::fill(&gap, BandProfile::Fillet, SEGMENTS).expect("the fill runs");
    assert!(fill.filled(), "fill {:?}", fill.refusal);

    let mesh = fill.mesh.as_ref().expect("a filled mesh").borrow();
    // Each point with the vertices it stands for: a centroid is measured only where its whole
    // triangle lies clear of the ends, since a cell reaching a corner's mitre row is corner patch,
    // not cylinder.
    let mut points: Vec<(Vector3, Vec<Vector3>)> = Vec::new();
    for t in 0..mesh.triangle_count() {
        if mesh.is_removed(t) {
            continue;
        }

        let tri = mesh.triangles()[t as usize];
        let a = mesh.position(tri.a);
        let b = mesh.position(tri.b);
        let c = mesh.position(tri.c);
        points.push((a, vec![a]));
        points.push((b, vec![b]));
        points.push((c, vec![c]));
        points.push(((a + b + c) / 3.0, vec![a, b, c]));
    }

    let mut worst = 0.0_f64;
    for &(start, end) in lines {
        let length = (end - start).length();
        let along = (end - start) / length;
        let perpendicular: Vec<usize> = (0..3).filter(|&i| along[i].abs() < 0.5).collect();

        for (p, span) in &points {
            let p = *p;
            let s = (p - start).dot(along);
            if span.iter().any(|&v| {
                (v - start).dot(along) < RADIUS || (v - start).dot(along) > length - RADIUS
            }) || perpendicular
                .iter()
                .any(|&i| sign(p[i]) != sign(start[i]) || p[i].abs() <= HALF - RADIUS + 1e-9)
            {
                continue;
            }

            let distance = perpendicular
                .iter()
                .map(|&i| {
                    let centre = start[i] - (sign(start[i]) as f64 * RADIUS);
                    (p[i] - centre) * (p[i] - centre)
                })
                .sum::<f64>()
                .sqrt();
            if RADIUS - distance > worst {
                worst = RADIUS - distance;
                println!(
                    "  edge {start:?}-{end:?}: {p:?} at s {s:.4} stands {:.3e} inside",
                    RADIUS - distance
                );
            }

            // Outside the round is a bulge, not a sag - equally a wrong shape.
            worst = worst.max(distance - RADIUS);
        }
    }

    println!(
        "worst sag {worst:.3e} against the allowance {:.3e}",
        sag_allowance()
    );
    worst
}
