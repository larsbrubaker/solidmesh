//! The point query's cutoff walk (`ChainField::nearest_to`) answers bit for bit what the reference
//! walk in segment order (`ChainField::visit_nearest_in_index_order`) answers: over the Tea Scooper
//! inner fillet's chains (`teascooper-inner-pick0`, the case that made the query worth speeding up)
//! and the near ties the tie rule decides - a regular polygon read on its axis, a point equidistant
//! from two segments, points on a mitre's bisector, and ladders of rungs a few tie widths apart
//! whose boxes bound their readings exactly, the one case here a cutoff guard too small fails. With `LEVELLINE_COST=1`
//! (`level_line_cost_gate.rs`) the Tea Scooper case also says how many segments each walk searched.
//! Each test's cases run as a loop inside it.

use std::time::Instant;

use solidmesh::bevel::level_line::{ChainField, ChainFieldNearest};
use solidmesh_math::seeded_random::SeededRandom;
use solidmesh_math::Vector3;

use crate::level_line_cost_gate::{self as gate, say};
use crate::level_line_fixture::LevelLineFixture;

/// Both walks' answers at `point`, asserted identical bit for bit, as (reference, cutoff).
fn both_walks(
    field: &ChainField,
    point: Vector3,
    case: &str,
) -> (ChainFieldNearest, ChainFieldNearest) {
    let mut reference = ChainFieldNearest::start(field.points[0]);
    field.visit_nearest_in_index_order(0, point, &mut reference);
    let fast = field.nearest_to(point);
    let bits = |n: &ChainFieldNearest| {
        [
            n.best.to_bits(),
            n.parameter.to_bits(),
            n.distance.to_bits(),
            n.nearest.x.to_bits(),
            n.nearest.y.to_bits(),
            n.nearest.z.to_bits(),
        ]
    };
    assert_eq!(
        bits(&fast),
        bits(&reference),
        "{case} at {point:?}: cutoff walk {fast:?}, reference walk {reference:?}"
    );
    (reference, fast)
}

/// A random offset up to `reach` along each axis.
fn jitter(random: &mut SeededRandom, reach: f64) -> Vector3 {
    let mut component = || ((random.next_double() * 2.0) - 1.0) * reach;
    Vector3::new(component(), component(), component())
}

/// The Tea Scooper fixture's chains as fields the way a pick reads them - each open chain at the
/// fixture's size and bend threshold, with and without end rays - plus the closed loop its first
/// two chains make, as (name, field).
fn tea_scooper_fields(fixture: &LevelLineFixture) -> Vec<(String, ChainField)> {
    let points_of = |chain: &[i32]| -> Vec<Vector3> {
        chain
            .iter()
            .map(|&k| fixture.positions[k as usize])
            .collect()
    };
    let mut fields = Vec::new();
    for (c, chain) in fixture.chains.iter().enumerate() {
        let points = points_of(chain);
        let setbacks = vec![fixture.size; points.len()];
        let mitred = ChainField::from_polyline_sharp(
            &points,
            fixture.closed[c],
            &setbacks,
            fixture.sharp_turn_degrees,
        )
        .expect("a chain");
        fields.push((format!("chain {c}"), mitred));
        if !fixture.closed[c] {
            let rayed = ChainField::from_polyline_bounded(
                &points,
                false,
                &setbacks,
                fixture.sharp_turn_degrees,
                (true, true),
                (None, None),
            )
            .expect("a chain");
            fields.push((format!("chain {c} with end rays"), rayed));
        }
    }

    // The first chain ends where the second starts and the second ends where the first starts.
    let first = &fixture.chains[0];
    let second = &fixture.chains[1];
    assert_eq!(first.last(), second.first());
    assert_eq!(second.last(), first.first());
    let mut loop_indices = first.clone();
    loop_indices.extend_from_slice(&second[1..second.len() - 1]);
    let points = points_of(&loop_indices);
    let setbacks = vec![fixture.size; points.len()];
    let closed =
        ChainField::from_polyline_sharp(&points, true, &setbacks, fixture.sharp_turn_degrees)
            .expect("a loop");
    fields.push(("the first two chains' loop".to_string(), closed));
    fields
}

/// The points the Tea Scooper case asks: every other mesh vertex, every chain point, a jittered
/// point near each chain point and each segment's middle.
fn tea_scooper_points(fixture: &LevelLineFixture, field: &ChainField) -> Vec<Vector3> {
    let mut random = SeededRandom::new(20261120);
    let mut points: Vec<Vector3> = fixture.positions.iter().step_by(2).copied().collect();
    let n = field.points.len();
    for (i, &p) in field.points.iter().enumerate() {
        points.push(p);
        points.push(p + jitter(&mut random, 2.0 * fixture.size));
        points.push((p + field.points[(i + 1) % n]) * 0.5);
    }

    points
}

#[test]
fn the_cutoff_walk_answers_what_the_reference_walk_answers_round_the_tea_scooper() {
    let fixture = LevelLineFixture::load("teascooper-inner-pick0");
    for (name, field) in tea_scooper_fields(&fixture) {
        for point in tea_scooper_points(&fixture, &field) {
            both_walks(&field, point, &name);
        }
    }
}

/// How many segments each walk searches per query round the Tea Scooper, over the mesh vertices a
/// band could hold (reading under twice the size), with each walk's time. Runs with
/// `LEVELLINE_COST=1`.
#[test]
fn the_cutoff_walk_searches_fewer_segments_round_the_tea_scooper() {
    if !gate::asked() {
        return;
    }

    let fixture = LevelLineFixture::load("teascooper-inner-pick0");
    for (name, field) in tea_scooper_fields(&fixture) {
        let near: Vec<Vector3> = fixture
            .positions
            .iter()
            .copied()
            .filter(|&p| field.nearest_to(p).best < 2.0 * fixture.size)
            .collect();
        let (mut reference_searched, mut fast_searched) = (0i64, 0i64);
        for &point in &near {
            let (reference, fast) = both_walks(&field, point, &name);
            reference_searched += i64::from(reference.searched);
            fast_searched += i64::from(fast.searched);
        }

        let started = Instant::now();
        for &point in &near {
            let mut reference = ChainFieldNearest::start(field.points[0]);
            field.visit_nearest_in_index_order(0, point, &mut reference);
            std::hint::black_box(reference);
        }
        let reference_seconds = started.elapsed().as_secs_f64();
        let started = Instant::now();
        for &point in &near {
            std::hint::black_box(field.nearest_to(point));
        }
        let fast_seconds = started.elapsed().as_secs_f64();

        let queries = near.len().max(1) as f64;
        say(&format!(
            "{name} ({} segments, {} queries): segments searched per query {:.1} reference, \
             {:.1} cutoff; {:.2} us vs {:.2} us per query",
            field.segment_count(),
            near.len(),
            reference_searched as f64 / queries,
            fast_searched as f64 / queries,
            1e6 * reference_seconds / queries,
            1e6 * fast_seconds / queries,
        ));
    }
}

/// A regular 48-gon - the design's cylinders - read on and about its axis, where every segment
/// reads within the tie allowance of every other: the cutoff must leave the tie to the same
/// segment the reference walk gives it to, mitred or not, with a constant or a varying setback.
#[test]
fn the_cutoff_walk_keeps_the_reference_walks_tie_on_a_polygons_axis() {
    let centre = Vector3::new(31.0, -72.5, 25.0);
    let sides = 48;
    let radius = 3.25;
    let points: Vec<Vector3> = (0..sides)
        .map(|i| {
            let angle = (i as f64 / sides as f64) * std::f64::consts::TAU;
            centre + Vector3::new(radius * angle.cos(), 0.0, radius * angle.sin())
        })
        .collect();
    let constant = vec![1.5; sides];
    let varying: Vec<f64> = (0..sides).map(|i| 1.0 + (0.01 * (i % 3) as f64)).collect();

    let plain = ChainField::from_polyline(&points, true, &constant).expect("a polygon");
    // Every axis point is a near tie: all segments read within the tie allowance of the best.
    let axis = centre + Vector3::new(0.0, 0.7, 0.0);
    let readings: Vec<f64> = (0..sides)
        .map(|i| plain.least_over_segment_from_point(i, axis).0)
        .collect();
    let least = readings.iter().copied().fold(f64::INFINITY, f64::min);
    let tied = readings
        .iter()
        .filter(|&&r| r - least <= ChainField::TIE_RELATIVE * (1.0 + least.abs()))
        .count();
    assert!(
        tied > sides / 2,
        "only {tied} of {sides} segments tie on the axis"
    );

    let fields = [
        ("plain", plain),
        (
            "mitred",
            ChainField::from_polyline_sharp(&points, true, &constant, 5.0).expect("a polygon"),
        ),
        (
            "varying setback",
            ChainField::from_polyline(&points, true, &varying).expect("a polygon"),
        ),
    ];
    let mut random = SeededRandom::new(48);
    for (name, field) in &fields {
        for step in -40..=40 {
            let along = centre + Vector3::new(0.0, step as f64 * 0.1, 0.0);
            both_walks(field, along, name);
            for scale in [1e-15, 1e-13, 1e-11, 1e-9, 1e-6] {
                both_walks(field, along + jitter(&mut random, scale), name);
            }
        }
    }
}

/// A point equidistant from two segments - across a U's two legs, in the outer quadrant of an L's
/// corner - and on a mitre's bisector, where its two rays tie.
#[test]
fn the_cutoff_walk_keeps_the_reference_walks_tie_between_segments_and_rays() {
    let u = [
        Vector3::new(0.0, 0.0, 0.0),
        Vector3::new(10.0, 0.0, 0.0),
        Vector3::new(10.0, 2.0, 0.0),
        Vector3::new(0.0, 2.0, 0.0),
    ];
    let u_field = ChainField::from_polyline(&u, false, &[0.5; 4]).expect("a U");
    let u_mitred = ChainField::from_polyline_sharp(&u, false, &[0.5; 4], 20.0).expect("a U");
    let mut random = SeededRandom::new(2);
    for field in [&u_field, &u_mitred] {
        for step in 0..=100 {
            let between = Vector3::new(step as f64 * 0.11, 1.0, 0.0);
            both_walks(field, between, "between the U's legs");
            both_walks(
                field,
                between + jitter(&mut random, 1e-13),
                "between the U's legs",
            );
        }
    }

    let l = [
        Vector3::new(0.0, 0.0, 0.0),
        Vector3::new(10.0, 0.0, 0.0),
        Vector3::new(10.0, 10.0, 0.0),
    ];
    let plain = ChainField::from_polyline(&l, false, &[1.0; 3]).expect("an L");
    let mitred = ChainField::from_polyline_sharp(&l, false, &[1.0; 3], 20.0).expect("an L");
    let rayed = ChainField::from_polyline_bounded(
        &l,
        false,
        &[1.0, 1.5, 2.0],
        20.0,
        (true, true),
        (None, None),
    )
    .expect("an L");
    for (name, field) in [
        ("plain L", &plain),
        ("mitred L", &mitred),
        ("rayed L", &rayed),
    ] {
        for step in 0..=60 {
            let out = step as f64 * 0.1;
            // The outer quadrant's diagonal, the mitre's bisector and the inner quadrant's.
            for at in [
                Vector3::new(10.0 + out, -out, 0.0),
                Vector3::new(10.0 - out, out, 0.0),
                Vector3::new(10.0 + out, -out, 0.5),
            ] {
                both_walks(field, at, name);
                both_walks(field, at + jitter(&mut random, 1e-12), name);
            }

            // Past each end, where an end ray competes with the segment it continues.
            both_walks(field, Vector3::new(-out, 0.3, 0.0), name);
            both_walks(field, Vector3::new(10.3, 10.0 + out, 0.0), name);
        }
    }
}

/// Rungs parallel to x at heights `heights` over the origin, joined by connectors 30 units off, so
/// each rung's box is flat and its bound is its reading within rounding: the cutoff alone decides
/// which rungs the walk sees, and the rungs a few tie widths apart make a cascade of near ties.
fn ladder(heights: &[f64], setback: f64) -> ChainField {
    let mut points = Vec::new();
    for (i, &height) in heights.iter().enumerate() {
        let (from, to) = if i % 2 == 0 { (-1.0, 1.0) } else { (1.0, -1.0) };
        points.push(Vector3::new(from, height, 0.0));
        points.push(Vector3::new(to, height, 0.0));
        points.push(Vector3::new(to, height, 30.0));
    }

    let setbacks = vec![setback; points.len()];
    ChainField::from_polyline(&points, false, &setbacks).expect("a ladder")
}

/// Three rungs, each nearer than the last by under the tie allowance taken from the one before: the
/// reference walk keeps the first rung's reading through the cascade, and a cutoff at the least
/// reading would skip it and keep another.
#[test]
fn the_cutoff_walk_keeps_the_first_of_a_cascade_of_near_ties() {
    let field = ladder(&[1.0 + 4e-12, 1.0 + 1e-12, 1.0], 0.0);
    both_walks(&field, Vector3::ZERO, "three rungs");
}

/// Random ladders of 2 to 13 rungs within six tie widths, with and without a setback, read from
/// just off the origin: with a guard that does not grow with the number of readings, some of these
/// answer differently.
#[test]
fn the_cutoff_walk_keeps_the_reference_walks_ties_on_random_ladders() {
    let mut random = SeededRandom::new(7);
    for case in 0..20_000 {
        let rungs = 2 + (random.next_double() * 12.0) as usize;
        let base = 0.5 + (random.next_double() * 3.0);
        let tie = ChainField::TIE_RELATIVE * (1.0 + base);
        let heights: Vec<f64> = (0..rungs)
            .map(|_| base + (random.next_double() * 6.0 * tie))
            .collect();
        let setback = if case % 2 == 0 { 0.0 } else { 0.25 };
        let field = ladder(&heights, setback);
        let at = Vector3::new(
            (random.next_double() - 0.5) * 1e-9,
            0.0,
            (random.next_double() - 0.5) * 1e-9,
        );
        both_walks(&field, at, &format!("ladder case {case} {heights:?}"));
    }
}
