//! At an end whose ray is BOUNDED the field is clipped by the end face's plane - `max(value, (p -
//! V) . n)` - so the band stops exactly on that plane. A straight chain along x from 0 to 10,
//! setback 1, its last end's plane through (10, 0, 0) leaning 45 degrees: `n = (1, 0, 1) / sqrt 2`.

use geometry_processing::bevel::level_line::{ChainField, EndRayBound};
use vector_math::dotnet_random::DotNetRandom;
use vector_math::Vector3;

const POINTS: [Vector3; 2] = [Vector3::new(0.0, 0.0, 0.0), Vector3::new(10.0, 0.0, 0.0)];

const SETBACKS: [f64; 2] = [1.0, 1.0];

fn normal() -> Vector3 {
    Vector3::new(1.0, 0.0, 1.0).get_normal()
}

fn bound(reach_per_setback: f64, plane_normal: Vector3) -> EndRayBound {
    EndRayBound {
        reach_per_setback,
        plane_normal,
        ..EndRayBound::default()
    }
}

fn bounded() -> ChainField {
    ChainField::from_polyline_bounded(
        &POINTS,
        false,
        &SETBACKS,
        f64::INFINITY,
        (false, true),
        (None, Some(bound(4.0, normal()))),
    )
    .expect("a valid chain")
}

fn plane(p: Vector3) -> f64 {
    (p - POINTS[1]).dot(normal())
}

fn value(field: &ChainField, p: Vector3) -> f64 {
    field.evaluate(p).expect("no budget").value
}

#[test]
fn past_the_end_plane_a_point_reads_the_plane_and_the_end_parameter() {
    let field = bounded();
    for q in [
        Vector3::new(12.0, 0.0, 0.5),
        Vector3::new(10.5, 0.2, 1.0),
        Vector3::new(13.0, -2.0, 3.0),
    ] {
        let sample = field.evaluate(q).expect("no budget");
        let ball = ((q.y * q.y) + (q.z * q.z)).sqrt() - 1.0;
        let expected = ball.max(plane(q));
        assert!((sample.value - expected).abs() <= 1e-12);
        if plane(q) > ball {
            assert_eq!(sample.parameter, field.length);
        }

        let exhaustive = field.evaluate_exhaustive(q);
        assert_eq!(exhaustive.value, sample.value);
        assert_eq!(exhaustive.parameter, sample.parameter);
    }

    // Short of the plane nothing changes: an unbounded-ray field reads the same.
    let plain =
        ChainField::from_polyline_rayed(&POINTS, false, &SETBACKS, f64::INFINITY, (false, true))
            .expect("a valid chain");
    for q in [
        Vector3::new(5.0, 3.0, 1.0),
        Vector3::new(9.0, 0.5, -2.0),
        Vector3::new(10.2, 0.0, -1.0),
    ] {
        let here = field.evaluate(q).expect("no budget");
        let there = plain.evaluate(q).expect("no budget");
        assert_eq!(here.value, there.value);
        assert_eq!(here.parameter, there.parameter);
    }
}

#[test]
fn the_least_along_a_segment_crossing_the_plane_is_the_clipped_least() {
    // Along x = 12, y = 0: max(|z| - 1, (2 + z) / sqrt 2) is least where the two cross, z = -sqrt 2.
    let field = bounded();
    let least = field
        .min_along_segment(Vector3::new(12.0, 0.0, -3.0), Vector3::new(12.0, 0.0, 3.0))
        .expect("no budget");
    assert!((least.value - (2f64.sqrt() - 1.0)).abs() <= 1e-9);
    assert!((least.point - Vector3::new(12.0, 0.0, -(2f64.sqrt()))).length() <= 1e-6);
}

#[test]
fn the_least_over_a_triangle_crossing_the_plane_is_the_clipped_least() {
    // In y = 0 with x >= 11 the least is at x = 11, where |z| - 1 and (1 + z) / sqrt 2 cross: z = -1, value 0.
    let field = bounded();
    let least = field
        .min_over_triangle(
            Vector3::new(11.0, 0.0, -3.0),
            Vector3::new(11.0, 0.0, 3.0),
            Vector3::new(14.0, 0.0, 0.0),
        )
        .expect("no budget");
    assert!(least.value.abs() <= 1e-9);
    assert!((least.point - Vector3::new(11.0, 0.0, -1.0)).length() <= 1e-6);
}

#[test]
fn the_least_agrees_with_a_dense_sample_of_the_clipped_field() {
    let field = bounded();
    let mut random = DotNetRandom::new(20260923);
    let mut somewhere = || {
        let x = 8.0 + (random.next_double() * 8.0);
        let y = (random.next_double() * 6.0) - 3.0;
        let z = (random.next_double() * 6.0) - 3.0;
        Vector3::new(x, y, z)
    };
    const STEPS: i32 = 400;
    const TRI_STEPS: i32 = 120;
    for _ in 0..20 {
        let a = somewhere();
        let b = somewhere();
        let c = somewhere();
        let mut sampled = f64::INFINITY;
        for i in 0..=STEPS {
            sampled = sampled.min(value(&field, a + ((b - a) * (i as f64 / STEPS as f64))));
        }

        let along = field.min_along_segment(a, b).expect("no budget").value;
        assert!(along <= sampled + 1e-12);
        assert!(along >= sampled - ((b - a).length() / STEPS as f64));

        let mut sampled_tri = f64::INFINITY;
        for i in 0..=TRI_STEPS {
            for j in 0..=(TRI_STEPS - i) {
                let p = a
                    + ((b - a) * (i as f64 / TRI_STEPS as f64))
                    + ((c - a) * (j as f64 / TRI_STEPS as f64));
                sampled_tri = sampled_tri.min(value(&field, p));
            }
        }

        let spacing =
            (b - a).length().max((c - a).length().max((c - b).length())) / TRI_STEPS as f64;
        let over = field.min_over_triangle(a, b, c).expect("no budget").value;
        assert!(over <= sampled_tri + 1e-12);
        assert!(over >= sampled_tri - spacing);
    }
}

#[test]
fn indexed_and_exhaustive_agree_on_a_bounded_field() {
    let mut points = Vec::new();
    let mut setbacks = Vec::new();
    for i in 0..30 {
        let x = i as f64 * 0.5;
        points.push(Vector3::new(x, x.sin(), 0.5 * (2.0 * x).cos()));
        setbacks.push(1.0 + (0.3 * (3.0 * x).sin()));
    }

    let first = (points[0] - points[1]).get_normal() + Vector3::new(0.0, 0.7, 0.4);
    let last = (points[29] - points[28]).get_normal() + Vector3::new(0.0, -0.5, 0.8);
    let field = ChainField::from_polyline_bounded(
        &points,
        false,
        &setbacks,
        f64::INFINITY,
        (true, true),
        (Some(bound(4.0, first)), Some(bound(4.0, last))),
    )
    .expect("a valid chain");
    let mut random = DotNetRandom::new(20260924);
    let mut near = |end: Vector3| {
        let x = (random.next_double() * 8.0) - 4.0;
        let y = (random.next_double() * 8.0) - 4.0;
        let z = (random.next_double() * 8.0) - 4.0;
        end + Vector3::new(x, y, z)
    };
    for i in 0..400 {
        let p = near(if i % 2 == 0 { points[0] } else { points[29] });
        let indexed = field.evaluate(p).expect("no budget");
        let exhaustive = field.evaluate_exhaustive(p);
        assert!((indexed.value - exhaustive.value).abs() <= 1e-9);
        assert!((indexed.parameter - exhaustive.parameter).abs() <= 1e-9);
        assert!((indexed.nearest - exhaustive.nearest).length() < 1e-9);
    }

    for i in 0..40 {
        let end = if i % 2 == 0 { points[0] } else { points[29] };
        let a = near(end);
        let b = near(end);
        let c = near(end);
        let along = field.min_along_segment(a, b).expect("no budget");
        let over = field.min_over_triangle(a, b, c).expect("no budget");
        assert!((value(&field, along.point) - along.value).abs() <= 1e-9);
        assert!((value(&field, over.point) - over.value).abs() <= 1e-9);
        for k in 0..=50 {
            let t = k as f64 / 50.0;
            assert!(along.value <= value(&field, a + ((b - a) * t)) + 1e-12);
            assert!(over.value <= value(&field, a + ((b - a) * t) + ((c - b) * (t * 0.5))) + 1e-12);
        }
    }
}
