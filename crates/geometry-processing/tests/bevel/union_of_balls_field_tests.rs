//! The field is the signed distance to the UNION OF BALLS round the chain - `min over chain points
//! c of (|q - c| - setback(c))` - exactly 1-Lipschitz even where the setback varies, so a bounded
//! search closes on its tolerance.

use std::rc::Rc;

use geometry_processing::bevel::level_line::{ChainField, LevelLineWork, ScalarField, WorkBudget};
use vector_math::dotnet_random::DotNetRandom;
use vector_math::{cs_math, Vector3};

/// A closed octagon of radius 10 in Z = 0 whose setback alternates `even` and `odd` round it.
fn alternating_octagon(even: f64, odd: f64) -> ChainField {
    let mut points = Vec::new();
    let mut setbacks = Vec::new();
    for i in 0..8 {
        let angle = i as f64 * std::f64::consts::PI / 4.0;
        points.push(Vector3::new(10.0 * angle.cos(), 10.0 * angle.sin(), 0.0));
        setbacks.push(if i % 2 == 0 { even } else { odd });
    }

    ChainField::from_polyline(&points, true, &setbacks).expect("a valid chain")
}

/// The minimum over a fine sampling of the chain - the definition itself, read by brute force.
fn brute_force_value(field: &ChainField, q: Vector3, samples: i32) -> f64 {
    let mut best = f64::INFINITY;
    for i in 0..=samples {
        let parameter = field.length * i as f64 / samples as f64;
        let c = field.point_at(parameter);
        best = cs_math::min_f64(best, (q - c).length() - field.setback_at(parameter));
    }

    best
}

fn somewhere_about(random: &mut DotNetRandom) -> Vector3 {
    let x = (random.next_double() * 40.0) - 20.0;
    let y = (random.next_double() * 40.0) - 20.0;
    let z = (random.next_double() * 20.0) - 10.0;
    Vector3::new(x, y, z)
}

fn value(field: &ChainField, point: Vector3) -> f64 {
    field.value_at(point).expect("no budget")
}

#[test]
fn the_field_is_one_lipschitz() {
    let field = alternating_octagon(1.0, 3.0);
    let mut random = DotNetRandom::new(20260918);
    let mut worst = 0.0;

    for _ in 0..2000 {
        let p = somewhere_about(&mut random);
        let q = somewhere_about(&mut random);
        let slack = (value(&field, p) - value(&field, q)).abs() - (p - q).length();
        worst = cs_math::max_f64(worst, slack);
    }

    assert!(
        worst <= 1e-9,
        "a minimum of 1-Lipschitz functions is 1-Lipschitz, so no two readings differ by more than the distance between them"
    );
}

#[test]
fn the_closed_form_is_the_minimum_over_the_whole_chain() {
    let field = alternating_octagon(1.0, 3.0);
    let mut random = DotNetRandom::new(7761);

    for _ in 0..8 {
        let q = somewhere_about(&mut random);

        assert!(
            (value(&field, q) - brute_force_value(&field, q, 4_000_000)).abs() <= 1e-9,
            "the field IS the minimum over every chain point of distance less that point's setback"
        );
    }
}

#[test]
fn the_sample_names_the_chain_point_its_value_was_read_at() {
    let field = alternating_octagon(1.0, 3.0);
    let mut random = DotNetRandom::new(991);

    for _ in 0..200 {
        let q = somewhere_about(&mut random);

        let sample = field.evaluate(q).expect("no budget");
        assert!(((q - sample.nearest).length() - sample.distance).abs() <= 1e-9);
        assert!(
            ((sample.distance - field.setback_at(sample.parameter)) - sample.value).abs() <= 1e-9
        );
        assert!(
            (field.point_at(sample.parameter) - sample.nearest)
                .length()
                .abs()
                <= 1e-9
        );
    }
}

#[test]
fn a_bounded_search_closes_when_the_setback_varies() {
    let field = alternating_octagon(1.0, 3.0);

    // An edge well inside the band, crossing the place two chain stretches of different setback
    // compete.
    let from = Vector3::new(6.5, -3.0, 0.0);
    let to = Vector3::new(6.5, 3.0, 0.0);

    // Measured on a budget of its own: a WorkBudget is put in force per thread and its spent is
    // this call's alone.
    let budget = Rc::new(WorkBudget::new(1_000_000).expect("a positive limit"));
    let peak = {
        let _work = LevelLineWork::begin(Some(budget.clone()));
        field
            .max_along_segment(from, to, 0.03)
            .expect("within budget")
    };

    let readings = budget.spent.get();

    println!("MaxAlongSegment readings {readings}, peak {}", peak.value);

    assert!(
        readings < 20000,
        "a 1-Lipschitz bound closes on the tolerance; the old bound gave away the setback spread and ran to the depth limit"
    );
    assert!(
        peak.value >= cs_math::max_f64(value(&field, from), value(&field, to)),
        "the greatest value is at least what the ends read"
    );
}

#[test]
fn readings_are_invariant_under_a_rigid_motion() {
    let moved = |p: Vector3| {
        let turned = Vector3::new(
            (p.x * 0.7f64.cos()) - (p.y * 0.7f64.sin()),
            (p.x * 0.7f64.sin()) + (p.y * 0.7f64.cos()),
            p.z,
        );
        turned + Vector3::new(-13.5, 8.25, 4.75)
    };

    let here = alternating_octagon(1.0, 3.0);
    let mut moved_points = Vec::new();
    let mut setbacks = Vec::new();
    for (i, &point) in here.points().iter().enumerate() {
        moved_points.push(moved(point));
        setbacks.push(if i % 2 == 0 { 1.0 } else { 3.0 });
    }

    let there = ChainField::from_polyline(&moved_points, true, &setbacks).expect("a valid chain");
    let mut random = DotNetRandom::new(31337);

    for _ in 0..500 {
        let q = somewhere_about(&mut random);

        assert!(
            (value(&there, moved(q)) - value(&here, q)).abs() <= 1e-9,
            "the field is a property of the chain and its setbacks, not of where they stand"
        );
    }
}

#[test]
fn a_uniform_setback_reads_as_plain_distance_less_the_setback() {
    let field = alternating_octagon(2.0, 2.0);
    let mut random = DotNetRandom::new(505);

    for _ in 0..50 {
        let q = somewhere_about(&mut random);

        let sample = field.evaluate(q).expect("no budget");
        assert!(
            (sample.value - brute_force_value(&field, q, 200000)).abs() <= 1e-6,
            "with one setback everywhere the union of balls is the plain distance less that setback"
        );
        assert!((sample.value - (sample.distance - 2.0)).abs() <= 1e-12);
    }
}
