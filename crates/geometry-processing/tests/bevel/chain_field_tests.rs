//! The distance field the level line is the zero set of - exact, deterministic and continuous. Each
//! test's cases run as a loop inside it.

use std::time::Instant;

use geometry_processing::bevel::level_line::{chain_setback, ChainField, ScalarField};
use vector_math::seeded_random::SeededRandom;
use vector_math::Vector3;

/// A straight chain along X, so the right answer can be written down by hand.
fn straight_chain(setback: f64) -> ChainField {
    ChainField::from_polyline(
        &[Vector3::new(0.0, 0.0, 0.0), Vector3::new(10.0, 0.0, 0.0)],
        false,
        &[setback, setback],
    )
    .expect("a valid chain")
}

/// A closed unit-ten square in the Z = 0 plane, walked counter-clockwise from the origin.
fn closed_square(setbacks: &[f64]) -> ChainField {
    ChainField::from_polyline(&square_points(), true, setbacks).expect("a valid chain")
}

fn square_points() -> [Vector3; 4] {
    [
        Vector3::new(0.0, 0.0, 0.0),
        Vector3::new(10.0, 0.0, 0.0),
        Vector3::new(10.0, 10.0, 0.0),
        Vector3::new(0.0, 10.0, 0.0),
    ]
}

fn near(actual: f64, expected: f64, within: f64, why: &str) {
    assert!(
        (actual - expected).abs() <= within,
        "{why}: expected {expected} within {within}, got {actual}"
    );
}

#[test]
fn distance_and_parameter_are_exact_on_a_straight_chain() {
    let rows = [
        (3.0, 4.0, 4.0, 3.0, "square off the middle of the chain: the distance is the offset and the parameter is the foot of it"),
        (0.0, 1.5, 1.5, 0.0, "straight out from the chain's start"),
        (10.0, -2.5, 2.5, 10.0, "straight out from the chain's end, from the other side"),
        (7.0, 0.0, 0.0, 7.0, "a point ON the chain reads zero distance and its own parameter"),
    ];
    for (x, y, expected_distance, expected_parameter, why) in rows {
        let sample = straight_chain(2.0)
            .evaluate(Vector3::new(x, y, 0.0))
            .expect("no budget");

        near(sample.distance, expected_distance, 1e-12, why);
        near(sample.parameter, expected_parameter, 1e-12, why);
        near(
            sample.nearest.x,
            expected_parameter,
            1e-12,
            "the nearest point is the foot of the perpendicular",
        );
    }
}

#[test]
fn past_the_ends_of_an_open_chain_the_end_point_is_nearest() {
    let rows = [
        (
            -3.0,
            4.0,
            5.0,
            0.0,
            "beyond the start, the start point is nearest and the parameter pins at 0",
        ),
        (
            14.0,
            3.0,
            5.0,
            10.0,
            "beyond the end, the end point is nearest and the parameter pins at the length",
        ),
        (
            -5.0,
            0.0,
            5.0,
            0.0,
            "straight back along the chain's own line, before it starts",
        ),
        (
            12.0,
            0.0,
            2.0,
            10.0,
            "straight on along the chain's own line, after it ends",
        ),
    ];
    for (x, y, expected_distance, expected_parameter, why) in rows {
        let sample = straight_chain(2.0)
            .evaluate(Vector3::new(x, y, 0.0))
            .expect("no budget");

        near(sample.distance, expected_distance, 1e-12, why);
        near(sample.parameter, expected_parameter, 1e-12, why);
    }
}

#[test]
fn every_point_in_a_corners_outer_quadrant_reads_the_corners_parameter() {
    let field = closed_square(&[1.0, 1.0, 1.0, 1.0]);
    let corner = Vector3::new(10.0, 0.0, 0.0);
    const CORNER_PARAMETER: f64 = 10.0;

    let mut angle = 0.0;
    while angle <= 90.0 {
        let radians = angle * std::f64::consts::PI / 180.0;

        // The outer quadrant at (10, 0) spans from straight down (-Y) round to straight out (+X).
        let mut radius = 0.25;
        while radius <= 4.0 {
            let point = corner + Vector3::new(radians.sin() * radius, -radians.cos() * radius, 0.0);
            let sample = field.evaluate(point).expect("no budget");

            near(
                sample.parameter,
                CORNER_PARAMETER,
                1e-9,
                &format!("at {angle} degrees and radius {radius} the corner itself is nearest, so the whole fan reads one parameter"),
            );
            near(
                sample.distance,
                radius,
                1e-9,
                "and the distance is just the radius of the fan",
            );
            radius += 0.75;
        }

        angle += 5.0;
    }
}

#[test]
fn a_tie_goes_to_the_lower_parameter_and_stays_there() {
    let field = closed_square(&[1.0, 1.0, 1.0, 1.0]);
    let centre = Vector3::new(5.0, 5.0, 0.0);

    let first = field.evaluate(centre).expect("no budget");

    near(
        first.distance,
        5.0,
        1e-12,
        "the centre is five from every side",
    );
    near(
        first.parameter,
        5.0,
        1e-12,
        "all four sides answer five, and the tie is broken by the lowest parameter - the middle of the first side",
    );

    for _ in 0..8 {
        let repeat = field.evaluate(centre).expect("no budget");
        assert_eq!(
            repeat.parameter, first.parameter,
            "the rule is a rule, not a race: the same point always reads the same parameter"
        );
    }
}

#[test]
fn setback_is_linear_along_each_edge_and_continuous_round_the_wrap() {
    let field = closed_square(&[1.0, 2.0, 3.0, 4.0]);

    near(field.length, 40.0, 1e-12, "length");
    near(field.max_setback, 4.0, 1e-12, "max setback");

    // Linear along the first edge, from 1 at parameter 0 to 2 at parameter 10.
    let mut t = 0.0;
    while t <= 1.0 {
        near(
            field.setback_at(t * 10.0),
            1.0 + t,
            1e-12,
            "the setback runs straight between the two points' setbacks",
        );
        t += 0.125;
    }

    // And round the wrap: the last edge runs from 4 back to the first point's 1.
    let mut t = 0.0;
    while t <= 1.0 {
        near(
            field.setback_at(30.0 + (t * 10.0)),
            4.0 + (t * (1.0 - 4.0)),
            1e-12,
            "the closing edge is an edge like any other",
        );
        t += 0.125;
    }

    let just_before_the_wrap = field.setback_at(40.0 - 1e-7);
    near(
        just_before_the_wrap,
        1.0,
        1e-6,
        "arriving back at the start must read what leaving it read - no step at the wrap",
    );
    near(
        field.setback_at(40.0),
        field.setback_at(0.0),
        1e-12,
        "a closed chain's parameter wraps rather than clamping",
    );
}

#[test]
fn the_zero_set_on_a_straight_edge_is_the_parallel_line_at_the_setback() {
    const SETBACK: f64 = 2.0;
    let field = straight_chain(SETBACK);
    let value = |p: Vector3| field.evaluate(p).expect("no budget").value;

    let mut x = 1.0;
    while x <= 9.0 {
        near(
            value(Vector3::new(x, SETBACK, 0.0)),
            0.0,
            1e-12,
            "exactly a setback out from the chain is the level line",
        );
        near(
            value(Vector3::new(x, SETBACK - 0.5, 0.0)),
            -0.5,
            1e-12,
            "inside the boundary the value is negative by how far inside",
        );
        near(
            value(Vector3::new(x, SETBACK + 1.25, 0.0)),
            1.25,
            1e-12,
            "outside it is positive by how far outside",
        );

        // Out of plane too - the field is a distance in space, not in the face's plane.
        near(
            value(Vector3::new(x, 0.0, SETBACK)),
            0.0,
            1e-12,
            "the boundary is a cylinder of the setback about the chain, whichever side is asked",
        );
        x += 1.0;
    }
}

fn random_point(
    random: &mut SeededRandom,
    xy: f64,
    xy_offset: f64,
    z: f64,
    z_offset: f64,
) -> Vector3 {
    let x = (random.next_double() * xy) - xy_offset;
    let y = (random.next_double() * xy) - xy_offset;
    let z = (random.next_double() * z) - z_offset;
    Vector3::new(x, y, z)
}

#[test]
fn the_field_is_one_lipschitz_in_distance() {
    let field = closed_square(&[1.0, 1.0, 1.0, 1.0]);
    let mut random = SeededRandom::new(20260918);

    for _ in 0..2000 {
        let a = random_point(&mut random, 30.0, 10.0, 20.0, 10.0);
        let b = random_point(&mut random, 30.0, 10.0, 20.0, 10.0);

        let change = (field.evaluate(a).expect("no budget").distance
            - field.evaluate(b).expect("no budget").distance)
            .abs();

        assert!(
            change <= (b - a).length() + 1e-9,
            "a distance field cannot change faster than the point moves"
        );
    }
}

#[test]
fn a_fillets_setback_is_the_radius_times_tan_of_half_the_angle_between_the_sides() {
    let rows = [
        (
            90.0,
            3.0,
            3.0,
            "a square edge asks for exactly the radius - the rule's anchor",
        ),
        (
            60.0,
            3.0,
            1.7320508075688772,
            "a blunter edge (normals only 60 apart) needs less room: r * tan(30)",
        ),
        (
            120.0,
            3.0,
            5.196152422706632,
            "a sharper edge needs more: r * tan(60)",
        ),
        (
            0.0,
            3.0,
            0.0,
            "two sides facing the same way are not an edge at all, and ask for nothing",
        ),
    ];
    for (degrees_between_normals, radius, expected, why) in rows {
        let radians: f64 = degrees_between_normals * std::f64::consts::PI / 180.0;
        let n_left = Vector3::new(0.0, 0.0, 1.0);
        let n_right = Vector3::new(radians.sin(), 0.0, radians.cos());

        near(
            chain_setback::for_fillet(radius, n_left, n_right),
            expected,
            1e-9,
            why,
        );
    }
}

#[test]
fn a_chamfers_setback_is_its_size() {
    assert_eq!(
        chain_setback::for_chamfer(2.75),
        2.75,
        "a chamfer's size IS how far back it reaches; there is nothing to work out"
    );
}

#[test]
fn distance_is_invariant_under_a_rigid_motion() {
    let points = square_points();
    let setbacks = [1.0, 2.0, 3.0, 4.0];

    let here = ChainField::from_polyline(&points, true, &setbacks).expect("a valid chain");
    let moved: Vec<Vector3> = points.iter().map(|&p| rigidly_moved(p)).collect();
    let there = ChainField::from_polyline(&moved, true, &setbacks).expect("a valid chain");
    let mut random = SeededRandom::new(4711);

    for _ in 0..500 {
        let query = random_point(&mut random, 30.0, 10.0, 20.0, 10.0);

        let before = here.evaluate(query).expect("no budget");
        let after = there.evaluate(rigidly_moved(query)).expect("no budget");

        near(
            after.distance,
            before.distance,
            1e-9,
            "a rotation and a translation are not geometry",
        );
        near(
            after.parameter,
            before.parameter,
            1e-9,
            "and neither is the chain parameter of the nearest point",
        );
    }
}

#[test]
fn evaluating_a_hundred_thousand_points_against_a_sixty_four_segment_loop_is_recorded() {
    const SEGMENTS: usize = 64;
    const QUERIES: usize = 100000;

    let mut ring = Vec::with_capacity(SEGMENTS);
    let mut setbacks = Vec::with_capacity(SEGMENTS);
    for i in 0..SEGMENTS {
        let angle = i as f64 * 2.0 * std::f64::consts::PI / SEGMENTS as f64;
        ring.push(Vector3::new(angle.cos() * 10.0, angle.sin() * 10.0, 0.0));
        setbacks.push(1.0);
    }

    let field = ChainField::from_polyline(&ring, true, &setbacks).expect("a valid chain");
    let mut random = SeededRandom::new(1234);
    let queries: Vec<Vector3> = (0..QUERIES)
        .map(|_| random_point(&mut random, 30.0, 15.0, 10.0, 5.0))
        .collect();

    // One warm pass so the timed pass is not measuring the first touch of the caches.
    let mut soak = 0.0;
    for query in queries.iter().take(1000) {
        soak += field.evaluate(*query).expect("no budget").value;
    }

    let timer = Instant::now();
    for query in &queries {
        soak += field.evaluate(*query).expect("no budget").value;
    }

    let elapsed = timer.elapsed();
    println!(
        "ChainField: {QUERIES} evaluations against {SEGMENTS} segments took {:.1} ms ({:.2} us each)",
        elapsed.as_secs_f64() * 1000.0,
        elapsed.as_secs_f64() * 1e6 / QUERIES as f64
    );

    assert!(
        soak.is_finite(),
        "the work has to actually happen to be timed"
    );
    assert!(
        elapsed.as_secs_f64() < 10.0,
        "this is a loose ceiling, not a pin - it catches an accidental order of magnitude, nothing finer"
    );
}

#[test]
fn the_greatest_value_over_a_triangle_agrees_with_brute_force() {
    let chain = ChainField::from_polyline(
        &[
            Vector3::new(0.0, 0.0, 0.0),
            Vector3::new(10.0, 0.0, 0.0),
            Vector3::new(10.0, 6.0, 0.0),
            Vector3::new(0.0, 6.0, 0.0),
        ],
        true,
        &[0.7, 0.7, 0.7, 0.7],
    )
    .expect("a valid chain");

    let mut random = SeededRandom::new(20260918);
    for _ in 0..20 {
        let mut corner = || {
            let x = random.next_double() * 10.0;
            let y = random.next_double() * 6.0;
            Vector3::new(x, y, 0.0)
        };
        let (a, b, c) = (corner(), corner(), corner());

        let greatest = chain.max_over_triangle(a, b, c, 1e-6).expect("no budget");

        let mut sampled = f64::NEG_INFINITY;
        const STEPS: i32 = 120;
        for i in 0..=STEPS {
            for j in 0..=(STEPS - i) {
                let u = i as f64 / STEPS as f64;
                let v = j as f64 / STEPS as f64;
                let value = chain
                    .value_at(a + ((b - a) * u) + ((c - a) * v))
                    .expect("no budget");
                sampled = vector_math::ieee_math::max_f64(sampled, value);
            }
        }

        assert!(
            greatest.value >= sampled - 1e-6,
            "a bounded search may not be beaten by a sample of the same triangle"
        );
        near(
            chain.value_at(greatest.point).expect("no budget"),
            greatest.value,
            1e-9,
            "the value reported is the value at the point reported",
        );
    }
}

#[test]
fn a_thin_island_is_found_where_a_sample_would_miss_it() {
    let chain = ChainField::from_polyline(
        &[
            Vector3::new(0.0, 0.0, 0.0),
            Vector3::new(10.0, 0.0, 0.0),
            Vector3::new(10.0, 1.0, 0.0),
            Vector3::new(0.0, 1.0, 0.0),
        ],
        true,
        &[0.45, 0.45, 0.45, 0.45],
    )
    .expect("a valid chain");

    // The face as the two triangles a quad is made of - the shape the refinement is handed.
    let greatest = chain
        .max_over_triangle(
            Vector3::new(0.0, 0.0, 0.0),
            Vector3::new(10.0, 0.0, 0.0),
            Vector3::new(10.0, 1.0, 0.0),
            1e-6,
        )
        .expect("no budget");

    assert!(
        greatest.value > 0.0,
        "the middle of the face stands 0.5 from the rim and only 0.45 is taken"
    );
    near(
        greatest.value,
        0.05,
        1e-5,
        "the ridge runs half way across the 1 wide face",
    );

    let centroid =
        (Vector3::new(0.0, 0.0, 0.0) + Vector3::new(10.0, 0.0, 0.0) + Vector3::new(10.0, 1.0, 0.0))
            / 3.0;
    assert!(
        chain.value_at(centroid).expect("no budget") < 0.0,
        "the centroid of that triangle is inside - which is why sampling missed the island"
    );
}

/// A fixed rotation about Z then X, plus a translation - anything rigid will do.
fn rigidly_moved(point: Vector3) -> Vector3 {
    const ABOUT_Z: f64 = 0.7;
    const ABOUT_X: f64 = 0.3;

    let after_z = Vector3::new(
        (point.x * ABOUT_Z.cos()) - (point.y * ABOUT_Z.sin()),
        (point.x * ABOUT_Z.sin()) + (point.y * ABOUT_Z.cos()),
        point.z,
    );

    let after_x = Vector3::new(
        after_z.x,
        (after_z.y * ABOUT_X.cos()) - (after_z.z * ABOUT_X.sin()),
        (after_z.y * ABOUT_X.sin()) + (after_z.z * ABOUT_X.cos()),
    );

    after_x + Vector3::new(-17.25, 4.5, 31.75)
}
