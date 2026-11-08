//! The end ray - an open chain's line carried on past an end named in `ChainField::end_rays`. Past
//! that end a point reads its offset from the extended LINE and a parameter beyond the end; short
//! of the end, and at an end without a ray, it reads exactly as before. Each test's cases run as a
//! loop.

use solidmesh::bevel::level_line::ChainField;
use solidmesh_math::Vector3;

const POINTS: [Vector3; 3] = [
    Vector3::new(0.0, 0.0, 0.0),
    Vector3::new(4.0, 1.0, 0.0),
    Vector3::new(10.0, 1.0, 0.0),
];

const SETBACKS: [f64; 3] = [1.0, 1.0, 1.0];

fn rayed(end_rays: (bool, bool)) -> ChainField {
    ChainField::from_polyline_rayed(&POINTS, false, &SETBACKS, f64::INFINITY, end_rays)
        .expect("a valid chain")
}

#[test]
fn past_the_last_end_a_point_reads_the_extended_line_and_a_parameter_beyond_it() {
    for (x, y, z) in [(13.0, 3.0, 0.0), (11.5, 1.0, 2.0)] {
        let field = rayed((false, true));
        let length = field.length;
        let q = Vector3::new(x, y, z);
        let sample = field.evaluate(q).expect("no budget");
        let offset: f64 = (((y - 1.0) * (y - 1.0)) + (z * z)).sqrt();
        assert!((sample.value - (offset - 1.0)).abs() <= 1e-12);
        assert!((sample.parameter - (length + (x - 10.0))).abs() <= 1e-12);

        let exhaustive = field.evaluate_exhaustive(q);
        assert_eq!(exhaustive.value, sample.value);
        assert_eq!(exhaustive.parameter, sample.parameter);
    }
}

#[test]
fn before_the_first_end_the_parameter_runs_below_zero() {
    let field = rayed((true, false));
    let direction = (POINTS[1] - POINTS[0]).get_normal();
    let q = POINTS[0] - (direction * 3.0) + (Vector3::new(-direction.y, direction.x, 0.0) * 2.0);
    let sample = field.evaluate(q).expect("no budget");
    assert!((sample.value - 1.0).abs() <= 1e-12);
    assert!((sample.parameter + 3.0).abs() <= 1e-12);

    // The last end has no ray: it keeps the end vertex's ball and the clamped parameter.
    let past = field
        .evaluate(Vector3::new(13.0, 5.0, 0.0))
        .expect("no budget");
    assert!((past.value - (5.0 - 1.0)).abs() <= 1e-12);
    assert_eq!(past.parameter, field.length);
}

#[test]
fn short_of_the_ends_the_field_is_bit_identical_to_the_plain_one() {
    let plain = ChainField::from_polyline_sharp(&POINTS, false, &SETBACKS, f64::INFINITY)
        .expect("a valid chain");
    let rayed = rayed((true, true));
    for q in [
        Vector3::new(2.0, 3.0, 1.0),
        Vector3::new(7.0, -2.0, 0.5),
        Vector3::new(10.0, 4.0, 0.0),
        Vector3::new(1.0, -3.0, 0.0),
    ] {
        let a = plain.evaluate(q).expect("no budget");
        let b = rayed.evaluate(q).expect("no budget");
        assert_eq!(b.value, a.value);
        assert_eq!(b.parameter, a.parameter);
    }
}
