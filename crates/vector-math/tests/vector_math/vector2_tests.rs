// Vector2 behaviour: the signed delta angle around a center, and distance to a line.

use vector_math::{math_helper, Vector2};

#[test]
fn get_delta_angle_tests() {
    // angles around 0
    {
        let center = Vector2::new(0.0, 0.0);
        let start = Vector2::new(1.0, 0.0);
        let end = Vector2::new(0.0, 1.0);
        assert!((math_helper::TAU / 4.0 - center.get_delta_angle(start, end)).abs() < 0.0001);

        let start = Vector2::new(-1.0, 0.0);
        let end = Vector2::new(0.0, 1.0);
        assert!((-math_helper::TAU / 4.0 - center.get_delta_angle(start, end)).abs() < 0.0001);
    }

    // angles around 1,2
    {
        let center = Vector2::new(1.0, 2.0);
        let start = Vector2::new(2.0, 2.0);
        let end = Vector2::new(1.0, 3.0);
        assert!((math_helper::TAU / 4.0 - center.get_delta_angle(start, end)).abs() < 0.0001);

        let start = Vector2::new(0.0, 2.0);
        let end = Vector2::new(1.0, 3.0);
        assert!((-math_helper::TAU / 4.0 - center.get_delta_angle(start, end)).abs() < 0.0001);
    }
}

#[test]
fn distance_to_line_tests() {
    let d = |p: (f64, f64), s: (f64, f64), e: (f64, f64)| {
        Vector2::distance_point_to_line(
            Vector2::new(p.0, p.1),
            Vector2::new(s.0, s.1),
            Vector2::new(e.0, e.1),
        )
    };
    // outside the line
    assert_eq!(d((0.0, 0.0), (5.0, 0.0), (10.0, 0.0)), 5.0);
    assert_eq!(d((15.0, 0.0), (5.0, 0.0), (10.0, 0.0)), 5.0);
    assert_eq!(d((-1.0, -1.0), (0.0, 0.0), (10.0, 0.0)), 2f64.sqrt());
    assert_eq!(d((-1.0, 1.0), (0.0, 0.0), (10.0, 0.0)), 2f64.sqrt());
    assert_eq!(d((11.0, -1.0), (0.0, 0.0), (10.0, 0.0)), 2f64.sqrt());
    assert_eq!(d((11.0, 1.0), (0.0, 0.0), (10.0, 0.0)), 2f64.sqrt());
    // inside the line
    assert_eq!(d((7.0, 5.0), (5.0, 0.0), (10.0, 0.0)), 5.0);
    assert_eq!(d((7.0, -5.0), (5.0, 0.0), (10.0, 0.0)), 5.0);
    // line is a point
    assert_eq!(d((0.0, 0.0), (5.0, 0.0), (5.0, 0.0)), 5.0);
}
