// Easing curves: the linear, cubic and quadratic curves, in every option, invert back to their
// input across [0, 1].

use vector_math::easing::{self, EaseOption, EaseType, DEFAULT_INVERSE_TOLERANCE};

#[test]
fn static_function_tests() {
    fn inverse_working(ease_type: EaseType, ease_option: EaseOption, k: f64) {
        let output = easing::calculate(ease_type, ease_option, k);
        let input =
            easing::calculate_inverse(ease_type, ease_option, output, DEFAULT_INVERSE_TOLERANCE);
        assert!(
            (input - k).abs() <= 0.001,
            "{ease_type:?} {ease_option:?}: inverse of {k} gave {input}"
        );
    }

    fn range_test(ease_type: EaseType) {
        let mut i = 0.0;
        while i <= 1.0 {
            inverse_working(ease_type, EaseOption::In, i);
            inverse_working(ease_type, EaseOption::InOut, i);
            inverse_working(ease_type, EaseOption::Out, i);
            i += 0.05;
        }
    }

    range_test(EaseType::Linear);
    range_test(EaseType::Cubic);
    range_test(EaseType::Quadratic);
}
