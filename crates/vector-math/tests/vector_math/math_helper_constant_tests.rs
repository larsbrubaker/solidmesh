// Port of `MathHelperConstantTests` (agg-sharp VectorMathTests.cs).

use std::f64::consts;
use vector_math::math_helper;

// These were once float literals widened to double, off from the true values by about 1e-7.
#[test]
fn constants_are_full_double_precision() {
    assert_eq!(math_helper::PI, consts::PI);
    assert_eq!(math_helper::TAU, 2.0 * consts::PI);
    assert_eq!(math_helper::TWO_PI, 2.0 * consts::PI);
    assert_eq!(math_helper::PI_OVER_2, consts::PI / 2.0);
    assert_eq!(math_helper::PI_OVER_3, consts::PI / 3.0);
    assert_eq!(math_helper::PI_OVER_4, consts::PI / 4.0);
    assert_eq!(math_helper::PI_OVER_6, consts::PI / 6.0);
    assert_eq!(math_helper::THREE_PI_OVER_2, 3.0 * consts::PI / 2.0);
    assert_eq!(math_helper::E, consts::E);
    assert_eq!(math_helper::LOG10_E, consts::E.log10());
    assert_eq!(math_helper::LOG2_E, consts::E.log2());
}
