//! Common mathematical functions and constants (originally OpenTK's, MIT/X11; see
//! THIRD_PARTY_NOTICES.md). The vector types use `TAU` and `range_0_to_tau`; the model and
//! renderer use the rest.
//!
//! Input a function cannot handle (an out-of-range argument, an angle outside its range, an
//! overflowing product) comes back as `Err(MathError)` for the caller to handle.

use crate::ieee_math::log_base2;
use std::fmt;

/// Pi as an f64.
pub const PI: f64 = std::f64::consts::PI;
/// Two Pi.
pub const TAU: f64 = PI * 2.0;
/// Pi divided by two.
pub const PI_OVER_2: f64 = PI / 2.0;
/// Pi divided by three.
pub const PI_OVER_3: f64 = PI / 3.0;
/// Pi divided by four.
pub const PI_OVER_4: f64 = PI / 4.0;
/// Pi divided by six.
pub const PI_OVER_6: f64 = PI / 6.0;
/// Pi multiplied by two.
pub const TWO_PI: f64 = 2.0 * PI;
/// Pi multiplied by 3 and divided by two (evaluated as `(3 * Pi) / 2`, which fixes the last
/// bit).
pub const THREE_PI_OVER_2: f64 = 3.0 * PI / 2.0;
/// E.
pub const E: f64 = std::f64::consts::E;
/// The base-10 logarithm of E. The literal `0.43429448190325182765` is this double.
pub const LOG10_E: f64 = std::f64::consts::LOG10_E;
/// The base-2 logarithm of E. The literal `1.44269504088896340736` is this double.
pub const LOG2_E: f64 = std::f64::consts::LOG2_E;

/// Why a `math_helper` function rejected its input.
#[derive(Debug, Clone, PartialEq)]
pub enum MathError {
    /// `NextPowerOfTwo` was given a negative number ("Must be positive.").
    MustBePositive,
    /// `Range0ToTau` could not bring the value into [0, Tau].
    NotInRange0ToTau(f64),
    /// `GetDeltaAngle` was given an angle that is not already in [0, Tau].
    AngleNotInRange0ToTau(f64),
    /// `binomial_coefficient`'s factorial product overflowed to zero (or to -1 under
    /// `i64::MIN`), so the division has no answer.
    DivisionOverflow,
    /// A matrix builder was given an out-of-range argument, naming the parameter (e.g.
    /// `create_perspective_field_of_view`'s "fovy").
    ArgumentOutOfRange(&'static str),
}

impl fmt::Display for MathError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            MathError::MustBePositive => write!(f, "n: Must be positive."),
            MathError::NotInRange0ToTau(_) => write!(f, "Value >= 0 && Value <= Tau"),
            MathError::AngleNotInRange0ToTau(_) => {
                write!(f, "StartAngle != Range0ToTau(StartAngle)")
            }
            MathError::DivisionOverflow => write!(f, "Attempted to divide by zero."),
            MathError::ArgumentOutOfRange(name) => write!(
                f,
                "Specified argument was out of the range of valid values. (Parameter '{name}')"
            ),
        }
    }
}

impl std::error::Error for MathError {}

/// The smallest power of two >= `n` (1 for `n` <= 1). Past 2^30 no `i32` power of two is
/// large enough, so the result saturates at `i32::MAX` instead of wrapping (a wrapped shift
/// would never reach `n` and loop forever).
pub fn first_power_two_greater_than_or_equal_to(n: i32) -> i32 {
    if n > 1 << 30 {
        return i32::MAX;
    }
    let mut pow2_size: i32 = 1;
    while pow2_size < n {
        pow2_size <<= 1;
    }
    pow2_size
}

/// `2^ceil(log2(n))`, corrected to be exact: the logarithm's rounding can land just above a
/// whole number for an exact power of two (or just below it next to one), and `ceil` would
/// then double (or halve) the answer. The result is the smallest power of two >= `n`.
fn power_of_two_at_or_above(n: f64) -> f64 {
    let mut pow2 = 2f64.powf(log_base2(n).ceil());
    if pow2.is_finite() && pow2 > 0.0 {
        if pow2 * 0.5 >= n {
            pow2 *= 0.5;
        } else if pow2 < n {
            pow2 *= 2.0;
        }
    }
    pow2
}

/// The smallest power of two >= `n` (0 for 0). The cast saturates, so a result too large for
/// `i64` is `i64::MAX`.
pub fn next_power_of_two_i64(n: i64) -> Result<i64, MathError> {
    if n < 0 {
        return Err(MathError::MustBePositive);
    }
    Ok(power_of_two_at_or_above(n as f64) as i64)
}

/// The smallest power of two >= `n` (0 for 0).
pub fn next_power_of_two_i32(n: i32) -> Result<i32, MathError> {
    if n < 0 {
        return Err(MathError::MustBePositive);
    }
    Ok(power_of_two_at_or_above(f64::from(n)) as i32)
}

/// The smallest power of two >= `n` (0 for 0).
pub fn next_power_of_two_f32(n: f32) -> Result<f32, MathError> {
    if n < 0.0 {
        return Err(MathError::MustBePositive);
    }
    Ok(power_of_two_at_or_above(f64::from(n)) as f32)
}

/// The smallest power of two >= `n` (0 for 0).
pub fn next_power_of_two_f64(n: f64) -> Result<f64, MathError> {
    if n < 0.0 {
        return Err(MathError::MustBePositive);
    }
    Ok(power_of_two_at_or_above(n))
}

/// `test` limited to `[min, max]` (NaN passes through).
pub fn clamp(test: f64, min: f64, max: f64) -> f64 {
    if test < min {
        return min;
    }
    if test > max {
        return max;
    }
    test
}

/// `n!`, saturating at `i64::MAX` past 20! (the largest factorial an `i64` holds).
pub fn factorial(n: i32) -> i64 {
    let mut result: i64 = 1;
    let mut n = n;
    while n > 1 {
        result = result.saturating_mul(i64::from(n));
        n -= 1;
    }
    result
}

/// `n! / (k! * (n - k)!)`.
pub fn binomial_coefficient(n: i32, k: i32) -> Result<i64, MathError> {
    factorial(n)
        .checked_div(factorial(k).wrapping_mul(factorial(n.wrapping_sub(k))))
        .ok_or(MathError::DivisionOverflow)
}

/// Brings an angle into `[0, Tau]`. NaN passes through, so a bad angle stays visible.
pub fn range_0_to_tau(value: f64) -> Result<f64, MathError> {
    let mut value = value;
    if value < 0.0 {
        value += TAU;
    }

    if (value - TAU) > 0.00001 {
        // The cast truncates; out-of-range values saturate instead of wrapping.
        let num_turns = (value / TAU) as i32;
        value -= TAU * f64::from(num_turns);
    }

    // Not `!(0.0..=TAU).contains(&value)`: NaN must pass this check (see the doc comment).
    #[allow(clippy::manual_range_contains)]
    if value < 0.0 || value > TAU {
        return Err(MathError::NotInRange0ToTau(value));
    }

    Ok(value)
}

/// `end_angle - start_angle`, folded into `[-Tau, Tau]`. Both angles must
/// already be in `[0, Tau]`.
pub fn get_delta_angle(start_angle: f64, end_angle: f64) -> Result<f64, MathError> {
    // `!=` is true for NaN, so a NaN angle fails here too.
    #[allow(clippy::float_cmp)]
    if start_angle != range_0_to_tau(start_angle)? {
        return Err(MathError::AngleNotInRange0ToTau(start_angle));
    }
    #[allow(clippy::float_cmp)]
    if end_angle != range_0_to_tau(end_angle)? {
        return Err(MathError::AngleNotInRange0ToTau(end_angle));
    }

    let mut delta_angle = end_angle - start_angle;
    if delta_angle > TAU {
        delta_angle -= TAU;
    }

    if delta_angle < -TAU {
        delta_angle += TAU;
    }

    Ok(delta_angle)
}

/// Degrees to radians.
pub fn degrees_to_radians(degrees: f64) -> f64 {
    const DEG_TO_RAD: f64 = PI / 180.0;
    degrees * DEG_TO_RAD
}

/// Radians to degrees.
pub fn radians_to_degrees(radians: f64) -> f64 {
    const RAD_TO_DEG: f64 = 180.0 / PI;
    radians * RAD_TO_DEG
}

/// Inches to millimeters.
pub fn inches_to_mm(inches: f64) -> f64 {
    inches * 25.4
}

/// Swaps two values.
pub fn swap<T>(a: &mut T, b: &mut T) {
    std::mem::swap(a, b);
}

/// True when `a` is strictly within `difference_allowed` of `b`.
pub fn almost_equal(a: f64, b: f64, difference_allowed: f64) -> bool {
    a > b - difference_allowed && a < b + difference_allowed
}
