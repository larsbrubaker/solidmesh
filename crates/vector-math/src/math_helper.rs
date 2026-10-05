//! Common mathematical functions and constants. Ports agg-sharp `VectorMath/MathHelper.cs`
//! (originally OpenTK's, MIT/X11; see THIRD_PARTY_NOTICES.md). The vector types use `TAU`
//! and `range_0_to_tau`; later phases use the rest.
//!
//! Where the C# throws (`ArgumentOutOfRangeException`, `Exception`, `DivideByZeroException`)
//! the Rust returns `Err(MathError)`.

use crate::cs_math::log_base2;
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
/// Pi multiplied by 3 and divided by two (evaluated as `(3 * Pi) / 2`, as in C#).
pub const THREE_PI_OVER_2: f64 = 3.0 * PI / 2.0;
/// E.
pub const E: f64 = std::f64::consts::E;
/// The base-10 logarithm of E. The C# literal `0.43429448190325182765` is this double.
pub const LOG10_E: f64 = std::f64::consts::LOG10_E;
/// The base-2 logarithm of E. The C# literal `1.44269504088896340736` is this double.
pub const LOG2_E: f64 = std::f64::consts::LOG2_E;

/// Why a `MathHelper` function rejected its input (the C# throws instead).
#[derive(Debug, Clone, PartialEq)]
pub enum MathError {
    /// `NextPowerOfTwo` was given a negative number ("Must be positive.").
    MustBePositive,
    /// `Range0ToTau` could not bring the value into [0, Tau].
    NotInRange0ToTau(f64),
    /// `GetDeltaAngle` was given an angle that is not already in [0, Tau].
    AngleNotInRange0ToTau(f64),
    /// `BinomialCoefficient`'s factorial product overflowed to zero (or to -1 under
    /// `long.MinValue`), which C# reports as a division error.
    DivisionOverflow,
    /// A matrix builder was given an out-of-range argument (C# `ArgumentOutOfRangeException`
    /// naming the parameter, e.g. `CreatePerspectiveFieldOfView`'s "fovy").
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

/// The smallest power of two >= `n` (C# `FirstPowerTowGreaterThanOrEqualTo`). C# `<<=` on an
/// int wraps; so does this.
pub fn first_power_two_greater_than_or_equal_to(n: i32) -> i32 {
    let mut pow2_size: i32 = 1;
    while pow2_size < n {
        pow2_size = pow2_size.wrapping_shl(1);
    }
    pow2_size
}

/// `2^ceil(log2(n))` (C# `NextPowerOfTwo(long)`). The cast saturates, as .NET 9+ does.
pub fn next_power_of_two_i64(n: i64) -> Result<i64, MathError> {
    if n < 0 {
        return Err(MathError::MustBePositive);
    }
    Ok(2f64.powf(log_base2(n as f64).ceil()) as i64)
}

/// `2^ceil(log2(n))` (C# `NextPowerOfTwo(int)`).
pub fn next_power_of_two_i32(n: i32) -> Result<i32, MathError> {
    if n < 0 {
        return Err(MathError::MustBePositive);
    }
    Ok(2f64.powf(log_base2(f64::from(n)).ceil()) as i32)
}

/// `2^ceil(log2(n))` (C# `NextPowerOfTwo(float)`).
pub fn next_power_of_two_f32(n: f32) -> Result<f32, MathError> {
    if n < 0.0 {
        return Err(MathError::MustBePositive);
    }
    Ok(2f64.powf(log_base2(f64::from(n)).ceil()) as f32)
}

/// `2^ceil(log2(n))` (C# `NextPowerOfTwo(double)`).
pub fn next_power_of_two_f64(n: f64) -> Result<f64, MathError> {
    if n < 0.0 {
        return Err(MathError::MustBePositive);
    }
    Ok(2f64.powf(log_base2(n).ceil()))
}

/// `test` limited to `[min, max]` (C# `MathHelper.Clamp`; NaN passes through).
pub fn clamp(test: f64, min: f64, max: f64) -> f64 {
    if test < min {
        return min;
    }
    if test > max {
        return max;
    }
    test
}

/// `n!` (C# `Factorial`). C# `long` multiplication is unchecked, so it wraps past 20!.
pub fn factorial(n: i32) -> i64 {
    let mut result: i64 = 1;
    let mut n = n;
    while n > 1 {
        result = result.wrapping_mul(i64::from(n));
        n -= 1;
    }
    result
}

/// `n! / (k! * (n - k)!)` (C# `BinomialCoefficient`).
pub fn binomial_coefficient(n: i32, k: i32) -> Result<i64, MathError> {
    factorial(n)
        .checked_div(factorial(k).wrapping_mul(factorial(n.wrapping_sub(k))))
        .ok_or(MathError::DivisionOverflow)
}

/// Brings an angle into `[0, Tau]` (C# `Range0ToTau`). NaN passes through, as in C#.
pub fn range_0_to_tau(value: f64) -> Result<f64, MathError> {
    let mut value = value;
    if value < 0.0 {
        value += TAU;
    }

    if (value - TAU) > 0.00001 {
        // `(int)` truncates; out-of-range values saturate, as .NET 9+ does.
        let num_turns = (value / TAU) as i32;
        value -= TAU * f64::from(num_turns);
    }

    // Not `!(0.0..=TAU).contains(&value)`: NaN must pass, as it does in C#.
    #[allow(clippy::manual_range_contains)]
    if value < 0.0 || value > TAU {
        return Err(MathError::NotInRange0ToTau(value));
    }

    Ok(value)
}

/// `end_angle - start_angle`, folded into `[-Tau, Tau]` (C# `GetDeltaAngle`). Both angles must
/// already be in `[0, Tau]`.
pub fn get_delta_angle(start_angle: f64, end_angle: f64) -> Result<f64, MathError> {
    // C# compares with `!=`, so a NaN angle fails here too.
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

/// Degrees to radians (C# `DegreesToRadians`).
pub fn degrees_to_radians(degrees: f64) -> f64 {
    const DEG_TO_RAD: f64 = PI / 180.0;
    degrees * DEG_TO_RAD
}

/// Radians to degrees (C# `RadiansToDegrees`).
pub fn radians_to_degrees(radians: f64) -> f64 {
    const RAD_TO_DEG: f64 = 180.0 / PI;
    radians * RAD_TO_DEG
}

/// Inches to millimeters (C# `InchesToMm`).
pub fn inches_to_mm(inches: f64) -> f64 {
    inches * 25.4
}

/// Swaps two values (C# `Swap(ref double, ref double)` and `Swap(ref float, ref float)`).
pub fn swap<T>(a: &mut T, b: &mut T) {
    std::mem::swap(a, b);
}

/// True when `a` is strictly within `difference_allowed` of `b` (C# `AlmostEqual`).
pub fn almost_equal(a: f64, b: f64, difference_allowed: f64) -> bool {
    a > b - difference_allowed && a < b + difference_allowed
}
