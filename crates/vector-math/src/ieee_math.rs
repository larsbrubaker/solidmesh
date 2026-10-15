//! Max and min that propagate NaN and order -0 below +0 (IEEE 754-2019 maximum/minimum), where
//! `f64::max`/`f64::min` ignore NaN. The vector types and bounding boxes take their max and min
//! through these, so a NaN component shows in the result instead of silently disappearing.

/// The larger of two doubles; NaN if either is NaN, +0 over -0.
pub fn max_f64(val1: f64, val2: f64) -> f64 {
    if val1 != val2 {
        if !val1.is_nan() {
            return if val2 < val1 { val1 } else { val2 };
        }
        return val1;
    }
    if val2.is_sign_negative() {
        val1
    } else {
        val2
    }
}

/// The smaller of two doubles; NaN if either is NaN, -0 under +0.
pub fn min_f64(val1: f64, val2: f64) -> f64 {
    if val1 != val2 && !val1.is_nan() {
        return if val1 < val2 { val1 } else { val2 };
    }
    if val1.is_sign_negative() {
        val1
    } else {
        val2
    }
}

/// The larger of two floats; NaN if either is NaN, +0 over -0.
pub fn max_f32(val1: f32, val2: f32) -> f32 {
    if val1 != val2 {
        if !val1.is_nan() {
            return if val2 < val1 { val1 } else { val2 };
        }
        return val1;
    }
    if val2.is_sign_negative() {
        val1
    } else {
        val2
    }
}

/// The smaller of two floats; NaN if either is NaN, -0 under +0.
pub fn min_f32(val1: f32, val2: f32) -> f32 {
    if val1 != val2 && !val1.is_nan() {
        return if val1 < val2 { val1 } else { val2 };
    }
    if val1.is_sign_negative() {
        val1
    } else {
        val2
    }
}

/// The base-2 logarithm as `ln(a) / ln(2)`, not `log2`: the two can differ in the last bit,
/// so a power of two can come out a hair off its whole-number logarithm; the
/// `next_power_of_two_*` helpers correct for that after rounding up.
pub fn log_base2(a: f64) -> f64 {
    a.ln() / 2f64.ln()
}
