//! `System.Math` semantics where they differ from Rust's std: `Math.Max`/`Math.Min` propagate
//! NaN and order -0 below +0 (IEEE 754-2019 maximum/minimum, as .NET Core 3.0+ implements
//! them), while `f64::max`/`f64::min` ignore NaN. The vector types use these wherever the C#
//! calls `Math.Max`/`Math.Min`, so a NaN component behaves as it does in MatterCAD.

/// C# `Math.Max(double, double)`.
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

/// C# `Math.Min(double, double)`.
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

/// C# `Math.Max(float, float)`.
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

/// C# `Math.Min(float, float)`.
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

/// C# `Math.Log(a, 2)`: `Log(a) / Log(2)`, not `log2`, so results match bit for bit.
pub fn log_base2(a: f64) -> f64 {
    a.ln() / 2f64.ln()
}
