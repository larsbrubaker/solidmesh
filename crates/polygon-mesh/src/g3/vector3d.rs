//! The `[f64; 3]` arithmetic of geometry3Sharp's `Vector3d` (`math/Vector3d.cs`) and the
//! triangle helpers of `MathUtil` (`math/MathUtil.cs`) that the remeshing code (`mesh_util`,
//! `mesh_refiner_base`, `remesher*`) and `MinimalHoleFill`'s quality metrics (`Area`,
//! `AspectRatio`) use, each with C#'s operation order.

// Consumed by `MinimalHoleFill` (mesh repair), which lands in a later slice; until then only the
// tests reach some of it.
#![allow(dead_code)]

/// C# `MathUtil.Epsilon`.
pub(crate) const EPSILON: f64 = f64::EPSILON; // 2.2204460492503131e-016
/// C# `MathUtil.ZeroTolerance`.
pub(crate) const ZERO_TOLERANCE: f64 = 1e-08;
/// C# `MathUtil.ZeroTolerancef` (a `float` constant widened to `double`).
pub(crate) const ZERO_TOLERANCEF: f64 = 1e-06_f32 as f64;
/// C# `MathUtil.Epsilonf` (a `float` constant widened to `double`).
pub(crate) const EPSILONF: f64 = f32::EPSILON as f64; // 1.192092896e-07F
/// C# `MathUtil.TwoPI`.
pub(crate) const TWO_PI: f64 = 2.0 * std::f64::consts::PI;

/// `a + b`.
pub(crate) fn add(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[0] + b[0], a[1] + b[1], a[2] + b[2]]
}

/// `a - b`.
pub(crate) fn sub(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}

/// `a * s`.
pub(crate) fn scale(a: [f64; 3], s: f64) -> [f64; 3] {
    [a[0] * s, a[1] * s, a[2] * s]
}

/// C# `Vector3d.Dot`.
pub(crate) fn dot(a: [f64; 3], b: [f64; 3]) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

/// C# `Vector3d.Cross`.
pub(crate) fn cross(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}

/// C# `Vector3d.Length`.
pub(crate) fn length(a: [f64; 3]) -> f64 {
    (a[0] * a[0] + a[1] * a[1] + a[2] * a[2]).sqrt()
}

/// C# `Vector3d.Normalized`: the zero vector when the length is at most `MathUtil.Epsilon`.
pub(crate) fn normalized(a: [f64; 3]) -> [f64; 3] {
    let length = length(a);
    if length > EPSILON {
        let inv_length = 1.0 / length;
        [a[0] * inv_length, a[1] * inv_length, a[2] * inv_length]
    } else {
        [0.0; 3]
    }
}

/// C# `Vector3d.Normalize()` (in place, default epsilon); the same values as
/// [`normalized`].
pub(crate) fn normalize(a: &mut [f64; 3]) {
    *a = normalized(*a);
}

/// C# `Vector3d.Distance`.
pub(crate) fn distance(a: [f64; 3], b: [f64; 3]) -> f64 {
    distance_squared(a, b).sqrt()
}

/// C# `Vector3d.DistanceSquared` (`a.DistanceSquared(b)`: `b - a` per component).
pub(crate) fn distance_squared(a: [f64; 3], b: [f64; 3]) -> f64 {
    let dx = b[0] - a[0];
    let dy = b[1] - a[1];
    let dz = b[2] - a[2];
    dx * dx + dy * dy + dz * dz
}

/// C# `Vector3d.AngleR`: the dot clamped to [-1, 1], then `acos`.
pub(crate) fn angle_r(a: [f64; 3], b: [f64; 3]) -> f64 {
    let f_dot = clamp(dot(a, b), -1.0, 1.0);
    f_dot.acos()
}

/// C# `Vector3d.EpsilonEqual`.
pub(crate) fn epsilon_equal(a: [f64; 3], b: [f64; 3], epsilon: f64) -> bool {
    (a[0] - b[0]).abs() <= epsilon
        && (a[1] - b[1]).abs() <= epsilon
        && (a[2] - b[2]).abs() <= epsilon
}

/// C# `Vector3d.IsFinite`.
pub(crate) fn is_finite(a: [f64; 3]) -> bool {
    a.iter().all(|c| c.is_finite())
}

/// C# `MathUtil.Normal`: both edges normalized before the cross product, then the cross
/// normalized.
pub(crate) fn tri_normal(v1: [f64; 3], v2: [f64; 3], v3: [f64; 3]) -> [f64; 3] {
    let mut edge1 = sub(v2, v1);
    let mut edge2 = sub(v3, v2);
    normalize(&mut edge1);
    normalize(&mut edge2);
    let mut v_cross = cross(edge1, edge2);
    normalize(&mut v_cross);
    v_cross
}

/// C# `MathUtil.FastNormalDirection`: the unnormalized cross product.
pub(crate) fn fast_normal_direction(v1: [f64; 3], v2: [f64; 3], v3: [f64; 3]) -> [f64; 3] {
    let edge1 = sub(v2, v1);
    let edge2 = sub(v3, v1);
    cross(edge1, edge2)
}

/// C# `MathUtil.Area(v1, v2, v3)`: half the length of the edge cross product.
pub(crate) fn area(v1: [f64; 3], v2: [f64; 3], v3: [f64; 3]) -> f64 {
    0.5 * length(cross(sub(v2, v1), sub(v3, v1)))
}

/// C# `MathUtil.AspectRatio(v1, v2, v3)`: `abc / (8 (s-a)(s-b)(s-c))`, 1 for an equilateral
/// triangle and growing as it degenerates.
pub(crate) fn aspect_ratio(v1: [f64; 3], v2: [f64; 3], v3: [f64; 3]) -> f64 {
    let a = distance(v1, v2);
    let b = distance(v2, v3);
    let c = distance(v3, v1);
    let s = (a + b + c) / 2.0;
    (a * b * c) / (8.0 * (s - a) * (s - b) * (s - c))
}

/// C# `MathUtil.Clamp(double, double, double)`.
pub(crate) fn clamp(f: f64, low: f64, high: f64) -> f64 {
    if f < low {
        low
    } else if f > high {
        high
    } else {
        f
    }
}
