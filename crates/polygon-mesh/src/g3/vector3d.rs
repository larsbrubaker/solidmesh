//! `[f64; 3]` vector arithmetic and triangle helpers (derived from geometry3Sharp) that the
//! remeshing code (`mesh_util`, `mesh_refiner_base`, `remesher*`) and `MinimalHoleFill`'s
//! quality metrics ([`area`], [`aspect_ratio`]) use. Each keeps a fixed operation order, since
//! the hole fill's results are pinned bit for bit.

/// The double machine epsilon: the cutoff below which a vector is too short to normalize.
pub(crate) const EPSILON: f64 = f64::EPSILON; // 2.2204460492503131e-016
#[allow(dead_code)] // mesh API only the tests call so far
/// The general double zero tolerance.
pub(crate) const ZERO_TOLERANCE: f64 = 1e-08;
/// The single-precision zero tolerance (a `float` constant widened to `double`).
pub(crate) const ZERO_TOLERANCEF: f64 = 1e-06_f32 as f64;
/// The single-precision machine epsilon (a `float` constant widened to `double`).
pub(crate) const EPSILONF: f64 = f32::EPSILON as f64; // 1.192092896e-07F
/// Two pi.
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

/// `a . b`.
pub(crate) fn dot(a: [f64; 3], b: [f64; 3]) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

/// `a x b`.
pub(crate) fn cross(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}

/// The length of `a`.
pub(crate) fn length(a: [f64; 3]) -> f64 {
    (a[0] * a[0] + a[1] * a[1] + a[2] * a[2]).sqrt()
}

/// The unit vector, or the zero vector when the length is at most [`EPSILON`].
pub(crate) fn normalized(a: [f64; 3]) -> [f64; 3] {
    let length = length(a);
    if length > EPSILON {
        let inv_length = 1.0 / length;
        [a[0] * inv_length, a[1] * inv_length, a[2] * inv_length]
    } else {
        [0.0; 3]
    }
}

/// Normalizes `a` in place; the same values as
/// [`normalized`].
pub(crate) fn normalize(a: &mut [f64; 3]) {
    *a = normalized(*a);
}

/// The distance between `a` and `b`.
pub(crate) fn distance(a: [f64; 3], b: [f64; 3]) -> f64 {
    distance_squared(a, b).sqrt()
}

/// The squared distance between `a` and `b` (computed from `b - a` per component).
pub(crate) fn distance_squared(a: [f64; 3], b: [f64; 3]) -> f64 {
    let dx = b[0] - a[0];
    let dy = b[1] - a[1];
    let dz = b[2] - a[2];
    dx * dx + dy * dy + dz * dz
}

/// The dot clamped to [-1, 1], then `acos`.
pub(crate) fn angle_r(a: [f64; 3], b: [f64; 3]) -> f64 {
    let f_dot = clamp(dot(a, b), -1.0, 1.0);
    f_dot.acos()
}

/// Whether every component of `a` and `b` differs by at most `epsilon`.
pub(crate) fn epsilon_equal(a: [f64; 3], b: [f64; 3], epsilon: f64) -> bool {
    (a[0] - b[0]).abs() <= epsilon
        && (a[1] - b[1]).abs() <= epsilon
        && (a[2] - b[2]).abs() <= epsilon
}

/// Whether every component is finite.
pub(crate) fn is_finite(a: [f64; 3]) -> bool {
    a.iter().all(|c| c.is_finite())
}

/// Both edges normalized before the cross product, then the cross
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

/// The unnormalized cross product.
pub(crate) fn fast_normal_direction(v1: [f64; 3], v2: [f64; 3], v3: [f64; 3]) -> [f64; 3] {
    let edge1 = sub(v2, v1);
    let edge2 = sub(v3, v1);
    cross(edge1, edge2)
}

/// Half the length of the edge cross product.
pub(crate) fn area(v1: [f64; 3], v2: [f64; 3], v3: [f64; 3]) -> f64 {
    0.5 * length(cross(sub(v2, v1), sub(v3, v1)))
}

/// `abc / (8 (s-a)(s-b)(s-c))`, 1 for an equilateral
/// triangle and growing as it degenerates.
pub(crate) fn aspect_ratio(v1: [f64; 3], v2: [f64; 3], v3: [f64; 3]) -> f64 {
    let a = distance(v1, v2);
    let b = distance(v2, v3);
    let c = distance(v3, v1);
    let s = (a + b + c) / 2.0;
    (a * b * c) / (8.0 * (s - a) * (s - b) * (s - c))
}

/// `f` limited to `[low, high]`; a NaN `f` passes through.
pub(crate) fn clamp(f: f64, low: f64, high: f64) -> f64 {
    if f < low {
        low
    } else if f > high {
        high
    } else {
        f
    }
}
