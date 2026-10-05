//! geometry3Sharp's ray queries the AABB tree's `FindNearestHitTriangle` uses: `Ray3d`'s
//! direction normalization (`curve/Ray3.cs`), `IntrRay3Triangle3.Intersects`
//! (`intersection/IntrRay3Triangle3.cs`) and `IntrRay3AxisAlignedBox3.FindRayIntersectT` with
//! `IntrLine3AxisAlignedBox3.DoClipping` / `Clip` (`intersection/IntrLine3AxisAlignedBox3.cs`).

use super::dmesh3::AxisAlignedBox3d;

/// g3 `MathUtil.ZeroTolerance`.
const ZERO_TOLERANCE: f64 = 1e-08;
/// g3 `MathUtil.Epsilon`.
const EPSILON: f64 = 2.220_446_049_250_313e-16;

fn sub(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}

fn dot(a: [f64; 3], b: [f64; 3]) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

fn cross(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}

/// g3 `Ray3d`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct Ray3d {
    pub(crate) origin: [f64; 3],
    pub(crate) direction: [f64; 3],
}

impl Ray3d {
    /// C# `new Ray3d(origin, direction)`: the direction is normalized unless it already is
    /// (`IsNormalized`: squared length within `ZeroTolerance` of one).
    pub(crate) fn new(origin: [f64; 3], direction: [f64; 3]) -> Self {
        let mut direction = direction;
        if !is_normalized(direction) {
            let length = dot(direction, direction).sqrt();
            direction = if length > EPSILON {
                let inv = 1.0 / length;
                [direction[0] * inv, direction[1] * inv, direction[2] * inv]
            } else {
                [0.0; 3]
            };
        }
        Ray3d { origin, direction }
    }
}

/// C# `Vector3d.IsNormalized`.
pub(crate) fn is_normalized(v: [f64; 3]) -> bool {
    (dot(v, v) - 1.0).abs() < ZERO_TOLERANCE
}

/// C# `IntrRay3Triangle3.Intersects(ref ray, ref V0, ref V1, ref V2, out rayT)`: the ray
/// parameter of the hit, `None` for a miss (and for a ray parallel to the triangle).
pub(crate) fn ray_triangle_intersects(
    ray: &Ray3d,
    v0: [f64; 3],
    v1: [f64; 3],
    v2: [f64; 3],
) -> Option<f64> {
    // Compute the offset origin, edges, and normal.
    let diff = sub(ray.origin, v0);
    let edge1 = sub(v1, v0);
    let edge2 = sub(v2, v0);
    let normal = cross(edge1, edge2);

    // Solve Q + t*D = b1*E1 + b2*E2 (Q = kDiff, D = ray direction,
    // E1 = kEdge1, E2 = kEdge2, N = Cross(E1,E2)) by
    //   |Dot(D,N)|*b1 = sign(Dot(D,N))*Dot(D,Cross(Q,E2))
    //   |Dot(D,N)|*b2 = sign(Dot(D,N))*Dot(D,Cross(E1,Q))
    //   |Dot(D,N)|*t = -sign(Dot(D,N))*Dot(Q,N)
    let mut ddn = dot(ray.direction, normal);
    let sign;
    if ddn > ZERO_TOLERANCE {
        sign = 1.0;
    } else if ddn < -ZERO_TOLERANCE {
        sign = -1.0;
        ddn = -ddn;
    } else {
        // Ray and triangle are parallel, call it a "no intersection"
        // even if the ray does intersect.
        return None;
    }

    let dd_qx_e2 = sign * dot(ray.direction, cross(diff, edge2));
    if dd_qx_e2 >= 0.0 {
        let dd_e1x_q = sign * dot(ray.direction, cross(edge1, diff));
        if dd_e1x_q >= 0.0 && dd_qx_e2 + dd_e1x_q <= ddn {
            // Line intersects triangle, check if ray does.
            let qdn = -sign * dot(diff, normal);
            if qdn >= 0.0 {
                // Ray intersects triangle.
                let inv = 1.0 / ddn;
                return Some(qdn * inv);
            }
            // else: t < 0, no intersection
        }
        // else: b1 < 0 or b1+b2 > 1, no intersection
    }
    // else: b1 < 0, no intersection
    None
}

/// C# `IntrLine3AxisAlignedBox3.Clip`.
fn clip(denom: f64, numer: f64, t0: &mut f64, t1: &mut f64) -> bool {
    // Return value is 'true' if line segment intersects the current test
    // plane.  Otherwise 'false' is returned in which case the line segment
    // is entirely clipped.
    if denom > 0.0 {
        if numer - denom * *t1 > ZERO_TOLERANCE {
            return false;
        }
        if numer > denom * *t0 {
            *t0 = numer / denom;
        }
        true
    } else if denom < 0.0 {
        if numer - denom * *t0 > ZERO_TOLERANCE {
            return false;
        }
        if numer > denom * *t1 {
            *t1 = numer / denom;
        }
        true
    } else {
        numer <= 0.0
    }
}

/// C# `IntrRay3AxisAlignedBox3.FindRayIntersectT`: the entry parameter of the ray into the
/// (solid) box, `None` for a miss.
pub(crate) fn ray_box_intersect_t(ray: &Ray3d, bx: &AxisAlignedBox3d) -> Option<f64> {
    let (mut t0, mut t1) = (0.0, f64::MAX);
    let center: [f64; 3] = std::array::from_fn(|a| 0.5 * (bx.min[a] + bx.max[a]));
    let extent: [f64; 3] = std::array::from_fn(|a| (bx.max[a] - bx.min[a]) * 0.5);
    let b_origin = sub(ray.origin, center);
    let d = ray.direction;
    let not_all_clipped = clip(d[0], -b_origin[0] - extent[0], &mut t0, &mut t1)
        && clip(-d[0], b_origin[0] - extent[0], &mut t0, &mut t1)
        && clip(d[1], -b_origin[1] - extent[1], &mut t0, &mut t1)
        && clip(-d[1], b_origin[1] - extent[1], &mut t0, &mut t1)
        && clip(d[2], -b_origin[2] - extent[2], &mut t0, &mut t1)
        && clip(-d[2], b_origin[2] - extent[2], &mut t0, &mut t1);
    // `solid` is true for a ray query, so any unclipped ray hits
    if not_all_clipped {
        Some(t0)
    } else {
        None
    }
}
