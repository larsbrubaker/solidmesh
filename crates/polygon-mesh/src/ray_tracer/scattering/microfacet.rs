//! The formulas the scattering model is built from: the GGX microfacet distribution,
//! height-correlated Smith masking-shadowing, visible-normal sampling, Schlick and exact dielectric
//! Fresnel, reflection and refraction about a microfacet normal. Pure functions on unit vectors in
//! the local frame (+z up); `model` combines them into a surface. Part of `scattering`.

use std::f64::consts::PI;

use solidmesh_math::Vector3;

/// A direction this close to the horizon scatters nothing: its cosine is zero to any purpose and
/// the formulas below divide by it.
pub(super) const MIN_COS: f64 = 1e-9;

/// The smallest GGX width used, small enough to look like a mirror; below it a surface is smooth
/// and reflects as a delta lobe.
pub(super) const MIN_ALPHA: f64 = 1e-3;

/// Glass with an index of refraction within this of 1 is treated as a smooth interface, whatever
/// its roughness: a microfacet there would bend a ray by about this many radians and the
/// half-vector formulas turn singular at no index step. Fresnel stays the exact flat-interface
/// value, so it is not neglected where it matters near grazing, where even this small an index step
/// reflects strongly (at cos 1e-3 and IOR 1.0005, about 0.9).
pub(super) const INDEX_MATCHED: f64 = 1e-3;

/// `v` scaled to unit length, or `None` when it has no direction. Scales by the largest component
/// first so that neither huge nor subnormal input loses its direction in the squares.
pub(super) fn unit_direction(v: Vector3) -> Option<Vector3> {
    if !(v.x.is_finite() && v.y.is_finite() && v.z.is_finite()) {
        return None;
    }
    let largest = v.x.abs().max(v.y.abs()).max(v.z.abs());
    if largest <= 0.0 {
        return None;
    }
    let scaled = v / largest;
    let length = scaled.length_squared().sqrt();
    if length > 0.0 {
        Some(scaled / length)
    } else {
        None
    }
}

/// The GGX density of microfacet normal `h`. Written with `sin^2` kept apart from `cos^2` so the
/// denominator stays exact for a narrow lobe at the top of the hemisphere, where the usual
/// `cos^2 * (alpha^2 - 1) + 1` would round to zero.
pub(super) fn ggx_density(h: Vector3, alpha: f64) -> f64 {
    let sin2 = h.x * h.x + h.y * h.y;
    let cos2 = h.z * h.z;
    let alpha2 = alpha * alpha;
    let denominator = sin2 + alpha2 * cos2;
    alpha2 / (PI * denominator * denominator)
}

/// The Smith auxiliary term of a direction with cosine `cos_theta > 0`.
pub(super) fn smith_lambda(cos_theta: f64, alpha: f64) -> f64 {
    let cos2 = cos_theta * cos_theta;
    let tan2 = (1.0 - cos2) / cos2;
    ((1.0 + alpha * alpha * tan2).sqrt() - 1.0) * 0.5
}

/// The fraction of microfacets visible from a direction with cosine `cos_theta`.
pub(super) fn smith_masking(cos_theta: f64, alpha: f64) -> f64 {
    1.0 / (1.0 + smith_lambda(cos_theta, alpha))
}

/// The fraction of microfacets visible from both directions at once, accounting for a facet high
/// on the surface being more likely to be seen from both. Exact for reflection; used as an
/// approximation for refraction (see `transmitted_glass`), where it stays at or below either
/// direction's own masking.
pub(super) fn smith_masking_shadowing(cos_o: f64, cos_i: f64, alpha: f64) -> f64 {
    1.0 / (1.0 + smith_lambda(cos_o, alpha) + smith_lambda(cos_i, alpha))
}

/// The density, over microfacet normals, of the normals `wo` can see. `wo` is in the upper
/// hemisphere.
pub(super) fn visible_normal_density(wo: Vector3, h: Vector3, alpha: f64) -> f64 {
    let cos_h = wo.dot(h).max(0.0);
    smith_masking(wo.z, alpha) * cos_h * ggx_density(h, alpha) / wo.z
}

/// Draws a microfacet normal, in the upper hemisphere, from the normals `wo` can see. Stretching
/// the surface so that it has width 1 turns the visible normals into a uniform spherical cap
/// around the stretched view direction; unstretching the drawn point gives the normal.
pub(super) fn sample_visible_normal(wo: Vector3, alpha: f64, u: f64, v: f64) -> Option<Vector3> {
    let stretched = unit_direction(Vector3::new(alpha * wo.x, alpha * wo.y, wo.z))?;
    let phi = 2.0 * PI * u;
    let z = (1.0 - v) * (1.0 + stretched.z) - stretched.z;
    let sin_theta = (1.0 - z * z).max(0.0).sqrt();
    let on_cap = Vector3::new(sin_theta * libm::cos(phi), sin_theta * libm::sin(phi), z);
    let half = on_cap + stretched;
    unit_direction(Vector3::new(
        alpha * half.x,
        alpha * half.y,
        half.z.max(0.0),
    ))
}

/// `w` mirrored about `h`.
pub(super) fn reflect(w: Vector3, h: Vector3) -> Vector3 {
    h * (2.0 * w.dot(h)) - w
}

/// `w` refracted through the plane with normal `h`, where `w` and `h` are on the same side and
/// `eta_ratio` is the side of `w`'s index over the other's; `None` at total internal reflection.
pub(super) fn refract(w: Vector3, h: Vector3, eta_ratio: f64) -> Option<Vector3> {
    let cos_incident = w.dot(h);
    let sin2_transmit = eta_ratio * eta_ratio * (1.0 - cos_incident * cos_incident);
    if sin2_transmit >= 1.0 {
        return None;
    }
    let cos_transmit = (1.0 - sin2_transmit).sqrt();
    Some(h * (eta_ratio * cos_incident - cos_transmit) - w * eta_ratio)
}

/// Schlick's reflectance at incidence cosine `cos_theta`, from `f0` at normal incidence to `f90`
/// at grazing.
pub(super) fn schlick(f0: f64, f90: f64, cos_theta: f64) -> f64 {
    f0 + (f90 - f0) * fifth_power(1.0 - cos_theta.clamp(0.0, 1.0))
}

/// `value` to the fifth by plain multiplication in a fixed order: a library `powi` makes no
/// promise about its last bit across platforms or versions, and an albedo table cell depends on
/// every bit of this value.
pub(super) fn fifth_power(value: f64) -> f64 {
    let square = value * value;
    value * (square * square)
}

/// The grazing reflectance a Schlick `f0` goes to: 1 for anything with a real index step, falling
/// to 0 with `f0` so an interface that does not exist does not reflect.
pub(super) fn grazing_reflectance(f0: f64) -> f64 {
    (50.0 * f0).clamp(0.0, 1.0)
}

/// The exact Fresnel reflectance of a dielectric interface, with `cos_incident` measured on the
/// `eta_from` side. 1 at and past the critical angle.
pub(super) fn dielectric_fresnel(cos_incident: f64, eta_from: f64, eta_to: f64) -> f64 {
    let cos_i = cos_incident.abs().min(1.0);
    let ratio = eta_from / eta_to;
    let sin2_t = ratio * ratio * (1.0 - cos_i * cos_i);
    if sin2_t >= 1.0 {
        return 1.0;
    }
    let cos_t = (1.0 - sin2_t).sqrt();
    let perpendicular = (eta_from * cos_i - eta_to * cos_t) / (eta_from * cos_i + eta_to * cos_t);
    let parallel = (eta_to * cos_i - eta_from * cos_t) / (eta_to * cos_i + eta_from * cos_t);
    0.5 * (parallel * parallel + perpendicular * perpendicular)
}
