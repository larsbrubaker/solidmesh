//! The GPU route's weights for sunlight through glass, in Rust: `path_sample_sun`,
//! `path_sun_toward`, `path_sun_toward_light`, `path_shadow_weight`, `path_sun_after` and the
//! sun's part of `path_escaped` in `sun.wgsl`, and the path loop's use of them in `path.wgsl`, in
//! the shader's `f32` operations and order, so a test can hold each against the CPU's rules
//! (`glass_chain.rs`, `sun_toward.rs`, `integrator.rs`, through their doc-hidden hooks) case by
//! case without a graphics card, as `sun_line.rs` does for the line the shadow ray walks. A test
//! pins the shader's text to these steps.
//!
//! **What the caller gives.** The scattering model's values at a direction ([`SunAtF32`]: whether
//! the frame and the triangle agree on its side, its local `z`, the BSDF and the two density parts
//! `surface_pdf_by_lobe` returns) come from a closure, and the line's walk ([`SunLineF32`]) from
//! another, called with whether it goes straight through glass. [`sample_sun_f32`] decides in the
//! shader's order which of them it asks and what it makes of the answers.

use super::sun_line::SunLineF32;
use crate::ray_tracer::LightStrategy;

type V3 = [f32; 3];

/// `LARGEST_F32` in `scatter.wgsl`.
const LARGEST_F32: f32 = 3.402_823_4e38;

/// `clean` in `scatter.wgsl`: `value` when positive and finite-looking, else 0.
fn clean(value: f32) -> f32 {
    if value > 0.0 && value < LARGEST_F32 {
        value
    } else {
        0.0
    }
}

fn times(a: V3, b: V3) -> V3 {
    [a[0] * b[0], a[1] * b[1], a[2] * b[2]]
}

fn by(a: V3, s: f32) -> V3 {
    [a[0] * s, a[1] * s, a[2] * s]
}

fn plus(a: V3, b: V3) -> V3 {
    [a[0] + b[0], a[1] + b[1], a[2] + b[2]]
}

/// `path_power_heuristic`.
pub fn power_heuristic_f32(pdf: f32, other: f32) -> f32 {
    if pdf > 0.0 {
        let ratio = other / pdf;
        if ratio >= 1e18 {
            return 0.0;
        }
        return 1.0 / (1.0 + ratio * ratio);
    }
    0.0
}

/// `path_shadow_weight`.
pub fn shadow_weight_f32(strategy: LightStrategy, sun_pdf: f32, pdf: f32) -> f32 {
    match strategy {
        LightStrategy::Combined => power_heuristic_f32(sun_pdf, pdf),
        LightStrategy::SurfaceSamplingOnly => 0.0,
        LightStrategy::NextEventOnly => 1.0,
    }
}

/// `SunToward` in `sun.wgsl`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SunTowardF32 {
    pub bsdf: V3,
    pub cos: f32,
    pub pdf_light: f32,
    pub radiance: V3,
    pub other_pdf: f32,
    /// The other lobes' share of the BSDF.
    pub other: f32,
    /// The glass lobe's share.
    pub glass: f32,
    pub glass_weight: f32,
}

/// `path_sun_toward`: `parts` are `surface_pdf_by_lobe`'s two uncleaned parts at local `wi_z`.
#[allow(clippy::too_many_arguments)]
pub fn sun_toward_f32(
    strategy: LightStrategy,
    wo_z: f32,
    wi_z: f32,
    parts: [f32; 2],
    bsdf: V3,
    pdf_light: f32,
    radiance: V3,
    chain: f32,
) -> SunTowardF32 {
    let through_surface = wo_z * wi_z < 0.0;
    let other_pdf = clean(parts[0]);
    let glass_pdf = clean(parts[1]);
    let (mut other, mut glass) = (1.0, 0.0);
    if through_surface {
        (other, glass) = (0.0, 1.0);
    } else if glass_pdf > 0.0 && other_pdf > 0.0 {
        let total = other_pdf + glass_pdf;
        (other, glass) = (other_pdf / total, glass_pdf / total);
    } else if glass_pdf > 0.0 {
        (other, glass) = (0.0, 1.0);
    }
    SunTowardF32 {
        bsdf,
        cos: wi_z.abs(),
        pdf_light,
        radiance,
        other_pdf,
        other,
        glass,
        glass_weight: shadow_weight_f32(strategy, pdf_light, glass_pdf) * chain,
    }
}

impl SunTowardF32 {
    /// `path_sun_wants_glass`.
    pub fn wants_glass(&self) -> bool {
        self.glass > 0.0 && self.glass_weight > 0.0
    }

    /// How `path_sample_sun` walks the line for these shares: `Some(true)` straight through glass,
    /// `Some(false)` stopped at any surface, `None` not at all.
    pub fn walk(&self) -> Option<bool> {
        if self.other <= 0.0 && !self.wants_glass() {
            None
        } else {
            Some(self.other > 0.0)
        }
    }

    /// `path_sun_toward_light`, for a line that reached the sun having passed `transmittance`
    /// and crossed `crossed` faces of glass.
    pub fn light(&self, strategy: LightStrategy, transmittance: V3, crossed: u32) -> V3 {
        let mut direct = [0.0; 3];
        if self.other > 0.0 {
            let weight = if crossed == 0 {
                shadow_weight_f32(strategy, self.pdf_light, self.other_pdf)
            } else {
                power_heuristic_f32(self.pdf_light, self.other_pdf)
            };
            if weight > 0.0 {
                let passed = if self.other < 1.0 {
                    by(transmittance, self.other)
                } else {
                    transmittance
                };
                direct = by(
                    times(times(self.bsdf, passed), self.radiance),
                    self.cos / self.pdf_light * weight,
                );
            }
        }
        if self.wants_glass() && crossed == 0 {
            let glass = by(
                times(times(self.bsdf, [self.glass; 3]), self.radiance),
                self.cos / self.pdf_light * self.glass_weight,
            );
            direct = plus(direct, glass);
        }
        direct
    }
}

/// The scattering model's values toward one world direction at a vertex, as `path_sample_sun`
/// reads them.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SunAtF32 {
    /// `path_sides_agree` for the direction.
    pub sides_agree: bool,
    /// The direction's local `z`.
    pub wi_z: f32,
    pub bsdf: V3,
    /// `surface_pdf_by_lobe`'s two parts.
    pub parts: [f32; 2],
    /// `surface_pdf`.
    pub pdf: f32,
}

/// The view's values `path_sample_sun` reads.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SunViewF32 {
    /// `PATH_FLAG_SUN_STRAIGHT`.
    pub straight: bool,
    pub has_sun: bool,
    pub strategy: LightStrategy,
    /// The sun's density (`sun_tangent.w`), 0 with no sun.
    pub sun_pdf: f32,
    pub sun_radiance: V3,
}

/// `SunShadow`, and how the line was walked (`None` not walked, else whether straight).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SunShadowF32 {
    pub light: V3,
    pub receiver_pdf: f32,
    pub walked: Option<bool>,
}

/// `path_sample_sun` at a vertex whose local `wo` has `wo_z`, the path's `chain` and `in_glass`,
/// and whether the vertex is at a seam or on a contact (`at_seam_or_contact`), for the sun
/// direction `line` the shader drew: `at` gives the model's values toward a world direction, and
/// `walk` the line's walk, straight through glass or not.
#[allow(clippy::too_many_arguments)]
pub fn sample_sun_f32(
    view: &SunViewF32,
    wo_z: f32,
    chain: f32,
    in_glass: bool,
    at_seam_or_contact: bool,
    line: V3,
    at: &mut dyn FnMut(V3) -> SunAtF32,
    walk: &mut dyn FnMut(bool) -> SunLineF32,
) -> SunShadowF32 {
    let mut shadow = SunShadowF32 {
        light: [0.0; 3],
        receiver_pdf: view.sun_pdf,
        walked: None,
    };
    let straight = view.straight;
    if !view.has_sun || (view.strategy == LightStrategy::SurfaceSamplingOnly && !straight) {
        return shadow;
    }
    let inside = straight && in_glass && !at_seam_or_contact;
    let mut toward_line = at(line);
    if !toward_line.sides_agree {
        return shadow;
    }
    if largest(toward_line.bsdf) <= 0.0 {
        return shadow;
    }
    let mut toward = None;
    let mut walk_straight = true;
    if straight && !inside {
        let shares = sun_toward_f32(
            view.strategy,
            wo_z,
            toward_line.wi_z,
            toward_line.parts,
            toward_line.bsdf,
            view.sun_pdf,
            view.sun_radiance,
            chain,
        );
        let Some(mode) = shares.walk() else {
            return shadow;
        };
        walk_straight = mode;
        toward = Some(shares);
    }
    shadow.walked = Some(walk_straight);
    let found = walk(walk_straight);
    if !straight {
        if !found.reached {
            return shadow;
        }
        let pdf_light = view.sun_pdf;
        let mut weight = 1.0;
        if view.strategy == LightStrategy::Combined {
            weight = power_heuristic_f32(pdf_light, toward_line.pdf);
        }
        shadow.light = by(
            times(toward_line.bsdf, view.sun_radiance),
            toward_line.wi_z.abs() / pdf_light * weight,
        );
        return shadow;
    }
    if inside {
        let bent = found.first_face.and_then(|(outward, ior)| {
            super::sun_line::sun_bent_f32(outward, ior, line).map(|bent| (ior, bent))
        });
        let mut direction = line;
        let mut radiance = view.sun_radiance;
        if let Some((ior, (bent_inside, cos_out, cos_in))) = bent {
            let ior2 = ior * ior;
            direction = bent_inside;
            shadow.receiver_pdf = view.sun_pdf * ior2 * cos_in / cos_out;
            radiance = by(view.sun_radiance, ior2);
        }
        if !found.reached {
            return shadow;
        }
        toward_line = at(direction);
        if bent.is_some() && !toward_line.sides_agree {
            return shadow;
        }
        if largest(toward_line.bsdf) <= 0.0 {
            return shadow;
        }
        toward = Some(sun_toward_f32(
            view.strategy,
            wo_z,
            toward_line.wi_z,
            toward_line.parts,
            toward_line.bsdf,
            shadow.receiver_pdf,
            radiance,
            chain,
        ));
    } else if !found.reached {
        return shadow;
    }
    if let Some(toward) = toward {
        shadow.light = toward.light(view.strategy, found.transmittance, found.crossed);
    }
    shadow
}

fn largest(c: V3) -> f32 {
    c[0].max(c[1]).max(c[2])
}

/// `PathSunState` in `sun.wgsl`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SunStateF32 {
    pub chain: f32,
    pub in_chain: bool,
    pub last_glass: bool,
    pub in_glass: bool,
}

impl SunStateF32 {
    /// The camera ray's record, in air.
    pub fn camera() -> SunStateF32 {
        SunStateF32 {
            chain: 1.0,
            in_chain: false,
            last_glass: false,
            in_glass: false,
        }
    }
}

/// What `path_sun_after` reads of a sampled event (`ScatterSample`).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SunEventF32 {
    pub is_specular: bool,
    pub from_glass: bool,
    /// The whole density.
    pub pdf: f32,
    /// The cleaned parts, other lobes and glass lobe (0 for a delta).
    pub other_pdf: f32,
    pub glass_pdf: f32,
    /// The sampled direction's local `z`.
    pub direction_z: f32,
}

/// `path_sun_after`.
pub fn sun_after_f32(
    state: SunStateF32,
    drawn: &SunEventF32,
    at_seam: bool,
    glass_surface: bool,
    receiver_pdf: f32,
) -> SunStateF32 {
    let mut next = state;
    if at_seam {
        next.in_glass = true;
    } else if glass_surface {
        next.in_glass = drawn.direction_z < 0.0;
    }
    if drawn.from_glass {
        next.in_chain = true;
        next.last_glass = true;
        return next;
    }
    let carried = if state.in_chain { state.chain } else { 1.0 };
    next.chain = if drawn.is_specular {
        carried
    } else {
        carried * power_heuristic_f32(drawn.other_pdf, receiver_pdf)
    };
    next.in_chain = next.in_glass;
    next.last_glass = false;
    next
}

/// The arrival's density the path loop keeps after `drawn`, `None` for a delta: the whole density,
/// or with `PATH_FLAG_SUN_STRAIGHT` the drawn lobe's own part.
pub fn arrival_pdf_f32(straight: bool, drawn: &SunEventF32) -> Option<f32> {
    if drawn.is_specular {
        return None;
    }
    if straight {
        return Some(if drawn.from_glass {
            drawn.glass_pdf
        } else {
            drawn.other_pdf
        });
    }
    Some(drawn.pdf)
}

/// The weight `path_escaped` gives the sun's disc for a ray that arrived with density `arrival`
/// (`None` a delta) in `state`.
pub fn escaped_sun_weight_f32(
    strategy: LightStrategy,
    arrival: Option<f32>,
    sun_pdf: f32,
    state: &SunStateF32,
) -> f32 {
    let mut weight = 1.0;
    if let Some(pdf) = arrival {
        if strategy != LightStrategy::SurfaceSamplingOnly {
            weight = if strategy == LightStrategy::NextEventOnly {
                0.0
            } else {
                power_heuristic_f32(pdf, sun_pdf)
            };
        }
    }
    if state.last_glass {
        weight *= state.chain;
    }
    weight
}
