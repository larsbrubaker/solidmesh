//! What a vertex's BSDF sends toward one sun direction under [`SunThroughGlass::Straight`], in
//! its two shares (`glass_chain` says why): the other lobes' share goes straight through glass,
//! the glass lobe's counts only a line that crossed nothing, scaled by the chain's share of exact
//! transport. `integrator`'s `sample_sun` builds a [`Toward`] for its sun direction, asks it how
//! far to walk the line (`sun_through_glass`) and what the line brings; a receiver inside glass
//! first reads the sun bent at the glass's face (`bent_sun`).
//!
//! The doc-hidden hooks at the end ([`sun_toward_at`], [`SunPathAt`] and its steps) run these
//! rules and `glass_chain`'s record on values a caller gives, so a mirror of them in another
//! precision (the GPU route's `flat_tree/sun_weights.rs`) can be checked against them case by
//! case. They are test hooks, not part of the stable interface.
//!
//! [`SunThroughGlass::Straight`]: super::SunThroughGlass::Straight

use super::glass_chain::{power_heuristic, shadow_weight, Event, SunPath};
use super::integrator::LightStrategy;
use super::sun_through_glass::{LineMode, SunReach};

fn multiply(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[0] * b[0], a[1] * b[1], a[2] * b[2]]
}

fn scaled(a: [f64; 3], s: f64) -> [f64; 3] {
    [a[0] * s, a[1] * s, a[2] * s]
}

fn add_to(sum: &mut [f64; 3], add: [f64; 3]) {
    sum[0] += add[0];
    sum[1] += add[1];
    sum[2] += add[2];
}

/// One sun direction's two shares at a vertex, with what each share's term needs.
pub(super) struct Toward {
    bsdf: [f64; 3],
    /// The direction's cosine at the vertex.
    cos: f64,
    /// The sun's density along the direction.
    pdf_light: f64,
    /// The sun's radiance along the direction.
    radiance: [f64; 3],
    other_pdf: f64,
    shares: LobeShares,
    glass_weight: f64,
}

impl Toward {
    /// The shares toward local direction `wi_z` (of the vertex's `wo_z`), from the surface's
    /// cleaned density parts there (`lobe_densities`), `chain` the path's (`SunPath::chain`).
    #[allow(clippy::too_many_arguments)]
    pub(super) fn of(
        strategy: LightStrategy,
        wo_z: f64,
        wi_z: f64,
        bsdf: [f64; 3],
        pdf_light: f64,
        radiance: [f64; 3],
        (other_pdf, glass_pdf): (f64, f64),
        chain: f64,
    ) -> Toward {
        let through_surface = wo_z * wi_z < 0.0;
        Toward {
            bsdf,
            cos: wi_z.abs(),
            pdf_light,
            radiance,
            other_pdf,
            shares: lobe_shares(other_pdf, glass_pdf, through_surface),
            glass_weight: shadow_weight(strategy, pdf_light, glass_pdf) * chain,
        }
    }

    fn wants_glass(&self) -> bool {
        self.shares.glass > 0.0 && self.glass_weight > 0.0
    }

    /// How far the line is walked: straight through glass when the other lobes' share wants it,
    /// stopped at any surface when only the glass share does, not at all when neither does.
    pub(super) fn mode(&self) -> Option<LineMode> {
        if self.shares.other > 0.0 {
            Some(LineMode::Straight)
        } else if self.wants_glass() {
            Some(LineMode::StopAtAny)
        } else {
            None
        }
    }

    /// The light a line that reached the sun brings.
    pub(super) fn light(&self, strategy: LightStrategy, reach: &SunReach) -> Option<[f64; 3]> {
        // Each share's term is `bsdf * passed * radiance * (cos / pdf_light * weight)`, the one
        // share a surface with a single kind of lobe has taking the plain shadow ray's operations.
        let term = |passed: [f64; 3], weight: f64| {
            scaled(
                multiply(multiply(self.bsdf, passed), self.radiance),
                self.cos / self.pdf_light * weight,
            )
        };
        let mut direct: Option<[f64; 3]> = None;
        if self.shares.other > 0.0 {
            let weight = if reach.crossed == 0 {
                shadow_weight(strategy, self.pdf_light, self.other_pdf)
            } else {
                power_heuristic(self.pdf_light, self.other_pdf)
            };
            if weight > 0.0 {
                let passed = if self.shares.other < 1.0 {
                    scaled(reach.transmittance, self.shares.other)
                } else {
                    reach.transmittance
                };
                direct = Some(term(passed, weight));
            }
        }
        if self.wants_glass() && reach.crossed == 0 {
            let glass = term([self.shares.glass; 3], self.glass_weight);
            direct = Some(match direct {
                Some(mut sum) => {
                    add_to(&mut sum, glass);
                    sum
                }
                None => glass,
            });
        }
        direct
    }
}

/// How much of the BSDF toward a direction each kind of lobe holds (`glass_chain`).
struct LobeShares {
    other: f64,
    glass: f64,
}

/// The shares the two densities give: in proportion where both are there, all to the one that
/// is, all to the other lobes where neither is (a delta-only glass lobe puts nothing toward a
/// sampled direction), and all to glass through the surface, which only glass crosses.
fn lobe_shares(other_pdf: f64, glass_pdf: f64, through_surface: bool) -> LobeShares {
    if through_surface {
        LobeShares {
            other: 0.0,
            glass: 1.0,
        }
    } else if glass_pdf > 0.0 && other_pdf > 0.0 {
        let total = other_pdf + glass_pdf;
        LobeShares {
            other: other_pdf / total,
            glass: glass_pdf / total,
        }
    } else if glass_pdf > 0.0 {
        LobeShares {
            other: 0.0,
            glass: 1.0,
        }
    } else {
        LobeShares {
            other: 1.0,
            glass: 0.0,
        }
    }
}

/// The sun as a receiver inside glass of index `ior` reads it through the face its line leaves by
/// (`glass_chain`, "A receiver inside glass"): the disc seen inside, `n^2 cos_in / cos_out` times
/// denser than the sun's `sun_pdf`, at `n^2` times its radiance.
pub(super) fn bent_sun(
    sun_pdf: f64,
    radiance: [f64; 3],
    ior: f64,
    cos_in: f64,
    cos_out: f64,
) -> (f64, [f64; 3]) {
    let ior_squared = ior * ior;
    (
        sun_pdf * ior_squared * cos_in / cos_out,
        scaled(radiance, ior_squared),
    )
}

/// Whether the ray leaving an event travels inside glass (`glass_chain`): both sides of a seam
/// are glass, a surface with a glass share (`transmissive`) has its true outside as +z, so the
/// sampled direction's `z` says, and an opaque part leaves the ray in what it arrived through.
pub(super) fn in_glass_after(
    in_glass: bool,
    at_seam: bool,
    transmissive: bool,
    direction_z: f64,
) -> bool {
    if at_seam {
        true
    } else if transmissive {
        direction_z < 0.0
    } else {
        in_glass
    }
}

/// What [`sun_toward_at`] reports: the two shares, the glass share's weight (`chain` in), how far
/// the line is walked (`Some(true)` straight through glass, `Some(false)` stopped at any surface,
/// `None` not walked) and, for a line that reached the sun, what it brings.
#[doc(hidden)]
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SunTowardAt {
    pub other_share: f64,
    pub glass_share: f64,
    pub glass_weight: f64,
    pub walk: Option<bool>,
    pub light: Option<[f64; 3]>,
}

/// The CPU's shares and shadow term for one sun direction at a vertex: the surface's cleaned
/// density parts `densities` (other lobes, glass lobe) toward local `wi_z` from `wo_z`, the
/// BSDF there, the sun's density and radiance along it, the path's `chain`, and what the line
/// found (`reach`: transmittance and faces crossed, `None` blocked). A test hook.
#[doc(hidden)]
#[allow(clippy::too_many_arguments)]
pub fn sun_toward_at(
    strategy: LightStrategy,
    wo_z: f64,
    wi_z: f64,
    bsdf: [f64; 3],
    pdf_light: f64,
    radiance: [f64; 3],
    densities: (f64, f64),
    chain: f64,
    reach: Option<([f64; 3], u32)>,
) -> SunTowardAt {
    let toward = Toward::of(
        strategy, wo_z, wi_z, bsdf, pdf_light, radiance, densities, chain,
    );
    let light = reach.and_then(|(transmittance, crossed)| {
        toward.light(
            strategy,
            &SunReach {
                transmittance,
                crossed,
            },
        )
    });
    SunTowardAt {
        other_share: toward.shares.other,
        glass_share: toward.shares.glass,
        glass_weight: toward.glass_weight,
        walk: toward.mode().map(|mode| mode == LineMode::Straight),
        light,
    }
}

/// The sun's density and radiance a receiver inside glass reads through a face of index `ior`
/// its line leaves at `cos_out`, the bent direction's cosine inside being `cos_in`. A test hook.
#[doc(hidden)]
pub fn bent_sun_at(
    sun_pdf: f64,
    radiance: [f64; 3],
    ior: f64,
    cos_in: f64,
    cos_out: f64,
) -> (f64, [f64; 3]) {
    bent_sun(sun_pdf, radiance, ior, cos_in, cos_out)
}

/// `glass_chain`'s record of how a path got to its ray, and whether that ray travels inside
/// glass, as plain fields. A test hook.
#[doc(hidden)]
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SunPathAt {
    /// Exact's arrival: `Some(pdf)` sampled, `None` the camera or a delta event.
    pub arrival: Option<f64>,
    pub chain: f64,
    pub in_chain: bool,
    pub last_glass: bool,
    pub in_glass: bool,
}

impl SunPathAt {
    /// The camera ray, in air.
    pub fn camera() -> SunPathAt {
        SunPathAt::of(SunPath::camera(), false)
    }

    fn of(path: SunPath, in_glass: bool) -> SunPathAt {
        let (arrival, chain, in_chain, last_glass) = path.parts();
        SunPathAt {
            arrival,
            chain,
            in_chain,
            last_glass,
            in_glass,
        }
    }

    /// The record after an event the path loop drew (`is_specular`, `from_glass`, its whole
    /// density `pdf` and cleaned parts `densities`, its local direction's `direction_z`) at a
    /// vertex `at_seam` or on a `transmissive` surface, `straight` the render's, `receiver_pdf`
    /// the sun's density the vertex's own weight was taken against: `in_glass` first, then
    /// `SunPath::after`, as `trace_path` writes them.
    #[allow(clippy::too_many_arguments)]
    pub fn after(
        self,
        straight: bool,
        is_specular: bool,
        from_glass: bool,
        pdf: f64,
        densities: (f64, f64),
        direction_z: f64,
        at_seam: bool,
        transmissive: bool,
        receiver_pdf: f64,
    ) -> SunPathAt {
        let event = Event::of_draw(straight, is_specular, from_glass, pdf, densities);
        let in_glass = in_glass_after(self.in_glass, at_seam, transmissive, direction_z);
        let path = SunPath::from_parts(self.arrival, self.chain, self.in_chain, self.last_glass);
        SunPathAt::of(
            path.after(event, straight, receiver_pdf, in_glass),
            in_glass,
        )
    }

    /// The weight of the sun's disc for a ray in this state that leaves the scene into it.
    pub fn sun_weight(self, strategy: LightStrategy, sun_pdf: f64) -> f64 {
        SunPath::from_parts(self.arrival, self.chain, self.in_chain, self.last_glass)
            .sun_weight(strategy, sun_pdf)
    }
}
