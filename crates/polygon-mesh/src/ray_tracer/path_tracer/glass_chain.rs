//! [`SunThroughGlass`], the render setting that says how sunlight reaches what lies behind glass,
//! and the record a path keeps of how it got to its current ray (`SunPath`), which weighs the sun's
//! disc when the path leaves the scene and the glass share of a vertex's shadow ray. `integrator`
//! asks this file at every event; `sun_through_glass` walks the line toward the sun.
//!
//! **Exact** is the plain path tracer: a shadow ray stops at any surface, glass included, and the
//! sun behind glass is found only by a bounce that refracts into its disc, weighted against the
//! shadow ray as everywhere. It is the reference, and the one a render that draws light focused
//! by glass (caustics) would build on. The floor under glass is then lit by rare bright samples.
//!
//! **Straight** (the default) mixes two routes by which sunlight reaches a receiver through
//! glass: the straight line, unbent, times each face's `straight_transmittance` (smooth, no
//! specks), and exact transport through the glass, the plain path tracer's. A **receiver** is the
//! camera or any event drawn from a lobe other than glass (diffuse, glossy, a mirror); an event
//! drawn from a surface's glass lobe (a reflection or refraction off its glass share, smooth or
//! frosted: `ScatteringSurface::sample_by_lobe` says which lobe drew it) is never a receiver: it
//! carries light toward the camera for the nearest receiver before it. The two routes are a
//! partition of unity, weighted by the power heuristic densities the receiver `x` already has:
//!
//! `sun through glass at x = w_n * straight + w_b * exact`, `w_n = PH(p_sun, p_o(d'))` for a
//! straight line along `d'`, `w_b = PH(p_o(d), p_sun)` for the bounce along `d` that meets glass,
//!
//! where `p_o` is the density of `x`'s lobes other than glass and `p_sun` the sun's. A delta
//! receiver (a mirror, the camera) has no shadow ray and `w_b = 1`: what it sees through glass is
//! Exact's. A diffuse receiver under a 2.5 degree sun has `w_b <= (0.318 / 167)^2`, about 3.6e-6,
//! so it gets the straight line and the light that only arrives as specks is gone to five digits.
//!
//! **A surface partly glass** splits as the scattering model does (`pdf_by_lobe`): `p = p_o +
//! p_g`, the other lobes' and the glass lobe's densities, and the BSDF `f` toward a direction is
//! shared in proportion, `f p_o / p` and `f p_g / p`. With no glass density there the other lobes
//! hold all of `f`; with no other density, or through the surface, the glass lobe does.
//!
//! **State.** Exact's arrival (`Sampled { pdf }` or `Delta`), `chain` (starting at 1),
//! `in_chain` and `last_glass` (both starting false); `carried` is `chain` when `in_chain`, else
//! 1. After an event:
//! 1. another lobe, sampled with density `p_o`: `chain = carried * PH(p_o, p_sun)` whatever the
//!    `LightStrategy` (a model weight, not a strategy weight), `Sampled { p_o }`, `last_glass`
//!    false;
//! 2. another lobe, delta: `chain = carried`, `Delta`, `last_glass` false;
//! 3. the glass lobe, any sample: `chain` kept, `in_chain` true, `Sampled { p_g }` or `Delta`,
//!    `last_glass` true;
//!
//! and after another lobe's event `in_chain` is whether the ray leaving it travels inside glass
//! (`integrator` follows that: the camera is in air, a surface with a glass share is left on the
//! side the ray goes, an opaque part leaves the ray in what it arrived through, a seam is glass on
//! both sides).
//!
//! **When each is written and read.** The state and `in_glass` are written once per scattering
//! event, after that vertex's shadow ray and before the next ray: `in_glass` first, then
//! `after`, which reads it. Nothing else writes them, so a step that moves the path with no
//! event keeps them: a pass through a part not there, a pass straight through a seam, the
//! landing on a surface past a see-through part resting on it (`onto_rest`), a traced hit taken
//! as the other face of such a contact (`met_first`), and the step across a contact to the face
//! at the same point. So a floor vertex a line reached past a see-through part, glass or not, is
//! in air when the line came from air (the part was never entered); one reached from
//! see-through glass found there, by its exit event, is in air because that event refracted out
//! of the glass's face; and the face of a part reached across a contact from the floor below is
//! met from air. The vertex's shadow ray reads `chain` (the glass lobe's share), and `in_glass`
//! with no seam and no contact partner makes it a receiver inside glass (below); an escape
//! reads `arrival`, `chain` and `last_glass` (`sun_weight`).
//!
//! **What `chain` is.** A *chain* starts at a receiver whose light the path follows into glass,
//! takes in every glass event and every receiver reached through glass or inside it after that,
//! and ends at the first receiver reached along a ray in open air with no glass event since,
//! which starts afresh (`carried = 1`). `chain` is the product of the `w_b` of the receivers in
//! it: the share of exact transport through glass the light reaching the nearest receiver keeps,
//! times the share the receiver before it keeps of the light reaching it, back to the chain's
//! start. A receiver in a chain (a plastic share inside a pane, a part inside a glass block, a
//! wall seen through a pane) thus passes on its own `w_b` times the chain's. Its own straight
//! line, at its own `w_n`, and its light along a clear line (Exact's pair) are not scaled.
//!
//! On escape into the sun's disc the weight is Exact's for the arrival ("weighted" is the power
//! heuristic against `p_sun` under [`LightStrategy::Combined`], 1 under `SurfaceSamplingOnly`, 0
//! under `NextEventOnly`; 1 for `Delta`), times `chain` when `last_glass`: only a ray a glass event
//! sent is exact transport through glass. A ray a receiver sent along a line that leaves into the
//! sun crossing nothing is the escape half of Exact's clear-line pair, whatever `in_glass` says:
//! glass partly there entered by a face found there and left by one found not there, a seam where
//! only one part is there, an open or single-sided glass mesh and inverted winding all leave
//! `in_glass` true in open air, and there the pair must still sum to 1. In closed glass that is
//! all there a ray inside cannot leave without a glass event, so `last_glass` and `in_chain` agree
//! there on every escape. A vertex's shadow ray is one sun direction and one line walked straight
//! through glass (`sun_through_glass`), which says whether it reached the sun, what it passed
//! (`T`) and how many faces it crossed:
//! - the other lobes' share counts `T` times Exact's strategy weight against `p_o` when the line
//!   crossed nothing, and `T * PH(p_sun, p_o)` under every strategy when it crossed glass (so a
//!   `SurfaceSamplingOnly` render still walks the line in a scene with glass);
//! - the glass lobe's share counts only when the line reached the sun crossing nothing, which is
//!   Exact's stop-at-any shadow ray read from the same walk, times Exact's strategy weight against
//!   `p_g`, times `chain`. A vertex with no other share asks the walk only that.
//!
//! **A receiver inside glass** (its arriving ray travels inside glass, not at a seam or a contact)
//! sees the sun through the glass's face along the bent direction, not the line. Its line `d'` is
//! walked first; where the walk's first trace finds a face of glass crossed from inside its solid
//! (hit on its back) with outward shading normal `N` and index `n`, the receiver reads the sun
//! along `d = refract(d', N, 1 / n)`, the direction inside that leaves along `d'` (`cos_out =
//! N.d'`, `cos_in = N.d`), over the disc as seen inside, `p_sun_in = p_sun n^2 cos_in / cos_out`
//! (from `dw_in cos_in n^2 = dw_out cos_out`), at the radiance inside, `n^2 L`. Its straight term
//! is `f(d) |cos_x(d)| T n^2 L / p_sun_in * PH(p_sun_in, p_o(d))`, and the `w_b` that sets `chain`
//! there is `PH(p_o, p_sun_in)`. For a broad lobe, `f(d) = f(d')`, this is `f |cos_x(d)| T L
//! cos_out / (p_sun cos_in)`, which for a receiver parallel to the face is today's unbent `f
//! cos_out T L / p_sun`: the irradiance `n^2 L dw_in cos_in = L dw_out cos_out` is the same, so a
//! matte part inside glass keeps its light (measured: 21.4166 against 21.4165 straight down). The
//! line beyond that face is still walked unbent (transmittance and blocking as before), and `T`
//! keeps the Fresnel at the line's angle. It is exact for a flat first face; at a curved one it
//! bends at the plane touching the face where the line crosses it, so it reads the lobe along the
//! right direction but not the focusing or spreading the curve does. A first hit that is not glass
//! crossed from inside (blocked, a face entered, a seam, a face the start crossed) leaves the line
//! unbent and `p_sun_in = p_sun`. A receiver in open air (`in_glass` false) never asks, so it reads
//! as before to the bit. The receiver's `w_b` takes `p_sun_in` from its own sun sample's face, a
//! disc's width from the bounce's direction, so the two weights sum to 1 only to within how much
//! `cos_in / cos_out` varies across the disc.
//!
//! **Each light path is counted once or dropped.** Per family, at a receiver `x` and a direction:
//! - Other lobes, clear line: Exact's pair, the shadow ray and the escape, sums to 1; a scene
//!   with no glass takes Exact's path to the bit.
//! - Other lobes, line through glass: the shadow ray alone at `w_n`; the bounce along that
//!   direction meets glass and is the next family, at `w_b`.
//! - Glass lobe, with receiver `x` before it: Exact's own pair at that glass vertex (its
//!   stop-at-any shadow ray against its escape, or a delta's escape at 1), all scaled by
//!   `chain`: `w_b(x)` when `x` starts the chain.
//! - Another lobe's delta through glass: no shadow ray exists, so `w_n = 0`, and the exact route
//!   carries `carried`, the limit of the sharp glossy case.
//! - The camera: `chain = 1`, which is Exact; a chain of glass events alone keeps it at 1.
//! - Blocked: no one counts it.
//!
//! Dropped, by the product: light that reaches a receiver inside a chain by exact transport
//! through glass keeps only the chain's share of it. Under a pane that is the floor, refraction
//! in, reflections off the pane's plastic share, refraction out, the sun (internal back and forth,
//! which arrived as specks), and the floor, glass, a mirror inside glass or behind more glass,
//! glass, the sun (a mirror's caustic seen through glass). The floor, glass, a mirror in open air,
//! the sun is kept as Exact keeps it: the mirror's ray leaves into open air, so it starts afresh.
//!
//! **Why nothing steps.** A branch is safe only where its two sides agree in the limit. The lobe
//! branch: a lobe's share goes to 0 as transmission or metallic does. Delta against sampled:
//! `PH(p, p_sun) -> 1` as `p` grows, so a sharp glossy receiver's `chain` meets the mirror's
//! `carried` (inside a chain too: `carried * PH(p, p_sun) -> carried`) and its `w_n` meets 0.
//! Whether a ray travels inside glass does not depend on any slider. "Crossed glass": `T` goes to
//! the open line's value as the glass share does, and the two routes coincide as glass stops
//! bending (index-matched glass reads Exact's mean).
//!
//! **Known limits**, measured (2.5 degree sun unless named):
//! 1. Just above another lobe's smooth threshold its tail, where `p_o < p_sun`, takes the straight
//!    estimate while a delta has no tail. Under a clear pane, roughness 0.031 to 0.032, metal and
//!    plastic floors with the sun at 0, 60 and 75 degrees step -0.15% to -0.42% where Exact steps
//!    -0.15% to -0.23% (all within 1.1 standard errors).
//! 2. The straight line is unbent behind glass, and bent only at the first face for a receiver
//!    inside it. A metal cube inside a flat-faced clear block (`ior` 1.5) reads Exact's glint
//!    within noise at every roughness (sun and view 30 degrees, 0.031 to 0.5: z within +-1.0;
//!    overhead within +-2.3). Inside a glass ball of radius 3 the curved face focuses what the
//!    bent line does not: against Exact, straight down, -2.9% at roughness 0.032, -7.5% at 0.05,
//!    -19% at 0.08, -37% at 0.12, -48% at 0.2, -47% at 0.3, -28% at 0.5. Behind bending glass the
//!    unbent line gives a metal 0.3 floor under a glass ball +40% (polished +5%), a metal 0.3 floor
//!    under a frosted 0.3 pane at 30 degrees -16%, a matte floor there +6.6%.
//! 3. How much glass-first light is kept depends on the sun's size, by design (`p_sun`).
//! 4. Glints of a sharp sampled lobe other than glass seen through glass are kept at its `w_n`:
//!    over a matte floor under a pane half (three quarters) glass and the rest plastic, sun 60
//!    degrees off straight up with a 10 degree radius, 65536 samples of 8 bounces, samples over
//!    0.5 (the floor reads 0.033, 0.064): 0 at roughness 0 and 0.031, 2 (1) at 0.032, 4 (4) at
//!    0.05, 8 (7) at 0.1, 0 (0) at 0.3, the largest 1.38 (1.82). The plastic share inside the
//!    pane reads the sun bent at the pane's far face, over a disc `n^2 cos_in / cos_out` times
//!    denser, so its sharp tail keeps more of the line than the unbent line gave it (0 (1), 2 (2),
//!    2 (12), largest 0.71 (1.08)).
//! 5. A frosted part under a clear cover, seen by the camera: its glass share's line stops at the
//!    cover, so it is lit through the exact route only, as under Exact.
//!
//! **Bound.** A sample that reaches the sun through glass from a receiver with density `p` toward
//! it carries at most about `L PH(p, p_sun)` of the sun's radiance `L` times what its path passed.
//!
//! Exact's state follows Exact's rules from every state (`after` with `straight` false): every
//! event with a density leads to `Sampled` by the whole density, every delta to `Delta`, and
//! `chain` stays 1 and unused.

use super::integrator::LightStrategy;

/// How sunlight reaches what lies behind glass: see the header of this module.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum SunThroughGlass {
    /// The sun reaches what lies behind glass mostly along its straight, unbent line, dimmed and
    /// tinted at each face, and through the glass itself only as much as the surface it lights
    /// would find it by its own sharp reflection: smooth light under glass, mirrors and the
    /// camera seeing through glass as Exact does.
    #[default]
    Straight,
    /// The plain path tracer: shadow rays stop at glass, and the sun behind it is found only by
    /// paths that refract into it. The reference, and the base for focused light (caustics).
    Exact,
}

/// How the path got to the current ray, as Exact weighs the sun's disc on escape.
#[derive(Clone, Copy, Debug, PartialEq)]
pub(super) enum Arrival {
    /// The last event had density `pdf` toward the ray.
    Sampled { pdf: f64 },
    /// The camera ray, or a delta event last.
    Delta,
}

/// A sampled scattering event, as the rules read it. `pdf` is the density the path's escape into
/// the sun is weighted by: the whole density in Exact, its lobe's part in Straight.
#[derive(Clone, Copy, Debug, PartialEq)]
pub(super) enum Event {
    Scatter {
        pdf: f64,
    },
    /// `pdf` is `None` for a delta event.
    Glass {
        pdf: Option<f64>,
    },
    Mirror,
}

impl Event {
    /// The event of a sample with density `pdf` (a choice probability for a delta lobe), delta
    /// or not, drawn from the glass lobe or not.
    pub(super) fn of(is_specular: bool, from_glass: bool, pdf: f64) -> Event {
        match (from_glass, is_specular) {
            (true, true) => Event::Glass { pdf: None },
            (true, false) => Event::Glass { pdf: Some(pdf) },
            (false, true) => Event::Mirror,
            (false, false) => Event::Scatter { pdf },
        }
    }

    /// The event of a sample the path loop drew, `pdf` its whole density and `densities` its
    /// cleaned parts (other lobes, glass lobe): in Straight a sampled event is weighted on escape
    /// by its own lobe's part, which is what the vertex's shadow ray weighs that lobe's share
    /// against.
    pub(super) fn of_draw(
        straight: bool,
        is_specular: bool,
        from_glass: bool,
        pdf: f64,
        (other, glass): (f64, f64),
    ) -> Event {
        let event_pdf = if straight && !is_specular {
            if from_glass {
                glass
            } else {
                other
            }
        } else {
            pdf
        };
        Event::of(is_specular, from_glass, event_pdf)
    }
}

/// The header's state: Exact's arrival, and the share of exact transport through glass the
/// chain the path is in keeps.
#[derive(Clone, Copy, Debug, PartialEq)]
pub(super) struct SunPath {
    arrival: Arrival,
    chain: f64,
    in_chain: bool,
    /// Whether the last event was drawn from a glass lobe: only then is the ray's escape the
    /// end of exact transport through glass that `chain` scales.
    last_glass: bool,
}

impl SunPath {
    /// The camera ray: a delta receiver.
    pub(super) fn camera() -> SunPath {
        SunPath {
            arrival: Arrival::Delta,
            chain: 1.0,
            in_chain: false,
            last_glass: false,
        }
    }

    /// The record from its parts: the arrival's density (`None` for a delta), `chain`, `in_chain`
    /// and `last_glass`.
    pub(super) fn from_parts(
        arrival: Option<f64>,
        chain: f64,
        in_chain: bool,
        last_glass: bool,
    ) -> SunPath {
        SunPath {
            arrival: arrival.map_or(Arrival::Delta, |pdf| Arrival::Sampled { pdf }),
            chain,
            in_chain,
            last_glass,
        }
    }

    /// The record's parts, as [`SunPath::from_parts`] takes them.
    pub(super) fn parts(self) -> (Option<f64>, f64, bool, bool) {
        let arrival = match self.arrival {
            Arrival::Sampled { pdf } => Some(pdf),
            Arrival::Delta => None,
        };
        (arrival, self.chain, self.in_chain, self.last_glass)
    }

    /// The share of exact transport through glass the chain keeps (the nearest receiver's times
    /// the chain's before it), which a glass lobe's shadow ray is scaled by.
    pub(super) fn chain(self) -> f64 {
        self.chain
    }

    /// The share of exact transport through glass the chain this path is in keeps: `chain` in a
    /// chain, 1 outside any. A receiver's light through glass is scaled by it.
    pub(super) fn carried(self) -> f64 {
        if self.in_chain {
            self.chain
        } else {
            1.0
        }
    }

    /// The state after `event`; `straight` is whether the render mixes the straight line in for
    /// this scene's glass, `sun_pdf` the sun's density (0 with no sun), and `in_glass` whether
    /// the ray leaving the event travels inside glass.
    pub(super) fn after(
        self,
        event: Event,
        straight: bool,
        sun_pdf: f64,
        in_glass: bool,
    ) -> SunPath {
        if !straight {
            let arrival = match event {
                Event::Scatter { pdf } | Event::Glass { pdf: Some(pdf) } => {
                    Arrival::Sampled { pdf }
                }
                Event::Glass { pdf: None } | Event::Mirror => Arrival::Delta,
            };
            return SunPath {
                arrival,
                last_glass: matches!(event, Event::Glass { .. }),
                ..self
            };
        }
        // A receiver in a chain passes on its own share times the chain's; one outside any
        // starts afresh.
        let carried = self.carried();
        match event {
            Event::Scatter { pdf } => SunPath {
                arrival: Arrival::Sampled { pdf },
                chain: carried * power_heuristic(pdf, sun_pdf),
                in_chain: in_glass,
                last_glass: false,
            },
            Event::Mirror => SunPath {
                arrival: Arrival::Delta,
                chain: carried,
                in_chain: in_glass,
                last_glass: false,
            },
            Event::Glass { pdf } => SunPath {
                arrival: pdf.map_or(Arrival::Delta, |pdf| Arrival::Sampled { pdf }),
                chain: self.chain,
                in_chain: true,
                last_glass: true,
            },
        }
    }

    /// The weight of the sun's disc for a ray in this state that leaves the scene into it, the
    /// sun's density being `sun_pdf`.
    pub(super) fn sun_weight(self, strategy: LightStrategy, sun_pdf: f64) -> f64 {
        let weight = match self.arrival {
            Arrival::Delta => 1.0,
            Arrival::Sampled { pdf } => bounce_weight(strategy, pdf, sun_pdf),
        };
        if self.last_glass {
            weight * self.chain
        } else {
            weight
        }
    }
}

/// The bounce's share of a sun direction it found with density `pdf`, against the shadow ray's
/// `sun_pdf`, under `strategy`.
pub(super) fn bounce_weight(strategy: LightStrategy, pdf: f64, sun_pdf: f64) -> f64 {
    match strategy {
        LightStrategy::Combined => power_heuristic(pdf, sun_pdf),
        LightStrategy::SurfaceSamplingOnly => 1.0,
        LightStrategy::NextEventOnly => 0.0,
    }
}

/// The shadow ray's share of a sun direction of density `sun_pdf`, against a lobe that would find
/// it with density `pdf`, under `strategy`: what the bounce's share leaves.
pub(super) fn shadow_weight(strategy: LightStrategy, sun_pdf: f64, pdf: f64) -> f64 {
    match strategy {
        LightStrategy::Combined => power_heuristic(sun_pdf, pdf),
        LightStrategy::SurfaceSamplingOnly => 0.0,
        LightStrategy::NextEventOnly => 1.0,
    }
}

/// The power heuristic of `pdf` against `other`: `pdf^2 / (pdf^2 + other^2)`, from their ratio
/// so that large densities can't overflow the squares into an infinite division. In [0, 1] for
/// any input the scattering model and a `Sun` can produce; zero against zero weighs nothing.
pub(super) fn power_heuristic(pdf: f64, other: f64) -> f64 {
    if pdf > 0.0 {
        let ratio = other / pdf;
        1.0 / (1.0 + ratio * ratio)
    } else {
        0.0
    }
}
