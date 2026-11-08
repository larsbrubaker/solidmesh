//! The three operations a path tracer needs from a [`ScatteringSurface`]: `evaluate` (the BSDF
//! value), `sample` (a new direction, drawn from the visible microfacet normals or the cosine
//! hemisphere, with its weight) and `pdf` (the density of `sample`, lobe choice included, for
//! combining with light sampling). The maths behind them is described in `scattering`'s header;
//! the formulas are in `microfacet`, the precomputed albedo in `albedo`.

use std::f64::consts::PI;

use solidmesh_math::Vector3;

use super::albedo::{directional_albedo, mean_albedo};
use super::microfacet::{
    dielectric_fresnel, ggx_density, grazing_reflectance, reflect, refract, sample_visible_normal,
    schlick, smith_masking_shadowing, unit_direction, visible_normal_density, INDEX_MATCHED,
    MIN_ALPHA, MIN_COS,
};
use super::surface::ScatteringSurface;

/// One sampled scattering event. `direction` is the new direction away from the surface, in the
/// same local frame as `wo`.
///
/// For an event with `is_specular` false: `pdf` is the solid-angle density
/// [`ScatteringSurface::pdf`] gives this direction, and `weight` is `evaluate * |cos| / pdf`, the
/// factor a path's throughput is multiplied by.
///
/// For a perfectly specular event (a delta lobe): the delta lobe has no density to report and
/// contributes nothing to `evaluate` or `pdf` toward this direction (a smooth surface's other
/// lobes may still have a value there). Its `pdf` is the discrete probability that this event was
/// chosen, and `weight` is its throughput divided by that probability. A tracer must not weight
/// these against light sampling.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ScatteringSample {
    pub direction: Vector3,
    pub pdf: f64,
    pub weight: [f64; 3],
    pub is_specular: bool,
}

/// The cosine-weighted mean of Schlick's grazing term `(1 - c)^5`, in closed form, which is how
/// much of the way from `F0` to `F90` the Fresnel reflectance averages over the hemisphere.
const MEAN_SCHLICK_GRAZING: f64 = 1.0 / 21.0;

/// Below this share of energy lost to single scatter there is nothing to put back, and the
/// compensating lobe's ratio of two near-zero numbers would be noise.
const MIN_SINGLE_SCATTER_LOSS: f64 = 1e-5;

/// The largest value below 1, so a random number is always in `[0, 1)`.
const ALMOST_ONE: f64 = 1.0 - f64::EPSILON / 2.0;

/// How much of the surface is each kind of material; sums to 1.
struct Mix {
    metal: f64,
    plastic: f64,
    glass: f64,
}

/// The probability of each lobe being chosen; sums to 1, or to 0 when nothing can scatter. The
/// cosine lobe holds both the diffuse and the multiple-scatter energy, which share one sampling
/// distribution.
struct LobeChoice {
    specular: f64,
    cosine: f64,
    glass: f64,
}

/// The chosen lobe.
enum Lobe {
    Specular,
    Cosine,
    Glass,
}

/// Which way up glass is being worked with: mirrored in z so that `wo` is always above, with the
/// index of refraction on each side.
struct GlassSide {
    flipped: bool,
    wo: Vector3,
    eta_wo: f64,
    eta_wi: f64,
}

/// What the albedo table says about this surface as a whole, independent of direction.
struct ReflectiveEnergy {
    /// True when the single-scatter lobe drops enough energy to put any back.
    multiple_scatter: bool,
    /// The cosine-weighted mean of the white-Fresnel single-scatter albedo.
    mean_white: f64,
    /// How the multiple-scatter lobe is colored, per channel, the shares of metal and plastic
    /// included.
    multiple_scatter_fresnel: [f64; 3],
    /// The same for plastic alone, which is what the diffuse layer sits under.
    plastic_multiple_scatter: f64,
    /// The mean reflected energy of the plastic specular lobe, multiple scatter included.
    plastic_mean: f64,
    /// Plastic's Fresnel reflectances.
    plastic_f0: f64,
    plastic_f90: f64,
}

/// What the albedo table says about one direction.
struct DirectionalEnergy {
    /// Share of the reflectance reached from `F0`, then from `F90`.
    a: f64,
    b: f64,
    /// The single-scatter albedo with white Fresnel.
    white: f64,
    /// All the energy the plastic specular lobe reflects along this direction.
    plastic: f64,
}

/// `v` as the opposite side sees it.
fn mirror_z(v: Vector3) -> Vector3 {
    Vector3::new(v.x, v.y, -v.z)
}

/// Mean of the three channels.
fn mean(rgb: [f64; 3]) -> f64 {
    (rgb[0] + rgb[1] + rgb[2]) / 3.0
}

/// `value`, or zero when it is negative or not finite (a division that overflowed).
fn clean(value: f64) -> f64 {
    if value.is_finite() && value > 0.0 {
        value
    } else {
        0.0
    }
}

/// `value` as a random number in `[0, 1)`.
fn random(value: f64) -> f64 {
    if value.is_finite() {
        value.clamp(0.0, ALMOST_ONE)
    } else {
        0.0
    }
}

/// The color of the multiple-scatter lobe. The energy that bounces off more than one facet meets
/// the mean Fresnel reflectance once per bounce, a geometric series in the mean albedo; white
/// comes back as 1, a colored metal more saturated than itself.
fn multiple_scatter_fresnel(f0: f64, f90: f64, mean_white: f64) -> f64 {
    let mean_fresnel = f0 + (f90 - f0) * MEAN_SCHLICK_GRAZING;
    let denominator = 1.0 - mean_fresnel * (1.0 - mean_white);
    if denominator > 0.0 {
        mean_fresnel * mean_fresnel * mean_white / denominator
    } else {
        0.0
    }
}

impl ScatteringSurface {
    /// The BSDF toward the unit directions `wo` (camera side) and `wi` (light side), per channel.
    /// Zero for directions on or within `MIN_COS` of the horizon, for an opaque surface with
    /// either below it, and for delta lobes. See `scattering`'s header for the conventions.
    pub fn evaluate(&self, wo: Vector3, wi: Vector3) -> [f64; 3] {
        let (Some(wo), Some(wi)) = (unit_direction(wo), unit_direction(wi)) else {
            return [0.0; 3];
        };
        let mix = self.mix();
        let alpha = self.alpha();
        let mut value = [0.0; 3];
        if wo.z >= MIN_COS && wi.z >= MIN_COS {
            let energy = self.reflective_energy(&mix, alpha);
            let at_o = self.directional_energy(&energy, wo.z, alpha);
            let at_i = self.directional_energy(&energy, wi.z, alpha);
            if !self.is_smooth() {
                for (channel, single) in value
                    .iter_mut()
                    .zip(self.single_scatter(&mix, wo, wi, alpha))
                {
                    *channel += single;
                }
            }
            if energy.multiple_scatter {
                let scale =
                    (1.0 - at_o.white) * (1.0 - at_i.white) / (PI * (1.0 - energy.mean_white));
                for (channel, fresnel) in value.iter_mut().zip(energy.multiple_scatter_fresnel) {
                    *channel += fresnel * scale;
                }
            }
            let diffuse =
                mix.plastic * (1.0 - at_o.plastic).max(0.0) * (1.0 - at_i.plastic).max(0.0)
                    / (PI * (1.0 - energy.plastic_mean));
            for (channel, base) in value.iter_mut().zip(self.base_color) {
                *channel += base * diffuse;
            }
        }
        if self.is_rough_glass(&mix) && wo.z.abs() >= MIN_COS && wi.z.abs() >= MIN_COS {
            let side = self.glass_side(wo);
            let up_wi = if side.flipped { mirror_z(wi) } else { wi };
            let glass = if up_wi.z > 0.0 {
                reflected_glass(&side, up_wi, alpha, mix.glass).map(|f| [f; 3])
            } else {
                transmitted_glass(&side, up_wi, alpha, mix.glass, self.tint())
            };
            if let Some(glass) = glass {
                for (channel, contribution) in value.iter_mut().zip(glass) {
                    *channel += contribution;
                }
            }
        }
        value.map(clean)
    }

    /// The single-scatter specular term alone, unmixed from the other lobes: the part the albedo
    /// table is integrated from, exposed so tests can check it independently.
    #[doc(hidden)]
    pub fn evaluate_single_scatter(&self, wo: Vector3, wi: Vector3) -> [f64; 3] {
        let (Some(wo), Some(wi)) = (unit_direction(wo), unit_direction(wi)) else {
            return [0.0; 3];
        };
        if wo.z < MIN_COS || wi.z < MIN_COS || self.is_smooth() {
            return [0.0; 3];
        }
        self.single_scatter(&self.mix(), wo, wi, self.alpha())
            .map(clean)
    }

    /// The solid-angle density [`ScatteringSurface::sample`] draws `wi` from for this `wo`, with
    /// the choice of lobe included. Zero toward delta lobes.
    pub fn pdf(&self, wo: Vector3, wi: Vector3) -> f64 {
        let (Some(wo), Some(wi)) = (unit_direction(wo), unit_direction(wi)) else {
            return 0.0;
        };
        let mix = self.mix();
        let alpha = self.alpha();
        let choice = self.lobe_choice(&mix, wo.z);
        let mut density = 0.0;
        if wo.z >= MIN_COS && wi.z >= MIN_COS {
            if choice.specular > 0.0 && !self.is_smooth() {
                if let Some(h) = unit_direction(wo + wi) {
                    density +=
                        choice.specular * visible_normal_density(wo, h, alpha) / (4.0 * wo.dot(h));
                }
            }
            density += choice.cosine * wi.z / PI;
        }
        if choice.glass > 0.0
            && self.is_rough_glass(&mix)
            && wo.z.abs() >= MIN_COS
            && wi.z.abs() >= MIN_COS
        {
            let side = self.glass_side(wo);
            let up_wi = if side.flipped { mirror_z(wi) } else { wi };
            let glass = if up_wi.z > 0.0 {
                reflected_glass_density(&side, up_wi, alpha)
            } else {
                transmitted_glass_density(&side, up_wi, alpha)
            };
            density += choice.glass * glass.unwrap_or(0.0);
        }
        clean(density)
    }

    /// Draws the next direction of a path arriving along `wo`, from three random numbers in
    /// `[0, 1)` (out-of-range ones are clamped). `None` when nothing scatters: `wo` is on the
    /// horizon or has no direction, it is below an opaque surface, the surface absorbs everything,
    /// or a rough lobe drew a direction on the wrong side of the surface.
    pub fn sample(&self, wo: Vector3, u: [f64; 3]) -> Option<ScatteringSample> {
        let wo = unit_direction(wo)?;
        if wo.z.abs() < MIN_COS {
            return None;
        }
        let [pick, u1, u2] = u.map(random);
        let mix = self.mix();
        let choice = self.lobe_choice(&mix, wo.z);
        let lobe = if pick < choice.specular && choice.specular > 0.0 {
            Lobe::Specular
        } else if pick < choice.specular + choice.cosine && choice.cosine > 0.0 {
            Lobe::Cosine
        } else if choice.glass > 0.0 {
            Lobe::Glass
        } else if choice.cosine > 0.0 {
            Lobe::Cosine
        } else if choice.specular > 0.0 {
            Lobe::Specular
        } else {
            return None;
        };
        match lobe {
            Lobe::Specular => self.sample_specular(&mix, wo, choice.specular, u1, u2),
            Lobe::Cosine => self.sample_cosine(wo, u1, u2),
            Lobe::Glass => {
                let remapped = (pick - choice.specular - choice.cosine) / choice.glass;
                self.sample_glass(&mix, wo, choice.glass, random(remapped), u1, u2)
            }
        }
    }

    fn mix(&self) -> Mix {
        let dielectric = 1.0 - self.metallic;
        Mix {
            metal: self.metallic,
            plastic: dielectric * (1.0 - self.transmission),
            glass: dielectric * self.transmission,
        }
    }

    fn alpha(&self) -> f64 {
        (self.roughness * self.roughness).max(MIN_ALPHA)
    }

    fn is_smooth(&self) -> bool {
        self.roughness * self.roughness < MIN_ALPHA
    }

    /// True when the glass share has a microfacet lobe to evaluate.
    fn is_rough_glass(&self, mix: &Mix) -> bool {
        mix.glass > 0.0 && !self.is_smooth() && !self.is_index_matched()
    }

    fn is_index_matched(&self) -> bool {
        self.ior - 1.0 < INDEX_MATCHED
    }

    /// Reflectance at normal incidence of the dielectric part.
    fn dielectric_f0(&self) -> f64 {
        let ratio = (self.ior - 1.0) / (self.ior + 1.0);
        ratio * ratio
    }

    /// The tint of one crossing of a glass interface: two crossings give the base color.
    fn tint(&self) -> [f64; 3] {
        self.base_color.map(f64::sqrt)
    }

    /// The Fresnel reflectance of the reflective layer at incidence cosine `cos_theta`: the metal
    /// and plastic shares each with their own reflectances. Mixing `f0` and the grazing value
    /// separately keeps this an exact blend of the two materials. A metal always goes to white at
    /// grazing; only the dielectric part's grazing value falls away as its `F0` does.
    fn specular_reflectance(&self, mix: &Mix, cos_theta: f64) -> [f64; 3] {
        let plastic_f0 = self.dielectric_f0();
        let plastic_f90 = grazing_reflectance(plastic_f0);
        self.base_color.map(|base| {
            let f0 = mix.metal * base + mix.plastic * plastic_f0;
            let f90 = mix.metal + mix.plastic * plastic_f90;
            schlick(f0, f90, cos_theta)
        })
    }

    /// The single-scatter specular BSDF.
    fn single_scatter(&self, mix: &Mix, wo: Vector3, wi: Vector3, alpha: f64) -> [f64; 3] {
        let Some(h) = unit_direction(wo + wi) else {
            return [0.0; 3];
        };
        let term = ggx_density(h, alpha) * smith_masking_shadowing(wo.z, wi.z, alpha)
            / (4.0 * wo.z * wi.z);
        self.specular_reflectance(mix, wo.dot(h)).map(|f| f * term)
    }

    /// The direction-independent part of what the albedo table says about this surface.
    fn reflective_energy(&self, mix: &Mix, alpha: f64) -> ReflectiveEnergy {
        let [mean_a, mean_b] = mean_albedo(alpha);
        let mean_white = (mean_a + mean_b).min(1.0);
        let multiple_scatter = !self.is_smooth() && 1.0 - mean_white >= MIN_SINGLE_SCATTER_LOSS;
        let plastic_f0 = self.dielectric_f0();
        let plastic_f90 = grazing_reflectance(plastic_f0);
        let plastic_multiple_scatter = if multiple_scatter {
            multiple_scatter_fresnel(plastic_f0, plastic_f90, mean_white)
        } else {
            0.0
        };
        let multiple_scatter_color = self.base_color.map(|base| {
            if multiple_scatter {
                mix.metal * multiple_scatter_fresnel(base, 1.0, mean_white)
                    + mix.plastic * plastic_multiple_scatter
            } else {
                0.0
            }
        });
        let plastic_mean = plastic_f0 * mean_a
            + plastic_f90 * mean_b
            + plastic_multiple_scatter * (1.0 - mean_white);
        ReflectiveEnergy {
            multiple_scatter,
            mean_white,
            multiple_scatter_fresnel: multiple_scatter_color,
            plastic_multiple_scatter,
            plastic_mean,
            plastic_f0,
            plastic_f90,
        }
    }

    /// What the albedo table says about the direction at cosine `cos`.
    fn directional_energy(
        &self,
        energy: &ReflectiveEnergy,
        cos: f64,
        alpha: f64,
    ) -> DirectionalEnergy {
        let [a, b] = directional_albedo(cos, alpha);
        let white = (a + b).min(1.0);
        let plastic = energy.plastic_f0 * a
            + energy.plastic_f90 * b
            + energy.plastic_multiple_scatter * (1.0 - white);
        DirectionalEnergy {
            a,
            b,
            white,
            plastic,
        }
    }

    /// How likely each lobe is, weighted by the energy it returns along `wo`. Depends on `wo`
    /// alone so that `pdf` can reproduce it for any `wi`.
    fn lobe_choice(&self, mix: &Mix, cos_o: f64) -> LobeChoice {
        let (specular, cosine) = if cos_o >= MIN_COS {
            let alpha = self.alpha();
            let energy = self.reflective_energy(mix, alpha);
            let at_o = self.directional_energy(&energy, cos_o, alpha);
            let single = self.base_color.map(|base| {
                mix.metal * (base * at_o.a + at_o.b)
                    + mix.plastic * (energy.plastic_f0 * at_o.a + energy.plastic_f90 * at_o.b)
            });
            let multiple = mean(energy.multiple_scatter_fresnel) * (1.0 - at_o.white);
            let diffuse = mix.plastic * mean(self.base_color) * (1.0 - at_o.plastic).max(0.0);
            (mean(single), multiple + diffuse)
        } else {
            (0.0, 0.0)
        };
        let total = specular + cosine + mix.glass;
        if total > 0.0 {
            LobeChoice {
                specular: specular / total,
                cosine: cosine / total,
                glass: mix.glass / total,
            }
        } else {
            LobeChoice {
                specular: 0.0,
                cosine: 0.0,
                glass: 0.0,
            }
        }
    }

    fn glass_side(&self, wo: Vector3) -> GlassSide {
        if wo.z >= 0.0 {
            GlassSide {
                flipped: false,
                wo,
                eta_wo: 1.0,
                eta_wi: self.ior,
            }
        } else {
            GlassSide {
                flipped: true,
                wo: mirror_z(wo),
                eta_wo: self.ior,
                eta_wi: 1.0,
            }
        }
    }

    fn sample_specular(
        &self,
        mix: &Mix,
        wo: Vector3,
        probability: f64,
        u1: f64,
        u2: f64,
    ) -> Option<ScatteringSample> {
        if self.is_smooth() {
            let fresnel = self.specular_reflectance(mix, wo.z);
            return delta_event(mirror_about_z(wo), probability, fresnel, probability);
        }
        let h = sample_visible_normal(wo, self.alpha(), u1, u2)?;
        let wi = unit_direction(reflect(wo, h))?;
        if wi.z < MIN_COS {
            return None;
        }
        self.finish_sample(wo, wi)
    }

    fn sample_cosine(&self, wo: Vector3, u1: f64, u2: f64) -> Option<ScatteringSample> {
        let radius = u1.sqrt();
        let phi = 2.0 * PI * u2;
        let wi = Vector3::new(
            radius * libm::cos(phi),
            radius * libm::sin(phi),
            (1.0 - u1).max(0.0).sqrt(),
        );
        if wi.z < MIN_COS {
            return None;
        }
        self.finish_sample(wo, wi)
    }

    /// `pick` chooses between reflection and refraction, whose lobe share is already taken care
    /// of in `probability`. Glass with no index step is sampled as smooth: the half-vector
    /// formulas are singular there, so it gets the exact flat-interface Fresnel and Snell instead
    /// of a microfacet lobe.
    fn sample_glass(
        &self,
        mix: &Mix,
        wo: Vector3,
        probability: f64,
        pick: f64,
        u1: f64,
        u2: f64,
    ) -> Option<ScatteringSample> {
        let side = self.glass_side(wo);
        let unmirror = |w: Vector3| if side.flipped { mirror_z(w) } else { w };
        if self.is_smooth() || self.is_index_matched() {
            let fresnel = dielectric_fresnel(side.wo.z, side.eta_wo, side.eta_wi);
            if pick < fresnel {
                let throughput = [mix.glass; 3];
                let event = probability * fresnel;
                let direction = unmirror(mirror_about_z(side.wo));
                return delta_event(direction, event, throughput.map(|t| t * fresnel), event);
            }
            let eta_ratio = side.eta_wo / side.eta_wi;
            let up = refract(side.wo, Vector3::UNIT_Z, eta_ratio)?;
            let radiance = eta_ratio * eta_ratio;
            let event = probability * (1.0 - fresnel);
            let throughput = self
                .tint()
                .map(|tint| mix.glass * (1.0 - fresnel) * tint * radiance);
            return delta_event(unmirror(up), event, throughput, event);
        }
        let h = sample_visible_normal(side.wo, self.alpha(), u1, u2)?;
        let fresnel = dielectric_fresnel(side.wo.dot(h), side.eta_wo, side.eta_wi);
        let up_wi = if pick < fresnel {
            let reflected = unit_direction(reflect(side.wo, h))?;
            if reflected.z < MIN_COS {
                return None;
            }
            reflected
        } else {
            let refracted = unit_direction(refract(side.wo, h, side.eta_wo / side.eta_wi)?)?;
            if refracted.z > -MIN_COS {
                return None;
            }
            refracted
        };
        self.finish_sample(wo, unmirror(up_wi))
    }

    /// The sample for a direction a rough or cosine lobe drew, weighted by the whole BSDF over
    /// the density of the whole `sample`, so the weight matches `evaluate` and `pdf` exactly.
    fn finish_sample(&self, wo: Vector3, wi: Vector3) -> Option<ScatteringSample> {
        let density = self.pdf(wo, wi);
        if density <= 0.0 {
            return None;
        }
        let cosine = wi.z.abs();
        let weight = self.evaluate(wo, wi).map(|f| clean(f * cosine / density));
        Some(ScatteringSample {
            direction: wi,
            pdf: density,
            weight,
            is_specular: false,
        })
    }
}

/// `wo` mirrored about the surface normal.
fn mirror_about_z(wo: Vector3) -> Vector3 {
    Vector3::new(-wo.x, -wo.y, wo.z)
}

/// A delta event: `throughput` is what the lobe returns, `probability` how often it was chosen.
fn delta_event(
    direction: Vector3,
    probability: f64,
    throughput: [f64; 3],
    pdf: f64,
) -> Option<ScatteringSample> {
    let direction = unit_direction(direction)?;
    if probability <= 0.0 {
        return None;
    }
    Some(ScatteringSample {
        direction,
        pdf: clean(pdf),
        weight: throughput.map(|t| clean(t / probability)),
        is_specular: true,
    })
}

/// The BSDF of glass reflecting about the half vector of `wo` and `wi`, in the frame with `wo`
/// above; `None` where it is undefined.
fn reflected_glass(side: &GlassSide, up_wi: Vector3, alpha: f64, share: f64) -> Option<f64> {
    let h = unit_direction(side.wo + up_wi)?;
    let fresnel = dielectric_fresnel(side.wo.dot(h), side.eta_wo, side.eta_wi);
    let term = ggx_density(h, alpha) * smith_masking_shadowing(side.wo.z, up_wi.z, alpha)
        / (4.0 * side.wo.z * up_wi.z);
    Some(share * fresnel * term)
}

/// The sampling density of glass reflection given `wo`, as a density over `wi`.
fn reflected_glass_density(side: &GlassSide, up_wi: Vector3, alpha: f64) -> Option<f64> {
    let h = unit_direction(side.wo + up_wi)?;
    let cos_o = side.wo.dot(h);
    let fresnel = dielectric_fresnel(cos_o, side.eta_wo, side.eta_wi);
    Some(fresnel * visible_normal_density(side.wo, h, alpha) / (4.0 * cos_o))
}

/// The microfacet normal that refracts `wo` into `up_wi` (always pointing up), and the cosines
/// of each direction against it; `None` when no facet could connect them.
fn transmission_half(side: &GlassSide, up_wi: Vector3) -> Option<(Vector3, f64, f64)> {
    let unoriented = unit_direction(side.wo * side.eta_wo + up_wi * side.eta_wi)?;
    let h = if unoriented.z < 0.0 {
        -unoriented
    } else {
        unoriented
    };
    let cos_o = side.wo.dot(h);
    let cos_i = up_wi.dot(h);
    if cos_o <= 0.0 || cos_i >= 0.0 {
        return None;
    }
    Some((h, cos_o, cos_i))
}

/// The BSDF of glass refraction, tinted, with the radiance scaling for tracing from the camera.
fn transmitted_glass(
    side: &GlassSide,
    up_wi: Vector3,
    alpha: f64,
    share: f64,
    tint: [f64; 3],
) -> Option<[f64; 3]> {
    let (h, cos_o, cos_i) = transmission_half(side, up_wi)?;
    let denominator = side.eta_wo * cos_o + side.eta_wi * cos_i;
    if denominator == 0.0 {
        return None;
    }
    let fresnel = dielectric_fresnel(cos_o, side.eta_wo, side.eta_wi);
    // A refracted pair sits on opposite sides of the surface, where the exact height-correlated
    // form is a beta function of the two Smith terms. The reflection form used here stands in for
    // it: a deliberate approximation that stays at or below the masking of either direction alone.
    let term = ggx_density(h, alpha)
        * smith_masking_shadowing(side.wo.z, -up_wi.z, alpha)
        * (cos_o * cos_i).abs()
        / (side.wo.z * up_wi.z.abs())
        * (side.eta_wo * side.eta_wo)
        / (denominator * denominator);
    Some(tint.map(|channel| share * channel * (1.0 - fresnel) * term))
}

/// The sampling density of glass refraction given `wo`, as a density over `wi`.
fn transmitted_glass_density(side: &GlassSide, up_wi: Vector3, alpha: f64) -> Option<f64> {
    let (h, cos_o, cos_i) = transmission_half(side, up_wi)?;
    let denominator = side.eta_wo * cos_o + side.eta_wi * cos_i;
    if denominator == 0.0 {
        return None;
    }
    let fresnel = dielectric_fresnel(cos_o, side.eta_wo, side.eta_wi);
    let jacobian = side.eta_wi * side.eta_wi * cos_i.abs() / (denominator * denominator);
    Some((1.0 - fresnel) * visible_normal_density(side.wo, h, alpha) * jacobian)
}
