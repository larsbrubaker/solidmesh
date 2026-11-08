//! [`ScatteringSurface`], the plain description of a surface for the scattering model: linear-RGB
//! base color, metallic, roughness, transmission and index of refraction, each clamped into its
//! range on construction, with named presets. Part of `scattering`; `model` shades with it.

/// A surface as a path tracer shades it. Fields are private so the ranges below always hold;
/// out-of-range input is clamped and a non-finite value becomes the range's default.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ScatteringSurface {
    pub(super) base_color: [f64; 3],
    pub(super) metallic: f64,
    pub(super) roughness: f64,
    pub(super) transmission: f64,
    pub(super) ior: f64,
}

/// The lowest allowed index of refraction: vacuum, as nothing is optically thinner.
const MIN_IOR: f64 = 1.0;
/// The highest allowed index of refraction, above any transparent material a user will meet
/// (diamond is 2.42).
const MAX_IOR: f64 = 4.0;
/// The index of refraction used when none is given: ordinary glass and plastic.
const DEFAULT_IOR: f64 = 1.5;

/// `value` clamped to `[low, high]`, or `fallback` when it is not finite.
fn clamp_or(value: f64, low: f64, high: f64, fallback: f64) -> f64 {
    if value.is_finite() {
        value.clamp(low, high)
    } else {
        fallback
    }
}

impl ScatteringSurface {
    /// A surface: `base_color` in linear RGB in [0, 1]; `metallic` 0 (dielectric) to 1 (metal);
    /// `roughness` 0 (polished) to 1, perceptual as a slider is, the GGX width being its square;
    /// `transmission` 0 (opaque) to 1 (glass), ignored by the metal share; `ior` the index of
    /// refraction, clamped to [1, 4]. Out-of-range values clamp; a non-finite value becomes 0
    /// (1 for roughness, the safe broad end, 1.5 for the index of refraction).
    pub fn new(
        base_color: [f64; 3],
        metallic: f64,
        roughness: f64,
        transmission: f64,
        ior: f64,
    ) -> Self {
        ScatteringSurface {
            base_color: base_color.map(|channel| clamp_or(channel, 0.0, 1.0, 0.0)),
            metallic: clamp_or(metallic, 0.0, 1.0, 0.0),
            roughness: clamp_or(roughness, 0.0, 1.0, 1.0),
            transmission: clamp_or(transmission, 0.0, 1.0, 0.0),
            ior: clamp_or(ior, MIN_IOR, MAX_IOR, DEFAULT_IOR),
        }
    }

    /// The base color, linear RGB.
    pub fn base_color(&self) -> [f64; 3] {
        self.base_color
    }

    /// 0 for a dielectric, 1 for a metal.
    pub fn metallic(&self) -> f64 {
        self.metallic
    }

    /// Perceptual roughness; the GGX width is its square.
    pub fn roughness(&self) -> f64 {
        self.roughness
    }

    /// 0 for opaque, 1 for glass.
    pub fn transmission(&self) -> f64 {
        self.transmission
    }

    /// The index of refraction.
    pub fn ior(&self) -> f64 {
        self.ior
    }

    /// Flat colored plastic: a broad, weak sheen.
    pub fn matte_plastic(base_color: [f64; 3]) -> Self {
        Self::new(base_color, 0.0, 0.6, 0.0, DEFAULT_IOR)
    }

    /// Shiny colored plastic: a tight highlight over the color.
    pub fn glossy_plastic(base_color: [f64; 3]) -> Self {
        Self::new(base_color, 0.0, 0.15, 0.0, DEFAULT_IOR)
    }

    /// Metal with a visibly blurred reflection.
    pub fn brushed_metal(base_color: [f64; 3]) -> Self {
        Self::new(base_color, 1.0, 0.35, 0.0, DEFAULT_IOR)
    }

    /// Metal with a sharp, mirror-like reflection.
    pub fn polished_metal(base_color: [f64; 3]) -> Self {
        Self::new(base_color, 1.0, 0.05, 0.0, DEFAULT_IOR)
    }

    /// Clear, perfectly smooth glass.
    pub fn glass() -> Self {
        Self::new([1.0, 1.0, 1.0], 0.0, 0.0, 1.0, DEFAULT_IOR)
    }

    /// Dark-sheened rubber: nearly all diffuse, a very broad highlight.
    pub fn rubber(base_color: [f64; 3]) -> Self {
        Self::new(base_color, 0.0, 0.85, 0.0, 1.52)
    }
}
