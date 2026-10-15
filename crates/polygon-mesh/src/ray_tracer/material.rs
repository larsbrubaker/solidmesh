//! Surface materials and the primitives that carry them.
//!
//! Every material implements the [`MaterialAbstract`] trait over a shared
//! [`MaterialProperties`] block (gloss, transparency, reflection, refraction with their
//! defaults); shapes share materials as [`MaterialRef`]. A shape built without a material holds
//! `None`, and asking it for a color is an error. [`Primitive`] is a traceable with a material
//! and a color at a hit; [`super::Traceable::as_primitive`] reaches it from a hit object.

use std::fmt::Debug;
use std::sync::Arc;

use agg_path::ColorF;

use super::{IntersectInfo, RayTracerError, Traceable};

/// A shared material.
pub type MaterialRef = Arc<dyn MaterialAbstract>;

/// The lighting values every material has.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MaterialProperties {
    pub gloss: f64,
    pub transparency: f64,
    pub reflection: f64,
    pub refraction: f64,
}

impl Default for MaterialProperties {
    /// An opaque, non-reflective surface of gloss 2 and refraction 0.5.
    fn default() -> Self {
        MaterialProperties {
            gloss: 2.0,        // set a realistic value by default
            transparency: 0.0, // opaque by default
            reflection: 0.0,   // no reflection by default
            refraction: 0.50,  // default refraction for now
        }
    }
}

/// A surface's color source.
pub trait MaterialAbstract: Debug + Send + Sync {
    /// The lighting values (gloss, transparency, reflection, refraction).
    fn properties(&self) -> &MaterialProperties;

    /// True when the color depends on the texture coordinates.
    fn has_texture(&self) -> bool;

    /// The color at texture coordinates `(u, v)`.
    fn get_color(&self, u: f64, v: f64) -> ColorF;

    fn reflection(&self) -> f64 {
        self.properties().reflection
    }

    fn refraction(&self) -> f64 {
        self.properties().refraction
    }

    fn transparency(&self) -> f64 {
        self.properties().transparency
    }

    fn gloss(&self) -> f64 {
        self.properties().gloss
    }
}

/// Wraps any value into [-1, 1) rotationally: 1.7 → -0.3, -1.1 → 0.9, -2.3 → -0.3 (for
/// textured materials). `%` keeps the dividend's sign, hence the two corrections.
pub fn wrap_up(mut t: f64) -> f64 {
    t %= 2.0;
    if t < -1.0 {
        t += 2.0;
    }
    if t >= 1.0 {
        t -= 2.0;
    }
    t
}

/// One color everywhere.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SolidMaterial {
    color: ColorF,
    properties: MaterialProperties,
}

impl SolidMaterial {
    /// A material of `color` with the given reflection, transparency and gloss; refraction
    /// keeps its default.
    pub fn new(color: ColorF, reflection: f64, transparency: f64, gloss: f64) -> Self {
        SolidMaterial {
            color,
            properties: MaterialProperties {
                reflection,
                transparency,
                gloss,
                ..MaterialProperties::default()
            },
        }
    }

    /// Mutable lighting values.
    pub fn properties_mut(&mut self) -> &mut MaterialProperties {
        &mut self.properties
    }
}

impl MaterialAbstract for SolidMaterial {
    fn properties(&self) -> &MaterialProperties {
        &self.properties
    }

    fn has_texture(&self) -> bool {
        false
    }

    fn get_color(&self, _u: f64, _v: f64) -> ColorF {
        self.color
    }
}

/// The material a shape starts with before it is given its own: opaque magenta, so a shape
/// that never got one stands out.
pub fn base_shape_default_material() -> MaterialRef {
    Arc::new(SolidMaterial::new(
        ColorF::from_f64(1.0, 0.0, 1.0, 1.0),
        0.0,
        0.0,
        0.0,
    ))
}

/// A traceable with a surface. Shapes that keep a material set it with their own inherent
/// `set_material`.
pub trait Primitive: Traceable {
    /// The material, `None` for a shape built without one.
    fn material(&self) -> Option<&MaterialRef>;

    /// The texture coordinates of a hit on this shape.
    fn get_uv(&self, info: &IntersectInfo<'_>) -> Result<(f64, f64), RayTracerError>;

    /// The color at a hit: the texture at the hit's coordinates for a textured material, the
    /// material's (0, 0) color otherwise.
    fn get_color(&self, info: &IntersectInfo<'_>) -> Result<ColorF, RayTracerError> {
        let material = self.material().ok_or(RayTracerError::NoMaterial)?;
        if material.has_texture() {
            let (u, v) = self.get_uv(info)?;
            Ok(material.get_color(u, v))
        } else {
            Ok(material.get_color(0.0, 0.0))
        }
    }
}
