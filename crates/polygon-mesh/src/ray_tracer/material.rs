//! Surface materials and the primitives that carry them (agg-sharp
//! `RayTracer/Materials/MaterialAbstract.cs`, `Materials/SolidMaterial.cs`,
//! `Primitive/IPrimitive.cs`, and the color half of `Primitive/Shapes/BaseShape.cs`).
//!
//! C#'s abstract `MaterialAbstract` class becomes the [`MaterialAbstract`] trait over a shared
//! [`MaterialProperties`] block (gloss, transparency, reflection, refraction with their C#
//! defaults); shapes share materials the way C# shares references, as [`MaterialRef`]. A shape
//! built with C#'s `null` material holds `None`, and asking it for a color is an error where the
//! C# dereferences null. [`Primitive`] is C# `IPrimitive`: a traceable with a material and a
//! color at a hit; [`super::Traceable::as_primitive`] reaches it from a hit object.

use std::fmt::Debug;
use std::sync::Arc;

use agg_path::ColorF;

use super::{IntersectInfo, RayTracerError, Traceable};

/// A shared material (C# `MaterialAbstract` reference).
pub type MaterialRef = Arc<dyn MaterialAbstract>;

/// The lighting values every material has (C# `MaterialAbstract`'s fields).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MaterialProperties {
    pub gloss: f64,
    pub transparency: f64,
    pub reflection: f64,
    pub refraction: f64,
}

impl Default for MaterialProperties {
    /// The C# constructor's values.
    fn default() -> Self {
        MaterialProperties {
            gloss: 2.0,        // set a realistic value by default
            transparency: 0.0, // opaque by default
            reflection: 0.0,   // no reflection by default
            refraction: 0.50,  // default refraction for now
        }
    }
}

/// A surface's color source (C# `abstract class MaterialAbstract`).
pub trait MaterialAbstract: Debug + Send + Sync {
    /// The lighting values (C# `Gloss`, `Transparency`, `Reflection`, `Refraction`).
    fn properties(&self) -> &MaterialProperties;

    /// True when the color depends on the texture coordinates (C# `HasTexture`).
    fn has_texture(&self) -> bool;

    /// The color at texture coordinates `(u, v)` (C# `GetColor`).
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

/// Wraps any value into [-1, 1) rotationally: 1.7 → -0.3, -1.1 → 0.9, -2.3 → -0.3 (C#
/// `MaterialAbstract.WrapUp`, for textured materials). C# `%` on doubles is Rust `%`.
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

/// One color everywhere (C# `SolidMaterial`).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SolidMaterial {
    color: ColorF,
    properties: MaterialProperties,
}

impl SolidMaterial {
    /// C# `SolidMaterial(ColorF color, double reflection, double transparency, double gloss)`;
    /// refraction keeps its default.
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

    /// Mutable lighting values (the C# property setters).
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

/// The material `BaseShape`'s constructor gives a shape before the derived constructor sets
/// its own: opaque magenta.
pub fn base_shape_default_material() -> MaterialRef {
    Arc::new(SolidMaterial::new(
        ColorF::from_f64(1.0, 0.0, 1.0, 1.0),
        0.0,
        0.0,
        0.0,
    ))
}

/// A traceable with a surface (C# `IPrimitive`). The C# `Material` setter is an inherent
/// `set_material` on the shapes that keep one.
pub trait Primitive: Traceable {
    /// The material, `None` where the C# holds `null` (C# `Material`).
    fn material(&self) -> Option<&MaterialRef>;

    /// The texture coordinates of a hit on this shape (C# `BaseShape.GetUv`).
    fn get_uv(&self, info: &IntersectInfo<'_>) -> Result<(f64, f64), RayTracerError>;

    /// The color at a hit (C# `BaseShape.GetColor`): the texture at the hit's coordinates for a
    /// textured material, the material's (0, 0) color otherwise.
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
