//! [`RenderScene`], what a render reads: the placed parts, one [`ScatteringSurface`] per part and
//! the lights. Plain, immutable data, so it can be shared between threads. An empty scene is a
//! valid scene: every ray escapes to the lights.
//!
//! **Color.** The surface's base color is multiplied by the hit face's color. A mesh's face colors
//! are 8-bit display colors, sRGB encoded as the app stores them, so they are decoded to linear
//! first. The uncolored face (white) leaves the surface as it is, and that case skips the rebuild.

use solidmesh_math::Color;

use super::display::srgb_to_linear;
use super::lights::LightSetup;
use crate::ray_tracer::{PlacedParts, RayTracerError, ScatteringSurface};

/// A scene ready to render.
pub struct RenderScene {
    parts: PlacedParts,
    surfaces: Vec<ScatteringSurface>,
    lights: LightSetup,
    /// Every 8-bit channel value, decoded, so a hit doesn't take a power per color.
    srgb_to_linear: [f64; 256],
}

/// The decoded value of each 8-bit channel value.
fn decoded_table() -> [f64; 256] {
    // `code` is below 256, so the cast keeps it exactly.
    std::array::from_fn(|code| srgb_to_linear(code as u8))
}

impl RenderScene {
    /// `parts`, each shaded by the surface at its index, under `lights`. A surface count that
    /// differs from the part count is [`RayTracerError::InvalidArgument`].
    pub fn new(
        parts: PlacedParts,
        surfaces: Vec<ScatteringSurface>,
        lights: LightSetup,
    ) -> Result<Self, RayTracerError> {
        if surfaces.len() != parts.parts().len() {
            return Err(RayTracerError::InvalidArgument(
                "a render scene needs exactly one surface for each part",
            ));
        }
        Ok(RenderScene {
            parts,
            surfaces,
            lights,
            srgb_to_linear: decoded_table(),
        })
    }

    /// A scene with no parts: every ray goes straight to the `lights`.
    pub fn empty(lights: LightSetup) -> Self {
        RenderScene {
            parts: PlacedParts::new(Vec::new()),
            surfaces: Vec::new(),
            lights,
            srgb_to_linear: decoded_table(),
        }
    }

    /// The parts.
    pub fn parts(&self) -> &PlacedParts {
        &self.parts
    }

    /// The lights.
    pub fn lights(&self) -> &LightSetup {
        &self.lights
    }

    /// The surface of part `part_index`, its base color multiplied by `face_color`.
    pub(super) fn surface_at(
        &self,
        part_index: usize,
        face_color: Color,
    ) -> Option<ScatteringSurface> {
        let surface = *self.surfaces.get(part_index)?;
        if face_color.red == 255 && face_color.green == 255 && face_color.blue == 255 {
            return Some(surface);
        }
        let base = surface.base_color();
        let tinted = [
            base[0] * self.srgb_to_linear[usize::from(face_color.red)],
            base[1] * self.srgb_to_linear[usize::from(face_color.green)],
            base[2] * self.srgb_to_linear[usize::from(face_color.blue)],
        ];
        Some(ScatteringSurface::new(
            tinted,
            surface.metallic(),
            surface.roughness(),
            surface.transmission(),
            surface.ior(),
        ))
    }
}
