//! [`RenderScene`], what a render reads: the placed parts, one [`ScatteringSurface`] per part and
//! the lights. Plain, immutable data, so it can be shared between threads. An empty scene is a
//! valid scene: every ray escapes to the lights.
//!
//! **Color.** The surface's base color is multiplied by the hit face's color. A mesh's face colors
//! are 8-bit display colors, sRGB encoded as the app stores them, so they are decoded to linear
//! first. The uncolored face (white) leaves the surface as it is, and that case skips the rebuild.
//!
//! **Opacity.** Each part has an opacity in [0, 1], what fraction of it is present, separate from
//! its surface: a half-there part is half coverage of a surface that is all surface, not a tint.
//! The opacities are 1 unless a caller says otherwise; `see_through` says what a ray does with
//! them, and a scene with every part at opacity 1 renders exactly as if none of this existed.

use solidmesh_math::{Color, Ray};

use super::display::srgb_to_linear;
use super::lights::LightSetup;
use super::see_through::{continued_ray, StraightLine};
use crate::ray_tracer::{PlacedParts, RayTracerError, ScatteringSurface};

/// A scene ready to render.
pub struct RenderScene {
    parts: PlacedParts,
    surfaces: Vec<ScatteringSurface>,
    /// What fraction of each part is present, empty when every part is fully present.
    opacities: Vec<f64>,
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
            opacities: Vec::new(),
            lights,
            srgb_to_linear: decoded_table(),
        })
    }

    /// A scene with no parts: every ray goes straight to the `lights`.
    pub fn empty(lights: LightSetup) -> Self {
        RenderScene {
            parts: PlacedParts::new(Vec::new()),
            surfaces: Vec::new(),
            opacities: Vec::new(),
            lights,
            srgb_to_linear: decoded_table(),
        }
    }

    /// This scene with each part as present as `opacities` says at its index: 1 is a fully
    /// there part and 0 one that no ray meets. A count that differs from the part count, or a
    /// value that isn't in [0, 1], is [`RayTracerError::InvalidArgument`].
    pub fn with_part_opacities(mut self, opacities: Vec<f64>) -> Result<Self, RayTracerError> {
        if opacities.len() != self.parts.parts().len() {
            return Err(RayTracerError::InvalidArgument(
                "a render scene needs exactly one opacity for each part",
            ));
        }
        // Written so a NaN is refused as well.
        if !opacities
            .iter()
            .all(|opacity| (0.0..=1.0).contains(opacity))
        {
            return Err(RayTracerError::InvalidArgument(
                "a part's opacity must be between 0 and 1",
            ));
        }
        self.opacities = if opacities.iter().all(|&opacity| opacity >= 1.0) {
            Vec::new()
        } else {
            opacities
        };
        Ok(self)
    }

    /// The parts.
    pub fn parts(&self) -> &PlacedParts {
        &self.parts
    }

    /// What fraction of part `part_index` is present, or `None` when there is no such part.
    pub fn part_opacity(&self, part_index: usize) -> Option<f64> {
        if part_index >= self.parts.parts().len() {
            None
        } else {
            Some(self.opacities.get(part_index).copied().unwrap_or(1.0))
        }
    }

    /// Whether any part is less than fully present.
    pub(super) fn has_see_through_parts(&self) -> bool {
        !self.opacities.is_empty()
    }

    /// How much of `ray` the parts cover: 0 when it reaches nothing, 1 when it ends on an opaque
    /// part, and otherwise one minus the product of what each part along it leaves uncovered, a
    /// part counted once however many of its surfaces the ray crosses. No randomness and no
    /// bounce: what a caller alpha-compositing its render over a backdrop needs for a camera ray.
    pub fn coverage_along(&self, mut ray: Ray) -> Result<f64, RayTracerError> {
        let direction = ray.direction_normal;
        let mut line = StraightLine::default();
        let mut uncovered = 1.0;
        loop {
            let Some(hit) = self.parts.trace(&mut ray) else {
                return Ok(1.0 - uncovered);
            };
            let opacity = self.part_opacity(hit.part.index()).unwrap_or(1.0);
            if opacity >= 1.0 {
                return Ok(1.0);
            }
            match line.uncovered_by(hit.part.index(), opacity) {
                // A line past the cap is treated as the end of the line: it is covered.
                None => return Ok(1.0),
                Some(left) => uncovered *= left,
            }
            ray = continued_ray(&hit.surface_point()?, direction);
        }
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
