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
//!
//! **Glass on glass.** Building the scene finds, once, where faces of glass parts lie on each
//! other (`glass_contacts`), so the path loop knows at a hit whether a glass face meets air or
//! another glass. A scene where no glass face lies on another keeps an empty table.
//!
//! **Glass resting on an opaque part.** The same search between glass parts and the other parts
//! finds where glass rests on an opaque surface (`glass_rest` says what a path does there). A
//! scene with no glass, or no glass resting on anything, keeps an empty table.
//!
//! **A see-through part resting on an opaque part.** Setting the part opacities finds, by the
//! same search, where a part less than fully there rests on a fully there part that is not glass
//! (`see_through_rest` says what a line passing it does there). A scene with every part fully
//! there, or no see-through part resting on anything, keeps an empty table.
//!
//! **Coverage ends where its line does.** [`RenderScene::coverage_along`] keeps the line's far
//! limit on every stretch past a surface, so a line a caller ends at its floor's plane counts
//! nothing below it, wherever a part's face lies on that plane. A stretch past a surface is traced
//! to its next face wherever that lies, because the line may be inside a part there whose face it
//! entered by was skipped: a see-through part's base lying on another part's top is passed with
//! the other part's top, the next stretch starting just past both.
//!
//! **Which parts the end lies inside.** Once a stretch's next face lies past the end (by more than
//! that face's surface offset: a face closer than that lies at the end, since the walk's steps
//! cannot tell such faces apart), or a stretch starts past it, the walk goes on along the line to
//! its last face and counts, for each part it had not passed before the end, the faces it crosses.
//! A ray from a point inside a closed solid crosses its surface an odd number of times and one from
//! outside an even number, whatever the winding, nesting or overlap, so the end lies inside exactly
//! the parts crossed an odd number of times: each then counts as a part met before the end does, a
//! fully there one covering the line and a see-through one leaving what it does not cover, once.
//! So a see-through part resting on another part counts that part as one lifted off it does, a
//! line reaching the floor inside a part sunk through it counts that part, and a part the line
//! meets only past its end counts only if the end lies inside it: a part wholly below the floor
//! counts nothing, and neither does a part straddling the floor that the line would enter through
//! a side below it. The coverage is deterministic: no presence is drawn, so the order the parts
//! are counted in (the order the walk leaves them listed) is the one thing the routes share.
//! Only a scene with see-through parts walks past the end; in one without them the first face a
//! line meets is fully there and ends it.
//!
//! The limits: an open mesh has no inside, and the line crossing it past the end flips its count
//! as a closed one's face would; faces of two parts lying on each other past the end are passed as
//! one, like any faces closer than a surface offset, so the part whose face was skipped reads the
//! wrong count, and the CPU and the WGSL route may skip different faces there, so the two routes
//! can disagree on such lines (two see-through boxes sharing two planes, sunk through the floor,
//! under a resting see-through box: 23 and 35 of 2,304 lines differ between the CPU and the `f32`
//! mirror, neither above 0.3.0's coverage on any line); a face lying in the floor's
//! plane met on the line's first stretch is decided by each route's own rounding, since the first
//! stretch is traced to the end with no margin (an opaque box with its top in the floor's plane,
//! seen directly, covers 61 of 2,304 lines on the CPU and 833 in the `f32` mirror), while one met
//! past a see-through part resting on the floor covers, the end's margin applying there; and a
//! walk that reaches `MAX_CROSSINGS_PER_LINE` crossings (every face past the end counts toward it,
//! opaque ones too) ends there covered, as a line reaching the cap before the end does.

use solidmesh_math::{Color, Ray};

use super::display::srgb_to_linear;
use super::glass_contacts::{ContactSearchWork, GlassContacts};
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
    /// Where faces of glass parts lie on each other.
    contacts: GlassContacts,
    /// Where faces of glass parts lie on faces of opaque parts.
    rests: GlassContacts,
    /// Where faces of parts less than fully there lie on faces of fully there opaque parts.
    see_through_rests: GlassContacts,
    /// Every 8-bit channel value, decoded, so a hit doesn't take a power per color.
    srgb_to_linear: [f64; 256],
}

/// The decoded value of each 8-bit channel value.
fn decoded_table() -> [f64; 256] {
    // `code` is below 256, so the cast keeps it exactly.
    std::array::from_fn(|code| srgb_to_linear(code as u8))
}

/// Flips whether `part` is in `odd`, the parts a coverage line has crossed an odd number of times
/// past its end: a part leaving the list gives its place to the last one, as the shader's does.
fn flip(odd: &mut Vec<usize>, part: usize) {
    match odd.iter().position(|&other| other == part) {
        Some(index) => {
            odd.swap_remove(index);
        }
        None => odd.push(part),
    }
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
        let (contacts, rests) = GlassContacts::find_both(&parts, &surfaces);
        Ok(RenderScene {
            parts,
            surfaces,
            opacities: Vec::new(),
            lights,
            contacts,
            rests,
            see_through_rests: GlassContacts::default(),
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
            contacts: GlassContacts::default(),
            rests: GlassContacts::default(),
            see_through_rests: GlassContacts::default(),
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
        self.see_through_rests =
            GlassContacts::find_see_through_rests(&self.parts, &self.surfaces, &self.opacities);
        Ok(self)
    }

    /// Where faces of glass parts lie on each other.
    pub(crate) fn glass_contacts(&self) -> &GlassContacts {
        &self.contacts
    }

    /// Where faces of glass parts lie on faces of opaque parts.
    pub(crate) fn glass_rests(&self) -> &GlassContacts {
        &self.rests
    }

    /// Where faces of parts less than fully there lie on faces of fully there opaque parts.
    pub(crate) fn see_through_rests(&self) -> &GlassContacts {
        &self.see_through_rests
    }

    /// What finding where glass faces lie on each other took when this scene was built.
    pub fn glass_contact_search_work(&self) -> ContactSearchWork {
        self.contacts.work()
    }

    /// What finding where glass faces lie on faces of opaque parts took when this scene was
    /// built, after the glass-on-glass search: its triangles are those it carried into world space
    /// itself, a glass part the first search already carried counting there and not here.
    pub fn glass_rest_search_work(&self) -> ContactSearchWork {
        self.rests.work()
    }

    /// What finding where faces of parts less than fully there lie on faces of fully there opaque
    /// parts took when the part opacities were last set: all zero when no part is see-through.
    pub fn see_through_rest_search_work(&self) -> ContactSearchWork {
        self.see_through_rests.work()
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
    pub(crate) fn has_see_through_parts(&self) -> bool {
        !self.opacities.is_empty()
    }

    /// How much of `ray` the parts cover: 0 when it reaches nothing, 1 when it ends on an opaque
    /// part, and otherwise one minus the product of what each part along it leaves uncovered, a
    /// part counted once however many of its surfaces the ray crosses. No randomness and no
    /// bounce: what a caller alpha-compositing its render over a backdrop needs for a camera ray.
    /// The ray's far limit holds for the whole line, past every surface it goes through: a part
    /// counts when the line is inside it before its end (see the module header).
    pub fn coverage_along(&self, mut ray: Ray) -> Result<f64, RayTracerError> {
        let direction = ray.direction_normal;
        // Where the line ends, when it ends, and how far the current stretch has left to it.
        let end = ray
            .max_distance_to_consider
            .is_finite()
            .then(|| ray.origin + direction * ray.max_distance_to_consider);
        let mut left = ray.max_distance_to_consider;
        let see_through = self.has_see_through_parts();
        let mut line = StraightLine::default();
        let mut uncovered = 1.0;
        // Whether the walk has gone past the end, and the parts it had not passed before the end
        // that it has crossed an odd number of times since.
        let mut past = false;
        let mut odd: Vec<usize> = Vec::new();
        while let Some(hit) = self.parts.trace(&mut ray) {
            let part = hit.part.index();
            // Only a scene with see-through parts walks a stretch that can reach past the end,
            // and only there does a hit need its surface point before it is passed.
            let mut point = None;
            if !past && see_through && hit.info.distance_to_hit > left {
                let at = hit.surface_point()?;
                // A face within one surface offset past the end lies at the end: the walk's
                // steps cannot tell faces that close apart.
                past = hit.info.distance_to_hit > left + at.ray_offset.length();
                point = Some(at);
            }
            if past {
                if !line.count_pass() {
                    return Ok(1.0);
                }
                if !line.has_passed(part) {
                    flip(&mut odd, part);
                }
            } else {
                let opacity = self.part_opacity(part).unwrap_or(1.0);
                if opacity >= 1.0 {
                    return Ok(1.0);
                }
                match line.uncovered_by(part, opacity) {
                    // A line past the cap is treated as the end of the line: it is covered.
                    None => return Ok(1.0),
                    Some(part_left) => uncovered *= part_left,
                }
            }
            let point = match point {
                Some(point) => point,
                None => hit.surface_point()?,
            };
            ray = continued_ray(&point, direction);
            if let (Some(end), false) = (end, past) {
                left = (end - ray.origin).dot(direction);
                if left.is_nan() {
                    return Ok(1.0 - uncovered);
                }
                if left <= 0.0 {
                    if !see_through {
                        return Ok(1.0 - uncovered);
                    }
                    past = true;
                }
            }
        }
        // The line's end lies inside each part it crossed an odd number of times past the end.
        for &part in &odd {
            let opacity = self.part_opacity(part).unwrap_or(1.0);
            if opacity >= 1.0 {
                return Ok(1.0);
            }
            uncovered *= 1.0 - opacity;
        }
        Ok(1.0 - uncovered)
    }

    /// Part `part_index`'s own surface, before any face color.
    pub(crate) fn surface(&self, part_index: usize) -> Option<&ScatteringSurface> {
        self.surfaces.get(part_index)
    }

    /// Every 8-bit channel value, decoded to linear, the table a colored face reads.
    pub(crate) fn srgb_table(&self) -> &[f64; 256] {
        &self.srgb_to_linear
    }

    /// The lights.
    pub fn lights(&self) -> &LightSetup {
        &self.lights
    }

    /// The surface of part `part_index`, its base color multiplied by `face_color`.
    pub(crate) fn surface_at(
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
