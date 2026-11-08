//! The Outline to Solid page: contours in mm, edited in the browser, turned into solids by
//! `processors::outline_to_mesh` - `extrude_contours` (square ends), `extrude_bevelled_contours`
//! (rounded or chamfered ends) and `revolve_contours` (a full or partial sweep about z).
//!
//! A bevel is the library's list of `(height, inset)` rings: here a quarter circle of the bevel's
//! radius cut into `steps` rings (one step is a chamfer), the insets measured from the original
//! profile as the library expects.

use std::f64::consts::FRAC_PI_2;

use solidmesh::clipper::JoinType;
use solidmesh::math::Vector2;
use solidmesh::mesh::processors::outline_to_mesh::{
    extrude_bevelled_contours, extrude_contours, revolve_contours,
};

use crate::Scene;

/// Contours from the browser's flat arrays: `coords` holds x, y pairs, `counts` how many points
/// each contour takes, in order. Points past the counts are ignored.
pub fn contours_from_flat(coords: &[f64], counts: &[u32]) -> Vec<Vec<Vector2>> {
    let mut points = coords
        .as_chunks::<2>()
        .0
        .iter()
        .map(|&[x, y]| Vector2::new(x, y));
    counts
        .iter()
        .map(|&n| points.by_ref().take(n as usize).collect())
        .collect()
}

/// The `(height, inset)` rings of a top bevel of `radius` on a prism `height` tall: a quarter
/// circle cut into `steps` straight pieces, rising from `height - radius` to the top.
pub fn quarter_round_bevel(height: f64, radius: f64, steps: u32) -> Vec<(f64, f64)> {
    let n = steps.max(1);
    let start = height - radius;
    (0..n)
        .map(|i| {
            let at = |k: u32| FRAC_PI_2 * f64::from(k) / f64::from(n);
            let ring_height = start + radius * at(i).sin();
            let inset = -(radius - radius * at(i + 1).cos());
            (ring_height, inset)
        })
        .collect()
}

/// The bottom bevel's rings: the top bevel's mirror image, ordered from the top of the bevel
/// downward as the library reads them.
fn bottom_bevel_rings(radius: f64, steps: u32) -> Vec<(f64, f64)> {
    quarter_round_bevel(radius, radius, steps)
        .into_iter()
        .map(|(h, inset)| (radius - h, inset))
        .collect()
}

/// The contours extruded `height` mm, with a top and bottom bevel of the given radius (0 for a
/// square end); `round_joins` rounds the bevel's corners where the profile turns.
pub fn extrude_scene(
    contours: &[Vec<Vector2>],
    height: f64,
    top_bevel: f64,
    bottom_bevel: f64,
    steps: u32,
    round_joins: bool,
) -> Result<Scene, String> {
    // Each bevel can take at most half the height, or the two would cross.
    let top = top_bevel.clamp(0.0, height / 2.0);
    let bottom = bottom_bevel.clamp(0.0, height / 2.0);
    let mesh = if top <= 0.0 && bottom <= 0.0 {
        extrude_contours(contours, height)
    } else {
        let join = if round_joins {
            JoinType::Round
        } else {
            JoinType::Miter
        };
        let top_rings = (top > 0.0).then(|| quarter_round_bevel(height, top, steps));
        let bottom_rings = (bottom > 0.0).then(|| bottom_bevel_rings(bottom, steps));
        extrude_bevelled_contours(
            contours,
            height,
            top_rings.as_deref(),
            join,
            bottom_rings.as_deref(),
            join,
        )
    }
    .map_err(|e| format!("The extrusion failed: {e}"))?;
    let kind = if top > 0.0 || bottom > 0.0 {
        "extrude_bevelled_contours"
    } else {
        "extrude_contours"
    };
    Ok(Scene::new(
        mesh,
        format!("{kind} on {} contour(s).", contours.len()),
    ))
}

/// The contours (x out from the axis, y along it) revolved about z from `start_deg` to `end_deg`
/// in `steps` strips.
pub fn revolve_scene(
    contours: &[Vec<Vector2>],
    steps: u32,
    start_deg: f64,
    end_deg: f64,
) -> Result<Scene, String> {
    let mesh = revolve_contours(
        contours,
        steps.max(1) as i32,
        start_deg.to_radians(),
        end_deg.to_radians(),
        true,
    )
    .map_err(|e| format!("The revolve failed: {e}"))?;
    Ok(Scene::new(
        mesh,
        format!(
            "revolve_contours, {:.0} to {:.0} degrees in {} steps.",
            start_deg, end_deg, steps
        ),
    ))
}
