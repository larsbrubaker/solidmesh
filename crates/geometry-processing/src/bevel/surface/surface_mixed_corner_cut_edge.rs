//! Which edge of a mixed corner is really the CUT where the bead rolled along a chorded wall
//! (`Bevel/Surface/SurfaceMixedCornerCutEdge.cs`, a static class: free functions here). The
//! sagitta guard of the wall's chords lands here first; the cut edge's walk lands with a later
//! package.

use vector_math::Vector3;

use super::SurfaceWallSection;

/// How square two of a section's facets have to stand to each other to be two triangles of ONE
/// chord of the wall rather than two chords: a millionth off parallel, which is float storage and
/// nothing else, because a wall's chords turn by whole degrees.
pub const ONE_CHORDS_FACETS: f64 = 1.0 - 1e-6;

/// The widest sagitta of the wall's chords along `section`, or zero where there is no section
/// or no turn in it.
///
/// Read off the section the roll itself walked rather than off a circle fitted to the wall,
/// because the facets are what the bead really rests on: the turn between two chords' normals and
/// the longer of the two chords give the circle they chord - `R = c / 2 sin(turn / 2)` - and the
/// sagitta is `R (1 - cos(turn / 2))`. The widest such sagitta, because this is a guard on a
/// reading and not the reading itself.
///
/// By CHORD and not by facet. A chorded wall is triangulated, so one chord of it is two facets of
/// the same normal and one step of the section is half a chord; measured facet by facet the circle
/// would come out half the size it is.
#[allow(clippy::neg_cmp_op_on_partial_ord)] // C#'s `!(x > 0)`, which refuses a NaN as well.
pub fn sagitta_of_the_walls_chords(section: Option<&SurfaceWallSection>) -> f64 {
    let mut sagitta = 0.0;

    let Some(section) = section else {
        return sagitta;
    };

    let mut chords: Vec<(Vector3, f64)> = Vec::new();

    for step in &section.facets {
        let normal = step.normal.get_normal();

        if let Some(last) = chords.last_mut() {
            if last.0.dot(normal) > ONE_CHORDS_FACETS {
                last.1 += step.crossed();
                continue;
            }
        }

        chords.push((normal, step.crossed()));
    }

    for chord in 1..chords.len() {
        let before = chords[chord - 1];
        let here = chords[chord];

        let turn = before.0.dot(here.0).clamp(-1.0, 1.0).acos();
        let across = f64::max(before.1, here.1);

        if !(turn > 0.0) || !(across > 0.0) {
            continue;
        }

        let radius = across / (2.0 * (turn / 2.0).sin());

        sagitta = f64::max(sagitta, radius * (1.0 - (turn / 2.0).cos()));
    }

    sagitta
}
