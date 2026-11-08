//! Which edge of a mixed corner is really the CUT where the bead rolled along a chorded wall. The
//! sagitta guard of the wall's chords, how far apart two skew lines stand, and the cut edge's walk
//! ([`try_find`], [`try_find_for_the_bead`], [`try_walk`]).

use solidmesh_math::{ieee_math, Vector3};

use super::surface_mixed_corner_read_off_the_edge_plans::rests_on_the_crease_of_the_face_shared_with;
use super::{SurfaceEdgePlan, SurfaceLine, SurfaceWallSection};
use crate::topology::EdgeConvexity;

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
#[allow(clippy::neg_cmp_op_on_partial_ord)] // `!(x > 0)` on purpose: it refuses a NaN as well.
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

        let turn = libm::acos(before.0.dot(here.0).clamp(-1.0, 1.0));
        let across = ieee_math::max_f64(before.1, here.1);

        if !(turn > 0.0) || !(across > 0.0) {
            continue;
        }

        let radius = across / (2.0 * libm::sin(turn / 2.0));

        sagitta = ieee_math::max_f64(sagitta, radius * (1.0 - libm::cos(turn / 2.0)));
    }

    sagitta
}

/// How far apart two skew lines stand - the length of their common perpendicular - or `None`
/// where they are parallel and have no single one.
#[allow(clippy::neg_cmp_op_on_partial_ord)] // `!(x > y)` on purpose: it refuses a NaN as well.
pub fn try_apart(first: SurfaceLine, second: SurfaceLine) -> Option<f64> {
    let between = second.point - first.point;
    let direction_dot = first.direction.dot(second.direction);
    let determinant = 1.0 - (direction_dot * direction_dot);

    if !(determinant.abs() > 1e-12) {
        return None;
    }

    let along_first = between.dot(first.direction);
    let along_second = between.dot(second.direction);
    let here = (along_first - (direction_dot * along_second)) / determinant;
    let there = ((direction_dot * along_first) - along_second) / determinant;

    Some(
        ((first.point + (first.direction * here)) - (second.point + (second.direction * there)))
            .length(),
    )
}

/// The cut edge of the corner between `planned[corner_index]` and the next, walking from the cut's
/// side away from the bead (the leaving edge where the cut arrives, else the arriving one). Returns
/// the cut's index and the edges swallowed before it.
pub fn try_find(
    planned: &[SurfaceEdgePlan],
    is_closed: bool,
    corner_index: i32,
    cut_is_arriving: bool,
    size: f64,
    near_enough: f64,
) -> Option<(i32, Vec<i32>)> {
    let count = planned.len() as i32;
    let leaving = (corner_index + 1) % count;
    let bead_index = if cut_is_arriving {
        leaving
    } else {
        corner_index
    };

    try_walk(
        planned,
        is_closed,
        if cut_is_arriving {
            corner_index
        } else {
            leaving
        },
        if cut_is_arriving { -1 } else { 1 },
        bead_index,
        &planned[bead_index as usize],
        size,
        near_enough,
    )
}

/// The same walk for a `bead` of another chain meeting this run at its start (or end).
pub fn try_find_for_the_bead(
    planned: &[SurfaceEdgePlan],
    is_closed: bool,
    corner_is_at_the_start: bool,
    bead: &SurfaceEdgePlan,
    size: f64,
    near_enough: f64,
) -> Option<(i32, Vec<i32>)> {
    try_walk(
        planned,
        is_closed,
        if corner_is_at_the_start {
            0
        } else {
            planned.len() as i32 - 1
        },
        if corner_is_at_the_start { 1 } else { -1 },
        -1,
        bead,
        size,
        near_enough,
    )
}

/// Walks the run from `at` by `step` until an edge's centre line stands 2r from the bead's (the
/// cut), or - for a bead resting on a crease - past the local maximum of that distance (the chord
/// before is the cut, within the wall chords' sagitta of 2r). `None` where the walk meets a concave
/// or centre-less edge, clears the bead without touching, runs off an open end or comes back to the
/// bead. Returns the cut's index and the edges swallowed before it.
#[allow(clippy::too_many_arguments)]
pub fn try_walk(
    planned: &[SurfaceEdgePlan],
    is_closed: bool,
    mut at: i32,
    step: i32,
    bead_index: i32,
    bead: &SurfaceEdgePlan,
    size: f64,
    near_enough: f64,
) -> Option<(i32, Vec<i32>)> {
    let count = planned.len() as i32;

    if !bead.has_center_line {
        return None;
    }

    let mut passed = Vec::new();

    // A bead RESTING ON A CREASE is tangent to neither facet's plane; qualified against the face
    // the walked chords share.
    let rests_on_the_crease =
        rests_on_the_crease_of_the_face_shared_with(bead, &planned[at as usize]);

    // How far short of 2r a crease landing is allowed to stand.
    let allowed_deficit = if rests_on_the_crease {
        sagitta_of_the_walls_chords(bead.wall_section.as_deref()) + near_enough
    } else {
        0.0
    };

    let mut nearest = -1;
    let mut nearest_apart = f64::NEG_INFINITY;

    for _ in 0..count {
        let plan = &planned[at as usize];

        if plan.convexity() != EdgeConvexity::Convex || !plan.has_center_line {
            return None;
        }

        let apart = try_apart(plan.center_line, bead.center_line)?;

        if (apart - (2.0 * size)).abs() <= near_enough {
            return Some((at, passed));
        }

        // Down from the local maximum by more than the part's own ruler.
        if rests_on_the_crease && nearest >= 0 && apart < nearest_apart - near_enough {
            if (2.0 * size) - nearest_apart > allowed_deficit {
                // No tangency here to hand over on.
                return None;
            }

            // Past the local maximum: the chord before this one is the cut.
            passed.pop();
            return Some((nearest, passed));
        }

        nearest = at;
        nearest_apart = apart;

        if apart > 2.0 * size {
            // The ball has already cleared this edge's cylinder without touching one.
            return None;
        }

        passed.push(at);

        let mut ahead = at + step;
        if ahead < 0 || ahead >= count {
            if !is_closed {
                return None;
            }

            ahead = (ahead + count) % count;
        }

        if ahead == bead_index {
            // All the way round to the bead itself.
            return None;
        }

        at = ahead;
    }

    None
}
