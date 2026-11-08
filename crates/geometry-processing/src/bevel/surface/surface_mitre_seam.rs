//! Where two consecutive edges' bands meet on the face they share - the mitre SEAM's point: the
//! crossing of the two tangent lines on that face, read on every face the two share rather than on
//! the left alone, so the walk's own direction never tells two readings apart.

use std::sync::Arc;

use solidmesh_math::Vector3;

use super::{SurfaceEdgePlan, SurfaceLine};
use crate::topology::FaceGroup;

/// How near parallel two tangent lines have to be to be read as never crossing, as the distance
/// of their cosine from one.
const PARALLEL_WITHIN: f64 = 1e-9;

/// Where the two edges' tangent lines cross on a face they share - of all the shared faces, the
/// pair that came closest to meeting - or `None` where no pair on a shared face crosses within
/// `near_enough`.
pub fn try_the_point(
    arriving: &SurfaceEdgePlan,
    leaving: &SurfaceEdgePlan,
    near_enough: f64,
) -> Option<Vector3> {
    let mut closest = f64::INFINITY;
    let mut through = None;

    for (line, on_the_left) in the_tangent_lines_on_the_shared_faces(arriving, leaving) {
        let Some(theirs) = the_theirs_on_the_same_face(arriving, leaving, on_the_left) else {
            continue;
        };
        if they_are_parallel(line, theirs) {
            continue;
        }
        let Some((crossing, missed_by)) = line.try_cross_missed_by(&theirs, near_enough) else {
            continue;
        };

        if missed_by < closest {
            closest = missed_by;
            through = Some(crossing);
        }
    }

    through
}

/// Whether two lines run parallel, to `PARALLEL_WITHIN`.
pub fn they_are_parallel(mine: SurfaceLine, theirs: SurfaceLine) -> bool {
    (mine.direction.dot(theirs.direction).abs() - 1.0).abs() < PARALLEL_WITHIN
}

/// `mine`'s tangent lines on the faces it shares with `theirs`, each with whether it is `mine`'s
/// left one - left first.
pub fn the_tangent_lines_on_the_shared_faces(
    mine: &SurfaceEdgePlan,
    theirs: &SurfaceEdgePlan,
) -> Vec<(SurfaceLine, bool)> {
    let mut lines = Vec::with_capacity(2);

    if is_one_of(&mine.left_region, theirs) {
        lines.push((mine.left_tangent_line, true));
    }

    if is_one_of(&mine.right_region, theirs) {
        lines.push((mine.right_tangent_line, false));
    }

    lines
}

/// `theirs`' tangent line on the face `mine`'s left (or right) one lies on, or `None` where
/// `theirs` does not stand on that face. Faces are the same face by identity (`Arc::ptr_eq`).
pub fn the_theirs_on_the_same_face(
    mine: &SurfaceEdgePlan,
    theirs: &SurfaceEdgePlan,
    mine_is_on_the_left: bool,
) -> Option<SurfaceLine> {
    let region = if mine_is_on_the_left {
        &mine.left_region
    } else {
        &mine.right_region
    };

    if Arc::ptr_eq(region, &theirs.left_region) {
        return Some(theirs.left_tangent_line);
    }

    if Arc::ptr_eq(region, &theirs.right_region) {
        return Some(theirs.right_tangent_line);
    }

    None
}

/// Whether the two edges' tangent lines are ONE line on some face they share: parallel, and
/// within `near_enough` of each other. Asked of EVERY face the two share, the same set
/// [`try_the_point`] walks: one crease's two collinear pieces are one line on the face that
/// carries the crease whether the walk calls that face its left or its right.
pub fn the_lines_are_one(
    arriving: &SurfaceEdgePlan,
    leaving: &SurfaceEdgePlan,
    near_enough: f64,
) -> bool {
    the_tangent_lines_on_the_shared_faces(arriving, leaving)
        .into_iter()
        .any(|(line, on_the_left)| {
            the_theirs_on_the_same_face(arriving, leaving, on_the_left).is_some_and(|theirs| {
                they_are_parallel(line, theirs) && line.distance_to(theirs.point) <= near_enough
            })
        })
}

/// [`try_the_point`], or `corner` where there is none.
pub fn the_point_or(
    arriving: &SurfaceEdgePlan,
    leaving: &SurfaceEdgePlan,
    near_enough: f64,
    corner: Vector3,
) -> Vector3 {
    try_the_point(arriving, leaving, near_enough).unwrap_or(corner)
}

/// `mine`'s tangent line on the first face (left before right) it shares with `theirs`, or
/// `None` where it shares none.
pub fn try_the_tangent_line_on_the_shared_face(
    mine: &SurfaceEdgePlan,
    theirs: &SurfaceEdgePlan,
) -> Option<SurfaceLine> {
    if is_one_of(&mine.left_region, theirs) {
        return Some(mine.left_tangent_line);
    }

    if is_one_of(&mine.right_region, theirs) {
        return Some(mine.right_tangent_line);
    }

    None
}

/// Whether `region` is (by identity) one of `other`'s two faces.
pub fn is_one_of(region: &Arc<FaceGroup>, other: &SurfaceEdgePlan) -> bool {
    Arc::ptr_eq(region, &other.left_region) || Arc::ptr_eq(region, &other.right_region)
}
