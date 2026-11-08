//! Two polylines sewn into one band of triangles: the ZIPPER. Neither polyline has to carry the
//! same number of points as the other, and every point of both ends up in the band exactly once,
//! which is what lets a row of three chords hand over to a row of four with no vertex left on an
//! edge it does not end. The corner patch's rows (`SurfaceCornerPatchRows`) sew with it;
//! [`super::surface_fan`] winds what it collects.
//!
//! **Which one advances.** Each polyline is read by its own INDEX FRACTION - point `i` of a line of
//! `M` chords stands at `i / M` - and the step taken is whichever line's next point comes first,
//! the upper one winning a tie. That is the same rule a grid of equal rows already follows: two
//! rows of `k` and `k + 1` chords alternate lower, upper, lower, ... and end on a lower, which is
//! exactly the two triangles per cell the square corner's grid emits. Reproducing that case is not
//! a nicety - the cube octant is the shape the whole surface ladder was measured on, and it is held
//! to its own face list.
//!
//! **Why index fractions and not chord lengths.** Cumulative length would be the better ruler on a
//! row whose chords are uneven, and it is not used because it is not STABLE: the rows' interior
//! points are a barycentric mix put back on the sphere, so their chords differ from each other by a
//! rounding, and a ruler that reads those roundings would choose a different step for the square
//! corner than the grid did. Index fractions are exact in integers - `(i + 1) / M <= (j + 1) / N`
//! is `(i + 1) N <= (j + 1) M` - so the comparison never turns on a rounding at all.

use solidmesh_math::Vector3;

use super::surface_fan;
use super::SurfaceVertexPool;

/// Sews the band between two polylines, both running the same way, and both of whose ends are meant
/// to meet - `upper`'s first point pairs with `lower`'s first and its last with `lower`'s last.
///
/// The triangles are collected UNWOUND - in the order the zipper reads them, rather than turned to
/// face out of the material - because the patch is not finished when the last band is sewn: it is
/// refined once on the inside (`BevelPatchRefinement`) before anything is published, and a
/// triangle cut in four has to be wound afterwards anyway. [`emit_outward`] is the pass that does
/// it, and is the only thing that writes to a face list.
pub fn sew(band: &mut Vec<(i32, i32, i32)>, upper: &[i32], lower: &[i32]) {
    let over_upper = upper.len() as i64 - 1;
    let over_lower = lower.len() as i64 - 1;

    if over_upper < 0 || over_lower < 0 {
        return;
    }

    let mut i: i64 = 0;
    let mut j: i64 = 0;

    while i < over_upper || j < over_lower {
        // The upper line steps when it has a step left and its next point does not come after the
        // lower's - the tie going to the upper, which is what makes the equal-count band alternate.
        let step_upper =
            i < over_upper && (j == over_lower || (i + 1) * over_lower <= (j + 1) * over_upper);

        band.push((
            upper[i as usize],
            lower[j as usize],
            if step_upper {
                upper[(i + 1) as usize]
            } else {
                lower[(j + 1) as usize]
            },
        ));

        if step_upper {
            i += 1;
        } else {
            j += 1;
        }
    }
}

/// One triangle wound to face out of the MATERIAL, which for a patch on a sphere is away from
/// `centre` where the solid is inside that sphere and towards it where the solid is outside it.
/// Each triangle is wound by ITS OWN outward direction - its centroid seen from the centre - rather
/// than by one normal for the whole patch, which at an oblique corner is no longer within a right
/// angle of every triangle of it.
///
/// `material_outside_the_ball` says which side of the sphere the solid is on, and so which way
/// "out" is: false at a convex corner, where the ball is buried in the material and the surface
/// faces AWAY from `centre`; true at the inside corner of a pocket, where the ball is at rest in
/// the VOID and the same surface faces TOWARDS it.
pub fn emit_outward(
    pool: &SurfaceVertexPool,
    faces: &mut Vec<(i32, i32, i32)>,
    first: i32,
    second: i32,
    third: i32,
    centre: Vector3,
    material_outside_the_ball: bool,
) {
    if !surface_fan::is_worth_keeping(pool, first, second, third) {
        return;
    }

    let centroid =
        (pool.position_of(first) + pool.position_of(second) + pool.position_of(third)) / 3.0;
    let outward = if material_outside_the_ball {
        centre - centroid
    } else {
        centroid - centre
    };

    surface_fan::emit_triangle_facing(pool, faces, first, second, third, outward);
}
