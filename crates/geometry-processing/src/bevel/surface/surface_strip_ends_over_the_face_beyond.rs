//! The end of a band that RAN OFF its landing face, closed over the face beyond. Ports
//! `Bevel/Surface/SurfaceStripEndsOverTheFaceBeyond.cs` (C# `internal static class`; a module of
//! free functions here). What is here is the end ring at a NOTCHED run-off end
//! ([`try_the_notched_ring`]); the close itself (`TryCloseOverTheFaceBeyond`) lands with a later
//! package. The trim it reads is [`SurfaceStripTrimmedByTheFaceBeyond`], the cross section comes
//! from [`super::surface_strip_tail_around_a_feature`].

use super::{
    surface_strip_tail_around_a_feature, SurfaceStrip, SurfaceStripNotchedByAFeature,
    SurfaceStripTrimmedAtTheCrossing, SurfaceStripTrimmedByTheFaceBeyond, SurfaceVertexPool,
};
use crate::bevel::BevelAdvisory;

/// C# `TryTheNotchedRing`: the strip's end ring at a NOTCHED end - every rail the feature took
/// pulled back to a cross section standing CLEAR of the notch, and every other rail standing where it
/// always did, on the end plane - with where each rail meets it (C#'s `ring` and `meetsAt`).
///
/// The stitch ends a strip on a ring of CHORDS, and the notch's own rim is not one: between the two
/// rails the ridge leaves through, the rim dips back to the exit, which is deeper than either rail's
/// crossing. So the strip stops short of the notch entirely, on a clean cross section standing clear
/// of it, and everything past it is sewn band by band (`SurfaceStripTailAroundAFeature`).
///
/// Only the notched rails move. An un-notched rail still reaches the end plane, which keeps the two
/// rails at the edges of the strip standing on their SEAMS' own last vertices - a point minted part
/// way along a seam would be a T junction with the walkback's face across it.
#[allow(clippy::too_many_arguments)]
pub fn try_the_notched_ring(
    pool: &mut SurfaceVertexPool,
    strip: &SurfaceStrip,
    off_the_face: &SurfaceStripTrimmedByTheFaceBeyond,
    notch: &SurfaceStripNotchedByAFeature,
    crossed_at_the_other_end: Option<&SurfaceStripTrimmedAtTheCrossing>,
    notched_at_the_other_end: Option<&SurfaceStripNotchedByAFeature>,
    on_the_seam: i32,
) -> Result<(Vec<i32>, Vec<f64>), BevelAdvisory> {
    let rails = strip.rail_count();

    let mut ring = vec![0i32; rails as usize];
    let mut meets_at = vec![0.0; rails as usize];

    let section = surface_strip_tail_around_a_feature::try_the_section_clear_of_the_notch(
        pool,
        strip,
        notch,
        notched_at_the_other_end,
        crossed_at_the_other_end,
        &off_the_face.trimmed_at,
    )?;

    for at in 0..rails {
        let rail =
            SurfaceStripTrimmedByTheFaceBeyond::rail_at(off_the_face.side, rails, at) as usize;

        if at < off_the_face.first_kept {
            // Air over the whole edge: the run-off already collapsed this rail onto the shared point,
            // and it stands there at the far parameter whatever the notch does.
            ring[rail] = off_the_face.shared;
            meets_at[rail] = off_the_face.far_param;
            continue;
        }

        meets_at[rail] = if notch.notches(rail as i32) {
            section
        } else {
            off_the_face.far_param
        };

        ring[rail] = if at == rails - 1 {
            on_the_seam
        } else {
            pool.add(strip.point_on(rail as i32, meets_at[rail]))
        };
    }

    Ok((ring, meets_at))
}
