//! The seams read again, from the mesh as it stands once the corners and the ends have cut it
//! about. Ports `Bevel/Surface/SurfaceSeamReRead.cs` (C# `internal static class`; free functions
//! here): [`take_in_the_later_splits`], [`later_splits_carried_by_a_face`] and
//! [`try_find_one_standing_off_the_seam`].
//!
//! A strip's two seams are picked from the walkback's tangent runs, before any corner bite or end
//! cut has run (`SurfaceBevelBuilder.TryPickTheRunOverTheEdge`). Those stages CUT faces, and a cut
//! lands new vertices wherever its plane crosses a side - including a side that is another chain's
//! tangent line, when the two chains' bands share a face. The sewing knits those vertices into
//! every face that carries one (`surface_split_sewing::sew_in_the_splits`), so the face beside the
//! line is split at them while the strip across the line, stitched from the walkback-time run,
//! spans them in one go: a row of T junctions along the seam, and a surface that does not close.
//!
//! So before the stitch every seam is read again: every vertex now standing on that side's run,
//! strictly between the run's own two ends, is taken into it in its place along the strip. The
//! ends are left exactly where they were - the mitre and end rings stand there, and the stitch
//! measures its windows from them.

use std::collections::HashSet;
use std::rc::Rc;

use super::surface_mitre_joint_step::SideSeams;
use super::surface_split_sewing::stands_on_the_side;
use super::{SurfaceStrip, SurfaceVertexPool};

/// C# `TakeInTheLaterSplits(pool, candidates, strips, seams)`: takes into every seam of `seams`
/// (indexed edge by side) the vertices that landed on its run after the walkback read it.
///
/// `candidates` are the vertices to look at - the ones a later stage made and a face now carries
/// ([`later_splits_carried_by_a_face`]). Per seam rather than per face, so the whole re-read costs
/// one pass over a handful of points for each of a chain's edges. `strips` are the strips, by edge
/// index, with `None` where a blend swallows the edge.
pub fn take_in_the_later_splits(
    pool: &SurfaceVertexPool,
    candidates: &[i32],
    strips: &[Option<Rc<SurfaceStrip>>],
    seams: &mut [SideSeams],
) {
    if candidates.is_empty() {
        return;
    }

    for (edge, strip) in strips.iter().enumerate() {
        for seam in seams[edge].iter_mut() {
            let Some(strip) = strip else {
                continue;
            };

            let Some(seam) = seam.as_mut() else {
                continue;
            };

            if seam.len() < 2 {
                continue;
            }

            take_in_the_later_splits_of_one(pool, candidates, strip, seam);
        }
    }
}

/// C# `LaterSplitsCarriedByAFace`: the vertices standing on faces that a stage after the walkback
/// made - everything numbered `made_from` or above that `faces` uses, each once, in the order the
/// faces first use them.
///
/// A vertex no face carries is not on any face's boundary either, so sewing the seam to it would
/// put a station where nothing meets it. Only the ones a face kept can be the far side of a T
/// junction.
pub fn later_splits_carried_by_a_face(faces: &[(i32, i32, i32)], made_from: i32) -> Vec<i32> {
    let mut carried = Vec::new();
    let mut seen = HashSet::new();

    for face in faces {
        for corner in [face.0, face.1, face.2] {
            if corner >= made_from && seen.insert(corner) {
                carried.push(corner);
            }
        }
    }

    carried
}

/// C#'s private `TakeInTheLaterSplits(pool, candidates, strip, seam)`: one seam, with everything
/// now standing between its ends taken into it in order.
fn take_in_the_later_splits_of_one(
    pool: &SurfaceVertexPool,
    candidates: &[i32],
    strip: &SurfaceStrip,
    seam: &mut Vec<(i32, f64)>,
) {
    let from = pool.position_of(seam[0].0);
    let to = pool.position_of(seam[seam.len() - 1].0);

    let mut taken: Option<Vec<(i32, f64)>> = None;

    for &candidate in candidates {
        if carries(seam, candidate)
            || !stands_on_the_side(pool, from, to, pool.position_of(candidate)).0
        {
            continue;
        }

        taken
            .get_or_insert_with(Vec::new)
            .push((candidate, strip.param_of(pool.position_of(candidate))));
    }

    let Some(taken) = taken else {
        return;
    };

    seam.extend(taken);

    // The run reads along the strip's own parameter, so sorting by it puts every new vertex where
    // it stands on the line - and leaves the two ends the ends, since everything taken in is
    // strictly between them. C#'s `List.Sort` is unstable; two seam points at one parameter would
    // be one point welded twice, so the stable sort here decides nothing C# decides differently.
    seam.sort_by(|first, second| first.1.total_cmp(&second.1));
}

/// C# `TryFindOneStandingOffTheSeam`: the first of `candidates` standing part way along `seam` with
/// no station of the seam at it, where there is one - which is a T junction about to be stitched
/// over.
///
/// The same reading [`take_in_the_later_splits`] makes, asked again at the stitch so that a vertex
/// the re-read could not take in refuses by name (`SurfaceStripSeamDidNotMeet`) rather than
/// publishing a crack.
pub fn try_find_one_standing_off_the_seam(
    pool: &SurfaceVertexPool,
    candidates: &[i32],
    seam: &[(i32, f64)],
) -> Option<i32> {
    if seam.len() < 2 {
        return None;
    }

    let from = pool.position_of(seam[0].0);
    let to = pool.position_of(seam[seam.len() - 1].0);

    candidates.iter().copied().find(|&candidate| {
        !carries(seam, candidate)
            && stands_on_the_side(pool, from, to, pool.position_of(candidate)).0
    })
}

/// C# `Carries`: whether the seam already has this vertex.
fn carries(seam: &[(i32, f64)], vertex: i32) -> bool {
    seam.iter().any(|on| on.0 == vertex)
}
