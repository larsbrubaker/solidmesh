//! The corner cap's end hole closed by the one filler. Ports `CornerCap.EndHoleByTheOneFiller`
//! from `Bevel/LevelLine/CornerCap.HoleFill.cs` (a partial of the C# static class `CornerCap`):
//! an end hole [`super::corner_cap_zip_end::zip_end`] cannot zip about V, closed flat where it is
//! planar and faired by [`super::corner_cap_hole_fill::fill_one`] otherwise
//! ([`end_hole_by_the_one_filler`]).

use vector_math::Vector3;

use super::corner_cap_flat_ends::closes_flat;
use super::corner_cap_hole_fill::fill_one;
use super::corner_cap_hole_fill_switch::use_hole_fill;
use super::{GapFillRefusal, LevelLineMesh, LevelLineMeshError};

/// C# `CornerCap.EndHoleByTheOneFiller`: an end hole [`super::corner_cap_zip_end::zip_end`] cannot
/// zip about V - the end run turns back round it - closed by the one filler instead of refused:
/// flat where the hole is planar, faired by [`fill_one`] otherwise. The loop is the row forward
/// then the run back from T to S, the way the cap lays its edges. Where the band flowed over an
/// end face narrower than the size (EndFaceFlow), the end run leaves the row's end across the
/// flowed face and comes back up the end wall (Goes to Sides' c17: the run drops 0.078 below the
/// plane of V, S and T), so it neither rises in angle about V nor lies in one plane; the old fan
/// refused it CornerCapNotStarShaped. No strip end is handed over: each row edge takes its strip
/// triangle's plane as its cross-tangent, so the patch leaves the fillet tangentially. `offending`
/// is kept where the filler refuses too.
#[allow(clippy::too_many_arguments)]
pub fn end_hole_by_the_one_filler(
    mesh: &mut LevelLineMesh,
    row: &[i32],
    run: &[i32],
    v: Vector3,
    added: &mut Vec<i32>,
    added_vertices: &mut Vec<i32>,
    offending: &mut i32,
) -> Result<GapFillRefusal, LevelLineMeshError> {
    if row[0] != run[0] || row[row.len() - 1] != run[run.len() - 1] || !use_hole_fill() {
        return Ok(GapFillRefusal::CornerCapNotStarShaped);
    }

    if closes_flat(mesh, row, run, added)? {
        return Ok(GapFillRefusal::None);
    }

    // row.Concat(run.Skip(1).Take(run.Count - 2).Reverse())
    let inner: &[i32] = if run.len() > 2 {
        &run[1..run.len() - 1]
    } else {
        &[]
    };
    let loop_: Vec<i32> = row
        .iter()
        .copied()
        .chain(inner.iter().rev().copied())
        .collect();
    let (refusal, filled) = fill_one(mesh, &loop_, &[v], Some(&[]), added, added_vertices)?;
    if refusal != GapFillRefusal::None {
        *offending = if filled >= 0 { filled } else { *offending };
    }

    Ok(refusal)
}
