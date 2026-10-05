//! The open gap fill's cut sections. Ports the nested record of
//! `Bevel/LevelLine/OpenGapFill.CutSection.cs` (a partial of the C# static class `OpenGapFill`): a
//! run-out's flat region ([`OpenGapFillFlatRunOutRegion`]) and the region a run-out's flat cut
//! rows close ([`flat_run_out`]), and whether a spoke fans from a run-out rail vertex
//! ([`fans_from_a_rail_run_out`]). The other functions land with later packages.

use std::collections::HashSet;

use vector_math::cs_math;

use super::flat_hole_fill;
use super::open_gap_fill::is_straight_row;
use super::{ChainField, HoleFillTriangle, LevelLineMesh, OpenGapFillOpenRail};

/// C# `OpenGapFill.FlatRunOutRegion`: a run-out's flat region - its loop of mesh vertices, the
/// triangles closing it (loop indices), and the last cut row.
#[derive(Debug, Clone, PartialEq)]
pub struct OpenGapFillFlatRunOutRegion {
    /// The region's loop of mesh vertices.
    pub r#loop: Vec<i32>,
    /// The triangles closing the loop, as indices into it.
    pub triangles: Vec<HoleFillTriangle>,
    /// The last cut row.
    pub last_cut: i32,
}

/// The flat region a run-out leaves where its cut rows (`on_the_plane`) all fan from ONE left rail
/// vertex and every later row is laid straight from it: that vertex, the cut rows' first vertices
/// and the straight rows' last ones, closed by the flat hole fill. `None` where the rows are not
/// that shape, the loop repeats a vertex, or the fill refuses.
pub fn flat_run_out(
    mesh: &LevelLineMesh,
    chain: &ChainField,
    spokes: &[(i32, i32)],
    arcs: &[Vec<i32>],
    left: &OpenGapFillOpenRail,
    on_the_plane: &[bool],
) -> Option<OpenGapFillFlatRunOutRegion> {
    let first_cut = on_the_plane.iter().position(|&on| on)?;
    let last_cut = on_the_plane.iter().rposition(|&on| on)?;
    if last_cut + 1 >= spokes.len() {
        return None;
    }

    let a = left.vertices[spokes[first_cut].0 as usize];
    let mut r#loop = vec![a];
    for k in first_cut..=last_cut {
        if !on_the_plane[k] || spokes[k].0 != spokes[first_cut].0 {
            return None;
        }

        r#loop.push(arcs[k][0]);
    }

    for k in last_cut + 1..spokes.len() {
        let row = &arcs[k];
        if spokes[k].0 != spokes[first_cut].0 || row[0] != a || !is_straight_row(row) {
            return None;
        }

        r#loop.push(row[row.len() - 1]);
    }

    if r#loop.iter().collect::<HashSet<_>>().len() != r#loop.len() {
        return None;
    }

    let points: Vec<_> = r#loop.iter().map(|&v| mesh.position(v)).collect();
    let (triangles, _) =
        flat_hole_fill::fill(&points, 1e-6 * cs_math::max_f64(1.0, chain.length), None)?;
    Some(OpenGapFillFlatRunOutRegion {
        r#loop,
        triangles,
        last_cut: last_cut as i32,
    })
}

/// Whether spoke (`l`, `r`) fans from a left-rail vertex that ran out: the left vertex is shared
/// with other spokes and the right one stands further along the chain (by more than `tie`).
pub fn fans_from_a_rail_run_out(
    spokes: &[(i32, i32)],
    l: i32,
    r: i32,
    left: &OpenGapFillOpenRail,
    right: &OpenGapFillOpenRail,
    tie: f64,
) -> bool {
    l > 0
        && right.parameters[r as usize] > left.parameters[l as usize] + tie
        && spokes.iter().filter(|spoke| spoke.0 == l).count() > 1
}
