//! The open gap fill's cut sections. Ports the nested record of
//! `Bevel/LevelLine/OpenGapFill.CutSection.cs` (a partial of the C# static class `OpenGapFill`): a
//! run-out's flat region ([`OpenGapFillFlatRunOutRegion`]) and the region a run-out's flat cut
//! rows close ([`flat_run_out`]), and whether a spoke fans from a run-out rail vertex
//! ([`fans_from_a_rail_run_out`]), and the cut section's row ([`cut_section_arc`]). The other
//! functions land with later packages.

use std::collections::HashSet;

use vector_math::{cs_math, Vector3};

use super::flat_hole_fill;
use super::gap_fill::{self, arc_point};
use super::open_gap_fill::{backwards, is_straight_row};
use super::{
    ChainField, HoleFillTriangle, LevelLineMesh, LevelLineMeshError, LevelLineWork,
    OpenGapFillOpenRail, WorkError,
};

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

/// C# `OpenGapFill.LayBesideACutRow`: the cell between two rows on one left vertex `a` where either
/// starts on the face `a` ran out onto (a cut row), read from their right ends as the
/// right-advancing strip reads them, with the rows' first edge laid too. Beside an in-plane row (a
/// straight row onto its far end B) the cut row fans onto B. Where no flat region takes the face's
/// piece, each cut row's crossing is fanned onto `a`.
#[allow(clippy::too_many_arguments)]
pub fn lay_beside_a_cut_row(
    mesh: &mut LevelLineMesh,
    before: &[i32],
    after: &[i32],
    before_cut: bool,
    after_cut: bool,
    a: i32,
    flat_region: bool,
    laid: &mut Vec<i32>,
) -> Result<(), LevelLineMeshError> {
    let u = backwards(before);
    let v = backwards(after);
    let last = u.len() - 1;
    if before_cut && !after_cut && is_straight_row(after) {
        let b = after[after.len() - 1];
        for r in 0..last {
            gap_fill::lay(mesh, u[r], b, u[r + 1], laid)?;
        }

        if !flat_region {
            gap_fill::lay(mesh, u[last], b, a, laid)?;
        }

        return Ok(());
    }

    for r in 0..last {
        gap_fill::lay(mesh, u[r], v[r], v[r + 1], laid)?;
        gap_fill::lay(mesh, u[r], v[r + 1], u[r + 1], laid)?;
    }

    if !flat_region && before_cut && after_cut {
        gap_fill::lay(mesh, u[last], v[last], a, laid)?;
    }

    Ok(())
}

/// A row drawn as a point at a fraction from its left end (0) to its right (1).
pub type RowAt = Box<dyn Fn(f64) -> Vector3>;

/// What [`cut_section_arc`] reads: the row, or `None` where the caller keeps its arc; whether B
/// itself stands in the cutting plane (the whole row is the straight edge A-B); and the fraction
/// where the section crosses that plane.
pub struct CutSectionArc {
    /// The row, or `None` where the reading does not hold.
    pub row: Option<RowAt>,
    /// True where B stands in the plane of the face A ran out onto.
    pub in_the_plane: bool,
    /// The fraction where the section crosses the plane; the round is the part from there to B.
    pub cut_at: f64,
}

/// C#'s `Math.Sign(double)` (its NaN throw is not reachable from finite rails).
fn sign(value: f64) -> i32 {
    if value > 0.0 {
        1
    } else if value < 0.0 {
        -1
    } else {
        0
    }
}

/// C# `OpenGapFill.CutSectionArc`: the row for a spoke fanning from the run-out left vertex A
/// (`left`'s vertex `l`), as a point at fraction f from A (0) to the right vertex B (`right`'s
/// vertex `r`, 1): the square section of the round at B's parameter, cut by the plane of the face A
/// ran out onto, the cut part replaced by the straight line in that plane back to A. B itself in
/// that plane makes the whole row the straight edge A-B. No row where the reading does not hold (a
/// turning chain between the two parameters, B not on the round, no cut): the caller keeps its
/// arc.
pub fn cut_section_arc(
    mesh: &LevelLineMesh,
    chain: &ChainField,
    left: &OpenGapFillOpenRail,
    l: usize,
    right: &OpenGapFillOpenRail,
    r: usize,
) -> Result<CutSectionArc, WorkError> {
    let none = |in_the_plane: bool| CutSectionArc {
        row: None,
        in_the_plane,
        cut_at: 0.0,
    };
    let t_a = left.parameters[l];
    let t_b = right.parameters[r];
    let (at_a, c_a) = chain.point_on_the_ray(t_a);
    let (at_b, c_b) = chain.point_on_the_ray(t_b);
    if at_a != t_a || at_b != t_b {
        return Ok(none(false));
    }

    // straight between the two parameters, so the round there is a prism along the chain
    let scale = cs_math::max_f64(1.0, chain.length);
    if ((c_b - c_a).length() - (t_b - t_a)).abs() > 1e-6 * scale {
        return Ok(none(false));
    }

    let pinned = left.vertices[l];
    let a = mesh.position(pinned);
    let b = mesh.position(right.vertices[r]);
    let previous = mesh.position(left.vertices[l - 1]);

    // The face A ran out onto: of the kept triangles on A, the one whose plane stands farthest
    // from the left rail's previous vertex (which lies in the face A ran out of).
    let mut normal = Vector3::ZERO;
    let mut farthest = 0.0;
    for t in 0..mesh.triangle_count() {
        LevelLineWork::charge(1)?;
        let triangle = mesh.triangles()[t as usize];
        if mesh.is_removed(t)
            || (triangle.a != pinned && triangle.b != pinned && triangle.c != pinned)
        {
            continue;
        }

        let corner = mesh.position(triangle.a);
        let mut n = (mesh.position(triangle.b) - corner).cross(mesh.position(triangle.c) - corner);
        if n.length() <= 0.0 {
            continue;
        }

        n = n.get_normal();
        let off = n.dot(previous - a).abs();
        if off > farthest {
            farthest = off;
            normal = n;
        }
    }

    let snap = 1e-6 * scale;
    if farthest <= snap {
        return Ok(none(false));
    }

    let side = move |point: Vector3| normal.dot(point - a);

    // B already in the cutting plane: the whole row lies in it.
    if side(b).abs() <= snap {
        return Ok(none(true));
    }

    // B's own section, square to the chain's line, and A carried square into it: the round's
    // left tangent there. A can stand short of the tangent (it is read linearly along the edge it
    // ran out over), so the section's left end is put AT the tangent distance, in A's direction.
    let line = (c_b - c_a).get_normal();
    let c_b = c_a + (line * (b - c_a).dot(line));
    let mut a_square = a + (line * (b - a).dot(line));
    let tangent = (b - c_b).length();
    if ((a_square - c_b).length() - tangent).abs() > 0.1 * tangent {
        return Ok(none(false));
    }

    a_square = c_b + ((a_square - c_b).get_normal() * tangent);
    if sign(side(a_square)) == sign(side(b)) {
        return Ok(none(false));
    }

    let section = move |f: f64| arc_point(a_square, c_b, b, f);
    let (mut low, mut high) = (0.0, 1.0);
    for _ in 0..60 {
        let mid = (low + high) / 2.0;
        if sign(side(section(mid))) == sign(side(b)) {
            high = mid;
        } else {
            low = mid;
        }
    }

    let cut = (low + high) / 2.0;
    let on_plane = section(cut);
    Ok(CutSectionArc {
        row: Some(Box::new(move |f: f64| {
            if f >= cut {
                section(f)
            } else {
                a + ((on_plane - a) * (f / cut))
            }
        })),
        in_the_plane: false,
        cut_at: cut,
    })
}
