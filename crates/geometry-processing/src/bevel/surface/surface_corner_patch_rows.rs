//! The spherical triangle between three rim arcs, laid as PARAMETRIC ROWS and sewn band by band -
//! which is what a corner of three UNEQUAL wedges needs, its three arcs sweeping three different
//! angles and so being chorded three different numbers of times. Ports
//! `Bevel/Surface/SurfaceCornerPatchRows.cs` (C# `internal static class`; a module of free
//! functions here). The bands are sewn with [`super::surface_row_zip`] and the interior refined
//! with [`crate::bevel::bevel_patch_refinement`].
//!
//! **Where the rows run.** The apex is the corner opposite the LONGEST side, so the side carrying
//! the most chords is the one the rows run down to and no row ever has to be coarser than the arc
//! it ends on. Row `k` of `q` ends on vertex `k` of the shorter of the two sides from the apex and
//! on vertex `round(k p / q)` of the longer, and carries `round(k n / q)` chords of its own, so
//! that the last row IS the base - every rim vertex of all three arcs is a row end or a row point.
//!
//! **Where the extra rim vertices go.** Between two rows the long side may carry more than one
//! chord, and those surplus vertices are appended to the upper row's end before the band is sewn,
//! which keeps the arc's chords as edges of the band rather than leaving a rim vertex standing in
//! the middle of a triangle's edge, which is a crack.
//!
//! **A POCKET corner is the same rows reversed**, each triangle wound the other way about, which is
//! the one thing `material_outside_the_ball` changes. **The square corner is the same thing**: with
//! `p = q = n` there is never a surplus and the zipper alternates - the cube octant's triangular
//! grid, connectivity for connectivity.
//!
//! **And then the inside is cut once more.** The rows sag a sixty-fifth of the radius at eight
//! segments, over the chord tolerance, so every row is laid as a great-circle arc between its own
//! two rim ends, and the sewn bands are refined once on the INTERIOR before they are published -
//! rim arcs untouched, since they are three strips' own end rings (`refined`).
//!
//! C# `Math.Round(.., MidpointRounding.AwayFromZero)` is `f64::round`.

use std::collections::HashMap;

use vector_math::Vector3;

use super::{surface_row_zip, SurfaceVertexPool};
use crate::bevel::{bevel_patch_refinement, bevel_seam_ring};

/// How nearly a row's two rim ends may stand OPPOSITE each other on the ball before the row is no
/// longer one arc: the cosine of about 179.92 degrees. Two antipodal points have no shortest arc
/// between them, and the slerp that would lay it divides by the sine of that angle and answers a
/// direction made almost entirely of rounding. A refusal by name says where the build stopped,
/// where a NaN quietly dropped by the area check would hide it.
const ROW_ENDS_STAND_OPPOSITE: f64 = -0.999999;

/// C# `TryEmit`: lays the rows and sews them. False where the rim is not on one sphere about
/// `centre`, or where the three sides do not make a triangle to put rows in.
///
/// `sides` are the three rim arcs, each running on from where the last left off; `size` is the
/// ball's radius; `material_outside_the_ball` is false for the convex octant, whose ball is buried
/// in the material, and true for the inside corner of a pocket, whose ball is at rest in the void -
/// the same rings, the same rows, the same points, wound the other way about.
pub fn try_emit(
    pool: &mut SurfaceVertexPool,
    faces: &mut Vec<(i32, i32, i32)>,
    sides: &[Vec<i32>],
    centre: Vector3,
    size: f64,
    material_outside_the_ball: bool,
) -> bool {
    if sides.len() != 3 || !the_rim_is_on_the_ball(pool, sides, centre, size) {
        return false;
    }

    let mut chords = [0i32; 3];

    for side in 0..3 {
        chords[side] = sides[side].len() as i32 - 1;

        if chords[side] < 1 {
            return false;
        }
    }

    // The apex stands opposite the longest side. A tie is the square corner, where any of the three
    // gives the same rows, and the FIRST is taken so that the octant's own face list does not move.
    let mut apex_at = 0;

    for corner in 1..3 {
        if chords[(corner + 1) % 3] > chords[(apex_at + 1) % 3] {
            apex_at = corner;
        }
    }

    // The two sides from the apex, both read FROM it, and the base between their far ends.
    let to_last = sides[apex_at].clone();
    let to_first = turned(&sides[(apex_at + 2) % 3]);

    // Rows run from the short side to the long one, so that the surplus is always at a row's end.
    // The tie again goes the way the grid went: the short side first, the long side last.
    let short_first = to_first.len() <= to_last.len();

    let base_row = if short_first {
        turned(&sides[(apex_at + 1) % 3])
    } else {
        sides[(apex_at + 1) % 3].clone()
    };
    let (short_side, long_side) = if short_first {
        (to_first, to_last)
    } else {
        (to_last, to_first)
    };

    let q = short_side.len() as i32 - 1;
    let p = long_side.len() as i32 - 1;
    let n = base_row.len() as i32 - 1;

    let mut rows: Vec<Vec<i32>> = Vec::with_capacity(q as usize + 1);
    rows.push(vec![short_side[0]]);
    let mut ends_on = vec![0i32; q as usize + 1];

    ends_on[q as usize] = p;

    for k in 1..q {
        let ku = k as usize;
        ends_on[ku] = (k as f64 * p as f64 / q as f64).round() as i32;

        if ends_on[ku] <= ends_on[ku - 1] || ends_on[ku] >= p {
            // Two rows ending on one rim vertex would leave the arc between them unsewn, and a row
            // ending past the base is no row at all.
            return false;
        }

        let Some(row) = row_across(
            pool,
            k,
            q,
            ends_on[ku],
            n,
            &short_side,
            &long_side,
            centre,
            size,
        ) else {
            // No arc to lay the row on - see ROW_ENDS_STAND_OPPOSITE. The corner refuses whole rather
            // than lidding with points the winding would silently drop.
            return false;
        };

        rows.push(row);
    }

    rows.push(base_row);

    let mut laid = Vec::new();

    for band in 0..rows.len() - 1 {
        surface_row_zip::sew(
            &mut laid,
            &with_the_surplus(&rows[band], &long_side, ends_on[band], ends_on[band + 1]),
            &rows[band + 1],
        );
    }

    for (v0, v1, v2) in refined(pool, &laid, sides, centre, size) {
        surface_row_zip::emit_outward(pool, faces, v0, v1, v2, centre, material_outside_the_ball);
    }

    true
}

/// C# `Refined`: the rows cut once more on the INSIDE - every edge that is not on one of the three
/// rim arcs split at its great-circle midpoint, which is
/// [`bevel_patch_refinement::refine`]'s own rule and code, read here in the pool's indices.
///
/// The rim arcs ARE the strips' own end rings - the patch is sewn to them by index - so their chord
/// count cannot change here. The rows alone sag 0.0763 on a square pocket corner at r 5 and eight
/// segments against a chord tolerance of 0.05; one level of midpoints reads 0.0299 there. The count
/// stays predictable: `4T - R` faces, with `T` the rows' triangles and `R` the rim's own chords
/// (on the fixture's 6/8/6 corner, `4 * 48 - 20 = 172`).
fn refined(
    pool: &mut SurfaceVertexPool,
    laid: &[(i32, i32, i32)],
    sides: &[Vec<i32>],
    centre: Vector3,
    size: f64,
) -> Vec<(i32, i32, i32)> {
    // Refine works in DIRECTIONS about the centre and in its own index space, so the patch's pool
    // vertices are numbered locally for it and numbered back afterwards. The midpoints it appends
    // come back through pool.add - minted once each, and shared with anything already standing
    // there. The local map is only looked up, so its order cannot reach the output.
    let mut directions: Vec<Vector3> = Vec::new();
    let mut in_the_pool: Vec<i32> = Vec::new();
    let mut locally: HashMap<i32, i32> = HashMap::new();

    let mut local_of = |pool: &SurfaceVertexPool,
                        directions: &mut Vec<Vector3>,
                        in_the_pool: &mut Vec<i32>,
                        vertex: i32|
     -> i32 {
        if let Some(&local) = locally.get(&vertex) {
            return local;
        }

        let local = directions.len() as i32;

        directions.push((pool.position_of(vertex) - centre).get_normal());
        in_the_pool.push(vertex);
        locally.insert(vertex, local);

        local
    };

    let mut walks: Vec<Vec<i32>> = Vec::with_capacity(sides.len());

    for side in sides {
        let walk: Vec<i32> = side
            .iter()
            .map(|&vertex| local_of(pool, &mut directions, &mut in_the_pool, vertex))
            .collect();

        walks.push(walk);
    }

    let mut patch = Vec::with_capacity(laid.len());

    for &(v0, v1, v2) in laid {
        let a = local_of(pool, &mut directions, &mut in_the_pool, v0);
        let b = local_of(pool, &mut directions, &mut in_the_pool, v1);
        let c = local_of(pool, &mut directions, &mut in_the_pool, v2);
        patch.push((a, b, c));
    }

    let refined = bevel_patch_refinement::refine(&mut directions, &patch, Some(&walks));

    for &direction in &directions[in_the_pool.len()..] {
        in_the_pool.push(pool.add(centre + (direction * size)));
    }

    refined
        .iter()
        .map(|&(a, b, c)| {
            (
                in_the_pool[a as usize],
                in_the_pool[b as usize],
                in_the_pool[c as usize],
            )
        })
        .collect()
}

/// C# `RowAcross`: one row - its two ends are rim vertices the strips already own, and its interior
/// is the GREAT CIRCLE between those two ends, sampled in equal angles. `None` (C# null) where the
/// row's two ends stand OPPOSITE on the ball and there is no one arc between them
/// ([`ROW_ENDS_STAND_OPPOSITE`]), which the caller turns into the corner's own refusal.
///
/// A row is a curve on the ball between two points of the ball, and the arc between them is the
/// shortest one there is - so its chords are the shortest the row can be laid with, and a chord's
/// sag goes as the square of its length. The barycentric mix this used to be stretched the lattice
/// (0.0763 against the arc's 0.0630 on a pocket corner at r 5 and eight segments).
#[allow(clippy::too_many_arguments)]
fn row_across(
    pool: &mut SurfaceVertexPool,
    k: i32,
    q: i32,
    j: i32,
    n: i32,
    short_side: &[i32],
    long_side: &[i32],
    centre: Vector3,
    size: f64,
) -> Option<Vec<i32>> {
    let carries = 1.max((k as f64 * n as f64 / q as f64).round() as i32) as usize;
    let mut row = vec![0i32; carries + 1];

    row[0] = short_side[k as usize];
    row[carries] = long_side[j as usize];

    let from_end = (pool.position_of(row[0]) - centre).get_normal();
    let to_end = (pool.position_of(row[carries]) - centre).get_normal();

    if from_end.dot(to_end) < ROW_ENDS_STAND_OPPOSITE {
        return None;
    }

    for (step, slot) in row.iter_mut().enumerate().take(carries).skip(1) {
        let across = step as f64 / carries as f64;

        *slot = pool.add(centre + (bevel_seam_ring::slerp(from_end, to_end, across) * size));
    }

    Some(row)
}

/// C# `WithTheSurplus`: a row with the long side's own rim vertices between this row's end and the
/// next row's end put on the end of it, so that the band below carries every one of them.
fn with_the_surplus(row: &[i32], long_side: &[i32], from: i32, to: i32) -> Vec<i32> {
    if to <= from + 1 {
        return row.to_vec();
    }

    let mut carried = Vec::with_capacity(row.len() + (to - from) as usize);

    carried.extend_from_slice(row);

    for at in from + 1..to {
        carried.push(long_side[at as usize]);
    }

    carried
}

/// C# `TheRimIsOnTheBall`: whether every rim vertex stands the radius from the centre. It does
/// whenever each strip's rails lie on a cylinder about an axis through the centre; rails ROLLED
/// along a curved wall lie on no such cylinder, and there the rim is three curves the sphere does
/// not carry - which is a corner this patch cannot be, rather than one it may lid badly.
fn the_rim_is_on_the_ball(
    pool: &SurfaceVertexPool,
    sides: &[Vec<i32>],
    centre: Vector3,
    size: f64,
) -> bool {
    for side in sides {
        for &vertex in side {
            if ((pool.position_of(vertex) - centre).length() - size).abs() > pool.near_enough() {
                return false;
            }
        }
    }

    true
}

/// C# `Turned`: a side read the other way round.
fn turned(side: &[i32]) -> Vec<i32> {
    side.iter().rev().copied().collect()
}
