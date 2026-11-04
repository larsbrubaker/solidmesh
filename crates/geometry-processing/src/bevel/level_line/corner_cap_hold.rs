//! The corner cap's held patch: the vertices the hole filler added, held within the faces the rows
//! leave along ([`held_within_the_faces`]), and the held patch checked again as the filler's own
//! was. A vertex standing through faces (in front of a convex row's face, behind a concave row's)
//! is moved to the CLOSEST point standing on or behind every one of them
//! ([`closest_within_the_planes`]): where two or three planes hold it, that point lies where they
//! meet, solved exactly in f64 over every subset of up to three planes. The held patch's shape
//! re-checks are [`the_held_patch_fails`]. The faces are
//! `corner_cap_hole_fill::faces_the_rows_leave_along`'s, and `corner_cap_hole_fill::fill_one`
//! refuses the patch where the hold returns `None`.

use vector_math::{ieee_math, Vector3};

use super::corner_cap_hole_fill::{faces_the_rows_leave_along, FacePlane};
use super::{hole_fill, CornerCapLastRow, HoleFillFailure, HoleFillResult, LevelLineMesh};

/// The added fill vertices (index `n` on) held on the band's side of the faces the rows leave
/// along: a vertex the faired filler left standing through such a face, by less than the bulge
/// allowance `bulging_fill_vertex` refuses past, is moved to the closest point on or behind every
/// face ([`closest_within_the_planes`]). The held patch is then checked again as the filler's own
/// was: `None` (the patch refused) where a vertex still stands through any face, or holding turned
/// a triangle over or flattened one (`hole_fill::validate`). A corner of one sense only: at a mixed
/// corner a vertex stands through a row's face legitimately where it belongs to the other row.
pub fn held_within_the_faces(
    mesh: &LevelLineMesh,
    mut filled: HoleFillResult,
    n: i32,
    rows: &[CornerCapLastRow],
    points: &[Vector3],
) -> Option<HoleFillResult> {
    let mixed = rows.iter().any(|r| r.concave) && rows.iter().any(|r| !r.concave);
    if mixed || rows.is_empty() {
        return Some(filled);
    }

    // No removed face near the hole: nothing to hold the patch within, so it stands as filled.
    let Some(faces) = faces_the_rows_leave_along(mesh, rows, points) else {
        return Some(filled);
    };
    // Each face as a half-space the vertex must stand on or behind: (origin, outward normal), the
    // outward side the one a vertex must not stand on.
    // Both arc ends of a row often lie on one face: a plane already in the list (same outward
    // normal, origin on it) is read once, so the planes holding a vertex stay independent.
    let mut planes: Vec<FacePlane> = Vec::new();
    for &(concave, a, b) in &faces {
        let sign = if concave { -1.0 } else { 1.0 };
        for (origin, normal) in [a, b] {
            let outward = normal * sign;
            let same = |&(o, m): &FacePlane| {
                m.dot(outward) >= 1.0 - 1e-12
                    && m.dot(origin - o).abs() <= 1e-9 * (1.0 + o.length())
            };
            if !planes.iter().any(same) {
                planes.push((origin, outward));
            }
        }
    }
    let before: Vec<Vector3> = filled.vertices.clone();
    for point in filled.vertices.iter_mut().skip(n as usize) {
        *point = closest_within_the_planes(*point, &planes)?;
    }

    // The re-checks the filler's own patch passed: through no face past rounding at the vertex's
    // own magnitude, no triangle turned over, none flattened.
    let through = |point: Vector3| {
        planes
            .iter()
            .map(|&(origin, outward)| outward.dot(point - origin))
            .fold(f64::NEG_INFINITY, ieee_math::max_f64)
    };
    let within = |point: Vector3| through(point) <= 1e-9 * (1.0 + point.length());
    if !filled
        .vertices
        .iter()
        .skip(n as usize)
        .all(|&point| within(point))
    {
        return None;
    }

    if the_held_patch_fails(&before, &filled, n as usize).is_some() {
        return None;
    }

    Some(filled)
}

/// Which of the two shape re-checks a held patch fails, against the filler's own vertices
/// `before` (same triangles): [`HeldPatchFault::TurnedOver`] where a triangle's normal no longer
/// points the way it did (a flattened one, its normal zero, counts as turned), else
/// [`HeldPatchFault::Invalid`] where the filler's own validation (`hole_fill::validate`, with the
/// loop's tiny area) refuses it; `None` where it passes both.
pub fn the_held_patch_fails(
    before: &[Vector3],
    held: &HoleFillResult,
    loop_count: usize,
) -> Option<HeldPatchFault> {
    let normal = |v: &[Vector3], t: &super::HoleFillTriangle| {
        (v[t.b as usize] - v[t.a as usize]).cross(v[t.c as usize] - v[t.a as usize])
    };
    let kept_its_turn = held
        .triangles
        .iter()
        .all(|t| normal(before, t).dot(normal(&held.vertices, t)) > 0.0);
    if !kept_its_turn {
        return Some(HeldPatchFault::TurnedOver);
    }

    let tiny = hole_fill::tiny_area(&held.vertices[..loop_count]);
    let failure = hole_fill::validate(&held.vertices, &held.triangles, loop_count, tiny);
    (failure != HoleFillFailure::None).then_some(HeldPatchFault::Invalid(failure))
}

/// Why a held patch is refused ([`the_held_patch_fails`]).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum HeldPatchFault {
    /// A triangle turned over (or flattened to no normal) by the hold.
    TurnedOver,
    /// The filler's own validation refuses the held patch.
    Invalid(HoleFillFailure),
}

/// The closest point to `p` standing on or behind every plane (each `(origin, unit outward
/// normal)`): `p` itself where it crosses none (past rounding at its own magnitude). Otherwise the
/// closest point lies where some one, two or three of the planes meet, with every weight pulling
/// it inward: each subset of up to three planes is tried, in order (by size, then by the planes'
/// indices), its projection solved exactly in f64 ([`project_onto`]), and kept where no weight is
/// negative and it stands within every plane; the nearest kept wins, the earliest subset on a tie.
/// The region is convex, so that is the one closest point. `None` where no subset holds the point
/// (the planes leave no room, or they meet only where they are too near parallel to solve).
pub fn closest_within_the_planes(p: Vector3, planes: &[(Vector3, Vector3)]) -> Option<Vector3> {
    let slack = |x: Vector3| 1e-12 * (1.0 + x.length());
    let within = |x: Vector3| {
        planes
            .iter()
            .all(|&(origin, outward)| outward.dot(x - origin) <= slack(x))
    };
    if within(p) {
        return Some(p);
    }

    let n = planes.len();
    let mut subsets: Vec<Vec<usize>> = (0..n).map(|i| vec![i]).collect();
    for i in 0..n {
        for j in i + 1..n {
            subsets.push(vec![i, j]);
        }
    }
    for i in 0..n {
        for j in i + 1..n {
            for k in j + 1..n {
                subsets.push(vec![i, j, k]);
            }
        }
    }

    let mut best: Option<(f64, Vector3)> = None;
    for subset in &subsets {
        let Some((x, weights)) = project_onto(p, planes, subset) else {
            continue;
        };
        if weights.iter().any(|&w| w.is_nan() || w < -slack(p)) || !within(x) {
            continue;
        }
        let distance = (x - p).length_squared();
        if best.is_none_or(|(nearest, _)| distance < nearest) {
            best = Some((distance, x));
        }
    }

    best.map(|(_, x)| x)
}

/// The projection of `p` onto the planes `active` names (as equalities), with each plane's weight:
/// x = p - sum(w_j m_j), where m_j·(x - o_j) = 0 for every active j - the Gram system
/// sum_j (m_k·m_j) w_j = m_k·(p - o_k), at most three, solved by Gaussian elimination with partial
/// pivoting (ties to the earlier row). `None` where the system is singular or over three planes.
fn project_onto(
    p: Vector3,
    planes: &[(Vector3, Vector3)],
    active: &[usize],
) -> Option<(Vector3, Vec<f64>)> {
    let k = active.len();
    if k == 0 || k > 3 {
        return None;
    }

    let mut a = [[0.0_f64; 4]; 3];
    for (r, &i) in active.iter().enumerate() {
        let (origin, m) = planes[i];
        for (c, &j) in active.iter().enumerate() {
            a[r][c] = m.dot(planes[j].1);
        }
        a[r][3] = m.dot(p - origin);
    }

    for col in 0..k {
        let mut pivot = col;
        for row in col + 1..k {
            if a[row][col].abs() > a[pivot][col].abs() {
                pivot = row;
            }
        }
        // Planes this near parallel do not meet where a hold can use; a NaN meets nowhere.
        let size = a[pivot][col].abs();
        if size.is_nan() || size <= 1e-12 {
            return None;
        }
        a.swap(col, pivot);
        for row in 0..k {
            if row != col {
                let factor = a[row][col] / a[col][col];
                let source = a[col];
                for (target, &from) in a[row][col..].iter_mut().zip(&source[col..]) {
                    let scaled = factor * from;
                    *target -= scaled;
                }
            }
        }
    }

    let weights: Vec<f64> = (0..k).map(|r| a[r][3] / a[r][r]).collect();
    let mut x = p;
    for (r, &i) in active.iter().enumerate() {
        x -= planes[i].1 * weights[r];
    }
    Some((x, weights))
}
