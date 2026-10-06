//! A side's rail carried on past a chain's end until the whole row has left the end plane. Ports
//! `Bevel/LevelLine/VirtualRailPastTheEnd.cs` (a C# static class, so a module of functions): the
//! loops carried past each rayed end ([`extended`], [`extend_both`]), where a run leaves the rail
//! at the end ([`to_the_rail`]) and where the row last touches the end plane ([`last_to_leave`]).
//!
//! Flow rung 3: a side's rail run on VIRTUALLY past a chain end that carries the end ray (an
//! oblique end face of the same sense). The end face is not cut by the chain's distance, so the cut
//! leaves each side's run ending on the side-to-end crease: the rail up to the point P where it
//! meets that crease, then the crease on up to the end vertex. Where the end face leans OUT over
//! the end (the section's top meets the end plane beyond P), the rows must run on past P until the
//! whole row has passed the end plane - the end plane then clips them (the trim, next rung). So the
//! crease stretch above P is dropped from the run and the rail is carried on in the side's own
//! plane, at the setback from the chain's extended line, to that parameter: one minted point, the
//! row there touching the end plane at its last point to leave it.
//!
//! Assumes a STRAIGHT end segment, a flat side across the stretch and a FILLET or chamfer row laid
//! as the fill lays it (a circular arc tangent to both sides, or the straight chord). A run whose
//! end does not stand on a ray end, or whose section passes the end plane before P, is returned as
//! it was.

use vector_math::{cs_math, Vector3};

use super::{BandGapResult, BandProfile, ChainField, GapLoop, LevelLineMesh, WorkError};

/// How far past the last row's touch of the end plane the rail is carried, in setbacks.
pub const PAST_THE_LAST_TOUCH: f64 = 0.25;

/// C# `VirtualRailPastTheEnd.Extended`: the loops with each side's run carried past each rayed end
/// where the row has not yet left the end plane at P; the loops themselves wherever nothing is
/// carried. Every vertex minted is appended to `minted`. C#'s optional `backFacingOnly = false` is
/// explicit here.
///
/// The gap's mesh is borrowed mutably while the runs are carried. A gap with no mesh (a refusal
/// raised before the working mesh existed - never handed here, since only a built gap is filled)
/// leaves the loops as they are.
pub fn extended(
    gap: &BandGapResult,
    loops: &[GapLoop],
    profile: BandProfile,
    minted: &mut Vec<i32>,
    back_facing_only: bool,
) -> Result<Vec<GapLoop>, WorkError> {
    if gap.chains.len() != 1 || loops.iter().any(|l| l.is_closed) {
        return Ok(loops.to_vec());
    }

    let chain = &*gap.chains[0];
    let rays = chain.end_rays;
    if !rays.0 && !rays.1 {
        return Ok(loops.to_vec());
    }

    let sides: Vec<Vec<&GapLoop>> = [1, 2]
        .iter()
        .map(|&s| loops.iter().filter(|l| l.side == s).collect())
        .collect();
    if sides.iter().any(|s| s.len() != 1) {
        return Ok(loops.to_vec());
    }

    let Some(mesh) = gap.mesh.as_ref() else {
        return Ok(loops.to_vec());
    };
    let mut mesh = mesh.borrow_mut();

    let mut left = sides[0][0].clone();
    let mut right = sides[1][0].clone();
    for last in [false, true] {
        if !(if last { rays.1 } else { rays.0 }) {
            continue;
        }

        let extended = extend_both(
            &mut mesh,
            chain,
            &left,
            &right,
            last,
            profile,
            minted,
            back_facing_only,
        )?;
        if let Some(extended) = extended {
            (left, right) = extended;
        }
    }

    Ok(loops
        .iter()
        .map(|l| match l.side {
            1 => left.clone(),
            2 => right.clone(),
            _ => l.clone(),
        })
        .collect())
}

/// C# `VirtualRailPastTheEnd.ExtendBoth`: both sides' runs carried past the `last` (or first) end
/// to where the row last leaves the end plane, as (left, right); `None` where that end is bounded,
/// has no end face facing as asked, a run has no rail end there, or the row has already left the
/// end plane at P.
#[allow(clippy::too_many_arguments)]
pub fn extend_both(
    mesh: &mut LevelLineMesh,
    chain: &ChainField,
    left: &GapLoop,
    right: &GapLoop,
    last: bool,
    profile: BandProfile,
    minted: &mut Vec<i32>,
    back_facing_only: bool,
) -> Result<Option<(GapLoop, GapLoop)>, WorkError> {
    // A BOUNDED ray's field is clipped by the end plane (ChainField.EndClip): the rails already
    // stop on it.
    let bound = if last {
        chain.end_ray_bounds.1
    } else {
        chain.end_ray_bounds.0
    };
    if bound.map(|b| b.same_sense) == Some(true) {
        return Ok(None);
    }

    let points = chain.points();
    let end = if last {
        points[points.len() - 1]
    } else {
        points[0]
    };
    let outward = (if last {
        points[points.len() - 1] - points[points.len() - 2]
    } else {
        points[0] - points[1]
    })
    .get_normal();
    let end_parameter = if last { chain.length } else { 0.0 };
    let setback = chain.setback_at(end_parameter);

    // The end face's plane: a kept, untagged triangle at the end vertex, faced along the chain.
    let mut normal: Option<Vector3> = None;
    let mut back_facing = false;
    let mut t = 0;
    while t < mesh.triangle_count() && normal.is_none() {
        let tri = mesh.triangles[t as usize];
        if mesh.is_removed(t)
            || tri.tag != 0
            || ((mesh.position(tri.a) - end).length() > mesh.same_place
                && (mesh.position(tri.b) - end).length() > mesh.same_place
                && (mesh.position(tri.c) - end).length() > mesh.same_place)
        {
            t += 1;
            continue;
        }

        let n = (mesh.position(tri.b) - mesh.position(tri.a))
            .cross(mesh.position(tri.c) - mesh.position(tri.a))
            .get_normal();
        if n.dot(outward).abs() > 1e-6 {
            normal = Some(if n.dot(outward) > 0.0 { n } else { -n });
            back_facing = n.dot(outward) < 0.0;
        }

        t += 1;
    }

    // The fill asks only for ends whose end face faces BACK over the chain (an opposite-sense wall
    // the round runs into short of the end vertex); the leaning-out reading stays the test's.
    let Some(plane) = normal else {
        return Ok(None);
    };
    if back_facing_only != back_facing {
        return Ok(None);
    }

    let left_cut = to_the_rail(mesh, chain, left, last, setback, back_facing)?;
    let right_cut = to_the_rail(mesh, chain, right, last, setback, back_facing)?;
    let (Some(left_cut), Some(right_cut)) = (left_cut, right_cut) else {
        return Ok(None);
    };

    // The row at the end vertex, and how far past the end its last point is from leaving the end
    // plane.
    let rail_left = end + (left_cut.away * setback);
    let rail_right = end + (right_cut.away * setback);
    let reach = last_to_leave(rail_left, rail_right, end, outward, plane, profile);
    let along = plane.dot(outward);
    let exit = end_parameter + ((if last { 1.0 } else { -1.0 }) * reach / along);
    let t_p = left_cut.parameter_at_p;
    if if last { exit <= t_p } else { exit >= t_p } {
        return Ok(None);
    }

    // The rail is carried a little PAST the row's last touch, so the last row stands wholly beyond
    // the plane: at the touch itself the row meets the plane at one point read only to float noise,
    // and the clip minted a second vertex beside it there (Fillet Thin Front: 92 and 101 at one
    // place).
    let past = (reach / along)
        + (if back_facing {
            PAST_THE_LAST_TOUCH * setback
        } else {
            0.0
        });
    let exit = end_parameter + ((if last { 1.0 } else { -1.0 }) * past);

    let mut with = |run: &GapLoop, cut: ToTheRail| {
        let vertex = mesh.add_vertex(end + (outward * past) + (cut.away * setback));
        minted.push(vertex);
        let mut vertices = run.vertices.clone();
        let mut parameters = run.parameters.clone();
        let keep = cut.keep_count as usize;
        if cut.at_tail {
            vertices.truncate(keep);
            parameters.truncate(keep);
            vertices.push(vertex);
            parameters.push(exit);
        } else {
            vertices.drain(0..vertices.len() - keep);
            parameters.drain(0..parameters.len() - keep);
            vertices.insert(0, vertex);
            parameters.insert(0, exit);
        }

        GapLoop::new(
            run.side, false, vertices, parameters, run.chain, false, None,
        )
    };

    let left = with(left, left_cut);
    let right = with(right, right_cut);
    Ok(Some((left, right)))
}

/// How far behind the end plane (through `end`, normal `normal`) the row between the two rail
/// points reaches at most: its ends, and for a fillet the top of its arc where that stands on the
/// end vertex's side of the chord.
pub fn last_to_leave(
    rail_left: Vector3,
    rail_right: Vector3,
    end: Vector3,
    outward: Vector3,
    normal: Vector3,
    profile: BandProfile,
) -> f64 {
    let behind = |q: Vector3| -(q - end).dot(normal);
    let mut best = cs_math::max_f64(behind(rail_left), behind(rail_right));
    if profile != BandProfile::Fillet {
        return best;
    }

    let to_left = rail_left - end;
    let to_right = rail_right - end;
    let cos = to_left
        .get_normal()
        .dot(to_right.get_normal())
        .clamp(-1.0, 1.0);
    let half = cos.acos() / 2.0;
    let bisector = (to_left.get_normal() + to_right.get_normal()).get_normal();
    let centre = end + (bisector * (to_left.length() / half.cos()));
    let radius = to_left.length() * half.tan();

    // The function's direction in the section plane (square to the chain).
    let direction = -normal - (outward * (-normal).dot(outward));
    if direction.length_squared() == 0.0 {
        return best;
    }

    let top = centre + (direction.get_normal() * radius);

    // On the arc when it stands on the end vertex's side of the chord between the rail points.
    let chord_mid = (rail_left + rail_right) / 2.0;
    if (top - chord_mid).dot(end - chord_mid) >= 0.0 {
        best = cs_math::max_f64(best, behind(top));
    }

    best
}

/// Where a run leaves the rail at an end ([`to_the_rail`]).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ToTheRail {
    /// How many vertices are kept, counted from the far end.
    pub keep_count: i32,
    /// The side's direction off the chain at P.
    pub away: Vector3,
    /// The chain parameter at P.
    pub parameter_at_p: f64,
    /// Whether the end is at the run's tail.
    pub at_tail: bool,
}

/// C# `VirtualRailPastTheEnd.ToTheRail`: where the run leaves the rail at the end: the run is cut
/// back to its last vertex ON the rail (P, where the rail meets the side-to-end crease), dropping
/// the crease stretch beyond it. `None` where nothing stands beyond P - the run already stops on
/// the rail (or has no rail at all) - unless `may_end_at_p`: a back-facing wall's side run ends AT
/// P (nothing of the crease runs on past it), so P is its last vertex.
pub fn to_the_rail(
    mesh: &LevelLineMesh,
    chain: &ChainField,
    run: &GapLoop,
    last: bool,
    setback: f64,
    may_end_at_p: bool,
) -> Result<Option<ToTheRail>, WorkError> {
    let at_tail = last == (run.parameters[run.parameters.len() - 1] >= run.parameters[0]);
    let mut ordered = run.vertices.clone();
    if !at_tail {
        ordered.reverse();
    }

    let on_rail = 1e-9 * cs_math::max_f64(1.0, setback) + (4.0 * mesh.same_place);
    let mut keep = ordered.len();
    while keep > 1
        && chain
            .evaluate(mesh.position(ordered[keep - 1]))?
            .value
            .abs()
            > on_rail
    {
        keep -= 1;
    }

    if (keep == ordered.len() && !may_end_at_p) || keep < 2 {
        return Ok(None);
    }

    let p = mesh.position(ordered[keep - 1]);
    let sample = chain.evaluate(p)?;
    let away = p - sample.nearest;
    Ok(Some(ToTheRail {
        keep_count: keep as i32,
        away: away.get_normal(),
        parameter_at_p: sample.parameter,
        at_tail,
    }))
}
