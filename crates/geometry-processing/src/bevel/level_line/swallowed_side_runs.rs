//! A side the band swallows whole: the tests and the made-up runs. Ports the swallow readings of
//! `Bevel/LevelLine/SwallowedSide.cs` (a C# static class; its mesh readings are in
//! `swallowed_side.rs`): the growing end walls ([`growing_walls`], [`back_facing_wall`]), whether a
//! side is swallowed ([`is_swallowed`]), the run made up off the mesh ([`virtual_run`],
//! [`virtual_run_over`]), the span the rows must cover ([`row_span`]), the room where the two ties
//! merge ([`ties_merge_room`]) and both sides' runs ([`runs_if_both_swallowed`]). The swallowed
//! strip's fill lands with its method package.

use vector_math::{cs_math, Vector3};

use super::swallowed_side::{away_on_side, side_vertices, tolerance, touches};
use super::virtual_rail_past_the_end::{last_to_leave, PAST_THE_LAST_TOUCH};
use super::{BandProfile, ChainField, LevelLineMesh, WorkError};

/// A wall as a point on it and its outward (air-side) unit normal.
pub type Wall = (Vector3, Vector3);

/// A made-up run: rail points with the chain parameter each stands at.
pub type VirtualRun = Vec<(Vector3, f64)>;

/// C# `SwallowedSide.IsSwallowed`: true when `side` of the single open `chain` lies wholly inside
/// the strip at the chain's setback, both chain ends stand on growing (back-facing) walls that are
/// not parallel, every vertex of the side stands on or in front of both wall planes, and the side
/// reaches the crease where the two walls meet (a vertex off the chain on both planes) - so the
/// room need not stop at that far corner: the walls clip the whole strip there.
pub fn is_swallowed(
    mesh: &LevelLineMesh,
    chain: &ChainField,
    side: i32,
) -> Result<bool, WorkError> {
    let Some((first, last)) = growing_walls(mesh, chain) else {
        return Ok(false);
    };

    let tolerance = tolerance(chain);
    let vertices = side_vertices(mesh, side);
    if vertices.is_empty() {
        return Ok(false);
    }

    let mut reaches_the_shared_crease = false;
    for &vertex in &vertices {
        let q = mesh.position(vertex);
        let sample = chain.evaluate(q)?;
        let to_first = (q - first.0).dot(first.1);
        let to_last = (q - last.0).dot(last.1);
        if sample.value > tolerance || to_first < -tolerance || to_last < -tolerance {
            return Ok(false);
        }

        if sample.distance > tolerance && to_first.abs() <= tolerance && to_last.abs() <= tolerance
        {
            reaches_the_shared_crease = true;
        }
    }

    Ok(reaches_the_shared_crease)
}

/// C# `SwallowedSide.VirtualRun(mesh, chain, side)`: the swallowed side's run, made up off the
/// mesh: the rail - each chain point carried the setback away from the chain in the side's own
/// plane, square to the chain - between where it crosses the first end's wall plane and where it
/// crosses the last end's, in ascending chain parameter, with a point at every chain vertex
/// between. Parameters may stand past the chain's ends. `None` where there is no such run. On a
/// swallowed side the rail stands behind BOTH walls, so its crossings come in the opposite order
/// to the chain's ends; the run is the span between them whichever way round.
pub fn virtual_run(mesh: &LevelLineMesh, chain: &ChainField, side: i32) -> Option<VirtualRun> {
    virtual_run_over(mesh, chain, side, None)
}

/// C# `SwallowedSide.VirtualRun(mesh, chain, side, span)`: [`virtual_run`] over the given parameter
/// span (the rows' [`row_span`]) in place of the rail's own crossings of the wall planes, with
/// stations a quarter setback apart at most: the strip's rows stand only at the run's points, and
/// one cell spanning the whole run laid the round flat between its end rows, so the wall clip's
/// curve cut across it stood 0.19 off the cylinder (Fillet Thin Front at 4.9); four cells read
/// 0.015.
#[allow(clippy::needless_range_loop)]
pub fn virtual_run_over(
    mesh: &LevelLineMesh,
    chain: &ChainField,
    side: i32,
    span: Option<(f64, f64)>,
) -> Option<VirtualRun> {
    let (first, last) = growing_walls(mesh, chain)?;
    let points = &chain.points;
    let mut starts = vec![0.0; points.len()];
    for i in 1..points.len() {
        starts[i] = starts[i - 1] + (points[i] - points[i - 1]).length();
    }

    let mut aways = Vec::with_capacity(points.len() - 1);
    for i in 0..points.len() - 1 {
        aways.push(away_on_side(
            mesh,
            points[i],
            points[i + 1],
            side,
            tolerance(chain),
        )?);
    }

    // The rail on segment i, read on the segment's LINE so the end segments carry on past the
    // chain.
    let rail_on = |i: usize, t: f64| {
        let direction = (points[i + 1] - points[i]).get_normal();
        points[i]
            + (direction * (t - starts[i]))
            + (aways[i] * chain.setback_at(t.clamp(0.0, chain.length)))
    };

    let crossing = |wall: Wall, near_last: bool| {
        let mut best: Option<f64> = None;
        for i in 0..aways.len() {
            // The rail is read at the segment's two ends and the plane's root taken on the line
            // between them - exact for a setback uniform along the segment.
            let t0 = starts[i];
            let t1 = starts[i + 1];
            let f0 = (rail_on(i, t0) - wall.0).dot(wall.1);
            let f1 = (rail_on(i, t1) - wall.0).dot(wall.1);
            if f0 == f1 {
                continue;
            }

            let t = t0 + ((t1 - t0) * f0 / (f0 - f1));
            let within = (t >= t0 || i == 0) && (t <= t1 || i == aways.len() - 1);
            if within && best.is_none_or(|b| if near_last { t > b } else { t < b }) {
                best = Some(t);
            }
        }

        best
    };

    let a = crossing(first, false)?;
    let b = crossing(last, true)?;
    let from = span.map_or(cs_math::min_f64(a, b), |s| s.0);
    let to = span.map_or(cs_math::max_f64(a, b), |s| s.1);
    let segment_of = |t: f64| {
        let mut i = 0;
        while i < aways.len() - 1 && t > starts[i + 1] {
            i += 1;
        }

        i
    };

    let mut run = vec![(rail_on(segment_of(from), from), from)];
    for i in 1..points.len() - 1 {
        if starts[i] > from && starts[i] < to {
            // A bent chain's rail has a corner here; the incoming segment's side names it.
            run.push((rail_on(i - 1, starts[i]), starts[i]));
        }
    }

    if span.is_some() {
        // C#'s (int)Math.Ceiling: a small count of stations.
        let n = ((to - from) / (PAST_THE_LAST_TOUCH * chain.max_setback)).ceil() as i32;
        for k in 1..n {
            let t = from + ((to - from) * k as f64 / n as f64);
            run.push((rail_on(segment_of(t), t), t));
        }

        // C# sorts unstably by parameter; the parameters are distinct but for a corner standing on
        // a station, which this keeps in the order laid.
        run.sort_by(|p, q| p.1.total_cmp(&q.1));
    }

    run.push((rail_on(segment_of(to), to), to));
    Some(run)
}

/// C# `SwallowedSide.RowSpan`: the parameters the swallowed strip's rows must span: every row
/// that reaches the air in front of both walls, carried [`PAST_THE_LAST_TOUCH`] setbacks past the
/// last row to touch each wall, so the end rows stand wholly behind a wall. Read on a STRAIGHT
/// chain only, where each row is the last translated; `None` otherwise. Each wall's reading is
/// linear in the parameter (the row's greatest distance in front of the plane plus the chain
/// point's own), and its zero is where the row last touches that wall. Fillet Thin Front at 4.9:
/// the span is 2.09 .. 7.91 from v9.
pub fn row_span(
    mesh: &LevelLineMesh,
    chain: &ChainField,
    profile: BandProfile,
) -> Option<(f64, f64)> {
    let walls = growing_walls(mesh, chain)?;
    let points = &chain.points;
    let p0 = points[0];
    let direction = (points[points.len() - 1] - p0).get_normal();
    if points
        .iter()
        .any(|&p| ((p - p0) - (direction * (p - p0).dot(direction))).length() > tolerance(chain))
    {
        return None;
    }

    let end = points[points.len() - 1];
    let left = away_on_side(mesh, p0, end, 1, tolerance(chain))?;
    let right = away_on_side(mesh, p0, end, 2, tolerance(chain))?;

    // The rows at the chain's two ends, and each wall's greatest in-front distance over them.
    let in_front = |wall: Wall, t: f64| {
        let on_chain = p0 + (direction * t);
        let setback = chain.setback_at(t.clamp(0.0, chain.length));
        last_to_leave(
            on_chain + (left * setback),
            on_chain + (right * setback),
            on_chain,
            direction,
            -wall.1,
            profile,
        ) + (on_chain - wall.0).dot(wall.1)
    };

    let mut touches = Vec::with_capacity(2);
    for wall in [walls.0, walls.1] {
        let f0 = in_front(wall, 0.0);
        let f1 = in_front(wall, chain.length);
        if f0 == f1 {
            return None;
        }

        touches.push(chain.length * f0 / (f0 - f1));
    }

    // C#'s Enumerable.Min / Max over the two touches.
    let least = if touches[0].is_nan() || touches[1].is_nan() {
        f64::NAN
    } else {
        touches[0].min(touches[1])
    };
    let most = if touches[0].is_nan() || touches[1] > touches[0] {
        touches[1]
    } else {
        touches[0]
    };
    let margin = PAST_THE_LAST_TOUCH * chain.max_setback;
    Some((least - margin, most + margin))
}

/// C# `SwallowedSide.TiesMergeRoom`: the room of a swallowed side, read in place of its self
/// room: the size at which the round stops reaching the crease the two walls share - the two ties
/// on it merge, and past that the fin goes whole. Read on a straight fillet chain (the round a
/// cylinder about the rows' centres, radius setback * tan(half angle)); the crease line's distance
/// from the axis and the radius are both linear in the size, so the merge is where they are equal,
/// the first such size above `above`. `None` where this is not a swallowed side of that kind.
/// Fillet Thin Front: 5 / (sqrt 13 / 2 - 1) = 6.23.
pub fn ties_merge_room(
    mesh: &LevelLineMesh,
    field_at: &dyn Fn(f64) -> ChainField,
    side: i32,
    profile: BandProfile,
    above: f64,
) -> Result<Option<f64>, WorkError> {
    if profile != BandProfile::Fillet {
        return Ok(None);
    }

    let past = field_at(above * (1.0 + 1e-6));
    if !is_swallowed(mesh, &past, side)? {
        return Ok(None);
    }

    let unit = field_at(1.0);
    let walls = growing_walls(mesh, &unit);
    let points = &unit.points;
    let tolerance = tolerance(&past);
    let end = points[points.len() - 1];
    let left = away_on_side(mesh, points[0], end, 1, tolerance);
    let right = away_on_side(mesh, points[0], end, 2, tolerance);
    let (Some((first, last)), Some(left), Some(right)) = (walls, left, right) else {
        return Ok(None);
    };

    let crease_direction = first.1.cross(last.1);
    let chain_direction = (end - points[0]).get_normal();
    let mut across = chain_direction.cross(crease_direction);
    if across.length() < 1e-9 {
        return Ok(None);
    }

    across = across.get_normal();

    // A point on the shared crease: a vertex of the side on both wall planes, off the chain.
    let mut on_crease = None;
    for &vertex in &side_vertices(mesh, side) {
        let q = mesh.position(vertex);
        if past.evaluate(q)?.distance > tolerance
            && (q - first.0).dot(first.1).abs() <= tolerance
            && (q - last.0).dot(last.1).abs() <= tolerance
        {
            on_crease = Some(q);
            break;
        }
    }

    let Some(on_crease) = on_crease else {
        return Ok(None);
    };

    let setback = unit.setback_at(0.0);
    let half = left.dot(right).clamp(-1.0, 1.0).acos() / 2.0;
    let bisector = (left + right).get_normal();
    let centre_offset = bisector * (setback / half.cos());
    let radius = setback * half.tan();
    let a = (points[0] - on_crease).dot(across);
    let b = centre_offset.dot(across);
    let mut best: Option<f64> = None;
    for s in [-a / (b - radius), -a / (b + radius)] {
        if s.is_finite() && s > above && best.is_none_or(|best| s < best) {
            best = Some(s);
        }
    }

    Ok(best)
}

/// C# `SwallowedSide.RunsIfBothSwallowed`: both sides' made-up runs over the rows' [`row_span`]
/// when BOTH sides are swallowed - read on the cut's mesh before its triangles are removed, since
/// the side's plane is read off the side; `None` otherwise (a side with a run of its own goes the
/// ordinary way).
pub fn runs_if_both_swallowed(
    mesh: &LevelLineMesh,
    chain: &ChainField,
    profile: BandProfile,
) -> Result<Option<[VirtualRun; 2]>, WorkError> {
    if growing_walls(mesh, chain).is_none()
        || !is_swallowed(mesh, chain, 1)?
        || !is_swallowed(mesh, chain, 2)?
    {
        return Ok(None);
    }

    let Some(span) = row_span(mesh, chain, profile) else {
        return Ok(None);
    };
    let left = virtual_run_over(mesh, chain, 1, Some(span));
    let right = virtual_run_over(mesh, chain, 2, Some(span));
    Ok(match (left, right) {
        (Some(left), Some(right)) => Some([left, right]),
        _ => None,
    })
}

/// C# `SwallowedSide.GrowingWalls`: the two end walls, each as a point on it and its outward
/// (air-side) face normal, when the chain is open, both ends carry the end ray, neither is a
/// same-sense bounded end, both walls face BACK over the chain (they grow) and they are not
/// parallel (they meet at a crease); otherwise `None`.
pub fn growing_walls(mesh: &LevelLineMesh, chain: &ChainField) -> Option<(Wall, Wall)> {
    if chain.is_closed
        || !chain.end_rays.0
        || !chain.end_rays.1
        || chain.end_ray_bounds.0.is_some_and(|b| b.same_sense)
        || chain.end_ray_bounds.1.is_some_and(|b| b.same_sense)
    {
        return None;
    }

    let first = back_facing_wall(mesh, chain, false)?;
    let last = back_facing_wall(mesh, chain, true)?;
    if first.1.dot(last.1).abs() > 1.0 - 1e-9 {
        return None;
    }

    Some((first, last))
}

/// C# `SwallowedSide.BackFacingWall`: the end face at one chain end - a kept, untagged triangle at
/// the end vertex that is not square to the chain's direction there (the same reading the virtual
/// rail takes) - when its outward normal faces back over the chain.
pub fn back_facing_wall(mesh: &LevelLineMesh, chain: &ChainField, last: bool) -> Option<Wall> {
    let points = &chain.points;
    let n = points.len();
    let end = if last { points[n - 1] } else { points[0] };
    let outward = if last {
        points[n - 1] - points[n - 2]
    } else {
        points[0] - points[1]
    }
    .get_normal();
    for t in 0..mesh.triangle_count() {
        let tri = mesh.triangles()[t as usize];
        if mesh.is_removed(t) || tri.tag != 0 || !touches(mesh, tri, end) {
            continue;
        }

        let a = mesh.position(tri.a);
        let normal = (mesh.position(tri.b) - a)
            .cross(mesh.position(tri.c) - a)
            .get_normal();
        if normal.dot(outward).abs() > 1e-6 {
            return if normal.dot(outward) < 0.0 {
                Some((end, normal))
            } else {
                None
            };
        }
    }

    None
}
