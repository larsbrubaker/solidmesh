//! The fill across a chain's end face: its readings - a rail carried along the end plane to the
//! clip corner
//! ([`rails_to_the_clip_corner`]), whether both rails end past the chain's end
//! ([`runs_end_past_the_end`]), whether the end face meets the sides convexly
//! ([`meets_the_sides_convexly`]), whether a run is planar ([`is_planar`]) and an end face's one
//! run ([`run`]), a trimmed run with one end put back whole ([`with_whole_end`]), whether a gap
//! is concave ([`is_concave`]), a run without its end-face edges ([`without_end_face_edges`]) and
//! the crease stretch along the end face ([`crease_run`]). The fill itself (`TryEndFaces`,
//! `TryGrowingEndFaces`) is in `end_face_fill_try.rs`, kept apart for the file-size limit and
//! re-exported here.

use vector_math::{cs_math, Vector3};

pub use super::end_face_fill_try::{try_end_faces, try_growing_end_faces};
use super::open_gap_fill;
use super::{BandGapResult, ChainField, GapLoop, LevelLineMesh, LevelLineTriangle};

/// Where the chain's end ray is bounded by a same-sense end plane, moves the rail's stretch that
/// runs along that plane to the clip corner off the rail and onto the end run, as (rail, run); the
/// pair unchanged otherwise. The end run at end 1 closes on the rail's head, at end 0 it starts
/// from its tail.
pub fn rails_to_the_clip_corner(
    mesh: &LevelLineMesh,
    chain: &ChainField,
    rail: &GapLoop,
    run: &[i32],
    last: bool,
) -> (GapLoop, Vec<i32>) {
    let bound = if last {
        chain.end_ray_bounds.1
    } else {
        chain.end_ray_bounds.0
    };
    let bound = match bound {
        Some(bound) if bound.same_sense && bound.crease_room != f64::INFINITY => bound,
        _ => return (rail.clone(), run.to_vec()),
    };

    let points = chain.points();
    let end = if last {
        points[points.len() - 1]
    } else {
        points[0]
    };
    let normal = bound.plane_normal.get_normal();
    let on_the_plane = |v: i32| (mesh.position(v) - end).dot(normal).abs() <= 1e-5;

    let mut vertices = rail.vertices.clone();
    let mut parameters = rail.parameters.clone();
    if !last {
        vertices.reverse();
        parameters.reverse();
    }

    let mut corner = 0;
    while corner + 1 < vertices.len()
        && on_the_plane(vertices[corner])
        && on_the_plane(vertices[corner + 1])
    {
        corner += 1;
    }

    let run_end = if last { run[run.len() - 1] } else { run[0] };
    if corner == 0 || vertices.len() - corner < 2 || run_end != vertices[0] {
        return (rail.clone(), run.to_vec());
    }

    let stretch = &vertices[..=corner];
    let new_run: Vec<i32> = if last {
        run.iter().chain(&stretch[1..]).copied().collect()
    } else {
        stretch[1..].iter().rev().chain(run).copied().collect()
    };
    vertices.drain(0..corner);
    parameters.drain(0..corner);
    if !last {
        vertices.reverse();
        parameters.reverse();
    }

    (
        GapLoop::new(
            rail.side,
            rail.is_closed,
            vertices,
            parameters,
            rail.chain,
            false,
            None,
        ),
        new_run,
    )
}

/// True when both rails' ends at the chain's `last` (or first) end stand at or past the chain's
/// end point along its outward direction.
pub fn runs_end_past_the_end(
    mesh: &LevelLineMesh,
    chain: &ChainField,
    left: &GapLoop,
    right: &GapLoop,
    last: bool,
) -> bool {
    const SLACK: f64 = 1e-9;
    let points = chain.points();
    let n = points.len();
    let end = if last { points[n - 1] } else { points[0] };
    let outward = if last {
        points[n - 1] - points[n - 2]
    } else {
        points[0] - points[1]
    }
    .get_normal();
    let pl = mesh.position(if last {
        left.vertices[left.vertices.len() - 1]
    } else {
        left.vertices[0]
    });
    let pr = mesh.position(if last {
        right.vertices[0]
    } else {
        right.vertices[right.vertices.len() - 1]
    });
    (pl - end).dot(outward) >= -SLACK && (pr - end).dot(outward) >= -SLACK
}

/// True when some side triangle (tag 1 or 2) at `v` shares an edge with an end-face triangle
/// (`end_tag`) that folds convexly away from it.
pub fn meets_the_sides_convexly(mesh: &LevelLineMesh, end_tag: i32, v: Vector3) -> bool {
    let normal_of = |t: LevelLineTriangle| {
        let a = mesh.position(t.a);
        (mesh.position(t.b) - a).cross(mesh.position(t.c) - a)
    };

    // Read over every triangle that ever stood at V, removed or not: the cut has taken the side
    // triangles round V out by now, and the fold is the input's, which their planes still say.
    let at_v: Vec<LevelLineTriangle> = mesh
        .triangles()
        .iter()
        .copied()
        .filter(|t| mesh.position(t.a) == v || mesh.position(t.b) == v || mesh.position(t.c) == v)
        .collect();

    for side in at_v.iter().filter(|t| t.tag == 1 || t.tag == 2) {
        for end in at_v.iter().filter(|t| t.tag == end_tag) {
            let side_corners = [side.a, side.b, side.c];
            let end_corners = [end.a, end.b, end.c];
            // Distinct, in the first sequence's order.
            let mut shared: Vec<i32> = Vec::new();
            for c in side_corners {
                if end_corners.contains(&c) && !shared.contains(&c) {
                    shared.push(c);
                }
            }

            if shared.len() != 2 {
                continue;
            }

            let Some(far) = end_corners.into_iter().find(|c| !shared.contains(c)) else {
                continue;
            };
            let normal = normal_of(*side);
            if normal.length_squared() > 0.0
                && normal.dot(mesh.position(far) - mesh.position(shared[0])) < 0.0
            {
                return true;
            }
        }
    }

    false
}

/// True when a run of four or more vertices lies in one plane, to a billionth of its extent (at
/// least 1); a shorter or collinear run is planar.
pub fn is_planar(mesh: &LevelLineMesh, run: &[i32]) -> bool {
    if run.len() < 4 {
        return true;
    }

    let o = mesh.position(run[0]);
    // A NaN is displaced by any number.
    let extent = run
        .iter()
        .map(|&v| (mesh.position(v) - o).length())
        .fold(
            f64::NAN,
            |best, x| if x > best || best.is_nan() { x } else { best },
        );
    let mut best = Vector3::ZERO;
    for &i in run {
        for &j in run {
            let n = (mesh.position(i) - o).cross(mesh.position(j) - o);
            if n.length() > best.length() {
                best = n;
            }
        }
    }

    if best.length() == 0.0 {
        return true;
    }

    let normal = best.get_normal();
    run.iter()
        .all(|&v| (mesh.position(v) - o).dot(normal).abs() <= 1e-9 * cs_math::max_f64(1.0, extent))
}

/// The vertices of the gap's one open run on side `tag`, running `from` to `to`; `None` (noting the
/// site) where there is not exactly one such run.
pub fn run(gap: &BandGapResult, tag: i32, from: i32, to: i32) -> Option<Vec<i32>> {
    let runs: Vec<&GapLoop> = gap.loops.iter().filter(|l| l.side == tag).collect();
    if runs.len() != 1
        || runs[0].is_closed
        || runs[0].vertices[0] != from
        || runs[0].vertices[runs[0].vertices.len() - 1] != to
    {
        open_gap_fill::append_ends_site(" / EndFaceFill:516");
        return None;
    }

    Some(runs[0].vertices.clone())
}

/// `run` with its tail (or head) put back as `whole` has it, from
/// the run's last (or first) vertex on: the other end keeps whatever the trim made of it. The run
/// itself where that vertex is not on the whole run.
pub fn with_whole_end(run: &GapLoop, whole: &GapLoop, at_tail: bool) -> GapLoop {
    let key = if at_tail {
        run.vertices[run.vertices.len() - 1]
    } else {
        run.vertices[0]
    };
    let Some(at) = whole.vertices.iter().position(|&v| v == key) else {
        return run.clone();
    };

    fn joined<T: Clone>(mine: &[T], theirs: &[T], at: usize, at_tail: bool) -> Vec<T> {
        if at_tail {
            mine.iter()
                .chain(theirs.iter().skip(at + 1))
                .cloned()
                .collect()
        } else {
            theirs.iter().take(at).chain(mine.iter()).cloned().collect()
        }
    }

    let mut joined_loop = GapLoop::new(
        run.side,
        run.is_closed,
        joined(&run.vertices, &whole.vertices, at, at_tail),
        joined(&run.parameters, &whole.parameters, at, at_tail),
        run.chain,
        run.has_mixed_owners,
        Some(run.owners.clone()),
    );
    joined_loop.vertex_owners = match (&run.vertex_owners, &whole.vertex_owners) {
        (Some(mine), Some(theirs)) => Some(joined(mine, theirs, at, at_tail)),
        _ => None,
    };
    joined_loop.vertex_ties = match (&run.vertex_ties, &whole.vertex_ties) {
        (Some(mine), Some(theirs)) => Some(joined(mine, theirs, at, at_tail)),
        _ => None,
    };
    joined_loop
}

/// NaN sorts below every number.
fn cs_compare(a: f64, b: f64) -> std::cmp::Ordering {
    match (a.is_nan(), b.is_nan()) {
        (true, true) => std::cmp::Ordering::Equal,
        (true, false) => std::cmp::Ordering::Less,
        (false, true) => std::cmp::Ordering::Greater,
        _ => a.partial_cmp(&b).unwrap_or(std::cmp::Ordering::Equal),
    }
}

/// Whether the gap between `left` and `right` is concave - read at the
/// run's middle PARAMETER, on the rail edge whose span crosses it (else the nearest), on the first
/// triangle with area standing on that edge: concave where the vertex of `right` nearest that
/// parameter stands in front of it.
pub fn is_concave(mesh: &LevelLineMesh, left: &GapLoop, right: &GapLoop) -> bool {
    let parameters = &left.parameters;
    let at = (parameters[0] + parameters[parameters.len() - 1]) / 2.0;

    // OrderBy(..).First(): the first of the least.
    let mut nearest = 0;
    for i in 1..right.vertices.len() {
        if cs_compare(
            (right.parameters[i] - at).abs(),
            (right.parameters[nearest] - at).abs(),
        ) == std::cmp::Ordering::Less
        {
            nearest = i;
        }
    }
    let across = right.vertices[nearest];

    // OrderBy(spans it ? 0 : 1).ThenBy(distance of its middle): a stable sort.
    let mut edges: Vec<usize> = (0..left.vertices.len().saturating_sub(1)).collect();
    let spans = |i: usize| {
        let (p, q) = (parameters[i], parameters[i + 1]);
        cs_math::min_f64(p, q) <= at && at <= cs_math::max_f64(p, q)
    };
    edges.sort_by(|&i, &j| {
        let key_i = if spans(i) { 0 } else { 1 };
        let key_j = if spans(j) { 0 } else { 1 };
        key_i.cmp(&key_j).then_with(|| {
            cs_compare(
                ((parameters[i] + parameters[i + 1]) / 2.0 - at).abs(),
                ((parameters[j] + parameters[j + 1]) / 2.0 - at).abs(),
            )
        })
    });

    for i in edges {
        for &t in mesh.triangles_on(left.vertices[i], left.vertices[i + 1]) {
            let face = mesh.triangles()[t as usize];
            let a0 = mesh.position(face.a);
            let n = (mesh.position(face.b) - a0).cross(mesh.position(face.c) - a0);
            if n.length() > 0.0 {
                return n.dot(mesh.position(across) - mesh.position(left.vertices[i])) > 0.0;
            }
        }
    }

    false
}

/// `run` without the edges at either end that stand on the
/// end face alone (tag 0); `None` (noting the site) where nothing is left.
pub fn without_end_face_edges(mesh: &LevelLineMesh, run: &GapLoop) -> Option<GapLoop> {
    let on_the_end_face_alone = |a: i32, b: i32| {
        let standing = mesh.triangles_on(a, b);
        standing.len() == 1 && mesh.triangles()[standing[0] as usize].tag == 0
    };

    let mut from = 0;
    let mut to = run.vertices.len() as i32 - 1;
    while from < to
        && on_the_end_face_alone(run.vertices[from as usize], run.vertices[from as usize + 1])
    {
        from += 1;
    }

    while to > from
        && on_the_end_face_alone(run.vertices[to as usize - 1], run.vertices[to as usize])
    {
        to -= 1;
    }

    if to - from < 1 {
        open_gap_fill::append_ends_site(" / EndFaceFill:435");
        return None;
    }

    if from == 0 && to == run.vertices.len() as i32 - 1 {
        return Some(run.clone());
    }

    let (from, count) = (from as usize, (to - from + 1) as usize);
    let mut trimmed = GapLoop::new(
        run.side,
        run.is_closed,
        run.vertices[from..from + count].to_vec(),
        run.parameters[from..from + count].to_vec(),
        run.chain,
        run.has_mixed_owners,
        Some(run.owners.clone()),
    );
    trimmed.vertex_owners = run
        .vertex_owners
        .as_ref()
        .map(|owners| owners.iter().skip(from).take(count).copied().collect());
    trimmed.vertex_ties = run
        .vertex_ties
        .as_ref()
        .map(|ties| ties.iter().skip(from).take(count).cloned().collect());
    Some(trimmed)
}

/// The boundary stretch from `from` to `to` along edges standing on
/// the end face alone (tag 0), which must pass through `end_vertex`; `None` (noting the site)
/// where it branches, dead-ends, loops or misses the end vertex.
pub fn crease_run(
    mesh: &LevelLineMesh,
    from: i32,
    to: i32,
    end_vertex: Vector3,
) -> Option<Vec<i32>> {
    let mut run = vec![from];
    let mut previous = -1;
    let mut current = from;
    let edges = mesh.edges();
    while current != to {
        let mut next = -1;
        for &(low, high) in &edges {
            if low != current && high != current {
                continue;
            }

            let other = if low == current { high } else { low };
            let standing = mesh.triangles_on(low, high);
            if other == previous
                || standing.len() != 1
                || mesh.triangles()[standing[0] as usize].tag != 0
            {
                continue;
            }

            if next >= 0 {
                // Two ways on: the boundary branches, and no single stretch is named.
                open_gap_fill::append_ends_site(" / EndFaceFill:488");
                return None;
            }

            next = other;
        }

        if next < 0 || run.contains(&next) {
            open_gap_fill::append_ends_site(" / EndFaceFill:496");
            return None;
        }

        run.push(next);
        previous = current;
        current = next;
    }

    let away = |v: i32| (mesh.position(v) - end_vertex).length();
    if !run.iter().any(|&v| away(v) <= mesh.same_place()) {
        // The least, with NaN reading as the smallest.
        let mut least = away(run[0]);
        for &v in &run[1..] {
            let x = away(v);
            if x < least || x.is_nan() {
                least = x;
            }
        }
        open_gap_fill::append_ends_site(&format!(
            " / EndFaceFill:511 run of {} misses the end vertex by {}",
            run.len(),
            vector_math::number_format::format_f64_max_decimals(least, 3)
        ));
        return None;
    }

    Some(run)
}
