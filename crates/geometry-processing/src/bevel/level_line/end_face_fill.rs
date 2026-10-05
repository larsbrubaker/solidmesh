//! The fill across a chain's end face. Ports `Bevel/LevelLine/EndFaceFill.cs` (a C# static class):
//! so far its readings - a rail carried along the end plane to the clip corner
//! ([`rails_to_the_clip_corner`]), whether both rails end past the chain's end
//! ([`runs_end_past_the_end`]), whether the end face meets the sides convexly
//! ([`meets_the_sides_convexly`]), whether a run is planar ([`is_planar`]) and an end face's one
//! run ([`run`]); the fill itself lands with later packages.

use vector_math::{cs_math, Vector3};

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
            // C# `Intersect`: distinct, in the first sequence's order.
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
    // C#'s `Enumerable.Max`: a NaN is displaced by any number.
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
