//! An end face's last row on a LEANING end face: the round's own section in that face's plane.
//!
//! An open fill whose ends are closed by end-face caps lays a last row at each end, from the left
//! rail's end to the right rail's, both on the end face; the cap between that row and the end run
//! the cut left is a flat piece of the end face (`corner_cap_zip_end`, closed flat by
//! `corner_cap_flat_ends::closes_flat`), so the row must lie in the face's plane. On a square end
//! the row is the round's square section, drawn by `open_gap_fill::arcs` as every other row is. On
//! a leaning end face the round's section is that square section carried along the chain into the
//! face: the round is swept along the chain, so near its end every point of the square section
//! slides along the chain's direction until it meets the end face. That slide is an affine map
//! from the square plane onto the end face, and a rational quadratic carried by an affine map is
//! the rational quadratic of its carried control points with the same weight, so the oblique row
//! is the square row's control point slid onto the face with the square row's weight - exactly the
//! section, never a row drawn in the oblique plane by that plane's own angle (which bows off the
//! round: 0.11 inside the round's cylinder on the groove leaning 8 degrees).
//!
//! The square plane is the one through the end vertex, and the square row is the level line's,
//! drawn about the end vertex. Before the slide, both rail ends are slid along the chain into that
//! square plane; the slide takes them back to themselves.
//!
//! Both the end vertex and the slide hold only on a FLAT end face ([`flat_end_faces`]): its end
//! run and the chain's end vertex in one plane within the mesh's snap, the same snap the flat cap
//! closes the hole within, or, at a STOPPED end, every run vertex on its end plane by
//! `ChainField::on_a_stopped_plane`, the rule its band was cut by. On a curved end face (the D prism's rounded end, faceted round its
//! axis, whose rail on the top face ends on a facet 1.6 off the plane of the facet at the end
//! vertex) the row has no one face to lie in, and it is drawn as every other row is.
//!
//! A face within [`SQUARE_COSINE`] of square to the chain keeps the square row bit for bit, so no
//! square end moves by the projection's rounding.
//!
//! At a STOPPED end whose side folds across the band (a faceted twisted surface), the end plane's
//! trace on that side bends, so the row's tangents at the rail corners no longer meet at the end
//! vertex: [`stopped_end_controls`] gives the point where the traces at the two rail corners meet,
//! and the row is drawn about it (slid as above), so the flat cap between row and trace never
//! crosses itself. A side that is one plane keeps the end vertex.

use vector_math::{ieee_math, Vector3};

use super::gap_fill::arc_point;
use super::{ChainField, LevelLineMesh};

/// How near the end face's normal may come to the chain's direction (the cosine between them)
/// before the face is read as square to the chain and its row is left to the square row's drawing:
/// within about 4.5e-5 radians of square.
pub const SQUARE_COSINE: f64 = 1.0 - 1e-9;

/// How far the end face may turn from square before its row is not read as the round's section:
/// a face whose normal stands within this cosine of square to the chain (the face nearly along the
/// chain) has no section the slide can reach.
const ALONG_THE_CHAIN_COSINE: f64 = 1e-6;

/// What the section row of one end asks: the end it closes and the chain's unit direction there.
#[derive(Clone, Copy, Debug)]
pub struct EndRowAsked {
    /// Whether the row closes the chain's far end (parameter `chain.length`) rather than its start.
    pub at_the_far_end: bool,
    /// The chain's unit direction at that end.
    pub tangent: Vector3,
}

/// The row from `p` to `q` (the two rails' ends on a leaning end face) as the round's section in
/// that face: a point for each share `f` from 0 at `p` to 1 at `q`. `None` where the face is
/// square to the chain within [`SQUARE_COSINE`], where it runs along the chain, or where `p`, `q`
/// and the end vertex span no plane; the caller then draws the row as it draws every other.
pub fn section_row(
    chain: &ChainField,
    p: Vector3,
    q: Vector3,
    asked: EndRowAsked,
    control: Option<Vector3>,
) -> Option<impl Fn(f64) -> Vector3> {
    let end = if asked.at_the_far_end {
        chain.length
    } else {
        0.0
    };
    let v = chain.point_at(end);
    let t = asked.tangent;
    let normal = (p - v).cross(q - v);
    let length = normal.length();
    // Zero, or not a number: no plane.
    if length.is_nan() || length <= 0.0 || t.length_squared() == 0.0 {
        return None;
    }
    let n = normal * (1.0 / length);
    let lean = t.dot(n);
    if lean.abs() >= SQUARE_COSINE || lean.abs() <= ALONG_THE_CHAIN_COSINE {
        return None;
    }

    // The square plane through the end vertex; e is the chain's line where it crosses that plane.
    // Read as v plus the offset along t rather than v itself, so every bit stays as it was (a -0.0
    // component of v comes out +0.0 this way).
    let reference = v;
    let e = v + (t * (reference - v).dot(t));
    let into_the_square_plane = |x: Vector3| x - (t * (x - e).dot(t));
    let (p_square, q_square) = (into_the_square_plane(p), into_the_square_plane(q));
    // Where the sides fold, the row's tangents meet at `control` in the face, not at the end
    // vertex ([`stopped_end_controls`]); the slide is affine, so it carries that point too.
    let c_square = control.map_or(e, into_the_square_plane);

    // Each point of the square row slides along the chain onto the end face.
    let onto_the_face = move |x: Vector3| x + (t * ((v - x).dot(n) / lean));
    Some(move |f: f64| onto_the_face(arc_point(p_square, c_square, q_square, f)))
}

/// Which of the chain's two ends close on a FLAT end face: the end run there (from one rail's end
/// to the other's) and the chain's end vertex lie in one plane, every run vertex within the mesh's
/// `same_place` of the plane through the run's two ends and the end vertex. That snap is the one
/// the flat cap (`corner_cap_flat_ends::closes_flat`) closes the hole within, so an end read flat
/// here is one the cap can close in its plane. A fade (no end runs), a run of fewer than two
/// vertices, or ends and vertex spanning no plane is not flat. A STOPPED end
/// (`stopped_end_clip.rs`) is flat where every run vertex stands on its end plane by
/// `ChainField::on_a_stopped_plane`, the rule its band was cut by.
pub fn flat_end_faces(
    mesh: &LevelLineMesh,
    chain: &ChainField,
    end_runs: Option<&[Vec<i32>]>,
) -> [bool; 2] {
    let Some(end_runs) = end_runs else {
        return [false, false];
    };
    let snap = mesh.same_place();
    [0, 1].map(|e| {
        let Some(run) = end_runs.get(e) else {
            return false;
        };
        if run.len() < 2 {
            return false;
        }
        let v = chain.point_at(if e == 0 { 0.0 } else { chain.length });
        // A STOPPED end's run lies in its own end plane by the rule the band cut reads it by.
        let bound = if e == 0 {
            chain.end_ray_bounds.0
        } else {
            chain.end_ray_bounds.1
        };
        if let Some(bound) = bound.filter(|bound| bound.stopped) {
            let normal = bound.plane_normal.get_normal();
            return run.iter().all(|&i| {
                ChainField::on_a_stopped_plane(v, normal, bound.plane_tilt, mesh.position(i))
            });
        }
        let (p, q) = (mesh.position(run[0]), mesh.position(run[run.len() - 1]));
        let normal = (p - v).cross(q - v);
        let length = normal.length();
        // Zero, or not a number: no plane.
        if length.is_nan() || length <= 0.0 {
            return false;
        }
        let n = normal * (1.0 / length);
        run.iter()
            .all(|&i| (mesh.position(i) - v).dot(n).abs() <= snap)
    })
}

/// How far from a rail corner, against the row's span, the end run's next vertex must stand
/// before its direction is read as the side's trace there: nearer, float noise in its position
/// would turn the tangent.
const TRACE_DIRECTION_LEAST_SHARE: f64 = 1e-6;

/// The control point of each STOPPED end's last row where a side folds across the band: the
/// point in the end plane where the two sides' traces at the rail corners, carried on as straight
/// lines, meet. The round rests tangent on the face each rail lies in, so its section in the end
/// plane leaves each rail corner along that face's trace (the end run's first and last edges),
/// and where a side is one plane both traces run straight to the end vertex and the row is drawn
/// about it as before (`None`). Across a folded side (a faceted twisted surface) the trace bends,
/// and a row drawn about the end vertex leaves the rail across the trace, so the cap's hole
/// between that row and the end run crossed itself. `None` too where the traces do not meet
/// ahead of both rail corners.
pub fn stopped_end_controls(
    mesh: &LevelLineMesh,
    chain: &ChainField,
    end_runs: Option<&[Vec<i32>]>,
) -> [Option<Vector3>; 2] {
    let Some(end_runs) = end_runs else {
        return [None, None];
    };
    let snap = mesh.same_place();
    [0, 1].map(|e| {
        let bound = if e == 0 {
            chain.end_ray_bounds.0
        } else {
            chain.end_ray_bounds.1
        };
        if !bound.is_some_and(|bound| bound.stopped) {
            return None;
        }
        let run = end_runs.get(e)?;
        if run.len() < 3 {
            return None;
        }
        let v = chain.point_at(if e == 0 { 0.0 } else { chain.length });
        let p = mesh.position(run[0]);
        let q = mesh.position(run[run.len() - 1]);
        let least = TRACE_DIRECTION_LEAST_SHARE * (q - p).length();
        let leaving = |from: Vector3, run: &mut dyn Iterator<Item = &i32>| {
            run.map(|&i| mesh.position(i) - from)
                .find(|d| d.length() > least)
                .map(|d| d.get_normal())
        };
        let along_p = leaving(p, &mut run.iter().skip(1))?;
        let along_q = leaving(q, &mut run.iter().rev().skip(1))?;
        where_the_traces_meet(v, (p, along_p), (q, along_q), snap)
    })
}

/// Where the traces leaving the rail corners `p` and `q` (each with its unit direction toward the
/// end vertex's side), carried on as straight lines, meet: the stopped end's row control
/// ([`stopped_end_controls`]). `None` where both traces run straight through the end vertex `v`
/// within `snap` (the row stays about `v`), where they do not meet ahead of both corners, or where
/// they meet farther from `v` than the farther rail corner. Each trace runs from `v` to its rail
/// corner, so the round's tangent there turns from the line to `v` by no more than the trace bends
/// on the way, and the true section's control stays within that reach of `v`; a meeting point
/// past it is two nearly parallel traces (a fold just past a rail corner, or a trace edge read
/// along a fold), whose far crossing would bow the row out of the section.
pub fn where_the_traces_meet(
    v: Vector3,
    (p, along_p): (Vector3, Vector3),
    (q, along_q): (Vector3, Vector3),
    snap: f64,
) -> Option<Vector3> {
    let off_the_line = |from: Vector3, along: Vector3| {
        let d = v - from;
        (d - (along * d.dot(along))).length()
    };
    if off_the_line(p, along_p) <= snap && off_the_line(q, along_q) <= snap {
        return None;
    }

    // The nearest points of the two lines p + s along_p and q + t along_q (one point where they
    // meet, as two traces in one plane do).
    let w = p - q;
    let b = along_p.dot(along_q);
    let (d, f) = (along_p.dot(w), along_q.dot(w));
    let denominator = 1.0 - (b * b);
    if denominator.is_nan() || denominator <= 0.0 {
        return None;
    }
    let s = ((b * f) - d) / denominator;
    let t = (f - (b * d)) / denominator;
    if s.is_nan() || t.is_nan() || s <= 0.0 || t <= 0.0 {
        return None;
    }
    let meeting = ((p + (along_p * s)) + (q + (along_q * t))) * 0.5;
    let reach = ieee_math::max_f64((p - v).length(), (q - v).length());
    // Not a number reads as past the reach.
    #[allow(clippy::neg_cmp_op_on_partial_ord)]
    if !((meeting - v).length() <= reach) {
        return None;
    }
    Some(meeting)
}
